/**
 * @file   app_skaijs.c
 * @brief  Host screen for sandboxed JS apps (ADR-0019 Phase 3).
 *
 * A JS app IS a GUI app. That is not a detail — skai_ui refuses calls from any
 * thread but LVGL's, so the script has to run where the widgets live. The app
 * framework's mailbox already delivers on_start on the LVGL thread, so hosting
 * the interpreter here is what makes drawing legal at all.
 *
 * The host owns the container; skai_ui only borrows it for the run. Tearing the
 * container down invalidates every widget id, so a stale id from a previous run
 * cannot address anything in the next one.
 */
#include <rtthread.h>
#include <string.h>

/* ui_handler.h first: it is what defines APP_ID_SKAIJS (only when QuickJS is
 * built), so guarding on that symbol before including it would silently
 * compile the whole file away. */
#include "ui_handler.h"

#ifdef APP_ID_SKAIJS

#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "ui_img_helper.h"

#include "skai/skai_js.h"
#include "skai/skai_display.h"
#include "skai/skai_ui.h"
#include "skai/skai_alarm.h"

#include "communicate_task.h"

#define DBG_TAG "app.skaijs"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* One definition, shared with the installer: skai_pkg refuses a package whose
   payload would not fit here, rather than letting it truncate on the way in. */
#include "skai/skai_pkg.h"
#define SKAIJS_SRC_MAX SKAI_PKG_JS_SRC_MAX

static lv_obj_t *s_root;
/* Polls for fired background alarms (skai_alarm.c fires on its own thread; a JS
   handler may only run on this one), turning them into on_change("alarm"). */
static lv_timer_t *s_alarm_poll;

static void alarm_poll_cb(lv_timer_t *t)
{
    (void)t;
    if (skai_alarm_take_fired_event())
        skai_js_notify_change("alarm");
}
/* Borrowed, not copied: the launcher's buffer (PSRAM, skai_pkg.c) or a sim
   sample's static string, both of which outlive the run. A second 48 KB copy
   in SRAM is exactly what moving the buffer to PSRAM was meant to avoid. */
static const char *s_src;
static char      s_asset_dir[64];
static skai_js_policy_t s_policy;

/* ── reports to the phone ──
 * Every app-log line (console.log, an exception and its stack, a watchdog or
 * quota stop) goes up on 0x29 so whoever wrote the app can see it. Bounded: an
 * app logging from a 50 ms interval would otherwise own the BLE link. */
#define SKAIJS_LOG_PER_SEC 8
static rt_tick_t s_log_window;
static uint16_t  s_log_in_window;
static uint32_t  s_log_dropped;

static void log_sink(const char *keyid, const char *line)
{
    (void)keyid;
    rt_tick_t now = rt_tick_get();

    /* Also on the console: the only way to read an app's log on a watch with
       no phone attached, and on the simulator. */
    rt_kprintf("[js %s] %s\n", s_policy.app_id, line);

    if (now - s_log_window >= RT_TICK_PER_SECOND)
    {
        if (s_log_dropped > 0)
        {
            char note[48];
            rt_snprintf(note, sizeof(note), "(%u log lines dropped)", (unsigned)s_log_dropped);
            commu_send_skaiapp_run(s_policy.app_id, "log", NULL, note);
            s_log_dropped = 0;
        }
        s_log_window = now;
        s_log_in_window = 0;
    }
    if (s_log_in_window >= SKAIJS_LOG_PER_SEC)
    {
        s_log_dropped++;
        return;
    }
    s_log_in_window++;
    commu_send_skaiapp_run(s_policy.app_id, "log", NULL, line);
}

/* Loaded before launch (by the phone push path, or skaijs_run on the sim).
 * Capabilities come from the package manifest; the caller owns that array and
 * must keep it alive, which the sim samples do by making it static. */
void skaijs_set_source(const char *src, const skai_js_policy_t *policy)
{
    if (!src || !policy)
        return;
    s_src = src;
    s_policy = *policy;
}

static void on_start(void)
{
    skai_js_result_t r;

    s_root = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_root);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_COVER, 0);
    /* Plain full-screen container. skai_ui puts its own padded flex column
     * inside it and keeps this one for explicitly aligned widgets, so the two
     * placement models do not fight. */
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    /* Assets live beside the installed package, so ui.image resolves inside
     * this app's directory and nowhere else. */
    rt_snprintf(s_asset_dir, sizeof(s_asset_dir), "/skaiapp/%s/%s",
                s_policy.keyid, s_policy.app_id);
    skai_ui_attach(s_root, s_asset_dir);

    if (s_src == NULL || s_src[0] == '\0')
    {
        LOG_W("no script loaded");
        return;
    }

    s_log_window = rt_tick_get();
    s_log_in_window = 0;
    s_log_dropped = 0;
    skai_js_set_log_sink(log_sink);
    s_alarm_poll = lv_timer_create(alarm_poll_cb, 500, NULL);

    /* Context stays open for the app's lifetime: click handlers are JS
     * closures, so freeing the runtime after the first eval would leave every
     * button pointing at nothing (ADR-0019 decision 13). */
    if (skai_js_open(&s_policy) != SKAI_JS_OK)
    {
        LOG_E("could not start the JS runtime");
        commu_send_skaiapp_run(s_policy.app_id, "start", "internal", NULL);
        return;
    }

    r = skai_js_eval(s_src, (uint32_t)strlen(s_src), &s_policy);
    commu_send_skaiapp_run(s_policy.app_id, "start", skai_js_result_name(r), NULL);
    if (r != SKAI_JS_OK)
    {
        /* The app stays on screen showing whatever it managed to draw. A
         * sandbox stop is not a crash — the reason already went to the
         * app-scoped log for the developer. */
        LOG_W("script stopped: %s", skai_js_result_name(r));
    }
}

static void on_stop(void)
{
    /* Whatever the script did to the screen stops when the script does. The app
       is never asked to put the brightness back, so it cannot fail to. */
    skai_display_restore();
    if (s_alarm_poll)
    {
        lv_timer_del(s_alarm_poll);
        s_alarm_poll = NULL;
    }
    skai_js_close();
    skai_js_set_log_sink(NULL);
    if (s_policy.app_id[0] != '\0')
        commu_send_skaiapp_run(s_policy.app_id, "stop", NULL, NULL);
    skai_ui_detach();
    if (s_root != NULL)
    {
        lv_obj_del(s_root); /* takes every widget the script created with it */
        s_root = NULL;
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    (void)param;
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
        break;
    case GUI_APP_MSG_ONRESUME:
        /* The C apps set brightness on EVERY resume; a JS body runs once. The
           host re-applies what the app asked for, so a backgrounded app does
           not come back dark with no way to fix it. */
        skai_display_reapply();
        break;
    case GUI_APP_MSG_ONPAUSE:
        /* Backgrounded, not closed: the context stays open so click handlers
           survive, but the brightness is the user's again. */
        skai_display_restore();
        break;
    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

static int app_main(intent_t i)
{
    (void)i;
    gui_app_regist_msg_handler(APP_ID_SKAIJS, msg_handler);
    return 0;
}

/* Borrows the "skaiapp" launcher label. This host has no user-facing identity
 * yet — a JS app is launched by its own name, not by opening a generic host —
 * so adding an .arb string just to fill the launcher would be a resource entry
 * with no reader. Give it its own string when JS apps get launcher tiles. */
BUILTIN_APP_EXPORT(LV_EXT_STR_ID(skaiapp), IMG_LOGO, APP_ID_SKAIJS, app_main, 1);

#endif /* APP_ID_SKAIJS */
