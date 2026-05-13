/**
 ******************************************************************************
 * @file   app_sleep.c
 * @author Skaiwalk software development team
 * @brief  Sleep summary LVGL app. Polls SkaiWatchSys.sleep_state (populated
 *         by the LCPU sleep_fusion classifier via watch_sys_sync RPC) at
 *         1 Hz and renders today's stage totals + current stage banner.
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include <rtthread.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "watch_global_data.h"
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
    #include "ui_img_helper.h"
#endif

#define DBG_TAG "app.sleep"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_SLEEP

/* Wire-format mode values, mirrored from sleep_service.c / health_algo.h.
   Kept local rather than including the LCPU header — HCPU shouldn't be
   pulling in the algorithm side. */
#define WIRE_TSLEEP        0x01
#define WIRE_TDEEP_SLEEP   0x02
#define WIRE_TGET_UP       0x03
#define WIRE_TNOT_WEAR     0x04
#define WIRE_TREM_SLEEP    0x05

typedef struct
{
    lv_obj_t *root;
    lv_obj_t *title_label;
    lv_obj_t *total_label;        /* big "Hh Mm" total sleep number */
    lv_obj_t *total_caption;      /* "Total sleep" subtitle */
    lv_obj_t *deep_label;
    lv_obj_t *rem_label;
    lv_obj_t *light_label;
    lv_obj_t *waso_label;
    lv_obj_t *stage_banner;       /* "Currently: Deep" pill at the bottom */
    lv_obj_t *hr_label;           /* "60 bpm" small text near banner */
    lv_timer_t *refresh_timer;
} app_sleep_t;

static app_sleep_t *p_app_sleep = NULL;

static const char *prv_stage_name(uint8_t mode)
{
    switch (mode)
    {
    case WIRE_TSLEEP:       return "Light";
    case WIRE_TDEEP_SLEEP:  return "Deep";
    case WIRE_TGET_UP:      return "Awake";
    case WIRE_TNOT_WEAR:    return "Not worn";
    case WIRE_TREM_SLEEP:   return "REM";
    default:                return "—";
    }
}

static lv_color_t prv_stage_color(uint8_t mode)
{
    switch (mode)
    {
    case WIRE_TSLEEP:       return lv_color_hex(0x4F8DFF); /* light blue */
    case WIRE_TDEEP_SLEEP:  return lv_color_hex(0x1B3FA0); /* deep blue */
    case WIRE_TREM_SLEEP:   return lv_color_hex(0xB94DFF); /* purple */
    case WIRE_TGET_UP:      return lv_color_hex(0xFFA500); /* orange */
    case WIRE_TNOT_WEAR:    return lv_color_hex(0x666666); /* grey */
    default:                return lv_color_hex(0x333333);
    }
}

static void prv_update_labels(void)
{
    if (!p_app_sleep || !lv_obj_is_valid(p_app_sleep->root))
    {
        return;
    }

    /* SkaiWatchSys is packed — copy the substruct to an aligned local
       instead of holding a pointer into the packed parent. */
    watch_sys_sleep_state_t s;
    memcpy(&s, (const void *)&SkaiWatchSys.sleep_state, sizeof(s));
    char buf[32];

    uint16_t total = s.total_sleep_min;
    snprintf(buf, sizeof(buf), "%uh %02um", total / 60u, total % 60u);
    lv_label_set_text(p_app_sleep->total_label, buf);

    snprintf(buf, sizeof(buf), "Deep   %u min", (unsigned)s.deep_min);
    lv_label_set_text(p_app_sleep->deep_label, buf);

    snprintf(buf, sizeof(buf), "REM    %u min", (unsigned)s.rem_min);
    lv_label_set_text(p_app_sleep->rem_label, buf);

    snprintf(buf, sizeof(buf), "Light  %u min", (unsigned)s.light_min);
    lv_label_set_text(p_app_sleep->light_label, buf);

    snprintf(buf, sizeof(buf), "WASO   %u min", (unsigned)s.awake_after_onset_min);
    lv_label_set_text(p_app_sleep->waso_label, buf);

    snprintf(buf, sizeof(buf), "Now: %s", prv_stage_name(s.mode));
    lv_label_set_text(p_app_sleep->stage_banner, buf);
    lv_obj_set_style_bg_color(p_app_sleep->stage_banner,
                              prv_stage_color(s.mode), 0);

    if (s.current_hr > 0)
    {
        snprintf(buf, sizeof(buf), "HR %u  (rest %u)",
                 (unsigned)s.current_hr, (unsigned)s.resting_hr);
    }
    else
    {
        snprintf(buf, sizeof(buf), "HR --  (rest %u)", (unsigned)s.resting_hr);
    }
    lv_label_set_text(p_app_sleep->hr_label, buf);
}

static void prv_refresh_cb(lv_timer_t *t)
{
    (void)t;
    prv_update_labels();
}

static void prv_on_screen_touch(lv_event_t *e)
{
    (void)e;
    gui_app_exit(APP_ID_SLEEP);
}

static void prv_build_ui(lv_obj_t *parent)
{
    /* Black backdrop. */
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(parent, prv_on_screen_touch, LV_EVENT_CLICKED, NULL);

    /* Title. */
    p_app_sleep->title_label = lv_label_create(parent);
    lv_obj_set_style_text_font(p_app_sleep->title_label,
                               LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(p_app_sleep->title_label,
                                lv_color_hex(0xCCCCCC), 0);
    lv_label_set_text(p_app_sleep->title_label, "Sleep");
    lv_obj_align(p_app_sleep->title_label, LV_ALIGN_TOP_MID, 0, 60);

    /* Big total. */
    p_app_sleep->total_label = lv_label_create(parent);
    lv_obj_set_style_text_font(p_app_sleep->total_label,
                               LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_set_style_text_color(p_app_sleep->total_label, lv_color_white(), 0);
    lv_label_set_text(p_app_sleep->total_label, "0h 00m");
    lv_obj_align(p_app_sleep->total_label, LV_ALIGN_TOP_MID, 0, 95);

    /* Subtitle for total. */
    p_app_sleep->total_caption = lv_label_create(parent);
    lv_obj_set_style_text_color(p_app_sleep->total_caption,
                                lv_color_hex(0x999999), 0);
    lv_label_set_text(p_app_sleep->total_caption, "Total sleep today");
    lv_obj_align(p_app_sleep->total_caption, LV_ALIGN_TOP_MID, 0, 160);

    /* Stage breakdown. */
    p_app_sleep->deep_label = lv_label_create(parent);
    lv_obj_set_style_text_color(p_app_sleep->deep_label,
                                lv_color_hex(0x1B3FA0), 0);
    lv_label_set_text(p_app_sleep->deep_label, "Deep   0 min");
    lv_obj_align(p_app_sleep->deep_label, LV_ALIGN_TOP_MID, 0, 195);

    p_app_sleep->rem_label = lv_label_create(parent);
    lv_obj_set_style_text_color(p_app_sleep->rem_label,
                                lv_color_hex(0xB94DFF), 0);
    lv_label_set_text(p_app_sleep->rem_label, "REM    0 min");
    lv_obj_align(p_app_sleep->rem_label, LV_ALIGN_TOP_MID, 0, 225);

    p_app_sleep->light_label = lv_label_create(parent);
    lv_obj_set_style_text_color(p_app_sleep->light_label,
                                lv_color_hex(0x4F8DFF), 0);
    lv_label_set_text(p_app_sleep->light_label, "Light  0 min");
    lv_obj_align(p_app_sleep->light_label, LV_ALIGN_TOP_MID, 0, 255);

    p_app_sleep->waso_label = lv_label_create(parent);
    lv_obj_set_style_text_color(p_app_sleep->waso_label,
                                lv_color_hex(0x999999), 0);
    lv_label_set_text(p_app_sleep->waso_label, "WASO   0 min");
    lv_obj_align(p_app_sleep->waso_label, LV_ALIGN_TOP_MID, 0, 285);

    /* Current-stage banner. Inverted: dark pill, coloured background. */
    p_app_sleep->stage_banner = lv_label_create(parent);
    lv_obj_set_style_pad_hor(p_app_sleep->stage_banner, 16, 0);
    lv_obj_set_style_pad_ver(p_app_sleep->stage_banner, 8, 0);
    lv_obj_set_style_radius(p_app_sleep->stage_banner, 20, 0);
    lv_obj_set_style_bg_opa(p_app_sleep->stage_banner, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(p_app_sleep->stage_banner,
                              lv_color_hex(0x333333), 0);
    lv_obj_set_style_text_color(p_app_sleep->stage_banner,
                                lv_color_white(), 0);
    lv_label_set_text(p_app_sleep->stage_banner, "Now: —");
    lv_obj_align(p_app_sleep->stage_banner, LV_ALIGN_BOTTOM_MID, 0, -90);

    /* HR row under the banner. */
    p_app_sleep->hr_label = lv_label_create(parent);
    lv_obj_set_style_text_color(p_app_sleep->hr_label,
                                lv_color_hex(0x999999), 0);
    lv_label_set_text(p_app_sleep->hr_label, "HR --  (rest --)");
    lv_obj_align(p_app_sleep->hr_label, LV_ALIGN_BOTTOM_MID, 0, -55);
}

static void on_start(void)
{
    p_app_sleep = rt_malloc(sizeof(app_sleep_t));
    if (!p_app_sleep)
    {
        LOG_E("app_sleep: out of memory");
        return;
    }
    memset(p_app_sleep, 0, sizeof(app_sleep_t));
    p_app_sleep->root = lv_scr_act();
    prv_build_ui(p_app_sleep->root);
    prv_update_labels();
}

static void on_resume(void)
{
    if (!p_app_sleep) return;
    /* 1 Hz polling — sleep_state only changes on stage transitions (a few
       times per night) so polling cost is negligible vs. wiring up a
       handler. Kept here so we also pick up daily-aggregate increments
       between transitions for the WASO counter. */
    if (!p_app_sleep->refresh_timer)
    {
        p_app_sleep->refresh_timer = lv_timer_create(prv_refresh_cb, 1000, NULL);
    }
}

static void on_pause(void)
{
    if (p_app_sleep && p_app_sleep->refresh_timer)
    {
        lv_timer_del(p_app_sleep->refresh_timer);
        p_app_sleep->refresh_timer = NULL;
    }
}

static void on_stop(void)
{
    if (!p_app_sleep) return;
    if (p_app_sleep->refresh_timer)
    {
        lv_timer_del(p_app_sleep->refresh_timer);
        p_app_sleep->refresh_timer = NULL;
    }
    /* lv_obj_clean on the root is handled by gui_app_fwk when the screen
       is torn down; we just free our own ctx struct. */
    rt_free(p_app_sleep);
    p_app_sleep = NULL;
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    (void)param;
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:   on_start();   break;
    case GUI_APP_MSG_ONRESUME:  on_resume();  break;
    case GUI_APP_MSG_ONPAUSE:   on_pause();   break;
    case GUI_APP_MSG_ONSTOP:    on_stop();    break;
    default: break;
    }
}

static int app_main(intent_t i)
{
    (void)i;
    gui_app_regist_msg_handler(APP_ID_SLEEP, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(sleep), IMG_SLEEP, APP_ID_SLEEP, app_main);

#endif /* APP_ID_SLEEP */
