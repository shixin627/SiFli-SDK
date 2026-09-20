/**
 * @file   app_skaiapp.c
 * @brief  SkaiApp host — the ONE builtin app that surfaces every AI-generated
 *         mini-app (SkaiLink ADR-0037). Launcher lists installed packages from
 *         skaiapp_store; opening one parses its JSON into a transient model and
 *         renders it (skaiapp_render). A 500 ms tick refreshes live binds and
 *         follows store changes, so a phone push UPDATES the open page in
 *         place — the "不滿意再改" loop lands here.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <rtthread.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "ui_img_helper.h"
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#endif
#include "skaiapp_pkg.h"
#include "skaiapp_store.h"
#include "skaiapp_render.h"
#ifdef PKG_USING_QUICKJS
#include "skai/skai_pkg.h"
#endif

#define DBG_TAG "app.skaiapp"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_SKAIAPP

typedef struct
{
    lv_obj_t *root;      /* full-screen holder on lv_scr_act() */
    lv_obj_t *launcher;  /* list view */
    lv_obj_t *page;      /* rendered mini-app view */
    lv_obj_t *back_btn;  /* overlay ‹ on the page view */
    lv_obj_t *confirm;   /* "remove this app?" sheet over the launcher */
    skaiapp_model_t *model;
    skaiapp_render_ctx_t ctx;
    char open_id[SKAIAPP_ID_MAX];
    uint32_t last_gen;
    lv_timer_t *tick;
    bool paused;
} skaiapp_ui_t;

static skaiapp_ui_t ui;

static void build_launcher(void);
static void open_app(const char *id);
static void close_page(void);

/* ── launcher ── */

#ifdef PKG_USING_QUICKJS
/* JS apps (signed packages, ADR-0019 Phase 3) sit in the same list as the
   declarative ones: to the person wearing the watch both are "an app the AI
   made", and one list is where they look. The row index is looked up again on
   tap because the package scan order is the filesystem's, not ours. */
static void js_row_click_cb(lv_event_t *e)
{
    static skai_pkg_info_t info;   /* ~1.4 KB: keep it off the GUI stack */
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);

    if (skai_pkg_at(idx, &info) && info.is_js)
    {
        skai_pkg_result_t r = skai_pkg_launch(info.keyid, info.app_id);
        if (r != SKAI_PKG_OK)
        {
            LOG_W("launch %s/%s: %s", info.keyid, info.app_id, skai_pkg_result_name(r));
        }
    }
}
#endif

/* ── remove an app from the watch itself ──
 *
 * Until now an app the AI made could only be taken off from the phone, by
 * asking the Bot; on the wrist there was no way at all, and the watch holds
 * 16 (founder 2026-09-20: 「手錶上加上長按可以出現刪除 ai 新增的 app 的選項」).
 * Long press a row, confirm, and it is gone. A long press cannot be a slip,
 * and the sheet asks before anything is deleted. */

typedef struct
{
    bool is_js;
    int  idx;
    char name[SKAIAPP_NAME_MAX];
} remove_target_t;

static remove_target_t s_target;

static void close_confirm(void)
{
    if (ui.confirm != NULL)
    {
        lv_obj_del(ui.confirm);
        ui.confirm = NULL;
    }
}

static void confirm_cancel_cb(lv_event_t *e)
{
    (void)e;
    close_confirm();
}

static void confirm_remove_cb(lv_event_t *e)
{
    (void)e;
    close_confirm();
    if (s_target.is_js)
    {
#ifdef PKG_USING_QUICKJS
        static skai_pkg_info_t info;
        if (skai_pkg_at(s_target.idx, &info) && info.is_js)
        {
            skai_pkg_result_t r = skai_pkg_remove(info.keyid, info.app_id);
            LOG_I("remove %s/%s: %s", info.keyid, info.app_id,
                  skai_pkg_result_name(r));
        }
#endif
    }
    else
    {
        char id[SKAIAPP_ID_MAX];
        if (skaiapp_store_meta(s_target.idx, id, NULL, NULL))
        {
            skaiapp_store_remove(id);
        }
    }
    build_launcher();
}

static lv_obj_t *sheet_button(lv_obj_t *parent, const char *text,
                              uint32_t bg, uint32_t fg, lv_event_cb_t cb)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, 130, 56);
    lv_obj_set_style_radius(b, 28, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(bg), 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = lv_label_create(b);
    lv_obj_set_style_text_font(l, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(l, lv_color_hex(fg), 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return b;
}

static void ask_remove(bool is_js, int idx, const char *name)
{
    close_confirm();
    s_target.is_js = is_js;
    s_target.idx = idx;
    rt_strncpy(s_target.name, name ? name : "", sizeof(s_target.name) - 1);
    s_target.name[sizeof(s_target.name) - 1] = '\0';

    /* Dim the launcher so the question owns the screen. */
    ui.confirm = lv_obj_create(ui.root);
    lv_obj_remove_style_all(ui.confirm);
    lv_obj_set_size(ui.confirm, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(ui.confirm, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ui.confirm, LV_OPA_80, 0);
    lv_obj_clear_flag(ui.confirm, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *card = lv_obj_create(ui.confirm);
    lv_obj_remove_style_all(card);
    lv_obj_set_size(card, 330, 240);
    lv_obj_center(card);
    lv_obj_set_style_radius(card, 24, 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(card, 20, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(card, 12, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *q = lv_label_create(card);
    lv_obj_set_style_text_font(q, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(q, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(q, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(q, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(q, 280);
    lv_label_set_text(q, s_target.name);

    lv_obj_t *sub = lv_label_create(card);
    lv_obj_set_style_text_font(sub, LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(sub, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_align(sub, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(sub, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(sub, 280);
    lv_label_set_text(sub, LV_EXT_STR_GET_BY_KEY(skaiapp_remove_q,
                      "Remove this app from the watch?"));

    lv_obj_t *row = lv_obj_create(card);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, 280, 56);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 12, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    sheet_button(row, LV_EXT_STR_GET_BY_KEY(skaiapp_cancel, "Cancel"),
                 0x2C2C2E, 0xFFFFFF, confirm_cancel_cb);
    sheet_button(row, LV_EXT_STR_GET_BY_KEY(skaiapp_remove, "Remove"),
                 0xFF453A, 0xFFFFFF, confirm_remove_cb);
}

static void row_long_cb(lv_event_t *e)
{
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);
    char id[SKAIAPP_ID_MAX], name[SKAIAPP_NAME_MAX];
    if (skaiapp_store_meta(idx, id, name, NULL))
    {
        ask_remove(false, idx, name);
    }
}

#ifdef PKG_USING_QUICKJS
static void js_row_long_cb(lv_event_t *e)
{
    static skai_pkg_info_t info;
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);

    if (skai_pkg_at(idx, &info) && info.is_js)
    {
        ask_remove(true, idx, info.name);
    }
}
#endif

static void row_click_cb(lv_event_t *e)
{
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);
    char id[SKAIAPP_ID_MAX];
    if (skaiapp_store_meta(idx, id, NULL, NULL))
    {
        open_app(id);
    }
}

static void build_launcher(void)
{
    if (ui.launcher != NULL)
    {
        lv_obj_del(ui.launcher);
        ui.launcher = NULL;
    }
    ui.launcher = lv_obj_create(ui.root);
    lv_obj_remove_style_all(ui.launcher);
    lv_obj_set_size(ui.launcher, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(ui.launcher, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ui.launcher, LV_OPA_COVER, 0);
    lv_obj_set_flex_flow(ui.launcher, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui.launcher, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_top(ui.launcher, 70, 0);
    lv_obj_set_style_pad_bottom(ui.launcher, 90, 0);
    lv_obj_set_style_pad_row(ui.launcher, 12, 0);
    lv_obj_add_flag(ui.launcher, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(ui.launcher, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *title = lv_label_create(ui.launcher);
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(skaiapp, "AI Apps"));

    int n = skaiapp_store_count();
    int n_js = 0;
#ifdef PKG_USING_QUICKJS
    n_js = skai_pkg_count();
#endif
    if (n == 0 && n_js == 0)
    {
        lv_obj_t *empty = lv_label_create(ui.launcher);
        lv_obj_set_style_text_font(empty,
                                   LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
        lv_obj_set_style_text_color(empty, lv_color_hex(0x8E8E93), 0);
        lv_obj_set_style_text_align(empty, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_long_mode(empty, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(empty, 320);
        lv_obj_set_style_pad_top(empty, 60, 0);
        lv_label_set_text(empty, LV_EXT_STR_GET_BY_KEY(skaiapp_empty,
                          "No mini apps yet.\nCreate one with AI in SkaiLink."));
        return;
    }
    for (int i = 0; i < n; i++)
    {
        char id[SKAIAPP_ID_MAX], name[SKAIAPP_NAME_MAX];
        uint8_t icon = 0xFF;
        if (!skaiapp_store_meta(i, id, name, &icon))
        {
            break;
        }
        lv_obj_t *row = lv_btn_create(ui.launcher);
        lv_obj_set_size(row, 340, 72);
        lv_obj_set_style_radius(row, 20, 0);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_shadow_width(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_hor(row, 20, 0);
        lv_obj_set_style_pad_column(row, 16, 0);
        /* SHORT_CLICKED, not CLICKED: LVGL sends CLICKED on release even when the
           press was long, so opening on CLICKED would open the app the moment the
           person lifted their finger off the remove sheet (2026-09-20). */
        lv_obj_add_event_cb(row, row_click_cb, LV_EVENT_SHORT_CLICKED,
                            (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row, row_long_cb, LV_EVENT_LONG_PRESSED,
                            (void *)(uintptr_t)i);

        lv_obj_t *img = lv_img_create(row);
        lv_img_set_src(img, skaiapp_render_icon_src(icon));

        lv_obj_t *lbl = lv_label_create(row);
        lv_obj_set_style_text_font(lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, 220);
        lv_label_set_text(lbl, name);
    }
#ifdef PKG_USING_QUICKJS
    for (int i = 0; i < n_js; i++)
    {
        static skai_pkg_info_t info;
        if (!skai_pkg_at(i, &info) || !info.is_js)
        {
            continue;
        }
        lv_obj_t *row = lv_btn_create(ui.launcher);
        lv_obj_set_size(row, 340, 72);
        lv_obj_set_style_radius(row, 20, 0);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_shadow_width(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_hor(row, 20, 0);
        lv_obj_set_style_pad_column(row, 16, 0);
        lv_obj_add_event_cb(row, js_row_click_cb, LV_EVENT_SHORT_CLICKED,
                            (void *)(uintptr_t)i);
        lv_obj_add_event_cb(row, js_row_long_cb, LV_EVENT_LONG_PRESSED,
                            (void *)(uintptr_t)i);

        lv_obj_t *img = lv_img_create(row);
        lv_img_set_src(img, IMG_LOGO);

        lv_obj_t *lbl = lv_label_create(row);
        lv_obj_set_style_text_font(lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, 220);
        lv_label_set_text(lbl, info.name);
    }
#endif
}

/* ── page ── */

static void back_click_cb(lv_event_t *e)
{
    (void)e;
    close_page();
}

static void close_page(void)
{
    if (ui.page != NULL)
    {
        lv_obj_del(ui.page);
        ui.page = NULL;
    }
    if (ui.back_btn != NULL)
    {
        lv_obj_del(ui.back_btn);
        ui.back_btn = NULL;
    }
    skaiapp_render_detach();
    if (ui.model != NULL)
    {
        rt_free(ui.model);
        ui.model = NULL;
    }
    ui.open_id[0] = '\0';
    if (ui.launcher != NULL)
    {
        lv_obj_clear_flag(ui.launcher, LV_OBJ_FLAG_HIDDEN);
    }
    build_launcher();
}

static void open_app(const char *id)
{
    uint8_t *raw = NULL;
    uint32_t len = 0;
    if (skaiapp_store_load(id, &raw, &len) != 0)
    {
        LOG_W("open '%s': load failed", id);
        return;
    }
    skaiapp_model_t *m = rt_malloc(sizeof(skaiapp_model_t));
    if (m == NULL)
    {
        rt_free(raw);
        return;
    }
    /* display re-parse: seed_out NULL so live timer state is untouched */
    int pr = skaiapp_pkg_parse(raw, len, m, NULL);
    rt_free(raw);
    if (pr != 0)
    {
        LOG_W("open '%s': parse=%d", id, pr);
        rt_free(m);
        return;
    }

    /* tear down any previous page, keep launcher hidden behind the page */
    if (ui.page != NULL)
    {
        lv_obj_del(ui.page);
        ui.page = NULL;
    }
    if (ui.back_btn != NULL)
    {
        lv_obj_del(ui.back_btn);
        ui.back_btn = NULL;
    }
    skaiapp_render_detach();
    if (ui.model != NULL)
    {
        rt_free(ui.model);
    }
    ui.model = m;
    strncpy(ui.open_id, id, sizeof(ui.open_id) - 1);
    ui.open_id[sizeof(ui.open_id) - 1] = '\0';

    if (ui.launcher != NULL)
    {
        lv_obj_add_flag(ui.launcher, LV_OBJ_FLAG_HIDDEN);
    }
    ui.page = skaiapp_render_page(ui.root, ui.model, &ui.ctx);

    ui.back_btn = lv_btn_create(ui.root);
    lv_obj_set_size(ui.back_btn, 56, 56);
    lv_obj_set_style_radius(ui.back_btn, 28, 0);
    lv_obj_set_style_bg_color(ui.back_btn, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_bg_opa(ui.back_btn, LV_OPA_70, 0);
    lv_obj_set_style_shadow_width(ui.back_btn, 0, 0);
    lv_obj_align(ui.back_btn, LV_ALIGN_TOP_LEFT, 26, 40);
    lv_obj_add_event_cb(ui.back_btn, back_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(ui.back_btn);
    lv_label_set_text(bl, LV_SYMBOL_LEFT);
    lv_obj_center(bl);
}

/* ── tick: live binds + follow store changes (push-updates-open-page) ── */

static void tick_cb(lv_timer_t *t)
{
    (void)t;
    if (ui.paused || ui.root == NULL)
    {
        return;
    }
    uint32_t gen = skaiapp_store_generation();
    if (gen != ui.last_gen)
    {
        ui.last_gen = gen;
        if (ui.open_id[0] != '\0' && skaiapp_store_exists(ui.open_id))
        {
            char keep[SKAIAPP_ID_MAX];
            strncpy(keep, ui.open_id, sizeof(keep));
            keep[sizeof(keep) - 1] = '\0';
            open_app(keep); /* re-parse: phone just updated this app in place */
        }
        else if (ui.open_id[0] != '\0')
        {
            close_page();   /* the open app was removed */
        }
        else
        {
            build_launcher();
        }
        return;
    }
    if (ui.page != NULL && ui.model != NULL)
    {
        skaiapp_render_refresh(ui.model, &ui.ctx);
    }
}

/* ── lifecycle ── */

static void on_start(void)
{
    memset(&ui, 0, sizeof(ui));
#if defined(BSP_USING_PC_SIMULATOR)
    /* FINSH stdin is unreliable headless — seed the embedded samples so the
       render/engine paths can be exercised + screenshotted on the sim. */
    extern void skaiapp_sim_seed_all_if_empty(void);
    skaiapp_sim_seed_all_if_empty();
#endif
    ui.root = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(ui.root);
    lv_obj_set_size(ui.root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(ui.root, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ui.root, LV_OPA_COVER, 0);

    ui.last_gen = skaiapp_store_generation();
    build_launcher();

    /* a push landed ≤30 s ago → the phone is showing off the new app: jump in */
    char fresh[SKAIAPP_ID_MAX];
    if (skaiapp_store_last_installed(fresh) && skaiapp_store_exists(fresh))
    {
        open_app(fresh);
    }
#if defined(BSP_USING_PC_SIMULATOR)
    /* sim: deterministically open the memo+timer sample so a headless screenshot
       exercises the memo render path (user-authored text) alongside a timer. */
    else if (skaiapp_store_exists("note-timer"))
    {
        open_app("note-timer");
    }
#endif

    ui.tick = lv_timer_create(tick_cb, 500, NULL);
}

static void on_resume(void)
{
    ui.paused = false;
    if (ui.page != NULL && ui.model != NULL)
    {
        skaiapp_render_refresh(ui.model, &ui.ctx);
    }
}

static void on_pause(void)
{
    ui.paused = true;
}

static void on_stop(void)
{
    if (ui.tick != NULL)
    {
        lv_timer_del(ui.tick);
        ui.tick = NULL;
    }
    skaiapp_render_detach();
    if (ui.root != NULL)
    {
        lv_obj_del(ui.root); /* takes launcher/page/back_btn with it */
        ui.root = NULL;
    }
    if (ui.model != NULL)
    {
        rt_free(ui.model);
        ui.model = NULL;
    }
    ui.launcher = NULL;
    ui.page = NULL;
    ui.back_btn = NULL;
    ui.confirm = NULL;   /* deleting the root took the sheet with it */
    ui.open_id[0] = '\0';
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    (void)param;
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        /* app_run 直接開啟不經 Main 狀態機，左緣右滑返回 bar 仍隱藏，這裡補開 */
        extern void display_gesture_detect_objs(uint32_t idx, bool display);
        display_gesture_detect_objs(0, true);
        on_start();
        break;
    }
    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;
    case GUI_APP_MSG_ONPAUSE:
        on_pause();
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
    gui_app_regist_msg_handler(APP_ID_SKAIAPP, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(skaiapp), IMG_LOGO, APP_ID_SKAIAPP, app_main, 1);
#endif /* APP_ID_SKAIAPP */
