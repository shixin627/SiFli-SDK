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
    if (n == 0)
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
        lv_obj_add_event_cb(row, row_click_cb, LV_EVENT_CLICKED,
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

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(skaiapp), IMG_LOGO, APP_ID_SKAIAPP, app_main);
#endif /* APP_ID_SKAIAPP */
