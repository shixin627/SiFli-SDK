/**
 * @file   app_onboarding.c
 * @brief  Watch onboarding tutorial overlay.
 *
 * State machine:
 *   INACTIVE
 *     |  onboarding_start()
 *     v
 *   HOME — overlay is on lv_layer_top():
 *          - 4 colored edge bars (top/bottom/left/right) pulsing
 *          - center "release" hint: arrow that slides side to side
 *            (placeholder; replace with lv_gif_create on hardware where
 *             LV_USE_GIF=y — PC sim has it disabled).
 *          Each bar disappears as the user swipes to the corresponding
 *          drawer:
 *            top  -> message page    (is_at_message)
 *            bot  -> control center  (is_at_control_center)
 *            left -> instruction list(is_at_instruction_list)
 *            right-> AI interface    (is_at_ai_interface)
 *     |  is_at_instruction_list()
 *     v
 *   LIST  — bars + release hint hidden; "tap" hint shown:
 *           a pulsing ring at center plus the tip "點擊執行,或再次滑動進入 AI"
 *     |  is_at_ai_interface()
 *     v
 *   DONE  — overlay torn down, polling stopped.
 *
 * MSH trigger:  `onboarding [start|stop]`
 *
 * Future hookup: flash a "tutorial_done" KV via FlashDB and skip
 * onboarding_start() on subsequent boots when set.
 */

#include <rtthread.h>
#include <string.h>
#include "lvgl.h"
#include "lv_ext_resource_manager.h"
#include "app_mainmenu.h"
#include "app_onboarding.h"

#ifdef RT_USING_FINSH
    #include <finsh.h>
#endif

#define DBG_TAG "app.onboarding"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ===== Tunables ===== */
#define BAR_THICK              14
#define SCREEN_W               466
#define SCREEN_H               466
#define BAR_DIM_OPA            LV_OPA_50
#define BAR_BRIGHT_OPA         LV_OPA_COVER
#define POLL_PERIOD_MS         200
#define BAR_PULSE_MS           700
#define ARROW_SLIDE_MS         1500
#define RING_PULSE_MS          700

/* ===== State ===== */
typedef enum
{
    OB_INACTIVE = 0,
    OB_HOME,
    OB_LIST,
    OB_DONE,
} ob_state_t;

static ob_state_t _state = OB_INACTIVE;

/* Per-direction visited flags (true = user already reached that drawer). */
static bool _visited_top, _visited_bottom, _visited_left, _visited_right;

/* LVGL objects */
static lv_obj_t *_root;
static lv_obj_t *_bar_top, *_bar_bottom, *_bar_left, *_bar_right;
static lv_obj_t *_release_hint;   /* container for home-phase center hint */
static lv_obj_t *_release_arrow;  /* sliding arrow inside release_hint */
static lv_obj_t *_tap_hint;       /* container for list-phase center hint */
static lv_obj_t *_tap_ring;       /* pulsing ring inside tap_hint */

static lv_timer_t *_poll_timer;

/* ===== Animation callbacks ===== */
static void anim_bg_opa_cb(void *obj, int32_t v)
{
    if (obj)
        lv_obj_set_style_bg_opa((lv_obj_t *)obj, (lv_opa_t)v, 0);
}

static void anim_x_cb(void *obj, int32_t v)
{
    if (obj)
        lv_obj_set_x((lv_obj_t *)obj, v);
}

static void anim_size_cb(void *obj, int32_t v)
{
    if (!obj)
        return;
    lv_obj_set_size((lv_obj_t *)obj, v, v);
    lv_obj_align((lv_obj_t *)obj, LV_ALIGN_CENTER, 0, 0);
}

/* ===== Builders ===== */
static lv_obj_t *make_edge_bar(int x, int y, int w, int h)
{
    lv_obj_t *bar = lv_obj_create(_root);
    lv_obj_remove_style_all(bar);
    lv_obj_set_pos(bar, x, y);
    lv_obj_set_size(bar, w, h);
    lv_obj_set_style_bg_color(bar, lv_color_make(0xFF, 0xCC, 0x33), 0);
    lv_obj_set_style_bg_opa(bar, BAR_DIM_OPA, 0);
    lv_obj_set_style_radius(bar, (h < w ? h : w) / 2, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    /* Pulse opacity to draw the eye to the bar. */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, bar);
    lv_anim_set_exec_cb(&a, anim_bg_opa_cb);
    lv_anim_set_values(&a, BAR_DIM_OPA, BAR_BRIGHT_OPA);
    lv_anim_set_time(&a, BAR_PULSE_MS);
    lv_anim_set_playback_time(&a, BAR_PULSE_MS);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);

    return bar;
}

static void create_release_hint(void)
{
    _release_hint = lv_obj_create(_root);
    lv_obj_remove_style_all(_release_hint);
    lv_obj_set_size(_release_hint, 280, 110);
    lv_obj_align(_release_hint, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(_release_hint, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_bg_opa(_release_hint, LV_OPA_60, 0);
    lv_obj_set_style_radius(_release_hint, 12, 0);
    lv_obj_clear_flag(_release_hint, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(_release_hint, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(_release_hint);
    lv_label_set_text(lbl, LV_EXT_STR_GET_BY_KEY(swipe_explore, "Swipe in four directions to explore"));
    lv_obj_set_style_text_color(lbl, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 12);

    /* Sliding arrow stand-in for release.gif (LV_USE_GIF is off on PC). */
    _release_arrow = lv_label_create(_release_hint);
    lv_label_set_text(_release_arrow,
                      LV_SYMBOL_RIGHT " " LV_SYMBOL_RIGHT " "
                                      LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(_release_arrow,
                                lv_color_make(0xFF, 0xCC, 0x33), 0);
    lv_obj_align(_release_arrow, LV_ALIGN_BOTTOM_LEFT, 20, -16);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, _release_arrow);
    lv_anim_set_exec_cb(&a, anim_x_cb);
    lv_anim_set_values(&a, 20, 200);
    lv_anim_set_time(&a, ARROW_SLIDE_MS);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

static void create_tap_hint(void)
{
    _tap_hint = lv_obj_create(_root);
    lv_obj_remove_style_all(_tap_hint);
    lv_obj_set_size(_tap_hint, 320, 180);
    lv_obj_align(_tap_hint, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(_tap_hint, lv_color_make(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_bg_opa(_tap_hint, LV_OPA_60, 0);
    lv_obj_set_style_radius(_tap_hint, 12, 0);
    lv_obj_clear_flag(_tap_hint, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(_tap_hint, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(_tap_hint, LV_OBJ_FLAG_HIDDEN);

    /* Pulsing ring stand-in for tap.gif (LV_USE_GIF is off on PC). */
    _tap_ring = lv_obj_create(_tap_hint);
    lv_obj_remove_style_all(_tap_ring);
    lv_obj_set_size(_tap_ring, 50, 50);
    lv_obj_align(_tap_ring, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_radius(_tap_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(_tap_ring, 4, 0);
    lv_obj_set_style_border_color(_tap_ring,
                                  lv_color_make(0xFF, 0xCC, 0x33), 0);
    lv_obj_set_style_bg_opa(_tap_ring, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(_tap_ring, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(_tap_ring, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(_tap_hint);
    lv_label_set_text(lbl, LV_EXT_STR_GET_BY_KEY(swipe_run_ai, "Tap to run, or swipe again for AI"));
    lv_obj_set_style_text_color(lbl, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -16);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, _tap_ring);
    lv_anim_set_exec_cb(&a, anim_size_cb);
    lv_anim_set_values(&a, 40, 90);
    lv_anim_set_time(&a, RING_PULSE_MS);
    lv_anim_set_playback_time(&a, RING_PULSE_MS);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

static void create_overlay(void)
{
    if (_root)
        return;
    _root = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(_root);
    lv_obj_set_size(_root, SCREEN_W, SCREEN_H);
    lv_obj_align(_root, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(_root, LV_OPA_TRANSP, 0);
    /* Crucial: keep underlying tile_view scroll/swipes working. */
    lv_obj_clear_flag(_root, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(_root, LV_OBJ_FLAG_SCROLLABLE);

    _bar_top = make_edge_bar(0, 0, SCREEN_W, BAR_THICK);
    _bar_bottom =
        make_edge_bar(0, SCREEN_H - BAR_THICK, SCREEN_W, BAR_THICK);
    _bar_left =
        make_edge_bar(0, BAR_THICK, BAR_THICK, SCREEN_H - 2 * BAR_THICK);
    _bar_right = make_edge_bar(SCREEN_W - BAR_THICK, BAR_THICK, BAR_THICK,
                               SCREEN_H - 2 * BAR_THICK);

    create_release_hint();
    create_tap_hint();

    LOG_I("onboarding overlay created");
}

static void destroy_overlay(void)
{
    if (!_root)
        return;
    lv_obj_del(_root);
    _root = NULL;
    _bar_top = _bar_bottom = _bar_left = _bar_right = NULL;
    _release_hint = _release_arrow = NULL;
    _tap_hint = _tap_ring = NULL;
    LOG_I("onboarding overlay destroyed");
}

static void hide_obj(lv_obj_t *obj)
{
    if (obj && !lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN))
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

static void show_obj(lv_obj_t *obj)
{
    if (obj && lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN))
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

static void poll_cb(lv_timer_t *t)
{
    (void)t;
    if (_state == OB_INACTIVE || _state == OB_DONE)
        return;
    if (!_root)
        return;

    /* Detect drawer visits (any phase). */
    if (is_at_message())
        _visited_top = true;
    if (is_at_control_center())
        _visited_bottom = true;
    if (is_at_instruction_list())
        _visited_left = true;
    if (is_at_ai_interface())
        _visited_right = true;

    if (_state == OB_HOME)
    {
        /* Hide visited bars. */
        if (_visited_top)
            hide_obj(_bar_top);
        if (_visited_bottom)
            hide_obj(_bar_bottom);
        if (_visited_left)
            hide_obj(_bar_left);
        if (_visited_right)
            hide_obj(_bar_right);

        /* Phase advance: instruction list reached -> LIST phase. */
        if (is_at_instruction_list())
        {
            hide_obj(_release_hint);
            hide_obj(_bar_top);
            hide_obj(_bar_bottom);
            hide_obj(_bar_left);
            hide_obj(_bar_right);
            show_obj(_tap_hint);
            _state = OB_LIST;
            LOG_I("onboarding: HOME -> LIST");
        }
    }
    else if (_state == OB_LIST)
    {
        /* AI page reached -> tutorial complete. */
        if (is_at_ai_interface())
        {
            destroy_overlay();
            _state = OB_DONE;
            if (_poll_timer)
                lv_timer_pause(_poll_timer);
            LOG_I("onboarding: LIST -> DONE (reached AI)");
        }
    }
}

void onboarding_start(void)
{
    if (_state != OB_INACTIVE && _state != OB_DONE)
    {
        LOG_I("onboarding_start: already active");
        return;
    }
    _visited_top = _visited_bottom = _visited_left = _visited_right = false;
    create_overlay();
    _state = OB_HOME;
    if (!_poll_timer)
    {
        _poll_timer = lv_timer_create(poll_cb, POLL_PERIOD_MS, NULL);
    }
    else
    {
        lv_timer_resume(_poll_timer);
    }
    LOG_I("onboarding started");
}

void onboarding_stop(void)
{
    _state = OB_INACTIVE;
    if (_poll_timer)
        lv_timer_pause(_poll_timer);
    destroy_overlay();
    LOG_I("onboarding stopped");
}

bool onboarding_is_active(void)
{
    return _state != OB_INACTIVE && _state != OB_DONE;
}

#ifdef RT_USING_FINSH

static int onboarding_msh(int argc, char **argv)
{
    if (argc < 2 || rt_strcmp(argv[1], "start") == 0)
    {
        onboarding_start();
        rt_kprintf("onboarding started\n");
    }
    else if (rt_strcmp(argv[1], "stop") == 0)
    {
        onboarding_stop();
        rt_kprintf("onboarding stopped\n");
    }
    else if (rt_strcmp(argv[1], "status") == 0)
    {
        rt_kprintf("onboarding active: %s\n",
                   onboarding_is_active() ? "yes" : "no");
    }
    else
    {
        rt_kprintf("usage: onboarding [start|stop|status]\n");
    }
    return 0;
}
MSH_CMD_EXPORT_ALIAS(onboarding_msh, onboarding,
                     onboarding[start | stop | status] : tutorial overlay);

#endif /* RT_USING_FINSH */
