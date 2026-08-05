/*
 * Edge-back gesture bars, ported from middleware/lvgl/lvsf/lvsf_gesture.c.
 *
 * lvsf_v9 ships the header for this and no implementation, so a v9 build had
 * lvsf_gesture_init() and display_gesture_detect_objs() as link errors. Same
 * behaviour as v8, with the v9 renames applied:
 *
 *   lv_img_*            -> lv_image_*        LV_IMG_DECLARE -> LV_IMAGE_DECLARE
 *   lv_obj_clear_flag   -> lv_obj_remove_flag
 *   lv_obj_del          -> lv_obj_delete
 *   lv_indev_get_act    -> lv_indev_active
 *   lv_anim_set_time    -> lv_anim_set_duration
 *   set_style_img_opa   -> set_style_image_opa
 *   obj->coords.x1      -> lv_obj_get_coords(), so this does not need
 *                          lvgl_private.h
 *
 * The animation exec callbacks take int32_t here rather than v8's uint16_t.
 * v9's lv_anim_exec_xcb_t is (void *, int32_t); the v8 file cast its way past
 * that mismatch, and the cast hid a real one -- back_hint_anim_cb tested
 * `x < 0` on an unsigned parameter, which is never true. With the right
 * signature that guard works.
 */
#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"

#include <string.h>

#define DBG_TAG "lvsf.gesture"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if defined(GUI_APP_FRAMEWORK) && (!defined(APP_TRANS_ANIMATION_NONE))

#include "gui_app_fwk.h"

#define LVSF_GESTURE_DETECT_WIDTH     (20)
#define LVSF_GESTURE_DETECT_HEIGHT    (10)
#define LVSF_GESTURE_LIMIT            (80)
#define LVSF_GESTURE_BACK_HINT_SIZE   (70)

/* 0 left, 1 top, 2 right, 3 bottom */
LV_IMAGE_DECLARE(img_left_arrow);
lv_obj_t *gesture_detect_objs[4];
lv_obj_t *gesture_img_objs[4];
static bool    gesture_is_enabled = true;
bool           gesture_is_active = false;
static uint8_t gesture_enable_reg = 0xff;

static int32_t obj_left_edge(lv_obj_t *obj)
{
    lv_area_t a;

    lv_obj_get_coords(obj, &a);
    return a.x1;
}

uint8_t lvsf_gesture_enable_register(uint8_t enable)
{
    uint8_t old_enable = gesture_enable_reg;
    gesture_enable_reg = enable;
    return old_enable;
}

void lvsf_gesture_bars_realign(void)
{
    if (gesture_detect_objs[0])
    {
        if (gesture_img_objs[0])
        {
            lv_obj_set_size(gesture_detect_objs[0],
                            lv_obj_get_self_width(gesture_img_objs[0]) +
                            LVSF_GESTURE_DETECT_WIDTH,
                            LV_VER_RES_MAX);
            lv_obj_align(gesture_img_objs[0], LV_ALIGN_LEFT_MID, 0, 0);
            lv_obj_add_flag(gesture_img_objs[0], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_set_size(gesture_detect_objs[0], LVSF_GESTURE_DETECT_WIDTH,
                            LV_VER_RES_MAX);
        }
        lv_obj_align(gesture_detect_objs[0], LV_ALIGN_OUT_LEFT_MID, 0, 0);
    }
    if (gesture_detect_objs[2])
    {
        lv_obj_set_size(gesture_detect_objs[2], LVSF_GESTURE_DETECT_WIDTH,
                        LV_VER_RES_MAX);
        lv_obj_align(gesture_detect_objs[2], LV_ALIGN_RIGHT_MID, 0, 0);
    }
    if (gesture_detect_objs[1])
    {
        lv_obj_set_size(gesture_detect_objs[1], LV_HOR_RES_MAX,
                        LVSF_GESTURE_DETECT_HEIGHT);
        lv_obj_align(gesture_detect_objs[1], LV_ALIGN_TOP_MID, 0, 0);
    }
    if (gesture_detect_objs[3])
    {
        lv_obj_set_size(gesture_detect_objs[3], LV_HOR_RES_MAX,
                        LVSF_GESTURE_DETECT_HEIGHT);
        lv_obj_align(gesture_detect_objs[3], LV_ALIGN_BOTTOM_MID, 0, 0);
    }
}

static void gesture_enable_update(bool enable)
{
    for (uint32_t i = 0; i < 4; i++)
    {
        if (gesture_detect_objs[i])
        {
            if (enable)
                lv_obj_remove_flag(gesture_detect_objs[i], LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(gesture_detect_objs[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static lv_obj_t *gesture_detect_hint_obj = NULL;
static lv_obj_t *gesture_detect_hint_icon = NULL;
static lv_anim_t back_hint_anim;
static lv_anim_t back_hint_release_anim;
static bool      back_hint_hidden = true;
static bool      back_hint_vibrated = false;   /* buzz once per crossing */

static void back_hint_anim_cb(void *obj, int32_t x)
{
    if (!lv_obj_is_valid(obj))
    {
        return;
    }
    if (x < 0)
    {
        return;
    }

    if (x < LVSF_GESTURE_LIMIT)
    {
        int32_t obj_width = LVSF_GESTURE_BACK_HINT_SIZE * x / LVSF_GESTURE_LIMIT;

        lv_obj_set_width(obj, obj_width);
        lv_obj_set_style_radius(obj, 25, 0);
        lv_obj_align(obj, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_set_style_image_opa(gesture_detect_hint_icon,
                                   (obj_width > 30) ? LV_OPA_COVER : LV_OPA_TRANSP,
                                   0);
        if (back_hint_vibrated)
        {
            back_hint_vibrated = false;
        }
    }
    else
    {
        lv_obj_set_width(obj, LVSF_GESTURE_BACK_HINT_SIZE);
        lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
        lv_obj_align(obj, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_set_style_image_opa(gesture_detect_hint_icon, LV_OPA_COVER, 0);

        if (!back_hint_vibrated)
        {
#ifdef SkaiwalkWatchOS
            extern void motor_pattern_scrolling_app(void);
            motor_pattern_scrolling_app();
#endif
            back_hint_vibrated = true;
        }
    }
}

static void hidden_back_hint(bool hide)
{
    back_hint_hidden = hide;
    if (hide)
    {
        lv_obj_add_flag(gesture_detect_hint_obj, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_set_style_bg_opa(gesture_detect_hint_obj, LV_OPA_80, 0);
        lv_obj_remove_flag(gesture_detect_hint_obj, LV_OBJ_FLAG_HIDDEN);
    }
}

static void hidden_back_hint_release_anim_cb(lv_anim_t *a)
{
    LV_UNUSED(a);
    hidden_back_hint(true);
    back_hint_anim_cb(gesture_detect_hint_obj, 0);
}

static bool back_hint_anim_is_running = false;

static void hidden_back_hint_cb(lv_anim_t *a)
{
    LV_UNUSED(a);
    back_hint_anim_cb(gesture_detect_hint_obj, LVSF_GESTURE_LIMIT);
    back_hint_anim_is_running = false;
}

static void set_gesture_detect_hint_obj_opa(void *obj, int32_t opa)
{
    if (lv_obj_is_valid(obj))
    {
        lv_obj_set_style_bg_opa(obj, (lv_opa_t)opa, 0);
    }
}

static void left_bar_event_handler(lv_event_t *e)
{
#define x_2_process(x) ((x > 0) ? ((x) * GUI_APP_TRANS_PROGRESS_MAX / LV_HOR_RES) : 0)

    lv_obj_t       *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    static int32_t     drag_offset = 0;
    static lv_point_t  start_point;
    static lv_point_t  end_point;

    if (event == LV_EVENT_SCROLL_BEGIN)
    {
        if (gesture_img_objs[0])
            lv_obj_remove_flag(gesture_img_objs[0], LV_OBJ_FLAG_HIDDEN);
        gesture_is_active = true;

        gui_app_manual_animation_start(x_2_process(obj_left_edge(obj)));
    }
    else if (event == LV_EVENT_PRESSING)
    {
        lv_indev_get_point(lv_indev_active(), &end_point);
        drag_offset = LV_ABS(end_point.x - start_point.x);

        if (drag_offset > LVSF_GESTURE_LIMIT - 10 && back_hint_hidden)
        {
            back_hint_anim_is_running = true;
            hidden_back_hint(false);
            lv_anim_init(&back_hint_anim);
            lv_anim_set_var(&back_hint_anim, gesture_detect_hint_obj);
            lv_anim_set_duration(&back_hint_anim, 50);
            lv_anim_set_values(&back_hint_anim, 0, LVSF_GESTURE_LIMIT);
            lv_anim_set_exec_cb(&back_hint_anim, back_hint_anim_cb);
            lv_anim_set_completed_cb(&back_hint_anim, hidden_back_hint_cb);
            lv_anim_start(&back_hint_anim);
        }
        if (!back_hint_hidden && !back_hint_anim_is_running)
        {
            back_hint_anim_cb(gesture_detect_hint_obj, drag_offset);
        }

        if (gesture_is_active)
            gui_app_manual_animation_update(x_2_process(obj_left_edge(obj)));
    }
    else if (event == LV_EVENT_PRESSED)
    {
        lv_indev_get_point(lv_indev_active(), &start_point);
    }
    else if (event == LV_EVENT_RELEASED)
    {
        back_hint_vibrated = false;

        if (LV_ABS(start_point.x - end_point.x) > LVSF_GESTURE_LIMIT)
        {
            lv_anim_init(&back_hint_release_anim);
            lv_anim_set_duration(&back_hint_release_anim, 200);
            lv_anim_set_values(&back_hint_release_anim, LV_OPA_80, LV_OPA_TRANSP);
            lv_anim_set_var(&back_hint_release_anim, gesture_detect_hint_obj);
            lv_anim_set_exec_cb(&back_hint_release_anim,
                                set_gesture_detect_hint_obj_opa);
            lv_anim_set_completed_cb(&back_hint_release_anim,
                                     hidden_back_hint_release_anim_cb);
            lv_anim_start(&back_hint_release_anim);
#ifdef SkaiwalkWatchOS
            extern void lvgl_set_global_keypad_esc_cmd(void);
            lvgl_set_global_keypad_esc_cmd();
#endif
        }
        else
        {
            lv_anim_init(&back_hint_release_anim);
            lv_anim_set_duration(&back_hint_release_anim, 100);
            lv_anim_set_values(&back_hint_release_anim, drag_offset, 0);
            lv_anim_set_var(&back_hint_release_anim, gesture_detect_hint_obj);
            lv_anim_set_exec_cb(&back_hint_release_anim, back_hint_anim_cb);
            lv_anim_set_completed_cb(&back_hint_release_anim,
                                     hidden_back_hint_release_anim_cb);
            lv_anim_start(&back_hint_release_anim);
        }

        /* v8 had a second `else if (RELEASED || PRESS_LOST)` after this one,
           which RELEASED could never reach. Fold the teardown in here so it
           actually runs, and give PRESS_LOST its own branch below. */
        if (gesture_is_active)
            gui_app_manual_animation_stop(x_2_process(obj_left_edge(obj)));

        lvsf_gesture_bars_realign();
        gesture_is_active = false;
        gesture_enable_update(gesture_is_enabled);
    }
    else if (event == LV_EVENT_PRESS_LOST)
    {
        if (gesture_is_active)
            gui_app_manual_animation_stop(x_2_process(obj_left_edge(obj)));

        lvsf_gesture_bars_realign();
        gesture_is_active = false;
        gesture_enable_update(gesture_is_enabled);
    }
}

void lvsf_gesture_init(lv_obj_t *parent)
{
    lv_obj_t *bar;

    LOG_D("lvsf_gesture_init: parent=%p hor=%d ver=%d",
          parent, (int)LV_HOR_RES_MAX, (int)LV_VER_RES_MAX);
    memset(gesture_detect_objs, 0, sizeof(gesture_detect_objs));
    memset(gesture_img_objs, 0, sizeof(gesture_img_objs));
    lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    bar = lv_obj_create(parent);
    lv_obj_add_event_cb(bar, left_bar_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bar, lv_palette_main(LV_PALETTE_RED),
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    gesture_detect_objs[0] = bar;

    gesture_detect_hint_obj = lv_obj_create(parent);
    lv_obj_set_size(gesture_detect_hint_obj, LVSF_GESTURE_BACK_HINT_SIZE,
                    LVSF_GESTURE_BACK_HINT_SIZE);
    lv_obj_align(gesture_detect_hint_obj, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_flag(gesture_detect_hint_obj, LV_OBJ_FLAG_HIDDEN);

    gesture_detect_hint_icon = lv_image_create(gesture_detect_hint_obj);
    lv_image_set_src(gesture_detect_hint_icon, &img_left_arrow);
    lv_obj_align(gesture_detect_hint_icon, LV_ALIGN_CENTER, 0, 0);

    lvsf_gesture_bars_realign();
}

void lvsf_gesture_deinit(void)
{
    for (uint32_t i = 0; i < 4; i++)
    {
        if (gesture_detect_objs[i])
            lv_obj_delete(gesture_detect_objs[i]);
    }
    memset(gesture_detect_objs, 0, sizeof(gesture_detect_objs));
    memset(gesture_img_objs, 0, sizeof(gesture_img_objs));
}

void lvsf_gesture_set_image(uint32_t idx, const void *src_img)
{
    gesture_img_objs[idx] = lv_image_create(gesture_detect_objs[idx]);
    lv_image_set_src(gesture_img_objs[idx], src_img);
    lv_obj_add_flag(gesture_img_objs[idx], LV_OBJ_FLAG_HIDDEN);
    lvsf_gesture_bars_realign();
}

void display_gesture_detect_objs(uint32_t idx, bool display)
{
    if (idx < 4 && lv_obj_is_valid(gesture_detect_objs[idx]))
    {
        LOG_D("display_gesture_detect_objs idx=%d display=%d", idx, display);
        if (display)
            lv_obj_remove_flag(gesture_detect_objs[idx], LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(gesture_detect_objs[idx], LV_OBJ_FLAG_HIDDEN);
    }
}

void lvsf_gesture_bring_to_front(void)
{
    /* Raise the edge-back detector bars AND the drag hint to the front of their
       parent (lv_layer_top). A later full-screen lv_layer_top overlay is created
       above them, so without this the overlay swallows the swipe and hides the
       hint. Hint first, so the transparent detector stays topmost for input. */
    if (gesture_detect_hint_obj && lv_obj_is_valid(gesture_detect_hint_obj))
        lv_obj_move_foreground(gesture_detect_hint_obj);
    for (int i = 0; i < 4; i++)
    {
        if (gesture_detect_objs[i] && lv_obj_is_valid(gesture_detect_objs[i]))
            lv_obj_move_foreground(gesture_detect_objs[i]);
    }
}

void lvsf_gesture_disable(void)
{
    gesture_is_enabled = false;
    if (!gesture_is_active)          /* leave them alone mid-gesture */
        gesture_enable_update(gesture_is_enabled);
}

void lvsf_gesture_enable(void)
{
    gesture_is_enabled = true;
    if (!gesture_is_active)
        gesture_enable_update(gesture_is_enabled);
}

#else /* no GUI_APP_FRAMEWORK, or transitions disabled */

uint8_t lvsf_gesture_enable_register(uint8_t enable)
{
    LV_UNUSED(enable);
    return 0;
}

void lvsf_gesture_bars_realign(void)      { }
void lvsf_gesture_init(lv_obj_t *parent)  { LV_UNUSED(parent); }
void lvsf_gesture_deinit(void)            { }
void lvsf_gesture_bring_to_front(void)    { }
void lvsf_gesture_disable(void)           { }
void lvsf_gesture_enable(void)            { }

void lvsf_gesture_set_image(uint32_t idx, const void *src_img)
{
    LV_UNUSED(idx);
    LV_UNUSED(src_img);
}

void display_gesture_detect_objs(uint32_t idx, bool display)
{
    LV_UNUSED(idx);
    LV_UNUSED(display);
}

#endif /* GUI_APP_FRAMEWORK && !APP_TRANS_ANIMATION_NONE */
