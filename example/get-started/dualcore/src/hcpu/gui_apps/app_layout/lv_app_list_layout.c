/**
 ******************************************************************************
 * @file   lv_app_list_layout.c
 * @author Skaiwalk software development team
 * @brief  App grid list (3 icons per row) displayed below the instruction list
 ******************************************************************************
 */

#include "lvgl.h"
#include "lv_ext_resource_manager.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "ui_handler.h"
#include "bsp_board.h"   /* kReleaseMode */
#include "ui_img_helper.h"
#include "arc_scroll.h"
#include <string.h>

#define DBG_TAG "app_list.layout"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define APP_LIST_COLS       3
#define APP_LIST_PAD_H      40  /* horizontal inset from each side */
#define APP_LIST_CELL_W     ((LV_HOR_RES - 2 * APP_LIST_PAD_H) / APP_LIST_COLS)
#define APP_LIST_CELL_H     APP_LIST_CELL_W
#define APP_LIST_PAD_TOP    20
#define APP_LIST_SLOT_ANGLE_DEG 30 /* 弧形滑過 30° = 一行 */
#define APP_LIST_BAND_THICKNESS 90
#define APP_LIST_EXTRA_SCROLL APP_LIST_CELL_H /* 拉到底之後可再往下一個 cell */
/* 466x466 round display: the LAST grid row settles with its bottom at this screen Y
   at scroll-end. Tuned so the bottom row's icons (80px source @129% ≈ 103px, r≈52)
   sit fully inside the circle — for col0/col2 that means the row centre stays within
   ~[106,360] of the display centre. 415 puts the last row centre at ~351 (≈9px inside
   the limit) while keeping the empty band below it small (~51px); lower values cut the
   corner icon, much higher values leave too big a gap under the grid. */
#define APP_LIST_BOTTOM_VISIBLE 415

/*******************************************************************************
 * Configure which apps to show in the grid.
 * Comment / uncomment lines to add or remove apps.
 ******************************************************************************/
static const uint16_t APP_LIST_ITEMS[] = {
#ifdef APP_ID_TIMER
    app_id_timer,
#endif
    app_id_flashlight,
#ifdef APP_ID_RECORDER
    app_id_recorder,
#endif
    app_id_exercise,
#ifdef APP_ID_SLEEP
    app_id_sleep,
#endif
#ifdef APP_ID_CALCULATOR
    app_id_calculator,
#endif
#ifdef APP_ID_WEATHER
    app_id_weather,
#endif
#ifdef APP_ID_MEDIA
    // app_id_media,
#endif
#ifdef APP_ID_ALARM
    app_id_alarm,
#endif
#ifdef APP_ID_SETTING
    app_id_setting,
#endif
#ifdef APP_ID_MOUSE
    app_id_mouse,
#endif
#ifdef APP_ID_PHOTO
    app_id_photo,
#endif
};

#define APP_LIST_COUNT (sizeof(APP_LIST_ITEMS) / sizeof(APP_LIST_ITEMS[0]))

/* The scrollable grid container, kept so the page can be reset to its top row on
   re-entry (lv_app_list_layout_reset_scroll). */
static lv_obj_t *s_app_list_container = NULL;

static const char *get_app_id_str(uint16_t app_id)
{
    switch (app_id)
    {
    case app_id_flashlight:  return APP_ID_FLASHLIGHT;
#ifdef APP_ID_RECORDER
    case app_id_recorder:    return APP_ID_RECORDER;
#endif
    case app_id_exercise:    return APP_ID_EXERCISE;
#ifdef APP_ID_SLEEP
    case app_id_sleep:       return APP_ID_SLEEP;
#endif
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:  return APP_ID_CALCULATOR;
#endif
#ifdef APP_ID_WEATHER
    case app_id_weather:     return APP_ID_WEATHER;
#endif
#ifdef APP_ID_MEDIA
    case app_id_media:       return APP_ID_MEDIA;
#endif
#ifdef APP_ID_TIMER
    case app_id_timer:       return APP_ID_TIMER;
#endif
#ifdef APP_ID_ALARM
    case app_id_alarm:       return APP_ID_ALARM;
#endif
#ifdef APP_ID_SETTING
    case app_id_setting:     return APP_ID_SETTING;
#endif
#ifdef APP_ID_MOUSE
    case app_id_mouse:       return APP_ID_MOUSE;
#endif
#ifdef APP_ID_TOUCHPAD
    case app_id_touchpad:    return APP_ID_TOUCHPAD;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:       return APP_ID_PHOTO;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur: return APP_ID_GAME_DINOSAUR;
#endif
    default:                 return APP_ID_MAIN;
    }
}

static const char *get_app_list_icon(uint16_t app_id)
{
    switch (app_id)
    {
    case app_id_flashlight:  return IMG_FLASHLIGHT;
#ifdef APP_ID_RECORDER
    case app_id_recorder:    return IMG_RECORDER;
#endif
    case app_id_exercise:    return IMG_WORKOUT;
#ifdef APP_ID_SLEEP
    case app_id_sleep:       return IMG_SLEEP;
#endif
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:  return IMG_CALCULATOR;
#endif
#ifdef APP_ID_WEATHER
    case app_id_weather:     return IMG_GROUP;
#endif
#ifdef APP_ID_MEDIA
    case app_id_media:       return IMG_ITUNES;
#endif
#ifdef APP_ID_TIMER
    case app_id_timer:       return IMG_ALARM_2;
#endif
#ifdef APP_ID_ALARM
    case app_id_alarm:       return IMG_ALARM;
#endif
#ifdef APP_ID_SETTING
    case app_id_setting:     return IMG_SETTINGS_APP;
#endif
#ifdef APP_ID_MOUSE
    case app_id_mouse:       return IMG_MOUSE;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:       return IMG_PHOTO;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur: return IMG_GAME;
#endif
    default:                 return IMG_LOGO;
    }
}

static void app_list_item_click_cb(lv_event_t *evt)
{
    const char *app_id_str = (const char *)evt->user_data;
    LOG_I("App list click: %s", app_id_str);
    animate_to_home_from_instruction_list();
    gui_app_run(app_id_str);
}

static lv_obj_t *find_cell_at_point(lv_obj_t *container, lv_point_t pt)
{
    if (container == NULL || !lv_obj_is_valid(container)) return NULL;
    uint32_t cnt = lv_obj_get_child_cnt(container);
    for (uint32_t i = 0; i < cnt; i++)
    {
        lv_obj_t *cell = lv_obj_get_child(container, i);
        if (cell == NULL || !lv_obj_is_valid(cell)) continue;
        if (lv_obj_has_flag(cell, LV_OBJ_FLAG_HIDDEN)) continue;
        lv_area_t a;
        lv_obj_get_coords(cell, &a);
        if (pt.x >= a.x1 && pt.x <= a.x2 && pt.y >= a.y1 && pt.y <= a.y2)
        {
            return cell;
        }
    }
    return NULL;
}

static lv_obj_t *app_list_arc_tap_cb(lv_point_t press_point, void *ctx)
{
    return find_cell_at_point((lv_obj_t *)ctx, press_point);
}

/* Bottom edge (natural inner Y) of the lowest interactive cell = the grid's content
 * height. Only CLICKABLE children count — cells carry that flag, spacers/non-widgets
 * don't. lv_obj_get_y already folds in scroll compensation (see lv_obj_pos.c), so this
 * is the true inner Y; don't add scroll_y. */
static lv_coord_t app_list_content_bottom(lv_obj_t *container)
{
    if (container == NULL || !lv_obj_is_valid(container)) return 0;
    lv_coord_t max_bottom = 0;
    uint32_t cnt = lv_obj_get_child_cnt(container);
    for (uint32_t i = 0; i < cnt; i++)
    {
        lv_obj_t *c = lv_obj_get_child(container, i);
        if (c == NULL || !lv_obj_is_valid(c)) continue;
        if (lv_obj_has_flag(c, LV_OBJ_FLAG_HIDDEN)) continue;
        if (!lv_obj_has_flag(c, LV_OBJ_FLAG_CLICKABLE)) continue;
        lv_coord_t bottom = lv_obj_get_y(c) + lv_obj_get_height(c);
        if (bottom > max_bottom) max_bottom = bottom;
    }
    return max_bottom;
}

static lv_coord_t app_list_max_scroll(lv_obj_t *container)
{
    /* Cap the scroll so that at the scroll-end the bottom row's bottom edge rests at
       APP_LIST_BOTTOM_VISIBLE on the round screen — clear of the bottom-corner curve
       that was cutting the rightmost icon. snap_cb pairs with this: it pulls the last
       row up to here the instant the row scrolls fully into view (see below). */
    lv_coord_t max_bottom = app_list_content_bottom(container);
    if (max_bottom <= APP_LIST_BOTTOM_VISIBLE) return 0;
    return max_bottom - APP_LIST_BOTTOM_VISIBLE;
}

static lv_obj_t *app_list_arc_snap_cb(void *ctx)
{
    /* grid 沒有 selected item 概念：把 overshoot 拉回 [0, max]，並在最後一行整個
     * 滑進畫面底部時 snap 到 max，避免它停在被圓角削掉的最底。直接呼叫
     * lv_obj_scroll_to_y、回傳 NULL 讓 released_cb 不再動 */
    lv_obj_t *container = (lv_obj_t *)ctx;
    if (container == NULL || !lv_obj_is_valid(container)) return NULL;
    lv_coord_t cur = lv_obj_get_scroll_y(container);
    lv_coord_t max_scroll = app_list_max_scroll(container);
    if (cur < 0)
    {
        lv_obj_scroll_to_y(container, 0, LV_ANIM_ON);
    }
    else if (cur > max_scroll)
    {
        lv_obj_scroll_to_y(container, max_scroll, LV_ANIM_ON);
    }
    else if (max_scroll > 0 && cur < max_scroll &&
             (app_list_content_bottom(container) - cur) <= LV_VER_RES)
    {
        /* 466x466 round display: the instant the LAST row's bottom edge has scrolled
           fully onto the screen (content_bottom - cur <= screen height) it is still
           sitting LOW in the clipped bottom corner — the user stops here thinking it's
           the bottom and never reaches the cleared max. So snap fully to max: the last
           row lands with its bottom at APP_LIST_BOTTOM_VISIBLE, well inside the circle.
           The upper view (last row still partly below the screen) doesn't trigger. */
        lv_obj_scroll_to_y(container, max_scroll, LV_ANIM_ON);
    }
    return NULL;
}

static void app_list_arc_bounds_cb(lv_coord_t *out_min, lv_coord_t *out_max,
                                   void *ctx)
{
    lv_obj_t *container = (lv_obj_t *)ctx;
    if (container == NULL || !lv_obj_is_valid(container))
    {
        *out_min = 0;
        *out_max = 0;
        return;
    }
    *out_min = 0;
    *out_max = app_list_max_scroll(container);
}

lv_obj_t *lv_app_list_layout_create(lv_obj_t *parent)
{
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_style_pad_all(container, 0, 0);
    lv_obj_align(container, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(container, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_top(container, APP_LIST_PAD_TOP, 0);
    /* CRITICAL for the round display: the container is natively SCROLLABLE, so LVGL
       clamps lv_obj_scroll_to_y() to [0, content_height - viewport]. Without this the
       scroll bottoms out with the last grid row sitting at the very screen edge, where
       the round corner cuts the rightmost icon — and our snap-to-cleared-position is
       clamped straight back, so nothing moves. Extend the scrollable area by exactly
       (screen - APP_LIST_BOTTOM_VISIBLE) so the native bottom == app_list_max_scroll:
       scrolling to the bottom then lands with the last row's bottom edge at
       APP_LIST_BOTTOM_VISIBLE, clear of the corner. */
    lv_obj_set_style_pad_bottom(container, LV_VER_RES - APP_LIST_BOTTOM_VISIBLE, 0);

    uint8_t count = APP_LIST_COUNT;
    uint8_t rows = (count + APP_LIST_COLS - 1) / APP_LIST_COLS;

    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t row = i / APP_LIST_COLS;
        uint8_t col = i % APP_LIST_COLS;

        lv_obj_t *cell = lv_obj_create(container);
        lv_obj_set_size(cell, APP_LIST_CELL_W, APP_LIST_CELL_H);
        lv_obj_set_style_bg_opa(cell, LV_OPA_0, 0);
        lv_obj_set_style_border_width(cell, 0, 0);
        lv_obj_set_style_pad_all(cell, 0, 0);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);

        lv_coord_t x = APP_LIST_PAD_H + col * APP_LIST_CELL_W;
        lv_coord_t y = row * APP_LIST_CELL_H + APP_LIST_PAD_TOP;
        lv_obj_set_pos(cell, x, y);

        /* Icon (no label) */
        lv_obj_t *icon = lv_img_create(cell);
        lv_img_set_src(icon, get_app_list_icon(APP_LIST_ITEMS[i]));
        lv_img_set_zoom(icon, 330); /* 256=100%, 330≈129% */
        lv_obj_center(icon);

        /* Click handler */
        lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(cell, app_list_item_click_cb, LV_EVENT_CLICKED,
                            (void *)get_app_id_str(APP_LIST_ITEMS[i]));
    }

    /* 右側弧形觸控滾動：grid 用 row 當 slot，bounds_cb 提供 [0, max] 邊界
     * （centered-list 公式不適用），tap_cb 把右欄 cell 的點擊轉發給 cell */
    arc_scroll_config_t arc_cfg = {0};
    arc_cfg.parent = parent;
    arc_cfg.list = container;
    arc_cfg.slot_height_px = APP_LIST_CELL_H;
    arc_cfg.item_height_px = APP_LIST_CELL_H;
    arc_cfg.slot_angle_deg = APP_LIST_SLOT_ANGLE_DEG;
    arc_cfg.item_count = rows;
    arc_cfg.band_thickness = APP_LIST_BAND_THICKNESS;
    arc_cfg.lock_ancestors = true;
    arc_cfg.tap_cb = app_list_arc_tap_cb;
    arc_cfg.snap_cb = app_list_arc_snap_cb;
    arc_cfg.bounds_cb = app_list_arc_bounds_cb;
    arc_cfg.ctx = container;
    arc_scroll_create(&arc_cfg);

    s_app_list_container = container;
    LOG_I("App list created with %d apps, %d rows", count, rows);
    return container;
}

/* Reset the app grid back to its top row. Called when the App List page is about
   to be revealed (the right-edge pull) so re-entering it always starts at the top
   instead of wherever the last visit left it scrolled. */
void lv_app_list_layout_reset_scroll(void)
{
    if (s_app_list_container && lv_obj_is_valid(s_app_list_container))
        lv_obj_scroll_to_y(s_app_list_container, 0, LV_ANIM_OFF);
}
