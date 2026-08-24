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
    // app_id_mouse,   // entry moved to media page (right swipe)
#endif
#ifdef APP_ID_TV_REMOTE
    app_id_tv_remote,
#endif
#ifdef APP_ID_PHOTO
    app_id_photo,
#endif
#ifdef APP_ID_SKAIAPP
    app_id_skaiapp,
#endif
};

#define APP_LIST_COUNT (sizeof(APP_LIST_ITEMS) / sizeof(APP_LIST_ITEMS[0]))


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
#ifdef APP_ID_TV_REMOTE
    case app_id_tv_remote:   return APP_ID_TV_REMOTE;
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
#ifdef APP_ID_SKAIAPP
    case app_id_skaiapp:     return APP_ID_SKAIAPP;
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
#ifdef APP_ID_TV_REMOTE
    /* TODO(asset): reusing the media glyph — no TV icon in the resource pack
       yet. app_id_media is commented out of this grid, so it reads unambiguously
       for now, but a real remote/TV icon should replace it. */
    case app_id_tv_remote:   return IMG_ITUNES;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:       return IMG_PHOTO;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur: return IMG_GAME;
#endif
#ifdef APP_ID_SKAIAPP
    case app_id_skaiapp:     return IMG_LOGO;
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


/* ADR-0020:App List 以「區段」形式嵌進控制中心的捲動容器(下方上拉頁)。
   自己不捲動 —— 高度 = 內容高,捲動交給外層 parent;也不掛 arc_scroll
   (外層是複合頁,弧捲的 slot 對位在這裡沒有意義,先用觸控捲動)。 */
lv_obj_t *lv_app_list_layout_create_embedded(lv_obj_t *parent, lv_coord_t y_top)
{
    uint8_t count = APP_LIST_COUNT;
    uint8_t rows = (count + APP_LIST_COLS - 1) / APP_LIST_COLS;

    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_remove_style_all(container);
    lv_obj_set_size(container, LV_HOR_RES, rows * APP_LIST_CELL_H + 40);
    lv_obj_set_pos(container, 0, y_top);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t row = i / APP_LIST_COLS;
        uint8_t col = i % APP_LIST_COLS;

        lv_obj_t *cell = lv_obj_create(container);
        lv_obj_remove_style_all(cell);
        lv_obj_set_size(cell, APP_LIST_CELL_W, APP_LIST_CELL_H);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(cell, APP_LIST_PAD_H + col * APP_LIST_CELL_W,
                       row * APP_LIST_CELL_H);

        lv_obj_t *icon = lv_img_create(cell);
        lv_img_set_src(icon, get_app_list_icon(APP_LIST_ITEMS[i]));
        lv_img_set_zoom(icon, 330);
        lv_obj_center(icon);

        lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
        /* 垂直軸交還外層,拖曳從 cell 上開始也能捲動整頁。 */
        lv_obj_add_flag(cell, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
        lv_obj_add_event_cb(cell, app_list_item_click_cb, LV_EVENT_CLICKED,
                            (void *)get_app_id_str(APP_LIST_ITEMS[i]));
    }

    LOG_I("App list embedded: %d apps at y=%d", count, (int)y_top);
    return container;
}

