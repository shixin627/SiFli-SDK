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
#include "ui_img_helper.h"
#include <string.h>

#define DBG_TAG "app_list.layout"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define APP_LIST_COLS       3
#define APP_LIST_PAD_H      40  /* horizontal inset from each side */
#define APP_LIST_CELL_W     ((LV_HOR_RES - 2 * APP_LIST_PAD_H) / APP_LIST_COLS)
#define APP_LIST_CELL_H     APP_LIST_CELL_W

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
#ifdef APP_ID_CALCULATOR
    app_id_calculator,
#endif
#ifdef APP_ID_CALENDAR
    app_id_calendar,
#endif
#ifdef APP_ID_WEATHER
    app_id_weather,
#endif
#ifdef APP_ID_MEDIA
    app_id_media,
#endif
#ifdef APP_ID_HEART_RATE
    // app_id_heart_rate,
#endif
#ifdef APP_ID_ACTIVITY
    // app_id_activity,
#endif
#ifdef APP_ID_ALARM
    // app_id_alarm,
#endif
#ifdef APP_ID_SETTING
    app_id_setting,
#endif
#ifdef APP_ID_MOUSE
    // app_id_mouse,
#endif
#ifdef APP_ID_TOUCHSCREEN
    app_id_touchscreen,
#endif
#ifdef APP_ID_TOUCHPAD
    app_id_touchpad,
#endif
#ifdef APP_ID_PHOTO
    app_id_photo,
#endif
#ifdef APP_ID_GAME_DINOSAUR
    // app_id_game_dinosaur,
#endif
#ifdef APP_ID_NOTE_CHATROOM
    // app_id_note,
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
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:  return APP_ID_CALCULATOR;
#endif
#ifdef APP_ID_CALENDAR
    case app_id_calendar:    return APP_ID_CALENDAR;
#endif
#ifdef APP_ID_WEATHER
    case app_id_weather:     return APP_ID_WEATHER;
#endif
#ifdef APP_ID_MEDIA
    case app_id_media:       return APP_ID_MEDIA;
#endif
#ifdef APP_ID_HEART_RATE
    case app_id_heart_rate:  return APP_ID_HEART_RATE;
#endif
#ifdef APP_ID_ACTIVITY
    case app_id_activity:    return APP_ID_ACTIVITY;
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
#ifdef APP_ID_TOUCHSCREEN
    case app_id_touchscreen: return APP_ID_TOUCHSCREEN;
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
#ifdef APP_ID_NOTE_CHATROOM
    case app_id_note:        return APP_ID_NOTE_CHATROOM;
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
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:  return IMG_CALCULATOR;
#endif
#ifdef APP_ID_CALENDAR
    case app_id_calendar:    return IMG_CALENDAR;
#endif
#ifdef APP_ID_WEATHER
    case app_id_weather:     return IMG_GROUP;
#endif
#ifdef APP_ID_MEDIA
    case app_id_media:       return IMG_ITUNES;
#endif
#ifdef APP_ID_HEART_RATE
    case app_id_heart_rate:  return IMG_HEART_RATE;
#endif
#ifdef APP_ID_ACTIVITY
    case app_id_activity:    return IMG_ACTIVITY;
#endif
#ifdef APP_ID_TIMER
    case app_id_timer:       return IMG_ALARM_2;
#endif
#ifdef APP_ID_ALARM
    case app_id_alarm:       return IMG_ALARM;
#endif
#ifdef APP_ID_SETTING
    case app_id_setting:     return IMG_SETTINGS;
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
#ifdef APP_ID_NOTE_CHATROOM
    case app_id_note:        return IMG_NOTE;
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
    lv_obj_set_style_pad_top(container, 20, 0);

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
        lv_coord_t y = row * APP_LIST_CELL_H + 20;
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

    LOG_I("App list created with %d apps, %d rows", count, rows);
    return container;
}
