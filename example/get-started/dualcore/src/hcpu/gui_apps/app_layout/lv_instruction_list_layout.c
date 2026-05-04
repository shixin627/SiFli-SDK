/**
 ******************************************************************************
 * @file   lv_instruction_list_layout.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "lvgl.h"
#include "lv_simplified_obj.h"
#include "lv_ext_resource_manager.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "watch_system_interact.h"
#include "custom_trans_anim.h"
#include <math.h>
#include "ui_helper.h"
#include "ui_img_helper.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#include "ui_handler.h"
#ifdef BSP_USING_GESTURE_HANDLER
    #include "gesture_handler.h"
#endif
#include "watch_global_data.h"
#ifdef BSP_USING_BLOC
    #include "bloc_v2t.h"
    #include "bloc_peripheral.h"
    #include "bloc_weather.h"
    #include "bloc_calendar.h"
    #include "bloc_motion_tracking.h"
#endif
#include "communicate_protocol.h"
#include "communicate_task.h"
#include <cJSON.h>

LV_IMG_DECLARE(voice_group);
LV_IMG_DECLARE(menu_icon);

#define DBG_TAG "instruction.list.layout"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define APP_ID "instruction_list"
#include <stdio.h>
#include <stdint.h>

/*******************************************************************************
 * Unified List Item
 ******************************************************************************/
#define MAX_LIST_ITEMS 30
#define LIST_ITEM_ID_LEN 64
#define LIST_ITEM_TITLE_LEN 64

typedef struct
{
    char id[LIST_ITEM_ID_LEN]; // app_id string or instruction UUID
    char title[LIST_ITEM_TITLE_LEN];
    const char *icon;      // icon resource pointer, can be NULL
    char img_path[64];     // file-based image path for instructions
    lv_obj_t *widget;      // app widget obj, NULL for instructions
    bool is_instruction;   // true = custom instruction, false = app
    bool is_interval;      // for instructions: has toggle switch
    bool enabled;          // toggle state
    uint32_t interval_sec; // intervalSeconds
    char trigger_type[32]; // e.g. "interval", "once", etc.
    uint32_t version;      // version from server
} list_item_t;

static list_item_t list_items[MAX_LIST_ITEMS];
static uint8_t list_item_count =
    0; // total count of all items (app + instructions)
static uint8_t app_base_count =
    0; // number of app items loaded from INSTRUCTION_LIST_ITEMS_DEFINITION

/* Callback: tapped or toggled. Receives id string and enabled state. */
static void (*instruction_tap_cb)(const char *id, bool enabled) = NULL;

void set_custom_instruction_tap_cb(void (*cb)(const char *id, bool enabled))
{
    instruction_tap_cb = cb;
}

static lv_obj_t *switch_objs[MAX_LIST_ITEMS]; // toggle switches for any item

#define LIST_ITEM_WIDTH (80)
#define LIST_ITEM_HEIGHT (80)
#define LIST_ITEM_SPACING (-100)
#define LIST_ITEM_BTN_WIDTH (150)
#define LIST_ITEM_BTN_HEIGHT (150)
#define LIST_ITEM_WIDGET_HEIGHT (200)
#define LIST_ITEM_WIDGET_WIDTH (430)

#define LIST_RADIUS (466)
#define LIST_ADJUST_X_POSITION (-466)

#define MOVABLE_ARC_RADIUS (233)
#define MOVABLE_ARC_CENTER_X (233)
#define MOVABLE_ARC_CENTER_Y (233)
#define MOVABLE_ARC_START_ANGLE (-15)
#define MOVABLE_ARC_END_ANGLE (15)
#define MOVABLE_ARC_WIDTH (3)

#define LIST_ITEM_RADIUS (240)
#define LIST_ITEM_BORDER_SIDE LV_BORDER_SIDE_RIGHT

#define DOT_SMOLL_PROPORTION (0.6)
#define DOT_BIG_PROPORTION (1.3)
#define DOT_BG_SIZE (100 * DOT_BIG_PROPORTION) + 2

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// LV_IMG_DECLARE(img_flashlight);
// LV_IMG_DECLARE(img_activity);
LV_IMG_DECLARE(weather);
// LV_IMG_DECLARE(img_calculator);
// LV_IMG_DECLARE(img_calendar);
// LV_IMG_DECLARE(img_messages);
// LV_IMG_DECLARE(img_note);
// LV_IMG_DECLARE(img_alarm);
LV_IMG_DECLARE(message_bar);
// LV_IMG_DECLARE(img_mouse);
// LV_IMG_DECLARE(img_touchscreen);
// LV_IMG_DECLARE(img_touchpad);
// LV_IMG_DECLARE(img_logo);
// LV_IMG_DECLARE(img_photo);
// LV_IMG_DECLARE(img_workout);
// LV_IMG_DECLARE(img_recorder);
// LV_IMG_DECLARE(small_img_logo_matting);
LV_IMG_DECLARE(select_prompt);
LV_IMG_DECLARE(icon_release);
LV_IMG_DECLARE(app_icon_frame);
// LV_IMG_DECLARE(img_messages);

uint16_t INSTRUCTION_LIST_ITEMS_DEFINITION[] = {
#ifdef APP_ID_TIMER
    app_id_timer,
#endif
    app_id_flashlight,
#ifdef APP_ID_CALCULATOR
// app_id_calculator,
#endif
    // app_id_exercise,
    // app_id_heart_rate,
    app_id_recorder,
#ifdef APP_ID_ACTIVITY
// app_id_activity,
#endif
// app_id_calendar,
#ifdef APP_ID_TOUCHSCREEN
    app_id_touchscreen,
#endif
#ifdef APP_ID_PHOTO
// app_id_photo,
#endif
#ifdef APP_ID_TOUCHPAD
    app_id_touchpad,
#endif
// app_id_weather,
#ifdef APP_ID_IOT_GATE
// app_id_iot_gate,
#endif
#ifdef APP_ID_GAME_DINOSAUR
// app_id_game_dinosaur,
#endif
#ifdef APP_ID_MEDIA
// app_id_media,
#endif
    // app_id_ai,
};

uint8_t return_app_count(void)
{
    return app_base_count;
}

uint8_t return_total_list_count(void)
{
    return list_item_count;
}

typedef struct
{
    lv_obj_t *list;
    lv_obj_t *p_instruction_list_bg;
    lv_obj_t *p_instruction_list_ai_bg;
    lv_obj_t *p_instruction_list_ai_icon;
    lv_obj_t *p_app_indicator_btn[MAX_LIST_ITEMS];
    lv_obj_t *indicator_dots[MAX_LIST_ITEMS];
    lv_obj_t *indicator_dots_bg[MAX_LIST_ITEMS];
    lv_obj_t *movable_range_arc; // 可移動範圍圓弧線
    lv_obj_t
        *app_list_tileview;  // vertical tileview: instruction list + app grid
    lv_obj_t *app_list_tile; // tile 1: app grid page
} instruction_list_layout_t;
static instruction_list_layout_t *p_instruction_list_layout;
static bool created = false;

static bool pause_instruction_list = true;

const char *get_app_icon(uint8_t app_id)
{
    switch (app_id)
    {
#ifdef APP_ID_SKAI
    case app_id_ai:
        return IMG_LOGO;
#endif
#ifdef APP_ID_RECORDER
    case app_id_recorder:
        return IMG_RECORDER;
#endif
    case app_id_calendar:
        return IMG_CALENDAR;
    case app_id_weather:
        return IMG_GROUP;
    case app_id_exercise:
        return IMG_WORKOUT;
    case app_id_flashlight:
        return IMG_FLASHLIGHT;
#ifdef APP_ID_MEDIA
    case app_id_media:
        return IMG_ITUNES;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur:
        return IMG_GAME;
#endif
#ifdef APP_ID_IOT_GATE
    case app_id_iot_gate:
        return IMG_GAME;
#endif
#ifdef APP_ID_HEART_RATE
    case app_id_heart_rate:
        return IMG_HEART_RATE;
#endif
#ifdef APP_ID_ACTIVITY
    case app_id_activity:
        return IMG_ACTIVITY;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:
        return IMG_PHOTO;
#endif
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:
        return IMG_CALCULATOR;
#endif
#ifdef APP_ID_TIMER
    case app_id_timer:
        return IMG_ALARM_2;
#endif
#ifdef APP_ID_ALARM
    case app_id_alarm:
        return IMG_ALARM;
#endif
#ifdef APP_ID_SETTING
    case app_id_setting:
        return IMG_SETTINGS;
#endif
#ifdef APP_ID_MESSAGE_LIST
    case app_id_message_list:
        return IMG_MESSAGES;
#endif
#ifdef APP_ID_MOUSE
    case app_id_mouse:
        return IMG_MOUSE;
#endif
#ifdef APP_ID_TOUCHSCREEN
    case app_id_touchscreen:
        return IMG_TOUCHSCREEN;
#endif
#ifdef APP_ID_TOUCHPAD
    case app_id_touchpad:
        return IMG_TOUCHPAD;
#endif
    default:
        return IMG_LOGO;
    }
}

typedef struct
{
    lv_obj_t *right;
    lv_obj_t *left;
    lv_obj_t *top;
    lv_obj_t *bottom;
    lv_obj_t *right_img;
    lv_obj_t *left_img;
    lv_obj_t *top_img;
    lv_obj_t *bottom_img;
} quick_open_app_t;

void load_instruction_list(void);

static const lv_style_const_prop_t LIST_ITEM_STYLE_PROPS[] = {
    LV_STYLE_CONST_WIDTH(LIST_ITEM_WIDTH),
    LV_STYLE_CONST_HEIGHT(LIST_ITEM_HEIGHT),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t LIST_ITEM_TITLE_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_14),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(LIST_ITEM_STYLE, LIST_ITEM_STYLE_PROPS);
LV_STYLE_CONST_INIT(LIST_ITEM_TITLE_STYLE, LIST_ITEM_TITLE_STYLE_PROPS);

static void set_icon_y(lv_obj_t *obj, lv_coord_t y)
{
    lv_obj_align(obj, LV_ALIGN_RIGHT_MID, -20, y);
}

static void animate_icon_vertical(lv_obj_t *obj, bool move_up)
{
    lv_coord_t end_y = move_up ? -30 : 30;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_values(&a, 0, end_y);
    lv_anim_set_time(&a, 200);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)set_icon_y);
    lv_anim_start(&a);
}

static void set_label_y(lv_obj_t *obj, lv_coord_t y)
{
    lv_obj_align(obj, LV_ALIGN_CENTER, 0, y);
}

static void animate_label_vertical(lv_obj_t *obj, bool move_up)
{
    lv_coord_t end_y = move_up ? -30 : 30;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_values(&a, 0, end_y);
    lv_anim_set_time(&a, 200);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)set_label_y);
    lv_anim_start(&a);
}

lv_obj_t *app_icon_shadow[MAX_LIST_ITEMS];
static bool is_indicator_dots_visible = true;
static uint16_t selected_item_index = 0;
static uint16_t last_zoom[MAX_LIST_ITEMS] = {0};
static void update_indicator_dots_position(int input_value)
{
    // LOG_I("Updating indicator dots position, input value: %d", input_value);
    if (p_instruction_list_layout == NULL || !is_indicator_dots_visible)
        return;

    int total_dots = list_item_count;
    if (total_dots <= 0)
        return;

    // LOG_I("Updating indicator dots position, input value: %d",
    //       input_value);

    const int circle_radius = 300;
    const int center_x = 120;
    const int center_y = 233;

    const float angle_per_dot = 27.0f;

    float base_input = 63.0f;
    float degrees_per_200_input = angle_per_dot;
    float totlal_input_range = 100 * total_dots;

    float offset_angle =
        ((totlal_input_range - (float)input_value) - base_input) /
        (float)(totlal_input_range / total_dots) * degrees_per_200_input;

    for (int i = 0; i < total_dots; i++)
    {
        if (p_instruction_list_layout->indicator_dots[i] == NULL)
            continue;

        float base_angle = i * angle_per_dot;
        float current_angle = base_angle - offset_angle;

        while (current_angle < 0)
        {
            current_angle += 360.0f;
        }
        while (current_angle >= 360.0f)
        {
            current_angle -= 360.0f;
        }

        float angle_rad = current_angle * M_PI / 180.0f;

        int dot_x = center_x + (int)(circle_radius * cos(angle_rad));
        int dot_y = center_y + (int)(circle_radius * sin(angle_rad));

        /* 畫面為 466x466 圓形，指示點中心距離螢幕中心超過 (半徑 + 半個 dot)
         * 就完全看不到， 直接 HIDDEN 並跳過後面的 opa / zoom / set_pos
         * 計算，避免 dot 越多越卡 */
        {
            const int screen_cx = LV_HOR_RES / 2;
            const int screen_cy = LV_VER_RES / 2;
            const int visible_r = LV_HOR_RES / 2 + (int)(DOT_BG_SIZE) / 2;
            int ddx = dot_x - screen_cx;
            int ddy = dot_y - screen_cy;
            lv_obj_t *dot_bg_obj =
                p_instruction_list_layout->indicator_dots_bg[i];
            if (ddx * ddx + ddy * ddy > visible_r * visible_r)
            {
                if (dot_bg_obj != NULL &&
                    !lv_obj_has_flag(dot_bg_obj, LV_OBJ_FLAG_HIDDEN))
                {
                    lv_obj_add_flag(dot_bg_obj, LV_OBJ_FLAG_HIDDEN);
                }
                continue;
            }
            if (dot_bg_obj != NULL &&
                lv_obj_has_flag(dot_bg_obj, LV_OBJ_FLAG_HIDDEN))
            {
                lv_obj_clear_flag(dot_bg_obj, LV_OBJ_FLAG_HIDDEN);
            }
        }

        // 限制 dot_y 超過 450 或小於 16 時，dot_x 不再變動
        static int last_valid_dot_x[MAX_LIST_ITEMS] = {0};
        if (dot_y > 450 || dot_y < 16)
        {
            if (p_instruction_list_layout->indicator_dots_bg[i] != NULL)
            {
                // dot_x 不變，使用上一次合法的 dot_x
                dot_x = last_valid_dot_x[i];
            }
        }
        else
        {
            last_valid_dot_x[i] = dot_x;
        }

        float angle_from_horizontal = current_angle;

        if (angle_from_horizontal > 180.0f)
        {
            angle_from_horizontal = 360.0f - angle_from_horizontal;
        }

        float distance_angle = angle_from_horizontal;
        if (distance_angle > 90.0f)
        {
            distance_angle = 180.0f - distance_angle;
        }

        float max_distance_angle = 25.0f;
        float ratio = 0.0f;

        if (distance_angle <= max_distance_angle)
        {
            ratio = 1.0f - (distance_angle / max_distance_angle);
        }
        else
        {
            ratio = 0.0f;
        }

        int dot_size = DOT_BG_SIZE;
        int opacity = (int)(LV_OPA_30 + (LV_OPA_COVER - LV_OPA_30) * ratio);
        if (opacity < LV_OPA_30)
            opacity = LV_OPA_30;
        if (opacity > LV_OPA_COVER)
            opacity = LV_OPA_COVER;

        lv_obj_set_style_img_opa(p_instruction_list_layout->indicator_dots[i],
                                 opacity, 0);

        uint16_t zoom =
            (uint16_t)(255 *
                       (DOT_SMOLL_PROPORTION +
                        (DOT_BIG_PROPORTION - DOT_SMOLL_PROPORTION) * ratio));
        if (abs((int)zoom - (int)last_zoom[i]) > 5)
        {
            lv_img_set_zoom(app_icon_shadow[i], zoom);
            lv_img_set_zoom(p_instruction_list_layout->indicator_dots[i], zoom);
            last_zoom[i] = zoom;
        }
        lv_obj_center(p_instruction_list_layout->indicator_dots[i]);
        dot_x -= (dot_size + 30) / 2;
        dot_y -= dot_size / 2;

        lv_obj_set_pos(p_instruction_list_layout->indicator_dots_bg[i], dot_x,
                       dot_y);
    }
}

static void create_indicator_dots(lv_obj_t *parent)
{
    if (p_instruction_list_layout == NULL)
        return;

    int total_dots = list_item_count;

    for (int i = 0; i < total_dots; i++)
    {
        lv_obj_t *dot_bg = lv_obj_create(parent);
        lv_obj_set_size(dot_bg, DOT_BG_SIZE, DOT_BG_SIZE);
        lv_obj_set_style_bg_opa(dot_bg, LV_OPA_0, 0);
        lv_obj_clear_flag(dot_bg, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(dot_bg, LV_OBJ_FLAG_CLICKABLE);

        app_icon_shadow[i] = lv_img_create(dot_bg);
        lv_img_set_src(app_icon_shadow[i], &app_icon_frame);
        lv_obj_align(app_icon_shadow[i], LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);

        lv_obj_t *dot = lv_img_create(dot_bg);
        lv_obj_center(dot);
        if (list_items[i].img_path[0] != '\0')
        {
            lv_img_set_src(dot, list_items[i].img_path);
        }
        else if (list_items[i].icon != NULL)
        {
            lv_img_set_src(dot, list_items[i].icon);
#ifdef APP_ID_SKAI
            if (strcmp(list_items[i].id, APP_ID_SKAI) == 0)
            {
                lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
            }
#endif
        }
        else
        {
            /* Instructions without icon: show frame only */
            lv_img_set_src(dot, &app_icon_frame);
        }

        p_instruction_list_layout->indicator_dots_bg[i] = dot_bg;
        p_instruction_list_layout->indicator_dots[i] = dot;
    }

    /* 初始定位 */
    update_indicator_dots_position(37);
}

extern void tap_on_ai_hint(void);
static bool is_open_ai_gesture = false;
static lv_obj_t *ai_voice_btn = NULL;
static lv_obj_t *ai_voice_send_icon = NULL;
static lv_obj_t *ai_gaus_bg = NULL;

void set_indicator_dots_visible(bool visible)
{
    if (is_indicator_dots_visible == visible)
        return;
    is_indicator_dots_visible = visible;
    for (int i = 0; i < list_item_count; i++)
    {
        if (p_instruction_list_layout->indicator_dots[i] != NULL)
        {
            if (visible)
            {
                lv_obj_set_style_img_opa(
                    p_instruction_list_layout->indicator_dots[i], LV_OPA_60, 0);
            }
            else
            {
                lv_obj_set_style_img_opa(
                    p_instruction_list_layout->indicator_dots[i], LV_OPA_20, 0);
            }
        }
    }
}

static void create_movable_range_arc(lv_obj_t *parent)
{
    if (p_instruction_list_layout == NULL)
        return;

    lv_obj_t *arc = lv_arc_create(parent);

    lv_obj_set_size(arc, LIST_RADIUS, LIST_RADIUS);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);

    int start_angle = 348;
    int end_angle = 13;

    lv_arc_set_range(arc, 0, 30);
    lv_arc_set_bg_start_angle(arc, start_angle);
    lv_arc_set_bg_end_angle(arc, end_angle);
    lv_arc_set_value(arc, 30);

    lv_obj_set_style_arc_width(arc, 0, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_set_style_arc_width(arc, MOVABLE_ARC_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(arc, LV_OPA_70, LV_PART_INDICATOR);

    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc, 0, LV_PART_KNOB);

    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_SCROLLABLE);

    p_instruction_list_layout->movable_range_arc = arc;

    LOG_D("Movable range arc created: radius=%d, start=%d, end=%d",
          LIST_RADIUS / 2, MOVABLE_ARC_START_ANGLE, MOVABLE_ARC_END_ANGLE);
}

extern lv_img_dsc_t *create_widget_snapshot_img(lv_obj_t *target_obj);
lv_obj_t *app_icon[MAX_LIST_ITEMS];
lv_obj_t *app_widget[MAX_LIST_ITEMS];
lv_obj_t *touch_obj[MAX_LIST_ITEMS];
lv_obj_t *app_label[MAX_LIST_ITEMS];
static lv_obj_t *widget_img = NULL;
static bool left_hand_mode = true;
static bool need_correction = false;
static bool is_widget_animation_active = false; // 追蹤 widget 動畫狀態
static uint8_t app_scroll_target_item = 0;
static uint16_t old_selected_item_index = -1;
static lv_obj_t *selected_label;
static lv_obj_t *instruction_list_main_status_bar;
static rt_tick_t last_gohame_time = 0;

void set_arc_stripe_external_offset(int16_t offset_degrees)
{
    update_indicator_dots_position(offset_degrees);
}

static void animate_open_selected_widget_cb(lv_anim_t *a)
{
    if (selected_item_index < list_item_count)
    {
        if (app_widget[selected_item_index] != NULL &&
            lv_obj_is_valid(app_widget[selected_item_index]))
        {
            lv_obj_clear_flag(app_widget[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
        if (touch_obj[selected_item_index] != NULL &&
            lv_obj_is_valid(touch_obj[selected_item_index]))
        {
            lv_obj_clear_flag(touch_obj[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
    }
    lv_obj_add_flag(widget_img, LV_OBJ_FLAG_HIDDEN);

    is_widget_animation_active = false;
}

static void set_widget_img_opa(lv_obj_t *obj, lv_opa_t opa)
{
    // lv_obj_set_style_img_opa(obj, opa, LV_STATE_DEFAULT);
}

static void animate_widget_img_opa(lv_obj_t *obj)
{
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_values(&a, 0, 255);
    lv_anim_set_time(&a, 200);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)set_widget_img_opa);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_set_ready_cb(&a, animate_open_selected_widget_cb);
    lv_anim_start(&a);
}

static lv_coord_t last_y_diff_on_selected = 0;

static bool open_gesture_control = false;
// 停止所有動畫並復原狀態的函數
static void stop_all_animations_and_reset(void)
{
    // 如果沒有進行中的動畫，直接返回
    if (!is_widget_animation_active)
    {
        return;
    }
    else
    {
        LOG_D("Stopping all animations and resetting states");
    }

    if (widget_img != NULL && lv_obj_is_valid(widget_img))
    {
        lv_anim_del(widget_img, (lv_anim_exec_xcb_t)set_widget_img_opa);
        lv_obj_add_flag(widget_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_img_opa(widget_img, LV_OPA_0, LV_STATE_DEFAULT);
    }

    for (uint8_t i = 0; i < list_item_count; i++)
    {
        if (app_icon[i] != NULL && lv_obj_is_valid(app_icon[i]))
        {
            lv_anim_del(app_icon[i], (lv_anim_exec_xcb_t)set_icon_y);
            lv_obj_align(app_icon[i], LV_ALIGN_RIGHT_MID, -25, 0);
        }

        if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
        {
            lv_anim_del(app_label[i], (lv_anim_exec_xcb_t)set_label_y);
            lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
            lv_obj_clear_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
        }

        if (app_widget[i] != NULL && lv_obj_is_valid(app_widget[i]))
        {
            lv_obj_add_flag(app_widget[i], LV_OBJ_FLAG_HIDDEN);
            if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
            {
                lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    is_widget_animation_active = false;

    LOG_D("All animations stopped and states reset");
}

extern void open_skai_widget_ai(bool open);
static bool is_open_instruction_list_ai = false;
bool get_is_open_instruction_list_ai(void)
{
    return is_open_instruction_list_ai;
}
void set_is_open_instruction_list_ai(bool open)
{
    is_open_instruction_list_ai = open;
}
static bool is_at_ai_widget = false;
static bool scroll_initialized = false;
static bool touching_screen = false;
static lv_obj_t *instruction_list_page = NULL;

// 延遲設置 touching_screen 為 false 的定時器
static rt_timer_t touching_screen_timer = NULL;
static void touching_screen_timer_callback(void *parameter)
{
    touching_screen = false;
    LOG_D("touching_screen set to false after delay");
}

static void start_touching_screen_timer(void)
{
    if (!touching_screen_timer)
    {
        touching_screen_timer = rt_timer_create(
            "touching_screen_timer", touching_screen_timer_callback, NULL,
            rt_tick_from_millisecond(500), RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(touching_screen_timer);
    }
    rt_timer_start(touching_screen_timer);
}

static void stop_touching_screen_timer(void)
{
    if (touching_screen_timer)
    {
        rt_timer_stop(touching_screen_timer);
    }
}

static bool open_scroll_motor = false;
static void scroll_list(lv_obj_t *obj, int16_t drift)
{
    uint16_t min_offset = LV_VER_RES;
    uint8_t child_cnt = obj->spec_attr->child_cnt;
    lv_coord_t y_diff = 0;
    lv_coord_t y_diff2 = 0;
    lv_coord_t first_y_diff = 0;
    lv_coord_t selected_item_y_diff = 0;
    lv_coord_t last_y_diff = 0;
    lv_coord_t widget_hight = LIST_ITEM_WIDGET_HEIGHT;
    if (!scroll_initialized)
    {
        scroll_initialized = true;
    }
    if (instruction_list_page->coords.y1 == 0)
        need_correction = false;
    else
        need_correction = true;

    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = obj->spec_attr->children[i];
        widget_hight = LIST_ITEM_WIDGET_HEIGHT;
        lv_coord_t y_center = child->coords.y1 + widget_hight / 2;
        if (need_correction)
        {
            y_diff = y_center - LV_VER_RES / 2 - instruction_list_page->coords.y1;
        }
        else
        {
            y_diff = y_center - LV_VER_RES / 2;
        }
        if (i == 0)
        {
            first_y_diff = y_diff;
        }
        else if (i == child_cnt - 1)
        {
            last_y_diff = y_diff;
        }
        y_diff2 = y_diff;
        y_diff = LV_ABS(y_diff);
        lv_coord_t x_trans;
        if (y_diff >= LIST_RADIUS)
        {
            if (left_hand_mode)
            {
                x_trans = 0;
            }
            else
            {
                x_trans = LIST_RADIUS;
            }
        }
        else
        {
            if (y_diff < min_offset)
            {
                min_offset = y_diff;
                selected_item_y_diff = y_diff2;
                selected_item_index = i;
            }
            rt_uint32_t x_sqr = LIST_RADIUS * LIST_RADIUS - y_diff * y_diff;
            lv_sqrt_res_t res;
            lv_sqrt(x_sqr, &res, 0x8000);
            if (left_hand_mode)
            {
                x_trans = res.i + LIST_ADJUST_X_POSITION;
            }
            else
            {
                x_trans = LIST_RADIUS - res.i;
            }
        }
        lv_obj_set_style_translate_x(child, x_trans, LV_STATE_DEFAULT);
        lv_obj_mark_layout_as_dirty(child);
        static lv_coord_t s_last_y_diff[MAX_LIST_ITEMS] = {0};
        static uint8_t last_brightness[MAX_LIST_ITEMS] = {255};
        static uint8_t s_last_zoom[MAX_LIST_ITEMS] = {0};
        const lv_coord_t DIFF_THRESHOLD = 15; // 變化超過5才更新

        if (abs((int)y_diff - (int)s_last_y_diff[i]) > DIFF_THRESHOLD)
        {
            s_last_y_diff[i] = y_diff;
            // 計算亮度值：選中時全白(255)，遠離時變暗(最暗到80)
            uint8_t brightness = 0;
            if (y_diff >= 75)
            {
                brightness = 0; // 最暗的灰色
            }
            else
            {
                // 從白色(255)漸變到暗灰(80)
                brightness = 255 - (y_diff * (255 - 0) / 75);
            }
            if (brightness != last_brightness[i])
            {
                // 使用顏色深淺代替透明度，創建從白色到灰色的漸變
                lv_color_t text_color =
                    lv_color_make(brightness, brightness, brightness);
                if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                {
                    lv_obj_set_style_text_color(app_label[i], text_color, 0);
                }
                if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                {
                    lv_obj_set_style_bg_opa(switch_objs[i], brightness, 0);
                    lv_obj_t *knob = lv_obj_get_child(switch_objs[i], 0);
                    if (knob != NULL && lv_obj_is_valid(knob))
                    {
                        lv_obj_set_style_bg_opa(knob, brightness, 0);
                    }
                }
                last_brightness[i] = brightness;
            }
        }
    }
    if (touching_screen)
    {
        /* 手勢用全部項目範圍 */
        int target_value = child_cnt * 100 + first_y_diff - 63;
        if (target_value < 0)
            target_value = 0;
        else if (target_value > get_total_moving_distance())
            target_value = get_total_moving_distance();
        set_prev_sensor_quat(target_value);

        /* 指示點也用全部項目範圍 */
        int dots_value = child_cnt * 100 + first_y_diff - 63;
        if (SkaiWatchSys.motion_control_lock)
        {
            update_indicator_dots_position(dots_value);
        }

    }
    if (selected_item_index != old_selected_item_index)
    {
        if (selected_item_index == INSTRUCTION_LIST_ITEMS_DEFINITION[app_id_ai])
        {
            is_at_ai_widget = true;
        }
        else
        {
            is_at_ai_widget = false;
        }
        set_paused_control_with_arm(false);
        if (selected_item_index == child_cnt - 1)
        {
            last_y_diff_on_selected = last_y_diff;
        }
        old_selected_item_index = selected_item_index;
        // LOG_D("selected_app_index: %d", selected_item_index);

        for (uint8_t i = 0; i < child_cnt; i++)
        {
            lv_obj_t *child = obj->spec_attr->children[i];
            if (i == selected_item_index)
            {
                LOG_I("instruction DEBUG Selected item index: %d", i);
                if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
                    lv_obj_clear_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
                if (!list_items[i].is_instruction
#ifdef APP_ID_SKAI
                    && strcmp(list_items[i].id, APP_ID_SKAI) != 0
#endif
                )
                {
                    if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                        lv_obj_clear_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                }
                else
                {
                    if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                        lv_obj_clear_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                }
                if ((i < app_base_count || list_items[i].img_path[0] != '\0') &&
                    app_icon_shadow[i] != NULL &&
                    lv_obj_is_valid(app_icon_shadow[i]))
                    lv_obj_clear_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);
                if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                    lv_obj_clear_flag(switch_objs[i], LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                if ((i < app_base_count || list_items[i].img_path[0] != '\0') &&
                    app_icon_shadow[i] != NULL &&
                    lv_obj_is_valid(app_icon_shadow[i]))
                    lv_obj_add_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);
                if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
                    lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
                if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                    lv_obj_add_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                    lv_obj_add_flag(switch_objs[i], LV_OBJ_FLAG_HIDDEN);
            }
            if (app_icon[i] != NULL && lv_obj_is_valid(app_icon[i]))
                lv_obj_align(app_icon[i], LV_ALIGN_RIGHT_MID, -25, 0);
            if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
            lv_obj_set_style_border_opa(lv_obj_get_child(child, 0), LV_OPA_10,
                                        LV_STATE_DEFAULT);
        }
        if (get_scrolling_motor_vibrate_status() && open_scroll_motor)
        {
            motor_pattern_scrolling_app();
        }
    }
}

static uint8_t prev_app_scroll_target_item = 0;
void open_selected_widget(bool need_widget_img_anima)
{
    if (is_widget_animation_active || created == false)
    {
        return;
    }

    /* Check if selected item has a widget */
    if (selected_item_index >= list_item_count ||
        app_widget[selected_item_index] == NULL)
    {
        if (selected_item_index < list_item_count &&
            touch_obj[selected_item_index] != NULL &&
            lv_obj_is_valid(touch_obj[selected_item_index]))
        {
            lv_obj_clear_flag(touch_obj[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    bool selected_app_has_widget =
        (app_widget[selected_item_index] != NULL &&
         lv_obj_is_valid(app_widget[selected_item_index]));

    if (!selected_app_has_widget)
    {
        if (touch_obj[selected_item_index] != NULL &&
            lv_obj_is_valid(touch_obj[selected_item_index]))
        {
            lv_obj_clear_flag(touch_obj[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    if (widget_img != NULL && lv_obj_is_valid(widget_img))
    {
        lv_obj_del(widget_img);
        widget_img = NULL;
    }

    is_widget_animation_active = true;
    lv_obj_add_flag(app_icon[selected_item_index], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(app_label[selected_item_index], LV_OBJ_FLAG_HIDDEN);
    if (need_widget_img_anima && app_widget[selected_item_index] != NULL &&
        lv_obj_is_valid(app_widget[selected_item_index]))
    {
        lv_obj_t *app_widget_img =
            lv_img_create(p_instruction_list_layout->p_instruction_list_bg);
        lv_obj_set_style_radius(app_widget_img, 50, LV_STATE_DEFAULT);
        lv_img_dsc_t *img_desc =
            create_widget_snapshot_img(app_widget[selected_item_index]);
        lv_img_set_src(app_widget_img, img_desc);
        widget_img = app_widget_img;
        lv_obj_add_flag(widget_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(widget_img, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_img_opa(widget_img, LV_OPA_0, LV_STATE_DEFAULT);
        animate_widget_img_opa(widget_img);
        if (selected_item_index != 0)
        {
            if (app_icon[selected_item_index - 1] != NULL &&
                lv_obj_is_valid(app_icon[selected_item_index - 1]))
                animate_icon_vertical(app_icon[selected_item_index - 1], true);
            if (app_label[selected_item_index - 1] != NULL &&
                lv_obj_is_valid(app_label[selected_item_index - 1]))
                animate_label_vertical(app_label[selected_item_index - 1],
                                       true);
        }
        if (selected_item_index != list_item_count - 1)
        {
            if (app_icon[selected_item_index + 1] != NULL &&
                lv_obj_is_valid(app_icon[selected_item_index + 1]))
                animate_icon_vertical(app_icon[selected_item_index + 1], false);
            if (app_label[selected_item_index + 1] != NULL &&
                lv_obj_is_valid(app_label[selected_item_index + 1]))
                animate_label_vertical(app_label[selected_item_index + 1],
                                       false);
        }
    }
    else
    {
        is_widget_animation_active = false;
        lv_obj_clear_flag(app_widget[selected_item_index], LV_OBJ_FLAG_HIDDEN);
        if (touch_obj[selected_item_index] != NULL &&
            lv_obj_is_valid(touch_obj[selected_item_index]))
        {
            lv_obj_clear_flag(touch_obj[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
        if (selected_item_index != 0)
        {
            if (app_icon[selected_item_index - 1] != NULL &&
                lv_obj_is_valid(app_icon[selected_item_index - 1]))
            {
                lv_obj_align(app_icon[selected_item_index - 1],
                             LV_ALIGN_RIGHT_MID, -20, -30);
            }

            if (app_label[selected_item_index - 1] != NULL &&
                lv_obj_is_valid(app_label[selected_item_index - 1]))
            {
                lv_obj_align(app_label[selected_item_index - 1],
                             LV_ALIGN_CENTER, -20, -30);
            }
        }
        if (selected_item_index != list_item_count - 1)
        {
            if (app_icon[selected_item_index + 1] != NULL &&
                lv_obj_is_valid(app_icon[selected_item_index + 1]))
            {
                lv_obj_align(app_icon[selected_item_index + 1],
                             LV_ALIGN_RIGHT_MID, -20, 30);
            }
            if (app_label[selected_item_index + 1] != NULL &&
                lv_obj_is_valid(app_label[selected_item_index + 1]))
            {
                lv_obj_align(app_label[selected_item_index + 1],
                             LV_ALIGN_CENTER, -30, 30);
            }
        }
    }
    if (selected_label != NULL && lv_obj_is_valid(selected_label))
    {
        lv_obj_set_style_border_opa(selected_label, LV_OPA_30,
                                    LV_STATE_DEFAULT);
    }
}

static void selected_widget_timer_callback(void *parameter)
{
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_WIDGET_LIST_SELECT};
    lvgl_send_msg(msg);
}

static rt_timer_t selected_widget_timer = NULL;
static void selected_widget_timer_start(void)
{
    if (!selected_widget_timer)
    {
        selected_widget_timer = rt_timer_create(
            "speaking_debounce_timer", selected_widget_timer_callback, NULL,
            300, RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(selected_widget_timer);
    }
    rt_timer_start(selected_widget_timer);
}

static void list_window_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;
    switch (evt->code)
    {
    case LV_EVENT_SCROLL_BEGIN:
    {
        // 開始滾動時停止延遲定時器
        stop_touching_screen_timer();
        if (is_user_touching_screen())
        {
            if (!touching_screen)
            {
                touching_screen = true;
            }
        }
        // 當開始滾動時停止所有動畫並復原狀態
        stop_all_animations_and_reset();
        // LOG_D("APP LIST Scroll begin");
        break;
    }
    case LV_EVENT_SCROLL:
    {
        if (is_user_touching_screen())
        {
            if (!touching_screen)
            {
                touching_screen = true;
            }
        }
        scroll_list(obj, 0);
        break;
    }
    case LV_EVENT_SCROLL_END:
    {
        if (prev_app_scroll_target_item != selected_item_index)
        {
            prev_app_scroll_target_item = selected_item_index;
        }
        if (!is_user_touching_screen())
        {
            // 延遲 0.5 秒後才設置 touching_screen 為 false
            start_touching_screen_timer();
        }
        // LOG_D("APP LIST Scroll ended :%d", touching_screen);
        break;
    }
    default:
        break;
    }
    if (obj == NULL)
    {
        return;
    }
}

extern void check_is_at_instruction_list(void);
static void app_list_window_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;
    switch (evt->code)
    {
    case LV_EVENT_VALUE_CHANGED:
    {
        if (lv_obj_get_scroll_y(p_instruction_list_layout->app_list_tileview)/466 != 0)
            return;
        check_is_at_instruction_list();
        break;
    }
    default:
        break;
    }
    if (obj == NULL)
    {
        return;
    }
}
extern void set_skai_widget_opa(uint8_t opa);
extern void set_skai_widget_input_text(const char *text);
static void set_ai_bg_opa(void *obj, int32_t opa)
{
    uint8_t bg_opa = 240 * opa / 255;
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            bg_opa, 0);
    set_skai_widget_opa(opa);
}

/* Mic status: animate ai_voice_btn opacity based on VAD */
static void ai_voice_btn_opa_anim_cb(void *obj, int32_t value)
{
    lv_obj_set_style_bg_opa((lv_obj_t *)obj, value, 0);
}

void handle_ai_voice_btn_vad(bool speaking)
{
    if (ai_voice_btn && lv_obj_is_valid(ai_voice_btn))
    {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, ai_voice_btn);
        lv_anim_set_values(&a, lv_obj_get_style_bg_opa(ai_voice_btn, 0),
                           speaking ? LV_OPA_30 : LV_OPA_10);
        lv_anim_set_exec_cb(&a, ai_voice_btn_opa_anim_cb);
        lv_anim_set_time(&a, 200);
        lv_anim_start(&a);
    }
}

/* Animate skai_widget fade-in */
static bool skai_widget_shown = false;
static void skai_fade_in_anim_cb(void *var, int32_t value)
{
    set_skai_widget_opa((uint8_t)value);
}

/* Called from app_skai.c when speech text is updated */
void instruction_ai_show_skai_widget(void)
{
    if (!is_open_instruction_list_ai || !p_instruction_list_layout)
        return;
    if (skai_widget_shown)
        return;
    skai_widget_shown = true;
    /* Phase 2: text arrived — show gaus bg, send icon, raise bg, fade
     * skai_widget in */
    if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
    {
        lv_obj_clear_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
    }
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
    {
        lv_obj_clear_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_50, 0);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, NULL);
    lv_anim_set_values(&a, 0, LV_OPA_COVER);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, skai_fade_in_anim_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
}

static rt_tick_t last_ai_widget_open_time = 0;
static bool ai_widget_opened_by_drag = false;
void tap_on_ai_widget(void);
void animate_open_ai_widget(void)
{
    if (!get_bluetooth_connection_status())
    {
        create_connection_tips();
        LOG_D("Bluetooth is connected, ignoring voice recognition event");
        return;
    }
    last_ai_widget_open_time = rt_tick_get();
    ai_widget_opened_by_drag = false;
    /* Show ai_page instantly (no slide from left).
       Call tap_on_ai_widget() BEFORE set_tile_id: the tile change fires
       SCROLL_END → VALUE_CHANGED synchronously with LV_ANIM_OFF, and the
       ai_tileview_event_cb would otherwise see
       is_open_instruction_list_ai==false and run the drag-open branch, wrongly
       setting ai_widget_opened_by_drag. */
    lv_obj_clear_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                      LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(p_instruction_list_layout->p_instruction_list_ai_bg);
    tap_on_ai_widget();
    lv_obj_set_tile_id(p_instruction_list_layout->p_instruction_list_ai_bg, 0,
                       0, LV_ANIM_OFF);
    /* set_tile_id fires SCROLL events that drive bg_opa toward ~240 (dark)
       based on scroll position. Reassert the Phase 1 light bg so wrist-raise
       shows only the mic without blacking out the background. */
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_30, 0);
}

void close_ai_widget(void)
{
    extern void clear_skai_widget_ai_reply(void);
    skai_widget_shown = false;
    ai_widget_opened_by_drag = false;
    is_open_instruction_list_ai = false;
    // lvgl_msg_handler.handle_vad_status = NULL;
    clear_skai_widget_ai_reply();
    set_skai_widget_opa(0);
    if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
    {
        lv_obj_add_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_0, 0);
    lv_obj_set_tile_id(p_instruction_list_layout->p_instruction_list_ai_bg, 1,
                       0, LV_ANIM_OFF);
    lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                    LV_OBJ_FLAG_HIDDEN);
}

void check_ai_widget_auto_close(void)
{
    extern bool get_skai_input_text_is_null(void);
    /* Manual drag-open: user explicitly opened it, don't auto-close */
    if (ai_widget_opened_by_drag)
        return;
    if (is_open_instruction_list_ai && !is_at_ai_widget &&
        get_skai_input_text_is_null())
    {
        rt_tick_t current_time = rt_tick_get();
        if (current_time - last_ai_widget_open_time < 3000) // 5秒后自动关闭
        {
            close_ai_widget();
            is_open_instruction_list_ai = false;
        }
    }
}

void tap_on_ai_widget(void)
{
    if (!get_bluetooth_connection_status())
    {
        create_connection_tips();
        LOG_D("Bluetooth is connected, ignoring voice recognition event");
        return;
    }
    // if (is_open_instruction_list_ai)
    // {
    //     extern void send_to_ai(void);
    //     send_to_ai();
    //     return;
    // }
    // lv_obj_clear_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
    // LV_OBJ_FLAG_HIDDEN); lv_anim_t a; lv_anim_init(&a); lv_anim_set_var(&a,
    // p_instruction_list_layout->p_instruction_list_ai_bg);
    // lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER); lv_anim_set_time(&a,
    // 300); lv_anim_set_exec_cb(&a, set_ai_bg_opa); lv_anim_set_path_cb(&a,
    // lv_anim_path_ease_in_out); lv_anim_start(&a);
    /* Phase 1: show voice indicator only, skai_widget stays transparent */
    skai_widget_shown = false;
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_30, 0);
    set_skai_widget_opa(0);
    if (ai_voice_btn && lv_obj_is_valid(ai_voice_btn))
    {
        lv_obj_clear_flag(ai_voice_btn, LV_OBJ_FLAG_HIDDEN);
    }
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
    {
        lv_obj_add_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
    if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
    {
        lv_obj_add_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
    }
    LOG_D("AI widget opened");
    is_open_instruction_list_ai = true;
    open_skai_widget_ai(true);
    // animate_to_ai_page();
    set_skai_widget_input_text("");
    set_ai_open_mic(true);
    show_speech_indicator(true);
    voice_provider.start_v2t();
    // set_free_control_with_arm(false);
    set_paused_control_with_arm(true);
}

static bool instruction_list_ai_tapped = false;
bool get_instruction_list_ai_tapped(void)
{
    return instruction_list_ai_tapped;
}
void set_instruction_list_ai_tapped(void)
{
    instruction_list_ai_tapped = false;
}
void tap_on_ai_hint(void)
{
    if (!get_bluetooth_connection_status())
    {
        create_connection_tips();
        LOG_D("Bluetooth is connected, ignoring voice recognition event");
        return;
    }
    if (isTextEmpty())
    {
        LOG_D("tap_on_ai_hint: empty, skip send");
        return;
    }
    instruction_list_ai_tapped = true;
    extern void send_to_ai(void);
    extern void set_skai_widget_awaiting_ai(void);
    send_to_ai();
    /* Immediate visual feedback: show "AI處理中..." placeholder inside the
       widget. The first streamed AI chunk replaces it. */
    set_skai_widget_awaiting_ai();
    LOG_D("tap_on_ai_hint: send_to_ai fired");
}

/* Called when the first AI reply chunk arrives: hide the "sending" sand icon
   so the mic button indicates the widget is ready for re-ask. */
void on_ai_reply_started(void)
{
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon) &&
        !lv_obj_has_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
}

void show_send_icon(void)
{
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
    {
        lv_obj_clear_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
}

/*******************************************************************************
 * Send instruction update back to phone
 ******************************************************************************/
static void send_instruction_update(list_item_t *item)
{
    if (!item->is_instruction)
        return;

    cJSON *root = cJSON_CreateObject();
    if (!root)
        return;

    cJSON_AddStringToObject(root, "id", item->id);
    cJSON_AddStringToObject(root, "title", item->title);
    cJSON_AddNumberToObject(root, "version", item->version);

    cJSON *trigger = cJSON_CreateObject();
    cJSON_AddStringToObject(trigger, "type", item->trigger_type);
    if (item->is_interval)
        cJSON_AddNumberToObject(trigger, "intervalSeconds", item->interval_sec);
    cJSON_AddItemToObject(root, "trigger", trigger);

    if (item->is_interval)
        cJSON_AddBoolToObject(root, "enabled", item->enabled);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str)
    {
        LOG_I("Send instruction update: %s", json_str);
        commu_send_update_instruction(json_str);
        cJSON_free(json_str);
    }
}

void request_instruction_image(const char *id)
{
    if (!id || id[0] == '\0')
        return;

    extern void set_pending_instruction_img_id(const char *id);
    set_pending_instruction_img_id(id);

    commu_send_get_instruction_img(id);
    LOG_I("Requested instruction image for id=%s", id);
}

extern void iot_gate_widget_tap_event_cb(void);
extern void media_widget_tap_event_cb(void);
static void on_item_tap(list_item_t *item)
{
    LOG_D("on_item_tap: %s", item->id);
    if (item->is_instruction)
    {
        /* Instruction items are handled in on_tap, not here */
        return;
    }
    if (is_open_instruction_list_ai)
    {
        if (!isTextEmpty())
            tap_on_ai_hint();
        else
            LOG_D("AI input is empty, ignoring tap");
    }
    else if (!is_open_instruction_list_ai)
    {
        animate_to_home_from_instruction_list();
        gui_app_run(item->id);
    }
}
static void flash_restore_cb(lv_timer_t *timer)
{
    lv_obj_t *label = (lv_obj_t *)timer->user_data;
    if (label != NULL && lv_obj_is_valid(label))
    {
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    }
}

static void update_custom_switch_visual(uint8_t idx)
{
    lv_obj_t *sw_bg = switch_objs[idx];
    if (sw_bg == NULL || !lv_obj_is_valid(sw_bg))
        return;
    lv_obj_t *knob = lv_obj_get_child(sw_bg, 0);
    if (knob == NULL || !lv_obj_is_valid(knob))
        return;
    if (list_items[idx].enabled)
    {
        lv_obj_set_style_bg_color(sw_bg, lv_color_hex(0x00CCFF), 0);
        lv_obj_align(knob, LV_ALIGN_RIGHT_MID, -1, 0);
    }
    else
    {
        lv_obj_set_style_bg_color(sw_bg, lv_color_hex(0x444444), 0);
        lv_obj_align(knob, LV_ALIGN_LEFT_MID, 1, 0);
    }
}

static void flash_instruction_label(lv_obj_t *label)
{
    if (label == NULL || !lv_obj_is_valid(label))
        return;
    /* 先亮起（高亮色） */
    lv_obj_set_style_text_color(label, lv_color_hex(0x00CCFF), 0);
    /* 300ms 後恢復白色 */
    lv_timer_t *t = lv_timer_create(flash_restore_cb, 300, (void *)label);
    lv_timer_set_repeat_count(t, 1);
}

static void list_item_click_event_cb(lv_event_t *evt)
{
    list_item_t *item = (list_item_t *)evt->user_data;
    lv_obj_t *obj = evt->target;
    LOG_D("ID: %s,obj:%p", item->id, obj);

    if (item->is_instruction)
    {
        LOG_I("Custom instruction tapped: id=%s, title=%s", item->id,
              item->title);
        if (is_open_instruction_list_ai)
        {
            if (!isTextEmpty())
                tap_on_ai_hint();
            return;
        }
        /* Find index in list_items */
        for (uint8_t j = 0; j < list_item_count; j++)
        {
            if (&list_items[j] == item)
            {
                if (item->is_interval)
                {
                    item->enabled = !item->enabled;
                    update_custom_switch_visual(j);
                }
                flash_instruction_label(app_label[j]);
                break;
            }
        }
        send_instruction_update(item);
        if (instruction_tap_cb)
        {
            instruction_tap_cb(item->id, item->enabled);
        }
    }
#ifdef APP_ID_SKAI
    else if (strcmp(item->id, APP_ID_SKAI) == 0)
    {
        tap_on_ai_hint();
    }
#endif
    else
    {
        on_item_tap(item);
    }
}

static bool tap_to_open_control = false;
static bool open_quick_app = false;
static void on_tap(void)
{
    if (selected_item_index >= list_item_count)
        return;

    list_item_t *item = &list_items[selected_item_index];
    if (item->is_instruction)
    {
        LOG_I("Custom instruction tapped via gesture: id=%s", item->id);
        if (is_open_instruction_list_ai)
        {
            if (!isTextEmpty())
                tap_on_ai_hint();
            return;
        }
        if (item->is_interval)
        {
            item->enabled = !item->enabled;
            update_custom_switch_visual(selected_item_index);
        }
        flash_instruction_label(app_label[selected_item_index]);
        send_instruction_update(item);
        if (instruction_tap_cb)
        {
            instruction_tap_cb(item->id, item->enabled);
        }
    }
    else
    {
        on_item_tap(item);
    }
}

static int16_t find_app_index_by_id(uint16_t app_id)
{
    for (int i = 0; i < ARRAY_SIZE(INSTRUCTION_LIST_ITEMS_DEFINITION); i++)
    {
        if (INSTRUCTION_LIST_ITEMS_DEFINITION[i] == app_id)
        {
            return i;
        }
    }
    return app_base_count - 1; // 未找到
}

extern char *get_media_title(void);
extern bool is_have_message_now(void);
static uint16_t gesture_starting_value = 0;
static void reset_list(void)
{
    if (p_instruction_list_layout->list == NULL)
    {
        return;
    }
    open_scroll_motor = false;
    disable_scrolling_motor_vibrate();
    set_paused_control_with_arm(false);
    scroll_initialized = false;
    uint8_t scroll_to_index;
    {
        /* 滾到列表最下面那個項目 */
        scroll_to_index = list_item_count - 1;
        app_scroll_target_item = scroll_to_index;
    }
    gesture_starting_value = 37;
    selected_item_index = app_scroll_target_item;
    prev_app_scroll_target_item = app_scroll_target_item;
    lv_obj_t *child =
        lv_obj_get_child(p_instruction_list_layout->list, scroll_to_index);
    lv_obj_scroll_to_view(child, LV_ANIM_OFF);
    if (!scroll_initialized)
    {
        scroll_list(p_instruction_list_layout->list, 0);
    }
    update_indicator_dots_position(gesture_starting_value);
    open_selected_widget(false);
    is_widget_animation_active = false;
    enable_scrolling_motor_vibrate();
    open_scroll_motor = true;
}

uint16_t get_gesture_starting_value(void)
{
    return gesture_starting_value;
}

extern void media_widget_trigger_drag_by_py(int p_y);
extern void iot_gate_trigger_drag_by_py(int p_y);
static void button_selection(gesture_position_t gesture_position)
{
    const int p_y = gesture_position.gesture_position_y;
    if (selected_item_index == find_app_index_by_id(app_id_media))
    {
        media_widget_trigger_drag_by_py(p_y);
    }
    else if (selected_item_index == find_app_index_by_id(app_id_iot_gate))
    {
        iot_gate_trigger_drag_by_py(p_y);
    }
}

#ifdef USE_QUICK_OPEN_AI
static uint8_t quick_app_id = 0;
static void gesture_tap_event_handler(uint8_t gesture)
{
    if (gesture != 1)
    {
        return;
    }
    if (quick_app_id == 0)
    {
        force_release_finger();
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_CONTROL_QUICK_BTN;
        msg.data.quick_btn_action.new_point = 0;
        msg.data.quick_btn_action.swich_quick_btn = 0;
        lvgl_send_msg(msg);
        msg.type = LVGL_MSG_TYPE_SWITCH_FLASHLIGHT;
        lvgl_send_msg(msg);
    }
    else if (quick_app_id == 1)
    {
        force_release_finger();
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_CONTROL_QUICK_BTN;
        msg.data.quick_btn_action.new_point = 0;
        msg.data.quick_btn_action.swich_quick_btn = 0;
        lvgl_send_msg(msg);
    }
}

static void open_quick_app_timer_cb(void *param)
{
    lvgl_msg_handler.handle_tap_indicator = gesture_tap_event_handler;
}

static rt_timer_t timer_open_quick_app = RT_NULL;
static void start_open_quick_app_timer(uint8_t set_app_id)
{
    quick_app_id = set_app_id;
    if (!timer_open_quick_app)
    {
        timer_open_quick_app = rt_timer_create(
            "open_quick_app", open_quick_app_timer_cb, RT_NULL,
            rt_tick_from_millisecond(300), RT_TIMER_FLAG_ONE_SHOT);
        if (timer_open_quick_app == RT_NULL)
        {
            LOG_E("Failed to create timer_open_quick_app");
            return;
        }
        LOG_D("timer_open_quick_app is created");
    }
    else
    {
        rt_timer_stop(timer_open_quick_app);
    }
    rt_timer_start(timer_open_quick_app);
}

static void stop_open_quick_app_timer(void)
{
    if (timer_open_quick_app)
    {
        rt_timer_stop(timer_open_quick_app);
    }
}

static bool open_vibration = false;
static quick_open_app_t quick_open_app_obj;
static void move_quick_icon(QuickBtn pame)
{
    uint16_t distance = pame.new_point;
    uint16_t swich_quick_btn = pame.swich_quick_btn;

    if (swich_quick_btn == 1 && is_at_instruction_list())
    {
    }
    else if (swich_quick_btn == 2)
    {
    }
    else if (swich_quick_btn == 3 && !app_voice_get_voice2text_status() &&
             !is_at_home())
    {
        if (distance > 45)
        {
            if (open_vibration)
            {
                start_open_quick_app_timer(1);
                open_vibration = false;
            }
            open_quick_app = true;
        }
        else
        {
            lvgl_msg_handler.handle_tap_indicator = NULL;
            stop_open_quick_app_timer();
            open_quick_app = false;
            open_vibration = true;
        }
    }
    else if (swich_quick_btn == 4 && !app_voice_get_voice2text_status() &&
             !is_at_home())
    {
        if (open_vibration)
        {
            start_open_quick_app_timer(1);
            open_vibration = false;
        }
        open_quick_app = true;
        lv_obj_set_style_border_opa(quick_open_app_obj.left, LV_OPA_80,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(quick_open_app_obj.left, LV_OPA_100,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_pos(quick_open_app_obj.left, 50, 258);
    }
    else if (swich_quick_btn == 0)
    {
        lvgl_msg_handler.handle_tap_indicator = NULL;
        stop_open_quick_app_timer();
        open_quick_app = false;
        lv_obj_set_style_border_opa(quick_open_app_obj.bottom, LV_OPA_0,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(quick_open_app_obj.bottom, LV_OPA_0,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_pos(quick_open_app_obj.bottom, 258, 0);
        lv_obj_set_style_border_opa(quick_open_app_obj.left, LV_OPA_0,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(quick_open_app_obj.left, LV_OPA_0,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_pos(quick_open_app_obj.left, 0, 258);
    }
}
#endif

// 設置指示點位置的公共函數
// input_value: 0-1000 的輸入值
// 100 時第一個點在最右邊中心（0度），900 時最後一個點在最右邊中心（0度）
// 所有點分佈在80度範圍內（-40度到+40度）
void set_instruction_list_indicator_dots_position(int input_value)
{
    update_indicator_dots_position(input_value);
}

#ifdef TEST_INDICATOR_ANIMATION
// 動畫測試相關的變數
static lv_anim_t indicator_test_anim;
static bool indicator_test_running = false;

// 動畫執行回調函數 - 符合LVGL要求的簽名
static int prev_value = 0;
static void indicator_test_anim_exec_cb(void *var, int32_t value)
{
    if (abs(value - prev_value) < 10)
    {
        return; // 如果變化不大，則忽略
    }
    prev_value = value;
    set_instruction_list_indicator_dots_position((int)value);
    LOG_D("Animation test: input_value = %d", (int)value);
}

// 開始動畫測試
void start_indicator_dots_animation_test(void)
{
    if (p_instruction_list_layout == NULL || !created)
    {
        LOG_D("App list layout not created yet, cannot start animation test");
        return;
    }

    if (indicator_test_running)
    {
        LOG_D("Animation test already running");
        return;
    }

    LOG_D("Starting indicator dots animation test (0->1000->0 loop)");
    indicator_test_running = true;

    // 初始化動畫
    lv_anim_init(&indicator_test_anim);
    lv_anim_set_var(&indicator_test_anim, NULL);
    lv_anim_set_exec_cb(&indicator_test_anim, indicator_test_anim_exec_cb);
    lv_anim_set_playback_time(&indicator_test_anim, 5000); // 3秒從0到1000
    lv_anim_set_time(&indicator_test_anim, 5000);          // 3秒從0到1000
    lv_anim_set_values(&indicator_test_anim, 0, 1250);
    lv_anim_set_path_cb(&indicator_test_anim, lv_anim_path_ease_in_out);
    lv_anim_set_repeat_count(&indicator_test_anim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&indicator_test_anim);
}

// 停止動畫測試
void stop_indicator_dots_animation_test(void)
{
    if (!indicator_test_running)
    {
        LOG_D("Animation test not running");
        return;
    }

    LOG_D("Stopping indicator dots animation test");
    indicator_test_running = false;
    lv_anim_del(&indicator_test_anim, indicator_test_anim_exec_cb);

    // 重置到中間位置
    set_instruction_list_indicator_dots_position(500);
}
#endif

static bool send_to_ai_again = false;
bool get_send_to_ai_again(void)
{
    return send_to_ai_again;
}
void set_send_to_ai_again(bool value)
{
    send_to_ai_again = value;
}
extern void set_ai_open_mic(bool is_open);
extern bool skai_widget_has_ai_reply(void);
extern void clear_skai_widget_ai_reply(void);
extern bool get_voice_recognition_started(void);
extern void clearVoice2Text(void);
static void logo_click_event_cb(lv_event_t *evt)
{
    /* Re-ask has priority: once the AI has replied, the button's job is to
       clear everything and start a new voice capture — even if the old
       speech text is still in the v2t buffer. */
    if (skai_widget_has_ai_reply() && !get_voice_recognition_started())
    {
        if (!get_bluetooth_connection_status())
        {
            create_connection_tips();
            return;
        }
        send_to_ai_again = true;
        clearVoice2Text();
        clear_skai_widget_ai_reply();
        set_skai_widget_input_text("");
        set_ai_open_mic(true);
        open_skai_widget_ai(true);
        voice_provider.start_v2t();
        return;
    }
    /* Normal flow: if we have speech text in the buffer, send it to AI. */
    if (!isTextEmpty())
    {
        tap_on_ai_hint();
        lv_obj_add_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
}
static void ai_bar_event_cb(lv_event_t *evt)
{
    if (evt->code == LV_EVENT_PRESSED)
    {
        if (!get_bluetooth_connection_status())
        {
            create_connection_tips();
            LOG_D("Bluetooth is connected, ignoring voice recognition event");
            return;
        }
        lv_obj_clear_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                          LV_OBJ_FLAG_HIDDEN);
        /* Pre-set Phase 2 visual state so the skai_widget and gaus_bg are
           already opaque as the user drags ai_page in. Skipped if AI is
           already open (e.g. wrist-raise Phase 1 is active with mic only). */
        if (!is_open_instruction_list_ai)
        {
            if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
            {
                lv_obj_clear_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
            }
            if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
            {
                lv_obj_clear_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
            }
            set_skai_widget_opa(LV_OPA_COVER);
        }
    }
}
static uint16_t ai_bg_opa = 240;
static void ai_tileview_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = lv_event_get_target(evt);
    switch (evt->code)
    {
    case LV_EVENT_SCROLL:
    {
        uint16_t ai_scroll_x =
            (466 - lv_obj_get_scroll_x(obj)) * ai_bg_opa / 350;
        uint8_t calculated_opa =
            (ai_scroll_x > ai_bg_opa) ? ai_bg_opa : ai_scroll_x;
        lv_obj_set_style_bg_opa(
            p_instruction_list_layout->p_instruction_list_ai_bg, calculated_opa,
            0);
        break;
    }
    case LV_EVENT_VALUE_CHANGED:
    {
        lv_coord_t scroll_x = lv_obj_get_scroll_x(obj);
        if (abs(scroll_x) % 466 != 0)
        {
            break;
        }
        rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(evt);
        if (active_pos == 0)
        {
            // is_open_instruction_list_ai = false;
            voice_provider.stop_v2t();
            stop_voice_recognition(V2T_INTENT_NOTHING);
            set_is_open_instruction_list_ai(false);
            open_skai_widget_ai(false);
            set_paused_control_with_arm(false);
            set_ai_open_mic(false);
            // lvgl_msg_handler.handle_vad_status = NULL;
            skai_widget_shown = false;
            if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
            {
                lv_obj_add_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
            }
            // show_speech_indicator(false);
            // voice_provider.stop_v2t();
            // close_ai_widget();
            lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OBJ_FLAG_HIDDEN);
        }
        else if (active_pos == 1)
        {
            if (!is_open_instruction_list_ai)
            {
                set_ai_open_mic(true);
                tap_on_ai_widget();
                /* Drag-opened: jump straight to Phase 2 (widget visible, dark
                   bg) without the fade-in animation — the drag itself already
                   provides the reveal motion. */
                ai_widget_opened_by_drag = true;
                skai_widget_shown = true;
                if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
                {
                    lv_obj_clear_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
                }
                // if (ai_voice_send_icon &&
                // lv_obj_is_valid(ai_voice_send_icon))
                // {
                //     lv_obj_clear_flag(ai_voice_send_icon,
                //     LV_OBJ_FLAG_HIDDEN);
                // }
                lv_obj_set_style_bg_opa(
                    p_instruction_list_layout->p_instruction_list_ai_bg,
                    LV_OPA_50, 0);
                set_skai_widget_opa(LV_OPA_COVER);
            }
        }
        else
        {
            LOG_W("Unknown tileview position: %d", active_pos);
        }
        break;
    }
    default:
        break;
    }
}

static void home_tileview_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = lv_event_get_target(evt);
    switch (evt->code)
    {
    case LV_EVENT_RELEASED:
    {
        /* Only hide ai_bg if user pressed without actually opening the AI page.
           `obj` is the home_page tile (always scroll 0,0); check the tileview's
           scroll via its parent, and keep it visible whenever AI is open. */
        lv_obj_t *tileview = lv_obj_get_parent(obj);
        if (!is_open_instruction_list_ai && tileview &&
            lv_obj_get_scroll_x(tileview) == 466)
        {
            lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OBJ_FLAG_HIDDEN);
        }
        break;
    }
    default:
        break;
    }
}

/*******************************************************************************
 * Custom Instruction API
 ******************************************************************************/

uint8_t get_custom_instruction_count(void)
{
    if (list_item_count > app_base_count)
        return list_item_count - app_base_count;
    return 0;
}

void clear_custom_instructions(void)
{
    list_item_count = app_base_count;
}

/* 根據 id 找到已有的指令，回傳 index，找不到回傳 -1 */
static int find_instruction_by_id(const char *id)
{
    for (uint8_t i = app_base_count; i < list_item_count; i++)
    {
        if (strcmp(list_items[i].id, id) == 0)
            return i;
    }
    return -1;
}

void remove_custom_instruction(const char *id)
{
    int idx = find_instruction_by_id(id);
    if (idx < 0)
    {
        LOG_W("remove_custom_instruction: id=%s not found", id);
        return;
    }

    /* Delete image file if exists */
    if (list_items[idx].img_path[0] != '\0')
    {
        remove(list_items[idx].img_path);
        LOG_I("Deleted instruction image: %s", list_items[idx].img_path);
    }

    /* Shift remaining items left */
    for (uint8_t i = idx; i < list_item_count - 1; i++)
    {
        memcpy(&list_items[i], &list_items[i + 1], sizeof(list_item_t));
    }
    list_item_count--;

    LOG_I("Removed instruction: id=%s", id);
}

void add_or_update_custom_instruction(const char *id, const char *title,
                                      const char *trigger_type,
                                      uint32_t interval_sec, bool enabled,
                                      uint32_t version)
{
    bool is_interval = (trigger_type && strcmp(trigger_type, "interval") == 0);
    int idx = find_instruction_by_id(id);
    if (idx >= 0)
    {
        /* 已存在 — 更新標題、參數和開關狀態 */
        strncpy(list_items[idx].title, title, LIST_ITEM_TITLE_LEN - 1);
        list_items[idx].title[LIST_ITEM_TITLE_LEN - 1] = '\0';
        list_items[idx].is_interval = is_interval;
        list_items[idx].interval_sec = interval_sec;
        list_items[idx].enabled = enabled;
        list_items[idx].version = version;
        if (trigger_type)
        {
            strncpy(list_items[idx].trigger_type, trigger_type, 31);
            list_items[idx].trigger_type[31] = '\0';
        }
        LOG_I("Updated id=%s, enabled=%d", id, enabled);
        return;
    }

    if (list_item_count >= MAX_LIST_ITEMS)
        return;

    list_item_t *instr = &list_items[list_item_count];
    memset(instr, 0, sizeof(list_item_t));
    strncpy(instr->id, id, LIST_ITEM_ID_LEN - 1);
    instr->id[LIST_ITEM_ID_LEN - 1] = '\0';
    strncpy(instr->title, title, LIST_ITEM_TITLE_LEN - 1);
    instr->title[LIST_ITEM_TITLE_LEN - 1] = '\0';
    if (trigger_type)
    {
        strncpy(instr->trigger_type, trigger_type, 31);
        instr->trigger_type[31] = '\0';
    }
    instr->icon = NULL;
    instr->widget = NULL;
    instr->is_instruction = true;
    instr->is_interval = is_interval;
    instr->enabled = enabled;
    instr->interval_sec = interval_sec;
    instr->version = version;
    list_item_count++;
}

/* Helper: create list item UI objects for items in [start_idx, end_idx) */
static void create_list_items_ui(lv_obj_t *list, uint8_t start_idx,
                                 uint8_t end_idx)
{
    for (uint8_t i = start_idx; i < end_idx; i++)
    {
        lv_obj_t *widget = NULL;
        lv_obj_t *item = lv_simplified_obj_create(list);
        lv_obj_clear_flag(item, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(item, 466, LIST_ITEM_WIDGET_HEIGHT);
        if (i == 0)
        {
            lv_obj_set_pos(item, 0, (100 + LIST_ITEM_SPACING));
        }
        else
        {
            lv_obj_set_pos(item, 0,
                           (LIST_ITEM_WIDGET_HEIGHT + LIST_ITEM_SPACING) * i +
                               (100 + LIST_ITEM_SPACING));
        }

        bool has_widget = false;

        if (!list_items[i].is_instruction)
        {
            /* App items: check for special widgets */
            if (i < ARRAY_SIZE(INSTRUCTION_LIST_ITEMS_DEFINITION))
            {
                if (INSTRUCTION_LIST_ITEMS_DEFINITION[i] == app_id_ai)
                {
                    extern lv_obj_t *lv_skai_widget_builder(lv_obj_t * parent);
                    widget = lv_skai_widget_builder(item);
                    has_widget = true;
                }
            }
        }

        if (has_widget && widget != NULL)
        {
            lv_obj_set_size(widget, LIST_ITEM_WIDGET_WIDTH,
                            LIST_ITEM_WIDGET_HEIGHT);
            if (i < ARRAY_SIZE(INSTRUCTION_LIST_ITEMS_DEFINITION) &&
                INSTRUCTION_LIST_ITEMS_DEFINITION[i] != app_id_ai)
            {
                lv_obj_set_style_clip_corner(widget, true, 0);
            }
            else
            {
                lv_obj_add_flag(widget, LV_OBJ_FLAG_SCROLLABLE);
            }
            if (i < ARRAY_SIZE(INSTRUCTION_LIST_ITEMS_DEFINITION) &&
                INSTRUCTION_LIST_ITEMS_DEFINITION[i] != app_id_ai)
            {
                lv_obj_set_style_border_color(widget, lv_color_hex(0xFFFFFF),
                                              0);
                lv_obj_set_style_border_width(widget, 2, 0);
                lv_obj_set_style_border_opa(widget, LV_OPA_20, 0);
                lv_obj_add_event_cb(widget, list_item_click_event_cb,
                                    LV_EVENT_CLICKED, (void *)&list_items[i]);
            }
        }

        /* Create touch overlay */
        touch_obj[i] = lv_obj_create(item);
        lv_obj_set_size(touch_obj[i], LIST_ITEM_WIDGET_WIDTH,
                        LIST_ITEM_WIDGET_HEIGHT);
        lv_obj_set_style_bg_opa(touch_obj[i], LV_OPA_0, 0);
        lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_align(touch_obj[i], LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_event_cb(touch_obj[i], list_item_click_event_cb,
                            LV_EVENT_CLICKED, (void *)&list_items[i]);

        app_widget[i] = widget;
        list_items[i].widget = widget;

        /* Create app icon */
        p_instruction_list_layout->p_app_indicator_btn[i] = lv_img_create(item);
        if (list_items[i].img_path[0] != '\0')
        {
            lv_img_set_src(p_instruction_list_layout->p_app_indicator_btn[i],
                           list_items[i].img_path);
        }
        else if (list_items[i].icon != NULL)
        {
            lv_img_set_src(p_instruction_list_layout->p_app_indicator_btn[i],
                           list_items[i].icon);
        }
        lv_obj_align(p_instruction_list_layout->p_app_indicator_btn[i],
                     LV_ALIGN_RIGHT_MID, -25, 0);
        app_icon[i] = p_instruction_list_layout->p_app_indicator_btn[i];
        lv_obj_add_event_cb(app_icon[i], list_item_click_event_cb,
                            LV_EVENT_CLICKED, (void *)&list_items[i]);
        lv_obj_add_flag(app_icon[i], LV_OBJ_FLAG_HIDDEN);
        if (list_items[i].icon == NULL && list_items[i].img_path[0] == '\0')
        {
            lv_obj_add_flag(app_icon[i], LV_OBJ_FLAG_HIDDEN);
        }

        /* Create label */
        app_label[i] = lv_label_create(item);
        lv_label_set_text(app_label[i], list_items[i].title);
        lv_obj_set_style_text_font(app_label[i],
                                   LV_EXT_FONT_GET(get_system_font_size(1)), 0);
        lv_obj_set_style_text_color(app_label[i], lv_color_hex(0xFFFFFF), 0);

        /* For instructions with interval, create switch and position label left
         */
        if (list_items[i].is_instruction && list_items[i].is_interval)
        {
            lv_obj_align(app_label[i], LV_ALIGN_LEFT_MID, 30, 0);

            /* 開關底座 */
            lv_obj_t *sw_bg = lv_obj_create(item);
            lv_obj_set_size(sw_bg, 50, 26);
            lv_obj_align(sw_bg, LV_ALIGN_RIGHT_MID, -30, 0);
            lv_obj_set_style_radius(sw_bg, 13, 0);
            lv_obj_set_style_border_width(sw_bg, 0, 0);
            lv_obj_clear_flag(sw_bg, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(sw_bg, LV_OBJ_FLAG_CLICKABLE);

            /* 圓形 knob */
            lv_obj_t *knob = lv_obj_create(sw_bg);
            lv_obj_set_size(knob, 20, 20);
            lv_obj_set_style_radius(knob, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(knob, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_border_width(knob, 0, 0);
            lv_obj_clear_flag(knob, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(knob, LV_OBJ_FLAG_CLICKABLE);

            if (list_items[i].enabled)
            {
                lv_obj_set_style_bg_color(sw_bg, lv_color_hex(0x00CCFF), 0);
                lv_obj_align(knob, LV_ALIGN_RIGHT_MID, -1, 0);
            }
            else
            {
                lv_obj_set_style_bg_color(sw_bg, lv_color_hex(0x444444), 0);
                lv_obj_align(knob, LV_ALIGN_LEFT_MID, 1, 0);
            }

            switch_objs[i] = sw_bg;
        }
        else
        {
            lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
            switch_objs[i] = NULL;
        }

        /* Hide labels for ai apps */
        if (!list_items[i].is_instruction &&
            i < ARRAY_SIZE(INSTRUCTION_LIST_ITEMS_DEFINITION))
        {
            if (INSTRUCTION_LIST_ITEMS_DEFINITION[i] == app_id_ai)
            {
                lv_obj_add_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
            }
        }

        LOG_D("List item %d: id=%s, title=%s, is_instruction=%d", i,
              list_items[i].id, list_items[i].title,
              list_items[i].is_instruction);
    }
}

static rt_tick_t s_last_refresh_tick = 0;
static lv_timer_t *s_pending_refresh_timer = NULL;

static void deferred_refresh_cb(lv_timer_t *t)
{
    s_pending_refresh_timer = NULL;
    /* Bypass debounce on the trailing run so the latest pending state
       (e.g. multiple instructions added in rapid succession) is rebuilt. */
    s_last_refresh_tick = 0;
    lv_timer_del(t);
    refresh_custom_instructions();
}

void refresh_custom_instructions(void)
{
    open_scroll_motor = false;
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->list == NULL)
        return;

    LOG_I("Refreshing custom instructions...");
    /* Trailing-edge debounce: within 500ms, skip the immediate run but
       schedule a deferred refresh so the latest call still rebuilds the UI.
       Without this, a burst of N>=2 add_or_update calls would leave
       list_item_count incremented past the number of indicator dots actually
       created, breaking dot positioning. */
    rt_tick_t now = rt_tick_get();
    if (s_last_refresh_tick != 0 &&
        (now - s_last_refresh_tick) < rt_tick_from_millisecond(500))
    {
        LOG_I("refresh_custom_instructions: deferred (debounce)");
        if (s_pending_refresh_timer == NULL)
        {
            s_pending_refresh_timer =
                lv_timer_create(deferred_refresh_cb, 550, NULL);
            lv_timer_set_repeat_count(s_pending_refresh_timer, 1);
        }
        return;
    }
    s_last_refresh_tick = now;
    if (s_pending_refresh_timer != NULL)
    {
        lv_timer_del(s_pending_refresh_timer);
        s_pending_refresh_timer = NULL;
    }

    lv_obj_t *list = p_instruction_list_layout->list;

    /* 先刪除舊的指示點（它們是 bg 的子物件，lv_obj_clean(list) 不會刪到） */
    lv_obj_t *bg = p_instruction_list_layout->p_instruction_list_bg;
    for (uint8_t i = 0; i < MAX_LIST_ITEMS; i++)
    {
        if (p_instruction_list_layout->indicator_dots_bg[i] != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->indicator_dots_bg[i]))
        {
            lv_obj_del(p_instruction_list_layout->indicator_dots_bg[i]);
        }
    }

    /* Save current scroll position and selected index */
    lv_coord_t saved_scroll_y = lv_obj_get_scroll_y(list);
    uint16_t saved_selected = selected_item_index;

    /* Delete ALL children of the list */
    lv_obj_clean(list);

    /* Reset UI arrays */
    for (uint8_t i = 0; i < MAX_LIST_ITEMS; i++)
    {
        app_icon[i] = NULL;
        app_widget[i] = NULL;
        touch_obj[i] = NULL;
        app_label[i] = NULL;
        switch_objs[i] = NULL;
        app_icon_shadow[i] = NULL;
        p_instruction_list_layout->p_app_indicator_btn[i] = NULL;
        p_instruction_list_layout->indicator_dots[i] = NULL;
        p_instruction_list_layout->indicator_dots_bg[i] = NULL;
    }

    /* Recreate all list item UI */
    create_list_items_ui(list, 0, list_item_count);

    /* 重建指示點 */
    create_indicator_dots(bg);

    /* Keep the AI widget tileview above the recreated dots — dots are
       siblings of p_instruction_list_ai_bg under p_instruction_list_bg and
       new children are drawn on top, so re-raise the AI widget. */
    if (p_instruction_list_layout->p_instruction_list_ai_bg != NULL &&
        lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_ai_bg))
    {
        lv_obj_move_foreground(
            p_instruction_list_layout->p_instruction_list_ai_bg);
    }

    /* Force layout so child coords are valid */
    lv_obj_update_layout(list);

    /* Restore scroll position */
    old_selected_item_index = (uint16_t)-1;
    if (saved_selected >= list_item_count && list_item_count > 0)
    {
        /* Selected item was removed — scroll to new last item */
        uint16_t target = list_item_count - 1;
        app_scroll_target_item = target;
        selected_item_index = target;
        lv_obj_t *child = lv_obj_get_child(list, target);
        if (child && lv_obj_is_valid(child))
            lv_obj_scroll_to_view(child, LV_ANIM_OFF);
        lv_obj_update_layout(list);
        scroll_list(list, 0);
    }
    else
    {
        /* Add or no change — keep exact same scroll position */
        lv_obj_scroll_to_y(list, saved_scroll_y, LV_ANIM_OFF);
        lv_obj_update_layout(list);
        scroll_list(list, 0);
    }

    /* Recalculate input_value for indicator dots based on current
     * selected_item_index */
    if (list_item_count > 0)
    {
        float total_range = 100.0f * list_item_count;
        float base_input = 63.0f;
        float input_val = total_range - base_input -
                          selected_item_index * (total_range / list_item_count);
        gesture_starting_value = (uint16_t)input_val;
        update_indicator_dots_position(gesture_starting_value);
    }
    open_scroll_motor = true;
    if (!is_at_instruction_list())
    {
        reset_list();
    }
    LOG_D("refresh_custom_instructions: %d items total", list_item_count);
}

void update_instruction_image(const char *id, const char *path)
{
    int idx = find_instruction_by_id(id);
    if (idx < 0)
    {
        LOG_W("update_instruction_image: id=%s not found", id);
        return;
    }

    /* Build image path using id prefix (before first '-') */
    char id_prefix[64];
    strncpy(id_prefix, id, sizeof(id_prefix) - 1);
    id_prefix[sizeof(id_prefix) - 1] = '\0';
    char *dash = strchr(id_prefix, '-');
    if (dash)
        *dash = '\0';

    char img_path[128];
    rt_snprintf(img_path, sizeof(img_path), "/assets/images/instruction/%s.bin",
                id_prefix);

    lv_img_cache_invalidate_src(list_items[idx].img_path);
    strncpy(list_items[idx].img_path, img_path,
            sizeof(list_items[idx].img_path) - 1);
    list_items[idx].img_path[sizeof(list_items[idx].img_path) - 1] = '\0';

    /* Update the indicator dot directly if UI exists */
    if (p_instruction_list_layout != NULL &&
        p_instruction_list_layout->indicator_dots[idx] != NULL &&
        lv_obj_is_valid(p_instruction_list_layout->indicator_dots[idx]))
    {
        lv_img_cache_invalidate_src(img_path);
        lv_img_set_src(p_instruction_list_layout->indicator_dots[idx],
                       list_items[idx].img_path);
        lv_obj_clear_flag(p_instruction_list_layout->indicator_dots[idx],
                          LV_OBJ_FLAG_HIDDEN);
        LOG_I("Updated instructionwith new image %s", img_path);
    }
}

static lv_obj_t *ai_tileview = NULL;

static void go_to_app_list_btn_cb(lv_event_t *evt)
{
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->app_list_tileview == NULL)
        return;
    lv_obj_set_tile_id(p_instruction_list_layout->app_list_tileview, 0, 1,
                       LV_ANIM_ON);
    LOG_I("Navigate to app list via button");
}

void back_to_instruction_list_btn(void)
{
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->app_list_tileview == NULL)
        return;
    lv_obj_set_tile_id(p_instruction_list_layout->app_list_tileview, 0, 0,
                       LV_ANIM_ON);
    LOG_I("Navigate back to instruction list via button");
}

bool get_app_list_tileview_page(void)
{
    /* App list tileview was removed; the instruction list is the only page. */
    return true;
}

lv_obj_t *lv_instruction_list_layout_create(lv_obj_t *parent)
{
    // 檢查是否已經分配，如果是則先釋放
    if (p_instruction_list_layout != NULL)
    {
        LOG_W("p_instruction_list_layout already exists, cleaning up...");
        instruction_list_deinit();
    }
    size_t allocate_size = sizeof(instruction_list_layout_t);
    p_instruction_list_layout = (instruction_list_layout_t *)lv_mem_alloc(
        sizeof(instruction_list_layout_t));
    if (p_instruction_list_layout == NULL)
    {
        LOG_E("Failed to allocate memory for p_instruction_list_layout");
        return NULL;
    }
    memset(p_instruction_list_layout, 0, sizeof(instruction_list_layout_t));
    memset(switch_objs, 0, sizeof(switch_objs));
    LOG_I("[CHECK_MEMORY]instruction_list_init(%d bytes)", allocate_size);
    instruction_list_page = parent;

    load_instruction_list();

    lv_obj_t *p_instruction_list_bg = lv_obj_create(parent);
    p_instruction_list_layout->p_instruction_list_bg = p_instruction_list_bg;
    lv_obj_set_style_bg_opa(p_instruction_list_bg, LV_OPA_0, 0);
    lv_obj_set_size(p_instruction_list_bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(p_instruction_list_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(p_instruction_list_bg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *p_instruction_list = lv_obj_create(p_instruction_list_bg);
    p_instruction_list_layout->list = p_instruction_list;
    LOG_D("p_instruction_list: %p", p_instruction_list);
    lv_obj_set_size(p_instruction_list, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(p_instruction_list, LV_OPA_0, 0);
    lv_obj_add_flag(p_instruction_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(p_instruction_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_instruction_list, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(p_instruction_list, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_pad_ver(p_instruction_list, LV_HOR_RES / 2, 0);
    lv_obj_align(p_instruction_list, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(p_instruction_list, list_window_scroll_event_cb,
                        LV_EVENT_ALL, NULL);

    /* Create all list item UI objects (apps + any pre-existing instructions) */
    create_list_items_ui(p_instruction_list, 0, list_item_count);

    // 創建指示點
    create_indicator_dots(p_instruction_list_bg);

    lv_obj_t *ai_bar = lv_obj_create(p_instruction_list_bg);
    lv_obj_set_size(ai_bar, 80, LV_VER_RES);
    lv_obj_align(ai_bar, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(ai_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(ai_bar, 0, 0);
    lv_obj_add_event_cb(ai_bar, ai_bar_event_cb, LV_EVENT_ALL, NULL);
    // lv_obj_add_flag(ai_bar, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(ai_bar, LV_OBJ_FLAG_PRESS_LOCK);

    p_instruction_list_layout->p_instruction_list_ai_bg =
        lv_tileview_create(p_instruction_list_bg);
    lv_obj_set_scrollbar_mode(
        p_instruction_list_layout->p_instruction_list_ai_bg,
        LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(p_instruction_list_layout->p_instruction_list_ai_bg,
                    LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_0, 0);
    lv_obj_align(p_instruction_list_layout->p_instruction_list_ai_bg,
                 LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *home_page = lv_tileview_add_tile(
        p_instruction_list_layout->p_instruction_list_ai_bg, 1, 0, LV_DIR_HOR);
    lv_obj_set_size(home_page, LV_HOR_RES, LV_VER_RES);
    // lv_obj_set_style_bg_color(home_page, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(home_page, LV_OPA_0, 0);
    lv_obj_add_event_cb(home_page, home_tileview_event_cb, LV_EVENT_RELEASED,
                        NULL);
    lv_obj_t *ai_page = lv_tileview_add_tile(
        p_instruction_list_layout->p_instruction_list_ai_bg, 0, 0, LV_DIR_HOR);
    lv_obj_set_size(ai_page, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(ai_page, LV_OPA_0, 0);
    /* Stop vertical scroll from chaining up to the app_list_tileview
       (which would slide to the app grid). The AI widget should only scroll
       its own content. */
    lv_obj_clear_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                      LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_clear_flag(ai_page, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_set_tile_id(p_instruction_list_layout->p_instruction_list_ai_bg, 1,
                       0, LV_ANIM_OFF);
    lv_obj_add_event_cb(p_instruction_list_layout->p_instruction_list_ai_bg,
                        ai_tileview_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                    LV_OBJ_FLAG_HIDDEN);
    /* Gaus background to cover instruction list when widget shows */
    ai_gaus_bg = lv_img_create(ai_page);
    lv_img_set_src(ai_gaus_bg, GAUS_CLOCK1_BG);
    lv_obj_align(ai_gaus_bg, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(ai_gaus_bg, 512);
    lv_obj_add_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);

    extern lv_obj_t *lv_skai_widget_builder(lv_obj_t * parent);
    lv_obj_t *skai_widget = lv_skai_widget_builder(ai_page);
    lv_obj_align(skai_widget, LV_ALIGN_CENTER, 0, 0);

    /* Voice indicator button — replaces ai_hint at bottom center */
    ai_voice_btn = lv_obj_create(ai_page);
    lv_obj_set_size(ai_voice_btn, 62, 62);
    lv_obj_set_style_radius(ai_voice_btn, 31, 0);
    lv_obj_set_style_bg_color(ai_voice_btn, lv_color_hex(0x00AAFF), 0);
    lv_obj_set_style_bg_opa(ai_voice_btn, LV_OPA_10, 0);
    lv_obj_align(ai_voice_btn, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(ai_voice_btn, logo_click_event_cb, LV_EVENT_CLICKED,
                        NULL);
    lv_obj_t *ai_voice_img = lv_img_create(ai_voice_btn);
    lv_img_set_src(ai_voice_img, &voice_group);
    lv_obj_align(ai_voice_img, LV_ALIGN_CENTER, 0, 0);
    ai_voice_send_icon = lv_img_create(ai_voice_btn);
    lv_img_set_src(ai_voice_send_icon, ICON_SAND);
    lv_obj_align(ai_voice_send_icon, LV_ALIGN_CENTER, 2, 2);
    lv_obj_add_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    // 創建可移動範圍圓弧線
    // create_movable_range_arc(p_instruction_list_bg);
    created = true;
    myLancher[app_index_instruction_list].reset_list = reset_list;
    if (myLancher[app_index_instruction_list].reset_list != NULL)
    {
        myLancher[app_index_instruction_list].reset_list();
    }
    lv_event_send(p_instruction_list, LV_EVENT_SCROLL, NULL);

    myLancher[app_index_instruction_list].on_tap = on_tap;

#ifdef TEST_INDICATOR_ANIMATION
    start_indicator_dots_animation_test(); // 開始指示點動畫測試
#endif
    return p_instruction_list_bg;
}

// static rt_uint32_t last_scroll_time = 0;
static void scroll_list_to_index(uint16_t page)
{
    if (p_instruction_list_layout->list == NULL ||
        !lv_obj_is_valid(p_instruction_list_layout->list))
    {
        LOG_E("p_instruction_list_layout->list is NULL");
        return;
    }

    app_scroll_target_item = page;
    LOG_D("scroll_list_to_index: %d", page);
    lv_obj_t *child = lv_obj_get_child(p_instruction_list_layout->list, page);
    if (!lv_obj_is_valid(child))
    {
        LOG_W("scroll_list_to_index: child %d is invalid", page);
        return;
    }
    // lv_disp_trig_activity(NULL);
    // set_scroll_anim_time(true);
    lv_obj_scroll_to_view(child, LV_ANIM_ON);
    LOG_D("scroll_list_to_index done: %d", page);
    // set_scroll_anim_time(false);
    scroll_list(p_instruction_list_layout->list, 0);
}

static void instruction_list_scroll_to_app(int8_t action)
{
    if (pause_instruction_list)
    {
        return;
    }

    if (action >= 0 && action < list_item_count)
    {
        scroll_list_to_index(action);
    }

    else
    {
        LOG_W("Target index out of bounds: %d", action);
    }
}

/* map_app_id fills a list_item_t from an app_id enum value */
static void map_app_id(uint8_t app_id, list_item_t *item)
{
    memset(item, 0, sizeof(list_item_t));
    item->is_instruction = false;
    item->is_interval = false;
    item->enabled = false;
    item->interval_sec = 0;
    item->widget = NULL;

    const char *title = "Unknown";
    const char *icon = IMG_LOGO;
    const char *id_str = APP_ID_MAIN;

    switch (app_id)
    {
#ifdef APP_ID_SKAI
    case app_id_ai:
        title = LV_EXT_STR_GET_BY_KEY(skai_ai, "AI");
        icon = IMG_LOGO;
        id_str = APP_ID_SKAI;
        break;
#endif
#ifdef APP_ID_RECORDER
    case app_id_recorder:
        title = LV_EXT_STR_GET_BY_KEY(recorder, "Recorder");
        icon = IMG_RECORDER;
        id_str = APP_ID_RECORDER;
        break;
#endif
#ifdef APP_ID_CALENDAR
    case app_id_calendar:
        title = LV_EXT_STR_GET_BY_KEY(calendar, "Calendar");
        icon = IMG_CALENDAR;
        id_str = APP_ID_CALENDAR;
        break;
#endif
#ifdef APP_ID_WEATHER
    case app_id_weather:
        title = LV_EXT_STR_GET_BY_KEY(weather, "Weather");
        icon = IMG_GROUP;
        id_str = APP_ID_WEATHER;
        break;
#endif
#ifdef APP_ID_EXERCISE
    case app_id_exercise:
        title = LV_EXT_STR_GET_BY_KEY(exercise, "Exercise");
        icon = IMG_WORKOUT;
        id_str = APP_ID_EXERCISE;
        break;
#endif
#ifdef APP_ID_FLASHLIGHT
    case app_id_flashlight:
        title = LV_EXT_STR_GET_BY_KEY(flashlight, "Flashlight");
        icon = IMG_FLASHLIGHT;
        id_str = APP_ID_FLASHLIGHT;
        break;
#endif
#ifdef APP_ID_MEDIA
    case app_id_media:
        title = LV_EXT_STR_GET_BY_KEY(media, "Media");
        icon = IMG_ITUNES;
        id_str = APP_ID_MEDIA;
        break;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:
        title = LV_EXT_STR_GET_BY_KEY(photo, "Photo");
        icon = IMG_PHOTO;
        id_str = APP_ID_PHOTO;
        break;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur:
        title = LV_EXT_STR_GET_BY_KEY(game, "Game");
        icon = IMG_GAME;
        id_str = APP_ID_GAME_DINOSAUR;
        break;
#endif
#ifdef APP_ID_IOT_GATE
    case app_id_iot_gate:
        title = LV_EXT_STR_GET_BY_KEY(iot_gate, "iot_gate");
        icon = IMG_GAME;
        id_str = APP_ID_IOT_GATE;
        break;
#endif
#ifdef APP_ID_HEART_RATE
    case app_id_heart_rate:
        title = LV_EXT_STR_GET_BY_KEY(heart_rate, "Heart Rate");
        icon = IMG_HEART_RATE;
        id_str = APP_ID_HEART_RATE;
        break;
#endif
#ifdef APP_ID_ACTIVITY
    case app_id_activity:
        title = LV_EXT_STR_GET_BY_KEY(activity, "Activity");
        icon = IMG_ACTIVITY;
        id_str = APP_ID_ACTIVITY;
        break;
#endif
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:
        title = LV_EXT_STR_GET_BY_KEY(calculator, "Calculator");
        icon = IMG_CALCULATOR;
        id_str = APP_ID_CALCULATOR;
        break;
#endif
#ifdef APP_ID_TIMER
    case app_id_timer:
        title = LV_EXT_STR_GET_BY_KEY(timer, "Timer");
        icon = IMG_ALARM_2;
        id_str = APP_ID_TIMER;
        break;
#endif
#ifdef APP_ID_ALARM
    case app_id_alarm:
        title = LV_EXT_STR_GET_BY_KEY(alarm, "Alarm");
        icon = IMG_ALARM;
        id_str = APP_ID_ALARM;
        break;
#endif
#ifdef APP_ID_SETTING
    case app_id_setting:
        title = LV_EXT_STR_GET_BY_KEY(setting, "Setting");
        icon = IMG_SETTINGS;
        id_str = APP_ID_SETTING;
        break;
#endif
#ifdef APP_ID_MESSAGE_LIST
    case app_id_message_list:
        title = LV_EXT_STR_GET_BY_KEY(message, "Message");
        icon = IMG_MESSAGES;
        id_str = APP_ID_MESSAGE_LIST;
        break;
#endif
#ifdef APP_ID_MOUSE
    case app_id_mouse:
        title = LV_EXT_STR_GET_BY_KEY(mouse, "Mouse");
        icon = IMG_MOUSE;
        id_str = APP_ID_MOUSE;
        break;
#endif
#ifdef APP_ID_TOUCHSCREEN
    case app_id_touchscreen:
        title = LV_EXT_STR_GET_BY_KEY(touchscreen, "Touchscreen");
        icon = &img_touchscreen;
        id_str = APP_ID_TOUCHSCREEN;
        break;
#endif
#ifdef APP_ID_TOUCHPAD
    case app_id_touchpad:
        title = LV_EXT_STR_GET_BY_KEY(touchpad, "Touchpad");
        icon = &img_touchpad;
        id_str = APP_ID_TOUCHPAD;
        break;
#endif
    default:
        break;
    }

    strncpy(item->id, id_str, LIST_ITEM_ID_LEN - 1);
    item->id[LIST_ITEM_ID_LEN - 1] = '\0';
    strncpy(item->title, title, LIST_ITEM_TITLE_LEN - 1);
    item->title[LIST_ITEM_TITLE_LEN - 1] = '\0';
    item->icon = icon;
}

void load_instruction_list(void)
{
    uint8_t n = ARRAY_SIZE(INSTRUCTION_LIST_ITEMS_DEFINITION);
    for (uint8_t i = 0; i < n; i++)
    {
        map_app_id(INSTRUCTION_LIST_ITEMS_DEFINITION[i], &list_items[i]);
        LOG_D("App %d: ID=%s, Title=%s", i, list_items[i].id,
              list_items[i].title);
    }
    app_base_count = n;
    list_item_count = n;
}

static rt_int32_t init(lv_obj_t *parent)
{
    lv_instruction_list_layout_create(parent);
    return RT_EOK;
}

rt_int32_t instruction_list_resume(void)
{
    if (pause_instruction_list == false)
    {
        return RT_EOK;
    }
    open_gesture_control = false;
    // if (get_need_open_gesture_control())
    {
        set_paused_control_with_arm(false);
        open_gesture_control = true;
        // switch_watch_motion_control_mode(true, true);
        set_free_control_with_arm(true);
    }
    extern void reset_speech_coding(void);
    reset_speech_coding();
    pause_instruction_list = false;
    lvgl_msg_handler.handle_nav_bar_control = instruction_list_scroll_to_app;
    LOG_I("instruction_list_resume");
#ifdef USE_QUICK_OPEN_AI
    open_vibration = true;
#endif
    // lvgl_msg_handler.handle_widgets_control = button_selection;
    return RT_EOK;
}

rt_int32_t instruction_list_pause(void)
{
    if (pause_instruction_list == true)
    {
        return RT_EOK;
    }
    pause_instruction_list = true;
    set_paused_control_with_arm(true);
    LOG_I("instruction_list_pause");
    if (gui_app_is_actived("Main"))
    {
        if (lvgl_msg_handler.handle_nav_bar_control ==
            instruction_list_scroll_to_app)
        {
            lvgl_msg_handler.handle_nav_bar_control = NULL;
        }
        lvgl_msg_handler.handle_tap_indicator = NULL;
        lvgl_msg_handler.handle_nav_bar_control = NULL;
    }
    return RT_EOK;
}

rt_int32_t instruction_list_deinit(void)
{
    // 清理 touching_screen 定時器
    if (touching_screen_timer)
    {
        rt_timer_stop(touching_screen_timer);
        rt_timer_delete(touching_screen_timer);
        touching_screen_timer = NULL;
    }

#ifdef USE_QUICK_OPEN_AI
    if (timer_open_quick_app)
    {
        rt_timer_stop(timer_open_quick_app);
        rt_timer_delete(timer_open_quick_app);
        timer_open_quick_app = RT_NULL;
    }
#endif
    if (myLancher[app_index_instruction_list].reset_list != NULL)
    {
        myLancher[app_index_instruction_list].reset_list = NULL;
    }

    if (selected_widget_timer)
    {
        rt_timer_stop(selected_widget_timer);
        rt_timer_delete(selected_widget_timer);
        selected_widget_timer = NULL;
    }
    // 停止所有動畫並重置狀態
    stop_all_animations_and_reset();

    if (p_instruction_list_layout)
    {
        // 銷毀所有項目
        for (uint8_t i = 0; i < MAX_LIST_ITEMS; i++)
        {
            if (p_instruction_list_layout->indicator_dots[i] != NULL &&
                lv_obj_is_valid(p_instruction_list_layout->indicator_dots[i]))
            {
                lv_obj_del(p_instruction_list_layout->indicator_dots[i]);
                p_instruction_list_layout->indicator_dots[i] = NULL;
            }
            if (p_instruction_list_layout->indicator_dots_bg[i] != NULL &&
                lv_obj_is_valid(
                    p_instruction_list_layout->indicator_dots_bg[i]))
            {
                lv_obj_del(p_instruction_list_layout->indicator_dots_bg[i]);
                p_instruction_list_layout->indicator_dots_bg[i] = NULL;
            }
            if (app_widget[i] != NULL && lv_obj_is_valid(app_widget[i]))
            {
                lv_obj_del(app_widget[i]);
                app_widget[i] = NULL;
            }
            if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
            {
                lv_obj_del(touch_obj[i]);
                touch_obj[i] = NULL;
            }
            if (app_icon[i] != NULL && lv_obj_is_valid(app_icon[i]))
            {
                lv_obj_del(app_icon[i]);
                app_icon[i] = NULL;
            }
            if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
            {
                lv_obj_del(app_label[i]);
                app_label[i] = NULL;
            }
            if (p_instruction_list_layout->p_app_indicator_btn[i] != NULL &&
                lv_obj_is_valid(
                    p_instruction_list_layout->p_app_indicator_btn[i]))
            {
                lv_obj_del(p_instruction_list_layout->p_app_indicator_btn[i]);
                p_instruction_list_layout->p_app_indicator_btn[i] = NULL;
                LOG_D("instruction list Deleted p_app_indicator_btn %d", i);
            }
            switch_objs[i] = NULL;
        }

        // 銷毀可移動範圍圓弧線
        if (p_instruction_list_layout->movable_range_arc != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->movable_range_arc))
        {
            lv_obj_del(p_instruction_list_layout->movable_range_arc);
            p_instruction_list_layout->movable_range_arc = NULL;
            LOG_D("instruction list Deleted movable range arc");
        }

        if (p_instruction_list_layout->list != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->list))
        {
            lv_obj_del(p_instruction_list_layout->list);
            p_instruction_list_layout->list = NULL;
        }
        if (p_instruction_list_layout->p_instruction_list_bg != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg))
        {
            lv_obj_del(p_instruction_list_layout->p_instruction_list_bg);
            p_instruction_list_layout->p_instruction_list_bg = NULL;
        }
        if (p_instruction_list_layout->p_instruction_list_ai_bg != NULL &&
            lv_obj_is_valid(
                p_instruction_list_layout->p_instruction_list_ai_bg))
        {
            lv_obj_del(p_instruction_list_layout->p_instruction_list_ai_bg);
            p_instruction_list_layout->p_instruction_list_ai_bg = NULL;
        }
        if (p_instruction_list_layout->p_instruction_list_ai_icon != NULL &&
            lv_obj_is_valid(
                p_instruction_list_layout->p_instruction_list_ai_icon))
        {
            lv_obj_del(p_instruction_list_layout->p_instruction_list_ai_icon);
            p_instruction_list_layout->p_instruction_list_ai_icon = NULL;
        }

        if (p_instruction_list_layout->app_list_tileview != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->app_list_tileview))
        {
            lv_obj_del(p_instruction_list_layout->app_list_tileview);
            p_instruction_list_layout->app_list_tileview = NULL;
            p_instruction_list_layout->app_list_tile = NULL;
        }

        lv_mem_free(p_instruction_list_layout);
        p_instruction_list_layout = NULL;
        LOG_I("[CHECK_MEMORY]instruction_list_deinit");
    }

    // extern void media_widget_stop(void);
    // media_widget_stop();
    // extern void iot_gate_widget_stop(void);
    // iot_gate_widget_stop();
    // extern void activity_widget_stop(void);
    // activity_widget_stop();
    // extern void message_widget_stop(void);
    // message_widget_stop();
    // extern void calendar_widget_stop(void);
    // calendar_widget_stop();
    // extern void weather_widget_stop(void);
    // weather_widget_stop();
    // extern void note_widget_stop(void);
    // note_widget_stop();

    LOG_I("instruction_list_deinit");
    pause_instruction_list = true;
    return RT_EOK;
}

#if 0
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
	switch (msg)
	{
	case GUI_APP_MSG_ONSTART:
	{
		lv_obj_t *scr = lv_scr_act();
		init(scr);
	}
	break;

	case GUI_APP_MSG_ONRESUME:
		instruction_list_resume();
		break;

	case GUI_APP_MSG_ONPAUSE:
		instruction_list_pause();
		break;

	case GUI_APP_MSG_ONSTOP:
		instruction_list_deinit();
		break;
	default:
		break;
	}
}

static int app_main(intent_t i)
{
	gui_app_regist_msg_handler(APP_ID, msg_handler);

	return 0;
}
LV_IMG_DECLARE(skaiwalkicon);
BUILTIN_APP_EXPORT(LV_EXT_STR_ID(instruction_list), LV_EXT_IMG_GET(skaiwalkicon), APP_ID, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
