/**
 ******************************************************************************
 * @file   lv_app_list_layout.c
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
#include "watch_system_core_task.h"
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

#define DBG_TAG "app.list.layout"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define APP_ID "app_list"
#include <stdio.h>
#include <stdint.h>

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

typedef struct
{
    const char *title;
    const char *icon;
    lv_obj_t *widget;
    char *app_id;
} app_list_item_t;

uint16_t APP_LIST_ITEMS_DEFINITION[] = {
#ifdef APP_ID_TIMER
    app_id_timer,
#endif
    app_id_flashlight,
#ifdef APP_ID_CALCULATOR
// app_id_calculator,
#endif
    app_id_exercise,
    // app_id_heart_rate,
    app_id_recorder,
#ifdef APP_ID_ACTIVITY
// app_id_activity,
#endif
    app_id_calendar,
#ifdef APP_ID_TOUCHSCREEN
    app_id_touchscreen,
#endif
#ifdef APP_ID_PHOTO
    app_id_photo,
#endif
#ifdef APP_ID_TOUCHPAD
    app_id_touchpad,
#endif
    app_id_weather,
#ifdef APP_ID_IOT_GATE
// app_id_iot_gate,
#endif
#ifdef APP_ID_GAME_DINOSAUR
// app_id_game_dinosaur,
#endif
#ifdef APP_ID_MEDIA
// app_id_media,
#endif
#ifdef APP_ID_NOTE_CHATROOM
// app_id_note,
#endif
    // app_id_ai,
};

uint8_t return_app_count(void)
{
    LOG_D("APP_LIST_ITEMS_DEFINITION size:%d",
          ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION));
    return ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION);
}

typedef struct
{
    lv_obj_t *list;
    lv_obj_t *p_app_list_bg;
    lv_obj_t *p_app_list_ai_bg;
    lv_obj_t *p_app_list_ai_icon;
    lv_obj_t *p_app_indicator_btn[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)];
    lv_obj_t *
        indicator_dots[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)]; // 灰色指示點陣列
    lv_obj_t *indicator_dots_bg[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)];
    lv_obj_t *movable_range_arc; // 可移動範圍圓弧線
} app_list_layout_t;
static app_list_layout_t *p_app_list_layout;
static bool created = false;

static bool pause_app_list = true;

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
#ifdef APP_ID_NOTE_CHATROOM
    case app_id_note:
        return IMG_NOTE;
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

app_list_item_t app_list_items[app_id_thirty];
void load_app_list(void);

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

lv_obj_t *app_icon_shadow[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)];
static bool is_indicator_dots_visible = true;
static uint16_t selected_item_index = ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION) - 1;
static uint16_t last_zoom[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)] = {0};
static void update_indicator_dots_position(int input_value)
{
    // LOG_I("Updating indicator dots position, input value: %d", input_value);
    if (p_app_list_layout == NULL || !is_indicator_dots_visible)
        return;

    int total_dots = ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION);
    if (total_dots <= 0)
        return;

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
        if (p_app_list_layout->indicator_dots[i] == NULL)
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

        // 限制 dot_y 超過 450 或小於 16 時，dot_x 不再變動
        static int last_valid_dot_x[32] = {0};
        if (dot_y > 450 || dot_y < 16)
        {
            if (p_app_list_layout->indicator_dots_bg[i] != NULL)
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

        lv_obj_set_style_img_opa(p_app_list_layout->indicator_dots[i], opacity,
                                 0);

        uint16_t zoom =
            (uint16_t)(255 *
                       (DOT_SMOLL_PROPORTION +
                        (DOT_BIG_PROPORTION - DOT_SMOLL_PROPORTION) * ratio));
        if (abs((int)zoom - (int)last_zoom[i]) > 5)
        {
            lv_img_set_zoom(app_icon_shadow[i], zoom);
            lv_img_set_zoom(p_app_list_layout->indicator_dots[i], zoom);
            last_zoom[i] = zoom;
        }
        lv_obj_center(p_app_list_layout->indicator_dots[i]);
        dot_x -= (dot_size + 30) / 2;
        dot_y -= dot_size / 2;

        lv_obj_set_pos(p_app_list_layout->indicator_dots_bg[i], dot_x, dot_y);
    }
}

static void create_indicator_dots(lv_obj_t *parent)
{
    if (p_app_list_layout == NULL)
        return;

    int total_dots = ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION);

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
        // lv_obj_set_size(dot, 100, 100);
        lv_obj_center(dot);
        lv_img_set_src(dot, app_list_items[i].icon);
        if (APP_LIST_ITEMS_DEFINITION[i] == app_id_ai ||
            APP_LIST_ITEMS_DEFINITION[i] == app_id_note)
        {
            lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
        }

        p_app_list_layout->indicator_dots_bg[i] = dot_bg;
        p_app_list_layout->indicator_dots[i] = dot;
    }

    update_indicator_dots_position(37);
}

// static void create_ai_hint_icon(lv_obj_t *parent)
// {
//     if (p_app_list_layout == NULL)
//         return;

//     lv_obj_t *ai_hint_bg = lv_obj_create(parent);
//     lv_obj_set_size(ai_hint_bg, 80, 80);
//     lv_obj_set_style_radius(ai_hint_bg, 80, 0);
//     lv_obj_set_style_bg_color(ai_hint_bg, lv_color_hex(0x000000), 0);
//     lv_obj_set_style_bg_opa(ai_hint_bg, LV_OPA_100, 0);
//     lv_obj_align(ai_hint_bg, LV_ALIGN_RIGHT_MID, 93, 0);
//     lv_obj_set_style_border_width(ai_hint_bg, 2, 0);
//     lv_obj_set_style_border_color(ai_hint_bg, lv_color_hex(0xFFFFFF), 0);
//     lv_obj_set_style_border_opa(ai_hint_bg, LV_OPA_0, 0);
//     p_app_list_layout->p_app_list_ai_bg = ai_hint_bg;
//     lv_obj_t *ai_hint_icon = lv_img_create(ai_hint_bg);
//     lv_img_set_src(ai_hint_icon, SMALL_IMG_LOGO_MATTING);
//     lv_obj_align(ai_hint_icon, LV_ALIGN_CENTER, 0, 0);
//     p_app_list_layout->p_app_list_ai_icon = ai_hint_icon;

//     LOG_D("AI hint icon created");
// }

extern void tap_on_ai_hint(void);
static bool is_open_ai_gesture = false;
// void set_ai_hint_x(uint8_t x)
// {
//     lv_obj_align(p_app_list_layout->p_app_list_ai_bg, LV_ALIGN_RIGHT_MID,
//                  -x + 93, 0);
//     if (x > 85)
//     {
//         set_paused_control_with_arm(true);
//         lv_obj_set_style_border_opa(p_app_list_layout->p_app_list_ai_bg,
//                                     LV_OPA_COVER, 0);
//         lv_obj_set_style_img_opa(p_app_list_layout->p_app_list_ai_icon,
//                                  LV_OPA_COVER, 0);
//         if (!is_open_ai_gesture)
//         {
//             motor_pattern_unlocked();
//             is_open_ai_gesture = true;
//             tap_on_ai_hint();
//         }
//     }
//     else
//     {
//         set_paused_control_with_arm(false);
//         lv_obj_set_style_border_opa(p_app_list_layout->p_app_list_ai_bg,
//                                     LV_OPA_0, 0);
//         lv_obj_set_style_img_opa(p_app_list_layout->p_app_list_ai_icon,
//                                  LV_OPA_50, 0);
//         if (is_open_ai_gesture)
//         {
//             is_open_ai_gesture = false;
//         }
//     }
// }

void set_indicator_dots_visible(bool visible)
{
    if (is_indicator_dots_visible == visible)
        return;
    is_indicator_dots_visible = visible;
    for (int i = 0; i < ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION); i++)
    {
        if (p_app_list_layout->indicator_dots[i] != NULL)
        {
            if (visible)
            {
                lv_obj_set_style_img_opa(p_app_list_layout->indicator_dots[i],
                                         LV_OPA_60, 0);
            }
            else
            {
                lv_obj_set_style_img_opa(p_app_list_layout->indicator_dots[i],
                                         LV_OPA_20, 0);
            }
        }
    }
}

static void create_movable_range_arc(lv_obj_t *parent)
{
    if (p_app_list_layout == NULL)
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

    p_app_list_layout->movable_range_arc = arc;

    LOG_D("Movable range arc created: radius=%d, start=%d°, end=%d°",
          LIST_RADIUS / 2, MOVABLE_ARC_START_ANGLE, MOVABLE_ARC_END_ANGLE);
}

extern lv_img_dsc_t *create_widget_snapshot_img(lv_obj_t *target_obj);
lv_obj_t *app_icon[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)];
lv_obj_t *app_widget[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)];
lv_obj_t *touch_obj[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)];
lv_obj_t *app_label[ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION)];
static lv_obj_t *widget_img = NULL;
static bool left_hand_mode = true;
static bool need_correction = false;
static bool is_widget_animation_active = false; // 追蹤 widget 動畫狀態
static uint8_t app_scroll_target_item = 0;
static uint16_t old_selected_item_index = -1;
static lv_obj_t *selected_label;
static lv_obj_t *app_list_main_status_bar;
static rt_tick_t last_gohame_time = 0;

void set_arc_stripe_external_offset(int16_t offset_degrees)
{
    update_indicator_dots_position(offset_degrees);
}

static void animate_open_selected_widget_cb(lv_anim_t *a)
{
    if (app_widget[selected_item_index] != NULL &&
        lv_obj_is_valid(app_widget[selected_item_index]))
    {
        lv_obj_clear_flag(app_widget[selected_item_index], LV_OBJ_FLAG_HIDDEN);
    }
    if (touch_obj[selected_item_index] != NULL &&
        lv_obj_is_valid(touch_obj[selected_item_index]))
    {
        lv_obj_clear_flag(touch_obj[selected_item_index], LV_OBJ_FLAG_HIDDEN);
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

    for (uint8_t i = 0; i < ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION); i++)
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
static bool is_open_app_list_ai = false;
bool get_is_open_app_list_ai(void)
{
    return is_open_app_list_ai;
}
void set_is_open_app_list_ai(bool open)
{
    is_open_app_list_ai = open;
}
static bool is_at_ai_widget = false;
static bool scroll_initialized = false;
static bool touching_screen = false;
static lv_obj_t *app_list_page = NULL;

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

    if (app_list_page->coords.y1 == -466)
        need_correction = true;
    else
        need_correction = false;

    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = obj->spec_attr->children[i];
        widget_hight = LIST_ITEM_WIDGET_HEIGHT;
        lv_coord_t y_center = child->coords.y1 + widget_hight / 2;
        if (need_correction)
        {
            y_diff = y_center - LV_VER_RES / 2 + 466;
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
        static lv_coord_t last_y_diff[32] = {0};    // 假設最大32個app
        static uint8_t last_brightness[32] = {255}; // 儲存上次的亮度值
        static uint8_t last_zoom[32] = {0};
        const lv_coord_t DIFF_THRESHOLD = 15; // 變化超過5才更新

        if (abs((int)y_diff - (int)last_y_diff[i]) > DIFF_THRESHOLD)
        {
            last_y_diff[i] = y_diff;
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
            // uint8_t zoom;
            // // 讓 zoom 隨 y_diff 線性變化，y_diff 越大 zoom 越小
            // if (y_diff >= 200) {
            //     zoom = 255 * 0.7;
            // } else {
            //     // zoom 從 255 (y_diff=0) 線性降到 255*0.7 (y_diff=200)
            //     zoom = 255*1.2 - (y_diff * (255*1.2 - (uint8_t)(255 * 0.7)) /
            //     200);
            // }
            if (brightness != last_brightness[i])
            {
                // 使用顏色深淺代替透明度，創建從白色到灰色的漸變
                lv_color_t text_color =
                    lv_color_make(brightness, brightness, brightness);
                lv_obj_set_style_text_color(app_label[i], text_color, 0);
                last_brightness[i] = brightness;
            }
            // if (zoom != last_zoom[i]) {
            //     // lv_img_set_zoom(p_app_list_layout->indicator_dots[i],
            //     zoom); last_zoom[i] = zoom;
            // }
            // if (i == 9)
            //     LOG_D("App %d: y_diff=%d, opa=%d, zoom=%d", i, y_diff, opa,
            //     zoom);
            // lv_obj_center(p_app_list_layout->indicator_dots[i]);
        }
    }
    if (touching_screen)
    {
        // int target_value = (child_cnt - selected_item_index) * 125 - 63;//+
        // selected_item_y_diff - 63
        int target_value = child_cnt * 100 + first_y_diff - 63;
        if (target_value < 0)
        {
            target_value = 0;
        }
        else if (target_value > get_total_moving_distance())
        {
            target_value = get_total_moving_distance();
        }
        if (target_value < 0)
        {
            target_value = 0;
        }
        else if (target_value > get_total_moving_distance())
        {
            target_value = get_total_moving_distance();
        }
        set_prev_sensor_quat(target_value);
        if (SkaiWatchSys.motion_control_lock)
        {
            update_indicator_dots_position(target_value);
        }
    }
    if (selected_item_index != old_selected_item_index)
    {
        if (selected_item_index == APP_LIST_ITEMS_DEFINITION[app_id_ai])
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
                LOG_D("Selected item index: %d", i);
                // lv_img_set_zoom(p_app_list_layout->indicator_dots[i], 256);
                // selected_label = lv_obj_get_child(child, 0);
                lv_obj_clear_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
                if (APP_LIST_ITEMS_DEFINITION[i] != app_id_note &&
                    APP_LIST_ITEMS_DEFINITION[i] != app_id_ai)
                {
                    lv_obj_clear_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                }

                lv_obj_clear_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);
                // 將選中項的文本標籤放大一個字號
                // lv_obj_set_style_text_font(
                //     app_label[i], LV_EXT_FONT_GET(get_system_font_size(1)),
                //     0);
            }
            else
            {
                // lv_img_set_zoom(p_app_list_layout->indicator_dots[i],
                //                 256 * 0.75);
                lv_obj_add_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                // 將非選中項的文本標籤恢復為正常字號
                // lv_obj_set_style_text_font(
                //     app_label[i], LV_EXT_FONT_GET(get_system_font_size(0)),
                //     0);
            }
            {
                int distance_from_the_center = i - selected_item_index;
                if (abs(distance_from_the_center) > 1 || true)
                {
                    lv_obj_align(app_icon[i], LV_ALIGN_RIGHT_MID, -25, 0);
                    lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
                }
                // if (APP_LIST_ITEMS_DEFINITION[i] != app_id_note &&
                //     APP_LIST_ITEMS_DEFINITION[i] != app_id_ai)
                // {
                //     lv_obj_clear_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                // }
                lv_obj_set_style_border_opa(lv_obj_get_child(child, 0),
                                            LV_OPA_10, LV_STATE_DEFAULT);
            }
        }
        if (get_scrolling_motor_vibrate_status())
        {
            motor_pattern_scrolling_app();
        }
    }
    // if (selected_item_index == child_cnt - 1)
    // {
    // 	if ((last_y_diff - last_y_diff_on_selected) < -150 && rt_tick_get() -
    // last_gohame_time > 500) // last_y_diff < -100
    // 	{
    // 		last_gohame_time = rt_tick_get();
    // 		animate_to_home_from_app_list();
    // 	}
    // }
}

static uint8_t prev_app_scroll_target_item = 0;
void open_selected_widget(bool need_widget_img_anima)
{
    if (is_widget_animation_active || created == false)
    {
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
            lv_img_create(p_app_list_layout->p_app_list_bg);
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
            animate_icon_vertical(app_icon[selected_item_index - 1], true);
            animate_label_vertical(app_label[selected_item_index - 1], true);
        }
        if (selected_item_index != ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION) - 1)
        {
            animate_icon_vertical(app_icon[selected_item_index + 1], false);
            animate_label_vertical(app_label[selected_item_index + 1], false);
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
            if (lv_obj_is_valid(app_icon[selected_item_index - 1]))
            {
                lv_obj_align(app_icon[selected_item_index - 1],
                             LV_ALIGN_RIGHT_MID, -20, -30);
            }

            if (lv_obj_is_valid(app_label[selected_item_index - 1]))
            {
                lv_obj_align(app_label[selected_item_index - 1],
                             LV_ALIGN_CENTER, -20, -30);
            }
        }
        if (selected_item_index != ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION) - 1)
        {
            lv_obj_align(app_icon[selected_item_index + 1], LV_ALIGN_RIGHT_MID,
                         -20, 30);
            lv_obj_align(app_label[selected_item_index + 1], LV_ALIGN_CENTER, -30,
                         30);
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

extern void set_skai_widget_opa(uint8_t opa);
extern void set_skai_widget_input_text(const char *text);
static void set_ai_bg_opa(void *obj, int32_t opa)
{
    uint8_t bg_opa = 240 * opa / 255;
    lv_obj_set_style_bg_opa(p_app_list_layout->p_app_list_ai_bg, bg_opa, 0);
    set_skai_widget_opa(opa);
}

static rt_tick_t last_ai_widget_open_time = 0;
void animate_open_ai_widget(void)
{
    if (!get_bluetooth_connection_status())
    {
        create_connection_tips();
        LOG_D("Bluetooth is connected, ignoring voice recognition event");
        return;
    }
    last_ai_widget_open_time = rt_tick_get();
    // animate_to_page(p_app_list_layout->p_app_list_ai_bg, 300);
    lv_obj_clear_flag(p_app_list_layout->p_app_list_ai_bg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_tile_id(p_app_list_layout->p_app_list_ai_bg, 0, 0, LV_ANIM_ON);
}

void close_ai_widget(void)
{
    // lv_anim_del(p_app_list_layout->p_app_list_ai_bg, set_ai_bg_opa);
    // lv_obj_add_flag(p_app_list_layout->p_app_list_ai_bg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_tile_id(p_app_list_layout->p_app_list_ai_bg, 1, 0, LV_ANIM_ON);
    // set_skai_widget_opa(0);
    // LOG_I("AI widget closed");
}

void check_ai_widget_auto_close(void)
{
    extern bool get_skai_input_text_is_null(void);
    if (is_open_app_list_ai && !is_at_ai_widget && get_skai_input_text_is_null())
    {
        rt_tick_t current_time = rt_tick_get();
        if (current_time - last_ai_widget_open_time < 3000) // 5秒后自动关闭
        {
            close_ai_widget();
            is_open_app_list_ai = false;
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
    // if (is_open_app_list_ai)
    // {
    //     extern void send_to_ai(void);
    //     send_to_ai();
    //     return;
    // }
    // lv_obj_clear_flag(p_app_list_layout->p_app_list_ai_bg,
    // LV_OBJ_FLAG_HIDDEN); lv_anim_t a; lv_anim_init(&a); lv_anim_set_var(&a,
    // p_app_list_layout->p_app_list_ai_bg); lv_anim_set_values(&a,
    // LV_OPA_TRANSP, LV_OPA_COVER); lv_anim_set_time(&a, 300);
    // lv_anim_set_exec_cb(&a, set_ai_bg_opa);
    // lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    // lv_anim_start(&a);
    set_ai_bg_opa(NULL, LV_OPA_COVER);
    LOG_D("AI widget opened");
    is_open_app_list_ai = true;
    open_skai_widget_ai(true);
    // animate_to_ai_page();
    set_skai_widget_input_text("");
    set_ai_open_mic(true);
    show_speech_indicator(true);
    voice_provider.start_v2t();
    // set_free_control_with_arm(false);
    set_paused_control_with_arm(true);
}


static bool app_list_ai_tapped = false;
bool get_app_list_ai_tapped(void)
{
    return app_list_ai_tapped;
}
void set_app_list_ai_tapped(void)
{
    app_list_ai_tapped = false;
}
void tap_on_ai_hint(void)
{
    if (!get_bluetooth_connection_status())
    {
        create_connection_tips();
        LOG_D("Bluetooth is connected, ignoring voice recognition event");
        return;
    }
    app_list_ai_tapped = true;
    extern void send_to_ai(void);
    send_to_ai();
    animate_to_home_from_app_list();
    animate_to_ai_page();
    close_ai_widget();
}

extern void iot_gate_widget_tap_event_cb(void);
extern void media_widget_tap_event_cb(void);
static void on_item_tap(app_list_item_t *item)
{
    LOG_D("on_item_tap: %s", item->app_id);
    // if (is_open_ai_gesture)
    // {
    // 	tap_on_ai_hint();
    // }
    // else
    // if (strcmp(item->app_id, APP_ID_SKAI) == 0)
    // {
    //     if (is_open_app_list_ai)
    //     {
    //         if (!isTextEmpty())
    //             tap_on_ai_hint();
    //         else
    //             LOG_D("AI input is empty, ignoring tap");
    //     }
    //     else
    //     {
    //         tap_on_ai_widget();
    //     }
    // }
    if (is_open_app_list_ai)
    {
        if (!isTextEmpty())
            tap_on_ai_hint();
        else
            LOG_D("AI input is empty, ignoring tap");
    }
    else if (!is_open_app_list_ai)
    {
        animate_to_home_from_app_list();
        gui_app_run(item->app_id);
    }
}
static void list_item_click_event_cb(lv_event_t *evt)
{
    app_list_item_t *item = (app_list_item_t *)evt->user_data;
    lv_obj_t *obj = evt->target;
    LOG_D("ID: %s,obj:%p", item->app_id, obj);
    if (strcmp(item->app_id, APP_ID_SKAI) == 0)
    {
        tap_on_ai_hint();
    }
    else
    {
        on_item_tap(item);
    }
}

static bool tap_to_open_control = false;
static bool open_quick_app = false;
static void on_tap(void)
{
    LOG_D("open app TEST1");
    on_item_tap(&app_list_items[selected_item_index]);
}

static int16_t find_app_index_by_id(uint16_t app_id)
{
    for (int i = 0; i < ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION); i++)
    {
        if (APP_LIST_ITEMS_DEFINITION[i] == app_id)
        {
            return i;
        }
    }
    return ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION) - 1; // 未找到
}

extern char *get_media_title(void);
extern bool is_have_message_now(void);
static uint16_t gesture_starting_value = 0;
static void reset_list(void)
{
    if (p_app_list_layout->list == NULL)
    {
        return;
    }
    disable_scrolling_motor_vibrate();
    set_paused_control_with_arm(false);
    scroll_initialized = false;
    uint8_t scroll_to_index;
    uint16_t page_range =
        1250 / ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION); // 保持原值
    // const char *media_title = get_media_title();
    // if (media_title[0] != '\0')
    // {
    //     scroll_to_index = find_app_index_by_id(app_id_media);
    //     app_scroll_target_item = find_app_index_by_id(app_id_media);
    // }
    // else
    // if (is_have_message_now())
    // {
    //     scroll_to_index = find_app_index_by_id(app_id_message_list);
    //     app_scroll_target_item = find_app_index_by_id(app_id_message_list);
    // }
    // else
    {
        scroll_to_index = find_app_index_by_id(app_id_ai);
        app_scroll_target_item = find_app_index_by_id(app_id_ai);
    }
    gesture_starting_value =
        (100 * (ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION) - scroll_to_index - 1)) +
        37;

    selected_item_index = app_scroll_target_item;
    prev_app_scroll_target_item = app_scroll_target_item;
    lv_obj_t *child =
        lv_obj_get_child(p_app_list_layout->list, scroll_to_index);
    lv_obj_scroll_to_view(child, LV_ANIM_OFF);
    if (!scroll_initialized)
    {
        scroll_list(p_app_list_layout->list, 0);
    }
    update_indicator_dots_position(gesture_starting_value);
    open_selected_widget(false);
    is_widget_animation_active = false;
    enable_scrolling_motor_vibrate();
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

    if (swich_quick_btn == 1 && is_at_app_list())
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
void set_app_list_indicator_dots_position(int input_value)
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
    set_app_list_indicator_dots_position((int)value);
    LOG_D("Animation test: input_value = %d", (int)value);
}

// 開始動畫測試
void start_indicator_dots_animation_test(void)
{
    if (p_app_list_layout == NULL || !created)
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
    set_app_list_indicator_dots_position(500);
}
#endif

extern void set_ai_open_mic(bool is_open);
static void logo_click_event_cb(lv_event_t *evt)
{
    // set_ai_open_mic(true);
    // extern void tap_on_ai_widget(void);
    // tap_on_ai_widget();
    if (!isTextEmpty())
        tap_on_ai_hint();
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
        lv_obj_clear_flag(p_app_list_layout->p_app_list_ai_bg,
                          LV_OBJ_FLAG_HIDDEN);
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
        lv_obj_set_style_bg_opa(p_app_list_layout->p_app_list_ai_bg,
                                calculated_opa, 0);
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
            // is_open_app_list_ai = false;
            voice_provider.stop_v2t();
            stop_voice_recognition(V2T_INTENT_NOTHING);
            set_is_open_app_list_ai(false);
            open_skai_widget_ai(false);
            set_paused_control_with_arm(false);
            set_ai_open_mic(false);
            // show_speech_indicator(false);
            // voice_provider.stop_v2t();
            // close_ai_widget();
            lv_obj_add_flag(p_app_list_layout->p_app_list_ai_bg,
                            LV_OBJ_FLAG_HIDDEN);
        }
        else if (active_pos == 1)
        {
            set_ai_open_mic(true);
            tap_on_ai_widget();
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

static lv_obj_t *ai_tileview = NULL;
lv_obj_t *lv_app_list_layout_create(lv_obj_t *parent)
{
    // 檢查是否已經分配，如果是則先釋放
    if (p_app_list_layout != NULL)
    {
        LOG_W("p_app_list_layout already exists, cleaning up...");
        app_list_deinit();
    }
    size_t allocate_size = sizeof(app_list_layout_t);
    p_app_list_layout =
        (app_list_layout_t *)lv_mem_alloc(sizeof(app_list_layout_t));
    if (p_app_list_layout == NULL)
    {
        LOG_E("Failed to allocate memory for p_app_list_layout");
        return NULL;
    }
    memset(p_app_list_layout, 0, sizeof(app_list_layout_t));
    LOG_I("[CHECK_MEMORY]app_list_init(%d bytes)", allocate_size);
    app_list_page = parent;

    extern void media_widget_start(void);
    media_widget_start();
    extern void iot_gate_widget_start(void);
    iot_gate_widget_start();
    extern void recorder_widget_start(void);
    recorder_widget_start();
    extern void activity_widget_start(void);
    activity_widget_start();
    extern void calendar_widget_start(void);
    calendar_widget_start();
    extern void weather_widget_start(void);
    weather_widget_start();
    extern void note_widget_start(void);
    note_widget_start();

    load_app_list();
    lv_obj_t *p_app_list_bg = lv_obj_create(parent);
    p_app_list_layout->p_app_list_bg = p_app_list_bg;
    lv_obj_set_style_bg_opa(p_app_list_bg, LV_OPA_0, 0);
    lv_obj_set_size(p_app_list_bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(p_app_list_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(p_app_list_bg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *p_app_list = lv_obj_create(p_app_list_bg);
    p_app_list_layout->list = p_app_list;
    LOG_D("p_app_list: %p", p_app_list);
    lv_obj_set_size(p_app_list, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(p_app_list, LV_OPA_0, 0);
    lv_obj_add_flag(p_app_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(p_app_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_app_list, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(p_app_list, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_pad_ver(p_app_list, LV_HOR_RES / 2, 0);
    lv_obj_align(p_app_list, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(p_app_list, list_window_scroll_event_cb, LV_EVENT_ALL,
                        NULL);

    for (uint8_t i = 0; i < ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION); i++)
    {
        lv_obj_t *widget = NULL;
        lv_obj_t *item = lv_simplified_obj_create(p_app_list);
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

        bool has_widget = true;

        if (APP_LIST_ITEMS_DEFINITION[i] == app_id_note)
        {
            lv_obj_set_pos(item, 0,
                           (LIST_ITEM_WIDGET_HEIGHT + LIST_ITEM_SPACING) * i +
                               (130 + LIST_ITEM_SPACING));
            extern lv_obj_t *lv_note_widget_builder(lv_obj_t * parent);
            widget = lv_note_widget_builder(item);
        }
        else if (APP_LIST_ITEMS_DEFINITION[i] == app_id_ai)
        {
            extern lv_obj_t *lv_skai_widget_builder(lv_obj_t * parent);
            widget = lv_skai_widget_builder(item);
        }
        else
        {
            widget = NULL;
            has_widget = false;
        }

        if (has_widget && widget != NULL)
        {

            lv_obj_set_size(widget, LIST_ITEM_WIDGET_WIDTH,
                            LIST_ITEM_WIDGET_HEIGHT);
            if (APP_LIST_ITEMS_DEFINITION[i] != app_id_ai)
            {
                lv_obj_set_style_clip_corner(widget, true, 0);
            }
            else
            {
                lv_obj_add_flag(widget, LV_OBJ_FLAG_SCROLLABLE);
            }
            if (APP_LIST_ITEMS_DEFINITION[i] != app_id_ai)
            {
                lv_obj_set_style_border_color(widget, lv_color_hex(0xFFFFFF),
                                              0);
                lv_obj_set_style_border_width(widget, 2, 0);
                lv_obj_set_style_border_opa(widget, LV_OPA_20, 0);
                lv_obj_add_event_cb(widget, list_item_click_event_cb,
                                    LV_EVENT_CLICKED,
                                    (void *)&app_list_items[i]);
            }
        }

        touch_obj[i] = lv_obj_create(item);
        lv_obj_set_size(touch_obj[i], LIST_ITEM_WIDGET_WIDTH,
                        LIST_ITEM_WIDGET_HEIGHT);
        lv_obj_set_style_bg_opa(touch_obj[i], LV_OPA_0, 0);
        lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_align(touch_obj[i], LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_event_cb(touch_obj[i], list_item_click_event_cb,
                            LV_EVENT_CLICKED, (void *)&app_list_items[i]);

        app_widget[i] = widget;
        p_app_list_layout->p_app_indicator_btn[i] = lv_img_create(item);
        lv_img_set_src(p_app_list_layout->p_app_indicator_btn[i],
                       app_list_items[i].icon);
        lv_obj_align(p_app_list_layout->p_app_indicator_btn[i],
                     LV_ALIGN_RIGHT_MID, -25, 0);
        app_icon[i] = p_app_list_layout->p_app_indicator_btn[i];
        lv_obj_add_event_cb(app_icon[i], list_item_click_event_cb,
                            LV_EVENT_CLICKED, (void *)&app_list_items[i]);
        lv_obj_add_flag(app_icon[i], LV_OBJ_FLAG_HIDDEN);

        app_label[i] = lv_label_create(item);
        lv_label_set_text(app_label[i], app_list_items[i].title);
        lv_obj_set_style_text_font(app_label[i],
                                   LV_EXT_FONT_GET(get_system_font_size(1)), 0);
        lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
        if (APP_LIST_ITEMS_DEFINITION[i] == app_id_note ||
            APP_LIST_ITEMS_DEFINITION[i] == app_id_ai)
        {
            lv_obj_add_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // 創建指示點
    create_indicator_dots(p_app_list_bg);

    lv_obj_t *ai_bar = lv_obj_create(p_app_list_bg);
    lv_obj_set_size(ai_bar, 80, LV_VER_RES);
    lv_obj_align(ai_bar, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(ai_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(ai_bar, 0, 0);
    lv_obj_add_event_cb(ai_bar, ai_bar_event_cb, LV_EVENT_ALL, NULL);
    // lv_obj_add_flag(ai_bar, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(ai_bar, LV_OBJ_FLAG_PRESS_LOCK);

    p_app_list_layout->p_app_list_ai_bg = lv_tileview_create(p_app_list_bg);
    lv_obj_set_scrollbar_mode(p_app_list_layout->p_app_list_ai_bg,
                              LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(p_app_list_layout->p_app_list_ai_bg, LV_HOR_RES,
                    LV_VER_RES);
    lv_obj_set_style_bg_opa(p_app_list_layout->p_app_list_ai_bg, LV_OPA_0, 0);
    lv_obj_align(p_app_list_layout->p_app_list_ai_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *home_page = lv_tileview_add_tile(
        p_app_list_layout->p_app_list_ai_bg, 1, 0, LV_DIR_HOR);
    lv_obj_set_size(home_page, LV_HOR_RES, LV_VER_RES);
    // lv_obj_set_style_bg_color(home_page, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(home_page, LV_OPA_0, 0);
    lv_obj_t *ai_page = lv_tileview_add_tile(
        p_app_list_layout->p_app_list_ai_bg, 0, 0, LV_DIR_HOR);
    lv_obj_set_size(ai_page, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(ai_page, LV_OPA_0, 0);
    lv_obj_set_tile_id(p_app_list_layout->p_app_list_ai_bg, 1, 0, LV_ANIM_OFF);
    lv_obj_add_event_cb(p_app_list_layout->p_app_list_ai_bg,
                        ai_tileview_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(p_app_list_layout->p_app_list_ai_bg, LV_OBJ_FLAG_HIDDEN);

    // create_ai_hint_icon(p_app_list_bg);
    // p_app_list_layout->p_app_list_ai_bg = lv_obj_create(ai_page);
    // lv_obj_set_size(p_app_list_layout->p_app_list_ai_bg, 466, 466);
    // lv_obj_align(p_app_list_layout->p_app_list_ai_bg, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_set_style_bg_color(p_app_list_layout->p_app_list_ai_bg,
    //                           lv_color_hex(0x000000), 0);
    // lv_obj_set_style_bg_opa(p_app_list_layout->p_app_list_ai_bg, 240, 0);
    extern lv_obj_t *lv_skai_widget_builder(lv_obj_t * parent);
    lv_obj_t *skai_widget = lv_skai_widget_builder(ai_page);
    lv_obj_align(skai_widget, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *ai_hint = lv_img_create(ai_page);
    lv_obj_set_size(ai_hint, 80, 80);
    lv_img_set_src(ai_hint, IMG_LOGO);
    lv_obj_align(ai_hint, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_t *ai_hint_btn = lv_obj_create(ai_hint);
    lv_obj_set_size(ai_hint_btn, 80, 80);
    lv_obj_set_style_bg_opa(ai_hint_btn, LV_OPA_TRANSP, 0);
    lv_obj_add_event_cb(ai_hint_btn, logo_click_event_cb, LV_EVENT_CLICKED,
                        NULL);
    lv_obj_align(ai_hint_btn, LV_ALIGN_CENTER, 0, 0);

    // 創建可移動範圍圓弧線
    // create_movable_range_arc(p_app_list_bg);
    created = true;
    myLancher[app_index_app_list].reset_list = reset_list;
    if (myLancher[app_index_app_list].reset_list != NULL)
    {
        myLancher[app_index_app_list].reset_list();
    }

    lv_event_send(p_app_list, LV_EVENT_SCROLL, NULL);

    myLancher[app_index_app_list].on_tap = on_tap;

#ifdef TEST_INDICATOR_ANIMATION
    start_indicator_dots_animation_test(); // 開始指示點動畫測試
#endif
    return p_app_list_bg;
}

// static rt_uint32_t last_scroll_time = 0;
static void scroll_list_to_index(uint16_t page)
{
    if (p_app_list_layout->list == NULL ||
        !lv_obj_is_valid(p_app_list_layout->list))
    {
        LOG_E("p_app_list_layout->list is NULL");
        return;
    }

    app_scroll_target_item = page;
    LOG_D("scroll_list_to_index: %d", page);
    lv_obj_t *child = lv_obj_get_child(p_app_list_layout->list, page);
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
    scroll_list(p_app_list_layout->list, 0);
}

static void app_list_scroll_to_app(int8_t action)
{
    if (pause_app_list)
    {
        return;
    }

    if (action >= 0 && action < ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION))
    {
        scroll_list_to_index(action);
    }

    else
    {
        LOG_W("Target index out of bounds: %d", action);
    }
}

app_list_item_t map_app_id(uint8_t app_id)
{
    app_list_item_t item;
    switch (app_id)
    {
#ifdef APP_ID_SKAI
    case app_id_ai:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(skai_ai, "AI");
        item.icon = IMG_LOGO;
        item.app_id = APP_ID_SKAI;
    }
    break;
#endif
#ifdef APP_ID_RECORDER
    case app_id_recorder:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(recorder, "Recorder");
        item.icon = IMG_RECORDER;
        item.app_id = APP_ID_RECORDER;
    }
    break;
#endif
#ifdef APP_ID_NOTE_CHATROOM
    case app_id_note:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(note, "Note");
        item.icon = IMG_NOTE;
        item.app_id = APP_ID_NOTE_CHATROOM;
    }
    break;
#endif

#ifdef APP_ID_CALENDAR
    case app_id_calendar:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(calendar, "Calendar");
        item.icon = IMG_CALENDAR;
        item.app_id = APP_ID_CALENDAR;
        break;
    }
#endif

#ifdef APP_ID_WEATHER
    case app_id_weather:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(weather, "Weather");
        item.icon = IMG_GROUP;
        item.app_id = APP_ID_WEATHER;
        break;
    }
#endif

#ifdef APP_ID_EXERCISE
    case app_id_exercise:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(exercise, "Exercise");
        item.icon = IMG_WORKOUT;
        item.app_id = APP_ID_EXERCISE;
        break;
    }
#endif

#ifdef APP_ID_FLASHLIGHT
    case app_id_flashlight:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(flashlight, "Flashlight");
        item.icon = IMG_FLASHLIGHT;
        item.app_id = APP_ID_FLASHLIGHT;
        break;
    }
#endif

#ifdef APP_ID_MEDIA
    case app_id_media:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(media, "Media");
        item.icon = IMG_ITUNES;
        item.app_id = APP_ID_MEDIA;
    }
    break;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(photo, "Photo");
        item.icon = IMG_PHOTO;
        item.app_id = APP_ID_PHOTO;
    }
    break;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(game, "Game");
        item.icon = IMG_GAME;
        item.app_id = APP_ID_GAME_DINOSAUR;
    }
    break;
#endif

#ifdef APP_ID_IOT_GATE
    case app_id_iot_gate:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(iot_gate, "iot_gate");
        item.icon = IMG_GAME;
        item.app_id = APP_ID_IOT_GATE;
    }
    break;
#endif

#ifdef APP_ID_HEART_RATE
    case app_id_heart_rate:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(heart_rate, "Heart Rate");
        item.icon = IMG_HEART_RATE;
        item.app_id = APP_ID_HEART_RATE;
        break;
    }
#endif

#ifdef APP_ID_ACTIVITY
    case app_id_activity:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(activity, "Activity");
        item.icon = IMG_ACTIVITY;
        item.app_id = APP_ID_ACTIVITY;
        break;
    }
#endif

#ifdef APP_ID_CALCULATOR
    case app_id_calculator:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(calculator, "Calculator");
        item.icon = IMG_CALCULATOR;
        item.app_id = APP_ID_CALCULATOR;
        break;
    }
#endif

#ifdef APP_ID_TIMER
    case app_id_timer:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(timer, "Timer");
        item.icon = IMG_ALARM_2;
        item.app_id = APP_ID_TIMER;
        break;
    }

#endif

#ifdef APP_ID_ALARM
    case app_id_alarm:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(alarm, "Alarm");
        item.icon = IMG_ALARM;
        item.app_id = APP_ID_ALARM;
        break;
    }
#endif

#ifdef APP_ID_SETTING
    case app_id_setting:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(setting, "Setting");
        item.icon = IMG_SETTINGS;
        item.app_id = APP_ID_SETTING;
        break;
    }
#endif

#ifdef APP_ID_MESSAGE_LIST
    case app_id_message_list:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(message, "Message");
        item.icon = IMG_MESSAGES;
        item.app_id = APP_ID_MESSAGE_LIST;
        break;
    }
#endif

#ifdef APP_ID_MOUSE
    case app_id_mouse:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(mouse, "Mouse");
        item.icon = IMG_MOUSE;
        item.app_id = APP_ID_MOUSE;
        break;
    }
#endif

#ifdef APP_ID_TOUCHSCREEN
    case app_id_touchscreen:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(touchscreen, "Touchscreen");
        item.icon = &img_touchscreen;
        item.app_id = APP_ID_TOUCHSCREEN;
        break;
    }
#endif

#ifdef APP_ID_TOUCHPAD
    case app_id_touchpad:
    {
        item.title = LV_EXT_STR_GET_BY_KEY(touchpad, "Touchpad");
        item.icon = &img_touchpad;
        item.app_id = APP_ID_TOUCHPAD;
        break;
    }
#endif

    default:
    {
        item.title = "Unknown";
        item.icon = IMG_LOGO;
        item.app_id = APP_ID_MAIN;
        break;
    }
    }
    return item;
}

void load_app_list(void)
{
    uint8_t n = ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION);
    for (uint8_t i = 0; i < n; i++)
    {
        app_list_items[i] = map_app_id(APP_LIST_ITEMS_DEFINITION[i]);
        LOG_D("App %d: ID=%d, Title=%s", i, app_list_items[i].app_id,
              app_list_items[i].title);
    }
}

static rt_int32_t init(lv_obj_t *parent)
{
    lv_app_list_layout_create(parent);
    return RT_EOK;
}

rt_int32_t app_list_resume(void)
{
    if (pause_app_list == false)
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
    pause_app_list = false;
    lvgl_msg_handler.handle_nav_bar_control = app_list_scroll_to_app;
    LOG_I("app_list_resume");
#ifdef USE_QUICK_OPEN_AI
    open_vibration = true;
#endif
    request_weather_within_six_hours(false);
    request_calendar_on_mobile(false);
    // watch_sys_sync.request_pedometer_data();
    // lvgl_msg_handler.handle_widgets_control = button_selection;
    return RT_EOK;
}

rt_int32_t app_list_pause(void)
{
    if (pause_app_list == true)
    {
        return RT_EOK;
    }
    pause_app_list = true;
    set_paused_control_with_arm(true);
    LOG_I("app_list_pause");
    if (gui_app_is_actived("Main"))
    {
        if (lvgl_msg_handler.handle_nav_bar_control == app_list_scroll_to_app)
        {
            lvgl_msg_handler.handle_nav_bar_control = NULL;
        }
        lvgl_msg_handler.handle_tap_indicator = NULL;
        lvgl_msg_handler.handle_nav_bar_control = NULL;
    }
    // set_ai_hint_x(0);
    // if (lvgl_msg_handler.handle_widgets_control == button_selection)
    // {
    // 	lvgl_msg_handler.handle_widgets_control = NULL;
    // }
    // extern void open_skai_widget_ai(bool open);
    // open_skai_widget_ai(false);

    return RT_EOK;
}

rt_int32_t app_list_deinit(void)
{
    // Delete the timer if it exists
    extern void close_note_chatroom_ui_app(void);
    close_note_chatroom_ui_app();

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
    if (myLancher[app_index_app_list].reset_list != NULL)
    {
        myLancher[app_index_app_list].reset_list = NULL;
    }

    if (selected_widget_timer)
    {
        rt_timer_stop(selected_widget_timer);
        rt_timer_delete(selected_widget_timer);
        selected_widget_timer = NULL;
    }
    // 停止所有動畫並重置狀態
    stop_all_animations_and_reset();

    if (p_app_list_layout)
    {
        // 銷毀所有指示點
        for (uint8_t i = 0; i < ARRAY_SIZE(APP_LIST_ITEMS_DEFINITION); i++)
        {
            if (lv_obj_is_valid(p_app_list_layout->indicator_dots[i]))
            {
                lv_obj_del(p_app_list_layout->indicator_dots[i]);
                p_app_list_layout->indicator_dots[i] = NULL;
            }
            if (lv_obj_is_valid(p_app_list_layout->indicator_dots_bg[i]))
            {
                lv_obj_del(p_app_list_layout->indicator_dots_bg[i]);
                p_app_list_layout->indicator_dots_bg[i] = NULL;
            }
            if (lv_obj_is_valid(app_widget[i]))
            {
                lv_obj_del(app_widget[i]);
                app_widget[i] = NULL;
            }
            if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
            {
                lv_obj_del(touch_obj[i]);
                touch_obj[i] = NULL;
            }
            if (lv_obj_is_valid(app_icon[i]))
            {
                lv_obj_del(app_icon[i]);
                app_icon[i] = NULL;
            }
            if (lv_obj_is_valid(app_label[i]))
            {
                lv_obj_del(app_label[i]);
                app_label[i] = NULL;
            }
            if (lv_obj_is_valid(p_app_list_layout->p_app_indicator_btn[i]))
            {
                lv_obj_del(p_app_list_layout->p_app_indicator_btn[i]);
                p_app_list_layout->p_app_indicator_btn[i] = NULL;
                LOG_D("app list Deleted p_app_indicator_btn %d", i);
            }
        }

        // 銷毀可移動範圍圓弧線
        if (lv_obj_is_valid(p_app_list_layout->movable_range_arc))
        {
            lv_obj_del(p_app_list_layout->movable_range_arc);
            p_app_list_layout->movable_range_arc = NULL;
            LOG_D("app list Deleted movable range arc");
        }

        if (lv_obj_is_valid(p_app_list_layout->list))
        {
            lv_obj_del(p_app_list_layout->list);
            p_app_list_layout->list = NULL;
        }
        if (lv_obj_is_valid(p_app_list_layout->p_app_list_bg))
        {
            lv_obj_del(p_app_list_layout->p_app_list_bg);
            p_app_list_layout->p_app_list_bg = NULL;
        }
        if (lv_obj_is_valid(p_app_list_layout->p_app_list_ai_bg))
        {
            lv_obj_del(p_app_list_layout->p_app_list_ai_bg);
            p_app_list_layout->p_app_list_ai_bg = NULL;
        }
        if (lv_obj_is_valid(p_app_list_layout->p_app_list_ai_icon))
        {
            lv_obj_del(p_app_list_layout->p_app_list_ai_icon);
            p_app_list_layout->p_app_list_ai_icon = NULL;
        }

        lv_mem_free(p_app_list_layout);
        p_app_list_layout = NULL;
        LOG_I("[CHECK_MEMORY]app_list_deinit");
    }

    extern void media_widget_stop(void);
    media_widget_stop();
    extern void iot_gate_widget_stop(void);
    iot_gate_widget_stop();
    extern void recorder_widget_stop(void);
    recorder_widget_stop();
    extern void activity_widget_stop(void);
    activity_widget_stop();
    extern void message_widget_stop(void);
    message_widget_stop();
    extern void calendar_widget_stop(void);
    calendar_widget_stop();
    extern void weather_widget_stop(void);
    weather_widget_stop();
    extern void note_widget_stop(void);
    note_widget_stop();

    LOG_I("app_list_deinit");
    pause_app_list = true;
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
		app_list_resume();
		break;

	case GUI_APP_MSG_ONPAUSE:
		app_list_pause();
		break;

	case GUI_APP_MSG_ONSTOP:
		app_list_deinit();
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
BUILTIN_APP_EXPORT(LV_EXT_STR_ID(app_list), LV_EXT_IMG_GET(skaiwalkicon), APP_ID, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/