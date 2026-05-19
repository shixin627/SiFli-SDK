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
#include "arc_scroll.h"
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
LV_IMG_DECLARE(plus);
LV_IMG_DECLARE(icon_mic);
LV_IMG_DECLARE(message_widget_bg);

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

#define DOT_SMOLL_PROPORTION (0.5)
#define DOT_BIG_PROPORTION (1.3)
#define DOT_BG_SIZE (100 * DOT_BIG_PROPORTION) + 2
/* 縮放曲線指數：1.0 = 線性、2.0 = 平方（中央放大效果突出，邊緣下降快）、
 * 3.0 = 立方（更陡峭）。值越大，「中央 dot 顯著大、其他 dot 都很小」越明顯 */
#define DOT_ZOOM_EXPONENT 2.0f

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
    app_id_recorder,
#ifdef APP_ID_PHOTO
// app_id_photo,
#endif
// app_id_weather,
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
    lv_obj_t *mic_bar; /* bottom mic-trigger bar; hidden when AI widget open */
    lv_obj_t *p_app_indicator_btn[MAX_LIST_ITEMS];
    lv_obj_t *indicator_dots[MAX_LIST_ITEMS];
    lv_obj_t *indicator_dots_bg[MAX_LIST_ITEMS];
    lv_obj_t *movable_range_arc; // 可移動範圍圓弧線
    lv_obj_t
        *app_list_tileview;  // vertical tileview: instruction list + app grid
    lv_obj_t *app_list_tile; // tile 1: app grid page
    arc_scroll_handle_t *arc_handle; // 共用 arc-scroll 模組 instance
} instruction_list_layout_t;
static instruction_list_layout_t *p_instruction_list_layout;
static bool created = false;

static bool pause_instruction_list = true;

const char *get_app_icon(uint8_t app_id)
{
    switch (app_id)
    {
#ifdef APP_ID_RECORDER
    case app_id_recorder:
        return IMG_RECORDER;
#endif
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
#ifdef APP_ID_MOUSE
    case app_id_mouse:
        return IMG_MOUSE;
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

/* SKAIBAR option-tracking session lifetime — decoupled from
   is_open_instruction_list_ai. The voice/v2t session ends when the
   pill fades (close_ai_widget), but founder direction 2026-05-19 is
   that the phone-side SKAIBAR option highlight should keep following
   the watch's scroll position even after the pill is gone. The
   session ends only when the user leaves the instruction_list page
   entirely (instruction_list_pause).
   Set true inside tap_on_ai_widget (the moment the user invokes the
   SKAIBAR flow); cleared inside instruction_list_pause. Declared up
   here so scroll_list (line ~790) and tap_on_ai_widget /
   instruction_list_pause (further down) all see the same storage. */
static bool s_skaibar_tracking_active = false;

/* arc-scroll detached / discrete 模式狀態 — 拖動時 arc 不動 list、由 drag_cb
 * 接管，到 page change 才 snap。完整定義在後面，scroll_list 要先 visible。
 * 用獨立 bool flag 而不是 input 的特殊值當「是否已初始化」 — elastic overshoot
 * 會讓 input 掉到負值（min_input - 50 = -13），不能再用 <0 當 sentinel */
static bool s_inst_arc_drag_active = false;
static bool s_inst_drag_initialized = false;
static int s_inst_drag_input = 0;
static int s_inst_drag_last_idx = -1;    /* 上一次中央的 dot idx */
static void inst_arc_reset_drag_state(void);
static void scroll_list_to_index(uint16_t page);

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

    /* 跟 app_exercise.c::apply_circular_layout 對齊：圓心在螢幕正中、
     * radius=200。原本 (120, 233, 300) 那組會讓 arc 中心偏左、半徑大、
     * dots 上下散開比較廣，視覺上跟 exercise 不一樣。
     * center_x 往左偏 30 px：中央 dot zoom 到 1.3x（130 px 寬）時，沒偏的話
     * 右邊會跑出螢幕；偏 30 後最右邊大約在 448，剛好在 466 螢幕內 */
    const int circle_radius = 200;
    const int center_x = LV_HOR_RES / 2 - 20;
    const int center_y = LV_VER_RES / 2;

    const float angle_per_dot = 36.0f; /* 跟 app_exercise.c 的 ICON_SLOT_ANGLE_DEG=36 對齊 */

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
        /* 不做 [0,360) normalize — 留 signed angle，方便用 |angle| > 90 一刀
         * 過濾掉「在 list 第一格時 dot N-1 從另一邊 wrap 過來出現在上方」的問題。
         * 例如 N=10 顆 dot，第一格時 dot 9 的 base_angle = 9*36 = 324°，wrap 後
         * 變成 (270, 360) 區間 → 既有 (90,270) 過濾擋不到 → 出現在右上方。
         * signed_angle = 324°（不 wrap）→ > 90° → 直接 hide */
        float current_angle = base_angle - offset_angle;

        float angle_rad = current_angle * M_PI / 180.0f;

        int dot_x = center_x + (int)(circle_radius * cos(angle_rad));
        int dot_y = center_y + (int)(circle_radius * sin(angle_rad));

        /* 用 |signed angle| > 90° 一次過濾掉左半圓 + 從另一邊繞回來的 dots */
        {
            lv_obj_t *dot_bg_obj =
                p_instruction_list_layout->indicator_dots_bg[i];
            if (current_angle < -90.0f || current_angle > 90.0f)
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

        /* 跟 app_exercise.c::apply_circular_layout 一致：用 cos(abs_angle) 做
         * 平滑漸層。current_angle 現在是 signed [-90,90]，直接 fabsf 就是
         * 從水平右軸算起的 abs_angle */
        float abs_angle_deg = fabsf(current_angle);
        float ratio = cosf(abs_angle_deg * (float)M_PI / 180.0f);

        int dot_size = DOT_BG_SIZE;
        int opacity = (int)(LV_OPA_30 + (LV_OPA_COVER - LV_OPA_30) * ratio);
        if (opacity < LV_OPA_30)
            opacity = LV_OPA_30;
        if (opacity > LV_OPA_COVER)
            opacity = LV_OPA_COVER;

        lv_obj_set_style_img_opa(p_instruction_list_layout->indicator_dots[i],
                                 opacity, 0);

        /* 用指數曲線 ratio^N 取代線性 ratio：N>1 時，中央 dot 大幅放大，
         * 邊緣 dot 快速縮小，視覺上中央更突出 */
        float zoom_ratio = powf(ratio, DOT_ZOOM_EXPONENT);
        uint16_t zoom =
            (uint16_t)(255 *
                       (DOT_SMOLL_PROPORTION +
                        (DOT_BIG_PROPORTION - DOT_SMOLL_PROPORTION) * zoom_ratio));
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

/* fwd decl — dot click 直接走跟 touch_obj 同樣的 click handler，省去重複邏輯 */
static void list_item_click_event_cb(lv_event_t *evt);

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
        /* 仿 app_exercise.c 把 icon 本身設成可點擊：tap 任一可見 dot 直接觸發
         * 對應 item 的 click handler，不需要先把它 scroll 到中央再點 */
        lv_obj_add_flag(dot_bg, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(dot_bg, list_item_click_event_cb,
                            LV_EVENT_CLICKED, (void *)&list_items[i]);

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
/* Transcript label inside the styled skai_widget pill — shows the spoken
   text after voice_say (PC sim) or real ASR (real hw). */
static lv_obj_t *s_voice_transcript_label = NULL;
/* Reference to the styled pill (lv_skai_widget_builder return value).
   For scroll-fade we animate per-property opa on multiple objects (LVGL
   lv_obj_set_style_opa does not cascade in this build — must use
   _bg_opa / _border_opa / _text_opa / _img_opa explicitly). */
static lv_obj_t *s_skai_widget = NULL;
/* The voice-group image inside ai_voice_btn — needs img_opa fade. */
static lv_obj_t *s_voice_img = NULL;
/* The pill background image (message_widget_bg) — gives the pill its
   border + shape on real-hw where the LVGL border style is invisible. */
static lv_obj_t *s_pill_bg_img = NULL;

void instruction_list_set_voice_transcript(const char *text)
{
    if (s_voice_transcript_label && lv_obj_is_valid(s_voice_transcript_label))
    {
        lv_label_set_text(s_voice_transcript_label, text ? text : "");
    }
}

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
                /* drag_cb 模式下 selected_item_index 由 page-change snap 統一管理，
                 * scroll_list 不要再從 card y_diff 推回去（會跟 snap 動畫打架）*/
                if (!s_inst_arc_drag_active)
                {
                    selected_item_index = i;
                }
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

        /* 指示點也用全部項目範圍。drag_cb 模式下 dot 由 inst_arc_drag_cb 用累積
         * 的 input 自己更新，scroll_list 不要再用 list scroll_y 反推蓋過去 */
        int dots_value = child_cnt * 100 + first_y_diff - 63;
        if (SkaiWatchSys.motion_control_lock && !s_inst_arc_drag_active)
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
        /* SKAIBAR option-tracking: report the scrolled-into-focus custom
           instruction as the current skaibar option, same wire format
           as hid_mouse arc selection. Gated on s_skaibar_tracking_active
           rather than is_open_instruction_list_ai so the phone-side
           option highlight keeps following the watch's scroll even
           after the user dismisses the input pill (founder direction
           2026-05-19). Tracking is armed in tap_on_ai_widget and
           cleared in instruction_list_pause. Only fires for custom
           instructions (selected >= app_base_count); the app slots
           before the custom list aren't part of the skaibar option
           set. Dedup is the outer selected_item_index != old check,
           so we don't spam the same idx on every scroll-list re-compute. */
        if (s_skaibar_tracking_active &&
            selected_item_index >= app_base_count)
        {
            uint8_t skaibar_idx =
                (uint8_t)(selected_item_index - app_base_count);
            commu_send_skaibar_selected(skaibar_idx);
            LOG_D("[skaibar] sent selected idx=%u (raw=%u, base=%u)",
                  (unsigned)skaibar_idx,
                  (unsigned)selected_item_index,
                  (unsigned)app_base_count);
        }
        old_selected_item_index = selected_item_index;
        // LOG_D("selected_app_index: %d", selected_item_index);

        LOG_D("DBGinner-loop start child_cnt=%d sel=%d", child_cnt, selected_item_index);
        for (uint8_t i = 0; i < child_cnt; i++)
        {
            lv_obj_t *child = obj->spec_attr->children[i];
            LOG_D("DBGi=%d child=%p", i, child);
            if (i == selected_item_index)
            {
                LOG_D("instruction DEBUG Selected item index: %d", i);
                if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
                    lv_obj_clear_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
                if (!list_items[i].is_instruction)
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
            LOG_D("DBGi=%d before app_icon", i);
            if (app_icon[i] != NULL && lv_obj_is_valid(app_icon[i]))
                lv_obj_align(app_icon[i], LV_ALIGN_RIGHT_MID, -25, 0);
            LOG_D("DBGi=%d before app_label", i);
            if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
            LOG_D("DBGi=%d before get_child", i);
            {
                lv_obj_t *first_child = lv_obj_get_child(child, 0);
                LOG_D("DBGi=%d first_child=%p", i, first_child);
                if (first_child)
                    lv_obj_set_style_border_opa(first_child, LV_OPA_10,
                                                LV_STATE_DEFAULT);
                LOG_D("DBGi=%d after set_style_border_opa", i);
            }
        }
        LOG_D("DBGinner-loop end");
        if (get_scrolling_motor_vibrate_status() && open_scroll_motor)
        {
            LOG_D("DBGcalling motor_pattern_scrolling_app");
            motor_pattern_scrolling_app();
            LOG_D("DBGafter motor_pattern_scrolling_app");
        }
        LOG_D("DBGscroll_list returning");
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

/* Forward decls — defined further down with the AI widget block. */
static void ai_widget_fade_on_scroll(void);
void close_ai_widget(void);
/* Gate for ai_widget_fade_on_scroll: set true while refresh_custom_instructions
   does programmatic scrolls. Declared here (not at refresh_* location) so
   list_window_scroll_event_cb can read it. Actually defined below. */
static bool s_in_refresh_scroll = false;

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
        /* Modal-ish dismiss: scrolling the instruction list while the AI
           widget is open fades it out (per office-hours doc Q4).
           Gate on !s_in_refresh_scroll — programmatic scrolls inside
           refresh_custom_instructions raise this flag, so only user
           drags (LVGL direct scroll OR arc_scroll-driven scroll) get
           through. This also works on PC sim (touch_sim drags reach
           the list directly via LVGL pointer indev). */
        if (!s_in_refresh_scroll)
        {
            ai_widget_fade_on_scroll();
        }
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

/* Called from app_skai.c when speech text is updated.
 *
 * Was Phase 2 of a three-element reveal: (a) show ai_gaus_bg Gaussian
 * blur backdrop, (b) raise p_instruction_list_ai_bg bg_opa from 0→50
 * (dark scrim), (c) fade skai_widget pill in. User direction
 * 2026-05-19: the input pill should appear in isolation — no scrim,
 * no blur backdrop, so the instruction list behind it stays readable.
 * Only the pill fade-in (c) survives. send_icon stays revealed because
 * it sits inside the pill, not as a separate backdrop element. */
void instruction_ai_show_skai_widget(void)
{
    if (!is_open_instruction_list_ai || !p_instruction_list_layout)
        return;
    if (skai_widget_shown)
        return;
    skai_widget_shown = true;
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
    {
        lv_obj_clear_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
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
/* Guard flag: when animate_open_ai_widget drives the tileview from tile 1 → 0
   with LV_ANIM_ON, the resulting SCROLL_END fires VALUE_CHANGED with
   active_pos=0. Without the guard, ai_tileview_event_cb would interpret that
   as "user dismissed to home" and close the widget we just opened. */
static bool s_programmatic_open_in_progress = false;

/* Mock instruction-update timer — simulates phone/PC pushing list updates
   while voice listening is active. Real backend wiring deferred per office-hours
   doc §5 P2 (latency unverified). Cycles a small fake set. */
static rt_timer_t mock_inst_update_timer = RT_NULL;
static int mock_inst_cycle = 0;
static const char *MOCK_INST_TITLES[] = {
    "Set 5 min timer",
    "Send 'on my way' to Mom",
    "Play workout playlist",
    "Add milk to grocery list",
};
static const char *MOCK_INST_IDS[] = {
    "mock-timer-001",
    "mock-msg-002",
    "mock-music-003",
    "mock-grocery-004",
};
#define MOCK_INST_COUNT (sizeof(MOCK_INST_IDS) / sizeof(MOCK_INST_IDS[0]))

static void mock_inst_update_cb(void *param)
{
    int idx = mock_inst_cycle % MOCK_INST_COUNT;
    add_or_update_custom_instruction(MOCK_INST_IDS[idx], MOCK_INST_TITLES[idx],
                                     "once", 0, false, mock_inst_cycle + 1);
    mock_inst_cycle++;
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_INSTRUCTION_LIST};
    lvgl_send_msg(msg);
}

static void start_mock_inst_update(void)
{
    if (mock_inst_update_timer == RT_NULL)
    {
        /* SOFT timer required: callback calls add_or_update_custom_instruction
           (strncpy on shared array) + lvgl_send_msg. HARD timer = interrupt
           context, unsafe for either. SOFT timer runs in dedicated timer
           thread context, safe to call kernel APIs. */
        mock_inst_update_timer = rt_timer_create(
            "mock_inst_upd", mock_inst_update_cb, RT_NULL,
            rt_tick_from_millisecond(1500),
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (mock_inst_update_timer != RT_NULL)
    {
        rt_timer_start(mock_inst_update_timer);
        LOG_I("mock instruction update timer started");
    }
}

static void stop_mock_inst_update(void)
{
    if (mock_inst_update_timer != RT_NULL)
    {
        rt_timer_stop(mock_inst_update_timer);
        LOG_I("mock instruction update timer stopped");
    }
}

/* Click handler for the bottom mic bar — re-uses the same flow that the
   release IMU gesture takes (see gesture_recognition_task.c:401). */
static void mic_bar_event_cb(lv_event_t *evt)
{
    if (evt->code == LV_EVENT_CLICKED)
    {
        LOG_I("Mic bar tapped — opening AI widget");
        extern void animate_open_ai_widget(void);
        animate_open_ai_widget();
    }
}

/* Fade-out the AI widget when the user scrolls the instruction list while
   the widget is open. Modal-ish dismiss model (per office-hours doc Q4):
   scroll = explicit dismissal; user re-invokes via bar tap or release. */
static bool ai_fade_anim_active = false;
static void ai_fade_done_cb(lv_anim_t *a)
{
    ai_fade_anim_active = false;
    if (is_open_instruction_list_ai)
    {
        close_ai_widget();
    }
}

/* Baseline opacities for each animated property when fully visible.
   Match the values set in lv_instruction_list_layout_create. */
#define PILL_BG_IMG_BASELINE  LV_OPA_COVER
#define TRANSCRIPT_BASELINE   LV_OPA_80
#define VOICEBTN_BG_BASELINE  LV_OPA_10
#define VOICEIMG_BASELINE     LV_OPA_COVER

/* Apply a fade fraction (0..255) to all visual sub-parts of the pill.
   value=255 → fully visible (each property = its baseline).
   value=0   → fully invisible. LVGL's lv_obj_set_style_opa does NOT
   cascade in this build, so each sub-part gets its own per-property
   opa call: img_opa for images, text_opa for labels, bg_opa for solid
   bg fills. */
static void skai_widget_fade_anim_cb(void *var, int32_t value)
{
    (void)var;
    lv_opa_t f = (lv_opa_t)value;  /* 0..255 fade factor */
    if (s_pill_bg_img && lv_obj_is_valid(s_pill_bg_img))
    {
        lv_obj_set_style_img_opa(s_pill_bg_img, f, 0);
    }
    if (s_voice_transcript_label && lv_obj_is_valid(s_voice_transcript_label))
    {
        lv_obj_set_style_text_opa(s_voice_transcript_label,
                                  (lv_opa_t)((TRANSCRIPT_BASELINE * f) / 255),
                                  0);
    }
    if (ai_voice_btn && lv_obj_is_valid(ai_voice_btn))
    {
        lv_obj_set_style_bg_opa(ai_voice_btn,
                                (lv_opa_t)((VOICEBTN_BG_BASELINE * f) / 255), 0);
    }
    if (s_voice_img && lv_obj_is_valid(s_voice_img))
    {
        lv_obj_set_style_img_opa(s_voice_img, f, 0);
    }
}

static void ai_widget_fade_on_scroll(void)
{
    if (!is_open_instruction_list_ai || ai_fade_anim_active)
        return;
    if (!s_skai_widget || !lv_obj_is_valid(s_skai_widget))
        return;
    ai_fade_anim_active = true;
    /* Single fade fraction drives all sub-parts. 255 → 0 over 600ms
       ease-out gives a smooth dissolve that tracks the scroll motion. */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_skai_widget);
    lv_anim_set_values(&a, 255, 0);
    lv_anim_set_time(&a, 600);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, skai_widget_fade_anim_cb);
    lv_anim_set_ready_cb(&a, ai_fade_done_cb);
    lv_anim_start(&a);
}

/* Reset all visual sub-parts to their baseline (fully-visible) opacities.
   Called by animate_open_ai_widget so a prior fade doesn't leave the
   pill stuck at low opacity. */
static void skai_widget_restore_full_opa(void)
{
    if (s_pill_bg_img && lv_obj_is_valid(s_pill_bg_img))
    {
        lv_obj_set_style_img_opa(s_pill_bg_img, PILL_BG_IMG_BASELINE, 0);
    }
    if (s_voice_transcript_label && lv_obj_is_valid(s_voice_transcript_label))
    {
        lv_obj_set_style_text_opa(s_voice_transcript_label, TRANSCRIPT_BASELINE,
                                  0);
    }
    if (ai_voice_btn && lv_obj_is_valid(ai_voice_btn))
    {
        lv_obj_set_style_bg_opa(ai_voice_btn, VOICEBTN_BG_BASELINE, 0);
    }
    if (s_voice_img && lv_obj_is_valid(s_voice_img))
    {
        lv_obj_set_style_img_opa(s_voice_img, VOICEIMG_BASELINE, 0);
    }
}

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
    /* Cancel any in-flight scroll-fade so the widget doesn't immediately
       fade back out after the slide-in completes. */
    if (ai_fade_anim_active)
    {
        if (s_skai_widget && lv_obj_is_valid(s_skai_widget))
        {
            lv_anim_del(s_skai_widget, skai_widget_fade_anim_cb);
        }
        ai_fade_anim_active = false;
    }
    /* Restore all sub-part opacities to their baselines (in case a prior
       fade left them dim). */
    skai_widget_restore_full_opa();
    /* Slide-from-left animation: pre-position at tile 1 (home, x=466), then
       animate to tile 0 (ai_page, x=0). The s_programmatic_open_in_progress
       guard makes ai_tileview_event_cb skip its close branch when our
       animated settle fires VALUE_CHANGED at active_pos=0. */
    lv_obj_clear_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                      LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(p_instruction_list_layout->p_instruction_list_ai_bg);
    /* Hide the mic-trigger bar — AI widget is now active, its own voice
       button handles further interaction. */
    if (p_instruction_list_layout->mic_bar &&
        lv_obj_is_valid(p_instruction_list_layout->mic_bar))
    {
        lv_obj_add_flag(p_instruction_list_layout->mic_bar,
                        LV_OBJ_FLAG_HIDDEN);
    }
    s_programmatic_open_in_progress = true;
    lv_obj_set_tile_id(p_instruction_list_layout->p_instruction_list_ai_bg, 1,
                       0, LV_ANIM_OFF);
    tap_on_ai_widget();
    lv_obj_set_tile_id(p_instruction_list_layout->p_instruction_list_ai_bg, 0,
                       0, LV_ANIM_ON);
    /* set_tile_id fires SCROLL events that drive bg_opa via ai_tileview_event_cb.
       Keep the OUTER strip transparent — only the styled skai_widget pill
       inside should be visible, matching the Liquid Glass design language. */
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_0, 0);
    /* Auto-cycling mock list updates DISABLED — the `voice_say <text>` MSH
       command provides explicit, on-demand voice-input simulation instead.
       Uncomment to re-enable the passive cycling demo. */
    /* start_mock_inst_update(); */

    /* PC sim only: wire VAD-status messages to the mic-flash animation.
       On real hardware bloc_v2t.c sets this handler when voice_provider.start_v2t
       runs; the PC sim stub is no-op so we need to wire it ourselves to make
       LVGL_MSG_TYPE_VAD_STATUS (from MSH `voice_say`) light up the mic button. */
#ifdef BSP_USING_PC_SIMULATOR
    extern void handle_ai_voice_btn_vad(bool speaking);
    lvgl_msg_handler.handle_vad_status = handle_ai_voice_btn_vad;
#endif
}

void close_ai_widget(void)
{
    extern void clear_skai_widget_ai_reply(void);
    stop_mock_inst_update();
    /* Restore the mic-trigger bar so user can re-invoke the AI widget. */
    if (p_instruction_list_layout && p_instruction_list_layout->mic_bar &&
        lv_obj_is_valid(p_instruction_list_layout->mic_bar))
    {
        lv_obj_clear_flag(p_instruction_list_layout->mic_bar,
                          LV_OBJ_FLAG_HIDDEN);
    }
    /* Tear down v2t. Three calls because the event-driven STOP path in
       voice_recognition_entry has a "skip if AI processing" early-return
       that leaves voice_recognition_started=true — so the next mic-bar
       tap's VOICE_RECOGNITION_START event short-circuits and the phone
       never gets a fresh handshake. We hit all three teardown surfaces
       directly so the dismiss is unconditional:
         1. voice_provider.stop_v2t()  — fires the async STOP event
            (which may or may not run cleanly depending on AI state)
         2. stop_voice_recognition()  — sync: clears voice2TextStatus,
            unsubscribes mic, sends user-speaking-end notify
         3. set_voice_recognition_started(false)  — sync: clears the
            re-entry gate the event handler normally clears. Without
            this, the next tap can't restart v2t when AI is still
            processing the prior utterance. */
    voice_provider.stop_v2t();
    stop_voice_recognition(V2T_INTENT_NOTHING);
    set_voice_recognition_started(false);
    set_paused_control_with_arm(false);
    set_ai_open_mic(false);
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
    /* Founder direction 2026-05-19: instruction_list AI widget mic
       routes spoken input to the skaibar / mouse-mode pipeline instead
       of the chat pipeline. Setting the one-shot override before
       start_v2t() makes voice_recognition_entry pass V2T_INTENT_SKAIBAR
       (0x03) to start_voice_recognition + notify_user_speaking_intent
       — the phone treats the resulting transcript as a skaibar command,
       not a chat query. The override auto-resets to CHAT after the
       START handler runs so no other callsite is affected. */
    voice_set_pending_v2t_intent(V2T_INTENT_SKAIBAR);
    voice_provider.start_v2t();
    /* Arm SKAIBAR option-tracking — scrolls will start reporting idx
       to the phone via commu_send_skaibar_selected. Survives the
       widget close (pill fade), cleared on instruction_list_pause. */
    s_skaibar_tracking_active = true;
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
        /* SKAIBAR commit — tap on a custom instruction while the
           tracking session is alive (mic was opened at some point on
           this page, page not yet paused). Send the option idx the
           phone uses as its action trigger. Independent of the
           AI-widget visibility, the existing send_instruction_update
           flow below, and the toggle/flash visuals — phone may use
           the COMMITTED notify alongside, e.g. to invoke a script. */
        if (s_skaibar_tracking_active)
        {
            for (uint8_t j = 0; j < list_item_count; j++)
            {
                if (&list_items[j] == item && j >= app_base_count)
                {
                    commu_send_skaibar_committed(
                        (uint8_t)(j - app_base_count));
                    LOG_D("[skaibar] committed idx=%u (raw=%u)",
                          (unsigned)(j - app_base_count), (unsigned)j);
                    break;
                }
            }
        }
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
        /* SKAIBAR commit — same rationale as list_item_click_event_cb,
           parallel gesture path. selected_item_index is the raw
           list_items index; subtract app_base_count for the 0-based
           skaibar-option index the phone expects. */
        if (s_skaibar_tracking_active &&
            selected_item_index >= app_base_count)
        {
            commu_send_skaibar_committed(
                (uint8_t)(selected_item_index - app_base_count));
            LOG_D("[skaibar] committed via gesture idx=%u (raw=%u)",
                  (unsigned)(selected_item_index - app_base_count),
                  (unsigned)selected_item_index);
        }
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

static void reset_list_internal(void)
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

/* Thread-aware wrapper assigned to myLancher[...].reset_list. Callers in
   bloc_notification (KE_EVT2) hit reset_list_internal's lv_obj_scroll_to_view
   which cascades through scroll events and blows the 4KB BLE stack. Defer to
   the LVGL thread when invoked off-thread. */
static void reset_list(void)
{
    if (!is_on_lvgl_thread())
    {
        lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_RESET_INSTRUCTION_LIST};
        lvgl_send_msg(msg);
        return;
    }
    reset_list_internal();
}

/* Public entry called by the LVGL_MSG_TYPE_RESET_INSTRUCTION_LIST handler.
   Always invoked from the LVGL thread, so no thread check needed. */
void apply_instruction_list_reset_on_lvgl_thread(void)
{
    reset_list_internal();
}

uint16_t get_gesture_starting_value(void)
{
    return gesture_starting_value;
}

extern void media_widget_trigger_drag_by_py(int p_y);
static void button_selection(gesture_position_t gesture_position)
{
    const int p_y = gesture_position.gesture_position_y;
    if (selected_item_index == find_app_index_by_id(app_id_media))
    {
        media_widget_trigger_drag_by_py(p_y);
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

static lv_obj_t *complete_on_phone_tip_window = NULL;
static lv_obj_t *complete_on_phone_tip_label = NULL;
static lv_timer_t *complete_on_phone_tip_timer = NULL;

static void complete_on_phone_tip_fade_ready_cb(lv_anim_t *anim)
{
    if (lv_obj_is_valid(complete_on_phone_tip_window))
    {
        lv_obj_del(complete_on_phone_tip_window);
        complete_on_phone_tip_window = NULL;
        complete_on_phone_tip_label = NULL;
    }
}

static void complete_on_phone_tip_set_opa(void *obj, int32_t opa)
{
    if (lv_obj_is_valid(complete_on_phone_tip_window))
    {
        uint8_t window_opa = opa * LV_OPA_90 / LV_OPA_100;
        lv_obj_set_style_bg_opa(complete_on_phone_tip_window, window_opa,
                                LV_PART_MAIN);
        uint8_t border_opa = opa * LV_OPA_50 / LV_OPA_100;
        lv_obj_set_style_border_opa(complete_on_phone_tip_window, border_opa,
                                    LV_PART_MAIN);
        if (complete_on_phone_tip_label)
        {
            lv_obj_set_style_text_opa(complete_on_phone_tip_label, opa,
                                      LV_PART_MAIN);
        }
    }
}

static void complete_on_phone_tip_timer_cb(lv_timer_t *timer)
{
    if (complete_on_phone_tip_timer)
    {
        lv_timer_del(complete_on_phone_tip_timer);
        complete_on_phone_tip_timer = NULL;
    }
    if (lv_obj_is_valid(complete_on_phone_tip_window))
    {
        lv_anim_t fade_out_anim;
        lv_anim_init(&fade_out_anim);
        lv_anim_set_var(&fade_out_anim, complete_on_phone_tip_window);
        lv_anim_set_exec_cb(&fade_out_anim, complete_on_phone_tip_set_opa);
        lv_anim_set_values(&fade_out_anim, LV_OPA_100, LV_OPA_TRANSP);
        lv_anim_set_time(&fade_out_anim, 600);
        lv_anim_set_path_cb(&fade_out_anim, lv_anim_path_ease_in);
        lv_anim_set_ready_cb(&fade_out_anim,
                             complete_on_phone_tip_fade_ready_cb);
        lv_anim_start(&fade_out_anim);
    }
}

static void show_complete_on_phone_tip(void)
{
    if (complete_on_phone_tip_timer)
    {
        lv_timer_del(complete_on_phone_tip_timer);
        complete_on_phone_tip_timer = NULL;
    }
    if (lv_obj_is_valid(complete_on_phone_tip_window))
    {
        lv_obj_del(complete_on_phone_tip_window);
        complete_on_phone_tip_window = NULL;
        complete_on_phone_tip_label = NULL;
    }

    complete_on_phone_tip_window = lv_obj_create(lv_layer_top());
    lv_obj_set_size(complete_on_phone_tip_window, 260, 70);
    lv_obj_align(complete_on_phone_tip_window, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(complete_on_phone_tip_window,
                              lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(complete_on_phone_tip_window, LV_OPA_90,
                            LV_PART_MAIN);
    lv_obj_set_style_border_color(complete_on_phone_tip_window,
                                  lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(complete_on_phone_tip_window, 1,
                                  LV_PART_MAIN);
    lv_obj_set_style_border_opa(complete_on_phone_tip_window, LV_OPA_50,
                                LV_PART_MAIN);
    lv_obj_set_style_radius(complete_on_phone_tip_window, 35, LV_PART_MAIN);
    lv_obj_clear_flag(complete_on_phone_tip_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(complete_on_phone_tip_window, LV_OBJ_FLAG_CLICKABLE);

    complete_on_phone_tip_label = lv_label_create(complete_on_phone_tip_window);
    lv_label_set_text(complete_on_phone_tip_label, "在手機上完成");
    lv_obj_set_style_text_color(complete_on_phone_tip_label, lv_color_white(),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(complete_on_phone_tip_label,
                               LV_EXT_FONT_GET(get_system_font_size(1)),
                               LV_PART_MAIN);
    lv_obj_center(complete_on_phone_tip_label);

    complete_on_phone_tip_timer =
        lv_timer_create(complete_on_phone_tip_timer_cb, 1000, NULL);
    lv_timer_set_repeat_count(complete_on_phone_tip_timer, 1);
}

static void add_instruction_btn_event_cb(lv_event_t *evt)
{
    LOG_I("add_inst_btn clicked: show 'complete on phone' tip");
    if (get_bluetooth_connection_status())
    {
        const char *json = "{\"action\":\"add\"}";
        LOG_I("Send create-instruction request: %s", json);
        commu_send_update_instruction(json);
    }
    show_complete_on_phone_tip();
}

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
/* Was 240 = dark backdrop frame around the AI widget. Set to 0 so the
   styled skai_widget pill (Liquid Glass aesthetic) sits cleanly over the
   list without a darker enclosing rectangle. */
static uint16_t ai_bg_opa = 0;
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
            /* Skip close path if we're inside our own animated open —
               this VALUE_CHANGED is the settle of animate_open_ai_widget's
               tile 1→0 anim, not a user dismissal. */
            if (s_programmatic_open_in_progress)
            {
                s_programmatic_open_in_progress = false;
                break;
            }
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

        /* Default visibility: only the currently selected item exposes its
           label / switch / touch overlay. scroll_list normally maintains this
           when the selection changes, but on initial create / rebuild no
           scroll has fired yet, so non-selected items would otherwise show
           every label at once. */
        if (i != selected_item_index)
        {
            if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                lv_obj_add_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
            if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                lv_obj_add_flag(switch_objs[i], LV_OBJ_FLAG_HIDDEN);
            if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
                lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
        }

        LOG_D("List item %d: id=%s, title=%s, is_instruction=%d", i,
              list_items[i].id, list_items[i].title,
              list_items[i].is_instruction);
    }
}

static rt_tick_t s_last_refresh_tick = 0;
static lv_timer_t *s_pending_refresh_timer = NULL;
/* s_in_refresh_scroll is declared earlier in the file (near the forward
   decls for list_window_scroll_event_cb) so the scroll handler can read
   it. It is set true for the duration of refresh_custom_instructions(). */
/* Set by external callers (e.g. voice_say MSH demo) to make the next
   refresh_custom_instructions auto-scroll to the newly added last item
   so the user sees the addition without manually scrolling. */
static bool s_force_scroll_to_last = false;
/* When set, refresh_custom_instructions overrides the indicator-dot input
   value so the dots span the BOTTOM half of the arc (angles 0..+72°) with
   the newest item at angle +72°. Without this, the natural "selected=last"
   layout places dots in the TOP half (angles -72..0°), which users
   intuitively read as "scrolled to the top of the list". */
static bool s_force_visual_at_bottom = false;
void instruction_list_force_scroll_to_last(void)
{
    s_force_scroll_to_last = true;
    s_force_visual_at_bottom = true;
}

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

    /* Gate scroll-to-fade: scroll events fired inside this function are
       programmatic and must NOT dismiss the AI widget. */
    s_in_refresh_scroll = true;

    /* Founder direction 2026-05-19: every list update lands on the newest
       (last) item. Previously the s_force_scroll_to_last gate had to be
       opted in per-caller (instruction_list_force_scroll_to_last); now
       refresh itself flips it so single-item upserts (0x65), batch
       replace-all (0x6B), voice_say, and the mock cycler all converge
       on the same UX — newest at the focus / centre. The other two
       branches further down (saved_selected out of range, restore prior
       position) become dead but I leave them so callers that DO want
       to preserve position can clear the flag before refresh. */
    s_force_scroll_to_last = true;

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

    /* Invalidate the image cache for every instruction's current image path.
       update_instruction_image() running on a non-LVGL thread skips its own
       cache invalidation (deferring it here is safer than calling LVGL APIs
       on KE_EVT2's 4KB stack). When the phone replaces an existing image at
       the same path, we need this flush so lv_img_set_src below picks up the
       new pixels rather than a stale cached entry. */
    for (uint8_t i = 0; i < list_item_count; i++)
    {
        if (list_items[i].img_path[0] != '\0')
            lv_img_cache_invalidate_src(list_items[i].img_path);
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

    /* 新建的 dots 是 bg 的 child，appended 在尾端 → 預設 z-order 在 arc_zone
     * 上面，導致 dots 把 press 從 arc_zone 搶走。先把 arc_zone 拉回最上層，
     * 再把 ai_bg 拉到最上 — 最終順序：dots → arc_zone → ai_bg（top）*/
    if (p_instruction_list_layout->arc_handle != NULL)
    {
        arc_scroll_bring_to_front(p_instruction_list_layout->arc_handle);
    }
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
    if (s_force_scroll_to_last && list_item_count > 0)
    {
        /* Voice-say flow: scroll so the LAST (newest) item becomes the
           focused/centered one. LV_ANIM_OFF makes the scroll settle
           synchronously — important because scroll_list() below reads
           child coords, and with ANIM_ON the items would still be at
           their pre-scroll positions, yielding wrong dot placement. */
        s_force_scroll_to_last = false;
        uint16_t target = list_item_count - 1;
        app_scroll_target_item = target;
        selected_item_index = target;
        lv_obj_t *child = lv_obj_get_child(list, target);
        if (child && lv_obj_is_valid(child))
            lv_obj_scroll_to_view(child, LV_ANIM_OFF);
        lv_obj_update_layout(list);
        scroll_list(list, 0);
        /* scroll_list() reassigns selected_item_index to whichever item is
           closest to screen y-center; when the list scroll didn't land
           exactly on the target (LVGL scroll-to-view doesn't always
           perfectly center for short lists), the wrong item becomes
           selected. Re-assert so the label/highlight points to the
           newly-spoken item. */
        selected_item_index = target;
        app_scroll_target_item = target;
        LOG_I("[VOICE] re-asserted selected_item_index=%u (list_item_count=%u)",
              (unsigned)target, (unsigned)list_item_count);
    }
    else if (saved_selected >= list_item_count && list_item_count > 0)
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
     * selected_item_index. With selected=N-1 (last item after voice_say),
     * input_val resolves to 37 → the new item sits at angle 0° (right-mid,
     * y=233), naturally above the pill at y=296+. Older items appear at
     * negative angles (above center). This is the "selected at center,
     * older above" arc convention, which avoids hiding the new item
     * behind the AI widget pill. */
    s_force_visual_at_bottom = false; /* unused now; reset for safety */
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
    /* item 數變了 → 同步給共用 arc_scroll 模組做 scroll clamp */
    if (p_instruction_list_layout->arc_handle != NULL)
    {
        arc_scroll_set_item_count(p_instruction_list_layout->arc_handle,
                                  list_item_count);
    }
    LOG_D("refresh_custom_instructions: %d items total", list_item_count);
    s_in_refresh_scroll = false;
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

    /* Path-string update is thread-safe (single owner per index in practice). */
    strncpy(list_items[idx].img_path, img_path,
            sizeof(list_items[idx].img_path) - 1);
    list_items[idx].img_path[sizeof(list_items[idx].img_path) - 1] = '\0';

    /* LVGL ops only on the LVGL thread.
       BLE notify (parse_notify) and file-receive callback (bloc_filesystem)
       both run on KE_EVT2 (4KB stack). lv_img_set_src / lv_obj_clear_flag
       cascade through lv_event_send → potentially the list's scroll handler
       → scroll_list, blowing the stack. Defer the rebuild via the LVGL msg
       queue; refresh_custom_instructions invalidates caches and recreates
       the indicator dots from list_items[i].img_path. */
    if (!is_on_lvgl_thread())
    {
        lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_INSTRUCTION_LIST};
        lvgl_send_msg(msg);
        return;
    }

    lv_img_cache_invalidate_src(img_path);

    /* Update the indicator dot directly if UI exists */
    if (p_instruction_list_layout != NULL &&
        p_instruction_list_layout->indicator_dots[idx] != NULL &&
        lv_obj_is_valid(p_instruction_list_layout->indicator_dots[idx]))
    {
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

/* 右側弧形觸控滾動 — 改用共用模組 common/arc_scroll.h，跟 exercise、clock
 * 等其他位置共享同一份偵測算法。LIST_ITEM_SLOT_HEIGHT / LIST_ITEM_SLOT_ANGLE_DEG
 * 仍留著，給 cfg 傳進共用模組用。 */
#define LIST_ITEM_SLOT_HEIGHT (LIST_ITEM_WIDGET_HEIGHT + LIST_ITEM_SPACING)
#define LIST_ITEM_SLOT_ANGLE_DEG 36 /* 與 update_indicator_dots_position::angle_per_dot 一致；跟 exercise 對齊 */

static void inst_arc_reset_drag_state(void)
{
    s_inst_arc_drag_active = false;
    s_inst_drag_initialized = false;
    s_inst_drag_input = 0;
    s_inst_drag_last_idx = -1;
}

/* dot 回彈動畫 — release 時若 input 還在 elastic overshoot 區，把它從當前
 * 位置補間到 canonical（idx 對應的 input value），看得到 dot 平滑回彈 */
static int32_t s_inst_snap_anim_dummy;
static void inst_snap_anim_exec_cb(void *var, int32_t value)
{
    (void)var;
    if (s_inst_arc_drag_active) return;
    update_indicator_dots_position((int)value);
}

static void inst_start_snap_anim(int from, int to)
{
    lv_anim_del(&s_inst_snap_anim_dummy, inst_snap_anim_exec_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_inst_snap_anim_dummy);
    lv_anim_set_exec_cb(&a, inst_snap_anim_exec_cb);
    lv_anim_set_values(&a, from, to);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

/* drag_cb 模式：arc 拖動時不直接動 list，由這裡接管。
 * - 累積 input value，每幀 call update_indicator_dots_position 讓 dot 平滑轉
 * - 偵測「最靠近中央的 dot 換了一顆」（= page change）才 call scroll_list_to_index
 *   把 list snap 到對應 item，list 動畫期間 scroll_list 的 gate 會擋住反推 */
static void inst_arc_drag_cb(lv_coord_t scroll_delta_px, void *ctx)
{
    (void)ctx;
    int total = (int)list_item_count;
    if (total <= 0) return;

    s_inst_arc_drag_active = true;

    /* 第一次進來：用目前的 selected_item_index 反算 input。
     * input 跟 idx 對應公式（見 update_indicator_dots_position 的 offset_angle）：
     *   input = 100*(N - idx) - 63 */
    if (!s_inst_drag_initialized)
    {
        /* 新的拖動開始 — 取消上一輪 release 起的 snap anim */
        lv_anim_del(&s_inst_snap_anim_dummy, inst_snap_anim_exec_cb);
        int idx = (int)selected_item_index;
        if (idx < 0) idx = 0;
        if (idx >= total) idx = total - 1;
        s_inst_drag_input = 100 * (total - idx) - 63;
        s_inst_drag_last_idx = idx;
        s_inst_drag_initialized = true;
    }

    /* d_input = -d_scroll * 100 / pitch；instruction_list 的 pitch =
     * LIST_ITEM_SLOT_HEIGHT，dots_value 公式裡是 1:1（因為 SLOT_HEIGHT=100），
     * 寫成 generic 式更安全 */
    const int pitch = LIST_ITEM_SLOT_HEIGHT;
    int target_input = s_inst_drag_input - ((int)scroll_delta_px * 100) / pitch;

    int min_input = 100 - 63;             /* idx=N-1 */
    int max_input = 100 * total - 63;     /* idx=0 */
    /* elastic overshoot：跟 message_list 一致，邊界外 0.4 resistance、上限 50
     * input 單位（= 半 slot）。snap_cb 在釋放時把 dot 拉回 valid */
    const int MAX_OVERSHOOT = 50;
    if (target_input < min_input)
    {
        int over_now = (s_inst_drag_input < min_input) ? (min_input - s_inst_drag_input) : 0;
        int over_raw = min_input - target_input;
        if (over_raw > over_now)
        {
            int additional = (over_raw - over_now) * 4 / 10;
            int new_over = over_now + additional;
            if (new_over > MAX_OVERSHOOT) new_over = MAX_OVERSHOOT;
            target_input = min_input - new_over;
        }
    }
    else if (target_input > max_input)
    {
        int over_now = (s_inst_drag_input > max_input) ? (s_inst_drag_input - max_input) : 0;
        int over_raw = target_input - max_input;
        if (over_raw > over_now)
        {
            int additional = (over_raw - over_now) * 4 / 10;
            int new_over = over_now + additional;
            if (new_over > MAX_OVERSHOOT) new_over = MAX_OVERSHOOT;
            target_input = max_input + new_over;
        }
    }
    s_inst_drag_input = target_input;

    update_indicator_dots_position(s_inst_drag_input);

    int closest_idx = total - ((s_inst_drag_input + 63 + 50) / 100); /* round */
    if (closest_idx < 0) closest_idx = 0;
    if (closest_idx >= total) closest_idx = total - 1;

    if (closest_idx != s_inst_drag_last_idx)
    {
        s_inst_drag_last_idx = closest_idx;
        scroll_list_to_index((uint16_t)closest_idx);
    }
}

static lv_obj_t *list_arc_tap_cb(lv_point_t pt, void *ctx)
{
    (void)ctx;
    /* tap 收尾，順手 reset drag state */
    inst_arc_reset_drag_state();
    /* arc 模組 overlay 攔走 press → CLICK 不會 bubble 到 dot 或 touch_obj。
     * 仿 app_exercise.c 的 tap 路徑：
     *   1. 先用 press 點比對所有可見 indicator dot 的 bbox，找到哪顆 dot 就 forward
     *      CLICKED 給那顆，dot 上有註冊 list_item_click_event_cb（user_data = 對應
     *      list_items[i] ptr），所以點哪顆 dot 就觸發那 item 的動作。
     *   2. fallback：press 不在任何 dot 上但在選中項 touch_obj 範圍內 → forward 給
     *      touch_obj，行為跟舊版相同（保留中央區塊大面積可點）。 */
    if (p_instruction_list_layout != NULL)
    {
        for (int i = 0; i < list_item_count; i++)
        {
            lv_obj_t *dot_bg = p_instruction_list_layout->indicator_dots_bg[i];
            if (dot_bg == NULL) continue;
            if (!lv_obj_is_valid(dot_bg)) continue;
            if (lv_obj_has_flag(dot_bg, LV_OBJ_FLAG_HIDDEN)) continue;
            lv_area_t a;
            lv_obj_get_coords(dot_bg, &a);
            if (pt.x >= a.x1 && pt.x <= a.x2 && pt.y >= a.y1 && pt.y <= a.y2)
            {
                return dot_bg;
            }
        }
    }
    if (selected_item_index >= list_item_count) return NULL;
    if (touch_obj[selected_item_index] == NULL) return NULL;
    if (!lv_obj_is_valid(touch_obj[selected_item_index])) return NULL;
    if (lv_obj_has_flag(touch_obj[selected_item_index], LV_OBJ_FLAG_HIDDEN)) return NULL;
    lv_area_t a;
    lv_obj_get_coords(touch_obj[selected_item_index], &a);
    if (pt.x < a.x1 || pt.x > a.x2 || pt.y < a.y1 || pt.y > a.y2) return NULL;
    return touch_obj[selected_item_index];
}

static lv_obj_t *list_arc_snap_cb(void *ctx)
{
    (void)ctx;
    /* list 已經在 page change 當下被 snap 過了。如果 drag 結束在 elastic
     * overshoot 區（input 超出 valid 範圍），起一個 anim 把 dot 從當前位置
     * 平滑彈回 idx 對應的 canonical input，看得到回彈動畫 */
    if (s_inst_drag_last_idx >= 0)
    {
        int total = (int)list_item_count;
        if (total > 0)
        {
            int snap_input = 100 * (total - s_inst_drag_last_idx) - 63;
            if (s_inst_drag_input != snap_input)
            {
                inst_start_snap_anim(s_inst_drag_input, snap_input);
            }
        }
    }
    inst_arc_reset_drag_state();
    return NULL;
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

    /* 共用 arc_scroll 模組內部自帶 idempotent lock（lock 前先 unlock），
     * 不需要在這邊清 stale 狀態 */

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

    /* 右側弧形觸控滾動 — 用共用模組 common/arc_scroll.h。
     * 放在 ai_bar / ai_bg 之前 → AI 開啟時 ai_bg 蓋在上面，自然停用 */
    arc_scroll_config_t arc_cfg = {
        .parent          = p_instruction_list_bg,
        .list            = p_instruction_list,
        .slot_height_px  = LIST_ITEM_SLOT_HEIGHT,        /* 100 = 200 + (-100) */
        .item_height_px  = LIST_ITEM_WIDGET_HEIGHT,      /* 200，items 互相重疊 100 */
        .slot_angle_deg  = LIST_ITEM_SLOT_ANGLE_DEG,
        .item_count      = list_item_count,
        .band_thickness  = 90,
        .lock_ancestors  = true, /* instruction_list 是 tileview 子層，要鎖外層 */
        .tap_cb          = list_arc_tap_cb,
        .snap_cb         = list_arc_snap_cb,
        .drag_cb         = inst_arc_drag_cb,
        .ctx             = NULL,
    };
    p_instruction_list_layout->arc_handle = arc_scroll_create(&arc_cfg);
    /* DEBUG：顯示 arc band 觸發範圍。確認位置後可以拿掉這行 */
    // arc_scroll_set_debug_visible(p_instruction_list_layout->arc_handle, true);

    lv_obj_t *ai_bar = lv_obj_create(p_instruction_list_bg);
    lv_obj_set_size(ai_bar, 80, LV_VER_RES);
    lv_obj_align(ai_bar, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(ai_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(ai_bar, 0, 0);
    lv_obj_add_event_cb(ai_bar, ai_bar_event_cb, LV_EVENT_ALL, NULL);
    // lv_obj_add_flag(ai_bar, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(ai_bar, LV_OBJ_FLAG_PRESS_LOCK);

    /* The bottom-center "+" add-instruction button has been removed — the
       mic-bar voice trigger now owns the bottom-center affordance slot. The
       phone-side create-instruction flow remains available; can be re-added
       to a different location (e.g. settings) if user-facing entry is needed.

    [removed: add_inst_btn block — see git blame for prior 70x50 plus pill] */

    /* Bottom mic bar — permanent voice-input affordance. Click opens the AI
       widget the same way the release IMU gesture does. Explicitly hidden
       when AI widget is open (its trigger role is done; the AI widget's own
       voice button takes over). */
    lv_obj_t *mic_bar = lv_obj_create(p_instruction_list_bg);
    p_instruction_list_layout->mic_bar = mic_bar;
    lv_obj_set_size(mic_bar, 240, 50);
    lv_obj_align(mic_bar, LV_ALIGN_BOTTOM_MID, 0, -75);
    lv_obj_set_style_bg_color(mic_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(mic_bar, LV_OPA_50, 0);
    lv_obj_set_style_radius(mic_bar, 25, 0);
    lv_obj_set_style_border_width(mic_bar, 0, 0);
    lv_obj_set_style_pad_all(mic_bar, 0, 0);
    lv_obj_clear_flag(mic_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(mic_bar, mic_bar_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *mic_bar_icon = lv_img_create(mic_bar);
    lv_img_set_src(mic_bar_icon, &icon_mic);
    lv_obj_align(mic_bar_icon, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(mic_bar_icon, LV_OBJ_FLAG_CLICKABLE);

    /* AI widget docks BELOW the instruction list — only occupies the bottom
       strip. Sized to host the native message_widget_bg image (442x252)
       which becomes the pill's visual border/background. */
    #define AI_BG_STRIP_H 260
    p_instruction_list_layout->p_instruction_list_ai_bg =
        lv_tileview_create(p_instruction_list_bg);
    lv_obj_set_scrollbar_mode(
        p_instruction_list_layout->p_instruction_list_ai_bg,
        LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(p_instruction_list_layout->p_instruction_list_ai_bg,
                    LV_HOR_RES, AI_BG_STRIP_H);
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_0, 0);
    lv_obj_align(p_instruction_list_layout->p_instruction_list_ai_bg,
                 LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_t *home_page = lv_tileview_add_tile(
        p_instruction_list_layout->p_instruction_list_ai_bg, 1, 0, LV_DIR_HOR);
    lv_obj_set_size(home_page, LV_HOR_RES, AI_BG_STRIP_H);
    // lv_obj_set_style_bg_color(home_page, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(home_page, LV_OPA_0, 0);
    lv_obj_add_event_cb(home_page, home_tileview_event_cb, LV_EVENT_RELEASED,
                        NULL);
    lv_obj_t *ai_page = lv_tileview_add_tile(
        p_instruction_list_layout->p_instruction_list_ai_bg, 0, 0, LV_DIR_HOR);
    lv_obj_set_size(ai_page, LV_HOR_RES, AI_BG_STRIP_H);
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
    s_skai_widget = skai_widget;
    /* Pill structure: transparent container sized to the native
       message_widget_bg image (442x252). The image child supplies the
       visual border + rounded shape — matches the production Skaiwalk
       look on real hardware, where LVGL's border-style stroke is too
       thin to render. */
    lv_obj_set_size(skai_widget, 442, 252);
    lv_obj_align(skai_widget, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_opa(skai_widget, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(skai_widget, 0, 0);
    lv_obj_set_style_pad_all(skai_widget, 0, 0);
    /* Background image — gives the pill its visible border + shape. */
    s_pill_bg_img = lv_img_create(skai_widget);
    lv_obj_align(s_pill_bg_img, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_src(s_pill_bg_img, &message_widget_bg);
    lv_obj_clear_flag(s_pill_bg_img, LV_OBJ_FLAG_CLICKABLE);
    /* Transcript label — sits over the bg image, displays the spoken text. */
    s_voice_transcript_label = lv_label_create(skai_widget);
    lv_label_set_text(s_voice_transcript_label, "");
    lv_obj_set_width(s_voice_transcript_label, 360);
    lv_label_set_long_mode(s_voice_transcript_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(s_voice_transcript_label, lv_color_white(), 0);
    lv_obj_set_style_text_opa(s_voice_transcript_label, LV_OPA_80, 0);
    lv_obj_set_style_text_align(s_voice_transcript_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_voice_transcript_label, LV_ALIGN_TOP_MID, 0, 60);

    /* Voice indicator button — replaces ai_hint at bottom center.
       Lifted up by 25px so it sits visually inside the wrapping skai_widget
       pill instead of overhanging its bottom edge. */
    ai_voice_btn = lv_obj_create(ai_page);
    lv_obj_set_size(ai_voice_btn, 62, 62);
    lv_obj_set_style_radius(ai_voice_btn, 31, 0);
    lv_obj_set_style_bg_color(ai_voice_btn, lv_color_hex(0x00AAFF), 0);
    lv_obj_set_style_bg_opa(ai_voice_btn, LV_OPA_10, 0);
    lv_obj_align(ai_voice_btn, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_add_event_cb(ai_voice_btn, logo_click_event_cb, LV_EVENT_CLICKED,
                        NULL);
    lv_obj_t *ai_voice_img = lv_img_create(ai_voice_btn);
    s_voice_img = ai_voice_img;
    lv_img_set_src(ai_voice_img, &voice_group);
    lv_obj_align(ai_voice_img, LV_ALIGN_CENTER, 0, 0);
    ai_voice_send_icon = lv_img_create(ai_voice_btn);
    lv_img_set_src(ai_voice_send_icon, ICON_SAND);
    lv_obj_align(ai_voice_send_icon, LV_ALIGN_CENTER, 2, 2);
    lv_obj_add_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    // 創建可移動範圍圓弧線
    // create_movable_range_arc(p_instruction_list_bg);
    created = true;
    LOG_I("instruction_list_init: before myLancher reset_list");
    myLancher[app_index_instruction_list].reset_list = reset_list;
    if (myLancher[app_index_instruction_list].reset_list != NULL)
    {
        myLancher[app_index_instruction_list].reset_list();
    }
    LOG_I("instruction_list_init: before lv_event_send SCROLL");
    lv_event_send(p_instruction_list, LV_EVENT_SCROLL, NULL);
    LOG_I("instruction_list_init: after lv_event_send SCROLL");

    myLancher[app_index_instruction_list].on_tap = on_tap;
    LOG_I("instruction_list_init: returning");

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
    /* drag_cb 模式下 scroll_list 的 selected_item_index 自動更新被 gate 擋住，
     * 這邊先手動 set 起來，等下 scroll_list call 跑到 line 939 那段
     * (selected != old) 的可見性 loop 才會把新的 widget/label show 出來、舊的藏起來。
     * motion 路徑（NAV_BAR_CONTROL）也共用這個函式，多 set 一次同樣 idempotent */
    if (page < list_item_count)
    {
        selected_item_index = page;
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
#ifdef APP_ID_RECORDER
    case app_id_recorder:
        title = LV_EXT_STR_GET_BY_KEY(recorder, "Recorder");
        icon = IMG_RECORDER;
        id_str = APP_ID_RECORDER;
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
#ifdef APP_ID_MOUSE
    case app_id_mouse:
        title = LV_EXT_STR_GET_BY_KEY(mouse, "Mouse");
        icon = IMG_MOUSE;
        id_str = APP_ID_MOUSE;
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
    uint8_t custom_count = (list_item_count > app_base_count)
                               ? list_item_count - app_base_count
                               : 0;

    /* App slot count changed → shift custom items so they line up after the
       new app block. Walk the right direction to avoid clobbering on overlap. */
    if (custom_count > 0 && app_base_count != n)
    {
        if (n > app_base_count)
        {
            for (int i = custom_count - 1; i >= 0; i--)
                memcpy(&list_items[n + i],
                       &list_items[app_base_count + i], sizeof(list_item_t));
        }
        else
        {
            for (uint8_t i = 0; i < custom_count; i++)
                memcpy(&list_items[n + i],
                       &list_items[app_base_count + i], sizeof(list_item_t));
        }
    }

    for (uint8_t i = 0; i < n; i++)
    {
        map_app_id(INSTRUCTION_LIST_ITEMS_DEFINITION[i], &list_items[i]);
        LOG_D("App %d: ID=%s, Title=%s", i, list_items[i].id,
              list_items[i].title);
    }
    app_base_count = n;
    list_item_count = n + custom_count;
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
    /* Leaving the instruction_list page ends the SKAIBAR option-tracking
       session. Phone-side highlight stops following watch scroll until
       the user re-enters this page and taps the mic bar again. */
    s_skaibar_tracking_active = false;
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
