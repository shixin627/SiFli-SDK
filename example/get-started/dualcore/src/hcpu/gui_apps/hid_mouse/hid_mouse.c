/**
 ******************************************************************************
 * @file   hid_mouse.c
 * @author Skaiwalk software development team
 * @brief  HID Mouse application for Skaiwatch
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

/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include <string.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include <math.h>

#ifdef BSP_USING_BLOC
    #include "bloc_control.h"
    #include "custom_trans_anim.h"
    #include "bloc_setting.h"
    #include "bloc_peripheral.h"
#endif

#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif
#include "watch_system_core_task.h"

#include "communicate_protocol.h"
#include "ui_handler.h"
#include "ui_helper.h"
#include "ui_img_helper.h"
#include "lvsf_gesture.h"
#include "ble_device_manager.h"
#include "bloc_motion_tracking.h"
#include "ble_hid.h"
#include "bf0_hal.h"
#include "bf0_sys_cfg.h"

#ifdef APP_ID_MOUSE

    #define DBG_TAG "hid.mouse"
    #define DBG_LVL DBG_LOG
    #include <rtdbg.h>

    /*********************
     *      DEFINES
     *********************/
    #define PRESSED_TIME_MS 500
    #define SCROLLING_THRESHOLD 20
    #define EDGE_THRESHOLD_PIXELS 20
    #define BOTTOM_EDGE_THRESHOLD 50
    #define GESTURE_DISTANCE_THRESHOLD 50
    #define GESTURE_TIMER_MS 200
    #define INERTIA_TIMER_MS 20
    #define INERTIA_DECAY_FACTOR 0.95f
    #define MIN_SCROLL_SPEED 1.0f
    #define SCROLL_SPEED_MULTIPLIER 2.0f
    #define DOUBLE_TAP_MS 300

    #define SIMULATE_MOUSE_RIGHT_BUTTON 0
    #define USING_MOUSE_WHEEL_SCROLLING 1
    #define USING_TOUCHSCREEN_SCROLLING 0
    #define USING_EDGE_BOTTOM_DETECTION 0
    #define USING_EDGE_LEFT_DETECTION 0
    #define USING_EDGE_RIGHT_DETECTION 0

    #define ENABLE_MENU_FEATURE 1
    #define KB_ANIM_TIME_MS 300

    // FSR-402 pressure sensor ADC config
    #define FSR_ADC_DEV_NAME "bat1"
    #define FSR_ADC_CHANNEL 3
    #define FSR_ADC_READ_MS 100

    #define FRC_THRESHOLD_BTN 100
    #define FRC_THRESHOLD_MOVE_LOCK 10
    // #define USE_FSR_ADC 1

/*********************
 *      TYPEDEFS
 *********************/
typedef enum
{
    EDGE_NONE = 0,
    EDGE_BOTTOM = 1,
    EDGE_LEFT = 2,
    EDGE_RIGHT = 3
} edge_position_t;

/*********************
 *  STATIC VARIABLES
 *********************/
LV_IMG_DECLARE(img_mouse);
LV_IMG_DECLARE(plus_button);
LV_IMG_DECLARE(enter_icon);
LV_IMG_DECLARE(capital_icon);
LV_IMG_DECLARE(backspace_icon);
LV_IMG_DECLARE(icon_mic);
LV_IMG_DECLARE(space);
LV_IMG_DECLARE(img_skai); // 160 * 160
LV_IMG_DECLARE(erth);

static lv_point_t touchscreen_point;
static lv_point_t start_point;
static lv_point_t last_point;
static lv_point_t bottom_bar_start_point;
static lv_point_t bottom_bar_last_point;
static unsigned int press_time = 0;
static bool user_touching = false;
static bool pressing = false;
static bool scrolling = false;
static bool scrolling_confirmed = false;
static bool moving = false;
static bool has_moved_during_touch = false;

static bool go_back = false;
static bool handfree = false;
static bool is_gesture_active = false;
static bool is_bottom_bar_gesture_active = false;
static bool gesture_timer_enabled = false;
static bool bottom_bar_gesture_timer_enabled = false;
static bool gesture_detected = false;

// 滚动方向锁定：一旦确定滚动方向就保持该方向
static bool scroll_direction_locked = false;
static bool is_horizontal_scroll = false;

// 慣性滾動
static float inertia_velocity = 0.0f;
static float inertia_accumulator = 0.0f;
static lv_timer_t *inertia_timer = NULL;
static uint32_t last_scroll_tick = 0;
static uint32_t last_click_time = 0;

// FSR-402 ADC pressure sensor
static rt_device_t fsr_adc_dev = NULL;
static rt_timer_t fsr_adc_timer = NULL;
// static lv_obj_t *fsr_adc_label = NULL;
static rt_uint32_t fsr_adc_value = 0;

    #if SIMULATE_MOUSE_RIGHT_BUTTON
static bool pressed_left_half = false;
    #endif

static bool start_from_edge[4] = {false};
static lv_timer_t *scroll_timer = NULL;
static lv_indev_t *indev_global = NULL;
static lv_obj_t *device_sw = NULL;
static uint8_t zoom_count = 0;
static lv_obj_t *menu_bg = NULL;
static lv_obj_t *menu_tileview = NULL;
static lv_obj_t *menu_home_tile = NULL;
static lv_obj_t *menu_content_tile = NULL;
static lv_obj_t *menu_swipe_area = NULL;
static lv_obj_t *left_scroll_bar = NULL;
static lv_point_t scroll_bar_last_point;
static bool left_scroll_active = false;

// 判斷觸碰點是否在左側滾動觸發區域
// UI 弧線寬度 30px、角度 150°~210°，但觸發範圍更大
static bool is_point_in_left_arc(const lv_point_t *p)
{
    float cx = LV_HOR_RES_MAX / 2.0f;
    float cy = LV_VER_RES_MAX / 2.0f;
    float dx = p->x - cx;
    float dy = p->y - cy;
    float dist = sqrtf(dx * dx + dy * dy);
    float outer_r = cx;
    float inner_r = outer_r - 100.0f; // 觸發範圍比 UI（30px）更寬
    if (dist < inner_r || dist > outer_r)
        return false;
    // 只接受左側（x < 中心）
    if (dx >= 0)
        return false;
    // 角度放寬到約 140°~220°（sin(40°) ≈ 0.643）
    float max_dy = dist * 0.643f;
    return (dy >= -max_dy && dy <= max_dy);
}
static lv_obj_t *control_page = NULL;
static lv_obj_t *status_bar_area = NULL;
static lv_obj_t *connected_device_label = NULL;
static lv_obj_t *crosshair_line1 = NULL;
static lv_obj_t *crosshair_line2 = NULL;
static lv_obj_t *status_bar_time_h = NULL;
static lv_obj_t *status_bar_time_m = NULL;
static lv_obj_t *status_bar_time_symbol = NULL;
static lv_obj_t *text_input_bar = NULL;
static lv_obj_t *text_input_bar_bg = NULL;
static uint8_t *text_input_open_value = NULL;
static lv_timer_t *text_input_bar_timer = NULL;
// static lv_timer_t *colon_blink_timer = NULL;
static bool colon_visible = true;

// Keyboard related variables
static lv_obj_t *keyboard = NULL;
static lv_obj_t *text_area = NULL;
static bool keyboard_visible = false;
static lv_obj_t *custom_keyboard = NULL;
static lv_obj_t *keyboard_container = NULL;

static uint8_t keyboard_text_size = 1;

// Keyboard mode control
typedef enum
{
    KEYBOARD_MODE_LETTERS = 0,
    KEYBOARD_MODE_NUMBERS
} keyboard_mode_t;

static keyboard_mode_t current_keyboard_mode = KEYBOARD_MODE_LETTERS;

// Case control for letters
static bool is_uppercase = false;
static lv_obj_t *caps_btn = NULL;
static lv_obj_t *mode_btn = NULL;
static lv_obj_t *del_img = NULL;

// Key popup variables
static lv_obj_t *key_popup = NULL;
static lv_obj_t *key_popup_label = NULL;

// Proximity detection variables
static bool proximity_mode_active = false;
static lv_point_t press_start_point = {0, 0};
static lv_obj_t *closest_btn = NULL;
static lv_obj_t *currently_pressed_btn = NULL;

// Input display variables (now children of text_input_bar_bg)
static lv_obj_t *input_content_container = NULL;
static lv_obj_t *input_display_label = NULL;
static lv_obj_t *input_cursor = NULL;
static lv_obj_t *input_enter_btn = NULL;
static lv_timer_t *cursor_blink_timer = NULL;
static char input_buffer[128] = {0};
static int input_length = 0;
static bool cursor_visible = true;
    #define MAX_DISPLAY_WIDTH 280 // 輸入框最大顯示寬度（縮短後）

// File list variables
static lv_obj_t *file_list = NULL;
typedef struct
{
    char name[32];
    void (*callback)(void);
} file_item_t;

static file_item_t file_items[10]; // 最多10個file
static int file_items_count = 0;

// Long press detection variables
static rt_timer_t long_press_timer = NULL;
static bool is_long_press_triggered = false;
static const char *long_press_key_text = NULL;
static const char *closest_key_text = NULL;
static lv_obj_t *all_keys[50]; // 存儲所有按鍵的數組
static int all_keys_count = 0;

    #if USING_EDGE_BOTTOM_DETECTION
static rt_timer_t multiple_pages_timer;
    #endif

bool is_skai_touch_enabled(void)
{
    return user_touching;
}

static unsigned int fsr_change_time = 0;
bool is_fsr_change_detected(void)
{
    if ((rt_tick_get() - fsr_change_time) < 200)
    {
        return true;
    }
    return false; // FSR變化持續500ms內視為有效
}

void set_air_mouse_moving_state(bool state)
{
    if (state != moving)
    {
        moving = state;
    }
}

/*********************
 *  STATIC FUNCTIONS
 *********************/

// Key popup functions
static void show_key_popup(lv_obj_t *btn, const char *key_text);
static void hide_key_popup(void);

// Long press detection functions
static void long_press_timer_callback(void *parameter);
static void start_long_press_timer(const char *key_text);
static void stop_long_press_timer(void);

// Proximity detection functions
static void register_key_button(lv_obj_t *btn);
static lv_obj_t *find_closest_key(lv_point_t touch_point);
static void handle_proximity_input(lv_event_t *e);
static void input_enter_btn_event_cb(lv_event_t *e);

// File list functions
static void file_item_clicked_cb(lv_event_t *e);
static void add_file_to_list(const char *file_name, void (*callback)(void));
static void dummy_file1_callback(void);
static void dummy_file2_callback(void);
static void dummy_file3_callback(void);

    #if USING_EDGE_BOTTOM_DETECTION
static void start_multiple_pages_timer(void);
    #endif

// static void update_crosshair_brightness(void)
// {
//     if (crosshair_line1 != NULL && crosshair_line2 != NULL)
//     {
//         if (user_touching)
//         {
//             // Bright color when touching
//             lv_obj_set_style_bg_color(crosshair_line1,
//             lv_color_hex(0xCCCCCC),
//                                       0);
//             lv_obj_set_style_bg_color(crosshair_line2,
//             lv_color_hex(0xCCCCCC),
//                                       0);
//         }
//         else
//         {
//             // Dim color when not touching
//             lv_obj_set_style_bg_color(crosshair_line1,
//             lv_color_hex(0x666666),
//                                       0);
//             lv_obj_set_style_bg_color(crosshair_line2,
//             lv_color_hex(0x666666),
//                                       0);
//         }
//     }
// }

/**
 * @brief Update cursor position to follow the text
 */
static void update_cursor_position(void)
{
    if (input_cursor == NULL || input_display_label == NULL)
        return;

    // 獲取文字標籤的實際位置和寬度
    lv_coord_t label_x = lv_obj_get_x(input_display_label);
    lv_coord_t text_width = lv_obj_get_width(input_display_label);

    // 將游標放在文字右側
    lv_obj_align_to(input_cursor, input_display_label, LV_ALIGN_OUT_RIGHT_MID,
                    2, 0);
}

/**
 * @brief Cursor blink timer callback
 */
static void cursor_blink_cb(lv_timer_t *timer)
{
    if (input_cursor == NULL)
        return;

    cursor_visible = !cursor_visible;

    if (cursor_visible)
    {
        lv_obj_clear_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief Start cursor blinking
 */
static void start_cursor_blink(void)
{
    if (cursor_blink_timer == NULL)
    {
        cursor_blink_timer = lv_timer_create(cursor_blink_cb, 500, NULL);
    }
    else
    {
        lv_timer_resume(cursor_blink_timer);
    }
    cursor_visible = true;
    if (input_cursor != NULL)
    {
        lv_obj_clear_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief Stop cursor blinking
 */
static void stop_cursor_blink(void)
{
    if (cursor_blink_timer != NULL)
    {
        lv_timer_pause(cursor_blink_timer);
    }
    if (input_cursor != NULL)
    {
        lv_obj_add_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief Clear input display buffer and update display
 */
static void clear_input_display(void)
{
    memset(input_buffer, 0, sizeof(input_buffer));
    input_length = 0;
    if (input_display_label != NULL)
    {
        lv_label_set_text(input_display_label, "");
        // 重新對齊 label 到左側
        lv_obj_align(input_display_label, LV_ALIGN_LEFT_MID, 10, 0);
    }
    // 更新游標位置
    update_cursor_position();
}

/**
 * @brief Update input display with current buffer, scrolling left if needed
 */
static void update_input_display(void)
{
    if (input_display_label == NULL)
        return;

    lv_label_set_text(input_display_label, input_buffer);

    // 檢查文字寬度是否超過顯示區域
    lv_coord_t text_width = lv_obj_get_width(input_display_label);
    if (text_width > MAX_DISPLAY_WIDTH)
    {
        // 向左對齊，讓最新輸入的文字可見
        lv_obj_align(input_display_label, LV_ALIGN_RIGHT_MID, -10, 0);
    }
    else
    {
        // 文字未超過寬度，居中顯示
        lv_obj_align(input_display_label, LV_ALIGN_LEFT_MID, 10, 0);
    }

    // 更新游標位置
    update_cursor_position();
}

/**
 * @brief Add character to input buffer
 */
static void add_to_input_buffer(const char *text)
{
    if (text == NULL || input_length >= sizeof(input_buffer) - 1)
        return;

    size_t text_len = strlen(text);
    if (input_length + text_len < sizeof(input_buffer) - 1)
    {
        strcat(input_buffer, text);
        input_length += text_len;
        update_input_display();
    }
}

/**
 * @brief Remove last character from input buffer (backspace)
 */
static void remove_from_input_buffer(void)
{
    if (input_length > 0)
    {
        input_buffer[input_length - 1] = '\0';
        input_length--;
        update_input_display();
    }
}

/**
 * @brief Helper: set translate_y style (used as lv_anim exec callback)
 */
extern void set_stop_mouse_move(bool stop);
static void anim_set_translate_y(void *obj, int32_t v)
{
    lv_obj_set_style_translate_y((lv_obj_t *)obj, v, 0);
}

/**
 * @brief Animation ready callback when keyboard close animation finishes
 */
static void keyboard_close_anim_ready_cb(lv_anim_t *a)
{
    // Restore bar to closed style
    lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_border_width(text_input_bar_bg, 0, LV_PART_MAIN);

    // Show indicator, hide input content
    lv_obj_clear_flag(text_input_bar, LV_OBJ_FLAG_HIDDEN);
    if (input_content_container != NULL)
        lv_obj_add_flag(input_content_container, LV_OBJ_FLAG_HIDDEN);
    if (input_cursor != NULL)
        lv_obj_add_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);

    // Hide keyboard and reset its translate_y
    if (keyboard_container != NULL)
    {
        lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_y(keyboard_container, 0, 0);
    }
}

/**
 * @brief Toggle keyboard visibility with animation
 */
void toggle_keyboard_visibility(void)
{
    if (keyboard_container == NULL)
        return;

    lv_anim_t a;

    // Cancel any ongoing animations on bar and keyboard
    lv_anim_del(text_input_bar_bg, NULL);
    lv_anim_del(keyboard_container, NULL);

    if (keyboard_visible)
    {
        // === CLOSE KEYBOARD ===
        keyboard_visible = false;
        set_stop_mouse_move(false);
        hide_key_popup();
        clear_input_display();
        stop_cursor_blink();

        // Hide enter button immediately
        if (input_enter_btn != NULL)
            lv_obj_add_flag(input_enter_btn, LV_OBJ_FLAG_HIDDEN);

        // Animate bar x: open -> closed
        lv_anim_init(&a);
        lv_anim_set_var(&a, text_input_bar_bg);
        lv_anim_set_values(&a, lv_obj_get_x(text_input_bar_bg),
                           (LV_HOR_RES_MAX - 200) / 2);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);

        // Animate bar y: open -> closed
        lv_anim_init(&a);
        lv_anim_set_var(&a, text_input_bar_bg);
        lv_anim_set_values(&a, lv_obj_get_y(text_input_bar_bg),
                           LV_VER_RES_MAX - 50);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);

        // Animate bar width: open -> closed
        lv_anim_init(&a);
        lv_anim_set_var(&a, text_input_bar_bg);
        lv_anim_set_values(&a, lv_obj_get_width(text_input_bar_bg), 200);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_width);
        lv_anim_start(&a);

        // Animate bar height: open -> closed (with ready callback)
        lv_anim_init(&a);
        lv_anim_set_var(&a, text_input_bar_bg);
        lv_anim_set_values(&a, lv_obj_get_height(text_input_bar_bg), 50);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_height);
        lv_anim_set_ready_cb(&a, keyboard_close_anim_ready_cb);
        lv_anim_start(&a);

        // Animate keyboard sliding down using translate_y
        // (preserves the original BOTTOM_MID alignment)
        lv_anim_init(&a);
        lv_anim_set_var(&a, keyboard_container);
        lv_anim_set_values(&a, 0, 300);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_set_translate_y);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);
    }
    else
    {
        // === OPEN KEYBOARD ===
        set_stop_mouse_move(true);
        keyboard_visible = true;

        // Set open style on bar
        lv_obj_set_style_bg_color(text_input_bar_bg, lv_color_hex(0x1a1a1a),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_90, LV_PART_MAIN);
        lv_obj_set_style_border_color(text_input_bar_bg, lv_color_hex(0xFFFFFF),
                                      LV_PART_MAIN);
        lv_obj_set_style_border_width(text_input_bar_bg, 2, LV_PART_MAIN);
        lv_obj_set_style_border_opa(text_input_bar_bg, LV_OPA_50, LV_PART_MAIN);

        // Hide indicator, show input content
        lv_obj_add_flag(text_input_bar, LV_OBJ_FLAG_HIDDEN);
        if (input_content_container != NULL)
            lv_obj_clear_flag(input_content_container, LV_OBJ_FLAG_HIDDEN);

        // Show enter button
        if (input_enter_btn != NULL)
            lv_obj_clear_flag(input_enter_btn, LV_OBJ_FLAG_HIDDEN);

        // Show keyboard off-screen (translate_y pushes it below visible area)
        lv_obj_clear_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_y(keyboard_container, 300, 0);

        // Target positions for open state
        int32_t open_x = (LV_HOR_RES_MAX - 310) / 2 - 35;
        int32_t open_y = LV_VER_RES_MAX - 305 - 45;

        // Animate bar x: closed -> open
        lv_anim_init(&a);
        lv_anim_set_var(&a, text_input_bar_bg);
        lv_anim_set_values(&a, lv_obj_get_x(text_input_bar_bg), open_x);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);

        // Animate bar y: closed -> open
        lv_anim_init(&a);
        lv_anim_set_var(&a, text_input_bar_bg);
        lv_anim_set_values(&a, lv_obj_get_y(text_input_bar_bg), open_y);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);

        // Animate bar width: closed -> open
        lv_anim_init(&a);
        lv_anim_set_var(&a, text_input_bar_bg);
        lv_anim_set_values(&a, lv_obj_get_width(text_input_bar_bg), 310);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_width);
        lv_anim_start(&a);

        // Animate bar height: closed -> open
        lv_anim_init(&a);
        lv_anim_set_var(&a, text_input_bar_bg);
        lv_anim_set_values(&a, lv_obj_get_height(text_input_bar_bg), 45);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_height);
        lv_anim_start(&a);

        // Animate keyboard sliding up using translate_y (300 -> 0)
        // Keyboard is aligned at BOTTOM_MID, translate_y offsets from there
        lv_anim_init(&a);
        lv_anim_set_var(&a, keyboard_container);
        lv_anim_set_values(&a, 300, 0);
        lv_anim_set_time(&a, KB_ANIM_TIME_MS);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_set_translate_y);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);

        start_cursor_blink();
    }
}

/**
 * @brief Text area event callback for keyboard interaction
 * @param e Pointer to the event
 */
static void ta_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = lv_event_get_target(e);
    lv_obj_t *kb = (lv_obj_t *)lv_event_get_user_data(e);

    if (code == LV_EVENT_FOCUSED)
    {
        if (kb != NULL)
        {
            lv_keyboard_set_textarea(kb, ta);
            lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
            keyboard_visible = true;
        }
    }

    if (code == LV_EVENT_DEFOCUSED)
    {
        if (kb != NULL)
        {
            lv_keyboard_set_textarea(kb, NULL);
            lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
            keyboard_visible = false;
        }
    }
}

/**
 * @brief Switch keyboard mode (you can call this to change keyboard layout)
 * @param mode Keyboard mode to switch to
 */
static void switch_keyboard_mode(lv_keyboard_mode_t mode)
{
    if (keyboard != NULL)
    {
        lv_keyboard_set_mode(keyboard, mode);
    }
}

/**
 * @brief Create custom keyboard map (example of custom layout)
 */
static void setup_custom_keyboard_map(void)
{
    // Example: Create a simple number pad
    static const char *number_map[] = {"1",
                                       "2",
                                       "3",
                                       "\n",
                                       "4",
                                       "5",
                                       "6",
                                       "\n",
                                       "7",
                                       "8",
                                       "9",
                                       "\n",
                                       LV_SYMBOL_BACKSPACE,
                                       "0",
                                       LV_SYMBOL_OK,
                                       ""};

    static const lv_btnmatrix_ctrl_t number_ctrl[] = {1, 1, 1, 1, 1, 1,
                                                      1, 1, 1, 2, 1, 2};

    if (keyboard != NULL)
    {
        // You can set custom map like this:
        // lv_keyboard_set_map(keyboard, LV_KEYBOARD_MODE_USER_1, number_map,
        // number_ctrl); lv_keyboard_set_mode(keyboard,
        // LV_KEYBOARD_MODE_USER_1);
    }
}

/**
 * @brief Custom keyboard button event callback
 */
static void custom_keyboard_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED)
    {
        uint16_t id = lv_btnmatrix_get_selected_btn(obj);
        const char *txt = lv_btnmatrix_get_btn_text(obj, id);

        if (txt != NULL) // text_area != NULL &&
        {
            if (strcmp(txt, "Del") == 0)
            {
                // lv_textarea_del_char(text_area);
            }
            else if (strcmp(txt, "Enter") == 0)
            {
                // lv_textarea_add_char(text_area, '\n');
            }
            else if (strcmp(txt, "Space") == 0)
            {
                // lv_textarea_add_char(text_area, ' ');
            }
            else if (strcmp(txt, "Hide") == 0)
            {
                // Hide keyboard
                toggle_keyboard_visibility();
            }
            else
            {
                // lv_textarea_add_text(text_area, txt);
            }
        }
    }
}

/**
 * @brief Show key popup above the pressed button
 */
static void show_key_popup(lv_obj_t *btn, const char *key_text)
{
    if (key_popup != NULL)
    {
        lv_obj_del(key_popup);
        key_popup = NULL;
        key_popup_label = NULL;
    }

    // 創建彈出框容器
    key_popup = lv_obj_create(lv_obj_get_parent(keyboard_container));
    lv_obj_set_size(key_popup, 90, 90);

    // 計算按鍵位置並將彈出框放在按鍵上方
    lv_coord_t btn_x = lv_obj_get_x(btn) + lv_obj_get_x(keyboard_container);
    lv_coord_t btn_y = lv_obj_get_y(btn) + lv_obj_get_y(keyboard_container);
    lv_coord_t popup_x = btn_x + (lv_obj_get_width(btn) - 90) / 2 + 10;
    lv_coord_t popup_y = btn_y - 105; // 在按鍵上方

    // 螢幕尺寸 466x466
    const lv_coord_t screen_width = 466;
    const lv_coord_t screen_height = 466;
    const lv_coord_t popup_width = 75;
    const lv_coord_t popup_height = 80;
    const lv_coord_t screen_center_x = screen_width / 2;
    const lv_coord_t screen_center_y = screen_height / 2;
    const lv_coord_t screen_radius = screen_width / 2;

    // 計算彈出框中心點
    lv_coord_t popup_center_x = popup_x + popup_width / 2;
    lv_coord_t popup_center_y = popup_y + popup_height / 2;

    // 計算與螢幕中心的距離
    double distance = sqrt((popup_center_x - screen_center_x) *
                               (popup_center_x - screen_center_x) +
                           (popup_center_y - screen_center_y) *
                               (popup_center_y - screen_center_y));

    // 如果彈出框中心超出圓形邊界，將其拉回圓形範圍內
    if (distance + popup_width / 2 > screen_radius - 10) // 留10像素邊距
    {
        double angle = atan2(popup_center_y - screen_center_y,
                             popup_center_x - screen_center_x);
        double new_distance = screen_radius - popup_width / 2 - 10;
        popup_center_x = screen_center_x + new_distance * cos(angle);
        popup_center_y = screen_center_y + new_distance * sin(angle);
        popup_x = popup_center_x - popup_width / 2;
        popup_y = popup_center_y - popup_height / 2;
    }

    // 基本的矩形邊界檢查作為備份
    if (popup_x < 10)
    {
        popup_x = 10;
    }
    else if (popup_x + popup_width > screen_width - 10)
    {
        popup_x = screen_width - popup_width - 10;
    }

    if (popup_y < 10)
    {
        popup_y = 10;
    }
    else if (popup_y + popup_height > screen_height - 10)
    {
        popup_y = screen_height - popup_height - 10;
    }

    lv_obj_set_pos(key_popup, popup_x, popup_y);

    // 設置彈出框樣式
    lv_obj_set_style_bg_color(key_popup, lv_color_hex(0xA5A5A5), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(key_popup, LV_OPA_100, LV_PART_MAIN);
    lv_obj_set_style_radius(key_popup, 18, LV_PART_MAIN);
    lv_obj_set_style_border_width(key_popup, 0, LV_PART_MAIN);

    // 創建文字標籤
    key_popup_label = lv_label_create(key_popup);
    lv_label_set_text(key_popup_label, key_text);
    lv_obj_set_style_text_color(key_popup_label, lv_color_hex(0x000000),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(
        key_popup_label,
        LV_EXT_FONT_GET(get_system_font_size(keyboard_text_size)),
        LV_PART_MAIN); // 更大字體
    lv_obj_center(key_popup_label);

    // 確保彈出框在最頂層
    lv_obj_move_foreground(key_popup);
}

/**
 * @brief Hide key popup
 */
static void hide_key_popup(void)
{
    if (key_popup != NULL)
    {
        lv_obj_del(key_popup);
        key_popup = NULL;
        key_popup_label = NULL;
    }
}

/**
 * @brief Long press timer callback
 */
static void long_press_timer_callback(void *parameter)
{
    const char *key_text = (const char *)parameter;

    if (key_text != NULL && strcmp(key_text, "Space") == 0)
    {
        is_long_press_triggered = true;
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_MOUSE_OPEN_V2T;
        lvgl_send_msg(msg);
    }
}

void open_v2t_mic(void)
{
    bool mic_listen_status = true;
    extern void set_voice_recognition_notified_from_mouse(bool status);
    set_voice_recognition_notified_from_mouse(true);
    watch_system_interact(INTERACT_MIC_V2T_INPUT, &mic_listen_status);
    // 在這裡添加你想要的長按空白鍵功能
    // 例如：啟動語音輸入、切換輸入法、打開設置等

    // 隱藏彈出框（如果有的話）
    hide_key_popup();
}

/**
 * @brief Start long press timer
 */
static void start_long_press_timer(const char *key_text)
{
    // 停止之前的定時器
    stop_long_press_timer();

    // 只對 Space 按鍵啟動長按檢測
    if (key_text != NULL && strcmp(key_text, "Space") == 0)
    {
        is_long_press_triggered = false;
        long_press_key_text = key_text;

        // 創建定時器，800ms 後觸發長按事件
        long_press_timer = rt_timer_create(
            "long_press", long_press_timer_callback,
            (void *)long_press_key_text, rt_tick_from_millisecond(800),
            RT_TIMER_FLAG_ONE_SHOT);
        if (long_press_timer != NULL)
        {
            rt_timer_start(long_press_timer);
        }
    }
}

/**
 * @brief Stop long press timer
 */
static void stop_long_press_timer(void)
{
    if (long_press_timer != NULL)
    {
        rt_timer_stop(long_press_timer);
        rt_timer_delete(long_press_timer);
        long_press_timer = NULL;
    }
    long_press_key_text = NULL;
}

/**
 * @brief Get button text, handling special cases like Enter button and Caps
 * Lock button
 */
static const char *get_button_text(lv_obj_t *btn)
{
    // 檢查是否是特殊按鍵（有圖像而不是文字）
    lv_obj_t *child = lv_obj_get_child(btn, 0);
    if (child != NULL)
    {
        // 如果是圖像，檢查是哪個圖像
        if (lv_obj_check_type(child, &lv_img_class))
        {
            const void *src = lv_img_get_src(child);

            // 檢查是否是 Enter 按鍵
            if (src == &enter_icon)
            {
                return "Enter";
            }
            // 檢查是否是 Caps Lock 按鍵
            else if (src == &capital_icon)
            {
                return "Caps";
            }
            // 檢查是否是 Del/Backspace 按鍵
            else if (src == &backspace_icon)
            {
                return "Del";
            }
            // 檢查是否是 Close 按鍵
            else if (strcmp(src, DOWN_ARROW) == 0)
            {
                return "Close";
            }
            // 檢查是否是 Mode 切換按鍵
            else if (src == &erth)
            {
                return "Mode";
            }
            // 檢查是否是 Space 按鍵
            else if (src == &space)
            {
                return "Space";
            }
            // 其他未知圖像
            else
            {
                return "";
            }
        }
        // 如果是標籤，返回文字
        else if (lv_obj_check_type(child, &lv_label_class))
        {
            return lv_label_get_text(child);
        }
    }
    return "";
}

/**
 * @brief Event callback for input_enter_btn
 */
static void input_enter_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED)
    {
        LOG_D("Enter button clicked");
        control_provider.ble_hid_keyboard_input("\n");
        clear_input_display();
    }
}

/**
 * @brief Register a key button for proximity detection
 */
static void register_key_button(lv_obj_t *btn)
{
    if (all_keys_count < 50)
    {
        all_keys[all_keys_count] = btn;
        all_keys_count++;
    }
}

/**
 * @brief Find the closest key to the given touch point
 */
static lv_obj_t *find_closest_key(lv_point_t touch_point)
{
    if (all_keys_count == 0)
        return NULL;

    lv_obj_t *closest = NULL;
    float min_distance = 999999.0f;

    for (int i = 0; i < all_keys_count; i++)
    {
        lv_obj_t *btn = all_keys[i];
        if (btn == NULL)
            continue;

        // 計算按鍵的絕對邊界
        // input_enter_btn 是在 parent 上創建的，不需要加上 keyboard_container
        // 的偏移
        lv_coord_t btn_x, btn_y;
        if (btn == input_enter_btn)
        {
            btn_x = lv_obj_get_x(btn);
            btn_y = lv_obj_get_y(btn);
        }
        else
        {
            btn_x = lv_obj_get_x(btn) + lv_obj_get_x(keyboard_container);
            btn_y = lv_obj_get_y(btn) + lv_obj_get_y(keyboard_container);
        }
        lv_coord_t btn_width = lv_obj_get_width(btn);
        lv_coord_t btn_height = lv_obj_get_height(btn);

        // 首先檢查觸摸點是否在按鍵邊界內
        bool is_in_bounds = false;

        // 檢查是否是 Close 按鍵，如果是則使用特殊的觸發區域
        const char *btn_text = get_button_text(btn);
        if (strcmp(btn_text, "Close") == 0)
        {
            // Close 按鍵的觸發區域向下延伸15像素
            is_in_bounds = (touch_point.x >= btn_x &&
                            touch_point.x <= (btn_x + btn_width) &&
                            touch_point.y >= (btn_y + 15) &&
                            touch_point.y <= (btn_y + btn_height + 15));
        }
        else
        {
            // 其他按鍵使用正常的邊界檢查
            is_in_bounds = (touch_point.x >= btn_x &&
                            touch_point.x <= (btn_x + btn_width) &&
                            touch_point.y >= btn_y &&
                            touch_point.y <= (btn_y + btn_height));
        }

        if (is_in_bounds)
        {
            // 如果在邊界內，直接返回這個按鍵
            return btn;
        }

        // 如果不在邊界內，計算到按鍵中心的距離
        lv_coord_t btn_center_x = btn_x + btn_width / 2;
        lv_coord_t btn_center_y = btn_y + btn_height / 2;

        float dx = touch_point.x - btn_center_x;
        float dy = touch_point.y - btn_center_y;
        float distance = sqrtf(dx * dx + dy * dy);

        if (distance < min_distance)
        {
            min_distance = distance;
            closest = btn;
        }
    }

    return closest;
}

/**
 * @brief Handle proximity-based input
 */
static void create_circular_keyboard_layout(lv_obj_t *parent);
static void handle_proximity_input(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *container = lv_event_get_target(e);

    if (code == LV_EVENT_PRESSED)
    {
        // 記錄按下位置
        lv_indev_t *indev = lv_indev_get_act();
        if (indev != NULL)
        {
            lv_indev_get_point(indev, &press_start_point);
            proximity_mode_active = true;

            // 找到最接近的按鍵並顯示彈出框
            closest_btn = find_closest_key(press_start_point);
            if (closest_btn != NULL)
            {
                closest_key_text = get_button_text(closest_btn);

                // 對於Del按鈕，切換圖片而不是改變背景色
                if (strcmp(closest_key_text, "Del") == 0 && del_img != NULL)
                {
                    // lv_img_set_src(del_img, &backspace_pressed_icon);
                }
                else
                {
                    // 設置按鈕為按下狀態，顯示淺藍色背景
                    lv_obj_add_state(closest_btn, LV_STATE_PRESSED);
                }
                currently_pressed_btn = closest_btn;

                // 啟動長按檢測（僅針對 Space 按鍵）
                start_long_press_timer(closest_key_text);

                // 只對字母、數字和符號顯示彈出框
                if (strcmp(closest_key_text, "Close") != 0 &&
                    strcmp(closest_key_text, "Mode") != 0 &&
                    strcmp(closest_key_text, "Del") != 0 &&
                    strcmp(closest_key_text, "Space") != 0 &&
                    strcmp(closest_key_text, "Enter") != 0 &&
                    strcmp(closest_key_text, "Caps") != 0)
                {
                    show_key_popup(closest_btn, closest_key_text);
                }
            }
        }
    }
    else if (code == LV_EVENT_PRESSING && proximity_mode_active)
    {
        // 持續按住時，更新最接近的按鍵
        lv_indev_t *indev = lv_indev_get_act();
        if (indev != NULL)
        {
            lv_point_t current_point;
            lv_indev_get_point(indev, &current_point);

            lv_obj_t *new_closest = find_closest_key(current_point);
            if (new_closest != closest_btn && new_closest != NULL)
            {
                // 清除之前按鈕的按下狀態
                if (currently_pressed_btn != NULL)
                {
                    const char *prev_key_text =
                        get_button_text(currently_pressed_btn);
                    if (strcmp(prev_key_text, "Del") == 0 && del_img != NULL)
                    {
                        // 恢復Del按鈕的原始圖片
                        lv_img_set_src(del_img, &backspace_icon);
                    }
                    else
                    {
                        lv_obj_clear_state(currently_pressed_btn,
                                           LV_STATE_PRESSED);
                    }
                }

                closest_btn = new_closest;
                closest_key_text = get_button_text(closest_btn);

                // 設置新按鈕的按下狀態
                if (strcmp(closest_key_text, "Del") == 0 && del_img != NULL)
                {
                    // lv_img_set_src(del_img, &backspace_pressed_icon);
                }
                else
                {
                    lv_obj_add_state(closest_btn, LV_STATE_PRESSED);
                }
                currently_pressed_btn = closest_btn;
                // 更新彈出框
                if (strcmp(closest_key_text, "Close") != 0 &&
                    strcmp(closest_key_text, "Mode") != 0 &&
                    strcmp(closest_key_text, "Del") != 0 &&
                    strcmp(closest_key_text, "Space") != 0 &&
                    strcmp(closest_key_text, "Enter") != 0 &&
                    strcmp(closest_key_text, "Caps") != 0)
                {
                    show_key_popup(closest_btn, closest_key_text);
                }
                else
                {
                    hide_key_popup();
                }
            }
            else if (new_closest == NULL && currently_pressed_btn != NULL)
            {
                // 手指移動到沒有按鈕的區域，清除按下狀態
                const char *prev_key_text =
                    get_button_text(currently_pressed_btn);
                if (strcmp(prev_key_text, "Del") == 0 && del_img != NULL)
                {
                    // 恢復Del按鈕的原始圖片
                    lv_img_set_src(del_img, &backspace_icon);
                }
                else
                {
                    lv_obj_clear_state(currently_pressed_btn, LV_STATE_PRESSED);
                }
                currently_pressed_btn = NULL;
                closest_btn = NULL;
                hide_key_popup();
            }
        }
    }
    else if (code == LV_EVENT_RELEASED && proximity_mode_active)
    {
        // 停止長按定時器
        stop_long_press_timer();

        // 清除按鈕的按下狀態
        if (currently_pressed_btn != NULL)
        {
            const char *prev_key_text = get_button_text(currently_pressed_btn);
            if (strcmp(prev_key_text, "Del") == 0 && del_img != NULL)
            {
                // 恢復Del按鈕的原始圖片
                lv_img_set_src(del_img, &backspace_icon);
            }
            else
            {
                lv_obj_clear_state(currently_pressed_btn, LV_STATE_PRESSED);
            }
            currently_pressed_btn = NULL;
        }

        // 放開時輸入最接近的按鍵
        proximity_mode_active = false;
        hide_key_popup();

        if (closest_btn != NULL && closest_key_text != NULL)
        {
            // 如果已經觸發了長按事件，則不執行正常的按鍵功能
            if (is_long_press_triggered)
            {
                LOG_D("Long press was triggered, skipping normal key function");
                is_long_press_triggered = false;
            }
            else
            {
                LOG_D("Proximity input: %s", closest_key_text);

                // 處理功能按鍵
                if (strcmp(closest_key_text, "Close") == 0)
                {
                    toggle_keyboard_visibility();
                }
                else if (strcmp(closest_key_text, "Mode") == 0)
                {
                    // 切換鍵盤模式
                    switch (current_keyboard_mode)
                    {
                    case KEYBOARD_MODE_LETTERS:
                        current_keyboard_mode = KEYBOARD_MODE_NUMBERS;
                        break;
                    case KEYBOARD_MODE_NUMBERS:
                        current_keyboard_mode = KEYBOARD_MODE_LETTERS;
                        break;
                    }

                    // 重新建立鍵盤佈局
                    if (keyboard_container != NULL)
                    {
                        lv_obj_t *parent =
                            lv_obj_get_parent(keyboard_container);
                        bool was_visible = !lv_obj_has_flag(keyboard_container,
                                                            LV_OBJ_FLAG_HIDDEN);

                        lv_obj_del(keyboard_container);
                        create_circular_keyboard_layout(parent);
                        if (was_visible)
                        {
                            lv_obj_clear_flag(keyboard_container,
                                              LV_OBJ_FLAG_HIDDEN);
                            keyboard_visible = true;
                            update_input_display();
                            start_cursor_blink();
                        }
                    }
                }
                else if (strcmp(closest_key_text, "Caps") == 0)
                {
                    // 切換大小寫
                    is_uppercase = !is_uppercase;
                    LOG_D("Toggle case: %s", is_uppercase ? "UPPER" : "lower");

                    // 重新建立鍵盤佈局以更新字母顯示
                    if (keyboard_container != NULL &&
                        current_keyboard_mode == KEYBOARD_MODE_LETTERS)
                    {
                        lv_obj_t *parent =
                            lv_obj_get_parent(keyboard_container);
                        bool was_visible = !lv_obj_has_flag(keyboard_container,
                                                            LV_OBJ_FLAG_HIDDEN);

                        lv_obj_del(keyboard_container);
                        create_circular_keyboard_layout(parent);
                        if (was_visible)
                        {
                            lv_obj_clear_flag(keyboard_container,
                                              LV_OBJ_FLAG_HIDDEN);
                            keyboard_visible = true;
                            update_input_display();
                            start_cursor_blink();
                        }
                    }
                }
                else if (strcmp(closest_key_text, "Enter") == 0)
                {
                    LOG_D("Enter key pressed");
                    control_provider.ble_hid_keyboard_input("\n");
                    clear_input_display(); // 按下 Enter 清除輸入
                }
                else if (strcmp(closest_key_text, "Del") == 0)
                {
                    LOG_D("Delete key pressed");
                    control_provider.ble_hid_keyboard_input("\b");
                    remove_from_input_buffer(); // 從顯示中刪除字符
                }
                else if (strcmp(closest_key_text, "Space") == 0)
                {
                    LOG_D("Space key pressed");
                    control_provider.ble_hid_keyboard_input(" ");
                    add_to_input_buffer(" "); // 添加空格到顯示
                }
                else
                {
                    // 檢查是否為字母按鍵，根據大小寫狀態輸出正確的字符
                    if (current_keyboard_mode == KEYBOARD_MODE_LETTERS &&
                        strlen(closest_key_text) == 1 &&
                        ((closest_key_text[0] >= 'A' &&
                          closest_key_text[0] <= 'Z') ||
                         (closest_key_text[0] >= 'a' &&
                          closest_key_text[0] <= 'z')))
                    {
                        char output_char[2] = {0};
                        if (is_uppercase)
                        {
                            // 輸出大寫字母
                            if (closest_key_text[0] >= 'a' &&
                                closest_key_text[0] <= 'z')
                            {
                                output_char[0] =
                                    closest_key_text[0] - 32; // 轉換為大寫
                            }
                            else
                            {
                                output_char[0] =
                                    closest_key_text[0]; // 已經是大寫
                            }
                        }
                        else
                        {
                            // 輸出小寫字母
                            if (closest_key_text[0] >= 'A' &&
                                closest_key_text[0] <= 'Z')
                            {
                                output_char[0] =
                                    closest_key_text[0] + 32; // 轉換為小寫
                            }
                            else
                            {
                                output_char[0] =
                                    closest_key_text[0]; // 已經是小寫
                            }
                        }
                        output_char[1] = '\0';
                        control_provider.ble_hid_keyboard_input(output_char);
                        add_to_input_buffer(output_char); // 添加到輸入顯示
                    }
                    else
                    {
                        control_provider.ble_hid_keyboard_input(
                            closest_key_text);
                        add_to_input_buffer(closest_key_text); // 添加到輸入顯示
                    }
                }
                // 這裡可以添加其他按鍵的實際輸入處理邏輯
                // 例如：發送按鍵字符到文本框或HID設備
            }
        }

        closest_btn = NULL;
        closest_key_text = NULL;
    }
}

/**
 * @brief Create a single circular button with case-sensitive text
 */
static uint8_t keyboard_btn_size = 45; // 每個按鍵的大小
static lv_obj_t *create_circular_button(lv_obj_t *parent, const char *text,
                                        int x, int y)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, keyboard_btn_size - 5,
                    keyboard_btn_size); // 固定圓形尺寸
    lv_obj_set_pos(btn, x, y);

    // 圓形樣式
    lv_obj_set_style_radius(btn, 5, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2a2a2a), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x4a90e2),
                              LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(btn, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_PRESSED);

    // 添加文字，根據大小寫狀態調整
    lv_obj_t *label = lv_label_create(btn);

    // 如果是字母且處於字母模式，根據大小寫狀態調整顯示
    if (current_keyboard_mode == KEYBOARD_MODE_LETTERS && strlen(text) == 1 &&
        text[0] >= 'A' && text[0] <= 'Z')
    {
        static char display_text[2] = {0};
        if (is_uppercase)
        {
            display_text[0] = text[0]; // 保持大寫
        }
        else
        {
            display_text[0] = text[0] + 32; // 轉換為小寫
        }
        display_text[1] = '\0';
        lv_label_set_text(label, display_text);
    }
    else
    {
        lv_label_set_text(label, text);
    }

    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_opa(label, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_text_font(
        label, LV_EXT_FONT_GET(get_system_font_size(keyboard_text_size)),
        LV_PART_MAIN);
    // lv_obj_center(label);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -5);

    // 移除個別按鍵的事件處理，讓鍵盤容器統一處理
    // 但保持按鍵的視覺反饋
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);

    // 註冊按鍵以便接近度檢測
    register_key_button(btn);

    return btn;
}

/**
 * @brief Create circular keyboard layout
 */
static void create_circular_keyboard_layout(lv_obj_t *parent)
{
    // 創建鍵盤容器
    keyboard_container = lv_obj_create(parent);
    lv_obj_set_size(keyboard_container, 466, 300);
    lv_obj_align(keyboard_container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(keyboard_container, lv_color_hex(0x0f0f0f),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(keyboard_container, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_border_color(keyboard_container, lv_color_hex(0xFFFFFF),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(keyboard_container, 2, LV_PART_MAIN);
    lv_obj_set_style_border_opa(keyboard_container, LV_OPA_30, LV_PART_MAIN);
    lv_obj_set_style_border_side(keyboard_container, LV_BORDER_SIDE_TOP,
                                 LV_PART_MAIN);
    lv_obj_set_style_radius(keyboard_container, 25, LV_PART_MAIN);
    lv_obj_set_style_pad_all(keyboard_container, 8, LV_PART_MAIN);

    // 重置按鍵數組
    all_keys_count = 0;

    // 添加接近度檢測事件到鍵盤容器
    lv_obj_add_event_cb(keyboard_container, handle_proximity_input,
                        LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(keyboard_container, handle_proximity_input,
                        LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(keyboard_container, handle_proximity_input,
                        LV_EVENT_RELEASED, NULL);

    // 根據模式創建不同的按鍵佈局
    int row1_y = 5;
    int row1_start_x = 0;
    int row2_y = 70;
    int row2_start_x = 3;
    int row3_y = 135;
    int row3_start_x = 25;

    if (current_keyboard_mode == KEYBOARD_MODE_LETTERS)
    {
        // 第一行 - 字母鍵
        create_circular_button(keyboard_container, "Q",
                               row1_start_x + 0 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "W",
                               row1_start_x + 1 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "E",
                               row1_start_x + 2 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "R",
                               row1_start_x + 3 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "T",
                               row1_start_x + 4 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "Y",
                               row1_start_x + 5 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "U",
                               row1_start_x + 6 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "I",
                               row1_start_x + 7 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "O",
                               row1_start_x + 8 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "P",
                               row1_start_x + 9 * (keyboard_btn_size), row1_y);

        // 第二行
        create_circular_button(keyboard_container, "A",
                               row2_start_x + 0 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "S",
                               row2_start_x + 1 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "D",
                               row2_start_x + 2 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "F",
                               row2_start_x + 3 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "G",
                               row2_start_x + 4 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "H",
                               row2_start_x + 5 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "J",
                               row2_start_x + 6 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "K",
                               row2_start_x + 7 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "L",
                               row2_start_x + 8 * (keyboard_btn_size + 5),
                               row2_y);

        // Caps Lock 按鍵放在 Z 左邊
        caps_btn = lv_obj_create(keyboard_container);
        lv_obj_set_size(caps_btn, 35, 40);
        lv_obj_set_pos(caps_btn, row3_start_x, row3_y + 5);
        lv_obj_set_style_bg_opa(caps_btn, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(caps_btn, 0, LV_PART_MAIN);
        lv_obj_clear_flag(caps_btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(caps_btn, LV_OBJ_FLAG_CLICKABLE);
        register_key_button(caps_btn);

        lv_obj_t *caps_label = lv_img_create(caps_btn);
        lv_img_set_src(caps_label, &capital_icon);
        lv_obj_align(caps_label, LV_ALIGN_CENTER, 0, 0);

        // 根據大小寫狀態設置圖標透明度
        if (is_uppercase)
        {
            lv_obj_set_style_img_opa(caps_label, LV_OPA_COVER, LV_PART_MAIN);
        }
        else
        {
            lv_obj_set_style_img_opa(caps_label, LV_OPA_50, LV_PART_MAIN);
        }

        // 第三行 - Z 那排向右移動
        int row3_offset = row3_start_x + 40; // Caps 按鍵寬度 + 間距
        create_circular_button(keyboard_container, "Z",
                               row3_offset + 0 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "X",
                               row3_offset + 1 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "C",
                               row3_offset + 2 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "V",
                               row3_offset + 3 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "B",
                               row3_offset + 4 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "N",
                               row3_offset + 5 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "M",
                               row3_offset + 6 * (keyboard_btn_size + 2),
                               row3_y);
    }
    else if (current_keyboard_mode == KEYBOARD_MODE_NUMBERS)
    {
        // 數字佈局
        create_circular_button(keyboard_container, "1",
                               row1_start_x + 0 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "2",
                               row1_start_x + 1 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "3",
                               row1_start_x + 2 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "4",
                               row1_start_x + 3 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "5",
                               row1_start_x + 4 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "6",
                               row1_start_x + 5 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "7",
                               row1_start_x + 6 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "8",
                               row1_start_x + 7 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "9",
                               row1_start_x + 8 * (keyboard_btn_size), row1_y);
        create_circular_button(keyboard_container, "0",
                               row1_start_x + 9 * (keyboard_btn_size), row1_y);

        create_circular_button(keyboard_container, "!",
                               row2_start_x + 0 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "@",
                               row2_start_x + 1 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "#",
                               row2_start_x + 2 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "$",
                               row2_start_x + 3 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "%",
                               row2_start_x + 4 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "^",
                               row2_start_x + 5 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "&",
                               row2_start_x + 6 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "*",
                               row2_start_x + 7 * (keyboard_btn_size + 5),
                               row2_y);
        create_circular_button(keyboard_container, "(",
                               row2_start_x + 8 * (keyboard_btn_size + 5),
                               row2_y);

        create_circular_button(keyboard_container, ")",
                               row3_start_x + 0 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "-",
                               row3_start_x + 1 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "+",
                               row3_start_x + 2 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "=",
                               row3_start_x + 3 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "[",
                               row3_start_x + 4 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "]",
                               row3_start_x + 5 * (keyboard_btn_size + 2),
                               row3_y);
        create_circular_button(keyboard_container, "\\",
                               row3_start_x + 6 * (keyboard_btn_size + 2),
                               row3_y);
    }

    // 第四行 - 功能鍵
    int row4_y = 195;

    // 模式切換按鍵 - 動態顯示當前模式
    mode_btn = lv_obj_create(keyboard_container);
    lv_obj_set_size(mode_btn, 50, 50);
    lv_obj_set_pos(mode_btn, 75, row4_y);
    lv_obj_set_style_bg_opa(mode_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(mode_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(mode_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(mode_btn, LV_OBJ_FLAG_CLICKABLE);
    register_key_button(mode_btn);
    lv_obj_t *mode_icon = lv_img_create(mode_btn);
    lv_img_set_src(mode_icon, &erth);
    lv_obj_center(mode_icon);

    // Space 按鍵
    lv_obj_t *space_btn = lv_obj_create(keyboard_container);
    lv_obj_set_size(space_btn, 120, 50);
    lv_obj_set_pos(space_btn, 160, row4_y);
    lv_obj_set_style_bg_opa(space_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(space_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(space_btn, LV_OBJ_FLAG_SCROLLABLE);
    // lv_obj_t *space_label = lv_label_create(space_btn);
    // lv_label_set_text(space_label, "Space");
    // lv_obj_set_style_text_color(space_label, lv_color_hex(0xFFFFFF),
    // LV_PART_MAIN); lv_obj_set_style_text_opa(space_label, LV_OPA_70,
    // LV_PART_MAIN); lv_obj_align(space_label, LV_ALIGN_CENTER, -10, 0);
    // lv_obj_update_layout(space_label);
    lv_obj_t *space_icon = lv_img_create(space_btn);
    lv_img_set_src(space_icon, &space);
    lv_obj_align(space_icon, LV_ALIGN_CENTER, 0, 10);
    // lv_img_set_zoom(space_icon, 256 * 0.6); // 調整圖標大小
    lv_obj_set_style_img_opa(space_icon, LV_OPA_50, LV_PART_MAIN);

    // lv_obj_center(space_label);
    lv_obj_clear_flag(space_btn, LV_OBJ_FLAG_CLICKABLE);
    register_key_button(space_btn);
    lv_obj_t *mic_icon = lv_img_create(space_btn);
    lv_img_set_src(mic_icon, &icon_mic);
    lv_obj_align(mic_icon, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(mic_icon, 256 * 0.7); // 調整圖標大小
    lv_obj_set_style_img_opa(mic_icon, LV_OPA_30, LV_PART_MAIN);

    // Del 按鍵（原本 Enter 的位置）
    lv_obj_t *del_btn = lv_obj_create(keyboard_container);
    lv_obj_set_size(del_btn, 80, 50);
    lv_obj_set_pos(del_btn, 290, row4_y);
    lv_obj_set_style_bg_opa(del_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(del_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(del_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(del_btn, LV_OBJ_FLAG_CLICKABLE);
    register_key_button(del_btn);
    del_img = lv_img_create(del_btn);
    lv_img_set_src(del_img, &backspace_icon);
    lv_obj_align(del_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_img_opa(del_img, LV_OPA_50, LV_PART_MAIN);
    lv_obj_clear_flag(del_img, LV_OBJ_FLAG_CLICKABLE);

    int row5_y = 255;

    // Close 按鍵
    lv_obj_t *close_btn = lv_obj_create(keyboard_container);
    lv_obj_set_size(close_btn, 120, 25);
    lv_obj_set_pos(close_btn, 160, row5_y);
    lv_obj_set_style_bg_opa(close_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(close_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(close_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *close_img = lv_img_create(close_btn);
    lv_img_set_src(close_img, DOWN_ARROW);
    lv_obj_align(close_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(close_img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(close_btn, LV_OBJ_FLAG_CLICKABLE);
    register_key_button(close_btn);

    // 初始時隱藏
    lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
}

/**
 * @brief Create completely custom keyboard with custom button layout
 */
static void create_custom_keyboard(lv_obj_t *parent)
{
    // 為 466x466 圓形屏幕設計的弧形鍵盤佈局
    // 採用弧形排列，按鍵數量從上到下遞增然後遞減，符合圓形邊緣
    static const char *custom_map[] = {
        // 第一行 - 6個按鍵 (圓形頂部較窄)
        "1", "2", "3", "4", "5", "6", "\n",
        // 第二行 - 8個按鍵
        "Q", "W", "E", "R", "T", "Y", "U", "I", "\n",
        // 第三行 - 9個按鍵 (圓形中部最寬)
        "A", "S", "D", "F", "G", "H", "J", "K", "L", "\n",
        // 第四行 - 8個按鍵
        "Z", "X", "C", "V", "B", "N", "M", "P", "\n",
        // 第五行 - 6個按鍵 (圓形底部)
        "7", "8", "9", "0", "O", "Del", "\n",
        // 第六行 - 3個功能鍵 (最底部)
        "Space", "Enter", "Hide", ""};

    // 控制每個按鍵的寬度，創建弧形效果
    static const lv_btnmatrix_ctrl_t custom_ctrl[] = {
        // 第一行 - 6個按鍵，居中效果
        1, 1, 1, 1, 1, 1,
        // 第二行 - 8個按鍵
        1, 1, 1, 1, 1, 1, 1, 1,
        // 第三行 - 9個按鍵，圓形最寬處
        1, 1, 1, 1, 1, 1, 1, 1, 1,
        // 第四行 - 8個按鍵
        1, 1, 1, 1, 1, 1, 1, 1,
        // 第五行 - 6個按鍵
        1, 1, 1, 1, 1, 2,
        // 第六行 - 功能鍵
        3, 2, 2};

    custom_keyboard = lv_btnmatrix_create(parent);
    lv_btnmatrix_set_map(custom_keyboard, custom_map);
    lv_btnmatrix_set_ctrl_map(custom_keyboard, custom_ctrl);

    // 為 466x466 圓形屏幕精確調整
    int keyboard_width = 380;  // 減小寬度讓按鍵更緊湊
    int keyboard_height = 300; // 減小高度
    lv_obj_set_size(custom_keyboard, keyboard_width, keyboard_height);
    lv_obj_align(custom_keyboard, LV_ALIGN_CENTER, 0, 40);

    // 圓形屏幕專用的視覺設計
    lv_obj_set_style_bg_color(custom_keyboard, lv_color_hex(0x0f0f0f),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(custom_keyboard, LV_OPA_90, LV_PART_MAIN);

    // 漸層圓形邊框
    lv_obj_set_style_border_color(custom_keyboard, lv_color_hex(0x4a90e2),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(custom_keyboard, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(custom_keyboard, 25, LV_PART_MAIN);

    // 減少內邊距使按鍵更緊湊
    lv_obj_set_style_pad_all(custom_keyboard, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(custom_keyboard, 2, LV_PART_MAIN);

    // 按鍵設計 - 圓形按鍵
    lv_obj_set_style_bg_color(custom_keyboard, lv_color_hex(0x2a2a2a),
                              LV_PART_ITEMS);
    lv_obj_set_style_bg_color(custom_keyboard, lv_color_hex(0x4a90e2),
                              LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_text_color(custom_keyboard, lv_color_hex(0xFFFFFF),
                                LV_PART_ITEMS);

    // 設置按鍵為圓形 - 半徑設為按鍵高度的一半
    lv_obj_set_style_radius(custom_keyboard, LV_RADIUS_CIRCLE, LV_PART_ITEMS);

    // 設置固定的按鍵尺寸而不是自動填充
    lv_obj_set_style_min_width(custom_keyboard, 32, LV_PART_ITEMS);
    lv_obj_set_style_min_height(custom_keyboard, 32, LV_PART_ITEMS);
    lv_obj_set_style_max_width(custom_keyboard, 36, LV_PART_ITEMS);
    lv_obj_set_style_max_height(custom_keyboard, 36, LV_PART_ITEMS);

    lv_obj_set_style_border_width(custom_keyboard, 1, LV_PART_ITEMS);
    lv_obj_set_style_border_color(custom_keyboard, lv_color_hex(0x404040),
                                  LV_PART_ITEMS);
    lv_obj_set_style_border_opa(custom_keyboard, LV_OPA_50, LV_PART_ITEMS);

    // 設置文字大小適應圓形按鍵 (使用系統默認字體)
    // lv_obj_set_style_text_font(custom_keyboard, &lv_font_montserrat_12,
    // LV_PART_ITEMS);

    // 添加微妙的陰影效果
    lv_obj_set_style_shadow_width(custom_keyboard, 6, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(custom_keyboard, lv_color_hex(0x000000),
                                  LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(custom_keyboard, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_shadow_spread(custom_keyboard, 1, LV_PART_MAIN);

    // 按鍵輕微陰影
    lv_obj_set_style_shadow_width(custom_keyboard, 1, LV_PART_ITEMS);
    lv_obj_set_style_shadow_color(custom_keyboard, lv_color_hex(0x000000),
                                  LV_PART_ITEMS);
    lv_obj_set_style_shadow_opa(custom_keyboard, LV_OPA_30, LV_PART_ITEMS);

    // 添加事件處理
    lv_obj_add_event_cb(custom_keyboard, custom_keyboard_event_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);

    // 初始時隱藏
    lv_obj_add_flag(custom_keyboard, LV_OBJ_FLAG_HIDDEN);
}

// static void colon_blink_timer_cb(lv_timer_t *timer);
static void update_time_display(void);

/**
 * @brief Maps screen coordinates to HID touchscreen coordinates
 * @param point Pointer to the point to be mapped
 */
static void map_screen_coordinate_to_hid_touchscreen(lv_point_t *point)
{
    uint16_t origin_x = point->x;
    uint16_t origin_y = point->y;

    // Map X coordinate
    uint16_t new_x = origin_x * 128.0f / (LV_HOR_RES_MAX - 22);
    point->x = (new_x > 127) ? 127 : new_x;

    // Map Y coordinate
    uint16_t new_y = origin_y * 128.0f / (LV_VER_RES_MAX - 22);
    point->y = (new_y > 127) ? 127 : new_y;

    LOG_D("[screen]x: %d, y: %d => [map]x: %d, y: %d", origin_x, origin_y,
          point->x, point->y);
}

/**
 * @brief Checks if a point is near the edge of the screen
 * @param point The point to check
 * @return true if near edge, false otherwise
 */
static bool is_point_near_edge(const lv_point_t *point)
{
    static const uint16_t center_x = LV_HOR_RES_MAX / 2;
    static const uint16_t center_y = LV_VER_RES_MAX / 2;
    static const uint16_t radius = LV_HOR_RES_MAX / 2;

    int dx = point->x - center_x;
    int dy = point->y - center_y;
    float distance = sqrtf(dx * dx + dy * dy);

    return (radius - distance) < EDGE_THRESHOLD_PIXELS;
}

/**
 * @brief Updates edge detection flags based on touch position
 * @param point The touch point
 */
static void update_edge_detection(const lv_point_t *point)
{
    if (is_point_near_edge(point))
    {
    #if USING_EDGE_BOTTOM_DETECTION
        start_from_edge[EDGE_BOTTOM] =
            point->y > (LV_VER_RES_MAX - BOTTOM_EDGE_THRESHOLD);
    #endif
    #if USING_EDGE_LEFT_DETECTION
        start_from_edge[EDGE_LEFT] = point->x < BOTTOM_EDGE_THRESHOLD;
    #endif
    #if USING_EDGE_RIGHT_DETECTION
        start_from_edge[EDGE_RIGHT] =
            point->x > (LV_HOR_RES_MAX - BOTTOM_EDGE_THRESHOLD);
    #endif
    }
    else
    {
    #if USING_EDGE_BOTTOM_DETECTION
        start_from_edge[EDGE_BOTTOM] = false;
    #endif
    #if USING_EDGE_LEFT_DETECTION
        start_from_edge[EDGE_LEFT] = false;
    #endif
    #if USING_EDGE_RIGHT_DETECTION
        start_from_edge[EDGE_RIGHT] = false;
    #endif
    }
}

/**
 * @brief Handles scrolling logic for different edge positions
 * @param current_point Current touch point
 * @param delta_x X movement delta
 * @param delta_y Y movement delta
 */
static bool handle_edge_scrolling(const lv_point_t *current_point,
                                  int16_t delta_x, int16_t delta_y)
{
    #if USING_EDGE_BOTTOM_DETECTION
    if (start_from_edge[EDGE_BOTTOM])
    {
        if (!gesture_detected && current_point->y < (LV_VER_RES_MAX - 100))
        {
            gesture_detected = true;
            gesture_timer_enabled = true;
            start_multiple_pages_timer();
        }
        return true;
    }
    #endif
    #if USING_EDGE_RIGHT_DETECTION
    if (start_from_edge[EDGE_RIGHT])
    {
        if (!gesture_detected && current_point->x > (LV_HOR_RES_MAX - 100))
        {
            gesture_detected = true;
            control_provider.trigger_finger_event(3);
        }
        return true;
    }
    #endif
    #if USING_EDGE_LEFT_DETECTION
    if (start_from_edge[EDGE_LEFT])
    {
        if (!go_back && current_point->x > 100)
        {
            LOG_D("SCROLLING LEFT");
            go_back = true;
            motor_pattern_scrolling_app();
            // control_provider.ble_hid_consumer_back();
            control_provider.trigger_finger_event(2);
        }
        return true;
    }
    #endif
    return false;
}

/**
 * @brief Handles mouse wheel scrolling
 * @param delta_x X movement delta
 * @param delta_y Y movement delta
 */
static void handle_mouse_wheel_scrolling(int16_t delta_x, int16_t delta_y)
{
    #if USING_MOUSE_WHEEL_SCROLLING
    RT_ASSERT(control_provider.ble_hid_mouse_wheel_scroll);

    // 根据锁定的方向执行滚动
    if (is_horizontal_scroll)
    {
        // LOG_D("MOVE X : %d", delta_x);
        control_provider.ble_hid_mouse_pan_scroll(delta_x /
                                                  SCROLLING_THRESHOLD);
    }
    else
    {
        control_provider.ble_hid_mouse_wheel_scroll(delta_y /
                                                    SCROLLING_THRESHOLD);
    }
    #endif
}
    #if USING_TOUCHSCREEN_SCROLLING
/**
 * @brief Handles touchscreen scrolling
 * @param current_point Current touch point
 */
static void handle_touchscreen_scrolling(const lv_point_t *current_point)
{

    touchscreen_point = *current_point;
    map_screen_coordinate_to_hid_touchscreen(&touchscreen_point);
    if (control_provider.ble_hid_touch_screen_press != NULL)
    {
        control_provider.ble_hid_touch_screen_press(touchscreen_point.x,
                                                    touchscreen_point.y);
    }
}
    #endif

/**
 * @brief Timer callback for blinking colon in time display
 * @param timer Pointer to the timer
 */
static void update_time_symbol(void);

/**
 * @brief Update time display with or without colon
 */
static void update_time_symbol(void)
{
    if (lv_obj_is_valid(status_bar_time_symbol))
    {
        char time_str[6];
        if (colon_visible)
        {
            rt_snprintf(time_str, 6, ":");
        }
        else
        {
            rt_snprintf(time_str, 6, " ");
        }
        lv_label_set_text(status_bar_time_symbol, time_str);
    }
}

static void update_time_display(void)
{
    if (lv_obj_is_valid(status_bar_time_h) &&
        lv_obj_is_valid(status_bar_time_m))
    {
        T_UTC_TIME current_time = SkaiWatchSys.Global_Time;
        char time_str[3];
        rt_snprintf(time_str, 3, "%02d", current_time.hour);
        lv_label_set_text(status_bar_time_h, time_str);
        rt_snprintf(time_str, 3, "%02d", current_time.minutes);
        lv_label_set_text(status_bar_time_m, time_str);
    }
}

    #if USING_EDGE_BOTTOM_DETECTION
/**
 * @brief Multiple pages callback
 * @param parameter Callback parameter
 */
static void multiple_pages_cb(void *parameter)
{
    // LOG_D("multiple_pages_cb");
    LOG_D("Gesture detected: multiple_pages_cb");
    if (bottom_bar_gesture_timer_enabled)
    {
        bottom_bar_gesture_timer_enabled = false;
        // if (gesture_detected)
        // {
        // is_gesture_active = true;
        is_bottom_bar_gesture_active = true;
        // control_provider.trigger_finger_event(4);
        LOG_D("Gesture detected: 4");
        // }
    }
}

/**
 * @brief Starts the multiple pages timer
 */
static void start_multiple_pages_timer(void)
{
    if (!multiple_pages_timer)
    {
        multiple_pages_timer = rt_timer_create(
            "multiple_pages_timer", multiple_pages_cb, RT_NULL,
            rt_tick_from_millisecond(GESTURE_TIMER_MS), RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(multiple_pages_timer);
    }
    rt_timer_start(multiple_pages_timer);
}
    #endif
/**
 * @brief Inertia scroll timer callback
 */
static void inertia_scroll_timer_cb(lv_timer_t *timer)
{
    inertia_velocity *= INERTIA_DECAY_FACTOR;

    if (fabsf(inertia_velocity) < MIN_SCROLL_SPEED)
    {
        lv_timer_del(timer);
        inertia_timer = NULL;
        inertia_accumulator = 0.0f;
        return;
    }

    inertia_accumulator += inertia_velocity * (INERTIA_TIMER_MS / 1000.0f);
    int8_t scroll_val = (int8_t)inertia_accumulator;
    inertia_accumulator -= (float)scroll_val;

    if (scroll_val != 0)
    {
        if (is_horizontal_scroll)
        {
            control_provider.ble_hid_mouse_pan_scroll(scroll_val);
        }
        else
        {
            control_provider.ble_hid_mouse_wheel_scroll(scroll_val);
        }
    }
}

/**
 * @brief Handles the pressed event
 * @param indev Input device
 */
static void handle_pressed_event(lv_indev_t *indev)
{
    user_touching = true;
    // update_crosshair_brightness();
    press_time = lv_tick_get();
    air_mouse_movement_lock_reset();

    lv_indev_get_point(indev, &start_point);
    lv_indev_get_point(indev, &last_point);
    pressing = false;
    scrolling = false;
    moving = false;

    // 判斷是否按在左側滾動弧線區域
    left_scroll_active = is_point_in_left_arc(&start_point);
    if (left_scroll_active)
    {
        scroll_bar_last_point = start_point;
        set_stop_mouse_move(true);
        motor_pattern_damping();
        LOG_D("left scroll bar pressed");
    }

    // 重置滚动方向锁定
    scroll_direction_locked = false;
    is_horizontal_scroll = false;

    has_moved_during_touch = false;

    // 停止慣性滾動
    if (inertia_timer)
    {
        lv_timer_del(inertia_timer);
        inertia_timer = NULL;
    }
    inertia_velocity = 0.0f;
    inertia_accumulator = 0.0f;
    last_scroll_tick = lv_tick_get();

    #if SIMULATE_MOUSE_RIGHT_BUTTON
    pressed_left_half = start_point.x < (LV_HOR_RES_MAX / 2);
    #endif

    LOG_D("pressed x: %d, y: %d", start_point.x, start_point.y);
    update_edge_detection(&start_point);
    gesture_detected = false;

    // 雙擊拖曳：第二下按下去直接觸發長按效果
    #ifndef USE_FSR_ADC
    if (last_click_time > 0 &&
        (lv_tick_get() - last_click_time) < DOUBLE_TAP_MS)
    {
        pressing = true;
        control_provider.ble_hid_mouse_left_press();
        motor_pattern_touchpad_slide();
        LOG_D("Air mouse - double tap hold (left press)");
        last_click_time = 0;
    }
    #endif
}

/**
 * @brief Handles the pressing event
 * @param indev Input device
 * @param current_point Current touch point
 */
static void handle_pressing_event(lv_indev_t *indev,
                                  const lv_point_t *current_point)
{
    // 左側滾動弧線模式：只做上下滾輪
    if (left_scroll_active)
    {
        int16_t delta_y = current_point->y - scroll_bar_last_point.y;
        if (abs(delta_y) >= SCROLLING_THRESHOLD)
        {
            int8_t scroll_val = delta_y / SCROLLING_THRESHOLD;
            if (SkaiWatchSys.phone_os_version == IOS)
            {
                scroll_val = -scroll_val;
            }
            motor_pattern_damping();
            control_provider.ble_hid_mouse_wheel_scroll(scroll_val);
            scroll_bar_last_point = *current_point;
        }
        return;
    }

    if (pressing)
    {
        return;
    }
    int16_t delta_x = current_point->x - last_point.x;
    int16_t delta_y = current_point->y - last_point.y;
    if (handle_edge_scrolling(current_point, delta_x, delta_y))
    {
        return;
    }

    int16_t dx_from_start = current_point->x - start_point.x;
    int16_t dy_from_start = current_point->y - start_point.y;

    if ((abs(dx_from_start) >= SCROLLING_THRESHOLD) ||
        (abs(dy_from_start) >= SCROLLING_THRESHOLD))
    {
        scrolling = true;
        if (abs(dy_from_start) < abs(dx_from_start))
        {
            is_horizontal_scroll = true;
        }
        else
        {
            is_horizontal_scroll = false;
        }
        scroll_direction_locked = true;
    }

    if (scrolling)
    {
        // 每次更新 last_point，讓滑鼠移動更順暢
        last_point.x = current_point->x;
        last_point.y = current_point->y;

        if (delta_x == 0 && delta_y == 0)
        {
            // 手指靜止（沒有拖曳）→ 恢復體感滑鼠
            set_stop_mouse_move(false);
        }
        else
        {
            // 正在拖曳 → 控制滑鼠移動（距離翻倍），鎖住體感滑鼠
            set_stop_mouse_move(true);
            control_provider.ble_hid_mouse_move(delta_x * 1.5, delta_y * 1.5);
        }

        return;
    }

    // 記錄這次觸碰期間是否有移動過
    if (moving)
    {
        has_moved_during_touch = true;
    }

    // 長按觸發：超過閾值且這次觸碰期間從未移動過
    #ifndef USE_FSR_ADC
    if (!has_moved_during_touch &&
        (lv_tick_get() - press_time > PRESSED_TIME_MS))
    {
        pressing = true;
        #if SIMULATE_MOUSE_RIGHT_BUTTON
        if (lv_tick_get() - press_time > PRESSED_TIME_MS)
        {
            if (pressed_left_half)
            {
        #endif
                control_provider.ble_hid_mouse_left_press();
                motor_pattern_touchpad_slide();
                LOG_D("Air mouse - left press");

        #if SIMULATE_MOUSE_RIGHT_BUTTON
            }
            else
            {
                control_provider.ble_hid_mouse_right_press();
                LOG_D("Air mouse - right press");
            }
        }
        #endif
    }
    #endif
}

/**
 * @brief Handles the released event
 * @param indev Input device
 */
static void handle_released_event(lv_indev_t *indev)
{
    user_touching = false;

    // 左側滾動弧線放手
    if (left_scroll_active)
    {
        left_scroll_active = false;
        set_stop_mouse_move(false);
        motor_pattern_stop();
        LOG_D("left scroll bar released");
        return;
    }

    // update_crosshair_brightness();
    if (is_gesture_active)
    {
        is_gesture_active = false;
        lv_point_t now_point;
        lv_indev_get_point(indev, &now_point);

        int16_t dx = now_point.x - start_point.x;
        int16_t dy = now_point.y - start_point.y;
        int16_t distance = sqrt(dx * dx + dy * dy);

        if (distance <= GESTURE_DISTANCE_THRESHOLD)
        {
            control_provider.trigger_finger_event(6);
            LOG_D("Gesture detected: 6");
        }
    }

    #if USING_EDGE_BOTTOM_DETECTION
    if (gesture_timer_enabled)
    {
        rt_timer_stop(multiple_pages_timer);
        if (gesture_detected)
        {
            control_provider.trigger_finger_event(5);
            LOG_D("Gesture detected: 5");
        }
    }
    #endif

    if (scrolling)
    {
        scrolling = false;
        LOG_D("Gesture detected: touch mouse move released");

        // 恢復體感滑鼠
        set_stop_mouse_move(false);
    }
    else
    {
    #if SIMULATE_MOUSE_RIGHT_BUTTON
        if (pressed_left_half)
    #endif
        {
            if (pressing)
            {
                control_provider.ble_hid_mouse_left_release();
                LOG_D("Air mouse - left release");
            }
            else
            {
                if (!scrolling &&
                    (lv_tick_get() - press_time <= PRESSED_TIME_MS))
                {
                    LOG_D("Air mouse - click_left");
                    motor_pattern_tap();
                    control_provider.ble_hid_mouse_left_click();
                    last_click_time = lv_tick_get();
                }
            }
        }
    #if SIMULATE_MOUSE_RIGHT_BUTTON
        else
        {
            if (pressing)
            {
                control_provider.ble_hid_mouse_right_release();
                LOG_D("Air mouse - right release");
            }
            else
            {
                if (!scrolling &&
                    (lv_tick_get() - press_time <= PRESSED_TIME_MS))
                {
                    LOG_D("Air mouse - click_right");
                    control_provider.ble_hid_mouse_right_click();
                    motor_pattern_tap();
                }
            }
        }
    #endif
    }

    go_back = false;
    gesture_detected = false;
    #if USING_EDGE_BOTTOM_DETECTION
    gesture_timer_enabled = false;
    #endif
}
static void top_logo_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    switch (code)
    {
    case LV_EVENT_PRESSED:
        user_touching = true;
        // update_crosshair_brightness();
        break;

    case LV_EVENT_PRESSING:
        break;

    case LV_EVENT_RELEASED:
        user_touching = false;
        // update_crosshair_brightness();
        break;

    case LV_EVENT_CLICKED:
        break;

    default:
        break;
    }
}
/**
 * @brief Main event callback for touch events
 * @param e Pointer to the event
 */
static void plain_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    // indev_global = indev;

    // LOG_D("plain_event_cb: %d", code);

    switch (code)
    {
    case LV_EVENT_PRESSED:
        handle_pressed_event(indev);
        break;

    case LV_EVENT_PRESSING:
    {
        lv_point_t now_point;
        lv_indev_get_point(indev, &now_point);
        handle_pressing_event(indev, &now_point);
        break;
    }

    case LV_EVENT_RELEASED:
        handle_released_event(indev);
        break;

    case LV_EVENT_CLICKED:
        // Handle click event if needed
        break;

    default:
        break;
    }
}

    #if ENABLE_MENU_FEATURE
/**
 * @brief Tileview event callback for menu swipe
 */
static void menu_tileview_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED)
    {
        lv_obj_t *active_tile = lv_tileview_get_tile_act(obj);
        if (active_tile == menu_home_tile)
        {
            /* 滑回首頁，隱藏 tileview 並恢復滑鼠控制 */
            lv_obj_add_flag(menu_tileview, LV_OBJ_FLAG_HIDDEN);
            set_stop_mouse_move(false);
        }
        else if (active_tile == menu_content_tile)
        {
            /* 進入 menu 頁面，停止滑鼠移動 */
            set_stop_mouse_move(true);
        }
    }
}

/**
 * @brief Top swipe area callback - show menu tileview on press
 */
static void menu_swipe_area_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED)
    {
        if (menu_tileview != NULL)
        {
            LOG_D("menu_swipe_area_event_cb: show menu tileview");
            /* 設定到首頁 tile，讓使用者從頂部往下滑到 menu */
            lv_obj_set_tile(menu_tileview, menu_home_tile, LV_ANIM_OFF);
            // lv_obj_clear_flag(menu_tileview, LV_OBJ_FLAG_HIDDEN);
        }
    }
    else if (code == LV_EVENT_RELEASED)
    {
        if (lv_obj_get_scroll_x(menu_tileview) == 0 &&
            lv_obj_get_scroll_y(menu_tileview) == 466)
        {
            /* 滑回首頁，隱藏 tileview 並恢復滑鼠控制 */
            lv_obj_add_flag(menu_tileview, LV_OBJ_FLAG_HIDDEN);
            set_stop_mouse_move(false);
        }
    }
}
    #endif

    #if ENABLE_MENU_FEATURE

/**
 * @brief Calibrate button event callback
 * @param e Pointer to the event
 */
static void calibrate_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        watch_sys_sync.notify_calibration_global_attitude();
    }
}

/**
 * @brief Handfree mode switch event callback
 * @param e Pointer to the event
 */
static void handfree_mode_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    handfree = (lv_obj_get_state(obj) & LV_STATE_CHECKED) ? true : false;
}

/**
 * @brief File item clicked callback
 * @param e Pointer to the event
 */
static void file_item_clicked_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        lv_obj_t *clicked_obj = lv_event_get_target(e);
        void (*callback)(void) = (void (*)(void))lv_event_get_user_data(e);

        if (callback)
        {
            callback();
        }
    }
}

/**
 * @brief Add file to list
 * @param file_name Name of the file
 * @param callback Callback function when file is clicked
 */
static void add_file_to_list(const char *file_name, void (*callback)(void))
{
    if (file_items_count >= 10)
    {
        LOG_E("File list is full!");
        return;
    }

    strncpy(file_items[file_items_count].name, file_name,
            sizeof(file_items[file_items_count].name) - 1);
    file_items[file_items_count]
        .name[sizeof(file_items[file_items_count].name) - 1] = '\0';
    file_items[file_items_count].callback = callback;
    file_items_count++;

    // Create visual representation
    if (file_list)
    {
        lv_obj_t *item_container = lv_obj_create(file_list);
        lv_obj_set_size(item_container, lv_pct(100), 80);
        lv_obj_set_style_bg_color(item_container, lv_color_hex(0x333333), 0);
        lv_obj_set_style_bg_opa(item_container, LV_OPA_80, 0);
        lv_obj_set_style_radius(item_container, 10, 0);
        lv_obj_set_style_border_width(item_container, 2, 0);
        lv_obj_set_style_border_color(item_container, lv_color_hex(0x666666),
                                      0);
        lv_obj_add_event_cb(item_container, file_item_clicked_cb,
                            LV_EVENT_CLICKED, callback);

        // Create circular icon placeholder
        lv_obj_t *icon = lv_obj_create(item_container);
        lv_obj_set_size(icon, 50, 50);
        lv_obj_set_style_bg_color(icon, lv_color_hex(0x00AA00), 0);
        lv_obj_set_style_radius(icon, 25, 0);
        lv_obj_align(icon, LV_ALIGN_LEFT_MID, 10, 0);

        // Create file name label
        lv_obj_t *label = lv_label_create(item_container);
        lv_label_set_text(label, file_name);
        lv_obj_align_to(label, icon, LV_ALIGN_OUT_RIGHT_MID, 15, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
    }
}

/**
 * @brief Dummy file 1 callback
 */
static void dummy_file1_callback(void)
{
    LOG_I("Dummy File 1 clicked!");
    // Here you would handle the file selection
}

/**
 * @brief Dummy file 2 callback
 */
static void dummy_file2_callback(void)
{
    LOG_I("Dummy File 2 clicked!");
    // Here you would handle the file selection
}

/**
 * @brief Dummy file 3 callback
 */
static void dummy_file3_callback(void)
{
    LOG_I("Dummy File 3 clicked!");
    // Here you would handle the file selection
}

// Forward declarations for BLE functions
extern void ble_app_advertising_start(bool restart_adv, bool mouse_mode,
                                      bool pairing_mode);

// Menu device list UI state
static struct
{
    lv_obj_t *device_list;
    lv_obj_t *empty_label;
    lv_obj_t *reset_btn;
} menu_dev_list_ui = {0};

        // Extra long press (1.2 seconds) for device deletion
        #define MENU_EXTRA_LONG_PRESS_MS 800
static lv_timer_t *menu_delete_timer = NULL;
static uint8_t menu_delete_device_idx = 0xFF;
static lv_obj_t *menu_delete_target_btn = NULL;

// Delete confirmation dialog
static lv_obj_t *menu_delete_confirm_msgbox = NULL;
static uint8_t menu_pending_delete_idx = 0xFF;

/**
 * @brief Device item click callback for menu
 */
/**
 * @brief Timer callback for extra long press device deletion (runs in LVGL
 * context)
 */
/**
 * @brief Delete confirmation dialog event callback
 */
static void menu_delete_confirm_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_current_target(e);
    const char *btn_txt = lv_msgbox_get_active_btn_text(obj);

    if (btn_txt)
    {
        if (strcmp(btn_txt, "Yes") == 0)
        {
            // User confirmed deletion
            if (menu_pending_delete_idx != 0xFF)
            {
                LOG_I("User confirmed: disconnecting and deleting device "
                      "[%d]",
                      menu_pending_delete_idx);
                // First disconnect the device if connected
                ble_dev_mgr_disconnect_device(menu_pending_delete_idx);
                // Then remove the device from database
                ble_dev_mgr_remove_device(menu_pending_delete_idx);
                menu_pending_delete_idx = 0xFF;
            }
        }
        else
        {
            // User cancelled
            LOG_D("User cancelled device deletion");
            menu_pending_delete_idx = 0xFF;
        }
    }

    // Close the message box
    lv_msgbox_close(obj);
    menu_delete_confirm_msgbox = NULL;
}

/**
 * @brief Show delete confirmation dialog
 */
static void menu_show_delete_confirm(uint8_t device_idx)
{
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db || device_idx >= MAX_BONDED_DEVICES)
    {
        return;
    }

    const bonded_device_t *dev = &db->devices[device_idx];
    if (!dev->is_valid)
    {
        return;
    }

    // Close existing dialog if any
    if (menu_delete_confirm_msgbox)
    {
        lv_msgbox_close(menu_delete_confirm_msgbox);
        menu_delete_confirm_msgbox = NULL;
    }

    menu_pending_delete_idx = device_idx;

    // Create confirmation message
    static char msg_buf[128];
    lv_snprintf(msg_buf, sizeof(msg_buf), "Delete device?\n%s",
                dev->device_name);

    // Button texts
    static const char *btns[] = {"Yes", "No", ""};

    // Create message box
    menu_delete_confirm_msgbox =
        lv_msgbox_create(NULL, "Confirm Delete", msg_buf, btns, false);
    lv_obj_set_style_bg_color(menu_delete_confirm_msgbox,
                              lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_text_color(menu_delete_confirm_msgbox,
                                lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(menu_delete_confirm_msgbox);

    // Add event callback
    lv_obj_add_event_cb(menu_delete_confirm_msgbox, menu_delete_confirm_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
}

static void menu_delete_timer_cb(lv_timer_t *timer)
{
    if (menu_delete_device_idx != 0xFF)
    {
        LOG_I("Extra long press triggered: showing delete confirmation for "
              "device [%d]",
              menu_delete_device_idx);
        uint8_t idx_to_delete = menu_delete_device_idx;
        menu_delete_device_idx = 0xFF;
        menu_delete_target_btn = NULL;

        // Delete timer after use
        if (menu_delete_timer)
        {
            lv_timer_del(menu_delete_timer);
            menu_delete_timer = NULL;
        }

        // Show confirmation dialog instead of directly deleting
        menu_show_delete_confirm(idx_to_delete);
    }
}

/**
 * @brief Start extra long press timer for device deletion
 */
static void menu_start_delete_timer(uint8_t device_idx, lv_obj_t *btn)
{
    menu_delete_device_idx = device_idx;
    menu_delete_target_btn = btn;

    // Stop existing timer if any
    if (menu_delete_timer)
    {
        lv_timer_del(menu_delete_timer);
        menu_delete_timer = NULL;
    }

    // Create LVGL timer - runs in LVGL context, safe for UI operations
    menu_delete_timer = lv_timer_create(
        menu_delete_timer_cb,
        MENU_EXTRA_LONG_PRESS_MS - 400, // subtract LVGL long press time
        NULL);
    lv_timer_set_repeat_count(menu_delete_timer, 1); // One-shot
    LOG_D("Delete timer started for device [%d]", device_idx);
}

/**
 * @brief Stop extra long press timer
 */
static void menu_stop_delete_timer(void)
{
    if (menu_delete_timer)
    {
        lv_timer_del(menu_delete_timer);
        menu_delete_timer = NULL;
    }
    menu_delete_device_idx = 0xFF;
    menu_delete_target_btn = NULL;
}

static uint8_t control_device_idx = 0;
static void menu_device_item_click_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);
    uint8_t device_idx = (uint8_t)(uintptr_t)lv_obj_get_user_data(btn);

    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db || device_idx >= MAX_BONDED_DEVICES)
    {
        return;
    }

    const bonded_device_t *dev = &db->devices[device_idx];
    if (!dev->is_valid)
    {
        return;
    }

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        LOG_D("Menu device clicked: idx=%d, name=%s, conn_idx=%d", device_idx,
              dev->device_name, dev->conn_idx);
        menu_stop_delete_timer();
        // Use the actual connection index from the device database
        ble_hid_set_conn_idx(dev->conn_idx);
        control_device_idx = device_idx;
        // ble_dev_mgr_connect_device(device_idx);
        int ret = ble_dev_mgr_set_active_device(device_idx);
    }
    else if (LV_EVENT_PRESSED == event)
    {
        LOG_D("Menu device long pressed: idx=%d", device_idx);
        // Start extra long press timer for deletion
        // But do not allow deletion of main phone device
        extern uint8_t get_main_phonepeer_conn_idx(void);
        uint8_t main_phone_conn_idx = get_main_phonepeer_conn_idx();
        bool is_main_phone = (main_phone_conn_idx != 0xFF &&
                              dev->conn_idx == main_phone_conn_idx);
        if (!menu_delete_timer && !is_main_phone)
        {
            menu_start_delete_timer(device_idx, btn);
        }
    }
    else if (LV_EVENT_RELEASED == event || LV_EVENT_PRESS_LOST == event)
    {
        // Stop delete timer if user releases before 2 seconds
        menu_stop_delete_timer();
    }
}

/**
 * @brief Create a device list item for menu
 */
static lv_obj_t *menu_create_device_item(lv_obj_t *parent,
                                         const bonded_device_t *device,
                                         uint8_t device_idx)
{
    int active_idx = control_device_idx;
    bool is_active = (active_idx == device_idx);

    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, LV_PCT(48), 70);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x3A3A3A), LV_STATE_PRESSED);
    lv_obj_set_style_pad_all(btn, 6, 0);

    if (is_active)
    {
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x00AAFF), 0);
    }
    else
    {
        lv_obj_set_style_border_width(btn, 0, 0);
    }

    lv_obj_set_user_data(btn, (void *)(uintptr_t)device_idx);

    // Connection status LED indicator - positioned on the left, vertically
    // centered
    lv_obj_t *led_indicator = lv_obj_create(btn);
    lv_obj_set_size(led_indicator, 12, 12);
    lv_obj_set_style_radius(led_indicator, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(led_indicator, 0, 0);
    lv_obj_set_style_pad_all(led_indicator, 0, 0);
    lv_obj_align(led_indicator, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_clear_flag(led_indicator,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    if (device->conn_idx != 0xFF)
    {
        // Connected - Green LED
        lv_obj_set_style_bg_color(led_indicator, lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_shadow_color(led_indicator, lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_shadow_width(led_indicator, 8, 0);
        lv_obj_set_style_shadow_spread(led_indicator, 2, 0);
    }
    else
    {
        // Not connected - Gray LED
        lv_obj_set_style_bg_color(led_indicator, lv_color_hex(0x666666), 0);
    }

    // Text container - positioned after LED
    lv_obj_t *text_cont = lv_obj_create(btn);
    lv_obj_set_size(text_cont, LV_PCT(85), LV_SIZE_CONTENT);
    lv_obj_align(text_cont, LV_ALIGN_LEFT_MID, 18, 0);
    lv_obj_set_style_bg_opa(text_cont, LV_OPA_0, 0);
    lv_obj_set_style_border_width(text_cont, 0, 0);
    lv_obj_set_style_pad_all(text_cont, 0, 0);
    lv_obj_set_flex_flow(text_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(text_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(text_cont, 2, 0);
    lv_obj_clear_flag(text_cont, LV_OBJ_FLAG_CLICKABLE);
    // LOG_D("device type: %d,name: %s,conn_idx: %d", device->device_type,
    // device->device_name,
    //       device->conn_idx);

    // Device name
    lv_obj_t *name_label = lv_label_create(text_cont);
    lv_label_set_text(name_label, device->device_name);
    lv_obj_set_style_text_color(name_label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_long_mode(name_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(name_label, LV_PCT(100));
    lv_obj_set_style_text_align(name_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_clear_flag(name_label, LV_OBJ_FLAG_CLICKABLE);

    return btn;
}

// Forward declaration
static void menu_reset_ble_btn_cb(lv_event_t *e);
static void menu_close_btn_event_cb(lv_event_t *e);

/**
 * @brief Refresh menu device list
 */
static void menu_refresh_device_list(void)
{
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db)
    {
        return;
    }

    // Sync control_device_idx and g_conn_idx from active_device_idx (source of
    // truth)
    int active_idx = ble_dev_mgr_get_active_device();
    if (active_idx >= 0 && active_idx < MAX_BONDED_DEVICES &&
        db->devices[active_idx].is_valid)
    {
        control_device_idx = (uint8_t)active_idx;
        if (db->devices[active_idx].conn_idx != 0xFF)
        {
            ble_hid_set_conn_idx(db->devices[active_idx].conn_idx);
        }
    }
    if (menu_dev_list_ui.device_list &&
        lv_obj_is_valid(menu_dev_list_ui.device_list))
    {
        lv_obj_clean(menu_dev_list_ui.device_list);
    }
    if (db->count == 0)
    {
        if (menu_dev_list_ui.empty_label &&
            lv_obj_is_valid(menu_dev_list_ui.empty_label))
        {
            lv_obj_clear_flag(menu_dev_list_ui.empty_label, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }
    else
    {
        if (menu_dev_list_ui.empty_label &&
            lv_obj_is_valid(menu_dev_list_ui.empty_label))
        {
            lv_obj_add_flag(menu_dev_list_ui.empty_label, LV_OBJ_FLAG_HIDDEN);
        }
    }

    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (db->devices[i].is_valid && menu_dev_list_ui.device_list &&
            lv_obj_is_valid(menu_dev_list_ui.device_list))
        {
            lv_obj_t *item = menu_create_device_item(
                menu_dev_list_ui.device_list, &db->devices[i], i);
            lv_obj_add_event_cb(item, menu_device_item_click_cb,
                                LV_EVENT_SHORT_CLICKED, NULL);
            lv_obj_add_event_cb(item, menu_device_item_click_cb,
                                LV_EVENT_PRESSED, NULL);
            lv_obj_add_event_cb(item, menu_device_item_click_cb,
                                LV_EVENT_RELEASED, NULL);
            lv_obj_add_event_cb(item, menu_device_item_click_cb,
                                LV_EVENT_PRESS_LOST, NULL);
        }
    }
    if (menu_dev_list_ui.device_list &&
        lv_obj_is_valid(menu_dev_list_ui.device_list))
    {
        // Add Reset BLE button at the end of device list
        lv_obj_t *reset_btn = lv_btn_create(menu_dev_list_ui.device_list);
        lv_obj_set_size(reset_btn, LV_PCT(100), 50);
        lv_obj_set_style_radius(reset_btn, 8, 0);
        lv_obj_set_style_bg_color(reset_btn, lv_color_hex(0x0066CC), 0);
        lv_obj_set_style_bg_color(reset_btn, lv_color_hex(0x0055AA),
                                  LV_STATE_PRESSED);
        lv_obj_add_event_cb(reset_btn, menu_reset_ble_btn_cb,
                            LV_EVENT_SHORT_CLICKED, NULL);

        lv_obj_t *reset_label = lv_label_create(reset_btn);
        lv_label_set_text(reset_label, "Add Device");
        lv_obj_set_style_text_color(reset_label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(reset_label);
        menu_dev_list_ui.reset_btn = reset_btn;

        lv_obj_t *close_app_btn = lv_btn_create(menu_dev_list_ui.device_list);
        lv_obj_set_size(close_app_btn, LV_PCT(100), 50);
        lv_obj_set_style_radius(close_app_btn, 8, 0);
        lv_obj_set_style_bg_color(close_app_btn, lv_color_hex(0xCC0000), 0);
        lv_obj_set_style_bg_color(close_app_btn, lv_color_hex(0xAA0000),
                                  LV_STATE_PRESSED);
        lv_obj_add_event_cb(close_app_btn, menu_close_btn_event_cb,
                            LV_EVENT_SHORT_CLICKED, NULL);
        lv_obj_t *close_label = lv_label_create(close_app_btn);
        lv_label_set_text(close_label, "Exit Mouse App");
        lv_obj_set_style_text_color(close_label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(close_label);
    }
}

/**
 * @brief Reset BLE button callback for menu
 */
static void menu_reset_ble_btn_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event)
    {
        LOG_I("Menu: Reset BLE advertising");
        ble_app_advertising_start(true, true, false);
    }
}

static void menu_close_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();
    }
}

/**
 * @brief Device manager event callback for menu
 */
void refresh_connected_device_label(void)
{
    if (!lv_obj_is_valid(connected_device_label))
        return;

    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    int active_idx = ble_dev_mgr_get_active_device();
    if (db && active_idx >= 0 && active_idx < MAX_BONDED_DEVICES &&
        db->devices[active_idx].is_valid &&
        db->devices[active_idx].conn_idx != 0xFF)
    {
        lv_label_set_text(connected_device_label,
                          db->devices[active_idx].device_name);
    }
    else
    {
        lv_label_set_text(connected_device_label, "");
    }
}

static void menu_dev_mgr_event_cb(dev_mgr_event_t event, uint8_t device_idx,
                                  void *user_data)
{
    LOG_I("Menu dev mgr event: %d, idx: %d", event, device_idx);
    if (gui_app_is_actived(APP_ID_MOUSE))
    {
        menu_refresh_device_list();
        refresh_connected_device_label();
    }
}

/**
 * @brief Creates the menu window
 * @param par Parent object
 * @return Pointer to the created menu object
 */
static lv_obj_t *menu_window(lv_obj_t *par)
{
    /* 建立 tileview: 上方是首頁（透明），下方是 menu 內容 */
    menu_tileview = lv_tileview_create(par);
    lv_obj_set_size(menu_tileview, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_scrollbar_mode(menu_tileview, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_opa(menu_tileview, LV_OPA_TRANSP, LV_PART_MAIN);

    /* Tile 0 (col=0, row=0): Menu 內容頁（上方）- 可以往下滑回首頁 */
    menu_content_tile =
        lv_tileview_add_tile(menu_tileview, 0, 0, LV_DIR_BOTTOM);
    lv_obj_set_size(menu_content_tile, LV_HOR_RES_MAX, LV_VER_RES_MAX);

    /* Tile 1 (col=0, row=1): 首頁（下方）- 透明，可以往上滑開啟 menu */
    menu_home_tile = lv_tileview_add_tile(menu_tileview, 0, 1, LV_DIR_TOP);
    lv_obj_set_style_bg_opa(menu_home_tile, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_size(menu_home_tile, LV_HOR_RES_MAX, LV_VER_RES_MAX);

    lv_obj_t *device_page_bar = lv_obj_create(menu_home_tile);
    lv_obj_set_size(device_page_bar, 100, 10);
    lv_obj_set_style_bg_color(device_page_bar, lv_color_hex(0x5B5B5B),
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(device_page_bar, LV_ALIGN_TOP_MID, 0, 50);

    /* Menu 背景 */
    menu_bg = lv_obj_create(menu_content_tile);
    lv_obj_set_size(menu_bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(menu_bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(menu_bg, LV_OPA_80, 0);
    lv_obj_set_style_radius(menu_bg, 233, 0);
    lv_obj_align(menu_bg, LV_ALIGN_CENTER, 0, 0);

    // Enable scrolling for menu_bg
    lv_obj_set_style_pad_all(menu_bg, 10, 0);
    lv_obj_set_flex_flow(menu_bg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(menu_bg, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(menu_bg, 8, 0);

    // Title label
    lv_obj_t *title_label = lv_label_create(menu_bg);
    lv_label_set_text(title_label, "Bluetooth Devices");
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xFFFFFF), 0);

    // Device list container
    lv_obj_t *device_list = lv_obj_create(menu_bg);
    lv_obj_set_size(device_list, LV_PCT(100), 380);
    lv_obj_set_style_bg_opa(device_list, LV_OPA_0, 0);
    lv_obj_set_style_border_width(device_list, 0, 0);
    lv_obj_set_style_pad_all(device_list, 0, 0);
    lv_obj_set_flex_flow(device_list, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(device_list, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(device_list, 6, 0);
    lv_obj_set_style_pad_column(device_list, 6, 0);
    menu_dev_list_ui.device_list = device_list;

    // Empty state label
    lv_obj_t *empty_label = lv_label_create(menu_bg);
    lv_label_set_text(empty_label, "No paired devices");
    lv_obj_set_style_text_color(empty_label, lv_color_hex(0x888888), 0);
    lv_obj_add_flag(empty_label, LV_OBJ_FLAG_HIDDEN);
    menu_dev_list_ui.empty_label = empty_label;

    // Register device manager callback and refresh list
    ble_dev_mgr_register_callback(menu_dev_mgr_event_cb, NULL);
    menu_refresh_device_list();

    /* 預設顯示首頁 tile 並隱藏整個 tileview */
    lv_obj_set_tile(menu_tileview, menu_home_tile, LV_ANIM_OFF);
    lv_obj_add_flag(menu_tileview, LV_OBJ_FLAG_HIDDEN);

    /* 註冊 tileview 事件 */
    lv_obj_add_event_cb(menu_tileview, menu_tileview_event_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);

    return par;
}
    #endif

// static lv_obj_t *text_input_bar = NULL;
static uint8_t text_input_bar_pressed_count = 0;
static bool text_input_bar_pressed = false;
static rt_tick_t text_input_bar_pressing_time = NULL;
static rt_tick_t text_input_bar_press_time = NULL;
static uint16_t max_move_y = 0;
static uint8_t test_count = 0;
static float prev_elapsed = 0.0f;
static void text_input_bar_cb(lv_event_t *e)
{
    // Don't handle gestures when keyboard is visible (bar is now input display)
    if (keyboard_visible)
        return;

    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();

    switch (code)
    {
    case LV_EVENT_PRESSED:
        lv_indev_get_point(indev, &bottom_bar_start_point);
        lv_indev_get_point(indev, &bottom_bar_last_point);
        text_input_bar_press_time = rt_tick_get();
        max_move_y = 0;
        notify_provider.holding_displacement(0, 0, 0);
        break;

    case LV_EVENT_PRESSING:
    {
        lv_point_t bottom_bar_now_point;
        lv_indev_get_point(indev, &bottom_bar_now_point);
        // handle_pressing_event(indev, &bottom_bar_now_point);
        int dx = bottom_bar_now_point.x - bottom_bar_start_point.x;
        int dy = bottom_bar_now_point.y - bottom_bar_start_point.y;
        uint16_t move_y = abs(dy);
        if (move_y > max_move_y)
            max_move_y = move_y;

        notify_provider.holding_displacement(1, dx, dy);
        if (is_bottom_bar_gesture_active)
            return;
        if (move_y > 10)
        {
            uint8_t text_input_bar_move_y = 10 + (move_y - 10) / 20;
            lv_obj_align(text_input_bar, LV_ALIGN_BOTTOM_MID, 0,
                         -text_input_bar_move_y);
    #if USING_EDGE_BOTTOM_DETECTION
            if (!bottom_bar_gesture_timer_enabled && move_y > 150)
            {
                bottom_bar_gesture_timer_enabled = true;
                start_multiple_pages_timer();
            }
    #endif
        }
        else if (max_move_y <= 10 &&
                 rt_tick_get() - text_input_bar_press_time >= 500 &&
                 !text_input_bar_pressed)
        {
            if (!text_input_bar_pressed)
            {
                text_input_bar_pressed = true;
                motor_pattern_wheel_scrolling();
                extern void set_voice_recognition_notified_from_mouse(
                    bool status);
                set_voice_recognition_notified_from_mouse(true);
                watch_system_interact(INTERACT_MIC_LISTEN,
                                      &text_input_bar_pressed);
                LOG_D("Gesture detected: voice recognition");
            }
        }

        // Map (rt_tick_get() - text_input_bar_press_time) from 0~500 to
        // 1~1.2
        float elapsed = (float)(rt_tick_get() - text_input_bar_press_time);
        if (fabs(elapsed - prev_elapsed) > 30 && elapsed < 500.0f)
        {
            // LOG_D("elapsed: %f, prev_elapsed: %f", elapsed,
            // prev_elapsed);
            prev_elapsed = elapsed;
            float scale = 1.0f + (elapsed / 500.0f) * 0.2f;
            if (scale > 1.2f)
                scale = 1.2f;
            if (scale < 1.0f)
                scale = 1.0f;
            // Use scale as needed, for example:
            lv_obj_set_size(text_input_bar, 100 * scale, 10);
        }

        break;
    }

    case LV_EVENT_RELEASED:
        // handle_released_event(indev);
        {
            lv_point_t bottom_bar_now_point;
            lv_indev_get_point(indev, &bottom_bar_now_point);
            uint16_t move_y =
                abs(bottom_bar_now_point.y - bottom_bar_start_point.y);
            int dx = bottom_bar_now_point.x - bottom_bar_start_point.x;
            int dy = bottom_bar_now_point.y - bottom_bar_start_point.y;
            notify_provider.holding_displacement(2, dx, dy);
            LOG_D("Gesture detected: bottom bar released, move_y: %d", move_y);
            lv_obj_set_size(text_input_bar, 100, 10);
            lv_obj_align(text_input_bar, LV_ALIGN_BOTTOM_MID, 0, -10);
            if (move_y < 60 && is_bottom_bar_gesture_active)
            {
                // control_provider.trigger_finger_event(6);
            }
            if (!text_input_bar_pressed && max_move_y < 60 &&
                !bottom_bar_gesture_timer_enabled &&
                !is_bottom_bar_gesture_active)
            {
                LOG_D("Gesture detected: short press");
                // Toggle keyboard visibility when short press is detected
                toggle_keyboard_visibility();
            }
            else if (text_input_bar_pressed)
            {
                text_input_bar_pressed = false;
            }
    #if USING_EDGE_BOTTOM_DETECTION
            if (bottom_bar_gesture_timer_enabled)
            {
                bottom_bar_gesture_timer_enabled = false;
                rt_timer_stop(multiple_pages_timer);
                // control_provider.trigger_finger_event(5);
            }
    #endif

            is_bottom_bar_gesture_active = false;
        }
        break;

    case LV_EVENT_CLICKED:
        // Handle click event if needed
        break;

    default:
        break;
    }
}

/**
 * @brief Initialize FSR-402 ADC device
 */
static void fsr_adc_init(void)
{
    HAL_PIN_Set_Analog(PAD_PB25, 1);
    fsr_adc_dev = rt_device_find(FSR_ADC_DEV_NAME);
    if (fsr_adc_dev != NULL)
    {
        rt_adc_enable((rt_adc_device_t)fsr_adc_dev, FSR_ADC_CHANNEL);
        LOG_I("FSR-402 ADC initialized on channel %d", FSR_ADC_CHANNEL);
    }
    else
    {
        LOG_E("FSR-402 ADC device not found!");
    }
}

/**
 * @brief Deinitialize FSR-402 ADC device
 */
static void fsr_adc_deinit(void)
{
    if (fsr_adc_dev != NULL)
    {
        rt_adc_disable((rt_adc_device_t)fsr_adc_dev, FSR_ADC_CHANNEL);
        fsr_adc_dev = NULL;
    }
}

/**
 * @brief Read FSR-402 ADC value
 * @return ADC value in 0.1mV units
 */
static rt_uint32_t fsr_adc_read_value(void)
{
    if (fsr_adc_dev == NULL)
        return 0;
    return rt_adc_read((rt_adc_device_t)fsr_adc_dev, FSR_ADC_CHANNEL);
}

/**
 * @brief LVGL timer callback for periodic FSR ADC reading and display update
 */
static void fsr_adc_timer_cb(void *parameter)
{
    peripheral_provider.read_fsr_adc();
}

static rt_timer_t fsr_press_timer = NULL;
static bool fsr_press_timer_active = false;
static bool mouse_pressed = false;
static void fsr_press_timer_cb(void *parameter)
{
    if (user_touching)
    {
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_MOUSE_LONG_PRESS;
        lvgl_send_msg(msg);
    }
}

void fsr_long_press(void)
{
    // This function can be called from LVGL context when a long press is
    // detected
    if (mouse_pressed)
    {
        LOG_D("FSR long press detected: sending left click");
        control_provider.ble_hid_mouse_left_press();
        motor_pattern_touchpad_slide();
        fsr_press_timer_active = false;
    }
}

static void start_fsr_press_timer(void)
{
    fsr_press_timer_active = true;
    if (fsr_press_timer == NULL)
    {
        fsr_press_timer = rt_timer_create("fsr_press", fsr_press_timer_cb, NULL,
                                          30, RT_TIMER_FLAG_PERIODIC);
    }
    if (fsr_press_timer)
    {
        rt_timer_start(fsr_press_timer);
    }
}

static void stop_fsr_press_timer(void)
{
    fsr_press_timer_active = false;
    if (fsr_press_timer)
    {
        rt_timer_stop(fsr_press_timer);
    }
}

static float prev_fsr_adc = 0.0f;
void fsr_adc_read(void)
{
    // int duration = rt_tick_get();
    #ifdef USE_FSR_ADC
    fsr_adc_value = fsr_adc_read_value();
    // LOG_D("FSR ADC raw value: %d (%.1fmV)", fsr_adc_value, fsr_adc_value / 10.0f);
    // duration = rt_tick_get() - duration;
    // LOG_D("FSR ADC read and process duration: %d ms", duration);
    // if (fsr_adc_label != NULL && lv_obj_is_valid(fsr_adc_label))
    // {
    //     char buf[48];
    //     rt_snprintf(buf, sizeof(buf), "FSR: %d.%dmV", fsr_adc_value / 10,
    //                 fsr_adc_value % 10);
    //     // LOG_D("FSR ADC value: %s", buf);
    //     lv_label_set_text(fsr_adc_label, buf);
    // }

    // LOG_D("fsr_adc_diff from prev: %.2fmV",
    //       (fsr_adc_value / 10.0f) - prev_fsr_adc);

    if (fabs((fsr_adc_value / 10.0f) - prev_fsr_adc) > FRC_THRESHOLD_BTN)
    {
        if ((fsr_adc_value / 10.0f) < prev_fsr_adc && !mouse_pressed)
        {
            LOG_D("FSR pressed");
            mouse_pressed = true;
            start_fsr_press_timer();
        }
        else if ((fsr_adc_value / 10.0f) > prev_fsr_adc && mouse_pressed)
        {

            mouse_pressed = false;
            if (fsr_press_timer_active)
            {
                control_provider.ble_hid_mouse_left_click();
                motor_pattern_touchpad_slide();
                LOG_D("FSR click");
            }
            else
            {
                control_provider.ble_hid_mouse_left_release();
                LOG_D("FSR released");
            }
            stop_fsr_press_timer();
        }
    }
    if (fabs((fsr_adc_value / 10.0f) - prev_fsr_adc) >
            FRC_THRESHOLD_MOVE_LOCK &&
        !mouse_pressed)
    {
        fsr_change_time = rt_tick_get();
    }

    prev_fsr_adc = fsr_adc_value / 10.0f;

    #endif
}

/**
 * @brief Creates the mouse screen
 * @param scr Screen object
 */
void lv_create_mouse_screen(lv_obj_t *scr)
{
    lv_obj_t *bg = common_black_bg(scr);
    lv_obj_set_scrollbar_mode(bg, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);

    // Touch background
    lv_obj_t *touch_bg = lv_obj_create(bg);
    lv_obj_set_size(touch_bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(touch_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(touch_bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(touch_bg, LV_OPA_0, 0);
    lv_obj_add_event_cb(touch_bg, plain_event_cb, LV_EVENT_ALL, NULL);

    // 左側滾動弧形條（貼著圓形畫面左側邊緣）
    left_scroll_bar = (lv_obj_t *)lv_arc_create(bg);
    lv_obj_set_size(left_scroll_bar, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(left_scroll_bar, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation((lv_obj_t *)left_scroll_bar, 0);
    lv_arc_set_bg_angles((lv_obj_t *)left_scroll_bar, 150, 210);
    // 前景角度設為 0，讓 indicator 不顯示
    lv_arc_set_angles((lv_obj_t *)left_scroll_bar, 0, 0);
    lv_arc_set_mode((lv_obj_t *)left_scroll_bar, LV_ARC_MODE_NORMAL);
    // 隱藏旋鈕
    lv_obj_set_style_pad_all(left_scroll_bar, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(left_scroll_bar, LV_OPA_0, LV_PART_KNOB);
    // 背景弧線樣式（只顯示背景弧線）
    lv_obj_set_style_arc_width(left_scroll_bar, 30, LV_PART_MAIN);
    lv_obj_set_style_arc_color(left_scroll_bar, lv_color_hex(0x333333),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_opa(left_scroll_bar, LV_OPA_60, LV_PART_MAIN);
    // 隱藏前景 indicator
    lv_obj_set_style_arc_width(left_scroll_bar, 0, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(left_scroll_bar, LV_OPA_0, LV_PART_INDICATOR);
    lv_obj_clear_flag(left_scroll_bar, LV_OBJ_FLAG_SCROLLABLE);
    // 純視覺，不攔截觸碰（由 plain_event_cb 判斷位置）
    lv_obj_clear_flag(left_scroll_bar, LV_OBJ_FLAG_CLICKABLE);

    // Crosshair lines (dimmed by default, brighten when touching logo)
    lv_coord_t line_width = 3;
    lv_coord_t line_length = LV_HOR_RES_MAX - 6;
    // crosshair_line1 = lv_obj_create(bg);
    // lv_obj_set_size(crosshair_line1, line_length, line_width);
    // lv_obj_align(crosshair_line1, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_set_style_bg_color(crosshair_line1, lv_color_hex(0x666666), 0);

    // crosshair_line2 = lv_obj_create(bg);
    // lv_obj_set_size(crosshair_line2, line_width, line_length);
    // lv_obj_align(crosshair_line2, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_set_style_bg_color(crosshair_line2, lv_color_hex(0x666666), 0);

    // // SKAI logo area with background (blocks mouse events)
    // lv_obj_t *skai_logo_area = lv_obj_create(bg);
    // lv_obj_set_size(skai_logo_area, LV_HOR_RES_MAX, 100);
    // lv_obj_align(skai_logo_area, LV_ALIGN_TOP_MID, 0, 0);
    // lv_obj_set_style_bg_color(skai_logo_area, lv_color_hex(0x1a1a1a),
    //                           LV_PART_MAIN);
    // lv_obj_set_style_bg_opa(skai_logo_area, LV_OPA_80, LV_PART_MAIN);
    // lv_obj_set_style_border_width(skai_logo_area, 0, LV_PART_MAIN);
    // lv_obj_set_style_radius(skai_logo_area, 0, LV_PART_MAIN);
    // lv_obj_clear_flag(skai_logo_area, LV_OBJ_FLAG_SCROLLABLE);
    // // Block touch events from propagating to touch_bg
    // lv_obj_add_flag(skai_logo_area, LV_OBJ_FLAG_CLICKABLE);
    // lv_obj_add_event_cb(skai_logo_area, top_logo_event_cb, LV_EVENT_ALL,
    // NULL);

    // SKAI logo at top center
    // lv_obj_t *skai_logo = lv_img_create(skai_logo_area);
    // lv_img_set_src(skai_logo, &img_skai);
    // lv_img_set_zoom(skai_logo, 256 * 0.2); // Scale from 160x160 to 60x60
    // lv_obj_align(skai_logo, LV_ALIGN_CENTER, 0, 0);

    // Connected device name label at top
    connected_device_label = lv_label_create(bg);
    lv_label_set_text(connected_device_label, "");
    lv_obj_set_size(connected_device_label, 150, 44);
    lv_obj_set_style_text_color(connected_device_label, lv_color_hex(0xAAAAAA),
                                0);
    lv_obj_set_style_text_align(connected_device_label, LV_TEXT_ALIGN_CENTER,
                                0);
    lv_label_set_long_mode(connected_device_label, LV_LABEL_LONG_DOT);
    lv_obj_align(connected_device_label, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_clear_flag(connected_device_label, LV_OBJ_FLAG_CLICKABLE);
    // Initialize with current active device name
    {
        const bonded_devices_db_t *db = ble_dev_mgr_get_database();
        int active_idx = ble_dev_mgr_get_active_device();
        if (db && active_idx >= 0 && active_idx < MAX_BONDED_DEVICES &&
            db->devices[active_idx].is_valid)
        {
            lv_label_set_text(connected_device_label,
                              db->devices[active_idx].device_name);
        }
    }

    // FSR-402 ADC real-time display label
    // fsr_adc_label = lv_label_create(bg);
    // lv_label_set_text(fsr_adc_label, "FSR: --");
    // lv_obj_set_style_text_color(fsr_adc_label, lv_color_hex(0x00FF88), 0);
    // lv_obj_set_style_text_font(fsr_adc_label,
    //                            LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    // lv_obj_align(fsr_adc_label, LV_ALIGN_TOP_MID, 0, 50);
    // lv_obj_clear_flag(fsr_adc_label, LV_OBJ_FLAG_CLICKABLE);

    // Init ADC and start periodic reading
    fsr_adc_init();
    if (!fsr_adc_timer)
    {
        fsr_adc_timer =
            rt_timer_create("fsr_adc", fsr_adc_timer_cb, NULL, FSR_ADC_READ_MS,
                            RT_TIMER_FLAG_PERIODIC);
    }
    else
    {
        rt_timer_stop(fsr_adc_timer);
    }
    rt_timer_start(fsr_adc_timer);

    // Unified input bar: starts as small indicator at bottom, animates to
    // input display above keyboard
    text_input_bar_bg = lv_obj_create(bg);
    lv_obj_set_size(text_input_bar_bg, 200, 50);
    lv_obj_set_pos(text_input_bar_bg, (LV_HOR_RES_MAX - 200) / 2,
                   LV_VER_RES_MAX - 50);
    lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_border_width(text_input_bar_bg, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(text_input_bar_bg, 22, LV_PART_MAIN);
    lv_obj_set_style_pad_all(text_input_bar_bg, 5, LV_PART_MAIN);
    lv_obj_clear_flag(text_input_bar_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(text_input_bar_bg, text_input_bar_cb, LV_EVENT_ALL,
                        NULL);

    // Indicator line (visible in closed state)
    text_input_bar = lv_obj_create(text_input_bar_bg);
    lv_obj_set_size(text_input_bar, 130, 25);
    lv_obj_set_style_bg_color(text_input_bar, lv_color_hex(0x1a1a1a),
                              LV_PART_MAIN);
    lv_obj_set_style_border_width(text_input_bar, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(text_input_bar, 50, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(text_input_bar, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_border_color(text_input_bar, lv_color_hex(0xFFFFFF),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(text_input_bar, 2, LV_PART_MAIN);
    lv_obj_set_style_border_opa(text_input_bar, LV_OPA_50, LV_PART_MAIN);
    lv_obj_align(text_input_bar, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_clear_flag(text_input_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(text_input_bar, text_input_bar_cb, LV_EVENT_ALL, NULL);

    // Input content container (hidden in closed state)
    input_content_container = lv_obj_create(text_input_bar_bg);
    lv_obj_set_size(input_content_container, 290, 40);
    lv_obj_center(input_content_container);
    lv_obj_set_style_bg_opa(input_content_container, LV_OPA_TRANSP,
                            LV_PART_MAIN);
    lv_obj_set_style_border_width(input_content_container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(input_content_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(input_content_container, LV_OBJ_FLAG_HIDDEN);

    // Input text label
    input_display_label = lv_label_create(input_content_container);
    lv_label_set_text(input_display_label, "");
    lv_obj_set_style_text_color(input_display_label, lv_color_hex(0xFFFFFF),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(input_display_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)),
                               LV_PART_MAIN);
    lv_obj_set_width(input_display_label, LV_SIZE_CONTENT);
    lv_label_set_long_mode(input_display_label, LV_LABEL_LONG_CLIP);
    lv_obj_align(input_display_label, LV_ALIGN_LEFT_MID, 10, 0);

    // Blinking cursor
    input_cursor = lv_obj_create(text_input_bar_bg);
    lv_obj_set_size(input_cursor, 2, 25);
    lv_obj_set_style_bg_color(input_cursor, lv_color_hex(0x4a90e2),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(input_cursor, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(input_cursor, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(input_cursor, 1, LV_PART_MAIN);
    lv_obj_align_to(input_cursor, input_display_label, LV_ALIGN_OUT_RIGHT_MID,
                    2, 0);
    lv_obj_add_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);

    // Enter button (hidden initially, shows next to bar when keyboard opens)
    input_enter_btn = lv_obj_create(bg);
    lv_obj_set_size(input_enter_btn, 50, 45);
    lv_obj_set_pos(input_enter_btn, (LV_HOR_RES_MAX - 50) / 2 + 165,
                   LV_VER_RES_MAX - 305 - 45);
    lv_obj_set_style_bg_color(input_enter_btn, lv_color_hex(0x4a90e2),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(input_enter_btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(input_enter_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(input_enter_btn, 22, LV_PART_MAIN);
    lv_obj_clear_flag(input_enter_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(input_enter_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(input_enter_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(input_enter_btn, input_enter_btn_event_cb,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t *enter_img = lv_img_create(input_enter_btn);
    lv_img_set_src(enter_img, &enter_icon);
    lv_obj_center(enter_img);

    #if ENABLE_MENU_FEATURE
    menu_window(bg);

    /* 螢幕頂部透明觸控區域，按下後顯示 tileview 讓使用者往下滑開啟 menu */
    menu_swipe_area = lv_obj_create(bg);
    lv_obj_set_size(menu_swipe_area, LV_HOR_RES_MAX, LV_VER_RES_MAX / 8);
    lv_obj_align(menu_swipe_area, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(menu_swipe_area, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_border_width(menu_swipe_area, 0, LV_PART_MAIN);
    lv_obj_clear_flag(menu_swipe_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(menu_swipe_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(menu_swipe_area, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(menu_swipe_area, menu_swipe_area_event_cb, LV_EVENT_ALL,
                        NULL);
    #endif

    // Create custom circular keyboard
    create_circular_keyboard_layout(bg);
}

void refersh_mouse_status_bar_time(void)
{
    T_UTC_TIME current_time = SkaiWatchSys.Global_Time;
    // Just call update_time_display to refresh with current colon state
    update_time_display();
}

/*********************
 *  PUBLIC FUNCTIONS
 *********************/

/**
 * @brief Gets the handfree mode state
 * @return true if handfree mode is enabled, false otherwise
 */
bool get_hid_mouse_handfree_mode(void)
{
    return handfree;
}

/**
 * @brief Checks if mouse movement is locked
 * @return true if movement is locked, false otherwise
 */
bool app_hid_mouse_movement_lock(void)
{
    return (rt_tick_get() - press_time) < PRESSED_TIME_MS;
}

/**
 * @brief Handles application start
 * @param scr Screen object
 */
static void on_start(lv_obj_t *scr)
{
    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    lv_create_mouse_screen(scr);
    app_control_set_mouse_mode(true);

    extern void set_status_bar_area_up_state(bool state);
    extern void set_status_bar_area_down_state(bool state);
    extern void set_status_bar_area_left_state(bool state);

    set_status_bar_area_up_state(false);
    set_status_bar_area_down_state(false);
    set_status_bar_area_left_state(false);
    display_gesture_detect_objs(0, false);
    RT_ASSERT(control_provider.trigger_finger_event);
    RT_ASSERT(control_provider.ble_hid_consumer_back);
    // ble_app_advertising_start(SkaiWatchSys.gap_conn_state ==
    // GAP_CONN_STATE_DISCONNECTED, true, false);
}

/**
 * @brief Handles application resume
 */
static void on_resume(void)
{
    reset_lvgl_msg_handler();
    if (handfree)
    {
        extern void switch_watch_motion_control_mode(bool enable,
                                                     bool animation);
        switch_watch_motion_control_mode(true, false);
    }

    setting_provider.set_power_save_mode(0);
}

/**
 * @brief Handles application pause
 */
static void on_pause(void)
{
    setting_provider.set_power_save_mode(1);
}

/**
 * @brief Handles application stop
 */
static void on_stop(void)
{
    app_control_set_mouse_mode(false);

    // Clean up menu tileview
    menu_tileview = NULL;
    menu_home_tile = NULL;
    menu_content_tile = NULL;
    menu_bg = NULL;
    menu_swipe_area = NULL;
    left_scroll_bar = NULL;

    // Clean up file list
    file_list = NULL;
    file_items_count = 0;

    // Clean up inertia timer
    if (inertia_timer != NULL)
    {
        lv_timer_del(inertia_timer);
        inertia_timer = NULL;
    }
    inertia_velocity = 0.0f;

    // Cancel animations before cleanup
    if (text_input_bar_bg != NULL)
        lv_anim_del(text_input_bar_bg, NULL);
    if (keyboard_container != NULL)
        lv_anim_del(keyboard_container, NULL);

    // Clean up keyboard resources
    if (keyboard != NULL)
    {
        keyboard = NULL;
    }
    if (custom_keyboard != NULL)
    {
        custom_keyboard = NULL;
    }
    if (keyboard_container != NULL)
    {
        keyboard_container = NULL;
    }
    keyboard_visible = false;

    // Clean up input display (now part of text_input_bar_bg)
    input_content_container = NULL;
    input_display_label = NULL;
    input_cursor = NULL;
    input_enter_btn = NULL;
    if (cursor_blink_timer != NULL)
    {
        lv_timer_del(cursor_blink_timer);
        cursor_blink_timer = NULL;
    }
    clear_input_display();

    // Clean up FSR ADC
    if (fsr_adc_timer != NULL)
    {
        rt_timer_stop(fsr_adc_timer);
        fsr_adc_timer = NULL;
    }
    fsr_adc_deinit();
    // fsr_adc_label = NULL;

    // Clean up crosshair lines
    crosshair_line1 = NULL;
    crosshair_line2 = NULL;

    extern void set_status_bar_area_up_state(bool state);
    extern void set_status_bar_area_down_state(bool state);
    extern void set_status_bar_area_left_state(bool state);

    set_status_bar_area_up_state(true);
    set_status_bar_area_down_state(true);
    set_status_bar_area_left_state(true);
    // ble_app_advertising_start(SkaiWatchSys.gap_conn_state ==
    // GAP_CONN_STATE_DISCONNECTED, false, false);
}

/**
 * @brief Resumes the watch system mouse
 */
void watch_system_mouse_resume(void)
{
    app_control_set_mouse_mode(true);
    // if (handfree)
    // {
    extern void switch_watch_motion_control_mode(bool enable, bool animation);
    switch_watch_motion_control_mode(true, false);
    // }

    setting_provider.set_power_save_mode(0);
    watch_sys_sync.notify_calibration_global_attitude();
}

/**
 * @brief Pauses the watch system mouse
 */
void watch_system_mouse_pause(void)
{
    app_control_set_mouse_mode(false);
}

/**
 * @brief Clear all files from the mouse file list
 */
void hid_mouse_clear_files(void)
{
    file_items_count = 0;

    // Clear visual list if it exists
    if (file_list)
    {
        // Remove all file item children (keep handheld and calibrate which
        // are the first two)
        lv_obj_t *child =
            lv_obj_get_child(file_list, 2); // Start from third child
        while (child)
        {
            lv_obj_t *next_child = lv_obj_get_child(file_list, 2);
            lv_obj_del(child);
            child = next_child;
        }
    }
}

/**
 * @brief Message handler for the application
 * @param msg Message type
 * @param param Message parameter
 */
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        lv_obj_t *scr = lv_scr_act();
        on_start(scr);
        break;
    }
    case GUI_APP_MSG_ONRESUME:
        watch_system_mouse_resume();
        break;
    case GUI_APP_MSG_ONPAUSE:
        watch_system_mouse_pause();
        break;
    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

/**
 * @brief Main application function
 * @param i Intent parameter
 * @return 0 on success
 */
static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_MOUSE, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(mouse), LV_EXT_IMG_GET(img_mouse),
                   APP_ID_MOUSE, app_main);

#endif /* APP_ID_MOUSE */

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/