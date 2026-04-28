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
    #define SCROLLING_THRESHOLD 5
    #define EDGE_THRESHOLD_PIXELS 20
    #define BOTTOM_EDGE_THRESHOLD 50
    #define GESTURE_DISTANCE_THRESHOLD 50
    #define GESTURE_TIMER_MS 200
    #define INERTIA_TIMER_MS 20
    #define INERTIA_DECAY_FACTOR 0.95f
    #define MIN_SCROLL_SPEED 1.0f
    #define SCROLL_SPEED_MULTIPLIER 2.0f

    #define SIMULATE_MOUSE_RIGHT_BUTTON 0
    #define USING_MOUSE_WHEEL_SCROLLING 1
    #define USING_TOUCHSCREEN_SCROLLING 0
    #define USING_EDGE_BOTTOM_DETECTION 0
    #define USING_EDGE_LEFT_DETECTION 0
    #define USING_EDGE_RIGHT_DETECTION 0
    // 除錯顯示：把弧形觸發區與中間態區段以折線標出邊界
    #define SHOW_SCROLL_ZONE_DEBUG 0

    #define ENABLE_MENU_FEATURE 0
    #define KB_ANIM_TIME_MS 100

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
LV_IMG_DECLARE(img_left_arrow);
LV_IMG_DECLARE(Map_fill);
LV_IMG_DECLARE(switch_icon);

static lv_point_t touchscreen_point;
static lv_point_t start_point;
static lv_point_t last_point;
static lv_point_t bottom_bar_start_point;
static lv_point_t bottom_bar_last_point;
static unsigned int press_time = 0;
static bool user_touching = false;
static bool scrolling = false;
static bool scrolling_confirmed = false;
static bool moving = false;

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
static bool left_scroll_active = false;

// 左側滾動區（弧形 + 中間態）等待方向判定的 pending 狀態：
// 上下→升級為 left_scroll_active 角度滾動；向右→送 mouse back；向左→走拖曳
static bool center_zone_pending = false;
// 已進入 back-hint 流程（向右拖曳中）：pressing 持續期更新 hint，release 判定觸發
static bool back_pending_active = false;
static int16_t back_hint_drag_offset = 0;
// 按下時是否落在弧形區：弧形按下整個 session 都跳過 BLE_HID_Mouse_Touch_*
// 避免 deferred click（按弧形不該觸發點擊）
static bool press_in_arc_zone = false;

// === 向右返回 hint（仿 lvsf_gesture.c 的返回視覺/動畫流程）===
    #define BACK_HINT_LIMIT 80
    #define BACK_HINT_SIZE 70
static lv_obj_t *back_hint_obj = NULL;
static lv_obj_t *back_hint_icon = NULL;
static lv_anim_t back_hint_anim;
static lv_anim_t back_hint_release_anim;
static bool back_hint_hidden = true;
static bool back_hint_vibrated = false;
static bool back_hint_anim_is_running = false;
static void back_hint_anim_cb(void *obj, uint16_t x);
static void hidden_back_hint(bool hide);
static void hidden_back_hint_release_anim_cb(lv_anim_t *a);
static void hidden_back_hint_cb(lv_anim_t *a);
static void set_back_hint_obj_opa(void *obj, uint8_t opa);

// === 從下往上拉觸發 multitask 的 hint（同套 hint 流程，方向換成 BOTTOM_MID）===
    #define MULTITASK_HINT_LIMIT 80
    #define MULTITASK_HINT_SIZE 70
static bool multitask_pending_active = false;
static int16_t multitask_hint_drag_offset = 0;
static lv_obj_t *multitask_hint_obj = NULL;
static lv_obj_t *multitask_hint_icon = NULL;
static lv_anim_t multitask_hint_anim;
static lv_anim_t multitask_hint_release_anim;
static bool multitask_hint_hidden = true;
static bool multitask_hint_vibrated = false;
static bool multitask_hint_anim_is_running = false;
static void multitask_hint_anim_cb(void *obj, uint16_t x);
static void hidden_multitask_hint(bool hide);
static void hidden_multitask_hint_release_anim_cb(lv_anim_t *a);
static void hidden_multitask_hint_cb(lv_anim_t *a);
static void set_multitask_hint_obj_opa(void *obj, uint8_t opa);

    #if SHOW_SCROLL_ZONE_DEBUG
        // 折線近似圓弧的取樣點數（19 點 = 5° 間距，覆蓋 90° 弧）
        #define DBG_ZONE_ARC_POINTS 19
static lv_point_t dbg_zone_outer_pts[DBG_ZONE_ARC_POINTS];
static lv_point_t dbg_zone_inner_pts[DBG_ZONE_ARC_POINTS];
    #endif

    // 左側滾動節點：以手指繞螢幕中心的角度追蹤，可無限旋轉；
    // 節點沿弧線跟手移動，每通過一個節點觸發一次滾輪。
    #define LEFT_SCROLL_NODE_COUNT 5
    #define LEFT_SCROLL_ARC_MIN_DEG 135.0f
    #define LEFT_SCROLL_ARC_MAX_DEG 225.0f
    #define LEFT_SCROLL_ARC_SPAN_DEG 90.0f
    #define LEFT_SCROLL_NODE_SPACING_DEG                                       \
        (LEFT_SCROLL_ARC_SPAN_DEG / LEFT_SCROLL_NODE_COUNT) // ≈ 12.86°
    // 每觸發一個節點時，送出的 wheel 步數倍率（手感不夠強就調大）
    #define LEFT_SCROLL_STEP_MULTIPLIER 3
    // 中間滾動觸發區：弧形觸發區內緣再往內延伸的弧形圓環厚度（px）
    // 幾何上接在 is_point_in_left_arc 的內緣（dist = outer_r - 50）後面
    // 上下方向觸發後升級為左弧形角度滾動，跟左弧形共用同一套邏輯
    #define CENTER_SCROLL_ZONE_THICKNESS 50
    #define LEFT_SCROLL_NODE_MAX_SIZE 14
    #define LEFT_SCROLL_NODE_MIN_SIZE 3
    // 未觸碰時暗/細，觸碰時亮/粗，100ms 過渡
    #define LEFT_SCROLL_ARC_W_DIM 12
    #define LEFT_SCROLL_ARC_W_ACTIVE 35
    #define LEFT_SCROLL_ARC_OPA_DIM LV_OPA_TRANSP
    #define LEFT_SCROLL_ARC_OPA_ACTIVE LV_OPA_TRANSP
    #define LEFT_SCROLL_NODE_OPA_DIM LV_OPA_70
    #define LEFT_SCROLL_NODE_OPA_ACTIVE LV_OPA_COVER
    #define LEFT_SCROLL_UI_ANIM_MS 300
    #ifndef LEFT_SCROLL_PI
        #define LEFT_SCROLL_PI 3.14159265358979323846f
    #endif
static lv_obj_t *left_scroll_nodes[LEFT_SCROLL_NODE_COUNT] = {NULL};
static lv_point_t left_scroll_node_pts[LEFT_SCROLL_NODE_COUNT][2];
static float scroll_last_theta = 0.0f;      // 上次手指相對中心的角度（弧度）
static float scroll_accum_angle = 0.0f;     // 未觸發滾動的累積角度（弧度）
static float scroll_node_offset_deg = 0.0f; // 節點視覺偏移（度，已正規化）
static int32_t scroll_ui_level = 0;         // 0 = 暗/細，1000 = 亮/粗（動畫用）

// 左側滾動節點相關（實作在 mouse screen 建立處）
static float left_scroll_finger_theta(const lv_point_t *p);
static float left_scroll_normalize_delta(float d);
static void update_left_scroll_nodes(void);
static void apply_scroll_ui_level(void);
static void animate_scroll_ui_to(bool active);
static void scroll_node_snap_anim_cb(void *var, int32_t v);
static void snap_scroll_nodes(void);

// === HID 模式切換（上方 label 左右拖動切換）===
typedef enum
{
    HID_MODE_TRACKPAD = 0,
    HID_MODE_KEYBOARD,
    HID_MODE_COUNT
} hid_mode_t;
static hid_mode_t current_hid_mode = HID_MODE_TRACKPAD;
static const char *const hid_mode_names[HID_MODE_COUNT] = {
    "Trackpad",
    "Keyboard",
};
// label 顯示「按下去會切到的下一個 mode」名稱，不是當前 mode
static inline const char *next_mode_name(hid_mode_t mode)
{
    return hid_mode_names[((int)mode + 1) % HID_MODE_COUNT];
}
    #define MODE_SWIPE_COMMIT_THRESHOLD 60
    #define MODE_SWIPE_ANIM_TIME_MS 200
static bool mode_swipe_active = false;
static int16_t mode_swipe_start_x = 0;
static hid_mode_t mode_swipe_target = HID_MODE_TRACKPAD;
static int8_t mode_swipe_target_side = -1;
static lv_anim_t mode_swipe_anim;     // 留著編譯通過（dispose 內還有 ref）
static int32_t mode_swipe_anim_value; // 同上
// 自製 timer-based 動畫（避開 LVGL anim 的 race）
static lv_timer_t *mode_swipe_timer = NULL;
static int32_t mode_swipe_from_dx = 0;
static int32_t mode_swipe_to_dx = 0;
static uint32_t mode_swipe_start_tick = 0;
static bool mode_swipe_is_commit = false;
// 每個 mode 用一個 480×480 透明容器包，切換時整組移動
static lv_obj_t *mode_container[HID_MODE_COUNT] = {NULL};
static void apply_hid_mode(hid_mode_t mode);
static void mode_label_event_cb(lv_event_t *e);

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
    float inner_r = outer_r - 50.0f; // 觸發範圍比 UI（30px）更寬
    if (dist < inner_r || dist > outer_r)
        return false;
    // 只接受左側（x < 中心）
    if (dx >= 0)
        return false;
    // 角度放寬到約 125°~235°（sin(55°) ≈ 0.819）
    float max_dy = dist * 0.819f;
    return (dy >= -max_dy && dy <= max_dy);
}

// 判斷觸碰點是否在中間滾動觸發區
// 幾何：弧形觸發區（dist = outer_r-50 ~ outer_r）的內緣，再往內延 THICKNESS px，
// 角度範圍跟 is_point_in_left_arc 相同（左半 ±55°），兩塊接成完整的左側弧帶
static bool is_point_in_center_scroll_zone(const lv_point_t *p)
{
    float cx = LV_HOR_RES_MAX / 2.0f;
    float cy = LV_VER_RES_MAX / 2.0f;
    float dx = p->x - cx;
    float dy = p->y - cy;
    float dist = sqrtf(dx * dx + dy * dy);
    float outer_r = cx;
    float zone_outer = outer_r - 50.0f; // 接弧形觸發區的內緣
    float zone_inner = zone_outer - (float)CENTER_SCROLL_ZONE_THICKNESS;
    if (dist < zone_inner || dist > zone_outer)
        return false;
    if (dx >= 0)
        return false;
    float max_dy = dist * 0.819f;
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
// 跨 mode 的下方拖動 hit area（trackpad mode 觸發 multitask hint）
// 因為 text_input_bar_bg 已被 reparent 到 mode_container[KEYBOARD]，
// trackpad mode 下方需要獨立 hit area
static lv_obj_t *bottom_swipe_area = NULL;
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

    // indicator bar 永遠隱藏（不再顯示），鍵盤關閉時只隱藏輸入內容
    // lv_obj_clear_flag(text_input_bar, LV_OBJ_FLAG_HIDDEN);
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
        // set_stop_mouse_move(false);
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
        // lv_anim_set_path_cb(&a, lv_anim_path_overshoot);
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
        // set_stop_mouse_move(true);
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

    // close_btn 已移除（鍵盤關閉由 mode 切換取代）

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
    scrolling = false;
    moving = false;

    // 左側滾動範圍（弧形 + 中間態）統一走方向判定：
    //   上下→升級為角度滾動（left_scroll_active）
    //   向右→觸發 ble_hid_mouse_back（瀏覽器後退鍵）
    //   向左→退出 pending，走原本拖曳/滑鼠移動
    left_scroll_active = false;
    center_zone_pending = false;
    back_pending_active = false;
    back_hint_drag_offset = 0;
    bool in_arc = is_point_in_left_arc(&start_point);
    bool in_center = is_point_in_center_scroll_zone(&start_point);
    press_in_arc_zone = in_arc;
    if (in_arc || in_center)
    {
        center_zone_pending = true;
        // 取消可能還在跑的 snap 動畫，避免按下後 offset 被動畫繼續覆蓋
        lv_anim_del(&scroll_node_offset_deg, scroll_node_snap_anim_cb);
        animate_scroll_ui_to(true); // 觸碰立刻給視覺回饋（亮起）
        LOG_D("scroll zone pending (arc=%d center=%d)", in_arc, in_center);
    }

    // 重置滚动方向锁定
    scroll_direction_locked = false;
    is_horizontal_scroll = false;

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

    // 弧形區整個 session 跳過 BLE_HID 點擊偵測（避免按弧形誤觸 click）
    // 中間態與其他區域仍走 BLE_HID 偵測（沒移動放開時可正常觸發點擊）
    if (!press_in_arc_zone)
    {
        BLE_HID_Mouse_Touch_Press((uint16_t)start_point.x,
                                  (uint16_t)start_point.y);
    }
}

/**
 * @brief Handles the pressing event
 * @param indev Input device
 * @param current_point Current touch point
 */
static void handle_pressing_event(lv_indev_t *indev,
                                  const lv_point_t *current_point)
{
    if (!press_in_arc_zone && !left_scroll_active)
    {
        BLE_HID_Mouse_Touch_Move((uint16_t)current_point->x,
                                 (uint16_t)current_point->y);
    }

    // 在 back-hint 流程中：持續更新 hint 寬度，不再做其他處理
    if (back_pending_active)
    {
        int16_t dx = current_point->x - start_point.x;
        if (dx < 0) dx = 0;
        back_hint_drag_offset = dx;
        // drag 接近門檻且 hint 還隱藏 → 啟動 50ms 進場動畫（從 0 → LIMIT 寬度）
        if (dx > BACK_HINT_LIMIT - 10 && back_hint_hidden)
        {
            back_hint_anim_is_running = true;
            hidden_back_hint(false);
            lv_anim_init(&back_hint_anim);
            lv_anim_set_var(&back_hint_anim, back_hint_obj);
            lv_anim_set_time(&back_hint_anim, 50);
            lv_anim_set_values(&back_hint_anim, 0, BACK_HINT_LIMIT);
            lv_anim_set_exec_cb(&back_hint_anim,
                                (lv_anim_exec_xcb_t)back_hint_anim_cb);
            lv_anim_set_ready_cb(&back_hint_anim, hidden_back_hint_cb);
            lv_anim_start(&back_hint_anim);
        }
        // hint 已顯示且進場動畫沒在跑 → 直接跟手更新寬度
        if (!back_hint_hidden && !back_hint_anim_is_running)
        {
            back_hint_anim_cb(back_hint_obj, (uint16_t)dx);
        }
        return;
    }

    // 左側滾動弧線模式：以角度追蹤手指繞中心的旋轉量，
    // 可連續旋轉多圈；節點跟手在弧線上循環移動，每過一個節點滾動一次。
    if (left_scroll_active)
    {
        float theta = left_scroll_finger_theta(current_point);
        float delta = left_scroll_normalize_delta(theta - scroll_last_theta);
        scroll_last_theta = theta;

        // 手指幾乎沒動就不更新視覺、也不累積（避免 BLE/LVGL 做無謂工作）
        if (fabsf(delta) < 0.001f)
        {
            return;
        }

        // 更新節點視覺偏移，讓節點跟手旋轉
        scroll_node_offset_deg += delta * 180.0f / LEFT_SCROLL_PI;
        // 把偏移收斂到 [0, span)，避免長時間旋轉後浮點精度損失
        scroll_node_offset_deg =
            fmodf(scroll_node_offset_deg, LEFT_SCROLL_ARC_SPAN_DEG);
        update_left_scroll_nodes();

        // 累積角度，計算這次事件要觸發多少個節點單位
        scroll_accum_angle += delta;
        const float threshold_rad =
            LEFT_SCROLL_NODE_SPACING_DEG * LEFT_SCROLL_PI / 180.0f;
        int steps = 0;
        while (fabsf(scroll_accum_angle) >= threshold_rad)
        {
            int sign = (scroll_accum_angle > 0.0f) ? 1 : -1;
            steps += sign;
            scroll_accum_angle -= (float)sign * threshold_rad;
        }
        // 一次 move event 只送一筆 BLE 報告（合併本次所有節點單位），
        // 避免快速旋轉時把 BLE/motor queue 灌爆造成記憶體堆積
        if (steps != 0)
        {
            int scaled = steps * LEFT_SCROLL_STEP_MULTIPLIER;
            if (scaled > 127)
                scaled = 127;
            else if (scaled < -127)
                scaled = -127;
            int8_t scroll_val = (int8_t)scaled;
            if (SkaiWatchSys.phone_os_version == IOS)
            {
                scroll_val = -scroll_val;
            }
            motor_pattern_damping();
            control_provider.ble_hid_mouse_wheel_scroll(scroll_val);
        }
        return;
    }

    int16_t delta_x = current_point->x - last_point.x;
    int16_t delta_y = current_point->y - last_point.y;

    // 左側滾動區方向判定：上下→升級滾動；向右→mouse back；向左→走拖曳
    if (center_zone_pending)
    {
        int16_t dx_from_start = current_point->x - start_point.x;
        int16_t dy_from_start = current_point->y - start_point.y;
        if ((abs(dx_from_start) >= SCROLLING_THRESHOLD) ||
            (abs(dy_from_start) >= SCROLLING_THRESHOLD))
        {
            center_zone_pending = false;
            if (abs(dy_from_start) > abs(dx_from_start))
            {
                // 強制把 BLE_HID 內部 deferred-click state 推到 IDLE
                BLE_HID_Mouse_Touch_Move((uint16_t)(start_point.x + 100),
                                         (uint16_t)start_point.y);
                // 上下 → 升級為左弧形角度滾動（共用既有邏輯）
                left_scroll_active = true;
                scrolling = true;
                scroll_last_theta = left_scroll_finger_theta(current_point);
                scroll_accum_angle = 0.0f;
                motor_pattern_damping();
                LOG_D("scroll zone -> wheel scroll");
                return;
            }
            else if (dx_from_start > 0)
            {
                // 向右 → 進入 back-hint 流程（hint 動畫追蹤拖曳，release 時才觸發）
                BLE_HID_Mouse_Touch_Move((uint16_t)(start_point.x + 100),
                                         (uint16_t)start_point.y);
                scrolling = true;          // 避免放開時誤觸 click
                back_pending_active = true;
                back_hint_drag_offset = dx_from_start;
                back_hint_vibrated = false;
                animate_scroll_ui_to(false); // 弧形 UI 淡回暗
                LOG_D("scroll zone -> back hint");
                return;
            }
            // 向左 → 退出 pending，UI 淡回，讓底下拖曳邏輯接手送 mouse_move
            animate_scroll_ui_to(false);
        }
        else
        {
            // 還沒過閾值，繼續等
            return;
        }
    }

    if (handle_edge_scrolling(current_point, delta_x, delta_y))
    {
        return;
    }

    int16_t dx_from_start = current_point->x - start_point.x;
    int16_t dy_from_start = current_point->y - start_point.y;

    if (((abs(dx_from_start) >= SCROLLING_THRESHOLD) ||
        (abs(dy_from_start) >= SCROLLING_THRESHOLD)) && !scroll_direction_locked)
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
        last_point.x = current_point->x;
        last_point.y = current_point->y;
    }

    if (scrolling)
    {
        // 每次更新 last_point，讓滑鼠移動更順暢
        if (delta_x == 0 && delta_y == 0)
        {
            // 手指靜止（沒有拖曳）→ 恢復體感滑鼠
            // set_stop_mouse_move(false);
        }
        else
        {
            // 正在拖曳 → 控制滑鼠移動（距離翻倍），鎖住體感滑鼠
            // set_stop_mouse_move(true);
            // LOG_D("touch mouse move - delta_x: %d, delta_y: %d", delta_x,
            //       delta_y);
            // if (abs(delta_x) < 60 && abs(delta_y) < 60)
            {
                control_provider.ble_hid_mouse_move(delta_x * 1.5, delta_y * 1.5);
                last_point.x = current_point->x;
                last_point.y = current_point->y;
            }
        }

        return;
    }
}

/**
 * @brief Handles the released event
 * @param indev Input device
 */
static void handle_released_event(lv_indev_t *indev)
{
    user_touching = false;

    lv_point_t _release_pt;
    lv_indev_get_point(indev, &_release_pt);
    // 弧形整 session、或已升級的滾動模式都跳過 BLE_HID 釋放，避免 deferred click
    bool _long_press_ended = false;
    if (!press_in_arc_zone && !left_scroll_active)
    {
        _long_press_ended = BLE_HID_Mouse_Touch_Release(
            (uint16_t)_release_pt.x, (uint16_t)_release_pt.y);
    }

    // 左側滾動弧線放手
    if (left_scroll_active)
    {
        left_scroll_active = false;
        // set_stop_mouse_move(false);
        motor_pattern_stop();
        animate_scroll_ui_to(false); // 放開後淡回暗/細（100ms）
        snap_scroll_nodes();         // 節點 snap 回對齊位置（中央條置中）
        LOG_D("left scroll bar released");
        return;
    }

    // back-hint 流程放手：依拖曳距離判定觸發 back 或縮回（仿 lvsf_gesture）
    if (back_pending_active)
    {
        back_pending_active = false;
        back_hint_vibrated = false;
        if (back_hint_drag_offset > BACK_HINT_LIMIT)
        {
            // 過門檻 → 觸發 mouse back + 200ms opa 淡出
            if (control_provider.ble_hid_mouse_back)
            {
                control_provider.ble_hid_mouse_back();
            }
            lv_anim_init(&back_hint_release_anim);
            lv_anim_set_time(&back_hint_release_anim, 200);
            lv_anim_set_values(&back_hint_release_anim, LV_OPA_80,
                               LV_OPA_TRANSP);
            lv_anim_set_var(&back_hint_release_anim, back_hint_obj);
            lv_anim_set_exec_cb(&back_hint_release_anim,
                                (lv_anim_exec_xcb_t)set_back_hint_obj_opa);
            lv_anim_set_ready_cb(&back_hint_release_anim,
                                 hidden_back_hint_release_anim_cb);
            lv_anim_start(&back_hint_release_anim);
            LOG_D("back hint -> mouse back");
        }
        else
        {
            // 沒過門檻 → 100ms 寬度縮回 0
            lv_anim_init(&back_hint_release_anim);
            lv_anim_set_time(&back_hint_release_anim, 100);
            lv_anim_set_values(&back_hint_release_anim,
                               back_hint_drag_offset, 0);
            lv_anim_set_var(&back_hint_release_anim, back_hint_obj);
            lv_anim_set_exec_cb(&back_hint_release_anim,
                                (lv_anim_exec_xcb_t)back_hint_anim_cb);
            lv_anim_set_ready_cb(&back_hint_release_anim,
                                 hidden_back_hint_release_anim_cb);
            lv_anim_start(&back_hint_release_anim);
            LOG_D("back hint -> retract");
        }
        back_hint_drag_offset = 0;
        press_in_arc_zone = false;
        return;
    }

    // pending 沒解除就放手（沒移動超過閾值）：清旗標
    // - 弧形：滾動區不該 click，淡回 UI 並 early return
    // - 中間態：fall-through 到底下 click 流程（沒移動可以點擊）
    if (center_zone_pending)
    {
        center_zone_pending = false;
        if (press_in_arc_zone)
        {
            animate_scroll_ui_to(false);
            press_in_arc_zone = false;
            return;
        }
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

        // // 恢復體感滑鼠
        // set_stop_mouse_move(false);
    }
    else
    {
    #if SIMULATE_MOUSE_RIGHT_BUTTON
        if (pressed_left_half)
    #endif
        {
            if (!scrolling && !_long_press_ended &&
                (lv_tick_get() - press_time <= PRESSED_TIME_MS))
            {
                LOG_D("Air mouse - click_left");
                motor_pattern_tap();
                control_provider.ble_hid_mouse_left_click();
            }
        }
    #if SIMULATE_MOUSE_RIGHT_BUTTON
        else
        {
            if (!scrolling && !_long_press_ended &&
                (lv_tick_get() - press_time <= PRESSED_TIME_MS))
            {
                LOG_D("Air mouse - click_right");
                control_provider.ble_hid_mouse_right_click();
                motor_pattern_tap();
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
    {
        handle_pressed_event(indev);
        break;
    }
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

/**
 * @brief 刷新 mouse 畫面頂部顯示的當前控制裝置名稱
 *        放在 ENABLE_MENU_FEATURE guard 外面，讓 status bar
 * 的選單切換後也能更新
 */
void refresh_connected_device_label(void)
{
    // label 已改用作 mode 顯示，連接事件不再覆蓋；保留函式給 caller 不需改動
    if (!lv_obj_is_valid(connected_device_label))
        return;
    lv_label_set_text(connected_device_label, next_mode_name(current_hid_mode));
}

// static lv_obj_t *text_input_bar = NULL;
static rt_tick_t text_input_bar_pressing_time = NULL;
static rt_tick_t text_input_bar_press_time = NULL;
static uint16_t max_move_y = 0;
static uint8_t test_count = 0;
static float prev_elapsed = 0.0f;

    // 底部 bar 被上拉到某個視覺高度且停住後觸發多工鍵
    // 門檻看的是「bar 實際 UI 位置」，不是手指位移（bar 用 1:20 衰減）
    // bar 公式：lift = 10 + (move_y - 10) / 20；貼底時 lift=10
    // 例：lift=12 ≈ 手指 ~50px、lift=15 ≈ ~110px、lift=20 ≈ ~210px
    #define BOTTOM_BAR_MULTITASK_BAR_LIFT 12 // bar 升到離底部 >= 12px 才可觸發
    #define BOTTOM_BAR_MULTITASK_STILL_EPSILON 10 // 停住容忍範圍（px）
    #define BOTTOM_BAR_MULTITASK_HOLD_MS 100      // 停住多久觸發
static bool bottom_bar_multitask_ready = false;   // 已達成往上位移條件
static bool bottom_bar_multitask_fired = false;   // 這次 gesture 已觸發過
static lv_point_t bottom_bar_multitask_anchor;
// 用 rt_timer 而非 lv_timer：拖曳時 BLE queue 會把 LVGL task 塞到 lv_timer
// 延遲數百 ms 才 fire；rt_timer 跑在獨立 timer thread，不受 LVGL 佔用影響
static rt_timer_t bottom_bar_multitask_timer = NULL;

static void bottom_bar_multitask_fire(void)
{
    LOG_D("Bottom bar: multitask triggered");
    if (bottom_bar_multitask_fired)
        return;
    bottom_bar_multitask_fired = true;
    if (control_provider.ble_hid_keyboard_multitask != NULL)
    {
        control_provider.ble_hid_keyboard_multitask(true);
    }
    motor_pattern_tap();
}

static void bottom_bar_multitask_rt_timer_cb(void *parameter)
{
    // Soft timer thread 被 BLE/其他高優 thread 卡住時，本 callback 可能比
    // 預期晚很多才排到執行。RELEASED/再次 PRESSED 會把 ready 設成 false，
    // 這裡先 double-check 才真的送 HID，避免放手後「事後被戳」。
    if (!bottom_bar_multitask_ready || bottom_bar_multitask_fired)
        return;
    bottom_bar_multitask_fire();
}

// 手指每次位移超過 epsilon（或剛 armed）就呼叫，重新倒數 HOLD_MS
static void bottom_bar_multitask_restart_timer(void)
{
    if (bottom_bar_multitask_timer == NULL)
    {
        bottom_bar_multitask_timer = rt_timer_create(
            "bb_mt", bottom_bar_multitask_rt_timer_cb, NULL,
            rt_tick_from_millisecond(BOTTOM_BAR_MULTITASK_HOLD_MS),
            RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (bottom_bar_multitask_timer != NULL)
    {
        rt_timer_start(bottom_bar_multitask_timer);
    }
}

static void bottom_bar_multitask_cancel_timer(void)
{
    if (bottom_bar_multitask_timer != NULL)
    {
        rt_timer_stop(bottom_bar_multitask_timer);
    }
}
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
        bottom_bar_multitask_ready = false;
        bottom_bar_multitask_fired = false;
        bottom_bar_multitask_cancel_timer();
        // 重置 multitask hint 流程狀態
        multitask_pending_active = false;
        multitask_hint_drag_offset = 0;
        multitask_hint_vibrated = false;
        // notify_provider.holding_displacement(0, 0, 0);
        break;

    case LV_EVENT_PRESSING:
    {
        lv_point_t bottom_bar_now_point;
        lv_indev_get_point(indev, &bottom_bar_now_point);
        int dx = bottom_bar_now_point.x - bottom_bar_start_point.x;
        int dy = bottom_bar_now_point.y - bottom_bar_start_point.y;
        uint16_t move_y = abs(dy);
        if (move_y > max_move_y)
            max_move_y = move_y;

        // 向上拖（dy < 0）→ 進入 multitask hint 流程，更新 hint 寬度
        // 不再做 bar 視覺位移 / scale 縮放
        if (dy < 0)
        {
            if (!multitask_pending_active && move_y >= 5)
            {
                multitask_pending_active = true;
            }
            if (multitask_pending_active)
            {
                int up_amount = -dy;
                if (up_amount < 0) up_amount = 0;
                multitask_hint_drag_offset = (int16_t)up_amount;

                // drag 接近門檻且 hint 還隱藏 → 啟動 50ms 進場動畫
                if (up_amount > MULTITASK_HINT_LIMIT - 10 && multitask_hint_hidden)
                {
                    multitask_hint_anim_is_running = true;
                    hidden_multitask_hint(false);
                    lv_anim_init(&multitask_hint_anim);
                    lv_anim_set_var(&multitask_hint_anim, multitask_hint_obj);
                    lv_anim_set_time(&multitask_hint_anim, 50);
                    lv_anim_set_values(&multitask_hint_anim, 0,
                                       MULTITASK_HINT_LIMIT);
                    lv_anim_set_exec_cb(
                        &multitask_hint_anim,
                        (lv_anim_exec_xcb_t)multitask_hint_anim_cb);
                    lv_anim_set_ready_cb(&multitask_hint_anim,
                                         hidden_multitask_hint_cb);
                    lv_anim_start(&multitask_hint_anim);
                }
                if (!multitask_hint_hidden && !multitask_hint_anim_is_running)
                {
                    multitask_hint_anim_cb(multitask_hint_obj,
                                           (uint16_t)up_amount);
                }
            }
        }

        if (is_bottom_bar_gesture_active)
            return;
    #if USING_EDGE_BOTTOM_DETECTION
        if (move_y > 10 && !bottom_bar_gesture_timer_enabled && move_y > 150)
        {
            bottom_bar_gesture_timer_enabled = true;
            start_multiple_pages_timer();
        }
    #endif
        break;
    }

    case LV_EVENT_RELEASED:
        {
            lv_point_t bottom_bar_now_point;
            lv_indev_get_point(indev, &bottom_bar_now_point);
            uint16_t move_y =
                abs(bottom_bar_now_point.y - bottom_bar_start_point.y);
            LOG_D("Gesture detected: bottom bar released, move_y: %d", move_y);

            // multitask hint 流程：依 drag 是否過 LIMIT 觸發或縮回
            if (multitask_pending_active)
            {
                multitask_pending_active = false;
                multitask_hint_vibrated = false;
                if (multitask_hint_drag_offset > MULTITASK_HINT_LIMIT)
                {
                    if (control_provider.ble_hid_keyboard_multitask)
                    {
                        control_provider.ble_hid_keyboard_multitask(true);
                    }
                    lv_anim_init(&multitask_hint_release_anim);
                    lv_anim_set_time(&multitask_hint_release_anim, 200);
                    lv_anim_set_values(&multitask_hint_release_anim,
                                       LV_OPA_80, LV_OPA_TRANSP);
                    lv_anim_set_var(&multitask_hint_release_anim,
                                    multitask_hint_obj);
                    lv_anim_set_exec_cb(
                        &multitask_hint_release_anim,
                        (lv_anim_exec_xcb_t)set_multitask_hint_obj_opa);
                    lv_anim_set_ready_cb(
                        &multitask_hint_release_anim,
                        hidden_multitask_hint_release_anim_cb);
                    lv_anim_start(&multitask_hint_release_anim);
                    LOG_D("multitask hint -> fire");
                }
                else
                {
                    lv_anim_init(&multitask_hint_release_anim);
                    lv_anim_set_time(&multitask_hint_release_anim, 100);
                    lv_anim_set_values(&multitask_hint_release_anim,
                                       multitask_hint_drag_offset, 0);
                    lv_anim_set_var(&multitask_hint_release_anim,
                                    multitask_hint_obj);
                    lv_anim_set_exec_cb(
                        &multitask_hint_release_anim,
                        (lv_anim_exec_xcb_t)multitask_hint_anim_cb);
                    lv_anim_set_ready_cb(
                        &multitask_hint_release_anim,
                        hidden_multitask_hint_release_anim_cb);
                    lv_anim_start(&multitask_hint_release_anim);
                    LOG_D("multitask hint -> retract");
                }
                multitask_hint_drag_offset = 0;
                // hint 流程觸發了，不要再走 short press toggle_keyboard
                is_bottom_bar_gesture_active = false;
                bottom_bar_multitask_cancel_timer();
                break;
            }

            if (max_move_y < 60 && !bottom_bar_gesture_timer_enabled &&
                !is_bottom_bar_gesture_active)
            {
                LOG_D("Gesture detected: short press");
                // Toggle keyboard visibility when short press is detected
                toggle_keyboard_visibility();
            }
    #if USING_EDGE_BOTTOM_DETECTION
            if (bottom_bar_gesture_timer_enabled)
            {
                bottom_bar_gesture_timer_enabled = false;
                rt_timer_stop(multiple_pages_timer);
            }
    #endif

            is_bottom_bar_gesture_active = false;
            bottom_bar_multitask_ready = false;
            bottom_bar_multitask_fired = false;
            bottom_bar_multitask_cancel_timer();
        }
        break;

    case LV_EVENT_CLICKED:
        // Handle click event if needed
        break;

    default:
        break;
    }
}

    #ifndef USE_FSR_ADC
/**
 * @brief Initialize FSR-402 ADC device
 */
void fsr_adc_init(void)
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
void fsr_adc_deinit(void)
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
rt_uint32_t fsr_adc_read_value(void)
{
    if (fsr_adc_dev == NULL)
        return 0;
    return rt_adc_read((rt_adc_device_t)fsr_adc_dev, FSR_ADC_CHANNEL);
}
    #endif

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
    // LOG_D("FSR ADC raw value: %d (%.1fmV)", fsr_adc_value, fsr_adc_value
    // / 10.0f); duration = rt_tick_get() - duration; LOG_D("FSR ADC read and
    // process duration: %d ms", duration); if (fsr_adc_label != NULL &&
    // lv_obj_is_valid(fsr_adc_label))
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
 * @brief 計算手指相對螢幕中心的角度（atan2 慣例，LVGL 時鐘方向）
 */
static float left_scroll_finger_theta(const lv_point_t *p)
{
    const float cx = LV_HOR_RES_MAX / 2.0f;
    const float cy = LV_VER_RES_MAX / 2.0f;
    return atan2f((float)p->y - cy, (float)p->x - cx);
}

/**
 * @brief 把角度差正規化到 [-π, π]，處理跨越 ±π 的情況
 */
static float left_scroll_normalize_delta(float d)
{
    while (d > LEFT_SCROLL_PI)
        d -= 2.0f * LEFT_SCROLL_PI;
    while (d < -LEFT_SCROLL_PI)
        d += 2.0f * LEFT_SCROLL_PI;
    return d;
}

/**
 * @brief 根據目前的視覺偏移更新所有節點的位置與大小
 *        節點會跟手旋轉並在 150°~210° 弧線上循環，越靠近 180° 越大
 */
static void update_left_scroll_nodes(void)
{
    if (left_scroll_nodes[0] == NULL)
        return;

    const float cx = LV_HOR_RES_MAX / 2.0f;
    const float cy = LV_VER_RES_MAX / 2.0f;
    // arc 寬度固定，節點自身大小依 scroll_ui_level 變化
    const int32_t cur_arc_w = LEFT_SCROLL_ARC_W_ACTIVE;
    const float mid_r = cx - (float)cur_arc_w * 0.5f;
    const float span = LEFT_SCROLL_ARC_SPAN_DEG;
    const float spacing = LEFT_SCROLL_NODE_SPACING_DEG;
    // 節點尺寸固定為最大，不隨觸碰/放開變化（僅透明度會淡出）
    const float size_scale = 1.0f;

    // 節點透明度：在 DIM 與 ACTIVE 間依 scroll_ui_level 插值
    int32_t opa_v = (int32_t)LEFT_SCROLL_NODE_OPA_DIM +
                    ((int32_t)LEFT_SCROLL_NODE_OPA_ACTIVE -
                     (int32_t)LEFT_SCROLL_NODE_OPA_DIM) *
                        scroll_ui_level / 1000;
    if (opa_v < 0)
        opa_v = 0;
    if (opa_v > 255)
        opa_v = 255;
    const lv_opa_t node_opa = (lv_opa_t)opa_v;

    for (int i = 0; i < LEFT_SCROLL_NODE_COUNT; i++)
    {
        if (left_scroll_nodes[i] == NULL)
            continue;

        float base = LEFT_SCROLL_ARC_MIN_DEG + spacing * (float)i;
        float angle_deg = base + scroll_node_offset_deg;

        // 包裹到 [150°, 210°) 讓節點在可視弧線內循環
        angle_deg = fmodf(angle_deg - LEFT_SCROLL_ARC_MIN_DEG, span);
        if (angle_deg < 0.0f)
            angle_deg += span;
        angle_deg += LEFT_SCROLL_ARC_MIN_DEG;

        float rad = angle_deg * LEFT_SCROLL_PI / 180.0f;
        int16_t px = (int16_t)(cx + mid_r * cosf(rad));
        int16_t py = (int16_t)(cy + mid_r * sinf(rad));

        // 越靠近 180°（弧線正中）越大，兩端最小（cos² 平滑過渡）
        float t = (angle_deg - 180.0f) / (span * 0.5f); // [-1, 1)
        float factor = cosf(t * LEFT_SCROLL_PI * 0.5f);
        factor = factor * factor;
        // int16_t size = (int16_t)(((float)LEFT_SCROLL_NODE_MIN_SIZE +
        //                            (float)(LEFT_SCROLL_NODE_MAX_SIZE -
        //                                    LEFT_SCROLL_NODE_MIN_SIZE) *
        //                                factor) *
        //                           size_scale);
        // 線長度（徑向）與線寬隨 size_scale 變化
        float tick_len = 20.0f * size_scale;
        int16_t line_w = (int16_t)(6.0f * size_scale);
        if (tick_len < 2.0f) tick_len = 2.0f;
        if (line_w < 1) line_w = 1;

        // 徑向單位向量（從圓心指向節點）
        float ux = cosf(rad);
        float uy = sinf(rad);
        // 兩端點沿徑向：外端（靠邊）與內端（靠圓心）
        int16_t x0 = (int16_t)(px + ux * tick_len * 0.5f);
        int16_t y0 = (int16_t)(py + uy * tick_len * 0.5f);
        int16_t x1 = (int16_t)(px - ux * tick_len * 0.5f);
        int16_t y1 = (int16_t)(py - uy * tick_len * 0.5f);
        left_scroll_node_pts[i][0].x = x0;
        left_scroll_node_pts[i][0].y = y0;
        left_scroll_node_pts[i][1].x = x1;
        left_scroll_node_pts[i][1].y = y1;

        // 越靠近弧線中央（180°）節點越亮，兩端淡出
        // 不動 line_opa（避免 round-cap 殘色），改用 line_color 朝黑色背景插值
        // 效果：中央 0x4D 灰、邊緣接近 0x0F 幾乎融入黑底
        float blend = 0.2f + 0.8f * factor;
        if (blend < 0.0f) blend = 0.0f;
        if (blend > 1.0f) blend = 1.0f;
        uint8_t cv = (uint8_t)((float)0x4D * blend);
        lv_color_t node_color = lv_color_make(cv, cv, cv);

        lv_line_set_points(left_scroll_nodes[i], left_scroll_node_pts[i], 2);
        lv_obj_set_style_line_width(left_scroll_nodes[i], line_w, 0);
        lv_obj_set_style_line_color(left_scroll_nodes[i], node_color, 0);
        // lv_obj_set_style_line_opa(left_scroll_nodes[i], node_opa, 0);
    }
}

/**
 * @brief 依當前 scroll_ui_level（0 暗/細 ~ 1000 亮/粗）套用 arc 與節點樣式
 */
static void apply_scroll_ui_level(void)
{
    if (left_scroll_bar != NULL)
    {
        // arc 寬度固定，僅透明度仍隨觸碰動畫
        int32_t opa_v = (int32_t)LEFT_SCROLL_ARC_OPA_DIM +
                        ((int32_t)LEFT_SCROLL_ARC_OPA_ACTIVE -
                         (int32_t)LEFT_SCROLL_ARC_OPA_DIM) *
                            scroll_ui_level / 1000;
        if (opa_v < 0)
            opa_v = 0;
        if (opa_v > 255)
            opa_v = 255;
        lv_obj_set_style_arc_width(left_scroll_bar,
                                   LEFT_SCROLL_ARC_W_ACTIVE, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(left_scroll_bar, opa_v,
                                 LV_PART_MAIN);
    }
    update_left_scroll_nodes();
}

static void scroll_ui_anim_cb(void *var, int32_t v)
{
    (void)var;
    scroll_ui_level = v;
    apply_scroll_ui_level();
}

/**
 * @brief 啟動 100ms UI 過渡：active=true → 亮/粗，false → 暗/細
 */
static void animate_scroll_ui_to(bool active)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &scroll_ui_level);
    lv_anim_set_exec_cb(&a, scroll_ui_anim_cb);
    lv_anim_set_time(&a, LEFT_SCROLL_UI_ANIM_MS);
    lv_anim_set_values(&a, scroll_ui_level, active ? 1000 : 0);
    lv_anim_start(&a); // 會自動取代同 var+cb 的既有動畫
}

/**
 * @brief 節點對齊動畫的 exec callback：v 是 offset_deg × 100
 */
static void scroll_node_snap_anim_cb(void *var, int32_t v)
{
    (void)var;
    scroll_node_offset_deg = (float)v / 100.0f;
    update_left_scroll_nodes();
}

// === 返回 hint 動畫實作（仿 lvsf_gesture.c）===
static void back_hint_anim_cb(void *obj, uint16_t x)
{
    if (!lv_obj_is_valid(obj))
        return;
    if (x < BACK_HINT_LIMIT)
    {
        uint16_t obj_width = (uint16_t)((uint32_t)BACK_HINT_SIZE * x /
                                        BACK_HINT_LIMIT);
        lv_obj_set_width(obj, obj_width);
        lv_obj_set_style_radius(obj, 25, 0);
        lv_obj_align(obj, LV_ALIGN_LEFT_MID, 0, 0);
        if (back_hint_icon)
        {
            lv_obj_set_style_img_opa(back_hint_icon,
                                     obj_width > 30 ? LV_OPA_COVER
                                                    : LV_OPA_TRANSP,
                                     0);
        }
        if (back_hint_vibrated)
        {
            back_hint_vibrated = false; // 跌回門檻下，重置以便下次再震
        }
    }
    else
    {
        lv_obj_set_width(obj, BACK_HINT_SIZE);
        lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
        lv_obj_align(obj, LV_ALIGN_LEFT_MID, 0, 0);
        if (back_hint_icon)
            lv_obj_set_style_img_opa(back_hint_icon, LV_OPA_COVER, 0);
        if (!back_hint_vibrated)
        {
            motor_pattern_scrolling_app(); // 跨過門檻瞬間震動一次
            back_hint_vibrated = true;
        }
    }
}

static void hidden_back_hint(bool hide)
{
    back_hint_hidden = hide;
    if (!back_hint_obj)
        return;
    if (hide)
    {
        lv_obj_add_flag(back_hint_obj, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_set_style_bg_opa(back_hint_obj, LV_OPA_80, 0);
        lv_obj_clear_flag(back_hint_obj, LV_OBJ_FLAG_HIDDEN);
    }
}

static void hidden_back_hint_release_anim_cb(lv_anim_t *a)
{
    (void)a;
    hidden_back_hint(true);
    back_hint_anim_cb(back_hint_obj, 0);
}

static void hidden_back_hint_cb(lv_anim_t *a)
{
    (void)a;
    back_hint_anim_cb(back_hint_obj, BACK_HINT_LIMIT);
    back_hint_anim_is_running = false;
}

static void set_back_hint_obj_opa(void *obj, uint8_t opa)
{
    // 分別設背景與圖片 opa（這個 build 上整體 opa 不平滑）
    if (lv_obj_is_valid(obj))
        lv_obj_set_style_bg_opa(obj, opa, 0);
    if (back_hint_icon && lv_obj_is_valid(back_hint_icon))
        lv_obj_set_style_img_opa(back_hint_icon, opa, 0);
}

// === multitask hint 動畫實作（從下往上拉，圖示用 Map_fill）===
// 因 align 是 BOTTOM_MID，高度從 0 增長 = 從底邊往上長
static void multitask_hint_anim_cb(void *obj, uint16_t x)
{
    if (!lv_obj_is_valid(obj))
        return;
    if (x < MULTITASK_HINT_LIMIT)
    {
        uint16_t obj_height = (uint16_t)((uint32_t)MULTITASK_HINT_SIZE * x /
                                         MULTITASK_HINT_LIMIT);
        lv_obj_set_height(obj, obj_height);
        lv_obj_set_style_radius(obj, 25, 0);
        lv_obj_align(obj, LV_ALIGN_BOTTOM_MID, 0, 0);
        if (multitask_hint_icon)
        {
            lv_obj_set_style_img_opa(multitask_hint_icon,
                                     obj_height > 30 ? LV_OPA_COVER
                                                     : LV_OPA_TRANSP,
                                     0);
        }
        if (multitask_hint_vibrated)
        {
            multitask_hint_vibrated = false;
        }
    }
    else
    {
        lv_obj_set_height(obj, MULTITASK_HINT_SIZE);
        lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
        lv_obj_align(obj, LV_ALIGN_BOTTOM_MID, 0, 0);
        if (multitask_hint_icon)
            lv_obj_set_style_img_opa(multitask_hint_icon, LV_OPA_COVER, 0);
        if (!multitask_hint_vibrated)
        {
            motor_pattern_scrolling_app();
            multitask_hint_vibrated = true;
        }
    }
}

static void hidden_multitask_hint(bool hide)
{
    multitask_hint_hidden = hide;
    if (!multitask_hint_obj)
        return;
    if (hide)
    {
        lv_obj_add_flag(multitask_hint_obj, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_set_style_bg_opa(multitask_hint_obj, LV_OPA_80, 0);
        // icon img_opa 由 anim_cb 動態設定（依高度顯示與否）
        lv_obj_clear_flag(multitask_hint_obj, LV_OBJ_FLAG_HIDDEN);
    }
}

static void hidden_multitask_hint_release_anim_cb(lv_anim_t *a)
{
    (void)a;
    hidden_multitask_hint(true);
    multitask_hint_anim_cb(multitask_hint_obj, 0);
}

static void hidden_multitask_hint_cb(lv_anim_t *a)
{
    (void)a;
    multitask_hint_anim_cb(multitask_hint_obj, MULTITASK_HINT_LIMIT);
    multitask_hint_anim_is_running = false;
}

static void set_multitask_hint_obj_opa(void *obj, uint8_t opa)
{
    // 分別設背景與圖片 opa（這個 build 上整體 opa 不平滑）
    if (lv_obj_is_valid(obj))
        lv_obj_set_style_bg_opa(obj, opa, 0);
    if (multitask_hint_icon && lv_obj_is_valid(multitask_hint_icon))
        lv_obj_set_style_img_opa(multitask_hint_icon, opa, 0);
}

// === HID mode 切換：每個 mode 一個 480×480 透明容器，整組 translate_x ===

static void mode_set_translate_x(hid_mode_t mode, int16_t tx)
{
    // 直接改 obj 本體 x 位置（不走 transform system，避免 translate_x 的 race）
    if (mode_container[mode] && lv_obj_is_valid(mode_container[mode]))
        lv_obj_set_x(mode_container[mode], tx);
}

static void mode_set_visible(hid_mode_t mode, bool visible)
{
    if (mode_container[mode] && lv_obj_is_valid(mode_container[mode]))
    {
        if (visible)
            lv_obj_clear_flag(mode_container[mode], LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(mode_container[mode], LV_OBJ_FLAG_HIDDEN);
    }

    // bottom_swipe_area 只在 trackpad mode 接收觸控（觸發 multitask hint）
    // keyboard mode 下方是鍵盤，不該觸發 multitask
    if (mode == HID_MODE_KEYBOARD && bottom_swipe_area &&
        lv_obj_is_valid(bottom_swipe_area))
    {
        if (visible)
            lv_obj_add_flag(bottom_swipe_area, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(bottom_swipe_area, LV_OBJ_FLAG_HIDDEN);
    }

    // Keyboard mode 額外要 setup 鍵盤相關 UI（仿 toggle_keyboard_visibility 但無動畫）
    if (mode == HID_MODE_KEYBOARD)
    {
        if (visible)
        {
            if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
            {
                // 樣式：深色框
                lv_obj_set_style_bg_color(text_input_bar_bg,
                                          lv_color_hex(0x1a1a1a), LV_PART_MAIN);
                lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_90,
                                        LV_PART_MAIN);
                lv_obj_set_style_border_color(text_input_bar_bg,
                                              lv_color_hex(0xFFFFFF),
                                              LV_PART_MAIN);
                lv_obj_set_style_border_width(text_input_bar_bg, 2,
                                              LV_PART_MAIN);
                lv_obj_set_style_border_opa(text_input_bar_bg, LV_OPA_50,
                                            LV_PART_MAIN);
                // 位置與大小：移到鍵盤上方（toggle_keyboard 的 open 終點）
                int32_t open_x = (LV_HOR_RES_MAX - 310) / 2 - 35;
                int32_t open_y = LV_VER_RES_MAX - 305 - 45;
                lv_obj_set_pos(text_input_bar_bg, open_x, open_y);
                lv_obj_set_size(text_input_bar_bg, 310, 45);
            }
            if (input_content_container &&
                lv_obj_is_valid(input_content_container))
                lv_obj_clear_flag(input_content_container, LV_OBJ_FLAG_HIDDEN);
            if (input_enter_btn && lv_obj_is_valid(input_enter_btn))
                lv_obj_clear_flag(input_enter_btn, LV_OBJ_FLAG_HIDDEN);
            start_cursor_blink();
        }
        else
        {
            if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
            {
                // 樣式：透明、無邊框
                lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_0,
                                        LV_PART_MAIN);
                lv_obj_set_style_border_width(text_input_bar_bg, 0,
                                              LV_PART_MAIN);
                // 位置/大小回到 closed 終點（底部小指示帶）
                lv_obj_set_pos(text_input_bar_bg,
                               (LV_HOR_RES_MAX - 200) / 2,
                               LV_VER_RES_MAX - 50);
                lv_obj_set_size(text_input_bar_bg, 200, 50);
            }
            if (input_content_container &&
                lv_obj_is_valid(input_content_container))
                lv_obj_add_flag(input_content_container, LV_OBJ_FLAG_HIDDEN);
            if (input_enter_btn && lv_obj_is_valid(input_enter_btn))
                lv_obj_add_flag(input_enter_btn, LV_OBJ_FLAG_HIDDEN);
            stop_cursor_blink();
        }
    }
}

// 立即套用 mode（不動畫）：初始化用
static void apply_hid_mode(hid_mode_t mode)
{
    current_hid_mode = mode;
    if (connected_device_label && lv_obj_is_valid(connected_device_label))
        lv_label_set_text(connected_device_label, next_mode_name(mode));
    for (int i = 0; i < HID_MODE_COUNT; i++)
    {
        mode_set_translate_x((hid_mode_t)i, 0);
        mode_set_visible((hid_mode_t)i, (hid_mode_t)i == mode);
    }
    keyboard_visible = (mode == HID_MODE_KEYBOARD);
    LOG_D("HID mode -> %s", hid_mode_names[mode]);
}

// 拖動動畫 cb：v 是當前 dx，同步更新 current 與 target 的 translate_x
static void mode_swipe_anim_cb(void *var, int32_t v)
{
    (void)var;
    int16_t dx = (int16_t)v;
    mode_set_translate_x(current_hid_mode, dx);
    int16_t target_offset = mode_swipe_target_side * LV_HOR_RES_MAX + dx;
    mode_set_translate_x(mode_swipe_target, target_offset);
}

// 用 lv_async_call 把 swap / reset 推到下一個 LVGL frame，避開 anim 最後一格
// cb 跟 ready_cb 同 frame 對 translate_x 的 race（前面實測：在 ready_cb 直接
// set 0 會跟 anim cb 設的最終值打架，導致 mode_container 停在非 0 位置）
static void mode_swipe_commit_async_cb(void *user_data)
{
    (void)user_data;
    hid_mode_t old_mode = current_hid_mode;
    current_hid_mode = mode_swipe_target;
    mode_set_visible(old_mode, false);
    mode_set_translate_x(old_mode, 0);
    mode_set_translate_x(current_hid_mode, 0);
    if (connected_device_label && lv_obj_is_valid(connected_device_label))
        lv_label_set_text(connected_device_label,
                          hid_mode_names[current_hid_mode]);
    keyboard_visible = (current_hid_mode == HID_MODE_KEYBOARD);
    mode_swipe_active = false;
    LOG_D("mode commit (async) -> %s", hid_mode_names[current_hid_mode]);
}

static void mode_swipe_cancel_async_cb(void *user_data)
{
    (void)user_data;
    mode_set_visible(mode_swipe_target, false);
    mode_set_translate_x(current_hid_mode, 0);
    mode_set_translate_x(mode_swipe_target, 0);
    mode_swipe_active = false;
}

static void mode_swipe_commit_anim_ready(lv_anim_t *a)
{
    (void)a;
    lv_async_call(mode_swipe_commit_async_cb, NULL);
}

static void mode_swipe_cancel_anim_ready(lv_anim_t *a)
{
    (void)a;
    lv_async_call(mode_swipe_cancel_async_cb, NULL);
}

// 自製 timer-based 動畫
static void mode_swipe_timer_cb(lv_timer_t *t)
{
    uint32_t elapsed = lv_tick_elaps(mode_swipe_start_tick);
    bool finished = (elapsed >= MODE_SWIPE_ANIM_TIME_MS);
    int32_t v;
    if (finished)
    {
        v = mode_swipe_to_dx;
    }
    else
    {
        float p = (float)elapsed / (float)MODE_SWIPE_ANIM_TIME_MS;
        float ease = 1.0f - (1.0f - p) * (1.0f - p) * (1.0f - p);
        v = mode_swipe_from_dx +
            (int32_t)((mode_swipe_to_dx - mode_swipe_from_dx) * ease);
    }
    mode_set_translate_x(current_hid_mode, (int16_t)v);
    int16_t target_offset = mode_swipe_target_side * LV_HOR_RES_MAX + v;
    mode_set_translate_x(mode_swipe_target, target_offset);

    if (finished)
    {
        lv_timer_del(t);
        mode_swipe_timer = NULL;
        if (mode_swipe_is_commit)
        {
            hid_mode_t old_mode = current_hid_mode;
            current_hid_mode = mode_swipe_target;
            mode_set_visible(old_mode, false);
            mode_set_translate_x(old_mode, 0);
            mode_set_translate_x(current_hid_mode, 0);
            if (connected_device_label &&
                lv_obj_is_valid(connected_device_label))
                lv_label_set_text(connected_device_label,
                                  next_mode_name(current_hid_mode));
            keyboard_visible = (current_hid_mode == HID_MODE_KEYBOARD);
            LOG_D("mode commit -> %s", hid_mode_names[current_hid_mode]);
        }
        else
        {
            mode_set_visible(mode_swipe_target, false);
            mode_set_translate_x(current_hid_mode, 0);
            mode_set_translate_x(mode_swipe_target, 0);
        }
        mode_swipe_active = false;
    }
}

static void mode_swipe_kill_timer(void)
{
    if (mode_swipe_timer)
    {
        lv_timer_del(mode_swipe_timer);
        mode_swipe_timer = NULL;
    }
}

static void mode_label_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev)
        return;
    lv_point_t pt;
    lv_indev_get_point(indev, &pt);

    if (code == LV_EVENT_PRESSED)
    {
        mode_swipe_kill_timer();
        mode_swipe_start_x = pt.x;
        mode_swipe_active = true;
        mode_swipe_target =
            (hid_mode_t)(((int)current_hid_mode + 1) % HID_MODE_COUNT);
        mode_swipe_target_side = +1;
        mode_set_visible(mode_swipe_target, true);
        mode_set_translate_x(current_hid_mode, 0);
        mode_set_translate_x(mode_swipe_target, LV_HOR_RES_MAX);
    }
    else if (code == LV_EVENT_PRESSING)
    {
        if (!mode_swipe_active)
            return;
        int16_t dx = pt.x - mode_swipe_start_x;
        mode_swipe_target_side = (dx >= 0) ? -1 : +1;
        mode_set_translate_x(current_hid_mode, dx);
        int16_t target_offset =
            mode_swipe_target_side * LV_HOR_RES_MAX + dx;
        mode_set_translate_x(mode_swipe_target, target_offset);
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        if (!mode_swipe_active)
            return;
        int16_t dx = pt.x - mode_swipe_start_x;
        int abs_dx = abs(dx);

        // 視為點擊（dx 很小）→ instant swap 沒動畫，避免 dx 符號 jitter
        // 造成 target 進場方向不統一
        if (abs_dx < 10)
        {
            mode_swipe_kill_timer();
            hid_mode_t old_mode = current_hid_mode;
            hid_mode_t new_mode = (hid_mode_t)(
                ((int)current_hid_mode + 1) % HID_MODE_COUNT);
            current_hid_mode = new_mode;
            mode_set_visible(old_mode, false);
            mode_set_visible(new_mode, true);
            mode_set_translate_x(old_mode, 0);
            mode_set_translate_x(new_mode, 0);
            if (connected_device_label &&
                lv_obj_is_valid(connected_device_label))
                lv_label_set_text(connected_device_label,
                                  next_mode_name(current_hid_mode));
            keyboard_visible = (new_mode == HID_MODE_KEYBOARD);
            mode_swipe_active = false;
            LOG_D("mode tap -> %s", hid_mode_names[new_mode]);
            return;
        }

        mode_swipe_is_commit = (abs_dx > MODE_SWIPE_COMMIT_THRESHOLD);
        mode_swipe_from_dx = dx;
        mode_swipe_to_dx = mode_swipe_is_commit
                               ? -mode_swipe_target_side * LV_HOR_RES_MAX
                               : 0;
        mode_swipe_start_tick = lv_tick_get();
        mode_swipe_kill_timer();
        mode_swipe_timer = lv_timer_create(mode_swipe_timer_cb, 16, NULL);
    }
}

/**
 * @brief 放開時把節點對齊到「某個節點正好落在 180° 中央」的位置
 *        snap 位置每 18° 一格、bias 9°（5 個合法值：9°, 27°, 45°, 63°, 81°）
 */
static void snap_scroll_nodes(void)
{
    const float spacing = LEFT_SCROLL_NODE_SPACING_DEG; // 18°
    const float bias = spacing * 0.5f;                  // 9°
    float k = roundf((scroll_node_offset_deg - bias) / spacing);
    float target = k * spacing + bias;

    int32_t from = (int32_t)(scroll_node_offset_deg * 100.0f);
    int32_t to = (int32_t)(target * 100.0f);
    if (from == to)
        return; // 已在 snap 位置

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &scroll_node_offset_deg);
    lv_anim_set_exec_cb(&a, scroll_node_snap_anim_cb);
    lv_anim_set_time(&a, 120);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_values(&a, from, to);
    lv_anim_start(&a);
}

/**
 * @brief 建立左側滾動弧線上的節點（初始位置與大小由 update 設定）
 */
static void create_left_scroll_nodes(lv_obj_t *parent)
{
    for (int i = 0; i < LEFT_SCROLL_NODE_COUNT; i++)
    {
        lv_obj_t *line = lv_line_create(parent);
        lv_obj_remove_style_all(line);
        lv_obj_set_pos(line, 0, 0);
        lv_obj_set_style_line_color(line, lv_color_hex(0x4D4D4D), 0);
        lv_obj_set_style_line_rounded(line, true, 0);
        lv_obj_set_style_line_width(line, 3, 0);
        lv_obj_set_style_line_opa(line, LV_OPA_COVER, 0);
        lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
        left_scroll_nodes[i] = line;
    }
    scroll_node_offset_deg = 0.0f;
    scroll_ui_level = 0; // 預設暗/細
    apply_scroll_ui_level();
}

/**
 * @brief 建立 Trackpad mode 的所有 UI 元件（弧形滾動條、節點、debug 邊界等）
 *        要加新的 trackpad 元件就在這個函式內加，parent 用傳進來的參數
 *        （即 mode_container[HID_MODE_TRACKPAD]，會跟著 mode 切換動）
 */
static void create_trackpad_mode_ui(lv_obj_t *parent)
{
    // 左側滾動弧形條（貼著圓形畫面左側邊緣）
    left_scroll_bar = (lv_obj_t *)lv_arc_create(parent);
    lv_obj_set_size(left_scroll_bar, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(left_scroll_bar, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation((lv_obj_t *)left_scroll_bar, 0);
    lv_arc_set_bg_angles((lv_obj_t *)left_scroll_bar, 135, 225);
    lv_arc_set_angles((lv_obj_t *)left_scroll_bar, 0, 0);
    lv_arc_set_mode((lv_obj_t *)left_scroll_bar, LV_ARC_MODE_NORMAL);
    lv_obj_set_style_pad_all(left_scroll_bar, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(left_scroll_bar, LV_OPA_0, LV_PART_KNOB);
    lv_obj_set_style_arc_width(left_scroll_bar, 30, LV_PART_MAIN);
    lv_obj_set_style_arc_color(left_scroll_bar, lv_color_hex(0x333333),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_opa(left_scroll_bar, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_arc_width(left_scroll_bar, 0, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(left_scroll_bar, LV_OPA_0, LV_PART_INDICATOR);
    lv_obj_clear_flag(left_scroll_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(left_scroll_bar, LV_OBJ_FLAG_CLICKABLE);

    // 弧線上的節點指示點
    create_left_scroll_nodes(parent);

    #if SHOW_SCROLL_ZONE_DEBUG
    {
        const float cxf = LV_HOR_RES_MAX / 2.0f;
        const float cyf = LV_VER_RES_MAX / 2.0f;
        const float dist_outer = (float)(LV_HOR_RES_MAX / 2) - 50.0f;
        const float dist_inner =
            dist_outer - (float)CENTER_SCROLL_ZONE_THICKNESS;
        for (int i = 0; i < DBG_ZONE_ARC_POINTS; i++)
        {
            float angle_deg =
                135.0f + 90.0f * (float)i / (float)(DBG_ZONE_ARC_POINTS - 1);
            float rad = angle_deg * LEFT_SCROLL_PI / 180.0f;
            float c = cosf(rad);
            float s = sinf(rad);
            dbg_zone_outer_pts[i].x = (lv_coord_t)(cxf + dist_outer * c);
            dbg_zone_outer_pts[i].y = (lv_coord_t)(cyf + dist_outer * s);
            dbg_zone_inner_pts[i].x = (lv_coord_t)(cxf + dist_inner * c);
            dbg_zone_inner_pts[i].y = (lv_coord_t)(cyf + dist_inner * s);
        }
        lv_obj_t *dbg_line_outer = lv_line_create(parent);
        lv_line_set_points(dbg_line_outer, dbg_zone_outer_pts,
                           DBG_ZONE_ARC_POINTS);
        lv_obj_set_style_line_color(dbg_line_outer, lv_color_hex(0xFF3030), 0);
        lv_obj_set_style_line_width(dbg_line_outer, 2, 0);
        lv_obj_set_style_line_opa(dbg_line_outer, LV_OPA_70, 0);
        lv_obj_set_style_line_rounded(dbg_line_outer, true, 0);
        lv_obj_clear_flag(dbg_line_outer, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(dbg_line_outer, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *dbg_line_inner = lv_line_create(parent);
        lv_line_set_points(dbg_line_inner, dbg_zone_inner_pts,
                           DBG_ZONE_ARC_POINTS);
        lv_obj_set_style_line_color(dbg_line_inner, lv_color_hex(0x3080FF), 0);
        lv_obj_set_style_line_width(dbg_line_inner, 2, 0);
        lv_obj_set_style_line_opa(dbg_line_inner, LV_OPA_70, 0);
        lv_obj_set_style_line_rounded(dbg_line_inner, true, 0);
        lv_obj_clear_flag(dbg_line_inner, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(dbg_line_inner, LV_OBJ_FLAG_SCROLLABLE);
    }
    #endif
}

/**
 * @brief 建立 Keyboard mode 的所有 UI 元件
 *        （input bar、輸入框、Enter 鍵、鍵盤本身）
 *        要加新的 keyboard 元件就在這個函式內加
 */
static void create_keyboard_mode_ui(lv_obj_t *parent)
{
    // Input bar 容器（深色框，keyboard mode 顯示在鍵盤上方）
    text_input_bar_bg = lv_obj_create(parent);
    lv_obj_set_size(text_input_bar_bg, 200, 50);
    lv_obj_set_pos(text_input_bar_bg, (LV_HOR_RES_MAX - 200) / 2,
                   LV_VER_RES_MAX - 50);
    lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_border_width(text_input_bar_bg, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(text_input_bar_bg, 22, LV_PART_MAIN);
    lv_obj_set_style_pad_all(text_input_bar_bg, 5, LV_PART_MAIN);
    lv_obj_clear_flag(text_input_bar_bg, LV_OBJ_FLAG_SCROLLABLE);

    // Indicator line (永遠 hidden，事件處理在 bottom_swipe_area)
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
    lv_obj_add_flag(text_input_bar, LV_OBJ_FLAG_HIDDEN);

    // Input content container
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

    // Enter button
    input_enter_btn = lv_obj_create(parent);
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

    // 鍵盤 layout（內部創建 keyboard_container, custom_keyboard, 所有按鍵）
    create_circular_keyboard_layout(parent);
    if (keyboard_container)
    {
        // mode 系統用 hidden 控制可見性，需要解除預設 hidden 並 reset translate_y
        lv_obj_clear_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_y(keyboard_container, 0, 0);
        lv_obj_align(keyboard_container, LV_ALIGN_BOTTOM_MID, 0, 0);
    }
    if (custom_keyboard)
        lv_obj_clear_flag(custom_keyboard, LV_OBJ_FLAG_HIDDEN);
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
    // 關掉 bg 的 scroll：拖 mode_label 時 scroll event 冒泡到 bg 會被 bg 攔截，
    // 導致 child 被偏移（看起來像動畫終點位置不對）
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLL_CHAIN);

    // Touch background（跨 mode 觸控接收）
    lv_obj_t *touch_bg = lv_obj_create(bg);
    lv_obj_set_size(touch_bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(touch_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(touch_bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(touch_bg, LV_OPA_0, 0);
    lv_obj_add_event_cb(touch_bg, plain_event_cb, LV_EVENT_ALL, NULL);

    // === Mode containers（每個 mode 一個 480×480 透明容器）===
    for (int i = 0; i < HID_MODE_COUNT; i++)
    {
        mode_container[i] = lv_obj_create(bg);
        lv_obj_remove_style_all(mode_container[i]);
        lv_obj_set_size(mode_container[i], LV_HOR_RES_MAX, LV_VER_RES_MAX);
        lv_obj_set_pos(mode_container[i], 0, 0);
        lv_obj_set_style_bg_opa(mode_container[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(mode_container[i], 0, 0);
        lv_obj_set_style_pad_all(mode_container[i], 0, 0);
        lv_obj_clear_flag(mode_container[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(mode_container[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // === Per-mode UI ===
    // 加新元件改下面兩個函式
    create_trackpad_mode_ui(mode_container[HID_MODE_TRACKPAD]);
    create_keyboard_mode_ui(mode_container[HID_MODE_KEYBOARD]);

    // === Cross-mode UI（跨 mode 共用元件）===
    // 向右返回 hint：螢幕左側中央，預設隱藏；超過拖曳門檻會顯示成圓 + 左箭頭
    back_hint_obj = lv_obj_create(bg);
    lv_obj_remove_style_all(back_hint_obj);
    lv_obj_set_size(back_hint_obj, BACK_HINT_SIZE, BACK_HINT_SIZE);
    lv_obj_align(back_hint_obj, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(back_hint_obj, lv_color_hex(0x444444), 0);
    lv_obj_set_style_bg_opa(back_hint_obj, LV_OPA_80, 0);
    lv_obj_set_style_radius(back_hint_obj, 25, 0);
    lv_obj_clear_flag(back_hint_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(back_hint_obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(back_hint_obj, LV_OBJ_FLAG_HIDDEN);
    back_hint_hidden = true;
    back_hint_vibrated = false;
    back_hint_anim_is_running = false;

    back_hint_icon = lv_img_create(back_hint_obj);
    lv_img_set_src(back_hint_icon, &img_left_arrow);
    lv_obj_align(back_hint_icon, LV_ALIGN_CENTER, 0, 0);

    // 從下往上拉觸發 multitask 的 hint：螢幕底部中央，圖示用 Map_fill
    multitask_hint_obj = lv_obj_create(bg);
    lv_obj_remove_style_all(multitask_hint_obj);
    lv_obj_set_size(multitask_hint_obj, MULTITASK_HINT_SIZE, MULTITASK_HINT_SIZE);
    lv_obj_align(multitask_hint_obj, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(multitask_hint_obj, lv_color_hex(0x444444), 0);
    lv_obj_set_style_bg_opa(multitask_hint_obj, LV_OPA_80, 0);
    lv_obj_set_style_radius(multitask_hint_obj, 25, 0);
    lv_obj_clear_flag(multitask_hint_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(multitask_hint_obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(multitask_hint_obj, LV_OBJ_FLAG_HIDDEN);
    multitask_hint_hidden = true;
    multitask_hint_vibrated = false;
    multitask_hint_anim_is_running = false;

    multitask_hint_icon = lv_img_create(multitask_hint_obj);
    lv_img_set_src(multitask_hint_icon, &Map_fill);
    lv_obj_set_style_img_opa(multitask_hint_icon, LV_OPA_TRANSP, 0);
    lv_obj_align(multitask_hint_icon, LV_ALIGN_CENTER, 0, 0);

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
    lv_obj_t *switch_mode = lv_img_create(bg);
    lv_img_set_src(switch_mode, &switch_icon);
    lv_obj_align(switch_mode, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_event_cb(switch_mode, mode_label_event_cb, LV_EVENT_ALL, NULL);

    // Connected device name label at top
    connected_device_label = lv_label_create(bg);
    lv_label_set_text(connected_device_label, "");
    lv_obj_set_size(connected_device_label, 150, 44);
    lv_obj_set_style_text_color(connected_device_label, lv_color_hex(0xAAAAAA),
                                0);
    lv_obj_set_style_text_align(connected_device_label, LV_TEXT_ALIGN_CENTER,
                                0);
    lv_label_set_long_mode(connected_device_label, LV_LABEL_LONG_DOT);
    lv_obj_align(connected_device_label, LV_ALIGN_TOP_MID, 0, 24);
    // 改用作 mode label：可左右拖動切換模式
    lv_obj_add_flag(connected_device_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(connected_device_label, mode_label_event_cb,
                        LV_EVENT_ALL, NULL);
    lv_label_set_text(connected_device_label, next_mode_name(current_hid_mode));

    // FSR-402 ADC real-time display label
    // fsr_adc_label = lv_label_create(bg);
    // lv_label_set_text(fsr_adc_label, "FSR: --");
    // lv_obj_set_style_text_color(fsr_adc_label, lv_color_hex(0x00FF88), 0);
    // lv_obj_set_style_text_font(fsr_adc_label,
    //                            LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    // lv_obj_align(fsr_adc_label, LV_ALIGN_TOP_MID, 0, 50);
    // lv_obj_clear_flag(fsr_adc_label, LV_OBJ_FLAG_CLICKABLE);

    // Init ADC and start periodic reading
    #ifndef USE_FSR_ADC
        // fsr_adc_init();
    #endif
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

    // Trackpad mode 下方 multitask hint 觸發區（跨 mode hit area）
    bottom_swipe_area = lv_obj_create(bg);
    lv_obj_remove_style_all(bottom_swipe_area);
    lv_obj_set_size(bottom_swipe_area, 200, 50);
    lv_obj_set_pos(bottom_swipe_area, (LV_HOR_RES_MAX - 200) / 2,
                   LV_VER_RES_MAX - 50);
    lv_obj_set_style_bg_opa(bottom_swipe_area, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(bottom_swipe_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(bottom_swipe_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(bottom_swipe_area, text_input_bar_cb, LV_EVENT_ALL,
                        NULL);

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

    apply_hid_mode(current_hid_mode);
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

void set_hid_mouse_handfree_mode(void)
{
    handfree = !handfree;
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
    // if (handfree)
    // {
        extern void switch_watch_motion_control_mode(bool enable,
                                                     bool animation);
        switch_watch_motion_control_mode(true, false);
    // }

    setting_provider.set_power_save_mode(0);
}

/**
 * @brief Handles application pause
 */
static void on_pause(void)
{
    setting_provider.set_power_save_mode(1);
    switch_watch_motion_control_mode(false, false);
}

/**
 * @brief Handles application stop
 */
static void on_stop(void)
{
    app_control_set_mouse_mode(false);

    // 停掉底部 bar 多工鍵 timer（如果還在走）
    if (bottom_bar_multitask_timer != NULL)
    {
        rt_timer_stop(bottom_bar_multitask_timer);
        rt_timer_delete(bottom_bar_multitask_timer);
        bottom_bar_multitask_timer = NULL;
    }
    bottom_bar_multitask_ready = false;
    bottom_bar_multitask_fired = false;

    // Clean up menu tileview
    menu_tileview = NULL;
    menu_home_tile = NULL;
    menu_content_tile = NULL;
    menu_bg = NULL;
    menu_swipe_area = NULL;
    bottom_swipe_area = NULL;
    lv_anim_del(&scroll_ui_level, NULL); // 停掉 UI 過渡動畫
    lv_anim_del(&scroll_node_offset_deg, NULL); // 停掉節點 snap 動畫
    lv_anim_del(&back_hint_anim, NULL);          // 停掉 back hint 進場動畫
    lv_anim_del(&back_hint_release_anim, NULL);  // 停掉 back hint 釋放動畫
    back_hint_obj = NULL;
    back_hint_icon = NULL;
    back_hint_hidden = true;
    back_hint_vibrated = false;
    back_hint_anim_is_running = false;
    back_pending_active = false;
    back_hint_drag_offset = 0;
    lv_anim_del(&multitask_hint_anim, NULL);
    lv_anim_del(&multitask_hint_release_anim, NULL);
    multitask_hint_obj = NULL;
    multitask_hint_icon = NULL;
    multitask_hint_hidden = true;
    multitask_hint_vibrated = false;
    multitask_hint_anim_is_running = false;
    multitask_pending_active = false;
    multitask_hint_drag_offset = 0;
    lv_anim_del(&mode_swipe_anim_value, mode_swipe_anim_cb);
    lv_async_call_cancel(mode_swipe_commit_async_cb, NULL);
    lv_async_call_cancel(mode_swipe_cancel_async_cb, NULL);
    if (mode_swipe_timer)
    {
        lv_timer_del(mode_swipe_timer);
        mode_swipe_timer = NULL;
    }
    mode_swipe_active = false;
    for (int i = 0; i < HID_MODE_COUNT; i++)
        mode_container[i] = NULL;
    left_scroll_bar = NULL;
    for (int i = 0; i < LEFT_SCROLL_NODE_COUNT; i++)
    {
        left_scroll_nodes[i] = NULL;
    }
    scroll_node_offset_deg = 0.0f;
    scroll_accum_angle = 0.0f;
    scroll_last_theta = 0.0f;
    scroll_ui_level = 0;
    left_scroll_active = false;

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
    #ifndef USE_FSR_ADC
    // fsr_adc_deinit();
    #endif
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
extern void switch_watch_motion_control_mode(bool enable, bool animation);
void watch_system_mouse_resume(void)
{
    app_control_set_mouse_mode(true);
    // if (handfree)
    // {
    
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
    switch_watch_motion_control_mode(false, false);
    setting_provider.set_power_save_mode(1);
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