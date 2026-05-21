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
    #include "bloc_v2t.h" // start_voice_recognition + voice_provider (AI 路徑用)
#endif

#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif

#include "communicate_protocol.h"
#include "communicate_task.h" // commu_send_skaibar_selected
#include "app_clock_status_bar.h" // app_clock_device_change_bar_open / set_device_change_bar_area_right_state
#include "ui_handler.h"
#include <cJSON.h>
#include "ui_helper.h"
#include "ui_img_helper.h"
#include "lvsf_gesture.h"
#include "ble_device_manager.h"
#include "bloc_motion_tracking.h"
#include "ble_hid.h"
#ifndef BSP_USING_PC_SIMULATOR
/* SiFli chip HAL headers — ARM-only, and currently unused in this TU (no
   HAL_ or bf0_ calls in the body). Guarded so the UI layer compiles under
   the PC simulator (T1 part 3: ARM seam). */
#include "bf0_hal.h"
#include "bf0_sys_cfg.h"
#endif
#include "hid_mouse.h" /* component API: hid_mouse_create / hid_mouse_destroy */

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

    // FSR-402 pressure sensor: HW driver in bloc_peripheral.c, sampler thread
    // in bloc_control.c (gated by USING_FSR_ADC_SAMPLER).

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
LV_IMG_DECLARE(img_left_arrow); // 還有給 back_hint_icon 用
LV_IMG_DECLARE(Map_fill);
LV_IMG_DECLARE(micro_icon);
LV_IMG_DECLARE(micro_open_icon); // V2T active 時的 icon（淺藍麥克風）
LV_IMG_DECLARE(switch_icon);
LV_IMG_DECLARE(keyboard_icon);
LV_IMG_DECLARE(down_arrow); // 輸入框下方收回按鈕

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
static lv_obj_t *v2t_mic_img = NULL;
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
// 起手點是否落在「右」弧形觸碰區，給 device-change 選單左滑手勢用
static bool press_in_right_arc_zone = false;

// is_point_in_right_arc 真正定義在下面 (~line 400)；這裡 forward decl 給
// device_change_bar_hit_test_cb 用
static bool is_point_in_right_arc(const lv_point_t *p);

// 右側 device-change bar 跟弧形滾動觸碰區重疊；用 HIT_TEST 過濾：
// 點如果在弧形觸碰區就拒絕命中，press 落到 touch_bg 走 arc 滾動；
// 點在 bar 範圍但不在弧形區（純邊緣）才命中 bar，跑原本左滑開選單流程
static void device_change_bar_hit_test_cb(lv_event_t *e)
{
    lv_hit_test_info_t *info = lv_event_get_hit_test_info(e);
    if (info->res == false) return; // 標準 hit 已 reject 就不用再判
    if (is_point_in_right_arc(info->point))
        info->res = false;
}

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
// 右側滾動條（mirror，跟 left 一起在 bg，跨 mode 共用）
static lv_obj_t *right_scroll_bar = NULL;
static lv_obj_t *right_scroll_nodes[LEFT_SCROLL_NODE_COUNT] = {NULL};
static lv_point_t right_scroll_node_pts[LEFT_SCROLL_NODE_COUNT][2];
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

// keyboard mode 下半部 mic 區前置宣告
static void create_kbd_mic_section(lv_obj_t *parent);
static void kbd_lower_set_keyboard(bool show_kbd);
static void kbd_lower_arrow_event_cb(lv_event_t *e);
static void kbd_mic_btn_event_cb(lv_event_t *e);
static void kbd_lower_update_arrows_visibility(void);
static void kbd_lower_update_arcs_visibility(void);
// 點擊輸入模式外的空白區域 → 收回 trackpad
static void keyboard_mode_outside_click_cb(lv_event_t *e);
// trackpad bar 點擊 → keyboard mode 的展開動畫
static void start_trackpad_to_kbd_expand_anim(void);
// 輸入框往下拖 → 收回 trackpad 的動畫
static void text_input_bar_drag_event_cb(lv_event_t *e);
static void start_kbd_to_trackpad_collapse_anim(int32_t from_progress,
                                                bool commit);

// media center 前置宣告
static void create_media_center_panel(lv_obj_t *parent);
static void media_center_set_open(bool open, bool animate);
static void media_center_update_play_icon(bool playing);
static void media_center_play_btn_cb(lv_event_t *e);
static void media_center_prev_btn_cb(lv_event_t *e);
static void media_center_next_btn_cb(lv_event_t *e);
static void media_center_vol_up_btn_cb(lv_event_t *e);
static void media_center_vol_down_btn_cb(lv_event_t *e);
static void status_bar_area_up_cb(lv_event_t *e);
static void media_tileview_event_cb(lv_event_t *e);
static void hid_mode_toggle(void);

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

// 右側滾動觸發區（mirror of left arc，角度 -55°~+55°）
static bool is_point_in_right_arc(const lv_point_t *p)
{
    float cx = LV_HOR_RES_MAX / 2.0f;
    float cy = LV_VER_RES_MAX / 2.0f;
    float dx = p->x - cx;
    float dy = p->y - cy;
    float dist = sqrtf(dx * dx + dy * dy);
    float outer_r = cx;
    float inner_r = outer_r - 50.0f;
    if (dist < inner_r || dist > outer_r)
        return false;
    // 只接受右側（x > 中心）
    if (dx <= 0)
        return false;
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
static lv_obj_t *input_enter_img = NULL; // 裡面的 enter icon img，用 lv_img_set_zoom 縮放
static lv_timer_t *cursor_blink_timer = NULL;
static char input_buffer[128] = {0};
static int input_length = 0;
static bool cursor_visible = true;
// V2T 鎖：當輸入框累積 ≥ 90 字符時 set true，自動關麥克風；
// input 清空（buffer 空）才解鎖，期間不允許再次長按空白鍵啟動 V2T
static bool mouse_v2t_locked = false;
    #define MOUSE_V2T_MAX_CHARS 90

// trackpad bar → input bar expand 動畫的終點尺寸/位置
// (跨多個函式被引用，所以提前在這裡 #define 而不是塞到 expand 區塊裡)
    #define EXPAND_END_W 380
    #define EXPAND_END_H 90
    #define EXPAND_END_X ((LV_HOR_RES_MAX - EXPAND_END_W) / 2)
// mic view 時 input bar y：置中於螢幕（480 高 / 2 - 90/2 = 195）
// 切到 keyboard view 時上升到 EXPAND_END_Y_KBD（騰出鍵盤空間）
    #define EXPAND_END_Y 195
    #define EXPAND_END_Y_KBD 75
// V2T 啟動中（mic 開）：keyboard mode 的 space btn 跟 trackpad mode 的 mic btn
// 都顯示紅圓 X，按下關 mic + paste + clear
static bool mouse_v2t_active = false;
// keyboard mode 的 space btn 內元件
static lv_obj_t *space_btn = NULL;
static lv_obj_t *space_icon = NULL;
static lv_obj_t *mic_icon = NULL;
static lv_obj_t *space_red_dot = NULL;
static lv_obj_t *space_red_dot_x = NULL;
// trackpad mode 的 mic btn 內元件（功能跟 keyboard 長按 space 完全等效）
static lv_obj_t *trackpad_mic_btn = NULL;
static lv_obj_t *trackpad_mic_icon = NULL;
static lv_obj_t *trackpad_mic_red_dot = NULL;
static lv_obj_t *trackpad_mic_red_dot_x = NULL;

// Keyboard mode 下半部 mic 區（mic 按鈕 + 右側鍵盤按鈕，跟 keyboard 互換顯示）
static lv_obj_t *kbd_mic_section = NULL;
static lv_obj_t *kbd_mic_section_mic_btn = NULL;
static lv_obj_t *kbd_mic_section_mic_img = NULL; // 中央 mic icon，V2T 開關時切圖
static lv_obj_t *kbd_mic_section_mic_pulse = NULL; // V2T active 時的脈衝圓
static lv_obj_t *kbd_mic_section_right_arrow = NULL;
// micro_open_icon 是淺藍麥克風，脈衝色取相近的藍
#define KBD_MIC_PULSE_COLOR 0x5DA8FF
#define KBD_MIC_PULSE_MAX_SIZE 100
#define KBD_MIC_PULSE_PERIOD_MS 1200
static bool kbd_lower_is_keyboard = false; // false = mic 區顯示，true = 鍵盤顯示
// collapse 動畫進行中：擋掉外部入口（outside-click / down-arrow / drag）重複觸發
static bool collapse_anim_running = false;

// （之前的 kbd_right_arc 已刪除，改用 trackpad mode 的左/右兩側弧形跨 mode 共用）

// =====================================================================
// 媒體中心 pull-down（仿 app_clock_status_bar 的 tileview 模式）
// =====================================================================
//   - 頂部 status_bar_area_up 是個透明 hit zone：PRESS 時把 tileview 顯示出來
//     並設到 home tile (0,0)；RELEASE（press 沒被 tileview 接走）時隱藏
//     tileview，並執行 mode toggle（tap-to-switch）
//   - tileview 兩格：home (0,0) 透明、media (0,1) 媒體中心內容
//     LV_DIR_BOTTOM/TOP 讓使用者可以拖下去開、拖上去收
//   - tileview value-changed：snap 回 home 時自動隱藏 tileview
//   - title / play state 由 bloc_control notify_media_title 路由進來
static lv_obj_t *status_bar_area_up = NULL;
static lv_obj_t *media_tileview = NULL;
static lv_obj_t *media_home_tile = NULL;
static lv_obj_t *media_tile = NULL;
static lv_obj_t *media_center_title_label = NULL;
static lv_obj_t *media_center_play_btn = NULL;
static lv_obj_t *media_center_play_img = NULL;
static bool media_center_play_state = false;

static void update_v2t_btn_appearance(bool mic_active);
static void mouse_v2t_set_active(bool active);
static void mouse_v2t_open(void);
static void mouse_v2t_open_with_intent(uint8_t intent);
static void mouse_v2t_close_and_paste(void);
// 由 mouse_v2t_open_with_intent 設定，open_v2t_mic (LVGL msg handler)
// 讀取以決定 start_voice_recognition 的 intent；handler 結束時 reset 回預設
static uint8_t pending_v2t_intent = V2T_INTENT_MIC_INPUTE;
// trackpad bar 長按是否已觸發 → 抑制 RELEASED 走短按的 expand 流程
static bool bar_long_press_fired = false;

// === skaibar 模式（V2T CHAT 開啟期間活躍）======================================
//   PC 透過 KEY_SKAIBAR_OPTIONS 送 JSON 選項列表 (e.g. {"options":["a","b","c"]})
//   使用者旋轉左/右 arc 時，輸入框文字改成當前選項、送 KEY_SKAIBAR_SELECTED(idx)
    #define SKAIBAR_MAX_OPTIONS 16
    #define SKAIBAR_MAX_OPTION_LEN 64
static bool skaibar_active = false;
static char skaibar_options[SKAIBAR_MAX_OPTIONS][SKAIBAR_MAX_OPTION_LEN];
static uint8_t skaibar_options_count = 0;
static int16_t skaibar_selected_idx = -1; // -1 = 還沒選任何選項
static void skaibar_apply_selection_to_input_bar(void);
static void skaibar_scroll_step(int steps);
void mouse_skaibar_set_options_json(const char *json);
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

    #if USING_EDGE_BOTTOM_DETECTION
static void start_multiple_pages_timer(void);
    #endif

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
    mouse_v2t_locked = false; // 清空後解鎖 V2T
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
        if (input_length == 0)
            mouse_v2t_locked = false; // backspace 刪到清空後解鎖 V2T
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

// kbd_lower_arrow_event_cb 中 keyboard 往下滑出後 hide，下次升起才會從畫面外起步
static void keyboard_hide_after_slide_down_cb(lv_anim_t *a)
{
    lv_obj_t *obj = (lv_obj_t *)a->var;
    if (obj && lv_obj_is_valid(obj))
    {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        // translate_y 還原成 0，下次手動再設 300 起步
        lv_obj_set_style_translate_y(obj, 0, 0);
    }
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
        if (mouse_v2t_locked)
        {
            // V2T 鎖定中（input 滿 90 字尚未清空），不允許再次啟動
            LOG_D("V2T locked, ignore long-press space");
            return;
        }
        is_long_press_triggered = true;
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_MOUSE_OPEN_V2T;
        lvgl_send_msg(msg);
    }
}

void open_v2t_mic(void)
{
    extern void set_voice_recognition_notified_from_mouse(bool status);
    set_voice_recognition_notified_from_mouse(true);
    // 預設 intent (MIC_INPUTE) 走 interact_mic_v2t_input；其他 intent (e.g. CHAT)
    // 直接呼叫 start_voice_recognition 帶該 intent，避開 interact 內 hardcode 的 0x04
    if (pending_v2t_intent == V2T_INTENT_MIC_INPUTE)
    {
        interact_mic_v2t_input();
    }
    else
    {
        voice_provider.vad_init();
        start_voice_recognition(pending_v2t_intent);
    }
    // mic 啟動 → space btn 切成紅圓 X，按下會關 mic
    mouse_v2t_set_active(true);

    // 隱藏彈出框（如果有的話）
    hide_key_popup();

    // 用完 reset 回預設，避免影響下一次 open
    pending_v2t_intent = V2T_INTENT_MIC_INPUTE;
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
        // all_keys[] 只裝鍵盤上的鍵（透過 register_key_button 加進來），
        // 都是 keyboard_container 的 child → 加上 container 偏移就是絕對座標
        lv_coord_t btn_x = lv_obj_get_x(btn) + lv_obj_get_x(keyboard_container);
        lv_coord_t btn_y = lv_obj_get_y(btn) + lv_obj_get_y(keyboard_container);
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
                    if (mouse_v2t_active)
                    {
                        mouse_v2t_close_and_paste();
                    }
                    else
                    {
                        LOG_D("Space key pressed");
                        control_provider.ble_hid_keyboard_input(" ");
                        add_to_input_buffer(" "); // 添加空格到顯示
                    }
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
    // 拿掉鍵盤上方的白色邊框
    lv_obj_set_style_border_width(keyboard_container, 0, LV_PART_MAIN);
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
    lv_obj_set_pos(mode_btn, 65, row4_y);
    lv_obj_set_style_bg_opa(mode_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(mode_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(mode_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(mode_btn, LV_OBJ_FLAG_CLICKABLE);
    register_key_button(mode_btn);
    lv_obj_t *mode_icon = lv_img_create(mode_btn);
    lv_img_set_src(mode_icon, &erth);
    lv_obj_center(mode_icon);

    // Space 按鍵
    space_btn = lv_obj_create(keyboard_container);
    lv_obj_set_size(space_btn, 120, 50);
    lv_obj_set_pos(space_btn, 160, row4_y);
    lv_obj_set_style_bg_opa(space_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(space_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(space_btn, LV_OBJ_FLAG_SCROLLABLE);
    space_icon = lv_img_create(space_btn);
    lv_img_set_src(space_icon, &space);
    lv_obj_align(space_icon, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_style_img_opa(space_icon, LV_OPA_50, LV_PART_MAIN);

    lv_obj_clear_flag(space_btn, LV_OBJ_FLAG_CLICKABLE);
    register_key_button(space_btn);
    mic_icon = lv_img_create(space_btn);
    lv_img_set_src(mic_icon, &icon_mic);
    lv_obj_align(mic_icon, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(mic_icon, 256 * 0.7);
    lv_obj_set_style_img_opa(mic_icon, LV_OPA_30, LV_PART_MAIN);

    // 紅色圓點 + 白色 X：mic 啟動時顯示，按下關 mic（預設隱藏）
    space_red_dot = lv_obj_create(space_btn);
    lv_obj_remove_style_all(space_red_dot);
    lv_obj_set_size(space_red_dot, 120, 60);
    lv_obj_align(space_red_dot, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(space_red_dot, lv_color_hex(0x4A83FF), 0);
    lv_obj_set_style_bg_opa(space_red_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(space_red_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(space_red_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(space_red_dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(space_red_dot, LV_OBJ_FLAG_HIDDEN);

    space_red_dot_x = lv_label_create(space_red_dot);
    lv_label_set_text(space_red_dot_x, "確定");
    lv_obj_set_style_text_color(space_red_dot_x, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(space_red_dot_x, LV_ALIGN_CENTER, 0, 0);

    // Del 按鍵（原本 Enter 的位置）
    lv_obj_t *del_btn = lv_obj_create(keyboard_container);
    lv_obj_set_size(del_btn, 80, 50);
    lv_obj_set_pos(del_btn, 300, row4_y);
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
    bool in_left_arc = is_point_in_left_arc(&start_point);
    bool in_right_arc = is_point_in_right_arc(&start_point);
    bool in_center = is_point_in_center_scroll_zone(&start_point);
    bool in_arc = in_left_arc || in_right_arc;
    press_in_arc_zone = in_arc;
    press_in_right_arc_zone = in_right_arc;
    if (in_arc || in_center)
    {
        center_zone_pending = true;
        // 取消可能還在跑的 snap 動畫，避免按下後 offset 被動畫繼續覆蓋
        lv_anim_del(&scroll_node_offset_deg, scroll_node_snap_anim_cb);
        animate_scroll_ui_to(true); // 觸碰立刻給視覺回饋（亮起）
        LOG_D("scroll zone pending (left=%d right=%d center=%d)",
              in_left_arc, in_right_arc, in_center);
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
            // skaibar 模式（長按 bar 進入 + PC 已送選項）→ 滾動切選項，
            // 不送 BLE HID wheel_scroll；保留 motor_pattern_damping 觸覺回饋
            if (skaibar_active && skaibar_options_count > 0)
            {
                motor_pattern_damping();
                skaibar_scroll_step(steps);
                return;
            }
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
            else if (press_in_right_arc_zone)
            {
                // 右弧區起手 + 往左拖 → 開 device-change 選單
                // 弧形帶的左滑會被 bar ADV_HITTEST 拒絕落到 touch_bg，
                // 在這裡 programmatic open 補上選單入口
                animate_scroll_ui_to(false);
                scrolling = true; // 避免放開時誤觸 click
                app_clock_device_change_bar_open();
                LOG_D("right-arc swipe left -> device-change menu");
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
        break;

    case LV_EVENT_PRESSING:
        break;

    case LV_EVENT_RELEASED:
        user_touching = false;
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

// Forward declarations for BLE functions
extern void ble_app_advertising_start(bool mouse_mode, bool pairing_mode);

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
        ble_app_advertising_start(true, false);
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

// kbd_mic_section 中央 mic 按鈕的脈衝動畫：v ∈ [0,100] 同時驅動 size 跟 opa
//   v=0   → size 0×0、bg_opa COVER（剛開始浮現）
//   v=100 → size 40×40、bg_opa TRANSP（擴散到最大時透明）
//   repeat infinite 形成循環脈衝
static void kbd_mic_pulse_anim_cb(void *var, int32_t v)
{
    (void)var;
    if (!kbd_mic_section_mic_pulse ||
        !lv_obj_is_valid(kbd_mic_section_mic_pulse))
        return;
    lv_coord_t s = (lv_coord_t)(KBD_MIC_PULSE_MAX_SIZE * v / 100);
    if (s < 2) s = 2; // LVGL 對 0/1 px obj 可能 skip render，保底 2px
    lv_obj_set_size(kbd_mic_section_mic_pulse, s, s);
    // align CENTER 在某些 LVGL 8 版本不會因 set_size 自動 re-center → 顯式 re-center
    lv_obj_center(kbd_mic_section_mic_pulse);
    lv_opa_t opa = (lv_opa_t)(LV_OPA_COVER - (LV_OPA_COVER * v / 100));
    lv_obj_set_style_bg_opa(kbd_mic_section_mic_pulse, opa, LV_PART_MAIN);
    lv_obj_invalidate(kbd_mic_section_mic_pulse);
}

// input bar 邊框 opa 閃爍動畫 cb：v 是 0~100 → 對應 border_opa
static void input_bar_blink_anim_cb(void *var, int32_t v)
{
    (void)var;
    if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
        lv_obj_set_style_border_opa(text_input_bar_bg, (lv_opa_t)v,
                                    LV_PART_MAIN);
}

static void kbd_mic_section_set_v2t_active(bool active)
{
    if (kbd_mic_section_mic_img &&
        lv_obj_is_valid(kbd_mic_section_mic_img))
    {
        lv_img_set_src(kbd_mic_section_mic_img,
                       active ? &micro_open_icon : &micro_icon);
    }
    // 中央 mic btn 已 hidden（新版需求改用 input bar 邊框閃爍表示 V2T 啟動）；
    // pulse 動畫仍處理一下避免殘留：active 時不啟動、inactive 時清掉
    if (kbd_mic_section_mic_pulse &&
        lv_obj_is_valid(kbd_mic_section_mic_pulse))
    {
        lv_anim_del(kbd_mic_section_mic_pulse, kbd_mic_pulse_anim_cb);
        lv_obj_add_flag(kbd_mic_section_mic_pulse, LV_OBJ_FLAG_HIDDEN);
    }

    // input bar 邊框閃爍：active 時 LV_OPA_30 ↔ LV_OPA_COVER 循環，inactive 時穩定
    lv_anim_del(NULL, input_bar_blink_anim_cb);
    if (active)
    {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, &input_bar_blink_anim_cb); // 用 cb 位址當識別
        lv_anim_set_exec_cb(&a, input_bar_blink_anim_cb);
        lv_anim_set_values(&a, LV_OPA_30, LV_OPA_COVER);
        lv_anim_set_time(&a, 600);
        lv_anim_set_playback_time(&a, 600);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_start(&a);
    }
    else if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
    {
        // 還原原本邊框 opa（沒啟動 V2T 時 keyboard mode 的 input bar 邊框是 50%）
        lv_obj_set_style_border_opa(text_input_bar_bg, LV_OPA_50, LV_PART_MAIN);
    }
}

// 切換 V2T 按鈕（keyboard space btn + trackpad mic btn）內 child 的可見性：
//   mic_active = true  → 隱藏 mic icon、顯示紅圓 X
//   mic_active = false → 反之
static void update_v2t_btn_appearance(bool mic_active)
{
    // Keyboard mode 的 space btn
    if (space_icon && lv_obj_is_valid(space_icon))
    {
        if (mic_active)
            lv_obj_add_flag(space_icon, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(space_icon, LV_OBJ_FLAG_HIDDEN);
    }
    if (mic_icon && lv_obj_is_valid(mic_icon))
    {
        if (mic_active)
            lv_obj_add_flag(mic_icon, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(mic_icon, LV_OBJ_FLAG_HIDDEN);
    }
    if (space_red_dot && lv_obj_is_valid(space_red_dot))
    {
        if (mic_active)
            lv_obj_clear_flag(space_red_dot, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(space_red_dot, LV_OBJ_FLAG_HIDDEN);
    }
    // Trackpad mode 的 trackpad_mic_btn 現在是 home-indicator 樣式的細 bar
    // （100×16 BOTTOM_MID -20），不再隨 V2T 狀態改尺寸；
    // 之前 50×50 / 120×60 的 resize 是舊的 capsule 「確定」按鈕殘留 → 移除
    // （否則 V2T close 後 bar 會被改成 50×50 → 變白色方塊）
    if (trackpad_mic_icon && lv_obj_is_valid(trackpad_mic_icon))
    {
        if (mic_active)
            lv_obj_add_flag(trackpad_mic_icon, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(trackpad_mic_icon, LV_OBJ_FLAG_HIDDEN);
    }
    if (trackpad_mic_red_dot && lv_obj_is_valid(trackpad_mic_red_dot))
    {
        if (mic_active)
        {
            lv_obj_set_size(trackpad_mic_red_dot, 120, 60);
            lv_obj_align(trackpad_mic_red_dot, LV_ALIGN_CENTER, 0, 0);
            lv_obj_clear_flag(trackpad_mic_red_dot, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_set_size(trackpad_mic_red_dot, 50, 50);
            lv_obj_add_flag(trackpad_mic_red_dot, LV_OBJ_FLAG_HIDDEN);
        }
    }
    // Keyboard mode mic 區中央按鈕：換 icon + 啟停脈衝動畫
    kbd_mic_section_set_v2t_active(mic_active);
}

// 集中設 V2T 啟動狀態 + 同步兩個 mode 的按鈕視覺 + trackpad 中央 panel
static void mouse_v2t_set_active(bool active)
{
    mouse_v2t_active = active;
    // skaibar_active 由「怎麼進入輸入模式」決定（長按 / 短按），跟 V2T
    // 開關獨立 —— V2T 關掉後使用者還能用鍵盤打字、滾 arc 切選項
    update_v2t_btn_appearance(active);
}

// 開 mic with intent：
//   V2T_INTENT_MIC_INPUTE → 純語音輸入（短按 mic / 鍵盤空白鍵長按等預設路徑）
//   V2T_INTENT_SKAIBAR    → AI 對話 + 選項列表（trackpad bar 長按）
// intent 暫存到 pending_v2t_intent，由 open_v2t_mic (LVGL msg handler) 讀取
static void mouse_v2t_open_with_intent(uint8_t intent)
{
    if (mouse_v2t_locked)
    {
        LOG_D("V2T locked, ignore open");
        return;
    }
    pending_v2t_intent = intent;
    is_long_press_triggered = true;
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MOUSE_OPEN_V2T;
    lvgl_send_msg(msg);
}

// 預設：純語音輸入
static void mouse_v2t_open(void)
{
    mouse_v2t_open_with_intent(V2T_INTENT_MIC_INPUTE);
}

// 關 mic（不清空 input bar、不貼上）
// 清空只在使用者按 Enter 時做（input_enter_btn_event_cb / 鍵盤 Enter 鍵）
static void mouse_v2t_close_and_paste(void)
{
    interact_mic_listen(false);
    mouse_v2t_set_active(false);
    LOG_D("V2T close (input bar preserved)");
}

// trackpad mic btn 點擊：根據當前狀態切換開/關
static void trackpad_mic_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    if (mouse_v2t_active)
        mouse_v2t_close_and_paste();
    else
        mouse_v2t_open();
}

/**
 * @brief 把 V2T 結果文字直接 set 到 hid_mouse 的 input bar
 *        （V2T 是 streaming 累積文字，每次來都是「目前為止全文」→ 整個 replace buffer）
 *        注意：必須在 LVGL thread 內呼叫；外部入口請用 append_text_to_mouse_input()
 */
void mouse_apply_v2t_input(const char *text)
{
    if (text == NULL)
        return;
    size_t text_len = strlen(text);
    if (text_len >= sizeof(input_buffer))
        text_len = sizeof(input_buffer) - 1;
    memset(input_buffer, 0, sizeof(input_buffer));
    memcpy(input_buffer, text, text_len);
    input_buffer[text_len] = '\0';
    input_length = (int)text_len;
    if (input_display_label != NULL)
        update_input_display();

    // 字數超過上限 → 自動關麥克風 + 鎖定（直到 input 清空才能再開）
    if (text_len >= MOUSE_V2T_MAX_CHARS && !mouse_v2t_locked)
    {
        mouse_v2t_locked = true;
        interact_mic_listen(false);
        mouse_v2t_set_active(false); // 同步 space btn 視覺
        LOG_D("V2T input >= %d chars, mic auto-closed and locked",
              MOUSE_V2T_MAX_CHARS);
    }
}

/**
 * @brief V2T 結果通知入口：仿 append_text_to_input_message 的方式，
 *        從 V2T 模組拿合併文字、透過 LVGL message queue 切到 LVGL thread 套用
 */
void append_text_to_mouse_input(void)
{
    extern char *get_combined_voice2text(void);
    char *text = get_combined_voice2text();
    if (text == NULL)
        return;
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MOUSE_INPUT_TEXT;
    msg.data.message = text;
    lvgl_send_msg(msg);
}

// static lv_obj_t *text_input_bar = NULL;
static rt_tick_t text_input_bar_pressing_time = NULL;
static rt_tick_t text_input_bar_press_time = NULL;
static uint16_t max_move_y = 0;
static uint8_t test_count = 0;
static float prev_elapsed = 0.0f;

// === skaibar 選項 helpers ====================================================

// 把目前選到的選項文字塞進 input bar（取代 V2T 流入的文字）
static void skaibar_apply_selection_to_input_bar(void)
{
    if (skaibar_selected_idx < 0 ||
        skaibar_selected_idx >= skaibar_options_count)
        return;
    const char *text = skaibar_options[skaibar_selected_idx];
    size_t len = strlen(text);
    if (len >= sizeof(input_buffer))
        len = sizeof(input_buffer) - 1;
    memset(input_buffer, 0, sizeof(input_buffer));
    memcpy(input_buffer, text, len);
    input_buffer[len] = '\0';
    input_length = (int)len;
    if (input_display_label != NULL)
        update_input_display();
}

// 弧形滾動觸發：每個 step = 一個節點 = 一筆切換
// 第一次滾動（idx==-1）：往哪邊都先跳到 idx=0；之後正常累加 / 累減 + clamp
static void skaibar_scroll_step(int steps)
{
    if (skaibar_options_count == 0) return;
    int16_t new_idx;
    if (skaibar_selected_idx < 0)
    {
        new_idx = 0; // 第一次滾就秀第一個
    }
    else
    {
        new_idx = skaibar_selected_idx + steps;
        if (new_idx < 0) new_idx = 0;
        if (new_idx >= skaibar_options_count)
            new_idx = skaibar_options_count - 1;
    }
    if (new_idx == skaibar_selected_idx) return; // 已在邊界，沒變
    skaibar_selected_idx = new_idx;
    skaibar_apply_selection_to_input_bar();
    commu_send_skaibar_selected((uint8_t)skaibar_selected_idx);
}

// PC 透過 KEY_SKAIBAR_OPTIONS 送 JSON 進來
// 期待格式：{"options":["opt1","opt2",...]} 或裸 array ["opt1","opt2",...]
// 解析後存到 skaibar_options[][]；若使用者選到的 idx 超過新列表大小，clamp 到末
void mouse_skaibar_set_options_json(const char *json)
{
    if (json == NULL) return;
    cJSON *root = cJSON_Parse(json);
    if (root == NULL)
    {
        LOG_W("skaibar: JSON parse failed");
        return;
    }
    cJSON *arr = NULL;
    if (cJSON_IsArray(root))
    {
        arr = root;
    }
    else
    {
        arr = cJSON_GetObjectItem(root, "options");
    }
    if (!cJSON_IsArray(arr))
    {
        LOG_W("skaibar: JSON has no array (key 'options' or bare array)");
        cJSON_Delete(root);
        return;
    }

    uint8_t n = 0;
    cJSON *item = NULL;
    cJSON_ArrayForEach(item, arr)
    {
        if (n >= SKAIBAR_MAX_OPTIONS) break;
        if (!cJSON_IsString(item)) continue;
        strncpy(skaibar_options[n], item->valuestring,
                SKAIBAR_MAX_OPTION_LEN - 1);
        skaibar_options[n][SKAIBAR_MAX_OPTION_LEN - 1] = '\0';
        n++;
    }
    skaibar_options_count = n;
    cJSON_Delete(root);

    // 若選到的 idx 已經超過新列表 → clamp + 同步 UI + 回報新 idx
    if (skaibar_selected_idx >= skaibar_options_count)
    {
        skaibar_selected_idx =
            skaibar_options_count > 0 ? skaibar_options_count - 1 : -1;
        if (skaibar_selected_idx >= 0)
        {
            skaibar_apply_selection_to_input_bar();
            commu_send_skaibar_selected((uint8_t)skaibar_selected_idx);
        }
    }
}

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

/* T4 (b): hosted-mode return hook. When device_pager hosts the mouse it
   registers a back callback; the bottom-bar up gesture then returns to the
   instruction layer instead of firing multitask. */
static bool s_hosted = false;
static void (*s_host_back_cb)(void) = NULL;
static void (*s_host_pull_cb)(int up_px, int released) = NULL;
void hid_mouse_set_host_back_cb(void (*cb)(void))
{
    s_host_back_cb = cb;
    s_hosted = (cb != NULL || s_host_pull_cb != NULL);
}
void hid_mouse_set_host_pull_cb(void (*cb)(int up_px, int released))
{
    s_host_pull_cb = cb;
    s_hosted = (cb != NULL || s_host_back_cb != NULL);
}

static void bottom_bar_multitask_fire(void)
{
    LOG_D("Bottom bar: multitask triggered");
    if (bottom_bar_multitask_fired)
        return;
    /* hosted: suppress the multitask send here (this can run on the rt_timer
       thread — no LVGL calls). The actual return is driven on the LVGL thread
       by the release handler below. */
    if (s_hosted && s_host_back_cb)
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
        // 重置長按 flag，等下一次 LV_EVENT_LONG_PRESSED 觸發
        bar_long_press_fired = false;
        // notify_provider.holding_displacement(0, 0, 0);
        break;

    case LV_EVENT_LONG_PRESSED:
        // 長按 bar → 跟短按 release 同樣的路徑：進 skaibar 模式 + 開 V2T。
        // 差別只在時機 —— 長按不用等使用者放開，LONG_PRESS 觸發就立刻進場
        bar_long_press_fired = true;
        skaibar_active = true;
        skaibar_selected_idx = -1;
        if (current_hid_mode != HID_MODE_KEYBOARD)
            start_trackpad_to_kbd_expand_anim();
        mouse_v2t_open_with_intent(V2T_INTENT_SKAIBAR);
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

                /* Hosted (device_pager): delegate the up-drag so the host
                   finger-follows pulling its instruction panel back into view.
                   Skip the mouse's own multitask hint entirely. */
                if (s_host_pull_cb)
                {
                    s_host_pull_cb(up_amount, 0);
                    break;
                }

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
                /* Hosted (device_pager): hand the release to the host, which
                   decides commit (restore the instruction list) vs cancel
                   (snap back to the mouse) from how far it was pulled up. No
                   multitask hint was shown, so nothing to animate away. */
                if (s_host_pull_cb)
                {
                    s_host_pull_cb(multitask_hint_drag_offset, 1);
                    break;
                }
                if (multitask_hint_drag_offset > MULTITASK_HINT_LIMIT)
                {
                    /* hosted (device_pager): bottom-bar up returns to the
                       instruction layer instead of sending multitask. Safe to
                       call here — this is the LVGL-thread release handler. */
                    if (s_hosted && s_host_back_cb)
                    {
                        s_host_back_cb();
                    }
                    else if (control_provider.ble_hid_keyboard_multitask)
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

            if (max_move_y < 10 && !bottom_bar_gesture_timer_enabled &&
                !is_bottom_bar_gesture_active && !bar_long_press_fired)
            {
                // 純點擊（沒明顯拖動、也沒長按）：跟長按一樣進 skaibar 模式 +
                // 自動開 V2T。差別只在進場時機（短按要等 release、長按 LONG_PRESS
                // 觸發就立刻進，給長按一點即時反饋的優勢）
                LOG_D("Bottom bar tap → expand to skaibar mode");
                skaibar_active = true;
                skaibar_selected_idx = -1;
                if (current_hid_mode != HID_MODE_KEYBOARD)
                    start_trackpad_to_kbd_expand_anim();
                mouse_v2t_open_with_intent(V2T_INTENT_SKAIBAR);
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

    // === 右側節點 mirror（弧 -45° ~ +45°，中心 0° 最亮）===
    if (right_scroll_nodes[0] == NULL) return;
    for (int i = 0; i < LEFT_SCROLL_NODE_COUNT; i++)
    {
        if (right_scroll_nodes[i] == NULL) continue;

        // 右弧 base 從 -45° 起，spacing 一樣 18°
        float base = -45.0f + spacing * (float)i;
        float angle_deg = base + scroll_node_offset_deg;
        angle_deg = fmodf(angle_deg - (-45.0f), span);
        if (angle_deg < 0.0f) angle_deg += span;
        angle_deg += -45.0f;

        float rad = angle_deg * LEFT_SCROLL_PI / 180.0f;
        int16_t px = (int16_t)(cx + mid_r * cosf(rad));
        int16_t py = (int16_t)(cy + mid_r * sinf(rad));

        // 越靠近 0°（右弧正中）越亮
        float t = angle_deg / (span * 0.5f); // [-1, 1)
        float factor = cosf(t * LEFT_SCROLL_PI * 0.5f);
        factor = factor * factor;

        float tick_len = 20.0f * size_scale;
        int16_t line_w = (int16_t)(6.0f * size_scale);
        if (tick_len < 2.0f) tick_len = 2.0f;
        if (line_w < 1) line_w = 1;

        float ux = cosf(rad);
        float uy = sinf(rad);
        int16_t x0 = (int16_t)(px + ux * tick_len * 0.5f);
        int16_t y0 = (int16_t)(py + uy * tick_len * 0.5f);
        int16_t x1 = (int16_t)(px - ux * tick_len * 0.5f);
        int16_t y1 = (int16_t)(py - uy * tick_len * 0.5f);
        right_scroll_node_pts[i][0].x = x0;
        right_scroll_node_pts[i][0].y = y0;
        right_scroll_node_pts[i][1].x = x1;
        right_scroll_node_pts[i][1].y = y1;

        float blend = 0.2f + 0.8f * factor;
        if (blend < 0.0f) blend = 0.0f;
        if (blend > 1.0f) blend = 1.0f;
        uint8_t cv = (uint8_t)((float)0x4D * blend);
        lv_color_t node_color = lv_color_make(cv, cv, cv);

        lv_line_set_points(right_scroll_nodes[i], right_scroll_node_pts[i], 2);
        lv_obj_set_style_line_width(right_scroll_nodes[i], line_w, 0);
        lv_obj_set_style_line_color(right_scroll_nodes[i], node_color, 0);
    }
}

/**
 * @brief 依當前 scroll_ui_level（0 暗/細 ~ 1000 亮/粗）套用 arc 與節點樣式
 */
static void apply_scroll_ui_level(void)
{
    int32_t opa_v = (int32_t)LEFT_SCROLL_ARC_OPA_DIM +
                    ((int32_t)LEFT_SCROLL_ARC_OPA_ACTIVE -
                     (int32_t)LEFT_SCROLL_ARC_OPA_DIM) *
                        scroll_ui_level / 1000;
    if (opa_v < 0) opa_v = 0;
    if (opa_v > 255) opa_v = 255;
    if (left_scroll_bar != NULL)
    {
        lv_obj_set_style_arc_width(left_scroll_bar,
                                   LEFT_SCROLL_ARC_W_ACTIVE, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(left_scroll_bar, opa_v, LV_PART_MAIN);
    }
    if (right_scroll_bar != NULL)
    {
        lv_obj_set_style_arc_width(right_scroll_bar,
                                   LEFT_SCROLL_ARC_W_ACTIVE, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(right_scroll_bar, opa_v, LV_PART_MAIN);
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
    // status_bar_area_up 同理：480×80 的頂部下拉感應區跟 input_enter_btn
    // (y=10 h=45) 完全重疊，且 status_bar_area_up 是 bg 直接子物件 → z-order
    // 在 mode_container 之上，Enter 的 press 永遠先被它吃掉
    if (mode == HID_MODE_KEYBOARD && status_bar_area_up &&
        lv_obj_is_valid(status_bar_area_up))
    {
        if (visible)
            lv_obj_add_flag(status_bar_area_up, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(status_bar_area_up, LV_OBJ_FLAG_HIDDEN);
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
    // 同時建立左/右兩側的節點，parent = bg（跨 mode 共用）
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
        right_scroll_nodes[i] = line;
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
    // 左/右兩側滾動弧形條改放在 bg（parent 的 parent），跨 mode 共用
    // 不再跟著 mode_container[TRACKPAD] 隱藏；keyboard mode mic view 也看得到
    lv_obj_t *arc_parent = lv_obj_get_parent(parent);

    // 左側滾動弧形條（貼著圓形畫面左側邊緣）
    left_scroll_bar = (lv_obj_t *)lv_arc_create(arc_parent);
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

    // 右側滾動弧形條（mirror，角度 -45°~+45° = 315°~405°）
    right_scroll_bar = (lv_obj_t *)lv_arc_create(arc_parent);
    lv_obj_set_size(right_scroll_bar, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(right_scroll_bar, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation((lv_obj_t *)right_scroll_bar, 0);
    lv_arc_set_bg_angles((lv_obj_t *)right_scroll_bar, 315, 405);
    lv_arc_set_angles((lv_obj_t *)right_scroll_bar, 0, 0);
    lv_arc_set_mode((lv_obj_t *)right_scroll_bar, LV_ARC_MODE_NORMAL);
    lv_obj_set_style_pad_all(right_scroll_bar, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(right_scroll_bar, LV_OPA_0, LV_PART_KNOB);
    lv_obj_set_style_arc_width(right_scroll_bar, 30, LV_PART_MAIN);
    lv_obj_set_style_arc_color(right_scroll_bar, lv_color_hex(0x333333),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_opa(right_scroll_bar, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_arc_width(right_scroll_bar, 0, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(right_scroll_bar, LV_OPA_0, LV_PART_INDICATOR);
    lv_obj_clear_flag(right_scroll_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(right_scroll_bar, LV_OBJ_FLAG_CLICKABLE);

    // 弧線上的節點指示點（左+右兩組都在 bg 內）
    create_left_scroll_nodes(arc_parent);

    // === Trackpad mode 下方按鈕：通用清單選擇器入口（取代原本的麥克風）===
    // 點擊跟拖動都由下方擴大版的 bottom_swipe_area 統一處理：
    //   點擊 → 進入 keyboard mode（展開輸入介面）；拖向上 → multitask hint
    // bar 樣式：270×16 圓角橫條，看起來像 home indicator / pill
    trackpad_mic_btn = lv_obj_create(parent);
    lv_obj_set_size(trackpad_mic_btn, 100, 16);
    lv_obj_align(trackpad_mic_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    // dark style 跟 keyboard mode 的 text_input_bar_bg 一致；collapse 縮到
    // 同位置時兩者外觀相同 → 切換 mode 看起來像同一條 bar 沒有色差跳動
    lv_obj_set_style_bg_color(trackpad_mic_btn, lv_color_hex(0x1a1a1a),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(trackpad_mic_btn, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_border_color(trackpad_mic_btn, lv_color_hex(0xFFFFFF),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(trackpad_mic_btn, 2, LV_PART_MAIN);
    lv_obj_set_style_border_opa(trackpad_mic_btn, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_radius(trackpad_mic_btn, 8, LV_PART_MAIN);
    lv_obj_clear_flag(trackpad_mic_btn, LV_OBJ_FLAG_SCROLLABLE);
    // 不 CLICKABLE：事件穿透到下層 bottom_swipe_area 統一處理
    lv_obj_clear_flag(trackpad_mic_btn, LV_OBJ_FLAG_CLICKABLE);

    // v2t_mic_img / trackpad_mic_icon 保留變數但不放任何視覺內容
    // （bar 本身就是視覺）
    v2t_mic_img = trackpad_mic_btn;
    trackpad_mic_icon = trackpad_mic_btn;

    // 保留 mic_red_dot 物件以維持原有 V2T pipeline 引用，但 trackpad mode
    // 不再走 V2T → 始終 hidden
    trackpad_mic_red_dot = lv_obj_create(trackpad_mic_btn);
    lv_obj_remove_style_all(trackpad_mic_red_dot);
    lv_obj_set_size(trackpad_mic_red_dot, 50, 50);
    lv_obj_align(trackpad_mic_red_dot, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(trackpad_mic_red_dot, lv_color_hex(0x4A83FF), 0);
    lv_obj_set_style_bg_opa(trackpad_mic_red_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(trackpad_mic_red_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(trackpad_mic_red_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(trackpad_mic_red_dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(trackpad_mic_red_dot, LV_OBJ_FLAG_HIDDEN);

    trackpad_mic_red_dot_x = lv_label_create(trackpad_mic_red_dot);
    lv_label_set_text(trackpad_mic_red_dot_x, "確定");
    lv_obj_set_style_text_color(trackpad_mic_red_dot_x, lv_color_hex(0xFFFFFF),
                                0);
    lv_obj_align(trackpad_mic_red_dot_x, LV_ALIGN_CENTER, 0, 0);

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
// mode_container HIT_TEST：點在左/右弧形滾動區或中間延伸觸發區就拒絕命中，
// 讓 press 落到下方的 touch_bg 走 arc 滾動邏輯
// （否則 mode_container CLICKABLE 會吃掉所有觸控 → 滾動失效 + release 誤觸退出）
static void keyboard_mode_container_hit_test_cb(lv_event_t *e)
{
    lv_hit_test_info_t *info = lv_event_get_hit_test_info(e);
    if (info->res == false) return;
    if (is_point_in_left_arc(info->point) ||
        is_point_in_right_arc(info->point) ||
        is_point_in_center_scroll_zone(info->point))
        info->res = false;
}

static void create_keyboard_mode_ui(lv_obj_t *parent)
{
    // mode_container 變成 CLICKABLE，接收沒落在任何 child 的點擊事件 →
    // 觸發 keyboard_mode_outside_click_cb 退出輸入模式
    // HIT_TEST cb 排除弧形滾動區，讓滾動條照常運作
    //   ADV_HITTEST flag 是 LVGL 啟動 LV_EVENT_HIT_TEST 派發的前置條件，
    //   沒設這個 flag → HIT_TEST cb 永遠不被呼叫 → 滾動區全被 mode_container 吃掉
    lv_obj_add_flag(parent, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(parent, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_add_event_cb(parent, keyboard_mode_outside_click_cb,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(parent, keyboard_mode_container_hit_test_cb,
                        LV_EVENT_HIT_TEST, NULL);

    // Input bar 容器（深色框，keyboard mode 顯示在鍵盤上方）
    text_input_bar_bg = lv_obj_create(parent);
    // 高度跟 input_enter_btn (45) 一致；頂部對齊 y=110（466 圓內 Enter btn 最高位置）
    lv_obj_set_size(text_input_bar_bg, 280, 45);
    lv_obj_set_pos(text_input_bar_bg, (LV_HOR_RES_MAX - 280) / 2, 110);
    // 視覺樣式跟 trackpad_mic_btn 一致（白色半透明 pill），讓
    // tap bar → 展開動畫看起來是同一條 bar 在長大
    lv_obj_set_style_bg_color(text_input_bar_bg, lv_color_hex(0xFFFFFF),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_border_width(text_input_bar_bg, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(text_input_bar_bg, 100, LV_PART_MAIN);
    lv_obj_set_style_pad_all(text_input_bar_bg, 5, LV_PART_MAIN);
    lv_obj_clear_flag(text_input_bar_bg, LV_OBJ_FLAG_SCROLLABLE);
    // 接收 PRESSED/PRESSING/RELEASED 來偵測「往下拖收回 trackpad mode」
    lv_obj_add_flag(text_input_bar_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(text_input_bar_bg, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(text_input_bar_bg, text_input_bar_drag_event_cb,
                        LV_EVENT_ALL, NULL);

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

    // Input content container — 留出右側 80px 給內嵌 enter btn
    input_content_container = lv_obj_create(text_input_bar_bg);
    lv_obj_set_size(input_content_container, 210, 40);
    lv_obj_align(input_content_container, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_set_style_bg_opa(input_content_container, LV_OPA_TRANSP,
                            LV_PART_MAIN);
    lv_obj_set_style_border_width(input_content_container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(input_content_container, LV_OBJ_FLAG_SCROLLABLE);
    // 不要吃掉 press 事件：讓 text_input_bar_bg 收到 PRESSED 來偵測往下拖
    lv_obj_clear_flag(input_content_container, LV_OBJ_FLAG_CLICKABLE);
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
    // parent = input_content_container（跟 label 同一個），不能掛在 text_input_bar_bg：
    //   bar 在 expand anim 期間會從 280×45 長到 300×90，container 跟 label 都用
    //   LEFT_MID align 自動跟 bar 高度走，但 align_to 是 one-shot，cursor 掛在
    //   bar 上會被釘在「創建當下的螢幕位置」不會跟著 label 走 → 偏到頂部
    input_cursor = lv_obj_create(input_content_container);
    lv_obj_set_size(input_cursor, 2, 25);
    lv_obj_set_style_bg_color(input_cursor, lv_color_hex(0x4a90e2),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(input_cursor, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(input_cursor, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(input_cursor, 1, LV_PART_MAIN);
    lv_obj_align_to(input_cursor, input_display_label, LV_ALIGN_OUT_RIGHT_MID,
                    2, 0);
    lv_obj_add_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);

    // Enter button - 內嵌於 text_input_bar_bg 右側（看起來像 input bar 的一部分）
    // parent=text_input_bar_bg → 自動跟 bar 一起被 expand_anim_driver_cb 移動
    // 沒有背景圓圈，只有 enter icon 跟一個透明的點擊熱區
    input_enter_btn = lv_obj_create(text_input_bar_bg);
    lv_obj_set_size(input_enter_btn, 70, 70);
    lv_obj_align(input_enter_btn, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_opa(input_enter_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(input_enter_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(input_enter_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(input_enter_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(input_enter_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(input_enter_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(input_enter_btn, input_enter_btn_event_cb,
                        LV_EVENT_CLICKED, NULL);
    input_enter_img = lv_img_create(input_enter_btn);
    lv_img_set_src(input_enter_img, &enter_icon);
    lv_obj_center(input_enter_img);
    // 預設 zoom = 256 (100%)，由 expand_anim_driver_cb 在動畫期間調整
    lv_img_set_zoom(input_enter_img, 256);

    // 鍵盤 layout（內部創建 keyboard_container, custom_keyboard, 所有按鍵）
    create_circular_keyboard_layout(parent);
    if (keyboard_container)
    {
        // 預設藏起鍵盤，由下方 mic 區的右箭頭/左滑切換顯示
        lv_obj_set_style_translate_y(keyboard_container, 0, 0);
        lv_obj_align(keyboard_container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
    }
    if (custom_keyboard)
        lv_obj_clear_flag(custom_keyboard, LV_OBJ_FLAG_HIDDEN);

    // 下半部 mic 區（mic + < > 箭頭），預設顯示
    create_kbd_mic_section(parent);
    kbd_lower_is_keyboard = false; // 預設 mic 區
    // 滾輪 arc 不再在 keyboard mode 內建；改用 trackpad mode 的 left/right
    // arcs 直接放在 bg，跨 mode 共用

    // ★ 把 input bar 拉到最上層（Enter btn 是其 child 會跟著）
    //   原本創建順序：text_input_bar_bg → ... → keyboard_container → kbd_mic_section
    //   z-order 上 kbd_mic_section 在最上面 → mic view 時把整個輸入框區域擋住
    //   foreground 後 bar 在最上層，touch 直接落到輸入框 → 可以下拖收回
    if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
        lv_obj_move_foreground(text_input_bar_bg);
}

// =====================================================================
// Keyboard mode 下半部 mic 區實作
// =====================================================================
//   - 預設顯示在鍵盤位置（取代 keyboard_container 的可見性）
//   - 內含：左箭頭、麥克風按鈕（點擊 toggle V2T）、右箭頭
//   - 點右箭頭 / 左滑 → 切換成 keyboard
//   - 點左箭頭 / 右滑 → 切回 mic 區

static void create_kbd_mic_section(lv_obj_t *parent)
{
    // section 380×480 居中，左右各留 50px 給左/右弧形 scroll bar 使用
    // （CLICKABLE 全覆蓋會擋住下層 touch_bg 對弧形觸碰的偵測）
    kbd_mic_section = lv_obj_create(parent);
    lv_obj_remove_style_all(kbd_mic_section);
    lv_obj_set_size(kbd_mic_section, 380, LV_VER_RES_MAX);
    lv_obj_align(kbd_mic_section, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(kbd_mic_section, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_SCROLLABLE);
    // 不 CLICKABLE：空白區域的點擊要落到 mode_container 觸發 click-outside 退出
    lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_CLICKABLE);

    kbd_mic_section_mic_btn = lv_obj_create(kbd_mic_section);
    lv_obj_set_size(kbd_mic_section_mic_btn, 130, 130);
    lv_obj_align(kbd_mic_section_mic_btn, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style_bg_color(kbd_mic_section_mic_btn,
                              lv_color_hex(0x1a1a1a), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(kbd_mic_section_mic_btn, LV_OPA_COVER,
                            LV_PART_MAIN);
    lv_obj_set_style_radius(kbd_mic_section_mic_btn, LV_RADIUS_CIRCLE,
                            LV_PART_MAIN);
    lv_obj_set_style_border_width(kbd_mic_section_mic_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(kbd_mic_section_mic_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(kbd_mic_section_mic_btn, LV_OBJ_FLAG_CLICKABLE);
    // EVENT_BUBBLE：拖曳事件冒泡給 kbd_mic_section 的 swipe handler
    // PRESS_LOCK：拖出 btn 範圍時 act_obj 不轉移，press session 才會連貫
    lv_obj_add_flag(kbd_mic_section_mic_btn, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(kbd_mic_section_mic_btn, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(kbd_mic_section_mic_btn, kbd_mic_btn_event_cb,
                        LV_EVENT_CLICKED, NULL);
    // 新版需求：不再顯示中央大 mic 按鈕，V2T active 改由 input bar 邊框閃爍提示
    lv_obj_add_flag(kbd_mic_section_mic_btn, LV_OBJ_FLAG_HIDDEN);

    kbd_mic_section_mic_img = lv_img_create(kbd_mic_section_mic_btn);
    lv_img_set_src(kbd_mic_section_mic_img, &micro_icon);
    lv_obj_center(kbd_mic_section_mic_img);

    // V2T 脈衝圓：放在 mic icon 之後 → z-order 在 icon 上面，這樣才看得到。
    // opa 隨 size 漸減（COVER→TRANSP），中間幀是半透明，icon 還是看得到。
    // 初始就給 valid size + hidden（從 0×0 起步 LVGL 可能 skip render）；
    // V2T active 時開動畫 0→40 + opa COVER→TRANSP 循環
    kbd_mic_section_mic_pulse = lv_obj_create(kbd_mic_section_mic_btn);
    lv_obj_remove_style_all(kbd_mic_section_mic_pulse);
    lv_obj_set_size(kbd_mic_section_mic_pulse,
                    KBD_MIC_PULSE_MAX_SIZE, KBD_MIC_PULSE_MAX_SIZE);
    lv_obj_center(kbd_mic_section_mic_pulse); // align CENTER：size 變動會自動 re-center
    lv_obj_set_style_bg_color(kbd_mic_section_mic_pulse,
                              lv_color_hex(KBD_MIC_PULSE_COLOR), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(kbd_mic_section_mic_pulse, LV_OPA_COVER,
                            LV_PART_MAIN);
    lv_obj_set_style_radius(kbd_mic_section_mic_pulse, LV_RADIUS_CIRCLE,
                            LV_PART_MAIN);
    lv_obj_set_style_border_width(kbd_mic_section_mic_pulse, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(kbd_mic_section_mic_pulse, 0, LV_PART_MAIN);
    lv_obj_clear_flag(kbd_mic_section_mic_pulse, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(kbd_mic_section_mic_pulse, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(kbd_mic_section_mic_pulse, LV_OBJ_FLAG_HIDDEN);

    // 底部置中鍵盤按鈕：mic view 顯示，點擊切換到 keyboard view（鍵盤升起）
    // （左/右滑切回 mic view 靠右滑手勢，這裡只放底部一顆按鈕）
    kbd_mic_section_right_arrow = lv_obj_create(kbd_mic_section);
    lv_obj_remove_style_all(kbd_mic_section_right_arrow);
    lv_obj_set_size(kbd_mic_section_right_arrow, 50, 50);
    lv_obj_align(kbd_mic_section_right_arrow, LV_ALIGN_BOTTOM_MID, 0, -50);
    lv_obj_set_style_bg_color(kbd_mic_section_right_arrow,
                              lv_color_hex(0x333333), 0);
    lv_obj_set_style_bg_opa(kbd_mic_section_right_arrow, LV_OPA_60, 0);
    lv_obj_set_style_radius(kbd_mic_section_right_arrow, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(kbd_mic_section_right_arrow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(kbd_mic_section_right_arrow, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(kbd_mic_section_right_arrow,
                        kbd_lower_arrow_event_cb, LV_EVENT_CLICKED,
                        (void *)(intptr_t)1); // 1 = 切到 keyboard
    lv_obj_t *right_arrow_img = lv_img_create(kbd_mic_section_right_arrow);
    lv_img_set_src(right_arrow_img, &keyboard_icon);
    lv_img_set_zoom(right_arrow_img, 150); // 放大一點點
    lv_obj_center(right_arrow_img);

    // 初始狀態：mic view → 顯示鍵盤按鈕
    kbd_lower_update_arrows_visibility();
}

static void kbd_lower_update_arrows_visibility(void)
{
    // mic view 才顯示鍵盤按鈕；keyboard view 隱藏（回 mic 靠右滑手勢）
    if (kbd_mic_section_right_arrow &&
        lv_obj_is_valid(kbd_mic_section_right_arrow))
    {
        if (kbd_lower_is_keyboard)
            lv_obj_add_flag(kbd_mic_section_right_arrow,
                            LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(kbd_mic_section_right_arrow,
                              LV_OBJ_FLAG_HIDDEN);
    }
}

static void kbd_lower_set_keyboard(bool show_kbd)
{
    kbd_lower_is_keyboard = show_kbd;
    if (show_kbd)
    {
        if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
            lv_obj_add_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
            lv_obj_clear_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
            lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
        if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
            lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
    }
    kbd_lower_update_arrows_visibility();
    kbd_lower_update_arcs_visibility();
}

// 依當前 sub-view 切換兩側弧形 scroll bars 的顯示
// 鍵盤 view → 隱藏（避免跟鍵盤邊緣重疊）
// mic view / trackpad mode → 顯示
static void kbd_lower_update_arcs_visibility(void)
{
    bool hide = kbd_lower_is_keyboard;
    if (left_scroll_bar && lv_obj_is_valid(left_scroll_bar))
    {
        if (hide) lv_obj_add_flag(left_scroll_bar, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_clear_flag(left_scroll_bar, LV_OBJ_FLAG_HIDDEN);
    }
    if (right_scroll_bar && lv_obj_is_valid(right_scroll_bar))
    {
        if (hide) lv_obj_add_flag(right_scroll_bar, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_clear_flag(right_scroll_bar, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < LEFT_SCROLL_NODE_COUNT; i++)
    {
        if (left_scroll_nodes[i] && lv_obj_is_valid(left_scroll_nodes[i]))
        {
            if (hide) lv_obj_add_flag(left_scroll_nodes[i], LV_OBJ_FLAG_HIDDEN);
            else      lv_obj_clear_flag(left_scroll_nodes[i], LV_OBJ_FLAG_HIDDEN);
        }
        if (right_scroll_nodes[i] && lv_obj_is_valid(right_scroll_nodes[i]))
        {
            if (hide) lv_obj_add_flag(right_scroll_nodes[i], LV_OBJ_FLAG_HIDDEN);
            else      lv_obj_clear_flag(right_scroll_nodes[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

// keyboard mode 空白區域點擊 → 收回成 trackpad mode
// LVGL event bubble：點 input bar / enter btn / 鍵盤鍵 / 弧形 scroll bar 都是
// child 自己消化掉，container 收不到 → 只有點落在空白區才觸發
static void keyboard_mode_outside_click_cb(lv_event_t *e)
{
    // target == current_target 表示點擊直接落在 mode_container 上（沒被 child 吃）
    if (lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    // 還在動畫中（mode_swipe 或 collapse）不重複觸發
    //   注意：current_hid_mode 在 collapse done_cb 才會從 KEYBOARD 翻成
    //   TRACKPAD，動畫期間單看 current_hid_mode 擋不住，必須另外看 anim flag
    if (current_hid_mode != HID_MODE_KEYBOARD) return;
    if (collapse_anim_running) return;
    // V2T 還在錄就先停掉再 collapse（跟 down_arrow 路徑一致）
    if (mouse_v2t_active)
        mouse_v2t_close_and_paste();
    // keyboard view 時 bar 在 y=75；先 snap 回 mic 視覺狀態，
    // 否則 collapse 動畫起點 y=195（EXPAND_END_Y）會跟 bar 實際 y=75 不一致 → 跳一下
    if (kbd_lower_is_keyboard)
    {
        if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
        {
            lv_anim_del(text_input_bar_bg, (lv_anim_exec_xcb_t)lv_obj_set_y);
            lv_obj_set_y(text_input_bar_bg, EXPAND_END_Y);
        }
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
        {
            lv_anim_del(keyboard_container, NULL);
            lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_translate_y(keyboard_container, 0, 0);
        }
        if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
            lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
        kbd_lower_is_keyboard = false;
        // 立即重算 arc 可見性：keyboard view 時被 hide 的 left/right scroll bar
        // 要在 collapse 動畫開始前先顯示出來，否則使用者會看到「輸入框縮回去
        // 後弧線才突然冒出來」（done_cb 才會 update_arcs_visibility 是太晚）
        kbd_lower_update_arcs_visibility();
    }
    start_kbd_to_trackpad_collapse_anim(100, true);
}

static void kbd_lower_arrow_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    int arrow = (int)(intptr_t)lv_event_get_user_data(e);
    // 0 = 左箭頭 → 切回 mic（從 keyboard 回 mic）
    // 1 = 右箭頭 / 底部鍵盤鈕 → 切到 keyboard
    bool to_kbd = (arrow == 1);
    if (to_kbd && mouse_v2t_active)
        mouse_v2t_close_and_paste();

    if (to_kbd)
    {
        // mic → keyboard：
        //   1. mic section 直接 hide
        //   2. input bar y 從 195 → 75 (上移到鍵盤上方)
        //   3. keyboard 從下方升起 translate_y 300 → 0
        if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
        {
            lv_obj_set_style_translate_x(kbd_mic_section, 0, 0);
            lv_obj_add_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
        }
        // 進場：input bar 上移 + keyboard 升起都用 overshoot 帶彈性
        // 退場（else 分支）維持 ease_out 比較乾脆
        if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
        {
            lv_anim_del(text_input_bar_bg, (lv_anim_exec_xcb_t)lv_obj_set_y);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, text_input_bar_bg);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
            lv_anim_set_values(&a, EXPAND_END_Y, EXPAND_END_Y_KBD);
            lv_anim_set_time(&a, 350);
            lv_anim_set_path_cb(&a, lv_anim_path_overshoot);
            lv_anim_start(&a);
        }
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
        {
            lv_obj_clear_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_translate_x(keyboard_container, 0, 0);
            lv_obj_set_style_translate_y(keyboard_container, 300, 0);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, keyboard_container);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_set_translate_y);
            lv_anim_set_values(&a, 300, 0);
            lv_anim_set_time(&a, 350);
            lv_anim_set_path_cb(&a, lv_anim_path_overshoot);
            lv_anim_start(&a);
        }
        kbd_lower_is_keyboard = true;
        kbd_lower_update_arrows_visibility();
        kbd_lower_update_arcs_visibility();
    }
    else
    {
        // keyboard → mic：
        //   1. keyboard 往下滑出 (translate_y 0 → 300) 後 hide
        //   2. input bar y 從 75 → 195 (回到螢幕中央)
        //   3. 顯示 mic section
        if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
        {
            lv_anim_del(text_input_bar_bg, (lv_anim_exec_xcb_t)lv_obj_set_y);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, text_input_bar_bg);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
            lv_anim_set_values(&a, EXPAND_END_Y_KBD, EXPAND_END_Y);
            lv_anim_set_time(&a, 250);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_start(&a);
        }
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
        {
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, keyboard_container);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_set_translate_y);
            lv_anim_set_values(&a, 0, 300);
            lv_anim_set_time(&a, 250);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_set_ready_cb(&a, keyboard_hide_after_slide_down_cb);
            lv_anim_start(&a);
        }
        if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
        {
            lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_translate_x(kbd_mic_section, 0, 0);
        }
        kbd_lower_is_keyboard = false;
        kbd_lower_update_arrows_visibility();
        kbd_lower_update_arcs_visibility();
    }
}

// =====================================================================
// Trackpad bar → keyboard mode 展開動畫
// =====================================================================
//   單一 driver anim (0..100) 同步推進四件事：
//     - text_input_bar_bg 從 trackpad bar 的位置/尺寸 → 輸入框最終位置/尺寸
//     - kbd_mic_section / 左右箭頭 / 右弧節點 / 右弧 touch 的 translate_y
//       從 +200（off-screen 下方）→ 0（最終位置）
//   動畫結束後 hide trackpad mode container，trackpad bar 恢復 unhide
//   讓回到 trackpad mode 時 bar 還在
    #define EXPAND_ANIM_TIME_MS 250
// 起點 = trackpad_mic_btn 的實際螢幕座標，在 expand 啟動時 snapshot
//   理論值是 (190, 444, 100, 16)，但靠 lv_obj_get_coords 直接抓更可靠：
//   align/set_pos 計算 + theme padding 可能產生 1~2 px 偏差，導致 collapse
//   終點跟 trackpad bar 對不齊（visual flicker on commit）
    #define EXPAND_FALLBACK_X 190
    #define EXPAND_FALLBACK_Y 444
    #define EXPAND_FALLBACK_W 100
    #define EXPAND_FALLBACK_H 16
static lv_coord_t expand_start_x = EXPAND_FALLBACK_X;
static lv_coord_t expand_start_y = EXPAND_FALLBACK_Y;
static lv_coord_t expand_start_w = EXPAND_FALLBACK_W;
static lv_coord_t expand_start_h = EXPAND_FALLBACK_H;
// 終點 = text_input_bar_bg 的 set_pos / set_size 預設值
// （y=110, h=45 跟右邊的 input_enter_btn 對齊頂底；466 圓內 Enter 最高位置）
    #define EXPAND_TRANSLATE_START 320 // 下方元件起始 translate_y（mic 中心 240 + 半徑 65 ≈ 305 → 320 確保完全 off-screen）

static inline lv_coord_t expand_lerp(int32_t a, int32_t b, int32_t v_x100)
{
    return (lv_coord_t)(a + (b - a) * v_x100 / 100);
}

static void expand_anim_driver_cb(void *var, int32_t v)
{
    (void)var;
    // input bar 跟 trackpad bar 位置/尺寸間 lerp
    if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
    {
        lv_obj_set_pos(text_input_bar_bg,
                       expand_lerp(expand_start_x, EXPAND_END_X, v),
                       expand_lerp(expand_start_y, EXPAND_END_Y, v));
        lv_obj_set_size(text_input_bar_bg,
                        expand_lerp(expand_start_w, EXPAND_END_W, v),
                        expand_lerp(expand_start_h, EXPAND_END_H, v));
    }
    // 下方元件 translate_y 從 +EXPAND_TRANSLATE_START → 0
    //   mic view 時可見的是 kbd_mic_section
    //   keyboard view 時可見的是 keyboard_container
    //   兩個都跟著 ty 移動，hidden 的那個沒視覺影響但保留同步以便 collapse 用
    lv_coord_t ty = (lv_coord_t)(EXPAND_TRANSLATE_START * (100 - v) / 100);
    if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
        lv_obj_set_style_translate_y(kbd_mic_section, ty, 0);
    if (keyboard_container && lv_obj_is_valid(keyboard_container))
        lv_obj_set_style_translate_y(keyboard_container, ty, 0);
    // input_enter_btn 是 text_input_bar_bg 的 child（內嵌右側）：
    //   - btn 自己 bbox = 70×70（align RIGHT_MID 跟著 parent size 自動定位）
    //   - 視覺縮放：用 lv_img_set_zoom 直接縮 enter icon img（transform_zoom 套在
    //     parent obj 在這個 LVGL build 不會 cascade 到 child img）
    //   - 透明度：lv_obj_set_style_img_opa 設 img 本身的繪製透明度
    // overshoot path 會讓 v 暫時超過 100，opa 必須 clamp，否則 uint8_t cast wrap
    // 成接近 0 → enter icon 在彈跳峰值瞬間閃成透明
    if (input_enter_img && lv_obj_is_valid(input_enter_img))
    {
        int32_t v_opa = v > 100 ? 100 : (v < 0 ? 0 : v);
        uint16_t zoom = (uint16_t)(256 * v / 100);
        if (zoom < 1) zoom = 1;
        lv_img_set_zoom(input_enter_img, zoom);
        lv_obj_set_style_img_opa(input_enter_img,
                                 (lv_opa_t)(LV_OPA_COVER * v_opa / 100),
                                 LV_PART_MAIN);
    }
    // 箭頭現在是 section 的子物件，自動跟 translate_y，不用單獨處理
}

static void expand_anim_done_cb(lv_anim_t *a)
{
    (void)a;
    // 完成後正式收掉 trackpad mode
    mode_set_visible(HID_MODE_TRACKPAD, false);
    // bar 還原 unhide 讓下次回 trackpad 時看得到
    if (trackpad_mic_btn && lv_obj_is_valid(trackpad_mic_btn))
        lv_obj_clear_flag(trackpad_mic_btn, LV_OBJ_FLAG_HIDDEN);
}

static void start_trackpad_to_kbd_expand_anim(void)
{
    if (current_hid_mode == HID_MODE_KEYBOARD) return;

    // 0. snapshot trackpad bar 實際螢幕座標當 expand 起點
    //    必須在 hide 之前抓（hide 後 coords 仍 valid，但顯式 layout 才保險）
    if (trackpad_mic_btn && lv_obj_is_valid(trackpad_mic_btn))
    {
        lv_obj_update_layout(trackpad_mic_btn);
        lv_area_t c;
        lv_obj_get_coords(trackpad_mic_btn, &c);
        // text_input_bar_bg 的 parent (mode_container[KEYBOARD]) 也在 (0,0)
        // 且 pad=0，所以 c.x1/y1 直接當 set_pos 值即可
        expand_start_x = c.x1;
        expand_start_y = c.y1;
        expand_start_w = c.x2 - c.x1 + 1;
        expand_start_h = c.y2 - c.y1 + 1;
    }

    // 1. trackpad bar 視覺隱藏（輸入框接手繼續顯示「同一條」感）
    if (trackpad_mic_btn && lv_obj_is_valid(trackpad_mic_btn))
        lv_obj_add_flag(trackpad_mic_btn, LV_OBJ_FLAG_HIDDEN);

    // 2. 顯示 keyboard mode container（讓 input bar 等可被渲染）
    mode_set_visible(HID_MODE_KEYBOARD, true);
    current_hid_mode = HID_MODE_KEYBOARD;
    keyboard_visible = true;

    // 2.5 強制把 sub-view 設成 mic：每次 expand 都從語音輸入開始，
    //     即使先前 collapse 時還在 keyboard view（或被別處改過 state）
    kbd_lower_is_keyboard = false;
    if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
    {
        lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_x(kbd_mic_section, 0, 0);
    }
    if (keyboard_container && lv_obj_is_valid(keyboard_container))
    {
        lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_x(keyboard_container, 0, 0);
    }
    kbd_lower_update_arrows_visibility();


    // 3. 強制把 input bar 設到起點（trackpad bar 同位置/同尺寸），
    //    避免動畫第一幀視覺跳動
    expand_anim_driver_cb(NULL, 0);

    // 4. 啟動 driver anim 0 → 100
    //    overshoot path：bar 會先衝到目標位置/尺寸的「外側一點點」再彈回，
    //    視覺上像有彈性而非單調 ease；time 拉到 350ms 給彈跳一點呼吸空間
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, text_input_bar_bg); // var 不重要，cb 內不使用
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, 350);
    lv_anim_set_path_cb(&a, lv_anim_path_overshoot);
    lv_anim_set_exec_cb(&a, expand_anim_driver_cb);
    lv_anim_set_ready_cb(&a, expand_anim_done_cb);
    lv_anim_start(&a);
}

// =====================================================================
// Keyboard mode → trackpad mode 收回動畫（input bar 往下拖手勢觸發）
// =====================================================================
//   - 反向 reuse expand_anim_driver_cb：v 100→0 把 input bar 收回到
//     trackpad bar 的位置/尺寸，同時 kbd_mic_section translate_y 往下退場
//   - PRESSING 期間：跟手用 v = 100 - dy*100/RANGE
//   - RELEASED：dy >= COMMIT_PX → commit 收回；否則 snap 回 keyboard
    #define TIB_DRAG_ENGAGE_PX 12  // 過此 threshold 才視為「進入下拉」
    #define TIB_DRAG_COMMIT_PX 40  // 過此 threshold 釋放 → 真的收回
    #define TIB_DRAG_RANGE_PX 180  // dy = RANGE 時 progress = 0（完全收回）

static int16_t tib_drag_start_y = 0;
static int16_t tib_drag_start_x = 0;
static int16_t tib_drag_last_dy = 0; // 在 PRESSING 持續更新，RELEASED 直接讀
static bool tib_drag_tracking = false;
static bool tib_drag_engaged = false;
static bool tib_drag_rejected = false; // 偵測到水平拖曳 → 放棄這次 session
static bool collapse_anim_commit = false;

static void collapse_anim_done_cb(lv_anim_t *a)
{
    (void)a;
    collapse_anim_running = false;
    if (collapse_anim_commit)
    {
        // 切換到 trackpad mode：
        //   1. V2T 若在錄 → 收掉
        //   2. trackpad_mic_btn 取消 hidden（之前 expand 時被 hide）
        //   3. mode container 切過去；mode_set_visible(KEYBOARD, false) 內部
        //      會重設 text_input_bar_bg 樣式/位置回 closed 狀態
        //   4. kbd_mic_section 還原 translate_y / 視 visibility 預設
        if (mouse_v2t_active)
            mouse_v2t_close_and_paste();
        if (trackpad_mic_btn && lv_obj_is_valid(trackpad_mic_btn))
            lv_obj_clear_flag(trackpad_mic_btn, LV_OBJ_FLAG_HIDDEN);
        mode_set_visible(HID_MODE_TRACKPAD, true);
        mode_set_visible(HID_MODE_KEYBOARD, false);
        current_hid_mode = HID_MODE_TRACKPAD;
        keyboard_visible = false;
        // 收回 trackpad 一律 reset 成 mic view，下次 expand 重新從語音
        // 輸入開始（即使先前在 keyboard view 才拉下也一樣）
        if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
        {
            lv_obj_set_style_translate_y(kbd_mic_section, 0, 0);
            lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
        }
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
        {
            lv_obj_set_style_translate_y(keyboard_container, 0, 0);
            lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
        }
        // Enter btn 在 mode_set_visible(KEYBOARD, false) 已被 hide；
        // 把 translate_y 重設為 0，下次 expand 才能從 -80 → 0 進場
        if (input_enter_btn && lv_obj_is_valid(input_enter_btn))
            lv_obj_set_style_translate_y(input_enter_btn, 0, 0);
        kbd_lower_is_keyboard = false;
        kbd_lower_update_arrows_visibility();
        kbd_lower_update_arrows_visibility();
        kbd_lower_update_arcs_visibility();
        // 完全退出輸入模式 → 清掉 skaibar 狀態
        skaibar_active = false;
        skaibar_options_count = 0;
        skaibar_selected_idx = -1;
    }
    else
    {
        // snap 回 keyboard mode：把 UI 鎖回最終位
        expand_anim_driver_cb(NULL, 100);
    }
}

static void start_kbd_to_trackpad_collapse_anim(int32_t from_progress,
                                                bool commit)
{
    collapse_anim_commit = commit;
    collapse_anim_running = true;
    int32_t to_progress = commit ? 0 : 100;
    if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
        lv_anim_del(text_input_bar_bg, NULL);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, text_input_bar_bg); // var 不重要，cb 內不使用
    lv_anim_set_values(&a, from_progress, to_progress);
    lv_anim_set_time(&a, EXPAND_ANIM_TIME_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, expand_anim_driver_cb);
    lv_anim_set_ready_cb(&a, collapse_anim_done_cb);
    lv_anim_start(&a);
}

static void text_input_bar_drag_event_cb(lv_event_t *e)
{
    // 只在 keyboard mode 才處理；其他狀態（含動畫中）直接忽略
    if (current_hid_mode != HID_MODE_KEYBOARD) return;
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t pt;
    lv_indev_get_point(indev, &pt);

    if (code == LV_EVENT_PRESSED)
    {
        tib_drag_start_x = pt.x;
        tib_drag_start_y = pt.y;
        tib_drag_last_dy = 0;
        tib_drag_tracking = true;
        tib_drag_engaged = false;
        tib_drag_rejected = false;
    }
    else if (code == LV_EVENT_PRESSING && tib_drag_tracking &&
             !tib_drag_rejected)
    {
        int16_t dy = pt.y - tib_drag_start_y;
        int16_t dx = pt.x - tib_drag_start_x;
        int16_t dx_abs = dx < 0 ? -dx : dx;

        if (!tib_drag_engaged)
        {
            // 還沒過閾值前：如果手指明顯往水平方向動，這次 session 放棄
            if (dx_abs >= TIB_DRAG_ENGAGE_PX && dx_abs > dy)
            {
                tib_drag_rejected = true;
                return;
            }
            // 必須往下移動超過 ENGAGE_PX 才進入拖曳；往上拖無效
            if (dy < TIB_DRAG_ENGAGE_PX) return;
            tib_drag_engaged = true;
            // 殺掉任何還在跑的 expand/collapse anim
            if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
                lv_anim_del(text_input_bar_bg, NULL);
        }

        if (dy < 0) dy = 0;
        // 即時記下 dy；RELEASED 直接用這個值，不再依賴 release 時的 indev pt
        tib_drag_last_dy = dy;
        int32_t progress = 100 - (int32_t)dy * 100 / TIB_DRAG_RANGE_PX;
        if (progress < 0) progress = 0;
        if (progress > 100) progress = 100;
        expand_anim_driver_cb(NULL, progress);
    }
    else if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) &&
             tib_drag_tracking)
    {
        tib_drag_tracking = false;
        if (!tib_drag_engaged) return;
        // 用 PRESSING 累積的 dy；release 時的 pt 在某些 PRESS_LOST 情境下會
        // 變回起點/0/或 stale，靠 RELEASED 自己重算 dy 不可靠
        int16_t dy = tib_drag_last_dy;
        if (dy < 0) dy = 0;
        int32_t progress = 100 - (int32_t)dy * 100 / TIB_DRAG_RANGE_PX;
        if (progress < 0) progress = 0;
        if (progress > 100) progress = 100;
        bool commit = dy >= TIB_DRAG_COMMIT_PX;
        start_kbd_to_trackpad_collapse_anim(progress, commit);
    }
}

static void kbd_mic_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    // 點 mic btn → toggle V2T（沿用 keyboard 長按空白鍵的邏輯）
    if (mouse_v2t_active)
        mouse_v2t_close_and_paste();
    else
        mouse_v2t_open();
}

// =====================================================================
// 媒體中心 pull-down panel 實作
// =====================================================================

// 用 ble_hid.h 內現成的 consumer report 函式：
extern void play_pause_through_hid(void);
extern void play_next_through_hid(void);
extern void play_prev_through_hid(void);
extern void volume_up_through_hid(void);
extern void volume_down_through_hid(void);
extern char *get_media_title(void);

LV_IMG_DECLARE(img_media_play);
LV_IMG_DECLARE(img_media_pause);
LV_IMG_DECLARE(img_media_previous);
LV_IMG_DECLARE(img_media_next);
LV_IMG_DECLARE(volume_up);
LV_IMG_DECLARE(volume_down);

static void media_center_update_play_icon(bool playing)
{
    media_center_play_state = playing;
    if (!media_center_play_img || !lv_obj_is_valid(media_center_play_img))
        return;
    lv_img_set_src(media_center_play_img,
                   playing ? &img_media_pause : &img_media_play);
}

static void media_center_play_btn_cb(lv_event_t *e)
{
    (void)e;
    LOG_D("media center play/pause tap");
    play_pause_through_hid();
    media_center_update_play_icon(!media_center_play_state);
}

static void media_center_prev_btn_cb(lv_event_t *e)
{
    (void)e;
    LOG_D("media center prev tap");
    play_prev_through_hid();
}

static void media_center_next_btn_cb(lv_event_t *e)
{
    (void)e;
    LOG_D("media center next tap");
    play_next_through_hid();
}

static void media_center_vol_up_btn_cb(lv_event_t *e)
{
    (void)e;
    LOG_D("media center vol up tap");
    volume_up_through_hid();
}

static void media_center_vol_down_btn_cb(lv_event_t *e)
{
    (void)e;
    LOG_D("media center vol down tap");
    volume_down_through_hid();
}

static lv_obj_t *media_center_make_icon_btn(lv_obj_t *parent,
                                            const lv_img_dsc_t *icon,
                                            lv_event_cb_t cb,
                                            uint16_t size)
{
    lv_obj_t *btn = lv_obj_create(parent);
    lv_obj_remove_style_all(btn);
    lv_obj_set_size(btn, size, size);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *img = lv_img_create(btn);
    lv_img_set_src(img, icon);
    // 圖檔原生尺寸 ~150px（沿用 app_media.c 在 116×80 btn 用 zoom=0.6 的比例）
    // 依 btn size 線性縮放，讓 icon 約佔 btn 寬度的 ~70%
    uint16_t zoom = (uint16_t)(((uint32_t)256 * size * 7) / (10 * 150));
    lv_img_set_zoom(img, zoom);
    lv_obj_center(img);
    return btn;
}

/**
 * @brief 建立媒體中心 tileview（仿 app_clock_status_bar）：
 *        - tileview 全螢幕，預設 hidden，初始 tile = home (0,1)
 *        - media tile (0,0)：媒體中心內容，geometric 在 home 上方
 *          → 從 home 往下拉，scroll_y 減少，media 從畫面上方滑下來
 *        - home tile (0,1)：透明，初始位置
 *        拖曳/snap/動畫全交給 LVGL tileview 處理，不再自己算 y
 */
static void create_media_center_panel(lv_obj_t *parent)
{
    media_tileview = lv_tileview_create(parent);
    lv_obj_set_size(media_tileview, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(media_tileview, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_scrollbar_mode(media_tileview, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_opa(media_tileview, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(media_tileview, 0, 0);
    lv_obj_set_style_pad_all(media_tileview, 0, 0);
    lv_obj_add_flag(media_tileview, LV_OBJ_FLAG_HIDDEN);

    // media (0,0)：geometric 上方，往下拖才滾到這裡
    media_tile = lv_tileview_add_tile(media_tileview, 0, 0, LV_DIR_BOTTOM);
    lv_obj_set_size(media_tile, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(media_tile, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(media_tile, LV_OPA_80, 0);
    lv_obj_set_style_border_width(media_tile, 0, 0);
    lv_obj_set_style_radius(media_tile, 0, 0);
    lv_obj_set_scrollbar_mode(media_tile, LV_SCROLLBAR_MODE_OFF);

    // home (0,1)：geometric 下方，初始位置；只能往上滾去 media
    media_home_tile = lv_tileview_add_tile(media_tileview, 0, 1, LV_DIR_TOP);
    lv_obj_set_style_bg_opa(media_home_tile, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(media_home_tile, 0, 0);
    lv_obj_set_size(media_home_tile, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_scrollbar_mode(media_home_tile, LV_SCROLLBAR_MODE_OFF);

    // 曲名
    media_center_title_label = lv_label_create(media_tile);
    lv_label_set_text(media_center_title_label, "Media Title");
    lv_obj_set_style_text_color(media_center_title_label,
                                lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(media_center_title_label,
                                LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(media_center_title_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(media_center_title_label, LV_HOR_RES_MAX - 80);
    lv_obj_align(media_center_title_label, LV_ALIGN_TOP_MID, 0, 100);

    // 控制列：上一首 / 播放 / 下一首（btn 與 icon 同步放大 1.5x）
    lv_obj_t *btn_prev =
        media_center_make_icon_btn(media_tile, &img_media_previous,
                                   media_center_prev_btn_cb, 90);
    lv_obj_align(btn_prev, LV_ALIGN_CENTER, -120, 0);

    media_center_play_btn =
        media_center_make_icon_btn(media_tile, &img_media_play,
                                   media_center_play_btn_cb, 120);
    lv_obj_align(media_center_play_btn, LV_ALIGN_CENTER, 0, 0);
    media_center_play_img = lv_obj_get_child(media_center_play_btn, 0);

    lv_obj_t *btn_next = media_center_make_icon_btn(media_tile,
                                                    &img_media_next,
                                                    media_center_next_btn_cb,
                                                    90);
    lv_obj_align(btn_next, LV_ALIGN_CENTER, 120, 0);

    // 音量 -/+
    lv_obj_t *btn_vol_down =
        media_center_make_icon_btn(media_tile, &volume_down,
                                   media_center_vol_down_btn_cb, 75);
    lv_obj_align(btn_vol_down, LV_ALIGN_BOTTOM_MID, -90, -80);

    lv_obj_t *btn_vol_up =
        media_center_make_icon_btn(media_tile, &volume_up,
                                   media_center_vol_up_btn_cb, 75);
    lv_obj_align(btn_vol_up, LV_ALIGN_BOTTOM_MID, 90, -80);

    // tileview value-changed：snap 回 home 時自動隱藏
    lv_obj_add_event_cb(media_tileview, media_tileview_event_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);

    // 起始 tile 設 home (0,1)
    lv_obj_set_tile_id(media_tileview, 0, 1, false);
}

/**
 * @brief tileview value-changed cb：snap 完成後決定是否要收掉 tileview
 *        - tile 在 home → tileview 收進 hidden（讓底下 mouse mode UI 可用）
 *        - tile 在 media → 維持顯示
 */
static void media_tileview_event_cb(lv_event_t *e)
{
    (void)e;
    if (!media_tileview || !lv_obj_is_valid(media_tileview))
        return;
    lv_obj_t *act = lv_tileview_get_tile_act(media_tileview);
    if (act == media_home_tile)
    {
        lv_obj_add_flag(media_tileview, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief 對外的開/收媒體中心：透過 lv_obj_set_tile_id 切換，
 *        animate=true 時用 tileview 內建動畫
 */
static void media_center_set_open(bool open, bool animate)
{
    if (!media_tileview || !lv_obj_is_valid(media_tileview))
        return;
    if (open)
    {
        lv_obj_clear_flag(media_tileview, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(media_tileview);
        // 開啟 = 滾到 media tile (0,0)
        lv_obj_set_tile_id(media_tileview, 0, 0, animate);
        if (media_center_title_label &&
            lv_obj_is_valid(media_center_title_label))
        {
            char *title = get_media_title();
            lv_label_set_text(media_center_title_label,
                              (title && title[0]) ? title : "Media Title");
        }
    }
    else
    {
        // 收起 = 滾到 home tile (0,1)
        lv_obj_set_tile_id(media_tileview, 0, 1, animate);
        // value-changed 會在動畫結束後收 hidden；無動畫直接收
        if (!animate)
            lv_obj_add_flag(media_tileview, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief 切到下一個 hid mode（觸碰板 ↔ 鍵盤）
 *        從原本 mode_label_event_cb 的「短拖視為 tap」分支抽出來共用
 */
static void hid_mode_toggle(void)
{
    mode_swipe_kill_timer();
    hid_mode_t old_mode = current_hid_mode;
    hid_mode_t new_mode =
        (hid_mode_t)(((int)current_hid_mode + 1) % HID_MODE_COUNT);
    current_hid_mode = new_mode;
    mode_set_visible(old_mode, false);
    mode_set_visible(new_mode, true);
    mode_set_translate_x(old_mode, 0);
    mode_set_translate_x(new_mode, 0);
    keyboard_visible = (new_mode == HID_MODE_KEYBOARD);
    mode_swipe_active = false;
    LOG_D("mode tap -> %s", hid_mode_names[new_mode]);
}

/**
 * @brief status_bar_area_up 事件 cb（仿 app_clock_status_bar 的
 *        notification_status_bar_cb）
 *        - PRESSED：把 tileview 顯示出來、tile 設成 home (0,0)
 *          這樣使用者後續拖曳時 LVGL 會把 press 轉給 tileview，
 *          由 tileview 原生處理拖曳/snap/動畫
 *        - RELEASED：只在 press 沒被 tileview 接走時才會 fire
 *          → 視為純點擊：收掉 tileview + 切換 mode
 */
static void status_bar_area_up_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED)
    {
        if (!media_tileview || !lv_obj_is_valid(media_tileview))
            return;
        // 把 tileview 鎖在 home (0,1)；接下來使用者往下拖時 LVGL 把
        // press 轉給 tileview，tileview 自己滾到 media (0,0)
        lv_obj_set_tile_id(media_tileview, 0, 1, false);
        lv_obj_clear_flag(media_tileview, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(media_tileview);
    }
    else if (code == LV_EVENT_RELEASED)
    {
        // 沒被 tileview 接走 → tap → 收 tileview
        // （之前在這裡會 hid_mode_toggle，現在改用底部 bar 切換 mode）
        if (media_tileview && lv_obj_is_valid(media_tileview))
        {
            lv_obj_add_flag(media_tileview, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/**
 * @brief 對外暴露的更新介面：曲名/播放狀態變化時通知 mouse mode 媒體中心
 *        bloc_control.c 的 notify_media_title 改成也呼叫這個
 */
void mouse_mode_handle_media_title(const char *title)
{
    if (!media_center_title_label ||
        !lv_obj_is_valid(media_center_title_label))
        return;
    lv_label_set_text(media_center_title_label,
                      (title && title[0]) ? title : "Media Title");
}

void mouse_mode_handle_media_play_state(bool playing)
{
    media_center_update_play_icon(playing);
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

    // 純視覺，事件交給下面的 status_bar_area_up 統一處理

    // 頂部 status_bar_area_up：媒體中心 pull-down trigger（仿 clock 模式）
    //   - PRESS 把 tileview 顯示出來，press 自動轉給 tileview 處理拖曳
    //   - RELEASE 只在沒拖曳的 tap 情境會 fire → 收 tileview + 切 mode
    //   - 故意「不」加 PRESS_LOCK，讓 press 在 tileview 顯示後可以轉移過去
    status_bar_area_up = lv_obj_create(bg);
    lv_obj_remove_style_all(status_bar_area_up);
    lv_obj_set_size(status_bar_area_up, LV_HOR_RES_MAX, 80);
    lv_obj_align(status_bar_area_up, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(status_bar_area_up, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(status_bar_area_up, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(status_bar_area_up, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_flag(status_bar_area_up, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(status_bar_area_up, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_event_cb(status_bar_area_up, status_bar_area_up_cb,
                        LV_EVENT_ALL, NULL);

    // Trackpad mode 下方 multitask hint 觸發區（跨 mode hit area）
    bottom_swipe_area = lv_obj_create(bg);
    lv_obj_remove_style_all(bottom_swipe_area);
    // 擴大範圍覆蓋 trackpad mic btn 跟周圍，整個下方區域都接收觸碰
    lv_obj_set_size(bottom_swipe_area, 280, 100);
    lv_obj_set_pos(bottom_swipe_area, (LV_HOR_RES_MAX - 280) / 2,
                   LV_VER_RES_MAX - 100);
    lv_obj_set_style_bg_opa(bottom_swipe_area, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(bottom_swipe_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(bottom_swipe_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(bottom_swipe_area, text_input_bar_cb, LV_EVENT_ALL,
                        NULL);

    // === 媒體中心 pull-down panel（從頂部模式切換條往下拉觸發）===
    create_media_center_panel(bg);

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

// 直接設定 handfree state（不 toggle），給 fsr 壓感 sampler 用
void set_hid_mouse_handfree_mode_to(bool v)
{
    handfree = v;
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
/* T1 part 1 (host decouple): was static on_start(scr). Now the public
   component entry — builds the mouse UI under any host. The gui_app glue
   calls it with lv_scr_act(); device_pager (T4) calls it with its tile. */
void hid_mouse_create(lv_obj_t *scr)
{
    /* Screen-level launch transition only when we own the screen. When hosted
       in a container (T4 device_pager), skip it — the pager drives its own
       reveal animation and the trans-anim would flash a blank overlay. */
    if (scr == lv_scr_act())
        cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    lv_create_mouse_screen(scr);
    app_control_set_mouse_mode(true);

    extern void set_status_bar_area_up_state(bool state);
    extern void set_status_bar_area_down_state(bool state);
    extern void set_status_bar_area_left_state(bool state);

    set_status_bar_area_up_state(false);
    set_status_bar_area_down_state(false);
    set_status_bar_area_left_state(false);

    // 右側 device-change bar 在 lv_layer_top()，原本會擋住右弧滾動。
    // 加 ADV_HITTEST + 自訂 HIT_TEST cb：點在弧形觸碰區就拒絕命中，
    // 觸碰會掉到底下的 touch_bg 走 arc 滾動；不在弧形區的（純從邊緣
    // 左滑）才落到 bar 開選單
    lv_obj_t *dc_bar = get_device_change_bar_area_right();
    if (dc_bar && lv_obj_is_valid(dc_bar))
    {
        lv_obj_add_flag(dc_bar, LV_OBJ_FLAG_ADV_HITTEST);
        lv_obj_add_event_cb(dc_bar, device_change_bar_hit_test_cb,
                            LV_EVENT_HIT_TEST, NULL);
    }
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
 * @brief Tear down the mouse UI + deactivate control surface.
 *        T1 part 1 (host decouple): was static on_stop(void).
 */
void hid_mouse_destroy(void)
{
    app_control_set_mouse_mode(false);

    // 還原右側 device-change bar 的 ADV_HITTEST + 拿掉自訂 cb，
    // 下個 app 看到的就是原本完整的 hit 行為
    lv_obj_t *dc_bar = get_device_change_bar_area_right();
    if (dc_bar && lv_obj_is_valid(dc_bar))
    {
        lv_obj_remove_event_cb(dc_bar, device_change_bar_hit_test_cb);
        lv_obj_clear_flag(dc_bar, LV_OBJ_FLAG_ADV_HITTEST);
    }

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
    right_scroll_bar = NULL;
    for (int i = 0; i < LEFT_SCROLL_NODE_COUNT; i++)
    {
        left_scroll_nodes[i] = NULL;
        right_scroll_nodes[i] = NULL;
    }
    scroll_node_offset_deg = 0.0f;
    scroll_accum_angle = 0.0f;
    scroll_last_theta = 0.0f;
    scroll_ui_level = 0;
    left_scroll_active = false;

    // 媒體中心清理
    media_tileview = NULL;
    media_home_tile = NULL;
    media_tile = NULL;
    media_center_title_label = NULL;
    media_center_play_btn = NULL;
    media_center_play_img = NULL;
    status_bar_area_up = NULL;

    // Keyboard mode 下半部 mic 區清理
    kbd_mic_section = NULL;
    kbd_mic_section_mic_btn = NULL;
    kbd_mic_section_mic_img = NULL;
    kbd_mic_section_mic_pulse = NULL;
    kbd_mic_section_right_arrow = NULL;
    kbd_lower_is_keyboard = false;

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
    input_enter_img = NULL;
    if (cursor_blink_timer != NULL)
    {
        lv_timer_del(cursor_blink_timer);
        cursor_blink_timer = NULL;
    }
    clear_input_display();

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
        hid_mouse_create(scr);
        break;
    }
    case GUI_APP_MSG_ONRESUME:
        watch_system_mouse_resume();
        break;
    case GUI_APP_MSG_ONPAUSE:
        watch_system_mouse_pause();
        break;
    case GUI_APP_MSG_ONSTOP:
        hid_mouse_destroy();
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
