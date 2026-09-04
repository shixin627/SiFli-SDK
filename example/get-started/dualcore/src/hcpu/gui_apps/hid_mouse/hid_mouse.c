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
#include "app_clock_status_bar.h" // app_clock_device_change_bar_open
#include "ui_handler.h"
#include <cJSON.h>
#include "pinyin_dict.h" /* 中文拼音字典(founder 2026-07-22) */
#include "en_words.h"    /* 英文常用字表(單字推薦,founder 2026-07-22) */
#include "ui_helper.h"
#include "ui_img_helper.h"
#include "lvsf_gesture.h"
#include "ble_device_manager.h"
#include "bloc_motion_tracking.h"
#include "ble_hid.h"
#include "communicate_task.h" /* commu_send_mouse_back — device-page trackpad relay */
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
    /* 觸控座標跳變濾波上限(founder 2026-07-30 診斷確診):觸控面板單軸座標會瞬間噴到異常值
       (另一軸不動)→游標瞬移。單軸 delta 超此=誤報,丟這幀游標移動(基準照更新,下一幀座標
       回正常就繼續)。實測正常滑動/拖曳每幀 ≤43px、跳變 64~257px,取中間 60。太多正常快移
       被擋往上調、還漏跳變往下降。 */
    #define TOUCH_JUMP_REJECT_PX 60
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
LV_IMG_DECLARE(mouse_mode_icon);
LV_IMG_DECLARE(plus_button);
LV_IMG_DECLARE(enter_icon);
LV_IMG_DECLARE(capital_icon);
LV_IMG_DECLARE(backspace_icon);
LV_IMG_DECLARE(message_widget_bg); /* 語音站的輸入框底圖(與立起面板同一張,442x252) */
LV_IMG_DECLARE(img_logo);  /* 語音站:送 skaibar 查詢 */
LV_IMG_DECLARE(icon_send); /* 語音站:打進電腦剛剛點的輸入框(有聚焦才顯示) */
LV_IMG_DECLARE(icon_mic);
LV_IMG_DECLARE(space);
LV_IMG_DECLARE(img_skai); // 160 * 160
LV_IMG_DECLARE(erth);
LV_IMG_DECLARE(img_left_arrow); // 還有給 back_hint_icon 用
LV_IMG_DECLARE(img_right_arrow); // 頂部設備切換右箭頭
LV_IMG_DECLARE(Map_fill);
LV_IMG_DECLARE(micro_icon);
LV_IMG_DECLARE(micro_open_icon); // V2T active 時的 icon（淺藍麥克風）
LV_IMG_DECLARE(switch_icon);
LV_IMG_DECLARE(keyboard_icon);
LV_IMG_DECLARE(down_arrow); // 輸入框下方收回按鈕

static lv_point_t start_point;
static lv_point_t last_point;
static lv_point_t bottom_bar_start_point;
static lv_point_t bottom_bar_last_point;
static unsigned int press_time = 0;
static bool user_touching = false;
static bool scrolling = false;
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
// 起手點是否落在「右」弧形觸碰區：hosted（pager 掛載）左拖=滑鼠頁滑走
// 回錶盤；standalone 左拖=開 device-change 選單（standalone 唯一退出口）
static bool press_in_right_arc_zone = false;
// 右弧左拖 finger-follow 回錶盤中：本次 press 剩餘的 PRESSING 拿來驅動主
// tileview 跟手(不送滑鼠移動),放開才 commit/snap。改成 finger-follow(而非
// 過門檻直接播動畫)順便消掉「手指還按著、settle 把 HID 路由翻成真 BLE report
// 而動到手機實體游標」的窗口——teardown 延到放開後才發生。
static bool home_pull_active = false;
static int  home_pull_prog = 0; // 0..100;放開時 >=50 → commit 回錶盤,否則 snap 回
// 真正定義在 media tileview 區（hid_mouse_set_hosted）；tentative fwd decl
// 給右弧左拖的 hosted vs standalone 分流用
static bool s_hosted_by_pager;

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
// 滾輪節點進場因子（0 = 全黑融入背景 ~ 1000 = 正常灰）。預設 1000：standalone
// 不淡入；hosted（device_pager）建立後呼叫 hid_mouse_fade_in_scroll_wheel() 才從 0 淡。
static int32_t s_node_entrance = 1000;

// 左側滾動節點相關（實作在 mouse screen 建立處）
static float left_scroll_finger_theta(const lv_point_t *p);
static float left_scroll_normalize_delta(float d);
static void update_left_scroll_nodes(void);
static void apply_scroll_ui_level(void);
static void animate_scroll_ui_to(bool active);
static void scroll_node_snap_anim_cb(void *var, int32_t v);
static void snap_scroll_nodes(void);

// 頂部設備切換器：trackpad 頂部設備名 label 兩側箭頭切換設備（取代舊右側抽屜）
static char s_dev_active_id[SYNCED_DEVICE_ID_LEN] = {0}; // 目前選中控制的設備 id（高亮 + 頂部標籤用）
static lv_obj_t *s_ctrl_dev_label = NULL;    // trackpad 頂部「控制中設備」名稱常駐標籤
static lv_obj_t *s_dev_offline_overlay = NULL; /* active 設備斷線=觸碰板區灰+「斷線」
                                                  (founder 2026-07-22 二改:蓋 y80 以下
                                                  +吃 press 擋操作;頂部留給媒體下拉) */
static lv_obj_t *s_dev_status_dot = NULL; /* 媒體頁設備名上方在線燈號(綠/紅) */
static bool dev_active_offline(void); /* 定義在 overlay sync 旁 */
static lv_obj_t *s_dev_left_arrow = NULL;    // 設備名左側「上一台」箭頭（循環）
static lv_obj_t *s_dev_right_arrow = NULL;   // 設備名右側「下一台」箭頭（循環）
static void update_ctrl_dev_label(void); // 依 s_dev_active_id 更新頂部控制中設備名 + 箭頭可見性
static void devbar_update_name(void);    // 底部設備 bar 的名稱(定義在下方 devbar 段)
static bool devbar_consume_click(void);  // 這一次按壓是拖曳換設備 → 吃掉隨後的 CLICKED
static bool devbar_drag_active(void);    // 正在拖曳換設備(long-press 要讓開)
LV_IMG_DECLARE(device_btn);  // 設備鈕藥丸圖（與舊 device-change bar 共用同一資源）
/* skaibar_img(176x31 的靜態圖)2026-09-04 退役:底部那條改成自己畫的藥丸 + 設備名,
   才有辦法在拖曳時把寬度收成一顆球(圖片縮放會變形)。見下方 devbar 段。 */

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
    #define MODE_SWIPE_COMMIT_THRESHOLD 60
    #define MODE_SWIPE_ANIM_TIME_MS 200
static bool mode_swipe_active = false;
static hid_mode_t mode_swipe_target = HID_MODE_TRACKPAD;
static int8_t mode_swipe_target_side = -1;
static int32_t mode_swipe_anim_value; // 同上
// 自製 timer-based 動畫（避開 LVGL anim 的 race）
static lv_timer_t *mode_swipe_timer = NULL;
// 每個 mode 用一個 480×480 透明容器包，切換時整組移動
static lv_obj_t *mode_container[HID_MODE_COUNT] = {NULL};
static void apply_hid_mode(hid_mode_t mode);
/* R40:鍵盤模式 UI 延遲建立(38KB,見 mode_set_visible 的說明)。 */
static void kbd_ensure_built(void);

// keyboard mode 下半部 mic 區前置宣告
static void create_kbd_mic_section(lv_obj_t *parent);
static void kbd_lower_set_keyboard(bool show_kbd);
static void kbd_lower_switch(bool to_kbd); /* 鍵盤⇄語音兩站切換(含輸入列高度動畫) */
static void expand_anim_driver_cb(void *var, int32_t v);
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
// 設備切換箭頭 cb(定義在後段;媒體頁頂部設備名區用)
static void dev_arrow_prev_cb(lv_event_t *e);
static void dev_arrow_next_cb(lv_event_t *e);
static void media_center_set_open(bool open, bool animate);
static void media_center_update_play_icon(bool playing);
static void media_center_play_btn_cb(lv_event_t *e);
static void media_center_prev_btn_cb(lv_event_t *e);
static void media_center_next_btn_cb(lv_event_t *e);
static void media_center_vol_hold_cb(lv_event_t *e);
static void status_bar_area_up_cb(lv_event_t *e);
static void media_tileview_event_cb(lv_event_t *e);
static void hid_mode_toggle(void);

// 左右滾動弧的「觸發帶」厚度(px,貼外緣往內算)。原 50(比 UI 弧 30px 寬),
// 2026-07-02 使用者要求減半到 25 — 觸發帶維持貼邊,只變薄,誤觸率降低。
#define ARC_TOUCH_BAND_PX 25.0f

// 判斷觸碰點是否在左側滾動觸發區域
// UI 弧線寬度 30px、角度 150°~210°
static bool is_point_in_left_arc(const lv_point_t *p)
{
    float cx = LV_HOR_RES_MAX / 2.0f;
    float cy = LV_VER_RES_MAX / 2.0f;
    float dx = p->x - cx;
    float dy = p->y - cy;
    float dist = sqrtf(dx * dx + dy * dy);
    float outer_r = cx;
    float inner_r = outer_r - ARC_TOUCH_BAND_PX;
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
    float inner_r = outer_r - ARC_TOUCH_BAND_PX;
    if (dist < inner_r || dist > outer_r)
        return false;
    // 只接受右側（x > 中心）
    if (dx <= 0)
        return false;
    float max_dy = dist * 0.819f;
    return (dy >= -max_dy && dy <= max_dy);
}

// 判斷觸碰點是否在中間滾動觸發區
// 幾何：弧形觸發區（dist = outer_r-ARC_TOUCH_BAND_PX ~ outer_r）的內緣，再往內
// 延 THICKNESS px，角度範圍跟 is_point_in_left_arc 相同（左半 ±55°），
// 兩塊接成完整的左側弧帶
static bool is_point_in_center_scroll_zone(const lv_point_t *p)
{
    float cx = LV_HOR_RES_MAX / 2.0f;
    float cy = LV_VER_RES_MAX / 2.0f;
    float dx = p->x - cx;
    float dy = p->y - cy;
    float dist = sqrtf(dx * dx + dy * dy);
    float outer_r = cx;
    float zone_outer = outer_r - ARC_TOUCH_BAND_PX; // 接弧形觸發區的內緣
    float zone_inner = zone_outer - (float)CENTER_SCROLL_ZONE_THICKNESS;
    if (dist < zone_inner || dist > zone_outer)
        return false;
    if (dx >= 0)
        return false;
    float max_dy = dist * 0.819f;
    return (dy >= -max_dy && dy <= max_dy);
}
static lv_obj_t *crosshair_line1 = NULL;
static lv_obj_t *crosshair_line2 = NULL;
static lv_obj_t *text_input_bar = NULL;
static lv_obj_t *text_input_bar_bg = NULL;
// 跨 mode 的下方拖動 hit area（trackpad mode 觸發 multitask hint）
// 因為 text_input_bar_bg 已被 reparent 到 mode_container[KEYBOARD]，
// trackpad mode 下方需要獨立 hit area
static lv_obj_t *bottom_swipe_area = NULL;
// static lv_timer_t *colon_blink_timer = NULL;

// Keyboard related variables
static lv_obj_t *keyboard = NULL;
static bool keyboard_visible = false;
static lv_obj_t *custom_keyboard = NULL;
static lv_obj_t *keyboard_container = NULL;

static uint8_t keyboard_text_size = 1;

// Keyboard mode control
typedef enum
{
    KEYBOARD_MODE_LETTERS = 0,
    KEYBOARD_MODE_NUMBERS,
    /* 中文拼音(founder 2026-07-22):字母布局同 LETTERS,按鍵進拼音緩衝,
       輸入框下方選字列挑字上屏(字典=pinyin_dict.c,錶上離線)。 */
    KEYBOARD_MODE_CHINESE
} keyboard_mode_t;

static keyboard_mode_t current_keyboard_mode = KEYBOARD_MODE_LETTERS;

// Case control for letters
static bool is_uppercase = false;
static lv_obj_t *caps_btn = NULL;
static lv_obj_t *mode_btn = NULL;
static lv_obj_t *del_img = NULL;
/* 鍵盤 Mode 第三站=手寫(founder 2026-07-22 英文→數字→手寫循環);定義在手寫段。 */
static void hw_open_from_mode_switch(void);
/* 鍵盤頂部退出鈕(founder 2026-07-22:同手寫版):收鍵盤回觸碰板。掛跨 mode bg,
   顯藏由 mode_set_visible(KEYBOARD) 特例+swipe commit 的 extras sync 管。 */
static lv_obj_t *kbd_exit_btn = NULL;
static void kbd_exit_btn_event_cb(lv_event_t *e);
/* 螢幕最上方的下拉感應區:往下拖=收掉輸入框/鍵盤,回到選項清單(founder 2026-08-17)。
   高度取到輸入框上緣(VOICE_BOX_Y=107)之前,不蓋內容。 */
#define KBD_TOP_PULL_H 100
static lv_obj_t *kbd_top_pull = NULL;
static void kbd_top_pull_event_cb(lv_event_t *e);

/* ── 中文拼音狀態(founder 2026-07-22) ── */
#define KBD_CAND_MAX 20 /* 候選上限;列可左右滑看更多(founder 2026-07-22) */
static lv_obj_t *s_kbd_cand_row = NULL; /* 輸入框下方選字列(pill 同款) */
static lv_obj_t *s_kbd_py_lbl = NULL;   /* 列左端:目前拼音緩衝 */
static lv_obj_t *s_kbd_cand_btns[KBD_CAND_MAX];
static lv_obj_t *s_kbd_cand_lbls[KBD_CAND_MAX];
static char s_kbd_cand_texts[KBD_CAND_MAX][8]; /* 候選字 UTF-8(按下時上屏用) */
static int s_kbd_cand_count = 0;
static char s_py_buf[16] = {0}; /* 注音緩衝(UTF-8,最多 4 符號×3B;founder 改注音) */
static int s_py_len = 0;
/* 英文模式:目前單字緩衝(推薦用;空白/Enter/換模式重置)。founder 2026-07-22:
   英文也要數字列+單字推薦。 */
static char s_en_word[24] = {0};
static int s_en_len = 0;
/* 列目前顯示的內容種類(commit 行為依此分流) */
enum
{
    KBD_ROW_NONE = 0,
    KBD_ROW_DIGITS,  /* 數字 1~9:上屏該數字 */
    KBD_ROW_ZH_CAND, /* 注音候選:上屏整字 */
    KBD_ROW_EN_SUGG, /* 英文推薦:補完(只送尚未打的字尾) */
};
static uint8_t s_kbd_row_kind = KBD_ROW_NONE;
static void kbd_cand_refresh(void);
static void kbd_pinyin_clear(void);
static void kbd_cand_commit(int idx);
struct zy_map_s; /* 注音表(定義在組字區);按鍵 handler 只當存在性判斷用 */
static const struct zy_map_s *zy_find(const char *sym);

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
/* 觸控板右緣的鍵盤鈕:電腦有聚焦輸入框時才浮現(founder 2026-08-17)。 */
static lv_obj_t *kbd_side_btn = NULL;
static void kbd_side_btn_event_cb(lv_event_t *e);
static void mouse_open_input_station(bool direct_field);
static lv_obj_t *trackpad_mic_icon = NULL;
static lv_obj_t *trackpad_mic_red_dot = NULL;
static void bar_ai_on_tap(void); /* fwd：tap 當下立刻收自有底部 bar（定義在 lv_create 前） */

// Keyboard mode 下半部 mic 區（mic 按鈕 + 右側鍵盤按鈕，跟 keyboard 互換顯示）
static lv_obj_t *kbd_mic_section = NULL;
/* R40:鍵盤模式 UI 是否已建(延遲建立,38KB)。teardown 時歸 false。 */
static bool s_kbd_ui_built = false;
static lv_obj_t *kbd_mic_section_mic_btn = NULL;
static lv_obj_t *kbd_mic_section_mic_img = NULL; // 中央 mic icon，V2T 開關時切圖
static lv_obj_t *kbd_mic_section_mic_pulse = NULL; // V2T active 時的脈衝圓
static lv_obj_t *kbd_mic_section_right_arrow = NULL;
/* 語音站的控制項(founder 2026-08-03:立起面板那一整套搬進鍵盤區)。幾何沿用立起面板的
   相對關係:麥克風與刪除鍵並排在輸入列下方,logo/send 在輸入列上方。輸入列在 y=195
   (EXPAND_END_Y)、高約 90,mic_section 是整片置中,所以列的 dy 從畫面中心 233 起算。 */
/* 這次語音站是從 session 抽屜進來的 → 輸入框往下拖收合時滑回抽屜,不是回觸控板
   (founder 2026-08-17;細節見 mouse_drawer_open_input)。 */
static bool s_kbd_from_drawer = false;
/* 直打模式（founder 2026-08-18：「點手錶上右邊出現的鍵盤後進入的輸入模式就不用再叫出
   skaibar 了，只要直接輸入到我點的輸入框就好」）。由觸控板右緣那顆鍵盤鈕進站時立起 ——
   那顆鈕本來就只在「電腦有聚焦輸入框」時才浮現，所以這條路等於使用者已經指定了目的地。
   立起後：① 送給電腦的召喚變成 forceOpen=false / inputOnly=false，電腦於是**不開面板**，
   改把每一次 preview 直接打進那個欄位（桌面 RemoteFocusedTextInputRouter 的 direct-typing
   latch，增量 diff＋backspace）；② icon_send 不再送 commit —— 字早就在欄位裡了，再送一次
   會整段重打成雙份；③ 拖到 AI logo 那條手勢整個關掉（見 ai_drag_logo_show）：這個模式的
   定義就是「不叫 skaibar」，要問 AI 走 bar 開抽屜那條，不受影響。 */
static bool s_kbd_direct_field = false;
/* 這一按落在送出鍵上 → 輸入框的手勢整串讓開(見 voice_box_gesture_cb)。 */
static bool s_voice_press_on_send = false;
/* 抽屜地球鍵按下 → 進場動畫落地後才切鍵盤輪盤(見 kbd_enter_slide_done)。 */
static bool s_kbd_pending_wheel = false;
static lv_obj_t *kbd_voice_del_btn = NULL;
static lv_obj_t *kbd_voice_logo_btn = NULL;
static lv_obj_t *kbd_voice_send_btn = NULL;
static lv_timer_t *kbd_voice_del_repeat = NULL;
#define VOICE_BTN_D      64   /* 麥克風圓鈕直徑(立起面板同尺寸) */
/* 下方三顆:鍵盤靠左、麥克風置中、刪除靠右(founder 2026-08-03)。
   y=403 那排在 466 圓內可用半寬 159,±110 都進得去。 */
#define VOICE_MIC_DX      0
#define VOICE_DEL_DX    110
/* 大框(442×252)置中後上下各餘 107,所以控制項的 dy 直接沿用立起面板實測過的那組。
   y=403 那一排在 466 圓內可用半寬 159 —— 鍵盤鈕 -130 / 麥克風 -40 / 刪除 +52 都在裡面。 */
#define VOICE_ROW_DY    170   /* 大框下方那排:鍵盤鈕 + 麥克風 + 刪除 */
/* logo/send 與立起面板同位(y=55)。模式圖示與框上緣之間只有 53px,塞不下 80px 的 logo,
   所以語音站比照立起面板**把頂部模式圖示收起來**(它整片蓋掉狀態列,這裡改成手動藏)。 */
#define VOICE_ICON_DY  (-178)
/* 送出鍵貼在輸入框內緣右側的內縮量(負值=往左)。沿用它取代的 enter_icon 原本的 -10。 */
#define SEND_BTN_INSET_X (-10)
/* 視覺置中的微調(founder 2026-08-18 眼驗:「還是稍微偏上,直接把他往下移動 2pix」)。
   **幾何上本來就是置中的** —— 真機探針量到對齊穩定後,鍵盤站 gap_top=5 / gap_bot=6、
   語音站 109/109,對稱到 1px 以內。偏上是視覺重心(圖案墨水滿版 34x34 但重量偏上),
   量不出來、只能照眼睛調 —— 這是刻意的光學補償,不是在修正算錯的座標。
   2 → 3(founder 二次眼驗「再往下 1pix」)。 */
#define SEND_BTN_NUDGE_Y 3
/* 送出鍵的觸控外擴。圖示視覺只有 ~24px,靠這個把熱區撐到 34+2*15=64(≥44pt 基線);
   輸入框的手勢也用同一個值讓開,兩邊一致才不會有「看得到卻按不到」的縫。 */
#define SEND_BTN_EXT_CLICK 15
/* icon_send 原生 34x34,塞進 45 高的鍵盤輸入列幾乎頂滿(founder 2026-08-18:「圖片有點太大」)。
   縮的是**繪製**:lv_img_set_zoom 只影響畫出來的大小,物件 bbox 仍是 34x34,加上
   ext_click_area 15 → 觸控標的維持 64px(≥44pt 基線),founder 要的「觸碰範圍不變」。 */
#define SEND_BTN_ZOOM  180   /* 34 * 180/256 ≈ 24px 視覺 */
#define SEND_BTN_IMG_W 34
#define VOICE_KBD_DX  (-110)  /* 回鍵盤鈕:同一排最左 */
#define VOICE_LOGO_DX  (-29)
#define VOICE_SEND_DX    52
#define VOICE_DEL_REPEAT_MS 120
static void kbd_voice_del_event_cb(lv_event_t *e);
static void kbd_voice_send_event_cb(lv_event_t *e);
static void voice_do_send(int which);
static void kbd_voice_layout_send_icons(void);
/* 「長按錄音 → 往上拖到 logo → 放開送 AI」用的浮動圖示(founder 2026-08-07:
   常駐的送 AI logo 拿掉,改成按住的當下在手指上方生一個淡的,拖上去變實心,
   放開才送)。s_ai_drag_armed = 手指目前在圖示範圍內。 */
static lv_obj_t *s_ai_drag_logo = NULL;
static bool s_ai_drag_armed = false;
/* 手勢來源(麥克風鈕 / 輸入框):期間補上 PRESS_LOCK,手指移出它的範圍才不會被
   LVGL 判成 PRESS_LOST 而中斷 —— 那正是「還沒放開就送出去」的原因。 */
static lv_obj_t *s_ai_drag_src = NULL;
#define AI_DRAG_LOGO_DY 110  /* 圖示中心在按壓點正上方多少 px */
#define AI_DRAG_HIT_R 58     /* 判定進入圖示的半徑(比圖大一圈,好命中) */
static void voice_delete_selection(void);
static void voice_box_gesture_cb(lv_event_t *e);
static void kbd_lower_switch(bool to_kbd);
static void voice_preview_schedule(void); /* 推手錶文字給電腦(防抖) */
static void kbd_voice_del_stop_repeat(void); /* 收掉刪除鍵的連續刪除 timer */
static void kbd_voice_del_update_icon(void); /* 有字=退格圖 / 沒字=退出圖 */
static void voice_ball_event_cb(lv_event_t *e);
static void update_cursor_position(void);
// micro_open_icon 是淺藍麥克風，脈衝色取相近的藍
#define KBD_MIC_PULSE_COLOR 0x5DA8FF
#define KBD_MIC_PULSE_MAX_SIZE 100
#define KBD_MIC_PULSE_PERIOD_MS 1200
static bool kbd_lower_is_keyboard = false; // false = mic 區顯示，true = 鍵盤顯示
/* 2026-08-15 hosted 滑鼠 OOM 修:語音站進場只建輸入列+mic 區,圓形鍵盤(鍵盤模式 38K 裡的
   大頭)延到使用者真的切去鍵盤站(kbd_lower_switch(true) 的 lazy 分支)才建。hosted 滑鼠
   free 約 42K,整包建下去只剩 4.6K,隨後任何配置 lv_obj_create 回 NULL → hard fault
   (真機兩度複現,[heap] kbd-build:after free=4672 緊接 mem manage fault)。 */
static bool s_kbd_build_defer_wheel = false;
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
static lv_obj_t *s_top_logo = NULL; /* 頂部滑鼠圖(左拉進手寫時跟手位移;宣告提前
                                       到此=手寫段 hw_open_commit 也要歸位它) */
static void top_logo_tx_anim_exec(void *obj, int32_t v); /* 定義在 status_bar 段 */
static lv_obj_t *media_tileview = NULL;
static lv_obj_t *media_home_tile = NULL;
static lv_obj_t *media_tile = NULL;
static lv_obj_t *media_center_title_label = NULL;
/* 頂部下拉去處覆寫（面板 host 時指向 lv_top_panel 的 reveal），NULL = 自有媒體層 */
static void (*s_pulldown_cb)(void) = NULL;
static lv_obj_t *media_center_play_img = NULL;
static bool media_center_play_state = false;

static void update_v2t_btn_appearance(bool mic_active);
static void mouse_v2t_set_active(bool active);
static void voice_glow_set_active(bool on); /* 語音站大框外圈的錄音脈衝 */
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

static int file_items_count = 0;

// Long press detection variables
static rt_timer_t long_press_timer = NULL;
static bool is_long_press_triggered = false;
/* 「長按空白鍵 = 按著麥克風」專用旗標。不能共用 is_long_press_triggered ——
   那支被 mouse_v2t_open_with_intent() 也設成 true(麥克風鈕、語音站長按輸入框
   都會經過),拿它當「放開就停止錄音」的條件會把那些 session 一起掐掉
   (founder 2026-08-07:長按輸入框開語音辨識沒有文字回來)。 */
static bool s_space_hold_v2t = false;
static const char *long_press_key_text = NULL;
static const char *closest_key_text = NULL;
/* 注音佈局的鍵數遠比英文多，50 不夠 —— 而 register_key_button() 滿了是
   **靜默丟棄**，第四行（地球/空白/刪除）又是最後才註册，所以在注音頁面那三顆
   根本沒進到接近度比對表，按下去完全沒反應（founder 2026-08-07）。 */
#define MAX_KEY_BUTTONS 80
static lv_obj_t *all_keys[MAX_KEY_BUTTONS]; // 存儲所有按鍵的數組
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
static void kbd_del_update_icon(void); /* 鍵盤站刪除鍵:空框時顯示成「收下」 */
static void mouse_exit_input_station(void); /* 離開輸入模式(帶收合動畫)的單一出口 */
static void kbd_commit_to_trackpad(void); /* 收合收尾:切回觸控板容器 */
static void kbd_bar_set_voice_box(bool voice); /* 語音大框 ⇄ 鍵盤藥丸(定義在換站段) */

// Long press detection functions
static void long_press_timer_callback(void *parameter);
static void start_long_press_timer(const char *key_text);
static void stop_long_press_timer(void);

// Proximity detection functions
static void register_key_button(lv_obj_t *btn);
static lv_obj_t *find_closest_key(lv_point_t touch_point);
static void handle_proximity_input(lv_event_t *e);

    #if USING_EDGE_BOTTOM_DETECTION
static void start_multiple_pages_timer(void);
    #endif

/**
 * @brief Update cursor position to follow the text
 */
static bool s_voice_box_on = false; /* 語音站的大框正在顯示(見 kbd_bar_set_voice_box) */

/* ── 語音站的游標 / 框選狀態(founder 2026-08-03:立起面板那一整套搬進來) ──
   文字真相是本地的 input_buffer(UTF-8),所以插入點用 **byte offset** 存;要餵給 LVGL 的
   letter_pos / text-selection 時再換算成 code point。-1 = 停在最末端。 */
static int s_voice_caret = -1;                        /* byte offset,-1 = 末端 */
static lv_obj_t *s_voice_caret_ball = NULL;           /* 游標下方的拖曳把手 */
#define VOICE_BALL_D 22
/* 按下麥克風當刻,插入點左右兩半的快照 —— 串流轉錄每次都用它們重組(見 mouse_apply_v2t_input)。 */
static char s_voice_ins_prefix[128] = {0};
static char s_voice_ins_suffix[128] = {0};
static bool s_voice_ins_armed = false;
static int s_voice_sel_from_cp = -1, s_voice_sel_to_cp = -1; /* 反白範圍(code point) */

/* 前 bytes 個位元組裡有幾個字(UTF-8 續接位元組 10xxxxxx 不算)。 */
static int voice_cp_count(const char *s, int bytes)
{
    int n = 0;
    for (int i = 0; i < bytes && s[i]; i++)
        if (((unsigned char)s[i] & 0xC0) != 0x80)
            n++;
    return n;
}

/* 第 cp 個字的起始 byte offset(cp >= 字數時回總長度)。 */
static int voice_byte_of_cp(const char *s, int cp)
{
    int n = 0, i = 0;
    while (s[i])
    {
        if (((unsigned char)s[i] & 0xC0) != 0x80)
        {
            if (n == cp)
                return i;
            n++;
        }
        i++;
    }
    return i;
}

/* 目前插入點的 byte offset(把 -1 正規化成末端)。 */
static int voice_caret_bytes(void)
{
    if (s_voice_caret < 0 || s_voice_caret > input_length)
        return input_length;
    return s_voice_caret;
}

static bool voice_sel_active(void)
{
    return s_voice_sel_from_cp >= 0 && s_voice_sel_to_cp > s_voice_sel_from_cp;
}

static void voice_sel_clear(void)
{
    s_voice_sel_from_cp = -1;
    s_voice_sel_to_cp = -1;
    if (input_display_label && lv_obj_is_valid(input_display_label))
    {
        lv_label_set_text_sel_start(input_display_label, LV_DRAW_LABEL_NO_TXT_SEL);
        lv_label_set_text_sel_end(input_display_label, LV_DRAW_LABEL_NO_TXT_SEL);
    }
}

static void voice_sel_apply(int a_cp, int b_cp)
{
    if (a_cp > b_cp) { int t = a_cp; a_cp = b_cp; b_cp = t; }
    s_voice_sel_from_cp = a_cp;
    s_voice_sel_to_cp = b_cp;
    if (input_display_label && lv_obj_is_valid(input_display_label))
    {
        lv_label_set_text_sel_start(input_display_label, (uint32_t)a_cp);
        lv_label_set_text_sel_end(input_display_label, (uint32_t)b_cp);
    }
}

static void update_cursor_position(void)
{
    if (input_cursor == NULL || input_display_label == NULL)
        return;

    /* 有插入點就依插入點畫,兩站共用(2026-08-07 founder:在語音站點好位置切到鍵盤,
       游標卡在字中間)—— 鍵盤站原本一律把游標貼在 label 右緣,而文字超寬時 label 會
       改成靠右對齊只露後半段,那個「右緣」看起來就卡在某個字中間。 */
    if (s_voice_box_on || (s_voice_caret >= 0 && s_voice_caret <= input_length))
    {
        /* 語音站是**四行折行**的大框:label 是固定寬 360 的 WRAP,用 OUT_RIGHT_MID 會把游標
           擺到整塊 360 的右邊、垂直置中。改成問 LVGL「插入點那個字」的座標(與立起面板同招)。
           有框選時游標不畫 —— 「選了一段」和「插在某一點」是互斥的兩種狀態。 */
        if (voice_sel_active())
        {
            lv_obj_add_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);
            if (s_voice_caret_ball && lv_obj_is_valid(s_voice_caret_ball))
                lv_obj_add_flag(s_voice_caret_ball, LV_OBJ_FLAG_HIDDEN);
            return;
        }
        int cp = voice_cp_count(input_buffer, voice_caret_bytes());
        lv_point_t p;
        lv_label_get_letter_pos(input_display_label, (uint32_t)cp, &p);
        const lv_font_t *f =
            lv_obj_get_style_text_font(input_display_label, LV_PART_MAIN);
        lv_coord_t lh = lv_font_get_line_height(f);
        /* 從**整棵子樹**refresh 後再讀座標。只 update 這個 label 不夠:label 的 x/y 是
           parent(input_content_container)那一層算出來的,而進語音站的同一輪裡 container
           的尺寸與 label 的對齊(鍵盤站 LEFT_MID → 語音站 TOP_LEFT)才剛被改掉 —— 只
           refresh label 自己,讀到的仍是上一輪的位置。 */
        if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
            lv_obj_update_layout(text_input_bar_bg);
        else
            lv_obj_update_layout(input_display_label);
        lv_coord_t cx = lv_obj_get_x(input_display_label) + p.x;
        lv_coord_t cy = lv_obj_get_y(input_display_label) + p.y;
        lv_obj_set_size(input_cursor, 2, lh);
        /* **先把對齊清掉**：鍵盤站舊路徑用過
           lv_obj_align_to(input_cursor, label, LV_ALIGN_OUT_RIGHT_MID)，LVGL v8 記住那個
           align 之後，lv_obj_set_pos() 給的值會被當成「相對於對齊點的偷移」，
           游標就被推到畫面外 —— 跟先前 label 被推出去是同一個坑
           （founder 2026-08-07：跳到鍵盤後游標失蹤）。 */
        lv_obj_set_align(input_cursor, LV_ALIGN_DEFAULT);
        lv_obj_set_pos(input_cursor, cx, cy);
        lv_obj_clear_flag(input_cursor, LV_OBJ_FLAG_HIDDEN);
        /* 圓球把手只屬於語音站(鍵盤站沒有拖游標這個手勢) */
        if (s_voice_box_on && s_voice_caret_ball &&
            lv_obj_is_valid(s_voice_caret_ball))
        {
            lv_coord_t by = cy + lh + 2;
            lv_coord_t lim = lv_obj_get_height(lv_obj_get_parent(input_cursor)) -
                             VOICE_BALL_D;
            if (by > lim) by = lim;
            lv_obj_set_pos(s_voice_caret_ball, cx + 1 - VOICE_BALL_D / 2, by);
            lv_obj_clear_flag(s_voice_caret_ball, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

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
        /* 對齊要跟**當下這一站**一致。語音站是四行折行的大框(TOP_LEFT),鍵盤站是單行
           藥丸(LEFT_MID)。原本這裡不分站別一律 LEFT_MID,而進站流程是「先進語音站、
           **再**清空」(mouse_open_input_station 的新鮮進場),於是 kbd_bar_set_voice_box
           剛設好的 TOP_LEFT 被這行蓋回垂直置中 —— 游標依 label 位置畫,就落在框的正中央
           偏左而不是左上角(founder 2026-08-18:「第一次進輸入模式游標在中間靠左」)。
           按地球切到鍵盤再切回語音走的是 kbd_lower_switch,會重設 TOP_LEFT 且後面沒有人
           再清空,所以那條路徑一直是對的 —— 這就是「只有第一次」的來由。 */
        if (s_voice_box_on)
            lv_obj_align(input_display_label, LV_ALIGN_TOP_LEFT, 0, 0);
        else
            lv_obj_align(input_display_label, LV_ALIGN_LEFT_MID, 10, 0);
    }
    // 更新游標位置
    update_cursor_position();
    kbd_del_update_icon(); /* 清空之後那顆變「收下」 */
}

/**
 * @brief Update input display with current buffer, scrolling left if needed
 */
static void update_input_display(void)
{
    if (input_display_label == NULL)
        return;

    lv_label_set_text(input_display_label, input_buffer);

    /* 語音站的大框是固定寬折行,不走單行那套「太長就靠右露出最新文字」—— 那會把整塊
       360 寬的 label 推到右邊。對齊在進站時已經設成 TOP_LEFT,這裡不要再動它。 */
    kbd_del_update_icon(); /* 鍵盤站那顆:有字=退格 / 沒字=收下 */
    if (s_voice_box_on)
    {
        update_cursor_position();
        kbd_voice_del_update_icon(); /* 有字=退格 / 沒字=退出 */
        voice_preview_schedule();    /* 把手錶上的字推給電腦那條輸入框(防抖) */
        return;
    }

    /* 先強制重算版面再量寬度 —— 從語音站切過來的那一瞬，label 還是
       語音站的「寬 360 折行」模式，這時問到的寬度與字元座標都是折行版本
       （插入點那個字在第二行、x 很小），算出來的偷移會被夾成「貼左」=
       顯示開頭；要等使用者按一下刪除鍵、版面重算過之後才正確
       （founder 2026-08-07 實測）。 */
    lv_obj_update_layout(input_display_label);
    // 檢查文字寬度是否超過顯示區域
    lv_coord_t text_width = lv_obj_get_width(input_display_label);
    if (text_width > MAX_DISPLAY_WIDTH)
    {
        /* 太長時原本一律靠右 = 永遠只看得到**最後面**那段。插入點在中間時,那個字
           被捲到可視範圍外,游標自然畫不到正確的地方(founder 2026-08-07:「轉到鍵盤
           那也只看得到最後面的文字沒有跟著游標動」)。
           改成**跟著插入點捲**:先問 LVGL 插入點那個字在 label 內的 x,再把 label
           推到讓它落在可視窗內;沒有插入點(-1)才維持舊的靠右露出最新文字。 */
        if (s_voice_caret >= 0 && s_voice_caret <= input_length)
        {
            /* **不能**先 align(RIGHT_MID) 再 set_x() —— LVGL v8 的 align 會把後續
               的 x 當成「相對於對齊點的偏移」,於是整條 label 被從右對齊位置再往右
               推出畫面(2026-08-07 實測:點擊永遠對應到第一個字、刪除時文字一個一個
               從左邊冒出來)。全程用同一種語意:左對齊 + 左偏移。 */
            lv_obj_align(input_display_label, LV_ALIGN_LEFT_MID, 0, 0);
            lv_obj_update_layout(input_display_label);
            uint32_t cp = (uint32_t)voice_cp_count(input_buffer,
                                                   voice_caret_bytes());
            lv_point_t cp_pos;
            lv_label_get_letter_pos(input_display_label, cp, &cp_pos);
            const lv_coord_t pad = 10;
            /* 可視窗 = [pad, pad + MAX_DISPLAY_WIDTH];ox = label 左緣的偏移。
               **插入點置中**(founder 2026-08-07:點 8 跟 9 中間,跳過去要看到
               「45678'9101112」游標在正中間) —— 只做「超出才捲」的話,插入點靠前時
               整條會停在開頭,那正是先前看到的行為。
               兩端夾住:靠近開頭就顯示開頭、靠近結尾就顯示結尾,中間才置中。 */
            lv_coord_t ox = pad + MAX_DISPLAY_WIDTH / 2 - cp_pos.x;
            lv_coord_t min_ox = pad + MAX_DISPLAY_WIDTH - text_width; /* 尾巴貼右 */
            if (ox < min_ox) ox = min_ox;
            if (ox > pad) ox = pad;                                   /* 頭貼左 */
            lv_obj_align(input_display_label, LV_ALIGN_LEFT_MID, ox, 0);
        }
        else
        {
            // 沒有插入點:維持舊行為,靠右露出最新輸入的文字
            lv_obj_align(input_display_label, LV_ALIGN_RIGHT_MID, -10, 0);
        }
    }
    else
    {
        // 文字未超過寬度，居中顯示
        lv_obj_align(input_display_label, LV_ALIGN_LEFT_MID, 10, 0);
    }

    // 更新游標位置
    update_cursor_position();
    /* 鍵盤站也要推 —— 這行原本只在上面 s_voice_box_on 那個分支裡,而那個分支結尾就
       return,所以「語音輸入的字有同步到電腦、切到鍵盤打的字沒有」(founder
       2026-08-17)。文字真相是同一個 input_buffer,在哪一站顯示不改變電腦那條輸入框
       該鏡射什麼;防抖 250ms 本來就擋著連續按鍵。 */
    voice_preview_schedule();
}

/**
 * @brief Add character to input buffer
 */
static void add_to_input_buffer(const char *text)
{
    if (text == NULL || input_length >= sizeof(input_buffer) - 1)
        return;

    size_t text_len = strlen(text);
    if (input_length + text_len >= sizeof(input_buffer) - 1)
        return;

    /* 有插入點就插在那裡(語音站點過字、或切到鍵盤後沿用同一個插入點),
       沒有(-1)才接在最後。原本一律 strcat,所以在中間點好游標再打字,字還是
       跑到尾巴去(founder 2026-08-07)。刪除那條 remove_from_input_buffer 早就
       依插入點在刪了,這邊補齊對稱。 */
    if (s_voice_caret >= 0 && s_voice_caret <= input_length)
    {
        memmove(input_buffer + s_voice_caret + text_len,
                input_buffer + s_voice_caret,
                (size_t)(input_length - s_voice_caret) + 1);
        memcpy(input_buffer + s_voice_caret, text, text_len);
        s_voice_caret += (int)text_len; /* 游標跟到剛打的字後面 */
    }
    else
    {
        strcat(input_buffer, text);
    }
    input_length += text_len;
    update_input_display();
}

/**
 * @brief Remove last character from input buffer (backspace)
 */
/* 語音站:刪掉框選的那一段。刪完插入點停在該段起點。 */
static void voice_delete_selection(void)
{
    if (!voice_sel_active())
        return;
    int a = voice_byte_of_cp(input_buffer, s_voice_sel_from_cp);
    int b = voice_byte_of_cp(input_buffer, s_voice_sel_to_cp);
    if (b > input_length) b = input_length;
    if (a >= b) { voice_sel_clear(); return; }
    memmove(input_buffer + a, input_buffer + b, (size_t)(input_length - b) + 1);
    input_length -= (b - a);
    s_voice_caret = a;
    voice_sel_clear();
    if (input_length == 0)
        mouse_v2t_locked = false;
    update_input_display();
}

static void remove_from_input_buffer(void)
{
    /* 有框選就整段刪;否則刪**插入點前**一個字(不是永遠刪最後一個)。
       2026-08-07:條件從 s_voice_box_on 改成「插入點有效」—— 游標現在會跟著人從
       語音站帶進鍵盤站(見 kbd_bar_set_voice_box / add_to_input_buffer),但這裡
       還在看「是不是語音站」,所以在鍵盤station 打字插得進中間、刪除卻還是從
       尾巴刪(founder 2026-08-07)。兩邊改用同一個判準才對稱。 */
    if (s_voice_box_on || (s_voice_caret >= 0 && s_voice_caret <= input_length))
    {
        if (voice_sel_active()) { voice_delete_selection(); return; }
        int cut = voice_caret_bytes();
        if (cut <= 0)
            return; /* 插入點在最前面,前面沒東西可刪 */
        int prev = cut - 1;
        while (prev > 0 && ((unsigned char)input_buffer[prev] & 0xC0) == 0x80)
            prev--;
        memmove(input_buffer + prev, input_buffer + cut,
                (size_t)(input_length - cut) + 1);
        input_length -= (cut - prev);
        s_voice_caret = prev;
        if (input_length == 0)
            mouse_v2t_locked = false;
        update_input_display();
        return;
    }
    if (input_length > 0)
    {
        /* 退一個**字**,不是退一個 byte。中文一字 3 bytes,原本的寫法會切出半個字
           (畫面變亂碼)。UTF-8 續接位元組都是 10xxxxxx,往前跳過它們就回到字首。
           語音輸入進來的幾乎都是中文,所以這條在語音站是天天會踩到的。 */
        int cut = input_length - 1;
        while (cut > 0 && ((unsigned char)input_buffer[cut] & 0xC0) == 0x80)
            cut--;
        input_buffer[cut] = '\0';
        input_length = cut;
        if (input_length == 0)
            mouse_v2t_locked = false; // backspace 刪到清空後解鎖 V2T
        update_input_display();
    }
}

/**
 * @brief Helper: set translate_y style (used as lv_anim exec callback)
 */
extern void set_stop_mouse_move(bool stop);
/* 觸控板 vs 飛鼠先到先贏互鎖(bloc_motion_tracking.c,founder 2026-07-24)。宣告放這麼前面
   是因為 handle_pressing_event 送游標前要先問 mouse_air_cursor_owned()。 */
extern bool mouse_air_cursor_owned(void);
extern void bloc_touch_cursor_claim(void);
extern void bloc_cursor_owner_reset(void);
extern void bloc_touch_finger_active(void); /* 手指在滑的「前哨」信號:比正式 claim 更早擋飛鼠 */
/* 語音站 ⇄ 鍵盤改成左右切換(founder 2026-08-07)用的 helper。原本是鍵盤由下
   往上升 / 往下降(translate_y),語音區只是硬 show/hide —— 兩站是「並排的兩頁」
   而不是上下疊,左右滑比較符合它的心智模型。輸入框本身的 y 位移(語音站置中
   大框 ↔ 鍵盤上方細列)是兩站真實的版面差,不屬於切換動作,維持不動。 */
static void anim_set_translate_x(void *obj, int32_t v)
{
    lv_obj_t *t = (lv_obj_t *)obj;
    if (t && lv_obj_is_valid(t))
        lv_obj_set_style_translate_x(t, (lv_coord_t)v, 0);
}

/* 進場動畫結束：**一定把位移歸零**。動畫若被取消/沒跑，容器會停在起點
   (+466 = 螢幕外)，而 find_closest_key() 是「沒有距離上限的最近鍵」—— 所有鍵都在
   螢幕外時，按哪裡都會被判給同一顆鍵(實測:按空白全部變地球)。 */
static void kbd_slide_in_done_cb(lv_anim_t *a)
{
    lv_obj_t *obj = (lv_obj_t *)a->var;
    if (obj && lv_obj_is_valid(obj))
        lv_obj_set_style_translate_x(obj, 0, 0);
}

/* 舊鍵盤滑出畫面後直接刪掉(切輸入法時新舊兩塊並存滑動,見下面 Mode 分支) */
static void kbd_del_after_slide_x_cb(lv_anim_t *a)
{
    lv_obj_t *obj = (lv_obj_t *)a->var;
    if (obj && lv_obj_is_valid(obj))
        lv_obj_del(obj);
}

/* 滑出畫面後收起來,並把位移歸零(下次進場自己重設起點) */
static void hide_after_slide_x_cb(lv_anim_t *a)
{
    lv_obj_t *obj = (lv_obj_t *)a->var;
    if (obj && lv_obj_is_valid(obj))
    {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_x(obj, 0, 0);
    }
}

static void anim_set_translate_y(void *obj, int32_t v)
{
    lv_obj_set_style_translate_y((lv_obj_t *)obj, v, 0);
}

/* 垂直版的兩支收尾(對應 kbd_slide_in_done_cb / hide_after_slide_x_cb)。
   語音⇄鍵盤的換站改成「鍵盤上下進出」之後要用這兩支,理由同橫向版:動畫若被取消,
   容器會停在螢幕外,而 find_closest_key() 沒有距離上限,按哪裡都會判給同一顆鍵。 */
static void kbd_slide_in_y_done_cb(lv_anim_t *a)
{
    lv_obj_t *obj = (lv_obj_t *)a->var;
    if (obj && lv_obj_is_valid(obj))
        lv_obj_set_style_translate_y(obj, 0, 0);
}

static void hide_after_slide_y_cb(lv_anim_t *a)
{
    lv_obj_t *obj = (lv_obj_t *)a->var;
    if (obj && lv_obj_is_valid(obj))
    {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_y(obj, 0, 0);
    }
}

/* 輸入框在兩站之間**長大 / 縮小**(founder 2026-08-18:「鍵盤往下收掉,同時上面的輸入框
   變大成語音輸入的輸入框;切成鍵盤就反著來」)。原本兩站是左右換頁、框的尺寸瞬間替換,
   看起來像兩個不同的東西輪播;改成同一個框在兩種幾何之間補間,才讀得出是同一條輸入框。
   四個維度一起補間(x/y/w/h):寬度不同 → x 也要跟著,否則長大的過程會偏一邊。
   起訖值放 static:LVGL 的 anim exec 只帶得進一個 int32 進度值。 */
static lv_coord_t s_barmorph_x0, s_barmorph_y0, s_barmorph_w0, s_barmorph_h0;
static lv_coord_t s_barmorph_x1, s_barmorph_y1, s_barmorph_w1, s_barmorph_h1;
static lv_coord_t s_barmorph_r0 = 100, s_barmorph_r1 = 100;
/* 語音框的底圖(定義在下方的語音站段落),收尾 cb 要用 —— 前向宣告,別為了一支
   callback 把整段搬家。 */
static lv_obj_t *s_bar_voice_frame;
/* 語音框的圓角 = 卡片圖(message_widget_bg)實際的轉角半徑,長大過程用框自己的圓角逼近它,
   落定才換成真正的圖。**80 是量出來的**:取那張 442x252 PNG 左上角 alpha 邊緣的取樣點做
   圓弧擬合(x=4→y=55、x=8→y=45、x=16→y=32、x=36→y=13),r=80 幾乎完全吻合。
   先前憑印象寫 30,交棒瞬間轉角明顯對不上(founder 2026-08-18:「角落的弧形跟圖片不太
   一樣」)。要改先重量,別再猜。 */
#define VOICE_BOX_RADIUS 80

static lv_coord_t barmorph_lerp(lv_coord_t a, lv_coord_t b, int32_t v)
{
    return (lv_coord_t)(a + ((int32_t)(b - a) * v) / 100);
}

static void kbd_bar_morph_cb(void *var, int32_t v)
{
    lv_obj_t *bar = (lv_obj_t *)var;
    if (!bar || !lv_obj_is_valid(bar)) return;
    lv_obj_set_size(bar,
                    barmorph_lerp(s_barmorph_w0, s_barmorph_w1, v),
                    barmorph_lerp(s_barmorph_h0, s_barmorph_h1, v));
    lv_obj_set_pos(bar,
                   barmorph_lerp(s_barmorph_x0, s_barmorph_x1, v),
                   barmorph_lerp(s_barmorph_y0, s_barmorph_y1, v));
    lv_obj_set_style_radius(bar, barmorph_lerp(s_barmorph_r0, s_barmorph_r1, v),
                            LV_PART_MAIN);
}

/* 長大成語音框的收尾:這時才換上語音站真正的外觀(框自己不畫、改由卡片圖呈現)。
   **為什麼不能在動畫開始就換**:語音框的 bg_opa 是透明的,外觀全靠那張固定尺寸的卡片圖,
   而子物件會被裁切到父容器 —— 框還小的時候只露得出卡片中央那一小塊(幾乎沒有內容),
   看起來就是「框在變大的過程中整個消失」(founder 2026-08-18)。縮回鍵盤那個方向沒這問題,
   因為鍵盤那套外觀是框**自己**畫的,一路都在。 */
static void kbd_bar_morph_to_voice_done_cb(lv_anim_t *a)
{
    lv_obj_t *bar = (lv_obj_t *)a->var;
    if (!bar || !lv_obj_is_valid(bar)) return;
    if (!s_voice_box_on) return; /* 動畫還沒跑完就又切走了 → 別把語音外觀套到鍵盤站 */
    lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
    if (s_bar_voice_frame && lv_obj_is_valid(s_bar_voice_frame))
    {
        lv_obj_clear_flag(s_bar_voice_frame, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_background(s_bar_voice_frame);
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

        // Show keyboard off-screen (translate_y pushes it below visible area)
        lv_obj_clear_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_y(keyboard_container, 300, 0);

        // Target positions for open state
        int32_t open_x = (LV_HOR_RES_MAX - 310) / 2; /* 置中(founder 2026-07-22;
            舊 -35 偏移=歷史殘留,在 y64 會讓左端超出圓弧) */
        int32_t open_y = 64; /* =手寫候選欄同高(founder 2026-07-22:兩模式共用一條輸入列) */

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
    /* 2026-08-07 founder:在鍵盤長按空白鍵 → 除了開麥克風,畫面也要跳到語音站,
       手指按著的期間等同按著麥克風鈕(放開就結束,見 handle_proximity_input 的
       RELEASED/PRESS_LOST)。這支是空白鍵長按那條路徑的 handler(mic 鈕自己走
       mouse_v2t_open),所以只在還停在鍵盤那一站時才切。 */
    if (kbd_lower_is_keyboard)
    {
        s_space_hold_v2t = true; /* 這一輪是空白鍵長按進來的 → 放開就結束 */
        kbd_lower_switch(false);
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
            /* down_arrow 也算 Del:空框時這顆鍵**顯示成「收下」但身分不變**
               (kbd_del_update_icon)。這支函式是靠子圖的 src 反查鍵名的,漏了這條的話
               換圖等於把鍵名一起換掉,RELEASED 時比不中任何分支 → 按下去毫無反應
               (founder 2026-08-18:「我按他怎麼沒有退出輸入模式」)。
               擺在下面那條 strcmp(src, DOWN_ARROW) **之前**還有一個好處:那條是拿
               lv_img_dsc_t 的指標當字串比,先在這裡攔下來就不會走到那個未定義行為。 */
            else if (src == &backspace_icon || src == &down_arrow)
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
 * @brief Register a key button for proximity detection
 */
static void register_key_button(lv_obj_t *btn)
{
    if (all_keys_count < MAX_KEY_BUTTONS)
    {
        all_keys[all_keys_count] = btn;
        all_keys_count++;
    }
    else
    {
        /* 滿了就丟掉是這個 bug 的根源，而且以前完全無聲。至少讓它叫出來。 */
        LOG_E("[kbd] key registry FULL (%d) — key dropped, it will not respond",
              MAX_KEY_BUTTONS);
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

        /* 直接問 LVGL 的實際絕對座標。原本是自己重算
             btn_x = lv_obj_get_x(btn) + lv_obj_get_x(keyboard_container)
           —— 但 keyboard_container 有 pad_all=8，而子物件的 set_pos 是相對
           **內容區**(已含 padding)，這裡漏了那 8px → 每一顆鍵的判定框都比
           畫面位置往左上偏 8px。50×50 的鍵偏 8px = 16%，按邊緣就落空或被
           判給隔壁，正是「有時候才有反應」(founder 2026-08-07)。
           用 lv_obj_get_coords 還能順便吃到 align / translate 造成的位移。 */
        lv_area_t btn_area;
        lv_obj_get_coords(btn, &btn_area);
        lv_coord_t btn_x = btn_area.x1;
        lv_coord_t btn_y = btn_area.y1;
        lv_coord_t btn_width = btn_area.x2 - btn_area.x1 + 1;
        lv_coord_t btn_height = btn_area.y2 - btn_area.y1 + 1;

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
            /* 注音模式的空白鍵：触控範圍**往上**擴大，視覺位置不動
               (founder 2026-08-07)。注音版面把空白鍵下移到 y=235(絕對 409-459)，
               已經貼著圓形螢幕下緣(y=459 那條線可視寬度只剩 x176-290)，手指很難
               壓實；而 ㄈ/ㄥ/ㄦ 只借用第四行的**兩側**，中間那塊(原本空白鍵的位置)
               是空的 —— 往上擴就是把那塊還給它，不會搞到別人。
               就算真的跟某顆注音鍵重疊也安全：本函式是「第一個包含該點的鍵」
               勝出，而注音鍵的註册順序在第四行之前，仍然優先。 */
            lv_coord_t top_ext = 0;
            if (current_keyboard_mode == KEYBOARD_MODE_CHINESE &&
                strcmp(btn_text, "Space") == 0)
            {
                top_ext = 45;
            }
            is_in_bounds = (touch_point.x >= btn_x &&
                            touch_point.x <= (btn_x + btn_width) &&
                            touch_point.y >= (btn_y - top_ext) &&
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

/* 鍵盤模式下，**畫在鍵盤之上**的底部浮動物件必須全部藏好 —— 它們都是 bg 的直接
   子物件，z-order 在 mode_container 之上，任一個漏掉就會蓋住最後一排(空白/地球/
   刪除)。它們的 cb 雖然都寫了「鍵盤模式就 early-return」，但**早退不等於放行**:
   物件仍是 clickable，LVGL 照樣把 press 派給它，底下的鍵盤根本收不到事件
   (founder 2026-08-07:中文空白鍵完全沒反應，log 連 [hit] 都沒有)。
   中文佈局把空白鍵下移到 y≈401..451 讓位給ㄈㄥㄦ，正好落進這條 chrome 帶，
   英文的在 y≈361..411 大半在帶子上緣之外 —— 這就是「只有中文按不到」。
   顯藏邏輯散在好幾條進場路徑上，統一由這裡掃。 */
/* 鍵盤的鍵是不是正露在畫面上(不看模式旗標 —— 進場路徑有十來條,旗標的翻面時機
   各不相同,曾經因此掃不到)。 */
static bool kbd_keys_showing(void)
{
    if (!keyboard_container || !lv_obj_is_valid(keyboard_container))
        return false;
    if (lv_obj_has_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN))
        return false;
    lv_area_t a;
    lv_obj_get_coords(keyboard_container, &a);
    return a.x1 < LV_HOR_RES && a.x2 > 0; /* 換頁動畫停在畫面外的不算 */
}

/* 輸入畫面**當前這一站**的作用區(鍵盤站=鍵盤容器;語音站=麥克風那一段)。
   兩站都有自己的底部按鈕、也都會被外來浮層蓋住,所以判準不能只寫鍵盤
   (founder 2026-08-07:中文空白鍵通了,但長按空白跳到語音站後,下面的刪除與切換
   又按不到 —— 語音站裡 keyboard_container 是藏的，防護整個沒生效)。 */
static lv_obj_t *kbd_input_active_area(void)
{
    /* 前兩條是**輸入模式**的作用區,一定要先確認人真的在輸入模式(founder 2026-08-17:
       「又按不到了,他好像是變成按到觸控板」)。
       真兇:`s_voice_box_on` 只在 kbd_bar_set_voice_box() 寫,而回觸控板的
       kbd_commit_to_trackpad() 從來沒清過它,卻**刻意**把 kbd_mic_section 的 HIDDEN
       清掉(為了下次進場重新從語音站開始)。於是「進過一次語音站再回觸控板」之後:
       旗標卡 true + section 可見 → 這裡回傳 kbd_mic_section(380 x 全高、置中),而
       s_top_logo(BOTTOM_MID,x≈177..289)整顆落在那個範圍內 → chrome_hit_test_cb 把
       **每一次**按下都判定為「讓給輸入區」→ tap 穿過去變成觸控板拖曳,skaibar_img
       從此點不動、連 CLICKED 都不會發出(所以 [logo] 探針也印不出來)。
       用 mode 當閘門是結構性的修法:不管哪個旗標日後又走味,人在觸控板上就絕不可能
       armed 出一個輸入區來擋自己。 */
    if (current_hid_mode == HID_MODE_KEYBOARD)
    {
        if (kbd_keys_showing())
            return keyboard_container;
        if (s_voice_box_on && kbd_mic_section && lv_obj_is_valid(kbd_mic_section) &&
            !lv_obj_has_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN))
            return kbd_mic_section;
    }
    /* 觸控板站:底部那顆「進輸入」圖示同病 —— 全螢幕的觸控板感應面有時排在它上面，
       按下就被吃掉，症狀是「圖示只有靠上一小塊按得到」(founder 2026-08-07,
       [press] 坐實 obj area=(0,0)-(465,465) 接走)。保護範圍只取圖示**本體**座標，
       不含 ext_click_area 那 34px 外圈 —— 否則觸控板中下方會多出一塊 131×131
       的死區，從那裡起手的拖曳會變成點圖示。 */
    if (current_hid_mode == HID_MODE_TRACKPAD && s_top_logo &&
        lv_obj_is_valid(s_top_logo) &&
        !lv_obj_has_flag(s_top_logo, LV_OBJ_FLAG_HIDDEN))
        return s_top_logo;
    return NULL;
}

/* 底部 chrome(鍵盤入口圖 / skaibar 觸控帶 / 頂部下拉感應區)與輸入區重疊時,在
   **按下的當下**自己拒絕命中,LVGL 會繼續往下找到底下的鍵/按鈕。
   為什麼不靠顯藏:「鍵盤露出來」有十來條進場路徑,每條都要記得藏這幾個物件,漏一條
   就整排按不到,而且 cb 裡的 early-return 不算放行 —— 物件只要還 hit 得到,press
   就停在它身上(founder 2026-08-07:中文空白鍵連 [hit] 都印不出來)。
   hit-test 是每次按下現算的,跟旗標同不同步無關。 */
static void chrome_hit_test_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_HIT_TEST)
        return;
    lv_hit_test_info_t *info = lv_event_get_param(e);
    if (!info || !info->point) return;
    lv_obj_t *act = kbd_input_active_area();
    if (!act) return;
    /* 作用區自己(或它的子孫)不能對自己放行 —— 觸控板站的作用區就是那顆圖示本身，
       而它身上正掛著這支 cb(它同時也是別人的攔截者)，不擋掉就會拒絕自己的命中。 */
    for (lv_obj_t *q = lv_event_get_target(e); q; q = lv_obj_get_parent(q))
        if (q == act) return;
    /* 只在**按下點真的落在作用區內**才放行 —— 浮層伸出作用區以外的部分照常可按，
       不會因為輸入畫面開著就整個失效。 */
    lv_area_t aa;
    lv_obj_get_coords(act, &aa);
    if (_lv_area_is_point_on(&aa, info->point, 0))
        info->res = false;
}

/* 鍵盤露出時，把**任何**壓在鍵盤範圍上的可點擊物件都掛上「按下時放行」。
   為什麼要通用掃描而不是一個一個點名：實測吃掉按下的物件一路從底部鍵盤圖示、
   skaibar 觸控帶，追到 lv_layer_top() 上的 mic_bar、再到螢幕層底部那條 y437..465
   的橫帶(founder 2026-08-07，[press] 探針逐一坐實)——它們分屬不同模組、不同圖層，
   而且會隨浮層顯示與否來去，這正是「有時候擋有時候不擋」的來源。
   由 40ms poll 驅動，晚一步才出現的浮層下一拍就會被蓋到。 */
static void kbd_guard_overlappers(void)
{
    lv_obj_t *act = kbd_input_active_area();
    if (!act) return;
    lv_area_t ka;
    lv_obj_get_coords(act, &ka);

    lv_obj_t *roots[] = { lv_scr_act(), lv_layer_top(), lv_layer_sys() };
    for (unsigned r = 0; r < sizeof(roots) / sizeof(roots[0]); r++)
    {
        if (!roots[r]) continue;
        uint32_t n1 = lv_obj_get_child_cnt(roots[r]);
        for (uint32_t i = 0; i < n1; i++)
        {
            lv_obj_t *c1 = lv_obj_get_child(roots[r], i);
            uint32_t n2 = lv_obj_get_child_cnt(c1);
            for (uint32_t j = 0; j <= n2; j++)
            {
                lv_obj_t *o = (j == 0) ? c1 : lv_obj_get_child(c1, j - 1);
                if (!o) continue;
                if (lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN)) continue;
                if (!lv_obj_has_flag(o, LV_OBJ_FLAG_CLICKABLE)) continue;
                if (lv_obj_has_flag(o, LV_OBJ_FLAG_ADV_HITTEST)) continue;
                /* 作用區自己(含裡面的鍵/按鈕)不能放行 */
                bool inside_act = false;
                for (lv_obj_t *q = o; q; q = lv_obj_get_parent(q))
                    if (q == act) { inside_act = true; break; }
                if (inside_act) continue;
                /* 全螢幕的感應面也要納管 —— 觸控板那面就是 (0,0)-(465,465)，
                   正是吃掉底部圖示的元兇。這是安全的:lv_indev_search_obj() 先遞迴
                   搜子物件、才用 hit_test 決定自己能不能認領，所以拒絕命中不會連帶
                   擋掉底下的東西(已對 LVGL v8 原始碼確認)。而且放行只在作用區那個
                   小矩形內生效，其餘範圍照常。 */
                lv_area_t oa;
                lv_obj_get_click_area(o, &oa);
                if (oa.x2 < ka.x1 || oa.x1 > ka.x2 || oa.y2 < ka.y1 ||
                    oa.y1 > ka.y2)
                    continue;
                lv_obj_add_flag(o, LV_OBJ_FLAG_ADV_HITTEST);
                lv_obj_add_event_cb(o, chrome_hit_test_cb, LV_EVENT_HIT_TEST,
                                    NULL);
            }
        }
    }
}

static void kbd_hide_bottom_chrome(void)
{
    struct { lv_obj_t *o; const char *n; } chrome[] = {
        { s_top_logo,         "bottom keyboard icon" },
        { bottom_swipe_area,  "bottom swipe area" },
        { kbd_exit_btn,       "kbd exit btn" },
        { s_ai_drag_logo,     "ai drag logo" },
        { status_bar_area_up, "top pull zone" },
    };
    for (unsigned k = 0; k < sizeof(chrome) / sizeof(chrome[0]); k++)
    {
        lv_obj_t *o = chrome[k].o;
        if (o == NULL || !lv_obj_is_valid(o)) continue;
        if (lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN)) continue;
        LOG_W("keyboard: %s was visible over keys -> hide", chrome[k].n);
        lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 鍵盤站的刪除鍵:框裡沒字就是「收下」鍵(founder 2026-08-18),與語音站那顆同一條規則
   (kbd_voice_del_update_icon)。注音組字中(s_py_len>0)仍算有內容可刪 —— 那時刪的是還沒
   上屏的符號,把它變成收下會讓使用者退不掉組字。 */
static bool kbd_del_acts_as_exit(void)
{
    return (input_length == 0 && s_py_len == 0);
}

static void kbd_del_update_icon(void)
{
    if (del_img == NULL || !lv_obj_is_valid(del_img))
        return;
    lv_img_set_src(del_img, kbd_del_acts_as_exit() ? (const void *)&down_arrow
                                                   : (const void *)&backspace_icon);
}

/**
 * @brief Handle proximity-based input
 */
static void create_circular_keyboard_layout(lv_obj_t *parent);
static void handle_proximity_input(lv_event_t *e)
{
    /* 已經不在鍵盤那一站了 → 不再追鍵、不再彈提示框。
       長按空白鍵進語音站時,手指**還按著**,而鍵盤正往左滑出:PRESSING 每一幀都重跑
       find_closest_key(),鍵在手指底下移動,於是一路判給不同的字母並彈出它的提示框
       —— 切換過程中會看到鍵盤的文字提示框閃過(founder 2026-08-18)。
       RELEASED / PRESS_LOST **不擋**:空白鍵長按那條流程要靠它們結束錄音(放開就停),
       擋掉會變成放開後麥克風還開著。 */
    lv_event_code_t code0 = lv_event_get_code(e);
    if ((code0 == LV_EVENT_PRESSED || code0 == LV_EVENT_PRESSING) &&
        !kbd_lower_is_keyboard)
    {
        hide_key_popup();
        if (currently_pressed_btn != NULL)
        {
            lv_obj_clear_state(currently_pressed_btn, LV_STATE_PRESSED);
            currently_pressed_btn = NULL;
        }
        closest_btn = NULL;
        return;
    }
    if (lv_event_get_code(e) == LV_EVENT_PRESSED)
    {
        lv_indev_t *di = lv_indev_get_act();
        (void)di;
        /* 保險:鍵盤容器若不在畫面上，比對會全面錯亂 —— find_closest_key()
           是「沒有距離上限的最近鍵」，所有鍵都在螢幕外時，按哪裡都會被判給
           同一顆(實測:按空白全部變地球、一直切輸入法)。
           **看實際座標**而不是 translate —— 前一版只檢查 translate，但實測
           translate=0 而座標却在 x466，保險完全沒接到。 */
        /* 鍵盤模式下，**畫在鍵盤之上**的底部浮動物件必須全部藏好 ——
           它們都是 bg 的直接子物件， z-order 在 mode_container 之上，任一個漏掉
           就會蓋住最後一排（空白/地球/刪除），手感就是「有東西擋在上面」
           (founder 2026-08-07)。各自的顯藏邏輯分散在好幾條路徑上，這裡統一掃一次，
           並把真的漏掉的那個記下來。 */
        kbd_hide_bottom_chrome();
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
        {
            lv_area_t ka;
            lv_obj_get_coords(keyboard_container, &ka);
            if (ka.x1 != 0 || ka.x2 < 0 || ka.x1 >= LV_HOR_RES)
            {
                lv_anim_del(keyboard_container, anim_set_translate_x);
                lv_obj_set_style_translate_x(keyboard_container, 0, 0);
                lv_obj_set_style_translate_y(keyboard_container, 0, 0);
                lv_obj_set_x(keyboard_container, 0);
                lv_obj_align(keyboard_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                lv_obj_update_layout(keyboard_container);
                LOG_W("keyboard container was off-screen -> geometry reset");
            }
        }
    }
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

                /* 每一顆鍵的觸覺回饋(founder 2026-08-07)。掛在這裡是因為所有鍵盤
                   (英文/數字/符號/注音)都走同一條 proximity 判定,包含空白/地球/
                   刪除 —— 不必逐鍵接線,也不會漏掉哪個佈局。 */
                motor_pattern_key_tick();

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
                        // 恢復Del按鈕的原始圖片(空框時那是「收下」不是退格)
                        kbd_del_update_icon();
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
                    // 恢復Del按鈕的原始圖片(空框時那是「收下」不是退格)
                    kbd_del_update_icon();
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
    else if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) &&
             proximity_mode_active)
    {
        // 停止長按定時器
        stop_long_press_timer();

        /* 長按空白鍵進來的錄音:放開就結束並上屏(等同放開麥克風鈕)。
           PRESS_LOST 也要收 —— 長按當下畫面已切到語音站,鍵盤容器被動畫收走時
           LVGL 送的是 PRESS_LOST 而不是 RELEASED。
           只認 s_space_hold_v2t:麥克風鈕、語音站長按輸入框那些 session 不歸這裡
           管,它們有自己的結束方式(放開鈕 / 再按一次)。 */
        if (s_space_hold_v2t)
        {
            s_space_hold_v2t = false;
            if (mouse_v2t_active)
            {
                mouse_v2t_close_and_paste();
            }
        }

        // 清除按鈕的按下狀態
        if (currently_pressed_btn != NULL)
        {
            const char *prev_key_text = get_button_text(currently_pressed_btn);
            if (strcmp(prev_key_text, "Del") == 0 && del_img != NULL)
            {
                // 恢復Del按鈕的原始圖片(空框時那是「收下」不是退格)
                kbd_del_update_icon();
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
                    /* 舊鍵盤「點空白/下箭頭=收鍵盤」殘留,founder 2026-07-22 拔除:
                       proximity 找鍵無距離上限,點空白會誤中它;退出只走頂部
                       退出鈕與 bar 下拉。 */
                }
                else if (strcmp(closest_key_text, "Mode") == 0)
                {
                    /* 輸入法循環:英文→中文→數字→**語音**→英文。
                       founder 2026-08-03:第四站原位換掉手寫 —— 手寫的程式整段保留,
                       只是循環不再走到它(hw_open_from_mode_switch 因此變成無呼叫者)。 */
                    bool to_voice = false;
                    if (current_keyboard_mode == KEYBOARD_MODE_LETTERS)
                    {
                        current_keyboard_mode = KEYBOARD_MODE_CHINESE;
                    }
                    else if (current_keyboard_mode == KEYBOARD_MODE_CHINESE)
                    {
                        kbd_pinyin_clear();
                        current_keyboard_mode = KEYBOARD_MODE_NUMBERS;
                    }
                    else
                    {
                        to_voice = true;
                    }
                    if (to_voice)
                    {
                        /* 數字→語音:**不離開鍵盤模式**,只把下半部從鍵盤換成 mic 區 ——
                           上方那條輸入列共用,這就是 founder 要的「嵌在鍵盤區塊裡」
                           (對比手寫是另開一個接管整個錶面的 view)。mode 變數維持
                           NUMBERS = 已建好的布局;「回英文」由 mic 區那顆鍵盤鈕錨定
                           (kbd_lower_arrow_event_cb),循環閉合。
                           走 kbd_lower_switch 而不是 kbd_lower_set_keyboard:輸入列的
                           高度(鍵盤站 y=75 / 語音站 y=195)是那裡的動畫換的,只切可見性
                           會讓輸入列卡在鍵盤的高度。 */
                        kbd_lower_switch(false);
                    }
                    else if (keyboard_container != NULL)
                    {
                        /* 切輸入法(英文→中文→數字)也走左右換頁(founder
                           2026-08-07),方向與「語音→鍵盤」一致:新的從右邊進來、
                           舊的往左滑走。舊的先從 keyboard_container 脫鉤(新的立刻
                           接手當前鍵盤),滑完才刪 —— 兩塊短暫並存才有得滑。
                           舊的順手拿掉 CLICKABLE:它身上還掛著 proximity 的事件
                           cb,滑動途中若吃到按壓會打到已經作廢的鍵位表。 */
                        lv_obj_t *parent =
                            lv_obj_get_parent(keyboard_container);
                        lv_obj_t *old_kbd = keyboard_container;
                        bool was_visible =
                            !lv_obj_has_flag(old_kbd, LV_OBJ_FLAG_HIDDEN);

                        /* 2026-08-16 真機當機(founder「切到下個鍵盤才當」):並存的
                           新舊兩塊輪盤(~21K)+中文站首批 CJK glyph(FT clean_cache
                           560K/700K 同刻觸發)+雙輪盤 EPIC 合成,把主 heap 從 ~41K
                           一路吃到 0 → data_proxy_process `RT_NULL != body` assert
                           重開。free 不到 60K 就放棄換頁動畫,先刪舊再建新(同
                           Caps 鍵路徑) —— 峰值少一塊輪盤、合成面積也減半。 */
                        bool slide_ok;
                        {
                            rt_uint32_t kb_total = 0, kb_used = 0, kb_mx = 0;
                            rt_memory_info(&kb_total, &kb_used, &kb_mx);
                            slide_ok = (kb_total - kb_used >= 60 * 1024);
                            LOG_W("[kbd] mode-switch free=%u slide=%d",
                                  (unsigned)(kb_total - kb_used), (int)slide_ok);
                        }
                        if (!slide_ok)
                        {
                            lv_obj_del(old_kbd);
                            old_kbd = NULL;
                        }

                        keyboard_container = NULL;
                        create_circular_keyboard_layout(parent);
                        if (was_visible)
                        {
                            lv_obj_clear_flag(keyboard_container,
                                              LV_OBJ_FLAG_HIDDEN);
                            keyboard_visible = true;
                            update_input_display();
                            start_cursor_blink();
                        }
                        if (!was_visible || old_kbd == NULL)
                        {
                            if (old_kbd)
                                lv_obj_del(old_kbd);
                        }
                        else
                        {
                            lv_obj_clear_flag(old_kbd, LV_OBJ_FLAG_CLICKABLE);
                            lv_anim_del(old_kbd, anim_set_translate_x);
                            lv_anim_t ao;
                            lv_anim_init(&ao);
                            lv_anim_set_var(&ao, old_kbd);
                            lv_anim_set_exec_cb(&ao, anim_set_translate_x);
                            lv_anim_set_values(&ao, 0, -LV_HOR_RES);
                            lv_anim_set_time(&ao, 220);
                            lv_anim_set_path_cb(&ao, lv_anim_path_ease_out);
                            lv_anim_set_ready_cb(&ao, kbd_del_after_slide_x_cb);
                            lv_anim_start(&ao);

                            lv_obj_set_style_translate_x(keyboard_container,
                                                         LV_HOR_RES, 0);
                            lv_anim_t an;
                            lv_anim_init(&an);
                            lv_anim_set_var(&an, keyboard_container);
                            lv_anim_set_exec_cb(&an, anim_set_translate_x);
                            lv_anim_set_values(&an, LV_HOR_RES, 0);
                            lv_anim_set_time(&an, 220);
                            lv_anim_set_path_cb(&an, lv_anim_path_ease_out);
                            lv_anim_start(&an);
                        }
                        kbd_cand_refresh(); /* 切進中文=數字快捷列/切走=藏 */
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
                    kbd_pinyin_clear(); /* 殘留拼音丟棄(不自動上屏) */
                    kbd_cand_refresh(); /* 中文模式回數字快捷列 */
                    control_provider.ble_hid_keyboard_input("\n");
                    clear_input_display(); // 按下 Enter 清除輸入
                }
                else if (strcmp(closest_key_text, "Del") == 0)
                {
                    LOG_D("Delete key pressed");
                    /* 空框 = 這顆是「收下」鍵 → 離開輸入模式回觸控板,與語音站那顆
                       (kbd_voice_del_event_cb)以及送出後的去處一致。 */
                    if (kbd_del_acts_as_exit())
                    {
                        LOG_I("[kbd] delete key acted as EXIT (box empty)");
                        mouse_exit_input_station();
                        return;
                    }
                    if (current_keyboard_mode == KEYBOARD_MODE_CHINESE &&
                        s_py_len > 0)
                    {
                        /* 組字中:刪最後一個注音符號(UTF-8 尾刪),不動已上屏文字 */
                        while (s_py_len > 0 &&
                               ((uint8_t)s_py_buf[s_py_len - 1] & 0xC0) == 0x80)
                            s_py_len--;
                        if (s_py_len > 0)
                            s_py_len--;
                        s_py_buf[s_py_len] = '\0';
                        kbd_cand_refresh();
                        kbd_del_update_icon(); /* 組字刪光 → 這顆變「收下」 */
                    }
                    else
                    {
                        control_provider.ble_hid_keyboard_input("\b");
                        remove_from_input_buffer(); // 從顯示中刪除字符
                        if (current_keyboard_mode == KEYBOARD_MODE_LETTERS &&
                            s_en_len > 0)
                        {
                            s_en_word[--s_en_len] = '\0'; /* 單字同步退格 */
                        }
                        kbd_cand_refresh();
                    }
                }
                else if (strcmp(closest_key_text, "Space") == 0)
                {
                    if (mouse_v2t_active)
                    {
                        mouse_v2t_close_and_paste();
                    }
                    else if (current_keyboard_mode == KEYBOARD_MODE_CHINESE &&
                             s_py_len > 0 && s_kbd_cand_count > 0)
                    {
                        /* 組字中才取首選(數字快捷列狀態的空白=一般空白) */
                        kbd_cand_commit(0);
                    }
                    else
                    {
                        LOG_D("Space key pressed");
                        control_provider.ble_hid_keyboard_input(" ");
                        add_to_input_buffer(" "); // 添加空格到顯示
                        s_en_len = 0; /* 單字結束→推薦列回數字列 */
                        s_en_word[0] = '\0';
                        kbd_cand_refresh();
                    }
                }
                else if (current_keyboard_mode == KEYBOARD_MODE_CHINESE &&
                         (strlen(closest_key_text) == 2 ||
                          strlen(closest_key_text) == 3) &&
                         zy_find(closest_key_text) != NULL)
                {
                    /* 注音模式:符號進組字緩衝(不上屏),選字列即時刷新。
                       聲調=2 bytes、其餘=3 bytes。 */
                    size_t zl = strlen(closest_key_text);
                    if (s_py_len <= (int)(sizeof(s_py_buf) - 1 - zl))
                    {
                        memcpy(&s_py_buf[s_py_len], closest_key_text, zl);
                        s_py_len += (int)zl;
                        s_py_buf[s_py_len] = '\0';
                    }
                    kbd_cand_refresh();
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
                        /* PC sim 沒有 BLE HID,這個函式指標是 NULL —— 直接呼叫會當場當機。
                           以前踩不到是因為模擬器進不了鍵盤(唯一入口手寫在 sim 走不到);
                           2026-08-03 入口改成語音站之後第一次打字就炸。真機不受影響。 */
                        if (control_provider.ble_hid_keyboard_input)
                            control_provider.ble_hid_keyboard_input(output_char);
                        add_to_input_buffer(output_char); // 添加到輸入顯示
                        /* 單字緩衝+推薦更新(founder 2026-07-22 英文推薦) */
                        if (s_en_len < (int)sizeof(s_en_word) - 1)
                        {
                            s_en_word[s_en_len++] = (char)(output_char[0] | 32);
                            s_en_word[s_en_len] = '\0';
                        }
                        kbd_cand_refresh();
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

/* ── 中文拼音引擎(founder 2026-07-22):錶上離線,字典=pinyin_dict.c ── */

/* 候選查找:exact 音節優先;**只有無精確命中**(音節未完成,如 ㄋ 單獨=「n」)
   才做 prefix 擴充(founder 2026-07-22:ㄘㄜ 完整音節被 ceng 的「曾」污染)。
   每候選=一個 UTF-8 字。回傳候選數。 */
static int kbd_pinyin_lookup(const char *buf, char out[][8], int max)
{
    int n = 0;
    if (buf[0] == '\0')
        return 0;
    size_t blen = strlen(buf);
    for (int pass = 0; pass < 2 && n < max; pass++)
    {
        if (pass == 1 && n > 0)
            break; /* 有精確命中=音節完整,不做 prefix 擴充 */
        for (int i = 0; i < g_pinyin_dict_count && n < max; i++)
        {
            const pinyin_dict_entry_t *e = &g_pinyin_dict[i];
            bool exact = (strcmp(e->py, buf) == 0);
            bool prefix = (!exact && strncmp(e->py, buf, blen) == 0);
            if ((pass == 0 && !exact) || (pass == 1 && !prefix))
                continue;
            const char *p = e->py_mb;
            while (*p && n < max)
            {
                uint8_t c0 = (uint8_t)*p;
                int cl = (c0 >= 0xF0) ? 4 : (c0 >= 0xE0) ? 3
                         : (c0 >= 0xC0)                  ? 2
                                                         : 1;
                memcpy(out[n], p, (size_t)cl);
                out[n][cl] = '\0';
                n++;
                p += cl;
            }
        }
    }
    return n;
}

/* ── 注音→拼音組字(founder 2026-07-22 改注音;字典仍是拼音表,音節一一對應) ── */
typedef struct zy_map_s
{
    const char *zy; /* 注音符號(UTF-8 3B) */
    const char *py; /* 拼音片段 */
    uint8_t kind;   /* 0=聲母 1=介音 2=韻母 */
} zy_map_t;

static const zy_map_t s_zy_map[] = {
    {"ㄅ", "b", 0},  {"ㄆ", "p", 0},  {"ㄇ", "m", 0},  {"ㄈ", "f", 0},
    {"ㄉ", "d", 0},  {"ㄊ", "t", 0},  {"ㄋ", "n", 0},  {"ㄌ", "l", 0},
    {"ㄍ", "g", 0},  {"ㄎ", "k", 0},  {"ㄏ", "h", 0},  {"ㄐ", "j", 0},
    {"ㄑ", "q", 0},  {"ㄒ", "x", 0},  {"ㄓ", "zh", 0}, {"ㄔ", "ch", 0},
    {"ㄕ", "sh", 0}, {"ㄖ", "r", 0},  {"ㄗ", "z", 0},  {"ㄘ", "c", 0},
    {"ㄙ", "s", 0},  {"ㄧ", "i", 1},  {"一", "i", 1},  /* 一=ㄧ 鍵帽替身
        (U+3127 這套字型是直書形立著,founder 2026-07-22;鍵帽顯示「一」) */
    {"ㄨ", "u", 1},  {"ㄩ", "v", 1},
    {"ㄚ", "a", 2},  {"ㄛ", "o", 2},  {"ㄜ", "e", 2},  {"ㄝ", "e", 2},
    {"ㄞ", "ai", 2}, {"ㄟ", "ei", 2}, {"ㄠ", "ao", 2}, {"ㄡ", "ou", 2},
    {"ㄢ", "an", 2}, {"ㄣ", "en", 2}, {"ㄤ", "ang", 2}, {"ㄥ", "eng", 2},
    {"ㄦ", "er", 2},
    /* 聲調(kind 3):進緩衝顯示、組音忽略(字典不分調;founder 2026-07-22 要標準
       鍵盤位含聲調)。注意:UTF-8 2 bytes(其餘符號 3 bytes)。 */
    {"ˊ", "", 3}, {"ˇ", "", 3}, {"ˋ", "", 3}, {"˙", "", 3},
};

static const zy_map_t *zy_find(const char *sym)
{
    for (unsigned i = 0; i < sizeof(s_zy_map) / sizeof(s_zy_map[0]); i++)
    {
        size_t zl = strlen(s_zy_map[i].zy);
        if (strncmp(s_zy_map[i].zy, sym, zl) == 0)
            return &s_zy_map[i];
    }
    return NULL;
}

/* 韻母(帶介音時)的拼音寫法。key=韻母拼音片段。 */
static const char *zy_rime(const char med, const char *fin, bool zero_init,
                           const char *init)
{
    /* 回傳「介音+韻母」段;zero_init=零聲母(整音節寫法變 y/w 開頭)。 */
    static char rime[8];
    rime[0] = '\0';
    if (med == 'i')
    {
        if (!fin)            strcpy(rime, zero_init ? "yi" : "i");
        else if (!strcmp(fin, "a"))   strcpy(rime, zero_init ? "ya" : "ia");
        else if (!strcmp(fin, "o"))   strcpy(rime, zero_init ? "yo" : "io");
        else if (!strcmp(fin, "e"))   strcpy(rime, zero_init ? "ye" : "ie");
        else if (!strcmp(fin, "ao"))  strcpy(rime, zero_init ? "yao" : "iao");
        else if (!strcmp(fin, "ou"))  strcpy(rime, zero_init ? "you" : "iu");
        else if (!strcmp(fin, "an"))  strcpy(rime, zero_init ? "yan" : "ian");
        else if (!strcmp(fin, "en"))  strcpy(rime, zero_init ? "yin" : "in");
        else if (!strcmp(fin, "ang")) strcpy(rime, zero_init ? "yang" : "iang");
        else if (!strcmp(fin, "eng")) strcpy(rime, zero_init ? "ying" : "ing");
    }
    else if (med == 'u')
    {
        if (!fin)            strcpy(rime, zero_init ? "wu" : "u");
        else if (!strcmp(fin, "a"))   strcpy(rime, zero_init ? "wa" : "ua");
        else if (!strcmp(fin, "o"))   strcpy(rime, zero_init ? "wo" : "uo");
        else if (!strcmp(fin, "ai"))  strcpy(rime, zero_init ? "wai" : "uai");
        else if (!strcmp(fin, "ei"))  strcpy(rime, zero_init ? "wei" : "ui");
        else if (!strcmp(fin, "an"))  strcpy(rime, zero_init ? "wan" : "uan");
        else if (!strcmp(fin, "en"))  strcpy(rime, zero_init ? "wen" : "un");
        else if (!strcmp(fin, "ang")) strcpy(rime, zero_init ? "wang" : "uang");
        else if (!strcmp(fin, "eng")) strcpy(rime, zero_init ? "weng" : "ong");
    }
    else if (med == 'v')
    {
        bool jqx = (init && (init[0] == 'j' || init[0] == 'q' || init[0] == 'x'));
        if (!fin)            strcpy(rime, zero_init ? "yu" : (jqx ? "u" : "v"));
        else if (!strcmp(fin, "e"))   strcpy(rime, zero_init ? "yue" : "ue");
        else if (!strcmp(fin, "an"))  strcpy(rime, zero_init ? "yuan" : "uan");
        else if (!strcmp(fin, "en"))  strcpy(rime, zero_init ? "yun" : "un");
        else if (!strcmp(fin, "eng")) strcpy(rime, zero_init ? "yong" : "iong");
    }
    else if (fin)
    {
        strcpy(rime, fin); /* 無介音:韻母直寫 */
    }
    return rime;
}

/* 注音緩衝→拼音音節(供 kbd_pinyin_lookup;不合法組合→空字串=查無)。 */
static void zhuyin_compose(const char *zy, char *out, size_t outsz)
{
    const char *init = NULL;
    char med = 0;
    const char *fin = NULL;
    out[0] = '\0';
    const char *p = zy;
    while (*p)
    {
        const zy_map_t *m = zy_find(p);
        if (m == NULL)
            return;
        if (m->kind == 3)
        {
            /* 聲調:忽略(字典不分調,只留在緩衝顯示) */
        }
        else if (m->kind == 0 && init == NULL && med == 0 && fin == NULL)
            init = m->py;
        else if (m->kind == 1 && med == 0 && fin == NULL)
            med = m->py[0];
        else if (m->kind == 2 && fin == NULL)
            fin = m->py;
        else
            return; /* 順序不合法 */
        p += strlen(m->zy);
    }
    if (init == NULL && med == 0 && fin == NULL)
        return;
    const char *rime = zy_rime(med, fin, init == NULL, init);
    if (init == NULL)
    {
        /* 零聲母:有介音走 y/w 寫法;純韻母直寫(ㄚ=a、ㄦ=er…) */
        rt_snprintf(out, outsz, "%s", (med != 0) ? rime : (fin ? fin : ""));
    }
    else if (med == 0 && fin == NULL)
    {
        /* 聲母單獨:捲舌/平舌=整音節(zhi/zi…),其他=prefix 查找 */
        if (!strcmp(init, "zh") || !strcmp(init, "ch") || !strcmp(init, "sh") ||
            !strcmp(init, "r") || !strcmp(init, "z") || !strcmp(init, "c") ||
            !strcmp(init, "s"))
            rt_snprintf(out, outsz, "%si", init);
        else
            rt_snprintf(out, outsz, "%s", init);
    }
    else
    {
        rt_snprintf(out, outsz, "%s%s", init, rime);
    }
}

static void kbd_pinyin_clear(void)
{
    s_py_len = 0;
    s_py_buf[0] = '\0';
    s_en_len = 0;
    s_en_word[0] = '\0';
    s_kbd_cand_count = 0;
    s_kbd_row_kind = KBD_ROW_NONE;
    if (s_kbd_cand_row && lv_obj_is_valid(s_kbd_cand_row))
        lv_obj_add_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
}

/* 選字列刷新:中文/英文模式=常駐一條——組字/打單字中顯示候選或英文推薦,
   否則數字快捷列 1~9(founder 2026-07-22);數字模式藏(布局本身有數字)。 */
static void kbd_cand_refresh(void)
{
    if (s_kbd_cand_row == NULL || !lv_obj_is_valid(s_kbd_cand_row))
        return;
    if (current_keyboard_mode == KEYBOARD_MODE_NUMBERS)
    {
        s_kbd_cand_count = 0;
        s_kbd_row_kind = KBD_ROW_NONE;
        lv_obj_add_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    /* 英文模式:打了 >=2 個字母就找推薦;找得到=推薦列,找不到=數字列 */
    if (current_keyboard_mode == KEYBOARD_MODE_LETTERS && s_en_len >= 2)
    {
        int n = 0;
        for (int i = 0; i < g_en_words_count && n < KBD_CAND_MAX; i++)
        {
            if (strncmp(g_en_words[i], s_en_word, (size_t)s_en_len) == 0 &&
                (int)strlen(g_en_words[i]) > s_en_len)
            {
                strncpy(s_kbd_cand_texts[n], g_en_words[i],
                        sizeof(s_kbd_cand_texts[0]) - 1);
                s_kbd_cand_texts[n][sizeof(s_kbd_cand_texts[0]) - 1] = '\0';
                n++;
            }
        }
        if (n > 0)
        {
            s_kbd_row_kind = KBD_ROW_EN_SUGG;
            s_kbd_cand_count = n;
            if (s_kbd_py_lbl)
                lv_obj_add_flag(s_kbd_py_lbl, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_pad_left(s_kbd_cand_row, 36, LV_PART_MAIN);
            lv_obj_set_style_pad_right(s_kbd_cand_row, 36, LV_PART_MAIN);
            lv_obj_set_style_pad_column(s_kbd_cand_row, 10, LV_PART_MAIN);
            lv_obj_set_flex_align(s_kbd_cand_row, LV_FLEX_ALIGN_START,
                                  LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            for (int i = 0; i < KBD_CAND_MAX; i++)
            {
                if (i < n)
                {
                    lv_obj_set_width(s_kbd_cand_btns[i], LV_SIZE_CONTENT);
                    lv_obj_set_style_pad_hor(s_kbd_cand_btns[i], 8,
                                             LV_PART_MAIN);
                    lv_label_set_text(s_kbd_cand_lbls[i],
                                      s_kbd_cand_texts[i]);
                    lv_obj_clear_flag(s_kbd_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
                }
                else
                {
                    lv_obj_add_flag(s_kbd_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
                }
            }
            lv_obj_clear_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
            lv_obj_scroll_to_x(s_kbd_cand_row, 0, LV_ANIM_OFF);
            lv_obj_move_foreground(s_kbd_cand_row);
            return;
        }
        /* 無推薦 → 落到數字列 */
    }
    if (current_keyboard_mode == KEYBOARD_MODE_LETTERS || s_py_len == 0)
    {
        /* 空狀態=數字快捷列(按=直接上屏該數字,走同 commit 路)。
           均分置中(founder 2026-07-22:組字模式的左 pad 36 在這像超大第一格,
           且靠左排把 9 推進圓弧被切)。 */
        if (s_kbd_py_lbl)
            lv_obj_add_flag(s_kbd_py_lbl, LV_OBJ_FLAG_HIDDEN);
        /* 零間隙+鈕寬=線距(450/9=50):數字在「線到線」正中(founder 2026-07-22:
           間隙讓數字看起來靠右)。 */
        lv_obj_set_style_pad_left(s_kbd_cand_row, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_right(s_kbd_cand_row, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_column(s_kbd_cand_row, 0, LV_PART_MAIN);
        lv_obj_set_flex_align(s_kbd_cand_row, LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        s_kbd_row_kind = KBD_ROW_DIGITS;
        s_kbd_cand_count = 9;
        for (int i = 0; i < KBD_CAND_MAX; i++)
        {
            if (i < 9)
            {
                s_kbd_cand_texts[i][0] = (char)('1' + i);
                s_kbd_cand_texts[i][1] = '\0';
                lv_obj_set_width(s_kbd_cand_btns[i], 50);
                lv_obj_set_style_pad_hor(s_kbd_cand_btns[i], 0, LV_PART_MAIN);
                lv_label_set_text(s_kbd_cand_lbls[i], s_kbd_cand_texts[i]);
                lv_obj_clear_flag(s_kbd_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_add_flag(s_kbd_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
        lv_obj_clear_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
        lv_obj_scroll_to_x(s_kbd_cand_row, 0, LV_ANIM_OFF);
        lv_obj_move_foreground(s_kbd_cand_row);
        return;
    }
    if (s_kbd_py_lbl)
        lv_obj_clear_flag(s_kbd_py_lbl, LV_OBJ_FLAG_HIDDEN);
    /* 組字模式:還原起始排列+安全 pad(左 36 避圓弧)+鈕寬/間隙(數字模式改過) */
    lv_obj_set_style_pad_left(s_kbd_cand_row, 36, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_kbd_cand_row, 36, LV_PART_MAIN);
    lv_obj_set_style_pad_column(s_kbd_cand_row, 6, LV_PART_MAIN);
    lv_obj_set_flex_align(s_kbd_cand_row, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_label_set_text(s_kbd_py_lbl, s_py_buf); /* 顯示注音符號串 */
    char py[12];
    zhuyin_compose(s_py_buf, py, sizeof(py));
    s_kbd_cand_count =
        py[0] ? kbd_pinyin_lookup(py, s_kbd_cand_texts, KBD_CAND_MAX) : 0;
    s_kbd_row_kind = KBD_ROW_ZH_CAND;
    for (int i = 0; i < KBD_CAND_MAX; i++)
    {
        if (i < s_kbd_cand_count)
        {
            lv_obj_set_width(s_kbd_cand_btns[i], 42); /* 還原(數字模式撐 50) */
            lv_obj_set_style_pad_hor(s_kbd_cand_btns[i], 0, LV_PART_MAIN);
            lv_label_set_text(s_kbd_cand_lbls[i], s_kbd_cand_texts[i]);
            lv_obj_clear_flag(s_kbd_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_kbd_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    lv_obj_clear_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
    lv_obj_scroll_to_x(s_kbd_cand_row, 0, LV_ANIM_OFF); /* 新一輪候選捲回開頭 */
    lv_obj_move_foreground(s_kbd_cand_row);
}

/* 候選上屏:送遠端(文字注入,V2T 同路)+本地輸入列。英文推薦=補完(只送
   尚未打的字尾,前綴已即時上屏);其餘=送整字/數字。 */
static void kbd_cand_commit(int idx)
{
    if (idx < 0 || idx >= s_kbd_cand_count)
        return;
    const char *send = s_kbd_cand_texts[idx];
    if (s_kbd_row_kind == KBD_ROW_EN_SUGG)
    {
        if ((int)strlen(send) <= s_en_len)
            return;
        send += s_en_len; /* 補完字尾 */
    }
    if (control_provider.ble_hid_keyboard_input)
        control_provider.ble_hid_keyboard_input((char *)send);
    add_to_input_buffer(send);
    kbd_pinyin_clear();
    kbd_cand_refresh(); /* 回到空狀態=數字快捷列 */
}

static void kbd_cand_btn_cb(lv_event_t *e)
{
    kbd_cand_commit((int)(intptr_t)lv_event_get_user_data(e));
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

    // 如果是字母且處於字母/中文模式，根據大小寫狀態調整顯示(中文恆小寫拼音)
    if (current_keyboard_mode != KEYBOARD_MODE_NUMBERS && strlen(text) == 1 &&
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
    /* 內容超出 1px 就會長滾動條+可上下滾(founder 2026-07-22 注音空白下移後
       觸發;此 container 從未清 SCROLLABLE 的舊帳):固定鍵盤,關捲動。 */
    lv_obj_clear_flag(keyboard_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(keyboard_container, LV_SCROLLBAR_MODE_OFF);

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

    if (current_keyboard_mode == KEYBOARD_MODE_CHINESE)
    {
        /* 注音布局=**標準注音鍵盤位**(大千;founder 2026-07-22:直行 ㄅㄆㄇㄈ
           對齊肌肉記憶)。聲調位(ˇˋˊ)留空(字典不分調);ㄦ 原在 '-' 放不下,
           借 ˙ 位(第 7 欄)。row4(mode/space/del) 照舊在 y195。 */
        caps_btn = NULL; /* 本布局無 Caps,清 stale 引用 */
        static const char *zy_rows[4][10] = {
            {"ㄅ", "ㄉ", "ˇ", "ˋ", "ㄓ", "ˊ", "˙", "ㄚ", "ㄞ", "ㄢ"},
            {"ㄆ", "ㄊ", "ㄍ", "ㄐ", "ㄔ", "ㄗ", "一", "ㄛ", "ㄟ", "ㄣ"},
            {"ㄇ", "ㄋ", "ㄎ", "ㄑ", "ㄕ", "ㄘ", "ㄨ", "ㄜ", "ㄠ", "ㄤ"},
            {NULL, "ㄌ", "ㄏ", "ㄒ", "ㄖ", "ㄙ", "ㄩ", "ㄝ", "ㄡ", NULL},
        };
        static const int zy_ys[4] = {0, 48, 96, 144};
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 10; c++)
                if (zy_rows[r][c] != NULL)
                    create_circular_button(keyboard_container, zy_rows[r][c],
                                           c * keyboard_btn_size, zy_ys[r]);
        /* row4 兩角被圓弧切掉(founder 2026-07-22 截圖):ㄈ/ㄥ 連同借位的 ㄦ
           改放功能排(空白鍵下移讓出的位置,mode 與 del 之間)。 */
        create_circular_button(keyboard_container, "ㄈ", 160, 195);
        create_circular_button(keyboard_container, "ㄥ", 205, 195);
        create_circular_button(keyboard_container, "ㄦ", 250, 195);
    }
    else if (current_keyboard_mode != KEYBOARD_MODE_NUMBERS)
    {
        // 第一行 - 字母鍵(英文 QWERTY)
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

    // Space 按鍵(中文模式下移讓位給 ㄈㄥㄦ,founder 2026-07-22)
    space_btn = lv_obj_create(keyboard_container);
    lv_obj_set_size(space_btn, 120, 50);
    lv_obj_set_pos(space_btn, 160,
                   (current_keyboard_mode == KEYBOARD_MODE_CHINESE) ? 235
                                                                    : row4_y);
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
    lv_label_set_text(space_red_dot_x, LV_EXT_STR_GET_BY_KEY(ok, "OK"));
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
    /* 版面重建(切輸入法)後圖要照當下的框內容重挑,否則空框時會退回退格圖。
       擺在 align/opa 之前無妨,src 換掉不影響那些樣式。 */
    lv_obj_align(del_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_img_opa(del_img, LV_OPA_50, LV_PART_MAIN);
    lv_obj_clear_flag(del_img, LV_OBJ_FLAG_CLICKABLE);
    kbd_del_update_icon();

    // close_btn 已移除（鍵盤關閉由 mode 切換取代）


    /* 建立當下就把位移歸零 —— 左右換頁動畫的起點是 translate_x = +466
       (螢幕外)，動畫若沒跑到尾(被 lv_anim_del 中斷、或容器在動畫期間被
       重建)，容器就停在起點。而 find_closest_key() 是「沒有距離上限的最近
       鍵」—— 所有鍵都在螢幕外時，按哪裡都會被判給同一顆，手感就是
       「最下排很難按到/按空白變成切輸入法」(founder 2026-08-07 實測)。
       不要依賴動畫的收尾來歸零；動畫要起點時自己再設。 */
    lv_obj_set_style_translate_x(keyboard_container, 0, 0);
    lv_obj_set_style_translate_y(keyboard_container, 0, 0);

    // 初始時隱藏
    lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
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

    #if USING_TOUCHSCREEN_SCROLLING
    #endif

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

    // 滾動範圍（左右弧形 + 中間態）統一走方向判定：
    //   上下→升級為角度滾動（left_scroll_active）
    //   向右→(back 手勢已停用)退出 pending，走拖曳/滑鼠移動
    //   向左（右弧起手）→ 滑鼠頁滑走回錶盤（L/R swap：home 在右）
    //   向左（其他）→退出 pending，走原本拖曳/滑鼠移動
    left_scroll_active = false;
    center_zone_pending = false;
    back_pending_active = false;
    home_pull_active = false;
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
    if (home_pull_active)
    {
        // finger-follow:拖多遠 = watch face 進來多少,放開才定案(見右弧左拖分支)
        home_pull_prog =
            app_clock_status_bar_pull_home(current_point->x - start_point.x);
        return;
    }
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

    // 滾動區方向判定：上下→升級滾動；向右→mouse back；
    // 向左+右弧→回錶盤；向左其他→走拖曳
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
                /* 向右 → back(上一頁)手勢已停用(2026-07-02 使用者要求:
                   誤觸+攔截游標)。不進 back-hint、不消費手勢 — UI 淡回後
                   fall through 讓底下拖曳邏輯接手送 mouse_move。要復原改回
                   back_pending_active 流程(見 git history)。 */
                animate_scroll_ui_to(false);
                LOG_D("scroll zone -> rightward: back disabled, fall to drag");
            }
            else if (press_in_right_arc_zone)
            {
                animate_scroll_ui_to(false);
                scrolling = true; // 避免放開時誤觸 click
                if (s_hosted_by_pager)
                {
                    // L/R swap：右弧區起手 + 往左拖 → finger-follow 把滑鼠頁往左
                    // 拖出、watch face 跟著手指滑進來(主 tileview 左 tile，home 在
                    // 右邊)。放開才 commit/snap(handle_released_event)。原本這手勢
                    // 開 device-change 選單;裝置切換已由名稱條拖曳/輪播/箭頭取代，
                    // hosted 下選單入口讓位給回家手勢。
                    home_pull_active = true;
                    home_pull_prog = app_clock_status_bar_pull_home(
                        current_point->x - start_point.x);
                    LOG_D("right-arc swipe left -> pull home (finger-follow)");
                }
                else
                {
                    // standalone APP_ID_MOUSE：設備切換已移到頂部設備名兩側箭頭，
                    // 右弧左拖不再開抽屜（消費此手勢、不做事）。
                    LOG_D("right-arc swipe left -> (standalone) switch moved to top arrows");
                }
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
                /* 送游標要同時滿足:①非觸控座標跳變(單軸 delta 超 TOUCH_JUMP_REJECT_PX=觸控
                   面板誤報,丟這幀→消除瞬移) ②飛鼠沒 claim(先到先贏互鎖,飛鼠先到則讓出)。
                   兩種情況 last_point 都照更新——不更新的話 delta 一直對舊點累積,恢復時爆衝。 */
                if (LV_MAX(LV_ABS(delta_x), LV_ABS(delta_y)) <= TOUCH_JUMP_REJECT_PX &&
                    !mouse_air_cursor_owned())
                {
                    control_provider.ble_hid_mouse_move(delta_x * 1.5, delta_y * 1.5);
                }
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

    // 右弧左拖 finger-follow 回錶盤放手：拖過半 → commit 滑回 home(settle 時
    // 拆 device 頁);沒過半 → snap 回裝置頁。teardown 在這裡(放開後)才發生。
    if (home_pull_active)
    {
        home_pull_active = false;
        app_clock_status_bar_pull_home_release(home_pull_prog >= 50);
        animate_scroll_ui_to(false);
        press_in_arc_zone = false;
        LOG_D("home pull released: %s",
              home_pull_prog >= 50 ? "commit -> watch face" : "snap back");
        return;
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
            // 設備頁的 mouse 走 SKAI_LINK 轉送給 APP（back 是 consumer report，
            // 不經 mouse_report_send，所以在呼叫點 route，不能改 GoBack 本體）；
            // 獨立 mouse app 仍走 BLE HID 的 ble_hid_mouse_back。
            if (ble_hid_mouse_app_route())
            {
                commu_send_mouse_back();
            }
            else if (control_provider.ble_hid_mouse_back)
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
/* ─────────────────────────────────────────────────────────────────────────
   主觸控板 長按=拖曳(二模式) / 右緣左拉=手寫(2026-07-20 founder 定案)。
   - 按下=小震;按住不動滿 1s = arm(大震),之後二選一**先到先贏**:
     手指主動滑 → 觸控 1:1 拖曳;手腕/手錶動(gyro) → 體感拖曳(左鍵按住+gyro 驅動
     游標,bloc s_motion_drag_active report)。1s 內手指移動=一般移動(無左鍵)。
     慢跟隨/gyro 區分邏輯沿用圓盤時代(founder 2026-07-17 調校)。只在非弧形/滾動
     區起手才 arm。
   - press 起手在右緣(HW_EDGE_ZONE_W)且向左拉超過 HW_EDGE_PULL_PX(水平主導)
     → 進手寫(hw_view 接管,直接在錶面上寫)。
   - 側立(FACE_SIDE)時按住畫面 → 側立圓盤(bloc pose-dial),放開=執行。
   ───────────────────────────────────────────────────────────────────────── */
#define DIAL_HOLD_TO_START_MS     700 /* 按住不動滿 0.7s 才 arm(founder 2026-07-20:按下小震、
                                         滿門檻大震進長按;1s 太久改 0.7s);門檻內手指移動＝
                                         一般移動滑鼠(無左鍵)、動手錶=gyro 自由移動 */
#define DIAL_HOLD_DRIFT_CANCEL_PX  18 /* arm「之前」：離起點超過就別 arm（判定為滑動）*/
#define DIAL_DRAG_DIST_PX           8 /* arm「之後」：手指離「基準點」累積移動超過＝主動
                                         滑動→進拖曳。手腕在動(mouse_dial_wrist_moving)時基準「慢跟隨」
                                         手指(追一半)、連動/漂移慢會被追上不累積、主動滑快追不上照累積;
                                         手腕靜止時完全不跟隨、手指一滑就累積。故門檻可小、慢速滑也觸發
                                         (founder 2026-07-17 拖曳要滑很遠才判斷→改慢跟隨+降門檻)。 */
extern void mouse_dial_cancel(void);
extern bool mouse_dial_active(void);
extern bool mouse_dial_wrist_moving(void);
extern bool mouse_wrist_accum_triggered(void); /* 體感拖曳進入判定(大震後累積移動量) */
extern void bloc_wrist_accum_reset(void);
/* 側立圓盤(bloc_motion_tracking.c):側立中按住畫面=開、放開=執行(founder 2026-07-20 二改) */
extern bool bloc_dial_pose_touch_ready(void);
extern void mouse_dial_pose_begin(void);
extern void mouse_dial_pose_commit(void);
extern void mouse_dial_pose_cancel(void);
extern void ble_hid_mouse_cancel_touch(void);
extern void ble_hid_mouse_begin_drag(void);
extern void ble_hid_mouse_disable_longpress(void);
extern bool ble_hid_mouse_drag_edge_pan(uint16_t x, uint16_t y);
extern void ble_hid_mouse_drag_edge_pan_stop(void);
/* 右緣拉出跟手三段式(定義在下方手寫段;press 由本觸控機持有並轉發) */
static void hw_pull_begin(lv_coord_t dx0, lv_coord_t finger_x);
static void hw_pull_follow_update(lv_coord_t dx, lv_coord_t finger_x);
static void hw_pull_follow_end(lv_coord_t dx);
static void hw_pull_cancel(void);
static lv_timer_t *s_dial_hold_timer = NULL;
static lv_point_t s_dial_press_start;
static bool s_pose_dial_touch = false; /* 側立圓盤按住中:PRESSED 開 session、RELEASED commit */
static lv_point_t s_pose_start;        /* 側立圓盤 press 起點(判手指滑離→轉觸控板) */
static lv_point_t s_pose_last;         /* 轉觸控板後算游標 delta 的上一幀手指位置 */
static bool s_pose_moved_to_tp = false; /* 側立圓盤已因手指滑退出、轉觸控板(founder 2026-07-30:
                                           側立按住不動=圓盤/手指一滑=觸控板移游標) */
#define POSE_TO_TRACKPAD_PX 12         /* 側立圓盤中手指滑離起點超此(px)=退出圓盤轉觸控板 */
static bool s_hw_hold_armed = false; /* 長按已滿 1s,等「手指滑=觸控拖曳/手腕動=體感拖曳」判定(先到先贏) */
static bool s_motion_drag = false;   /* 體感拖曳中:左鍵按住,游標由 gyro 驅動(bloc report) */
/* 右緣向左拉出=進手寫(founder 2026-07-20 三改) */
#define HW_EDGE_ZONE_W   60 /* 右緣起手區寬度(px) */
#define HW_EDGE_PULL_PX  10 /* 向左拉超過此距離(且水平主導)=view 現形開始跟手 */
static bool s_edge_swipe_armed = false; /* press 起手在右緣,等左拉判定 */
static lv_point_t s_edge_start;
static bool s_hw_pull_active = false; /* 右緣拉出跟手中(press 由本觸控機持有並轉發位移) */
/* 未 arm 階段「手指 vs 手腕」先到先贏的手指基準:手腕在動時追一半(慢跟隨),把手腕轉動
   帶著手指在板上滑的連動位移濾掉——不濾的話 5px 門檻每次都先被連動撞破、觸控板永遠先
   贏,07-20「按住不動、動手錶=游標動」就實質失效(沿用 07-17 圓盤的同一套區分器)。 */
static lv_point_t s_free_ref;
/* 手指「前哨」軟門檻(founder 2026-07-24「有點太容易變飛鼠」):比 5px 正式 claim 更敏感,
   手指一開始滑就先擋飛鼠,不用等累積到 claim。用手指意圖(有沒有在滑)仲裁,取代之前靠
   gyro 門檻猜意圖的拉扯——那個高了體感有停頓、低了滑板易變飛鼠,二選一。 */
#define FINGER_ACTIVE_PX 3
/* 按下寬限期(ms):PRESSED 後這段時間先給觸控板優先、不送體感飛鼠游標。按下瞬間手指還沒
   表態,手腕的自然晃動不該馬上被當體感送游標→游標順移到新位置後才被手指接手(founder
   2026-07-24)。過了寬限期手指還沒滑,才認定體感意圖放行。比 arm(700ms)短,在其之前。 */
#define PRESS_FREE_GRACE_MS 200
static bool s_dial_drag = false;    /* 拖曳物件中：1:1 相對移動 + 左鍵按著 */
static lv_point_t s_dial_drag_last; /* 拖曳中上一幀手指位置，算 1:1 相對移動量 */

static void dial_hold_cancel(void)
{
    if (s_dial_hold_timer)
    {
        lv_timer_del(s_dial_hold_timer);
        s_dial_hold_timer = NULL;
    }
}

static void dial_hold_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_dial_hold_timer = NULL; /* repeat_count=1 跑完自刪，只清指標 */
    LOG_I("[drag] trackpad hold %dms -> armed", DIAL_HOLD_TO_START_MS);
    /* 清掉觸控板 tap/long-press 狀態機：armed 期間手指按著不動，不能讓原本
       long-press 誤觸發左鍵（進拖曳時才主動按左鍵）。 */
    ble_hid_mouse_cancel_touch();
    motor_pattern_tap(); /* arm 進長按:小震(founder 2026-07-24 由大震改小震) */
    /* arm 瞬間凍結游標+清甩動記錄(founder:選字後體感拖曳變重新框選——arm 前的
       自由移動/甩動前奏把游標帶離選取區,左鍵才按下去=從新位置開始框選。凍結後
       左鍵會按在使用者瞄準的位置,arm 後的新甩動才開始拖)。 */
    bloc_press_free_move_set(false);
    bloc_wrist_accum_reset();
    s_hw_hold_armed = true;
}

/* PRESSED：只在中央觸控板（非弧形/滾動起手）武裝 hold 計時器。press_in_arc_zone
   由 handle_pressed_event 先算好，故本函式必須排在它之後呼叫。 */
static void dial_hold_on_pressed(lv_indev_t *indev)
{
    dial_hold_cancel();
    s_hw_hold_armed = false; /* 新一次按下:清上次殘留(正常已在 RELEASED/PRESS_LOST 清過) */
    /* 先到先贏的手指基準:每次 press 都要重設,含下面 early return 的弧形區起手——不重設
       就會拿上一次 press 的殘留座標來比,一進 PRESSING 距離就爆表、誤判手指贏鎖住飛鼠。 */
    if (indev)
        lv_indev_get_point(indev, &s_free_ref);
    if (press_in_arc_zone)
        return; /* 弧形滾動起手不 arm */
    /* 碰畫面不再震動(founder 2026-07-24 拿掉按下回饋)。按下寬限期:PRESSED 先壓著不啟用
       體感自由移動(見 PRESS_FREE_GRACE_MS)。過了寬限期手指還沒滑,才在 PRESSING 放行——
       消除按下初期手腕晃動誤觸的游標順移。 */
    bloc_press_free_move_set(false);
    bloc_wrist_accum_reset(); /* 從按下起算:門檻內手錶動夠多=取消長按(對稱手指的 drift-cancel) */
    if (indev)
        lv_indev_get_point(indev, &s_dial_press_start);
    /* 禁用原本的 500ms long-press：dial 用自己的 500ms timer 接管「長按」語意，
       兩個 500ms timer 並存會 race 誤按左鍵。只停 timer、不動 touch state，tap→click 不受影響。 */
    ble_hid_mouse_disable_longpress();
    s_dial_hold_timer = lv_timer_create(dial_hold_timer_cb,
                                        DIAL_HOLD_TO_START_MS, NULL);
    lv_timer_set_repeat_count(s_dial_hold_timer, 1);
}

/* 進入 1:1 拖曳物件：清掉觸控板 tap/long-press 狀態機、按下左鍵，之後手指移多少游標移
   多少。比原本 edge_pan（邊緣平移，手指在中心區域游標不動）直覺——中心也能拖，修
   founder 2026-07-16 回報的「按下去後一開始的拖動電腦端沒反應」。 */
static void dial_drag_begin(const lv_point_t *now)
{
    ble_hid_mouse_cancel_touch();
    bloc_press_free_move_set(false); /* 觸控拖曳=手指驅動,關掉 gyro 自由移動 */
    if (control_provider.ble_hid_mouse_left_press)
        control_provider.ble_hid_mouse_left_press();
    s_dial_drag = true;
    s_dial_drag_last = *now;
    motor_pattern_tap();
}

/* 進入體感拖曳(founder 2026-07-20:長按滿1s後「移動整個手錶」=左鍵按住+gyro 驅動
   游標)。與觸控 1:1 拖曳互斥、先到先贏。 */
static void motion_drag_begin(void)
{
    ble_hid_mouse_cancel_touch();
    bloc_press_free_move_set(false); /* 換成拖曳通道(左鍵按住) */
    if (control_provider.ble_hid_mouse_left_press)
        control_provider.ble_hid_mouse_left_press();
    s_motion_drag = true;
    bloc_motion_drag_set(true); /* bloc 開始把 gyro delta 打進游標(同飛鼠 report) */
    motor_pattern_tap();
}

/* 收體感拖曳:停 gyro 驅動+放左鍵。冪等。 */
static void motion_drag_end(void)
{
    if (!s_motion_drag)
        return;
    s_motion_drag = false;
    bloc_motion_drag_set(false);
    if (control_provider.ble_hid_mouse_left_release)
        control_provider.ble_hid_mouse_left_release();
}

/* 清 手寫 arm / 側立圓盤 / 拖曳 / edge-pan / long-press timer 的殘留狀態機。離開或暫停
   滑鼠 app 時呼叫——否則狀態帶到下次進 app 會卡(s_dial_drag 殘留→PRESSING 一直走拖曳、
   s_dial_active 殘留→air_mouse 圓盤分支永遠 return、lv_timer 殘留→UAF)。冪等，
   可安全重複呼叫(founder 2026-07-17 卡住根因)。 */
static void dial_drag_state_reset(void)
{
    dial_hold_cancel();
    s_hw_hold_armed = false;
    s_pose_dial_touch = false;
    s_pose_moved_to_tp = false;
    s_edge_swipe_armed = false;
    motion_drag_end(); /* 體感拖曳殘留(冪等) */
    bloc_press_free_move_set(false);
    bloc_cursor_owner_reset(); /* 離開/暫停 app:互鎖殘留會跟著帶到下次進 app */
    if (mouse_dial_active())
        mouse_dial_cancel();              /* 清側立圓盤(含 pose 旗標)+桌面 hide 圓盤 */
    if (s_dial_drag)
    {
        s_dial_drag = false;
        ble_hid_mouse_drag_edge_pan_stop();
        if (control_provider.ble_hid_mouse_left_release)
            control_provider.ble_hid_mouse_left_release();
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
        /* 側立中按下畫面=開側立圓盤(founder 2026-07-20 二改:按住=桌面圓盤現形、
           手腕比方向、放開=執行)。跳過一般 press 機制——不送 BLE HID touch、
           不 arm 手寫、不進 click 判定,這個 press 從頭到尾屬於圓盤。 */
        if (bloc_dial_pose_touch_ready())
        {
            s_pose_dial_touch = true;
            s_pose_moved_to_tp = false;
            if (indev)
            {
                lv_indev_get_point(indev, &s_pose_start);
                s_pose_last = s_pose_start;
            }
            mouse_dial_pose_begin();
            break;
        }
        handle_pressed_event(indev);
        dial_hold_on_pressed(indev); /* 主觸控板長按→拖曳 arm：武裝 hold 計時器 */
        /* 右緣左拉進手寫已退役(founder 2026-07-22:改頂部 logo tap 開,
           見 status_bar_area_up_cb);edge armed 恆 false。 */
        s_edge_swipe_armed = false;
        break;
    }
    case LV_EVENT_PRESSING:
    {
        if (s_pose_dial_touch)
        {
            /* 側立按住不動=圓盤(手腕比方向,bloc 積分);手指滑離起點超門檻=退出圓盤、轉觸控板
               移游標(founder 2026-07-30)。 */
            lv_point_t pp;
            lv_indev_get_point(indev, &pp);
            if (!s_pose_moved_to_tp &&
                LV_ABS(pp.x - s_pose_start.x) + LV_ABS(pp.y - s_pose_start.y) >
                    POSE_TO_TRACKPAD_PX)
            {
                s_pose_moved_to_tp = true;
                mouse_dial_pose_cancel(); /* 桌面收圓盤,不 commit */
                s_pose_last = pp;         /* 從當下起算游標 delta,免第一幀爆衝 */
            }
            if (s_pose_moved_to_tp)
            {
                /* 觸控板:手指相對移動送游標(同觸控板 1.5x)。飛鼠 owned 時讓出(互鎖)。 */
                int dx = pp.x - s_pose_last.x;
                int dy = pp.y - s_pose_last.y;
                if ((dx || dy) && !mouse_air_cursor_owned() &&
                    control_provider.ble_hid_mouse_move)
                    control_provider.ble_hid_mouse_move((int8_t)(dx * 1.5),
                                                        (int8_t)(dy * 1.5));
                s_pose_last = pp;
            }
            break;
        }

        lv_point_t now_point;
        lv_indev_get_point(indev, &now_point);

        /* (0a) 右緣拉出跟手中:本狀態機持有 press,轉發累積位移給手寫 view(參考
           app_clock_status_bar 的 follow_update;view 不可點,press 不會被搶走)。 */
        if (s_hw_pull_active)
        {
            hw_pull_follow_update(now_point.x - s_edge_start.x, now_point.x);
            break;
        }

        /* (0) 右緣向左拉出進手寫已退役(founder 2026-07-22:頂部 logo tap 開)。 */

        /* (1) 已在拖曳物件：手指移多少游標移多少（1:1 相對移動，左鍵按著）。 */
        if (s_dial_drag)
        {
            /* 手指到觸控板邊緣→edge_pan 持續往該方向平移(手指沒空間了還想繼續移,founder
               2026-07-17);中心 band→1:1 相對移動。互斥:在邊緣就不送 1:1、讓 pan timer 接手。 */
            bool at_edge = ble_hid_mouse_drag_edge_pan(now_point.x, now_point.y);
            if (!at_edge)
            {
                int mdx = now_point.x - s_dial_drag_last.x;
                int mdy = now_point.y - s_dial_drag_last.y;
                /* 觸控座標跳變濾波(同觸控板):拖曳也用觸控座標,單軸爆步=誤報,丟這幀不送。
                   基準 s_dial_drag_last 照更新(下方 if 外)。 */
                if (LV_MAX(LV_ABS(mdx), LV_ABS(mdy)) <= TOUCH_JUMP_REJECT_PX)
                {
                    if (mdx > 127) mdx = 127; else if (mdx < -127) mdx = -127;
                    if (mdy > 127) mdy = 127; else if (mdy < -127) mdy = -127;
                    if ((mdx || mdy) && control_provider.ble_hid_mouse_move)
                        control_provider.ble_hid_mouse_move((int8_t)mdx, (int8_t)mdy);
                }
            }
            s_dial_drag_last = now_point;
            break;
        }

        lv_coord_t dx = now_point.x - s_dial_press_start.x;
        lv_coord_t dy = now_point.y - s_dial_press_start.y;

        /* (2) 長按已 arm:二選一先到先贏(founder 2026-07-20)——手指主動滑=觸控 1:1 拖曳;
           手腕/手錶在動(gyro 累積過門檻)=體感拖曳。 */
        if (s_hw_hold_armed)
        {
            /* 判定源用「原始累積位移」fdist(相對 arm 起點)。手指拖的反作用力讓手腕微動(每幀
               gyro≥3),舊版的慢跟隨基準會去追手指、把主動位移吃掉,同時 wrist_accum 被同一副
               作用灌到門檻,雙雙把手指拖誤導成體感拖(founder 2026-07-24「長按手指拖變飛鼠、
               不能手指滑動」)。原始位移不受這污染。 */
            int fdist = LV_ABS(dx) + LV_ABS(dy);
            /* 手指在滑就刷前哨→凍結 bloc 的 wrist_accum,免得手指副作用 gyro 觸發體感拖。 */
            if (fdist > FINGER_ACTIVE_PX)
                bloc_touch_finger_active();
            if (fdist > DIAL_DRAG_DIST_PX)
            {
                s_hw_hold_armed = false;
                dial_drag_begin(&now_point);
                break;
            }
            if (mouse_wrist_accum_triggered())
            {
                /* 手指幾乎沒動、純手錶大動才到這(手指在滑時 wrist_accum 被凍結)=真體感拖。 */
                s_hw_hold_armed = false;
                motion_drag_begin();
            }
            break; /* 未判定：按住等待，不控游標 */
        }

        /* (2b) 體感拖曳中:游標由 gyro(bloc report),手指不做任何游標事。 */
        if (s_motion_drag)
            break;

        /* (3) 還沒 arm，手指移動 = 一般移動滑鼠（原本 handle_pressing_event 的
           scrolling→ble_hid_mouse_move 1.5x 移游標），並取消 arm 計時（手指移動＝不是長按）。
           按住不動滿 500ms 才 arm；500ms 後手指移動改走 (2) 的拖曳。手指沒動時一樣走
           handle_pressing_event（tap 判斷）。 */
        /* (3a) 觸控板 vs 飛鼠先到先贏(founder 2026-07-24:兩條都送 ble_hid_mouse_move,
           會同時推游標)。手指基準慢跟隨扣掉手腕連動後還滑得動＝手指先到,claim 走觸控板
           並鎖住飛鼠;飛鼠側由 bloc 在 gyro 累積過解鎖門檻時 claim。兩邊 claim 前都先問
           對方贏了沒=不會雙鎖。贏家鎖到放開手指為止(RELEASED/PRESS_LOST 才 reset)。 */
        if (!mouse_air_cursor_owned())
        {
            if (mouse_dial_wrist_moving())
            {
                s_free_ref.x += (now_point.x - s_free_ref.x) / 2;
                s_free_ref.y += (now_point.y - s_free_ref.y) / 2;
            }
            lv_coord_t fd = LV_ABS(now_point.x - s_free_ref.x) +
                            LV_ABS(now_point.y - s_free_ref.y);
            /* 前哨:手指一滑(fd 過軟門檻)就擋飛鼠——滑觸控板時手指一動就壓過飛鼠,不被手腕
               微動搶走游標。體感情境手指按住不動、慢跟隨把 fd 收斂到 0,不觸發,飛鼠照樣即時。
               改純前哨動態(不再 claim owner 永久鎖):手指停 250ms 讓飛鼠接手、再滑切回觸控板,
               同一次按著能來回切;兩者同幀都動時前哨天生讓觸控板贏(founder 2026-07-30)。 */
            if (fd > FINGER_ACTIVE_PX)
                bloc_touch_finger_active();
        }

        /* 過了按下寬限期、手指還按著沒 arm=可能是體感意圖,放行自由移動(PRESSED 壓著沒啟用)。
           寬限期內手腕自然晃動不送游標→消除按下初期順移;手指若已在滑,前哨(touch_finger_recent)
           照擋飛鼠,放行無妨。 */
        if (lv_tick_elaps(press_time) > PRESS_FREE_GRACE_MS)
            bloc_press_free_move_set(true);

        if (s_dial_hold_timer &&
            (LV_ABS(dx) + LV_ABS(dy) > DIAL_HOLD_DRIFT_CANCEL_PX ||
             mouse_wrist_accum_triggered()))
            dial_hold_cancel(); /* 門檻內手指滑「或」手錶動=一般移動,取消長按
                                   (founder:按下直接動手錶,後面不能又按左鍵) */
        handle_pressing_event(indev, &now_point);
        break;
    }

    case LV_EVENT_RELEASED:
        s_edge_swipe_armed = false;
        bloc_press_free_move_set(false); /* 放開=停按住期間的 gyro 自由移動 */
        bloc_cursor_owner_reset(); /* 放開=互鎖歸零。不清的話 s_touch_cursor_owned 殘留會把
                                      handfree 飛鼠一路鎖到下次按下 */
        if (s_hw_pull_active)
        {
            /* 拉出放開:commit 判定交給 follow_end(距離/快甩),跳過 click 判定 */
            lv_point_t rp;
            lv_indev_get_point(indev, &rp);
            hw_pull_follow_end(rp.x - s_edge_start.x);
            user_touching = false;
            break;
        }
        if (s_pose_dial_touch)
        {
            s_pose_dial_touch = false;
            if (s_pose_moved_to_tp)
            {
                /* 已轉觸控板:放開=無事(已移游標,圓盤早收、不 commit、不誤 click)。 */
                s_pose_moved_to_tp = false;
            }
            else
            {
                /* 手指沒滑=圓盤:以當下方向 commit(桌面執行);跳過 handle_released 免誤 click */
                mouse_dial_pose_commit();
            }
            user_touching = false;
            break;
        }
        dial_hold_cancel();
        if (s_dial_drag)
        {
            /* 拖曳物件放開：停邊緣平移 + 放開左鍵。 */
            s_dial_drag = false;
            ble_hid_mouse_drag_edge_pan_stop();
            if (control_provider.ble_hid_mouse_left_release)
                control_provider.ble_hid_mouse_left_release();
            user_touching = false;
            break;
        }
        if (s_motion_drag)
        {
            /* 體感拖曳放開:放左鍵、停 gyro 驅動。 */
            motion_drag_end();
            user_touching = false;
            break;
        }
        if (s_hw_hold_armed)
        {
            /* 長按沒拖就放開：視為無事。清遠端觸控態（arm 前已 Touch_Press），
               跳過 handle_released_event 以免誤觸 left_click。 */
            lv_point_t rp;
            lv_indev_get_point(indev, &rp);
            s_hw_hold_armed = false;
            BLE_HID_Mouse_Touch_Release((uint16_t)rp.x, (uint16_t)rp.y);
            user_touching = false;
            break;
        }
        handle_released_event(indev);
        break;

    case LV_EVENT_PRESS_LOST:
        /* press 被搶（上層 UI 亮出 / 手指滑出）→ 沒有 RELEASED，必須在這裡收尾，否則
           s_hw_hold_armed / s_dial_drag 殘留會讓下次 PRESSING 走錯分支（founder 2026-07-16
           同型教訓）。 */
        if (s_hw_pull_active)
            hw_pull_cancel(); /* press 被搶=縮回不開 */
        if (s_pose_dial_touch)
        {
            s_pose_dial_touch = false;
            s_pose_moved_to_tp = false;
            mouse_dial_pose_cancel(); /* press 被搶=不 commit,桌面收圓盤 */
        }
        motion_drag_end(); /* 體感拖曳被搶=放左鍵停驅動(冪等) */
        bloc_press_free_move_set(false);
        bloc_cursor_owner_reset(); /* press 被搶=互鎖歸零(同 RELEASED) */
        dial_hold_cancel();
        s_hw_hold_armed = false;
        s_edge_swipe_armed = false;
        if (s_dial_drag)
        {
            s_dial_drag = false;
            ble_hid_mouse_drag_edge_pan_stop();
            if (control_provider.ble_hid_mouse_left_release)
                control_provider.ble_hid_mouse_left_release();
        }
        user_touching = false;
        break;

    case LV_EVENT_CLICKED:
        /* Tap-outside-to-cancel for the AI input box opened via instruction_list_
           bar_tap_device's remote-focus flow: that flow parks p_instruction_list_bg
           off-screen with translate_x, which in this LVGL v8 build moves the
           object's real hit-test rect (lv_obj_refr_pos folds translate into x/y,
           it is NOT a draw-only transform) — so a tap on the visible screen never
           reaches list_window_scroll_event_cb and falls through to touch_bg (this
           object) instead. Only closes when the box is actually open
           (get_is_open_instruction_list_ai); the NORMAL browse-list flow keeps its
           list on-screen, so its taps are already caught by the top-layer search
           before ever reaching touch_bg — this branch is a no-op for that case. */
        {
            extern bool get_is_open_instruction_list_ai(void);
            if (get_is_open_instruction_list_ai())
            {
                extern void instruction_list_cancel_ai_widget(void);
                instruction_list_cancel_ai_widget();
            }
        }
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
    /* 語音站另有一圈沿大框擴散的藍色脈衝(那裡的邊框閃爍提示已被關掉,見
       kbd_bar_set_voice_box)。不在語音站時是 no-op。 */
    voice_glow_set_active(active);
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
    /* 語音站:把插入點左右兩半拍下來,這一段講的字就插在中間(見 mouse_apply_v2t_input)。
       有框選時先把選取那段刪掉 —— 「選起來再講」= 取代那一段,與一般編輯器一致。 */
    if (s_voice_box_on)
    {
        if (voice_sel_active())
            voice_delete_selection();
        int cut = voice_caret_bytes();
        int keep_pre = cut < (int)sizeof(s_voice_ins_prefix) - 1
                           ? cut : (int)sizeof(s_voice_ins_prefix) - 1;
        memcpy(s_voice_ins_prefix, input_buffer, keep_pre);
        s_voice_ins_prefix[keep_pre] = '\0';
        rt_strncpy(s_voice_ins_suffix, input_buffer + cut,
                   sizeof(s_voice_ins_suffix) - 1);
        s_voice_ins_suffix[sizeof(s_voice_ins_suffix) - 1] = '\0';
        s_voice_ins_armed = true;
    }
#ifdef BSP_USING_PC_SIMULATOR
    /* PC sim 沒有音訊 HAL(speech/skai 不編進來),真的去開語音辨識會當場當機。只更新視覺
       狀態就好 —— 錄音本來就只能在真機驗,但擋住這裡之後,語音站的**其他**互動(框選、
       游標、刪除)在模擬器就測得動了。 */
    LOG_I("[voice] PC sim: v2t open suppressed (no audio HAL)");
    mouse_v2t_set_active(true);
    return;
#else
    pending_v2t_intent = intent;
    is_long_press_triggered = true;
#endif
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MOUSE_OPEN_V2T;
    lvgl_send_msg(msg);
}

// 預設：純語音輸入
static void mouse_v2t_open(void)
{
    mouse_v2t_open_with_intent(V2T_INTENT_MIC_INPUTE);
}

// 由「錶面立起正對臉」姿態偵測 (bloc_motion_tracking, motion thread) 跨層觸發：
// 發 LVGL msg 轉到 LVGL thread 的 open_skaibar_from_pose，複製「點底部 bar」帶出
// 單設備 skaibar 列表 (instruction_list_bar_tap_device)，非開 v2t 錄音。
// thread-safe：只發 msg，實際開 skaibar 在 LVGL thread。
void hid_mouse_trigger_skaibar_from_pose(void)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MOUSE_OPEN_SKAIBAR;
    lvgl_send_msg(msg);
}

// (2026-07-16 的 hid_mouse_trigger_close_lift_mic_from_pose 已於 2026-07-31 移除：
//  founder「放下不要直接退出」—— 手腕放下不再收掉立起輸入面板，所以整條 motion-thread→
//  LVGL msg 的關閉轉發沒有觸發源了。面板現在只由使用者的明確動作結束：icon_send / logo /
//  再點一次底部 bar。)

/* ═══════════════════════════════════════════════════════════════════════════
   手寫模式 (handwriting) —— 2026-07-20 founder 三改定案:
   進入=「畫面右緣向左拉出」:view 跟著手指從右滑進來(觸控機 (0) 分支→hw_pull_begin,
   之後 hw_view cb 跟手),放開拉超過 1/3 → snap 展開並 commit(送 0x1b start),
   不足 → snap 縮回(不開)。體感照 app_clock_status_bar 通知列表的拉出(founder 指定)。
   輸入=**直接在錶面上寫**(hw_view 觸控 cb 把指尖座標餵 bloc_handwrite_feed_point、
   按=下筆/放=提筆+本地 lv_line 軌跡即畫,畫布=錶面解析度,0x1b 串流到手機 ML Kit
   辨識+桌面軌跡);結束=提筆後 idle(HW_IDLE_END_MS)沒再下筆,桌面自動送出辨識文字。
   全螢幕 CLICKABLE overlay 天然攔掉主觸控板的拖曳/edge_pan/一般游標;
   gyro→游標 report 由 air_mouse_process 的手寫分支 early-return 擋。
   view 建一次、show/hide 復用(同 lift-mic pattern)。
   ═══════════════════════════════════════════════════════════════════════════ */
/* 畫布=錶面解析度(2026-07-20 三改:直接在錶面上寫,觸控座標=筆跡座標;0x1b start
   帶 LV_HOR_RES×LV_VER_RES,手機 ML Kit WritingArea/桌面書寫區都吃 start 的 w/h 自動適配)。 */

extern lv_obj_t *hid_mouse_ui_host(void);
extern void motor_pattern_unlocked(void);
extern bool is_at_mouse_mode(void);
extern bool app_control_get_mouse_mode(void);

static lv_obj_t *s_hw_view = NULL;
static lv_obj_t *s_hw_backdrop = NULL; /* 進場黑底:獨立於 view,原地漸黑(founder
                                          2026-07-22:元件滑入,觸碰板原地變黑) */
static lv_obj_t *s_hw_btn_exit = NULL;  /* 頂部:退出(取消不送出,founder 2026-07-20) */
static lv_obj_t *s_hw_btn_clear = NULL; /* 底部情境鍵——板上有字=清空(文字)/沒字=刪除(圖) */
static lv_obj_t *s_hw_btn_clear_lbl = NULL; /* 清空狀態的文字 */
static lv_obj_t *s_hw_btn_clear_img = NULL; /* 刪除狀態的圖=鍵盤頁同款 backspace_icon
                                               (founder 2026-07-22:別用預設藍鈕) */
static lv_obj_t *s_hw_btn_mode = NULL; /* 左下(=鍵盤 mode_btn 同位):erth 切輸入法圖,
                                          點=收手寫開鍵盤(英文→數字→手寫循環) */
static lv_obj_t *s_hw_btn_enter = NULL; /* 退出鈕下方同欄:輸入(送出)——候選清單空才顯示;
                                           有候選時該欄顯示候選(founder 2026-07-22:底部
                                           不再有下個字/輸入,定稿一律按候選字) */

/* 候選字列(founder 2026-07-20 晚):退出鈕下方一欄,手機每次辨識下發 top-5(0x1c)。
   按候選=用該字定稿+換下個字(送 "n"+i);輸入鈕只在候選清空時顯示=先定稿才能送出。 */
#define HW_CAND_MAX 5
static lv_obj_t *s_hw_cand_row = NULL;
static lv_obj_t *s_hw_cand_btns[HW_CAND_MAX];
static lv_obj_t *s_hw_cand_lbls[HW_CAND_MAX];
static int s_hw_cand_count = 0;
static volatile bool s_hw_view_active = false; /* motion thread 讀(姿勢保險收斂) */
static lv_coord_t s_hw_pull_vx = 0;     /* 最近一 tick 的水平步進(放開 fling 判定,參考 vx<-6) */
static lv_coord_t s_hw_pull_prev_x = 0;

/* 本地軌跡(2026-07-20 三改加回:手指直寫要看得到筆跡;觸控座標=螢幕座標零映射)。
   池=每「字」的筆畫(按「下個字」清板重來):20 筆 × 80 點,複雜中文字(15-20+ 筆)
   不再吃掉前面的筆畫(founder:「筆畫會不見」=舊 8 筆池互吃)。80 點@觸控率足夠
   單筆 1-2 秒;靜態 RAM 20×80×4B=6.4KB。 */
#define HW_TRACE_STROKES 20
#define HW_TRACE_PTS     80
static lv_obj_t *s_hw_lines[HW_TRACE_STROKES];
static lv_point_t s_hw_line_pts[HW_TRACE_STROKES][HW_TRACE_PTS];
static uint16_t s_hw_line_n[HW_TRACE_STROKES];
static int8_t s_hw_stroke_idx = -1;
static bool s_hw_local_pen = false;

static void hw_ink_append(const lv_point_t *p);
static void hw_ink_clear(void);
static void hw_cand_clear_local(void);
static void hw_pull_snap(bool open);

/* 黑底透明度=滑入進度(view x=466→opa 0、x=0→opa 255)。所有動 view x 的路徑
   (snap/exit anim/左拉跟手)都要跟著呼。 */
static void hw_backdrop_sync(lv_coord_t view_x)
{
    if (s_hw_backdrop == NULL)
        return;
    lv_coord_t x = view_x;
    if (x < 0)
        x = 0;
    if (x > LV_HOR_RES)
        x = LV_HOR_RES;
    lv_obj_set_style_bg_opa(s_hw_backdrop,
                            (lv_opa_t)(255 - (255 * x) / LV_HOR_RES), 0);
}

static void hw_backdrop_hide(void)
{
    if (s_hw_backdrop)
    {
        lv_obj_add_flag(s_hw_backdrop, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_opa(s_hw_backdrop, LV_OPA_TRANSP, 0);
    }
}
static void hw_open_commit(void);
static void close_handwrite_from_pose(void); /* 定義在本段尾:輸入鈕=送出 */
static void hw_cancel_session(void);         /* 定義在本段尾:退出鈕/離開 app=取消 */


static void hw_view_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    lv_point_t p = {0, 0};
    if (indev)
        lv_indev_get_point(indev, &p);

    /* 直接在錶面上寫(founder 2026-07-20 三改):觸控座標=筆跡,同步畫本地軌跡。
       PRESSED+PRESSING 都算下筆:LVGL 無 PRESS_LOCK、press 每 tick 重 hit-test。
       (拉出跟手階段 view 不可點,事件到不了這裡——commit 後才開 CLICKABLE。) */
    if (code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING)
    {
        bloc_handwrite_feed_point((int)p.x, (int)p.y);
        bloc_handwrite_set_pen(true);
        hw_ink_append(&p);
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        bloc_handwrite_set_pen(false);
        /* 單點筆畫(點/頓筆):lv_line 一個點畫不出東西——補一個 1px 偏移點,
           圓頭線帽渲染成一顆點(founder 2026-07-22:「只用點的畫面上什麼都
           沒有」;辨識端吃座標不受影響)。 */
        if (s_hw_local_pen && s_hw_stroke_idx >= 0 &&
            s_hw_line_n[s_hw_stroke_idx] == 1)
        {
            int idx = s_hw_stroke_idx;
            s_hw_line_pts[idx][1].x = (lv_coord_t)(s_hw_line_pts[idx][0].x + 1);
            s_hw_line_pts[idx][1].y = s_hw_line_pts[idx][0].y;
            s_hw_line_n[idx] = 2;
            if (s_hw_lines[idx])
                lv_line_set_points(s_hw_lines[idx], s_hw_line_pts[idx], 2);
        }
        s_hw_local_pen = false; /* 下次按下=新一筆 */
    }
}

/* 按鈕(founder 2026-07-20:不要自動輸入/2026-07-22:定稿一律按候選字):頂部退出=
   取消(不送出);候選欄=按候選定稿+換下字,清單空時同欄顯示輸入=結束送出。
   按鈕不 bubble,不會誤餵筆跡。 */
/* 按候選字=用該候選定稿+換下個字("n"+pick;founder 2026-07-20 晚)。 */
static void hw_cand_btn_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= s_hw_cand_count)
        return;
    bloc_handwrite_next_pick(idx); /* 送 "n"+i:手機用 candidates[i] 定稿 */
    hw_ink_clear();                /* 清板+清候選,輸入鈕現形 */
}

/* 候選列清空(本地立即;手機事後也會下發空清單,冪等)+輸入鈕現形。 */
static void hw_cand_clear_local(void)
{
    s_hw_cand_count = 0;
    if (s_hw_cand_row)
        lv_obj_add_flag(s_hw_cand_row, LV_OBJ_FLAG_HIDDEN);
    if (s_hw_btn_enter)
        lv_obj_clear_flag(s_hw_btn_enter, LV_OBJ_FLAG_HIDDEN);
}

/* GUI thread(communicate 0x1c→ui_handler):候選清單更新。count=0=清空。
   view 沒開(遲到 frame)直接忽略。 */
void mouse_handwrite_candidates(const char *const *texts, int count)
{
    if (!s_hw_view_active || s_hw_cand_row == NULL)
        return;
    if (count > HW_CAND_MAX)
        count = HW_CAND_MAX;
    if (count <= 0)
    {
        hw_cand_clear_local();
        return;
    }
    s_hw_cand_count = count;
    for (int i = 0; i < HW_CAND_MAX; i++)
    {
        if (i < count)
        {
            lv_label_set_text(s_hw_cand_lbls[i], texts[i]);
            lv_obj_clear_flag(s_hw_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_hw_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    lv_obj_clear_flag(s_hw_cand_row, LV_OBJ_FLAG_HIDDEN);
    if (s_hw_btn_enter)
        lv_obj_add_flag(s_hw_btn_enter, LV_OBJ_FLAG_HIDDEN); /* 有候選=先定稿,藏輸入 */
}

/* 情境鍵內容:板上有字=「清空」文字、沒字=鍵盤頁的 backspace 圖(founder 2026-07-20
   合併/2026-07-22 刪除改圖)。 */
static void hw_btn_clear_refresh_label(void)
{
    if (s_hw_btn_clear_lbl == NULL || s_hw_btn_clear_img == NULL)
        return;
    if (s_hw_stroke_idx < 0)
    {
        lv_obj_add_flag(s_hw_btn_clear_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_hw_btn_clear_img, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_label_set_text(s_hw_btn_clear_lbl,
                          LV_EXT_STR_GET_BY_KEY(handwrite_clear, "Clear"));
        lv_obj_clear_flag(s_hw_btn_clear_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_hw_btn_clear_img, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 情境鍵:板上有字=清空(擦掉當前字重寫,送 "c");沒字=刪除(backspace 已定稿
   最後一字,送 "b")。 */
static void hw_btn_clear_cb(lv_event_t *e)
{
    (void)e;
    if (s_hw_stroke_idx < 0)
    {
        bloc_handwrite_backspace();
    }
    else
    {
        bloc_handwrite_clear();
        hw_ink_clear();
    }
}

static void hw_btn_enter_cb(lv_event_t *e)
{
    (void)e;
    close_handwrite_from_pose(); /* 送 end → 手機 commit → 桌面送出 */
}

/* 退出動畫收尾:藏 view+座標歸位。 */
static void hw_exit_anim_done(lv_anim_t *a)
{
    (void)a;
    if (s_hw_view)
    {
        lv_obj_add_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_x(s_hw_view, 0);
        lv_obj_set_y(s_hw_view, 0);
    }
    hw_backdrop_hide();
}

static void hw_exit_anim_exec(void *obj, int32_t v)
{
    lv_obj_set_x((lv_obj_t *)obj, (lv_coord_t)v);
    hw_backdrop_sync((lv_coord_t)v); /* 黑底跟頁面離開同步漸退 */
}

/* 退出=取消 session(不送出)。視覺=整版往右滑出(founder 2026-07-22:從右邊
   進來就從右邊回去,跟右緣拉出的 snap 取消同款);底下先鋪好觸碰板=落點。 */
static void hw_btn_exit_cb(lv_event_t *e)
{
    (void)e;
    if (!s_hw_view_active)
        return;
    s_hw_view_active = false;
    bloc_handwrite_cancel(); /* 上行 "x" 即刻送(手機清狀態+收 skaibar) */
    if (current_hid_mode != HID_MODE_TRACKPAD)
        apply_hid_mode(HID_MODE_TRACKPAD);
    /* 滑鼠圖從左緣同步滑回(founder 2026-07-22:退出時圖要跟著頁面回來,
       鏡像進場的滑出;不然揭開時已站在原位=靜態)。 */
    if (s_top_logo && lv_obj_is_valid(s_top_logo))
    {
        lv_anim_del(s_top_logo, NULL);
        lv_obj_set_style_translate_x(s_top_logo,
                                     (lv_coord_t)-(int32_t)LV_HOR_RES, 0);
        lv_anim_t la;
        lv_anim_init(&la);
        lv_anim_set_var(&la, s_top_logo);
        lv_anim_set_exec_cb(&la, top_logo_tx_anim_exec);
        lv_anim_set_values(&la, (int32_t)-(int32_t)LV_HOR_RES, 0);
        lv_anim_set_time(&la, 200);
        lv_anim_set_path_cb(&la, lv_anim_path_ease_out);
        lv_anim_start(&la);
    }
    if (s_hw_view)
    {
        lv_obj_clear_flag(s_hw_view, LV_OBJ_FLAG_CLICKABLE); /* 滑出中不吃筆跡 */
        lv_anim_del(s_hw_view, NULL);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, s_hw_view);
        lv_anim_set_exec_cb(&a, hw_exit_anim_exec);
        lv_anim_set_values(&a, lv_obj_get_x(s_hw_view), LV_HOR_RES);
        lv_anim_set_time(&a, 200); /* 右緣拉出 snap 同款時長 */
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_set_ready_cb(&a, hw_exit_anim_done);
        lv_anim_start(&a);
    }
    LOG_I("[handwrite] close view (cancel, slide-right)");
}

/* 切輸入法(erth 鈕):收手寫(取消不送出)→開鍵盤。mode 在數字→手寫時已復位
   英文,循環=英文→數字→手寫→英文(founder 2026-07-22)。 */
static void hw_btn_mode_cb(lv_event_t *e)
{
    (void)e;
    hw_cancel_session();
    /* 循環錨定:手寫→**英文**,一律強制 mode+重建布局(founder 2026-07-22:殘留
       數字布局害循環卡「寫→數→寫」,英文永遠到不了)。 */
    kbd_pinyin_clear();
    current_keyboard_mode = KEYBOARD_MODE_LETTERS;
    if (keyboard_container != NULL)
    {
        lv_obj_t *kb_parent = lv_obj_get_parent(keyboard_container);
        lv_obj_del(keyboard_container);
        create_circular_keyboard_layout(kb_parent);
    }
    /* 切整個 app 到鍵盤模式:觸碰板模式下鍵盤整棵子樹藏著,光 toggle 可見性
       什麼都看不到(founder 2026-07-22:按了跑回觸碰板)。 */
    apply_hid_mode(HID_MODE_KEYBOARD);
    if (keyboard_container != NULL)
    {
        lv_anim_del(keyboard_container, NULL);
        lv_obj_set_style_translate_y(keyboard_container, 0, 0);
    }
    /* 正規升鍵盤(勿手動 clear HIDDEN):藏 mic 區(含黑色⌨鈕)+顯 container+
       arrows/arcs 同步。手動展開會漏藏 mic 區,退出時⌨鈕露出(founder
       2026-07-22 截圖)。**別呼 expand_anim_driver_cb(NULL,100)**——那是
       mic-view expand 的另一套 bar 幾何(380×90@y195),會把 mode_set_visible
       剛放好的 310×45@y64 拉去畫面中央(founder:「下層多一個輸入框」)。 */
    kbd_lower_set_keyboard(true);
    kbd_cand_refresh(); /* 進鍵盤立即出列(數字快捷列) */
}

/* 頂部 logo tap 的去處(founder 2026-08-03:語音**原位換掉手寫**)。
   手寫原本同時是「入口」與「循環第一站」,只換循環那一站的話入口還是會走到它 ——
   所以入口也要改過來,手寫才真的走不到(程式整段保留,比照它自己的 dead-code 待遇)。
   實作刻意重用 hw_btn_mode_cb 那條已驗過的「切進鍵盤模式」序列,末端再走
   kbd_lower_switch(false) 切到語音站,不自己手刻輸入列幾何(鍵盤站與語音站的
   bar 尺寸/位置是兩套,手刻很容易變成 founder 2026-07-22 那個「下層多一個輸入框」)。 */
/* 2026-08-16 heap 根治輪之後 gate 移除:hosted 頂部面板三頁改「用時建、離場放」,
   進場 free 常態 ~80K,原本「聊天回合後只剩 ~31K → <38K 拒絕」的處境不存在了。 */

/* 注意:`s_kbd_from_drawer` 必須由**呼叫端在進來之前**設好,不能在這裡預設清掉 ——
   本函式末端的 kbd_lower_switch(false) → kbd_bar_set_voice_box(true) 就要靠它決定
   送給電腦的 inputOnly(抽屜流程要選項,inputOnly=false)。 */
static bool mouse_open_voice_station(void)
{
    kbd_pinyin_clear();
    current_keyboard_mode = KEYBOARD_MODE_LETTERS;
    if (keyboard_container != NULL)
    {
        lv_obj_t *kb_parent = lv_obj_get_parent(keyboard_container);
        lv_obj_del(keyboard_container);
        create_circular_keyboard_layout(kb_parent);
    }
    /* 首次建置(kbd_ensure_built)走輕量版:語音站用不到圓形鍵盤,別為它把 heap 壓到
       個位數 KB(hosted 滑鼠 OOM,見 s_kbd_build_defer_wheel)。已建過(上面 rebuild
       分支)不受影響。 */
    s_kbd_build_defer_wheel = true;
    apply_hid_mode(HID_MODE_KEYBOARD);
    s_kbd_build_defer_wheel = false;
    if (keyboard_container != NULL)
    {
        lv_anim_del(keyboard_container, NULL);
        lv_obj_set_style_translate_y(keyboard_container, 0, 0);
    }
    /* 只翻旗標、不把鍵盤顯示出來:原本呼叫 kbd_lower_set_keyboard(true) 會先 un-hide
       鍵盤,接著 kbd_lower_switch(false) 再把它切走 —— 換成左右動畫之後,那一瞬間
       就變成「一整片鍵盤往右滑出」才進語音站(founder 2026-08-07)。
       kbd_lower_switch(false) 需要的只是「目前狀態算鍵盤站」這個前提。 */
    kbd_lower_is_keyboard = true;
    kbd_cand_refresh();
    kbd_lower_switch(false); /* 立刻退到語音站(輸入列高度由它的動畫帶下去) */
    LOG_I("[voice] open voice station (top-logo tap)");
    return true;
}

/* 本地軌跡:清空全部筆畫。 */
static void hw_ink_clear(void)
{
    for (int i = 0; i < HW_TRACE_STROKES; i++)
    {
        s_hw_line_n[i] = 0;
        if (s_hw_lines[i])
            lv_line_set_points(s_hw_lines[i], s_hw_line_pts[i], 0);
    }
    s_hw_stroke_idx = -1;
    s_hw_local_pen = false;
    hw_btn_clear_refresh_label(); /* 板空了→情境鍵變「刪除」 */
    hw_cand_clear_local();        /* 候選屬於板上的字,板清=候選清 */
}

/* 本地軌跡:追加一點(觸控座標=螢幕座標,無需映射)。 */
static void hw_ink_append(const lv_point_t *p)
{
    if (!s_hw_local_pen)
    {
        bool was_empty = (s_hw_stroke_idx < 0);
        if (s_hw_stroke_idx < HW_TRACE_STROKES - 1)
            s_hw_stroke_idx++;
        s_hw_line_n[s_hw_stroke_idx] = 0;
        s_hw_local_pen = true;
        if (was_empty)
            hw_btn_clear_refresh_label(); /* 開始寫字→情境鍵變「清空」 */
    }
    int idx = s_hw_stroke_idx;
    uint16_t n = s_hw_line_n[idx];
    if (n < HW_TRACE_PTS &&
        (n == 0 || s_hw_line_pts[idx][n - 1].x != p->x ||
         s_hw_line_pts[idx][n - 1].y != p->y))
    {
        s_hw_line_pts[idx][n] = *p;
        s_hw_line_n[idx] = (uint16_t)(n + 1);
        if (s_hw_lines[idx])
            lv_line_set_points(s_hw_lines[idx], s_hw_line_pts[idx], s_hw_line_n[idx]);
    }
}

static void ensure_hw_view(void)
{
    if (s_hw_view != NULL)
        return;
    lv_obj_t *host = hid_mouse_ui_host();
    if (host == NULL)
        host = lv_scr_act();
    /* 切換途中 view 超出右緣會讓 host 長滾動條(founder 2026-07-22):關掉。 */
    lv_obj_set_scrollbar_mode(host, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(host, LV_OBJ_FLAG_SCROLLABLE);

    /* 進場黑底:獨立 backdrop 原地漸黑(不跟 view 滑),view 本體透明=只有元件
       滑進來(founder 2026-07-22)。 */
    s_hw_backdrop = lv_obj_create(host);
    lv_obj_remove_style_all(s_hw_backdrop);
    lv_obj_set_size(s_hw_backdrop, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_hw_backdrop, 0, 0);
    lv_obj_set_style_bg_color(s_hw_backdrop, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_hw_backdrop, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(s_hw_backdrop, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_hw_backdrop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_hw_backdrop, LV_OBJ_FLAG_HIDDEN);

    s_hw_view = lv_obj_create(host);
    lv_obj_remove_style_all(s_hw_view);
    lv_obj_set_size(s_hw_view, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_hw_view, 0, 0);
    /* view 本體透明(黑底由 backdrop 提供);本地軌跡/按鈕等元件跟著 view 滑。 */
    lv_obj_set_style_bg_opa(s_hw_view, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(s_hw_view, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_hw_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_hw_view, hw_view_event_cb, LV_EVENT_ALL, NULL);

    /* 頂部=mouse_mode_icon(取消不送出;founder 2026-07-30:輸入頁頂部改放滑鼠圖=「點我回
       滑鼠模式」,與觸控板頂部鍵盤圖對調——顯示目標模式。點本頁頂部=滑回退出)。位置
       對齊 status_bar 頂部 80 高感應區置中。 */
    s_hw_btn_exit = lv_obj_create(s_hw_view);
    lv_obj_remove_style_all(s_hw_btn_exit);
    lv_obj_set_size(s_hw_btn_exit, 100, 80);
    lv_obj_align(s_hw_btn_exit, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(s_hw_btn_exit, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_hw_btn_exit, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_hw_btn_exit, hw_btn_exit_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *exit_img = lv_img_create(s_hw_btn_exit);
    lv_img_set_src(exit_img, &mouse_mode_icon);
    lv_img_set_zoom(exit_img, 180);
    lv_obj_center(exit_img);
    lv_obj_clear_flag(exit_img, LV_OBJ_FLAG_CLICKABLE);

    /* 候選列=鍵盤輸入框同款 pill(founder 2026-07-22 視覺統一:同位(43,64)、同
       310×45、同深色底+白框+radius 100——兩模式看起來是同一條輸入列)。列本體
       不可點,候選鈕(透明+白字)自己吃 press。 */
    s_hw_cand_row = lv_obj_create(s_hw_view);
    lv_obj_remove_style_all(s_hw_cand_row);
    lv_obj_set_size(s_hw_cand_row, 310, 45);
    lv_obj_set_pos(s_hw_cand_row, (LV_HOR_RES_MAX - 310) / 2, 64); /* 置中,同鍵盤 bar */
    lv_obj_set_style_bg_color(s_hw_cand_row, lv_color_hex(0x1a1a1a), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_hw_cand_row, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_hw_cand_row, lv_color_hex(0xFFFFFF),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(s_hw_cand_row, 2, LV_PART_MAIN);
    lv_obj_set_style_border_opa(s_hw_cand_row, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_radius(s_hw_cand_row, 100, LV_PART_MAIN);
    lv_obj_clear_flag(s_hw_cand_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_hw_cand_row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_flex_flow(s_hw_cand_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_hw_cand_row, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(s_hw_cand_row, LV_OBJ_FLAG_HIDDEN);
    for (int i = 0; i < HW_CAND_MAX; i++)
    {
        s_hw_cand_btns[i] = lv_obj_create(s_hw_cand_row);
        lv_obj_set_size(s_hw_cand_btns[i], 52, 39);
        lv_obj_set_style_bg_opa(s_hw_cand_btns[i], LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(s_hw_cand_btns[i], 0, LV_PART_MAIN);
        lv_obj_clear_flag(s_hw_cand_btns[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(s_hw_cand_btns[i], hw_cand_btn_cb, LV_EVENT_CLICKED,
                            (void *)(intptr_t)i);
        s_hw_cand_lbls[i] = lv_label_create(s_hw_cand_btns[i]);
        lv_obj_set_style_text_color(s_hw_cand_lbls[i], lv_color_hex(0xFFFFFF), 0);
        lv_label_set_text(s_hw_cand_lbls[i], "");
        lv_obj_center(s_hw_cand_lbls[i]);
        lv_obj_add_flag(s_hw_cand_btns[i], LV_OBJ_FLAG_HIDDEN);
    }

    /* 本地軌跡線(建立在字樣之後=畫在其上層) */
    for (int i = 0; i < HW_TRACE_STROKES; i++)
    {
        s_hw_lines[i] = lv_line_create(s_hw_view);
        lv_obj_set_style_line_width(s_hw_lines[i], 4, 0);
        lv_obj_set_style_line_color(s_hw_lines[i], lv_color_hex(0xA6D3E6), 0);
        lv_obj_set_style_line_rounded(s_hw_lines[i], true, 0);
    }

    /* 底部只剩情境鍵(清空/刪除),置中(founder 2026-07-22:下個字/輸入退出底部——
       定稿一律按候選字,輸入移到候選欄)。透明容器照鍵盤頁 del_btn 樣式(founder:
       預設藍鈕醜);按鈕自己吃 press,不會 bubble 進筆跡。 */
    s_hw_btn_clear = lv_obj_create(s_hw_view);
    /* 位置=鍵盤頁 del_btn 的螢幕絕對位(founder 2026-07-22 對齊肌肉記憶):
       container top 166 + pad 8 + 內部(300,195) → (308,369),同 80×50。 */
    lv_obj_set_size(s_hw_btn_clear, 80, 50);
    lv_obj_set_pos(s_hw_btn_clear, 308, 369);
    lv_obj_set_style_bg_opa(s_hw_btn_clear, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_hw_btn_clear, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_hw_btn_clear, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_hw_btn_clear, hw_btn_clear_cb, LV_EVENT_CLICKED, NULL);
    s_hw_btn_clear_lbl = lv_label_create(s_hw_btn_clear);
    lv_obj_set_style_text_color(s_hw_btn_clear_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(s_hw_btn_clear_lbl);
    s_hw_btn_clear_img = lv_img_create(s_hw_btn_clear);
    lv_img_set_src(s_hw_btn_clear_img, &backspace_icon);
    lv_obj_align(s_hw_btn_clear_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_img_opa(s_hw_btn_clear_img, LV_OPA_50, LV_PART_MAIN);
    lv_obj_clear_flag(s_hw_btn_clear_img, LV_OBJ_FLAG_CLICKABLE);
    hw_btn_clear_refresh_label(); /* 板上有字=清空文字/沒字=刪除圖 */

    /* 切輸入法鈕:位置=鍵盤 mode_btn 絕對位(container 166+pad 8+內部(65,195)→
       (73,369)),同 erth 圖;點=收手寫開鍵盤(founder 2026-07-22 循環)。 */
    s_hw_btn_mode = lv_obj_create(s_hw_view);
    lv_obj_set_size(s_hw_btn_mode, 50, 50);
    lv_obj_set_pos(s_hw_btn_mode, 73, 369);
    lv_obj_set_style_bg_opa(s_hw_btn_mode, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_hw_btn_mode, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_hw_btn_mode, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_hw_btn_mode, hw_btn_mode_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *hw_mode_img = lv_img_create(s_hw_btn_mode);
    lv_img_set_src(hw_mode_img, &erth);
    lv_obj_center(hw_mode_img);
    lv_obj_clear_flag(hw_mode_img, LV_OBJ_FLAG_CLICKABLE);

    /* 輸入鈕=候選欄的空狀態:與候選列同一位置,候選清單沒東西才顯示
       (有候選→先按候選定稿,才輪得到送出)。 */
    s_hw_btn_enter = lv_obj_create(s_hw_view);
    lv_obj_set_size(s_hw_btn_enter, 310, 45);
    lv_obj_set_pos(s_hw_btn_enter, (LV_HOR_RES_MAX - 310) / 2, 64); /* 置中,同鍵盤 bar */
    lv_obj_set_style_bg_color(s_hw_btn_enter, lv_color_hex(0x1a1a1a), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_hw_btn_enter, LV_OPA_90, LV_PART_MAIN);
    /* 亮起提示(founder 2026-07-22):候選清空=可送出,sky-accent 全亮框+字,
       跟一般白框 pill 一眼區分。 */
    lv_obj_set_style_border_color(s_hw_btn_enter, lv_color_hex(0xA6D3E6),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(s_hw_btn_enter, 2, LV_PART_MAIN);
    lv_obj_set_style_border_opa(s_hw_btn_enter, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_hw_btn_enter, 100, LV_PART_MAIN);
    lv_obj_clear_flag(s_hw_btn_enter, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_hw_btn_enter, hw_btn_enter_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_enter = lv_label_create(s_hw_btn_enter);
    lv_obj_set_style_text_color(lbl_enter, lv_color_hex(0xA6D3E6), 0);
    lv_label_set_text(lbl_enter, LV_EXT_STR_GET_BY_KEY(handwrite_enter, "Enter"));
    lv_obj_center(lbl_enter);
}

/* snap 動畫完成:展開→正式開啟 session。 */
static void hw_pull_anim_open_done(lv_anim_t *a)
{
    (void)a;
    hw_open_commit();
}

static void hw_pull_anim_cancel_done(lv_anim_t *a)
{
    (void)a;
    if (s_hw_view)
    {
        lv_obj_add_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_x(s_hw_view, 0);
    }
    hw_backdrop_hide();
}

static void hw_pull_anim_exec(void *obj, int32_t v)
{
    lv_obj_set_x((lv_obj_t *)obj, (lv_coord_t)v);
    hw_backdrop_sync((lv_coord_t)v); /* 黑底跟滑入進度漸黑/漸退 */
}

/* 放開後 snap:open=滑到 0 並 commit、!open=滑回右緣並藏。 */
static void hw_pull_snap(bool open)
{
    if (s_hw_view == NULL)
        return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_hw_view);
    lv_anim_set_exec_cb(&a, hw_pull_anim_exec);
    lv_anim_set_values(&a, lv_obj_get_x(s_hw_view), open ? 0 : LV_HOR_RES);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, open ? hw_pull_anim_open_done
                                  : hw_pull_anim_cancel_done);
    lv_anim_start(&a);
}

/* ── 右緣拉出跟手三段式(照 app_clock_status_bar 的 clock_main_*_follow_begin/
   update/end,founder 指定體感,方向改右→左):press 由觸控機持有並轉發位移
   (sole writer 無抖動;view 拉出期間不可點,press 不會被 hit-test 搶走),放開以
   「拉過 1/4 螢幕 或 快甩(vx≤-6/tick)」commit,set-tile 的 snap 用 lv_anim 等價。 ── */

/* begin:view 於右緣現形(關 CLICKABLE)並定位到當下已拉距離。尚未 commit(不送 start)。 */
static void hw_pull_begin(lv_coord_t dx0, lv_coord_t finger_x)
{
    if (s_hw_view_active || s_hw_pull_active)
        return;
    if (!(is_at_mouse_mode() || app_control_get_mouse_mode()))
        return;
    extern bool instruction_list_lift_input_view_open(void);
    if (instruction_list_lift_input_view_open())
        return; /* 語音輸入中不搶 */
    dial_drag_state_reset(); /* 清 hold timer/拖曳/側立圓盤殘留 */
    ensure_hw_view();
    lv_anim_del(s_hw_view, NULL); /* 殘留 snap/退出動畫 */
    lv_obj_set_y(s_hw_view, 0);   /* 退出下滑被中斷的殘留位移歸位 */
    hw_ink_clear();
    lv_obj_clear_flag(s_hw_view, LV_OBJ_FLAG_CLICKABLE); /* 拉出期間 press 留在觸控板 */
    lv_coord_t x = LV_HOR_RES + dx0; /* dx0<0(向左拉)→x<466 */
    if (x < 0) x = 0;
    if (x > LV_HOR_RES) x = LV_HOR_RES;
    lv_obj_set_x(s_hw_view, x);
    lv_obj_clear_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_hw_view);
    s_hw_pull_vx = 0;
    s_hw_pull_prev_x = finger_x;
    s_hw_pull_active = true;
}

/* update:每 PRESSING 轉發累積位移(dx<0=向左),並記單 tick 步進當 fling 依據。 */
static void hw_pull_follow_update(lv_coord_t dx, lv_coord_t finger_x)
{
    if (!s_hw_pull_active || s_hw_view == NULL)
        return;
    s_hw_pull_vx = finger_x - s_hw_pull_prev_x;
    s_hw_pull_prev_x = finger_x;
    lv_coord_t x = LV_HOR_RES + dx;
    if (x < 0) x = 0;
    if (x > LV_HOR_RES) x = LV_HOR_RES;
    lv_obj_set_x(s_hw_view, x);
}

/* end:放開 commit 判定=拉過 1/4 螢幕 或 快甩(參考 applist_follow_end 的 dx≤-1/4 || vx<-6)。 */
static void hw_pull_follow_end(lv_coord_t dx)
{
    if (!s_hw_pull_active)
        return;
    s_hw_pull_active = false;
    hw_pull_snap(dx <= -(LV_HOR_RES / 4) || s_hw_pull_vx <= -6);
}

/* press 被搶(PRESS_LOST):縮回不 commit。 */
static void hw_pull_cancel(void)
{
    if (!s_hw_pull_active)
        return;
    s_hw_pull_active = false;
    hw_pull_snap(false);
}

/* snap 展開完成:正式開啟(開 CLICKABLE 接筆跡、送 0x1b start,桌面書寫區現形)。 */
static void hw_open_commit(void)
{
    if (s_hw_view_active)
        return;
    lv_obj_set_x(s_hw_view, 0);
    lv_obj_add_flag(s_hw_view, LV_OBJ_FLAG_CLICKABLE);
    /* 黑底全黑就位(mode-switch 直開等路徑沒走 staging 也要有黑底)。 */
    if (s_hw_backdrop)
    {
        lv_obj_clear_flag(s_hw_backdrop, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_hw_backdrop);
        lv_obj_move_foreground(s_hw_view);
        hw_backdrop_sync(0);
    }
    /* 左拉進場的滑鼠圖:頁面已蓋滿,無感歸位(退出滑回時要在原位)。 */
    if (s_top_logo && lv_obj_is_valid(s_top_logo))
    {
        lv_anim_del(s_top_logo, NULL);
        lv_obj_set_style_translate_x(s_top_logo, 0, 0);
    }
    s_hw_view_active = true;
    bloc_handwrite_begin(LV_HOR_RES, LV_VER_RES); /* 送 0x1b start(畫布=錶面解析度) */
    motor_pattern_unlocked(); /* 短震=手寫就緒 */
    LOG_I("[handwrite] open view canvas=%dx%d", (int)LV_HOR_RES, (int)LV_VER_RES);
}

/* GUI thread:「輸入」鈕=送出並收掉手寫。冪等。
   (2026-07-20:姿勢保險已拆,這裡只剩送出這一個入口——不會再有自動執行。) */
static void close_handwrite_from_pose(void)
{
    if (!s_hw_view_active)
        return;
    s_hw_view_active = false;
    bloc_handwrite_end(); /* 旗標交 motion thread 送 pending "u"+"end"(手機 commit 辨識) */
    if (s_hw_view)
    {
        lv_anim_del(s_hw_view, NULL);
        lv_obj_add_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_x(s_hw_view, 0);
        lv_obj_set_y(s_hw_view, 0);
    }
    hw_backdrop_hide();
    LOG_I("[handwrite] close view (submit)");
}

/* GUI thread:取消收掉手寫(退出鈕/離開或暫停 app 清殘留)。**不送出**。冪等。 */
static void hw_cancel_session(void)
{
    if (s_hw_pull_active)
    {
        /* 拉到一半被收:直接取消,不送任何 frame(session 未開) */
        s_hw_pull_active = false;
        if (s_hw_view)
        {
            lv_anim_del(s_hw_view, NULL);
            lv_obj_add_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_x(s_hw_view, 0);
            lv_obj_set_y(s_hw_view, 0);
        }
        hw_backdrop_hide();
    }
    if (!s_hw_view_active)
        return;
    s_hw_view_active = false;
    bloc_handwrite_cancel(); /* 旗標交 motion thread 送 "x"(手機清狀態+收 skaibar) */
    if (s_hw_view)
    {
        lv_anim_del(s_hw_view, NULL);
        lv_obj_add_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_x(s_hw_view, 0);
        lv_obj_set_y(s_hw_view, 0);
    }
    hw_backdrop_hide();
    LOG_I("[handwrite] close view (cancel)");
}

/* 手寫頁 staging:備妥於右緣外(x=LV_HOR_RES,可見,未 commit)。回 false=情境不允許。
   tap 滑入與圖示左拉跟手共用(founder 2026-07-22)。 */
static bool hw_view_stage_offscreen(void)
{
    if (s_hw_view_active || s_hw_pull_active)
        return false;
    if (!(is_at_mouse_mode() || app_control_get_mouse_mode()))
        return false;
    extern bool instruction_list_lift_input_view_open(void);
    if (instruction_list_lift_input_view_open())
        return false; /* 語音輸入中不搶 */
    dial_drag_state_reset();
    ensure_hw_view();
    lv_anim_del(s_hw_view, NULL);
    lv_obj_set_y(s_hw_view, 0);
    hw_ink_clear();
    lv_obj_clear_flag(s_hw_view, LV_OBJ_FLAG_CLICKABLE); /* 進場中不吃筆跡 */
    lv_obj_set_x(s_hw_view, LV_HOR_RES);
    lv_obj_clear_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
    if (s_hw_backdrop)
    {
        lv_obj_clear_flag(s_hw_backdrop, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_hw_backdrop); /* 蓋觸碰板,墊在 view 下 */
        hw_backdrop_sync(LV_HOR_RES);          /* opa 0 起步 */
    }
    lv_obj_move_foreground(s_hw_view);
    /* 同幀 set_x 後 coords 未 realize,get_x 讀到舊值 0 → snap 變 0→0 瞬切
       (founder 2026-07-22:「點了瞬間切換沒看到滑入」)。強制 layout 讓
       hw_pull_snap 拿到真起點 466。 */
    lv_obj_update_layout(s_hw_view);
    return true;
}


/* GUI thread:鍵盤 Mode 鈕第三站——程式化直開手寫(無 snap 動畫)。
   冪等;非滑鼠 app 情境直接 no-op。 */
static void hw_open_from_mode_switch(void)
{
    if (s_hw_view_active)
        return;
    if (!(is_at_mouse_mode() || app_control_get_mouse_mode()))
        return;
    dial_drag_state_reset(); /* 清 hold timer/拖曳/側立圓盤殘留(同 pull begin) */
    ensure_hw_view();
    lv_anim_del(s_hw_view, NULL);
    hw_ink_clear();
    lv_obj_set_x(s_hw_view, 0);
    lv_obj_set_y(s_hw_view, 0);
    lv_obj_clear_flag(s_hw_view, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_hw_view);
    hw_open_commit(); /* CLICKABLE+0x1b start+短震 */
}

// 關 mic（不清空 input bar、不貼上）
// 清空只在使用者按 Enter 時做（鍵盤 Enter 鍵 / 送出鍵）
static void mouse_v2t_close_and_paste(void)
{
#ifndef BSP_USING_PC_SIMULATOR
    interact_mic_listen(false); /* PC sim 沒有音訊 HAL,同 open 那條一起擋掉 */
#endif
    mouse_v2t_set_active(false);
    LOG_D("V2T close (input bar preserved)");
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
    /* 語音站:**插在插入點**,不是整段取代(founder 2026-08-03)。V2T 是串流累積的「目前為止
       全文」,所以每次都用「按下麥克風當刻的前半 + 這次的全文 + 後半」重組 —— 邊講邊長,
       前後既有的字不會被蓋掉。鍵盤站維持原本的整段取代。 */
    if (s_voice_box_on && s_voice_ins_armed)
    {
        size_t pre = strlen(s_voice_ins_prefix), suf = strlen(s_voice_ins_suffix);
        if (pre + text_len + suf >= sizeof(input_buffer))
            text_len = sizeof(input_buffer) - 1 - pre - suf; /* 截尾,不動前後文 */
        memset(input_buffer, 0, sizeof(input_buffer));
        memcpy(input_buffer, s_voice_ins_prefix, pre);
        memcpy(input_buffer + pre, text, text_len);
        memcpy(input_buffer + pre + text_len, s_voice_ins_suffix, suf);
        input_length = (int)(pre + text_len + suf);
        s_voice_caret = (int)(pre + text_len); /* 游標跟到這段的尾巴 */
        if (input_display_label != NULL)
            update_input_display();
        return;
    }
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
    /* TEMP DIAG(2026-08-16 鍵盤語音沒文字):W 級確認文字有走到滑鼠輸入列這一站。 */
    LOG_W("[v2t] -> mouse input len=%d", text ? (int)strlen(text) : -1);
    if (text == NULL)
        return;
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MOUSE_INPUT_TEXT;
    msg.data.message = text;
    lvgl_send_msg(msg);
}

static rt_tick_t text_input_bar_press_time = NULL;
static uint16_t max_move_y = 0;

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
// 用 rt_timer 而非 lv_timer：拖曳時 BLE queue 會把 LVGL task 塞到 lv_timer
// 延遲數百 ms 才 fire；rt_timer 跑在獨立 timer thread，不受 LVGL 佔用影響
static rt_timer_t bottom_bar_multitask_timer = NULL;

/* T4 (b): hosted-mode return hook. When device_pager hosts the mouse it
   registers a back callback; the bottom-bar up gesture then returns to the
   instruction layer instead of firing multitask. */
static bool s_hosted = false;
static void (*s_host_back_cb)(void) = NULL;
static void (*s_host_pull_cb)(int up_px, int released) = NULL;

/* Trackpad bottom bar: tap → 開「控制中設備」的 skaibar（AI 對話 + 該設備選項，選項由
   active 設備經 KEY_SKAIBAR_OPTIONS 送來）。設備選擇已搬進滑鼠 app 自己的右拉抽屜，
   故這條 bar 重新啟用（先前因「device 頁擁有語音輸入」暫時關閉）。up-drag 仍是
   multitask / host pull-back。 */
static bool s_bottom_input_disabled = false;
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
        /* 2026-08-07:長按開「立起輸入面板」也退役。2026-08-03 那次只拿掉了短按
           那條(見下面 RELEASED 分支的註解)與姿勢入口(bloc_motion_tracking),長按
           這條漏掉,所以面板看起來「刪過了還在」(founder)。至此 hid_mouse 這邊
           對 open_skaibar_from_pose / hid_mouse_toggle_lift_input_panel 已無任何
           呼叫者,比照手寫與姿勢入口:程式原地保留、不再被走到。
           輸入的唯一入口 = 底部 keyboard_icon → 鍵盤模式第四站(語音站)。 */
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

        // 向上拖（dy < 0）→ multitask hint 流程已停用(2026-07-02 使用者要求:
        // 誤觸多頁面指令)。dy<0 不再進 pending — 不出 hint、不 fire multitask;
        // hit area 也已縮半(bottom_swipe_area),縮小攔截範圍。tap/長按開
        // skaibar 不受影響。要復原把條件改回 (dy < 0)。
        if (0)
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

            if (!s_bottom_input_disabled &&
                max_move_y < 10 && !bottom_bar_gesture_timer_enabled &&
                !is_bottom_bar_gesture_active && !bar_long_press_fired)
            {
                /* 底部 bar 純點擊曾經是「開立起輸入面板」(founder 2026-07-31),
                   2026-08-03 退役:語音輸入改成鍵盤模式的第四站,入口單一化。
                   這裡不接手其他行為 —— 上方 logo tap 進鍵盤模式的路徑沒有變。 */
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
        // 進場淡入：cv 額外乘 s_node_entrance/1000，0 時純黑融入背景（用顏色淡，
        // 不動 line_opa，避免 round-cap 殘色 / layer 開銷）
        uint8_t cv = (uint8_t)((float)0x4D * blend * (float)s_node_entrance / 1000.0f);
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
        uint8_t cv = (uint8_t)((float)0x4D * blend * (float)s_node_entrance / 1000.0f);
        lv_color_t node_color = lv_color_make(cv, cv, cv);

        lv_line_set_points(right_scroll_nodes[i], right_scroll_node_pts[i], 2);
        lv_obj_set_style_line_width(right_scroll_nodes[i], line_w, 0);
        lv_obj_set_style_line_color(right_scroll_nodes[i], node_color, 0);
    }
}

/* Scroll-wheel entrance fade (hosted/device_pager): the wheel is the gray tick
   NODES (the arc itself is always transparent), drawn by COLOR not opacity, so
   we fade them in by ramping s_node_entrance 0→1000 (black→gray) — no layer
   opacity (too costly/fails on a full-screen object on the watch) and no
   line_opa animation (round-cap residue). */
static void node_entrance_anim_cb(void *var, int32_t v)
{
    (void)var;
    s_node_entrance = v;
    update_left_scroll_nodes();
}

void hid_mouse_fade_in_scroll_wheel(void)
{
    if (left_scroll_nodes[0] == NULL) return; /* nodes not built yet */
    lv_anim_del(&s_node_entrance, node_entrance_anim_cb);
    s_node_entrance = 0;
    update_left_scroll_nodes();               /* start fully black (invisible) */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_node_entrance);
    lv_anim_set_exec_cb(&a, node_entrance_anim_cb);
    lv_anim_set_values(&a, 0, 1000);
    lv_anim_set_time(&a, 400);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
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

/* y 版本:輸入模式「由下往上進場」用(見 bottom_logo_cb)。跟 x 一樣直接改本體位置。 */
static void mode_set_y(hid_mode_t mode, int16_t ty)
{
    if (mode_container[mode] && lv_obj_is_valid(mode_container[mode]))
        lv_obj_set_y(mode_container[mode], ty);
}

/* 觸碰板的左右滾動弧+節點放在 bg 跨 mode 層(不隨 mode_container 顯藏),
   鍵盤模式不該出現(founder 2026-07-22:「鍵盤左右邊的滾動條」)。 */
static void scroll_arcs_set_hidden(bool hidden)
{
    lv_obj_t *arcs[2] = {left_scroll_bar, right_scroll_bar};
    for (int i = 0; i < 2; i++)
    {
        if (arcs[i] == NULL || !lv_obj_is_valid(arcs[i]))
            continue;
        if (hidden)
            lv_obj_add_flag(arcs[i], LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(arcs[i], LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < LEFT_SCROLL_NODE_COUNT; i++)
    {
        if (left_scroll_nodes[i] && lv_obj_is_valid(left_scroll_nodes[i]))
        {
            if (hidden)
                lv_obj_add_flag(left_scroll_nodes[i], LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_clear_flag(left_scroll_nodes[i], LV_OBJ_FLAG_HIDDEN);
        }
        if (right_scroll_nodes[i] && lv_obj_is_valid(right_scroll_nodes[i]))
        {
            if (hidden)
                lv_obj_add_flag(right_scroll_nodes[i], LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_clear_flag(right_scroll_nodes[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* 鍵盤模式的跨層附件同步(滾動弧藏/退出鈕顯)。swipe 換模式的 commit 不走
   mode_set_visible(容器靠 translate 推出畫面),附件要在 commit 點自行 sync。 */
static void kbd_mode_extras_sync(void)
{
    bool kb = (current_hid_mode == HID_MODE_KEYBOARD);
    scroll_arcs_set_hidden(kb);
    /* 2026-08-07 founder:進輸入畫面後鍵盤圖與滑鼠圖都不要出現 → 這顆退出鈕
       一律隱藏(物件保留,cb 還在,只是沒有可見入口)。回觸控板走右滑 /
       語音站的右箭頭。 */
    (void)kb;
    if (kbd_exit_btn && lv_obj_is_valid(kbd_exit_btn))
        lv_obj_add_flag(kbd_exit_btn, LV_OBJ_FLAG_HIDDEN);
    if (kb)
        kbd_cand_refresh(); /* swipe 進鍵盤也要立即出列(數字/推薦) */
}

static void mode_set_visible(hid_mode_t mode, bool visible)
{
    /* R40:鍵盤模式那一整套 UI 改成**第一次真的要顯示時才建**。量測結果:滑鼠圖層
       共 41.7KB,其中 create_keyboard_mode_ui 一支就 38KB(trackpad 只有 3.6KB)——
       而從媒體欄進來的人用的是 trackpad,鍵盤常常整個 session 都沒用到。這 38KB 正
       是「進滑鼠頁只剩 6KB、退出時亮 tileview 就 sys memory is full」的元凶。
       這裡是所有進鍵盤路徑(apply_hid_mode / expand 動畫 / 直接 set_visible)的共同
       關卡,掛在這一個點就全覆蓋。 */
    if (mode == HID_MODE_KEYBOARD && visible)
        kbd_ensure_built();
    if (mode_container[mode] && lv_obj_is_valid(mode_container[mode]))
    {
        if (visible)
            lv_obj_clear_flag(mode_container[mode], LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(mode_container[mode], LV_OBJ_FLAG_HIDDEN);
    }

    // 觸碰板滾動弧+節點:鍵盤模式進=藏、離開=顯回(bg 跨 mode 層,不會自動跟)
    if (mode == HID_MODE_KEYBOARD)
        scroll_arcs_set_hidden(visible);

    // 鍵盤頂部退出鈕(同上跨 mode 層):進鍵盤=顯、離開=藏
    /* 見 kbd_mode_extras_sync:輸入畫面不再顯示這顆滑鼠圖,恆藏。 */
    if (kbd_exit_btn && lv_obj_is_valid(kbd_exit_btn))
        lv_obj_add_flag(kbd_exit_btn, LV_OBJ_FLAG_HIDDEN);

    // 選字/推薦/數字列(跨 mode 層):離開鍵盤=清狀態+藏;進鍵盤=立即刷新
    // (founder 2026-07-22:進英文畫面要馬上看到數字列,不能等打字才出現)
    if (mode == HID_MODE_KEYBOARD)
    {
        if (visible)
            kbd_cand_refresh();
        else
            kbd_pinyin_clear();
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
    // status_bar_area_up 同理：480×80 的頂部下拉感應區跟輸入列
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
    /* keyboard_icon 2026-08-07 搬到底部之後不再是 status_bar_area_up 的子物件，
       keyboard mode 要自己藏(下面整片是鍵盤，不該再壓一張圖)。 */
    if (mode == HID_MODE_KEYBOARD && s_top_logo && lv_obj_is_valid(s_top_logo))
    {
        if (visible)
            lv_obj_add_flag(s_top_logo, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(s_top_logo, LV_OBJ_FLAG_HIDDEN);
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
                // 位置與大小：toggle_keyboard 的 open 終點
                int32_t open_x = (LV_HOR_RES_MAX - 310) / 2; /* 置中(同 toggle) */
                int32_t open_y = 64; /* =手寫候選欄同高(founder 2026-07-22 統一輸入列) */
                lv_obj_set_pos(text_input_bar_bg, open_x, open_y);
                lv_obj_set_size(text_input_bar_bg, 310, 45);
            }
            if (input_content_container &&
                lv_obj_is_valid(input_content_container))
                lv_obj_clear_flag(input_content_container, LV_OBJ_FLAG_HIDDEN);
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
        mode_set_y((hid_mode_t)i, 0); /* 進場動畫被打斷時也保證歸位 */
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
    kbd_mode_extras_sync(); /* swipe 進鍵盤不走 mode_set_visible(KEYBOARD,true) */
    LOG_D("mode commit (async) -> %s", hid_mode_names[current_hid_mode]);
}

static void mode_swipe_cancel_async_cb(void *user_data)
{
    (void)user_data;
    mode_set_visible(mode_swipe_target, false);
    mode_set_translate_x(current_hid_mode, 0);
    mode_set_translate_x(mode_swipe_target, 0);
    mode_swipe_active = false;
    kbd_mode_extras_sync(); /* 取消回原 mode,附件跟 current 對齊 */
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

    // 滑鼠模式底部常駐的 skaibar 視覺 bar（只放那張圖）。non-clickable，點擊穿透到
    // bottom_swipe_area 開 skaibar。由 poll 查 instruction_list_floating_bar_visible() 同步：
    // 共用浮層 bar 一現就收它、一收就還原它，frame 對齊兩條的交接，避免重疊/閃/空窗。
    trackpad_mic_btn = lv_obj_create(parent);
    lv_obj_set_size(trackpad_mic_btn, 176, 31);
    lv_obj_align(trackpad_mic_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_opa(trackpad_mic_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(trackpad_mic_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(trackpad_mic_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(trackpad_mic_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(trackpad_mic_btn, LV_OBJ_FLAG_CLICKABLE);
    /* 底部的入口圖在 s_top_logo(hosted 模式顯示 skaibar_img,見其建立處);這個
       容器只承載底部 bar 的多工提示/手勢狀態機,不再放第二張圖。 */

    /* 右緣鍵盤鈕(founder 2026-08-17):**電腦上點了任何輸入框**時才浮現 —— 按它直接
       進輸入頁,打的字送去電腦那個聚焦中的欄位。底部 bar 從此專責「找東西(抽屜)」,
       打字有自己的入口,兩件事不再共用一顆按鈕。
       顯藏由既有的 40ms poll(bar_ai_sync_timer_cb)依 0x17 快取旗標驅動 —— 旗標本身
       是通訊執行緒寫的,不能在那邊碰 LVGL(本檔最典型的當機來源)。
       造型沿用語音站底部那顆鍵盤鈕(50 圓 + keyboard_icon zoom 150),同一件事同一個樣子。 */
    kbd_side_btn = lv_obj_create(parent);
    lv_obj_remove_style_all(kbd_side_btn);
    lv_obj_set_size(kbd_side_btn, 50, 50);
    lv_obj_align(kbd_side_btn, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_bg_color(kbd_side_btn, lv_color_hex(0x333333), 0);
    lv_obj_set_style_bg_opa(kbd_side_btn, LV_OPA_60, 0);
    lv_obj_set_style_radius(kbd_side_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(kbd_side_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(kbd_side_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(kbd_side_btn, 8); /* 視覺 50,觸控補到 66 > 44pt 基線 */
    lv_obj_add_event_cb(kbd_side_btn, kbd_side_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(kbd_side_btn, LV_OBJ_FLAG_HIDDEN); /* 預設收起,有聚焦才浮現 */
    {
        lv_obj_t *side_img = lv_img_create(kbd_side_btn);
        lv_img_set_src(side_img, &keyboard_icon);
        lv_img_set_zoom(side_img, 150);
        lv_obj_center(side_img);
        lv_obj_clear_flag(side_img, LV_OBJ_FLAG_CLICKABLE);
    }

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
/* 語音站空白處**輕點**→切到鍵盤(founder 2026-08-07)。
   必須綁 LV_EVENT_SHORT_CLICKED,不能跟退出那支共用 LV_EVENT_CLICKED ——
   LVGL v8 放開時 SHORT_CLICKED 只在「沒觸發長按」才送,CLICKED 則**長按也照送**。
   第一版錯放在 CLICKED 那支,於是「長按輸入框錄音、放開」也被判成輕點,
   kbd_lower_switch(true) 開頭的 `if (to_kbd && mouse_v2t_active)
   mouse_v2t_close_and_paste()` 當場把那段錄音收掉 → 沒有文字回來。 */
static void voice_tap_to_kbd_cb(lv_event_t *e)
{
    if (lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    /* 同 keyboard_mode_outside_click_cb:擋掉觸控驅動的 (0,0) 假放開事件,
       否則 i2c 一出錯就會自己跳去鍵盤。 */
    {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t p;
        if (indev)
        {
            lv_indev_get_point(indev, &p);
            if (p.x == 0 && p.y == 0) return;
        }
    }
    if (!s_voice_box_on) return;
    if (current_hid_mode != HID_MODE_KEYBOARD) return;
    if (collapse_anim_running) return;
    if (mouse_v2t_active) return; /* 錄音中不理會,免得手一歪把正在講的拽走 */
    kbd_lower_switch(true);
}

static void keyboard_mode_container_hit_test_cb(lv_event_t *e)
{
    lv_hit_test_info_t *info = lv_event_get_hit_test_info(e);
    if (info->res == false) return;
    /* 這些排除區是留給左右滾動弧的,而弧只在**語音站**有作用 ——
       kbd_lower_update_arcs_visibility() 一進鍵盤就把它們全藏了。原本這裡卻無
       條件排除,等於鍵盤畫面有幾條看不見的死區在吃按壓,而且愈靠下愈寬:
       弧帶(離邊 25px)與中央捲動區(再往內 50px)都受 ±55° 角度限制,在第四行
       (絕對 y≈394)換算出來左側吃掉 x≈65..189 —— 地球鍵(73..123)整顆在裡面、
       空白鍵(168..288)左緣也被切到,中文模式空白再下移 40px 更嚴重。
       (founder 2026-08-07:下面那排很難按到) → 鍵盤畫面直接不套用。 */
    if (kbd_lower_is_keyboard) return;
    if (is_point_in_left_arc(info->point) ||
        is_point_in_right_arc(info->point) ||
        is_point_in_center_scroll_zone(info->point))
        info->res = false;
}

/* R40:第一次要顯示鍵盤模式時才建它的 UI(38KB)。所有進鍵盤的路徑都會經過
   mode_set_visible(HID_MODE_KEYBOARD, true),那裡是唯一呼叫點。 */
static void create_keyboard_mode_ui(lv_obj_t *parent);

static void kbd_ensure_built(void)
{
    if (s_kbd_ui_built)
        return;
    lv_obj_t *kc = mode_container[HID_MODE_KEYBOARD];
    if (kc == NULL || !lv_obj_is_valid(kc))
        return;
    s_kbd_ui_built = true; /* 先立旗:建構過程若間接再進來,不會遞迴重建 */
    extern void clock_main_heap_log(const char *tag);
    clock_main_heap_log("kbd-build:before");
    create_keyboard_mode_ui(kc);
    clock_main_heap_log("kbd-build:after");
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
    lv_obj_add_event_cb(parent, voice_tap_to_kbd_cb, LV_EVENT_SHORT_CLICKED,
                        NULL);
    lv_obj_add_event_cb(parent, keyboard_mode_container_hit_test_cb,
                        LV_EVENT_HIT_TEST, NULL);

    // Input bar 容器（深色框，keyboard mode 顯示在鍵盤上方）
    /* 輸入頁的 mouse_mode_icon =「點我回滑鼠」,與觸控板那顆鍵盤圖成對。
       2026-08-07 founder:鍵盤圖搬到底部中間後,這顆也一起搬,兩顆同一個位置
       進出對齊。位置壓在 row4(mode/space/del,絕對 y≈369 起)**之下** —
       鍵盤容器是 466×300 貼底(y166..466),row4 底緣約 y428,所以鈕做 80×40
       貼底(y≈424..464)剛好避開 space 鍵的觸控範圍。
       掛跨 mode 層,預設藏;顯藏由 mode_set_visible(KEYBOARD)+kbd_mode_extras_sync。 */
    kbd_exit_btn = lv_obj_create(parent);
    lv_obj_remove_style_all(kbd_exit_btn);
    lv_obj_set_size(kbd_exit_btn, 80, 40);
    lv_obj_align(kbd_exit_btn, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_add_flag(kbd_exit_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(kbd_exit_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(kbd_exit_btn, kbd_exit_btn_event_cb, LV_EVENT_CLICKED,
                        NULL);
    lv_obj_t *kbd_exit_img = lv_img_create(kbd_exit_btn);
    lv_img_set_src(kbd_exit_img, &mouse_mode_icon);
    lv_img_set_zoom(kbd_exit_img, 150);
    lv_obj_center(kbd_exit_img);
    lv_obj_clear_flag(kbd_exit_img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(kbd_exit_btn, LV_OBJ_FLAG_HIDDEN);

    /* 中文選字列(founder 2026-07-22):輸入框正下方(bar y64 h45→列 y117),
       pill 同款樣式;左端拼音+最多 5 候選(透明白字鈕)。組字時才現形。 */
    /* 開放式選字列(founder 2026-07-22:pill 圓角會被捲動內容穿出,改「上下
       各一條橫線」、無左右框;矩形邊界自然裁切,捲動不再有超框問題)。 */
    s_kbd_cand_row = lv_obj_create(parent);
    lv_obj_remove_style_all(s_kbd_cand_row);
    /* 全寬到邊緣(founder 2026-07-22):上下線直通圓緣;內容用左右 pad 避開
       圓弧裁切(該高度弦緣 x≈31)。 */
    lv_obj_set_size(s_kbd_cand_row, LV_HOR_RES, 45);
    lv_obj_set_pos(s_kbd_cand_row, 0, 117);
    lv_obj_set_style_bg_color(s_kbd_cand_row, lv_color_hex(0x1a1a1a),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_kbd_cand_row, LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_kbd_cand_row, lv_color_hex(0xFFFFFF),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(s_kbd_cand_row, 2, LV_PART_MAIN);
    lv_obj_set_style_border_opa(s_kbd_cand_row, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_border_side(s_kbd_cand_row,
                                 LV_BORDER_SIDE_TOP | LV_BORDER_SIDE_BOTTOM,
                                 LV_PART_MAIN);
    /* 可左右滑看更多候選:水平捲動、藏滾動條;按著候選字拖也會滑,點一下仍是選字。 */
    lv_obj_add_flag(s_kbd_cand_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(s_kbd_cand_row, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(s_kbd_cand_row, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(s_kbd_cand_row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_flex_flow(s_kbd_cand_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_kbd_cand_row, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_left(s_kbd_cand_row, 36, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_kbd_cand_row, 36, LV_PART_MAIN);
    lv_obj_set_style_pad_column(s_kbd_cand_row, 6, LV_PART_MAIN);
    lv_obj_add_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
    s_kbd_py_lbl = lv_label_create(s_kbd_cand_row);
    lv_obj_set_style_text_color(s_kbd_py_lbl, lv_color_hex(0x8a8a8a), 0);
    /* 拼音與候選之間的淡分隔線 */
    lv_obj_set_style_pad_right(s_kbd_py_lbl, 8, 0);
    lv_obj_set_style_border_color(s_kbd_py_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(s_kbd_py_lbl, 1, 0);
    lv_obj_set_style_border_opa(s_kbd_py_lbl, LV_OPA_20, 0);
    lv_obj_set_style_border_side(s_kbd_py_lbl, LV_BORDER_SIDE_RIGHT, 0);
    lv_label_set_text(s_kbd_py_lbl, "");
    for (int ci = 0; ci < KBD_CAND_MAX; ci++)
    {
        s_kbd_cand_btns[ci] = lv_obj_create(s_kbd_cand_row);
        lv_obj_set_size(s_kbd_cand_btns[ci], 42, 39);
        lv_obj_set_style_bg_opa(s_kbd_cand_btns[ci], LV_OPA_TRANSP,
                                LV_PART_MAIN);
        /* 選項之間淡淡的分隔線(founder 2026-07-22):每顆右緣 1px 白 20% */
        lv_obj_set_style_border_color(s_kbd_cand_btns[ci],
                                      lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_border_width(s_kbd_cand_btns[ci], 1, LV_PART_MAIN);
        lv_obj_set_style_border_opa(s_kbd_cand_btns[ci], LV_OPA_20,
                                    LV_PART_MAIN);
        lv_obj_set_style_border_side(s_kbd_cand_btns[ci], LV_BORDER_SIDE_RIGHT,
                                     LV_PART_MAIN);
        lv_obj_clear_flag(s_kbd_cand_btns[ci], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(s_kbd_cand_btns[ci], kbd_cand_btn_cb,
                            LV_EVENT_CLICKED, (void *)(intptr_t)ci);
        s_kbd_cand_lbls[ci] = lv_label_create(s_kbd_cand_btns[ci]);
        lv_obj_set_style_text_color(s_kbd_cand_lbls[ci],
                                    lv_color_hex(0xFFFFFF), 0);
        lv_label_set_text(s_kbd_cand_lbls[ci], "");
        lv_obj_center(s_kbd_cand_lbls[ci]);
        lv_obj_add_flag(s_kbd_cand_btns[ci], LV_OBJ_FLAG_HIDDEN);
    }

    text_input_bar_bg = lv_obj_create(parent);
    // 高度 45；頂部對齊 y=110（466 圓內輸入列最高位置）
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
    /* 語音站:同一個框上的點/長按/拖 → 移游標 / 定位並錄音 / 框選。內部自己看
       s_voice_box_on,鍵盤站是 no-op(那時這個框的手勢是「往下拖收回」)。 */
    lv_obj_add_event_cb(text_input_bar_bg, voice_box_gesture_cb, LV_EVENT_ALL, NULL);

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
    /* 語音站框選的反白樣式。不設的話吃 LVGL 預設的**白底**,白字白底根本看不出選了什麼
       (founder 2026-08-03)。與立起面板同一組:淡藍 30%、文字維持白。 */
    lv_obj_set_style_bg_color(input_display_label, lv_color_hex(KBD_MIC_PULSE_COLOR),
                              LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(input_display_label, LV_OPA_30, LV_PART_SELECTED);
    lv_obj_set_style_text_color(input_display_label, lv_color_white(), LV_PART_SELECTED);

    // Blinking cursor
    // parent = input_content_container（跟 label 同一個），不能掛在 text_input_bar_bg：
    //   bar 在 expand anim 期間會從 280×45 長到 300×90，container 跟 label 都用
    //   LEFT_MID align 自動跟 bar 高度走，但 align_to 是 one-shot，cursor 掛在
    //   bar 上會被釘在「創建當下的螢幕位置」不會跟著 label 走 → 偏到頂部
    /* 語音站:游標下方的圓球把手(按住拖 = 帶著游標走)。與 input_cursor 同一個 parent,
       座標才對得上。鍵盤站不顯示。 */
    s_voice_caret_ball = lv_obj_create(input_content_container);
    lv_obj_remove_style_all(s_voice_caret_ball);
    lv_obj_set_size(s_voice_caret_ball, VOICE_BALL_D, VOICE_BALL_D);
    lv_obj_set_style_bg_color(s_voice_caret_ball, lv_color_hex(KBD_MIC_PULSE_COLOR), 0);
    lv_obj_set_style_bg_opa(s_voice_caret_ball, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_voice_caret_ball, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_flag(s_voice_caret_ball, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_voice_caret_ball, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_ext_click_area(s_voice_caret_ball, 14);
    lv_obj_add_event_cb(s_voice_caret_ball, voice_ball_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_voice_caret_ball, voice_ball_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(s_voice_caret_ball, voice_ball_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_voice_caret_ball, voice_ball_event_cb, LV_EVENT_PRESS_LOST, NULL);
    lv_obj_add_flag(s_voice_caret_ball, LV_OBJ_FLAG_HIDDEN);

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

    // 鍵盤 layout（內部創建 keyboard_container, custom_keyboard, 所有按鍵）
    // 語音站進場(s_kbd_build_defer_wheel)不建 —— kbd_lower_switch(true) 的 lazy
    // 分支會在使用者真的切去鍵盤站時補建(heap 見 s_kbd_build_defer_wheel 註解)。
    if (!s_kbd_build_defer_wheel)
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

    /* 螢幕最上方的下拉感應區(founder 2026-08-17:「點畫面最上面往下拉可以把輸入框或
       鍵盤收下來回到顯示選項」)。輸入框上緣在 VOICE_BOX_Y=107(鍵盤站的輸入列更低),
       所以 0..KBD_TOP_PULL_H 這一帶是空的,拿來當手勢起手區不會蓋到任何內容。
       直接複用 text_input_bar_bg 那套已驗過的下拉狀態機(跟手 progress、40px commit
       門檻、收合落點依 s_kbd_from_drawer 決定回抽屜還是回觸控板),不另外刻一套。
       ADV_HITTEST:kbd_guard_overlappers() 每 40ms 會把「壓在輸入作用區上的可點物件」
       通通掛上 chrome_hit_test_cb(按下即放行給底下的鍵),那會讓這個感應區永遠收不到
       press。它的跳過條件正是「已經有 ADV_HITTEST」——設上去即可豁免,而 LVGL 在沒有
       HIT_TEST cb 時 res 預設為 true,命中行為與一般物件相同。
       移到背景:上方的 icon_send / logo(VOICE_ICON_DY,y≈55)落在這一帶,要留在前景。 */
    kbd_top_pull = lv_obj_create(parent);
    lv_obj_remove_style_all(kbd_top_pull);
    lv_obj_set_size(kbd_top_pull, LV_HOR_RES, KBD_TOP_PULL_H);
    lv_obj_align(kbd_top_pull, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(kbd_top_pull, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(kbd_top_pull, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(kbd_top_pull, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(kbd_top_pull, LV_OBJ_FLAG_PRESS_LOCK); /* 手指往下拖出這一區也要續收 PRESSING */
    lv_obj_add_flag(kbd_top_pull, LV_OBJ_FLAG_ADV_HITTEST); /* 見上:豁免 kbd_guard_overlappers */
    lv_obj_add_event_cb(kbd_top_pull, kbd_top_pull_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_move_background(kbd_top_pull);
}

/* 頂部下拉感應區 → 沿用輸入框那條下拉收合。只擋掉 SHORT_CLICKED:那條在原 handler 裡是
   「點字移游標」,對著頂部空白區點一下不該把游標亂跳。 */
static void kbd_top_pull_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_SHORT_CLICKED)
        return;
    text_input_bar_drag_event_cb(e);
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
    lv_obj_set_size(kbd_mic_section_mic_btn, VOICE_BTN_D, VOICE_BTN_D);
    lv_obj_align(kbd_mic_section_mic_btn, LV_ALIGN_CENTER, VOICE_MIC_DX,
                 VOICE_ROW_DY);
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
    /* 按住講話 / 放開停止(founder 2026-08-03:語音站沿用立起面板那套手感,不是 tap 開關)。
       原本這顆是隱藏的(「V2T active 改由 input bar 邊框閃爍提示」),語音站要有實體麥克風
       可按,所以放回來。 */
    lv_obj_add_event_cb(kbd_mic_section_mic_btn, kbd_mic_btn_event_cb,
                        LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(kbd_mic_section_mic_btn, kbd_mic_btn_event_cb,
                        LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(kbd_mic_section_mic_btn, kbd_mic_btn_event_cb,
                        LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(kbd_mic_section_mic_btn, kbd_mic_btn_event_cb,
                        LV_EVENT_PRESS_LOST, NULL);

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
    lv_obj_align(kbd_mic_section_right_arrow, LV_ALIGN_CENTER, VOICE_KBD_DX,
                 VOICE_ROW_DY);
    /* 語音站與鍵盤站的地球鍵是**同一顆功能**(切輸入法),外觀就該一樣(founder
       2026-08-18:「語音輸入的地球按鈕怎麼跟鍵盤那邊不一樣」)。鍵盤輪盤 row4 的
       mode_btn 是 50x50 全透明容器 + 原尺寸(32px)地球圖,沒有底圓;這裡原本多了一圈
       0x333333/60% 的底圓、圖又縮到 zoom 150(=19px),所以看起來是「小地球黏在灰圓上」。
       拿掉底圓、圖回原尺寸,兩站一致。 */
    lv_obj_set_style_bg_opa(kbd_mic_section_right_arrow, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(kbd_mic_section_right_arrow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(kbd_mic_section_right_arrow, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(kbd_mic_section_right_arrow,
                        kbd_lower_arrow_event_cb, LV_EVENT_CLICKED,
                        (void *)(intptr_t)1); // 1 = 切到 keyboard
    lv_obj_t *right_arrow_img = lv_img_create(kbd_mic_section_right_arrow);
    /* founder 2026-08-17:這顆的語意是「切輸入法」,圖統一成地球(erth)——與鍵盤輪盤
       row4 的 mode_btn、以及抽屜三鍵列最左那顆同一張,三處一致。 */
    lv_img_set_src(right_arrow_img, &erth); /* 原尺寸,同鍵盤站的 mode_btn */
    lv_obj_center(right_arrow_img);

    /* ── 語音站的其餘控制項(founder 2026-08-03:把立起面板那一整套搬進鍵盤區) ──
       文字沿用鍵盤模式自己的 input_buffer(單一真相,才能跟注音/英文混著用),所以刪除
       直接走 remove_from_input_buffer(),不必再跟手機來回。 */
    kbd_voice_del_btn = lv_img_create(kbd_mic_section);
    lv_img_set_src(kbd_voice_del_btn, &backspace_icon);
    lv_obj_align(kbd_voice_del_btn, LV_ALIGN_CENTER, VOICE_DEL_DX, VOICE_ROW_DY);
    lv_obj_add_flag(kbd_voice_del_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(kbd_voice_del_btn, 14);
    lv_obj_add_event_cb(kbd_voice_del_btn, kbd_voice_del_event_cb, LV_EVENT_SHORT_CLICKED, NULL);
    lv_obj_add_event_cb(kbd_voice_del_btn, kbd_voice_del_event_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(kbd_voice_del_btn, kbd_voice_del_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(kbd_voice_del_btn, kbd_voice_del_event_cb, LV_EVENT_PRESS_LOST, NULL);

    /* 上方兩顆送出:logo = 當 skaibar 查詢送出(送查詢不自動執行)、
       icon_send = 打進電腦剛剛點的那個輸入框(只有電腦有聚焦欄位時才出現)。 */
    /* 常駐的「送 AI」logo 已移除(founder 2026-08-07):改成長按錄音時在手指上方
       生一個淡的浮動 logo,拖上去變實心、放開才送(見 ai_drag_logo_*)。
       kbd_voice_logo_btn 保留為 NULL,相關 layout / 顯藏都有 NULL 防護。 */
    kbd_voice_logo_btn = NULL;

    /* 送出鍵掛在**輸入框自己**裡面、貼右緣(founder 2026-08-18:「幫我移到輸入框裡面的
       右邊」)。掛 text_input_bar_bg 有三個好處:兩站共用同一個 bar,所以語音站/鍵盤站都在;
       bar 的所有位移(展開/收合動畫、換站的 y 位移)它自動跟著;而它接手的正是原本內嵌在
       這裡的 enter_icon 的位置 —— 那顆已經移除(同一顆按鍵、同一件事)。 */
    kbd_voice_send_btn = lv_img_create(text_input_bar_bg);
    lv_img_set_src(kbd_voice_send_btn, &icon_send);
    /* 縮圖但不縮 bbox。pivot 設在圖正中央,縮放才是往中心收(預設 pivot 也是中心,寫明
       是因為這顆之後若換圖或改 zoom,漏設 pivot 會變成往左上角縮)。 */
    lv_img_set_pivot(kbd_voice_send_btn, SEND_BTN_IMG_W / 2, SEND_BTN_IMG_W / 2);
    lv_img_set_zoom(kbd_voice_send_btn, SEND_BTN_ZOOM);
    lv_obj_add_flag(kbd_voice_send_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(kbd_voice_send_btn, SEND_BTN_EXT_CLICK);
    lv_obj_add_event_cb(kbd_voice_send_btn, kbd_voice_send_event_cb, LV_EVENT_CLICKED,
                        (void *)(intptr_t)1);
    kbd_voice_layout_send_icons();

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
    /* 邊緣地帶不算「點空白要退出」(founder 2026-08-07:點到某些區塊會自動退出)。
       2026-08-07 稍早為了修「下面那排很難按到」,把鍵盤容器 hit-test 的弧帶/中央
       捲動區排除整個關掉了 —— 那些區域因此改成直接命中 mode_container,而這支正是
       掛在它身上的「點空白收回觸控板」,於是邊緣一點就退出。判斷搬到這裡:按鍵維持
       按得到(hit-test 保持寬鬆),邊緣則不再誤觸退出。 */
    {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t p;
        if (indev)
        {
            lv_indev_get_point(indev, &p);
            /* 觸控驅動的錯誤哨兵:ft3168 讀 I2C 失敗時會偽造一個 (0,0) 的放開
               事件餵給 LVGL(見 drv_ft3168 的 "Error, return Up event, x=0, y=0"),
               LVGL 把它當成一次落在 mode_container 上的正常點擊 → 直接收掉輸入
               模式。真機上這是隨機發生的,跟使用者實際點哪裡無關
               (founder 2026-08-07:點麥克風右邊一點就被踢出去;log 實測座標
               全是 (0,0))。圓形螢幕的 (0,0) 在可視圓之外,本來就不可能是真觸控,
               直接無視。 */
            if (p.x == 0 && p.y == 0)
                return;
            if (is_point_in_left_arc(&p) || is_point_in_right_arc(&p) ||
                is_point_in_center_scroll_zone(&p))
                return;
        }
    }
    /* **語音站不吃「退出輸入模式」這條**(founder 2026-08-04):那一站的按鈕都是
       圖示、標的小,想點下方麥克風而稍微點偏就會落到空白處,整個輸入模式當場被
       收掉、剛講的字也沒了。離開語音站要走明確的出口:鍵盤鈕、空框時的退出鍵、
       或送出。
       語音站的「輕點切鍵盤」不在這支 —— 見 voice_tap_to_kbd_cb(綁 SHORT_CLICKED)。 */
    if (s_voice_box_on) return;
    // 還在動畫中（mode_swipe 或 collapse）不重複觸發
    //   注意：current_hid_mode 在 collapse done_cb 才會從 KEYBOARD 翻成
    //   TRACKPAD，動畫期間單看 current_hid_mode 擋不住，必須另外看 anim flag
    if (current_hid_mode != HID_MODE_KEYBOARD) return;
    if (collapse_anim_running) return;
    // V2T 還在錄就先停掉再 collapse（跟 down_arrow 路徑一致）
    if (mouse_v2t_active)
    {
        mouse_v2t_close_and_paste();
    }
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

/* 退出動畫:框從**當下的位置**一路收到底部那條指示帶,中途不繞路。
   先前是先補間到收合動畫的起點(mic view 380x90@195,螢幕正中)再交棒給既有的收合 ——
   從鍵盤站(y=75)退出時,框會先往**下走到中間**再開始收,看起來像走了兩段
   (founder 2026-08-18:「為什麼退出動畫會先看到輸入框移動到正中間再往下收」)。
   既有那段收合是為「往下拖」寫的,它的起點固定在 mic view;退出鍵/送出的起點是任意的,
   所以這裡自己開一段:框的幾何+圓角,加上下方元件(mic 區 / 鍵盤)一起往下退場。 */
#define EXIT_ANIM_TIME_MS 240
/* 下方元件退場的位移量。與展開動畫的 EXPAND_TRANSLATE_START 同值(那個 define 在
   檔案更下面的展開動畫段,這裡先用同一個數字,兩者要一起改)。 */
#define EXIT_TRANSLATE_END 320
static lv_coord_t s_exit_ty_end = 0; /* 下方元件退場的位移終點 */

static void kbd_exit_morph_cb(void *var, int32_t v)
{
    lv_obj_t *bar = (lv_obj_t *)var;
    if (bar && lv_obj_is_valid(bar))
    {
        lv_obj_set_size(bar,
                        barmorph_lerp(s_barmorph_w0, s_barmorph_w1, v),
                        barmorph_lerp(s_barmorph_h0, s_barmorph_h1, v));
        lv_obj_set_pos(bar,
                       barmorph_lerp(s_barmorph_x0, s_barmorph_x1, v),
                       barmorph_lerp(s_barmorph_y0, s_barmorph_y1, v));
        lv_obj_set_style_radius(bar,
                                barmorph_lerp(s_barmorph_r0, s_barmorph_r1, v),
                                LV_PART_MAIN);
    }
    lv_coord_t ty = barmorph_lerp(0, s_exit_ty_end, v);
    if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
        lv_obj_set_style_translate_y(kbd_mic_section, ty, 0);
    if (keyboard_container && lv_obj_is_valid(keyboard_container))
        lv_obj_set_style_translate_y(keyboard_container, ty, 0);
}

/* 淡出**單獨一條動畫**,而且是線性的。掛在收合的進度值上行不通:收合走 ease_out,
   進度在時間的前段就衝掉大半,於是「畫面上還在走」的那段時間裡進度早已接近 100,
   透明度跟著提前歸零 —— 看起來就是還沒收到三分之一就不見了(founder 2026-08-18)。
   線性 + 延遲起跑,透明度才跟著**時間**均勻掉,與眼睛看到的行程一致。 */
/* 淡出要**逐項調各自的 opa**,不能用物件級的 lv_obj_set_style_opa()。
   這版 LVGL 的 lv_obj_init_draw_rect_dsc() 開頭就是
       lv_opa_t opa = LV_OPA_COVER;
       if (part != LV_PART_MAIN) opa = lv_obj_get_style_opa(obj, part);
   —— MAIN 直接跳過,永遠 COVER。LV_STYLE_OPA 在屬性表標的是 LAYER_REFR,要靠「先畫到
   中介圖層再整層混合」才成立,而這塊硬體的繪圖管線對 layer/transform 支援有限(同一類
   坑:FT label 不吃 transform_zoom)。所以先前那行是**靜默無效**,founder 看到的就是
   「收下去但完全沒變透明」。
   text_opa 是可繼承屬性,設在框上就會傳到裡面的 label;圖示類要各自設 img_opa。
   f = 0..255 的淡出係數,各項用自己的基準值去乘,才不會把原本半透明的邊框調成全不透明。 */
static void kbd_exit_fade_apply(lv_obj_t *bar, int32_t f)
{
    if (f < 0) f = 0;
    if (f > LV_OPA_COVER) f = LV_OPA_COVER;
    if (bar && lv_obj_is_valid(bar))
    {
        lv_obj_set_style_bg_opa(bar, (lv_opa_t)(LV_OPA_90 * f / LV_OPA_COVER),
                                LV_PART_MAIN);
        lv_obj_set_style_border_opa(bar, (lv_opa_t)(LV_OPA_50 * f / LV_OPA_COVER),
                                    LV_PART_MAIN);
        lv_obj_set_style_text_opa(bar, (lv_opa_t)f, LV_PART_MAIN); /* 繼承給 label */
    }
    if (input_cursor && lv_obj_is_valid(input_cursor))
        lv_obj_set_style_bg_opa(input_cursor, (lv_opa_t)f, LV_PART_MAIN);
    if (kbd_voice_send_btn && lv_obj_is_valid(kbd_voice_send_btn))
        lv_obj_set_style_img_opa(kbd_voice_send_btn, (lv_opa_t)f, LV_PART_MAIN);
    if (s_bar_voice_frame && lv_obj_is_valid(s_bar_voice_frame))
        lv_obj_set_style_img_opa(s_bar_voice_frame, (lv_opa_t)f, LV_PART_MAIN);
}

static void kbd_exit_opa_cb(void *var, int32_t v)
{
    kbd_exit_fade_apply((lv_obj_t *)var, v);
}

static void kbd_exit_morph_done_cb(lv_anim_t *a)
{
    collapse_anim_running = false;
    /* **透明度一定要還原**:這些是 style 上的值,不還原的話下次開輸入框整條 bar 是隱形的
       (物件在、事件也照收,只是看不見 —— 最難查的那種)。 */
    kbd_exit_fade_apply((lv_obj_t *)a->var, LV_OPA_COVER);
    /* 與往下拖那條收合共用同一支收尾:容器切換、狀態復位都在裡面。 */
    kbd_commit_to_trackpad();
}

/* 「離開輸入模式」的單一出口(founder 2026-08-18:「鍵盤跟語音模式退出的畫面怎麼是瞬間
   消失沒有動畫」)。先前空框退出鍵/送出都是直接 apply_hid_mode(HID_MODE_TRACKPAD) ——
   那只是換容器,沒有任何動畫,畫面當場消失。 */
static void mouse_exit_input_station(void)
{
    if (current_hid_mode != HID_MODE_KEYBOARD)
        return;
    if (collapse_anim_running)
        return; /* 已經在收了,別疊第二段 */
    if (mouse_v2t_active)
        mouse_v2t_close_and_paste();

    bool bar_ok = (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg));
    lv_coord_t x0 = 0, y0 = 0, w0 = 0, h0 = 0;
    if (bar_ok)
    {
        lv_obj_update_layout(text_input_bar_bg);
        x0 = lv_obj_get_x(text_input_bar_bg);
        y0 = lv_obj_get_y(text_input_bar_bg);
        w0 = lv_obj_get_width(text_input_bar_bg);
        h0 = lv_obj_get_height(text_input_bar_bg);
    }
    bool from_voice = s_voice_box_on;
    if (from_voice)
    {
        /* 語音站的外觀是卡片圖,跟不了縮放(見 kbd_bar_morph_to_voice_done_cb);這支
           順便把語音站的狀態(框選/放大鏡/連刪 timer/preview)一併收乾淨。 */
        kbd_bar_set_voice_box(false);
    }
    /* 選字/數字快捷列跟不了框的縮放,直接收掉(同往下拖那條的作法)。 */
    if (s_kbd_cand_row && lv_obj_is_valid(s_kbd_cand_row))
        lv_obj_add_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
    /* 弧線要在收合開始前就回來,否則會看到「框收完之後弧線才冒出來」。 */
    kbd_lower_update_arcs_visibility();
    if (!bar_ok)
    {
        kbd_commit_to_trackpad();
        return;
    }
    /* 終點 = 觸控板上那條收合狀態的指示帶(mode_set_visible 的 closed 幾何)。 */
    s_barmorph_x0 = x0;
    s_barmorph_y0 = y0;
    s_barmorph_w0 = w0;
    s_barmorph_h0 = h0;
    s_barmorph_x1 = (LV_HOR_RES_MAX - 200) / 2;
    s_barmorph_y1 = LV_VER_RES_MAX - 50;
    s_barmorph_w1 = 200;
    s_barmorph_h1 = 50;
    s_barmorph_r0 = from_voice ? VOICE_BOX_RADIUS : 100;
    s_barmorph_r1 = 100;
    /* 鍵盤還在畫面上就讓它跟著往下退場(和框同一個方向、同一段時間);已經是語音站的話
       退場的是 mic 區那排。兩個都掛同一條位移,hidden 的那個沒有視覺影響。 */
    s_exit_ty_end = EXIT_TRANSLATE_END;
    collapse_anim_running = true;
    lv_anim_del(text_input_bar_bg, NULL); /* 換站的 morph / y 動畫都要讓開 */
    kbd_exit_morph_cb(text_input_bar_bg, 0);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, text_input_bar_bg);
    lv_anim_set_exec_cb(&a, kbd_exit_morph_cb);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, EXIT_ANIM_TIME_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, kbd_exit_morph_done_cb);
    lv_anim_start(&a);
    /* 淡出:前 40% 的時間維持不透明(先看清楚它在往下走),之後線性淡到 0,與收合同時落地。 */
    lv_anim_t fa;
    lv_anim_init(&fa);
    lv_anim_set_var(&fa, text_input_bar_bg);
    lv_anim_set_exec_cb(&fa, kbd_exit_opa_cb);
    lv_anim_set_values(&fa, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_delay(&fa, EXIT_ANIM_TIME_MS * 2 / 5);
    lv_anim_set_time(&fa, EXIT_ANIM_TIME_MS * 3 / 5);
    lv_anim_set_path_cb(&fa, lv_anim_path_linear);
    lv_anim_start(&fa);
}

/* 鍵盤 ⇄ 語音(mic)兩站的切換本體。**輸入列的高度是靠這裡的動畫換的**,只呼
   kbd_lower_set_keyboard() 切可見性會讓輸入列停在另一站的高度 —— 所以 Mode 鍵切到
   語音那站也要走這條,不能走捷徑。原本只有箭頭 cb 用,2026-08-03 抽出來共用。 */
/* 語音站要大框、鍵盤站維持藥丸(founder 2026-08-03:「跟之前點下面圖片進去的輸入框不一樣」)。
   **離開鍵盤站時把當下幾何記下來、回來再放回去** —— 這條 bar 的尺寸是 expand 動畫 /
   mode_set_visible 依情境設的,硬寫回一組數字就是 founder 2026-07-22「下層多一個輸入框」
   的重演。快照為 0 = 還沒離開過鍵盤站。 */
/* 鍵盤站的輸入列幾何 —— 與 mode_set_visible(KEYBOARD,true) 那段同一組值。 */
#define KBD_BAR_W 310
#define KBD_BAR_H 45
/* mode_set_visible(KEYBOARD,true) 會給這條 bar 白色 2px 邊框(OPA_50)。語音站只關底色不夠 ——
   邊框照樣把 442×252 畫成一個圓角外框,疊在圖片框上變成兩個框(founder 2026-08-03 一眼看出
   「一個是圖片的一個是你自己畫的」)。邊框也要一起收,回鍵盤站再放回去。 */
/* 語音站的框是**圖片**(message_widget_bg),不是畫出來的圓角矩形 —— founder 2026-08-03
   一眼認出來「我記得之前是用圖片」。鍵盤站仍是白色藥丸,所以圖片只在語音站顯示,
   同時把 bar 自己的底色關掉,不然白藥丸會透在圖片後面。 */
static lv_obj_t *s_bar_voice_frame = NULL;
#define VOICE_BOX_W 442  /* == 立起面板的 message_widget_bg 原生尺寸 */
#define VOICE_BOX_H 252
/* 大框置中(466 圓內上下各餘 107)。語音站的 bar 目標高度用它,不是 EXPAND_END_Y(195)——
   195 起算 252 高會一路蓋到 447,把下方那排按鈕整個壓在底下(sim 2026-08-03 第一版)。 */
#define VOICE_BOX_Y ((LV_VER_RES_MAX - VOICE_BOX_H) / 2)

/* 錄音中沿大框外圈擴散的藍色脈衝。原本 V2T 的提示是「輸入列**邊框**閃爍」,但語音站為了
   不跟圖片框重疊已經把邊框關掉,提示等於被一起關掉(founder 2026-08-03:「按著麥克風時外框
   沒有藍色的動畫」)。改成與立起面板同一套:同尺寸圓角矩形從框緣往外擴散漸淡、循環。 */
#define VOICE_GLOW_GROW 40
#define VOICE_GLOW_RADIUS 40
#define VOICE_GLOW_PERIOD_MS 1200
static lv_obj_t *s_voice_glow = NULL;

static void voice_glow_anim_cb(void *var, int32_t v)
{
    lv_obj_t *ring = (lv_obj_t *)var;
    if (!ring || !lv_obj_is_valid(ring))
        return;
    lv_coord_t g = (lv_coord_t)(VOICE_GLOW_GROW * v / 256);
    lv_obj_set_size(ring, VOICE_BOX_W + g * 2, VOICE_BOX_H + g * 2);
    lv_obj_set_pos(ring, (LV_HOR_RES_MAX - VOICE_BOX_W) / 2 - g, VOICE_BOX_Y - g);
    lv_obj_set_style_radius(ring, VOICE_GLOW_RADIUS + g, 0);
    lv_obj_set_style_border_opa(ring, LV_OPA_COVER - (LV_OPA_COVER * v / 256), 0);
}

static void voice_glow_set_active(bool on)
{
    if (!on)
    {
        if (s_voice_glow && lv_obj_is_valid(s_voice_glow))
        {
            lv_anim_del(s_voice_glow, voice_glow_anim_cb);
            lv_obj_add_flag(s_voice_glow, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }
    if (!s_voice_box_on) /* 只有語音站才有這個框可以圍 */
        return;
    if (!text_input_bar_bg || !lv_obj_is_valid(text_input_bar_bg))
        return;
    if (!s_voice_glow || !lv_obj_is_valid(s_voice_glow))
    {
        s_voice_glow = lv_obj_create(lv_obj_get_parent(text_input_bar_bg));
        lv_obj_remove_style_all(s_voice_glow);
        lv_obj_set_style_border_color(s_voice_glow, lv_color_hex(KBD_MIC_PULSE_COLOR), 0);
        lv_obj_set_style_border_width(s_voice_glow, 3, 0);
        lv_obj_set_style_bg_opa(s_voice_glow, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(s_voice_glow, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(s_voice_glow, LV_OBJ_FLAG_SCROLLABLE);
    }
    lv_obj_clear_flag(s_voice_glow, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_background(s_voice_glow); /* 壓在框與文字下面 */
    lv_anim_del(s_voice_glow, voice_glow_anim_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_voice_glow);
    lv_anim_set_values(&a, 0, 256);
    lv_anim_set_time(&a, VOICE_GLOW_PERIOD_MS);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, voice_glow_anim_cb);
    lv_anim_start(&a);
}

/* 把手錶當下的文字推給電腦那條輸入框。語音站的文字真相在本地,不推的話電腦是空的
   (founder 2026-08-03:「電腦上出現的輸入框裡面沒有同步我手表上的文字」)。
   **防抖 250ms** —— 串流轉錄與連續刪除每次都會改字,逐次發 BLE 會塞爆連線。 */
#define VOICE_PREVIEW_DEBOUNCE_MS 250
static lv_timer_t *s_voice_preview_timer = NULL;

static void voice_preview_fire_cb(lv_timer_t *t)
{
    (void)t;
    s_voice_preview_timer = NULL; /* one-shot */
    extern bool commu_send_voice_station_preview(const char *text);
    commu_send_voice_station_preview(input_buffer);
}

static void voice_preview_schedule(void)
{
    if (s_voice_preview_timer)
        lv_timer_reset(s_voice_preview_timer); /* 還在打字就往後延 */
    else
    {
        s_voice_preview_timer =
            lv_timer_create(voice_preview_fire_cb, VOICE_PREVIEW_DEBOUNCE_MS, NULL);
        lv_timer_set_repeat_count(s_voice_preview_timer, 1);
    }
}

/* ── 長按放大鏡(founder 2026-08-03:「在按下的位置上方一小段距離加一個放大鏡效果讓我知道
   我按到哪裡」)──
   **不能用 transform_zoom 做** —— FreeType label 套 zoom 會直接消失(既有教訓)。改成用大一級
   的字型重畫插入點前後幾個字,並在裡面畫一條游標線,等效於放大那一小段。掛在 lv_layer_top()
   上,不會被輸入框或鍵盤裁切。 */
#define VOICE_LOUPE_W 170
#define VOICE_LOUPE_H 66
#define VOICE_LOUPE_LIFT 30 /* 浮在「那一行字」上方多遠(founder 2026-08-04:再往上一點) */
#define VOICE_LOUPE_CTX 4   /* 插入點前後各顯示幾個字 */
/* 466 圓螢幕在高度 y 的可用半寬 = sqrt(R² - (y-R)²)。放大鏡靠上/下時方形邊界夾不住,
   會凸出圓外(founder 2026-08-04:「超過圓形錶盤就不要繼續左右動,卡在邊緣就好」)。
   自己算整數平方根,不拉進浮點數學。 */
static lv_coord_t circ_half_width_at(lv_coord_t y)
{
    const int32_t R = LV_HOR_RES_MAX / 2;
    int32_t dy = y - R;
    if (dy < 0) dy = -dy;
    if (dy >= R) return 0;
    uint32_t v = (uint32_t)(R * R - dy * dy);
    uint32_t r = 0, b = 256;
    while (b)
    {
        uint32_t t = r + b;
        if (t * t <= v) r = t;
        b >>= 1;
    }
    return (lv_coord_t)r;
}

static lv_obj_t *s_voice_loupe = NULL;
static lv_obj_t *s_voice_loupe_lbl = NULL;
static lv_obj_t *s_voice_loupe_caret = NULL;

static void voice_loupe_hide(void)
{
    if (s_voice_loupe && lv_obj_is_valid(s_voice_loupe))
        lv_obj_add_flag(s_voice_loupe, LV_OBJ_FLAG_HIDDEN);
}

/* 位置:**水平跟著手指、垂直鎖在插入點那一行的上方**(iOS 文字放大鏡的做法)。
   founder 2026-08-03 兩次修正:先是「不要跟著手指飄」→ 我把水平也鎖成置中,
   再被指出「沒有跟著我手的位置左右移動」—— 要鎖的只有**垂直**(它該待在那一行上方,
   不是黏在指尖),水平仍要跟手,否則看不出在放大哪一段。 */
static void voice_loupe_show(lv_point_t touch, int caret_cp)
{
    if (!s_voice_loupe || !lv_obj_is_valid(s_voice_loupe))
    {
        s_voice_loupe = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(s_voice_loupe);
        lv_obj_set_size(s_voice_loupe, VOICE_LOUPE_W, VOICE_LOUPE_H);
        lv_obj_set_style_bg_color(s_voice_loupe, lv_color_hex(0x101010), 0);
        lv_obj_set_style_bg_opa(s_voice_loupe, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(s_voice_loupe, 16, 0);
        lv_obj_set_style_border_color(s_voice_loupe, lv_color_hex(KBD_MIC_PULSE_COLOR), 0);
        lv_obj_set_style_border_width(s_voice_loupe, 2, 0);
        lv_obj_set_style_border_opa(s_voice_loupe, LV_OPA_70, 0);
        lv_obj_clear_flag(s_voice_loupe, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(s_voice_loupe, LV_OBJ_FLAG_SCROLLABLE);

        s_voice_loupe_lbl = lv_label_create(s_voice_loupe);
        lv_obj_set_style_text_color(s_voice_loupe_lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(s_voice_loupe_lbl,
                                   LV_EXT_FONT_GET(get_system_font_size(2)), 0);
        lv_label_set_long_mode(s_voice_loupe_lbl, LV_LABEL_LONG_CLIP);
        lv_obj_align(s_voice_loupe_lbl, LV_ALIGN_LEFT_MID, 8, 0);

        s_voice_loupe_caret = lv_obj_create(s_voice_loupe);
        lv_obj_remove_style_all(s_voice_loupe_caret);
        lv_obj_set_style_bg_color(s_voice_loupe_caret,
                                  lv_color_hex(KBD_MIC_PULSE_COLOR), 0);
        lv_obj_set_style_bg_opa(s_voice_loupe_caret, LV_OPA_COVER, 0);
        lv_obj_clear_flag(s_voice_loupe_caret, LV_OBJ_FLAG_CLICKABLE);
    }

    /* 取插入點前後各 VOICE_LOUPE_CTX 個字當放大內容。 */
    int total = voice_cp_count(input_buffer, input_length);
    if (caret_cp < 0) caret_cp = total;
    int from = caret_cp - VOICE_LOUPE_CTX; if (from < 0) from = 0;
    int to = caret_cp + VOICE_LOUPE_CTX;   if (to > total) to = total;
    int a = voice_byte_of_cp(input_buffer, from);
    int b = voice_byte_of_cp(input_buffer, to);
    char win[64];
    int n = b - a;
    if (n > (int)sizeof(win) - 1) n = (int)sizeof(win) - 1;
    memcpy(win, input_buffer + a, n);
    win[n] = '\0';
    lv_label_set_text(s_voice_loupe_lbl, win);
    lv_obj_update_layout(s_voice_loupe);

    /* 游標線畫在放大內容裡對應的位置。 */
    lv_point_t p;
    lv_label_get_letter_pos(s_voice_loupe_lbl, (uint32_t)(caret_cp - from), &p);
    const lv_font_t *f = lv_obj_get_style_text_font(s_voice_loupe_lbl, LV_PART_MAIN);
    lv_coord_t lh = lv_font_get_line_height(f);
    lv_obj_set_size(s_voice_loupe_caret, 2, lh);
    lv_obj_set_pos(s_voice_loupe_caret, lv_obj_get_x(s_voice_loupe_lbl) + p.x,
                   lv_obj_get_y(s_voice_loupe_lbl) + p.y);

    /* 錨定在插入點那一行的上方:問 label 該字元的座標,換成螢幕座標再往上放。 */
    lv_coord_t line_top = 0, line_h = 24;
    if (input_display_label && lv_obj_is_valid(input_display_label))
    {
        lv_point_t lp;
        lv_label_get_letter_pos(input_display_label, (uint32_t)caret_cp, &lp);
        const lv_font_t *lf =
            lv_obj_get_style_text_font(input_display_label, LV_PART_MAIN);
        line_h = lv_font_get_line_height(lf);
        line_top = input_display_label->coords.y1 + lp.y;
    }
    lv_coord_t y = line_top - VOICE_LOUPE_LIFT - VOICE_LOUPE_H;
    if (y < 6)
        y = line_top + line_h + VOICE_LOUPE_LIFT; /* 那行太靠頂就改放它下方 */
    /* 水平跟手指,但夾在**圓弧**內:取盒子上下緣中離圓心較遠的那一邊算可用半寬,
       兩個角才都在錶盤裡。到邊就停住不再跟。 */
    lv_coord_t x = touch.x - VOICE_LOUPE_W / 2;
    lv_coord_t top_h = circ_half_width_at(y);
    lv_coord_t bot_h = circ_half_width_at(y + VOICE_LOUPE_H);
    lv_coord_t half = (top_h < bot_h) ? top_h : bot_h;
    lv_coord_t cx = LV_HOR_RES_MAX / 2;
    if (half * 2 <= VOICE_LOUPE_W)
        x = cx - VOICE_LOUPE_W / 2; /* 這個高度塞不下 → 置中(至少對稱) */
    else
    {
        if (x < cx - half) x = cx - half;
        if (x > cx + half - VOICE_LOUPE_W) x = cx + half - VOICE_LOUPE_W;
    }
    lv_obj_set_pos(s_voice_loupe, x, y);
    lv_obj_clear_flag(s_voice_loupe, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_voice_loupe);
}

/* 絕對座標 → label 上的第幾個字。 */
static bool voice_letter_at(lv_point_t abs, uint32_t *out)
{
    if (!input_display_label || !lv_obj_is_valid(input_display_label) || input_length == 0)
        return false;
    lv_point_t p = abs;
    p.x -= input_display_label->coords.x1;
    p.y -= input_display_label->coords.y1;
    *out = lv_label_get_letter_on(input_display_label, &p);
    return true;
}

static void voice_set_caret_cp(int cp)
{
    s_voice_caret = voice_byte_of_cp(input_buffer, cp);
    voice_sel_clear();
    update_cursor_position();
}

/* 大框上的三種手勢(與立起面板同一套):
     點一下 = 只移游標;長按不動 = 移游標並開始錄音;長按後拖 = 從按下的字開始框選。
   「講話」與「框選」用 150ms 確認窗分開 —— 一拖就作廢確認窗,不會兩者同時發生。 */
static lv_point_t s_voice_press_pt;
static bool s_voice_press_valid = false;
static bool s_voice_dragging = false;
static int s_voice_sel_anchor_cp = -1;
static lv_timer_t *s_voice_hold_arm = NULL;
#define VOICE_DRAG_SLOP 12
#define VOICE_HOLD_ARM_MS 150

static void voice_hold_arm_stop(void)
{
    if (s_voice_hold_arm) { lv_timer_del(s_voice_hold_arm); s_voice_hold_arm = NULL; }
}

static void ai_drag_logo_hide(void)
{
    if (s_ai_drag_logo && lv_obj_is_valid(s_ai_drag_logo))
        lv_obj_add_flag(s_ai_drag_logo, LV_OBJ_FLAG_HIDDEN);
    if (s_ai_drag_src && lv_obj_is_valid(s_ai_drag_src))
        lv_obj_clear_flag(s_ai_drag_src, LV_OBJ_FLAG_PRESS_LOCK);
    s_ai_drag_src = NULL;
    s_ai_drag_armed = false;
}

static void ai_drag_logo_show(lv_point_t at, lv_obj_t *src)
{
    if (kbd_mic_section == NULL || !lv_obj_is_valid(kbd_mic_section))
        return;
    /* 直打模式沒有「送去 skaibar」這個去處：電腦端此刻是 direct-typing latch，送出去的
       runSkaibar 會把同一段字再打進使用者的文件一次。連浮動 logo 都不出現 = 手勢不會被
       arm（s_ai_drag_armed 恆 false），放開時不會誤送。 */
    if (s_kbd_direct_field)
        return;
    /* 鎖住 press:手指要能離開來源物件往上走,中途不能被判成 PRESS_LOST */
    s_ai_drag_src = src;
    if (src && lv_obj_is_valid(src))
        lv_obj_add_flag(src, LV_OBJ_FLAG_PRESS_LOCK);
    if (s_ai_drag_logo == NULL || !lv_obj_is_valid(s_ai_drag_logo))
    {
        /* 掛在 mode container(= mic section 的父層):跟語音站同一個座標系,
           又不會被 mic section 的換頁位移帶著跑。 */
        s_ai_drag_logo = lv_img_create(lv_obj_get_parent(kbd_mic_section));
        lv_img_set_src(s_ai_drag_logo, &img_logo);
        lv_obj_clear_flag(s_ai_drag_logo, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(s_ai_drag_logo, LV_OBJ_FLAG_SCROLLABLE);
    }
    lv_obj_move_foreground(s_ai_drag_logo);
    lv_obj_update_layout(s_ai_drag_logo);
    lv_coord_t w = lv_obj_get_width(s_ai_drag_logo);
    lv_coord_t h = lv_obj_get_height(s_ai_drag_logo);
    lv_coord_t x = at.x - w / 2;
    lv_coord_t y = at.y - AI_DRAG_LOGO_DY - h / 2;
    if (x < 8) x = 8;
    if (x > LV_HOR_RES - w - 8) x = LV_HOR_RES - w - 8;
    if (y < 8) y = 8;
    lv_obj_set_pos(s_ai_drag_logo, x, y);
    lv_obj_set_style_img_opa(s_ai_drag_logo, LV_OPA_40, 0); /* 淡淡的 */
    lv_obj_clear_flag(s_ai_drag_logo, LV_OBJ_FLAG_HIDDEN);
    s_ai_drag_armed = false;
}

/* 手指是否落在圖示上 → 實心/淡出兩態,並記住 armed 供放開時判斷。 */
static void ai_drag_logo_track(lv_point_t p)
{
    if (s_ai_drag_logo == NULL || !lv_obj_is_valid(s_ai_drag_logo)) return;
    if (lv_obj_has_flag(s_ai_drag_logo, LV_OBJ_FLAG_HIDDEN)) return;
    lv_area_t a;
    lv_obj_get_coords(s_ai_drag_logo, &a);
    lv_coord_t cx = (a.x1 + a.x2) / 2, cy = (a.y1 + a.y2) / 2;
    int32_t dx = p.x - cx, dy = p.y - cy;
    bool hit = (dx * dx + dy * dy) <= (int32_t)AI_DRAG_HIT_R * AI_DRAG_HIT_R;
    if (hit == s_ai_drag_armed) return; /* 狀態沒變就別一直重設樣式 */
    s_ai_drag_armed = hit;
    lv_obj_set_style_img_opa(s_ai_drag_logo, hit ? LV_OPA_COVER : LV_OPA_40, 0);
    if (hit)
    {
        extern void motor_pattern_unlocked(void);
        motor_pattern_unlocked(); /* 進入判定範圍震一下,不用盯著看 */
    }
}

static void voice_hold_arm_cb(lv_timer_t *t)
{
    (void)t;
    s_voice_hold_arm = NULL; /* one-shot */
    if (s_voice_dragging) return;             /* 已變成框選 */
    if (!text_input_bar_bg || !lv_obj_is_valid(text_input_bar_bg)) return;
    if (!lv_obj_has_state(text_input_bar_bg, LV_STATE_PRESSED)) return; /* 已放開 */
    if (!mouse_v2t_active) mouse_v2t_open();
    ai_drag_logo_show(s_voice_press_pt, text_input_bar_bg); /* 按壓點上方生出淡 logo */
}

/* 這一按是不是落在送出鍵(含它的觸控外擴)上? 送出鍵是輸入框的 child,而輸入框自己
   吃 PRESSED/SHORT_CLICKED —— 短按輸入框 = 跳去鍵盤站,所以瞄準送出鍵時稍微偏一點,
   就會變成「跳去鍵盤」而不是送出(founder 2026-08-18:「很容易判斷成點到輸入框」)。
   圖示縮成 24px 之後,看得到的目標比實際熱區小,更容易發生。用座標判定,不依賴 LVGL
   的 hit-test 誰先拿到 press。 */
static bool voice_press_on_send_btn(const lv_point_t *p)
{
    if (!kbd_voice_send_btn || !lv_obj_is_valid(kbd_voice_send_btn)) return false;
    if (lv_obj_has_flag(kbd_voice_send_btn, LV_OBJ_FLAG_HIDDEN)) return false;
    lv_area_t a;
    lv_obj_get_coords(kbd_voice_send_btn, &a);
    lv_coord_t ext = SEND_BTN_EXT_CLICK;
    return (p->x >= a.x1 - ext && p->x <= a.x2 + ext &&
            p->y >= a.y1 - ext && p->y <= a.y2 + ext);
}

static void voice_box_gesture_cb(lv_event_t *e)
{
    if (!s_voice_box_on)
        return; /* 鍵盤站的這條 bar 是「往下拖收回」,不歸這裡管 */
    /* 這一按屬於送出鍵 → 整串事件都不歸輸入框管(latch 到下一次 PRESSED 才重算)。 */
    if (lv_event_get_code(e) == LV_EVENT_PRESSED)
    {
        lv_indev_t *id0 = lv_indev_get_act();
        lv_point_t p0;
        s_voice_press_on_send = false;
        if (id0)
        {
            lv_indev_get_point(id0, &p0);
            s_voice_press_on_send = voice_press_on_send_btn(&p0);
        }
    }
    if (s_voice_press_on_send)
        return;
    switch (lv_event_get_code(e))
    {
    case LV_EVENT_PRESSED:
    {
        lv_indev_t *indev = lv_indev_get_act();
        s_voice_press_valid = (indev != NULL);
        if (indev) lv_indev_get_point(indev, &s_voice_press_pt);
        voice_sel_clear();
        s_voice_dragging = false;
        s_voice_sel_anchor_cp = -1;
        break;
    }
    case LV_EVENT_SHORT_CLICKED:
    {
        uint32_t idx;
        if (s_voice_press_valid && voice_letter_at(s_voice_press_pt, &idx))
            voice_set_caret_cp((int)idx);
        /* 輕點文字區 = 想編輯 → 直接跳到鍵盤(founder 2026-08-07)。游標位置先設好
           再切,鍵盤一出來就停在你點的那個位置。錄音中不切,免得把正在講的拽走。 */
        if (!mouse_v2t_active && !collapse_anim_running)
            kbd_lower_switch(true);
        break;
    }
    case LV_EVENT_LONG_PRESSED:
    {
        uint32_t idx;
        if (s_voice_press_valid && voice_letter_at(s_voice_press_pt, &idx))
        {
            voice_set_caret_cp((int)idx);
            s_voice_sel_anchor_cp = (int)idx;
            voice_loupe_show(s_voice_press_pt, (int)idx); /* 放大鏡:看清楚按到哪 */
        }
        else
        {
            s_voice_sel_anchor_cp = -1; /* 空框:純粹開始講話,沒有字可以圈 */
        }
        voice_hold_arm_stop();
        s_voice_hold_arm = lv_timer_create(voice_hold_arm_cb, VOICE_HOLD_ARM_MS, NULL);
        lv_timer_set_repeat_count(s_voice_hold_arm, 1);
        break;
    }
    case LV_EVENT_PRESSING:
    {
        /* 圖示在的時候先分流:**左右拖 = 圈字**,把圖示收掉、交還給放大鏡那條
           (founder 2026-08-07:「如果有左右拖動就不要出現了只要有放大鏡就好」);
           其餘方向(主要是往上)才算「拖去 AI」,這一輪就不做框選。
           圖示一旦因橫拖收掉,這次按壓就不再回來 —— 意圖已經表明是圈字。 */
        if (s_ai_drag_logo && lv_obj_is_valid(s_ai_drag_logo) &&
            !lv_obj_has_flag(s_ai_drag_logo, LV_OBJ_FLAG_HIDDEN))
        {
            lv_indev_t *ind = lv_indev_get_act();
            if (ind)
            {
                lv_point_t cur2;
                lv_indev_get_point(ind, &cur2);
                lv_coord_t adx = cur2.x - s_voice_press_pt.x;
                lv_coord_t ady = cur2.y - s_voice_press_pt.y;
                if (LV_ABS(adx) > LV_ABS(ady) && LV_ABS(adx) > VOICE_DRAG_SLOP)
                {
                    ai_drag_logo_hide(); /* 橫拖 → 圈字,讓位給放大鏡 */
                }
                else
                {
                    ai_drag_logo_track(cur2);
                    break; /* 這一輪歸「拖去 AI」,不做框選 */
                }
            }
        }
        if (s_voice_sel_anchor_cp < 0) break;
        lv_indev_t *indev = lv_indev_get_act();
        if (!indev) break;
        lv_point_t cur;
        lv_indev_get_point(indev, &cur);
        lv_coord_t dx = cur.x - s_voice_press_pt.x, dy = cur.y - s_voice_press_pt.y;
        if (!s_voice_dragging &&
            (int32_t)dx * dx + (int32_t)dy * dy < VOICE_DRAG_SLOP * VOICE_DRAG_SLOP)
            break;
        if (!s_voice_dragging)
        {
            s_voice_dragging = true;
            voice_hold_arm_stop();                 /* 這是框選不是講話 */
            if (mouse_v2t_active) mouse_v2t_close_and_paste();
            extern void motor_pattern_unlocked(void);
            motor_pattern_unlocked();
        }
        uint32_t idx;
        if (voice_letter_at(cur, &idx))
        {
            voice_sel_apply(s_voice_sel_anchor_cp, (int)idx);
            update_cursor_position(); /* 有選取 → 游標/圓球收起 */
            voice_loupe_show(cur, (int)idx); /* 水平跟手、垂直錨在那一行上方 */
        }
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
    {
        voice_hold_arm_stop();
        voice_loupe_hide();
        /* 同麥克風鈕:只有真的放開才送,PRESS_LOST 當取消 */
        bool send_ai = s_ai_drag_armed &&
                       (lv_event_get_code(e) == LV_EVENT_RELEASED);
        ai_drag_logo_hide();
        if (mouse_v2t_active) mouse_v2t_close_and_paste();
        s_voice_dragging = false;
        s_voice_sel_anchor_cp = -1;
        if (send_ai)
            voice_do_send(0);
        break;
    }
    default:
        break;
    }
}

/* 圓球把手:按住拖 = 帶著游標走(與框選分在不同物件上,不必猜意圖)。 */
static lv_coord_t s_voice_ball_grab_dy = 0;
static bool s_voice_ball_dragging = false;

static void voice_ball_event_cb(lv_event_t *e)
{
    switch (lv_event_get_code(e))
    {
    case LV_EVENT_PRESSED:
        s_voice_ball_dragging = true;
        if (input_cursor && lv_obj_is_valid(input_cursor) &&
            s_voice_caret_ball && lv_obj_is_valid(s_voice_caret_ball))
        {
            /* 球心與游標中線的距離**按下當刻量**(球被夾回框內時距離會變小,寫死會算到隔壁行)。 */
            s_voice_ball_grab_dy =
                (s_voice_caret_ball->coords.y1 + s_voice_caret_ball->coords.y2) / 2 -
                (input_cursor->coords.y1 + input_cursor->coords.y2) / 2;
        }
        {
            extern void motor_pattern_unlocked(void);
            motor_pattern_unlocked();
        }
        break;
    case LV_EVENT_PRESSING:
    {
        if (!s_voice_ball_dragging) break;
        lv_indev_t *indev = lv_indev_get_act();
        if (!indev) break;
        lv_point_t cur;
        lv_indev_get_point(indev, &cur);
        lv_point_t finger = cur;       /* 放大鏡的水平位置用**手指**的 x */
        cur.y -= s_voice_ball_grab_dy; /* 手指抓的是球,游標在它上方 */
        uint32_t idx;
        if (voice_letter_at(cur, &idx))
        {
            voice_set_caret_cp((int)idx);
            voice_loupe_show(finger, (int)idx);
        }
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        s_voice_ball_dragging = false;
        voice_loupe_hide();
        break;
    default:
        break;
    }
}

static void kbd_bar_set_voice_box(bool voice)
{
    s_voice_box_on = voice;
    if (!voice)
    {
        voice_sel_clear();
        voice_hold_arm_stop();
        kbd_voice_del_stop_repeat(); /* 離站別留著連續刪除的 timer */
        voice_loupe_hide();          /* 同理:別把放大鏡留在畫面上 */
        if (s_voice_preview_timer) /* 離站別留著待發的 preview */
        {
            lv_timer_del(s_voice_preview_timer);
            s_voice_preview_timer = NULL;
        }
        /* 2026-08-07 founder:在語音站點好游標再切到鍵盤,游標不能跳回最後 ——
           原本這裡無條件歸零(-1 = 末端),等於把剛選好的插入點丟掉。游標留著,
           讓鍵盤接著在那個位置打字;真正該清的地方是送出/清空(voice_do_send)。
           只收起圓球把手(那是語音站專用的視覺)。 */
        if (s_voice_caret_ball && lv_obj_is_valid(s_voice_caret_ball))
            lv_obj_add_flag(s_voice_caret_ball, LV_OBJ_FLAG_HIDDEN);
    }
    if (!voice)
        voice_glow_set_active(false); /* 離站順手收掉,別留一圈在鍵盤上 */
    /* 底部的 keyboard_icon:語音站要讓位。離站時**只有回到觸控板才放回來** ——
       2026-08-07 founder:進輸入畫面後鍵盤圖與滑鼠圖都不要出現。原本無條件
       clear_flag,離開語音站回到鍵盤時它就冒回鍵盤下緣。 */
    if (s_top_logo && lv_obj_is_valid(s_top_logo))
    {
        if (voice || current_hid_mode == HID_MODE_KEYBOARD)
            lv_obj_add_flag(s_top_logo, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(s_top_logo, LV_OBJ_FLAG_HIDDEN);
    }
    if (!text_input_bar_bg || !lv_obj_is_valid(text_input_bar_bg))
        return;
    if (voice)
    {
        lv_obj_set_size(text_input_bar_bg, VOICE_BOX_W, VOICE_BOX_H);
        lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(text_input_bar_bg, 0, LV_PART_MAIN);
        lv_obj_set_x(text_input_bar_bg, (LV_HOR_RES_MAX - VOICE_BOX_W) / 2);
        /* 底圖:延遲建立(第一次進語音站才建),建好後移到最底層,壓在文字/游標下面。 */
        if (!s_bar_voice_frame || !lv_obj_is_valid(s_bar_voice_frame))
        {
            s_bar_voice_frame = lv_img_create(text_input_bar_bg);
            lv_img_set_src(s_bar_voice_frame, &message_widget_bg);
            lv_obj_clear_flag(s_bar_voice_frame, LV_OBJ_FLAG_CLICKABLE);
        }
        lv_obj_center(s_bar_voice_frame);
        lv_obj_clear_flag(s_bar_voice_frame, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_background(s_bar_voice_frame);
        if (input_content_container && lv_obj_is_valid(input_content_container))
        {
            /* 送出鍵現在坐在框內右緣,文字欄要讓出那一塊,否則長行會從圖示底下穿過去
               (框 442 寬、內容欄置中 360 → 右邊只剩 41px 邊距,不夠讓一顆圖示)。 */
            extern bool instruction_list_remote_target_has_focus(void);
            lv_coord_t cw = instruction_list_remote_target_has_focus() ? 312 : 360;
            lv_obj_set_size(input_content_container, cw, VOICE_BOX_H - 40);
            lv_obj_align(input_content_container, LV_ALIGN_TOP_MID, 0, 10);
        }
        if (input_display_label && lv_obj_is_valid(input_display_label))
        {
            lv_obj_set_width(input_display_label, 360);
            lv_label_set_long_mode(input_display_label, LV_LABEL_LONG_WRAP);
            lv_obj_align(input_display_label, LV_ALIGN_TOP_LEFT, 0, 0);
        }
        /* 大框裡不放 Enter —— 送出走上方 logo / icon_send。 */
        /* 進站要重畫一次:游標從單行的 OUT_RIGHT_MID 換成折行座標,圓球也要現形
           (sim 2026-08-03:帶著字進站時球不出現,因為沒有任何東西觸發重繪)。 */
        lv_obj_update_layout(text_input_bar_bg);
        update_cursor_position();
        /* **通知手機/電腦**:進語音站 = summonSkaibar{inputOnly:true}。電腦收到才會跳出
           純輸入框(不出選項),並把召喚前聚焦的那個欄位記成 icon_send 的目的地。
           founder 2026-08-03:「電腦這怎麼沒有出現輸入框」—— 我先前只做了手錶端的 UI,
           這條完全沒接。兩個入口(頂部圖示 / Mode 鍵)都會經過這裡。 */
        {
            extern bool commu_send_skaibar_open_device_ex(bool force_open, bool input_only);
            /* inputOnly:一般進語音站=true(電腦只留輸入框,並把召喚前聚焦的欄位記成
               icon_send 的目的地)。但**從 session 抽屜進來**時要的正好相反 —— 講的話
               是拿去搜這台電腦的 sessions/actions/檔案,電腦必須把選項算出來並鏡像回
               手錶(0x03),所以 inputOnly=false(founder 2026-08-17:「為什麼上面沒有
               選項?兩邊都沒有」)。 */
            /* 直打模式：forceOpen=false 讓電腦「有聚焦輸入框就讓給它」（桌面
               SkaibarRelayController.Summon 的 TryBegin 分支）—— 面板整個不開，之後每一次
               preview 都直接落在使用者剛剛點的那個欄位。萬一召喚抵達時焦點已經沒了，
               TryBegin 失敗、桌面自動退回開一般面板，不會變成「打了字卻不知道去哪」。 */
            if (s_kbd_direct_field)
                commu_send_skaibar_open_device_ex(false, false);
            else
                commu_send_skaibar_open_device_ex(true, !s_kbd_from_drawer);
            /* 同時把 instruction_list 的單設備 session 建起來(只建 layout + 記 device_id,
               不開任何面板)。按 logo 送出後要用它叫清單 —— 沒有這一步 open_browse() 會因為
               layout 不存在直接 return,清單靜默不出現。 */
            extern bool instruction_list_prepare_single_device(const char *device_id);
            instruction_list_prepare_single_device(s_dev_active_id);
        }
        kbd_voice_layout_send_icons(); /* 依電腦當下的聚焦狀態決定 icon_send 出不出現 */
        kbd_voice_del_update_icon();   /* 空框開場 = 退出鍵 */
        voice_preview_schedule();      /* 進站就把現有文字同步過去(可能是鍵盤打的) */
    }
    else
    {
        /* 還原成鍵盤站那組**固定值**,不用快照 —— 快照抓到的可能是別的狀態(mic-view 的
           380×90、建立時的 280×45、或收合的 200×50),還原就把鍵盤的輸入框弄短了
           (founder 2026-08-03:「我剛剛切過去看到她變短了」)。這組數字與
           mode_set_visible(KEYBOARD,true) 那段一致 —— 那裡是鍵盤站幾何的來源。 */
        lv_obj_set_size(text_input_bar_bg, KBD_BAR_W, KBD_BAR_H);
        lv_obj_set_style_radius(text_input_bar_bg, 100, LV_PART_MAIN);
        lv_obj_set_style_bg_color(text_input_bar_bg, lv_color_hex(0x1a1a1a), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_90, LV_PART_MAIN);
        lv_obj_set_style_border_color(text_input_bar_bg, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_border_width(text_input_bar_bg, 2, LV_PART_MAIN);
        lv_obj_set_style_border_opa(text_input_bar_bg, LV_OPA_50, LV_PART_MAIN);
        lv_obj_set_x(text_input_bar_bg, (LV_HOR_RES_MAX - KBD_BAR_W) / 2);
        if (s_bar_voice_frame && lv_obj_is_valid(s_bar_voice_frame))
            lv_obj_add_flag(s_bar_voice_frame, LV_OBJ_FLAG_HIDDEN);
        if (input_content_container && lv_obj_is_valid(input_content_container))
        {
            lv_obj_set_size(input_content_container, 210, 40);
            lv_obj_align(input_content_container, LV_ALIGN_LEFT_MID, 5, 0);
        }
        if (input_display_label && lv_obj_is_valid(input_display_label))
        {
            lv_obj_set_width(input_display_label, LV_SIZE_CONTENT);
            lv_label_set_long_mode(input_display_label, LV_LABEL_LONG_CLIP);
            lv_obj_align(input_display_label, LV_ALIGN_LEFT_MID, 10, 0);
        }
    }
}

/* 切到鍵盤站之後，下一輪才把畫面捲到插入點（版面需要一輪才從折行
   大框回到單行）。 */
static void kbd_scroll_to_caret_async(void *unused)
{
    (void)unused;
    update_input_display();

}

static void kbd_lower_switch(bool to_kbd)
{
    /* 2026-08-16 heap 根治輪之後 gate 移除:hosted 進場 free 常態 ~80K,輪盤建置
       (~21K)不再有 OOM 風險;輸入法切換的低 heap 防護改在 Mode 鍵的
       「free<60K 免並存動畫」那裡(降級而非拒絕)。 */
    /* 換站前先記住輸入框的幾何 —— kbd_bar_set_voice_box() 會把新站的尺寸/位置**瞬間**
       設好,記下舊的才有辦法在下面補間回去(見 kbd_bar_morph_cb)。 */
    bool morph_bar = (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg) &&
                      !lv_obj_has_flag(text_input_bar_bg, LV_OBJ_FLAG_HIDDEN));
    if (morph_bar)
    {
        lv_obj_update_layout(text_input_bar_bg);
        s_barmorph_x0 = lv_obj_get_x(text_input_bar_bg);
        s_barmorph_y0 = lv_obj_get_y(text_input_bar_bg);
        s_barmorph_w0 = lv_obj_get_width(text_input_bar_bg);
        s_barmorph_h0 = lv_obj_get_height(text_input_bar_bg);
    }
    kbd_bar_set_voice_box(!to_kbd);
    if (to_kbd && mouse_v2t_active)
    {
        mouse_v2t_close_and_paste();
    }
    if (!to_kbd)
    {
        /* 離開鍵盤站:把還掛在畫面上的按鍵提示框與按下狀態收乾淨。上面那道 gate 擋的是
           轉場期間**新的**提示框,這裡收的是進轉場前那一顆(長按空白鍵時它可能正亮著)。 */
        hide_key_popup();
        if (currently_pressed_btn != NULL)
        {
            lv_obj_clear_state(currently_pressed_btn, LV_STATE_PRESSED);
            currently_pressed_btn = NULL;
        }
        closest_btn = NULL;
    }
    /* 送出鍵的 y 是從 bar 高度算出來的,而兩站的 bar 高度不同(442x252 / 310x45) ——
       換站後重算,否則它會停在上一站的高度上。 */
    kbd_voice_layout_send_icons();

    if (to_kbd)
    {
        /* 語音站輕量進場(s_kbd_build_defer_wheel)沒建圓形鍵盤 —— 使用者現在真的要切
           鍵盤站,在這裡補建(建在鍵盤 mode container 上,同 create_keyboard_mode_ui
           的 parent);建完照走下面的升起動畫。 */
        if (keyboard_container == NULL && mode_container[HID_MODE_KEYBOARD] != NULL)
        {
            kbd_pinyin_clear();
            current_keyboard_mode = KEYBOARD_MODE_LETTERS;
            create_circular_keyboard_layout(mode_container[HID_MODE_KEYBOARD]);
            if (keyboard_container != NULL)
            {
                lv_obj_set_style_translate_y(keyboard_container, 0, 0);
                lv_obj_align(keyboard_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
            }
        }
        /* 語音→英文,把循環閉合(founder 2026-08-03:英文→中文→數字→語音→英文)。
           語音那一站是從數字進來的,mode 變數還停在 NUMBERS,不重建的話會回到數字鍵盤。
           要在下面的升起動畫之前重建 —— 動畫抓的是 keyboard_container,重建會換掉它。 */
        if (current_keyboard_mode != KEYBOARD_MODE_LETTERS &&
            keyboard_container != NULL && lv_obj_is_valid(keyboard_container))
        {
            kbd_pinyin_clear();
            current_keyboard_mode = KEYBOARD_MODE_LETTERS;
            lv_obj_t *kbd_parent = lv_obj_get_parent(keyboard_container);
            bool kbd_was_visible =
                !lv_obj_has_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
            lv_obj_del(keyboard_container);
            create_circular_keyboard_layout(kbd_parent);
            if (kbd_was_visible)
                lv_obj_clear_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
            kbd_cand_refresh(); /* 離開中文 → 收掉候選/數字快捷列 */
        }
        // mic → keyboard：
        //   1. mic section 直接 hide
        //   2. input bar y 從 195 → 75 (上移到鍵盤上方)
        //   3. keyboard 從下方升起 translate_y 300 → 0
        /* 語音區的三顆(麥克風/地球/收下)跟著往**下**退出畫面,把位置讓給升起來的鍵盤。 */
        if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
        {
            lv_anim_del(kbd_mic_section, anim_set_translate_x);
            lv_anim_del(kbd_mic_section, anim_set_translate_y);
            lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_translate_x(kbd_mic_section, 0, 0);
            lv_obj_set_style_translate_y(kbd_mic_section, 0, 0);
            lv_anim_t am;
            lv_anim_init(&am);
            lv_anim_set_var(&am, kbd_mic_section);
            lv_anim_set_exec_cb(&am, anim_set_translate_y);
            lv_anim_set_values(&am, 0, LV_VER_RES);
            lv_anim_set_time(&am, 260);
            lv_anim_set_path_cb(&am, lv_anim_path_ease_out);
            lv_anim_set_ready_cb(&am, hide_after_slide_y_cb);
            lv_anim_start(&am);
        }
        /* 輸入框從語音大框**縮**回鍵盤藥丸(位置與尺寸一起補間)。 */
        if (morph_bar)
        {
            lv_anim_del(text_input_bar_bg, (lv_anim_exec_xcb_t)lv_obj_set_y);
            lv_anim_del(text_input_bar_bg, kbd_bar_morph_cb);
            s_barmorph_x1 = (LV_HOR_RES_MAX - KBD_BAR_W) / 2;
            s_barmorph_y1 = EXPAND_END_Y_KBD;
            s_barmorph_w1 = KBD_BAR_W;
            s_barmorph_h1 = KBD_BAR_H;
            s_barmorph_r0 = VOICE_BOX_RADIUS;
            s_barmorph_r1 = 100;
            kbd_bar_morph_cb(text_input_bar_bg, 0); /* 起點立刻就位,別閃一格終點 */
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, text_input_bar_bg);
            lv_anim_set_exec_cb(&a, kbd_bar_morph_cb);
            lv_anim_set_values(&a, 0, 100);
            lv_anim_set_time(&a, 260);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_start(&a);
        }
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
        {
            /* 鍵盤從**下方升起**(founder 2026-08-18)。原本是從右邊滑進來,與輸入框
               長大/縮小的方向對不上,看起來像兩頁在左右輪播而不是同一個輸入面在變形。 */
            lv_anim_del(keyboard_container, anim_set_translate_x);
            lv_anim_del(keyboard_container, anim_set_translate_y);
            lv_obj_clear_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_translate_x(keyboard_container, 0, 0);
            lv_obj_set_style_translate_y(keyboard_container, LV_VER_RES, 0);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, keyboard_container);
            lv_anim_set_exec_cb(&a, anim_set_translate_y);
            lv_anim_set_values(&a, LV_VER_RES, 0);
            lv_anim_set_time(&a, 260);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_set_ready_cb(&a, kbd_slide_in_y_done_cb);
            lv_anim_start(&a);
        }
        kbd_lower_is_keyboard = true;
        /* 游標的閃爍是 timer 驅動的,而 start/stop_cursor_blink() 只綁輸入框的
           開/關,沒有綁「語音站 ⇄ 鍵盤站」的切換 —— 切過來時沒人重新啟動它,
           游標就一直停在隱藏狀態(founder 2026-08-07:跳到鍵盤後游標消失)。 */
        start_cursor_blink();
        /* 畫面捲到插入點：用 async 等版面從「折行大框」完全切回單行之後再算，
           否則拿到的還是舊幾何（見 update_input_display 裡的說明）。 */
        lv_async_call(kbd_scroll_to_caret_async, NULL);
        kbd_lower_update_arrows_visibility();
        kbd_lower_update_arcs_visibility();
    }
    else
    {
        // keyboard → mic：
        //   1. keyboard 往下滑出 (translate_y 0 → 300) 後 hide
        //   2. input bar y 從 75 → 195 (回到螢幕中央)
        //   3. 顯示 mic section
        /* 這一輪到底是不是「真的從鍵盤切過來」:鍵盤有在畫面上才算(從觸控板直接進語音站時
           它是藏著的)。原本只有下面的橫向動畫看它,現在框的變形也要看 —— 進場那條路有它
           自己的展開動畫在驅動框的幾何(0→100 收在 mic view 的 380x90@195),我這段變形
           跑在它前面,於是使用者會看到「框先長到語音框的正常大小,又縮成中間一個小框」
           (founder 2026-08-18)。進場不歸這裡管。 */
        bool from_keyboard = keyboard_container &&
                             lv_obj_is_valid(keyboard_container) &&
                             !lv_obj_has_flag(keyboard_container,
                                              LV_OBJ_FLAG_HIDDEN);
        /* 進場那條路不演變形,但**位置還是要放對**:kbd_bar_set_voice_box() 只設了尺寸與 x,
           y 一向是靠這裡的動畫帶到 VOICE_BOX_Y 的。上一輪把動畫 gate 掉之後就沒有人設 y,
           框於是停在鍵盤站的高度(founder 2026-08-18:「輸入框位置整個往上跑」)。這裡直接
           就位,不動畫 —— 進場的視覺歸進場動畫管。 */
        if (morph_bar && !from_keyboard)
        {
            lv_obj_set_size(text_input_bar_bg, VOICE_BOX_W, VOICE_BOX_H);
            lv_obj_set_pos(text_input_bar_bg,
                           (LV_HOR_RES_MAX - VOICE_BOX_W) / 2, VOICE_BOX_Y);
            lv_obj_set_style_radius(text_input_bar_bg, VOICE_BOX_RADIUS,
                                    LV_PART_MAIN);
        }
        /* 輸入框從鍵盤藥丸**長大**成語音大框(位置與尺寸一起補間)。 */
        if (morph_bar && from_keyboard)
        {
            lv_anim_del(text_input_bar_bg, (lv_anim_exec_xcb_t)lv_obj_set_y);
            lv_anim_del(text_input_bar_bg, kbd_bar_morph_cb);
            s_barmorph_x1 = (LV_HOR_RES_MAX - VOICE_BOX_W) / 2;
            s_barmorph_y1 = VOICE_BOX_Y;
            s_barmorph_w1 = VOICE_BOX_W;
            s_barmorph_h1 = VOICE_BOX_H;
            s_barmorph_r0 = 100;               /* 鍵盤藥丸 */
            s_barmorph_r1 = VOICE_BOX_RADIUS;  /* 逼近卡片圖的轉角 */
            /* 變形期間**框自己畫**(沿用鍵盤那套底色+邊框),卡片圖先收起來 ——
               否則長大的過程中框會整個看不見(見 kbd_bar_morph_to_voice_done_cb)。
               kbd_bar_set_voice_box() 已經把語音外觀套上去了,這裡先還原回去。 */
            if (s_bar_voice_frame && lv_obj_is_valid(s_bar_voice_frame))
                lv_obj_add_flag(s_bar_voice_frame, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(text_input_bar_bg, lv_color_hex(0x1a1a1a),
                                      LV_PART_MAIN);
            lv_obj_set_style_bg_opa(text_input_bar_bg, LV_OPA_90, LV_PART_MAIN);
            lv_obj_set_style_border_color(text_input_bar_bg, lv_color_hex(0xFFFFFF),
                                          LV_PART_MAIN);
            lv_obj_set_style_border_width(text_input_bar_bg, 2, LV_PART_MAIN);
            lv_obj_set_style_border_opa(text_input_bar_bg, LV_OPA_50, LV_PART_MAIN);
            kbd_bar_morph_cb(text_input_bar_bg, 0);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, text_input_bar_bg);
            lv_anim_set_exec_cb(&a, kbd_bar_morph_cb);
            lv_anim_set_values(&a, 0, 100);
            lv_anim_set_time(&a, 250);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_set_ready_cb(&a, kbd_bar_morph_to_voice_done_cb);
            lv_anim_start(&a);
        }
        if (keyboard_container && lv_obj_is_valid(keyboard_container))
        {
            lv_anim_del(keyboard_container, anim_set_translate_x);
            lv_obj_set_style_translate_y(keyboard_container, 0, 0);
            /* 鍵盤本來就藏著(例如從觸控板直接進語音站)就沒有「滑出去」這回事 ——
               照演的話使用者會先看到一整片鍵盤往右邊離開才進到語音站
               (founder 2026-08-07)。只有真的在畫面上才演退場。 */
            if (lv_obj_has_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN))
            {
                lv_obj_set_style_translate_x(keyboard_container, 0, 0);
            }
            else if (!from_keyboard)
            {
                /* 進場(不是換頁)就直接藏並歸零，不演退場也不留位移 */
                lv_obj_add_flag(keyboard_container, LV_OBJ_FLAG_HIDDEN);
                lv_obj_set_style_translate_x(keyboard_container, 0, 0);
            }
            else
            {
                /* 鍵盤**往下收掉**(founder 2026-08-18),與輸入框同時長大成語音框 ——
                   兩個動作方向一致才讀得出「鍵盤讓位給輸入框」而不是兩頁在輪播。 */
                lv_anim_del(keyboard_container, anim_set_translate_y);
                lv_obj_set_style_translate_x(keyboard_container, 0, 0);
                lv_anim_t a;
                lv_anim_init(&a);
                lv_anim_set_var(&a, keyboard_container);
                lv_anim_set_exec_cb(&a, anim_set_translate_y);
                lv_anim_set_values(&a, 0, LV_VER_RES);
                lv_anim_set_time(&a, 260);
                lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
                lv_anim_set_ready_cb(&a, hide_after_slide_y_cb);
                lv_anim_start(&a);
            }
        }
        if (kbd_mic_section && lv_obj_is_valid(kbd_mic_section))
        {
            lv_anim_del(kbd_mic_section, anim_set_translate_x);
            lv_obj_clear_flag(kbd_mic_section, LV_OBJ_FLAG_HIDDEN);
            if (!from_keyboard)
            {
                /* 不是換頁,是整個輸入頁進場 → 就位即可,橫向交給外層那個由下往上 */
                lv_obj_set_style_translate_x(kbd_mic_section, 0, 0);
            }
            else
            {
                /* 三顆按鈕跟著鍵盤讓出的空間**由下往上**回到定位。 */
                lv_anim_del(kbd_mic_section, anim_set_translate_y);
                lv_obj_set_style_translate_x(kbd_mic_section, 0, 0);
                lv_obj_set_style_translate_y(kbd_mic_section, LV_VER_RES, 0);
                lv_anim_t am;
                lv_anim_init(&am);
                lv_anim_set_var(&am, kbd_mic_section);
                lv_anim_set_exec_cb(&am, anim_set_translate_y);
                lv_anim_set_values(&am, LV_VER_RES, 0);
                lv_anim_set_time(&am, 260);
                lv_anim_set_path_cb(&am, lv_anim_path_ease_out);
                lv_anim_start(&am);
            }
        }
        /* 選字/數字快捷列屬於鍵盤,語音站沒有鍵可打,留著只是擋住畫面
           (sim 2026-08-03:進語音站頂部掛著一排 1-9)。 */
        if (s_kbd_cand_row && lv_obj_is_valid(s_kbd_cand_row))
            lv_obj_add_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
        kbd_lower_is_keyboard = false;
        kbd_lower_update_arrows_visibility();
        kbd_lower_update_arcs_visibility();
    }
}

static void kbd_lower_arrow_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    // 0 = 左箭頭 → 切回 mic（從 keyboard 回 mic）
    // 1 = 右箭頭 / 底部鍵盤鈕 → 切到 keyboard
    kbd_lower_switch((int)(intptr_t)lv_event_get_user_data(e) == 1);
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
// （y=110, h=45；466 圓內輸入列最高位置）
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
    // 箭頭現在是 section 的子物件，自動跟 translate_y，不用單獨處理
}

static void expand_anim_done_cb(lv_anim_t *a)
{
    (void)a;
    // 完成後正式收掉 trackpad mode。bar 的顯示已由 mode_set_visible 連動（TRACKPAD
    // 隱藏→bar 隱藏），不在此 un-hide：否則 keyboard mode 期間 bar 的 HIDDEN 被清掉，
    // 第二次開輸入框時會殘留在畫面底部。回 trackpad 由 collapse 的 mode_set_visible 還原。
    mode_set_visible(HID_MODE_TRACKPAD, false);
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
/* 這次收合的落點是「滑回 session 抽屜」而不是光禿禿的觸控板 —— 只有**輸入框往下拖**
   這條手勢會設(founder 2026-08-17:「按著上面往下滑才會把輸入框或鍵盤往下收掉、拉回
   已經有對應輸入變更過的選項」);點外面/右上退出鈕是離開整個流程,抽屜一併收掉。 */
static bool s_collapse_to_drawer = false;

/* 鍵盤→觸碰板的 commit 收尾(bar 下拉 collapse 與頂部退出鈕右滑共用)。 */
static void kbd_commit_to_trackpad(void)
{
    {
        // 切換到 trackpad mode：
        //   1. V2T 若在錄 → 收掉
        //   2. trackpad_mic_btn 取消 hidden（之前 expand 時被 hide）
        //   3. mode container 切過去；mode_set_visible(KEYBOARD, false) 內部
        //      會重設 text_input_bar_bg 樣式/位置回 closed 狀態
        //   4. kbd_mic_section 還原 translate_y / 視 visibility 預設
        if (mouse_v2t_active)
        {
            mouse_v2t_close_and_paste();
        }
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
        kbd_lower_is_keyboard = false;
        /* 語音站的大框狀態要跟著收 —— 這裡漏清是「回觸控板後 skaibar_img 從此點不動」
           的根源(見 kbd_input_active_area 的說明),而且它同時決定退格/游標的語意
           (remove_from_input_buffer),留著會讓觸控板站沿用語音站的編輯行為。
           只清旗標不呼叫 kbd_bar_set_voice_box():收合動畫已經在處理幾何,
           不要在收尾當下再插一次版面重排。 */
        s_voice_box_on = false;
        kbd_lower_update_arrows_visibility();
        kbd_lower_update_arrows_visibility();
        kbd_lower_update_arcs_visibility();
        // 完全退出輸入模式 → 清掉 skaibar 狀態
        skaibar_active = false;
        skaibar_options_count = 0;
        skaibar_selected_idx = -1;
    }
    /* 直打模式從**任何**出口離站(下拉收合、右上退出鈕、退格鍵當退出鍵)都要解除電腦的
       direct-typing latch。漏掉的話電腦會一直把後續任何 setSkaibarText 打進使用者的文件。
       送出那條已經自己解除並清旗標,所以這裡是 idempotent 的補網。 */
    if (s_kbd_direct_field)
    {
        s_kbd_direct_field = false;
        extern bool commu_send_skaibar_dismiss(void);
        commu_send_skaibar_dismiss();
    }
    /* 從 session 抽屜進來的:下拉收合 → 抽屜帶著這輪語音搜尋後的最新選項滑回來;
       其他離場方式(點外面、右上退出鈕)→ 整條流程結束,抽屜也收掉並通知電腦。 */
    if (s_kbd_from_drawer)
    {
        s_kbd_from_drawer = false;
        if (s_collapse_to_drawer)
        {
            extern void instruction_list_drawer_slide_in(void);
            instruction_list_drawer_slide_in();
        }
        else
        {
            extern void instruction_list_bar_device_dismiss(void);
            instruction_list_bar_device_dismiss();
        }
    }
    s_collapse_to_drawer = false;
}

static void collapse_anim_done_cb(lv_anim_t *a)
{
    (void)a;
    collapse_anim_running = false;
    if (collapse_anim_commit)
    {
        kbd_commit_to_trackpad();
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
    /* bar 下拉收回:數字/選字列跟不了 bar 的縮放動畫,commit 時立即收掉 */
    if (commit && s_kbd_cand_row && lv_obj_is_valid(s_kbd_cand_row))
        lv_obj_add_flag(s_kbd_cand_row, LV_OBJ_FLAG_HIDDEN);
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

    if (code == LV_EVENT_SHORT_CLICKED)
    {
        /* 鍵盤站也要能點字移游標,邏輯與語音站一致(founder 2026-08-07)。
           只認短按 —— 往下拖收回輸入模式那條手勢照舊。 */
        /* 用**按下**時記下的座標，不能用這裡現讀的 —— SHORT_CLICKED 是放開時才
           觸發，而 ft3168 在 Up 事件回報的座標是 (0,0)(這顆面板的老毛病，別處已
           擋過)。現讀會得到 (0,0)，換算成 label 相對座標是負的，
           lv_label_get_letter_on() 一律回第 0 個字 —— 症狀就是「點哪裡游標都跳到
           文字最前面」(founder 2026-08-07，[tap] 探針九次全是 (0,0) 坐實)。 */
        lv_point_t tp = { tib_drag_start_x, tib_drag_start_y };
        if (tp.x == 0 && tp.y == 0)
            return; /* 連按下都是幻影座標 → 這一下不算 */
        uint32_t idx = 0;
        if (voice_letter_at(tp, &idx))
            voice_set_caret_cp((int)idx);
        return;
    }
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
        /* 下拉是唯一「收掉輸入框但留在抽屜流程裡」的手勢。 */
        s_collapse_to_drawer = commit && s_kbd_from_drawer;
        start_kbd_to_trackpad_collapse_anim(progress, commit);
    }
}

/* 鍵盤退出=整組(鍵盤+輸入 bar+退出鈕)往右滑出——跟手寫退出一致(founder
   2026-07-22:「不要往下縮」);bar 下拉手勢仍走原 collapse(跟手方向)。 */
static void kbd_exit_slide_exec(void *var, int32_t v)
{
    (void)var;
    if (keyboard_container && lv_obj_is_valid(keyboard_container))
        lv_obj_set_style_translate_x(keyboard_container, (lv_coord_t)v, 0);
    if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
        lv_obj_set_style_translate_x(text_input_bar_bg, (lv_coord_t)v, 0);
    if (kbd_exit_btn && lv_obj_is_valid(kbd_exit_btn))
        lv_obj_set_style_translate_x(kbd_exit_btn, (lv_coord_t)v, 0);
    /* 數字/選字列跟整組一起滑出(founder 2026-07-22:原本留在原地到結束才消失) */
    if (s_kbd_cand_row && lv_obj_is_valid(s_kbd_cand_row))
        lv_obj_set_style_translate_x(s_kbd_cand_row, (lv_coord_t)v, 0);
}

static void kbd_exit_slide_done_cb(lv_anim_t *a)
{
    (void)a;
    collapse_anim_running = false;
    kbd_exit_slide_exec(NULL, 0); /* translate 歸位(同 frame 內接著就被藏) */
    /* 把 expand 佈局歸到 collapsed(=下拉收合的終點):commit 會把 mic 區 reset
       成可見備下次用,不歸位的話它停在展開位,黑色鍵盤圖示鈕會露在畫面上
       (founder 2026-07-22)。 */
    expand_anim_driver_cb(NULL, 0);
    kbd_commit_to_trackpad();
}

static void kbd_exit_btn_event_cb(lv_event_t *e)
{
    (void)e;
    if (current_hid_mode != HID_MODE_KEYBOARD || collapse_anim_running)
        return;
    collapse_anim_running = true; /* 滑出中擋 bar 下拉/重複點擊 */
    /* 先把觸碰板鋪在底下(手寫退出同款,founder 2026-07-22:滑出途中要看到
       落點不是黑畫面);滑出的三件套提到前景,從觸碰板上面滑走。 */
    if (keyboard_container && lv_obj_is_valid(keyboard_container))
        lv_obj_move_foreground(keyboard_container);
    if (text_input_bar_bg && lv_obj_is_valid(text_input_bar_bg))
        lv_obj_move_foreground(text_input_bar_bg);
    if (kbd_exit_btn && lv_obj_is_valid(kbd_exit_btn))
        lv_obj_move_foreground(kbd_exit_btn);
    mode_set_visible(HID_MODE_TRACKPAD, true);
    if (trackpad_mic_btn && lv_obj_is_valid(trackpad_mic_btn))
        lv_obj_clear_flag(trackpad_mic_btn, LV_OBJ_FLAG_HIDDEN);
    scroll_arcs_set_hidden(false); /* 觸碰板滾動弧提前顯回 */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, kbd_exit_btn); /* var 不重要,exec 內不使用 */
    lv_anim_set_values(&a, 0, LV_HOR_RES);
    lv_anim_set_time(&a, 200); /* 手寫退出右滑同款 */
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, kbd_exit_slide_exec);
    lv_anim_set_ready_cb(&a, kbd_exit_slide_done_cb);
    lv_anim_start(&a);
}

/* 語音站的麥克風:**按住講話、放開停止**(founder 2026-08-03,與立起面板同手感)。
   原本是 tap toggle —— 對講機式的按住在手錶上更不會忘記關。 */
static void kbd_mic_btn_event_cb(lv_event_t *e)
{
    switch (lv_event_get_code(e))
    {
    case LV_EVENT_PRESSED:
    {
        if (!mouse_v2t_active)
            mouse_v2t_open();
        /* 按住麥克風 → 手指正上方生出淡 logo,往上拖變實心、放開才送 AI
           (founder 2026-08-07)。 */
        lv_indev_t *indev = lv_indev_get_act();
        if (indev)
        {
            lv_point_t pt;
            lv_indev_get_point(indev, &pt);
            ai_drag_logo_show(pt, lv_event_get_target(e));
        }
        break;
    }
    case LV_EVENT_PRESSING:
    {
        lv_indev_t *indev = lv_indev_get_act();
        if (indev)
        {
            lv_point_t pt;
            lv_indev_get_point(indev, &pt);
            ai_drag_logo_track(pt);
        }
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
    {
        /* 只有真的放開才送;PRESS_LOST 是 press 被搶走,當取消。 */
        bool send_ai = s_ai_drag_armed &&
                       (lv_event_get_code(e) == LV_EVENT_RELEASED);
        ai_drag_logo_hide();
        if (mouse_v2t_active)
        {
            mouse_v2t_close_and_paste();
        }
        if (send_ai)
            voice_do_send(0);
        break;
    }
    default:
        break;
    }
}

/* 沒字 = 這顆鍵是退出鍵,圖示跟著換成向下箭頭(沿用滑鼠 app「收回」的同一張圖)。
   先前建了按鈕卻沒有任何地方更新圖示,所以永遠顯示退格圖 —— founder 2026-08-04
   「為什麼輸入框是空的時候刪除鍵沒有變成退出鍵」。 */
static void kbd_voice_del_update_icon(void)
{
    if (!kbd_voice_del_btn || !lv_obj_is_valid(kbd_voice_del_btn))
        return;
    lv_img_set_src(kbd_voice_del_btn,
                   input_length > 0 ? (const void *)&backspace_icon
                                    : (const void *)&down_arrow);
}

static void kbd_voice_del_stop_repeat(void)
{
    if (kbd_voice_del_repeat)
    {
        lv_timer_del(kbd_voice_del_repeat);
        kbd_voice_del_repeat = NULL;
    }
}

static void kbd_voice_del_repeat_cb(lv_timer_t *t)
{
    (void)t;
    /* 刪光了就自己收掉,不要空轉 —— 少一個「timer 活著但沒人記得停」的機會。 */
    if (input_length == 0)
    {
        kbd_voice_del_stop_repeat();
        return;
    }
    remove_from_input_buffer();
}

/* 刪除鍵:點一下刪一個字、長按連續刪(120ms)。文字是本地 input_buffer,所以直接刪,
   不像立起面板要跟手機來回。 */
static void kbd_voice_del_event_cb(lv_event_t *e)
{
    /* **放開一定要先收掉連續刪除的 timer**,不能被下面的空框攔截吃掉 —— 長按刪到全空的
       那一刻 input_length 變 0,放開送來的 RELEASED 就會被攔截 return,timer 於是永遠在
       背景每 120ms 刪一次:使用者按麥克風講出來的字一出現就被吃掉
       (founder 2026-08-03:「我沒按刪除只按著麥克風他會一直把出現的文字馬上刪掉」)。 */
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
        kbd_voice_del_stop_repeat();

    /* 框裡沒字 = 這顆鍵是退出鍵(founder 2026-08-01,語音站沿用)。攔在最前面,
       空框按住不放也不會「連續退出」。 */
    if (input_length == 0)
    {
        if (lv_event_get_code(e) == LV_EVENT_SHORT_CLICKED)
        {
            /* 退出鍵 = **離開輸入模式**回觸控板,與 icon_send 送完的去處一致。
               先前寫成退回鍵盤站,按下去會跳出英文鍵盤,不是使用者要的「退出」
               (founder 2026-08-04)。 */
            LOG_I("[voice] delete key acted as EXIT (box empty)");
            mouse_exit_input_station();
        }
        return;
    }
    switch (lv_event_get_code(e))
    {
    case LV_EVENT_SHORT_CLICKED:
        remove_from_input_buffer();
        break;
    case LV_EVENT_LONG_PRESSED:
        kbd_voice_del_stop_repeat();
        remove_from_input_buffer(); /* 先刪一個,不等第一次 tick */
        kbd_voice_del_repeat =
            lv_timer_create(kbd_voice_del_repeat_cb, VOICE_DEL_REPEAT_MS, NULL);
        break;
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        kbd_voice_del_stop_repeat();
        break;
    default:
        break;
    }
}

/* 依「電腦目前有沒有聚焦輸入框」擺放上方兩顆:有 → logo 左 / send 右;沒有 → 只有 logo,
   置中。與立起面板同一條規則(instruction_list 那份的 lift_input_layout_icons)。 */
static void kbd_voice_send_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    /* 跟鍵盤每一顆鍵同一個觸覺回饋(founder 2026-08-18)。用 motor_pattern_key_tick 而不是
       另挑一個 pattern:這顆在使用者眼裡就是鍵盤上的一顆鍵,手感不該自成一格。 */
    extern void motor_pattern_key_tick(void);
    motor_pattern_key_tick();
    voice_do_send((int)(intptr_t)lv_event_get_user_data(e));
}

static void kbd_voice_layout_send_icons(void)
{
    extern bool instruction_list_remote_target_has_focus(void);
    bool show_send = instruction_list_remote_target_has_focus();
    /* 送 AI 的 logo 已改成長按時的浮動圖示(kbd_voice_logo_btn 恆為 NULL),所以這裡
       只剩 icon_send 要擺 —— 而且它現在是單獨一顆,置中就好,不必再跟 logo 分左右。
       (2026-08-07 之前:兩顆並排,靠 VOICE_LOGO_DX / VOICE_SEND_DX 分家。) */
    if (kbd_voice_logo_btn && lv_obj_is_valid(kbd_voice_logo_btn))
        lv_obj_align(kbd_voice_logo_btn, LV_ALIGN_CENTER,
                     show_send ? VOICE_LOGO_DX : 0, VOICE_ICON_DY);
    if (kbd_voice_send_btn && lv_obj_is_valid(kbd_voice_send_btn))
    {
        if (show_send)
        {
            /* 貼輸入框右緣、垂直置中,再加 SEND_BTN_NUDGE_Y 的光學補償。
               對齊基準用 RIGHT_MID:它對的是 parent 的**內容區**,而這個框的 pad 四邊相等
               (真機探針:pad_t=5、border 語音站 0 / 鍵盤站 2),內容區中線=框中線,不必補償。
               **不要**改成「用框高自己算 y 偏移」(2026-08-18 試過,founder:「還是稍微偏上」):
               lv_obj_align 存的是偏移量,LVGL 之後每次 layout 都拿同一個偏移去套**當下**的
               框高;而框高一直在變(語音站 252 / 鍵盤站 45、展開收合動畫每幀都變),算的時候
               與套用時的框高一旦不同就整個歪掉 —— 探針拍到過 gap_top=111 / gap_bot=-100
               (按鈕掉到框外)。RIGHT_MID 每次 layout 對當下幾何重算,沒有這個時間差。 */
            lv_obj_align(kbd_voice_send_btn, LV_ALIGN_RIGHT_MID, SEND_BTN_INSET_X,
                         SEND_BTN_NUDGE_Y);
            lv_obj_clear_flag(kbd_voice_send_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(kbd_voice_send_btn); /* 鍵盤/候選列都可能後建 */
        }
        else
        {
            lv_obj_add_flag(kbd_voice_send_btn, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* 送出。which:0 = logo(當 skaibar 查詢送出,送查詢不自動執行)、1 = icon_send(打進電腦
   剛剛點的那個輸入框)。文字在手錶本地,所以**這裡要把文字一起送出去** —— 與立起面板的
   0x1d 不同(那條刻意不帶文字,因為真相在手機)。AI 口語整理由手機端在收到後跑。 */
/* which:0 = 送 skaibar 查詢(AI)、1 = 打進電腦聚焦中的輸入框。
   從常駐按鈕與「長按拖到 logo 放開」兩條入口共用,所以抽出來。 */
static void voice_do_send(int which)
{
    if (input_length == 0)
    {
        extern void motor_pattern_unlocked(void);
        motor_pattern_unlocked(); /* 沒字可送:震一下,別靜默(等於按鈕是死的) */
        LOG_I("[voice] send tapped with no text — ignored");
        return;
    }
    if (s_kbd_direct_field)
    {
        /* 直打模式(founder 2026-08-18:「按它就是按 enter 的意思」):字每一拍都已經打進電腦
           那個欄位了,所以這顆**不是**「送出文字」而是「在那個欄位按 Enter」。走 dest="enter"
           →手機 runSkaibar{text, submit:true}→桌面 TryRunText:文字相同時 diff 是空的(不會重打
           成雙份),然後敲一下 Enter。仍然把文字一起帶上去 —— 萬一兩邊因為丟包而不同步,這一
           筆會把差額補完再送出,比只送一個 Enter 穩。
           送完**離開輸入模式**(founder 2026-08-18 二改:「同時要退出鍵盤模式」)—— 所以這裡
           順手解除電腦的 direct-typing latch,不然人已經走了,電腦還把後續任何文字往那個欄位
           打。順序是先 enter 後 dismiss,同一條連線依序抵達,Enter 不會被 dismiss 搶先。 */
        extern bool commu_send_voice_station_commit(const char *dest, const char *text);
        commu_send_voice_station_commit("enter", input_buffer);
        LOG_I("[voice] direct-field enter len=%d", input_length);
        extern bool commu_send_skaibar_dismiss(void);
        commu_send_skaibar_dismiss();
        s_kbd_direct_field = false;
    }
    else
    {
        extern bool commu_send_voice_station_commit(const char *dest, const char *text);
        commu_send_voice_station_commit(which == 1 ? "field" : "skaibar", input_buffer);
        LOG_I("[voice] commit dest=%s len=%d", which == 1 ? "field" : "skaibar",
              input_length);
    }

    if (mouse_v2t_active)
    {
        mouse_v2t_close_and_paste();
    }
    /* 送出後文字就交出去了,本地清空 —— 不清的話下一次進來還掛著上一輪的字。 */
    memset(input_buffer, 0, sizeof(input_buffer));
    input_length = 0;
    s_voice_caret = -1;
    voice_sel_clear();
    mouse_v2t_locked = false;
    update_input_display();

    if (which == 1)
    {
        /* icon_send:文字已經打進電腦那個欄位,手錶離開輸入模式(founder 2026-08-03:
           「我按下輸入後手表要退出輸入模式」)。回觸控板,與立起面板送完就收掉同義。 */
        mouse_exit_input_station();
    }
    else
    {
        /* logo:當 skaibar 查詢送出 → 叫出選項清單(founder:「像原本那樣變成選項列表」)。
           走的是立起面板 logo 那條同一個出口 instruction_list_open_browse(),清單內容
           由電腦算好、經手機推回來。 */
        skaibar_active = true; /* 之後 arc 滾動 = 切選項,不是捲動 */
        /* **要先離開鍵盤模式**,清單才看得到 —— 不退的話鍵盤那層還蓋在上面,
           instruction_list 開了也被壓在底下(founder 2026-08-03:「電腦正確但手錶沒有
           退出輸入變成選項」)。立起面板對應的動作是 lift_input_hide_view()。 */
        apply_hid_mode(HID_MODE_TRACKPAD);
        extern void instruction_list_bar_set_blur(bool on);
        extern void instruction_list_open_browse(void);
        instruction_list_bar_set_blur(false); /* 非錶盤、不模糊 */
        instruction_list_open_browse();
    }
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

/* Media transport buttons. When an active remote target is selected
   (ble_hid_mouse_app_route()), relay the command to it via SKAI_LINK 0x18 (cmd
   strings match the air-mouse mediaControl verbs); otherwise keep driving the
   phone's own media over BLE HID consumer report, unchanged. */
static void media_center_play_btn_cb(lv_event_t *e)
{
    (void)e;
    LOG_D("media center play/pause tap");
    if (ble_hid_mouse_app_route())
        commu_send_media_relay("playPause");
    else
        play_pause_through_hid();
    media_center_update_play_icon(!media_center_play_state);
}

static void media_center_prev_btn_cb(lv_event_t *e)
{
    (void)e;
    LOG_D("media center prev tap");
    if (ble_hid_mouse_app_route())
        commu_send_media_relay("previous");
    else
        play_prev_through_hid();
}

static void media_center_next_btn_cb(lv_event_t *e)
{
    (void)e;
    LOG_D("media center next tap");
    if (ble_hid_mouse_app_route())
        commu_send_media_relay("next");
    else
        play_next_through_hid();
}

/* ─── 音量鍵長按連續 ───────────────────────────────────────────────────────
   按住 >0.5s 起每 100ms 送一次 volumeUp/Down（持續調整），放開停；短按（<0.5s）只送
   一次。play/prev/next 維持點一次的 CLICKED、不受影響。timer 在放開 + screen teardown
   （media_center_vol_cancel_repeat）都會清，避免 timer UAF 死當。 */
static lv_timer_t *s_vol_repeat_timer = NULL;
static uint32_t s_vol_press_tick = 0;
static int s_vol_dir = 0; /* +1 加 / -1 減 */
static bool s_vol_repeated = false;

static void media_center_vol_send(int dir)
{
    if (dir > 0)
    {
        if (ble_hid_mouse_app_route()) commu_send_media_relay("volumeUp");
        else volume_up_through_hid();
    }
    else
    {
        if (ble_hid_mouse_app_route()) commu_send_media_relay("volumeDown");
        else volume_down_through_hid();
    }
}

static void media_center_vol_cancel_repeat(void)
{
    if (s_vol_repeat_timer)
    {
        lv_timer_del(s_vol_repeat_timer);
        s_vol_repeat_timer = NULL;
    }
}

static void media_center_vol_repeat_cb(lv_timer_t *t)
{
    (void)t;
    /* 按住滿 0.5s 才開始連續送；前 0.5s 內的 tick 不送（短按由放開時處理）。 */
    if (lv_tick_elaps(s_vol_press_tick) >= 500)
    {
        media_center_vol_send(s_vol_dir);
        s_vol_repeated = true;
    }
}

/* 音量鍵事件（綁 LV_EVENT_ALL，只處理 PRESSED / RELEASED / PRESS_LOST）。
   user_data = 方向（+1 加 / -1 減）。 */
static void media_center_vol_hold_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    if (code == LV_EVENT_PRESSED)
    {
        s_vol_dir = dir;
        s_vol_press_tick = lv_tick_get();
        s_vol_repeated = false;
        media_center_vol_cancel_repeat();
        s_vol_repeat_timer =
            lv_timer_create(media_center_vol_repeat_cb, 100, NULL);
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        media_center_vol_cancel_repeat();
        if (!s_vol_repeated) media_center_vol_send(s_vol_dir); /* 短按：只送一次 */
    }
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
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *img = lv_img_create(btn);
    lv_img_set_src(img, icon);
    // 圖檔原生尺寸 ~150px（沿用 app_media.c 在 116×80 btn 用 zoom=0.6 的比例）
    // 依 btn size 線性縮放，讓 icon 約佔 btn 寬度的 ~70%
    uint16_t zoom = (uint16_t)(((uint32_t)256 * size * 7) / (10 * 150));
    lv_img_set_zoom(img, zoom);
    lv_obj_center(img);
    return btn;
}

/* 音量條:目前顯示中的那一格(同 media_center_title_label 的 live 指標作法)。 */
static lv_obj_t *s_media_vol_slider = NULL;
/* 收合狀態的音量鍵。條子平常收著,點它才展開 —— 與音樂 widget 一致
   (founder 2026-08-19:「我想要他跟那邊一樣是先點個按鈕才展開」)。 */
static lv_obj_t *s_media_vol_btn = NULL;
static bool s_media_vol_expanded = false;
/* 綁定一頁之後,還沒收到「這一台的音量」的第一筆回報。第一筆只校準把手、不展開:
   建立時的值只是佔位,拿它跟真實音量比對必然「看起來像變動」,於是每次滑到一頁
   條子就自己彈出來 —— 那不是使用者調的音量,不該叫它出來(founder 2026-08-20)。
   校準之後才恢復「真的變動就展開」。 */
static bool s_media_vol_calibrated = false;
static lv_timer_t *s_media_vol_collapse_timer = NULL;
#define MEDIA_VOL_BAR_WIDTH 300
#define MEDIA_VOL_BAR_Y (-70)
#define MEDIA_VOL_COLLAPSE_MS 3000 /* 同 app_media.c 的 VOL_BAR_COLLAPSE_TIMEOUT */

static void media_vol_bar_anim_width_cb(void *var, int32_t v)
{
    lv_obj_t *bar = (lv_obj_t *)var;
    if (!bar || !lv_obj_is_valid(bar)) return;
    lv_obj_set_width(bar, v);
    /* 寬度變了要重新置中,否則會從左緣長出來(音樂 widget 同款做法)。 */
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, MEDIA_VOL_BAR_Y);
    uint8_t opa = (uint8_t)((v * 255) / MEDIA_VOL_BAR_WIDTH);
    lv_obj_set_style_bg_opa(bar, opa, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, opa, LV_PART_INDICATOR);
}

static void media_vol_collapse_timer_cancel(void)
{
    if (s_media_vol_collapse_timer)
    {
        lv_timer_del(s_media_vol_collapse_timer);
        s_media_vol_collapse_timer = NULL;
    }
}

static void media_vol_bar_hide_done_cb(lv_anim_t *a)
{
    lv_obj_t *bar = (lv_obj_t *)a->var;
    if (bar && lv_obj_is_valid(bar)) lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);
    if (s_media_vol_btn && lv_obj_is_valid(s_media_vol_btn))
        lv_obj_clear_flag(s_media_vol_btn, LV_OBJ_FLAG_HIDDEN);
}

static void media_vol_collapse(void)
{
    media_vol_collapse_timer_cancel();
    if (!s_media_vol_expanded) return;
    s_media_vol_expanded = false;
    if (!s_media_vol_slider || !lv_obj_is_valid(s_media_vol_slider)) return;
    lv_anim_del(s_media_vol_slider, media_vol_bar_anim_width_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_media_vol_slider);
    lv_anim_set_values(&a, lv_obj_get_width(s_media_vol_slider), 0);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, media_vol_bar_anim_width_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_ready_cb(&a, media_vol_bar_hide_done_cb);
    lv_anim_start(&a);
}

static void media_vol_collapse_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_media_vol_collapse_timer = NULL; /* one-shot:自己跑完就沒了 */
    media_vol_collapse();
}

/* 每一次互動都把 3 秒重新計時 —— 拖到一半被收掉會很惱人。 */
static void media_vol_arm_collapse(void)
{
    media_vol_collapse_timer_cancel();
    s_media_vol_collapse_timer =
        lv_timer_create(media_vol_collapse_timer_cb, MEDIA_VOL_COLLAPSE_MS, NULL);
    lv_timer_set_repeat_count(s_media_vol_collapse_timer, 1);
}

static void media_vol_expand(void)
{
    if (!s_media_vol_slider || !lv_obj_is_valid(s_media_vol_slider)) return;
    if (s_media_vol_expanded)
    {
        media_vol_arm_collapse();
        return;
    }
    s_media_vol_expanded = true;
    /* 音量鍵讓位:它與條子佔同一格,留著會被壓在條子底下。 */
    if (s_media_vol_btn && lv_obj_is_valid(s_media_vol_btn))
        lv_obj_add_flag(s_media_vol_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_media_vol_slider, LV_OBJ_FLAG_HIDDEN);
    lv_anim_del(s_media_vol_slider, media_vol_bar_anim_width_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_media_vol_slider);
    lv_anim_set_values(&a, 0, MEDIA_VOL_BAR_WIDTH);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, media_vol_bar_anim_width_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
    media_vol_arm_collapse();
}

static void media_vol_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (s_media_vol_expanded) media_vol_collapse();
    else media_vol_expand();
}
/* 使用者正在拖 → 遠端回報先不要動把手,否則手指還按著就被拉回舊值。 */
static bool s_media_vol_dragging = false;
/* 上一次真的送出去的值 + 時間,用來節流(拖曳每一格都送會塞滿 BLE)。 */
static int s_media_vol_last_sent = -1;
static uint32_t s_media_vol_last_tick = 0;
#define MEDIA_VOL_SEND_MIN_MS 80
/* 送出去、還在等對方把同一個值回報回來的那個值(-1 = 沒有待對帳的)。
   為什麼需要:條子是樂觀更新(按下就到位),但對方的回報有往返延遲。放開手指的瞬間
   s_media_vol_dragging 就解除了,而**在你按之前對方就已經送出、還在路上**的那筆回報
   這時才到 —— 它帶的是舊音量,於是條子被拉回舊值,等對方真的套用新值再回報一次,才又
   animate 回去。使用者看到的就是「按下跳到最大、放開彈回中間、再自己滑回最大」
   (founder 2026-08-20)。送出後只接受自己的回聲,其餘在途的舊回報一律丟掉。 */
static int s_media_vol_pending_echo = -1;
static uint32_t s_media_vol_pending_tick = 0;
/* 對方可能永遠不回(舊桌面不回報音量、或那筆掉了)。超過這個時間就放棄等待,恢復接受
   回報 —— 寧可晚一點跟上真實值,也不要把條子永久鎖在一個對方其實沒套用的數字上。 */
#define MEDIA_VOL_ECHO_TIMEOUT_MS 1500

/* 送出目前的滑桿值。force=放開手指那一下:無論節流與否都要送,否則最終落點可能沒送到。 */
static void media_vol_send(int value, bool force)
{
    if (value < 0) value = 0;
    if (value > 100) value = 100;
    uint32_t now = lv_tick_get();
    if (!force)
    {
        if (value == s_media_vol_last_sent) return;
        if ((now - s_media_vol_last_tick) < MEDIA_VOL_SEND_MIN_MS) return;
    }
    s_media_vol_last_sent = value;
    s_media_vol_last_tick = now;
    s_media_vol_pending_echo = value; /* 等這個值回來才算對帳完成 */
    s_media_vol_pending_tick = now;
    /* 目標是遠端設備(電腦)→ 送絕對音量給它;目標是**手機自己**→ 走 BLE 的絕對音量,
       也就是音樂 widget 那顆音量條用的同一支(app_media.c 的 bt_speaker_set_volume)。
       原本這裡沒有遠端目標就直接 return,於是媒體頁切到「手機」那一頁時整條拉了沒反應
       (founder 2026-08-19:「我手錶媒體頁有一個是手機的,我調那邊的音量條手機不會有
       變化,但電腦是 OK 的」)——手機的音量一直有現成的路,只是這裡沒有接上去。 */
    if (ble_hid_mouse_app_route())
    {
        extern bool commu_send_media_volume(int percent);
        commu_send_media_volume(value);
    }
    else
    {
        control_provider.bt_speaker_set_volume((uint8_t)value, true);
    }
}

/* 觸控 x → 0..100。lv_bar 自己不處理拖曳(它是唯讀顯示元件),所以值要自己算 ——
   與音樂 widget 的 widget_bar_event_cb 同一套算法,手感才一致:點哪裡就跳到哪裡。 */
static int media_vol_value_at(lv_obj_t *bar, const lv_point_t *p)
{
    lv_area_t coords;
    lv_obj_get_coords(bar, &coords);
    lv_coord_t w = lv_obj_get_width(bar);
    if (w <= 0) return 0;
    lv_coord_t rel_x = p->x - coords.x1;
    if (rel_x < 0) rel_x = 0;
    if (rel_x > w) rel_x = w;
    return (int)((rel_x * 100) / w);
}

static void media_center_vol_slider_cb(lv_event_t *e)
{
    lv_obj_t *sl = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    lv_point_t p;
    switch (code)
    {
    case LV_EVENT_PRESSED:
    case LV_EVENT_PRESSING:
        if (!indev) break;
        lv_indev_get_point(indev, &p);
        s_media_vol_dragging = true;
        media_vol_arm_collapse(); /* 手指在上面 = 別收 */
        {
            int v = media_vol_value_at(sl, &p);
            lv_bar_set_value(sl, v, LV_ANIM_OFF);
            media_vol_send(v, false);
        }
        break;
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        s_media_vol_dragging = false;
        media_vol_arm_collapse(); /* 放手後再數 3 秒 */
        media_vol_send((int)lv_bar_get_value(sl), true); /* 落點一定要送 */
        break;
    default:
        break;
    }
}

/* 送出後的回聲對帳:true = 這筆該丟掉。相同值 = 自己的回聲,對帳完成(條子已經顯示
   它了,不用再動也不該展開);不同值且還在等待窗內 = 在途的舊回報,丟掉;超時 = 放棄
   等待,接受對方說的。 */
static bool media_vol_echo_should_drop(int percent)
{
    if (s_media_vol_pending_echo < 0) return false;
    if (percent == s_media_vol_pending_echo)
    {
        s_media_vol_pending_echo = -1;
        return true;
    }
    if ((lv_tick_get() - s_media_vol_pending_tick) < MEDIA_VOL_ECHO_TIMEOUT_MS)
        return true;
    s_media_vol_pending_echo = -1;
    return false;
}

/* 手機**自己**的音量在別處變動(實體音量鍵、手機上調整、AVRCP 通知)→ 媒體頁若正指著
   手機那一頁,條子要跟著走。來源是 bloc_control 既有的下行,它本來就在餵音樂 widget 與
   音樂 app 的兩條 bar,這裡只是第三個消費者(founder 2026-08-19:「手機還沒接上」)。
   指著遠端設備時不理會:那時條子顯示的是那台電腦的音量,不是手機的。 */
void mouse_mode_handle_phone_volume(uint8_t percent)
{
    if (percent > 100) return;
    if (!s_media_vol_slider || !lv_obj_is_valid(s_media_vol_slider)) return;
    if (ble_hid_mouse_app_route()) return; /* 目標是電腦 → 這筆不是它的音量 */
    if (s_media_vol_dragging) return;      /* 手指還在上面,別跟使用者搶 */
    if (media_vol_echo_should_drop((int)percent)) return;
    if (!s_media_vol_calibrated)
    {
        /* 這一頁的第一筆:校準,不展開(見 s_media_vol_calibrated)。 */
        s_media_vol_calibrated = true;
        lv_bar_set_value(s_media_vol_slider, percent, LV_ANIM_OFF);
        return;
    }
    if ((int)lv_bar_get_value(s_media_vol_slider) == (int)percent) return;
    lv_bar_set_value(s_media_vol_slider, percent, LV_ANIM_ON);
    media_vol_expand(); /* 與電腦那條、與音樂 widget 一致:變動時自己展開 */
}

/* 0x19 帶回來的該設備音量 → 校準把手。-1 = 對方沒回報,維持現狀。 */
void mouse_mode_handle_remote_volume(const char *device_id, int percent)
{
    if (percent < 0 || percent > 100) return;
    if (!s_media_vol_slider || !lv_obj_is_valid(s_media_vol_slider)) return;
    if (s_media_vol_dragging) return; /* 手指還在上面,別跟使用者搶 */
    if (device_id == NULL || s_dev_active_id[0] == '\0' ||
        strncmp(device_id, s_dev_active_id, SYNCED_DEVICE_ID_LEN) != 0)
        return;
    if (media_vol_echo_should_drop(percent)) return;
    if (!s_media_vol_calibrated)
    {
        /* 這一頁的第一筆:把佔位換成真實音量,但不展開(見 s_media_vol_calibrated)。 */
        s_media_vol_calibrated = true;
        lv_bar_set_value(s_media_vol_slider, percent, LV_ANIM_OFF);
        s_media_vol_last_sent = percent;
        return;
    }
    /* 值真的變了才動作 —— 相同數值的重複回報不該把條子叫出來。 */
    if ((int)lv_bar_get_value(s_media_vol_slider) == percent) return;
    lv_bar_set_value(s_media_vol_slider, percent, LV_ANIM_ON);
    /* 並且**自動展開**(founder 2026-08-19:「調音量時手錶上那頁的音量條不會自己展開」),
       與音樂 widget 的 set_widget_vol_bar_value 同款。
       先前這裡刻意不展開,理由是「回報搭在每一次 media 更新上,會三不五時自己跳出來」——
       那個理由現在不成立了:桌面端已改成**只在音量真的變動時**才廣播,所以一筆回報就代表
       一次真實的音量變化,正是該讓使用者看到條子的時機。 */
    media_vol_expand();
    s_media_vol_last_sent = percent; /* 這是電腦端的現況,不要再回送 */
}

/* === 媒體內容 builder（曲名 + 上/播/下 + 音量）====================================
   2026-08-06 重構:同一份控制列現在有兩個使用者 —
     1) 本檔 create_media_center_panel 的下拉媒體頁(獨立開 APP_ID_MOUSE 時仍在)
     2) 錶盤頂部面板(lv_top_panel.c)每台設備一頁的媒體格
   所以把控制列抽出來，兩邊共用同一份 callback，避免兩套實作各自漂移。
   設備名/箭頭**不**在這裡：面板版的設備列是固定在面板頂部、不隨頁捲動的。 */
/* [seed_phone_volume] 只有手機那一欄該給 true。頁面是每台設備各建一頁、而且由
   clock_main_media_cols_content_ensure() 一個迴圈一次建完,所以這裡不能拿
   ble_hid_mouse_app_route()(單一的「當前目標」全域旗標)來判斷 —— 那會把手機的
   音量蓋到每一欄上,包含電腦的(founder 2026-08-20 抓到)。 */
static void media_content_build(lv_obj_t *parent, lv_obj_t **out_title,
                                lv_obj_t **out_play_img,
                                lv_obj_t **out_vol_slider,
                                lv_obj_t **out_vol_btn,
                                bool seed_phone_volume)
{
    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, "Media Title");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
    lv_obj_set_width(title, LV_HOR_RES_MAX - 80);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 100);

    lv_obj_t *btn_prev = media_center_make_icon_btn(
        parent, &img_media_previous, media_center_prev_btn_cb, 90);
    lv_obj_align(btn_prev, LV_ALIGN_CENTER, -120, 0);

    lv_obj_t *btn_play = media_center_make_icon_btn(
        parent, &img_media_play, media_center_play_btn_cb, 120);
    lv_obj_align(btn_play, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *btn_next = media_center_make_icon_btn(
        parent, &img_media_next, media_center_next_btn_cb, 90);
    lv_obj_align(btn_next, LV_ALIGN_CENTER, 120, 0);

    /* 音量:滑桿(founder 2026-08-18「我拉多少就調多少音量」)。兩端的圖示留著當刻度提示,
       但**不再可點** —— 原本的 +/- 是按鍵、送 volumeUp/Down 讓接收端按系統音量鍵(相對、
       被 OS 級距量化),拉到哪就是多少做不到;滑桿送的是絕對值。
       圓螢幕:滑桿寬 240 置中,兩側各留一顆 32px 圖示還在圓內。 */
    /* 樣式與行為都對齊音樂 widget 的音量條(app_media.c 的 app_vol_bar,founder 2026-08-19
       「我希望音量調可以像我音樂 widget 的音量調整那樣」):lv_bar 而不是 slider —— 沒有獨立
       把手,點/拖到哪就是哪(值由 PRESSING 時的觸控 x 換算)。顏色、圓角、粗細沿用那邊的數字,
       只有寬度依這一頁的版面縮到 300(那邊是 370,鋪滿整個音樂 app 的寬度)。 */
    lv_obj_t *slider = lv_bar_create(parent);
    lv_bar_set_range(slider, 0, 100);
    lv_obj_set_size(slider, 300, 60);
    lv_obj_align(slider, LV_ALIGN_BOTTOM_MID, 0, -70);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x2F2F2F), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0xCDCDCD), LV_PART_INDICATOR);
    lv_obj_set_style_radius(slider, 16, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, 16, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(slider, LV_OPA_100, LV_PART_MAIN);
    /* 電腦那幾頁只能站佔位,等 0x19 回報校準(校準不展開,見 s_media_vol_calibrated)。
       手機那頁不一樣 —— bloc_control 手上已經有手機的音量(連線當下手機就推過一次),
       直接拿來當初值,而那一頁沒有 0x19 可以事後校準它。 */
    uint8_t slider_init = 50;
    if (seed_phone_volume && control_provider.bt_speaker_get_volume)
        slider_init = control_provider.bt_speaker_get_volume();
    lv_bar_set_value(slider, slider_init, LV_ANIM_OFF);
    /* 遠端回報是離散取樣(桌面每 250ms 送一次),直接套用會一格一格跳。給 bar 一個補間
       時間,兩筆之間就用滑的接起來(founder 2026-08-19:「不要讓它是瞬間跳過去」)。
       250ms = 取樣間隔:剛好在下一筆到達時走完,不會累積延遲也不會停頓。 */
    lv_obj_set_style_anim_time(slider, 250, LV_PART_MAIN);
    lv_obj_add_flag(slider, LV_OBJ_FLAG_CLICKABLE); /* lv_bar 預設不可點,要自己開 */
    /* **捲動死路**:音量條自己不可捲動,LVGL 的 find_scroll_obj 就會往上找可捲動的祖先,
       把橫向拖曳交給媒體頁的 pager —— 拉音量會把頁面一起拖走(founder 2026-08-19)。
       清掉 SCROLL_CHAIN 之後,鏈在這裡就斷,拖曳完整留給音量條。兩軸都清:垂直方向上面
       是媒體 tileview(下拉那層),同樣不該被拉音量的手指帶動。 */
    lv_obj_clear_flag(slider, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_clear_flag(slider, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_add_event_cb(slider, media_center_vol_slider_cb, LV_EVENT_ALL, NULL);
    /* 平常收著,點音量鍵才展開(音樂 widget 同款)。寬度 0 起跳,展開動畫從那裡長出來。 */
    lv_obj_add_flag(slider, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(slider, 0);

    /* 收合狀態的音量鍵 —— 樣式沿用音樂 app 的 app_vol_icon_btn(40x32、圓角 16、白底 8%)。 */
    lv_obj_t *vol_btn = lv_btn_create(parent);
    lv_obj_set_size(vol_btn, 40, 32);
    lv_obj_set_style_radius(vol_btn, 16, 0);
    lv_obj_set_style_bg_color(vol_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(vol_btn, 20, 0);
    lv_obj_set_style_shadow_width(vol_btn, 0, 0);
    lv_obj_align(vol_btn, LV_ALIGN_BOTTOM_MID, 0, MEDIA_VOL_BAR_Y);
    lv_obj_add_event_cb(vol_btn, media_vol_btn_cb, LV_EVENT_ALL, NULL);
    lv_obj_t *vol_ico = lv_img_create(vol_btn);
    lv_img_set_src(vol_ico, &volume_up);
    lv_img_set_zoom(vol_ico, 255 * 30 / 85);
    lv_obj_align(vol_ico, LV_ALIGN_CENTER, 0, 0);

    if (out_title) *out_title = title;
    if (out_play_img) *out_play_img = lv_obj_get_child(btn_play, 0);
    if (out_vol_slider) *out_vol_slider = slider;
    if (out_vol_btn) *out_vol_btn = vol_btn;
}

/* 面板媒體格:每格記住自己的曲名 label / 播放圖示，bind 時才把 file-static 的
   「live」指標指過去 —— 0x19 / 0x46 的曲名路由(mouse_mode_handle_*)完全不必改，
   永遠只寫目前顯示中的那一格。 */
typedef struct
{
    lv_obj_t *title;
    lv_obj_t *play_img;
    lv_obj_t *vol_slider;
    lv_obj_t *vol_btn;
} media_page_widgets_t;

static void media_page_del_cb(lv_event_t *e)
{
    media_page_widgets_t *w = (media_page_widgets_t *)lv_event_get_user_data(e);
    if (w == NULL) return;
    if (media_center_title_label == w->title)
    {
        media_center_title_label = NULL;
        media_center_play_img = NULL;
    }
    if (s_media_vol_slider == w->vol_slider)
    {
        /* 這一格要被刪了:收合 timer 的 callback 會碰到這些指標,一定要先收掉,
           否則就是又一個「活過畫面拆除的 lv_timer」(這支 app 已知的 UAF 形狀)。 */
        media_vol_collapse_timer_cancel();
        s_media_vol_slider = NULL;
        s_media_vol_btn = NULL;
        s_media_vol_expanded = false;
    }
    lv_mem_free(w);
}

/* [is_phone_column] = 這一頁屬於手機那一欄(頂部面板的第 0 欄)。呼叫端知道欄號,
   builder 不知道 —— 別在 builder 裡用全域的當前目標旗標猜。 */
lv_obj_t *hid_mouse_media_page_create(lv_obj_t *parent, bool is_phone_column)
{
    media_page_widgets_t *w =
        (media_page_widgets_t *)lv_mem_alloc(sizeof(media_page_widgets_t));
    if (w == NULL) return NULL;
    memset(w, 0, sizeof(*w));

    lv_obj_t *page = lv_obj_create(parent);
    lv_obj_remove_style_all(page);
    lv_obj_set_size(page, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_user_data(page, w);
    lv_obj_add_event_cb(page, media_page_del_cb, LV_EVENT_DELETE, w);

    media_content_build(page, &w->title, &w->play_img, &w->vol_slider, &w->vol_btn,
                        is_phone_column);
    return page;
}

void hid_mouse_media_page_bind(lv_obj_t *page)
{
    if (page == NULL || !lv_obj_is_valid(page)) return;
    media_page_widgets_t *w = (media_page_widgets_t *)lv_obj_get_user_data(page);
    if (w == NULL) return;
    media_center_title_label = w->title;
    media_center_play_img = w->play_img;
    s_media_vol_slider = w->vol_slider;
    s_media_vol_btn = w->vol_btn;
    /* 換頁 = 重新收合:上一頁展開過的狀態不該帶到這一頁。 */
    media_vol_collapse_timer_cancel();
    s_media_vol_expanded = false;
    if (w->vol_slider && lv_obj_is_valid(w->vol_slider))
    {
        lv_anim_del(w->vol_slider, media_vol_bar_anim_width_cb);
        lv_obj_add_flag(w->vol_slider, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_width(w->vol_slider, 0);
    }
    if (w->vol_btn && lv_obj_is_valid(w->vol_btn))
        lv_obj_clear_flag(w->vol_btn, LV_OBJ_FLAG_HIDDEN);
    /* 換設備 = 換一台機器的音量,舊的節流基準作廢(否則新設備第一筆相同數值會被吃掉)。 */
    s_media_vol_last_sent = -1;
    /* 也還沒收到新這一台的音量:下一筆回報是校準,不是使用者調的。 */
    s_media_vol_calibrated = false;
    /* 對上一台送出的值不該拿來跟這一台的回報對帳。 */
    s_media_vol_pending_echo = -1;
}

void hid_mouse_media_page_reset_title(lv_obj_t *page)
{
    if (page == NULL || !lv_obj_is_valid(page)) return;
    media_page_widgets_t *w = (media_page_widgets_t *)lv_obj_get_user_data(page);
    if (w == NULL || !lv_obj_is_valid(w->title)) return;
    lv_label_set_text(w->title, "Media Title");
}

/* 離開滑鼠 App：延後到事件處理結束才拆畫面，避免在自身 event cb 內同步拆畫面 UAF。
   （從舊右側抽屜的 exit 鈕移來，現掛在媒體頁。） */
static void media_exit_async_cb(void *p)
{
    (void)p;
    gui_app_exit(APP_ID_MOUSE);
}
static void media_exit_btn_cb(lv_event_t *e)
{
    (void)e;
    lv_async_call(media_exit_async_cb, NULL);
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

    /* 媒體頁頂部:「控制中設備」名稱+左右切換箭頭(2026-07-02 從 trackpad
       頂部搬來 — 使用者要求)。同座標系(tile 全屏),沿用原 y40/42 位置;
       label 非 clickable、箭頭可點;無設備時 update_ctrl_dev_label 隱藏。 */
    /* 在線燈號:設備名上方小圓點,綠=在線/紅=斷線(founder 2026-07-22);
       顏色/顯藏由 dev_offline_overlay_sync 管。 */
    s_dev_status_dot = lv_obj_create(media_tile);
    lv_obj_remove_style_all(s_dev_status_dot);
    lv_obj_set_size(s_dev_status_dot, 10, 10);
    lv_obj_align(s_dev_status_dot, LV_ALIGN_TOP_MID, 0, 26);
    lv_obj_set_style_radius(s_dev_status_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_dev_status_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_dev_status_dot, lv_color_hex(0x4CAF50), 0);
    lv_obj_clear_flag(s_dev_status_dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_dev_status_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_dev_status_dot, LV_OBJ_FLAG_HIDDEN);

    s_ctrl_dev_label = lv_label_create(media_tile);
    lv_obj_set_width(s_ctrl_dev_label, 200);
    lv_label_set_long_mode(s_ctrl_dev_label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(s_ctrl_dev_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_ctrl_dev_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(s_ctrl_dev_label, LV_OPA_80, 0);
    lv_obj_align(s_ctrl_dev_label, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_clear_flag(s_ctrl_dev_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_ctrl_dev_label, LV_OBJ_FLAG_SCROLLABLE);

    s_dev_left_arrow = lv_img_create(media_tile);
    lv_img_set_src(s_dev_left_arrow, &img_left_arrow);
    lv_obj_align(s_dev_left_arrow, LV_ALIGN_TOP_MID, -117, 42);
    lv_obj_add_flag(s_dev_left_arrow, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_dev_left_arrow, 24);
    lv_obj_add_event_cb(s_dev_left_arrow, dev_arrow_prev_cb, LV_EVENT_CLICKED,
                        NULL);

    s_dev_right_arrow = lv_img_create(media_tile);
    lv_img_set_src(s_dev_right_arrow, &img_right_arrow);
    lv_obj_align(s_dev_right_arrow, LV_ALIGN_TOP_MID, 117, 42);
    lv_obj_add_flag(s_dev_right_arrow, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_dev_right_arrow, 24);
    lv_obj_add_event_cb(s_dev_right_arrow, dev_arrow_next_cb, LV_EVENT_CLICKED,
                        NULL);

    update_ctrl_dev_label(); // 依目前 active 設備設定文字/顯示 + 箭頭可見性

    // 曲名 + 控制列 + 音量（與錶盤頂部面板的媒體格共用同一份 builder）
    /* 這一頁跟著 active 設備動態切換,沒有固定歸屬 → 不種,靠校準。 */
    media_content_build(media_tile, &media_center_title_label,
                        &media_center_play_img, &s_media_vol_slider,
                        &s_media_vol_btn, false);

    // 離開 App：紅色 Exit 鈕（從舊右側抽屜移來），放媒體頁最底
    lv_obj_t *media_exit_btn = lv_btn_create(media_tile);
    lv_obj_set_size(media_exit_btn, 160, 56);
    lv_obj_set_style_radius(media_exit_btn, 28, 0);
    lv_obj_set_style_bg_color(media_exit_btn, lv_color_hex(0xFF3B30), 0);
    lv_obj_set_style_bg_color(media_exit_btn, lv_color_hex(0xD9342B),
                              LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(media_exit_btn, LV_OPA_COVER, 0);
    lv_obj_clear_flag(media_exit_btn, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_align(media_exit_btn, LV_ALIGN_BOTTOM_MID, 0, -24);
    lv_obj_add_event_cb(media_exit_btn, media_exit_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *media_exit_lbl = lv_label_create(media_exit_btn);
    lv_label_set_text(media_exit_lbl, "Exit");
    lv_obj_set_style_text_color(media_exit_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(media_exit_lbl);
    lv_obj_clear_flag(media_exit_lbl, LV_OBJ_FLAG_CLICKABLE);

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
/* When the device-pager hosts this trackpad, the pager owns the top + bottom
   edges (its device-name strip + bottom input bar). Suppress the mouse page's
   own media-center pull-down so the two don't fight. Set from
   device_pager_set_active via hid_mouse_set_hosted. */
static bool s_hosted_by_pager = false;
void hid_mouse_set_hosted(bool hosted) { s_hosted_by_pager = hosted; }

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
        /* Seed the title. Use the phone's own media (0x46) ONLY when not routing
           to a remote target; for an active remote target leave whatever the last
           0x19 wrote, so opening the centre doesn't flash the phone's own song. */
        if (media_center_title_label &&
            lv_obj_is_valid(media_center_title_label) &&
            !ble_hid_mouse_app_route())
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

/* === 頂部區手勢分流：按住不動=飛鼠 / 下拉=媒體中心 / 快 tap=收合 ==========
   按下先照舊亮出 media tileview 保住下拉路徑，同時開 hold 計時器：
   - 手指往下拖離頂部區 → press 轉給 tileview（PRESS_LOST）→ 取消 hold，
     媒體下拉行為與從前完全相同
   - 位移累積超過閾值（慢速拖，還沒出區）→ 也取消 hold，讓拖曳走媒體
   - 按滿 TOP_HOLD_TO_FLY_MS 沒動 → 進飛鼠（handfree，同 FSR 捏壓開關），
     動態加 PRESS_LOCK 鎖住 press，之後手指小滑不會誤斷；放開即回 trackpad */
#define TOP_HOLD_TO_FLY_MS 250
/* 拖曳判定＝離按下起點的曼哈頓距離（不是每幀位移累積——真機手指按住
   自帶 ±1-2px/幀抖動，累積必超標、hold 永遠 fire 不了，2026-07-06 踩過） */
#define TOP_HOLD_DRIFT_CANCEL_PX 18

void set_hid_mouse_handfree_mode_to(bool v); // 定義在本檔後段
static lv_timer_t *s_top_hold_timer = NULL;
/* 圖示左拉=手寫頁跟手進場(founder 2026-07-22:按著滑鼠圖往左滑,滑鼠圖左滑走+
   手寫頁(頂部鍵盤圖)從右跟進;放開 1/4 或快甩=commit)。 */
static bool s_top_hw_pull = false;
static lv_coord_t s_top_hw_last_x = 0;
static lv_coord_t s_top_hw_vx = 0;

static void top_logo_tx_anim_exec(void *obj, int32_t v)
{
    lv_obj_set_style_translate_x((lv_obj_t *)obj, (lv_coord_t)v, 0);
}

/* 取消左拉:滑鼠圖動畫滑回原位(跳回會閃,founder 2026-07-22)。 */
static void top_logo_slide_home(void)
{
    if (s_top_logo == NULL || !lv_obj_is_valid(s_top_logo))
        return;
    lv_anim_del(s_top_logo, NULL);
    lv_coord_t cur = lv_obj_get_style_translate_x(s_top_logo, LV_PART_MAIN);
    if (cur == 0)
        return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_top_logo);
    lv_anim_set_exec_cb(&a, top_logo_tx_anim_exec);
    lv_anim_set_values(&a, cur, 0);
    lv_anim_set_time(&a, 200); /* =hw_pull_snap 縮回同步 */
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}
static bool s_top_fly_active = false;
static lv_point_t s_top_press_start;

static void top_hold_cancel(void)
{
    if (s_top_hold_timer)
    {
        lv_timer_del(s_top_hold_timer);
        s_top_hold_timer = NULL;
    }
}

/* 結束飛鼠（如果在飛）並還原 press 鎖。冪等。 */
static void top_fly_end(const char *why)
{
    if (s_top_fly_active)
    {
        s_top_fly_active = false;
        LOG_I("[logo-fly] top %s -> handfree OFF", why);
        set_hid_mouse_handfree_mode_to(false);
    }
    if (status_bar_area_up && lv_obj_is_valid(status_bar_area_up))
        lv_obj_clear_flag(status_bar_area_up, LV_OBJ_FLAG_PRESS_LOCK);
}

/* 頂部區按住＝飛鼠（明確要飛鼠）時 true；供 air_mouse_process 判斷要不要忽略姿態 switch：
   頂部飛鼠忽略姿態（傾斜手腕不該被擋），但 FSR 壓感觸發的 handfree（bloc_control.c，手臂/
   袖子壓到手錶）保留姿態保護，免得舉手腕時誤壓 FSR 就亂飛（founder 2026-07-16）。 */
bool hid_mouse_top_fly_active(void)
{
    return s_top_fly_active;
}

static void top_hold_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_top_hold_timer = NULL; // repeat_count=1 跑完自刪，只清指標
    s_top_fly_active = true;
    // 鎖住 press：飛鼠期間手指移動不觸發重新 hit-test（會 PRESS_LOST 誤關）
    if (status_bar_area_up && lv_obj_is_valid(status_bar_area_up))
        lv_obj_add_flag(status_bar_area_up, LV_OBJ_FLAG_PRESS_LOCK);
    LOG_I("[logo-fly] top hold %dms -> handfree ON", TOP_HOLD_TO_FLY_MS);
    set_hid_mouse_handfree_mode_to(true);
}

/* 底部 keyboard_icon 的 tap = 進輸入(語音站)。與舊的頂部 tap 同一條路徑、
   同一個離線保護。 */
/* 輸入模式由下往上進場:整個 mode container 從 +LV_VER_RES 滑到 0。
   **不能**改用 start_trackpad_to_kbd_expand_anim() —— 那條是舊版語音輸入畫面
   (輸入框 380×90 停在 y195 + kbd_mic_section)的進場動畫,接回去會把整個版面
   一起換成舊的(founder 2026-08-07:「為什麼打開的畫面整個不一樣了」)。這裡只加
   位移,落地狀態完全交給 mouse_open_voice_station()。 */
static void kbd_enter_slide_exec(void *var, int32_t v)
{
    lv_obj_t *c = (lv_obj_t *)var;
    if (c && lv_obj_is_valid(c))
        lv_obj_set_y(c, (lv_coord_t)v);
}

static void kbd_enter_slide_done(lv_anim_t *a)
{
    (void)a;
    /* 滑上來的途中底下要看得到觸控板(同退出動畫的做法),到位才收掉。 */
    mode_set_visible(HID_MODE_TRACKPAD, false);
    /* 抽屜的地球鍵:進場動畫落地**之後**才切去鍵盤輪盤。不在進場當下切的原因有二:
       ① mouse_open_voice_station() 末端本來就跑著 kbd_lower_switch(false) 的換站動畫,
          同一輪再切一次等於兩條動畫互打;
       ② 圓形鍵盤是延遲建的(s_kbd_build_defer_wheel),補建走的正是 kbd_lower_switch(true)
          的 lazy 分支 —— 用 set_keyboard 只翻可見性會得到一片空的鍵盤。 */
    if (s_kbd_pending_wheel)
    {
        s_kbd_pending_wheel = false;
        kbd_lower_switch(true);
    }
}

/* 進場的上滑動畫改用 async 起跑:mouse_open_voice_station() 會整個重建鍵盤版面
   (create_circular_keyboard_layout),那一下把 LVGL thread 卡住數百 ms;若在同一輪
   lv_timer_handler 內建動畫,anim timer 這一輪算出的 elapsed 就含那塊停頓,第一步
   直接跳到終點 = 看起來完全沒有從下面上來(founder 2026-08-07)。丟到下一輪才建,
   那塊 tick 已被吃掉,動畫從乾淨的時間基準起跑。 */
static void kbd_enter_slide_async(void *unused)
{
    (void)unused;
    lv_obj_t *kc = mode_container[HID_MODE_KEYBOARD];
    if (kc == NULL || !lv_obj_is_valid(kc))
        return;
    lv_anim_del(kc, kbd_enter_slide_exec);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, kc);
    lv_anim_set_values(&a, LV_VER_RES, 0);
    lv_anim_set_time(&a, 260);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, kbd_enter_slide_exec);
    lv_anim_set_ready_cb(&a, kbd_enter_slide_done);
    lv_anim_start(&a);
}

/* 2026-08-15:電腦「沒有」聚焦輸入框時,底部 bar 改開「這台電腦」的搜尋抽屜 ——
   0x0E 讓電腦把 skaibar 叫出來(預設 Sessions 檢視=這台電腦的 session 列表),電腦面板的
   可見列表經 deviceActions(0x03)鏡像回手錶浮層清單,底部浮層 mic 一點就開語音輸入框
   (V2T_INTENT_SKAIBAR;手機在單設備模式下把轉錄同步 setSkaibarText 打進電腦面板,電腦
   即時搜 sessions+actions+檔案,結果再鏡像回來 → 兩邊同步)。捲動清單=收框留選項+停語音
   (既有 ai_widget_fade_on_scroll);點選項=commit 給電腦執行(既有路徑)。
   direct_voice=true(長按)略過 browse 直接開語音框。回傳 false=沒有控制目標,呼叫端
   fallback 語音站,tap 不落空。 */
static bool mouse_open_device_search(bool direct_voice)
{
    extern bool instruction_list_prepare_single_device(const char *device_id);
    extern void instruction_list_bar_set_visible(bool visible);
    extern void instruction_list_open_browse(void);
    extern void animate_open_ai_widget(void);
    extern bool commu_send_skaibar_open_device(bool force_open);
    if (!instruction_list_prepare_single_device(s_dev_active_id))
        return false;
    /* 抽屜是**新鮮進場**:輸入列要是空的。不清的話,上一輪
       skaibar_apply_selection_to_input_bar() 寫進 input_buffer 的那個「被點到的選項」
       會一路留著 —— 症狀就是「點過選項進 session、離開再打開,輸入框還是上次的東西」
       (founder 2026-08-17)。放在 prepare 成功之後:開不起來就不該動使用者的文字。 */
    clear_input_display();
    commu_send_skaibar_open_device(true);
    instruction_list_bar_set_visible(true);
    if (direct_voice)
        animate_open_ai_widget();
    else
        instruction_list_open_browse();
    return true;
}

/* ==========================================================================
   抽屜 → 語音站(founder 2026-08-17)
   --------------------------------------------------------------------------
   抽屜底部那排(地球/麥克風/收下)由 lv_instruction_list_layout 畫,座標與語音站的
   kbd_mic_section 逐格對齊。使用者按下抽屜的麥克風時:
     ① 進語音站(mouse_open_voice_station,落地版面一個像素都不改)
     ② 整個 keyboard mode container 由下往上滑進來(既有 kbd_enter_slide)
     ③ 同時把 session 清單往**上**推出畫面(instruction_list_drawer_push_up)
   抽屜那排在轉場期間留在原位(它在 layer_top、蓋在最上面),等語音站自己那排滑到
   同一格才被 push_up 的 ready_cb 收掉 —— 視覺上「那排完全不動,只有輸入框往上長、
   清單被推出去」。

   s_kbd_from_drawer 記住「這次語音站是從抽屜進來的」:輸入框往下拖收合時不是回到
   光禿禿的觸控板,而是**滑回抽屜**(帶著這輪語音搜尋後的最新選項)。要真的回觸控板
   走抽屜那排的收下鍵,或語音站自己的右上退出鈕。
   ========================================================================== */
bool mouse_drawer_open_input(bool want_keyboard)
{
    if (current_hid_mode == HID_MODE_KEYBOARD)
        return false; /* 已經在輸入畫面(重複事件) */
    /* 容器先驗、再切模式 —— 反過來的話這個 early-return 會把 current_hid_mode 留在
       KEYBOARD 卻沒有任何鍵盤畫面,之後每次點 bar 都被「已經在輸入畫面」擋掉。 */
    lv_obj_t *kc = mode_container[HID_MODE_KEYBOARD];
    if (kc == NULL || !lv_obj_is_valid(kc))
        return false;
    /* 先立旗再進場:mouse_open_voice_station() 內部就會送 0x0E 給電腦,而 inputOnly
       要靠這面旗決定(抽屜流程=要選項)。 */
    s_kbd_from_drawer = true;
    s_kbd_direct_field = false; /* 抽屜流程要的是選項,不是打進欄位 */
    if (!mouse_open_voice_station())
    {
        s_kbd_from_drawer = false;
        return false;
    }
    mode_set_visible(HID_MODE_TRACKPAD, true); /* 滑上來時底下露觸控板 */
    lv_anim_del(kc, kbd_enter_slide_exec);
    lv_obj_set_y(kc, LV_VER_RES); /* 起點立刻就位,動畫下一輪才建(見 kbd_enter_slide_async) */
    lv_async_call(kbd_enter_slide_async, NULL);
    {
        extern void instruction_list_drawer_push_up(void);
        instruction_list_drawer_push_up();
    }
    s_kbd_pending_wheel = want_keyboard; /* 地球 = 落地後接著切到鍵盤輪盤 */
    LOG_I("[drawer] -> voice station (kbd=%d)", (int)want_keyboard);
    return true;
}

/* 抽屜麥克風的按住/放開直接驅動語音站同一套後端。
   **intent 必須用語音站原生的 MIC_INPUTE,不能用 SKAIBAR**(founder 2026-08-17:
   「按麥克風說話他沒有文字回來了」)—— interact_voice_recognition() 的路由鏈裡
   `check_if_user_speaking_to_ai()` 排在滑鼠分支**前面**,而 SKAIBAR intent 會讓 VAD
   把 speaking_to_ai 立起來,轉錄於是被上游分支接走、送進 append_text_to_input_message()
   (=instruction_list 的 AI widget 輸入框)。但這條流程裡那個 widget 已經被推走關掉了,
   文字就沒有任何地方顯示。走 MIC_INPUTE 才會落到滑鼠分支的 append_text_to_mouse_input()
   =語音站自己的輸入框。
   **intent 仍必須是 SKAIBAR**:手錶沒有任何「送 skaibar 查詢文字」的 wire,唯一能讓
   電腦真的去搜的路徑就是這個 intent —— 轉錄經手機 routeSkaibarTranscript → setSkaibarText
   → 電腦搜 sessions/actions/檔案 → 結果經 0x03 鏡像回手錶。語音站自己那條
   commu_send_voice_station_preview() 走的是 KEY_LIFT_INPUT_CARET(0x1e){"preview":...},
   那是**純顯示**的推播,電腦看得到字但不會搜(founder 2026-08-17:「電腦上的 SKAIBAR
   也有文字輸入,但為什麼上面沒有選項」)。
   而「SKAIBAR intent 會害手錶看不到字」的老問題已在 interact_voice_recognition() 那層
   根治(mouse_voice_owns_transcript),不必再靠換 intent 迴避。 */
void mouse_drawer_voice_set(bool on)
{
    if (on)
    {
        if (!mouse_v2t_active)
            mouse_v2t_open_with_intent(V2T_INTENT_SKAIBAR);
    }
    else if (mouse_v2t_active)
    {
        mouse_v2t_close_and_paste();
    }
}

/* 長按放開後 LVGL 還會補一顆 CLICKED —— 這面旗標讓 bottom_logo_cb 把它吃掉
   (同 mic_bar 的 s_mic_lp_consumed pattern)。 */
static bool s_logo_lp_consumed = false;

static void bottom_logo_long_press_cb(lv_event_t *e)
{
    (void)e;
    /* 拖曳換設備進行中:LVGL 仍會在 400ms 到點時補一顆 LONG_PRESSED,不能讓它把
       語音站叫起來蓋掉正在跑的切換手勢。 */
    if (devbar_drag_active())
        return;
    if (dev_active_offline())
        return;
    if (current_hid_mode == HID_MODE_KEYBOARD)
        return;
    if (mouse_open_device_search(true)) /* 長按=直接進語音輸入 */
        s_logo_lp_consumed = true;
}

static void bottom_logo_cb(lv_event_t *e)
{
    (void)e;
    /* 「點了沒反應」這類回報最花時間的就是分不出**事件沒進來**和**進來了走錯分支**
       (同 mic_bar_event_cb 的 [mic] tap 探針)。這一行印出來 = 事件確實到了滑鼠 app;
       完全沒有這行 = 有東西在上面把 tap 吃掉了(歷史上是 layer_top 那片看不見的
       mic_hit,見 lv_instruction_list_layout.c 的 mic_hit_follow_bar)。 */
    LOG_W("[logo] tap offline=%d mode=%d lp=%d", (int)dev_active_offline(),
          (int)current_hid_mode, (int)s_logo_lp_consumed);
    /* 這一次按壓是左右拖曳(換設備)→ 放開時 LVGL 照樣補一顆 CLICKED,不能當成 tap
       去開抽屜(founder 2026-09-04 的拖曳手勢與既有的 tap 共用同一個物件)。 */
    if (devbar_consume_click())
        return;
    if (dev_active_offline())
        return;
    if (current_hid_mode == HID_MODE_KEYBOARD)
        return; /* 已經在輸入畫面 */
    if (s_logo_lp_consumed)
    {
        s_logo_lp_consumed = false; /* 長按同一次按壓的放開,不再疊一次 tap 行為 */
        return;
    }
    /* founder 2026-08-17:**一律**開 session/搜尋抽屜 —— 不再看 0x17 聚焦旗標分流。
       「打字給電腦聚焦中的那個輸入框」整條移到觸控板右緣新的鍵盤鈕(kbd_side_btn),
       語意才乾淨:bar = 找東西、右緣鍵盤鈕 = 打字。抽屜自己的底部三鍵列裡就有麥克風
       與地球,語音/鍵盤兩站從抽屜都進得去。
       沒有控制目標時 fallback 語音站,維持 tap 必有反應。 */
    if (mouse_open_device_search(false))
        return;
    /* founder 2026-08-11 R8:底部這張圖(hosted 顯示 skaibar_img)tap = 開**原本
       按鍵盤那個輸入模式** —— 只換圖,行為不變(R7 一度改成開 session,改回)。 */
    /* 2026-08-07 founder:「叫出輸入模式時可以看到他從下面出來嗎」——可以,
       start_trackpad_to_kbd_expand_anim() 這條進場動畫本來就寫好了(輸入框從
       底部那條 176×31 的 bar 長出來、下半部 translate_y 由下往上、350ms
       overshoot),只是入口幾次搬家後失去呼叫者,變成 mouse_open_voice_station()
       直接瞬間切過去。這裡接回它。
       先重建鍵盤版面(回英文/清注音緩衝),再起動畫 —— 展開動畫本身會把
       keyboard_container 藏起來先進語音站,所以重建不會被看到。 */
    /* 這裡**不要**再重建一次鍵盤版面 —— mouse_open_voice_station() 內部已經做了。
       重複一次等於白付一次數百 ms 的停頓,正是動畫被吃掉的主因。 */
    mouse_open_input_station(false);
}

/* 進「輸入模式」的共同入口:底部 bar 沒有控制目標時的 fallback、以及觸控板右緣
   鍵盤鈕(電腦有聚焦輸入框時)。落地狀態=現行語音站版面,一個像素都不動。
   direct_field=true(右緣鍵盤鈕)→ 電腦不開 skaibar,字直接進使用者點的那個輸入框。 */
static void mouse_open_input_station(bool direct_field)
{
    s_kbd_from_drawer = false; /* 這兩個入口都不是抽屜流程:電腦只出輸入框 */
    s_kbd_direct_field = direct_field;
    if (!mouse_open_voice_station())
        return; /* 被拒:留在觸控板,別跑進場動畫(容器沒切) */
    /* 同抽屜:從觸控板/底部 bar 新鮮進來的輸入站也要是空的。刻意**不**放進
       mouse_open_voice_station() —— 抽屜的麥克風那條(9949)也走它,而那是站內轉場,
       清掉會把使用者正在編輯的字吃掉。 */
    clear_input_display();
    lv_obj_t *kc = mode_container[HID_MODE_KEYBOARD];
    if (kc == NULL || !lv_obj_is_valid(kc))
        return;
    mode_set_visible(HID_MODE_TRACKPAD, true); /* 滑上來時底下露觸控板 */
    /* 起點立刻就位(不能等 async,否則會先閃一格在定位上),動畫下一輪才建 */
    lv_anim_del(kc, kbd_enter_slide_exec);
    lv_obj_set_y(kc, LV_VER_RES);
    lv_async_call(kbd_enter_slide_async, NULL);
}

/* 右緣鍵盤鈕:電腦上正聚焦著某個輸入框 → 直接開輸入頁打進去(icon_send 由語音站
   自己依同一個 0x17 旗標決定要不要出現,這裡不必再判一次)。 */
static void kbd_side_btn_event_cb(lv_event_t *e)
{
    (void)e;
    if (dev_active_offline())
        return;
    if (current_hid_mode == HID_MODE_KEYBOARD)
        return;
    /* 這顆鈕只在電腦有聚焦輸入框時才浮現 → 目的地已經指定,走直打。 */
    mouse_open_input_station(true);
}

/**
 * @brief status_bar_area_up 事件 cb（仿 app_clock_status_bar 的
 *        notification_status_bar_cb）
 *        - PRESSED：把 tileview 顯示出來、tile 設成 home (0,1)
 *          這樣使用者後續拖曳時 LVGL 會把 press 轉給 tileview，
 *          由 tileview 原生處理拖曳/snap/動畫；同時開 hold 計時器
 *        - PRESSING：位移累積過大 → 取消 hold（拖曳意圖，走媒體）
 *        - PRESS_LOST：press 被 tileview 接走 → 取消 hold / 結束飛鼠
 *        - RELEASED：只在 press 沒被 tileview 接走時才會 fire
 *          → tap 或飛鼠放開：收掉 tileview + 結束飛鼠
 */
static void status_bar_area_up_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED)
    {
        /* 這裡「不能」先亮 media tileview —— LVGL 無 PRESS_LOCK 時每 tick
           重新 hit-test，全螢幕 tileview 一亮 press 下一 tick 就被它搶走
           (PRESS_LOST)，hold 永遠 fire 不了（2026-07-06 真機踩過）。
           tileview 延到 PRESSING 確認拖曳意圖時才亮。 */
        {
            lv_indev_t *indev = lv_indev_get_act();
            if (indev)
                lv_indev_get_point(indev, &s_top_press_start);
        }
        top_hold_cancel();
        s_top_hold_timer = lv_timer_create(top_hold_timer_cb,
                                           TOP_HOLD_TO_FLY_MS, NULL);
        lv_timer_set_repeat_count(s_top_hold_timer, 1);
    }
    else if (code == LV_EVENT_PRESSING)
    {
        /* 圖示左拉跟手中:手寫頁 x=右緣+dx、滑鼠圖 translate=dx(founder
           2026-07-22:滑鼠圖左滑走+鍵盤頁從右進)。 */
        if (s_top_hw_pull)
        {
            lv_indev_t *indev = lv_indev_get_act();
            if (indev)
            {
                lv_point_t now;
                lv_indev_get_point(indev, &now);
                lv_coord_t dx = now.x - s_top_press_start.x;
                if (dx > 0)
                    dx = 0;
                s_top_hw_vx = now.x - s_top_hw_last_x;
                s_top_hw_last_x = now.x;
                if (s_hw_view)
                {
                    lv_coord_t x = (lv_coord_t)(LV_HOR_RES + dx);
                    if (x < 0)
                        x = 0;
                    lv_obj_set_x(s_hw_view, x);
                    hw_backdrop_sync(x); /* 黑底跟手漸黑 */
                }
                if (s_top_logo && lv_obj_is_valid(s_top_logo))
                    lv_obj_set_style_translate_x(s_top_logo, dx, 0);
            }
        }
        else if (s_top_hold_timer)
        {
            lv_indev_t *indev = lv_indev_get_act();
            if (indev)
            {
                lv_point_t now;
                lv_indev_get_point(indev, &now);
                lv_coord_t dx = now.x - s_top_press_start.x;
                lv_coord_t dy = now.y - s_top_press_start.y;
                if (LV_ABS(dx) + LV_ABS(dy) > TOP_HOLD_DRIFT_CANCEL_PX)
                {
                    top_hold_cancel();
                    /* 水平左拉主導=手寫頁跟手進場;**明確下拉主導**=媒體下拉;
                       其他方向(斜向/上向)=手滑,不觸發任何層(founder 2026-07-22:
                       tap 手一晃就跳媒體層擋住滑鼠圖)。斷線時只留媒體下拉。 */
                    if (dx < 0 && LV_ABS(dx) > LV_ABS(dy) &&
                        !dev_active_offline() && hw_view_stage_offscreen())
                    {
                        s_top_hw_pull = true;
                        s_top_hw_last_x = now.x;
                        s_top_hw_vx = 0;
                        /* 跟手期間鎖住 press:此區平常故意不鎖(媒體下拉要讓
                           tileview 接手),但左拉時手指滑出 80px 帶會 PRESS_LOST
                           →縮回(founder:「有移動但滑不進來」)。放開時解鎖。 */
                        lv_obj_add_flag(status_bar_area_up,
                                        LV_OBJ_FLAG_PRESS_LOCK);
                    }
                    else if (dy > 0 && dy > LV_ABS(dx) && s_pulldown_cb)
                    {
                        /* 面板 host 模式：明確往下拉 → 亮出錶盤頂部面板，press
                           下一 tick 被它的 tileview 接走（同一機制，只是換去處）*/
                        s_pulldown_cb();
                    }
                    else if (dy > 0 && dy > LV_ABS(dx) && media_tileview &&
                             lv_obj_is_valid(media_tileview))
                    {
                        // 明確往下拉 → 媒體下拉：這時才亮 tileview，
                        // press 下一 tick 被它接走（原機制）
                        lv_obj_set_tile_id(media_tileview, 0, 1, false);
                        lv_obj_clear_flag(media_tileview, LV_OBJ_FLAG_HIDDEN);
                        lv_obj_move_foreground(media_tileview);
                    }
                }
            }
        }
    }
    else if (code == LV_EVENT_PRESS_LOST)
    {
        if (s_top_hw_pull)
        {
            /* 左拉中被搶:縮回不 commit,滑鼠圖動畫滑回+解鎖 press */
            s_top_hw_pull = false;
            if (status_bar_area_up && lv_obj_is_valid(status_bar_area_up))
                lv_obj_clear_flag(status_bar_area_up, LV_OBJ_FLAG_PRESS_LOCK);
            top_logo_slide_home();
            hw_pull_snap(false);
        }
        // press 被 tileview 接走（媒體下拉路徑）
        top_hold_cancel();
        top_fly_end("PRESS_LOST");
    }
    else if (code == LV_EVENT_RELEASED)
    {
        if (s_top_hw_pull)
        {
            /* 左拉放開:過 1/4 或快甩=commit 開手寫,否則縮回。commit 時滑鼠圖
               **不在這裡歸位**——頁面還沒蓋滿,立刻歸位會閃一下跳回(founder
               2026-07-22);等 hw_open_commit(蓋滿)才歸位。取消=動畫滑回。 */
            s_top_hw_pull = false;
            if (status_bar_area_up && lv_obj_is_valid(status_bar_area_up))
                lv_obj_clear_flag(status_bar_area_up, LV_OBJ_FLAG_PRESS_LOCK);
            bool commit = false;
            if (s_hw_view)
                commit = (lv_obj_get_x(s_hw_view) <= (LV_HOR_RES * 3) / 4) ||
                         (s_top_hw_vx <= -6);
            if (commit)
            {
                /* 快甩早放:圖不能凍在放開位(founder 2026-07-22)——跟進場頁
                   同步 200ms 繼續滑出左緣;hw_open_commit 蓋滿後歸位。 */
                if (s_top_logo && lv_obj_is_valid(s_top_logo))
                {
                    lv_anim_del(s_top_logo, NULL);
                    lv_anim_t la;
                    lv_anim_init(&la);
                    lv_anim_set_var(&la, s_top_logo);
                    lv_anim_set_exec_cb(&la, top_logo_tx_anim_exec);
                    lv_anim_set_values(
                        &la,
                        lv_obj_get_style_translate_x(s_top_logo, LV_PART_MAIN),
                        (int32_t)-(int32_t)LV_HOR_RES);
                    lv_anim_set_time(&la, 200);
                    lv_anim_set_path_cb(&la, lv_anim_path_ease_out);
                    lv_anim_start(&la);
                }
            }
            else
            {
                top_logo_slide_home();
            }
            hw_pull_snap(commit);
            top_hold_cancel();
            return;
        }
        /* hold timer 還活著=沒進飛鼠也沒拖=tap(飛鼠 timer 已 fire=NULL、
           拖曳意圖時已 cancel=NULL)。 */
        bool was_tap = (s_top_hold_timer != NULL);
        top_hold_cancel();
        top_fly_end("RELEASED");
        // tap（沒拖沒 hold）或飛鼠放開 → 收 tileview
        if (media_tileview && lv_obj_is_valid(media_tileview))
        {
            lv_obj_add_flag(media_tileview, LV_OBJ_FLAG_HIDDEN);
        }
        /* 2026-08-07:進輸入的 tap 搬到底部的 keyboard_icon(bottom_logo_cb)，
           頂部這塊只留「下拉開面板 / 按住進飛鼠」。was_tap 保留給上面的
           top_fly_end 判斷用。 */
        (void)was_tap;
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
    /* This is the phone's OWN media (NOTIFY 0x46). When a remote target is the
       active selection, the media centre shows that device's now-playing instead
       (mouse_mode_handle_remote_media_state / 0x19); ignore the phone's own here. */
    if (ble_hid_mouse_app_route())
        return;
    lv_label_set_text(media_center_title_label,
                      (title && title[0]) ? title : "Media Title");
}

/* SKAI_LINK 0x19 downlink: the ACTIVE remote target device's now-playing, routed
   from communicate_parse_skailink.c::handle_media_state. Filtered by device_id
   against the current active selection so a late frame for a just-deselected
   device can't clobber the UI. Drives ONLY the mouse app's media centre — the
   watch-face media widget keeps using the phone's own 0x46. */
void mouse_mode_handle_remote_media_state(const char *device_id, const char *title,
                                          const char *artist, bool playing)
{
    (void)artist; /* media centre shows title only for now */
    if (!media_center_title_label || !lv_obj_is_valid(media_center_title_label))
        return;
    if (device_id == NULL || s_dev_active_id[0] == '\0' ||
        strncmp(device_id, s_dev_active_id, SYNCED_DEVICE_ID_LEN) != 0)
        return;
    lv_label_set_text(media_center_title_label,
                      (title && title[0]) ? title : "Media Title");
    media_center_update_play_icon(playing);
}

void mouse_mode_handle_media_play_state(bool playing)
{
    media_center_update_play_icon(playing);
}

/**
 * @brief Creates the mouse screen
 * @param scr Screen object
 */
/* === 頂部設備切換器 =========================================================
   取代舊的右側設備清單抽屜：trackpad 頂部設備名 label 兩側各一個箭頭，點擊切到
   相鄰設備（循環）。選了之後一樣走手機 relay（commu_send_active_device +
   ble_hid_mouse_set_app_route）。設備資料來自 E7 device_registry。 */

/* 在 registry 找目前 s_dev_active_id 的 index；不在/未選回 -1。 */
static int active_device_index(void)
{
    if (s_dev_active_id[0] == '\0')
        return -1;
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES)
        n = MAX_SYNCED_DEVICES;
    for (uint8_t i = 0; i < n; i++)
        if (strncmp((const char *)SkaiWatchSys.device_registry.devices[i].id,
                    s_dev_active_id, SYNCED_DEVICE_ID_LEN) == 0)
            return (int)i;
    return -1;
}

/* 進入滑鼠頁的預設設備 index：主要(status 2) → 第一個在線(1) → 清單第一個；
   registry 空回 -1。 */
static int pick_default_device(void)
{
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES)
        n = MAX_SYNCED_DEVICES;
    if (n == 0)
        return -1;
    int first_online = -1;
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t st = SkaiWatchSys.device_status[i];
        if (st == 2)
            return (int)i;
        if (st == 1 && first_online < 0)
            first_online = (int)i;
    }
    return (first_online >= 0) ? first_online : 0;
}

/* 把第 idx 台設為 active：更新 id、走手機 relay 控制、刷新頂部名稱。idx 無效則忽略。 */
static void set_active_device_by_index(int idx)
{
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES)
        n = MAX_SYNCED_DEVICES;
    if (idx < 0 || idx >= (int)n)
        return;
    const char *id = (const char *)SkaiWatchSys.device_registry.devices[idx].id;
    if (!id || !id[0])
        return;
    commu_send_active_device(id);
    ble_hid_mouse_set_app_route(true);
    strncpy(s_dev_active_id, id, sizeof(s_dev_active_id) - 1);
    s_dev_active_id[sizeof(s_dev_active_id) - 1] = '\0';
    update_ctrl_dev_label();
    devbar_update_name(); /* 底部設備 bar 跟著換名 */
    /* Switched target: reset the media title to the placeholder until this
       device's now-playing arrives (the phone re-pushes 0x19 on active-select),
       so the previous device's title doesn't linger as if it were this one's. */
    if (media_center_title_label && lv_obj_is_valid(media_center_title_label))
        lv_label_set_text(media_center_title_label, "Media Title");
    LOG_I("[dev_switch] active -> idx=%d/%d id=%s", idx, (int)n, id);
}

/* 切到相鄰設備（dir=-1 上一台 / +1 下一台），到頭循環。<2 台則無動作。
   離線設備**照樣可切**(founder 2026-07-22 定案):切到離線台時由
   dev_offline_overlay_sync 顯示灰版+「斷線」,而非從循環裡藏掉。 */
static void switch_active_device(int dir)
{
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES)
        n = MAX_SYNCED_DEVICES;
    if (n < 2)
        return;
    int cur = active_device_index();
    if (cur < 0)
        cur = 0;
    int next = (cur + dir + (int)n) % (int)n;
    set_active_device_by_index(next);
}

static void dev_arrow_prev_cb(lv_event_t *e)
{
    (void)e;
    switch_active_device(-1);
}

static void dev_arrow_next_cb(lv_event_t *e)
{
    (void)e;
    switch_active_device(+1);
}

/* === 對外的「控制中設備」API（錶盤頂部面板 lv_top_panel.c 用）====================
   面板頂部有一份自己的設備名 + 箭頭，但**選台的真相**還是這裡（registry index +
   commu_send_active_device + app_route），面板只是另一個驅動它的 UI。不另開一份
   state，避免面板選 A、滑鼠控 B。 */
int hid_mouse_device_count(void)
{
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES) n = MAX_SYNCED_DEVICES;
    return (int)n;
}

int hid_mouse_active_device_index(void) { return active_device_index(); }

void hid_mouse_set_active_device_index(int idx) { set_active_device_by_index(idx); }

void hid_mouse_switch_active_device(int dir) { switch_active_device(dir); }

/* 沒有有效的控制目標時挑一台預設（主要 → 第一個在線 → 清單第一個）。與獨立
   滑鼠 app 進場時的邏輯同一份，讓頂部面板一拉開就有「控制中設備」可顯示。
   registry 空 → 什麼都不做（不去動既有 route）。 */
/* 把控制目標退回「當前連線的手機」：清掉遠端選台、通知手機取消 active、
   route 關掉 → 滑鼠/媒體都回到 BLE HID 直連手機本身。頂部面板停在通知列表
   （非媒體頁）時走這條（founder 2026-08-06:「通知列表的時候上面的要是控制
   當前連線的手機」）。 */
void hid_mouse_clear_active_device(void)
{
    if (s_dev_active_id[0] == '\0')
        return;
    s_dev_active_id[0] = '\0';
    LOG_W("[active] mouse clear");
    commu_send_active_device("");
    ble_hid_mouse_set_app_route(false);
    LOG_I("[dev_switch] active -> phone (cleared)");
}

static const char *active_device_name(void); /* 定義在下方 */
void hid_mouse_ensure_active_device(void)
{
    if (active_device_name() != NULL)
        return;
    int def = pick_default_device();
    if (def >= 0)
        set_active_device_by_index(def);
}

const char *hid_mouse_device_name(int idx)
{
    if (idx < 0 || idx >= hid_mouse_device_count()) return NULL;
    return (const char *)SkaiWatchSys.device_name[idx];
}

bool hid_mouse_device_online(int idx)
{
    if (idx < 0 || idx >= hid_mouse_device_count()) return false;
    return SkaiWatchSys.device_status[idx] != 0;
}

const char *hid_mouse_device_id(int idx)
{
    if (idx < 0 || idx >= hid_mouse_device_count()) return NULL;
    const char *id = SkaiWatchSys.device_registry.devices[idx].id;
    return (id[0] != '\0') ? id : NULL;
}

/* 頂部下拉的去處覆寫：面板 host 滑鼠模式時，頂部往下拉要拉出「錶盤頂部面板」，
   而不是滑鼠 app 自己那層媒體 tileview（founder 2026-08-06：APP 內上方的媒體
   中心可以不要）。NULL = 維持原本的自有媒體下拉（獨立開 APP_ID_MOUSE 時）。 */
void hid_mouse_set_pulldown_cb(void (*cb)(void)) { s_pulldown_cb = cb; }

/* active 設備是否斷線(id 不在 registry=已移除,視同斷線)。 */
static bool dev_active_offline(void)
{
    if (s_dev_active_id[0] == '\0')
        return false;
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES)
        n = MAX_SYNCED_DEVICES;
    for (uint8_t i = 0; i < n; i++)
    {
        if (strncmp((const char *)SkaiWatchSys.device_registry.devices[i].id,
                    s_dev_active_id, SYNCED_DEVICE_ID_LEN) == 0)
            return (SkaiWatchSys.device_status[i] == 0);
    }
    return true;
}

/* active 設備斷線=觸碰板區(y80 以下)灰+中央「斷線」(founder 2026-07-22 二改)。
   overlay 吃 press=擋觸碰板/bar/滾動弧;頂部 80px 不蓋=媒體下拉照常(其 tap/
   hold/左拉手勢另外 gate,只留下拉)。media tileview 開啟時 foreground 蓋過
   overlay=面板內切設備可操作。同函式順路管媒體頁設備名上方的在線燈號。 */
static void dev_offline_overlay_sync(void)
{
    bool offline = dev_active_offline();
    /* 燈號:綠=在線/紅=斷線;無 active 設備=藏 */
    if (s_dev_status_dot && lv_obj_is_valid(s_dev_status_dot))
    {
        if (s_dev_active_id[0] == '\0')
        {
            lv_obj_add_flag(s_dev_status_dot, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_set_style_bg_color(s_dev_status_dot,
                                      offline ? lv_color_hex(0xE05A5A)
                                              : lv_color_hex(0x4CAF50),
                                      0);
            lv_obj_clear_flag(s_dev_status_dot, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (!offline)
    {
        if (s_dev_offline_overlay && lv_obj_is_valid(s_dev_offline_overlay))
            lv_obj_add_flag(s_dev_offline_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (s_dev_offline_overlay == NULL ||
        !lv_obj_is_valid(s_dev_offline_overlay))
    {
        /* 掛媒體面板同一 parent:不同層的 move_foreground 拉不過彼此,面板
           會被 overlay 永遠壓住(founder 2026-07-22:媒體中心切到斷線設備
           OFFLINE 蓋在面板上)。 */
        lv_obj_t *host = NULL;
        if (media_tileview && lv_obj_is_valid(media_tileview))
            host = lv_obj_get_parent(media_tileview);
        if (host == NULL)
            host = hid_mouse_ui_host();
        if (host == NULL)
            host = lv_scr_act();
        s_dev_offline_overlay = lv_obj_create(host);
        lv_obj_remove_style_all(s_dev_offline_overlay);
        lv_obj_set_size(s_dev_offline_overlay, LV_HOR_RES, LV_VER_RES - 80);
        lv_obj_set_pos(s_dev_offline_overlay, 0, 80);
        lv_obj_set_style_bg_color(s_dev_offline_overlay, lv_color_hex(0x666666),
                                  0);
        lv_obj_set_style_bg_opa(s_dev_offline_overlay, LV_OPA_60, 0);
        lv_obj_add_flag(s_dev_offline_overlay, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(s_dev_offline_overlay, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *lbl = lv_label_create(s_dev_offline_overlay);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        lv_label_set_text(lbl,
                          LV_EXT_STR_GET_BY_KEY(device_offline, "Offline"));
        /* 螢幕正中(overlay 從 y80 起,中心在 y273,上移 40 補回 y233) */
        lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -40);
    }
    if (lv_obj_has_flag(s_dev_offline_overlay, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_clear_flag(s_dev_offline_overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_dev_offline_overlay);
    }
    /* OFFLINE 恆在媒體面板**下層**(founder 2026-07-22 定案:面板開著時被遮、
       拉上去才看到;同 parent 才拉得動)。每 tick 保證,面板中途開也不會被壓。 */
    if (media_tileview && lv_obj_is_valid(media_tileview) &&
        !lv_obj_has_flag(media_tileview, LV_OBJ_FLAG_HIDDEN))
        lv_obj_move_foreground(media_tileview);
}

/* 由 s_dev_active_id 在 E7 registry 反查目前控制設備的名稱；id 不在 registry(已移除/
   尚未同步) → NULL。registry 可能重排，故用 id 當穩定鍵。 */
static const char *active_device_name(void)
{
    if (s_dev_active_id[0] == '\0')
        return NULL;
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES)
        n = MAX_SYNCED_DEVICES;
    for (uint8_t i = 0; i < n; i++)
        if (strncmp((const char *)SkaiWatchSys.device_registry.devices[i].id,
                    s_dev_active_id, SYNCED_DEVICE_ID_LEN) == 0)
            return (const char *)SkaiWatchSys.device_name[i];
    return NULL;
}

/* 更新 trackpad 頂部「控制中設備」標籤：有控制中設備就顯示其名稱、否則隱藏。 */
static void update_ctrl_dev_label(void)
{
    if (!s_ctrl_dev_label || !lv_obj_is_valid(s_ctrl_dev_label))
        return;
    const char *name = active_device_name();
    if (name && name[0])
    {
        lv_label_set_text(s_ctrl_dev_label, name);
        lv_obj_clear_flag(s_ctrl_dev_label, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(s_ctrl_dev_label, LV_OBJ_FLAG_HIDDEN);
    }
    /* 箭頭：有設備就顯示（1 台時 inert，無別台可切）；無設備才隱藏 */
    {
        uint8_t cnt = SkaiWatchSys.device_registry.count;
        if (cnt > MAX_SYNCED_DEVICES)
            cnt = MAX_SYNCED_DEVICES;
        bool show_arrows = (cnt >= 1);
        if (s_dev_left_arrow && lv_obj_is_valid(s_dev_left_arrow))
        {
            if (show_arrows)
                lv_obj_clear_flag(s_dev_left_arrow, LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(s_dev_left_arrow, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_dev_right_arrow && lv_obj_is_valid(s_dev_right_arrow))
        {
            if (show_arrows)
                lv_obj_clear_flag(s_dev_right_arrow, LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(s_dev_right_arrow, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* === 底部設備 bar:一條藥丸,中間是「控制中設備」名稱 ==========================
   founder 2026-09-04:「滑鼠 app 內底下的 skaibar_img 換成像手機 app 中間下面的 bar
   一樣,只是中間顯示的是設備名稱(點他後做的事情一樣),左右拖動也要像手機拖動下面
   bar 一樣的切換設備效果(動畫也要一樣)」。

   tap/長按行為一個字都沒動(bottom_logo_cb / bottom_logo_long_press_cb):換的是外觀
   與「拖曳=換設備」這條新手勢。

   動畫是手機 BottomNavBar 那條 Safari 網址列換分頁的複刻,四個門檻逐一對齊
   SkaiLink android-native `ui/home/HomeNav.kt`(BAR_NEIGHBOUR_REVEAL / _WIDEN_AT /
   BAR_SWIPE_SHRINK_BY / barContentAlpha 的 0.6~0.95):
     - 出去的藥丸**縮成一顆球**再淡出,中心緩到 slot 邊緣 —— 不是跟著手指走出畫面被
       裁成直邊(那在手機上被判定成 rendering fault,這裡同理)。
     - 進來的先是一顆球,等出去的走遠了(0.5)才開始變寬,alpha 隨到位程度浮現。
     - 兩條的**內容**(名稱)在寬度不夠時才淡出/淡入,不是硬切;名稱維持整條的寬度置中
       被裁切,所以字不會隨著藥丸變窄而重新排版。
   跟手機不同的只有一點:手機進來的那條刻意不寫設備名(它上面有頂部設備列會講),手錶
   這裡沒有別的地方能講,所以進來的球長大後顯示的就是**要切過去那台**的名字。 */

#define DEVBAR_W 200            /* 藥丸本體寬(舊 skaibar_img 是 176x31) */
#define DEVBAR_H 38             /* 高 = 進來/出去那顆「球」的直徑 */
/* 字級:**固定 FONT_SMALL(20px)**,不跟系統字級走(founder 2026-09-04:「框框裡的字
   有點太大了」)。原本寫 get_system_font_size(0) —— 那是使用者設定的 0..6 索引,
   預設會換算成 FONT_NORMAL(24px),在 38px 高的藥丸裡幾乎頂滿。這條 bar 是 chrome、
   不是內文,跟著系統字級放大只會撐爆自己。20px 是 FT 註冊的最小級,再小會沒有字型。 */
#define DEVBAR_FONT_PX FONT_SMALL
/* 貼底的距離。**不能沿用舊圖的 -12** —— 螢幕是 466 直徑的圓,越靠底邊可用的弦寬
   縮得越快,藥丸的左右兩顆圓角會被切掉(founder 2026-09-04:「線在左右腳有點被切掉」)。
   算式:藥丸是個 stadium,離螢幕中心最遠的點 = 圓角的圓心 + 圓角半徑,
     圓角圓心離中心 = sqrt((W/2 - H/2)^2 + dy^2),dy = 藥丸中心到螢幕中心的垂直距離
   要 sqrt(81^2 + dy^2) + 19 <= 233 → dy <= 191。
   -12 時 dy=202 → 最遠點 236.6,**超出半徑 233 約 4px**(就是被切的那一段);
   -26 時 dy=188 → 最遠點 223.7,離邊還有 9px。 */
#define DEVBAR_Y (-26)
#define DEVBAR_REVEAL_AT  0.30f /* 進來的球在這個行程之後出現(手機 BAR_NEIGHBOUR_REVEAL) */
#define DEVBAR_WIDEN_AT   0.50f /* …在這之後才開始變寬(BAR_NEIGHBOUR_WIDEN_AT) */
#define DEVBAR_SHRINK_AT  0.70f /* 出去的在這段行程內從整條縮成一顆球(BAR_SWIPE_SHRINK_BY) */
#define DEVBAR_CONTENT_HIDDEN_AT 0.60f /* 內容淡出/淡入的填滿率門檻(barContentAlpha) */
#define DEVBAR_CONTENT_FULL_AT   0.95f
/* 行程換算:手機的 progress = 手指位移 / 頁寬,所以這裡用整個螢幕寬當 1.0,
   門檻同樣是頁寬的 0.20(DEVICE_SWIPE_THRESHOLD_FRACTION)。 */
#define DEVBAR_TRAVEL_PX  (LV_HOR_RES_MAX)
#define DEVBAR_COMMIT_PX  (LV_HOR_RES_MAX / 5)
#define DEVBAR_EDGE_PX    14    /* 沒有別台可切時的橡皮筋上限(DEVICE_SWIPE_EDGE_TRAVEL_PX) */
#define DEVBAR_EDGE_DAMP  0.18f
#define DEVBAR_SLOP_PX    6     /* 超過這個位移才算拖曳(以下仍是 tap) */
#define DEVBAR_SETTLE_MS  220   /* 放開後的落地/彈回時間 */

static lv_obj_t *s_devbar_label = NULL;    /* s_top_logo(=出去的那條)裡的名稱 */
static lv_obj_t *s_devbar_in = NULL;       /* 進來的那條,只在拖曳中出現 */
static lv_obj_t *s_devbar_in_label = NULL;
static int32_t s_devbar_drag_px = 0;       /* 目前的視覺位移(左負右正) */
static int32_t s_devbar_travel = 0;        /* 這一次按壓累積的手指位移 */
static bool s_devbar_dragging = false;     /* 已過 slop = 這一次按壓是拖曳不是 tap */
static bool s_devbar_landing = false;      /* 放開後的落地動畫進行中 */
static int  s_devbar_dir = 0;              /* +1=下一台(往左拖) / -1=上一台 / 0=沒有可切的 */
static int  s_devbar_pending = 0;          /* 落地動畫跑完要換到哪個方向(0=這次不換,只是彈回) */

static float devbar_clamp01(float v)
{
    return (v < 0.f) ? 0.f : ((v > 1.f) ? 1.f : v);
}

/* 內容(名稱)的可見度:寬度還夠就全滿,快被藥丸壓到時隨移動淡掉。與手機
   barContentAlpha 同式。 */
static lv_opa_t devbar_content_opa(float w)
{
    float fill = w / (float)DEVBAR_W;
    float a = devbar_clamp01((fill - DEVBAR_CONTENT_HIDDEN_AT) /
                             (DEVBAR_CONTENT_FULL_AT - DEVBAR_CONTENT_HIDDEN_AT));
    return (lv_opa_t)(a * 255.f);
}

/* registry 第 idx 台的名字;超出範圍/沒名字回 NULL。 */
static const char *devbar_name_at(int idx)
{
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES)
        n = MAX_SYNCED_DEVICES;
    if (idx < 0 || idx >= (int)n)
        return NULL;
    const char *nm = (const char *)SkaiWatchSys.device_name[idx];
    return (nm && nm[0]) ? nm : NULL;
}

/* 往 dir 方向的鄰居 index(循環,跟 switch_active_device 同一套算法);不足兩台回 -1。 */
static int devbar_neighbour_index(int dir)
{
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES)
        n = MAX_SYNCED_DEVICES;
    if (n < 2 || dir == 0)
        return -1;
    int cur = active_device_index();
    if (cur < 0)
        cur = 0;
    return (cur + dir + (int)n) % (int)n;
}

/* 靜止時 bar 上的字:控制中那台的名字,沒有目標就講「沒有設備」。只在真的變了才寫,
   所以 40ms poll 每拍呼叫也不會讓 label 一直重排。 */
static void devbar_update_name(void)
{
    if (!s_devbar_label || !lv_obj_is_valid(s_devbar_label))
        return;
    const char *name = active_device_name();
    if (!name || !name[0])
    {
        /* 沒有遠端目標 = 控制目標就是**連著的那支手機**(ble_hid_mouse_set_app_route(false)),
           所以講「手機」,跟媒體欄第 0 欄的表頭同一套說法(app_clock_status_bar.c 的
           media_col_headers_refresh)。
           founder 2026-09-04 回報「在 DESKTOP 那頁拉出媒體頁後 bar 內的設備名稱會變成 -」:
           拉媒體頁**本來就會換控制目標** —— media_col_bind() 依落點那一欄綁,落在第一欄
           (手機)時直接 hid_mouse_clear_active_device()。以前滑鼠頁沒有任何地方顯示控制
           目標,所以這件事一直是隱形的;現在 bar 會講,就不能在那個狀態下變空白。 */
        name = LV_EXT_STR_GET_BY_KEY(connected_phone, "Phone");
    }
    const char *cur = lv_label_get_text(s_devbar_label);
    if (cur && strcmp(cur, name) == 0)
        return;
    lv_label_set_text(s_devbar_label, name);
}

/* 進來的那條寫上**要切過去那台**的名字。手機刻意留白(它上面還有一條頂部設備列會
   講是哪台),手錶沒有那條列,所以名字就得由進來的球長大後自己講。 */
static void devbar_set_incoming_name(int dir)
{
    if (!s_devbar_in_label || !lv_obj_is_valid(s_devbar_in_label))
        return;
    const char *name = devbar_name_at(devbar_neighbour_index(dir));
    if (!name)
        name = LV_EXT_STR_GET_BY_KEY(no_device, "No device");
    const char *cur = lv_label_get_text(s_devbar_in_label);
    if (cur && strcmp(cur, name) == 0)
        return;
    lv_label_set_text(s_devbar_in_label, name);
}

/* 依目前位移擺好兩條藥丸。drag_px 由手指(PRESSING)或落地動畫餵進來,是唯一的輸入 ——
   跟手機一樣「一個 dragX、三個表面讀它」,不讓兩條各自算各自的。 */
static void devbar_apply(int32_t drag_px)
{
    if (!s_top_logo || !lv_obj_is_valid(s_top_logo))
        return;

    const float W = (float)DEVBAR_W;
    const float H = (float)DEVBAR_H;
    float prog = (float)drag_px / (float)DEVBAR_TRAVEL_PX;
    if (prog > 1.f)  prog = 1.f;
    if (prog < -1.f) prog = -1.f;
    float trav = (prog < 0.f) ? -prog : prog;
    float sign = (prog > 0.f) ? 1.f : -1.f;
    bool swiping = (s_devbar_dir != 0) && (drag_px != 0);

    /* 出去的:縮成球(0→0.7)、中心緩到 slot 邊緣、之後(0.7→1)淡出 */
    float out_shrink = devbar_clamp01(trav / DEVBAR_SHRINK_AT);
    float out_w = W + (H - W) * out_shrink;
    float out_rest = W * 0.5f;
    float out_exit = (sign > 0.f) ? (W - H * 0.5f) : (H * 0.5f);
    float out_c = out_rest + (out_exit - out_rest) * out_shrink;
    float out_a = 1.f - devbar_clamp01((trav - DEVBAR_SHRINK_AT) /
                                       (1.f - DEVBAR_SHRINK_AT));
    if (!swiping)
    {
        out_w = W;
        out_c = out_rest;
        out_a = 1.f;
    }

    lv_obj_set_width(s_top_logo, (lv_coord_t)(out_w + 0.5f));
    lv_obj_set_style_translate_x(s_top_logo, (lv_coord_t)(out_c - out_rest), 0);
    lv_obj_set_style_opa(s_top_logo, (lv_opa_t)(out_a * 255.f), 0);
    if (s_devbar_label && lv_obj_is_valid(s_devbar_label))
        lv_obj_set_style_text_opa(s_devbar_label,
                                  swiping ? devbar_content_opa(out_w) : LV_OPA_90, 0);

    if (!s_devbar_in || !lv_obj_is_valid(s_devbar_in))
        return;

    /* 進來的:0.3 之後以一顆球現身,0.5 之後才變寬(太早變寬會蓋到還沒走的那條) */
    float arrival = devbar_clamp01((trav - DEVBAR_REVEAL_AT) /
                                   (1.f - DEVBAR_REVEAL_AT));
    if (!swiping || arrival <= 0.f)
    {
        lv_obj_add_flag(s_devbar_in, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    float in_grow = devbar_clamp01((trav - DEVBAR_WIDEN_AT) /
                                   (1.f - DEVBAR_WIDEN_AT));
    float in_w = H + (W - H) * in_grow;
    float in_entry = (sign > 0.f) ? (H * 0.5f) : (W - H * 0.5f);
    float in_c = in_entry + (W * 0.5f - in_entry) * arrival;
    float in_a = arrival * 2.5f;
    if (in_a > 1.f) in_a = 1.f;

    lv_obj_set_width(s_devbar_in, (lv_coord_t)(in_w + 0.5f));
    lv_obj_set_style_translate_x(s_devbar_in, (lv_coord_t)(in_c - W * 0.5f), 0);
    lv_obj_set_style_opa(s_devbar_in, (lv_opa_t)(in_a * 255.f), 0);
    if (s_devbar_in_label && lv_obj_is_valid(s_devbar_in_label))
        lv_obj_set_style_text_opa(s_devbar_in_label, devbar_content_opa(in_w), 0);
    lv_obj_clear_flag(s_devbar_in, LV_OBJ_FLAG_HIDDEN);
}

/* 拖曳結束/取消後回到靜止:位移歸零、進來的那條收掉、名字重取(可能剛換過台)。 */
static void devbar_rest(void)
{
    s_devbar_drag_px = 0;
    s_devbar_dir = 0;
    s_devbar_landing = false;
    devbar_update_name();
    devbar_apply(0);
}

static void devbar_anim_exec_cb(void *var, int32_t v)
{
    (void)var;
    s_devbar_drag_px = v;
    devbar_apply(v);
}

/* 落地動畫結束 = 真的換台。切換**在這裡**才發生(手機也是:先把動畫跑完才換資料,
   先換資料會露出「新的那條畫在舊的位置」那一格)。
   也給「動畫還在跑就又按下去」那條路用 —— 那一顆按壓會把動畫殺掉,欠的那次切換要
   在這裡先還掉,不然手勢做完了設備卻沒換。 */
static void devbar_commit_pending(void)
{
    int dir = s_devbar_pending;
    s_devbar_pending = 0;
    s_devbar_dir = 0;
    s_devbar_landing = false;
    s_devbar_drag_px = 0;
    if (dir != 0)
        switch_active_device(dir);   /* 內含 commu_send_active_device + 名稱刷新 */
    devbar_rest();
}

static void devbar_settle_done_cb(lv_anim_t *a)
{
    (void)a;
    devbar_commit_pending();
}

static void devbar_animate_to(int32_t to)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_devbar_drag_px);
    lv_anim_set_exec_cb(&a, devbar_anim_exec_cb);
    lv_anim_set_values(&a, s_devbar_drag_px, to);
    lv_anim_set_time(&a, DEVBAR_SETTLE_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, devbar_settle_done_cb);
    lv_anim_start(&a);
}

/* 手指在 bar 上的左右拖曳 = 換設備。tap 完全不受影響:沒過 slop 就沒有任何事發生,
   而過了 slop 的那一次按壓會把隨後那顆 CLICKED 吃掉(s_devbar_dragging)。 */
static void devbar_press_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED)
    {
        lv_anim_del(&s_devbar_drag_px, devbar_anim_exec_cb);
        /* 上一次的落地動畫被這顆按壓殺掉了 → 欠的那次切換先還掉(不然快速連拖兩次,
           第一次會靜靜消失)。 */
        if (s_devbar_landing)
            devbar_commit_pending();
        s_devbar_travel = 0;
        s_devbar_dragging = false;
        s_devbar_landing = false;
        s_devbar_dir = 0;
        s_devbar_drag_px = 0;
        return;
    }

    if (code == LV_EVENT_PRESSING)
    {
        if (s_devbar_landing)
            return;
        lv_indev_t *indev = lv_indev_get_act();
        if (!indev)
            return;
        lv_point_t v;
        lv_indev_get_vect(indev, &v);
        s_devbar_travel += v.x;
        if (!s_devbar_dragging)
        {
            int32_t ax = (s_devbar_travel < 0) ? -s_devbar_travel : s_devbar_travel;
            if (ax < DEVBAR_SLOP_PX)
                return;
            s_devbar_dragging = true;
        }
        int dir = (s_devbar_travel < 0) ? +1 : -1;   /* 往左拖=下一台,同手機 */
        bool toward_end = (devbar_neighbour_index(dir) < 0);
        if (!toward_end && dir != s_devbar_dir)
            devbar_set_incoming_name(dir); /* 中途改變方向也要換成另一邊那台 */
        s_devbar_dir = toward_end ? 0 : dir;
        int32_t visual = s_devbar_travel;
        if (toward_end)
        {
            /* 沒有別台可切:給一點感覺、但不承諾(手機的 rubber-band) */
            visual = (int32_t)((float)s_devbar_travel * DEVBAR_EDGE_DAMP);
            if (visual >  DEVBAR_EDGE_PX) visual =  DEVBAR_EDGE_PX;
            if (visual < -DEVBAR_EDGE_PX) visual = -DEVBAR_EDGE_PX;
        }
        s_devbar_drag_px = visual;
        devbar_apply(visual);
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        if (!s_devbar_dragging)
            return;                      /* 純 tap:交給 bottom_logo_cb */
        int32_t ax = (s_devbar_travel < 0) ? -s_devbar_travel : s_devbar_travel;
        bool commit = (s_devbar_dir != 0) && (ax >= DEVBAR_COMMIT_PX);
        s_devbar_landing = true;
        s_devbar_pending = commit ? s_devbar_dir : 0;
        /* commit:跑完整段行程 —— 出去的走光、進來的到位,然後資料才換手。
           否則:彈回 0,什麼都沒換。 */
        devbar_animate_to(commit ? (-s_devbar_dir * DEVBAR_TRAVEL_PX) : 0);
        return;
    }
}

static bool devbar_drag_active(void)
{
    return s_devbar_dragging;
}

/* 拖曳過的那一次按壓,LVGL 放開時還會補一顆 CLICKED —— 讓 bottom_logo_cb 認得它。
   同 s_logo_lp_consumed 的 pattern。 */
static bool devbar_consume_click(void)
{
    if (!s_devbar_dragging)
        return false;
    s_devbar_dragging = false;
    return true;
}

/* 建底部設備 bar:進來的那條先建(z 在下),出去的那條就是 s_top_logo(z 在上、掛事件)。
   兩條都置中對齊 BOTTOM_MID,水平位移一律走 translate_x,所以寬度變化不會改變它們的
   對齊基準。 */
static void devbar_style_capsule(lv_obj_t *o)
{
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, DEVBAR_W, DEVBAR_H);
    lv_obj_align(o, LV_ALIGN_BOTTOM_MID, 0, DEVBAR_Y);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    /* 顏色照抄手機的 liquidGlassChrome(ui/components/GlassBlur.kt):
         backdrop blur → SkaiBackdrop(#0B0F16) @52% → GlassChromeTint(白 @10%)
         → rim GlassChromeRim(白 @18%,1dp)
       手錶沒有 backdrop blur,所以把前兩層先合成成**一層**:
         合成 alpha = 1 - 0.48*0.90 = 0.568  → opa 145
         合成顏色  = (0.10*255 + 0.90*0.52*#0B0F16) / 0.568 ≈ #36393F
       之前寫純黑 @60% 是我自己配的,偏黑偏悶,跟手機那條偏藍灰的玻璃不是同一個東西
       (founder 2026-09-04:「框框的顏色 ui 好像跟手機上的不太一樣」)。 */
    lv_obj_set_style_bg_color(o, lv_color_hex(0x36393F), 0);
    lv_obj_set_style_bg_opa(o, 145, 0);
    lv_obj_set_style_border_width(o, 1, 0);
    lv_obj_set_style_border_color(o, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(o, 46, 0);   /* 白 @18% = 手機的 GlassChromeRim */
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t *devbar_make_label(lv_obj_t *parent)
{
    lv_obj_t *l = lv_label_create(parent);
    /* 寬度固定 = 整條的內寬並置中:藥丸變窄時字是**被裁切**的,不是重新排版
       (手機那邊是 requiredWidth(slotWidth) + clipToBounds,同一招)。 */
    const lv_font_t *f = LV_EXT_FONT_GET(DEVBAR_FONT_PX);
    lv_obj_set_style_text_font(l, f, 0);
    /* **高度鎖成一行**。LVGL 的 LV_LABEL_LONG_DOT 是「維持物件大小,把文字斷行,在
       最後一行寫點點」—— 它看的是物件的**高度**,而不是「永遠只有一行」。高度留
       LV_SIZE_CONTENT 的話,長名字(DESKTOP-DU738D5)會先斷成兩行、然後物件長高把兩行
       都顯示出來,一個點都不會出現(founder 2026-09-04:「整段文字上下排了」)。
       固定成一行的行高之後,第二行沒有位置可長,LONG_DOT 才會在第一行結尾補「...」。
       為什麼一開始看起來是對的、拉完媒體頁才壞:名字只在**變動時**重設(手機↔DESKTOP
       來回一趟就是一次重設),重設才會重新斷行。 */
    lv_coord_t line_h = f ? (lv_coord_t)lv_font_get_line_height(f) : 24;
    lv_obj_set_size(l, DEVBAR_W - 24, line_h);
    lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(l, LV_OPA_90, 0);
    lv_obj_align(l, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE);
    return l;
}


/* device_pager_refresh(跑在每次手機 E7 同步)會把 active 清成 ""；滑鼠介面開著且已選
   設備時，它擁有 active relay 目標、不該被清掉。device_pager 用此 gate。
   2026-08-15 真機抓到:hosted 滑鼠(錶盤頂部面板 hid_mouse_build_ui,APP_ID_MOUSE 非
   active)之前不算「擁有」→ 進滑鼠後第一次 E7 同步(~60s)就把 relay 目標洗掉 → 手機
   singleDeviceMode 垮掉、聚合 actions 回灌單設備抽屜、0x0E summon 路由無目標被丟
   (「再點 skaibar_img 出來的是手機 actions」)。app_control_get_mouse_mode 兩條路徑
   (standalone/hosted)都會設,拿它補上。 */
bool hid_mouse_owns_active_target(void)
{
    extern bool app_control_get_mouse_mode(void);
    return (gui_app_is_actived(APP_ID_MOUSE) || app_control_get_mouse_mode()) &&
           s_dev_active_id[0] != '\0';
}

/* 公開:滑鼠的鍵盤/語音站模式是否開著。instruction_list 的 R33 加固用 —— 這段期間
   鍵盤 UI 佔著 heap,隱藏中的共享清單不准重建(2026-08-15 真機 sys memory full)。 */
bool hid_mouse_keyboard_mode_active(void)
{
    extern bool app_control_get_mouse_mode(void);
    return app_control_get_mouse_mode() && current_hid_mode == HID_MODE_KEYBOARD;
}

/* 自有底部 bar(trackpad_mic_btn,只放 skaibar 圖)的隱藏邏輯：instruction_list 浮層 bar/
   清單一出現就收掉它,避免兩條重疊。第一次 tap(列表浮入)立刻收(bar_ai_on_tap),整段顯示期間
   由 poll(instruction_list_is_visible)維持隱藏,關閉後還原。全在 hid_mouse 內、不碰共用元件。 */
extern bool instruction_list_floating_bar_visible(void); /* 浮層 bar(s_global_bar_layer)實際可見 */
static lv_timer_t *s_bar_ai_sync_timer = NULL;
static rt_tick_t s_last_bar_tap_tick = 0;
#define BAR_TAP_MORPH_GRACE_MS 300 /* tap→浮層 bar 出現的橋接窗，立刻收後撐到 getter 轉 true */
static void bar_ai_sync_set_hidden(bool hide)
{
    if (!trackpad_mic_btn || !lv_obj_is_valid(trackpad_mic_btn))
        return;
    bool hidden = lv_obj_has_flag(trackpad_mic_btn, LV_OBJ_FLAG_HIDDEN);
    if (hide && !hidden)
        lv_obj_add_flag(trackpad_mic_btn, LV_OBJ_FLAG_HIDDEN);
    else if (!hide && hidden)
        lv_obj_clear_flag(trackpad_mic_btn, LV_OBJ_FLAG_HIDDEN);
}

/* 立起輸入面板開著時,觸控板自己的上下兩個圖示要一起讓位(founder 2026-08-01):頂部的
   keyboard_icon 與底部的 skaibar_img。跟 bar_ai_sync_set_hidden 一樣由 poll 驅動,
   所以不必在開/關兩處各記一次狀態。 */
static void lift_chrome_set_hidden(bool hide)
{
    if (!s_top_logo || !lv_obj_is_valid(s_top_logo))
        return;
    /* 這支由 40ms poll 驅動,會無條件把圖示放回來 —— 鍵盤模式下必須讓它繼續藏,
       否則 apply_hid_mode / 語音站那邊剛藏好,下一個 tick 就被這裡拆掉
       (2026-08-07 founder:進輸入畫面後鍵盤圖與滑鼠圖都不要出現)。 */
    if (current_hid_mode == HID_MODE_KEYBOARD)
        hide = true;
    bool hidden = lv_obj_has_flag(s_top_logo, LV_OBJ_FLAG_HIDDEN);
    if (hide && !hidden)
        lv_obj_add_flag(s_top_logo, LV_OBJ_FLAG_HIDDEN);
    else if (!hide && hidden)
        lv_obj_clear_flag(s_top_logo, LV_OBJ_FLAG_HIDDEN);
}
static void bar_ai_on_tap(void)
{
    s_last_bar_tap_tick = rt_tick_get();
    bar_ai_sync_set_hidden(true); /* 立刻收，不等 poll → 第一次 tap 就不會看到兩條 */
}

// 「錶面立起正對臉」姿態觸發：在 LVGL thread 開啟「立起輸入面板」(輸入框 + 上方 logo/send
// + 下方小麥克風)。2026-07-31 founder 改版：不再要求控制中那台設備有聚焦輸入框——一律開，
// 聚焦狀態只決定 icon_send 出不出現(見 instruction_list_open_lift_input_view)。唯一的無效
// 情形是沒有控制目標(s_dev_active_id 空)。由 motion thread 的 hid_mouse_trigger_skaibar_
// from_pose 發 LVGL_MSG_TYPE_MOUSE_OPEN_SKAIBAR 轉進來，故此處已在 LVGL thread、可直接碰 UI。
void open_skaibar_from_pose(void)
{
    if (s_bottom_input_disabled)
    {
        return;
    }
    if (s_hw_view_active)
    {
        /* 側立手寫進行中不疊輸入面板——姿勢從側立轉到立起時,close-handwrite 保險
           (set_gravity_position VERTICAL 邊緣)會先收掉手寫,下一次再立起才開面板。 */
        return;
    }
    extern bool instruction_list_open_lift_input_view(const char *device_id);
    instruction_list_open_lift_input_view(s_dev_active_id);
    /* 底部自有 bar(skaibar_img)與頂部 keyboard_icon 由 poll sync(bar_ai_sync_timer_cb)
       依 instruction_list_lift_input_view_open() 一起收掉 —— founder 2026-08-01：面板開著
       時觸控板上下兩個圖示都要讓位。(2026-07-17 曾要求底部維持原樣，已被這次改口取代。) */
}

/* 給 instruction_list 在「切換浮層 bar 顯示/隱藏的當幀」同步呼叫 → frame-perfect 交接，
   消除 poll 40ms 延遲造成的那一閃/空窗。off-mouse(trackpad_mic_btn NULL)為 no-op、錶盤不受影響。 */
void hid_mouse_set_own_bar_hidden(bool hide)
{
    if (!hide)
        s_last_bar_tap_tick = 0; /* 顯示時清 tap-grace，避免 poll 又把它壓回去 */
    bar_ai_sync_set_hidden(hide);
}
static void bar_ai_sync_timer_cb(lv_timer_t *t)
{
    (void)t;
    /* 底部物件 dump(botdump_walk/botdump_run)已完成任務並移除 —— 它是 2026-08-17 定位
       「三鍵列下方那張多餘小圖」的工具:走物件樹把 y>=340、祖先鏈無 HIDDEN 的物件連同
       圖檔名印出來。需要時加回來、掛在 instruction_list_is_visible() 的上升緣即可
       (注意 MSH_CMD_EXPORT 在這個 build 會被 armlink 當未使用段移掉,不能靠 msh 觸發)。 */
    /* 鍵盤露出時，把新冒出來、壓在鍵盤上的浮層一律設成按下時放行(見
       kbd_guard_overlappers 的說明)。放在這支既有的 40ms poll 上，浮層晚一步
       出現也追得到；沒開鍵盤時第一行就 return，成本可忽略。 */
    kbd_guard_overlappers();
    /* 直接對齊「那條浮層 bar」的實際可見性(不是清單)：它一現就收自有 bar、一收就還原。
       tap_grace 只橋接「點下到浮層 bar 出現」那短短一段(立刻收後撐住,避免 getter 還沒 true)。 */
    bool engaged = instruction_list_floating_bar_visible();
    bool tap_grace = (rt_tick_get() - s_last_bar_tap_tick) <
                     rt_tick_from_millisecond(BAR_TAP_MORPH_GRACE_MS);
    /* 立起輸入面板也算「有東西蓋在上面」—— 2026-07-17 當時的決定是這條流程底部維持自有 bar
       原樣,2026-08-01 founder 改口:面板開著時上下兩個圖示都要藏。以較新的為準。 */
    extern bool instruction_list_lift_input_view_open(void);
    bool lift = instruction_list_lift_input_view_open();
    bar_ai_sync_set_hidden(engaged || tap_grace || lift);
    /* founder 2026-08-17(連五輪回報「三鍵列下方還有那張小麥克風圖」):**這才是那張圖**。
       s_top_logo 是滑鼠頁底部唯一看得見的入口圖(hosted 顯示 skaibar_img,BOTTOM_MID −12,
       正好落在三鍵列下方更靠底緣),而這支 poll 原本只在立起面板開著時收它 —— 抽屜/浮層
       清單開著時 lift=false,於是**每一拍都主動把它清回可見**。
       上面那支 bar_ai_sync_set_hidden 收的是 trackpad_mic_btn:那是個不放圖的空容器
       (見其建立處註解),收它視覺上毫無效果 —— 這就是「浮層一出現就收自有 bar」這條規則
       名存實亡、而我連追五輪都打在 mic_bar / trackpad_mic_btn 這些沒有圖的東西上的原因。
       浮層 bar 一現(engaged)就跟立起面板同待遇:底部這張圖讓位。 */
    lift_chrome_set_hidden(lift || engaged || tap_grace);
    /* 底部設備 bar 的名字:registry / 設備名是手機晚一步同步進來的(E7),進 app 當下
       常常還是空的。只在字真的變了才寫,所以擺在這支 40ms poll 上不花什麼。 */
    devbar_update_name();
    /* 抽屜/語音站期間強制壓住共用清單那條舊 mic pill —— 它的顯藏有六個寫入點,而開抽屜
       進場鏈的最後一棒(reveal_drag_begin → refresh_home_bar)本身就是專門叫它出來的。
       在這支每拍都跑的 poll 上收尾,任何路徑最多只能讓它閃一幀(founder 連三輪回報)。 */
    {
        extern void instruction_list_drawer_enforce_bar_hidden(void);
        instruction_list_drawer_enforce_bar_hidden();
    }
    /* 右緣鍵盤鈕:只在**觸控板露著、電腦有聚焦輸入框、沒有別的東西蓋在上面**時浮現。
       0x17 旗標由通訊執行緒寫、這裡(LVGL 執行緒)讀 —— 單一 bool,不需鎖。 */
    if (kbd_side_btn && lv_obj_is_valid(kbd_side_btn))
    {
        extern bool instruction_list_remote_target_has_focus(void);
        bool want = (current_hid_mode == HID_MODE_TRACKPAD) && !engaged &&
                    !tap_grace && !lift && !dev_active_offline() &&
                    instruction_list_remote_target_has_focus();
        bool hidden = lv_obj_has_flag(kbd_side_btn, LV_OBJ_FLAG_HIDDEN);
        if (want && hidden)
            lv_obj_clear_flag(kbd_side_btn, LV_OBJ_FLAG_HIDDEN);
        else if (!want && !hidden)
            lv_obj_add_flag(kbd_side_btn, LV_OBJ_FLAG_HIDDEN);
    }
    dev_offline_overlay_sync(); /* active 設備斷線=灰版+「斷線」(順路 poll) */
}

void lv_create_mouse_screen(lv_obj_t *scr)
{
    lv_obj_t *bg = common_black_bg(scr);
    lv_obj_set_scrollbar_mode(bg, LV_SCROLLBAR_MODE_OFF);
    /* Standalone mouse app keeps its solid black backdrop; when hosted (device_pager,
       scr != the active screen) keep the trackpad bg fully transparent so the page's
       own backdrop / the watch face shows through instead of a black layer. */
    lv_obj_set_style_bg_opa(bg, (scr == lv_scr_act()) ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
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

    /* R39 診斷:量到滑鼠圖層整層要 ~41.7KB,而進來前只剩 ~47KB —— 退出時要同時亮
       tileview 就爆。先切開看是誰吃的(trackpad / keyboard 兩段最大),別再猜。
       穩定後連同其他 [heap] 診斷一起移除。 */
    extern void clock_main_heap_log(const char *tag);
    clock_main_heap_log("mouse-ui:containers");

    // === Per-mode UI ===
    // 加新元件改下面兩個函式
    create_trackpad_mode_ui(mode_container[HID_MODE_TRACKPAD]);
    clock_main_heap_log("mouse-ui:trackpad");
    /* R40:鍵盤那 38KB 不在這裡建 —— 第一次切進鍵盤模式時由 mode_set_visible →
       kbd_ensure_built() 建(整個 session 沒用到鍵盤就完全不花)。 */
    s_kbd_ui_built = false;

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

    /* 「控制中設備」名稱+切換箭頭已搬進媒體下拉頁頂部
       (create_media_center_panel,2026-07-02 使用者要求) — trackpad 頂部
       不再放常駐設備名。 */

    // Trackpad mode 下方 bar 觸控區（跨 mode hit area:tap/長按開 skaibar）
    bottom_swipe_area = lv_obj_create(bg);
    lv_obj_remove_style_all(bottom_swipe_area);
    // 高度 100→50(2026-07-02):multitask 上拉手勢停用後,這區只剩 tap/長按,
    // 縮到 bar 本體附近,上方 50px 還給 trackpad 游標,不再攔截拖曳
    lv_obj_set_size(bottom_swipe_area, 280, 50);
    lv_obj_set_pos(bottom_swipe_area, (LV_HOR_RES_MAX - 280) / 2,
                   LV_VER_RES_MAX - 50);
    lv_obj_set_style_bg_opa(bottom_swipe_area, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(bottom_swipe_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(bottom_swipe_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(bottom_swipe_area, text_input_bar_cb, LV_EVENT_ALL,
                        NULL);
    /* 這條 280×50 貼在 y=416..466，與鍵盤最下排重疊 → 鍵盤露出時放行(見
       chrome_hit_test_cb)。 */
    lv_obj_add_flag(bottom_swipe_area, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_add_event_cb(bottom_swipe_area, chrome_hit_test_cb, LV_EVENT_HIT_TEST,
                        NULL);

    /* 進輸入用的 keyboard_icon:2026-08-07 founder 從頂部搬到**底部中間**，連同
       「點它 = 進輸入」的行為一起搬。頂部現在專職「下拉開面板 / 按住進飛鼠」，
       不再吃 tap，手勢不打架。建在 bottom_swipe_area 之後 = z-order 在它之上，
       所以圖上的 tap 歸自己、圖以外的那條 280×50 仍舊是 skaibar 的 tap/長按。
       zoom 180 = 手寫頁頂部同款(64×64 原圖，渲染 ~45px)。 */
    /* 底部這條(founder 2026-09-04):外觀改成手機 app 底部那條藥丸,中間是**控制中設備
       的名字**;tap/長按做的事跟舊的 skaibar_img 一模一樣(開 session/搜尋抽屜、長按進
       語音),多出來的是「左右拖動 = 換設備」,動畫比照手機。細節見上方 devbar 段。
       先建**進來的**那條 → z 在下,拖曳時它從後面長出來,不會蓋到還沒走完的那條
       (手機那邊也是先畫 incoming 再畫 outgoing)。 */
    s_devbar_in = lv_obj_create(bg);
    devbar_style_capsule(s_devbar_in);
    s_devbar_in_label = devbar_make_label(s_devbar_in);
    lv_obj_add_flag(s_devbar_in, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_devbar_in, LV_OBJ_FLAG_CLICKABLE); /* 純畫面,手指永遠在真的那條上 */

    s_top_logo = lv_obj_create(bg);
    devbar_style_capsule(s_top_logo);
    s_devbar_label = devbar_make_label(s_top_logo);
    devbar_update_name();
    lv_obj_add_flag(s_top_logo, LV_OBJ_FLAG_CLICKABLE);
    /* 能按到的範圍要比可視範圍大(founder 2026-08-07)。藥丸本身已比舊圖大一圈,
       外圈相應收到 24。 */
    lv_obj_set_ext_click_area(s_top_logo, 24);
    /* 這圈放大後會壓到鍵盤最下排(英文空白鍵 y 369..418、中文的下移到 y 401..451),
       症狀是「最下排按不到,偶爾才中一次」(founder 2026-08-07 實測 [geo])。
       鍵盤露出時 hit-test 放行給底下的鍵。 */
    lv_obj_add_flag(s_top_logo, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_add_event_cb(s_top_logo, chrome_hit_test_cb, LV_EVENT_HIT_TEST, NULL);
    lv_obj_add_event_cb(s_top_logo, bottom_logo_cb, LV_EVENT_CLICKED, NULL);
    /* 長按=無聚焦輸入框時直接進語音搜尋(2026-08-15);其 CLICKED 由 s_logo_lp_consumed 吃掉 */
    lv_obj_add_event_cb(s_top_logo, bottom_logo_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);
    /* 左右拖動換設備:PRESSED/PRESSING/RELEASED 三顆自己收 —— tap 沒過 slop 就完全
       沒有副作用,過了 slop 的那一次按壓由 devbar_consume_click 把 CLICKED 吃掉。 */
    lv_obj_add_event_cb(s_top_logo, devbar_press_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_top_logo, devbar_press_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(s_top_logo, devbar_press_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_top_logo, devbar_press_cb, LV_EVENT_PRESS_LOST, NULL);

    /* === APP 內建的媒體中心下拉層:整個退役 ==========================================
       2026-08-06 已先對「面板 host 模式」停建(founder:「APP 內上方的媒體中心可以不
       要」);ADR-0020 之後媒體中心是錶盤右側每台設備一欄的常駐頁,獨立開 APP_ID_MOUSE
       時同樣隨時滑得到,所以這一份就是純重複 —— founder 2026-08-12:「滑鼠 app 本來上
       面是不是還有一個媒體頁面?如果還有的話把那個拿掉,現在有外面的那個就夠了」。
       連帶好處是 heap:這台只剩 ~40KB,整層 tileview+兩個全螢幕 tile+控制列不再常駐。
       所有 media_tileview 的使用點本來就都有 NULL 防護(它在 host 模式下一直是 NULL),
       所以不建它不需要其他改動;offline overlay 取 host 也有 fallback。 */

    // 啟動自有底部 bar 的隱藏同步 poll（instruction_list 浮層 bar 顯示時收掉它）
    if (s_bar_ai_sync_timer == NULL)
        s_bar_ai_sync_timer = lv_timer_create(bar_ai_sync_timer_cb, 40, NULL);


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

// 直接設定 handfree state（不 toggle），給 fsr 壓感 sampler / 頂部 logo 用
void set_hid_mouse_handfree_mode_to(bool v)
{
    if (handfree != v)
        LOG_I("[logo-fly] handfree -> %d", v);
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
/* The host the mouse UI (its file-static object globals) was last built on. The
   UI is a SINGLETON — device_pager and the standalone APP_ID_MOUSE app take turns
   building it. device_pager persists its copy and uses this to detect when the
   standalone app stole the globals (host != its tile) so it can rebuild. NULL when
   no UI is built (after hid_mouse_destroy). */
static lv_obj_t *s_ui_host = NULL;
lv_obj_t *hid_mouse_ui_host(void) { return s_ui_host; }

/* Pure UI build — no global state. Persist-friendly: device_pager calls this once
   per host so the trackpad is already on its tile when the page slides in. */
void hid_mouse_build_ui(lv_obj_t *scr)
{
    /* Screen-level launch transition only when we own the screen. When hosted
       in a container (T4 device_pager), skip it — the pager drives its own
       reveal animation and the trans-anim would flash a blank overlay. */
    if (scr == lv_scr_act())
        cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    lv_create_mouse_screen(scr);
    { extern void clock_main_heap_log(const char *tag); clock_main_heap_log("mouse-ui:done"); }
    s_ui_host = scr;
}

/* The global "enter mouse mode" side effects (mouse-mode flag, status-bar gesture
   zones off, gesture detect off). Toggled per page entry so a persisted UI can
   sit inert off-page without the watch stuck in mouse mode. */
void hid_mouse_enter_mode(void)
{
    /* Pin the link FAST (15-35 ms) for the pointer uplink — with the param
       fix, idle SLOW now really applies (100-120 ms, latency 9), far too
       slow for a cursor. Request BEFORE raising the mouse flag: the flag
       makes the main.c guard reject sub-ULTRA changes, which would swallow
       this very request. Exit paths clear the flag first, then drop SLOW.
       (The 2026-07-02 "entry request crashes the LCPU" episodes were NOT
       this call — root cause was a use-after-free in the then-uncommitted
       watchface swipe-catcher WIP that rode along in the same builds.) */
    skaiwatch_ble_set_performance(BLE_PERF_FAST);
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

/* Reverse of hid_mouse_enter_mode WITHOUT tearing the UI down — device_pager calls
   this on page-leave so the persisted trackpad UI survives for the next swipe.
   (hid_mouse_destroy still does this PLUS the full UI teardown for app stop / the
   no-device case.) Mirrors the mode-related lines of hid_mouse_destroy. */
void hid_mouse_exit_mode(void)
{
    app_control_set_mouse_mode(false);
    skaiwatch_ble_set_performance(BLE_PERF_SLOW);
    extern void set_status_bar_area_up_state(bool state);
    extern void set_status_bar_area_down_state(bool state);
    extern void set_status_bar_area_left_state(bool state);
    set_status_bar_area_up_state(true);
    set_status_bar_area_down_state(true);
    set_status_bar_area_left_state(true);
}

void hid_mouse_create(lv_obj_t *scr)
{
    hid_mouse_build_ui(scr);
    hid_mouse_enter_mode();
}


/**
 * @brief Tear down the mouse UI + deactivate control surface.
 *        T1 part 1 (host decouple): was static on_stop(void).
 */
void hid_mouse_destroy(void)
{
    hw_cancel_session(); /* 手寫殘留:取消(不送出)收乾淨(離開 app 清殘留) */
    /* view 是 scr 子物件、screen teardown 一併釋放——只清指標,下次 ensure 重建 */
    s_hw_view = NULL;
    s_hw_btn_exit = NULL;
    s_hw_btn_clear = NULL;
    s_hw_btn_clear_lbl = NULL;
    s_hw_btn_clear_img = NULL;
    s_hw_btn_mode = NULL;
    s_hw_btn_enter = NULL;
    s_hw_backdrop = NULL;
    s_hw_cand_row = NULL;
    s_hw_cand_count = 0;
    for (int i = 0; i < HW_CAND_MAX; i++)
    {
        s_hw_cand_btns[i] = NULL;
        s_hw_cand_lbls[i] = NULL;
    }
    for (int i = 0; i < HW_TRACE_STROKES; i++)
        s_hw_lines[i] = NULL;
    dial_drag_state_reset(); /* 離開 app 先清 dial/拖曳/timer 殘留(founder 2026-07-17 卡住根因) */
    app_control_set_mouse_mode(false);
    skaiwatch_ble_set_performance(BLE_PERF_SLOW);

    /* 離開滑鼠 app：若 bar 還在單設備 skaibar 模式,把共享浮層清單還原成錶盤清單 + 通知
       電腦收掉它的 skaibar,避免設備選項殘留到錶盤底部 bar。idempotent —— 非單設備模式
       (含 device_pager 內嵌 trackpad 的 teardown)直接 no-op。 */
    {
        extern void instruction_list_bar_device_dismiss(void);
        instruction_list_bar_device_dismiss();
    }

    /* 待發的 preview timer 也要清。它只在「離站」那條路被刪(kbd_lower_switch),離開整個
       app 這條路從來沒清過 —— 一個活過 app 拆除、callback 還會發 BLE 的 pending
       lv_timer,正是這個 app 已知那類卡死/UAF 的形狀(比較 2026-07-17 dial timer 殘留)。
       LVGL timer 不隨 obj 樹一起釋放,所以要自己收。 */
    if (s_voice_preview_timer != NULL)
    {
        lv_timer_del(s_voice_preview_timer);
        s_voice_preview_timer = NULL;
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
    lv_anim_del(&s_node_entrance, node_entrance_anim_cb); // 停掉滾輪進場淡入
    s_node_entrance = 1000; // 還原：下個（含 standalone）建立時節點為全顯示
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
    s_ui_host = NULL; /* UI torn down — device_pager will rebuild on next entry */
    /* 丙：s_dev_active_id 不在這裡清 — 保留「上次控制的設備」，重入時 ONSTART 自動
       接回(見 GUI_APP_MSG_ONSTART)；reboot 才隨 static 歸零。 */
    s_ctrl_dev_label = NULL;   /* 標籤是 scr 子物件、screen teardown 一併釋放，清指標 */
    s_dev_offline_overlay = NULL; /* 同上,清 stale 引用 */
    s_dev_status_dot = NULL;
    s_dev_left_arrow = NULL;   /* 箭頭同為 scr 子物件、teardown 一併釋放，清指標 */
    s_dev_right_arrow = NULL;
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
    media_center_vol_cancel_repeat(); // 停音量長按 repeat timer，避免 screen teardown 後 timer UAF
    media_tileview = NULL;
    media_home_tile = NULL;
    media_tile = NULL;
    media_center_title_label = NULL;
    media_center_play_img = NULL;
    status_bar_area_up = NULL;
    /* 底部設備 bar:落地動畫可能還在跑,exec cb 會碰這些物件 → 先殺 anim 再歸零 */
    lv_anim_del(&s_devbar_drag_px, devbar_anim_exec_cb);
    s_top_logo = NULL;
    s_devbar_label = NULL;
    s_devbar_in = NULL;
    s_devbar_in_label = NULL;
    s_devbar_drag_px = 0;
    s_devbar_travel = 0;
    s_devbar_dragging = false;
    s_devbar_landing = false;
    s_devbar_dir = 0;
    s_devbar_pending = 0;
    s_top_hw_pull = false;
    // 頂部按住進的飛鼠模式：app 被拆時可能收不到 RELEASED，static 殘留
    // true 會讓下次進 app 直接是飛鼠 → 拆除時一律歸位
    top_hold_cancel();
    s_top_fly_active = false;
    handfree = false;

    // Keyboard mode 下半部 mic 區清理
    s_kbd_ui_built = false; /* R40:圖層拆了,下次進鍵盤要重建 */
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
    kbd_exit_btn = NULL; /* 物件隨 bg 子樹拆除,清 stale 引用 */
    kbd_side_btn = NULL;
    kbd_top_pull = NULL;
    s_kbd_cand_row = NULL;
    s_kbd_py_lbl = NULL;
    for (int ci = 0; ci < KBD_CAND_MAX; ci++)
    {
        s_kbd_cand_btns[ci] = NULL;
        s_kbd_cand_lbls[ci] = NULL;
    }
    s_kbd_cand_count = 0;
    s_py_len = 0;
    s_py_buf[0] = '\0';
    keyboard_visible = false;

    // Clean up input display (now part of text_input_bar_bg)
    input_content_container = NULL;
    input_display_label = NULL;
    input_cursor = NULL;
    if (cursor_blink_timer != NULL)
    {
        lv_timer_del(cursor_blink_timer);
        cursor_blink_timer = NULL;
    }
    if (s_bar_ai_sync_timer != NULL)
    {
        lv_timer_del(s_bar_ai_sync_timer);
        s_bar_ai_sync_timer = NULL;
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
        /* P3 麥克風 OOM 修復:standalone 滑鼠 app 進來時把恆駐的錶盤(Main app)整個拆掉,釋放
           ~111K SRAM(實機驗 337K→226K used)給麥克風 WebRtcNsx(需一塊連續 ~13.3K)。用明確
           gui_app_exit(乾淨走框架 app_destory),**刻意不碰 instruction_list** —— 碰它會害 Main
           的 subpage 停不掉、拆不乾淨(踩過)。錶盤被拆後它建的 instruction_list 單例也沒了 →
           滑鼠 skaibar 改在 bar tap 時 lazy 自建(見 instruction_list_bar_tap_device)。離開滑鼠
           app 時框架自動重跑 Main 重建錶盤。device_pager 內嵌路徑不走本 standalone msg_handler。 */
        {
            extern int gui_app_exit(const char *id);
            gui_app_exit(APP_ID_MAIN);
        }
        /* 丙：記住上次控制的設備 — 重入時若它還在 registry 就自動接回 relay 控制；
           否則(沒記憶/設備已不在)回到 BLE HID 直連。s_dev_active_id 跨 app 重入保留
           (destroy 不清)，reboot 才隨 static 歸零。 */
        if (active_device_name() != NULL)
        {
            /* 上次控制的設備還在 → 接回 relay 控制 */
            LOG_W("[active] mouse reenter");
            commu_send_active_device(s_dev_active_id);
            ble_hid_mouse_set_app_route(true);
        }
        else
        {
            /* 沒記憶/設備已不在 → 預設選一台（主要→第一個在線→清單第一個），不再空狀態 */
            int def = pick_default_device();
            if (def >= 0)
            {
                set_active_device_by_index(def);
            }
            else
            {
                s_dev_active_id[0] = '\0'; /* registry 真的空 → 才回 BLE 直連 */
                ble_hid_mouse_set_app_route(false);
            }
        }
        break;
    }
    case GUI_APP_MSG_ONRESUME:
        watch_system_mouse_resume();
        break;
    case GUI_APP_MSG_ONPAUSE:
        watch_system_mouse_pause();
        break;
    case GUI_APP_MSG_ONSTOP:
        /* P3:standalone 滑鼠 app 退出 → 框架會重跑 Main 重建錶盤(連同它自己的 instruction_list +
           overlay)。所以這裡【絕對不要】碰共享 instruction_list —— deinit / restore_base / hide
           overlay 都會打到 Main 剛重建好的新清單(實測:晚到的 deinit 拆掉 Main 新清單 → bar 消失
           + 後續觸控 i2c err → HCPU WDT1 凍結重開)。滑鼠 lazy 那份由 Main 重跑 create 時 idempotent
           清掉。這裡只通知電腦收 skaibar(0x0C)+ 清單設備旗標。 */
        {
            extern void instruction_list_skaibar_dismiss_notify_only(void);
            instruction_list_skaibar_dismiss_notify_only();
        }
        hid_mouse_destroy(); /* 旗標已清 → 其內 instruction_list_bar_device_dismiss 變 no-op,不碰共享清單 */
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
                   APP_ID_MOUSE, app_main, 1);

#endif /* APP_ID_MOUSE */

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
