/**
 ******************************************************************************
 * @file   app_timer.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*********************
 *      INCLUDES
 *********************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "arc_scroll.h"
#include "app_mainmenu.h"
#include "ui_img_helper.h"
#include "bloc_control.h"
#include "bloc_motion_tracking.h"
#include "bloc_motor.h"
#include "lv_simplified_obj.h"
#include "bloc_setting.h"
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
#include "watch_global_data.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif

#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#endif
#define DBG_TAG "app.timer"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_TIMER

// Exercise app style constants
#define LIST_TIMER_ACCENT_COLOR (0xFF9500)  /* iOS 風格橘 — 同時用於 title 跟 pie indicator */

// 圓形圖示列表佈局常數（與 app_exercise.c 同樣的視覺規格）
#define TIMER_OPTION_COUNT 12
#define TIMER_ICON_ITEM_SIZE 80
#define TIMER_ICON_ZOOM_CENTER 256
#define TIMER_ICON_ZOOM_MIN 128
#define TIMER_ICON_OPA_CENTER LV_OPA_COVER
#define TIMER_ICON_OPA_MIN LV_OPA_30
#define TIMER_ICON_SLOT_HEIGHT (TIMER_ICON_ITEM_SIZE + 10)
#define TIMER_ICON_SLOT_ANGLE_DEG 36
#define TIMER_ICON_ARC_RADIUS 200
/* zoom / opa 量化步階：值越大，set_size / arc_width / arc_opa 呼叫越少（越省 CPU），
 * 但視覺上跳得越粗。zoom 32 → size 每 ~10px 跳一階；opa 32 → 每 ~12% 跳一階 */
#define TIMER_ICON_ZOOM_STEP 16
#define TIMER_ICON_OPA_STEP  16

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

/**
 * @brief Structure to hold all UI components of the timer app
 */
typedef struct
{
    lv_obj_t *bg;                  // 背景容器
    lv_obj_t *list_container;      // 列表外層容器
    lv_obj_t *timer_list;          // List of timer duration options
    lv_obj_t *timer_title;         // 標題
    lv_obj_t *timer_label;         // Label showing countdown time
    lv_obj_t *countdown_screen;    // 倒計時界面
    lv_obj_t *pause_button;        // Button to pause/resume timer
    lv_obj_t *stop_button;         // Button to stop timer
    lv_obj_t *current_button;      // Currently selected button in list
} timer_ui_t;

typedef struct
{
    lv_ex_data_t *ui_data;
    rt_timer_t countdown_timer; // RT-Thread timer for countdown
    uint32_t remaining_time;    // Remaining time in seconds
    bool is_paused;             // Whether timer is paused
} app_timer_data_ctx_t;

static timer_ui_t ui; // All UI components
static app_timer_data_ctx_t app_timer_data_ctx = {0};

LV_IMG_DECLARE(img_media_play);
LV_IMG_DECLARE(img_media_pause);
LV_IMG_DECLARE(img_media_previous); // 用作重啟圖標
LV_IMG_DECLARE(img_pause);
LV_IMG_DECLARE(app_icon_frame);     // 中央選中項目背後的 icon 框（仿 lv_instruction_list_layout.c）

static const char *timer_options[] = {
    "30 secs",
    "1 min",
    "2 mins",
    "3 mins",
    "4 mins",
    "5 mins",
    "10 mins",
    "15 mins",
    "20 mins",
    "25 mins",
    "30 mins",
    "1 hour",
    ""};

/* 各 timer option 對應的秒數，用來算圓餅圖填充比例（全滿 = 3600 秒 = 1 hour）。
 * Index 與 timer_options[] 一一對應 */
static const uint32_t timer_seconds[TIMER_OPTION_COUNT] = {
    30,    /* "30 secs" */
    60,    /* "1 min"   */
    120,   /* "2 mins"  */
    180,   /* "3 mins"  */
    240,   /* "4 mins"  */
    300,   /* "5 mins"  */
    600,   /* "10 mins" */
    900,   /* "15 mins" */
    1200,  /* "20 mins" */
    1500,  /* "25 mins" */
    1800,  /* "30 mins" */
    3600,  /* "1 hour"  */
};

/* 圓形 icon 排列：仿 app_exercise.c 的做法
 *   - icons 是 list 的 floating child，用 lv_obj_set_pos 直接放在環上
 *   - 另外建 N 個透明 snap_targets 提供 LVGL snap-to-center 用的線性 y anchor
 *   - labels 是 icon 的 child，跟著 icon 移動，預設藏起來只顯示選中那個
 *   - frames 是 icon 的 child（z-order 在 pie 後面），平常藏起來只顯示中央那個 */
static lv_obj_t *timer_icons[TIMER_OPTION_COUNT] = {0};
static lv_obj_t *timer_icon_pies[TIMER_OPTION_COUNT] = {0};
static lv_obj_t *timer_icon_frames[TIMER_OPTION_COUNT] = {0};
static lv_obj_t *timer_name_labels[TIMER_OPTION_COUNT] = {0};

/* 量化後的 zoom/opa 上次套用值 — 避免每個 scroll event 都重複呼叫 set_size /
 * set_arc_width / set_arc_opa / set_zoom 觸發 invalidate 重畫填滿 disk arc。
 * 量化步階：zoom = 8（target_size 差 2-3px 視覺幾乎一樣），opa = 8 */
static int16_t timer_icon_last_zoom[TIMER_OPTION_COUNT] = {0};
static lv_opa_t timer_icon_last_opa[TIMER_OPTION_COUNT] = {0};

/* Forward declarations for functions */
static void update_timer_label(void);
static void show_counter_listview(void);
static lv_obj_t *create_timer_list(lv_obj_t *parent);
static void refresh_ui(lv_obj_t *_, void *para);
static void show_timeout_notification(void);
static void timer_list_nav_control(int8_t action);
static void refresh_pause_button_icon(void);

// 追蹤當前選中的timer選項索引
static int16_t selected_timer_index = 0;

// Create data bindings
static void create_timer_data_bindings(void)
{
    if (app_timer_data_ctx.ui_data)
        return;
    lv_ex_binding_t binding;
    app_timer_data_ctx.ui_data = lv_ex_data_create("app.timer", LV_EX_DATA_POINTER);
    binding.target = ui.timer_label;
    binding.arg_type = LV_EX_DATA_POINTER;
    binding.setter = (void *)refresh_ui;
    lv_ex_bind_data(app_timer_data_ctx.ui_data, &binding);
}

// Clean up data bindings
static void cleanup_timer_data_bindings(void)
{
    lv_ex_data_t *data;
    if (app_timer_data_ctx.ui_data)
    {
        data = app_timer_data_ctx.ui_data;
        app_timer_data_ctx.ui_data = NULL;
        lv_ex_data_delete(data);
    }
}

/**
 * @brief Safely remove the countdown timer
 */
static void remove_countdown_timer(void)
{
    if (app_timer_data_ctx.countdown_timer)
    {
        rt_timer_stop(app_timer_data_ctx.countdown_timer);
        rt_timer_delete(app_timer_data_ctx.countdown_timer);
        app_timer_data_ctx.countdown_timer = NULL;
    }
}

static bool _timeout = false;

/**
 * @brief Callback for the countdown timer
 */
static void countdown_timer_cb(void *parameter)
{
    if (!app_timer_data_ctx.is_paused && app_timer_data_ctx.remaining_time > 0)
    {
        app_timer_data_ctx.remaining_time--;

        if (app_timer_data_ctx.ui_data)
        {
            lv_ex_data_set_value(app_timer_data_ctx.ui_data, (void *)NULL);
        }

        if (app_timer_data_ctx.remaining_time == 0)
        {
            _timeout = true;
            watch_system_interact(INTERACT_TIMER_REMINDER, NULL);
            LOG_D("Timer finished");
        }
        LOG_D("Remaining time: %d", app_timer_data_ctx.remaining_time);
    }
}

/**
 * @brief Update the timer display label
 */
static void update_timer_label(void)
{
    if (!lv_obj_is_valid(ui.timer_label))
    {
        return;
    }

    uint32_t hours = app_timer_data_ctx.remaining_time / 3600;
    uint32_t minutes = (app_timer_data_ctx.remaining_time % 3600) / 60;
    uint32_t seconds = app_timer_data_ctx.remaining_time % 60;
    char time_str[9];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hours, minutes, seconds);
    lv_label_set_text(ui.timer_label, time_str);
}

/**
 * @brief Show the timer view (after selecting a time)
 */
static void show_new_timer_view(const char *text)
{
    // 隱藏列表容器
    if (lv_obj_is_valid(ui.list_container))
    {
        lv_obj_add_flag(ui.list_container, LV_OBJ_FLAG_HIDDEN);
    }

    // 顯示倒計時界面
    if (lv_obj_is_valid(ui.countdown_screen))
    {
        lv_obj_clear_flag(ui.countdown_screen, LV_OBJ_FLAG_HIDDEN);
    }

    // 切換控制模式
    set_open_control_options(false);
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_widgets_control = NULL;
    lvgl_msg_handler.handle_nav_bar_control = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
#endif
}

/**
 * @brief Create and start a new countdown timer
 */
static void create_countdown_timer(void)
{
    remove_countdown_timer();
    app_timer_data_ctx.countdown_timer = rt_timer_create("countdown_timer",
                                                         countdown_timer_cb,
                                                         NULL,
                                                         RT_TICK_PER_SECOND,
                                                         RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (app_timer_data_ctx.countdown_timer)
    {
        rt_timer_start(app_timer_data_ctx.countdown_timer);
        app_timer_data_ctx.is_paused = false;
    }
    else
    {
        // Handle timer creation failure
        LOG_E("Failed to create countdown timer");
    }
}

/**
 * @brief Update UI when timer data changes
 */
static void refresh_ui(lv_obj_t *_, void *para)
{
    if (app_timer_data_ctx.remaining_time == 0)
    {
        show_counter_listview();
        show_timeout_notification();
        motor_pattern_timer_reminder();
    }
    else
    {
        update_timer_label();
    }
}

/**
 * @brief Handle tap on a timer option button
 */
static void tap_button(lv_obj_t *btn)
{
    if (!lv_obj_is_valid(btn))
    {
        return;
    }

    /* user_data 在 create_timer_list 是設成 (intptr_t)i 編碼的索引 */
    int idx = (int)(intptr_t)lv_obj_get_user_data(btn);
    if (idx >= 0 && idx < TIMER_OPTION_COUNT)
    {
        app_timer_data_ctx.remaining_time = timer_seconds[idx];
    }
    else
    {
        app_timer_data_ctx.remaining_time = 0;
        idx = 0;
    }

    create_timer_data_bindings();
    update_timer_label();
    show_new_timer_view(timer_options[idx]);
    create_countdown_timer();
}

static void show_counter_listview(void)
{
    // 隱藏倒計時界面
    if (lv_obj_is_valid(ui.countdown_screen))
    {
        lv_obj_add_flag(ui.countdown_screen, LV_OBJ_FLAG_HIDDEN);
    }

    // 顯示列表容器
    if (lv_obj_is_valid(ui.list_container))
    {
        lv_obj_clear_flag(ui.list_container, LV_OBJ_FLAG_HIDDEN);
    }

    // 重置標題
    if (lv_obj_is_valid(ui.timer_title))
    {
        lv_label_set_text(ui.timer_title, "Timer");
    }

    // 切換到列表滾動控制模式
    set_open_control_options(false);
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_widgets_control = NULL;
    lvgl_msg_handler.handle_nav_bar_control = timer_list_nav_control;
#endif
}

static void refresh_timer_status(void)
{
    if (app_timer_data_ctx.countdown_timer)
    {
        if (app_timer_data_ctx.is_paused)
        {
            rt_timer_stop(app_timer_data_ctx.countdown_timer);
        }
        else
        {
            rt_timer_start(app_timer_data_ctx.countdown_timer);
        }
    }
}

static void refresh_pause_button_icon(void)
{
    if (lv_obj_is_valid(ui.pause_button))
    {
        lv_obj_t *img = lv_obj_get_child(ui.pause_button, 0);
        if (lv_obj_is_valid(img))
        {
            lv_img_set_src(img, app_timer_data_ctx.is_paused ? &img_media_play : &img_media_pause);
        }
    }
}

// 暫停按鈕事件處理
static void pause_button_event_cb(lv_event_t *e)
{
    app_timer_data_ctx.is_paused = !app_timer_data_ctx.is_paused;
    refresh_timer_status();
    refresh_pause_button_icon();
}

// 停止按鈕事件處理
static void stop_button_event_cb(lv_event_t *e)
{
    remove_countdown_timer();
    show_counter_listview();
    app_timer_data_ctx.remaining_time = 0;
}

static void timer_list_event_cb(lv_event_t *e)
{
    if (LV_EVENT_CLICKED == lv_event_get_code(e))
    {
        tap_button(lv_event_get_target(e));
    }
}

static void press_cb(uint8_t press)
{
    LOG_D("Timer press event: %d", press);
    if (press && !app_timer_data_ctx.countdown_timer && ui.current_button && lv_obj_is_valid(ui.current_button))
    {
        motor_pattern_touchpad_slide();
        tap_button(ui.current_button);
    }
}

static void handle_tap_event(void)
{
    LOG_D("Timer tap event");
    if (app_timer_data_ctx.countdown_timer)
    {
        pause_button_event_cb(NULL);
    }
    else if (ui.current_button && lv_obj_is_valid(ui.current_button))
    {
        tap_button(ui.current_button);
    }
}

/* 右側弧形觸控滾動 — tap 時把 click 派給 press 點所在的 icon；
 * 落在空白處則 fallback 給目前選中 icon（點邊緣 = 啟動選中項目） */
static lv_obj_t *timer_arc_tap_cb(lv_point_t pt, void *ctx)
{
    (void)ctx;
    for (int i = 0; i < TIMER_OPTION_COUNT; i++)
    {
        if (timer_icons[i] == NULL) continue;
        if (!lv_obj_is_valid(timer_icons[i])) continue;
        if (lv_obj_has_flag(timer_icons[i], LV_OBJ_FLAG_HIDDEN)) continue;
        lv_area_t a;
        lv_obj_get_coords(timer_icons[i], &a);
        if (pt.x >= a.x1 && pt.x <= a.x2 && pt.y >= a.y1 && pt.y <= a.y2)
        {
            return timer_icons[i];
        }
    }
    if (selected_timer_index >= 0 && selected_timer_index < TIMER_OPTION_COUNT &&
        timer_icons[selected_timer_index] != NULL &&
        lv_obj_is_valid(timer_icons[selected_timer_index]) &&
        !lv_obj_has_flag(timer_icons[selected_timer_index], LV_OBJ_FLAG_HIDDEN))
    {
        return timer_icons[selected_timer_index];
    }
    return NULL;
}

static lv_obj_t *timer_arc_snap_cb(void *ctx)
{
    (void)ctx;
    if (ui.timer_list == NULL || !lv_obj_is_valid(ui.timer_list)) return NULL;
    if (selected_timer_index < 0 || selected_timer_index >= TIMER_OPTION_COUNT) return NULL;
    return lv_obj_get_child(ui.timer_list, selected_timer_index);
}

/* label 偏到螢幕左側、跑出 icon bbox，無法接到 click。
 * 在 label 視覺位置疊一個透明 overlay 接 click，再轉發給目前選中的 icon */
static void label_tap_zone_click_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (selected_timer_index < 0 || selected_timer_index >= TIMER_OPTION_COUNT) return;
    lv_obj_t *icon = timer_icons[selected_timer_index];
    if (icon != NULL && lv_obj_is_valid(icon) &&
        !lv_obj_has_flag(icon, LV_OBJ_FLAG_HIDDEN))
    {
        lv_event_send(icon, LV_EVENT_CLICKED, NULL);
    }
}

/* 弧形排列：仿 app_exercise.c::apply_circular_layout
 *   offset_angle = (scroll_y - base_scroll) / SLOT × angle_per_slot
 *   current_angle_i = i × angle_per_slot - offset_angle
 *   icon_screen_pos = (cx + R·cos(angle), cy + R·sin(angle)) */
static void apply_circular_layout(lv_obj_t *list)
{
    const int32_t cx = LV_HOR_RES / 2 - 20;
    const int32_t cy = LV_VER_RES / 2;
    const float angle_per_slot = (float)TIMER_ICON_SLOT_ANGLE_DEG * (M_PI / 180.0f);
    const lv_coord_t list_x1 = list->coords.x1;
    const lv_coord_t list_y1 = list->coords.y1;
    const lv_coord_t pad_top = lv_obj_get_style_pad_top(list, LV_PART_MAIN);
    const lv_coord_t pad_left = lv_obj_get_style_pad_left(list, LV_PART_MAIN);

    const int32_t base_scroll = list_y1 + pad_top + TIMER_ICON_ITEM_SIZE / 2 - cy;
    /* 拖動時 LVGL 的 elastic overshoot 會讓 scroll_y 暫時衝出正常範圍，
     * offset_angle 算出來會非常大 → 全部 icon 的 abs_angle > 90° 被一起藏起來，
     * 直到放開彈回。夾在 [base_scroll, base_scroll + (N-1)*SLOT] 內，
     * 確保任何 scroll 位置至少有一個 icon 落在 ±90° 內，不會整片消失 */
    lv_coord_t scroll_y = lv_obj_get_scroll_y(list);
    const int32_t scroll_y_max = base_scroll +
                                  (TIMER_OPTION_COUNT - 1) * TIMER_ICON_SLOT_HEIGHT;
    if (scroll_y < base_scroll) scroll_y = base_scroll;
    if (scroll_y > scroll_y_max) scroll_y = scroll_y_max;
    const float scroll_in_slots = (float)(scroll_y - base_scroll) / TIMER_ICON_SLOT_HEIGHT;
    const float offset_angle = scroll_in_slots * angle_per_slot;

    int closest_i = 0;
    float min_abs_angle = M_PI;

    for (int i = 0; i < TIMER_OPTION_COUNT; i++)
    {
        if (timer_icons[i] == NULL) continue;

        float current_angle = (float)i * angle_per_slot - offset_angle;
        float abs_angle = fabsf(current_angle);

        if (abs_angle < min_abs_angle)
        {
            min_abs_angle = abs_angle;
            closest_i = i;
        }

        if (abs_angle > M_PI / 2.0f)
        {
            lv_obj_add_flag(timer_icons[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(timer_icons[i], LV_OBJ_FLAG_HIDDEN);

        /* 中央 = ZOOM_CENTER（80px 全大），邊緣 = ZOOM_MIN（≈40px），用 cos(angle) 插值。
         * zoom 量化到 TIMER_ICON_ZOOM_STEP 一階 → 大多數 scroll event 落在同一階，
         * 跳過 set_size / set_arc_width / set_zoom（這些都會 invalidate 觸發
         * disk arc 重畫，最貴）。提高 STEP → 更省 CPU，視覺跳得粗一點 */
        int32_t zoom_range = TIMER_ICON_ZOOM_CENTER - TIMER_ICON_ZOOM_MIN;
        int32_t zoom_raw = TIMER_ICON_ZOOM_MIN + (int32_t)(zoom_range * cosf(abs_angle));
        int32_t zoom = ((zoom_raw + TIMER_ICON_ZOOM_STEP / 2) / TIMER_ICON_ZOOM_STEP)
                       * TIMER_ICON_ZOOM_STEP;
        if (zoom < TIMER_ICON_ZOOM_MIN) zoom = TIMER_ICON_ZOOM_MIN;
        if (zoom > TIMER_ICON_ZOOM_CENTER) zoom = TIMER_ICON_ZOOM_CENTER;
        int32_t target_size = (TIMER_ICON_ITEM_SIZE * zoom) / 256;
        if (target_size < 2) target_size = 2;

        lv_obj_t *pie = timer_icon_pies[i];
        bool zoom_changed = ((int16_t)zoom != timer_icon_last_zoom[i]);
        if (zoom_changed)
        {
            lv_obj_set_size(timer_icons[i], target_size, target_size);
            if (pie != NULL && lv_obj_is_valid(pie))
            {
                int32_t new_radius = target_size / 2;
                lv_obj_set_size(pie, target_size, target_size);
                lv_obj_center(pie);
                lv_obj_set_style_arc_width(pie, new_radius, LV_PART_MAIN);
                lv_obj_set_style_arc_width(pie, new_radius, LV_PART_INDICATOR);
            }
            if (timer_icon_frames[i] != NULL && lv_obj_is_valid(timer_icon_frames[i]))
            {
                lv_img_set_zoom(timer_icon_frames[i], zoom);
                lv_obj_center(timer_icon_frames[i]);
            }
            timer_icon_last_zoom[i] = (int16_t)zoom;
        }

        /* 位置每個 event 都要更新（手指移動的視覺反饋來源） */
        int32_t icon_sx = cx + (int32_t)(TIMER_ICON_ARC_RADIUS * cosf(current_angle));
        int32_t icon_sy = cy + (int32_t)(TIMER_ICON_ARC_RADIUS * sinf(current_angle));
        int32_t local_x = icon_sx - list_x1 - pad_left - target_size / 2;
        int32_t local_y = icon_sy - list_y1 - pad_top - target_size / 2;
        lv_obj_set_pos(timer_icons[i], local_x, local_y);

        /* opa 同樣量化（TIMER_ICON_OPA_STEP），沒變就不重設（arc_opa 也會 invalidate 重畫）。
         * 必須在 int32_t 算完並 clamp 才 cast — 直接 cast 成 uint8_t 在量化值 = 256
         * 時會 wrap 成 0 → 中央最亮那段反而被打到 OPA_MIN（突然變暗）*/
        int32_t opa_range = TIMER_ICON_OPA_CENTER - TIMER_ICON_OPA_MIN;
        int32_t opa_raw = TIMER_ICON_OPA_MIN + (int32_t)(opa_range * cosf(abs_angle));
        int32_t opa_q = ((opa_raw + TIMER_ICON_OPA_STEP / 2) / TIMER_ICON_OPA_STEP)
                        * TIMER_ICON_OPA_STEP;
        if (opa_q > LV_OPA_COVER) opa_q = LV_OPA_COVER;
        if (opa_q < TIMER_ICON_OPA_MIN) opa_q = TIMER_ICON_OPA_MIN;
        lv_opa_t opa = (lv_opa_t)opa_q;
        if (opa != timer_icon_last_opa[i])
        {
            if (pie != NULL && lv_obj_is_valid(pie))
            {
                /* bg ring 原本 70% opa，乘上整體 opa 比例 */
                lv_opa_t main_opa = (lv_opa_t)((opa * (uint16_t)LV_OPA_70) / 255);
                lv_obj_set_style_arc_opa(pie, main_opa, LV_PART_MAIN);
                lv_obj_set_style_arc_opa(pie, opa, LV_PART_INDICATOR);
            }
            timer_icon_last_opa[i] = opa;
        }
    }

    if (closest_i != selected_timer_index)
    {
        selected_timer_index = closest_i;
    }
    if (selected_timer_index >= 0 && selected_timer_index < TIMER_OPTION_COUNT)
    {
        ui.current_button = timer_icons[selected_timer_index];
    }
    for (int i = 0; i < TIMER_OPTION_COUNT; i++)
    {
        bool is_center = (i == closest_i && min_abs_angle <= M_PI / 2.0f);
        if (timer_name_labels[i] != NULL)
        {
            if (is_center)
                lv_obj_clear_flag(timer_name_labels[i], LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(timer_name_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
        /* frame 只在中央那個顯示（仿 lv_instruction_list_layout.c 的 app_icon_shadow） */
        if (timer_icon_frames[i] != NULL)
        {
            if (is_center)
                lv_obj_clear_flag(timer_icon_frames[i], LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(timer_icon_frames[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void scroll_timer_list(lv_obj_t *list)
{
    apply_circular_layout(list);
}

// timer列表滾動事件處理
static void timer_list_scroll_event_cb(lv_event_t *e)
{
    lv_obj_t *list = lv_event_get_target(e);
    if (list == NULL)
        return;

    switch (e->code)
    {
    case LV_EVENT_SCROLL:
        scroll_timer_list(list);
        break;
    default:
        break;
    }
}

// 滾動到指定的timer選項 - 用於體感控制
static void scroll_timer_list_to_index(int8_t index)
{
    if (index < 0 || index >= TIMER_OPTION_COUNT)
    {
        LOG_W("Timer index out of bounds: %d", index);
        return;
    }

    if (!lv_obj_is_valid(ui.timer_list))
    {
        LOG_E("Timer list is not valid");
        return;
    }

    LOG_D("Scrolling to timer list index: %d", index);
    lv_obj_t *target_item = lv_obj_get_child(ui.timer_list, index);
    if (target_item && lv_obj_is_valid(target_item))
    {
        lv_obj_scroll_to_view(target_item, LV_ANIM_ON);
    }
}

// 體感控制用於滾動timer列表
static void timer_list_nav_control(int8_t action)
{
    scroll_timer_list_to_index(action);
}

// 創建 timer 列表 - 與 app_exercise.c 相同的圓形圖示佈局
static lv_obj_t *create_timer_list(lv_obj_t *parent)
{
    ui.current_button = NULL;
    /* 重置 zoom/opa cache（icon 重新建立，第一次 apply 必須套用一輪） */
    memset(timer_icon_last_zoom, 0, sizeof(timer_icon_last_zoom));
    memset(timer_icon_last_opa, 0, sizeof(timer_icon_last_opa));

    // 創建列表外層容器
    lv_obj_t *list_container = lv_obj_create(parent);
    lv_obj_set_size(list_container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(list_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_container, 0, 0);
    lv_obj_set_style_pad_all(list_container, 0, 0);
    lv_obj_clear_flag(list_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(list_container, LV_ALIGN_TOP_MID, 0, 0);

    /* List 是全螢幕的 scrollable 容器（無 flex），仿
     * app_exercise.c::create_workout_list 的做法 */
    lv_obj_t *list = lv_obj_create(list_container);
    lv_obj_set_size(list, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(list, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(list, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_pad_ver(list, LV_VER_RES / 2, 0);
    lv_obj_add_event_cb(list, timer_list_scroll_event_cb, LV_EVENT_ALL, NULL);
    ui.timer_list = list;

    /* N 個透明 snap_targets（non-floating），只負責提供 LVGL 的 snap-to-center
     * anchors，視覺上不顯示 */
    for (int i = 0; i < TIMER_OPTION_COUNT; i++)
    {
        lv_obj_t *snap_target = lv_obj_create(list);
        lv_obj_set_size(snap_target, TIMER_ICON_ITEM_SIZE, TIMER_ICON_ITEM_SIZE);
        lv_obj_set_pos(snap_target, 0, i * TIMER_ICON_SLOT_HEIGHT);
        lv_obj_set_style_bg_opa(snap_target, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(snap_target, 0, 0);
        lv_obj_set_style_pad_all(snap_target, 0, 0);
        lv_obj_clear_flag(snap_target, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(snap_target, LV_OBJ_FLAG_SCROLLABLE);
    }

    /* N 個 visible icons：list 的 floating child（不影響 scroll content size）。
     * 每個 icon 是「透明 generic 容器（hit zone）+ 內嵌 lv_arc 圓餅圖（純視覺）」。
     * lv_obj_remove_style_all 把 default theme 樣式（rounded bg、padding 等）洗掉，
     * 才不會在 arc 旁邊再多一個圓角矩形 */
    const int32_t pie_radius = TIMER_ICON_ITEM_SIZE / 2;
    for (int i = 0; i < TIMER_OPTION_COUNT; i++)
    {
        timer_icons[i] = lv_obj_create(list);
        lv_obj_remove_style_all(timer_icons[i]);
        lv_obj_set_size(timer_icons[i], TIMER_ICON_ITEM_SIZE, TIMER_ICON_ITEM_SIZE);
        lv_obj_clear_flag(timer_icons[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(timer_icons[i], LV_OBJ_FLAG_FLOATING);
        lv_obj_add_flag(timer_icons[i], LV_OBJ_FLAG_OVERFLOW_VISIBLE);
        lv_obj_add_flag(timer_icons[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(timer_icons[i], (void *)(intptr_t)i);
        lv_obj_add_event_cb(timer_icons[i], timer_list_event_cb,
                            LV_EVENT_CLICKED, NULL);

        /* 先建 frame（child 0，最底層）→ pie 蓋在它上面。
         * 仿 lv_instruction_list_layout.c 的 app_icon_shadow 做法：平常藏起來，
         * 在 apply_circular_layout 結尾才把 closest_i 對應的 frame 顯示 */
        timer_icon_frames[i] = lv_img_create(timer_icons[i]);
        lv_img_set_src(timer_icon_frames[i], &app_icon_frame);
        lv_obj_center(timer_icon_frames[i]);
        lv_obj_clear_flag(timer_icon_frames[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(timer_icon_frames[i], LV_OBJ_FLAG_HIDDEN);

        /* 圓餅圖：bg 是滿圓灰盤，indicator 是從 12 點鐘順時針的 pie wedge。
         * arc_width = radius → 整個 disk 填滿（不是中空 ring）。
         * 比例 = 該選項秒數 / 3600（1 hour 為全滿） */
        timer_icon_pies[i] = lv_arc_create(timer_icons[i]);
        lv_obj_t *pie = timer_icon_pies[i];
        lv_obj_remove_style_all(pie);
        lv_obj_set_size(pie, TIMER_ICON_ITEM_SIZE, TIMER_ICON_ITEM_SIZE);
        lv_obj_center(pie);
        lv_obj_clear_flag(pie, LV_OBJ_FLAG_CLICKABLE); /* click/drag 由 container 處理 */
        lv_obj_clear_flag(pie, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_arc_width(pie, pie_radius, LV_PART_MAIN);
        lv_obj_set_style_arc_width(pie, pie_radius, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(pie, lv_color_hex(0x404040), LV_PART_MAIN);
        lv_obj_set_style_arc_opa(pie, LV_OPA_70, LV_PART_MAIN);
        lv_obj_set_style_arc_color(pie,
                                   lv_color_hex(LIST_TIMER_ACCENT_COLOR),
                                   LV_PART_INDICATOR);
        lv_obj_set_style_arc_opa(pie, LV_OPA_COVER, LV_PART_INDICATOR);

        /* 0° 在 3 點鐘方向，rotation=270 把起點轉到 12 點鐘，順時針填充 */
        lv_arc_set_rotation(pie, 270);
        lv_arc_set_bg_angles(pie, 0, 360);
        uint32_t fill_deg32 = (timer_seconds[i] * 360u + 1800u) / 3600u; /* 四捨五入 */
        if (fill_deg32 == 0) fill_deg32 = 1;       /* 太小也至少露一條縫 */
        if (fill_deg32 > 360) fill_deg32 = 360;
        lv_arc_set_angles(pie, 0, (uint16_t)fill_deg32);

        /* label 改掛在 list_container（不滾、不轉），不再跟著 icon 旋轉跑。
         * 可見性還是用 closest_i 在 apply_circular_layout 切，icon 指到誰
         * 就 show 誰、其他 hide。click 走 label_tap_zone overlay，不需要 bubble。
         * 螢幕位置：對齊 LEFT_MID 偏 35px → 跟舊版「icon 在 angle=0 時 label 左
         * 邊在螢幕 x=35」對齊（cx + R - icon_w/2 - 338 = 35） */
        timer_name_labels[i] = lv_label_create(list_container);
        lv_label_set_text(timer_name_labels[i], timer_options[i]);
        lv_obj_set_style_text_color(timer_name_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(timer_name_labels[i],
                                   LV_EXT_FONT_GET(get_system_font_size(1)), 0);
        lv_obj_set_width(timer_name_labels[i], 250);
        lv_obj_set_style_text_align(timer_name_labels[i],
                                    LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(timer_name_labels[i], LV_ALIGN_LEFT_MID, 35, 0);
        if (i != 0)
        {
            lv_obj_add_flag(timer_name_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    lv_obj_update_layout(list);

    /* 預設選中第一個項目 */
    selected_timer_index = 0;
    lv_obj_scroll_to_view(lv_obj_get_child(list, 0), LV_ANIM_OFF);
    apply_circular_layout(list);

    /* 頂端中央標題 */
    ui.timer_title = lv_label_create(list_container);
    lv_label_set_text(ui.timer_title, "Timer");
    lv_obj_set_style_text_align(ui.timer_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(ui.timer_title,
                               LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(ui.timer_title,
                                lv_color_hex(LIST_TIMER_ACCENT_COLOR), 0);
    lv_obj_align(ui.timer_title, LV_ALIGN_TOP_MID, 0, 30);

    /* 右側弧形觸控滾動 — 用共用模組 common/arc_scroll.h */
    arc_scroll_config_t arc_cfg = {
        .parent          = list_container,
        .list            = list,
        .slot_height_px  = TIMER_ICON_SLOT_HEIGHT,
        .item_height_px  = TIMER_ICON_ITEM_SIZE,
        .slot_angle_deg  = TIMER_ICON_SLOT_ANGLE_DEG,
        .item_count      = TIMER_OPTION_COUNT,
        .band_thickness  = 90,
        .lock_ancestors  = false,
        .tap_cb          = timer_arc_tap_cb,
        .snap_cb         = timer_arc_snap_cb,
        .ctx             = NULL,
    };
    arc_scroll_create(&arc_cfg);

    /* label 文字區塊的 click overlay：透明、寬度涵蓋 label 視覺範圍。
     * 加在 arc_zone 之後 → z-order 在 arc_zone 上面，左側點擊優先給 overlay
     * 接走（arc_zone 反正只接受右側弧帶內的 hit_test）*/
    lv_obj_t *label_tap_zone = lv_obj_create(list_container);
    lv_obj_set_size(label_tap_zone, 250, 80);
    lv_obj_align(label_tap_zone, LV_ALIGN_LEFT_MID, 27, 0);
    lv_obj_set_style_bg_opa(label_tap_zone, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(label_tap_zone, 0, 0);
    lv_obj_set_style_pad_all(label_tap_zone, 0, 0);
    lv_obj_clear_flag(label_tap_zone, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(label_tap_zone, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(label_tap_zone, label_tap_zone_click_cb,
                        LV_EVENT_CLICKED, NULL);

    return list_container;
}

// 創建倒計時界面 - 與exercise workout screen相同的樣式
static lv_obj_t *create_countdown_screen(lv_obj_t *parent)
{
    lv_obj_t *countdown_screen = lv_obj_create(parent);
    lv_obj_set_size(countdown_screen, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(countdown_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(countdown_screen, LV_OPA_100, 0);
    lv_obj_set_style_border_width(countdown_screen, 0, 0);
    lv_obj_clear_flag(countdown_screen, LV_OBJ_FLAG_SCROLLABLE);

    // 創建計時標籤 - 大字體顯示時間
    ui.timer_label = lv_label_create(countdown_screen);
    lv_obj_set_style_text_font(ui.timer_label, LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_set_style_text_color(ui.timer_label, lv_color_white(), 0);
    lv_obj_align(ui.timer_label, LV_ALIGN_TOP_MID, 0, 120);
    lv_label_set_text(ui.timer_label, "00:00:00");

    // 暫停按鈕 - 與exercise app相同樣式
    ui.pause_button = lv_btn_create(countdown_screen);
    lv_obj_set_size(ui.pause_button, 120, 80);
    lv_obj_set_style_radius(ui.pause_button, 50, 0);
    lv_obj_set_style_bg_color(ui.pause_button, lv_color_hex(0x333333), 0);
    lv_obj_align(ui.pause_button, LV_ALIGN_TOP_RIGHT, -40, 280);

    lv_obj_t *pause_img = lv_img_create(ui.pause_button);
    lv_img_set_src(pause_img, &img_media_pause);
    lv_img_set_zoom(pause_img, 128);
    lv_obj_center(pause_img);

    lv_obj_add_event_cb(ui.pause_button, pause_button_event_cb, LV_EVENT_CLICKED, NULL);

    // 停止按鈕 - 與exercise app相同樣式
    ui.stop_button = lv_btn_create(countdown_screen);
    lv_obj_set_size(ui.stop_button, 120, 80);
    lv_obj_set_style_radius(ui.stop_button, 50, 0);
    lv_obj_set_style_bg_color(ui.stop_button, lv_color_hex(0x333333), 0);
    lv_obj_align(ui.stop_button, LV_ALIGN_TOP_LEFT, 40, 280);

    lv_obj_t *stop_label = lv_label_create(ui.stop_button);
    lv_label_set_text(stop_label, "STOP");
    lv_obj_set_style_text_color(stop_label, lv_color_white(), 0);
    lv_obj_center(stop_label);

    lv_obj_add_event_cb(ui.stop_button, stop_button_event_cb, LV_EVENT_CLICKED, NULL);

    return countdown_screen;
}

static lv_obj_t *timeout_msg_box = NULL;

static void close_timeout_notification_cb(lv_event_t *e)
{
    if (timeout_msg_box && lv_obj_is_valid(timeout_msg_box))
    {
        // 獲取並刪除遮罩層
        lv_obj_t *mask = (lv_obj_t *)lv_obj_get_user_data(timeout_msg_box);
        if (mask && lv_obj_is_valid(mask))
        {
            lv_obj_del(mask); // 刪除遮罩層會同時刪除其子元素（包括timeout_msg_box）
        }
        else
        {
            lv_obj_del(timeout_msg_box); // 以防萬一，如果找不到遮罩層
        }
        timeout_msg_box = NULL;
    }
    _timeout = false;
    remove_countdown_timer();
    show_counter_listview();
    setting_provider.set_power_save_mode(1);
}

static void show_timeout_notification(void)
{
    if (timeout_msg_box && lv_obj_is_valid(timeout_msg_box))
        return;
    
    setting_provider.set_power_save_mode(0);
    // 創建一個全屏遮罩層
    lv_obj_t *mask = lv_obj_create(lv_scr_act());
    lv_obj_set_size(mask, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(mask, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(mask, LV_OPA_60, 0);
    lv_obj_set_style_border_width(mask, 0, 0);
    lv_obj_clear_flag(mask, LV_OBJ_FLAG_SCROLLABLE);

    // 創建主要消息框
    timeout_msg_box = lv_obj_create(mask);
    lv_obj_set_size(timeout_msg_box, LV_PCT(75), LV_PCT(75));
    lv_obj_align(timeout_msg_box, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(timeout_msg_box, 50, 0); // 使用純色背景
    lv_obj_set_style_bg_color(timeout_msg_box, lv_color_hex(0x303040), 0);
    lv_obj_set_style_bg_opa(timeout_msg_box, 240, 0); // 約95%不透明度    // 添加精緻邊框
    lv_obj_set_style_border_width(timeout_msg_box, 2, 0);
    lv_obj_set_style_border_color(timeout_msg_box, lv_color_hex(0x6080FF), 0);
    lv_obj_set_style_border_opa(timeout_msg_box, LV_OPA_40, 0);

    // 添加鬧鐘圖標
    lv_obj_t *alarm_icon = lv_img_create(timeout_msg_box);
    lv_img_set_src(alarm_icon, IMG_ALARM_2);
    lv_obj_align(alarm_icon, LV_ALIGN_TOP_MID, 0, 40);

    // 添加主標題
    lv_obj_t *title = lv_label_create(timeout_msg_box);
    lv_label_set_text(title, "Time's Up!");
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align_to(title, alarm_icon, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    // 添加副標題
    lv_obj_t *subtitle = lv_label_create(timeout_msg_box);
    lv_label_set_text(subtitle, "Your timer has finished");
    lv_obj_set_style_text_font(subtitle, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(subtitle, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    // 創建一個精美的關閉按鈕
    lv_obj_t *close_btn = lv_btn_create(timeout_msg_box);
    lv_obj_set_size(close_btn, 160, 60);
    lv_obj_align(close_btn, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_obj_set_style_radius(close_btn, 12, 0); // 按鈕漸變背景
    lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x5070DD), 0);
    lv_obj_set_style_bg_opa(close_btn, LV_OPA_100, 0); // 按鈕邊框
    lv_obj_set_style_border_width(close_btn, 1, 0);
    lv_obj_set_style_border_color(close_btn, lv_color_hex(0x80A0FF), 0);
    lv_obj_set_style_border_opa(close_btn, LV_OPA_50, 0);

    // 按下效果
    lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x4060CC), LV_STATE_PRESSED);

    // 添加按鈕文字
    lv_obj_t *btn_label = lv_label_create(close_btn);
    lv_label_set_text(btn_label, "Close");
    lv_obj_set_style_text_font(btn_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(btn_label);

    // 添加點擊事件
    lv_obj_add_event_cb(close_btn, close_timeout_notification_cb, LV_EVENT_CLICKED, NULL);

    // 將整個遮罩添加到 timeout_msg_box 用戶數據中，以便在關閉時一併刪除
    lv_obj_set_user_data(timeout_msg_box, mask);
}

static void create_timer_app_ui(lv_obj_t *parent)
{
    // 創建主背景容器
    ui.bg = common_black_bg(parent);

    /* gaus_clock1_bg 圖底圖 — 加在 ui.bg 第一個 child（最底層）。
     * countdown_screen 自帶不透明黑底會蓋住它，只有列表視圖會看到背景圖 */
    lv_obj_t *bg_img = lv_img_create(ui.bg);
    lv_img_set_src(bg_img, &gaus_clock1_bg);
    lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(bg_img, 512);
    lv_obj_clear_flag(bg_img, LV_OBJ_FLAG_CLICKABLE);

    // 創建列表容器
    ui.list_container = create_timer_list(ui.bg);

    // 創建倒計時界面
    ui.countdown_screen = create_countdown_screen(ui.bg);
    // 默認隱藏倒計時界面
    lv_obj_add_flag(ui.countdown_screen, LV_OBJ_FLAG_HIDDEN);
}

static void on_start(void)
{
    // Initialize UI context
    memset(&ui, 0, sizeof(timer_ui_t));
    create_timer_app_ui(lv_scr_act());

    if (app_timer_data_ctx.countdown_timer)
    {
        if (_timeout)
        {
            LOG_D("Show timeout notification");
            show_timeout_notification();
            motor_pattern_timer_reminder();
        }
        else
        {
            LOG_D("Resume countdown timer");
            create_timer_data_bindings();
            update_timer_label();
            // 顯示倒計時界面
            if (lv_obj_is_valid(ui.countdown_screen))
            {
                lv_obj_clear_flag(ui.countdown_screen, LV_OBJ_FLAG_HIDDEN);
            }
            if (lv_obj_is_valid(ui.list_container))
            {
                lv_obj_add_flag(ui.list_container, LV_OBJ_FLAG_HIDDEN);
            }
            refresh_pause_button_icon();
        }
    }
    else
    {
        LOG_D("Show timer list view");
        show_counter_listview();
    }
}

static void on_pause(void)
{
    refresh_timer_status();

#ifdef BSP_USING_UI_HANDLER
    if (lvgl_msg_handler.handle_nav_bar_control == timer_list_nav_control)
    {
        lvgl_msg_handler.handle_nav_bar_control = NULL;
    }
    if (lvgl_msg_handler.handle_tap_indicator == press_cb)
    {
        lvgl_msg_handler.handle_tap_indicator = NULL;
    }
#endif
    LOG_D("Timer app paused");
}

static void on_resume(void)
{
    refresh_timer_status();

    // 根據當前狀態設置不同的handler
    if (app_timer_data_ctx.countdown_timer)
    {
        // 計時器運行中
        set_open_control_options(false);
        set_free_control_with_arm(false);
#ifdef BSP_USING_UI_HANDLER
        lvgl_msg_handler.handle_tap_event = handle_tap_event;
        lvgl_msg_handler.handle_widgets_control = NULL;
        lvgl_msg_handler.handle_nav_bar_control = NULL;
        lvgl_msg_handler.handle_tap_indicator = NULL;
#endif
    }
    else
    {
        // timer列表模式 - 啟用列表滾動控制
        set_scroll_segment_count(TIMER_OPTION_COUNT);
        set_prev_sensor_quat(0);
        set_open_control_options(false);
        set_free_control_with_arm(true);
#ifdef BSP_USING_UI_HANDLER
        lvgl_msg_handler.handle_tap_event = handle_tap_event;
        lvgl_msg_handler.handle_widgets_control = NULL;
        lvgl_msg_handler.handle_nav_bar_control = timer_list_nav_control;
        lvgl_msg_handler.handle_tap_indicator = press_cb;
#endif
    }
}

static void on_stop(void)
{
    cleanup_timer_data_bindings();

    // 關閉體感控制選項
    set_open_control_options(false);
    set_free_control_with_arm(false);

    // Reset UI handler
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_tap_event = NULL;
    lvgl_msg_handler.handle_widgets_control = NULL;
    lvgl_msg_handler.handle_nav_bar_control = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
#endif

    // 清理UI
    if (lv_obj_is_valid(ui.bg))
    {
        lv_obj_del(ui.bg);
    }

    // Reset app context
    memset(&ui, 0, sizeof(timer_ui_t));
    selected_timer_index = 0;
    _timeout = false;
    setting_provider.set_power_save_mode(1);
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_TIMER, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(timer), IMG_ALARM_2, APP_ID_TIMER, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/