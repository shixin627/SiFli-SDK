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
#include "app_mainmenu.h"
#include "ui_img_helper.h"
#include "bloc_control.h"
#include "bloc_motion_tracking.h"
#include "bloc_motor.h"
#include "lv_simplified_obj.h"
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
#include "watch_global_data.h"
#include "watch_system_core_task.h"
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

/**
 * @brief Structure to hold all UI components of the timer app
 *
 * This helps organize the UI elements and makes it easier to
 * initialize, access, and clean them up
 */
typedef struct
{
    lv_obj_t *timer_list; // List of timer duration options
    lv_obj_t *timer_title;
    lv_obj_t *timer_label; // Label showing countdown time
    lv_obj_t *timer_view;
    lv_obj_t *pause_button;       // Button to pause/resume timer
    lv_obj_t *delete_button;      // Button to delete current timer
    lv_obj_t *current_button;     // Currently selected button in list
    lv_obj_t *center_btn_in_list; // Current center button in list

    // 水平三按鈕控制
    lv_obj_t *btn_select_background;
    lv_obj_t *btn_restart_bg;
    lv_obj_t *btn_pause_bg;
    lv_obj_t *btn_delete_bg;

    // timer列表選擇框
    lv_obj_t *list_select_background;
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

/* Forward declarations for functions */
static void update_timer_label(void);
static void show_counter_listview(void);
static lv_obj_t *create_timer_list(lv_obj_t *parent);
static void refresh_ui(lv_obj_t *_, void *para);
static void show_timeout_notification(void);
static void create_control_buttons(lv_obj_t *parent);
static void hide_control_buttons(void);
static void show_control_buttons(void);
static void timer_gesture_control(gesture_position_t gesture_position);
static void timer_list_nav_control(int8_t action);
static void update_list_selection_background(lv_obj_t *list);

// Control button selection state
static uint8_t button_selection_index = 1; // 0=restart, 1=pause, 2=delete
static uint8_t app_prev_selection_index = 1;
static lv_coord_t app_prev_offset = 0;
static lv_coord_t app_prev_move_offset = 0;

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
static void set_timeout(bool status)
{
    _timeout = status;
}

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
            set_timeout(true);
            watch_system_interact(INTERACT_TIMER_REMINDER, NULL);
            LOG_D("Timer finished");
        }
        LOG_D("Remaining time: %d", app_timer_data_ctx.remaining_time);
    }
}

/**
 * @brief Select a button in the timer list
 */
static void select_button(lv_obj_t *button)
{
    // if (ui.current_button)
    // {
    //     lv_obj_set_style_shadow_opa(ui.current_button, LV_OPA_0, 0);
    // }
    ui.current_button = button;
    // lv_obj_set_style_shadow_opa(ui.current_button, LV_OPA_100, 0);
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
    // 隱藏timer列表
    if (lv_obj_is_valid(ui.timer_list))
    {
        lv_obj_add_flag(ui.timer_list, LV_OBJ_FLAG_HIDDEN);
    }

    // 隱藏列表選擇框
    if (lv_obj_is_valid(ui.list_select_background))
    {
        lv_obj_add_flag(ui.list_select_background, LV_OBJ_FLAG_HIDDEN);
    }

    // 顯示計時器顯示標籤
    if (lv_obj_is_valid(ui.timer_label))
    {
        lv_obj_clear_flag(ui.timer_label, LV_OBJ_FLAG_HIDDEN);
    }

    // 顯示控制按鈕
    show_control_buttons();

    // 更新標題
    if (lv_obj_is_valid(ui.timer_title))
    {
        lv_label_set_text(ui.timer_title, text);
    }

    // 切換到三按鈕體感控制模式
    set_open_control_options(true);
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_widgets_control = timer_gesture_control;
    lvgl_msg_handler.handle_nav_bar_control = NULL;
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
        // remove_countdown_timer();
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

    const char *text = (const char *)lv_obj_get_user_data(btn);
    if (strcmp(text, "30 secs") == 0)
        app_timer_data_ctx.remaining_time = 30;
    else if (strcmp(text, "1 min") == 0)
        app_timer_data_ctx.remaining_time = 60;
    else if (strcmp(text, "2 mins") == 0)
        app_timer_data_ctx.remaining_time = 120;
    else if (strcmp(text, "3 mins") == 0)
        app_timer_data_ctx.remaining_time = 180;
    else if (strcmp(text, "4 mins") == 0)
        app_timer_data_ctx.remaining_time = 240;
    else if (strcmp(text, "5 mins") == 0)
        app_timer_data_ctx.remaining_time = 300;
    else if (strcmp(text, "10 mins") == 0)
        app_timer_data_ctx.remaining_time = 600;
    else if (strcmp(text, "15 mins") == 0)
        app_timer_data_ctx.remaining_time = 900;
    else if (strcmp(text, "20 mins") == 0)
        app_timer_data_ctx.remaining_time = 1200;
    else if (strcmp(text, "25 mins") == 0)
        app_timer_data_ctx.remaining_time = 1500;
    else if (strcmp(text, "30 mins") == 0)
        app_timer_data_ctx.remaining_time = 1800;
    else if (strcmp(text, "1 hour") == 0)
        app_timer_data_ctx.remaining_time = 3600;
    else
        app_timer_data_ctx.remaining_time = 0;

    create_timer_data_bindings();
    update_timer_label();
    show_new_timer_view(text);
    create_countdown_timer();
}

/* ...continue with rest of functions, updated to use ui... */

#define USE_ARC_LIST 0
static void check_center_button(lv_event_t *e)
{
    lv_obj_t *list = lv_event_get_target(e);
    lv_area_t list_area;
    lv_obj_get_coords(list, &list_area);
    lv_coord_t list_center_y = (list_area.y1 + list_area.y2) / 2;

    lv_obj_t *btn;
    lv_obj_t *center_btn = NULL;
    lv_coord_t min_diff = LV_COORD_MAX;

    uint8_t child_cnt = list->spec_attr->child_cnt;
    for (uint8_t i = 0; i < child_cnt; i++)
    {
        btn = list->spec_attr->children[i];
        lv_area_t btn_area;
        lv_obj_get_coords(btn, &btn_area);
        lv_coord_t btn_center_y = (btn_area.y1 + btn_area.y2) / 2;
        lv_coord_t diff = LV_ABS(btn_center_y - list_center_y);

#if USE_ARC_LIST
        // 計算相對位置（-1到1的範圍）
        float relative_pos = (float)diff / (list_area.y2 - list_area.y1);
        if (relative_pos > 1.0f)
            relative_pos = 1.0f;

        // 計算縮放和偏移
        float scale = 1.0f - (0.2f * relative_pos); // 中心最大，邊緣縮小20%
        float x_offset = sinf(relative_pos * (20.0f * 3.14159f / 180.0f)) * 30;

        lv_obj_set_style_translate_x(btn, (int16_t)x_offset, 0);
#endif

        if (diff < min_diff)
        {
            min_diff = diff;
            center_btn = btn;
        }
    }

    if (center_btn && center_btn != ui.center_btn_in_list)
    {
        ui.center_btn_in_list = center_btn;
        select_button(center_btn);
    }
}

static void scroll_end_event_cb(lv_event_t *e)
{
    if (ui.current_button)
    {
        lv_obj_scroll_to_view(ui.current_button, LV_ANIM_ON);
    }
}

static void show_counter_listview(void)
{
    // 隱藏計時器顯示標籤
    if (lv_obj_is_valid(ui.timer_label))
    {
        lv_obj_add_flag(ui.timer_label, LV_OBJ_FLAG_HIDDEN);
    }

    // 隱藏控制按鈕
    hide_control_buttons();

    // 顯示或創建timer列表
    if (lv_obj_is_valid(ui.timer_list))
    {
        lv_obj_clear_flag(ui.timer_list, LV_OBJ_FLAG_HIDDEN);
    }
    else if (lv_obj_is_valid(ui.timer_view))
    {
        // 在timer_view容器中創建列表
        ui.timer_list = create_timer_list(ui.timer_view);
    }

    // 顯示列表選擇框
    if (lv_obj_is_valid(ui.list_select_background))
    {
        lv_obj_clear_flag(ui.list_select_background, LV_OBJ_FLAG_HIDDEN);
        // 更新選擇框位置
        if (lv_obj_is_valid(ui.timer_list))
        {
            update_list_selection_background(ui.timer_list);
        }
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

static void restart_button_event_cb(lv_event_t *e)
{
    if (app_timer_data_ctx.countdown_timer)
    {
        // 重啟計時器：重置剩餘時間，重新開始倒數
        app_timer_data_ctx.is_paused = false;
        refresh_timer_status();

        // 如果中間按鈕顯示暫停圖標，改為播放圖標
        if (lv_obj_is_valid(ui.btn_pause_bg))
        {
            lv_obj_t *img = lv_obj_get_child(ui.btn_pause_bg, 0);
            if (lv_obj_is_valid(img))
            {
                lv_img_set_src(img, &img_media_pause);
            }
        }
    }
}

static void pause_button_event_cb_old(lv_event_t *e)
{
    app_timer_data_ctx.is_paused = !app_timer_data_ctx.is_paused;
    refresh_timer_status();

    if (lv_obj_is_valid(ui.pause_button))
    {
        lv_obj_t *img = lv_obj_get_child(ui.pause_button, 0);
        if (lv_obj_is_valid(img))
        {
            lv_img_set_src(img, app_timer_data_ctx.is_paused ? &img_media_play : &img_media_pause);
        }
    }
}

// 新的暫停按鈕事件處理 - 使用新的按鈕結構
static void pause_button_event_cb(lv_event_t *e)
{
    app_timer_data_ctx.is_paused = !app_timer_data_ctx.is_paused;
    refresh_timer_status();

    if (lv_obj_is_valid(ui.btn_pause_bg))
    {
        lv_obj_t *img = lv_obj_get_child(ui.btn_pause_bg, 0);
        if (lv_obj_is_valid(img))
        {
            lv_img_set_src(img, app_timer_data_ctx.is_paused ? &img_media_play : &img_media_pause);
        }
    }
}

static void delete_countdown_timer_cb(lv_event_t *e)
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

static uint8_t old_page_index = 0;
static void tile_change_event_cb(lv_event_t *e)
{
    lv_obj_t *tv = lv_event_get_target(e);
    switch (e->code)
    {
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint16_t active_pos = (rt_uint16_t)lv_event_get_param(e);
        if (old_page_index != active_pos)
        {
            old_page_index = active_pos;
            LOG_D("tileview_active_pos: %d", active_pos);
            lv_obj_t *indicator = lv_obj_get_child(tv->parent, -1); // 假設指示器是最後一個子元素
            if (lv_obj_is_valid(indicator))
            {
                uint32_t dot_cnt = lv_obj_get_child_cnt(indicator);
                uint32_t i;
                for (i = 0; i < dot_cnt; i++)
                {
                    lv_obj_t *dot = lv_obj_get_child(indicator, i);
                    if (i == active_pos)
                    {
                        // 當前頁面的指示點 - 高亮
                        lv_obj_set_style_bg_opa(dot, LV_OPA_100, 0);
                        // 設置為稍大尺寸
                        lv_obj_set_size(dot, 12, 12);
                    }
                    else
                    {
                        // 非當前頁面的指示點
                        lv_obj_set_style_bg_opa(dot, LV_OPA_30, 0);
                        // 恢復原始尺寸
                        lv_obj_set_size(dot, 10, 10);
                    }
                }
            }
        }
    }
    break;

    default:
        break;
    }
}

// 追蹤當前選中的timer選項索引
static int8_t selected_timer_index = 0;

// 更新列表選擇框位置
static void update_list_selection_background(lv_obj_t *list)
{
    if (!lv_obj_is_valid(ui.list_select_background) || !lv_obj_is_valid(list))
        return;

    // 計算當前居中的項目
    uint8_t child_cnt = list->spec_attr->child_cnt;
    if (child_cnt == 0)
        return;

    lv_area_t list_area;
    lv_obj_get_coords(list, &list_area);
    lv_coord_t list_center_y = (list_area.y1 + list_area.y2) / 2;

    lv_coord_t min_diff = LV_COORD_MAX;
    lv_obj_t *center_item = NULL;

    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *item = lv_obj_get_child(list, i);
        lv_area_t item_area;
        lv_obj_get_coords(item, &item_area);
        lv_coord_t item_center_y = (item_area.y1 + item_area.y2) / 2;
        lv_coord_t diff = LV_ABS(item_center_y - list_center_y);

        if (diff < min_diff)
        {
            min_diff = diff;
            center_item = item;
        }
    }

    if (center_item)
    {
        // 獲取center_item的絕對位置
        lv_area_t item_area;
        lv_obj_get_coords(center_item, &item_area);
        lv_coord_t item_center_y = (item_area.y1 + item_area.y2) / 2;

        // 計算選擇框應該在屏幕上的Y位置
        lv_coord_t screen_center_y = LV_VER_RES / 2;
        lv_coord_t offset_y = item_center_y - screen_center_y;

        // 更新選擇框位置，讓它對齊到當前居中的項目
        lv_obj_align(ui.list_select_background, LV_ALIGN_CENTER, 0, offset_y);
    }
}

// timer列表滾動事件處理 - 類似app list
static void timer_list_scroll_event_cb(lv_event_t *e)
{
    lv_obj_t *list = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);

    switch (code)
    {
    case LV_EVENT_SCROLL:
    case LV_EVENT_SCROLL_END:
    {
        // 計算當前居中的項目
        uint8_t child_cnt = list->spec_attr->child_cnt;
        lv_area_t list_area;
        lv_obj_get_coords(list, &list_area);
        lv_coord_t list_center_y = (list_area.y1 + list_area.y2) / 2;

        lv_coord_t min_diff = LV_COORD_MAX;
        int8_t center_index = 0;

        for (uint8_t i = 0; i < child_cnt; i++)
        {
            lv_obj_t *item = lv_obj_get_child(list, i);
            lv_area_t item_area;
            lv_obj_get_coords(item, &item_area);
            lv_coord_t item_center_y = (item_area.y1 + item_area.y2) / 2;
            lv_coord_t diff = LV_ABS(item_center_y - list_center_y);

            if (diff < min_diff)
            {
                min_diff = diff;
                center_index = i;
            }
        }

        selected_timer_index = center_index;
        ui.current_button = lv_obj_get_child(list, center_index);

        // 更新選擇框位置
        update_list_selection_background(list);

        if (code == LV_EVENT_SCROLL_END)
        {
            LOG_D("Timer list scrolled to index: %d", selected_timer_index);
        }
        break;
    }
    default:
        break;
    }
}

// 滾動到指定的timer選項 - 用於體感控制
static void scroll_timer_list_to_index(int8_t index)
{
    // 計算選項數量
    int option_count = 0;
    while (timer_options[option_count][0] != '\0')
        option_count++;

    if (index < 0 || index >= option_count)
    {
        LOG_W("Timer index out of bounds: %d", index);
        return;
    }

    if (!lv_obj_is_valid(ui.timer_list))
    {
        LOG_E("Timer list is not valid");
        return;
    }

    selected_timer_index = index;
    lv_obj_t *target_item = lv_obj_get_child(ui.timer_list, index);
    if (target_item && lv_obj_is_valid(target_item))
    {
        lv_obj_scroll_to_view(target_item, LV_ANIM_ON);
        ui.current_button = target_item;
    }
}

// 體感控制用於滾動timer列表
static void timer_list_nav_control(int8_t action)
{
    if (action >= 0)
    {
        scroll_timer_list_to_index(action);
    }
    else
    {
        // 相對滾動
        int8_t new_index = selected_timer_index + action;

        // 計算選項數量
        int option_count = 0;
        while (timer_options[option_count][0] != '\0')
            option_count++;

        if (new_index < 0)
            new_index = 0;
        if (new_index >= option_count)
            new_index = option_count - 1;

        scroll_timer_list_to_index(new_index);
    }
}

// App list style constants
#define LIST_ITEM_WIDGET_WIDTH (430)
#define LIST_ITEM_WIDGET_HEIGHT (250)
#define LIST_ITEM_SPACING (-100)

static lv_obj_t *create_timer_list(lv_obj_t *parent)
{
    ui.current_button = NULL;
    ui.center_btn_in_list = NULL;

    // 計算選項數量
    int option_count = 0;
    while (timer_options[option_count][0] != '\0')
        option_count++;

    // 創建選擇框背景 (略大於label，無背景色，僅邊框)
    ui.list_select_background = lv_obj_create(parent);
    lv_obj_set_size(ui.list_select_background, 180, 60);  // 更小巧的尺寸，略大於label
    lv_obj_set_style_bg_opa(ui.list_select_background, LV_OPA_0, 0);  // 無背景色
    lv_obj_set_style_border_width(ui.list_select_background, 2, 0);
    lv_obj_set_style_border_color(ui.list_select_background, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(ui.list_select_background, LV_OPA_60, 0);  // 邊框稍微透明
    lv_obj_set_style_radius(ui.list_select_background, 10, 0);  // 較小的圓角
    lv_obj_align(ui.list_select_background, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(ui.list_select_background, LV_OBJ_FLAG_SCROLLABLE);

    // 創建垂直滾動列表容器 - 完全使用app list的樣式
    lv_obj_t *list_container = lv_obj_create(parent);
    lv_obj_set_size(list_container, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(list_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(list_container, 0, 0);
    lv_obj_add_flag(list_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(list_container, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_snap_y(list_container, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_pad_ver(list_container, LV_VER_RES / 2, 0); // 上下padding讓第一項和最後一項能居中
    lv_obj_align(list_container, LV_ALIGN_CENTER, 0, 0);

    // 添加滾動事件處理
    lv_obj_add_event_cb(list_container, timer_list_scroll_event_cb, LV_EVENT_ALL, NULL);

    // 創建垂直列表中的計時器選項 - 完全使用app list的樣式
    for (int i = 0; i < option_count; i++)
    {
        // 使用 lv_simplified_obj_create 創建項目 (與app list相同)
        lv_obj_t *item = lv_simplified_obj_create(list_container);
        lv_obj_set_size(item, LIST_ITEM_WIDGET_WIDTH, LIST_ITEM_WIDGET_HEIGHT);

        // 使用與app list相同的位置計算方式
        if (i == 0)
        {
            lv_obj_set_pos(item, 0, (100 + LIST_ITEM_SPACING));
        }
        else
        {
            lv_obj_set_pos(item, 0, (LIST_ITEM_WIDGET_HEIGHT + LIST_ITEM_SPACING) * i + (100 + LIST_ITEM_SPACING));
        }
        lv_obj_add_flag(item, LV_OBJ_FLAG_CLICKABLE);

        // 添加點擊事件
        lv_obj_add_event_cb(item, timer_list_event_cb, LV_EVENT_CLICKED, NULL);

        // 保存timer選項文本，用於後續識別
        lv_obj_set_user_data(item, (void *)timer_options[i]);

        // 創建timer時長標籤 (替換app list中的app name)
        lv_obj_t *label = lv_label_create(item);
        lv_label_set_text(label, timer_options[i]);
        lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }

    // 滾動到中間位置(例如3 mins)
    selected_timer_index = 3;
    if (option_count > selected_timer_index)
    {
        lv_obj_scroll_to_view(lv_obj_get_child(list_container, selected_timer_index), LV_ANIM_OFF);
        ui.current_button = lv_obj_get_child(list_container, selected_timer_index);
    }

    return list_container;
}

// 創建水平三按鈕控制 - 重啟/暫停/刪除
static void create_control_buttons(lv_obj_t *parent)
{
    // 為圓形表盤優化按鈕位置
    // 三個按鈕均勻分布在圓形表盤的中央區域

    // 創建選擇背景 - 位於中央
    ui.btn_select_background = lv_obj_create(parent);
    lv_obj_set_size(ui.btn_select_background, 120, 90);
    lv_obj_set_style_bg_color(ui.btn_select_background, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(ui.btn_select_background, LV_OPA_20, 0);
    lv_obj_set_style_border_width(ui.btn_select_background, 2, 0);
    lv_obj_set_style_border_color(ui.btn_select_background, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(ui.btn_select_background, LV_OPA_50, 0);
    lv_obj_set_style_radius(ui.btn_select_background, 15, 0);
    lv_obj_align(ui.btn_select_background, LV_ALIGN_CENTER, 0, 40);

    // 左側按鈕 - 重啟計時器
    ui.btn_restart_bg = lv_obj_create(parent);
    lv_obj_set_size(ui.btn_restart_bg, 110, 85);
    lv_obj_set_style_bg_opa(ui.btn_restart_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(ui.btn_restart_bg, 0, 0);
    // 圓形表盤優化位置：左側位置
    lv_obj_align(ui.btn_restart_bg, LV_ALIGN_LEFT_MID, 15, 40);
    lv_obj_add_flag(ui.btn_restart_bg, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *img_restart = lv_img_create(ui.btn_restart_bg);
    lv_img_set_src(img_restart, &img_media_previous);
    lv_obj_center(img_restart);

    // 添加左側按鈕的點擊事件
    lv_obj_add_event_cb(ui.btn_restart_bg, restart_button_event_cb, LV_EVENT_CLICKED, NULL);

    // 中間按鈕 - 暫停/繼續
    ui.btn_pause_bg = lv_obj_create(parent);
    lv_obj_set_size(ui.btn_pause_bg, 110, 85);
    lv_obj_set_style_bg_opa(ui.btn_pause_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(ui.btn_pause_bg, 0, 0);
    // 圓形表盤優化位置：中央
    lv_obj_align(ui.btn_pause_bg, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(ui.btn_pause_bg, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *img_pause = lv_img_create(ui.btn_pause_bg);
    lv_img_set_src(img_pause, &img_media_pause);
    lv_obj_center(img_pause);

    // 添加中間按鈕的點擊事件
    lv_obj_add_event_cb(ui.btn_pause_bg, pause_button_event_cb, LV_EVENT_CLICKED, NULL);

    // 右側按鈕 - 刪除/取消
    ui.btn_delete_bg = lv_obj_create(parent);
    lv_obj_set_size(ui.btn_delete_bg, 110, 85);
    lv_obj_set_style_bg_opa(ui.btn_delete_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(ui.btn_delete_bg, 0, 0);
    // 圓形表盤優化位置：右側位置
    lv_obj_align(ui.btn_delete_bg, LV_ALIGN_RIGHT_MID, -15, 40);
    lv_obj_add_flag(ui.btn_delete_bg, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *img_delete_icon = lv_img_create(ui.btn_delete_bg);
    lv_img_set_src(img_delete_icon, ICON_TRASH);
    lv_obj_center(img_delete_icon);

    // 添加右側按鈕的點擊事件
    lv_obj_add_event_cb(ui.btn_delete_bg, delete_countdown_timer_cb, LV_EVENT_CLICKED, NULL);
}

// 隱藏控制按鈕
static void hide_control_buttons(void)
{
    if (ui.btn_select_background && lv_obj_is_valid(ui.btn_select_background))
        lv_obj_add_flag(ui.btn_select_background, LV_OBJ_FLAG_HIDDEN);
    if (ui.btn_restart_bg && lv_obj_is_valid(ui.btn_restart_bg))
        lv_obj_add_flag(ui.btn_restart_bg, LV_OBJ_FLAG_HIDDEN);
    if (ui.btn_pause_bg && lv_obj_is_valid(ui.btn_pause_bg))
        lv_obj_add_flag(ui.btn_pause_bg, LV_OBJ_FLAG_HIDDEN);
    if (ui.btn_delete_bg && lv_obj_is_valid(ui.btn_delete_bg))
        lv_obj_add_flag(ui.btn_delete_bg, LV_OBJ_FLAG_HIDDEN);
}

// 顯示控制按鈕
static void show_control_buttons(void)
{
    if (ui.btn_select_background && lv_obj_is_valid(ui.btn_select_background))
        lv_obj_clear_flag(ui.btn_select_background, LV_OBJ_FLAG_HIDDEN);
    if (ui.btn_restart_bg && lv_obj_is_valid(ui.btn_restart_bg))
        lv_obj_clear_flag(ui.btn_restart_bg, LV_OBJ_FLAG_HIDDEN);
    if (ui.btn_pause_bg && lv_obj_is_valid(ui.btn_pause_bg))
        lv_obj_clear_flag(ui.btn_pause_bg, LV_OBJ_FLAG_HIDDEN);
    if (ui.btn_delete_bg && lv_obj_is_valid(ui.btn_delete_bg))
        lv_obj_clear_flag(ui.btn_delete_bg, LV_OBJ_FLAG_HIDDEN);
}

// 設置選擇背景位置（與iot_gate相同的邏輯）
static void set_app_selection_bg_pos(uint8_t selection_index, lv_coord_t offset)
{
    if (ui.btn_select_background == NULL || (app_prev_selection_index == selection_index && app_prev_offset == offset))
        return;

    int move_offset = 0;
    switch (selection_index)
    {
    case 0: // 重啟按鈕 (左側)
        move_offset = -150 + offset;
        break;
    case 1: // 暫停按鈕 (中間)
        move_offset = offset;
        break;
    case 2: // 刪除按鈕 (右側)
        move_offset = 150 + offset;
        break;
    default:
        break;
    }

    lv_obj_align(ui.btn_select_background, LV_ALIGN_CENTER, move_offset, 40);
    app_prev_move_offset = move_offset;
    app_prev_selection_index = selection_index;
    app_prev_offset = offset;
}

// 體感控制邏輯 - 與iot_gate相同的實現方式
// Y軸位置範圍 (0~466):
//     0~155: 選擇右側按鈕 (刪除/category 2)
//     155~311: 選擇中間按鈕 (暫停/category 1)
//     311~466: 選擇左側按鈕 (重啟/category 0)
static void timer_gesture_control(gesture_position_t gesture_position)
{
    // 只在計時器運行時才處理體感控制
    if (!app_timer_data_ctx.countdown_timer || !lv_obj_is_valid(ui.btn_select_background))
    {
        return;
    }

    int p_y = gesture_position.gesture_position_y;
    lv_coord_t diff = 0;
    uint8_t category;

    // 根據Y軸位置判斷選擇哪個按鈕（與iot_gate相同的邏輯）
    if (p_y >= 0 && p_y < 155)
    {
        // 將 0~155 映射到 0~-12 (向右拖拽，選擇刪除按鈕 - 右側)
        diff = -(12 * p_y) / 155;
        category = 2; // 刪除按鈕 (右側)
    }
    else if (p_y > 311 && p_y <= 466)
    {
        // 將 311~466 映射到 12~0 (向左拖拽，選擇重啟按鈕 - 左側)
        diff = 12 - (12 * (p_y - 311)) / (466 - 311);
        category = 0; // 重啟按鈕 (左側)
    }
    else if (p_y >= 155 && p_y <= 311)
    {
        // 將 155~311 映射到 12~-12 (中間區域，選擇暫停按鈕)
        diff = 12 - (24 * (p_y - 155)) / (311 - 155);
        category = 1; // 暫停按鈕 (中間)
    }
    else
    {
        category = 1; // 默認選擇暫停按鈕
    }

    button_selection_index = category;
    set_app_selection_bg_pos(category, diff);
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
    set_timeout(false);
    remove_countdown_timer();
    show_counter_listview();
}

static void show_timeout_notification(void)
{
    if (timeout_msg_box && lv_obj_is_valid(timeout_msg_box))
        return;

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

static lv_obj_t *create_timer_screen(lv_obj_t *parent)
{
    // 創建主背景容器
    lv_obj_t *bg_container = lv_obj_create(parent);
    lv_obj_set_size(bg_container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(bg_container, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(bg_container, LV_OPA_100, 0);
    lv_obj_set_style_border_width(bg_container, 0, 0);
    lv_obj_set_style_pad_all(bg_container, 0, 0);
    lv_obj_set_style_radius(bg_container, LV_RADIUS_CIRCLE, 0);
    lv_obj_align(bg_container, LV_ALIGN_CENTER, 0, 0);

    // 添加標題（頂部）
    ui.timer_title = lv_label_create(bg_container);
    lv_label_set_text(ui.timer_title, "Timer");
    lv_obj_set_style_text_font(ui.timer_title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(ui.timer_title, lv_color_hex(0x80A0FF), 0);
    lv_obj_align(ui.timer_title, LV_ALIGN_TOP_MID, 0, 15);

    // 創建計時器顯示標籤（頂部區域，初始隱藏）
    ui.timer_label = lv_label_create(bg_container);
    lv_obj_set_style_text_font(ui.timer_label, LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_obj_set_style_text_color(ui.timer_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(ui.timer_label, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_add_flag(ui.timer_label, LV_OBJ_FLAG_HIDDEN);

    // 創建水平三按鈕控制（底部中央，適合圓形表盤）
    create_control_buttons(bg_container);

    // 初始時隱藏控制按鈕
    hide_control_buttons();

    return bg_container;
}

static void on_start(void)
{
    // Initialize UI context
    memset(&ui, 0, sizeof(timer_ui_t));
    ui.timer_view = create_timer_screen(lv_scr_act());

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
            // 顯示計時器標籤和控制按鈕
            if (lv_obj_is_valid(ui.timer_label))
                lv_obj_clear_flag(ui.timer_label, LV_OBJ_FLAG_HIDDEN);
            show_control_buttons();
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
}

static void on_resume(void)
{
    // switch_watch_motion_control_mode(true, false);
    refresh_timer_status();

    // 根據當前狀態設置不同的handler
    if (app_timer_data_ctx.countdown_timer)
    {
        // 計時器運行中 - 啟用三按鈕體感控制
        set_open_control_options(true);
        set_free_control_with_arm(false);
#ifdef BSP_USING_UI_HANDLER
        lvgl_msg_handler.handle_tap_event = handle_tap_event;
        lvgl_msg_handler.handle_widgets_control = timer_gesture_control;
        lvgl_msg_handler.handle_nav_bar_control = NULL;
#endif
    }
    else
    {
        // timer列表模式 - 啟用列表滾動控制
        set_open_control_options(false);
        set_free_control_with_arm(true);
#ifdef BSP_USING_UI_HANDLER
        lvgl_msg_handler.handle_tap_event = handle_tap_event;
        lvgl_msg_handler.handle_widgets_control = NULL;
        lvgl_msg_handler.handle_nav_bar_control = timer_list_nav_control; // 用於體感滾動列表
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
#endif

    // Reset app context
    memset(&ui, 0, sizeof(timer_ui_t));
    _timeout = false;
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