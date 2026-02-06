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
#include "bloc_setting.h"
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

// Exercise app style constants
#define LIST_TIMER_HEIGHT (200)
#define LIST_TIMER_WIDGET_RADIUS (40)
#define LIST_TIMER_BG_COLOR (0x1E1E1E)
#define LIST_TIMER_ACCENT_COLOR (0x80A0FF)
#define LIST_TIMER_ROW_SPACING (50)

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
static void timer_list_nav_control(int8_t action);
static void refresh_pause_button_icon(void);

// 追蹤當前選中的timer選項索引
static int16_t old_selected_timer_index = -1;
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

// 列表滾動處理 - 與exercise app相同的邏輯
static void scroll_timer_list(lv_obj_t *list)
{
    uint16_t min_offset = LV_VER_RES;
    uint8_t child_cnt = list->spec_attr->child_cnt;
    lv_coord_t y_diff = 0;

    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = list->spec_attr->children[i];
        lv_coord_t y_center = child->coords.y1 + LIST_TIMER_HEIGHT / 2;
        y_diff = y_center - LV_VER_RES / 2;
        y_diff = LV_ABS(y_diff);
        if (y_diff < min_offset)
        {
            min_offset = y_diff;
            selected_timer_index = i;
        }
    }

    // 使用邊框指示選中狀態 - 與exercise app相同
    if (old_selected_timer_index != selected_timer_index)
    {
        old_selected_timer_index = selected_timer_index;
        for (uint8_t i = 0; i < child_cnt; i++)
        {
            lv_obj_t *item = list->spec_attr->children[i];
            if (i == selected_timer_index)
            {
                lv_obj_set_style_border_width(item, 1, 0);
                ui.current_button = item;
            }
            else
            {
                lv_obj_set_style_border_width(item, 0, 0);
            }
        }
    }
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

// 創建timer列表 - 與exercise app相同的樣式
static lv_obj_t *create_timer_list(lv_obj_t *parent)
{
    ui.current_button = NULL;

    // 計算選項數量
    int option_count = 0;
    while (timer_options[option_count][0] != '\0')
        option_count++;

    // 創建列表外層容器
    lv_obj_t *list_container = lv_obj_create(parent);
    lv_obj_set_size(list_container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(list_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_container, 0, 0);
    lv_obj_set_style_pad_all(list_container, 0, 0);
    lv_obj_align(list_container, LV_ALIGN_TOP_MID, 0, 0);

    // 創建垂直列表 - 與exercise app相同
    lv_obj_t *list = lv_obj_create(list_container);
    lv_obj_set_size(list, LV_PCT(90), 466);
    lv_obj_align(list, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(list, LV_OPA_0, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_snap_y(list, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_style_pad_row(list, LIST_TIMER_ROW_SPACING, 0);
    lv_obj_set_style_pad_ver(list, LV_VER_RES / 2, 0);
    lv_obj_add_event_cb(list, timer_list_scroll_event_cb, LV_EVENT_ALL, NULL);
    ui.timer_list = list;

    // 添加標題背景 - 與exercise app相同
    lv_obj_t *title_bg = lv_obj_create(list_container);
    lv_obj_set_size(title_bg, 466, 80);
    lv_obj_set_style_bg_color(title_bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(title_bg, LV_OPA_80, 0);
    lv_obj_set_style_border_width(title_bg, 0, 0);
    lv_obj_align(title_bg, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(title_bg, LV_OBJ_FLAG_SCROLLABLE);

    ui.timer_title = lv_label_create(title_bg);
    lv_obj_set_size(ui.timer_title, 466, 40);
    lv_label_set_text(ui.timer_title, "Timer");
    lv_obj_set_style_text_align(ui.timer_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(ui.timer_title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(ui.timer_title, lv_color_hex(LIST_TIMER_ACCENT_COLOR), 0);
    lv_obj_align(ui.timer_title, LV_ALIGN_CENTER, 0, 0);

    // 添加計時器選項到列表 - 與exercise app workout widget相同的樣式
    for (int i = 0; i < option_count; i++)
    {
        // 創建timer選項widget
        lv_obj_t *timer_widget = lv_obj_create(list);
        lv_obj_set_size(timer_widget, LV_PCT(100), LIST_TIMER_HEIGHT);
        lv_obj_set_style_radius(timer_widget, LIST_TIMER_WIDGET_RADIUS, 0);
        lv_obj_set_style_bg_color(timer_widget, lv_color_hex(LIST_TIMER_BG_COLOR), 0);
        lv_obj_set_style_bg_opa(timer_widget, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(timer_widget, 0, 0);
        lv_obj_set_style_border_color(timer_widget, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_border_opa(timer_widget, LV_OPA_50, 0);
        lv_obj_clear_flag(timer_widget, LV_OBJ_FLAG_SCROLLABLE);

        // 添加鬧鐘圖標
        lv_obj_t *icon = lv_img_create(timer_widget);
        lv_img_set_src(icon, IMG_ALARM_2);
        lv_obj_align(icon, LV_ALIGN_LEFT_MID, 20, -30);

        // 添加時間標籤
        lv_obj_t *label = lv_label_create(timer_widget);
        lv_label_set_text(label, timer_options[i]);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
        lv_obj_align_to(label, icon, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        // 添加開始按鈕 - 與exercise app相同
        lv_obj_t *start_btn = lv_obj_create(timer_widget);
        lv_obj_set_size(start_btn, 85, 85);
        lv_obj_align(start_btn, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(start_btn, 45, 0);
        lv_obj_set_style_bg_color(start_btn, lv_color_hex(LIST_TIMER_ACCENT_COLOR), 0);
        lv_obj_set_style_border_width(start_btn, 0, 0);
        lv_obj_clear_flag(start_btn, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *start_icon = lv_img_create(start_btn);
        lv_img_set_src(start_icon, &img_media_play);
        lv_img_set_zoom(start_icon, 200);
        lv_obj_align(start_icon, LV_ALIGN_CENTER, 5, 0);

        // 添加點擊事件
        lv_obj_add_event_cb(timer_widget, timer_list_event_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_set_user_data(timer_widget, (void *)timer_options[i]);
    }

    // 滾動到預設位置
    selected_timer_index = 3;
    old_selected_timer_index = -1;
    if (option_count > selected_timer_index)
    {
        lv_obj_t *default_item = lv_obj_get_child(list, selected_timer_index);
        lv_obj_scroll_to_view(default_item, LV_ANIM_OFF);
        ui.current_button = default_item;
        // 設置初始選中狀態
        lv_obj_set_style_border_width(default_item, 1, 0);
        old_selected_timer_index = selected_timer_index;
    }

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
    set_timeout(false);
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

    // 計算選項數量用於設置segment count
    int option_count = 0;
    while (timer_options[option_count][0] != '\0')
        option_count++;

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
        set_scroll_segment_count(option_count);
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
    old_selected_timer_index = -1;
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