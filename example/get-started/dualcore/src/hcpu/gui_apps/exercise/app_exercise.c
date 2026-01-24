/**
 ******************************************************************************
 * @file   app_exercise.c
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
#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include <time.h>
#include <cJSON.h>
#include "sensor.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "ui_datasrv_subscriber.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "bloc_motor.h"
#include "bloc_setting.h"
#include "bloc_exercise.h"
#include "bloc_filesystem.h"
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
#include "watch_global_data.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#endif

#define DBG_TAG "app.exercise"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_EXERCISE

// 运动UI控件集合
typedef struct
{
    lv_obj_t *bg;           // 背景
    lv_obj_t *tileview;     // 主视图容器
    lv_obj_t *list_tile;    // 运动列表视图
    lv_obj_t *history_tile; // 历史记录视图
    lv_obj_t *workout_list; // 运动列表
    lv_obj_t *history_list; // 历史记录列表
    lv_obj_t *workout_screen;
    lv_obj_t *timer_label; // 计时标签
    lv_obj_t *metrics_container;
    lv_obj_t *heart_rate_label; // 心率标签
    lv_obj_t *calories_label;   // 卡路里标签
    lv_obj_t *pause_button;     // 暂停按钮
    lv_obj_t *stop_button;      // 停止按钮
} workout_ui_t;

typedef struct
{
    lv_ex_data_t *session_data;
    bool active;
} app_exercise_data_ctx_t;

static app_exercise_data_ctx_t app_exercise_data_ctx;

// 声明图标资源
LV_IMG_DECLARE(img_workout);
LV_IMG_DECLARE(img_workout_running);
LV_IMG_DECLARE(img_workout_walking);
LV_IMG_DECLARE(img_workout_cycling);
LV_IMG_DECLARE(img_workout_swimming);
LV_IMG_DECLARE(img_workout_hiking);
LV_IMG_DECLARE(img_workout_yoga);
LV_IMG_DECLARE(img_workout_gym);
LV_IMG_DECLARE(img_media_play);
LV_IMG_DECLARE(img_media_pause);
LV_IMG_DECLARE(img_red_heart);
LV_IMG_DECLARE(img_calories);
LV_IMG_DECLARE(icon_history);

extern void refresh_heartrate_widget(uint8_t hr);

typedef struct
{
    const char *name;       // 运动名称
    const void *icon;       // 运动图标
    float calories_per_min; // 每分钟卡路里消耗基数
} workout_info_t;

const workout_info_t workout_list[WORKOUT_COUNT] = {
    [WORKOUT_RUNNING] = {WORKOUT_RUNNING_TITLE, &img_workout_running, 10.0f},
    [WORKOUT_WALKING] = {WORKOUT_WALKING_TITLE, &img_workout_walking, 5.0f},
    [WORKOUT_CYCLING] = {WORKOUT_CYCLING_TITLE, &img_workout_cycling, 8.0f},
    [WORKOUT_SWIMMING] = {WORKOUT_SWIMMING_TITLE, &img_workout_swimming, 11.0f},
    [WORKOUT_HIKING] = {WORKOUT_HIKING_TITLE, &img_workout_hiking, 7.0f},
    [WORKOUT_YOGA] = {WORKOUT_YOGA_TITLE, &img_workout_yoga, 4.0f},
    [WORKOUT_GYM] = {WORKOUT_GYM_TITLE, &img_workout_gym, 9.0f}};

const workout_info_t *get_workout_list(void)
{
    return workout_list;
}

// 全局UI和会话数据
static workout_ui_t ui;
static workout_session_t current_session;
static void show_workout_view();
static lv_obj_t *create_workout_list(lv_obj_t *parent);
static lv_obj_t *create_history_list(lv_obj_t *parent);
static void handle_tap_event(void);
static void refresh_session_pause_button(void);

static void update_workout_display(void)
{
    if (!lv_obj_is_valid(ui.timer_label))
        return;

    // Format time as hours:minutes:seconds
    uint32_t hours = current_session.duration / 3600;
    uint32_t minutes = (current_session.duration % 3600) / 60;
    uint32_t seconds = current_session.duration % 60;

    char time_str[16];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hours, minutes, seconds);
    lv_label_set_text(ui.timer_label, time_str);

    // Update calories display
    char cal_str[16];
    snprintf(cal_str, sizeof(cal_str), "%d KCAL", current_session.calories);
    lv_label_set_text(ui.calories_label, cal_str);

    LOG_D("Update workout display: time=%s, calories=%s", time_str, cal_str);
}

// 计时器回调函数
static void workout_timer_cb(void *parameter)
{
    if (current_session.is_paused)
        return;

    // 更新持续时间
    current_session.duration++;

    // 每分钟处理一次心率数据和卡路里计算
    if (current_session.duration % 60 == 0)
    {
        // 只有在有心率数据时才计算平均心率
        if (current_session.num_of_hr_in_min > 0)
        {
            uint16_t avr_hr_in_min = current_session.sum_of_hr_in_min / current_session.num_of_hr_in_min;

            // 存储这一分钟的平均心率
            if (current_session.hrm_count < MAX_WORKOUT_MINUTES)
            {
                current_session.hrm_array[current_session.hrm_count] = avr_hr_in_min;
                current_session.hrm_count++;
            }

            // 计算并累加卡路里消耗
            float calories_minute = calculate_calories(
                workout_list[current_session.type].calories_per_min,
                60,
                avr_hr_in_min);
            current_session.calories += calories_minute;

            // 重置心率累计值，准备下一分钟的计算
            current_session.sum_of_hr_in_min = 0;
            current_session.num_of_hr_in_min = 0;
        }
    }

    if (app_exercise_data_ctx.session_data)
    {
        // 更新UI显示
        lv_ex_data_set_value(app_exercise_data_ctx.session_data, (void *)&current_session);
    }
}

static void ui_heart_rate_callback(int hr)
{
    if (hr != current_session.heart_rate)
    {
        LOG_D("[UI]Heart rate changed: %d", hr);
        if (lv_obj_is_valid(ui.heart_rate_label))
        {
            // Update UI display
            char bpm_str[16];
            snprintf(bpm_str, sizeof(bpm_str), "%d BPM", current_session.heart_rate);
            lv_label_set_text(ui.heart_rate_label, bpm_str);
        }
        refresh_heartrate_widget(hr);
    }
}

void app_exercise_background_hr_cb(int hr)
{
    if (!app_exercise_data_ctx.active)
    {
        return;
    }

    if (hr >= 0)
    {
        // Accumulate heart rate for average calculation
        current_session.sum_of_hr_in_min += hr;
        current_session.num_of_hr_in_min++;

        // Update UI only when heart rate changes
        if (hr != current_session.heart_rate)
        {
            LOG_D("[BG] Heart rate changed: %d", hr);
            // Update session and global heart rate values
            current_session.heart_rate = hr;
            SkaiWatchSys.heart_rate_bpm = hr;
        }
    }
}

// 开始运动会话
static void start_workout_session(workout_type_t type)
{
    LOG_D("Starting workout session of type: %s", workout_list[type].name);

    // 初始化会话数据
    memset(&current_session, 0, sizeof(workout_session_t));
    current_session.type = type;
    current_session.heart_rate = 0;
    current_session.is_paused = false;

    // Create or restart the workout timer
    if (!current_session.workout_timer)
    {
        current_session.workout_timer = rt_timer_create(
            "workout_timer",
            workout_timer_cb,
            RT_NULL,
            RT_TICK_PER_SECOND,
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

        if (!current_session.workout_timer)
        {
            LOG_E("Failed to create workout timer");
            return;
        }
    }
    else
    {
        rt_timer_stop(current_session.workout_timer);
    }

    // Start the timer
    rt_timer_start(current_session.workout_timer);

    // Set up heart rate monitoring
    sensor_subscription_t sensor_subscription = (sensor_subscription_t){
        .type = SENSOR_TYPE_HEART_RATE,
        .status = true,
    };
    watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
    app_exercise_data_ctx.active = true;
    lvgl_msg_handler.handle_hr = ui_heart_rate_callback;

    // Switch to workout view
    show_workout_view();
}

// 切换到运动选择界面
static void show_workout_list_view()
{
    // 显示运动列表视图
    lv_obj_set_tile_id(ui.tileview, 0, 0, LV_ANIM_ON);
}

// 切换到历史记录界面
static void show_history_view()
{
    // 显示历史记录视图
    lv_obj_set_tile_id(ui.tileview, 0, 1, LV_ANIM_ON);
}

static void refresh_ui(lv_obj_t *_, workout_session_t *session)
{
    update_workout_display();
}

// 切换到运动计时界面
static void show_workout_view()
{
    // 隐藏运动选择列表和tileview
    lv_obj_add_flag(ui.tileview, LV_OBJ_FLAG_HIDDEN);

    lv_ex_binding_t binding;
    app_exercise_data_ctx.session_data = lv_ex_data_create("exercise.session", LV_EX_DATA_POINTER);
    binding.target = ui.bg;
    binding.arg_type = LV_EX_DATA_POINTER;
    binding.setter = (void *)refresh_ui;
    lv_ex_bind_data(app_exercise_data_ctx.session_data, &binding);

    // 显示运动计时界面元素
    lv_obj_clear_flag(ui.workout_screen, LV_OBJ_FLAG_HIDDEN);

    // 更新显示
    update_workout_display();

    refresh_session_pause_button();
}

int stop_exercise(void)
{
    LOG_D("Stopping exercise session");
    int ret = 0;

    // Stop and delete workout timer
    if (current_session.workout_timer)
    {
        rt_timer_stop(current_session.workout_timer);
        rt_timer_delete(current_session.workout_timer);
        current_session.workout_timer = NULL;

        // If we have a valid workout session, store it in the file system
        if (current_session.duration > 0)
        {
            ret = store_exercise_data(&current_session);
            if (ret == 0)
            {
                LOG_D("Exercise data stored successfully");
            }
            else
            {
                LOG_E("Failed to store exercise data: %d", ret);
                if (ret == EXERCISE_DATA_TOO_SHORT)
                {
                    ui_show_hint_toast("Exercise session too short");
                }
                else if (ret == EXERCISE_NO_HRM_DATA)
                {
                    ui_show_hint_toast("No heart rate data recorded");
                }
                else if (ret == EXERCISE_NO_CALORIES_DATA)
                {
                    ui_show_hint_toast("No calories data recorded");
                }
                else if (ret == EXERCISE_FILE_OPEN_FAILED)
                {
                    ui_show_hint_toast("Failed to open file for writing");
                }
                else
                {
                    ui_show_hint_toast("Failed to store exercise data: %d", ret);
                }
            }
        }
    }

    // Hide workout screen and show navigation view
    if (lv_obj_is_valid(ui.workout_screen))
    {
        lv_obj_add_flag(ui.workout_screen, LV_OBJ_FLAG_HIDDEN);
    }

    if (lv_obj_is_valid(ui.tileview))
    {
        lv_obj_clear_flag(ui.tileview, LV_OBJ_FLAG_HIDDEN);
        // Refresh the history list to show the newly added workout
        create_history_list(ui.history_tile);
    }

    // Clean up data bindings
    if (app_exercise_data_ctx.session_data)
    {
        lv_ex_data_t *data = app_exercise_data_ctx.session_data;
        app_exercise_data_ctx.session_data = NULL;
        lv_ex_data_delete(data);
    }

    // Disable heart rate monitoring mode
    app_exercise_data_ctx.active = false;
    sensor_subscription_t sensor_subscription = (sensor_subscription_t){
        .type = SENSOR_TYPE_HEART_RATE,
        .status = false,
    };
    watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);

    return ret;
}

static void refresh_session_pause_button(void)
{
    lv_obj_t *img = lv_obj_get_child(ui.pause_button, 0);
    lv_img_set_src(img, current_session.is_paused ? &img_media_play : &img_media_pause);
}

// 暂停/继续按钮回调
static void pause_button_event_cb(lv_event_t *e)
{
    current_session.is_paused = !current_session.is_paused;
    refresh_session_pause_button();
}

// 停止按钮回调
static void stop_button_event_cb(lv_event_t *e)
{
    if (stop_exercise() == 0)
    {
        show_history_view();
    }
    else
    {
        // 停止运动会话失败，显示错误信息
        ui_show_hint_toast("Failed to stop exercise session");
    }
}

static void nav_to_history_icon_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        show_history_view();
    }
}

// 选择运动项目
static void workout_list_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        lv_obj_t *btn = lv_event_get_target(e);
        const char *name = (const char *)lv_obj_get_user_data(btn);

        // 查找按钮对应的运动类型
        for (int i = 0; i < WORKOUT_COUNT; i++)
        {
            if (strcmp(name, workout_list[i].name) == 0)
            {
                watch_system_interact(INTERACT_MOTOR_VIBRATE_SLIDING, NULL);
                start_workout_session((workout_type_t)i);
                break;
            }
        }
    }
}

// 删除或查看历史记录
static void delete_history_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_user_data(e);

    // 获取文件名
    lv_obj_t *label = lv_obj_get_child(obj, 0);
    const char *filename = lv_label_get_text(label);

    // 构建完整文件路径
    char file_path[64];
    snprintf(file_path, sizeof(file_path), "/exercise/%s", filename);

    // 删除文件
    if (remove(file_path) == 0)
    {
        LOG_D("File %s deleted successfully", file_path);
    }
    else
    {
        LOG_E("Failed to delete file %s", file_path);
    }

    // 更新UI
    lv_obj_del(obj);
    create_history_list(ui.history_tile);
}

// 历史记录左滑动处理
static void history_swipe_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    if (code == LV_EVENT_GESTURE)
    {
        lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
        if (dir == LV_DIR_LEFT)
        {
            // 创建删除按钮
            LOG_D("Swipe left on history item");
            lv_obj_t *delete_btn = lv_btn_create(obj);
            lv_obj_set_size(delete_btn, 60, lv_obj_get_height(obj));
            lv_obj_align(delete_btn, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(delete_btn, lv_color_hex(0xFF0000), 0);

            lv_obj_t *label = lv_label_create(delete_btn);
            lv_label_set_text(label, "Delete");
            lv_obj_center(label);

            lv_obj_add_event_cb(delete_btn, delete_history_event_handler, LV_EVENT_CLICKED, obj);
        }
    }
}

// 历史记录项目点击处理
static void history_list_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        lv_obj_t *item = lv_event_get_target(e);
        // 获取记录信息
        // 这只是从按钮中获取文件名，我们在实际应用中可能需要更多数据
        const char *filename = lv_list_get_btn_text(ui.history_list, item);
        LOG_D("Selected history record: %s", filename);

        if (!SkaiWatchSys.connected_to_phone)
        {
            ui_show_hint_toast("Phone not connected, cannot sync exercise %s\n", filename);
            return;
        }

        // TODO: 显示历史记录详情或播放动画

        char file_path[40];
        snprintf(file_path, sizeof(file_path), "/exercise/%s", filename);
        int ret = bloc_file_system.sync_file(file_path, false);
        if (ret == 0)
        {
            LOG_D("Sync exercise data successfully");
        }
        else
        {
            LOG_E("Failed to sync exercise data");
        }
    }
}

// 创建历史记录列表视图
static lv_obj_t *create_history_list(lv_obj_t *parent)
{
    // 删除之前的容器内容
    if (lv_obj_get_child_cnt(parent) > 0)
    {
        lv_obj_clean(parent);
    }

    // 创建容器
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_HOR_RES, LV_VER_RES - 60);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    // 添加标题
    lv_obj_t *title = lv_label_create(container);
    lv_label_set_text(title, "Workout History");
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // 添加向上滑动提示
    lv_obj_t *scroll_hint = lv_label_create(container);
    lv_label_set_text(scroll_hint, "▲ swipe down for workouts");
    lv_obj_set_style_text_color(scroll_hint, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(scroll_hint, LV_ALIGN_TOP_MID, 0, 40);

    // 创建列表容器
    lv_obj_t *list = lv_list_create(container);
    lv_obj_set_size(list, lv_pct(100), lv_pct(80));
    lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_pad_row(list, 5, 0);
    lv_obj_set_style_bg_color(list, lv_color_black(), 0);
    ui.history_list = list;

    // 获取历史记录
    workout_history_t *history = get_workout_history();
    if (!history || history->count == 0)
    {
        lv_obj_t *no_history = lv_label_create(container);
        lv_label_set_text(no_history, "No workout history yet");
        lv_obj_set_style_text_color(no_history, lv_color_hex(0xAAAAAA), 0);
        lv_obj_center(no_history);
        free_workout_history(history);
        return container;
    }

    LOG_D("Workout history count: %d", history->count);

    // 填充历史记录列表
    for (int i = 0; i < history->count; i++)
    {
        workout_record_t *record = &history->records[i];
        char date_str[32];
        format_date_string(record->timestamp, date_str, sizeof(date_str));

        // 创建列表项按钮
        lv_obj_t *btn = lv_list_add_btn(list, workout_list[record->type].icon, record->filename);
        lv_obj_set_height(btn, 80);

        // 设置按钮样式
        lv_obj_set_style_radius(btn, 15, 0);
        lv_obj_set_style_bg_color(btn, lv_color_make(100, 160, 200), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_70, 0);

        // 添加详情标签
        lv_obj_t *details = lv_label_create(btn);
        lv_label_set_text_fmt(details, "%s - %d cal - %d min",
                              workout_list[record->type].name,
                              record->calories,
                              record->duration / 60);
        lv_obj_align(details, LV_ALIGN_BOTTOM_LEFT, 60, -5);
        lv_obj_set_style_text_color(details, lv_color_hex(0xDDDDDD), 0);

        // 添加日期标签
        lv_obj_t *date_label = lv_label_create(btn);
        lv_label_set_text(date_label, date_str);
        lv_obj_align(date_label, LV_ALIGN_TOP_LEFT, 60, 5);
        lv_obj_set_style_text_color(date_label, lv_color_hex(0xFFFFFF), 0);

        // 添加点击事件
        lv_obj_add_event_cb(btn, history_list_event_cb, LV_EVENT_CLICKED, NULL);

        // 添加滑动删除功能
        lv_obj_add_event_cb(btn, history_swipe_event_handler, LV_EVENT_GESTURE, NULL);
    }

    // 释放内存
    free_workout_history(history);

    return container;
}

static lv_obj_t *create_workout_list(lv_obj_t *parent)
{
    // Create the list container with dark theme styling
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_style_pad_all(container, 0, 0);
    lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_pad_bottom(container, 60, 0);

    // Add a title with white text
    lv_obj_t *title = lv_label_create(container);
    lv_label_set_text(title, "Start Workout");
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // 创建网格布局容器
    lv_obj_t *grid_container = lv_obj_create(container);
    lv_obj_set_size(grid_container, LV_PCT(90), LV_PCT(70));
    lv_obj_set_style_bg_opa(grid_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(grid_container, 0, 0);
    lv_obj_set_style_pad_all(grid_container, 10, 0);
    lv_obj_align_to(grid_container, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);

    // 网格参数设置
    const int btn_size = 120;   // 按钮大小
    const int h_spacing = 30;   // 水平间距
    const int v_spacing = 40;   // 垂直间距
    const int cols_per_row = 3; // 每行3个
    const int rows = 2;         // 2行

    // 计算布局
    int grid_width = cols_per_row * btn_size + (cols_per_row - 1) * h_spacing;
    int grid_height = rows * btn_size + (rows - 1) * v_spacing;
    int start_x = (lv_obj_get_width(grid_container) - grid_width) / 2;
    int start_y = (lv_obj_get_height(grid_container) - grid_height) / 2;

    // 创建运动图标网格
    for (int i = 0; i < WORKOUT_COUNT; i++)
    {
        int row = i / cols_per_row;
        int col = i % cols_per_row;

        // 当前位置
        int x = start_x + col * (btn_size + h_spacing);
        int y = start_y + row * (btn_size + v_spacing);

        // 创建按钮容器
        lv_obj_t *btn_container = lv_obj_create(grid_container);
        lv_obj_set_size(btn_container, btn_size, btn_size);
        lv_obj_set_pos(btn_container, x, y);
        lv_obj_set_style_radius(btn_container, 20, 0);
        lv_obj_set_style_bg_color(btn_container, lv_color_hex(0x1E1E1E), 0);
        lv_obj_set_style_bg_opa(btn_container, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(btn_container, 0, 0);
        lv_obj_clear_flag(btn_container, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(btn_container, LV_OBJ_FLAG_CLICKABLE);

        // 添加图标
        lv_obj_t *img = lv_img_create(btn_container);
        lv_img_set_src(img, workout_list[i].icon);
        lv_obj_align(img, LV_ALIGN_TOP_MID, 0, 15);

        // 添加文本标签
        lv_obj_t *label = lv_label_create(btn_container);
        lv_label_set_text(label, workout_list[i].name);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
        lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -10);

        // 添加点击事件
        lv_obj_add_event_cb(btn_container, workout_list_event_cb, LV_EVENT_CLICKED, NULL);

        // 储存按钮文本用于后续识别
        lv_obj_set_user_data(btn_container, (void *)workout_list[i].name);
    }

    // Add a icon at the bottom to navigate to history
    lv_obj_t *history_icon = common_icon_button_with_bg(container, &icon_history, nav_to_history_icon_event_cb);
    lv_obj_align(history_icon, LV_ALIGN_BOTTOM_MID, 0, -20);

    return container;
}

static lv_obj_t *create_workout_screen(lv_obj_t *parent)
{
    lv_obj_t *workout_screen = lv_obj_create(parent);
    lv_obj_set_size(workout_screen, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(workout_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(workout_screen, LV_OPA_100, 0);
    // 创建计时标签
    ui.timer_label = lv_label_create(workout_screen);
    lv_obj_set_style_text_font(ui.timer_label, LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_obj_align(ui.timer_label, LV_ALIGN_TOP_MID, 0, 50);
    lv_label_set_text(ui.timer_label, "00:00:00");

    // 创建心率和卡路里指标区域
    lv_obj_t *metrics_container = common_widget_container(workout_screen);
    lv_obj_set_height(metrics_container, 150);
    lv_obj_set_width(metrics_container, LV_PCT(90));
    lv_obj_align_to(metrics_container, ui.timer_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(metrics_container, LV_OPA_50, 0);
    lv_obj_set_style_radius(metrics_container, 20, 0);
    ui.metrics_container = metrics_container;

    // 创建心率图标和标签
    lv_obj_t *hr_icon = lv_img_create(metrics_container);
    lv_img_set_src(hr_icon, &img_red_heart);
    lv_obj_align(hr_icon, LV_ALIGN_TOP_LEFT, 20, 0);
    lv_img_set_zoom(hr_icon, 128);

    ui.heart_rate_label = lv_label_create(metrics_container);
    lv_obj_set_style_text_font(ui.heart_rate_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align_to(ui.heart_rate_label, hr_icon, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    lv_label_set_text(ui.heart_rate_label, "-- BPM");

    // 创建卡路里图标和标签
    lv_obj_t *cal_icon = lv_img_create(metrics_container);
    lv_img_set_src(cal_icon, &img_calories);
    lv_obj_align_to(cal_icon, hr_icon, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    ui.calories_label = lv_label_create(metrics_container);
    lv_obj_set_style_text_font(ui.calories_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align_to(ui.calories_label, cal_icon, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    lv_label_set_text(ui.calories_label, "0 KCAL");

    // 创建暂停按钮
    ui.pause_button = common_icon_button_with_bg(workout_screen, &img_media_pause, pause_button_event_cb);
    // lv_obj_set_style_shadow_width(ui.pause_button, 30, 0);
    // lv_obj_set_style_shadow_color(ui.pause_button, lv_color_hex(0xFFFFFF), 0);
    // lv_obj_set_style_shadow_opa(ui.pause_button, LV_OPA_100, 0);
    lv_obj_align_to(ui.pause_button, metrics_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    // 创建停止按钮
    ui.stop_button = common_text_button(workout_screen, "Stop", NULL, NULL, NULL, stop_button_event_cb);
    lv_obj_align_to(ui.stop_button, ui.pause_button, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    return workout_screen;
}

static void handle_back_event(void)
{
    if (!current_session.workout_timer)
    {
        // 如果在运动列表页面，直接返回到主菜单
        gui_app_self_exit();
    }
    else
    {
        // TODO: 可以添加确认对话框询问是否要停止当前运动
    }
}

static void on_start(void)
{
    // 初始化UI结构
    memset(&ui, 0, sizeof(workout_ui_t));

    // 创建基础UI
    ui.bg = common_black_bg(lv_scr_act());

    // Create tileview (main container for vertical navigation)
    ui.tileview = lv_tileview_create(ui.bg);
    lv_obj_set_size(ui.tileview, LV_HOR_RES, LV_VER_RES);

    // 创建基础UI
    ui.list_tile = lv_tileview_add_tile(ui.tileview, 0, 0, LV_DIR_VER);
    lv_obj_set_pos(ui.list_tile, 0, 0);
    lv_obj_set_size(ui.list_tile, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_user_data(ui.list_tile, (void *)"workout_list");
    ui.workout_list = create_workout_list(ui.list_tile);

    // 创建历史记录视图(第二个tile)
    ui.history_tile = lv_tileview_add_tile(ui.tileview, 0, 1, LV_DIR_VER);
    // add user data
    lv_obj_set_user_data(ui.history_tile, (void *)"history_tile");
    lv_obj_set_pos(ui.history_tile, 0, LV_VER_RES);
    lv_obj_set_size(ui.history_tile, LV_HOR_RES, LV_VER_RES);
    create_history_list(ui.history_tile);

    // 创建运动界面元素
    ui.workout_screen = create_workout_screen(ui.bg);
    // 默认隐藏运动界面
    lv_obj_add_flag(ui.workout_screen, LV_OBJ_FLAG_HIDDEN);
    // 默认显示运动列表
    show_workout_list_view();
}

static void on_resume(void)
{
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_back_event = handle_back_event;
#endif
}

static void on_pause(void)
{
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_back_event = NULL;
#endif
}

static void on_stop(void)
{
    stop_exercise();

    // 清理UI
    lv_obj_del(ui.bg);
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
    gui_app_regist_msg_handler(APP_ID_EXERCISE, msg_handler);
    return 0;
}

// 注册应用
BUILTIN_APP_EXPORT(LV_EXT_STR_ID(exercise), LV_EXT_IMG_GET(img_workout), APP_ID_EXERCISE, app_main);

#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
