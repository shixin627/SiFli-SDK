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
#include "bloc_motion_tracking.h"
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
    #include "watch_global_data.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
#endif
#define ICON_LIST_X_OFFSET (-10)
#define ICON_ITEM_SIZE 80
#define ICON_ZOOM_CENTER 256
#define ICON_ZOOM_MIN 128
#define ICON_OPA_CENTER LV_OPA_COVER           /* 中央 icon 完全不透明 */
#define ICON_OPA_MIN LV_OPA_30                 /* 邊緣 icon 最低透明度（30%）*/
#define ICON_SLOT_HEIGHT (ICON_ITEM_SIZE + 10) /* flex 主軸每格高度（item + pad_row） */
#define ICON_SLOT_ANGLE_DEG 36                 /* 每格對應的角度（等角間距）*/
#define ICON_ARC_RADIUS 200                    /* item 圓心軌跡半徑 */

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

#ifdef APP_ID_EXERCISE

    #define DBG_TAG "app.exercise"
    #define DBG_LVL DBG_LOG
    #include <rtdbg.h>

typedef struct
{
    lv_obj_t *bg;           // 背景
    lv_obj_t *tileview;     // 主视图容器
    lv_obj_t *workout_list; // 运动列表
    lv_obj_t *workout_screen;
    lv_obj_t *timer_label;            // 计时标签
    lv_obj_t *heart_rate_label;       // 心率标签(运动界面)
    lv_obj_t *title_heart_rate_label; // 心率标签(标题栏)
    lv_obj_t *calories_label;         // 卡路里标签
    lv_obj_t *pause_button;           // 暂停按钮
    lv_obj_t *stop_button;            // 停止按钮
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
workout_session_t *get_current_workout_session(void)
{
    return &current_session;
}
static void show_workout_view();
static lv_obj_t *create_workout_list(lv_obj_t *parent);
static void refresh_session_pause_button(void);
static lv_obj_t *cal_label;

static void update_workout_display(void)
{
    if (!lv_obj_is_valid(ui.timer_label))
        return;

    // Format time as hours:minutes:seconds
    uint32_t hours = current_session.duration / 3600;
    uint32_t minutes = (current_session.duration % 3600) / 60;
    uint32_t seconds = current_session.duration % 60;

    char time_str[16];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hours, minutes,
             seconds);
    lv_label_set_text(ui.timer_label, time_str);

    // Update calories display
    char cal_str[16];
    snprintf(cal_str, sizeof(cal_str), "%d", current_session.calories);
    lv_label_set_text(ui.calories_label, cal_str);
    lv_obj_align_to(ui.calories_label, cal_label, LV_ALIGN_OUT_BOTTOM_MID, 0,
                    0);

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
            uint16_t avr_hr_in_min = current_session.sum_of_hr_in_min /
                                     current_session.num_of_hr_in_min;

            // 存储这一分钟的平均心率
            if (current_session.hrm_count < MAX_WORKOUT_MINUTES)
            {
                current_session.hrm_array[current_session.hrm_count] =
                    avr_hr_in_min;
                current_session.hrm_count++;
            }

            // 计算并累加卡路里消耗
            float calories_minute = calculate_calories(
                workout_list[current_session.type].calories_per_min, 60,
                avr_hr_in_min);
            current_session.calories += calories_minute;
            LOG_D("Minute %d: avg HR=%d, calories this minute=%.2f, total "
                  "calories=%.2f",
                  current_session.duration / 60, avr_hr_in_min, calories_minute,
                  current_session.calories);
            // 重置心率累计值，准备下一分钟的计算
            current_session.sum_of_hr_in_min = 0;
            current_session.num_of_hr_in_min = 0;
        }
        else
        {
            LOG_D("No heart rate data for this minute, skipping calorie "
                  "calculation");
        }
    }

    if (app_exercise_data_ctx.session_data)
    {
        // 更新UI显示
        lv_ex_data_set_value(app_exercise_data_ctx.session_data,
                             (void *)&current_session);
    }
}

static void ui_heart_rate_callback(int hr)
{
    // if (hr != current_session.heart_rate)
    {
        LOG_D("[UI]Heart rate changed: %d", hr);
        char bpm_str[16];
        snprintf(bpm_str, sizeof(bpm_str), "%d", current_session.heart_rate);

        // Update workout screen heart rate label
        if (lv_obj_is_valid(ui.heart_rate_label))
        {
            lv_label_set_text(ui.heart_rate_label, bpm_str);
        }
        snprintf(bpm_str, sizeof(bpm_str), "%d", hr);
        // Update title heart rate label
        if (lv_obj_is_valid(ui.title_heart_rate_label))
        {
            lv_label_set_text(ui.title_heart_rate_label, bpm_str);
        }
        // refresh_heartrate_widget(hr);
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
            "workout_timer", workout_timer_cb, RT_NULL, RT_TICK_PER_SECOND,
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

    app_exercise_data_ctx.active = true;

    // Switch to workout view
    show_workout_view();
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
    app_exercise_data_ctx.session_data =
        lv_ex_data_create("exercise.session", LV_EX_DATA_POINTER);
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

                const char *file_path = get_last_exercise_file();
                if (file_path && strlen(file_path) > 0)
                {
                    int sync_ret =
                        bloc_file_system.sync_file((char *)file_path, false);
                    if (sync_ret == 0)
                    {
                        LOG_D("sync last exercise: %s", file_path);
                    }
                    else
                    {
                        LOG_E("failed to sync last exercise: %s", file_path);
                    }
                }
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
                    ui_show_hint_toast("Failed to store exercise data: %d",
                                       ret);
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

    return ret;
}

static void refresh_session_pause_button(void)
{
    lv_obj_t *img = lv_obj_get_child(ui.pause_button, 0);
    lv_img_set_src(img, current_session.is_paused ? &img_media_play
                                                  : &img_media_pause);
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
    if (stop_exercise() != 0)
    {
        ui_show_hint_toast("Failed to stop exercise session");
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
                motor_pattern_touchpad_slide();
                start_workout_session((workout_type_t)i);
                break;
            }
        }
    }
}

static uint16_t old_selected_exercise_index = -1;
static uint16_t selected_exercise_index = 0;
static void press_cb(uint8_t press)
{
    if (press && !lv_obj_has_flag(ui.tileview, LV_OBJ_FLAG_HIDDEN))
    {
        motor_pattern_touchpad_slide();
        start_workout_session((workout_type_t)selected_exercise_index);
    }
}

static void handle_back_event(void)
{
    if (app_exercise_data_ctx.active)
    {
        stop_exercise();
    }
    else
    {
        gui_app_self_exit();
    }
}

/* 仿 lv_instruction_list_layout.c 的指示點做法：
 *   - icons 是 list 的 floating child，用 lv_obj_set_pos 直接放在環上
 *   - 每個 icon 有固定 base_angle = i * step；offset_angle 由 scroll_y 推導
 *   - 整個 ring 隨 scroll 旋轉，不走 flex/translate 那條鏈
 *   - 另外建 7 個透明 snap_targets 提供 LVGL snap-to-center 用的線性 y anchor
 *   - labels 是 icon 的 child，跟著 icon 移動，預設藏起來只顯示選中的那個 */
static lv_obj_t *workout_icons[WORKOUT_COUNT] = {0};
static lv_obj_t *workout_name_labels[WORKOUT_COUNT] = {0};

static void update_workout_name_label(void)
{
    if (selected_exercise_index >= WORKOUT_COUNT) return;
    for (int j = 0; j < WORKOUT_COUNT; j++)
    {
        if (!lv_obj_is_valid(workout_name_labels[j])) continue;
        if (j == selected_exercise_index)
        {
            lv_obj_clear_flag(workout_name_labels[j], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(workout_name_labels[j], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* 弧形排列：仿 lv_instruction_list_layout.c::update_indicator_dots_position 的做法。
 *
 *   offset_angle = (scroll_y - base_scroll) / SLOT × angle_per_slot
 *   current_angle_i = i × angle_per_slot - offset_angle  (含 ±π 環繞)
 *   icon_screen_pos = (cx + R·cos(angle), cy + R·sin(angle))
 *
 * 用 lv_obj_set_pos 直接放，不經過 flex / translate。
 * 同時找出 |angle| 最小的 icon 設成 selected，並更新 label 顯示。 */
static void apply_circular_layout(lv_obj_t *list)
{
    const int32_t cx = LV_HOR_RES / 2;
    const int32_t cy = LV_VER_RES / 2;
    const float angle_per_slot = (float)ICON_SLOT_ANGLE_DEG * (M_PI / 180.0f);
    const lv_coord_t list_x1 = list->coords.x1;
    const lv_coord_t list_y1 = list->coords.y1;
    const lv_coord_t pad_top = lv_obj_get_style_pad_top(list, LV_PART_MAIN);
    const lv_coord_t pad_left = lv_obj_get_style_pad_left(list, LV_PART_MAIN);

    /* base_scroll = snap_target i 被中央時對應的 scroll_y（i = 0 的情況）*/
    const int32_t base_scroll = list_y1 + pad_top + ICON_ITEM_SIZE / 2 - cy;
    const lv_coord_t scroll_y = lv_obj_get_scroll_y(list);
    const float scroll_in_slots = (float)(scroll_y - base_scroll) / ICON_SLOT_HEIGHT;
    const float offset_angle = scroll_in_slots * angle_per_slot;

    int closest_i = 0;
    float min_abs_angle = M_PI;

    for (int i = 0; i < WORKOUT_COUNT; i++)
    {
        if (workout_icons[i] == NULL) continue;

        float current_angle = (float)i * angle_per_slot - offset_angle;
        while (current_angle > M_PI) current_angle -= 2.0f * M_PI;
        while (current_angle <= -M_PI) current_angle += 2.0f * M_PI;
        float abs_angle = fabsf(current_angle);

        if (abs_angle < min_abs_angle)
        {
            min_abs_angle = abs_angle;
            closest_i = i;
        }

        if (abs_angle > M_PI / 2.0f)
        {
            lv_obj_add_flag(workout_icons[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(workout_icons[i], LV_OBJ_FLAG_HIDDEN);

        /* 環上的螢幕座標 → list 內部 set_pos 座標
         *   screen = list.x1 + pad_left + set_pos_x  (LVGL floating obj 規則) */
        int32_t icon_sx = cx + (int32_t)(ICON_ARC_RADIUS * cosf(current_angle));
        int32_t icon_sy = cy + (int32_t)(ICON_ARC_RADIUS * sinf(current_angle));
        int32_t local_x = icon_sx - list_x1 - pad_left - ICON_ITEM_SIZE / 2;
        int32_t local_y = icon_sy - list_y1 - pad_top - ICON_ITEM_SIZE / 2;
        lv_obj_set_pos(workout_icons[i], local_x, local_y);

        int32_t zoom_range = ICON_ZOOM_CENTER - ICON_ZOOM_MIN;
        int32_t zoom = ICON_ZOOM_MIN + (int32_t)(zoom_range * cosf(abs_angle));
        lv_img_set_zoom(workout_icons[i], zoom);

        /* opacity：中央 = COVER (255)，邊緣 = MIN (≈76)，用 cos(angle) 做插值 */
        int32_t opa_range = ICON_OPA_CENTER - ICON_OPA_MIN;
        lv_opa_t opa = ICON_OPA_MIN + (lv_opa_t)(opa_range * cosf(abs_angle));
        lv_obj_set_style_img_opa(workout_icons[i], opa, 0);
    }

    /* 更新 label：closest_i 顯示，其他隱藏 */
    if (closest_i != selected_exercise_index)
    {
        selected_exercise_index = closest_i;
    }
    for (int i = 0; i < WORKOUT_COUNT; i++)
    {
        if (workout_name_labels[i] == NULL) continue;
        if (i == closest_i && min_abs_angle <= M_PI / 2.0f)
        {
            lv_obj_clear_flag(workout_name_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(workout_name_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void scroll_list(lv_obj_t *obj)
{
    /* apply_circular_layout 已經依 scroll_y 算出 closest_i，更新
     * selected_exercise_index，並切換 label 顯示。這裡只剩 old_selected
     * 偵測，避免重複工作 */
    apply_circular_layout(obj);

    old_selected_exercise_index = selected_exercise_index;
}

static void list_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;
    if (obj == NULL)
    {
        return;
    }
    switch (evt->code)
    {
    case LV_EVENT_SCROLL:
        scroll_list(obj);
        break;
    default:
        break;
    }
}

static lv_obj_t *create_workout_list(lv_obj_t *parent)
{
    lv_obj_t *list_container = lv_obj_create(parent);
    lv_obj_set_size(list_container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(list_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_container, 0, 0);
    lv_obj_set_style_pad_all(list_container, 0, 0);
    lv_obj_clear_flag(list_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(list_container, LV_ALIGN_TOP_MID, 0, 0);

    /* List 是全螢幕的 scrollable 容器（無 flex），仿
     * lv_instruction_list_layout.c 的 p_instruction_list 做法 */
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
    lv_obj_add_event_cb(list, list_scroll_event_cb, LV_EVENT_ALL, NULL);
    ui.workout_list = list;

    /* 7 個透明 snap_targets（non-floating），只負責提供 LVGL 的 snap-to-center
     * anchors，視覺上不顯示 */
    for (int i = 0; i < WORKOUT_COUNT; i++)
    {
        lv_obj_t *snap_target = lv_obj_create(list);
        lv_obj_set_size(snap_target, ICON_ITEM_SIZE, ICON_ITEM_SIZE);
        lv_obj_set_pos(snap_target, 0, i * ICON_SLOT_HEIGHT);
        lv_obj_set_style_bg_opa(snap_target, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(snap_target, 0, 0);
        lv_obj_set_style_pad_all(snap_target, 0, 0);
        lv_obj_clear_flag(snap_target, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(snap_target, LV_OBJ_FLAG_SCROLLABLE);
    }

    /* 7 個 visible icons：list 的 floating child（不影響 scroll content size）。
     * 位置由 apply_circular_layout 用 lv_obj_set_pos 直接放，不走 flex/translate。
     * clickable，drag 會 bubble 給 list 觸發 scroll。 */
    for (int i = 0; i < WORKOUT_COUNT; i++)
    {
        workout_icons[i] = lv_img_create(list);
        lv_img_set_src(workout_icons[i], workout_list[i].icon);
        lv_img_set_size_mode(workout_icons[i], LV_IMG_SIZE_MODE_REAL);
        const lv_img_dsc_t *dsc = (const lv_img_dsc_t *)workout_list[i].icon;
        lv_img_set_pivot(workout_icons[i], dsc->header.w / 2, dsc->header.h / 2);
        lv_obj_add_flag(workout_icons[i], LV_OBJ_FLAG_FLOATING);
        lv_obj_add_flag(workout_icons[i], LV_OBJ_FLAG_OVERFLOW_VISIBLE);
        lv_obj_add_flag(workout_icons[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(workout_icons[i], (void *)workout_list[i].name);
        lv_obj_add_event_cb(workout_icons[i], workout_list_event_cb,
                            LV_EVENT_CLICKED, NULL);

        /* label 是 icon 的 child；click 會 bubble 給 icon 觸發 workout。
         * 對齊到 icon 左外側，當 icon 在 angle=0 時 label 視覺中心 ~ 螢幕 x=180 */
        workout_name_labels[i] = lv_label_create(workout_icons[i]);
        lv_label_set_text(workout_name_labels[i], workout_list[i].name);
        lv_obj_set_style_text_color(workout_name_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(workout_name_labels[i],
                                   LV_EXT_FONT_GET(get_system_font_size(1)), 0);
        lv_obj_set_width(workout_name_labels[i], 250);
        lv_obj_set_style_text_align(workout_name_labels[i],
                                    LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(workout_name_labels[i], LV_ALIGN_LEFT_MID, -338, 0);
        if (i != 0)
        {
            lv_obj_add_flag(workout_name_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    lv_obj_update_layout(list);
    /* 滾到第一個 snap target 中央，然後 apply 一次把 icons 放上環 */
    lv_obj_scroll_to_view(lv_obj_get_child(list, 0), LV_ANIM_OFF);
    apply_circular_layout(list);

    /* 頂端中央心率顯示 */
    lv_obj_t *hr_container = lv_obj_create(list_container);
    lv_obj_set_size(hr_container, 200, 60);
    lv_obj_set_style_bg_opa(hr_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(hr_container, 0, 0);
    lv_obj_set_style_pad_all(hr_container, 0, 0);
    lv_obj_clear_flag(hr_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(hr_container, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t *heart_icon = lv_img_create(hr_container);
    lv_img_set_src(heart_icon, &img_red_heart);
    lv_obj_align(heart_icon, LV_ALIGN_CENTER, -30, 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, heart_icon);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_zoom);
    lv_anim_set_values(&a, 75, 100);
    lv_anim_set_time(&a, 2000);
    lv_anim_set_playback_time(&a, 2000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);

    ui.title_heart_rate_label = lv_label_create(hr_container);
    lv_label_set_text(ui.title_heart_rate_label, "--");
    lv_obj_set_style_text_font(ui.title_heart_rate_label,
                               LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(ui.title_heart_rate_label,
                                lv_color_hex(0xFFFFFF), 0);
    lv_obj_align_to(ui.title_heart_rate_label, heart_icon,
                    LV_ALIGN_OUT_RIGHT_MID, -5, 0);

    return list_container;
}

static void start_hr_icon_anim(lv_obj_t *hr_icon)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, hr_icon);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_zoom);
    lv_anim_set_values(&a, 100, 128);
    lv_anim_set_time(&a, 700);
    lv_anim_set_playback_time(&a, 700);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

static lv_obj_t *create_workout_screen(lv_obj_t *parent)
{
    lv_obj_t *workout_screen = lv_obj_create(parent);
    lv_obj_set_size(workout_screen, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(workout_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(workout_screen, LV_OPA_100, 0);
    // 创建计时标签
    ui.timer_label = lv_label_create(workout_screen);
    lv_obj_set_style_text_font(ui.timer_label,
                               LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_align(ui.timer_label, LV_ALIGN_TOP_MID, 0, 70);
    lv_label_set_text(ui.timer_label, "00:00:00");

    // 创建卡路里图标和标签
    cal_label = lv_label_create(workout_screen);
    lv_obj_align(cal_label, LV_ALIGN_TOP_MID, 0, 230); // LV_ALIGN_TOP_LEFT 100
    lv_obj_set_style_text_opa(cal_label, LV_OPA_70, 0);
    lv_label_set_text(cal_label, "KCAL");

    ui.calories_label = lv_label_create(workout_screen);
    lv_obj_set_style_text_font(ui.calories_label,
                               LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_label_set_text(ui.calories_label, "0");
    lv_obj_align_to(ui.calories_label, cal_label, LV_ALIGN_OUT_BOTTOM_MID, 0,
                    0);

    // 创建心率图标和标签
    lv_obj_t *hr_icon = lv_img_create(workout_screen);
    lv_img_set_src(hr_icon, &img_red_heart);
    lv_obj_align(hr_icon, LV_ALIGN_BOTTOM_MID, -28, -10);
    lv_img_set_zoom(hr_icon, 100); // 初始 100
    start_hr_icon_anim(hr_icon);

    ui.heart_rate_label = lv_label_create(workout_screen);
    lv_obj_set_style_text_font(ui.heart_rate_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(ui.heart_rate_label, hr_icon, LV_ALIGN_OUT_RIGHT_MID, -7,
                    0);
    lv_label_set_text(ui.heart_rate_label, "--");

    ui.pause_button = lv_btn_create(workout_screen);
    lv_obj_set_size(ui.pause_button, 120, 80);
    lv_obj_set_style_radius(ui.pause_button, 50, 0);
    lv_obj_set_style_bg_color(ui.pause_button, lv_color_hex(0x333333), 0);
    lv_obj_align(ui.pause_button, LV_ALIGN_TOP_RIGHT, -40, 250);
    lv_obj_t *pause_img = lv_img_create(ui.pause_button);
    lv_img_set_src(pause_img, &img_media_pause);
    lv_img_set_zoom(pause_img, 128);
    lv_obj_center(pause_img);

    lv_obj_add_event_cb(ui.pause_button, pause_button_event_cb,
                        LV_EVENT_CLICKED, NULL);

    ui.stop_button = lv_btn_create(workout_screen);
    lv_obj_set_size(ui.stop_button, 120, 80);
    lv_obj_set_style_radius(ui.stop_button, 50, 0);
    lv_obj_set_style_bg_color(ui.stop_button, lv_color_hex(0x333333), 0);
    lv_obj_align(ui.stop_button, LV_ALIGN_TOP_LEFT, 40, 250);
    lv_obj_t *stop_label = lv_label_create(ui.stop_button);
    lv_label_set_text(stop_label, "STOP");
    lv_obj_center(stop_label);
    lv_obj_add_event_cb(ui.stop_button, stop_button_event_cb, LV_EVENT_CLICKED,
                        NULL);
    return workout_screen;
}

static void scroll_list_to_index(int8_t page)
{
    LOG_D("Scrolling to workout list index: %d", page);
    if (page < 0 || page > WORKOUT_COUNT + 1)
        return;
    // 获取列表对象
    lv_obj_t *list = ui.workout_list;
    if (!lv_obj_is_valid(list))
        return;

    // 计算目标对象的位置
    lv_obj_t *target_item = lv_obj_get_child(list, page);
    if (!lv_obj_is_valid(target_item))
        return;

    // 滚动到目标对象
    lv_obj_scroll_to_view(target_item, LV_ANIM_ON);
}

static void on_start(void)
{
    // 初始化UI结构
    memset(&ui, 0, sizeof(workout_ui_t));

    // 创建基础UI
    ui.bg = common_black_bg(lv_scr_act());

    ui.tileview = create_workout_list(ui.bg);

    ui.workout_screen = create_workout_screen(ui.bg);
    lv_obj_add_flag(ui.workout_screen, LV_OBJ_FLAG_HIDDEN);
}

static void on_resume(void)
{
    // Set up heart rate monitoring
    sensor_subscription_t sensor_subscription = (sensor_subscription_t){
        .type = SENSOR_TYPE_HEART_RATE,
        .status = true,
    };
    watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
    lvgl_msg_handler.handle_hr = ui_heart_rate_callback;
    set_scroll_segment_count(WORKOUT_COUNT);
    set_prev_sensor_quat(0);
    #ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_back_event = handle_back_event;
    #endif
    lvgl_msg_handler.handle_nav_bar_control = scroll_list_to_index;
    lvgl_msg_handler.handle_tap_indicator = press_cb;
}

static void on_pause(void)
{
    #ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_back_event = NULL;
    #endif
    if (lvgl_msg_handler.handle_nav_bar_control == scroll_list_to_index)
    {
        lvgl_msg_handler.handle_nav_bar_control = NULL;
    }
    if (lvgl_msg_handler.handle_tap_indicator == press_cb)
    {
        lvgl_msg_handler.handle_tap_indicator = NULL;
    }
    sensor_subscription_t sensor_subscription = (sensor_subscription_t){
        .type = SENSOR_TYPE_HEART_RATE,
        .status = false,
    };
    watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
    LOG_D("Exercise app paused");
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

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(exercise), LV_EXT_IMG_GET(img_workout),
                   APP_ID_EXERCISE, app_main);

#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
