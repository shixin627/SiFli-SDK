/**
 ******************************************************************************
 * @file   app_developer.c
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
#include <rtthread.h>
#include <rtdevice.h>
#include <dfs_posix.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

// Define file flags if not available
#ifndef O_CREAT
#define O_CREAT 0x0100
#endif
#ifndef O_WRONLY
#define O_WRONLY 0x0001
#endif
#ifndef O_TRUNC
#define O_TRUNC 0x0400
#endif

#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "custom_trans_anim.h"
#include "app_mainmenu.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#endif
#ifdef BSP_USING_BLOC
#include "bloc_control.h"
#include "bloc_notification.h"
#include "bloc_peripheral.h"
#endif
#ifdef BSP_USING_WATCH_SYS_CLIENT
#include "watch_sys_service.h"
#endif

#define DBG_TAG "app.developer"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Referenced by bloc_peripheral PPG thread — kept outside the wrap. */
bool ppg_data_collection = false;

#if !kReleaseMode

#ifdef WIN32
void lv_gpu_set_enable(bool en) {};
bool lv_gpu_is_enabled(void)
{
    return false;
};
#else
extern void lv_gpu_set_enable(bool en);
extern bool lv_gpu_is_enabled(void);
#endif

extern int log_file_backend_is_enabled(void);
extern int log_file_backend_set_enabled(int enable);

extern void accelerometer_subscribe(void);
extern void accelerometer_unsubscribe(void);
extern void heart_rate_subscribe(void);
extern void heart_rate_unsubscribe(void);

// FS test related variables
static bool fs_test_running = false;
static int fs_test_file_count = 0;
static lv_obj_t *fs_info_label;
static lv_obj_t *fs_test_btn;
static lv_timer_t *fs_update_timer;

// Add these function prototypes
static void test_all_callback(lv_event_t *e);
static void restart_callback(lv_event_t *e);
static void reset_restart_num_callback(lv_event_t *e);
static void ble_log_sw_event_callback(lv_event_t *e);
static void file_log_sw_event_callback(lv_event_t *e);
static void accel_sub_unsub_sw_event_callback(lv_event_t *e);
static void hr_sub_unsub_sw_event_callback(lv_event_t *e);
static void imu_data_collection_sw_event_callback(lv_event_t *e);
static void imu_data_collection_error_sw_event_callback(lv_event_t *e);
static void imu_raw_data_collection_sw_event_callback(lv_event_t *e);
static void imu_lock_sw_event_callback(lv_event_t *e);
static void ppg_data_collection_sw_event_callback(lv_event_t *e);
static void tap_and_hold_sw_event_callback(lv_event_t *e);
static void motor_switch_event_callback(lv_event_t *e);
static void fps_cpu_load_switch_event_callback(lv_event_t *e);
static void EPIC_switch_event_callback(lv_event_t *e);
static void fs_test_callback(lv_event_t *e);
static void fs_update_info_cb(lv_timer_t *timer);

// indicates whether data collection from an Inertial Measurement Unit (IMU) is active or enabled.
// indicates whether raw data collection from the IMU is active or enabled.

extern bool pause_sleep_cause_of_imu_reson(void);
bool pause_sleep_cause_of_dev_reson(void)
{
    return ppg_data_collection || pause_sleep_cause_of_imu_reson();
}

static lv_obj_t *accel_data_label;
static lv_obj_t *gyro_data_label;
static lv_obj_t *attitude_data_label;
static lv_obj_t *hr_data_label;
static lv_obj_t *battery_voltage_label;
static lv_obj_t *battery_percentage_label;
static lv_obj_t *slider;
static lv_timer_t *battery_request_timer = NULL;
static bool battery_request_enabled = false;
static lv_obj_t *reset_restart_num_btn = NULL;
static lv_obj_t *reset_restart_num_label = NULL;
static void back_btn_event_callback(lv_event_t *e)
{
    // lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();
    }
}

static void get_battery_voltage_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        watch_system_interact(WATCH_REQUEST_BATTERY, NULL);
    }
}

/**
 * @brief Timer callback to request battery voltage every second
 */
static void battery_request_timer_cb(lv_timer_t *timer)
{
    watch_system_interact(WATCH_REQUEST_BATTERY, NULL);
}

/**
 * @brief Battery request switch event callback
 */
static void battery_request_switch_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        battery_request_enabled = !battery_request_enabled;

        if (battery_request_enabled)
        {
            // Start timer - request battery every 1000ms
            if (battery_request_timer == NULL)
            {
                battery_request_timer = lv_timer_create(battery_request_timer_cb, 1000, NULL);
            }
            LOG_I("Battery request timer started");
        }
        else
        {
            // Stop timer
            if (battery_request_timer != NULL)
            {
                lv_timer_del(battery_request_timer);
                battery_request_timer = NULL;
            }
            LOG_I("Battery request timer stopped");
        }
    }
}

static void get_charge_status_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        watch_system_interact(WATCH_REQUEST_CHARGE_STATUS, NULL);
    }
}

static void vibrate_motor_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        motor_params_t param = {
            .duty_cycle = 50,  // 50%
            .period = 200000,  // 0.2s
            .repeat_times = 5, // 5 times
        };
        peripheral_provider.control_motor(true, &param);
    }
}

static void random_address_btn_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        extern void generate_random_public_address(uint8_t device_id);
        generate_random_public_address(0);
        rt_thread_mdelay(50);
        watch_system_interact(WATCH_REBOOT, NULL);
    }
}

#ifndef BSP_USING_PC_SIMULATOR
extern bool ppg_service_subscribed(void);
#else
bool ppg_service_subscribed(void)
{
    return false;
}
#endif

static void lv_create_dev_screen(void)
{
    // header
    lv_obj_t *title = lv_lvsfheader_create(lv_scr_act());
    lv_lvsfheader_set_title(title, "Developer App");
    lv_lvsfheader_set_visible_item(title, LVSF_HEADER_BRANCH);
    lv_lvsfheader_back_event_cb(title, back_btn_event_callback);

    cust_trans_anim_config(CUST_ANIM_TYPE_2, NULL);

    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_align_to(cont, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    lv_obj_set_size(cont, LV_HOR_RES, LV_VER_RES - lv_obj_get_height(title));
    // 設置佈局為列佈局
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);

    // 設置子對象的對齊方式為居中
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    /* Test All */
    lv_obj_t *test_all_btn = common_text_button(cont, "Test All", NULL, NULL, NULL, test_all_callback);
    /* 重新啟動 */
    lv_obj_t *restart_btn = common_text_button(cont, "Restart", NULL, NULL, NULL, restart_callback);
    /* 重置重啟次數 */
    char restart_num_text[32];
    snprintf(restart_num_text, sizeof(restart_num_text), "Reset Restart Num (%d)", SkaiWatchSys.watch_restart_num);
    reset_restart_num_btn = common_text_button(cont, restart_num_text, NULL, NULL, NULL, reset_restart_num_callback);
    reset_restart_num_label = lv_obj_get_child(reset_restart_num_btn, 0); // 獲取按鈕中的 label
    /* 開啟藍芽log */
    lv_obj_t *ble_log_container = lv_obj_create(cont);
    lv_obj_set_size(ble_log_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(ble_log_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ble_log_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(ble_log_container, restart_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);

    lv_obj_t *ble_log_label = lv_label_create(ble_log_container);
    lv_label_set_text(ble_log_label, "BLE log");
    lv_obj_set_style_text_font(ble_log_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    lv_obj_t *ble_log_sw = lv_switch_create(ble_log_container);
    lv_obj_add_state(ble_log_sw, SkaiWatchSys.flag_field.debug_mode ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(ble_log_sw, 100, 50);
    lv_obj_add_event_cb(ble_log_sw, ble_log_sw_event_callback, LV_EVENT_CLICKED, NULL);

    /* File log switch (writes logs to /logs/ ; default OFF) */
    lv_obj_t *file_log_container = lv_obj_create(cont);
    lv_obj_set_size(file_log_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(file_log_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(file_log_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(file_log_container, ble_log_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);

    lv_obj_t *file_log_label = lv_label_create(file_log_container);
    lv_label_set_text(file_log_label, "File log");
    lv_obj_set_style_text_font(file_log_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    lv_obj_t *file_log_sw = lv_switch_create(file_log_container);
    lv_obj_add_state(file_log_sw, log_file_backend_is_enabled() ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(file_log_sw, 100, 50);
    lv_obj_add_event_cb(file_log_sw, file_log_sw_event_callback, LV_EVENT_CLICKED, NULL);

    /* [IMU] Create accelerometer subscription/unsubscription label and switch */
    extern bool accel_service_subscribed(void);
    lv_obj_t *accel_sub_unsub_container = lv_obj_create(cont);
    lv_obj_set_size(accel_sub_unsub_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(accel_sub_unsub_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(accel_sub_unsub_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(accel_sub_unsub_container, file_log_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);

    bool is_subscribed = accel_service_subscribed();
    lv_obj_t *accel_sub_unsub_label = lv_label_create(accel_sub_unsub_container);
    lv_label_set_text(accel_sub_unsub_label, "Toggle IMU");
    lv_obj_set_style_text_font(accel_sub_unsub_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    lv_obj_t *accel_sub_unsub_sw = lv_switch_create(accel_sub_unsub_container);
    lv_obj_add_state(accel_sub_unsub_sw, is_subscribed ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(accel_sub_unsub_sw, 100, 50);
    lv_obj_add_event_cb(accel_sub_unsub_sw, accel_sub_unsub_sw_event_callback, LV_EVENT_CLICKED, NULL);

    // Create a label to display accelerometer data
    accel_data_label = lv_label_create(cont);
    lv_label_set_text(accel_data_label, "Accel: x, y, z");
    lv_obj_set_style_text_font(accel_data_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(accel_data_label, accel_sub_unsub_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    // Display attitude data
    attitude_data_label = lv_label_create(cont);
    lv_label_set_text(attitude_data_label, "Attitude: x, y, z");
    lv_obj_set_style_text_font(attitude_data_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(attitude_data_label, accel_data_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* IMU Data Collection Section */
    lv_obj_t *imu_section_title = lv_label_create(cont);
    lv_label_set_text(imu_section_title, "IMU Data Collection");
    lv_obj_set_style_text_font(imu_section_title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(imu_section_title, attitude_data_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    /* Create IMU container with grid layout for 2x2 or 2x3 arrangement */
    lv_obj_t *imu_container = lv_obj_create(cont);
    lv_obj_set_size(imu_container, LV_HOR_RES - 40, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(imu_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(imu_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(imu_container, imu_section_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* Row 2: IMU RAW data collection + lock IMU */
    lv_obj_t *imu_row2 = lv_obj_create(imu_container);
    lv_obj_set_size(imu_row2, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(imu_row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(imu_row2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *imu_raw_container = lv_obj_create(imu_row2);
    lv_obj_set_size(imu_raw_container, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(imu_raw_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(imu_raw_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(imu_raw_container, 5, 0);

    lv_obj_t *imu_raw_data_collection_label = lv_label_create(imu_raw_container);
    lv_label_set_text(imu_raw_data_collection_label, "RAW data");
    lv_obj_set_style_text_font(imu_raw_data_collection_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    lv_obj_t *imu_raw_data_collection_sw = lv_switch_create(imu_raw_container);
    lv_obj_add_state(imu_raw_data_collection_sw, imu_raw_data_collection ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(imu_raw_data_collection_sw, 80, 40);
    lv_obj_add_event_cb(imu_raw_data_collection_sw, imu_raw_data_collection_sw_event_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lock_imu_container_else = lv_obj_create(imu_row2);
    lv_obj_set_size(lock_imu_container_else, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(lock_imu_container_else, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(lock_imu_container_else, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(lock_imu_container_else, 5, 0);

    lv_obj_t *lock_imu_label_else = lv_label_create(lock_imu_container_else);
    lv_label_set_text(lock_imu_label_else, "lock IMU");
    lv_obj_set_style_text_font(lock_imu_label_else, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    lv_obj_t *lock_imu_sw_else = lv_switch_create(lock_imu_container_else);
    lv_obj_add_state(lock_imu_sw_else, false ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(lock_imu_sw_else, 80, 40);
    lv_obj_add_event_cb(lock_imu_sw_else, imu_lock_sw_event_callback, LV_EVENT_CLICKED, NULL);

    /* [PPG] Create PPG subscription/unsubscription label and switch */
    lv_obj_t *hr_sub_unsub_container = lv_obj_create(cont);
    lv_obj_set_size(hr_sub_unsub_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(hr_sub_unsub_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hr_sub_unsub_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(hr_sub_unsub_container, imu_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    lv_obj_t *hr_sub_unsub_label = lv_label_create(hr_sub_unsub_container);
    is_subscribed = ppg_service_subscribed();
    lv_label_set_text(hr_sub_unsub_label, "Toggle PPG");
    lv_obj_set_style_text_font(hr_sub_unsub_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    lv_obj_t *hr_sub_unsub_sw = lv_switch_create(hr_sub_unsub_container);
    lv_obj_add_state(hr_sub_unsub_sw, is_subscribed ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(hr_sub_unsub_sw, 100, 50);
    lv_obj_add_event_cb(hr_sub_unsub_sw, hr_sub_unsub_sw_event_callback, LV_EVENT_CLICKED, NULL);

    /* 開啟PPG收集數據模式 */
    lv_obj_t *ppg_data_collection_label = lv_label_create(cont);
    lv_label_set_text(ppg_data_collection_label, "PPG data collection");
    lv_obj_set_style_text_font(ppg_data_collection_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_t *ppg_data_collection_sw = lv_switch_create(cont);
    lv_obj_add_state(ppg_data_collection_sw, ppg_data_collection ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(ppg_data_collection_sw, 100, 50);
    lv_obj_align_to(ppg_data_collection_label, hr_sub_unsub_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_align_to(ppg_data_collection_sw, ppg_data_collection_label, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(ppg_data_collection_sw, ppg_data_collection_sw_event_callback, LV_EVENT_CLICKED, NULL);

    // Create a label to display heart rate data
    hr_data_label = lv_label_create(cont);
    lv_label_set_text(hr_data_label, "HR: 0 bpm");
    lv_obj_set_style_text_font(hr_data_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(hr_data_label, ppg_data_collection_sw, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* [Tap and Hold] Create Tap and Hold mode switch */
    lv_obj_t *tap_and_hold_container = lv_obj_create(cont);
    lv_obj_set_size(tap_and_hold_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(tap_and_hold_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(tap_and_hold_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(tap_and_hold_container, hr_data_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    lv_obj_t *tap_and_hold_label = lv_label_create(tap_and_hold_container);
    lv_label_set_text(tap_and_hold_label, "Tap and Hold");
    lv_obj_set_style_text_font(tap_and_hold_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    extern bool get_enable_tap_and_hold(void);
    lv_obj_t *tap_and_hold_sw = lv_switch_create(tap_and_hold_container);
    lv_obj_add_state(tap_and_hold_sw, get_enable_tap_and_hold() ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(tap_and_hold_sw, 100, 50);
    lv_obj_add_event_cb(tap_and_hold_sw, tap_and_hold_sw_event_callback, LV_EVENT_CLICKED, NULL);

    /* [Battery] Create a button to request battery voltage */
    lv_obj_t *get_battery_voltage_btn = common_text_button(cont, "battery voltage", get_system_font_size(0), 366, 100, get_battery_voltage_btn_event_cb);
    lv_obj_align_to(get_battery_voltage_btn, tap_and_hold_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    // display voltage
    battery_voltage_label = lv_label_create(cont);
    lv_label_set_text(battery_voltage_label, " - mV");
    lv_obj_set_style_text_font(battery_voltage_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(battery_voltage_label, get_battery_voltage_btn, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    battery_percentage_label = lv_label_create(cont);
    lv_label_set_text(battery_percentage_label, " - %%");
    lv_obj_set_style_text_font(battery_percentage_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(battery_percentage_label, battery_voltage_label, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* [Battery Request Switch] Create a switch to auto request battery every second */
    lv_obj_t *battery_request_switch_label = lv_label_create(cont);
    lv_label_set_text(battery_request_switch_label, "Auto Request Battery");
    lv_obj_set_style_text_font(battery_request_switch_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(battery_request_switch_label, get_battery_voltage_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_t *battery_request_switch = lv_switch_create(cont);
    lv_obj_add_state(battery_request_switch, battery_request_enabled ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(battery_request_switch, 100, 50);
    lv_obj_align_to(battery_request_switch, battery_request_switch_label, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    lv_obj_add_event_cb(battery_request_switch, battery_request_switch_event_callback, LV_EVENT_CLICKED, NULL);

    /* [Charger] Create a button to request charge status */
    lv_obj_t *get_charge_status_btn = common_text_button(cont, "charge status", get_system_font_size(0), 366, 100, get_charge_status_btn_event_cb);
    lv_obj_align_to(get_charge_status_btn, battery_request_switch, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* [Motor] Create a switch to control motor state */
    lv_obj_t *motor_switch_label = lv_label_create(cont);
    lv_label_set_text(motor_switch_label, "Motor Switch");
    lv_obj_set_style_text_font(motor_switch_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(motor_switch_label, get_charge_status_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_t *motor_switch = lv_switch_create(cont);
    lv_obj_add_state(motor_switch, get_motor_switch_state() ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(motor_switch, 100, 50);
    lv_obj_align_to(motor_switch, motor_switch_label, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    lv_obj_add_event_cb(motor_switch, motor_switch_event_callback, LV_EVENT_CLICKED, NULL);

    /* [Motor] create a button to vibrate motor */
    lv_obj_t *vibrate_motor_btn = common_text_button(cont, "Vibrate Motor", get_system_font_size(0), 366, 100, vibrate_motor_btn_event_cb);
    lv_obj_align_to(vibrate_motor_btn, motor_switch, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_t *ble_random_address_btn = common_text_button(cont, "random address", get_system_font_size(0), 366, 100, random_address_btn_event_callback);
    lv_obj_align_to(ble_random_address_btn, vibrate_motor_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* [Performance] FPS & CPU load switch */
    lv_obj_t *fps_cpu_container = lv_obj_create(cont);
    lv_obj_set_size(fps_cpu_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(fps_cpu_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(fps_cpu_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(fps_cpu_container, ble_random_address_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    lv_obj_t *fps_cpu_label = lv_label_create(fps_cpu_container);
    lv_label_set_text(fps_cpu_label, "FPS & CPU load");
    lv_obj_set_style_text_font(fps_cpu_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    lv_obj_t *fps_cpu_sw = lv_switch_create(fps_cpu_container);
    lv_obj_add_state(fps_cpu_sw, get_display_fps_and_cpu_load() ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(fps_cpu_sw, 100, 50);
    lv_obj_add_event_cb(fps_cpu_sw, fps_cpu_load_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

    /* [GPU] GPU switch */
    lv_obj_t *gpu_container = lv_obj_create(cont);
    lv_obj_set_size(gpu_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(gpu_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(gpu_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(gpu_container, fps_cpu_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_t *gpu_label = lv_label_create(gpu_container);
    lv_label_set_text(gpu_label, "GPU");
    lv_obj_set_style_text_font(gpu_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    lv_obj_t *gpu_sw = lv_switch_create(gpu_container);
    lv_obj_add_state(gpu_sw, lv_gpu_is_enabled() ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(gpu_sw, 100, 50);
#ifdef WIN32
    lv_obj_add_state(gpu_sw, LV_STATE_DISABLED);
#endif
    lv_obj_add_event_cb(gpu_sw, EPIC_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

    /* [FS Test] File System Test */
    lv_obj_t *fs_test_title = lv_label_create(cont);
    lv_label_set_text(fs_test_title, "File System Test");
    lv_obj_set_style_text_font(fs_test_title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(fs_test_title, gpu_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    /* FS info display */
    fs_info_label = lv_label_create(cont);
    lv_label_set_text(fs_info_label, "FS: Loading...");
    lv_obj_set_style_text_font(fs_info_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(fs_info_label, fs_test_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* FS test button */
    fs_test_btn = common_text_button(cont, "Start FS Test", get_system_font_size(0), 366, 100, fs_test_callback);
    lv_obj_align_to(fs_test_btn, fs_info_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    // lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
}

#ifdef BSP_USING_UI_HANDLER
static void handle_imu_acc(void *param)
{
    SpaceVector state = *(SpaceVector *)param;
    char buf[32];
    snprintf(buf, sizeof(buf), "Accel: %4.2f, %4.2f, %4.2f", state.x, state.y, state.z);
    lv_label_set_text(accel_data_label, buf);
}

static void handle_imu_gyro(void *param)
{
    SpaceVector state = *(SpaceVector *)param;
    char buf[32];
    snprintf(buf, sizeof(buf), "Gyro: %4.2f, %4.2f, %4.2f", state.x, state.y, state.z);
    lv_label_set_text(gyro_data_label, buf);
}

static void handle_imu_attitude(void *param)
{
    SpaceVector state = *(SpaceVector *)param;
    char buf[32];
    snprintf(buf, sizeof(buf), "Attitude: %4.2f, %4.2f, %4.2f", state.x, state.y, state.z);
    lv_label_set_text(attitude_data_label, buf);
}

static void handle_heart_rate(int hr)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "HR: %d bpm", hr);
    lv_label_set_text(hr_data_label, buf);
}

static void handle_battery_voltage(void *param)
{
    uint16_t voltage = *(uint16_t *)param;
    char buf[32];
    snprintf(buf, sizeof(buf), " %d mV", voltage);
    lv_label_set_text(battery_voltage_label, buf);
}

static void handle_battery_percentage(uint8_t percentage)
{
    char buf[32];
    snprintf(buf, sizeof(buf), " %d%%", percentage);
    lv_label_set_text(battery_percentage_label, buf);
}
#endif

static int32_t wheel_handler(int16_t diff, lv_indev_state_t event, void *user_data)
{
    LOG_D("wheel_handler: diff = %d, event = %d", diff, event);
    if (diff != 0)
    {
        lv_slider_set_value(slider, lv_slider_get_value(slider) + diff, LV_ANIM_ON);
    }
    return 0;
}

static void on_start(void)
{
    lv_create_dev_screen();
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_imu_acc = handle_imu_acc;
    lvgl_msg_handler.handle_imu_gyro = handle_imu_gyro;
    lvgl_msg_handler.handle_imu_attitude = handle_imu_attitude;
    lvgl_msg_handler.handle_hr = handle_heart_rate;
    lvgl_msg_handler.handle_battery_voltage = handle_battery_voltage;
    lvgl_msg_handler.handle_battery_percentage = handle_battery_percentage;
#endif

    // Start FS info update timer
    fs_update_timer = lv_timer_create(fs_update_info_cb, 1000, NULL);
    fs_update_info_cb(NULL); // Initial update

    // wheel_default_handler_register(wheel_handler, NULL);
}

static void on_resume(void)
{
}

static void on_pause(void)
{
}

static void on_stop(void)
{
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_imu_acc = NULL;
    lvgl_msg_handler.handle_imu_gyro = NULL;
    lvgl_msg_handler.handle_imu_attitude = NULL;
    lvgl_msg_handler.handle_hr = NULL;
    lvgl_msg_handler.handle_battery_voltage = NULL;
    lvgl_msg_handler.handle_battery_percentage = NULL;
#endif

    // Stop FS test and timer
    fs_test_running = false;
    if (fs_update_timer)
    {
        lv_timer_del(fs_update_timer);
        fs_update_timer = NULL;
    }

    // Stop battery request timer
    if (battery_request_timer)
    {
        lv_timer_del(battery_request_timer);
        battery_request_timer = NULL;
    }
    battery_request_enabled = false;

    // wheel_default_handler_register(NULL, NULL);
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

void app_developer_main(void)
{
    gui_app_create_page("developer", msg_handler);
}

static void test_all_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == lv_event_get_code(e))
    {
        control_provider.notify_unit_test_action(1);
    }
}

static void restart_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == lv_event_get_code(e))
    {
        watch_system_interact(WATCH_REBOOT, NULL);
    }
}

static void reset_restart_num_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == lv_event_get_code(e))
    {
        extern void reset_watch_restart_number(void);
        reset_watch_restart_number();

        // 更新按鈕文字
        if (reset_restart_num_label != NULL)
        {
            char restart_num_text[32];
            snprintf(restart_num_text, sizeof(restart_num_text), "Reset Restart Num (%d)", SkaiWatchSys.watch_restart_num);
            lv_label_set_text(reset_restart_num_label, restart_num_text);
        }
    }
}

extern void open_test_mode(bool open);
static void ble_log_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
    {
        SkaiWatchSys.flag_field.debug_mode = 1;
        open_test_mode(true);
    }
    else
    {
        SkaiWatchSys.flag_field.debug_mode = 0;
        open_test_mode(false);
    }
}

static void file_log_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    int enable = (lv_obj_get_state(obj) & LV_STATE_CHECKED) ? 1 : 0;
    int ret = log_file_backend_set_enabled(enable);
    if (ret != RT_EOK)
    {
        /* Revert UI state on failure so the switch reflects reality. */
        if (enable)
        {
            lv_obj_clear_state(obj, LV_STATE_CHECKED);
        }
        else
        {
            lv_obj_add_state(obj, LV_STATE_CHECKED);
        }
        LOG_E("File log toggle failed (ret=%d)", ret);
    }
    else
    {
        LOG_I("File log %s", enable ? "enabled" : "disabled");
    }
}

// File System Test Functions
static void fs_update_info_cb(lv_timer_t *timer)
{
    struct statfs fs_stat;
    char info_buf[128];
    if (statfs("/", &fs_stat) == 0)
    {
        uint64_t total_size = (uint64_t)fs_stat.f_blocks * fs_stat.f_bsize;
        uint64_t free_size = (uint64_t)fs_stat.f_bfree * fs_stat.f_bsize;
        uint64_t used_size = total_size - free_size;

        snprintf(info_buf, sizeof(info_buf),
                 "FS: %llu/%llu KB\nFiles: %d",
                 used_size / 1024, total_size / 1024, fs_test_file_count);
    }
    else
    {
        snprintf(info_buf, sizeof(info_buf), "FS: Error reading stats");
    }

    lv_label_set_text(fs_info_label, info_buf);
}

static void fs_create_test_file(int index)
{
    char filename[64];
    char data[256];
    int fd;

    snprintf(filename, sizeof(filename), "/test_file_%04d.txt", index);
    snprintf(data, sizeof(data), "Test file %d - %s", index, "This is test data for file system testing. ");

    // Expand data to make file larger
    for (int i = 0; i < 5; i++)
    {
        strcat(data, "More test data. ");
    }

    fd = open(filename, O_CREAT | O_WRONLY | O_TRUNC, 0666);
    if (fd >= 0)
    {
        write(fd, data, strlen(data));
        close(fd);
        fs_test_file_count++;
        LOG_I("Created file %s (%d bytes)", filename, (int)strlen(data));
    }
    else
    {
        LOG_E("Failed to create file %s: %s", filename, strerror(errno));
    }
}

static void fs_delete_all_test_files(void)
{
    char filename[64];

    for (int i = 0; i < fs_test_file_count; i++)
    {
        snprintf(filename, sizeof(filename), "/test_file_%04d.txt", i);
        unlink(filename);
    }
    fs_test_file_count = 0;
}

static void fs_test_task(void *parameter)
{
    int max_files = 1000; // Maximum files to try creating
    struct statfs fs_stat;

    LOG_I("FS Test: Starting file creation test");

    // Phase 1: Create files until disk is full
    for (int i = 0; i < max_files; i++)
    {
        if (!fs_test_running)
            break;

        fs_create_test_file(i);

        // Check disk space every 10 files
        if (i % 10 == 0)
        {
            if (statfs("/", &fs_stat) == 0)
            {
                uint64_t free_size = (uint64_t)fs_stat.f_bfree * fs_stat.f_bsize;
                if (free_size < 10240) // Less than 10KB free
                {
                    LOG_I("FS Test: Disk nearly full, stopping creation");
                    break;
                }
            }
        }

        rt_thread_mdelay(50); // Small delay
    }

    LOG_I("FS Test: Created %d files", fs_test_file_count);
    rt_thread_mdelay(2000); // Wait 2 seconds

    // Phase 2: Delete all test files
    if (fs_test_running)
    {
        LOG_I("FS Test: Starting file deletion");
        fs_delete_all_test_files();
        LOG_I("FS Test: All test files deleted");
    }

    fs_test_running = false;
    lv_label_set_text(lv_obj_get_child(fs_test_btn, 0), "Start FS Test");
}

static void fs_test_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == lv_event_get_code(e))
    {
        if (!fs_test_running)
        {
            fs_test_running = true;
            lv_label_set_text(lv_obj_get_child(obj, 0), "Stop FS Test");

            // Create test thread
            rt_thread_t thread = rt_thread_create("fs_test",
                                                  fs_test_task,
                                                  NULL,
                                                  4096,
                                                  RT_THREAD_PRIORITY_MAX - 2,
                                                  20);
            if (thread != RT_NULL)
            {
                rt_thread_startup(thread);
            }
        }
        else
        {
            fs_test_running = false;
            lv_label_set_text(lv_obj_get_child(obj, 0), "Start FS Test");
        }
    }
}

// Add these new functions
static void accel_sub_unsub_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
    {
        peripheral_provider.subscribe_accelerometer_sensor(true);
        LOG_D("Accelerometer subscribed");
    }
    else
    {
        peripheral_provider.subscribe_accelerometer_sensor(false);
        LOG_D("Accelerometer unsubscribed");
    }
}

static void hr_sub_unsub_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
    {
        peripheral_provider.subscribe_ppg_signal(true);
    }
    else
    {
        peripheral_provider.subscribe_ppg_signal(false);
    }
}

static void ppg_data_collection_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
    {
        ppg_data_collection = true;
    }
    else
    {
        ppg_data_collection = false;
    }
}

static void tap_and_hold_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    extern void set_enable_tap_and_hold(bool enable);
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
    {
        set_enable_tap_and_hold(true);
        LOG_I("Tap and Hold mode enabled");
        watch_sys_sync.set_tap_and_hold_mode(true);
    }
    else
    {
        set_enable_tap_and_hold(false);
        LOG_I("Tap and Hold mode disabled");
        watch_sys_sync.set_tap_and_hold_mode(false);
    }
}

static void motor_switch_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
    {
        set_motor_switch_state(1);
    }
    else
    {
        set_motor_switch_state(0);
    }
}

static void fps_cpu_load_switch_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_VALUE_CHANGED == event)
    {
        // switch display fps and cpu load
        set_display_fps_and_cpu_load(lv_obj_get_state(obj) & LV_STATE_CHECKED);

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
            lv_obj_add_state(obj, LV_STATE_PRESSED);
    }
}

static void EPIC_switch_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_VALUE_CHANGED == event)
    {
        // switch object refresh mask
        lv_gpu_set_enable(lv_obj_get_state(obj) & LV_STATE_CHECKED);

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
            lv_obj_add_state(obj, LV_STATE_PRESSED);
    }
}

#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/