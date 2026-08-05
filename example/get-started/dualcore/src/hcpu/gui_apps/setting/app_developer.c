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

/* For FS_REGION_SIZE: lets the FS test compare the mounted FAT size against
   the flash region the partition table provisions, so we can tell whether the
   filesystem actually spans the whole partition. Not present on the PC sim. */
#if !defined(WIN32) && !defined(BSP_USING_PC_SIMULATOR)
#include "board.h"
#endif

#define DBG_TAG "app.developer"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Referenced by bloc_peripheral PPG thread — kept outside the wrap. */
bool ppg_data_collection = false;

/* 2026-07-29 founder: ship the Development page in release too, so the restart
 * count (watch_restart_num) + File-log toggle can be checked on-watch without
 * a dev build. Was gated on !kReleaseMode. */
#if 1

/* The pair lives in lv_drivers/lv_gpu_new_api.c, which is v8-only -- there is
   no v9 equivalent. Same stub the PC build already used. */
#if defined(WIN32) || !defined(DISABLE_LVGL_V9)
void lv_gpu_set_enable(bool en) { LV_UNUSED(en); };
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
static uint32_t fs_test_total_kb = 0;   /* total KB written this run (shown on the dev page) */
static int fs_test_last_err = 0;        /* errno of last create/write failure, 0 = none */
static char fs_test_buf[1024];          /* reused write/verify buffer (kept off the test thread stack) */
static lv_obj_t *fs_info_label;
static lv_obj_t *fs_test_btn;
static lv_obj_t *fs_clean_btn;
static lv_timer_t *fs_update_timer;

// Add these function prototypes
static void test_all_callback(lv_event_t *e);
static void restart_callback(lv_event_t *e);
static void reset_restart_num_callback(lv_event_t *e);
static void ble_log_sw_event_callback(lv_event_t *e);
static void file_log_sw_event_callback(lv_event_t *e);
static void accel_sub_unsub_sw_event_callback(lv_event_t *e);
static void hr_sub_unsub_sw_event_callback(lv_event_t *e);
extern void imu_raw_data_collection_sw_event_callback(lv_event_t *e);
static void ppg_data_collection_sw_event_callback(lv_event_t *e);
static void tap_and_hold_sw_event_callback(lv_event_t *e);
static void motor_switch_event_callback(lv_event_t *e);
static void fps_cpu_load_switch_event_callback(lv_event_t *e);
static void EPIC_switch_event_callback(lv_event_t *e);
static void fs_test_callback(lv_event_t *e);
static void fs_clean_callback(lv_event_t *e);
static void fs_update_info_cb(lv_timer_t *timer);
extern void imu_lock_sw_event_callback(lv_event_t *e);

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
static lv_timer_t *battery_request_timer = NULL;
static bool battery_request_enabled = false;
static lv_obj_t *reset_restart_num_btn = NULL;
static lv_obj_t *reset_restart_num_label = NULL;

/* Gesture peak-confirm control (see the slider in the layout below). */
extern uint16_t gesture_confirm_get(void);
extern void gesture_confirm_set(uint16_t peak_x100);
static lv_obj_t *gcap_confirm_slider = NULL;
static lv_obj_t *gcap_confirm_label = NULL;
static char gcap_confirm_text[48];

/* Slider spans 0..GCAP_CONFIRM_MAX in GCAP_CONFIRM_STEP increments (both are
   peak ×100). Upper bound 1.50: real gestures in the walking capture started
   at 0.83 and noise reached 1.33, so everything useful sits well below that.
   0 = off, which is also the ship default. */
#define GCAP_CONFIRM_MAX 150
#define GCAP_CONFIRM_STEP 5

static void gcap_confirm_refresh_label(uint16_t v)
{
    if (v == 0)
    {
        snprintf(gcap_confirm_text, sizeof(gcap_confirm_text),
                 "Gesture confirm: OFF");
    }
    else
    {
        snprintf(gcap_confirm_text, sizeof(gcap_confirm_text),
                 "Gesture confirm: %d.%02d", v / 100, v % 100);
    }
    if (gcap_confirm_label)
    {
        lv_label_set_text(gcap_confirm_label, gcap_confirm_text);
    }
}

static void gcap_confirm_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    int32_t raw = lv_slider_get_value(obj);
    /* Quantise to GCAP_CONFIRM_STEP — dragging on a watch-sized screen can't
       resolve single units, and the stored value is what the capture path
       compares against. */
    uint16_t v = (uint16_t)(((raw + GCAP_CONFIRM_STEP / 2) / GCAP_CONFIRM_STEP) *
                            GCAP_CONFIRM_STEP);
    gesture_confirm_set(v);
    gcap_confirm_refresh_label(v);
    LOG_I("Gesture confirm -> %d", v);
}
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
        watch_sys_sync.request_battery_voltage();
    }
}

/**
 * @brief Timer callback to request battery voltage every second
 */
static void battery_request_timer_cb(lv_timer_t *timer)
{
    watch_sys_sync.request_battery_voltage();
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
        watch_sys_sync.request_charge_status();
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
        peripheral_provider.hcpu_reboot();
    }
}

/* Real definition lives in hr_client.c, but it is not linked in every config
 * (e.g. this release build). A weak fallback keeps the Development page
 * linkable everywhere now that the page ships in release too — hr_client.c's
 * strong definition wins whenever present. Only presets the PPG toggle state. */
bool ppg_service_subscribed(void);

/* MSVC has no weak symbols — RT_WEAK is empty there (rtdef.h) — so on the PC
 * simulator this fallback collides with hr_client.c's strong definition
 * instead of losing to it. Skip it where hr_client.c is actually linked. */
#if !defined(_MSC_VER) || !defined(BSP_USING_HR_CLIENT)
RT_WEAK bool ppg_service_subscribed(void)
{
    return false;
}
#endif

static void lv_create_dev_screen(void)
{
    // header
    /* lv_lvsfheader_* comes from the closed gui_widgets library, which has no
       v9 build. This is the same thing out of plain objects: a full-width strip
       holding a back arrow and a centred title. */
    lv_obj_t *title = lv_obj_create(lv_screen_active());
    lv_obj_set_size(title, LV_HOR_RES_MAX, 56);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(title, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(title, 0, 0);
    lv_obj_set_style_pad_all(title, 0, 0);
    lv_obj_remove_flag(title, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *back_btn = lv_button_create(title);
    lv_obj_set_size(back_btn, 48, 48);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_set_style_bg_opa(back_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_shadow_width(back_btn, 0, 0);
    lv_obj_add_event_cb(back_btn, back_btn_event_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_center(back_lbl);

    lv_obj_t *title_lbl = lv_label_create(title);
    lv_label_set_text(title_lbl, "Developer App");
    lv_obj_center(title_lbl);

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
    fs_test_btn = common_text_button(cont, "FS Fill Test", get_system_font_size(0), 366, 100, fs_test_callback);
    lv_obj_align_to(fs_test_btn, fs_info_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    fs_clean_btn = common_text_button(cont, "FS Cleanup", get_system_font_size(0), 366, 100, fs_clean_callback);
    lv_obj_align_to(fs_clean_btn, fs_test_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* Gesture peak-confirm (walking only). On-watch control on purpose: the
       MSH route needs a UART cable, which is exactly what a walking test
       can't have. */
    lv_obj_t *gcap_confirm_container = lv_obj_create(cont);
    lv_obj_set_size(gcap_confirm_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(gcap_confirm_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(gcap_confirm_container, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    /* Don't let this container scroll or chain a drag up to the page: dragging
       the slider must move the knob, not pan the page sideways. */
    lv_obj_clear_flag(gcap_confirm_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(gcap_confirm_container, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_align_to(gcap_confirm_container, fs_clean_btn,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    gcap_confirm_label = lv_label_create(gcap_confirm_container);
    lv_obj_set_style_text_font(gcap_confirm_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    gcap_confirm_slider = lv_slider_create(gcap_confirm_container);
    lv_slider_set_range(gcap_confirm_slider, 0, GCAP_CONFIRM_MAX);
    lv_slider_set_value(gcap_confirm_slider, gesture_confirm_get(), LV_ANIM_OFF);
    /* 220 wide, not 300: the container adds its own padding on both sides and
       the screen is 466 round, so a 300 slider pushed the page wide enough to
       scroll horizontally — and a page that pans sideways steals the drag. */
    lv_obj_set_size(gcap_confirm_slider, 220, 40);
    lv_obj_clear_flag(gcap_confirm_slider, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_add_event_cb(gcap_confirm_slider, gcap_confirm_callback,
                        LV_EVENT_VALUE_CHANGED, NULL);
    /* Label text is only produced here, so it starts in sync with the slider. */
    gcap_confirm_refresh_label(gesture_confirm_get());

    /* Tail spacer: this is the last item on a ROUND screen, so without extra
       room below it the slider can only ever sit in the bottom arc where the
       display is narrowest and the knob is hardest to grab. The spacer lets it
       be scrolled up to the middle of the screen. */
    lv_obj_t *gcap_tail_spacer = lv_obj_create(cont);
    lv_obj_set_size(gcap_tail_spacer, LV_HOR_RES / 2, 140);
    lv_obj_set_style_bg_opa(gcap_tail_spacer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(gcap_tail_spacer, 0, 0);
    lv_obj_clear_flag(gcap_tail_spacer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(gcap_tail_spacer, gcap_confirm_container,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

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
        peripheral_provider.hcpu_reboot();
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
    char info_buf[224];

    if (statfs("/", &fs_stat) != 0)
    {
        lv_label_set_text(fs_info_label, "FS: Error reading stats");
        return;
    }

    uint64_t total = (uint64_t)fs_stat.f_blocks * fs_stat.f_bsize;
    uint64_t freeb = (uint64_t)fs_stat.f_bfree * fs_stat.f_bsize;
    uint64_t used  = total - freeb;

/* whole.tenths MB without floating point (printf %f is costly/absent here) */
#define DEV_MB_INT(b)  ((unsigned long)((b) / (1024UL * 1024UL)))
#define DEV_MB_FRAC(b) ((unsigned long)(((b) % (1024UL * 1024UL)) * 10UL / (1024UL * 1024UL)))

    int n = snprintf(info_buf, sizeof(info_buf),
                     "/ total %lu.%lu MB\nused %lu.%lu  free %lu.%lu MB\nfiles %d",
                     DEV_MB_INT(total), DEV_MB_FRAC(total),
                     DEV_MB_INT(used),  DEV_MB_FRAC(used),
                     DEV_MB_INT(freeb), DEV_MB_FRAC(freeb),
                     fs_test_file_count);
    if (n < 0 || n >= (int)sizeof(info_buf))
        n = 0;

    /* Append capacity-test progress/result so the dev page shows it live
       (the UART console may be quiet). err 0 = pass, -2 = integrity fail. */
    if (fs_test_total_kb || fs_test_last_err)
    {
        int m = snprintf(info_buf + n, sizeof(info_buf) - (size_t)n,
                         "  wrote %lu.%lu MB err %d",
                         (unsigned long)(fs_test_total_kb / 1024),
                         (unsigned long)((fs_test_total_kb % 1024) * 10 / 1024),
                         fs_test_last_err);
        if (m > 0 && n + m < (int)sizeof(info_buf))
            n += m;
    }

#ifdef FS_REGION_SIZE
    /* Compare the mounted FAT against the region the partition table reserves.
       If the FAT spans most of the region it grew correctly; if it's far
       smaller the partition never expanded (the bug this verifies the fix for). */
    uint64_t region = (uint64_t)FS_REGION_SIZE;
    /* GROWN if the FAT spans over half the partition. The "undersized" bug was
       5.8 MB (6.6%); a fully-grown volume is ~65 MB (~75%, capped by dhara's GC
       reserve + FAT overhead), so 80% was unreachable and always read UNDERSIZED. */
    const char *verdict = (total >= region / 2) ? "GROWN" : "UNDERSIZED!";
    snprintf(info_buf + n, sizeof(info_buf) - (size_t)n,
             "\nregion %lu.%lu MB %s",
             DEV_MB_INT(region), DEV_MB_FRAC(region), verdict);
#endif

#undef DEV_MB_INT
#undef DEV_MB_FRAC

    lv_label_set_text(fs_info_label, info_buf);
}

#define FSTEST_DIR       "/FSTEST"
#define FSTEST_FILE_SIZE (64 * 1024)   /* bytes per file; ~1000 files fill the 65 MB volume */

/* Create one FSTEST_FILE_SIZE file filled with a per-file verifiable pattern,
   inside a subdirectory. NOT in root: a FAT16 root dir has a fixed 512-entry
   cap, which the old root-based test exhausted at ~169 LFN files regardless of
   free space. Returns bytes written, or 0 on failure (errno in fs_test_last_err). */
static int fs_create_test_file(int index)
{
    char filename[64];
    int fd, total = 0, n, i;

    snprintf(filename, sizeof(filename), FSTEST_DIR "/F%04d.DAT", index);
    fd = open(filename, O_CREAT | O_WRONLY | O_TRUNC, 0666);
    if (fd < 0)
    {
        fs_test_last_err = errno;
        LOG_E("FS Test: create %s failed: errno=%d (%s)", filename, errno, strerror(errno));
        return 0;
    }

    for (i = 0; i < (int)sizeof(fs_test_buf); i++)
        fs_test_buf[i] = (char)(index * 31 + i);

    while (total < FSTEST_FILE_SIZE)
    {
        n = write(fd, fs_test_buf, sizeof(fs_test_buf));
        if (n != (int)sizeof(fs_test_buf))
        {
            fs_test_last_err = errno ? errno : -1;   /* short write == disk full */
            LOG_E("FS Test: write %s @%d failed: ret=%d errno=%d (%s)",
                  filename, total, n, errno, strerror(errno));
            break;
        }
        total += n;
    }
    close(fd);

    if (total < FSTEST_FILE_SIZE)
    {
        unlink(filename);   /* drop the partial file so the volume is left clean */
        return 0;
    }
    return total;
}

/* Delete everything in FSTEST_DIR by enumerating the directory, not by walking
   fs_test_file_count. After a crash/reboot the in-memory count is 0 while the
   files are still on disk, so the old count-based loop deleted nothing
   ("cleanup deleting 0 files") and left the volume full; a cleanup that itself
   crashed partway could also leave a non-contiguous set an index walk would
   miss. readdir sees whatever is actually there. The F%04d.DAT names are 8.3
   (single dir slot, no LFN), so unlinking during the readdir walk is safe on
   FatFs. Yields periodically so a large delete on this high-priority thread
   does not starve the watchdog. Returns the number of files deleted. */
static int fs_delete_all_test_files(void)
{
    char filename[64];
    struct dirent *entry;
    int deleted = 0;
    DIR *dir = opendir(FSTEST_DIR);

    if (dir != NULL)
    {
        while ((entry = readdir(dir)) != NULL)
        {
            if (entry->d_type != DT_REG)
                continue;
            snprintf(filename, sizeof(filename), FSTEST_DIR "/%s", entry->d_name);
            if (unlink(filename) == 0)
                deleted++;
            if (deleted && (deleted & 0x1F) == 0)
                rt_thread_mdelay(1);   /* yield ~every 32 deletes; avoid WDT */
        }
        closedir(dir);
    }
    rmdir(FSTEST_DIR);
    fs_test_file_count = 0;
    return deleted;
}

/* Capacity + integrity test:
     1. fill the volume with 64 KB files in a subdir until a write hits ENOSPC,
     2. read file 0 back and verify its byte pattern (catch silent corruption),
     3. delete everything and confirm free space returns.
   Progress/results are published via globals and rendered by fs_update_info_cb
   (runs in the LVGL timer context) so the on-screen dev page shows them even
   when the UART console is quiet. */
static void fs_test_task(void *parameter)
{
    const int max_files = 2000;          /* safety cap; the loop normally ends on ENOSPC */
    struct statfs fs_stat;
    uint64_t free_before = 0, free_after = 0;
    int i, integrity_ok = 0, fd;

    fs_test_file_count = 0;
    fs_test_total_kb = 0;
    fs_test_last_err = 0;

    LOG_I("FS Test: start (%dKB files into %s)", FSTEST_FILE_SIZE / 1024, FSTEST_DIR);

    mkdir(FSTEST_DIR, 0777);              /* harmless if it already exists */
    if (statfs("/", &fs_stat) == 0)
        free_before = (uint64_t)fs_stat.f_bfree * fs_stat.f_bsize;

    /* Phase 1: fill with real data until a write fails (disk full). */
    for (i = 0; i < max_files; i++)
    {
        int bytes;
        if (!fs_test_running)
            break;
        bytes = fs_create_test_file(i);
        if (bytes <= 0)
            break;                        /* ENOSPC or error -> done filling */
        fs_test_file_count++;
        fs_test_total_kb += (uint32_t)(bytes / 1024);
        rt_thread_mdelay(5);
    }

    if (statfs("/", &fs_stat) == 0)
        free_after = (uint64_t)fs_stat.f_bfree * fs_stat.f_bsize;

    LOG_I("FS Test: wrote %d files / %lu KB; free %lu KB -> %lu KB; last_err=%d",
          fs_test_file_count, (unsigned long)fs_test_total_kb,
          (unsigned long)(free_before / 1024), (unsigned long)(free_after / 1024),
          fs_test_last_err);

    /* Phase 2: read file 0 back and verify the per-file pattern. */
    if (fs_test_running && fs_test_file_count > 0)
    {
        char filename[64];
        static char vbuf[1024];
        int n, off = 0, bad = 0;

        snprintf(filename, sizeof(filename), FSTEST_DIR "/F%04d.DAT", 0);
        fd = open(filename, O_RDONLY);
        if (fd >= 0)
        {
            while ((n = read(fd, vbuf, sizeof(vbuf))) > 0)
            {
                for (i = 0; i < n; i++)
                    if (vbuf[i] != (char)((off + i) % (int)sizeof(fs_test_buf))) { bad = 1; break; }
                if (bad) break;
                off += n;
            }
            close(fd);
            integrity_ok = (!bad && off == FSTEST_FILE_SIZE);
        }
        LOG_I("FS Test: integrity %s (verified %d bytes of %s)",
              integrity_ok ? "OK" : "FAIL", off, filename);
        if (!integrity_ok && !fs_test_last_err)
            fs_test_last_err = -2;        /* surface a non-zero "integrity failed" code */
    }

    LOG_I("FS Test: fill done, %d files (%lu KB) kept in %s; press Cleanup to delete",
          fs_test_file_count, (unsigned long)fs_test_total_kb, FSTEST_DIR);

    fs_test_running = false;
    lv_label_set_text(lv_obj_get_child(fs_test_btn, 0), "FS Fill Test");
}

static void fs_test_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == lv_event_get_code(e))
    {
        if (!fs_test_running)
        {
            fs_test_running = true;
            lv_label_set_text(lv_obj_get_child(obj, 0), "Stop Fill");

            // Create test thread
            rt_thread_t thread = rt_thread_create("fs_test",
                                                  fs_test_task,
                                                  NULL,
                                                  8192,
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
            lv_label_set_text(lv_obj_get_child(obj, 0), "FS Fill Test");
        }
    }
}

static void fs_clean_task(void *parameter)
{
    struct statfs fs_stat;
    int deleted;

    LOG_I("FS Test: cleanup scanning %s", FSTEST_DIR);
    deleted = fs_delete_all_test_files();   /* enumerates dir + rmdir, reboot-safe, yields */
    fs_test_total_kb = 0;
    fs_test_last_err = 0;
    if (statfs("/", &fs_stat) == 0)
        LOG_I("FS Test: cleanup done, deleted %d files, free now %lu KB",
              deleted, (unsigned long)((uint64_t)fs_stat.f_bfree * fs_stat.f_bsize / 1024));

    fs_test_running = false;
    lv_label_set_text(lv_obj_get_child(fs_clean_btn, 0), "FS Cleanup");
}

static void fs_clean_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == lv_event_get_code(e))
    {
        if (fs_test_running)
            return;                       /* a fill or cleanup is already in progress */

        fs_test_running = true;
        lv_label_set_text(lv_obj_get_child(obj, 0), "Cleaning...");

        rt_thread_t thread = rt_thread_create("fs_clean",
                                              fs_clean_task,
                                              NULL,
                                              8192,
                                              RT_THREAD_PRIORITY_MAX - 2,
                                              20);
        if (thread != RT_NULL)
            rt_thread_startup(thread);
        else
        {
            fs_test_running = false;
            lv_label_set_text(lv_obj_get_child(obj, 0), "FS Cleanup");
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