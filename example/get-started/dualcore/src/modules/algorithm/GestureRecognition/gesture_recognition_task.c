/**
 ******************************************************************************
 * @file   gesture_recognition_task.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
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

#include <rtthread.h>
#include <board.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Project headers */
#include "gesture_recognition_task.h"
#include "ui_helper.h"
#include "ui_handler.h"
#include "drv_touch.h"
#include "gui_app_fwk.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "bf0_hal_fft.h"
#include "real_fft.h"

/* Conditional includes */
#include "watch_sys_service.h"
#include "communicate_protocol.h"
#include "communicate_update_image.h"
#include "main_functions.h"
#include "constants.h"
#include "gesture_handler.h"
#include "watch_system_interact.h"
#include "watch_system_core_task.h"
#include "bloc_peripheral.h"
#include "bloc_setting.h"
#include "bloc_notification.h"
#include "bloc_control.h"
#include "bloc_v2t.h"
#include "bloc_motion_tracking.h"
#include "watch_global_data.h"

#define DBG_TAG "APP.GESTURE"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

/* External function from app_gesture.c for data collection */
extern void app_gesture_receive_imu_data(int16_t (*dataset)[6], int sample_num);

static rt_thread_t gesture_recognition_thread = RT_NULL;
static rt_thread_t send_gesture_data_thread = RT_NULL;

/* Buffers for sensor data processing */
static float identifyWindow[TAP_TARGET_SAMPLE_NUM][3];
static float release_identifyWindow[RELEASE_TARGET_SAMPLE_NUM][3];
static float ppgidentifyWindow[16][kChannelReleaseNumber];

#if USE_FFT_FILTER
volatile static uint8_t fft_done_flag;
static int32_t fft_in_buffer[32];
static int32_t fft_out_buffer[32];
extern fft_env_t g_fft_env;
/**
 * @brief FFT completion callback function
 *
 * @param fft Pointer to the FFT handle
 */
static void fft_done(FFT_HandleTypeDef *fft)
{
    fft_done_flag = 1;
}

/**
 * @brief Initialize FFT hardware for gesture recognition
 */
void init_fft(void)
{ // Initialize driver and enable FFT IRQ
    LOG_D("init_fft");
    HAL_NVIC_SetPriority(FFT1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(FFT1_IRQn);

    HAL_RCC_EnableModule(RCC_MOD_FFT1);

    g_fft_env.fft_handle.Instance = hwp_fft1;
    HAL_FFT_Init(&g_fft_env.fft_handle);
}

void deinit_fft(void)
{
    HAL_FFT_DeInit(&g_fft_env.fft_handle);
    g_fft_env.fft_handle.Instance = NULL;

    HAL_RCC_DisableModule(RCC_MOD_FFT1);
    HAL_NVIC_DisableIRQ(FFT1_IRQn);
    LOG_D("deinit_fft");
}

// 計算標準差
float calc_std(const float *data, int len)
{
    float mean = 0.0f, sum = 0.0f;
    for (int i = 0; i < len; ++i)
        mean += data[i];
    mean /= len;
    for (int i = 0; i < len; ++i)
        sum += (data[i] - mean) * (data[i] - mean);
    return sqrtf(sum / len);
}

bool check_linear_acce_condition_with_fft(void)
{
    FFT_ConfigTypeDef config;
    HAL_StatusTypeDef res;

    /* 初始化 */
    memset(&config, 0, sizeof(config));

    config.bitwidth = FFT_BW_32BIT; // 16bit FFT: input format is Q1.15
    config.fft_length = FFT_LEN_32; // output format is Q5.11
    config.ifft_flag = 0;
    config.rfft_flag = 0;
    config.input_data = fft_in_buffer;
    config.output_data = fft_out_buffer;

    fft_done_flag = 0;
    g_fft_env.fft_handle.CpltCallback = fft_done;
    res = HAL_FFT_StartFFT_IT(&g_fft_env.fft_handle, &config);

    /* wait for interrupt, fft_done_flag is changed to 1 in fft_done */
    int timeout = 10; // 設定最大等待次數，依實際情況調整
    while (0 == fft_done_flag && timeout-- > 0)
    {
        rt_thread_mdelay(1); // 每次延遲1ms
    }
    if (timeout <= 0)
    {
        LOG_E("FFT timeout");
        return false; // 超時，返回錯誤
    }

    float fft_data[32];
    for (int i = 0; i < 32; i++)
    {
        fft_data[i] = fft_out_buffer[i] / 2048.0f;
    }

    float std = calc_std(fft_data, 32);
    // LOG_I("FFT std=%0.5f", std);
    if (std > 0.06f) // 0.0875
    {
        return false;
    }

    // ui_show_hint_toast("[o] FFT std=%0.5f", std);
    return true;
}
#endif

/**
 * @brief Handle detected gesture by updating gesture index
 *
 * @param label The gesture label detected by the recognition algorithm
 */
static void handle_gesture(int label)
{
    if (label >= 0 && label < kGestureCount)
    {
        setGestureIndex(label);
    }
}

static void get_gesture_data(gesture_data_t *gesture, int sample_num,
                             uint8_t divide_rate)
{
    if (sample_num > TAP_TARGET_SAMPLE_NUM)
    {
        for (int i = 0; i < sample_num; i++)
        {
            int index = i * divide_rate;
            float acceleration_x =
                gesture->dataset[index].x / INT16_to_G * GRAVITY;
            float acceleration_y =
                gesture->dataset[index].y / INT16_to_G * GRAVITY;
            float acceleration_z =
                gesture->dataset[index].z / INT16_to_G * GRAVITY;
            release_identifyWindow[i][0] = acceleration_x;
            release_identifyWindow[i][1] = acceleration_y;
            release_identifyWindow[i][2] = acceleration_z;

#if USE_FFT_FILTER
            // int16_t total_acc = sqrt(dataset->dataset[i][0] *
            // dataset->dataset[i][0] +
            //                          dataset->dataset[i][1] *
            //                          dataset->dataset[i][1] +
            //                          dataset->dataset[i][2] *
            //                          dataset->dataset[i][2]);
            if (i < 32)
            {
                // fft_in_buffer[i] = total_acc;
                fft_in_buffer[i] = dataset->dataset[i][2];
            }
#endif
        }
    }
    else
    {
        for (int i = 0; i < sample_num; i++)
        {
            int index = i * divide_rate;
            float acceleration_x =
                gesture->dataset[index].x / INT16_to_G * GRAVITY;
            float acceleration_y =
                gesture->dataset[index].y / INT16_to_G * GRAVITY;
            float acceleration_z =
                gesture->dataset[index].z / INT16_to_G * GRAVITY;
            identifyWindow[i][0] = acceleration_x;
            identifyWindow[i][1] = acceleration_y;
            identifyWindow[i][2] = acceleration_z;
#if USE_FFT_FILTER
            // int16_t total_acc = sqrt(gesture->dataset[i][0] *
            // gesture->dataset[i][0] +
            //                          gesture->dataset[i][1] *
            //                          gesture->dataset[i][1] +
            //                          gesture->dataset[i][2] *
            //                          gesture->dataset[i][2]);
            if (i < 32)
            {
                // fft_in_buffer[i] = total_acc;
                fft_in_buffer[i] = gesture->dataset[i][2];
            }
#endif
        }
    }
}

static void get_gesture_data_with_ppg(gesture_data_t *dataset, int sample_num,
                                      uint8_t divide_rate)
{
    for (int i = 0; i < sample_num; i++)
    {
#if kChannelReleaseNumber == 4
        if (i == 0)
        {
            continue;
        }
        ppgidentifyWindow[i - 1][0] =
            dataset->dataset_ppg[i].acce.x / INT16_to_G * GRAVITY;
        ppgidentifyWindow[i - 1][1] =
            dataset->dataset_ppg[i].acce.y / INT16_to_G * GRAVITY;
        ppgidentifyWindow[i - 1][2] =
            dataset->dataset_ppg[i].acce.z / INT16_to_G * GRAVITY;
        float diff_ppg =
            dataset->dataset_ppg[i].ppg - dataset->dataset_ppg[i - 1].ppg;
        ppgidentifyWindow[i - 1][3] = diff_ppg;
#else
        int index = i * divide_rate;
        ppgidentifyWindow[i][0] =
            dataset->dataset_ppg[index].acce.x / INT16_to_G * GRAVITY;
        ppgidentifyWindow[i][1] =
            dataset->dataset_ppg[index].acce.y / INT16_to_G * GRAVITY;
        ppgidentifyWindow[i][2] =
            dataset->dataset_ppg[index].acce.z / INT16_to_G * GRAVITY;
#endif
#if USE_FFT_FILTER
        // if (i <= 16)
        // {
        //     int16_t total_acc = sqrt(dataset->dataset_ppg[i].acce.x *
        //     dataset->dataset_ppg[i].acce.x +
        //                              dataset->dataset_ppg[i].acce.y *
        //                              dataset->dataset_ppg[i].acce.y +
        //                              dataset->dataset_ppg[i].acce.z *
        //                              dataset->dataset_ppg[i].acce.z);
        //     fft_buffer_imu[i - 1] = total_acc;
        // }
#endif
    }
}

static bool gesture_recognition_lock = false;
static uint8_t unknown_gesture_count = 0;
static rt_tick_t last_gesture_recognition_time = 0;
static rt_timer_t gesture_lock_timer = RT_NULL;
static void gesture_lock_timer_callback(void *parameter)
{
    gesture_recognition_lock = false;
    unknown_gesture_count = 0;
    if (gesture_lock_timer != RT_NULL)
    {
        rt_timer_delete(gesture_lock_timer);
        gesture_lock_timer = RT_NULL;
    }
}

static void trigger_gesture_unlock_timer(void)
{
    if (gesture_lock_timer == RT_NULL)
    {
        gesture_lock_timer =
            rt_timer_create("gesture_lock_timer", gesture_lock_timer_callback,
                            RT_NULL, 1000, RT_TIMER_FLAG_ONE_SHOT);
    }

    if (gesture_lock_timer != RT_NULL)
    {
        rt_timer_start(gesture_lock_timer);
    }
}

extern bool imu_data_collection;
extern bool imu_data_collection_error;
extern bool gesture_tap_collection;

int tap_recognition_score;
int release_recognition_score;
int gesture_recognition_threshold = DEFAULT_GESTURE_THRESHOLD;
void set_gesture_recognition_threshold(int threshold)
{
    if (threshold >= 50 && threshold <= 100)
    {
        gesture_recognition_threshold = threshold;
    }
}
int get_gesture_recognition_threshold(void)
{
    return gesture_recognition_threshold;
}

static void gesture_recognition_algorithm(gesture_data_t *gesture)
{
    LOG_D("gesture_recognition_algorithm sample_num:%d", gesture->sample_num);
    uint8_t sample_num = gesture->sample_num;
    // convert int16_t data to float in g unit to identifyWindow
    // if (imu_raw_data_collection)
    // {
    //     packMatrixToBuffer(gsensorSamplesBuffer, gesture->dataset, NULL,
    //                        sample_num);
    //     sensor_buf_t buffer_info = {.data = gsensorSamplesBuffer,
    //                                 .length = sample_num * BYTES_PER_SAMPLE};
    //     L1SendData data = {.event = L1SEND_LINEAR_ACCE_BUFFER,
    //                        .res.imu_data = buffer_info};
    //     L1_send_event(data);
    // }
    // else 
    if (imu_data_collection)
    {
        if (imu_data_collection_error)
        {
            // incorrect tap gesture
            if (sample_num == TAP_TARGET_SAMPLE_NUM &&
                (gesture_tap_collection || gui_app_is_actived(APP_ID_GESTURE)))
            {
                get_gesture_data(gesture, TAP_TARGET_SAMPLE_NUM, 1);
                rt_tick_t tick_time_start = rt_tick_get();
                tap_recognition_score = recognize_gesture_tap(identifyWindow);
                rt_tick_t tick_time_end = rt_tick_get();
                rt_tick_t cost_tick = tick_time_end - tick_time_start;
                LOG_D("recognize tap gesture cost_tick:%d, score:%d", cost_tick,
                      tap_recognition_score);
                if (tap_recognition_score > gesture_recognition_threshold)
                {
                    packMatrixToBuffer(gsensorSamplesBuffer, gesture->dataset,
                                       NULL, TAP_TARGET_SAMPLE_NUM);
                    sensor_buf_t buffer_info = {
                        .data = gsensorSamplesBuffer,
                        .length = TAP_TARGET_SAMPLE_NUM * BYTES_PER_SAMPLE};
                    L1SendData data = {.event = L1SEND_LINEAR_ACCE_BUFFER,
                                       .res.imu_data = buffer_info};
                    L1_send_event(data);
                }
            }
            // incorrect release gesture
            else if (sample_num == RELEASE_TARGET_SAMPLE_NUM &&
                     (!gesture_tap_collection ||
                      gui_app_is_actived(APP_ID_GESTURE)))
            {
                get_gesture_data(gesture, sample_num, 1);
                rt_tick_t tick_time_start = rt_tick_get();
                release_recognition_score =
                    recognize_gesture_release(release_identifyWindow);
                rt_tick_t tick_time_end = rt_tick_get();
                rt_tick_t cost_tick = tick_time_end - tick_time_start;
                LOG_D("recognize release gesture cost_tick:%d, score:%d",
                      cost_tick, release_recognition_score);
                if (release_recognition_score > gesture_recognition_threshold)
                {
                    packMatrixToBuffer(gsensorSamplesBuffer, gesture->dataset,
                                       NULL, sample_num);
                    sensor_buf_t buffer_info = {.data = gsensorSamplesBuffer,
                                                .length = sample_num *
                                                          BYTES_PER_SAMPLE};
                    L1SendData data = {.event = L1SEND_LINEAR_ACCE_BUFFER,
                                       .res.imu_data = buffer_info};
                    L1_send_event(data);
                }
            }
        }
        else
        {
            packMatrixToBuffer(gsensorSamplesBuffer, gesture->dataset, NULL,
                               sample_num);
            sensor_buf_t buffer_info = {.data = gsensorSamplesBuffer,
                                        .length =
                                            sample_num * BYTES_PER_SAMPLE};
            L1SendData data = {.event = L1SEND_LINEAR_ACCE_BUFFER,
                               .res.imu_data = buffer_info};
            L1_send_event(data);
            static int16_t converted_data[RELEASE_TARGET_SAMPLE_NUM][6];
            for (int i = 0; i < sample_num && i < RELEASE_TARGET_SAMPLE_NUM;
                 i++)
            {
                converted_data[i][0] = gesture->dataset[i].x;
                converted_data[i][1] = gesture->dataset[i].y;
                converted_data[i][2] = gesture->dataset[i].z;
                converted_data[i][3] = gesture->dataset[i].gravity_x;
                converted_data[i][4] = gesture->dataset[i].gravity_y;
                converted_data[i][5] = gesture->dataset[i].gravity_z;
            }
            app_gesture_receive_imu_data(converted_data, sample_num);
        }
    }
    else if (gesture_recognition_lock)
    {
        return;
    }
    else if (sample_num == TAP_TARGET_SAMPLE_NUM)
    {
        get_gesture_data(gesture, sample_num, 1);
#if USE_FFT_FILTER
        // if (check_linear_acce_condition_with_fft() || collection_mode)
#endif
        {
            int label = kNoGesture;
            rt_tick_t tick_time_start = rt_tick_get();
            tap_recognition_score = recognize_gesture_tap(identifyWindow);
            last_gesture_recognition_time = rt_tick_get();
            LOG_I("recognize tap gesture cost_tick:%d, score:%d",
                  last_gesture_recognition_time - tick_time_start,
                  tap_recognition_score);
            if (tap_recognition_score > gesture_recognition_threshold)
            {
                label = kTapGesture;
                send_virtual_gesture_event(GESTURE_EVENT_PRESS);
                extern bool get_enable_tap_and_hold(void);
                if (get_enable_tap_and_hold())
                {
                    // if (watch_sys_sync.is_ppg_enabled())
                    watch_sys_sync.notify_tap_detected();
                }
                else
                {
                    rt_thread_mdelay(100);
                    send_virtual_gesture_event(GESTURE_EVENT_TAP);
                }
            }
            else
            {
                control_provider.trigger_unknown_event();
                unknown_gesture_count++;
                if (unknown_gesture_count >= 3 &&
                    last_gesture_recognition_time - tick_time_start > 1000)
                {
                    gesture_recognition_lock = true;
                    trigger_gesture_unlock_timer();
                }
            }
            handle_gesture(label);
        }
    }
    else if (sample_num == RELEASE_TARGET_SAMPLE_NUM)
    {
        get_gesture_data(gesture, sample_num, 1);
#if USE_FFT_FILTER
        // if (check_ppg_condition_with_fft())
#endif
        {
            rt_tick_t tick_time_start = rt_tick_get();
            release_recognition_score =
                recognize_gesture_release(release_identifyWindow);
            rt_tick_t tick_time_end = rt_tick_get();
            rt_tick_t cost_tick = tick_time_end - tick_time_start;
            if (release_recognition_score > gesture_recognition_threshold)
            {
                gesture_unlock_screen_handler();
                LOG_D("recognize release gesture cost_tick:%d, score:%d",
                      cost_tick, release_recognition_score);
            }
            else
            {
                LOG_D("recognize unknown gesture cost_tick:%d, score:%d",
                      cost_tick, release_recognition_score);
            }
        }
    }
}

extern bool get_hid_mouse_handfree_mode(void);

#define IMU_THREAD_STACK_SIZE 4 * 1024
#define IMU_THREAD_PRIORITY RT_THREAD_PRIORITY_MIDDLE - 1
#define IMU_THREAD_TIMESLICE 10

static void gesture_recognition_thread_entry(void *parameter)
{
    init_gesture_recognition_model();
    init_gesture_recognition_release_model();
    
    watch_sensor.gesture_sem =
        rt_sem_create("gesture_sem", 0, RT_IPC_FLAG_FIFO);
#if USE_FFT_FILTER
    init_fft();
#endif

    static Vector3 accData, gyroData, magData;
    while (1)
    {
        rt_sem_take(watch_sensor.gesture_sem, RT_WAITING_FOREVER);

        if (SkaiWatchSys.charger_status != NoCharge)
        {
            continue;
        }
        if (is_user_touching_screen())
        {
            continue;
        }

        if (!is_at_app_list())
        {
            if (!open_gesture_model())
            {
                continue;
            }

            // 飛鼠手持模式下
            if (app_control_get_mouse_mode() && !get_hid_mouse_handfree_mode())
            {
                continue;
            }

            if (has_user_started_controlling_with_arm())
            {
                continue;
            }
        }

        if (is_ble_dfu_thread_running())
        {
            continue;
        }

        gesture_recognition_algorithm(&watch_sensor.gesture_data);
    }
}

/**
 * @brief Thread initialization function for gesture recognition
 *
 * @return RT_EOK on success, -RT_ERROR on failure
 */
static int gesture_recognition_thread_init(void)
{
    gesture_recognition_thread = rt_thread_create(
        "gesture_recognition", gesture_recognition_thread_entry, RT_NULL,
        IMU_THREAD_STACK_SIZE, IMU_THREAD_PRIORITY, IMU_THREAD_TIMESLICE);
    if (gesture_recognition_thread != RT_NULL)
    {
        rt_thread_startup(gesture_recognition_thread);
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
#ifndef BSP_USING_PC_SIMULATOR
INIT_APP_EXPORT(gesture_recognition_thread_init);
#endif

#ifdef BSP_USING_PM
static int utest_gesture(int argc, char *argv[])
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "release") == 0)
        {
            gesture_unlock_screen_handler();
        }
        else if (strcmp(argv[1], "tap") == 0)
        {
            send_virtual_gesture_event(GESTURE_EVENT_PRESS);
            extern bool get_enable_tap_and_hold(void);
            if (get_enable_tap_and_hold())
            {
                // if (watch_sys_sync.is_ppg_enabled())
                watch_sys_sync.notify_tap_detected();
            }
            else
            {
                rt_thread_mdelay(100);
                send_virtual_gesture_event(GESTURE_EVENT_TAP);
            }
        }
        else if (strcmp(argv[1], "movement") == 0)
        {
            // gesture_recognition_lock = true;
            // trigger_gesture_unlock_timer();
        }
    }
    return 0;
}
MSH_CMD_EXPORT(utest_gesture, "utest_gesture [OPTION] ...");
#endif /* BSP_USING_PM */

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/