/**
 ******************************************************************************
 * @file   bloc_peripheral.c
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
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "bloc_peripheral.h"
#include "watch_global_data.h"
#ifdef BSP_USING_BLOC_NOTIFY
    #include "bloc_notification.h"
#endif
#ifdef BSP_USING_COMMUNICATE
    #include "communicate_protocol.h"
#endif
#include "watch_sys_service.h"

#ifndef SOC_BF0_LCPU
    #ifdef BSP_USING_PM
        #include "gui_app_pm.h"
    #endif
#endif

#define DBG_TAG "bloc.peripheral"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static bool _sleep_mode = false;

bool is_hcpu_suspend(void)
{
    return _sleep_mode;
}

bool is_sleep_mode(void)
{
#ifndef SOC_BF0_LCPU
    bool active = gui_is_active();
    return !active;
#else
    return _sleep_mode;
#endif
}
void set_sleep_mode(bool mode)
{
    if (_sleep_mode == mode)
    {
        return;
    }
    _sleep_mode = mode;
    LOG_D("set sleep mode %d\n", mode);
}

/* tap and hold mode variables */
static bool enable_tap_and_hold = ENABLE_TAP_AND_HOLD;
bool get_enable_tap_and_hold(void)
{
    return enable_tap_and_hold;
}
void set_enable_tap_and_hold(bool enable)
{
    if (enable_tap_and_hold == enable)
    {
        return;
    }
    enable_tap_and_hold = enable;
    LOG_I("Tap and Hold mode %s", enable ? "enabled" : "disabled");
}

#ifdef SOC_BF0_LCPU
    #define THREAD_STACK_SIZE 2 * 1024
#else
    #define THREAD_STACK_SIZE 1 * 1024
#endif

#ifndef SOC_BF0_LCPU
    #define THREAD_PRIORITY 8
    #define THREAD_TIMESLICE 10

static bool _tap_status = false;
static bool get_tap_status(void)
{
    return _tap_status;
}
static void set_tap_status(bool status)
{
    if (_tap_status != status)
    {
        _tap_status = status;
        LOG_I("set tap status %d", status);
    }
}

PeripheralProvider peripheral_provider;
static rt_thread_t peripheral_task_tid = RT_NULL;
static rt_mq_t peripheral_queue_handle = RT_NULL;
static void send_peripheral_data(PeripheralMessageData data)
{
    rt_mq_send(peripheral_queue_handle, &data, sizeof(data));
}

    #ifndef SOC_BF0_LCPU
extern void accelerometer_subscribe(void);
extern void accelerometer_unsubscribe(void);
extern void gyroscope_subscribe(void);
extern void gyroscope_unsubscribe(void);
extern void magnetometer_subscribe(void);
extern void magnetometer_unsubscribe(void);
extern void heart_rate_subscribe(void);
extern void heart_rate_unsubscribe(void);
extern void ppg_subscribe(void);
extern void ppg_unsubscribe(void);
extern void audio_subscribe(void);
extern void audio_unsubscribe(void);
extern void audio_start_recording(char *file_name);
extern void audio_stop_recording(void);
extern void audio_start_playing(void *parameter);
extern void audio_stop_playing(void);
extern void powermgr_subscribe(int todo_count);
extern void powermgr_unsubscribe(void);
extern void powermgr_set_screen_brightness(uint16_t brightness);
extern void powermgr_set_screen_timeout(uint16_t timeout);
extern void powermgr_reverse_watch_screen(void);

        #ifdef BSP_USING_PC_SIMULATOR
void heart_rate_subscribe(void)
{
    LOG_D("heart rate subscribe");
}
void heart_rate_unsubscribe(void)
{
    LOG_D("heart rate unsubscribe");
}
void ppg_subscribe(void)
{
    LOG_D("ppg subscribe");
}
void ppg_unsubscribe(void)
{
    LOG_D("ppg unsubscribe");
}
void audio_subscribe(void)
{
    LOG_D("audio subscribe");
}
void audio_unsubscribe(void)
{
    LOG_D("audio unsubscribe");
}
void audio_start_recording(char *file_name)
{
    LOG_D("audio start recording");
}
void audio_stop_recording(void)
{
    LOG_D("audio stop recording");
}
void audio_start_playing(void *parameter)
{
    LOG_D("audio start playing");
}
void audio_stop_playing(void)
{
    LOG_D("audio stop playing");
}
        #endif //  #ifndef BSP_USING_PC_SIMULATOR

static void hcpu_resume(void)
{
    PeripheralMessageData data;
    data.event = HCPU_RESUME;
    send_peripheral_data(data);
}
static void hcpu_suspend(void)
{
    PeripheralMessageData data;
    data.event = HCPU_SUSPEND;
    send_peripheral_data(data);
}

static void ppg_sensor_power_control(uint8_t status)
{
    PeripheralMessageData data;
    data.event = POWER_MANAGE_HR;
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

static void subscribe_accelerometer_sensor(bool status)
{
    PeripheralMessageData data;
    data.event = SUBSCRIBE_ACCELEROMETER;
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

static void subscribe_gyroscope_sensor(bool status)
{
    PeripheralMessageData data;
    data.event = SUBSCRIBE_GYROSCOPE;
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

static void subscribe_magnetometer_sensor(bool status)
{
    PeripheralMessageData data;
    data.event = SUBSCRIBE_MAGNETOMETER;
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

static void subscribe_power_manager(uint16_t todo_count)
{
    PeripheralMessageData data;
    data.event = SUBSCRIBE_POWERMANAGER;
    data.arg.value = todo_count;
    send_peripheral_data(data);
}

static void set_screen_brightness(uint16_t brightness)
{
    PeripheralMessageData data;
    data.event = SET_SCREEN_BRIGHTNESS;
    data.arg.value = brightness;
    send_peripheral_data(data);
}

static void set_screen_timeout(uint16_t timeout)
{
    PeripheralMessageData data;
    data.event = SET_SCREEN_TIMEOUT;
    data.arg.value = timeout;
    send_peripheral_data(data);
}

static void reverse_watch_screen(bool status)
{
    PeripheralMessageData data;
    data.event = REVERSE_WATCH_SCREEN;
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

/// heart rate
static void subscribe_hr_sensor(bool status)
{
    PeripheralMessageData data;
    data.event = SUBSCRIBE_HR;
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

static void subscribe_ppg_signal(bool status)
{
    PeripheralMessageData data;
    data.event = SUBSCRIBE_PPG;
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

static void control_motor_vibration(bool enable, motor_params_t *params)
{
    PeripheralMessageData data;
    data.event = CONTROL_MOTOR;
    data.arg.motor_control.enable = enable;
    if (enable && params)
    {
        data.arg.motor_control.params = *params;
    }
    send_peripheral_data(data);
}

static bool audio_subscribed = false;
static bool get_audio_subscribed(void)
{
    return audio_subscribed;
}
static void set_audio_subscribed(bool status)
{
    audio_subscribed = status;
}
static void subscribe_audio_mic_sensor(bool status)
{
    if (get_audio_subscribed() == status)
    {
        return;
    }
    PeripheralMessageData data;
    data.event = SUBSCRIBE_AUDIO_MIC;
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

static void audio_recording(bool toggle)
{
    PeripheralMessageData data;
    data.event = RECORD_AUDIO_MIC;
    data.arg.audio_record_toggle = toggle;
    send_peripheral_data(data);
}

static void audio_sync(void)
{
    PeripheralMessageData data;
    data.event = SYNC_RECORD_AUDIO;
    send_peripheral_data(data);
}

static void audio_playback(bool toggle)
{
    PeripheralMessageData data;
    data.event = PLAY_AUDIO_SPEAKER;
    data.arg.audio_play_toggle = toggle;
    send_peripheral_data(data);
}

static void control_rgb_led(bool enable, rgb_led_params_t *params)
{
    PeripheralMessageData data;
    data.event = CONTROL_RGB_LED;
    data.arg.rgb_led_control.enable = enable;
    if (enable && params)
    {
        data.arg.rgb_led_control.params = *params;
    }
    send_peripheral_data(data);
}

    #else

static void sensor_power_manage(uint8_t type, uint32_t data)
{
    if (type == SENSOR_IMU)
    {
        PeripheralMessageData msg;
        msg.event = POWER_MANAGE_IMU;
        msg.arg.data = data;
        send_peripheral_data(msg);
    }
    else if (type == SENSOR_PPG)
    {
        PeripheralMessageData msg;
        msg.event = POWER_MANAGE_HR;
        msg.arg.data = data;
        send_peripheral_data(msg);
    }
}
    #endif // #ifndef SOC_BF0_LCPU

static int bloc_peripheral_register(void)
{
    peripheral_provider.get_tap_status = get_tap_status;
    peripheral_provider.set_tap_status = set_tap_status;
    #ifndef SOC_BF0_LCPU
    peripheral_provider.hcpu_resume = hcpu_resume;
    peripheral_provider.hcpu_suspend = hcpu_suspend;
    peripheral_provider.hr_set_power = ppg_sensor_power_control;
    peripheral_provider.subscribe_accelerometer_sensor =
        subscribe_accelerometer_sensor;
    peripheral_provider.subscribe_gyroscope_sensor = subscribe_gyroscope_sensor;
    peripheral_provider.subscribe_magnetometer_sensor =
        subscribe_magnetometer_sensor;
    peripheral_provider.subscribe_power_manager = subscribe_power_manager;
    peripheral_provider.set_screen_brightness = set_screen_brightness;
    peripheral_provider.set_screen_timeout = set_screen_timeout;
    peripheral_provider.reverse_watch_screen = reverse_watch_screen;
    peripheral_provider.subscribe_hr_sensor = subscribe_hr_sensor;
    peripheral_provider.subscribe_ppg_signal = subscribe_ppg_signal;
    peripheral_provider.control_motor = control_motor_vibration;
    peripheral_provider.set_audio_code_status = set_audio_subscribed;
    peripheral_provider.get_audio_code_status = get_audio_subscribed;
    peripheral_provider.subscribe_audio_mic_sensor = subscribe_audio_mic_sensor;
    peripheral_provider.audio_recording = audio_recording;
    peripheral_provider.audio_sync = audio_sync;
    peripheral_provider.audio_playback = audio_playback;
    peripheral_provider.control_rgb_led = control_rgb_led;
    #else
    // peripheral_provider.lift_status_callback = send_lift_status_to_client;
    // peripheral_provider.soft_adt_callback = send_soft_adt_status_to_client;
    // peripheral_provider.gesture_callback = send_gesture_status_to_client;
    peripheral_provider.sensor_power_manage = sensor_power_manage;
    #endif
    return 0;
}
INIT_APP_EXPORT(bloc_peripheral_register);

static void peripheral_task_entry(void *parameter)
{
    static PeripheralMessageData data;
    while (1)
    {
        if (rt_mq_recv(peripheral_queue_handle, &data, sizeof(data),
                       RT_WAITING_FOREVER) == RT_EOK)
        {
            LOG_D("handle peripheral message: %d\n", data.event);
            switch (data.event)
            {
    #ifndef SOC_BF0_LCPU
            case SUBSCRIBE_ACCELEROMETER:
            {
                if (data.arg.subscribe_status)
                {
                    accelerometer_subscribe();
                }
                else
                {
                    accelerometer_unsubscribe();
                }
            }
            break;
            case SUBSCRIBE_GYROSCOPE:
                break;
            case SUBSCRIBE_MAGNETOMETER:
                break;
            case SUBSCRIBE_HR:
            {
                if (data.arg.subscribe_status)
                {
                    heart_rate_subscribe();
                }
                else
                {
                    heart_rate_unsubscribe();
                }
            }
            break;
        #if !kReleaseMode
            case SUBSCRIBE_PPG:
            {
                if (data.arg.subscribe_status)
                {
                    ppg_subscribe();
                }
                else
                {
                    ppg_unsubscribe();
                }
            }
            break;
        #endif
            case SUBSCRIBE_AUDIO_MIC:
            {
                if (data.arg.subscribe_status)
                {
                    audio_subscribe();
                }
                else
                {
                    audio_unsubscribe();
                }
            }
            break;
            case RECORD_AUDIO_MIC:
            {
                // TODO: start or stop audio recording
            }
            break;
            case SYNC_RECORD_AUDIO:
            {
                // TODO: implement audio sync
            }
            break;
            case PLAY_AUDIO_SPEAKER:
            {
                // TODO: implement audio playback
            }
            break;
        #ifndef BSP_USING_PC_SIMULATOR
            case SUBSCRIBE_POWERMANAGER:
            {
                if (data.arg.value)
                {
                    powermgr_subscribe(data.arg.value);
                }
                else
                {
                    powermgr_unsubscribe();
                }
            }
            break;
            case SET_SCREEN_BRIGHTNESS:
            {
                powermgr_set_screen_brightness(data.arg.value);
            }
            break;
            case SET_SCREEN_TIMEOUT:
            {
                powermgr_set_screen_timeout(data.arg.value);
            }
            break;
            case REVERSE_WATCH_SCREEN:
            {
                if (data.arg.subscribe_status)
                {
                    powermgr_reverse_watch_screen();
                }
            }
            break;
            case HCPU_RESUME:
            {
                watch_sys_sync.sync_api_lock(SkaiWatchSys.motion_control_lock);
                // rt_thread_mdelay(50);
                accelerometer_subscribe();
                // rt_thread_mdelay(60);
                // 防止正在測量時被打斷
                if (SkaiWatchSys.hrs_start_up_mode == 0)
                {
                    peripheral_provider.hr_set_power(1);
                }
                watch_sys_sync.notify_system_wakeup();
                SkaiWatchSys.pre_hcpu_wakeup_tick = rt_tick_get();
                SkaiWatchSys.sys_power_status = SYS_POWER_STATUS_ON;
                set_sleep_mode(false);
            }
            break;
            case HCPU_SUSPEND:
            {
                watch_sys_sync.notify_system_standby();
                if (SkaiWatchSys.hrs_start_up_mode == 0)
                {
                    watch_sys_sync.hr_power_manage(0);
                }
                // rt_thread_mdelay(60);
                accelerometer_unsubscribe();
                // rt_thread_mdelay(50);
                SkaiWatchSys.sys_power_status = SYS_POWER_STATUS_SLEEP;
                set_sleep_mode(true);
            }
            break;
        #endif // BSP_USING_PC_SIMULATOR
            case POWER_MANAGE_HR:
            {
                watch_sys_sync.hr_power_manage(data.arg.subscribe_status);
                break;
            }
    #else

            // case LIFT_STATUS_CALLBACK:
            // {
            // 	watch_sys_sync.lift_status_callback(data.arg.value);
            // }
            // break;
            // case SOFT_ADT_STATUS_CALLBACK:
            // {
            // 	watch_sys_sync.soft_adt_status_callback(data.arg.subscribe_status);
            // }
            // break;
            // case GESTURE_CALLBACK:
            // {
            // 	watch_sys_sync.notify_gesture_event(data.arg.data);
            // }
            // break;
            case POWER_MANAGE_HR:
            {
                if (peripheral_provider.hr_set_power)
                {
                    peripheral_provider.hr_set_power(data.arg.data);
                }
            }
            break;
            case POWER_MANAGE_IMU:
            {
                if (peripheral_provider.imu_set_power)
                {
                    LOG_D("power manage imu: %d\n", data.arg.data);
                    peripheral_provider.imu_set_power(data.arg.data);
                }
            }
            break;
    #endif // #ifndef SOC_BF0_LCPU
            case CONTROL_MOTOR:
            {
    #ifdef BSP_USING_WATCH_SYS_CLIENT
                if (data.arg.motor_control.enable)
                {
                    motor_params_t param = {
                        .duty_cycle = data.arg.motor_control.params.duty_cycle,
                        .period = data.arg.motor_control.params.period,
                        .repeat_times =
                            data.arg.motor_control.params.repeat_times,
                    };
                    watch_sys_sync.control_motor(true, &param);
                }
                else
                {
                    watch_sys_sync.control_motor(false, NULL);
                }
    #endif
            }
            break;
            case CONTROL_RGB_LED:
            {
    #ifdef BSP_USING_WATCH_SYS_CLIENT
                watch_sys_rgb_led_params_t params;
                params.enable = data.arg.rgb_led_control.enable;

                if (data.arg.rgb_led_control.enable)
                {
                    rgb_color_t color = data.arg.rgb_led_control.params.color;
                    params.red = color.red;
                    params.green = color.green;
                    params.blue = color.blue;
                    params.brightness =
                        data.arg.rgb_led_control.params.brightness;
                    params.animation_mode =
                        data.arg.rgb_led_control.params.animation_mode;
                    params.period_ms = data.arg.rgb_led_control.params
                                           .period_ms; // Default period
                    params.repeat_times =
                        data.arg.rgb_led_control.params
                            .repeat_times; // Infinite by default

                    LOG_D("RGB LED control sent to LCPU: R=%d G=%d B=%d "
                          "Brightness=%d Mode=%d Period=%d Repeat=%d",
                          params.red, params.green, params.blue,
                          params.brightness, params.animation_mode,
                          params.period_ms, params.repeat_times);
                }
                else
                {
                    params.red = 0;
                    params.green = 0;
                    params.blue = 0;
                    params.brightness = 0;
                    params.animation_mode = 0;
                    params.period_ms = 0;
                    params.repeat_times = 0;
                    LOG_D("RGB LED disable sent to LCPU");
                }

                watch_sys_sync.control_rgb_led(&params);
    #endif
            }
            break;
            default:
            {
                LOG_W("unknown peripheral event: %d", data.event);
            }
            break;
            }
        }
    }
}

static int peripheral_task_init(void)
{
    peripheral_queue_handle =
        rt_mq_create("peripheral_queue", sizeof(PeripheralMessageData), 10,
                     RT_IPC_FLAG_FIFO);
    peripheral_task_tid =
        rt_thread_create("peripheral", peripheral_task_entry, RT_NULL,
                         THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (peripheral_task_tid != RT_NULL)
    {
        rt_thread_startup(peripheral_task_tid);
    }
    return 0;
}

INIT_APP_EXPORT(peripheral_task_init);
#endif // #ifndef SOC_BF0_LCPU

watch_sensor_t watch_sensor;

#if defined(SOC_BF0_HCPU)

void process_motion_sensor_data(motion_sensor_data_t *data)
{
    watch_sensor.imu_data.timestamp = data->imu.timestamp;
    watch_sensor.imu_data.acce = data->imu.acce;
    watch_sensor.imu_data.gyro = data->imu.gyro;
    watch_sensor.motion_data.timestamp = data->motion.timestamp;
    watch_sensor.motion_data.linear_acce = data->motion.linear_acce;
    watch_sensor.motion_data.gravity = data->motion.gravity;
    watch_sensor.motion_data.global_q = data->motion.global_q;
    watch_sensor.motion_data.sensor_q = data->motion.sensor_q;

    if (watch_sensor.imu_sem)
    {
        rt_sem_release(watch_sensor.imu_sem);
    }
}

#elif defined(SOC_BF0_LCPU)
int16_t gsensor_fifo_buffer[GSENSOR_FIFO_BUFFER_SIZE][3];
uint16_t gsensor_fifo_buffer_index = 0;

static int watch_sensor_init_lcpu(void)
{
    #if ENABLE_IMU_SEM_FIFO
    watch_sensor.imu_sem = rt_sem_create("imu_sem", 0, RT_IPC_FLAG_FIFO);
    #endif

    #if ENABLE_PPG_SEM_FIFO
    watch_sensor.ppg_sem = rt_sem_create("ppg_sem", 0, RT_IPC_FLAG_FIFO);
    #endif
    return 0;
}
INIT_APP_EXPORT(watch_sensor_init_lcpu);
#endif

void motion_data_fetch(motion_data_t *data)
{
    watch_sensor.motion_data.timestamp = data->timestamp;
    watch_sensor.motion_data.linear_acce = data->linear_acce;
    watch_sensor.motion_data.gravity = data->gravity;
    watch_sensor.motion_data.global_q = data->global_q;
    watch_sensor.motion_data.sensor_q = data->sensor_q;
    // LOG_D("[T:%d]motion_data_fetch", data->timestamp);
}

bool check_if_imu_sensor_data_is_normal(void)
{
    static Vector3 prevAccData, prevGyroData;

    // Check if both accelerometer and gyroscope data have changed since last
    // check
    bool hasChanged = (prevAccData.x != watch_sensor.imu_data.acce.x ||
                       prevAccData.y != watch_sensor.imu_data.acce.y ||
                       prevAccData.z != watch_sensor.imu_data.acce.z ||
                       prevGyroData.x != watch_sensor.imu_data.gyro.x ||
                       prevGyroData.y != watch_sensor.imu_data.gyro.y ||
                       prevGyroData.z != watch_sensor.imu_data.gyro.z);

    // Update previous values
    prevAccData = watch_sensor.imu_data.acce;
    prevGyroData = watch_sensor.imu_data.gyro;

    return hasChanged;
}

void process_ppg_sensor_data(uint8_t sample_num, uint32_t *data,
                             uint32_t *data2)
{
    if (sample_num > 2)
    {
        sample_num = 2;
    }

    // Process primary PPG data if available
    if (data != NULL)
    {
        watch_sensor.ppg_data.sample_num = sample_num;
        watch_sensor.ppg_data.timestamp = rt_tick_get_millisecond();
        memcpy(watch_sensor.ppg_data.raw_data, data,
               sample_num * sizeof(uint32_t));
    }

    // Process secondary PPG data if available
    if (data2 != NULL)
    {
        watch_sensor.ppg_data2.sample_num = sample_num;
        watch_sensor.ppg_data2.timestamp = rt_tick_get_millisecond();
        memcpy(watch_sensor.ppg_data2.raw_data, data2,
               sample_num * sizeof(uint32_t));
    }
    // LOG_I("[T:%d]ppg_data_fetch: %d, %d", watch_sensor.ppg_data.timestamp,
    // watch_sensor.ppg_data.raw_data[0], watch_sensor.ppg_data2.raw_data[0]);
    // Only trigger the semaphore if not in sleep mode
    if (!is_sleep_mode())
    {
#if ENABLE_PPG_SEM_FIFO
        if (watch_sensor.ppg_sem)
            rt_sem_release(watch_sensor.ppg_sem);
#else
    #ifdef SOC_BF0_LCPU
        for (uint8_t i = 0; i < sample_num; i++)
        {
            process_ppg_rawdata(watch_sensor.ppg_data.raw_data[i]);
        }
    #endif
#endif
    }
}

bool check_if_ppg_sensor_data_is_normal(void)
{
    static uint32_t ppgData;
    if (ppgData == watch_sensor.ppg_data.raw_data[0])
    {
        return false;
    }
    ppgData = watch_sensor.ppg_data.raw_data[0];
    return true;
}

#ifndef SOC_BF0_LCPU

void hr_data_handler(hr_sensor_data_t *data)
{
    watch_sensor.hr_data.timestamp = data->timestamp;
    watch_sensor.hr_data.hr = data->hr;

    notify_provider.hr(data->hr);
}

    #if ENABLE_PPG_SEM_FIFO
        /**
         ***** PPG data processing
         */
        #define PPG_THREAD_STACK_SIZE 1 * 1024
        #define PPG_THREAD_PRIORITY 16
        #define PPG_THREAD_TIMESLICE 10
static rt_thread_t gesture_ppg_thread = RT_NULL;
//*************連續傳原始數據*************//
static void send_ppg_dataset_with_ble(float *ppg, int len)
{
    sensor_buf_float_t ppg_data = {.data = ppg, .length = len};
    L1SendData data = {.event = L1SEND_HEART_RATE_SERIES,
                       .res.ppg_data = ppg_data};
    L1_send_event(data);
}

extern bool ppg_data_collection;
static float ble_ppg_data[4];
static void gesture_ppg_thread_entry(void *parameter)
{
    watch_sensor.ppg_sem = rt_sem_create("ppg_sem", 0, RT_IPC_FLAG_FIFO);
    while (1)
    {
        rt_sem_take(watch_sensor.ppg_sem, RT_WAITING_FOREVER);

        if (ppg_data_collection)
        {
            int sample_num = watch_sensor.ppg_data.sample_num;
            for (int i = 0; i < sample_num; i++)
            {
                ble_ppg_data[2 * i] = (float)watch_sensor.ppg_data.raw_data[i];
                ble_ppg_data[2 * i + 1] =
                    (float)watch_sensor.ppg_data2.raw_data[i];
            }
            send_ppg_dataset_with_ble(ble_ppg_data, sample_num * 2);
        }
    }
}

static int gesture_ppg_thread_init(void)
{
    gesture_ppg_thread = rt_thread_create(
        "ppg", gesture_ppg_thread_entry, RT_NULL, PPG_THREAD_STACK_SIZE,
        PPG_THREAD_PRIORITY, PPG_THREAD_TIMESLICE);
    if (gesture_ppg_thread != RT_NULL)
    {
        rt_thread_startup(gesture_ppg_thread);
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
INIT_APP_EXPORT(gesture_ppg_thread_init);

static int utest_peripheral_task(int argc, char *argv[])
{
    if (argc >= 3)
    {
        if (strcmp(argv[1], "-acce") == 0)
        {
            bool status = atoi(argv[2]);
            subscribe_accelerometer_sensor(status);
        }
        else if (strcmp(argv[1], "-gyro") == 0)
        {
            bool status = atoi(argv[2]);
            subscribe_gyroscope_sensor(status);
        }
        else if (strcmp(argv[1], "-mag") == 0)
        {
            bool status = atoi(argv[2]);
            subscribe_magnetometer_sensor(status);
        }
        else if (strcmp(argv[1], "-ppg") == 0)
        {
            bool status = atoi(argv[2]);
            subscribe_ppg_signal(status);
        }
        else if (strcmp(argv[1], "-hr") == 0)
        {
            bool status = atoi(argv[2]);
            subscribe_hr_sensor(status);
        }
        else if (strcmp(argv[1], "-audio") == 0)
        {
            bool status = atoi(argv[2]);
            subscribe_audio_mic_sensor(status);
        }
        else if (strcmp(argv[1], "-record") == 0)
        {
            bool status = atoi(argv[2]);
            audio_recording(status);
        }
        else if (strcmp(argv[1], "-play") == 0)
        {
            bool status = atoi(argv[2]);
            audio_playback(status);
        }
    }
    else
    {
        LOG_D("utest_peripheral_task [OPTION] ...\n"
              "Options:\n"
              "  -acce [0/1]\n"
              "  -gyro [0/1]\n"
              "  -mag [0/1]\n"
              "  -ppg [0/1]\n"
              "  -hr [0/1]\n"
              "  -audio [0/1]\n"
              "  -record [0/1]\n"
              "  -play [0/1]\n"
              "  -request_batt_lvl\n");
    }
    return 0;
}
MSH_CMD_EXPORT(utest_peripheral_task, "utest_peripheral_task [OPTION] ...");
    #endif // #ifdef ENABLE_PPG_SEM_FIFO
    /////////////////////////
    // #define UTEST_PBR_PIN
    #ifdef UTEST_PBR_PIN
        #define TEST_INT_PIN 162 // 162=2+160

static void module_int_handle(void *args)
{
    LOG_D("test module int %d", rt_tick_get());
}
int hal_module_int_init(void)
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at module\n");
        return -1;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

    // int pin cfg
    m.pin = TEST_INT_PIN;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable charging int
    rt_pin_mode(TEST_INT_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING_FALLING, module_int_handle,
                      (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);

    rt_device_close(device);
    return 0;
}
INIT_APP_EXPORT(hal_module_int_init);

        #define THREAD_STACK_SIZE 1 * 1024
        #define THREAD_PRIORITY 25
        #define THREAD_TIMESLICE 10
rt_thread_t module_task_tid = RT_NULL;
static void module_task_entry(void *parameter)
{
    while (1)
    {
        int pin_status = rt_pin_read(TEST_INT_PIN);
        LOG_D("[t:%d]module pin status: %d", rt_tick_get(), pin_status);
        rt_thread_mdelay(1);
    }
}
static int module_task_init(void)
{
    module_task_tid =
        rt_thread_create("module_task", module_task_entry, RT_NULL,
                         THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (module_task_tid != RT_NULL)
    {
        rt_thread_startup(module_task_tid);
    }
    return 0;
}
INIT_APP_EXPORT(module_task_init);

    #endif // #ifdef UTEST_PBR_PIN
/////////////////////////
#else
static bool imu_data_collection_mode = false;
static bool imu_rawdata_collection = false;
/**
 * @brief Enable or disable IMU data collection
 * @param enable 0: none, 1: low frequency(50Hz), 2: high frequency(250Hz)
 */
void set_imu_data_collection(bool enable)
{
    imu_data_collection_mode = enable;
}

bool is_imu_data_collection(void)
{
    return imu_data_collection_mode;
}

/**
 * @brief Enable or disable IMU raw data collection
 * @param enable true to enable collection, false to disable
 */
void set_imu_rawdata_collection(bool enable)
{
    LOG_D("Set IMU raw data collection: %d", enable);
    imu_rawdata_collection = enable;
}
bool is_imu_rawdata_collection(void)
{
    return imu_rawdata_collection;
}

    #if ENABLE_PPG_SEM_FIFO
        #define PPG_THREAD_STACK_SIZE 1 * 1024
        #define PPG_THREAD_PRIORITY 16
        #define PPG_THREAD_TIMESLICE 10
/**
 * @brief PPG thread entry function
 * @param parameter Thread parameter
 */
static void gesture_ppg_thread_entry(void *parameter)
{
    while (1)
    {
        rt_sem_take(watch_sensor.ppg_sem, RT_WAITING_FOREVER);

        // Process main PPG data
        uint32_t *buffer = gh3018_get_ppg();
        if (buffer != NULL)
        {
            memcpy(buffer, watch_sensor.ppg_data.raw_data,
                   watch_sensor.ppg_data.sample_num * sizeof(uint32_t));
            for (int i = 0; i < watch_sensor.ppg_data.sample_num; i++)
            {
                process_ppg_rawdata(buffer[i]);
            }
        }
        // Process secondary PPG data if available
        // buffer = gh3018_get_ppg2();
        // if (buffer != NULL)
        // {
        //     memcpy(buffer, watch_sensor.ppg_data2.raw_data,
        //     watch_sensor.ppg_data2.sample_num * sizeof(uint32_t));
        // }
    }
}

/**
 * @brief Initialize PPG thread
 * @return RT_EOK on success, error code otherwise
 */
static int gesture_ppg_thread_init(void)
{
    gesture_ppg_thread = rt_thread_create(
        "ppg", gesture_ppg_thread_entry, RT_NULL, PPG_THREAD_STACK_SIZE,
        PPG_THREAD_PRIORITY, PPG_THREAD_TIMESLICE);
    if (gesture_ppg_thread != RT_NULL)
    {
        rt_thread_startup(gesture_ppg_thread);
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
INIT_APP_EXPORT(gesture_ppg_thread_init);
    #endif

#endif // #ifndef SOC_BF0_LCPU

// ----- RTC -----
static time_t now;
time_t get_current_time(void)
{
    time(&now);
    return now;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/