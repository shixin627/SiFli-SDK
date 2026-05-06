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
/* Temporary PPG producer/consumer race instrumentation. Remove this line
 * (and the matching one in data_service.c) once verification is done. */
#ifndef PPG_RACE_DEBUG
    #define PPG_RACE_DEBUG 1
#endif

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "bf0_ble_bass.h"
#include "bloc_peripheral.h"
#include "bloc_notification.h"
    #include "bloc_v2t.h"
#ifdef BSP_USING_COMMUNICATE
    #include "communicate_protocol.h"
    #include "communicate_task.h"
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

bool is_sleep_mode(void)
{
#ifndef SOC_BF0_LCPU
    bool active = gui_is_active();
    return !active;
#else
    return _sleep_mode;
#endif
}

#ifdef SOC_BF0_LCPU
void set_sleep_mode(bool mode)
{
    if (_sleep_mode == mode)
    {
        return;
    }
    _sleep_mode = mode;
}
#else

bool is_hcpu_suspend(void)
{
    return SkaiWatchSys.sys_power_status == SYS_POWER_STATUS_OFF;
}
#endif

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
    #define THREAD_STACK_SIZE 4 * 1024
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
extern void heart_rate_subscribe(void);
extern void heart_rate_unsubscribe(void);
extern void ppg_subscribe(void);
extern void ppg_unsubscribe(void);
extern void audio_subscribe(void);
extern void audio_unsubscribe(void);

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
        #endif //  #ifndef BSP_USING_PC_SIMULATOR

/* Helpers used by the small wrappers below. The provider exposes them as
   distinct function pointers, so each wrapper has to be its own symbol. */
static void send_simple_event(PeripheralEvent ev)
{
    PeripheralMessageData data = { .event = ev };
    send_peripheral_data(data);
}
static void send_subscribe_event(PeripheralEvent ev, bool status)
{
    PeripheralMessageData data = { .event = ev };
    data.arg.subscribe_status = status;
    send_peripheral_data(data);
}

static void hcpu_reboot(void)  { send_simple_event(HCPU_REBOOT); }
static void hcpu_resume(void)  { send_simple_event(HCPU_RESUME); }
static void hcpu_suspend(void) { send_simple_event(HCPU_SUSPEND); }

static void subscribe_audio_mic_sensor(bool status)
{
    if (voice_provider.audio_subscribed == status) return;
    send_subscribe_event(SUBSCRIBE_AUDIO_MIC, status);
}
static void ppg_sensor_power_control(uint8_t status)    { send_subscribe_event(POWER_MANAGE_HR, status); }
static void subscribe_accelerometer_sensor(bool status) { send_subscribe_event(SUBSCRIBE_ACCELEROMETER, status); }
static void subscribe_hr_sensor(bool status)            { send_subscribe_event(SUBSCRIBE_HR, status); }
static void subscribe_ppg_signal(bool status)           { send_subscribe_event(SUBSCRIBE_PPG, status); }

static bool motor_on = false;
static rt_timer_t motor_on_timer = NULL;
static void set_motor_off(void *param)
{
    motor_on = false;
}
bool get_motor_status(void)
{
    // LOG_I("Motor status: %s", motor_on ? "ON" : "OFF");
    return motor_on;
}
static void start_motor_on_timer(uint32_t duration_ms)
{
    if (motor_on_timer)
    {
        rt_timer_stop(motor_on_timer);
    }

    if (!motor_on_timer)
    {
        motor_on_timer = rt_timer_create("motor_off_timer", set_motor_off, NULL,
                                         duration_ms, RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_control(motor_on_timer, RT_TIMER_CTRL_SET_TIME, &duration_ms);
    }

    if (motor_on_timer)
    {
        rt_timer_start(motor_on_timer);
    }
}

static void control_motor_vibration(bool enable, motor_params_t *params)
{
    if (SkaiWatchSys.sys_power_status != SYS_POWER_STATUS_ON)
    {
        LOG_W("Cannot control motor while HCPU is suspended");
        return;
    }
    LOG_I("Control motor: %s, duty_cycle: %d, period: %d, repeat_times: %d",
          enable ? "ON" : "OFF", params ? params->duty_cycle : 0,
          params ? params->period : 0, params ? params->repeat_times : 0);
    PeripheralMessageData data;
    data.event = CONTROL_MOTOR;
    data.arg.motor_control.enable = enable;
    if (enable && params)
    {
        data.arg.motor_control.params = *params;
    }
    send_peripheral_data(data);
    motor_on = true;
    start_motor_on_timer(((params->period / 1000) * params->repeat_times) +
                         600);
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

static void save_watch_shared_prefs(watch_prefs_key key)
{
    PeripheralMessageData data;
    data.event = SAVE_SHARE_PREFS;
    data.arg.value = key;
    send_peripheral_data(data);
}

static void notify_battery_voltage(uint16_t voltage)
{
    PeripheralMessageData data;
    data.event = NOTIFY_BATTERY_VOLTAGE;
    data.arg.data = voltage;
    send_peripheral_data(data);
}

static bool low_power_warning = false;
static void charge_status_callback(uint8_t status)
{
    PeripheralMessageData data;
    data.event = CHARGE_STATUS_CALLBACK;
    data.arg.value = status;
    send_peripheral_data(data);
}

static void read_fsr_adc(void)
{
    PeripheralMessageData data;
    data.event = FSR_ADC_READ;
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
    peripheral_provider.hcpu_reboot = hcpu_reboot;
    peripheral_provider.hcpu_resume = hcpu_resume;
    peripheral_provider.hcpu_suspend = hcpu_suspend;
    peripheral_provider.hr_set_power = ppg_sensor_power_control;
    peripheral_provider.subscribe_accelerometer_sensor =
        subscribe_accelerometer_sensor;
    peripheral_provider.subscribe_hr_sensor = subscribe_hr_sensor;
    peripheral_provider.subscribe_ppg_signal = subscribe_ppg_signal;
    peripheral_provider.control_motor = control_motor_vibration;
    peripheral_provider.subscribe_audio_mic_sensor = subscribe_audio_mic_sensor;
    peripheral_provider.control_rgb_led = control_rgb_led;
    peripheral_provider.save_watch_shared_prefs = save_watch_shared_prefs;
    peripheral_provider.notify_battery_voltage = notify_battery_voltage;
    peripheral_provider.charge_status_callback = charge_status_callback;
    peripheral_provider.read_fsr_adc = read_fsr_adc;
    #else
    peripheral_provider.sensor_power_manage = sensor_power_manage;
    #endif
    return 0;
}
INIT_APP_EXPORT(bloc_peripheral_register);

extern void fsr_adc_read(void);
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
                voice_provider.audio_subscribed = data.arg.subscribe_status;
            }
            break;
        #ifndef BSP_USING_PC_SIMULATOR
            case HCPU_REBOOT:
            {
                watch_config_struct_flash_write();
                rt_thread_mdelay(50);
                extern void drv_reboot(void);
                drv_reboot();
                break;
            }
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
                extern void wakeup_chack_music_pause(void);
                wakeup_chack_music_pause();
                watch_sys_sync.notify_system_wakeup();
                SkaiWatchSys.pre_hcpu_wakeup_tick = rt_tick_get();
                SkaiWatchSys.sys_power_status = SYS_POWER_STATUS_ON;
                break;
            }

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
                break;
            }
        #endif // BSP_USING_PC_SIMULATOR
            case POWER_MANAGE_HR:
            {
                watch_sys_sync.hr_power_manage(data.arg.subscribe_status);
                break;
            }
    #else

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

    #ifdef USING_FSR_ADC_SAMPLER
            case FSR_ADC_READ:
            {
                fsr_adc_read();
            }
            break;
    #endif

    #ifdef BSP_USING_WATCH_SYS_CLIENT
            case CONTROL_MOTOR:
            {

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
            }
            break;
        #ifdef RGB_LED_CONTROL_PIN
            case CONTROL_RGB_LED:
            {
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
            }
            break;
        #endif
    #endif

            case SAVE_SHARE_PREFS:
            {
    #ifndef BSP_USING_PC_SIMULATOR
                watch_prefs_key local_key = (watch_prefs_key)data.arg.value;
                store_watch_prefs(local_key);
    #endif
                break;
            }

            case NOTIFY_BATTERY_VOLTAGE:
            {
    #ifndef BSP_USING_PC_SIMULATOR
                    // Notify system components about battery status
        #ifdef BSP_USING_BLOC_NOTIFY
                notify_provider.battery_voltage(SkaiWatchSys.battery_vol_value);
                notify_provider.battery_level(SkaiWatchSys.battery_level_value);
        #endif
                ble_bass_notify_battery_lvl(SkaiWatchSys.watch_conn_id,
                                            SkaiWatchSys.battery_level_value);

                // Handle low battery warnings and shutdown
                if (SkaiWatchSys.battery_level_value <= 1)
                {
                    if (!low_power_warning)
                    {
                        low_power_warning = true;
                        LOG_W("Battery level is very low, please charge it.");
                        watch_system_interact(INTERACT_BAT_LOW_LEVEL,
                                              &low_power_warning);
                    }
                    else if (SkaiWatchSys.battery_level_value == 0)
                    {
                        if (SkaiWatchSys.charger_status == InCharging)
                        {
                            LOG_W("Battery level is 0, but charging in "
                                  "progress.");
                        }
                        else
                        {
                            LOG_W("Battery level is 0, power off.");
                            // watch_system_interact(INTERACT_POWEROFF, NULL);
                        }
                    }
                }
                else if (low_power_warning)
                {
                    // Clear warning once battery is above critical level
                    low_power_warning = false;
                    LOG_I("Battery level is normal.");
                    watch_system_interact(INTERACT_BAT_LOW_LEVEL,
                                          &low_power_warning);
                }
    #endif
                break;
            }

            case CHARGE_STATUS_CALLBACK:
    #ifdef BSP_USING_BLOC_NOTIFY
                notify_provider.charge_status(SkaiWatchSys.charger_status);
    #endif
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
    /* Copy IMU timestamp/acce/gyro only; mag and sample_rate are populated
       elsewhere and must not be clobbered, so don't copy the whole struct. */
    watch_sensor.imu_data.timestamp = data->imu.timestamp;
    watch_sensor.imu_data.acce      = data->imu.acce;
    watch_sensor.imu_data.gyro      = data->imu.gyro;
    watch_sensor.motion_data        = data->motion;
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

#ifdef PPG_RACE_DEBUG
    {
        static uint32_t s_prod_seq = 0;
        static uint32_t s_prev_ts = 0;
        uint32_t v0 = (data && sample_num > 0) ? data[0] : 0;
        uint32_t v1 = (data && sample_num > 1) ? data[1] : 0;
        uint32_t ts = (uint32_t)rt_tick_get_millisecond();
        uint32_t gap = s_prev_ts ? (ts - s_prev_ts) : 0;
        s_prod_seq++;
        if (gap > 50)
        {
            LOG_W("[PPG-PROD] seq=%u n=%u v0=%u v1=%u ts=%u GAP=%ums",
                  s_prod_seq, sample_num, v0, v1, ts, gap);
        }
        else
        {
            LOG_I("[PPG-PROD] seq=%u n=%u v0=%u v1=%u ts=%u", s_prod_seq,
                  sample_num, v0, v1, ts);
        }
        s_prev_ts = ts;
    }
#endif

    /* Note: previously released a per-PPG semaphore for a consumer thread.
       That FIFO mechanism (ENABLE_PPG_SEM_FIFO) is gated off on both cores
       and has been removed. */
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