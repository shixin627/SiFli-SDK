/**
 ******************************************************************************
 * @file   watch_system_client.c
 * @author Skaiwalk software development team
 * @brief  Watch system client source.
 *
 ******************************************************************************
 */
/**
 * @attention
 * Copyright (c) 2018 - 2023,  Skaiwalk Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated
 * circuit in a product or a software update for such product, must reproduce
 * the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <rtthread.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include "data_service_subscriber.h"
#ifdef BSP_USING_WATCH_SYS_CLIENT
    #include "watch_sys_service.h"
#endif
#include "ui_datasrv_subscriber.h"
#include "bloc_setting.h"
#include "bloc_motor.h"
#include "power_manager_service.h"
#include "littlevgl2rtt.h"
#include "bloc_peripheral.h"
#ifdef BSP_USING_BLOC_NOTIFY
    #include "bloc_notification.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif
#include "bsp_board.h"
#include "math.h"
#ifdef BSP_USING_GESTURE_HANDLER
    #include "gesture_handler.h"
#endif
#include "ui_handler.h"
#include "app_mainmenu.h"
#ifdef BSP_USING_COMMUNICATE
    #include "communicate_protocol.h"
    #include "communicate_task.h"
#endif

#define DBG_LVL DBG_LOG
#define DBG_TAG "CLIENT.POWER"
#include "rtdbg.h"

#define ENABLE_IMU_SENSOR 1
#define ENABLE_MAG_SENSOR 0
#define ENABLE_PPG_SENSOR 1
#define ENABLE_CHARGE_MONITOR 1

//////////////////// Watch System Client ////////////////////
watch_sys_sync_t watch_sys_sync;
static datac_handle_t watch_sys_client_handle;

#if ENABLE_PPG_SENSOR
static bool _is_imu_sensor_enabled = false;

static void set_imu_enabled(bool enabled)
{
    _is_imu_sensor_enabled = enabled;
}

bool is_imu_enabled(void)
{
    return _is_imu_sensor_enabled;
}
#endif

#if ENABLE_MAG_SENSOR
static bool _is_mag_sensor_enabled = false;

void set_mag_sensor_enabled(bool enabled)
{
    _is_mag_sensor_enabled = enabled;
}

bool is_mag_sensor_enabled(void)
{
    return _is_mag_sensor_enabled;
}
#endif

#if ENABLE_PPG_SENSOR
static bool _is_ppg_sensor_enabled = false;

void set_ppg_enabled(bool enabled)
{
    _is_ppg_sensor_enabled = enabled;
}

bool is_ppg_enabled(void)
{
    return _is_ppg_sensor_enabled;
}
#endif

static void set_battery_voltage(uint16_t voltage, uint16_t percentage)
{
    if (voltage != SkaiWatchSys.battery_vol_value)
    {
        SkaiWatchSys.battery_vol_value = voltage;
    }
    if (percentage != SkaiWatchSys.battery_level_value)
    {
        SkaiWatchSys.battery_level_value = percentage;
    }
}

extern int get_gravity_position(void);

static int watch_sys_service_callback(data_callback_arg_t *arg)
{
    LOG_I("watch_sys_service_callback: msg_id=%d", arg->msg_id);
    switch (arg->msg_id)
    {
    case MSG_SERVICE_DATA_NTF_IND:
    {
        watch_sys_service_data_ntf_ind_t *data_ntf_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ntf_ind_t));
        data_ntf_ind = (watch_sys_service_data_ntf_ind_t *)arg->data;
        RT_ASSERT(data_ntf_ind);
        uint16_t percentage = data_ntf_ind->data & 0xFFFF;
        uint32_t vol = data_ntf_ind->data >> 16;
        set_battery_voltage(vol, percentage);
        // send_sys_interact_event(SYS_EVENT_BATT_VOLTAGE);
        peripheral_provider.notify_battery_voltage(vol);
        break;
    }
    case MSG_SERVICE_BATTERY_DATA_RSP:
    {
        watch_sys_service_data_rsp_t *data_rsp;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_rsp_t));
        data_rsp = (watch_sys_service_data_rsp_t *)arg->data;
        RT_ASSERT(data_rsp);
        uint16_t percentage = data_rsp->data & 0xFFFF;
        uint16_t vol = data_rsp->data >> 16;
        set_battery_voltage(vol, percentage);
        // send_sys_interact_event(SYS_EVENT_BATT_VOLTAGE);
        peripheral_provider.notify_battery_voltage(vol);
        break;
    }
    case MSG_SERVICE_BATTERY_DATA_IND:
    {
        watch_sys_service_data_ind_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
        data_ind = (watch_sys_service_data_ind_t *)arg->data;
        RT_ASSERT(data_ind);
        uint16_t percentage = data_ind->data & 0xFFFF;
        uint16_t vol = data_ind->data >> 16;
        set_battery_voltage(vol, percentage);
        // send_sys_interact_event(SYS_EVENT_BATT_VOLTAGE);
        peripheral_provider.notify_battery_voltage(vol);
        break;
    }
    case MSG_SERVICE_CHARGE_STATE_IND:
    {
        watch_sys_service_data_ind_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
        data_ind = (watch_sys_service_data_ind_t *)arg->data;
        RT_ASSERT(data_ind);
        uint8_t status = data_ind->data;
        SkaiWatchSys.charger_status = status;
        // send_sys_interact_event(SYS_EVENT_BATT_CHARGE);
        peripheral_provider.charge_status_callback(status);
        break;
    }
    case MSG_SERVICE_IMU_STATE_IND:
    {
        watch_sys_service_data_ind_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
        data_ind = (watch_sys_service_data_ind_t *)arg->data;
        RT_ASSERT(data_ind);
        bool status = data_ind->data;
        set_imu_enabled(status);
        break;
    }
    case MSG_SERVICE_MAG_STATE_IND:
    {
        watch_sys_service_data_ind_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
        data_ind = (watch_sys_service_data_ind_t *)arg->data;
        RT_ASSERT(data_ind);
        bool status = data_ind->data;
        break;
    }
    case MSG_SERVICE_PPG_STATE_IND:
    {
        watch_sys_service_data_ind_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
        data_ind = (watch_sys_service_data_ind_t *)arg->data;
        RT_ASSERT(data_ind);
        bool status = data_ind->data;
        set_ppg_enabled(status);
        break;
    }
    case MSG_SERVICE_LIFT_IND:
    {
        watch_sys_service_data_ind_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
        data_ind = (watch_sys_service_data_ind_t *)arg->data;
        RT_ASSERT(data_ind);
        uint8_t status = data_ind->data;
        if (status == 0 && setting_provider.get_power_save_mode() != 0)
        {
#if !kReleaseMode
            extern bool pause_sleep_cause_of_dev_reson(void);
            if (!pause_sleep_cause_of_dev_reson())
#endif
            {
                if (!is_sleep_mode())
                {
                    watch_system_interact(WATCH_SLEEP, NULL);
                }
            }
        }
        else if (status == 2) // 正常抬腕
        {
            // watch_hcpu_resume_with_reason(WAKEUP_REASON_OTHER);
            SkaiWatchSys.flag_field.is_wearing = true;
            watch_system_interact(HCPU_WAKEUP, NULL);
        }
        else if (status == 3) // 往內旋解鎖
        {
            if (!is_ble_dfu_thread_running())
            {
                // watch_hcpu_resume_with_reason(WAKEUP_REASON_ROTATE_INWARD);
                send_virtual_gesture_event(GESTURE_EVENT_WRIST_PRONATION);
            }
        }
        break;
    }
    case MSG_SERVICE_SOFT_ADT_IND:
    {
        watch_sys_service_data_ind_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
        data_ind = (watch_sys_service_data_ind_t *)arg->data;
        RT_ASSERT(data_ind);
        bool status = data_ind->data;
        LOG_I("[WATCH_SOFT_ADT] soft detect status:%d", status);
        SkaiWatchSys.flag_field.is_wearing = status;
        break;
    }
    case MSG_SERVICE_GESTURE_IND:
    {
        watch_sys_service_data_ind_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
        data_ind = (watch_sys_service_data_ind_t *)arg->data;
        RT_ASSERT(data_ind);
        rt_uint32_t gesture = data_ind->data;
#ifdef BSP_USING_GESTURE_HANDLER
        send_virtual_gesture_event(gesture);
#endif
        break;
    }
    case MSG_SERVICE_GESTURE_DATASET_IND:
    {
        watch_sys_gesture_dataset_rsp_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_gesture_dataset_rsp_t));
        data_ind = (watch_sys_gesture_dataset_rsp_t *)arg->data;
        RT_ASSERT(data_ind);
        watch_sensor.gesture_data.timestamp = data_ind->timestamp;
        watch_sensor.gesture_data.sample_num = data_ind->count;
        memcpy(watch_sensor.gesture_data.dataset, data_ind->acce,
               data_ind->count * sizeof(watch_sys_linear_acce_t));
        if (watch_sensor.gesture_sem)
        {
            rt_sem_release(watch_sensor.gesture_sem);
        }
        break;
    }
    case MSG_SERVICE_GESTURE_PPG_DATASET_IND:
    {
        watch_sys_gesture_ppg_dataset_rsp_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_gesture_ppg_dataset_rsp_t));
        data_ind = (watch_sys_gesture_ppg_dataset_rsp_t *)arg->data;
        RT_ASSERT(data_ind);
        watch_sensor.gesture_data.timestamp = data_ind->timestamp;
        watch_sensor.gesture_data.sample_num = data_ind->count;
        memcpy(watch_sensor.gesture_data.dataset_ppg, data_ind->acce,
               data_ind->count * sizeof(watch_sys_linear_acce_ppg_t));
        if (watch_sensor.gesture_sem)
        {
            rt_sem_release(watch_sensor.gesture_sem);
        }
        break;
    }
    case MSG_SERVICE_HEALTH_INFO_IND:
    {
        watch_sys_heath_info_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_heath_info_t));
        data_ind = (watch_sys_heath_info_t *)arg->data;
        RT_ASSERT(data_ind);
        SkaiWatchSys.health_info_today = *data_ind;
        SkaiWatchSys.gPedoData.global_steps = data_ind->steps;
        SkaiWatchSys.gPedoData.global_distance = data_ind->distance; // mm
        SkaiWatchSys.gPedoData.global_calories = data_ind->calories; // cal
        commu_send_sport_data();
        break;
    }
    case MSG_SERVICE_SLEEP_STATE_IND:
    {
        watch_sys_sleep_state_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_sleep_state_t));
        data_ind = (watch_sys_sleep_state_t *)arg->data;
        RT_ASSERT(data_ind);
        set_watch_sleep_state(data_ind);
        break;
    }
    case MSG_SERVICE_DEBUG_LOG_IND:
    {
        watch_sys_debug_log_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_debug_log_t));
        data_ind = (watch_sys_debug_log_t *)arg->data;
        RT_ASSERT(data_ind);
        char *log = data_ind->log;
        LOG_I(log);
        break;
    }
    case MSG_SERVICE_MINUTE_ACTIVITY_IND:
    {
        watch_sys_minute_activity_t *data_ind;
        RT_ASSERT(arg->data_len == sizeof(watch_sys_minute_activity_t));
        data_ind = (watch_sys_minute_activity_t *)arg->data;
        RT_ASSERT(data_ind);
        SkaiWatchSys.activity = *data_ind;
        commu_send_minute_activity();
        break;
    }
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        RT_ASSERT(watch_sys_client_handle == rsp->handle);
        break;
    }
    default:
        break;
    }

    return 0;
}

/**
 * @brief Send a message to the watch system service with retry support
 *
 * @param msg The message to send
 * @param retry_delay_ms Delay before retry in milliseconds
 * @param max_retries Maximum number of retries
 * @return rt_err_t RT_EOK on success, error code otherwise
 */
static rt_err_t send_watch_sys_msg_with_retry(data_msg_t *msg,
                                              uint32_t retry_delay_ms,
                                              uint8_t max_retries)
{
    if (watch_sys_client_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("Watch system client handle is not initialized");
        return -RT_ERROR;
    }
    rt_err_t err = RT_EOK;
    uint8_t retry_count = 0;
    uint8_t operation_code = msg->body[0];
    // Initial send attempt
    err = datac_send_msg(watch_sys_client_handle, msg);

    // Retry logic
    while (err != RT_EOK && retry_count < max_retries)
    {
        LOG_E("send code [%d]failed, retrying (%d/%d)", operation_code,
              retry_count + 1, max_retries);
        rt_thread_mdelay(retry_delay_ms);
        err = datac_send_msg(watch_sys_client_handle, msg);
        retry_count++;
    }

    // Log the final result
    if (err != RT_EOK)
    {
        LOG_E("send code [%d] failed after %d retries", operation_code,
              retry_count);
    }
    else if (retry_count > 0)
    {
        LOG_D("send code [%d] succeeded after %d retries", operation_code,
              retry_count);
    }
    else
    {
        LOG_D("send code [%d] succeeded on first attempt", operation_code);
    }

    return err;
}

static int request_battery_voltage(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = SysRequestBattery;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    // RT_ASSERT(RT_EOK == err);
    return err;
}

static int request_charge_status(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = SysRequestChargeStatus;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    // RT_ASSERT(RT_EOK == err);
    return err;
}

static int request_pedometer_data(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = SysRequestPedometerData;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static int notify_system_standby(void)
{
    if (gui_app_is_actived(APP_ID_MESSAGE))
    {
        gui_app_exit(APP_ID_MESSAGE);
    }
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = SysStandBy;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static int notify_system_wakeup(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = SysWakeUp;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static int hr_power_manage(bool enable)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = PpgSensorPowerManage;
    msg.body[1] = enable ? 1 : 0;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static int sync_api_lock(bool locked)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = SysSyncApiLock;
    msg.body[1] = locked ? 1 : 0;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    LOG_D("sync_api_lock:%d", locked);
    return err;
}

static bool tap_type = false;
void set_tap_type(bool type)
{
    tap_type = type;
}
static int notify_tap_detected(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = UserTapDetected;
    msg.body[1] = tap_type;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    RT_ASSERT(RT_EOK == err);
    return err;
}

static int notify_imu_data_collection(bool status)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = ImuDataCollection;
    msg.body[1] = status ? 1 : 0;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    RT_ASSERT(RT_EOK == err);
    return err;
}

static int notify_imu_rawdata_collection(bool status)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = ImuRawdataCollection;
    msg.body[1] = status ? 1 : 0;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    RT_ASSERT(RT_EOK == err);
    return err;
}

static int notify_calibration_global_attitude(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = CalibrateGlobalAttitude;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static int notify_user_profile(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = UserProfileUpdate;
    msg.body[1] = SkaiWatchSys.user_data.user_profile.bit_field.gender;
    msg.body[2] = SkaiWatchSys.user_data.user_profile.bit_field.age;
    msg.body[3] = SkaiWatchSys.user_data.user_profile.bit_field.height;
    msg.body[4] = SkaiWatchSys.user_data.user_profile.bit_field.weight;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static int notify_debug_param_update(uint8_t value)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = DebugParamUpdate;
    msg.body[1] = value;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static int control_motor(bool enable, motor_params_t *param)
{
    LOG_D("MOTOR control duty_cycle: %d, period: %d, repeat_times: %d",
          param->duty_cycle, param->period, param->repeat_times);
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = MotorControl;
    msg.body[1] = enable ? 1 : 0;
    if (enable)
    {
        msg.body[2] = param->duty_cycle;
        memcpy(msg.body + 3, &param->period, sizeof(rt_uint32_t));
        msg.body[7] = param->repeat_times;
    }
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static void set_debug_mode(bool enable)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = DebugMode;
    msg.body[1] = enable ? 1 : 0;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
}

static int set_multi_gesture_mode(bool enable)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = MultiGestureMode;
    msg.body[1] = enable ? 1 : 0;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

static int set_tap_and_hold_mode(bool enable)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = TapAndHoldMode;
    msg.body[1] = enable ? 1 : 0;
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}

/**
 * @brief Control RGB LED with structured parameters
 *
 * This function sends RGB LED control parameters to the LCPU using the
 * watch_sys_rgb_led_params_t structure for better maintainability.
 *
 * Message format in msg.body:
 * [0]: RgbLedControl command
 * [1]: enable flag
 * [2]: red (0-255)
 * [3]: green (0-255)
 * [4]: blue (0-255)
 * [5]: brightness (0-100)
 * [6]: animation_mode
 * [7-10]: period_ms (uint32_t)
 * [11]: repeat_times
 *
 * @param params Pointer to RGB LED parameters structure
 * @return rt_err_t RT_EOK on success, error code otherwise
 */
static int control_rgb_led(watch_sys_rgb_led_params_t *params)
{
    if (params == NULL)
    {
        LOG_E("RGB LED control: NULL parameters");
        return -RT_ERROR;
    }

    LOG_D("RGB LED control: enable=%d, R=%d, G=%d, B=%d, brightness=%d, "
          "mode=%d, period=%d, repeat=%d",
          params->enable, params->red, params->green, params->blue,
          params->brightness, params->animation_mode, params->period_ms,
          params->repeat_times);

    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;

    msg.body[0] = RgbLedControl;
    msg.body[1] = params->enable;

    if (params->enable)
    {
        msg.body[2] = params->red;
        msg.body[3] = params->green;
        msg.body[4] = params->blue;
        msg.body[5] = params->brightness;
        msg.body[6] = params->animation_mode;
        memcpy(msg.body + 7, &params->period_ms, sizeof(uint16_t));
        memcpy(msg.body + 9, &params->repeat_times, sizeof(uint16_t));
    }

    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);

    return err;
}

/**
 * @brief Register synchronization functions for the watch system
 *
 * This function sets up function pointers in the watch_sys_sync structure
 * to enable communication between the HCPU and LCPU subsystems.
 */
static void register_watch_sys_sync_funs(void)
{
    watch_sys_sync.is_imu_enabled = is_imu_enabled;
    watch_sys_sync.is_ppg_enabled = is_ppg_enabled;
    watch_sys_sync.request_battery_voltage = request_battery_voltage;
    watch_sys_sync.request_charge_status = request_charge_status;
    watch_sys_sync.request_pedometer_data = request_pedometer_data;
    watch_sys_sync.sync_api_lock = sync_api_lock;
    watch_sys_sync.notify_system_standby = notify_system_standby;
    watch_sys_sync.notify_system_wakeup = notify_system_wakeup;
    watch_sys_sync.hr_power_manage = hr_power_manage;
    watch_sys_sync.notify_tap_detected = notify_tap_detected;
    watch_sys_sync.notify_imu_data_collection = notify_imu_data_collection;
    watch_sys_sync.notify_imu_rawdata_collection =
        notify_imu_rawdata_collection;
    watch_sys_sync.notify_calibration_global_attitude =
        notify_calibration_global_attitude;
    watch_sys_sync.notify_user_profile = notify_user_profile;
    watch_sys_sync.notify_debug_param_update = notify_debug_param_update;
    watch_sys_sync.control_motor = control_motor;
    watch_sys_sync.set_debug_mode = set_debug_mode;
    watch_sys_sync.set_multi_gesture_mode = set_multi_gesture_mode;
    watch_sys_sync.set_tap_and_hold_mode = set_tap_and_hold_mode;
    watch_sys_sync.control_rgb_led = control_rgb_led;
}

/**
 * @brief Subscribe to the dual-core synchronization service
 *
 * @return int 0 on success, negative value on failure
 */
int SubscribeDualCoreSyncService(void)
{
    if (watch_sys_client_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        watch_sys_client_handle = datac_open();
    }

    if (watch_sys_client_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_subscribe(watch_sys_client_handle, WATCHSYS_SERVICE_NAME,
                        watch_sys_service_callback, 0);
        LOG_D("Subscribe DualCoreSyncService\n");
        return 0;
    }
    else
    {
        LOG_E("Failed to open DualCoreSyncService");
        return -1;
    }
}

/**
 * @brief Unsubscribe from the dual-core synchronization service
 *
 * @return int 0 on success, negative value on failure
 */
int UnsubscribeDualCoreSyncService(void)
{
    if (watch_sys_client_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("DualCoreSync Service was not initialized");
        return -1;
    }

    rt_err_t result = datac_unsubscribe(watch_sys_client_handle);
    if (result == RT_EOK)
    {
        LOG_I("Successfully unsubscribed from DualCoreSyncService");
        result = datac_close(watch_sys_client_handle);
        if (result == RT_EOK)
        {
            watch_sys_client_handle = DATA_CLIENT_INVALID_HANDLE;
            return 0;
        }
        else
        {
            LOG_E("Failed to close DualCoreSyncService handle, error code: %d",
                  result);
            return -3;
        }
    }
    else
    {
        LOG_E("Failed to unsubscribe from DualCoreSyncService, error code: %d",
              result);
        return -2;
    }
}

static int init_watch_sys_client(void)
{
    // SubscribeDualCoreSyncService();
    register_watch_sys_sync_funs();
    return 0;
}
INIT_APP_EXPORT(init_watch_sys_client);

// #define UTEST_WATCHSYS_CLIENT
#ifdef UTEST_WATCHSYS_CLIENT
static int power_request(int argc, char *argv[])
{
    if (argc == 2)
    {
        if (strcmp(argv[1], "battery") == 0)
        {
            request_battery_voltage();
        }
    }
    return 0;
}
MSH_CMD_EXPORT(power_request, "power request")
#endif
