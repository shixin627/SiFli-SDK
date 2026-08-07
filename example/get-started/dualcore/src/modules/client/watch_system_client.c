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
#include "bloc_health.h"
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

/* Cast `arg->data` to a pointer of the expected indication/response struct,
   asserting that the payload size matches. Eliminates the 4-line preamble
   that every callback case used to repeat. */
#define UNPACK_DATA(arg, T, var)                                               \
    RT_ASSERT((arg)->data_len == sizeof(T));                                   \
    T *var = (T *)(arg)->data;                                                 \
    RT_ASSERT(var)

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

/* ---- continuous-HR batch backlog ------------------------------------------
   LCPU hands over one 30 s batch at a time and cannot see whether BLE is up, so
   the outage retry has to live here. Without it a disconnected stretch is gone
   for good AND indistinguishable from "the sensor produced nothing" — that is
   how the 2026-08-02 overnight run lost 65 minutes.

   Depth 8 = 4 minutes of outage absorbed for ~1.7 KB. Deeper is not obviously
   better: this is a diagnostic stream, and a link down longer than a few minutes
   means the night has a real hole no buffer can paper over. Oldest is dropped
   first so the record stays contiguous at the recent end. */
#define HR_CONT_BACKLOG 8
static watch_sys_hr_cont_t s_hr_cont_q[HR_CONT_BACKLOG];
static uint8_t s_hr_cont_head = 0, s_hr_cont_n = 0;
static uint16_t s_hr_cont_dropped = 0;

static bool hr_cont_try_send(const watch_sys_hr_cont_t *r)
{
    return commu_send_hr_cont(r->base_ts, r->interval_s, r->count, r->bpm,
                              r->qscore, r->qlevel, r->accst, r->accel, r->pi_e3);
}

static void hr_cont_enqueue(const watch_sys_hr_cont_t *rec)
{
    /* Drain the backlog before the new batch so ordering is preserved; stop at the
       first failure — the link is still down and further attempts just burn time
       inside the service callback. */
    while (s_hr_cont_n > 0)
    {
        if (!hr_cont_try_send(&s_hr_cont_q[s_hr_cont_head])) break;
        s_hr_cont_head = (uint8_t)((s_hr_cont_head + 1) % HR_CONT_BACKLOG);
        s_hr_cont_n--;
    }
    if (s_hr_cont_n == 0 && hr_cont_try_send(rec)) return;

    if (s_hr_cont_n >= HR_CONT_BACKLOG)
    {
        s_hr_cont_head = (uint8_t)((s_hr_cont_head + 1) % HR_CONT_BACKLOG);
        s_hr_cont_n--;
        if (++s_hr_cont_dropped % 20 == 1)
        {
            LOG_W("hr_cont backlog full, dropped %u batches", (unsigned)s_hr_cont_dropped);
        }
    }
    s_hr_cont_q[(s_hr_cont_head + s_hr_cont_n) % HR_CONT_BACKLOG] = *rec;
    s_hr_cont_n++;
}

static int watch_sys_service_callback(data_callback_arg_t *arg)
{
    LOG_I("watch_sys_service_callback: msg_id=%d", arg->msg_id);
    switch (arg->msg_id)
    {
    case MSG_SERVICE_DATA_NTF_IND:
    {
        UNPACK_DATA(arg, watch_sys_service_data_ntf_ind_t, data_ntf_ind);
        uint16_t percentage = data_ntf_ind->data & 0xFFFF;
        uint32_t vol = data_ntf_ind->data >> 16;
        set_battery_voltage(vol, percentage);
        peripheral_provider.notify_battery_voltage(vol);
        break;
    }
    case MSG_SERVICE_BATTERY_DATA_RSP:
    {
        UNPACK_DATA(arg, watch_sys_service_data_rsp_t, data_rsp);
        uint16_t percentage = data_rsp->data & 0xFFFF;
        uint16_t vol = data_rsp->data >> 16;
        set_battery_voltage(vol, percentage);
        peripheral_provider.notify_battery_voltage(vol);
        break;
    }
    case MSG_SERVICE_BATTERY_DATA_IND:
    {
        UNPACK_DATA(arg, watch_sys_service_data_ind_t, data_ind);
        uint16_t percentage = data_ind->data & 0xFFFF;
        uint16_t vol = data_ind->data >> 16;
        set_battery_voltage(vol, percentage);
        peripheral_provider.notify_battery_voltage(vol);
        break;
    }
    case MSG_SERVICE_CHARGE_STATE_IND:
    {
        UNPACK_DATA(arg, watch_sys_service_data_ind_t, data_ind);
        uint8_t status = data_ind->data;
        /* The LCPU now reports on every read rather than only on a transition,
         * so the cores can resync after an LCPU-only restart. Dedup here
         * instead: charger_status is what the UI renders, so an unchanged
         * value means there is nothing to redraw -- and re-firing the callback
         * would pop the charging screen back up on every wake-up, even after
         * the user dismissed it. */
        if (SkaiWatchSys.charger_status == status)
            break;
        SkaiWatchSys.charger_status = status;
        peripheral_provider.charge_status_callback(status);
        break;
    }
    case MSG_SERVICE_IMU_STATE_IND:
    {
        UNPACK_DATA(arg, watch_sys_service_data_ind_t, data_ind);
        set_imu_enabled((bool)data_ind->data);
        break;
    }
    case MSG_SERVICE_MAG_STATE_IND:
    {
        /* Payload validated for shape parity with sibling cases; mag state is
           not currently consumed (no setter wired up). */
        UNPACK_DATA(arg, watch_sys_service_data_ind_t, data_ind);
        (void)data_ind;
        break;
    }
    case MSG_SERVICE_PPG_STATE_IND:
    {
        UNPACK_DATA(arg, watch_sys_service_data_ind_t, data_ind);
        set_ppg_enabled((bool)data_ind->data);
        break;
    }
    case MSG_SERVICE_LIFT_IND:
    {
        UNPACK_DATA(arg, watch_sys_service_data_ind_t, data_ind);
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
                    watch_system_sleep();
                }
            }
        }
        else if (status == 2) // 正常抬腕
        {
            // watch_hcpu_resume_with_reason(WAKEUP_REASON_OTHER);
            SkaiWatchSys.flag_field.is_wearing = true;
            watch_system_wakeup();
        }
        else if (status == 3) // 往內旋解鎖
        {
            // watch_hcpu_resume_with_reason(WAKEUP_REASON_ROTATE_INWARD);
            send_virtual_gesture_event(GESTURE_EVENT_WRIST_PRONATION);
        }
        break;
    }
    case MSG_SERVICE_SOFT_ADT_IND:
    {
        UNPACK_DATA(arg, watch_sys_service_data_ind_t, data_ind);
        bool status = (bool)data_ind->data;
        LOG_I("[WATCH_SOFT_ADT] soft detect status:%d", status);
        SkaiWatchSys.flag_field.is_wearing = status;
        break;
    }
    case MSG_SERVICE_GESTURE_IND:
    {
        UNPACK_DATA(arg, watch_sys_service_data_ind_t, data_ind);
#ifdef BSP_USING_GESTURE_HANDLER
        send_virtual_gesture_event((rt_uint32_t)data_ind->data);
#else
        (void)data_ind;
#endif
        break;
    }
    case MSG_SERVICE_GESTURE_DATASET_IND:
    {
        UNPACK_DATA(arg, watch_sys_gesture_dataset_rsp_t, data_ind);
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
        UNPACK_DATA(arg, watch_sys_gesture_ppg_dataset_rsp_t, data_ind);
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
        UNPACK_DATA(arg, watch_sys_heath_info_t, data_ind);
        SkaiWatchSys.health_info_today = *data_ind;
        SkaiWatchSys.gPedoData.global_steps = data_ind->steps;
        SkaiWatchSys.gPedoData.global_distance = data_ind->distance; // mm
        SkaiWatchSys.gPedoData.global_calories = data_ind->calories; // cal
        commu_send_sport_data();
        break;
    }
    case MSG_SERVICE_HR_SAMPLE_IND:
    {
        UNPACK_DATA(arg, watch_sys_hr_sample_t, data_ind);
        /* Background daily HR-curve point from LCPU -> forward to phone. */
        commu_send_heart_curve_sample(data_ind->timestamp, data_ind->bpm);
#ifdef BSP_USING_BLOC_FILESYSTEM
        /* Persist for store-and-forward (survives BLE disconnect). */
        health_store_hr_async(data_ind->timestamp, data_ind->bpm);
#endif
        break;
    }
    case MSG_SERVICE_HR_SKIP_IND:
    {
        UNPACK_DATA(arg, watch_sys_hr_skip_t, data_ind);
        /* Background HR-curve gap reason from LCPU -> forward to phone. */
        commu_send_heart_curve_skip(data_ind->timestamp, data_ind->reason);
#ifdef BSP_USING_BLOC_FILESYSTEM
        /* Persist so sleep gaps survive BLE disconnect (mirrors HR-sample). */
        health_store_hr_skip_async(data_ind->timestamp, data_ind->reason);
#endif
        break;
    }
    case MSG_SERVICE_WEAR_DIAG_IND:
    {
        UNPACK_DATA(arg, watch_sys_wear_diag_t, data_ind);
        /* Wear-detect diagnostic record from LCPU -> forward to phone (the
           phone appends to a daily CSV). Live-push only: records lost while
           BLE is down are acceptable for a diagnostic stream. */
        commu_send_wear_diag(data_ind->ts, data_ind->evt, data_ind->status,
                             data_ind->dc_q4, data_ind->pi_e6,
                             data_ind->pi_range_e6, data_ind->imu_var_e4);
        break;
    }
    case MSG_SERVICE_SLEEP_DIAG_IND:
    {
        UNPACK_DATA(arg, watch_sys_sleep_diag_t, data_ind);
        commu_send_sleep_diag(data_ind->ts, data_ind->score, data_ind->hr,
                              data_ind->hr_std, data_ind->stage, data_ind->veto,
                              data_ind->rhr, data_ind->worn, data_ind->rest,
                              data_ind->fresh, data_ind->total, data_ind->deep,
                              data_ind->rem, data_ind->light, data_ind->pi_e3,
                              data_ind->frame_pct, data_ind->rate_info,
                              data_ind->own_info, data_ind->rep_pct,
                              data_ind->accel_act);
        break;
    }
    case MSG_SERVICE_HR_CONT_IND:
    {
        UNPACK_DATA(arg, watch_sys_hr_cont_t, data_ind);
        hr_cont_enqueue(data_ind);
        break;
    }
    case MSG_SERVICE_HR_WINDOW_IND:
    {
        UNPACK_DATA(arg, watch_sys_hr_window_t, data_ind);
        commu_send_hr_window(data_ind->ts, data_ind->bpm, data_ind->conf,
                             data_ind->count, data_ind->win);
        break;
    }
    case MSG_SERVICE_SLEEP_STATE_IND:
    {
        UNPACK_DATA(arg, watch_sys_sleep_state_t, data_ind);
        set_watch_sleep_state(data_ind);
        break;
    }
    case MSG_SERVICE_DEBUG_LOG_IND:
    {
        UNPACK_DATA(arg, watch_sys_debug_log_t, data_ind);
        LOG_I(data_ind->log);
        break;
    }
    case MSG_SERVICE_MINUTE_ACTIVITY_IND:
    {
        UNPACK_DATA(arg, watch_sys_minute_activity_t, data_ind);
        SkaiWatchSys.activity = *data_ind;
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
#ifdef BSP_USING_PC_SIMULATOR
    /* PC sim has no LCPU peer, so every send to the watch-system service fails
       and then blocks the caller through the mdelay retry loop. Spamming it
       from gesture handlers (motor haptics on arc-scroll / bottom-bar) starves
       the GUI thread and takes the sim down. Pretend success and skip the RPC
       entirely — HCPU reads use cached globals (SkaiWatchSys.*), not replies. */
    (void)msg; (void)retry_delay_ms; (void)max_retries;
    return RT_EOK;
#else
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
    // else
    // {
    //     LOG_D("send code [%d] succeeded on first attempt", operation_code);
    // }

    return err;
#endif /* BSP_USING_PC_SIMULATOR */
}

/* Build a SYS_DATA_REQ from {cmd, optional body bytes after body[0]} and send
   it through the retry helper. The 17 LCPU-bound notify/request functions
   below all collapse to one or two lines via this. */
static rt_err_t send_sys_data_req(uint8_t cmd, const uint8_t *body_after_cmd,
                                   size_t body_len)
{
    data_msg_t msg;
    msg.body[0] = cmd;
    if (body_after_cmd && body_len > 0)
    {
        memcpy(&msg.body[1], body_after_cmd, body_len);
    }
    /* data_service_init_msg fills the message header; the body has already
       been populated above so we deliberately discard the returned pointer. */
    (void)data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    return send_watch_sys_msg_with_retry(&msg, 50, 1);
}

static rt_err_t send_sys_cmd_only(uint8_t cmd)
{
    return send_sys_data_req(cmd, NULL, 0);
}

static rt_err_t send_sys_cmd_b1(uint8_t cmd, uint8_t b1)
{
    return send_sys_data_req(cmd, &b1, 1);
}

static int request_battery_voltage(void)  { return send_sys_cmd_only(SysRequestBattery); }
static int request_charge_status(void)    { return send_sys_cmd_only(SysRequestChargeStatus); }
static int request_pedometer_data(void)   { return send_sys_cmd_only(SysRequestPedometerData); }
static int notify_system_wakeup(void)     { return send_sys_cmd_only(SysWakeUp); }
static int notify_calibration_global_attitude(void) { return send_sys_cmd_only(CalibrateGlobalAttitude); }

static int notify_system_standby(void)
{
    if (gui_app_is_actived(APP_ID_MESSAGE))
    {
        gui_app_exit(APP_ID_MESSAGE);
    }
    return send_sys_cmd_only(SysStandBy);
}

static int hr_power_manage(bool enable) { return send_sys_cmd_b1(PpgSensorPowerManage, enable ? 1 : 0); }

static int sync_api_lock(bool locked)
{
    rt_err_t err = send_sys_cmd_b1(SysSyncApiLock, locked ? 1 : 0);
    LOG_D("sync_api_lock:%d", locked);
    return err;
}

static bool tap_type = false;
void set_tap_type(bool type) { tap_type = type; }

/* These three preserve the original `RT_ASSERT(RT_EOK == err)` panic-on-fail
   contract — any other caller that doesn't carry the assert is using err
   only as a return value. */
static int notify_tap_detected(void)
{
    rt_err_t err = send_sys_cmd_b1(UserTapDetected, tap_type);
    RT_ASSERT(RT_EOK == err);
    return err;
}

static int notify_imu_data_collection(bool status)
{
    rt_err_t err = send_sys_cmd_b1(ImuDataCollection, status ? 1 : 0);
    RT_ASSERT(RT_EOK == err);
    return err;
}

static int notify_imu_rawdata_collection(bool status)
{
    rt_err_t err = send_sys_cmd_b1(ImuRawdataCollection, status ? 1 : 0);
    RT_ASSERT(RT_EOK == err);
    return err;
}

static int notify_user_profile(void)
{
    uint8_t b[4] = {
        SkaiWatchSys.user_data.user_profile.bit_field.gender,
        SkaiWatchSys.user_data.user_profile.bit_field.age,
        SkaiWatchSys.user_data.user_profile.bit_field.height,
        SkaiWatchSys.user_data.user_profile.bit_field.weight,
    };
    return send_sys_data_req(UserProfileUpdate, b, sizeof(b));
}

static int notify_debug_param_update(uint8_t value) { return send_sys_cmd_b1(DebugParamUpdate, value); }

static int control_motor(bool enable, motor_params_t *param)
{
    LOG_D("MOTOR control duty_cycle: %d, period: %d, repeat_times: %d",
          param->duty_cycle, param->period, param->repeat_times);
    /* When enable=false the legacy code wrote only body[1] and left
       body[2..7] uninitialized (LCPU side ignores them). Preserve that —
       send 1-byte body in that case to avoid leaking stack contents. */
    if (!enable)
    {
        return send_sys_cmd_b1(MotorControl, 0);
    }
    uint8_t b[7];
    b[0] = 1;
    b[1] = param->duty_cycle;
    memcpy(b + 2, &param->period, sizeof(rt_uint32_t));
    b[6] = param->repeat_times;
    return send_sys_data_req(MotorControl, b, sizeof(b));
}

static void set_debug_mode(bool enable)
{
    (void)send_sys_cmd_b1(DebugMode, enable ? 1 : 0);
}

static int set_multi_gesture_mode(bool enable) { return send_sys_cmd_b1(MultiGestureMode,  enable ? 1 : 0); }
static int set_tap_and_hold_mode(bool enable)  { return send_sys_cmd_b1(TapAndHoldMode,    enable ? 1 : 0); }
static int set_wear_detect_enable(bool enable) { return send_sys_cmd_b1(WearDetectEnable,  enable ? 1 : 0); }
static int set_hr_continuous(bool enable)      { return send_sys_cmd_b1(HrContinuousMode,  enable ? 1 : 0); }

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
    watch_sys_sync.set_wear_detect_enable = set_wear_detect_enable;
    watch_sys_sync.set_hr_continuous = set_hr_continuous;
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
