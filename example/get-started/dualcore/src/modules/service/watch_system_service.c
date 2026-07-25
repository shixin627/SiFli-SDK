/**
 ******************************************************************************
 * @file   watch_system_service.c
 * @author Skaiwalk software development team
 * @brief  LCPU 系統服務層。透過 data_service 框架向 HCPU 推送電池電壓、
 *         充電狀態、抬手/手勢事件、健康資訊、活動記錄等通知，並處理來自
 *         HCPU 的系統指令（休眠/喚醒、感測器電源、IMU 校準、馬達控制等）。
 ******************************************************************************
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <sensor.h>
#include "acce_service.h"
#include "hr_service.h"
#include "data_service_provider.h"
#include "watch_sys_service.h"
#include "bloc_battery.h"
#ifdef ACC_USING_BMI270
    #include "bmi270_driver.h"  /* HW step counter API */
#endif
#ifdef BSP_USING_SLEEP_FUSION
    #include "sleep_fusion.h"   /* daily aggregates filled into IND */
    /* hr_service.h is already included above for hr_service_get_latest_bpm */
#endif
#ifdef BSP_USING_SLEEP_VANHEES
    #include "sleep_vanhees.h" /* accel-only sleep-period daily aggregates */
#endif
#ifdef BSP_USING_WEAR_DETECT
    #include "wear_detect.h"   /* wear_detect_set_enabled (diagnostic override) */
#endif

#define DBG_TAG "watch_sys"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef MSG_SEND_INTERVAL_MS
static rt_tick_t last_msg_send_time = 0;

static bool can_send_message(void)
{
    rt_tick_t current_time = rt_tick_get();
    rt_tick_t interval_ticks = rt_tick_from_millisecond(MSG_SEND_INTERVAL_MS);

    if (current_time - last_msg_send_time >= interval_ticks)
    {
        last_msg_send_time = current_time;
        return true;
    }
    return false;
}
#endif

typedef struct
{
    int ref_count;
    rt_device_t device;
    datas_handle_t service;

    uint16_t data;
} watch_sys_service_env_t;

extern void set_imu_data_collection(bool enable);
extern void set_imu_rawdata_collection(bool enable);

watch_sys_sync_t watch_sys_sync;
static watch_sys_service_env_t watch_sys_service_env;
static int32_t watch_sys_service_msg_handler(datas_handle_t service,
                                             data_msg_t *msg);
static data_service_config_t watch_sys_service_cb = {
    .max_client_num = 1,
    .queue = RT_NULL, /* share the same queue of data service process thread */
    .data_filter = NULL,
    .msg_handler = watch_sys_service_msg_handler,
};

static bool gesture_lock = true;
static bool _multi_gesture_mode = false;

/* Push an arbitrary blob as a service IND message. No-op if service is not
   yet registered; asserts on push failure (queue full / wrong cb). */
static void push_msg_to_hcpu(uint16_t msg_id, const void *data, size_t len)
{
    if (watch_sys_service_env.service == NULL) return;
    int32_t r = datas_push_msg_to_client(watch_sys_service_env.service,
                                          msg_id, (uint16_t)len,
                                          (uint8_t *)data);
    RT_ASSERT(0 == r);
}

/* Common single-uint32 IND payload (battery / charge / lift / soft-adt /
   gesture all share watch_sys_service_data_ind_t with one .data field). */
static void push_uint32_ind(uint16_t msg_id, uint32_t data)
{
    watch_sys_service_data_ind_t data_ind = { .data = data };
    push_msg_to_hcpu(msg_id, &data_ind, sizeof(data_ind));
}

static void notify_battery_voltage(uint32_t data)
{
    if (watch_sys_service_env.service == NULL)
        return;
    rt_err_t err = datas_data_ready(watch_sys_service_env.service, sizeof(data),
                                    (uint8_t *)data);
    RT_ASSERT(RT_EOK == err);
}

static void indicate_battery_voltage(uint32_t data)
{
#ifdef MSG_SEND_INTERVAL_MS
    if (!can_send_message()) return;
#endif
    push_uint32_ind(MSG_SERVICE_BATTERY_DATA_IND, data);
}

static void charge_status_callback(int status)
{
#ifdef MSG_SEND_INTERVAL_MS
    if (!can_send_message()) return;
#endif
    push_uint32_ind(MSG_SERVICE_CHARGE_STATE_IND, (uint32_t)status);
}

static void lift_status_callback(uint8_t status)
{
#ifdef MSG_SEND_INTERVAL_MS
    if (!can_send_message()) return;
#endif
    push_uint32_ind(MSG_SERVICE_LIFT_IND, status);
}

static void soft_adt_status_callback(bool status)
{
    push_uint32_ind(MSG_SERVICE_SOFT_ADT_IND, (uint32_t)status);
    LOG_I("Wear detect: %s", status ? "ON WRIST" : "OFF WRIST");

    /* Wear-gate PPG hardware: with the watch off-wrist there's nothing for
       the LED to measure, so kill the LED until the user puts it back on.
       The ref-counted timer in hr_service keeps running (so re-subscribers
       still work), but reads return nothing while powered down. On
       re-wear, the hardware comes back up only if there's still a
       subscriber — otherwise we'd waste LED current with no consumer. */
    if (status)
    {
        if (is_ppg_service_ready())
        {
            /* Re-power only if someone is actively waiting on HR data.
               hr_subscribe/unsubscribe is the source of truth here. */
            if (hr_service_subscriber_count() > 0)
            {
                hr_set_power(1);
            }
        }
    }
    else
    {
        if (is_ppg_service_ready())
        {
            hr_set_power(0);
        }
    }
}

static void notify_gesture_event(uint32_t gesture)
{
    if (is_sleep_mode()) return;
#ifdef MSG_SEND_INTERVAL_MS
    if (!can_send_message()) return;
#endif
    push_uint32_ind(MSG_SERVICE_GESTURE_IND, gesture);
}

static void notify_health_info(void)
{
    if (is_sleep_mode()) return;
#ifdef MSG_SEND_INTERVAL_MS
    if (!can_send_message()) return;
#endif

#ifdef ACC_USING_BMI270
    /* Step count is read straight from the BMI270 on-chip step counter
       (the SW Kraepelin path is disabled). Distance / calories are not
       computed here yet — set to 0 until we plumb in stride-based
       estimation. */
    uint32_t hw_steps = 0;
    if (bmi270_hw_step_counter_read(&hw_steps) != 0)
    {
        return; /* chip read failed; skip this push */
    }
    watch_sys_heath_info_t data_ind = {
        .steps    = hw_steps,
        .distance = 0,
        .calories = 0,
    };
    push_msg_to_hcpu(MSG_SERVICE_HEALTH_INFO_IND, &data_ind, sizeof(data_ind));
#endif
}

static void notify_hr_sample(uint32_t timestamp, uint8_t bpm)
{
    /* Deliberately NOT gated by is_sleep_mode(): a daily HR curve is sampled
       mostly while the screen is OFF, so we must forward background samples in
       DARK too. The ~15 min cadence makes the HCPU wake negligible. */
    if (bpm == 0) return;
    watch_sys_hr_sample_t data_ind = {
        .timestamp = timestamp,
        .bpm = bpm,
    };
    push_msg_to_hcpu(MSG_SERVICE_HR_SAMPLE_IND, &data_ind, sizeof(data_ind));
}

static void notify_hr_skip(uint32_t timestamp, uint8_t reason)
{
    /* Mirrors notify_hr_sample: forward a 5-min bucket's gap reason to HCPU,
       which sends KEY_HEART_CURVE_SKIP and persists it. Not gated by
       is_sleep_mode() — sleep is exactly when these gaps matter most. */
    watch_sys_hr_skip_t data_ind = {
        .timestamp = timestamp,
        .reason = reason,
    };
    push_msg_to_hcpu(MSG_SERVICE_HR_SKIP_IND, &data_ind, sizeof(data_ind));
}

static void notify_wear_diag(const watch_sys_wear_diag_t *rec)
{
    /* Mirrors notify_hr_skip: forward a wear-detect diagnostic record to HCPU,
       which sends KEY_WEAR_DIAG. Not gated by is_sleep_mode() — night is
       exactly when these records matter (cable-less per-unit diagnosis). */
    if (rec == NULL) return;
    push_msg_to_hcpu(MSG_SERVICE_WEAR_DIAG_IND, rec, sizeof(*rec));
}

static void notify_sleep_diag(const watch_sys_sleep_diag_t *rec)
{
    /* Mirrors notify_wear_diag: forward a per-minute sleep-fusion diagnostic to
       HCPU, which sends KEY_SLEEP_DIAG. Not gated by is_sleep_mode() — night is
       exactly when these records matter (cable-less per-unit diagnosis). */
    if (rec == NULL) return;
    push_msg_to_hcpu(MSG_SERVICE_SLEEP_DIAG_IND, rec, sizeof(*rec));
}

static void notify_sleep_state(uint8_t mode, uint32_t timestamp_utc)
{
    watch_sys_sleep_state_t data_ind;
    memset(&data_ind, 0, sizeof(data_ind));
    data_ind.mode = mode;
    data_ind.timestamp_utc = timestamp_utc;
#ifdef BSP_USING_SLEEP_FUSION
    const sleep_fusion_output_t *out = sleep_fusion_current();
    if (out)
    {
        data_ind.total_sleep_min       = out->total_sleep_min;
        data_ind.deep_min              = out->deep_min;
        data_ind.rem_min               = out->rem_min;
        data_ind.light_min             = out->light_min;
        data_ind.awake_after_onset_min = out->awake_after_onset_min;
        data_ind.resting_hr            = out->last_hr_baseline_bpm;
    }
#ifdef BSP_USING_HR_SVC
    data_ind.current_hr = hr_service_get_latest_bpm();
#endif
#endif
#ifdef BSP_USING_SLEEP_VANHEES
    const sleep_vanhees_output_t *vh = sleep_vanhees_current();
    if (vh)
    {
        data_ind.total_sleep_min       = vh->total_sleep_min;
        data_ind.light_min             = vh->total_sleep_min; /* undifferentiated: no Deep/REM */
        data_ind.awake_after_onset_min = vh->awake_after_onset_min;
    }
#endif
    push_msg_to_hcpu(MSG_SERVICE_SLEEP_STATE_IND, &data_ind, sizeof(data_ind));
}

static void notify_minute_of_activity(time_t utc_now, uint8_t steps,
                                      uint8_t orientation, uint16_t vmc)
{
    if (is_sleep_mode()) return;
    watch_sys_minute_activity_t data_ind = {
        .utc_now     = utc_now,
        .steps       = steps,
        .orientation = orientation,
        .vmc         = vmc,
    };
    push_msg_to_hcpu(MSG_SERVICE_MINUTE_ACTIVITY_IND, &data_ind, sizeof(data_ind));
}

extern time_t get_current_time(void);
static void notify_debug_log(char *log)
{
    watch_sys_debug_log_t data_ind;
    data_ind.rtc_time = get_current_time();
    strncpy(data_ind.log, log, sizeof(data_ind.log));
    push_msg_to_hcpu(MSG_SERVICE_DEBUG_LOG_IND, &data_ind, sizeof(data_ind));
}

static bool debug_mode = false;
void WATCH_LCPU_LOG_DEBUG(const char *format, ...)
{
    if (!debug_mode)
        return;
    char log_buffer[128];
    memset(log_buffer, 0, 128);
    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer, 128, format, args);
    va_end(args);
    watch_sys_sync.notify_debug_log(log_buffer);
}

uint8_t gesture_threshold_factor = 50;
static void update_debug_param(uint8_t value)
{
    gesture_threshold_factor = value;
}
static rt_tick_t last_hcpu_wakeup_time = 0;
bool is_hcpu_wakeup_in_last_3s(void)
{
    return rt_tick_get_millisecond() - last_hcpu_wakeup_time < 3000;
}
static int32_t watch_sys_service_msg_handler(datas_handle_t service,
                                             data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_SYS_DATA_REQ:
    {
        uint8_t header = msg->body[0];
        LOG_D("watch_sys_service_msg_handler: SYS_DATA_REQ header=%d", header);
        switch (header)
        {

        case SysStandBy:
        {
            LOG_I("System Stand by");
            /* Order matters: acce_set_power(LOW) ends by re-enabling INT1
               (bmi270_sensor_power_low_mode). set_sleep_mode() fires the
               on_lcpu_sleep_mode_changed hook, which in HW wrist-wake mode
               wants the final say on IRQ state — so it must run AFTER the
               acce power transition. */
            acce_set_power(RT_SENSOR_POWER_LOW);
            set_sleep_mode(true);
        }
        break;

        case SysWakeUp:
        {
            LOG_I("System Wake up");
            last_hcpu_wakeup_time = rt_tick_get_millisecond();
            /* Defer the battery sample: reading the GPADC the instant we wake,
               before its analog front-end settles, occasionally returns an
               implausibly low value that then sticks until the next sample. */
            bloc_battery_read_voltage_after_settle();
            acce_set_power(RT_SENSOR_POWER_HIGH);
            set_sleep_mode(false);
        }
        break;

        case SysSyncApiLock:
        {
            uint8_t locked = msg->body[1];
            gesture_lock = locked;
            break;
        }

        case SysRequestBattery:
        {
            bloc_battery_read_voltage();
            break;
        }

        case SysRequestChargeStatus:
        {
            bloc_battery_read_charge_status();
            break;
        }

        case PpgSensorPowerManage:
        {
            if (!battery_get_charge_state()->is_plugged)
            {
                hr_set_power(msg->body[1]);
            }
            break;
        }

        case UserTapDetected:
        {
            LOG_I("User Tap Detected");
            tap_detected_callback(msg->body[1]);
        }
        break;

        case ImuDataCollection:
        {
            LOG_I("IMU Data Collection");
            set_imu_data_collection(msg->body[1] == 1);
        }
        break;

        case ImuRawdataCollection:
        {
            LOG_I("IMU Raw Data Collection");
            set_imu_rawdata_collection(msg->body[1] == 1);
        }
        break;

        case CalibrateGlobalAttitude:
        {
            LOG_I("Calibrate Global Attitude");
            calibrate_global_attitude();
        }
        break;

        case UserProfileUpdate:
        {
            LOG_I("User Profile Update");
            break;
        }

        case SysRequestPedometerData:
        {
            notify_health_info();
            break;
        }

        case MotorControl:
        {
            if (msg->body[1] == 1)
            {
                motor_params_t motor_param;
                motor_param.duty_cycle = msg->body[2];
                motor_param.period = *(rt_uint32_t *)(msg->body + 3);
                motor_param.repeat_times = msg->body[7];
                if (motor_provider.start_motor)
                {
                    motor_provider.start_motor(&motor_param);
                }
            }
            else
            {
                if (motor_provider.stop_motor)
                {
                    motor_provider.stop_motor();
                }
            }
            break;
        }

        case DebugMode:
        {
            uint8_t mode = msg->body[1];
            debug_mode = (mode == 1);
            break;
        }

        case DebugParamUpdate:
        {
            update_debug_param(msg->body[1]);
            break;
        }

        case MultiGestureMode:
        {
            uint8_t mode = msg->body[1];
            _multi_gesture_mode = (mode == 1);
            break;
        }

        case TapAndHoldMode:
        {
            uint8_t mode = msg->body[1];
            set_enable_tap_and_hold(mode == 1);
            break;
        }

        case WearDetectEnable:
        {
#ifdef BSP_USING_WEAR_DETECT
            wear_detect_set_enabled(msg->body[1] == 1);
#endif
            break;
        }

        default:
            break;
        }
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_rdy_ind =
            (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        int32_t result;
        watch_sys_service_data_ntf_ind_t data_ntf_ind;

        RT_ASSERT(data_rdy_ind);

        data_ntf_ind.data = (uint32_t)data_rdy_ind->data;
        result = datas_push_data_to_client(service, sizeof(data_ntf_ind),
                                           (uint8_t *)&data_ntf_ind);
        RT_ASSERT(0 == result);
        break;
    }
    default:
    {
        LOG_E("watch_sys_service_msg_handler: unknown msg_id %d", msg->msg_id);
        break;
    }
    }

    return 0;
}

static void register_watch_sys_service_funs(void)
{
    watch_sys_sync.notify_battery_voltage = indicate_battery_voltage;
    watch_sys_sync.charge_status_callback = charge_status_callback;
    watch_sys_sync.lift_status_callback = lift_status_callback;
    watch_sys_sync.soft_adt_status_callback = soft_adt_status_callback;
    watch_sys_sync.notify_gesture_event = notify_gesture_event;
    watch_sys_sync.notify_health_info = notify_health_info;
    watch_sys_sync.notify_hr_sample = notify_hr_sample;
    watch_sys_sync.notify_hr_skip = notify_hr_skip;
    watch_sys_sync.notify_wear_diag = notify_wear_diag;
    watch_sys_sync.notify_minute_of_activity = notify_minute_of_activity;
    watch_sys_sync.notify_sleep_state = notify_sleep_state;
    watch_sys_sync.notify_debug_log = notify_debug_log;
    watch_sys_sync.notify_sleep_diag = notify_sleep_diag;
}

static int watch_sys_service_register(void)
{
    /* register battery voltage service*/
    watch_sys_service_env.service =
        datas_register(WATCHSYS_SERVICE_NAME, &watch_sys_service_cb);
    RT_ASSERT(watch_sys_service_env.service);
    register_watch_sys_service_funs();
    return 0;
}
INIT_COMPONENT_EXPORT(watch_sys_service_register);

#if !kReleaseMode
static int test_watch_sys_service(int argc,
                                  char *argv[]) // test_watch_sys_service ble_bool
{
    if (argc < 2)
    {
        LOG_I("Usage: test_watch_sys_service <option>\n");
        return -1;
    }
    if (strcmp(argv[1], "notify_voltage") == 0)
    {
        if (argc == 3)
        {
            uint32_t loc_voltage = atoi(argv[2]);
            notify_battery_voltage(loc_voltage);
        }
    }
    else if (strcmp(argv[1], "indicate_voltage") == 0)
    {
        if (argc == 3)
        {
            uint32_t loc_voltage = atoi(argv[2]);
            indicate_battery_voltage(loc_voltage);
        }
    }
    else if (strcmp(argv[1], "ble_bool") == 0)
    {
        LOG_D("ble_bool command received.");
        extern int close_acce_service(void);
        close_acce_service();
        extern int close_hr_service(void);
        close_hr_service();
    }
    return 0;
}
MSH_CMD_EXPORT(test_watch_sys_service, "test battery service")
#endif
