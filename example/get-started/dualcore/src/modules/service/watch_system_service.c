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
#ifdef BSP_USING_ACTIVITY_ALGO_KRAEPELIN
    #include "activity.h"
    #include "activity_private.h"
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
bool get_locked_status(void)
{
    return gesture_lock;
}

static bool _multi_gesture_mode = false;
bool is_multi_gesture_mode(void)
{
    return _multi_gesture_mode;
}

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

#ifdef BSP_USING_ACTIVITY_ALGO_KRAEPELIN
    watch_sys_heath_info_t data_ind = {
        .steps    = activity_private_state()->step_data.steps,
        .distance = activity_private_state()->distance_mm,
        .calories = activity_private_state()->active_calories,
    };
    push_msg_to_hcpu(MSG_SERVICE_HEALTH_INFO_IND, &data_ind, sizeof(data_ind));
#endif
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
            bloc_battery_read_voltage();
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
#ifdef BSP_USING_ACTIVITY_ALGO_KRAEPELIN
            activity_prefs_set_gender(msg->body[1]);
            activity_prefs_set_age_years(msg->body[2]);
            activity_prefs_set_height_mm(msg->body[3] * 10);
            activity_prefs_set_weight_dag(msg->body[4] * 100);
#endif
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

        case RgbLedControl:
        {
            // Extract parameters from message body into structured format
            watch_sys_rgb_led_params_t params;
            params.enable = msg->body[1];
            params.red = msg->body[2];
            params.green = msg->body[3];
            params.blue = msg->body[4];
            params.brightness = msg->body[5];
            params.animation_mode = msg->body[6];
            params.period_ms = *(rt_uint16_t *)(msg->body + 7);
            params.repeat_times = *(rt_uint16_t *)(msg->body + 9);

            LOG_D("RgbLedControl: enable=%d, R=%d, G=%d, B=%d, brightness=%d, "
                  "mode=%d, period=%d, repeat=%d",
                  params.enable, params.red, params.green, params.blue,
                  params.brightness, params.animation_mode, params.period_ms,
                  params.repeat_times);
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
    watch_sys_sync.notify_minute_of_activity = notify_minute_of_activity;
    watch_sys_sync.notify_debug_log = notify_debug_log;
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
