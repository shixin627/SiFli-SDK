#ifndef _WATCH_SYS_SERVICE_H_
#define _WATCH_SYS_SERVICE_H_
#include "board.h"
#include "rtconfig.h"
#include "data_service.h"
#include "bsp_board.h"
#include "bloc_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

    enum
    {
        MSG_SERVICE_SYS_DATA_REQ = (MSG_SERVICE_CUSTOM_ID_BEGIN),
        MSG_SERVICE_BATTERY_DATA_RSP =
            (MSG_SERVICE_SYS_DATA_REQ | RSP_MSG_TYPE),
        MSG_SERVICE_BATTERY_DATA_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 1) | RSP_MSG_TYPE),
        MSG_SERVICE_CHARGE_STATE_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 2) | RSP_MSG_TYPE),
        MSG_SERVICE_IMU_STATE_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 3) | RSP_MSG_TYPE),
        MSG_SERVICE_MAG_STATE_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 4) | RSP_MSG_TYPE),
        MSG_SERVICE_PPG_STATE_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 5) | RSP_MSG_TYPE),
        MSG_SERVICE_LIFT_IND = ((MSG_SERVICE_SYS_DATA_REQ + 6) | RSP_MSG_TYPE),
        MSG_SERVICE_SOFT_ADT_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 7) | RSP_MSG_TYPE),
        MSG_SERVICE_GESTURE_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 8) | RSP_MSG_TYPE),
        MSG_SERVICE_GESTURE_DATASET_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 9) | RSP_MSG_TYPE),
        MSG_SERVICE_GESTURE_PPG_DATASET_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 10) | RSP_MSG_TYPE),
        MSG_SERVICE_HEALTH_INFO_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 11) | RSP_MSG_TYPE),
        MSG_SERVICE_SLEEP_STATE_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 12) | RSP_MSG_TYPE),
        MSG_SERVICE_DEBUG_LOG_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 13) | RSP_MSG_TYPE),
        MSG_SERVICE_MINUTE_ACTIVITY_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 14) | RSP_MSG_TYPE),
    };

    typedef enum
    {
        SysStandBy,
        SysWakeUp,
        SysSyncApiLock,
        SysRequestBattery,
        SysRequestChargeStatus,
        PpgSensorPowerManage,
        UserTapDetected,
        ImuDataCollection,
        ImuRawdataCollection,
        CalibrateGlobalAttitude,
        UserProfileUpdate,
        SysRequestPedometerData,
        MotorControl,
        DebugMode,
        DebugParamUpdate,
        MultiGestureMode,
        TapAndHoldMode,
        RgbLedControl,
    } client_msg_t;

    typedef struct
    {
        uint32_t data;
    } watch_sys_service_data_rsp_t;

    typedef struct
    {
        uint32_t data;
    } watch_sys_service_data_ntf_ind_t;

    typedef struct
    {
        uint32_t data;
    } watch_sys_service_data_ind_t;

    typedef struct
    {
        uint32_t timestamp_s;
        uint16_t timestamp_ms;
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t gravity_x;
        int16_t gravity_y;
        int16_t gravity_z;
        uint16_t ppg_data;
        uint16_t fsr_adc_value;
        bool on_pressed;
    } watch_sys_linear_acce_t;

    typedef struct
    {
        watch_sys_linear_acce_t acce;
        uint16_t ppg;
    } watch_sys_linear_acce_ppg_t;

    typedef struct
    {
        uint32_t timestamp;
        uint16_t count;
        watch_sys_linear_acce_t acce[MAX_RAWDATA_TIME_STEP];
    } watch_sys_gesture_dataset_rsp_t;

    typedef struct
    {
        uint32_t timestamp;
        uint16_t count;
        watch_sys_linear_acce_ppg_t acce[16];
    } watch_sys_gesture_ppg_dataset_rsp_t;
    typedef struct
    {
        uint32_t steps;
        uint32_t distance;
        uint32_t calories;
    } watch_sys_heath_info_t;

    typedef struct
    {
        uint32_t total_seconds;
        uint32_t total_restful_seconds;
    } watch_sys_sleep_state_t;

    typedef struct
    {
        uint32_t rtc_time;
        char log[128];
    } watch_sys_debug_log_t;

    typedef struct
    {
        uint32_t utc_now;
        uint8_t steps;
        uint8_t orientation;
        uint16_t vmc;
    } watch_sys_minute_activity_t;

    /**
     * @brief RGB LED control parameters for inter-core communication
     * This structure is used to pass RGB LED control parameters between HCPU and LCPU
     */
    typedef struct
    {
        uint8_t enable;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        uint8_t brightness;
        uint8_t animation_mode;
        uint16_t period_ms;
        uint16_t repeat_times;
    } watch_sys_rgb_led_params_t;

    typedef struct
    {
#if defined(SOC_BF0_HCPU)
        bool (*is_imu_enabled)(void);
        // bool (*is_mag_enabled)(void);
        bool (*is_ppg_enabled)(void);
        // hcpu->lcpu request functions
        int (*request_battery_voltage)(void);
        int (*request_charge_status)(void);
        int (*request_pedometer_data)(void);
        int (*sync_api_lock)(bool locked);
        int (*notify_system_standby)(void);
        int (*notify_system_wakeup)(void);
        int (*hr_power_manage)(bool status);
        int (*notify_tap_detected)(void);
        int (*notify_imu_data_collection)(bool status);
        int (*notify_imu_rawdata_collection)(bool status);
        int (*notify_calibration_global_attitude)(void);
        int (*notify_user_profile)(void);
        int (*notify_debug_param_update)(uint8_t value);
        int (*control_motor)(bool enable, motor_params_t *param);
        void (*set_debug_mode)(bool mode);
        int (*set_multi_gesture_mode)(bool enable);
        int (*set_tap_and_hold_mode)(bool enable);
        int (*control_rgb_led)(watch_sys_rgb_led_params_t *params);
#else
    // lcpu->hcpu notify functions
    void (*notify_battery_voltage)(uint32_t data);
    void (*charge_status_callback)(int status);
    void (*lift_status_callback)(uint8_t status);
    void (*soft_adt_status_callback)(bool status);
    void (*notify_gesture_event)(uint32_t gesture);
    void (*notify_health_info)(void);
    void (*notify_sleep_state)(uint8_t mode, uint32_t timestamp);
    void (*notify_minute_of_activity)(time_t utc_now, uint8_t steps,
                                      uint8_t orientation, uint16_t vmc);
    void (*notify_debug_log)(char *log);
#endif
    } watch_sys_sync_t;

    extern void WATCH_LCPU_LOG_DEBUG(const char *format, ...);

#define WATCHSYS_SERVICE_NAME "WATCHSYS"

    extern watch_sys_sync_t watch_sys_sync;
    int SubscribeDualCoreSyncService(void);
    int UnsubscribeDualCoreSyncService(void);

#ifdef __cplusplus
}
#endif

#endif /* _WATCH_SYS_SERVICE_H_ */
