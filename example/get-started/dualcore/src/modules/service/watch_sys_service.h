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
        MSG_SERVICE_HR_SAMPLE_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 15) | RSP_MSG_TYPE),
        MSG_SERVICE_HR_SKIP_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 16) | RSP_MSG_TYPE),
        MSG_SERVICE_WEAR_DIAG_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 17) | RSP_MSG_TYPE),
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
        WearDetectEnable,
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

    /* Sleep state IND payload (LCPU -> HCPU).
       `mode` is a T_SLEEP_STATUS-compatible value:
         0x01 TSLEEP, 0x02 TDEEP_SLEEP, 0x03 TGET_UP, 0x04 TNOT_WEAR,
         0x05 TREM_SLEEP (extension introduced by sleep_fusion).
       Sent on each stage transition. Daily aggregates are included so
       the HCPU receiver can render a summary without keeping its own
       state machine. */
    typedef struct
    {
        uint8_t  mode;
        uint8_t  reserved[3];
        uint32_t timestamp_utc;        /* UTC second of this transition  */
        uint16_t total_sleep_min;      /* light + deep + rem today       */
        uint16_t deep_min;
        uint16_t rem_min;
        uint16_t light_min;
        uint16_t awake_after_onset_min; /* WASO today                    */
        uint8_t  current_hr;            /* 0 if unknown                  */
        uint8_t  resting_hr;
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

    /* Background daily HR-curve sample (LCPU -> HCPU). One timestamped ambient
       HR reading taken ~every 15 min by the LCPU sampler; HCPU forwards it to
       the phone via KEY_HEART_CURVE_SAMPLE to build a daily heart-rate curve. */
    typedef struct
    {
        uint32_t timestamp; /* UTC second of the sample */
        uint8_t  bpm;       /* heart rate; 0 = invalid  */
    } watch_sys_hr_sample_t;

    /* Background HR-curve GAP reason (LCPU -> HCPU). Emitted once per 5-min
       bucket that produced no HR point; HCPU forwards via KEY_HEART_CURVE_SKIP
       and persists it so sleep gaps survive BLE disconnect. */
    typedef struct
    {
        uint32_t timestamp; /* UTC second of the 5-min bucket start */
        uint8_t  reason;    /* wire reason code 1..7; see ADR-0011 D2 */
    } watch_sys_hr_skip_t;

    /* Wear-detect diagnostic event codes (LCPU -> HCPU -> phone, frozen —
       the dart CSV writer names them; do not reorder). */
    typedef enum
    {
        WEAR_DIAG_EVT_ON = 1,            /* wear state -> WEARING            */
        WEAR_DIAG_EVT_OFF = 2,           /* wear state -> NOT_WEARING        */
        WEAR_DIAG_EVT_SAMPLE = 3,        /* periodic eval snapshot (~60s)    */
        WEAR_DIAG_EVT_PROBE_OPEN = 4,    /* motion opened PPG probe window   */
        WEAR_DIAG_EVT_PROBE_EXPIRE = 5,  /* probe expired without a wrist    */
        WEAR_DIAG_EVT_BREAK_ARM = 6,     /* contact break + motion armed     */
        WEAR_DIAG_EVT_BREAK_CONFIRM = 7, /* pulse re-confirmed after break   */
        WEAR_DIAG_EVT_BREAK_TIMEOUT = 8, /* re-confirm ran out of live evals */
        /* HR-burst quality summary (emitted by hr_service per bg_hr burst).
           TEMPORARY instrumentation (ADR 0016) to settle, without serial,
           whether the algo's hba_confi/hba_snr hold data while valid_* read 0.
           Reuses the 14-byte wire with REPURPOSED fields (no scaling):
             status      = valid_level (0..2)
             dc_q4       = reads
             pi_e6       = accepted reads
             pi_range_e6 = hba_confi x100   (was qual_rej; gate off so it's 0)
             imu_var_e4  = hba_snr   x100   (was valid_score; known 0)
           Read from the phone CSV: reads=dc/4, acc=pi*1e6, qlvl=status,
           hba_confi=(pi_range*1e6)/100, hba_snr=(imu_var*1e4)/100. Remove after. */
        WEAR_DIAG_EVT_HR_BURST = 9,
    } watch_sys_wear_diag_evt_t;

    /* Wear-detect diagnostic record (LCPU -> HCPU). HCPU forwards it via
       KEY_WEAR_DIAG; cable-less units have no serial console so this is the
       only window into nightly wear-detect internals. */
    typedef struct
    {
        uint32_t ts;           /* UTC second of the event                  */
        uint8_t  evt;          /* watch_sys_wear_diag_evt_t                */
        uint8_t  status;       /* wear status at emit time (0 off / 1 on)  */
        uint16_t dc_q4;        /* PPG DC mean / 4                          */
        uint16_t pi_e6;        /* PI * 1e6, clamped to 65535               */
        uint16_t pi_range_e6;  /* PI range * 1e6, clamped                  */
        uint16_t imu_var_e4;   /* IMU variance * 1e4, clamped              */
    } watch_sys_wear_diag_t;

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
        int (*set_wear_detect_enable)(bool enable);
#else
    // lcpu->hcpu notify functions
    void (*notify_battery_voltage)(uint32_t data);
    void (*charge_status_callback)(int status);
    void (*lift_status_callback)(uint8_t status);
    void (*soft_adt_status_callback)(bool status);
    void (*notify_gesture_event)(uint32_t gesture);
    void (*notify_health_info)(void);
    void (*notify_hr_sample)(uint32_t timestamp, uint8_t bpm);
    void (*notify_hr_skip)(uint32_t timestamp, uint8_t reason);
    void (*notify_wear_diag)(const watch_sys_wear_diag_t *rec);
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
