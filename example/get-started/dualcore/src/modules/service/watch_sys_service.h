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
        MSG_SERVICE_SLEEP_DIAG_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 18) | RSP_MSG_TYPE),
        MSG_SERVICE_HR_CONT_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 19) | RSP_MSG_TYPE),
        MSG_SERVICE_HR_WINDOW_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 20) | RSP_MSG_TYPE),
        MSG_SERVICE_HR_RAW_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 21) | RSP_MSG_TYPE),
        MSG_SERVICE_HR_BURST_IND =
            ((MSG_SERVICE_SYS_DATA_REQ + 22) | RSP_MSG_TYPE),
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
        /* Diagnostic: run HR the way the Exercise app does — PPG held on and the
           HBA algorithm never re-initialised — instead of bg_hr's cold-started
           bursts. Both regimes open the identical sensor mode (POWER_HIGH ->
           GH30X_FUNCTION_HR @ 25 Hz), so this isolates ONE variable: continuity.
           If the nightly 2x disappears here, the doubling is a cold-start
           acquisition failure and is fixable by changing the regime; if it
           persists, the regime is innocent. */
        HrContinuousMode,
        /* HCPU -> LCPU: restore the battery gauge's SOC after a reset. body[1]
           carries the percentage (1..100; 0 means "nothing stored"). The gauge
           state lives in LCPU RAM, which is re-loaded and zeroed on every
           reset. */
        SysSeedBatterySoc,
        /* HCPU -> LCPU: may the HR diagnostic streams emit? body[1] 1 = yes,
           0 = no, which is also what LCPU boots with. Re-asserted on every
           periodic tick like WearDetectEnable, so a silent LCPU reboot
           converges back to the persisted value instead of losing it. */
        HrDiagCapture,
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
        /* What wear detection believed AT THE MOMENT this sample was taken.
           WATCH_SYS_WORN_* below. Added 2026-08-31: a watch face-down on a
           desk keeps producing heart rate, and until now nothing downstream
           could tell those readings apart from a wrist's — 2 h 16 min of desk
           "heart rate" reached the phone on 2026-08-31 with no marker at all.
           Read here rather than from the HCPU mirror because that mirror is
           set true on wrist-raise and never set back to false. */
        uint8_t  worn;
    } watch_sys_hr_sample_t;

    /* Tri-state, because "we don't know" must not collapse into "worn".
       UNKNOWN is what a backfilled sample carries: the watch's own
       /health/hr_*.json has no wear column, so replayed points genuinely
       cannot say. The phone must render UNKNOWN as ordinary, not as off-wrist. */
#define WATCH_SYS_WORN_NO       0u
#define WATCH_SYS_WORN_YES      1u
#define WATCH_SYS_WORN_UNKNOWN  0xFFu

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
        /* The per-session learned DC baseline, same /4 scaling as dc_q4 so the
           two compare directly. Added 2026-08-31: every ON/OFF decision is
           dc vs a factor of THIS, and it was the one quantity the diagnostics
           never carried — leaving "has the baseline been dragged onto the
           surface the watch is resting on?" unanswerable from data. */
        uint16_t base_q4;
    } watch_sys_wear_diag_t;

    /* Per-minute sleep-fusion diagnostic record (LCPU -> HCPU). HCPU forwards
       it via KEY_SLEEP_DIAG; cable-less units have no serial console, so this
       is the only window into why a night was scored the way it was. hr_std is
       the candidate PPG signal-quality (SQI/jitter) input. Temporary. */
    typedef struct
    {
        uint32_t ts;       /* UTC second of the minute eval               */
        uint16_t score;    /* Cole-Kripke activity score (last)           */
        uint8_t  hr;       /* minute HR fed to fusion (0 = absent)        */
        uint8_t  hr_std;   /* burst HR std — SQI candidate (jitter)       */
        uint8_t  stage;    /* sleep_fusion_stage_t 0..4                   */
        uint8_t  veto;     /* HR wake-veto active this minute (0/1)       */
        uint8_t  rhr;      /* learned resting HR                          */
        uint8_t  worn;     /* is_worn (0/1)                               */
        uint8_t  rest;     /* rest-candidate dense gate active (0/1)      */
        uint8_t  fresh;    /* HR this minute was a fresh burst (0/1)      */
        uint16_t total;    /* daily total_sleep_min accumulator (L+D+R)   */
        uint16_t deep;     /* daily deep_min                              */
        uint16_t rem;      /* daily rem_min                               */
        uint16_t light;    /* daily light_min                             */
        uint16_t pi_e3;    /* last PPG burst PI*1000 (AC/DC) — SQI candidate */
        uint16_t rate_info; /* (hr divider << 8) | algo-frames-%. The divider is
                             * chip_rate/25 recomputed per burst from an I2C read;
                             * divider 2 with the chip at 25 Hz feeds the algo
                             * 12.5 Hz while it believes 25 -> exactly DOUBLE for
                             * that whole burst. frame_pct cannot see this (it
                             * counts upstream of the divider).                  */
        /* (hr_autocorr best confidence << 8) | longest identical raw-PPG run.
           Confidence answers the question left open on 2026-08-05, when four
           isolated outliers (171/144/101/38 bpm) could not be classified as
           "low confidence — raise the gate" versus "confident and wrong — fix
           the rule", because it only ever reached the LCPU console. The run
           length proves — rather than infers from reading code — that the
           staircase seen in the gesture-collection stream is absent from the HR
           path: a 17-bit ADC on live tissue does not repeat a sample by chance,
           so anything above 1 is real. */
        uint16_t own_info;
        uint8_t  rep_pct;   /* % of raw samples identical to their predecessor */
        uint16_t accel_act; /* wrist activity of the estimated window: mean
                             * |d(accel)|/sample summed over 3 axes. Gates the
                             * motion compensation, and is the only off-watch
                             * evidence that the accelerometer reference is
                             * alive at all — it silently was not for a night. */
        uint16_t frame_pct; /* last PPG burst: delivered frames as % of 25 Hz *
                             * burst seconds. The HBA algo assumes 25 Hz and the
                             * samples are untimestamped, so a shortfall here is
                             * a proportional HR over-read: ~50% would explain
                             * the nightly 2x episodes as lost frames rather
                             * than physiology. ~100% clears the timebase.     */
    } watch_sys_sleep_diag_t;

/* Continuous-HR diagnostic batch (LCPU -> HCPU -> phone via KEY_HR_CONT_DIAG).
   Buffered on the watch and flushed once a minute rather than streamed per
   sample: 1 Hz of individual BLE notifies all night would be both wasteful and
   a different power profile from the Exercise app we are trying to imitate. */
/* 30 x 1 Hz = one flush every 30 s. Was 60; halved when the per-sample record
   grew from 3 bytes to 7, to keep the BLE payload (6 + 7*N) far inside
   MAX_PACKET_PAYLOAD_SIZE (507) rather than merely under it. */
#define WATCH_SYS_HR_CONT_MAX 30

/* One captured hr_autocorr window (LCPU -> HCPU -> KEY_HR_WINDOW_DUMP). */
#define WATCH_SYS_HR_WIN_MAX 256
/* Paired wrist movement over the SAME window, decimated 4:1 (see
   hr_autocorr_last_accel). 256+64+11 = 331 bytes, inside MAX_PACKET_PAYLOAD_SIZE. */
#define WATCH_SYS_HR_ACC_MAX 64

    typedef struct
    {
        uint32_t ts;                       /* when the estimate was produced    */
        uint8_t  bpm;                      /* the implausible value             */
        uint8_t  conf;                     /* its confidence 0..100             */
        uint16_t count;                    /* samples in win[]                  */
        uint16_t acc_count;                /* entries in acc[], 0 = none        */
        uint8_t  acc_shift;                /* acc[] << this = raw LSB units     */
        int8_t   win[WATCH_SYS_HR_WIN_MAX];/* detrended PPG, oldest first       */
        int8_t   acc[WATCH_SYS_HR_ACC_MAX];/* |x|+|y|+|z| about its own mean    */
    } watch_sys_hr_window_t;

/* The SAME window at full precision, plus the transform needed to invert it
   back to raw sensor counts (@ref hr_autocorr_last_work):

       raw[i] = ((fit_a_q16 + fit_b_q16 * i) >> 16) + (win[i] << shift)

   with i the index in the whole window, i.e. first_index + position in win[].

   Sent in chunks because 256 int16 does not fit one BLE frame. Two chunks of
   128 make each message 272 bytes -- SMALLER than the int8 record above, which
   is already in the field, so this adds no new size risk on either the
   cross-core queue or the BLE link. Every chunk repeats the fit so the offline
   side never depends on chunk order or on both chunks arriving. */
#define WATCH_SYS_HR_RAW_CHUNK 128

    typedef struct
    {
        uint32_t ts;                       /* matches the paired window record  */
        int64_t  fit_a_q16;                /* intercept of the removed line     */
        int64_t  fit_b_q16;                /* slope, per sample                 */
        uint16_t first_index;              /* win[0] is this index of the window*/
        uint16_t count;                    /* entries in win[]                  */
        uint8_t  shift;                    /* win[] << this = counts about fit  */
        int16_t  win[WATCH_SYS_HR_RAW_CHUNK];
    } watch_sys_hr_raw_t;

/* One finished burst, @ref KEY_HR_BURST_SUMMARY. 24 bytes -- an order of
   magnitude under the window records already in the field. */
    typedef struct
    {
        uint32_t ts;                       /* burst end, watch wall-clock       */
        uint32_t dur_ms;                   /* incl. extensions                  */
        uint32_t samples;                  /* PPG frames that reached OUR ring  */
        uint16_t reads;                    /* 1 Hz sensor reads attempted       */
        uint16_t readfail;                 /* of those, failed                  */
        uint16_t frame_pct;                /* chip delivered / expected, %      */
        uint16_t rate_info;                /* divider<<8 | algo delivered %     */
        uint8_t  extends;                  /* @ref BGHR_EXTEND_MAX              */
        uint8_t  best;                     /* median BPM, 0 = never locked      */
        uint8_t  reason;                   /* wire reason, 0 = published        */
        uint8_t  power_veto;               /* mode switches refused this burst  */
    } watch_sys_hr_burst_t;

    typedef struct
    {
        uint32_t base_ts;                  /* UTC second of sample[0]           */
        uint8_t  interval_s;               /* seconds between samples (1)       */
        uint8_t  count;                    /* valid entries in the arrays       */
        uint8_t  bpm[WATCH_SYS_HR_CONT_MAX];      /* 0 = algo produced nothing  */
        uint8_t  qscore[WATCH_SYS_HR_CONT_MAX];   /* Goodix valid_score  0-100  */
        uint8_t  qlevel[WATCH_SYS_HR_CONT_MAX];   /* Goodix valid_level  0-2    */
        /* (acc_info << 5) | acc_scene — the ALGORITHM's own motion state (0 rest /
           1 walk / 2 run) and scene id (hba_scenes_e, 0..24, fits in 5 bits). If it
           classifies a motionless 4 a.m. as a running scene, that alone explains the
           high plateaus, and unlike the confidence fields these are outputs the lib
           very likely populates. */
        uint8_t  accst[WATCH_SYS_HR_CONT_MAX];
        /* Wrist motion for THIS second (bghr_accel_delta, >>10 LSB). sleep_diag's
           Cole-Kripke score is per-minute and window-smoothed — useless for asking
           what happened in the ten seconds a plateau began. */
        uint8_t  accel[WATCH_SYS_HR_CONT_MAX];
        /* Perfusion index x1000 over this one second (AC/DC of the raw PPG). The
           only in-band signal-quality measure still alive on this lib, so it is the
           leading candidate both for the MECHANISM (does perfusion collapse just
           before the algorithm goes wrong?) and for a usable gate. */
        uint16_t pi_e3[WATCH_SYS_HR_CONT_MAX];
    } watch_sys_hr_cont_t;

    typedef struct
    {
#if defined(SOC_BF0_HCPU)
        bool (*is_imu_enabled)(void);
        // bool (*is_mag_enabled)(void);
        bool (*is_ppg_enabled)(void);
        // hcpu->lcpu request functions
        int (*request_battery_voltage)(void);
        int (*request_charge_status)(void);
        int (*seed_battery_soc)(uint8_t percent);
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
        int (*set_hr_continuous)(bool enable);
        int (*set_hr_diag)(bool enable);
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
    void (*notify_sleep_diag)(const watch_sys_sleep_diag_t *rec);
    /* Fire-and-forget to HCPU. A BLE outage is absorbed by HCPU's backlog ring
       (watch_system_client.c): losing every disconnected minute is worse than
       useless here, because the hole looks identical to "the sensor produced
       nothing" — which is exactly how the 2026-08-02 night lost 65 minutes. */
    void (*notify_hr_cont)(const watch_sys_hr_cont_t *rec);
    void (*notify_hr_window)(const watch_sys_hr_window_t *rec);
    void (*notify_hr_raw)(const watch_sys_hr_raw_t *rec);
    void (*notify_hr_burst)(const watch_sys_hr_burst_t *rec);
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
