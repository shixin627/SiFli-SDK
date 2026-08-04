/**
 ******************************************************************************
 * @file   bloc_motion_tracking.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "app_mainmenu.h"
#ifdef BSP_USING_AIR_MOUSE
    #include "air_mouse.h"
#endif
#include "watch_global_data.h"
#ifdef BSP_USING_COMMUNICATE
    #include "communicate_protocol.h"
    #include "communicate_task.h"
#endif
#include "ui_handler.h"
#include "watch_system_interact.h"
#ifdef BSP_USING_BLOC
    #include "bloc_peripheral.h"
    #include "bloc_control.h"
    #include "bloc_skaiwalk.h"
    #include "watch_system_interact.h"
    #include "bloc_motion_tracking.h"
    #include "bloc_v2t.h"
    #include "bloc_setting.h"
#endif

#define DBG_TAG "bloc.motion_tracking"
#include "bsp_board.h"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "gui_app_pm.h" /* always — gui_is_active() gates the IMU consumer */

// ============================================================================
// Waveform Capture Configuration (migrated from LCPU gesture_detect.c)
// ============================================================================
#define ENABLE_WAVEFORM_CAPTURE 1

#if ENABLE_WAVEFORM_CAPTURE
    /* Sliding window and threshold constants.
       ACCEL_WINDOW_SIZE must hold GESTURE_PEAK_SEARCH + the larger
       FEEDBACK_ACCEL_SAMPLES_* so the peak-aligned back-fill below can reach
       back past the peak once the peak is confirmed (10 + 10 = 20 → 24). */
    #define ACCEL_WINDOW_SIZE 24
    #define GYRO_WINDOW_SIZE 10
    #define GYRO_LOCK_THRESHOLD 2000.0f
    #define GESTURE_RELEASE_COOLDOWN_PERIOD_MS 100
    #define GESTURE_COLLECTION_COOLDOWN_PERIOD_MS 500
    #define GESTURE_TAP_COOLDOWN_PERIOD_MS 100
    #define MAX_GESTURE_DURATION_MS 160
    #define MIN_GESTURE_SAMPLES 10
    #define MIN_DIFFERENCE_ACCEL_MAX 1.0f

    #define FEEDBACK_ACCEL_SAMPLES_FOR_TAP 9
    #define FEEDBACK_ACCEL_SAMPLES_FOR_RELEASE 10
    #define RELEASE_START_THRESHOLD 1.0f
    #define TAP_START_THRESHOLD 0.3f

    /* ── Peak-aligned capture (2026-07-29) ────────────────────────────────
       Replaces "threshold crossing starts an exclusive 35-sample collection".
       Two measured problems with that scheme, both verified against a walking
       capture whose touch_adc gives ground-truth tap times:

       1. The window was anchored on the THRESHOLD CROSSING, not on the event
          peak. Crossing→peak latency is itself variable (0-78 ms measured), so
          the real tap landed at window index 9..24 — the 2nd-stage model saw a
          different alignment every time.
       2. Collection was exclusive: once started it ran 35 samples + cooldown
          with no re-evaluation, so a stronger real tap arriving mid-window was
          invisible. Walking spent 57% of wall-clock inside such windows.

       New scheme: arm on the threshold, then keep tracking — ANY stronger peak
       re-anchors the window (GESTURE_REALIGN_ON_STRONGER). The window is only
       cut once the peak has been quiet for GESTURE_PEAK_SEARCH samples, and is
       back-filled from the sliding window so the peak always lands at index
       FEEDBACK_ACCEL_SAMPLES_* — a fixed alignment.

       Measured on the walking capture (ground truth = touch_adc press edges):
         current: 21 windows, 57% occupancy, 2/3 taps caught, index [18,-,24]
         new:     15 windows, 41% occupancy, 3/3 taps caught, index [9,9,9]
       Still capture regression-checked: identical window count/occupancy, all
       6 events caught, alignment [9,9,17]/[18,17,17] → [9,9,9]/[9,9,9]. */
    #define GESTURE_PEAK_SEARCH 10   /* samples of quiet needed to confirm peak */
    #define GESTURE_ARM_TIMEOUT 80   /* give up if no window in 800 ms          */
    #define GESTURE_ADAPTIVE_K 5.0f  /* thr = max(fixed, median × K)            */
    #define GESTURE_ADAPTIVE_MEDIAN_SAMPLES 75
    #define GESTURE_CONTEXT_WALK_MEDIAN 0.03f /* median above this ≈ walking    */

// Gesture types
typedef enum
{
    GESTURE_TYPE_TAP,
    GESTURE_TYPE_RELEASE,
} gesture_type_t;

// Gesture state structure for waveform capture
typedef struct
{
    Vector3 sliding_window_accel[ACCEL_WINDOW_SIZE];
    Vector3 sliding_window_gravity[ACCEL_WINDOW_SIZE];
    uint32_t sliding_window_ppg[ACCEL_WINDOW_SIZE];
    rt_uint32_t sliding_window_fsr_adc[ACCEL_WINDOW_SIZE];
    float gyro_sliding_window[GYRO_WINDOW_SIZE];
    uint8_t gyro_count;
    bool if_watchface_visible;
    bool gyro_lock_status;
    float difference_accel;
    bool on_pressed;
} waveform_gesture_state_t;

/* Per-extractor state for the peak-aligned capture. TAP and RELEASE are two
   independent extractors (game mode runs both at once), so this cannot live in
   the shared waveform_gesture_state_t. */
typedef struct
{
    bool armed;
    uint16_t arm_age;  /* samples since arming                     */
    uint16_t peak_age; /* samples since the current peak           */
    float peak_val;    /* difference_accel at the current peak     */
    uint8_t realigns;  /* re-anchors within this arm (diagnostic)  */
    /* Background as it was when this window ARMED. The "is the wearer moving
       too much to trust a gesture" gate has to judge the background BEFORE the
       gesture: by the time the window closes, the running median has been
       lifted by the gesture itself, so a re-computed median makes a strong
       gesture look like violent motion and throws it away. Measured on-device:
       a real press (fsr 5221, finger confirmed down) closed with med=0.13 and
       0.37 and was dropped by the 0.25 gate. */
    float arm_median;
    float arm_threshold; /* threshold in force at arm time (for the log) */
} capture_state_t;

// Static variables for waveform capture
static gesture_dataset_t tap_dataset = {0};
static gesture_dataset_t release_dataset = {0};
static capture_state_t tap_capture = {0};
static capture_state_t release_capture = {0};
static waveform_gesture_state_t waveform_gesture_state = {0};

#if !kReleaseMode
/* Diagnostic override (MSH `gcap both`) — run BOTH extractors regardless of
   mode, so a capture session can see TAP and RELEASE windows side by side on
   any screen. Deliberately NOT app_control_set_game_mode(): that one also
   fires watch_sys_sync.set_multi_gesture_mode() across to the LCPU, which
   would change behaviour beyond the thing being measured. This flag is read
   in exactly one place (the extractor dispatch) and does nothing else. */
static bool capture_both_override = false;

/* Peak (×100) at or above which a CUT line is tagged " BIG". Walking crosses
   the 0.3 tap threshold ~11% of the time, so the phone's live log scrolls
   constantly and real gestures are lost in it — filtering the phone log on
   "BIG" pulls them back out. Measured still-capture separation was clean
   (real 145..550 vs noise 32..71); walking taps run smaller, hence adjustable
   via `gcap pk <n>` rather than hard-coded. */
static uint16_t gesture_big_pk = 100;
#endif

/* Peak-confirm threshold (×100), applied ONLY while the background says the
   wearer is walking. Arm/capture is untouched — this gates whether the cut
   window is worth an inference pass.
   Measured (walking capture, touch_adc + PPG ground truth): real gestures peak
   at 0.83/0.92/1.41/3.95/4.63/5.46, noise at 0.35..1.33. A 0.80 confirm drops
   noise 9 → 3 with all six real gestures kept. Standing still needs NO confirm
   (that capture had zero false windows) and would lose 4 of 6 at 0.80, hence
   the walking-only gating.
   NOT hard-coded to 80: the margin to the weakest real gesture is only 4% on a
   single 6-gesture capture. Runtime-tunable via `gcap confirm <n>`, default 0
   (off) until more captures pin it down. Lives outside !kReleaseMode because
   it changes real behaviour, not just diagnostics. */
static uint16_t gesture_confirm_pk = 0;

/* Exposed for the developer screen so the value can be changed ON THE WATCH —
   the MSH route needs a UART cable, which is exactly what a walking test
   can't have. */
uint16_t gesture_confirm_get(void)
{
    return gesture_confirm_pk;
}

void gesture_confirm_set(uint16_t peak_x100)
{
    gesture_confirm_pk = peak_x100;
}

#if !kReleaseMode

/* Per-window diagnostic logging (the CUT / GST lines), OFF by default. It is
   genuinely noisy — walking crosses the tap threshold ~11% of the time, so
   this emits roughly one line per second per extractor, on both UART and the
   BLE link to the phone. Turn on only for a capture session: `gcap log on`. */
static bool capture_log_on = false;

/* Read by gesture_recognition_task.c so one switch covers both stages. */
bool gesture_capture_log_on(void)
{
    return capture_log_on;
}
#endif
static watch_sys_linear_acce_t targetWave_algo[MAX_RAWDATA_TIME_STEP];

static float difference_accel_sliding_window[MAX_GESTURE_SAMPLES] = {0.0f};
static int difference_accel_count = 0;
static float prev_linear_accel_resultant = 0.0f;
static bool user_hand_horizontal = false;

// Forward declarations for waveform capture functions
static void waveform_capture_process(motion_data_t *motion_data, Vector3 *gyro);
#endif // ENABLE_WAVEFORM_CAPTURE

#define MOTION_TRACKING_THREAD_STACK_SIZE 2 * 1024
#define MOTION_TRACKING_THREAD_PRIORITY 17
#define MOTION_TRACKING_THREAD_TIMESLICE 10

#define STARTED_MOVE_THRESHOLD                                                 \
    4.0f // 當角度變化超過此值時，表示使用者已經開始控制手臂

static rt_thread_t motion_tracking_thread = RT_NULL;

static euler_angle_t motion_tracking_algorithm(Quaternion *quaternion,
                                               Quaternion *prev_quat);
static void send_quaternion_to_ble_client(rt_uint32_t ts, Quaternion *q);
static Quaternion multiply_quaternions(Quaternion *q1, Quaternion *q2);

/* Cross-module externs (defined in sibling bloc translation units). */
extern bool get_enable_tap_and_hold(void);
extern bool get_is_open_instruction_list_ai(void);
extern bool message_media_widget_focused(void);

static bool stop_mouse_move = false;

#ifdef BSP_USING_AIR_MOUSE
static void air_mouse_process(rt_uint32_t ts, Quaternion *quaternion,
                              Quaternion *prev_quat);
#endif

static float total_yaw_energy = 0;
/* 體感滾動：每格固定 6 度（與列表長度無關）。control_angle 隨 count 等比
 * 放大到 6×count，使每格角度 = control_angle / count = DEGREES_PER_SLOT。 */
#define DEGREES_PER_SLOT 6
static uint8_t scroll_segment_count = 1;
static uint16_t page_range = 100; // 每個頁面的範圍
static float total_moving_distance = 1100.0f;
static uint8_t control_angle = 60; // 進任何列表前的 fallback；進場後由 set_scroll_segment_count 覆寫為 6×count
float get_total_moving_distance(void)
{
    return total_moving_distance;
}
void set_scroll_segment_count(uint8_t count)
{
    if (count > 0 && count <= 11)
    {
        scroll_segment_count = count;
        // page_range = total_moving_distance / scroll_segment_count; //
        total_moving_distance = scroll_segment_count * page_range;
        // 每個頁面的範圍
        // page_range = 125;
        /* 固定 6 度/格：control_angle 是「yaw 角度→energy」換算的分母
         * (total_yaw_energy_uint = -yaw × total_moving_distance / control_angle)，
         * 隨 count 等比放大後每格角度 = control_angle / count = 6 度，不隨列表長度變。
         * count ≤ 11（上面的 gate）→ 6×count ≤ 66，不超出 uint8_t。 */
        control_angle = DEGREES_PER_SLOT * count;
    }
    LOG_D("set_scroll_segment_count:%d,%d", scroll_segment_count, page_range);
}

static Quaternion prev_sensor_quat = {.w = 1, .x = 0, .y = 0, .z = 0};
static motion_data_t *watch_sensor_motion_data = {0};

// ============================================================================
// Waveform Capture Helper Functions (migrated from LCPU gesture_detect.c)
// ============================================================================
#if ENABLE_WAVEFORM_CAPTURE

static double total_acceleration_magnitude(double x, double y, double z)
{
    return sqrt(x * x + y * y + z * z);
}

/**
 * @brief Converts sliding window acceleration data to target waveform
 */
static bool check_ppg_error = false;
static void
getTargetWaveformFromSlidingWindow(gesture_dataset_t *dataset,
                                   watch_sys_linear_acce_t *targetWave,
                                   int sample_len)
{
    for (uint8_t i = 0; i < sample_len; i++)
    {
        targetWave[i].timestamp_s = dataset->timestamp_s[i];
        targetWave[i].timestamp_ms = dataset->timestamp_ms[i];
        targetWave[i].x =
            (float)(*(dataset->waveform[i] + 0)) * INT16_to_G / GRAVITY;
        targetWave[i].y =
            (float)(*(dataset->waveform[i] + 1)) * INT16_to_G / GRAVITY;
        targetWave[i].z =
            (float)(*(dataset->waveform[i] + 2)) * INT16_to_G / GRAVITY;
        targetWave[i].gravity_x =
            (float)(*(dataset->waveform[i] + 3)) * INT16_to_DPS;
        targetWave[i].gravity_y =
            (float)(*(dataset->waveform[i] + 4)) * INT16_to_DPS;
        targetWave[i].gravity_z =
            (float)(*(dataset->waveform[i] + 5)) * INT16_to_DPS;
        targetWave[i].fsr_adc_value = dataset->fsr_adc_value[i];
        targetWave[i].ppg_data = dataset->ppg_data[i];
        targetWave[i].on_pressed = dataset->on_pressed[i];
    }

    // Check if PPG data is stuck alternating between only two values
    if (sample_len >= 4)
    {
        uint16_t val_a = dataset->ppg_data[0];
        uint16_t val_b = val_a;
        bool found_second = false;
        bool is_stuck = true;

        for (uint8_t i = 1; i < sample_len; i++)
        {
            uint16_t v = dataset->ppg_data[i];
            if (!found_second && v != val_a)
            {
                val_b = v;
                found_second = true;
            }
            else if (v != val_a && v != val_b)
            {
                is_stuck = false;
                break;
            }
        }

        if (is_stuck && found_second)
        {
            // LOG_D("PPG stuck: alternating between %d and %d (%d samples)",
            //       val_a, val_b, sample_len);
            check_ppg_error = true;
        }
        else
        {
            // LOG_D("PPG data looks good");
            check_ppg_error = false;
        }
    }
}

static int cooldown_period = 0;
static rt_tick_t wait_start_time = 0;
extern bool imu_data_collection;
static void reset_gesture_state(gesture_dataset_t *dataset,
                                uint32_t current_time, gesture_type_t type,
                                uint8_t code)
{
    if (dataset->gesture_sample_count == 0)
    {
        return;
    }
    dataset->gesture_started = false;
    dataset->gesture_ended = false;
    dataset->gesture_sample_count = 0;
    wait_start_time = current_time;
    if (imu_data_collection)
    {
        cooldown_period = 600;
    }
    else
    {
        if (type == GESTURE_TYPE_RELEASE)
        {
            cooldown_period = GESTURE_COLLECTION_COOLDOWN_PERIOD_MS;
        }
        else
        {
            cooldown_period = GESTURE_TAP_COOLDOWN_PERIOD_MS;
        }
    }
    // LOG_D("Reset gesture state, code: %d,%d,%d", code,cooldown_period,
    // current_time);
}

static uint16_t waveform_rtc_millisecond = 0;
static void store_gesture_sample(gesture_dataset_t *dataset, time_t ts,
                                 Vector3 *linear_accel, Vector3 *gravity,
                                 uint32_t ppg_data, rt_uint32_t fsr_adc_value,
                                 bool on_pressed)
{
    if (dataset->gesture_sample_count < MAX_RAWDATA_TIME_STEP)
    {
        dataset->waveform[dataset->gesture_sample_count][0] = linear_accel->x;
        dataset->waveform[dataset->gesture_sample_count][1] = linear_accel->y;
        dataset->waveform[dataset->gesture_sample_count][2] = linear_accel->z;
        dataset->waveform[dataset->gesture_sample_count][3] = gravity->x;
        dataset->waveform[dataset->gesture_sample_count][4] = gravity->y;
        dataset->waveform[dataset->gesture_sample_count][5] = gravity->z;
        dataset->timestamp_s[dataset->gesture_sample_count] = ts;
        dataset->timestamp_ms[dataset->gesture_sample_count] =
            waveform_rtc_millisecond;
        dataset->ppg_data[dataset->gesture_sample_count] = ppg_data;
        dataset->fsr_adc_value[dataset->gesture_sample_count] = fsr_adc_value;
        dataset->on_pressed[dataset->gesture_sample_count] = on_pressed;
        dataset->gesture_sample_count++;
    }
    // LOG_D("acc[%d]: [%0.3f, %0.3f, %0.3f], gravity: [%0.3f, %0.3f, %0.3f],
    // ppg: %d, on_pressed: %d",
    //           dataset->gesture_sample_count, linear_accel->x,
    //           linear_accel->y, linear_accel->z, gravity->x, gravity->y,
    //           gravity->z, ppg_data, on_pressed);
}

static void fill_realtime_accel_sliding_window(Vector3 *accel, Vector3 *gravity,
                                               uint32_t ppg,
                                               rt_uint32_t fsr_adc_value,
                                               waveform_gesture_state_t *state)
{
    for (int i = 0; i < ACCEL_WINDOW_SIZE - 1; i++)
    {
        state->sliding_window_accel[i] = state->sliding_window_accel[i + 1];
        state->sliding_window_gravity[i] = state->sliding_window_gravity[i + 1];
        state->sliding_window_ppg[i] = state->sliding_window_ppg[i + 1];
        state->sliding_window_fsr_adc[i] = state->sliding_window_fsr_adc[i + 1];
    }
    state->sliding_window_accel[ACCEL_WINDOW_SIZE - 1] = *accel;
    state->sliding_window_gravity[ACCEL_WINDOW_SIZE - 1] = *gravity;
    state->sliding_window_ppg[ACCEL_WINDOW_SIZE - 1] = ppg;
    state->sliding_window_fsr_adc[ACCEL_WINDOW_SIZE - 1] = fsr_adc_value;
    // LOG_D("Updated sliding window with ppg: %d",ppg);
}

static bool check_gyro_threshold(Vector3 *gyro, waveform_gesture_state_t *state)
{
    float gyro_magnitude =
        sqrtf(gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);
    state->gyro_sliding_window[state->gyro_count] = gyro_magnitude;
    state->gyro_count = (state->gyro_count + 1) % GYRO_WINDOW_SIZE;
    float total_gyro = 0.0f;
    for (int i = 0; i < GYRO_WINDOW_SIZE - 1; i++)
    {
        total_gyro += state->gyro_sliding_window[i];
    }
    bool gyro_threshold_exceeded = total_gyro > GYRO_LOCK_THRESHOLD;
    state->gyro_lock_status = gyro_threshold_exceeded;
    return gyro_threshold_exceeded;
}

// Calculate median of difference_accel_sliding_window
static float calculate_median_difference_accel(uint8_t check_samples)
{
    float temp_array[MAX_GESTURE_SAMPLES];
    int loop_limit = check_samples;

    for (int i = 0; i < loop_limit; i++)
    {
        int index;
        if (i < difference_accel_count)
        {
            index = difference_accel_count - 1 - i;
        }
        else
        {
            int wrap_around_offset = i - difference_accel_count;
            index = MAX_GESTURE_SAMPLES - 1 - wrap_around_offset;
        }
        temp_array[i] = difference_accel_sliding_window[index];
    }

    // Simple bubble sort
    for (int i = 0; i < loop_limit - 1; i++)
    {
        for (int j = 0; j < loop_limit - i - 1; j++)
        {
            if (temp_array[j] > temp_array[j + 1])
            {
                float temp = temp_array[j];
                temp_array[j] = temp_array[j + 1];
                temp_array[j + 1] = temp;
            }
        }
    }

    float median;
    if (loop_limit % 2 == 0)
    {
        median = (temp_array[loop_limit / 2 - 1] + temp_array[loop_limit / 2]) /
                 2.0f;
    }
    else
    {
        median = temp_array[loop_limit / 2];
    }

    return median;
}

/**
 * @brief Packs accelerometer matrix data into a byte buffer (little-endian).
 *
 * Layout per sample (BYTES_PER_SAMPLE = 22):
 *   [0..3]   timestamp_s   (u32)
 *   [4..5]   timestamp_ms  (u16)
 *   [6..7]   x             (i16)
 *   [8..9]   y             (i16)
 *   [10..11] z             (i16)
 *   [12..13] gravity_x     (i16)
 *   [14..15] gravity_y     (i16)
 *   [16..17] gravity_z     (i16)
 *   [18..19] ppg_data      (u16)
 *   [20..21] fsr_adc_value (u16)
 *
 * `fft_buffer` is unused in the current path (all callers pass NULL); kept
 * for API compatibility.
 */
void packMatrixToBuffer(uint8_t *targetArray, watch_sys_linear_acce_t *dataset,
                        int32_t *fft_buffer, int sample_len)
{
    if (fft_buffer != NULL)
    {
        return;
    }
    for (uint8_t i = 0; i < sample_len; i++)
    {
        uint8_t *p = &targetArray[i * BYTES_PER_SAMPLE];
        memcpy(p + 0, &dataset[i].timestamp_s, sizeof(dataset[i].timestamp_s));
        memcpy(p + 4, &dataset[i].timestamp_ms,
               sizeof(dataset[i].timestamp_ms));
        memcpy(p + 6, &dataset[i].x, sizeof(dataset[i].x));
        memcpy(p + 8, &dataset[i].y, sizeof(dataset[i].y));
        memcpy(p + 10, &dataset[i].z, sizeof(dataset[i].z));
        memcpy(p + 12, &dataset[i].gravity_x, sizeof(dataset[i].gravity_x));
        memcpy(p + 14, &dataset[i].gravity_y, sizeof(dataset[i].gravity_y));
        memcpy(p + 16, &dataset[i].gravity_z, sizeof(dataset[i].gravity_z));
        memcpy(p + 18, &dataset[i].ppg_data, sizeof(dataset[i].ppg_data));
        memcpy(p + 20, &dataset[i].fsr_adc_value,
               sizeof(dataset[i].fsr_adc_value));
    }
}

/**
 * @brief Notify gesture recognition task with dataset
 * Directly fills watch_sensor.gesture_data and releases gesture_sem
 */
static void notify_gesture_dataset_hcpu(uint32_t timestamp, int count,
                                        watch_sys_linear_acce_t *data)
{
    watch_sensor.gesture_data.timestamp = timestamp;
    watch_sensor.gesture_data.sample_num = count;
    memcpy(watch_sensor.gesture_data.dataset, data,
           count * sizeof(watch_sys_linear_acce_t));

    if (watch_sensor.gesture_sem)
    {
        rt_sem_release(watch_sensor.gesture_sem);
        LOG_D("Gesture dataset ready, samples: %d", count);
    }
}

/**
 * @brief Main gesture event capture function for HCPU
 */
static uint32_t prev_ppg_rawdata[3] = {0};
static bool open_ppg_chacked = false;
void set_open_ppg_chacked(bool checked)
{
    open_ppg_chacked = checked;
}
bool get_open_ppg_chacked(void)
{
    return open_ppg_chacked;
}

/**
 * @brief Report one cut window on the UART log (COM14, tools/dev_console).
 *
 * One line per emitted window so a live capture session can be checked
 * event-by-event ("did it catch all three taps?"). Floats are scaled ×100 and
 * printed as integers — ulog's formatter has no %f.
 *
 *   CUT TAP pk=141 thr=36 med=7 now=13 fsr=3100 ra=1 age=21 SENT
 *        │      │      │     │    │      │     │    │    └ SENT/DROP-med/DROP-pose
 *        │      │      │     │    │      │     │    └ samples from arm to emit
 *        │      │      │     │    │      │     └ re-anchors onto a stronger peak
 *        │      │      │     │    │      └ FSR ADC at emit — ground truth for
 *        │      │      │     │    │        "was the finger down": ~16000 resting,
 *        │      │      │     │    │        drops hard on a real press, so a high
 *        │      │      │     │    │        fsr here means that window was noise
 *        │      │      │     │    └ background median ×100 AT EMIT — compare
 *        │      │      │     │      against med to see how much the gesture
 *        │      │      │     │      itself lifted the running median
 *        │      │      │     └ background median ×100 AT ARM (what the gate
 *        │      │      │       judges; walking ≈ 7, still ≈ 1)
 *        │      │      └ threshold in force AT ARM ×100 (adaptive)
 *        │      └ peak difference_accel ×100
 *        └ TAP / REL
 *
 * thr/med are sampled at ARM, not at emit — reporting the recomputed values was
 * what made a `pk=148 thr=185` line look self-contradictory.
 *
 * Dev builds only — compiled out in release.
 */
static void gesture_capture_report(gesture_type_t type, capture_state_t *cap,
                                   float median_now, rt_uint32_t fsr_adc_value,
                                   bool is_gesture, bool posture_ok,
                                   bool confirm_ok)
{
#if !kReleaseMode
    if (!capture_log_on)
    {
        return;
    }
    /* Formatted once, emitted on BOTH transports. UART (COM12) is for bench
       tests; BLE is the only viewer that MOVES WITH THE WEARER, which is what
       a walking test needs — you can't drag a UART cable along. The phone
       ring-buffers 200 lines, so the log can be read after the walk rather
       than watched during it. Local buffer, not wristband_ble_log.c's shared
       one: that has no lock and is documented as BLE-rx-context only, while
       this runs on the motion-tracking thread. */
    char line[96];
    int pk100 = (int)(cap->peak_val * 100.0f);
    rt_snprintf(line, sizeof(line),
                "CUT %s pk=%d thr=%d med=%d now=%d fsr=%d ra=%d age=%d %s%s",
                (type == GESTURE_TYPE_TAP) ? "TAP" : "REL", pk100,
                (int)(cap->arm_threshold * 100.0f),
                (int)(cap->arm_median * 100.0f), (int)(median_now * 100.0f),
                (int)fsr_adc_value, cap->realigns, cap->arm_age,
                !is_gesture      ? "DROP-med"
                : !posture_ok    ? "DROP-pose"
                : !confirm_ok    ? "DROP-confirm"
                                 : "SENT",
                (pk100 >= (int)gesture_big_pk) ? " BIG" : "");
    LOG_I("%s", line);
    #ifdef BSP_USING_COMMUNICATE
    commu_send_bluetooth_log(line);
    #endif
#else
    (void)type; (void)cap; (void)median_now;
    (void)fsr_adc_value; (void)is_gesture; (void)posture_ok;
#endif
}

extern int get_gesture_recognition_threshold(void);
static void gesture_event_capture_hcpu(uint16_t freq, time_t ts,
                                       Vector3 *linear_acce, Vector3 *gyro,
                                       Vector3 *gravity, uint32_t ppg,
                                       rt_uint32_t fsr_adc_value,
                                       waveform_gesture_state_t *state,
                                       gesture_type_t type,
                                       gesture_dataset_t *dataset,
                                       capture_state_t *cap)
{
    rt_tick_t current_time = rt_tick_get_millisecond();

    uint32_t ppg_diff_rawdata =
        abs((int32_t)ppg - (int32_t)prev_ppg_rawdata[2]);

    for (int i = 0; i < 2; i++)
    {
        prev_ppg_rawdata[i] = prev_ppg_rawdata[i + 1];
    }
    prev_ppg_rawdata[2] = ppg;
    if ((current_time - wait_start_time) < cooldown_period)
    {
        return;
    }
    if (current_time - SkaiWatchSys.pre_hcpu_wakeup_tick < 300)
    {
        return;
    }
    else if (state->if_watchface_visible == false && !imu_data_collection)
    {
        cap->armed = false; /* posture gate — drop any in-flight arm too */
        reset_gesture_state(dataset, current_time, type, 2);
        return;
    }
    else if (state->gyro_lock_status && !imu_data_collection)
    {
        cap->armed = false; /* gyro lock — same */
        reset_gesture_state(dataset, current_time, type, 3);
        return;
    }

    int target_samples = 0;
    float start_threshold = 0.0f;
    int feedback_samples = 0;
    if (type == GESTURE_TYPE_TAP)
    {
        target_samples = GESTURE_TAP_TIME_STEP;
        start_threshold = TAP_START_THRESHOLD;
        feedback_samples = FEEDBACK_ACCEL_SAMPLES_FOR_TAP;
    }
    else if (type == GESTURE_TYPE_RELEASE)
    {
        target_samples = GESTURE_RELEASE_TIME_STEP;
        start_threshold = RELEASE_START_THRESHOLD;
        feedback_samples = FEEDBACK_ACCEL_SAMPLES_FOR_RELEASE;
    }

    /* Adaptive threshold — the fixed constant is a FLOOR, the running
       background median lifts it while the wearer moves. Measured background
       median: 0.013 (still) vs 0.071 (walking), so the same K leaves the still
       working point untouched (0.013 × 5 = 0.065 < 0.3, floor wins) while
       lifting the walking one to ~0.36. Walking window count drops 21 → 15
       with no loss of real taps; K = 8 starts dropping the weakest tap. */
    float bg_median =
        calculate_median_difference_accel(GESTURE_ADAPTIVE_MEDIAN_SAMPLES);
    float trigger_threshold = bg_median * GESTURE_ADAPTIVE_K;
    if (trigger_threshold < start_threshold)
    {
        trigger_threshold = start_threshold;
    }

    bool ppg_triggered =
        (ppg_diff_rawdata > (uint32_t)get_gesture_recognition_threshold() &&
         open_ppg_chacked);
    bool accel_triggered =
        (waveform_gesture_state.difference_accel > trigger_threshold &&
         !open_ppg_chacked);

    /* ── Idle: wait for a threshold crossing to ARM ───────────────────────
       Arming stores nothing. The window is only cut once the peak is known,
       so that the peak always lands at the same index (see below). */
    if (!cap->armed)
    {
        if (!accel_triggered && !ppg_triggered)
        {
            return;
        }
        if (ppg_triggered)
        {
            LOG_D("PPG DIFF");
        }
        cap->armed = true;
        cap->arm_age = 0;
        cap->peak_age = 0;
        cap->peak_val = waveform_gesture_state.difference_accel;
        cap->realigns = 0;
        cap->arm_median = bg_median;
        cap->arm_threshold = trigger_threshold;
        return;
    }

    /* ── Armed: keep evaluating every sample. ────────────────────────────── */
    cap->arm_age++;
    cap->peak_age++;

    if (waveform_gesture_state.difference_accel > cap->peak_val)
    {
        /* A stronger sample than the current anchor — re-anchor onto it. This
           is what makes a real tap arriving inside a noise-triggered window
           visible again: the stronger event takes the window instead of being
           swallowed by the weaker one that happened to fire first. */
        cap->peak_val = waveform_gesture_state.difference_accel;
        cap->peak_age = 0;
        if (dataset->gesture_sample_count > 0)
        {
            dataset->gesture_sample_count = 0;
            dataset->gesture_started = false;
            cap->realigns++;
        }
    }

    /* Continuous motion could keep bumping the peak forever — bail out rather
       than stay armed indefinitely. No cooldown: nothing was emitted, so the
       extractor should be free to re-arm immediately. */
    if (cap->arm_age > GESTURE_ARM_TIMEOUT)
    {
        dataset->gesture_sample_count = 0;
        dataset->gesture_started = false;
        dataset->gesture_ended = false;
        cap->armed = false;
        return;
    }

    /* ── Peak confirmed → back-fill so the peak sits at a FIXED index ──────
       The sliding window's newest slot is the current sample, which is
       `peak_age` samples after the peak. Copying from
       `ACCEL_WINDOW_SIZE-1-peak_age-feedback_samples` up to the newest slot
       puts the peak at dataset index `feedback_samples` every time — the same
       index the old code gave the THRESHOLD CROSSING, so the window shape and
       length the 2nd-stage model sees are unchanged. */
    if (dataset->gesture_sample_count == 0)
    {
        if (cap->peak_age < GESTURE_PEAK_SEARCH)
        {
            return; /* a stronger peak may still arrive */
        }
        int first = ACCEL_WINDOW_SIZE - 1 - (int)cap->peak_age - feedback_samples;
        if (first < 0)
        {
            first = 0;
        }
        for (int i = first; i < ACCEL_WINDOW_SIZE; i++)
        {
            store_gesture_sample(dataset, ts, &state->sliding_window_accel[i],
                                 &state->sliding_window_gravity[i],
                                 state->sliding_window_ppg[i],
                                 state->sliding_window_fsr_adc[i],
                                 state->on_pressed);
        }
        dataset->gesture_started = true;
        return; /* the current sample is already inside the back-fill */
    }

    /* ── Collecting the tail after the peak ──────────────────────────────── */
    store_gesture_sample(dataset, ts, linear_acce, gravity, ppg, fsr_adc_value,
                         state->on_pressed);
    if (dataset->gesture_sample_count >= target_samples)
    {
        dataset->gesture_ended = true;
        static rt_tick_t median_lock_trigger_time = 0;
        /* Judge the background from ARM time, not now — see capture_state_t
           .arm_median. Re-computing here lets a strong gesture veto itself. */
        float median_difference_accel = cap->arm_median;
        bool is_gesture = true;
        if (median_difference_accel > 0.25f)
        {
            is_gesture = false;
            median_lock_trigger_time = current_time;
        }
        else if (median_lock_trigger_time != 0 &&
                 (current_time - median_lock_trigger_time) < 1000)
        {
            is_gesture = false;
        }
        bool posture_ok = (user_hand_horizontal || type == GESTURE_TYPE_TAP ||
                           imu_data_collection);
        /* Peak confirm — walking only (see gesture_confirm_pk). Standing still
           produced zero false windows, so applying it there would only cost
           real gestures. */
        bool confirm_ok = true;
        if (gesture_confirm_pk > 0 &&
            cap->arm_median > GESTURE_CONTEXT_WALK_MEDIAN)
        {
            confirm_ok =
                (cap->peak_val * 100.0f) >= (float)gesture_confirm_pk;
        }
        if (is_gesture && posture_ok && confirm_ok)
        {
            getTargetWaveformFromSlidingWindow(dataset, targetWave_algo,
                                               target_samples);
            // Directly notify gesture recognition task on HCPU
            // if (!check_ppg_error)
            {
                notify_gesture_dataset_hcpu(rt_tick_get(), target_samples,
                                            targetWave_algo);
            }
        }
        gesture_capture_report(type, cap, calculate_median_difference_accel(75),
                               fsr_adc_value, is_gesture, posture_ok,
                               confirm_ok);
        cap->armed = false;
        reset_gesture_state(dataset, current_time, type, 7);
    }
}

/**
 * @brief Process waveform capture - called from motion_tracking_in_hcpu
 */
static uint8_t get_ppg_count = 0;
static void waveform_capture_process(motion_data_t *motion_data, Vector3 *gyro)
{
    time_t current_ts = rt_tick_get(); // time(NULL);
    Vector3 *linear_acce = &motion_data->linear_acce;
    Vector3 *gravity = &motion_data->gravity;
    get_ppg_count ^= 1;
    uint32_t ppg_rawdata = motion_data->ppg_raw_data.raw_data[get_ppg_count];

    #ifdef USING_FSR_ADC_SAMPLER
    rt_uint32_t fsr_adc_value = bloc_control_fsr_adc_latest();
    #else
    rt_uint32_t fsr_adc_value = 0;
    #endif
    // LOG_D("ppg_rawdata:%d, fsr_adc_value:%d", ppg_rawdata, fsr_adc_value);

    // Update hand position detection
    uint8_t gesture_threshold_factor =
        bloc_setting_get_gesture_detect_threshold();
    float horizontal_threshold = gesture_threshold_factor * 0.01f;
    user_hand_horizontal = (gravity->x < 0.9 && gravity->x > -0.4);

    // Update watchface visibility
    waveform_gesture_state.if_watchface_visible =
        (gravity->y > -0.7 && gravity->z > -0.6) ||
        app_control_get_mouse_mode() || message_media_widget_focused();

    // Calculate linear acceleration difference
    float linear_accel_resultant = total_acceleration_magnitude(
        linear_acce->x, linear_acce->y, linear_acce->z);
    waveform_gesture_state.difference_accel =
        fabsf(linear_accel_resultant - prev_linear_accel_resultant);
    difference_accel_sliding_window[difference_accel_count] =
        waveform_gesture_state.difference_accel;
    if (difference_accel_count < MAX_GESTURE_SAMPLES - 1)
    {
        difference_accel_count++;
    }
    else
    {
        difference_accel_count = 0;
    }
    prev_linear_accel_resultant = linear_accel_resultant;

    #ifdef REAL_TIME_IMU_DATA_COLLECTION
    extern bool imu_raw_data_collection;
    extern bool imu_mouse_data_collection;
    if (imu_raw_data_collection || imu_mouse_data_collection)
    {
        // LOG_D("Collecting IMU raw data: ppg:%d, fsr_adc:%d", ppg_rawdata,
        //       fsr_adc_value);
        store_gesture_sample(&release_dataset, rt_tick_get_millisecond(),
                             linear_acce, gravity, ppg_rawdata, fsr_adc_value,
                             waveform_gesture_state.on_pressed);
        if (release_dataset.gesture_sample_count >= MAX_RAWDATA_TIME_STEP)
        {
            getTargetWaveformFromSlidingWindow(
                &release_dataset, targetWave_algo, MAX_RAWDATA_TIME_STEP);
            // if (!check_ppg_error)
            {
                packMatrixToBuffer(gsensorSamplesBuffer, targetWave_algo, NULL,
                                   release_dataset.gesture_sample_count);
                commu_send_linear_acce_buffer(
                    gsensorSamplesBuffer,
                    release_dataset.gesture_sample_count * BYTES_PER_SAMPLE);
            }
            release_dataset.gesture_sample_count = 0;
        }
        return;
    }
    #endif

    // Update sliding windows
    fill_realtime_accel_sliding_window(linear_acce, gravity, ppg_rawdata,
                                       fsr_adc_value, &waveform_gesture_state);
    check_gyro_threshold(gyro, &waveform_gesture_state);

    bool capture_both = app_control_get_game_mode();
#if !kReleaseMode
    capture_both = capture_both || capture_both_override;
#endif

    if (capture_both)
    {
        gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                   linear_acce, gyro, gravity, ppg_rawdata,
                                   fsr_adc_value, &waveform_gesture_state,
                                   GESTURE_TYPE_TAP, &tap_dataset,
                                   &tap_capture);
        gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                   linear_acce, gyro, gravity, ppg_rawdata,
                                   fsr_adc_value, &waveform_gesture_state,
                                   GESTURE_TYPE_RELEASE, &release_dataset,
                                   &release_capture);
    }
    else
    {
        if (SkaiWatchSys.motion_control_lock)
        {
            gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                       linear_acce, gyro, gravity, ppg_rawdata,
                                       fsr_adc_value, &waveform_gesture_state,
                                       GESTURE_TYPE_RELEASE, &release_dataset,
                                       &release_capture);
        }
        else
        {
            gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                       linear_acce, gyro, gravity, ppg_rawdata,
                                       fsr_adc_value, &waveform_gesture_state,
                                       GESTURE_TYPE_TAP, &tap_dataset,
                                       &tap_capture);
        }
    }
}

#if !kReleaseMode
/**
 * @brief MSH: `gcap both` / `gcap normal` / `gcap` — pick which extractors run.
 *
 * `both` forces TAP and RELEASE to run side by side on any screen so a capture
 * session can compare them; `normal` restores the shipping dispatch (RELEASE
 * only while motion_control_lock, TAP only in motion-control mode). Diagnostic
 * only — compiled out in release.
 */
static void gcap(int argc, char **argv)
{
    if (argc >= 2)
    {
        if (rt_strcmp(argv[1], "both") == 0)
        {
            capture_both_override = true;
        }
        else if (rt_strcmp(argv[1], "normal") == 0)
        {
            capture_both_override = false;
        }
        else if (rt_strcmp(argv[1], "pk") == 0 && argc >= 3)
        {
            gesture_big_pk = (uint16_t)atoi(argv[2]);
        }
        else if (rt_strcmp(argv[1], "log") == 0 && argc >= 3)
        {
            capture_log_on = (rt_strcmp(argv[2], "on") == 0);
        }
        else if (rt_strcmp(argv[1], "confirm") == 0 && argc >= 3)
        {
            gesture_confirm_pk = (uint16_t)atoi(argv[2]);
        }
        else
        {
            rt_kprintf("usage: gcap [both|normal|pk <x100>|log on|log off|"
                       "confirm <x100>]\n");
            return;
        }
    }
    rt_kprintf("gcap: both_override=%d game_mode=%d motion_control_lock=%d "
               "big_pk=%d confirm=%d log=%s\n      -> running: %s\n",
               capture_both_override, app_control_get_game_mode(),
               SkaiWatchSys.motion_control_lock, gesture_big_pk,
               gesture_confirm_pk, capture_log_on ? "on" : "off",
               (capture_both_override || app_control_get_game_mode())
                   ? "TAP + RELEASE"
                   : (SkaiWatchSys.motion_control_lock ? "RELEASE only"
                                                       : "TAP only"));
}
MSH_CMD_EXPORT(gcap, "gesture capture extractors: gcap [both|normal]");
#endif

#endif // ENABLE_WAVEFORM_CAPTURE
void set_prev_sensor_quat(uint16_t target_value)
{
    /* PC sim / pre-IMU-init: motion_data callback never registered so
     * watch_sensor_motion_data stays NULL. Skip — air-mouse positioning
     * isn't meaningful without sensor data anyway. */
    if (watch_sensor_motion_data == NULL)
        return;
    float middle_delta_yaw =
        (float)(target_value * control_angle) /
        total_moving_distance; // 將 total_yaw_energy_uint 轉換為浮點數
    float middle_delta_yaw_deg =
        -(middle_delta_yaw) * 3.14159265f / 180.0f; // 30度轉弧度
    Quaternion rotation_middle_deg = {.w = cosf(middle_delta_yaw_deg / 2.0f),
                                      .x = 0.0f,
                                      .y = 0.0f,
                                      .z = sinf(middle_delta_yaw_deg / 2.0f)};
    // page_mid 就是 total_yaw_energy_uint 當前 page_range 區間的中間值
    prev_sensor_quat = multiply_quaternions(&watch_sensor_motion_data->sensor_q,
                                            &rotation_middle_deg);
    // LOG_D("set_prev_sensor_quat:%f,%f,%f,%f", prev_sensor_quat.w,
    // prev_sensor_quat.x, prev_sensor_quat.y, prev_sensor_quat.z);
}

static uint8_t previous_page = 0;
static int total_yaw_energy_uint = 0;
void list_auto_positioning(void)
{
    if (watch_sensor_motion_data == NULL)
        return;
    // 找出 total_yaw_energy_uint 當前所在 page_range 區間的中間值
    uint8_t current_page = total_yaw_energy_uint / page_range;
    int page_middle = current_page * page_range + (page_range / 2);
    LOG_D("page_middle:%d,%d", page_middle, current_page);
    float middle_delta_yaw =
        (float)(page_middle * control_angle) /
        total_moving_distance; // 將 total_yaw_energy_uint 轉換為浮點數
    float middle_delta_yaw_deg =
        -(middle_delta_yaw) * 3.14159265f / 180.0f; // 30度轉弧度
    Quaternion rotation_middle_deg = {.w = cosf(middle_delta_yaw_deg / 2.0f),
                                      .x = 0.0f,
                                      .y = 0.0f,
                                      .z = sinf(middle_delta_yaw_deg / 2.0f)};
    // page_mid 就是 total_yaw_energy_uint 當前 page_range 區間的中間值
    prev_sensor_quat = multiply_quaternions(&watch_sensor_motion_data->sensor_q,
                                            &rotation_middle_deg);
}

static bool on_boundary = false;
static int prev_total_yaw_energy_uint = 0;
static void navigation_bar_control_with_euler_angle(euler_angle_t *delta,
                                                    motion_data_t *motion_data)
{
    if ((delta->yaw > 0.05 ||
         delta->yaw <
             -0.05)) //  && ((fabs(delta->roll)*1.75) < fabs(delta->yaw))
    {
        total_yaw_energy_uint =
            (int)(-delta->yaw * total_moving_distance / control_angle);
        // LOG_D("total_yaw_energy_uint:%d,%f,%f", total_yaw_energy_uint,
        // delta->yaw, (float)(-delta->yaw * total_moving_distance /
        // control_angle));
        int bottom_line = page_range * (scroll_segment_count);
        if (total_yaw_energy_uint >= bottom_line)
        {
            total_yaw_energy_uint = bottom_line - 1; // 限制最大值
            set_prev_sensor_quat(bottom_line - 1);
        }
        else if (total_yaw_energy_uint <= 0)
        {
            total_yaw_energy_uint = 0; // 確保負值
            set_prev_sensor_quat(0);
        }

        int page =
            scroll_segment_count - (total_yaw_energy_uint / page_range) - 1;
        if ((page == 0 || page >= scroll_segment_count - 1) && on_boundary &&
            scroll_segment_count > 2)
        {
            motor_pattern_unlocked();
            on_boundary = false;
        }
        else if (page != 0 && page < scroll_segment_count - 1)
        {
            on_boundary = true;
        }

        if (page < 0)
            page = 0; // 最小限制在0

        if (abs(prev_total_yaw_energy_uint - total_yaw_energy_uint) > 3)
        {
            prev_total_yaw_energy_uint = total_yaw_energy_uint;
            lvgl_msg_t msg;
            msg.type = LVGL_MSG_TYPE_APP_LIST_SCROLL_BAR_OFFSET;
            msg.data.scroll_offset = (int)((float)total_yaw_energy_uint);
            // msg.data.scroll_offset = total_yaw_energy_uint;
            lvgl_send_msg(msg);
        }

        if (previous_page != page && !is_user_touching_screen())
        {
            // LOG_I("total_yaw_energy_uint_page:%d,%d,%d", page,
            // total_yaw_energy_uint, page_range);
            previous_page = page;
            lvgl_msg_t msg;
            msg.type = LVGL_MSG_TYPE_NAV_BAR_CONTROL;
            msg.data.action = page;
            lvgl_send_msg(msg);
        }
    }
    // else if ((fabs(delta->roll)*1.75) > fabs(delta->yaw))
    // {
    // 	// total_yaw_energy = 0; // reset if the energy is too low
    // 	LOG_D("navigation_bar_control_with_euler_angle reset due to
    // roll:%f,yaw:%f", delta->roll, delta->yaw);
    // }
}

static void send_quaternion_to_ble_client(rt_uint32_t ts, Quaternion *q)
{
    static rt_uint32_t last_tick = 0;
    if (ts - last_tick >= 8) // 125Hz = 8ms
    {
        float quat[4] = {q->w, q->x, q->y, q->z};
#ifdef BSP_USING_BLOC_SKAIWALK
        bloc_skaiwalk_prepare_quaternion_buffer(quat);
#endif
        last_tick = ts;
    }
}

/**
 * @brief Multiplies two quaternions.
 *
 * This function takes two quaternions as input and returns their product.
 * Quaternion multiplication is not commutative, meaning the order of the
 * operands matters.
 *
 * @param q1 Pointer to the first quaternion.
 * @param q2 Pointer to the second quaternion.
 * @return The product of the two quaternions.
 */
static Quaternion multiply_quaternions(Quaternion *q1, Quaternion *q2)
{
    Quaternion result;

    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
    result.z = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;

    return result;
}

static Quaternion conjugate(Quaternion *q)
{
    Quaternion result;
    result.w = q->w;
    result.x = -q->x;
    result.y = -q->y;
    result.z = -q->z;
    return result;
}

// Calculate delta quaternion between two quaternions = q1 * conjugate(q2),
// where q2 is the previous quaternion
static Quaternion calculateDeltaQuaternion(Quaternion *q, Quaternion *prev_quat)
{
    Quaternion conjugatedQ = conjugate(prev_quat);
    Quaternion deltaQ = multiply_quaternions(q, &conjugatedQ);
    return deltaQ;
}

//-------------------------------------------------------------------------------------------
euler_angle_t computeEulerAngleFromQuaternion(Quaternion *q)
{
    float roll = atan2f(2.0f * (q->w * q->x + q->y * q->z),
                        q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z) *
                 57.29577951f;
    float pitch = asinf(2.0f * (q->x * q->z - q->w * q->y)) * 57.29577951f;
    float yaw = -atan2f(2.0f * (q->x * q->y + q->w * q->z),
                        q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z) *
                57.29577951f;

    return (euler_angle_t){.roll = roll, .pitch = pitch, .yaw = yaw};
}

static euler_angle_t motion_tracking_algorithm(Quaternion *quaternion,
                                               Quaternion *prev_quat)
{
    Quaternion deltaQ = calculateDeltaQuaternion(quaternion, prev_quat);
    return computeEulerAngleFromQuaternion(&deltaQ);
}

#ifdef BSP_USING_AIR_MOUSE
extern bool is_skai_touch_enabled(void);
extern void set_air_mouse_moving_state(bool state);
extern int get_gravity_position(void); /* 定義在本檔後方,側立 gate 前置宣告 */
static void report_air_mouse_data(air_plane_delta_movement_t *movement,
                                  rt_uint32_t ts)
{
    /* 側立時不送連續飛鼠游標(founder 2026-07-30:側立只用圓盤(手腕比方向)+觸控板(手指滑),
       手錶整體移動/傾斜的 air mouse 不控游標)。s_press_free_move / s_motion_drag / handfree
       都經此共同出口一處擋;圓盤走 commu_send_dial_dir、觸控板手指滑走 hid_mouse 的 move,
       都不經這裡,故側立圓盤+觸控板照常。 */
    if (get_gravity_position() == GRAVITY_POSITION_FACE_SIDE)
        return;
    // if (abs(movement->x >= 3) || abs(movement->y) >= 3)
    // {
    //     set_air_mouse_moving_state(true);
    // }
    // else
    // {
    //     set_air_mouse_moving_state(false);
    // }
    if (movement->x == 0 && movement->y == 0)
    {
        return;
    }
    /* 取負校正：這組 air-mouse delta 跟 dial 圓盤同源，dial 累積時已取負把方向校正對
       （手腕上下左右對應正確）；頂部飛鼠若送原始 delta 就會跟 dial 相反（founder
       2026-07-16 真機驗出），這裡一併反向。 */
    control_provider.ble_hid_mouse_move((int8_t)(-movement->x), (int8_t)(-movement->y));
    movement->last_report_ts = ts;
    movement->x = 0;
    movement->y = 0;
}

extern bool app_hid_mouse_movement_lock(void);
static uint8_t log_count = 0;

    // DPS to rad/s conversion factor (PI / 180)
    #define DPS_TO_RADS 0.01745329f
    // Default mouse sensitivity (matching GyroService)
    #define AIR_MOUSE_SENSITIVITY 20.0f
    // 移動鎖：解鎖 = 飛鼠開始送游標的「觸發距離」。判定用「淨位移」(帶符號累加 dx/dy,見
    // s_air_net_dx/dy)——手腕來回抖動互相抵消不觸發,只有真把手錶移到別處淨位移才累積過
    // 門檻(founder 2026-07-30:原用總路程,抖動也一路累加→長按畫面變飛鼠;改淨位移區分
    // 抖動 vs 真移動)。門檻演進:40→60→80(founder 2026-07-31 再加大成 40 的兩倍:還是有點
    // 容易長按變飛鼠)。只 gate 飛鼠;觸控板送游標不看它。太容易變飛鼠再往上加、體感太鈍再降。
    #define GYRO_MOVE_CANCEL_THRESHOLD 80.0f
    // 飛鼠移動中停頓 >0.5s 重鎖後的「重觸發」門檻(founder 2026-07-31,1.5× 而非 2×):已經在
    // 飛鼠模式、只是中途停一下想再動,不必像初次從靜止那麼費力,但仍要過門檻擋放開瞬間微動。
    #define GYRO_MOVE_RETRIGGER_THRESHOLD 60.0f
    // 飛鼠移動中「停在小區域超過這麼久」= 準備放開的訊號 → 重鎖(founder 2026-07-31)。
    #define AIR_PAUSE_HOLD_MS 500
    // 「小區域」半徑(淨位移單位):停頓偵測用參考點+半徑,淨位移離參考點超過此值=還在飛、更新
    // 參考重計時;在半徑內累積夠 AIR_PAUSE_HOLD_MS 就重鎖。太大=難判定停頓、太小=微晃就重鎖。
    #define AIR_PAUSE_RADIUS 40
    /* 按住觸控板期間「飛鼠搶走游標」的專屬門檻(founder 2026-07-24 真機:手腕盡量不動想滑
       觸控板,卻一直變成體感)。不能共用上面的 30——手指在錶面上滑的反作用力本身就會帶動
       手腕,gyro 一路累積;加上 IMU(100Hz+)判定頻率遠高於 LVGL PRESSING(~30Hz),飛鼠幾乎
       必勝。拉高成要「明確轉手腕」才搶得走,純滑觸控板累積達不到,手指那 5px 就能先到。 */
    #define PRESS_FREE_MOVE_CLAIM_UNITS 120.0f
    // Roll-compensation sign for data-collection air-mouse (see air_mouse_process).
    // Flip to -1.0f on device if roll compensation goes the WRONG way.
    #define AIR_MOUSE_COLLECT_ROLL_SIGN 1.0f
    // Vertical-axis sign for the collection air-mouse's gyro_y mapping (2026-07-16:
    // gyro_x read ~0 for up/down in the collection hold posture — real device
    // testing found gyro_y is the responsive axis there). Flip to +1.0f on device
    // if up/down comes out reversed.
    #define AIR_MOUSE_COLLECT_V_SIGN -1.0f

static bool mouse_movement_lock = false;
static float gyro_movement_distance = 0.0f; /* 總路程:claim owner(鎖觸控板)仍用它 */
/* 淨位移(帶符號累加 dx/dy):飛鼠觸發判定用。手腕來回抖動互相抵消、只有真移到別處才增大,
   區分抖動 vs 真移動(founder 2026-07-30)。PRESSED 歸零。 */
static int s_air_net_dx = 0, s_air_net_dy = 0;
/* 飛鼠觸發門檻切換 + 停頓偵測(founder 2026-07-31)。s_air_retrigger:false=初次觸發用 80、
   停頓重鎖後 true=用 60。停頓偵測(僅飛鼠移動中):淨位移離參考點在半徑內就累積計時,滿 0.5s
   重鎖=準備放開,擋放開瞬間微動又把游標帶跑。三者 PRESSED 歸零(retrigger 回 false)。 */
static bool s_air_retrigger = false;
static int s_air_pause_ref_x = 0, s_air_pause_ref_y = 0;
static rt_tick_t s_air_pause_since = 0;

/* 觸控板 vs 飛鼠仲裁:改由下面的「手指前哨」純動態決定(founder 2026-07-30「能切換,兩個
   同時做只觸控板」)。以下這組「先到先贏」永久鎖(claim owner)已停用——claim 點全數拔掉,
   兩個 owner 恆 false、對應 gate 條件恆通過;變數/reset 保留,隨時能復活顯式鎖。
   - s_touch_cursor_owned:曾在「手指先滑贏」時 claim → 擋 report_air_mouse_data。
   - s_air_cursor_owned:曾在飛鼠真送游標時 claim → hid_mouse 據此擋觸控板那條。 */
static volatile bool s_touch_cursor_owned = false;
static volatile bool s_air_cursor_owned = false;
/* 手指「前哨」:hid_mouse 每偵測到手指在滑就刷新此 tick。飛鼠送游標前先看手指最近有沒有
   在滑——用手指意圖仲裁,取代之前靠 gyro 門檻猜(高了體感停頓、低了滑板變飛鼠,二選一)。
   claim(s_touch_cursor_owned)是「已定案手指贏」、前哨是「手指正在滑、暫別讓飛鼠插」。 */
static volatile rt_tick_t s_finger_active_tick = 0;
#define FINGER_ACTIVE_HOLD_MS 250 /* 手指停止滑動後仍擋飛鼠這麼久,涵蓋滑滑停停的間隙 */

/* Data-collection air-mouse roll reference (file scope so the always-run motion
 * handler can invalidate it when collection ends — air_mouse_process itself only
 * runs in mouse modes and can't reliably see the off state). */
static Vector3 s_collect_gref = {0};
static bool s_collect_gref_valid = false;

/* Phone-game (data-collection) air-mouse mapping selector (2026-07-10). true =
 * the desktop mouse app's PROVEN mapping (gyro_x = up/down tilt, gyro_z =
 * left/right) — that path never reverses. false = the old roll-compensated gyro_y
 * path (kept live below as a fallback), which intermittently reversed (roll wrapped
 * past ±90°, or the y-z fallback went raw body-frame). Flip to false + rebuild if
 * the new mapping still reverses on device, so the game stays usable either way. */
static bool s_collect_use_mouse_app_map = true;

void air_mouse_movement_lock_reset(void)
{
    mouse_movement_lock = true;
    gyro_movement_distance = 0.0f;
    s_air_net_dx = 0;
    s_air_net_dy = 0;
    /* 新一次按下:互鎖歸零重新比(本函式由 hid_mouse PRESSED 無條件呼叫) */
    s_touch_cursor_owned = false;
    s_air_cursor_owned = false;
    s_finger_active_tick = 0;
    /* 新 press 從初次門檻(80)開始、清停頓偵測 */
    s_air_retrigger = false;
    s_air_pause_since = 0;
}

/* 觸控板/飛鼠互鎖(見上方 s_touch_cursor_owned 註解)。claim 只由 hid_mouse 的手指判定
   呼叫;reset 在 RELEASED/PRESS_LOST 呼叫——放開後不清的話 s_touch_cursor_owned 殘留
   會把 handfree 飛鼠一路鎖到下次按下。 */
void bloc_touch_cursor_claim(void)
{
    /* 飛鼠已經贏了就不搶(motion thread 可能剛 claim 完)——兩邊各自先讓,把「同一幀雙
       claim→兩條都被鎖、這次 press 游標不動」的窄窗口再縮小(放開就恢復,非致命)。 */
    if (s_air_cursor_owned || s_touch_cursor_owned)
        return;
    s_touch_cursor_owned = true;
}
/* 手指前哨:hid_mouse 偵測到手指在滑就呼叫,刷新 tick。 */
void bloc_touch_finger_active(void) { s_finger_active_tick = rt_tick_get(); }
static bool touch_finger_recent(void)
{
    return s_finger_active_tick != 0 &&
           (rt_tick_get() - s_finger_active_tick) <
               rt_tick_from_millisecond(FINGER_ACTIVE_HOLD_MS);
}
void bloc_cursor_owner_reset(void)
{
    s_touch_cursor_owned = false;
    s_air_cursor_owned = false;
    s_finger_active_tick = 0;
}
bool mouse_air_cursor_owned(void) { return s_air_cursor_owned; }

static bool switch_freehand_mode = false;
bool get_switch_freehand_mode(void)
{
    return switch_freehand_mode;
}
static bool switch_mouse_scroll_mode = false;
bool get_switch_mouse_scroll_mode(void)
{
    return switch_mouse_scroll_mode;
}
static bool scroll_up_mode = false;
bool get_scroll_up_mode(void)
{
    return scroll_up_mode;
}
extern bool get_hid_mouse_handfree_mode(void);
extern bool hid_mouse_top_fly_active(void);

/* ─────────────────────────────────────────────────────────────────────────
   側立方向盤 (pose-dial) —— 2026-07-20 founder 對調:圓盤改由「側立手錶」(錶面轉向
   側邊,原本手寫的姿勢)觸發;主觸控板長按讓給手寫。無手錶 UI:手腕比劃方向(側立
   軸向),向量角度 → 8 向 sector、向量長度 → 力度 mag,節流上傳 KEY_DIAL_DIR(0x1a),
   桌面以游標為中心畫圓盤、對應扇形高亮。「轉回朝上」穩定=以當下 sector commit;
   立起/垂下=取消。wire (start/update/end{dir,mag}) 與觸控時代完全相同,桌面零改動。
   dir 0=上，順時針 1=右上 2=右 3=右下 4=下 5=左下 6=左 7=左上；len<deadzone → -1。
   ───────────────────────────────────────────────────────────────────────── */
#define DIAL_DEADZONE_UNITS   40.0f  /* 累積向量長度 < 此 = 中心 deadzone（不選）*/
#define DIAL_MAX_UNITS       260.0f  /* 向量長度上限（mag=1000）；超過 clamp。真機再調 */
#define DIAL_SEND_INTERVAL_MS   33   /* update 節流 ~30Hz，避免洗版 BLE/log */
#define DIAL_WRIST_MOVE_UNITS  3.0f  /* air_mouse 每幀 delta(|dx|+|dy|) >= 此＝手腕在轉(比方向)，
                                        非手指滑。hid_mouse 拖曳判定用它區分「手腕動 vs 手指動」──
                                        手腕動時手指的連動位移不算拖曳，圓盤才穩(founder 2026-07-17)*/
#define DIAL_WRIST_HOLD_MS      200  /* 手腕停止後仍視為「比方向中」這麼久，涵蓋手指慣性殘留位移，
                                        免得手腕轉到定位剛停的瞬間被手指殘留誤判成拖曳 */
#define DIAL_WRIST_ACCUM_UNITS 40.0f /* 進入「體感拖曳」:大震後手錶移動的**累積總量**過此門檻
                                        (founder 2026-07-20:瞬間速度判定慢慢拖不會動→改總距離,
                                        慢動作也會累積觸發;雜訊被 air_mouse deadzone 濾掉不累積) */

/* 側立圓盤軸向:gyro(rad/s)直接線性累積(無游標加速曲線),gain=7 沿用空中手寫時代手感。
   符號=側立手寫 pen 映射同款(x←-gx / y←+gz)——2026-07-20 founder 真機:「上下左右都反」
   =朝上圓盤那套 pointing 取負慣例在側立**不成立**(凍結向量設計拿掉了回中抵消,指哪
   就是哪),雙軸翻回 pen 映射。再反就再翻這兩個 SIGN([dial-pose] 1Hz log 對方向)。 */
#define DIAL_POSE_GAIN      7.0f
#define DIAL_POSE_SIGN_X  (-1.0f)  /* 對 gyro_x → ax(螢幕 x 右+) */
#define DIAL_POSE_SIGN_Y  (+1.0f)  /* 對 gyro_z → ay(螢幕 y 下+) */
#define DIAL_POSE_FREEZE_GX 0.60f  /* |gravity.x| 掉出此值=正在離開側立→凍結向量,
                                      退出旋轉的過渡角速度不污染已比好的方向 */

static volatile bool s_dial_active = false;
static volatile bool s_dial_pose_active = false; /* 側立圓盤 session(motion thread 姿勢機開關) */
static volatile bool s_motion_drag_active = false; /* 體感拖曳(GUI 長按狀態機開關,gyro→游標+左鍵按住) */
static volatile bool s_press_free_move = false;    /* 按住觸控板期間(1s 內)gyro→游標,無左鍵(founder:
                                                      按下馬上動手錶游標就要動) */
static volatile rt_tick_t s_dial_wrist_last_move = 0; /* 最後一次手腕明顯轉動的 tick(gyro 判定用) */
static volatile float s_wrist_accum = 0.0f; /* 大震後手錶移動累積量(體感拖曳進入判定;arm 時歸零) */
static bool  s_dial_reset_pending = false;
static float s_dial_ax = 0.0f, s_dial_ay = 0.0f; /* 累積向量：螢幕座標 x 右+ / y 下+ */
static int   s_dial_cur_dir = -1;
static int   s_dial_cur_mag = 0;
static rt_tick_t s_dial_last_send = 0;
static rt_tick_t s_dial_pose_last_dbg = 0;

/* 累積向量 (ax,ay)（螢幕座標）→ 8 向 sector + mag(0..1000)。桌面圓盤扇形用同一 dir。 */
static int dial_vector_to_sector(float ax, float ay, int *mag_out)
{
    float len = sqrtf(ax * ax + ay * ay);
    if (len < DIAL_DEADZONE_UNITS)
    {
        *mag_out = 0;
        return -1;
    }
    float clamped = (len > DIAL_MAX_UNITS) ? DIAL_MAX_UNITS : len;
    float m = (clamped - DIAL_DEADZONE_UNITS) / (DIAL_MAX_UNITS - DIAL_DEADZONE_UNITS);
    int mag = (int)(m * 1000.0f + 0.5f);
    *mag_out = (mag > 1000) ? 1000 : (mag < 0 ? 0 : mag);
    /* atan2(y,x) 螢幕座標：0=右 +90=下 ±180=左 -90=上。轉「0=上、順時針」：
       sector = round((deg+90)/45) mod 8。 */
    float deg = atan2f(ay, ax) * 57.29577951f;
    int sector = (int)floorf((deg + 90.0f) / 45.0f + 0.5f);
    return ((sector % 8) + 8) % 8;
}

/* 累積向量 → clamp + sector 更新 + 節流送 update。側立累積路徑用。 */
static void mouse_dial_vector_refresh(void)
{
    if (s_dial_reset_pending)
    {
        s_dial_ax = 0.0f; s_dial_ay = 0.0f;
        s_dial_cur_dir = -1; s_dial_cur_mag = 0;
        s_dial_reset_pending = false;
    }
    /* clamp 累積向量長度，避免比劃久了積分無限增長、commit 時方向遲鈍 */
    float len = sqrtf(s_dial_ax * s_dial_ax + s_dial_ay * s_dial_ay);
    if (len > DIAL_MAX_UNITS)
    {
        float k = DIAL_MAX_UNITS / len;
        s_dial_ax *= k; s_dial_ay *= k;
    }
    int mag = 0;
    s_dial_cur_dir = dial_vector_to_sector(s_dial_ax, s_dial_ay, &mag);
    s_dial_cur_mag = mag;

    rt_tick_t now = rt_tick_get();
    if (now - s_dial_last_send >= rt_tick_from_millisecond(DIAL_SEND_INTERVAL_MS))
    {
        extern bool commu_send_dial_dir(const char *phase, int dir, int mag);
        commu_send_dial_dir("update", s_dial_cur_dir, s_dial_cur_mag);
        s_dial_last_send = now;
    }
}

/* start/end/cancel:桌面 wire 的三個階段。start 由側立姿勢機(mouse_dial_pose_begin)呼叫;
   end/cancel 也給 hid_mouse.c 的殘留清理(dial_drag_state_reset)用(extern)。 */
void mouse_dial_start(void)
{
    s_dial_reset_pending = true;
    s_dial_active = true;
    s_dial_cur_dir = -1;
    s_dial_wrist_last_move = 0; /* 重置手腕活動,開場先當靜止 */
    s_dial_last_send = 0; /* 讓開場那一幀的 update 立即送出 */
    extern bool commu_send_dial_dir(const char *phase, int dir, int mag);
    commu_send_dial_dir("start", -1, 0);
}
void mouse_dial_end(void)
{
    if (!s_dial_active) return;
    s_dial_active = false;
    s_dial_pose_active = false;
    extern bool commu_send_dial_dir(const char *phase, int dir, int mag);
    commu_send_dial_dir("end", s_dial_cur_dir, s_dial_cur_mag);
}
/* 取消 dial（不 commit 方向）：桌面收 dir=-1 就 hide 圓盤。 */
void mouse_dial_cancel(void)
{
    if (!s_dial_active) return;
    s_dial_active = false;
    s_dial_pose_active = false;
    extern bool commu_send_dial_dir(const char *phase, int dir, int mag);
    commu_send_dial_dir("end", -1, 0);
}
bool mouse_dial_active(void) { return s_dial_active; }

/* 體感拖曳開關(hid_mouse 長按狀態機呼叫,GUI thread;volatile bool 單字寫入安全)。 */
void bloc_motion_drag_set(bool on) { s_motion_drag_active = on; }

/* 按住期間自由移動開關(PRESSED 開/進拖曳或放開關):gyro 驅動游標但不含左鍵。 */
void bloc_press_free_move_set(bool on) { s_press_free_move = on; }

/* hid_mouse.c 拖曳判定用：手腕最近 DIAL_WRIST_HOLD_MS 內是否在轉(比方向)。手腕在轉時手指的
   連動位移不該當成拖曳(圓盤才穩)，手腕靜止才把手指主動滑視為拖曳──gyro 是唯一能區分「手腕動
   vs 手指動」的信號(founder 2026-07-17 圓盤被手指連動誤觸消失)。 */
bool mouse_dial_wrist_moving(void)
{
    return (rt_tick_get() - s_dial_wrist_last_move)
           < rt_tick_from_millisecond(DIAL_WRIST_HOLD_MS);
}

/* 體感拖曳進入判定:大震後手錶移動累積總量過門檻(慢動作也會累積到)。 */
bool mouse_wrist_accum_triggered(void)
{
    return s_wrist_accum >= DIAL_WRIST_ACCUM_UNITS;
}

/* 歸零累積量(hid 在大震時呼叫):大震**之前**的瞄準/前奏動作不算,只累積之後的
   移動(founder:選字後拖曳變重新框選——前奏動作把游標帶跑+瞬間誤觸發)。 */
void bloc_wrist_accum_reset(void)
{
    s_wrist_accum = 0.0f;
}

/* ─────────────────────────────────────────────────────────────────────────
   手寫 (handwriting) —— 2026-07-20 founder 三改:觸發=「畫面右緣向左拉出」
   (hid_mouse 觸控機偵測→開 view→bloc_handwrite_begin);輸入=**直接在錶面上寫**
   (觸控座標=筆跡,hw_view 的觸控 cb 餵 bloc_handwrite_feed_point+set_pen,
   畫布=錶面解析度;不再用 gyro 體感積分)。
   本模組角色=0x1b 串流引擎:motion thread 以固定節奏讀 s_hw_x/y+pen 做
   下筆"d"/筆點"m"批次/提筆"u"/懸浮"h" 的組幀與節流上傳。
   結束:提筆後 HW_IDLE_END_MS 沒再下筆=寫完→自動收掉送出;另有
   VERTICAL/SIDE 姿勢保險收斂。
   ───────────────────────────────────────────────────────────────────────── */
#define HW_DEADZONE_RADS   0.02f  /* gyro 雜訊死區——現僅側立圓盤累積用(手寫已改觸控) */
#define HW_FLUSH_MS        40     /* 筆點批次上傳節流(~25Hz) */
#define HW_BATCH_MAX       6
#define HW_HOVER_MS        120    /* 提筆時筆尖位置低頻上傳(桌面畫 hover 點) */

static volatile bool s_hw_active = false;
static volatile bool s_hw_pen_down = false; /* GUI thread 寫(觸控),motion thread 讀 */
static bool s_hw_prev_pen = false;
static float s_hw_x = 0, s_hw_y = 0;
static int s_hw_w = 466, s_hw_h = 466;
static int s_hw_last_sent_x = -1, s_hw_last_sent_y = -1;
static int s_hw_hover_x = -1, s_hw_hover_y = -1;
static int16_t s_hw_batch[HW_BATCH_MAX][2];
static int s_hw_batch_n = 0;
static rt_tick_t s_hw_last_flush = 0, s_hw_last_hover = 0, s_hw_last_dbg = 0;
/* GUI→motion 請求旗標(單寫單讀):批次緩衝與 0x1b 送幀**只在 motion thread 動**——
   GUI 按鈕直接動批次曾與 motion 併發互踩(mem manage fault,2026-07-20 真機兩連炸)。 */
static volatile bool s_hw_req_end = false;
static volatile bool s_hw_req_cancel = false;
static volatile bool s_hw_req_next = false;
static volatile bool s_hw_req_clear = false;
static volatile bool s_hw_req_backspace = false;
/* 按候選字定稿(founder 2026-07-20 晚):-1=無(取 top-1),>=0=候選 index。
   先寫 pick 再立 next 旗標,motion thread 反序讀,無鎖交接。 */
static volatile int s_hw_req_next_pick = -1;

bool bloc_handwrite_active(void)
{
    return s_hw_active;
}

void bloc_handwrite_set_pen(bool down)
{
    s_hw_pen_down = down;
}

/* GUI thread(hw_view 觸控 cb)每 tick 餵當前指尖座標(=錶面座標=畫布座標)。
   單字 float 寫入在 ARM 上原子,motion thread 讀到前後值皆合法。 */
void bloc_handwrite_feed_point(int x, int y)
{
    if (x < 0) x = 0;
    if (x > s_hw_w) x = s_hw_w;
    if (y < 0) y = 0;
    if (y > s_hw_h) y = s_hw_h;
    s_hw_x = (float)x;
    s_hw_y = (float)y;
}

void bloc_handwrite_begin(int canvas_w, int canvas_h)
{
    s_hw_w = (canvas_w > 0) ? canvas_w : 466;
    s_hw_h = (canvas_h > 0) ? canvas_h : 466;
    s_hw_x = s_hw_w / 2.0f;
    s_hw_y = s_hw_h / 2.0f;
    s_hw_pen_down = false;
    s_hw_prev_pen = false;
    s_hw_batch_n = 0;
    s_hw_last_sent_x = s_hw_last_sent_y = -1;
    s_hw_hover_x = s_hw_hover_y = -1;
    s_hw_req_end = s_hw_req_cancel = s_hw_req_next = false;
    s_hw_req_clear = s_hw_req_backspace = false;
    s_hw_req_next_pick = -1;
    s_hw_last_flush = s_hw_last_hover = s_hw_last_dbg = rt_tick_get();
    char json[48];
    rt_snprintf(json, sizeof(json), "{\"ph\":\"start\",\"w\":%d,\"h\":%d}", s_hw_w, s_hw_h);
    extern bool commu_send_handwrite(const char *json);
    commu_send_handwrite(json);
    LOG_I("[handwrite] begin %dx%d", s_hw_w, s_hw_h);
    s_hw_active = true; /* 最後才開:motion thread 看到 active 時上面狀態已就緒 */
}

static void hw_batch_flush(const char *ph)
{
    if (s_hw_batch_n <= 0) return;
    char json[24 + HW_BATCH_MAX * 14];
    int off = rt_snprintf(json, sizeof(json), "{\"ph\":\"%s\",\"p\":[", ph);
    for (int i = 0; i < s_hw_batch_n && off > 0 && off < (int)sizeof(json); i++)
        off += rt_snprintf(json + off, sizeof(json) - off, "%s[%d,%d]",
                           i ? "," : "", (int)s_hw_batch[i][0], (int)s_hw_batch[i][1]);
    if (off > 0 && off < (int)sizeof(json) - 2)
    {
        off += rt_snprintf(json + off, sizeof(json) - off, "]}");
        extern bool commu_send_handwrite(const char *json);
        commu_send_handwrite(json);
    }
    s_hw_batch_n = 0;
}

static void hw_batch_push(void)
{
    int x = (int)s_hw_x, y = (int)s_hw_y;
    if (x == s_hw_last_sent_x && y == s_hw_last_sent_y) return; /* 沒動不重複推 */
    if (s_hw_batch_n >= HW_BATCH_MAX) return;
    s_hw_batch[s_hw_batch_n][0] = (int16_t)x;
    s_hw_batch[s_hw_batch_n][1] = (int16_t)y;
    s_hw_batch_n++;
    s_hw_last_sent_x = x;
    s_hw_last_sent_y = y;
}

/* ── GUI 按鈕入口(2026-07-20 真機兩連炸後改制):全部**只設旗標**,實際的批次收尾
   與 0x1b 送幀由 motion thread 統一執行——批次緩衝單一寫者,GUI 直接動批次會與
   motion 併發互踩(mem manage fault)。
   cancel="x" 退出不送出;backspace="b" 刪已定稿最後一字;clear="c" 擦掉當前字重寫;
   next="n" 當前字定稿+清板;end="end" 送出。 */
void bloc_handwrite_cancel(void)
{
    if (s_hw_active) s_hw_req_cancel = true;
}

void bloc_handwrite_backspace(void)
{
    if (s_hw_active) s_hw_req_backspace = true;
}

void bloc_handwrite_clear(void)
{
    if (s_hw_active) s_hw_req_clear = true;
}

void bloc_handwrite_next_char(void)
{
    if (s_hw_active) s_hw_req_next = true;
}

/* 按候選清單第 idx 個=用該候選定稿+換下個字(= next 帶 pick)。 */
void bloc_handwrite_next_pick(int idx)
{
    if (!s_hw_active || idx < 0)
        return;
    s_hw_req_next_pick = idx; /* 先 pick 後旗標(見宣告處) */
    s_hw_req_next = true;
}

void bloc_handwrite_end(void)
{
    if (s_hw_active) s_hw_req_end = true;
}

/* motion thread(air_mouse_process 每 sample 轉入):以固定節奏讀 s_hw_x/y(由 GUI
   觸控 feed)+pen 狀態,做 0x1b 組幀/批次/節流+idle 結束判定。筆尖座標不在這裡
   產生(2026-07-20 三改:輸入=錶面觸控,非 gyro 積分)。 */
static void handwrite_motion_process(void)
{
    rt_tick_t now = rt_tick_get();
    extern bool commu_send_handwrite(const char *json);

    /* GUI 請求在這裡執行(批次/送幀單一寫者=本 thread)。end/cancel 為終結請求,
       處理後本 sample 結束。 */
    if (s_hw_req_cancel || s_hw_req_end)
    {
        bool cancel = s_hw_req_cancel;
        s_hw_req_cancel = false;
        s_hw_req_end = false;
        s_hw_req_next = false;
        s_hw_req_clear = false;
        s_hw_req_backspace = false;
        s_hw_req_next_pick = -1;
        if (s_hw_prev_pen)
        {
            hw_batch_flush("m");
            commu_send_handwrite("{\"ph\":\"u\"}");
        }
        s_hw_prev_pen = false;
        s_hw_pen_down = false;
        commu_send_handwrite(cancel ? "{\"ph\":\"x\"}" : "{\"ph\":\"end\"}");
        LOG_I("[handwrite] %s", cancel ? "cancel" : "end");
        s_hw_active = false; /* 最後關 */
        return;
    }
    if (s_hw_req_next || s_hw_req_clear)
    {
        bool next = s_hw_req_next;
        int pick = s_hw_req_next_pick;
        s_hw_req_next = false;
        s_hw_req_clear = false;
        s_hw_req_next_pick = -1;
        if (s_hw_prev_pen)
        {
            hw_batch_flush("m");
            commu_send_handwrite("{\"ph\":\"u\"}");
            s_hw_prev_pen = false;
            s_hw_pen_down = false;
        }
        else
        {
            hw_batch_flush("m");
        }
        if (next && pick >= 0)
        {
            /* 候選定稿:i=候選 index,手機用 candidates[i] 取代 top-1 */
            char json[32];
            rt_snprintf(json, sizeof(json), "{\"ph\":\"n\",\"i\":%d}", pick);
            commu_send_handwrite(json);
        }
        else
        {
            commu_send_handwrite(next ? "{\"ph\":\"n\"}" : "{\"ph\":\"c\"}");
        }
        LOG_I("[handwrite] %s pick=%d", next ? "next-char" : "clear", pick);
    }
    if (s_hw_req_backspace)
    {
        s_hw_req_backspace = false;
        commu_send_handwrite("{\"ph\":\"b\"}");
        LOG_I("[handwrite] backspace");
    }

    /* 結束由使用者按「輸入/退出」鈕(founder 2026-07-20:不要任何自動送出);
       姿勢保險收斂已拆(錶面直寫時手腕角度千變萬化,舊保險=「自動幫我執行」兇手)。 */
    bool pen = s_hw_pen_down;
    if (pen && !s_hw_prev_pen)
    {
        /* 下筆:批次重啟、先送 "d" 帶第一點 */
        s_hw_batch_n = 0;
        s_hw_last_sent_x = s_hw_last_sent_y = -1;
        hw_batch_push();
        hw_batch_flush("d");
        s_hw_last_flush = now;
    }
    else if (pen)
    {
        hw_batch_push();
        if (s_hw_batch_n >= HW_BATCH_MAX ||
            (s_hw_batch_n > 0 &&
             now - s_hw_last_flush >= rt_tick_from_millisecond(HW_FLUSH_MS)))
        {
            hw_batch_flush("m");
            s_hw_last_flush = now;
        }
    }
    else
    {
        if (s_hw_prev_pen)
        {
            hw_batch_flush("m"); /* 殘批先送,筆畫尾端完整 */
            extern bool commu_send_handwrite(const char *json);
            commu_send_handwrite("{\"ph\":\"u\"}");
        }
        int hx = (int)s_hw_x, hy = (int)s_hw_y;
        if ((hx != s_hw_hover_x || hy != s_hw_hover_y) &&
            now - s_hw_last_hover >= rt_tick_from_millisecond(HW_HOVER_MS))
        {
            char json[40];
            rt_snprintf(json, sizeof(json), "{\"ph\":\"h\",\"p\":[[%d,%d]]}", hx, hy);
            extern bool commu_send_handwrite(const char *json);
            commu_send_handwrite(json);
            s_hw_hover_x = hx;
            s_hw_hover_y = hy;
            s_hw_last_hover = now;
        }
    }
    /* ~1Hz 筆尖狀態 debug(觸控座標=畫布座標,真機對照桌面軌跡用)。 */
    if (now - s_hw_last_dbg >= rt_tick_from_millisecond(1000))
    {
        LOG_I("[handwrite] pen=%d p=(%d,%d)", (int)pen, (int)s_hw_x, (int)s_hw_y);
        s_hw_last_dbg = now;
    }
    s_hw_prev_pen = pen;
}

/* ── 側立圓盤 session ──
   2026-07-20 founder 二改:「側立後**按住畫面**=桌面出現圓盤、放開=執行選項」。
   begin/commit/cancel 由 hid_mouse.c 的觸控 PRESSED/RELEASED/PRESS_LOST 呼叫
   (GUI thread;commu_send 佇列化、跨 thread 安全),姿勢機只做保險 cancel。 */
bool bloc_dial_pose_touch_ready(void)
{
    /* 側立幾何成立=按下畫面該開圓盤(手寫進行中讓位)。用 getter:
       gravity_position 變數宣告在本檔更下方,這裡不在 scope。
       (founder 2026-07-30:圓盤要保留;側立只關「圓盤以外的連續飛鼠」,見
       report_air_mouse_data 的側立 gate。) */
    return get_gravity_position() == GRAVITY_POSITION_FACE_SIDE && !s_hw_active;
}

void mouse_dial_pose_begin(void)
{
    if (s_dial_pose_active || s_hw_active)
        return;
    mouse_dial_start(); /* 送 start(桌面圓盤現形)+清向量(reset_pending) */
    s_dial_pose_active = true;
    s_dial_pose_last_dbg = rt_tick_get();
    motor_pattern_unlocked(); /* 短震=圓盤就緒(側立無手錶 UI,觸覺當回饋) */
    LOG_I("[dial-pose] begin gx100=%d",
          (int)(watch_sensor.motion_data.gravity.x * 100.0f));
}

/* 放開畫面=以當下方向 commit(桌面執行)。冪等。 */
void mouse_dial_pose_commit(void)
{
    if (!s_dial_pose_active)
        return;
    LOG_I("[dial-pose] commit dir=%d mag=%d", s_dial_cur_dir, s_dial_cur_mag);
    mouse_dial_end(); /* 內含清 s_dial_pose_active */
}

/* 取消(不 commit):PRESS_LOST/立起/垂下保險姿態。冪等。 */
void mouse_dial_pose_cancel(void)
{
    if (!s_dial_pose_active)
        return;
    mouse_dial_cancel(); /* 內含清 s_dial_pose_active */
    LOG_I("[dial-pose] cancel");
}

static bool mouse_dial_pose_active(void)
{
    return s_dial_pose_active;
}

/* motion thread(air_mouse_process 每 sample 轉入):按住畫面期間手腕比方向。
   commit=放開畫面(hid_mouse RELEASED→mouse_dial_pose_commit),此處不再有
   「轉回朝上自動 commit」——放開才算數(founder 2026-07-20 二改)。 */
static void dial_pose_motion_process(float gyro_x, float gyro_z)
{
    rt_tick_t now = rt_tick_get();
    Vector3 g = watch_sensor.motion_data.gravity;

    /* 側立幾何還在才累積:離開側立(手臂放回/旋轉過渡)就凍結向量,過渡的角速度
       不污染已比好的方向;放開時 commit 的是凍結前的方向。 */
    if (fabsf(g.x) > DIAL_POSE_FREEZE_GX)
    {
        float rx = (fabsf(gyro_x) < HW_DEADZONE_RADS) ? 0.0f : gyro_x;
        float rz = (fabsf(gyro_z) < HW_DEADZONE_RADS) ? 0.0f : gyro_z;
        s_dial_ax += DIAL_POSE_SIGN_X * rx * DIAL_POSE_GAIN;
        s_dial_ay += DIAL_POSE_SIGN_Y * rz * DIAL_POSE_GAIN;
        mouse_dial_vector_refresh();
    }

    /* 軸向/符號真機定調用:~1Hz。founder 各方向比一下即可對出 DIAL_POSE_SIGN_*。 */
    if (now - s_dial_pose_last_dbg >= rt_tick_from_millisecond(1000))
    {
        LOG_I("[dial-pose] dir=%d mag=%d ax=%d ay=%d gx=%d gz=%d mrad/s",
              s_dial_cur_dir, s_dial_cur_mag, (int)s_dial_ax, (int)s_dial_ay,
              (int)(gyro_x * 1000.0f), (int)(gyro_z * 1000.0f));
        s_dial_pose_last_dbg = now;
    }
}

static void air_mouse_process(rt_uint32_t ts, Quaternion *quaternion,
                              Quaternion *prev_quat)
{
    static air_plane_delta_movement_t delta_movement;

    // if (app_hid_mouse_movement_lock())
    // {
    //     LOG_D("air_mouse_process: mouse movement locked");
    //     return;
    // }

    // Use raw gyroscope data directly (GyroService approach)
    // Convert from dps to rad/s to match GyroService units

    float gyro_z = watch_sensor.imu_data.gyro.z * DPS_TO_RADS;
    float gyro_y =
        watch_sensor.imu_data.gyro.y * DPS_TO_RADS; // 新增Y軸陀螺儀數據
    /* 上下擺腕是繞 x（螢幕橫軸）——y 沿前臂方向是翻腕 roll，
       2026-07-06 logo 飛鼠實機驗出「上下抓錯軸」 */
    float gyro_x = watch_sensor.imu_data.gyro.x * DPS_TO_RADS;

    /* ── 側立圓盤/手寫:gyro 直接餵各自的積分器,不進游標 pipeline。置於一切分支
       之前——期間資料收集/游標 report/moving-state 全不動作。兩者互斥(pose_begin
       gate s_hw_active;手寫 arm 需要按著觸控板,側立時不會)。
       (IMU 座標系翻轉兜底防禦在下面 else 分支,正常永不觸發,這兩個分支不重複。) ── */
    if (s_dial_pose_active)
    {
        dial_pose_motion_process(gyro_x, gyro_z);
        return;
    }
    if (s_hw_active)
    {
        handwrite_motion_process(); /* 觸控筆跡的組幀/節流(座標由 GUI feed) */
        return;
    }

    extern bool imu_mouse_data_collection;
    /* Roll-compensation reference (DATA-COLLECTION MODE ONLY). Captured at the
     * START of a session = the user's holding posture, so the mapping is IDENTITY
     * there (== the raw mapping the real mouse app uses, correct at a normal
     * posture). RELATIVE to this reference — no assumption about which body axis is
     * "up" — so normal posture is always right; only wrist ROLL away from it is
     * compensated, keeping left/right & up/down consistent. */
    if (imu_mouse_data_collection)
    {
        if (s_collect_use_mouse_app_map)
        {
            /* 2026-07-16: collection-mode hold posture reads ~0 on gyro_x for real
             * up/down wrist motion (confirmed on device) — unlike the desktop
             * mouse app's posture, where gyro_x is the responsive vertical axis.
             * Use gyro_y (sign-flippable via AIR_MOUSE_COLLECT_V_SIGN) instead;
             * horizontal stays on gyro_z, unaffected. */
            s_collect_gref_valid = false; /* stale ref shouldn't linger if we switch back */
            delta_movement = air_mouse_algorithm(
                AIR_MOUSE_COLLECT_V_SIGN * gyro_y, gyro_z, AIR_MOUSE_SENSITIVITY);
        }
        else
        {
            /* OLD roll-compensated path — PRESERVED as a fallback (see the selector
             * s_collect_use_mouse_app_map). Uses gyro_y (翻腕 roll 軸) rotated by the
             * gravity-roll relative to the session-start posture. */
            Vector3 g = watch_sensor.motion_data.gravity;
            if (!s_collect_gref_valid)
            {
                s_collect_gref = g;
                s_collect_gref_valid = true;
            }
            float refh = sqrtf(s_collect_gref.y * s_collect_gref.y + s_collect_gref.z * s_collect_gref.z);
            float curh = sqrtf(g.y * g.y + g.z * g.z);
            if (refh > 0.2f && curh > 0.2f) /* both well-defined in the y-z plane */
            {
                /* signed roll from the reference to the current gravity, in the y-z plane */
                float cross = s_collect_gref.y * g.z - s_collect_gref.z * g.y;
                float dot = s_collect_gref.y * g.y + s_collect_gref.z * g.z;
                float roll = AIR_MOUSE_COLLECT_ROLL_SIGN * atan2f(cross, dot);
                float cr = cosf(roll);
                float sr = sinf(roll);
                float h = gyro_z * cr + gyro_y * sr;  /* roll-consistent horizontal rate */
                float v = -gyro_z * sr + gyro_y * cr; /* roll-consistent vertical rate */
                delta_movement = air_mouse_algorithm(-v, h, AIR_MOUSE_SENSITIVITY);
            }
            else
            {
                delta_movement =
                    air_mouse_algorithm(-gyro_y, gyro_z, AIR_MOUSE_SENSITIVITY);
            }
        }
    }
    else
    {
        s_collect_gref_valid = false; /* recapture the start posture next session */
        /* 垂直=繞 x（上下擺腕，正號＝實機兩輪驗出的方向）、水平=繞 z：
           gyro_y 是翻腕 roll 軸、對不上上下（2026-07-06 logo 飛鼠）。
           data-collection 分支有自己的 roll 補償映射，不隨動。 */
        /* IMU 座標系翻轉「兜底」防禦（非主要修法）：dial/飛鼠方向偶發全反的根因已治本——
           以前 BMI270 晶片 remap 只在第一次螢幕睡眠時寫入,開機到那之前是出廠軸=全反窗口;
           2026-07-17 改為 redirect_sensor_data 軟體層固定翻轉、晶片恆 identity(bmi270_driver.c),
           時機問題消除。此段保留當第二道保險:萬一未知來源再讓座標系整組翻(gravity.z 戴姿
           恆 −0.98,變 +0.98=翻了),自動對 gyro_x/z 取負。遲滯 ±0.3 防手腕傾斜抖動;正常
           情況永不觸發。 */
        {
            static bool s_imu_frame_flipped = false;
            float gz_now = watch_sensor.motion_data.gravity.z;
            if (gz_now > 0.3f)       s_imu_frame_flipped = true;
            else if (gz_now < -0.3f) s_imu_frame_flipped = false;
            if (s_imu_frame_flipped)
            {
                gyro_x = -gyro_x;
                gyro_z = -gyro_z;
            }
        }
        delta_movement =
            air_mouse_algorithm(gyro_x, gyro_z, AIR_MOUSE_SENSITIVITY);
    }

    /* 手腕活動追蹤(hid_mouse 長按 armed 判定用):弱門檻=慢跟隨/連動過濾、
       累積量=體感拖曳進入(大震後總移動距離,慢動作也累積)。主路徑常時更新。 */
    {
        float wrist_mag = fabsf((float)delta_movement.x) +
                          fabsf((float)delta_movement.y);
        if (wrist_mag >= DIAL_WRIST_MOVE_UNITS)
            s_dial_wrist_last_move = rt_tick_get();
        /* 手指在滑時凍結累積(founder 2026-07-24「長按手指拖變飛鼠、不能手指滑動」):手指拖
           的反作用力 gyro 不算「體感拖意圖」,否則這副作用就把 wrist_accum 灌到門檻、把手指
           拖誤觸成體感拖(motion_drag)。手指沒滑(純手錶大動)才照常累積=真體感拖。 */
        if (!touch_finger_recent())
            s_wrist_accum += wrist_mag; /* deadzone 已濾雜訊,靜置時 ≈0 不累積 */
    }

    if (abs(delta_movement.x) >= 3 || abs(delta_movement.y) >= 3)
    {
        set_air_mouse_moving_state(true);
    }
    else
    {
        set_air_mouse_moving_state(false);
    }

    // 飛鼠觸發:累積「淨位移」(帶符號),超過閾值才解鎖。手腕來回抖動 dx/dy 正負相抵、淨位移
    // 一直很小=不觸發;真把手錶移到別處淨位移才持續變大→解鎖(founder 2026-07-30)。
    // gyro_movement_distance(總路程)保留給下方 claim owner。
    gyro_movement_distance += abs(delta_movement.x) + abs(delta_movement.y);
    s_air_net_dx += delta_movement.x;
    s_air_net_dy += delta_movement.y;
    /* 觸發門檻:初次從靜止要 80,停頓重鎖後只要 60(founder 2026-07-31)。 */
    float air_move_thr = s_air_retrigger ? GYRO_MOVE_RETRIGGER_THRESHOLD
                                         : GYRO_MOVE_CANCEL_THRESHOLD;
    if (mouse_movement_lock &&
        (abs(s_air_net_dx) + abs(s_air_net_dy)) > air_move_thr)
    {
        mouse_movement_lock = false;
        /* 剛解鎖進飛鼠:以當前淨位移當停頓偵測參考點、起算計時。 */
        s_air_pause_ref_x = s_air_net_dx;
        s_air_pause_ref_y = s_air_net_dy;
        s_air_pause_since = rt_tick_get();
    }
    else if (!mouse_movement_lock)
    {
        /* 飛鼠移動中:淨位移離參考點還在半徑內就累積停頓計時,移出就更新參考點重計時。停在
           小區域滿 0.5s=準備放開 → 重鎖,net 歸零、下次改用重觸發門檻(60),擋放開瞬間微動
           又把游標帶跑(founder 2026-07-31)。 */
        int pause_drift = abs(s_air_net_dx - s_air_pause_ref_x) +
                          abs(s_air_net_dy - s_air_pause_ref_y);
        if (pause_drift > AIR_PAUSE_RADIUS)
        {
            s_air_pause_ref_x = s_air_net_dx;
            s_air_pause_ref_y = s_air_net_dy;
            s_air_pause_since = rt_tick_get();
        }
        else if (s_air_pause_since != 0 &&
                 (rt_tick_get() - s_air_pause_since) >
                     rt_tick_from_millisecond(AIR_PAUSE_HOLD_MS))
        {
            mouse_movement_lock = true;
            s_air_net_dx = 0;
            s_air_net_dy = 0;
            s_air_retrigger = true;
            s_air_pause_since = 0;
        }
    }

    if (imu_mouse_data_collection)
    {
        /* Data-collection mode: always relay the cursor delta to the phone via
         * the 0x08 uplink (not BLE-HID), regardless of FSR/handfree — the game
         * cursor needs continuous motion; clicks are judged phone-side from the
         * fsr_adc already carried in the 0x50 raw stream. */
        if (delta_movement.x != 0 || delta_movement.y != 0)
        {
            extern bool commu_send_mouse_move(int dx, int dy);
            commu_send_mouse_move(delta_movement.x, delta_movement.y);
            delta_movement.x = 0;
            delta_movement.y = 0;
        }
    }
    /* 體感拖曳(長按滿1s後手腕先動=左鍵按住+gyro 驅動游標)與「按住期間自由移動」
       (按下即動手錶=游標直接動,無左鍵;founder 2026-07-20)。report 慣例同飛鼠;
       左鍵狀態由 hid_mouse 管,這裡只負責游標。 */
    else if ((s_motion_drag_active ||
              (s_press_free_move && !s_touch_cursor_owned &&
               !touch_finger_recent())) &&
             !mouse_movement_lock)
    {
        /* 按住期間的自由移動＝互鎖的飛鼠側。仲裁用**手指前哨**(touch_finger_recent):手指
           最近在滑就讓給觸控板,手指沒滑才送飛鼠——直接對應意圖,不再靠 gyro 門檻猜。
           送游標仍只等 mouse_movement_lock(30),手指沒滑時即時跟手錶(founder 2026-07-20
           「按下馬上動手錶游標就要動」);過了前哨這關就即時,不像純 gyro 門檻要慢慢累積
           (founder 2026-07-24「有點太容易變飛鼠」←低門檻 / 「游標停一小段才跟上」←高門檻,
           兩者是同一 gyro 門檻的二選一,改用手指意圖跳出)。
           不再 claim owner(先前「一贏就永久鎖到放開」)——改純前哨動態:飛鼠這條每幀只看
           touch_finger_recent,手指停 250ms 才輪到飛鼠、手指再滑立刻讓回觸控板,同一次按著
           也能來回切(founder 2026-07-30「能切換,但兩個同時做只觸控板」)。 */
        report_air_mouse_data(&delta_movement, ts);
    }
    // handfree 飛鼠的姿態 gate 分兩種來源：①頂部區按住（`hid_mouse_top_fly_active`，明確要
    // 飛鼠）→ 忽略姿態 switch，因為飛鼠靠傾斜手腕、傾斜本來就會讓 gravity 進 freehand/scroll
    // 區間，用姿態擋會讓頂部飛鼠斷掉（founder 2026-07-16 fh=1 sc=1 擋住）；②FSR 壓感觸發的
    // handfree（`bloc_control.c`，手臂/袖子壓到手錶）→ 保留姿態 switch，免得舉手腕時誤壓 FSR
    // 就在極端姿態亂飛（founder 2026-07-16「沒按頂部、舉到角度就變自由模式」）。移動鎖都保留。
    else if (!mouse_movement_lock && !s_touch_cursor_owned &&
             get_hid_mouse_handfree_mode() &&
             (hid_mouse_top_fly_active() ||
              (!switch_freehand_mode && !switch_mouse_scroll_mode)))
    {
        /* 舉起手勢的大麥克風畫面開著時不送游標——語音輸入期間手腕動作不該兼職當游標
           (防禦性 gate,非聚焦被搶 bug 的修法,見 instruction_list_lift_input_view_open 註解)。 */
        extern bool instruction_list_lift_input_view_open(void);
        if (!instruction_list_lift_input_view_open())
        {
            report_air_mouse_data(&delta_movement, ts);
        }
    }
    // else if (!mouse_movement_lock)
    // {
    //     LOG_D("Air mouse locked log: freehand_mode=%d, scroll_mode=%d,
    //     handfree_mode=%d",
    //           switch_freehand_mode, switch_mouse_scroll_mode,
    //           get_hid_mouse_handfree_mode());
    // }
}
#else  /* !BSP_USING_AIR_MOUSE(PC sim 等無 IMU 平台):set_gravity_position 與 hid_mouse
          的引用照編,側立圓盤給 no-op(真機實作在上面 gate 內)。 */
bool bloc_dial_pose_touch_ready(void) { return false; }
void mouse_dial_pose_begin(void) {}
void mouse_dial_pose_commit(void) {}
void mouse_dial_pose_cancel(void) {}
static bool mouse_dial_pose_active(void) { return false; }
#endif

static uint16_t threshold = 3000;
static float gyro_z_count = 2600;
static float gyro_y_count = 2600; // 新增Y軸陀螺儀計數器
static uint16_t media_x_control = 0;
static uint16_t media_y_control = 0;
static uint16_t pevr_media_control[2] = {0, 0};
static rt_tick_t last_vibrate_time_at_boundary = 0;
static float movement_scale_ratio =
    1.0f;                     // 移動比例縮放係數，預設為1.0（不縮放）
#define MOTOR_INTERVAL_MS 300 // Interval in milliseconds

void set_media_control_threshold(uint16_t threshold_value)
{
    threshold = threshold_value;
}

void reset_control_pos(void)
{
    media_x_control = 233;
    pevr_media_control[0] = 0;
    pevr_media_control[1] = 0;
    gyro_z_count = threshold + (threshold / 30);
    gyro_y_count = threshold + (threshold / 30); // 重置Y軸陀螺儀計數器
}

static float control_gravity_x_stard = 0.5f;
static float control_gravity_x_end = -0.5f;
static bool circular_border = true;
void set_control_gravity_x_range(float start, float end, bool is_circular)
{
    circular_border = is_circular;
    control_gravity_x_stard = start;
    control_gravity_x_end = end;
}
static void app_control_interface(Vector3 *gyro, Vector3 *gravity)
{
    // 計算原始座標（不受圓形邊界限制）

    float raw_x, raw_y;
    uint16_t overflow_threshold =
        threshold / 30; // 設定溢出閾值，防止計數器過度增減
    if (gyro->z > 2 || gyro->z < -2)
    {
        if (gyro_z_count <= (threshold + overflow_threshold) * 2)
        {
            gyro_z_count += gyro->z;
            if (gyro_z_count > (threshold + overflow_threshold) * 2)
            {
                gyro_z_count = (threshold + overflow_threshold) * 2;
            }
            else if (gyro_z_count < 0)
            {
                gyro_z_count = 0;
            }
        }
    }
    // 計算 X 座標（基於陀螺儀）
    if (gyro_z_count < overflow_threshold)
    {
        raw_x = 466;
        if (gyro_z_count == 0)
        {
            gyro_z_count = overflow_threshold;
        }
    }
    else if (gyro_z_count >=
             ((threshold + overflow_threshold) * 2 - overflow_threshold))
    {
        raw_x = 0;
        if (gyro_z_count >= (threshold + overflow_threshold) * 2)
        {
            gyro_z_count =
                (threshold + overflow_threshold) * 2 - overflow_threshold;
        }
    }
    else
    {
        raw_x = 466 -
                ((gyro_z_count - overflow_threshold) * 466) / ((threshold) * 2);
    }

    // 計算 Y 座標（基於重力感應）
    if (control_gravity_x_end > gravity->x)
    {
        raw_y = 0;
    }
    else if (control_gravity_x_stard < gravity->x)
    {
        raw_y = 466;
    }
    else if (control_gravity_x_end < gravity->x &&
             gravity->x < control_gravity_x_stard)
    {
        raw_y = 466 * (gravity->x - control_gravity_x_end) /
                (control_gravity_x_stard - control_gravity_x_end);
    }
    // LOG_D("raw_y:%0.3f,gravity->x:%0.3f,stard:%0.3f,end:%0.3f", raw_y,
    // gravity->x, control_gravity_x_stard, control_gravity_x_end);

    if (circular_border)
    {
        // 將座標轉換為相對於圓心的座標（圓心在 233, 233）
        float center_x = 233.0f;
        float center_y = 233.0f;
        float radius = 233.0f; // 圓形半徑

        float relative_x = raw_x - center_x;
        float relative_y = raw_y - center_y;

        // 應用移動比例縮放
        relative_x *= movement_scale_ratio;
        relative_y *= movement_scale_ratio;

        // 計算距離圓心的距離
        float distance =
            sqrtf(relative_x * relative_x + relative_y * relative_y);

        // 如果點在圓形邊界內，直接使用縮放後的座標
        if (distance <= radius)
        {
            media_x_control = (uint16_t)(center_x + relative_x);
            media_y_control = (uint16_t)(center_y + relative_y);
        }
        else
        {
            // 如果點超出圓形邊界，投影到圓形邊界上
            float scale = radius / distance;
            media_x_control = (uint16_t)(center_x + relative_x * scale);
            media_y_control = (uint16_t)(center_y + relative_y * scale);
        }
    }
    else
    {
        // 應用移動比例縮放
        float center_x = 233.0f;
        float center_y = 233.0f;

        float relative_x = raw_x;
        float relative_y = raw_y;

        relative_x *= movement_scale_ratio;
        // relative_y *= movement_scale_ratio;

        media_x_control = (uint16_t)(relative_x);
        media_y_control = (uint16_t)(relative_y);

        if (media_x_control > 466)
            media_x_control = 466;
        if (media_y_control > 466)
            media_y_control = 466;
    }

    if (abs(media_x_control - pevr_media_control[0]) > 5 ||
        abs(media_y_control - pevr_media_control[1]) > 5)
    {
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_WIDGETS_CONTROL;
        msg.data.widgets_control.gesture_position_x = media_x_control;
        msg.data.widgets_control.gesture_position_y = media_y_control;
        // LOG_D("widgets_control.gesture_position_x:%d,y:%d",
        // msg.data.widgets_control.gesture_position_x,
        // msg.data.widgets_control.gesture_position_y);
        lvgl_send_msg(msg);
        pevr_media_control[0] = media_x_control;
        pevr_media_control[1] = media_y_control;
    }
}

static int gravity_position = GRAVITY_POSITION_OTHER;

/* ── 「錶面立起正對臉」姿態 (GRAVITY_POSITION_VERTICAL) 觸發滑鼠 app 麥克風 ──
 * gravity.z = 錶面法線 (switch_freehand_mode = gravity.z < -0.5 = 錶面朝下)。
 * 真機 [POSE-CAL] 校準 (2026-07-06)：
 *   平放錶面朝上 → g≈(0, 0.05, +0.99)
 *   立起正對臉   → g≈(-0.05, +0.96, +0.22)  ← gravity.y 正向主導 (12點朝上)、z 由 1 降下
 * 故判定＝gravity.y 高 + gravity.z 低。y 為「正」向，與既有 SIDE (gravity.y < -0.7)
 * 反向、不相撞；亦與 HORIZONTAL (fabs(gravity.y) < 0.5) 互斥。
 * 去抖：幾何條件連續穩定 ~300ms 才算數，避免滑鼠操作中手腕晃動掃過豎直角就誤觸。*/
#define GRAVITY_VERTICAL_Y_MIN 0.85f  /* gravity.y > 此值 = 錶身直立、12點朝上 (實測 0.92-0.97) */
#define GRAVITY_VERTICAL_Z_MAX 0.50f  /* gravity.z < 此值 = 錶面法線離開朝天 (實測立起 0.20-0.37) */
/* 「錶面轉向側邊」(側立手寫姿,滑鼠 app):重力落在 ±X 主導、面法線離開朝天。與 VERTICAL
   (y 主導)/SIDE(y 負主導)/HORIZONTAL(x 介於 -0.3..0.5)天然互斥。兩側符號先都接受
   (戴法/朝向差異),真機 log([handwrite] begin 的 gx100)鎖定「朝左」符號後再收斂單側。
   去抖同 VERTICAL:幾何連續穩定 ~300ms 才算數,防滑鼠操作中掃過側立角誤觸。 */
#define GRAVITY_FACE_SIDE_X_MIN 0.85f  /* |gravity.x| > 此值 = 錶面轉向側邊 */
#define GRAVITY_FACE_SIDE_Z_MAX 0.50f  /* |gravity.z| < 此值 = 面法線離開鉛直 */
#define GRAVITY_FACE_SIDE_STABLE_MS 300
/* 去抖用「時間」不用「幀數」：motion 更新率會浮動 (實測平放靜止約 3Hz、
 * 動作時更高)，固定幀數的保持時間不可預期 (30 幀 @ 3Hz ≈ 10s 太久)。
 * 記錄進入姿態的 tick，vertical_geom 持續此毫秒數才承認 VERTICAL。*/
#define GRAVITY_VERTICAL_STABLE_MS 300
static rt_tick_t s_vertical_geom_since = 0; /* 0 = 目前不在 vertical_geom 姿態 */
static rt_tick_t s_faceside_geom_since = 0; /* 0 = 目前不在 faceside_geom(側立手寫)姿態 */

int get_gravity_position(void)
{
    return gravity_position;
}

static bool is_ai_open_mic = false;
void reset_ai_open_mic(void)
{
    is_ai_open_mic = false;
}
void set_ai_open_mic(bool is_open)
{
    if (is_open != is_ai_open_mic)
    {
        is_ai_open_mic = is_open;
    }
}
extern void set_is_open_instruction_list_ai(bool open);
extern bool get_is_at_ai_widget(void);
void set_gravity_position(int position)
{
    if (position == gravity_position)
    {
        return;
    }
    LOG_D("gravity_position: %d -> %d", gravity_position, position);
    int prev_gravity_position = gravity_position;
    gravity_position = position;
    /* 2026-07-31 founder：「放下不要直接退出」—— 離開「立起」姿態不再收掉輸入面板。
       原本這裡有一組 prev_gravity_position 邊緣偵測會發 LVGL msg 去關面板，實測就是它在
       使用者「立起帶出面板 → 把手腕放下來看畫面/按按鈕」時把面板關掉的。面板改為只由明確
       動作結束(icon_send / logo / 再點一次底部 bar)，故整條關閉觸發移除。
       (prev_gravity_position 仍留著給下面的其他姿態判斷用。) */
    if (gravity_position == GRAVITY_POSITION_AI &&
        !SkaiWatchSys.motion_control_lock && !is_at_ai_interface() &&
        is_at_instruction_list())
    {
        /* Haptic = "successfully triggered". When disconnected the box won't open
           (animate_open_ai_widget shows the not-connected tip instead) — skip the
           buzz. */
        extern bool get_bluetooth_connection_status(void);
        if (get_bluetooth_connection_status())
            motor_pattern_unlocked();
        extern void animate_open_ai_widget(void);
        animate_open_ai_widget();
    }
    /* 「錶面立起正對臉」→ 滑鼠 app 前景時帶出單設備 skaibar (等同點底部 bar)。
       gui_app_is_actived() 會 assert GUI thread，此處在 motion thread，故只用
       is_at_mouse_mode / app_control_get_mouse_mode 判前景 (兩者皆不 assert)。
       gravity_position 為邊緣觸發 (同 state 早 return)，保持豎直不動不會重觸；
       放平回 OTHER/HORIZONTAL 後再立起才會再觸發。 */
    /* 「立起手錶 → 帶出輸入面板」已於 2026-08-03 退役(founder):語音輸入改成鍵盤模式的
       第四站,入口單一化,姿勢不再是入口。hid_mouse_trigger_skaibar_from_pose /
       open_skaibar_from_pose 連同 LVGL_MSG_TYPE_MOUSE_OPEN_SKAIBAR 一併失去呼叫者,
       比照手寫的處理:程式原地保留、不再被走到。 */
    /* 手寫的姿勢收斂保險已全拆(2026-07-20 founder:「為什麼還會自動執行」——錶面
       直寫時手腕角度千變萬化,VERTICAL/SIDE/FACE_SIDE 邊緣頻繁誤中,舊保險=寫到
       一半被強制 end 自動送出;結束現在只走退出/輸入鈕+離開 app 取消)。 */
    if (gravity_position == GRAVITY_POSITION_VERTICAL ||
        gravity_position == GRAVITY_POSITION_SIDE)
    {
        /* 側立圓盤收斂保險:立起/垂下=顯然不比了,取消(不 commit)。commit 只走
           放開畫面。放在上面 skaibar 分支之後:同一次 VERTICAL 邊緣先被 gate
           擋掉召喚、這裡才清旗標。 */
        mouse_dial_pose_cancel();
    }
#ifdef SHOW_UNGRAB_ENABLE_INDICATOR
    uint8_t app_id = app_id_mainmenu;
    if (is_at_home() && !is_at_message())
    {
        if (gravity_position == 1)
        {
            app_id = get_quick_open_app(0);
        }
        else if (gravity_position == 2)
        {
            app_id = get_quick_open_app(1);
        }
    }
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_RELEASE_INDICATOR;
    msg.data.action = app_id;
    lvgl_send_msg(msg);
#endif
}

void reset_gravity_position(void)
{
    gravity_position = GRAVITY_POSITION_OTHER;
#ifdef SHOW_UNGRAB_ENABLE_INDICATOR
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_RELEASE_INDICATOR;
    msg.data.message = APP_ID_MAIN;
    lvgl_send_msg(msg);
#endif
}

static bool can_open_ai_interface(void)
{
    if (is_at_home() || is_at_control_center() || is_at_mouse_mode() ||
        gui_app_is_actived(APP_ID_FLASHLIGHT) ||
        gui_app_is_actived(APP_ID_TIMER) || gui_app_is_actived(APP_ID_MOUSE) ||
        gui_app_is_actived(APP_ID_EXERCISE) ||
        gui_app_is_actived(APP_ID_GESTURE) ||
        gui_app_is_actived(APP_ID_RECORDER) ||
        (!is_ai_open_mic && app_voice_get_listening_status()))
    {
        return false;
    }
    return true;
}

extern void level_bar_update(int16_t value);
static uint8_t pevr_ai_hint_bg_pos = 0;
static void calculate_gravity_position(Vector3 *gravity)
{
    if (gravity->x > -1 && gravity->x < 1 && is_at_home())
    {
        level_bar_update(
            (int16_t)(gravity->x *
                      -100)); // Update level bar based on gravity x-axis
    }
    /* 「錶面立起正對臉」時間去抖：vertical_geom 幾何條件要「持續」
       GRAVITY_VERTICAL_STABLE_MS 毫秒才承認為 VERTICAL，避免滑鼠操作中手腕瞬間
       掃過豎直角度就誤觸 mic。用 wall-clock tick 不用幀數 — motion 更新率浮動，
       固定幀數保持時間不可預期。其他姿態 (SIDE/HORIZONTAL) 維持即時切換不變。
       2026-07-17：founder 要求先還原這版本自己驗證，之前拿掉去抖動的版本先不用。 */
    bool vertical_geom = (gravity->y > GRAVITY_VERTICAL_Y_MIN &&
                          gravity->z < GRAVITY_VERTICAL_Z_MAX);
    if (vertical_geom)
    {
        if (s_vertical_geom_since == 0)
            s_vertical_geom_since = rt_tick_get();
    }
    else
    {
        s_vertical_geom_since = 0;
    }
    rt_tick_t vertical_held =
        (s_vertical_geom_since == 0) ? 0 : (rt_tick_get() - s_vertical_geom_since);
    bool vertical_stable =
        vertical_geom &&
        (vertical_held >= rt_tick_from_millisecond(GRAVITY_VERTICAL_STABLE_MS));

    /* 「錶面轉向側邊」(側立手寫姿) 幾何+去抖,同 vertical 的 wall-clock 模式。 */
    bool faceside_geom = (fabsf(gravity->x) > GRAVITY_FACE_SIDE_X_MIN &&
                          fabsf(gravity->z) < GRAVITY_FACE_SIDE_Z_MAX);
    if (faceside_geom)
    {
        if (s_faceside_geom_since == 0)
            s_faceside_geom_since = rt_tick_get();
    }
    else
    {
        s_faceside_geom_since = 0;
    }
    bool faceside_stable =
        faceside_geom && (s_faceside_geom_since != 0) &&
        ((rt_tick_get() - s_faceside_geom_since) >=
         rt_tick_from_millisecond(GRAVITY_FACE_SIDE_STABLE_MS));

    int decided_position;
    if (gravity->y < -0.7 && gravity->z < 0.3)
    {
        decided_position = GRAVITY_POSITION_SIDE;
    }
    else if (vertical_stable)
    {
        decided_position = GRAVITY_POSITION_VERTICAL;
    }
    else if (faceside_stable)
    {
        decided_position = GRAVITY_POSITION_FACE_SIDE;
    }
    else if ((gravity->x < 0.5 && gravity->x > -0.3) && fabs(gravity->y) < 0.5)
    {
        decided_position = GRAVITY_POSITION_HORIZONTAL;
    }
    else
    {
        decided_position = GRAVITY_POSITION_OTHER;
    }
    set_gravity_position(decided_position);
    // if (gravity->x < 0.5 && get_is_open_instruction_list_ai())
    // {
    //     extern void check_ai_widget_auto_close(void);
    //     check_ai_widget_auto_close();
    // }
}

/* ── 錶盤長按 + 水平左右搖兩下 → 開滑鼠 app ─────────────────────────────────
   (founder 2026-08-04)手指按住錶面不放 → 長按 arm → 期間水平左右擺兩下就開。
   arm/disarm 全由錶盤 catcher 驅動(app_clock_main.c;手指一滑動或放開就 disarm),
   本段只在 armed 時看 IMU,平時零成本。

   「水平擺」判定用**角速度在重力方向的投影** ω_v = ω·ĝ (ĝ=重力單位向量):
   那就是「繞世界垂直軸轉多快」,與手當下把錶拿成什麼角度無關(平放/斜著/立起
   都準);上下擺腕與翻腕 roll 都是繞水平軸,投影≈0,不會誤觸。
   計數只認**方向交替**的脈衝:左→右 = 兩下(founder 2026-08-04 由三下改二)。
   同方向連著擺不算來回,重新起算。

   門檻是真機待調的三個旋鈕 —— disarm 時會 log 這次長按期間的 |ω_v| 峰值,
   founder 搖了沒開就看那個數字改 SHAKE_PEAK_DPS。 */
#define SHAKE_PEAK_DPS 120.0f /* 一下「擺動」要達到的角速度峰值(度/秒) */
#define SHAKE_RESET_DPS 40.0f /* 回落到此以下才重新武裝同方向(遲滯,防抖動連計) */
#define SHAKE_NEED_PULSES 2   /* 兩下 = 兩個方向交替的脈衝(左→右,一個來回) */
#define SHAKE_GAP_MS 700      /* 兩下間隔超過此 = 不是連續搖晃,計數從頭 */

static volatile bool s_shake_armed = false; /* GUI thread 寫、motion thread 讀 */
static int s_shake_pulses = 0;      /* 已計入的脈衝數 */
static int s_shake_dir = 0;         /* 當前脈衝方向(+1/-1),0=已回落到靜止帶 */
static int s_shake_last_dir = 0;    /* 最後計入的脈衝方向 */
static rt_tick_t s_shake_last_pulse = 0;
static float s_shake_peak_dps = 0.0f; /* 本次 arm 期間 |ω_v| 峰值(調參用) */

/* GUI thread(錶盤 catcher):長按=on、手指滑動/放開/錶盤收掉=off。 */
void bloc_watchface_shake_arm(bool on)
{
    if (on == s_shake_armed)
    {
        return;
    }
    if (!on && s_shake_armed)
    {
        LOG_I("[wf-shake] disarm pulses=%d peak=%d dps", s_shake_pulses,
              (int)s_shake_peak_dps);
    }
    s_shake_armed = on;
    s_shake_pulses = 0;
    s_shake_dir = 0;
    s_shake_last_dir = 0;
    s_shake_last_pulse = 0;
    s_shake_peak_dps = 0.0f;
}

/* motion thread(每個 IMU sample)。armed 才做事。 */
static void watchface_shake_process(motion_data_t *motion_data)
{
    if (!s_shake_armed)
    {
        return;
    }

    Vector3 *g = &motion_data->gravity;
    Vector3 *w = &watch_sensor.imu_data.gyro; /* dps */
    float wv = w->x * g->x + w->y * g->y + w->z * g->z;
    float mag = fabsf(wv);
    if (mag > s_shake_peak_dps)
    {
        s_shake_peak_dps = mag;
    }

    if (mag < SHAKE_RESET_DPS)
    {
        s_shake_dir = 0; /* 回靜止帶:下次超門檻算新的一下 */
        return;
    }
    if (mag < SHAKE_PEAK_DPS)
    {
        return; /* 遲滯帶之間:不計數也不重新武裝 */
    }

    int dir = (wv > 0) ? 1 : -1;
    if (dir == s_shake_dir)
    {
        return; /* 同一下的持續期間,只計一次 */
    }
    s_shake_dir = dir;

    rt_tick_t now = rt_tick_get();
    if (s_shake_last_dir == 0 ||
        (now - s_shake_last_pulse) > rt_tick_from_millisecond(SHAKE_GAP_MS) ||
        dir == s_shake_last_dir)
    {
        /* 第一下 / 隔太久 / 同向沒來回 → 從這一下重新起算 */
        s_shake_pulses = 1;
    }
    else
    {
        s_shake_pulses++;
    }
    s_shake_last_dir = dir;
    s_shake_last_pulse = now;
    LOG_I("[wf-shake] pulse=%d dir=%d wv=%d dps", s_shake_pulses, dir, (int)wv);

    if (s_shake_pulses >= SHAKE_NEED_PULSES)
    {
        LOG_I("[wf-shake] triggered -> open mouse app (peak=%d dps)",
              (int)s_shake_peak_dps);
        s_shake_armed = false; /* 一次長按只觸發一次;放開再長按才會重新 arm */
        s_shake_pulses = 0;
        s_shake_dir = 0;
        s_shake_last_dir = 0;
        motor_pattern_unlocked(); /* 短震=收到了(錶盤沒有其他回饋) */
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_WATCHFACE_SHAKE_OPEN_MOUSE;
        lvgl_send_msg(msg); /* gui_app_run 必須在 LVGL thread */
    }
}

static Quaternion tap_state_q;
static Quaternion tap_state_q_prev;
static bool tap_state_q_flag = false;

static euler_angle_t befor_tap_delta_angle = {
    .yaw = 0.0f, .pitch = 0.0f, .roll = 0.0f};
static float befor_tap_delta_max_angle = 0.0f;

// 使用者正在移動手臂
bool has_user_started_controlling_with_arm(void)
{
    if (get_enable_tap_and_hold())
    {
        double delta_angle_abs =
            fabs(befor_tap_delta_angle.yaw); // fabs(befor_tap_delta_max_angle)
        return delta_angle_abs >= STARTED_MOVE_THRESHOLD;
    }
    else
    {
        return false;
    }
}

bool _free_control_with_arm = false;
bool free_control_with_arm(void)
{
    return _free_control_with_arm;
}

void set_free_control_with_arm(bool flag)
{
    if (flag != _free_control_with_arm)
    {
        LOG_I("set_free_control_with_arm:%d", flag);
        _free_control_with_arm = flag;
    }
}

static bool open_control_options = false;
void set_open_control_options(bool open)
{
    if (open != open_control_options)
    {
        LOG_I("set_open_control_options:%d", open);
        open_control_options = open;
    }
}

static bool paused_control_with_arm = false;
extern void set_indicator_dots_visible(bool visible);
void set_paused_control_with_arm(bool paused)
{
    if (paused != paused_control_with_arm)
    {
        LOG_I("set_paused_control_with_arm:%d", paused);
        paused_control_with_arm = paused;
        // set_indicator_dots_visible(!paused);
    }
}

void set_stop_mouse_move(bool stop)
{
    stop_mouse_move = stop;
}

static uint8_t test_log_count = 0;
extern uint8_t get_message_page_count(void);
static euler_angle_t pevr_befor_switch_widget_delta_angle;
static float prev_delta_roll = 0.0f;
static float prev_delta_yaw = 0.0f;
static void motion_tracking_in_hcpu(motion_data_t *motion_data)
{
    static Quaternion prev_global_quat = {.w = 1, .x = 0, .y = 0, .z = 0};
    watch_sensor_motion_data = motion_data;
#if ENABLE_WAVEFORM_CAPTURE
    // Process waveform capture for gesture recognition (migrated from LCPU)
    // This runs regardless of UI state to capture gestures consistently
    if (!is_sleep_mode())
    {
        waveform_capture_process(motion_data, &watch_sensor.imu_data.gyro);
    }
#endif
    calculate_gravity_position(&motion_data->gravity);
    /* 錶盤長按+搖兩下:錶盤就是 is_at_home,必須放在下面那個 early return 之前。
       armed 才做事(arm 只發生在錶面被長按時),平時是一個 bool 判斷。 */
    watchface_shake_process(motion_data);
    if (is_at_home() && !is_at_speech_interface() && !is_at_control_center())
    {
        return;
    }
    // stop_mouse_move = motion_data->gravity.y < -0.4;

    if (get_enable_tap_and_hold())
    {
        if (!tap_state_q_flag && peripheral_provider.get_tap_status())
        {
            tap_state_q = motion_data->sensor_q;
            tap_state_q_flag = true;
        }
        else if (tap_state_q_flag && !peripheral_provider.get_tap_status())
        {
            tap_state_q_flag = false;
            befor_tap_delta_max_angle = 0.0f;
            befor_tap_delta_angle.yaw = 0.0f;
            befor_tap_delta_angle.pitch = 0.0f;
            befor_tap_delta_angle.roll = 0.0f;
        }
    }

#ifdef BSP_USING_AIR_MOUSE
    extern bool imu_mouse_data_collection;
    /* Invalidate the collection roll reference whenever collection is OFF, so every
     * session recaptures a FRESH start posture. This handler runs every frame;
     * air_mouse_process does NOT when the gesture app is foreground with collection
     * off, so it can't reset the reference itself — a stale reference from the
     * previous session is exactly why the direction "reversed again" next game. */
    if (!imu_mouse_data_collection)
    {
        s_collect_gref_valid = false;
    }
    /* imu_mouse_data_collection first so the short-circuit skips the
     * gui_app_is_actived() call (asserts on the GUI thread) while collecting. */
    if (imu_mouse_data_collection || app_control_get_mouse_mode() ||
        is_at_mouse_mode() || gui_app_is_actived(APP_ID_MOUSE))
    {
        air_mouse_process(motion_data->timestamp, &motion_data->global_q,
                          &prev_global_quat);
        prev_global_quat = motion_data->global_q;
        switch_freehand_mode = motion_data->gravity.z < -0.5f;
        switch_mouse_scroll_mode = motion_data->gravity.y < -0.7f;
        scroll_up_mode = motion_data->gravity.x >= 0;
    }
#endif
    else
    {
        euler_angle_t delta_senor_angle = motion_tracking_algorithm(
            &motion_data->sensor_q, &prev_sensor_quat);
        // prev_sensor_quat = motion_data->sensor_q;
        if (SkaiWatchSys.idle_state || !SkaiWatchSys.flag_field.is_wearing ||
            SkaiWatchSys.motion_control_lock)
        {
            return;
        }

        if (!is_at_message() && !get_is_open_instruction_list_ai())
        {
            set_paused_control_with_arm(!(motion_data->gravity.x < 0.3 &&
                                          motion_data->gravity.x > -0.3));
        }

        if (app_control_get_motion_tracking())
        {
            if (free_control_with_arm() &&
                !is_at_ai_interface()) //&& !is_user_touching_screen()
            {
                // 上下
                float diff_delta_roll =
                    fabs(delta_senor_angle.roll - prev_delta_roll);
                // 左右
                float diff_delta_yaw =
                    fabs(delta_senor_angle.yaw - prev_delta_yaw);
                if (!paused_control_with_arm)
                {
                    if (!get_enable_tap_and_hold() ||
                        peripheral_provider.get_tap_status())
                    {
                        if (diff_delta_roll < diff_delta_yaw * 0.8)
                        {
                            navigation_bar_control_with_euler_angle(
                                &delta_senor_angle, motion_data);
                        }
                    }
                }
                else
                {
                    set_prev_sensor_quat(total_yaw_energy_uint);
                }
                prev_delta_roll = delta_senor_angle.roll;
                prev_delta_yaw = delta_senor_angle.yaw;
                if (open_control_options && !is_at_ai_interface())
                {
                    app_control_interface(&watch_sensor.imu_data.gyro,
                                          &motion_data->gravity);
                }
            }
            else if (open_control_options && !is_at_ai_interface())
            {
                app_control_interface(&watch_sensor.imu_data.gyro,
                                      &motion_data->gravity);
            }
        }

        if (get_enable_tap_and_hold())
        {
            if (peripheral_provider.get_tap_status())
            {
                // 計算當前四元數與上一次手指點擊時四元數的差值
                befor_tap_delta_angle = motion_tracking_algorithm(
                    &motion_data->sensor_q, &tap_state_q);
                double delta_angle_abs = fabs(befor_tap_delta_angle.yaw);
                if (delta_angle_abs > befor_tap_delta_max_angle)
                {
                    befor_tap_delta_max_angle = delta_angle_abs;
                }
            }
        }
    }
}

/*
 ***** Motion Tracking processing
 */
static void motion_tracking_thread_entry(void *parameter)
{
    watch_sensor.imu_sem = rt_sem_create("imu_sem", 0, RT_IPC_FLAG_FIFO);
    while (1)
    {
        rt_sem_take(watch_sensor.imu_sem, RT_WAITING_FOREVER);
        /* GUI suspended (screen off): everything downstream of
           motion_tracking_in_hcpu is already gated by is_sleep_mode() —
           which is just !gui_is_active() on HCPU. Skip the call entirely
           so the thread returns to sleep without doing the wrapper work. */
        if (!gui_is_active())
        {
            continue;
        }
        motion_data_t motion_data = watch_sensor.motion_data;
        motion_tracking_in_hcpu(&motion_data);
    }
}

static int motion_tracking_thread_init(void)
{
    motion_tracking_thread = rt_thread_create(
        "motion_tracking", motion_tracking_thread_entry, RT_NULL,
        MOTION_TRACKING_THREAD_STACK_SIZE, MOTION_TRACKING_THREAD_PRIORITY,
        MOTION_TRACKING_THREAD_TIMESLICE);
    if (motion_tracking_thread != RT_NULL)
    {
        rt_thread_startup(motion_tracking_thread);
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}
INIT_APP_EXPORT(motion_tracking_thread_init);

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/