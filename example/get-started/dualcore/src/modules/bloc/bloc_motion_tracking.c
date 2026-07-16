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
    // Sliding window and threshold constants
    #define ACCEL_WINDOW_SIZE 15
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

// Static variables for waveform capture
static gesture_dataset_t tap_dataset = {0};
static gesture_dataset_t release_dataset = {0};
static waveform_gesture_state_t waveform_gesture_state = {0};
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

static bool stop_mouse_move = false;

#ifdef BSP_USING_AIR_MOUSE
static void air_mouse_process(rt_uint32_t ts, Quaternion *quaternion,
                              Quaternion *prev_quat);
#endif

static float total_yaw_energy = 0;
static uint8_t scroll_segment_count = 1;
static uint16_t page_range = 100; // 每個頁面的範圍
static float total_moving_distance = 1100.0f;
static uint8_t control_angle = 60; // 預設控制角度為30度
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

extern int get_gesture_recognition_threshold(void);
static void gesture_event_capture_hcpu(uint16_t freq, time_t ts,
                                       Vector3 *linear_acce, Vector3 *gyro,
                                       Vector3 *gravity, uint32_t ppg,
                                       rt_uint32_t fsr_adc_value,
                                       waveform_gesture_state_t *state,
                                       gesture_type_t type,
                                       gesture_dataset_t *dataset)
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
        reset_gesture_state(dataset, current_time, type, 2);
        return;
    }
    else if (state->gyro_lock_status && !imu_data_collection)
    {
        reset_gesture_state(dataset, current_time, type, 3);
        return;
    }

    int target_samples = 0;
    float start_threshold = 0.0f;
    if (type == GESTURE_TYPE_TAP)
    {
        target_samples = GESTURE_TAP_TIME_STEP;
        start_threshold = TAP_START_THRESHOLD;
    }
    else if (type == GESTURE_TYPE_RELEASE)
    {
        target_samples = GESTURE_RELEASE_TIME_STEP;
        start_threshold = RELEASE_START_THRESHOLD;
    }

    if (!dataset->gesture_started && !dataset->gesture_ended)
    {
        if ((waveform_gesture_state.difference_accel > start_threshold &&
             !open_ppg_chacked) ||
            (ppg_diff_rawdata > get_gesture_recognition_threshold() &&
             open_ppg_chacked))
        {
            if (ppg_diff_rawdata > get_gesture_recognition_threshold() &&
                open_ppg_chacked)
                LOG_D("PPG DIFF");
            dataset->gesture_started = true;
            int feedback_samples = 0;
            if (type == GESTURE_TYPE_TAP)
            {
                feedback_samples = FEEDBACK_ACCEL_SAMPLES_FOR_TAP;
            }
            else if (type == GESTURE_TYPE_RELEASE)
            {
                feedback_samples = FEEDBACK_ACCEL_SAMPLES_FOR_RELEASE;
            }
            for (int i = 0; i < feedback_samples; i++)
            {
                int accel_index = ACCEL_WINDOW_SIZE - 1 - feedback_samples + i;
                store_gesture_sample(
                    dataset, ts, &state->sliding_window_accel[accel_index],
                    &state->sliding_window_gravity[accel_index],
                    state->sliding_window_ppg[accel_index],
                    state->sliding_window_fsr_adc[accel_index],
                    state->on_pressed);
            }
        }
    }

    if (dataset->gesture_started)
    {
        store_gesture_sample(dataset, ts, linear_acce, gravity, ppg,
                             fsr_adc_value, state->on_pressed);
        if (dataset->gesture_sample_count >= target_samples)
        {
            dataset->gesture_ended = true;
            static rt_tick_t median_lock_trigger_time = 0;
            float median_difference_accel =
                calculate_median_difference_accel(75);
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
            if (is_gesture && (user_hand_horizontal ||
                               type == GESTURE_TYPE_TAP || imu_data_collection))
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
            reset_gesture_state(dataset, current_time, type, 7);
        }
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
        app_control_get_mouse_mode();

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

    if (app_control_get_game_mode())
    {
        gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                   linear_acce, gyro, gravity, ppg_rawdata,
                                   fsr_adc_value, &waveform_gesture_state,
                                   GESTURE_TYPE_TAP, &tap_dataset);
        gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                   linear_acce, gyro, gravity, ppg_rawdata,
                                   fsr_adc_value, &waveform_gesture_state,
                                   GESTURE_TYPE_RELEASE, &release_dataset);
    }
    else
    {
        if (SkaiWatchSys.motion_control_lock)
        {
            gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                       linear_acce, gyro, gravity, ppg_rawdata,
                                       fsr_adc_value, &waveform_gesture_state,
                                       GESTURE_TYPE_RELEASE, &release_dataset);
        }
        else
        {
            gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                       linear_acce, gyro, gravity, ppg_rawdata,
                                       fsr_adc_value, &waveform_gesture_state,
                                       GESTURE_TYPE_TAP, &tap_dataset);
        }
    }
}

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
static void report_air_mouse_data(air_plane_delta_movement_t *movement,
                                  rt_uint32_t ts)
{
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
    control_provider.ble_hid_mouse_move(movement->x, movement->y);
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
    // 移動鎖：觸碰面板後累積移動量超過此閾值才解鎖
    #define GYRO_MOVE_CANCEL_THRESHOLD 30.0f
    // Roll-compensation sign for data-collection air-mouse (see air_mouse_process).
    // Flip to -1.0f on device if roll compensation goes the WRONG way.
    #define AIR_MOUSE_COLLECT_ROLL_SIGN 1.0f
    // Vertical-axis sign for the collection air-mouse's gyro_y mapping (2026-07-16:
    // gyro_x read ~0 for up/down in the collection hold posture — real device
    // testing found gyro_y is the responsive axis there). Flip to +1.0f on device
    // if up/down comes out reversed.
    #define AIR_MOUSE_COLLECT_V_SIGN -1.0f

static bool mouse_movement_lock = false;
static float gyro_movement_distance = 0.0f;

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
}

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

/* ─────────────────────────────────────────────────────────────────────────
   主觸控板長按方向盤 (dial) —— 頂部 logo「按住=飛鼠」的姊妹功能。
   手指按住主觸控板不動 ~250ms（hid_mouse.c 的手勢狀態機開啟）後，air-mouse 的
   位移 delta 不再移動游標，而是「累積成一個向量」：向量角度 → 8 向 sector、
   向量長度 → 力度 mag。節流上傳 KEY_DIAL_DIR(0x1a)，桌面以游標為中心畫圓盤、
   對應扇形高亮。放開時以當下 sector commit。
   dir 0=上，順時針 1=右上 2=右 3=右下 4=下 5=左下 6=左 7=左上；len<deadzone → -1。
   ───────────────────────────────────────────────────────────────────────── */
#define DIAL_DEADZONE_UNITS   40.0f  /* 累積向量長度 < 此 = 中心 deadzone（不選）*/
#define DIAL_MAX_UNITS       260.0f  /* 向量長度上限（mag=1000）；超過 clamp。真機再調 */
#define DIAL_SEND_INTERVAL_MS   33   /* update 節流 ~30Hz，避免洗版 BLE/log */

static volatile bool s_dial_active = false;
static bool  s_dial_reset_pending = false;
static float s_dial_ax = 0.0f, s_dial_ay = 0.0f; /* 累積向量：螢幕座標 x 右+ / y 下+ */
static int   s_dial_cur_dir = -1;
static int   s_dial_cur_mag = 0;
static rt_tick_t s_dial_last_send = 0;

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

/* air_mouse_process 每幀在 dial 模式呼叫：累積 delta、更新當前 sector、節流送 update。 */
static void mouse_dial_accumulate(float dx, float dy)
{
    if (s_dial_reset_pending)
    {
        s_dial_ax = 0.0f; s_dial_ay = 0.0f;
        s_dial_cur_dir = -1; s_dial_cur_mag = 0;
        s_dial_reset_pending = false;
    }
    s_dial_ax += dx;
    s_dial_ay += dy;
    /* clamp 累積向量長度，避免長按久了積分無限增長、放開後方向遲鈍 */
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

/* hid_mouse.c 的觸控板長按狀態機呼叫這三個（extern）。 */
void mouse_dial_start(void)
{
    s_dial_reset_pending = true;
    s_dial_active = true;
    s_dial_last_send = 0; /* 讓開場那一幀的 update 立即送出 */
    extern bool commu_send_dial_dir(const char *phase, int dir, int mag);
    commu_send_dial_dir("start", -1, 0);
}
void mouse_dial_end(void)
{
    if (!s_dial_active) return;
    s_dial_active = false;
    extern bool commu_send_dial_dir(const char *phase, int dir, int mag);
    commu_send_dial_dir("end", s_dial_cur_dir, s_dial_cur_mag);
}
bool mouse_dial_active(void) { return s_dial_active; }

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
        delta_movement =
            air_mouse_algorithm(gyro_x, gyro_z, AIR_MOUSE_SENSITIVITY);
    }

    /* ── 主觸控板長按方向盤 (dial)：手指按住不動時 air-mouse 位移不移游標，改累積
       成方向向量→8 向+力度上傳 0x1a（桌面畫圓盤）。dial 期間 handfree 為 false，
       下面的游標 report gate 本就不觸發；這裡提早 return 略過 moving-state/lock。 ── */
    if (mouse_dial_active())
    {
        mouse_dial_accumulate((float)delta_movement.x, (float)delta_movement.y);
        return;
    }

    if (abs(delta_movement.x) >= 3 || abs(delta_movement.y) >= 3)
    {
        set_air_mouse_moving_state(true);
    }
    else
    {
        set_air_mouse_moving_state(false);
    }

    // 累積移動量，超過閾值才解鎖
    gyro_movement_distance += abs(delta_movement.x) + abs(delta_movement.y);
    if (mouse_movement_lock &&
        gyro_movement_distance > GYRO_MOVE_CANCEL_THRESHOLD)
    {
        mouse_movement_lock = false;
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
    // 按住面板才可體感移動，且移動鎖已解除
    else if (!mouse_movement_lock && get_hid_mouse_handfree_mode() &&
        !switch_freehand_mode &&
        !switch_mouse_scroll_mode) //! stop_mouse_move &&
                                   //! is_skai_touch_enabled() &&
    {
        report_air_mouse_data(&delta_movement, ts);
    }
    // else if (!mouse_movement_lock)
    // {
    //     LOG_D("Air mouse locked log: freehand_mode=%d, scroll_mode=%d,
    //     handfree_mode=%d",
    //           switch_freehand_mode, switch_mouse_scroll_mode,
    //           get_hid_mouse_handfree_mode());
    // }
}
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
/* 去抖用「時間」不用「幀數」：motion 更新率會浮動 (實測平放靜止約 3Hz、
 * 動作時更高)，固定幀數的保持時間不可預期 (30 幀 @ 3Hz ≈ 10s 太久)。
 * 記錄進入姿態的 tick，vertical_geom 持續此毫秒數才承認 VERTICAL。*/
#define GRAVITY_VERTICAL_STABLE_MS 300
static rt_tick_t s_vertical_geom_since = 0; /* 0 = 目前不在 vertical_geom 姿態 */

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
    LOG_D("gravity_position:%d", position);
    gravity_position = position;
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
    if (gravity_position == GRAVITY_POSITION_VERTICAL &&
        (is_at_mouse_mode() || app_control_get_mouse_mode()))
    {
        extern void hid_mouse_trigger_skaibar_from_pose(void);
        hid_mouse_trigger_skaibar_from_pose();
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
    // LOG_D("gravity x:%f,y:%f,z:%f", gravity->x, gravity->y, gravity->z);
    if (gravity->x > -1 && gravity->x < 1 && is_at_home())
    {
        level_bar_update(
            (int16_t)(gravity->x *
                      -100)); // Update level bar based on gravity x-axis
    }
    /* 「錶面立起正對臉」時間去抖：vertical_geom 幾何條件要「持續」
       GRAVITY_VERTICAL_STABLE_MS 毫秒才承認為 VERTICAL，避免滑鼠操作中手腕瞬間
       掃過豎直角度就誤觸 mic。用 wall-clock tick 不用幀數 — motion 更新率浮動，
       固定幀數保持時間不可預期。其他姿態 (SIDE/HORIZONTAL) 維持即時切換不變。 */
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

    if (gravity->y < -0.7 && gravity->z < 0.3)
    {
        set_gravity_position(GRAVITY_POSITION_SIDE);
    }
    else if (vertical_stable)
    {
        set_gravity_position(GRAVITY_POSITION_VERTICAL);
    }
    else if ((gravity->x < 0.5 && gravity->x > -0.3) && fabs(gravity->y) < 0.5)
    {
        set_gravity_position(GRAVITY_POSITION_HORIZONTAL);
    }
    else
    {
        set_gravity_position(GRAVITY_POSITION_OTHER);
    }
    // if (gravity->x < 0.5 && get_is_open_instruction_list_ai())
    // {
    //     extern void check_ai_widget_auto_close(void);
    //     check_ai_widget_auto_close();
    // }
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