/**
 ******************************************************************************
 * @file   gesture_detect.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
#include <rtthread.h>
#include <board.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "acce_service.h"
#include "gesture_detect.h"
#include "hand_tracking.h"
#include "watch_sys_service.h"
#include "bsp_board.h"
#ifdef BSP_USING_MAHONY_AHRS
    #include "sensor_fusion.h"
#endif
#include "bloc_peripheral.h"
#include "bloc_battery.h"
#ifdef BSP_USING_GESTURE_HANDLER
    #include "gesture_handler.h"
#endif
#ifdef BSP_USING_HEALTH_ALGO
    #include "health_algo.h"
#endif
#ifdef BSP_USING_ACTIVITY_ALGO_KRAEPELIN
    #include "activity.h"
#endif

#define DBG_TAG "gesture.detect"
#define DBG_LVL DBG_INFO
#include "rtdbg.h"

// Configuration constants
#define USE_PPG_VARIANCE 0
#define TAP_DETECT_TIME 500
#define PIN_DEBOUNCE_TIME_MS 10
// 專屬於本函式的狀態與 buffer，與原有算法完全隔離
#define ACCEL_WINDOW_SIZE 15
#define GYRO_WINDOW_SIZE 10
#define GYRO_LOCK_THRESHOLD 2000.0f
#define GESTURE_RELEASE_COOLDOWN_PERIOD_MS 100
#define GESTURE_COLLECTION_COOLDOWN_PERIOD_MS 500
#define GESTURE_TAP_COOLDOWN_PERIOD_MS 100
#define MAX_GESTURE_DURATION_MS 160
#define MIN_GESTURE_SAMPLES 10
#define MIN_DIFFERENCE_ACCEL_MAX 1.0f
#define PPG_ANOMALY_DISABLE_DURATION_MS 1000
#define PPG_FILTER_SAMPLE_NUM 10
static float ppg_buffer[PPG_FILTER_SAMPLE_NUM + 1];

#define GESTURE_EVENT_TAP (1 << 1)
#define GESTURE_EVENT_HOLD (1 << 2)
#define GESTURE_EVENT_FINGER_RELEASE (1 << 3)
#define GESTURE_EVENT_FORCE_RELEASE (1 << 4)
#define GESTURE_EVENT_BACK (1 << 6)
// Gesture types
typedef enum
{
    GESTURE_TAP,
    GESTURE_RELEASE,
} gesture_type_t;

// Gesture state structure
typedef struct
{
    Vector3 sliding_window_accel[ACCEL_WINDOW_SIZE];
    Vector3 sliding_window_gravity[ACCEL_WINDOW_SIZE];
    float gyro_sliding_window[GYRO_WINDOW_SIZE];
    uint8_t gyro_count;
    bool if_watchface_visible;
    bool gyro_lock_status;
    float difference_accel;
    bool on_pressed;
} gesture_state_t;

static bool user_hand_horizontal = false;
static gesture_dataset_t tap_dataset = {0};
static gesture_dataset_t release_dataset = {0};
static gesture_state_t my_gesture_state = {0};
static watch_sys_linear_acce_t targetWave_algo[MAX_RAWDATA_TIME_STEP];
static void gesture_event_capture(uint16_t freq, time_t ts,
                                  Vector3 *linear_acce, Vector3 *gyro,
                                  Vector3 *gravity, float ppg,
                                  gesture_state_t *state, gesture_type_t type,
                                  gesture_dataset_t *dataset);

// State variables

static bool ppg_gradient_anomaly_detected = false;
static uint32_t ppg_anomaly_detection_time = 0;
// Timer variables
static rt_timer_t timer_zero_velocity = NULL;
static rt_timer_t timer_hold_confirm = NULL;
static rt_timer_t timer_auto_release = NULL;

static bool zero_velocity = true;
static uint32_t press_detected_timestamp = 0;
static bool press_detected_flag = false;
static bool release_detecting_flag = false;

static bool is_finger_holding = false;
static bool open_wrist_rotation = false;
static bool open_gesture_release_model = false;

// Thread handles
static rt_thread_t gesture_imu_thread = RT_NULL;
static rt_thread_t gesture_ppg_thread = RT_NULL;

// IMU data storage
static Vector3 accData, gyroData;
static Vector3 watch_gravity;
static Vector3 fakeAccData = {0, 0, 9.8};
static Quaternion global_q;
static Quaternion sensor_q;
static sensor_fusion_param_t sensor_fusion_param;
static sensor_fusion_param_t subjective_sensor_fusion_param;

// Timing variables
static uint32_t tap_detect_time = 0;
static uint32_t last_time_start_to_move = 0;
static rt_tick_t linear_acce_not_move_time = 0;

// For motion detection
#define MOTION_THRESHOLD 30 // 0.05g @ 512 LSB/g

static volatile bool need_to_handle_gsensor_int = false;

/**
 * @brief Converts sliding window acceleration data to target waveform
 * @param slidingWinAcc Sliding window acceleration data
 * @param targetWave Target waveform buffer
 * @param sample_len Number of samples to convert
 */
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
        targetWave[i].ppg_data = dataset->ppg_data[i];
        targetWave[i].on_pressed = dataset->on_pressed[i];
    }
}

static double total_acceleration(double x, double y, double z)
{
    return sqrt(x * x + y * y + z * z);
}
#ifdef BSP_USING_MAHONY_AHRS
void calibrate_global_attitude(void)
{
    resetAHRS(&sensor_fusion_param);
}

static void calibrate_sensor_attitude(void)
{
    resetAHRS(&subjective_sensor_fusion_param);
}

static Quaternion sensor_fusion_algorithm(sensor_fusion_param_t *param,
                                          Vector3 *acce, Vector3 *gyro)
{
    float gxrs = DEGREE_TO_RADIAN(gyro->x);
    float gyrs = DEGREE_TO_RADIAN(gyro->y);
    float gzrs = DEGREE_TO_RADIAN(gyro->z);
    Quaternion q =
        updateIMU(param, gxrs, gyrs, gzrs, acce->x, acce->y, acce->z);
    return q;
}

#endif

static void timer_zero_velocity_callback(void *param)
{
    LOG_I("Zero velocity");
    zero_velocity = true;
}

static void start_zero_velocity_detect_timer(bool *zero_velocity_ptr)
{
    if (!timer_zero_velocity)
    {
        timer_zero_velocity = rt_timer_create(
            "zero_velocity_detect", timer_zero_velocity_callback, RT_NULL,
            rt_tick_from_millisecond(200), RT_TIMER_FLAG_ONE_SHOT);
    }

    if (*zero_velocity_ptr)
    {
        *zero_velocity_ptr = false;
        last_time_start_to_move = rt_tick_get_millisecond();
    }

    rt_timer_start(timer_zero_velocity);
}

static void timer_hold_confirm_callback(void *param)
{
    is_finger_holding = true;
    // if (watch_sys_sync.notify_gesture_event)
    //     watch_sys_sync.notify_gesture_event(GESTURE_EVENT_HOLD);
}

static void start_hold_confirm_timer(void)
{
    if (!timer_hold_confirm)
    {
        timer_hold_confirm = rt_timer_create(
            "hold_confirm", timer_hold_confirm_callback, RT_NULL,
            rt_tick_from_millisecond(TAP_DETECT_TIME), RT_TIMER_FLAG_ONE_SHOT);
    }
    rt_timer_start(timer_hold_confirm);
}

static void stop_hold_confirm_timer(void)
{
    if (timer_hold_confirm)
    {
        rt_timer_stop(timer_hold_confirm);
    }
}

static void stop_release_detection(void)
{
    release_detecting_flag = false;
    stop_hold_confirm_timer();
}

static void handle_release_event(bool force_release)
{
    stop_release_detection();
    if (force_release)
    {
        LOG_D("[%s]force release", __func__);
        is_finger_holding = false;
        if (watch_sys_sync.notify_gesture_event)
            watch_sys_sync.notify_gesture_event(GESTURE_EVENT_FORCE_RELEASE);
    }
    else
    {
        if (is_finger_holding)
        {
            LOG_D("[%s]release", __func__);
            is_finger_holding = false;
            if (watch_sys_sync.notify_gesture_event)
                watch_sys_sync.notify_gesture_event(
                    GESTURE_EVENT_FINGER_RELEASE);
        }
        else
        {
            LOG_D("[%s]tap", __func__);
            if (watch_sys_sync.notify_gesture_event)
                watch_sys_sync.notify_gesture_event(GESTURE_EVENT_TAP);
        }
    }
}

static void timer_auto_release_callback(void *param)
{
    handle_release_event(false);
}

static void start_auto_release_timer(void)
{
    if (!timer_auto_release)
    {
        timer_auto_release = rt_timer_create(
            "timer_auto_release", timer_auto_release_callback, RT_NULL,
            rt_tick_from_millisecond(100), RT_TIMER_FLAG_ONE_SHOT);
    }
    rt_timer_start(timer_auto_release);
}

static bool should_trigger_press_detection(void)
{
    if (press_detected_timestamp == 0)
        return false;

    uint32_t current_time = rt_tick_get_millisecond();
    if ((current_time - press_detected_timestamp) >= 200)
    {
        press_detected_timestamp = 0;
        return true;
    }
    return false;
}

/**
 * @brief Called when tap is detected
 * @param param Callback parameter
 */
static uint8_t gesture_mode = 0;
void tap_detected_callback(uint8_t tap_mode)
{
    extern bool get_enable_tap_and_hold(void);
    if (get_enable_tap_and_hold())
    {
        if (!is_ppg_service_ready())
        {
            start_auto_release_timer();
        }
        else
        {
            press_detected_timestamp = rt_tick_get_millisecond();
            start_hold_confirm_timer();
        }
    }

    gesture_mode = tap_mode;
    is_finger_holding = false;
    tap_detect_time = rt_tick_get_millisecond();
}

static inline void reset_gesture_state(gesture_dataset_t *dataset,
                                       uint32_t current_time, uint8_t code)
{
    if (dataset->gesture_sample_count == 0)
    {
        return;
    }
    // WATCH_LCPU_LOG_DEBUG("Reset gesture state at %d", code);
    dataset->gesture_started = false;
    dataset->gesture_ended = false;
    dataset->gesture_sample_count = 0;
    dataset->wait_start_time = current_time;
}

static uint16_t rtc_millisecond = 0;
static inline void store_gesture_sample(gesture_dataset_t *dataset, time_t ts,
                                        Vector3 *linear_accel, Vector3 *gravity,
                                        uint16_t ppg_data, bool on_pressed)
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
        dataset->timestamp_ms[dataset->gesture_sample_count] = rtc_millisecond;
        dataset->ppg_data[dataset->gesture_sample_count] = ppg_data;
        dataset->on_pressed[dataset->gesture_sample_count] = on_pressed;
        dataset->gesture_sample_count++;
    }
}

static rt_timer_t timer_time_correction = NULL;
static time_t time_correction_timer_start_sec = 0;
static void timer_time_correction_callback(void *param)
{
    if (time_correction_timer_start_sec != time(NULL))
    {
        time_correction_timer_start_sec = time(NULL);
        rtc_millisecond = 0;
    }
    else
    {
        rtc_millisecond += 1;
        if (rtc_millisecond >= 1000)
        {
            rtc_millisecond = 0;
        }
    }
}

static void start_time_correction_timer(void)
{
    if (!timer_time_correction)
    {
        time_correction_timer_start_sec = time(NULL);
        timer_time_correction = rt_timer_create(
            "timer_time_correction", timer_time_correction_callback, RT_NULL,
            rt_tick_from_millisecond(1), RT_TIMER_FLAG_PERIODIC);
        if (timer_time_correction == RT_NULL)
        {
            LOG_E("create timer_time_correction failed");
        }
    }

    rt_timer_start(timer_time_correction);
}

static inline void fill_realtime_accel_sliding_window(Vector3 *accel,
                                                      Vector3 *gravity,
                                                      gesture_state_t *state)
{
    for (int i = 0; i < ACCEL_WINDOW_SIZE - 1; i++)
    {
        state->sliding_window_accel[i] = state->sliding_window_accel[i + 1];
        state->sliding_window_gravity[i] = state->sliding_window_gravity[i + 1];
    }
    state->sliding_window_accel[ACCEL_WINDOW_SIZE - 1] = *accel;
    state->sliding_window_gravity[ACCEL_WINDOW_SIZE - 1] = *gravity;
}

static inline bool check_gyro_threshold(Vector3 *gyro, gesture_state_t *state)
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

#ifdef REAL_TIME_IMU_DATA_COLLECTION
static uint8_t realtime_samples = 0;
#endif

#define FEEDBACK_ACCEL_SAMPLES_FOR_TAP 9
#define FEEDBACK_ACCEL_SAMPLES_FOR_RELEASE 10
#define RELEASE_START_THRESHOLD 1.0f
#define TAP_START_THRESHOLD 0.3f
static float difference_accel_sliding_window[MAX_GESTURE_SAMPLES] = {0.0f};
static int difference_accel_count = 0;

extern uint8_t gesture_threshold_factor;
static const uint8_t abnormal_acceleration_difference_threshold = 40;
static bool abnormal_acceleration_vibration = false;
static rt_tick_t abnormal_acceleration_vibration_ts = 0;
// 计算difference_accel_sliding_window的中位数
static float calculate_median_difference_accel(uint8_t check_samples)
{
    float temp_array[MAX_GESTURE_SAMPLES];
    int loop_limit = check_samples;

    // 收集要检查的数据
    for (int i = 0; i < loop_limit; i++)
    {
        int index;

        if (i < difference_accel_count)
        {
            // 检查最新的数据 (从 difference_accel_count-1 往回)
            index = difference_accel_count - 1 - i;
        }
        else
        {
            // 检查环形缓冲区中较旧的数据
            int wrap_around_offset = i - difference_accel_count;
            index = MAX_GESTURE_SAMPLES - 1 - wrap_around_offset;
        }

        temp_array[i] = difference_accel_sliding_window[index];
    }

    // 简单的冒泡排序来排序数据
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

    // 计算中位数
    float median;
    if (loop_limit % 2 == 0)
    {
        // 偶数个数据，取中间两个数的平均值
        median = (temp_array[loop_limit / 2 - 1] + temp_array[loop_limit / 2]) /
                 2.0f;
    }
    else
    {
        // 奇数个数据，取中间的数
        median = temp_array[loop_limit / 2];
    }

    // WATCH_LCPU_LOG_DEBUG("Median of difference_accel: %f", median);

    return median;
}

static void gesture_event_capture(uint16_t freq, time_t ts,
                                  Vector3 *linear_acce, Vector3 *gyro,
                                  Vector3 *gravity, float ppg,
                                  gesture_state_t *state, gesture_type_t type,
                                  gesture_dataset_t *dataset)
{
    rt_tick_t current_time = rt_tick_get_millisecond();

    int cooldown_period = 0;
    if (type == GESTURE_RELEASE || is_imu_data_collection())
    {
        cooldown_period = GESTURE_COLLECTION_COOLDOWN_PERIOD_MS;
    }
    else
    {
        cooldown_period = GESTURE_TAP_COOLDOWN_PERIOD_MS;
    }
    if ((current_time - dataset->wait_start_time) < cooldown_period)
    {
        return;
    }

    if (motor_provider.get_motor_status())
    {
        reset_gesture_state(dataset, current_time, 1);
        return;
    }
    else if (state->if_watchface_visible == false)
    {
        reset_gesture_state(dataset, current_time, 2);
        return;
    }
    else if (state->gyro_lock_status)
    {
        reset_gesture_state(dataset, current_time, 3);
        return;
    }

    int target_samples = 0;
    float start_threshold = 0.0f;
    if (type == GESTURE_TAP)
    {
        target_samples = GESTURE_TAP_TIME_STEP;
        start_threshold = TAP_START_THRESHOLD;
    }
    else if (type == GESTURE_RELEASE)
    {
        target_samples = GESTURE_RELEASE_TIME_STEP;
        start_threshold = RELEASE_START_THRESHOLD;
    }

    if (!dataset->gesture_started && !dataset->gesture_ended)
    {
        if (my_gesture_state.difference_accel > start_threshold)
        {
            // WATCH_LCPU_LOG_DEBUG("gesture_started, difference_accel:%f,
            // start_threshold:%f", difference_accel, start_threshold);
            dataset->gesture_started = true;
            int feedback_samples = 0;
            if (type == GESTURE_TAP)
            {
                feedback_samples = FEEDBACK_ACCEL_SAMPLES_FOR_TAP;
            }
            else if (type == GESTURE_RELEASE)
            {
                feedback_samples = FEEDBACK_ACCEL_SAMPLES_FOR_RELEASE;
            }
            for (int i = 0; i < feedback_samples; i++)
            {
                int accel_index = ACCEL_WINDOW_SIZE - 1 - feedback_samples + i;
                int ppg_index =
                    PPG_FILTER_SAMPLE_NUM - 1 - feedback_samples + i;
                // Ensure PPG index is within bounds
                if (ppg_index < 0)
                    ppg_index = 0;
                if (ppg_index >= PPG_FILTER_SAMPLE_NUM)
                    ppg_index = PPG_FILTER_SAMPLE_NUM - 1;
                store_gesture_sample(
                    dataset, ts, &state->sliding_window_accel[accel_index],
                    &state->sliding_window_gravity[accel_index],
                    ppg_buffer[ppg_index], state->on_pressed);
            }
        }
    }

    if (dataset->gesture_started)
    {
        store_gesture_sample(dataset, ts, linear_acce, gravity, ppg,
                             state->on_pressed);
        if (dataset->gesture_sample_count >= target_samples)
        {
            dataset->gesture_ended = true;
            float median_difference_accel =
                calculate_median_difference_accel(75);
            bool is_gesture = true;
            if (median_difference_accel > 0.25f)
            {
                is_gesture = false;
            }
            if (is_gesture && (user_hand_horizontal || type == GESTURE_TAP))
            {
                // LOG_D("GET WAVE[%d] %f", target_samples,
                // median_difference_accel);
                getTargetWaveformFromSlidingWindow(dataset, targetWave_algo,
                                                   target_samples);
                if (watch_sys_sync.notify_gesture_dataset)
                    watch_sys_sync.notify_gesture_dataset(
                        rt_tick_get(), target_samples,
                        (void *)targetWave_algo); //, targetWave_algo
            }
            if (is_imu_data_collection())
            {
                reset_gesture_state(dataset, current_time, 6);
            }
            else
            {
                reset_gesture_state(dataset, current_time - cooldown_period, 7);
            }
        }
    }
}

#if USE_PPG_VARIANCE
static uint16_t gesture_release_ppg_rawdata_diff_buf[GESTURE_RELEASE_TIME_STEP];
static uint16_t gesture_release_ppg_rawdata_diff_index = 0;
static bool calculate_ppg_variance(void)
{
    float mean = 0.0f;
    float variance = 0.0f; // Calculate average value
    for (int i = 0; i < GESTURE_RELEASE_SLIDING_WINDOW_SIZE; i++)
    {
        mean += gesture_release_ppg_rawdata_diff_buf[i];
    }
    mean /= GESTURE_RELEASE_SLIDING_WINDOW_SIZE; // Calculate variance
    for (int i = 0; i < GESTURE_RELEASE_SLIDING_WINDOW_SIZE; i++)
    {
        float diff = gesture_release_ppg_rawdata_diff_buf[i] - mean;
        variance += diff * diff;
    }
    variance /= GESTURE_RELEASE_SLIDING_WINDOW_SIZE;
    LOG_D("PPG Variance: %f", variance);
    if (variance > 100)
    {
        return true;
    }
    else
    {
        return false;
    }
}
#endif

/**
 * @brief Calculate gravity vector from quaternion
 * @param q Quaternion
 * @return Gravity vector
 */
static Vector3 calculate_gravity(Quaternion *q)
{
    Vector3 v;
    v.x = 2 * (q->x * q->z - q->w * q->y);
    v.y = 2 * (q->w * q->x + q->y * q->z);
    v.z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    return v;
}

/**
 * @brief Calculate linear acceleration by removing gravity component
 * @param acceleration Raw acceleration
 * @param gravity Gravity vector
 * @return Linear acceleration vector
 */
Vector3 calculate_linear_acceleration(Vector3 *acceleration, Vector3 *gravity)
{
    Vector3 linear_acceleration;
    linear_acceleration.x = acceleration->x - gravity->x * GRAVITY;
    linear_acceleration.y = acceleration->y - gravity->y * GRAVITY;
    linear_acceleration.z = acceleration->z - gravity->z * GRAVITY;
    return linear_acceleration;
}

/**
 * @brief Check if device is moving based on gyroscope data (threshold °/s)
 * @param gyro_y Y-axis rotation rate
 * @return true if moving, false otherwise
 */
static bool judge_if_moving_by_gyro(float gyro_y)
{
    return fabs(gyro_y) >= 20;
}

/**
 * @brief Check if device is moving based on gyroscope data (threshold 5°/s)
 * @param gyro_y Y-axis rotation rate
 * @param gyro_z Z-axis rotation rate
 * @return true if moving, false otherwise
 */
static bool judge_if_moving5_by_gyro(float gyro_y, float gyro_z)
{
    return fabs(gyro_z) >= 5;
}

// Constants for algorithm ratio conversion
#define HEALTH_ALGO_RATIO 512.0 / GRAVITY    // 512 LSB/g
#define ACTIVITY_ALGO_RATIO 1000.0 / GRAVITY // 1000 LSB/g

#ifdef BSP_USING_ACTIVITY_ALGO_KRAEPELIN
/**
 * @brief Create ring buffer for accelerometer raw data
 */
static struct rt_ringbuffer *accel_rawdata_rb;
static void create_accel_rawdata_rb(void)
{
    accel_rawdata_rb = rt_ringbuffer_create(sizeof(AccelRawData) * 125);
    if (accel_rawdata_rb == RT_NULL)
    {
        LOG_E("create accel_rawdata_rb failed");
    }
}

extern AccelRawData *activity_get_accel_rawdata(void);
extern void notify_activity_algorithm(void);

/**
 * @brief Handle accelerometer data in ring buffer
 * @param data New accelerometer data
 */
static void handle_accel_rawdata_rb(AccelRawData *data)
{
    if (rt_ringbuffer_space_len(accel_rawdata_rb) < sizeof(AccelRawData))
    {
        AccelRawData *global_rawdata = activity_get_accel_rawdata();
        rt_ringbuffer_get(accel_rawdata_rb, global_rawdata,
                          sizeof(AccelRawData) * 125);
        notify_activity_algorithm();
    }

    int ret = rt_ringbuffer_put(accel_rawdata_rb, data, sizeof(AccelRawData));
    if (ret != sizeof(AccelRawData))
    {
        LOG_E("Failed to put data into accel_rawdata_rb, ret: %d", ret);
    }
}
#endif

/**
 * @brief Add data to G-sensor FIFO buffer
 * @param rawdata Raw accelerometer data
 */
void add_to_gsensor_fifo(signed short *rawdata)
{
    for (int i = 0; i < 3; i++)
    {
        gsensor_fifo_buffer[gsensor_fifo_buffer_index][i] = rawdata[i];
    }
    gsensor_fifo_buffer_index++;
    if (gsensor_fifo_buffer_index >= GSENSOR_FIFO_BUFFER_SIZE)
    {
        gsensor_fifo_buffer_index = 0;
    }
}

/**
 * @brief Process accelerometer data for motion detection
 * @param accRawData Raw accelerometer data
 */
static void motion_detection_process(signed short *accRawData)
{
    static float last_total_acceleration = 0.0;
    float x = accRawData[0];
    float y = accRawData[1];
    float z = accRawData[2];
    float total_acceleration = sqrt(x * x + y * y + z * z);
    float diff = fabs(total_acceleration - last_total_acceleration);

    if (diff >= MOTION_THRESHOLD)
    {
        // LOG_D("Motion detected total acceleration:%.3f, diff:%.3f",
        // total_acceleration, diff);
        need_to_handle_gsensor_int = true;
    }

    last_total_acceleration = total_acceleration;
}

/**
 * @brief Process motion data at 25Hz rate for health algorithms
 * @param now Current time
 * @param accData Accelerometer data
 */
void handle_motion_data_in_25hz(rt_tick_t now, Vector3 *accData)
{
    signed short accRawData[3] = {0};
    accRawData[0] = accData->x * HEALTH_ALGO_RATIO;
    accRawData[1] = accData->y * HEALTH_ALGO_RATIO;
    accRawData[2] = accData->z * HEALTH_ALGO_RATIO;

    add_to_gsensor_fifo(accRawData);
    motion_detection_process(accRawData);

#ifdef BSP_USING_HEALTH_ALGO
    rtk_gsa_fsm(accRawData);
#endif

#ifdef BSP_USING_ACTIVITY_ALGO_KRAEPELIN
    AccelRawData rawdata;
    rawdata.x = accData->x * ACTIVITY_ALGO_RATIO;
    rawdata.y = accData->y * ACTIVITY_ALGO_RATIO;
    rawdata.z = accData->z * ACTIVITY_ALGO_RATIO;
    handle_accel_rawdata_rb(&rawdata);
#endif
}

/**
 * @brief Process IMU data for gesture detection
 * @param hz Sample rate in Hz
 * @param accData Accelerometer data
 * @param gyroData Gyroscope data
 * @return cost time in ms
 */

int handle_imu_data(float hz, Vector3 *accData, Vector3 *gyroData)
{
    time_t ts = time(NULL);
    static float pre_freq = 0;
    rt_tick_t now = rt_tick_get_millisecond();

    // Calculate orientation using sensor fusion
    global_q = sensor_fusion_algorithm(&sensor_fusion_param, accData, gyroData);
    watch_gravity = calculate_gravity(&global_q);
#ifdef BSP_USING_HAND_TRACKING
    float horizontal_threshold =
        gesture_threshold_factor * 0.01f; // Default 0.3
    // Hand position detection
    user_hand_horizontal =
        (watch_gravity.x < 0.9 && watch_gravity.x > -horizontal_threshold);
    if (watch_gravity.y > -0.7 && watch_gravity.z > -0.6)
    {
        if (!my_gesture_state.if_watchface_visible)
        {
            my_gesture_state.if_watchface_visible = true;
        }
    }
    else
    {
        if (my_gesture_state.if_watchface_visible)
        {
            my_gesture_state.if_watchface_visible = false;
        }
    }

    if (user_hand_horizontal && fabs(gyroData->x) > fabs(gyroData->y) &&
        fabs(gyroData->x) > fabs(gyroData->z))
    {
        if (!open_wrist_rotation)
        {
            open_wrist_rotation = true;
        }
    }
    else
    {
        if (open_wrist_rotation)
        {
            open_wrist_rotation = false;
        }
    }
    // Zero velocity detection
    if (judge_if_moving_by_gyro(gyroData->y))
    {
        start_zero_velocity_detect_timer(&zero_velocity);
    }
#endif

    // Sample rate adaptation for algorithms
    static uint8_t health_algo_counter = 0;
    if (hz != pre_freq)
    {
        health_algo_counter = 0;
        pre_freq = hz;
    }

    uint8_t target_count = hz / 25;
    if (health_algo_counter < target_count)
    {
        health_algo_counter++;
    }

    if (health_algo_counter >= target_count)
    {
        handle_motion_data_in_25hz(now, accData);
#ifdef BSP_USING_HAND_TRACKING
        hand_tracking_data_update(
            25, gyroData->x, gyroData->y, open_wrist_rotation,
            my_gesture_state.if_watchface_visible, zero_velocity);
#endif
        health_algo_counter = 0;
    }
#if (CUSTOMER_BOARD_VER != BOARD_VER_13)
    if (battery_charge_state.is_charging)
    {
        return (rt_tick_get_millisecond() - now);
    }
#endif

    // Pin debounce logic (10ms)
    // static uint32_t last_pin_read_time = 0;
    // static uint8_t last_pin_state = 0;
    // uint8_t current_pin_state = rt_pin_read(128);
    // if (current_pin_state != last_pin_state)
    // {
    //     last_pin_read_time = now;
    //     last_pin_state = current_pin_state;
    // }
    // else if ((now - last_pin_read_time) >= PIN_DEBOUNCE_TIME_MS)
    // {
    //     my_gesture_state.on_pressed = current_pin_state;
    // }

    // Process G-sensor interrupt if needed
    if (need_to_handle_gsensor_int)
    {
        hal_gsensor_drv_int1_handler();
        need_to_handle_gsensor_int = false;
    }

    if (hz == IMU_NOARMAL_SAMPLE_RATE && !is_sleep_mode())
    {
        sensor_q = sensor_fusion_algorithm(&subjective_sensor_fusion_param,
                                           &fakeAccData, gyroData);
        Vector3 linear_acce =
            calculate_linear_acceleration(accData, &watch_gravity);
        motion_data_t motion_data = {
            .timestamp = now,
            .linear_acce = linear_acce,
            .gravity = watch_gravity,
            .global_q = global_q,
            .sensor_q = sensor_q,
        };
        motion_data_fetch(&motion_data);

        static bool ppg_get = false;
        float ppg_rawdata = 0.0f;
        if (ppg_get)
        {
            ppg_rawdata = ppg_buffer[PPG_FILTER_SAMPLE_NUM - 1];
        }
        else
        {
            ppg_rawdata = ppg_buffer[PPG_FILTER_SAMPLE_NUM - 2];
        }
        ppg_get = !ppg_get;

        static float prev_linear_accel = 0.0f;
        float linear_accel_resultant =
            total_acceleration(linear_acce.x, linear_acce.y, linear_acce.z);
        my_gesture_state.difference_accel =
            fabsf(linear_accel_resultant - prev_linear_accel);
        difference_accel_sliding_window[difference_accel_count] =
            my_gesture_state.difference_accel;
        if (difference_accel_count < MAX_GESTURE_SAMPLES - 1)
        {
            difference_accel_count++;
        }
        else
        {
            difference_accel_count = 0;
        }
        prev_linear_accel = linear_accel_resultant;
#ifdef REAL_TIME_IMU_DATA_COLLECTION
        if (is_imu_rawdata_collection())
        {
            store_gesture_sample(&release_dataset, ts, &linear_acce, gyroData,
                                 ppg_rawdata, my_gesture_state.on_pressed);
            if (release_dataset.gesture_sample_count >= MAX_RAWDATA_TIME_STEP)
            {
                getTargetWaveformFromSlidingWindow(
                    &release_dataset, targetWave_algo, MAX_RAWDATA_TIME_STEP);
                if (watch_sys_sync.notify_gesture_dataset)
                    watch_sys_sync.notify_gesture_dataset(
                        rt_tick_get(), MAX_RAWDATA_TIME_STEP,
                        (void *)targetWave_algo); //
                release_dataset.gesture_sample_count = 0;
            }
            return (rt_tick_get_millisecond() - now);
        }
#endif

        fill_realtime_accel_sliding_window(&linear_acce, &watch_gravity,
                                           &my_gesture_state);

        check_gyro_threshold(gyroData, &my_gesture_state);

        if (is_multi_gesture_mode())
        {
            gesture_event_capture(IMU_NOARMAL_SAMPLE_RATE, ts, &linear_acce,
                                  gyroData, &watch_gravity, ppg_rawdata,
                                  &my_gesture_state, GESTURE_TAP, &tap_dataset);
            gesture_event_capture(IMU_NOARMAL_SAMPLE_RATE, ts, &linear_acce,
                                  gyroData, &watch_gravity, ppg_rawdata,
                                  &my_gesture_state, GESTURE_RELEASE,
                                  &release_dataset);
            return (rt_tick_get_millisecond() - now);
        }
        else
        {
            if (!get_locked_status())
            {
                gesture_event_capture(IMU_NOARMAL_SAMPLE_RATE, ts, &linear_acce,
                                      gyroData, &watch_gravity, ppg_rawdata,
                                      &my_gesture_state, GESTURE_TAP,
                                      &tap_dataset);
            }
            else
            {
                gesture_event_capture(IMU_NOARMAL_SAMPLE_RATE, ts, &linear_acce,
                                      gyroData, &watch_gravity, ppg_rawdata,
                                      &my_gesture_state, GESTURE_RELEASE,
                                      &release_dataset);
            }
        }
    }
    return (rt_tick_get_millisecond() - now);
}

/**
 * @brief Callback function for lift detection
 * @param lift Lift status
 */
static void lift_cb(uint8_t lift)
{
    if (watch_sys_sync.lift_status_callback == NULL)
    {
        return;
    }
    if (is_sleep_mode())
    {
        if (lift == 0)
        {
            return;
        }
        else if (lift == 3)
        {
            if (user_hand_horizontal)
            {
                watch_sys_sync.lift_status_callback(2);
            }
            return;
        }
    }
    watch_sys_sync.lift_status_callback(lift);
}

static void back_cb(void)
{
    if (watch_sys_sync.lift_status_callback == NULL)
    {
        return;
    }
    if (is_sleep_mode())
    {
        if (user_hand_horizontal)
        {
            watch_sys_sync.lift_status_callback(2);
        }
    }
    else
    {
        watch_sys_sync.notify_gesture_event(GESTURE_EVENT_BACK);
    }
}

#if ENABLE_IMU_SEM_FIFO
    // Thread definitions
    #define IMU_THREAD_STACK_SIZE 512 * 4
    #define IMU_THREAD_PRIORITY 15
    #define IMU_THREAD_TIMESLICE 10

/**
 * @brief IMU thread entry function
 * @param parameter Thread parameter
 */
static void gesture_imu_thread_entry(void *parameter)
{
    while (1)
    {
        rt_sem_take(watch_sensor.imu_sem, RT_WAITING_FOREVER);
        accData = watch_sensor.imu_data.acce;
        gyroData = watch_sensor.imu_data.gyro;
        handle_imu_data(watch_sensor.imu_data.sample_rate, &accData, &gyroData);
    }
}
#endif

/**
 * @brief Initialize IMU thread
 * @return RT_EOK on success, error code otherwise
 */
static int gesture_imu_thread_init(void)
{
    calibrate_global_attitude();
    calibrate_sensor_attitude();
#ifdef BSP_USING_ACTIVITY_ALGO_KRAEPELIN
    create_accel_rawdata_rb();
#endif

// Initialize feature modules
#ifdef BSP_USING_HAND_TRACKING
    hand_tracking_init(lift_cb, back_cb);
#endif
    // start_time_correction_timer();
#if ENABLE_IMU_SEM_FIFO
    gesture_imu_thread = rt_thread_create(
        "imu", gesture_imu_thread_entry, RT_NULL, IMU_THREAD_STACK_SIZE,
        IMU_THREAD_PRIORITY, IMU_THREAD_TIMESLICE);
    if (gesture_imu_thread != RT_NULL)
    {
        rt_thread_startup(gesture_imu_thread);
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
#endif
}
INIT_APP_EXPORT(gesture_imu_thread_init);

/////////////////////////////////////////////
// Constants for PPG processing
#define PPG_BUFFER_LENGTH 7
#define PPG_END_THRESHOLD_RATIIO 0.4
#define PPG_END_THRESTAP_RATIIO 0.3
#define STANDARD_FIRST_THRESHOLD -400

/**
 * @brief Process gradient for gesture detection
 * @param gradient Current gradient
 * @param end_threshold Threshold for determining release
 */
static void handle_gradient(float gradient, float end_threshold)
{
    static float positive_gradient_sum = 0;
    static float negative_gradient_sum = 0;
    if (gradient > 0)
    {
        negative_gradient_sum = 0;
    }
    else if (gradient < 0)
    {
        positive_gradient_sum = 0;
    }

    if (end_threshold < 0)
    {
        positive_gradient_sum = 0;
        negative_gradient_sum += gradient;
        if ((gradient < end_threshold))
        {
            negative_gradient_sum = 0;
            handle_release_event(false);
        }
    }
    else
    {
        negative_gradient_sum = 0;
        positive_gradient_sum += gradient;
        if ((gradient > end_threshold))
        {
            positive_gradient_sum = 0;
            handle_release_event(false);
        }
    }
}

extern bool is_hcpu_wakeup_in_last_3s(void);
/**
 * @brief Process raw PPG data for gesture detection
 * @param rawdata Raw PPG value
 */

void process_ppg_rawdata(uint32_t rawdata)
{
    // LOG_D("PPG Raw Data: %d", rawdata);
    static float prev_ppg_value[6] = {0};
    static float ppg_gradient_array[PPG_BUFFER_LENGTH + 1];

    uint32_t current_time = rt_tick_get_millisecond();

    // Update filter buffer
    ppg_buffer[PPG_FILTER_SAMPLE_NUM] = (float)rawdata;
    float ppg_average_value = 0;

    // Calculate moving average
    for (int i = 0; i < PPG_FILTER_SAMPLE_NUM; i++)
    {
        ppg_buffer[i] = ppg_buffer[i + 1];
        ppg_average_value += ppg_buffer[i];
    }
    ppg_average_value /= PPG_FILTER_SAMPLE_NUM;

#if USE_PPG_VARIANCE
    static uint32_t pevr_rawdata = 0;
    gesture_release_ppg_rawdata_diff_buf
        [gesture_release_ppg_rawdata_diff_index] = abs(rawdata - pevr_rawdata);
    if (gesture_release_ppg_rawdata_diff_index < 24)
    {
        gesture_release_ppg_rawdata_diff_index++;
    }
    else
    {
        gesture_release_ppg_rawdata_diff_index = 0;
    }
    pevr_rawdata = rawdata;
#endif

    // Update previous values buffer
    for (int i = 0; i < 5; i++)
    {
        prev_ppg_value[i] = prev_ppg_value[i + 1];
    }
    prev_ppg_value[5] = ppg_average_value;

    // Calculate gradient
    float gradient = ppg_average_value - prev_ppg_value[0];

    // Update gradient arrays
    for (int i = 0; i < PPG_BUFFER_LENGTH; i++)
    {
        ppg_gradient_array[i] = ppg_gradient_array[i + 1];
    }
    ppg_gradient_array[PPG_BUFFER_LENGTH] = gradient;

    float ppg_gradient_average = 0;
    for (int i = 0; i < PPG_BUFFER_LENGTH + 1; i++)
    {
        ppg_gradient_average += ppg_gradient_array[i];
    }
    ppg_gradient_average /= (PPG_BUFFER_LENGTH + 1);

    if (ppg_gradient_anomaly_detected)
    {
        if ((current_time - ppg_anomaly_detection_time) >=
            PPG_ANOMALY_DISABLE_DURATION_MS)
        {
            ppg_gradient_anomaly_detected = false;
            LOG_D("PPG gradient anomaly cleared");
        }
    }
    else
    {
        if (fabsf(ppg_gradient_average) > 2000 && !is_hcpu_wakeup_in_last_3s())
        {
            ppg_gradient_anomaly_detected = true;
            LOG_D("PPG gradient anomaly detected: %f", ppg_gradient_average);
            ppg_anomaly_detection_time = current_time;
        }
    }

    static float standard_count = 0;
    static float standard_ppg_gradient = 0;

    extern bool get_enable_tap_and_hold(void);
    if (get_enable_tap_and_hold())
    {
        // Check for hold release condition
        bool open_hold_delta =
            fabs(gradient) > fabs(ppg_gradient_average) * 1.5 &&
            fabs(ppg_gradient_average) > 5;
        if (open_hold_delta && is_finger_holding) // && zero_velocity
        {
            if (gesture_mode == 0 || (current_time - tap_detect_time) > 500)
            {
                handle_release_event(false);
            }
        }

        if (should_trigger_press_detection())
        {
            press_detected_flag = true;
        }

        if (press_detected_flag)
        {
            // Determine characteristics of first half
            int first_half_characteristics = 0;
            for (int i = 0; i < PPG_BUFFER_LENGTH; i++)
            {
                if (ppg_gradient_array[i] > 0)
                {
                    first_half_characteristics += 1;
                }
                else
                {
                    first_half_characteristics -= 1;
                }
            }

            // Find maximum gradient
            standard_count = 0;
            float max_gradient = 0;
            for (int i = 0; i < PPG_BUFFER_LENGTH; i++)
            {
                if (first_half_characteristics > 0)
                {
                    if (max_gradient < ppg_gradient_array[i])
                    {
                        max_gradient = ppg_gradient_array[i];
                    }
                }
                else
                {
                    if (max_gradient > ppg_gradient_array[i])
                    {
                        max_gradient = ppg_gradient_array[i];
                    }
                }
                if (fabs(max_gradient) < fabs(ppg_gradient_array[i]))
                {
                    max_gradient = ppg_gradient_array[i];
                }
            }

            standard_ppg_gradient = max_gradient;
            press_detected_flag = false;
            release_detecting_flag = true;

            return;
        }

        // Handle release detection
        if (release_detecting_flag)
        {
            float end_gradient_threshold = 0.0f;
            if (standard_count < 6)
            {
                standard_count++;
                end_gradient_threshold =
                    -standard_ppg_gradient * PPG_END_THRESTAP_RATIIO;
            }
            else
            {
                end_gradient_threshold =
                    -standard_ppg_gradient * PPG_END_THRESHOLD_RATIIO;
            }

            if (!is_finger_holding) // && zero_velocity
            {
                handle_gradient(gradient, end_gradient_threshold);
            }
        }
    }
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/