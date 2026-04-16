/**
 ******************************************************************************
 * @file   gesture_detect.c
 * @author Skaiwalk software development team
 * @brief  手勢偵測核心模組。負責 IMU 感測融合(Mahony AHRS)、重力/線性加速度
 *         分離、PPG 訊號梯度分析，以實現觸碰(tap)、長按(hold)、放開(release)
 *         等手勢事件偵測，並將運動資料透過 motion_data_fetch() 傳送至 HCPU。
 ******************************************************************************
 */
#include <rtthread.h>
#include <board.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
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
#ifdef BSP_USING_WEAR_DETECT
    #include "wear_detect.h"
#endif

#define DBG_TAG "gesture.detect"
#define DBG_LVL DBG_INFO
#include "rtdbg.h"

// Configuration constants
#define USE_PPG_VARIANCE 0
#define TAP_DETECT_TIME 500
#define PPG_ANOMALY_DISABLE_DURATION_MS 1000
#define PPG_FILTER_SAMPLE_NUM 10
static float ppg_buffer[PPG_FILTER_SAMPLE_NUM];

#define GESTURE_EVENT_TAP (1 << 1)
#define GESTURE_EVENT_HOLD (1 << 2)
#define GESTURE_EVENT_FINGER_RELEASE (1 << 3)
#define GESTURE_EVENT_FORCE_RELEASE (1 << 4)
#define GESTURE_EVENT_BACK (1 << 6)

static bool user_hand_horizontal = false;
static bool watchface_visible = false;

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

// Thread handles
static rt_thread_t gesture_imu_thread = RT_NULL;

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

// For motion detection
#define MOTION_THRESHOLD 30 // 0.05g @ 512 LSB/g

static volatile bool need_to_handle_gsensor_int = false;

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

void reinitialize_ahrs_from_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z)
{
    float ax = (float)raw_x;
    float ay = (float)raw_y;
    float az = (float)raw_z;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.01f)
        return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    float pitch = asinf(-ax);
    float roll = atan2f(ay, az);

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    sensor_fusion_param.q0 = cr * cp;
    sensor_fusion_param.q1 = sr * cp;
    sensor_fusion_param.q2 = cr * sp;
    sensor_fusion_param.q3 = -sr * sp;
    sensor_fusion_param.integralFBx = 0.0f;
    sensor_fusion_param.integralFBy = 0.0f;
    sensor_fusion_param.integralFBz = 0.0f;

    LOG_I("AHRS reinitialized from accel: pitch=%.2f, roll=%.2f",
          pitch * 57.29577951f, roll * 57.29577951f);
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

extern uint8_t gesture_threshold_factor;
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

static bool judge_if_moving_by_gyro(float gyro_y)
{
    return fabsf(gyro_y) >= 20.0f;
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
    float total_acceleration = sqrtf(x * x + y * y + z * z);
    float diff = fabsf(total_acceleration - last_total_acceleration);

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
    static float pre_freq = 0;
    rt_tick_t now = rt_tick_get_millisecond();

    // Calculate orientation using sensor fusion
    global_q = sensor_fusion_algorithm(&sensor_fusion_param, accData, gyroData);
    watch_gravity = calculate_gravity(&global_q);
#ifdef BSP_USING_WEAR_DETECT
    wear_detect_feed_imu(accData, hz);
#endif
#ifdef BSP_USING_HAND_TRACKING
    float horizontal_threshold =
        gesture_threshold_factor * 0.01f; // Default 0.3
    // Hand position detection
    user_hand_horizontal =
        (watch_gravity.x < 0.9f && watch_gravity.x > -horizontal_threshold);
    watchface_visible = (fabs(watch_gravity.x) < 0.4f &&
                         watch_gravity.y > -0.7f && watch_gravity.z > -0.6f);
    float abs_gx = fabsf(gyroData->x);
    open_wrist_rotation = user_hand_horizontal && abs_gx > fabsf(gyroData->y) &&
                          abs_gx > fabsf(gyroData->z);
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
        hand_tracking_data_update(25, gyroData->x, gyroData->y,
                                  open_wrist_rotation, watchface_visible,
                                  zero_velocity);
#endif
        health_algo_counter = 0;
    }
    // Process G-sensor interrupt if needed
    if (need_to_handle_gsensor_int)
    {
        hal_gsensor_drv_int1_handler();
        need_to_handle_gsensor_int = false;
    }

    if (hz == IMU_NOARMAL_SAMPLE_RATE && !is_sleep_mode())
    {
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

        // Note: Waveform capture algorithm has been moved to HCPU
        // (bloc_motion_tracking.c) The HCPU will receive motion_data via
        // motion_data_fetch() and process gesture waveform capture directly,
        // then notify gesture_sem when ready.
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
#ifdef BSP_USING_WEAR_DETECT
    wear_detect_init();
#endif
#ifdef BSP_USING_HAND_TRACKING
    hand_tracking_init(lift_cb, back_cb);
#endif
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
#define PPG_END_THRESHOLD_RATIIO 0.4f
#define PPG_END_THRESTAP_RATIIO 0.3f

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
static uint8_t log_count = 0;
void process_ppg_rawdata(uint32_t rawdata)
{
#ifdef BSP_USING_WEAR_DETECT
    wear_detect_feed_ppg(rawdata, 0);
#endif
    // if (log_count < 10)
    // {
    //     log_count++;
    // }
    // else
    // {
    //     log_count = 0;
    //     LOG_D("PPG raw data: %d", rawdata);
    // }
    
    static float prev_ppg_value[6] = {0};
    static float ppg_gradient_array[PPG_BUFFER_LENGTH + 1];

    uint32_t current_time = rt_tick_get_millisecond();

    // Shift filter buffer and insert new sample
    memmove(&ppg_buffer[0], &ppg_buffer[1],
            (PPG_FILTER_SAMPLE_NUM - 1) * sizeof(float));
    ppg_buffer[PPG_FILTER_SAMPLE_NUM - 1] = (float)rawdata;

    // Calculate moving average
    float ppg_average_value = 0;
    for (int i = 0; i < PPG_FILTER_SAMPLE_NUM; i++)
    {
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
    memmove(&prev_ppg_value[0], &prev_ppg_value[1], 5 * sizeof(float));
    prev_ppg_value[5] = ppg_average_value;

    // Calculate gradient
    float gradient = ppg_average_value - prev_ppg_value[0];

    // Update gradient array
    memmove(&ppg_gradient_array[0], &ppg_gradient_array[1],
            PPG_BUFFER_LENGTH * sizeof(float));
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
            fabsf(gradient) > fabsf(ppg_gradient_average) * 1.5f &&
            fabsf(ppg_gradient_average) > 5.0f;
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
                if (fabsf(max_gradient) < fabsf(ppg_gradient_array[i]))
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