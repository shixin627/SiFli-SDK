/**
 ******************************************************************************
 * @file   bloc_motion_tracking.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

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
// static void gesture_event_capture_hcpu(uint16_t freq, time_t ts,
//                                        Vector3 *linear_acce, Vector3 *gyro,
//                                        Vector3 *gravity, float ppg,
//                                        waveform_gesture_state_t *state,
//                                        gesture_type_t type,
//                                        gesture_dataset_t *dataset);
#endif // ENABLE_WAVEFORM_CAPTURE

#define ENABLE_SEND_GRAVITY_TO_BLE_CLIENT 0

#define ENABLE_QUICK_ACTION 0

#define MOTION_TRACKING_THREAD_STACK_SIZE 2 * 1024
#define MOTION_TRACKING_THREAD_PRIORITY 17
#define MOTION_TRACKING_THREAD_TIMESLICE 10

#define DISPLACEMENT_YAW_THRESHOLD                                             \
    2.0f                        // 當角度變化超過此值時，才更新一次移動量
#define DEFAULT_ROTATION_MAG 30 // 水平旋轉操作之放大倍數
#define STARTED_MOVE_THRESHOLD                                                 \
    4.0f // 當角度變化超過此值時，表示使用者已經開始控制手臂
#define VOLUME_CONTROL_RATIO 2.0f // 音量控制的放大倍數

extern rt_err_t air_mouse_rx_indicate(void);

static rt_thread_t motion_tracking_thread = RT_NULL;

static euler_angle_t motion_tracking_algorithm(Quaternion *quaternion,
                                               Quaternion *prev_quat);
static void send_quaternion_to_ble_client(rt_uint32_t ts, Quaternion *q);
static Quaternion multiply_quaternions(Quaternion *q1, Quaternion *q2);

static bool stop_mouse_move = false;

#ifdef BSP_USING_AIR_MOUSE
static void air_mouse_process(rt_uint32_t ts, Quaternion *quaternion,
                              Quaternion *prev_quat);
#endif

#define INIT_OFFSET_Y (LV_VER_RES / 2 + 100)

static float q_vertical_movement_magnification = 1.0;
void set_q_vertical_movement_magnification(float mag)
{
    q_vertical_movement_magnification = mag;
}
static void use_q_vertical_movement(float delta_angle)
{
    /// **** Simulate air mouse pointer device **** ///
    air_mouse_rx_indicate();
    int16_t rotation_magnification =
        -DEFAULT_ROTATION_MAG * q_vertical_movement_magnification;
    float displacement_yaw = delta_angle * rotation_magnification;
    float yaw_threshold =
        q_vertical_movement_magnification * DISPLACEMENT_YAW_THRESHOLD;
    if (fabs(delta_angle) > yaw_threshold)
    {
        int16_t new_position = INIT_OFFSET_Y + (int16_t)displacement_yaw;
        setCoordinateY(new_position);
    }
}

#if ENABLE_MEDIA_VOLUMN_BAR_CONTROL

static uint8_t volume_control_difference = 0;
uint8_t get_volume_control_difference(void)
{
    return volume_control_difference;
}

    #define VOLUME_CONTROL_TIMEOUT 200 // 200ms timeout
static uint8_t vertical_movement_start_volume = 0;
static uint8_t pevr_volume_control_value = 0;

static rt_timer_t volume_control_timer = RT_NULL;

static void volume_control_timeout_cb(void *parameter)
{
    // 在這裡處理超時後的動作
    volume_control_difference =
        abs(vertical_movement_start_volume - pevr_volume_control_value) / 2;
    LOG_D("Volume control timeout %d", volume_control_difference);
    if (vertical_movement_start_volume > pevr_volume_control_value)
    {
        sys_media_event_set(SYS_EVENT_VOLUME_DOWN);
    }
    else if (vertical_movement_start_volume < pevr_volume_control_value)
    {
        sys_media_event_set(SYS_EVENT_VOLUME_UP);
    }
}

static void reset_volume_control_timer(void)
{
    if (!volume_control_timer)
    {
        // 首次建立計時器
        volume_control_timer = rt_timer_create(
            "vol_ctrl_timer", volume_control_timeout_cb, RT_NULL,
            VOLUME_CONTROL_TIMEOUT, RT_TIMER_FLAG_ONE_SHOT);
    }

    // 重新啟動計時器
    rt_timer_stop(volume_control_timer);
    rt_timer_start(volume_control_timer);
}

extern uint8_t get_volume_control_value(void);
extern bool get_volume_control_status(void);
static uint8_t volume_control_value = 0;
static void media_use_q_vertical_movement(float delta_angle)
{
    /// **** Simulate air mouse pointer device **** ///
    if (!SkaiWatchSys.connected_to_phone)
    {
        reset_volume_control_timer();
        vertical_movement_start_volume = get_volume_control_value();
    }
    float displacement_yaw = delta_angle * VOLUME_CONTROL_RATIO;
    if ((fabs(delta_angle) > DISPLACEMENT_YAW_THRESHOLD || start_move_flag) &&
        get_volume_control_status())
    {
        int new_volume = get_volume_control_value() + (int16_t)displacement_yaw;
        new_volume =
            (new_volume > 100) ? 100 : (new_volume < 0 ? 0 : new_volume);
        if (abs(new_volume - pevr_volume_control_value) >= 10 ||
            new_volume == 0 || new_volume == 100)
        {
            lvgl_msg_t msg;
            msg.type = LVGL_MSG_TYPE_VOLUME_CONTROL;
            msg.data.volume_control = new_volume;
            LOG_D("msg.data.volume_control:%d", msg.data.volume_control);
            lvgl_send_msg(msg);
            pevr_volume_control_value = new_volume;
        }
    }
}
#endif

#ifdef APP_ID_WIDGETS
extern void set_init_navigation_bar_position(void);

static bool is_hand_up = false;

static bool navigation_bar_handled = false;

static int16_t navigation_bar_control_value = 0;
static int16_t navigation_bar_control_value_prev = 0;

void press_navigation_bar(void)
{
    if (is_hand_up)
    {
        return;
    }

    if (gui_app_is_actived(APP_ID_WIDGETS))
    {
        set_init_navigation_bar_position();
        navigation_bar_handled = true;
        navigation_bar_control_value = 0;
    }
}

void release_navigation_bar(void)
{
    if (gui_app_is_actived(APP_ID_WIDGETS))
    {
        navigation_bar_handled = false;
    }
}

void widget_page_flip(bool is_gravity_x_positive)
{
    is_hand_up = is_gravity_x_positive;
    LOG_D("is_hand_up:%d", is_hand_up);
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_HAND_UP;
    msg.data.action = !is_hand_up;
    lvgl_send_msg(msg);
}

#endif // APP_ID_WIDGETS

#define SCROLL_APP_THRESHOLD 200
static uint8_t move_count = 0;
static int16_t old_position = 0;
static int16_t last_scroll_time = 0;
static void navigation_bar_control_with_quaternion(float delta_angle)
{
    /// **** Simulate air mouse pointer device **** ///
    air_mouse_rx_indicate();
    int16_t rotation_magnification = -DEFAULT_ROTATION_MAG;

    float displacement_yaw = delta_angle * rotation_magnification;
    if (fabs(delta_angle) > DISPLACEMENT_YAW_THRESHOLD)
    {
        int16_t new_position = (LV_VER_RES / 2) + (int16_t)displacement_yaw;
    }
}

// extern void app_list_scroll_to_app(bool up);
// extern void mesage_list_scroll_to_app(bool up);
// extern void control_app_list_scroll_to_app(bool up);
// static void navigation_bar_control_with_gyro(Vector3 *gyro)
// {
//     static float navigation_gyro_z_count = 0;
//     if (gyro->z > 5 || gyro->z < -5)
//     {
//         navigation_gyro_z_count += gyro->z;
//         if (fabs(navigation_gyro_z_count) > SCROLL_APP_THRESHOLD)
//         {
//             last_scroll_time = rt_tick_get_millisecond();
//             if (navigation_gyro_z_count > 0)
//             {
//                 if (is_at_message())
//                 {
//                     mesage_list_scroll_to_app(false);
//                 }
//                 else if (is_at_app_list())
//                 {
//                     app_list_scroll_to_app(false);
//                 }
//             }
//             else
//             {
//                 if (is_at_message())
//                 {
//                     mesage_list_scroll_to_app(true);
//                 }
//                 else if (is_at_app_list())
//                 {
//                     app_list_scroll_to_app(true);
//                 }
//             }
//             navigation_gyro_z_count = 0;
//         }
//     }
// }

static float total_yaw_energy = 0;
static uint8_t scroll_segment_count = 1;
static uint16_t page_range = 100; // 每個頁面的範圍
static float total_moving_distance = 1100.0f;
static uint8_t control_angle = 80; // 預設控制角度為30度
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

static void reset_gesture_state(gesture_dataset_t *dataset,
                                uint32_t current_time, uint8_t code)
{
    if (dataset->gesture_sample_count == 0)
    {
        return;
    }
    dataset->gesture_started = false;
    dataset->gesture_ended = false;
    dataset->gesture_sample_count = 0;
    dataset->wait_start_time = current_time;
    LOG_D("Reset gesture state, code: %d", code);
}

static uint16_t waveform_rtc_millisecond = 0;
static void store_gesture_sample(gesture_dataset_t *dataset, time_t ts,
                                 Vector3 *linear_accel, Vector3 *gravity,
                                 uint32_t ppg_data, bool on_pressed)
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
                                               waveform_gesture_state_t *state)
{
    for (int i = 0; i < ACCEL_WINDOW_SIZE - 1; i++)
    {
        state->sliding_window_accel[i] = state->sliding_window_accel[i + 1];
        state->sliding_window_gravity[i] = state->sliding_window_gravity[i + 1];
        state->sliding_window_ppg[i] = state->sliding_window_ppg[i + 1];
    }
    state->sliding_window_accel[ACCEL_WINDOW_SIZE - 1] = *accel;
    state->sliding_window_gravity[ACCEL_WINDOW_SIZE - 1] = *gravity;
    state->sliding_window_ppg[ACCEL_WINDOW_SIZE - 1] = ppg;
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
 * @brief Packs accelerometer matrix data into a byte buffer
 *
 * @param targetArray The target buffer to store packed data
 * @param matrix The source accelerometer data matrix
 * @param sample_len The number of samples to pack
 */
void packMatrixToBuffer(uint8_t *targetArray, watch_sys_linear_acce_t *dataset,
                        int32_t *fft_buffer, int sample_len)
{
    for (uint8_t i = 0; i < sample_len; i++)
    {
        if (fft_buffer == NULL)
        {
            // timestamp_s(uint32_t -> 4 bytes)
            // LOG_D("Pack sample %d: ts_s=%u, ts_ms=%u, x=%d, y=%d, z=%d,
            // gx=%d, gy=%d, gz=%d",
            //       i,
            //       dataset[i].timestamp_s,
            //       dataset[i].timestamp_ms,
            //       dataset[i].x,
            //       dataset[i].y,
            //       dataset[i].z,
            //       dataset[i].gravity_x,
            //       dataset[i].gravity_y,
            //       dataset[i].gravity_z);
            targetArray[i * BYTES_PER_SAMPLE + 0] =
                (uint8_t)(dataset[i].timestamp_s & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 1] =
                (uint8_t)((dataset[i].timestamp_s >> 8) & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 2] =
                (uint8_t)((dataset[i].timestamp_s >> 16) & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 3] =
                (uint8_t)((dataset[i].timestamp_s >> 24) & 0xFF);
            // timestamp_ms(uint16_t -> 2 bytes)
            targetArray[i * BYTES_PER_SAMPLE + 4] =
                (uint8_t)(dataset[i].timestamp_ms & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 5] =
                (uint8_t)((dataset[i].timestamp_ms >> 8) & 0xFF);
            // x(int16_t -> 2 bytes)
            targetArray[i * BYTES_PER_SAMPLE + 6] =
                (uint8_t)(dataset[i].x & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 7] =
                (uint8_t)((dataset[i].x >> 8) & 0xFF);
            // y(int16_t -> 2 bytes)
            targetArray[i * BYTES_PER_SAMPLE + 8] =
                (uint8_t)(dataset[i].y & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 9] =
                (uint8_t)((dataset[i].y >> 8) & 0xFF);
            // z(int16_t -> 2 bytes)
            targetArray[i * BYTES_PER_SAMPLE + 10] =
                (uint8_t)(dataset[i].z & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 11] =
                (uint8_t)((dataset[i].z >> 8) & 0xFF);
            // gravity_x, gravity_y, gravity_z
            // x(int16_t -> 2 bytes)
            targetArray[i * BYTES_PER_SAMPLE + 12] =
                (uint8_t)(dataset[i].gravity_x & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 13] =
                (uint8_t)((dataset[i].gravity_x >> 8) & 0xFF);
            // y(int16_t -> 2 bytes)
            targetArray[i * BYTES_PER_SAMPLE + 14] =
                (uint8_t)(dataset[i].gravity_y & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 15] =
                (uint8_t)((dataset[i].gravity_y >> 8) & 0xFF);
            // z(int16_t -> 2 bytes)
            targetArray[i * BYTES_PER_SAMPLE + 16] =
                (uint8_t)(dataset[i].gravity_z & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 17] =
                (uint8_t)((dataset[i].gravity_z >> 8) & 0xFF);
            // ppg_data(uint16_t -> 2 bytes)
            targetArray[i * BYTES_PER_SAMPLE + 18] =
                (uint8_t)(dataset[i].ppg_data & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 19] =
                (uint8_t)((dataset[i].ppg_data >> 8) & 0xFF);
            targetArray[i * BYTES_PER_SAMPLE + 20] =
                (uint8_t)(dataset[i].on_pressed ? 1 : 0);
        }
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
extern bool imu_data_collection;
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
static void gesture_event_capture_hcpu(uint16_t freq, time_t ts,
                                       Vector3 *linear_acce, Vector3 *gyro,
                                       Vector3 *gravity, uint32_t ppg,
                                       waveform_gesture_state_t *state,
                                       gesture_type_t type,
                                       gesture_dataset_t *dataset)
{
    rt_tick_t current_time = rt_tick_get_millisecond();

    int cooldown_period = 0;
    if (imu_data_collection)
    {
        cooldown_period = 800;
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
    uint32_t ppg_diff_rawdata =
        abs((int32_t)ppg - (int32_t)prev_ppg_rawdata[2]);

    for (int i = 0; i < 2; i++)
    {
        prev_ppg_rawdata[i] = prev_ppg_rawdata[i + 1];
    }
    prev_ppg_rawdata[2] = ppg;
    if ((current_time - dataset->wait_start_time) < cooldown_period)
    {
        return;
    }

    // Check lock conditions
    // if (motor_provider.get_motor_status())
    // {
    // 	reset_gesture_state(dataset, current_time, 1);
    // 	return;
    // }
    else if (state->if_watchface_visible == false && !imu_data_collection)
    {
        reset_gesture_state(dataset, current_time, 2);
        return;
    }
    else if (state->gyro_lock_status && !imu_data_collection)
    {
        reset_gesture_state(dataset, current_time, 3);
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
        if ((waveform_gesture_state.difference_accel > start_threshold && !open_ppg_chacked) ||
            (ppg_diff_rawdata > 20 && open_ppg_chacked))
        {
            if (ppg_diff_rawdata > 20 && open_ppg_chacked)
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
                    state->sliding_window_ppg[accel_index], state->on_pressed);
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
            reset_gesture_state(dataset, current_time - cooldown_period, 7);
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
    if (get_ppg_count == 0)
    {
        get_ppg_count = 1;
    }
    else
    {
        get_ppg_count = 0;
    }
    uint32_t ppg_rawdata = motion_data->ppg_raw_data.raw_data[get_ppg_count];

    // Update hand position detection
    uint8_t gesture_threshold_factor =
        bloc_setting_get_gesture_detect_threshold();
    float horizontal_threshold = gesture_threshold_factor * 0.01f;
    user_hand_horizontal = (gravity->x < 0.9 && gravity->x > -0.4);

    // Update watchface visibility
    if (gravity->y > -0.7 && gravity->z > -0.6)
    {
        if (!waveform_gesture_state.if_watchface_visible)
        {
            waveform_gesture_state.if_watchface_visible = true;
        }
    }
    else
    {
        if (waveform_gesture_state.if_watchface_visible)
        {
            waveform_gesture_state.if_watchface_visible = false;
        }
    }

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
    if (imu_raw_data_collection)
    {
        store_gesture_sample(&release_dataset, rt_tick_get_millisecond(),
                             linear_acce, gravity, ppg_rawdata,
                             waveform_gesture_state.on_pressed);
        if (release_dataset.gesture_sample_count >= MAX_RAWDATA_TIME_STEP)
        {
            getTargetWaveformFromSlidingWindow(
                &release_dataset, targetWave_algo, MAX_RAWDATA_TIME_STEP);
            // if (!check_ppg_error)
            {
                packMatrixToBuffer(gsensorSamplesBuffer, targetWave_algo, NULL,
                                   release_dataset.gesture_sample_count);
                sensor_buf_t buffer_info = {
                    .data = gsensorSamplesBuffer,
                    .length = release_dataset.gesture_sample_count *
                              BYTES_PER_SAMPLE};
                L1SendData data = {.event = L1SEND_LINEAR_ACCE_BUFFER,
                                   .res.imu_data = buffer_info};
                L1_send_event(data);
            }
            release_dataset.gesture_sample_count = 0;
        }
        return;
    }
    #endif

    // Update sliding windows
    fill_realtime_accel_sliding_window(linear_acce, gravity, ppg_rawdata,
                                       &waveform_gesture_state);
    check_gyro_threshold(gyro, &waveform_gesture_state);

    if (app_control_get_game_mode())
    {
        gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                   linear_acce, gyro, gravity, ppg_rawdata,
                                   &waveform_gesture_state, GESTURE_TYPE_TAP,
                                   &tap_dataset);
        gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                   linear_acce, gyro, gravity, ppg_rawdata,
                                   &waveform_gesture_state,
                                   GESTURE_TYPE_RELEASE, &release_dataset);
    }
    else
    {
        if (SkaiWatchSys.motion_control_lock)
        {
            gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                       linear_acce, gyro, gravity, ppg_rawdata,
                                       &waveform_gesture_state,
                                       GESTURE_TYPE_RELEASE, &release_dataset);
        }
        else
        {
            gesture_event_capture_hcpu(IMU_NOARMAL_SAMPLE_RATE, current_ts,
                                       linear_acce, gyro, gravity, ppg_rawdata,
                                       &waveform_gesture_state,
                                       GESTURE_TYPE_TAP, &tap_dataset);
        }
    }
}

#endif // ENABLE_WAVEFORM_CAPTURE
void set_prev_sensor_quat(uint16_t target_value)
{
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
#ifdef BSP_USING_COMMUNICATE
        L1SendData l1_send_data = {.event = L1SEND_QUATERNION_DATA,
                                   .res = NULL};
        L1_send_event(l1_send_data);
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
extern bool is_fsr_change_detected(void);
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

static bool mouse_movement_lock = false;
static float gyro_movement_distance = 0.0f;

void air_mouse_movement_lock_reset(void)
{
    mouse_movement_lock = true;
    gyro_movement_distance = 0.0f;
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
    float gyro_x = watch_sensor.imu_data.gyro.x * DPS_TO_RADS;
    float gyro_z = watch_sensor.imu_data.gyro.z * DPS_TO_RADS;

    delta_movement = air_mouse_algorithm(gyro_x, gyro_z, AIR_MOUSE_SENSITIVITY);

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

    // if (is_fsr_change_detected())
    // {
    //     LOG_D("FSR change detected, locking mouse movement");
    // }

    // 按住面板才可體感移動，且移動鎖已解除
    if (!stop_mouse_move && is_skai_touch_enabled() && !mouse_movement_lock &&
        !is_fsr_change_detected())
    {
        report_air_mouse_data(&delta_movement, ts);
    }
}
#endif

#if ENABLE_QUICK_ACTION
static uint8_t pevr_quick_btn_state = 6;
static int new_point = 0;
static uint16_t pevr_point = 0;
static uint8_t swich_quick_btn = 2; // 0:左icon, 1:上icon, 2:右icon 3:全消失
static float conversion_ratio = 0.0f;
static bool close_quick_app = false;
static void quick_action_process(Vector3 *gravity)
{
    if (gravity->x > 0.5 && !is_at_home() &&
        !gui_app_is_actived(APP_ID_WIDGETS) &&
        !gui_app_is_actived(APP_ID_MESSAGE) && !is_ai_interface_active())
    {
        if (gravity->x < 0.61)
        {
            swich_quick_btn = 3;
            new_point = (gravity->x - 0.5) * 500;
        }
        else
        {
            swich_quick_btn = 4;
        }
    }
    else
    {
        new_point = 0;
        swich_quick_btn = 0;
    }

    if (new_point != pevr_point || swich_quick_btn != pevr_quick_btn_state)
    {
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_CONTROL_QUICK_BTN;
        msg.data.quick_btn_action.new_point = new_point;
        msg.data.quick_btn_action.swich_quick_btn = swich_quick_btn;
        lvgl_send_msg(msg);
        pevr_point = new_point;
        pevr_quick_btn_state = swich_quick_btn;
    }
}
#endif

static float standard_gravity_components[3] = {0.0f, 0.0f, 0.0f};
void set_standard_value_of_gravity_components(void)
{
    motion_data_t motion_data = watch_sensor.motion_data;
    Vector3 gravity = motion_data.gravity;
    standard_gravity_components[0] = gravity.x;
    standard_gravity_components[1] = gravity.y;
    standard_gravity_components[2] = gravity.z;
    LOG_D("standard_gravity_components[0]:%f", standard_gravity_components[0]);
}

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

void set_movement_scale_ratio(float ratio)
{
    if (ratio > 0.0f && ratio <= 3.0f) // 限制比例範圍在0.1到2.0之間
    {
        movement_scale_ratio = ratio;
    }
}

float get_movement_scale_ratio(void)
{
    return movement_scale_ratio;
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
            // rt_tick_t current_tick = rt_tick_get();
            // if (current_tick - last_vibrate_time_at_boundary >=
            // rt_tick_from_millisecond(MOTOR_INTERVAL_MS))
            // {
            // 	last_vibrate_time_at_boundary = current_tick; // Update last log
            // time 	motor_provider.motor_vibrate_scrolling_app();
            // }
            gyro_z_count = overflow_threshold;
        }
    }
    else if (gyro_z_count >=
             ((threshold + overflow_threshold) * 2 - overflow_threshold))
    {
        raw_x = 0;
        if (gyro_z_count >= (threshold + overflow_threshold) * 2)
        {
            // rt_tick_t current_tick = rt_tick_get();
            // if (current_tick - last_vibrate_time_at_boundary >=
            // rt_tick_from_millisecond(MOTOR_INTERVAL_MS))
            // {
            // 	last_vibrate_time_at_boundary = current_tick; // Update last log
            // time 	motor_provider.motor_vibrate_scrolling_app();
            // }
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
        if (media_x_control < 0)
            media_x_control = 0;
        if (media_y_control > 466)
            media_y_control = 466;
        if (media_y_control < 0)
            media_y_control = 0;
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

static bool is_hand_lifting = false; // Track the current state
bool check_hand_lifting(void)
{
    return is_hand_lifting;
}

static int gravity_position = GRAVITY_POSITION_OTHER;

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
extern void set_is_open_app_list_ai(bool open);
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
        is_at_app_list()) // && get_is_at_ai_widget()
    {
        // watch_system_interact(INTERACT_MOTOR_VIBRATE_TEST, NULL);
        // animate_to_ai_page();
        // is_ai_open_mic = true;
        // show_speech_indicator(true);
        // voice_provider.start_v2t();
        // set_is_open_app_list_ai(true);
        extern void animate_open_ai_widget(void);
        animate_open_ai_widget();
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
    LOG_D("reset_gravity_position");
    gravity_position = GRAVITY_POSITION_OTHER;

#ifdef SHOW_UNGRAB_ENABLE_INDICATOR
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_RELEASE_INDICATOR;
    msg.data.message = APP_ID_MAIN;
    lvgl_send_msg(msg);
#endif
}

static bool ai_interface_lock_flag = false;
static void ai_interface_lock(void)
{
    ai_interface_lock_flag = true;
}

static void ai_interface_unlock(void)
{
    ai_interface_lock_flag = false;
}

static bool can_open_ai_interface(void)
{
    // Check if the AI interface can be opened
    if (is_at_home() || is_at_control_center() || is_at_mouse_mode() ||
        ai_interface_lock_flag || gui_app_is_actived(APP_ID_FLASHLIGHT) ||
        gui_app_is_actived(APP_ID_TIMER) || gui_app_is_actived(APP_ID_MOUSE) ||
        gui_app_is_actived(APP_ID_EXERCISE) ||
        gui_app_is_actived(APP_ID_GESTURE) ||
        gui_app_is_actived(APP_ID_RECORDER) ||
        // gui_app_is_actived(APP_ID_BATTERY) ||
        // gui_app_is_actived(APP_ID_MEDIA) ||
        (!is_ai_open_mic && app_voice_get_listening_status()))
    {
        return false;
    }
    return true;
}

// extern void set_ai_hint_x(uint8_t x);
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
    if (gravity->y < -0.7 && gravity->z < 0.3)
    {
        set_gravity_position(GRAVITY_POSITION_SIDE);
    }
    else if (gravity->x > 0.5 && can_open_ai_interface())
    {
        set_gravity_position(GRAVITY_POSITION_AI);
    }
    else if ((gravity->x < 0.5 && gravity->x > -0.3) && fabs(gravity->y) < 0.5)
    {
        set_gravity_position(GRAVITY_POSITION_HORIZONTAL);
    }
    else
    {
        set_gravity_position(GRAVITY_POSITION_OTHER);
    }
}

void widget_ai_open(bool is_open)
{
    if (is_open)
    {
        is_hand_lifting = true;
    }
    else
    {
        is_hand_lifting = false;
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
    extern bool get_enable_tap_and_hold(void);
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

extern bool get_is_open_app_list_ai(void);
extern uint8_t get_message_page_count(void);
static euler_angle_t pevr_befor_switch_widget_delta_angle;
static float prev_delta_roll = 0.0f;
static float prev_delta_yaw = 0.0f;
static void motion_tracking_in_hcpu(motion_data_t *motion_data)
{
    static Quaternion prev_global_quat = {.w = 1, .x = 0, .y = 0, .z = 0};
    watch_sensor_motion_data = motion_data;
    if (is_ble_dfu_thread_running())
    {
        return;
    }

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
#if ENABLE_QUICK_ACTIONs
    if (motion_data->gravity.z > -0.2)
    {
        if (!peripheral_provider.get_tap_status())
        {
            quick_action_process(&motion_data->gravity);
        }
    }
#endif

    extern bool get_enable_tap_and_hold(void);
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
    if (app_control_get_mouse_mode() || is_at_mouse_mode() ||
        gui_app_is_actived(APP_ID_MOUSE)) // app_control_get_cursor_mode()
    {
        air_mouse_process(motion_data->timestamp, &motion_data->global_q,
                          &prev_global_quat);
        prev_global_quat = motion_data->global_q;
        // LOG_D("air mouse process, global_q: w:%f, x:%f, y:%f, z:%f",
        // motion_data->global_q.w, motion_data->global_q.x,
        // motion_data->global_q.y, motion_data->global_q.z);
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

        // if (!peripheral_provider.get_tap_status())
        if (!is_at_message() && !get_is_open_app_list_ai())
        {
            if (motion_data->gravity.x < 0.3 && motion_data->gravity.x > -0.3)
            {
                set_paused_control_with_arm(false);
            }
            else
            {
                set_paused_control_with_arm(true);
            }
        }
        {
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
                    // navigation_bar_control_with_gyro(&watch_sensor.imu_data.gyro);
                    // LOG_D("watch_sensor.imu_data.gyro
                    // x:%0.5f,y:%0.5f,z:%0.5f",
                    // watch_sensor.imu_data.gyro.x,watch_sensor.imu_data.gyro.y,watch_sensor.imu_data.gyro.z);
                    if (!paused_control_with_arm) //&&
                                                  // fabs(watch_sensor.imu_data.gyro.x)
                                                  //< 20
                    {
                        extern bool get_enable_tap_and_hold(void);
                        if (!get_enable_tap_and_hold() ||
                            peripheral_provider.get_tap_status())
                        {
                            // if (diff_delta_roll < diff_delta_yaw)//
                            // diff_delta_roll < diff_delta_yaw * 0.8 &&
                            // diff_delta_roll < 0.3f
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
#if ENABLE_SEND_GRAVITY_TO_BLE_CLIENT
                    L1SendData data = {.event = L1SEND_GSENSOR_GRAVITY_DATA};
                    L1_send_event(data);
#endif
                }
                else if (open_control_options && !is_at_ai_interface())
                {
                    app_control_interface(&watch_sensor.imu_data.gyro,
                                          &motion_data->gravity);
                }
                // LOG_D("control_options:%d,%d,%d", paused_control_with_arm,
                // free_control_with_arm(),is_at_ai_interface());
            }
        }
        extern bool get_enable_tap_and_hold(void);
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

#if ENABLE_MEDIA_VOLUMN_BAR_CONTROL
        else if (gui_app_is_actived(APP_ID_MEDIA))
        {
            media_use_q_vertical_movement(befor_tap_delta_angle.yaw);
        }
#endif
    }
}

#ifdef REAL_TIME_ACCEL_STEPS_COLLECTION
extern bool accel_steps_data_collection;
extern bool imu_6Axis_data_collection;
static uint8_t accel_sample_num = 0;
static uint8_t accelSamplesBuffer[384] = {0}; // 64 * 6 = 384
#endif

/*
 ***** Motion Tracking processing
 */
static void motion_tracking_thread_entry(void *parameter)
{
    watch_sensor.imu_sem = rt_sem_create("imu_sem", 0, RT_IPC_FLAG_FIFO);
    while (1)
    {
        rt_sem_take(watch_sensor.imu_sem, RT_WAITING_FOREVER);
        motion_data_t motion_data = watch_sensor.motion_data;
#ifdef REAL_TIME_ACCEL_STEPS_COLLECTION
        if (accel_steps_data_collection)
        {
            if (accel_sample_num < 64)
            {
                int16_t accel[3];
                accel[0] = watch_sensor.imu_data.acce.x * INT16_to_G / GRAVITY;
                accel[1] = watch_sensor.imu_data.acce.y * INT16_to_G / GRAVITY;
                accel[2] = watch_sensor.imu_data.acce.z * INT16_to_G / GRAVITY;
                memcpy(&accelSamplesBuffer[accel_sample_num * 6], accel, 6);
                accel_sample_num++;
            }
            else
            {
                sensor_buf_t buffer_info = {.data = accelSamplesBuffer,
                                            .length = 384};
                L1SendData data = {.event = L1SEND_LINEAR_ACCE_BUFFER,
                                   .res.imu_data = buffer_info};
                L1_send_event(data);
                accel_sample_num = 0;
            }
        }
        else if (imu_6Axis_data_collection)
        {
            if (accel_sample_num < 32)
            {
                int16_t imu[6];
                imu[0] = watch_sensor.imu_data.acce.x * INT16_to_G / GRAVITY;
                imu[1] = watch_sensor.imu_data.acce.y * INT16_to_G / GRAVITY;
                imu[2] = watch_sensor.imu_data.acce.z * INT16_to_G / GRAVITY;
                imu[3] = watch_sensor.imu_data.gyro.x * INT16_to_DPS;
                imu[4] = watch_sensor.imu_data.gyro.y * INT16_to_DPS;
                imu[5] = watch_sensor.imu_data.gyro.z * INT16_to_DPS;
                memcpy(&accelSamplesBuffer[accel_sample_num * 12], imu, 12);
                accel_sample_num++;
            }
            else
            {
                sensor_buf_t buffer_info = {.data = accelSamplesBuffer,
                                            .length = 384};
                L1SendData data = {.event = L1SEND_IMU_BUFFER,
                                   .res.imu_data = buffer_info};
                L1_send_event(data);
                accel_sample_num = 0;
            }
        }
#endif
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
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
INIT_APP_EXPORT(motion_tracking_thread_init);

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/