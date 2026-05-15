#ifndef GESTURE_DETECT_TASK_H
#define GESTURE_DETECT_TASK_H

#include <stdbool.h>
#include "watch_global_data.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void tap_detected_callback(uint8_t tap_mode);
    void calibrate_global_attitude(void);
    extern void process_ppg_rawdata(uint32_t rawdata);
    extern int handle_imu_data(float hz, Vector3 *accData, Vector3 *gyroData);
    /* AHRS-only update for the screen-off FIFO drain path — feeds one
       accel+gyro pair into the global Mahony filter without doing any of
       the awake-time work (gravity, hand tracking, sensor_q, health). */
    extern void update_global_attitude(Vector3 *accData, Vector3 *gyroData);

    /* Pose-verify the watch is in "looking at watch" attitude RIGHT NOW.
       Computes gravity from the current global_q (kept fresh by FIFO drain
       during screen-off) and applies the same envelope the awake-time SW
       algorithm uses for `watchface_visible`. Used by HW wrist-wake INT
       handler to suppress false triggers before waking HCPU. */
    extern bool is_in_viewing_pose(void);

#ifdef __cplusplus
}
#endif

#endif // GESTURE_DETECT_TASK_H