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
    /* Legacy AHRS-only update for the screen-off FIFO drain path. No longer
       armed (gyro suspended in DARK, FIFO watermark INT off); retained only
       because bmi270_drain_fifo_to_ahrs() in the sensor driver references
       it. Feeds one accel+gyro pair into the global Mahony filter. */
    extern void update_global_attitude(Vector3 *accData, Vector3 *gyroData);

    /* Pose-verify the watch is in "looking at watch" attitude from a single
       fresh accel sample read at the wrist-wake firing instant. Normalises
       the (watch-frame) accel to get the gravity direction and applies the
       same envelope the awake-time SW algorithm uses for `watchface_visible`.
       Used by the HW wrist-wake INT handler to suppress false triggers
       before waking HCPU. */
    extern bool is_in_viewing_pose_from_accel(float ax, float ay, float az);

    /* Seed the screen-on global AHRS from a single gravity-only accel sample
       so the first post-wake updateIMU() is already converged (skips the
       ~3 s Mahony cold start). Accel must be in the watch frame. */
    extern void gesture_seed_attitude_from_accel(float ax, float ay, float az);

#ifdef __cplusplus
}
#endif

#endif // GESTURE_DETECT_TASK_H