#ifndef GESTURE_DETECT_TASK_H
#define GESTURE_DETECT_TASK_H

#include "watch_global_data.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void tap_detected_callback(uint8_t tap_mode);
    void calibrate_global_attitude(void);
    void reinitialize_ahrs_from_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z);
    extern void process_ppg_rawdata(uint32_t rawdata);
    extern int handle_imu_data(float hz, Vector3 *accData, Vector3 *gyroData);

#ifdef __cplusplus
}
#endif

#endif // GESTURE_DETECT_TASK_H