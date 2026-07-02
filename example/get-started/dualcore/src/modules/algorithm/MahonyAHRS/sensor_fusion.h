//=============================================================================================
// sensor_fusion.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
//=============================================================================================
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H
#include "watch_global_data.h"

typedef struct
{
	volatile float q0, q1, q2, q3;
	volatile float integralFBx, integralFBy, integralFBz;
} sensor_fusion_param_t;

#define DEGREE_TO_RADIAN(degree) (0.01745329251994329576923690768489 * degree)
#define RADIAN_TO_DEGREE(rad) ((rad) * 57.29577951f)

//-------------------------------------------------------------------------------------------
// Function declarations
extern void setSampleFrequencyAHRS(float freq);
extern Quaternion updateIMU(sensor_fusion_param_t *param, float gx, float gy, float gz, float ax, float ay, float az);
extern void resetAHRS(sensor_fusion_param_t *param);
/* Seed the quaternion from a single gravity-only accel reading so the first
   updateIMU() is already converged (skips the ~3 s Mahony cold start). */
extern void seedAHRSFromAccel(sensor_fusion_param_t *param, float ax, float ay, float az);

#endif
