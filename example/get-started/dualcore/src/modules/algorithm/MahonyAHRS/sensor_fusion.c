/**
 ******************************************************************************
 * @file   sensor_fusion.c
 * @author Skaiwalk software development team
 * @brief Madgwick's implementation of Mayhony's AHRS algorithm.
 * See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * From the x-io website "Open-source resources available on this website are
 * provided under the GNU General Public Licence unless an alternative licence
 * is provided in source."
 *
 * Algorithm paper:
 * http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934&url=http%3A%2F%2Fieeexplore.ieee.org%2Fstamp%2Fstamp.jsp%3Ftp%3D%26arnumber%3D4608934
 *
 * Implementation:
 * https://ahrs.readthedocs.io/en/latest/filters/madgwick.html
 *
 ******************************************************************************
 */
//-------------------------------------------------------------------------------------------
// Header files

#include "sensor_fusion.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define DEFAULT_SAMPLE_FREQ 100.0f // sample frequency in Hz
#define DEFAULT_INV_SAMPLE_FREQ (1.0f / DEFAULT_SAMPLE_FREQ)
#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain
#define DEG_TO_RAD 0.01745329251994329576923690768489f
#define _gyroMeasError (40.0f * DEG_TO_RAD)
#define _beta (sqrt(3.0f / 4.0f) * _gyroMeasError)
#define M_PI 3.14159265358979323846f
//---------------------------------------------------------------------------------------------------
// Variable definitions
volatile float twoKp = twoKpDef;                                                          // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                                                          // 2 * integral gain (Ki)
// volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                                // quaternion of sensor frame relative to auxiliary frame
volatile Quaternion calibrated_quaternion = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f}; // quaternion of sensor frame relative to auxiliary frame
// volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;                // integral error terms scaled by Ki
float sampleFrequency = DEFAULT_SAMPLE_FREQ;
float invSampleFreq = DEFAULT_INV_SAMPLE_FREQ;
//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    union
    {
        float f;
        long l;
    } i;
    i.f = x;
    i.l = 0x5f3759df - (i.l >> 1);
    float y = i.f;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void getInverseFrequency(float freq)
{
    // Get the inverse of the sample frequency
    invSampleFreq = 1.0f / freq;
}

void setSampleFrequencyAHRS(float freq)
{
    // rt_kprintf("sampleFreq: %f\n", freq);
    sampleFrequency = freq;
    getInverseFrequency(freq);
}
//-------------------------------------------------------------------------------------------
// IMU algorithm update

/// @brief
/// @param gx gyroscope x (radians/sec)
/// @param gy gyroscope y (radians/sec)
/// @param gz gyroscope z (radians/sec)
/// @param ax accelerometer x (m/s^2)
/// @param ay accelerometer y (m/s^2)
/// @param az accelerometer z (m/s^2)
/// @return
Quaternion updateIMU(sensor_fusion_param_t *param, float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    // float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // norm = sqrt(ax * ax + ay * ay + az * az);
        // ax /= norm;
        // ay /= norm;
        // az /= norm;

        // Estimated direction of gravity
        halfvx = param->q1 * param->q3 - param->q0 * param->q2;
        halfvy = param->q0 * param->q1 + param->q2 * param->q3;
        halfvz = param->q0 * param->q0 - 0.5f + param->q3 * param->q3;
        // Error is sum of cross product between estimated
        // and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            // integral error scaled by Ki
            param->integralFBx += twoKi * halfex * invSampleFreq;
            param->integralFBy += twoKi * halfey * invSampleFreq;
            param->integralFBz += twoKi * halfez * invSampleFreq;
            gx += param->integralFBx; // apply integral feedback
            gy += param->integralFBy;
            gz += param->integralFBz;
        }
        else
        {
            param->integralFBx = 0.0f; // prevent integral windup
            param->integralFBy = 0.0f;
            param->integralFBz = 0.0f;
        }
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * invSampleFreq); // pre-multiply common factors
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);
    qa = param->q0;
    qb = param->q1;
    qc = param->q2;
    param->q0 += (-qb * gx - qc * gy - param->q3 * gz);
    param->q1 += (qa * gx + qc * gz - param->q3 * gy);
    param->q2 += (qa * gy - qb * gz + param->q3 * gx);
    param->q3 += (qa * gz + qb * gy - qc * gx);
    // Normalise quaternion
    recipNorm = invSqrt(param->q0 * param->q0 + param->q1 * param->q1 + param->q2 * param->q2 + param->q3 * param->q3);
    param->q0 *= recipNorm;
    param->q1 *= recipNorm;
    param->q2 *= recipNorm;
    param->q3 *= recipNorm;
    // norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    // q0 /= norm;
    // q1 /= norm;
    // q2 /= norm;
    // q3 /= norm;
    Quaternion q = {.w = param->q0, .x = param->q1, .y = param->q2, .z = param->q3};
    return q;
}

// AHRS algorithm reset
void resetAHRS(sensor_fusion_param_t *param)
{
    param->q0 = 1.0f;
    param->q1 = 0.0f;
    param->q2 = 0.0f;
    param->q3 = 0.0f;
    param->integralFBx = 0.0f;
    param->integralFBy = 0.0f;
    param->integralFBz = 0.0f;
}

// Seed the quaternion directly from a single (assumed gravity-only) accel
// reading so the very first updateIMU() sample is already converged -- no
// Mahony proportional-correction cold start (~3 s). The chosen quaternion
// has zero yaw and rotates body +Z onto the measured gravity direction, so
// calculate_gravity(q) reproduces the normalised input exactly (signs match
// this codebase's calculate_gravity / updateIMU convention: gravity reads
// +Z when the watch lies flat). Integral terms are cleared.
void seedAHRSFromAccel(sensor_fusion_param_t *p, float ax, float ay, float az)
{
    float n = invSqrt(ax * ax + ay * ay + az * az);
    ax *= n;
    ay *= n;
    az *= n;
    if (az >= 0.9999f)
    {
        p->q0 = 1;
        p->q1 = 0;
        p->q2 = 0;
        p->q3 = 0;
    }
    else if (az <= -0.9999f)
    {
        p->q0 = 0;
        p->q1 = 1;
        p->q2 = 0;
        p->q3 = 0;
    }
    else
    {
        p->q0 = sqrtf((1.0f + az) * 0.5f);
        float inv = 1.0f / (2.0f * p->q0);
        p->q1 = ay * inv;
        p->q2 = -ax * inv;
        p->q3 = 0.0f;
    }
    p->integralFBx = p->integralFBy = p->integralFBz = 0.0f;
}

//============================================================================================
// END OF CODE
//============================================================================================