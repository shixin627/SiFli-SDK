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
static Quaternion _q;
Quaternion get_attitude(void)
{
    return _q;
}

//-------------------------------------------------------------------------------------------
// AHRS algorithm update
Quaternion updateAHRS(sensor_fusion_param_t *param, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid
    // (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        return updateIMU(param, gx, gy, gz, ax, ay, az);
    }

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = param->q0 * param->q0;
        q0q1 = param->q0 * param->q1;
        q0q2 = param->q0 * param->q2;
        q0q3 = param->q0 * param->q3;
        q1q1 = param->q1 * param->q1;
        q1q2 = param->q1 * param->q2;
        q1q3 = param->q1 * param->q3;
        q2q2 = param->q2 * param->q2;
        q2q3 = param->q2 * param->q3;
        q3q3 = param->q3 * param->q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction
        // and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

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
    Quaternion q = {.w = param->q0, .x = param->q1, .y = param->q2, .z = param->q3};
    return q;
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

euler_angle_t QuaternionToEuler(Quaternion q)
{
    euler_angle_t euler;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.roll = (float)atan2(sinr_cosp, cosr_cosp) * 57.29577951f;

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        euler.pitch = (float)copysign(90.0f, sinp); // use 90 degrees if out of range
    else
        euler.pitch = (float)asin(sinp) * 57.29577951f;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.yaw = (float)atan2(siny_cosp, cosy_cosp) * 57.29577951f;

    return euler;
}

//============================================================================================
// END OF CODE
//============================================================================================