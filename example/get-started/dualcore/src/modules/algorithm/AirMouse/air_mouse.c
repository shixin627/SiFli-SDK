/**
 ******************************************************************************
 * @file   air_mouse.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <math.h>
#include <rtthread.h>
#include "air_mouse.h"

// Gyro-based air mouse parameters (ported from GyroService)
#define GYRO_SMOOTHING_FACTOR   0.8f
#define GYRO_DEAD_ZONE          0.05f
#define GYRO_ACCEL_THRESHOLD    0.3f
#define GYRO_ACCEL_EXPONENT     1.2f
#define GYRO_ACCEL_MULTIPLIER   1.5f

// Exponential smoothing state
static float last_smoothed_x = 0.0f;
static float last_smoothed_z = 0.0f;

/**
 * @brief Apply non-linear acceleration curve.
 *
 * Below the threshold the value passes through linearly for precise control.
 * Above the threshold an exponential curve is applied so fast hand movements
 * produce larger cursor jumps.
 */
static float apply_acceleration(float value)
{
    float abs_val = fabsf(value);
    if (abs_val < GYRO_ACCEL_THRESHOLD)
    {
        return value;
    }
    float excess = abs_val - GYRO_ACCEL_THRESHOLD;
    float accelerated = GYRO_ACCEL_THRESHOLD +
                        powf(excess, GYRO_ACCEL_EXPONENT) * GYRO_ACCEL_MULTIPLIER;
    return (value > 0.0f ? 1.0f : -1.0f) * accelerated;
}

void air_mouse_reset(void)
{
    last_smoothed_x = 0.0f;
    last_smoothed_z = 0.0f;
}

/**
 * @brief Gyro-based air mouse algorithm (ported from GyroService).
 *
 * Reads gyroscope rotation rates and converts them to relative mouse
 * movement deltas using exponential smoothing, dead-zone filtering,
 * and a non-linear acceleration curve.
 *
 * @param gyro_x  Gyroscope X-axis rotation rate (rad/s)
 * @param gyro_z  Gyroscope Z-axis rotation rate (rad/s)
 * @param sensitivity  Mouse sensitivity multiplier
 * @return Delta movement for BLE HID report
 */
air_plane_delta_movement_t air_mouse_algorithm(float gyro_x, float gyro_z, float sensitivity)
{
    air_plane_delta_movement_t movement = {0, 0, 0};

    // Exponential smoothing to reduce jitter
    float x = GYRO_SMOOTHING_FACTOR * gyro_x + (1.0f - GYRO_SMOOTHING_FACTOR) * last_smoothed_x;
    float z = GYRO_SMOOTHING_FACTOR * gyro_z + (1.0f - GYRO_SMOOTHING_FACTOR) * last_smoothed_z;
    last_smoothed_x = x;
    last_smoothed_z = z;

    // Dead-zone filter to ignore micro-jitter
    if (fabsf(x) < GYRO_DEAD_ZONE) x = 0.0f;
    if (fabsf(z) < GYRO_DEAD_ZONE) z = 0.0f;

    // Apply non-linear acceleration curve
    x = apply_acceleration(x);
    z = apply_acceleration(z);

    // Map gyro axes to mouse axes:
    //   gyro Z rotation -> horizontal cursor movement (deltaX)
    //   gyro X rotation -> vertical cursor movement (deltaY)
    float deltaX = -z * sensitivity;
    float deltaY = -x * sensitivity;

    // Clamp to int8_t range
    if (deltaX > 127.0f) deltaX = 127.0f;
    else if (deltaX < -128.0f) deltaX = -128.0f;
    if (deltaY > 127.0f) deltaY = 127.0f;
    else if (deltaY < -128.0f) deltaY = -128.0f;

    movement.x = (int8_t)deltaX;
    movement.y = (int8_t)deltaY;

    return movement;
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
