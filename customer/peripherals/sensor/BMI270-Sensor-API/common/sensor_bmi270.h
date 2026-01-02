/**
  ******************************************************************************
  * @file   sensor_bmi270.h
  * @author Skaiwalk software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2018 - 2024,  Skaiwalk Technology
 *
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
 * 3. Neither the name of Skaiwalk nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
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
 *
 */

#ifndef SENSOR_BMI270_H__
#define SENSOR_BMI270_H__

#include "board.h"
#include "sensor.h"

#include "bmi270.h"
#include "bmi270_driver.h"

/* Accelerometer full scale range */
enum bmi270_accel_range
{
    BMI270_ACCEL_RANGE_2G  = 0, // ±2G
    BMI270_ACCEL_RANGE_4G  = 1, // ±4G
    BMI270_ACCEL_RANGE_8G  = 2, // ±8G
    BMI270_ACCEL_RANGE_16G = 3  // ±16G
};

/* Gyroscope full scale range */
enum bmi270_gyro_range
{
    BMI270_GYRO_RANGE_250DPS  = 0, // ±250°/s
    BMI270_GYRO_RANGE_500DPS  = 1, // ±500°/s
    BMI270_GYRO_RANGE_1000DPS = 2, // ±1000°/s
    BMI270_GYRO_RANGE_2000DPS = 3  // ±2000°/s
};

/* Digital Low Pass Filter parameters */
enum bmi270_dlpf
{
    BMI270_DLPF_DISABLE = 0, //256HZ
    BMI270_DLPF_188HZ = 1,
    BMI270_DLPF_98HZ  = 2,
    BMI270_DLPF_42HZ  = 3,
    BMI270_DLPF_20HZ  = 4,
    BMI270_DLPF_10HZ  = 5,
    BMI270_DLPF_5HZ   = 6
};

/* sleep mode parameters */
enum bmi270_sleep
{
    BMI270_SLEEP_DISABLE = 0,
    BMI270_SLEEP_ENABLE  = 1
};

/* Supported configuration items */
enum bmi270_cmd
{
    BMI270_GYRO_RANGE,  /* Gyroscope full scale range */
    BMI270_ACCEL_RANGE, /* Accelerometer full scale range */
    BMI270_DLPF_CONFIG, /* Digital Low Pass Filter */
    BMI270_SAMPLE_RATE, /* Sample Rate —— 16-bit unsigned value.
                            Sample Rate = [1000 -  4]HZ when dlpf is enable
                            Sample Rate = [8000 - 32]HZ when dlpf is disable */
    BMI270_SLEEP        /* Sleep mode */
};

/* 3-axis data structure */
struct bmi270_3axes
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};

/* icm20948 config structure */
struct bmi270_config
{
    rt_uint16_t accel_range;
    rt_uint16_t gyro_range;
};

/* icm20948 device structure */
struct bmi270_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    struct bmi270_config config;
};

int rt_hw_bmi270_register(const char *name, struct rt_sensor_config *cfg);
int rt_hw_bmi270_init(void);
int rt_hw_bmi270_deinit(void);
extern void bmi270_sensor_power_high_mode(void);
extern void bmi270_sensor_power_low_mode(void);

#endif  // SENSOR_BMI270_H__
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
