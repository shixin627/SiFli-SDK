/**
 ******************************************************************************
 * @file   bmi270_driver.h
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * @attention
 * Copyright (c) 2018 - 2023,  Skaiwalk Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __BMI270_SENSOR_HDR_FILE__
#define __BMI270_SENSOR_HDR_FILE__

#include <stdbool.h>
#include "bmi270.h"

typedef enum
{
    BMI270_FIFO_ODR_DISABLE = 0,
    BMI270_FIFO_ODR_12Hz5 = 1,
    BMI270_FIFO_ODR_26Hz = 2,
    BMI270_FIFO_ODR_52Hz = 3,
    BMI270_FIFO_ODR_104Hz = 4,
    BMI270_FIFO_ODR_208Hz = 5,
    BMI270_FIFO_ODR_416Hz = 6,
    BMI270_FIFO_ODR_833Hz = 7,
    BMI270_FIFO_ODR_1k66Hz = 8,
    BMI270_FIFO_ODR_3k33Hz = 9,
    BMI270_FIFO_ODR_6k66Hz = 10,
    BMI270_FIFO_ODR_RATE_ND = 11, /* ERROR CODE */
} bmi270_fifo_odr_t;

typedef enum
{
    BMI270_FIFO_GYRO = 0,
    BMI270_FIFO_XL = 1,
    BMI270_FIFO_TEMP = 2,
    BMI270_FIFO_STEP = 3,
    BMI270_FIFO_CNT
} bmi270_fifo_func_t;

typedef enum
{
#ifdef USING_GYRO_SENSOR
    BMI270_FIFO_PATTERN_GX1 = 0,
    BMI270_FIFO_PATTERN_GY2 = 1,
    BMI270_FIFO_PATTERN_GZ3 = 2,
    BMI270_FIFO_PATTERN_XLX1 = 3,
    BMI270_FIFO_PATTERN_XLY2 = 4,
    BMI270_FIFO_PATTERN_XLZ3 = 5,
#else
    BMI270_FIFO_PATTERN_XLX1 = 0,
    BMI270_FIFO_PATTERN_XLY2 = 1,
    BMI270_FIFO_PATTERN_XLZ3 = 2,
#endif
} bmi270_fifo_pattern_id_t;

// #define BMI270_USE_INT
struct bmi2_sens_axes_data *bmi270_get_accel(void);
struct bmi2_sens_axes_data *bmi270_get_gyro(void);
bool power_opt_mode(void);

int bmi270_initialized(void);
uint32_t bmi270_get_bus_handle(void);
uint8_t bmi270_get_dev_addr(void);
uint8_t bmi270_get_dev_id(void);
int bmi270_open(void);
int bmi270_high_performance_mode(void);
int bmi270_low_power_mode(void);
int bmi270_close(void);
long rt_bmi270_irq_pin_enable(uint32_t enabled);
int bmi270_fifo_enable(bmi270_fifo_func_t func, bmi270_fifo_odr_t rate);
int bmi270_fifo_disable(bmi270_fifo_func_t func);
int bmi270_get_fifo_count(void);
int bmi270_read_fifo(uint8_t *buf, int len);
int bmi270_set_fifo_threshold(int thd);
int bmi270_get_waterm_status(void);
int bmi270_get_overrun_status(void);
int bmi270_get_fifo_full_status(void);
int bmi270_get_fifo_empty_status(void);
int bmi270_set_fifo_mode(uint8_t val);
int bmi270_get_fifo_pattern(void);
int bmi270_get_fifo_data_arr(void);

int bmi270_self_check(void);
int bmi270_awt_enable(int en);
int bmi270_pedo_enable(int en);
int bmi270_pedo_fifo2step(uint8_t *buf, int len);

int bmi270_gyro_read(int16_t *psX, int16_t *psY, int16_t *psZ);
int bmi270_accel_read(int16_t *psX, int16_t *psY, int16_t *psZ);
int bmi270_tempra_read(float *tempra);
int bmi270_step_read(int32_t *step);

void bmi270_accel_set_range(uint8_t range);
void bmi270_gyro_set_range(uint8_t range);

#endif /* __BMI270_SENSOR_HDR_FILE__*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
