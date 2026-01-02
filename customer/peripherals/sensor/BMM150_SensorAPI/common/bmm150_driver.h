/**
 ******************************************************************************
 * @file   bmm150_driver.c
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

#ifndef __BMM150_SENSOR_HDR_FILE__
#define __BMM150_SENSOR_HDR_FILE__

#include "bmm150.h"

extern struct bmi2_sens_axes_data *bmm150_get_magnet(void);

int bmm150_initialized(void);
uint32_t bmm150_get_bus_handle(void);
uint8_t bmm150_get_dev_addr(void);
uint8_t bmm150_get_dev_id(void);
int bmm150_open(void);
int bmm150_close(void);
int bmm150_self_check(void);
int bmm150_mag_read(int16_t *psX, int16_t *psY, int16_t *psZ);
void bmm150_set_range(uint8_t range);

#endif /* __bmm150_SENSOR_HDR_FILE__*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
