/**
 ******************************************************************************
 * @file   gh3018.c
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

#ifndef GOODIX_GH3018_H__
#define GOODIX_GH3018_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "board.h"
#include "sensor.h"
#include "gh30x_example.h"

    extern uint32_t gh3018_get_i2c_handle(void);

    extern uint8_t gh3018_get_dev_addr(void);

    extern int gh3018_self_check(void);

    extern uint32_t gh3018_get_hr(void);

    extern void gh3018_set_hr(uint32_t hr);

    extern uint32_t *gh3018_get_ppg(void);
    extern uint32_t *gh3018_get_ppg2(void);

    extern int init_gh3018_sensor(void);
    extern void gh30x_api_lock(void);
    extern void gh30x_api_unlock(void);
    extern int open_gh3018(void);
    extern int open_gh3018_high_power(void);
    extern int set_gh3018_hr_mode(void);
    extern int close_gh3018(void);
    extern void soft_adt_callback(bool status);

#ifdef __cplusplus
}
#endif

#endif // GOODIX_GH3018_H__

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/