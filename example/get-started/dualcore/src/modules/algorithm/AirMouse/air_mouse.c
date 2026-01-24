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

#define SCALE_FACTOR 1.0f // 縮放因子
#define OFFSET_Y 1.0f       // 偏移量
#define OFFSET_X 5.0f       // 偏移量

#define FINAL_SCALE_FACTOR 2.0f // 縮放因子

//-------------------------------------------------------------------------------------------
// Definition of functions

float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

air_plane_delta_movement_t air_mouse_algorithm(float *x, float *y, float dx, float dy, float multi_factor)
{
    static uint8_t count_of_below_bound = 0;
    air_plane_delta_movement_t movement = {0, 0};

    float dx_in_pixel = dx * multi_factor;
    float dy_in_pixel = dy * multi_factor;

    *x = dx_in_pixel;
    *y = dy_in_pixel;

    // rt_kprintf("[mouse]dx:%f, dy:%f\n", dx_in_pixel, dy_in_pixel);

    float abs_total_x = OFFSET_Y + 0.7f + sigmoid(SCALE_FACTOR * (fabs(*x) - OFFSET_X));
    float abs_total_y = OFFSET_Y + 0.5f + sigmoid(SCALE_FACTOR * (fabs(*y) - OFFSET_X));

    float scaled_x = *x * abs_total_x;
    float scaled_y = *y * abs_total_y;
    // rt_kprintf("[mouse]scaled_x:%f, scaled_y:%f\n", scaled_x, scaled_y);
    // 限制範圍在 int8_t 的範圍內
    if (scaled_x > 127.0f)
    {
        scaled_x = 127.0f;
    }
    else if (scaled_x < -128.0f)
    {
        scaled_x = -128.0f;
    }
    if (scaled_y > 127.0f)
    {
        scaled_y = 127.0f;
    }
    else if (scaled_y < -128.0f)
    {
        scaled_y = -128.0f;
    }

    movement.x = (int8_t)scaled_x;
    movement.y = (int8_t)scaled_y;
    *x = 0.0f;
    *y = 0.0f;

    // rt_kprintf("movement x:%d, y:%d\n", movement.x, movement.y);
    return movement;
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/