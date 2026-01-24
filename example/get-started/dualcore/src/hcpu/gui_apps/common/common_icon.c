/**
 ******************************************************************************
 * @file   common_icon.c
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
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"

/*
 * @brief Create a ripple effect
 */
static lv_obj_t *create_ripple(app_speech_ripple_t *ripple, uint8_t init_height, lv_obj_t *par, lv_obj_t *ref_obj, uint8_t index)
{
    /* Create a new object for the ripple */
    ripple->widget = lv_obj_create(par);
    lv_obj_set_style_bg_color(ripple->widget, lv_color_hex(0x66FFE6), 0);
    lv_obj_set_style_bg_grad_color(ripple->widget, lv_color_hex(0x6699FF), 0);
    lv_obj_set_style_bg_grad_dir(ripple->widget, LV_GRAD_DIR_VER, 0);
    lv_obj_set_size(ripple->widget, 8, init_height);
    if (ref_obj == NULL)
    {
        lv_obj_align(ripple->widget, LV_ALIGN_LEFT_MID, 8, 0);
    }
    else
    {
        lv_obj_align(ripple->widget, LV_ALIGN_LEFT_MID, 8 + 11 * index, 0);
    }

    /* Create a new animation */
    lv_anim_init(&ripple->anim);
    lv_anim_set_var(&ripple->anim, ripple->widget);                            // Set the object to animate
    lv_anim_set_exec_cb(&ripple->anim, (lv_anim_exec_xcb_t)lv_obj_set_height); // Set the property to animate (changed from lv_obj_set_width)
    lv_anim_set_values(&ripple->anim, 12, 24);                                 // Set the start and end values
    lv_anim_set_time(&ripple->anim, 1000);                                     // Set the duration
    lv_anim_set_playback_time(&ripple->anim, 500);                             // Set the playback duration
    lv_anim_set_repeat_count(&ripple->anim, LV_ANIM_REPEAT_INFINITE);          // Set the repeat count

    return ripple->widget;
}

lv_obj_t *create_animate_ripples(app_speech_ripple_t *ripple, lv_obj_t *par, uint8_t height)
{
    lv_obj_t *ripple1 = create_ripple(&ripple[0], height, par, NULL, 0);
    lv_obj_t *ripple2 = create_ripple(&ripple[1], height * 1.5, par, ripple1, 1);
    lv_obj_t *ripple3 = create_ripple(&ripple[2], height, par, ripple2, 2);
    lv_obj_t *ripple4 = create_ripple(&ripple[3], height * 1.2, par, ripple3, 3);
    return ripple4;
}

void trigger_voice_ripple(app_speech_ripple_t *ripple, bool start)
{
    if (start)
    {
        for (int i = 0; i < 4; i++)
        {
            lv_anim_set_delay(&ripple[i].anim, i * 100); // Set a delay before starting each animation
            lv_anim_start(&ripple[i].anim);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            lv_anim_del(ripple[i].widget, (lv_anim_exec_xcb_t)lv_obj_set_height);
        }
    }
    rt_kprintf("trigger_voice_ripple %d\n", start);
}

static lv_obj_t *create_point(app_speech_ripple_t *point, lv_coord_t init_y, lv_obj_t *par, lv_obj_t *ref_obj, uint8_t index)
{
    /* Create a new object for the ripple */
    point->widget = lv_obj_create(par);
    lv_obj_set_style_bg_color(point->widget, lv_color_hex(0x66FFE6), 0);
    lv_obj_set_style_bg_grad_color(point->widget, lv_color_hex(0x6699FF), 0);
    lv_obj_set_style_bg_grad_dir(point->widget, LV_GRAD_DIR_VER, 0);
    lv_obj_set_size(point->widget, 10, 10);
    if (ref_obj == NULL)
    {
        lv_obj_align(point->widget, LV_ALIGN_LEFT_MID, 10, 0);
    }
    else
    {
        lv_obj_align(point->widget, LV_ALIGN_LEFT_MID, 10 + 13 * index, 0);
    }

    /* Create a new animation */
    lv_anim_init(&point->anim);
    lv_anim_set_var(&point->anim, point->widget);                        // Set the object to animate
    lv_anim_set_exec_cb(&point->anim, (lv_anim_exec_xcb_t)lv_obj_set_y); // Set the property to animate (changed from lv_obj_set_height)
    lv_anim_set_values(&point->anim, init_y, init_y + 10);               // Set the start and end values
    lv_anim_set_time(&point->anim, 500);                                 // Set the duration
    lv_anim_set_playback_time(&point->anim, 250);                        // Set the playback duration
    lv_anim_set_repeat_count(&point->anim, LV_ANIM_REPEAT_INFINITE);     // Set the repeat count

    return point->widget;
}

lv_obj_t *create_animate_points(app_speech_ripple_t *point, lv_obj_t *par)
{
    lv_obj_t *point1 = create_point(&point[0], 0, par, NULL, 0);
    lv_obj_t *point2 = create_point(&point[1], 0, par, point1, 1);
    lv_obj_t *point3 = create_point(&point[2], 0, par, point2, 2);
    return point3;
}

void wait_for_message(app_speech_ripple_t *point, bool start)
{
    if (start)
    {
        for (int i = 0; i < 3; i++)
        {
            lv_obj_set_y(point[i].widget, 0);
            lv_anim_set_delay(&point[i].anim, i * 150); // Set a delay before starting each animation
            lv_anim_start(&point[i].anim);
        }
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            lv_anim_del(point[i].widget, (lv_anim_exec_xcb_t)lv_obj_set_height);
        }
    }
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/