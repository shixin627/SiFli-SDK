/**
 ******************************************************************************
 * @file   bloc_motion_tracking.h
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

#ifndef __BLOC_MOTION_TRACKING_H__
#define __BLOC_MOTION_TRACKING_H__

#ifdef __cplusplus
extern "C"
{
#endif
    typedef enum
    {
        GRAVITY_POSITION_OTHER = 0,
        GRAVITY_POSITION_SIDE,
        GRAVITY_POSITION_HORIZONTAL,
        GRAVITY_POSITION_VERTICAL,
        GRAVITY_POSITION_AI
    } gravity_position_t;

#define MOTION_TRACKING_PARAMETER 2000
    extern bool has_user_started_controlling_with_arm(void);
    extern int get_gravity_position(void);
    extern void set_free_control_with_arm(bool flag);
    extern void set_open_control_options(bool open);
    extern void set_paused_control_with_arm(bool paused);
    extern void reset_ai_open_mic(void);
    extern void set_ai_open_mic(bool is_open);
    extern void reset_control_pos(void);
    extern void set_control_gravity_x_range(float start, float end, bool is_circular);
    extern void set_media_control_threshold(uint16_t threshold_value);
    extern void reset_total_yaw_energy(void);
    extern void set_scroll_segment_count(uint8_t count);
    extern float get_total_moving_distance(void);
    extern void list_auto_positioning(void);
    extern void set_prev_sensor_quat(uint16_t target_value);

    typedef struct
    {
        void (*ai_interface_lock)(void);
        void (*ai_interface_unlock)(void);
    } MotionTracking;
#ifdef __cplusplus
}
#endif

#endif //__BLOC_MOTION_TRACKING_H__

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/