/**
 ******************************************************************************
 * @file   hand_tracking.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "hand_tracking.h"
#include "gesture_handler.h"
#include "watch_global_data.h"
#include "bloc_peripheral.h"
#define LOG_TAG "hand.tracking"
#define DBG_LVL DBG_INFO
#include <drv_log.h>
//-------------------------------------------------------------------------------------------
// Definitions
#define GESTURE_COOLDOWN_TIME 300
#define USING_PUT_DOWN_TIMER 1

#if USING_PUT_DOWN_TIMER
    // 在手往身體反方向轉的情況下，如果在800ms內沒有再次轉回，則視為放下手腕
    #define PUT_DOWN_CONFIRM_TIME 800
    #define PUT_DOWN_CONFIRM_TIME_MS                                           \
        rt_tick_from_millisecond(PUT_DOWN_CONFIRM_TIME)
#endif

#define LIFT_DETECT_GYRO_ON_WAKEUP_x 500
#define LIFT_DETECT_GYRO_ON_SLEEP_x 180

#define THRESHOLD_PARAM_ON_SLEEP 450
#define THRESHOLD_PARAM_ON_WAKEUP 2400
#define WRIST_PRONATION_THRESHOLD 700 // 800
#define GESTURE_BACK_DURATION 500     // 500ms

static float max_pronation_gyro_x = 0;
static float max_supination_gyro_x = 0;
static float last_max_pronation_gyro_x = 0;
static float last_max_supination_gyro_x = 0;
static float threshold = 0;
static uint8_t count = 0;
static uint32_t put_down_time = 0;
static bool hand_status = false; // true: lifting, false: putdown
static rt_tick_t last_hand_lift_time = 0;
static rt_tick_t last_hand_lift_y_time = 0;
static bool gesture_threshold_achieved = false;
static rt_tick_t gesture_threshold_achieved_time = 0;
static float max_supination_gyro_x_abs = 0;
static float max_pronation_gyro_x_abs = 0;
static float wrist_rotate_ratio = 0;

static uint8_t zero_velocity_buffer[25];
static uint8_t zero_velocity_buffer_index = 0;
bool check_zero_velocity_event(void)
{
    for (uint8_t i = 0; i < 25; i++)
    {
        if (zero_velocity_buffer[i])
        {
            return true;
        }
    }
    return false;
}

extern bool is_multi_gesture_mode(void);

static HandTrackingProvider hand_tracking;
void hand_tracking_init(void (*lift_callback)(uint8_t lift),
                        void (*back_callback)(void))
{
    hand_tracking.lift_callback = lift_callback;
    hand_tracking.back_callback = back_callback;
}

static bool is_hand_lifting(float gyro_x)
{
    return gyro_x > LIFT_DETECT_GYRO_ON_SLEEP_x;
}

static bool is_hand_put_down(float gyro_x)
{
    return gyro_x < -LIFT_DETECT_GYRO_ON_SLEEP_x;
}

static bool is_hand_wrist_pronation(float gyro_x)
{
    return gyro_x > 200;
}

static bool is_hand_unlock_wrist_rotation(float gyro_x)
{
    return gyro_x < 100 && gyro_x > -100;
}

static bool is_hand_wrist_supination(float gyro_x)
{
    return gyro_x < -200;
}

static float sum_gyro_put_down_x = 0;
static float sum_gyro_hand_lifting_x = 0;
static float sum_gyro_hand_lifting_y = 0;

#if USING_PUT_DOWN_TIMER
static rt_timer_t timer_put_down_confirm;
static rt_tick_t last_wrist_pronation_time = 0;
static rt_tick_t last_lift2_time = 0;
static rt_tick_t last_gesture_event_time = 0;
static bool put_down_started = false;
extern void main_send_hand_lift_event(void);
static void timer_put_down_confirm_callback(void *param)
{
    put_down_started = false;
    if (!hand_tracking.lift_callback)
        return;
    if (last_gesture_event_time + GESTURE_COOLDOWN_TIME < rt_tick_get())
        main_send_hand_lift_event();
    // hand_tracking.lift_callback(0);
}

void hand_tracking_lift_callback(uint8_t lift)
{
    if (!hand_tracking.lift_callback)
        return;
    hand_tracking.lift_callback(lift);
    LOG_I("Hand lift callback triggered with lift: %d", lift);
}

static void trigger_lift_event(void)
{
    if (!hand_tracking.lift_callback)
        return;
    static rt_tick_t last_time = 0;
    if (last_time + GESTURE_COOLDOWN_TIME < rt_tick_get())
    {
        last_time = rt_tick_get();
        hand_tracking.lift_callback(1);
    }
}

static void trigger_lift2_event(void)
{
    if (!hand_tracking.lift_callback)
        return;
    if (last_lift2_time + GESTURE_COOLDOWN_TIME < rt_tick_get())
    {
        last_lift2_time = rt_tick_get();
        hand_tracking.lift_callback(2);
    }
}

static void trigger_lift3_event(void)
{
    if (!hand_tracking.lift_callback)
        return;
    if (last_gesture_event_time + GESTURE_COOLDOWN_TIME < rt_tick_get())
    {
        if (check_zero_velocity_event())
        {
            last_gesture_event_time = rt_tick_get();
            hand_tracking.lift_callback(3);
        }
        else
        {
            // LOG_W("Zero velocity event not detected, lift3 event not
            // triggered");
        }
    }
}

static void trigger_back_event(void)
{
    if (!hand_tracking.back_callback)
        return;
    if (last_gesture_event_time + GESTURE_COOLDOWN_TIME < rt_tick_get())
    {
        if (check_zero_velocity_event())
        {
            last_gesture_event_time = rt_tick_get();
            hand_tracking.back_callback();
        }
    }
}

void back_event(void)
{
    hand_tracking.back_callback();
}

static uint8_t state = 0;
static bool gyro_y_check_watch = false;
static void timer_check_watch_callback(void *param)
{
    state = 0;
    sum_gyro_put_down_x = 0;
    gyro_y_check_watch = true;
    gesture_threshold_achieved = false;
    // LOG_I("Check watch timer triggered, state reset to 0");
}

static rt_timer_t timer_check_watch;
static void start_check_watch_timer(void)
{

    if (!timer_check_watch)
    {
        timer_check_watch =
            rt_timer_create("timer_check_watch", timer_check_watch_callback,
                            RT_NULL, 200, RT_TIMER_FLAG_ONE_SHOT);
    }
    rt_timer_start(timer_check_watch);
}

static void stop_check_watch_timer(void)
{
    if (timer_check_watch)
    {
        rt_timer_stop(timer_check_watch);
    }
}

static void start_put_down_confirm_timer(void)
{
    if (!timer_put_down_confirm)
    {
        timer_put_down_confirm = rt_timer_create(
            "timer_put_down_confirm", timer_put_down_confirm_callback, RT_NULL,
            PUT_DOWN_CONFIRM_TIME_MS, RT_TIMER_FLAG_ONE_SHOT);
        if (!timer_put_down_confirm)
        {
            // Handle timer creation failure
            LOG_E("Failed to create put down confirm timer");
            return;
        }
    }
    if (!put_down_started)
    {
        rt_timer_start(timer_put_down_confirm);
        put_down_started = true;
    }
}

static void stop_put_down_confirm_timer(void)
{
    if (timer_put_down_confirm && put_down_started)
    {
        rt_timer_stop(timer_put_down_confirm);
        put_down_started = false;
    }
    // LOG_D("sum_gyro_x: %d", sum_gyro_x);
    sum_gyro_put_down_x = 0;
    sum_gyro_hand_lifting_x = 0;
}
#endif

void hand_tracking_data_update(float freq, float gyro_x, float gyro_y,
                               bool open_wrist_rotation,
                               bool if_watchface_visible, bool zero_velocity)
{
    static rt_tick_t state_enter_time = 0;
    rt_tick_t current_time = rt_tick_get();
    threshold = THRESHOLD_PARAM_ON_SLEEP * (freq / 25);
    zero_velocity_buffer[zero_velocity_buffer_index] = zero_velocity;
    zero_velocity_buffer_index++;
    if (zero_velocity_buffer_index >= 25)
    {
        zero_velocity_buffer_index = 0;
    }

    switch (state)
    {
    case 0:
    {
        if (is_hand_lifting(gyro_x))
        {
            sum_gyro_put_down_x += gyro_x;
            sum_gyro_hand_lifting_x = 0;
            if (open_wrist_rotation)
            {
                if (gyro_x > max_pronation_gyro_x)
                {
                    max_pronation_gyro_x = gyro_x;
                }
                max_supination_gyro_x = 0;
            }

            // if (sum_gyro_put_down_x > (threshold * 0.7))
            // {
            //     trigger_lift_event();
            // }

            if (gesture_threshold_achieved)
            {
                if (current_time - gesture_threshold_achieved_time < 50)
                {
                    max_pronation_gyro_x_abs = fabsf(max_pronation_gyro_x);
                    wrist_rotate_ratio =
                        max_pronation_gyro_x_abs / max_supination_gyro_x_abs;
                    // LOG_I("max_supination_gx:%f, max_pronation_gx:%f,
                    // ratio:%f(threshold:%f)", last_max_supination_gyro_x,
                    // max_pronation_gyro_x, ratio, ratio_threshold);
                    if (max_pronation_gyro_x_abs > WRIST_PRONATION_THRESHOLD &&
                        wrist_rotate_ratio > 1.5)
                    {
                        trigger_lift3_event();
#if USING_PUT_DOWN_TIMER
                        stop_put_down_confirm_timer();
#endif
                        stop_check_watch_timer();
                        last_max_supination_gyro_x = 0;
                        max_pronation_gyro_x = 0;
                        gesture_threshold_achieved = false;
                        sum_gyro_put_down_x = 0;
                        // LOG_I("Trigger lift3 event,
                        // max_pronation_gyro_x:%0.3f, ratio:%0.3f",
                        // max_pronation_gyro_x, ratio);
                    }
                    else
                    {
                        // LOG_I("max_pronation_gyro_x:%0.3f, ratio:%0.3f",
                        // max_pronation_gyro_x, ratio);
                        if (put_down_time + GESTURE_BACK_DURATION >
                                current_time &&
                            max_supination_gyro_x_abs > 400 &&
                            sum_gyro_put_down_x > threshold * 1.3)
                        {
                            // LOG_I("Trigger back event max_supination:%f,sum_gyro:%f", max_supination_gyro_x_abs, sum_gyro_put_down_x);
                            trigger_back_event();
#if USING_PUT_DOWN_TIMER
                            stop_put_down_confirm_timer();
#endif
                            stop_check_watch_timer();
                            last_max_supination_gyro_x = 0;
                            max_pronation_gyro_x = 0;
                            state = 3;
                            state_enter_time = current_time;
                            gesture_threshold_achieved = false;
                            sum_gyro_put_down_x = 0;
                        }
                    }
                }
                else
                {
                    gesture_threshold_achieved = false;
                    state = 1;
                }
            }
            else
            {
                if (sum_gyro_put_down_x > threshold)
                {
                    start_check_watch_timer();
                    // state = 1;
                    gesture_threshold_achieved = true;
                    gesture_threshold_achieved_time = current_time;
                    max_supination_gyro_x_abs =
                        fabsf(last_max_supination_gyro_x);
                    max_pronation_gyro_x_abs = fabsf(max_pronation_gyro_x);
                    wrist_rotate_ratio =
                        max_pronation_gyro_x_abs / max_supination_gyro_x_abs;
                    // LOG_I("Hand lifting detected, state changed to 1:%0.3f",
                    // max_pronation_gyro_x);
                }
            }
        }
        else if (is_hand_put_down(gyro_x))
        {
            sum_gyro_hand_lifting_x += gyro_x;
            sum_gyro_put_down_x = 0;
            if (gyro_x < max_supination_gyro_x)
            {
                max_supination_gyro_x = gyro_x;
            }
            max_pronation_gyro_x = 0;
            if (sum_gyro_hand_lifting_x < -(threshold))
            {
                last_max_supination_gyro_x = max_supination_gyro_x;
                state = 2;
                put_down_time = current_time;
            }
        }
        else
        {
            count++;
            if (count >= freq / 2)
            {
                sum_gyro_put_down_x = 0;
                sum_gyro_hand_lifting_x = 0;
            }
        }
        break;
    }
    case 1:
    {
#if USING_PUT_DOWN_TIMER
        stop_put_down_confirm_timer();
#endif
        if (if_watchface_visible)
        {
            // LOG_I("Hand lifting detected, lift2");
            stop_check_watch_timer();
            if (!hand_status ||
                (current_time - last_hand_lift_time) > RT_TICK_PER_SECOND)
            {
                hand_status = true;
                trigger_lift2_event();
                last_hand_lift_time = current_time;
            }
            // else
            // {
            //     LOG_I("Hand lifting detected, hand_status: %d,time:%d",
            //     hand_status,current_time - last_hand_lift_time);
            // }
            sum_gyro_put_down_x = 0;
            state = 0;
        }
        break;
    }
    case 2:
    {
        if (hand_status)
        {
            hand_status = false;
        }
        sum_gyro_hand_lifting_x = 0;
        state = 0;
#if USING_PUT_DOWN_TIMER
        start_put_down_confirm_timer();
#endif
        break;
    }
    case 3:
    {
        if (current_time - state_enter_time > 200)
        {
            state = 0;
        }
        break;
    }
    default:
    {
        state = 0;
        break;
    }
    }

    if (gyro_y_check_watch)
    {
        rt_tick_t gyro_y_current_time = rt_tick_get();
        if (gyro_y < -30)
        {
            sum_gyro_hand_lifting_y += gyro_y;
            // LOG_D("Lifting y... sum_gyro_y: %0.3f", sum_gyro_hand_lifting_y);
            // if (sum_gyro_hand_lifting_y < -500)
            // {
            //     trigger_lift_event();
            // }
            if (sum_gyro_hand_lifting_y < -600)
            {
                if (hand_tracking.lift_callback)
                {
                    start_check_watch_timer();
                    gyro_y_check_watch = false;
                }
                sum_gyro_hand_lifting_y = 0;
            }
            last_hand_lift_y_time = gyro_y_current_time;
        }
        else if (gyro_y > 30)
        {
            sum_gyro_hand_lifting_y += gyro_y;
        }
        if (gyro_y_current_time - last_hand_lift_y_time > 500)
        {
            sum_gyro_hand_lifting_y = 0;
        }
    }
    else
    {
        if (if_watchface_visible)
        {
            stop_check_watch_timer();
            trigger_lift2_event();
            last_hand_lift_time = current_time;
            gyro_y_check_watch = true;
        }
    }
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/