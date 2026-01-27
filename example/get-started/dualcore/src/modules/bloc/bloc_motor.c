/**
 ******************************************************************************
 * @file   bloc_motor.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
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
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <math.h>
#include "bloc_peripheral.h"
#include "bloc_control.h"
#include "bloc_motor.h"
#include "bloc_setting.h"

#define DBG_TAG "bloc.motor"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define MOTOR_FREQ 300
#ifndef RT_PI
    #define RT_PI 3.14159265358979323846f
#endif

#ifdef MOTOR_ENABLED
    #include "drv_motor.h"
    #include "drivers/motor_device.h"

static const bool enable_motor = true;

    #define SMOOTH_VIBRATION

// 馬達任務相關定義
    #define MOTOR_TASK_PRIORITY 11
    #define MOTOR_TASK_STACK_SIZE 1024
    #define MOTOR_TASK_TIMESLICE 5

// 馬達事件定義
    #define MOTOR_EVENT_START (1 << 0)
    #define MOTOR_EVENT_STOP (1 << 1)

// 馬達任務相關變量
static rt_thread_t motor_task = RT_NULL;
static rt_event_t motor_event = RT_NULL;
static motor_params_t g_motor_params = {0};

static rt_device_t motor_device = RT_NULL;
static void bloc_motor_vibrate(motor_params_t *params);
static void bloc_motor_stop(void);

static rt_tick_t last_vibrate_tick = 0;
static bool motor_stop_flag = true;
static uint8_t motor_control_io_pin_state = 0;
static uint8_t motor_power_io_pin_state = 0;

static bool _motor_power_on = false;

static void set_motor_power(bool on)
{
    if (_motor_power_on != on)
    {
        _motor_power_on = on;
        // LOG_D("Motor power %s", on ? "ON" : "OFF");
    }
}

// 發送馬達停止事件
static void send_motor_stop_event(void)
{
    if (motor_event)
    {
        rt_event_send(motor_event, MOTOR_EVENT_STOP);
    }
}

    // #define ENABLE_AUTO_OFF_MOTOR
    #ifdef ENABLE_AUTO_OFF_MOTOR
static rt_timer_t auto_off_timer = RT_NULL;
static void auto_stop_motor(void)
{
    if (!auto_off_timer)
    {
        // 啟動一個1秒的timer自動關閉馬達
        auto_off_timer = rt_timer_create(
            "motor_off", (void (*)(void *))send_motor_stop_event, NULL,
            RT_TICK_PER_SECOND, RT_TIMER_FLAG_ONE_SHOT);
    }
    if (auto_off_timer)
    {
        rt_timer_start(auto_off_timer);
    }
}
    #endif

static void bloc_motor_vibrate(motor_params_t *params)
{
    if (!enable_motor)
    {
        LOG_W("[%s]Motor is disabled in settings.", __func__);
        return;
    }
    LOG_D("[%s] params->period:%d,duty_cycle:%d,repeat_times:%d", __func__,
          params->period, params->duty_cycle, params->repeat_times);
    if (params->duty_cycle > 100)
    {
        params->duty_cycle = 100;
    }

    #ifdef ENABLE_AUTO_OFF_MOTOR
    if (auto_off_timer)
    {
        rt_timer_stop(auto_off_timer);
    }
    #endif

    if (motor_device == NULL)
    {
        motor_device = rt_device_find(MOTOR_DEVICE_NAME);
        if (!motor_device)
        {
            LOG_E("can't find %s device!", MOTOR_DEVICE_NAME);
            return;
        }
        if (rt_device_open(motor_device, RT_DEVICE_FLAG_RDWR) != RT_EOK)
        {
            motor_device = NULL;
            return;
        }
    }

    rt_device_control(motor_device, MOTOR_CONTROL_OPEN_DEVICE, NULL);
    set_motor_power(true);
    motor_stop_flag = false;

    #ifdef MOTOR_USE_PWM
    uint32_t duration_on, duration_off = 0;

    duration_on = params->period / 2;
    if (params->repeat_times > 1)
    {
        duration_off = params->period / 2;
    }

    int i = 0;
        #ifdef SMOOTH_VIBRATION
    // 平滑震動: duty_cycle 由小到大再到小
    int base_duty = params->duty_cycle;
    int max_duty = 100;
    int ramp_time = duration_on / 2; // 上升/下降各佔一半時間

    // 動態決定步數與步長
    int step_count = (duration_on / 1000) / 2;
    if (step_count < 1)
        step_count = 1;
    int step = (max_duty - base_duty) / step_count;
    if (step < 1)
        step = 1;

    int ramp_steps = (base_duty < max_duty) ? (max_duty - base_duty) / step : 0;
    int ramp_delay = (ramp_steps > 0) ? (ramp_time / ramp_steps) : ramp_time;
        #endif

    rt_uint32_t duty_cycle = 0;
    while (i < params->repeat_times)
    {
        last_vibrate_tick = rt_tick_get();

        #ifdef SMOOTH_VIBRATION
        if (base_duty < 100)
        {
            // 使用阻尼正弦波形 (damped sine wave) 來產生平滑震動
            int total_steps = ramp_steps * 2;
            if (total_steps < 2)
                total_steps = 2;
            float damping = 0.15f; // 阻尼係數,可調整震動衰減速度
            for (int s = 0; s < total_steps; s++)
            {
                float t = (float)s / (total_steps - 1); // 0~1
                // 阻尼正弦波: y = A * exp(-damping * t) * sin(π * t)
                float amplitude = (max_duty - base_duty) * expf(-damping * t);
                int d = base_duty + (int)(amplitude * sinf(2 * RT_PI * t));
                if (d > 100)
                    d = 100;
                if (d < 0)
                    d = 0;
                duty_cycle = d;
                rt_device_control(motor_device, MOTOR_CONTROL_PWM_OPEN,
                                  (void *)&duty_cycle);
                rt_thread_delay(ramp_delay / 1000);
            }
        }
        else
        #endif
        {
            // 100% duty 直接震動
            duty_cycle = base_duty; // MOTOR_PERIOD
            rt_device_control(motor_device, MOTOR_CONTROL_PWM_OPEN,
                              (void *)&duty_cycle);
            rt_thread_delay(duration_on / 1000);
        }

        duty_cycle = 0;
        rt_device_control(motor_device, MOTOR_CONTROL_PWM_OPEN,
                          (void *)&duty_cycle);
        if (duration_off > 0)
        {
            rt_thread_delay(duration_off / 1000);
        }
        if (motor_stop_flag)
        {
            break;
        }
        i++;
    }

        // rt_device_control(motor_device, MOTOR_CONTROL_PWM_CLOSE, NULL);
    #else

    duration_on = params->period / 2;
    if (params->repeat_times > 1)
    {
        duration_off = params->period / 2;
    }
    int i = 0;
    while (i < params->repeat_times)
    {
        if (motor_stop_flag)
        {
            break;
        }
        last_vibrate_tick = rt_tick_get();
        motor_control_io_pin_state = 1;
        rt_device_control(motor_device, MOTOR_CONTROL_CTRL_IO,
                          &motor_control_io_pin_state);
        rt_thread_delay(duration_on / 1000);
        motor_control_io_pin_state = 0;
        rt_device_control(motor_device, MOTOR_CONTROL_CTRL_IO,
                          &motor_control_io_pin_state);
        if (duration_off > 0)
        {
            rt_thread_delay(duration_off / 1000);
        }
        i++;
    }
    #endif // MOTOR_USE_PWM

    motor_stop_flag = true;
    #ifdef ENABLE_AUTO_OFF_MOTOR
    auto_stop_motor();
    #else
    send_motor_stop_event();
    #endif
}

static void bloc_motor_stop(void)
{
    if (motor_device)
    {
    #ifdef MOTOR_USE_PWM
        rt_device_control(motor_device, MOTOR_CONTROL_PWM_CLOSE, NULL);
    #else
        motor_control_io_pin_state = 0;
        rt_device_control(motor_device, MOTOR_CONTROL_CTRL_IO,
                          &motor_control_io_pin_state);
    #endif
        rt_device_control(motor_device, MOTOR_CONTROL_CLOSE_DEVICE, NULL);
        set_motor_power(false);
        rt_device_close(motor_device);
        motor_device = NULL;
    }
}

// 馬達任務入口函數
static void motor_task_entry(void *parameter)
{
    rt_uint32_t event;

    LOG_I("Motor task started");

    while (1)
    {
        // 等待事件
        if (rt_event_recv(motor_event, MOTOR_EVENT_START | MOTOR_EVENT_STOP,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &event) == RT_EOK)
        {
            if (event & MOTOR_EVENT_START)
            {
                LOG_D("Motor task received START event");
                bloc_motor_vibrate(&g_motor_params);
            }

            if (event & MOTOR_EVENT_STOP)
            {
                LOG_D("Motor task received STOP event");
                bloc_motor_stop();
            }
        }
    }
}

static int motor_task_init(void)
{
    // 創建事件對象
    motor_event = rt_event_create("motor_evt", RT_IPC_FLAG_FIFO);
    if (!motor_event)
    {
        LOG_E("Failed to create motor event");
        return -RT_ERROR;
    }

    // 創建馬達任務
    motor_task = rt_thread_create("motor_task", motor_task_entry, RT_NULL,
                                  MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY,
                                  MOTOR_TASK_TIMESLICE);
    if (!motor_task)
    {
        LOG_E("Failed to create motor task");
        rt_event_delete(motor_event);
        motor_event = RT_NULL;
        return -RT_ERROR;
    }

    // 啟動任務
    rt_thread_startup(motor_task);
    LOG_I("Motor task initialized successfully");

    return RT_EOK;
}
    #ifndef BSP_USING_PC_SIMULATOR
INIT_APP_EXPORT(motor_task_init);
    #endif

    #ifdef BSP_USING_PC_SIMULATOR
void motor_vibrate_start(motor_params_t *params)
{
    LOG_D("vibration task start");
}
void motor_vibrate_stop(void)
{
    LOG_D("vibration task stop");
}
    #else
void motor_vibrate_start(motor_params_t *params)
{
    if (!motor_event)
    {
        LOG_E("Motor event not initialized");
        return;
    }

    // 保存參數到全局變量
    g_motor_params = *params;

    // 發送啟動事件到馬達任務
    rt_event_send(motor_event, MOTOR_EVENT_START);
}

void motor_vibrate_stop(void)
{
    // 設置停止標誌
    motor_stop_flag = true;

    // 發送停止事件到馬達任務
    if (motor_event)
    {
        rt_event_send(motor_event, MOTOR_EVENT_STOP);
    }
}

    #endif //  #ifndef BSP_USING_PC_SIMULATOR

#endif /* MOTOR_ENABLED */

/////////// IMPLEMENTATION ///////////

static bool GetMotorStatus(void)
{
#if (CUSTOMER_BOARD_VER == BOARD_VER_13)
    return false;
#endif

#ifdef MOTOR_ENABLED
    if (motor_stop_flag && (rt_tick_get() - last_vibrate_tick > 250))
    {
        return false;
    }
    return true;
#else
    return false;
#endif
}
/// @brief Start the motor vibration
/// @param duty_cycle The duty cycle of the PWM signal
/// @param period_ms The period of the PWM signal
/// @param repeat_times The number of times to repeat the vibration
static void MotorVibrateStart(motor_params_t *vibration_params)
{
#if (CUSTOMER_BOARD_VER == BOARD_VER_13)
    return;
#endif

#ifdef MOTOR_ENABLED && (CUSTOMER_BOARD_VER != BOARD_VER_13)
    motor_vibrate_start(vibration_params);
#endif
}

/// @brief Stop the motor vibration
static void MotorVibrateStop(void)
{
#if (CUSTOMER_BOARD_VER == BOARD_VER_13)
    return;
#endif

#ifdef MOTOR_ENABLED && (CUSTOMER_BOARD_VER != BOARD_VER_13)
    motor_vibrate_stop();
#endif
}

// Combine function in provider object
bloc_motor_t motor_provider;

static int bloc_motor_register(void)
{
    motor_provider.get_motor_status = GetMotorStatus;
    motor_provider.start_motor = MotorVibrateStart;
    motor_provider.stop_motor = MotorVibrateStop;
    return 0;
}

INIT_APP_EXPORT(bloc_motor_register);

#if 0

/// @brief
/// @param argc
/// @param argv start [duty cycle] [period(us)] [repeat times]
/// @return

static int utest_motor_vibrate(int argc, char *argv[])
{
    if (strcmp(argv[1], "-start") ==
        0) // utest_motor_vibrate -start 75 500000 5
    {
        int duty_cycle = atoi(argv[2]);
        if (duty_cycle > 100)
        {
            duty_cycle = 100;
        }
        else if (duty_cycle < 0)
        {
            duty_cycle = 0;
        }
        int period = atoi(argv[3]);
        int repeat_times = atoi(argv[4]);
        LOG_D("duty_cycle: %d, period: %d, repeat_times: %d", duty_cycle,
              period, repeat_times);
        static motor_params_t params;
        params.duty_cycle = duty_cycle;
        params.period = period;
        params.repeat_times = repeat_times;
        motor_provider.start_motor(&params);
    }
    else if (strcmp(argv[1], "-stop") == 0) // utest_motor_vibrate -stop
    {
        LOG_D("stop motor");
        motor_provider.stop_motor();
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(utest_motor_vibrate, control motor vibrate);

static int utest_motor_pwm(int argc, char *argv[])
{
    if (argc != 2)
    {
        LOG_D("Usage: utest_motor_pwm <channel>");
        return -RT_ERROR;
    }
    bool is_comp = false;
    int channel = atoi(argv[1]);
    LOG_D("utest_motor_pwm channel:%d", channel);
    struct rt_device_pwm *device = RT_NULL;

    device = (struct rt_device_pwm *)rt_device_find("pwm4");
    if (!device)
    {
        return -RT_ERROR;
    }

    rt_pwm_set(device, channel, 500000000, 250000000);
    if (is_comp) // is pwm complementary
    {
        struct rt_pwm_break_dead bkd = {0}; // TODO: set the BKD.
        rt_pwm_set_brk_dead(device, (rt_uint32_t *)&bkd, 100);
        rt_pwm_enable2(device, channel, 1);
    }
    else
    {
        rt_pwm_enable(device, channel);
    }
    rt_thread_mdelay(1000);
    rt_pwm_disable(device, channel);
    return 0;
}
MSH_CMD_EXPORT(utest_motor_pwm, "utest_motor_pwm [OPTION] ...");
#endif

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
