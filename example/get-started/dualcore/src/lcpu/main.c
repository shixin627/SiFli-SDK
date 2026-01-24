/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "board.h"
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
    #include "drv_gpio.h"
#endif /* BSP_USING_PM */
#include "button.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "drivers/rt_drv_pwm.h"
#include "drv_rgbled.h"
#include "bloc_rgb_led.h"

#ifdef BSP_KEY1_ACTIVE_HIGH
    #define BUTTON_ACTIVE_POL BUTTON_ACTIVE_HIGH
#else
    #define BUTTON_ACTIVE_POL BUTTON_ACTIVE_LOW
#endif

static rt_timer_t rc10k_time_handle;
static int32_t key1_button_handle;


void main_send_read_charge_status_event(void)
{
    // if (main_event)
    // {
    //     rt_event_send(main_event, MAIN_EVENT_BATTERY_CHARGING);
    // }
}

void main_send_read_voltage_event(void)
{
    // if (main_event)
    // {
    //     rt_event_send(main_event, MAIN_EVENT_BATTERY_VOLTAGE);
    // }
}

void main_send_rgb_start_event(void)
{
    // if (main_event)
    // {
    //     rt_event_send(main_event, MAIN_EVENT_RGB_START);
    // }
}

void main_send_rgb_stop_event(void)
{
    // if (main_event)
    // {
    //     rt_event_send(main_event, MAIN_EVENT_RGB_STOP);
    // }
}

void main_send_hand_lift_event(void)
{
    // if (main_event)
    // {
    //     rt_event_send(main_event, MAIN_EVENT_HAND_LIFT);
    // }
}

void button_event_handler(int32_t pin, button_action_t button_action)
{
    rt_kprintf("pin: %d, action: %d\n", pin, button_action);
}

static void init_pin(void)
{
#if (BSP_KEY1_PIN >= GPIO1_PIN_NUM)
    button_cfg_t cfg;
#if defined(BSP_USING_PM)
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;
#endif /* BSP_USING_PM */

    cfg.pin = BSP_KEY1_PIN;
    cfg.active_state = BUTTON_ACTIVE_POL;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));
    key1_button_handle = id;

#if defined(BSP_USING_PM)
    gpio = GET_GPIO_INSTANCE(BSP_KEY1_PIN);
    gpio_pin = GET_GPIOx_PIN(BSP_KEY1_PIN);

    wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    RT_ASSERT(wakeup_pin >= 0);

    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_DOUBLE_EDGE);
#endif /* BSP_USING_PM */

#endif /* BSP_KEY1_PIN < GPIO1_PIN_NUM */
}

void rc10k_timeout_handler(void *parameter)
{
    if (HAL_LXT_DISABLED())
    {
        HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
    }
    else
    {
        rt_timer_stop(rc10k_time_handle);
    }
}

#define STRESS_STACK_SIZE       1024
#define MATH_THREAD_COUNT       3
#define MATH_THREAD_PRIO_BASE   21

static float simple_sin(int degrees)
{
    float radians = degrees * 3.14159f / 180.0f;
    float x = radians;
    float x2 = x * x;
    float x3 = x2 * x;
    float x5 = x3 * x2;
    return x - (x3 / 6.0f) + (x5 / 120.0f);
}

/*
 * Math load thread: continuous floating-point and integer operations
 * to stress CPU while LED animation runs independently.
 */
static void math_load_entry(void *param)
{
    uint32_t id = (uint32_t)(rt_uint32_t)param;
    volatile float result = 0.0f;
    volatile uint32_t count = 0;

    rt_kprintf("[MATH] Thread%u started\n", id);

    while (1) {
        /* Floating-point workload */
        for (int i = 1; i < 500; i++) {
            result += simple_sin(i % 360);
            result *= 1.0001f;
        }
        /* Integer workload */
        volatile uint32_t val = 0x12345678;
        for (int i = 0; i < 1000; i++) {
            val = val * 1103515245 + 12345;
            val ^= (val >> 16);
        }
        count++;
        if ((count % 1000) == 0) {
            rt_kprintf("[MATH] Thread%u: loops=%u result=%.2f\n", id, count, result);
        }
        rt_thread_mdelay(1);
    }
}

int main(void)
{
    // init_pin();
    if (HAL_LXT_DISABLED())
    {
        rc10k_time_handle  = rt_timer_create("rc10", rc10k_timeout_handler,  NULL,
                                             rt_tick_from_millisecond(15 * 1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
        RT_ASSERT(rc10k_time_handle);
        rt_timer_start(rc10k_time_handle);
    }

    rt_thread_t tid;

    rt_kprintf("\n========================================\n");
    rt_kprintf("  RGB LED Stress Test (CPU Load)\n");
    rt_kprintf("  1 LED task + %d math tasks\n", MATH_THREAD_COUNT);
    rt_kprintf("========================================\n\n");

    rgb_led_init();

    /* Create math load threads (priority 21~30) */
    // for (int i = 0; i < MATH_THREAD_COUNT; i++) {
    //     char name[8];
    //     rt_snprintf(name, sizeof(name), "math%d", i);
    //     tid = rt_thread_create(name, math_load_entry, (void *)(rt_uint32_t)i,
    //                            STRESS_STACK_SIZE, MATH_THREAD_PRIO_BASE + i, 10);
    //     if (tid) rt_thread_startup(tid);
    //     else rt_kprintf("[STRESS] Failed to create %s\n", name);
    // }

    /* LED fade animation runs in main thread context */
    rgb_led_params_t params;
    params.red = 0x00;
    params.green = 0xff;
    params.blue = 0x00;
    params.brightness = 100;
    params.mode = RGB_ANIM_FADE;
    params.period = 1000;
    params.repeat_times = 0; /* infinite */

    rt_kprintf("[LED] Green fade started\n");
    rgb_led_animate(&params);

    while (1)
    {
        rt_thread_mdelay(3000);
    }
    return RT_EOK;
}

