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
#include "bloc_battery.h"
#ifdef BSP_KEY1_ACTIVE_HIGH
    #define BUTTON_ACTIVE_POL BUTTON_ACTIVE_HIGH
#else
    #define BUTTON_ACTIVE_POL BUTTON_ACTIVE_LOW
#endif

static rt_timer_t rc10k_time_handle;
static int32_t key1_button_handle;
static rt_event_t main_event;

#define MAIN_EVENT_BATTERY_CHARGING (1 << 0)
#define MAIN_EVENT_BATTERY_VOLTAGE (1 << 1)
#define MAIN_EVENT_RGB_START (1 << 2)
#define MAIN_EVENT_RGB_STOP (1 << 3)
#define MAIN_EVENT_HAND_LIFT (1 << 4)
#define MAIN_EVENT_ALL                                                         \
    (MAIN_EVENT_BATTERY_CHARGING | MAIN_EVENT_BATTERY_VOLTAGE |                \
     MAIN_EVENT_RGB_START | MAIN_EVENT_RGB_STOP | MAIN_EVENT_HAND_LIFT)

void main_send_read_charge_status_event(void)
{
    if (main_event)
    {
        rt_event_send(main_event, MAIN_EVENT_BATTERY_CHARGING);
    }
}

void main_send_read_voltage_event(void)
{
    if (main_event)
    {
        rt_event_send(main_event, MAIN_EVENT_BATTERY_VOLTAGE);
    }
}

void main_send_rgb_start_event(void)
{
    if (main_event)
    {
        rt_event_send(main_event, MAIN_EVENT_RGB_START);
    }
}

void main_send_rgb_stop_event(void)
{
    if (main_event)
    {
        rt_event_send(main_event, MAIN_EVENT_RGB_STOP);
    }
}

void main_send_hand_lift_event(void)
{
    if (main_event)
    {
        rt_event_send(main_event, MAIN_EVENT_HAND_LIFT);
    }
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

#ifdef RT_USING_WDT
/**
 * @brief This function is invoked in WDT_IRQHandler.
 *        It can be overidden to do some work when WDT1 timeout occured.
 *        Ex. to store exception context and reboot immediately.
 */
void wdt_store_exception_information(void)
{
    rt_kprintf("LCPH WDT1 timeout occurs.\n");
    extern void drv_reboot(void);
    drv_reboot();
    return;
}

/**
 * @brief WDT ON/OFF
 * @param en 0: OFF 1: ON
 */
static void watchdog_set_status(uint8_t en)
{
    #ifdef RT_USING_WDT
    /* Set wdt status 0. */
    rt_hw_watchdog_set_status(en);
    /* Avoid repeat set hook. */
    rt_hw_watchdog_hook(0);
    if (!en)
    {
        /* Stop wdt. */
        rt_hw_watchdog_deinit();
    }
    else
    {
        /* Set hook for watchdog petting. */
        rt_hw_watchdog_hook(1);
    }
    #endif
}
#endif

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

#ifdef RT_USING_WDT
    /* Diable WDT. */
    watchdog_set_status(0);
    rt_kprintf("LCPH WDT off.\n");
    /* Enable WDT. */
    watchdog_set_status(1);
    rt_kprintf("LCPH WDT on.(timeout: %d seconds)\n", WDT_TIMEOUT);
#endif /* RT_USING_WDT */

#if defined(RGB_SK6812MINI_HS_ENABLE)
    // 初始化 RGB LED
    rgb_led_init();
#endif

    // 創建事件對象
    main_event = rt_event_create("main_evt", RT_IPC_FLAG_FIFO);
    RT_ASSERT(main_event != RT_NULL);
    // battery_get_charge_state()->charge_percent = 100;
    rt_uint32_t recv_set = 0;
    while (1)
    {
        // rt_err_t result = rt_event_recv(main_event, MAIN_EVENT_ALL,
        //                                 RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
        //                                 RT_WAITING_FOREVER, &recv_set);
        rt_err_t result = rt_event_recv(main_event, MAIN_EVENT_ALL,
                                       RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                       rt_tick_from_millisecond(30), &recv_set);

        if (result == RT_EOK)
        {
#ifdef CHARGE_DETECT_PIN
            if (recv_set & MAIN_EVENT_BATTERY_CHARGING)
            {
                bloc_battery_handle_charging_event();
            }
#endif

            if (recv_set & MAIN_EVENT_BATTERY_VOLTAGE)
            {
                bloc_battery_handle_voltage_event();
            }

#if defined(RGB_SK6812MINI_HS_ENABLE)
            if (recv_set & MAIN_EVENT_RGB_START)
            {
                bloc_rgb_led_handle_start_event();
            }

            if (recv_set & MAIN_EVENT_RGB_STOP)
            {
                bloc_rgb_led_handle_stop_event();
            }
#endif

            if (recv_set & MAIN_EVENT_HAND_LIFT)
            {
                extern void hand_tracking_lift_callback(uint8_t lift);
                hand_tracking_lift_callback(0);
            }
        }
        else
        {
            if (battery_get_charge_state()->is_plugged && battery_get_charge_state()->charge_percent > 10)
            {
                rgb_fade_cycle_base_on_battery_level(battery_get_charge_state()->charge_percent);
            }
        }
    }
    return RT_EOK;
}

