/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include "button.h"
#include <board.h>

/*
    action:
    BUTTON_PRESSED  = 0,    Indicates that a button is pressed
    BUTTON_RELEASED = 1,    Indicates that a button is released
    BUTTON_LONG_PRESSED = 2, Indicates that a button is long released
    BUTTON_CLICKED  = 3,     Indicates that a button is clicked
*/
static void button_event_handler(int32_t pin, button_action_t action)
{
    rt_kprintf("button:%d,%d\n", pin, action);
}

int main(void)
{
    /*
    587 board PB pin needs to add 96
    54+96->150
    56+96->152
    */

    button_cfg_t cfg;

#ifdef BSP_USING_KEY1
    cfg.pin = BSP_KEY1_PIN;
#ifdef BSP_KEY1_ACTIVE_HIGH
    cfg.active_state = BUTTON_ACTIVE_HIGH;
#else
    cfg.active_state = BUTTON_ACTIVE_LOW;
#endif
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));
#endif

#ifdef BSP_USING_KEY2
    cfg.pin = BSP_KEY2_PIN;
#ifdef BSP_KEY2_ACTIVE_HIGH
    cfg.active_state = BUTTON_ACTIVE_HIGH;
#else
    cfg.active_state = BUTTON_ACTIVE_LOW;
#endif
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));
#endif

    while (1)
    {
        rt_thread_mdelay(1000000);
    }

    return RT_EOK;
}

