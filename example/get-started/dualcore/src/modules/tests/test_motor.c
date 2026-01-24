/*
 * Copyright (c) 2018-2024, Skaiwalk Development Team
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2024-02-16     jack         first version
 */
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include "board.h"
#include "bloc_peripheral.h"

#define DBG_TAG "utest.motor"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/// @brief
/// @param argc
/// @param argv start [duty cycle] [period(us)] [repeat times]
/// @return

static int utest_motor_vibrate(int argc, char *argv[])
{
    if (strcmp(argv[1], "-start") == 0) // utest_motor_vibrate -start 75 500000 5
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
        LOG_D("duty_cycle: %d, period: %d, repeat_times: %d", duty_cycle, period, repeat_times);
        motor_params_t param = {
            .duty_cycle = duty_cycle,
            .period = period,
            .repeat_times = repeat_times,
        };
        peripheral_provider.control_motor(true, &param);
    }
    else if (strcmp(argv[1], "-stop") == 0) // utest_motor_vibrate -stop
    {
        LOG_D("stop motor");
        peripheral_provider.control_motor(false, NULL);
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(utest_motor_vibrate, control motor vibrate);
