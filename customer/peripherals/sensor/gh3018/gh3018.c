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
#include <math.h>
#include <string.h>
#include "stdlib.h"
#include "board.h"
#include "gh30x_example.h"
#include "gh3018.h"
#ifdef BSP_USING_BLOC_PERIPHERAL
    #include "bloc_peripheral.h"
#endif

#define DRV_DEBUG
#define LOG_TAG "drv.hbd"
#include <drv_log.h>

#ifdef HR_USING_GH3018

static rt_mutex_t api_lock;

extern void hal_gh30x_pin_set(uint8_t en);

int init_gh3018_sensor(void)
{
    if (api_lock == NULL)
    {
        api_lock = rt_mutex_create("gh30x_lock", RT_IPC_FLAG_FIFO);
    }
    RT_ASSERT(api_lock != NULL);
    int res = gh30x_module_init();
    if (res == 1)
        return 0;

    return 1;
}

void gh30x_api_lock(void)
{
    rt_mutex_take(api_lock, RT_WAITING_FOREVER);
}

void gh30x_api_unlock(void)
{
    rt_mutex_release(api_lock);
}

int open_gh3018(void)
{
    LOG_I("%s", __func__);
    gh30x_module_stop();
    // gh30x_module_start(GH30X_FUNCTION_SPO2);
    // gh30x_module_start(GH30X_FUNCTION_HR | GH30X_FUNCTION_SOFT_ADT);
    gh30x_module_start(GH30X_FUNCTION_HRV); // 25:GH30X_FUNCTION_SOFT_ADT
                                            // 100:GH30X_FUNCTION_HRV
    return 0;
}

int open_gh3018_high_power(void)
{
    LOG_I("%s", __func__);
    gh30x_module_stop();
    // gh30x_module_start(GH30X_FUNCTION_SPO2);
    gh30x_module_start(GH30X_FUNCTION_HR | GH30X_FUNCTION_SOFT_ADT);
    // gh30x_module_start(GH30X_FUNCTION_SOFT_ADT);
    return 0;
}

int open_gh3018_low_power(void)
{
    LOG_I("%s", __func__);
    gh30x_module_stop();
    gh30x_module_start(GH30X_FUNCTION_ADT | GH30X_FUNCTION_SOFT_ADT);
    return 0;
}

int set_gh3018_hr_mode(void)
{
    LOG_I("%s", __func__);
    gh30x_api_lock();
    HBD_FifoConfig(0, HBD_FUNCTIONAL_STATE_DISABLE);
    HBD_FifoConfig(1, HBD_FUNCTIONAL_STATE_DISABLE);

    gh30x_module_stop();
    gh30x_module_start(GH30X_FUNCTION_HR);

    HBD_FifoConfig(0, HBD_FUNCTIONAL_STATE_ENABLE);
    HBD_FifoConfig(1, HBD_FUNCTIONAL_STATE_ENABLE);
    gh30x_api_unlock();
    return 0;
}

int set_gh3018_spo2_mode(void)
{
    LOG_I("%s", __func__);
    gh30x_api_lock();
    gh30x_module_stop();
    gh30x_module_start(GH30X_FUNCTION_SPO2);
    gh30x_api_unlock();
    return 0;
}

int close_gh3018(void)
{
    LOG_I("%s", __func__);
    gh30x_module_stop();
    return 0;
}

int reset_gh3018(void)
{
    LOG_I("%s\n", __func__);
    // gh30x_module_stop();
    // gh30x_module_start(GH30X_FUNCTION_SPO2);
    return 0;
}

static bool last_wearing_status = false;
void soft_adt_callback(bool status)
{
    // LOG_D("soft_adt_callback %d\n", status);
    if (last_wearing_status == status)
        return;

    if (watch_sys_sync.soft_adt_status_callback)
    {
        watch_sys_sync.soft_adt_status_callback(status);
    }
    last_wearing_status = status;
}

    #define DRV_GH3018_TEST

    #ifdef DRV_GH3018_TEST
        #include <string.h>

// 打印内存使用情况
void print_memory_usage(void)
{
    rt_uint32_t total, used, max_used;
    rt_memory_info(&total, &used, &max_used);
    rt_kprintf("Total memory: %d bytes\n", total);
    rt_kprintf("Used memory: %d bytes\n", used);
    rt_kprintf("Maximum used memory: %d bytes\n", max_used);
}

int cmd_hbd(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "-open") == 0)
        {
            int res = gh30x_module_init();
            LOG_I("Initial gh3018 %d\n", res);
        }
        else if (strcmp(argv[1], "-hb") == 0)
        {
            set_gh3018_hr_mode();
            print_memory_usage();
            LOG_I("start HB\n");
        }
        else if (strcmp(argv[1], "-spo") == 0)
        {
            set_gh3018_spo2_mode();
            LOG_I("start spo2\n");
        }
        else if (strcmp(argv[1], "-hb2") == 0)
        {
            gh30x_module_start(GH30X_FUNCTION_HR);
            LOG_I("start HB without ADT\n");
        }
        else if (strcmp(argv[1], "-spo2") == 0)
        {
            gh30x_module_start(GH30X_FUNCTION_SPO2);
            LOG_I("start spo2 without ADT\n");
        }
        else if (strcmp(argv[1], "-stop") == 0)
        {
            gh30x_module_stop();
            LOG_I("stop gh3018\n");
        }
        else if (strcmp(argv[1], "-reset") == 0)
        {
        }
        else
        {
            LOG_I("Invalid parameter\n");
        }
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_hbd, __cmd_hbd, Test driver gh3018);

    #endif // DRV_GH3018_TEST

#endif // HR_USING_GH3018

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/