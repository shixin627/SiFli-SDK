/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"


/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    rt_kprintf("cherryusb device msc_sdcard demo!\n");

    extern void msc_device_init(uint8_t busid, uint32_t reg_base);
    msc_device_init(0, (uintptr_t)USBC_BASE);

    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
