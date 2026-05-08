/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * PC simulator stub for the HCPU.
 *
 * Provides app_main() — called from middleware/simulator/application.c after
 * RT-Thread is up. main() comes from simulator/startup.c.
 *
 * Companion file pc_link_stubs.c carries the bulk of stub implementations
 * for symbols normally provided by ARM-only modules (BLE stack, voice/gesture/
 * skai apps, peripheral HAL).
 */
#include <rtthread.h>
#include <rtdevice.h>

int app_main(void)
{
    rt_kprintf("PC simulator app_main: HCPU stub\n");
    return RT_EOK;
}
