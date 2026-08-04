/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#ifdef RT_USING_MODULE
    #include "dlmodule.h"
    #include "dlfcn.h"
#endif
#ifdef BSP_USING_DFU
    #include "dfu.h"
#endif

#ifdef RWBT_ENABLE
    #include "rwip.h"
#endif

#include "ke_event.h"

int pre_main(void)
{
    extern void bluetooth_patch_install(void);
    bluetooth_patch_install();
    rtthread_startup();
    return 0;
}

__USED int main(void)
{
    int count = 0;
#ifdef LCPU_INIT_ON_ROM
    _ke_evt_task_entry_handler(KE_EVNET_PRI_HIGH);
#endif // LCPU_INIT_ON_ROM
    return RT_EOK;
}




