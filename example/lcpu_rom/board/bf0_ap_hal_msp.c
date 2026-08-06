/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <stdint.h>
#include "bf0_hal.h"
#include "register.h"

void HAL_MspInit(void)
{
#ifdef SOC_BF0_HCPU
    {
        BSP_IO_Init();
    }
#endif
}

__weak void mpu_config(void)
{

}

#if 1
void HAL_PostMspInit(void)
{

}
#endif


