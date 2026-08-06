/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"

uint32_t pmu_ldo_inc(int inc)
{
    /* Change HPSYS LDO VERF */

    uint32_t org = READ_REG(hwp_pmuc->HPSYS_VOUT);
    uint32_t value = org + inc;
    if (value > 0xf)
        value = 0xf;
    if (value < 0xe)
        value = 0xe;
    hwp_pmuc->HPSYS_VOUT = value;
    HAL_Delay_us(20);

    return org;
}

void pmu_ldo_recover(uint32_t org)
{
    hwp_pmuc->HPSYS_VOUT = org;
}



