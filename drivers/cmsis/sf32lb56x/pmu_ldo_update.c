/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"

uint32_t pmu_ldo_inc(int inc)
{
    /* Change HPSYS LDO VERF */
    uint32_t org = READ_REG(hwp_pmuc->HPSYS_LDO);
    org &= PMUC_HPSYS_LDO_VREF_Msk;
    org >>= PMUC_HPSYS_LDO_VREF_Pos;
    uint32_t value = org + inc;
    if (value > 0xf)
        value = 0xf;
    MODIFY_REG(hwp_pmuc->HPSYS_LDO, PMUC_HPSYS_LDO_VREF_Msk, value << PMUC_HPSYS_LDO_VREF_Pos);

    return org;
}

void pmu_ldo_recover(uint32_t org)
{
    MODIFY_REG(hwp_pmuc->HPSYS_LDO, PMUC_HPSYS_LDO_VREF_Msk, org << PMUC_HPSYS_LDO_VREF_Pos);
}



