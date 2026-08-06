/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_AON_SF32LB55X_H
#define __BF0_HAL_AON_SF32LB55X_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __BF0_HAL_AON_H
#error "Not include this file directly, include bf0_hal_aon.h"
#endif /* __BF0_HAL_AON_H */


/** @brief hpsys wakeup source */
typedef enum
{
    /* First part, keep enum value same as macro definition to simplify implementation */
    HPAON_WAKEUP_SRC_RTC       = HPSYS_AON_WER_RTC_Pos,   /**< RTC wakeup source */
    HPAON_WAKEUP_SRC_LPTIM1    = HPSYS_AON_WER_LPTIM1_Pos,    /**< LPTIM1 wakeup source */
    HPAON_WAKEUP_SRC_LP2HP_REQ = HPSYS_AON_WER_LP2HP_REQ_Pos,  /**< LP2HP manual wakeup source */
    HPAON_WAKEUP_SRC_LP2HP_IRQ = HPSYS_AON_WER_LP2HP_IRQ_Pos,  /**< LP2HP mailbox interrupt wakeup source */

    /* Second part, PIN wakeup source */
    /* NOTE:  HPAON_WAKEUP_SRC_PIN0 value must be greater than any non-pin wakeup source */
    HPAON_WAKEUP_SRC_PIN0 = 16,  /**< PIN0 wakeup source  */
    HPAON_WAKEUP_SRC_PIN1 = 17,  /**< PIN1 wakeup source  */
    HPAON_WAKEUP_SRC_PIN2,       /**< PIN2 wakeup source  */
    HPAON_WAKEUP_SRC_PIN3,       /**< PIN3 wakeup source  */
    HPAON_WAKEUP_SRC_PIN_LAST = HPAON_WAKEUP_SRC_PIN3,
} HPAON_WakeupSrcTypeDef;


/** @brief lpsys wakeup source */
typedef enum
{
    /* First part, keep enum value same as macro definition to simplify implementation */
    LPAON_WAKEUP_SRC_RTC       = LPSYS_AON_WER_RTC_Pos,       /**< RTC wakeup source */
    LPAON_WAKEUP_SRC_HP2LP_REQ = LPSYS_AON_WER_HP2LP_REQ_Pos, /**< HP2LP manual wakeup source */
    LPAON_WAKEUP_SRC_HP2LP_IRQ = LPSYS_AON_WER_HP2LP_IRQ_Pos, /**< HP2LP mailbox interrupt wakeup source */
    LPAON_WAKEUP_SRC_LPTIM2    = LPSYS_AON_WER_LPTIM2_Pos,   /**< LPTIM2 wakeup source */
    LPAON_WAKEUP_SRC_LPCOMP1   = LPSYS_AON_WER_LPCOMP1_Pos,  /**< LPCOMP1 wakeup source */
    LPAON_WAKEUP_SRC_LPCOMP2   = LPSYS_AON_WER_LPCOMP2_Pos,  /**< LPCOMP2 wakeup source */
    LPAON_WAKEUP_SRC_BLE       = LPSYS_AON_WER_BLE_Pos,      /**< BLE wakeup source */

    /* Second part, PIN wakeup source */
    /* NOTE:  HPAON_WAKEUP_SRC_PIN0 value must be greater than any non-pin wakeup source */
    LPAON_WAKEUP_SRC_PIN0 = 16,     /**< PIN0 wakeup source */
    LPAON_WAKEUP_SRC_PIN1,          /**< PIN1 wakeup source */
    LPAON_WAKEUP_SRC_PIN2,          /**< PIN2 wakeup source */
    LPAON_WAKEUP_SRC_PIN3,          /**< PIN3 wakeup source */
    LPAON_WAKEUP_SRC_PIN4,          /**< PIN4 wakeup source */
    LPAON_WAKEUP_SRC_PIN5,          /**< PIN5 wakeup source */
    LPAON_WAKEUP_SRC_PIN_LAST = LPAON_WAKEUP_SRC_PIN5,
} LPAON_WakeupSrcTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_AON_SF32LB55X_H */