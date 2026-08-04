/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LPSYS_AON_H
#define __LPSYS_AON_H

typedef struct
{
    __IO uint32_t PMR;
    __IO uint32_t CR;
    __IO uint32_t ACR;
    __IO uint32_t LSCR;
    __IO uint32_t DSCR;
    __IO uint32_t SBCR;
    __IO uint32_t WER;
    __IO uint32_t WSR;
    __IO uint32_t WCR;
    __IO uint32_t ISSR;
    __IO uint32_t TARGET;
    __IO uint32_t ACTUAL;
    __IO uint32_t PRE_WKUP;
    __IO uint32_t SLP_CFG;
    __IO uint32_t SLP_CTRL;
    __IO uint32_t ANACR;
    __IO uint32_t GTIMR;
    __IO uint32_t RESERVE0;
    __IO uint32_t RESERVE1;
    __IO uint32_t RSVD1[45];
    __IO uint32_t SPR;
    __IO uint32_t PCR;
} LPSYS_AON_TypeDef;


/***************** Bit definition for LPSYS_AON_PMR register ******************/
#define LPSYS_AON_PMR_MODE_Pos          (0U)
#define LPSYS_AON_PMR_MODE_Msk          (0x3UL << LPSYS_AON_PMR_MODE_Pos)
#define LPSYS_AON_PMR_MODE              LPSYS_AON_PMR_MODE_Msk
#define LPSYS_AON_PMR_CPUWAIT_Pos       (2U)
#define LPSYS_AON_PMR_CPUWAIT_Msk       (0x1UL << LPSYS_AON_PMR_CPUWAIT_Pos)
#define LPSYS_AON_PMR_CPUWAIT           LPSYS_AON_PMR_CPUWAIT_Msk
#define LPSYS_AON_PMR_FORCE_SLEEP_Pos   (31U)
#define LPSYS_AON_PMR_FORCE_SLEEP_Msk   (0x1UL << LPSYS_AON_PMR_FORCE_SLEEP_Pos)
#define LPSYS_AON_PMR_FORCE_SLEEP       LPSYS_AON_PMR_FORCE_SLEEP_Msk

/****************** Bit definition for LPSYS_AON_CR register ******************/
#define LPSYS_AON_CR_PINOUT_SEL0_Pos    (0U)
#define LPSYS_AON_CR_PINOUT_SEL0_Msk    (0xFUL << LPSYS_AON_CR_PINOUT_SEL0_Pos)
#define LPSYS_AON_CR_PINOUT_SEL0        LPSYS_AON_CR_PINOUT_SEL0_Msk
#define LPSYS_AON_CR_PINOUT_SEL1_Pos    (4U)
#define LPSYS_AON_CR_PINOUT_SEL1_Msk    (0xFUL << LPSYS_AON_CR_PINOUT_SEL1_Pos)
#define LPSYS_AON_CR_PINOUT_SEL1        LPSYS_AON_CR_PINOUT_SEL1_Msk
#define LPSYS_AON_CR_GTIM_EN_Pos        (31U)
#define LPSYS_AON_CR_GTIM_EN_Msk        (0x1UL << LPSYS_AON_CR_GTIM_EN_Pos)
#define LPSYS_AON_CR_GTIM_EN            LPSYS_AON_CR_GTIM_EN_Msk

/***************** Bit definition for LPSYS_AON_ACR register ******************/
#define LPSYS_AON_ACR_HRC48_REQ_Pos     (0U)
#define LPSYS_AON_ACR_HRC48_REQ_Msk     (0x1UL << LPSYS_AON_ACR_HRC48_REQ_Pos)
#define LPSYS_AON_ACR_HRC48_REQ         LPSYS_AON_ACR_HRC48_REQ_Msk
#define LPSYS_AON_ACR_HXT48_REQ_Pos     (1U)
#define LPSYS_AON_ACR_HXT48_REQ_Msk     (0x1UL << LPSYS_AON_ACR_HXT48_REQ_Pos)
#define LPSYS_AON_ACR_HXT48_REQ         LPSYS_AON_ACR_HXT48_REQ_Msk
#define LPSYS_AON_ACR_PWR_REQ_Pos       (2U)
#define LPSYS_AON_ACR_PWR_REQ_Msk       (0x1UL << LPSYS_AON_ACR_PWR_REQ_Pos)
#define LPSYS_AON_ACR_PWR_REQ           LPSYS_AON_ACR_PWR_REQ_Msk
#define LPSYS_AON_ACR_EXTPWR_REQ_Pos    (3U)
#define LPSYS_AON_ACR_EXTPWR_REQ_Msk    (0x1UL << LPSYS_AON_ACR_EXTPWR_REQ_Pos)
#define LPSYS_AON_ACR_EXTPWR_REQ        LPSYS_AON_ACR_EXTPWR_REQ_Msk
#define LPSYS_AON_ACR_HRC48_RDY_Pos     (30U)
#define LPSYS_AON_ACR_HRC48_RDY_Msk     (0x1UL << LPSYS_AON_ACR_HRC48_RDY_Pos)
#define LPSYS_AON_ACR_HRC48_RDY         LPSYS_AON_ACR_HRC48_RDY_Msk
#define LPSYS_AON_ACR_HXT48_RDY_Pos     (31U)
#define LPSYS_AON_ACR_HXT48_RDY_Msk     (0x1UL << LPSYS_AON_ACR_HXT48_RDY_Pos)
#define LPSYS_AON_ACR_HXT48_RDY         LPSYS_AON_ACR_HXT48_RDY_Msk

/***************** Bit definition for LPSYS_AON_LSCR register *****************/
#define LPSYS_AON_LSCR_HRC48_REQ_Pos    (0U)
#define LPSYS_AON_LSCR_HRC48_REQ_Msk    (0x1UL << LPSYS_AON_LSCR_HRC48_REQ_Pos)
#define LPSYS_AON_LSCR_HRC48_REQ        LPSYS_AON_LSCR_HRC48_REQ_Msk
#define LPSYS_AON_LSCR_HXT48_REQ_Pos    (1U)
#define LPSYS_AON_LSCR_HXT48_REQ_Msk    (0x1UL << LPSYS_AON_LSCR_HXT48_REQ_Pos)
#define LPSYS_AON_LSCR_HXT48_REQ        LPSYS_AON_LSCR_HXT48_REQ_Msk
#define LPSYS_AON_LSCR_PWR_REQ_Pos      (2U)
#define LPSYS_AON_LSCR_PWR_REQ_Msk      (0x1UL << LPSYS_AON_LSCR_PWR_REQ_Pos)
#define LPSYS_AON_LSCR_PWR_REQ          LPSYS_AON_LSCR_PWR_REQ_Msk
#define LPSYS_AON_LSCR_EXTPWR_REQ_Pos   (3U)
#define LPSYS_AON_LSCR_EXTPWR_REQ_Msk   (0x1UL << LPSYS_AON_LSCR_EXTPWR_REQ_Pos)
#define LPSYS_AON_LSCR_EXTPWR_REQ       LPSYS_AON_LSCR_EXTPWR_REQ_Msk

/***************** Bit definition for LPSYS_AON_DSCR register *****************/
#define LPSYS_AON_DSCR_HRC48_REQ_Pos    (0U)
#define LPSYS_AON_DSCR_HRC48_REQ_Msk    (0x1UL << LPSYS_AON_DSCR_HRC48_REQ_Pos)
#define LPSYS_AON_DSCR_HRC48_REQ        LPSYS_AON_DSCR_HRC48_REQ_Msk
#define LPSYS_AON_DSCR_HXT48_REQ_Pos    (1U)
#define LPSYS_AON_DSCR_HXT48_REQ_Msk    (0x1UL << LPSYS_AON_DSCR_HXT48_REQ_Pos)
#define LPSYS_AON_DSCR_HXT48_REQ        LPSYS_AON_DSCR_HXT48_REQ_Msk
#define LPSYS_AON_DSCR_PWR_REQ_Pos      (2U)
#define LPSYS_AON_DSCR_PWR_REQ_Msk      (0x1UL << LPSYS_AON_DSCR_PWR_REQ_Pos)
#define LPSYS_AON_DSCR_PWR_REQ          LPSYS_AON_DSCR_PWR_REQ_Msk
#define LPSYS_AON_DSCR_EXTPWR_REQ_Pos   (3U)
#define LPSYS_AON_DSCR_EXTPWR_REQ_Msk   (0x1UL << LPSYS_AON_DSCR_EXTPWR_REQ_Pos)
#define LPSYS_AON_DSCR_EXTPWR_REQ       LPSYS_AON_DSCR_EXTPWR_REQ_Msk

/***************** Bit definition for LPSYS_AON_SBCR register *****************/
#define LPSYS_AON_SBCR_HRC48_REQ_Pos    (0U)
#define LPSYS_AON_SBCR_HRC48_REQ_Msk    (0x1UL << LPSYS_AON_SBCR_HRC48_REQ_Pos)
#define LPSYS_AON_SBCR_HRC48_REQ        LPSYS_AON_SBCR_HRC48_REQ_Msk
#define LPSYS_AON_SBCR_HXT48_REQ_Pos    (1U)
#define LPSYS_AON_SBCR_HXT48_REQ_Msk    (0x1UL << LPSYS_AON_SBCR_HXT48_REQ_Pos)
#define LPSYS_AON_SBCR_HXT48_REQ        LPSYS_AON_SBCR_HXT48_REQ_Msk
#define LPSYS_AON_SBCR_PWR_REQ_Pos      (2U)
#define LPSYS_AON_SBCR_PWR_REQ_Msk      (0x1UL << LPSYS_AON_SBCR_PWR_REQ_Pos)
#define LPSYS_AON_SBCR_PWR_REQ          LPSYS_AON_SBCR_PWR_REQ_Msk
#define LPSYS_AON_SBCR_EXTPWR_REQ_Pos   (3U)
#define LPSYS_AON_SBCR_EXTPWR_REQ_Msk   (0x1UL << LPSYS_AON_SBCR_EXTPWR_REQ_Pos)
#define LPSYS_AON_SBCR_EXTPWR_REQ       LPSYS_AON_SBCR_EXTPWR_REQ_Msk
#define LPSYS_AON_SBCR_PD_RAM0_Pos      (6U)
#define LPSYS_AON_SBCR_PD_RAM0_Msk      (0x1UL << LPSYS_AON_SBCR_PD_RAM0_Pos)
#define LPSYS_AON_SBCR_PD_RAM0          LPSYS_AON_SBCR_PD_RAM0_Msk
#define LPSYS_AON_SBCR_PD_RAM1_Pos      (7U)
#define LPSYS_AON_SBCR_PD_RAM1_Msk      (0x1UL << LPSYS_AON_SBCR_PD_RAM1_Pos)
#define LPSYS_AON_SBCR_PD_RAM1          LPSYS_AON_SBCR_PD_RAM1_Msk

/***************** Bit definition for LPSYS_AON_WER register ******************/
#define LPSYS_AON_WER_BT_Pos            (0U)
#define LPSYS_AON_WER_BT_Msk            (0x1UL << LPSYS_AON_WER_BT_Pos)
#define LPSYS_AON_WER_BT                LPSYS_AON_WER_BT_Msk
#define LPSYS_AON_WER_WDT2_Pos          (1U)
#define LPSYS_AON_WER_WDT2_Msk          (0x1UL << LPSYS_AON_WER_WDT2_Pos)
#define LPSYS_AON_WER_WDT2              LPSYS_AON_WER_WDT2_Msk
#define LPSYS_AON_WER_LPTIM3_Pos        (2U)
#define LPSYS_AON_WER_LPTIM3_Msk        (0x1UL << LPSYS_AON_WER_LPTIM3_Pos)
#define LPSYS_AON_WER_LPTIM3            LPSYS_AON_WER_LPTIM3_Msk
#define LPSYS_AON_WER_HP2LP_REQ_Pos     (6U)
#define LPSYS_AON_WER_HP2LP_REQ_Msk     (0x1UL << LPSYS_AON_WER_HP2LP_REQ_Pos)
#define LPSYS_AON_WER_HP2LP_REQ         LPSYS_AON_WER_HP2LP_REQ_Msk
#define LPSYS_AON_WER_HP2LP_IRQ_Pos     (7U)
#define LPSYS_AON_WER_HP2LP_IRQ_Msk     (0x1UL << LPSYS_AON_WER_HP2LP_IRQ_Pos)
#define LPSYS_AON_WER_HP2LP_IRQ         LPSYS_AON_WER_HP2LP_IRQ_Msk

/***************** Bit definition for LPSYS_AON_WSR register ******************/
#define LPSYS_AON_WSR_BT_Pos            (0U)
#define LPSYS_AON_WSR_BT_Msk            (0x1UL << LPSYS_AON_WSR_BT_Pos)
#define LPSYS_AON_WSR_BT                LPSYS_AON_WSR_BT_Msk
#define LPSYS_AON_WSR_WDT2_Pos          (1U)
#define LPSYS_AON_WSR_WDT2_Msk          (0x1UL << LPSYS_AON_WSR_WDT2_Pos)
#define LPSYS_AON_WSR_WDT2              LPSYS_AON_WSR_WDT2_Msk
#define LPSYS_AON_WSR_LPTIM3_Pos        (2U)
#define LPSYS_AON_WSR_LPTIM3_Msk        (0x1UL << LPSYS_AON_WSR_LPTIM3_Pos)
#define LPSYS_AON_WSR_LPTIM3            LPSYS_AON_WSR_LPTIM3_Msk
#define LPSYS_AON_WSR_HP2LP_REQ_Pos     (6U)
#define LPSYS_AON_WSR_HP2LP_REQ_Msk     (0x1UL << LPSYS_AON_WSR_HP2LP_REQ_Pos)
#define LPSYS_AON_WSR_HP2LP_REQ         LPSYS_AON_WSR_HP2LP_REQ_Msk
#define LPSYS_AON_WSR_HP2LP_IRQ_Pos     (7U)
#define LPSYS_AON_WSR_HP2LP_IRQ_Msk     (0x1UL << LPSYS_AON_WSR_HP2LP_IRQ_Pos)
#define LPSYS_AON_WSR_HP2LP_IRQ         LPSYS_AON_WSR_HP2LP_IRQ_Msk

#define LPSYS_AON_WSR_PIN_ALL           (0UL)

/***************** Bit definition for LPSYS_AON_WCR register ******************/
#define LPSYS_AON_WCR_AON_Pos           (31U)
#define LPSYS_AON_WCR_AON_Msk           (0x1UL << LPSYS_AON_WCR_AON_Pos)
#define LPSYS_AON_WCR_AON               LPSYS_AON_WCR_AON_Msk

/***************** Bit definition for LPSYS_AON_ISSR register *****************/
#define LPSYS_AON_ISSR_LP2HP_REQ_Pos    (0U)
#define LPSYS_AON_ISSR_LP2HP_REQ_Msk    (0x1UL << LPSYS_AON_ISSR_LP2HP_REQ_Pos)
#define LPSYS_AON_ISSR_LP2HP_REQ        LPSYS_AON_ISSR_LP2HP_REQ_Msk
#define LPSYS_AON_ISSR_HP2LP_REQ_Pos    (1U)
#define LPSYS_AON_ISSR_HP2LP_REQ_Msk    (0x1UL << LPSYS_AON_ISSR_HP2LP_REQ_Pos)
#define LPSYS_AON_ISSR_HP2LP_REQ        LPSYS_AON_ISSR_HP2LP_REQ_Msk
#define LPSYS_AON_ISSR_LP_ACTIVE_Pos    (4U)
#define LPSYS_AON_ISSR_LP_ACTIVE_Msk    (0x1UL << LPSYS_AON_ISSR_LP_ACTIVE_Pos)
#define LPSYS_AON_ISSR_LP_ACTIVE        LPSYS_AON_ISSR_LP_ACTIVE_Msk
#define LPSYS_AON_ISSR_HP_ACTIVE_Pos    (5U)
#define LPSYS_AON_ISSR_HP_ACTIVE_Msk    (0x1UL << LPSYS_AON_ISSR_HP_ACTIVE_Pos)
#define LPSYS_AON_ISSR_HP_ACTIVE        LPSYS_AON_ISSR_HP_ACTIVE_Msk

/**************** Bit definition for LPSYS_AON_TARGET register ****************/
#define LPSYS_AON_TARGET_SLEEP_TARGET_Pos  (0U)
#define LPSYS_AON_TARGET_SLEEP_TARGET_Msk  (0xFFFFFFFUL << LPSYS_AON_TARGET_SLEEP_TARGET_Pos)
#define LPSYS_AON_TARGET_SLEEP_TARGET   LPSYS_AON_TARGET_SLEEP_TARGET_Msk

/**************** Bit definition for LPSYS_AON_ACTUAL register ****************/
#define LPSYS_AON_ACTUAL_SLEEP_CNT_Pos  (0U)
#define LPSYS_AON_ACTUAL_SLEEP_CNT_Msk  (0xFFFFFFFUL << LPSYS_AON_ACTUAL_SLEEP_CNT_Pos)
#define LPSYS_AON_ACTUAL_SLEEP_CNT      LPSYS_AON_ACTUAL_SLEEP_CNT_Msk

/*************** Bit definition for LPSYS_AON_PRE_WKUP register ***************/
#define LPSYS_AON_PRE_WKUP_XTAL_TIME_Pos  (0U)
#define LPSYS_AON_PRE_WKUP_XTAL_TIME_Msk  (0x3FFUL << LPSYS_AON_PRE_WKUP_XTAL_TIME_Pos)
#define LPSYS_AON_PRE_WKUP_XTAL_TIME    LPSYS_AON_PRE_WKUP_XTAL_TIME_Msk
#define LPSYS_AON_PRE_WKUP_WKUP_TIME_Pos  (16U)
#define LPSYS_AON_PRE_WKUP_WKUP_TIME_Msk  (0x3FFUL << LPSYS_AON_PRE_WKUP_WKUP_TIME_Pos)
#define LPSYS_AON_PRE_WKUP_WKUP_TIME    LPSYS_AON_PRE_WKUP_WKUP_TIME_Msk

/*************** Bit definition for LPSYS_AON_SLP_CFG register ****************/
#define LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON_Pos  (2U)
#define LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON_Msk  (0x1UL << LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON_Pos)
#define LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON  LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON_Msk
#define LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF_Pos  (3U)
#define LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF_Msk  (0x1UL << LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF_Pos)
#define LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF  LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF_Msk

/*************** Bit definition for LPSYS_AON_SLP_CTRL register ***************/
#define LPSYS_AON_SLP_CTRL_SLEEP_REQ_Pos  (0U)
#define LPSYS_AON_SLP_CTRL_SLEEP_REQ_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_SLEEP_REQ_Pos)
#define LPSYS_AON_SLP_CTRL_SLEEP_REQ    LPSYS_AON_SLP_CTRL_SLEEP_REQ_Msk
#define LPSYS_AON_SLP_CTRL_WKUP_REQ_Pos  (1U)
#define LPSYS_AON_SLP_CTRL_WKUP_REQ_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_WKUP_REQ_Pos)
#define LPSYS_AON_SLP_CTRL_WKUP_REQ     LPSYS_AON_SLP_CTRL_WKUP_REQ_Msk
#define LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Pos  (4U)
#define LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Pos)
#define LPSYS_AON_SLP_CTRL_SLEEP_STATUS  LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Msk
#define LPSYS_AON_SLP_CTRL_XTAL_REQ_Pos  (5U)
#define LPSYS_AON_SLP_CTRL_XTAL_REQ_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_XTAL_REQ_Pos)
#define LPSYS_AON_SLP_CTRL_XTAL_REQ     LPSYS_AON_SLP_CTRL_XTAL_REQ_Msk
#define LPSYS_AON_SLP_CTRL_BT_WKUP_Pos  (6U)
#define LPSYS_AON_SLP_CTRL_BT_WKUP_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_BT_WKUP_Pos)
#define LPSYS_AON_SLP_CTRL_BT_WKUP      LPSYS_AON_SLP_CTRL_BT_WKUP_Msk

/**************** Bit definition for LPSYS_AON_ANACR register *****************/
#define LPSYS_AON_ANACR_VLP_ISO_Pos     (0U)
#define LPSYS_AON_ANACR_VLP_ISO_Msk     (0x1UL << LPSYS_AON_ANACR_VLP_ISO_Pos)
#define LPSYS_AON_ANACR_VLP_ISO         LPSYS_AON_ANACR_VLP_ISO_Msk
#define LPSYS_AON_ANACR_AUTO_ISO_Pos    (1U)
#define LPSYS_AON_ANACR_AUTO_ISO_Msk    (0x1UL << LPSYS_AON_ANACR_AUTO_ISO_Pos)
#define LPSYS_AON_ANACR_AUTO_ISO        LPSYS_AON_ANACR_AUTO_ISO_Msk

/**************** Bit definition for LPSYS_AON_GTIMR register *****************/
#define LPSYS_AON_GTIMR_CNT_Pos         (0U)
#define LPSYS_AON_GTIMR_CNT_Msk         (0xFFFFFFFFUL << LPSYS_AON_GTIMR_CNT_Pos)
#define LPSYS_AON_GTIMR_CNT             LPSYS_AON_GTIMR_CNT_Msk

/*************** Bit definition for LPSYS_AON_RESERVE0 register ***************/
#define LPSYS_AON_RESERVE0_DATA_Pos     (0U)
#define LPSYS_AON_RESERVE0_DATA_Msk     (0xFFFFFFFFUL << LPSYS_AON_RESERVE0_DATA_Pos)
#define LPSYS_AON_RESERVE0_DATA         LPSYS_AON_RESERVE0_DATA_Msk

/*************** Bit definition for LPSYS_AON_RESERVE1 register ***************/
#define LPSYS_AON_RESERVE1_DATA_Pos     (0U)
#define LPSYS_AON_RESERVE1_DATA_Msk     (0xFFFFFFFFUL << LPSYS_AON_RESERVE1_DATA_Pos)
#define LPSYS_AON_RESERVE1_DATA         LPSYS_AON_RESERVE1_DATA_Msk

/***************** Bit definition for LPSYS_AON_SPR register ******************/
#define LPSYS_AON_SPR_SP_Pos            (0U)
#define LPSYS_AON_SPR_SP_Msk            (0xFFFFFFFFUL << LPSYS_AON_SPR_SP_Pos)
#define LPSYS_AON_SPR_SP                LPSYS_AON_SPR_SP_Msk

/***************** Bit definition for LPSYS_AON_PCR register ******************/
#define LPSYS_AON_PCR_PC_Pos            (0U)
#define LPSYS_AON_PCR_PC_Msk            (0xFFFFFFFFUL << LPSYS_AON_PCR_PC_Pos)
#define LPSYS_AON_PCR_PC                LPSYS_AON_PCR_PC_Msk

#endif
