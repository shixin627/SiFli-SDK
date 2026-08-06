/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PMUC_H
#define __PMUC_H

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t WER;
    __IO uint32_t WSR;
    __IO uint32_t WCR;
    __IO uint32_t VRTC_CR;
    __IO uint32_t VRET_CR;
    __IO uint32_t LRC10_CR;
    __IO uint32_t LRC32_CR;
    __IO uint32_t LXT_CR;
    __IO uint32_t AON_BG;
    __IO uint32_t AON_LDO;
    __IO uint32_t BUCK_CR1;
    __IO uint32_t BUCK_CR2;
    __IO uint32_t BUCK_CR3;
    __IO uint32_t CHG_CR1;
    __IO uint32_t CHG_CR2;
    __IO uint32_t CHG_CR3;
    __IO uint32_t CHG_CR4;
    __IO uint32_t CHG_CR5;
    __IO uint32_t CHG_SR;
    __IO uint32_t HPSYS_LDO;
    __IO uint32_t LPSYS_LDO;
    __IO uint32_t HPSYS_SWR;
    __IO uint32_t LPSYS_SWR;
    __IO uint32_t PERI_LDO;
    __IO uint32_t PMU_TR;
    __IO uint32_t PMU_RSVD;
    __IO uint32_t HXT_CR1;
    __IO uint32_t HXT_CR2;
    __IO uint32_t HXT_CR3;
    __IO uint32_t HRC_CR1;
    __IO uint32_t HRC_CR2;
    __IO uint32_t CAU_BGR;
    __IO uint32_t CAU_TR;
    __IO uint32_t CAU_RSVD;
    __IO uint32_t PWRKEY_CNT;
    __IO uint32_t HPSYS_VOUT;
    __IO uint32_t LPSYS_VOUT;
    __IO uint32_t BUCK_VOUT;
    __IO uint32_t WKUP_MODE;
    __IO uint32_t WKUP_CNT;
    __IO uint32_t PRSR1;
    __IO uint32_t PRSR2;
    __IO uint32_t PRCR1;
    __IO uint32_t PRCR2;
    __IO uint32_t PBRCR;
    __IO uint32_t NULLB8;
    __IO uint32_t NULLBC;
    __IO uint32_t NULLC0;
    __IO uint32_t NULLC4;
    __IO uint32_t NULLC8;
    __IO uint32_t NULLCC;
    __IO uint32_t NULLD0;
} PMUC_TypeDef;


/******************** Bit definition for PMUC_CR register *********************/
#define PMUC_CR_SEL_WDT_Pos             (0U)
#define PMUC_CR_SEL_WDT_Msk             (0x1UL << PMUC_CR_SEL_WDT_Pos)
#define PMUC_CR_SEL_WDT                 PMUC_CR_SEL_WDT_Msk
#define PMUC_CR_SEL_RTC_Pos             (1U)
#define PMUC_CR_SEL_RTC_Msk             (0x1UL << PMUC_CR_SEL_RTC_Pos)
#define PMUC_CR_SEL_RTC                 PMUC_CR_SEL_RTC_Msk
#define PMUC_CR_HIBER_EN_Pos            (2U)
#define PMUC_CR_HIBER_EN_Msk            (0x1UL << PMUC_CR_HIBER_EN_Pos)
#define PMUC_CR_HIBER_EN                PMUC_CR_HIBER_EN_Msk
#define PMUC_CR_REBOOT_Pos              (3U)
#define PMUC_CR_REBOOT_Msk              (0x1UL << PMUC_CR_REBOOT_Pos)
#define PMUC_CR_REBOOT                  PMUC_CR_REBOOT_Msk
#define PMUC_CR_LPSYSRST_Pos            (4U)
#define PMUC_CR_LPSYSRST_Msk            (0x1UL << PMUC_CR_LPSYSRST_Pos)
#define PMUC_CR_LPSYSRST                PMUC_CR_LPSYSRST_Msk

/******************** Bit definition for PMUC_WER register ********************/
#define PMUC_WER_PA33_Pos               (0U)
#define PMUC_WER_PA33_Msk               (0x1UL << PMUC_WER_PA33_Pos)
#define PMUC_WER_PA33                   PMUC_WER_PA33_Msk
#define PMUC_WER_PA34_Pos               (1U)
#define PMUC_WER_PA34_Msk               (0x1UL << PMUC_WER_PA34_Pos)
#define PMUC_WER_PA34                   PMUC_WER_PA34_Msk
#define PMUC_WER_PA35_Pos               (2U)
#define PMUC_WER_PA35_Msk               (0x1UL << PMUC_WER_PA35_Pos)
#define PMUC_WER_PA35                   PMUC_WER_PA35_Msk
#define PMUC_WER_PA36_Pos               (3U)
#define PMUC_WER_PA36_Msk               (0x1UL << PMUC_WER_PA36_Pos)
#define PMUC_WER_PA36                   PMUC_WER_PA36_Msk
#define PMUC_WER_PA37_Pos               (4U)
#define PMUC_WER_PA37_Msk               (0x1UL << PMUC_WER_PA37_Pos)
#define PMUC_WER_PA37                   PMUC_WER_PA37_Msk
#define PMUC_WER_PA38_Pos               (5U)
#define PMUC_WER_PA38_Msk               (0x1UL << PMUC_WER_PA38_Pos)
#define PMUC_WER_PA38                   PMUC_WER_PA38_Msk
#define PMUC_WER_PA39_Pos               (6U)
#define PMUC_WER_PA39_Msk               (0x1UL << PMUC_WER_PA39_Pos)
#define PMUC_WER_PA39                   PMUC_WER_PA39_Msk
#define PMUC_WER_PA40_Pos               (7U)
#define PMUC_WER_PA40_Msk               (0x1UL << PMUC_WER_PA40_Pos)
#define PMUC_WER_PA40                   PMUC_WER_PA40_Msk
#define PMUC_WER_PA41_Pos               (8U)
#define PMUC_WER_PA41_Msk               (0x1UL << PMUC_WER_PA41_Pos)
#define PMUC_WER_PA41                   PMUC_WER_PA41_Msk
#define PMUC_WER_PA42_Pos               (9U)
#define PMUC_WER_PA42_Msk               (0x1UL << PMUC_WER_PA42_Pos)
#define PMUC_WER_PA42                   PMUC_WER_PA42_Msk
#define PMUC_WER_PA24_Pos               (10U)
#define PMUC_WER_PA24_Msk               (0x1UL << PMUC_WER_PA24_Pos)
#define PMUC_WER_PA24                   PMUC_WER_PA24_Msk
#define PMUC_WER_PA25_Pos               (11U)
#define PMUC_WER_PA25_Msk               (0x1UL << PMUC_WER_PA25_Pos)
#define PMUC_WER_PA25                   PMUC_WER_PA25_Msk
#define PMUC_WER_PA26_Pos               (12U)
#define PMUC_WER_PA26_Msk               (0x1UL << PMUC_WER_PA26_Pos)
#define PMUC_WER_PA26                   PMUC_WER_PA26_Msk
#define PMUC_WER_PA27_Pos               (13U)
#define PMUC_WER_PA27_Msk               (0x1UL << PMUC_WER_PA27_Pos)
#define PMUC_WER_PA27                   PMUC_WER_PA27_Msk
#define PMUC_WER_RTC_Pos                (16U)
#define PMUC_WER_RTC_Msk                (0x1UL << PMUC_WER_RTC_Pos)
#define PMUC_WER_RTC                    PMUC_WER_RTC_Msk
#define PMUC_WER_IWDT_Pos               (17U)
#define PMUC_WER_IWDT_Msk               (0x1UL << PMUC_WER_IWDT_Pos)
#define PMUC_WER_IWDT                   PMUC_WER_IWDT_Msk
#define PMUC_WER_CHG_Pos                (25U)
#define PMUC_WER_CHG_Msk                (0x1UL << PMUC_WER_CHG_Pos)
#define PMUC_WER_CHG                    PMUC_WER_CHG_Msk
#define PMUC_WER_LOWBAT_Pos             (29U)
#define PMUC_WER_LOWBAT_Msk             (0x1UL << PMUC_WER_LOWBAT_Pos)
#define PMUC_WER_LOWBAT                 PMUC_WER_LOWBAT_Msk
#define PMUC_WER_PWRKEY_Pos             (30U)
#define PMUC_WER_PWRKEY_Msk             (0x1UL << PMUC_WER_PWRKEY_Pos)
#define PMUC_WER_PWRKEY                 PMUC_WER_PWRKEY_Msk

/******************** Bit definition for PMUC_WSR register ********************/
#define PMUC_WSR_PA33_Pos               (0U)
#define PMUC_WSR_PA33_Msk               (0x1UL << PMUC_WSR_PA33_Pos)
#define PMUC_WSR_PA33                   PMUC_WSR_PA33_Msk
#define PMUC_WSR_PA34_Pos               (1U)
#define PMUC_WSR_PA34_Msk               (0x1UL << PMUC_WSR_PA34_Pos)
#define PMUC_WSR_PA34                   PMUC_WSR_PA34_Msk
#define PMUC_WSR_PA35_Pos               (2U)
#define PMUC_WSR_PA35_Msk               (0x1UL << PMUC_WSR_PA35_Pos)
#define PMUC_WSR_PA35                   PMUC_WSR_PA35_Msk
#define PMUC_WSR_PA36_Pos               (3U)
#define PMUC_WSR_PA36_Msk               (0x1UL << PMUC_WSR_PA36_Pos)
#define PMUC_WSR_PA36                   PMUC_WSR_PA36_Msk
#define PMUC_WSR_PA37_Pos               (4U)
#define PMUC_WSR_PA37_Msk               (0x1UL << PMUC_WSR_PA37_Pos)
#define PMUC_WSR_PA37                   PMUC_WSR_PA37_Msk
#define PMUC_WSR_PA38_Pos               (5U)
#define PMUC_WSR_PA38_Msk               (0x1UL << PMUC_WSR_PA38_Pos)
#define PMUC_WSR_PA38                   PMUC_WSR_PA38_Msk
#define PMUC_WSR_PA39_Pos               (6U)
#define PMUC_WSR_PA39_Msk               (0x1UL << PMUC_WSR_PA39_Pos)
#define PMUC_WSR_PA39                   PMUC_WSR_PA39_Msk
#define PMUC_WSR_PA40_Pos               (7U)
#define PMUC_WSR_PA40_Msk               (0x1UL << PMUC_WSR_PA40_Pos)
#define PMUC_WSR_PA40                   PMUC_WSR_PA40_Msk
#define PMUC_WSR_PA41_Pos               (8U)
#define PMUC_WSR_PA41_Msk               (0x1UL << PMUC_WSR_PA41_Pos)
#define PMUC_WSR_PA41                   PMUC_WSR_PA41_Msk
#define PMUC_WSR_PA42_Pos               (9U)
#define PMUC_WSR_PA42_Msk               (0x1UL << PMUC_WSR_PA42_Pos)
#define PMUC_WSR_PA42                   PMUC_WSR_PA42_Msk
#define PMUC_WSR_PA24_Pos               (10U)
#define PMUC_WSR_PA24_Msk               (0x1UL << PMUC_WSR_PA24_Pos)
#define PMUC_WSR_PA24                   PMUC_WSR_PA24_Msk
#define PMUC_WSR_PA25_Pos               (11U)
#define PMUC_WSR_PA25_Msk               (0x1UL << PMUC_WSR_PA25_Pos)
#define PMUC_WSR_PA25                   PMUC_WSR_PA25_Msk
#define PMUC_WSR_PA26_Pos               (12U)
#define PMUC_WSR_PA26_Msk               (0x1UL << PMUC_WSR_PA26_Pos)
#define PMUC_WSR_PA26                   PMUC_WSR_PA26_Msk
#define PMUC_WSR_PA27_Pos               (13U)
#define PMUC_WSR_PA27_Msk               (0x1UL << PMUC_WSR_PA27_Pos)
#define PMUC_WSR_PA27                   PMUC_WSR_PA27_Msk
#define PMUC_WSR_RTC_Pos                (16U)
#define PMUC_WSR_RTC_Msk                (0x1UL << PMUC_WSR_RTC_Pos)
#define PMUC_WSR_RTC                    PMUC_WSR_RTC_Msk
#define PMUC_WSR_IWDT_Pos               (17U)
#define PMUC_WSR_IWDT_Msk               (0x1UL << PMUC_WSR_IWDT_Pos)
#define PMUC_WSR_IWDT                   PMUC_WSR_IWDT_Msk
#define PMUC_WSR_CHG_Pos                (25U)
#define PMUC_WSR_CHG_Msk                (0x1UL << PMUC_WSR_CHG_Pos)
#define PMUC_WSR_CHG                    PMUC_WSR_CHG_Msk
#define PMUC_WSR_SYSRSTREQ_Pos          (27U)
#define PMUC_WSR_SYSRSTREQ_Msk          (0x1UL << PMUC_WSR_SYSRSTREQ_Pos)
#define PMUC_WSR_SYSRSTREQ              PMUC_WSR_SYSRSTREQ_Msk
#define PMUC_WSR_IWDTRST_Pos            (28U)
#define PMUC_WSR_IWDTRST_Msk            (0x1UL << PMUC_WSR_IWDTRST_Pos)
#define PMUC_WSR_IWDTRST                PMUC_WSR_IWDTRST_Msk
#define PMUC_WSR_LOWBAT_Pos             (29U)
#define PMUC_WSR_LOWBAT_Msk             (0x1UL << PMUC_WSR_LOWBAT_Pos)
#define PMUC_WSR_LOWBAT                 PMUC_WSR_LOWBAT_Msk
#define PMUC_WSR_PWRKEY_Pos             (30U)
#define PMUC_WSR_PWRKEY_Msk             (0x1UL << PMUC_WSR_PWRKEY_Pos)
#define PMUC_WSR_PWRKEY                 PMUC_WSR_PWRKEY_Msk
#define PMUC_WSR_PIN_ALL                (PMUC_WSR_PA33 | PMUC_WSR_PA34 | PMUC_WSR_PA35 | PMUC_WSR_PA36      \
                                         | PMUC_WSR_PA37 | PMUC_WSR_PA38 | PMUC_WSR_PA39 | PMUC_WSR_PA40    \
                                         | PMUC_WSR_PA41 | PMUC_WSR_PA42                                    \
                                         | PMUC_WSR_PA24 | PMUC_WSR_PA25 | PMUC_WSR_PA26 | PMUC_WSR_PA27)
#define PMUC_WSR_PIN_NUM                (14)

/******************** Bit definition for PMUC_WCR register ********************/
#define PMUC_WCR_PA33_Pos               (0U)
#define PMUC_WCR_PA33_Msk               (0x1UL << PMUC_WCR_PA33_Pos)
#define PMUC_WCR_PA33                   PMUC_WCR_PA33_Msk
#define PMUC_WCR_PA34_Pos               (1U)
#define PMUC_WCR_PA34_Msk               (0x1UL << PMUC_WCR_PA34_Pos)
#define PMUC_WCR_PA34                   PMUC_WCR_PA34_Msk
#define PMUC_WCR_PA35_Pos               (2U)
#define PMUC_WCR_PA35_Msk               (0x1UL << PMUC_WCR_PA35_Pos)
#define PMUC_WCR_PA35                   PMUC_WCR_PA35_Msk
#define PMUC_WCR_PA36_Pos               (3U)
#define PMUC_WCR_PA36_Msk               (0x1UL << PMUC_WCR_PA36_Pos)
#define PMUC_WCR_PA36                   PMUC_WCR_PA36_Msk
#define PMUC_WCR_PA37_Pos               (4U)
#define PMUC_WCR_PA37_Msk               (0x1UL << PMUC_WCR_PA37_Pos)
#define PMUC_WCR_PA37                   PMUC_WCR_PA37_Msk
#define PMUC_WCR_PA38_Pos               (5U)
#define PMUC_WCR_PA38_Msk               (0x1UL << PMUC_WCR_PA38_Pos)
#define PMUC_WCR_PA38                   PMUC_WCR_PA38_Msk
#define PMUC_WCR_PA39_Pos               (6U)
#define PMUC_WCR_PA39_Msk               (0x1UL << PMUC_WCR_PA39_Pos)
#define PMUC_WCR_PA39                   PMUC_WCR_PA39_Msk
#define PMUC_WCR_PA40_Pos               (7U)
#define PMUC_WCR_PA40_Msk               (0x1UL << PMUC_WCR_PA40_Pos)
#define PMUC_WCR_PA40                   PMUC_WCR_PA40_Msk
#define PMUC_WCR_PA41_Pos               (8U)
#define PMUC_WCR_PA41_Msk               (0x1UL << PMUC_WCR_PA41_Pos)
#define PMUC_WCR_PA41                   PMUC_WCR_PA41_Msk
#define PMUC_WCR_PA42_Pos               (9U)
#define PMUC_WCR_PA42_Msk               (0x1UL << PMUC_WCR_PA42_Pos)
#define PMUC_WCR_PA42                   PMUC_WCR_PA42_Msk
#define PMUC_WCR_PA24_Pos               (10U)
#define PMUC_WCR_PA24_Msk               (0x1UL << PMUC_WCR_PA24_Pos)
#define PMUC_WCR_PA24                   PMUC_WCR_PA24_Msk
#define PMUC_WCR_PA25_Pos               (11U)
#define PMUC_WCR_PA25_Msk               (0x1UL << PMUC_WCR_PA25_Pos)
#define PMUC_WCR_PA25                   PMUC_WCR_PA25_Msk
#define PMUC_WCR_PA26_Pos               (12U)
#define PMUC_WCR_PA26_Msk               (0x1UL << PMUC_WCR_PA26_Pos)
#define PMUC_WCR_PA26                   PMUC_WCR_PA26_Msk
#define PMUC_WCR_PA27_Pos               (13U)
#define PMUC_WCR_PA27_Msk               (0x1UL << PMUC_WCR_PA27_Pos)
#define PMUC_WCR_PA27                   PMUC_WCR_PA27_Msk
#define PMUC_WCR_SYSRSTREQ_Pos          (27U)
#define PMUC_WCR_SYSRSTREQ_Msk          (0x1UL << PMUC_WCR_SYSRSTREQ_Pos)
#define PMUC_WCR_SYSRSTREQ              PMUC_WCR_SYSRSTREQ_Msk
#define PMUC_WCR_IWDTRST_Pos            (28U)
#define PMUC_WCR_IWDTRST_Msk            (0x1UL << PMUC_WCR_IWDTRST_Pos)
#define PMUC_WCR_IWDTRST                PMUC_WCR_IWDTRST_Msk
#define PMUC_WCR_LOWBAT_Pos             (29U)
#define PMUC_WCR_LOWBAT_Msk             (0x1UL << PMUC_WCR_LOWBAT_Pos)
#define PMUC_WCR_LOWBAT                 PMUC_WCR_LOWBAT_Msk
#define PMUC_WCR_PWRKEY_Pos             (30U)
#define PMUC_WCR_PWRKEY_Msk             (0x1UL << PMUC_WCR_PWRKEY_Pos)
#define PMUC_WCR_PWRKEY                 PMUC_WCR_PWRKEY_Msk

/****************** Bit definition for PMUC_VRTC_CR register ******************/
#define PMUC_VRTC_CR_VRTC_VBIT_Pos      (0U)
#define PMUC_VRTC_CR_VRTC_VBIT_Msk      (0xFUL << PMUC_VRTC_CR_VRTC_VBIT_Pos)
#define PMUC_VRTC_CR_VRTC_VBIT          PMUC_VRTC_CR_VRTC_VBIT_Msk
#define PMUC_VRTC_CR_VRTC_TRIM_Pos      (4U)
#define PMUC_VRTC_CR_VRTC_TRIM_Msk      (0xFUL << PMUC_VRTC_CR_VRTC_TRIM_Pos)
#define PMUC_VRTC_CR_VRTC_TRIM          PMUC_VRTC_CR_VRTC_TRIM_Msk
#define PMUC_VRTC_CR_BOR_EN_Pos         (8U)
#define PMUC_VRTC_CR_BOR_EN_Msk         (0x1UL << PMUC_VRTC_CR_BOR_EN_Pos)
#define PMUC_VRTC_CR_BOR_EN             PMUC_VRTC_CR_BOR_EN_Msk
#define PMUC_VRTC_CR_BOR_VT_TRIM_Pos    (9U)
#define PMUC_VRTC_CR_BOR_VT_TRIM_Msk    (0xFUL << PMUC_VRTC_CR_BOR_VT_TRIM_Pos)
#define PMUC_VRTC_CR_BOR_VT_TRIM        PMUC_VRTC_CR_BOR_VT_TRIM_Msk
#define PMUC_VRTC_CR_RESERVE_Pos        (13U)
#define PMUC_VRTC_CR_RESERVE_Msk        (0x7FUL << PMUC_VRTC_CR_RESERVE_Pos)
#define PMUC_VRTC_CR_RESERVE            PMUC_VRTC_CR_RESERVE_Msk

/****************** Bit definition for PMUC_VRET_CR register ******************/
#define PMUC_VRET_CR_EN_Pos             (0U)
#define PMUC_VRET_CR_EN_Msk             (0x1UL << PMUC_VRET_CR_EN_Pos)
#define PMUC_VRET_CR_EN                 PMUC_VRET_CR_EN_Msk
#define PMUC_VRET_CR_BM_Pos             (1U)
#define PMUC_VRET_CR_BM_Msk             (0x1UL << PMUC_VRET_CR_BM_Pos)
#define PMUC_VRET_CR_BM                 PMUC_VRET_CR_BM_Msk
#define PMUC_VRET_CR_VBIT_Pos           (2U)
#define PMUC_VRET_CR_VBIT_Msk           (0xFUL << PMUC_VRET_CR_VBIT_Pos)
#define PMUC_VRET_CR_VBIT               PMUC_VRET_CR_VBIT_Msk
#define PMUC_VRET_CR_TRIM_Pos           (10U)
#define PMUC_VRET_CR_TRIM_Msk           (0xFUL << PMUC_VRET_CR_TRIM_Pos)
#define PMUC_VRET_CR_TRIM               PMUC_VRET_CR_TRIM_Msk
#define PMUC_VRET_CR_DLY_Pos            (16U)
#define PMUC_VRET_CR_DLY_Msk            (0x3FUL << PMUC_VRET_CR_DLY_Pos)
#define PMUC_VRET_CR_DLY                PMUC_VRET_CR_DLY_Msk
#define PMUC_VRET_CR_RDY_Pos            (31U)
#define PMUC_VRET_CR_RDY_Msk            (0x1UL << PMUC_VRET_CR_RDY_Pos)
#define PMUC_VRET_CR_RDY                PMUC_VRET_CR_RDY_Msk

/***************** Bit definition for PMUC_LRC10_CR register ******************/
#define PMUC_LRC10_CR_EN_Pos            (0U)
#define PMUC_LRC10_CR_EN_Msk            (0x1UL << PMUC_LRC10_CR_EN_Pos)
#define PMUC_LRC10_CR_EN                PMUC_LRC10_CR_EN_Msk
#define PMUC_LRC10_CR_CMPBM1_Pos        (1U)
#define PMUC_LRC10_CR_CMPBM1_Msk        (0x3UL << PMUC_LRC10_CR_CMPBM1_Pos)
#define PMUC_LRC10_CR_CMPBM1            PMUC_LRC10_CR_CMPBM1_Msk
#define PMUC_LRC10_CR_CMPBM2_Pos        (3U)
#define PMUC_LRC10_CR_CMPBM2_Msk        (0x1UL << PMUC_LRC10_CR_CMPBM2_Pos)
#define PMUC_LRC10_CR_CMPBM2            PMUC_LRC10_CR_CMPBM2_Msk
#define PMUC_LRC10_CR_CHGCRT_Pos        (4U)
#define PMUC_LRC10_CR_CHGCRT_Msk        (0x3UL << PMUC_LRC10_CR_CHGCRT_Pos)
#define PMUC_LRC10_CR_CHGCRT            PMUC_LRC10_CR_CHGCRT_Msk
#define PMUC_LRC10_CR_CHGCAP_Pos        (6U)
#define PMUC_LRC10_CR_CHGCAP_Msk        (0x3UL << PMUC_LRC10_CR_CHGCAP_Pos)
#define PMUC_LRC10_CR_CHGCAP            PMUC_LRC10_CR_CHGCAP_Msk
#define PMUC_LRC10_CR_REFRES_Pos        (8U)
#define PMUC_LRC10_CR_REFRES_Msk        (0x1UL << PMUC_LRC10_CR_REFRES_Pos)
#define PMUC_LRC10_CR_REFRES            PMUC_LRC10_CR_REFRES_Msk
#define PMUC_LRC10_CR_LDO_EN_Pos        (9U)
#define PMUC_LRC10_CR_LDO_EN_Msk        (0x1UL << PMUC_LRC10_CR_LDO_EN_Pos)
#define PMUC_LRC10_CR_LDO_EN            PMUC_LRC10_CR_LDO_EN_Msk
#define PMUC_LRC10_CR_LDO_VBIT_Pos      (10U)
#define PMUC_LRC10_CR_LDO_VBIT_Msk      (0xFUL << PMUC_LRC10_CR_LDO_VBIT_Pos)
#define PMUC_LRC10_CR_LDO_VBIT          PMUC_LRC10_CR_LDO_VBIT_Msk
#define PMUC_LRC10_CR_LDO_TRIM_Pos      (14U)
#define PMUC_LRC10_CR_LDO_TRIM_Msk      (0xFUL << PMUC_LRC10_CR_LDO_TRIM_Pos)
#define PMUC_LRC10_CR_LDO_TRIM          PMUC_LRC10_CR_LDO_TRIM_Msk
#define PMUC_LRC10_CR_RDY_Pos           (31U)
#define PMUC_LRC10_CR_RDY_Msk           (0x1UL << PMUC_LRC10_CR_RDY_Pos)
#define PMUC_LRC10_CR_RDY               PMUC_LRC10_CR_RDY_Msk

/***************** Bit definition for PMUC_LRC32_CR register ******************/
#define PMUC_LRC32_CR_EN_Pos            (0U)
#define PMUC_LRC32_CR_EN_Msk            (0x1UL << PMUC_LRC32_CR_EN_Pos)
#define PMUC_LRC32_CR_EN                PMUC_LRC32_CR_EN_Msk
#define PMUC_LRC32_CR_CMPBM1_Pos        (1U)
#define PMUC_LRC32_CR_CMPBM1_Msk        (0x3UL << PMUC_LRC32_CR_CMPBM1_Pos)
#define PMUC_LRC32_CR_CMPBM1            PMUC_LRC32_CR_CMPBM1_Msk
#define PMUC_LRC32_CR_CMPBM2_Pos        (3U)
#define PMUC_LRC32_CR_CMPBM2_Msk        (0x1UL << PMUC_LRC32_CR_CMPBM2_Pos)
#define PMUC_LRC32_CR_CMPBM2            PMUC_LRC32_CR_CMPBM2_Msk
#define PMUC_LRC32_CR_CHGCRT_Pos        (4U)
#define PMUC_LRC32_CR_CHGCRT_Msk        (0x3UL << PMUC_LRC32_CR_CHGCRT_Pos)
#define PMUC_LRC32_CR_CHGCRT            PMUC_LRC32_CR_CHGCRT_Msk
#define PMUC_LRC32_CR_RSEL_Pos          (6U)
#define PMUC_LRC32_CR_RSEL_Msk          (0xFUL << PMUC_LRC32_CR_RSEL_Pos)
#define PMUC_LRC32_CR_RSEL              PMUC_LRC32_CR_RSEL_Msk
#define PMUC_LRC32_CR_RDY_Pos           (31U)
#define PMUC_LRC32_CR_RDY_Msk           (0x1UL << PMUC_LRC32_CR_RDY_Pos)
#define PMUC_LRC32_CR_RDY               PMUC_LRC32_CR_RDY_Msk

/****************** Bit definition for PMUC_LXT_CR register *******************/
#define PMUC_LXT_CR_EN_Pos              (0U)
#define PMUC_LXT_CR_EN_Msk              (0x1UL << PMUC_LXT_CR_EN_Pos)
#define PMUC_LXT_CR_EN                  PMUC_LXT_CR_EN_Msk
#define PMUC_LXT_CR_RSN_Pos             (1U)
#define PMUC_LXT_CR_RSN_Msk             (0x1UL << PMUC_LXT_CR_RSN_Pos)
#define PMUC_LXT_CR_RSN                 PMUC_LXT_CR_RSN_Msk
#define PMUC_LXT_CR_BM_Pos              (2U)
#define PMUC_LXT_CR_BM_Msk              (0xFUL << PMUC_LXT_CR_BM_Pos)
#define PMUC_LXT_CR_BM                  PMUC_LXT_CR_BM_Msk
#define PMUC_LXT_CR_AMP_BM_Pos          (6U)
#define PMUC_LXT_CR_AMP_BM_Msk          (0x3UL << PMUC_LXT_CR_AMP_BM_Pos)
#define PMUC_LXT_CR_AMP_BM              PMUC_LXT_CR_AMP_BM_Msk
#define PMUC_LXT_CR_AMPCTRL_ENB_Pos     (8U)
#define PMUC_LXT_CR_AMPCTRL_ENB_Msk     (0x1UL << PMUC_LXT_CR_AMPCTRL_ENB_Pos)
#define PMUC_LXT_CR_AMPCTRL_ENB         PMUC_LXT_CR_AMPCTRL_ENB_Msk
#define PMUC_LXT_CR_BMSEL_Pos           (9U)
#define PMUC_LXT_CR_BMSEL_Msk           (0x1UL << PMUC_LXT_CR_BMSEL_Pos)
#define PMUC_LXT_CR_BMSEL               PMUC_LXT_CR_BMSEL_Msk
#define PMUC_LXT_CR_BMSTART_Pos         (10U)
#define PMUC_LXT_CR_BMSTART_Msk         (0xFUL << PMUC_LXT_CR_BMSTART_Pos)
#define PMUC_LXT_CR_BMSTART             PMUC_LXT_CR_BMSTART_Msk
#define PMUC_LXT_CR_CAP_SEL_Pos         (14U)
#define PMUC_LXT_CR_CAP_SEL_Msk         (0x1UL << PMUC_LXT_CR_CAP_SEL_Pos)
#define PMUC_LXT_CR_CAP_SEL             PMUC_LXT_CR_CAP_SEL_Msk
#define PMUC_LXT_CR_EXT_EN_Pos          (15U)
#define PMUC_LXT_CR_EXT_EN_Msk          (0x1UL << PMUC_LXT_CR_EXT_EN_Pos)
#define PMUC_LXT_CR_EXT_EN              PMUC_LXT_CR_EXT_EN_Msk
#define PMUC_LXT_CR_RDY_Pos             (31U)
#define PMUC_LXT_CR_RDY_Msk             (0x1UL << PMUC_LXT_CR_RDY_Pos)
#define PMUC_LXT_CR_RDY                 PMUC_LXT_CR_RDY_Msk

/****************** Bit definition for PMUC_AON_BG register *******************/
#define PMUC_AON_BG_BUF_VOS_TRIM_Pos    (0U)
#define PMUC_AON_BG_BUF_VOS_TRIM_Msk    (0x7UL << PMUC_AON_BG_BUF_VOS_TRIM_Pos)
#define PMUC_AON_BG_BUF_VOS_TRIM        PMUC_AON_BG_BUF_VOS_TRIM_Msk
#define PMUC_AON_BG_BUF_VOS_STEP_Pos    (3U)
#define PMUC_AON_BG_BUF_VOS_STEP_Msk    (0x3UL << PMUC_AON_BG_BUF_VOS_STEP_Pos)
#define PMUC_AON_BG_BUF_VOS_STEP        PMUC_AON_BG_BUF_VOS_STEP_Msk
#define PMUC_AON_BG_BUF_VOS_POLAR_Pos   (5U)
#define PMUC_AON_BG_BUF_VOS_POLAR_Msk   (0x1UL << PMUC_AON_BG_BUF_VOS_POLAR_Pos)
#define PMUC_AON_BG_BUF_VOS_POLAR       PMUC_AON_BG_BUF_VOS_POLAR_Msk

/****************** Bit definition for PMUC_AON_LDO register ******************/
#define PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos  (0U)
#define PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk  (0xFUL << PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Pos)
#define PMUC_AON_LDO_VBAT_LDO_SET_VOUT  PMUC_AON_LDO_VBAT_LDO_SET_VOUT_Msk
#define PMUC_AON_LDO_VBAT_LDO_EN_SS_Pos  (4U)
#define PMUC_AON_LDO_VBAT_LDO_EN_SS_Msk  (0x1UL << PMUC_AON_LDO_VBAT_LDO_EN_SS_Pos)
#define PMUC_AON_LDO_VBAT_LDO_EN_SS     PMUC_AON_LDO_VBAT_LDO_EN_SS_Msk
#define PMUC_AON_LDO_VBAT_LDO_EN_SWMODE_Pos  (5U)
#define PMUC_AON_LDO_VBAT_LDO_EN_SWMODE_Msk  (0x1UL << PMUC_AON_LDO_VBAT_LDO_EN_SWMODE_Pos)
#define PMUC_AON_LDO_VBAT_LDO_EN_SWMODE  PMUC_AON_LDO_VBAT_LDO_EN_SWMODE_Msk
#define PMUC_AON_LDO_VBAT_LDO_SWMODE_Pos  (6U)
#define PMUC_AON_LDO_VBAT_LDO_SWMODE_Msk  (0x1UL << PMUC_AON_LDO_VBAT_LDO_SWMODE_Pos)
#define PMUC_AON_LDO_VBAT_LDO_SWMODE    PMUC_AON_LDO_VBAT_LDO_SWMODE_Msk
#define PMUC_AON_LDO_VBAT_SET_VTHP_Pos  (8U)
#define PMUC_AON_LDO_VBAT_SET_VTHP_Msk  (0x7UL << PMUC_AON_LDO_VBAT_SET_VTHP_Pos)
#define PMUC_AON_LDO_VBAT_SET_VTHP      PMUC_AON_LDO_VBAT_SET_VTHP_Msk
#define PMUC_AON_LDO_VBAT_SET_VTHN_Pos  (11U)
#define PMUC_AON_LDO_VBAT_SET_VTHN_Msk  (0x7UL << PMUC_AON_LDO_VBAT_SET_VTHN_Pos)
#define PMUC_AON_LDO_VBAT_SET_VTHN      PMUC_AON_LDO_VBAT_SET_VTHN_Msk
#define PMUC_AON_LDO_VBAT_RDY_Pos       (14U)
#define PMUC_AON_LDO_VBAT_RDY_Msk       (0x1UL << PMUC_AON_LDO_VBAT_RDY_Pos)
#define PMUC_AON_LDO_VBAT_RDY           PMUC_AON_LDO_VBAT_RDY_Msk
#define PMUC_AON_LDO_VBUS_SET_VTHP_Pos  (16U)
#define PMUC_AON_LDO_VBUS_SET_VTHP_Msk  (0x7UL << PMUC_AON_LDO_VBUS_SET_VTHP_Pos)
#define PMUC_AON_LDO_VBUS_SET_VTHP      PMUC_AON_LDO_VBUS_SET_VTHP_Msk
#define PMUC_AON_LDO_VBUS_SET_VTHN_Pos  (19U)
#define PMUC_AON_LDO_VBUS_SET_VTHN_Msk  (0x7UL << PMUC_AON_LDO_VBUS_SET_VTHN_Pos)
#define PMUC_AON_LDO_VBUS_SET_VTHN      PMUC_AON_LDO_VBUS_SET_VTHN_Msk
#define PMUC_AON_LDO_VBUS_RDY_Pos       (22U)
#define PMUC_AON_LDO_VBUS_RDY_Msk       (0x1UL << PMUC_AON_LDO_VBUS_RDY_Pos)
#define PMUC_AON_LDO_VBUS_RDY           PMUC_AON_LDO_VBUS_RDY_Msk

/***************** Bit definition for PMUC_BUCK_CR1 register ******************/
#define PMUC_BUCK_CR1_EN_Pos            (0U)
#define PMUC_BUCK_CR1_EN_Msk            (0x1UL << PMUC_BUCK_CR1_EN_Pos)
#define PMUC_BUCK_CR1_EN                PMUC_BUCK_CR1_EN_Msk
#define PMUC_BUCK_CR1_CTRL_Pos          (1U)
#define PMUC_BUCK_CR1_CTRL_Msk          (0x1UL << PMUC_BUCK_CR1_CTRL_Pos)
#define PMUC_BUCK_CR1_CTRL              PMUC_BUCK_CR1_CTRL_Msk
#define PMUC_BUCK_CR1_BYPASS_PG_Pos     (2U)
#define PMUC_BUCK_CR1_BYPASS_PG_Msk     (0x1UL << PMUC_BUCK_CR1_BYPASS_PG_Pos)
#define PMUC_BUCK_CR1_BYPASS_PG         PMUC_BUCK_CR1_BYPASS_PG_Msk
#define PMUC_BUCK_CR1_BYPASS_OCP_Pos    (3U)
#define PMUC_BUCK_CR1_BYPASS_OCP_Msk    (0x1UL << PMUC_BUCK_CR1_BYPASS_OCP_Pos)
#define PMUC_BUCK_CR1_BYPASS_OCP        PMUC_BUCK_CR1_BYPASS_OCP_Msk
#define PMUC_BUCK_CR1_BYPASS_UVLO_Pos   (4U)
#define PMUC_BUCK_CR1_BYPASS_UVLO_Msk   (0x1UL << PMUC_BUCK_CR1_BYPASS_UVLO_Pos)
#define PMUC_BUCK_CR1_BYPASS_UVLO       PMUC_BUCK_CR1_BYPASS_UVLO_Msk
#define PMUC_BUCK_CR1_MOT_CTUNE_Pos     (6U)
#define PMUC_BUCK_CR1_MOT_CTUNE_Msk     (0x7UL << PMUC_BUCK_CR1_MOT_CTUNE_Pos)
#define PMUC_BUCK_CR1_MOT_CTUNE         PMUC_BUCK_CR1_MOT_CTUNE_Msk
#define PMUC_BUCK_CR1_COT_CTUNE_Pos     (9U)
#define PMUC_BUCK_CR1_COT_CTUNE_Msk     (0x7UL << PMUC_BUCK_CR1_COT_CTUNE_Pos)
#define PMUC_BUCK_CR1_COT_CTUNE         PMUC_BUCK_CR1_COT_CTUNE_Msk
#define PMUC_BUCK_CR1_COMP_BM_AHI_Pos   (12U)
#define PMUC_BUCK_CR1_COMP_BM_AHI_Msk   (0x1UL << PMUC_BUCK_CR1_COMP_BM_AHI_Pos)
#define PMUC_BUCK_CR1_COMP_BM_AHI       PMUC_BUCK_CR1_COMP_BM_AHI_Msk
#define PMUC_BUCK_CR1_COMP_IQ_TUNE_Pos  (13U)
#define PMUC_BUCK_CR1_COMP_IQ_TUNE_Msk  (0x3UL << PMUC_BUCK_CR1_COMP_IQ_TUNE_Pos)
#define PMUC_BUCK_CR1_COMP_IQ_TUNE      PMUC_BUCK_CR1_COMP_IQ_TUNE_Msk
#define PMUC_BUCK_CR1_COMP_IDYN_TUNE_Pos  (15U)
#define PMUC_BUCK_CR1_COMP_IDYN_TUNE_Msk  (0x3UL << PMUC_BUCK_CR1_COMP_IDYN_TUNE_Pos)
#define PMUC_BUCK_CR1_COMP_IDYN_TUNE    PMUC_BUCK_CR1_COMP_IDYN_TUNE_Msk
#define PMUC_BUCK_CR1_IOCP_TUNE_Pos     (17U)
#define PMUC_BUCK_CR1_IOCP_TUNE_Msk     (0x7UL << PMUC_BUCK_CR1_IOCP_TUNE_Pos)
#define PMUC_BUCK_CR1_IOCP_TUNE         PMUC_BUCK_CR1_IOCP_TUNE_Msk
#define PMUC_BUCK_CR1_SEL_IOCP_HI_Pos   (20U)
#define PMUC_BUCK_CR1_SEL_IOCP_HI_Msk   (0x1UL << PMUC_BUCK_CR1_SEL_IOCP_HI_Pos)
#define PMUC_BUCK_CR1_SEL_IOCP_HI       PMUC_BUCK_CR1_SEL_IOCP_HI_Msk
#define PMUC_BUCK_CR1_SEL_LX22_Pos      (21U)
#define PMUC_BUCK_CR1_SEL_LX22_Msk      (0x1UL << PMUC_BUCK_CR1_SEL_LX22_Pos)
#define PMUC_BUCK_CR1_SEL_LX22          PMUC_BUCK_CR1_SEL_LX22_Msk
#define PMUC_BUCK_CR1_OCP_AON_Pos       (22U)
#define PMUC_BUCK_CR1_OCP_AON_Msk       (0x1UL << PMUC_BUCK_CR1_OCP_AON_Pos)
#define PMUC_BUCK_CR1_OCP_AON           PMUC_BUCK_CR1_OCP_AON_Msk
#define PMUC_BUCK_CR1_ZCD_AON_Pos       (23U)
#define PMUC_BUCK_CR1_ZCD_AON_Msk       (0x1UL << PMUC_BUCK_CR1_ZCD_AON_Pos)
#define PMUC_BUCK_CR1_ZCD_AON           PMUC_BUCK_CR1_ZCD_AON_Msk
#define PMUC_BUCK_CR1_UVLO_X_BIAS_Pos   (24U)
#define PMUC_BUCK_CR1_UVLO_X_BIAS_Msk   (0x1UL << PMUC_BUCK_CR1_UVLO_X_BIAS_Pos)
#define PMUC_BUCK_CR1_UVLO_X_BIAS       PMUC_BUCK_CR1_UVLO_X_BIAS_Msk
#define PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Pos  (25U)
#define PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Msk  (0x7UL << PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Pos)
#define PMUC_BUCK_CR1_BG_BUF_VOS_TRIM   PMUC_BUCK_CR1_BG_BUF_VOS_TRIM_Msk
#define PMUC_BUCK_CR1_BG_BUF_VOS_STEP_Pos  (28U)
#define PMUC_BUCK_CR1_BG_BUF_VOS_STEP_Msk  (0x3UL << PMUC_BUCK_CR1_BG_BUF_VOS_STEP_Pos)
#define PMUC_BUCK_CR1_BG_BUF_VOS_STEP   PMUC_BUCK_CR1_BG_BUF_VOS_STEP_Msk
#define PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Pos  (30U)
#define PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Msk  (0x1UL << PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Pos)
#define PMUC_BUCK_CR1_BG_BUF_VOS_POLAR  PMUC_BUCK_CR1_BG_BUF_VOS_POLAR_Msk
#define PMUC_BUCK_CR1_SS_DONE_Pos       (31U)
#define PMUC_BUCK_CR1_SS_DONE_Msk       (0x1UL << PMUC_BUCK_CR1_SS_DONE_Pos)
#define PMUC_BUCK_CR1_SS_DONE           PMUC_BUCK_CR1_SS_DONE_Msk

/***************** Bit definition for PMUC_BUCK_CR2 register ******************/
#define PMUC_BUCK_CR2_H2M_EN_Pos        (0U)
#define PMUC_BUCK_CR2_H2M_EN_Msk        (0x1UL << PMUC_BUCK_CR2_H2M_EN_Pos)
#define PMUC_BUCK_CR2_H2M_EN            PMUC_BUCK_CR2_H2M_EN_Msk
#define PMUC_BUCK_CR2_H2L_EN_Pos        (1U)
#define PMUC_BUCK_CR2_H2L_EN_Msk        (0x1UL << PMUC_BUCK_CR2_H2L_EN_Pos)
#define PMUC_BUCK_CR2_H2L_EN            PMUC_BUCK_CR2_H2L_EN_Msk
#define PMUC_BUCK_CR2_M2L_EN_Pos        (2U)
#define PMUC_BUCK_CR2_M2L_EN_Msk        (0x1UL << PMUC_BUCK_CR2_M2L_EN_Pos)
#define PMUC_BUCK_CR2_M2L_EN            PMUC_BUCK_CR2_M2L_EN_Msk
#define PMUC_BUCK_CR2_L2M_EN_Pos        (3U)
#define PMUC_BUCK_CR2_L2M_EN_Msk        (0x1UL << PMUC_BUCK_CR2_L2M_EN_Pos)
#define PMUC_BUCK_CR2_L2M_EN            PMUC_BUCK_CR2_L2M_EN_Msk
#define PMUC_BUCK_CR2_M2H_CNT_Pos       (4U)
#define PMUC_BUCK_CR2_M2H_CNT_Msk       (0xFUL << PMUC_BUCK_CR2_M2H_CNT_Pos)
#define PMUC_BUCK_CR2_M2H_CNT           PMUC_BUCK_CR2_M2H_CNT_Msk
#define PMUC_BUCK_CR2_L2H_CNT_Pos       (8U)
#define PMUC_BUCK_CR2_L2H_CNT_Msk       (0xFUL << PMUC_BUCK_CR2_L2H_CNT_Pos)
#define PMUC_BUCK_CR2_L2H_CNT           PMUC_BUCK_CR2_L2H_CNT_Msk
#define PMUC_BUCK_CR2_L2M_CNT_Pos       (12U)
#define PMUC_BUCK_CR2_L2M_CNT_Msk       (0xFUL << PMUC_BUCK_CR2_L2M_CNT_Pos)
#define PMUC_BUCK_CR2_L2M_CNT           PMUC_BUCK_CR2_L2M_CNT_Msk
#define PMUC_BUCK_CR2_FORCE_RDY_Pos     (17U)
#define PMUC_BUCK_CR2_FORCE_RDY_Msk     (0x1UL << PMUC_BUCK_CR2_FORCE_RDY_Pos)
#define PMUC_BUCK_CR2_FORCE_RDY         PMUC_BUCK_CR2_FORCE_RDY_Msk
#define PMUC_BUCK_CR2_SET_VOUT_M_Pos    (18U)
#define PMUC_BUCK_CR2_SET_VOUT_M_Msk    (0x1FUL << PMUC_BUCK_CR2_SET_VOUT_M_Pos)
#define PMUC_BUCK_CR2_SET_VOUT_M        PMUC_BUCK_CR2_SET_VOUT_M_Msk
#define PMUC_BUCK_CR2_SET_VOUT_L_Pos    (23U)
#define PMUC_BUCK_CR2_SET_VOUT_L_Msk    (0x1FUL << PMUC_BUCK_CR2_SET_VOUT_L_Pos)
#define PMUC_BUCK_CR2_SET_VOUT_L        PMUC_BUCK_CR2_SET_VOUT_L_Msk
#define PMUC_BUCK_CR2_TDIS_Pos          (28U)
#define PMUC_BUCK_CR2_TDIS_Msk          (0xFUL << PMUC_BUCK_CR2_TDIS_Pos)
#define PMUC_BUCK_CR2_TDIS              PMUC_BUCK_CR2_TDIS_Msk

/***************** Bit definition for PMUC_BUCK_CR3 register ******************/
#define PMUC_BUCK_CR3_EN_REFTRK_Pos     (0U)
#define PMUC_BUCK_CR3_EN_REFTRK_Msk     (0x1UL << PMUC_BUCK_CR3_EN_REFTRK_Pos)
#define PMUC_BUCK_CR3_EN_REFTRK         PMUC_BUCK_CR3_EN_REFTRK_Msk
#define PMUC_BUCK_CR3_OCP_BM_Pos        (1U)
#define PMUC_BUCK_CR3_OCP_BM_Msk        (0x7UL << PMUC_BUCK_CR3_OCP_BM_Pos)
#define PMUC_BUCK_CR3_OCP_BM            PMUC_BUCK_CR3_OCP_BM_Msk
#define PMUC_BUCK_CR3_PSWDRV_BM_Pos     (4U)
#define PMUC_BUCK_CR3_PSWDRV_BM_Msk     (0x7UL << PMUC_BUCK_CR3_PSWDRV_BM_Pos)
#define PMUC_BUCK_CR3_PSWDRV_BM         PMUC_BUCK_CR3_PSWDRV_BM_Msk

/****************** Bit definition for PMUC_CHG_CR1 register ******************/
#define PMUC_CHG_CR1_EN_Pos             (0U)
#define PMUC_CHG_CR1_EN_Msk             (0x1UL << PMUC_CHG_CR1_EN_Pos)
#define PMUC_CHG_CR1_EN                 PMUC_CHG_CR1_EN_Msk
#define PMUC_CHG_CR1_LOOP_EN_Pos        (1U)
#define PMUC_CHG_CR1_LOOP_EN_Msk        (0x1UL << PMUC_CHG_CR1_LOOP_EN_Pos)
#define PMUC_CHG_CR1_LOOP_EN            PMUC_CHG_CR1_LOOP_EN_Msk
#define PMUC_CHG_CR1_CC_ICTRL_Pos       (2U)
#define PMUC_CHG_CR1_CC_ICTRL_Msk       (0x3FUL << PMUC_CHG_CR1_CC_ICTRL_Pos)
#define PMUC_CHG_CR1_CC_ICTRL           PMUC_CHG_CR1_CC_ICTRL_Msk
#define PMUC_CHG_CR1_CC_VCTRL_Pos       (8U)
#define PMUC_CHG_CR1_CC_VCTRL_Msk       (0x3FUL << PMUC_CHG_CR1_CC_VCTRL_Pos)
#define PMUC_CHG_CR1_CC_VCTRL           PMUC_CHG_CR1_CC_VCTRL_Msk
#define PMUC_CHG_CR1_CC_MP_Pos          (14U)
#define PMUC_CHG_CR1_CC_MP_Msk          (0x1FUL << PMUC_CHG_CR1_CC_MP_Pos)
#define PMUC_CHG_CR1_CC_MP              PMUC_CHG_CR1_CC_MP_Msk
#define PMUC_CHG_CR1_CC_MN_Pos          (19U)
#define PMUC_CHG_CR1_CC_MN_Msk          (0x1FUL << PMUC_CHG_CR1_CC_MN_Pos)
#define PMUC_CHG_CR1_CC_MN              PMUC_CHG_CR1_CC_MN_Msk
#define PMUC_CHG_CR1_CC_RANGE_Pos       (24U)
#define PMUC_CHG_CR1_CC_RANGE_Msk       (0x3UL << PMUC_CHG_CR1_CC_RANGE_Pos)
#define PMUC_CHG_CR1_CC_RANGE           PMUC_CHG_CR1_CC_RANGE_Msk
#define PMUC_CHG_CR1_CV_VCTRL_Pos       (26U)
#define PMUC_CHG_CR1_CV_VCTRL_Msk       (0x3FUL << PMUC_CHG_CR1_CV_VCTRL_Pos)
#define PMUC_CHG_CR1_CV_VCTRL           PMUC_CHG_CR1_CV_VCTRL_Msk

/****************** Bit definition for PMUC_CHG_CR2 register ******************/
#define PMUC_CHG_CR2_BG_PROG_V1P2_Pos   (0U)
#define PMUC_CHG_CR2_BG_PROG_V1P2_Msk   (0xFUL << PMUC_CHG_CR2_BG_PROG_V1P2_Pos)
#define PMUC_CHG_CR2_BG_PROG_V1P2       PMUC_CHG_CR2_BG_PROG_V1P2_Msk
#define PMUC_CHG_CR2_PRECC_RANGE_Pos    (4U)
#define PMUC_CHG_CR2_PRECC_RANGE_Msk    (0x3UL << PMUC_CHG_CR2_PRECC_RANGE_Pos)
#define PMUC_CHG_CR2_PRECC_RANGE        PMUC_CHG_CR2_PRECC_RANGE_Msk
#define PMUC_CHG_CR2_PRECC_ICTRL_Pos    (6U)
#define PMUC_CHG_CR2_PRECC_ICTRL_Msk    (0x3FUL << PMUC_CHG_CR2_PRECC_ICTRL_Pos)
#define PMUC_CHG_CR2_PRECC_ICTRL        PMUC_CHG_CR2_PRECC_ICTRL_Msk
#define PMUC_CHG_CR2_REP_VCTRL_Pos      (12U)
#define PMUC_CHG_CR2_REP_VCTRL_Msk      (0x3FUL << PMUC_CHG_CR2_REP_VCTRL_Pos)
#define PMUC_CHG_CR2_REP_VCTRL          PMUC_CHG_CR2_REP_VCTRL_Msk
#define PMUC_CHG_CR2_HIGH_VCTRL_Pos     (18U)
#define PMUC_CHG_CR2_HIGH_VCTRL_Msk     (0x3FUL << PMUC_CHG_CR2_HIGH_VCTRL_Pos)
#define PMUC_CHG_CR2_HIGH_VCTRL         PMUC_CHG_CR2_HIGH_VCTRL_Msk
#define PMUC_CHG_CR2_BM_EOC_Pos         (24U)
#define PMUC_CHG_CR2_BM_EOC_Msk         (0x7UL << PMUC_CHG_CR2_BM_EOC_Pos)
#define PMUC_CHG_CR2_BM_EOC             PMUC_CHG_CR2_BM_EOC_Msk
#define PMUC_CHG_CR2_RANGE_EOC_Pos      (27U)
#define PMUC_CHG_CR2_RANGE_EOC_Msk      (0x1UL << PMUC_CHG_CR2_RANGE_EOC_Pos)
#define PMUC_CHG_CR2_RANGE_EOC          PMUC_CHG_CR2_RANGE_EOC_Msk
#define PMUC_CHG_CR2_VBAT_RANGE_Pos     (28U)
#define PMUC_CHG_CR2_VBAT_RANGE_Msk     (0xFUL << PMUC_CHG_CR2_VBAT_RANGE_Pos)
#define PMUC_CHG_CR2_VBAT_RANGE         PMUC_CHG_CR2_VBAT_RANGE_Msk

/****************** Bit definition for PMUC_CHG_CR3 register ******************/
#define PMUC_CHG_CR3_DLY1_Pos           (0U)
#define PMUC_CHG_CR3_DLY1_Msk           (0x3FUL << PMUC_CHG_CR3_DLY1_Pos)
#define PMUC_CHG_CR3_DLY1               PMUC_CHG_CR3_DLY1_Msk
#define PMUC_CHG_CR3_DLY2_Pos           (6U)
#define PMUC_CHG_CR3_DLY2_Msk           (0x1FUL << PMUC_CHG_CR3_DLY2_Pos)
#define PMUC_CHG_CR3_DLY2               PMUC_CHG_CR3_DLY2_Msk
#define PMUC_CHG_CR3_FORCE_RST_Pos      (30U)
#define PMUC_CHG_CR3_FORCE_RST_Msk      (0x1UL << PMUC_CHG_CR3_FORCE_RST_Pos)
#define PMUC_CHG_CR3_FORCE_RST          PMUC_CHG_CR3_FORCE_RST_Msk
#define PMUC_CHG_CR3_FORCE_CTRL_Pos     (31U)
#define PMUC_CHG_CR3_FORCE_CTRL_Msk     (0x1UL << PMUC_CHG_CR3_FORCE_CTRL_Pos)
#define PMUC_CHG_CR3_FORCE_CTRL         PMUC_CHG_CR3_FORCE_CTRL_Msk

/****************** Bit definition for PMUC_CHG_CR4 register ******************/
#define PMUC_CHG_CR4_IE_VBUS_RDY_Pos    (0U)
#define PMUC_CHG_CR4_IE_VBUS_RDY_Msk    (0x1UL << PMUC_CHG_CR4_IE_VBUS_RDY_Pos)
#define PMUC_CHG_CR4_IE_VBUS_RDY        PMUC_CHG_CR4_IE_VBUS_RDY_Msk
#define PMUC_CHG_CR4_IE_VBAT_HIGH_Pos   (1U)
#define PMUC_CHG_CR4_IE_VBAT_HIGH_Msk   (0x1UL << PMUC_CHG_CR4_IE_VBAT_HIGH_Pos)
#define PMUC_CHG_CR4_IE_VBAT_HIGH       PMUC_CHG_CR4_IE_VBAT_HIGH_Msk
#define PMUC_CHG_CR4_IE_ABOVE_REP_Pos   (2U)
#define PMUC_CHG_CR4_IE_ABOVE_REP_Msk   (0x1UL << PMUC_CHG_CR4_IE_ABOVE_REP_Pos)
#define PMUC_CHG_CR4_IE_ABOVE_REP       PMUC_CHG_CR4_IE_ABOVE_REP_Msk
#define PMUC_CHG_CR4_IE_ABOVE_CC_Pos    (3U)
#define PMUC_CHG_CR4_IE_ABOVE_CC_Msk    (0x1UL << PMUC_CHG_CR4_IE_ABOVE_CC_Pos)
#define PMUC_CHG_CR4_IE_ABOVE_CC        PMUC_CHG_CR4_IE_ABOVE_CC_Msk
#define PMUC_CHG_CR4_IE_CC_MODE_Pos     (4U)
#define PMUC_CHG_CR4_IE_CC_MODE_Msk     (0x1UL << PMUC_CHG_CR4_IE_CC_MODE_Pos)
#define PMUC_CHG_CR4_IE_CC_MODE         PMUC_CHG_CR4_IE_CC_MODE_Msk
#define PMUC_CHG_CR4_IE_CV_MODE_Pos     (5U)
#define PMUC_CHG_CR4_IE_CV_MODE_Msk     (0x1UL << PMUC_CHG_CR4_IE_CV_MODE_Pos)
#define PMUC_CHG_CR4_IE_CV_MODE         PMUC_CHG_CR4_IE_CV_MODE_Msk
#define PMUC_CHG_CR4_IE_EOC_MODE_Pos    (6U)
#define PMUC_CHG_CR4_IE_EOC_MODE_Msk    (0x1UL << PMUC_CHG_CR4_IE_EOC_MODE_Pos)
#define PMUC_CHG_CR4_IE_EOC_MODE        PMUC_CHG_CR4_IE_EOC_MODE_Msk
#define PMUC_CHG_CR4_IE_EOC_Pos         (7U)
#define PMUC_CHG_CR4_IE_EOC_Msk         (0x1UL << PMUC_CHG_CR4_IE_EOC_Pos)
#define PMUC_CHG_CR4_IE_EOC             PMUC_CHG_CR4_IE_EOC_Msk
#define PMUC_CHG_CR4_IM_VBUS_RDY_Pos    (11U)
#define PMUC_CHG_CR4_IM_VBUS_RDY_Msk    (0x7UL << PMUC_CHG_CR4_IM_VBUS_RDY_Pos)
#define PMUC_CHG_CR4_IM_VBUS_RDY        PMUC_CHG_CR4_IM_VBUS_RDY_Msk
#define PMUC_CHG_CR4_IM_VBAT_HIGH_Pos   (14U)
#define PMUC_CHG_CR4_IM_VBAT_HIGH_Msk   (0x7UL << PMUC_CHG_CR4_IM_VBAT_HIGH_Pos)
#define PMUC_CHG_CR4_IM_VBAT_HIGH       PMUC_CHG_CR4_IM_VBAT_HIGH_Msk
#define PMUC_CHG_CR4_IM_ABOVE_REP_Pos   (17U)
#define PMUC_CHG_CR4_IM_ABOVE_REP_Msk   (0x7UL << PMUC_CHG_CR4_IM_ABOVE_REP_Pos)
#define PMUC_CHG_CR4_IM_ABOVE_REP       PMUC_CHG_CR4_IM_ABOVE_REP_Msk
#define PMUC_CHG_CR4_IM_ABOVE_CC_Pos    (20U)
#define PMUC_CHG_CR4_IM_ABOVE_CC_Msk    (0x7UL << PMUC_CHG_CR4_IM_ABOVE_CC_Pos)
#define PMUC_CHG_CR4_IM_ABOVE_CC        PMUC_CHG_CR4_IM_ABOVE_CC_Msk
#define PMUC_CHG_CR4_IM_CC_MODE_Pos     (23U)
#define PMUC_CHG_CR4_IM_CC_MODE_Msk     (0x7UL << PMUC_CHG_CR4_IM_CC_MODE_Pos)
#define PMUC_CHG_CR4_IM_CC_MODE         PMUC_CHG_CR4_IM_CC_MODE_Msk
#define PMUC_CHG_CR4_IM_CV_MODE_Pos     (26U)
#define PMUC_CHG_CR4_IM_CV_MODE_Msk     (0x7UL << PMUC_CHG_CR4_IM_CV_MODE_Pos)
#define PMUC_CHG_CR4_IM_CV_MODE         PMUC_CHG_CR4_IM_CV_MODE_Msk
#define PMUC_CHG_CR4_IM_EOC_MODE_Pos    (29U)
#define PMUC_CHG_CR4_IM_EOC_MODE_Msk    (0x7UL << PMUC_CHG_CR4_IM_EOC_MODE_Pos)
#define PMUC_CHG_CR4_IM_EOC_MODE        PMUC_CHG_CR4_IM_EOC_MODE_Msk

/****************** Bit definition for PMUC_CHG_CR5 register ******************/
#define PMUC_CHG_CR5_IC_VBUS_RDY_Pos    (0U)
#define PMUC_CHG_CR5_IC_VBUS_RDY_Msk    (0x1UL << PMUC_CHG_CR5_IC_VBUS_RDY_Pos)
#define PMUC_CHG_CR5_IC_VBUS_RDY        PMUC_CHG_CR5_IC_VBUS_RDY_Msk
#define PMUC_CHG_CR5_IC_VBAT_HIGH_Pos   (1U)
#define PMUC_CHG_CR5_IC_VBAT_HIGH_Msk   (0x1UL << PMUC_CHG_CR5_IC_VBAT_HIGH_Pos)
#define PMUC_CHG_CR5_IC_VBAT_HIGH       PMUC_CHG_CR5_IC_VBAT_HIGH_Msk
#define PMUC_CHG_CR5_IC_ABOVE_REP_Pos   (2U)
#define PMUC_CHG_CR5_IC_ABOVE_REP_Msk   (0x1UL << PMUC_CHG_CR5_IC_ABOVE_REP_Pos)
#define PMUC_CHG_CR5_IC_ABOVE_REP       PMUC_CHG_CR5_IC_ABOVE_REP_Msk
#define PMUC_CHG_CR5_IC_ABOVE_CC_Pos    (3U)
#define PMUC_CHG_CR5_IC_ABOVE_CC_Msk    (0x1UL << PMUC_CHG_CR5_IC_ABOVE_CC_Pos)
#define PMUC_CHG_CR5_IC_ABOVE_CC        PMUC_CHG_CR5_IC_ABOVE_CC_Msk
#define PMUC_CHG_CR5_IC_CC_MODE_Pos     (4U)
#define PMUC_CHG_CR5_IC_CC_MODE_Msk     (0x1UL << PMUC_CHG_CR5_IC_CC_MODE_Pos)
#define PMUC_CHG_CR5_IC_CC_MODE         PMUC_CHG_CR5_IC_CC_MODE_Msk
#define PMUC_CHG_CR5_IC_CV_MODE_Pos     (5U)
#define PMUC_CHG_CR5_IC_CV_MODE_Msk     (0x1UL << PMUC_CHG_CR5_IC_CV_MODE_Pos)
#define PMUC_CHG_CR5_IC_CV_MODE         PMUC_CHG_CR5_IC_CV_MODE_Msk
#define PMUC_CHG_CR5_IC_EOC_MODE_Pos    (6U)
#define PMUC_CHG_CR5_IC_EOC_MODE_Msk    (0x1UL << PMUC_CHG_CR5_IC_EOC_MODE_Pos)
#define PMUC_CHG_CR5_IC_EOC_MODE        PMUC_CHG_CR5_IC_EOC_MODE_Msk
#define PMUC_CHG_CR5_IC_EOC_Pos         (7U)
#define PMUC_CHG_CR5_IC_EOC_Msk         (0x1UL << PMUC_CHG_CR5_IC_EOC_Pos)
#define PMUC_CHG_CR5_IC_EOC             PMUC_CHG_CR5_IC_EOC_Msk
#define PMUC_CHG_CR5_IS_VBUS_RDY_Pos    (16U)
#define PMUC_CHG_CR5_IS_VBUS_RDY_Msk    (0x1UL << PMUC_CHG_CR5_IS_VBUS_RDY_Pos)
#define PMUC_CHG_CR5_IS_VBUS_RDY        PMUC_CHG_CR5_IS_VBUS_RDY_Msk
#define PMUC_CHG_CR5_IS_VBAT_HIGH_Pos   (17U)
#define PMUC_CHG_CR5_IS_VBAT_HIGH_Msk   (0x1UL << PMUC_CHG_CR5_IS_VBAT_HIGH_Pos)
#define PMUC_CHG_CR5_IS_VBAT_HIGH       PMUC_CHG_CR5_IS_VBAT_HIGH_Msk
#define PMUC_CHG_CR5_IS_ABOVE_REP_Pos   (18U)
#define PMUC_CHG_CR5_IS_ABOVE_REP_Msk   (0x1UL << PMUC_CHG_CR5_IS_ABOVE_REP_Pos)
#define PMUC_CHG_CR5_IS_ABOVE_REP       PMUC_CHG_CR5_IS_ABOVE_REP_Msk
#define PMUC_CHG_CR5_IS_ABOVE_CC_Pos    (19U)
#define PMUC_CHG_CR5_IS_ABOVE_CC_Msk    (0x1UL << PMUC_CHG_CR5_IS_ABOVE_CC_Pos)
#define PMUC_CHG_CR5_IS_ABOVE_CC        PMUC_CHG_CR5_IS_ABOVE_CC_Msk
#define PMUC_CHG_CR5_IS_CC_MODE_Pos     (20U)
#define PMUC_CHG_CR5_IS_CC_MODE_Msk     (0x1UL << PMUC_CHG_CR5_IS_CC_MODE_Pos)
#define PMUC_CHG_CR5_IS_CC_MODE         PMUC_CHG_CR5_IS_CC_MODE_Msk
#define PMUC_CHG_CR5_IS_CV_MODE_Pos     (21U)
#define PMUC_CHG_CR5_IS_CV_MODE_Msk     (0x1UL << PMUC_CHG_CR5_IS_CV_MODE_Pos)
#define PMUC_CHG_CR5_IS_CV_MODE         PMUC_CHG_CR5_IS_CV_MODE_Msk
#define PMUC_CHG_CR5_IS_EOC_MODE_Pos    (22U)
#define PMUC_CHG_CR5_IS_EOC_MODE_Msk    (0x1UL << PMUC_CHG_CR5_IS_EOC_MODE_Pos)
#define PMUC_CHG_CR5_IS_EOC_MODE        PMUC_CHG_CR5_IS_EOC_MODE_Msk
#define PMUC_CHG_CR5_IS_EOC_Pos         (23U)
#define PMUC_CHG_CR5_IS_EOC_Msk         (0x1UL << PMUC_CHG_CR5_IS_EOC_Pos)
#define PMUC_CHG_CR5_IS_EOC             PMUC_CHG_CR5_IS_EOC_Msk

/****************** Bit definition for PMUC_CHG_SR register *******************/
#define PMUC_CHG_SR_VBUS_RDY_OUT_Pos    (0U)
#define PMUC_CHG_SR_VBUS_RDY_OUT_Msk    (0x1UL << PMUC_CHG_SR_VBUS_RDY_OUT_Pos)
#define PMUC_CHG_SR_VBUS_RDY_OUT        PMUC_CHG_SR_VBUS_RDY_OUT_Msk
#define PMUC_CHG_SR_VBAT_HIGH_OUT_Pos   (1U)
#define PMUC_CHG_SR_VBAT_HIGH_OUT_Msk   (0x1UL << PMUC_CHG_SR_VBAT_HIGH_OUT_Pos)
#define PMUC_CHG_SR_VBAT_HIGH_OUT       PMUC_CHG_SR_VBAT_HIGH_OUT_Msk
#define PMUC_CHG_SR_VBAT_ABOVE_REP_OUT_Pos  (2U)
#define PMUC_CHG_SR_VBAT_ABOVE_REP_OUT_Msk  (0x1UL << PMUC_CHG_SR_VBAT_ABOVE_REP_OUT_Pos)
#define PMUC_CHG_SR_VBAT_ABOVE_REP_OUT  PMUC_CHG_SR_VBAT_ABOVE_REP_OUT_Msk
#define PMUC_CHG_SR_VBAT_ABOVE_CC_OUT_Pos  (3U)
#define PMUC_CHG_SR_VBAT_ABOVE_CC_OUT_Msk  (0x1UL << PMUC_CHG_SR_VBAT_ABOVE_CC_OUT_Pos)
#define PMUC_CHG_SR_VBAT_ABOVE_CC_OUT   PMUC_CHG_SR_VBAT_ABOVE_CC_OUT_Msk
#define PMUC_CHG_SR_CC_MODE_Pos         (4U)
#define PMUC_CHG_SR_CC_MODE_Msk         (0x1UL << PMUC_CHG_SR_CC_MODE_Pos)
#define PMUC_CHG_SR_CC_MODE             PMUC_CHG_SR_CC_MODE_Msk
#define PMUC_CHG_SR_CV_MODE_Pos         (5U)
#define PMUC_CHG_SR_CV_MODE_Msk         (0x1UL << PMUC_CHG_SR_CV_MODE_Pos)
#define PMUC_CHG_SR_CV_MODE             PMUC_CHG_SR_CV_MODE_Msk
#define PMUC_CHG_SR_EOC_MODE_Pos        (6U)
#define PMUC_CHG_SR_EOC_MODE_Msk        (0x1UL << PMUC_CHG_SR_EOC_MODE_Pos)
#define PMUC_CHG_SR_EOC_MODE            PMUC_CHG_SR_EOC_MODE_Msk
#define PMUC_CHG_SR_CHG_STATE_Pos       (8U)
#define PMUC_CHG_SR_CHG_STATE_Msk       (0x7FUL << PMUC_CHG_SR_CHG_STATE_Pos)
#define PMUC_CHG_SR_CHG_STATE           PMUC_CHG_SR_CHG_STATE_Msk

/***************** Bit definition for PMUC_HPSYS_LDO register *****************/
#define PMUC_HPSYS_LDO_EN_Pos           (0U)
#define PMUC_HPSYS_LDO_EN_Msk           (0x1UL << PMUC_HPSYS_LDO_EN_Pos)
#define PMUC_HPSYS_LDO_EN               PMUC_HPSYS_LDO_EN_Msk
#define PMUC_HPSYS_LDO_BP_Pos           (1U)
#define PMUC_HPSYS_LDO_BP_Msk           (0x1UL << PMUC_HPSYS_LDO_BP_Pos)
#define PMUC_HPSYS_LDO_BP               PMUC_HPSYS_LDO_BP_Msk
#define PMUC_HPSYS_LDO_VREF_Pos         (2U)
#define PMUC_HPSYS_LDO_VREF_Msk         (0xFUL << PMUC_HPSYS_LDO_VREF_Pos)
#define PMUC_HPSYS_LDO_VREF             PMUC_HPSYS_LDO_VREF_Msk
#define PMUC_HPSYS_LDO_VREF2_Pos        (6U)
#define PMUC_HPSYS_LDO_VREF2_Msk        (0xFUL << PMUC_HPSYS_LDO_VREF2_Pos)
#define PMUC_HPSYS_LDO_VREF2            PMUC_HPSYS_LDO_VREF2_Msk
#define PMUC_HPSYS_LDO_DLY_Pos          (10U)
#define PMUC_HPSYS_LDO_DLY_Msk          (0x3FUL << PMUC_HPSYS_LDO_DLY_Pos)
#define PMUC_HPSYS_LDO_DLY              PMUC_HPSYS_LDO_DLY_Msk
#define PMUC_HPSYS_LDO_RDY_Pos          (16U)
#define PMUC_HPSYS_LDO_RDY_Msk          (0x1UL << PMUC_HPSYS_LDO_RDY_Pos)
#define PMUC_HPSYS_LDO_RDY              PMUC_HPSYS_LDO_RDY_Msk

/***************** Bit definition for PMUC_LPSYS_LDO register *****************/
#define PMUC_LPSYS_LDO_EN_Pos           (0U)
#define PMUC_LPSYS_LDO_EN_Msk           (0x1UL << PMUC_LPSYS_LDO_EN_Pos)
#define PMUC_LPSYS_LDO_EN               PMUC_LPSYS_LDO_EN_Msk
#define PMUC_LPSYS_LDO_BP_Pos           (1U)
#define PMUC_LPSYS_LDO_BP_Msk           (0x1UL << PMUC_LPSYS_LDO_BP_Pos)
#define PMUC_LPSYS_LDO_BP               PMUC_LPSYS_LDO_BP_Msk
#define PMUC_LPSYS_LDO_VREF_Pos         (2U)
#define PMUC_LPSYS_LDO_VREF_Msk         (0xFUL << PMUC_LPSYS_LDO_VREF_Pos)
#define PMUC_LPSYS_LDO_VREF             PMUC_LPSYS_LDO_VREF_Msk
#define PMUC_LPSYS_LDO_VREF2_Pos        (6U)
#define PMUC_LPSYS_LDO_VREF2_Msk        (0xFUL << PMUC_LPSYS_LDO_VREF2_Pos)
#define PMUC_LPSYS_LDO_VREF2            PMUC_LPSYS_LDO_VREF2_Msk
#define PMUC_LPSYS_LDO_DLY_Pos          (10U)
#define PMUC_LPSYS_LDO_DLY_Msk          (0x3FUL << PMUC_LPSYS_LDO_DLY_Pos)
#define PMUC_LPSYS_LDO_DLY              PMUC_LPSYS_LDO_DLY_Msk
#define PMUC_LPSYS_LDO_RDY_Pos          (16U)
#define PMUC_LPSYS_LDO_RDY_Msk          (0x1UL << PMUC_LPSYS_LDO_RDY_Pos)
#define PMUC_LPSYS_LDO_RDY              PMUC_LPSYS_LDO_RDY_Msk

/***************** Bit definition for PMUC_HPSYS_SWR register *****************/
#define PMUC_HPSYS_SWR_PSW_Pos          (0U)
#define PMUC_HPSYS_SWR_PSW_Msk          (0x3UL << PMUC_HPSYS_SWR_PSW_Pos)
#define PMUC_HPSYS_SWR_PSW              PMUC_HPSYS_SWR_PSW_Msk
#define PMUC_HPSYS_SWR_PSW_RET_Pos      (2U)
#define PMUC_HPSYS_SWR_PSW_RET_Msk      (0x3UL << PMUC_HPSYS_SWR_PSW_RET_Pos)
#define PMUC_HPSYS_SWR_PSW_RET          PMUC_HPSYS_SWR_PSW_RET_Msk
#define PMUC_HPSYS_SWR_DLY_Pos          (4U)
#define PMUC_HPSYS_SWR_DLY_Msk          (0x7UL << PMUC_HPSYS_SWR_DLY_Pos)
#define PMUC_HPSYS_SWR_DLY              PMUC_HPSYS_SWR_DLY_Msk
#define PMUC_HPSYS_SWR_NORET_Pos        (7U)
#define PMUC_HPSYS_SWR_NORET_Msk        (0x1UL << PMUC_HPSYS_SWR_NORET_Pos)
#define PMUC_HPSYS_SWR_NORET            PMUC_HPSYS_SWR_NORET_Msk
#define PMUC_HPSYS_SWR_RDY_Pos          (31U)
#define PMUC_HPSYS_SWR_RDY_Msk          (0x1UL << PMUC_HPSYS_SWR_RDY_Pos)
#define PMUC_HPSYS_SWR_RDY              PMUC_HPSYS_SWR_RDY_Msk

/***************** Bit definition for PMUC_LPSYS_SWR register *****************/
#define PMUC_LPSYS_SWR_PSW_Pos          (0U)
#define PMUC_LPSYS_SWR_PSW_Msk          (0x3UL << PMUC_LPSYS_SWR_PSW_Pos)
#define PMUC_LPSYS_SWR_PSW              PMUC_LPSYS_SWR_PSW_Msk
#define PMUC_LPSYS_SWR_PSW_RET_Pos      (2U)
#define PMUC_LPSYS_SWR_PSW_RET_Msk      (0x3UL << PMUC_LPSYS_SWR_PSW_RET_Pos)
#define PMUC_LPSYS_SWR_PSW_RET          PMUC_LPSYS_SWR_PSW_RET_Msk
#define PMUC_LPSYS_SWR_DLY_Pos          (4U)
#define PMUC_LPSYS_SWR_DLY_Msk          (0x7UL << PMUC_LPSYS_SWR_DLY_Pos)
#define PMUC_LPSYS_SWR_DLY              PMUC_LPSYS_SWR_DLY_Msk
#define PMUC_LPSYS_SWR_NORET_Pos        (7U)
#define PMUC_LPSYS_SWR_NORET_Msk        (0x1UL << PMUC_LPSYS_SWR_NORET_Pos)
#define PMUC_LPSYS_SWR_NORET            PMUC_LPSYS_SWR_NORET_Msk
#define PMUC_LPSYS_SWR_RDY_Pos          (31U)
#define PMUC_LPSYS_SWR_RDY_Msk          (0x1UL << PMUC_LPSYS_SWR_RDY_Pos)
#define PMUC_LPSYS_SWR_RDY              PMUC_LPSYS_SWR_RDY_Msk

/***************** Bit definition for PMUC_PERI_LDO register ******************/
#define PMUC_PERI_LDO_EN_LDO18_Pos      (0U)
#define PMUC_PERI_LDO_EN_LDO18_Msk      (0x1UL << PMUC_PERI_LDO_EN_LDO18_Pos)
#define PMUC_PERI_LDO_EN_LDO18          PMUC_PERI_LDO_EN_LDO18_Msk
#define PMUC_PERI_LDO_LDO18_EN_SS_Pos   (1U)
#define PMUC_PERI_LDO_LDO18_EN_SS_Msk   (0x1UL << PMUC_PERI_LDO_LDO18_EN_SS_Pos)
#define PMUC_PERI_LDO_LDO18_EN_SS       PMUC_PERI_LDO_LDO18_EN_SS_Msk
#define PMUC_PERI_LDO_LDO18_PD_VOUT_Pos  (2U)
#define PMUC_PERI_LDO_LDO18_PD_VOUT_Msk  (0x1UL << PMUC_PERI_LDO_LDO18_PD_VOUT_Pos)
#define PMUC_PERI_LDO_LDO18_PD_VOUT     PMUC_PERI_LDO_LDO18_PD_VOUT_Msk
#define PMUC_PERI_LDO_LDO18_VREF_SEL_Pos  (3U)
#define PMUC_PERI_LDO_LDO18_VREF_SEL_Msk  (0xFUL << PMUC_PERI_LDO_LDO18_VREF_SEL_Pos)
#define PMUC_PERI_LDO_LDO18_VREF_SEL    PMUC_PERI_LDO_LDO18_VREF_SEL_Msk
#define PMUC_PERI_LDO_LDO18_RDY_Pos     (7U)
#define PMUC_PERI_LDO_LDO18_RDY_Msk     (0x1UL << PMUC_PERI_LDO_LDO18_RDY_Pos)
#define PMUC_PERI_LDO_LDO18_RDY         PMUC_PERI_LDO_LDO18_RDY_Msk
#define PMUC_PERI_LDO_EN_VDD33_LDO2_Pos  (8U)
#define PMUC_PERI_LDO_EN_VDD33_LDO2_Msk  (0x1UL << PMUC_PERI_LDO_EN_VDD33_LDO2_Pos)
#define PMUC_PERI_LDO_EN_VDD33_LDO2     PMUC_PERI_LDO_EN_VDD33_LDO2_Msk
#define PMUC_PERI_LDO_VDD33_LDO2_EN_SS_Pos  (9U)
#define PMUC_PERI_LDO_VDD33_LDO2_EN_SS_Msk  (0x1UL << PMUC_PERI_LDO_VDD33_LDO2_EN_SS_Pos)
#define PMUC_PERI_LDO_VDD33_LDO2_EN_SS  PMUC_PERI_LDO_VDD33_LDO2_EN_SS_Msk
#define PMUC_PERI_LDO_VDD33_LDO2_PD_VOUT_Pos  (10U)
#define PMUC_PERI_LDO_VDD33_LDO2_PD_VOUT_Msk  (0x1UL << PMUC_PERI_LDO_VDD33_LDO2_PD_VOUT_Pos)
#define PMUC_PERI_LDO_VDD33_LDO2_PD_VOUT  PMUC_PERI_LDO_VDD33_LDO2_PD_VOUT_Msk
#define PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Pos  (11U)
#define PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk  (0xFUL << PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Pos)
#define PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT  PMUC_PERI_LDO_VDD33_LDO2_SET_VOUT_Msk
#define PMUC_PERI_LDO_EN_VDD33_LDO3_Pos  (16U)
#define PMUC_PERI_LDO_EN_VDD33_LDO3_Msk  (0x1UL << PMUC_PERI_LDO_EN_VDD33_LDO3_Pos)
#define PMUC_PERI_LDO_EN_VDD33_LDO3     PMUC_PERI_LDO_EN_VDD33_LDO3_Msk
#define PMUC_PERI_LDO_VDD33_LDO3_EN_SS_Pos  (17U)
#define PMUC_PERI_LDO_VDD33_LDO3_EN_SS_Msk  (0x1UL << PMUC_PERI_LDO_VDD33_LDO3_EN_SS_Pos)
#define PMUC_PERI_LDO_VDD33_LDO3_EN_SS  PMUC_PERI_LDO_VDD33_LDO3_EN_SS_Msk
#define PMUC_PERI_LDO_VDD33_LDO3_PD_VOUT_Pos  (18U)
#define PMUC_PERI_LDO_VDD33_LDO3_PD_VOUT_Msk  (0x1UL << PMUC_PERI_LDO_VDD33_LDO3_PD_VOUT_Pos)
#define PMUC_PERI_LDO_VDD33_LDO3_PD_VOUT  PMUC_PERI_LDO_VDD33_LDO3_PD_VOUT_Msk
#define PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Pos  (19U)
#define PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk  (0xFUL << PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Pos)
#define PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT  PMUC_PERI_LDO_VDD33_LDO3_SET_VOUT_Msk

/****************** Bit definition for PMUC_PMU_TR register *******************/
#define PMUC_PMU_TR_PMU_DC_TR_Pos       (0U)
#define PMUC_PMU_TR_PMU_DC_TR_Msk       (0x7UL << PMUC_PMU_TR_PMU_DC_TR_Pos)
#define PMUC_PMU_TR_PMU_DC_TR           PMUC_PMU_TR_PMU_DC_TR_Msk
#define PMUC_PMU_TR_PMU_DC_MR_Pos       (3U)
#define PMUC_PMU_TR_PMU_DC_MR_Msk       (0x7UL << PMUC_PMU_TR_PMU_DC_MR_Pos)
#define PMUC_PMU_TR_PMU_DC_MR           PMUC_PMU_TR_PMU_DC_MR_Msk

/***************** Bit definition for PMUC_PMU_RSVD register ******************/
#define PMUC_PMU_RSVD_RESERVE0_Pos      (0U)
#define PMUC_PMU_RSVD_RESERVE0_Msk      (0xFFUL << PMUC_PMU_RSVD_RESERVE0_Pos)
#define PMUC_PMU_RSVD_RESERVE0          PMUC_PMU_RSVD_RESERVE0_Msk
#define PMUC_PMU_RSVD_RESERVE1_Pos      (8U)
#define PMUC_PMU_RSVD_RESERVE1_Msk      (0xFFUL << PMUC_PMU_RSVD_RESERVE1_Pos)
#define PMUC_PMU_RSVD_RESERVE1          PMUC_PMU_RSVD_RESERVE1_Msk
#define PMUC_PMU_RSVD_RESERVE2_Pos      (16U)
#define PMUC_PMU_RSVD_RESERVE2_Msk      (0xFFUL << PMUC_PMU_RSVD_RESERVE2_Pos)
#define PMUC_PMU_RSVD_RESERVE2          PMUC_PMU_RSVD_RESERVE2_Msk
#define PMUC_PMU_RSVD_RESERVE3_Pos      (24U)
#define PMUC_PMU_RSVD_RESERVE3_Msk      (0xFFUL << PMUC_PMU_RSVD_RESERVE3_Pos)
#define PMUC_PMU_RSVD_RESERVE3          PMUC_PMU_RSVD_RESERVE3_Msk

/****************** Bit definition for PMUC_HXT_CR1 register ******************/
#define PMUC_HXT_CR1_EN_Pos             (0U)
#define PMUC_HXT_CR1_EN_Msk             (0x1UL << PMUC_HXT_CR1_EN_Pos)
#define PMUC_HXT_CR1_EN                 PMUC_HXT_CR1_EN_Msk
#define PMUC_HXT_CR1_BUF_EN_Pos         (1U)
#define PMUC_HXT_CR1_BUF_EN_Msk         (0x1UL << PMUC_HXT_CR1_BUF_EN_Pos)
#define PMUC_HXT_CR1_BUF_EN             PMUC_HXT_CR1_BUF_EN_Msk
#define PMUC_HXT_CR1_BUF_DIG_EN_Pos     (2U)
#define PMUC_HXT_CR1_BUF_DIG_EN_Msk     (0x1UL << PMUC_HXT_CR1_BUF_DIG_EN_Pos)
#define PMUC_HXT_CR1_BUF_DIG_EN         PMUC_HXT_CR1_BUF_DIG_EN_Msk
#define PMUC_HXT_CR1_BUF_DIG_STR_Pos    (3U)
#define PMUC_HXT_CR1_BUF_DIG_STR_Msk    (0x3UL << PMUC_HXT_CR1_BUF_DIG_STR_Pos)
#define PMUC_HXT_CR1_BUF_DIG_STR        PMUC_HXT_CR1_BUF_DIG_STR_Msk
#define PMUC_HXT_CR1_BUF_DLL_EN_Pos     (5U)
#define PMUC_HXT_CR1_BUF_DLL_EN_Msk     (0x1UL << PMUC_HXT_CR1_BUF_DLL_EN_Pos)
#define PMUC_HXT_CR1_BUF_DLL_EN         PMUC_HXT_CR1_BUF_DLL_EN_Msk
#define PMUC_HXT_CR1_BUF_DLL_STR_Pos    (6U)
#define PMUC_HXT_CR1_BUF_DLL_STR_Msk    (0x3UL << PMUC_HXT_CR1_BUF_DLL_STR_Pos)
#define PMUC_HXT_CR1_BUF_DLL_STR        PMUC_HXT_CR1_BUF_DLL_STR_Msk
#define PMUC_HXT_CR1_BUF_AUD_EN_Pos     (8U)
#define PMUC_HXT_CR1_BUF_AUD_EN_Msk     (0x1UL << PMUC_HXT_CR1_BUF_AUD_EN_Pos)
#define PMUC_HXT_CR1_BUF_AUD_EN         PMUC_HXT_CR1_BUF_AUD_EN_Msk
#define PMUC_HXT_CR1_BUF_AUD_STR_Pos    (9U)
#define PMUC_HXT_CR1_BUF_AUD_STR_Msk    (0x3UL << PMUC_HXT_CR1_BUF_AUD_STR_Pos)
#define PMUC_HXT_CR1_BUF_AUD_STR        PMUC_HXT_CR1_BUF_AUD_STR_Msk
#define PMUC_HXT_CR1_BUF_RF_STR_Pos     (11U)
#define PMUC_HXT_CR1_BUF_RF_STR_Msk     (0x3UL << PMUC_HXT_CR1_BUF_RF_STR_Pos)
#define PMUC_HXT_CR1_BUF_RF_STR         PMUC_HXT_CR1_BUF_RF_STR_Msk
#define PMUC_HXT_CR1_LDO_VREF_Pos       (13U)
#define PMUC_HXT_CR1_LDO_VREF_Msk       (0xFUL << PMUC_HXT_CR1_LDO_VREF_Pos)
#define PMUC_HXT_CR1_LDO_VREF           PMUC_HXT_CR1_LDO_VREF_Msk
#define PMUC_HXT_CR1_LDO_FLT_RSEL_Pos   (17U)
#define PMUC_HXT_CR1_LDO_FLT_RSEL_Msk   (0x3UL << PMUC_HXT_CR1_LDO_FLT_RSEL_Pos)
#define PMUC_HXT_CR1_LDO_FLT_RSEL       PMUC_HXT_CR1_LDO_FLT_RSEL_Msk
#define PMUC_HXT_CR1_GM_EN_Pos          (19U)
#define PMUC_HXT_CR1_GM_EN_Msk          (0x1UL << PMUC_HXT_CR1_GM_EN_Pos)
#define PMUC_HXT_CR1_GM_EN              PMUC_HXT_CR1_GM_EN_Msk
#define PMUC_HXT_CR1_CBANK_SEL_Pos      (20U)
#define PMUC_HXT_CR1_CBANK_SEL_Msk      (0x3FFUL << PMUC_HXT_CR1_CBANK_SEL_Pos)
#define PMUC_HXT_CR1_CBANK_SEL          PMUC_HXT_CR1_CBANK_SEL_Msk

/****************** Bit definition for PMUC_HXT_CR2 register ******************/
#define PMUC_HXT_CR2_AGC_EN_Pos         (0U)
#define PMUC_HXT_CR2_AGC_EN_Msk         (0x1UL << PMUC_HXT_CR2_AGC_EN_Pos)
#define PMUC_HXT_CR2_AGC_EN             PMUC_HXT_CR2_AGC_EN_Msk
#define PMUC_HXT_CR2_AGC_ISTART_SEL_Pos  (1U)
#define PMUC_HXT_CR2_AGC_ISTART_SEL_Msk  (0x1UL << PMUC_HXT_CR2_AGC_ISTART_SEL_Pos)
#define PMUC_HXT_CR2_AGC_ISTART_SEL     PMUC_HXT_CR2_AGC_ISTART_SEL_Msk
#define PMUC_HXT_CR2_AGC_VTH_Pos        (2U)
#define PMUC_HXT_CR2_AGC_VTH_Msk        (0xFUL << PMUC_HXT_CR2_AGC_VTH_Pos)
#define PMUC_HXT_CR2_AGC_VTH            PMUC_HXT_CR2_AGC_VTH_Msk
#define PMUC_HXT_CR2_AGC_VINDC_Pos      (6U)
#define PMUC_HXT_CR2_AGC_VINDC_Msk      (0x3UL << PMUC_HXT_CR2_AGC_VINDC_Pos)
#define PMUC_HXT_CR2_AGC_VINDC          PMUC_HXT_CR2_AGC_VINDC_Msk
#define PMUC_HXT_CR2_ACBUF_SEL_Pos      (8U)
#define PMUC_HXT_CR2_ACBUF_SEL_Msk      (0x3UL << PMUC_HXT_CR2_ACBUF_SEL_Pos)
#define PMUC_HXT_CR2_ACBUF_SEL          PMUC_HXT_CR2_ACBUF_SEL_Msk
#define PMUC_HXT_CR2_ACBUF_RSEL_Pos     (10U)
#define PMUC_HXT_CR2_ACBUF_RSEL_Msk     (0x1UL << PMUC_HXT_CR2_ACBUF_RSEL_Pos)
#define PMUC_HXT_CR2_ACBUF_RSEL         PMUC_HXT_CR2_ACBUF_RSEL_Msk
#define PMUC_HXT_CR2_BUF_SEL2_Pos       (11U)
#define PMUC_HXT_CR2_BUF_SEL2_Msk       (0x3UL << PMUC_HXT_CR2_BUF_SEL2_Pos)
#define PMUC_HXT_CR2_BUF_SEL2           PMUC_HXT_CR2_BUF_SEL2_Msk
#define PMUC_HXT_CR2_BUF_SEL3_Pos       (13U)
#define PMUC_HXT_CR2_BUF_SEL3_Msk       (0x3UL << PMUC_HXT_CR2_BUF_SEL3_Pos)
#define PMUC_HXT_CR2_BUF_SEL3           PMUC_HXT_CR2_BUF_SEL3_Msk
#define PMUC_HXT_CR2_IDAC_EN_Pos        (15U)
#define PMUC_HXT_CR2_IDAC_EN_Msk        (0x1UL << PMUC_HXT_CR2_IDAC_EN_Pos)
#define PMUC_HXT_CR2_IDAC_EN            PMUC_HXT_CR2_IDAC_EN_Msk
#define PMUC_HXT_CR2_IDAC_Pos           (16U)
#define PMUC_HXT_CR2_IDAC_Msk           (0x3FFUL << PMUC_HXT_CR2_IDAC_Pos)
#define PMUC_HXT_CR2_IDAC               PMUC_HXT_CR2_IDAC_Msk
#define PMUC_HXT_CR2_SDADC_CLKIN_EN_Pos  (26U)
#define PMUC_HXT_CR2_SDADC_CLKIN_EN_Msk  (0x1UL << PMUC_HXT_CR2_SDADC_CLKIN_EN_Pos)
#define PMUC_HXT_CR2_SDADC_CLKIN_EN     PMUC_HXT_CR2_SDADC_CLKIN_EN_Msk
#define PMUC_HXT_CR2_SDADC_CLKDIV1_SEL_Pos  (27U)
#define PMUC_HXT_CR2_SDADC_CLKDIV1_SEL_Msk  (0x3UL << PMUC_HXT_CR2_SDADC_CLKDIV1_SEL_Pos)
#define PMUC_HXT_CR2_SDADC_CLKDIV1_SEL  PMUC_HXT_CR2_SDADC_CLKDIV1_SEL_Msk
#define PMUC_HXT_CR2_SDADC_CLKDIV2_SEL_Pos  (29U)
#define PMUC_HXT_CR2_SDADC_CLKDIV2_SEL_Msk  (0x3UL << PMUC_HXT_CR2_SDADC_CLKDIV2_SEL_Pos)
#define PMUC_HXT_CR2_SDADC_CLKDIV2_SEL  PMUC_HXT_CR2_SDADC_CLKDIV2_SEL_Msk
#define PMUC_HXT_CR2_SLEEP_EN_Pos       (31U)
#define PMUC_HXT_CR2_SLEEP_EN_Msk       (0x1UL << PMUC_HXT_CR2_SLEEP_EN_Pos)
#define PMUC_HXT_CR2_SLEEP_EN           PMUC_HXT_CR2_SLEEP_EN_Msk

/****************** Bit definition for PMUC_HXT_CR3 register ******************/
#define PMUC_HXT_CR3_BUF_DAC_STR_Pos    (0U)
#define PMUC_HXT_CR3_BUF_DAC_STR_Msk    (0x3UL << PMUC_HXT_CR3_BUF_DAC_STR_Pos)
#define PMUC_HXT_CR3_BUF_DAC_STR        PMUC_HXT_CR3_BUF_DAC_STR_Msk
#define PMUC_HXT_CR3_BUF_OSLO_STR_Pos   (2U)
#define PMUC_HXT_CR3_BUF_OSLO_STR_Msk   (0x3UL << PMUC_HXT_CR3_BUF_OSLO_STR_Pos)
#define PMUC_HXT_CR3_BUF_OSLO_STR       PMUC_HXT_CR3_BUF_OSLO_STR_Msk
#define PMUC_HXT_CR3_DLY_Pos            (4U)
#define PMUC_HXT_CR3_DLY_Msk            (0x3FUL << PMUC_HXT_CR3_DLY_Pos)
#define PMUC_HXT_CR3_DLY                PMUC_HXT_CR3_DLY_Msk

/****************** Bit definition for PMUC_HRC_CR1 register ******************/
#define PMUC_HRC_CR1_EN_Pos             (0U)
#define PMUC_HRC_CR1_EN_Msk             (0x1UL << PMUC_HRC_CR1_EN_Pos)
#define PMUC_HRC_CR1_EN                 PMUC_HRC_CR1_EN_Msk
#define PMUC_HRC_CR1_LDO_VREF_Pos       (1U)
#define PMUC_HRC_CR1_LDO_VREF_Msk       (0xFUL << PMUC_HRC_CR1_LDO_VREF_Pos)
#define PMUC_HRC_CR1_LDO_VREF           PMUC_HRC_CR1_LDO_VREF_Msk
#define PMUC_HRC_CR1_FREQ_TRIM_Pos      (5U)
#define PMUC_HRC_CR1_FREQ_TRIM_Msk      (0x7FFUL << PMUC_HRC_CR1_FREQ_TRIM_Pos)
#define PMUC_HRC_CR1_FREQ_TRIM          PMUC_HRC_CR1_FREQ_TRIM_Msk
#define PMUC_HRC_CR1_TEMP_TRIM_Pos      (16U)
#define PMUC_HRC_CR1_TEMP_TRIM_Msk      (0x7UL << PMUC_HRC_CR1_TEMP_TRIM_Pos)
#define PMUC_HRC_CR1_TEMP_TRIM          PMUC_HRC_CR1_TEMP_TRIM_Msk
#define PMUC_HRC_CR1_CLK96M_EN_Pos      (19U)
#define PMUC_HRC_CR1_CLK96M_EN_Msk      (0x1UL << PMUC_HRC_CR1_CLK96M_EN_Pos)
#define PMUC_HRC_CR1_CLK96M_EN          PMUC_HRC_CR1_CLK96M_EN_Msk
#define PMUC_HRC_CR1_CLKHP_EN_Pos       (20U)
#define PMUC_HRC_CR1_CLKHP_EN_Msk       (0x1UL << PMUC_HRC_CR1_CLKHP_EN_Pos)
#define PMUC_HRC_CR1_CLKHP_EN           PMUC_HRC_CR1_CLKHP_EN_Msk
#define PMUC_HRC_CR1_CLKHP_SEL_Pos      (21U)
#define PMUC_HRC_CR1_CLKHP_SEL_Msk      (0x3UL << PMUC_HRC_CR1_CLKHP_SEL_Pos)
#define PMUC_HRC_CR1_CLKHP_SEL          PMUC_HRC_CR1_CLKHP_SEL_Msk
#define PMUC_HRC_CR1_CLKHP_STR_Pos      (23U)
#define PMUC_HRC_CR1_CLKHP_STR_Msk      (0x3UL << PMUC_HRC_CR1_CLKHP_STR_Pos)
#define PMUC_HRC_CR1_CLKHP_STR          PMUC_HRC_CR1_CLKHP_STR_Msk
#define PMUC_HRC_CR1_CLKLP_EN_Pos       (25U)
#define PMUC_HRC_CR1_CLKLP_EN_Msk       (0x1UL << PMUC_HRC_CR1_CLKLP_EN_Pos)
#define PMUC_HRC_CR1_CLKLP_EN           PMUC_HRC_CR1_CLKLP_EN_Msk
#define PMUC_HRC_CR1_CLKLP_SEL_Pos      (26U)
#define PMUC_HRC_CR1_CLKLP_SEL_Msk      (0x3UL << PMUC_HRC_CR1_CLKLP_SEL_Pos)
#define PMUC_HRC_CR1_CLKLP_SEL          PMUC_HRC_CR1_CLKLP_SEL_Msk
#define PMUC_HRC_CR1_CLKLP_STR_Pos      (28U)
#define PMUC_HRC_CR1_CLKLP_STR_Msk      (0x3UL << PMUC_HRC_CR1_CLKLP_STR_Pos)
#define PMUC_HRC_CR1_CLKLP_STR          PMUC_HRC_CR1_CLKLP_STR_Msk
#define PMUC_HRC_CR1_DLY_Pos            (31U)
#define PMUC_HRC_CR1_DLY_Msk            (0x1UL << PMUC_HRC_CR1_DLY_Pos)
#define PMUC_HRC_CR1_DLY                PMUC_HRC_CR1_DLY_Msk

/****************** Bit definition for PMUC_HRC_CR2 register ******************/
#define PMUC_HRC_CR2_CLKX2_EN_Pos       (0U)
#define PMUC_HRC_CR2_CLKX2_EN_Msk       (0x1UL << PMUC_HRC_CR2_CLKX2_EN_Pos)
#define PMUC_HRC_CR2_CLKX2_EN           PMUC_HRC_CR2_CLKX2_EN_Msk
#define PMUC_HRC_CR2_IDWN_SEL_Pos       (1U)
#define PMUC_HRC_CR2_IDWN_SEL_Msk       (0x3UL << PMUC_HRC_CR2_IDWN_SEL_Pos)
#define PMUC_HRC_CR2_IDWN_SEL           PMUC_HRC_CR2_IDWN_SEL_Msk
#define PMUC_HRC_CR2_SS_EN_Pos          (3U)
#define PMUC_HRC_CR2_SS_EN_Msk          (0x1UL << PMUC_HRC_CR2_SS_EN_Pos)
#define PMUC_HRC_CR2_SS_EN              PMUC_HRC_CR2_SS_EN_Msk
#define PMUC_HRC_CR2_SS_CLK_SEL_Pos     (4U)
#define PMUC_HRC_CR2_SS_CLK_SEL_Msk     (0x3UL << PMUC_HRC_CR2_SS_CLK_SEL_Pos)
#define PMUC_HRC_CR2_SS_CLK_SEL         PMUC_HRC_CR2_SS_CLK_SEL_Msk
#define PMUC_HRC_CR2_SS_NSEL_Pos        (6U)
#define PMUC_HRC_CR2_SS_NSEL_Msk        (0x1UL << PMUC_HRC_CR2_SS_NSEL_Pos)
#define PMUC_HRC_CR2_SS_NSEL            PMUC_HRC_CR2_SS_NSEL_Msk
#define PMUC_HRC_CR2_SS_AVG0_EN_Pos     (7U)
#define PMUC_HRC_CR2_SS_AVG0_EN_Msk     (0x1UL << PMUC_HRC_CR2_SS_AVG0_EN_Pos)
#define PMUC_HRC_CR2_SS_AVG0_EN         PMUC_HRC_CR2_SS_AVG0_EN_Msk
#define PMUC_HRC_CR2_SS_FD_SEL_FRC_EN_Pos  (8U)
#define PMUC_HRC_CR2_SS_FD_SEL_FRC_EN_Msk  (0x1UL << PMUC_HRC_CR2_SS_FD_SEL_FRC_EN_Pos)
#define PMUC_HRC_CR2_SS_FD_SEL_FRC_EN   PMUC_HRC_CR2_SS_FD_SEL_FRC_EN_Msk
#define PMUC_HRC_CR2_SS_FD_SEL_Pos      (9U)
#define PMUC_HRC_CR2_SS_FD_SEL_Msk      (0x7UL << PMUC_HRC_CR2_SS_FD_SEL_Pos)
#define PMUC_HRC_CR2_SS_FD_SEL          PMUC_HRC_CR2_SS_FD_SEL_Msk

/****************** Bit definition for PMUC_CAU_BGR register ******************/
#define PMUC_CAU_BGR_HPBG_VDDPSW_EN_Pos  (0U)
#define PMUC_CAU_BGR_HPBG_VDDPSW_EN_Msk  (0x1UL << PMUC_CAU_BGR_HPBG_VDDPSW_EN_Pos)
#define PMUC_CAU_BGR_HPBG_VDDPSW_EN     PMUC_CAU_BGR_HPBG_VDDPSW_EN_Msk
#define PMUC_CAU_BGR_HPBG_EN_Pos        (1U)
#define PMUC_CAU_BGR_HPBG_EN_Msk        (0x1UL << PMUC_CAU_BGR_HPBG_EN_Pos)
#define PMUC_CAU_BGR_HPBG_EN            PMUC_CAU_BGR_HPBG_EN_Msk
#define PMUC_CAU_BGR_LPBG_EN_Pos        (2U)
#define PMUC_CAU_BGR_LPBG_EN_Msk        (0x1UL << PMUC_CAU_BGR_LPBG_EN_Pos)
#define PMUC_CAU_BGR_LPBG_EN            PMUC_CAU_BGR_LPBG_EN_Msk
#define PMUC_CAU_BGR_LPBG_VREF06_Pos    (3U)
#define PMUC_CAU_BGR_LPBG_VREF06_Msk    (0xFUL << PMUC_CAU_BGR_LPBG_VREF06_Pos)
#define PMUC_CAU_BGR_LPBG_VREF06        PMUC_CAU_BGR_LPBG_VREF06_Msk
#define PMUC_CAU_BGR_LPBG_VREF12_Pos    (7U)
#define PMUC_CAU_BGR_LPBG_VREF12_Msk    (0xFUL << PMUC_CAU_BGR_LPBG_VREF12_Pos)
#define PMUC_CAU_BGR_LPBG_VREF12        PMUC_CAU_BGR_LPBG_VREF12_Msk

/****************** Bit definition for PMUC_CAU_TR register *******************/
#define PMUC_CAU_TR_CAU_DC_TR_Pos       (0U)
#define PMUC_CAU_TR_CAU_DC_TR_Msk       (0x7UL << PMUC_CAU_TR_CAU_DC_TR_Pos)
#define PMUC_CAU_TR_CAU_DC_TR           PMUC_CAU_TR_CAU_DC_TR_Msk
#define PMUC_CAU_TR_CAU_DC_BR_Pos       (3U)
#define PMUC_CAU_TR_CAU_DC_BR_Msk       (0x7UL << PMUC_CAU_TR_CAU_DC_BR_Pos)
#define PMUC_CAU_TR_CAU_DC_BR           PMUC_CAU_TR_CAU_DC_BR_Msk
#define PMUC_CAU_TR_CAU_DC_MR_Pos       (6U)
#define PMUC_CAU_TR_CAU_DC_MR_Msk       (0x7UL << PMUC_CAU_TR_CAU_DC_MR_Pos)
#define PMUC_CAU_TR_CAU_DC_MR           PMUC_CAU_TR_CAU_DC_MR_Msk

/***************** Bit definition for PMUC_CAU_RSVD register ******************/
#define PMUC_CAU_RSVD_RESERVE0_Pos      (0U)
#define PMUC_CAU_RSVD_RESERVE0_Msk      (0xFFUL << PMUC_CAU_RSVD_RESERVE0_Pos)
#define PMUC_CAU_RSVD_RESERVE0          PMUC_CAU_RSVD_RESERVE0_Msk
#define PMUC_CAU_RSVD_RESERVE1_Pos      (8U)
#define PMUC_CAU_RSVD_RESERVE1_Msk      (0xFFUL << PMUC_CAU_RSVD_RESERVE1_Pos)
#define PMUC_CAU_RSVD_RESERVE1          PMUC_CAU_RSVD_RESERVE1_Msk

/**************** Bit definition for PMUC_PWRKEY_CNT register *****************/
#define PMUC_PWRKEY_CNT_RST_CNT_Pos     (4U)
#define PMUC_PWRKEY_CNT_RST_CNT_Msk     (0xFFFFUL << PMUC_PWRKEY_CNT_RST_CNT_Pos)
#define PMUC_PWRKEY_CNT_RST_CNT         PMUC_PWRKEY_CNT_RST_CNT_Msk

/**************** Bit definition for PMUC_HPSYS_VOUT register *****************/
#define PMUC_HPSYS_VOUT_VOUT_Pos        (0U)
#define PMUC_HPSYS_VOUT_VOUT_Msk        (0xFUL << PMUC_HPSYS_VOUT_VOUT_Pos)
#define PMUC_HPSYS_VOUT_VOUT            PMUC_HPSYS_VOUT_VOUT_Msk

/**************** Bit definition for PMUC_LPSYS_VOUT register *****************/
#define PMUC_LPSYS_VOUT_VOUT_Pos        (0U)
#define PMUC_LPSYS_VOUT_VOUT_Msk        (0xFUL << PMUC_LPSYS_VOUT_VOUT_Pos)
#define PMUC_LPSYS_VOUT_VOUT            PMUC_LPSYS_VOUT_VOUT_Msk

/***************** Bit definition for PMUC_BUCK_VOUT register *****************/
#define PMUC_BUCK_VOUT_VOUT_Pos         (0U)
#define PMUC_BUCK_VOUT_VOUT_Msk         (0x1FUL << PMUC_BUCK_VOUT_VOUT_Pos)
#define PMUC_BUCK_VOUT_VOUT             PMUC_BUCK_VOUT_VOUT_Msk

/***************** Bit definition for PMUC_WKUP_MODE register *****************/
#define PMUC_WKUP_MODE_PA33_MODE_Pos    (0U)
#define PMUC_WKUP_MODE_PA33_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA33_MODE_Pos)
#define PMUC_WKUP_MODE_PA33_MODE        PMUC_WKUP_MODE_PA33_MODE_Msk
#define PMUC_WKUP_MODE_PA34_MODE_Pos    (2U)
#define PMUC_WKUP_MODE_PA34_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA34_MODE_Pos)
#define PMUC_WKUP_MODE_PA34_MODE        PMUC_WKUP_MODE_PA34_MODE_Msk
#define PMUC_WKUP_MODE_PA35_MODE_Pos    (4U)
#define PMUC_WKUP_MODE_PA35_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA35_MODE_Pos)
#define PMUC_WKUP_MODE_PA35_MODE        PMUC_WKUP_MODE_PA35_MODE_Msk
#define PMUC_WKUP_MODE_PA36_MODE_Pos    (6U)
#define PMUC_WKUP_MODE_PA36_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA36_MODE_Pos)
#define PMUC_WKUP_MODE_PA36_MODE        PMUC_WKUP_MODE_PA36_MODE_Msk
#define PMUC_WKUP_MODE_PA37_MODE_Pos    (8U)
#define PMUC_WKUP_MODE_PA37_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA37_MODE_Pos)
#define PMUC_WKUP_MODE_PA37_MODE        PMUC_WKUP_MODE_PA37_MODE_Msk
#define PMUC_WKUP_MODE_PA38_MODE_Pos    (10U)
#define PMUC_WKUP_MODE_PA38_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA38_MODE_Pos)
#define PMUC_WKUP_MODE_PA38_MODE        PMUC_WKUP_MODE_PA38_MODE_Msk
#define PMUC_WKUP_MODE_PA39_MODE_Pos    (12U)
#define PMUC_WKUP_MODE_PA39_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA39_MODE_Pos)
#define PMUC_WKUP_MODE_PA39_MODE        PMUC_WKUP_MODE_PA39_MODE_Msk
#define PMUC_WKUP_MODE_PA40_MODE_Pos    (14U)
#define PMUC_WKUP_MODE_PA40_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA40_MODE_Pos)
#define PMUC_WKUP_MODE_PA40_MODE        PMUC_WKUP_MODE_PA40_MODE_Msk
#define PMUC_WKUP_MODE_PA41_MODE_Pos    (16U)
#define PMUC_WKUP_MODE_PA41_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA41_MODE_Pos)
#define PMUC_WKUP_MODE_PA41_MODE        PMUC_WKUP_MODE_PA41_MODE_Msk
#define PMUC_WKUP_MODE_PA42_MODE_Pos    (18U)
#define PMUC_WKUP_MODE_PA42_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA42_MODE_Pos)
#define PMUC_WKUP_MODE_PA42_MODE        PMUC_WKUP_MODE_PA42_MODE_Msk
#define PMUC_WKUP_MODE_PA24_MODE_Pos    (20U)
#define PMUC_WKUP_MODE_PA24_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA24_MODE_Pos)
#define PMUC_WKUP_MODE_PA24_MODE        PMUC_WKUP_MODE_PA24_MODE_Msk
#define PMUC_WKUP_MODE_PA25_MODE_Pos    (22U)
#define PMUC_WKUP_MODE_PA25_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA25_MODE_Pos)
#define PMUC_WKUP_MODE_PA25_MODE        PMUC_WKUP_MODE_PA25_MODE_Msk
#define PMUC_WKUP_MODE_PA26_MODE_Pos    (24U)
#define PMUC_WKUP_MODE_PA26_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA26_MODE_Pos)
#define PMUC_WKUP_MODE_PA26_MODE        PMUC_WKUP_MODE_PA26_MODE_Msk
#define PMUC_WKUP_MODE_PA27_MODE_Pos    (26U)
#define PMUC_WKUP_MODE_PA27_MODE_Msk    (0x3UL << PMUC_WKUP_MODE_PA27_MODE_Pos)
#define PMUC_WKUP_MODE_PA27_MODE        PMUC_WKUP_MODE_PA27_MODE_Msk

/***************** Bit definition for PMUC_WKUP_CNT register ******************/
#define PMUC_WKUP_CNT_PIN_DLY_Pos       (0U)
#define PMUC_WKUP_CNT_PIN_DLY_Msk       (0xFFFFUL << PMUC_WKUP_CNT_PIN_DLY_Pos)
#define PMUC_WKUP_CNT_PIN_DLY           PMUC_WKUP_CNT_PIN_DLY_Msk

/******************* Bit definition for PMUC_PRSR1 register *******************/
#define PMUC_PRSR1_PA0_Pos              (0U)
#define PMUC_PRSR1_PA0_Msk              (0x1UL << PMUC_PRSR1_PA0_Pos)
#define PMUC_PRSR1_PA0                  PMUC_PRSR1_PA0_Msk
#define PMUC_PRSR1_PA1_Pos              (1U)
#define PMUC_PRSR1_PA1_Msk              (0x1UL << PMUC_PRSR1_PA1_Pos)
#define PMUC_PRSR1_PA1                  PMUC_PRSR1_PA1_Msk
#define PMUC_PRSR1_PA2_Pos              (2U)
#define PMUC_PRSR1_PA2_Msk              (0x1UL << PMUC_PRSR1_PA2_Pos)
#define PMUC_PRSR1_PA2                  PMUC_PRSR1_PA2_Msk
#define PMUC_PRSR1_PA3_Pos              (3U)
#define PMUC_PRSR1_PA3_Msk              (0x1UL << PMUC_PRSR1_PA3_Pos)
#define PMUC_PRSR1_PA3                  PMUC_PRSR1_PA3_Msk
#define PMUC_PRSR1_PA4_Pos              (4U)
#define PMUC_PRSR1_PA4_Msk              (0x1UL << PMUC_PRSR1_PA4_Pos)
#define PMUC_PRSR1_PA4                  PMUC_PRSR1_PA4_Msk
#define PMUC_PRSR1_PA5_Pos              (5U)
#define PMUC_PRSR1_PA5_Msk              (0x1UL << PMUC_PRSR1_PA5_Pos)
#define PMUC_PRSR1_PA5                  PMUC_PRSR1_PA5_Msk
#define PMUC_PRSR1_PA6_Pos              (6U)
#define PMUC_PRSR1_PA6_Msk              (0x1UL << PMUC_PRSR1_PA6_Pos)
#define PMUC_PRSR1_PA6                  PMUC_PRSR1_PA6_Msk
#define PMUC_PRSR1_PA7_Pos              (7U)
#define PMUC_PRSR1_PA7_Msk              (0x1UL << PMUC_PRSR1_PA7_Pos)
#define PMUC_PRSR1_PA7                  PMUC_PRSR1_PA7_Msk
#define PMUC_PRSR1_PA8_Pos              (8U)
#define PMUC_PRSR1_PA8_Msk              (0x1UL << PMUC_PRSR1_PA8_Pos)
#define PMUC_PRSR1_PA8                  PMUC_PRSR1_PA8_Msk
#define PMUC_PRSR1_PA9_Pos              (9U)
#define PMUC_PRSR1_PA9_Msk              (0x1UL << PMUC_PRSR1_PA9_Pos)
#define PMUC_PRSR1_PA9                  PMUC_PRSR1_PA9_Msk
#define PMUC_PRSR1_PA10_Pos             (10U)
#define PMUC_PRSR1_PA10_Msk             (0x1UL << PMUC_PRSR1_PA10_Pos)
#define PMUC_PRSR1_PA10                 PMUC_PRSR1_PA10_Msk
#define PMUC_PRSR1_PA11_Pos             (11U)
#define PMUC_PRSR1_PA11_Msk             (0x1UL << PMUC_PRSR1_PA11_Pos)
#define PMUC_PRSR1_PA11                 PMUC_PRSR1_PA11_Msk
#define PMUC_PRSR1_PA12_Pos             (12U)
#define PMUC_PRSR1_PA12_Msk             (0x1UL << PMUC_PRSR1_PA12_Pos)
#define PMUC_PRSR1_PA12                 PMUC_PRSR1_PA12_Msk
#define PMUC_PRSR1_PA13_Pos             (13U)
#define PMUC_PRSR1_PA13_Msk             (0x1UL << PMUC_PRSR1_PA13_Pos)
#define PMUC_PRSR1_PA13                 PMUC_PRSR1_PA13_Msk
#define PMUC_PRSR1_PA14_Pos             (14U)
#define PMUC_PRSR1_PA14_Msk             (0x1UL << PMUC_PRSR1_PA14_Pos)
#define PMUC_PRSR1_PA14                 PMUC_PRSR1_PA14_Msk
#define PMUC_PRSR1_PA15_Pos             (15U)
#define PMUC_PRSR1_PA15_Msk             (0x1UL << PMUC_PRSR1_PA15_Pos)
#define PMUC_PRSR1_PA15                 PMUC_PRSR1_PA15_Msk
#define PMUC_PRSR1_PA16_Pos             (16U)
#define PMUC_PRSR1_PA16_Msk             (0x1UL << PMUC_PRSR1_PA16_Pos)
#define PMUC_PRSR1_PA16                 PMUC_PRSR1_PA16_Msk
#define PMUC_PRSR1_PA17_Pos             (17U)
#define PMUC_PRSR1_PA17_Msk             (0x1UL << PMUC_PRSR1_PA17_Pos)
#define PMUC_PRSR1_PA17                 PMUC_PRSR1_PA17_Msk
#define PMUC_PRSR1_PA18_Pos             (18U)
#define PMUC_PRSR1_PA18_Msk             (0x1UL << PMUC_PRSR1_PA18_Pos)
#define PMUC_PRSR1_PA18                 PMUC_PRSR1_PA18_Msk
#define PMUC_PRSR1_PA19_Pos             (19U)
#define PMUC_PRSR1_PA19_Msk             (0x1UL << PMUC_PRSR1_PA19_Pos)
#define PMUC_PRSR1_PA19                 PMUC_PRSR1_PA19_Msk
#define PMUC_PRSR1_PA20_Pos             (20U)
#define PMUC_PRSR1_PA20_Msk             (0x1UL << PMUC_PRSR1_PA20_Pos)
#define PMUC_PRSR1_PA20                 PMUC_PRSR1_PA20_Msk
#define PMUC_PRSR1_PA21_Pos             (21U)
#define PMUC_PRSR1_PA21_Msk             (0x1UL << PMUC_PRSR1_PA21_Pos)
#define PMUC_PRSR1_PA21                 PMUC_PRSR1_PA21_Msk
#define PMUC_PRSR1_PA22_Pos             (22U)
#define PMUC_PRSR1_PA22_Msk             (0x1UL << PMUC_PRSR1_PA22_Pos)
#define PMUC_PRSR1_PA22                 PMUC_PRSR1_PA22_Msk
#define PMUC_PRSR1_PA23_Pos             (23U)
#define PMUC_PRSR1_PA23_Msk             (0x1UL << PMUC_PRSR1_PA23_Pos)
#define PMUC_PRSR1_PA23                 PMUC_PRSR1_PA23_Msk
#define PMUC_PRSR1_PA24_Pos             (24U)
#define PMUC_PRSR1_PA24_Msk             (0x1UL << PMUC_PRSR1_PA24_Pos)
#define PMUC_PRSR1_PA24                 PMUC_PRSR1_PA24_Msk
#define PMUC_PRSR1_PA25_Pos             (25U)
#define PMUC_PRSR1_PA25_Msk             (0x1UL << PMUC_PRSR1_PA25_Pos)
#define PMUC_PRSR1_PA25                 PMUC_PRSR1_PA25_Msk
#define PMUC_PRSR1_PA26_Pos             (26U)
#define PMUC_PRSR1_PA26_Msk             (0x1UL << PMUC_PRSR1_PA26_Pos)
#define PMUC_PRSR1_PA26                 PMUC_PRSR1_PA26_Msk
#define PMUC_PRSR1_PA27_Pos             (27U)
#define PMUC_PRSR1_PA27_Msk             (0x1UL << PMUC_PRSR1_PA27_Pos)
#define PMUC_PRSR1_PA27                 PMUC_PRSR1_PA27_Msk
#define PMUC_PRSR1_PA28_Pos             (28U)
#define PMUC_PRSR1_PA28_Msk             (0x1UL << PMUC_PRSR1_PA28_Pos)
#define PMUC_PRSR1_PA28                 PMUC_PRSR1_PA28_Msk
#define PMUC_PRSR1_PA29_Pos             (29U)
#define PMUC_PRSR1_PA29_Msk             (0x1UL << PMUC_PRSR1_PA29_Pos)
#define PMUC_PRSR1_PA29                 PMUC_PRSR1_PA29_Msk
#define PMUC_PRSR1_PA30_Pos             (30U)
#define PMUC_PRSR1_PA30_Msk             (0x1UL << PMUC_PRSR1_PA30_Pos)
#define PMUC_PRSR1_PA30                 PMUC_PRSR1_PA30_Msk
#define PMUC_PRSR1_PA31_Pos             (31U)
#define PMUC_PRSR1_PA31_Msk             (0x1UL << PMUC_PRSR1_PA31_Pos)
#define PMUC_PRSR1_PA31                 PMUC_PRSR1_PA31_Msk

/******************* Bit definition for PMUC_PRSR2 register *******************/
#define PMUC_PRSR2_PA32_Pos             (0U)
#define PMUC_PRSR2_PA32_Msk             (0x1UL << PMUC_PRSR2_PA32_Pos)
#define PMUC_PRSR2_PA32                 PMUC_PRSR2_PA32_Msk
#define PMUC_PRSR2_PA33_Pos             (1U)
#define PMUC_PRSR2_PA33_Msk             (0x1UL << PMUC_PRSR2_PA33_Pos)
#define PMUC_PRSR2_PA33                 PMUC_PRSR2_PA33_Msk
#define PMUC_PRSR2_PA34_Pos             (2U)
#define PMUC_PRSR2_PA34_Msk             (0x1UL << PMUC_PRSR2_PA34_Pos)
#define PMUC_PRSR2_PA34                 PMUC_PRSR2_PA34_Msk
#define PMUC_PRSR2_PA35_Pos             (3U)
#define PMUC_PRSR2_PA35_Msk             (0x1UL << PMUC_PRSR2_PA35_Pos)
#define PMUC_PRSR2_PA35                 PMUC_PRSR2_PA35_Msk
#define PMUC_PRSR2_PA36_Pos             (4U)
#define PMUC_PRSR2_PA36_Msk             (0x1UL << PMUC_PRSR2_PA36_Pos)
#define PMUC_PRSR2_PA36                 PMUC_PRSR2_PA36_Msk
#define PMUC_PRSR2_PA37_Pos             (5U)
#define PMUC_PRSR2_PA37_Msk             (0x1UL << PMUC_PRSR2_PA37_Pos)
#define PMUC_PRSR2_PA37                 PMUC_PRSR2_PA37_Msk
#define PMUC_PRSR2_PA38_Pos             (6U)
#define PMUC_PRSR2_PA38_Msk             (0x1UL << PMUC_PRSR2_PA38_Pos)
#define PMUC_PRSR2_PA38                 PMUC_PRSR2_PA38_Msk
#define PMUC_PRSR2_PA39_Pos             (7U)
#define PMUC_PRSR2_PA39_Msk             (0x1UL << PMUC_PRSR2_PA39_Pos)
#define PMUC_PRSR2_PA39                 PMUC_PRSR2_PA39_Msk
#define PMUC_PRSR2_PA40_Pos             (8U)
#define PMUC_PRSR2_PA40_Msk             (0x1UL << PMUC_PRSR2_PA40_Pos)
#define PMUC_PRSR2_PA40                 PMUC_PRSR2_PA40_Msk
#define PMUC_PRSR2_PA41_Pos             (9U)
#define PMUC_PRSR2_PA41_Msk             (0x1UL << PMUC_PRSR2_PA41_Pos)
#define PMUC_PRSR2_PA41                 PMUC_PRSR2_PA41_Msk
#define PMUC_PRSR2_PA42_Pos             (10U)
#define PMUC_PRSR2_PA42_Msk             (0x1UL << PMUC_PRSR2_PA42_Pos)
#define PMUC_PRSR2_PA42                 PMUC_PRSR2_PA42_Msk
#define PMUC_PRSR2_PA43_Pos             (11U)
#define PMUC_PRSR2_PA43_Msk             (0x1UL << PMUC_PRSR2_PA43_Pos)
#define PMUC_PRSR2_PA43                 PMUC_PRSR2_PA43_Msk
#define PMUC_PRSR2_PA44_Pos             (12U)
#define PMUC_PRSR2_PA44_Msk             (0x1UL << PMUC_PRSR2_PA44_Pos)
#define PMUC_PRSR2_PA44                 PMUC_PRSR2_PA44_Msk
#define PMUC_PRSR2_PA45_Pos             (13U)
#define PMUC_PRSR2_PA45_Msk             (0x1UL << PMUC_PRSR2_PA45_Pos)
#define PMUC_PRSR2_PA45                 PMUC_PRSR2_PA45_Msk
#define PMUC_PRSR2_PA46_Pos             (14U)
#define PMUC_PRSR2_PA46_Msk             (0x1UL << PMUC_PRSR2_PA46_Pos)
#define PMUC_PRSR2_PA46                 PMUC_PRSR2_PA46_Msk
#define PMUC_PRSR2_PA47_Pos             (15U)
#define PMUC_PRSR2_PA47_Msk             (0x1UL << PMUC_PRSR2_PA47_Pos)
#define PMUC_PRSR2_PA47                 PMUC_PRSR2_PA47_Msk
#define PMUC_PRSR2_PA48_Pos             (16U)
#define PMUC_PRSR2_PA48_Msk             (0x1UL << PMUC_PRSR2_PA48_Pos)
#define PMUC_PRSR2_PA48                 PMUC_PRSR2_PA48_Msk
#define PMUC_PRSR2_PA49_Pos             (17U)
#define PMUC_PRSR2_PA49_Msk             (0x1UL << PMUC_PRSR2_PA49_Pos)
#define PMUC_PRSR2_PA49                 PMUC_PRSR2_PA49_Msk
#define PMUC_PRSR2_PA50_Pos             (18U)
#define PMUC_PRSR2_PA50_Msk             (0x1UL << PMUC_PRSR2_PA50_Pos)
#define PMUC_PRSR2_PA50                 PMUC_PRSR2_PA50_Msk
#define PMUC_PRSR2_PA51_Pos             (19U)
#define PMUC_PRSR2_PA51_Msk             (0x1UL << PMUC_PRSR2_PA51_Pos)
#define PMUC_PRSR2_PA51                 PMUC_PRSR2_PA51_Msk
#define PMUC_PRSR2_PA52_Pos             (20U)
#define PMUC_PRSR2_PA52_Msk             (0x1UL << PMUC_PRSR2_PA52_Pos)
#define PMUC_PRSR2_PA52                 PMUC_PRSR2_PA52_Msk
#define PMUC_PRSR2_PA53_Pos             (21U)
#define PMUC_PRSR2_PA53_Msk             (0x1UL << PMUC_PRSR2_PA53_Pos)
#define PMUC_PRSR2_PA53                 PMUC_PRSR2_PA53_Msk
#define PMUC_PRSR2_PA54_Pos             (22U)
#define PMUC_PRSR2_PA54_Msk             (0x1UL << PMUC_PRSR2_PA54_Pos)
#define PMUC_PRSR2_PA54                 PMUC_PRSR2_PA54_Msk
#define PMUC_PRSR2_PA55_Pos             (23U)
#define PMUC_PRSR2_PA55_Msk             (0x1UL << PMUC_PRSR2_PA55_Pos)
#define PMUC_PRSR2_PA55                 PMUC_PRSR2_PA55_Msk
#define PMUC_PRSR2_PA56_Pos             (24U)
#define PMUC_PRSR2_PA56_Msk             (0x1UL << PMUC_PRSR2_PA56_Pos)
#define PMUC_PRSR2_PA56                 PMUC_PRSR2_PA56_Msk
#define PMUC_PRSR2_PA57_Pos             (25U)
#define PMUC_PRSR2_PA57_Msk             (0x1UL << PMUC_PRSR2_PA57_Pos)
#define PMUC_PRSR2_PA57                 PMUC_PRSR2_PA57_Msk
#define PMUC_PRSR2_SB_Pos               (30U)
#define PMUC_PRSR2_SB_Msk               (0x1UL << PMUC_PRSR2_SB_Pos)
#define PMUC_PRSR2_SB                   PMUC_PRSR2_SB_Msk
#define PMUC_PRSR2_SA_Pos               (31U)
#define PMUC_PRSR2_SA_Msk               (0x1UL << PMUC_PRSR2_SA_Pos)
#define PMUC_PRSR2_SA                   PMUC_PRSR2_SA_Msk

/******************* Bit definition for PMUC_PRCR1 register *******************/
#define PMUC_PRCR1_PA0_Pos              (0U)
#define PMUC_PRCR1_PA0_Msk              (0x1UL << PMUC_PRCR1_PA0_Pos)
#define PMUC_PRCR1_PA0                  PMUC_PRCR1_PA0_Msk
#define PMUC_PRCR1_PA1_Pos              (1U)
#define PMUC_PRCR1_PA1_Msk              (0x1UL << PMUC_PRCR1_PA1_Pos)
#define PMUC_PRCR1_PA1                  PMUC_PRCR1_PA1_Msk
#define PMUC_PRCR1_PA2_Pos              (2U)
#define PMUC_PRCR1_PA2_Msk              (0x1UL << PMUC_PRCR1_PA2_Pos)
#define PMUC_PRCR1_PA2                  PMUC_PRCR1_PA2_Msk
#define PMUC_PRCR1_PA3_Pos              (3U)
#define PMUC_PRCR1_PA3_Msk              (0x1UL << PMUC_PRCR1_PA3_Pos)
#define PMUC_PRCR1_PA3                  PMUC_PRCR1_PA3_Msk
#define PMUC_PRCR1_PA4_Pos              (4U)
#define PMUC_PRCR1_PA4_Msk              (0x1UL << PMUC_PRCR1_PA4_Pos)
#define PMUC_PRCR1_PA4                  PMUC_PRCR1_PA4_Msk
#define PMUC_PRCR1_PA5_Pos              (5U)
#define PMUC_PRCR1_PA5_Msk              (0x1UL << PMUC_PRCR1_PA5_Pos)
#define PMUC_PRCR1_PA5                  PMUC_PRCR1_PA5_Msk
#define PMUC_PRCR1_PA6_Pos              (6U)
#define PMUC_PRCR1_PA6_Msk              (0x1UL << PMUC_PRCR1_PA6_Pos)
#define PMUC_PRCR1_PA6                  PMUC_PRCR1_PA6_Msk
#define PMUC_PRCR1_PA7_Pos              (7U)
#define PMUC_PRCR1_PA7_Msk              (0x1UL << PMUC_PRCR1_PA7_Pos)
#define PMUC_PRCR1_PA7                  PMUC_PRCR1_PA7_Msk
#define PMUC_PRCR1_PA8_Pos              (8U)
#define PMUC_PRCR1_PA8_Msk              (0x1UL << PMUC_PRCR1_PA8_Pos)
#define PMUC_PRCR1_PA8                  PMUC_PRCR1_PA8_Msk
#define PMUC_PRCR1_PA9_Pos              (9U)
#define PMUC_PRCR1_PA9_Msk              (0x1UL << PMUC_PRCR1_PA9_Pos)
#define PMUC_PRCR1_PA9                  PMUC_PRCR1_PA9_Msk
#define PMUC_PRCR1_PA10_Pos             (10U)
#define PMUC_PRCR1_PA10_Msk             (0x1UL << PMUC_PRCR1_PA10_Pos)
#define PMUC_PRCR1_PA10                 PMUC_PRCR1_PA10_Msk
#define PMUC_PRCR1_PA11_Pos             (11U)
#define PMUC_PRCR1_PA11_Msk             (0x1UL << PMUC_PRCR1_PA11_Pos)
#define PMUC_PRCR1_PA11                 PMUC_PRCR1_PA11_Msk
#define PMUC_PRCR1_PA12_Pos             (12U)
#define PMUC_PRCR1_PA12_Msk             (0x1UL << PMUC_PRCR1_PA12_Pos)
#define PMUC_PRCR1_PA12                 PMUC_PRCR1_PA12_Msk
#define PMUC_PRCR1_PA13_Pos             (13U)
#define PMUC_PRCR1_PA13_Msk             (0x1UL << PMUC_PRCR1_PA13_Pos)
#define PMUC_PRCR1_PA13                 PMUC_PRCR1_PA13_Msk
#define PMUC_PRCR1_PA14_Pos             (14U)
#define PMUC_PRCR1_PA14_Msk             (0x1UL << PMUC_PRCR1_PA14_Pos)
#define PMUC_PRCR1_PA14                 PMUC_PRCR1_PA14_Msk
#define PMUC_PRCR1_PA15_Pos             (15U)
#define PMUC_PRCR1_PA15_Msk             (0x1UL << PMUC_PRCR1_PA15_Pos)
#define PMUC_PRCR1_PA15                 PMUC_PRCR1_PA15_Msk
#define PMUC_PRCR1_PA16_Pos             (16U)
#define PMUC_PRCR1_PA16_Msk             (0x1UL << PMUC_PRCR1_PA16_Pos)
#define PMUC_PRCR1_PA16                 PMUC_PRCR1_PA16_Msk
#define PMUC_PRCR1_PA17_Pos             (17U)
#define PMUC_PRCR1_PA17_Msk             (0x1UL << PMUC_PRCR1_PA17_Pos)
#define PMUC_PRCR1_PA17                 PMUC_PRCR1_PA17_Msk
#define PMUC_PRCR1_PA18_Pos             (18U)
#define PMUC_PRCR1_PA18_Msk             (0x1UL << PMUC_PRCR1_PA18_Pos)
#define PMUC_PRCR1_PA18                 PMUC_PRCR1_PA18_Msk
#define PMUC_PRCR1_PA19_Pos             (19U)
#define PMUC_PRCR1_PA19_Msk             (0x1UL << PMUC_PRCR1_PA19_Pos)
#define PMUC_PRCR1_PA19                 PMUC_PRCR1_PA19_Msk
#define PMUC_PRCR1_PA20_Pos             (20U)
#define PMUC_PRCR1_PA20_Msk             (0x1UL << PMUC_PRCR1_PA20_Pos)
#define PMUC_PRCR1_PA20                 PMUC_PRCR1_PA20_Msk
#define PMUC_PRCR1_PA21_Pos             (21U)
#define PMUC_PRCR1_PA21_Msk             (0x1UL << PMUC_PRCR1_PA21_Pos)
#define PMUC_PRCR1_PA21                 PMUC_PRCR1_PA21_Msk
#define PMUC_PRCR1_PA22_Pos             (22U)
#define PMUC_PRCR1_PA22_Msk             (0x1UL << PMUC_PRCR1_PA22_Pos)
#define PMUC_PRCR1_PA22                 PMUC_PRCR1_PA22_Msk
#define PMUC_PRCR1_PA23_Pos             (23U)
#define PMUC_PRCR1_PA23_Msk             (0x1UL << PMUC_PRCR1_PA23_Pos)
#define PMUC_PRCR1_PA23                 PMUC_PRCR1_PA23_Msk
#define PMUC_PRCR1_PA24_Pos             (24U)
#define PMUC_PRCR1_PA24_Msk             (0x1UL << PMUC_PRCR1_PA24_Pos)
#define PMUC_PRCR1_PA24                 PMUC_PRCR1_PA24_Msk
#define PMUC_PRCR1_PA25_Pos             (25U)
#define PMUC_PRCR1_PA25_Msk             (0x1UL << PMUC_PRCR1_PA25_Pos)
#define PMUC_PRCR1_PA25                 PMUC_PRCR1_PA25_Msk
#define PMUC_PRCR1_PA26_Pos             (26U)
#define PMUC_PRCR1_PA26_Msk             (0x1UL << PMUC_PRCR1_PA26_Pos)
#define PMUC_PRCR1_PA26                 PMUC_PRCR1_PA26_Msk
#define PMUC_PRCR1_PA27_Pos             (27U)
#define PMUC_PRCR1_PA27_Msk             (0x1UL << PMUC_PRCR1_PA27_Pos)
#define PMUC_PRCR1_PA27                 PMUC_PRCR1_PA27_Msk
#define PMUC_PRCR1_PA28_Pos             (28U)
#define PMUC_PRCR1_PA28_Msk             (0x1UL << PMUC_PRCR1_PA28_Pos)
#define PMUC_PRCR1_PA28                 PMUC_PRCR1_PA28_Msk
#define PMUC_PRCR1_PA29_Pos             (29U)
#define PMUC_PRCR1_PA29_Msk             (0x1UL << PMUC_PRCR1_PA29_Pos)
#define PMUC_PRCR1_PA29                 PMUC_PRCR1_PA29_Msk
#define PMUC_PRCR1_PA30_Pos             (30U)
#define PMUC_PRCR1_PA30_Msk             (0x1UL << PMUC_PRCR1_PA30_Pos)
#define PMUC_PRCR1_PA30                 PMUC_PRCR1_PA30_Msk
#define PMUC_PRCR1_PA31_Pos             (31U)
#define PMUC_PRCR1_PA31_Msk             (0x1UL << PMUC_PRCR1_PA31_Pos)
#define PMUC_PRCR1_PA31                 PMUC_PRCR1_PA31_Msk

/******************* Bit definition for PMUC_PRCR2 register *******************/
#define PMUC_PRCR2_PA32_Pos             (0U)
#define PMUC_PRCR2_PA32_Msk             (0x1UL << PMUC_PRCR2_PA32_Pos)
#define PMUC_PRCR2_PA32                 PMUC_PRCR2_PA32_Msk
#define PMUC_PRCR2_PA33_Pos             (1U)
#define PMUC_PRCR2_PA33_Msk             (0x1UL << PMUC_PRCR2_PA33_Pos)
#define PMUC_PRCR2_PA33                 PMUC_PRCR2_PA33_Msk
#define PMUC_PRCR2_PA34_Pos             (2U)
#define PMUC_PRCR2_PA34_Msk             (0x1UL << PMUC_PRCR2_PA34_Pos)
#define PMUC_PRCR2_PA34                 PMUC_PRCR2_PA34_Msk
#define PMUC_PRCR2_PA35_Pos             (3U)
#define PMUC_PRCR2_PA35_Msk             (0x1UL << PMUC_PRCR2_PA35_Pos)
#define PMUC_PRCR2_PA35                 PMUC_PRCR2_PA35_Msk
#define PMUC_PRCR2_PA36_Pos             (4U)
#define PMUC_PRCR2_PA36_Msk             (0x1UL << PMUC_PRCR2_PA36_Pos)
#define PMUC_PRCR2_PA36                 PMUC_PRCR2_PA36_Msk
#define PMUC_PRCR2_PA37_Pos             (5U)
#define PMUC_PRCR2_PA37_Msk             (0x1UL << PMUC_PRCR2_PA37_Pos)
#define PMUC_PRCR2_PA37                 PMUC_PRCR2_PA37_Msk
#define PMUC_PRCR2_PA38_Pos             (6U)
#define PMUC_PRCR2_PA38_Msk             (0x1UL << PMUC_PRCR2_PA38_Pos)
#define PMUC_PRCR2_PA38                 PMUC_PRCR2_PA38_Msk
#define PMUC_PRCR2_PA39_Pos             (7U)
#define PMUC_PRCR2_PA39_Msk             (0x1UL << PMUC_PRCR2_PA39_Pos)
#define PMUC_PRCR2_PA39                 PMUC_PRCR2_PA39_Msk
#define PMUC_PRCR2_PA40_Pos             (8U)
#define PMUC_PRCR2_PA40_Msk             (0x1UL << PMUC_PRCR2_PA40_Pos)
#define PMUC_PRCR2_PA40                 PMUC_PRCR2_PA40_Msk
#define PMUC_PRCR2_PA41_Pos             (9U)
#define PMUC_PRCR2_PA41_Msk             (0x1UL << PMUC_PRCR2_PA41_Pos)
#define PMUC_PRCR2_PA41                 PMUC_PRCR2_PA41_Msk
#define PMUC_PRCR2_PA42_Pos             (10U)
#define PMUC_PRCR2_PA42_Msk             (0x1UL << PMUC_PRCR2_PA42_Pos)
#define PMUC_PRCR2_PA42                 PMUC_PRCR2_PA42_Msk
#define PMUC_PRCR2_PA43_Pos             (11U)
#define PMUC_PRCR2_PA43_Msk             (0x1UL << PMUC_PRCR2_PA43_Pos)
#define PMUC_PRCR2_PA43                 PMUC_PRCR2_PA43_Msk
#define PMUC_PRCR2_PA44_Pos             (12U)
#define PMUC_PRCR2_PA44_Msk             (0x1UL << PMUC_PRCR2_PA44_Pos)
#define PMUC_PRCR2_PA44                 PMUC_PRCR2_PA44_Msk
#define PMUC_PRCR2_PA45_Pos             (13U)
#define PMUC_PRCR2_PA45_Msk             (0x1UL << PMUC_PRCR2_PA45_Pos)
#define PMUC_PRCR2_PA45                 PMUC_PRCR2_PA45_Msk
#define PMUC_PRCR2_PA46_Pos             (14U)
#define PMUC_PRCR2_PA46_Msk             (0x1UL << PMUC_PRCR2_PA46_Pos)
#define PMUC_PRCR2_PA46                 PMUC_PRCR2_PA46_Msk
#define PMUC_PRCR2_PA47_Pos             (15U)
#define PMUC_PRCR2_PA47_Msk             (0x1UL << PMUC_PRCR2_PA47_Pos)
#define PMUC_PRCR2_PA47                 PMUC_PRCR2_PA47_Msk
#define PMUC_PRCR2_PA48_Pos             (16U)
#define PMUC_PRCR2_PA48_Msk             (0x1UL << PMUC_PRCR2_PA48_Pos)
#define PMUC_PRCR2_PA48                 PMUC_PRCR2_PA48_Msk
#define PMUC_PRCR2_PA49_Pos             (17U)
#define PMUC_PRCR2_PA49_Msk             (0x1UL << PMUC_PRCR2_PA49_Pos)
#define PMUC_PRCR2_PA49                 PMUC_PRCR2_PA49_Msk
#define PMUC_PRCR2_PA50_Pos             (18U)
#define PMUC_PRCR2_PA50_Msk             (0x1UL << PMUC_PRCR2_PA50_Pos)
#define PMUC_PRCR2_PA50                 PMUC_PRCR2_PA50_Msk
#define PMUC_PRCR2_PA51_Pos             (19U)
#define PMUC_PRCR2_PA51_Msk             (0x1UL << PMUC_PRCR2_PA51_Pos)
#define PMUC_PRCR2_PA51                 PMUC_PRCR2_PA51_Msk
#define PMUC_PRCR2_PA52_Pos             (20U)
#define PMUC_PRCR2_PA52_Msk             (0x1UL << PMUC_PRCR2_PA52_Pos)
#define PMUC_PRCR2_PA52                 PMUC_PRCR2_PA52_Msk
#define PMUC_PRCR2_PA53_Pos             (21U)
#define PMUC_PRCR2_PA53_Msk             (0x1UL << PMUC_PRCR2_PA53_Pos)
#define PMUC_PRCR2_PA53                 PMUC_PRCR2_PA53_Msk
#define PMUC_PRCR2_PA54_Pos             (22U)
#define PMUC_PRCR2_PA54_Msk             (0x1UL << PMUC_PRCR2_PA54_Pos)
#define PMUC_PRCR2_PA54                 PMUC_PRCR2_PA54_Msk
#define PMUC_PRCR2_PA55_Pos             (23U)
#define PMUC_PRCR2_PA55_Msk             (0x1UL << PMUC_PRCR2_PA55_Pos)
#define PMUC_PRCR2_PA55                 PMUC_PRCR2_PA55_Msk
#define PMUC_PRCR2_PA56_Pos             (24U)
#define PMUC_PRCR2_PA56_Msk             (0x1UL << PMUC_PRCR2_PA56_Pos)
#define PMUC_PRCR2_PA56                 PMUC_PRCR2_PA56_Msk
#define PMUC_PRCR2_PA57_Pos             (25U)
#define PMUC_PRCR2_PA57_Msk             (0x1UL << PMUC_PRCR2_PA57_Pos)
#define PMUC_PRCR2_PA57                 PMUC_PRCR2_PA57_Msk
#define PMUC_PRCR2_SB_Pos               (30U)
#define PMUC_PRCR2_SB_Msk               (0x1UL << PMUC_PRCR2_SB_Pos)
#define PMUC_PRCR2_SB                   PMUC_PRCR2_SB_Msk
#define PMUC_PRCR2_SA_Pos               (31U)
#define PMUC_PRCR2_SA_Msk               (0x1UL << PMUC_PRCR2_SA_Pos)
#define PMUC_PRCR2_SA                   PMUC_PRCR2_SA_Msk

/******************* Bit definition for PMUC_PBRCR register *******************/
#define PMUC_PBRCR_RTO_Pos              (0U)
#define PMUC_PBRCR_RTO_Msk              (0x1UL << PMUC_PBRCR_RTO_Pos)
#define PMUC_PBRCR_RTO                  PMUC_PBRCR_RTO_Msk
#define PMUC_PBRCR_SNS_Pos              (1U)
#define PMUC_PBRCR_SNS_Msk              (0x1UL << PMUC_PBRCR_SNS_Pos)
#define PMUC_PBRCR_SNS                  PMUC_PBRCR_SNS_Msk
#define PMUC_PBRCR_DBG_SEL_Pos          (4U)
#define PMUC_PBRCR_DBG_SEL_Msk          (0xFUL << PMUC_PBRCR_DBG_SEL_Pos)
#define PMUC_PBRCR_DBG_SEL              PMUC_PBRCR_DBG_SEL_Msk

#endif
