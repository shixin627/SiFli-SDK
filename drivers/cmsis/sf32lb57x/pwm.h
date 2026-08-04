/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PWM_H
#define __PWM_H

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR;
    __IO uint32_t FIFO;
} PWM_ChannelTypeDef;

typedef struct
{
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CR1;
    __IO uint32_t CNT1;
    __IO uint32_t PSC1;
    __IO uint32_t ARR1;
    __IO uint32_t RCR1;
    __IO uint32_t CCR1;
    __IO uint32_t FIFO1;
    __IO uint32_t CR2;
    __IO uint32_t CNT2;
    __IO uint32_t PSC2;
    __IO uint32_t ARR2;
    __IO uint32_t RCR2;
    __IO uint32_t CCR2;
    __IO uint32_t FIFO2;
    __IO uint32_t CR3;
    __IO uint32_t CNT3;
    __IO uint32_t PSC3;
    __IO uint32_t ARR3;
    __IO uint32_t RCR3;
    __IO uint32_t CCR3;
    __IO uint32_t FIFO3;
    __IO uint32_t CR4;
    __IO uint32_t CNT4;
    __IO uint32_t PSC4;
    __IO uint32_t ARR4;
    __IO uint32_t RCR4;
    __IO uint32_t CCR4;
    __IO uint32_t FIFO4;
    __IO uint32_t CR5;
    __IO uint32_t CNT5;
    __IO uint32_t PSC5;
    __IO uint32_t ARR5;
    __IO uint32_t RCR5;
    __IO uint32_t INTV5;
} PWM_TypeDef;


/********************* Bit definition for PWM_SR register *********************/
#define PWM_SR_UIF1_Pos                 (0U)
#define PWM_SR_UIF1_Msk                 (0x1UL << PWM_SR_UIF1_Pos)
#define PWM_SR_UIF1                     PWM_SR_UIF1_Msk
#define PWM_SR_UIF2_Pos                 (1U)
#define PWM_SR_UIF2_Msk                 (0x1UL << PWM_SR_UIF2_Pos)
#define PWM_SR_UIF2                     PWM_SR_UIF2_Msk
#define PWM_SR_UIF3_Pos                 (2U)
#define PWM_SR_UIF3_Msk                 (0x1UL << PWM_SR_UIF3_Pos)
#define PWM_SR_UIF3                     PWM_SR_UIF3_Msk
#define PWM_SR_UIF4_Pos                 (3U)
#define PWM_SR_UIF4_Msk                 (0x1UL << PWM_SR_UIF4_Pos)
#define PWM_SR_UIF4                     PWM_SR_UIF4_Msk
#define PWM_SR_UIF5_Pos                 (4U)
#define PWM_SR_UIF5_Msk                 (0x1UL << PWM_SR_UIF5_Pos)
#define PWM_SR_UIF5                     PWM_SR_UIF5_Msk
#define PWM_SR_CCIF1_Pos                (8U)
#define PWM_SR_CCIF1_Msk                (0x1UL << PWM_SR_CCIF1_Pos)
#define PWM_SR_CCIF1                    PWM_SR_CCIF1_Msk
#define PWM_SR_CCIF2_Pos                (9U)
#define PWM_SR_CCIF2_Msk                (0x1UL << PWM_SR_CCIF2_Pos)
#define PWM_SR_CCIF2                    PWM_SR_CCIF2_Msk
#define PWM_SR_CCIF3_Pos                (10U)
#define PWM_SR_CCIF3_Msk                (0x1UL << PWM_SR_CCIF3_Pos)
#define PWM_SR_CCIF3                    PWM_SR_CCIF3_Msk
#define PWM_SR_CCIF4_Pos                (11U)
#define PWM_SR_CCIF4_Msk                (0x1UL << PWM_SR_CCIF4_Pos)
#define PWM_SR_CCIF4                    PWM_SR_CCIF4_Msk
#define PWM_SR_UFIF1_Pos                (16U)
#define PWM_SR_UFIF1_Msk                (0x1UL << PWM_SR_UFIF1_Pos)
#define PWM_SR_UFIF1                    PWM_SR_UFIF1_Msk
#define PWM_SR_UFIF2_Pos                (17U)
#define PWM_SR_UFIF2_Msk                (0x1UL << PWM_SR_UFIF2_Pos)
#define PWM_SR_UFIF2                    PWM_SR_UFIF2_Msk
#define PWM_SR_UFIF3_Pos                (18U)
#define PWM_SR_UFIF3_Msk                (0x1UL << PWM_SR_UFIF3_Pos)
#define PWM_SR_UFIF3                    PWM_SR_UFIF3_Msk
#define PWM_SR_UFIF4_Pos                (19U)
#define PWM_SR_UFIF4_Msk                (0x1UL << PWM_SR_UFIF4_Pos)
#define PWM_SR_UFIF4                    PWM_SR_UFIF4_Msk

/******************** Bit definition for PWM_EGR register *********************/
#define PWM_EGR_UG1_Pos                 (0U)
#define PWM_EGR_UG1_Msk                 (0x1UL << PWM_EGR_UG1_Pos)
#define PWM_EGR_UG1                     PWM_EGR_UG1_Msk
#define PWM_EGR_UG2_Pos                 (1U)
#define PWM_EGR_UG2_Msk                 (0x1UL << PWM_EGR_UG2_Pos)
#define PWM_EGR_UG2                     PWM_EGR_UG2_Msk
#define PWM_EGR_UG3_Pos                 (2U)
#define PWM_EGR_UG3_Msk                 (0x1UL << PWM_EGR_UG3_Pos)
#define PWM_EGR_UG3                     PWM_EGR_UG3_Msk
#define PWM_EGR_UG4_Pos                 (3U)
#define PWM_EGR_UG4_Msk                 (0x1UL << PWM_EGR_UG4_Pos)
#define PWM_EGR_UG4                     PWM_EGR_UG4_Msk
#define PWM_EGR_UG5_Pos                 (4U)
#define PWM_EGR_UG5_Msk                 (0x1UL << PWM_EGR_UG5_Pos)
#define PWM_EGR_UG5                     PWM_EGR_UG5_Msk
#define PWM_EGR_TG1_Pos                 (8U)
#define PWM_EGR_TG1_Msk                 (0x1UL << PWM_EGR_TG1_Pos)
#define PWM_EGR_TG1                     PWM_EGR_TG1_Msk
#define PWM_EGR_TG2_Pos                 (9U)
#define PWM_EGR_TG2_Msk                 (0x1UL << PWM_EGR_TG2_Pos)
#define PWM_EGR_TG2                     PWM_EGR_TG2_Msk
#define PWM_EGR_TG3_Pos                 (10U)
#define PWM_EGR_TG3_Msk                 (0x1UL << PWM_EGR_TG3_Pos)
#define PWM_EGR_TG3                     PWM_EGR_TG3_Msk
#define PWM_EGR_TG4_Pos                 (11U)
#define PWM_EGR_TG4_Msk                 (0x1UL << PWM_EGR_TG4_Pos)
#define PWM_EGR_TG4                     PWM_EGR_TG4_Msk
#define PWM_EGR_APLOAD1_Pos             (16U)
#define PWM_EGR_APLOAD1_Msk             (0x1UL << PWM_EGR_APLOAD1_Pos)
#define PWM_EGR_APLOAD1                 PWM_EGR_APLOAD1_Msk
#define PWM_EGR_APLOAD2_Pos             (17U)
#define PWM_EGR_APLOAD2_Msk             (0x1UL << PWM_EGR_APLOAD2_Pos)
#define PWM_EGR_APLOAD2                 PWM_EGR_APLOAD2_Msk
#define PWM_EGR_APLOAD3_Pos             (18U)
#define PWM_EGR_APLOAD3_Msk             (0x1UL << PWM_EGR_APLOAD3_Pos)
#define PWM_EGR_APLOAD3                 PWM_EGR_APLOAD3_Msk
#define PWM_EGR_APLOAD4_Pos             (19U)
#define PWM_EGR_APLOAD4_Msk             (0x1UL << PWM_EGR_APLOAD4_Pos)
#define PWM_EGR_APLOAD4                 PWM_EGR_APLOAD4_Msk

/******************** Bit definition for PWM_CR1 register *********************/
#define PWM_CR1_CEN_Pos                 (0U)
#define PWM_CR1_CEN_Msk                 (0x1UL << PWM_CR1_CEN_Pos)
#define PWM_CR1_CEN                     PWM_CR1_CEN_Msk
#define PWM_CR1_UDIS_Pos                (1U)
#define PWM_CR1_UDIS_Msk                (0x1UL << PWM_CR1_UDIS_Pos)
#define PWM_CR1_UDIS                    PWM_CR1_UDIS_Msk
#define PWM_CR1_URS_Pos                 (2U)
#define PWM_CR1_URS_Msk                 (0x1UL << PWM_CR1_URS_Pos)
#define PWM_CR1_URS                     PWM_CR1_URS_Msk
#define PWM_CR1_OPM_Pos                 (3U)
#define PWM_CR1_OPM_Msk                 (0x1UL << PWM_CR1_OPM_Pos)
#define PWM_CR1_OPM                     PWM_CR1_OPM_Msk
#define PWM_CR1_CCE_Pos                 (4U)
#define PWM_CR1_CCE_Msk                 (0x1UL << PWM_CR1_CCE_Pos)
#define PWM_CR1_CCE                     PWM_CR1_CCE_Msk
#define PWM_CR1_CCP_Pos                 (5U)
#define PWM_CR1_CCP_Msk                 (0x1UL << PWM_CR1_CCP_Pos)
#define PWM_CR1_CCP                     PWM_CR1_CCP_Msk
#define PWM_CR1_OCPE_Pos                (6U)
#define PWM_CR1_OCPE_Msk                (0x1UL << PWM_CR1_OCPE_Pos)
#define PWM_CR1_OCPE                    PWM_CR1_OCPE_Msk
#define PWM_CR1_ARPE_Pos                (7U)
#define PWM_CR1_ARPE_Msk                (0x1UL << PWM_CR1_ARPE_Pos)
#define PWM_CR1_ARPE                    PWM_CR1_ARPE_Msk
#define PWM_CR1_OCM_Pos                 (8U)
#define PWM_CR1_OCM_Msk                 (0xFUL << PWM_CR1_OCM_Pos)
#define PWM_CR1_OCM                     PWM_CR1_OCM_Msk
#define PWM_CR1_APIDLE_Pos              (12U)
#define PWM_CR1_APIDLE_Msk              (0x1UL << PWM_CR1_APIDLE_Pos)
#define PWM_CR1_APIDLE                  PWM_CR1_APIDLE_Msk
#define PWM_CR1_APDMA_Pos               (13U)
#define PWM_CR1_APDMA_Msk               (0x1UL << PWM_CR1_APDMA_Pos)
#define PWM_CR1_APDMA                   PWM_CR1_APDMA_Msk
#define PWM_CR1_APMSB_Pos               (14U)
#define PWM_CR1_APMSB_Msk               (0x1UL << PWM_CR1_APMSB_Pos)
#define PWM_CR1_APMSB                   PWM_CR1_APMSB_Msk
#define PWM_CR1_SMS_Pos                 (16U)
#define PWM_CR1_SMS_Msk                 (0xFUL << PWM_CR1_SMS_Pos)
#define PWM_CR1_SMS                     PWM_CR1_SMS_Msk
#define PWM_CR1_TS_Pos                  (20U)
#define PWM_CR1_TS_Msk                  (0x7UL << PWM_CR1_TS_Pos)
#define PWM_CR1_TS                      PWM_CR1_TS_Msk
#define PWM_CR1_UFIE_Pos                (29U)
#define PWM_CR1_UFIE_Msk                (0x1UL << PWM_CR1_UFIE_Pos)
#define PWM_CR1_UFIE                    PWM_CR1_UFIE_Msk
#define PWM_CR1_CCIE_Pos                (30U)
#define PWM_CR1_CCIE_Msk                (0x1UL << PWM_CR1_CCIE_Pos)
#define PWM_CR1_CCIE                    PWM_CR1_CCIE_Msk
#define PWM_CR1_UIE_Pos                 (31U)
#define PWM_CR1_UIE_Msk                 (0x1UL << PWM_CR1_UIE_Pos)
#define PWM_CR1_UIE                     PWM_CR1_UIE_Msk

/******************** Bit definition for PWM_CNT1 register ********************/
#define PWM_CNT1_CNT_Pos                (0U)
#define PWM_CNT1_CNT_Msk                (0xFFFFFFFFUL << PWM_CNT1_CNT_Pos)
#define PWM_CNT1_CNT                    PWM_CNT1_CNT_Msk

/******************** Bit definition for PWM_PSC1 register ********************/
#define PWM_PSC1_PSC_Pos                (0U)
#define PWM_PSC1_PSC_Msk                (0xFFFFUL << PWM_PSC1_PSC_Pos)
#define PWM_PSC1_PSC                    PWM_PSC1_PSC_Msk

/******************** Bit definition for PWM_ARR1 register ********************/
#define PWM_ARR1_ARR_Pos                (0U)
#define PWM_ARR1_ARR_Msk                (0xFFFFFFFFUL << PWM_ARR1_ARR_Pos)
#define PWM_ARR1_ARR                    PWM_ARR1_ARR_Msk

/******************** Bit definition for PWM_RCR1 register ********************/
#define PWM_RCR1_REP_Pos                (0U)
#define PWM_RCR1_REP_Msk                (0xFFFFUL << PWM_RCR1_REP_Pos)
#define PWM_RCR1_REP                    PWM_RCR1_REP_Msk

/******************** Bit definition for PWM_CCR1 register ********************/
#define PWM_CCR1_CCR_Pos                (0U)
#define PWM_CCR1_CCR_Msk                (0xFFFFFFFFUL << PWM_CCR1_CCR_Pos)
#define PWM_CCR1_CCR                    PWM_CCR1_CCR_Msk

/******************* Bit definition for PWM_FIFO1 register ********************/
#define PWM_FIFO1_DATA_Pos              (0U)
#define PWM_FIFO1_DATA_Msk              (0xFFUL << PWM_FIFO1_DATA_Pos)
#define PWM_FIFO1_DATA                  PWM_FIFO1_DATA_Msk

/******************** Bit definition for PWM_CR2 register *********************/
#define PWM_CR2_CEN_Pos                 (0U)
#define PWM_CR2_CEN_Msk                 (0x1UL << PWM_CR2_CEN_Pos)
#define PWM_CR2_CEN                     PWM_CR2_CEN_Msk
#define PWM_CR2_UDIS_Pos                (1U)
#define PWM_CR2_UDIS_Msk                (0x1UL << PWM_CR2_UDIS_Pos)
#define PWM_CR2_UDIS                    PWM_CR2_UDIS_Msk
#define PWM_CR2_URS_Pos                 (2U)
#define PWM_CR2_URS_Msk                 (0x1UL << PWM_CR2_URS_Pos)
#define PWM_CR2_URS                     PWM_CR2_URS_Msk
#define PWM_CR2_OPM_Pos                 (3U)
#define PWM_CR2_OPM_Msk                 (0x1UL << PWM_CR2_OPM_Pos)
#define PWM_CR2_OPM                     PWM_CR2_OPM_Msk
#define PWM_CR2_CCE_Pos                 (4U)
#define PWM_CR2_CCE_Msk                 (0x1UL << PWM_CR2_CCE_Pos)
#define PWM_CR2_CCE                     PWM_CR2_CCE_Msk
#define PWM_CR2_CCP_Pos                 (5U)
#define PWM_CR2_CCP_Msk                 (0x1UL << PWM_CR2_CCP_Pos)
#define PWM_CR2_CCP                     PWM_CR2_CCP_Msk
#define PWM_CR2_OCPE_Pos                (6U)
#define PWM_CR2_OCPE_Msk                (0x1UL << PWM_CR2_OCPE_Pos)
#define PWM_CR2_OCPE                    PWM_CR2_OCPE_Msk
#define PWM_CR2_ARPE_Pos                (7U)
#define PWM_CR2_ARPE_Msk                (0x1UL << PWM_CR2_ARPE_Pos)
#define PWM_CR2_ARPE                    PWM_CR2_ARPE_Msk
#define PWM_CR2_OCM_Pos                 (8U)
#define PWM_CR2_OCM_Msk                 (0xFUL << PWM_CR2_OCM_Pos)
#define PWM_CR2_OCM                     PWM_CR2_OCM_Msk
#define PWM_CR2_APIDLE_Pos              (12U)
#define PWM_CR2_APIDLE_Msk              (0x1UL << PWM_CR2_APIDLE_Pos)
#define PWM_CR2_APIDLE                  PWM_CR2_APIDLE_Msk
#define PWM_CR2_APDMA_Pos               (13U)
#define PWM_CR2_APDMA_Msk               (0x1UL << PWM_CR2_APDMA_Pos)
#define PWM_CR2_APDMA                   PWM_CR2_APDMA_Msk
#define PWM_CR2_APMSB_Pos               (14U)
#define PWM_CR2_APMSB_Msk               (0x1UL << PWM_CR2_APMSB_Pos)
#define PWM_CR2_APMSB                   PWM_CR2_APMSB_Msk
#define PWM_CR2_SMS_Pos                 (16U)
#define PWM_CR2_SMS_Msk                 (0xFUL << PWM_CR2_SMS_Pos)
#define PWM_CR2_SMS                     PWM_CR2_SMS_Msk
#define PWM_CR2_TS_Pos                  (20U)
#define PWM_CR2_TS_Msk                  (0x7UL << PWM_CR2_TS_Pos)
#define PWM_CR2_TS                      PWM_CR2_TS_Msk
#define PWM_CR2_UFIE_Pos                (29U)
#define PWM_CR2_UFIE_Msk                (0x1UL << PWM_CR2_UFIE_Pos)
#define PWM_CR2_UFIE                    PWM_CR2_UFIE_Msk
#define PWM_CR2_CCIE_Pos                (30U)
#define PWM_CR2_CCIE_Msk                (0x1UL << PWM_CR2_CCIE_Pos)
#define PWM_CR2_CCIE                    PWM_CR2_CCIE_Msk
#define PWM_CR2_UIE_Pos                 (31U)
#define PWM_CR2_UIE_Msk                 (0x1UL << PWM_CR2_UIE_Pos)
#define PWM_CR2_UIE                     PWM_CR2_UIE_Msk

/******************** Bit definition for PWM_CNT2 register ********************/
#define PWM_CNT2_CNT_Pos                (0U)
#define PWM_CNT2_CNT_Msk                (0xFFFFFFFFUL << PWM_CNT2_CNT_Pos)
#define PWM_CNT2_CNT                    PWM_CNT2_CNT_Msk

/******************** Bit definition for PWM_PSC2 register ********************/
#define PWM_PSC2_PSC_Pos                (0U)
#define PWM_PSC2_PSC_Msk                (0xFFFFUL << PWM_PSC2_PSC_Pos)
#define PWM_PSC2_PSC                    PWM_PSC2_PSC_Msk

/******************** Bit definition for PWM_ARR2 register ********************/
#define PWM_ARR2_ARR_Pos                (0U)
#define PWM_ARR2_ARR_Msk                (0xFFFFFFFFUL << PWM_ARR2_ARR_Pos)
#define PWM_ARR2_ARR                    PWM_ARR2_ARR_Msk

/******************** Bit definition for PWM_RCR2 register ********************/
#define PWM_RCR2_REP_Pos                (0U)
#define PWM_RCR2_REP_Msk                (0xFFFFUL << PWM_RCR2_REP_Pos)
#define PWM_RCR2_REP                    PWM_RCR2_REP_Msk

/******************** Bit definition for PWM_CCR2 register ********************/
#define PWM_CCR2_CCR_Pos                (0U)
#define PWM_CCR2_CCR_Msk                (0xFFFFFFFFUL << PWM_CCR2_CCR_Pos)
#define PWM_CCR2_CCR                    PWM_CCR2_CCR_Msk

/******************* Bit definition for PWM_FIFO2 register ********************/
#define PWM_FIFO2_DATA_Pos              (0U)
#define PWM_FIFO2_DATA_Msk              (0xFFUL << PWM_FIFO2_DATA_Pos)
#define PWM_FIFO2_DATA                  PWM_FIFO2_DATA_Msk

/******************** Bit definition for PWM_CR3 register *********************/
#define PWM_CR3_CEN_Pos                 (0U)
#define PWM_CR3_CEN_Msk                 (0x1UL << PWM_CR3_CEN_Pos)
#define PWM_CR3_CEN                     PWM_CR3_CEN_Msk
#define PWM_CR3_UDIS_Pos                (1U)
#define PWM_CR3_UDIS_Msk                (0x1UL << PWM_CR3_UDIS_Pos)
#define PWM_CR3_UDIS                    PWM_CR3_UDIS_Msk
#define PWM_CR3_URS_Pos                 (2U)
#define PWM_CR3_URS_Msk                 (0x1UL << PWM_CR3_URS_Pos)
#define PWM_CR3_URS                     PWM_CR3_URS_Msk
#define PWM_CR3_OPM_Pos                 (3U)
#define PWM_CR3_OPM_Msk                 (0x1UL << PWM_CR3_OPM_Pos)
#define PWM_CR3_OPM                     PWM_CR3_OPM_Msk
#define PWM_CR3_CCE_Pos                 (4U)
#define PWM_CR3_CCE_Msk                 (0x1UL << PWM_CR3_CCE_Pos)
#define PWM_CR3_CCE                     PWM_CR3_CCE_Msk
#define PWM_CR3_CCP_Pos                 (5U)
#define PWM_CR3_CCP_Msk                 (0x1UL << PWM_CR3_CCP_Pos)
#define PWM_CR3_CCP                     PWM_CR3_CCP_Msk
#define PWM_CR3_OCPE_Pos                (6U)
#define PWM_CR3_OCPE_Msk                (0x1UL << PWM_CR3_OCPE_Pos)
#define PWM_CR3_OCPE                    PWM_CR3_OCPE_Msk
#define PWM_CR3_ARPE_Pos                (7U)
#define PWM_CR3_ARPE_Msk                (0x1UL << PWM_CR3_ARPE_Pos)
#define PWM_CR3_ARPE                    PWM_CR3_ARPE_Msk
#define PWM_CR3_OCM_Pos                 (8U)
#define PWM_CR3_OCM_Msk                 (0xFUL << PWM_CR3_OCM_Pos)
#define PWM_CR3_OCM                     PWM_CR3_OCM_Msk
#define PWM_CR3_APIDLE_Pos              (12U)
#define PWM_CR3_APIDLE_Msk              (0x1UL << PWM_CR3_APIDLE_Pos)
#define PWM_CR3_APIDLE                  PWM_CR3_APIDLE_Msk
#define PWM_CR3_APDMA_Pos               (13U)
#define PWM_CR3_APDMA_Msk               (0x1UL << PWM_CR3_APDMA_Pos)
#define PWM_CR3_APDMA                   PWM_CR3_APDMA_Msk
#define PWM_CR3_APMSB_Pos               (14U)
#define PWM_CR3_APMSB_Msk               (0x1UL << PWM_CR3_APMSB_Pos)
#define PWM_CR3_APMSB                   PWM_CR3_APMSB_Msk
#define PWM_CR3_SMS_Pos                 (16U)
#define PWM_CR3_SMS_Msk                 (0xFUL << PWM_CR3_SMS_Pos)
#define PWM_CR3_SMS                     PWM_CR3_SMS_Msk
#define PWM_CR3_TS_Pos                  (20U)
#define PWM_CR3_TS_Msk                  (0x7UL << PWM_CR3_TS_Pos)
#define PWM_CR3_TS                      PWM_CR3_TS_Msk
#define PWM_CR3_UFIE_Pos                (29U)
#define PWM_CR3_UFIE_Msk                (0x1UL << PWM_CR3_UFIE_Pos)
#define PWM_CR3_UFIE                    PWM_CR3_UFIE_Msk
#define PWM_CR3_CCIE_Pos                (30U)
#define PWM_CR3_CCIE_Msk                (0x1UL << PWM_CR3_CCIE_Pos)
#define PWM_CR3_CCIE                    PWM_CR3_CCIE_Msk
#define PWM_CR3_UIE_Pos                 (31U)
#define PWM_CR3_UIE_Msk                 (0x1UL << PWM_CR3_UIE_Pos)
#define PWM_CR3_UIE                     PWM_CR3_UIE_Msk

/******************** Bit definition for PWM_CNT3 register ********************/
#define PWM_CNT3_CNT_Pos                (0U)
#define PWM_CNT3_CNT_Msk                (0xFFFFFFFFUL << PWM_CNT3_CNT_Pos)
#define PWM_CNT3_CNT                    PWM_CNT3_CNT_Msk

/******************** Bit definition for PWM_PSC3 register ********************/
#define PWM_PSC3_PSC_Pos                (0U)
#define PWM_PSC3_PSC_Msk                (0xFFFFUL << PWM_PSC3_PSC_Pos)
#define PWM_PSC3_PSC                    PWM_PSC3_PSC_Msk

/******************** Bit definition for PWM_ARR3 register ********************/
#define PWM_ARR3_ARR_Pos                (0U)
#define PWM_ARR3_ARR_Msk                (0xFFFFFFFFUL << PWM_ARR3_ARR_Pos)
#define PWM_ARR3_ARR                    PWM_ARR3_ARR_Msk

/******************** Bit definition for PWM_RCR3 register ********************/
#define PWM_RCR3_REP_Pos                (0U)
#define PWM_RCR3_REP_Msk                (0xFFFFUL << PWM_RCR3_REP_Pos)
#define PWM_RCR3_REP                    PWM_RCR3_REP_Msk

/******************** Bit definition for PWM_CCR3 register ********************/
#define PWM_CCR3_CCR_Pos                (0U)
#define PWM_CCR3_CCR_Msk                (0xFFFFFFFFUL << PWM_CCR3_CCR_Pos)
#define PWM_CCR3_CCR                    PWM_CCR3_CCR_Msk

/******************* Bit definition for PWM_FIFO3 register ********************/
#define PWM_FIFO3_DATA_Pos              (0U)
#define PWM_FIFO3_DATA_Msk              (0xFFUL << PWM_FIFO3_DATA_Pos)
#define PWM_FIFO3_DATA                  PWM_FIFO3_DATA_Msk

/******************** Bit definition for PWM_CR4 register *********************/
#define PWM_CR4_CEN_Pos                 (0U)
#define PWM_CR4_CEN_Msk                 (0x1UL << PWM_CR4_CEN_Pos)
#define PWM_CR4_CEN                     PWM_CR4_CEN_Msk
#define PWM_CR4_UDIS_Pos                (1U)
#define PWM_CR4_UDIS_Msk                (0x1UL << PWM_CR4_UDIS_Pos)
#define PWM_CR4_UDIS                    PWM_CR4_UDIS_Msk
#define PWM_CR4_URS_Pos                 (2U)
#define PWM_CR4_URS_Msk                 (0x1UL << PWM_CR4_URS_Pos)
#define PWM_CR4_URS                     PWM_CR4_URS_Msk
#define PWM_CR4_OPM_Pos                 (3U)
#define PWM_CR4_OPM_Msk                 (0x1UL << PWM_CR4_OPM_Pos)
#define PWM_CR4_OPM                     PWM_CR4_OPM_Msk
#define PWM_CR4_CCE_Pos                 (4U)
#define PWM_CR4_CCE_Msk                 (0x1UL << PWM_CR4_CCE_Pos)
#define PWM_CR4_CCE                     PWM_CR4_CCE_Msk
#define PWM_CR4_CCP_Pos                 (5U)
#define PWM_CR4_CCP_Msk                 (0x1UL << PWM_CR4_CCP_Pos)
#define PWM_CR4_CCP                     PWM_CR4_CCP_Msk
#define PWM_CR4_OCPE_Pos                (6U)
#define PWM_CR4_OCPE_Msk                (0x1UL << PWM_CR4_OCPE_Pos)
#define PWM_CR4_OCPE                    PWM_CR4_OCPE_Msk
#define PWM_CR4_ARPE_Pos                (7U)
#define PWM_CR4_ARPE_Msk                (0x1UL << PWM_CR4_ARPE_Pos)
#define PWM_CR4_ARPE                    PWM_CR4_ARPE_Msk
#define PWM_CR4_OCM_Pos                 (8U)
#define PWM_CR4_OCM_Msk                 (0xFUL << PWM_CR4_OCM_Pos)
#define PWM_CR4_OCM                     PWM_CR4_OCM_Msk
#define PWM_CR4_APIDLE_Pos              (12U)
#define PWM_CR4_APIDLE_Msk              (0x1UL << PWM_CR4_APIDLE_Pos)
#define PWM_CR4_APIDLE                  PWM_CR4_APIDLE_Msk
#define PWM_CR4_APDMA_Pos               (13U)
#define PWM_CR4_APDMA_Msk               (0x1UL << PWM_CR4_APDMA_Pos)
#define PWM_CR4_APDMA                   PWM_CR4_APDMA_Msk
#define PWM_CR4_APMSB_Pos               (14U)
#define PWM_CR4_APMSB_Msk               (0x1UL << PWM_CR4_APMSB_Pos)
#define PWM_CR4_APMSB                   PWM_CR4_APMSB_Msk
#define PWM_CR4_SMS_Pos                 (16U)
#define PWM_CR4_SMS_Msk                 (0xFUL << PWM_CR4_SMS_Pos)
#define PWM_CR4_SMS                     PWM_CR4_SMS_Msk
#define PWM_CR4_TS_Pos                  (20U)
#define PWM_CR4_TS_Msk                  (0x7UL << PWM_CR4_TS_Pos)
#define PWM_CR4_TS                      PWM_CR4_TS_Msk
#define PWM_CR4_UFIE_Pos                (29U)
#define PWM_CR4_UFIE_Msk                (0x1UL << PWM_CR4_UFIE_Pos)
#define PWM_CR4_UFIE                    PWM_CR4_UFIE_Msk
#define PWM_CR4_CCIE_Pos                (30U)
#define PWM_CR4_CCIE_Msk                (0x1UL << PWM_CR4_CCIE_Pos)
#define PWM_CR4_CCIE                    PWM_CR4_CCIE_Msk
#define PWM_CR4_UIE_Pos                 (31U)
#define PWM_CR4_UIE_Msk                 (0x1UL << PWM_CR4_UIE_Pos)
#define PWM_CR4_UIE                     PWM_CR4_UIE_Msk

/******************** Bit definition for PWM_CNT4 register ********************/
#define PWM_CNT4_CNT_Pos                (0U)
#define PWM_CNT4_CNT_Msk                (0xFFFFFFFFUL << PWM_CNT4_CNT_Pos)
#define PWM_CNT4_CNT                    PWM_CNT4_CNT_Msk

/******************** Bit definition for PWM_PSC4 register ********************/
#define PWM_PSC4_PSC_Pos                (0U)
#define PWM_PSC4_PSC_Msk                (0xFFFFUL << PWM_PSC4_PSC_Pos)
#define PWM_PSC4_PSC                    PWM_PSC4_PSC_Msk

/******************** Bit definition for PWM_ARR4 register ********************/
#define PWM_ARR4_ARR_Pos                (0U)
#define PWM_ARR4_ARR_Msk                (0xFFFFFFFFUL << PWM_ARR4_ARR_Pos)
#define PWM_ARR4_ARR                    PWM_ARR4_ARR_Msk

/******************** Bit definition for PWM_RCR4 register ********************/
#define PWM_RCR4_REP_Pos                (0U)
#define PWM_RCR4_REP_Msk                (0xFFFFUL << PWM_RCR4_REP_Pos)
#define PWM_RCR4_REP                    PWM_RCR4_REP_Msk

/******************** Bit definition for PWM_CCR4 register ********************/
#define PWM_CCR4_CCR_Pos                (0U)
#define PWM_CCR4_CCR_Msk                (0xFFFFFFFFUL << PWM_CCR4_CCR_Pos)
#define PWM_CCR4_CCR                    PWM_CCR4_CCR_Msk

/******************* Bit definition for PWM_FIFO4 register ********************/
#define PWM_FIFO4_DATA_Pos              (0U)
#define PWM_FIFO4_DATA_Msk              (0xFFUL << PWM_FIFO4_DATA_Pos)
#define PWM_FIFO4_DATA                  PWM_FIFO4_DATA_Msk

/******************** Bit definition for PWM_CR5 register *********************/
#define PWM_CR5_CEN_Pos                 (0U)
#define PWM_CR5_CEN_Msk                 (0x1UL << PWM_CR5_CEN_Pos)
#define PWM_CR5_CEN                     PWM_CR5_CEN_Msk
#define PWM_CR5_UDIS_Pos                (1U)
#define PWM_CR5_UDIS_Msk                (0x1UL << PWM_CR5_UDIS_Pos)
#define PWM_CR5_UDIS                    PWM_CR5_UDIS_Msk
#define PWM_CR5_URS_Pos                 (2U)
#define PWM_CR5_URS_Msk                 (0x1UL << PWM_CR5_URS_Pos)
#define PWM_CR5_URS                     PWM_CR5_URS_Msk
#define PWM_CR5_OPM_Pos                 (3U)
#define PWM_CR5_OPM_Msk                 (0x1UL << PWM_CR5_OPM_Pos)
#define PWM_CR5_OPM                     PWM_CR5_OPM_Msk
#define PWM_CR5_CCE_Pos                 (4U)
#define PWM_CR5_CCE_Msk                 (0x1UL << PWM_CR5_CCE_Pos)
#define PWM_CR5_CCE                     PWM_CR5_CCE_Msk
#define PWM_CR5_CCP_Pos                 (5U)
#define PWM_CR5_CCP_Msk                 (0x1UL << PWM_CR5_CCP_Pos)
#define PWM_CR5_CCP                     PWM_CR5_CCP_Msk
#define PWM_CR5_UIE_Pos                 (31U)
#define PWM_CR5_UIE_Msk                 (0x1UL << PWM_CR5_UIE_Pos)
#define PWM_CR5_UIE                     PWM_CR5_UIE_Msk

/******************** Bit definition for PWM_CNT5 register ********************/
#define PWM_CNT5_CNT_Pos                (0U)
#define PWM_CNT5_CNT_Msk                (0xFFFFUL << PWM_CNT5_CNT_Pos)
#define PWM_CNT5_CNT                    PWM_CNT5_CNT_Msk

/******************** Bit definition for PWM_PSC5 register ********************/
#define PWM_PSC5_PSC_Pos                (0U)
#define PWM_PSC5_PSC_Msk                (0xFFFFUL << PWM_PSC5_PSC_Pos)
#define PWM_PSC5_PSC                    PWM_PSC5_PSC_Msk

/******************** Bit definition for PWM_ARR5 register ********************/
#define PWM_ARR5_ARR_Pos                (0U)
#define PWM_ARR5_ARR_Msk                (0xFFFFUL << PWM_ARR5_ARR_Pos)
#define PWM_ARR5_ARR                    PWM_ARR5_ARR_Msk

/******************** Bit definition for PWM_RCR5 register ********************/
#define PWM_RCR5_REP_Pos                (0U)
#define PWM_RCR5_REP_Msk                (0xFFUL << PWM_RCR5_REP_Pos)
#define PWM_RCR5_REP                    PWM_RCR5_REP_Msk

/******************* Bit definition for PWM_INTV5 register ********************/
#define PWM_INTV5_IDLE_Pos              (0U)
#define PWM_INTV5_IDLE_Msk              (0xFFFFUL << PWM_INTV5_IDLE_Pos)
#define PWM_INTV5_IDLE                  PWM_INTV5_IDLE_Msk
#define PWM_INTV5_PEAK_Pos              (16U)
#define PWM_INTV5_PEAK_Msk              (0xFFFFUL << PWM_INTV5_PEAK_Pos)
#define PWM_INTV5_PEAK                  PWM_INTV5_PEAK_Msk

#endif
