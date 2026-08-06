/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EPIC_H
#define __EPIC_H

typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t TL_POS;
    __IO uint32_t BR_POS;
    __IO uint32_t FILTER;
    __IO uint32_t SRC;
    __IO uint32_t FILL;
    __IO uint32_t MISC_CFG;
    __IO uint32_t RSVD;
} EPIC_LayerxTypeDef;


typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t TL_POS;
    __IO uint32_t BR_POS;
    __IO uint32_t EXTENTS;
    __IO uint32_t FILTER;
    __IO uint32_t SRC;
    __IO uint32_t ROT;
    __IO uint32_t ROT_STAT;
    __IO uint32_t SCALE_RATIO_H;
    __IO uint32_t SCALE_RATIO_V;
    __IO uint32_t FILL;
    __IO uint32_t MISC_CFG;
    __IO uint32_t RSVD[17];
} EPIC_VideoLayerxTypeDef;


typedef struct
{
    __IO uint32_t ROT_M_CFG1;
    __IO uint32_t ROT_M_CFG2;
    __IO uint32_t ROT_M_CFG3;
    __IO uint32_t SCALE_INIT_CFG1;
    __IO uint32_t SCALE_INIT_CFG2;
} EPIC_VideoLayerxTransTypeDef;


typedef struct
{
    __IO uint32_t COMMAND;
    __IO uint32_t STATUS;
    __IO uint32_t EOF_IRQ;
    __IO uint32_t SETTING;
    __IO uint32_t CANVAS_TL_POS;
    __IO uint32_t CANVAS_BR_POS;
    __IO uint32_t CANVAS_BG;
    __IO uint32_t VL_CFG;
    __IO uint32_t VL_TL_POS;
    __IO uint32_t VL_BR_POS;
    __IO uint32_t VL_EXTENTS;
    __IO uint32_t VL_FILTER;
    __IO uint32_t VL_SRC;
    __IO uint32_t VL_ROT;
    __IO uint32_t VL_ROT_STAT;
    __IO uint32_t VL_SCALE_RATIO_H;
    __IO uint32_t VL_SCALE_RATIO_V;
    __IO uint32_t VL_FILL;
    __IO uint32_t VL_MISC_CFG;
    __IO uint32_t RSVD4;
    __IO uint32_t L0_CFG;
    __IO uint32_t L0_TL_POS;
    __IO uint32_t L0_BR_POS;
    __IO uint32_t L0_FILTER;
    __IO uint32_t L0_SRC;
    __IO uint32_t L0_FILL;
    __IO uint32_t L0_MISC_CFG;
    __IO uint32_t RSVD3;
    __IO uint32_t L1_CFG;
    __IO uint32_t L1_TL_POS;
    __IO uint32_t L1_BR_POS;
    __IO uint32_t L1_FILTER;
    __IO uint32_t L1_SRC;
    __IO uint32_t L1_FILL;
    __IO uint32_t L1_MISC_CFG;
    __IO uint32_t RSVD2;
    __IO uint32_t L2_CFG;
    __IO uint32_t L2_TL_POS;
    __IO uint32_t L2_BR_POS;
    __IO uint32_t L2_FILTER;
    __IO uint32_t L2_SRC;
    __IO uint32_t L2_FILL;
    __IO uint32_t L2_MISC_CFG;
    __IO uint32_t L2_MAT_A0;
    __IO uint32_t L2_MAT_A1;
    __IO uint32_t L2_MAT_A2;
    __IO uint32_t L2_MAT_B0;
    __IO uint32_t L2_MAT_B1;
    __IO uint32_t L2_MAT_B2;
    __IO uint32_t L2_MAT_C0;
    __IO uint32_t L2_MAT_C1;
    __IO uint32_t L2_MAT_C2;
    __IO uint32_t RSVD1[4];
    __IO uint32_t MASK_CFG;
    __IO uint32_t MASK_TL_POS;
    __IO uint32_t MASK_BR_POS;
    __IO uint32_t MASK_SRC;
    __IO uint32_t COENG_CFG;
    __IO uint32_t JPEG_ENG_CFG0;
    __IO uint32_t JPEG_ENG_CFG1;
    __IO uint32_t DECOMP_CFG0;
    __IO uint32_t DECOMP_CFG1;
    __IO uint32_t DECOMP_CFG2;
    __IO uint32_t DECOMP_STAT;
    __IO uint32_t YUV_ENG_CFG0;
    __IO uint32_t YUV_ENG_CFG1;
    __IO uint32_t Y_SRC;
    __IO uint32_t U_SRC;
    __IO uint32_t V_SRC;
    __IO uint32_t COEF0;
    __IO uint32_t COEF1;
    __IO uint32_t CM_ENG_CONF0;
    __IO uint32_t CM_ENG_CONF1;
    __IO uint32_t CM_ENG_CONF2;
    __IO uint32_t CM_ENG_CONF3;
    __IO uint32_t CM_ENG_CONF4;
    __IO uint32_t CM_ENG_CONF5;
    __IO uint32_t CM_ENG_CONF6;
    __IO uint32_t CM_ENG_CONF7;
    __IO uint32_t DITHER_CONF;
    __IO uint32_t DITHER_LFSR;
    __IO uint32_t GREY_CONV;
    __IO uint32_t FLEX_SEL_L;
    __IO uint32_t FLEX_SEL_H;
    __IO uint32_t AHB_CTRL;
    __IO uint32_t AHB_MEM;
    __IO uint32_t AHB_STRIDE;
    __IO uint32_t DEBUG;
    __IO uint32_t VL_ROT_M_CFG1;
    __IO uint32_t VL_ROT_M_CFG2;
    __IO uint32_t VL_ROT_M_CFG3;
    __IO uint32_t VL_SCALE_INIT_CFG1;
    __IO uint32_t VL_SCALE_INIT_CFG2;
    __IO uint32_t AHBA_CNT;
    __IO uint32_t AHBB_CNT;
    __IO uint32_t PERF_CNT;
    __IO uint32_t TL_CFG;
    __IO uint32_t TL_SIZE;
    __IO uint32_t TL_SRC;
    __IO uint32_t CMPRCR;
    __IO uint32_t CMPRCFG0;
    __IO uint32_t CMPRCFG1;
    __IO uint32_t CMPRQR;
    __IO uint32_t CMPRDR;
    __IO uint32_t TL_GREY_CONV;
    __IO uint32_t TL_FLEX_SEL_L;
    __IO uint32_t TL_FLEX_SEL_H;
    __IO uint32_t TL_AHB_CTRL;
    __IO uint32_t TL_AHB_MEM;
    __IO uint32_t TL_AHB_STRIDE;
    __IO uint32_t TL_COMMAND;
    __IO uint32_t TL_STATUS;
    __IO uint32_t TL_IRQ;
    __IO uint32_t TL_SETTING;
    __IO uint32_t TL_PERF_CNT;
    __IO uint32_t CANVAS_STAT0;
    __IO uint32_t CANVAS_STAT1;
    __IO uint32_t ENG_STAT;
    __IO uint32_t OL_STAT;
    __IO uint32_t OL2_STAT;
    __IO uint32_t VL_STAT;
    __IO uint32_t ML_STAT;
    __IO uint32_t TL_STAT0;
    __IO uint32_t TL_STAT1;
    __IO uint32_t MEM_IF_STAT;
} EPIC_TypeDef;


/****************** Bit definition for EPIC_COMMAND register ******************/
#define EPIC_COMMAND_START_Pos          (0U)
#define EPIC_COMMAND_START_Msk          (0x1UL << EPIC_COMMAND_START_Pos)
#define EPIC_COMMAND_START              EPIC_COMMAND_START_Msk
#define EPIC_COMMAND_RESET_Pos          (1U)
#define EPIC_COMMAND_RESET_Msk          (0x1UL << EPIC_COMMAND_RESET_Pos)
#define EPIC_COMMAND_RESET              EPIC_COMMAND_RESET_Msk

/****************** Bit definition for EPIC_STATUS register *******************/
#define EPIC_STATUS_IA_BUSY_Pos         (0U)
#define EPIC_STATUS_IA_BUSY_Msk         (0x1UL << EPIC_STATUS_IA_BUSY_Pos)
#define EPIC_STATUS_IA_BUSY             EPIC_STATUS_IA_BUSY_Msk
#define EPIC_STATUS_LCD_BUSY_Pos        (1U)
#define EPIC_STATUS_LCD_BUSY_Msk        (0x1UL << EPIC_STATUS_LCD_BUSY_Pos)
#define EPIC_STATUS_LCD_BUSY            EPIC_STATUS_LCD_BUSY_Msk

/****************** Bit definition for EPIC_EOF_IRQ register ******************/
#define EPIC_EOF_IRQ_IRQ_CAUSE_Pos      (0U)
#define EPIC_EOF_IRQ_IRQ_CAUSE_Msk      (0x1UL << EPIC_EOF_IRQ_IRQ_CAUSE_Pos)
#define EPIC_EOF_IRQ_IRQ_CAUSE          EPIC_EOF_IRQ_IRQ_CAUSE_Msk
#define EPIC_EOF_IRQ_LINE_HIT_CAUSE_Pos  (1U)
#define EPIC_EOF_IRQ_LINE_HIT_CAUSE_Msk  (0x1UL << EPIC_EOF_IRQ_LINE_HIT_CAUSE_Pos)
#define EPIC_EOF_IRQ_LINE_HIT_CAUSE     EPIC_EOF_IRQ_LINE_HIT_CAUSE_Msk
#define EPIC_EOF_IRQ_ICB_OF_CAUSE_Pos   (2U)
#define EPIC_EOF_IRQ_ICB_OF_CAUSE_Msk   (0x1UL << EPIC_EOF_IRQ_ICB_OF_CAUSE_Pos)
#define EPIC_EOF_IRQ_ICB_OF_CAUSE       EPIC_EOF_IRQ_ICB_OF_CAUSE_Msk
#define EPIC_EOF_IRQ_Z1_CALC_ERR_CAUSE_Pos  (3U)
#define EPIC_EOF_IRQ_Z1_CALC_ERR_CAUSE_Msk  (0x1UL << EPIC_EOF_IRQ_Z1_CALC_ERR_CAUSE_Pos)
#define EPIC_EOF_IRQ_Z1_CALC_ERR_CAUSE  EPIC_EOF_IRQ_Z1_CALC_ERR_CAUSE_Msk
#define EPIC_EOF_IRQ_IRQ_STATUS_Pos     (16U)
#define EPIC_EOF_IRQ_IRQ_STATUS_Msk     (0x1UL << EPIC_EOF_IRQ_IRQ_STATUS_Pos)
#define EPIC_EOF_IRQ_IRQ_STATUS         EPIC_EOF_IRQ_IRQ_STATUS_Msk
#define EPIC_EOF_IRQ_LINE_HIT_STATUS_Pos  (17U)
#define EPIC_EOF_IRQ_LINE_HIT_STATUS_Msk  (0x1UL << EPIC_EOF_IRQ_LINE_HIT_STATUS_Pos)
#define EPIC_EOF_IRQ_LINE_HIT_STATUS    EPIC_EOF_IRQ_LINE_HIT_STATUS_Msk
#define EPIC_EOF_IRQ_ICB_OF_STATUS_Pos  (18U)
#define EPIC_EOF_IRQ_ICB_OF_STATUS_Msk  (0x1UL << EPIC_EOF_IRQ_ICB_OF_STATUS_Pos)
#define EPIC_EOF_IRQ_ICB_OF_STATUS      EPIC_EOF_IRQ_ICB_OF_STATUS_Msk
#define EPIC_EOF_IRQ_Z1_CALC_ERR_STATUS_Pos  (19U)
#define EPIC_EOF_IRQ_Z1_CALC_ERR_STATUS_Msk  (0x1UL << EPIC_EOF_IRQ_Z1_CALC_ERR_STATUS_Pos)
#define EPIC_EOF_IRQ_Z1_CALC_ERR_STATUS  EPIC_EOF_IRQ_Z1_CALC_ERR_STATUS_Msk

/****************** Bit definition for EPIC_SETTING register ******************/
#define EPIC_SETTING_EOF_IRQ_MASK_Pos   (0U)
#define EPIC_SETTING_EOF_IRQ_MASK_Msk   (0x1UL << EPIC_SETTING_EOF_IRQ_MASK_Pos)
#define EPIC_SETTING_EOF_IRQ_MASK       EPIC_SETTING_EOF_IRQ_MASK_Msk
#define EPIC_SETTING_LINE_IRQ_MASK_Pos  (1U)
#define EPIC_SETTING_LINE_IRQ_MASK_Msk  (0x1UL << EPIC_SETTING_LINE_IRQ_MASK_Pos)
#define EPIC_SETTING_LINE_IRQ_MASK      EPIC_SETTING_LINE_IRQ_MASK_Msk
#define EPIC_SETTING_ICB_OF_IRQ_MASK_Pos  (2U)
#define EPIC_SETTING_ICB_OF_IRQ_MASK_Msk  (0x1UL << EPIC_SETTING_ICB_OF_IRQ_MASK_Pos)
#define EPIC_SETTING_ICB_OF_IRQ_MASK    EPIC_SETTING_ICB_OF_IRQ_MASK_Msk
#define EPIC_SETTING_Z1_CALC_ERR_MASK_Pos  (3U)
#define EPIC_SETTING_Z1_CALC_ERR_MASK_Msk  (0x1UL << EPIC_SETTING_Z1_CALC_ERR_MASK_Pos)
#define EPIC_SETTING_Z1_CALC_ERR_MASK   EPIC_SETTING_Z1_CALC_ERR_MASK_Msk
#define EPIC_SETTING_AUTO_GATE_EN_Pos   (4U)
#define EPIC_SETTING_AUTO_GATE_EN_Msk   (0x1UL << EPIC_SETTING_AUTO_GATE_EN_Pos)
#define EPIC_SETTING_AUTO_GATE_EN       EPIC_SETTING_AUTO_GATE_EN_Msk
#define EPIC_SETTING_LINE_IRQ_NUM_Pos   (16U)
#define EPIC_SETTING_LINE_IRQ_NUM_Msk   (0x1FFFUL << EPIC_SETTING_LINE_IRQ_NUM_Pos)
#define EPIC_SETTING_LINE_IRQ_NUM       EPIC_SETTING_LINE_IRQ_NUM_Msk

/*************** Bit definition for EPIC_CANVAS_TL_POS register ***************/
#define EPIC_CANVAS_TL_POS_X0_Pos       (0U)
#define EPIC_CANVAS_TL_POS_X0_Msk       (0x1FFFUL << EPIC_CANVAS_TL_POS_X0_Pos)
#define EPIC_CANVAS_TL_POS_X0           EPIC_CANVAS_TL_POS_X0_Msk
#define EPIC_CANVAS_TL_POS_Y0_Pos       (16U)
#define EPIC_CANVAS_TL_POS_Y0_Msk       (0x1FFFUL << EPIC_CANVAS_TL_POS_Y0_Pos)
#define EPIC_CANVAS_TL_POS_Y0           EPIC_CANVAS_TL_POS_Y0_Msk

/*************** Bit definition for EPIC_CANVAS_BR_POS register ***************/
#define EPIC_CANVAS_BR_POS_X1_Pos       (0U)
#define EPIC_CANVAS_BR_POS_X1_Msk       (0x1FFFUL << EPIC_CANVAS_BR_POS_X1_Pos)
#define EPIC_CANVAS_BR_POS_X1           EPIC_CANVAS_BR_POS_X1_Msk
#define EPIC_CANVAS_BR_POS_Y1_Pos       (16U)
#define EPIC_CANVAS_BR_POS_Y1_Msk       (0x1FFFUL << EPIC_CANVAS_BR_POS_Y1_Pos)
#define EPIC_CANVAS_BR_POS_Y1           EPIC_CANVAS_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_CANVAS_BG register *****************/
#define EPIC_CANVAS_BG_BLUE_Pos         (0U)
#define EPIC_CANVAS_BG_BLUE_Msk         (0xFFUL << EPIC_CANVAS_BG_BLUE_Pos)
#define EPIC_CANVAS_BG_BLUE             EPIC_CANVAS_BG_BLUE_Msk
#define EPIC_CANVAS_BG_GREEN_Pos        (8U)
#define EPIC_CANVAS_BG_GREEN_Msk        (0xFFUL << EPIC_CANVAS_BG_GREEN_Pos)
#define EPIC_CANVAS_BG_GREEN            EPIC_CANVAS_BG_GREEN_Msk
#define EPIC_CANVAS_BG_RED_Pos          (16U)
#define EPIC_CANVAS_BG_RED_Msk          (0xFFUL << EPIC_CANVAS_BG_RED_Pos)
#define EPIC_CANVAS_BG_RED              EPIC_CANVAS_BG_RED_Msk
#define EPIC_CANVAS_BG_BG_BLENDING_BYPASS_Pos  (24U)
#define EPIC_CANVAS_BG_BG_BLENDING_BYPASS_Msk  (0x1UL << EPIC_CANVAS_BG_BG_BLENDING_BYPASS_Pos)
#define EPIC_CANVAS_BG_BG_BLENDING_BYPASS  EPIC_CANVAS_BG_BG_BLENDING_BYPASS_Msk
#define EPIC_CANVAS_BG_ALL_BLENDING_BYPASS_Pos  (25U)
#define EPIC_CANVAS_BG_ALL_BLENDING_BYPASS_Msk  (0x1UL << EPIC_CANVAS_BG_ALL_BLENDING_BYPASS_Pos)
#define EPIC_CANVAS_BG_ALL_BLENDING_BYPASS  EPIC_CANVAS_BG_ALL_BLENDING_BYPASS_Msk

/****************** Bit definition for EPIC_VL_CFG register *******************/
#define EPIC_VL_CFG_FORMAT_Pos          (0U)
#define EPIC_VL_CFG_FORMAT_Msk          (0xFUL << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_FORMAT              EPIC_VL_CFG_FORMAT_Msk
#define EPIC_VL_CFG_FMT_RGB565          (0 << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_FMT_RGB888          (1 << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_FMT_ARGB8888        (2 << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_FMT_ARGB8565        (3 << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_FMT_A8              (4 << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_FMT_A4              (5 << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_FMT_L8              (6 << EPIC_VL_CFG_FORMAT_Pos)
#define EPIC_VL_CFG_ALPHA_SEL_Pos       (4U)
#define EPIC_VL_CFG_ALPHA_SEL_Msk       (0x1UL << EPIC_VL_CFG_ALPHA_SEL_Pos)
#define EPIC_VL_CFG_ALPHA_SEL           EPIC_VL_CFG_ALPHA_SEL_Msk
#define EPIC_VL_CFG_ALPHA_Pos           (5U)
#define EPIC_VL_CFG_ALPHA_Msk           (0xFFUL << EPIC_VL_CFG_ALPHA_Pos)
#define EPIC_VL_CFG_ALPHA               EPIC_VL_CFG_ALPHA_Msk
#define EPIC_VL_CFG_ALPHA_BLEND_Pos     (13U)
#define EPIC_VL_CFG_ALPHA_BLEND_Msk     (0x1UL << EPIC_VL_CFG_ALPHA_BLEND_Pos)
#define EPIC_VL_CFG_ALPHA_BLEND         EPIC_VL_CFG_ALPHA_BLEND_Msk
#define EPIC_VL_CFG_FILTER_EN_Pos       (14U)
#define EPIC_VL_CFG_FILTER_EN_Msk       (0x1UL << EPIC_VL_CFG_FILTER_EN_Pos)
#define EPIC_VL_CFG_FILTER_EN           EPIC_VL_CFG_FILTER_EN_Msk
#define EPIC_VL_CFG_WIDTH_Pos           (16U)
#define EPIC_VL_CFG_WIDTH_Msk           (0x7FFFUL << EPIC_VL_CFG_WIDTH_Pos)
#define EPIC_VL_CFG_WIDTH               EPIC_VL_CFG_WIDTH_Msk
#define EPIC_VL_CFG_ACTIVE_Pos          (31U)
#define EPIC_VL_CFG_ACTIVE_Msk          (0x1UL << EPIC_VL_CFG_ACTIVE_Pos)
#define EPIC_VL_CFG_ACTIVE              EPIC_VL_CFG_ACTIVE_Msk

/***************** Bit definition for EPIC_VL_TL_POS register *****************/
#define EPIC_VL_TL_POS_X0_Pos           (0U)
#define EPIC_VL_TL_POS_X0_Msk           (0x1FFFUL << EPIC_VL_TL_POS_X0_Pos)
#define EPIC_VL_TL_POS_X0               EPIC_VL_TL_POS_X0_Msk
#define EPIC_VL_TL_POS_Y0_Pos           (16U)
#define EPIC_VL_TL_POS_Y0_Msk           (0x1FFFUL << EPIC_VL_TL_POS_Y0_Pos)
#define EPIC_VL_TL_POS_Y0               EPIC_VL_TL_POS_Y0_Msk

/***************** Bit definition for EPIC_VL_BR_POS register *****************/
#define EPIC_VL_BR_POS_X1_Pos           (0U)
#define EPIC_VL_BR_POS_X1_Msk           (0x1FFFUL << EPIC_VL_BR_POS_X1_Pos)
#define EPIC_VL_BR_POS_X1               EPIC_VL_BR_POS_X1_Msk
#define EPIC_VL_BR_POS_Y1_Pos           (16U)
#define EPIC_VL_BR_POS_Y1_Msk           (0x1FFFUL << EPIC_VL_BR_POS_Y1_Pos)
#define EPIC_VL_BR_POS_Y1               EPIC_VL_BR_POS_Y1_Msk

/**************** Bit definition for EPIC_VL_EXTENTS register *****************/
#define EPIC_VL_EXTENTS_MAX_LINE_Pos    (0U)
#define EPIC_VL_EXTENTS_MAX_LINE_Msk    (0x1FFFUL << EPIC_VL_EXTENTS_MAX_LINE_Pos)
#define EPIC_VL_EXTENTS_MAX_LINE        EPIC_VL_EXTENTS_MAX_LINE_Msk
#define EPIC_VL_EXTENTS_MAX_COL_Pos     (16U)
#define EPIC_VL_EXTENTS_MAX_COL_Msk     (0x1FFFUL << EPIC_VL_EXTENTS_MAX_COL_Pos)
#define EPIC_VL_EXTENTS_MAX_COL         EPIC_VL_EXTENTS_MAX_COL_Msk

/***************** Bit definition for EPIC_VL_FILTER register *****************/
#define EPIC_VL_FILTER_FILTER_B_Pos     (0U)
#define EPIC_VL_FILTER_FILTER_B_Msk     (0xFFUL << EPIC_VL_FILTER_FILTER_B_Pos)
#define EPIC_VL_FILTER_FILTER_B         EPIC_VL_FILTER_FILTER_B_Msk
#define EPIC_VL_FILTER_FILTER_G_Pos     (8U)
#define EPIC_VL_FILTER_FILTER_G_Msk     (0xFFUL << EPIC_VL_FILTER_FILTER_G_Pos)
#define EPIC_VL_FILTER_FILTER_G         EPIC_VL_FILTER_FILTER_G_Msk
#define EPIC_VL_FILTER_FILTER_R_Pos     (16U)
#define EPIC_VL_FILTER_FILTER_R_Msk     (0xFFUL << EPIC_VL_FILTER_FILTER_R_Pos)
#define EPIC_VL_FILTER_FILTER_R         EPIC_VL_FILTER_FILTER_R_Msk
#define EPIC_VL_FILTER_FILTER_MASK_Pos  (24U)
#define EPIC_VL_FILTER_FILTER_MASK_Msk  (0xFFUL << EPIC_VL_FILTER_FILTER_MASK_Pos)
#define EPIC_VL_FILTER_FILTER_MASK      EPIC_VL_FILTER_FILTER_MASK_Msk

/****************** Bit definition for EPIC_VL_SRC register *******************/
#define EPIC_VL_SRC_ADDR_Pos            (0U)
#define EPIC_VL_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_VL_SRC_ADDR_Pos)
#define EPIC_VL_SRC_ADDR                EPIC_VL_SRC_ADDR_Msk

/****************** Bit definition for EPIC_VL_ROT register *******************/
#define EPIC_VL_ROT_CALC_REQ_Pos        (0U)
#define EPIC_VL_ROT_CALC_REQ_Msk        (0x1UL << EPIC_VL_ROT_CALC_REQ_Pos)
#define EPIC_VL_ROT_CALC_REQ            EPIC_VL_ROT_CALC_REQ_Msk
#define EPIC_VL_ROT_CALC_CLR_Pos        (1U)
#define EPIC_VL_ROT_CALC_CLR_Msk        (0x1UL << EPIC_VL_ROT_CALC_CLR_Pos)
#define EPIC_VL_ROT_CALC_CLR            EPIC_VL_ROT_CALC_CLR_Msk
#define EPIC_VL_ROT_ROT_DEG_Pos         (2U)
#define EPIC_VL_ROT_ROT_DEG_Msk         (0x1FFUL << EPIC_VL_ROT_ROT_DEG_Pos)
#define EPIC_VL_ROT_ROT_DEG             EPIC_VL_ROT_ROT_DEG_Msk
#define EPIC_VL_ROT_CALC_DONE_Pos       (11U)
#define EPIC_VL_ROT_CALC_DONE_Msk       (0x1UL << EPIC_VL_ROT_CALC_DONE_Pos)
#define EPIC_VL_ROT_CALC_DONE           EPIC_VL_ROT_CALC_DONE_Msk
#define EPIC_VL_ROT_DEG_FORCE_Pos       (12U)
#define EPIC_VL_ROT_DEG_FORCE_Msk       (0x1UL << EPIC_VL_ROT_DEG_FORCE_Pos)
#define EPIC_VL_ROT_DEG_FORCE           EPIC_VL_ROT_DEG_FORCE_Msk
#define EPIC_VL_ROT_SPLIT_EN_Pos        (13U)
#define EPIC_VL_ROT_SPLIT_EN_Msk        (0x1UL << EPIC_VL_ROT_SPLIT_EN_Pos)
#define EPIC_VL_ROT_SPLIT_EN            EPIC_VL_ROT_SPLIT_EN_Msk

/**************** Bit definition for EPIC_VL_ROT_STAT register ****************/
#define EPIC_VL_ROT_STAT_ROT_MAX_LINE_Pos  (0U)
#define EPIC_VL_ROT_STAT_ROT_MAX_LINE_Msk  (0x3FFFUL << EPIC_VL_ROT_STAT_ROT_MAX_LINE_Pos)
#define EPIC_VL_ROT_STAT_ROT_MAX_LINE   EPIC_VL_ROT_STAT_ROT_MAX_LINE_Msk
#define EPIC_VL_ROT_STAT_ROT_MAX_COL_Pos  (16U)
#define EPIC_VL_ROT_STAT_ROT_MAX_COL_Msk  (0x3FFFUL << EPIC_VL_ROT_STAT_ROT_MAX_COL_Pos)
#define EPIC_VL_ROT_STAT_ROT_MAX_COL    EPIC_VL_ROT_STAT_ROT_MAX_COL_Msk

/************* Bit definition for EPIC_VL_SCALE_RATIO_H register **************/
#define EPIC_VL_SCALE_RATIO_H_XPITCH_Pos  (0U)
#define EPIC_VL_SCALE_RATIO_H_XPITCH_Msk  (0x1FFFFFFFUL << EPIC_VL_SCALE_RATIO_H_XPITCH_Pos)
#define EPIC_VL_SCALE_RATIO_H_XPITCH    EPIC_VL_SCALE_RATIO_H_XPITCH_Msk

/************* Bit definition for EPIC_VL_SCALE_RATIO_V register **************/
#define EPIC_VL_SCALE_RATIO_V_YPITCH_Pos  (0U)
#define EPIC_VL_SCALE_RATIO_V_YPITCH_Msk  (0x1FFFFFFFUL << EPIC_VL_SCALE_RATIO_V_YPITCH_Pos)
#define EPIC_VL_SCALE_RATIO_V_YPITCH    EPIC_VL_SCALE_RATIO_V_YPITCH_Msk

/****************** Bit definition for EPIC_VL_FILL register ******************/
#define EPIC_VL_FILL_BG_B_Pos           (0U)
#define EPIC_VL_FILL_BG_B_Msk           (0xFFUL << EPIC_VL_FILL_BG_B_Pos)
#define EPIC_VL_FILL_BG_B               EPIC_VL_FILL_BG_B_Msk
#define EPIC_VL_FILL_BG_G_Pos           (8U)
#define EPIC_VL_FILL_BG_G_Msk           (0xFFUL << EPIC_VL_FILL_BG_G_Pos)
#define EPIC_VL_FILL_BG_G               EPIC_VL_FILL_BG_G_Msk
#define EPIC_VL_FILL_BG_R_Pos           (16U)
#define EPIC_VL_FILL_BG_R_Msk           (0xFFUL << EPIC_VL_FILL_BG_R_Pos)
#define EPIC_VL_FILL_BG_R               EPIC_VL_FILL_BG_R_Msk
#define EPIC_VL_FILL_BG_MODE_Pos        (24U)
#define EPIC_VL_FILL_BG_MODE_Msk        (0x1UL << EPIC_VL_FILL_BG_MODE_Pos)
#define EPIC_VL_FILL_BG_MODE            EPIC_VL_FILL_BG_MODE_Msk
#define EPIC_VL_FILL_ENDIAN_Pos         (25U)
#define EPIC_VL_FILL_ENDIAN_Msk         (0x1UL << EPIC_VL_FILL_ENDIAN_Pos)
#define EPIC_VL_FILL_ENDIAN             EPIC_VL_FILL_ENDIAN_Msk
#define EPIC_VL_FILL_FIX_COLOR_Pos      (27U)
#define EPIC_VL_FILL_FIX_COLOR_Msk      (0x1UL << EPIC_VL_FILL_FIX_COLOR_Pos)
#define EPIC_VL_FILL_FIX_COLOR          EPIC_VL_FILL_FIX_COLOR_Msk
#define EPIC_VL_FILL_PIXEL_ORDER_Pos    (28U)
#define EPIC_VL_FILL_PIXEL_ORDER_Msk    (0x1UL << EPIC_VL_FILL_PIXEL_ORDER_Pos)
#define EPIC_VL_FILL_PIXEL_ORDER        EPIC_VL_FILL_PIXEL_ORDER_Msk

/**************** Bit definition for EPIC_VL_MISC_CFG register ****************/
#define EPIC_VL_MISC_CFG_CLUT_SEL_Pos   (0U)
#define EPIC_VL_MISC_CFG_CLUT_SEL_Msk   (0x1UL << EPIC_VL_MISC_CFG_CLUT_SEL_Pos)
#define EPIC_VL_MISC_CFG_CLUT_SEL       EPIC_VL_MISC_CFG_CLUT_SEL_Msk
#define EPIC_VL_MISC_CFG_V_MIRROR_Pos   (1U)
#define EPIC_VL_MISC_CFG_V_MIRROR_Msk   (0x1UL << EPIC_VL_MISC_CFG_V_MIRROR_Pos)
#define EPIC_VL_MISC_CFG_V_MIRROR       EPIC_VL_MISC_CFG_V_MIRROR_Msk
#define EPIC_VL_MISC_CFG_H_MIRROR_Pos   (2U)
#define EPIC_VL_MISC_CFG_H_MIRROR_Msk   (0x1UL << EPIC_VL_MISC_CFG_H_MIRROR_Pos)
#define EPIC_VL_MISC_CFG_H_MIRROR       EPIC_VL_MISC_CFG_H_MIRROR_Msk
#define EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Pos  (3U)
#define EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Msk  (0x1FFFUL << EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Pos)
#define EPIC_VL_MISC_CFG_COS_FORCE_VALUE  EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Msk
#define EPIC_VL_MISC_CFG_COS_FRAC_BIT            (12)
#define EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Pos  (16U)
#define EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Msk  (0x1FFFUL << EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Pos)
#define EPIC_VL_MISC_CFG_SIN_FORCE_VALUE  EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Msk
#define EPIC_VL_MISC_CFG_SIN_FRAC_BIT    (EPIC_VL_MISC_CFG_COS_FRAC_BIT)
#define EPIC_VL_MISC_CFG_FETCH_MODE_Pos  (29U)
#define EPIC_VL_MISC_CFG_FETCH_MODE_Msk  (0x1UL << EPIC_VL_MISC_CFG_FETCH_MODE_Pos)
#define EPIC_VL_MISC_CFG_FETCH_MODE     EPIC_VL_MISC_CFG_FETCH_MODE_Msk
#define EPIC_VL_MISC_CFG_BLEND_DEPTH_Pos  (30U)
#define EPIC_VL_MISC_CFG_BLEND_DEPTH_Msk  (0x3UL << EPIC_VL_MISC_CFG_BLEND_DEPTH_Pos)
#define EPIC_VL_MISC_CFG_BLEND_DEPTH    EPIC_VL_MISC_CFG_BLEND_DEPTH_Msk

/****************** Bit definition for EPIC_L0_CFG register *******************/
#define EPIC_L0_CFG_FORMAT_Pos          (0U)
#define EPIC_L0_CFG_FORMAT_Msk          (0xFUL << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FORMAT              EPIC_L0_CFG_FORMAT_Msk
#define EPIC_L0_CFG_FMT_RGB565          (0 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_RGB888          (1 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_ARGB8888        (2 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_ARGB8565        (3 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_A8              (4 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_A4              (5 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_L8              (6 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_A2              (7 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_FMT_L4              (8 << EPIC_L0_CFG_FORMAT_Pos)
#define EPIC_L0_CFG_ALPHA_SEL_Pos       (4U)
#define EPIC_L0_CFG_ALPHA_SEL_Msk       (0x1UL << EPIC_L0_CFG_ALPHA_SEL_Pos)
#define EPIC_L0_CFG_ALPHA_SEL           EPIC_L0_CFG_ALPHA_SEL_Msk
#define EPIC_L0_CFG_ALPHA_Pos           (5U)
#define EPIC_L0_CFG_ALPHA_Msk           (0xFFUL << EPIC_L0_CFG_ALPHA_Pos)
#define EPIC_L0_CFG_ALPHA               EPIC_L0_CFG_ALPHA_Msk
#define EPIC_L0_CFG_ALPHA_BLEND_Pos     (13U)
#define EPIC_L0_CFG_ALPHA_BLEND_Msk     (0x1UL << EPIC_L0_CFG_ALPHA_BLEND_Pos)
#define EPIC_L0_CFG_ALPHA_BLEND         EPIC_L0_CFG_ALPHA_BLEND_Msk
#define EPIC_L0_CFG_FILTER_EN_Pos       (14U)
#define EPIC_L0_CFG_FILTER_EN_Msk       (0x1UL << EPIC_L0_CFG_FILTER_EN_Pos)
#define EPIC_L0_CFG_FILTER_EN           EPIC_L0_CFG_FILTER_EN_Msk
#define EPIC_L0_CFG_WIDTH_Pos           (16U)
#define EPIC_L0_CFG_WIDTH_Msk           (0x7FFFUL << EPIC_L0_CFG_WIDTH_Pos)
#define EPIC_L0_CFG_WIDTH               EPIC_L0_CFG_WIDTH_Msk
#define EPIC_L0_CFG_ACTIVE_Pos          (31U)
#define EPIC_L0_CFG_ACTIVE_Msk          (0x1UL << EPIC_L0_CFG_ACTIVE_Pos)
#define EPIC_L0_CFG_ACTIVE              EPIC_L0_CFG_ACTIVE_Msk

/***************** Bit definition for EPIC_L0_TL_POS register *****************/
#define EPIC_L0_TL_POS_X0_Pos           (0U)
#define EPIC_L0_TL_POS_X0_Msk           (0x1FFFUL << EPIC_L0_TL_POS_X0_Pos)
#define EPIC_L0_TL_POS_X0               EPIC_L0_TL_POS_X0_Msk
#define EPIC_L0_TL_POS_Y0_Pos           (16U)
#define EPIC_L0_TL_POS_Y0_Msk           (0x1FFFUL << EPIC_L0_TL_POS_Y0_Pos)
#define EPIC_L0_TL_POS_Y0               EPIC_L0_TL_POS_Y0_Msk

/***************** Bit definition for EPIC_L0_BR_POS register *****************/
#define EPIC_L0_BR_POS_X1_Pos           (0U)
#define EPIC_L0_BR_POS_X1_Msk           (0x1FFFUL << EPIC_L0_BR_POS_X1_Pos)
#define EPIC_L0_BR_POS_X1               EPIC_L0_BR_POS_X1_Msk
#define EPIC_L0_BR_POS_Y1_Pos           (16U)
#define EPIC_L0_BR_POS_Y1_Msk           (0x1FFFUL << EPIC_L0_BR_POS_Y1_Pos)
#define EPIC_L0_BR_POS_Y1               EPIC_L0_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_L0_FILTER register *****************/
#define EPIC_L0_FILTER_FILTER_B_Pos     (0U)
#define EPIC_L0_FILTER_FILTER_B_Msk     (0xFFUL << EPIC_L0_FILTER_FILTER_B_Pos)
#define EPIC_L0_FILTER_FILTER_B         EPIC_L0_FILTER_FILTER_B_Msk
#define EPIC_L0_FILTER_FILTER_G_Pos     (8U)
#define EPIC_L0_FILTER_FILTER_G_Msk     (0xFFUL << EPIC_L0_FILTER_FILTER_G_Pos)
#define EPIC_L0_FILTER_FILTER_G         EPIC_L0_FILTER_FILTER_G_Msk
#define EPIC_L0_FILTER_FILTER_R_Pos     (16U)
#define EPIC_L0_FILTER_FILTER_R_Msk     (0xFFUL << EPIC_L0_FILTER_FILTER_R_Pos)
#define EPIC_L0_FILTER_FILTER_R         EPIC_L0_FILTER_FILTER_R_Msk
#define EPIC_L0_FILTER_FILTER_MASK_Pos  (24U)
#define EPIC_L0_FILTER_FILTER_MASK_Msk  (0xFFUL << EPIC_L0_FILTER_FILTER_MASK_Pos)
#define EPIC_L0_FILTER_FILTER_MASK      EPIC_L0_FILTER_FILTER_MASK_Msk

/****************** Bit definition for EPIC_L0_SRC register *******************/
#define EPIC_L0_SRC_ADDR_Pos            (0U)
#define EPIC_L0_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_L0_SRC_ADDR_Pos)
#define EPIC_L0_SRC_ADDR                EPIC_L0_SRC_ADDR_Msk

/****************** Bit definition for EPIC_L0_FILL register ******************/
#define EPIC_L0_FILL_BG_B_Pos           (0U)
#define EPIC_L0_FILL_BG_B_Msk           (0xFFUL << EPIC_L0_FILL_BG_B_Pos)
#define EPIC_L0_FILL_BG_B               EPIC_L0_FILL_BG_B_Msk
#define EPIC_L0_FILL_BG_G_Pos           (8U)
#define EPIC_L0_FILL_BG_G_Msk           (0xFFUL << EPIC_L0_FILL_BG_G_Pos)
#define EPIC_L0_FILL_BG_G               EPIC_L0_FILL_BG_G_Msk
#define EPIC_L0_FILL_BG_R_Pos           (16U)
#define EPIC_L0_FILL_BG_R_Msk           (0xFFUL << EPIC_L0_FILL_BG_R_Pos)
#define EPIC_L0_FILL_BG_R               EPIC_L0_FILL_BG_R_Msk
#define EPIC_L0_FILL_BG_MODE_Pos        (24U)
#define EPIC_L0_FILL_BG_MODE_Msk        (0x1UL << EPIC_L0_FILL_BG_MODE_Pos)
#define EPIC_L0_FILL_BG_MODE            EPIC_L0_FILL_BG_MODE_Msk
#define EPIC_L0_FILL_ENDIAN_Pos         (25U)
#define EPIC_L0_FILL_ENDIAN_Msk         (0x1UL << EPIC_L0_FILL_ENDIAN_Pos)
#define EPIC_L0_FILL_ENDIAN             EPIC_L0_FILL_ENDIAN_Msk
#define EPIC_L0_FILL_FIX_COLOR_Pos      (27U)
#define EPIC_L0_FILL_FIX_COLOR_Msk      (0x1UL << EPIC_L0_FILL_FIX_COLOR_Pos)
#define EPIC_L0_FILL_FIX_COLOR          EPIC_L0_FILL_FIX_COLOR_Msk
#define EPIC_L0_FILL_PIXEL_ORDER_Pos    (28U)
#define EPIC_L0_FILL_PIXEL_ORDER_Msk    (0x1UL << EPIC_L0_FILL_PIXEL_ORDER_Pos)
#define EPIC_L0_FILL_PIXEL_ORDER        EPIC_L0_FILL_PIXEL_ORDER_Msk

/**************** Bit definition for EPIC_L0_MISC_CFG register ****************/
#define EPIC_L0_MISC_CFG_CLUT_SEL_Pos   (0U)
#define EPIC_L0_MISC_CFG_CLUT_SEL_Msk   (0x1UL << EPIC_L0_MISC_CFG_CLUT_SEL_Pos)
#define EPIC_L0_MISC_CFG_CLUT_SEL       EPIC_L0_MISC_CFG_CLUT_SEL_Msk
#define EPIC_L0_MISC_CFG_V_MIRROR_Pos   (1U)
#define EPIC_L0_MISC_CFG_V_MIRROR_Msk   (0x1UL << EPIC_L0_MISC_CFG_V_MIRROR_Pos)
#define EPIC_L0_MISC_CFG_V_MIRROR       EPIC_L0_MISC_CFG_V_MIRROR_Msk

/****************** Bit definition for EPIC_L1_CFG register *******************/
#define EPIC_L1_CFG_FORMAT_Pos          (0U)
#define EPIC_L1_CFG_FORMAT_Msk          (0xFUL << EPIC_L1_CFG_FORMAT_Pos)
#define EPIC_L1_CFG_FORMAT              EPIC_L1_CFG_FORMAT_Msk
#define EPIC_L1_CFG_ALPHA_SEL_Pos       (4U)
#define EPIC_L1_CFG_ALPHA_SEL_Msk       (0x1UL << EPIC_L1_CFG_ALPHA_SEL_Pos)
#define EPIC_L1_CFG_ALPHA_SEL           EPIC_L1_CFG_ALPHA_SEL_Msk
#define EPIC_L1_CFG_ALPHA_Pos           (5U)
#define EPIC_L1_CFG_ALPHA_Msk           (0xFFUL << EPIC_L1_CFG_ALPHA_Pos)
#define EPIC_L1_CFG_ALPHA               EPIC_L1_CFG_ALPHA_Msk
#define EPIC_L1_CFG_ALPHA_BLEND_Pos     (13U)
#define EPIC_L1_CFG_ALPHA_BLEND_Msk     (0x1UL << EPIC_L1_CFG_ALPHA_BLEND_Pos)
#define EPIC_L1_CFG_ALPHA_BLEND         EPIC_L1_CFG_ALPHA_BLEND_Msk
#define EPIC_L1_CFG_FILTER_EN_Pos       (14U)
#define EPIC_L1_CFG_FILTER_EN_Msk       (0x1UL << EPIC_L1_CFG_FILTER_EN_Pos)
#define EPIC_L1_CFG_FILTER_EN           EPIC_L1_CFG_FILTER_EN_Msk
#define EPIC_L1_CFG_WIDTH_Pos           (16U)
#define EPIC_L1_CFG_WIDTH_Msk           (0x7FFFUL << EPIC_L1_CFG_WIDTH_Pos)
#define EPIC_L1_CFG_WIDTH               EPIC_L1_CFG_WIDTH_Msk
#define EPIC_L1_CFG_ACTIVE_Pos          (31U)
#define EPIC_L1_CFG_ACTIVE_Msk          (0x1UL << EPIC_L1_CFG_ACTIVE_Pos)
#define EPIC_L1_CFG_ACTIVE              EPIC_L1_CFG_ACTIVE_Msk

/***************** Bit definition for EPIC_L1_TL_POS register *****************/
#define EPIC_L1_TL_POS_X0_Pos           (0U)
#define EPIC_L1_TL_POS_X0_Msk           (0x1FFFUL << EPIC_L1_TL_POS_X0_Pos)
#define EPIC_L1_TL_POS_X0               EPIC_L1_TL_POS_X0_Msk
#define EPIC_L1_TL_POS_Y0_Pos           (16U)
#define EPIC_L1_TL_POS_Y0_Msk           (0x1FFFUL << EPIC_L1_TL_POS_Y0_Pos)
#define EPIC_L1_TL_POS_Y0               EPIC_L1_TL_POS_Y0_Msk

/***************** Bit definition for EPIC_L1_BR_POS register *****************/
#define EPIC_L1_BR_POS_X1_Pos           (0U)
#define EPIC_L1_BR_POS_X1_Msk           (0x1FFFUL << EPIC_L1_BR_POS_X1_Pos)
#define EPIC_L1_BR_POS_X1               EPIC_L1_BR_POS_X1_Msk
#define EPIC_L1_BR_POS_Y1_Pos           (16U)
#define EPIC_L1_BR_POS_Y1_Msk           (0x1FFFUL << EPIC_L1_BR_POS_Y1_Pos)
#define EPIC_L1_BR_POS_Y1               EPIC_L1_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_L1_FILTER register *****************/
#define EPIC_L1_FILTER_FILTER_B_Pos     (0U)
#define EPIC_L1_FILTER_FILTER_B_Msk     (0xFFUL << EPIC_L1_FILTER_FILTER_B_Pos)
#define EPIC_L1_FILTER_FILTER_B         EPIC_L1_FILTER_FILTER_B_Msk
#define EPIC_L1_FILTER_FILTER_G_Pos     (8U)
#define EPIC_L1_FILTER_FILTER_G_Msk     (0xFFUL << EPIC_L1_FILTER_FILTER_G_Pos)
#define EPIC_L1_FILTER_FILTER_G         EPIC_L1_FILTER_FILTER_G_Msk
#define EPIC_L1_FILTER_FILTER_R_Pos     (16U)
#define EPIC_L1_FILTER_FILTER_R_Msk     (0xFFUL << EPIC_L1_FILTER_FILTER_R_Pos)
#define EPIC_L1_FILTER_FILTER_R         EPIC_L1_FILTER_FILTER_R_Msk
#define EPIC_L1_FILTER_FILTER_MASK_Pos  (24U)
#define EPIC_L1_FILTER_FILTER_MASK_Msk  (0xFFUL << EPIC_L1_FILTER_FILTER_MASK_Pos)
#define EPIC_L1_FILTER_FILTER_MASK      EPIC_L1_FILTER_FILTER_MASK_Msk

/****************** Bit definition for EPIC_L1_SRC register *******************/
#define EPIC_L1_SRC_ADDR_Pos            (0U)
#define EPIC_L1_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_L1_SRC_ADDR_Pos)
#define EPIC_L1_SRC_ADDR                EPIC_L1_SRC_ADDR_Msk

/****************** Bit definition for EPIC_L1_FILL register ******************/
#define EPIC_L1_FILL_BG_B_Pos           (0U)
#define EPIC_L1_FILL_BG_B_Msk           (0xFFUL << EPIC_L1_FILL_BG_B_Pos)
#define EPIC_L1_FILL_BG_B               EPIC_L1_FILL_BG_B_Msk
#define EPIC_L1_FILL_BG_G_Pos           (8U)
#define EPIC_L1_FILL_BG_G_Msk           (0xFFUL << EPIC_L1_FILL_BG_G_Pos)
#define EPIC_L1_FILL_BG_G               EPIC_L1_FILL_BG_G_Msk
#define EPIC_L1_FILL_BG_R_Pos           (16U)
#define EPIC_L1_FILL_BG_R_Msk           (0xFFUL << EPIC_L1_FILL_BG_R_Pos)
#define EPIC_L1_FILL_BG_R               EPIC_L1_FILL_BG_R_Msk
#define EPIC_L1_FILL_BG_MODE_Pos        (24U)
#define EPIC_L1_FILL_BG_MODE_Msk        (0x1UL << EPIC_L1_FILL_BG_MODE_Pos)
#define EPIC_L1_FILL_BG_MODE            EPIC_L1_FILL_BG_MODE_Msk
#define EPIC_L1_FILL_ENDIAN_Pos         (25U)
#define EPIC_L1_FILL_ENDIAN_Msk         (0x1UL << EPIC_L1_FILL_ENDIAN_Pos)
#define EPIC_L1_FILL_ENDIAN             EPIC_L1_FILL_ENDIAN_Msk
#define EPIC_L1_FILL_FIX_COLOR_Pos      (27U)
#define EPIC_L1_FILL_FIX_COLOR_Msk      (0x1UL << EPIC_L1_FILL_FIX_COLOR_Pos)
#define EPIC_L1_FILL_FIX_COLOR          EPIC_L1_FILL_FIX_COLOR_Msk
#define EPIC_L1_FILL_PIXEL_ORDER_Pos    (28U)
#define EPIC_L1_FILL_PIXEL_ORDER_Msk    (0x1UL << EPIC_L1_FILL_PIXEL_ORDER_Pos)
#define EPIC_L1_FILL_PIXEL_ORDER        EPIC_L1_FILL_PIXEL_ORDER_Msk

/**************** Bit definition for EPIC_L1_MISC_CFG register ****************/
#define EPIC_L1_MISC_CFG_CLUT_SEL_Pos   (0U)
#define EPIC_L1_MISC_CFG_CLUT_SEL_Msk   (0x1UL << EPIC_L1_MISC_CFG_CLUT_SEL_Pos)
#define EPIC_L1_MISC_CFG_CLUT_SEL       EPIC_L1_MISC_CFG_CLUT_SEL_Msk
#define EPIC_L1_MISC_CFG_V_MIRROR_Pos   (1U)
#define EPIC_L1_MISC_CFG_V_MIRROR_Msk   (0x1UL << EPIC_L1_MISC_CFG_V_MIRROR_Pos)
#define EPIC_L1_MISC_CFG_V_MIRROR       EPIC_L1_MISC_CFG_V_MIRROR_Msk

/****************** Bit definition for EPIC_L2_CFG register *******************/
#define EPIC_L2_CFG_FORMAT_Pos          (0U)
#define EPIC_L2_CFG_FORMAT_Msk          (0xFUL << EPIC_L2_CFG_FORMAT_Pos)
#define EPIC_L2_CFG_FORMAT              EPIC_L2_CFG_FORMAT_Msk
#define EPIC_L2_CFG_ALPHA_SEL_Pos       (4U)
#define EPIC_L2_CFG_ALPHA_SEL_Msk       (0x1UL << EPIC_L2_CFG_ALPHA_SEL_Pos)
#define EPIC_L2_CFG_ALPHA_SEL           EPIC_L2_CFG_ALPHA_SEL_Msk
#define EPIC_L2_CFG_ALPHA_Pos           (5U)
#define EPIC_L2_CFG_ALPHA_Msk           (0xFFUL << EPIC_L2_CFG_ALPHA_Pos)
#define EPIC_L2_CFG_ALPHA               EPIC_L2_CFG_ALPHA_Msk
#define EPIC_L2_CFG_ALPHA_BLEND_Pos     (13U)
#define EPIC_L2_CFG_ALPHA_BLEND_Msk     (0x1UL << EPIC_L2_CFG_ALPHA_BLEND_Pos)
#define EPIC_L2_CFG_ALPHA_BLEND         EPIC_L2_CFG_ALPHA_BLEND_Msk
#define EPIC_L2_CFG_FILTER_EN_Pos       (14U)
#define EPIC_L2_CFG_FILTER_EN_Msk       (0x1UL << EPIC_L2_CFG_FILTER_EN_Pos)
#define EPIC_L2_CFG_FILTER_EN           EPIC_L2_CFG_FILTER_EN_Msk
#define EPIC_L2_CFG_WIDTH_Pos           (16U)
#define EPIC_L2_CFG_WIDTH_Msk           (0x7FFFUL << EPIC_L2_CFG_WIDTH_Pos)
#define EPIC_L2_CFG_WIDTH               EPIC_L2_CFG_WIDTH_Msk
#define EPIC_L2_CFG_ACTIVE_Pos          (31U)
#define EPIC_L2_CFG_ACTIVE_Msk          (0x1UL << EPIC_L2_CFG_ACTIVE_Pos)
#define EPIC_L2_CFG_ACTIVE              EPIC_L2_CFG_ACTIVE_Msk

/***************** Bit definition for EPIC_L2_TL_POS register *****************/
#define EPIC_L2_TL_POS_X0_Pos           (0U)
#define EPIC_L2_TL_POS_X0_Msk           (0x1FFFUL << EPIC_L2_TL_POS_X0_Pos)
#define EPIC_L2_TL_POS_X0               EPIC_L2_TL_POS_X0_Msk
#define EPIC_L2_TL_POS_Y0_Pos           (16U)
#define EPIC_L2_TL_POS_Y0_Msk           (0x1FFFUL << EPIC_L2_TL_POS_Y0_Pos)
#define EPIC_L2_TL_POS_Y0               EPIC_L2_TL_POS_Y0_Msk

/***************** Bit definition for EPIC_L2_BR_POS register *****************/
#define EPIC_L2_BR_POS_X1_Pos           (0U)
#define EPIC_L2_BR_POS_X1_Msk           (0x1FFFUL << EPIC_L2_BR_POS_X1_Pos)
#define EPIC_L2_BR_POS_X1               EPIC_L2_BR_POS_X1_Msk
#define EPIC_L2_BR_POS_Y1_Pos           (16U)
#define EPIC_L2_BR_POS_Y1_Msk           (0x1FFFUL << EPIC_L2_BR_POS_Y1_Pos)
#define EPIC_L2_BR_POS_Y1               EPIC_L2_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_L2_FILTER register *****************/
#define EPIC_L2_FILTER_FILTER_B_Pos     (0U)
#define EPIC_L2_FILTER_FILTER_B_Msk     (0xFFUL << EPIC_L2_FILTER_FILTER_B_Pos)
#define EPIC_L2_FILTER_FILTER_B         EPIC_L2_FILTER_FILTER_B_Msk
#define EPIC_L2_FILTER_FILTER_G_Pos     (8U)
#define EPIC_L2_FILTER_FILTER_G_Msk     (0xFFUL << EPIC_L2_FILTER_FILTER_G_Pos)
#define EPIC_L2_FILTER_FILTER_G         EPIC_L2_FILTER_FILTER_G_Msk
#define EPIC_L2_FILTER_FILTER_R_Pos     (16U)
#define EPIC_L2_FILTER_FILTER_R_Msk     (0xFFUL << EPIC_L2_FILTER_FILTER_R_Pos)
#define EPIC_L2_FILTER_FILTER_R         EPIC_L2_FILTER_FILTER_R_Msk
#define EPIC_L2_FILTER_FILTER_MASK_Pos  (24U)
#define EPIC_L2_FILTER_FILTER_MASK_Msk  (0xFFUL << EPIC_L2_FILTER_FILTER_MASK_Pos)
#define EPIC_L2_FILTER_FILTER_MASK      EPIC_L2_FILTER_FILTER_MASK_Msk

/****************** Bit definition for EPIC_L2_SRC register *******************/
#define EPIC_L2_SRC_ADDR_Pos            (0U)
#define EPIC_L2_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_L2_SRC_ADDR_Pos)
#define EPIC_L2_SRC_ADDR                EPIC_L2_SRC_ADDR_Msk

/****************** Bit definition for EPIC_L2_FILL register ******************/
#define EPIC_L2_FILL_BG_B_Pos           (0U)
#define EPIC_L2_FILL_BG_B_Msk           (0xFFUL << EPIC_L2_FILL_BG_B_Pos)
#define EPIC_L2_FILL_BG_B               EPIC_L2_FILL_BG_B_Msk
#define EPIC_L2_FILL_BG_G_Pos           (8U)
#define EPIC_L2_FILL_BG_G_Msk           (0xFFUL << EPIC_L2_FILL_BG_G_Pos)
#define EPIC_L2_FILL_BG_G               EPIC_L2_FILL_BG_G_Msk
#define EPIC_L2_FILL_BG_R_Pos           (16U)
#define EPIC_L2_FILL_BG_R_Msk           (0xFFUL << EPIC_L2_FILL_BG_R_Pos)
#define EPIC_L2_FILL_BG_R               EPIC_L2_FILL_BG_R_Msk
#define EPIC_L2_FILL_BG_MODE_Pos        (24U)
#define EPIC_L2_FILL_BG_MODE_Msk        (0x1UL << EPIC_L2_FILL_BG_MODE_Pos)
#define EPIC_L2_FILL_BG_MODE            EPIC_L2_FILL_BG_MODE_Msk
#define EPIC_L2_FILL_ENDIAN_Pos         (25U)
#define EPIC_L2_FILL_ENDIAN_Msk         (0x1UL << EPIC_L2_FILL_ENDIAN_Pos)
#define EPIC_L2_FILL_ENDIAN             EPIC_L2_FILL_ENDIAN_Msk
#define EPIC_L2_FILL_FIX_COLOR_Pos      (27U)
#define EPIC_L2_FILL_FIX_COLOR_Msk      (0x1UL << EPIC_L2_FILL_FIX_COLOR_Pos)
#define EPIC_L2_FILL_FIX_COLOR          EPIC_L2_FILL_FIX_COLOR_Msk
#define EPIC_L2_FILL_PIXEL_ORDER_Pos    (28U)
#define EPIC_L2_FILL_PIXEL_ORDER_Msk    (0x1UL << EPIC_L2_FILL_PIXEL_ORDER_Pos)
#define EPIC_L2_FILL_PIXEL_ORDER        EPIC_L2_FILL_PIXEL_ORDER_Msk

/**************** Bit definition for EPIC_L2_MISC_CFG register ****************/
#define EPIC_L2_MISC_CFG_CLUT_SEL_Pos   (0U)
#define EPIC_L2_MISC_CFG_CLUT_SEL_Msk   (0x1UL << EPIC_L2_MISC_CFG_CLUT_SEL_Pos)
#define EPIC_L2_MISC_CFG_CLUT_SEL       EPIC_L2_MISC_CFG_CLUT_SEL_Msk
#define EPIC_L2_MISC_CFG_MAX_COL_Pos    (1U)
#define EPIC_L2_MISC_CFG_MAX_COL_Msk    (0x1FFFUL << EPIC_L2_MISC_CFG_MAX_COL_Pos)
#define EPIC_L2_MISC_CFG_MAX_COL        EPIC_L2_MISC_CFG_MAX_COL_Msk
#define EPIC_L2_MISC_CFG_MAX_LINE_Pos   (14U)
#define EPIC_L2_MISC_CFG_MAX_LINE_Msk   (0x1FFFUL << EPIC_L2_MISC_CFG_MAX_LINE_Pos)
#define EPIC_L2_MISC_CFG_MAX_LINE       EPIC_L2_MISC_CFG_MAX_LINE_Msk
#define EPIC_L2_MISC_CFG_FETCH_MODE_Pos  (27U)
#define EPIC_L2_MISC_CFG_FETCH_MODE_Msk  (0x1UL << EPIC_L2_MISC_CFG_FETCH_MODE_Pos)
#define EPIC_L2_MISC_CFG_FETCH_MODE     EPIC_L2_MISC_CFG_FETCH_MODE_Msk
#define EPIC_L2_MISC_CFG_CACHE_EN_Pos   (28U)
#define EPIC_L2_MISC_CFG_CACHE_EN_Msk   (0x1UL << EPIC_L2_MISC_CFG_CACHE_EN_Pos)
#define EPIC_L2_MISC_CFG_CACHE_EN       EPIC_L2_MISC_CFG_CACHE_EN_Msk

/***************** Bit definition for EPIC_L2_MAT_A0 register *****************/
#define EPIC_L2_MAT_A0_VAL_Pos          (0U)
#define EPIC_L2_MAT_A0_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_A0_VAL_Pos)
#define EPIC_L2_MAT_A0_VAL              EPIC_L2_MAT_A0_VAL_Msk

/***************** Bit definition for EPIC_L2_MAT_A1 register *****************/
#define EPIC_L2_MAT_A1_VAL_Pos          (0U)
#define EPIC_L2_MAT_A1_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_A1_VAL_Pos)
#define EPIC_L2_MAT_A1_VAL              EPIC_L2_MAT_A1_VAL_Msk

/***************** Bit definition for EPIC_L2_MAT_A2 register *****************/
#define EPIC_L2_MAT_A2_VAL_Pos          (0U)
#define EPIC_L2_MAT_A2_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_A2_VAL_Pos)
#define EPIC_L2_MAT_A2_VAL              EPIC_L2_MAT_A2_VAL_Msk

/***************** Bit definition for EPIC_L2_MAT_B0 register *****************/
#define EPIC_L2_MAT_B0_VAL_Pos          (0U)
#define EPIC_L2_MAT_B0_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_B0_VAL_Pos)
#define EPIC_L2_MAT_B0_VAL              EPIC_L2_MAT_B0_VAL_Msk

/***************** Bit definition for EPIC_L2_MAT_B1 register *****************/
#define EPIC_L2_MAT_B1_VAL_Pos          (0U)
#define EPIC_L2_MAT_B1_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_B1_VAL_Pos)
#define EPIC_L2_MAT_B1_VAL              EPIC_L2_MAT_B1_VAL_Msk

/***************** Bit definition for EPIC_L2_MAT_B2 register *****************/
#define EPIC_L2_MAT_B2_VAL_Pos          (0U)
#define EPIC_L2_MAT_B2_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_B2_VAL_Pos)
#define EPIC_L2_MAT_B2_VAL              EPIC_L2_MAT_B2_VAL_Msk

/***************** Bit definition for EPIC_L2_MAT_C0 register *****************/
#define EPIC_L2_MAT_C0_VAL_Pos          (0U)
#define EPIC_L2_MAT_C0_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_C0_VAL_Pos)
#define EPIC_L2_MAT_C0_VAL              EPIC_L2_MAT_C0_VAL_Msk

/***************** Bit definition for EPIC_L2_MAT_C1 register *****************/
#define EPIC_L2_MAT_C1_VAL_Pos          (0U)
#define EPIC_L2_MAT_C1_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_C1_VAL_Pos)
#define EPIC_L2_MAT_C1_VAL              EPIC_L2_MAT_C1_VAL_Msk

/***************** Bit definition for EPIC_L2_MAT_C2 register *****************/
#define EPIC_L2_MAT_C2_VAL_Pos          (0U)
#define EPIC_L2_MAT_C2_VAL_Msk          (0xFFFFFFFFUL << EPIC_L2_MAT_C2_VAL_Pos)
#define EPIC_L2_MAT_C2_VAL              EPIC_L2_MAT_C2_VAL_Msk

/***************** Bit definition for EPIC_MASK_CFG register ******************/
#define EPIC_MASK_CFG_FORMAT_Pos        (0U)
#define EPIC_MASK_CFG_FORMAT_Msk        (0x1UL << EPIC_MASK_CFG_FORMAT_Pos)
#define EPIC_MASK_CFG_FORMAT            EPIC_MASK_CFG_FORMAT_Msk
#define EPIC_MASK_CFG_FMT_A4            (0x1UL << EPIC_MASK_CFG_FORMAT_Pos)
#define EPIC_MASK_CFG_FMT_A8            (0x0UL << EPIC_MASK_CFG_FORMAT_Pos)
#define EPIC_MASK_CFG_MIX_MODE_Pos      (1U)
#define EPIC_MASK_CFG_MIX_MODE_Msk      (0x1UL << EPIC_MASK_CFG_MIX_MODE_Pos)
#define EPIC_MASK_CFG_MIX_MODE          EPIC_MASK_CFG_MIX_MODE_Msk
#define EPIC_MASK_CFG_MIX_MODE_MULT          (0x0UL << EPIC_MASK_CFG_MIX_MODE_Pos)
#define EPIC_MASK_CFG_MIX_MODE_OVERWRITE     (0x1UL << EPIC_MASK_CFG_MIX_MODE_Pos)
#define EPIC_MASK_CFG_L0_MASK_EN_Pos    (2U)
#define EPIC_MASK_CFG_L0_MASK_EN_Msk    (0x1UL << EPIC_MASK_CFG_L0_MASK_EN_Pos)
#define EPIC_MASK_CFG_L0_MASK_EN        EPIC_MASK_CFG_L0_MASK_EN_Msk
#define EPIC_MASK_CFG_L1_MASK_EN_Pos    (3U)
#define EPIC_MASK_CFG_L1_MASK_EN_Msk    (0x1UL << EPIC_MASK_CFG_L1_MASK_EN_Pos)
#define EPIC_MASK_CFG_L1_MASK_EN        EPIC_MASK_CFG_L1_MASK_EN_Msk
#define EPIC_MASK_CFG_L2_MASK_EN_Pos    (4U)
#define EPIC_MASK_CFG_L2_MASK_EN_Msk    (0x1UL << EPIC_MASK_CFG_L2_MASK_EN_Pos)
#define EPIC_MASK_CFG_L2_MASK_EN        EPIC_MASK_CFG_L2_MASK_EN_Msk
#define EPIC_MASK_CFG_VL_MASK_EN_Pos    (5U)
#define EPIC_MASK_CFG_VL_MASK_EN_Msk    (0x1UL << EPIC_MASK_CFG_VL_MASK_EN_Pos)
#define EPIC_MASK_CFG_VL_MASK_EN        EPIC_MASK_CFG_VL_MASK_EN_Msk
#define EPIC_MASK_CFG_PIXEL_ORDER_Pos   (6U)
#define EPIC_MASK_CFG_PIXEL_ORDER_Msk   (0x1UL << EPIC_MASK_CFG_PIXEL_ORDER_Pos)
#define EPIC_MASK_CFG_PIXEL_ORDER       EPIC_MASK_CFG_PIXEL_ORDER_Msk
#define EPIC_MASK_CFG_WIDTH_Pos         (16U)
#define EPIC_MASK_CFG_WIDTH_Msk         (0x1FFFUL << EPIC_MASK_CFG_WIDTH_Pos)
#define EPIC_MASK_CFG_WIDTH             EPIC_MASK_CFG_WIDTH_Msk
#define EPIC_MASK_CFG_ACTIVE_Pos        (31U)
#define EPIC_MASK_CFG_ACTIVE_Msk        (0x1UL << EPIC_MASK_CFG_ACTIVE_Pos)
#define EPIC_MASK_CFG_ACTIVE            EPIC_MASK_CFG_ACTIVE_Msk

/**************** Bit definition for EPIC_MASK_TL_POS register ****************/
#define EPIC_MASK_TL_POS_X0_Pos         (0U)
#define EPIC_MASK_TL_POS_X0_Msk         (0x1FFFUL << EPIC_MASK_TL_POS_X0_Pos)
#define EPIC_MASK_TL_POS_X0             EPIC_MASK_TL_POS_X0_Msk
#define EPIC_MASK_TL_POS_Y0_Pos         (16U)
#define EPIC_MASK_TL_POS_Y0_Msk         (0x1FFFUL << EPIC_MASK_TL_POS_Y0_Pos)
#define EPIC_MASK_TL_POS_Y0             EPIC_MASK_TL_POS_Y0_Msk

/**************** Bit definition for EPIC_MASK_BR_POS register ****************/
#define EPIC_MASK_BR_POS_X1_Pos         (0U)
#define EPIC_MASK_BR_POS_X1_Msk         (0x1FFFUL << EPIC_MASK_BR_POS_X1_Pos)
#define EPIC_MASK_BR_POS_X1             EPIC_MASK_BR_POS_X1_Msk
#define EPIC_MASK_BR_POS_Y1_Pos         (16U)
#define EPIC_MASK_BR_POS_Y1_Msk         (0x1FFFUL << EPIC_MASK_BR_POS_Y1_Pos)
#define EPIC_MASK_BR_POS_Y1             EPIC_MASK_BR_POS_Y1_Msk

/***************** Bit definition for EPIC_MASK_SRC register ******************/
#define EPIC_MASK_SRC_ADDR_Pos          (0U)
#define EPIC_MASK_SRC_ADDR_Msk          (0xFFFFFFFFUL << EPIC_MASK_SRC_ADDR_Pos)
#define EPIC_MASK_SRC_ADDR              EPIC_MASK_SRC_ADDR_Msk

/***************** Bit definition for EPIC_COENG_CFG register *****************/
#define EPIC_COENG_CFG_EZIP_EN_Pos      (0U)
#define EPIC_COENG_CFG_EZIP_EN_Msk      (0x1UL << EPIC_COENG_CFG_EZIP_EN_Pos)
#define EPIC_COENG_CFG_EZIP_EN          EPIC_COENG_CFG_EZIP_EN_Msk
#define EPIC_COENG_CFG_EZIP_CH_SEL_Pos  (1U)
#define EPIC_COENG_CFG_EZIP_CH_SEL_Msk  (0x3UL << EPIC_COENG_CFG_EZIP_CH_SEL_Pos)
#define EPIC_COENG_CFG_EZIP_CH_SEL      EPIC_COENG_CFG_EZIP_CH_SEL_Msk
#define EPIC_COENG_CFG_YUV_EN_Pos       (3U)
#define EPIC_COENG_CFG_YUV_EN_Msk       (0x1UL << EPIC_COENG_CFG_YUV_EN_Pos)
#define EPIC_COENG_CFG_YUV_EN           EPIC_COENG_CFG_YUV_EN_Msk
#define EPIC_COENG_CFG_YUV_CH_SEL_Pos   (4U)
#define EPIC_COENG_CFG_YUV_CH_SEL_Msk   (0x3UL << EPIC_COENG_CFG_YUV_CH_SEL_Pos)
#define EPIC_COENG_CFG_YUV_CH_SEL       EPIC_COENG_CFG_YUV_CH_SEL_Msk
#define EPIC_COENG_CFG_DECOMP_EN_Pos    (6U)
#define EPIC_COENG_CFG_DECOMP_EN_Msk    (0x1UL << EPIC_COENG_CFG_DECOMP_EN_Pos)
#define EPIC_COENG_CFG_DECOMP_EN        EPIC_COENG_CFG_DECOMP_EN_Msk
#define EPIC_COENG_CFG_DECOMP_CH_SEL_Pos  (7U)
#define EPIC_COENG_CFG_DECOMP_CH_SEL_Msk  (0x3UL << EPIC_COENG_CFG_DECOMP_CH_SEL_Pos)
#define EPIC_COENG_CFG_DECOMP_CH_SEL    EPIC_COENG_CFG_DECOMP_CH_SEL_Msk
#define EPIC_COENG_CFG_JPEG_EN_Pos      (9U)
#define EPIC_COENG_CFG_JPEG_EN_Msk      (0x1UL << EPIC_COENG_CFG_JPEG_EN_Pos)
#define EPIC_COENG_CFG_JPEG_EN          EPIC_COENG_CFG_JPEG_EN_Msk
#define EPIC_COENG_CFG_JPEG_CH_SEL_Pos  (10U)
#define EPIC_COENG_CFG_JPEG_CH_SEL_Msk  (0x3UL << EPIC_COENG_CFG_JPEG_CH_SEL_Pos)
#define EPIC_COENG_CFG_JPEG_CH_SEL      EPIC_COENG_CFG_JPEG_CH_SEL_Msk
#define EPIC_COENG_CFG_CM_EN_Pos        (12U)
#define EPIC_COENG_CFG_CM_EN_Msk        (0x1UL << EPIC_COENG_CFG_CM_EN_Pos)
#define EPIC_COENG_CFG_CM_EN            EPIC_COENG_CFG_CM_EN_Msk
#define EPIC_COENG_CFG_CM_CH_SEL_Pos    (13U)
#define EPIC_COENG_CFG_CM_CH_SEL_Msk    (0x3UL << EPIC_COENG_CFG_CM_CH_SEL_Pos)
#define EPIC_COENG_CFG_CM_CH_SEL        EPIC_COENG_CFG_CM_CH_SEL_Msk

/*************** Bit definition for EPIC_JPEG_ENG_CFG0 register ***************/
#define EPIC_JPEG_ENG_CFG0_FUB_Pos      (0U)
#define EPIC_JPEG_ENG_CFG0_FUB_Msk      (0x3FFUL << EPIC_JPEG_ENG_CFG0_FUB_Pos)
#define EPIC_JPEG_ENG_CFG0_FUB          EPIC_JPEG_ENG_CFG0_FUB_Msk
#define EPIC_JPEG_ENG_CFG0_FUG_Pos      (10U)
#define EPIC_JPEG_ENG_CFG0_FUG_Msk      (0x3FFUL << EPIC_JPEG_ENG_CFG0_FUG_Pos)
#define EPIC_JPEG_ENG_CFG0_FUG          EPIC_JPEG_ENG_CFG0_FUG_Msk
#define EPIC_JPEG_ENG_CFG0_FY_Pos       (20U)
#define EPIC_JPEG_ENG_CFG0_FY_Msk       (0x3FFUL << EPIC_JPEG_ENG_CFG0_FY_Pos)
#define EPIC_JPEG_ENG_CFG0_FY           EPIC_JPEG_ENG_CFG0_FY_Msk
#define EPIC_JPEG_ENG_CFG0_RANGE_SEL_Pos  (30U)
#define EPIC_JPEG_ENG_CFG0_RANGE_SEL_Msk  (0x1UL << EPIC_JPEG_ENG_CFG0_RANGE_SEL_Pos)
#define EPIC_JPEG_ENG_CFG0_RANGE_SEL    EPIC_JPEG_ENG_CFG0_RANGE_SEL_Msk

/*************** Bit definition for EPIC_JPEG_ENG_CFG1 register ***************/
#define EPIC_JPEG_ENG_CFG1_FVG_Pos      (0U)
#define EPIC_JPEG_ENG_CFG1_FVG_Msk      (0x3FFUL << EPIC_JPEG_ENG_CFG1_FVG_Pos)
#define EPIC_JPEG_ENG_CFG1_FVG          EPIC_JPEG_ENG_CFG1_FVG_Msk
#define EPIC_JPEG_ENG_CFG1_FVR_Pos      (10U)
#define EPIC_JPEG_ENG_CFG1_FVR_Msk      (0x3FFUL << EPIC_JPEG_ENG_CFG1_FVR_Pos)
#define EPIC_JPEG_ENG_CFG1_FVR          EPIC_JPEG_ENG_CFG1_FVR_Msk

/**************** Bit definition for EPIC_DECOMP_CFG0 register ****************/
#define EPIC_DECOMP_CFG0_TARGET_WORDS_Pos  (0U)
#define EPIC_DECOMP_CFG0_TARGET_WORDS_Msk  (0xFFFUL << EPIC_DECOMP_CFG0_TARGET_WORDS_Pos)
#define EPIC_DECOMP_CFG0_TARGET_WORDS   EPIC_DECOMP_CFG0_TARGET_WORDS_Msk
#define EPIC_DECOMP_CFG0_COL_SIZE_Pos   (12U)
#define EPIC_DECOMP_CFG0_COL_SIZE_Msk   (0x7FFUL << EPIC_DECOMP_CFG0_COL_SIZE_Pos)
#define EPIC_DECOMP_CFG0_COL_SIZE       EPIC_DECOMP_CFG0_COL_SIZE_Msk

/**************** Bit definition for EPIC_DECOMP_CFG1 register ****************/
#define EPIC_DECOMP_CFG1_EXTRA_HIGH_Pos  (0U)
#define EPIC_DECOMP_CFG1_EXTRA_HIGH_Msk  (0xFUL << EPIC_DECOMP_CFG1_EXTRA_HIGH_Pos)
#define EPIC_DECOMP_CFG1_EXTRA_HIGH     EPIC_DECOMP_CFG1_EXTRA_HIGH_Msk
#define EPIC_DECOMP_CFG1_EXTRA_THRESHOLD_Pos  (4U)
#define EPIC_DECOMP_CFG1_EXTRA_THRESHOLD_Msk  (0xFUL << EPIC_DECOMP_CFG1_EXTRA_THRESHOLD_Pos)
#define EPIC_DECOMP_CFG1_EXTRA_THRESHOLD  EPIC_DECOMP_CFG1_EXTRA_THRESHOLD_Msk
#define EPIC_DECOMP_CFG1_USE_LOSSLESS_QIDX_Pos  (8U)
#define EPIC_DECOMP_CFG1_USE_LOSSLESS_QIDX_Msk  (0xFUL << EPIC_DECOMP_CFG1_USE_LOSSLESS_QIDX_Pos)
#define EPIC_DECOMP_CFG1_USE_LOSSLESS_QIDX  EPIC_DECOMP_CFG1_USE_LOSSLESS_QIDX_Msk
#define EPIC_DECOMP_CFG1_LOSSLESS_QIDX1_Pos  (12U)
#define EPIC_DECOMP_CFG1_LOSSLESS_QIDX1_Msk  (0xFUL << EPIC_DECOMP_CFG1_LOSSLESS_QIDX1_Pos)
#define EPIC_DECOMP_CFG1_LOSSLESS_QIDX1  EPIC_DECOMP_CFG1_LOSSLESS_QIDX1_Msk
#define EPIC_DECOMP_CFG1_LOSSLESS_QIDX2_Pos  (16U)
#define EPIC_DECOMP_CFG1_LOSSLESS_QIDX2_Msk  (0xFUL << EPIC_DECOMP_CFG1_LOSSLESS_QIDX2_Pos)
#define EPIC_DECOMP_CFG1_LOSSLESS_QIDX2  EPIC_DECOMP_CFG1_LOSSLESS_QIDX2_Msk
#define EPIC_DECOMP_CFG1_RESERVED0_Pos  (20U)
#define EPIC_DECOMP_CFG1_RESERVED0_Msk  (0xFFFUL << EPIC_DECOMP_CFG1_RESERVED0_Pos)
#define EPIC_DECOMP_CFG1_RESERVED0      EPIC_DECOMP_CFG1_RESERVED0_Msk

/**************** Bit definition for EPIC_DECOMP_CFG2 register ****************/
#define EPIC_DECOMP_CFG2_BLOCK_WIDTH_Pos  (0U)
#define EPIC_DECOMP_CFG2_BLOCK_WIDTH_Msk  (0x1UL << EPIC_DECOMP_CFG2_BLOCK_WIDTH_Pos)
#define EPIC_DECOMP_CFG2_BLOCK_WIDTH    EPIC_DECOMP_CFG2_BLOCK_WIDTH_Msk
#define EPIC_DECOMP_CFG2_DITHER_Pos     (1U)
#define EPIC_DECOMP_CFG2_DITHER_Msk     (0x1UL << EPIC_DECOMP_CFG2_DITHER_Pos)
#define EPIC_DECOMP_CFG2_DITHER         EPIC_DECOMP_CFG2_DITHER_Msk
#define EPIC_DECOMP_CFG2_RESERVED1_Pos  (2U)
#define EPIC_DECOMP_CFG2_RESERVED1_Msk  (0x3FUL << EPIC_DECOMP_CFG2_RESERVED1_Pos)
#define EPIC_DECOMP_CFG2_RESERVED1      EPIC_DECOMP_CFG2_RESERVED1_Msk
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_R_Pos  (8U)
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_R_Msk  (0xFUL << EPIC_DECOMP_CFG2_FAILOVER_BITS_R_Pos)
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_R  EPIC_DECOMP_CFG2_FAILOVER_BITS_R_Msk
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_G_Pos  (12U)
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_G_Msk  (0xFUL << EPIC_DECOMP_CFG2_FAILOVER_BITS_G_Pos)
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_G  EPIC_DECOMP_CFG2_FAILOVER_BITS_G_Msk
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_B_Pos  (16U)
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_B_Msk  (0xFUL << EPIC_DECOMP_CFG2_FAILOVER_BITS_B_Pos)
#define EPIC_DECOMP_CFG2_FAILOVER_BITS_B  EPIC_DECOMP_CFG2_FAILOVER_BITS_B_Msk
#define EPIC_DECOMP_CFG2_LINE_MIN_QIDX_Pos  (20U)
#define EPIC_DECOMP_CFG2_LINE_MIN_QIDX_Msk  (0xFUL << EPIC_DECOMP_CFG2_LINE_MIN_QIDX_Pos)
#define EPIC_DECOMP_CFG2_LINE_MIN_QIDX  EPIC_DECOMP_CFG2_LINE_MIN_QIDX_Msk
#define EPIC_DECOMP_CFG2_BLOCK_MIN_QIDX_Pos  (24U)
#define EPIC_DECOMP_CFG2_BLOCK_MIN_QIDX_Msk  (0xFUL << EPIC_DECOMP_CFG2_BLOCK_MIN_QIDX_Pos)
#define EPIC_DECOMP_CFG2_BLOCK_MIN_QIDX  EPIC_DECOMP_CFG2_BLOCK_MIN_QIDX_Msk
#define EPIC_DECOMP_CFG2_EXTRA_LOW_Pos  (28U)
#define EPIC_DECOMP_CFG2_EXTRA_LOW_Msk  (0xFUL << EPIC_DECOMP_CFG2_EXTRA_LOW_Pos)
#define EPIC_DECOMP_CFG2_EXTRA_LOW      EPIC_DECOMP_CFG2_EXTRA_LOW_Msk

/**************** Bit definition for EPIC_DECOMP_STAT register ****************/
#define EPIC_DECOMP_STAT_BUF_MAX_DEPTH_Pos  (0U)
#define EPIC_DECOMP_STAT_BUF_MAX_DEPTH_Msk  (0x7FUL << EPIC_DECOMP_STAT_BUF_MAX_DEPTH_Pos)
#define EPIC_DECOMP_STAT_BUF_MAX_DEPTH  EPIC_DECOMP_STAT_BUF_MAX_DEPTH_Msk

/*************** Bit definition for EPIC_YUV_ENG_CFG0 register ****************/
#define EPIC_YUV_ENG_CFG0_WIDTH_U_Pos   (0U)
#define EPIC_YUV_ENG_CFG0_WIDTH_U_Msk   (0x1FFFUL << EPIC_YUV_ENG_CFG0_WIDTH_U_Pos)
#define EPIC_YUV_ENG_CFG0_WIDTH_U       EPIC_YUV_ENG_CFG0_WIDTH_U_Msk
#define EPIC_YUV_ENG_CFG0_WIDTH_Y_Pos   (13U)
#define EPIC_YUV_ENG_CFG0_WIDTH_Y_Msk   (0x3FFFUL << EPIC_YUV_ENG_CFG0_WIDTH_Y_Pos)
#define EPIC_YUV_ENG_CFG0_WIDTH_Y       EPIC_YUV_ENG_CFG0_WIDTH_Y_Msk
#define EPIC_YUV_ENG_CFG0_FORMAT_Pos    (27U)
#define EPIC_YUV_ENG_CFG0_FORMAT_Msk    (0x7UL << EPIC_YUV_ENG_CFG0_FORMAT_Pos)
#define EPIC_YUV_ENG_CFG0_FORMAT        EPIC_YUV_ENG_CFG0_FORMAT_Msk
#define EPIC_YUV_ENG_CFG0_RANGE_SEL_Pos  (30U)
#define EPIC_YUV_ENG_CFG0_RANGE_SEL_Msk  (0x1UL << EPIC_YUV_ENG_CFG0_RANGE_SEL_Pos)
#define EPIC_YUV_ENG_CFG0_RANGE_SEL     EPIC_YUV_ENG_CFG0_RANGE_SEL_Msk

/*************** Bit definition for EPIC_YUV_ENG_CFG1 register ****************/
#define EPIC_YUV_ENG_CFG1_WIDTH_V_Pos   (0U)
#define EPIC_YUV_ENG_CFG1_WIDTH_V_Msk   (0x1FFFUL << EPIC_YUV_ENG_CFG1_WIDTH_V_Pos)
#define EPIC_YUV_ENG_CFG1_WIDTH_V       EPIC_YUV_ENG_CFG1_WIDTH_V_Msk

/******************* Bit definition for EPIC_Y_SRC register *******************/
#define EPIC_Y_SRC_ADDR_Pos             (0U)
#define EPIC_Y_SRC_ADDR_Msk             (0xFFFFFFFFUL << EPIC_Y_SRC_ADDR_Pos)
#define EPIC_Y_SRC_ADDR                 EPIC_Y_SRC_ADDR_Msk

/******************* Bit definition for EPIC_U_SRC register *******************/
#define EPIC_U_SRC_ADDR_Pos             (0U)
#define EPIC_U_SRC_ADDR_Msk             (0xFFFFFFFFUL << EPIC_U_SRC_ADDR_Pos)
#define EPIC_U_SRC_ADDR                 EPIC_U_SRC_ADDR_Msk

/******************* Bit definition for EPIC_V_SRC register *******************/
#define EPIC_V_SRC_ADDR_Pos             (0U)
#define EPIC_V_SRC_ADDR_Msk             (0xFFFFFFFFUL << EPIC_V_SRC_ADDR_Pos)
#define EPIC_V_SRC_ADDR                 EPIC_V_SRC_ADDR_Msk

/******************* Bit definition for EPIC_COEF0 register *******************/
#define EPIC_COEF0_FUB_Pos              (0U)
#define EPIC_COEF0_FUB_Msk              (0x3FFUL << EPIC_COEF0_FUB_Pos)
#define EPIC_COEF0_FUB                  EPIC_COEF0_FUB_Msk
#define EPIC_COEF0_FUG_Pos              (10U)
#define EPIC_COEF0_FUG_Msk              (0x3FFUL << EPIC_COEF0_FUG_Pos)
#define EPIC_COEF0_FUG                  EPIC_COEF0_FUG_Msk
#define EPIC_COEF0_FY_Pos               (20U)
#define EPIC_COEF0_FY_Msk               (0x3FFUL << EPIC_COEF0_FY_Pos)
#define EPIC_COEF0_FY                   EPIC_COEF0_FY_Msk

/******************* Bit definition for EPIC_COEF1 register *******************/
#define EPIC_COEF1_FVG_Pos              (0U)
#define EPIC_COEF1_FVG_Msk              (0x3FFUL << EPIC_COEF1_FVG_Pos)
#define EPIC_COEF1_FVG                  EPIC_COEF1_FVG_Msk
#define EPIC_COEF1_FVR_Pos              (10U)
#define EPIC_COEF1_FVR_Msk              (0x3FFUL << EPIC_COEF1_FVR_Pos)
#define EPIC_COEF1_FVR                  EPIC_COEF1_FVR_Msk

/*************** Bit definition for EPIC_CM_ENG_CONF0 register ****************/
#define EPIC_CM_ENG_CONF0_COEF_A0_Pos   (0U)
#define EPIC_CM_ENG_CONF0_COEF_A0_Msk   (0x3FFUL << EPIC_CM_ENG_CONF0_COEF_A0_Pos)
#define EPIC_CM_ENG_CONF0_COEF_A0       EPIC_CM_ENG_CONF0_COEF_A0_Msk
#define EPIC_CM_ENG_CONF0_COEF_B0_Pos   (10U)
#define EPIC_CM_ENG_CONF0_COEF_B0_Msk   (0x3FFUL << EPIC_CM_ENG_CONF0_COEF_B0_Pos)
#define EPIC_CM_ENG_CONF0_COEF_B0       EPIC_CM_ENG_CONF0_COEF_B0_Msk
#define EPIC_CM_ENG_CONF0_COEF_C0_Pos   (20U)
#define EPIC_CM_ENG_CONF0_COEF_C0_Msk   (0x3FFUL << EPIC_CM_ENG_CONF0_COEF_C0_Pos)
#define EPIC_CM_ENG_CONF0_COEF_C0       EPIC_CM_ENG_CONF0_COEF_C0_Msk

/*************** Bit definition for EPIC_CM_ENG_CONF1 register ****************/
#define EPIC_CM_ENG_CONF1_COEF_D0_Pos   (0U)
#define EPIC_CM_ENG_CONF1_COEF_D0_Msk   (0x3FFUL << EPIC_CM_ENG_CONF1_COEF_D0_Pos)
#define EPIC_CM_ENG_CONF1_COEF_D0       EPIC_CM_ENG_CONF1_COEF_D0_Msk
#define EPIC_CM_ENG_CONF1_COEF_E0_Pos   (10U)
#define EPIC_CM_ENG_CONF1_COEF_E0_Msk   (0x3FFFFUL << EPIC_CM_ENG_CONF1_COEF_E0_Pos)
#define EPIC_CM_ENG_CONF1_COEF_E0       EPIC_CM_ENG_CONF1_COEF_E0_Msk

/*************** Bit definition for EPIC_CM_ENG_CONF2 register ****************/
#define EPIC_CM_ENG_CONF2_COEF_A1_Pos   (0U)
#define EPIC_CM_ENG_CONF2_COEF_A1_Msk   (0x3FFUL << EPIC_CM_ENG_CONF2_COEF_A1_Pos)
#define EPIC_CM_ENG_CONF2_COEF_A1       EPIC_CM_ENG_CONF2_COEF_A1_Msk
#define EPIC_CM_ENG_CONF2_COEF_B1_Pos   (10U)
#define EPIC_CM_ENG_CONF2_COEF_B1_Msk   (0x3FFUL << EPIC_CM_ENG_CONF2_COEF_B1_Pos)
#define EPIC_CM_ENG_CONF2_COEF_B1       EPIC_CM_ENG_CONF2_COEF_B1_Msk
#define EPIC_CM_ENG_CONF2_COEF_C1_Pos   (20U)
#define EPIC_CM_ENG_CONF2_COEF_C1_Msk   (0x3FFUL << EPIC_CM_ENG_CONF2_COEF_C1_Pos)
#define EPIC_CM_ENG_CONF2_COEF_C1       EPIC_CM_ENG_CONF2_COEF_C1_Msk

/*************** Bit definition for EPIC_CM_ENG_CONF3 register ****************/
#define EPIC_CM_ENG_CONF3_COEF_D1_Pos   (0U)
#define EPIC_CM_ENG_CONF3_COEF_D1_Msk   (0x3FFUL << EPIC_CM_ENG_CONF3_COEF_D1_Pos)
#define EPIC_CM_ENG_CONF3_COEF_D1       EPIC_CM_ENG_CONF3_COEF_D1_Msk
#define EPIC_CM_ENG_CONF3_COEF_E1_Pos   (10U)
#define EPIC_CM_ENG_CONF3_COEF_E1_Msk   (0x3FFFFUL << EPIC_CM_ENG_CONF3_COEF_E1_Pos)
#define EPIC_CM_ENG_CONF3_COEF_E1       EPIC_CM_ENG_CONF3_COEF_E1_Msk

/*************** Bit definition for EPIC_CM_ENG_CONF4 register ****************/
#define EPIC_CM_ENG_CONF4_COEF_A2_Pos   (0U)
#define EPIC_CM_ENG_CONF4_COEF_A2_Msk   (0x3FFUL << EPIC_CM_ENG_CONF4_COEF_A2_Pos)
#define EPIC_CM_ENG_CONF4_COEF_A2       EPIC_CM_ENG_CONF4_COEF_A2_Msk
#define EPIC_CM_ENG_CONF4_COEF_B2_Pos   (10U)
#define EPIC_CM_ENG_CONF4_COEF_B2_Msk   (0x3FFUL << EPIC_CM_ENG_CONF4_COEF_B2_Pos)
#define EPIC_CM_ENG_CONF4_COEF_B2       EPIC_CM_ENG_CONF4_COEF_B2_Msk
#define EPIC_CM_ENG_CONF4_COEF_C2_Pos   (20U)
#define EPIC_CM_ENG_CONF4_COEF_C2_Msk   (0x3FFUL << EPIC_CM_ENG_CONF4_COEF_C2_Pos)
#define EPIC_CM_ENG_CONF4_COEF_C2       EPIC_CM_ENG_CONF4_COEF_C2_Msk

/*************** Bit definition for EPIC_CM_ENG_CONF5 register ****************/
#define EPIC_CM_ENG_CONF5_COEF_D2_Pos   (0U)
#define EPIC_CM_ENG_CONF5_COEF_D2_Msk   (0x3FFUL << EPIC_CM_ENG_CONF5_COEF_D2_Pos)
#define EPIC_CM_ENG_CONF5_COEF_D2       EPIC_CM_ENG_CONF5_COEF_D2_Msk
#define EPIC_CM_ENG_CONF5_COEF_E2_Pos   (10U)
#define EPIC_CM_ENG_CONF5_COEF_E2_Msk   (0x3FFFFUL << EPIC_CM_ENG_CONF5_COEF_E2_Pos)
#define EPIC_CM_ENG_CONF5_COEF_E2       EPIC_CM_ENG_CONF5_COEF_E2_Msk

/*************** Bit definition for EPIC_CM_ENG_CONF6 register ****************/
#define EPIC_CM_ENG_CONF6_COEF_A3_Pos   (0U)
#define EPIC_CM_ENG_CONF6_COEF_A3_Msk   (0x3FFUL << EPIC_CM_ENG_CONF6_COEF_A3_Pos)
#define EPIC_CM_ENG_CONF6_COEF_A3       EPIC_CM_ENG_CONF6_COEF_A3_Msk
#define EPIC_CM_ENG_CONF6_COEF_B3_Pos   (10U)
#define EPIC_CM_ENG_CONF6_COEF_B3_Msk   (0x3FFUL << EPIC_CM_ENG_CONF6_COEF_B3_Pos)
#define EPIC_CM_ENG_CONF6_COEF_B3       EPIC_CM_ENG_CONF6_COEF_B3_Msk
#define EPIC_CM_ENG_CONF6_COEF_C3_Pos   (20U)
#define EPIC_CM_ENG_CONF6_COEF_C3_Msk   (0x3FFUL << EPIC_CM_ENG_CONF6_COEF_C3_Pos)
#define EPIC_CM_ENG_CONF6_COEF_C3       EPIC_CM_ENG_CONF6_COEF_C3_Msk

/*************** Bit definition for EPIC_CM_ENG_CONF7 register ****************/
#define EPIC_CM_ENG_CONF7_COEF_D3_Pos   (0U)
#define EPIC_CM_ENG_CONF7_COEF_D3_Msk   (0x3FFUL << EPIC_CM_ENG_CONF7_COEF_D3_Pos)
#define EPIC_CM_ENG_CONF7_COEF_D3       EPIC_CM_ENG_CONF7_COEF_D3_Msk
#define EPIC_CM_ENG_CONF7_COEF_E3_Pos   (10U)
#define EPIC_CM_ENG_CONF7_COEF_E3_Msk   (0x3FFFFUL << EPIC_CM_ENG_CONF7_COEF_E3_Pos)
#define EPIC_CM_ENG_CONF7_COEF_E3       EPIC_CM_ENG_CONF7_COEF_E3_Msk

/**************** Bit definition for EPIC_DITHER_CONF register ****************/
#define EPIC_DITHER_CONF_RD_EN_Pos      (0U)
#define EPIC_DITHER_CONF_RD_EN_Msk      (0x1UL << EPIC_DITHER_CONF_RD_EN_Pos)
#define EPIC_DITHER_CONF_RD_EN          EPIC_DITHER_CONF_RD_EN_Msk
#define EPIC_DITHER_CONF_RD_BLUEW_Pos   (1U)
#define EPIC_DITHER_CONF_RD_BLUEW_Msk   (0x7UL << EPIC_DITHER_CONF_RD_BLUEW_Pos)
#define EPIC_DITHER_CONF_RD_BLUEW       EPIC_DITHER_CONF_RD_BLUEW_Msk
#define EPIC_DITHER_CONF_RD_GREENW_Pos  (4U)
#define EPIC_DITHER_CONF_RD_GREENW_Msk  (0x7UL << EPIC_DITHER_CONF_RD_GREENW_Pos)
#define EPIC_DITHER_CONF_RD_GREENW      EPIC_DITHER_CONF_RD_GREENW_Msk
#define EPIC_DITHER_CONF_RD_REDW_Pos    (7U)
#define EPIC_DITHER_CONF_RD_REDW_Msk    (0x7UL << EPIC_DITHER_CONF_RD_REDW_Pos)
#define EPIC_DITHER_CONF_RD_REDW        EPIC_DITHER_CONF_RD_REDW_Msk
#define EPIC_DITHER_CONF_LFSR_LOAD_SEL_Pos  (10U)
#define EPIC_DITHER_CONF_LFSR_LOAD_SEL_Msk  (0x3UL << EPIC_DITHER_CONF_LFSR_LOAD_SEL_Pos)
#define EPIC_DITHER_CONF_LFSR_LOAD_SEL  EPIC_DITHER_CONF_LFSR_LOAD_SEL_Msk
#define EPIC_DITHER_CONF_LFSR_LOAD_Pos  (12U)
#define EPIC_DITHER_CONF_LFSR_LOAD_Msk  (0x1UL << EPIC_DITHER_CONF_LFSR_LOAD_Pos)
#define EPIC_DITHER_CONF_LFSR_LOAD      EPIC_DITHER_CONF_LFSR_LOAD_Msk
#define EPIC_DITHER_CONF_FSD_EN_Pos     (13U)
#define EPIC_DITHER_CONF_FSD_EN_Msk     (0x1UL << EPIC_DITHER_CONF_FSD_EN_Pos)
#define EPIC_DITHER_CONF_FSD_EN         EPIC_DITHER_CONF_FSD_EN_Msk
#define EPIC_DITHER_CONF_FSD_BLUEW_Pos  (14U)
#define EPIC_DITHER_CONF_FSD_BLUEW_Msk  (0x7UL << EPIC_DITHER_CONF_FSD_BLUEW_Pos)
#define EPIC_DITHER_CONF_FSD_BLUEW      EPIC_DITHER_CONF_FSD_BLUEW_Msk
#define EPIC_DITHER_CONF_FSD_GREENW_Pos  (17U)
#define EPIC_DITHER_CONF_FSD_GREENW_Msk  (0x7UL << EPIC_DITHER_CONF_FSD_GREENW_Pos)
#define EPIC_DITHER_CONF_FSD_GREENW     EPIC_DITHER_CONF_FSD_GREENW_Msk
#define EPIC_DITHER_CONF_FSD_REDW_Pos   (20U)
#define EPIC_DITHER_CONF_FSD_REDW_Msk   (0x7UL << EPIC_DITHER_CONF_FSD_REDW_Pos)
#define EPIC_DITHER_CONF_FSD_REDW       EPIC_DITHER_CONF_FSD_REDW_Msk

/**************** Bit definition for EPIC_DITHER_LFSR register ****************/
#define EPIC_DITHER_LFSR_INIT_VAL_Pos   (0U)
#define EPIC_DITHER_LFSR_INIT_VAL_Msk   (0xFFFFFFFFUL << EPIC_DITHER_LFSR_INIT_VAL_Pos)
#define EPIC_DITHER_LFSR_INIT_VAL       EPIC_DITHER_LFSR_INIT_VAL_Msk

/***************** Bit definition for EPIC_GREY_CONV register *****************/
#define EPIC_GREY_CONV_COEF0_Pos        (0U)
#define EPIC_GREY_CONV_COEF0_Msk        (0x3FFUL << EPIC_GREY_CONV_COEF0_Pos)
#define EPIC_GREY_CONV_COEF0            EPIC_GREY_CONV_COEF0_Msk
#define EPIC_GREY_CONV_COEF1_Pos        (10U)
#define EPIC_GREY_CONV_COEF1_Msk        (0x3FFUL << EPIC_GREY_CONV_COEF1_Pos)
#define EPIC_GREY_CONV_COEF1            EPIC_GREY_CONV_COEF1_Msk
#define EPIC_GREY_CONV_COEF2_Pos        (20U)
#define EPIC_GREY_CONV_COEF2_Msk        (0x3FFUL << EPIC_GREY_CONV_COEF2_Pos)
#define EPIC_GREY_CONV_COEF2            EPIC_GREY_CONV_COEF2_Msk

/**************** Bit definition for EPIC_FLEX_SEL_L register *****************/
#define EPIC_FLEX_SEL_L_BIT0_Pos        (0U)
#define EPIC_FLEX_SEL_L_BIT0_Msk        (0x1FUL << EPIC_FLEX_SEL_L_BIT0_Pos)
#define EPIC_FLEX_SEL_L_BIT0            EPIC_FLEX_SEL_L_BIT0_Msk
#define EPIC_FLEX_SEL_L_BIT1_Pos        (5U)
#define EPIC_FLEX_SEL_L_BIT1_Msk        (0x1FUL << EPIC_FLEX_SEL_L_BIT1_Pos)
#define EPIC_FLEX_SEL_L_BIT1            EPIC_FLEX_SEL_L_BIT1_Msk
#define EPIC_FLEX_SEL_L_BIT2_Pos        (10U)
#define EPIC_FLEX_SEL_L_BIT2_Msk        (0x1FUL << EPIC_FLEX_SEL_L_BIT2_Pos)
#define EPIC_FLEX_SEL_L_BIT2            EPIC_FLEX_SEL_L_BIT2_Msk
#define EPIC_FLEX_SEL_L_BIT3_Pos        (15U)
#define EPIC_FLEX_SEL_L_BIT3_Msk        (0x1FUL << EPIC_FLEX_SEL_L_BIT3_Pos)
#define EPIC_FLEX_SEL_L_BIT3            EPIC_FLEX_SEL_L_BIT3_Msk

/**************** Bit definition for EPIC_FLEX_SEL_H register *****************/
#define EPIC_FLEX_SEL_H_BIT4_Pos        (0U)
#define EPIC_FLEX_SEL_H_BIT4_Msk        (0x1FUL << EPIC_FLEX_SEL_H_BIT4_Pos)
#define EPIC_FLEX_SEL_H_BIT4            EPIC_FLEX_SEL_H_BIT4_Msk
#define EPIC_FLEX_SEL_H_BIT5_Pos        (5U)
#define EPIC_FLEX_SEL_H_BIT5_Msk        (0x1FUL << EPIC_FLEX_SEL_H_BIT5_Pos)
#define EPIC_FLEX_SEL_H_BIT5            EPIC_FLEX_SEL_H_BIT5_Msk
#define EPIC_FLEX_SEL_H_BIT6_Pos        (10U)
#define EPIC_FLEX_SEL_H_BIT6_Msk        (0x1FUL << EPIC_FLEX_SEL_H_BIT6_Pos)
#define EPIC_FLEX_SEL_H_BIT6            EPIC_FLEX_SEL_H_BIT6_Msk
#define EPIC_FLEX_SEL_H_BIT7_Pos        (15U)
#define EPIC_FLEX_SEL_H_BIT7_Msk        (0x1FUL << EPIC_FLEX_SEL_H_BIT7_Pos)
#define EPIC_FLEX_SEL_H_BIT7            EPIC_FLEX_SEL_H_BIT7_Msk

/***************** Bit definition for EPIC_AHB_CTRL register ******************/
#define EPIC_AHB_CTRL_DESTINATION_Pos   (0U)
#define EPIC_AHB_CTRL_DESTINATION_Msk   (0x1UL << EPIC_AHB_CTRL_DESTINATION_Pos)
#define EPIC_AHB_CTRL_DESTINATION       EPIC_AHB_CTRL_DESTINATION_Msk
#define EPIC_AHB_CTRL_DEST_RAM          (0UL << EPIC_AHB_CTRL_DESTINATION_Pos)
#define EPIC_AHB_CTRL_DEST_LCD          (1UL << EPIC_AHB_CTRL_DESTINATION_Pos)
#define EPIC_AHB_CTRL_O_FORMAT_Pos      (1U)
#define EPIC_AHB_CTRL_O_FORMAT_Msk      (0xFUL << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FORMAT          EPIC_AHB_CTRL_O_FORMAT_Msk
#define EPIC_AHB_CTRL_O_FMT_RGB565      (0 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_RGB888      (1 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_ARGB8888    (2 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_ARGB8565    (3 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_A8          (4 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_GRAY8       (5 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_GRAY4       (6 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_GRAY2       (7 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_F8          (8 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_F4          (9 << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_O_FMT_F2          (0xA << EPIC_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_AHB_CTRL_AHB_SEL_Pos       (5U)
#define EPIC_AHB_CTRL_AHB_SEL_Msk       (0x1UL << EPIC_AHB_CTRL_AHB_SEL_Pos)
#define EPIC_AHB_CTRL_AHB_SEL           EPIC_AHB_CTRL_AHB_SEL_Msk
#define EPIC_AHB_CTRL_AHB_DIV_EN_Pos    (6U)
#define EPIC_AHB_CTRL_AHB_DIV_EN_Msk    (0x1UL << EPIC_AHB_CTRL_AHB_DIV_EN_Pos)
#define EPIC_AHB_CTRL_AHB_DIV_EN        EPIC_AHB_CTRL_AHB_DIV_EN_Msk
#define EPIC_AHB_CTRL_GREY_ORDER_Pos    (7U)
#define EPIC_AHB_CTRL_GREY_ORDER_Msk    (0x1UL << EPIC_AHB_CTRL_GREY_ORDER_Pos)
#define EPIC_AHB_CTRL_GREY_ORDER        EPIC_AHB_CTRL_GREY_ORDER_Msk

/****************** Bit definition for EPIC_AHB_MEM register ******************/
#define EPIC_AHB_MEM_ADDR_Pos           (0U)
#define EPIC_AHB_MEM_ADDR_Msk           (0xFFFFFFFFUL << EPIC_AHB_MEM_ADDR_Pos)
#define EPIC_AHB_MEM_ADDR               EPIC_AHB_MEM_ADDR_Msk

/**************** Bit definition for EPIC_AHB_STRIDE register *****************/
#define EPIC_AHB_STRIDE_OFFSET_Pos      (0U)
#define EPIC_AHB_STRIDE_OFFSET_Msk      (0xFFFFUL << EPIC_AHB_STRIDE_OFFSET_Pos)
#define EPIC_AHB_STRIDE_OFFSET          EPIC_AHB_STRIDE_OFFSET_Msk

/******************* Bit definition for EPIC_DEBUG register *******************/
#define EPIC_DEBUG_DEBUG_OUT_SEL_Pos    (0U)
#define EPIC_DEBUG_DEBUG_OUT_SEL_Msk    (0xFUL << EPIC_DEBUG_DEBUG_OUT_SEL_Pos)
#define EPIC_DEBUG_DEBUG_OUT_SEL        EPIC_DEBUG_DEBUG_OUT_SEL_Msk
#define EPIC_DEBUG_DEBUG_INT_SEL_Pos    (4U)
#define EPIC_DEBUG_DEBUG_INT_SEL_Msk    (0xFUL << EPIC_DEBUG_DEBUG_INT_SEL_Pos)
#define EPIC_DEBUG_DEBUG_INT_SEL        EPIC_DEBUG_DEBUG_INT_SEL_Msk
#define EPIC_DEBUG_DEBUG_INT_DATA_Pos   (16U)
#define EPIC_DEBUG_DEBUG_INT_DATA_Msk   (0xFFFFUL << EPIC_DEBUG_DEBUG_INT_DATA_Pos)
#define EPIC_DEBUG_DEBUG_INT_DATA       EPIC_DEBUG_DEBUG_INT_DATA_Msk

/*************** Bit definition for EPIC_VL_ROT_M_CFG1 register ***************/
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Pos  (0U)
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Msk  (0x3FFFUL << EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Pos)
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE  EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Msk
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Pos  (16U)
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Msk  (0x3FFFUL << EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Pos)
#define EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL  EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Msk
#define EPIC_VL_ROT_M_CFG1_M_MODE_Pos   (31U)
#define EPIC_VL_ROT_M_CFG1_M_MODE_Msk   (0x1UL << EPIC_VL_ROT_M_CFG1_M_MODE_Pos)
#define EPIC_VL_ROT_M_CFG1_M_MODE       EPIC_VL_ROT_M_CFG1_M_MODE_Msk

/*************** Bit definition for EPIC_VL_ROT_M_CFG2 register ***************/
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Pos  (0U)
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Msk  (0x3FFFUL << EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Pos)
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_X    EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Msk
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Pos  (16U)
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Msk  (0x3FFFUL << EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Pos)
#define EPIC_VL_ROT_M_CFG2_M_PIVOT_Y    EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Msk

/*************** Bit definition for EPIC_VL_ROT_M_CFG3 register ***************/
#define EPIC_VL_ROT_M_CFG3_M_XTL_Pos    (0U)
#define EPIC_VL_ROT_M_CFG3_M_XTL_Msk    (0x3FFFUL << EPIC_VL_ROT_M_CFG3_M_XTL_Pos)
#define EPIC_VL_ROT_M_CFG3_M_XTL        EPIC_VL_ROT_M_CFG3_M_XTL_Msk
#define EPIC_VL_ROT_M_CFG3_M_YTL_Pos    (16U)
#define EPIC_VL_ROT_M_CFG3_M_YTL_Msk    (0x3FFFUL << EPIC_VL_ROT_M_CFG3_M_YTL_Pos)
#define EPIC_VL_ROT_M_CFG3_M_YTL        EPIC_VL_ROT_M_CFG3_M_YTL_Msk

/************ Bit definition for EPIC_VL_SCALE_INIT_CFG1 register *************/
#define EPIC_VL_SCALE_INIT_CFG1_X_VAL_Pos  (0U)
#define EPIC_VL_SCALE_INIT_CFG1_X_VAL_Msk  (0x1FFFFFFFUL << EPIC_VL_SCALE_INIT_CFG1_X_VAL_Pos)
#define EPIC_VL_SCALE_INIT_CFG1_X_VAL   EPIC_VL_SCALE_INIT_CFG1_X_VAL_Msk

/************ Bit definition for EPIC_VL_SCALE_INIT_CFG2 register *************/
#define EPIC_VL_SCALE_INIT_CFG2_Y_VAL_Pos  (0U)
#define EPIC_VL_SCALE_INIT_CFG2_Y_VAL_Msk  (0x1FFFFFFFUL << EPIC_VL_SCALE_INIT_CFG2_Y_VAL_Pos)
#define EPIC_VL_SCALE_INIT_CFG2_Y_VAL   EPIC_VL_SCALE_INIT_CFG2_Y_VAL_Msk

/***************** Bit definition for EPIC_AHBA_CNT register ******************/
#define EPIC_AHBA_CNT_VAL_Pos           (0U)
#define EPIC_AHBA_CNT_VAL_Msk           (0xFFFFFFFFUL << EPIC_AHBA_CNT_VAL_Pos)
#define EPIC_AHBA_CNT_VAL               EPIC_AHBA_CNT_VAL_Msk

/***************** Bit definition for EPIC_AHBB_CNT register ******************/
#define EPIC_AHBB_CNT_VAL_Pos           (0U)
#define EPIC_AHBB_CNT_VAL_Msk           (0xFFFFFFFFUL << EPIC_AHBB_CNT_VAL_Pos)
#define EPIC_AHBB_CNT_VAL               EPIC_AHBB_CNT_VAL_Msk

/***************** Bit definition for EPIC_PERF_CNT register ******************/
#define EPIC_PERF_CNT_VAL_Pos           (0U)
#define EPIC_PERF_CNT_VAL_Msk           (0xFFFFFFFFUL << EPIC_PERF_CNT_VAL_Pos)
#define EPIC_PERF_CNT_VAL               EPIC_PERF_CNT_VAL_Msk

/****************** Bit definition for EPIC_TL_CFG register *******************/
#define EPIC_TL_CFG_FORMAT_Pos          (0U)
#define EPIC_TL_CFG_FORMAT_Msk          (0xFUL << EPIC_TL_CFG_FORMAT_Pos)
#define EPIC_TL_CFG_FORMAT              EPIC_TL_CFG_FORMAT_Msk
#define EPIC_TL_CFG_WIDTH_Pos           (16U)
#define EPIC_TL_CFG_WIDTH_Msk           (0x7FFFUL << EPIC_TL_CFG_WIDTH_Pos)
#define EPIC_TL_CFG_WIDTH               EPIC_TL_CFG_WIDTH_Msk

/****************** Bit definition for EPIC_TL_SIZE register ******************/
#define EPIC_TL_SIZE_MAX_LINE_Pos       (0U)
#define EPIC_TL_SIZE_MAX_LINE_Msk       (0x1FFFUL << EPIC_TL_SIZE_MAX_LINE_Pos)
#define EPIC_TL_SIZE_MAX_LINE           EPIC_TL_SIZE_MAX_LINE_Msk
#define EPIC_TL_SIZE_MAX_COL_Pos        (16U)
#define EPIC_TL_SIZE_MAX_COL_Msk        (0x1FFFUL << EPIC_TL_SIZE_MAX_COL_Pos)
#define EPIC_TL_SIZE_MAX_COL            EPIC_TL_SIZE_MAX_COL_Msk

/****************** Bit definition for EPIC_TL_SRC register *******************/
#define EPIC_TL_SRC_ADDR_Pos            (0U)
#define EPIC_TL_SRC_ADDR_Msk            (0xFFFFFFFFUL << EPIC_TL_SRC_ADDR_Pos)
#define EPIC_TL_SRC_ADDR                EPIC_TL_SRC_ADDR_Msk

/****************** Bit definition for EPIC_CMPRCR register *******************/
#define EPIC_CMPRCR_LINESIZE_Pos        (0U)
#define EPIC_CMPRCR_LINESIZE_Msk        (0xFFFUL << EPIC_CMPRCR_LINESIZE_Pos)
#define EPIC_CMPRCR_LINESIZE            EPIC_CMPRCR_LINESIZE_Msk
#define EPIC_CMPRCR_CHUNKCNT_Pos        (12U)
#define EPIC_CMPRCR_CHUNKCNT_Msk        (0xFUL << EPIC_CMPRCR_CHUNKCNT_Pos)
#define EPIC_CMPRCR_CHUNKCNT            EPIC_CMPRCR_CHUNKCNT_Msk
#define EPIC_CMPRCR_TGTSIZE_Pos         (16U)
#define EPIC_CMPRCR_TGTSIZE_Msk         (0xFFFUL << EPIC_CMPRCR_TGTSIZE_Pos)
#define EPIC_CMPRCR_TGTSIZE             EPIC_CMPRCR_TGTSIZE_Msk
#define EPIC_CMPRCR_CMPREN_Pos          (31U)
#define EPIC_CMPRCR_CMPREN_Msk          (0x1UL << EPIC_CMPRCR_CMPREN_Pos)
#define EPIC_CMPRCR_CMPREN              EPIC_CMPRCR_CMPREN_Msk

/***************** Bit definition for EPIC_CMPRCFG0 register ******************/
#define EPIC_CMPRCFG0_CFG_Pos           (0U)
#define EPIC_CMPRCFG0_CFG_Msk           (0xFFFFFFFFUL << EPIC_CMPRCFG0_CFG_Pos)
#define EPIC_CMPRCFG0_CFG               EPIC_CMPRCFG0_CFG_Msk

/***************** Bit definition for EPIC_CMPRCFG1 register ******************/
#define EPIC_CMPRCFG1_CFG_Pos           (0U)
#define EPIC_CMPRCFG1_CFG_Msk           (0xFFFFFFFFUL << EPIC_CMPRCFG1_CFG_Pos)
#define EPIC_CMPRCFG1_CFG               EPIC_CMPRCFG1_CFG_Msk

/****************** Bit definition for EPIC_CMPRQR register *******************/
#define EPIC_CMPRQR_LQR_Pos             (0U)
#define EPIC_CMPRQR_LQR_Msk             (0xFFUL << EPIC_CMPRQR_LQR_Pos)
#define EPIC_CMPRQR_LQR                 EPIC_CMPRQR_LQR_Msk
#define EPIC_CMPRQR_LQB_Pos             (8U)
#define EPIC_CMPRQR_LQB_Msk             (0xFFUL << EPIC_CMPRQR_LQB_Pos)
#define EPIC_CMPRQR_LQB                 EPIC_CMPRQR_LQB_Msk
#define EPIC_CMPRQR_DUMMY_Pos           (16U)
#define EPIC_CMPRQR_DUMMY_Msk           (0x1FFFUL << EPIC_CMPRQR_DUMMY_Pos)
#define EPIC_CMPRQR_DUMMY               EPIC_CMPRQR_DUMMY_Msk

/****************** Bit definition for EPIC_CMPRDR register *******************/
#define EPIC_CMPRDR_MAXBUF_Pos          (0U)
#define EPIC_CMPRDR_MAXBUF_Msk          (0x7FUL << EPIC_CMPRDR_MAXBUF_Pos)
#define EPIC_CMPRDR_MAXBUF              EPIC_CMPRDR_MAXBUF_Msk

/*************** Bit definition for EPIC_TL_GREY_CONV register ****************/
#define EPIC_TL_GREY_CONV_COEF0_Pos     (0U)
#define EPIC_TL_GREY_CONV_COEF0_Msk     (0x3FFUL << EPIC_TL_GREY_CONV_COEF0_Pos)
#define EPIC_TL_GREY_CONV_COEF0         EPIC_TL_GREY_CONV_COEF0_Msk
#define EPIC_TL_GREY_CONV_COEF1_Pos     (10U)
#define EPIC_TL_GREY_CONV_COEF1_Msk     (0x3FFUL << EPIC_TL_GREY_CONV_COEF1_Pos)
#define EPIC_TL_GREY_CONV_COEF1         EPIC_TL_GREY_CONV_COEF1_Msk
#define EPIC_TL_GREY_CONV_COEF2_Pos     (20U)
#define EPIC_TL_GREY_CONV_COEF2_Msk     (0x3FFUL << EPIC_TL_GREY_CONV_COEF2_Pos)
#define EPIC_TL_GREY_CONV_COEF2         EPIC_TL_GREY_CONV_COEF2_Msk

/*************** Bit definition for EPIC_TL_FLEX_SEL_L register ***************/
#define EPIC_TL_FLEX_SEL_L_BIT0_Pos     (0U)
#define EPIC_TL_FLEX_SEL_L_BIT0_Msk     (0x1FUL << EPIC_TL_FLEX_SEL_L_BIT0_Pos)
#define EPIC_TL_FLEX_SEL_L_BIT0         EPIC_TL_FLEX_SEL_L_BIT0_Msk
#define EPIC_TL_FLEX_SEL_L_BIT1_Pos     (5U)
#define EPIC_TL_FLEX_SEL_L_BIT1_Msk     (0x1FUL << EPIC_TL_FLEX_SEL_L_BIT1_Pos)
#define EPIC_TL_FLEX_SEL_L_BIT1         EPIC_TL_FLEX_SEL_L_BIT1_Msk
#define EPIC_TL_FLEX_SEL_L_BIT2_Pos     (10U)
#define EPIC_TL_FLEX_SEL_L_BIT2_Msk     (0x1FUL << EPIC_TL_FLEX_SEL_L_BIT2_Pos)
#define EPIC_TL_FLEX_SEL_L_BIT2         EPIC_TL_FLEX_SEL_L_BIT2_Msk
#define EPIC_TL_FLEX_SEL_L_BIT3_Pos     (15U)
#define EPIC_TL_FLEX_SEL_L_BIT3_Msk     (0x1FUL << EPIC_TL_FLEX_SEL_L_BIT3_Pos)
#define EPIC_TL_FLEX_SEL_L_BIT3         EPIC_TL_FLEX_SEL_L_BIT3_Msk

/*************** Bit definition for EPIC_TL_FLEX_SEL_H register ***************/
#define EPIC_TL_FLEX_SEL_H_BIT4_Pos     (0U)
#define EPIC_TL_FLEX_SEL_H_BIT4_Msk     (0x1FUL << EPIC_TL_FLEX_SEL_H_BIT4_Pos)
#define EPIC_TL_FLEX_SEL_H_BIT4         EPIC_TL_FLEX_SEL_H_BIT4_Msk
#define EPIC_TL_FLEX_SEL_H_BIT5_Pos     (5U)
#define EPIC_TL_FLEX_SEL_H_BIT5_Msk     (0x1FUL << EPIC_TL_FLEX_SEL_H_BIT5_Pos)
#define EPIC_TL_FLEX_SEL_H_BIT5         EPIC_TL_FLEX_SEL_H_BIT5_Msk
#define EPIC_TL_FLEX_SEL_H_BIT6_Pos     (10U)
#define EPIC_TL_FLEX_SEL_H_BIT6_Msk     (0x1FUL << EPIC_TL_FLEX_SEL_H_BIT6_Pos)
#define EPIC_TL_FLEX_SEL_H_BIT6         EPIC_TL_FLEX_SEL_H_BIT6_Msk
#define EPIC_TL_FLEX_SEL_H_BIT7_Pos     (15U)
#define EPIC_TL_FLEX_SEL_H_BIT7_Msk     (0x1FUL << EPIC_TL_FLEX_SEL_H_BIT7_Pos)
#define EPIC_TL_FLEX_SEL_H_BIT7         EPIC_TL_FLEX_SEL_H_BIT7_Msk

/**************** Bit definition for EPIC_TL_AHB_CTRL register ****************/
#define EPIC_TL_AHB_CTRL_O_FORMAT_Pos   (0U)
#define EPIC_TL_AHB_CTRL_O_FORMAT_Msk   (0xFUL << EPIC_TL_AHB_CTRL_O_FORMAT_Pos)
#define EPIC_TL_AHB_CTRL_O_FORMAT       EPIC_TL_AHB_CTRL_O_FORMAT_Msk
#define EPIC_TL_AHB_CTRL_AHB_DIV_EN_Pos  (4U)
#define EPIC_TL_AHB_CTRL_AHB_DIV_EN_Msk  (0x1UL << EPIC_TL_AHB_CTRL_AHB_DIV_EN_Pos)
#define EPIC_TL_AHB_CTRL_AHB_DIV_EN     EPIC_TL_AHB_CTRL_AHB_DIV_EN_Msk
#define EPIC_TL_AHB_CTRL_GREY_ORDER_Pos  (5U)
#define EPIC_TL_AHB_CTRL_GREY_ORDER_Msk  (0x1UL << EPIC_TL_AHB_CTRL_GREY_ORDER_Pos)
#define EPIC_TL_AHB_CTRL_GREY_ORDER     EPIC_TL_AHB_CTRL_GREY_ORDER_Msk

/**************** Bit definition for EPIC_TL_AHB_MEM register *****************/
#define EPIC_TL_AHB_MEM_ADDR_Pos        (0U)
#define EPIC_TL_AHB_MEM_ADDR_Msk        (0xFFFFFFFFUL << EPIC_TL_AHB_MEM_ADDR_Pos)
#define EPIC_TL_AHB_MEM_ADDR            EPIC_TL_AHB_MEM_ADDR_Msk

/*************** Bit definition for EPIC_TL_AHB_STRIDE register ***************/
#define EPIC_TL_AHB_STRIDE_OFFSET_Pos   (0U)
#define EPIC_TL_AHB_STRIDE_OFFSET_Msk   (0xFFFFUL << EPIC_TL_AHB_STRIDE_OFFSET_Pos)
#define EPIC_TL_AHB_STRIDE_OFFSET       EPIC_TL_AHB_STRIDE_OFFSET_Msk

/**************** Bit definition for EPIC_TL_COMMAND register *****************/
#define EPIC_TL_COMMAND_START_Pos       (0U)
#define EPIC_TL_COMMAND_START_Msk       (0x1UL << EPIC_TL_COMMAND_START_Pos)
#define EPIC_TL_COMMAND_START           EPIC_TL_COMMAND_START_Msk
#define EPIC_TL_COMMAND_RESET_Pos       (1U)
#define EPIC_TL_COMMAND_RESET_Msk       (0x1UL << EPIC_TL_COMMAND_RESET_Pos)
#define EPIC_TL_COMMAND_RESET           EPIC_TL_COMMAND_RESET_Msk

/***************** Bit definition for EPIC_TL_STATUS register *****************/
#define EPIC_TL_STATUS_TL_BUSY_Pos      (0U)
#define EPIC_TL_STATUS_TL_BUSY_Msk      (0x1UL << EPIC_TL_STATUS_TL_BUSY_Pos)
#define EPIC_TL_STATUS_TL_BUSY          EPIC_TL_STATUS_TL_BUSY_Msk
#define EPIC_TL_STATUS_TL_CTRL_BUSY_Pos  (1U)
#define EPIC_TL_STATUS_TL_CTRL_BUSY_Msk  (0x1UL << EPIC_TL_STATUS_TL_CTRL_BUSY_Pos)
#define EPIC_TL_STATUS_TL_CTRL_BUSY     EPIC_TL_STATUS_TL_CTRL_BUSY_Msk

/****************** Bit definition for EPIC_TL_IRQ register *******************/
#define EPIC_TL_IRQ_EOT_CAUSE_Pos       (0U)
#define EPIC_TL_IRQ_EOT_CAUSE_Msk       (0x1UL << EPIC_TL_IRQ_EOT_CAUSE_Pos)
#define EPIC_TL_IRQ_EOT_CAUSE           EPIC_TL_IRQ_EOT_CAUSE_Msk
#define EPIC_TL_IRQ_LINE_HIT_CAUSE_Pos  (1U)
#define EPIC_TL_IRQ_LINE_HIT_CAUSE_Msk  (0x1UL << EPIC_TL_IRQ_LINE_HIT_CAUSE_Pos)
#define EPIC_TL_IRQ_LINE_HIT_CAUSE      EPIC_TL_IRQ_LINE_HIT_CAUSE_Msk
#define EPIC_TL_IRQ_OCB_OF_CAUSE_Pos    (2U)
#define EPIC_TL_IRQ_OCB_OF_CAUSE_Msk    (0x1UL << EPIC_TL_IRQ_OCB_OF_CAUSE_Pos)
#define EPIC_TL_IRQ_OCB_OF_CAUSE        EPIC_TL_IRQ_OCB_OF_CAUSE_Msk
#define EPIC_TL_IRQ_EOT_STATUS_Pos      (16U)
#define EPIC_TL_IRQ_EOT_STATUS_Msk      (0x1UL << EPIC_TL_IRQ_EOT_STATUS_Pos)
#define EPIC_TL_IRQ_EOT_STATUS          EPIC_TL_IRQ_EOT_STATUS_Msk
#define EPIC_TL_IRQ_LINE_HIT_STATUS_Pos  (17U)
#define EPIC_TL_IRQ_LINE_HIT_STATUS_Msk  (0x1UL << EPIC_TL_IRQ_LINE_HIT_STATUS_Pos)
#define EPIC_TL_IRQ_LINE_HIT_STATUS     EPIC_TL_IRQ_LINE_HIT_STATUS_Msk
#define EPIC_TL_IRQ_OCB_OF_STATUS_Pos   (18U)
#define EPIC_TL_IRQ_OCB_OF_STATUS_Msk   (0x1UL << EPIC_TL_IRQ_OCB_OF_STATUS_Pos)
#define EPIC_TL_IRQ_OCB_OF_STATUS       EPIC_TL_IRQ_OCB_OF_STATUS_Msk

/**************** Bit definition for EPIC_TL_SETTING register *****************/
#define EPIC_TL_SETTING_EOT_IRQ_MASK_Pos  (0U)
#define EPIC_TL_SETTING_EOT_IRQ_MASK_Msk  (0x1UL << EPIC_TL_SETTING_EOT_IRQ_MASK_Pos)
#define EPIC_TL_SETTING_EOT_IRQ_MASK    EPIC_TL_SETTING_EOT_IRQ_MASK_Msk
#define EPIC_TL_SETTING_LINE_IRQ_MASK_Pos  (1U)
#define EPIC_TL_SETTING_LINE_IRQ_MASK_Msk  (0x1UL << EPIC_TL_SETTING_LINE_IRQ_MASK_Pos)
#define EPIC_TL_SETTING_LINE_IRQ_MASK   EPIC_TL_SETTING_LINE_IRQ_MASK_Msk
#define EPIC_TL_SETTING_OCB_OF_IRQ_MASK_Pos  (2U)
#define EPIC_TL_SETTING_OCB_OF_IRQ_MASK_Msk  (0x1UL << EPIC_TL_SETTING_OCB_OF_IRQ_MASK_Pos)
#define EPIC_TL_SETTING_OCB_OF_IRQ_MASK  EPIC_TL_SETTING_OCB_OF_IRQ_MASK_Msk
#define EPIC_TL_SETTING_LINE_IRQ_NUM_Pos  (16U)
#define EPIC_TL_SETTING_LINE_IRQ_NUM_Msk  (0x1FFFUL << EPIC_TL_SETTING_LINE_IRQ_NUM_Pos)
#define EPIC_TL_SETTING_LINE_IRQ_NUM    EPIC_TL_SETTING_LINE_IRQ_NUM_Msk

/**************** Bit definition for EPIC_TL_PERF_CNT register ****************/
#define EPIC_TL_PERF_CNT_VAL_Pos        (0U)
#define EPIC_TL_PERF_CNT_VAL_Msk        (0xFFFFFFFFUL << EPIC_TL_PERF_CNT_VAL_Pos)
#define EPIC_TL_PERF_CNT_VAL            EPIC_TL_PERF_CNT_VAL_Msk

/*************** Bit definition for EPIC_CANVAS_STAT0 register ****************/
#define EPIC_CANVAS_STAT0_FIFO_CNT_Pos  (0U)
#define EPIC_CANVAS_STAT0_FIFO_CNT_Msk  (0x7UL << EPIC_CANVAS_STAT0_FIFO_CNT_Pos)
#define EPIC_CANVAS_STAT0_FIFO_CNT      EPIC_CANVAS_STAT0_FIFO_CNT_Msk
#define EPIC_CANVAS_STAT0_POSTC_STAT_Pos  (3U)
#define EPIC_CANVAS_STAT0_POSTC_STAT_Msk  (0x7UL << EPIC_CANVAS_STAT0_POSTC_STAT_Pos)
#define EPIC_CANVAS_STAT0_POSTC_STAT    EPIC_CANVAS_STAT0_POSTC_STAT_Msk
#define EPIC_CANVAS_STAT0_PREC_STAT_Pos  (6U)
#define EPIC_CANVAS_STAT0_PREC_STAT_Msk  (0x7UL << EPIC_CANVAS_STAT0_PREC_STAT_Pos)
#define EPIC_CANVAS_STAT0_PREC_STAT     EPIC_CANVAS_STAT0_PREC_STAT_Msk
#define EPIC_CANVAS_STAT0_FETCH_STAT_Pos  (9U)
#define EPIC_CANVAS_STAT0_FETCH_STAT_Msk  (0x7UL << EPIC_CANVAS_STAT0_FETCH_STAT_Pos)
#define EPIC_CANVAS_STAT0_FETCH_STAT    EPIC_CANVAS_STAT0_FETCH_STAT_Msk

/*************** Bit definition for EPIC_CANVAS_STAT1 register ****************/
#define EPIC_CANVAS_STAT1_X_COR_Pos     (0U)
#define EPIC_CANVAS_STAT1_X_COR_Msk     (0x1FFFUL << EPIC_CANVAS_STAT1_X_COR_Pos)
#define EPIC_CANVAS_STAT1_X_COR         EPIC_CANVAS_STAT1_X_COR_Msk
#define EPIC_CANVAS_STAT1_Y_COR_Pos     (16U)
#define EPIC_CANVAS_STAT1_Y_COR_Msk     (0x1FFFUL << EPIC_CANVAS_STAT1_Y_COR_Pos)
#define EPIC_CANVAS_STAT1_Y_COR         EPIC_CANVAS_STAT1_Y_COR_Msk

/***************** Bit definition for EPIC_ENG_STAT register ******************/
#define EPIC_ENG_STAT_EZIP_LINE_CNT_Pos  (0U)
#define EPIC_ENG_STAT_EZIP_LINE_CNT_Msk  (0x1FFFUL << EPIC_ENG_STAT_EZIP_LINE_CNT_Pos)
#define EPIC_ENG_STAT_EZIP_LINE_CNT     EPIC_ENG_STAT_EZIP_LINE_CNT_Msk
#define EPIC_ENG_STAT_EZIP_RUN_STAT_Pos  (13U)
#define EPIC_ENG_STAT_EZIP_RUN_STAT_Msk  (0x7UL << EPIC_ENG_STAT_EZIP_RUN_STAT_Pos)
#define EPIC_ENG_STAT_EZIP_RUN_STAT     EPIC_ENG_STAT_EZIP_RUN_STAT_Msk
#define EPIC_ENG_STAT_JPEG_LINE_CNT_Pos  (16U)
#define EPIC_ENG_STAT_JPEG_LINE_CNT_Msk  (0x1FFFUL << EPIC_ENG_STAT_JPEG_LINE_CNT_Pos)
#define EPIC_ENG_STAT_JPEG_LINE_CNT     EPIC_ENG_STAT_JPEG_LINE_CNT_Msk
#define EPIC_ENG_STAT_JPEG_RUN_STAT_Pos  (29U)
#define EPIC_ENG_STAT_JPEG_RUN_STAT_Msk  (0x7UL << EPIC_ENG_STAT_JPEG_RUN_STAT_Pos)
#define EPIC_ENG_STAT_JPEG_RUN_STAT     EPIC_ENG_STAT_JPEG_RUN_STAT_Msk

/****************** Bit definition for EPIC_OL_STAT register ******************/
#define EPIC_OL_STAT_DONE_REQ0_Pos      (0U)
#define EPIC_OL_STAT_DONE_REQ0_Msk      (0x1UL << EPIC_OL_STAT_DONE_REQ0_Pos)
#define EPIC_OL_STAT_DONE_REQ0          EPIC_OL_STAT_DONE_REQ0_Msk
#define EPIC_OL_STAT_DATA_CONV0_Pos     (1U)
#define EPIC_OL_STAT_DATA_CONV0_Msk     (0x3UL << EPIC_OL_STAT_DATA_CONV0_Pos)
#define EPIC_OL_STAT_DATA_CONV0         EPIC_OL_STAT_DATA_CONV0_Msk
#define EPIC_OL_STAT_PF_DF0_Pos         (3U)
#define EPIC_OL_STAT_PF_DF0_Msk         (0x3UL << EPIC_OL_STAT_PF_DF0_Pos)
#define EPIC_OL_STAT_PF_DF0             EPIC_OL_STAT_PF_DF0_Msk
#define EPIC_OL_STAT_PF_PR0_Pos         (5U)
#define EPIC_OL_STAT_PF_PR0_Msk         (0x7UL << EPIC_OL_STAT_PF_PR0_Pos)
#define EPIC_OL_STAT_PF_PR0             EPIC_OL_STAT_PF_PR0_Msk
#define EPIC_OL_STAT_DONE_REQ1_Pos      (16U)
#define EPIC_OL_STAT_DONE_REQ1_Msk      (0x1UL << EPIC_OL_STAT_DONE_REQ1_Pos)
#define EPIC_OL_STAT_DONE_REQ1          EPIC_OL_STAT_DONE_REQ1_Msk
#define EPIC_OL_STAT_DATA_CONV1_Pos     (17U)
#define EPIC_OL_STAT_DATA_CONV1_Msk     (0x3UL << EPIC_OL_STAT_DATA_CONV1_Pos)
#define EPIC_OL_STAT_DATA_CONV1         EPIC_OL_STAT_DATA_CONV1_Msk
#define EPIC_OL_STAT_PF_DF1_Pos         (19U)
#define EPIC_OL_STAT_PF_DF1_Msk         (0x3UL << EPIC_OL_STAT_PF_DF1_Pos)
#define EPIC_OL_STAT_PF_DF1             EPIC_OL_STAT_PF_DF1_Msk
#define EPIC_OL_STAT_PF_PR1_Pos         (21U)
#define EPIC_OL_STAT_PF_PR1_Msk         (0x7UL << EPIC_OL_STAT_PF_PR1_Pos)
#define EPIC_OL_STAT_PF_PR1             EPIC_OL_STAT_PF_PR1_Msk

/***************** Bit definition for EPIC_OL2_STAT register ******************/
#define EPIC_OL2_STAT_DONE_REQ2_Pos     (0U)
#define EPIC_OL2_STAT_DONE_REQ2_Msk     (0x1UL << EPIC_OL2_STAT_DONE_REQ2_Pos)
#define EPIC_OL2_STAT_DONE_REQ2         EPIC_OL2_STAT_DONE_REQ2_Msk
#define EPIC_OL2_STAT_DATA_CONV2_Pos    (1U)
#define EPIC_OL2_STAT_DATA_CONV2_Msk    (0x3UL << EPIC_OL2_STAT_DATA_CONV2_Pos)
#define EPIC_OL2_STAT_DATA_CONV2        EPIC_OL2_STAT_DATA_CONV2_Msk
#define EPIC_OL2_STAT_PF_DF2_Pos        (3U)
#define EPIC_OL2_STAT_PF_DF2_Msk        (0x3UL << EPIC_OL2_STAT_PF_DF2_Pos)
#define EPIC_OL2_STAT_PF_DF2            EPIC_OL2_STAT_PF_DF2_Msk
#define EPIC_OL2_STAT_PF_PR2_Pos        (5U)
#define EPIC_OL2_STAT_PF_PR2_Msk        (0x7UL << EPIC_OL2_STAT_PF_PR2_Pos)
#define EPIC_OL2_STAT_PF_PR2            EPIC_OL2_STAT_PF_PR2_Msk

/****************** Bit definition for EPIC_VL_STAT register ******************/
#define EPIC_VL_STAT_RF_ROT_FE_Pos      (0U)
#define EPIC_VL_STAT_RF_ROT_FE_Msk      (0x7UL << EPIC_VL_STAT_RF_ROT_FE_Pos)
#define EPIC_VL_STAT_RF_ROT_FE          EPIC_VL_STAT_RF_ROT_FE_Msk
#define EPIC_VL_STAT_RF_ROT_BE_Pos      (3U)
#define EPIC_VL_STAT_RF_ROT_BE_Msk      (0xFUL << EPIC_VL_STAT_RF_ROT_BE_Pos)
#define EPIC_VL_STAT_RF_ROT_BE          EPIC_VL_STAT_RF_ROT_BE_Msk
#define EPIC_VL_STAT_NF_PR_Pos          (7U)
#define EPIC_VL_STAT_NF_PR_Msk          (0x7UL << EPIC_VL_STAT_NF_PR_Pos)
#define EPIC_VL_STAT_NF_PR              EPIC_VL_STAT_NF_PR_Msk
#define EPIC_VL_STAT_NF_DF_Pos          (10U)
#define EPIC_VL_STAT_NF_DF_Msk          (0x3UL << EPIC_VL_STAT_NF_DF_Pos)
#define EPIC_VL_STAT_NF_DF              EPIC_VL_STAT_NF_DF_Msk
#define EPIC_VL_STAT_NF_DATA_CONV_Pos   (12U)
#define EPIC_VL_STAT_NF_DATA_CONV_Msk   (0x3UL << EPIC_VL_STAT_NF_DATA_CONV_Pos)
#define EPIC_VL_STAT_NF_DATA_CONV       EPIC_VL_STAT_NF_DATA_CONV_Msk
#define EPIC_VL_STAT_SC_OUT_Pos         (14U)
#define EPIC_VL_STAT_SC_OUT_Msk         (0x3UL << EPIC_VL_STAT_SC_OUT_Pos)
#define EPIC_VL_STAT_SC_OUT             EPIC_VL_STAT_SC_OUT_Msk
#define EPIC_VL_STAT_SC_BE_Pos          (16U)
#define EPIC_VL_STAT_SC_BE_Msk          (0x7UL << EPIC_VL_STAT_SC_BE_Pos)
#define EPIC_VL_STAT_SC_BE              EPIC_VL_STAT_SC_BE_Msk
#define EPIC_VL_STAT_SC_FE_Pos          (19U)
#define EPIC_VL_STAT_SC_FE_Msk          (0xFUL << EPIC_VL_STAT_SC_FE_Pos)
#define EPIC_VL_STAT_SC_FE              EPIC_VL_STAT_SC_FE_Msk
#define EPIC_VL_STAT_SC_LB1_Pos         (23U)
#define EPIC_VL_STAT_SC_LB1_Msk         (0x3UL << EPIC_VL_STAT_SC_LB1_Pos)
#define EPIC_VL_STAT_SC_LB1             EPIC_VL_STAT_SC_LB1_Msk
#define EPIC_VL_STAT_SC_LB0_Pos         (25U)
#define EPIC_VL_STAT_SC_LB0_Msk         (0x3UL << EPIC_VL_STAT_SC_LB0_Pos)
#define EPIC_VL_STAT_SC_LB0             EPIC_VL_STAT_SC_LB0_Msk

/****************** Bit definition for EPIC_ML_STAT register ******************/
#define EPIC_ML_STAT_DONE_REQ_Pos       (0U)
#define EPIC_ML_STAT_DONE_REQ_Msk       (0x1UL << EPIC_ML_STAT_DONE_REQ_Pos)
#define EPIC_ML_STAT_DONE_REQ           EPIC_ML_STAT_DONE_REQ_Msk
#define EPIC_ML_STAT_MF_DF_Pos          (1U)
#define EPIC_ML_STAT_MF_DF_Msk          (0x3UL << EPIC_ML_STAT_MF_DF_Pos)
#define EPIC_ML_STAT_MF_DF              EPIC_ML_STAT_MF_DF_Msk
#define EPIC_ML_STAT_MF_PR_Pos          (3U)
#define EPIC_ML_STAT_MF_PR_Msk          (0x7UL << EPIC_ML_STAT_MF_PR_Pos)
#define EPIC_ML_STAT_MF_PR              EPIC_ML_STAT_MF_PR_Msk

/***************** Bit definition for EPIC_TL_STAT0 register ******************/
#define EPIC_TL_STAT0_X_COR_Pos         (0U)
#define EPIC_TL_STAT0_X_COR_Msk         (0x1FFFUL << EPIC_TL_STAT0_X_COR_Pos)
#define EPIC_TL_STAT0_X_COR             EPIC_TL_STAT0_X_COR_Msk
#define EPIC_TL_STAT0_Y_COR_Pos         (16U)
#define EPIC_TL_STAT0_Y_COR_Msk         (0x1FFFUL << EPIC_TL_STAT0_Y_COR_Pos)
#define EPIC_TL_STAT0_Y_COR             EPIC_TL_STAT0_Y_COR_Msk

/***************** Bit definition for EPIC_TL_STAT1 register ******************/
#define EPIC_TL_STAT1_DONE_REQ_Pos      (0U)
#define EPIC_TL_STAT1_DONE_REQ_Msk      (0x1UL << EPIC_TL_STAT1_DONE_REQ_Pos)
#define EPIC_TL_STAT1_DONE_REQ          EPIC_TL_STAT1_DONE_REQ_Msk
#define EPIC_TL_STAT1_DATA_CONV_Pos     (1U)
#define EPIC_TL_STAT1_DATA_CONV_Msk     (0x3UL << EPIC_TL_STAT1_DATA_CONV_Pos)
#define EPIC_TL_STAT1_DATA_CONV         EPIC_TL_STAT1_DATA_CONV_Msk
#define EPIC_TL_STAT1_PF_DF_Pos         (3U)
#define EPIC_TL_STAT1_PF_DF_Msk         (0x3UL << EPIC_TL_STAT1_PF_DF_Pos)
#define EPIC_TL_STAT1_PF_DF             EPIC_TL_STAT1_PF_DF_Msk
#define EPIC_TL_STAT1_PF_PR_Pos         (5U)
#define EPIC_TL_STAT1_PF_PR_Msk         (0x7UL << EPIC_TL_STAT1_PF_PR_Pos)
#define EPIC_TL_STAT1_PF_PR             EPIC_TL_STAT1_PF_PR_Msk
#define EPIC_TL_STAT1_AHB_CTRL_Pos      (16U)
#define EPIC_TL_STAT1_AHB_CTRL_Msk      (0x7UL << EPIC_TL_STAT1_AHB_CTRL_Pos)
#define EPIC_TL_STAT1_AHB_CTRL          EPIC_TL_STAT1_AHB_CTRL_Msk
#define EPIC_TL_STAT1_AHB_CTRL_EOL_Pos  (19U)
#define EPIC_TL_STAT1_AHB_CTRL_EOL_Msk  (0x1UL << EPIC_TL_STAT1_AHB_CTRL_EOL_Pos)
#define EPIC_TL_STAT1_AHB_CTRL_EOL      EPIC_TL_STAT1_AHB_CTRL_EOL_Msk
#define EPIC_TL_STAT1_AHB_CTRL_FIFO_CNT_Pos  (20U)
#define EPIC_TL_STAT1_AHB_CTRL_FIFO_CNT_Msk  (0xFFUL << EPIC_TL_STAT1_AHB_CTRL_FIFO_CNT_Pos)
#define EPIC_TL_STAT1_AHB_CTRL_FIFO_CNT  EPIC_TL_STAT1_AHB_CTRL_FIFO_CNT_Msk

/**************** Bit definition for EPIC_MEM_IF_STAT register ****************/
#define EPIC_MEM_IF_STAT_AHB0_Pos       (0U)
#define EPIC_MEM_IF_STAT_AHB0_Msk       (0xFUL << EPIC_MEM_IF_STAT_AHB0_Pos)
#define EPIC_MEM_IF_STAT_AHB0           EPIC_MEM_IF_STAT_AHB0_Msk
#define EPIC_MEM_IF_STAT_ARB_READ_PORT0_Pos  (4U)
#define EPIC_MEM_IF_STAT_ARB_READ_PORT0_Msk  (0x7UL << EPIC_MEM_IF_STAT_ARB_READ_PORT0_Pos)
#define EPIC_MEM_IF_STAT_ARB_READ_PORT0  EPIC_MEM_IF_STAT_ARB_READ_PORT0_Msk
#define EPIC_MEM_IF_STAT_ARB_MAIN0_Pos  (7U)
#define EPIC_MEM_IF_STAT_ARB_MAIN0_Msk  (0x7UL << EPIC_MEM_IF_STAT_ARB_MAIN0_Pos)
#define EPIC_MEM_IF_STAT_ARB_MAIN0      EPIC_MEM_IF_STAT_ARB_MAIN0_Msk
#define EPIC_MEM_IF_STAT_AHB1_Pos       (10U)
#define EPIC_MEM_IF_STAT_AHB1_Msk       (0xFUL << EPIC_MEM_IF_STAT_AHB1_Pos)
#define EPIC_MEM_IF_STAT_AHB1           EPIC_MEM_IF_STAT_AHB1_Msk
#define EPIC_MEM_IF_STAT_ARB_READ_PORT1_Pos  (14U)
#define EPIC_MEM_IF_STAT_ARB_READ_PORT1_Msk  (0x7UL << EPIC_MEM_IF_STAT_ARB_READ_PORT1_Pos)
#define EPIC_MEM_IF_STAT_ARB_READ_PORT1  EPIC_MEM_IF_STAT_ARB_READ_PORT1_Msk
#define EPIC_MEM_IF_STAT_ARB_MAIN1_Pos  (17U)
#define EPIC_MEM_IF_STAT_ARB_MAIN1_Msk  (0x7UL << EPIC_MEM_IF_STAT_ARB_MAIN1_Pos)
#define EPIC_MEM_IF_STAT_ARB_MAIN1      EPIC_MEM_IF_STAT_ARB_MAIN1_Msk
#define EPIC_MEM_IF_STAT_AHB_CTRL_Pos   (20U)
#define EPIC_MEM_IF_STAT_AHB_CTRL_Msk   (0x7UL << EPIC_MEM_IF_STAT_AHB_CTRL_Pos)
#define EPIC_MEM_IF_STAT_AHB_CTRL       EPIC_MEM_IF_STAT_AHB_CTRL_Msk
#define EPIC_MEM_IF_STAT_AHB_CTRL_EOL_Pos  (23U)
#define EPIC_MEM_IF_STAT_AHB_CTRL_EOL_Msk  (0x1UL << EPIC_MEM_IF_STAT_AHB_CTRL_EOL_Pos)
#define EPIC_MEM_IF_STAT_AHB_CTRL_EOL   EPIC_MEM_IF_STAT_AHB_CTRL_EOL_Msk
#define EPIC_MEM_IF_STAT_AHB_CTRL_FIFO_CNT_Pos  (24U)
#define EPIC_MEM_IF_STAT_AHB_CTRL_FIFO_CNT_Msk  (0xFFUL << EPIC_MEM_IF_STAT_AHB_CTRL_FIFO_CNT_Pos)
#define EPIC_MEM_IF_STAT_AHB_CTRL_FIFO_CNT  EPIC_MEM_IF_STAT_AHB_CTRL_FIFO_CNT_Msk

#define EPIC_MAX_X_SIZE  (EPIC_L0_TL_POS_X0_Msk >> EPIC_L0_TL_POS_X0_Pos)
#define EPIC_VL_SCALE_RATIO_XPITCH_MAX  (EPIC_VL_SCALE_RATIO_H_XPITCH_Msk >> EPIC_VL_SCALE_RATIO_H_XPITCH_Pos)
#define EPIC_VL_SCALE_RATIO_YPITCH_MAX  (EPIC_VL_SCALE_RATIO_V_YPITCH_Msk >> EPIC_VL_SCALE_RATIO_V_YPITCH_Pos)
#endif
