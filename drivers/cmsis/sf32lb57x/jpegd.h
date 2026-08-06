/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __JPEGD_H
#define __JPEGD_H

typedef struct
{
    __IO uint32_t JPEGD_EN;
    __IO uint32_t BUF_ADDR;
    __IO uint32_t SRC_ADDR;
    __IO uint32_t DST_ADDR0;
    __IO uint32_t DST_ADDR1;
    __IO uint32_t DST_ADDR2;
    __IO uint32_t JPEGD_PARA;
    __IO uint32_t SRC_LEN;
    __IO uint32_t CORE_CTRL;
    __IO uint32_t START_POINT;
    __IO uint32_t END_POINT;
    __IO uint32_t INT_EN;
    __IO uint32_t INT_STA;
    __IO uint32_t STATUS0;
    __IO uint32_t STATUS1;
    __IO uint32_t STATUS2;
    __IO uint32_t STATUS3;
    __IO uint32_t STATUS4;
    __IO uint32_t STATUS5;
    __IO uint32_t STATUS6;
    __IO uint32_t STATUS7;
    __IO uint32_t STATUS8;
    __IO uint32_t STATUS9;
    __IO uint32_t STATUS10;
    __IO uint32_t STATUS11;
    __IO uint32_t STATUS12;
    __IO uint32_t STATUS13;
    __IO uint32_t SOI_TIME;
    __IO uint32_t ACT_TIME;
    __IO uint32_t MJPEG_STA;
    __IO uint32_t MJPEG_FRM;
    __IO uint32_t MJPEG_START;
    __IO uint32_t DB_DATA0;
    __IO uint32_t DB_DATA1;
    __IO uint32_t DB_DATA2;
    __IO uint32_t DB_DATA3;
    __IO uint32_t DB_DATA4;
} JPEGD_TypeDef;


/***************** Bit definition for JPEGD_JPEGD_EN register *****************/
#define JPEGD_JPEGD_EN_JPEGD_EN_Pos     (0U)
#define JPEGD_JPEGD_EN_JPEGD_EN_Msk     (0x1UL << JPEGD_JPEGD_EN_JPEGD_EN_Pos)
#define JPEGD_JPEGD_EN_JPEGD_EN         JPEGD_JPEGD_EN_JPEGD_EN_Msk

/***************** Bit definition for JPEGD_BUF_ADDR register *****************/
#define JPEGD_BUF_ADDR_BUF_ADDR_Pos     (0U)
#define JPEGD_BUF_ADDR_BUF_ADDR_Msk     (0xFFFFFFFFUL << JPEGD_BUF_ADDR_BUF_ADDR_Pos)
#define JPEGD_BUF_ADDR_BUF_ADDR         JPEGD_BUF_ADDR_BUF_ADDR_Msk

/***************** Bit definition for JPEGD_SRC_ADDR register *****************/
#define JPEGD_SRC_ADDR_SRC_ADDR_Pos     (0U)
#define JPEGD_SRC_ADDR_SRC_ADDR_Msk     (0xFFFFFFFFUL << JPEGD_SRC_ADDR_SRC_ADDR_Pos)
#define JPEGD_SRC_ADDR_SRC_ADDR         JPEGD_SRC_ADDR_SRC_ADDR_Msk

/**************** Bit definition for JPEGD_DST_ADDR0 register *****************/
#define JPEGD_DST_ADDR0_DST_ADDR0_Pos   (0U)
#define JPEGD_DST_ADDR0_DST_ADDR0_Msk   (0xFFFFFFFFUL << JPEGD_DST_ADDR0_DST_ADDR0_Pos)
#define JPEGD_DST_ADDR0_DST_ADDR0       JPEGD_DST_ADDR0_DST_ADDR0_Msk

/**************** Bit definition for JPEGD_DST_ADDR1 register *****************/
#define JPEGD_DST_ADDR1_DST_ADDR1_Pos   (0U)
#define JPEGD_DST_ADDR1_DST_ADDR1_Msk   (0xFFFFFFFFUL << JPEGD_DST_ADDR1_DST_ADDR1_Pos)
#define JPEGD_DST_ADDR1_DST_ADDR1       JPEGD_DST_ADDR1_DST_ADDR1_Msk

/**************** Bit definition for JPEGD_DST_ADDR2 register *****************/
#define JPEGD_DST_ADDR2_DST_ADDR2_Pos   (0U)
#define JPEGD_DST_ADDR2_DST_ADDR2_Msk   (0xFFFFFFFFUL << JPEGD_DST_ADDR2_DST_ADDR2_Pos)
#define JPEGD_DST_ADDR2_DST_ADDR2       JPEGD_DST_ADDR2_DST_ADDR2_Msk

/**************** Bit definition for JPEGD_JPEGD_PARA register ****************/
#define JPEGD_JPEGD_PARA_OUT_SEL_Pos    (0U)
#define JPEGD_JPEGD_PARA_OUT_SEL_Msk    (0x1UL << JPEGD_JPEGD_PARA_OUT_SEL_Pos)
#define JPEGD_JPEGD_PARA_OUT_SEL        JPEGD_JPEGD_PARA_OUT_SEL_Msk
#define JPEGD_JPEGD_PARA_MJPEG_SEL_Pos  (4U)
#define JPEGD_JPEGD_PARA_MJPEG_SEL_Msk  (0x1UL << JPEGD_JPEGD_PARA_MJPEG_SEL_Pos)
#define JPEGD_JPEGD_PARA_MJPEG_SEL      JPEGD_JPEGD_PARA_MJPEG_SEL_Msk

/***************** Bit definition for JPEGD_SRC_LEN register ******************/
#define JPEGD_SRC_LEN_SRC_LEN_Pos       (0U)
#define JPEGD_SRC_LEN_SRC_LEN_Msk       (0xFFFFFFFFUL << JPEGD_SRC_LEN_SRC_LEN_Pos)
#define JPEGD_SRC_LEN_SRC_LEN           JPEGD_SRC_LEN_SRC_LEN_Msk

/**************** Bit definition for JPEGD_CORE_CTRL register *****************/
#define JPEGD_CORE_CTRL_CORE_CTRL_Pos   (0U)
#define JPEGD_CORE_CTRL_CORE_CTRL_Msk   (0x7UL << JPEGD_CORE_CTRL_CORE_CTRL_Pos)
#define JPEGD_CORE_CTRL_CORE_CTRL       JPEGD_CORE_CTRL_CORE_CTRL_Msk

/*************** Bit definition for JPEGD_START_POINT register ****************/
#define JPEGD_START_POINT_START_ROW_Pos  (0U)
#define JPEGD_START_POINT_START_ROW_Msk  (0xFFFFUL << JPEGD_START_POINT_START_ROW_Pos)
#define JPEGD_START_POINT_START_ROW     JPEGD_START_POINT_START_ROW_Msk
#define JPEGD_START_POINT_START_COL_Pos  (16U)
#define JPEGD_START_POINT_START_COL_Msk  (0xFFFFUL << JPEGD_START_POINT_START_COL_Pos)
#define JPEGD_START_POINT_START_COL     JPEGD_START_POINT_START_COL_Msk

/**************** Bit definition for JPEGD_END_POINT register *****************/
#define JPEGD_END_POINT_END_ROW_Pos     (0U)
#define JPEGD_END_POINT_END_ROW_Msk     (0xFFFFUL << JPEGD_END_POINT_END_ROW_Pos)
#define JPEGD_END_POINT_END_ROW         JPEGD_END_POINT_END_ROW_Msk
#define JPEGD_END_POINT_END_COL_Pos     (16U)
#define JPEGD_END_POINT_END_COL_Msk     (0xFFFFUL << JPEGD_END_POINT_END_COL_Pos)
#define JPEGD_END_POINT_END_COL         JPEGD_END_POINT_END_COL_Msk

/****************** Bit definition for JPEGD_INT_EN register ******************/
#define JPEGD_INT_EN_JPEGD_END_EN_Pos   (0U)
#define JPEGD_INT_EN_JPEGD_END_EN_Msk   (0x1UL << JPEGD_INT_EN_JPEGD_END_EN_Pos)
#define JPEGD_INT_EN_JPEGD_END_EN       JPEGD_INT_EN_JPEGD_END_EN_Msk
#define JPEGD_INT_EN_SOI_ERR_EN_Pos     (1U)
#define JPEGD_INT_EN_SOI_ERR_EN_Msk     (0x1UL << JPEGD_INT_EN_SOI_ERR_EN_Pos)
#define JPEGD_INT_EN_SOI_ERR_EN         JPEGD_INT_EN_SOI_ERR_EN_Msk
#define JPEGD_INT_EN_SCANACTIVE_ERR_EN_Pos  (2U)
#define JPEGD_INT_EN_SCANACTIVE_ERR_EN_Msk  (0x1UL << JPEGD_INT_EN_SCANACTIVE_ERR_EN_Pos)
#define JPEGD_INT_EN_SCANACTIVE_ERR_EN  JPEGD_INT_EN_SCANACTIVE_ERR_EN_Msk
#define JPEGD_INT_EN_TYPE_ERR_EN_Pos    (3U)
#define JPEGD_INT_EN_TYPE_ERR_EN_Msk    (0x1UL << JPEGD_INT_EN_TYPE_ERR_EN_Pos)
#define JPEGD_INT_EN_TYPE_ERR_EN        JPEGD_INT_EN_TYPE_ERR_EN_Msk
#define JPEGD_INT_EN_CONFG_ERR_EN_Pos   (4U)
#define JPEGD_INT_EN_CONFG_ERR_EN_Msk   (0x1UL << JPEGD_INT_EN_CONFG_ERR_EN_Pos)
#define JPEGD_INT_EN_CONFG_ERR_EN       JPEGD_INT_EN_CONFG_ERR_EN_Msk
#define JPEGD_INT_EN_SOB_ERR_EN_Pos     (5U)
#define JPEGD_INT_EN_SOB_ERR_EN_Msk     (0x1UL << JPEGD_INT_EN_SOB_ERR_EN_Pos)
#define JPEGD_INT_EN_SOB_ERR_EN         JPEGD_INT_EN_SOB_ERR_EN_Msk
#define JPEGD_INT_EN_EOB_ERR_EN_Pos     (6U)
#define JPEGD_INT_EN_EOB_ERR_EN_Msk     (0x1UL << JPEGD_INT_EN_EOB_ERR_EN_Pos)
#define JPEGD_INT_EN_EOB_ERR_EN         JPEGD_INT_EN_EOB_ERR_EN_Msk
#define JPEGD_INT_EN_LBS_ERR_EN_Pos     (7U)
#define JPEGD_INT_EN_LBS_ERR_EN_Msk     (0x1UL << JPEGD_INT_EN_LBS_ERR_EN_Pos)
#define JPEGD_INT_EN_LBS_ERR_EN         JPEGD_INT_EN_LBS_ERR_EN_Msk
#define JPEGD_INT_EN_START_ERR_EN_Pos   (8U)
#define JPEGD_INT_EN_START_ERR_EN_Msk   (0x1UL << JPEGD_INT_EN_START_ERR_EN_Pos)
#define JPEGD_INT_EN_START_ERR_EN       JPEGD_INT_EN_START_ERR_EN_Msk
#define JPEGD_INT_EN_MEM_ERR_EN_Pos     (9U)
#define JPEGD_INT_EN_MEM_ERR_EN_Msk     (0x1UL << JPEGD_INT_EN_MEM_ERR_EN_Pos)
#define JPEGD_INT_EN_MEM_ERR_EN         JPEGD_INT_EN_MEM_ERR_EN_Msk
#define JPEGD_INT_EN_MJPEG_END_EN_Pos   (10U)
#define JPEGD_INT_EN_MJPEG_END_EN_Msk   (0x1UL << JPEGD_INT_EN_MJPEG_END_EN_Pos)
#define JPEGD_INT_EN_MJPEG_END_EN       JPEGD_INT_EN_MJPEG_END_EN_Msk

/***************** Bit definition for JPEGD_INT_STA register ******************/
#define JPEGD_INT_STA_JPEGD_END_STA_Pos  (0U)
#define JPEGD_INT_STA_JPEGD_END_STA_Msk  (0x1UL << JPEGD_INT_STA_JPEGD_END_STA_Pos)
#define JPEGD_INT_STA_JPEGD_END_STA     JPEGD_INT_STA_JPEGD_END_STA_Msk
#define JPEGD_INT_STA_SOI_ERR_STA_Pos   (1U)
#define JPEGD_INT_STA_SOI_ERR_STA_Msk   (0x1UL << JPEGD_INT_STA_SOI_ERR_STA_Pos)
#define JPEGD_INT_STA_SOI_ERR_STA       JPEGD_INT_STA_SOI_ERR_STA_Msk
#define JPEGD_INT_STA_SCANACTIVE_ERR_STA_Pos  (2U)
#define JPEGD_INT_STA_SCANACTIVE_ERR_STA_Msk  (0x1UL << JPEGD_INT_STA_SCANACTIVE_ERR_STA_Pos)
#define JPEGD_INT_STA_SCANACTIVE_ERR_STA  JPEGD_INT_STA_SCANACTIVE_ERR_STA_Msk
#define JPEGD_INT_STA_TYPE_ERR_STA_Pos  (3U)
#define JPEGD_INT_STA_TYPE_ERR_STA_Msk  (0x1UL << JPEGD_INT_STA_TYPE_ERR_STA_Pos)
#define JPEGD_INT_STA_TYPE_ERR_STA      JPEGD_INT_STA_TYPE_ERR_STA_Msk
#define JPEGD_INT_STA_CONFG_ERR_STA_Pos  (4U)
#define JPEGD_INT_STA_CONFG_ERR_STA_Msk  (0x1UL << JPEGD_INT_STA_CONFG_ERR_STA_Pos)
#define JPEGD_INT_STA_CONFG_ERR_STA     JPEGD_INT_STA_CONFG_ERR_STA_Msk
#define JPEGD_INT_STA_SOB_ERR_STA_Pos   (5U)
#define JPEGD_INT_STA_SOB_ERR_STA_Msk   (0x1UL << JPEGD_INT_STA_SOB_ERR_STA_Pos)
#define JPEGD_INT_STA_SOB_ERR_STA       JPEGD_INT_STA_SOB_ERR_STA_Msk
#define JPEGD_INT_STA_EOB_ERR_STA_Pos   (6U)
#define JPEGD_INT_STA_EOB_ERR_STA_Msk   (0x1UL << JPEGD_INT_STA_EOB_ERR_STA_Pos)
#define JPEGD_INT_STA_EOB_ERR_STA       JPEGD_INT_STA_EOB_ERR_STA_Msk
#define JPEGD_INT_STA_LBS_ERR_STA_Pos   (7U)
#define JPEGD_INT_STA_LBS_ERR_STA_Msk   (0x1UL << JPEGD_INT_STA_LBS_ERR_STA_Pos)
#define JPEGD_INT_STA_LBS_ERR_STA       JPEGD_INT_STA_LBS_ERR_STA_Msk
#define JPEGD_INT_STA_START_ERR_STA_Pos  (8U)
#define JPEGD_INT_STA_START_ERR_STA_Msk  (0x1UL << JPEGD_INT_STA_START_ERR_STA_Pos)
#define JPEGD_INT_STA_START_ERR_STA     JPEGD_INT_STA_START_ERR_STA_Msk
#define JPEGD_INT_STA_MEM_ERR_STA_Pos   (9U)
#define JPEGD_INT_STA_MEM_ERR_STA_Msk   (0x1UL << JPEGD_INT_STA_MEM_ERR_STA_Pos)
#define JPEGD_INT_STA_MEM_ERR_STA       JPEGD_INT_STA_MEM_ERR_STA_Msk
#define JPEGD_INT_STA_MJPEG_END_STA_Pos  (10U)
#define JPEGD_INT_STA_MJPEG_END_STA_Msk  (0x1UL << JPEGD_INT_STA_MJPEG_END_STA_Pos)
#define JPEGD_INT_STA_MJPEG_END_STA     JPEGD_INT_STA_MJPEG_END_STA_Msk

/***************** Bit definition for JPEGD_STATUS0 register ******************/
#define JPEGD_STATUS0_STATUS0_Pos       (0U)
#define JPEGD_STATUS0_STATUS0_Msk       (0xFFFFUL << JPEGD_STATUS0_STATUS0_Pos)
#define JPEGD_STATUS0_STATUS0           JPEGD_STATUS0_STATUS0_Msk

/***************** Bit definition for JPEGD_STATUS1 register ******************/
#define JPEGD_STATUS1_STATUS1_Pos       (0U)
#define JPEGD_STATUS1_STATUS1_Msk       (0xFFFFUL << JPEGD_STATUS1_STATUS1_Pos)
#define JPEGD_STATUS1_STATUS1           JPEGD_STATUS1_STATUS1_Msk

/***************** Bit definition for JPEGD_STATUS2 register ******************/
#define JPEGD_STATUS2_STATUS2_Pos       (0U)
#define JPEGD_STATUS2_STATUS2_Msk       (0xFFFFUL << JPEGD_STATUS2_STATUS2_Pos)
#define JPEGD_STATUS2_STATUS2           JPEGD_STATUS2_STATUS2_Msk

/***************** Bit definition for JPEGD_STATUS3 register ******************/
#define JPEGD_STATUS3_STATUS3_Pos       (0U)
#define JPEGD_STATUS3_STATUS3_Msk       (0xFFFFUL << JPEGD_STATUS3_STATUS3_Pos)
#define JPEGD_STATUS3_STATUS3           JPEGD_STATUS3_STATUS3_Msk

/***************** Bit definition for JPEGD_STATUS4 register ******************/
#define JPEGD_STATUS4_STATUS4_Pos       (0U)
#define JPEGD_STATUS4_STATUS4_Msk       (0xFFFFUL << JPEGD_STATUS4_STATUS4_Pos)
#define JPEGD_STATUS4_STATUS4           JPEGD_STATUS4_STATUS4_Msk

/***************** Bit definition for JPEGD_STATUS5 register ******************/
#define JPEGD_STATUS5_STATUS5_Pos       (0U)
#define JPEGD_STATUS5_STATUS5_Msk       (0xFFFFUL << JPEGD_STATUS5_STATUS5_Pos)
#define JPEGD_STATUS5_STATUS5           JPEGD_STATUS5_STATUS5_Msk

/***************** Bit definition for JPEGD_STATUS6 register ******************/
#define JPEGD_STATUS6_STATUS6_Pos       (0U)
#define JPEGD_STATUS6_STATUS6_Msk       (0xFFFFUL << JPEGD_STATUS6_STATUS6_Pos)
#define JPEGD_STATUS6_STATUS6           JPEGD_STATUS6_STATUS6_Msk

/***************** Bit definition for JPEGD_STATUS7 register ******************/
#define JPEGD_STATUS7_STATUS7_Pos       (0U)
#define JPEGD_STATUS7_STATUS7_Msk       (0xFFFFUL << JPEGD_STATUS7_STATUS7_Pos)
#define JPEGD_STATUS7_STATUS7           JPEGD_STATUS7_STATUS7_Msk

/***************** Bit definition for JPEGD_STATUS8 register ******************/
#define JPEGD_STATUS8_STATUS8_Pos       (0U)
#define JPEGD_STATUS8_STATUS8_Msk       (0xFFFFUL << JPEGD_STATUS8_STATUS8_Pos)
#define JPEGD_STATUS8_STATUS8           JPEGD_STATUS8_STATUS8_Msk

/***************** Bit definition for JPEGD_STATUS9 register ******************/
#define JPEGD_STATUS9_STATUS9_Pos       (0U)
#define JPEGD_STATUS9_STATUS9_Msk       (0xFFFFUL << JPEGD_STATUS9_STATUS9_Pos)
#define JPEGD_STATUS9_STATUS9           JPEGD_STATUS9_STATUS9_Msk

/***************** Bit definition for JPEGD_STATUS10 register *****************/
#define JPEGD_STATUS10_STATUS10_Pos     (0U)
#define JPEGD_STATUS10_STATUS10_Msk     (0xFFFFUL << JPEGD_STATUS10_STATUS10_Pos)
#define JPEGD_STATUS10_STATUS10         JPEGD_STATUS10_STATUS10_Msk

/***************** Bit definition for JPEGD_STATUS11 register *****************/
#define JPEGD_STATUS11_STATUS11_Pos     (0U)
#define JPEGD_STATUS11_STATUS11_Msk     (0xFFFFUL << JPEGD_STATUS11_STATUS11_Pos)
#define JPEGD_STATUS11_STATUS11         JPEGD_STATUS11_STATUS11_Msk

/***************** Bit definition for JPEGD_STATUS12 register *****************/
#define JPEGD_STATUS12_STATUS12_Pos     (0U)
#define JPEGD_STATUS12_STATUS12_Msk     (0xFFFFUL << JPEGD_STATUS12_STATUS12_Pos)
#define JPEGD_STATUS12_STATUS12         JPEGD_STATUS12_STATUS12_Msk

/***************** Bit definition for JPEGD_STATUS13 register *****************/
#define JPEGD_STATUS13_STATUS13_Pos     (0U)
#define JPEGD_STATUS13_STATUS13_Msk     (0xFFFFUL << JPEGD_STATUS13_STATUS13_Pos)
#define JPEGD_STATUS13_STATUS13         JPEGD_STATUS13_STATUS13_Msk

/***************** Bit definition for JPEGD_SOI_TIME register *****************/
#define JPEGD_SOI_TIME_SOI_TIME_Pos     (0U)
#define JPEGD_SOI_TIME_SOI_TIME_Msk     (0xFFFFUL << JPEGD_SOI_TIME_SOI_TIME_Pos)
#define JPEGD_SOI_TIME_SOI_TIME         JPEGD_SOI_TIME_SOI_TIME_Msk

/***************** Bit definition for JPEGD_ACT_TIME register *****************/
#define JPEGD_ACT_TIME_ACT_TIME_Pos     (0U)
#define JPEGD_ACT_TIME_ACT_TIME_Msk     (0xFFFFFFFFUL << JPEGD_ACT_TIME_ACT_TIME_Pos)
#define JPEGD_ACT_TIME_ACT_TIME         JPEGD_ACT_TIME_ACT_TIME_Msk

/**************** Bit definition for JPEGD_MJPEG_STA register *****************/
#define JPEGD_MJPEG_STA_MJPEG_STA_Pos   (0U)
#define JPEGD_MJPEG_STA_MJPEG_STA_Msk   (0x1UL << JPEGD_MJPEG_STA_MJPEG_STA_Pos)
#define JPEGD_MJPEG_STA_MJPEG_STA       JPEGD_MJPEG_STA_MJPEG_STA_Msk

/**************** Bit definition for JPEGD_MJPEG_FRM register *****************/
#define JPEGD_MJPEG_FRM_FRM_CNT_Pos     (0U)
#define JPEGD_MJPEG_FRM_FRM_CNT_Msk     (0xFFFFUL << JPEGD_MJPEG_FRM_FRM_CNT_Pos)
#define JPEGD_MJPEG_FRM_FRM_CNT         JPEGD_MJPEG_FRM_FRM_CNT_Msk
#define JPEGD_MJPEG_FRM_FRM_NUM_Pos     (16U)
#define JPEGD_MJPEG_FRM_FRM_NUM_Msk     (0xFFFFUL << JPEGD_MJPEG_FRM_FRM_NUM_Pos)
#define JPEGD_MJPEG_FRM_FRM_NUM         JPEGD_MJPEG_FRM_FRM_NUM_Msk

/*************** Bit definition for JPEGD_MJPEG_START register ****************/
#define JPEGD_MJPEG_START_START_OFFSET_Pos  (0U)
#define JPEGD_MJPEG_START_START_OFFSET_Msk  (0xFFFFFFFFUL << JPEGD_MJPEG_START_START_OFFSET_Pos)
#define JPEGD_MJPEG_START_START_OFFSET  JPEGD_MJPEG_START_START_OFFSET_Msk

/***************** Bit definition for JPEGD_DB_DATA0 register *****************/
#define JPEGD_DB_DATA0_DB_DATA0_Pos     (0U)
#define JPEGD_DB_DATA0_DB_DATA0_Msk     (0xFFFFFFFFUL << JPEGD_DB_DATA0_DB_DATA0_Pos)
#define JPEGD_DB_DATA0_DB_DATA0         JPEGD_DB_DATA0_DB_DATA0_Msk

/***************** Bit definition for JPEGD_DB_DATA1 register *****************/
#define JPEGD_DB_DATA1_DB_DATA1_Pos     (0U)
#define JPEGD_DB_DATA1_DB_DATA1_Msk     (0xFFFFFFFFUL << JPEGD_DB_DATA1_DB_DATA1_Pos)
#define JPEGD_DB_DATA1_DB_DATA1         JPEGD_DB_DATA1_DB_DATA1_Msk

/***************** Bit definition for JPEGD_DB_DATA2 register *****************/
#define JPEGD_DB_DATA2_DB_DATA2_Pos     (0U)
#define JPEGD_DB_DATA2_DB_DATA2_Msk     (0xFFFFFFFFUL << JPEGD_DB_DATA2_DB_DATA2_Pos)
#define JPEGD_DB_DATA2_DB_DATA2         JPEGD_DB_DATA2_DB_DATA2_Msk

/***************** Bit definition for JPEGD_DB_DATA3 register *****************/
#define JPEGD_DB_DATA3_DB_DATA3_Pos     (0U)
#define JPEGD_DB_DATA3_DB_DATA3_Msk     (0xFFFFFFFFUL << JPEGD_DB_DATA3_DB_DATA3_Pos)
#define JPEGD_DB_DATA3_DB_DATA3         JPEGD_DB_DATA3_DB_DATA3_Msk

/***************** Bit definition for JPEGD_DB_DATA4 register *****************/
#define JPEGD_DB_DATA4_DB_DATA4_Pos     (0U)
#define JPEGD_DB_DATA4_DB_DATA4_Msk     (0xFFFFFFFFUL << JPEGD_DB_DATA4_DB_DATA4_Pos)
#define JPEGD_DB_DATA4_DB_DATA4         JPEGD_DB_DATA4_DB_DATA4_Msk

#endif
