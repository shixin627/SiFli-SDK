/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DCMI_H
#define __DCMI_H

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t SR;
    __IO uint32_t RIS;
    __IO uint32_t IE;
    __IO uint32_t MIS;
    __IO uint32_t ISC;
    __IO uint32_t ESCR;
    __IO uint32_t ESUR;
    __IO uint32_t CWSTRT;
    __IO uint32_t CWSIZE;
    __IO uint32_t RSVD2[4];
    __IO uint32_t DR;
    __IO uint32_t RSVD1[16];
    __IO uint32_t SPI_CR;
    __IO uint32_t SPI_SYNC;
    __IO uint32_t SPI_PKT_ID;
    __IO uint32_t BP;
    __IO uint32_t SPI_SR;
    __IO uint32_t DMA_CR;
    __IO uint32_t DBG_CR;
    __IO uint32_t YUV2RGB_CR1;
    __IO uint32_t YUV2RGB_CR2;
    __IO uint32_t JPEG_BYTE;
    __IO uint32_t IPI_CR;
    __IO uint32_t CONV_CR;
    __IO uint32_t FIFO_SR;
} DCMI_TypeDef;


/******************** Bit definition for DCMI_CR register *********************/
#define DCMI_CR_CAPTURE_Pos             (0U)
#define DCMI_CR_CAPTURE_Msk             (0x1UL << DCMI_CR_CAPTURE_Pos)
#define DCMI_CR_CAPTURE                 DCMI_CR_CAPTURE_Msk
#define DCMI_CR_CM_Pos                  (1U)
#define DCMI_CR_CM_Msk                  (0x1UL << DCMI_CR_CM_Pos)
#define DCMI_CR_CM                      DCMI_CR_CM_Msk
#define DCMI_CR_CROP_Pos                (2U)
#define DCMI_CR_CROP_Msk                (0x1UL << DCMI_CR_CROP_Pos)
#define DCMI_CR_CROP                    DCMI_CR_CROP_Msk
#define DCMI_CR_JPEG_Pos                (3U)
#define DCMI_CR_JPEG_Msk                (0x1UL << DCMI_CR_JPEG_Pos)
#define DCMI_CR_JPEG                    DCMI_CR_JPEG_Msk
#define DCMI_CR_ESS_Pos                 (4U)
#define DCMI_CR_ESS_Msk                 (0x1UL << DCMI_CR_ESS_Pos)
#define DCMI_CR_ESS                     DCMI_CR_ESS_Msk
#define DCMI_CR_PCKPOL_Pos              (5U)
#define DCMI_CR_PCKPOL_Msk              (0x1UL << DCMI_CR_PCKPOL_Pos)
#define DCMI_CR_PCKPOL                  DCMI_CR_PCKPOL_Msk
#define DCMI_CR_HSPOL_Pos               (6U)
#define DCMI_CR_HSPOL_Msk               (0x1UL << DCMI_CR_HSPOL_Pos)
#define DCMI_CR_HSPOL                   DCMI_CR_HSPOL_Msk
#define DCMI_CR_VSPOL_Pos               (7U)
#define DCMI_CR_VSPOL_Msk               (0x1UL << DCMI_CR_VSPOL_Pos)
#define DCMI_CR_VSPOL                   DCMI_CR_VSPOL_Msk
#define DCMI_CR_BIT_SWAP_Pos            (8U)
#define DCMI_CR_BIT_SWAP_Msk            (0x1UL << DCMI_CR_BIT_SWAP_Pos)
#define DCMI_CR_BIT_SWAP                DCMI_CR_BIT_SWAP_Msk
#define DCMI_CR_HALF_WORD_SWAP_Pos      (9U)
#define DCMI_CR_HALF_WORD_SWAP_Msk      (0x1UL << DCMI_CR_HALF_WORD_SWAP_Pos)
#define DCMI_CR_HALF_WORD_SWAP          DCMI_CR_HALF_WORD_SWAP_Msk
#define DCMI_CR_EDM_Pos                 (10U)
#define DCMI_CR_EDM_Msk                 (0x7UL << DCMI_CR_EDM_Pos)
#define DCMI_CR_EDM                     DCMI_CR_EDM_Msk
#define DCMI_CR_ENABLE_Pos              (14U)
#define DCMI_CR_ENABLE_Msk              (0x1UL << DCMI_CR_ENABLE_Pos)
#define DCMI_CR_ENABLE                  DCMI_CR_ENABLE_Msk
#define DCMI_CR_ESS_SWAP_Pos            (15U)
#define DCMI_CR_ESS_SWAP_Msk            (0x1UL << DCMI_CR_ESS_SWAP_Pos)
#define DCMI_CR_ESS_SWAP                DCMI_CR_ESS_SWAP_Msk
#define DCMI_CR_FCLKDIV_Pos             (16U)
#define DCMI_CR_FCLKDIV_Msk             (0x1FUL << DCMI_CR_FCLKDIV_Pos)
#define DCMI_CR_FCLKDIV                 DCMI_CR_FCLKDIV_Msk
#define DCMI_CR_FCLKEN_Pos              (21U)
#define DCMI_CR_FCLKEN_Msk              (0x1UL << DCMI_CR_FCLKEN_Pos)
#define DCMI_CR_FCLKEN                  DCMI_CR_FCLKEN_Msk
#define DCMI_CR_RSTB_Pos                (22U)
#define DCMI_CR_RSTB_Msk                (0x1UL << DCMI_CR_RSTB_Pos)
#define DCMI_CR_RSTB                    DCMI_CR_RSTB_Msk
#define DCMI_CR_RSTB_S_Pos              (23U)
#define DCMI_CR_RSTB_S_Msk              (0x1UL << DCMI_CR_RSTB_S_Pos)
#define DCMI_CR_RSTB_S                  DCMI_CR_RSTB_S_Msk
#define DCMI_CR_IF_SEL_Pos              (24U)
#define DCMI_CR_IF_SEL_Msk              (0x1UL << DCMI_CR_IF_SEL_Pos)
#define DCMI_CR_IF_SEL                  DCMI_CR_IF_SEL_Msk
#define DCMI_CR_IPI_EN_Pos              (25U)
#define DCMI_CR_IPI_EN_Msk              (0x1UL << DCMI_CR_IPI_EN_Pos)
#define DCMI_CR_IPI_EN                  DCMI_CR_IPI_EN_Msk

/******************** Bit definition for DCMI_SR register *********************/
#define DCMI_SR_HSYNC_Pos               (0U)
#define DCMI_SR_HSYNC_Msk               (0x1UL << DCMI_SR_HSYNC_Pos)
#define DCMI_SR_HSYNC                   DCMI_SR_HSYNC_Msk
#define DCMI_SR_VSYNC_Pos               (1U)
#define DCMI_SR_VSYNC_Msk               (0x1UL << DCMI_SR_VSYNC_Pos)
#define DCMI_SR_VSYNC                   DCMI_SR_VSYNC_Msk
#define DCMI_SR_FNE_Pos                 (2U)
#define DCMI_SR_FNE_Msk                 (0x1UL << DCMI_SR_FNE_Pos)
#define DCMI_SR_FNE                     DCMI_SR_FNE_Msk

/******************** Bit definition for DCMI_RIS register ********************/
#define DCMI_RIS_FRAME_RIS_Pos          (0U)
#define DCMI_RIS_FRAME_RIS_Msk          (0x1UL << DCMI_RIS_FRAME_RIS_Pos)
#define DCMI_RIS_FRAME_RIS              DCMI_RIS_FRAME_RIS_Msk
#define DCMI_RIS_OVR_RIS_Pos            (1U)
#define DCMI_RIS_OVR_RIS_Msk            (0x1UL << DCMI_RIS_OVR_RIS_Pos)
#define DCMI_RIS_OVR_RIS                DCMI_RIS_OVR_RIS_Msk
#define DCMI_RIS_FSM_ERR_RIS_Pos        (2U)
#define DCMI_RIS_FSM_ERR_RIS_Msk        (0x1UL << DCMI_RIS_FSM_ERR_RIS_Pos)
#define DCMI_RIS_FSM_ERR_RIS            DCMI_RIS_FSM_ERR_RIS_Msk
#define DCMI_RIS_VSYNC_RIS_Pos          (3U)
#define DCMI_RIS_VSYNC_RIS_Msk          (0x1UL << DCMI_RIS_VSYNC_RIS_Pos)
#define DCMI_RIS_VSYNC_RIS              DCMI_RIS_VSYNC_RIS_Msk
#define DCMI_RIS_LINE_RIS_Pos           (4U)
#define DCMI_RIS_LINE_RIS_Msk           (0x1UL << DCMI_RIS_LINE_RIS_Pos)
#define DCMI_RIS_LINE_RIS               DCMI_RIS_LINE_RIS_Msk
#define DCMI_RIS_PACK_SIZE_ERR_RIS_Pos  (5U)
#define DCMI_RIS_PACK_SIZE_ERR_RIS_Msk  (0x1UL << DCMI_RIS_PACK_SIZE_ERR_RIS_Pos)
#define DCMI_RIS_PACK_SIZE_ERR_RIS      DCMI_RIS_PACK_SIZE_ERR_RIS_Msk

/******************** Bit definition for DCMI_IE register *********************/
#define DCMI_IE_FRAME_IE_Pos            (0U)
#define DCMI_IE_FRAME_IE_Msk            (0x1UL << DCMI_IE_FRAME_IE_Pos)
#define DCMI_IE_FRAME_IE                DCMI_IE_FRAME_IE_Msk
#define DCMI_IE_OVR_IE_Pos              (1U)
#define DCMI_IE_OVR_IE_Msk              (0x1UL << DCMI_IE_OVR_IE_Pos)
#define DCMI_IE_OVR_IE                  DCMI_IE_OVR_IE_Msk
#define DCMI_IE_FSM_ERR_IE_Pos          (2U)
#define DCMI_IE_FSM_ERR_IE_Msk          (0x1UL << DCMI_IE_FSM_ERR_IE_Pos)
#define DCMI_IE_FSM_ERR_IE              DCMI_IE_FSM_ERR_IE_Msk
#define DCMI_IE_VSYNC_IE_Pos            (3U)
#define DCMI_IE_VSYNC_IE_Msk            (0x1UL << DCMI_IE_VSYNC_IE_Pos)
#define DCMI_IE_VSYNC_IE                DCMI_IE_VSYNC_IE_Msk
#define DCMI_IE_LINE_IE_Pos             (4U)
#define DCMI_IE_LINE_IE_Msk             (0x1UL << DCMI_IE_LINE_IE_Pos)
#define DCMI_IE_LINE_IE                 DCMI_IE_LINE_IE_Msk
#define DCMI_IE_PACK_SIZE_ERR_IE_Pos    (5U)
#define DCMI_IE_PACK_SIZE_ERR_IE_Msk    (0x1UL << DCMI_IE_PACK_SIZE_ERR_IE_Pos)
#define DCMI_IE_PACK_SIZE_ERR_IE        DCMI_IE_PACK_SIZE_ERR_IE_Msk

/******************** Bit definition for DCMI_MIS register ********************/
#define DCMI_MIS_FRAME_MIS_Pos          (0U)
#define DCMI_MIS_FRAME_MIS_Msk          (0x1UL << DCMI_MIS_FRAME_MIS_Pos)
#define DCMI_MIS_FRAME_MIS              DCMI_MIS_FRAME_MIS_Msk
#define DCMI_MIS_OVR_MIS_Pos            (1U)
#define DCMI_MIS_OVR_MIS_Msk            (0x1UL << DCMI_MIS_OVR_MIS_Pos)
#define DCMI_MIS_OVR_MIS                DCMI_MIS_OVR_MIS_Msk
#define DCMI_MIS_FSM_ERR_MIS_Pos        (2U)
#define DCMI_MIS_FSM_ERR_MIS_Msk        (0x1UL << DCMI_MIS_FSM_ERR_MIS_Pos)
#define DCMI_MIS_FSM_ERR_MIS            DCMI_MIS_FSM_ERR_MIS_Msk
#define DCMI_MIS_VSYNC_MIS_Pos          (3U)
#define DCMI_MIS_VSYNC_MIS_Msk          (0x1UL << DCMI_MIS_VSYNC_MIS_Pos)
#define DCMI_MIS_VSYNC_MIS              DCMI_MIS_VSYNC_MIS_Msk
#define DCMI_MIS_LINE_MIS_Pos           (4U)
#define DCMI_MIS_LINE_MIS_Msk           (0x1UL << DCMI_MIS_LINE_MIS_Pos)
#define DCMI_MIS_LINE_MIS               DCMI_MIS_LINE_MIS_Msk
#define DCMI_MIS_PACK_SIZE_ERR_MIS_Pos  (5U)
#define DCMI_MIS_PACK_SIZE_ERR_MIS_Msk  (0x1UL << DCMI_MIS_PACK_SIZE_ERR_MIS_Pos)
#define DCMI_MIS_PACK_SIZE_ERR_MIS      DCMI_MIS_PACK_SIZE_ERR_MIS_Msk

/******************** Bit definition for DCMI_ISC register ********************/
#define DCMI_ISC_FRAME_ISC_Pos          (0U)
#define DCMI_ISC_FRAME_ISC_Msk          (0x1UL << DCMI_ISC_FRAME_ISC_Pos)
#define DCMI_ISC_FRAME_ISC              DCMI_ISC_FRAME_ISC_Msk
#define DCMI_ISC_OVR_ISC_Pos            (1U)
#define DCMI_ISC_OVR_ISC_Msk            (0x1UL << DCMI_ISC_OVR_ISC_Pos)
#define DCMI_ISC_OVR_ISC                DCMI_ISC_OVR_ISC_Msk
#define DCMI_ISC_FSM_ERR_ISC_Pos        (2U)
#define DCMI_ISC_FSM_ERR_ISC_Msk        (0x1UL << DCMI_ISC_FSM_ERR_ISC_Pos)
#define DCMI_ISC_FSM_ERR_ISC            DCMI_ISC_FSM_ERR_ISC_Msk
#define DCMI_ISC_VSYNC_ISC_Pos          (3U)
#define DCMI_ISC_VSYNC_ISC_Msk          (0x1UL << DCMI_ISC_VSYNC_ISC_Pos)
#define DCMI_ISC_VSYNC_ISC              DCMI_ISC_VSYNC_ISC_Msk
#define DCMI_ISC_LINE_ISC_Pos           (4U)
#define DCMI_ISC_LINE_ISC_Msk           (0x1UL << DCMI_ISC_LINE_ISC_Pos)
#define DCMI_ISC_LINE_ISC               DCMI_ISC_LINE_ISC_Msk
#define DCMI_ISC_PACK_SIZE_ERR_ISC_Pos  (5U)
#define DCMI_ISC_PACK_SIZE_ERR_ISC_Msk  (0x1UL << DCMI_ISC_PACK_SIZE_ERR_ISC_Pos)
#define DCMI_ISC_PACK_SIZE_ERR_ISC      DCMI_ISC_PACK_SIZE_ERR_ISC_Msk

/******************* Bit definition for DCMI_ESCR register ********************/
#define DCMI_ESCR_FSC_Pos               (0U)
#define DCMI_ESCR_FSC_Msk               (0xFFUL << DCMI_ESCR_FSC_Pos)
#define DCMI_ESCR_FSC                   DCMI_ESCR_FSC_Msk
#define DCMI_ESCR_LSC_Pos               (8U)
#define DCMI_ESCR_LSC_Msk               (0xFFUL << DCMI_ESCR_LSC_Pos)
#define DCMI_ESCR_LSC                   DCMI_ESCR_LSC_Msk
#define DCMI_ESCR_LEC_Pos               (16U)
#define DCMI_ESCR_LEC_Msk               (0xFFUL << DCMI_ESCR_LEC_Pos)
#define DCMI_ESCR_LEC                   DCMI_ESCR_LEC_Msk
#define DCMI_ESCR_FEC_Pos               (24U)
#define DCMI_ESCR_FEC_Msk               (0xFFUL << DCMI_ESCR_FEC_Pos)
#define DCMI_ESCR_FEC                   DCMI_ESCR_FEC_Msk

/******************* Bit definition for DCMI_ESUR register ********************/
#define DCMI_ESUR_FSU_Pos               (0U)
#define DCMI_ESUR_FSU_Msk               (0xFFUL << DCMI_ESUR_FSU_Pos)
#define DCMI_ESUR_FSU                   DCMI_ESUR_FSU_Msk
#define DCMI_ESUR_LSU_Pos               (8U)
#define DCMI_ESUR_LSU_Msk               (0xFFUL << DCMI_ESUR_LSU_Pos)
#define DCMI_ESUR_LSU                   DCMI_ESUR_LSU_Msk
#define DCMI_ESUR_LEU_Pos               (16U)
#define DCMI_ESUR_LEU_Msk               (0xFFUL << DCMI_ESUR_LEU_Pos)
#define DCMI_ESUR_LEU                   DCMI_ESUR_LEU_Msk
#define DCMI_ESUR_FEU_Pos               (24U)
#define DCMI_ESUR_FEU_Msk               (0xFFUL << DCMI_ESUR_FEU_Pos)
#define DCMI_ESUR_FEU                   DCMI_ESUR_FEU_Msk

/****************** Bit definition for DCMI_CWSTRT register *******************/
#define DCMI_CWSTRT_HOFFCNT_Pos         (0U)
#define DCMI_CWSTRT_HOFFCNT_Msk         (0x3FFFUL << DCMI_CWSTRT_HOFFCNT_Pos)
#define DCMI_CWSTRT_HOFFCNT             DCMI_CWSTRT_HOFFCNT_Msk
#define DCMI_CWSTRT_VST_Pos             (16U)
#define DCMI_CWSTRT_VST_Msk             (0x1FFFUL << DCMI_CWSTRT_VST_Pos)
#define DCMI_CWSTRT_VST                 DCMI_CWSTRT_VST_Msk

/****************** Bit definition for DCMI_CWSIZE register *******************/
#define DCMI_CWSIZE_CAPCNT_Pos          (0U)
#define DCMI_CWSIZE_CAPCNT_Msk          (0x3FFFUL << DCMI_CWSIZE_CAPCNT_Pos)
#define DCMI_CWSIZE_CAPCNT              DCMI_CWSIZE_CAPCNT_Msk
#define DCMI_CWSIZE_VLINE_Pos           (16U)
#define DCMI_CWSIZE_VLINE_Msk           (0x3FFFUL << DCMI_CWSIZE_VLINE_Pos)
#define DCMI_CWSIZE_VLINE               DCMI_CWSIZE_VLINE_Msk

/******************** Bit definition for DCMI_DR register *********************/
#define DCMI_DR_DATA_Pos                (0U)
#define DCMI_DR_DATA_Msk                (0xFFFFFFFFUL << DCMI_DR_DATA_Pos)
#define DCMI_DR_DATA                    DCMI_DR_DATA_Msk

/****************** Bit definition for DCMI_SPI_CR register *******************/
#define DCMI_SPI_CR_MTK_MODE_Pos        (0U)
#define DCMI_SPI_CR_MTK_MODE_Msk        (0x1UL << DCMI_SPI_CR_MTK_MODE_Pos)
#define DCMI_SPI_CR_MTK_MODE            DCMI_SPI_CR_MTK_MODE_Msk
#define DCMI_SPI_CR_DI_SEL_Pos          (1U)
#define DCMI_SPI_CR_DI_SEL_Msk          (0x3UL << DCMI_SPI_CR_DI_SEL_Pos)
#define DCMI_SPI_CR_DI_SEL              DCMI_SPI_CR_DI_SEL_Msk
#define DCMI_SPI_CR_DI_SWAP_Pos         (3U)
#define DCMI_SPI_CR_DI_SWAP_Msk         (0x1UL << DCMI_SPI_CR_DI_SWAP_Pos)
#define DCMI_SPI_CR_DI_SWAP             DCMI_SPI_CR_DI_SWAP_Msk
#define DCMI_SPI_CR_HEADER_MSB_FIRST_Pos  (4U)
#define DCMI_SPI_CR_HEADER_MSB_FIRST_Msk  (0x1UL << DCMI_SPI_CR_HEADER_MSB_FIRST_Pos)
#define DCMI_SPI_CR_HEADER_MSB_FIRST    DCMI_SPI_CR_HEADER_MSB_FIRST_Msk
#define DCMI_SPI_CR_DATA_MSB_FIRST_Pos  (5U)
#define DCMI_SPI_CR_DATA_MSB_FIRST_Msk  (0x1UL << DCMI_SPI_CR_DATA_MSB_FIRST_Pos)
#define DCMI_SPI_CR_DATA_MSB_FIRST      DCMI_SPI_CR_DATA_MSB_FIRST_Msk
#define DCMI_SPI_CR_LOW_BYTE_FIRST_Pos  (6U)
#define DCMI_SPI_CR_LOW_BYTE_FIRST_Msk  (0x1UL << DCMI_SPI_CR_LOW_BYTE_FIRST_Pos)
#define DCMI_SPI_CR_LOW_BYTE_FIRST      DCMI_SPI_CR_LOW_BYTE_FIRST_Msk
#define DCMI_SPI_CR_LINE_NUM_EN_Pos     (7U)
#define DCMI_SPI_CR_LINE_NUM_EN_Msk     (0x1UL << DCMI_SPI_CR_LINE_NUM_EN_Pos)
#define DCMI_SPI_CR_LINE_NUM_EN         DCMI_SPI_CR_LINE_NUM_EN_Msk
#define DCMI_SPI_CR_PACKET_SIZE_Pos     (8U)
#define DCMI_SPI_CR_PACKET_SIZE_Msk     (0xFFFFUL << DCMI_SPI_CR_PACKET_SIZE_Pos)
#define DCMI_SPI_CR_PACKET_SIZE         DCMI_SPI_CR_PACKET_SIZE_Msk

/***************** Bit definition for DCMI_SPI_SYNC register ******************/
#define DCMI_SPI_SYNC_WORD_Pos          (0U)
#define DCMI_SPI_SYNC_WORD_Msk          (0xFFFFFFUL << DCMI_SPI_SYNC_WORD_Pos)
#define DCMI_SPI_SYNC_WORD              DCMI_SPI_SYNC_WORD_Msk

/**************** Bit definition for DCMI_SPI_PKT_ID register *****************/
#define DCMI_SPI_PKT_ID_FRAME_START_Pos  (0U)
#define DCMI_SPI_PKT_ID_FRAME_START_Msk  (0xFFUL << DCMI_SPI_PKT_ID_FRAME_START_Pos)
#define DCMI_SPI_PKT_ID_FRAME_START     DCMI_SPI_PKT_ID_FRAME_START_Msk
#define DCMI_SPI_PKT_ID_FRAME_END_Pos   (8U)
#define DCMI_SPI_PKT_ID_FRAME_END_Msk   (0xFFUL << DCMI_SPI_PKT_ID_FRAME_END_Pos)
#define DCMI_SPI_PKT_ID_FRAME_END       DCMI_SPI_PKT_ID_FRAME_END_Msk
#define DCMI_SPI_PKT_ID_LINE_START_Pos  (16U)
#define DCMI_SPI_PKT_ID_LINE_START_Msk  (0xFFUL << DCMI_SPI_PKT_ID_LINE_START_Pos)
#define DCMI_SPI_PKT_ID_LINE_START      DCMI_SPI_PKT_ID_LINE_START_Msk
#define DCMI_SPI_PKT_ID_LINE_END_Pos    (24U)
#define DCMI_SPI_PKT_ID_LINE_END_Msk    (0xFFUL << DCMI_SPI_PKT_ID_LINE_END_Pos)
#define DCMI_SPI_PKT_ID_LINE_END        DCMI_SPI_PKT_ID_LINE_END_Msk

/******************** Bit definition for DCMI_BP register *********************/
#define DCMI_BP_BYTE0_PO_Pos            (0U)
#define DCMI_BP_BYTE0_PO_Msk            (0x3UL << DCMI_BP_BYTE0_PO_Pos)
#define DCMI_BP_BYTE0_PO                DCMI_BP_BYTE0_PO_Msk
#define DCMI_BP_BYTE1_PO_Pos            (2U)
#define DCMI_BP_BYTE1_PO_Msk            (0x3UL << DCMI_BP_BYTE1_PO_Pos)
#define DCMI_BP_BYTE1_PO                DCMI_BP_BYTE1_PO_Msk
#define DCMI_BP_BYTE2_PO_Pos            (4U)
#define DCMI_BP_BYTE2_PO_Msk            (0x3UL << DCMI_BP_BYTE2_PO_Pos)
#define DCMI_BP_BYTE2_PO                DCMI_BP_BYTE2_PO_Msk
#define DCMI_BP_BYTE3_PO_Pos            (6U)
#define DCMI_BP_BYTE3_PO_Msk            (0x3UL << DCMI_BP_BYTE3_PO_Pos)
#define DCMI_BP_BYTE3_PO                DCMI_BP_BYTE3_PO_Msk

/****************** Bit definition for DCMI_SPI_SR register *******************/
#define DCMI_SPI_SR_LINE_NUM_Pos        (0U)
#define DCMI_SPI_SR_LINE_NUM_Msk        (0xFFFFUL << DCMI_SPI_SR_LINE_NUM_Pos)
#define DCMI_SPI_SR_LINE_NUM            DCMI_SPI_SR_LINE_NUM_Msk
#define DCMI_SPI_SR_PACKET_SIZE_Pos     (16U)
#define DCMI_SPI_SR_PACKET_SIZE_Msk     (0xFFFFUL << DCMI_SPI_SR_PACKET_SIZE_Pos)
#define DCMI_SPI_SR_PACKET_SIZE         DCMI_SPI_SR_PACKET_SIZE_Msk

/****************** Bit definition for DCMI_DMA_CR register *******************/
#define DCMI_DMA_CR_DMA_NUM_Pos         (0U)
#define DCMI_DMA_CR_DMA_NUM_Msk         (0x1FUL << DCMI_DMA_CR_DMA_NUM_Pos)
#define DCMI_DMA_CR_DMA_NUM             DCMI_DMA_CR_DMA_NUM_Msk

/****************** Bit definition for DCMI_DBG_CR register *******************/
#define DCMI_DBG_CR_DBG_SEL_Pos         (0U)
#define DCMI_DBG_CR_DBG_SEL_Msk         (0xFUL << DCMI_DBG_CR_DBG_SEL_Pos)
#define DCMI_DBG_CR_DBG_SEL             DCMI_DBG_CR_DBG_SEL_Msk
#define DCMI_DBG_CR_DBG_EN_Pos          (4U)
#define DCMI_DBG_CR_DBG_EN_Msk          (0x1UL << DCMI_DBG_CR_DBG_EN_Pos)
#define DCMI_DBG_CR_DBG_EN              DCMI_DBG_CR_DBG_EN_Msk

/**************** Bit definition for DCMI_YUV2RGB_CR1 register ****************/
#define DCMI_YUV2RGB_CR1_FUB_Pos        (0U)
#define DCMI_YUV2RGB_CR1_FUB_Msk        (0x3FFUL << DCMI_YUV2RGB_CR1_FUB_Pos)
#define DCMI_YUV2RGB_CR1_FUB            DCMI_YUV2RGB_CR1_FUB_Msk
#define DCMI_YUV2RGB_CR1_FUG_Pos        (10U)
#define DCMI_YUV2RGB_CR1_FUG_Msk        (0x3FFUL << DCMI_YUV2RGB_CR1_FUG_Pos)
#define DCMI_YUV2RGB_CR1_FUG            DCMI_YUV2RGB_CR1_FUG_Msk
#define DCMI_YUV2RGB_CR1_FY_Pos         (20U)
#define DCMI_YUV2RGB_CR1_FY_Msk         (0x3FFUL << DCMI_YUV2RGB_CR1_FY_Pos)
#define DCMI_YUV2RGB_CR1_FY             DCMI_YUV2RGB_CR1_FY_Msk
#define DCMI_YUV2RGB_CR1_RANGE_SEL_Pos  (30U)
#define DCMI_YUV2RGB_CR1_RANGE_SEL_Msk  (0x1UL << DCMI_YUV2RGB_CR1_RANGE_SEL_Pos)
#define DCMI_YUV2RGB_CR1_RANGE_SEL      DCMI_YUV2RGB_CR1_RANGE_SEL_Msk
#define DCMI_YUV2RGB_CR1_ENABLE_Pos     (31U)
#define DCMI_YUV2RGB_CR1_ENABLE_Msk     (0x1UL << DCMI_YUV2RGB_CR1_ENABLE_Pos)
#define DCMI_YUV2RGB_CR1_ENABLE         DCMI_YUV2RGB_CR1_ENABLE_Msk

/**************** Bit definition for DCMI_YUV2RGB_CR2 register ****************/
#define DCMI_YUV2RGB_CR2_FVG_Pos        (0U)
#define DCMI_YUV2RGB_CR2_FVG_Msk        (0x3FFUL << DCMI_YUV2RGB_CR2_FVG_Pos)
#define DCMI_YUV2RGB_CR2_FVG            DCMI_YUV2RGB_CR2_FVG_Msk
#define DCMI_YUV2RGB_CR2_FVR_Pos        (10U)
#define DCMI_YUV2RGB_CR2_FVR_Msk        (0x3FFUL << DCMI_YUV2RGB_CR2_FVR_Pos)
#define DCMI_YUV2RGB_CR2_FVR            DCMI_YUV2RGB_CR2_FVR_Msk

/***************** Bit definition for DCMI_JPEG_BYTE register *****************/
#define DCMI_JPEG_BYTE_COUNT_Pos        (0U)
#define DCMI_JPEG_BYTE_COUNT_Msk        (0xFFFFFFFFUL << DCMI_JPEG_BYTE_COUNT_Pos)
#define DCMI_JPEG_BYTE_COUNT            DCMI_JPEG_BYTE_COUNT_Msk

/****************** Bit definition for DCMI_IPI_CR register *******************/
#define DCMI_IPI_CR_DATA_TYPE_Pos       (0U)
#define DCMI_IPI_CR_DATA_TYPE_Msk       (0xFUL << DCMI_IPI_CR_DATA_TYPE_Pos)
#define DCMI_IPI_CR_DATA_TYPE           DCMI_IPI_CR_DATA_TYPE_Msk
#define DCMI_IPI_CR_EMBEDDED_BYPASS_Pos  (4U)
#define DCMI_IPI_CR_EMBEDDED_BYPASS_Msk  (0x1UL << DCMI_IPI_CR_EMBEDDED_BYPASS_Pos)
#define DCMI_IPI_CR_EMBEDDED_BYPASS     DCMI_IPI_CR_EMBEDDED_BYPASS_Msk

/****************** Bit definition for DCMI_CONV_CR register ******************/
#define DCMI_CONV_CR_CON_TO888_Pos      (0U)
#define DCMI_CONV_CR_CON_TO888_Msk      (0x1UL << DCMI_CONV_CR_CON_TO888_Pos)
#define DCMI_CONV_CR_CON_TO888          DCMI_CONV_CR_CON_TO888_Msk
#define DCMI_CONV_CR_CON_TO565_Pos      (1U)
#define DCMI_CONV_CR_CON_TO565_Msk      (0x1UL << DCMI_CONV_CR_CON_TO565_Pos)
#define DCMI_CONV_CR_CON_TO565          DCMI_CONV_CR_CON_TO565_Msk

/****************** Bit definition for DCMI_FIFO_SR register ******************/
#define DCMI_FIFO_SR_FIFO_CNT_Pos       (0U)
#define DCMI_FIFO_SR_FIFO_CNT_Msk       (0x1FFUL << DCMI_FIFO_SR_FIFO_CNT_Pos)
#define DCMI_FIFO_SR_FIFO_CNT           DCMI_FIFO_SR_FIFO_CNT_Msk

#endif
