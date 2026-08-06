/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_H
#define __SPI_H

typedef struct
{
    __IO uint32_t TOP_CTRL;
    __IO uint32_t FIFO_CTRL;
    __IO uint32_t INTE;
    __IO uint32_t TO;
    __IO uint32_t STATUS;
    __IO uint32_t DATA;
    __IO uint32_t FRM_HDR_DATA;
    __IO uint32_t TOP_CTRL2;
    __IO uint32_t RWOT_CVWRN;
} SPI_TypeDef;


/****************** Bit definition for SPI_TOP_CTRL register ******************/
#define SPI_TOP_CTRL_SSE_Pos            (0U)
#define SPI_TOP_CTRL_SSE_Msk            (0x1UL << SPI_TOP_CTRL_SSE_Pos)
#define SPI_TOP_CTRL_SSE                SPI_TOP_CTRL_SSE_Msk
#define SPI_TOP_CTRL_FRF_Pos            (1U)
#define SPI_TOP_CTRL_FRF_Msk            (0x3UL << SPI_TOP_CTRL_FRF_Pos)
#define SPI_TOP_CTRL_FRF                SPI_TOP_CTRL_FRF_Msk
#define SPI_TOP_CTRL_SCLKDIR_Pos        (3U)
#define SPI_TOP_CTRL_SCLKDIR_Msk        (0x1UL << SPI_TOP_CTRL_SCLKDIR_Pos)
#define SPI_TOP_CTRL_SCLKDIR            SPI_TOP_CTRL_SCLKDIR_Msk
#define SPI_TOP_CTRL_SFRMDIR_Pos        (4U)
#define SPI_TOP_CTRL_SFRMDIR_Msk        (0x1UL << SPI_TOP_CTRL_SFRMDIR_Pos)
#define SPI_TOP_CTRL_SFRMDIR            SPI_TOP_CTRL_SFRMDIR_Msk
#define SPI_TOP_CTRL_DSS_Pos            (5U)
#define SPI_TOP_CTRL_DSS_Msk            (0x1FUL << SPI_TOP_CTRL_DSS_Pos)
#define SPI_TOP_CTRL_DSS                SPI_TOP_CTRL_DSS_Msk
#define SPI_TOP_CTRL_SPO_Pos            (10U)
#define SPI_TOP_CTRL_SPO_Msk            (0x1UL << SPI_TOP_CTRL_SPO_Pos)
#define SPI_TOP_CTRL_SPO                SPI_TOP_CTRL_SPO_Msk
#define SPI_TOP_CTRL_SPH_Pos            (11U)
#define SPI_TOP_CTRL_SPH_Msk            (0x1UL << SPI_TOP_CTRL_SPH_Pos)
#define SPI_TOP_CTRL_SPH                SPI_TOP_CTRL_SPH_Msk
#define SPI_TOP_CTRL_LBM_Pos            (12U)
#define SPI_TOP_CTRL_LBM_Msk            (0x1UL << SPI_TOP_CTRL_LBM_Pos)
#define SPI_TOP_CTRL_LBM                SPI_TOP_CTRL_LBM_Msk
#define SPI_TOP_CTRL_TRAIL_Pos          (13U)
#define SPI_TOP_CTRL_TRAIL_Msk          (0x1UL << SPI_TOP_CTRL_TRAIL_Pos)
#define SPI_TOP_CTRL_TRAIL              SPI_TOP_CTRL_TRAIL_Msk
#define SPI_TOP_CTRL_HOLD_FRAME_LOW_Pos  (14U)
#define SPI_TOP_CTRL_HOLD_FRAME_LOW_Msk  (0x1UL << SPI_TOP_CTRL_HOLD_FRAME_LOW_Pos)
#define SPI_TOP_CTRL_HOLD_FRAME_LOW     SPI_TOP_CTRL_HOLD_FRAME_LOW_Msk
#define SPI_TOP_CTRL_IFS_Pos            (15U)
#define SPI_TOP_CTRL_IFS_Msk            (0x1UL << SPI_TOP_CTRL_IFS_Pos)
#define SPI_TOP_CTRL_IFS                SPI_TOP_CTRL_IFS_Msk
#define SPI_TOP_CTRL_SCFR_Pos           (16U)
#define SPI_TOP_CTRL_SCFR_Msk           (0x1UL << SPI_TOP_CTRL_SCFR_Pos)
#define SPI_TOP_CTRL_SCFR               SPI_TOP_CTRL_SCFR_Msk
#define SPI_TOP_CTRL_TTE_Pos            (17U)
#define SPI_TOP_CTRL_TTE_Msk            (0x1UL << SPI_TOP_CTRL_TTE_Pos)
#define SPI_TOP_CTRL_TTE                SPI_TOP_CTRL_TTE_Msk
#define SPI_TOP_CTRL_TTELP_Pos          (18U)
#define SPI_TOP_CTRL_TTELP_Msk          (0x1UL << SPI_TOP_CTRL_TTELP_Pos)
#define SPI_TOP_CTRL_TTELP              SPI_TOP_CTRL_TTELP_Msk
#define SPI_TOP_CTRL_CLK_DIV_Pos        (19U)
#define SPI_TOP_CTRL_CLK_DIV_Msk        (0x7FUL << SPI_TOP_CTRL_CLK_DIV_Pos)
#define SPI_TOP_CTRL_CLK_DIV            SPI_TOP_CTRL_CLK_DIV_Msk
#define SPI_TOP_CTRL_CLK_SEL_Pos        (26U)
#define SPI_TOP_CTRL_CLK_SEL_Msk        (0x1UL << SPI_TOP_CTRL_CLK_SEL_Pos)
#define SPI_TOP_CTRL_CLK_SEL            SPI_TOP_CTRL_CLK_SEL_Msk
#define SPI_TOP_CTRL_CLK_SSP_EN_Pos     (27U)
#define SPI_TOP_CTRL_CLK_SSP_EN_Msk     (0x1UL << SPI_TOP_CTRL_CLK_SSP_EN_Pos)
#define SPI_TOP_CTRL_CLK_SSP_EN         SPI_TOP_CTRL_CLK_SSP_EN_Msk
#define SPI_TOP_CTRL_SPI_DI_SEL_Pos     (28U)
#define SPI_TOP_CTRL_SPI_DI_SEL_Msk     (0x1UL << SPI_TOP_CTRL_SPI_DI_SEL_Pos)
#define SPI_TOP_CTRL_SPI_DI_SEL         SPI_TOP_CTRL_SPI_DI_SEL_Msk
#define SPI_TOP_CTRL_SPI_TRI_WIRE_EN_Pos  (29U)
#define SPI_TOP_CTRL_SPI_TRI_WIRE_EN_Msk  (0x1UL << SPI_TOP_CTRL_SPI_TRI_WIRE_EN_Pos)
#define SPI_TOP_CTRL_SPI_TRI_WIRE_EN    SPI_TOP_CTRL_SPI_TRI_WIRE_EN_Msk
#define SPI_TOP_CTRL_TXD_OEN_Pos        (30U)
#define SPI_TOP_CTRL_TXD_OEN_Msk        (0x1UL << SPI_TOP_CTRL_TXD_OEN_Pos)
#define SPI_TOP_CTRL_TXD_OEN            SPI_TOP_CTRL_TXD_OEN_Msk
#define SPI_TOP_CTRL_SSP_WORK_WIDTH_DYN_CHANGE_Pos  (31U)
#define SPI_TOP_CTRL_SSP_WORK_WIDTH_DYN_CHANGE_Msk  (0x1UL << SPI_TOP_CTRL_SSP_WORK_WIDTH_DYN_CHANGE_Pos)
#define SPI_TOP_CTRL_SSP_WORK_WIDTH_DYN_CHANGE  SPI_TOP_CTRL_SSP_WORK_WIDTH_DYN_CHANGE_Msk

/***************** Bit definition for SPI_FIFO_CTRL register ******************/
#define SPI_FIFO_CTRL_TFT_Pos           (0U)
#define SPI_FIFO_CTRL_TFT_Msk           (0x1FUL << SPI_FIFO_CTRL_TFT_Pos)
#define SPI_FIFO_CTRL_TFT               SPI_FIFO_CTRL_TFT_Msk
#define SPI_FIFO_CTRL_RFT_Pos           (5U)
#define SPI_FIFO_CTRL_RFT_Msk           (0x1FUL << SPI_FIFO_CTRL_RFT_Pos)
#define SPI_FIFO_CTRL_RFT               SPI_FIFO_CTRL_RFT_Msk
#define SPI_FIFO_CTRL_TSRE_Pos          (10U)
#define SPI_FIFO_CTRL_TSRE_Msk          (0x1UL << SPI_FIFO_CTRL_TSRE_Pos)
#define SPI_FIFO_CTRL_TSRE              SPI_FIFO_CTRL_TSRE_Msk
#define SPI_FIFO_CTRL_RSRE_Pos          (11U)
#define SPI_FIFO_CTRL_RSRE_Msk          (0x1UL << SPI_FIFO_CTRL_RSRE_Pos)
#define SPI_FIFO_CTRL_RSRE              SPI_FIFO_CTRL_RSRE_Msk
#define SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Pos  (12U)
#define SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Msk  (0x3UL << SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Pos)
#define SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN  SPI_FIFO_CTRL_RXFIFO_RD_ENDIAN_Msk
#define SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Pos  (14U)
#define SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Msk  (0x3UL << SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Pos)
#define SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN  SPI_FIFO_CTRL_TXFIFO_WR_ENDIAN_Msk
#define SPI_FIFO_CTRL_FPCKE_Pos         (16U)
#define SPI_FIFO_CTRL_FPCKE_Msk         (0x1UL << SPI_FIFO_CTRL_FPCKE_Pos)
#define SPI_FIFO_CTRL_FPCKE             SPI_FIFO_CTRL_FPCKE_Msk
#define SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL_Pos  (17U)
#define SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL_Msk  (0x1UL << SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL_Pos)
#define SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL  SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL_Msk
#define SPI_FIFO_CTRL_EFWR_Pos          (18U)
#define SPI_FIFO_CTRL_EFWR_Msk          (0x1UL << SPI_FIFO_CTRL_EFWR_Pos)
#define SPI_FIFO_CTRL_EFWR              SPI_FIFO_CTRL_EFWR_Msk
#define SPI_FIFO_CTRL_STRF_Pos          (19U)
#define SPI_FIFO_CTRL_STRF_Msk          (0x1UL << SPI_FIFO_CTRL_STRF_Pos)
#define SPI_FIFO_CTRL_STRF              SPI_FIFO_CTRL_STRF_Msk

/******************** Bit definition for SPI_INTE register ********************/
#define SPI_INTE_PINTE_Pos              (0U)
#define SPI_INTE_PINTE_Msk              (0x1UL << SPI_INTE_PINTE_Pos)
#define SPI_INTE_PINTE                  SPI_INTE_PINTE_Msk
#define SPI_INTE_TINTE_Pos              (1U)
#define SPI_INTE_TINTE_Msk              (0x1UL << SPI_INTE_TINTE_Pos)
#define SPI_INTE_TINTE                  SPI_INTE_TINTE_Msk
#define SPI_INTE_RIE_Pos                (2U)
#define SPI_INTE_RIE_Msk                (0x1UL << SPI_INTE_RIE_Pos)
#define SPI_INTE_RIE                    SPI_INTE_RIE_Msk
#define SPI_INTE_TIE_Pos                (3U)
#define SPI_INTE_TIE_Msk                (0x1UL << SPI_INTE_TIE_Pos)
#define SPI_INTE_TIE                    SPI_INTE_TIE_Msk
#define SPI_INTE_RIM_Pos                (4U)
#define SPI_INTE_RIM_Msk                (0x1UL << SPI_INTE_RIM_Pos)
#define SPI_INTE_RIM                    SPI_INTE_RIM_Msk
#define SPI_INTE_TIM_Pos                (5U)
#define SPI_INTE_TIM_Msk                (0x1UL << SPI_INTE_TIM_Pos)
#define SPI_INTE_TIM                    SPI_INTE_TIM_Msk
#define SPI_INTE_EBCEI_Pos              (6U)
#define SPI_INTE_EBCEI_Msk              (0x1UL << SPI_INTE_EBCEI_Pos)
#define SPI_INTE_EBCEI                  SPI_INTE_EBCEI_Msk

/********************* Bit definition for SPI_TO register *********************/
#define SPI_TO_TIMEOUT_Pos              (0U)
#define SPI_TO_TIMEOUT_Msk              (0xFFFFFFUL << SPI_TO_TIMEOUT_Pos)
#define SPI_TO_TIMEOUT                  SPI_TO_TIMEOUT_Msk

/******************* Bit definition for SPI_STATUS register *******************/
#define SPI_STATUS_BSY_Pos              (0U)
#define SPI_STATUS_BSY_Msk              (0x1UL << SPI_STATUS_BSY_Pos)
#define SPI_STATUS_BSY                  SPI_STATUS_BSY_Msk
#define SPI_STATUS_CSS_Pos              (1U)
#define SPI_STATUS_CSS_Msk              (0x1UL << SPI_STATUS_CSS_Pos)
#define SPI_STATUS_CSS                  SPI_STATUS_CSS_Msk
#define SPI_STATUS_PINT_Pos             (2U)
#define SPI_STATUS_PINT_Msk             (0x1UL << SPI_STATUS_PINT_Pos)
#define SPI_STATUS_PINT                 SPI_STATUS_PINT_Msk
#define SPI_STATUS_TINT_Pos             (3U)
#define SPI_STATUS_TINT_Msk             (0x1UL << SPI_STATUS_TINT_Pos)
#define SPI_STATUS_TINT                 SPI_STATUS_TINT_Msk
#define SPI_STATUS_EOC_Pos              (4U)
#define SPI_STATUS_EOC_Msk              (0x1UL << SPI_STATUS_EOC_Pos)
#define SPI_STATUS_EOC                  SPI_STATUS_EOC_Msk
#define SPI_STATUS_TFS_Pos              (5U)
#define SPI_STATUS_TFS_Msk              (0x1UL << SPI_STATUS_TFS_Pos)
#define SPI_STATUS_TFS                  SPI_STATUS_TFS_Msk
#define SPI_STATUS_TNF_Pos              (6U)
#define SPI_STATUS_TNF_Msk              (0x1UL << SPI_STATUS_TNF_Pos)
#define SPI_STATUS_TNF                  SPI_STATUS_TNF_Msk
#define SPI_STATUS_TFL_Pos              (7U)
#define SPI_STATUS_TFL_Msk              (0xFUL << SPI_STATUS_TFL_Pos)
#define SPI_STATUS_TFL                  SPI_STATUS_TFL_Msk
#define SPI_STATUS_TUR_Pos              (12U)
#define SPI_STATUS_TUR_Msk              (0x1UL << SPI_STATUS_TUR_Pos)
#define SPI_STATUS_TUR                  SPI_STATUS_TUR_Msk
#define SPI_STATUS_RFS_Pos              (13U)
#define SPI_STATUS_RFS_Msk              (0x1UL << SPI_STATUS_RFS_Pos)
#define SPI_STATUS_RFS                  SPI_STATUS_RFS_Msk
#define SPI_STATUS_RNE_Pos              (14U)
#define SPI_STATUS_RNE_Msk              (0x1UL << SPI_STATUS_RNE_Pos)
#define SPI_STATUS_RNE                  SPI_STATUS_RNE_Msk
#define SPI_STATUS_RFL_Pos              (15U)
#define SPI_STATUS_RFL_Msk              (0xFUL << SPI_STATUS_RFL_Pos)
#define SPI_STATUS_RFL                  SPI_STATUS_RFL_Msk
#define SPI_STATUS_ROR_Pos              (20U)
#define SPI_STATUS_ROR_Msk              (0x1UL << SPI_STATUS_ROR_Pos)
#define SPI_STATUS_ROR                  SPI_STATUS_ROR_Msk
#define SPI_STATUS_BCE_Pos              (21U)
#define SPI_STATUS_BCE_Msk              (0x1UL << SPI_STATUS_BCE_Pos)
#define SPI_STATUS_BCE                  SPI_STATUS_BCE_Msk
#define SPI_STATUS_TX_OSS_Pos           (22U)
#define SPI_STATUS_TX_OSS_Msk           (0x1UL << SPI_STATUS_TX_OSS_Pos)
#define SPI_STATUS_TX_OSS               SPI_STATUS_TX_OSS_Msk
#define SPI_STATUS_OSS_Pos              (23U)
#define SPI_STATUS_OSS_Msk              (0x1UL << SPI_STATUS_OSS_Pos)
#define SPI_STATUS_OSS                  SPI_STATUS_OSS_Msk

/******************** Bit definition for SPI_DATA register ********************/
#define SPI_DATA_DATA_Pos               (0U)
#define SPI_DATA_DATA_Msk               (0xFFFFFFFFUL << SPI_DATA_DATA_Pos)
#define SPI_DATA_DATA                   SPI_DATA_DATA_Msk

/**************** Bit definition for SPI_FRM_HDR_DATA register ****************/
#define SPI_FRM_HDR_DATA_HDR_DATA_Pos   (0U)
#define SPI_FRM_HDR_DATA_HDR_DATA_Msk   (0xFFFFFFFFUL << SPI_FRM_HDR_DATA_HDR_DATA_Pos)
#define SPI_FRM_HDR_DATA_HDR_DATA       SPI_FRM_HDR_DATA_HDR_DATA_Msk

/***************** Bit definition for SPI_TOP_CTRL2 register ******************/
#define SPI_TOP_CTRL2_RWOT_Pos          (0U)
#define SPI_TOP_CTRL2_RWOT_Msk          (0x1UL << SPI_TOP_CTRL2_RWOT_Pos)
#define SPI_TOP_CTRL2_RWOT              SPI_TOP_CTRL2_RWOT_Msk
#define SPI_TOP_CTRL2_CYCLE_RWOT_EN_Pos  (1U)
#define SPI_TOP_CTRL2_CYCLE_RWOT_EN_Msk  (0x1UL << SPI_TOP_CTRL2_CYCLE_RWOT_EN_Pos)
#define SPI_TOP_CTRL2_CYCLE_RWOT_EN     SPI_TOP_CTRL2_CYCLE_RWOT_EN_Msk
#define SPI_TOP_CTRL2_SET_RWOT_CYCLE_Pos  (2U)
#define SPI_TOP_CTRL2_SET_RWOT_CYCLE_Msk  (0x1UL << SPI_TOP_CTRL2_SET_RWOT_CYCLE_Pos)
#define SPI_TOP_CTRL2_SET_RWOT_CYCLE    SPI_TOP_CTRL2_SET_RWOT_CYCLE_Msk
#define SPI_TOP_CTRL2_CLR_RWOT_CYCLE_Pos  (3U)
#define SPI_TOP_CTRL2_CLR_RWOT_CYCLE_Msk  (0x1UL << SPI_TOP_CTRL2_CLR_RWOT_CYCLE_Pos)
#define SPI_TOP_CTRL2_CLR_RWOT_CYCLE    SPI_TOP_CTRL2_CLR_RWOT_CYCLE_Msk
#define SPI_TOP_CTRL2_MASK_RWOT_LAST_SAMPLE_Pos  (4U)
#define SPI_TOP_CTRL2_MASK_RWOT_LAST_SAMPLE_Msk  (0x1UL << SPI_TOP_CTRL2_MASK_RWOT_LAST_SAMPLE_Pos)
#define SPI_TOP_CTRL2_MASK_RWOT_LAST_SAMPLE  SPI_TOP_CTRL2_MASK_RWOT_LAST_SAMPLE_Msk
#define SPI_TOP_CTRL2_FRM_START_Pos     (5U)
#define SPI_TOP_CTRL2_FRM_START_Msk     (0x1UL << SPI_TOP_CTRL2_FRM_START_Pos)
#define SPI_TOP_CTRL2_FRM_START         SPI_TOP_CTRL2_FRM_START_Msk
#define SPI_TOP_CTRL2_FRM_HDR_LENGTH_Pos  (6U)
#define SPI_TOP_CTRL2_FRM_HDR_LENGTH_Msk  (0x1FUL << SPI_TOP_CTRL2_FRM_HDR_LENGTH_Pos)
#define SPI_TOP_CTRL2_FRM_HDR_LENGTH    SPI_TOP_CTRL2_FRM_HDR_LENGTH_Msk
#define SPI_TOP_CTRL2_FRM_TRI_WIRE_EN_Pos  (11U)
#define SPI_TOP_CTRL2_FRM_TRI_WIRE_EN_Msk  (0x1UL << SPI_TOP_CTRL2_FRM_TRI_WIRE_EN_Pos)
#define SPI_TOP_CTRL2_FRM_TRI_WIRE_EN   SPI_TOP_CTRL2_FRM_TRI_WIRE_EN_Msk
#define SPI_TOP_CTRL2_FRM_MODE_Pos      (12U)
#define SPI_TOP_CTRL2_FRM_MODE_Msk      (0x1UL << SPI_TOP_CTRL2_FRM_MODE_Pos)
#define SPI_TOP_CTRL2_FRM_MODE          SPI_TOP_CTRL2_FRM_MODE_Msk
#define SPI_TOP_CTRL2_INV_RX_CLK_Pos    (13U)
#define SPI_TOP_CTRL2_INV_RX_CLK_Msk    (0x1UL << SPI_TOP_CTRL2_INV_RX_CLK_Pos)
#define SPI_TOP_CTRL2_INV_RX_CLK        SPI_TOP_CTRL2_INV_RX_CLK_Msk
#define SPI_TOP_CTRL2_SELECT_GPIO_CLKI_Pos  (14U)
#define SPI_TOP_CTRL2_SELECT_GPIO_CLKI_Msk  (0x1UL << SPI_TOP_CTRL2_SELECT_GPIO_CLKI_Pos)
#define SPI_TOP_CTRL2_SELECT_GPIO_CLKI  SPI_TOP_CTRL2_SELECT_GPIO_CLKI_Msk
#define SPI_TOP_CTRL2_SSPRWOTCCM_Pos    (16U)
#define SPI_TOP_CTRL2_SSPRWOTCCM_Msk    (0xFFFFUL << SPI_TOP_CTRL2_SSPRWOTCCM_Pos)
#define SPI_TOP_CTRL2_SSPRWOTCCM        SPI_TOP_CTRL2_SSPRWOTCCM_Msk

/***************** Bit definition for SPI_RWOT_CVWRN register *****************/
#define SPI_RWOT_CVWRN_SSPRWOTCVWR_Pos  (0U)
#define SPI_RWOT_CVWRN_SSPRWOTCVWR_Msk  (0xFFFFFFFFUL << SPI_RWOT_CVWRN_SSPRWOTCVWR_Pos)
#define SPI_RWOT_CVWRN_SSPRWOTCVWR      SPI_RWOT_CVWRN_SSPRWOTCVWR_Msk

#endif
