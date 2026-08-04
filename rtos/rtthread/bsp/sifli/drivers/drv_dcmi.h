/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_DCMI_H__
#define __DRV_DCMI_H__

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DCMI_IF_CFG         0x01     /*dcmi interface config*/
#define DCMI_SPI_CFG        0x02     /*dcmi spi if config*/
#define DCMI_CM_CFG         0x03     /*dcmi capture mode config*/
#define DCMI_YUV2RGB_CFG    0x04     /*dcmi yuv2rgb config*/
#define DCMI_FB_CFG         0x05     /*dcmi frame buffer, size config*/
#define DCMI_ALL_CFG        0x0a     /*dcmi config all*/

/** @defgroup DCMI_TRANS_INTERFACE DCMI transfer interface
  * @{
  */
typedef enum
{
    DCMI_IF_DVP = 0,
    DCMI_IF_SPI = 1,
} DCMI_TRANS_INTERFACE;
/**
  * @}
  */

/** @defgroup DCMI_SPI_DATA_LINE DCMI spi data line
  * @{
  */
typedef enum
{
    DCMI_SPI_1LINE = 1,
    DCMI_SPI_2LINE = 2,
    DCMI_SPI_4LINE = 4,
} DCMI_SPI_DATA_LINE;
/**
  * @}
  */

/** @defgroup DCMI_SPI_CLK_EG DCMI spi clock eg
  * @{
  */
typedef enum
{
    DCMI_SPI_CLK_P = 0, //spi clk posedge
    DCMI_SPI_CLK_N = 1, //spi clk negedge
} DCMI_SPI_CLK_EG;
/**
  * @}
  */

/** @defgroup DCMI_CPT_MODE DCMI cpature mode
  * @{
  */
typedef enum
{
    DCMI_CPT_CON = 0, //continous mode
    DCMI_CPT_SIG = 1, //single shot snap
} DCMI_CPT_MODE;
/**
  * @}
  */

/** @defgroup DCMI_CPT_MODE DCMI cpature mode
  * @{
  */
typedef enum
{
    DCMI_YUV2RGB_DIS    = 0,
    DCMI_YUV2RGB_DFT_EN = 1, //enable and use default params
    DCMI_YUV2RGB_UPD_EN = 2, //enable and update yuv2rgb params
} DCMI_YUV2RGB;
/**
  * @}
  */

/** @defgroup dcmi spi interface config params
  * @{
  */
struct dcmi_spi_cfg
{
    uint32_t pkt_size; //bytes of one line
    uint32_t pkt_id; //low to high: frame_start, frame_end, line_start, line_end
    uint8_t spi_dataline; //spi data lines
    uint8_t spi_clkeg; //0: spi clk posedge; 1: spi clk negedge
    uint32_t spi_sync;
};
/**
  * @}
  */

/** @defgroup dcmi yuv2rgb config params
  * @{
  */
struct dcmi_yuv2rgb_cfg
{
    uint16_t fy;
    uint16_t fug;
    uint16_t fub;
    uint16_t fvr;
    uint16_t fvg;
};
/**
  * @}
  */

/** @defgroup dcmi config params
  * @{
  */
struct rt_dcmi_config
{
    uint8_t dcmi_cfg;
    uint8_t dcmi_if; //0:DVP interface; 1:spi interface
    struct dcmi_spi_cfg dcmi_spi; //only dcmi_if = 0 used
    uint8_t dcmi_cm; //0:continous mode 1:single shot snap
    uint8_t yuv2rgb_en;
    struct dcmi_yuv2rgb_cfg yuv2rgb;
    uint8_t *frame_buffer;
    uint32_t buffer_size;
};
/**
  * @}
  */

/** @defgroup dcmi device struct
  * @{
  */
struct rt_dcmi_device
{
    struct rt_device parent;
    DCMI_HandleTypeDef handle;
    DMA_HandleTypeDef hdma;
    IRQn_Type dcmi_irq;
    uint8_t *frame_buffer;
    uint32_t buffer_size;
    uint32_t error;
};
/**
  * @}
  */

void rt_hw_dcmi_dma_config(rt_device_t dev);
void rt_hw_dcmi_spi_config(rt_device_t dev, struct dcmi_spi_cfg *spi_cfg);


#define DCMI_ERROR_PACK_SIZE -1
#define DCMI_ERROR_FSM       -2
#define DCMI_ERROR_OVR       -3
extern int rt_hw_dcmi_get_error(rt_device_t dev);
struct rt_dcmi_device *rt_get_dcmi_device(void);

#ifdef __cplusplus
}
#endif

#endif


