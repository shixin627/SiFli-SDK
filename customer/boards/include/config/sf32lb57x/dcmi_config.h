/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DCMI_CONFIG_H__
#define __DCMI_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_DCMI

#ifndef BF0_DCMI_DMA_CONFIG
#define BF0_DCMI_DMA_CONFIG \
    {                                                           \
        .Instance = DCMI_DMA_INSTANCE,                          \
        .request  = DCMI_DMA_REQUEST,                           \
        .dma_irq_prio  = DCMI_DMA_IRQ_PRIO,                     \
        .dma_irq  = DCMI_DMA_IRQ,                               \
    }
#endif /* BF0_I2S2_CONFIG */

#endif /*  BSP_USING_DCMI */


#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_CONFIG_H__ */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
