/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PWM_CONFIG_H__
#define __PWM_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWMT1_CORE CORE_ID_HCPU
#define PWMT2_CORE CORE_ID_HCPU
#define PWMT3_CORE CORE_ID_LCPU
#define PWMT4_CORE CORE_ID_LCPU
#define PWMT5_CORE CORE_ID_LCPU
#define PWMA1_CORE CORE_ID_HCPU

#ifdef BSP_USING_PWMT1
#define PWMT1_CONFIG                                                           \
    {                                                                          \
       .tim_handle.Instance     = GPTIM1,                                      \
       .tim_handle.core         = PWMT1_CORE,                                  \
       .name                    = "pwmt1",                                     \
       .channel                 = 0                                            \
    }
#endif /* BSP_USING_PWMT1 */

#ifdef BSP_PWMT1_UPDATE_USING_DMA
#define PWMT1_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT1_UPDATE_DMA_IRQ_PRIO,       \
       .dma_handle.Instance                 = PWMT1_UPDATE_DMA_INSTANCE,       \
       .dma_handle.Init.Request             = PWMT1_UPDATE_DMA_REQUEST,        \
       .dma_irq                             = PWMT1_UPDATE_DMA_IRQ,            \
       .dma_handle_index                    = GPT_DMA_ID_UPDATE,               \
       .dma_handle.Init.PeriphDataAlignment = PWMT1_UPDATE_DMA_PDATAALIGN,     \
       .dma_handle.Init.MemDataAlignment    = PWMT1_UPDATE_DMA_MDATAALIGN      \
    }
#endif /* BSP_PWMT1_UPDATE_USING_DMA */

#ifdef BSP_PWMT1_CC1_USING_DMA
#define PWMT1_CC1_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT1_CC1_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT1_CC1_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT1_CC1_DMA_REQUEST,           \
       .dma_irq                             = PWMT1_CC1_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC1,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT1_CC1_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT1_CC1_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT1_CC1_USING_DMA */

#ifdef BSP_PWMT1_CC2_USING_DMA
#define PWMT1_CC2_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT1_CC2_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT1_CC2_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT1_CC2_DMA_REQUEST,           \
       .dma_irq                             = PWMT1_CC2_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC2,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT1_CC2_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT1_CC2_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT1_CC2_USING_DMA */

#ifdef BSP_PWMT1_CC3_USING_DMA
#define PWMT1_CC3_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT1_CC3_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT1_CC3_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT1_CC3_DMA_REQUEST,           \
       .dma_irq                             = PWMT1_CC3_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC3,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT1_CC3_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT1_CC3_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT1_CC3_USING_DMA */

#ifdef BSP_PWMT1_CC4_USING_DMA
#define PWMT1_CC4_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT1_CC4_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT1_CC4_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT1_CC4_DMA_REQUEST,           \
       .dma_irq                             = PWMT1_CC4_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC4,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT1_CC4_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT1_CC4_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT1_CC4_USING_DMA */

#ifdef BSP_USING_PWMT2
#define PWMT2_CONFIG                                                           \
    {                                                                          \
       .tim_handle.Instance     = GPTIM2,                                      \
       .tim_handle.core         = PWMT2_CORE,                                  \
       .name                    = "pwmt2",                                     \
       .channel                 = 0                                            \
    }
#endif /* BSP_USING_PWMT2 */

#ifdef BSP_PWMT2_UPDATE_USING_DMA
#define PWMT2_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT2_UPDATE_DMA_IRQ_PRIO,       \
       .dma_handle.Instance                 = PWMT2_UPDATE_DMA_INSTANCE,       \
       .dma_handle.Init.Request             = PWMT2_UPDATE_DMA_REQUEST,        \
       .dma_irq                             = PWMT2_UPDATE_DMA_IRQ,            \
       .dma_handle_index                    = GPT_DMA_ID_UPDATE,               \
       .dma_handle.Init.PeriphDataAlignment = PWMT2_UPDATE_DMA_PDATAALIGN,     \
       .dma_handle.Init.MemDataAlignment    = PWMT2_UPDATE_DMA_MDATAALIGN      \
    }
#endif /* BSP_USING_PWMT2 */

#ifdef BSP_PWMT2_CC1_USING_DMA
#define PWMT2_CC1_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT2_CC1_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT2_CC1_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT2_CC1_DMA_REQUEST,           \
       .dma_irq                             = PWMT2_CC1_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC1,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT2_CC1_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT2_CC1_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT2_CC1_USING_DMA */

#ifdef BSP_PWMT2_CC2_USING_DMA
#define PWMT2_CC2_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT2_CC2_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT2_CC2_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT2_CC2_DMA_REQUEST,           \
       .dma_irq                             = PWMT2_CC2_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC2,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT2_CC2_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT2_CC2_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT2_CC2_USING_DMA */

#ifdef BSP_USING_PWMT3
#define PWMT3_CONFIG                                                           \
    {                                                                          \
       .tim_handle.Instance     = GPTIM3,                                      \
       .tim_handle.core         = PWMT3_CORE,                                  \
       .name                    = "pwmt3",                                     \
       .channel                 = 0                                            \
    }
#endif /* BSP_USING_PWMT3 */

#ifdef BSP_PWMT3_UPDATE_USING_DMA
#define PWMT3_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT3_UPDATE_DMA_IRQ_PRIO,       \
       .dma_handle.Instance                 = PWMT3_UPDATE_DMA_INSTANCE,       \
       .dma_handle.Init.Request             = PWMT3_UPDATE_DMA_REQUEST,        \
       .dma_irq                             = PWMT3_UPDATE_DMA_IRQ,            \
       .dma_handle_index                    = GPT_DMA_ID_UPDATE,               \
       .dma_handle.Init.PeriphDataAlignment = PWMT3_UPDATE_DMA_PDATAALIGN,     \
       .dma_handle.Init.MemDataAlignment    = PWMT3_UPDATE_DMA_MDATAALIGN      \
    }
#endif /* BSP_PWMT3_UPDATE_USING_DMA */

#ifdef BSP_PWMT3_CC1_USING_DMA
#define PWMT3_CC1_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT3_CC1_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT3_CC1_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT3_CC1_DMA_REQUEST,           \
       .dma_irq                             = PWMT3_CC1_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC1,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT3_CC1_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT3_CC1_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT3_CC1_USING_DMA */

#ifdef BSP_PWMT3_CC2_USING_DMA
#define PWMT3_CC2_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT3_CC2_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT3_CC2_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT3_CC2_DMA_REQUEST,           \
       .dma_irq                             = PWMT3_CC2_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC2,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT3_CC2_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT3_CC2_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT3_CC2_USING_DMA */

#ifdef BSP_PWMT3_CC3_USING_DMA
#define PWMT3_CC3_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT3_CC3_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT3_CC3_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT3_CC3_DMA_REQUEST,           \
       .dma_irq                             = PWMT3_CC3_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC3,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT3_CC3_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT3_CC3_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT3_CC3_USING_DMA */

#ifdef BSP_PWMT3_CC4_USING_DMA
#define PWMT3_CC4_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT3_CC4_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT3_CC4_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT3_CC4_DMA_REQUEST,           \
       .dma_irq                             = PWMT3_CC4_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC4,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT3_CC4_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT3_CC4_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT3_CC4_USING_DMA */

#ifdef BSP_USING_PWMT4
#define PWMT4_CONFIG                                                           \
    {                                                                          \
       .tim_handle.Instance     = GPTIM4,                                      \
       .tim_handle.core         = PWMT4_CORE,                                  \
       .name                    = "pwmt4",                                     \
       .channel                 = 0                                            \
    }
#endif /* BSP_USING_PWMT4 */

#ifdef BSP_PWMT4_UPDATE_USING_DMA
#define PWMT4_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT4_UPDATE_DMA_IRQ_PRIO,       \
       .dma_handle.Instance                 = PWMT4_UPDATE_DMA_INSTANCE,       \
       .dma_handle.Init.Request             = PWMT4_UPDATE_DMA_REQUEST,        \
       .dma_irq                             = PWMT4_UPDATE_DMA_IRQ,            \
       .dma_handle_index                    = GPT_DMA_ID_UPDATE,               \
       .dma_handle.Init.PeriphDataAlignment = PWMT4_UPDATE_DMA_PDATAALIGN,     \
       .dma_handle.Init.MemDataAlignment    = PWMT4_UPDATE_DMA_MDATAALIGN      \
    }
#endif /* BSP_PWMT4_UPDATE_USING_DMA */

#ifdef BSP_PWMT4_CC1_USING_DMA
#define PWMT4_CC1_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT4_CC1_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT4_CC1_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT4_CC1_DMA_REQUEST,           \
       .dma_irq                             = PWMT4_CC1_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC1,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT4_CC1_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT4_CC1_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT4_CC1_USING_DMA */

#ifdef BSP_PWMT4_CC2_USING_DMA
#define PWMT4_CC2_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT4_CC2_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT4_CC2_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT4_CC2_DMA_REQUEST,           \
       .dma_irq                             = PWMT4_CC2_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC2,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT4_CC2_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT4_CC2_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT4_CC2_USING_DMA */

#ifdef BSP_PWMT4_CC3_USING_DMA
#define PWMT4_CC3_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT4_CC3_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT4_CC3_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT4_CC3_DMA_REQUEST,           \
       .dma_irq                             = PWMT4_CC3_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC3,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT4_CC3_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT4_CC3_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT4_CC3_USING_DMA */

#ifdef BSP_PWMT4_CC4_USING_DMA
#define PWMT4_CC4_DMA_CONFIG                                                   \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT4_CC4_DMA_IRQ_PRIO,          \
       .dma_handle.Instance                 = PWMT4_CC4_DMA_INSTANCE,          \
       .dma_handle.Init.Request             = PWMT4_CC4_DMA_REQUEST,           \
       .dma_irq                             = PWMT4_CC4_DMA_IRQ,               \
       .dma_handle_index                    = GPT_DMA_ID_CC4,                  \
       .dma_handle.Init.PeriphDataAlignment = PWMT4_CC4_DMA_PDATAALIGN,        \
       .dma_handle.Init.MemDataAlignment    = PWMT4_CC4_DMA_MDATAALIGN         \
    }
#endif /* BSP_PWMT4_CC4_USING_DMA */


#ifdef BSP_USING_PWMT5
#define PWMT5_CONFIG                                                           \
    {                                                                          \
       .tim_handle.Instance     = GPTIM5,                                      \
       .tim_handle.core         = PWMT5_CORE,                                  \
       .name                    = "pwmt5",                                     \
       .channel                 = 0                                            \
    }
#endif /* BSP_USING_PWMT5 */

#ifdef BSP_PWMT5_UPDATE_USING_DMA
#define PWMT5_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
       .dma_handle.Init.Priority            = PWMT5_UPDATE_DMA_IRQ_PRIO,       \
       .dma_handle.Instance                 = PWMT5_UPDATE_DMA_INSTANCE,       \
       .dma_handle.Init.Request             = PWMT5_UPDATE_DMA_REQUEST,        \
       .dma_irq                             = PWMT5_UPDATE_DMA_IRQ,            \
       .dma_handle_index                    = GPT_DMA_ID_UPDATE,               \
       .dma_handle.Init.PeriphDataAlignment = PWMT5_UPDATE_DMA_PDATAALIGN,     \
       .dma_handle.Init.MemDataAlignment    = PWMT5_UPDATE_DMA_MDATAALIGN      \
    }
#endif /* BSP_PWMT5_UPDATE_USING_DMA */

#ifdef BSP_USING_PWM_LPTIM1
#define PWM_LPTIM1_CONFIG                                                      \
    {                                                                          \
       .tim_handle.Instance     = hwp_lptim1,                                  \
       .name                    = "pwmlp1",                                    \
    }
#endif /* BSP_USING_PWM_LPTIM1 */

#ifdef BSP_USING_PWM_LPTIM2
#define PWM_LPTIM2_CONFIG                                                      \
    {                                                                          \
       .tim_handle.Instance     = hwp_lptim2,                                  \
       .name                    = "pwmlp2",                                    \
    }
#endif /* BSP_USING_PWM_LPTIM2 */

#ifdef BSP_USING_PWM_LPTIM3
#define PWM_LPTIM3_CONFIG                                                      \
    {                                                                          \
       .tim_handle.Instance     = hwp_lptim3,                                  \
       .name                    = "pwmlp3",                                    \
    }
#endif /* BSP_USING_PWM_LPTIM3 */

#ifdef __cplusplus
}
#endif

#endif /* __PWM_CONFIG_H__ */
