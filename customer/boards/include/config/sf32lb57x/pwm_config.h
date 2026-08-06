/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PWM_CONFIG_H__
#define __PWM_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWM1_CORE CORE_ID_HCPU
#define PWMT1_CORE CORE_ID_HCPU
#define PWMT2_CORE CORE_ID_HCPU


#ifdef BSP_USING_PWM1
#define PWM1_CONFIG                             \
    {                                           \
       .pwm_handle.Instance     = hwp_pwm1,       \
       .pwm_handle.core         = PWM1_CORE,    \
       .name                    = "pwm1",       \
       .channel                 = 0             \
    }
#endif /* BSP_USING_PWM1 */

#ifdef BSP_USING_PWMT1
#define PWMT1_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = GPTIM1,       \
       .tim_handle.core         = PWMT1_CORE,    \
       .name                    = "pwmt1",       \
       .channel                 = 0             \
    }
#endif /* BSP_USING_PWMT1 */

#ifdef BSP_USING_PWMT2
#define PWMT2_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = GPTIM2,         \
       .tim_handle.core         = PWMT2_CORE,    \
       .name                    = "pwmt2",       \
       .channel                 = 0             \
    }
#endif /* BSP_USING_PWMT2 */

#ifdef BSP_USING_PWMA1
#define PWMA1_CONFIG                             \
        {                                           \
           .tim_handle.Instance     = (GPT_TypeDef *)ATIM1,         \
           .tim_handle.core         = ATIM1_CORE,    \
           .name                    = "pwma1",       \
           .channel                 = 0             \
        }
#endif /* BSP_USING_PWM_A1 */

#ifdef BSP_USING_PWMA2
#define PWMA2_CONFIG                             \
        {                                           \
           .tim_handle.Instance     = (GPT_TypeDef *)ATIM2,         \
           .tim_handle.core         = ATIM2_CORE,    \
           .name                    = "pwma2",       \
           .channel                 = 0             \
        }
#endif /* BSP_USING_PWM_A2 */

#ifdef BSP_USING_PWM_LPTIM1
#define PWM_LPTIM1_CONFIG                       \
    {                                           \
       .tim_handle.Instance     = hwp_lptim1,   \
       .name                    = "pwmlp1",       \
    }

#endif /* BSP_USING_PWM_LPTIM1 */

#ifdef BSP_USING_PWM_LPTIM2
#define PWM_LPTIM2_CONFIG                       \
    {                                           \
       .tim_handle.Instance     = hwp_lptim2,   \
       .name                    = "pwmlp2",       \
    }
#endif /* BSP_USING_PWM_LPTIM2 */

#ifdef BSP_USING_PWM_LPTIM3
#define PWM_LPTIM3_CONFIG                       \
    {                                           \
       .tim_handle.Instance     = hwp_lptim3,   \
       .name                    = "pwmlp3",       \
    }
#endif /* BSP_USING_PWM_LPTIM3 */


#ifdef __cplusplus
}
#endif

#endif /* __PWM_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
