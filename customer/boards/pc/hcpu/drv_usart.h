/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * PC simulator stub of drv_usart.h. virtual_console.c includes it but only
 * dereferences ARM-specific fields under #ifdef SOC_BF0_HCPU paths gated by
 * BSP_USING_VIRTUAL_CONSOLE; on PC the file's PC-side branch (rt_kprintf
 * forwarding) doesn't need anything from here.
 */
#ifndef __DRV_USART_H__
#define __DRV_USART_H__

#include "rtthread.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif
