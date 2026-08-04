/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-27     thread-liu   first version
 */

#ifndef __DRV_OV2640_H__
#define __DRV_OV2640_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <board.h>
int rt_gc032a_init(uint8_t interface);
int rt_gc032a_deinit(void);
#ifdef __cplusplus
}
#endif

#endif
