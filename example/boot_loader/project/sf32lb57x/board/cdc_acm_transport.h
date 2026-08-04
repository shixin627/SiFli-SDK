/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CDC_ACM_H__
#define __CDC_ACM_H__

#include <stdint.h>
#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

extern transport_t usb_cdc_acm_transport;

void cdc_acm_init(uint8_t busid, uintptr_t base);

#ifdef __cplusplus
}
#endif

#endif /* __CDC_ACM_H__ */

