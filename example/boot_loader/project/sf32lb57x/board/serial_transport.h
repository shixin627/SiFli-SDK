/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SERIAL_TRANSPORT_H__
#define __SERIAL_TRANSPORT_H__

#include <stdint.h>
#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

extern transport_t serial_transport;

void serial_transport_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_TRANSPORT_H__ */

