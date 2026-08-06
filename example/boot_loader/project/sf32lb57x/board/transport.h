/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TRANSPORT_H__
#define __TRANSPORT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RX_DATA_BUF_SIZE    (8192)

typedef struct
{
    uint32_t total_size;
    uint32_t wr_pos;
    uint32_t rd_pos;
    uint8_t buf[RX_DATA_BUF_SIZE];
} rx_data_buf_t;


typedef struct
{
    rx_data_buf_t rx_data_buf;
    void (*send)(uint8_t *buf, uint32_t len);
} transport_t;


uint32_t xport_rx_buf_put(transport_t *transport, uint8_t *buf, uint32_t len);
uint32_t xport_rx_buf_get(transport_t *transport, uint8_t *buf, uint32_t len);
uint32_t xport_rx_buf_len(transport_t *transport);
void xport_send(transport_t *transport, uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __TRANSPORT_H__ */

