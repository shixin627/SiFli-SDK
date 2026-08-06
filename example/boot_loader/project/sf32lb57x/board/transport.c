/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtconfig.h"
#include <string.h>
#include <stdio.h>
#include "bf0_hal.h"
#include "transport.h"


uint32_t xport_rx_buf_put(transport_t *transport, uint8_t *buf, uint32_t len)
{
    uint32_t wr_size;
    uint32_t level;
    rx_data_buf_t *rx_data_buf;

    HAL_ASSERT(transport && buf);
    rx_data_buf = &transport->rx_data_buf;

    if ((rx_data_buf->total_size + len) > RX_DATA_BUF_SIZE)
    {
        printf("rx_data_buf full: %d, %d\n", rx_data_buf->total_size, len);
        len = RX_DATA_BUF_SIZE - rx_data_buf->total_size;
    }
    if (len == 0)
    {
        return 0;
    }

    if (rx_data_buf->wr_pos + len > RX_DATA_BUF_SIZE)
    {
        wr_size = RX_DATA_BUF_SIZE - rx_data_buf->wr_pos;
        memcpy(&rx_data_buf->buf[rx_data_buf->wr_pos], buf, wr_size);
        memcpy(&rx_data_buf->buf[0], &buf[wr_size], len - wr_size);

        level = HAL_DisableInterrupt();
        rx_data_buf->wr_pos = len - wr_size;
        rx_data_buf->total_size += len;
        HAL_EnableInterrupt(level);
    }
    else
    {
        memcpy(&rx_data_buf->buf[rx_data_buf->wr_pos], buf, len);
        level = HAL_DisableInterrupt();
        rx_data_buf->wr_pos += len;
        if (rx_data_buf->wr_pos >= RX_DATA_BUF_SIZE)
        {
            rx_data_buf->wr_pos = 0;
        }
        rx_data_buf->total_size += len;
        HAL_EnableInterrupt(level);
    }

    return len;
}

uint32_t xport_rx_buf_get(transport_t *transport, uint8_t *buf, uint32_t len)
{
    uint32_t rd_size;
    uint32_t level;
    rx_data_buf_t *rx_data_buf;

    HAL_ASSERT(transport && buf);

    rx_data_buf = &transport->rx_data_buf;
    if (len > rx_data_buf->total_size)
    {
        len = rx_data_buf->total_size;
    }
    if (len == 0)
    {
        return 0;
    }

    if (rx_data_buf->rd_pos + len > RX_DATA_BUF_SIZE)
    {
        rd_size = RX_DATA_BUF_SIZE - rx_data_buf->rd_pos;
        memcpy(buf, &rx_data_buf->buf[rx_data_buf->rd_pos], rd_size);
        memcpy(&buf[rd_size], &rx_data_buf->buf[0], len - rd_size);

        level = HAL_DisableInterrupt();
        rx_data_buf->rd_pos = len - rd_size;
        rx_data_buf->total_size -= len;
        HAL_EnableInterrupt(level);
    }
    else
    {
        memcpy(buf, &rx_data_buf->buf[rx_data_buf->rd_pos], len);
        level = HAL_DisableInterrupt();
        rx_data_buf->rd_pos += len;
        if (rx_data_buf->rd_pos >= RX_DATA_BUF_SIZE)
        {
            rx_data_buf->rd_pos = 0;
        }
        rx_data_buf->total_size -= len;
        HAL_EnableInterrupt(level);
    }

    return len;
}

uint32_t xport_rx_buf_len(transport_t *transport)
{
    HAL_ASSERT(transport);

    return transport->rx_data_buf.total_size;
}

void xport_send(transport_t *transport, uint8_t *buf, uint32_t len)
{
    HAL_ASSERT(transport);
    HAL_ASSERT(transport->send);

    transport->send(buf, len);
}

