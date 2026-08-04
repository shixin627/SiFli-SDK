/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtconfig.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "bf0_hal.h"
#include "transport.h"

#define SERIAL_RX_BUFFER_SIZE 256
#define SERIAL_HALF_RX_BUFFER_SIZE (SERIAL_RX_BUFFER_SIZE/2)

#define UART1_DMA_RX_IRQHandler                DMAC1_CH7_IRQHandler
#define UART1_RX_DMA_REQUEST                   DMA_REQUEST_5
#define UART1_RX_DMA_INSTANCE                  DMA1_Channel7

typedef struct
{
    uint16_t wr_index;
    uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];
    uint32_t last_dma_index;
} serial_xfer_ctx_t;

static UART_HandleTypeDef uart_handle;
static DMA_HandleTypeDef dma_rx_handle;
static serial_xfer_ctx_t serial_xfer_ctx;
transport_t serial_transport;

static void serial_send(uint8_t *buf, uint32_t len);

static void serial_send(uint8_t *buf, uint32_t len)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(&uart_handle, buf, len, 0xFFFF);
    HAL_ASSERT(HAL_OK == status);
}

static void serial_update_rx_buf_wr_index(serial_xfer_ctx_t *ctx, uint32_t len)
{
    if ((ctx->wr_index + len) <= SERIAL_RX_BUFFER_SIZE)
    {
        xport_rx_buf_put(&serial_transport, &ctx->rx_buffer[ctx->wr_index], len);
    }
    else
    {
        xport_rx_buf_put(&serial_transport, &ctx->rx_buffer[ctx->wr_index], SERIAL_RX_BUFFER_SIZE - ctx->wr_index);
        xport_rx_buf_put(&serial_transport, &ctx->rx_buffer[0], ctx->wr_index + len - SERIAL_RX_BUFFER_SIZE);
    }

    ctx->wr_index += len;
    if (ctx->wr_index >= SERIAL_RX_BUFFER_SIZE)
    {
        ctx->wr_index -= SERIAL_RX_BUFFER_SIZE;
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t recv_total_index;
    uint32_t recv_len;
    uint32_t mask;

    mask = HAL_DisableInterrupt();
    recv_total_index = SERIAL_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&dma_rx_handle);
    if (recv_total_index < serial_xfer_ctx.last_dma_index)
    {
        recv_len = SERIAL_RX_BUFFER_SIZE + recv_total_index - serial_xfer_ctx.last_dma_index;
    }
    else
    {
        recv_len = recv_total_index - serial_xfer_ctx.last_dma_index;
    }
    serial_xfer_ctx.last_dma_index = recv_total_index;
    if (recv_len)
    {
        serial_update_rx_buf_wr_index(&serial_xfer_ctx, recv_len);
    }

    HAL_EnableInterrupt(mask);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_RxHalfCpltCallback(huart);
}

void USART1_IRQHandler(void)
{
    if ((__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_IDLE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart_handle, UART_IT_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&uart_handle);
        HAL_UART_RxHalfCpltCallback(&uart_handle);
    }
    else
    {
        if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(&uart_handle);
        }
        if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_NE) != RESET)
        {
            __HAL_UART_CLEAR_NEFLAG(&uart_handle);
        }
        if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_FE) != RESET)
        {
            __HAL_UART_CLEAR_FEFLAG(&uart_handle);
        }
        if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_PE) != RESET)
        {
            __HAL_UART_CLEAR_PEFLAG(&uart_handle);
        }
        if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_CTS) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&uart_handle, UART_FLAG_CTS);
        }
        if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_TXE) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&uart_handle, UART_FLAG_TXE);
        }
        if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_TC) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&uart_handle, UART_FLAG_TC);
        }
        if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_RXNE) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&uart_handle, UART_FLAG_RXNE);
        }
    }
}

void UART1_DMA_RX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dma_rx_handle);
}


void serial_transport_init(void)
{
    serial_transport.send = serial_send;

    uart_handle.Instance        = hwp_usart1;
    uart_handle.Init.BaudRate   = 1000000;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.StopBits   = UART_STOPBITS_1;
    uart_handle.Init.Parity     = UART_PARITY_NONE;
    uart_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    uart_handle.Init.Mode       = UART_MODE_TX_RX;
    uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&uart_handle) != HAL_OK)
    {
        /* Initialization Error */
        HAL_ASSERT(0);
    }

    // Start RX DMA
    __HAL_LINKDMA(&(uart_handle), hdmarx, dma_rx_handle);
    dma_rx_handle.Instance = UART1_RX_DMA_INSTANCE;
    dma_rx_handle.Init.Request = UART1_RX_DMA_REQUEST;
    HAL_UART_DmaTransmit(&uart_handle, serial_xfer_ctx.rx_buffer, SERIAL_RX_BUFFER_SIZE, DMA_PERIPH_TO_MEMORY); /* DMA_PERIPH_TO_MEMORY */

    // For RX DMA, also need to enable UART IRQ.
    __HAL_UART_ENABLE_IT(&uart_handle, UART_IT_IDLE); /* Set to generates interrupts when UART idle */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

}
