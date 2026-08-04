/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <string.h>
#include <stdio.h>
#include "bf0_hal.h"
#include "board.h"
#include "../dfu/dfu.h"


void boot_uart_tx(USART_TypeDef *uart, uint8_t *data, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        while ((uart->ISR & UART_FLAG_TXE) == 0);
        uart->TDR = (uint32_t)data[i];
    }
}

/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{
}

void mpu_config(void)
{
    // Do nothing
}

void cache_enable(void)
{
    // Do nothing
}

