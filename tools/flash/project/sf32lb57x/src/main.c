
/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "flash_table.h"


UART_HandleTypeDef UartHandle;

static uint16_t flash1_div = 2;
static uint16_t flash2_div = 2;
static uint16_t flash3_div = 2;
static uint16_t flash4_div = 2;

static void SystemClock_Config(void);
static void Error_Handler(void);

#if 0
void uart_send_char(char c)
{
    hwp_usart4->CR1 |= (0x2UL << USART_CR1_M_Pos);   // set M = 2, data = 8-bit
    hwp_usart4->CR1 |= USART_CR1_UE;
    hwp_usart4->CR1 |= USART_CR1_TE;

    hwp_usart4->TDR = c;
    while (!(hwp_usart4->ISR & USART_ISR_TXE_Msk));
}
#endif

void debug_print(char *str)
{
#if 0
    char *s;
    
    s = str;
    while (*s != 0)
    {
        uart_send_char(*s);
        s++;
    }
#endif        
    HAL_UART_Transmit(&UartHandle, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}


uint16_t BSP_GetFlash1DIV(void)
{
    return flash1_div;
}

uint16_t BSP_GetFlash2DIV(void)
{
    return flash2_div;
}

uint16_t BSP_GetFlash3DIV(void)
{
    return flash3_div;
}

uint16_t BSP_GetFlash4DIV(void)
{
    return flash4_div;
}

void BSP_SetFlash1DIV(uint16_t div)
{
    flash1_div = div;
}

void BSP_SetFlash2DIV(uint16_t div)
{
    flash2_div = div;
}

void BSP_SetFlash3DIV(uint16_t div)
{
    flash3_div = div;
}

void BSP_SetFlash4DIV(uint16_t div)
{
    flash4_div = div;
}

static void JLINK_DRV_BSP_PIN_Init(void)
{
    //MODIFY_REG(hwp_qspi1->WDTR, QSPI_WDTR_TIMEOUT_Msk, QSPI_WDTR_TIMEOUT_Msk);
    //MODIFY_REG(hwp_qspi2->WDTR, QSPI_WDTR_TIMEOUT_Msk, QSPI_WDTR_TIMEOUT_Msk);
    //MODIFY_REG(hwp_qspi3->WDTR, QSPI_WDTR_TIMEOUT_Msk, QSPI_WDTR_TIMEOUT_Msk);

#ifdef JLINK_FLASH_1
    //TODO
    HAL_PIN_Set(PAD_SA01, MPI1_CS,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA09, MPI1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA00, MPI1_DIO2, PIN_PULLUP, 1);
#endif

#ifdef JLINK_FLASH_2
    HAL_PIN_Set(PAD_SB12, MPI2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB06, MPI2_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB10, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB05, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB04, MPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_SB11, MPI2_DIO3, PIN_PULLUP, 1);
#endif

#ifdef JLINK_FLASH_3
    HAL_PIN_Set(PAD_PA16, MPI3_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA12, MPI3_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA15, MPI3_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, MPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, MPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA17, MPI3_DIO3, PIN_PULLUP, 1);
#endif

#ifdef JLINK_FLASH_4
    HAL_PIN_Set(PAD_PA16, MPI4_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA12, MPI4_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA15, MPI4_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, MPI4_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, MPI4_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA17, MPI4_DIO3, PIN_PULLUP, 1);
#endif

    HAL_PIN_Set(PAD_PA19, USART1_TXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA18, USART1_RXD, PIN_PULLUP, 1);

    //__HAL_WDT_DISABLE();
    __HAL_IWDT_DISABLE();

    // delay 6ms to wait for flash power stable
    //HAL_Delay_us(0);
    //HAL_Delay_us(50000);
    {
        uint32_t sysFreq = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
        volatile uint32_t lootMs = 20 * (sysFreq / 1000) / 5;

        if (lootMs == 0)
        {
            lootMs = 480000;
        }

        while (lootMs-- > 0);
    }
}


void HAL_MspInit(void)
{
    
    spi_nor_table_init();
    spi_nand_table_init();

    JLINK_DRV_BSP_PIN_Init();

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
        - Word Length = 8 Bits (7 data bit + 1 parity bit) :
       BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
        - Stop Bit    = One Stop bit
        - Parity      = ODD parity
        - BaudRate    = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    memset(&UartHandle, 0, sizeof(UART_HandleTypeDef));
    UartHandle.Instance        = USART1;
    UartHandle.Init.BaudRate   = 1000000;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        HAL_ASSERT(0);
    }
    //debug_print("Init\r\n");
}


HAL_StatusTypeDef HAL_EFUSE_Init()
{
    return HAL_OK;
}

int BSP_CONFIG_get(int type, uint8_t *buf, int length)
{
    return 0;
}


//HAL_StatusTypeDef HAL_HPAON_StartGTimer(void)
//{
//    return HAL_OK;
//}

/* Private functions ---------------------------------------------------------*/

#ifdef JLINK
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    /* HAL library initialization:
         - Configure the Flash prefetch, instruction and Data caches
         - Systick timer is configured by default as source of time base, but user
           can eventually implement his proper time base source (a general purpose
           timer for example or other time source), keeping in mind that Time base
           duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
           handled in milliseconds basis.
         - Set NVIC Group Priority to 4
         - Low Level Initialization: global MSP (MCU Support Package) initialization
       */
    /* Initialize system resurces, such as MPU, SCB, efuse etc. */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

uint32_t SystemCoreClock = 48000000;

void SystemInit(void)
{
}

pm_power_on_mode_t SystemPowerOnModeGet(void)
{
    return PM_COLD_BOOT;
}

/**
  * @}
  */

/**
  * @}
  */
