/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* Sifli Configuration */

#define SOC_SF32LB57X 1

#define CORE "HCPU"
#define CPU "Cortex-M33"

/* RTOS */

#define BSP_USING_NO_OS
#define RT_USING_LIBC
#define BSP_USING_MPI 1
#define BSP_USING_QSPI 1
#define BSP_USING_SPI_FLASH 1
#define BSP_ENABLE_MPI1 1
#define BSP_ENABLE_QSPI1 1
#define BSP_MPI1_MODE_0 1
#define BSP_QSPI1_MODE 0
#define BSP_USING_NOR_FLASH1 1
#define BSP_QSPI1_USING_DMA 1
#define BSP_QSPI1_MEM_SIZE 8
#ifdef JLINK_FLASH_2
#define BSP_ENABLE_MPI2 1
#define BSP_ENABLE_QSPI2 1
#define BSP_MPI2_MODE_0 1
#define BSP_QSPI2_MODE 0
#define BSP_USING_NOR_FLASH2 1
#define BSP_QSPI2_USING_DMA 1
#define BSP_QSPI2_MEM_SIZE 32
#endif /* JLINK_FLASH_2 */
#ifdef JLINK_FLASH_3
#define BSP_ENABLE_MPI3 1
#define BSP_ENABLE_QSPI3 1
#define BSP_MPI3_MODE_0 1
#define BSP_QSPI3_MODE 0
#define BSP_USING_NOR_FLASH3 1
#define BSP_QSPI3_USING_DMA 1
#define BSP_QSPI3_MEM_SIZE 32
#endif /* JLINK_FLASH_3 */
#ifdef JLINK_FLASH_4
#define BSP_ENABLE_MPI4 1
#define BSP_ENABLE_QSPI4 1
#define BSP_MPI4_MODE_0 1
#define BSP_QSPI4_MODE 0
#define BSP_USING_NOR_FLASH4 1
#define BSP_QSPI4_USING_DMA 1
#define BSP_QSPI4_MEM_SIZE 32
#endif /* JLINK_FLASH_4 */
/* Select board peripherals */

#define BSP_USING_BOARD_FPGA_A0

/* Key config */

/* Sifli Middleware */

#define BSP_USING_EMPTY_ASSERT

/* RW Bluetooth */


/* Third party packages */

#define BF0_HCPU
//#define USE_V2_MEM  1

#define rt_hw_interrupt_enable(level)
#define rt_hw_interrupt_disable()  0
#define RT_EOK      0
#define RT_ERROR 1

#endif
