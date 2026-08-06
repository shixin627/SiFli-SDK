/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal_def.h"
#include "bf0_pin_const.h"

/* PAD_SA00 */
const pin_fsel_function_t pad_sa00_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DM},
    {4, MPI1_FLASH_CS},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA01 */
const pin_fsel_function_t pad_sa01_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DIO0},
    {4, MPI1_FLASH_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA02 */
const pin_fsel_function_t pad_sa02_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DIO1},
    {4, MPI1_FLASH_DIO1},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA03 */
const pin_fsel_function_t pad_sa03_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DIO2},
    {4, MPI1_FLASH_CS},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA04 */
const pin_fsel_function_t pad_sa04_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DIO3},
    {4, MPI1_FLASH_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA05 */
const pin_fsel_function_t pad_sa05_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_CS},
    {3, MPI1_PSRAM_DIO4},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA06 */
const pin_fsel_function_t pad_sa06_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_CLKB},
    {3, MPI1_PSRAM_DIO5},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA07 */
const pin_fsel_function_t pad_sa07_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_CLK},
    {3, MPI1_PSRAM_DIO6},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA08 */
const pin_fsel_function_t pad_sa08_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DIO4},
    {3, MPI1_PSRAM_DIO7},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA09 */
const pin_fsel_function_t pad_sa09_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DIO5},
    {3, MPI1_PSRAM_DQSDM},
    {4, MPI1_FLASH_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA10 */
const pin_fsel_function_t pad_sa10_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DIO6},
    {4, MPI1_FLASH_CLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA11 */
const pin_fsel_function_t pad_sa11_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DIO7},
    {3, MPI1_PSRAM_CLK},
    {4, MPI1_FLASH_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SA12 */
const pin_fsel_function_t pad_sa12_fsel_func_tbl[] =
{
    {1, MPI1_PSRAM_DQS},
    {2, MPI1_PSRAM_DQSDM},
    {3, MPI1_PSRAM_CS},
    {4, MPI1_FLASH_DIO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB00 */
const pin_fsel_function_t pad_sb00_fsel_func_tbl[] =
{
    {1, MPI2_DM},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB01 */
const pin_fsel_function_t pad_sb01_fsel_func_tbl[] =
{
    {1, MPI2_DIO0},
    {4, MPI3_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB02 */
const pin_fsel_function_t pad_sb02_fsel_func_tbl[] =
{
    {1, MPI2_DIO1},
    {4, MPI3_DIO1},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB03 */
const pin_fsel_function_t pad_sb03_fsel_func_tbl[] =
{
    {1, MPI2_DIO2},
    {4, MPI3_CS},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB04 */
const pin_fsel_function_t pad_sb04_fsel_func_tbl[] =
{
    {1, MPI2_DIO3},
    {4, MPI2_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB05 */
const pin_fsel_function_t pad_sb05_fsel_func_tbl[] =
{
    {1, MPI2_CS},
    {4, MPI2_DIO1},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB06 */
const pin_fsel_function_t pad_sb06_fsel_func_tbl[] =
{
    {1, MPI2_CLKB},
    {3, MPI2_DIO4},
    {4, MPI2_CS},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB07 */
const pin_fsel_function_t pad_sb07_fsel_func_tbl[] =
{
    {1, MPI2_CLK},
    {3, MPI2_DIO5},
    {4, MPI3_DIO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB08 */
const pin_fsel_function_t pad_sb08_fsel_func_tbl[] =
{
    {1, MPI2_DIO4},
    {3, MPI2_DIO6},
    {4, MPI3_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB09 */
const pin_fsel_function_t pad_sb09_fsel_func_tbl[] =
{
    {1, MPI2_DIO5},
    {3, MPI2_DIO7},
    {4, MPI3_CLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB10 */
const pin_fsel_function_t pad_sb10_fsel_func_tbl[] =
{
    {1, MPI2_DIO6},
    {3, MPI2_DQSDM},
    {4, MPI2_DIO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB11 */
const pin_fsel_function_t pad_sb11_fsel_func_tbl[] =
{
    {1, MPI2_DIO7},
    {3, MPI2_CLK},
    {4, MPI2_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_SB12 */
const pin_fsel_function_t pad_sb12_fsel_func_tbl[] =
{
    {1, MPI2_DQS},
    {2, MPI2_DQSDM},
    {3, MPI2_CS},
    {4, MPI2_CLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA00 */
const pin_fsel_function_t pad_pa00_fsel_func_tbl[] =
{
    {0, GPIO_A0},
    {1, LCDC1_SPI_RSTB},
    {3, SD2_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA01 */
const pin_fsel_function_t pad_pa01_fsel_func_tbl[] =
{
    {0, GPIO_A1},
    {3, SD2_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA02 */
const pin_fsel_function_t pad_pa02_fsel_func_tbl[] =
{
    {0, GPIO_A2},
    {1, LCDC1_SPI_TE},
    {3, SD2_CLK},
    {7, DBG_DO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA03 */
const pin_fsel_function_t pad_pa03_fsel_func_tbl[] =
{
    {0, GPIO_A3},
    {1, LCDC1_SPI_CS},
    {3, SD2_CMD},
    {7, DBG_DO1},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA04 */
const pin_fsel_function_t pad_pa04_fsel_func_tbl[] =
{
    {0, GPIO_A4},
    {1, LCDC1_SPI_CLK},
    {3, SD2_DIO0},
    {7, DBG_DO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA05 */
const pin_fsel_function_t pad_pa05_fsel_func_tbl[] =
{
    {0, GPIO_A5},
    {1, LCDC1_SPI_DIO0},
    {3, SD2_DIO1},
    {7, DBG_DO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA06 */
const pin_fsel_function_t pad_pa06_fsel_func_tbl[] =
{
    {0, GPIO_A6},
    {1, LCDC1_SPI_DIO1},
    {2, SD1_DIO2},
    {7, DBG_DO4},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA07 */
const pin_fsel_function_t pad_pa07_fsel_func_tbl[] =
{
    {0, GPIO_A7},
    {1, LCDC1_SPI_DIO2},
    {2, SD1_DIO3},
    {7, DBG_DO5},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA08 */
const pin_fsel_function_t pad_pa08_fsel_func_tbl[] =
{
    {0, GPIO_A8},
    {1, LCDC1_SPI_DIO3},
    {2, SD1_CLK},
    {7, DBG_DO6},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA09 */
const pin_fsel_function_t pad_pa09_fsel_func_tbl[] =
{
    {0, GPIO_A9},
    {2, SD1_CMD},
    {7, DBG_DO7},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA10 */
const pin_fsel_function_t pad_pa10_fsel_func_tbl[] =
{
    {0, GPIO_A10},
    {2, SD1_DIO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA11 */
const pin_fsel_function_t pad_pa11_fsel_func_tbl[] =
{
    {0, GPIO_A11},
    {1, MPI3_CS2},
    {2, SD1_DIO1},
    {7, AUD_CLK_EXT},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA12 */
const pin_fsel_function_t pad_pa12_fsel_func_tbl[] =
{
    {0, GPIO_A12},
    {1, MPI3_CS},
    {2, SD1_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA13 */
const pin_fsel_function_t pad_pa13_fsel_func_tbl[] =
{
    {0, GPIO_A13},
    {1, MPI3_DIO1},
    {2, SD1_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA14 */
const pin_fsel_function_t pad_pa14_fsel_func_tbl[] =
{
    {0, GPIO_A14},
    {1, MPI3_DIO2},
    {2, SD1_CLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA15 */
const pin_fsel_function_t pad_pa15_fsel_func_tbl[] =
{
    {0, GPIO_A15},
    {1, MPI3_DIO0},
    {2, SD1_CMD},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA16 */
const pin_fsel_function_t pad_pa16_fsel_func_tbl[] =
{
    {0, GPIO_A16},
    {1, MPI3_CLK},
    {2, SD1_DIO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA17 */
const pin_fsel_function_t pad_pa17_fsel_func_tbl[] =
{
    {0, GPIO_A17},
    {1, MPI3_DIO3},
    {2, SD1_DIO1},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA18 */
const pin_fsel_function_t pad_pa18_fsel_func_tbl[] =
{
    {0, GPIO_A18},
    {1, USART1_RXD},
    {2, SWCLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA19 */
const pin_fsel_function_t pad_pa19_fsel_func_tbl[] =
{
    {0, GPIO_A19},
    {1, USART1_TXD},
    {2, SWDIO},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA20 */
const pin_fsel_function_t pad_pa20_fsel_func_tbl[] =
{
    {0, GPIO_A20},
    {1, SWO},
    {7, DBG_CLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA21 */
const pin_fsel_function_t pad_pa21_fsel_func_tbl[] =
{
    {0, GPIO_A21},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA22 */
const pin_fsel_function_t pad_pa22_fsel_func_tbl[] =
{
    {0, GPIO_A22},
    {2, SD1_DIO2},
    {3, SD2_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA23 */
const pin_fsel_function_t pad_pa23_fsel_func_tbl[] =
{
    {0, GPIO_A23},
    {2, SD1_DIO3},
    {3, SD2_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA24 */
const pin_fsel_function_t pad_pa24_fsel_func_tbl[] =
{
    {0, GPIO_A24},
    {1, SPI1_DIO},
    {2, SD1_CLK},
    {3, SD2_CLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA25 */
const pin_fsel_function_t pad_pa25_fsel_func_tbl[] =
{
    {0, GPIO_A25},
    {1, SPI1_DI},
    {2, SD1_CMD},
    {3, SD2_CMD},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA26 */
const pin_fsel_function_t pad_pa26_fsel_func_tbl[] =
{
    {0, GPIO_A26},
    {2, SD1_DIO0},
    {3, SD2_DIO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA27 */
const pin_fsel_function_t pad_pa27_fsel_func_tbl[] =
{
    {0, GPIO_A27},
    {2, SD1_DIO1},
    {3, SD2_DIO1},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA28 */
const pin_fsel_function_t pad_pa28_fsel_func_tbl[] =
{
    {0, GPIO_A28},
    {1, SPI1_CLK},
    {3, SD2_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA29 */
const pin_fsel_function_t pad_pa29_fsel_func_tbl[] =
{
    {0, GPIO_A29},
    {1, SPI1_CS},
    {3, SD2_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA30 */
const pin_fsel_function_t pad_pa30_fsel_func_tbl[] =
{
    {0, GPIO_A30},
    {3, SD2_CLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA31 */
const pin_fsel_function_t pad_pa31_fsel_func_tbl[] =
{
    {0, GPIO_A31},
    {3, SD2_CMD},
    {7, DBG_DO8},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA32 */
const pin_fsel_function_t pad_pa32_fsel_func_tbl[] =
{
    {0, GPIO_A32},
    {3, SD2_DIO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA33 */
const pin_fsel_function_t pad_pa33_fsel_func_tbl[] =
{
    {0, GPIO_A33},
    {3, SD2_DIO1},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA34 */
const pin_fsel_function_t pad_pa34_fsel_func_tbl[] =
{
    {0, GPIO_A34},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA35 */
const pin_fsel_function_t pad_pa35_fsel_func_tbl[] =
{
    {0, GPIO_A35},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA36 */
const pin_fsel_function_t pad_pa36_fsel_func_tbl[] =
{
    {0, GPIO_A36},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA37 */
const pin_fsel_function_t pad_pa37_fsel_func_tbl[] =
{
    {0, GPIO_A37},
    {1, SPI2_DIO},
    {2, SD1_DIO2},
    {7, DBG_DO9},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA38 */
const pin_fsel_function_t pad_pa38_fsel_func_tbl[] =
{
    {0, GPIO_A38},
    {1, SPI2_DI},
    {2, SD1_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA39 */
const pin_fsel_function_t pad_pa39_fsel_func_tbl[] =
{
    {0, GPIO_A39},
    {1, SPI2_CLK},
    {2, SD1_CLK},
    {7, DBG_DO10},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA40 */
const pin_fsel_function_t pad_pa40_fsel_func_tbl[] =
{
    {0, GPIO_A40},
    {1, SPI2_CS},
    {2, SD1_CMD},
    {7, DBG_DO11},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA41 */
const pin_fsel_function_t pad_pa41_fsel_func_tbl[] =
{
    {0, GPIO_A41},
    {2, SD1_DIO0},
    {7, DBG_DO12},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA42 */
const pin_fsel_function_t pad_pa42_fsel_func_tbl[] =
{
    {0, GPIO_A42},
    {2, SD1_DIO1},
    {7, DBG_DO13},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA43 */
const pin_fsel_function_t pad_pa43_fsel_func_tbl[] =
{
    {0, GPIO_A43},
    {1, SWCLK},
    {7, DBG_DO14},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA44 */
const pin_fsel_function_t pad_pa44_fsel_func_tbl[] =
{
    {0, GPIO_A44},
    {1, SWDIO},
    {7, DBG_DO15},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA45 */
const pin_fsel_function_t pad_pa45_fsel_func_tbl[] =
{
    {0, GPIO_A45},
    {3, SD2_DIO2},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA46 */
const pin_fsel_function_t pad_pa46_fsel_func_tbl[] =
{
    {0, GPIO_A46},
    {3, SD2_DIO3},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA47 */
const pin_fsel_function_t pad_pa47_fsel_func_tbl[] =
{
    {0, GPIO_A47},
    {3, SD2_CLK},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA48 */
const pin_fsel_function_t pad_pa48_fsel_func_tbl[] =
{
    {0, GPIO_A48},
    {3, SD2_CMD},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA49 */
const pin_fsel_function_t pad_pa49_fsel_func_tbl[] =
{
    {0, GPIO_A49},
    {3, SD2_DIO0},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA50 */
const pin_fsel_function_t pad_pa50_fsel_func_tbl[] =
{
    {0, GPIO_A50},
    {3, SD2_DIO1},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA51 */
const pin_fsel_function_t pad_pa51_fsel_func_tbl[] =
{
    {0, GPIO_A51},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA52 */
const pin_fsel_function_t pad_pa52_fsel_func_tbl[] =
{
    {0, GPIO_A52},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA53 */
const pin_fsel_function_t pad_pa53_fsel_func_tbl[] =
{
    {0, GPIO_A53},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA54 */
const pin_fsel_function_t pad_pa54_fsel_func_tbl[] =
{
    {0, GPIO_A54},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA55 */
const pin_fsel_function_t pad_pa55_fsel_func_tbl[] =
{
    {0, GPIO_A55},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA56 */
const pin_fsel_function_t pad_pa56_fsel_func_tbl[] =
{
    {0, GPIO_A56},
    {0, PIN_FUNC_UNDEF},
};

/* PAD_PA57 */
const pin_fsel_function_t pad_pa57_fsel_func_tbl[] =
{
    {0, GPIO_A57},
    {0, PIN_FUNC_UNDEF},
};

const pin_fsel_function_t *const pad_fsel_func_tbls[HPSYS_PAD_NUM] =
{
    pad_sa00_fsel_func_tbl,
    pad_sa01_fsel_func_tbl,
    pad_sa02_fsel_func_tbl,
    pad_sa03_fsel_func_tbl,
    pad_sa04_fsel_func_tbl,
    pad_sa05_fsel_func_tbl,
    pad_sa06_fsel_func_tbl,
    pad_sa07_fsel_func_tbl,
    pad_sa08_fsel_func_tbl,
    pad_sa09_fsel_func_tbl,
    pad_sa10_fsel_func_tbl,
    pad_sa11_fsel_func_tbl,
    pad_sa12_fsel_func_tbl,
    pad_sb00_fsel_func_tbl,
    pad_sb01_fsel_func_tbl,
    pad_sb02_fsel_func_tbl,
    pad_sb03_fsel_func_tbl,
    pad_sb04_fsel_func_tbl,
    pad_sb05_fsel_func_tbl,
    pad_sb06_fsel_func_tbl,
    pad_sb07_fsel_func_tbl,
    pad_sb08_fsel_func_tbl,
    pad_sb09_fsel_func_tbl,
    pad_sb10_fsel_func_tbl,
    pad_sb11_fsel_func_tbl,
    pad_sb12_fsel_func_tbl,
    pad_pa00_fsel_func_tbl,
    pad_pa01_fsel_func_tbl,
    pad_pa02_fsel_func_tbl,
    pad_pa03_fsel_func_tbl,
    pad_pa04_fsel_func_tbl,
    pad_pa05_fsel_func_tbl,
    pad_pa06_fsel_func_tbl,
    pad_pa07_fsel_func_tbl,
    pad_pa08_fsel_func_tbl,
    pad_pa09_fsel_func_tbl,
    pad_pa10_fsel_func_tbl,
    pad_pa11_fsel_func_tbl,
    pad_pa12_fsel_func_tbl,
    pad_pa13_fsel_func_tbl,
    pad_pa14_fsel_func_tbl,
    pad_pa15_fsel_func_tbl,
    pad_pa16_fsel_func_tbl,
    pad_pa17_fsel_func_tbl,
    pad_pa18_fsel_func_tbl,
    pad_pa19_fsel_func_tbl,
    pad_pa20_fsel_func_tbl,
    pad_pa21_fsel_func_tbl,
    pad_pa22_fsel_func_tbl,
    pad_pa23_fsel_func_tbl,
    pad_pa24_fsel_func_tbl,
    pad_pa25_fsel_func_tbl,
    pad_pa26_fsel_func_tbl,
    pad_pa27_fsel_func_tbl,
    pad_pa28_fsel_func_tbl,
    pad_pa29_fsel_func_tbl,
    pad_pa30_fsel_func_tbl,
    pad_pa31_fsel_func_tbl,
    pad_pa32_fsel_func_tbl,
    pad_pa33_fsel_func_tbl,
    pad_pa34_fsel_func_tbl,
    pad_pa35_fsel_func_tbl,
    pad_pa36_fsel_func_tbl,
    pad_pa37_fsel_func_tbl,
    pad_pa38_fsel_func_tbl,
    pad_pa39_fsel_func_tbl,
    pad_pa40_fsel_func_tbl,
    pad_pa41_fsel_func_tbl,
    pad_pa42_fsel_func_tbl,
    pad_pa43_fsel_func_tbl,
    pad_pa44_fsel_func_tbl,
    pad_pa45_fsel_func_tbl,
    pad_pa46_fsel_func_tbl,
    pad_pa47_fsel_func_tbl,
    pad_pa48_fsel_func_tbl,
    pad_pa49_fsel_func_tbl,
    pad_pa50_fsel_func_tbl,
    pad_pa51_fsel_func_tbl,
    pad_pa52_fsel_func_tbl,
    pad_pa53_fsel_func_tbl,
    pad_pa54_fsel_func_tbl,
    pad_pa55_fsel_func_tbl,
    pad_pa56_fsel_func_tbl,
    pad_pa57_fsel_func_tbl,
};
