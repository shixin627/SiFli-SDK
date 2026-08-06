/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spi_tst_drv.h"

void set_spi1_clk_freq(uint8_t freq)
{

    hwp_spi1->TOP_CTRL &=  ~SPI_TOP_CTRL_CLK_DIV ;
    hwp_spi1->TOP_CTRL |=  freq << SPI_TOP_CTRL_CLK_DIV_Pos ;
    hwp_spi1->TOP_CTRL &=  ~SPI_TOP_CTRL_CLK_SEL ;
    if (freq == 1)
        hwp_spi1->TOP_CTRL |=  0x1 << SPI_TOP_CTRL_CLK_SEL_Pos;
    hwp_spi1->TOP_CTRL |=  0x1 << SPI_TOP_CTRL_CLK_SSP_EN_Pos ;
}

void enable_spi1_clk()
{
    hwp_spi1->TOP_CTRL |=  0x1 << SPI_TOP_CTRL_CLK_SSP_EN_Pos ;
}

void set_spi2_clk_freq(uint8_t freq)
{

    hwp_spi2->TOP_CTRL &=  ~SPI_TOP_CTRL_CLK_DIV ;
    hwp_spi2->TOP_CTRL |=  freq << SPI_TOP_CTRL_CLK_DIV_Pos ;
    hwp_spi2->TOP_CTRL &=  ~SPI_TOP_CTRL_CLK_SEL ;
    if (freq == 1)
        hwp_spi2->TOP_CTRL |=  0x1 << SPI_TOP_CTRL_CLK_SEL_Pos;
    hwp_spi2->TOP_CTRL |=  0x1 << SPI_TOP_CTRL_CLK_SSP_EN_Pos ;
}

void enable_spi2_clk()
{
    hwp_spi2->TOP_CTRL |=  0x1 << SPI_TOP_CTRL_CLK_SSP_EN_Pos ;
}

void enable_spi1_trx()
{
    hwp_spi1->TOP_CTRL |= SPI_TOP_CTRL_SSE;
    set_spi1_clkdiv(0x4);
    hwp_spi1->TOP_CTRL |= 0x1 << SPI_TOP_CTRL_CLK_SSP_EN_Pos;
}

void disable_spi1_trx()
{
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_SSE_Msk;
}

void set_spi1_tdata(uint32_t tdata)
{
    hwp_spi1->DATA = tdata ;
}

uint32_t get_spi1_rdata()
{
    return hwp_spi1->DATA;
}

void set_spi1_frm_width(uint8_t frm_width)
{
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_DSS_Msk;
    hwp_spi1->TOP_CTRL |= ((frm_width - 1) << SPI_TOP_CTRL_DSS_Pos);
}

void set_spi1_sph()
{
    hwp_spi1->TOP_CTRL |= SPI_TOP_CTRL_SPH;
}

void clear_spi1_sph()
{
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_SPH_Msk;
}

void set_spi1_spo()
{
    hwp_spi1->TOP_CTRL |= SPI_TOP_CTRL_SPO;

}

void clear_spi1_spo()
{
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_SPO_Msk;

}

void set_spi1_clkdiv(uint8_t div)
{
    hwp_spi1->TOP_CTRL &= (~SPI_TOP_CTRL_CLK_DIV_Msk);
    hwp_spi1->TOP_CTRL |= (div << SPI_TOP_CTRL_CLK_DIV_Pos);
}

