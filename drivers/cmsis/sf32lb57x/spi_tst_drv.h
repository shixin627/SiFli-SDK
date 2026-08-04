/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>             // for memcpy
//#include <rtthread.h>
//#include <rtdevice.h>
#include <stdlib.h>
#include <board.h>



void enable_spi1_trx() ;

void disable_spi1_trx() ;

void set_spi1_tdata(uint32_t tdata) ;

uint32_t get_spi1_rdata() ;

void set_spi1_frm_width(uint8_t frm_width) ;

void set_spi1_sph() ;

void clear_spi1_sph();

void set_spi1_spo();

void clear_spi1_spo();

void set_spi1_clkdiv(uint8_t div);

