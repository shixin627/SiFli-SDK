/**
 ******************************************************************************
 * @file   communicate_update_image.h
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __COMMUNICATE_UPDATE_IMAGE_H__
#define __COMMUNICATE_UPDATE_IMAGE_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include "stdint.h"
#include "stdbool.h"
#include "bloc_flash.h"

    extern void mark_ota_started(void);
    extern void init_ble_dfu_thread(dfu_img_id_t id, uint32_t dest_addr, uint32_t size);
    extern void stop_ble_dfu_thread(void);
    extern void verify_and_upgrade_dfu_image(void);
    extern void set_ble_dfu_start(void);
    extern void set_ble_dfu_thread_run(int run);
    extern void handle_ble_dfu_data(uint8_t *buf, uint16_t len);
    extern uint32_t get_cur_watch_image_size(void);
    extern bool is_ble_dfu_thread_running(void);

#ifdef PKG_USING_LZ4
    // LZ4 compression related functions
    extern void init_ble_dfu_thread_compressed(dfu_img_id_t id, uint32_t dest_addr, uint32_t original_size, uint32_t compressed_size);
#endif

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/