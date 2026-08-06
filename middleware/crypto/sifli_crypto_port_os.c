/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include <stdint.h>
#include "sifli_crypto_port.h"
#include "drv_aes.h"

sf_crypto_err_t sf_crypto_aes_enc_sync(const uint32_t *key, uint32_t key_size, const uint32_t *iv, uint8_t mode,
                                       uint8_t *input, uint32_t input_size, uint8_t *output)
{
    rt_err_t err;
    AES_KeyTypeDef cfg;
    AES_IOTypeDef data;

    cfg.key = (uint32_t *)key;
    cfg.key_size = key_size;
    cfg.iv = (uint32_t *)iv;
    cfg.mode = mode;
    data.in_data = input;
    data.out_data = output;
    data.size = input_size;
    err = drv_aes_enc_sync(&cfg, &data);
    RT_ASSERT(RT_EOK == err);

    return SF_CRYPTO_E_OK;
}

