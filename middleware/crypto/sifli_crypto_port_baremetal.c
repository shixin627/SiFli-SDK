/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <string.h>
#include <stdint.h>
#include "board.h"
#include "sifli_crypto_port.h"
sf_crypto_err_t sf_crypto_aes_enc_sync(const uint32_t *key, uint32_t key_size, const uint32_t *iv, uint8_t mode,
                                       uint8_t *input, uint32_t input_size, uint8_t *output)
{
    HAL_StatusTypeDef status;
    int r;

    r = HAL_AES_init((uint32_t *)key, key_size, (uint32_t *)iv, mode);
    HAL_ASSERT(0 == r);
    status = HAL_AES_run(AES_ENC, input, output, input_size);
    HAL_ASSERT(HAL_OK == status);

    return SF_CRYPTO_E_OK;
}

