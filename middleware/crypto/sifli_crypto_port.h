/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SIFLI_CRYPTO_PORT_H__
#define __SIFLI_CRYPTO_PORT_H__

#include <stdint.h>
#include "sifli_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

sf_crypto_err_t sf_crypto_aes_enc_sync(const uint32_t *key, uint32_t key_size, const uint32_t *iv, uint8_t mode,
                                       uint8_t *input, uint32_t input_size, uint8_t *output);


#ifdef __cplusplus
}
#endif


#endif /* __SIFLI_CRYPTO_PORT_H__ */

