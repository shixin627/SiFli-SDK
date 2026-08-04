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

#define SF_CRYPTO_AES_CMAC_RB                  0x87U

static void sf_crypto_cmac_gen_subkey(uint8_t *input, uint8_t *output)
{
    uint8_t overflow_bit;
    int32_t i;

    HAL_ASSERT(input && output);

    overflow_bit = 0;
    /* generate subkey in big-endian */
    for (i = (SF_CRYPTO_AES_CMAC_PACKET_SIZE - 1); i >= 0 ; i--)
    {
        output[i] = (input[i] << 1) | overflow_bit;
        /* bit7 is shifted to bit0 of previous byte */
        overflow_bit = input[i] >> 7;
    }

    if (input[0] & (uint8_t)0x80)
    {
        output[SF_CRYPTO_AES_CMAC_PACKET_SIZE - 1] ^= (uint8_t)SF_CRYPTO_AES_CMAC_RB;
    }
}

sf_crypto_err_t sf_crypto_aes_cmac_init(sf_crypto_aes_cmac_ctx_t *ctx, const uint32_t *key, uint16_t key_size)
{
    sf_crypto_err_t err;
    uint8_t plain_packet[SF_CRYPTO_AES_CMAC_PACKET_SIZE];
    uint8_t encrypted_packet[SF_CRYPTO_AES_CMAC_PACKET_SIZE];
    uint8_t *k1;
    uint8_t *k2;

    if (key_size != SF_CRYPTO_AES_CMAC_KEY_SIZE)
    {
        return -SF_CRYPTO_E_INVALID_KEY;
    }

    if (!ctx)
    {
        return -SF_CRYPTO_E_INVALID_ARG;
    }

    ctx->key = key;
    ctx->key_size = key_size;

    k1 = ctx->k1;
    k2 = ctx->k2;
    memset((void *)plain_packet, 0, sizeof(plain_packet));

    err = sf_crypto_aes_enc_sync(key, key_size, NULL, AES_MODE_ECB, plain_packet, sizeof(plain_packet), encrypted_packet);
    HAL_ASSERT(err == SF_CRYPTO_E_OK);

    sf_crypto_cmac_gen_subkey(encrypted_packet, k1);
    sf_crypto_cmac_gen_subkey(k1, k2);

    memset((void *)ctx->iv, 0, sizeof(ctx->iv));
    ctx->is_first = true;
    ctx->is_init = true;

    return SF_CRYPTO_E_OK;
}

sf_crypto_err_t sf_crypto_aes_cmac_reset(sf_crypto_aes_cmac_ctx_t *ctx)
{
    if (!ctx || !ctx->is_init)
    {
        return -SF_CRYPTO_E_CTX_NOT_INIT;
    }
    memset((void *)ctx->iv, 0, sizeof(ctx->iv));
    ctx->is_first = true;

    return SF_CRYPTO_E_OK;
}

sf_crypto_err_t sf_crypto_aes_cmac_calc(sf_crypto_aes_cmac_ctx_t *ctx, uint8_t *data, uint32_t data_size,
                                        bool is_last, uint8_t *hash)
{
    uint32_t packet_num;
    uint32_t i;
    uint8_t last_packet[SF_CRYPTO_AES_CMAC_PACKET_SIZE];
    uint16_t padding_size;
    sf_crypto_err_t err;

    if (!ctx || !hash)
    {
        return -SF_CRYPTO_E_INVALID_ARG;
    }

    if (!ctx->is_init)
    {
        return -SF_CRYPTO_E_CTX_NOT_INIT;
    }

    if ((data_size > 0) && !data)
    {
        return -SF_CRYPTO_E_INVALID_ARG;
    }

    if (0 == data_size)
    {
        /* only the first and last block could be empty */
        if (!ctx->is_first || !is_last)
        {
            return -SF_CRYPTO_E_INVALID_DATA_SIZE;
        }
    }

    if (!is_last && (data_size & (SF_CRYPTO_AES_CMAC_PACKET_SIZE - 1)))
    {
        /* only the last block could be non-aligned */
        return -SF_CRYPTO_E_INVALID_DATA_SIZE;
    }

    if (is_last)
    {
        packet_num = (data_size + (SF_CRYPTO_AES_CMAC_PACKET_SIZE - 1)) / SF_CRYPTO_AES_CMAC_PACKET_SIZE;
        if (0 == data_size)
        {
            padding_size = SF_CRYPTO_AES_CMAC_PACKET_SIZE;
            packet_num = 1;
        }
        else if (data_size & (SF_CRYPTO_AES_CMAC_PACKET_SIZE - 1))
        {
            padding_size = SF_CRYPTO_AES_CMAC_PACKET_SIZE - (data_size & (SF_CRYPTO_AES_CMAC_PACKET_SIZE - 1));
            memcpy((void *)last_packet, (void *)(data + (packet_num - 1) * SF_CRYPTO_AES_CMAC_PACKET_SIZE), (SF_CRYPTO_AES_CMAC_PACKET_SIZE - padding_size));
        }
        else
        {
            padding_size = 0;
            memcpy((void *)last_packet, (void *)(data + (packet_num - 1) * SF_CRYPTO_AES_CMAC_PACKET_SIZE), SF_CRYPTO_AES_CMAC_PACKET_SIZE);
        }


        if (padding_size > 0)
        {
            /* padding the first byte as 0x80 */
            last_packet[SF_CRYPTO_AES_CMAC_PACKET_SIZE - padding_size] = 0x80;
            /* the left is zero */
            for (i = 1; i < padding_size; i++)
            {
                last_packet[SF_CRYPTO_AES_CMAC_PACKET_SIZE - padding_size + i] = 0;
            }
            for (i = 0; i < SF_CRYPTO_AES_CMAC_PACKET_SIZE; i++)
            {
                last_packet[i] ^= ctx->k2[i];
            }
        }
        else
        {
            for (i = 0; i < SF_CRYPTO_AES_CMAC_PACKET_SIZE; i++)
            {
                last_packet[i] ^= ctx->k1[i];
            }
        }

        if (packet_num > 1)
        {
            /* calculate hash excluding last packet */
            err = sf_crypto_aes_enc_sync(ctx->key, ctx->key_size, ctx->iv, AES_MODE_CBC_MAC,
                                         data, (packet_num - 1) * SF_CRYPTO_AES_CMAC_PACKET_SIZE, hash);
            HAL_ASSERT(err == SF_CRYPTO_E_OK);
            memcpy((void *)ctx->iv, (void *)hash, sizeof(ctx->iv));
        }

        /* update hash by last packet */
        err = sf_crypto_aes_enc_sync(ctx->key, ctx->key_size, ctx->iv, AES_MODE_CBC_MAC,
                                     last_packet, SF_CRYPTO_AES_CMAC_PACKET_SIZE, hash);
        HAL_ASSERT(err == SF_CRYPTO_E_OK);
    }
    else
    {
        err = sf_crypto_aes_enc_sync(ctx->key, ctx->key_size, ctx->iv, AES_MODE_CBC_MAC,
                                     data, data_size, hash);
        HAL_ASSERT(err == SF_CRYPTO_E_OK);
    }

    /* update IV for next use */
    memcpy((void *)ctx->iv, (void *)hash, sizeof(ctx->iv));

    ctx->is_first = false;

    return SF_CRYPTO_E_OK;
}
