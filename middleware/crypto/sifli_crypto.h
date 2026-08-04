/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SIFLI_CRYPTO_H__
#define __SIFLI_CRYPTO_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SF_CRYPTO_E_OK                     0               /**< There is no error */
#define SF_CRYPTO_E_INVALID_ARG            1               /**< Invalid argument */
#define SF_CRYPTO_E_CTX_NOT_INIT           2               /**< Context is not initialized */
#define SF_CRYPTO_E_INVALID_KEY            3               /**< Invalid key */
#define SF_CRYPTO_E_INVALID_DATA_SIZE      4               /**< Invalid data size */

typedef int32_t sf_crypto_err_t;

#define SF_CRYPTO_AES_CMAC_PACKET_SIZE         16          /**< CMAC packet size in byte */
#define SF_CRYPTO_AES_CMAC_K1_SIZE             16          /**< CMAC key K1 size ini byte */
#define SF_CRYPTO_AES_CMAC_K2_SIZE             16          /**< CMAC key K2 size in byte */
#define SF_CRYPTO_AES_CMAC_HASH_SIZE           16          /**< CMAC hash size in byte */
#define SF_CRYPTO_AES_CMAC_KEY_SIZE            32          /**< CMAC original key size in byte */

/** AES-CMAC context structure */
typedef struct
{
    /** original key */
    const uint32_t *key;
    /** original key size, must be equal to SF_CRYPTO_AES_CMAC_KEY_SIZE */
    uint16_t key_size;
    /** CMAC key K1 */
    uint8_t k1[SF_CRYPTO_AES_CMAC_K1_SIZE];
    /** CMAC key K2 */
    uint8_t k2[SF_CRYPTO_AES_CMAC_K2_SIZE];
    /** CMAC IV, i.e. hash of last computation */
    uint32_t iv[SF_CRYPTO_AES_CMAC_HASH_SIZE / sizeof(uint32_t)];
    /** whether it's the first calculation, i.e. iv is all zero */
    bool is_first;
    /** whether it's initialized */
    bool is_init;
} sf_crypto_aes_cmac_ctx_t;


/**
 * @brief  Initialize AES-CMAC context by the given key
 *
 * @param[in]  ctx   context to be initialized
 * @param[in]  key   if key is NULL, rootkey in eFuse would be used.
 *                   Pointer content would be used later, it cannot be modified or freed
 * @param[in]  key_size key size in byte, only 32bytes key is supported
 *
 * @return   SF_CRYPTO_E_OK on success, otherwise on failure, e.g. -SF_CRYPTO_E_INVALID_ARG
 */
sf_crypto_err_t sf_crypto_aes_cmac_init(sf_crypto_aes_cmac_ctx_t *ctx, const uint32_t *key, uint16_t key_size);

/**
 * @brief  Reset AES-CMAC context
 *
 * Context state returns to the init state like calling sf_crypto_aes_cmac_init, IV in context is cleared.
 * The next calculation is regarded as the first block.
 *
 * @param[in]  ctx   context
 *
 * @return   SF_CRYPTO_E_OK on success, otherwise on failure, e.g. -SF_CRYPTO_E_INVALID_ARG
 */
sf_crypto_err_t sf_crypto_aes_cmac_reset(sf_crypto_aes_cmac_ctx_t *ctx);

/**
 * @brief  Calculate the hash
 *
 * @param[in]  ctx    context
 * @param[in]  data   data to be calculated
 * @param[in]  data_size   data size in byte
 * @param[in]  is_last     whether it's the last block, padding is applied to the last block
 * @param[out] hash        hash
 *
 * @return   SF_CRYPTO_E_OK on success, otherwise on failure, e.g. -SF_CRYPTO_E_INVALID_ARG
 */
sf_crypto_err_t sf_crypto_aes_cmac_calc(sf_crypto_aes_cmac_ctx_t *ctx, uint8_t *data, uint32_t data_size,
                                        bool is_last, uint8_t *hash);

#ifdef __cplusplus
}
#endif


#endif /* __SIFLI_CRYPTO_H__ */

