/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "register.h"
#include "../dfu/dfu.h"
#ifdef PKG_SIFLI_MBEDTLS_BOOT
    #include "mbedtls/cipher.h"
    #include "mbedtls/pk.h"
#endif
#include "boot_flash.h"
#include "sifli_crypto.h"
#include "secboot.h"

#ifndef ALIGN
    #define ALIGN(n)                    __attribute__((aligned(n)))
#endif /* ALIGN */

static uint8_t dfu_hash[SF_CRYPTO_AES_CMAC_HASH_SIZE];
ALIGN(4)
static uint8_t default_root_key[DFU_KEY_SIZE];
sboot_ctx_t sboot_ctx;

static uint8_t is_addr_in_nor(uint32_t addr)
{
    if (boot_handle && (boot_handle->isNand == 0)
            && (addr >= boot_handle->base) && (addr < boot_handle->base + boot_handle->size))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/* out buf size must more than 32 byte */
int sifli_hash_calculate(uint8_t *in, uint32_t in_size, uint8_t *out, uint8_t algo)
{
    int last, i;

    if (!in || !in_size || !out || algo > 3)
        return -1;

    HAL_HASH_reset();
    HAL_HASH_init(NULL, algo, 0);

    if (in_size > SPLIT_THRESHOLD)
    {
        for (i = 0; i < in_size; i += SPLIT_THRESHOLD)
        {
            last = (i + SPLIT_THRESHOLD >= in_size) ? 1 : 0;
            if (i > 0)
            {
                HAL_HASH_reset();
                HAL_HASH_init((uint32_t *)out, algo, last ? i : 0);
            }
            HAL_HASH_run(&in[i], last ? in_size - i : SPLIT_THRESHOLD, last);
            HAL_HASH_result(out);
        }
        HAL_HASH_result(out);
    }
    else
    {
        HAL_HASH_run(in, in_size, 1);
        HAL_HASH_result(out);
    }

    return 0;
}

int sifli_hash_verify(uint8_t *data, uint32_t data_size, uint8_t *hash, uint32_t hash_size)
{
    uint8_t hash_out[32] = {0};

    if (!data || !hash)
        return -1;

    if (sifli_hash_calculate(data, data_size, hash_out, HASH_ALGO_SHA256))
        return -1;

    if (memcmp(hash_out, hash, hash_size))
        return -1;

    return 0;
}

int sifli_sigkey_pub_verify(uint8_t *sigkey, uint32_t key_size)
{
    uint32_t size = 0;

    uint8_t sigkey_hash[DFU_SIG_HASH_SIZE] = {0};
    size = sifli_hw_efuse_read(EFUSE_ID_SIG_HASH, sigkey_hash, DFU_SIG_HASH_SIZE);
    if (size == DFU_SIG_HASH_SIZE)
        return sifli_hash_verify(sigkey, key_size, sigkey_hash, DFU_SIG_HASH_SIZE);
    else
        return -1;
}

#ifdef PKG_SIFLI_MBEDTLS_BOOT
int sifli_img_sig_hash_verify(uint8_t *img_hash_sig, uint8_t *sig_pub_key, uint8_t *image, uint32_t img_size)
{
    uint8_t img_hash[32] = {0};
    mbedtls_pk_context pk;

    /*1.calculate image hash*/
    if (sifli_hash_calculate(image, img_size, img_hash, HASH_ALGO_SHA256))
        return -1;

    /*2.verify image hash digital signature*/
    mbedtls_pk_init(&pk);
    if (mbedtls_pk_parse_public_key(&pk, sig_pub_key, DFU_SIG_KEY_SIZE))
        return -1;
    mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
    if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, img_hash, DFU_IMG_HASH_SIZE, img_hash_sig, DFU_SIG_SIZE))
        return -1;

    return 0;
}
#endif

void sifli_secboot_exception(uint8_t excpt)
{
    char *err = NULL;

    switch (excpt)
    {
    case SECBOOT_SIGKEY_PUB_ERR:
        err = "secboot sigkey pub err!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    case SECBOOT_IMG_HASH_SIG_ERR:
        err = "secboot img hash sig err!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    default:
        err = "secboot excpt null!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    }

    HAL_sw_breakpoint();
}

bool sifli_verify_img_cmac_hash(uint8_t *key, uint8_t *in_data, int size, uint8_t *hash)
{
    sf_crypto_aes_cmac_ctx_t ctx;
    sf_crypto_err_t err;
    bool succ = false;

    if (g_dfu_efuse_read_hook && !key)
    {
        g_dfu_efuse_read_hook(EFUSE_ID_ROOT, default_root_key, DFU_KEY_SIZE);
        key = &default_root_key[0];
    }

    err = sf_crypto_aes_cmac_init(&ctx, (const uint32_t *)key, EFUSE_ROOTKEY_BYTE_SIZE);
    HAL_ASSERT(SF_CRYPTO_E_OK == err);

    err = sf_crypto_aes_cmac_calc(&ctx, in_data, size, true, dfu_hash);
    if (err != SF_CRYPTO_E_OK)
    {
        goto __EXIT;
    }

    if (memcmp(dfu_hash, hash, sizeof(dfu_hash)) != 0)
    {
        goto __EXIT;
    }

    succ = true;

__EXIT:
    return succ;
}

void sboot_init(void)
{
    uint8_t pattern;
    int len;
    uint8_t rootkey[EFUSE_ROOTKEY_BYTE_SIZE];
    const uint32_t *bank0_data;

    /* load bank0 which has SWDDIS and SECEN*/
    bank0_data = sifli_hw_efuse_load_bank0();
    if (bank0_data)
    {
        len = HAL_EFUSE_Extract(bank0_data, EFUSE_SECEN_OFFSET, &pattern, EFUSE_SECEN_SIZE);
    }
    else
    {
        printf("load bank0 fails\n");
        len = 0;
    }
    if ((len > 0) && (pattern != 0))
    {
        sboot_ctx.sec_en = true;
    }
#ifdef CFG_BOOTROM
    if (bank0_data)
    {
        len = HAL_EFUSE_Extract(bank0_data, EFUSE_PINRST_OFFSET, &pattern, EFUSE_PINRST_SIZE);
        if ((EFUSE_PINRST_SIZE == len) && (3 == pattern))
        {
            /* 25ms for rc10k, 7ms for rc32k */
            hwp_pmuc->PWRKEY_CNT = 256;
        }
    }

    len = sifli_hw_efuse_read(EFUSE_ID_ROOT, rootkey, sizeof(rootkey));
    if (len != sizeof(rootkey))
    {
        printf("load rootkey fails\n");
    }
#endif /* CFG_BOOTROM */
}

void run_img(uint32_t dest)
{
    __asm
    (
        "LDR SP, [%0]                     \n"
        "LDR PC, [%0, #4]                 \n"
        :           /* Outputs */
        : "r"(dest) /* Inputs */
        :           /* Clobbers */
    );
}


void dfu_boot_img_in_flash(int flashid)
{
    uint32_t src = sec_config_cache.ftab[flashid].base;
    uint32_t dest = sec_config_cache.ftab[flashid].xip_base;
    int coreid = DFU_FLASH_IMG_IDX(flashid);
    struct image_header_enc *img_hdr = &(sec_config_cache.imgs[coreid]);
    uint32_t copy_len;

    if (img_hdr->length && (src != dest)
            && (is_addr_in_nor((uint32_t)dest) == 0))
    {
        if (sboot_ctx.sec_en)
        {
            copy_len = img_hdr->length + SF_CRYPTO_AES_CMAC_HASH_SIZE;
        }
        else
        {
            copy_len = img_hdr->length;
        }

        g_flash_read(src, (const int8_t *)dest, copy_len);
    }

    if (coreid < 2 * CORE_MAX)
    {
        coreid %= CORE_MAX;
        if (coreid == CORE_HCPU || coreid == CORE_BL || coreid == CORE_LCPU)
        {
            if (is_addr_in_nor(dest))
            {
                HAL_FLASH_ALIAS_CFG(boot_handle, dest, img_hdr->length, src - dest);
            }

            if (!sboot_ctx.sec_en || (coreid != CORE_BL) || sifli_verify_img_cmac_hash(NULL, (uint8_t *)dest, img_hdr->length, (uint8_t *)dest + img_hdr->length))
            {
                run_img(dest);
            }
            else
            {
                printf("image signature verification fails\n");
            }
        }
    }
}

