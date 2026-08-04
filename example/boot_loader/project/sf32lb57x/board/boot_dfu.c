/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "rtconfig.h"
#include "bf0_hal.h"
#include "../dfu/dfu.h"
#include "sifli_crypto.h"
#include "secboot.h"

#ifdef CFG_BOOTLOADER
    #define DBG_ENABLE
#endif /* CFG_BOOTLOADER */

#define LOG_TAG "dfu"
#define DBG_LVL    DBG_INFO

#ifndef ALIGN
    #define ALIGN(n)                    __attribute__((aligned(n)))
#endif /* ALIGN */


#define IS_MPI_ADDR(addr, i) ((addr >= MPI##i##_MEM_BASE) && (addr < (MPI##i##_MEM_BASE + QSPI##i##_MAX_SIZE)))

#define LOG_I(...)   printf(__VA_ARGS__)
#define LOG_E(...)   printf(__VA_ARGS__)

#define DFU_CMAC_BLK_SIZE    (2048)

static uint8_t dfu_temp[DFU_CMAC_BLK_SIZE];
static uint8_t dfu_hash[SF_CRYPTO_AES_CMAC_HASH_SIZE];
ALIGN(4)
static uint8_t default_root_key[DFU_KEY_SIZE];

struct sec_configuration *g_config_cache;
static struct image_header_enc g_boot_patch_img_head;
static uint32_t g_boot_patch_addr = BOOTLOADER_PATCH_CODE_ADDR;

//#define DFU_DEV
#ifdef DFU_DEV
    extern int sifli_hw_efuse_write_din(uint8_t id, uint8_t *data, int size);
#endif

static const uint8_t g_fake_dfu_efuse_root_key[DFU_KEY_SIZE] =
{
    0xE6, 0x92, 0xDF, 0xB5, 0xE8, 0xFA, 0xC2, 0x43, 0x78, 0x13, 0x34, 0x5D, 0x1B, 0x4B, 0xE2, 0xB9,
    0x50, 0x9D, 0xE5, 0xC2, 0xF4, 0x05, 0x62, 0xE6, 0xC8, 0x25, 0x85, 0x81, 0x02, 0x59, 0x17, 0x2B,
};

static void clear_interrupt_setting(void)
{
    uint32_t i;
    for (i = 0; i < 16; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        __DSB();
        __ISB();
    }
}


static void dfu_run_img(uint32_t dest)
{
    HAL_DisableInterrupt();
    clear_interrupt_setting();

    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}


static int dfu_get_efuse_hook(uint8_t id, uint8_t *data, int size)
{
    int ret = 0;

    if (id == EFUSE_ID_ROOT)
    {
        memcpy(data, (uint8_t *)g_fake_dfu_efuse_root_key, DFU_KEY_SIZE);
        ret = DFU_KEY_SIZE;
    }

    return ret;

}

static int dfu_process_hdr_sec(uint8_t flashid, uint8_t *data, int size)
{
    int r = DFU_FAIL;
    struct image_cfg_hdr *hdr = (struct image_cfg_hdr *)data;
    bool succ;

    if (DFU_FLASH_IMG_BOOT2 != flashid)
    {
        return r;
    }

    data += sizeof(struct image_cfg_hdr);
    size -= sizeof(struct image_cfg_hdr);

    succ = sifli_verify_img_cmac_hash(NULL, data, size, hdr->hash);
    if (succ)          // Use root key to decode image header
    {
        if (DFU_FLASH_IMG_BOOT2 == flashid)
        {
            memcpy(&g_boot_patch_img_head, data, sizeof(g_boot_patch_img_head));
            memset((uint8_t *)g_boot_patch_addr, 0, size);
        }
        r = DFU_SUCCESS;
    }
    else
    {
        LOG_E("image header verify fail\n");
    }
    return r;
}

static int dfu_process_body_sec(uint8_t flashid, uint8_t *data, int size)
{
    int r = DFU_FAIL;
    struct image_body_hdr *hdr = (struct image_body_hdr *)data;
    bool succ;
    struct image_header_enc *img_hdr;

    if (DFU_FLASH_IMG_BOOT2 != flashid)
    {
        return r;
    }

    size -= sizeof(struct image_body_hdr);
    data += sizeof(struct image_body_hdr);
    succ = sifli_verify_img_cmac_hash(NULL, data, size, hdr->hash);
    if (succ)
    {
        img_hdr = &g_boot_patch_img_head;
        sec_flash_write(flashid, hdr->offset, data, size);
        r = DFU_SUCCESS;
    }
    else
    {
        LOG_E("body verify fail, offset:%d\n", hdr->offset);
    }

    return r;
}

static int dfu_process_config_sec(uint8_t keyid, uint8_t *data, int len)
{
    int r = DFU_FAIL;

    if (keyid < DFU_CONFIG_FLASH_TABLE)
    {
        sifli_hw_efuse_write(keyid, data, len);
        r = DFU_SUCCESS;
    }
    else if (keyid == DFU_CONFIG_BOOT_PATCH_ADDR)
    {
        g_boot_patch_addr = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                            | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
        printf("config boot patch addr:0x%x", g_boot_patch_addr);
    }
    return r;
}

int sec_flash_read(int flashid, uint32_t offset, uint8_t *data, uint32_t size)
{
    if (flashid == DFU_FLASH_IMG_BOOT2)
    {
        memcpy(data, (uint8_t *)(g_boot_patch_addr + offset), size);
    }
    else
    {
        HAL_ASSERT(0);
    }
    return size;
}

void sec_flash_write(int flashid, uint32_t offset, uint8_t *data, uint32_t size)
{
    if (flashid == DFU_FLASH_IMG_BOOT2)
    {
        memcpy((uint8_t *)(g_boot_patch_addr + offset), data, size);
    }
    else
    {
        HAL_ASSERT(0);
    }
}

void sec_flash_erase(int flashid, uint32_t offset, uint32_t size)
{
    if (flashid == DFU_FLASH_IMG_BOOT2)
    {
        memset((uint8_t *)g_boot_patch_addr, 0, size);
    }
    else
    {
        HAL_ASSERT(0);
    }
}


void sec_flash_update(int flashid, uint32_t offset, uint8_t *data, uint32_t size)
{
    HAL_ASSERT(0);
}

void sec_flash_init()
{
}

static int dfu_end(uint8_t flashid)
{
    int offset = 0;
    int size = 0;
    int r = DFU_FAIL;
    int img_idx;
    struct image_header_enc *img_hdr;
    sf_crypto_aes_cmac_ctx_t ctx;
    sf_crypto_err_t err;
    bool is_last;
    uint8_t *key = NULL;

    if (DFU_FLASH_IMG_BOOT2 != flashid)
    {
        printf("invalid flashid:%d\n", flashid);
        return r;
    }

    img_idx = DFU_FLASH_IMG_IDX(flashid);
    img_hdr = &g_boot_patch_img_head;

    if (g_dfu_efuse_read_hook)
    {
        g_dfu_efuse_read_hook(EFUSE_ID_ROOT, default_root_key, DFU_KEY_SIZE);
        key = &default_root_key[0];
    }

    /* verify signature */
    err = sf_crypto_aes_cmac_init(&ctx, (const uint32_t *)key, EFUSE_ROOTKEY_BYTE_SIZE);
    HAL_ASSERT(SF_CRYPTO_E_OK == err);
    do
    {
        if ((offset + sizeof(dfu_temp)) >= img_hdr->length)
        {
            size = img_hdr->length - offset;
            is_last = true;
        }
        else
        {
            size = sizeof(dfu_temp);
            is_last = false;
        }

        if (size)
        {
            sec_flash_read(flashid, offset, dfu_temp, size);
            err = sf_crypto_aes_cmac_calc(&ctx, dfu_temp, size, is_last, dfu_hash);
            HAL_ASSERT(SF_CRYPTO_E_OK == err);
        }
        offset += size;
    }
    while (!is_last);

    if (memcmp(dfu_hash, img_hdr->sig, sizeof(dfu_hash)) != 0)
    {
        printf("img verify fail\n");
    }
    else
    {
        printf("jump to %x\n", *(uint32_t *)(g_boot_patch_addr + 4));
        dfu_run_img(g_boot_patch_addr);
        r = DFU_SUCCESS;
    }
    return r;
}

void dfu_init(void)
{
    if (!sboot_ctx.sec_en)
    {
        g_dfu_efuse_read_hook = dfu_get_efuse_hook;
    }
}

int dfu_receive_pkt(int len, uint8_t *data)
{
    int r;
    struct dfu_hdr *hdr = (struct dfu_hdr *)data;

    switch (hdr->command)
    {
    case DFU_IMG_HDR_ENC:
        r = dfu_process_hdr_sec(hdr->id, data + sizeof(struct dfu_hdr), len - sizeof(struct dfu_hdr));
        break;
    case DFU_IMG_BODY_ENC:
        r = dfu_process_body_sec(hdr->id, data + sizeof(struct dfu_hdr), len - sizeof(struct dfu_hdr));
        break;
    case DFU_CONFIG_ENC:
        r = dfu_process_config_sec(hdr->id, data + sizeof(struct dfu_hdr), len - sizeof(struct dfu_hdr));
        break;
    case DFU_END:
        r = dfu_end(hdr->id);
        break;
    default:
        r = DFU_FAIL;
        break;
    }
    return r;
}

