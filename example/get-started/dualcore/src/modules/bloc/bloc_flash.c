/**
 ******************************************************************************
 * @file   bloc_flash.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <rtthread.h>
#include <rthw.h>
#include "rtconfig.h"
#include "os_adaptor.h"
#include "board.h"
#ifndef BSP_USING_PC_SIMULATOR
#include "register.h"
#include "drv_flash.h"
#endif // !BSP_USING_PC_SIMULATOR

#include "bloc_flash.h"
#include "watch_global_data.h"
#include "fal.h"
#include "bf0_ble_common.h"

#ifdef RT_USING_DFS
#include "dfs_file.h"
#include "dfs_posix.h"
#include "app_mem.h"
#endif

#define DBG_TAG "bloc.flash"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

#define ENDIND_ADDR 0x9FFFFFFF

typedef enum
{
    DFU_FLASH_TYPE_NAND = 0,
    DFU_FLASH_TYPE_NOR,
} DFU_FLASH_TYPE;

/* Round size up to a power-of-two alignment boundary. */
#define FLASH_ALIGN_UP(sz, align) (((sz) + (align) - 1) & ~((align) - 1))

static int flash_erase(uint32_t dest, uint32_t offset, uint32_t size, uint8_t type)
{
    int ret = -1;
    // erase should 128k aligned
    if (type == DFU_FLASH_TYPE_NAND)
    {
        size = FLASH_ALIGN_UP(size, 0x20000);
    }
    // erase should 8k aligned
    else if (type == DFU_FLASH_TYPE_NOR)
    {
        size = FLASH_ALIGN_UP(size, 0x2000);
    }

    LOG_D("flash_erase dest 0x%x, size %d", dest, size);
    if (dest != 0xFFFFFFFF)
    {
        int ret1 = -1;
        if (type == DFU_FLASH_TYPE_NAND)
        {
#ifndef BSP_USING_PC_SIMULATOR
            ret1 = rt_nand_erase(dest, size);
#endif
        }
        else if (type == DFU_FLASH_TYPE_NOR)
        {
#ifndef BSP_USING_PC_SIMULATOR
            ret1 = rt_flash_erase(dest, size);
#endif
        }

        // int ret1 = 0;
        if (ret1 != 0)
        {
            ret = -2;
            LOG_E("flash_erase Fail! %d", ret1);
        }
        else
        {
            ret = 0;
        }
    }
    return ret;
}

static int flash_read(uint32_t dest, uint32_t offset, uint8_t *data, uint32_t size)
{
    int ret = -1;
    if (dest != 0xFFFFFFFF)
    {
#ifndef BSP_USING_PC_SIMULATOR
        uint32_t rd_size = rt_nand_read(dest + offset, data, size);
        // uint32_t rd_size = size;
        if (rd_size != size)
            ret = -2;
        else
            ret = 0;
#endif
    }
    return ret;
}

static int flash_write(uint32_t dest, uint8_t *data, uint32_t size, uint8_t type)
{
    int ret = -1;
    if (dest != 0xFFFFFFFF)
    {
        LOG_D("flash write dest 0x%x, size %d", dest, size);
#ifndef BSP_USING_PC_SIMULATOR
        uint32_t wr_size;
#endif
        if (type == DFU_FLASH_TYPE_NAND)
        {
#ifndef BSP_USING_PC_SIMULATOR
            wr_size = rt_nand_write_page(dest, data, size, NULL, 0);
#endif
        }
        else if (type == DFU_FLASH_TYPE_NOR)
        {
#ifndef BSP_USING_PC_SIMULATOR
            wr_size = rt_flash_write(dest, data, size);
#endif
        }
        // uint32_t wr_size = size;
#ifndef BSP_USING_PC_SIMULATOR
        if (wr_size != size)
            ret = -2;
        else
            ret = 0;
#endif
    }
    return ret;
}

bool check_image_write_address(uint32_t watch_image_write_addr, uint32_t len, uint32_t image_update_start_address, uint32_t binary_image_size_max)
{
    if ((watch_image_write_addr + len) >= ENDIND_ADDR)
        return false;
    if ((watch_image_write_addr + len) > (image_update_start_address + binary_image_size_max))
    {
        LOG_E("Binary Image expire MAX flash size!");
        return false;
    }
    if ((watch_image_write_addr < image_update_start_address))
    {
        LOG_E("Binary Image start address is invalid!");
        return false;
    }
    return true;
}

static void erase_sector(uint32_t destination, uint32_t erase_size)
{
    flash_erase(destination, 0, erase_size, DFU_FLASH_TYPE_NAND);
}

static bool write_page_sector(uint32_t watch_image_write_addr, uint8_t *buf, uint32_t len)
{
    // LOG_D("write page sector 0x%x, len %d", watch_image_write_addr, len);
    int ret = flash_write(watch_image_write_addr, buf, len, DFU_FLASH_TYPE_NAND);
    if (ret != 0)
    {
        LOG_E("write page sector fail");
        return false;
    }
    return true;
}

#define FLASH_WRITE_PACKET_SIZE 2048

static int dfu_update_firmware(image_header_t img_header)
{
    int ret = RT_EOK;
    uint32_t addr_offset = 0;
#ifndef BSP_USING_PC_SIMULATOR
    uint32_t download_addr;
#endif
    uint32_t target_addr;
    switch (img_header.id)
    {
    case DFU_IMG_ID_NAND_HCPU:
    {
#ifndef BSP_USING_PC_SIMULATOR
        download_addr = HCPU_CODE_DOWNLOAD_START_ADDRESS;
#endif
        target_addr = HCPU_CODE_START_ADDRESS;
        break;
    }

    case DFU_IMG_ID_NAND_LCPU:
    {
#ifndef BSP_USING_PC_SIMULATOR
        download_addr = LCPU_CODE_DOWNLOAD_START_ADDRESS;
#endif
        target_addr = LCPU_CODE_START_ADDRESS; // LCPU_FLASH_CODE_START_ADDR;
        LOG_D("LCPU code addr 0x%x, image length: %d", target_addr, img_header.length);
        break;
    }

    case DFU_IMG_ID_NAND_FTAB:
    {
#ifndef BSP_USING_PC_SIMULATOR
        download_addr = FTAB_START_DOWNLOAD_ADDRESS;
#endif
        target_addr = FTAB_START_ADDRESS;
        break;
    }

    case DFU_IMG_ID_NAND_BOOTLOADER:
    {
#ifndef BSP_USING_PC_SIMULATOR
        download_addr = BOOTLOADER_DOWNLOAD_START_ADDRESS;
#endif
        target_addr = BOOTLOADER_START_ADDRESS;
        LOG_D("BOOTLOADER addr 0x%x, image length: %d", target_addr, img_header.length);
        break;
    }

    case DFU_IMG_ID_NAND_LCPU_PATCH:
    {
#ifndef BSP_USING_PC_SIMULATOR
        download_addr = LCPU_PATCH_DOWNLOAD_START_ADDRESS;
#endif
        target_addr = LCPU_PATCH_START_ADDRESS;
        LOG_D("LCPU_PATCH addr 0x%x, image length: %d", target_addr, img_header.length);
        break;
    }

    default:
        LOG_E("Invalid image id %d", img_header.id);
        return -1;
    }

    // read image data from first section of download addr to make sure the image is valid
    uint8_t *temp_buf = rt_malloc(FLASH_WRITE_PACKET_SIZE);
    if (temp_buf == NULL)
    {
        LOG_E("Failed to malloc buf");
        return -1;
    }
#ifndef BSP_USING_PC_SIMULATOR
    ret = flash_read(download_addr, 0, temp_buf, FLASH_WRITE_PACKET_SIZE);
    if (ret != 0)
    {
        LOG_E("Failed to read on addr0x%x, ret=%d", download_addr, ret);
        rt_free(temp_buf);
        return ret;
    }
#endif

    LOG_D("erase target addr 0x%x", target_addr);

    /* Per-image flash parameters: erase max size + flash bus type.
       Looked up once and reused by both erase and write paths. */
    static const struct flash_img_param {
        uint8_t  id;
        uint32_t max_size;
        uint8_t  flash_type;
    } flash_map[] = {
        { DFU_IMG_ID_NAND_HCPU,       HCPU_CODE_MAX_SIZE,         DFU_FLASH_TYPE_NAND },
        { DFU_IMG_ID_NAND_LCPU,       LCPU_CODE_MAX_SIZE,         DFU_FLASH_TYPE_NOR  },
        { DFU_IMG_ID_NAND_FTAB,       FTAB_ACTUAL_MAX_SIZE,       DFU_FLASH_TYPE_NOR  },
        { DFU_IMG_ID_NAND_BOOTLOADER, BOOTLOADER_ACTUAL_MAX_SIZE, DFU_FLASH_TYPE_NOR  },
        { DFU_IMG_ID_NAND_LCPU_PATCH, LCPU_PATCH_ACTUAL_MAX_SIZE, DFU_FLASH_TYPE_NOR  },
    };
    const struct flash_img_param *params = NULL;
    for (size_t i = 0; i < sizeof(flash_map) / sizeof(flash_map[0]); i++)
    {
        if (flash_map[i].id == img_header.id)
        {
            params = &flash_map[i];
            break;
        }
    }
    if (params)
    {
        ret = flash_erase(target_addr, 0, params->max_size, params->flash_type);
    }

    if (ret != 0)
    {
        LOG_E("Failed to erase target addr ret=%d", ret);
        rt_free(temp_buf);
        return ret;
    }

    uint32_t offset = 0;
    uint32_t packet_size = FLASH_WRITE_PACKET_SIZE;

    while (offset < img_header.length)
    {
#ifndef BSP_USING_PC_SIMULATOR
        ret = flash_read(download_addr, offset, temp_buf, packet_size);
        if (ret != 0)
        {
            LOG_E("Failed to read on addr0x%x, ret=%d", download_addr + offset, ret);
            break;
        }
#endif
        if (offset + FLASH_WRITE_PACKET_SIZE > img_header.length)
        {
            // packet_size = img_header.length - offset;
            // Pad the rest of the buffer with 0xFF
            uint32_t last_packet_size = img_header.length - offset;
            memset(&temp_buf[last_packet_size], 0xFF, FLASH_WRITE_PACKET_SIZE - last_packet_size);
            packet_size = last_packet_size; // 修正：最後一包只寫有效長度
        }
        else
        {
            packet_size = FLASH_WRITE_PACKET_SIZE;
        }

        if (params)
        {
            ret = flash_write(target_addr + offset, temp_buf,
                              FLASH_WRITE_PACKET_SIZE, params->flash_type);
        }
        if (ret != 0)
        {
            LOG_E("Failed to write on addr0x%x, ret=%d", target_addr + offset, ret);
            break;
        }
        offset += packet_size;
    }
    // 檢查刷入長度
    if (offset != img_header.length)
    {
        LOG_E("Firmware update incomplete: written %d bytes, expected %d bytes", offset, img_header.length);
        ret = -3;
    }
    rt_free(temp_buf);

    return ret;
}

void halt_and_reset_lcpu_nor_flash(void)
{
#ifndef BSP_USING_PC_SIMULATOR
    LOG_D("%s", __func__);

    HAL_RCC_Reset_DMAC2_and_MPI5();
    HAL_RCC_Reset_and_Halt_LCPU(0);

    // reset mpi controller
    HAL_QSPIEX_FLASH_RESET2(hwp_mpi5);
    extern int rt_hw_flash5_init();
    int init_ret = rt_hw_flash5_init();
    LOG_D("[%s]init flash5 ret %d", __func__, init_ret);
    if (init_ret != 1)
    {
        RT_ASSERT(0);
    }
#endif
}

int dfu_check_fw_upgrade(image_header_t img_header)
{
    LOG_D("%s", __func__);
    int ret = RT_EOK;
    static uint8_t halt = 0;
    if (img_header.id == DFU_IMG_ID_NAND_LCPU)
    {
        if (halt == 0)
        {
            halt = 1;
            halt_and_reset_lcpu_nor_flash();
        }
    }

    return dfu_update_firmware(img_header);
}

static uint32_t get_download_destinate_address_with_id(uint8_t id)
{
    if (id == DFU_IMG_ID_NAND_HCPU)
    {
        return HCPU_CODE_DOWNLOAD_START_ADDRESS;
    }
    else if (id == DFU_IMG_ID_NAND_LCPU)
    {
        return LCPU_CODE_DOWNLOAD_START_ADDRESS;
    }
    else if (id == DFU_IMG_ID_NAND_FTAB)
    {
        return FTAB_START_DOWNLOAD_ADDRESS;
    }
    else if (id == DFU_IMG_ID_NAND_BOOTLOADER)
    {
        return BOOTLOADER_DOWNLOAD_START_ADDRESS;
    }
    else if (id == DFU_IMG_ID_NAND_LCPU_PATCH)
    {
        return LCPU_PATCH_DOWNLOAD_START_ADDRESS;
    }
    else
    {
        return IMAGE_DOWNLOAD_START_ADDRESS;
    }
}

FlashProvider flash_provider;

static int bloc_flash_provider_register(void)
{
    flash_provider.check_image_write_address = check_image_write_address;
    flash_provider.erase_sector = erase_sector;
    flash_provider.write_page_sector = write_page_sector;
    flash_provider.get_ota_address = get_download_destinate_address_with_id;
    return 0;
}

INIT_APP_EXPORT(bloc_flash_provider_register);

// #define UTEST_BLOC_FLASH
#ifdef UTEST_BLOC_FLASH
static int utest_bloc_flash(int argc, char *argv[])
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "-erase") == 0)
        {
            if (argc == 4)
            {
                uint32_t dest = strtoul(argv[2], NULL, 16);
                uint32_t size = atoi(argv[3]);
                flash_erase(dest, 0, size, DFU_FLASH_TYPE_NAND);
            }
            else
            {
                LOG_E("utest_bloc_flash -erase [dest] [size], example: utest_bloc_flash -erase 0x64400000 0x20000");
            }
        }
        else if (strcmp(argv[1], "-read") == 0)
        {
            if (argc == 5)
            {
                uint32_t dest = strtoul(argv[2], NULL, 16);
                uint32_t offset = atoi(argv[3]);
                uint32_t size = atoi(argv[4]);
                uint8_t *buf = rt_malloc(size);
                flash_read(dest, offset, buf, size);
                LOG_D("read data:");
                for (int i = 0; i < size; i++)
                {
                    LOG_RAW("%02x ", buf[i]);
                }
                rt_free(buf);
            }
            else
            {
                LOG_E("utest_bloc_flash -read [dest] [offset] [size]");
            }
        }
        else if (strcmp(argv[1], "-write") == 0)
        {
            if (argc == 5)
            {
                uint32_t dest = strtoul(argv[2], NULL, 16);
                uint8_t value = atoi(argv[3]);
                uint32_t size = atoi(argv[4]);
                uint8_t *buf = rt_malloc(size);
                memset(buf, value, size);
                flash_write(dest, buf, size, DFU_FLASH_TYPE_NAND);
                rt_free(buf);
            }
            else
            {
                LOG_E("utest_bloc_flash -write [dest] [value] [size]");
            }
        }
        else if (strcmp(argv[1], "-init_onchip") == 0)
        {
            // reset mpi controller
#ifndef BSP_USING_PC_SIMULATOR
            HAL_QSPIEX_FLASH_RESET2(hwp_mpi5);
            extern int rt_hw_flash5_init();
            int init_ret = rt_hw_flash5_init();
            LOG_D("dfu_check_lcpu_upgrade init flash5 ret %d", init_ret);
            if (init_ret != 1)
            {
                RT_ASSERT(0);
            }
#endif
        }
        else if (strcmp(argv[1], "-erase_onchip") == 0)
        {
            if (argc == 4)
            {
                uint32_t dest = strtoul(argv[2], NULL, 16);
                uint32_t size = atoi(argv[3]);
                flash_erase(dest, 0, size, DFU_FLASH_TYPE_NOR);
            }
            else
            {
                LOG_E("utest_bloc_flash -erase [dest] [size], example: utest_bloc_flash -erase 0x64400000 0x20000");
            }
        }
        else if (strcmp(argv[1], "-write_onchip") == 0)
        {
            if (argc == 5)
            {
                uint32_t dest = strtoul(argv[2], NULL, 16);
                uint8_t value = atoi(argv[3]);
                uint32_t size = atoi(argv[4]);
                uint8_t *buf = rt_malloc(size);
                memset(buf, value, size);
                flash_write(dest, buf, size, DFU_FLASH_TYPE_NOR);
                rt_free(buf);
            }
            else
            {
                LOG_E("utest_bloc_flash -write [dest] [value] [size]");
            }
        }
        else if (strcmp(argv[1], "upgrade_lcpu") == 0)
        {
            if (argc == 3)
            {
                uint32_t length = atoi(argv[2]);
                image_header_t header;
                header.id = DFU_IMG_ID_NAND_LCPU;
                header.length = length;
                int ret = dfu_check_fw_upgrade(header);
                if (ret)
                {
                    LOG_E("upgrade_lcpu fail");
                }
                else
                {
                    LOG_D("upgrade_lcpu success");
                }
            }
        }
        else if (strcmp(argv[1], "upgrade_hcpu") == 0)
        {
            if (argc == 3)
            {
                uint32_t length = atoi(argv[2]);
                image_header_t header;
                header.id = DFU_IMG_ID_NAND_HCPU;
                header.length = length;
                int ret = dfu_check_fw_upgrade(header);
                if (ret)
                {
                    LOG_E("upgrade_hcpu fail");
                }
                else
                {
                    LOG_D("upgrade_hcpu success");
                }
            }
        }
    }
    return 0;
}
MSH_CMD_EXPORT(utest_bloc_flash, "utest_bloc_flash [OPTION] ...");
#endif // UTEST_BLOC_FLASH

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
