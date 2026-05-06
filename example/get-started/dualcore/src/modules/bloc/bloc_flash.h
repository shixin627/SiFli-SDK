/**
*****************************************************************************************
*     Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      bloc_control.h
* @brief     business logic of control page
* @author    jack
* @date      2023-01-28
* @version   v1.0
**************************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2023 Skaiwalk Corporation</center></h2>
**************************************************************************************
*/

#ifndef __BLOC_FLASH_H__
#define __BLOC_FLASH_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "board.h"
#define IMAGE_MAX_SIZE (0x00800000)
#ifndef BSP_USING_PC_SIMULATOR
#define IMAGE_DOWNLOAD_START_ADDRESS BLE_OTA_REGION_START_ADDR
#else
#define IMAGE_DOWNLOAD_START_ADDRESS 0x8000000
#endif

#define HCPU_CODE_DOWNLOAD_START_ADDRESS IMAGE_DOWNLOAD_START_ADDRESS
#define HCPU_CODE_MAX_SIZE HCPU_FLASH_CODE_LOAD_REGION_SIZE
#define HCPU_CODE_START_ADDRESS HCPU_FLASH_CODE_LOAD_REGION_START_ADDR

#define LCPU_CODE_DOWNLOAD_START_ADDRESS HCPU_CODE_DOWNLOAD_START_ADDRESS + HCPU_CODE_MAX_SIZE
#define LCPU_CODE_MAX_SIZE (0x40000) // 256KB
#define LCPU_CODE_START_ADDRESS 0x1C040000 // LCPU_FLASH_CODE_START_ADDR

#define FTAB_START_DOWNLOAD_ADDRESS LCPU_CODE_DOWNLOAD_START_ADDRESS + LCPU_CODE_MAX_SIZE
#define FTAB_MAX_SIZE (0x20000) // 128KB (aligned to NAND block size, actual size 16KB)
#define FTAB_ACTUAL_MAX_SIZE (0x4000) // 16KB
#define FTAB_START_ADDRESS FLASH_TABLE_START_ADDR

#define BOOTLOADER_DOWNLOAD_START_ADDRESS FTAB_START_DOWNLOAD_ADDRESS + FTAB_MAX_SIZE
#define BOOTLOADER_MAX_SIZE (0x20000) // 128KB (aligned to NAND block size, actual size 32KB)
#define BOOTLOADER_ACTUAL_MAX_SIZE (0x8000) // 32KB
#define BOOTLOADER_START_ADDRESS FLASH_BOOT_LOADER_START_ADDR

#define LCPU_PATCH_DOWNLOAD_START_ADDRESS BOOTLOADER_DOWNLOAD_START_ADDRESS + BOOTLOADER_MAX_SIZE
#define LCPU_PATCH_MAX_SIZE (0x20000) // 128KB (aligned to NAND block size, actual size 16KB)
#define LCPU_PATCH_ACTUAL_MAX_SIZE (0x4000) // 16KB
#define LCPU_PATCH_START_ADDRESS 0x1C004000 // LCPU_PATCH_FLASH_START_ADDR

// Backup partition - located after download region
#define BACKUP_START_ADDRESS (IMAGE_DOWNLOAD_START_ADDRESS + HCPU_CODE_MAX_SIZE + LCPU_CODE_MAX_SIZE + FTAB_MAX_SIZE + BOOTLOADER_MAX_SIZE + LCPU_PATCH_MAX_SIZE)
#define BACKUP_HCPU_START_ADDRESS BACKUP_START_ADDRESS
#define BACKUP_HCPU_MAX_SIZE HCPU_CODE_MAX_SIZE
#define BACKUP_LCPU_START_ADDRESS (BACKUP_HCPU_START_ADDRESS + BACKUP_HCPU_MAX_SIZE)
#define BACKUP_LCPU_MAX_SIZE LCPU_CODE_MAX_SIZE
#define BACKUP_FTAB_START_ADDRESS (BACKUP_LCPU_START_ADDRESS + BACKUP_LCPU_MAX_SIZE)
#define BACKUP_FTAB_MAX_SIZE FTAB_MAX_SIZE
#define BACKUP_BOOTLOADER_START_ADDRESS (BACKUP_FTAB_START_ADDRESS + BACKUP_FTAB_MAX_SIZE)
#define BACKUP_BOOTLOADER_MAX_SIZE BOOTLOADER_MAX_SIZE
#define BACKUP_LCPU_PATCH_START_ADDRESS (BACKUP_BOOTLOADER_START_ADDRESS + BACKUP_BOOTLOADER_MAX_SIZE)
#define BACKUP_LCPU_PATCH_MAX_SIZE LCPU_PATCH_MAX_SIZE
#define BACKUP_TOTAL_SIZE (BACKUP_HCPU_MAX_SIZE + BACKUP_LCPU_MAX_SIZE + BACKUP_FTAB_MAX_SIZE + BACKUP_BOOTLOADER_MAX_SIZE + BACKUP_LCPU_PATCH_MAX_SIZE)

#define FS_START_DOWNLOAD_ADDRESS FS_REGION_START_ADDR
#define FS_MAX_SIZE (FS_REGION_SIZE) // 16MB
#define FS_START_ADDRESS FS_REGION_START_ADDR

    typedef enum
    {
        DFU_IMG_ID_NAND_HCPU,
        DFU_IMG_ID_NAND_LCPU,
        DFU_IMG_ID_NAND_FTAB,
        DFU_IMG_ID_NAND_BOOTLOADER,
        DFU_IMG_ID_NAND_LCPU_PATCH,
        DFU_IMG_ID_NAND_IMAGE,
        DFU_IMG_ID_NAND_MAX,
    } dfu_img_id_t;

    typedef struct
    {
        dfu_img_id_t id;
        uint32_t length;
    } image_header_t;

    typedef struct
    {
        bool (*check_image_write_address)(uint32_t watch_image_write_addr, uint32_t len, uint32_t image_update_start_address, uint32_t binary_image_size_max);
        void (*erase_sector)(uint32_t destination, uint32_t erase_size);
        bool (*write_page_sector)(uint32_t watch_image_write_addr, uint8_t *buf, uint32_t len);
        uint32_t (*get_ota_address)(uint8_t id);
    } FlashProvider;
    extern FlashProvider flash_provider;
    extern int dfu_check_fw_upgrade(image_header_t img_header);
#ifdef __cplusplus
}
#endif

#endif //__BLOC_FLASH_H__
