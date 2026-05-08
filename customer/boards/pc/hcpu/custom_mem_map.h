/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CUSTOM_MEM_MAP__
#define __CUSTOM_MEM_MAP__

#ifdef USING_PARTITION_TABLE
    #include "ptab.h"
#endif /* USING_PARTITION_TABLE */

/* PC simulator: stub partition offsets so fal_partition.c and bloc_flash.h
 * build. Real ARM builds get these from generated ptab.h; PC's ptab.json is
 * empty. Numbers are arbitrary — never read by the sim. */
#ifndef KVDB_DFU_REGION_OFFSET
#define KVDB_DFU_REGION_OFFSET   0x00000000
#define KVDB_DFU_REGION_SIZE     0x00040000
#define KVDB_BLE_REGION_OFFSET   0x00040000
#define KVDB_BLE_REGION_SIZE     0x00040000
#endif

#ifndef FLASH_BOOT_LOADER_START_ADDR
#define FLASH_BOOT_LOADER_START_ADDR             0x10020000
#define FLASH_TABLE_START_ADDR                   0x1C000000
#define HCPU_FLASH_CODE_LOAD_REGION_START_ADDR   0x12000000
#define HCPU_FLASH_CODE_LOAD_REGION_SIZE         0x00800000
#endif

#define FAL_PART_TABLE \
{ \
    {FAL_PART_MAGIC_WORD,       "dfu",      NOR_FLASH3_DEV_NAME,    KVDB_DFU_REGION_OFFSET,   KVDB_DFU_REGION_SIZE, 0}, \
    {FAL_PART_MAGIC_WORD,       "ble",      NOR_FLASH3_DEV_NAME,    KVDB_BLE_REGION_OFFSET,   KVDB_BLE_REGION_SIZE, 0}, \
}

#endif  /* __MEM_MAP__ */

