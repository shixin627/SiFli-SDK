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

#ifndef USING_PTAB_V3
#define FAL_PART_TABLE \
{ \
    {FAL_PART_MAGIC_WORD,       "dfu",      NOR_FLASH3_DEV_NAME,    KVDB_DFU_REGION_OFFSET,   KVDB_DFU_REGION_SIZE, 0}, \
    {FAL_PART_MAGIC_WORD,       "ble",      NOR_FLASH3_DEV_NAME,    KVDB_BLE_REGION_OFFSET,   KVDB_BLE_REGION_SIZE, 0}, \
}

/* Match the legacy CoreMark 52x LCPU layout on v1/v2 PTAB boards. */
#undef LCPU_RAM_DATA_START_ADDR
#define LCPU_RAM_DATA_START_ADDR (LPSYS_EM_BASE)

#undef LCPU_RAM_DATA_SIZE
#define LCPU_RAM_DATA_SIZE (LPSYS_EM_SIZE)

#undef LPSYS_RAM_SIZE
#define LPSYS_RAM_SIZE (32 * 1024)
#endif /* USING_PTAB_V3 */

#endif  /* __MEM_MAP__ */
