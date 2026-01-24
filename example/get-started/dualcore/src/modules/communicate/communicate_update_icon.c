enum
{
	__FILE_NUM__ = 0
};

/*********************************************************************************************************
 *               Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_update_icon.c
 * @brief    Update icon binary in flash
 * @details
 * @author   Jack
 * @date     2023-07-15
 * @version  v1.0
 *********************************************************************************************************
 */

#include "board.h"
#include "trace.h"
#include <string.h>
#include "ftl.h"
#include "app_flash_config.h"
#include "sync_data.h"
#include "communicate_parse.h"
#include "wristband_global_data.h"
#include "os_sync.h"
#include "communicate_update_icon.h"
#include "wristband_picture_res.h"
#include "bloc_flash.h"

#define BINARY_ICON_SIZE_MAX ICON_UPDATE_MAX_SIZE // 0.5MB

volatile uint32_t watch_icon_write_addr;

void WristBandIconBlockInit(void)
{
	watch_icon_write_addr = ICON_UPDATE_START_ADDRESS;
	uint8_t prev_bp_lv = 0;
	flash_sw_protect_unlock_by_addr_locked(ICON_UPDATE_START_ADDRESS, &prev_bp_lv);
	uint32_t s;
	s = os_lock();
	flash_erase_locked(FLASH_ERASE_SECTOR, ICON_UPDATE_START_ADDRESS);
	flash_erase_locked(FLASH_ERASE_SECTOR, ICON_UPDATE_START_ADDRESS + FLASH_SECTOR_SIZE);
	os_unlock(s);
	flash_set_block_protect_locked(prev_bp_lv);
}

bool WristBandBinaryIconStore(uint8_t *buf, uint16_t len)
{
	bool isValidRegion = bloc_flash_check_image_write_address(len, watch_icon_write_addr, ICON_UPDATE_START_ADDRESS, BINARY_ICON_SIZE_MAX);
	if (!isValidRegion)
	{
		return false;
	}

	uint8_t prev_bp_lv = 0;
	flash_sw_protect_unlock_by_addr_locked(ICON_UPDATE_START_ADDRESS, &prev_bp_lv);
	bloc_flash_erase_flash_sector(watch_icon_write_addr, ICON_UPDATE_START_ADDRESS, BINARY_FILE_PREPARED_ERASE_SECTOR);
	bloc_flash_write_per4bytes(watch_icon_write_addr, buf, len);
	watch_icon_write_addr = watch_icon_write_addr + len;
	flash_set_block_protect_locked(prev_bp_lv);
	return true;
}

void watch_binary_icon_update_handler(uint8_t *buf, uint16_t len)
{
	APP_PRINT_INFO1("watch binary icon update handler(address:%x)", watch_icon_write_addr);
	WristBandBinaryIconStore(buf, len);
}
