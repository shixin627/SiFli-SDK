enum
{
	__FILE_NUM__ = 0
};

/*********************************************************************************************************
 *               Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_update_font.c
 * @brief    Update font binary in flash
 * @details
 * @author   jack
 * @date     2023-03-12
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
#include "communicate_update_font.h"
#include "wristband_picture_res.h"
#include "bloc_flash.h"

#define BINARY_FONT_SIZE_MAX 0x00800000 // 8MB
volatile uint32_t watch_font_write_addr;

void WristBandFontBlockInit(void)
{
	watch_font_write_addr = FONT_UPDATE_START_ADDRESS;
	uint8_t prev_bp_lv = 0;
	flash_sw_protect_unlock_by_addr_locked(FONT_UPDATE_START_ADDRESS, &prev_bp_lv);
	uint32_t s;
	s = os_lock();
	flash_erase_locked(FLASH_ERASE_SECTOR, FONT_UPDATE_START_ADDRESS);
	flash_erase_locked(FLASH_ERASE_SECTOR, FONT_UPDATE_START_ADDRESS + FLASH_SECTOR_SIZE);
	os_unlock(s);
	flash_set_block_protect_locked(prev_bp_lv);
}

bool WristBandBinaryFontStore(uint8_t *buf, uint16_t len)
{
	bool isValidRegion = bloc_flash_check_image_write_address(len, watch_font_write_addr, FONT_UPDATE_START_ADDRESS, BINARY_FONT_SIZE_MAX);
	if (!isValidRegion)
	{
		return false;
	}
	uint8_t prev_bp_lv = 0;
	flash_sw_protect_unlock_by_addr_locked(FONT_UPDATE_START_ADDRESS, &prev_bp_lv);

	bloc_flash_erase_flash_sector(watch_font_write_addr, FONT_UPDATE_START_ADDRESS, BINARY_FILE_PREPARED_ERASE_SECTOR);

	bloc_flash_write_per4bytes(watch_font_write_addr, buf, len);

	watch_font_write_addr = watch_font_write_addr + len;
	flash_set_block_protect_locked(prev_bp_lv);
	return true;
}

void watch_binary_font_update_handler(uint8_t *buf, uint16_t len)
{
	APP_PRINT_INFO1("watch binary font update handler(address:%x)", watch_font_write_addr);
	WristBandBinaryFontStore(buf, len);
}
