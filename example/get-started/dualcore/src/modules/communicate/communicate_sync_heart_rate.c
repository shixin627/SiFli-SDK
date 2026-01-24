enum { __FILE_NUM__ = 0 };

/*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     app_flash.c
* @brief
* @details
* @author
* @date     2015-04-29
* @version  v0.1
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
#include "communicate_sync_heart_rate.h"

void WristBandHeartDataBlockInit(void)
{
    RtkWristbandSys.heart_address.p_write_addr = HEART_DATA_START_ADDRESS;
    RtkWristbandSys.heart_address.p_read_addr = HEART_DATA_START_ADDRESS;
    uint8_t prev_bp_lv = 0;
    flash_sw_protect_unlock_by_addr_locked(HEART_DATA_START_ADDRESS, &prev_bp_lv);
    uint32_t s;
    s = os_lock();
    flash_erase_locked(FLASH_ERASE_SECTOR, HEART_DATA_START_ADDRESS);
    flash_erase_locked(FLASH_ERASE_SECTOR, HEART_DATA_START_ADDRESS + FLASH_SECTOR_SIZE);
    os_unlock(s);
    flash_set_block_protect_locked(prev_bp_lv);
}


bool WristBandHeartRateDataStore(uint8_t *buf, uint16_t len)
{
    if ((RtkWristbandSys.heart_address.p_write_addr + len) > HEART_DATA_START_ADDRESS + 2 *
        FLASH_SECTOR_SIZE)
    {
        APP_PRINT_ERROR0("Pedo data expire MAX flash size!");
        return false;
    }
    else
    {
        uint8_t prev_bp_lv = 0;
        flash_sw_protect_unlock_by_addr_locked(HEART_DATA_START_ADDRESS, &prev_bp_lv);

        uint32_t data = 0;
        for (uint8_t i = 0; i < len; i = i + 4)
        {
            memcpy(&data, buf + i, 4);
            if (flash_auto_write_locked(RtkWristbandSys.heart_address.p_write_addr + i, data) == false)
            {
                APP_PRINT_ERROR0("Heart data save error!");
            }
        }
        RtkWristbandSys.heart_address.p_write_addr = RtkWristbandSys.heart_address.p_write_addr + len;

        flash_set_block_protect_locked(prev_bp_lv);
        return true;
    }
}

static bool WristBandHeartRateDataRestore(uint8_t *buf, uint16_t len)
{
    if ((len % 4) != 0)
    {
        return false;
    }
    if ((RtkWristbandSys.heart_address.p_read_addr + len) > RtkWristbandSys.heart_address.p_write_addr)
    {
        return false;
    }
    if (RtkWristbandSys.heart_address.p_read_addr == RtkWristbandSys.heart_address.p_write_addr)
    {
        return false;
    }
    uint32_t data = 0;
    for (uint8_t i = 0; i < len; i = i + 4)
    {
        if (flash_auto_read_locked((RtkWristbandSys.heart_address.p_read_addr + i) | 0x4000000,
                                   &data) == false)
        {
            APP_PRINT_ERROR0("heart data read error!");
        }
        memcpy(buf + i, &data, 4);
    }
    RtkWristbandSys.heart_address.p_read_addr = RtkWristbandSys.heart_address.p_read_addr + len;
    /* we had better do garbage collection here */
    if ((RtkWristbandSys.heart_address.p_read_addr > HEART_DATA_START_ADDRESS + FLASH_SECTOR_SIZE) && \
        (RtkWristbandSys.heart_address.p_write_addr > HEART_DATA_START_ADDRESS + FLASH_SECTOR_SIZE) && \
        (RtkWristbandSys.heart_address.p_read_addr == RtkWristbandSys.heart_address.p_write_addr))
    {
        RtkWristbandSys.heart_address.p_write_addr = HEART_DATA_START_ADDRESS;
        RtkWristbandSys.heart_address.p_read_addr = HEART_DATA_START_ADDRESS;
        uint8_t prev_bp_lv = 0;
        flash_sw_protect_unlock_by_addr_locked(HEART_DATA_START_ADDRESS, &prev_bp_lv);
        uint32_t s;
        s = os_lock();
        flash_erase_locked(FLASH_ERASE_SECTOR, HEART_DATA_START_ADDRESS);
        flash_erase_locked(FLASH_ERASE_SECTOR, HEART_DATA_START_ADDRESS + FLASH_SECTOR_SIZE);
        os_unlock(s);
        flash_set_block_protect_locked(prev_bp_lv);
        APP_PRINT_WARN0("HeartRate garbage collection !");
    }
    else
    {
        APP_PRINT_INFO0("no need do garbage collection !");
    }
    return true;
}

void heart_rate_monitor_handler(uint8_t heartRate)
{
    uint8_t buffer[8] = {0};
    HeartData_U    mHeartData;
    HeartHead_t    mHeartHead;

    mHeartHead.Date.date.day = RtkWristbandSys.Global_Time.day;
		mHeartHead.Date.date.month = RtkWristbandSys.Global_Time.month;
		mHeartHead.Date.date.year = RtkWristbandSys.Global_Time.year - 2000;
		mHeartHead.length = 1;

		mHeartData.bits.heart_rate = heartRate;
		mHeartData.bits.heart_second = 0;
		mHeartData.bits.timeStamp = 0;
		
		buffer[0] = mHeartHead.Date.data >> 8;
		buffer[1] = mHeartHead.Date.data;
		buffer[2] = 0;
		buffer[3] = mHeartHead.length;
		buffer[4] = mHeartData.data >> 24;
		buffer[5] = mHeartData.data >> 16;
		buffer[6] = mHeartData.data >> 8;
		buffer[7] = mHeartData.data;

		APP_PRINT_INFO0("save heart rate data\n");
		WristBandHeartRateDataStore(buffer, HEART_ITEM_LENGTH + HEART_HEAD_LENGTH);
}

void send_heart_data(void)
{
    uint8_t buffer[HEART_ITEM_LENGTH + HEART_HEAD_LENGTH];
    uint8_t psendbuf[0x20];
    psendbuf[0] = HEALTH_DATA_COMMAND_ID;                     /* command id*/
    psendbuf[1] = L2_HEADER_VERSION;                          /* L2 header version */
    psendbuf[2] = KEY_HEART_DATA_RETURN;                      /* first key, */
    psendbuf[3] = 0;                                          /* length high */
    psendbuf[4] = HEART_HEAD_LENGTH + HEART_ITEM_LENGTH;      /* length low */
    while (WristBandHeartRateDataRestore(buffer, HEART_ITEM_LENGTH + HEART_HEAD_LENGTH) == true)
    {
        memcpy(psendbuf + 5, buffer, HEART_HEAD_LENGTH + HEART_ITEM_LENGTH);
        if (wristband_send(psendbuf, 5 + HEART_HEAD_LENGTH + HEART_ITEM_LENGTH) == false)
        {
            WristBandHeartRateDataStore(buffer, HEART_HEAD_LENGTH + HEART_ITEM_LENGTH);
            break;
        }
    }
}
