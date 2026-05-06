/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_parse.h
* @brief
* @details
* @author
* @date
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef __COMMUNICATE_PARSE_H__
#define __COMMUNICATE_PARSE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "communicate_parse_setting.h"
#include "communicate_parse_bond.h"
#include "communicate_parse_control.h"
#include "communicate_parse_health.h"
#include "communicate_parse_notify.h"

    /* Command ID */
    typedef enum
    {
        FIRMWARE_UPDATE_CMD_ID = 0x01,
        SET_CONFIG_COMMAND_ID = 0x02,
        BOND_COMMAND_ID = 0x03,
        NOTIFY_COMMAND_ID = 0x04,
        HEALTH_DATA_COMMAND_ID = 0x05,
        FACTORY_TEST_COMMAND_ID = 0x06,
        CONTROL_COMMAND_ID = 0X07,
        BLUETOOTH_LOG_COMMAND_ID = 0x0a,
        TEST_FLASH_READ_WRITE = 0xFE,
        TEST_COMMAND_ID = 0xFF,
    } WRISTBAND_COMMUNICATE_COMMAND;

    typedef enum
    {
        QQ_NOTIFY_BIT = 0x01,               // BIT 0
        WECHAT_NOTIFY_BIT = 0x01 << 1,      // BIT 1
        SHOTMESSAGE_NOTIFY_BIT = 0x01 << 2, // BIT 2
        LINE_NOTIFY_BIT = 0x01 << 3,        // BIT 3
        GMAIL_NOTIFY_BIT = 0x01 << 4,       // BIT 4
        OTHER_NOTIFY_BIT = 0xff,
    } Incomming_Message_Notify_Bit_t;

    bool L2_frame_resolve(uint8_t *pData, uint16_t length);

    /* Wire-format big-endian byte readers (BLE multi-byte fields are MSB-first). */
    static inline uint32_t read_be32(const uint8_t *p)
    {
        return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
               ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
    }

    static inline uint16_t read_be16(const uint8_t *p)
    {
        return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
    }

    static inline void write_be16(uint8_t *p, uint16_t v)
    {
        p[0] = (uint8_t)(v >> 8);
        p[1] = (uint8_t)v;
    }

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_H__
