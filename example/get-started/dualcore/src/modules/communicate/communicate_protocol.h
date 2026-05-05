/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_protocol.c
* @brief       斯凱沃克通信協議
* @details   communicate_protocol implementation
* @author
* @date      2014-12-29
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef __COMMUNICATE_PROTOCOL_H__
#define __COMMUNICATE_PROTOCOL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "communicate_update_image.h"

#define GLOBAL_RECEIVE_BUFFER_SIZE 244
#define GLOBAL_RESPONSE_BUFFER_SIZE 244
/**************************************************************************
 * Note:
 * big_endian in communication protocol
 * without considering Byte padding
 * use constant shift
 ***************************************************************************/

/*Mobile Command Heading */
#define MOBILE_HEADER_VOICE_RECOGNITION_RESULT (0xA1)
#define MOBILE_HEADER_NOTIFICATION (0x4E)
/*Watch Command Heading */
#define WATCH_COMMAND_HEADER (0xC0)
#define WATCH_COMMAND_HEADER_AUDIO (0xA0)        // --- Audio Command Heading
#define WATCH_COMMAND_HEADER_NOTIFICATION (0x4E) // --- Notification Command Heading [N]
#define WATCH_DATA_HEADER_ACTIVITY (0xAC)        // Activity Data Heading [Ac]
#define WATCH_DATA_HEADER_GSENSOR (0x47)         // Gsensor Data Heading [G]

#define L2_HEADER_SIZE (2)       /*L2 header length*/
#define L2_HEADER_VERSION (0x00) /*L2 header version*/
#define L2_KEY_SIZE (1)
#define L2_PAYLOAD_HEADER_SIZE (3) /*L2 payload header*/

#define L2_FIRST_VALUE_POS (L2_HEADER_SIZE + L2_PAYLOAD_HEADER_SIZE)

    typedef enum
    {
        NONE = 0x00,
        IOS = 0x01,
        ANDROID = 0x02,
    } PHONE_OS_VERSION;

    /******************* Function definition **********************************/
    extern void skaiwatch_ble_set_performance(bool status);
    extern void ble_station_entry(void *parameter);
    extern void communicate_protocol_init(void);
    extern bool skaiwatch_ble_notify(uint8_t *buf, uint16_t length);
    extern bool skaiwatch_ble_audio_send(uint8_t *buf, uint16_t length);

    extern void L1_receive_data_without_crc_check(uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PROTOCOL_H__
