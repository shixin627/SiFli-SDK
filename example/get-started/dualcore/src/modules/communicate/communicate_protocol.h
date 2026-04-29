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

/******************* Macro defination *************************************/
#define L1_HEADER_MAGIC (0xAB)   /*header magic number */
#define L1_HEADER_VERSION (0x00) /*protocol version */
#define L1_HEADER_SIZE (8)       /*L1 header length*/

/**************************************************************************
 * define L1 header byte order
 ***************************************************************************/
#define L1_HEADER_MAGIC_POS (0)
#define L1_HEADER_PROTOCOL_VERSION_POS (1)
#define L1_PAYLOAD_LENGTH_HIGH_BYTE_POS (2) /* L1 payload lengh high byte */
#define L1_PAYLOAD_LENGTH_LOW_BYTE_POS (3)
#define L1_HEADER_CRC16_HIGH_BYTE_POS (4)
#define L1_HEADER_CRC16_LOW_BYTE_POS (5)
#define L1_HEADER_SEQ_ID_HIGH_BYTE_POS (6)
#define L1_HEADER_SEQ_ID_LOW_BYTE_POS (7)

    /********************************************************************************
     * define version response
     *********************************************************************************/
    typedef enum
    {
        DATA_PACKAGE = 0,
        RESPONSE_PACKAGE = 1,
    } L1_PACKAGE_TYPE;

    /********************************************************************************
     * define ack or nak
     *********************************************************************************/
    typedef enum
    {
        ACK = 0,
        NAK = 1,
    } L1_ERROR_FLAG;

#define L2_HEADER_SIZE (2)       /*L2 header length*/
#define L2_HEADER_VERSION (0x00) /*L2 header version*/
#define L2_KEY_SIZE (1)
#define L2_PAYLOAD_HEADER_SIZE (3) /*L2 payload header*/

#define L2_FIRST_VALUE_POS (L2_HEADER_SIZE + L2_PAYLOAD_HEADER_SIZE)

    /******************* Enum & Struct defination ******************************/
    /*L1 version defination */
    typedef struct
    {
        uint8_t version : 4;
        uint8_t ack_flag : 1;
        uint8_t err_flag : 1;
        uint8_t reserve : 2;
    } L1_version_def_t;

    typedef union
    {
        L1_version_def_t version_def;
        uint8_t value;
    } L1_version_value_t;

    typedef enum
    {
        NONE = 0x00,
        IOS = 0x01,
        ANDROID = 0x02,
    } PHONE_OS_VERSION;

    typedef enum
    {
        KEY_WEATHER_GET = 0x01,
        KEY_WEATHER_CURRENT = 0x02,
        KEY_WEATHER_FUTURE_HOUR = 0x03,
        KEY_WEATHER_FUTURE_DAY = 0x04,
    } WEATHER_KEY;

    typedef enum
    {
        WEATHER_SUCCESS = 0x0,
        WEATHER_CITY_INVALID = 0x01,
        WEATHER_SERVICE_INVALID = 0x02,
    } WEATHER_ERROR_CODE;

    /******************* Function definition **********************************/
    extern void skaiwatch_ble_set_performance(bool status);
    extern void ble_station_entry(void *parameter);
    extern void communicate_protocol_init(void);
    extern bool L1_send(uint8_t *buf, uint16_t length);
    extern void L1_receive_data(uint8_t *data, uint16_t length);
    extern bool skaiwatch_ble_notify(uint8_t *buf, uint16_t length);
    extern bool skaiwatch_ble_audio_send(uint8_t *buf, uint16_t length);

    extern void L1_receive_data_without_crc_check(uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PROTOCOL_H__
