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

    typedef enum
    {
        L1SEND_SPORT_AND_SLEEP_DATA = 1,
        L1SEND_BOND_FAIL_EVENT = 2,
        L1SEND_BOND_SUCCESS_EVENT = 3,
        L1SEND_LOGIN_FAIL_EVENT = 4,
        L1SEND_LOGIN_SUCCESS_EVENT = 5,
        L1SEND_RETURN_ALARM_EVENT = 6,
        L1SEND_RETURN_PHONE_CONTROL_CMD_EVENT = 7,
        L1SEND_RETURN_LIFT_SWITCH_EVENT = 8,
        L1SEND_RETURN_INCOMMING_MESSAGE_SETTINGS = 9,
        L1SEND_SIT_SETTING_RETURN = 10,
        L1SEND_HISTORY_DATA_SYNC_START = 11,
        L1SEND_HISTORY_DATA_SYNC_END = 12,
        L1SEND_SPORT_DATA = 13,
        L1SEND_SLEEP_DATA = 14,
        L1SEND_HEART_DATA = 15,
        L1SEND_BLOODPRESSURE_DATA = 16,
        L1SEND_EXERCISE_DATA = 17,
        L1SEND_RETURN_CANCEL_HEART_SAMPLE = 18,
        L1SEND_RETURN_HEART_SETTING = 19,
        L1SEND_RETURN_CALL_REJECT_COMMAND = 20,
        L1SEND_RETURN_FIND_MOBILE_COMMAND = 21,
        L1SEND_RETURN_FUNCTIONS_EVENT = 22,
        L1SEND_RETURN_CANCEL_BP_SAMPLE = 23,
        L1SEND_RETURN_HOUR_FORMAT_SETTING = 24,
        L1SEND_RETURN_DISTANCE_UNIT_SETTING = 25,
        L1SEND_RETURN_DNDM_SETTING = 26,
        L1SEND_RETURN_OLED_DISPLAY_TIME = 27,
        L1SEND_RETURN_LANGUAGE = 28,
        L1SEND_RETURN_TWIST_SWITCH_EVENT = 29,
        L1SEND_RETURN_CHARGE_STATUS = 31,
        L1SEND_RETURN_WEATHER_DATA_GET = 32,
        L1SEND_RETURN_CALENDAR_DATA_GET = 33,
        L1SEND_RETURN_DEVICE_INFO = 34,
        L1SEND_RETURN_DIAL_CHANGE = 35,
        L1SEND_RETURN_EXERCISEMODE_EVENT,
        L1SEND_RETURN_ANCS_INCOMMING_CALL,
        L1SEND_RETURN_HR_SAMPLE_EVENT,
        L1SEND_RETURN_BACKLIGHT_EVENT,
        L1SEND_RETURN_CHARGE_EVENT,
        L1SEND_RETURN_GSENSOR_ID_EVENT,
        L1SEND_RETURN_HR_ID_EVENT,
        L1SEND_RETURN_LOCK,
        L1SEND_RETURN_FACTORY_END_EVENT,
        L1SEND_RETURN_HIDDEN_FUNC,
        L1SEND_RETURN_BBPRO_CONNECTED_STATE,
        L1SEND_RETURN_BBPRO_CONN_INFO,
        L1SEND_RETURN_SOFT_ADT_STATUS,
        L1SEND_LINEAR_ACCE_BUFFER,
        L1SEND_IMU_BUFFER,
        L1SEND_BARO_BUFFER,
        L1SEND_GSENSOR_FFT_BUFFER,
        L1SEND_GSENSOR_PPG_BUFFER,
        L1SEND_GSENSOR_GRAVITY_DATA,
        L1SEND_BLUETOOTH_LOG,
        L1SEND_RETURN_VOICE2TEXT_INTENT,
        L1SEND_RETURN_VOICE_RECORD_INTENT,
        L1SEND_COORDINATE,
        L1SEND_RETURN_GESTURE_MODE_STATE,
        L1SEND_GESTURE_DETECT,
        L1SEND_REMOTE_INPUT,
        L1SEND_DISMISS_NOTIFICATION,
        L1SEND_CREATE_NOTE,
        L1SEND_UPDATE_NOTE,
        L1SEND_DELETE_NOTE,
        L1SEND_CREATE_CALENDAR,
        L1SEND_UPDATE_CALENDAR,
        L1SEND_DELETE_CALENDAR,
        L1SEND_HEART_RATE_SERIES,
        L1SEND_WATCH_SYSTEM_SYNC,
        L1SEND_MEDIA_CONTROL,
        L1SEND_MQTT_CONTROL,

        L1SEND_VOLUME_PERCENTAGE,
        L1SEND_RETURN_TP_COORDINATE,
        L1SEND_RETURN_TP_GESTURE,
        L1SEND_RETURN_USER_SPEAKING_STATE,
        L1SEND_CHAT_WITH_AI,
        L1SEND_QUATERNION_DATA,
        L1SEND_RETURN_BATTERY_VOLTAGE,
        L1SEND_RETURN_BATTERY_LEVEL,
        L1SEND_AUDIO_DATA,
        L1SEND_AUDIO_FILE,
        L1SEND_HOLDING_DISPLACEMENT,

        // ---sync file start
        L1SEND_START_SYNC_FILE,
        L1SEND_SYNC_FILE,
        L1SEND_END_SYNC_FILE,
        L1SEND_FILE_COMPARE_RESULT,
        // ---sync file end
        L1SEND_VIRTUAL_GESTURE,
        L1SEND_FINGER_TAP,
        L1SEND_CURSOR_MOVEMENT,
        L1SEND_VIRTUAL_MOVEMENT,
        L1SEND_RETURN_OTA_STATUS,
        L1SEND_RETURN_UTEST_STATE,
        L1SEND_RETURN_MINUTE_ACTIVITY,
        L1SEND_UPDATE_INSTRUCTION,
        L1SEND_INVALID,

        // L1SEND_CREATE_TASK,
        // L1SEND_TOGGLE_TASK,
        // L1SEND_TASK_SYNC_START,
        // L1SEND_TASK_SYNC_END,
        // L1SEND_UPDATE_TASK,
    } L1SEND_TYPE_WATCH;

    typedef struct
    {
        uint32_t *data;
        uint16_t length;
    } sensor_buf32_t;

    typedef struct
    {
        uint32_t timestamp;
        uint8_t *data;
        uint16_t length;
    } sensor_buf_t;

    typedef struct
    {
        uint8_t *data;
        uint16_t length;
    } file_buf_t;

    typedef struct
    {
        float *data;
        uint16_t length;
    } sensor_buf_float_t;

    typedef struct
    {
        uint8_t event;
        int x;
        int y;
    } holding_displacement_t;

    typedef struct
    {
        uint8_t direction;
        uint8_t distance;
        uint8_t is_tap;
    } virtual_movement_t;

    typedef union
    {
        int hr;
        float baro_data;
        uint8_t bt_speaker_volume;
        uint8_t lcd_brightness;
        uint8_t lcd_display_time;
        char *log_buffer_ptr;
        char *json_string_ptr;
        // sensor_buf32_t ppg_data;
        sensor_buf_float_t ppg_data;
        void *ppg_diff_ptr;
        sensor_buf_t imu_data;
        uint8_t voice2text_intent;
        sensor_buf_t audio_buffer;
        file_buf_t file_buffer;
        uint8_t gesture_label;
        uint8_t coordinate;
        virtual_movement_t virtual_movement;
        uint8_t status;
        uint8_t battery_level;
        uint16_t battery_voltage;
        uint32_t id;
        holding_displacement_t holding_displacement;
    } ResUnion;

    typedef struct
    {
        L1SEND_TYPE_WATCH event;
        ResUnion res;
    } L1SendData;

    /******************* Function definition **********************************/
    extern void skaiwatch_ble_set_performance(bool status);
    extern void ble_station_entry(void *parameter);
    extern void communicate_protocol_init(void);
    extern bool L1_send(uint8_t *buf, uint16_t length);
    extern void L1_receive_data(uint8_t *data, uint16_t length);
    // extern void L1_send_event(L1SendData data);
    extern int L1_send_event(L1SendData data);
    extern void L1_send_ota_event(L1SendData data);
    extern bool skaiwatch_ble_notify(uint8_t *buf, uint16_t length);
    extern bool skaiwatch_ble_audio_send(uint8_t *buf, uint16_t length);

    extern void L1_receive_data_without_crc_check(uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PROTOCOL_H__
