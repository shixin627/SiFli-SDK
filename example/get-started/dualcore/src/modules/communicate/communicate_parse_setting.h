/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_parse_setting.h
* @brief
* @details
* @author
* @date
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef __COMMUNICATE_PARSE_SETTING_H__
#define __COMMUNICATE_PARSE_SETTING_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"

    typedef enum
    {
        KEY_TIME_SETTINGS = 0x01,
        KEY_ALARM_SETTINGS = 0x02,
        KEY_REQUEST_ALARM_SETTINGS = 0x03,
        KEY_RETURN_ALARM_SETTINGS = 0x04,
        KEY_STEP_TARGET_SETTINGS = 0x05,
        KEY_SLEEP_TARGET_SETTINGS = 0x06,
        KEY_PROFILE_SETTINGS = 0x10,
        KEY_DEV_LOSS_ALERT_SETTINGS = 0x20,
        KEY_PHONE_OS_VERSION = 0x23,
        KEY_INCOMMING_MESSAGE_SETTINGS = 0x25,
        KEY_DIAL_SETTING = 0x38,
        KEY_DIAL_REQUEST = 0x39,
        KEY_DIAL_RETURN = 0x3A,
        KEY_HOUR_FORMAT_SETTING = 0x41,
        KEY_HOUR_FORMAT_REQUEST = 0x42,
        KEY_HOUR_FORMAT_RETURN = 0x43,
        KEY_DISTANCE_UNIT_SETTING = 0x44,
        KEY_DISTANCE_UNIT_REQUEST = 0x45,
        KEY_DISTANCE_UNIT_RETURN = 0x46,
        KEY_OLED_DISPLAY_TIME_SETTING = 0x4A,
        KEY_OLED_DISPLAY_TIME_REQUEST = 0x4B,
        KEY_OLED_DISPLAY_TIME_RETURN = 0x4C,
        KEY_LANGUAGE_SETTING = 0x4E,
        KEY_LANGUAGE_REQUEST = 0x4F,
        KEY_LANGUAGE_RETURN = 0x50,
        KEY_DEVICEINFO_REQUEST = 0x51,
        KEY_DEVICEINFO_RETURN = 0x52,
        KEY_GESTURE_ACCEL_LIMIT_SETTING = 0x67,
    } SETTINGS_KEY;

    void resolve_settings_config_command(uint8_t key, const uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_SETTING_H__
