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
        KEY_INCOMMING_MESSAGE_SETTINGS_REQUEST = 0x28,
        KEY_INCOMMING_MESSAGE_SETTINGS_RETURN = 0x29,
        KEY_LIFT_SWITCH_SETTING = 0x2A,
        KEY_LIFT_SWITCH_REQUEST = 0x2B,
        KEY_LIFT_SWITCH_RETURN = 0x2C,
        KEY_INCOMMING_MESSAGE_ALL_SETTINGS = 0x2D,
        KEY_TWIST_SWITCH_SETTING = 0x30,
        KEY_TWIST_SWITCH_REQUEST = 0x31,
        KEY_TWIST_SWITCH_RETURN = 0x32,
        KEY_DISPLAY_SWITCH_SETTING = 0x33,
        KEY_DISPLAY_SWTICH_REQUEST = 0x34,
        KEY_DISPLAY_SWTICH_RETURN = 0x35,
        KEY_FUNCTIONS_REQUEST = 0x36,
        KEY_FUNCTIONS_RETURN = 0x37,
        KEY_DIAL_SETTING = 0x38,
        KEY_DIAL_REQUEST = 0x39,
        KEY_DIAL_RETURN = 0x3A,
        KEY_EXERCISEMODE_REQUEST = 0x3B,
        KEY_BEIKE_MSG_SETTING = 0x3C,
        KEY_BEIKE_MSG_REQUEST = 0x3D,
        KEY_BEIKE_MSG_RETURN = 0x3E,
        KEY_HR_SAMPLE_REQUEST = 0x3F,
        KEY_HOUR_FORMAT_SETTING = 0x41,
        KEY_HOUR_FORMAT_REQUEST = 0x42,
        KEY_HOUR_FORMAT_RETURN = 0x43,
        KEY_DISTANCE_UNIT_SETTING = 0x44,
        KEY_DISTANCE_UNIT_REQUEST = 0x45,
        KEY_DISTANCE_UNIT_RETURN = 0x46,
        KEY_DNDM_SETTING = 0x47, /* Do not disturb mode setting */
        KEY_DNDM_REQUEST = 0x48, /* Do not disturb mode request */
        KEY_DNDM_RETURN = 0x49,  /* Do not disturb mode return  */
        KEY_OLED_DISPLAY_TIME_SETTING = 0x4A,
        KEY_OLED_DISPLAY_TIME_REQUEST = 0x4B,
        KEY_OLED_DISPLAY_TIME_RETURN = 0x4C,
        KEY_LANGUAGE_SETTING = 0x4E,
        KEY_LANGUAGE_REQUEST = 0x4F,
        KEY_LANGUAGE_RETURN = 0x50,
        KEY_DEVICEINFO_REQUEST = 0x51,
        KEY_DEVICEINFO_RETURN = 0x52,
        KEY_BACKLIGHT_SETTING = 0x53,
        KEY_BACKLIGHT_REQUEST = 0x54,
        KEY_BACKLIGHT_RETURN = 0x55,
        KEY_HIDDEN_FUNC_SETTING = 0x59,
        KEY_HIDDEN_FUNC_REQUEST = 0x5A,
        KEY_HIDDEN_FUNC_RETURN = 0x5B,
        KEY_BBPRO_MAC_REQUEST = 0x60,
        KEY_BBPRO_STATE_REQUEST = 0x61,
        KEY_BBPRO_CONNECTED_STATE_REQUEST = 0x62,
        KEY_BBPRO_CONNECTED_STATE_RETURN = 0x63,
        KEY_BBPRO_CREATE_CONNECTION_REQUEST = 0x64,
        KEY_MOTOR_STRENGTH_SETTING = 0x65,
        KEY_MOTOR_PERIOD_SETTING = 0x66,
        KEY_GESTURE_ACCEL_LIMIT_SETTING = 0x67,
        KEY_ACCEL_SUBSCRIBE_REQUEST = 0x68,
        KEY_HR_SUBSCRIBE_REQUEST = 0x6A,
        KEY_AUDIO_SUBSCRIBE_REQUEST = 0x6B,
    } SETTINGS_KEY;

    void resolve_settings_config_command(uint8_t key, const uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_SETTING_H__
