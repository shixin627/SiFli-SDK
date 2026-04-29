/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_protocol.c
* @brief
* @details
* @author
* @date
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef __COMMUNICATE_PARSE_CONTROL_H__
#define __COMMUNICATE_PARSE_CONTROL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"

	typedef enum
	{
		KEY_TAKE_PHOTO = 0X01,
		KEY_FIND_PHONE = 0X02,
		KEY_FIND_WATCH = 0X03,
		KEY_PHONE_MEDIA_CONTROL = 0X04,
		KEY_PHONE_MEDIA_STATUS = 0X05,
		KEY_PHONE_VOLUMN = 0X06,
		KEY_RETURN_VOLUMN = 0X07,
		KEY_PHONE_CAMERA_STATUS = 0X11,
		KEY_RETURN_VOICE2TEXT_INTENT = 0X13,
		KEY_RETURN_VOICE_RECORD_INTENT = 0X15,
		KEY_GESTURE_MODE_STATUS = 0X16,
		KEY_VIRTUAL_GESTURE = 0X17,
		KEY_TP_COORDINATE = 0X20,
		KEY_TP_GESTURE = 0X21,
		KEY_AUDIO_RECORD = 0X22,
		KEY_AUDIO_PLAY = 0X23,
		KEY_REBOOT = 0X24,
		KEY_SHUTDOWN = 0X25,
		KEY_SLEEP = 0X26,
		KEY_WAKEUP = 0X27,
		KEY_MIC_LISTEN = 0X28,
		KEY_APP_RUN = 0X29,
		KEY_COUNTROL_KEYBOARD = 0X2A,
		KEY_MQTT_CONTROL = 0X2B,
		KEY_PASTE_TEXT = 0X2C,
	} CONTROL_KEY;

	void resolve_Control_command(uint8_t key, const uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_CONTROL_H__
