/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_parse_control.h
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
		KEY_REBOOT = 0X24,
		KEY_SLEEP = 0X26,
		KEY_WAKEUP = 0X27,
		KEY_APP_RUN = 0X29,
	} CONTROL_KEY;

	void resolve_Control_command(uint8_t key, const uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_CONTROL_H__
