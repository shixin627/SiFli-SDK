/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse_skailink.h
 * @brief
 * @details
 * @author   shixin
 * @date     2023-05-23
 * @version  v1.0
 *********************************************************************************************************
 */

#ifndef __COMMUNICATE_PARSE_SKAILINK_H__
#define __COMMUNICATE_PARSE_SKAILINK_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"

	typedef enum
	{
		SKAILINK_KEY_BROWSER = 0x01,
		SKAILINK_KEY_SET_PRESENTATION = 0x02,
		SKAILINK_KEY_RETURN_PRESENTATION_ID = 0x03,
		SKAILINK_KEY_SET_PRESENT_DATA = 0x04,
		SKAILINK_KEY_RETURN_SLIDE_COMMAND = 0x06,
		SKAILINK_KEY_SET_SLIDE_URL = 0x07,
		SKAILINK_KEY_SET_NOTE_ID = 0x08,
		SKAILINK_KEY_SKAIOS_MODE_STATE = 0x09,
		SKAILINK_KEY_RETURN_CURTAIN_STATUS = 0x0A,
		SKAILINK_KEY_SEND_APP_ONPRESS_INDEX = 0x0B,
		// test
		KEY_UNIT_TEST_UNICODE = 0XE0,
		KEY_UNIT_TEST_PAGEVIEW = 0XE1,
		KEY_UNIT_TEST_PIN_STATUS_ALL = 0XE2,
		KEY_SET_DEBUG_MODE = 0XE3,
		KEY_UNIT_TEST_SYNC = 0XFE,
		KEY_UNIT_TEST_ALL = 0XFF,
	} SKAILINK_KEY;

	void resolve_SkaiLink_command(uint8_t key, const uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_SKAILINK_H__
