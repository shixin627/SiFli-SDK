/**
*********************************************************************************************************
*               Copyright(c) 2024, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_parse_factory.h
* @brief        FACTORY_TEST_COMMAND_ID (0x06) L2 keys + handler.
* @details      The factory test app is hidden from the watch UI and is only
*               reachable via this command, sent from the phone development page.
* *********************************************************************************************************
*/

#ifndef __COMMUNICATE_PARSE_FACTORY_H__
#define __COMMUNICATE_PARSE_FACTORY_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"

	typedef enum
	{
		KEY_OPEN_TEST_APP = 0x01, /* launch the hidden factory test app */
	} FACTORY_TEST_KEY;

	void resolve_factory_test_command(uint8_t key, const uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_FACTORY_H__
