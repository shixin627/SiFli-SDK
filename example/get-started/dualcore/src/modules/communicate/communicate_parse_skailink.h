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

	void resolve_SkaiLink_command(uint8_t key, const uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_SKAILINK_H__
