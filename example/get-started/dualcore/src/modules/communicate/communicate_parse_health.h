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

#ifndef __COMMUNICATE_PARSE_HEALTH_H__
#define __COMMUNICATE_PARSE_HEALTH_H__


#ifdef __cplusplus
extern "C" {
#endif



#include <stdbool.h>
#include "stdint.h"

typedef enum
{
    KEY_REQUEST_DATA   = 0x01,
    KEY_RETURN_SPORTS_DATA  = 0x02,
    KEY_RETURN_SLEEP_DATA  = 0x03,
    KEY_DAILY_DATA_SYNC = 0x09,
    KEY_LATEST_DATA_SYNC = 0x0A,
    KEY_REQUEST_HEART_DATA = 0x0D,
    KEY_HEART_DATA_RETURN = 0x0F,
} HEALTH_KEY;

void resolve_HealthData_command(uint8_t key, const uint8_t *pValue, uint16_t length);



#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_HEALTH_H__
