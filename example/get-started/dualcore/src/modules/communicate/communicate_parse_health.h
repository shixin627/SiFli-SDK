/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_parse_health.h
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
    /* Background daily HR-curve sample {timestamp:u32 LE, bpm:u8}. Distinct
       from KEY_HEART_DATA_RETURN (0x0F, live single value during exercise):
       this is the timestamped, sparse (~15 min) ambient HR that the phone
       accumulates into a daily heart-rate curve. */
    KEY_HEART_CURVE_SAMPLE = 0x10,
} HEALTH_KEY;

void resolve_HealthData_command(uint8_t key, const uint8_t *pValue, uint16_t length);

/* Persist today's sleep summary to /health/sleep_YYYYMMDD.json from the current
   SkaiWatchSys.sleep_state. Called on each stage transition so the file is
   always current; it is store-and-forwarded to the phone on reconnect (the
   KEY_REQUEST_DATA handler), which is the only way overnight sleep survives a
   BLE disconnect (the live KEY_RETURN_SLEEP_DATA push is dropped when offline).
   No-op while no sleep has accumulated today. */
void commu_health_save_sleep_file(void);



#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_HEALTH_H__
