/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse.c
 * @brief
 * @details
 * @author
 * @date
 * @version  v0.1
 *********************************************************************************************************
 */

#include <rtthread.h>
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "communicate_parse_skailink.h"

#define DBG_TAG "commu.parse.skailink"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/**
 * @brief   resolve SkaiLink data command received from remote APP
 * @param   key: L2 key
 * @param   pValue: received value pointer
 * @param   length: value length
 * @retval  error code
 */
void resolve_SkaiLink_command(uint8_t key, const uint8_t *pValue,
                              uint16_t length)
{
    (void)key;
    (void)pValue;
    (void)length;
}
