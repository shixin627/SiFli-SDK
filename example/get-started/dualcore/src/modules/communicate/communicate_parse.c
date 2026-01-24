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
#include "string.h"
#include "communicate_parse.h"
#include "communicate_protocol.h"

#define DBG_TAG "communicate.parse"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
/**
 * @brief   resolve received data from remote APP
 * @param   pValue: received value pointer
 * @param   length: value length
 * @retval  void
 */
bool L2_frame_resolve(uint8_t *pData, uint16_t length)
{
    if ((pData == NULL) || (length == 0))
    {
        return false;
    }

    WRISTBAND_COMMUNICATE_COMMAND command_id = (WRISTBAND_COMMUNICATE_COMMAND)pData[0];
    uint8_t first_key = pData[2];
    uint16_t first_value_length = (((pData[3] << 8) | pData[4]) & 0x1FF);
    // LOG_D("L2_frame_resolve ==> %b", TRACE_BINARY(length, (uint8_t *)pData));
    switch (command_id)
    {
    case BOND_COMMAND_ID:
    {
        // LOG_D("BOND_COMMAND, KEY = 0x%x", first_key);
        resolve_private_bond_command(first_key, pData + L2_FIRST_VALUE_POS, first_value_length);
    }
    break;

    case SET_CONFIG_COMMAND_ID:
    {
        LOG_D("SET_CONFIG_COMMAND, KEY = 0x%x", first_key);
        resolve_settings_config_command(first_key, pData + L2_FIRST_VALUE_POS, first_value_length);
    }
    break;

    case HEALTH_DATA_COMMAND_ID:
    {
        LOG_D("HEALTH_DATA_COMMAND, KEY = 0x%x", first_key);
        resolve_HealthData_command(first_key, pData + L2_FIRST_VALUE_POS, first_value_length);
    }
    break;

    case NOTIFY_COMMAND_ID:
    {
        LOG_D("NOTIFY_COMMAND_ID, KEY = 0x%x", first_key);
        resolve_Notify_command(first_key, pData + L2_FIRST_VALUE_POS, first_value_length);
    }
    break;

    case CONTROL_COMMAND_ID:
    {
        LOG_D("CONTROL_COMMAND, KEY = 0x%x", first_key);
        resolve_Control_command(first_key, pData + L2_FIRST_VALUE_POS, first_value_length);
    }
    break;

    case SKAI_LINK_COMMAND_ID:
    {
        LOG_D("SKAILINK_COMMAND, KEY = 0x%x", first_key);
        resolve_SkaiLink_command(first_key, pData + L2_FIRST_VALUE_POS, first_value_length);
    }
    break;

    default:
    {
        LOG_E("Unknown command id");
    }
    break;
    }
    return true;
}
