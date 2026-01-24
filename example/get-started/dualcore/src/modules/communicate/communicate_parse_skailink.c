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
#include "watch_global_data.h"
#include "string.h"
#include "bloc_control.h"

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
extern void open_test_mode(bool open);
void resolve_SkaiLink_command(uint8_t key, const uint8_t *pValue,
                              uint16_t length)
{
    switch (key)
    {
    case KEY_UNIT_TEST_UNICODE:
    {
        if (length == 2)
        {
            uint16_t unicode = (pValue[0] << 8) + pValue[1];
            LOG_I("unicode:%x", unicode);
        }
    }
    break;
    case KEY_UNIT_TEST_PAGEVIEW:
    {
        if (length == 1)
        {
            uint8_t action = pValue[0];
            control_provider.notify_pageview_action(action);
        }
    }
    break;
    case KEY_SET_DEBUG_MODE:
    {
        if (length == 1)
        {
            uint8_t mode = pValue[0];
            SkaiWatchSys.flag_field.debug_mode = mode;
            open_test_mode(mode == 1);
        }
    }
    break;
    case KEY_UNIT_TEST_PIN_STATUS_ALL:
    {
#ifdef RT_USING_FINSH
        extern void print_pin_state(int pin);
        if (length == 0)
        {
            for (int pin = 0; pin < 164; pin++)
            {
                print_pin_state(pin);
            }
        }
        else if (length == 1)
        {
            print_pin_state(pValue[0]);
        }
#endif
    }
    break;
    case KEY_UNIT_TEST_ALL:
    {
        if (length == 1)
        {
            uint8_t action = pValue[0];
            control_provider.notify_unit_test_action(action);
        }
    }
    break;
    default:
        break;
    }
}
