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
#include "watch_global_data.h"
#include "bloc_peripheral.h"
#include "bloc_weather.h"
#include "watch_system_interact.h"

#define DBG_TAG "commu.parse.bond"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
/* disconnect timer */
rt_timer_t unbond_disc_timer = RT_NULL;

void unbond_timerout_callback(void *pxTimer)
{
    if (SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_CONNECTED)
    {
        LOG_I("unbond timeout disconnect!");
        extern void le_disconnect(uint8_t conn_id);
        le_disconnect(SkaiWatchSys.watch_conn_id);
    }
}
void communicate_parse_init(void)
{
    unbond_disc_timer = rt_timer_create("unbond disc_timer", unbond_timerout_callback, RT_NULL, 2000, RT_TIMER_FLAG_ONE_SHOT);
}

/**
 * @brief   resolve private bond command received from remote APP
 * @param   key: L2 key
 * @param   pValue: received value pointer
 * @param   length: value length
 * @retval  error code
 */
void resolve_private_bond_command(uint8_t key, const uint8_t *pValue, uint16_t length)
{
    switch (key)
    {
    case KEY_BOND_REQUEST:
    {
        bool ret = RT_EOK;
        if (length != USER_ID_LENGTH)
        {
            LOG_E("[KEY_BOND_REQUEST]user id length error(%d != %d)", length, USER_ID_LENGTH);
            ret = -RT_ERROR;
        }
        // else if (SkaiWatchSys.flag_field.bond_state == true)
        // {
        //     LOG_E("[KEY_BOND_REQUEST]device had bonded");
        //     ret = -RT_ERROR;
        // }
        if (ret == RT_EOK)
        {
            LOG_I("[KEY_BOND_REQUEST]Bond Request");
            L1SendData data;
            data.event = L1SEND_BOND_SUCCESS_EVENT;
            L1_send_event(data);
            watch_system_interact(INTERACT_BONDED, (void *)pValue);
        }
        else
        {
            LOG_I("[KEY_BOND_REQUEST]send bond fail event");
            L1SendData data;
            data.event = L1SEND_BOND_FAIL_EVENT;
            L1_send_event(data);
        }
    }
    break;
    case KEY_LOGIN_REQUEST:
    {
        if (length == USER_ID_LENGTH) //  && SkaiWatchSys.flag_field.bond_state == true
        {
            LOG_I("[KEY_LOGIN_REQUEST]login request");
            extern void set_main_phonepeer_addr(void);
            set_main_phonepeer_addr();
            // request_weather_within_six_hours(true);
            /* check_user_id_bonded */
            // if (memcmp(pValue, (void *)SkaiWatchSys.user_data.user_id, length) == 0)
            {
                L1SendData data;
                data.event = L1SEND_LOGIN_SUCCESS_EVENT;
                L1_send_event(data);
                watch_system_interact(INTERACT_LOGIN, NULL);
            }
            // else
            // {
            //     LOG_W("[KEY_LOGIN_REQUEST]login fail");
            //     L1SendData data;
            //     data.event = L1SEND_LOGIN_FAIL_EVENT;
            //     L1_send_event(data);
            // }
        }
        else
        {
            L1SendData data;
            data.event = L1SEND_LOGIN_FAIL_EVENT;
            L1_send_event(data);
        }
    }
    break;
    case KEY_UNBOND:
    {
        watch_system_interact(INTERACT_CANCEL_BOND, NULL);

        if (unbond_disc_timer != RT_NULL)
        {
            rt_timer_start(unbond_disc_timer);
        }
    }
    break;
    default:
        break;
    }
}
