/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse_bond.c
 * @brief    Resolves BOND_COMMAND_ID payloads (bond/login request, unbond)
 *           sent from the phone. The watch acks success/fail and triggers
 *           the corresponding watch_system_interact events.
 *********************************************************************************************************
 */

#include <rtthread.h>
#include "string.h"
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "watch_global_data.h"
#include "bloc_peripheral.h"
#include "bloc_weather.h"
#include "watch_system_interact.h"

extern void le_disconnect(uint8_t conn_id);
extern void set_main_phonepeer_addr(void);

#define DBG_TAG "commu.parse.bond"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/* Disconnect after a 2-second pause when the phone sends KEY_UNBOND. The
   pause lets us finish flushing the unbond ack notification before the link
   drops; if the phone disconnects first the timer fires harmlessly. */
rt_timer_t unbond_disc_timer = RT_NULL;

void unbond_timerout_callback(void *pxTimer)
{
    if (SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_CONNECTED)
    {
        LOG_I("unbond timeout disconnect!");
        le_disconnect(SkaiWatchSys.watch_conn_id);
    }
}

void communicate_parse_init(void)
{
    unbond_disc_timer = rt_timer_create("unbond disc_timer",
                                         unbond_timerout_callback,
                                         RT_NULL, 2000,
                                         RT_TIMER_FLAG_ONE_SHOT);
}

/**
 * @brief   Resolve private bond commands from the phone.
 * @param   key      L2 key (one of BOND_KEY enums)
 * @param   pValue   payload pointer (may be borrowed by interact_bonded)
 * @param   length   payload length
 */
void resolve_private_bond_command(uint8_t key, const uint8_t *pValue, uint16_t length)
{
    switch (key)
    {
    case KEY_BOND_REQUEST:
    {
        if (length != USER_ID_LENGTH)
        {
            LOG_E("[KEY_BOND_REQUEST]user id length error(%d != %d)",
                  length, USER_ID_LENGTH);
            LOG_I("[KEY_BOND_REQUEST]send bond fail event");
            commu_send_bond_fail();
            break;
        }
        LOG_I("[KEY_BOND_REQUEST]Bond Request");
        commu_send_bond_success();
        interact_bonded(pValue);
    }
    break;
    case KEY_LOGIN_REQUEST:
    {
        if (length != USER_ID_LENGTH)
        {
            commu_send_login_fail();
            break;
        }
        LOG_I("[KEY_LOGIN_REQUEST]login request");
        set_main_phonepeer_addr();
        commu_send_login_success();
        SkaiWatchSys.flag_field.device_had_logged = true;
    }
    break;
    case KEY_UNBOND:
    {
        interact_cancel_bond();
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
