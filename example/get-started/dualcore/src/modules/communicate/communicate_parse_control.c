/*********************************************************************************************************
 *               Copyright(c) 2024, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse_control.c
 * @brief    Resolves CONTROL_COMMAND_ID payloads (find watch, media/volume,
 *           reboot/sleep/wake, app launch) sent from the phone.
 *********************************************************************************************************
 */
#include <rtthread.h>
#include "communicate_parse.h"
#include "communicate_parse_control.h"
#include "watch_global_data.h"
#include "app_mainmenu.h"
#include "ui_handler.h"
#include "bloc_control.h"
#include "bloc_peripheral.h"
#include "bloc_v2t.h"
#include "string.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif

#define DBG_TAG "commu.parse.control"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>
/**
 * @brief   resolve Control data command received from remote APP
 * @param   key: L2 key
 * @param   pValue: received value pointer
 * @param   length: value length
 * @retval  error code
 */
void resolve_Control_command(uint8_t key, const uint8_t *pValue,
                             uint16_t length)
{
    switch (key)
    {
    case KEY_FIND_WATCH:
    {
        if (length == 1)
        {
            uint8_t value = pValue[0];
            if (value == 0x01)
            {
                watch_system_interact(INTERACT_FIND_WATCH, NULL);
            }
            else
            {
                peripheral_provider.control_motor(false, NULL);
            }
        }
        else if (length == 5)
        {
            /* Test motor command: [enable:1][duty:1][period_ms BE:2][repeat:1] */
            uint8_t value      = pValue[0];
            uint8_t duty_cycle = pValue[1];
            uint32_t period_us = (uint32_t)read_be16(&pValue[2]) * 1000;
            uint8_t repeat_times = pValue[4];
            if (value == 0x01)
            {
                if (get_motor_switch_state())
                {
                    motor_params_t param = {
                        .duty_cycle = duty_cycle,
                        .period = period_us,
                        .repeat_times = repeat_times,
                    };
                    peripheral_provider.control_motor(true, &param);
                }
            }
            else
            {
                peripheral_provider.control_motor(false, NULL);
            }
        }
        break;
    }

    case KEY_PHONE_MEDIA_STATUS:
    {
        if (length == 1)
        {
            uint8_t value = pValue[0];
            watch_system_interact(INTERACT_SYNC_MEDIA_STATUS, &value);
        }
        break;
    }

    case KEY_PHONE_VOLUMN:
    {
        if (length == 1)
        {
            uint8_t percent = pValue[0];
            control_provider.bt_phone_set_volume(percent);
            LOG_D("Received volume percentage from phone: %d", percent);
        }
        break;
    }

    case KEY_REBOOT:
        watch_system_interact(WATCH_REBOOT, NULL);
        break;

    case KEY_SLEEP:
        watch_system_interact(WATCH_SLEEP, NULL);
        break;

    case KEY_WAKEUP:
        watch_system_interact(HCPU_WAKEUP, NULL);
        break;

    case KEY_APP_RUN:
    {
        extern void parse_open_app_command(const uint8_t *pValue,
                                           uint16_t length);
        parse_open_app_command(pValue, length);
    }
    break;
    default:
        break;
    }
}
