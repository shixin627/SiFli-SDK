/*********************************************************************************************************
 *               Copyright(c) 2024, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse_control.c
 * @brief
 * @details
 * @author
 * @date
 * @version  v0.1
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
#include "watch_system_core_task.h"
#endif

#define DBG_TAG "commu.parse.control"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
/**
 * @brief   resolve Control data command received from remote APP
 * @param   key: L2 key
 * @param   pValue: received value pointer
 * @param   length: value length
 * @retval  error code
 */
void resolve_Control_command(uint8_t key, const uint8_t *pValue, uint16_t length)
{
    switch (key)
    {
    case KEY_PHONE_CAMERA_STATUS:
    {
        if (length == 1)
        {
            uint8_t status = pValue[0];
            watch_system_interact(INTERACT_CAMERA, &status);
        }
        break;
    }

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
            // test motor command from app
            uint8_t value = pValue[0];
            uint8_t duty_cycle = pValue[1];
            uint32_t period_us = ((pValue[2] << 8) + pValue[3]) * 1000;
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
            control_provider.bt_speaker_set_volume(percent, false);
        }
        break;
    }

    case KEY_AUDIO_RECORD:
    {
        if (length == 1)
        {
            bool status = pValue[0] ? true : false;
            peripheral_provider.audio_recording(status);
        }
        break;
    }

    case KEY_AUDIO_PLAY:
    {
        if (length == 1)
        {
            bool status = pValue[0] ? true : false;
            peripheral_provider.audio_playback(status);
        }
        break;
    }

    case KEY_REBOOT:
        watch_system_interact(WATCH_REBOOT, NULL);
        break;

    case KEY_SHUTDOWN:
        send_sys_interact_event(SYS_EVENT_POWER_OFF);
        break;

    case KEY_SLEEP:
        watch_system_interact(WATCH_SLEEP, NULL);
        break;

    case KEY_WAKEUP:
        watch_hcpu_resume_with_reason(WAKEUP_REASON_OTHER);
        break;

    case KEY_MIC_LISTEN:
    {
        if (length == 1)
        {
            bool status = pValue[0] ? true : false;
            watch_system_interact(INTERACT_MIC_LISTEN, &status);
        }
    }
    break;
    case KEY_APP_RUN:
    {
        extern void parse_open_app_command(const uint8_t *pValue, uint16_t length);
        parse_open_app_command(pValue, length);
    }
    break;
    case KEY_COUNTROL_KEYBOARD:
    {
        if (length == 1)
        {
            bool status = pValue[0] ? true : false;
            if (is_at_mouse_mode() && status)
            {
                lvgl_msg_t msg;
                msg.type = LVGL_MSG_TYPE_MOUSE_OPEN_KEYBOARD;
                lvgl_send_msg(msg);
            }
            break;
        }
    }
    default:
        break;
    }
}
