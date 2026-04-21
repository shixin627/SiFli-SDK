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
#include "board.h"
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "watch_global_data.h"
#include "string.h"

#define DBG_TAG "commu.parse.health"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/**
 * @brief   resolve health data command received from remote APP
 * @param   key: L2 key
 * @param   pValue: received value pointer
 * @param   length: value length
 * @retval  error code
 */
void resolve_HealthData_command(uint8_t key, const uint8_t *pValue, uint16_t length)
{
    switch (key)
    {
    case KEY_REQUEST_DATA:
    {
        if (length == 0)
        {
            LOG_I("request today's sport data");
            L1SendData data;
            data.event = L1SEND_SPORT_DATA;
            L1_send_event(data);
        }
    }
    break;
    case KEY_SET_STEPS_NOTIFY:
    {
        if (length == 1)
        {
            LOG_I("Steps Autosync value:%d\n", pValue[0]);
            if (pValue[0])
            {
                SkaiWatchSys.flag_field.auto_sync_enable = true;
            }
            else
            {
                SkaiWatchSys.flag_field.auto_sync_enable = false;
            }
        }
    }
    break;

    case KEY_DAILY_DATA_SYNC:
    {
        uint32_t daily_step = 0;
        uint32_t daily_distance = 0;
        uint32_t daily_calory = 0;

        daily_step |= pValue[3];
        daily_step |= pValue[2] << 8;
        daily_step |= pValue[1] << 16;
        daily_step |= pValue[0] << 24;

        daily_distance |= pValue[3 + 4];
        daily_distance |= pValue[2 + 4] << 8;
        daily_distance |= pValue[1 + 4] << 16;
        daily_distance |= pValue[0 + 4] << 24;

        daily_calory |= pValue[3 + 8];
        daily_calory |= pValue[2 + 8] << 8;
        daily_calory |= pValue[1 + 8] << 16;
        daily_calory |= pValue[0 + 8] << 24;

        if (daily_step != SkaiWatchSys.gPedoData.global_steps)
        {
            SkaiWatchSys.gPedoData.global_steps = daily_step;
            SkaiWatchSys.gPedoData.global_calories = daily_calory * 4;
            SkaiWatchSys.gPedoData.global_distance = daily_distance * 1600;
            LOG_I("daily data sync, steps:%d, distance:%d, calories:%d",
                  SkaiWatchSys.gPedoData.global_steps,
                  SkaiWatchSys.gPedoData.global_distance,
                  SkaiWatchSys.gPedoData.global_calories);
        }
    }
    break;

    case KEY_LATEST_DATA_SYNC:
    {
        uint32_t calories = 0;
        uint16_t steps = 0;
        uint16_t distance = 0;
        calories |= pValue[2 + 0] << 24;
        calories |= pValue[2 + 1] << 16;
        calories |= pValue[2 + 2] << 8;
        calories |= pValue[2 + 3];

        steps |= pValue[6 + 0] << 8;
        steps |= pValue[6 + 1];

        distance |= pValue[8 + 0] << 8;
        distance |= pValue[8 + 1];

        SkaiWatchSys.gPedoData.quarter_steps = (uint16_t)steps;
        SkaiWatchSys.gPedoData.quarter_distance = (uint32_t)distance * 1600;
        SkaiWatchSys.gPedoData.quarter_calories = (uint32_t)calories * 4;
        LOG_I("latest data sync, steps:%d, distance:%d, calories:%d",
              SkaiWatchSys.gPedoData.quarter_steps,
              SkaiWatchSys.gPedoData.quarter_distance,
              SkaiWatchSys.gPedoData.quarter_calories);
    }
    break;
#if 0
    case KEY_REQUEST_HEART_DATA:
        {
            //start a single sample
            if (length == 1)
            {
                if (pValue[0] == 0x01)
                {
                    SkaiWatchSys.hrs_start_up_mode = HRS_MOBILE_START;
                    SkaiWatchSys.heart_sample_by_phone = true;
                    hal_hrs_timer_start();
                }
                else if (pValue[0] == 0x00)
                {
                    hal_hrs_timer_stop();
                }
            }
        }
        break;
    case KEY_HEART_DATA_SAMPLE_SETTING:
        {
            if (length == 2)
            {
                if (pValue[0] == 0x01)
                {
                    //to do get sample period
                    if (pValue[1] > 0 && pValue[1] <= 10)
                    {
                        hal_hrs_sample_continuously_start(pValue[1] * 60 * 1000);
                        SkaiWatchSys.hrs_detect_period = pValue[1];
                    }
                    else
                    {
                        hal_hrs_sample_continuously_start(300000);//5min
                        SkaiWatchSys.hrs_detect_period = 5;
                    }
                }
                else if (pValue[0] == 0x00)
                {
                    hal_hrs_sample_continuously_stop();
                    hal_hrs_timer_stop();
                    SkaiWatchSys.hrs_detect_period = 0;
                }
                uint32_t temp = SkaiWatchSys.hrs_detect_period;
                ftl_save(&temp, HRS_PERIOD_OFFSET, HRS_PERIOD_SIZE);
            }
        }
        break;
#endif
    case KEY_REQUEST_HEART_SAMPLE_SETTING:
    {
        if (length == 0)
        {
            LOG_I("request heart sample setting");
            L1SendData data;
            data.event = L1SEND_RETURN_HEART_SETTING;
            L1_send_event(data);
        }
    }
    break;
    default:
        break;
    }
}
