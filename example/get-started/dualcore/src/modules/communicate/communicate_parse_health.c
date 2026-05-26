/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse_health.c
 * @brief    Resolves HEALTH_DATA_COMMAND_ID payloads from the phone (request
 *           today's exercise, daily/quarter pedometer sync).
 *********************************************************************************************************
 */
#include <rtthread.h>
#include <stdio.h>
#include <time.h>
#include <dfs_posix.h>
#include <sys/stat.h>
#include "board.h"
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "watch_global_data.h"
#include "bloc_exercise.h"
#include "bloc_filesystem.h"
#include "string.h"

#define DBG_TAG "commu.parse.health"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/**
 * @brief   resolve health data command received from remote APP
 * @param   key: L2 key
 * @param   pValue: received value pointer
 * @param   length: value length
 * @retval  error code
 */
void resolve_HealthData_command(uint8_t key, const uint8_t *pValue,
                                uint16_t length)
{
    switch (key)
    {
    case KEY_REQUEST_DATA:
    {
        if (length == 0)
        {
            /* Sync ALL un-synced exercise days (not just today) so a multi-day
               offline period is not missed. delete_after_sync=false: the phone
               dedupes per day, and today's file was already re-sent on every
               request before this change, so re-receiving a day is idempotent
               on the phone. */
            LOG_I("request exercise data: sync all /exercise files");
            bloc_file_system.sync_folder_files("/exercise", false);
        }
    }
    break;
    case KEY_DAILY_DATA_SYNC:
    {
        uint32_t daily_step = read_be32(&pValue[0]);
        uint32_t daily_distance = read_be32(&pValue[4]);
        uint32_t daily_calory = read_be32(&pValue[8]);

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
        uint32_t calories = read_be32(&pValue[2]);
        uint16_t steps = read_be16(&pValue[6]);
        uint16_t distance = read_be16(&pValue[8]);

        SkaiWatchSys.gPedoData.quarter_steps = (uint16_t)steps;
        SkaiWatchSys.gPedoData.quarter_distance = (uint32_t)distance * 1600;
        SkaiWatchSys.gPedoData.quarter_calories = (uint32_t)calories * 4;
        LOG_I("latest data sync, steps:%d, distance:%d, calories:%d",
              SkaiWatchSys.gPedoData.quarter_steps,
              SkaiWatchSys.gPedoData.quarter_distance,
              SkaiWatchSys.gPedoData.quarter_calories);
    }
    break;
    default:
        break;
    }
}
