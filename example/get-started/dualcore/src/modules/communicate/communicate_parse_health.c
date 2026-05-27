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
#include "watch_sys_service.h"   /* watch_sys_sleep_state_t (SkaiWatchSys.sleep_state) */
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
            LOG_I("request today's exercise data");

            time_t now;
            struct tm *tm_info;
            char file_path[40];

            time(&now);
            tm_info = localtime(&now);
            snprintf(file_path, sizeof(file_path),
                     "/exercise/%04d%02d%02d.json",
                     tm_info->tm_year + 1900, tm_info->tm_mon + 1,
                     tm_info->tm_mday);

            struct stat st;
            if (stat(file_path, &st) == 0)
            {
                int sync_ret =
                    bloc_file_system.sync_file((char *)file_path, false);
                if (sync_ret == 0)
                {
                    LOG_D("sync today's exercise: %s", file_path);
                }
                else
                {
                    LOG_E("failed to sync today's exercise: %s", file_path);
                }
            }
            else
            {
                LOG_D("no exercise data today: %s", file_path);
            }

            /* Store-and-forward today's sleep summary. Overnight the phone is
               usually disconnected, so the live KEY_RETURN_SLEEP_DATA pushes are
               dropped; this hands over the persisted /health/sleep file on
               reconnect (the phone ingests sleep_*.json from the health folder). */
            char sleep_path[40];
            snprintf(sleep_path, sizeof(sleep_path),
                     "/health/sleep_%04d%02d%02d.json",
                     tm_info->tm_year + 1900, tm_info->tm_mon + 1,
                     tm_info->tm_mday);
            if (stat(sleep_path, &st) == 0)
            {
                if (bloc_file_system.sync_file((char *)sleep_path, false) == 0)
                {
                    LOG_D("sync today's sleep: %s", sleep_path);
                }
                else
                {
                    LOG_E("failed to sync today's sleep: %s", sleep_path);
                }
            }
            else
            {
                LOG_D("no sleep data today: %s", sleep_path);
            }
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

void commu_health_save_sleep_file(void)
{
    /* Copy out of the packed SkaiWatchSys first — taking the address of a packed
       member is an unaligned-pointer error under -Werror. */
    const watch_sys_sleep_state_t st = SkaiWatchSys.sleep_state;

    /* Nothing slept yet today -> don't create/overwrite with an all-zero record
       (e.g. a stray daytime wake transition, or just after the midnight reset). */
    if (st.total_sleep_min == 0 && st.awake_after_onset_min == 0)
    {
        return;
    }

    watch_storage_api_lock();

    time_t now;
    struct tm *tm_info;
    char path[40];
    time(&now);
    tm_info = localtime(&now);

    mkdir("/health", 0x777);
    snprintf(path, sizeof(path), "/health/sleep_%04d%02d%02d.json",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday);

    /* Flat JSON; keys MUST match the phone parser (SkaiLink
       watch_foreground_service.dart _processHealthFile: stage / ts_utc /
       total_min / deep_min / rem_min / light_min / waso_min / hr / rhr). */
    char json[224];
    snprintf(json, sizeof(json),
             "{\"stage\":%u,\"ts_utc\":%u,\"total_min\":%u,\"deep_min\":%u,"
             "\"rem_min\":%u,\"light_min\":%u,\"waso_min\":%u,\"hr\":%u,"
             "\"rhr\":%u}",
             (unsigned)st.mode, (unsigned)st.timestamp_utc,
             (unsigned)st.total_sleep_min, (unsigned)st.deep_min,
             (unsigned)st.rem_min, (unsigned)st.light_min,
             (unsigned)st.awake_after_onset_min, (unsigned)st.current_hr,
             (unsigned)st.resting_hr);

    FILE *fp = fopen(path, "w");
    if (fp != NULL)
    {
        fwrite(json, strlen(json), 1, fp);
        fclose(fp);
        LOG_D("saved sleep file %s: %s", path, json);
    }
    else
    {
        LOG_E("failed to open sleep file %s", path);
    }

    watch_storage_api_unlock();
}
