/**
 ******************************************************************************
 * @file   bloc_system_perception.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include "watch_global_data.h"
#include "bloc_system_perception.h"
#include "bloc_peripheral.h"
#ifdef BSP_USING_BLOC
    #include "bloc_v2t.h"
    #include "bloc_filesystem.h"
    #include "bloc_health.h"
#endif
#ifdef BSP_USING_WATCH_SYS_CLIENT
    #include "watch_sys_service.h"
#endif

#define DBG_TAG "sys.perception"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/* Private variables */
static bool last_device_logged = false;

/* RT-Thread thread handle and stack definition */
/* 4 KB, not 1 KB: app_periodic_task() reaches bloc_file_system.sync_folder_files()
 * which walks /health and /recorder, reads each file, serialises JSON and pushes
 * it over BLE — the same deep path the file_sys thread runs at 4 KB (~45% used).
 * At 1 KB this thread overflowed (RT-Thread "thread:periodic stack overflow",
 * dump showed periodic at 100%) whenever the per-minute tick coincided with a
 * sleep-stage write + reconnect health flush. Keep it at least as large as
 * file_sys/health_s. */
#define PERIODIC_TASK_STACK_SIZE 4096
#define PERIODIC_TASK_PRIORITY 27 // Adjust priority as needed
#define PERIODIC_TASK_TICK (60 * RT_TICK_PER_SECOND) // Run every 1 minute

static rt_thread_t periodic_task_thread = RT_NULL;

#ifdef BSP_USING_BLOC
/**
 * @brief Check /recorder for pending files and sync them to phone
 *        Called when device connects to phone after being disconnected
 */
static void check_and_sync_pending_recordings(void)
{
    if (!SkaiWatchSys.flag_field.device_had_logged)
    {
        return;
    }

    // Don't sync while recording is in progress
    if (app_voice_get_recording_status())
    {
        LOG_D("Recording in progress, skip pending sync");
        return;
    }

    // Don't sync if another file is already being synced
    sync_progress_t *progress = get_sync_progress();
    if (progress->sync_status)
    {
        LOG_D("Sync already in progress, skip pending sync");
        return;
    }

    LOG_D("Checking /recorder for pending files to sync");
    bloc_file_system.sync_folder_files("/recorder", true);
}
#endif /* BSP_USING_BLOC */

/**
 * @brief Periodic task that runs system checks
 */
void app_periodic_task(void)
{
#ifndef BSP_USING_PC_SIMULATOR
    time_t now = get_current_time();
    struct tm *time_info;
    time_info = localtime(&now);
    SkaiWatchSys.Global_Time.year = time_info->tm_year + 1900;
    SkaiWatchSys.Global_Time.month = time_info->tm_mon + 1;
    SkaiWatchSys.Global_Time.day = time_info->tm_mday;
    SkaiWatchSys.Global_Time.hour = time_info->tm_hour;
    SkaiWatchSys.Global_Time.minutes = time_info->tm_min;
    SkaiWatchSys.Global_Time.seconds = time_info->tm_sec;
    SkaiWatchSys.Global_Time.weekday = time_info->tm_wday;
    LOG_D("Running periodic system perception task - Current time: %d-%d-%d %d:%d:%d",
          SkaiWatchSys.Global_Time.year, SkaiWatchSys.Global_Time.month,
          SkaiWatchSys.Global_Time.day, SkaiWatchSys.Global_Time.hour,
          SkaiWatchSys.Global_Time.minutes, SkaiWatchSys.Global_Time.seconds);
    // peripheral_provider.save_watch_shared_prefs(WATCH_PREFS_KEY_GLOBAL_TIME); // 不需要手動儲存時間到shared prefs,因為只要系統不斷電,rtc模組會自動保存時間
#endif

#ifdef BSP_USING_BLOC
    /* Check pending /recorder sync whenever phone is logged in. The first
       transition (false -> true) gets an extra log so the boundary is visible
       in traces. */
    bool current_logged = SkaiWatchSys.flag_field.device_had_logged;
    if (current_logged)
    {
        if (!last_device_logged)
        {
            LOG_D("Phone connection detected, checking for pending recorder files");
        }
        check_and_sync_pending_recordings();
    }
    last_device_logged = current_logged;

    /* Flush persisted health files once per reconnect (rising edge of the live
       BLE link). delete_after_sync=false so a still-unsynced day survives until
       it ages out — no loss even on a flaky link; the phone upserts each day
       idempotently. The live push handles updates while already connected, so
       we deliberately do NOT re-flush every tick (that would re-send every
       retained day every minute). */
    {
        static bool s_last_connected = false;
        bool connected = SkaiWatchSys.connected_to_phone;
        if (connected && !s_last_connected)
        {
            sync_progress_t *progress = get_sync_progress();
            if (!progress->sync_status)
            {
                LOG_D("Reconnect: flushing pending /health files");
                bloc_file_system.sync_folder_files("/health", false);
            }
            health_cleanup_old_files();
        }
        s_last_connected = connected;
    }
#endif

#ifdef BSP_USING_WATCH_SYS_CLIENT
    watch_sys_sync.request_battery_voltage();
#endif
}

/**
 * @brief Thread entry function for periodic tasks
 * @param parameter Thread parameter (unused)
 */
static void periodic_task_entry(void *parameter)
{
    while (1)
    {
        // Run the periodic tasks
        app_periodic_task();

        // Sleep for the specified duration
        rt_thread_mdelay(PERIODIC_TASK_TICK);
    }
}

/**
 * @brief Initialize the system schedule and start the periodic task thread
 * @return RT_EOK if successful, otherwise error code
 */
rt_err_t bloc_system_schedule_init(void)
{
    rt_err_t result;

    // Create the periodic task thread
    periodic_task_thread =
        rt_thread_create("periodic_task", periodic_task_entry, RT_NULL,
                         PERIODIC_TASK_STACK_SIZE, PERIODIC_TASK_PRIORITY, 10);
    if (periodic_task_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    // Start the thread
    result = rt_thread_startup(periodic_task_thread);
    if (result != RT_EOK)
    {
        return result;
    }

    return RT_EOK;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
