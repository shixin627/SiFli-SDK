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
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif
#include "bloc_peripheral.h"

/* Private variables */
static uint32_t last_step_count = 0;
static uint32_t last_activity_time = 0;

/* RT-Thread thread handle and stack definition */
#define PERIODIC_TASK_STACK_SIZE 1024
#define PERIODIC_TASK_PRIORITY 27                   // Adjust priority as needed
#define PERIODIC_TASK_TICK (300 * RT_TICK_PER_SECOND) // Run every 5 minutes

static rt_thread_t periodic_task_thread = RT_NULL;
static uint8_t periodic_task_stack[PERIODIC_TASK_STACK_SIZE];

/**
 * @brief Check if current time is within the configured sit alert monitoring
 * hours
 * @return true if current time is within monitoring hours, false otherwise
 */
static bool is_within_monitoring_hours(void)
{
    uint8_t current_hour = SkaiWatchSys.Global_Time.hour;
    uint8_t start_hour = SkaiWatchSys.sit_alert_data.sit_alert.start_hour;
    uint8_t end_hour = SkaiWatchSys.sit_alert_data.sit_alert.end_hour;

    // Handle case where monitoring period spans midnight
    if (start_hour > end_hour)
    {
        return (current_hour >= start_hour || current_hour < end_hour);
    }
    else
    {
        return (current_hour >= start_hour && current_hour < end_hour);
    }
}

// user wearing status
static bool is_user_wearing(void)
{
    return SkaiWatchSys.flag_field.is_wearing != 0;
}

/**
 * @brief Check if today is a monitoring day based on day_flag_bits
 * @return true if today is a monitoring day, false otherwise
 */
static bool is_monitoring_day(void)
{
    uint8_t day_mask = 1 << SkaiWatchSys.Global_Time.weekday;
    return (SkaiWatchSys.sit_alert_data.sit_alert.day_flag_bits & day_mask) !=
           0;
}

/**
 * @brief Check if user has been active since last check
 * @return true if user has been active, false otherwise
 */
static bool check_activity(void)
{
    uint32_t current_steps = SkaiWatchSys.gPedoData.global_steps;
    uint16_t step_threshold =
        SkaiWatchSys.sit_alert_data.sit_alert.step_low_limit;

    // Check if steps have increased beyond threshold since last check
    uint32_t step_diff = current_steps - last_step_count;

    if (step_diff >= step_threshold)
    {
        last_step_count = current_steps;
        last_activity_time = get_current_time();
        return true;
    }

    return false;
}

/**
 * @brief Process the sit alert monitoring
 * @return SIT_ALERT_NONE if no alert needed, SIT_ALERT_TRIGGERED if alert
 * should be shown
 */
sit_alert_status_t sit_alert_process(void)
{
    // Check if feature is enabled
    if (!SkaiWatchSys.sit_alert_data.sit_alert.on_off)
    {
        return SIT_ALERT_NONE;
    }

    // Check if we should monitor today
    if (!is_monitoring_day())
    {
        return SIT_ALERT_NONE;
    }

    // Check if we're within monitoring hours
    if (!is_within_monitoring_hours())
    {
        return SIT_ALERT_NONE;
    }

    // Check if user is wearing the watch
    if (!is_user_wearing())
    {
        last_activity_time = get_current_time(); // Reset timer if not wearing
        return SIT_ALERT_NONE;
    }

    // Check if user has been active since last check
    if (check_activity())
    {
        return SIT_ALERT_NONE; // User has been active
    }

    // Calculate inactive duration in minutes
    uint32_t current_time = get_current_time();
    uint32_t inactive_duration =
        (current_time - last_activity_time) / 60; // Convert to minutes

    // Check if inactive duration exceeds threshold
    if (inactive_duration >= SkaiWatchSys.sit_alert_data.sit_alert.sit_min)
    {
        // Reset the timer after triggering alert
        last_activity_time = current_time;

        // Don't show alert if inactive for more than 12 hours (user likely
        // sleeping or not wearing)
        if (inactive_duration > 720) // 720 minutes = 12 hours
        {
            return SIT_ALERT_NONE;
        }
        return SIT_ALERT_TRIGGERED;
    }

    return SIT_ALERT_NONE;
}

/**
 * @brief Reset sit alert tracking after user acknowledges the alert
 */
void sit_alert_acknowledge(void)
{
    last_activity_time = get_current_time();
    last_step_count = SkaiWatchSys.gPedoData.global_steps;
}

/**
 * @brief Display the sit alert UI to the user
 */
void ui_display_sit_alert(void)
{
    watch_system_interact(INTERACT_LONG_SIT_ALERT, NULL);
    rt_thread_mdelay(1000);
    on_sit_alert_dismissed();
}

/**
 * @brief Initialize application data
 */
void app_init(void)
{
    // Initialize sit alert feature
    last_activity_time = get_current_time();
    last_step_count = SkaiWatchSys.gPedoData.global_steps;
}

/**
 * @brief Periodic task that runs system checks
 */
void app_periodic_task(void)
{
    // sit_alert_status_t status = sit_alert_process();

    // if (status == SIT_ALERT_TRIGGERED)
    // {
    // 	// Show alert to the user
    // 	ui_display_sit_alert();
    // }

    // extern void app_exercise_background_hr_cb(int hr);
    // app_exercise_background_hr_cb(1800);
}

/**
 * @brief Handler for sit alert dismissal
 */
void on_sit_alert_dismissed(void)
{
    // Called when user acknowledges the sit alert
    sit_alert_acknowledge();
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

    // Initialize sit alert feature
    app_init();

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
