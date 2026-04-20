/**
 ******************************************************************************
 * @file   <activity.c>
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtthread.h>
#include "util/math.h"
#include "util/size.h"
#include "util/units.h"
#include "util/time/time.h"
#include "activity.h"
#include "activity_algorithm.h"
#include "activity_calculators.h"
#include "activity_private.h"
#include "watch_sys_service.h"
#if ENABLE_ACTIVITY_INSIGHTS
#include "activity_insights.h"
#endif

// Our globals
static ActivityState s_activity_state;
// void ACTIVITY_LOG_DEBUG(const char *format, ...)
// {
//   char log_buffer[128];
//   memset(log_buffer, 0, 128);
//   va_list args;
//   va_start(args, format);
//   vsnprintf(log_buffer, 128, format, args);
//   va_end(args);
//   watch_sys_sync.notify_debug_log(log_buffer);
// }
// ------------------------------------------------------------------------------------------------
ActivityState *activity_private_state(void) { return &s_activity_state; }

static void prv_start_tracking_cb(void *context);
static void prv_stop_tracking_cb(void *context);
static void prv_set_enable_cb(void *context);
static void prv_accel_cb(AccelRawData *data, uint32_t num_samples, uint64_t timestamp);

static AccelRawData accel_rawdata[125] = {0};
static uint32_t last_minute_system_task_timestamp = 0;
static void prv_minute_system_task_cb(void *data);

AccelRawData *activity_get_accel_rawdata(void)
{
  return accel_rawdata;
}

// 直接處理加速度數據，不再使用事件通知
void notify_activity_algorithm(void)
{
  if (!s_activity_state.started)
  {
    return;
  }

  // 直接調用加速度處理函數
  uint32_t accel_handling_timestamp = rt_tick_get_millisecond();
  prv_accel_cb(accel_rawdata, 125, accel_handling_timestamp);

  // 每分鐘執行一次系統任務
  if (accel_handling_timestamp - last_minute_system_task_timestamp >= 59000)
  {
    last_minute_system_task_timestamp = accel_handling_timestamp;
    prv_minute_system_task_cb(NULL);
  }
}

// ------------------------------------------------------------------------------------------------
// Tail end of prv_process_minute_data, separated out to decrease stack requirements. This
// portion of the logic handles activity process, saving prefs to our backing store, and
// resetting metrics at midnight.
// We use NOINLINE to reduce the stack requirements during the minute handler (see PBL-38130)
static void NOINLINE prv_process_minute_data_tail(time_t utc_sec)
{
  bool need_history_update_event;
  uint16_t cur_day_index;
  mutex_lock_recursive(s_activity_state.mutex);
  {
    cur_day_index = time_util_get_day(utc_sec);
    need_history_update_event = (cur_day_index != s_activity_state.cur_day_index);

    // Call the activity sessions minute handler
    activity_sessions_prv_minute_handler(utc_sec);

    // Update our backing store if necessary
    // prv_update_storage(utc_sec);

    // If we are starting a new day, reset all metrics
    if (cur_day_index != s_activity_state.cur_day_index)
    {
      s_activity_state.step_data = (ActivityStepData){0};
      s_activity_state.sleep_data = (ActivitySleepData){0};
      memset(&s_activity_state.hr.metrics.minutes_in_zone, 0,
             sizeof(s_activity_state.hr.metrics.minutes_in_zone));
      s_activity_state.steps_per_minute_last_steps = 0;
      s_activity_state.distance_mm = 0;
      s_activity_state.active_calories = 0;
      s_activity_state.resting_calories = 0;
      activity_algorithm_metrics_changed_notification();
      s_activity_state.cur_day_index = cur_day_index;

      // Remove sessions that belong to the prior day
      activity_sessions_prv_remove_out_of_range_activity_sessions(utc_sec,
                                                                  false /*remove_ongoing*/);
#if ENABLE_ACTIVITY_INSIGHTS
      activity_insights_recalculate_stats();
#endif
    }

    // Update the heart rate sampling period if necessary
    // prv_heart_rate_subscription_update(time_get_uptime_seconds());
  }
  mutex_unlock_recursive(s_activity_state.mutex);

  // Send the history update event now if history has changed
  if (need_history_update_event)
  {
    ACTIVITY_LOG_DEBUG("Sending history update event");
    // PebbleEvent e = {
    //     .type = PEBBLE_HEALTH_SERVICE_EVENT,
    //     .health_event =
    //         {
    //             .type = HealthEventSignificantUpdate,
    //             .data.significant_update =
    //                 {
    //                     .day_id = cur_day_index,
    //                 },
    //         },
    // };
    // event_put(&e);
  }
}

// ------------------------------------------------------------------------------------------------
// Takes care of updating the history when we reach midnight as well as checking for changes in
// sleep state. Returns true if the sleep metrics were updated
// We use NOINLINE to reduce the stack requirements during the minute handler (see PBL-38130)
static void NOINLINE prv_process_minute_data(time_t utc_sec)
{
  // Update the metrics
  activity_metrics_prv_minute_handler(utc_sec);

  // Call the algorithm's minute handler. This gives it an opportunity to log minute data
  // to data logging. etc. In case the user settings have changed, pass the current ones in.
  ActivityGender gender = activity_prefs_get_gender();
  uint16_t weight_dag = activity_prefs_get_weight_dag();
  uint16_t height_mm = activity_prefs_get_height_mm();
  uint8_t age_years = activity_prefs_get_age_years();
  activity_algorithm_set_user(height_mm, weight_dag * 10, gender, age_years);

  AlgMinuteRecord minute_record = {};
  activity_algorithm_minute_handler(utc_sec, &minute_record);

  s_activity_state.last_vmc = minute_record.data.base.vmc;
  s_activity_state.last_orientation = minute_record.data.base.orientation;

  // The rest of the minute handling is separated into another method to decrease the stack
  // depth during the call to activity_algorithm_minute_handler() (above)
  prv_process_minute_data_tail(utc_sec);
}

extern time_t get_current_time(void);
// ------------------------------------------------------------------------------------------------
// This system task, triggered by a minute regular timer, takes care of updating the history
// when we reach midnight, checking for changes in sleep state, and updating insights
static void prv_minute_system_task_cb(void *data)
{
  if (!s_activity_state.started)
  {
    return;
  }
  ACTIVITY_LOG_DEBUG("running minute system task");

  // Get the current time
  // time_t utc_sec = rtc_get_time();
  time_t utc_sec = get_current_time();

  // Do our minute processing
  prv_process_minute_data(utc_sec);

#if ENABLE_ACTIVITY_INSIGHTS
  // Process insights
  mutex_lock_recursive(s_activity_state.mutex);
  {
    activity_insights_process_sleep_data(utc_sec);
    activity_insights_process_minute_data(utc_sec);
  }
  mutex_unlock_recursive(s_activity_state.mutex);
#endif
}

static bool prv_activity_allowed_to_be_enabled(void)
{
  return s_activity_state.enabled_run_level; // && s_activity_state.enabled_charging_state;
}

// Accel callback. Called from the KernelBG task.
// Processes incoming accelerometer samples and updates algorithm metrics.
static void prv_accel_cb(AccelRawData *data, uint32_t num_samples, uint64_t timestamp)
{
  // TODO:If the watch is vibrating, clear movement data
  // if (vibes_get_vibe_strength() != VIBE_STRENGTH_OFF)
  // {
  //   memset(data, 0, num_samples * sizeof(AccelRawData));
  // }
  ACTIVITY_LOG_DEBUG("Accel callback data: %d, %d, %d", data[0].x, data[0].y, data[0].z);

  // Process samples with the algorithm
  activity_algorithm_handle_accel(data, num_samples, timestamp);

  // Protect global metrics update with a mutex
  ActivityScalarStore prev_steps = s_activity_state.step_data.steps;
  mutex_lock_recursive(s_activity_state.mutex);
  {
    activity_algorithm_get_steps(&s_activity_state.step_data.steps);

    // Update stepping rate and accumulate distance if available
    uint16_t rate_steps;
    uint32_t rate_elapsed_ms;
    time_t rate_update_time;
    activity_algorithm_get_step_rate(&rate_steps, &rate_elapsed_ms, &rate_update_time);
    if (rate_update_time != s_activity_state.rate_last_update_time)
    {
      s_activity_state.rate_last_update_time = rate_update_time;
      uint32_t distance_mm = activity_private_compute_distance_mm(rate_steps, rate_elapsed_ms);
      s_activity_state.distance_mm += distance_mm;
      s_activity_state.active_calories += activity_private_compute_active_calories(distance_mm, rate_elapsed_ms);
    }
  }
  mutex_unlock_recursive(s_activity_state.mutex);

  // If steps have changed, post an event
  if (s_activity_state.step_data.steps != prev_steps)
  {
    // PebbleEvent e = {
    //     .type = PEBBLE_HEALTH_SERVICE_EVENT,
    //     .health_event = {
    //         .type = HealthEventMovementUpdate,
    //         .data.movement_update = {
    //             .steps = s_activity_state.step_data.steps,
    //         },
    //     },
    // };
    // event_put(&e);
    // WATCH_LCPU_LOG_DEBUG("health_info steps: %d", s_activity_state.step_data.steps);
    watch_sys_sync.notify_health_info();
  }
}

// ------------------------------------------------------------------------------------------------
// NOTE: Caller must have lock
static void prv_stop_tracking_early(void)
{
  // Don't do anything if we are already deinited.
  if (!s_activity_state.started)
  {
    return;
  }

  // Demands the underlying activity algorithms to clean up their act(ivity sessions) and save off
  // any new data they have (only to RAM...persisted to flash a few lines later).
  activity_algorithm_early_deinit();

  // Update storage before we close down
  s_activity_state.update_settings_counter = -1;
  // prv_update_storage(rtc_get_time());

  ACTIVITY_LOG_DEBUG("Updated and persisted sessions before stopping activity tracking");
}

// --- user profile: gender, weight, height, age
static uint8_t _my_gender = ActivityGenderMale;
ActivityGender activity_prefs_get_gender(void)
{
  return _my_gender;
}
void activity_prefs_set_gender(ActivityGender gender)
{
  if (_my_gender != gender)
  {
    _my_gender = gender;
    ACTIVITY_LOG_DEBUG("Gender set to %d", gender);
  }
}
// Decagram may refer to: 10 gram, or 0.01 kilogram
// ex. 70 kg = 7000 dag
uint16_t _my_weight_dag = 7000;
uint16_t activity_prefs_get_weight_dag(void)
{
  return _my_weight_dag;
}
void activity_prefs_set_weight_dag(uint16_t weight_dag)
{
  if (_my_weight_dag != weight_dag)
  {
    _my_weight_dag = weight_dag;
    ACTIVITY_LOG_DEBUG("Weight set to %d dag", weight_dag);
  }
}

uint16_t _my_height_mm = 1770;
uint16_t activity_prefs_get_height_mm(void)
{
  return _my_height_mm;
}
void activity_prefs_set_height_mm(uint16_t height_mm)
{
  if (_my_height_mm != height_mm)
  {
    _my_height_mm = height_mm;
    ACTIVITY_LOG_DEBUG("Height set to %d mm", height_mm);
  }
}

uint8_t _my_age_years = 27;
uint8_t activity_prefs_get_age_years(void)
{
  return _my_age_years;
}
void activity_prefs_set_age_years(uint8_t age_years)
{
  if (_my_age_years != age_years)
  {
    _my_age_years = age_years;
    ACTIVITY_LOG_DEBUG("Age set to %d years", age_years);
  }
}

// ------------------------------------------------------------------------------------------------
// Start activity tracking system callback
static void prv_start_tracking_cb(void *context)
{
  // PBL_ASSERT_TASK(PebbleTask_KernelBackground);
  // bool test_mode = (bool)context;
  // activity_prefs_set_activated();
  s_activity_state.should_be_started = true;
  if (!prv_activity_allowed_to_be_enabled() || s_activity_state.started)
  {
    return;
  }

  AccelSamplingRate sampling_rate;
  if (activity_algorithm_init(&sampling_rate))
  {
    ACTIVITY_LOG_DEBUG("Starting activity tracking...");

    // Set the user data
    ActivityGender gender = activity_prefs_get_gender();
    uint16_t weight_dag = activity_prefs_get_weight_dag();
    uint16_t height_mm = activity_prefs_get_height_mm();
    uint8_t age_years = activity_prefs_get_age_years();
    activity_algorithm_set_user(height_mm, weight_dag * 10, gender, age_years);
    activity_algorithm_metrics_changed_notification();

    // Register our minutes callback
    // cron_job_schedule(&s_activity_job);
    s_activity_state.started = true;
    LOG_I("Activity tracking started");

    // PebbleEvent event = {
    //     .type = PEBBLE_ACTIVITY_EVENT,
    //     .activity_event =
    //         {
    //             .type = PebbleActivityEvent_TrackingStarted,
    //         },
    // };
    // event_put(&event);
  }
}

// ------------------------------------------------------------------------------------------------
// Stop activity tracking system callback
static void prv_stop_tracking_cb(void *context)
{
  s_activity_state.should_be_started = false;
  if (!s_activity_state.started)
  {
    return;
  }

  RT_ASSERT(activity_algorithm_deinit());
  s_activity_state.started = false;
  ACTIVITY_LOG_DEBUG("activity tracking stopped");

  // PebbleEvent event = {
  //     .type = PEBBLE_ACTIVITY_EVENT,
  //     .activity_event =
  //         {
  //             .type = PebbleActivityEvent_TrackingStopped,
  //         },
  // };
  // event_put(&event);
}

// Enable/disable activity service callback.
// This function is used to switch the algorithm on or off based on the current state.
static void prv_set_enable_cb(void *context)
{
  mutex_lock_recursive(s_activity_state.mutex);
  {
    const bool enable = prv_activity_allowed_to_be_enabled();

    if (enable == s_activity_state.started)
    {
      // No change in enabled state, do nothing.
      goto cleanup;
    }

    if (enable)
    {
      // Algorithm should be enabled. Start tracking if it is marked to run.
      if (s_activity_state.should_be_started)
      {
        prv_start_tracking_cb(NULL);
      }
    }
    else
    {
      // Algorithm should be disabled. Stop tracking if active,
      // and mark it to be restarted when re-enabled.
      if (s_activity_state.started)
      {
        prv_stop_tracking_cb(NULL);
        s_activity_state.should_be_started = true;
      }
    }
  }
cleanup:
  mutex_unlock_recursive(s_activity_state.mutex);
}

static void prv_handle_activity_enabled_change(void)
{
  if (activity_tracking_on() && !prv_activity_allowed_to_be_enabled())
  {
    prv_stop_tracking_early();
  }
  // 直接調用，不使用事件
  prv_set_enable_cb(NULL);
}

// ------------------------------------------------------------------------------------------------
bool activity_init(void)
{
  s_activity_state = (ActivityState){};
  s_activity_state.mutex = rt_mutex_create("s_activity_state", RT_IPC_FLAG_FIFO);
  if (s_activity_state.mutex == RT_NULL)
  {
    ACTIVITY_LOG_DEBUG("Failed to create mutex");
    return false;
  }

  // Init the current day index
  // time_t utc_now = rtc_get_time();
  time_t utc_now = get_current_time();
  s_activity_state.cur_day_index = time_util_get_day(utc_now);

  // Init variables used to compute the derived metrics
  s_activity_state.steps_per_minute_last_steps = s_activity_state.step_data.steps;
  s_activity_state.distance_mm = s_activity_state.step_data.distance_meters * MM_PER_METER;
  s_activity_state.active_calories =
      s_activity_state.step_data.active_kcalories * ACTIVITY_CALORIES_PER_KCAL;

  return true;
}

// ------------------------------------------------------------------------------------------------
bool activity_start_tracking(bool test_mode)
{
  prv_start_tracking_cb(NULL);
  return true;
}

// ------------------------------------------------------------------------------------------------
bool activity_stop_tracking(void)
{
  mutex_lock_recursive(s_activity_state.mutex);
  {
    prv_stop_tracking_early();
  }
  mutex_unlock_recursive(s_activity_state.mutex);
  prv_stop_tracking_cb(NULL);
  return true;
}

// ------------------------------------------------------------------------------------------------
bool activity_tracking_on(void)
{
  bool result;
  mutex_lock_recursive(s_activity_state.mutex);
  result = s_activity_state.started;
  mutex_unlock_recursive(s_activity_state.mutex);
  return result;
}

// ------------------------------------------------------------------------------------------------
// Enable/disable this service. Used by the service manager's services_set_runlevel() call.
// Note that this can be called from a timer callback so we do all the heavy lifting from a
// kernel BG callback
void activity_set_enabled(bool enable)
{
  mutex_lock_recursive(s_activity_state.mutex);
  {
    s_activity_state.enabled_run_level = enable;
  }
  mutex_unlock_recursive(s_activity_state.mutex);
  prv_handle_activity_enabled_change();
}

void send_list_to_hcpu(time_t utc_now, uint8_t steps, uint8_t orientation, uint16_t vmc)
{
  // send list of: {steps, orientation, vmc} to hcpu
  ACTIVITY_LOG_DEBUG("send list of: {steps: %d, orientation: %d, vmc: %d} to hcpu", steps, orientation, vmc);
  if (watch_sys_sync.notify_minute_of_activity)
  {
    watch_sys_sync.notify_minute_of_activity(utc_now, steps, orientation, vmc);
  }
}

int activity_main(void)
{
  activity_init();
  activity_set_enabled(true);
  prv_start_tracking_cb(NULL);
  return 0;
}

// INIT_APP_EXPORT(activity_main);
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/