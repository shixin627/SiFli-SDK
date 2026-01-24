/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "activity.h"
#include "activity_algorithm.h"
#include "activity_calculators.h"
#include "activity_insights.h"
#include "activity_private.h"

// --------------------------------------------------------------------------------------------
ActivityScalarStore activity_metrics_prv_steps_per_minute(void)
{
  ActivityState *state = activity_private_state();
  return state->steps_per_minute;
}

// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_distance_mm(void)
{
  ActivityState *state = activity_private_state();
  return state->distance_mm;
}

// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_resting_calories(void)
{
  ActivityState *state = activity_private_state();
  return state->resting_calories;
}

// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_active_calories(void)
{
  ActivityState *state = activity_private_state();
  return state->active_calories;
}

// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_steps(void)
{
  ActivityState *state = activity_private_state();
  return state->step_data.steps;
}

void activity_metrics_prv_reset_hr_stats(void) {
  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);
  {
    state->hr.num_samples = 0;
    state->hr.num_quality_samples = 0;
    memset(state->hr.samples, 0, sizeof(state->hr.samples));
    memset(state->hr.weights, 0, sizeof(state->hr.weights));

    state->hr.metrics.previous_median_bpm = 0;
    state->hr.metrics.previous_median_total_weight_x100 = 0;
  }
  mutex_unlock_recursive(state->mutex);
}


// ----------------------------------------------------------------------------------------------
// Shift the history back one day and reset the current day's stats.
// We use NOINLINE to reduce the stack requirements during the minute handler (see PBL-38130)
static void NOINLINE prv_shift_history(time_t utc_now) {
  ActivityState *state = activity_private_state();
  LOG_I("resetting metrics for new day");
//   mutex_lock_recursive(state->mutex);
//   {
//     SettingsFile *file = activity_private_settings_open();
//     if (!file) {
//       goto unlock;
//     }
//     ActivitySettingsValueHistory history;
//     ActivityMetricInfo m_info;

//     for (ActivityMetric metric = ActivityMetricFirst; metric < ActivityMetricNumMetrics;
//          metric++) {
//       activity_metrics_prv_get_metric_info(metric, &m_info);

//       // Shift the history
//       if (m_info.has_history) {
//         PBL_ASSERTN(m_info.value_p);
//         settings_file_get(file, &m_info.settings_key, sizeof(m_info.settings_key), &history,
//                           sizeof(history));

//         for (int i = ACTIVITY_HISTORY_DAYS - 1; i >= 1; i--) {
//           history.values[i] = history.values[i - 1];
//         }
//         // We just wrapped up yesterday
//         history.values[1] = *m_info.value_p;

//         // Reset stats for today
//         history.values[0] = 0;
//         history.utc_sec = utc_now;

//         settings_file_set(file, &m_info.settings_key, sizeof(m_info.settings_key), &history,
//                           sizeof(history));
//       }
//     }
//     activity_private_settings_close(file);
//   }
// unlock:
//   mutex_unlock_recursive(state->mutex);
}

// ------------------------------------------------------------------------------------------
// The metrics minute handler
void activity_metrics_prv_minute_handler(time_t utc_sec) {
  ActivityState *state = activity_private_state();

  uint16_t cur_day_index = time_util_get_day(utc_sec);
  if (cur_day_index != state->cur_day_index) {
    // If we've just encountered a midnight rollover, shift history to the new day
    // before we compute metrics for the new day
    prv_shift_history(utc_sec);
  }

  // Update the derived metrics
  // prv_update_real_time_derived_metrics();
  // prv_update_step_derived_metrics(utc_sec);
  // prv_update_hr_derived_metrics();
}
