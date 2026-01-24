/**
 ******************************************************************************
 * @file   activity_algorithm_kraepelin.c
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
#include "rtconfig.h"
#include "acce_service.h"
#include "util/uuid.h"
#include "util/math.h"
#include "util/time/time.h"
#include "util/units.h"
#include "activity_private.h"
#include "activity_algorithm.h"
#include "activity_algorithm_kraepelin.h"
#include "kraepelin_algorithm.h"

#ifdef BSP_USING_BLOC_BATTERY
#include "bloc_battery.h"
#endif

#define DBG_TAG "activity.kraepelin"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

#define USE_CIRCULAR_BUFFER 0

// How many records we need to store in our circular buffer
// +1 for mgmt overhead
#define ALG_MINUTE_CBUF_NUM_RECORDS (MAX(ALG_MINUTES_PER_DLS_RECORD, ALG_MINUTES_PER_FILE_RECORD) + KALG_MAX_UNCERTAIN_SLEEP_M + 1)

// ---------------------------------------------------------------------------------------------
// Globals
typedef struct
{
  void *mutex;

  KAlgState *k_state; // Pointer to Kraepelin state variables

  // Accumulated steps
  int32_t steps;

  // Last computed step rate information
  uint8_t rate_steps;
  uint16_t rate_elapsed_ms;
  time_t rate_computed_time_s;

  // Minute data
  uint16_t minute_steps;

  AlgMinuteDLSRecord dls_record;
  AlgMinuteFileRecord file_record;

  // Metrics that we compute minute deltas of
  uint32_t prev_distance_mm;
  uint32_t prev_resting_calories;
  uint32_t prev_active_calories;

  // We hold the last N minutes of minute data in this circular buffer so that we can go
  // back and zero out the steps in older minutes once we determine that we were definitely
  // asleep for those minutes (there can be a KALG_MAX_AWAKE_LAG minute lag before we figure out
  // that we woke up).

#if USE_CIRCULAR_BUFFER
  SharedCircularBuffer minute_data_cbuf;
#endif
  AlgMinuteRecord minute_data_storage[ALG_MINUTE_CBUF_NUM_RECORDS];

  AlgMinuteRecord cbuf_record; // space for tmp record here to decrease stack requirements
} AlgState;
static AlgState *s_alg_state = NULL;

// ---------------------------------------------------------------------------------------------
// Mutex helpers
void *mutex_create_recursive(void *name)
{
  const char *mutex_name = (const char *)name;
  return rt_mutex_create(mutex_name, RT_IPC_FLAG_PRIO);
}

void mutex_lock_recursive(void *mutex)
{
  rt_mutex_take((rt_mutex_t)mutex, RT_WAITING_FOREVER);
}

void mutex_unlock_recursive(void *mutex)
{
  rt_mutex_release((rt_mutex_t)mutex);
}

void mutex_destroy(void *mutex)
{
  rt_mutex_delete((rt_mutex_t)mutex);
}

// ----------------------------------------------------------------------------------------------
static bool prv_lock(void)
{
  if (!s_alg_state)
  {
    LOG_E("Trying to use the activity algorithm but it hasn't been initialized");
    return false;
  }
  mutex_lock_recursive(s_alg_state->mutex);
  return true;
}

static void prv_unlock(void)
{
  mutex_unlock_recursive(s_alg_state->mutex);
}

// ----------------------------------------------------------------------------------------------
// Callback provided to kalg_activities_update to create activity sessions.
static void prv_create_activity_session_cb(void *context, KAlgActivityType kalg_activity,
                                           time_t start_utc, uint32_t len_sec, bool ongoing,
                                           bool delete, uint32_t steps, uint32_t resting_calories,
                                           uint32_t active_calories, uint32_t distance_mm)
{
  // Translate to one of the activity.h activity types
  ActivitySessionType activity = ActivitySessionTypeCount;
  switch (kalg_activity)
  {
  case KAlgActivityType_Walk:
    activity = ActivitySessionType_Walk;
    break;
  case KAlgActivityType_Run:
    activity = ActivitySessionType_Run;
    break;
  case KAlgActivityType_RestfulSleep:
    activity = ActivitySessionType_RestfulSleep;
    break;
  case KAlgActivityType_Sleep:
    activity = ActivitySessionType_Sleep;
    break;
  case KAlgActivityTypeCount:
    LOG_W("Unknown activity type %d", (int)kalg_activity);
    break;
  }
  RT_ASSERT(activity != ActivitySessionTypeCount);

  ActivitySession session = {
      .type = activity,
      .start_utc = start_utc,
      .length_min = len_sec / SECONDS_PER_MINUTE,
      .ongoing = ongoing,
      .step_data = {
          .steps = steps,
          .active_kcalories = ROUND(active_calories, ACTIVITY_CALORIES_PER_KCAL),
          .resting_kcalories = ROUND(resting_calories, ACTIVITY_CALORIES_PER_KCAL),
          .distance_meters = ROUND(distance_mm, MM_PER_METER),
      },
  };
  if (delete)
  {
    activity_sessions_prv_delete_activity_session(&session);
  }
  else
  {
    activity_sessions_prv_add_activity_session(&session);
  }
}

// ------------------------------------------------------------------------------------
bool activity_algorithm_init(AccelSamplingRate *sampling_rate)
{
  // Allocate kraepelin state and our globals
  RT_ASSERT(s_alg_state == 0);
  KAlgState *k_state = rt_malloc(kalg_state_size());
  if (k_state)
  {
    s_alg_state = rt_malloc(sizeof(AlgState));
  }
  if (!s_alg_state)
  {
    LOG_E("Not enough memory");
    free(k_state);
    return false;
  }

  // Init globals
  *s_alg_state = (AlgState){
      .mutex = mutex_create_recursive("activity_alg_mutex"),
      .k_state = k_state,
  };
  // Init the algorithm state
  kalg_init(k_state, NULL);

  // Reset all metrics
  activity_algorithm_metrics_changed_notification();

  // Return desired sampling rate
  *sampling_rate = KALG_SAMPLE_HZ;
  return true;
}

static void prv_activity_update_states(time_t utc_sec, AlgMinuteRecord *record_out,
                                       bool shutting_down);

// ------------------------------------------------------------------------------------
// This is called when the activity services is doing down. This tells all of our state machines
// that are are going to be shut down, and to save off any unsaved data/sleep/step sessions.
void activity_algorithm_early_deinit(void)
{
  if (!prv_lock())
  {
    return;
  }
  time_t utc_sec = get_current_time();
  // AlgMinuteRecord data storage needed for underlying function
  AlgMinuteRecord record_out = {};
  prv_activity_update_states(utc_sec, &record_out, true /* shutting_down */);

  prv_unlock();
}

// ------------------------------------------------------------------------------------
bool activity_algorithm_deinit(void)
{
  RT_ASSERT(s_alg_state);
  RT_ASSERT(s_alg_state->k_state);

  mutex_destroy(s_alg_state->mutex);
  rt_free(s_alg_state->k_state);
  rt_free(s_alg_state);
  s_alg_state = NULL;
  return true;
}

// ------------------------------------------------------------------------------------
bool activity_algorithm_set_user(uint32_t height_mm, uint32_t weight_g, ActivityGender gender,
                                 uint32_t age_years)
{
  return true;
}

// ------------------------------------------------------------------------------------
void activity_algorithm_handle_accel(AccelRawData *data, uint32_t num_samples,
                                     uint64_t timestamp_ms)
{
  if (!prv_lock())
  {
    return;
  }
  uint32_t consumed_samples;
  uint16_t new_steps = kalg_analyze_samples(s_alg_state->k_state, data, num_samples,
                                            &consumed_samples);
  s_alg_state->steps += new_steps;
  s_alg_state->minute_steps += new_steps;

  // Update our stepping rate if the algorithm just consumed samples
  if (consumed_samples != 0)
  {
    s_alg_state->rate_steps = new_steps;
    s_alg_state->rate_elapsed_ms = (consumed_samples * MS_PER_SECOND) / KALG_SAMPLE_HZ;
    s_alg_state->rate_computed_time_s = timestamp_ms / MS_PER_SECOND;
  }
  prv_unlock();
}

#define UPDATE_MINUTE_DATA 1

#if UPDATE_MINUTE_DATA
// ------------------------------------------------------------------------------------
// We use NOINLINE to reduce the stack requirements during the minute handler (see PBL-38130)
// Returns distance we traveled in the last minute, in mm.
static uint32_t NOINLINE prv_fill_minute_record(time_t utc_sec, AlgMinuteDLSSample *m_rec)
{
  bool still;
  kalg_minute_stats(s_alg_state->k_state, &m_rec->base.vmc,
                    &m_rec->base.orientation, &still);

  m_rec->base.steps = MIN(s_alg_state->minute_steps, UINT8_MAX);

  // No light sensor available, set light level to 0
  m_rec->base.light = 0;

// Are we connected to a charger?
#ifdef BSP_USING_BLOC_BATTERY
  m_rec->base.plugged_in = battery_get_charge_state()->is_plugged;
#else
  m_rec->base.plugged_in = 0;
#endif

  // Set active flag
  m_rec->base.active = (m_rec->base.steps >= ACTIVITY_ACTIVE_MINUTE_MIN_STEPS) ? 1 : 0;

  // Fill in resting calories, active calories, and distance covered in the previous minute
  const uint32_t resting_calories = activity_metrics_prv_get_resting_calories();
  m_rec->resting_calories = resting_calories - s_alg_state->prev_resting_calories;

  const uint32_t active_calories = activity_metrics_prv_get_active_calories();
  m_rec->active_calories = active_calories - s_alg_state->prev_active_calories;

  const uint32_t distance_mm = activity_metrics_prv_get_distance_mm();
  const uint32_t minute_distance_mm = distance_mm - s_alg_state->prev_distance_mm;
  const int k_mm_per_cm = 10;
  m_rec->distance_cm = ROUND(minute_distance_mm, k_mm_per_cm);

  // TODO: Fill in heart rate, heart rate sample weight, then reset it
  // int32_t median, heart_rate_total_weight_x100;
  // activity_metrics_prv_get_median_hr_bpm(&median, &heart_rate_total_weight_x100);
  // m_rec->heart_rate_bpm = (uint8_t)median;
  // m_rec->heart_rate_total_weight_x100 = (uint16_t)heart_rate_total_weight_x100;
  // m_rec->heart_rate_zone = activity_metrics_prv_get_hr_zone();

  return minute_distance_mm;
}

static void NOINLINE prv_reset_state_minute_handler(const AlgMinuteDLSSample *m_rec)
{
  s_alg_state->prev_resting_calories = activity_metrics_prv_get_resting_calories();
  s_alg_state->prev_active_calories = activity_metrics_prv_get_active_calories();
  s_alg_state->prev_distance_mm = activity_metrics_prv_get_distance_mm();
  activity_metrics_prv_reset_hr_stats();
}

#endif

static void prv_activity_update_states(time_t utc_sec, AlgMinuteRecord *record_out,
                                       bool shutting_down)
{
#if UPDATE_MINUTE_DATA
  // Make sure each record gets time stamped exactly on a minute boundary.
  utc_sec -= (utc_sec % SECONDS_PER_MINUTE);

  // Fill in the minute data structure that we log
  record_out->utc_sec = utc_sec - SECONDS_PER_MINUTE; // this data is for the previous minute
  AlgMinuteDLSSample *m_rec = &record_out->data;
  uint32_t minute_distance_mm = prv_fill_minute_record(utc_sec, m_rec);

  prv_reset_state_minute_handler(m_rec);

  ACTIVITY_LOG_DEBUG("minute handler: steps: %d, orientation: 0x%d, vmc: %d, "
                     "light: %d, plugged_in: %d",
                     m_rec->base.steps, m_rec->base.orientation, m_rec->base.vmc,
                     m_rec->base.light, (int)m_rec->base.plugged_in);

  // Pass the minute data onto the activity detection logic
  kalg_activities_update(s_alg_state->k_state, utc_sec, m_rec->base.steps, m_rec->base.vmc,
                         m_rec->base.orientation, m_rec->base.plugged_in, m_rec->resting_calories,
                         m_rec->active_calories, minute_distance_mm, shutting_down,
                         prv_create_activity_session_cb, NULL);
#endif
}

// ------------------------------------------------------------------------------------
bool activity_algorithm_metrics_changed_notification(void)
{
  if (!prv_lock())
  {
    return false;
  }
  s_alg_state->steps = activity_metrics_prv_get_steps();
  s_alg_state->prev_active_calories = activity_metrics_prv_get_active_calories();
  s_alg_state->prev_resting_calories = activity_metrics_prv_get_resting_calories();
  s_alg_state->prev_distance_mm = activity_metrics_prv_get_distance_mm();
  prv_unlock();
  return true;
}

// ------------------------------------------------------------------------------------
time_t activity_algorithm_get_last_sleep_utc(void)
{
  if (!prv_lock())
  {
    return 0;
  }
  time_t rv = kalg_activity_last_processed_time(s_alg_state->k_state, KAlgActivityType_Sleep);
  prv_unlock();
  return rv;
}

// ------------------------------------------------------------------------------------
// Post-process the passed in sleep sessions. This function identifies which sleep sessions are
// nap sessions and labels them as such
// @param num_input_sessions number of entries in the sessions array
// @param sessions array of activity sessions
void activity_algorithm_post_process_sleep_sessions(uint16_t num_input_sessions,
                                                    ActivitySession *sessions)
{
  if (num_input_sessions == 0)
  {
    return;
  }
  if (!prv_lock())
  {
    return;
  }

  // Now, go through and fix up the labels on the sleep sessions that should be categorized as
  // naps
  ActivitySession *session = sessions;
  ActivitySession *most_recent_nap_session = NULL;
  for (unsigned i = 0; i < num_input_sessions; i++, session++)
  {
    const unsigned start_minute = time_util_get_minute_of_day(session->start_utc);
    const time_t end_utc = session->start_utc + (session->length_min * SECONDS_PER_MINUTE);
    const unsigned end_minute = time_util_get_minute_of_day(end_utc);

    ACTIVITY_LOG_DEBUG("procesing activity %d, start_min: %u, len: %" PRIu16 "",
                       (int)session->type, start_minute, session->length_min);

    // Skip if not a sleep session
    if (!activity_sessions_prv_is_sleep_activity(session->type))
    {
      ACTIVITY_LOG_DEBUG("Not a sleep session");
      continue;
    }

    // Skip if still ongoing
    if (session->ongoing)
    {
      ACTIVITY_LOG_DEBUG("Still ongoing");
      continue;
    }

    // Skip if already labeled as a nap session
    if ((session->type == ActivitySessionType_Nap) || (session->type == ActivitySessionType_RestfulNap))
    {
      if (session->type == ActivitySessionType_Nap)
      {
        most_recent_nap_session = session;
      }
      ACTIVITY_LOG_DEBUG("Already labeled as a nap");
      continue;
    }

    if ((session->length_min > ALG_MAX_NAP_MINUTES) || !WITHIN(start_minute, ALG_PRIMARY_MORNING_MINUTE, ALG_PRIMARY_EVENING_MINUTE) || !WITHIN(end_minute, ALG_PRIMARY_MORNING_MINUTE, ALG_PRIMARY_EVENING_MINUTE))
    {
      // If too long, or not within the primary sleep range, can't be a nap
      ACTIVITY_LOG_DEBUG("Not within nap time bounds or duration");
      continue;
    }

    // If this is a restful session, it must be inside of the most recently labeled nap session
    // for us to re-label it. Note that this logic makes the assumption that restful sessions
    // always follow the container session that they belong to. This is assumption is valid given
    // the way that the algorithm identifies sleep sessions.
    if (session->type == ActivitySessionType_RestfulSleep)
    {
      if (most_recent_nap_session == NULL)
      {
        continue;
      }
      if ((session->start_utc < most_recent_nap_session->start_utc) || (session->start_utc > (most_recent_nap_session->start_utc + (most_recent_nap_session->length_min * SECONDS_PER_MINUTE))))
      {
        continue;
      }
    }

    // Label it as a nap
    if (session->type == ActivitySessionType_Sleep)
    {
      LOG_D("Found nap - start_utc: %d, start_min: %u, len: %d ",
            (int)session->start_utc, start_minute, session->length_min);
      session->type = ActivitySessionType_Nap;
      most_recent_nap_session = session;
    }
    else if (session->type == ActivitySessionType_RestfulSleep)
    {
      LOG_D("Found restful nap - start_utc: %d, start_min: %u, len: %d ",
            (int)session->start_utc, start_minute, session->length_min);
      session->type = ActivitySessionType_RestfulNap;
    }
  }
  prv_unlock();
}

// -------------------------------------------------------------------------------------
static void prv_init_minute_record(AlgMinuteRecordHdr *hdr, time_t utc_sec, bool for_file)
{
  time_t local_time = time_utc_to_local(utc_sec);
  int16_t local_time_offset_15_min = (local_time - utc_sec) / (15 * SECONDS_PER_MINUTE);

  *hdr = (AlgMinuteRecordHdr){
      .version = for_file ? ALG_MINUTE_FILE_RECORD_VERSION : ALG_DLS_MINUTES_RECORD_VERSION,
      .time_utc = utc_sec,
      .time_local_offset_15_min = local_time_offset_15_min,
      .sample_size = for_file ? sizeof(AlgMinuteFileSample) : sizeof(AlgMinuteDLSSample),
  };
}

// -------------------------------------------------------------------------------------
// We use NOINLINE to reduce the stack requirements during the minute handler (see PBL-38130)
static void NOINLINE prv_set_dls_minute_record_entry(AlgMinuteDLSRecord *dls_record,
                                                     AlgMinuteDLSSample *data, uint16_t sample_idx,
                                                     time_t sample_utc, bool was_sleeping)
{
  if (sample_idx == 0)
  {
    // If first record, init the header
    prv_init_minute_record(&dls_record->hdr, sample_utc, false /*to_file*/);
  }
  // Put in this minute
  dls_record->samples[sample_idx] = *data;
  dls_record->hdr.num_samples = sample_idx + 1;

  if (was_sleeping && (dls_record->samples[sample_idx].base.steps != 0))
  {
    // Subtract from our total steps since we decided we were definitely sleeping during
    // this minute
    LOG_D("Subtracting %d steps that occurred during sleep",
          dls_record->samples[sample_idx].base.steps);
    s_alg_state->steps -= dls_record->samples[sample_idx].base.steps;
    s_alg_state->steps = MAX(0, s_alg_state->steps);
    dls_record->samples[sample_idx].base.steps = 0;
    dls_record->samples[sample_idx].base.active = false;
    dls_record->samples[sample_idx].active_calories = 0;
    dls_record->samples[sample_idx].distance_cm = 0;
  }
}

// -------------------------------------------------------------------------------------
// Prepare a minute record for writing. Either file_record or dls_record should be non-NULL,
// never both.
// Returns true if we have enough data to prepare a record, false if not enough data.
// We use NOINLINE to reduce the stack requirements during the minute handler (see PBL-38130)
static bool NOINLINE prv_prepare_minute_data(uint16_t uncertain_m, time_t sleep_start_utc,
                                             uint16_t sleep_len_m, AlgMinuteFileRecord *file_record,
                                             AlgMinuteDLSRecord *dls_record, bool force_send)
{
#if USE_CIRCULAR_BUFFER
  // Get the circular buffer client we are working with
  SharedCircularBufferClient *cbuf_client = file_record ? &s_alg_state->file_minute_data_client
                                                        : &s_alg_state->dls_minute_data_client;
#endif
  const int16_t minutes_per_record = file_record ? ALG_MINUTES_PER_FILE_RECORD : ALG_MINUTES_PER_DLS_RECORD;

  // Empty the circular buffer while we have enough for a record
  time_t sleep_end_utc = sleep_start_utc + (sleep_len_m * SECONDS_PER_MINUTE);
#if USE_CIRCULAR_BUFFER
  int16_t certain_m = (shared_circular_buffer_get_read_space_remaining(
                           &s_alg_state->minute_data_cbuf, cbuf_client) /
                       sizeof(AlgMinuteRecord)) -
                      uncertain_m;
#else
  int16_t certain_m = 1;
#endif
  int minutes_this_record = MIN(certain_m, minutes_per_record);
  if (minutes_this_record == 0)
  {
    // nothing to send, even if we really wanted to
    return false;
  }
  else if (!force_send && minutes_this_record < minutes_per_record)
  {
    // didn't collect enough data for our regularly scheduled program
    return false;
  }

  // Fill up the next record
  AlgMinuteRecord *cbuf_record = &s_alg_state->cbuf_record;
  for (int i = 0; i < minutes_this_record; i++)
  {
    uint16_t length_out;
#if USE_CIRCULAR_BUFFER
    bool success = shared_circular_buffer_read_consume(
        &s_alg_state->minute_data_cbuf, cbuf_client, sizeof(*cbuf_record), (uint8_t *)cbuf_record,
        &length_out);
    RT_ASSERT(success);
#endif

    // See if we need to zero out steps in this record. We check that the start of the minute
    // is within the sleep bounds. The WITHIN macro returns true if the test value is
    // <= end_value, so we need to subract one minute from the end to see if the start of this
    // test minute is entirely within the sleep range.
    bool was_sleeping = WITHIN(cbuf_record->utc_sec, sleep_start_utc,
                               sleep_end_utc - SECONDS_PER_MINUTE);

    // Handle writing the record out to PFS
    if (file_record)
    {
      // prv_set_file_minute_record_entry(file_record, &cbuf_record->data, i, cbuf_record->utc_sec,
      //                                  was_sleeping);
    }
    else
    {
      prv_set_dls_minute_record_entry(dls_record, &cbuf_record->data, i, cbuf_record->utc_sec,
                                      was_sleeping);
    }
  }
  return true;
}

// -------------------------------------------------------------------------------------
// If we have enough minute data in our circular buffer, write it out to either the minute
// file or to data logging
static void prv_send_minute_data(uint16_t uncertain_m, time_t sleep_start_utc,
                                 uint16_t sleep_len_m, bool to_file, bool force_send)
{
  AlgMinuteFileRecord *file_record = NULL;
  AlgMinuteDLSRecord *dls_record = NULL;

  if (to_file)
  {
    file_record = &s_alg_state->file_record;
  }
  else
  {
    dls_record = &s_alg_state->dls_record;
  }

  // While we have whole minute records available for sending, send them.
  while (prv_prepare_minute_data(uncertain_m,
                                 sleep_start_utc,
                                 sleep_len_m,
                                 file_record,
                                 dls_record,
                                 force_send))
  {
    RT_ASSERT((file_record == NULL) != (dls_record == NULL));
    if (file_record)
    {
      // prv_write_minute_file_record(file_record);
    }

    if (dls_record)
    {
      // Handle writing the record out to data logging
      LOG_I("Logging %d MLD Records, First UTC: %d",
            dls_record->hdr.num_samples, dls_record->hdr.time_utc);
    }
  }
}

extern void send_list_to_hcpu(time_t utc_now, uint8_t steps, uint8_t orientation, uint16_t vmc);
// -------------------------------------------------------------------------------------------
// Handle storage and logging of the minute data
static void prv_log_minute_data(time_t utc_now, AlgMinuteRecord *minute_rec)
{
  // #if USE_CIRCULAR_BUFFER
  //   // Store the minute data into our circular buffer. The only place we ever read from this buffer
  //   // is below in this same method (during prv_send_minute_data) only from the KernelBG task, so no
  //   // need for a lock.
  //   bool success = shared_circular_buffer_write(&s_alg_state->minute_data_cbuf, (uint8_t *)minute_rec,
  //                                               sizeof(*minute_rec), false /*advance_slackers*/);
  //   if (!success)
  //   {
  //     // Although unlikely, we could get buffer overruns if we failed to open up the data logging
  //     // session after a number of retries. In that case, we will start dropping the oldest
  //     // minute data from data logging.
  //     LOG_E("Circular buffer overrun");
  //     success = shared_circular_buffer_write(&s_alg_state->minute_data_cbuf, (uint8_t *)minute_rec,
  //                                            sizeof(*minute_rec), true /*advance_slackers*/);
  //   }
  //   RT_ASSERT(success);
  // #endif

  //   // Find the number of "certain" minutes we have in the buffer. When we are asleep, the
  //   // most recent N minutes in the buffer will be uncertain because we don't know that we woke
  //   // until we see at least M minutes in a row of activity.
  //   KAlgOngoingSleepStats sleep_stats;
  //   kalg_get_sleep_stats(s_alg_state->k_state, &sleep_stats);

  //   // If there are any uncertain minutes, they will always be at the end
  //   int16_t uncertain_m = 0;
  //   if (sleep_stats.uncertain_start_utc != 0)
  //   {
  //     uncertain_m = (utc_now - sleep_stats.uncertain_start_utc) / SECONDS_PER_MINUTE;
  //   }
  //   if (uncertain_m > KALG_MAX_UNCERTAIN_SLEEP_M)
  //   {
  //     LOG_E("Unexpectedly large number of uncertain minutes");
  //     uncertain_m = KALG_MAX_UNCERTAIN_SLEEP_M;
  //   }

  //   // Send whatever complete DLS records we have
  //   prv_send_minute_data(uncertain_m, sleep_stats.sleep_start_utc, sleep_stats.sleep_len_m,
  //                        false /*to_file*/, false /*force_send*/);

  //   // Send whatever complete minute file records we have
  //   prv_send_minute_data(uncertain_m, sleep_stats.sleep_start_utc, sleep_stats.sleep_len_m,
  //                        true /*to_file*/, false /*force_send*/);

  // send list of: {steps, orientation, vmc} to hcpu
  send_list_to_hcpu(utc_now, minute_rec->data.base.steps, minute_rec->data.base.orientation, minute_rec->data.base.vmc);
}

// ------------------------------------------------------------------------------------
void activity_algorithm_minute_handler(time_t utc_sec, AlgMinuteRecord *record_out)
{
  if (!prv_lock())
  {
    return;
  }

  prv_activity_update_states(utc_sec, record_out, false /* shutting_down */);

  // Handle storage and logging of the minute data
  prv_log_minute_data(utc_sec, record_out);
  // Reset the minute stats
  s_alg_state->minute_steps = 0;
  prv_unlock();
}

// ------------------------------------------------------------------------------------
bool activity_algorithm_get_steps(uint16_t *steps)
{
  if (!prv_lock())
  {
    return false;
  }
  *steps = s_alg_state->steps;
  prv_unlock();
  return true;
}

// ------------------------------------------------------------------------------------
bool activity_algorithm_get_step_rate(uint16_t *steps, uint32_t *elapsed_ms, time_t *end_sec)
{
  if (!prv_lock())
  {
    return false;
  }
  *steps = s_alg_state->rate_steps;
  *elapsed_ms = s_alg_state->rate_elapsed_ms;
  *end_sec = s_alg_state->rate_computed_time_s;
  prv_unlock();
  return true;
}

// ------------------------------------------------------------------------------------
void activity_algorithm_enable_activity_tracking(bool enable)
{
  if (!activity_tracking_on())
  {
    return;
  }

  if (!prv_lock())
  {
    return;
  }

  kalg_enable_activity_tracking(s_alg_state->k_state, enable);

  prv_unlock();
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
