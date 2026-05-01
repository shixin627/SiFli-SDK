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

#include "util/size.h"
#include "util/math.h"
#include "activity.h"
#include "activity_algorithm.h"
#include "activity_insights.h"
#include "activity_private.h"
#include "watch_sys_service.h"

#define DBG_TAG "activity.session"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// ------------------------------------------------------------------------------------
// Figure out the cutoff times for sleep and step activities for today given the
// current time
static void prv_get_earliest_end_times_utc(time_t utc_sec,
                                           time_t *sleep_earliest_end_utc,
                                           time_t *step_earliest_end_utc)
{
    time_t start_of_today_utc = time_util_get_midnight_of(utc_sec);
    int last_sleep_second_of_day =
        ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY * SECONDS_PER_MINUTE;
    *sleep_earliest_end_utc =
        start_of_today_utc - (SECONDS_PER_DAY - last_sleep_second_of_day);
    *step_earliest_end_utc = start_of_today_utc;
}

// ------------------------------------------------------------------------------------
// Remove all activity sessions that are older than "today", those that are
// invalid because they are in the future, and optionally those that are still
// ongoing.
void activity_sessions_prv_remove_out_of_range_activity_sessions(
    time_t utc_sec, bool remove_ongoing)
{
    ActivityState *state = activity_private_state();
    uint16_t num_sessions_to_clear = 0;
    uint16_t *session_entries = &state->activity_sessions_count;
    ActivitySession *sessions = state->activity_sessions;

    // Figure out the cutoff times for sleep and step activities
    time_t sleep_earliest_end_utc;
    time_t step_earliest_end_utc;
    prv_get_earliest_end_times_utc(utc_sec, &sleep_earliest_end_utc,
                                   &step_earliest_end_utc);

    for (uint32_t i = 0; i < *session_entries; i++)
    {
        time_t end_utc;
        if (activity_sessions_prv_is_sleep_activity(sessions[i].type))
        {
            end_utc = sleep_earliest_end_utc;
        }
        else
        {
            end_utc = step_earliest_end_utc;
        }

        // See if we should keep this activity
        time_t end_time = sessions[i].start_utc +
                          (sessions[i].length_min * SECONDS_PER_MINUTE);
        if ((end_time >= end_utc) && (end_time <= utc_sec) &&
            (!remove_ongoing || !sessions[i].ongoing))
        {
            // Keep it
            continue;
        }

        // This one needs to be removed
        uint32_t remaining = *session_entries - i - 1;
        memcpy(&sessions[i], &sessions[i + 1], remaining * sizeof(*sessions));
        (*session_entries)--;
        num_sessions_to_clear++;
        i--;
    }

    // Zero out unused sessions at end. This is important because when we
    // re-init from stored settings, we detect the number of sessions we have by
    // checking for non-zero ones
    memset(&sessions[*session_entries], 0,
           num_sessions_to_clear * sizeof(ActivitySession));
}

// ------------------------------------------------------------------------------------
// Return true if the given activity type is a sleep activity
bool activity_sessions_prv_is_sleep_activity(ActivitySessionType activity_type)
{
    switch (activity_type)
    {
    case ActivitySessionType_Sleep:
    case ActivitySessionType_RestfulSleep:
    case ActivitySessionType_Nap:
    case ActivitySessionType_RestfulNap:
        return true;
    case ActivitySessionType_Walk:
    case ActivitySessionType_Run:
    case ActivitySessionType_Open:
        return false;
    case ActivitySessionType_None:
    case ActivitySessionTypeCount:
        break;
    }
    return false;
}

// ------------------------------------------------------------------------------------
// Return true if two activity sessions are equal in their type and start time
// @param[in] session_a ptr to first session
// @param[in] session_b ptr to second session
// @param[in] any_sleep if true, a match occurs if session_a and session_b are
// both sleep
//              activities, even if they are different types of sleep
static bool prv_activity_sessions_equal(ActivitySession *session_a,
                                        ActivitySession *session_b,
                                        bool any_sleep)
{
    bool type_matches;

    const bool a_is_sleep =
        activity_sessions_prv_is_sleep_activity(session_a->type);
    const bool b_is_sleep =
        activity_sessions_prv_is_sleep_activity(session_b->type);

    if (any_sleep && a_is_sleep && b_is_sleep)
    {
        type_matches = true;
    }
    else
    {
        type_matches = (session_a->type == session_b->type);
    }

    return type_matches && (session_a->start_utc == session_b->start_utc);
}

// ------------------------------------------------------------------------------------
// Register a new activity. Called by the algorithm code when it detects a new
// activity. If we already have this activity registered, it is updated.
void activity_sessions_prv_add_activity_session(ActivitySession *session)
{
    ActivityState *state = activity_private_state();
    mutex_lock_recursive(state->mutex);
    {
        if (!session->ongoing)
        {
            state->need_activities_saved = true;
        }

        // Modifying a sleep session?
        if (activity_sessions_prv_is_sleep_activity(session->type))
        {
            state->sleep_sessions_modified = true;
        }

        // If this is an existing activity, update it
        ActivitySession *stored_session = state->activity_sessions;
        for (uint16_t i = 0; i < state->activity_sessions_count;
             i++, stored_session++)
        {
            if (prv_activity_sessions_equal(session, stored_session,
                                            true /*any_sleep*/))
            {
                state->activity_sessions[i] = *session;
                goto unlock;
            }
        }

        // If no more room, fail
        if (state->activity_sessions_count >=
            ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT)
        {
            LOG_W("No more room for additional activities");
            goto unlock;
        }

        // Add this activity in
        LOG_I("Adding activity session %d, start_time: %" PRIu32,
              (int)session->type, (uint32_t)session->start_utc);
        state->activity_sessions[state->activity_sessions_count++] = *session;
    }
unlock:
    mutex_unlock_recursive(state->mutex);
}

// ------------------------------------------------------------------------------------
// Delete an ongoing activity. Called by the algorithm code when it decides that
// an activity that was previously ongoing should not be registered after all.
void activity_sessions_prv_delete_activity_session(ActivitySession *session)
{
    ActivityState *state = activity_private_state();
    mutex_lock_recursive(state->mutex);
    {
        // Look for this activity
        int found_session_idx = -1;
        ActivitySession *stored_session = state->activity_sessions;
        for (uint16_t i = 0; i < state->activity_sessions_count;
             i++, stored_session++)
        {
            if (prv_activity_sessions_equal(session, stored_session,
                                            false /*any_sleep*/))
            {
                found_session_idx = i;
                break;
            }
        }

        // If session not found, do nothing
        if (found_session_idx < 0)
        {
            LOG_W("Session to delete not found");
            goto unlock;
        }

        // The session we are deleting must be ongoing
        RT_ASSERT(stored_session->ongoing);

        // Remove this session
        int num_to_move =
            state->activity_sessions_count - found_session_idx - 1;
        RT_ASSERT(num_to_move >= 0);
        if (num_to_move == 0)
        {
            memset(&state->activity_sessions[found_session_idx], 0,
                   sizeof(ActivitySession));
        }
        else
        {
            memmove(&state->activity_sessions[found_session_idx],
                    &state->activity_sessions[found_session_idx + 1],
                    num_to_move * sizeof(ActivitySession));
        }
        state->activity_sessions_count--;
    }
unlock:
    mutex_unlock_recursive(state->mutex);
}

// --------------------------------------------------------------------------------------------
// Compute the total number of restful sleep seconds within a range of time
static uint32_t prv_sleep_restful_seconds(uint32_t num_sessions,
                                          ActivitySession *sessions,
                                          time_t start_utc, time_t end_utc)
{
    // Iterate through the sleep sessions, accumulating the total restful
    // seconds seen between start_utc and end_utc
    ActivitySession *session = sessions;
    uint32_t restful_sec = 0;
    for (uint32_t i = 0; i < num_sessions; i++, session++)
    {
        if ((session->type != ActivitySessionType_RestfulSleep) &&
            (session->type != ActivitySessionType_RestfulNap))
        {
            continue;
        }
        if ((session->start_utc >= start_utc) &&
            ((time_t)(session->start_utc +
                      (session->length_min * SECONDS_PER_MINUTE)) <= end_utc))
        {
            restful_sec += session->length_min * SECONDS_PER_MINUTE;
        }
    }
    return restful_sec;
}

// This structre holds stats we collected from going through a list of sleep
// sessions. It is filled in by prv_compute_sleep_stats
typedef struct
{
    ActivityScalarStore total_minutes;
    ActivityScalarStore restful_minutes;
    time_t enter_utc;      // When we entered sleep
    time_t today_exit_utc; // last exit time for today, for regular sleep only
    time_t last_exit_utc;  // last exit time (sleep or nap, ignoring "today"
                           // boundary)
    time_t last_deep_exit_utc; // last deep sleep exit time (sleep or nap,
                               // ignoring "today" boundary)
    uint32_t last_session_len_sec;
} ActivitySleepStats;

// --------------------------------------------------------------------------------------------
// Goes through a list of activity sessions and collect sleep stats
// @param[in] now_utc the UTC time when the activity sessions were computed
// @param[in] min_end_utc Only include sleep sessions that end AFTER this time
// @param[in] max_end_utc Only include sleep sessions that end BEFORE this time
// @param[in] last_processed_utc When activity sessions were computed, this is
// the UTC of the
//        most recent minute we had access to when activities were computed.
// @param[out] stats this structure is filled in with the sleep stats
// @return True if there were sleep session, False if not
static bool prv_compute_sleep_stats(time_t now_utc, time_t min_end_utc,
                                    time_t max_end_utc,
                                    ActivitySleepStats *stats)
{
    ActivityState *state = activity_private_state();
    *stats = (ActivitySleepStats){};

    bool rv = false;

    // Iterate through the sleep sessions, accumulating the total sleep minutes,
    // total restful minutes, sleep enter time, and sleep exit time.
    ActivitySession *session = state->activity_sessions;
    for (uint32_t i = 0; i < state->activity_sessions_count; i++, session++)
    {
        // Get info on this session
        stats->last_session_len_sec = session->length_min * SECONDS_PER_MINUTE;
        time_t session_exit_utc =
            session->start_utc + stats->last_session_len_sec;

        // Skip if it ended too early
        if (session_exit_utc < min_end_utc)
        {
            continue;
        }

        if ((session->type == ActivitySessionType_Sleep) ||
            (session->type == ActivitySessionType_Nap))
        {
            rv = true;
            // Accumulate sleep container stats
            if (session_exit_utc <= max_end_utc)
            {
                stats->total_minutes += session->length_min;
            }
            // Only regular sleep (not naps) should affect the enter and exit
            // times
            if (session->type == ActivitySessionType_Sleep)
            {
                stats->enter_utc =
                    (stats->enter_utc != 0)
                        ? MIN(session->start_utc, stats->enter_utc)
                        : session->start_utc;
                if ((session_exit_utc > stats->today_exit_utc) &&
                    (session_exit_utc <= max_end_utc))
                {
                    stats->today_exit_utc = session_exit_utc;
                }
            }
            stats->last_exit_utc = MAX(session_exit_utc, stats->last_exit_utc);
        }
        else if ((session->type == ActivitySessionType_RestfulSleep) ||
                 (session->type == ActivitySessionType_RestfulNap))
        {
            if (session_exit_utc <= max_end_utc)
            {
                // Accumulate restful sleep stats
                stats->restful_minutes += session->length_min;
            }
            stats->last_deep_exit_utc =
                MAX(stats->last_deep_exit_utc, session_exit_utc);
        }
    }

    return rv;
}

// --------------------------------------------------------------------------------------------
// Goes through a list of activity sessions and updates our sleep totals in the
// metrics accordingly. We also take this opportunity to post a sleep metric
// changed event for the SDK if the sleep totals have changed.
// @param num_sessions the number of sessions in the sessions array
// @param sessions array of activity sessions
// @param now_utc the UTC time when the activity sessions were computed
// @param max_end_utc Only include sleep sessions that end BEFORE this time
// @param last_processed_utc When activity sessions were computed, this is the
// UTC of the
//        most recent minute we had access to when activities were computed.
static void prv_update_sleep_metrics(time_t now_utc, time_t max_end_utc,
                                     time_t last_processed_utc)
{
    ActivityState *state = activity_private_state();
    mutex_lock_recursive(state->mutex);
    {
        // We will be filling in this structure based on the sleep sessions
        ActivitySleepData *sleep_data = &state->sleep_data;

        // If we detect a change in the sleep metrics, we want to post a health
        // event
        ActivitySleepData prev_sleep_data = *sleep_data;

        // Collect stats on sleep
        ActivitySleepStats stats;
        if (!prv_compute_sleep_stats(now_utc, 0 /*min_end_utc*/, max_end_utc,
                                     &stats))
        {
            // We didn't have any sleep data exit early
            goto unlock;
        }

        // Update our sleep metrics
        sleep_data->total_minutes = stats.total_minutes;
        sleep_data->restful_minutes = stats.restful_minutes;

        // Fill in the enter and exit minute
        uint16_t enter_minute = time_util_get_minute_of_day(stats.enter_utc);
        uint16_t exit_minute =
            time_util_get_minute_of_day(stats.today_exit_utc);
        sleep_data->enter_at_minute = enter_minute;
        sleep_data->exit_at_minute = exit_minute;

        // Fill in the rest of the sleep data metrics: the current state, and
        // how long we have been in the current state
        uint32_t delta_min =
            abs((int32_t)(last_processed_utc - stats.last_exit_utc)) /
            SECONDS_PER_MINUTE;

        // Figure out our current state
        if (delta_min > 1)
        {
            // We are awake
            sleep_data->cur_state = ActivitySleepStateAwake;
            if (stats.last_exit_utc != 0)
            {
                sleep_data->cur_state_elapsed_minutes =
                    (now_utc - stats.last_exit_utc) / SECONDS_PER_MINUTE;
            }
            else
            {
                sleep_data->cur_state_elapsed_minutes = MINUTES_PER_DAY;
            }
        }
        else
        {
            // We are still sleeping
            if (stats.last_deep_exit_utc == stats.last_exit_utc)
            {
                sleep_data->cur_state = ActivitySleepStateRestfulSleep;
            }
            else
            {
                sleep_data->cur_state = ActivitySleepStateLightSleep;
            }
            sleep_data->cur_state_elapsed_minutes =
                (stats.last_session_len_sec + now_utc - stats.last_exit_utc) /
                SECONDS_PER_MINUTE;
        }

        // If the info that is part of a health sleep event has changed, send
        // out a notification event
        if ((sleep_data->total_minutes != prev_sleep_data.total_minutes) ||
            (sleep_data->restful_minutes != prev_sleep_data.restful_minutes))
        {
            // TODO: Post a sleep changed event
            // PebbleEvent e = {
            //     .type = PEBBLE_HEALTH_SERVICE_EVENT,
            //     .health_event = {
            //         .type = HealthEventSleepUpdate,
            //         .data.sleep_update = {
            //             .total_seconds = sleep_data->total_minutes *
            //             SECONDS_PER_MINUTE, .total_restful_seconds =
            //             sleep_data->restful_minutes * SECONDS_PER_MINUTE,
            //         },
            //     },
            // };
            // event_put(&e);
            if (watch_sys_sync.notify_sleep_state)
            {
                watch_sys_sync.notify_sleep_state(
                    sleep_data->total_minutes * SECONDS_PER_MINUTE,
                    sleep_data->restful_minutes * SECONDS_PER_MINUTE);
            }
        }

        if (sleep_data->cur_state != prev_sleep_data.cur_state)
        {
            // Debug logging
            ACTIVITY_LOG_DEBUG(
                "total_min: %d, deep_min: %d, state: %d, state_min: %d",
                sleep_data->total_minutes, sleep_data->restful_minutes,
                sleep_data->cur_state, sleep_data->cur_state_elapsed_minutes);
        }
    }
unlock:
    mutex_unlock_recursive(state->mutex);
}

// --------------------------------------------------------------------------------------------
void activity_sessions_prv_get_sleep_bounds_utc(time_t now_utc,
                                                time_t *enter_utc,
                                                time_t *exit_utc)
{
    // Get useful UTC times
    time_t start_of_today_utc = time_util_get_midnight_of(now_utc);
    int minute_of_day = time_util_get_minute_of_day(now_utc);
    int last_sleep_second_of_day =
        ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY * SECONDS_PER_MINUTE;

    int first_sleep_utc;
    if (minute_of_day < ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY)
    {
        // It is before the ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY (currently 9pm)
        // cutoff, so use the previou day's cutoff
        first_sleep_utc =
            start_of_today_utc - (SECONDS_PER_DAY - last_sleep_second_of_day);
    }
    else
    {
        // It is after 9pm, so use the 9pm cutoff
        first_sleep_utc = start_of_today_utc + last_sleep_second_of_day;
    }

    // Compute stats for today
    ActivitySleepStats stats;
    prv_compute_sleep_stats(now_utc, first_sleep_utc /*min_utc*/,
                            now_utc /*max_utc*/, &stats);
    *enter_utc = stats.enter_utc;
    *exit_utc = stats.today_exit_utc;
}

// --------------------------------------------------------------------------------------------
// Goes through a list of activity sessions and logs new ones to data logging
static void prv_log_activities(time_t now_utc)
{
    ActivityState *state = activity_private_state();
    // Activity classes. All of the activities in a class share the same
    // "_exit_at_utc" state in the globals and the same settings key to persist
    // it.
    enum
    {
        // for ActivitySessionType_Sleep, ActivitySessionType_Nap
        ActivityClass_Sleep = 0,
        // for ActivitySessionType_RestfulSleep, ActivitySessionType_RestfulNap
        ActivityClass_RestfulSleep = 1,
        // for ActivitySessionType_Walk, ActivitySessionType_Run,
        // ActivitySessionType_Open
        ActivityClass_Step = 2,

        // Leave at end
        ActivityClassCount,
    };

    // List of event classes and info on each
    typedef struct
    {
        ActivitySettingsKey key; // settings key used to store last UTC time for
                                 // this activity class
        time_t *exit_utc;        // pointer to last UTC time in our globals
        bool modified;           // true if we need to update it.
    } ActivityClassParams;

    ActivityClassParams class_settings[ActivityClassCount] = {
        {ActivitySettingsKeyLastSleepActivityUTC,
         &state->logged_sleep_activity_exit_at_utc, false},

        {ActivitySettingsKeyLastRestfulSleepActivityUTC,
         &state->logged_restful_sleep_activity_exit_at_utc, false},

        {ActivitySettingsKeyLastStepActivityUTC,
         &state->logged_step_activity_exit_at_utc, false},
    };

    bool logged_event = false;
    ActivitySession *session = state->activity_sessions;
    for (uint32_t i = 0; i < state->activity_sessions_count; i++, session++)
    {
        // Get info on this activity
        uint32_t session_len_sec = session->length_min * SECONDS_PER_MINUTE;
        time_t session_exit_utc = session->start_utc + session_len_sec;

        ActivityClassParams *params = NULL;
        switch (session->type)
        {
        case ActivitySessionType_Sleep:
        case ActivitySessionType_Nap:
            params = &class_settings[ActivityClass_Sleep];
            break;

        case ActivitySessionType_RestfulSleep:
        case ActivitySessionType_RestfulNap:
            params = &class_settings[ActivityClass_RestfulSleep];
            break;

        case ActivitySessionType_Walk:
        case ActivitySessionType_Run:
        case ActivitySessionType_Open:
            params = &class_settings[ActivityClass_Step];
            break;
        case ActivitySessionType_None:
        case ActivitySessionTypeCount:
            break;
        }
        RT_ASSERT(params);

        // If this is an event we already logged, or it's still onging, don't
        // log it
        if (session->ongoing || (session_exit_utc <= *params->exit_utc))
        {
            continue;
        }

        // Don't log *any* sleep events until we know for sure we are awake. For
        // restful sessions in particular, even if the session ended, it might
        // later be converted to a restful nap session (after the container
        // sleep session it is in finally ends).
        if (activity_sessions_prv_is_sleep_activity(session->type))
        {
            if (state->sleep_data.cur_state != ActivitySleepStateAwake)
            {
                continue;
            }
        }

        *params->exit_utc = session_exit_utc;
        params->modified = true;
        logged_event = true;
    }

    // Update settings file if any events were logged
    if (logged_event)
    {
        mutex_lock_recursive(state->mutex);
        // if (file)
        {
            for (int i = 0; i < ActivityClassCount; i++)
            {
                ActivityClassParams *params = &class_settings[i];
            }
        }
        mutex_unlock_recursive(state->mutex);
    }
}

// --------------------------------------------------------------------------------------
void NOINLINE activity_sessions_prv_minute_handler(time_t utc_sec)
{
    ActivityState *state = activity_private_state();
    time_t last_sleep_processed_utc = activity_algorithm_get_last_sleep_utc();

    // Post process sleep sessions if we got any new sleep sessions that showed
    // up
    if (state->sleep_sessions_modified)
    {
        // Post-process the sleep activities. This is where we relabel sleep
        // sessions as nap sessions, depending on time and length heuristics.
        activity_algorithm_post_process_sleep_sessions(
            state->activity_sessions_count, state->activity_sessions);
        state->sleep_sessions_modified = false;
    }

    // Update sleep metrics
    // For today's metrics, we include sleep sessions that end between
    // ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY the previous day and
    // ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY today.
    // activity_algorithm_get_activity_sessions() insures that we only get
    // sessions that end after ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY the previous
    // day, so we just need to insure that the end BEFORE
    // ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY today.
    int last_sleep_utc_of_day =
        time_util_get_midnight_of(utc_sec) +
        ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY * SECONDS_PER_MINUTE;
    prv_update_sleep_metrics(utc_sec, last_sleep_utc_of_day,
                             last_sleep_processed_utc);

    // Log any new activites we detected to the phone
    prv_log_activities(utc_sec);
}

// ------------------------------------------------------------------------------------------------
bool activity_sessions_is_session_type_ongoing(ActivitySessionType type)
{
    ActivityState *state = activity_private_state();
    bool rv = false;

    mutex_lock_recursive(state->mutex);
    {
        for (int i = 0; i < state->activity_sessions_count; i++)
        {
            const ActivitySession *session = &state->activity_sessions[i];
            if (session->type == type && session->ongoing)
            {
                rv = true;
                break;
            }
        }
    }
    mutex_unlock_recursive(state->mutex);
    return rv;
}
