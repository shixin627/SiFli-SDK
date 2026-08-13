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

// #include "drivers/rtc.h"
#include "time.h"
#include "util/math.h"
#include <string.h>

// timezone abbreviation
static char s_timezone_abbr[TZ_LEN] = {0}; // longest timezone abbreviation is 5 char + null
static int32_t s_timezone_gmtoffset = 0;
static int32_t s_dst_adjust = SECONDS_PER_HOUR;
static time_t s_dst_start = 0;
static time_t s_dst_end = 0;

static const uint8_t s_mon_lengths[2][MONTHS_PER_YEAR] = {
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
    {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};


int32_t time_get_gmtoffset(void)
{
  return s_timezone_gmtoffset;
}

bool time_get_isdst(time_t utc_time)
{
  // do we have any DST set for the timezone we are in
  if ((s_dst_start == 0) || (s_dst_end == 0))
  {
    return false;
  }

  return ((s_dst_start <= utc_time) && (utc_time < s_dst_end));
}

int time_will_transition_dst(time_t prev, time_t next)
{
  if (time_get_isdst(prev) == time_get_isdst(next))
  {
    return 0;
  }
  else if (time_get_isdst(prev))
  {
    return time_get_dstoffset();
  }
  else
  {
    return -time_get_dstoffset();
  }
}

int32_t time_get_dstoffset(void)
{
  return s_dst_adjust;
}

time_t time_get_dst_start(void)
{
  return s_dst_start;
}

time_t time_get_dst_end(void)
{
  return s_dst_end;
}

time_t time_utc_to_local(time_t utc_time)
{
  utc_time += time_get_isdst(utc_time) ? s_dst_adjust : 0;
  utc_time += s_timezone_gmtoffset;
  return utc_time;
}

time_t time_local_to_utc(time_t local_time)
{
  // Note that there is 1 hour a year where it is impossible to undo the DST offset based solely
  // on local time. For example, if the clock goes backward by 1 hour at 2am, then all times
  // between 1am and 2am will appear twice, and there is no way to tell which of the two
  // intervals we are being passed.
  local_time -= s_timezone_gmtoffset;
  local_time -= time_get_isdst(local_time - s_dst_adjust) ? s_dst_adjust : 0;
  return local_time;
}

void time_get_timezone_abbr(char *out_buf, time_t utc_time)
{
  if (!out_buf)
  {
    return;
  }
  strncpy(out_buf, s_timezone_abbr, TZ_LEN);
  out_buf[TZ_LEN - 1] = 0;

  // Timezones with daylight savings, update modifier with current dst char
  // ie. P*T is PDT for daylight savings, PST for non-daylight savings
  char *tz_zone_dst_char = memchr(out_buf, '*', TZ_LEN);
  if (tz_zone_dst_char)
  {
    *tz_zone_dst_char = (time_get_isdst(utc_time)) ? 'D' : 'S';
    // Workaround for UK Winter, Greenwich Mean Time; UK Summer, British Summer Time
    if (!strncmp(out_buf, "BDT", TZ_LEN))
    {
      strncpy(out_buf, "BST", TZ_LEN);
    }
    else if (!strncmp(out_buf, "BST", TZ_LEN))
    {
      strncpy(out_buf, "GMT", TZ_LEN);
    }
  }
}

void time_util_split_seconds_into_parts(uint32_t seconds,
                                        uint32_t *day_part, uint32_t *hour_part, uint32_t *minute_part, uint32_t *second_part)
{
  *day_part = seconds / SECONDS_PER_DAY;
  seconds -= *day_part * SECONDS_PER_DAY;

  *hour_part = seconds / SECONDS_PER_HOUR;
  seconds -= *hour_part * SECONDS_PER_HOUR;

  *minute_part = seconds / SECONDS_PER_MINUTE;
  seconds -= *minute_part * SECONDS_PER_MINUTE;

  *second_part = seconds;
}

int time_util_get_num_hours(int hours, bool is24h)
{
  return is24h ? hours : (hours + 12 - 1) % 12 + 1;
}

int time_util_get_seconds_until_daily_time(struct tm *time, int hour, int minute)
{
  int hour_diff = hour - time->tm_hour;

  if (hour < time->tm_hour || (hour == time->tm_hour && minute <= time->tm_min))
  {
    // It's past the mark, skip to tomorrow.
    hour_diff += HOURS_PER_DAY;
  }

  int minutes_diff = (hour_diff * MINUTES_PER_HOUR) + (minute - time->tm_min);
  return (minutes_diff * SECONDS_PER_MINUTE) - (time->tm_sec);
}

void time_util_update_timezone(const TimezoneInfo *tz_info)
{
  strncpy(s_timezone_abbr, tz_info->tm_zone, sizeof(tz_info->tm_zone));
  s_timezone_abbr[TZ_LEN - 1] = '\0';
  s_timezone_gmtoffset = tz_info->tm_gmtoff;
  s_dst_start = tz_info->dst_start;
  s_dst_end = tz_info->dst_end;
  // Lord Howe Island has a half-hour DST
  if (tz_info->dst_id == DSTID_LORDHOWE)
  {
    s_dst_adjust = SECONDS_PER_HOUR / 2;
  }
  else
  {
    s_dst_adjust = SECONDS_PER_HOUR;
  }
}

// converts time_t to struct tm for localtime and gmtime
struct tm *time_to_tm(const time_t *tim_p, struct tm *res, bool utc_mode)
{
  time_t local_time;
  time_t utc_time = *tim_p;
  if (utc_mode)
  {
    local_time = time_utc_to_local(utc_time);
  }
  else
  {
    local_time = utc_time;
  }
  struct tm *time_info;

  time_info = localtime(&local_time);

  return time_info;
}

struct tm *_localtime_r(const time_t *timep, struct tm *result)
{
  return time_to_tm(timep, result, false);
}


time_t rtc_get_time(void)
{
  uint16_t ms;
  // extern time_t get_current_time(void);
  // time_t seconds = get_current_time();
  time_t seconds = 0;
	// time(&seconds);
  // rtc_get_time_ms(&seconds, &ms);
  return seconds;
}

time_t time_util_get_midnight_of(time_t ts)
{
  struct tm tm;
  _localtime_r(&ts, &tm);
  tm.tm_hour = 0;
  tm.tm_min = 0;
  tm.tm_sec = 0;
  return mktime(&tm);
}

bool time_util_range_spans_day(time_t start, time_t end, time_t start_of_day)
{
  return (start <= start_of_day && end >= (start_of_day + SECONDS_PER_DAY));
}

time_t time_utc_to_local_using_offset(time_t utc_time, int16_t utc_offset_min)
{
  if (utc_offset_min < 0)
  {
    return utc_time - (time_t)ABS(utc_offset_min) * SECONDS_PER_MINUTE;
  }
  else
  {
    return utc_time + (time_t)utc_offset_min * SECONDS_PER_MINUTE;
  }
}

time_t time_local_to_utc_using_offset(time_t local_time, int16_t utc_offset_min)
{
  if (utc_offset_min < 0)
  {
    return local_time + (time_t)ABS(utc_offset_min) * SECONDS_PER_MINUTE;
  }
  else
  {
    return local_time - (time_t)utc_offset_min * SECONDS_PER_MINUTE;
  }
}

time_t time_util_utc_to_local_offset(void)
{
  time_t now = rtc_get_time();
  return (time_utc_to_local(now) - now);
}

// ---------------------------------------------------------------------------------------
DayInWeek time_util_get_day_in_week(time_t utc_sec)
{
  struct tm local_tm;
  _localtime_r(&utc_sec, &local_tm);
  return local_tm.tm_wday;
}

// ---------------------------------------------------------------------------------------
uint16_t time_util_get_day(time_t utc_sec)
{
  // Convert to local seconds
  time_t local_sec = time_utc_to_local(utc_sec);

  // Figure out the day index
  return (local_sec / SECONDS_PER_DAY);
}

// ---------------------------------------------------------------------------------------
int time_util_get_minute_of_day(time_t utc_sec)
{
  struct tm local_tm;
  _localtime_r(&utc_sec, &local_tm);
  return (local_tm.tm_hour * MINUTES_PER_HOUR) + local_tm.tm_min;
}

// ---------------------------------------------------------------------------------------
int time_util_minute_of_day_adjust(int minute, int delta)
{
  minute += delta;
  if (minute < 0)
  {
    minute += MINUTES_PER_DAY;
  }
  else if (minute >= MINUTES_PER_DAY)
  {
    minute -= MINUTES_PER_DAY;
  }
  return minute;
}

// ---------------------------------------------------------------------------------------
time_t time_start_of_today(void)
{
  time_t now = rtc_get_time();
  return time_util_get_midnight_of(now);
}

// ---------------------------------------------------------------------------------------
#define configTICK_RATE_HZ 1000
uint32_t time_get_uptime_seconds(void)
{
  // extern rt_tick_t rt_tick_get(void);
  // uint32_t ticks = rt_tick_get();
  // return ticks / configTICK_RATE_HZ;
  return 0;
}
