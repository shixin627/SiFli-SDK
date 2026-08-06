/*
 * Skai SDK — weather. Backend: bloc_weather (get_weather(0) = current), which
 * is filled by the phone.
 *
 * This domain is the Phase 2 proof: it was added as annotated declarations
 * plus a wrapper, and the declarative renderer picked it up with no renderer
 * edit at all. That is the whole point of the dispatch table.
 *
 * Temperature is also why SKAI_NO_DATA is INT32_MIN rather than "any negative"
 * — -5 C is data, not an error.
 */
#ifndef SKAI_WEATHER_H
#define SKAI_WEATHER_H

#include <stdint.h>
#include "skai/skai_export.h"

/* Current temperature in whole degrees Celsius, rounded. SKAI_NO_DATA until
 * the phone has sent a forecast. */
SKAI_EXPORT("weather.temp", SKAI_T2, SKAI_THREAD_ANY, "%d\xc2\xb0")
int32_t skai_weather_temp(void);

SKAI_EXPORT("weather.temp_min", SKAI_T2, SKAI_THREAD_ANY, "%d\xc2\xb0")
int32_t skai_weather_temp_min(void);

SKAI_EXPORT("weather.temp_max", SKAI_T2, SKAI_THREAD_ANY, "%d\xc2\xb0")
int32_t skai_weather_temp_max(void);

/* Chance of precipitation, 0..100. */
SKAI_EXPORT("weather.rain_pct", SKAI_T2, SKAI_THREAD_ANY, "%d%%")
int32_t skai_weather_rain_pct(void);

/* Condition as a token from a closed set:
 *   "sun" "clear" "cloudy" "rain" "thunder" "snow"
 * Chosen to compose with the icon table — ui.icon("weather." + condition())
 * draws the matching glyph, and a token with no icon yet draws nothing rather
 * than the wrong thing. Empty when the phone has sent no forecast. */
SKAI_EXPORT("weather.condition", SKAI_T2, SKAI_THREAD_ANY, "%s")
int32_t skai_weather_condition(char *out, uint32_t cap);

/* The phone's own wording for the current conditions ("Clear", "Clouds", ...),
 * which is what the built-in screen prints under the temperature. condition()
 * is the normalised token for choosing an icon; this is the human string. */
SKAI_EXPORT("weather.description", SKAI_T2, SKAI_THREAD_ANY, "%s")
int32_t skai_weather_description(char *out, uint32_t cap);

/* True when the stored forecast is not for today — the condition the built-in
 * screen shows "please update on the phone" for. */
SKAI_EXPORT("weather.stale", SKAI_T2, SKAI_THREAD_ANY)
bool skai_weather_stale(void);

/* Ask the phone for a fresh forecast. Rate-limited inside bloc_weather (10 s
 * hard, 30 min soft), so this adds no throttle of its own and an app cannot
 * turn it into a radio drain. */
SKAI_EXPORT("weather.refresh", SKAI_T2, SKAI_THREAD_ANY)
bool skai_weather_refresh(void);

/* The place the forecast is for, as the phone named it. Empty if unknown. */
SKAI_EXPORT("weather.location", SKAI_T2, SKAI_THREAD_ANY, "%s")
int32_t skai_weather_location(char *out, uint32_t cap);

/* ── the hourly forecast ──
 *
 * Slot i is the i-th forecast AFTER now, nearest first: the phone sends
 * current conditions plus three slots roughly 3 hours apart, so i=0 is about
 * three hours out and hour_count() is at most 3. "Now" is not part of this
 * series — that is temp() / condition() above.
 *
 * Display hour_time(i) rather than computing "now + 3(i+1) hours": the phone
 * aligns the slots to wall-clock boundaries, so the gap to the first one
 * depends on what time it is.
 *
 * Every accessor returns no-data for an index outside 0..hour_count()-1. */
SKAI_EXPORT("weather.hour_count", SKAI_T2, SKAI_THREAD_ANY, "%d")
int32_t skai_weather_hour_count(void);

/* Pre-formatted in the user's 12/24-hour setting, e.g. "3 PM" or "15:00".
 * Formatted here rather than handing out an hour number, so the setting stays
 * a firmware decision and every app honours it without trying. */
SKAI_EXPORT("weather.hour_time", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_weather_hour_time(int32_t index, char *out, uint32_t cap);

SKAI_EXPORT("weather.hour_temp", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_weather_hour_temp(int32_t index);

/* Same closed token set as condition(). */
SKAI_EXPORT("weather.hour_cond", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_weather_hour_cond(int32_t index, char *out, uint32_t cap);

/* ── the daily summaries ──
 *
 * The phone sends five, and day 0 is the nearest. Same nearest-first order as
 * the hourly slots, and the same reason: the store holds them backwards and an
 * app should not have to know that.
 *
 * A daily record has a range rather than a reading, so there is no day_temp() —
 * day_min() and day_max() are the pair the built-in app shows. */
SKAI_EXPORT("weather.day_count", SKAI_T2, SKAI_THREAD_ANY, "%d")
int32_t skai_weather_day_count(void);

/* "MM/DD", the same form the built-in daily page prints. */
SKAI_EXPORT("weather.day_date", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_weather_day_date(int32_t index, char *out, uint32_t cap);

SKAI_EXPORT("weather.day_min", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_weather_day_min(int32_t index);

SKAI_EXPORT("weather.day_max", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_weather_day_max(int32_t index);

/* Same closed token set as condition(). */
SKAI_EXPORT("weather.day_cond", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_weather_day_cond(int32_t index, char *out, uint32_t cap);

/* Chance of precipitation for that day, 0..100. Returned whatever the
 * condition is — the built-in page hides it unless the day is a wet one, but
 * that is a presentation choice, and an app with day_cond() can make it. */
SKAI_EXPORT("weather.day_rain", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_weather_day_rain(int32_t index);

#endif /* SKAI_WEATHER_H */
