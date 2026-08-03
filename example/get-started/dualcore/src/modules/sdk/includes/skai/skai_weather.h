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

#endif /* SKAI_WEATHER_H */
