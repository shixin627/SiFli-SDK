/*
 * Skai SDK — health data.
 * Backend: SkaiWatchSys.gPedoData / .heart_rate_bpm / .flag_field.is_wearing,
 * all already mirrored from LCPU by watch_system_client. Reads are plain
 * global loads — no IPC, no blocking.
 *
 * T2 throughout: health data is special-category personal data under GDPR, so
 * every one of these needs per-item install consent (ADR-0019).
 */
#ifndef SKAI_HEALTH_H
#define SKAI_HEALTH_H

#include <stdbool.h>
#include <stdint.h>
#include "skai/skai_export.h"

/* Steps today. SKAI_NO_DATA until the first reading arrives after a boot. */
SKAI_EXPORT("health.steps", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_health_steps(void);

SKAI_EXPORT("health.distance_m", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_health_distance_m(void);

SKAI_EXPORT("health.calories", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_health_calories(void);

/* Last measured heart rate in bpm. SKAI_NO_DATA when there is no reading —
 * this is a cached value, not a trigger: reading it never starts the PPG. */
SKAI_EXPORT("health.heart_rate", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_health_heart_rate(void);

/* Wear detection. Useful for an app to know its readings are meaningless. */
SKAI_EXPORT("health.worn", SKAI_T2, SKAI_THREAD_ANY)
bool skai_health_worn(void);

/* Daily step goal the user set, so apps render progress against the same
 * number the watch does instead of inventing one. */
SKAI_EXPORT("health.step_goal", SKAI_T2, SKAI_THREAD_ANY)
int32_t skai_health_step_goal(void);

#endif /* SKAI_HEALTH_H */
