/*
 * Skai SDK — haptics. Phase 0: contract only, no implementation.
 * Backend (Phase 1): bloc_motor pattern playback (LRA).
 *
 * TIER GAP: ADR-0019's tier table does not place haptics. T1 assumed here
 * because Pebble grants vibes to every app and the abuse ceiling is
 * annoyance, not data. Confirm before the first external app ships —
 * tightening a tier later is a breaking change.
 */
#ifndef SKAI_HAPTIC_H
#define SKAI_HAPTIC_H

#include <stdbool.h>
#include <stdint.h>
#include "skai/skai_export.h"

/* Play a built-in pattern by id. Returns false if the id is unknown or the
 * motor is busy; never queues, so a spamming app cannot build a backlog. */
SKAI_EXPORT("haptic.vibrate", SKAI_T1, SKAI_THREAD_ANY)
bool skai_haptic_vibrate(uint32_t pattern_id);

SKAI_EXPORT("haptic.stop", SKAI_T1, SKAI_THREAD_ANY)
bool skai_haptic_stop(void);

#endif /* SKAI_HAPTIC_H */
