/*
 * Skai SDK — timers. Backend: rt_timer (soft timers).
 *
 * DELIBERATELY MOSTLY UNEXPORTED. A timer's whole point is a callback, and a
 * C function pointer has no projection into the scripting API — the external
 * shape needs the Phase 3 event loop (a JS function handle the runtime owns,
 * cancelled when the app is torn down or exceeds its quota). Annotating a
 * callback-taking function now would either force a fake type into the
 * dispatch vocabulary or produce a binding that cannot be sandboxed.
 *
 * So: the callback API below is internal C only, and the one thing that does
 * project cleanly — reading the monotonic clock — is exported. Apps can poll
 * against it today; scheduling arrives with the event loop.
 */
#ifndef SKAI_TIMER_H
#define SKAI_TIMER_H

#include <stdbool.h>
#include <stdint.h>
#include "skai/skai_export.h"

/* Milliseconds since boot. Wraps after ~49 days; compare differences, never
 * absolute values. Distinct from skai_time_now(), which is wall time and can
 * jump when the phone syncs the clock. */
SKAI_EXPORT("timer.uptime_ms", SKAI_T1, SKAI_THREAD_ANY)
uint32_t skai_timer_uptime_ms(void);

/* --- internal C only, not exported --- */

/* Concurrent timers available across all callers. Part of the contract, not
 * an implementation detail — a caller needs to know the ceiling exists. */
#define SKAI_TIMER_SLOTS 8

typedef void (*skai_timer_cb_t)(void *arg);

/* Returns a handle, or 0 on failure. `period_ms` under one tick is rounded up
 * to one tick rather than rejected. Callbacks run on the RT-Thread soft timer
 * thread, so they must not block and must not touch LVGL directly. */
uint32_t skai_timer_create(uint32_t period_ms, bool repeating,
                           skai_timer_cb_t cb, void *arg);

/* Both are safe to call on an already-stopped or unknown handle. */
bool skai_timer_stop(uint32_t handle);
bool skai_timer_destroy(uint32_t handle);

#endif /* SKAI_TIMER_H */
