/*
 * Skai SDK — wall time. Phase 0: contract only, no implementation.
 * Backend (Phase 1): RTC + watch_global_data.time.
 */
#ifndef SKAI_TIME_H
#define SKAI_TIME_H

#include <stdint.h>
#include "skai/skai_export.h"

/* Seconds since the Unix epoch, local time already applied. SKAI_NO_DATA
 * when the RTC is not yet valid after a cold boot. */
SKAI_EXPORT("time.now", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_time_now(void);

/* No utc_offset here on purpose: the RTC is set to LOCAL time by the phone
 * (app_clock_main calls localtime() straight on it), so the watch does not
 * know its own offset. An API that returned 0 would be lying. If UTC is ever
 * needed, the phone has to start sending the offset. */

/* strftime-subset format into `out`. Returns bytes written excluding NUL, or
 * negative on failure. Always NUL-terminates when cap > 0. */
SKAI_EXPORT("time.format", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_time_format(const char *fmt, char *out, uint32_t cap);

/* Current time the way the user chose to see it — honours the 12/24-hour
 * setting. Exists as its own capability because declarative binds cannot pass
 * a format string, and "the clock" is the single most common bind there is. */
SKAI_EXPORT("time.hhmm", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_time_hhmm(char *out, uint32_t cap);

/* Current date as MM/DD. */
SKAI_EXPORT("time.date_md", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_time_date_md(char *out, uint32_t cap);

#endif /* SKAI_TIME_H */
