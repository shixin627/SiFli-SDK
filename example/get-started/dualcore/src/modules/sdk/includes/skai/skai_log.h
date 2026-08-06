/*
 * Skai SDK — logging. Backend: ulog.
 *
 * Phase 3 note: for external apps this becomes the app-scoped log stream
 * (tagged by keyid, gated behind a user-enabled debug session, auto-expiring)
 * described in ADR-0019. System ulog stays off in release builds — it carries
 * notification text and health values. Same call, different sink.
 */
#ifndef SKAI_LOG_H
#define SKAI_LOG_H

#include <stdint.h>
#include "skai/skai_export.h"

#define SKAI_LOG_DEBUG  0
#define SKAI_LOG_INFO   1
#define SKAI_LOG_WARN   2
#define SKAI_LOG_ERROR  3

/* Unknown levels are logged at INFO rather than dropped — losing a message
 * because a caller passed 7 is worse than logging it at the wrong level. */
SKAI_EXPORT("log.write", SKAI_T1, SKAI_THREAD_ANY)
void skai_log_write(uint32_t level, const char *msg);

#endif /* SKAI_LOG_H */
