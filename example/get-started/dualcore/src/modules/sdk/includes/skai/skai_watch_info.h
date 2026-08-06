/*
 * Skai SDK — platform facts. Backend: VERSION_* in watch_global_data.h and
 * board defines. All compile-time constants.
 *
 * Exists so apps branch on capability rather than hardcoding 466x466 — the
 * screen size is the one thing every third-party layout will get wrong.
 */
#ifndef SKAI_WATCH_INFO_H
#define SKAI_WATCH_INFO_H

#include <stdbool.h>
#include <stdint.h>
#include "skai/skai_export.h"

/* "major.minor.revision" or "major.minor.revision-dev". */
SKAI_EXPORT("watchinfo.firmware", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_watchinfo_firmware(char *out, uint32_t cap);

SKAI_EXPORT("watchinfo.model", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_watchinfo_model(char *out, uint32_t cap);

SKAI_EXPORT("watchinfo.screen_width", SKAI_T1, SKAI_THREAD_ANY)
uint32_t skai_watchinfo_screen_width(void);

SKAI_EXPORT("watchinfo.screen_height", SKAI_T1, SKAI_THREAD_ANY)
uint32_t skai_watchinfo_screen_height(void);

/* True for a round display. Third-party layouts need this to avoid drawing
 * into the corners, which on a 466 circle are simply not there. */
SKAI_EXPORT("watchinfo.screen_round", SKAI_T1, SKAI_THREAD_ANY)
bool skai_watchinfo_screen_round(void);

#endif /* SKAI_WATCH_INFO_H */
