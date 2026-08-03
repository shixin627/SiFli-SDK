/*
 * Skai SDK — key/value persistence. Backend: share_prefs (FlashDB KVDB on the
 * "prefdb" partition), under the "skai" prefix.
 *
 * WHY THE HANDLE IS CACHED FOREVER: share_prefs_open() calls fdb_kvdb_init()
 * every time — there is no refcount or cache in middleware/share_prefs. On the
 * NAND prefdb partition a first init scans the whole region and can exceed
 * 8 s, which is longer than WDT1 (see bloc_notification.h). An open/set/close
 * per call would therefore reset the watch. We open lazily once and never
 * close.
 *
 * Values are small settings, not a filesystem: the KVDB region is 16 KB and is
 * shared with the rest of the watch's preferences. Anything bulky belongs in
 * the filesystem.
 */
#ifndef SKAI_PERSIST_H
#define SKAI_PERSIST_H

#include <stdbool.h>
#include <stdint.h>
#include "skai/skai_export.h"

/* Longest key accepted, excluding NUL. Keys are stored as "skai.<key>" and
 * share_prefs caps the combined name at 32. */
#define SKAI_PERSIST_KEY_MAX 24

/* SKAI_THREAD_APP, not ANY: these block on flash. Calling them from the LVGL
 * thread stalls rendering, and a janky watch face is the kind of bug that
 * gets blamed on the firmware rather than the app that caused it. */

SKAI_EXPORT("persist.get_int", SKAI_T1, SKAI_THREAD_APP)
int32_t skai_persist_get_int(const char *key, int32_t fallback);

SKAI_EXPORT("persist.set_int", SKAI_T1, SKAI_THREAD_APP)
bool skai_persist_set_int(const char *key, int32_t value);

/* Returns bytes written excluding NUL, or negative if the key is absent or
 * the store is unavailable. Always NUL-terminates when cap > 0. */
SKAI_EXPORT("persist.get_str", SKAI_T1, SKAI_THREAD_APP)
int32_t skai_persist_get_str(const char *key, char *out, uint32_t cap);

SKAI_EXPORT("persist.set_str", SKAI_T1, SKAI_THREAD_APP)
bool skai_persist_set_str(const char *key, const char *value);

/* True if the key is gone afterwards, including when it was never there. */
SKAI_EXPORT("persist.remove", SKAI_T1, SKAI_THREAD_APP)
bool skai_persist_remove(const char *key);

#endif /* SKAI_PERSIST_H */
