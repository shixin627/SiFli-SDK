/**
 * @file   skaiapp_store.h
 * @brief  SkaiApp package persistence (/skaiapp/<id>.json on the FAT FS) plus a
 *         tiny resident meta table (id/name/icon) for the launcher list.
 *
 * Threading: install/remove arrive on the BLE parse thread, the launcher reads
 * on the LVGL thread, the engine reads on its own thread → every entry point
 * takes the store mutex. LVGL objects are NEVER touched here; GUI change
 * propagation is a lock-free generation counter the host app polls.
 */
#ifndef SKAIAPP_STORE_H
#define SKAIAPP_STORE_H

#include "skaiapp_pkg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Scan /skaiapp at boot, rebuild meta + engine records. Safe to call twice. */
void skaiapp_store_init(void);

int  skaiapp_store_count(void);
/* Copy meta of slot `idx` (launcher order = scan order). false = no such slot. */
bool skaiapp_store_meta(int idx, char id[SKAIAPP_ID_MAX],
                        char name[SKAIAPP_NAME_MAX], uint8_t *icon);
bool skaiapp_store_exists(const char *id);

/* Validate + persist one raw package; feeds the engine on success.
   Returns a SKAIAPP_ACK_* code. out_id (optional) receives the package id. */
int  skaiapp_store_install(const uint8_t *json, uint32_t len,
                           char out_id[SKAIAPP_ID_MAX]);
/* Remove package file + meta + engine record. Returns SKAIAPP_ACK_*. */
int  skaiapp_store_remove(const char *id);

/* Load raw JSON into an rt_malloc'd buffer (caller rt_free's). 0 = ok. */
int  skaiapp_store_load(const char *id, uint8_t **buf, uint32_t *len);

/* Debounced persist target for the reminder on/off toggle (engine thread only —
   one small rewrite, mind the flash-XIP-stall rule). 0 = ok. */
int  skaiapp_store_rewrite_reminder_enabled(const char *id,
                                            const uint8_t enabled[SKAIAPP_MAX_REMINDERS],
                                            uint8_t n);

/* Bumped on every install/remove; the host app polls it from an lv_timer. */
uint32_t skaiapp_store_generation(void);

/* Most recent successfully installed id ("" if none). Returns true only while
   the install is FRESH (≤30 s) — lets the host jump straight to a just-pushed
   app when the phone follows with APP_RUN, without hijacking normal opens. */
bool skaiapp_store_last_installed(char out_id[SKAIAPP_ID_MAX]);

#ifdef __cplusplus
}
#endif

#endif /* SKAIAPP_STORE_H */
