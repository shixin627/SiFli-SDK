/**
 * @file   skaiapp_engine.h
 * @brief  SkaiApp background runtime: countdown timers + reminder schedules.
 *
 * Always-resident footprint is deliberately tiny (~2 KB static for 8 apps —
 * HCPU SRAM is chronically full): schedules keep numbers only. Human-readable
 * strings (app name, notify/message text) are re-read from the stored package
 * at fire time, on this engine's own thread (never a timer callback, never the
 * LVGL thread). Firing follows the OTA-hint precedent: build a stack
 * notification_t → interact_with_notification() → motor pattern.
 *
 * Reminder "next fire" is STATELESS: recomputed from the RTC wall clock on
 * every evaluation (survives reboots and phone time syncs; nothing persisted
 * except the user's enabled toggle, which is debounced back into the package
 * file). Countdown timers are RAM-only by design — a reboot clears them.
 */
#ifndef SKAIAPP_ENGINE_H
#define SKAIAPP_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "skaiapp_pkg.h"

/* seed filled by skaiapp_pkg_parse(feed_engine=true) */
typedef struct
{
    uint32_t duration_s;
    int8_t   next;      /* chained timer slot, -1 none */
    uint8_t  flags;     /* SKAIAPP_TF_* */
} skaiapp_seed_timer_t;

#define SKAIAPP_TF_AUTOSTART      0x01 /* start when the record loads (install/boot) */
#define SKAIAPP_TF_VIBRATE        0x02
#define SKAIAPP_TF_HAS_NOTIFY     0x04
#define SKAIAPP_TF_AUTOSTART_NEXT 0x08

typedef struct
{
    uint8_t  kind;      /* 0 interval / 1 daily */
    uint8_t  enabled;
    uint8_t  vibrate;
    uint8_t  n_times;
    uint16_t every_min;
    uint16_t win_start; /* minutes-of-day; win_start==win_end==0 → no window */
    uint16_t win_end;
    uint16_t times[SKAIAPP_MAX_DAILY_TIMES]; /* minutes-of-day */
} skaiapp_seed_rem_t;

typedef struct skaiapp_eng_seed
{
    uint8_t n_timers;
    uint8_t n_reminders;
    skaiapp_seed_timer_t t[SKAIAPP_MAX_TIMERS];
    skaiapp_seed_rem_t   r[SKAIAPP_MAX_REMINDERS];
} skaiapp_eng_seed_t;

/* thread + timer bootstrap (INIT_APP_EXPORT'd in skaiapp_engine.c) */
void skaiapp_engine_load(const char *app_id, const skaiapp_eng_seed_t *seed);
void skaiapp_engine_unload(const char *app_id);

/* timer controls (LVGL thread, from rendered buttons) */
void skaiapp_engine_timer_start(const char *app_id, int idx);
void skaiapp_engine_timer_pause(const char *app_id, int idx);
void skaiapp_engine_timer_reset(const char *app_id, int idx);
/* live query for the page refresh tick; false = no such timer */
bool skaiapp_engine_timer_query(const char *app_id, int idx,
                                uint32_t *remaining_ms, bool *running,
                                uint32_t *duration_s);

/* toggle returns the NEW enabled state; next-fire query: 0 = disabled/none,
   else minutes-of-day of the next fire (display as HH:MM) */
bool     skaiapp_engine_reminder_toggle(const char *app_id, int idx);
bool     skaiapp_engine_reminder_enabled(const char *app_id, int idx);
uint16_t skaiapp_engine_reminder_next(const char *app_id, int idx);

#ifdef __cplusplus
}
#endif

#endif /* SKAIAPP_ENGINE_H */
