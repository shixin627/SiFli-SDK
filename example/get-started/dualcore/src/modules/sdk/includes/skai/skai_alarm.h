/*
 * Skai SDK — alarms that fire even when the app is closed and the watch is
 * asleep (cooking timers, "take the laundry out", a pomodoro).
 *
 * A JS app's own setTimeout/setInterval stop when the watch sleeps: the RTOS
 * tick stops with the core. These do not — each one is a hardware RTC alarm,
 * the same mechanism the built-in alarm clock rings with, so the watch wakes,
 * vibrates and shows a notification carrying the app's text at the right
 * second whatever the app or the screen is doing.
 */
#ifndef SKAI_ALARM_H
#define SKAI_ALARM_H

#include <stdbool.h>
#include <stdint.h>
#include "skai/skai_export.h"

/* Alarms the whole watch holds at once, across all apps. */
#define SKAI_ALARM_SLOTS 8

/* Longest text shown when an alarm fires (bytes, UTF-8). */
#define SKAI_ALARM_TEXT_MAX 64

/* Fire in `seconds` (1..86400): the watch wakes, vibrates and shows `text` as a
 * notification, even if this app is closed or the screen is off. Returns an
 * alarm id (>0) to cancel or query it, or 0 when all slots are in use. An app
 * can only see and cancel its own alarms. When it fires, a running app's
 * skai.on_change("alarm.remaining", fn) handler runs. */
SKAI_EXPORT("alarm.set", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_alarm_set(const char *text, int32_t seconds);

/* Stop an alarm before it fires (or forget one that already fired). */
SKAI_EXPORT("alarm.cancel", SKAI_T1, SKAI_THREAD_ANY)
bool skai_alarm_cancel(int32_t id);

/* Seconds until the alarm fires; 0 once it has fired; null for an unknown id.
 * Read it from a setInterval to draw a countdown — the number stays right
 * across sleep because it comes from the clock, not from counting ticks. */
SKAI_EXPORT("alarm.remaining", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_alarm_remaining(int32_t id);

/* ── host side, not exported ── */
/* True once per batch of fired alarms; the JS host polls it on the LVGL thread
 * and turns it into skai_js_notify_change("alarm"). */
bool skai_alarm_take_fired_event(void);

#endif /* SKAI_ALARM_H */
