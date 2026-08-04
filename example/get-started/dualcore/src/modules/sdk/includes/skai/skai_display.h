/*
 * Skai SDK — screen control.
 *
 * One capability, and the interesting part is what the app is NOT trusted with:
 * it can raise the brightness, and it cannot lower it back. The firmware
 * restores the user's setting when the app leaves the foreground, so "set it to
 * maximum and never put it back" is not something an app can express — not by
 * malice, not by crashing, and not by forgetting.
 *
 * That is also why there is no display.brightness() to read the current value:
 * an app that cannot be responsible for restoring has no reason to know.
 */
#ifndef SKAI_DISPLAY_H
#define SKAI_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "skai_export.h"

/* Raise the screen to `percent` (3..100) for as long as this app is in front.
 * T2: the screen is shared, and a bright panel is the single biggest battery
 * cost a foreground app can impose.
 *
 * Out-of-range values are refused rather than clamped — a clamp would hide a
 * unit mix-up (0..255 is the other plausible scale) until someone saw the
 * watch. Returns false if refused. */
SKAI_EXPORT("display.set_brightness", SKAI_T2, SKAI_THREAD_LVGL)
bool skai_display_set_brightness(int32_t percent);

/* Hold the screen awake for as long as this app is in front. The idle timer
 * would otherwise dim and blank the panel mid-use, which is the difference
 * between a torch and a torch that turns itself off.
 *
 * Released by the firmware on pause or stop, exactly like the brightness — an
 * app cannot leave the screen pinned on by crashing or by forgetting. */
SKAI_EXPORT("display.set_power_save", SKAI_T2, SKAI_THREAD_LVGL)
bool skai_display_set_power_save(int32_t enabled);

/* Host-side, not a capability: put the user's brightness back. Called by the JS
 * app host when the app pauses or stops. Idempotent, and a no-op if the app
 * never touched the brightness, and releases any wake-hold. */
void skai_display_restore(void);

/* Host-side: re-apply what the app asked for, on resume. The C apps set
 * brightness on EVERY resume; a JS app's body runs once, so without this a
 * backgrounded app comes back dark and can never fix it. */
void skai_display_reapply(void);

#endif /* SKAI_DISPLAY_H */
