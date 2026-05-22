/**
 * @file    disp_refr_governor.h
 * @brief   Dynamic display refresh-rate governor (HCPU / LVGL v8).
 *
 * The CO5300 AMOLED is a command-mode panel: it self-refreshes from its own
 * GRAM, so once the static clock face is drawn the image persists with no
 * host activity. This lets us run LVGL's display refresh timer at 1 Hz on the
 * idle home/clock face and 60 Hz otherwise, saving HCPU wakeups.
 *
 * The governor is split into two layers:
 *   - Layer 1 (display refr timer): retimes lv_disp_get_default()->refr_timer
 *     between 16 ms (60 Hz) and 1000 ms (1 Hz). This is the rendering-side
 *     saving and is the core of the feature.
 *   - Layer 2 (indev read throttle): optionally slows / pauses the touch
 *     indev read timer in the 1 Hz state, relying on the CST816 touch
 *     INTERRUPT to wake & resume polling. This is a deeper power win but is
 *     guarded behind its own sub-flag (default OFF) so layer 1 can be
 *     validated first.
 *
 * The WHOLE governor is behind a compile guard (DISP_REFR_GOVERNOR_ENABLE)
 * and a runtime bool that defaults OFF, so default behavior is unchanged
 * fixed-60 Hz unless explicitly enabled (via MSH or disp_gov_set_enabled()).
 */
#ifndef __DISP_REFR_GOVERNOR_H__
#define __DISP_REFR_GOVERNOR_H__

#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>

/* Master compile guard. Define to 0 in a board config to strip the governor
 * entirely (all hook calls become no-ops via the stubs below). */
#ifndef DISP_REFR_GOVERNOR_ENABLE
    #define DISP_REFR_GOVERNOR_ENABLE 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if DISP_REFR_GOVERNOR_ENABLE

/* ---- lifecycle ---- */

/** Bind the governor to the default display's refr_timer. Safe to call once
 *  LVGL + the default display are up (called from the GUI task). */
void disp_gov_init(void);

/* ---- runtime master switch (default OFF) ---- */

void disp_gov_set_enabled(bool en);
bool disp_gov_is_enabled(void);

/* ---- layer-2 sub-switch (indev read throttle, default OFF) ---- */

void disp_gov_set_indev_throttle(bool en);
bool disp_gov_indev_throttle_enabled(void);

/* ---- layer-3 sub-switch (DVFS: couple HCPU run mode to refresh, default OFF) ----
 * Active/60 Hz -> NORMAL_SPEED (144 MHz); idle-home/1 Hz -> MEDIUM_SPEED
 * (48 MHz). Never requests HIGH_SPEED (240) — no sustained scenario needs it.
 * No-op when RT_USING_PM is absent. */
void disp_gov_set_dvfs(bool en);
bool disp_gov_dvfs_enabled(void);

/* ---- event hooks (called from the various input / power paths) ---- */

/** First "operate" input of a wake session: a touch PRESS while already
 *  awake, OR a navigating/animating gesture. Latches session_active=true,
 *  forces 60 Hz, and proactively calls lv_refr_now() so the first transition
 *  frame is not delayed by the (up to ~1 s away) next refr tick. */
void disp_gov_notify_operate(void);

/** Screen turned ON / woke (raise-wrist or wake-tap). Resets the session
 *  latch and arms a short debounce so the wake-causing touch does NOT count
 *  as an operate input — a wake glance stays at 1 Hz. */
void disp_gov_notify_screen_on(void);

/** Screen turned OFF. Resets the session latch. */
void disp_gov_notify_screen_off(void);

/** Touch indev read hook: returns true if the governor wants this touch
 *  sample suppressed from being treated as an operate input (i.e. inside the
 *  post-wake debounce window). Layer-1 callers can ignore the return value;
 *  it exists so the touch read path can both (a) report a real press as an
 *  operate input and (b) honor the wake debounce in one call. */
bool disp_gov_notify_touch_press(void);

/* ---- forced state (testing) ---- */

typedef enum
{
    DISP_GOV_FORCE_NONE = 0, /* governor decides */
    DISP_GOV_FORCE_60HZ,     /* pin 60 Hz */
    DISP_GOV_FORCE_1HZ,      /* pin 1 Hz  */
} disp_gov_force_t;

void disp_gov_set_force(disp_gov_force_t force);

#else /* !DISP_REFR_GOVERNOR_ENABLE — compile-stripped no-ops */

static inline void disp_gov_init(void) {}
static inline void disp_gov_set_enabled(bool en) { (void)en; }
static inline bool disp_gov_is_enabled(void) { return false; }
static inline void disp_gov_set_indev_throttle(bool en) { (void)en; }
static inline bool disp_gov_indev_throttle_enabled(void) { return false; }
static inline void disp_gov_set_dvfs(bool en) { (void)en; }
static inline bool disp_gov_dvfs_enabled(void) { return false; }
static inline void disp_gov_notify_operate(void) {}
static inline void disp_gov_notify_screen_on(void) {}
static inline void disp_gov_notify_screen_off(void) {}
static inline bool disp_gov_notify_touch_press(void) { return false; }

#endif /* DISP_REFR_GOVERNOR_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __DISP_REFR_GOVERNOR_H__ */
