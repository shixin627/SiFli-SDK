/**
 * @file    disp_refr_governor.c
 * @brief   Dynamic display refresh-rate governor (HCPU / LVGL v8).
 *
 * See disp_refr_governor.h for design rationale. Summary of the state
 * machine (implemented exactly as specified):
 *
 *   Refresh = 60 Hz by default. Drop to 1 Hz ONLY when ALL of:
 *     - is_at_home() is true, AND
 *     - session_active == false, AND
 *     - lv_anim_count_running() == 0.
 *
 *   session_active latch:
 *     - SET true on the first "operate" input of a wake session: a touch
 *       PRESS while awake, OR a navigating/animating gesture.
 *     - RESET false on screen-OFF and on screen-ON / wake.
 *     - While true, stay 60 Hz until screen-off (no touch-idle criterion).
 *
 *   Proactive bump: an operate input forces 60 Hz AND calls lv_refr_now()
 *   immediately, so the first transition frame is not delayed by the next
 *   (possibly ~1 s away) refr tick.
 *
 *   Wake-to-home stays 1 Hz: on screen-on we land on home with
 *   session_active=false and debounce the wake-causing touch for a short
 *   window so a wake-glance does not latch.
 *
 *   A short (~300 ms) one-shot re-check timer decides when to actually drop
 *   to 1 Hz after activity settles (so brief anims / banners on home run at
 *   60 Hz and then fall back, without latching).
 *
 * EVERYTHING here is behind DISP_REFR_GOVERNOR_ENABLE (compile guard) and a
 * runtime bool that defaults OFF, so the stock fixed-60 Hz behavior is
 * unchanged unless explicitly enabled.
 */

#include "disp_refr_governor.h"

#if DISP_REFR_GOVERNOR_ENABLE

#include "lvgl.h"
#include <rtthread.h> /* rt_thread_self — [GOV-DIAG] logs tag the calling thread */

#define DBG_TAG "disp.gov"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef RT_USING_PM
#include <drivers/pm.h>   /* rt_pm_run_enter / PM_RUN_MODE_* (layer-3 DVFS) */
#endif

/* is_at_home() lives in app_mainmenu.c; declared here to avoid pulling the
 * whole app_mainmenu.h surface into this self-contained module. */
extern bool is_at_home(void);

/* Touch indev accessor from middleware/lvgl/lv_drivers/lv_touch.c. May be
 * absent under some configs; we null-check before use. */
extern lv_indev_t *touch_get_indev_handler(void);

/* ---- tunables ---- */

#define GOV_PERIOD_60HZ_MS 16U   /* fast: 60 Hz display refresh           */
#define GOV_PERIOD_1HZ_MS 1000U  /* slow: 1 Hz on idle clock face         */
#define GOV_RECHECK_MS 300U      /* settle delay before dropping to 1 Hz  */
#define GOV_MONITOR_MS 300U      /* periodic anim-aware re-evaluation     */
#define GOV_WAKE_DEBOUNCE_MS 250U /* ignore touch as operate after wake    */
#define GOV_INDEV_SLOW_MS 1000U  /* layer-2: slowed indev poll in 1 Hz    */
#define GOV_FAST_HOLD_MS 300U    /* hold 60 Hz this long after wake/activity */

/* layer-3 DVFS run modes coupled to the refresh state. 144 MHz is enough for
 * 60 Hz partial-update GUI + the concurrent gesture-recognition load (display
 * flush is QSPI-bandwidth-bound, not CPU-bound); idle 1 Hz home drops to
 * 48 MHz. We deliberately never request HIGH_SPEED (240) here. */
#ifdef RT_USING_PM
#define GOV_RUN_MODE_FAST PM_RUN_MODE_NORMAL_SPEED /* 144 MHz */
#define GOV_RUN_MODE_SLOW PM_RUN_MODE_MEDIUM_SPEED /* 48 MHz  */
#endif

/* ---- runtime state ---- */

static bool s_enabled = true;           /* master switch: ON for HW bring-up (was OFF) */
static bool s_indev_throttle = false;   /* layer-2 sub-switch, default OFF */
static bool s_dvfs = true;              /* layer-3 DVFS: ON for HW bring-up (was OFF)   */
static bool s_inited = false;

static bool s_session_active = false;   /* the latch                       */
static bool s_is_fast = true;           /* current applied state (60 Hz)   */
static disp_gov_force_t s_force = DISP_GOV_FORCE_NONE;

/* Tick deadline: while lv_tick_get() < this, stay at 60 Hz regardless of idle.
   Set on every wake / operate to GOV_FAST_HOLD_MS ahead, giving a fresh wake or
   interaction a smooth 60 Hz window before the drop to 1 Hz. */
static uint32_t s_fast_until = 0;

static uint32_t s_wake_debounce_until = 0; /* tick deadline (0 = inactive) */

static lv_timer_t *s_recheck_timer = NULL; /* one-shot settle re-check     */
static lv_timer_t *s_monitor_timer = NULL; /* periodic anim-aware re-check */
static uint32_t s_indev_default_period = 0; /* saved indev read period     */

/* ---- helpers ---- */

static lv_timer_t *gov_refr_timer(void)
{
    lv_disp_t *d = lv_disp_get_default();
    return d ? d->refr_timer : NULL;
}

static lv_indev_t *gov_indev(void)
{
    return touch_get_indev_handler();
}

/* Layer-2: slow / restore the touch indev read timer. Only acts when the
 * sub-switch is on. Slowing relies on the CST816 INT to wake HCPU; the
 * driver's tpread thread is interrupt-driven, but LVGL still needs to call
 * input_read to pick up the buffered event, so we slow rather than fully
 * pause to guarantee responsiveness even if an edge is ever missed. */
static void gov_indev_set_slow(bool slow)
{
    if (!s_indev_throttle)
        return;

    lv_indev_t *indev = gov_indev();
    if (!indev || !indev->driver || !indev->driver->read_timer)
        return;

    lv_timer_t *rt = indev->driver->read_timer;

    if (s_indev_default_period == 0)
        s_indev_default_period = rt->period; /* capture stock period once */

    if (slow)
    {
        lv_timer_set_period(rt, GOV_INDEV_SLOW_MS);
    }
    else
    {
        uint32_t restore =
            s_indev_default_period ? s_indev_default_period : LV_INDEV_DEF_READ_PERIOD;
        lv_timer_set_period(rt, restore);
        /* reset so a pending long wait doesn't delay the first fast read */
        lv_timer_reset(rt);
    }
}

/* Layer-3 DVFS: couple HCPU run mode to the refresh state. Only acts when the
 * sub-switch is on and RT_USING_PM is present. An upclock (fast) applies
 * immediately inside rt_pm_run_enter; a downclock (slow) is deferred by the PM
 * framework until the system next idles — exactly what we want. Safe no-op if
 * PM isn't initialized (rt_pm_run_enter returns -RT_EIO). */
static void gov_set_run_mode(bool fast)
{
#ifdef RT_USING_PM
    if (!s_dvfs)
        return;
    rt_pm_run_enter(fast ? GOV_RUN_MODE_FAST : GOV_RUN_MODE_SLOW);
#else
    LV_UNUSED(fast);
#endif
}

/* Apply a refresh state to the LVGL refr_timer (and layer-2 indev + layer-3
 * DVFS). Ordering matters for DVFS: when going fast we upclock BEFORE the
 * first 60 Hz render; when going slow we downclock AFTER dropping to 1 Hz
 * (the PM framework defers the downclock to idle anyway). */
static void gov_apply(bool fast)
{
    /* No-op when already in the requested state. This is essential now that
     * the periodic monitor (gov_monitor_cb) calls gov_evaluate() every
     * GOV_MONITOR_MS: without this guard we'd re-set the refr_timer period,
     * lv_timer_reset() it (jittering the 60 Hz cadence), and re-enter the PM
     * run mode on every tick. */
    if (fast == s_is_fast)
    {
        /* [GOV-DIAG] Stuck-state probe: the no-op guard trusts s_is_fast, but
           if a cross-thread race tore an earlier apply, the actual refr period
           can disagree with the flag — and every later request gets swallowed
           right here. Log it with the calling thread when it happens. */
        lv_timer_t *chk = gov_refr_timer();
        if (chk && ((chk->period <= GOV_PERIOD_60HZ_MS) != fast))
            LOG_W("[GOV-DIAG] no-op swallow: want=%s flag=%s period=%ums thr=%s",
                  fast ? "60Hz" : "1Hz", s_is_fast ? "60Hz" : "1Hz",
                  (unsigned)chk->period, rt_thread_self()->name);
        return;
    }

    lv_timer_t *rt = gov_refr_timer();
    if (!rt)
        return;

    if (fast)
        gov_set_run_mode(true);

    lv_timer_set_period(rt, fast ? GOV_PERIOD_60HZ_MS : GOV_PERIOD_1HZ_MS);
    if (fast)
    {
        /* make sure the next tick is soon, not up to 1 s away */
        lv_timer_reset(rt);
    }

    gov_indev_set_slow(!fast);

    if (!fast)
        gov_set_run_mode(false);

    /* [GOV-DIAG] thread tag: this is expected to be the LVGL thread only —
       any other name here is the cross-thread apply the diagnosis hunts. */
    LOG_I("refresh -> %s (thr=%s)", fast ? "60Hz" : "1Hz",
          rt_thread_self()->name);
    s_is_fast = fast;
}

/* Should we be at the slow (1 Hz) state right now? */
static bool gov_should_be_slow(void)
{
    if (s_force == DISP_GOV_FORCE_60HZ)
        return false;
    if (s_force == DISP_GOV_FORCE_1HZ)
        return true;

    /* Hold 60 Hz for GOV_FAST_HOLD_MS after the last wake / activity, so a
       fresh wake stays smooth before we drop to 1 Hz. */
    if ((int32_t)(s_fast_until - lv_tick_get()) > 0)
        return false;

    /* All three conditions must hold to drop to 1 Hz. */
    if (s_session_active)
        return false;
    if (!is_at_home())
        return false;
    if (lv_anim_count_running() != 0)
        return false;

    return true;
}

/* Re-evaluate and apply the proper state. */
static void gov_evaluate(void)
{
    if (!s_enabled || !s_inited)
        return;

    bool slow = gov_should_be_slow();
    gov_apply(!slow);
}

/* One-shot settle re-check: fired ~300 ms after activity, decides whether to
 * actually drop to 1 Hz now that things have hopefully settled. */
static void gov_recheck_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    s_recheck_timer = NULL; /* one-shot auto-deletes itself below */
    gov_evaluate();
}

static void gov_schedule_recheck(void)
{
    if (!s_enabled || !s_inited)
        return;
    if (s_recheck_timer)
    {
        lv_timer_reset(s_recheck_timer);
        return;
    }
    s_recheck_timer = lv_timer_create(gov_recheck_cb, GOV_RECHECK_MS, NULL);
    if (s_recheck_timer)
        lv_timer_set_repeat_count(s_recheck_timer, 1); /* one-shot */
}

/* Periodic anim-aware monitor. The one-shot recheck only fires once after a
 * wake; it cannot catch an animation that STARTS later while we're already at
 * 1 Hz (e.g. a notification/media marquee scrolling on the idle clock face, a
 * blinking colon, a fade). This timer re-evaluates every GOV_MONITOR_MS so any
 * running lv_anim keeps us at 60 Hz, and we only settle to 1 Hz once nothing
 * is animating. Cheap: gov_apply() is a no-op when the state is unchanged.
 * Runs only while enabled; LVGL stops servicing lv_timers during sleep, so it
 * adds no wakeups in the sleep state. */
static void gov_monitor_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    gov_evaluate();

    /* [GOV-DIAG] Post-evaluate probes (LVGL thread, so reading the anim list
       is safe here). Two stuck signatures, both meaning the marquee would
       visibly step once per second:
         1. flag/period mismatch — a torn cross-thread gov_apply left
            s_is_fast out of sync, so evaluate's request was swallowed;
         2. still slow with anims running — should be impossible right after
            gov_evaluate() unless the upshift was lost.
       Rate-limited: first hit logs immediately, then every 20th (~6 s).
       A ~6 s heartbeat also proves this monitor timer itself still runs. */
    static uint32_t diag_ticks = 0;
    static uint32_t diag_stuck = 0;
    lv_timer_t *rt = gov_refr_timer();
    bool actual_fast = rt && (rt->period <= GOV_PERIOD_60HZ_MS);
    uint16_t anims = lv_anim_count_running();

    if (rt && actual_fast != s_is_fast)
    {
        if ((++diag_stuck % 20u) == 1u)
            LOG_W("[GOV-DIAG] flag/period mismatch: flag=%s period=%ums anim=%u n=%u",
                  s_is_fast ? "60Hz" : "1Hz", (unsigned)rt->period,
                  (unsigned)anims, (unsigned)diag_stuck);
    }
    else if (!s_is_fast && anims != 0)
    {
        if ((++diag_stuck % 20u) == 1u)
            LOG_W("[GOV-DIAG] stuck slow with anim=%u period=%ums n=%u",
                  (unsigned)anims, rt ? (unsigned)rt->period : 0u,
                  (unsigned)diag_stuck);
    }
    else
    {
        diag_stuck = 0;
    }

    if ((++diag_ticks % 20u) == 0u)
        LOG_I("[GOV-DIAG] monitor alive: %s period=%ums anim=%u",
              s_is_fast ? "60Hz" : "1Hz",
              rt ? (unsigned)rt->period : 0u, (unsigned)anims);
}

static void gov_start_monitor(void)
{
    if (s_monitor_timer)
        return;
    s_monitor_timer = lv_timer_create(gov_monitor_cb, GOV_MONITOR_MS, NULL);
}

static void gov_stop_monitor(void)
{
    if (s_monitor_timer)
    {
        lv_timer_del(s_monitor_timer);
        s_monitor_timer = NULL;
    }
}

/* ---- public API ---- */

void disp_gov_init(void)
{
    if (s_inited)
        return;

    lv_timer_t *rt = gov_refr_timer();
    if (!rt)
    {
        LOG_W("no default display refr_timer yet; init deferred");
        return;
    }

    s_inited = true;
    s_session_active = false;
    s_is_fast = true;
    s_wake_debounce_until = 0;

    /* Default OFF: do NOT touch the timer period unless enabled. The stock
     * 60 Hz behavior is preserved. */
    LOG_I("governor init (enabled=%d, indev_throttle=%d)",
          s_enabled, s_indev_throttle);

    if (s_enabled)
    {
        gov_evaluate();
        gov_start_monitor();
    }
}

void disp_gov_set_enabled(bool en)
{
    s_enabled = en;
    if (!s_inited)
        disp_gov_init();

    if (en)
    {
        gov_evaluate();
        gov_start_monitor();
    }
    else
    {
        gov_stop_monitor();
        /* Restore stock fixed-60 Hz and any throttled indev. */
        bool saved_throttle = s_indev_throttle;
        s_indev_throttle = true;       /* force indev restore path */
        gov_indev_set_slow(false);
        s_indev_throttle = saved_throttle;

        lv_timer_t *rt = gov_refr_timer();
        if (rt)
            lv_timer_set_period(rt, GOV_PERIOD_60HZ_MS);
        s_is_fast = true;
#ifdef RT_USING_PM
        /* Hand the clock back to stock (HIGH/240) so disabling the governor
         * fully reverts DVFS too. */
        if (s_dvfs)
            rt_pm_run_enter(PM_RUN_MODE_HIGH_SPEED);
#endif
    }
    LOG_I("governor %s", en ? "ENABLED" : "DISABLED");
}

bool disp_gov_is_enabled(void)
{
    return s_enabled;
}

void disp_gov_set_indev_throttle(bool en)
{
    if (s_indev_throttle == en)
        return;
    s_indev_throttle = en;
    if (!en)
    {
        /* restore stock indev period immediately */
        s_indev_throttle = true;
        gov_indev_set_slow(false);
        s_indev_throttle = false;
    }
    else
    {
        gov_evaluate();
    }
    LOG_I("indev throttle %s", en ? "ON" : "OFF");
}

bool disp_gov_indev_throttle_enabled(void)
{
    return s_indev_throttle;
}

void disp_gov_set_dvfs(bool en)
{
    if (s_dvfs == en)
        return;
    s_dvfs = en;
#ifdef RT_USING_PM
    if (!en)
    {
        /* restore stock clock (HIGH/240) immediately */
        rt_pm_run_enter(PM_RUN_MODE_HIGH_SPEED);
    }
    else if (s_enabled && s_inited)
    {
        /* apply the run mode matching the current refresh state now */
        gov_set_run_mode(s_is_fast);
    }
#endif
    LOG_I("dvfs %s", en ? "ON" : "OFF");
}

bool disp_gov_dvfs_enabled(void)
{
    return s_dvfs;
}

void disp_gov_notify_operate(void)
{
    if (!s_enabled || !s_inited)
        return;

    s_session_active = true;
    s_fast_until = lv_tick_get() + GOV_FAST_HOLD_MS;

    /* Proactive bump to 60 Hz. gov_apply() only sets timer-period fields
     * (cheap, low-risk). The heavy synchronous lv_refr_now() walks the display
     * tree and MUST run on the LVGL thread — but this hook is also called off
     * the LVGL thread (gesture_handler / handle_gesture_unlock run on the
     * gesture-recognition thread). Only force the immediate refresh when we're
     * on the LVGL thread (e.g. the touch indev read path); otherwise skip it —
     * the period is now 16 ms so the next refr tick is <=16 ms away anyway, so
     * the transition's first frame is not meaningfully delayed. */
    gov_apply(true);
    extern bool is_on_lvgl_thread(void);
    if (is_on_lvgl_thread())
        lv_refr_now(lv_disp_get_default());
}

void disp_gov_notify_screen_on(void)
{
    if (!s_inited)
        disp_gov_init();
    if (!s_enabled || !s_inited)
        return;

    /* New wake session: clear the latch and debounce the wake-causing touch
     * so a glance stays at 1 Hz. */
    s_session_active = false;
    s_wake_debounce_until = lv_tick_get() + GOV_WAKE_DEBOUNCE_MS;
    /* Hold 60 Hz for ~1 s after wake so the wake frame + any immediate
       interaction stay smooth; the monitor drops to 1 Hz once it expires. */
    s_fast_until = lv_tick_get() + GOV_FAST_HOLD_MS;

    /* Land fast for the wake fade-in; the settle re-check will drop us to
     * 1 Hz once we're idle on home. */
    gov_apply(true);
    /* Force an immediate synchronous paint so the wake frame shows NOW instead
     * of waiting up to ~1 s for the next refr tick at 1 Hz — this is THE wake-
     * latency fix. Same guard as disp_gov_notify_operate(): lv_refr_now() must
     * run on the LVGL thread, and the RESUME event handler runs there. */
    {
        extern bool is_on_lvgl_thread(void);
        if (is_on_lvgl_thread())
            lv_refr_now(lv_disp_get_default());
    }
    gov_schedule_recheck();
}

void disp_gov_notify_screen_off(void)
{
    if (!s_enabled || !s_inited)
        return;

    s_session_active = false;
    s_wake_debounce_until = 0;
    /* Pre-arm 60 Hz for the NEXT wake. lv_timers are frozen while the panel is
       off so this costs nothing until resume, but the wake then inherits a 60 Hz
       refr period — the slow part was waking up still at 1 Hz. The fade-out anim
       keeps gov_should_be_slow() false until the timers freeze, so the monitor
       cannot undo this before sleep. */
    gov_apply(true);
}

bool disp_gov_notify_touch_press(void)
{
    if (!s_enabled || !s_inited)
        return false;

    /* Inside the post-wake debounce window: this press is (or may be) the
     * wake-causing touch — ignore it as an operate input. */
    if (s_wake_debounce_until != 0)
    {
        if ((int32_t)(s_wake_debounce_until - lv_tick_get()) > 0)
            return true; /* suppressed */
        s_wake_debounce_until = 0; /* window elapsed */
    }

    /* A genuine post-wake operate press. */
    disp_gov_notify_operate();
    return false;
}

void disp_gov_set_force(disp_gov_force_t force)
{
    s_force = force;
    if (!s_inited)
        disp_gov_init();
    gov_evaluate();
    LOG_I("force = %d", (int)force);
}

/* ---- MSH commands (dev-only) ---- */

#if defined(RT_USING_FINSH) && !kReleaseMode
#include <finsh.h>

static const char *gov_state_str(bool fast)
{
    return fast ? "60Hz" : "1Hz";
}

static void disp_gov(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("disp_gov <cmd>\n");
        rt_kprintf("  on            - enable governor\n");
        rt_kprintf("  off           - disable (stock fixed-60Hz)\n");
        rt_kprintf("  indev on|off  - layer-2 touch indev throttle\n");
        rt_kprintf("  dvfs on|off   - layer-3 DVFS (48MHz idle / 144MHz active)\n");
        rt_kprintf("  force 60|1|auto - pin refresh / let governor decide\n");
        rt_kprintf("  state         - print current state\n");
        return;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        disp_gov_set_enabled(true);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        disp_gov_set_enabled(false);
    }
    else if (rt_strcmp(argv[1], "indev") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("usage: disp_gov indev on|off\n");
            return;
        }
        disp_gov_set_indev_throttle(rt_strcmp(argv[2], "on") == 0);
    }
    else if (rt_strcmp(argv[1], "dvfs") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("usage: disp_gov dvfs on|off\n");
            return;
        }
        disp_gov_set_dvfs(rt_strcmp(argv[2], "on") == 0);
    }
    else if (rt_strcmp(argv[1], "force") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("usage: disp_gov force 60|1|auto\n");
            return;
        }
        if (rt_strcmp(argv[2], "60") == 0)
            disp_gov_set_force(DISP_GOV_FORCE_60HZ);
        else if (rt_strcmp(argv[2], "1") == 0)
            disp_gov_set_force(DISP_GOV_FORCE_1HZ);
        else
            disp_gov_set_force(DISP_GOV_FORCE_NONE);
    }
    else if (rt_strcmp(argv[1], "state") == 0)
    {
        lv_timer_t *rt = gov_refr_timer();
        lv_indev_t *indev = gov_indev();
        rt_kprintf("=== disp refresh governor ===\n");
        rt_kprintf("enabled        : %s\n", s_enabled ? "YES" : "NO");
        rt_kprintf("inited         : %s\n", s_inited ? "YES" : "NO");
        rt_kprintf("indev throttle : %s\n", s_indev_throttle ? "ON" : "OFF");
        rt_kprintf("dvfs           : %s\n", s_dvfs ? "ON" : "OFF");
#ifdef RT_USING_PM
        rt_kprintf("run mode       : %u (0=240 1=144 2=48 3=24 MHz)\n",
                   (unsigned)rt_pm_run_mode_get());
#endif
        rt_kprintf("force          : %s\n",
                   s_force == DISP_GOV_FORCE_NONE ? "auto"
                   : s_force == DISP_GOV_FORCE_60HZ ? "60Hz"
                                                    : "1Hz");
        rt_kprintf("applied state  : %s\n", gov_state_str(s_is_fast));
        rt_kprintf("monitor        : %s (%u ms)\n",
                   s_monitor_timer ? "ON" : "OFF", (unsigned)GOV_MONITOR_MS);
        rt_kprintf("refr period    : %u ms\n",
                   rt ? (unsigned)rt->period : 0u);
        rt_kprintf("session_active : %s\n", s_session_active ? "YES" : "NO");
        rt_kprintf("is_at_home     : %s\n", is_at_home() ? "YES" : "NO");
        rt_kprintf("anim running   : %u\n", (unsigned)lv_anim_count_running());
        rt_kprintf("wake debounce  : %s\n",
                   (s_wake_debounce_until != 0 &&
                    (int32_t)(s_wake_debounce_until - lv_tick_get()) > 0)
                       ? "ACTIVE"
                       : "off");
        if (indev && indev->driver && indev->driver->read_timer)
            rt_kprintf("indev period   : %u ms\n",
                       (unsigned)indev->driver->read_timer->period);
    }
    else
    {
        rt_kprintf("unknown cmd '%s'\n", argv[1]);
    }
}
MSH_CMD_EXPORT(disp_gov, dynamic display refresh governor control);

#endif /* RT_USING_FINSH && !kReleaseMode */

#endif /* DISP_REFR_GOVERNOR_ENABLE */
