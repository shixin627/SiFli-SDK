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

#define DBG_TAG "disp.gov"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

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
#define GOV_WAKE_DEBOUNCE_MS 250U /* ignore touch as operate after wake    */
#define GOV_INDEV_SLOW_MS 1000U  /* layer-2: slowed indev poll in 1 Hz    */

/* ---- runtime state ---- */

static bool s_enabled = false;          /* master switch, default OFF      */
static bool s_indev_throttle = false;   /* layer-2 sub-switch, default OFF */
static bool s_inited = false;

static bool s_session_active = false;   /* the latch                       */
static bool s_is_fast = true;           /* current applied state (60 Hz)   */
static disp_gov_force_t s_force = DISP_GOV_FORCE_NONE;

static uint32_t s_wake_debounce_until = 0; /* tick deadline (0 = inactive) */

static lv_timer_t *s_recheck_timer = NULL; /* one-shot settle re-check     */
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

/* Apply a refresh state to the LVGL refr_timer (and layer-2 indev). */
static void gov_apply(bool fast)
{
    lv_timer_t *rt = gov_refr_timer();
    if (!rt)
        return;

    lv_timer_set_period(rt, fast ? GOV_PERIOD_60HZ_MS : GOV_PERIOD_1HZ_MS);
    if (fast)
    {
        /* make sure the next tick is soon, not up to 1 s away */
        lv_timer_reset(rt);
    }

    gov_indev_set_slow(!fast);

    if (s_is_fast != fast)
    {
        LOG_I("refresh -> %s", fast ? "60Hz" : "1Hz");
        s_is_fast = fast;
    }
}

/* Should we be at the slow (1 Hz) state right now? */
static bool gov_should_be_slow(void)
{
    if (s_force == DISP_GOV_FORCE_60HZ)
        return false;
    if (s_force == DISP_GOV_FORCE_1HZ)
        return true;

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
        gov_evaluate();
}

void disp_gov_set_enabled(bool en)
{
    s_enabled = en;
    if (!s_inited)
        disp_gov_init();

    if (en)
    {
        gov_evaluate();
    }
    else
    {
        /* Restore stock fixed-60 Hz and any throttled indev. */
        bool saved_throttle = s_indev_throttle;
        s_indev_throttle = true;       /* force indev restore path */
        gov_indev_set_slow(false);
        s_indev_throttle = saved_throttle;

        lv_timer_t *rt = gov_refr_timer();
        if (rt)
            lv_timer_set_period(rt, GOV_PERIOD_60HZ_MS);
        s_is_fast = true;
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

void disp_gov_notify_operate(void)
{
    if (!s_enabled || !s_inited)
        return;

    s_session_active = true;

    /* Proactive bump: 60 Hz now + force an immediate refresh so the first
     * transition frame is not delayed by the next refr tick. */
    gov_apply(true);
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

    /* Land fast for the wake fade-in; the settle re-check will drop us to
     * 1 Hz once we're idle on home. */
    gov_apply(true);
    gov_schedule_recheck();
}

void disp_gov_notify_screen_off(void)
{
    if (!s_enabled || !s_inited)
        return;

    s_session_active = false;
    s_wake_debounce_until = 0;
    /* Leave the period as-is; the panel is off. RESUME will re-apply. */
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
        rt_kprintf("force          : %s\n",
                   s_force == DISP_GOV_FORCE_NONE ? "auto"
                   : s_force == DISP_GOV_FORCE_60HZ ? "60Hz"
                                                    : "1Hz");
        rt_kprintf("applied state  : %s\n", gov_state_str(s_is_fast));
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
