/*
 * Skai SDK — internal C capability layer (Phase 1, ADR-0019).
 *
 * Thin wrappers over backends that already exist. Deliberately API, not ABI:
 * callers ship in the same commit, so signatures may be refactored freely and
 * the compiler catches breaks. No opaque handles, no versioned structs, no
 * compat shims — that discipline belongs to the external scripting API, which
 * is projected from these headers by tools/sdk/gen_dispatch.py.
 *
 * ponytail: one .c for eight domains. They are 3-line wrappers over globals;
 * eight files of boilerplate would be worse. Split a domain out when it grows
 * real state of its own.
 */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <rtthread.h>

#define DBG_TAG "skai"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "bloc_peripheral.h"
#include "bloc_weather.h"
#include "share_prefs.h"

#include "skai/skai_battery.h"
#include "skai/skai_haptic.h"
#include "skai/skai_health.h"
#include "skai/skai_log.h"
#include "skai/skai_persist.h"
#include "skai/skai_time.h"
#include "skai/skai_timer.h"
#include "skai/skai_watch_info.h"
#include "skai/skai_weather.h"

/* ------------------------------------------------------------------ time */

int32_t skai_time_now(void)
{
    time_t now = time(RT_NULL);
    /* Before the phone first syncs the clock the RTC reads near zero. Report
     * that as failure rather than handing apps a 1970 timestamp they will
     * render as a plausible-looking wrong date. */
    if (now < 946684800) /* 2000-01-01 */
        return SKAI_NO_DATA;
    return (int32_t)now;
}

int32_t skai_time_format(const char *fmt, char *out, uint32_t cap)
{
    time_t now;
    struct tm *tm_now;
    size_t n;

    if (!fmt || !out || cap == 0)
        return -1;
    out[0] = '\0';

    now = time(RT_NULL);
    tm_now = localtime(&now);
    if (!tm_now)
        return -1;

    n = strftime(out, cap, fmt, tm_now);
    /* strftime returns 0 both for "empty result" and "did not fit", and does
     * not necessarily terminate on overflow. Terminate and report failure. */
    if (n == 0)
    {
        out[0] = '\0';
        return -1;
    }
    return (int32_t)n;
}

int32_t skai_time_hhmm(char *out, uint32_t cap)
{
    /* The 12/24-hour rule is expressed once, here, as a strftime pattern.
     * gui_apps/common/ui_helper.c's ui_time_format_hhmm() is the peer that
     * formats an ARBITRARY time (this one only formats now); the two read the
     * same SkaiWatchSys.flag_field.hour_format and must stay in agreement. */
    return skai_time_format(SkaiWatchSys.flag_field.hour_format == 1
                            ? "%H:%M" : "%I:%M %p", out, cap);
}

int32_t skai_time_date_md(char *out, uint32_t cap)
{
    return skai_time_format("%m/%d", out, cap);
}

/* --------------------------------------------------------------- battery */

int32_t skai_battery_level(void)
{
    uint8_t level = SkaiWatchSys.battery_level_value;
    if (level == 0 || level > 100)
        return SKAI_NO_DATA; /* no reading from LCPU yet */
    return (int32_t)level;
}

bool skai_battery_charging(void)
{
    T_CHARGE_STATUS s = SkaiWatchSys.charger_status;
    return (s == InCharging) || (s == ChargingComplete);
}

/* ---------------------------------------------------------------- health */

int32_t skai_health_steps(void)      { return (int32_t)SkaiWatchSys.gPedoData.global_steps; }
int32_t skai_health_distance_m(void) { return (int32_t)SkaiWatchSys.gPedoData.global_distance; }
int32_t skai_health_calories(void)   { return (int32_t)SkaiWatchSys.gPedoData.global_calories; }
bool    skai_health_worn(void)       { return SkaiWatchSys.flag_field.is_wearing ? true : false; }

int32_t skai_health_heart_rate(void)
{
    uint8_t bpm = SkaiWatchSys.heart_rate_bpm;
    /* 0 means "no measurement", not "heart stopped". */
    if (bpm == 0)
        return SKAI_NO_DATA;
    return (int32_t)bpm;
}

int32_t skai_health_step_goal(void)
{
    uint32_t goal = SkaiWatchSys.gPedoData.daily_step_target;
    return goal ? (int32_t)goal : SKAI_NO_DATA;
}

/* --------------------------------------------------------------- weather */

/* get_weather(0) is the current slot. An all-zero record means the phone has
 * not sent a forecast yet — 0 C is a real temperature, so emptiness is judged
 * by the whole record, not by one field. */
static const weather_t *weather_now(void)
{
    const weather_t *w = get_weather(0);
    if (!w || (w->temperature == 0.0f && w->max_temperature == 0.0f
               && w->min_temperature == 0.0f && w->description[0] == '\0'))
        return RT_NULL;
    return w;
}

static int32_t weather_round(float c)
{
    return (int32_t)((c >= 0.0f) ? (c + 0.5f) : (c - 0.5f));
}

int32_t skai_weather_temp(void)
{
    const weather_t *w = weather_now();
    return w ? weather_round(w->temperature) : SKAI_NO_DATA;
}

int32_t skai_weather_temp_min(void)
{
    const weather_t *w = weather_now();
    return w ? weather_round(w->min_temperature) : SKAI_NO_DATA;
}

int32_t skai_weather_temp_max(void)
{
    const weather_t *w = weather_now();
    return w ? weather_round(w->max_temperature) : SKAI_NO_DATA;
}

int32_t skai_weather_rain_pct(void)
{
    const weather_t *w = weather_now();
    return w ? (int32_t)w->precipitationProbability : SKAI_NO_DATA;
}

/* ---------------------------------------------------------------- haptic */

/* Built-in patterns. duty_cycle/period(us)/repeat match motor_params_t; the
 * tap values follow alarm_client's burst so haptics feel consistent across
 * the watch instead of every caller inventing its own buzz. */
static const motor_params_t s_patterns[] =
{
    { .duty_cycle = 51, .period = 100000, .repeat_times = 1 }, /* 0: tap    */
    { .duty_cycle = 51, .period = 200000, .repeat_times = 2 }, /* 1: double */
    { .duty_cycle = 51, .period = 200000, .repeat_times = 3 }, /* 2: alert  */
    { .duty_cycle = 80, .period = 400000, .repeat_times = 1 }, /* 3: long   */
};
#define SKAI_HAPTIC_PATTERN_COUNT (sizeof(s_patterns) / sizeof(s_patterns[0]))

bool skai_haptic_vibrate(uint32_t pattern_id)
{
    motor_params_t params;

    if (pattern_id >= SKAI_HAPTIC_PATTERN_COUNT)
        return false;
    /* The user's global vibration mute wins. An app must not be able to buzz
     * a watch whose owner turned vibration off. */
    if (!get_motor_switch_state())
        return false;
    if (!peripheral_provider.control_motor)
        return false;

    params = s_patterns[pattern_id];
    peripheral_provider.control_motor(true, &params);
    return true;
}

bool skai_haptic_stop(void)
{
    if (!peripheral_provider.control_motor)
        return false;
    peripheral_provider.control_motor(false, RT_NULL);
    return true;
}

/* ------------------------------------------------------------------- log */

void skai_log_write(uint32_t level, const char *msg)
{
    if (!msg)
        return;
    switch (level)
    {
    case SKAI_LOG_DEBUG: LOG_D("%s", msg); break;
    case SKAI_LOG_WARN:  LOG_W("%s", msg); break;
    case SKAI_LOG_ERROR: LOG_E("%s", msg); break;
    default:             LOG_I("%s", msg); break; /* unknown level != dropped */
    }
}

/* --------------------------------------------------------------- persist */

/* ponytail: opened once, never closed. share_prefs_open() runs a full
 * fdb_kvdb_init every call (middleware/share_prefs/share_prefs_flashdb.c has
 * no cache), which on the NAND prefdb partition can exceed WDT1's 8 s. If
 * this ever needs to be closeable, add a refcount in share_prefs itself
 * rather than opening per call here. */
static share_prefs_t *s_prefs;

static share_prefs_t *persist_handle(const char *key)
{
    if (!key || key[0] == '\0' || strlen(key) > SKAI_PERSIST_KEY_MAX)
        return RT_NULL;
    if (!s_prefs)
    {
        s_prefs = share_prefs_open("skai", SHAREPREFS_MODE_PRIVATE);
        if (!s_prefs)
            LOG_W("share_prefs_open(skai) failed; persistence unavailable");
    }
    return s_prefs;
}

int32_t skai_persist_get_int(const char *key, int32_t fallback)
{
    share_prefs_t *p = persist_handle(key);
    if (!p)
        return fallback;
    return share_prefs_get_int(p, key, fallback);
}

bool skai_persist_set_int(const char *key, int32_t value)
{
    share_prefs_t *p = persist_handle(key);
    if (!p)
        return false;
    return share_prefs_set_int(p, key, value) == RT_EOK;
}

int32_t skai_persist_get_str(const char *key, char *out, uint32_t cap)
{
    share_prefs_t *p;
    int32_t n;

    if (!out || cap == 0)
        return -1;
    out[0] = '\0';

    p = persist_handle(key);
    if (!p)
        return -1;

    n = share_prefs_get_string(p, key, out, (int32_t)cap);
    if (n < 0)
    {
        out[0] = '\0';
        return -1;
    }
    out[cap - 1] = '\0';
    return n;
}

bool skai_persist_set_str(const char *key, const char *value)
{
    share_prefs_t *p = persist_handle(key);
    if (!p || !value)
        return false;
    return share_prefs_set_string(p, key, value) == RT_EOK;
}

bool skai_persist_remove(const char *key)
{
    share_prefs_t *p = persist_handle(key);
    if (!p)
        return false;
    /* Absent is a fine end state — the caller wanted it gone. */
    (void)share_prefs_remove(p, key);
    return true;
}

/* ----------------------------------------------------------------- timer */

/* ponytail: fixed slot table so a bad handle is rejected instead of
 * dereferenced. SKAI_TIMER_SLOTS is well past what the built-in apps use;
 * raise it in skai_timer.h if that stops being true. */
static rt_timer_t s_timers[SKAI_TIMER_SLOTS];

uint32_t skai_timer_uptime_ms(void)
{
    return (uint32_t)((uint64_t)rt_tick_get() * 1000u / RT_TICK_PER_SECOND);
}

static rt_timer_t timer_of(uint32_t handle)
{
    if (handle == 0 || handle > SKAI_TIMER_SLOTS)
        return RT_NULL;
    return s_timers[handle - 1];
}

uint32_t skai_timer_create(uint32_t period_ms, bool repeating,
                           skai_timer_cb_t cb, void *arg)
{
    rt_tick_t ticks;
    int slot;

    if (!cb)
        return 0;

    for (slot = 0; slot < SKAI_TIMER_SLOTS; slot++)
        if (!s_timers[slot])
            break;
    if (slot == SKAI_TIMER_SLOTS)
    {
        LOG_W("skai_timer: all %d slots in use", SKAI_TIMER_SLOTS);
        return 0;
    }

    /* Round up: a 1 ms request on a 100 Hz tick would otherwise become a
     * zero-tick timer, which rt_timer rejects. */
    ticks = rt_tick_from_millisecond((rt_int32_t)period_ms);
    if (ticks == 0)
        ticks = 1;

    s_timers[slot] = rt_timer_create("skai_tmr", cb, arg, ticks,
                                     (repeating ? RT_TIMER_FLAG_PERIODIC
                                                : RT_TIMER_FLAG_ONE_SHOT) |
                                     RT_TIMER_FLAG_SOFT_TIMER);
    if (!s_timers[slot])
        return 0;

    if (rt_timer_start(s_timers[slot]) != RT_EOK)
    {
        rt_timer_delete(s_timers[slot]);
        s_timers[slot] = RT_NULL;
        return 0;
    }
    return (uint32_t)(slot + 1);
}

bool skai_timer_stop(uint32_t handle)
{
    rt_timer_t t = timer_of(handle);
    if (!t)
        return false;
    rt_timer_stop(t);
    return true;
}

bool skai_timer_destroy(uint32_t handle)
{
    rt_timer_t t = timer_of(handle);
    if (!t)
        return false;
    rt_timer_stop(t);
    rt_timer_delete(t);
    s_timers[handle - 1] = RT_NULL;
    return true;
}

/* ------------------------------------------------------------ watch info */

int32_t skai_watchinfo_firmware(char *out, uint32_t cap)
{
    int n;
    if (!out || cap == 0)
        return -1;
    if (VERSION_DEV)
        n = rt_snprintf(out, cap, "%d.%d.%d-dev",
                        VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION);
    else
        n = rt_snprintf(out, cap, "%d.%d.%d",
                        VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION);
    out[cap - 1] = '\0';
    if (n < 0)
        return -1;
    return (n >= (int)cap) ? (int32_t)(cap - 1) : (int32_t)n;
}

int32_t skai_watchinfo_model(char *out, uint32_t cap)
{
    size_t n;
    /* Board identity, not marketing name — apps branch on hardware. */
    static const char model[] = "sf32lb56-watch";

    if (!out || cap == 0)
        return -1;
    n = strlen(model);
    if (n >= cap)
        n = cap - 1;
    memcpy(out, model, n);
    out[n] = '\0';
    return (int32_t)n;
}

/* The watch build defines LCD_*; the PC simulator only has LVGL's LV_*.
 * No silent 466 default — a board that supplies neither should fail to build
 * rather than ship apps a made-up screen size. */
#if defined(LCD_HOR_RES_MAX)
    #define SKAI_SCREEN_W LCD_HOR_RES_MAX
    #define SKAI_SCREEN_H LCD_VER_RES_MAX
#elif defined(LV_HOR_RES_MAX)
    #define SKAI_SCREEN_W LV_HOR_RES_MAX
    #define SKAI_SCREEN_H LV_VER_RES_MAX
#else
    #error "skai_watchinfo: no LCD_/LV_ screen resolution define for this board"
#endif

uint32_t skai_watchinfo_screen_width(void)  { return SKAI_SCREEN_W; }
uint32_t skai_watchinfo_screen_height(void) { return SKAI_SCREEN_H; }
bool     skai_watchinfo_screen_round(void)  { return true; }
