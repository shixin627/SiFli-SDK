/*
 * Skai SDK — alarms that survive sleep. See skai_alarm.h.
 *
 * Shape copied from the alarm clock (alarm_manager_service.c + alarm_client.c),
 * because that is the one path proven to wake this watch from deep sleep:
 *   rt_alarm (RTC, AON domain) → callback on the soft-RTC thread does nothing
 *   but release a semaphore → our worker wakes the GUI/HCPU, waits for the
 *   peripheral rails (a buzz in the first ~500 ms after resume is swallowed),
 *   then vibrates and posts the notification.
 * A kernel timer or rt_thread_mdelay would simply never come back once the
 * core stops (see the test app's wake cycle for the same lesson).
 */
#include <string.h>
#include <time.h>
#include <rtthread.h>
#include <rthw.h>

#define DBG_TAG "skai.alarm"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "skai/skai_alarm.h"
#include "skai/skai_js.h"
#include "bloc_notification.h"
#include "watch_system_interact.h"

#if defined(RT_USING_ALARM) && !defined(BSP_USING_PC_SIMULATOR)
#include <drivers/alarm.h>
#include "gui_app_pm.h"
#include "bloc_peripheral.h"
#define SKAI_ALARM_RTC 1
#else
#define SKAI_ALARM_RTC 0
#endif

typedef struct
{
    bool     used;
    bool     fired;
    time_t   at;                         /* wall-clock second it fires */
    char     app[SKAI_JS_APP_ID_MAX];    /* owner: only it may query/cancel */
    char     text[SKAI_ALARM_TEXT_MAX];
#if SKAI_ALARM_RTC
    rt_alarm_t hw;
#else
    rt_timer_t tm;                       /* simulator: no RTC alarm there */
#endif
} slot_t;

static slot_t        s_slots[SKAI_ALARM_SLOTS];
static struct rt_mutex s_lock;
static rt_sem_t      s_fire_sem;
static volatile bool s_fired_event;
static bool          s_inited;

static void ensure_init(void);

static const char *owner(void)
{
#ifdef PKG_USING_QUICKJS
    const char *a = skai_js_current_app_id();
    return a ? a : "";
#else
    return "";
#endif
}

static void on_hw_fire(void)
{
    /* Soft-RTC / timer context: nothing but a hand-off. */
    if (s_fire_sem)
        rt_sem_release(s_fire_sem);
}

#if SKAI_ALARM_RTC
static void rtc_cb(rt_alarm_t alarm, time_t ts)
{
    (void)alarm;
    (void)ts;
    on_hw_fire();
}
#else
static void tm_cb(void *arg)
{
    (void)arg;
    on_hw_fire();
}
#endif

static void release_hw(slot_t *s)
{
#if SKAI_ALARM_RTC
    if (s->hw)
    {
        rt_alarm_stop(s->hw);
        rt_alarm_delete(s->hw);
        s->hw = RT_NULL;
    }
#else
    if (s->tm)
    {
        rt_timer_stop(s->tm);
        rt_timer_delete(s->tm);
        s->tm = RT_NULL;
    }
#endif
}

int32_t skai_alarm_set(const char *text, int32_t seconds)
{
    int32_t id = 0;
    time_t now;

    ensure_init();
    if (seconds < 1)
        seconds = 1;
    if (seconds > 86400)
        seconds = 86400;
    now = time(RT_NULL);

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    for (int i = 0; i < SKAI_ALARM_SLOTS && id == 0; i++)
    {
        /* A fired alarm nobody cleared for 10 minutes is fair game. */
        if (s_slots[i].used && !(s_slots[i].fired && now - s_slots[i].at > 600))
            continue;
        slot_t *s = &s_slots[i];
        release_hw(s);
        memset(s, 0, sizeof(*s));
        s->used = true;
        s->at = now + seconds;
        rt_strncpy(s->app, owner(), sizeof(s->app) - 1);
        rt_strncpy(s->text, (text && text[0]) ? text : "Timer", sizeof(s->text) - 1);
#if SKAI_ALARM_RTC
        {
            struct rt_alarm_setup setup;
            struct tm at;
            time_t when = s->at;
            /* gmtime, not localtime: the RTC alarm service compares against
               gmtime_r(time()) (rtc/alarm.c), because this watch keeps its
               wall clock in time() as if it were UTC. A localtime conversion
               shifts the target by the timezone and the alarm is refused or
               rings hours off.
               Full date from the computed time, never RT_ALARM_TM_NOW: a timer
               that crosses midnight would otherwise be stamped with today and
               land in the past. */
            gmtime_r(&when, &at);
            memset(&setup, 0, sizeof(setup));
            setup.flag = RT_ALARM_ONESHOT;
            setup.wktime.tm_year = at.tm_year;
            setup.wktime.tm_mon = at.tm_mon;
            setup.wktime.tm_mday = at.tm_mday;
            setup.wktime.tm_hour = at.tm_hour;
            setup.wktime.tm_min = at.tm_min;
            setup.wktime.tm_sec = at.tm_sec;
            s->hw = rt_alarm_create(rtc_cb, &setup);
            rt_err_t started = s->hw ? rt_alarm_start(s->hw) : -RT_ENOMEM;
            if (started != RT_EOK)
            {
                LOG_W("rt_alarm failed: create=%s start=%d rtc=%s at %04d-%02d-%02d %02d:%02d:%02d",
                      s->hw ? "ok" : "NULL", (int)started,
                      rt_device_find("rtc") ? "ok" : "MISSING",
                      at.tm_year + 1900, at.tm_mon + 1, at.tm_mday,
                      at.tm_hour, at.tm_min, at.tm_sec);
                release_hw(s);
                s->used = false;
                break;
            }
        }
#else
        s->tm = rt_timer_create("skalm", tm_cb, RT_NULL,
                                rt_tick_from_millisecond(seconds * 1000),
                                RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        if (s->tm)
            rt_timer_start(s->tm);
#endif
        id = i + 1;
    }
    rt_mutex_release(&s_lock);
    if (id)
    {
        /* Let the worker recompute its wait with this alarm in it. */
        if (s_fire_sem)
            rt_sem_release(s_fire_sem);
        LOG_W("set #%d for %s in %ds", (int)id, owner(), (int)seconds);
    }
    return id;
}

static slot_t *mine(int32_t id)
{
    if (id < 1 || id > SKAI_ALARM_SLOTS || !s_slots[id - 1].used)
        return RT_NULL;
    if (strcmp(s_slots[id - 1].app, owner()) != 0)
        return RT_NULL;
    return &s_slots[id - 1];
}

bool skai_alarm_cancel(int32_t id)
{
    bool ok = false;

    ensure_init();
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    slot_t *s = mine(id);
    if (s)
    {
        release_hw(s);
        memset(s, 0, sizeof(*s));
        ok = true;
    }
    rt_mutex_release(&s_lock);
    return ok;
}

int32_t skai_alarm_remaining(int32_t id)
{
    int32_t left = SKAI_NO_DATA;

    ensure_init();
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    slot_t *s = mine(id);
    if (s)
    {
        time_t now = time(RT_NULL);
        left = (s->fired || s->at <= now) ? 0 : (int32_t)(s->at - now);
    }
    rt_mutex_release(&s_lock);
    return left;
}

bool skai_alarm_take_fired_event(void)
{
    if (!s_fired_event)
        return false;
    s_fired_event = false;
    return true;
}

static void fire(const char *text)
{
#if SKAI_ALARM_RTC
    /* Wake the screen/HCPU first and give the peripheral rails time: a buzz
       sent in the first ~500 ms after resume is silently lost. */
    if (!gui_is_active())
    {
        gui_pm_fsm(GUI_PM_ACTION_BUTTON_CLICKED);
        peripheral_provider.hcpu_resume();
        rt_thread_mdelay(500);
    }
#endif
    {
        notification_t n;
        memset(&n, 0, sizeof(n));
        rt_snprintf(n.id, sizeof(n.id), "skai-alarm:%u", (unsigned)rt_tick_get());
        strncpy(n.title, text, sizeof(n.title) - 1);
        strncpy(n.message, "\xe2\x8f\xb0", sizeof(n.message) - 1); /* ⏰ */
        n.sec_time = (uint32_t)time(RT_NULL);
        n.type = Notify_others;
        n.state = true;
        interact_with_notification(&n);
    }
    motor_pattern_timer_reminder();
    LOG_W("fired: %s", text);
}

static void worker(void *arg)
{
    (void)arg;
    for (;;)
    {
        char due[SKAI_ALARM_SLOTS][SKAI_ALARM_TEXT_MAX];
        int n_due = 0;

        /* Wait for the RTC alarm. The timeout is only a net (an RTC that
           rounded down, a watch that stayed awake) and exists only while an
           alarm is pending — an idle worker waking every second would cost
           battery and could hold the core out of deep sleep. */
        rt_int32_t wait = RT_WAITING_FOREVER;
        {
            time_t t = time(RT_NULL), nearest = 0;
            rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
            for (int i = 0; i < SKAI_ALARM_SLOTS; i++)
                if (s_slots[i].used && !s_slots[i].fired &&
                        (nearest == 0 || s_slots[i].at < nearest))
                    nearest = s_slots[i].at;
            rt_mutex_release(&s_lock);
            if (nearest)
                wait = rt_tick_from_millisecond(((nearest > t) ? (int32_t)(nearest - t) : 0) * 1000 + 1000);
        }
        rt_sem_take(s_fire_sem, wait);
        time_t now = time(RT_NULL);

        rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
        for (int i = 0; i < SKAI_ALARM_SLOTS; i++)
        {
            slot_t *s = &s_slots[i];
            if (!s->used || s->fired || s->at > now)
                continue;
            s->fired = true;
            release_hw(s);
            rt_strncpy(due[n_due], s->text, SKAI_ALARM_TEXT_MAX - 1);
            due[n_due][SKAI_ALARM_TEXT_MAX - 1] = '\0';
            n_due++;
        }
        rt_mutex_release(&s_lock);

        for (int i = 0; i < n_due; i++)
            fire(due[i]);
        if (n_due)
            s_fired_event = true;
    }
}

static void ensure_init(void)
{
    rt_base_t level;

    if (s_inited)
        return;
    level = rt_hw_interrupt_disable();
    if (s_inited)
    {
        rt_hw_interrupt_enable(level);
        return;
    }
    s_inited = true;
    rt_hw_interrupt_enable(level);

    rt_mutex_init(&s_lock, "skalm", RT_IPC_FLAG_PRIO);
    s_fire_sem = rt_sem_create("skalm", 0, RT_IPC_FLAG_FIFO);
    {
        rt_thread_t t = rt_thread_create("skalm", worker, RT_NULL, 4096, 18, 10);
        if (t)
            rt_thread_startup(t);
    }
}

#ifdef FINSH_USING_MSH
#include <finsh.h>
/* skalm_ls — every pending/fired alarm on the watch (dev console). */
static void skalm_ls(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    ensure_init();
    time_t now = time(RT_NULL);
    for (int i = 0; i < SKAI_ALARM_SLOTS; i++)
        if (s_slots[i].used)
            rt_kprintf("#%d app=%s %s in=%d text=%s\n", i + 1, s_slots[i].app,
                       s_slots[i].fired ? "FIRED" : "pending",
                       (int)(s_slots[i].at - now), s_slots[i].text);
}
MSH_CMD_EXPORT(skalm_ls, list skai alarms);

#if SKAI_ALARM_RTC
/* gui_sleep — put the screen and GUI to sleep now, as the side button does
   (dev console), to test that an alarm still wakes the watch. */
static void gui_sleep(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    gui_pm_fsm(GUI_PM_ACTION_SLEEP);
}
MSH_CMD_EXPORT(gui_sleep, suspend the GUI now (dev));
#endif
#endif
