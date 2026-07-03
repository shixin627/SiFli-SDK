/**
 * @file   skaiapp_engine.c
 * @brief  SkaiApp background runtime (see header for the design contract).
 *
 * Thread model: all mutation happens under s_mx; fires are gathered under the
 * lock into a small queue and EXECUTED after release (interact_with_notification
 * blocks while the HCPU is suspended — never hold the lock across it). The
 * worker thread is the only context that touches the FS (fire-time text
 * re-read + debounced toggle persist), mirroring the alarm service's
 * sem-wakes-thread pattern; rt_timer callbacks never do real work here.
 */
#include <string.h>
#include <stdio.h>
#include <rtthread.h>
#include "cJSON.h"
#include "skaiapp_engine.h"
#include "skaiapp_store.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "bloc_notification.h"

#define DBG_TAG "skaiapp.eng"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ── resident records (kept intentionally tiny — SRAM budget) ── */
typedef struct
{
    uint32_t duration_s;
    uint32_t remaining_ms;   /* valid when not running */
    uint32_t started_ms;     /* tick-ms when it entered running */
    uint32_t run_base_ms;    /* remaining at the moment it started */
    int8_t   next;
    uint8_t  flags;
    uint8_t  state;          /* 0 idle / 1 running / 2 paused / 3 done */
} eng_timer_t;

typedef struct
{
    skaiapp_seed_rem_t cfg;
    uint32_t last_fire_min;  /* wall minutes since epoch; 0 = never */
} eng_rem_t;

typedef struct
{
    bool used;
    char app_id[SKAIAPP_ID_MAX];
    uint8_t n_t, n_r;
    eng_timer_t t[SKAIAPP_MAX_TIMERS];
    eng_rem_t   r[SKAIAPP_MAX_REMINDERS];
} eng_app_t;

static eng_app_t s_apps[SKAIAPP_MAX_APPS];
static rt_mutex_t s_mx = RT_NULL;
static rt_sem_t s_sem = RT_NULL;
static uint32_t s_last_eval_min = 0;

/* debounced reminder-toggle persist (last-write-wins single slot) */
static char s_persist_id[SKAIAPP_ID_MAX];
static uint32_t s_persist_due_ms = 0;
static bool s_persist_armed = false;
#define PERSIST_DEBOUNCE_MS 5000

typedef struct
{
    char app_id[SKAIAPP_ID_MAX];
    uint8_t kind; /* 0 timer / 1 reminder */
    uint8_t idx;
    uint8_t vibrate;
} fire_item_t;

static void lock(void)   { if (s_mx) rt_mutex_take(s_mx, RT_WAITING_FOREVER); }
static void unlock(void) { if (s_mx) rt_mutex_release(s_mx); }
static void poke(void)   { if (s_sem) rt_sem_release(s_sem); }

static uint32_t now_ms(void)
{
    return (uint32_t)((uint64_t)rt_tick_get() * 1000u / RT_TICK_PER_SECOND);
}

static uint32_t wall_sec(void)
{
    return SkaiWatchSys.SecondCountRTC;
}

static eng_app_t *find_app(const char *app_id)
{
    for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
    {
        if (s_apps[i].used && strcmp(s_apps[i].app_id, app_id) == 0)
        {
            return &s_apps[i];
        }
    }
    return NULL;
}

static uint32_t timer_live_remaining(const eng_timer_t *t)
{
    if (t->state != 1)
    {
        return t->remaining_ms;
    }
    uint32_t elapsed = now_ms() - t->started_ms; /* unsigned wrap-safe */
    return (elapsed >= t->run_base_ms) ? 0 : (t->run_base_ms - elapsed);
}

/* ── load / unload (BLE parse thread via store, or boot scan) ── */

void skaiapp_engine_load(const char *app_id, const skaiapp_eng_seed_t *seed)
{
    if (app_id == NULL || seed == NULL || s_mx == RT_NULL)
    {
        return;
    }
    lock();
    eng_app_t *a = find_app(app_id);
    if (a == NULL)
    {
        for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
        {
            if (!s_apps[i].used)
            {
                a = &s_apps[i];
                break;
            }
        }
    }
    if (a == NULL)
    {
        unlock();
        LOG_W("engine full, '%s' not scheduled", app_id);
        return;
    }
    memset(a, 0, sizeof(*a));
    a->used = true;
    strncpy(a->app_id, app_id, SKAIAPP_ID_MAX - 1);
    a->n_t = seed->n_timers;
    a->n_r = seed->n_reminders;
    for (int i = 0; i < a->n_t; i++)
    {
        a->t[i].duration_s = seed->t[i].duration_s;
        a->t[i].remaining_ms = seed->t[i].duration_s * 1000u;
        a->t[i].next = seed->t[i].next;
        a->t[i].flags = seed->t[i].flags;
        a->t[i].state = 0;
        if (seed->t[i].flags & SKAIAPP_TF_AUTOSTART)
        {
            a->t[i].state = 1;
            a->t[i].started_ms = now_ms();
            a->t[i].run_base_ms = a->t[i].remaining_ms;
        }
    }
    for (int i = 0; i < a->n_r; i++)
    {
        a->r[i].cfg = seed->r[i];
        a->r[i].last_fire_min = 0;
    }
    unlock();
    poke();
}

void skaiapp_engine_unload(const char *app_id)
{
    if (app_id == NULL || s_mx == RT_NULL)
    {
        return;
    }
    lock();
    eng_app_t *a = find_app(app_id);
    if (a != NULL)
    {
        memset(a, 0, sizeof(*a));
    }
    unlock();
}

/* ── timer controls (LVGL thread) ── */

void skaiapp_engine_timer_start(const char *app_id, int idx)
{
    lock();
    eng_app_t *a = find_app(app_id);
    if (a != NULL && idx >= 0 && idx < a->n_t)
    {
        eng_timer_t *t = &a->t[idx];
        if (t->state != 1)
        {
            if (t->state == 3 || t->remaining_ms == 0)
            {
                t->remaining_ms = t->duration_s * 1000u;
            }
            t->run_base_ms = t->remaining_ms;
            t->started_ms = now_ms();
            t->state = 1;
        }
    }
    unlock();
    poke();
}

void skaiapp_engine_timer_pause(const char *app_id, int idx)
{
    lock();
    eng_app_t *a = find_app(app_id);
    if (a != NULL && idx >= 0 && idx < a->n_t)
    {
        eng_timer_t *t = &a->t[idx];
        if (t->state == 1)
        {
            t->remaining_ms = timer_live_remaining(t);
            t->state = 2;
        }
    }
    unlock();
}

void skaiapp_engine_timer_reset(const char *app_id, int idx)
{
    lock();
    eng_app_t *a = find_app(app_id);
    if (a != NULL && idx >= 0 && idx < a->n_t)
    {
        eng_timer_t *t = &a->t[idx];
        t->state = 0;
        t->remaining_ms = t->duration_s * 1000u;
    }
    unlock();
}

bool skaiapp_engine_timer_query(const char *app_id, int idx,
                                uint32_t *remaining_ms, bool *running,
                                uint32_t *duration_s)
{
    bool ok = false;
    lock();
    eng_app_t *a = find_app(app_id);
    if (a != NULL && idx >= 0 && idx < a->n_t)
    {
        const eng_timer_t *t = &a->t[idx];
        if (remaining_ms != NULL)
        {
            *remaining_ms = timer_live_remaining(t);
        }
        if (running != NULL)
        {
            *running = (t->state == 1);
        }
        if (duration_s != NULL)
        {
            *duration_s = t->duration_s;
        }
        ok = true;
    }
    unlock();
    return ok;
}

/* ── reminder controls ── */

bool skaiapp_engine_reminder_toggle(const char *app_id, int idx)
{
    bool now_on = false;
    lock();
    eng_app_t *a = find_app(app_id);
    if (a != NULL && idx >= 0 && idx < a->n_r)
    {
        a->r[idx].cfg.enabled = a->r[idx].cfg.enabled ? 0 : 1;
        now_on = a->r[idx].cfg.enabled;
        strncpy(s_persist_id, app_id, SKAIAPP_ID_MAX - 1);
        s_persist_id[SKAIAPP_ID_MAX - 1] = '\0';
        s_persist_due_ms = now_ms() + PERSIST_DEBOUNCE_MS;
        s_persist_armed = true;
    }
    unlock();
    poke();
    return now_on;
}

bool skaiapp_engine_reminder_enabled(const char *app_id, int idx)
{
    bool on = false;
    lock();
    eng_app_t *a = find_app(app_id);
    if (a != NULL && idx >= 0 && idx < a->n_r)
    {
        on = a->r[idx].cfg.enabled;
    }
    unlock();
    return on;
}

/* minutes-of-day of the next fire; 0xFFFF = disabled / none */
static uint16_t reminder_next_locked(const eng_rem_t *r, uint16_t mod_now)
{
    const skaiapp_seed_rem_t *c = &r->cfg;
    if (!c->enabled)
    {
        return 0xFFFF;
    }
    if (c->kind == 1) /* daily */
    {
        uint16_t best = 0xFFFF;
        uint16_t best_delta = 0xFFFF;
        for (int i = 0; i < c->n_times; i++)
        {
            uint16_t delta = (uint16_t)((c->times[i] + 1440 - mod_now - 1) % 1440) + 1;
            if (delta < best_delta)
            {
                best_delta = delta;
                best = c->times[i];
            }
        }
        return best;
    }
    /* interval */
    uint16_t every = (c->every_min == 0) ? 60 : c->every_min;
    bool has_win = !(c->win_start == 0 && c->win_end == 0);
    uint16_t start = has_win ? c->win_start : 0;
    for (uint16_t k = 1; k <= 1440; k++)
    {
        uint16_t m = (uint16_t)((mod_now + k) % 1440);
        uint16_t off = (uint16_t)((m + 1440 - start) % 1440);
        if ((off % every) != 0)
        {
            continue;
        }
        if (has_win)
        {
            bool in;
            if (c->win_start <= c->win_end)
            {
                in = (m >= c->win_start && m <= c->win_end);
            }
            else
            {
                in = (m >= c->win_start || m <= c->win_end);
            }
            if (!in)
            {
                continue;
            }
        }
        return m;
    }
    return 0xFFFF;
}

uint16_t skaiapp_engine_reminder_next(const char *app_id, int idx)
{
    uint16_t r = 0xFFFF;
    lock();
    eng_app_t *a = find_app(app_id);
    if (a != NULL && idx >= 0 && idx < a->n_r)
    {
        uint16_t mod_now = (uint16_t)((wall_sec() / 60u) % 1440u);
        r = reminder_next_locked(&a->r[idx], mod_now);
    }
    unlock();
    return r;
}

/* ── fire execution (worker thread, NO lock held) ── */

/* pull display strings out of the stored package; every copy bounded */
static void fire_notify(const fire_item_t *f)
{
    char title[SKAIAPP_NAME_MAX] = "";
    char text[SKAIAPP_NOTIFY_MAX] = "";

    uint8_t *raw = NULL;
    uint32_t len = 0;
    if (skaiapp_store_load(f->app_id, &raw, &len) == 0)
    {
        cJSON *root = cJSON_ParseWithLength((const char *)raw, len);
        rt_free(raw);
        if (root != NULL)
        {
            const cJSON *jn = cJSON_GetObjectItem(root, "name");
            if (jn != NULL && cJSON_IsString(jn) && jn->valuestring != NULL)
            {
                strncpy(title, jn->valuestring, sizeof(title) - 1);
            }
            const cJSON *arr = cJSON_GetObjectItem(root,
                                                   f->kind ? "reminders" : "timers");
            const cJSON *e = (arr != NULL) ? cJSON_GetArrayItem(arr, f->idx) : NULL;
            if (e != NULL)
            {
                const cJSON *jt = NULL;
                if (f->kind)
                {
                    jt = cJSON_GetObjectItem(e, "message");
                }
                else
                {
                    const cJSON *jf = cJSON_GetObjectItem(e, "on_fire");
                    if (jf != NULL)
                    {
                        jt = cJSON_GetObjectItem(jf, "notify");
                    }
                }
                if (jt != NULL && cJSON_IsString(jt) && jt->valuestring != NULL)
                {
                    strncpy(text, jt->valuestring, sizeof(text) - 1);
                }
            }
            cJSON_Delete(root);
        }
    }
    if (title[0] == '\0')
    {
        strncpy(title, f->app_id, sizeof(title) - 1);
    }
    if (text[0] == '\0')
    {
        /* timer chains may be silent-by-design; only notify when there is text
           for reminders (message is schema-required, so this is a fallback) */
        if (f->kind == 0)
        {
            goto vibrate_only;
        }
        strncpy(text, title, sizeof(text) - 1);
    }

    {
        notification_t n;
        memset(&n, 0, sizeof(n));
        rt_snprintf(n.id, sizeof(n.id), "skaiapp:%s:%c%d",
                    f->app_id, f->kind ? 'r' : 't', f->idx);
        strncpy(n.title, title, sizeof(n.title) - 1);
        strncpy(n.message, text, sizeof(n.message) - 1);
        n.sec_time = wall_sec();
        n.type = Notify_others;
        n.state = true;
        /* OTA-hint precedent: stack notification_t from a non-LVGL thread */
        interact_with_notification(&n);
    }

vibrate_only:
    if (f->vibrate)
    {
        if (f->kind)
        {
            motor_pattern_notification();
        }
        else
        {
            motor_pattern_timer_reminder();
        }
    }
    LOG_I("fired %s %c%d", f->app_id, f->kind ? 'r' : 't', f->idx);
}

/* ── worker thread ── */

static int gather_due_timers(fire_item_t *q, int cap)
{
    int n = 0;
    lock();
    for (int ai = 0; ai < SKAIAPP_MAX_APPS; ai++)
    {
        eng_app_t *a = &s_apps[ai];
        if (!a->used)
        {
            continue;
        }
        for (int ti = 0; ti < a->n_t && n < cap; ti++)
        {
            eng_timer_t *t = &a->t[ti];
            if (t->state == 1 && timer_live_remaining(t) == 0)
            {
                t->state = 3;
                t->remaining_ms = 0;
                if (t->flags & (SKAIAPP_TF_HAS_NOTIFY | SKAIAPP_TF_VIBRATE))
                {
                    strncpy(q[n].app_id, a->app_id, SKAIAPP_ID_MAX - 1);
                    q[n].app_id[SKAIAPP_ID_MAX - 1] = '\0';
                    q[n].kind = 0;
                    q[n].idx = (uint8_t)ti;
                    q[n].vibrate = (t->flags & SKAIAPP_TF_VIBRATE) ? 1 : 0;
                    n++;
                }
                if (t->next >= 0 && t->next < a->n_t)
                {
                    eng_timer_t *nx = &a->t[t->next];
                    nx->remaining_ms = nx->duration_s * 1000u;
                    if (t->flags & SKAIAPP_TF_AUTOSTART_NEXT)
                    {
                        nx->run_base_ms = nx->remaining_ms;
                        nx->started_ms = now_ms();
                        nx->state = 1;
                    }
                    else
                    {
                        nx->state = 0;
                    }
                }
            }
        }
    }
    unlock();
    return n;
}

static bool reminder_due_at(const skaiapp_seed_rem_t *c, uint16_t mod)
{
    if (!c->enabled)
    {
        return false;
    }
    if (c->kind == 1)
    {
        for (int i = 0; i < c->n_times; i++)
        {
            if (c->times[i] == mod)
            {
                return true;
            }
        }
        return false;
    }
    uint16_t every = (c->every_min == 0) ? 60 : c->every_min;
    bool has_win = !(c->win_start == 0 && c->win_end == 0);
    uint16_t start = has_win ? c->win_start : 0;
    if (has_win)
    {
        bool in;
        if (c->win_start <= c->win_end)
        {
            in = (mod >= c->win_start && mod <= c->win_end);
        }
        else
        {
            in = (mod >= c->win_start || mod <= c->win_end);
        }
        if (!in)
        {
            return false;
        }
    }
    return (((uint16_t)((mod + 1440 - start) % 1440)) % every) == 0;
}

static int gather_due_reminders(fire_item_t *q, int cap)
{
    int n = 0;
    uint32_t cur_min = wall_sec() / 60u;
    lock();
    uint32_t from = s_last_eval_min;
    if (from == 0 || cur_min < from || cur_min - from > 240)
    {
        from = cur_min; /* first run or big clock jump: evaluate only "now" */
    }
    else
    {
        from = from + 1;
    }
    for (uint32_t m = from; m <= cur_min; m++)
    {
        uint16_t mod = (uint16_t)(m % 1440u);
        for (int ai = 0; ai < SKAIAPP_MAX_APPS; ai++)
        {
            eng_app_t *a = &s_apps[ai];
            if (!a->used)
            {
                continue;
            }
            for (int ri = 0; ri < a->n_r && n < cap; ri++)
            {
                eng_rem_t *r = &a->r[ri];
                if (r->last_fire_min == m)
                {
                    continue;
                }
                if (reminder_due_at(&r->cfg, mod))
                {
                    r->last_fire_min = m;
                    strncpy(q[n].app_id, a->app_id, SKAIAPP_ID_MAX - 1);
                    q[n].app_id[SKAIAPP_ID_MAX - 1] = '\0';
                    q[n].kind = 1;
                    q[n].idx = (uint8_t)ri;
                    q[n].vibrate = r->cfg.vibrate;
                    n++;
                }
            }
        }
    }
    s_last_eval_min = cur_min;
    unlock();
    return n;
}

/* how long may we sleep? min(next timer expiry, next minute) — capped */
static uint32_t next_delay_ms(void)
{
    uint32_t best = UINT32_MAX;
    bool any_rem = false;
    lock();
    for (int ai = 0; ai < SKAIAPP_MAX_APPS; ai++)
    {
        eng_app_t *a = &s_apps[ai];
        if (!a->used)
        {
            continue;
        }
        for (int ti = 0; ti < a->n_t; ti++)
        {
            if (a->t[ti].state == 1)
            {
                uint32_t rem = timer_live_remaining(&a->t[ti]);
                if (rem < best)
                {
                    best = rem;
                }
            }
        }
        for (int ri = 0; ri < a->n_r; ri++)
        {
            if (a->r[ri].cfg.enabled)
            {
                any_rem = true;
            }
        }
    }
    bool persist = s_persist_armed;
    uint32_t persist_due = s_persist_due_ms;
    unlock();

    if (any_rem)
    {
        uint32_t to_minute = (60u - (wall_sec() % 60u)) * 1000u;
        if (to_minute < best)
        {
            best = to_minute;
        }
    }
    if (persist)
    {
        uint32_t nowm = now_ms();
        uint32_t d = (persist_due > nowm) ? (persist_due - nowm) : 1;
        if (d < best)
        {
            best = d;
        }
    }
    if (best == UINT32_MAX)
    {
        return UINT32_MAX;
    }
    if (best < 50)
    {
        best = 50;
    }
    if (best > 60000)
    {
        best = 60000;
    }
    return best;
}

static void process_persist(void)
{
    char id[SKAIAPP_ID_MAX];
    uint8_t enabled[SKAIAPP_MAX_REMINDERS];
    uint8_t n = 0;
    bool go = false;
    lock();
    if (s_persist_armed && now_ms() >= s_persist_due_ms)
    {
        eng_app_t *a = find_app(s_persist_id);
        if (a != NULL)
        {
            strncpy(id, s_persist_id, SKAIAPP_ID_MAX);
            id[SKAIAPP_ID_MAX - 1] = '\0';
            n = a->n_r;
            for (int i = 0; i < n; i++)
            {
                enabled[i] = a->r[i].cfg.enabled;
            }
            go = true;
        }
        s_persist_armed = false;
    }
    unlock();
    if (go)
    {
        if (skaiapp_store_rewrite_reminder_enabled(id, enabled, n) != 0)
        {
            LOG_W("persist toggle for '%s' failed", id);
        }
    }
}

static void engine_thread(void *arg)
{
    (void)arg;
    /* FS + engine records: scan here, off the init call chain, once DFS is up */
    skaiapp_store_init();

    fire_item_t q[8];
    while (1)
    {
        uint32_t delay = next_delay_ms();
        rt_sem_take(s_sem, (delay == UINT32_MAX)
                    ? RT_WAITING_FOREVER
                    : rt_tick_from_millisecond(delay));
        int n = gather_due_timers(q, 8);
        for (int i = 0; i < n; i++)
        {
            fire_notify(&q[i]);
        }
        n = gather_due_reminders(q, 8);
        for (int i = 0; i < n; i++)
        {
            fire_notify(&q[i]);
        }
        process_persist();
    }
}

static int skaiapp_engine_init(void)
{
    s_mx = rt_mutex_create("skaiapp_en", RT_IPC_FLAG_PRIO);
    s_sem = rt_sem_create("skaiapp_en", 0, RT_IPC_FLAG_FIFO);
    memset(s_apps, 0, sizeof(s_apps));
    rt_thread_t th = rt_thread_create("skaiapp", engine_thread, RT_NULL,
                                      3072, 18, 10);
    if (th != RT_NULL)
    {
        rt_thread_startup(th);
    }
    else
    {
        LOG_E("engine thread create failed");
    }
    return 0;
}
INIT_APP_EXPORT(skaiapp_engine_init);
