/**
 ******************************************************************************
 * @file   bloc_health.c
 * @author Skaiwalk software development team
 * @brief  Per-day JSON persistence for sleep / heart-rate data so it survives
 *         BLE disconnection (store-and-forward). Mirrors store_exercise_data().
 ******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <rtthread.h>
#include <dfs_posix.h>
#include <sys/stat.h>
#include "cJSON.h"
#include "bloc_health.h"
#include "communicate_task.h" /* commu_send_heart_curve_sample / _skip (replay) */
#include "watch_global_data.h"
#include "watch_sys_service.h" /* WATCH_SYS_WORN_* tri-state */

#define DBG_TAG "bloc.health"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

#define HEALTH_DIR "/health"

/* Read + parse a JSON file, or NULL if missing/empty/invalid. Caller frees
   the returned cJSON with cJSON_Delete(). Same shape as the exercise reader. */
static cJSON *read_json_file(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0 || st.st_size <= 0)
    {
        return NULL;
    }

    FILE *fp = fopen(path, "r");
    if (fp == NULL)
    {
        return NULL;
    }

    char *buf = rt_malloc(st.st_size + 1);
    if (buf == NULL)
    {
        fclose(fp);
        return NULL;
    }

    size_t read_size = fread(buf, 1, st.st_size, fp);
    buf[read_size] = '\0';
    fclose(fp);

    cJSON *root = cJSON_Parse(buf);
    rt_free(buf);
    return root;
}

/* Overwrite path with the pretty-printed JSON. Returns 0 on success. */
static int write_json_file(const char *path, cJSON *root)
{
    char *json_str = cJSON_Print(root);
    if (json_str == NULL)
    {
        return -1;
    }

    FILE *fp = fopen(path, "w");
    if (fp == NULL)
    {
        cJSON_free(json_str);
        return -1;
    }

    fwrite(json_str, strlen(json_str), 1, fp);
    fclose(fp);
    cJSON_free(json_str);
    return 0;
}

void store_sleep_data(const watch_sys_sleep_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    watch_storage_api_lock();

    time_t now;
    struct tm *tm_info;
    char filename[40];

    time(&now);
    tm_info = localtime(&now);

    mkdir(HEALTH_DIR, 0x777);

    /* One file per day; the sleep_state is a cumulative daily aggregate, so we
       overwrite with the latest snapshot rather than append. */
    snprintf(filename, sizeof(filename),
             HEALTH_DIR "/sleep_%04d%02d%02d.json", tm_info->tm_year + 1900,
             tm_info->tm_mon + 1, tm_info->tm_mday);

    /* Per-day cumulative aggregate: take the per-field MAX against the existing
       file instead of blindly overwriting. sleep_fusion's total is NOT persisted,
       so a mid-night watch reboot zeroes it and it re-accumulates from 0 — a blind
       overwrite would then let that small post-reboot value clobber the real total
       (this is the 462→60 bug). MAX keeps the daily total monotonic within the
       local day; the file is already per-day so this never bleeds across days. A
       genuine local-midnight reset starts a NEW day file, so it isn't max-pinned. */
    uint16_t old_total = 0, old_deep = 0, old_rem = 0, old_light = 0, old_waso = 0;
    {
        cJSON *existing = read_json_file(filename);
        if (existing != NULL)
        {
            cJSON *j;
            if ((j = cJSON_GetObjectItem(existing, "total_min")) != NULL) old_total = (uint16_t)j->valueint;
            if ((j = cJSON_GetObjectItem(existing, "deep_min"))  != NULL) old_deep  = (uint16_t)j->valueint;
            if ((j = cJSON_GetObjectItem(existing, "rem_min"))   != NULL) old_rem   = (uint16_t)j->valueint;
            if ((j = cJSON_GetObjectItem(existing, "light_min")) != NULL) old_light = (uint16_t)j->valueint;
            if ((j = cJSON_GetObjectItem(existing, "waso_min"))  != NULL) old_waso  = (uint16_t)j->valueint;
            cJSON_Delete(existing);
        }
    }
    uint16_t max_total = state->total_sleep_min > old_total ? state->total_sleep_min : old_total;
    uint16_t max_deep  = state->deep_min        > old_deep  ? state->deep_min        : old_deep;
    uint16_t max_rem   = state->rem_min         > old_rem   ? state->rem_min         : old_rem;
    uint16_t max_light = state->light_min       > old_light ? state->light_min       : old_light;
    uint16_t max_waso  = state->awake_after_onset_min > old_waso ? state->awake_after_onset_min : old_waso;

    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        watch_storage_api_unlock();
        return;
    }

    /* Raw values; the phone derives its local day bucket from ts_utc. */
    cJSON_AddNumberToObject(root, "ts_utc", (double)state->timestamp_utc);
    cJSON_AddNumberToObject(root, "stage", state->mode);
    cJSON_AddNumberToObject(root, "total_min", max_total);
    cJSON_AddNumberToObject(root, "deep_min", max_deep);
    cJSON_AddNumberToObject(root, "rem_min", max_rem);
    cJSON_AddNumberToObject(root, "light_min", max_light);
    cJSON_AddNumberToObject(root, "waso_min", max_waso);
    cJSON_AddNumberToObject(root, "hr", state->current_hr);
    cJSON_AddNumberToObject(root, "rhr", state->resting_hr);

    if (write_json_file(filename, root) != 0)
    {
        LOG_E("store_sleep_data: write %s failed", filename);
    }
    else
    {
        LOG_D("store_sleep_data: %s", filename);
    }

    cJSON_Delete(root);
    rt_thread_mdelay(50);
    watch_storage_api_unlock();
}

void store_hr_sample(uint32_t timestamp_utc, uint8_t bpm, uint8_t reason)
{
    watch_storage_api_lock();

    time_t t = (time_t)timestamp_utc;
    struct tm *tm_info = localtime(&t);
    char filename[40];

    mkdir(HEALTH_DIR, 0x777);

    snprintf(filename, sizeof(filename), HEALTH_DIR "/hr_%04d%02d%02d.json",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday);

    /* Append one {ts,bpm} point to the day's "samples" array. */
    cJSON *root = read_json_file(filename);
    cJSON *samples = NULL;
    if (root != NULL)
    {
        samples = cJSON_GetObjectItem(root, "samples");
        if (samples == NULL || !cJSON_IsArray(samples))
        {
            cJSON_DeleteItemFromObject(root, "samples");
            samples = cJSON_CreateArray();
            cJSON_AddItemToObject(root, "samples", samples);
        }
    }
    else
    {
        root = cJSON_CreateObject();
        samples = cJSON_CreateArray();
        cJSON_AddItemToObject(root, "samples", samples);
    }

    cJSON *item = cJSON_CreateObject();
    cJSON_AddNumberToObject(item, "ts", (double)timestamp_utc);
    cJSON_AddNumberToObject(item, "bpm", bpm);
    /* reason>0 marks a 5-min bucket with NO HR point (bpm is 0); the phone
       draws it as a coloured gap label. reason==0 is a normal HR sample. */
    if (reason != 0)
        cJSON_AddNumberToObject(item, "reason", reason);
    cJSON_AddItemToArray(samples, item);

    if (write_json_file(filename, root) != 0)
    {
        LOG_E("store_hr_sample: write %s failed", filename);
    }

    cJSON_Delete(root);
    rt_thread_mdelay(50);
    watch_storage_api_unlock();
}

/*============================================================================*
 *   Store-and-forward READ side: replay stored HR points to the phone
 *============================================================================*/

/* How far back a reconnecting phone can pull. Retention keeps 14 days, but a
   phone that has been away that long is better served by a fresh curve than by
   a multi-thousand-frame replay; 3 days covers the realistic cases (a night
   with BLE down, a flat phone, a weekend away from the app). */
#define HR_BACKFILL_MAX_DAYS   3
/* Hard ceiling on frames per request. At the ~10 min sleep cadence 3 days is
   ~430 points; 600 leaves headroom for denser daytime stretches without ever
   turning a malformed request into an unbounded send loop. */
#define HR_BACKFILL_MAX_FRAMES 600
/* Gap between frames. The BLE notify queue silently drops when saturated (the
   battery-percent stall had the same root cause), so replay is paced rather
   than blasted: 600 frames * 30 ms = ~18 s worst case, all in the background. */
#define HR_BACKFILL_PACING_MS  30

/* Replay every stored point with ts > since_ts, oldest day first so the phone
   receives them in chronological order. Runs ON THE WORKER THREAD: it holds the
   storage lock only while reading a day file, never while sending, so a long
   replay cannot block the writers. */
static void backfill_hr_samples(uint32_t since_ts)
{
    uint32_t sent = 0;
    time_t now = time(NULL);

    for (int day_ago = HR_BACKFILL_MAX_DAYS - 1; day_ago >= 0; day_ago--)
    {
        time_t day_t = now - (time_t)day_ago * 24 * 60 * 60;
        struct tm *tm_info = localtime(&day_t);
        char filename[40];
        snprintf(filename, sizeof(filename), HEALTH_DIR "/hr_%04d%02d%02d.json",
                 tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday);

        /* Parse under the lock, send outside it. */
        watch_storage_api_lock();
        cJSON *root = read_json_file(filename);
        watch_storage_api_unlock();
        if (root == NULL)
        {
            continue; /* no data that day */
        }

        cJSON *samples = cJSON_GetObjectItem(root, "samples");
        if (cJSON_IsArray(samples))
        {
            int n = cJSON_GetArraySize(samples);
            for (int i = 0; i < n; i++)
            {
                cJSON *item = cJSON_GetArrayItem(samples, i);
                cJSON *jts = cJSON_GetObjectItem(item, "ts");
                if (!cJSON_IsNumber(jts))
                {
                    continue;
                }
                uint32_t ts = (uint32_t)jts->valuedouble;
                if (ts <= since_ts)
                {
                    continue; /* phone already has this one */
                }

                cJSON *jreason = cJSON_GetObjectItem(item, "reason");
                bool ok;
                if (cJSON_IsNumber(jreason) && jreason->valueint > 0)
                {
                    ok = commu_send_heart_curve_skip(ts, (uint8_t)jreason->valueint);
                }
                else
                {
                    cJSON *jbpm = cJSON_GetObjectItem(item, "bpm");
                    /* UNKNOWN, not "worn": the stored JSON has no wear column,
                       so a replayed point cannot honestly claim either way. */
                    ok = commu_send_heart_curve_sample(
                        ts, cJSON_IsNumber(jbpm) ? (uint8_t)jbpm->valueint : 0,
                        WATCH_SYS_WORN_UNKNOWN);
                }

                if (!ok)
                {
                    /* Link dropped again (commu_send_* refuses when not
                       connected) — abandon; the next reconnect asks again. */
                    LOG_W("hr backfill aborted after %u frames (link down)", sent);
                    cJSON_Delete(root);
                    return;
                }

                if (++sent >= HR_BACKFILL_MAX_FRAMES)
                {
                    LOG_W("hr backfill hit the %u-frame cap", (unsigned)sent);
                    cJSON_Delete(root);
                    return;
                }
                rt_thread_mdelay(HR_BACKFILL_PACING_MS);
            }
        }
        cJSON_Delete(root);
    }

    LOG_I("hr backfill done: %u frames since %u", sent, since_ts);
}

/*============================================================================*
 *   Off-thread worker: marshal writes off the BLE/RPC threads (avoid STKOF)
 *============================================================================*/

typedef enum
{
    HEALTH_MSG_SLEEP = 0,
    HEALTH_MSG_HR,
    HEALTH_MSG_HR_BACKFILL, /* replay stored HR points newer than .backfill_since */
} health_msg_type_t;

typedef struct
{
    health_msg_type_t type;
    union
    {
        watch_sys_sleep_state_t sleep;
        struct
        {
            uint32_t ts;
            uint8_t bpm;
            uint8_t reason; /* 0 = real HR sample; >0 = gap reason (bpm unused) */
        } hr;
        uint32_t backfill_since; /* newest ts the phone already holds (0 = none) */
    } data;
} health_msg_t;

static rt_mq_t health_mq = RT_NULL;

static void health_worker_entry(void *parameter)
{
    health_msg_t msg;
    (void)parameter;

    while (1)
    {
        if (rt_mq_recv(health_mq, &msg, sizeof(msg), RT_WAITING_FOREVER) !=
            RT_EOK)
        {
            continue;
        }

        switch (msg.type)
        {
        case HEALTH_MSG_SLEEP:
            store_sleep_data(&msg.data.sleep);
            break;
        case HEALTH_MSG_HR:
            store_hr_sample(msg.data.hr.ts, msg.data.hr.bpm, msg.data.hr.reason);
            break;
        case HEALTH_MSG_HR_BACKFILL:
            backfill_hr_samples(msg.data.backfill_since);
            break;
        default:
            break;
        }
    }
}

void health_store_sleep_async(const watch_sys_sleep_state_t *state)
{
    if (state == NULL || health_mq == RT_NULL)
    {
        return;
    }

    health_msg_t msg;
    msg.type = HEALTH_MSG_SLEEP;
    msg.data.sleep = *state;
    if (rt_mq_send(health_mq, &msg, sizeof(msg)) != RT_EOK)
    {
        LOG_W("health_mq full; sleep record dropped");
    }
}

void health_store_hr_async(uint32_t timestamp_utc, uint8_t bpm)
{
    if (health_mq == RT_NULL)
    {
        return;
    }

    health_msg_t msg;
    msg.type = HEALTH_MSG_HR;
    msg.data.hr.ts = timestamp_utc;
    msg.data.hr.bpm = bpm;
    msg.data.hr.reason = 0;
    if (rt_mq_send(health_mq, &msg, sizeof(msg)) != RT_EOK)
    {
        LOG_W("health_mq full; hr sample dropped");
    }
}

void health_backfill_hr_async(uint32_t since_ts)
{
    if (health_mq == RT_NULL)
    {
        return;
    }

    health_msg_t msg;
    msg.type = HEALTH_MSG_HR_BACKFILL;
    msg.data.backfill_since = since_ts;
    if (rt_mq_send(health_mq, &msg, sizeof(msg)) != RT_EOK)
    {
        LOG_W("health_mq full; hr backfill request dropped");
    }
}

void health_store_hr_skip_async(uint32_t timestamp_utc, uint8_t reason)
{
    if (health_mq == RT_NULL)
    {
        return;
    }

    /* A 5-min bucket that produced no HR point, persisted as {ts,bpm:0,reason}
       so the gap reason survives BLE disconnect (e.g. a night's sleep) and
       syncs alongside the HR samples it sits between. */
    health_msg_t msg;
    msg.type = HEALTH_MSG_HR;
    msg.data.hr.ts = timestamp_utc;
    msg.data.hr.bpm = 0;
    msg.data.hr.reason = reason;
    if (rt_mq_send(health_mq, &msg, sizeof(msg)) != RT_EOK)
    {
        LOG_W("health_mq full; hr skip dropped");
    }
}

/* Auto-init at app startup (mirrors file_system_task_init). The worker just
   blocks on the queue until the first record arrives, so filesystem mount
   ordering is not a concern. */
static int bloc_health_task_init(void)
{
    rt_thread_t thread;

    health_mq = rt_mq_create("health_mq", sizeof(health_msg_t), 16,
                             RT_IPC_FLAG_FIFO);
    if (health_mq == RT_NULL)
    {
        LOG_E("Failed to create health message queue");
        return -RT_ERROR;
    }

    thread = rt_thread_create("health_store", health_worker_entry, RT_NULL,
                              4096, 22, 10);
    if (thread == RT_NULL)
    {
        LOG_E("Failed to create health worker thread");
        rt_mq_delete(health_mq);
        health_mq = RT_NULL;
        return -RT_ERROR;
    }
    rt_thread_startup(thread);
    return RT_EOK;
}
INIT_APP_EXPORT(bloc_health_task_init);

/* Retention window for /health files. delete_after_sync stays false on flush,
   so a day survives (re-sent on each reconnect) until it ages out — no loss. */
#define HEALTH_RETENTION_DAYS 14

void health_cleanup_old_files(void)
{
    time_t cutoff_t = time(NULL) - (time_t)HEALTH_RETENTION_DAYS * 24 * 60 * 60;
    struct tm *c = localtime(&cutoff_t);
    int cutoff =
        (c->tm_year + 1900) * 10000 + (c->tm_mon + 1) * 100 + c->tm_mday;

    watch_storage_api_lock();

    DIR *dir = opendir(HEALTH_DIR);
    if (dir != NULL)
    {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL)
        {
            /* sleep_YYYYMMDD.json / hr_YYYYMMDD.json -> 8 digits after '_'.
               YYYYMMDD as an int compares chronologically. */
            const char *us = strrchr(ent->d_name, '_');
            if (us == NULL || strlen(us + 1) < 8)
            {
                continue;
            }

            char datebuf[9];
            memcpy(datebuf, us + 1, 8);
            datebuf[8] = '\0';

            int all_digits = 1;
            for (int i = 0; i < 8; i++)
            {
                if (datebuf[i] < '0' || datebuf[i] > '9')
                {
                    all_digits = 0;
                    break;
                }
            }
            if (!all_digits)
            {
                continue;
            }

            if (atoi(datebuf) < cutoff)
            {
                char path[64];
                snprintf(path, sizeof(path), HEALTH_DIR "/%s", ent->d_name);
                unlink(path);
                LOG_I("health: pruned old %s", ent->d_name);
            }
        }
        closedir(dir);
    }

    watch_storage_api_unlock();
}
