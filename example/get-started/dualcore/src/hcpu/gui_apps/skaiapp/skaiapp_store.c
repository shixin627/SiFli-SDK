/**
 * @file   skaiapp_store.c
 * @brief  SkaiApp package persistence + resident meta table (see header).
 */
#include <string.h>
#include <stdio.h>
#include <rtthread.h>
#include <dfs_posix.h>
#include "cJSON.h"
#include "skaiapp_store.h"
#include "skaiapp_engine.h"

#define DBG_TAG "skaiapp.store"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define SKAIAPP_DIR "/skaiapp"

typedef struct
{
    bool    used;
    char    id[SKAIAPP_ID_MAX];
    char    name[SKAIAPP_NAME_MAX];
    uint8_t icon;
} meta_t;

static meta_t s_meta[SKAIAPP_MAX_APPS];
static rt_mutex_t s_mx = RT_NULL;
static volatile uint32_t s_gen = 0;
static char s_last[SKAIAPP_ID_MAX];
static rt_tick_t s_last_tick = 0;
static bool s_inited = false;

static void lock(void)   { if (s_mx) rt_mutex_take(s_mx, RT_WAITING_FOREVER); }
static void unlock(void) { if (s_mx) rt_mutex_release(s_mx); }
static bool id_path_safe(const char *id);

static void make_path(char *out, size_t cap, const char *id)
{
    rt_snprintf(out, cap, SKAIAPP_DIR "/%s.json", id);
}

static int meta_find(const char *id)
{
    for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
    {
        if (s_meta[i].used && strcmp(s_meta[i].id, id) == 0)
        {
            return i;
        }
    }
    return -1;
}

static int meta_free_slot(void)
{
    for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
    {
        if (!s_meta[i].used)
        {
            return i;
        }
    }
    return -1;
}

/* read one package file into an rt_malloc'd buffer; 0 = ok */
static int read_file(const char *path, uint8_t **out, uint32_t *out_len)
{
    int fd = open(path, O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        return -1;
    }
    uint8_t *buf = rt_malloc(SKAIAPP_PKG_MAX_BYTES + 1);
    if (buf == NULL)
    {
        close(fd);
        return -2;
    }
    int n = read(fd, buf, SKAIAPP_PKG_MAX_BYTES + 1);
    close(fd);
    if (n <= 0 || n > SKAIAPP_PKG_MAX_BYTES)
    {
        rt_free(buf);
        return -3;
    }
    buf[n] = '\0';
    *out = buf;
    *out_len = (uint32_t)n;
    return 0;
}

static int write_file(const char *path, const uint8_t *data, uint32_t len)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY);
    if (fd < 0)
    {
        return -1;
    }
    int n = write(fd, data, len);
    close(fd);
    return (n == (int)len) ? 0 : -2;
}

#if defined(BSP_USING_PC_SIMULATOR)
/* The PC sim mounts no writable FS (empty ptab). Keep the raw package bytes in
   a RAM slab so the whole install→launcher→render→engine pipeline is testable
   headless; real hardware uses the /skaiapp FS region and never compiles this. */
static uint8_t *s_sim_raw[SKAIAPP_MAX_APPS];
static uint32_t s_sim_len[SKAIAPP_MAX_APPS];
static char s_sim_id[SKAIAPP_MAX_APPS][SKAIAPP_ID_MAX];

static int sim_ram_put(const char *id, const uint8_t *data, uint32_t len)
{
    int slot = -1;
    for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
    {
        if (s_sim_raw[i] != NULL && strcmp(s_sim_id[i], id) == 0) { slot = i; break; }
    }
    if (slot < 0)
    {
        for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
        {
            if (s_sim_raw[i] == NULL) { slot = i; break; }
        }
    }
    if (slot < 0) { return -1; }
    if (s_sim_raw[slot] != NULL) { rt_free(s_sim_raw[slot]); }
    s_sim_raw[slot] = rt_malloc(len + 1);
    if (s_sim_raw[slot] == NULL) { return -1; }
    memcpy(s_sim_raw[slot], data, len);
    s_sim_raw[slot][len] = '\0';
    s_sim_len[slot] = len;
    strncpy(s_sim_id[slot], id, SKAIAPP_ID_MAX - 1);
    s_sim_id[slot][SKAIAPP_ID_MAX - 1] = '\0';
    return 0;
}
#endif

/* parse `raw` and (on success) register meta + engine record. */
static int adopt_package(const uint8_t *raw, uint32_t len, bool write_to_fs,
                         char out_id[SKAIAPP_ID_MAX])
{
    skaiapp_model_t *m = rt_malloc(sizeof(skaiapp_model_t));
    if (m == NULL)
    {
        return SKAIAPP_ACK_STORAGE;
    }
    skaiapp_eng_seed_t seed;
    int pr = skaiapp_pkg_parse(raw, len, m, &seed);
    if (pr != 0)
    {
        rt_free(m);
        return (pr == -2) ? SKAIAPP_ACK_UNSUPPORTED : SKAIAPP_ACK_PARSE;
    }

    lock();
    int slot = meta_find(m->id);
    if (slot < 0)
    {
        slot = meta_free_slot();
    }
    if (slot < 0)
    {
        unlock();
        rt_free(m);
        return SKAIAPP_ACK_LIMIT;
    }
    if (write_to_fs)
    {
        char path[64];
        make_path(path, sizeof(path), m->id);
        if (write_file(path, raw, len) != 0)
        {
#if defined(BSP_USING_PC_SIMULATOR)
            /* no writable FS on sim → RAM-back it so render/engine still run */
            if (sim_ram_put(m->id, raw, len) != 0)
            {
                unlock();
                rt_free(m);
                return SKAIAPP_ACK_STORAGE;
            }
#else
            unlock();
            LOG_E("write %s failed", path);
            rt_free(m);
            return SKAIAPP_ACK_STORAGE;
#endif
        }
    }
    s_meta[slot].used = true;
    strncpy(s_meta[slot].id, m->id, SKAIAPP_ID_MAX - 1);
    s_meta[slot].id[SKAIAPP_ID_MAX - 1] = '\0';
    strncpy(s_meta[slot].name, m->name, SKAIAPP_NAME_MAX - 1);
    s_meta[slot].name[SKAIAPP_NAME_MAX - 1] = '\0';
    s_meta[slot].icon = m->icon;
    if (write_to_fs) /* only real installs count as "fresh", not boot rescans */
    {
        strncpy(s_last, m->id, SKAIAPP_ID_MAX - 1);
        s_last[SKAIAPP_ID_MAX - 1] = '\0';
        s_last_tick = rt_tick_get();
    }
    if (out_id != NULL)
    {
        strncpy(out_id, m->id, SKAIAPP_ID_MAX - 1);
        out_id[SKAIAPP_ID_MAX - 1] = '\0';
    }
    unlock();

    skaiapp_engine_load(m->id, &seed);
    s_gen++;
    LOG_I("skaiapp '%s' (%s) ready, timers=%d reminders=%d items=%d",
          m->id, m->name, m->n_timers, m->n_reminders, m->n_items);
    rt_free(m);
    return SKAIAPP_ACK_OK;
}

void skaiapp_store_init(void)
{
    if (s_inited)
    {
        return;
    }
    s_inited = true;
    if (s_mx == RT_NULL)
    {
        s_mx = rt_mutex_create("skaiapp_st", RT_IPC_FLAG_PRIO);
    }
    memset(s_meta, 0, sizeof(s_meta));
    s_last[0] = '\0';

    if (mkdir(SKAIAPP_DIR, 0x777) != 0)
    {
        /* EEXIST is the normal case after first boot */
    }
    DIR *dir = opendir(SKAIAPP_DIR);
    if (dir == NULL)
    {
        LOG_W("opendir " SKAIAPP_DIR " failed (fs not ready?)");
        return;
    }
    struct dirent *ent;
    int adopted = 0;
    while ((ent = readdir(dir)) != NULL && adopted < SKAIAPP_MAX_APPS)
    {
        size_t n = strlen(ent->d_name);
        if (n < 6 || strcmp(&ent->d_name[n - 5], ".json") != 0)
        {
            continue;
        }
        char path[64];
        rt_snprintf(path, sizeof(path), SKAIAPP_DIR "/%s", ent->d_name);
        uint8_t *raw = NULL;
        uint32_t len = 0;
        if (read_file(path, &raw, &len) != 0)
        {
            LOG_W("scan: read %s failed", path);
            continue;
        }
        if (adopt_package(raw, len, false, NULL) == SKAIAPP_ACK_OK)
        {
            adopted++;
        }
        else
        {
            LOG_W("scan: %s rejected", path);
        }
        rt_free(raw);
    }
    closedir(dir);
    LOG_I("boot scan: %d skaiapp(s)", adopted);
}

int skaiapp_store_count(void)
{
    int c = 0;
    lock();
    for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
    {
        if (s_meta[i].used)
        {
            c++;
        }
    }
    unlock();
    return c;
}

bool skaiapp_store_meta(int idx, char id[SKAIAPP_ID_MAX],
                        char name[SKAIAPP_NAME_MAX], uint8_t *icon)
{
    bool ok = false;
    int seen = 0;
    lock();
    for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
    {
        if (!s_meta[i].used)
        {
            continue;
        }
        if (seen++ == idx)
        {
            if (id != NULL)
            {
                strncpy(id, s_meta[i].id, SKAIAPP_ID_MAX);
            }
            if (name != NULL)
            {
                strncpy(name, s_meta[i].name, SKAIAPP_NAME_MAX);
            }
            if (icon != NULL)
            {
                *icon = s_meta[i].icon;
            }
            ok = true;
            break;
        }
    }
    unlock();
    return ok;
}

bool skaiapp_store_exists(const char *id)
{
    lock();
    bool ok = (id != NULL && meta_find(id) >= 0);
    unlock();
    return ok;
}

int skaiapp_store_install(const uint8_t *json, uint32_t len,
                          char out_id[SKAIAPP_ID_MAX])
{
    if (json == NULL || len == 0 || len > SKAIAPP_PKG_MAX_BYTES)
    {
        return SKAIAPP_ACK_PARSE;
    }
    return adopt_package(json, len, true, out_id);
}

int skaiapp_store_remove(const char *id)
{
    if (id == NULL || id[0] == '\0')
    {
        return SKAIAPP_ACK_PARSE;
    }
    lock();
    int slot = meta_find(id);
    if (slot >= 0)
    {
        char path[64];
        make_path(path, sizeof(path), id);
        unlink(path);
        s_meta[slot].used = false;
    }
    unlock();
    if (slot >= 0)
    {
        skaiapp_engine_unload(id);
        s_gen++;
        LOG_I("skaiapp '%s' removed", id);
    }
    return SKAIAPP_ACK_OK; /* idempotent */
}

int skaiapp_store_load(const char *id, uint8_t **buf, uint32_t *len)
{
    if (id == NULL || buf == NULL || len == NULL || !id_path_safe(id))
    {
        return -1;
    }
#if defined(BSP_USING_PC_SIMULATOR)
    for (int i = 0; i < SKAIAPP_MAX_APPS; i++)
    {
        if (s_sim_raw[i] != NULL && strcmp(s_sim_id[i], id) == 0)
        {
            uint8_t *b = rt_malloc(s_sim_len[i] + 1);
            if (b == NULL) { return -2; }
            memcpy(b, s_sim_raw[i], s_sim_len[i] + 1);
            *buf = b;
            *len = s_sim_len[i];
            return 0;
        }
    }
#endif
    char path[64];
    make_path(path, sizeof(path), id);
    return read_file(path, buf, len);
}

/* local filename-charset guard (mirror of the parser's id_ok) */
static bool id_path_safe(const char *id)
{
    size_t n = strlen(id);
    if (n == 0 || n >= SKAIAPP_ID_MAX)
    {
        return false;
    }
    for (size_t i = 0; i < n; i++)
    {
        char c = id[i];
        bool alnum = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
        if (!(alnum || (i > 0 && c == '-')))
        {
            return false;
        }
    }
    return true;
}

int skaiapp_store_rewrite_reminder_enabled(const char *id,
                                           const uint8_t enabled[SKAIAPP_MAX_REMINDERS],
                                           uint8_t n)
{
    uint8_t *raw = NULL;
    uint32_t len = 0;
    if (skaiapp_store_load(id, &raw, &len) != 0)
    {
        return -1;
    }
    int ret = -2;
    cJSON *root = cJSON_ParseWithLength((const char *)raw, len);
    rt_free(raw);
    if (root == NULL)
    {
        return -2;
    }
    cJSON *arr = cJSON_GetObjectItem(root, "reminders");
    if (arr != NULL && cJSON_IsArray(arr))
    {
        uint8_t i = 0;
        cJSON *e = NULL;
        cJSON_ArrayForEach(e, arr)
        {
            if (i >= n || i >= SKAIAPP_MAX_REMINDERS)
            {
                break;
            }
            cJSON *je = cJSON_GetObjectItem(e, "enabled");
            if (je != NULL)
            {
                cJSON_SetBoolValue(je, enabled[i] ? 1 : 0);
            }
            else
            {
                cJSON_AddBoolToObject(e, "enabled", enabled[i] ? 1 : 0);
            }
            i++;
        }
        char *printed = cJSON_PrintUnformatted(root);
        if (printed != NULL)
        {
            size_t plen = strlen(printed);
            if (plen > 0 && plen <= SKAIAPP_PKG_MAX_BYTES)
            {
                char path[64];
                make_path(path, sizeof(path), id);
                ret = write_file(path, (const uint8_t *)printed, (uint32_t)plen);
#if defined(BSP_USING_PC_SIMULATOR)
                if (ret != 0) { ret = sim_ram_put(id, (const uint8_t *)printed, (uint32_t)plen); }
#endif
            }
            cJSON_free(printed);
        }
    }
    cJSON_Delete(root);
    return ret;
}

uint32_t skaiapp_store_generation(void)
{
    return s_gen;
}

bool skaiapp_store_last_installed(char out_id[SKAIAPP_ID_MAX])
{
    lock();
    strncpy(out_id, s_last, SKAIAPP_ID_MAX);
    out_id[SKAIAPP_ID_MAX - 1] = '\0';
    bool fresh = (s_last[0] != '\0') &&
                 ((rt_tick_get() - s_last_tick) < (rt_tick_t)(30 * RT_TICK_PER_SECOND));
    unlock();
    return fresh;
}
