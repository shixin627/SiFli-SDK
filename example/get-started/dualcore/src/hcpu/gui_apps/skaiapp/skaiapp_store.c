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
#define SKAIAPP_IMG_DIR SKAIAPP_DIR "/img"

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

/*
 * Drop the pictures of app `app_id` that no longer belong to it.
 *
 * Picture files are named `<appId>-<photoId>-<contentHash>.bin` by the phone
 * (SkaiappPhoto), so "still in use" is exactly "named in the package we just
 * installed". Anything else with this app's prefix is a picture the user
 * replaced — nobody else will ever delete it, and the watch's flash is small.
 * `m` NULL = the app is being removed, so every one of its pictures goes.
 *
 * One readdir per install/remove, off the LVGL thread (this runs on the BLE
 * parse thread): rare, bounded, and no file is opened — only stat-free unlink
 * of names we already matched. (The bot-avatar incident taught us never to put
 * unthrottled filesystem work on a path that repeats; this one does not.)
 */
static void sweep_photos(const char *app_id, const skaiapp_model_t *m)
{
    if (app_id == NULL || app_id[0] == '\0')
    {
        return;
    }
    DIR *dir = opendir(SKAIAPP_IMG_DIR);
    if (dir == NULL)
    {
        return; /* no pictures ever transferred — nothing to sweep */
    }
    size_t id_len = strlen(app_id);
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL)
    {
        /* only this app's files: "<appId>-..." */
        if (strncmp(ent->d_name, app_id, id_len) != 0 || ent->d_name[id_len] != '-')
        {
            continue;
        }
        bool referenced = false;
        if (m != NULL)
        {
            for (uint8_t i = 0; i < m->n_photos && !referenced; i++)
            {
                const char *src = m->photo_src[i];
                const char *slash = strrchr(src, '/');
                const char *base = (slash != NULL) ? slash + 1 : src;
                if (base[0] != '\0' && strcmp(base, ent->d_name) == 0)
                {
                    referenced = true;
                }
            }
        }
        if (referenced)
        {
            continue;
        }
        char path[SKAIAPP_PATH_MAX];
        rt_snprintf(path, sizeof(path), SKAIAPP_IMG_DIR "/%s", ent->d_name);
        if (unlink(path) == 0)
        {
            LOG_I("swept old picture %s", ent->d_name);
        }
    }
    closedir(dir);
}

/*
 * Keep the pictures the user chose ON THE WATCH when the phone re-pushes the
 * same app.
 *
 * A re-push is how every edit works ("make the title bigger"), and the phone's
 * copy of the package has no idea which album picture the user picked here — so
 * without this, changing anything about a barcode app silently empties its
 * frame and the user has to pick again. Only slots the incoming package leaves
 * EMPTY are carried over, so the phone can still set or clear a picture
 * deliberately.
 *
 * `raw`/`len` are the incoming document; returns a rewritten rt_malloc'd copy
 * (caller frees) or NULL when there is nothing to carry, in which case the
 * caller keeps using the original.
 */
static uint8_t *carry_over_photos(const char *id, const uint8_t *raw, uint32_t len,
                                  uint32_t *out_len)
{
    uint8_t *prev = NULL;
    uint32_t prev_len = 0;
    if (skaiapp_store_load(id, &prev, &prev_len) != 0)
    {
        return NULL; /* first install — nothing to keep */
    }
    cJSON *old_root = cJSON_ParseWithLength((const char *)prev, prev_len);
    rt_free(prev);
    if (old_root == NULL)
    {
        return NULL;
    }
    cJSON *new_root = cJSON_ParseWithLength((const char *)raw, len);
    if (new_root == NULL)
    {
        cJSON_Delete(old_root);
        return NULL;
    }
    cJSON *old_arr = cJSON_GetObjectItem(old_root, "photos");
    cJSON *new_arr = cJSON_GetObjectItem(new_root, "photos");
    bool changed = false;
    if (cJSON_IsArray(old_arr) && cJSON_IsArray(new_arr))
    {
        cJSON *ne = NULL;
        cJSON_ArrayForEach(ne, new_arr)
        {
            const cJSON *nid = cJSON_GetObjectItem(ne, "id");
            cJSON *nsrc = cJSON_GetObjectItem(ne, "src"); /* written below when present */
            if (!cJSON_IsString(nid) ||
                (cJSON_IsString(nsrc) && nsrc->valuestring[0] != '\0'))
            {
                continue; /* the phone stated a picture for this slot — respect it */
            }
            cJSON *oe = NULL;
            cJSON_ArrayForEach(oe, old_arr)
            {
                const cJSON *oid = cJSON_GetObjectItem(oe, "id");
                const cJSON *osrc = cJSON_GetObjectItem(oe, "src");
                if (cJSON_IsString(oid) && strcmp(oid->valuestring, nid->valuestring) == 0 &&
                    cJSON_IsString(osrc) && osrc->valuestring[0] != '\0')
                {
                    if (nsrc != NULL)
                    {
                        cJSON_SetValuestring(nsrc, osrc->valuestring);
                    }
                    else
                    {
                        cJSON_AddStringToObject(ne, "src", osrc->valuestring);
                    }
                    changed = true;
                    break;
                }
            }
        }
    }
    cJSON_Delete(old_root);
    uint8_t *merged = NULL;
    if (changed)
    {
        char *printed = cJSON_PrintUnformatted(new_root);
        if (printed != NULL)
        {
            size_t plen = strlen(printed);
            if (plen > 0 && plen <= SKAIAPP_PKG_MAX_BYTES)
            {
                merged = rt_malloc(plen + 1);
                if (merged != NULL)
                {
                    memcpy(merged, printed, plen + 1);
                    *out_len = (uint32_t)plen;
                }
            }
            cJSON_free(printed);
        }
    }
    cJSON_Delete(new_root);
    return merged;
}

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
        /* Keep any album picture the user chose on the WATCH for a slot this
           package leaves empty — an edit must not empty the frame. */
        const uint8_t *doc = raw;
        uint32_t doc_len = len;
        uint8_t *merged = NULL;
        if (m->n_photos > 0)
        {
            uint32_t merged_len = 0;
            merged = carry_over_photos(m->id, raw, len, &merged_len);
            if (merged != NULL)
            {
                doc = merged;
                doc_len = merged_len;
            }
        }
        if (write_file(path, doc, doc_len) != 0)
        {
#if defined(BSP_USING_PC_SIMULATOR)
            /* no writable FS on sim → RAM-back it so render/engine still run */
            if (sim_ram_put(m->id, doc, doc_len) != 0)
            {
                unlock();
                rt_free(merged);
                rt_free(m);
                return SKAIAPP_ACK_STORAGE;
            }
#else
            unlock();
            LOG_E("write %s failed", path);
            rt_free(merged);
            rt_free(m);
            return SKAIAPP_ACK_STORAGE;
#endif
        }
        rt_free(merged);
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
    if (write_to_fs) /* a real install: the package just named its pictures */
    {
        sweep_photos(m->id, m);
    }
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
        sweep_photos(id, NULL); /* the app is gone; so are its pictures */
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

int skaiapp_store_set_photo_src(const char *id, uint8_t idx, const char *path)
{
    if (id == NULL || path == NULL || idx >= SKAIAPP_MAX_PHOTOS)
    {
        return -1;
    }
    /* The only writer of this field on the watch is a tap on the picker, and the
       only thing it may name is a file in the watch's own album. Bounding it here
       keeps a malformed package (or a future caller) from pointing a slot at an
       arbitrary path. */
    if (strncmp(path, "/photo/", 7) != 0 || strlen(path) >= SKAIAPP_PATH_MAX)
    {
        LOG_W("photo pick rejected: %s", path);
        return -1;
    }
    uint8_t *raw = NULL;
    uint32_t len = 0;
    if (skaiapp_store_load(id, &raw, &len) != 0)
    {
        return -1;
    }
    cJSON *root = cJSON_ParseWithLength((const char *)raw, len);
    rt_free(raw);
    if (root == NULL)
    {
        return -2;
    }
    int ret = -2;
    cJSON *arr = cJSON_GetObjectItem(root, "photos");
    if (arr != NULL && cJSON_IsArray(arr))
    {
        cJSON *e = cJSON_GetArrayItem(arr, (int)idx);
        if (e != NULL)
        {
            cJSON *js = cJSON_GetObjectItem(e, "src");
            if (js != NULL)
            {
                cJSON_SetValuestring(js, path);
            }
            else
            {
                cJSON_AddStringToObject(e, "src", path);
            }
            char *printed = cJSON_PrintUnformatted(root);
            if (printed != NULL)
            {
                size_t plen = strlen(printed);
                if (plen > 0 && plen <= SKAIAPP_PKG_MAX_BYTES)
                {
                    char fpath[64];
                    make_path(fpath, sizeof(fpath), id);
                    ret = write_file(fpath, (const uint8_t *)printed, (uint32_t)plen);
#if defined(BSP_USING_PC_SIMULATOR)
                    if (ret != 0) { ret = sim_ram_put(id, (const uint8_t *)printed, (uint32_t)plen); }
#endif
                }
                cJSON_free(printed);
            }
        }
    }
    cJSON_Delete(root);
    if (ret == 0)
    {
        s_gen++; /* the host app's tick redraws the open page with the picture */
        LOG_I("skaiapp '%s' photo %u = %s", id, idx, path);
    }
    return ret;
}

int skaiapp_store_rewrite_state(const char *id,
                                const uint8_t enabled[SKAIAPP_MAX_REMINDERS],
                                uint8_t n,
                                const int32_t values[SKAIAPP_MAX_VARS],
                                uint8_t n_values)
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
    }
    /* Counters: write the live value into `val`, never over `init` — a re-push
       from the phone must still be able to reset the app to its starting state. */
    cJSON *varr = cJSON_GetObjectItem(root, "vars");
    if (varr != NULL && cJSON_IsArray(varr))
    {
        uint8_t i = 0;
        cJSON *e = NULL;
        cJSON_ArrayForEach(e, varr)
        {
            if (i >= n_values || i >= SKAIAPP_MAX_VARS)
            {
                break;
            }
            cJSON *jv = cJSON_GetObjectItem(e, "val");
            if (jv != NULL)
            {
                cJSON_SetNumberValue(jv, (double)values[i]);
            }
            else
            {
                cJSON_AddNumberToObject(e, "val", (double)values[i]);
            }
            i++;
        }
    }
    {
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
