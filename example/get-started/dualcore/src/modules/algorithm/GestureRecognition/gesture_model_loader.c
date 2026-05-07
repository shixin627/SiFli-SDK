/**
 ******************************************************************************
 * @file   gesture_model_loader.c
 * @author Skaiwalk software development team
 * @brief  Dynamic model loader implementation for gesture recognition
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gesture_model_loader.h"
#include "gesture_detect_model_data.h"
#include "rtthread.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "bloc_filesystem.h"
#include "cJSON.h"
#include <unistd.h>

#define DBG_TAG "MODEL.LOADER"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* Global model structures */
static gesture_model_t release_model = {0};

/*============================================================================*
 *                           Manifest infrastructure
 *  Phone is the pusher; watch is the source of truth for "what's actually
 *  installed". Layout:
 *    /model/manifest.json  →  { "ver": 1,
 *                               "models": [ {name, version, size}, ... ] }
 *  Atomicity: write via .tmp + rename. Boot revalidates each entry's file
 *  exists with the exact size; bad entries are dropped (and the file deleted)
 *  so the loader's "fall back to embedded" path takes over.
 *============================================================================*/

#define MANIFEST_FILE_MAX_BYTES 4096  /* sanity cap when reading from flash */

typedef struct
{
    const char *name;             /* logical name used in manifest + protocol */
    const char *path;             /* on-disk path */
    const char *builtin_version;  /* reported when no install present */
} known_model_entry_t;

/* Single source of truth for "models the firmware knows how to load". Adding
   a new model here automatically wires it through manifest validation, the
   to-JSON response, and the SET protocol. */
static const known_model_entry_t s_known_models[] = {
    { MODEL_NAME_RELEASE, GESTURE_RELEASE_MODEL_PATH, GESTURE_RELEASE_BUILTIN_VERSION },
};
#define KNOWN_MODEL_COUNT (sizeof(s_known_models) / sizeof(s_known_models[0]))

static const known_model_entry_t *find_known_model(const char *name)
{
    if (!name) return NULL;
    for (size_t i = 0; i < KNOWN_MODEL_COUNT; i++)
    {
        if (strcmp(s_known_models[i].name, name) == 0)
        {
            return &s_known_models[i];
        }
    }
    return NULL;
}

/* Read manifest.json into a heap buffer. Caller frees with rt_free(). */
static char *manifest_read_raw(void)
{
    struct stat st;
    if (stat(MODEL_MANIFEST_PATH, &st) != 0) return NULL;
    if (st.st_size <= 0 || st.st_size > MANIFEST_FILE_MAX_BYTES) return NULL;

    char *buf = (char *)rt_malloc(st.st_size + 1);
    if (!buf) return NULL;

    int fd = open(MODEL_MANIFEST_PATH, O_RDONLY | O_BINARY);
    if (fd < 0) { rt_free(buf); return NULL; }

    int n = read(fd, buf, st.st_size);
    close(fd);

    if (n != st.st_size) { rt_free(buf); return NULL; }
    buf[n] = '\0';
    return buf;
}

/* Parse manifest from disk. NULL on missing/corrupt — callers treat that as
   "no installed models", which makes the loader fall back to embedded data. */
static cJSON *manifest_load_root(void)
{
    char *raw = manifest_read_raw();
    if (!raw) return NULL;
    cJSON *root = cJSON_Parse(raw);
    rt_free(raw);
    if (!root)
    {
        LOG_W("Manifest parse failed; treating as empty");
    }
    return root;
}

/* Atomic save: write to .tmp, then rename over the live file. */
static int manifest_save_root(const cJSON *root)
{
    char *out = cJSON_PrintUnformatted(root);
    if (!out) return -1;

    size_t len = strlen(out);
    int fd = open(MODEL_MANIFEST_TMP_PATH,
                  O_WRONLY | O_CREAT | O_TRUNC | O_BINARY, 0666);
    if (fd < 0)
    {
        cJSON_free(out);
        LOG_E("Manifest tmp open failed");
        return -2;
    }

    int n = write(fd, out, len);
    close(fd);
    cJSON_free(out);

    if (n != (int)len)
    {
        unlink(MODEL_MANIFEST_TMP_PATH);
        LOG_E("Manifest write short: %d/%u", n, (unsigned)len);
        return -3;
    }

    if (rename(MODEL_MANIFEST_TMP_PATH, MODEL_MANIFEST_PATH) != 0)
    {
        unlink(MODEL_MANIFEST_TMP_PATH);
        LOG_E("Manifest rename failed");
        return -4;
    }
    return 0;
}

/* Boot-time sweep: drop manifest entries whose file is missing or whose
   recorded size doesn't match what's actually on disk. Orphaned files are
   removed too so the next push starts clean. */
static void manifest_validate_at_boot(void)
{
    cJSON *root = manifest_load_root();
    if (!root) return;

    cJSON *models = cJSON_GetObjectItem(root, "models");
    if (!cJSON_IsArray(models))
    {
        cJSON_Delete(root);
        return;
    }

    bool dirty = false;
    cJSON *entry = models->child;
    while (entry)
    {
        cJSON *next = entry->next;
        bool drop = false;
        const char *drop_reason = "";
        const known_model_entry_t *km = NULL;

        cJSON *name_j = cJSON_GetObjectItem(entry, "name");
        cJSON *size_j = cJSON_GetObjectItem(entry, "size");

        if (!cJSON_IsString(name_j))
        {
            drop = true;
            drop_reason = "missing name";
        }
        else
        {
            km = find_known_model(name_j->valuestring);
            if (!km)
            {
                drop = true;
                drop_reason = "unknown model";
            }
            else
            {
                struct stat st;
                if (stat(km->path, &st) != 0)
                {
                    drop = true;
                    drop_reason = "file missing";
                }
                else if (cJSON_IsNumber(size_j) &&
                         (uint32_t)st.st_size !=
                             (uint32_t)size_j->valuedouble)
                {
                    drop = true;
                    drop_reason = "size mismatch";
                }
            }
        }

        if (drop)
        {
            LOG_W("Manifest drop %s: %s",
                  cJSON_IsString(name_j) ? name_j->valuestring : "(unnamed)",
                  drop_reason);
            if (km)
            {
                /* Best-effort cleanup of an orphan/corrupt model file. */
                unlink(km->path);
            }
            cJSON_DetachItemViaPointer(models, entry);
            cJSON_Delete(entry);
            dirty = true;
        }
        entry = next;
    }

    if (dirty)
    {
        manifest_save_root(root);
    }
    cJSON_Delete(root);
}

/**
 * @brief Check if a file exists
 * @param path File path to check
 * @return true if file exists, false otherwise
 */
static bool file_exists(const char *path)
{
    struct stat st;
    return (stat(path, &st) == 0);
}

/**
 * @brief Load model from file system
 * @param file_path Path to the model file
 * @param model Pointer to model structure
 * @return 0 on success, negative on error
 */
static int load_model_from_file(const char *file_path, gesture_model_t *model)
{
    int fd = -1;
    struct stat file_stat;
    size_t bytes_read;

    if (!file_path || !model)
    {
        LOG_E("Invalid parameters");
        return -1;
    }

    /* Check if file exists */
    if (!file_exists(file_path))
    {
        LOG_W("Model file not found: %s", file_path);
        return -2;
    }

    /* Get file size */
    if (stat(file_path, &file_stat) != 0)
    {
        LOG_E("Failed to get file stats: %s", file_path);
        return -3;
    }

    if (file_stat.st_size > MAX_MODEL_SIZE)
    {
        LOG_E("Model file too large: %d bytes (max: %d)", file_stat.st_size, MAX_MODEL_SIZE);
        return -4;
    }

    /* Allocate memory for model */
    model->data = (uint8_t *)rt_malloc(file_stat.st_size);
    if (!model->data)
    {
        LOG_E("Failed to allocate memory for model (%d bytes)", file_stat.st_size);
        return -5;
    }

    /* Open and read the model file */
    fd = open(file_path, O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        LOG_E("Failed to open model file: %s", file_path);
        rt_free(model->data);
        model->data = NULL;
        return -6;
    }

    bytes_read = read(fd, model->data, file_stat.st_size);
    close(fd);

    if (bytes_read != file_stat.st_size)
    {
        LOG_E("Failed to read model file completely: read %d/%d bytes", bytes_read, file_stat.st_size);
        rt_free(model->data);
        model->data = NULL;
        return -7;
    }

    model->size = file_stat.st_size;
    model->is_dynamic = true;
    model->is_loaded = true;

    LOG_I("Successfully loaded model from %s (%d bytes)", file_path, model->size);
    return 0;
}

/**
 * @brief Initialize the model loader
 * @return 0 on success, negative on error
 */
int gesture_model_loader_init(void)
{
    LOG_I("Initializing gesture model loader");

    /* Initialize model structures */
    memset(&release_model, 0, sizeof(gesture_model_t));

    /* Self-heal manifest before anyone calls load_*_model — a bad entry plus
       a partial file would otherwise look "installed" to the loader. */
    manifest_validate_at_boot();

    return 0;
}

/**
 * @brief Load release detection model from file system or use default
 * @param model Pointer to model structure to initialize
 * @return 0 on success, negative on error
 */
int load_release_model(gesture_model_t *model)
{
    int ret;

    if (!model)
    {
        model = &release_model;
    }

    /* If model is already loaded, return success */
    if (model->is_loaded)
    {
        LOG_D("Release model already loaded");
        return 0;
    }

    /* Try to load from file system first */
    ret = load_model_from_file(GESTURE_RELEASE_MODEL_PATH, model);
    if (ret == 0)
    {
        LOG_I("Using dynamic release model from file system");
        return 0;
    }

    /* Fall back to default embedded model */
    LOG_I("Using default embedded release model");
    model->data = (uint8_t *)g_gesture_detect_release_model_data;
    model->size = g_gesture_detect_release_model_data_len;
    model->is_dynamic = false;
    model->is_loaded = true;

    return 0;
}

/**
 * @brief Unload dynamically loaded model and free memory
 * @param model Pointer to model structure to unload
 * @return 0 on success, negative on error
 */
int unload_model(gesture_model_t *model)
{
    /* Only free dynamically allocated models */
    if (model->is_dynamic && model->data)
    {
        rt_free(model->data);
    }

    /* Clear model structure */
    model->data = NULL;
    model->size = 0;
    model->is_dynamic = false;
    model->is_loaded = false;

    return 0;
}

/**
 * @brief Get the release model data pointer
 * @return Pointer to release model data
 */
const unsigned char *get_release_model_data(void)
{
    /* Load model if not already loaded */
    if (!release_model.is_loaded)
    {
        load_release_model(&release_model);
    }

    return release_model.data;
}

/**
 * @brief Get the release model data size
 * @return Size of release model data
 */
uint32_t get_release_model_size(void)
{
    /* Load model if not already loaded */
    if (!release_model.is_loaded)
    {
        load_release_model(&release_model);
    }

    return release_model.size;
}

int unload_release_model(void)
{
    return unload_model(&release_model);
}

void clear_release_model_file(void)
{
    bloc_file_system.delete_file(GESTURE_RELEASE_MODEL_PATH);
}

/*============================================================================*
 *                           Manifest public API
 *============================================================================*/

/* Find or create the entry in the "models" array for `name`. Caller is the
   owner of `root`; returned cJSON node is owned by `root`. */
static cJSON *manifest_find_or_create_entry(cJSON *root, const char *name)
{
    cJSON *models = cJSON_GetObjectItem(root, "models");
    if (!cJSON_IsArray(models))
    {
        return NULL;
    }

    cJSON *entry = NULL;
    cJSON_ArrayForEach(entry, models)
    {
        cJSON *n = cJSON_GetObjectItem(entry, "name");
        if (cJSON_IsString(n) && strcmp(n->valuestring, name) == 0)
        {
            return entry;
        }
    }

    entry = cJSON_CreateObject();
    if (!entry) return NULL;
    cJSON_AddStringToObject(entry, "name", name);
    cJSON_AddItemToArray(models, entry);
    return entry;
}

int model_manifest_set_version(const char *name, const char *version,
                               uint32_t size)
{
    if (!name || !version) return -1;
    if (!find_known_model(name))
    {
        LOG_W("Manifest set rejected: unknown model %s", name);
        return -2;
    }

    cJSON *root = manifest_load_root();
    if (!root)
    {
        root = cJSON_CreateObject();
        if (!root) return -3;
        cJSON_AddNumberToObject(root, "ver", 1);
        cJSON_AddItemToObject(root, "models", cJSON_CreateArray());
    }

    cJSON *entry = manifest_find_or_create_entry(root, name);
    if (!entry)
    {
        cJSON_Delete(root);
        return -4;
    }

    cJSON_DeleteItemFromObject(entry, "version");
    cJSON_AddStringToObject(entry, "version", version);
    cJSON_DeleteItemFromObject(entry, "size");
    cJSON_AddNumberToObject(entry, "size", (double)size);

    int ret = manifest_save_root(root);
    cJSON_Delete(root);

    if (ret == 0)
    {
        LOG_I("Manifest updated: %s = %s (%u bytes)", name, version,
              (unsigned)size);
    }
    return ret;
}

int model_manifest_to_json(char *out_buf, size_t out_size)
{
    if (!out_buf || out_size == 0) return -1;

    cJSON *disk = manifest_load_root();

    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        if (disk) cJSON_Delete(disk);
        return -2;
    }
    cJSON_AddNumberToObject(root, "ver", 1);
    cJSON *models = cJSON_AddArrayToObject(root, "models");

    /* Iterate KNOWN models so the response always has the same shape, even
       when nothing has been pushed yet. */
    for (size_t i = 0; i < KNOWN_MODEL_COUNT; i++)
    {
        const known_model_entry_t *km = &s_known_models[i];
        const char *version = km->builtin_version;
        uint32_t size = 0;

        if (disk)
        {
            cJSON *disk_models = cJSON_GetObjectItem(disk, "models");
            cJSON *e;
            cJSON_ArrayForEach(e, disk_models)
            {
                cJSON *n = cJSON_GetObjectItem(e, "name");
                if (cJSON_IsString(n) && strcmp(n->valuestring, km->name) == 0)
                {
                    cJSON *v = cJSON_GetObjectItem(e, "version");
                    cJSON *s = cJSON_GetObjectItem(e, "size");
                    if (cJSON_IsString(v)) version = v->valuestring;
                    if (cJSON_IsNumber(s)) size = (uint32_t)s->valuedouble;
                    break;
                }
            }
        }

        cJSON *entry = cJSON_CreateObject();
        cJSON_AddStringToObject(entry, "name", km->name);
        cJSON_AddStringToObject(entry, "version", version);
        cJSON_AddNumberToObject(entry, "size", (double)size);
        cJSON_AddItemToArray(models, entry);
    }

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (disk) cJSON_Delete(disk);

    if (!out) return -3;

    size_t len = strlen(out);
    if (len + 1 > out_size)
    {
        cJSON_free(out);
        return -4;
    }
    memcpy(out_buf, out, len + 1);
    cJSON_free(out);
    return (int)len;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
