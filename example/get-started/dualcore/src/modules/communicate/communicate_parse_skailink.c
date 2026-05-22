/**
 * @file   communicate_parse_skailink.c
 * @brief  ADR-0008 § E7: SKAI_LINK (0x20) device-sync handler.
 *
 * UNVERIFIED — written without an on-device build. BUILD-VERIFY on the dev
 * machine (`project\hcpu\_watch_build.cmd -j8`):
 *   1. Add this file to modules/communicate/SConscript if it lists sources
 *      explicitly (vs glob). Check how communicate_parse_notify.c is listed.
 *   2. cJSON.h resolves on the same include path as communicate_parse_notify.c.
 *   3. LOG_D/LOG_W/LOG_E + rt_* are in scope (match notify's includes; add the
 *      ulog/rtthread header if not).
 *   4. cJSON_ParseWithLength exists in this cJSON version; else null-terminate a
 *      bounded copy and use cJSON_Parse.
 *   5. SkaiWatchSys.device_name[] exists (added to watch_global_data.h alongside
 *      device_status[]) — needed for the UI's per-device name label.
 *   6. status byte semantics: 0 off / 1 on / 2 primary (display-derived).
 *   7. skai_device_ui_refresh() is a weak symbol — null-guarded so this links
 *      before the LVGL UI piece exists.
 */

#include "communicate_parse_skailink.h"

#include <cJSON.h>
#include <string.h>

#include "communicate_parse.h"
#include "communicate_task.h"
#include "watch_global_data.h"
#include "bsp_board.h"

#define DBG_TAG "commu.parse.skailink"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/* UI refresh hook implemented by the watch device screen (LVGL piece). Weak so
   this module links before the UI exists; null-guarded at the call site. */
extern void skai_device_ui_refresh(void) __attribute__((weak));

static void ui_refresh(void)
{
    if (skai_device_ui_refresh)
    {
        skai_device_ui_refresh();
    }
}

static int find_device_index(const char *id)
{
    if (id == NULL || id[0] == '\0')
    {
        return -1;
    }
    for (uint8_t i = 0; i < SkaiWatchSys.device_registry.count; i++)
    {
        if (strncmp((const char *)SkaiWatchSys.device_registry.devices[i].id,
                    id, SYNCED_DEVICE_ID_LEN) == 0)
        {
            return (int)i;
        }
    }
    return -1;
}

/* Find or append a device by id. Returns index, or -1 if the table is full. */
static int ensure_device(const char *id)
{
    int idx = find_device_index(id);
    if (idx >= 0)
    {
        return idx;
    }
    if (SkaiWatchSys.device_registry.count >= MAX_SYNCED_DEVICES)
    {
        LOG_W("skailink: device table full, dropping %s", id);
        return -1;
    }
    idx = SkaiWatchSys.device_registry.count;
    T_SYNCED_DEVICE *d = (T_SYNCED_DEVICE *)&SkaiWatchSys.device_registry.devices[idx];
    rt_memset(d, 0, sizeof(*d));
    strncpy(d->id, id, SYNCED_DEVICE_ID_LEN - 1);
    SkaiWatchSys.device_status[idx] = 0; /* off until a status delta says otherwise */
    SkaiWatchSys.device_registry.count = (uint8_t)(idx + 1);
    return idx;
}

static cJSON *parse_json(uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0)
    {
        return NULL;
    }
    return cJSON_ParseWithLength((const char *)pValue, length);
}

/* 0x01 — authoritative full list: rebuild membership + names + status. */
static void handle_device_list_batch(uint8_t *pValue, uint16_t length)
{
    cJSON *root = parse_json(pValue, length);
    if (!cJSON_IsArray(root))
    {
        LOG_W("skailink: device_list not an array");
        cJSON_Delete(root);
        return;
    }
    SkaiWatchSys.device_registry.count = 0;
    rt_memset((void *)SkaiWatchSys.device_status, 0, sizeof(SkaiWatchSys.device_status));

    cJSON *item = NULL;
    cJSON_ArrayForEach(item, root)
    {
        cJSON *j_id = cJSON_GetObjectItem(item, "id");
        if (!cJSON_IsString(j_id))
        {
            continue;
        }
        int idx = ensure_device(j_id->valuestring);
        if (idx < 0)
        {
            break;
        }
        cJSON *j_name = cJSON_GetObjectItem(item, "name");
        if (cJSON_IsString(j_name))
        {
            strncpy((char *)SkaiWatchSys.device_name[idx], j_name->valuestring,
                    SYNCED_DEVICE_NAME_LEN - 1);
        }
        cJSON *j_status = cJSON_GetObjectItem(item, "status");
        if (cJSON_IsNumber(j_status))
        {
            SkaiWatchSys.device_status[idx] = (uint8_t)j_status->valueint;
        }
    }
    cJSON_Delete(root);
    LOG_I("Loaded %d devices from skailink batch", SkaiWatchSys.device_registry.count);
    watch_prefs_save_device_registry_async(); /* ids changed — persist */
    LOG_I("Device registry updated: count=%d", SkaiWatchSys.device_registry.count);
    ui_refresh();
}

/* 0x02 — {id, status}: one device's status changed. */
static void handle_device_status_delta(uint8_t *pValue, uint16_t length)
{
    cJSON *root = parse_json(pValue, length);
    cJSON *j_id = cJSON_GetObjectItem(root, "id");
    cJSON *j_status = cJSON_GetObjectItem(root, "status");
    if (cJSON_IsString(j_id) && cJSON_IsNumber(j_status))
    {
        int idx = find_device_index(j_id->valuestring);
        if (idx >= 0)
        {
            SkaiWatchSys.device_status[idx] = (uint8_t)j_status->valueint;
            ui_refresh();
        }
    }
    cJSON_Delete(root);
}

/* 0x03 — {device_id, items:[...]}: a device's actions list (same as the left
   primary list's sync, just targeted at one device). */
static void handle_device_actions_batch(uint8_t *pValue, uint16_t length)
{
    cJSON *root = parse_json(pValue, length);
    cJSON *j_dev = cJSON_GetObjectItem(root, "device_id");
    cJSON *j_items = cJSON_GetObjectItem(root, "items");
    if (cJSON_IsString(j_dev) && cJSON_IsArray(j_items))
    {
        int idx = find_device_index(j_dev->valuestring);
        if (idx >= 0)
        {
            T_SYNCED_DEVICE *d =
                (T_SYNCED_DEVICE *)&SkaiWatchSys.device_registry.devices[idx];
            d->default_action_count = 0;
            cJSON *it = NULL;
            cJSON_ArrayForEach(it, j_items)
            {
                if (d->default_action_count >= MAX_DEFAULT_ACTIONS)
                {
                    break;
                }
                const char *title = cJSON_IsString(it) ? it->valuestring
                                    : cJSON_IsString(cJSON_GetObjectItem(it, "title"))
                                        ? cJSON_GetObjectItem(it, "title")->valuestring
                                        : NULL;
                if (title)
                {
                    strncpy(d->default_actions[d->default_action_count], title,
                            DEFAULT_ACTION_LEN - 1);
                    d->default_action_count++;
                }
            }
            watch_prefs_save_device_registry_async(); /* default actions changed — persist */
            ui_refresh();
        }
    }
    cJSON_Delete(root);
}

/* 0x04 — {id}: device removed (logout). */
static void handle_device_removed(uint8_t *pValue, uint16_t length)
{
    cJSON *root = parse_json(pValue, length);
    cJSON *j_id = cJSON_GetObjectItem(root, "id");
    if (cJSON_IsString(j_id))
    {
        int idx = find_device_index(j_id->valuestring);
        if (idx >= 0)
        {
            uint8_t last = (uint8_t)(SkaiWatchSys.device_registry.count - 1);
            /* compact: move the last entry into the hole */
            if ((uint8_t)idx != last)
            {
                rt_memcpy((void *)&SkaiWatchSys.device_registry.devices[idx],
                          (const void *)&SkaiWatchSys.device_registry.devices[last],
                          sizeof(T_SYNCED_DEVICE));
                SkaiWatchSys.device_status[idx] = SkaiWatchSys.device_status[last];
                strncpy((char *)SkaiWatchSys.device_name[idx],
                        (const char *)SkaiWatchSys.device_name[last],
                        SYNCED_DEVICE_NAME_LEN - 1);
            }
            SkaiWatchSys.device_registry.count = last;
            watch_prefs_save_device_registry_async();
            ui_refresh();
        }
    }
    cJSON_Delete(root);
}

void resolve_skailink_command(uint8_t key, uint8_t *pValue, uint16_t length)
{
    switch ((SKAI_LINK_KEY)key)
    {
    case KEY_DEVICE_LIST_BATCH:
        handle_device_list_batch(pValue, length);
        break;
    case KEY_DEVICE_STATUS_DELTA:
        handle_device_status_delta(pValue, length);
        break;
    case KEY_DEVICE_ACTIONS_BATCH:
        handle_device_actions_batch(pValue, length);
        break;
    case KEY_DEVICE_REMOVED:
        handle_device_removed(pValue, length);
        break;
    case KEY_ACTIVE_SELECT:
        /* Uplink-only (watch→phone); never received here. */
        LOG_W("skailink: KEY_ACTIVE_SELECT is uplink-only");
        break;
    default:
        LOG_W("skailink: unknown key 0x%02x", key);
        break;
    }
}
