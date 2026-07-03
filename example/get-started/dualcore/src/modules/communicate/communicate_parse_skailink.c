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
 *   7. skai_device_ui_refresh() is a STRONG reference (device_pager.c provides it
 *      in both watch + PC-sim builds). It was weak once, but a weak-only
 *      reference let armlink dead-strip the definition -> NULL -> live syncs
 *      never refreshed the device page (only re-entry did). Keep it strong.
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

/* UI refresh hook implemented by device_pager.c (the LVGL device screen), which
   is linked in BOTH the watch and PC-sim builds.
   MUST be a STRONG reference: when this was declared __attribute__((weak)) and
   null-guarded, the definition — referenced only weakly — got dead-stripped by
   armlink, so the symbol resolved to NULL and ui_refresh() became a silent no-op.
   The device page then only re-rendered on re-entry (device_pager_set_active ->
   device_pager_refresh), never on a live phone sync: items arrived in the
   registry but the on-screen list never refreshed. A strong reference keeps the
   definition and actually delivers the refresh (LVGL_MSG_TYPE_REFRESH_DEVICE_PAGER). */
extern void skai_device_ui_refresh(void);

static void ui_refresh(void)
{
    skai_device_ui_refresh();
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

/* 0x01 - authoritative full list: reconcile membership + names + status, and
   PRESERVE each surviving device's default_actions. Actions are owned by the
   0x03 batch (handle_device_actions_batch), NOT this one. The old code did
   count=0 + ensure_device, whose memset wiped every device's actions, so any
   0x01 (a status flip, the primary's ~60s membership poll, ...) blanked the
   device_pager's action list until a 0x03 refilled it - and the primary's 0x03
   forward is de-duped, so outside connect-time it often didn't, leaving the
   page's default options empty. Now: match by id (keep actions), append only
   genuinely-new devices (empty until their own 0x03), drop ids absent here. */
static void handle_device_list_batch(uint8_t *pValue, uint16_t length)
{
    cJSON *root = parse_json(pValue, length);
    if (!cJSON_IsArray(root))
    {
        LOG_W("skailink: device_list not an array");
        cJSON_Delete(root);
        return;
    }

    /* Registry slots that appear in this authoritative list. ensure_device only
       ever appends, so slot indices stay stable across the loop. */
    uint8_t seen[MAX_SYNCED_DEVICES];
    rt_memset(seen, 0, sizeof(seen));

    cJSON *item = NULL;
    cJSON_ArrayForEach(item, root)
    {
        cJSON *j_id = cJSON_GetObjectItem(item, "id");
        if (!cJSON_IsString(j_id))
        {
            continue;
        }
        int idx = ensure_device(j_id->valuestring); /* existing keeps actions; new is appended */
        if (idx < 0)
        {
            break;
        }
        seen[idx] = 1;
        cJSON *j_name = cJSON_GetObjectItem(item, "name");
        if (cJSON_IsString(j_name))
        {
            strncpy((char *)SkaiWatchSys.device_name[idx], j_name->valuestring,
                    SYNCED_DEVICE_NAME_LEN - 1);
            SkaiWatchSys.device_name[idx][SYNCED_DEVICE_NAME_LEN - 1] = '\0';
        }
        cJSON *j_status = cJSON_GetObjectItem(item, "status");
        SkaiWatchSys.device_status[idx] =
            cJSON_IsNumber(j_status) ? (uint8_t)j_status->valueint : 0;
    }

    /* Forward-compact: drop registry slots absent from this authoritative list,
       keeping survivors (with their default_actions) in order. w <= r always, so
       each source slot is read before any later write can reach it. */
    uint8_t w = 0;
    for (uint8_t r = 0; r < SkaiWatchSys.device_registry.count; r++)
    {
        if (!seen[r])
        {
            continue;
        }
        if (w != r)
        {
            rt_memcpy((void *)&SkaiWatchSys.device_registry.devices[w],
                      (const void *)&SkaiWatchSys.device_registry.devices[r],
                      sizeof(T_SYNCED_DEVICE));
            SkaiWatchSys.device_status[w] = SkaiWatchSys.device_status[r];
            strncpy((char *)SkaiWatchSys.device_name[w],
                    (const char *)SkaiWatchSys.device_name[r],
                    SYNCED_DEVICE_NAME_LEN - 1);
            SkaiWatchSys.device_name[w][SYNCED_DEVICE_NAME_LEN - 1] = '\0';
        }
        w++;
    }
    SkaiWatchSys.device_registry.count = w;
    cJSON_Delete(root);
    LOG_I("Loaded %d devices from skailink batch", SkaiWatchSys.device_registry.count);
    /* Dump the received device list so its contents can be inspected on the
       console (id / name / status + each device's item/action list). NOTE: the
       items arrive in a SEPARATE message (handle_device_actions_batch, 0x03), so
       items=0 here just means that device's actions haven't synced yet. */
    for (uint8_t i = 0; i < SkaiWatchSys.device_registry.count; i++)
    {
        T_SYNCED_DEVICE *d =
            (T_SYNCED_DEVICE *)&SkaiWatchSys.device_registry.devices[i];
        LOG_I("  device[%u]: id=%s name=%s status=%u items=%u", (unsigned)i,
              d->id,
              (const char *)SkaiWatchSys.device_name[i],
              (unsigned)SkaiWatchSys.device_status[i],
              (unsigned)d->default_action_count);
        for (uint8_t k = 0; k < d->default_action_count; k++)
        {
            LOG_I("      item[%u]: %s", (unsigned)k, d->default_actions[k]);
        }
    }
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
    /* Phone payload: {device_id, items:[<name strings>], types:[<int per item>]}.
       "types" is a SEPARATE array PARALLEL to "items" (same index), 0=instruction
       1=application 2=folder 3=ai. We walk types in lockstep with items by
       position (not by stored count) so a skipped item keeps the rest aligned. */
    cJSON *j_types = cJSON_GetObjectItem(root, "types");
    if (cJSON_IsString(j_dev) && cJSON_IsArray(j_items))
    {
        int idx = find_device_index(j_dev->valuestring);
        if (idx >= 0)
        {
            T_SYNCED_DEVICE *d =
                (T_SYNCED_DEVICE *)&SkaiWatchSys.device_registry.devices[idx];
            d->default_action_count = 0;
            cJSON *it = NULL;
            cJSON *jt = cJSON_IsArray(j_types) ? j_types->child : NULL;
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
                    char *slot = d->default_actions[d->default_action_count];
                    strncpy(slot, title, DEFAULT_ACTION_LEN - 1);
                    /* strncpy does NOT null-terminate when the source is >= the
                       limit; the buffer also isn't re-zeroed on a re-sync, so the
                       terminator is mandatory. Without it, %s / strlen on this
                       slot reads past the buffer into the next field (the garbled
                       item logs that bled the device id + header string). */
                    slot[DEFAULT_ACTION_LEN - 1] = '\0';
                    /* category from the parallel types[] entry at this position. */
                    d->default_action_types[d->default_action_count] =
                        (jt && cJSON_IsNumber(jt)) ? (uint8_t)jt->valueint : 0;
                    d->default_action_count++;
                }
                /* advance types in lockstep with items (by position, every item) */
                if (jt) jt = jt->next;
            }
            /* Dump the device's item (action) list so it can be inspected. */
            LOG_I("device actions for %s (name=%s): %u item(s)",
                  j_dev->valuestring, (const char *)SkaiWatchSys.device_name[idx],
                  (unsigned)d->default_action_count);
            for (uint8_t k = 0; k < d->default_action_count; k++)
            {
                LOG_I("    item[%u]: type=%u %s", (unsigned)k,
                      (unsigned)d->default_action_types[k], d->default_actions[k]);
            }
            ui_refresh();
        }
        else
        {
            LOG_W("device actions: unknown device_id %s (not in registry)",
                  j_dev->valuestring);
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
                SkaiWatchSys.device_name[idx][SYNCED_DEVICE_NAME_LEN - 1] = '\0';
            }
            SkaiWatchSys.device_registry.count = last;
            ui_refresh();
        }
    }
    cJSON_Delete(root);
}

/* 0x12 — {title, sending, messages:[{role,text}]}: the folded chat state for the
   currently-open @-conversation. Forwarded VERBATIM to the chat-room GUI
   (lv_chat_page.c), which parses + renders it on the LVGL thread (this runs on the
   BLE parse thread). Strong extern — same rule as skai_device_ui_refresh: a weak ref
   gets dead-stripped by armlink → the symbol resolves NULL → a silent no-op. */
extern void skai_chat_on_conv_state(const uint8_t *pValue, uint16_t length);

/* 0x13/0x14 — SkaiApp package install path (gui_apps/skaiapp/skaiapp_proto.c).
   Runs here on the BLE parse thread by design (chunk reassembly + FS write +
   engine reseed); acks back over KEY_SKAIAPP_ACK. Strong externs, same
   dead-strip rule as above. */
extern void skaiapp_on_push_chunk(const uint8_t *pValue, uint16_t length);
extern void skaiapp_on_remove(const uint8_t *pValue, uint16_t length);

static void handle_conv_state(uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0)
    {
        LOG_W("conv_state: empty payload");
        return;
    }
    skai_chat_on_conv_state(pValue, length);
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
    case KEY_ACTION_SELECT:
        /* Uplink-only (watch→phone); never received here. */
        LOG_W("skailink: KEY_ACTION_SELECT is uplink-only");
        break;
    case KEY_ACTION_FOCUS:
        /* Uplink-only (watch→phone); never received here. */
        LOG_W("skailink: KEY_ACTION_FOCUS is uplink-only");
        break;
    case KEY_MOUSE_MOVE:
    case KEY_MOUSE_BUTTON:
    case KEY_MOUSE_SCROLL:
    case KEY_MOUSE_BACK:
        /* Device-page trackpad relay — uplink-only (watch→phone); never received. */
        LOG_W("skailink: mouse key 0x%02x is uplink-only", key);
        break;
    case KEY_CONV_STATE:
        /* phone→watch (DOWNLINK): folded chat state for the open @-conversation. */
        handle_conv_state(pValue, length);
        break;
    case KEY_CONV_OPEN:
    case KEY_CONV_SEND:
    case KEY_CONV_CLOSE:
        /* @-conversation control — uplink-only (watch→phone); never received here. */
        LOG_W("skailink: conv key 0x%02x is uplink-only", key);
        break;
    case KEY_SKAIAPP_PUSH:
        /* phone→watch (DOWNLINK): chunked AI mini-app package (ADR-0037). */
        skaiapp_on_push_chunk(pValue, length);
        break;
    case KEY_SKAIAPP_REMOVE:
        /* phone→watch (DOWNLINK): uninstall one AI mini-app. */
        skaiapp_on_remove(pValue, length);
        break;
    case KEY_SKAIAPP_ACK:
        /* Uplink-only (watch→phone); never received here. */
        LOG_W("skailink: KEY_SKAIAPP_ACK is uplink-only");
        break;
    case KEY_SKAIAPP_VOICE:
        /* Uplink-only (watch→phone); never received here. */
        LOG_W("skailink: KEY_SKAIAPP_VOICE is uplink-only");
        break;
    default:
        LOG_W("skailink: unknown key 0x%02x", key);
        break;
    }
}
