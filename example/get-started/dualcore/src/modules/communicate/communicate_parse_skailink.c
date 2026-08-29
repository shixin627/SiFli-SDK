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
#include <rtthread.h>   /* rt_malloc / rt_hw_interrupt_* (media_state KE_EVT2→GUI 轉送) */
#include "ui_handler.h" /* lvgl_send_msg(LVGL_MSG_TYPE_MEDIA_STATE_RAW) */
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

/* ── 0x03 批次的每列穩定 id 側表(2026-08-30)────────────────────────────────
   registry 的 T_SYNCED_DEVICE 只存 title(DEFAULT_ACTION_LEN=32,塞不下
   "conv:hermes:local:<profile>:<uuid>" 這種 60 字的 id,而且 8 台×12 列×64B
   的靜態擴欄太肥)。滑鼠抽屜真正需要 id 的只有「使用者剛點下去的那份清單」,
   所以只留最後一個批次:與 default_actions 同步、同 index 對齊(在同一個
   迴圈裡寫入,跳過的列跳過)。查詢用 title 反查 —— 抽屜的 list_items id
   沿用 title(歷史契約,見 feed_single_device_options),真 id 只在點下
   bot/session 列時才需要。 */
static char s_batch_ids_dev[SYNCED_DEVICE_ID_LEN];
static char s_batch_ids[MAX_DEFAULT_ACTIONS][64];
static char s_batch_titles[MAX_DEFAULT_ACTIONS][DEFAULT_ACTION_LEN];
static uint8_t s_batch_ids_count;

/** 用 title 在最後一個 0x03 批次裡反查該列的穩定 id(LauncherAction.Id)。
    回傳 NULL = 該批不是這台設備的 / 沒帶 ids / title 沒中。同 title 多列取第一列。 */
const char *device_actions_id_for_title(const char *device_id, const char *title)
{
    if (device_id == NULL || title == NULL || title[0] == '\0')
        return NULL;
    if (strncmp(s_batch_ids_dev, device_id, SYNCED_DEVICE_ID_LEN) != 0)
        return NULL;
    for (uint8_t i = 0; i < s_batch_ids_count; i++)
    {
        if (s_batch_ids[i][0] != '\0' && strcmp(s_batch_titles[i], title) == 0)
            return s_batch_ids[i];
    }
    return NULL;
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
       position (not by stored count) so a skipped item keeps the rest aligned.
       "ids"(選配,2026-08-30)同樣 parallel:每列的桌面 LauncherAction.Id。 */
    cJSON *j_types = cJSON_GetObjectItem(root, "types");
    cJSON *j_ids = cJSON_GetObjectItem(root, "ids");
    if (cJSON_IsString(j_dev) && cJSON_IsArray(j_items))
    {
        int idx = find_device_index(j_dev->valuestring);
        if (idx >= 0)
        {
            T_SYNCED_DEVICE *d =
                (T_SYNCED_DEVICE *)&SkaiWatchSys.device_registry.devices[idx];
            d->default_action_count = 0;
            /* 每個批次都整份換掉側表(沒帶 ids 就清空)——舊批次的 id 對上新批次的
               title 會開錯房間,寧可 NULL。 */
            strncpy(s_batch_ids_dev, j_dev->valuestring, SYNCED_DEVICE_ID_LEN - 1);
            s_batch_ids_dev[SYNCED_DEVICE_ID_LEN - 1] = '\0';
            s_batch_ids_count = 0;
            cJSON *it = NULL;
            cJSON *jt = cJSON_IsArray(j_types) ? j_types->child : NULL;
            cJSON *ji = cJSON_IsArray(j_ids) ? j_ids->child : NULL;
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
                    /* 側表:title 存截斷後的(跟 registry 一致,反查才對得上)。 */
                    strncpy(s_batch_titles[s_batch_ids_count], slot,
                            DEFAULT_ACTION_LEN - 1);
                    s_batch_titles[s_batch_ids_count][DEFAULT_ACTION_LEN - 1] = '\0';
                    if (ji && cJSON_IsString(ji))
                    {
                        strncpy(s_batch_ids[s_batch_ids_count], ji->valuestring,
                                sizeof(s_batch_ids[0]) - 1);
                        s_batch_ids[s_batch_ids_count][sizeof(s_batch_ids[0]) - 1] = '\0';
                    }
                    else
                    {
                        s_batch_ids[s_batch_ids_count][0] = '\0';
                    }
                    s_batch_ids_count++;
                    d->default_action_count++;
                }
                /* advance types/ids in lockstep with items (by position, every item) */
                if (jt) jt = jt->next;
                if (ji) ji = ji->next;
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

/* 0x20 — the desktop session list for the watch-face session pager
   (lv_session_pager.c). Same contract as the chat state above: parsed off this BLE
   thread into a bounded buffer, rendered on the LVGL thread. Strong extern for the
   same dead-strip reason. */
extern void skai_sessions_on_conv_list(const uint8_t *pValue, uint16_t length);
/* True while the session pager owns the open conversation, i.e. a KEY_CONV_STATE
   belongs to a pager page rather than to the @-list chat room. */
extern bool skai_sessions_owns_conv(void);
extern void skai_sessions_on_conv_state(const uint8_t *pValue, uint16_t length);

static void handle_conv_state(uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0)
    {
        LOG_W("conv_state: empty payload");
        return;
    }
    /* TWO possible owners of an open conversation: the watch-face session pager (a
       desktop session page) and the @-list chat room. Only one is ever open, so ask the
       pager first and fall through to the chat room — this keeps lv_chat_page.c
       untouched by the pager work. */
    if (skai_sessions_owns_conv())
    {
        skai_sessions_on_conv_state(pValue, length);
        return;
    }
    skai_chat_on_conv_state(pValue, length);
}

static void handle_conv_list(uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0)
    {
        LOG_W("conv_list: empty payload");
        return;
    }
    skai_sessions_on_conv_list(pValue, length);
}

/* 0x17: {"focused":bool} — the box the standalone mouse app is controlling just gained/lost a
   focused native text input. Feeds instruction_list_set_remote_target_focus() so
   instruction_list_bar_tap_device() (lv_instruction_list_layout.c) knows whether the next bar-tap
   should skip straight to the input box or show the option list first. */
static void handle_remote_text_focus(uint8_t *pValue, uint16_t length)
{
    cJSON *root = parse_json(pValue, length);
    if (root == NULL)
    {
        LOG_W("skailink: remote_text_focus empty/malformed payload");
        return;
    }
    bool focused = cJSON_IsTrue(cJSON_GetObjectItem(root, "focused"));
    cJSON_Delete(root);

    LOG_I("skailink: remote_text_focus -> %s", focused ? "true" : "false");
    extern void instruction_list_set_remote_target_focus(bool focused);
    instruction_list_set_remote_target_focus(focused);
}

/* 0x19: {"device_id","title","artist","playing"} — the active target device's
   now-playing, pushed by the phone for the mouse app's media centre ONLY. The
   watch face's own media widget keeps using NOTIFY_KEY_MEDIA_TITLE (0x46, the
   phone's own session). hid_mouse filters by device_id against the current active
   selection so a late frame for a just-deselected device can't clobber the UI. */
/* 2026-07-18 STKOF 治本(同 KEY_MEDIA_TITLE):KE_EVT2 上不做 cJSON parse+UI 更新,
   原始 payload 進單槽(last-writer-wins)、發 msg 由 GUI thread 跑
   media_state_apply_pending()(原本的 parse+fanout 搬進去)。 */
static char *volatile s_media_state_pending = NULL;

static void handle_media_state(uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0) return;
    char *buf = (char *)rt_malloc((rt_size_t)length + 1);
    if (buf == NULL) return;
    memcpy(buf, pValue, length);
    buf[length] = '\0';
    rt_base_t level = rt_hw_interrupt_disable();
    char *old = s_media_state_pending;
    s_media_state_pending = buf;
    rt_hw_interrupt_enable(level);
    if (old != NULL) rt_free(old); /* GUI 還沒消化的舊 state 直接淘汰 */
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MEDIA_STATE_RAW;
    lvgl_send_msg(msg);
}

/* GUI thread(ui_handler LVGL_MSG_TYPE_MEDIA_STATE_RAW)。槽空=較新 msg 已先消化,no-op。 */
void media_state_apply_pending(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    char *buf = s_media_state_pending;
    s_media_state_pending = NULL;
    rt_hw_interrupt_enable(level);
    if (buf == NULL) return;
    cJSON *root = cJSON_Parse(buf);
    if (root == NULL)
    {
        LOG_W("skailink: media_state empty/malformed payload");
        rt_free(buf);
        return;
    }
    cJSON *j_id     = cJSON_GetObjectItem(root, "device_id");
    cJSON *j_title  = cJSON_GetObjectItem(root, "title");
    cJSON *j_artist = cJSON_GetObjectItem(root, "artist");
    cJSON *j_play   = cJSON_GetObjectItem(root, "playing");
    /* 該設備目前的系統音量 0..100,-1 = 對方沒回報(舊桌面 / 沒有音訊裝置)。
       音量條靠它擺把手的位置;-1 時**維持現狀**不要把把手歸零。 */
    cJSON *j_vol    = cJSON_GetObjectItem(root, "volume");
    const char *id     = cJSON_IsString(j_id)     ? j_id->valuestring     : "";
    const char *title  = cJSON_IsString(j_title)  ? j_title->valuestring  : "";
    const char *artist = cJSON_IsString(j_artist) ? j_artist->valuestring : "";
    bool playing = cJSON_IsTrue(j_play);
    int volume = cJSON_IsNumber(j_vol) ? j_vol->valueint : -1;

    extern void mouse_mode_handle_remote_media_state(const char *device_id,
                                                     const char *title,
                                                     const char *artist,
                                                     bool playing);
    extern void mouse_mode_handle_remote_volume(const char *device_id, int percent);
    mouse_mode_handle_remote_media_state(id, title, artist, playing);
    mouse_mode_handle_remote_volume(id, volume);
    cJSON_Delete(root);
    rt_free(buf);
}

/* ── 0x1c: 手寫候選字(phone→watch) ─────────────────────────────────────────
   media_state 同款單槽交接:BLE thread 收 raw payload 進槽+發 LVGL msg,GUI thread
   handwrite_cand_apply_pending() 消化(槽空=較新 msg 已先消化,no-op)。 */
static char *s_hw_cand_pending = NULL;

static void handle_handwrite_cand(uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0) return;
    char *buf = (char *)rt_malloc((rt_size_t)length + 1);
    if (buf == NULL) return;
    memcpy(buf, pValue, length);
    buf[length] = '\0';
    rt_base_t level = rt_hw_interrupt_disable();
    char *old = s_hw_cand_pending;
    s_hw_cand_pending = buf;
    rt_hw_interrupt_enable(level);
    if (old != NULL) rt_free(old);
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_HANDWRITE_CAND_RAW;
    lvgl_send_msg(msg);
}

/* GUI thread(ui_handler LVGL_MSG_TYPE_HANDWRITE_CAND_RAW)。 */
void handwrite_cand_apply_pending(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    char *buf = s_hw_cand_pending;
    s_hw_cand_pending = NULL;
    rt_hw_interrupt_enable(level);
    if (buf == NULL) return;

    extern void mouse_handwrite_candidates(const char *const *texts, int count);
    cJSON *root = cJSON_Parse(buf);
    if (root == NULL)
    {
        LOG_W("skailink: handwrite_cand malformed payload");
        rt_free(buf);
        return;
    }
    const char *texts[5];
    int count = 0;
    cJSON *arr = cJSON_GetObjectItem(root, "c");
    if (cJSON_IsArray(arr))
    {
        cJSON *it = NULL;
        cJSON_ArrayForEach(it, arr)
        {
            if (count >= 5) break;
            if (cJSON_IsString(it) && it->valuestring[0] != '\0')
                texts[count++] = it->valuestring;
        }
    }
    mouse_handwrite_candidates(texts, count); /* count=0 亦要送=清空候選列 */
    cJSON_Delete(root);
    rt_free(buf);
}

/* ── 0x23: TV 綁定狀態(phone→watch) ───────────────────────────────────────
   同 media_state 單槽交接:BLE thread 收 raw payload 進槽+發 LVGL msg,GUI thread
   tv_state_apply_pending() 消化(槽空=較新 msg 已先消化,no-op)。狀態列是 UI-only,
   丟掉舊的那筆沒有副作用。 */
static char *s_tv_state_pending = NULL;

static void handle_tv_state(uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0) return;
    char *buf = (char *)rt_malloc((rt_size_t)length + 1);
    if (buf == NULL) return;
    memcpy(buf, pValue, length);
    buf[length] = '\0';
    rt_base_t level = rt_hw_interrupt_disable();
    char *old = s_tv_state_pending;
    s_tv_state_pending = buf;
    rt_hw_interrupt_enable(level);
    if (old != NULL) rt_free(old);
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_TV_STATE_RAW;
    lvgl_send_msg(msg);
}

/* GUI thread(ui_handler LVGL_MSG_TYPE_TV_STATE_RAW)。 */
void tv_state_apply_pending(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    char *buf = s_tv_state_pending;
    s_tv_state_pending = NULL;
    rt_hw_interrupt_enable(level);
    if (buf == NULL) return;

    extern void tv_remote_handle_state(const char *name, const char *platform,
                                       const char *state, const char *detail);
    cJSON *root = cJSON_Parse(buf);
    if (root == NULL)
    {
        LOG_W("skailink: tv_state malformed payload");
        rt_free(buf);
        return;
    }
    cJSON *j_name = cJSON_GetObjectItem(root, "name");
    cJSON *j_plat = cJSON_GetObjectItem(root, "platform");
    cJSON *j_st   = cJSON_GetObjectItem(root, "state");
    cJSON *j_det  = cJSON_GetObjectItem(root, "detail");
    tv_remote_handle_state(cJSON_IsString(j_name) ? j_name->valuestring : "",
                           cJSON_IsString(j_plat) ? j_plat->valuestring : "",
                           cJSON_IsString(j_st)   ? j_st->valuestring   : "none",
                           cJSON_IsString(j_det)  ? j_det->valuestring  : "");
    cJSON_Delete(root);
    rt_free(buf);
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
    case KEY_CONV_LIST_REQ:
        /* @-conversation control — uplink-only (watch→phone); never received here. */
        LOG_W("skailink: conv key 0x%02x is uplink-only", key);
        break;
    case KEY_CONV_LIST:
        /* phone→watch (DOWNLINK): the desktop session list for the session pager. */
        handle_conv_list(pValue, length);
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
    case KEY_REMOTE_TEXT_FOCUS:
        /* phone→watch (DOWNLINK): controlled box's focused-text-input state changed. */
        handle_remote_text_focus(pValue, length);
        break;
    case KEY_MEDIA_CONTROL:
        /* Uplink-only (watch→phone); never received here. */
        LOG_W("skailink: KEY_MEDIA_CONTROL is uplink-only");
        break;
    case KEY_MEDIA_STATE:
        /* phone→watch (DOWNLINK): active target device's now-playing (mouse app). */
        handle_media_state(pValue, length);
        break;
    case KEY_HANDWRITE_CAND:
        /* phone→watch (DOWNLINK): 手寫當前字的辨識候選(ML Kit top-5)。 */
        handle_handwrite_cand(pValue, length);
        break;
    case KEY_TV_CONTROL:
        /* Uplink-only (watch→phone); never received here. */
        LOG_W("skailink: KEY_TV_CONTROL is uplink-only");
        break;
    case KEY_TV_STATE:
        /* phone→watch (DOWNLINK): bound TV + pairing state for the TV remote app. */
        handle_tv_state(pValue, length);
        break;
    default:
        LOG_W("skailink: unknown key 0x%02x", key);
        break;
    }
}
