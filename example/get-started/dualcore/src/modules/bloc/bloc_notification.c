/**
 ******************************************************************************
 * @file   bloc_notification.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <rtthread.h>

/* Bloc modules */
#include "bloc_notification.h"
#include "notification_reply.h"
#include "bloc_control.h"
#include "bloc_skaiwalk.h"
#include "bloc_v2t.h"
#include "bloc_peripheral.h"
#include "gui_app_fwk.h"

/* Communication modules */
#include "communicate_protocol.h"
#include "communicate_task.h"

/* System and data modules */
#include "watch_global_data.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif
#include "app_mainmenu.h"

/* UI modules */
#include "gui_app_pm.h"
#include "ui_handler.h"
#include "gui_app_fwk.h"
#include "app_message.h"
#include "app_clock_main.h"
#include "ui_helper.h"
#include "share_prefs.h"
#include "gui_app_int.h"

/* Utilities */
#include "cJSON.h"

#define DBG_TAG "bloc.notification"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/* Auto-pop the charge screen only in release builds. Dev and release now share
   the same physical board (VER_29), so the dev-vs-release split is keyed on
   kReleaseMode rather than the board version. */
#if kReleaseMode
    #define CHARGE_INTERACT_ENABLE 1
#else
    #define CHARGE_INTERACT_ENABLE 0
#endif

extern void incoming_call_set_caller(const char *title, const char *id,
                                     uint8_t type);
extern void incoming_call_close_if_active(const char *id);

static char temp_send_json_string[512];

static char temp_speech_text[512];


static notification_t _notification_list[ITEM_AMOUNT_NOTIFICATION];

const notification_t notifications_constant[] = {
    [0] = {.id = "1",
           .message = "Do you want to play basketball?",
           .title = "Jack",
           .state = true},
    [1] = {.id = "2",
           .message = "Do you want to play football?",
           .title = "Tom",
           .state = true},
    [2] = {.id = "3",
           .message = "Do you want to play tennis?",
           .title = "Jerry",
           .state = true},
    [3] = {.id = "4",
           .message = "Do you want to play badminton?",
           .title = "Lily",
           .state = true},
    [4] = {.id = "5",
           .message = "Do you want to play volleyball?",
           .title = "Lucy",
           .state = true},
    [5] = {.id = "6",
           .message = "Do you want to play pingpong?",
           .title = "Mike",
           .state = true},
    [6] = {.id = "7",
           .message = "Do you want to play baseball?",
           .title = "John",
           .state = true},
};

uint8_t notification_items_amount = 0;
uint8_t selected_notification_index = 0;

static struct rt_mutex notification_mutex;
static bool notification_mutex_ready;
static int notification_store_init(void)
{
    int result = rt_mutex_init(&notification_mutex, "notif", RT_IPC_FLAG_PRIO);
    notification_mutex_ready = result == RT_EOK;
    return result;
}
INIT_COMPONENT_EXPORT(notification_store_init);
static void notification_store_lock(void) { if (notification_mutex_ready) rt_mutex_take(&notification_mutex, RT_WAITING_FOREVER); }
static void notification_store_unlock(void) { if (notification_mutex_ready) rt_mutex_release(&notification_mutex); }

/**
 * @brief Mark a notification as read/seen
 * @param index Index of the notification to mark
 */
static void notifyNotification(uint8_t index)
{
    get_notification(index)->state = true;
}

/**
 * @brief Get the total count of active notifications
 * @return Number of active notifications
 */
uint32_t notification_center_get_info_count(void)
{
    return notification_items_amount;
}

/* Monotonic arrival counter — incremented every time update_notification()
   accepts a new item. Count-based dedup breaks when the ring buffer is full
   (oldest is overwritten so count stays at ITEM_AMOUNT_NOTIFICATION),
   callers that need "is this genuinely new?" should compare this seq
   against their last-seen value. */
static uint32_t notification_arrival_seq = 0;

uint32_t notification_center_get_arrival_seq(void)
{
    return notification_arrival_seq;
}

/* When the seq last advanced. The dial header uses it to tell "arrived just
   now" from "arrived a while ago and nobody popped it yet". */
static rt_tick_t notification_arrival_tick = 0;

rt_tick_t notification_center_get_arrival_tick(void)
{
    return notification_arrival_tick;
}

/**
 * @brief Get notification at specified index
 * @param index Index of the notification
 * @return Pointer to the notification
 */
notification_t *get_notification(int index)
{
    if (index < 0 || index >= ITEM_AMOUNT_NOTIFICATION)
    {
        LOG_E("get_notification: invalid index %d", index);
        return NULL;
    }
    return &_notification_list[index];
}

notification_t *get_notification_in_reversed_ui(int index)
{
    uint8_t real_index = notification_items_amount - index - 1;
    return &_notification_list[real_index];
}

void set_notification(const notification_t *notification, int index)
{
    notification_store_lock();
    _notification_list[index] = *notification;
    _notification_list[index].index = index + 1;
    notifyNotification(index);
    notification_store_unlock();
}

notification_t *get_cur_notification(void)
{
    return &_notification_list[selected_notification_index];
}

static void update_notification_locked(const notification_t *newNotification)
{
    /* If the user already dismissed this notification (within the persisted
     * ring), drop it silently. Covers the BLE-reconnect re-sync case where
     * the phone re-pushes ids we've already processed but locally removed. */
    if ((!newNotification->reply_revision && bloc_notification_is_dismissed(newNotification->id)) ||
        notification_reply_was_processed(newNotification->reply_instance, newNotification->reply_revision))
    {
        LOG_I("update_notification: id=%s previously dismissed, skipping",
              newNotification->id);
        return;
    }

    // Dedup: a re-pushed notification we already hold (same id, or same
    // visible content under a re-minted id) is handled IN PLACE — never
    // removed and re-inserted at slot 0. Re-inserting reordered the list
    // into the phone's resync iteration order on every reconnect: the
    // watchface mini-widget (slot 0) suddenly showed a different app's
    // notification and the list flipped newest-to-top. Only a genuinely
    // new arrival (or a genuine content update of an existing id) takes
    // slot 0. For Notify_Skaiwalk category, also dedup by type to keep
    // only the latest one.
    bool dup = false;
    for (int i = notification_items_amount - 1; i >= 0; i--)
    {
        if (newNotification->reply_revision &&
            !strcmp(_notification_list[i].reply_instance, newNotification->reply_instance) &&
            _notification_list[i].reply_revision > newNotification->reply_revision) return;
        dup = (newNotification->reply_revision && _notification_list[i].reply_revision) ?
            !strcmp(_notification_list[i].reply_instance, newNotification->reply_instance) :
            (strcmp(_notification_list[i].id, newNotification->id) == 0);
        if (dup &&
            strcmp(_notification_list[i].title, newNotification->title) == 0 &&
            strcmp(_notification_list[i].message, newNotification->message) == 0)
        {
            memcpy(_notification_list[i].id, newNotification->id, sizeof(newNotification->id));
            _notification_list[i].can_reply = newNotification->can_reply;
            _notification_list[i].reply_revision = newNotification->reply_revision;
            memcpy(_notification_list[i].reply_instance, newNotification->reply_instance, 33);
            _notification_list[i].option_count = newNotification->option_count;
            memcpy(_notification_list[i].options, newNotification->options, sizeof(newNotification->options));
            /* Identical reconnect re-push (the phone re-sends its whole
               active list on every reconnect): entry is already exactly
               here — keep position and sec_time, and no seq bump, so the
               callers' seq gate stays silent (no banner / buzz / wake). */
            return;
        }
        /* Same app + identical visible content under a DIFFERENT id: the
           phone re-minted the id across a disconnect (its sbn.key→id map
           entry was dropped while the matching dismiss couldn't be
           delivered) or across a bridge process restart. Adopt the phone's
           current id in place so watch-side dismiss/reply keep addressing
           the live notification — position, sec_time and seq untouched. */
        if (!dup && !newNotification->reply_revision && !_notification_list[i].reply_revision &&
            _notification_list[i].type == newNotification->type &&
            strcmp(_notification_list[i].title, newNotification->title) == 0 &&
            strcmp(_notification_list[i].message, newNotification->message) == 0)
        {
            LOG_I("update_notification: content-dup, adopting id=%s in place (was %s)",
                  newNotification->id, _notification_list[i].id);
            strncpy(_notification_list[i].id, newNotification->id,
                    sizeof(_notification_list[i].id) - 1);
            _notification_list[i].id[sizeof(_notification_list[i].id) - 1] = '\0';
            _notification_list[i].can_reply = newNotification->can_reply;
            memcpy(_notification_list[i].options, newNotification->options,
                   sizeof(newNotification->options));
            _notification_list[i].option_count = newNotification->option_count;
            return;
        }
        if (!dup && newNotification->type == Notify_Skaiwalk &&
            _notification_list[i].type == Notify_Skaiwalk)
        {
            dup = true;
        }
        if (dup)
        {
            /* Same id with CHANGED content (a genuine update from the
               source app) or a Notify_Skaiwalk replacement — fall through
               below to re-insert at slot 0 as a new arrival. */
            LOG_D("Dedup: removing existing notification id=%s type=%d",
                  _notification_list[i].id, _notification_list[i].type);
            // Shift remaining items down
            for (int j = i; j < notification_items_amount - 1; j++)
            {
                _notification_list[j] = _notification_list[j + 1];
            }
            notification_items_amount--;
            break;
        }
    }

    if (notification_items_amount > 0)
    {
        uint8_t i = 0;
        if (notification_items_amount == ITEM_AMOUNT_NOTIFICATION)
        {
            i = 1;
        }
        for (; i <= notification_items_amount - 1; i++)
        {
            set_notification(get_notification(notification_items_amount - i - 1),
                             notification_items_amount - i);
        }
    }

    set_notification(newNotification, 0);

    if (notification_items_amount < ITEM_AMOUNT_NOTIFICATION)
    {
        notification_items_amount++;
    }

    /* Everything reaching here is a new arrival in the seq-gate sense
       (brand new, a content update, or a Skaiwalk replacement) — advance
       the seq so the dial header pops + buzzes. Silent re-pushes returned
       early above without touching the list order or the seq. */
    notification_arrival_seq++;
    notification_arrival_tick = rt_tick_get();

    SkaiWatchSys.notification_number = notification_items_amount;
}

static void update_notification(const notification_t *notification)
{
    notification_store_lock();
    update_notification_locked(notification);
    notification_store_unlock();
}

/* A reply decision must not borrow a pointer that BLE ingress can replace. */
bool notification_reply_target_current(const char *instance, uint32_t revision, bool require_reply)
{
    if (!instance || !revision) return false;
    bool found = false;
    notification_store_lock();
    for (unsigned i = 0; i < notification_items_amount; i++) {
        const notification_t *item = &_notification_list[i];
        if (item->reply_revision == revision && !strcmp(item->reply_instance, instance) &&
            (!require_reply || item->can_reply)) { found = true; break; }
    }
    notification_store_unlock();
    return found;
}

/**
 * @brief Parse notification data from JSON string
 * @param json_str JSON string containing notification data
 * @param notification Pointer to notification structure to populate
 * @return true when `notification` was populated; false leaves it untouched.
 *
 * The return value is load-bearing: `handle_notification` parses into a static
 * struct, so a caller that ignores a failure re-sends whatever the PREVIOUS
 * notification was. If that one was a call card, the watch replays the
 * incoming-call screen for an unrelated notification.
 */
static bool parse_notification(const char *json_str,
                               notification_t *notification)
{
    if (!json_str || !notification)
    {
        LOG_E("parse_notification: invalid parameters");
        return false;
    }

    LOG_D("parse notification:%s", json_str);

    cJSON *root = cJSON_Parse(json_str);
    if (!root)
    {
        LOG_E("parse_notification: JSON parsing failed");
        return false;
    }

    cJSON *id_json      = cJSON_GetObjectItem(root, "id");
    cJSON *title_json   = cJSON_GetObjectItem(root, "title");
    cJSON *message_json = cJSON_GetObjectItem(root, "message");
    cJSON *reply_json   = cJSON_GetObjectItem(root, "reply");
    cJSON *calling_json = cJSON_GetObjectItem(root, "calling");
    cJSON *instance_json = cJSON_GetObjectItemCaseSensitive(root, "ri");
    cJSON *revision_json = cJSON_GetObjectItemCaseSensitive(root, "rv");
    notification->reply_instance[0] = 0;
    notification->reply_revision = 0;
    if (cJSON_IsString(instance_json) && strlen(instance_json->valuestring) == 32 &&
        strspn(instance_json->valuestring, "0123456789abcdef") == 32 &&
        cJSON_IsNumber(revision_json) && revision_json->valuedouble >= 1 &&
        revision_json->valuedouble <= 4294967295.0 &&
        revision_json->valuedouble == (uint32_t)revision_json->valuedouble) {
        memcpy(notification->reply_instance, instance_json->valuestring, 33);
        notification->reply_revision = (uint32_t)revision_json->valuedouble;
    }
    notification->calling = calling_json ? cJSON_IsTrue(calling_json) : false;

    /* Copy a cJSON string into a fixed-size dst array, falling back to `def`
       if the field is missing/null. `dst` must be a real array. */
    #define COPY_STR_OR_DEF(dst, j, def) do { \
        if ((j) && (j)->valuestring) { \
            strncpy((dst), (j)->valuestring, sizeof(dst) - 1); \
            (dst)[sizeof(dst) - 1] = '\0'; \
        } else { \
            strcpy((dst), (def)); \
        } \
    } while (0)

    COPY_STR_OR_DEF(notification->id,      id_json,      "unknown");
    COPY_STR_OR_DEF(notification->title,   title_json,   "Unknown");
    COPY_STR_OR_DEF(notification->message, message_json, "");
    #undef COPY_STR_OR_DEF

    notification->can_reply = reply_json ? cJSON_IsTrue(reply_json) : false;

    /* Optional "options": ["Yes","No",...] -- absent on most notifications. */
    notification->option_count = 0;
    cJSON *options_json = cJSON_GetObjectItem(root, "options");
    if (options_json && cJSON_IsArray(options_json))
    {
        cJSON *opt;
        cJSON_ArrayForEach(opt, options_json)
        {
            if (notification->option_count >= NOTIFICATION_OPTION_MAX)
                break;
            if (!opt->valuestring || opt->valuestring[0] == '\0')
                continue;
            char *dst = notification->options[notification->option_count++];
            strncpy(dst, opt->valuestring, NOTIFICATION_OPTION_LEN - 1);
            dst[NOTIFICATION_OPTION_LEN - 1] = '\0';
        }
    }

    LOG_D("id:%s", notification->id);
    LOG_D("title:%s", notification->title);
    LOG_D("message:%s", notification->message);
    LOG_D("reply:%d", notification->can_reply);

    cJSON_Delete(root);
    return true;
}

/* Notification id <-> app-name table. The order of `notify_name_map` is
   load-bearing: shorter names that are substrings of longer ones (e.g.
   "Mail" vs "Gmail", "Line" vs "LinkedIn") would shadow the longer ones
   under strstr — preserve the original priority by listing them first. */
static const char *const notify_app_names[] = {
    [Notify_calendar]   = "Calendar",
    [Notify_facebook]   = "Facebook",
    [Notify_facetime]   = "FaceTime",
    [Notify_instagram]  = "Instagram",
    [Notify_kakaotalk]  = "KakaoTalk",
    [Notify_line]       = "Line",
    [Notify_linkedin]   = "LinkedIn",
    [Notify_mail]       = "Mail",
    [Notify_messenger]  = "Messenger",
    [Notify_others]     = "Others",
    [Notify_QQ]         = "QQ",
    [Notify_skype]      = "Skype",
    [Notify_SMS]        = "SMS",
    [Notify_snap]       = "Snap",
    [Notify_twitter]    = "Twitter",
    [Notify_wechat]     = "WeChat",
    [Notify_whatsapp]   = "WhatsApp",
    [Notify_gmail]      = "Gmail",
    [Notify_dingtalk]   = "DingTalk",
    [Notify_googlechat] = "Google Chat",
    [Notify_discord]    = "Discord",
    [Notify_youtube]    = "YouTube",
    [Notify_tiktok]     = "TikTok",
    [Notify_telegram]   = "Telegram",
    [Notify_twitch]     = "Twitch",
    [Notify_slack]      = "Slack",
    [Notify_lark]       = "Lark",
    [Notify_reddit]     = "Reddit",
    [Notify_Skaiwalk]   = "Skaiwalk",
};

const char *get_app_name_from_notify_id(Notifications_Type notify_id)
{
    if ((unsigned)notify_id < sizeof(notify_app_names) / sizeof(notify_app_names[0])
        && notify_app_names[notify_id])
    {
        return notify_app_names[notify_id];
    }
    return "Unknown";
}

/* Same name list, ordered for ANCS lookup (Mail before Gmail etc., as in the
   original cascade — kept verbatim to preserve substring priorities). */
static const struct { const char *name; uint8_t type; } notify_name_map[] = {
    {"Calendar",     Notify_calendar},
    {"Facebook",     Notify_facebook},
    {"FaceTime",     Notify_facetime},
    {"Instagram",    Notify_instagram},
    {"KakaoTalk",    Notify_kakaotalk},
    {"Line",         Notify_line},
    {"LinkedIn",     Notify_linkedin},
    {"Mail",         Notify_mail},
    {"Messenger",    Notify_messenger},
    {"QQ",           Notify_QQ},
    {"Skype",        Notify_skype},
    {"SMS",          Notify_SMS},
    {"Snap",         Notify_snap},
    {"Twitter",      Notify_twitter},
    {"WeChat",       Notify_wechat},
    {"WhatsApp",     Notify_whatsapp},
    {"Gmail",        Notify_gmail},
    {"DingTalk",     Notify_dingtalk},
    {"Google Chat",  Notify_googlechat},
    {"Discord",      Notify_discord},
    {"YouTube",      Notify_youtube},
    {"TikTok",       Notify_tiktok},
    {"Telegram",     Notify_telegram},
    {"Twitch",       Notify_twitch},
    {"Slack",        Notify_slack},
    {"Lark",         Notify_lark},
    {"Reddit",       Notify_reddit},
    {"Skaiwalk",     Notify_Skaiwalk},
};

uint8_t get_notification_type_from_ios_ancs_name(const char *name)
{
    for (size_t i = 0; i < sizeof(notify_name_map) / sizeof(notify_name_map[0]); i++)
    {
        if (strstr(name, notify_name_map[i].name) != NULL)
        {
            return notify_name_map[i].type;
        }
    }
    return Notify_others;
}

static notification_t temp_notification;

void app_message_set_from_temp(notification_t *notification)
{
    const char *app_name = get_app_name_from_notify_id(notification->type);
    app_message_set_app_name((uint8_t *)app_name);
    app_message_set_title((uint8_t *)notification->title);
    app_message_set_content((uint8_t *)notification->message);
    app_message_set_app_index(notification->type);
}

void navigate_notification_info(notification_t *notification)
{
    app_message_set_from_temp(notification);
    intent_t intent = intent_init("message");
    intent_set_string(intent, "noti_id", notification->id);
    intent_set_string(intent, "noti_instance", notification->reply_instance);
    intent_set_uint32(intent, "noti_revision", notification->reply_revision);
    intent_set_string(intent, "noti_can_reply",
                      notification->can_reply ? "true" : "false");
    if (!is_at_instruction_list())
    {
        if (myLancher[app_index_instruction_list].reset_list != NULL)
        {
            myLancher[app_index_instruction_list].reset_list();
        }
    }
    intent_runapp(intent);
}

/**
 * Flatten a notification string into a single line, in place.
 *
 * Why: the dial header's body label runs LV_LABEL_LONG_SCROLL_CIRCULAR, and
 * LVGL's EXPAND flag only cancels *width-based* word wrap — a literal '\n' or
 * '\r' still breaks the line (lv_txt.c, _lv_txt_get_next_line()). A two-line
 * body then measures short in x and short in y, so neither the horizontal nor
 * the vertical scroll animation starts and the header freezes as two static
 * lines instead of a marquee. Android bodies carry newlines routinely (group
 * chats, InboxStyle digests, multi-line LINE messages), so the fix belongs at
 * the ingestion boundary rather than in one label.
 *
 * Tabs go too (same class of layout surprise), runs of whitespace collapse to
 * a single space, and leading/trailing space is trimmed. Only ASCII control
 * bytes are touched, so UTF-8 sequences (all continuation bytes >= 0x80) pass
 * through untouched.
 *
 * Also force-terminates the buffer: callers fill these fields with
 * strncpy(dst, src, sizeof(dst) - 1) into an uninitialised stack struct, which
 * leaves no NUL when the source is over-long.
 */
void notification_flatten_line(char *buf, size_t cap)
{
    if (buf == NULL || cap == 0)
    {
        return;
    }
    buf[cap - 1] = '\0';

    size_t w = 0;
    bool pending_space = false;
    for (size_t r = 0; r < cap - 1 && buf[r] != '\0'; r++)
    {
        char c = buf[r];
        if (c == '\n' || c == '\r' || c == '\t' || c == ' ')
        {
            /* Defer: collapses whitespace runs and drops leading space. */
            pending_space = (w > 0);
            continue;
        }
        if (pending_space)
        {
            buf[w++] = ' ';
            pending_space = false;
        }
        buf[w++] = c;
    }
    buf[w] = '\0'; /* pending_space left unwritten == trailing trim */
}

/* Message body keeps its line breaks for the detail page: \r\n / \r -> \n,
   tab -> space, other ASCII control bytes dropped, force-terminated. One-line
   consumers (list card, dial header) flatten a copy at display time. */
static void notification_clean_block(char *buf, size_t cap)
{
    if (buf == NULL || cap == 0)
    {
        return;
    }
    buf[cap - 1] = '\0';
    size_t w = 0;
    for (size_t r = 0; r < cap - 1 && buf[r] != '\0'; r++)
    {
        unsigned char c = (unsigned char)buf[r];
        if (c == '\r')
        {
            if (buf[r + 1] != '\n')
                buf[w++] = '\n';
            continue;
        }
        if (c == '\t')
            c = ' ';
        if (c < 0x20 && c != '\n')
            continue;
        buf[w++] = (char)c;
    }
    buf[w] = '\0';
}

/* Single choke point for everything that reaches the notification centre.
   Every ingestion path (iOS ANCS, Android/SkaiLink JSON, SkaiApp, OTA hint)
   funnels through interact_with_notification()/trigger_incoming_call_ui(), so
   sanitising here covers paths added later for free. */
static void notification_sanitize(notification_t *notification)
{
    if (notification == NULL)
    {
        return;
    }
    notification_flatten_line(notification->title, sizeof(notification->title));
    notification_clean_block(notification->message,
                             sizeof(notification->message));
}

static bool need_wakeup = false;
void interact_with_notification(notification_t *notification)
{
    notification_sanitize(notification);
    /* Stamp the arrival wall-clock here, for every ingestion path. The callers'
       own `sec_time = SkaiWatchSys.SecondCountRTC` is dead: nothing ever
       writes SecondCountRTC, so it is always 0. A re-pushed duplicate keeps
       the original stamp (update_notification() returns before copying). */
    notification->sec_time = (uint32_t)time(RT_NULL);
    /* Snapshot the arrival seq so update_notification()'s internal verdict
       (previously-dismissed drop / identical reconnect re-push → seq
       untouched) is observable here — those must not light the screen. */
    uint32_t seq_before = notification_center_get_arrival_seq();
    if (!notification->calling)
    {
        update_notification(notification);
    }
    if (myLancher[app_index_instruction_list].reset_list != NULL &&
        !is_at_instruction_list())
    {
        myLancher[app_index_instruction_list].reset_list();
    }

    if (SkaiWatchSys.DNDMode.config.status) // DND mode
    {
        return;
    }

    if (!notification->calling &&
        notification_center_get_arrival_seq() == seq_before)
    {
        /* Not a genuine new arrival: refresh the list silently and skip both
           the wakeup and the is_hcpu_suspend() spin below — with no wakeup
           issued, that spin would block this (BLE RX) thread until something
           else happens to wake the watch. */
        notify_provider.notification_refresh();
        return;
    }

    watch_system_wakeup();

    notify_provider.notification_refresh();
    while (is_hcpu_suspend())
    {
        need_wakeup = true;
        rt_thread_mdelay(100);
    }
    if (notification->calling)
    {
        motor_pattern_calling();
        incoming_call_set_caller(notification->title, notification->id,
                                 notification->type);
        gui_app_run(APP_ID_INCOMING_CALL);
    }
    need_wakeup = false;
}

void trigger_incoming_call_ui(notification_t *notification)
{
    if (!notification)
    {
        return;
    }
    notification_sanitize(notification);

    if (SkaiWatchSys.DNDMode.config.status)
    {
        return;
    }

    watch_system_wakeup();
    while (is_hcpu_suspend())
    {
        need_wakeup = true;
        rt_thread_mdelay(100);
    }

    motor_pattern_calling();
    incoming_call_set_caller(notification->title, notification->id,
                             notification->type);
    gui_app_run(APP_ID_INCOMING_CALL);
    need_wakeup = false;
}

void handle_notification(uint8_t notify_id, char *json_string)
{
    LOG_D("handle_notification, notify_id:%d, msgData:%s", notify_id,
          json_string);
    /* Clear BEFORE parsing, and bail on failure. `temp_notification` is static:
       reusing its stale contents replays the previous notification — with
       `calling` still set, an unparseable YouTube notification arriving during
       a LINE call popped a phantom incoming-call screen. */
    rt_memset(&temp_notification, 0, sizeof(temp_notification));
    if (!parse_notification(json_string, &temp_notification))
    {
        LOG_E("handle_notification: dropped unparseable notification");
        return;
    }

    temp_notification.sec_time = SkaiWatchSys.SecondCountRTC;
    temp_notification.type = notify_id;
    temp_notification.state = true;
    // app_message_set_from_temp(&temp_notification);
    interact_with_notification(&temp_notification);
}


static void remove_notification(notification_t *notifications, uint8_t *size,
                                const char *id)
{
    int output_index = 0;
    int count = 0;
    LOG_D("[%s] id:%s", __func__, id);
    for (int i = 0; i < *size; i++)
    {
        if (strcmp(notifications[i].id, id) != 0)
        {
            notifications[i].state = true;
            notifications[output_index++] = notifications[i];
        }
        else
        {
            count++;
            // TODO: disable notification widget
            // extern void DISABLE_NOTIFICATION_WIDGET(uint8_t index);
            // DISABLE_NOTIFICATION_WIDGET(*size - count);
        }
    }
    *size = output_index;
}

/* Reply module's GUI-thread commit: never dismiss another notification revision. */
bool notification_reply_remove_version(const char *instance, uint32_t revision)
{
    bool removed = false;
    notification_store_lock();
    for (unsigned i = 0; i < notification_items_amount; i++) {
        notification_t *item = &_notification_list[i];
        if (item->reply_revision != revision || strcmp(item->reply_instance, instance)) continue;
        if (item->calling) incoming_call_close_if_active(item->id);
        for (unsigned next = i + 1; next < notification_items_amount; next++)
            _notification_list[next - 1] = _notification_list[next];
        notification_items_amount--;
        SkaiWatchSys.notification_number = notification_items_amount;
        removed = true;
        break;
    }
    notification_store_unlock();
    if (removed) notify_provider.notification_refresh();
    return removed;
}

/**
 * @brief Remove a notification by ID from the watch list and notify the phone.
 *        This is the single entry point for watch-initiated dismissals.
 */
void remove_notification_by_id(const char *id)
{
    if (!id || !id[0]) return;
    notification_t match;
    bool found = false, ambiguous = false;
    notification_store_lock();
    for (unsigned i = 0; i < notification_items_amount; i++) {
        if (strcmp(_notification_list[i].id, id)) continue;
        if (found) { ambiguous = true; break; }
        match = _notification_list[i]; found = true;
    }
    notification_store_unlock();
    if (found && !ambiguous) notification_reply_dismiss(&match);
}

/* Legacy compatibility adapter, called only by the complete dismiss operation. */
bool notification_reply_remove_legacy(const notification_t *notification)
{
    if (!notification || notification->reply_revision || !notification->id[0]) return false;
    char id[NOTIFICATION_ID_LEN] = {0};
    bool removed = false;
    notification_store_lock();
    for (unsigned i = 0; i < notification_items_amount; i++) {
        const notification_t *item = &_notification_list[i];
        if (item->reply_revision || strcmp(item->id, notification->id) ||
            item->sec_time != notification->sec_time || strcmp(item->title, notification->title) ||
            strcmp(item->message, notification->message)) continue;
        memcpy(id, item->id, sizeof(id));
        bloc_notification_mark_dismissed(id);
        for (unsigned next = i + 1; next < notification_items_amount; next++)
            _notification_list[next - 1] = _notification_list[next];
        notification_items_amount--;
        SkaiWatchSys.notification_number = notification_items_amount;
        removed = true;
        break;
    }
    notification_store_unlock();
    if (removed) { commu_send_dismiss_notification(id); notify_provider.notification_refresh(); }
    return removed;
}

/**
 * @brief Phone dismissed a notification — remove it from watch list only.
 *        Called from communicate_parse_notify when KEY_DISMISS_NOTIFICATION
 *        is received. Does NOT send back to phone (would cause loop).
 */
void dismiss_notification_from_phone(const char *id)
{
    if (!id || strlen(id) == 0)
    {
        LOG_E("[%s]id is empty", __func__);
        return;
    }
    /* Modern notifications need a versioned removal on 0x20/0x30. A delayed
       legacy dismiss must never remove a replacement under the same old ID. */
    notification_store_lock();
    for (unsigned i = 0; i < notification_items_amount; i++)
        if (!strcmp(_notification_list[i].id, id) && _notification_list[i].reply_revision) { notification_store_unlock(); return; }
    LOG_I("[%s] phone dismiss → watch remove id:%s", __func__, id);
    /* Same reason as remove_notification_by_id: persist the dismiss so a
     * later reconnect re-push from the phone doesn't re-pop. */
    bloc_notification_mark_dismissed(id);
    incoming_call_close_if_active(id);
    remove_notification(_notification_list, &notification_items_amount, id);
    SkaiWatchSys.notification_number = notification_items_amount;
    notification_store_unlock();
    notify_provider.notification_refresh();
}

static void generate_json_for_gpt_message(const char *message)
{
    temp_send_json_string[0] = '\0';
    cJSON *obj = cJSON_CreateObject();
    if (!obj)
    {
        return;
    }
    cJSON_AddStringToObject(obj, "input", message);
    char *json = cJSON_PrintUnformatted(obj);
    if (json)
    {
        strncpy(temp_send_json_string, json, sizeof(temp_send_json_string) - 1);
        temp_send_json_string[sizeof(temp_send_json_string) - 1] = '\0';
        cJSON_free(json);
    }
    cJSON_Delete(obj);
}

static void send_message_to_chatgpt(const char *message)
{
    if (strlen(message) == 0)
    {
        return;
    }
    generate_json_for_gpt_message(message);
    commu_send_chat_with_ai(temp_send_json_string);
}

static void set_user_speech_text(char *text)
{
    strncpy(temp_speech_text, text, sizeof(temp_speech_text) - 1);
    temp_speech_text[sizeof(temp_speech_text) - 1] = '\0';
}

void handle_user_speech_intent(uint8_t intent, char *message)
{
    if (intent == V2T_INTENT_REMOTE_INPUT) {
        LOG_W("Notification replies require a fixed input context");
        return;
    }
    if (message == NULL || strlen(message) == 0)
    {
        LOG_E("[%s]message is empty", __func__);
        return;
    }
    LOG_I("[%s]intent:%d, message:%s", __func__, intent, message);
    set_user_speech_text(message);
    switch (intent)
    {
    case V2T_INTENT_CHAT:
    {
#ifdef BSP_USING_BLOC_NOTIFY
        send_message_to_chatgpt(temp_speech_text);
#endif
        break;
    }

    default:
        LOG_E("Unknown user speech intent");
        break;
    }
}

#ifdef BSP_USING_UI_HANDLER
static void bloc_notify_accelerometer(float x, float y, float z)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_IMU_ACC;
    msg.data.imu_acc.x = x;
    msg.data.imu_acc.y = y;
    msg.data.imu_acc.z = z;
    lvgl_send_msg(msg);
}

static void bloc_notify_gyroscope(float x, float y, float z)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_IMU_GYRO;
    msg.data.imu_gyro.x = x;
    msg.data.imu_gyro.y = y;
    msg.data.imu_gyro.z = z;
    lvgl_send_msg(msg);
}

static void bloc_notify_attitude(float roll, float pitch, float yaw)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_IMU_ATTITUDE;
    msg.data.imu_attitude.x = roll;
    msg.data.imu_attitude.y = pitch;
    msg.data.imu_attitude.z = yaw;
    lvgl_send_msg(msg);
}

static void bloc_notify_hr(int hr)
{
    if (hr == 0 || hr > 220)
    {
        return;
    }

    LOG_D("bloc_notify_hr:%d", hr);
    SkaiWatchSys.heart_rate_bpm = hr;
    extern void app_exercise_background_hr_cb(int hr);
    app_exercise_background_hr_cb(hr);

    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_HR;
    msg.data.hr = hr;
    lvgl_send_msg(msg);

    #ifdef BSP_USING_COMMUNICATE
    commu_send_heart_data(hr);
    #endif
}

static void bloc_notify_battery_voltage(uint16_t voltage)
{
    /* Forward only on a meaningful change. The raw ADC voltage jitters by a few
     * mV every read; forwarding every read flooded BOTH the LVGL queue and the
     * GUI-app mailbox — the latter used to RT_ASSERT when full, crashing the
     * watch (peripher-thread WDT/hardfault captured 2026-08). A 16 mV deadband
     * keeps the widget/phone current without the flood. */
    static uint16_t last_voltage = 0;
    uint16_t diff = (voltage > last_voltage) ? (voltage - last_voltage)
                                             : (last_voltage - voltage);
    if (last_voltage != 0 && diff < 16)
        return;
    last_voltage = voltage;

    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_BATTERY_VOLTAGE;
    msg.data.battery_voltage = voltage;
    lvgl_send_msg(msg);

    commu_send_battery_voltage(voltage);
}

static void bloc_notify_battery_level(uint8_t level)
{
    /* Level is coarse (0-100) and changes slowly; forward only on a real change
     * so it stays off the flooded queues (see bloc_notify_battery_voltage). */
    static uint8_t last_level = 0xFF;
    if (level == last_level)
        return;
    last_level = level;

    extern void refersh_battery(uint8_t battery_level);
    refersh_battery(level);

    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_BATTERY_LEVEL;
    msg.data.battery_level = level;
    lvgl_send_msg(msg);

    commu_send_battery_level(level);
}

// 跳出充電狀頁面
static void bloc_notify_charge_status(uint8_t status)
{
    refresh_charge_icon();
    commu_send_charge_status();

    #if CHARGE_INTERACT_ENABLE
    if (status > NoCharge)
    {
        gui_app_run(APP_ID_BATTERY);
    }
    else
    {
        watch_exit_app(APP_ID_BATTERY);
    }
    #endif
}

// static rt_timer_t notification_timer = RT_NULL;

static void send_refresh_notification_cmd(void)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_NOTIFICATION;
    msg.data.notification = NULL; // get_cur_notification();
    lvgl_send_msg(msg);
}

// 導航至回覆通知頁面
static void bloc_notification_navigate_to_reply(notification_t *notification)
{
    notification_reply_view_t view = notification_reply_open(notification);
    if (!view.input) { ui_show_hint_toast("無法回覆，請確認通知與手機版本"); return; }
    intent_t intent = intent_init("speech");
    intent_set_string(intent, "intent", "reply");
    intent_set_string(intent, "title", notification->title);
    intent_set_uint32(intent, "reply_input", view.input);
    if (view.draft) intent_set_string(intent, "reply_draft", view.draft);
    intent_set_string(intent, "reply_instance", notification->reply_instance);
    intent_set_uint32(intent, "reply_revision", notification->reply_revision);
    intent_runapp(intent);
}

// 刷新APP列表上方的時間
static void bloc_notify_time(void *parameter)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_TIME_TEXT;
    lvgl_send_msg(msg);
}

// 刷新錶盤上方的藍芽配對狀態
static void bloc_notify_bluetooth_connection(void)
{
    bool status = SkaiWatchSys.connected_to_phone;
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_BLUETOOTH_CONNECTION;
    msg.data.bluetooth_connection = status;
    lvgl_send_msg(msg);
}

static void bloc_notify_holding_displacement(uint8_t event, int x, int y)
{
    (void)event;
    (void)x;
    (void)y;
}

/* NotifyProvider global instance */
NotifyProvider notify_provider = {
    .accelerometer = bloc_notify_accelerometer,
    .gyroscope = bloc_notify_gyroscope,
    .attitude = bloc_notify_attitude,
    .hr = bloc_notify_hr,
    .battery_voltage = bloc_notify_battery_voltage,
    .battery_level = bloc_notify_battery_level,
    .charge_status = bloc_notify_charge_status,
    .notification_refresh = send_refresh_notification_cmd,
    .navigate_to_reply = bloc_notification_navigate_to_reply,
    .time_refresh = bloc_notify_time,
    .bluetooth_connection = bloc_notify_bluetooth_connection,
    .holding_displacement =
        bloc_notify_holding_displacement, // Placeholder for holding
                                          // displacement function
};

#if !kReleaseMode
static int bloc_notification_test(int argc, char *argv[])
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "send_noti") == 0)
        {
            notification_t notification;
            notification.sec_time = SkaiWatchSys.SecondCountRTC;
            /* optional 3rd arg = notification app type, so the per-app logo
               path (files under /assets/icons/) can be exercised too */
            notification.type = (argc >= 3) ? (uint8_t)atoi(argv[2]) : Notify_others;
            notification.state = true;
            strcpy(notification.id, "ota_start");
            strcpy(notification.title, "OTA Update hint");
            strcpy(notification.message,
                   "Please update your smartphone app to the latest version to "
                   "ensure compatibility.");
            interact_with_notification(&notification);
        }
        else if (strcmp(argv[1], "send_noti_long") == 0)
        {
            /* Title wider than the card's title row: the list card must
               truncate it with "..." and still leave room for the arrival
               time on the right. */
            notification_t notification;
            memset(&notification, 0, sizeof(notification));
            notification.type = Notify_others;
            notification.state = true;
            strcpy(notification.id, "long_title_probe");
            strcpy(notification.title,
                   "VeryLongNotificationTitleThatOverflowsTheLabelWidth");
            strcpy(notification.message, "Body of the long-title probe.");
            interact_with_notification(&notification);
        }
        else if (strcmp(argv[1], "send_noti_nl") == 0)
        {
            /* Multi-line fixture: the dial header's body label runs
               SCROLL_CIRCULAR, which still honours a literal '\n'. Before
               notification_sanitize() this froze the header as two static
               lines instead of a marquee. Keep the payload long enough that
               the flattened single line overflows the 240 px label, so a pass
               is visibly a marquee and not merely a short static line. */
            notification_t notification;
            notification.sec_time = SkaiWatchSys.SecondCountRTC;
            notification.type = Notify_others;
            notification.state = true;
            notification.can_reply = false;
            notification.calling = false;
            strcpy(notification.id, "nl_probe");
            strcpy(notification.title, "Line\nBreak\tProbe");
            strcpy(notification.message,
                   "  first line of the body\n\n"
                   "second line follows\r\n"
                   "third line trails  ");
            interact_with_notification(&notification);
            LOG_W("nl probe title=[%s]", notification.title);
            LOG_W("nl probe message=[%s]", notification.message);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(bloc_notification_test, "bloc_notification_test [OPTION] ...");
#endif
#endif

/* ===================================================================== */
/*  Dismissed-ID ring (Bug 3 fix — see header comment for design)        */
/* ===================================================================== */

/* 32 * 48B = 1.5 KB. Held at 32 (vs 64) to leave headroom in the 16 KB
 * "watch" share_prefs partition, which already carries flag_field,
 * msg_switch, phone_os_version, alarms etc. and needs room for FDB's
 * pre-write-then-delete cycle. 32 dismiss-ids covers typical "few
 * dismissals between reboots" usage. */
#define DISMISSED_RING_SIZE 32
#define DISMISSED_PREFS_NAME "watch"
#define DISMISSED_PREFS_KEY  "ntfdis"  /* short to keep KVDB key small */

typedef struct {
    uint32_t magic;     /* 0xD150D1D7 — validate flash blob */
    uint8_t  head;      /* next slot to write */
    uint8_t  count;     /* 0..DISMISSED_RING_SIZE */
    uint8_t  reserved[2];
    char     ids[DISMISSED_RING_SIZE][NOTIFICATION_ID_LEN];
} dismissed_ring_t;

#define DISMISSED_MAGIC 0xD150D1D7u

static dismissed_ring_t s_dismissed;
static bool             s_dismissed_loaded = false;
static bool             s_dismissed_dirty  = false;

static void dismissed_ensure_loaded(void)
{
    if (s_dismissed_loaded) return;
    /* Default-init in case load skips or fails: empty ring, valid magic. */
    memset(&s_dismissed, 0, sizeof(s_dismissed));
    s_dismissed.magic = DISMISSED_MAGIC;
    s_dismissed_loaded = true;
}

bool bloc_notification_is_dismissed(const char *id)
{
    if (!id || !id[0]) return false;
    dismissed_ensure_loaded();
    /* Linear scan over up to 64 entries — fine for a per-notification check. */
    for (uint8_t i = 0; i < s_dismissed.count; i++)
    {
        if (strncmp(s_dismissed.ids[i], id, NOTIFICATION_ID_LEN) == 0)
            return true;
    }
    return false;
}

void bloc_notification_mark_dismissed(const char *id)
{
    if (!id || !id[0]) return;
    dismissed_ensure_loaded();
    if (bloc_notification_is_dismissed(id)) return;   /* idempotent */

    /* Write at head, overwriting the oldest entry when full. */
    uint8_t slot = s_dismissed.head;
    memset(s_dismissed.ids[slot], 0, NOTIFICATION_ID_LEN);
    strncpy(s_dismissed.ids[slot], id, NOTIFICATION_ID_LEN - 1);
    s_dismissed.head = (uint8_t)((slot + 1) % DISMISSED_RING_SIZE);
    if (s_dismissed.count < DISMISSED_RING_SIZE) s_dismissed.count++;
    s_dismissed_dirty = true;
    LOG_D("dismissed id=%s slot=%u count=%u", id, slot, s_dismissed.count);
}

void bloc_notification_clear_dismissed(void)
{
    dismissed_ensure_loaded();
    memset(s_dismissed.ids, 0, sizeof(s_dismissed.ids));
    s_dismissed.head = 0;
    s_dismissed.count = 0;
    s_dismissed_dirty = true;
}

/* Called from watch_global_data.c during watch_config_struct_flash_read().
 * The caller has already opened the shared "watch" prefs handle and pays
 * the one-time fdb_kvdb_init cost on the NAND prefdb partition; we just
 * read one block out of it. void* in the signature mirrors the .h forward
 * decl so the public header doesn't need to pull share_prefs.h. */
void bloc_notification_read_dismissed(void *pref_handle)
{
    share_prefs_t *pref = (share_prefs_t *)pref_handle;

    /* Pre-seed defaults; flash read may overwrite or leave them alone. */
    memset(&s_dismissed, 0, sizeof(s_dismissed));
    s_dismissed.magic = DISMISSED_MAGIC;
    s_dismissed_loaded = true;
    s_dismissed_dirty  = false;

    if (!pref) { LOG_W("dismissed: read called with NULL pref"); return; }

    dismissed_ring_t tmp;
    int32_t got = share_prefs_get_block(pref, DISMISSED_PREFS_KEY,
                                        &tmp, (int32_t)sizeof(tmp));
    if (got != (int32_t)sizeof(tmp) || tmp.magic != DISMISSED_MAGIC)
    {
        LOG_I("dismissed: no/invalid blob (got=%d expect=%u), starting empty",
              got, (unsigned)sizeof(tmp));
        return;
    }
    if (tmp.count > DISMISSED_RING_SIZE || tmp.head >= DISMISSED_RING_SIZE)
    {
        LOG_W("dismissed: corrupted indices (head=%u count=%u), discarding",
              tmp.head, tmp.count);
        return;
    }
    s_dismissed = tmp;
    LOG_I("dismissed: loaded %u ids from flash", s_dismissed.count);
}

void bloc_notification_write_dismissed(void *pref_handle)
{
    share_prefs_t *pref = (share_prefs_t *)pref_handle;
    if (!pref) { LOG_E("dismissed: write called with NULL pref"); return; }
    rt_err_t r = share_prefs_set_block(pref, DISMISSED_PREFS_KEY,
                                       &s_dismissed,
                                       (int32_t)sizeof(s_dismissed));
    if (r == RT_EOK)
    {
        s_dismissed_dirty = false;
        LOG_I("dismissed: wrote %u ids to flash (blob=%u B)",
              s_dismissed.count, (unsigned)sizeof(s_dismissed));
    }
    else
    {
        LOG_E("dismissed: write failed (r=%d, blob=%u B)",
              (int)r, (unsigned)sizeof(s_dismissed));
    }
}

/* OTA-time hook: skip if nothing changed, else delegate to the shared
 * "watch" prefs path via store_watch_prefs(). That uses the cached pref
 * handle from watch_global_data's open_watch_prefs() helper, so by the
 * time OTA verify success runs we're not re-initializing FlashDB. */
void bloc_notification_save_dismissed_to_flash(void)
{
    if (!s_dismissed_loaded || !s_dismissed_dirty)
    {
        LOG_D("dismissed: save skipped (loaded=%d dirty=%d)",
              (int)s_dismissed_loaded, (int)s_dismissed_dirty);
        return;
    }
    store_watch_prefs(WATCH_PREFS_KEY_DISMISSED_NOTIFICATIONS);
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
