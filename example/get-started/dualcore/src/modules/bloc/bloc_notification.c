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
#include <rtthread.h>

/* Bloc modules */
#include "bloc_notification.h"
#include "bloc_control.h"
#include "bloc_skaiwalk.h"
#include "bloc_v2t.h"
#include "bloc_peripheral.h"
#include "gui_app_fwk.h"

/* Communication modules */
#include "communicate_protocol.h"

/* System and data modules */
#include "watch_global_data.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
    #include "watch_system_core_task.h"
#endif
#include "app_mainmenu.h"

/* UI modules */
#include "gui_app_pm.h"
#include "ui_handler.h"
#include "gui_app_fwk.h"
#include "app_message.h"
#include "app_clock_main.h"
#include "ui_helper.h"
#include "gui_app_int.h"

/* Utilities */
#include "cJSON.h"

#define DBG_TAG "bloc.notification"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#if CUSTOMER_BOARD_VER > BOARD_VER_28
    #define CHARGE_INTERACT_ENABLE 1
#else
    #define CHARGE_INTERACT_ENABLE 0
#endif

extern void incoming_call_set_caller(const char *title, const char *id,
                                     uint8_t type);
extern void incoming_call_close_if_active(const char *id);

static char temp_send_json_string[512];

static char temp_speech_text[512];

char replying_notification_id[NOTIFICATION_ID_LEN];

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

extern void motor_pattern_calling(void);

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

/**
 * @brief Get pointer to the first notification in the list
 * @return Pointer to the first notification
 */
notification_t *notification_center_get_header(void)
{
    return &_notification_list[0];
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

void set_notification(notification_t notification, int index)
{
    notification.index = index + 1;
    _notification_list[index] = notification;
    notifyNotification(index);
}

notification_t *get_cur_notification(void)
{
    return &_notification_list[selected_notification_index];
}

static void update_notification(notification_t newNotification)
{
    // Dedup: if a notification with the same ID already exists, remove it first
    // so re-synced notifications don't create duplicates.
    // For Notify_Skaiwalk category, also dedup by type to keep only the latest
    // one.
    bool dup = false;
    for (int i = notification_items_amount - 1; i >= 0; i--)
    {
        dup = (strcmp(_notification_list[i].id, newNotification.id) == 0);
        if (!dup && newNotification.type == Notify_Skaiwalk &&
            _notification_list[i].type == Notify_Skaiwalk)
        {
            dup = true;
        }
        if (dup)
        {
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
            notification_t prev_notification =
                *get_notification(notification_items_amount - i - 1);
            set_notification(prev_notification, notification_items_amount - i);
        }
    }

    set_notification(newNotification, 0);

    if (notification_items_amount < ITEM_AMOUNT_NOTIFICATION)
    {
        notification_items_amount++;
    }

    SkaiWatchSys.notification_number = notification_items_amount;
}

/**
 * @brief Parse notification data from JSON string
 * @param json_str JSON string containing notification data
 * @param notification Pointer to notification structure to populate
 */
static void parse_notification(const char *json_str,
                               notification_t *notification)
{
    if (!json_str || !notification)
    {
        LOG_E("parse_notification: invalid parameters");
        return;
    }

    LOG_D("parse notification:%s", json_str);

    cJSON *root = cJSON_Parse(json_str);
    if (!root)
    {
        LOG_E("parse_notification: JSON parsing failed");
        return;
    }

    cJSON *id_json = cJSON_GetObjectItem(root, "id");
    cJSON *title_json = cJSON_GetObjectItem(root, "title");
    cJSON *message_json = cJSON_GetObjectItem(root, "message");
    cJSON *reply_json = cJSON_GetObjectItem(root, "reply");
    cJSON *calling_json = cJSON_GetObjectItem(root, "calling");
    notification->calling = calling_json ? cJSON_IsTrue(calling_json) : false;

    if (id_json && id_json->valuestring)
    {
        strncpy(notification->id, id_json->valuestring,
                sizeof(notification->id) - 1);
        notification->id[sizeof(notification->id) - 1] = '\0';
    }
    else
    {
        strcpy(notification->id, "unknown");
    }

    if (title_json && title_json->valuestring)
    {
        strncpy(notification->title, title_json->valuestring,
                sizeof(notification->title) - 1);
        notification->title[sizeof(notification->title) - 1] = '\0';
    }
    else
    {
        strcpy(notification->title, "Unknown");
    }

    if (message_json && message_json->valuestring)
    {
        strncpy(notification->message, message_json->valuestring,
                sizeof(notification->message) - 1);
        notification->message[sizeof(notification->message) - 1] = '\0';
    }
    else
    {
        strcpy(notification->message, "");
    }

    notification->can_reply = reply_json ? cJSON_IsTrue(reply_json) : false;

    LOG_D("id:%s", notification->id);
    LOG_D("title:%s", notification->title);
    LOG_D("message:%s", notification->message);
    LOG_D("reply:%d", notification->can_reply);

    cJSON_Delete(root);
}

const char *get_app_name_from_notify_id(Notifications_Type notify_id)
{
    switch (notify_id)
    {
    case Notify_calendar:
        return "Calendar";
    case Notify_facebook:
        return "Facebook";
    case Notify_facetime:
        return "FaceTime";
    case Notify_instagram:
        return "Instagram";
    case Notify_kakaotalk:
        return "KakaoTalk";
    case Notify_line:
        return "Line";
    case Notify_linkedin:
        return "LinkedIn";
    case Notify_mail:
        return "Mail";
    case Notify_messenger:
        return "Messenger";
    case Notify_others:
        return "Others";
    case Notify_QQ:
        return "QQ";
    case Notify_skype:
        return "Skype";
    case Notify_SMS:
        return "SMS";
    case Notify_snap:
        return "Snap";
    case Notify_twitter:
        return "Twitter";
    case Notify_wechat:
        return "WeChat";
    case Notify_whatsapp:
        return "WhatsApp";
    case Notify_gmail:
        return "Gmail";
    case Notify_dingtalk:
        return "DingTalk";
    case Notify_googlechat:
        return "Google Chat";
    case Notify_discord:
        return "Discord";
    case Notify_youtube:
        return "YouTube";
    case Notify_tiktok:
        return "TikTok";
    case Notify_telegram:
        return "Telegram";
    case Notify_twitch:
        return "Twitch";
    case Notify_slack:
        return "Slack";
    case Notify_lark:
        return "Lark";
    case Notify_reddit:
        return "Reddit";
    case Notify_Skaiwalk:
        return "Skaiwalk";
    default:
        return "Unknown";
    }
}

uint8_t get_notification_type_from_ios_ancs_name(const char *name)
{
    if (strstr(name, "Calendar") != NULL)
    {
        return Notify_calendar;
    }
    else if (strstr(name, "Facebook") != NULL)
    {
        return Notify_facebook;
    }
    else if (strstr(name, "FaceTime") != NULL)
    {
        return Notify_facetime;
    }
    else if (strstr(name, "Instagram") != NULL)
    {
        return Notify_instagram;
    }
    else if (strstr(name, "KakaoTalk") != NULL)
    {
        return Notify_kakaotalk;
    }
    else if (strstr(name, "Line") != NULL)
    {
        return Notify_line;
    }
    else if (strstr(name, "LinkedIn") != NULL)
    {
        return Notify_linkedin;
    }
    else if (strstr(name, "Mail") != NULL)
    {
        return Notify_mail;
    }
    else if (strstr(name, "Messenger") != NULL)
    {
        return Notify_messenger;
    }
    else if (strstr(name, "QQ") != NULL)
    {
        return Notify_QQ;
    }
    else if (strstr(name, "Skype") != NULL)
    {
        return Notify_skype;
    }
    else if (strstr(name, "SMS") != NULL)
    {
        return Notify_SMS;
    }
    else if (strstr(name, "Snap") != NULL)
    {
        return Notify_snap;
    }
    else if (strstr(name, "Twitter") != NULL)
    {
        return Notify_twitter;
    }
    else if (strstr(name, "WeChat") != NULL)
    {
        return Notify_wechat;
    }
    else if (strstr(name, "WhatsApp") != NULL)
    {
        return Notify_whatsapp;
    }
    else if (strstr(name, "Gmail") != NULL)
    {
        return Notify_gmail;
    }
    else if (strstr(name, "DingTalk") != NULL)
    {
        return Notify_dingtalk;
    }
    else if (strstr(name, "Google Chat") != NULL)
    {
        return Notify_googlechat;
    }
    else if (strstr(name, "Discord") != NULL)
    {
        return Notify_discord;
    }
    else if (strstr(name, "YouTube") != NULL)
    {
        return Notify_youtube;
    }
    else if (strstr(name, "TikTok") != NULL)
    {
        return Notify_tiktok;
    }
    else if (strstr(name, "Telegram") != NULL)
    {
        return Notify_telegram;
    }
    else if (strstr(name, "Twitch") != NULL)
    {
        return Notify_twitch;
    }
    else if (strstr(name, "Slack") != NULL)
    {
        return Notify_slack;
    }
    else if (strstr(name, "Lark") != NULL)
    {
        return Notify_lark;
    }
    else if (strstr(name, "Reddit") != NULL)
    {
        return Notify_reddit;
    }
    else if (strstr(name, "Skaiwalk") != NULL)
    {
        return Notify_Skaiwalk;
    }
    else
    {
        return Notify_others;
    }
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

static bool need_wakeup = false;
void interact_with_notification(notification_t *notification)
{
    if (!notification->calling)
    {
        update_notification(*notification);
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

    watch_system_interact(HCPU_WAKEUP, NULL);

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

    if (SkaiWatchSys.DNDMode.config.status)
    {
        return;
    }

    watch_system_interact(HCPU_WAKEUP, NULL);
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
    parse_notification(json_string, &temp_notification);

    temp_notification.sec_time = SkaiWatchSys.SecondCountRTC;
    temp_notification.type = notify_id;
    temp_notification.state = true;
    // app_message_set_from_temp(&temp_notification);
    interact_with_notification(&temp_notification);
}

void init_notification_items(void)
{
    for (uint8_t i = 0; i < SkaiWatchSys.notification_number; i++)
    {
        if (get_notification(i)->index != 0)
        {
            notification_items_amount++;
            notifyNotification(i);
        }
    }
}

void get_notification_list_from_template(void)
{
    for (uint8_t i = 0;
         i < sizeof(notifications_constant) / sizeof(notification_t); i++)
    {
        LOG_D("notification(%d) is in constant resources", i);
        _notification_list[i] = notifications_constant[i];
        notification_items_amount++;
    }
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

static void generate_json_for_remote_input(const char *message, const char *id)
{
    strcpy(temp_send_json_string, "");
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "id", id);
    cJSON_AddStringToObject(obj, "m", message);
    strcpy(temp_send_json_string, cJSON_PrintUnformatted(obj));
    cJSON_Delete(obj);
}

static void remote_input_to_notification(const char *id, const char *message)
{
    if (strlen(message) == 0)
    {
        LOG_E("[%s]message is empty", __func__);
        return;
    }
    if (strlen(id) == 0)
    {
        LOG_E("[%s]id is empty", __func__);
        return;
    }
    LOG_D("[%s]id:%s, message:%s", __func__, id, message);
    generate_json_for_remote_input(message, id);
    L1SendData data;
    data.event = L1SEND_REMOTE_INPUT;
    data.res.json_string_ptr = temp_send_json_string;
    L1_send_event(data);
    remove_notification(_notification_list, &notification_items_amount, id);
    notify_provider.notification_refresh();
}

/**
 * @brief Remove a notification by ID from the watch list and notify the phone.
 *        This is the single entry point for watch-initiated dismissals.
 */
void remove_notification_by_id(const char *id)
{
    if (!id || strlen(id) == 0)
    {
        LOG_E("[%s]id is empty", __func__);
        return;
    }
    LOG_I("[%s] id:%s", __func__, id);

    /* Send dismiss to phone via BLE */
    strncpy(temp_send_json_string, id, sizeof(temp_send_json_string) - 1);
    temp_send_json_string[sizeof(temp_send_json_string) - 1] = '\0';

    L1SendData data;
    data.event = L1SEND_DISMISS_NOTIFICATION;
    data.res.json_string_ptr = temp_send_json_string;
    L1_send_event(data);

    /* Remove locally */
    remove_notification(_notification_list, &notification_items_amount, id);
    SkaiWatchSys.notification_number = notification_items_amount;
    notify_provider.notification_refresh();
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
    LOG_I("[%s] phone dismiss → watch remove id:%s", __func__, id);
    incoming_call_close_if_active(id);
    remove_notification(_notification_list, &notification_items_amount, id);
    SkaiWatchSys.notification_number = notification_items_amount;
    notify_provider.notification_refresh();
}

static void generate_json_for_gpt_message(const char *message)
{
    strcpy(temp_send_json_string, "");
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "input", message);
    strcpy(temp_send_json_string, cJSON_PrintUnformatted(obj));
    cJSON_Delete(obj);
}

static void send_message_to_chatgpt(const char *message)
{
    if (strlen(message) == 0)
    {
        return;
    }
    generate_json_for_gpt_message(message);
    L1SendData data;
    data.event = L1SEND_CHAT_WITH_AI;
    data.res.json_string_ptr = temp_send_json_string;
    L1_send_event(data);
}

static void generate_json_for_note_message(const char *message)
{
    strcpy(temp_send_json_string, "");
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "content", message);
    strcpy(temp_send_json_string, cJSON_PrintUnformatted(obj));
    cJSON_Delete(obj);
}

static void send_message_to_note(const char *message)
{
    if (strlen(message) == 0)
    {
        return;
    }
    generate_json_for_note_message(message);
    L1SendData data;
    data.event = L1SEND_CREATE_NOTE;
    data.res.json_string_ptr = temp_send_json_string;
    L1_send_event(data);
}

static void set_user_speech_text(char *text)
{
    strcpy(temp_speech_text, text);
}

void handle_user_speech_intent(uint8_t intent, char *message)
{
    if (message == NULL || strlen(message) == 0)
    {
        LOG_E("[%s]message is empty", __func__);
        return;
    }
    LOG_I("[%s]intent:%d, message:%s", __func__, intent, message);
    set_user_speech_text(message);
    switch (intent)
    {
    case V2T_INTENT_REMOTE_INPUT:
    {
#ifdef BSP_USING_BLOC_NOTIFY
        remote_input_to_notification(replying_notification_id,
                                     temp_speech_text);
#endif
        break;
    }
    case V2T_INTENT_CHAT:
    {
#ifdef BSP_USING_BLOC_NOTIFY
        send_message_to_chatgpt(temp_speech_text);
#endif
        break;
    }
    case V2T_INTENT_NOTE_CREATING:
    {
#if defined(APP_ID_NOTE_CHATROOM)
        append_text_to_latest_message(get_note_list(), skai_note_count_ptr(),
                                      temp_speech_text);

        // Save note list after adding new note
        save_note_list_to_file();
        send_message_to_note(temp_speech_text);
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
    L1SendData commuData;
    commuData.event = L1SEND_HEART_DATA;
    commuData.res.hr = hr;
    L1_send_event(commuData);
    #endif
}

static void bloc_notify_battery_voltage(uint16_t voltage)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_BATTERY_VOLTAGE;
    msg.data.battery_voltage = voltage;
    lvgl_send_msg(msg);

    L1SendData data;
    data.event = L1SEND_RETURN_BATTERY_VOLTAGE;
    data.res.battery_voltage = voltage;
    L1_send_event(data);
}

static void bloc_notify_battery_level(uint8_t level)
{
    extern void refersh_battery(uint8_t battery_level);
    refersh_battery(level);

    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_BATTERY_LEVEL;
    msg.data.battery_level = level;
    lvgl_send_msg(msg);

    L1SendData data;
    data.event = L1SEND_RETURN_BATTERY_LEVEL;
    data.res.battery_level = level;
    L1_send_event(data);
}

// 跳出充電狀頁面
static void bloc_notify_charge_status(uint8_t status)
{
    refresh_charge_icon();
    L1SendData data;
    data.event = L1SEND_RETURN_CHARGE_STATUS;
    L1_send_event(data);

    #if CHARGE_INTERACT_ENABLE
    if (status > NoCharge)
    {
        // uint8_t led_brightness = 20;
        // if (status == InCharging)
        // {
        //     watch_system_interact(INTERACT_RGB_LED_BREATHING_GREEN,
        //                           &led_brightness);
        // }
        // else if (status == ChargingComplete)
        // {
        //     watch_system_interact(INTERACT_RGB_LED_OPEN_GREEN,
        //     &led_brightness);
        // }
        gui_app_run(APP_ID_BATTERY);
    }
    else
    {
        // watch_system_interact(INTERACT_RGB_LED_CLOSE, NULL);
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
    strcpy(replying_notification_id, notification->id);
    intent_t intent = intent_init("speech");
    intent_set_string(intent, "intent", "reply");
    intent_set_string(intent, "title", notification->title);
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
    static int last_x = 0;
    static int last_y = 0;
    if (x == last_x && y == last_y && event == 1)
    {
        return; // No change in displacement
    }
    last_x = x;
    last_y = y;
    L1SendData data;
    data.event = L1SEND_HOLDING_DISPLACEMENT;
    data.res.holding_displacement.event = event;
    data.res.holding_displacement.x = x;
    data.res.holding_displacement.y = y;
    // LOG_D("bloc_notify_holding_displacement: event:%d, x:%d, y:%d", event, x,
    // y);
    L1_send_event(data);
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

static int bloc_notification_test(int argc, char *argv[])
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "send_noti") == 0)
        {
            notification_t notification;
            notification.sec_time = SkaiWatchSys.SecondCountRTC;
            notification.type = Notify_others;
            notification.state = true;
            strcpy(notification.id, "ota_start");
            strcpy(notification.title, "OTA Update hint");
            strcpy(notification.message,
                   "Please update your smartphone app to the latest version to "
                   "ensure compatibility.");
            interact_with_notification(&notification);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(bloc_notification_test, "bloc_notification_test [OPTION] ...");
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
