/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "string.h"
#ifdef RT_USING_BLUETOOTH
#include "bf0_sibles.h"
#include "bf0_ble_ancs.h"
#endif
#include "data_service_subscriber.h"
#include "app_message.h"
#include "app_incoming_call.h"
#include "bloc_notification.h"
#include "watch_global_data.h"
#if !defined(_MSC_VER)
#include "ancs_service.h"
#endif
#include "intent.h"
#define DBG_TAG "app.message.database"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
// static uint8_t g_ble_message[50];
static datac_handle_t message_service_handle;

/* Send an ANCS perform-notification-action (accept/decline an iOS call)
 * using the same data-service handle this module registered as subscriber.
 * positive != 0 → POSITIVE action, 0 → NEGATIVE. */
int ancs_perform_call_action(uint32_t noti_uid, int positive)
{
#if !defined(_MSC_VER)
    if (DATA_CLIENT_INVALID_HANDLE == message_service_handle)
    {
        return -1;
    }
    ancs_service_config_t config;
    memset(&config, 0, sizeof(config));
    config.command = ANCS_SERVICE_PERFORM_NOTIFY_ACTION;
    config.data.action.uid = noti_uid;
    config.data.action.act_id = positive ? BLE_ACTION_ID_POSITIVE : BLE_ACTION_ID_NEGATIVE;
    return datac_config(message_service_handle,
                        sizeof(ancs_service_config_t), (uint8_t *)&config);
#else
    (void)noti_uid;
    (void)positive;
    return -1;
#endif
}

static void print_hex(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        rt_kprintf("%02X ", data[i]);
    }
    rt_kprintf("\n");
}

static int app_ble_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        notification_t notification;
        rt_memset(&notification, 0, sizeof(notification));
        notification.state = true;
        notification.type = Notify_others;
        bool is_incoming_call = false;
        bool is_missed_call = false;
#ifndef _MSC_VER
        RT_ASSERT(arg->data);
        int16_t len = arg->data_len;
        ancs_service_noti_attr_t *value = (ancs_service_noti_attr_t *)arg->data;

        /* iOS categories: 1 = incoming call (also used for in-progress call
         * updates via MODIFIED events), 2 = missed call. Incoming / active
         * calls only drive the dedicated call UI — they must NOT land in the
         * notification list. Missed calls keep the normal notification path. */
        is_incoming_call = (value->cate_id == BLE_ANCS_CATEGORY_ID_INCOMING_CALL);
        is_missed_call   = (value->cate_id == BLE_ANCS_CATEGORY_ID_MISSED_CALL);
        notification.calling = is_incoming_call;

        /* Use ANCS noti_uid as the notification id. The incoming-call screen
         * uses it to match dismiss events from the phone. */
        rt_snprintf(notification.id, sizeof(notification.id), "ancs_%u",
                    (unsigned)value->noti_uid);
        notification.sec_time = SkaiWatchSys.SecondCountRTC;

        /* An iOS missed-call event means the ringing ended; close any open
         * incoming-call UI before we enqueue the missed-call notification. */
        if (is_missed_call)
        {
            incoming_call_force_close();
        }

        ble_ancs_attr_value_t *att_value = &value->value[0];
        for (uint32_t i = 0; i < value->attr_count; i++)
        {
            if (att_value->attr_id == BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME)
            {
                uint8_t *app_name = rt_malloc(att_value->len + 1);
                rt_memcpy(app_name, att_value->data, att_value->len);
                app_name[att_value->len] = 0;
                app_message_set_app_name(app_name);
                LOG_D("[ANCS] len:%d, DISPLAY_NAME:%s\n", att_value->len, app_name);
                print_hex(app_name, att_value->len);
                uint8_t type = get_notification_type_from_ios_ancs_name((const char *)app_name);
                app_message_set_app_index(type);
                notification.type = type;
                rt_free(app_name);
            }
            else if (att_value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE)
            {
                uint8_t *title = rt_malloc(att_value->len + 1);
                rt_memcpy(title, att_value->data, att_value->len);
                title[att_value->len] = 0;
                app_message_set_title(title);
                strncpy(notification.title, (const char *)title,
                        sizeof(notification.title) - 1);
                rt_free(title);
            }
            else if (att_value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE)
            {
                uint8_t *message = rt_malloc(att_value->len + 1);
                rt_memcpy(message, att_value->data, att_value->len);
                message[att_value->len] = 0;
                app_message_set_content(message);
                strncpy(notification.message, (const char *)message,
                        sizeof(notification.message) - 1);
                rt_free(message);
            }
            att_value = (ble_ancs_attr_value_t *)((uint8_t *)att_value + sizeof(ble_ancs_attr_value_t) + att_value->len);
        }

        /* Prime the incoming-call screen with the ANCS UID *before* starting
         * it so the accept/hangup buttons can fall back to ANCS actions when
         * HFP is not bonded (typical iOS BLE-only case). */
        if (is_incoming_call)
        {
            incoming_call_set_ancs_uid(value->noti_uid);
        }
#endif

        if (is_incoming_call)
        {
            /* Bypass interact_with_notification entirely: incoming / ongoing
             * calls should never be persisted in the notification list. */
            trigger_incoming_call_ui(&notification);
        }
        else
        {
            interact_with_notification(&notification);
        }
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
    }
    return 0;
}

int app_message_database_init(void)
{
    message_service_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != message_service_handle);
    datac_subscribe(message_service_handle, "ANCS", app_ble_callback, 0);
    return 0;
}

INIT_APP_EXPORT(app_message_database_init);
