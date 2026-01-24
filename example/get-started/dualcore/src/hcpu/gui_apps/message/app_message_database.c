/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "string.h"
#include "bf0_sibles.h"
#include "bf0_ble_ancs.h"
#include "data_service_subscriber.h"
#include "app_message.h"
#include "bloc_notification.h"
#if !defined(_MSC_VER)
#include "ancs_service.h"
#endif
#include "intent.h"
#define DBG_TAG "app.message.database"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
// static uint8_t g_ble_message[50];
static datac_handle_t message_service_handle;

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
#ifndef _MSC_VER
        RT_ASSERT(arg->data);
        int16_t len = arg->data_len;
        ancs_service_noti_attr_t *value = (ancs_service_noti_attr_t *)arg->data;
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
                strcpy(notification.title, (const char *)title);
                rt_free(title);
            }
            else if (att_value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE)
            {
                uint8_t *message = rt_malloc(att_value->len + 1);
                rt_memcpy(message, att_value->data, att_value->len);
                message[att_value->len] = 0;
                app_message_set_content(message);
                strcpy(notification.message, (const char *)message);
                rt_free(message);
            }
            // else if (att_value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID)
            // {
            //     uint8_t *app_id = rt_malloc(att_value->len + 1);
            //     rt_memcpy(app_id, att_value->data, att_value->len);
            //     app_id[att_value->len] = 0;
            //     LOG_D("[ANCS] len:%d, APP_ID:%s", att_value->len, app_id);
            //     rt_free(app_id);
            // }
            att_value = (ble_ancs_attr_value_t *)((uint8_t *)att_value + sizeof(ble_ancs_attr_value_t) + att_value->len);
        }
#endif

        // {
        //     intent_t i = intent_init("message");
        //     intent_set_string(i, "newfrom", "Unknown user");
        //     intent_runapp(i);
        //     intent_deinit(i);
        // }
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

#if 0
void send_a_sms(int argc, char **argv)
{
    app_message_set_app_name((const uint8_t *)"test_app");    
    app_message_set_title((const uint8_t *)"Hello");
    app_message_set_content((const uint8_t *)"World");
    
    intent_t i = intent_init("message");
    intent_set_string(i, "newfrom", "1234567");
    intent_runapp(i);
    intent_deinit(i);    
}
MSH_CMD_EXPORT_ALIAS(send_a_sms, sms, sms [length] [title] [content]);
#endif

int app_message_database_init(void)
{
    message_service_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != message_service_handle);
    datac_subscribe(message_service_handle, "ANCS", app_ble_callback, 0);
    return 0;
}

INIT_APP_EXPORT(app_message_database_init);
