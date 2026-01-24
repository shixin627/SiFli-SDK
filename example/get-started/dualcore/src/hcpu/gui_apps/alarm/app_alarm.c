/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "alarm_manager_service.h"
#include "ui_datasrv_subscriber.h"
#include "ui_handler.h"
#include "ui_img_helper.h"
#ifdef APP_ID_ALARM
/**
 *  description of app alarm
 *
 */
typedef struct
{
    lv_obj_t *list_cnt;
    datac_handle_t srv_handle;
    bool first_resume; // call onresume first after app startup
    uint8_t alarm_num;
} app_alarm_t;

static app_alarm_t *p_app_alarm = NULL;

// LV_IMG_DECLARE(img_alarm);

static void list_btn_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_VALUE_CHANGED == event)
    {
        data_msg_t msg;
        alarm_msg_t *p_alarm_msg;

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
            p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_ENABLE_ALARM_REQ, sizeof(alarm_msg_t));
        else
            p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_DISABLE_ALARM_REQ, sizeof(alarm_msg_t));

        p_alarm_msg->idx = (int32_t)lv_obj_get_user_data(obj);
        datac_send_msg(p_app_alarm->srv_handle, &msg);
    }
    else if (LV_EVENT_CLICKED == event)
    {
        extern void app_alarm_edit(uint32_t alarm_idx);
        app_alarm_edit((uint32_t)lv_obj_get_user_data(obj));
    }
}

static void add_alarm_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == event)
    {
        if (p_app_alarm->alarm_num < BSP_ALARM_MAX)
        {
            extern void app_alarm_edit_time(int32_t alarm_idx);
            app_alarm_edit_time(-1);
        }
        else
        {
            // TODO: Show messagebox indicate that could not add more alarm.
            lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "Cannot add more alarms.", NULL, true);
            lv_obj_center(msgbox);
        }
    }
}
// 自定義按鍵樣式
static lv_style_t style_btn;

static void init_styles(void)
{
    // 初始化普通按鍵樣式
    lv_style_init(&style_btn);
    lv_style_set_radius(&style_btn, 50);
    lv_style_set_bg_color(&style_btn, lv_color_hex(0xFFFFFF));
    lv_style_set_bg_opa(&style_btn, LV_OPA_10);
    lv_style_set_shadow_width(&style_btn, 30);
    lv_style_set_shadow_color(&style_btn, lv_color_hex(0xFFFFFF));
    lv_style_set_shadow_opa(&style_btn, LV_OPA_0);
}
static int srv_msg_handler(data_callback_arg_t *arg)
{
    if (!p_app_alarm && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        /* Subscribe data error*/
        if (p_app_alarm)
        {
            if (rsp->result >= 0)
            {
                /* Request alarm list*/
                data_msg_t msg;

                data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ, 0);
                datac_send_msg(p_app_alarm->srv_handle, &msg);
            }
        }

        break;
    }

    case ALARMMGR_MSG_GET_ALARM_LIST_NEXT_RSP:
    {
        alarm_msg_t *data = (alarm_msg_t *)arg->data;

        if (data)
        {
            char buf[32];
            lv_obj_t *list_btn;

            /*Add list button*/
            rt_sprintf(buf, "%d:%d", data->ctx.hour, data->ctx.minute);
            list_btn = lv_list_add_btn(p_app_alarm->list_cnt, NULL, buf);

            // lv_btn_set_checkable(list_btn, true);
            lv_obj_add_style(list_btn, &style_btn, 0);
            if (ALARM_STATE_ENABLE == data->ctx.state)
            {
                // lv_btn_set_state(list_btn, LV_BTN_STATE_CHECKED_PRESSED);
                lv_obj_add_state(list_btn, LV_STATE_CHECKED);
            }

            uint32_t index = data->idx;
            rt_kprintf("alarm index: %d\r\n", index);
            lv_obj_add_event_cb(list_btn, list_btn_event_callback, LV_EVENT_ALL, (void *)&index);
            p_app_alarm->alarm_num++;

            /* Request next alarm*/
            data_msg_t msg;
            uint8_t *msg_payload;

            msg_payload = data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ, sizeof(alarm_msg_t));
            memcpy(msg_payload, arg->data, sizeof(alarm_msg_t));

            datac_send_msg(p_app_alarm->srv_handle, &msg);
        }
        else
        {
            /*'Add' buttons at end of the list*/
            lv_obj_t *list_btn;

            list_btn = lv_list_add_btn(p_app_alarm->list_cnt, NULL, "Add");
            lv_obj_add_event_cb(list_btn, add_alarm_callback, LV_EVENT_CLICKED, NULL);
            lv_obj_add_style(list_btn, &style_btn, 0);
        }
    }
    break;

    default:
        break;
    }
    return 0;
}

void app_alarm_init(void *param)
{
    lv_obj_t *scr = lv_scr_act();

    // header
    // lv_obj_t *cont_title = lv_lvsfheader_create(scr);
    // lv_lvsfheader_set_title(cont_title, LV_EXT_STR_GET_BY_KEY(alarm_title, "Alarm Title"));
    // lv_lvsfheader_set_visible_item(cont_title, LVSF_HEADER_ROOT);

    // alarm list
    lv_obj_t *list1 = lv_list_create(scr);
    lv_obj_set_size(list1, 400, LV_VER_RES_MAX - 100);
    // 透明背景
    lv_obj_set_style_bg_color(list1, lv_color_make(0, 0, 0), 0);
    lv_obj_align(list1, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_scrollbar_mode(list1, LV_SCROLLBAR_MODE_OFF);

    p_app_alarm->list_cnt = list1;
}

static void on_start(void)
{
    RT_ASSERT(NULL == p_app_alarm);
    p_app_alarm = (app_alarm_t *)lv_mem_alloc(sizeof(app_alarm_t));
    memset(p_app_alarm, 0, sizeof(app_alarm_t));
    p_app_alarm->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    // 初始化樣式
    init_styles();
    app_alarm_init(NULL);

    p_app_alarm->first_resume = true;
}
static void on_resume(void)
{
    p_app_alarm->alarm_num = 0;

    p_app_alarm->srv_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_alarm->srv_handle);
    /* Subscribe service data*/
    ui_datac_subscribe(p_app_alarm->srv_handle, "alarmmgr", srv_msg_handler, 0);
    if (p_app_alarm->first_resume)
    {
        p_app_alarm->first_resume = false;
    }
}

static void on_pause(void)
{
    datac_close(p_app_alarm->srv_handle);
    p_app_alarm->srv_handle = DATA_CLIENT_INVALID_HANDLE;

    // lv_list_clean(p_app_alarm->list_cnt);
    lv_obj_clean(p_app_alarm->list_cnt);
    p_app_alarm->alarm_num = 0;
}

static void on_stop(void)
{
    if (p_app_alarm)
    {
        if (p_app_alarm->srv_handle != DATA_CLIENT_INVALID_HANDLE)
        {
            datac_close(p_app_alarm->srv_handle);
            p_app_alarm->srv_handle = DATA_CLIENT_INVALID_HANDLE;
        }

        lv_mem_free(p_app_alarm);
        p_app_alarm = NULL;
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

static int app_main(intent_t i)
{
    /* Regist root page message handler */
    gui_app_regist_msg_handler(APP_ID_ALARM, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(alarm), IMG_ALARM, APP_ID_ALARM, app_main);
#endif