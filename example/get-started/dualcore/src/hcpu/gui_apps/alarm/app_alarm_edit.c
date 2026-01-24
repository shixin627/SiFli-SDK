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

#define SUBPAGE_NAME "almedit"

#define DBG_TAG "APP.ALARM.E"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

/**
 *  description of app alarm
 *
 */
typedef struct
{
    lv_obj_t *list_cnt;
    int32_t alarm_idx;
    alarm_contxt_t alarm_ctx;
    datac_handle_t srv_handle;
} app_alarm_edit_t;

static app_alarm_edit_t *p_app_alarm_edit = NULL;

static void updata_alarm_data(void)
{
    /* Request alarm list*/
    data_msg_t msg;
    alarm_msg_t *p_alarm_msg;

    p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_EDIT_ALARM_REQ, sizeof(alarm_msg_t));
    p_alarm_msg->idx = p_app_alarm_edit->alarm_idx;
    memcpy(&p_alarm_msg->ctx, &p_app_alarm_edit->alarm_ctx, sizeof(alarm_contxt_t));
    datac_send_msg(p_app_alarm_edit->srv_handle, &msg);
}

static void delete_alarm_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == event)
    {
        /* Request alarm list*/
        data_msg_t msg;
        alarm_msg_t *p_alarm_msg;

        p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_DELETE_ALARM_REQ, sizeof(alarm_msg_t));
        p_alarm_msg->idx = p_app_alarm_edit->alarm_idx;
        datac_send_msg(p_app_alarm_edit->srv_handle, &msg);
    }
}

static void edit_snooze_btn_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == event)
    {
        if (ALARM_SNOOZE_ENABLE == p_app_alarm_edit->alarm_ctx.snooze)
        {
            p_app_alarm_edit->alarm_ctx.snooze = ALARM_SNOOZE_DISABLE;
            // lv_btn_set_state(obj, LV_BTN_STATE_RELEASED);
            lv_obj_add_state(obj, LV_STATE_CHECKED);
        }
        else
        {
            p_app_alarm_edit->alarm_ctx.snooze = ALARM_SNOOZE_ENABLE;
            // lv_btn_set_state(obj, LV_BTN_STATE_CHECKED_RELEASED);
            lv_obj_clear_state(obj, LV_STATE_CHECKED);
        }

        updata_alarm_data();
    }
}

static void edit_time_btn_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == event)
    {
        extern void app_alarm_edit_time(int32_t alarm_idx);
        app_alarm_edit_time(p_app_alarm_edit->alarm_idx);
    }
}

static void edit_repeat_btn_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_CLICKED == event)
    {
        extern void app_alarm_edit_repeat(uint32_t alarm_idx);
        app_alarm_edit_repeat(p_app_alarm_edit->alarm_idx);
    }
}

static int srv_msg_handler(data_callback_arg_t *arg)
{
    if (!p_app_alarm_edit && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
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
        if (p_app_alarm_edit)
        {
            if (rsp->result >= 0)
            {
                /* Request alarm content*/
                data_msg_t msg;
                alarm_msg_t *p_alarm_msg;

                p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ, sizeof(alarm_msg_t));
                p_alarm_msg->idx = p_app_alarm_edit->alarm_idx;
                datac_send_msg(p_app_alarm_edit->srv_handle, &msg);
            }
        }

        break;
    }

    case ALARMMGR_MSG_GET_ALARM_RSP:
    {
        alarm_msg_t *data = (alarm_msg_t *)arg->data;

        if (data)
        {
            char buf[32];
            lv_obj_t *list_btn;

            RT_ASSERT(p_app_alarm_edit->alarm_idx == data->idx);

            memcpy(&p_app_alarm_edit->alarm_ctx, &data->ctx, sizeof(alarm_contxt_t));

            /*Add list button*/
            rt_sprintf(buf, "%d:%d", data->ctx.hour, data->ctx.minute);
            list_btn = lv_list_add_btn(p_app_alarm_edit->list_cnt, NULL, buf);
            lv_obj_add_event_cb(list_btn, edit_time_btn_event_callback, LV_EVENT_CLICKED, NULL);

            if (ALARM_REPEAT_ONE_SHOT == data->ctx.days)
            {
                rt_sprintf(buf, LV_EXT_STR_GET_BY_KEY(Once, "Once"));
            }
            else if (ALARM_REPEAT_EVERYDAY == data->ctx.days)
            {
                rt_sprintf(buf, LV_EXT_STR_GET_BY_KEY(Everyday, "Everyday"));
            }
            else
            {
                uint8_t i;

                memset(buf, 0, sizeof(buf));

                for (i = 0; i < 7; i++)
                {
                    switch (data->ctx.days & (1 << i))
                    {
                    case ALARM_REPEAT_MONDAY:
                        strcat(buf, LV_EXT_STR_GET_BY_KEY(Mon, "Mon"));
                        break;
                    case ALARM_REPEAT_TUESDAY:
                        strcat(buf, LV_EXT_STR_GET_BY_KEY(Tue, "Tue"));
                        break;
                    case ALARM_REPEAT_WEDNESDAY:
                        strcat(buf, LV_EXT_STR_GET_BY_KEY(Wed, "Wed"));
                        break;
                    case ALARM_REPEAT_THURSDAY:
                        strcat(buf, LV_EXT_STR_GET_BY_KEY(Thu, "Thu"));
                        break;
                    case ALARM_REPEAT_FRIDAY:
                        strcat(buf, LV_EXT_STR_GET_BY_KEY(Fri, "Fri"));
                        break;
                    case ALARM_REPEAT_SATURDAY:
                        strcat(buf, LV_EXT_STR_GET_BY_KEY(Sat, "Sat"));
                        break;
                    case ALARM_REPEAT_SUNDAY:
                        strcat(buf, LV_EXT_STR_GET_BY_KEY(Sun, "Sun"));
                        break;
                    default:
                        continue;
                    }

                    strcat(buf, ",");
                }
            }

            list_btn = lv_list_add_btn(p_app_alarm_edit->list_cnt, NULL, buf);
            lv_obj_add_event_cb(list_btn, edit_repeat_btn_event_callback, LV_EVENT_CLICKED, NULL);

            list_btn = lv_list_add_btn(p_app_alarm_edit->list_cnt, NULL, "Snooze");
            // lv_btn_set_checkable(list_btn, true);
            lv_obj_add_state(list_btn, LV_STATE_CHECKED);
            
            if (ALARM_SNOOZE_ENABLE == data->ctx.snooze)
            {
                // lv_btn_set_state(list_btn, LV_BTN_STATE_CHECKED_RELEASED);
                lv_obj_clear_state(list_btn, LV_STATE_CHECKED);
            }
            lv_obj_add_event_cb(list_btn, edit_snooze_btn_event_callback, LV_EVENT_CLICKED, NULL);

            /*'Delete' buttons at end of the list*/
            list_btn = lv_list_add_btn(p_app_alarm_edit->list_cnt, NULL, "Delete alarm");
            lv_obj_add_event_cb(list_btn, delete_alarm_callback, LV_EVENT_CLICKED, NULL);
        }
    }
    break;

    case ALARMMGR_MSG_DELETE_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;

        if (rsp->result != 0)
        {
            LOG_I("alarm delete err: %d", rsp->result);
        }
        else
        {
            gui_app_goback();
        }
    }
    break;

    case ALARMMGR_MSG_EDIT_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;

        LOG_D("alarm edit err: %d", rsp->result);
    }
    break;

    default:
        break;
    }
    return 0;
}

static void back_event_handler(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_CLICKED)
    {
        gui_app_goback();
    }
}

void app_alarm_edit_init(void *param)
{
    lv_obj_t *scr = lv_scr_act();

    // header
    lv_obj_t *cont_title = lv_lvsfheader_create(scr);
    lv_lvsfheader_set_title(cont_title, LV_EXT_STR_GET_BY_KEY(alarm_edit, "Alarm Edit"));
    lv_lvsfheader_set_visible_item(cont_title, LVSF_HEADER_BRANCH);
    lv_lvsfheader_back_event_cb(cont_title, back_event_handler);

    // alarm list
    lv_obj_t *list1 = lv_list_create(scr);
    lv_obj_set_size(list1, LV_HOR_RES, LV_VER_RES - lv_obj_get_height(cont_title));
    lv_obj_align_to(list1, cont_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
    // lv_list_set_scrollbar_mode(list1, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollbar_mode(list1, LV_SCROLLBAR_MODE_OFF);

    p_app_alarm_edit->list_cnt = list1;
}

static void on_start(void)
{
    app_alarm_edit_init(NULL);
}

static void on_resume(void)
{
    /* Request alarm content*/
    data_msg_t msg;
    alarm_msg_t *p_alarm_msg;

    /* Subscribe service data*/
    if (p_app_alarm_edit->srv_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        p_app_alarm_edit->srv_handle = datac_open();
        RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_alarm_edit->srv_handle);
    }
    ui_datac_subscribe(p_app_alarm_edit->srv_handle, "alarmmgr", srv_msg_handler, 0);
    p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_REQ, sizeof(alarm_msg_t));
    p_alarm_msg->idx = p_app_alarm_edit->alarm_idx;
    datac_send_msg(p_app_alarm_edit->srv_handle, &msg);
    rt_kprintf("app_alarm_edit[%d] on_resume\r\n", p_app_alarm_edit->alarm_idx);
}

static void on_pause(void)
{
    if (p_app_alarm_edit->srv_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_close(p_app_alarm_edit->srv_handle);
        p_app_alarm_edit->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    }
    // lv_list_clean(p_app_alarm_edit->list_cnt);
    lv_obj_clean(p_app_alarm_edit->list_cnt);
}

static void on_stop(void)
{
    if (p_app_alarm_edit)
    {
        lv_mem_free(p_app_alarm_edit);
        p_app_alarm_edit = NULL;
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

void app_alarm_edit(uint32_t alarm_idx)
{
    rt_kprintf("app_alarm_edit[%d] entry\r\n", alarm_idx);

    RT_ASSERT(alarm_idx >= 0);

    RT_ASSERT(NULL == p_app_alarm_edit);
    p_app_alarm_edit = (app_alarm_edit_t *)lv_mem_alloc(sizeof(app_alarm_edit_t));
    memset(p_app_alarm_edit, 0, sizeof(app_alarm_edit_t));
    p_app_alarm_edit->srv_handle = DATA_CLIENT_INVALID_HANDLE;

    p_app_alarm_edit->alarm_idx = alarm_idx;

    gui_app_create_page(SUBPAGE_NAME, msg_handler);
}
