/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "alarm_manager_service.h"
#include "ui_datasrv_subscriber.h"

#define DBG_TAG "APP.ALARM.ET"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

#define SUBPAGE_NAME "almadd"

#define HOURS_ROLLER_STR "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n"
#define MINUTE_ROLLER_STR "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59\n"

/**
 *  description of app alarm
 *
 */
typedef struct
{
    lv_obj_t *roller_hour;
    lv_obj_t *roller_minute;
    lv_obj_t *btn_cancel;
    lv_obj_t *btn_setup;

    int32_t alarm_idx; // >0: edit existing alarm time, -1: add new alarm
    alarm_contxt_t alarm_ctx;
    datac_handle_t srv_handle;
} app_alarm_edit_time_t;

static app_alarm_edit_time_t *p_app_alarm_edit_time = NULL;

static void updata_alarm_data(void)
{
    /* Request alarm list*/
    data_msg_t msg;
    alarm_msg_t *p_alarm_msg;

    p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_EDIT_ALARM_REQ, sizeof(alarm_msg_t));
    p_alarm_msg->idx = p_app_alarm_edit_time->alarm_idx;
    memcpy(&p_alarm_msg->ctx, &p_app_alarm_edit_time->alarm_ctx, sizeof(alarm_contxt_t));
    datac_send_msg(p_app_alarm_edit_time->srv_handle, &msg);
}

static void cancel_event_handler(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_CLICKED)
    {
        gui_app_goback();
    }
}

static void set_event_handler(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_CLICKED)
    {
        /* Request alarm list*/
        data_msg_t msg;
        alarm_msg_t *p_alarm_msg;

        if (p_app_alarm_edit_time->alarm_idx >= 0)
        {
            p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_EDIT_ALARM_REQ, sizeof(alarm_msg_t));
            p_alarm_msg->idx = p_app_alarm_edit_time->alarm_idx;
        }
        else
        {
            p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_ADD_ALARM_REQ, sizeof(alarm_msg_t));
        }

        memcpy(&p_alarm_msg->ctx, &p_app_alarm_edit_time->alarm_ctx, sizeof(alarm_contxt_t));
        datac_send_msg(p_app_alarm_edit_time->srv_handle, &msg);
    }
}

static int srv_msg_handler(data_callback_arg_t *arg)
{
    if (!p_app_alarm_edit_time && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
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
        if (p_app_alarm_edit_time)
        {
            if (rsp->result >= 0)
            {
                /* Request alarm content from service*/
                if (p_app_alarm_edit_time->alarm_idx >= 0)
                {
                    data_msg_t msg;
                    alarm_msg_t *p_alarm_msg;

                    p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_REQ, sizeof(alarm_msg_t));
                    p_alarm_msg->idx = p_app_alarm_edit_time->alarm_idx;
                    datac_send_msg(p_app_alarm_edit_time->srv_handle, &msg);
                }
            }
        }

        break;
    }
    case ALARMMGR_MSG_GET_ALARM_RSP:
    {
        alarm_msg_t *data = (alarm_msg_t *)arg->data;

        if (data)
        {
            RT_ASSERT(p_app_alarm_edit_time->alarm_idx == data->idx);
            memcpy(&p_app_alarm_edit_time->alarm_ctx, &data->ctx, sizeof(alarm_contxt_t));
            lv_roller_set_selected(p_app_alarm_edit_time->roller_hour, data->ctx.hour, true);
            lv_roller_set_selected(p_app_alarm_edit_time->roller_minute, data->ctx.minute, true);
        }
    }
    break;
    case ALARMMGR_MSG_EDIT_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;

        LOG_D("alarm edit err: %d", rsp->result);

        gui_app_goback();
    }
    break;
    case ALARMMGR_MSG_ADD_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;

        LOG_D("alarm add err: %d", rsp->result);
        gui_app_goback();
    }
    break;
    default:
        break;
    }
    return 0;
}

static void hour_roller_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_VALUE_CHANGED == event)
    {
        p_app_alarm_edit_time->alarm_ctx.hour = lv_roller_get_selected(obj);
    }
}

static void minute_roller_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (LV_EVENT_VALUE_CHANGED == event)
    {
        p_app_alarm_edit_time->alarm_ctx.minute = lv_roller_get_selected(obj);
    }
}

#define BORDER_GAP_HOR 100
#define BORDER_GAP_VER 50

void app_alarm_edit_time_init(void *param)
{
    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *roller_h, *roller_m;

    roller_h = lv_roller_create(scr);
    lv_roller_set_options(roller_h, HOURS_ROLLER_STR, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(roller_h, 3);
    lv_obj_set_width(roller_h, 100);
    lv_obj_add_event_cb(roller_h, hour_roller_event_callback, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_align(roller_h, LV_ALIGN_TOP_LEFT, BORDER_GAP_HOR, BORDER_GAP_VER);
    p_app_alarm_edit_time->roller_hour = roller_h;

    roller_m = lv_roller_create(scr);
    lv_roller_set_options(roller_m, MINUTE_ROLLER_STR, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(roller_m, 3);
    lv_obj_set_width(roller_m, 100);
    lv_obj_add_event_cb(roller_m, minute_roller_event_callback, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_align(roller_m, LV_ALIGN_TOP_RIGHT, -BORDER_GAP_HOR, BORDER_GAP_VER);
    p_app_alarm_edit_time->roller_minute = roller_m;

    lv_obj_t *btn, *label;
    btn = lv_btn_create(scr);
    label = lv_label_create(btn);
    lv_label_set_text(label, LV_EXT_STR_GET_BY_KEY(cancel, "Canel"));
    // lv_obj_set_click(label, false);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align_to(btn, roller_h, LV_ALIGN_BOTTOM_MID, 0, BORDER_GAP_VER);
    lv_obj_add_event_cb(btn, cancel_event_handler, LV_EVENT_CLICKED, NULL);

    btn = lv_btn_create(scr);
    label = lv_label_create(btn);
    lv_label_set_text(label, LV_EXT_STR_GET_BY_KEY(set, "Set"));
    // lv_obj_set_click(label, false);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align_to(btn, roller_m, LV_ALIGN_BOTTOM_MID, 0, BORDER_GAP_VER);
    lv_obj_add_event_cb(btn, set_event_handler, LV_EVENT_CLICKED, NULL);
}

static void on_start(void)
{
    app_alarm_edit_time_init(NULL);
}

static void on_resume(void)
{
    /* Subscribe service data*/
    if (DATA_CLIENT_INVALID_HANDLE == p_app_alarm_edit_time->srv_handle)
    {
        p_app_alarm_edit_time->srv_handle = datac_open();
        RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_alarm_edit_time->srv_handle);
    }
    ui_datac_subscribe(p_app_alarm_edit_time->srv_handle, "alarmmgr", srv_msg_handler, 0);
}

static void on_pause(void)
{
    if (p_app_alarm_edit_time->srv_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_close(p_app_alarm_edit_time->srv_handle);
        p_app_alarm_edit_time->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    }
}

static void on_stop(void)
{
    if (p_app_alarm_edit_time)
    {
        lv_mem_free(p_app_alarm_edit_time);
        p_app_alarm_edit_time = NULL;
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

void app_alarm_edit_time(int32_t alarm_idx)
{
    rt_kprintf("app_alarm_edit_time[%d] entry\r\n", alarm_idx);

    RT_ASSERT(NULL == p_app_alarm_edit_time);
    p_app_alarm_edit_time = (app_alarm_edit_time_t *)lv_mem_alloc(sizeof(app_alarm_edit_time_t));
    memset(p_app_alarm_edit_time, 0, sizeof(app_alarm_edit_time_t));
    p_app_alarm_edit_time->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    p_app_alarm_edit_time->alarm_idx = alarm_idx;

    if (alarm_idx < 0) // add new alarm
    {
        p_app_alarm_edit_time->alarm_ctx.state = ALARM_STATE_ENABLE;
        p_app_alarm_edit_time->alarm_ctx.snooze = ALARM_SNOOZE_ENABLE;
        p_app_alarm_edit_time->alarm_ctx.days = ALARM_REPEAT_ONE_SHOT;
    }
    else
    {
        // get from data service
    }

    gui_app_create_page(SUBPAGE_NAME, msg_handler);
}
