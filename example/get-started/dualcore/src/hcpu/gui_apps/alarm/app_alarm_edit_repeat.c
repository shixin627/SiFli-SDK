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

#define DBG_TAG "APP.ALARM.ER"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

#define SUBPAGE_NAME "alm_rpt"

/**
 *  description of app alarm
 *
 */
typedef struct
{
    int32_t alarm_idx; // >0: edit existing alarm time, -1: add new alarm
    alarm_contxt_t alarm_ctx;
    datac_handle_t srv_handle;
    lv_obj_t *btnm;
} app_alarm_edit_repeat_t;

static app_alarm_edit_repeat_t *p_app_alarm_edit_repeat = NULL;

static const lv_btnmatrix_ctrl_t btnm_ctrl_map[] =
    {
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
        0 | LV_BTNMATRIX_CTRL_CHECKABLE | LV_BTNMATRIX_CTRL_CLICK_TRIG,
};

static const char *btnm_map[] =
    {
        "Everyday", "\n",
        "Workday", "\n",
        "Weekend", "\n",
        "Monsday", "\n",
        "Tuesday", "\n",
        "Wednesday", "\n",
        "Thursday", "\n",
        "Friday", "\n",
        "Saturday", "\n",
        "Sunday", ""};

static void back_event_handler(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_CLICKED)
    {
        gui_app_goback();
    }
}

static void update_btm_state_by_value(lv_obj_t *btnm, alarm_repeat_t repeat)
{
    uint8_t i;

    if (ALARM_REPEAT_EVERYDAY == repeat)
        lv_btnmatrix_set_btn_ctrl(btnm, 0, LV_BTNMATRIX_CTRL_CHECKED);
    else
        lv_btnmatrix_clear_btn_ctrl(btnm, 0, LV_BTNMATRIX_CTRL_CHECKED);

    if (ALARM_REPEAT_WORKDAY == repeat)
        lv_btnmatrix_set_btn_ctrl(btnm, 1, LV_BTNMATRIX_CTRL_CHECKED);
    else
        lv_btnmatrix_clear_btn_ctrl(btnm, 1, LV_BTNMATRIX_CTRL_CHECKED);

    if (ALARM_REPEAT_WEEKEND == repeat)
        lv_btnmatrix_set_btn_ctrl(btnm, 2, LV_BTNMATRIX_CTRL_CHECKED);
    else
        lv_btnmatrix_clear_btn_ctrl(btnm, 2, LV_BTNMATRIX_CTRL_CHECKED);

    for (i = 3; i < 10; i++)
    {
        if (repeat & (1 << (i - 3)))
            lv_btnmatrix_set_btn_ctrl(btnm, i, LV_BTNMATRIX_CTRL_CHECKED);
        else
            lv_btnmatrix_clear_btn_ctrl(btnm, i, LV_BTNMATRIX_CTRL_CHECKED);
    }
}

static void btnm_event_handler(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (event == LV_EVENT_VALUE_CHANGED)
    {
        // uint16_t idx = lv_btnmatrix_get_active_btn(obj);
        uint16_t idx = lv_btnmatrix_get_selected_btn(obj);
        alarm_repeat_t repeat = p_app_alarm_edit_repeat->alarm_ctx.days;

        switch (idx)
        {
        case 0:
        {
            if (ALARM_REPEAT_EVERYDAY == repeat)
                repeat = ALARM_REPEAT_ONE_SHOT;
            else
                repeat = ALARM_REPEAT_EVERYDAY;
        }
        break;

        case 1:
        {
            if (ALARM_REPEAT_WORKDAY == repeat)
                repeat = ALARM_REPEAT_ONE_SHOT;
            else
                repeat = ALARM_REPEAT_WORKDAY;
        }
        break;

        case 2:
        {
            if (ALARM_REPEAT_WEEKEND == repeat)
                repeat = ALARM_REPEAT_ONE_SHOT;
            else
                repeat = ALARM_REPEAT_WEEKEND;
        }
        break;

        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        {
            if (repeat & (1 << (idx - 3)))
                repeat &= ~(1 << (idx - 3));
            else
                repeat |= 1 << (idx - 3);
        }
        break;

        default:
            break;
        }

        p_app_alarm_edit_repeat->alarm_ctx.days = repeat;
        update_btm_state_by_value(obj, repeat);
    }
}

static int srv_msg_handler(data_callback_arg_t *arg)
{
    if (!p_app_alarm_edit_repeat && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
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
        if (p_app_alarm_edit_repeat)
        {
            if (rsp->result >= 0)
            {
                /* Request alarm content from service*/
                if (p_app_alarm_edit_repeat->alarm_idx >= 0)
                {
                    data_msg_t msg;
                    alarm_msg_t *p_alarm_msg;

                    p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_REQ, sizeof(alarm_msg_t));
                    p_alarm_msg->idx = p_app_alarm_edit_repeat->alarm_idx;
                    datac_send_msg(p_app_alarm_edit_repeat->srv_handle, &msg);
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
            RT_ASSERT(p_app_alarm_edit_repeat->alarm_idx == data->idx);
            memcpy(&p_app_alarm_edit_repeat->alarm_ctx, &data->ctx, sizeof(alarm_contxt_t));
            update_btm_state_by_value(p_app_alarm_edit_repeat->btnm, data->ctx.days);
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

    default:
        break;
    }
    return 0;
}

void app_alarm_edit_repeat_init(void *param)
{
    lv_obj_t *scr = lv_scr_act();

    // header
    lv_obj_t *cont_title = lv_lvsfheader_create(scr);
    lv_lvsfheader_set_title(cont_title, LV_EXT_STR_GET_BY_KEY(alarm_repeat, "Repeat"));
    lv_lvsfheader_set_visible_item(cont_title, LVSF_HEADER_BRANCH);
    lv_lvsfheader_back_event_cb(cont_title, back_event_handler);

    lv_obj_t *btnm1 = lv_btnmatrix_create(scr);
    lv_btnmatrix_set_map(btnm1, btnm_map);
    lv_obj_set_width(btnm1, LV_HOR_RES_MAX);
    lv_obj_set_height(btnm1, LV_VER_RES_MAX - lv_obj_get_height(cont_title) - 10);

    lv_obj_align_to(btnm1, cont_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_add_event_cb(btnm1, btnm_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
    lv_btnmatrix_set_ctrl_map(btnm1, btnm_ctrl_map);
    p_app_alarm_edit_repeat->btnm = btnm1;
}

static void on_start(void)
{
    app_alarm_edit_repeat_init(NULL);
}

static void on_resume(void)
{
    /* Subscribe service data*/
    if (DATA_CLIENT_INVALID_HANDLE == p_app_alarm_edit_repeat->srv_handle)
    {
        p_app_alarm_edit_repeat->srv_handle = datac_open();
        RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_alarm_edit_repeat->srv_handle);
    }

    ui_datac_subscribe(p_app_alarm_edit_repeat->srv_handle, "alarmmgr", srv_msg_handler, 0);
}

static void on_pause(void)
{
    data_msg_t msg;
    alarm_msg_t *p_alarm_msg;

    p_alarm_msg = (alarm_msg_t *)data_service_init_msg(&msg, ALARMMGR_MSG_EDIT_ALARM_REQ, sizeof(alarm_msg_t));
    p_alarm_msg->idx = p_app_alarm_edit_repeat->alarm_idx;
    memcpy(&p_alarm_msg->ctx, &p_app_alarm_edit_repeat->alarm_ctx, sizeof(alarm_contxt_t));
    datac_send_msg(p_app_alarm_edit_repeat->srv_handle, &msg);

    if (p_app_alarm_edit_repeat->srv_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_close(p_app_alarm_edit_repeat->srv_handle);
        p_app_alarm_edit_repeat->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    }
}

static void on_stop(void)
{
    if (p_app_alarm_edit_repeat)
    {
        lv_mem_free(p_app_alarm_edit_repeat);
        p_app_alarm_edit_repeat = NULL;
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

void app_alarm_edit_repeat(uint32_t alarm_idx)
{
    rt_kprintf("app_alarm_edit_repeat[%d] entry\r\n", alarm_idx);

    RT_ASSERT(alarm_idx >= 0);

    RT_ASSERT(NULL == p_app_alarm_edit_repeat);
    p_app_alarm_edit_repeat = (app_alarm_edit_repeat_t *)lv_mem_alloc(sizeof(app_alarm_edit_repeat_t));
    memset(p_app_alarm_edit_repeat, 0, sizeof(app_alarm_edit_repeat_t));
    p_app_alarm_edit_repeat->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    p_app_alarm_edit_repeat->alarm_idx = alarm_idx;

    gui_app_create_page(SUBPAGE_NAME, msg_handler);
}
