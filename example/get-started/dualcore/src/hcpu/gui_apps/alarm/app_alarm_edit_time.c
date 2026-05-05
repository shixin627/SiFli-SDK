/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "alarm_manager_service.h"
#include "alarm_client.h"
#include "ui_datasrv_subscriber.h"
#include "ui_helper.h"
#include "app_alarm_style.h"

#define DBG_TAG "APP.ALARM.ET"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

#define SUBPAGE_NAME "almadd"

#define HOURS_ROLLER_STR \
    "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n" \
    "13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n"
#define MINUTE_ROLLER_STR \
    "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n" \
    "13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n" \
    "26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n" \
    "39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n" \
    "52\n53\n54\n55\n56\n57\n58\n59\n"

/**
 * Apple-style time picker.
 *
 *   ┌─────────────────────┐
 *   │   Set Time          │   ← title (small)
 *   │  ╭──╮      ╭──╮     │
 *   │  │07│  :   │30│     │   ← dark rollers, accent-coloured selected row
 *   │  ╰──╯      ╰──╯     │
 *   │   [Cancel]    [Set] │   ← bottom buttons (neutral / accent-filled)
 *   └─────────────────────┘
 */

typedef struct
{
    lv_obj_t *roller_hour;
    lv_obj_t *roller_minute;

    int32_t alarm_idx; /* >=0: edit existing, -1: add new */
    alarm_contxt_t alarm_ctx;
    datac_handle_t srv_handle;
} app_alarm_edit_time_t;

static app_alarm_edit_time_t *p_app_alarm_edit_time = NULL;

/*********************
 *   EVENTS
 *********************/

static void cancel_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    gui_app_goback();
}

static void set_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    data_msg_t msg;
    alarm_msg_t *p;
    if (p_app_alarm_edit_time->alarm_idx >= 0)
    {
        p = (alarm_msg_t *)data_service_init_msg(
            &msg, ALARMMGR_MSG_EDIT_ALARM_REQ, sizeof(alarm_msg_t));
        p->idx = p_app_alarm_edit_time->alarm_idx;
    }
    else
    {
        p = (alarm_msg_t *)data_service_init_msg(
            &msg, ALARMMGR_MSG_ADD_ALARM_REQ, sizeof(alarm_msg_t));
    }
    memcpy(&p->ctx, &p_app_alarm_edit_time->alarm_ctx, sizeof(alarm_contxt_t));
    datac_send_msg(p_app_alarm_edit_time->srv_handle, &msg);
}

static void hour_roller_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    p_app_alarm_edit_time->alarm_ctx.hour =
        lv_roller_get_selected(lv_event_get_target(e));
}

static void minute_roller_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    p_app_alarm_edit_time->alarm_ctx.minute =
        lv_roller_get_selected(lv_event_get_target(e));
}

/*********************
 *   DATA SYNC
 *********************/

static int srv_msg_handler(data_callback_arg_t *arg)
{
    if (!p_app_alarm_edit_time && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
        return 0;

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (p_app_alarm_edit_time && rsp->result >= 0
            && p_app_alarm_edit_time->alarm_idx >= 0)
        {
            data_msg_t msg;
            alarm_msg_t *p = (alarm_msg_t *)data_service_init_msg(
                &msg, ALARMMGR_MSG_GET_ALARM_REQ, sizeof(alarm_msg_t));
            p->idx = p_app_alarm_edit_time->alarm_idx;
            datac_send_msg(p_app_alarm_edit_time->srv_handle, &msg);
        }
        break;
    }

    case ALARMMGR_MSG_GET_ALARM_RSP:
    {
        alarm_msg_t *data = (alarm_msg_t *)arg->data;
        if (data)
        {
            RT_ASSERT(p_app_alarm_edit_time->alarm_idx == data->idx);
            memcpy(&p_app_alarm_edit_time->alarm_ctx, &data->ctx,
                   sizeof(alarm_contxt_t));
            lv_roller_set_selected(p_app_alarm_edit_time->roller_hour,
                                   data->ctx.hour, LV_ANIM_OFF);
            lv_roller_set_selected(p_app_alarm_edit_time->roller_minute,
                                   data->ctx.minute, LV_ANIM_OFF);
        }
        break;
    }

    case ALARMMGR_MSG_EDIT_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;
        LOG_D("alarm edit err: %d", rsp->result);
        if (rsp->result == 0)
            watch_alarm_local_sync(1 /* EDIT */,
                                   p_app_alarm_edit_time->alarm_idx,
                                   &p_app_alarm_edit_time->alarm_ctx);
        gui_app_goback();
        break;
    }
    case ALARMMGR_MSG_ADD_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;
        LOG_D("alarm add err: %d", rsp->result);
        if (rsp->result == 0)
            watch_alarm_local_sync(0 /* ADD */, -1,
                                   &p_app_alarm_edit_time->alarm_ctx);
        gui_app_goback();
        break;
    }

    default: break;
    }
    return 0;
}

/*********************
 *   UI BUILDERS
 *********************/

static void style_dark_roller(lv_obj_t *roller)
{
    /* Main roller area — non-selected rows. The default text color was too
       dim (#6E6E73) against the dark card background; bump to the secondary
       grey so digits stay legible. */
    lv_obj_set_style_bg_color(roller, ALARM_COLOR_CARD, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(roller, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(roller, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(roller, ALARM_CARD_RADIUS, LV_PART_MAIN);
    lv_obj_set_style_text_color(roller, ALARM_COLOR_TEXT_SECONDARY,
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(
        roller, LV_EXT_FONT_GET(get_system_font_size(1)), LV_PART_MAIN);
    lv_obj_set_style_text_align(roller, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(roller, 12, LV_PART_MAIN);

    /* Selected row — bright white, larger font. */
    lv_obj_set_style_bg_opa(roller, LV_OPA_TRANSP, LV_PART_SELECTED);
    lv_obj_set_style_text_color(roller, ALARM_COLOR_TEXT_PRIMARY,
                                LV_PART_SELECTED);
    lv_obj_set_style_text_font(
        roller, LV_EXT_FONT_GET(get_system_font_size(2)), LV_PART_SELECTED);
}

static lv_obj_t *create_action_btn(lv_obj_t *parent, const char *text,
                                   bool primary, lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 120, 56);
    lv_obj_set_style_bg_color(btn,
                              primary ? ALARM_COLOR_ACCENT : ALARM_COLOR_NEUTRAL,
                              0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn, 28, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_font(
        lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_center(lbl);
    return btn;
}

static void build_layout(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, ALARM_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* Title — sits at the top in safe area. */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title,
                      p_app_alarm_edit_time->alarm_idx >= 0 ? "Edit Time"
                                                            : "New Alarm");
    lv_obj_set_style_text_font(
        title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(title, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 28);

    /* Hour roller (left). */
    lv_obj_t *roller_h = lv_roller_create(scr);
    lv_roller_set_options(roller_h, HOURS_ROLLER_STR, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(roller_h, 3);
    lv_obj_set_width(roller_h, 110);
    style_dark_roller(roller_h);
    lv_obj_align(roller_h, LV_ALIGN_CENTER, -70, -10);
    lv_obj_add_event_cb(roller_h, hour_roller_event_cb, LV_EVENT_VALUE_CHANGED,
                        NULL);
    p_app_alarm_edit_time->roller_hour = roller_h;

    /* Colon between rollers. */
    lv_obj_t *colon = lv_label_create(scr);
    lv_label_set_text(colon, ":");
    lv_obj_set_style_text_color(colon, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_font(
        colon, LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_align(colon, LV_ALIGN_CENTER, 0, -10);

    /* Minute roller (right). */
    lv_obj_t *roller_m = lv_roller_create(scr);
    lv_roller_set_options(roller_m, MINUTE_ROLLER_STR, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(roller_m, 3);
    lv_obj_set_width(roller_m, 110);
    style_dark_roller(roller_m);
    lv_obj_align(roller_m, LV_ALIGN_CENTER, 70, -10);
    lv_obj_add_event_cb(roller_m, minute_roller_event_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
    p_app_alarm_edit_time->roller_minute = roller_m;

    /* Bottom buttons. */
    lv_obj_t *cancel = create_action_btn(scr, "Cancel", false, cancel_event_cb);
    lv_obj_align(cancel, LV_ALIGN_BOTTOM_MID, -70, -28);
    lv_obj_t *set = create_action_btn(scr, "Set", true, set_event_cb);
    lv_obj_align(set, LV_ALIGN_BOTTOM_MID, 70, -28);

    /* Pre-fill rollers with the current ctx so the user sees something
       sensible while waiting for the GET response (no-op for new alarm). */
    lv_roller_set_selected(roller_h, p_app_alarm_edit_time->alarm_ctx.hour,
                           LV_ANIM_OFF);
    lv_roller_set_selected(roller_m, p_app_alarm_edit_time->alarm_ctx.minute,
                           LV_ANIM_OFF);
}

/*********************
 *   LIFECYCLE
 *********************/

static void on_start(void)
{
    build_layout();
}

static void on_resume(void)
{
    if (DATA_CLIENT_INVALID_HANDLE == p_app_alarm_edit_time->srv_handle)
    {
        p_app_alarm_edit_time->srv_handle = datac_open();
        RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_alarm_edit_time->srv_handle);
    }
    ui_datac_subscribe(p_app_alarm_edit_time->srv_handle, "alarmmgr",
                       srv_msg_handler, 0);
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
    case GUI_APP_MSG_ONSTART:  on_start();  break;
    case GUI_APP_MSG_ONRESUME: on_resume(); break;
    case GUI_APP_MSG_ONPAUSE:  on_pause();  break;
    case GUI_APP_MSG_ONSTOP:   on_stop();   break;
    default: break;
    }
}

void app_alarm_edit_time(int32_t alarm_idx)
{
    rt_kprintf("app_alarm_edit_time[%d] entry\r\n", alarm_idx);

    RT_ASSERT(NULL == p_app_alarm_edit_time);
    p_app_alarm_edit_time = (app_alarm_edit_time_t *)lv_mem_alloc(
        sizeof(app_alarm_edit_time_t));
    memset(p_app_alarm_edit_time, 0, sizeof(app_alarm_edit_time_t));
    p_app_alarm_edit_time->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    p_app_alarm_edit_time->alarm_idx = alarm_idx;

    if (alarm_idx < 0) /* defaults for a fresh alarm */
    {
        p_app_alarm_edit_time->alarm_ctx.state = ALARM_STATE_ENABLE;
        p_app_alarm_edit_time->alarm_ctx.snooze = ALARM_SNOOZE_ENABLE;
        p_app_alarm_edit_time->alarm_ctx.days = ALARM_REPEAT_ONE_SHOT;
        p_app_alarm_edit_time->alarm_ctx.hour = 8;
        p_app_alarm_edit_time->alarm_ctx.minute = 0;
    }

    gui_app_create_page(SUBPAGE_NAME, msg_handler);
}
