/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "alarm_manager_service.h"
#include "alarm_client.h"
#include "ui_datasrv_subscriber.h"
#include "ui_helper.h"
#include "app_alarm_style.h"

#define SUBPAGE_NAME "almedit"

#define DBG_TAG "APP.ALARM.E"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

/**
 * Apple-style edit-alarm page.
 *
 *   ┌─────────────────────────────┐
 *   │           07:30             │   ← big time, tappable → edit_time
 *   │ ┌─────────────────────────┐ │
 *   │ │ Repeat        Weekdays ›│ │   ← row → edit_repeat
 *   │ ├─────────────────────────┤ │
 *   │ │ Snooze              [◉] │ │   ← inline toggle
 *   │ └─────────────────────────┘ │
 *   │       [ Delete Alarm ]      │   ← red, danger-style
 *   └─────────────────────────────┘
 */

typedef struct
{
    lv_obj_t *time_lbl;
    lv_obj_t *repeat_value_lbl;
    lv_obj_t *snooze_sw;

    /* Delete button: two-tap confirmation. First tap morphs the label to
       "Tap again to confirm"; a 2 s timer resets it if the user changes
       their mind. Second tap (within the window) actually fires DELETE. */
    lv_obj_t *delete_btn;
    lv_obj_t *delete_lbl;
    lv_timer_t *delete_reset_timer;
    bool delete_armed;

    int32_t alarm_idx;
    alarm_contxt_t alarm_ctx;
    datac_handle_t srv_handle;
    bool received_data;  /* gates UI sync events until first GET completes */
} app_alarm_edit_t;

static app_alarm_edit_t *p_app_alarm_edit = NULL;

/*********************
 *   STATIC HELPERS
 *********************/

static const char *weekday_short_i18n(int idx) /* idx: 0=Sun .. 6=Sat */
{
    switch (idx) {
        case 0: return LV_EXT_STR_GET_BY_KEY(Sun, "Sun");
        case 1: return LV_EXT_STR_GET_BY_KEY(Mon, "Mon");
        case 2: return LV_EXT_STR_GET_BY_KEY(Tue, "Tue");
        case 3: return LV_EXT_STR_GET_BY_KEY(Wed, "Wed");
        case 4: return LV_EXT_STR_GET_BY_KEY(Thu, "Thu");
        case 5: return LV_EXT_STR_GET_BY_KEY(Fri, "Fri");
        case 6: return LV_EXT_STR_GET_BY_KEY(Sat, "Sat");
        default: return "";
    }
}

static void format_repeat_label(uint8_t days, char *out, size_t out_size)
{
    if (days == 0x00) { rt_snprintf(out, out_size, LV_EXT_STR_GET_BY_KEY(Once, "Once")); return; }
    if (days == 0x7F) { rt_snprintf(out, out_size, LV_EXT_STR_GET_BY_KEY(alarm_every_day, "Every Day")); return; }
    if (days == 0x3E) { rt_snprintf(out, out_size, LV_EXT_STR_GET_BY_KEY(alarm_weekdays, "Weekdays")); return; }
    if (days == 0x41) { rt_snprintf(out, out_size, LV_EXT_STR_GET_BY_KEY(alarm_weekends, "Weekends")); return; }

    static const char *labels[7] = {"Sun", "Mon", "Tue", "Wed",
                                    "Thu", "Fri", "Sat"};
    size_t pos = 0;
    out[0] = '\0';
    for (int i = 0; i < 7; i++)
    {
        if ((days >> i) & 1)
        {
            int n = rt_snprintf(out + pos, out_size - pos,
                                pos == 0 ? "%s" : ", %s", weekday_short_i18n(i));
            if (n <= 0 || (size_t)n >= out_size - pos) break;
            pos += n;
        }
    }
}

static void send_edit(void)
{
    if (!p_app_alarm_edit->received_data) return;
    data_msg_t msg;
    alarm_msg_t *p = (alarm_msg_t *)data_service_init_msg(
        &msg, ALARMMGR_MSG_EDIT_ALARM_REQ, sizeof(alarm_msg_t));
    p->idx = p_app_alarm_edit->alarm_idx;
    memcpy(&p->ctx, &p_app_alarm_edit->alarm_ctx, sizeof(alarm_contxt_t));
    datac_send_msg(p_app_alarm_edit->srv_handle, &msg);
}

/*********************
 *   EVENT CALLBACKS
 *********************/

static void time_card_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    extern void app_alarm_edit_time(int32_t alarm_idx);
    app_alarm_edit_time(p_app_alarm_edit->alarm_idx);
}

static void repeat_card_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    extern void app_alarm_edit_repeat(uint32_t alarm_idx);
    app_alarm_edit_repeat(p_app_alarm_edit->alarm_idx);
}

static void snooze_switch_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *sw = lv_event_get_target(e);
    p_app_alarm_edit->alarm_ctx.snooze =
        lv_obj_has_state(sw, LV_STATE_CHECKED) ? ALARM_SNOOZE_ENABLE
                                                : ALARM_SNOOZE_DISABLE;
    send_edit();
}

static void delete_disarm(void)
{
    if (!p_app_alarm_edit) return;
    p_app_alarm_edit->delete_armed = false;
    if (p_app_alarm_edit->delete_lbl)
        lv_label_set_text(p_app_alarm_edit->delete_lbl, LV_EXT_STR_GET_BY_KEY(delete_alarm, "Delete Alarm"));
    if (p_app_alarm_edit->delete_reset_timer)
    {
        lv_timer_del(p_app_alarm_edit->delete_reset_timer);
        p_app_alarm_edit->delete_reset_timer = NULL;
    }
}

static void delete_disarm_timer_cb(lv_timer_t *t)
{
    (void)t;
    delete_disarm();
}

static void delete_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    /* First tap: arm the button — change label, schedule auto-disarm.
       This avoids accidental deletion when the user briefly grazes the
       button: a real delete needs two deliberate taps. */
    if (!p_app_alarm_edit->delete_armed)
    {
        p_app_alarm_edit->delete_armed = true;
        lv_label_set_text(p_app_alarm_edit->delete_lbl, LV_EXT_STR_GET_BY_KEY(tap_confirm, "Tap again to confirm"));
        p_app_alarm_edit->delete_reset_timer = lv_timer_create(
            delete_disarm_timer_cb, 2000, NULL);
        lv_timer_set_repeat_count(p_app_alarm_edit->delete_reset_timer, 1);
        return;
    }

    /* Second tap within the window: actually delete. */
    delete_disarm();

    data_msg_t msg;
    alarm_msg_t *p = (alarm_msg_t *)data_service_init_msg(
        &msg, ALARMMGR_MSG_DELETE_ALARM_REQ, sizeof(alarm_msg_t));
    p->idx = p_app_alarm_edit->alarm_idx;
    datac_send_msg(p_app_alarm_edit->srv_handle, &msg);
}

/*********************
 *   UI BUILDERS
 *********************/

static void style_ios_switch(lv_obj_t *sw)
{
    lv_obj_set_size(sw, 56, 32);
    lv_obj_set_style_bg_color(sw, ALARM_COLOR_SWITCH_OFF, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, ALARM_COLOR_SWITCH_ON,
                              LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_radius(sw, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_radius(sw, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_radius(sw, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_pad_all(sw, 2, LV_PART_MAIN);
}

static lv_obj_t *create_setting_card(lv_obj_t *parent)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, LV_PCT(100), 64);
    lv_obj_set_style_bg_color(card, ALARM_COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, ALARM_CARD_RADIUS, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_set_style_pad_left(card, 20, 0);
    lv_obj_set_style_pad_right(card, 16, 0);
    lv_obj_set_style_pad_top(card, 0, 0);
    lv_obj_set_style_pad_bottom(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    return card;
}

static void build_layout(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, ALARM_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* Vertical scroll container — round screens look best with everything
       reachable via scroll, no fixed header. */
    lv_obj_t *root = lv_obj_create(scr);
    lv_obj_set_size(root, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(root, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_set_style_pad_hor(root, (LV_HOR_RES_MAX - ALARM_LIST_W) / 2, 0);
    lv_obj_set_style_pad_top(root, 30, 0);
    lv_obj_set_style_pad_bottom(root, 30, 0);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(root, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(root, 14, 0);
    lv_obj_set_scrollbar_mode(root, LV_SCROLLBAR_MODE_OFF);

    /* ── Big time at top — tap to edit time. ── */
    lv_obj_t *time_card = lv_obj_create(root);
    lv_obj_set_size(time_card, LV_PCT(100), 90);
    lv_obj_set_style_bg_opa(time_card, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(time_card, 0, 0);
    lv_obj_set_style_pad_all(time_card, 0, 0);
    lv_obj_clear_flag(time_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(time_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(time_card, time_card_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *time_lbl = lv_label_create(time_card);
    lv_label_set_text(time_lbl, "--:--");
    lv_obj_set_style_text_font(
        time_lbl, LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_set_style_text_color(time_lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_center(time_lbl);
    p_app_alarm_edit->time_lbl = time_lbl;

    /* ── Repeat row. ── */
    lv_obj_t *repeat_card = create_setting_card(root);
    lv_obj_add_flag(repeat_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(repeat_card, repeat_card_event_cb, LV_EVENT_CLICKED,
                        NULL);

    lv_obj_t *repeat_lbl = lv_label_create(repeat_card);
    lv_label_set_text(repeat_lbl, LV_EXT_STR_GET_BY_KEY(alarm_repeat, "Repeat"));
    lv_obj_set_style_text_font(
        repeat_lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(repeat_lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(repeat_lbl, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *repeat_chevron = lv_label_create(repeat_card);
    lv_label_set_text(repeat_chevron, ">");
    lv_obj_set_style_text_color(repeat_chevron, ALARM_COLOR_TEXT_DIM, 0);
    lv_obj_set_style_text_font(
        repeat_chevron, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align(repeat_chevron, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *repeat_value = lv_label_create(repeat_card);
    lv_label_set_text(repeat_value, LV_EXT_STR_GET_BY_KEY(Once, "Once"));
    lv_obj_set_style_text_color(repeat_value, ALARM_COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_text_font(
        repeat_value, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(repeat_value, LV_ALIGN_RIGHT_MID, -22, 0);
    p_app_alarm_edit->repeat_value_lbl = repeat_value;

    /* ── Snooze row with iOS toggle. ── */
    lv_obj_t *snooze_card = create_setting_card(root);

    lv_obj_t *snooze_lbl = lv_label_create(snooze_card);
    lv_label_set_text(snooze_lbl, LV_EXT_STR_GET_BY_KEY(snooze, "Snooze"));
    lv_obj_set_style_text_font(
        snooze_lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(snooze_lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(snooze_lbl, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *sw = lv_switch_create(snooze_card);
    style_ios_switch(sw);
    lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(sw, snooze_switch_event_cb, LV_EVENT_VALUE_CHANGED,
                        NULL);
    p_app_alarm_edit->snooze_sw = sw;

    /* ── Delete button — red, full-width pill. Two-tap confirmation. ── */
    lv_obj_t *del_btn = lv_btn_create(root);
    lv_obj_set_size(del_btn, LV_PCT(100), 64);
    lv_obj_set_style_bg_color(del_btn, ALARM_COLOR_DANGER, 0);
    lv_obj_set_style_bg_opa(del_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(del_btn, ALARM_CARD_RADIUS, 0);
    lv_obj_set_style_border_width(del_btn, 0, 0);
    lv_obj_set_style_shadow_width(del_btn, 0, 0);
    lv_obj_add_event_cb(del_btn, delete_btn_event_cb, LV_EVENT_CLICKED, NULL);
    p_app_alarm_edit->delete_btn = del_btn;

    lv_obj_t *del_lbl = lv_label_create(del_btn);
    lv_label_set_text(del_lbl, LV_EXT_STR_GET_BY_KEY(delete_alarm, "Delete Alarm"));
    lv_obj_set_style_text_color(del_lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_font(
        del_lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_center(del_lbl);
    p_app_alarm_edit->delete_lbl = del_lbl;
}

/*********************
 *   DATA SYNC
 *********************/

static void sync_ui_from_ctx(void)
{
    char buf[48];

    rt_snprintf(buf, sizeof(buf), "%02d:%02d",
                p_app_alarm_edit->alarm_ctx.hour,
                p_app_alarm_edit->alarm_ctx.minute);
    lv_label_set_text(p_app_alarm_edit->time_lbl, buf);

    format_repeat_label(p_app_alarm_edit->alarm_ctx.days, buf, sizeof(buf));
    lv_label_set_text(p_app_alarm_edit->repeat_value_lbl, buf);

    if (p_app_alarm_edit->alarm_ctx.snooze == ALARM_SNOOZE_ENABLE)
        lv_obj_add_state(p_app_alarm_edit->snooze_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(p_app_alarm_edit->snooze_sw, LV_STATE_CHECKED);
}

static int srv_msg_handler(data_callback_arg_t *arg)
{
    if (!p_app_alarm_edit && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
        return 0;

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (p_app_alarm_edit && rsp->result >= 0)
        {
            data_msg_t msg;
            alarm_msg_t *p = (alarm_msg_t *)data_service_init_msg(
                &msg, ALARMMGR_MSG_GET_ALARM_REQ, sizeof(alarm_msg_t));
            p->idx = p_app_alarm_edit->alarm_idx;
            datac_send_msg(p_app_alarm_edit->srv_handle, &msg);
        }
        break;
    }

    case ALARMMGR_MSG_GET_ALARM_RSP:
    {
        alarm_msg_t *data = (alarm_msg_t *)arg->data;
        if (data)
        {
            RT_ASSERT(p_app_alarm_edit->alarm_idx == data->idx);
            memcpy(&p_app_alarm_edit->alarm_ctx, &data->ctx,
                   sizeof(alarm_contxt_t));
            p_app_alarm_edit->received_data = true;
            sync_ui_from_ctx();
        }
        break;
    }

    case ALARMMGR_MSG_DELETE_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;
        if (rsp->result != 0) LOG_I("alarm delete err: %d", rsp->result);
        else
        {
            watch_alarm_local_sync(2 /* DELETE */,
                                   p_app_alarm_edit->alarm_idx, NULL);
            gui_app_goback();
        }
        break;
    }

    case ALARMMGR_MSG_EDIT_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;
        LOG_D("alarm edit err: %d", rsp->result);
        if (rsp->result == 0)
        {
            watch_alarm_local_sync(1 /* EDIT */,
                                   p_app_alarm_edit->alarm_idx,
                                   &p_app_alarm_edit->alarm_ctx);
        }
        break;
    }

    default:
        break;
    }
    return 0;
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
    if (p_app_alarm_edit->srv_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        p_app_alarm_edit->srv_handle = datac_open();
        RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_alarm_edit->srv_handle);
    }
    ui_datac_subscribe(p_app_alarm_edit->srv_handle, "alarmmgr",
                       srv_msg_handler, 0);
}

static void on_pause(void)
{
    /* Cancel any pending delete-disarm timer before tearing down — it
       references p_app_alarm_edit via callback. */
    if (p_app_alarm_edit->delete_reset_timer)
    {
        lv_timer_del(p_app_alarm_edit->delete_reset_timer);
        p_app_alarm_edit->delete_reset_timer = NULL;
    }
    if (p_app_alarm_edit->srv_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_close(p_app_alarm_edit->srv_handle);
        p_app_alarm_edit->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    }
}

static void on_stop(void)
{
    if (p_app_alarm_edit)
    {
        if (p_app_alarm_edit->delete_reset_timer)
        {
            lv_timer_del(p_app_alarm_edit->delete_reset_timer);
            p_app_alarm_edit->delete_reset_timer = NULL;
        }
        lv_mem_free(p_app_alarm_edit);
        p_app_alarm_edit = NULL;
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

void app_alarm_edit(uint32_t alarm_idx)
{
    rt_kprintf("app_alarm_edit[%d] entry\r\n", alarm_idx);

    RT_ASSERT(NULL == p_app_alarm_edit);
    p_app_alarm_edit = (app_alarm_edit_t *)lv_mem_alloc(sizeof(app_alarm_edit_t));
    memset(p_app_alarm_edit, 0, sizeof(app_alarm_edit_t));
    p_app_alarm_edit->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    p_app_alarm_edit->alarm_idx = alarm_idx;

    gui_app_create_page(SUBPAGE_NAME, msg_handler);
}
