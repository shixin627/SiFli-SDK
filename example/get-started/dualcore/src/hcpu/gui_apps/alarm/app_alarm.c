/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "alarm_manager_service.h"
#include "alarm_client.h"
#include "ui_datasrv_subscriber.h"
#include "ui_handler.h"
#include "ui_helper.h"
#include "ui_img_helper.h"
#ifdef APP_ID_ALARM

/**
 * Apple-style alarm list:
 *   [Alarm                            +]
 *   ┌──────────────────────────────────┐
 *   │ 07:30                         ◉  │
 *   │ Mon, Wed, Fri                    │
 *   ├──────────────────────────────────┤
 *   │ 09:00                         ◯  │   (disabled — dimmed)
 *   │ Weekends                         │
 *   └──────────────────────────────────┘
 *
 * The header sits above a scrollable list of alarm cards. Each card shows
 * the time, the repeat-day summary, and an iOS-style toggle. Tapping the
 * card body opens app_alarm_edit() to change time/repeat/delete; tapping
 * the toggle fires ENABLE_ALARM_REQ / DISABLE_ALARM_REQ. Tapping "+" at
 * the top-right opens app_alarm_edit_time(-1) to add a new alarm. The
 * existing alarm_manager_service IPC and the BLE-side sync paths are
 * untouched — this file only owns the list UI.
 */

/*********************
 *      DEFINES
 *********************/
#define ALARM_LIST_W 420
#define ALARM_CARD_H 96
#define ALARM_CARD_RADIUS 16
#define HEADER_H 64

#define COLOR_BG          lv_color_hex(0x000000)
#define COLOR_CARD        lv_color_hex(0x1C1C1E)  /* iOS dark surface */
#define COLOR_DIVIDER     lv_color_hex(0x2C2C2E)
#define COLOR_TEXT_PRIMARY    lv_color_hex(0xFFFFFF)
#define COLOR_TEXT_SECONDARY  lv_color_hex(0x8E8E93)
#define COLOR_TEXT_DIM        lv_color_hex(0x6E6E73)
#define COLOR_SWITCH_ON   lv_color_hex(0x34C759)  /* iOS green */
#define COLOR_SWITCH_OFF  lv_color_hex(0x39393D)
#define COLOR_ACCENT      lv_color_hex(0xFF9F0A)  /* iOS Alarm orange */

typedef struct
{
    lv_obj_t *list_cnt;       /* scroll container holding the cards */
    lv_obj_t *empty_label;    /* shown when list is empty */
    datac_handle_t srv_handle;
    bool first_resume;
    uint8_t alarm_num;
} app_alarm_t;

static app_alarm_t *p_app_alarm = NULL;

/*********************
 *   STATIC HELPERS
 *********************/

/**
 * Format the alarm repeat-day bitmask in iOS style. Bit layout matches the
 * runtime tm_wday convention: bit0=Sun, bit1=Mon, ..., bit6=Sat.
 */
static void format_repeat(uint8_t days, char *out, size_t out_size)
{
    if (days == 0x00)
    {
        rt_snprintf(out, out_size, "Once");
        return;
    }
    if (days == 0x7F)
    {
        rt_snprintf(out, out_size, "Every Day");
        return;
    }
    if (days == 0x3E) /* Mon-Fri = bits 1..5 */
    {
        rt_snprintf(out, out_size, "Weekdays");
        return;
    }
    if (days == 0x41) /* Sun + Sat = bits 0,6 */
    {
        rt_snprintf(out, out_size, "Weekends");
        return;
    }

    static const char *labels[7] = {"Sun", "Mon", "Tue", "Wed",
                                    "Thu", "Fri", "Sat"};
    size_t pos = 0;
    out[0] = '\0';
    for (int i = 0; i < 7; i++)
    {
        if ((days >> i) & 1)
        {
            int n = rt_snprintf(out + pos, out_size - pos,
                                pos == 0 ? "%s" : ", %s", labels[i]);
            if (n <= 0 || (size_t)n >= out_size - pos) break;
            pos += n;
        }
    }
}

/* Stash a small integer (alarm index) in an lv_obj's user_data slot. The
   pointer is never dereferenced; we just round-trip the bits. */
static inline void *idx_to_user_data(int32_t idx)
{
    return (void *)(uintptr_t)(idx + 1); /* +1 so 0 isn't NULL-confused */
}

static inline int32_t user_data_to_idx(const void *ud)
{
    return (int32_t)((uintptr_t)ud) - 1;
}

/*********************
 *   EVENT CALLBACKS
 *********************/

static void switch_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    lv_obj_t *sw = lv_event_get_target(e);
    int32_t idx = user_data_to_idx(lv_obj_get_user_data(sw));

    data_msg_t msg;
    alarm_msg_t *p;
    bool enable = lv_obj_has_state(sw, LV_STATE_CHECKED);
    p = (alarm_msg_t *)data_service_init_msg(
        &msg,
        enable ? ALARMMGR_MSG_ENABLE_ALARM_REQ : ALARMMGR_MSG_DISABLE_ALARM_REQ,
        sizeof(alarm_msg_t));
    p->idx = idx;
    datac_send_msg(p_app_alarm->srv_handle, &msg);
    /* Mirror the new state to the phone so its UI doesn't drift. */
    bloc_alarm_push_to_phone();
}

static void card_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    lv_obj_t *card = lv_event_get_target(e);
    int32_t idx = user_data_to_idx(lv_obj_get_user_data(card));

    extern void app_alarm_edit(uint32_t alarm_idx);
    app_alarm_edit((uint32_t)idx);
}

static void add_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (p_app_alarm->alarm_num >= BSP_ALARM_MAX)
    {
        lv_obj_t *msgbox = lv_msgbox_create(NULL, "Alarms",
                                            "Maximum reached.", NULL, true);
        lv_obj_center(msgbox);
        return;
    }
    extern void app_alarm_edit_time(int32_t alarm_idx);
    app_alarm_edit_time(-1);
}

/*********************
 *   UI BUILDERS
 *********************/

static void style_switch_ios(lv_obj_t *sw)
{
    lv_obj_set_size(sw, 56, 32);
    lv_obj_set_style_bg_color(sw, COLOR_SWITCH_OFF, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, COLOR_SWITCH_ON,
                              LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_radius(sw, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_radius(sw, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_radius(sw, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_pad_all(sw, 2, LV_PART_MAIN);
}

static void create_alarm_card(lv_obj_t *parent, const alarm_msg_t *alarm)
{
    bool enabled = (alarm->ctx.state == ALARM_STATE_ENABLE);

    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, LV_PCT(100), ALARM_CARD_H);
    lv_obj_set_style_bg_color(card, COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, ALARM_CARD_RADIUS, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_set_style_pad_left(card, 20, 0);
    lv_obj_set_style_pad_right(card, 12, 0);
    lv_obj_set_style_pad_top(card, 12, 0);
    lv_obj_set_style_pad_bottom(card, 12, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_user_data(card, idx_to_user_data(alarm->idx));
    lv_obj_add_event_cb(card, card_event_cb, LV_EVENT_CLICKED, NULL);

    /* Time label — large, dim when disabled. */
    char time_buf[8];
    rt_snprintf(time_buf, sizeof(time_buf), "%02d:%02d",
                alarm->ctx.hour, alarm->ctx.minute);
    lv_obj_t *time_lbl = lv_label_create(card);
    lv_label_set_text(time_lbl, time_buf);
    lv_obj_set_style_text_font(
        time_lbl, LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_obj_set_style_text_color(
        time_lbl, enabled ? COLOR_TEXT_PRIMARY : COLOR_TEXT_DIM, 0);
    lv_obj_align(time_lbl, LV_ALIGN_LEFT_MID, 0, -14);

    /* Repeat-day subtitle. */
    char repeat_buf[48];
    format_repeat(alarm->ctx.days, repeat_buf, sizeof(repeat_buf));
    lv_obj_t *sub_lbl = lv_label_create(card);
    lv_label_set_text(sub_lbl, repeat_buf);
    lv_obj_set_style_text_font(
        sub_lbl, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(sub_lbl, COLOR_TEXT_SECONDARY, 0);
    lv_obj_align(sub_lbl, LV_ALIGN_LEFT_MID, 0, 22);

    /* Toggle switch. */
    lv_obj_t *sw = lv_switch_create(card);
    style_switch_ios(sw);
    if (enabled) lv_obj_add_state(sw, LV_STATE_CHECKED);
    lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_user_data(sw, idx_to_user_data(alarm->idx));
    lv_obj_add_event_cb(sw, switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    /* LVGL doesn't bubble events by default, so a tap on the switch won't
       also trigger the card's CLICKED handler. */
}

static void show_empty_state(bool show)
{
    if (!p_app_alarm) return;
    if (p_app_alarm->empty_label)
    {
        if (show)
            lv_obj_clear_flag(p_app_alarm->empty_label, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(p_app_alarm->empty_label, LV_OBJ_FLAG_HIDDEN);
    }
}

/*********************
 *   DATA SERVICE
 *********************/

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
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (p_app_alarm && rsp->result >= 0)
        {
            data_msg_t msg;
            data_service_init_msg(
                &msg, ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ, 0);
            datac_send_msg(p_app_alarm->srv_handle, &msg);
        }
        break;
    }

    case ALARMMGR_MSG_GET_ALARM_LIST_NEXT_RSP:
    {
        alarm_msg_t *data = (alarm_msg_t *)arg->data;
        if (data)
        {
            create_alarm_card(p_app_alarm->list_cnt, data);
            p_app_alarm->alarm_num++;

            /* Request next alarm in the list. */
            data_msg_t msg;
            uint8_t *body = data_service_init_msg(
                &msg, ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ,
                sizeof(alarm_msg_t));
            memcpy(body, arg->data, sizeof(alarm_msg_t));
            datac_send_msg(p_app_alarm->srv_handle, &msg);
        }
        else
        {
            /* End of list — show empty state if no alarms loaded. */
            show_empty_state(p_app_alarm->alarm_num == 0);
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

static void build_layout(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* ── Header: "Alarm" title + "+" button ── */
    lv_obj_t *header = lv_obj_create(scr);
    lv_obj_set_size(header, LV_HOR_RES_MAX, HEADER_H);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_pad_all(header, 0, 0);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(header);
    lv_label_set_text(title, "Alarm");
    lv_obj_set_style_text_font(
        title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(title, COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *add_btn = lv_btn_create(header);
    lv_obj_set_size(add_btn, 44, 44);
    lv_obj_align(add_btn, LV_ALIGN_RIGHT_MID, -16, 0);
    lv_obj_set_style_bg_color(add_btn, COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(add_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(add_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(add_btn, 0, 0);
    lv_obj_set_style_shadow_width(add_btn, 0, 0);
    lv_obj_add_event_cb(add_btn, add_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *plus = lv_label_create(add_btn);
    lv_label_set_text(plus, "+");
    lv_obj_set_style_text_color(plus, COLOR_ACCENT, 0);
    lv_obj_set_style_text_font(
        plus, LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_obj_center(plus);

    /* ── Scrollable list of cards ── */
    lv_obj_t *list = lv_obj_create(scr);
    lv_obj_set_size(list, ALARM_LIST_W, LV_VER_RES_MAX - HEADER_H - 20);
    lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 8, 0);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(list, 10, 0);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);

    p_app_alarm->list_cnt = list;

    /* ── Empty state — sits behind the list, shown when alarm_num == 0. ── */
    lv_obj_t *empty = lv_label_create(scr);
    lv_label_set_text(empty, "No Alarms");
    lv_obj_set_style_text_font(
        empty, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(empty, COLOR_TEXT_SECONDARY, 0);
    lv_obj_align(empty, LV_ALIGN_CENTER, 0, 20);
    lv_obj_add_flag(empty, LV_OBJ_FLAG_HIDDEN);
    p_app_alarm->empty_label = empty;
}

static void on_start(void)
{
    RT_ASSERT(NULL == p_app_alarm);
    p_app_alarm = (app_alarm_t *)lv_mem_alloc(sizeof(app_alarm_t));
    memset(p_app_alarm, 0, sizeof(app_alarm_t));
    p_app_alarm->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    p_app_alarm->first_resume = true;

    build_layout();
}

static void on_resume(void)
{
    p_app_alarm->alarm_num = 0;
    show_empty_state(false);

    p_app_alarm->srv_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_alarm->srv_handle);
    ui_datac_subscribe(p_app_alarm->srv_handle, "alarmmgr", srv_msg_handler, 0);

    if (p_app_alarm->first_resume)
    {
        p_app_alarm->first_resume = false;
    }
}

static void on_pause(void)
{
    if (p_app_alarm->srv_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_close(p_app_alarm->srv_handle);
        p_app_alarm->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    }
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
    case GUI_APP_MSG_ONSTART:  on_start();  break;
    case GUI_APP_MSG_ONRESUME: on_resume(); break;
    case GUI_APP_MSG_ONPAUSE:  on_pause();  break;
    case GUI_APP_MSG_ONSTOP:   on_stop();   break;
    default: break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_ALARM, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(alarm), IMG_ALARM, APP_ID_ALARM, app_main);
#endif
