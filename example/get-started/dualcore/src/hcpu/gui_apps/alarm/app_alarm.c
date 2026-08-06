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
#include "app_alarm_style.h"
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
#define ALARM_CARD_H 96
#define HEADER_H 64
#define FOOTER_H 88        /* room for the floating "+" button */
#define ADD_BTN_SIZE 64    /* round, sits on the optical center bottom */

typedef enum
{
    ALARM_VIEW_LIST,
    ALARM_VIEW_RINGING,
} alarm_view_t;

typedef struct
{
    /* List view */
    lv_obj_t *list_cnt;       /* scroll container holding the cards */
    lv_obj_t *empty_label;    /* shown when list is empty */
    lv_obj_t *list_root;      /* groups all list-mode widgets for show/hide */

    /* Ringing view */
    lv_obj_t *ringing_root;
    lv_obj_t *ringing_time_lbl;

    /* Polling */
    lv_timer_t *fire_poll_timer; /* checks bloc_alarm_get_ringing_idx */

    datac_handle_t srv_handle;
    alarm_view_t view;
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

static void format_repeat(uint8_t days, char *out, size_t out_size)
{
    if (days == 0x00)
    {
        rt_snprintf(out, out_size, LV_EXT_STR_GET_BY_KEY(Once, "Once"));
        return;
    }
    if (days == 0x7F)
    {
        rt_snprintf(out, out_size, LV_EXT_STR_GET_BY_KEY(alarm_every_day, "Every Day"));
        return;
    }
    if (days == 0x3E) /* Mon-Fri = bits 1..5 */
    {
        rt_snprintf(out, out_size, LV_EXT_STR_GET_BY_KEY(alarm_weekdays, "Weekdays"));
        return;
    }
    if (days == 0x41) /* Sun + Sat = bits 0,6 */
    {
        rt_snprintf(out, out_size, LV_EXT_STR_GET_BY_KEY(alarm_weekends, "Weekends"));
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
                                pos == 0 ? "%s" : ", %s", weekday_short_i18n(i));
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

    /* Mirror the enable/disable into SkaiWatchSys.alarms[] reserved bit so
       phone polls see the latest state and persistence reflects it after
       reboot. memcpy through a local T_ALARM avoids the unaligned 64-bit
       bit-field RMW that triggers SCB_CFSR.UNALIGNED on Cortex-M. */
    if (idx >= 0 && idx < SkaiWatchSys.alarm_num)
    {
        T_ALARM tmp;
        memcpy(&tmp, &SkaiWatchSys.alarms[idx], sizeof(T_ALARM));
        tmp.alarm.reserved = enable ? 0x1 : 0x0;
        memcpy(&SkaiWatchSys.alarms[idx], &tmp, sizeof(T_ALARM));
        watch_prefs_save_alarms();
    }
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
        lv_obj_t *msgbox = lv_msgbox_create(NULL,
                                            LV_EXT_STR_GET_BY_KEY(alarm_max_title, "Alarms"),
                                            LV_EXT_STR_GET_BY_KEY(alarm_max_reached, "Maximum reached."),
                                            NULL, true);
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
    lv_obj_set_style_bg_color(sw, ALARM_COLOR_SWITCH_OFF, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, ALARM_COLOR_SWITCH_ON,
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
    lv_obj_set_style_bg_color(card, ALARM_COLOR_CARD, 0);
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
    char time_buf[12]; /* fits "12:30 PM" + NUL in 12-hour mode */
    ui_time_format_hhmm(time_buf, sizeof(time_buf),
                        alarm->ctx.hour, alarm->ctx.minute);
    lv_obj_t *time_lbl = lv_label_create(card);
    lv_label_set_text(time_lbl, time_buf);
    lv_obj_set_style_text_font(
        time_lbl, LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_obj_set_style_text_color(
        time_lbl, enabled ? ALARM_COLOR_TEXT_PRIMARY : ALARM_COLOR_TEXT_DIM, 0);
    lv_obj_align(time_lbl, LV_ALIGN_LEFT_MID, 0, -14);

    /* Repeat-day subtitle. */
    char repeat_buf[48];
    format_repeat(alarm->ctx.days, repeat_buf, sizeof(repeat_buf));
    lv_obj_t *sub_lbl = lv_label_create(card);
    lv_label_set_text(sub_lbl, repeat_buf);
    lv_obj_set_style_text_font(
        sub_lbl, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(sub_lbl, ALARM_COLOR_TEXT_SECONDARY, 0);
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
 *   RINGING VIEW
 *********************/

static void show_view(alarm_view_t v);

static void stop_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    bloc_alarm_stop_ringing(false);
    /* Exit back to the previous screen — matches Apple Watch behaviour. */
    gui_app_goback();
}

static void snooze_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    bloc_alarm_stop_ringing(true);
    /* Snooze creates a one-shot 5 min later — leave the app for now;
       it'll re-launch in ringing mode when the snooze fires. */
    gui_app_goback();
}

static void build_ringing_view(lv_obj_t *parent)
{
    lv_obj_t *root = lv_obj_create(parent);
    lv_obj_set_size(root, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(root, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(root, ALARM_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(root, LV_OBJ_FLAG_HIDDEN);
    p_app_alarm->ringing_root = root;

    /* "Alarm" label at top, in accent orange. */
    lv_obj_t *kicker = lv_label_create(root);
    lv_label_set_text(kicker, LV_EXT_STR_GET_BY_KEY(alarm, "Alarm"));
    lv_obj_set_style_text_color(kicker, ALARM_COLOR_ACCENT, 0);
    lv_obj_set_style_text_font(
        kicker, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align(kicker, LV_ALIGN_TOP_MID, 0, 80);

    /* Big time, fills the optical center. */
    lv_obj_t *time_lbl = lv_label_create(root);
    lv_label_set_text(time_lbl, "00:00");
    lv_obj_set_style_text_color(time_lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_font(
        time_lbl, LV_EXT_FONT_GET(get_system_font_size(4)), 0);
    lv_obj_align(time_lbl, LV_ALIGN_CENTER, 0, -30);
    p_app_alarm->ringing_time_lbl = time_lbl;

    /* Snooze button (secondary) — left of bottom row. */
    lv_obj_t *snooze_btn = lv_btn_create(root);
    lv_obj_set_size(snooze_btn, 140, 56);
    lv_obj_align(snooze_btn, LV_ALIGN_BOTTOM_MID, -80, -30);
    lv_obj_set_style_bg_color(snooze_btn, ALARM_COLOR_NEUTRAL, 0);
    lv_obj_set_style_bg_opa(snooze_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(snooze_btn, 28, 0);
    lv_obj_set_style_border_width(snooze_btn, 0, 0);
    lv_obj_set_style_shadow_width(snooze_btn, 0, 0);
    lv_obj_add_event_cb(snooze_btn, snooze_btn_event_cb, LV_EVENT_CLICKED,
                        NULL);
    lv_obj_t *snooze_lbl = lv_label_create(snooze_btn);
    lv_label_set_text(snooze_lbl, "Snooze");
    lv_obj_set_style_text_color(snooze_lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_font(
        snooze_lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_center(snooze_lbl);

    /* Stop button (primary danger) — right of bottom row. */
    lv_obj_t *stop_btn = lv_btn_create(root);
    lv_obj_set_size(stop_btn, 140, 56);
    lv_obj_align(stop_btn, LV_ALIGN_BOTTOM_MID, 80, -30);
    lv_obj_set_style_bg_color(stop_btn, ALARM_COLOR_DANGER, 0);
    lv_obj_set_style_bg_opa(stop_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(stop_btn, 28, 0);
    lv_obj_set_style_border_width(stop_btn, 0, 0);
    lv_obj_set_style_shadow_width(stop_btn, 0, 0);
    lv_obj_add_event_cb(stop_btn, stop_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *stop_lbl = lv_label_create(stop_btn);
    lv_label_set_text(stop_lbl, "Stop");
    lv_obj_set_style_text_color(stop_lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_font(
        stop_lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_center(stop_lbl);
}

/*********************
 *   LIST VIEW
 *********************/

static void build_list_view(lv_obj_t *parent)
{
    /* Group everything list-related under one root so we can hide/show in
       one call when flipping to/from the ringing view. */
    lv_obj_t *root = lv_obj_create(parent);
    lv_obj_set_size(root, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(root, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    p_app_alarm->list_root = root;

    /* Header: title only, centred — round screens clip the corners. */
    lv_obj_t *title = lv_label_create(root);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(alarm, "Alarm"));
    lv_obj_set_style_text_font(
        title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(title, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

    /* Scrollable list of alarm cards. */
    lv_obj_t *list = lv_obj_create(root);
    lv_obj_set_size(list, ALARM_LIST_W,
                    LV_VER_RES_MAX - HEADER_H - FOOTER_H);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, HEADER_H);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 8, 0);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(list, 10, 0);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);
    p_app_alarm->list_cnt = list;

    /* "+" floating button at bottom-centre — only safe area on a round face. */
    lv_obj_t *add_btn = lv_btn_create(root);
    lv_obj_set_size(add_btn, ADD_BTN_SIZE, ADD_BTN_SIZE);
    lv_obj_align(add_btn, LV_ALIGN_BOTTOM_MID, 0, -16);
    lv_obj_set_style_bg_color(add_btn, ALARM_COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(add_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(add_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(add_btn, 0, 0);
    lv_obj_set_style_shadow_width(add_btn, 16, 0);
    lv_obj_set_style_shadow_color(add_btn, lv_color_black(), 0);
    lv_obj_set_style_shadow_opa(add_btn, LV_OPA_30, 0);
    lv_obj_add_event_cb(add_btn, add_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *plus = lv_label_create(add_btn);
    lv_label_set_text(plus, "+");
    lv_obj_set_style_text_color(plus, ALARM_COLOR_ACCENT, 0);
    lv_obj_set_style_text_font(
        plus, LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_center(plus);

    /* "No Alarms" empty state — hidden until the list-load finishes empty. */
    lv_obj_t *empty = lv_label_create(root);
    lv_label_set_text(empty, LV_EXT_STR_GET_BY_KEY(no_alarms, "No Alarms"));
    lv_obj_set_style_text_font(
        empty, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(empty, ALARM_COLOR_TEXT_SECONDARY, 0);
    lv_obj_align(empty, LV_ALIGN_CENTER, 0, -10);
    lv_obj_add_flag(empty, LV_OBJ_FLAG_HIDDEN);
    p_app_alarm->empty_label = empty;
}

/*********************
 *   VIEW SWITCHING
 *********************/

static void enter_ringing_view(int32_t idx)
{
    char buf[12]; /* fits "12:30 PM" + NUL in 12-hour mode */
    /* Read time from the manager's stored alarm via SkaiWatchSys.alarms[]
       isn't reliable for snoozes; fetch from the request the user just made
       isn't easy either. Show wall-clock current time as a fallback — the
       alarm just fired, so this is the moment that matters. */
    ui_time_format_hhmm(buf, sizeof(buf),
                        SkaiWatchSys.Global_Time.hour,
                        SkaiWatchSys.Global_Time.minutes);
    lv_label_set_text(p_app_alarm->ringing_time_lbl, buf);
    show_view(ALARM_VIEW_RINGING);
    (void)idx;
}

static void show_view(alarm_view_t v)
{
    if (!p_app_alarm) return;
    p_app_alarm->view = v;
    if (v == ALARM_VIEW_RINGING)
    {
        if (p_app_alarm->list_root)
            lv_obj_add_flag(p_app_alarm->list_root, LV_OBJ_FLAG_HIDDEN);
        if (p_app_alarm->ringing_root)
            lv_obj_clear_flag(p_app_alarm->ringing_root, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        if (p_app_alarm->ringing_root)
            lv_obj_add_flag(p_app_alarm->ringing_root, LV_OBJ_FLAG_HIDDEN);
        if (p_app_alarm->list_root)
            lv_obj_clear_flag(p_app_alarm->list_root, LV_OBJ_FLAG_HIDDEN);
    }
}

static void fire_poll_cb(lv_timer_t *t)
{
    (void)t;
    if (!p_app_alarm) return;
    int32_t idx = bloc_alarm_get_ringing_idx();
    if (idx >= 0 && p_app_alarm->view != ALARM_VIEW_RINGING)
    {
        /* Alarm fired while user was on the list view — flip to ringing UI. */
        enter_ringing_view(idx);
    }
    else if (idx < 0 && p_app_alarm->view == ALARM_VIEW_RINGING)
    {
        /* External code stopped the ring (e.g. phone-side dismiss) — go back
           to the list. */
        show_view(ALARM_VIEW_LIST);
    }
}

/*********************
 *   LIFECYCLE
 *********************/

static void build_layout(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, ALARM_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    build_list_view(scr);
    build_ringing_view(scr);
}

static void on_start(void)
{
    RT_ASSERT(NULL == p_app_alarm);
    p_app_alarm = (app_alarm_t *)lv_mem_alloc(sizeof(app_alarm_t));
    memset(p_app_alarm, 0, sizeof(app_alarm_t));
    p_app_alarm->srv_handle = DATA_CLIENT_INVALID_HANDLE;

    build_layout();
}

static void on_resume(void)
{
    p_app_alarm->alarm_num = 0;
    show_empty_state(false);

    p_app_alarm->srv_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_alarm->srv_handle);
    ui_datac_subscribe(p_app_alarm->srv_handle, "alarmmgr", srv_msg_handler, 0);

    /* If we landed here because an alarm just fired, jump straight to the
       ringing view. Otherwise show the normal list. */
    int32_t ringing = bloc_alarm_get_ringing_idx();
    if (ringing >= 0)
        enter_ringing_view(ringing);
    else
        show_view(ALARM_VIEW_LIST);

    /* Poll while the app is in the foreground so we still flip into the
       ringing view if an alarm fires while the user is browsing the list. */
    if (!p_app_alarm->fire_poll_timer)
        p_app_alarm->fire_poll_timer = lv_timer_create(fire_poll_cb, 500, NULL);
}

static void on_pause(void)
{
    if (p_app_alarm->fire_poll_timer)
    {
        lv_timer_del(p_app_alarm->fire_poll_timer);
        p_app_alarm->fire_poll_timer = NULL;
    }
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
        if (p_app_alarm->fire_poll_timer)
        {
            lv_timer_del(p_app_alarm->fire_poll_timer);
            p_app_alarm->fire_poll_timer = NULL;
        }
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
    {
        /* app_run 直接開啟不經 Main 狀態機，左緣右滑返回 bar 仍隱藏，這裡補開 */
        extern void display_gesture_detect_objs(uint32_t idx, bool display);
        display_gesture_detect_objs(0, true);
        on_start();
        break;
    }
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

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(alarm), IMG_ALARM, APP_ID_ALARM, app_main, 1);
#endif
