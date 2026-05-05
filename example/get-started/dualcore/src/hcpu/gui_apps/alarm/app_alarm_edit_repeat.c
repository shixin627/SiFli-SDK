/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "alarm_manager_service.h"
#include "ui_datasrv_subscriber.h"
#include "ui_helper.h"
#include "alarm_client.h"
#include "app_alarm_style.h"

#define DBG_TAG "APP.ALARM.ER"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

#define SUBPAGE_NAME "alm_rpt"

/**
 * Apple-style repeat-day selector.
 *
 *   ┌─────────────────────────────┐
 *   │           Repeat            │
 *   │ ┌─────────────────────────┐ │
 *   │ │ Every Day            ✓  │ │  ← preset rows toggle on tap
 *   │ │ Weekdays                │ │
 *   │ │ Weekends                │ │
 *   │ ├─────────────────────────┤ │
 *   │ │ Sunday               ✓  │ │  ← individual day rows toggle bits
 *   │ │ Monday               ✓  │ │
 *   │ │ Tuesday                 │ │
 *   │ │ Wednesday               │ │
 *   │ │ ...                     │ │
 *   │ └─────────────────────────┘ │
 *   └─────────────────────────────┘
 *
 * Day-of-week bit layout: bit0=Sun, bit1=Mon, ..., bit6=Sat (matches the
 * runtime convention used by alarm_manager_service::setup_hw_alarm).
 */

typedef struct
{
    int32_t alarm_idx;
    alarm_contxt_t alarm_ctx;
    datac_handle_t srv_handle;

    /* Row references so we can repaint check marks without rebuilding. */
    lv_obj_t *day_check[7];      /* one per weekday (bit0..bit6) */
    lv_obj_t *preset_check[3];   /* Every Day, Weekdays, Weekends */
} app_alarm_edit_repeat_t;

static app_alarm_edit_repeat_t *p_app_alarm_edit_repeat = NULL;

static const char *DAY_LABELS[7] = {
    "Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday",
};

#define MASK_EVERY_DAY  0x7F
#define MASK_WEEKDAYS   0x3E /* Mon..Fri = bits 1..5 */
#define MASK_WEEKENDS   0x41 /* Sun + Sat = bits 0,6 */

/*********************
 *   STATIC HELPERS
 *********************/

static void refresh_checks(void)
{
    uint8_t d = p_app_alarm_edit_repeat->alarm_ctx.days;
    for (int i = 0; i < 7; i++)
    {
        if ((d >> i) & 1)
            lv_obj_clear_flag(p_app_alarm_edit_repeat->day_check[i],
                              LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(p_app_alarm_edit_repeat->day_check[i],
                            LV_OBJ_FLAG_HIDDEN);
    }
    static const uint8_t presets[3] = {MASK_EVERY_DAY, MASK_WEEKDAYS,
                                       MASK_WEEKENDS};
    for (int i = 0; i < 3; i++)
    {
        if (d == presets[i])
            lv_obj_clear_flag(p_app_alarm_edit_repeat->preset_check[i],
                              LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(p_app_alarm_edit_repeat->preset_check[i],
                            LV_OBJ_FLAG_HIDDEN);
    }
}

/*********************
 *   EVENT CALLBACKS
 *********************/

/* User data: 0..6 = weekday bit; 100..102 = preset (every/weekday/weekend). */
static void row_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    int32_t tag = (int32_t)(uintptr_t)lv_event_get_user_data(e);
    uint8_t *days = &p_app_alarm_edit_repeat->alarm_ctx.days;

    if (tag < 7)
    {
        *days ^= (uint8_t)(1 << tag);
    }
    else
    {
        static const uint8_t presets[3] = {MASK_EVERY_DAY, MASK_WEEKDAYS,
                                           MASK_WEEKENDS};
        uint8_t mask = presets[tag - 100];
        /* Tap an active preset to clear; tap an inactive one to set. */
        *days = (*days == mask) ? ALARM_REPEAT_ONE_SHOT : mask;
    }
    refresh_checks();
}

/*********************
 *   UI BUILDERS
 *********************/

static lv_obj_t *create_row(lv_obj_t *parent, const char *text, int32_t tag,
                            bool is_first_in_section)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), 56);
    lv_obj_set_style_bg_color(row, ALARM_COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
    /* Inside-section rows have flat top edge so consecutive rows look
       like a single rounded card. */
    lv_obj_set_style_radius(row, is_first_in_section ? ALARM_CARD_RADIUS : 0,
                            0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_left(row, 20, 0);
    lv_obj_set_style_pad_right(row, 16, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(row, row_event_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)tag);

    lv_obj_t *lbl = lv_label_create(row);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_font(
        lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);

    /* Check mark stays as the row's right-side accessory; hidden by default,
       refresh_checks() toggles visibility based on alarm_ctx.days. */
    lv_obj_t *check = lv_label_create(row);
    lv_label_set_text(check, LV_SYMBOL_OK);
    lv_obj_set_style_text_color(check, ALARM_COLOR_ACCENT, 0);
    lv_obj_set_style_text_font(
        check, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align(check, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_flag(check, LV_OBJ_FLAG_HIDDEN);

    if (tag < 7)
        p_app_alarm_edit_repeat->day_check[tag] = check;
    else
        p_app_alarm_edit_repeat->preset_check[tag - 100] = check;

    return row;
}

static void build_layout(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, ALARM_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* Title at the top safe area. */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Repeat");
    lv_obj_set_style_text_font(
        title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(title, ALARM_COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

    /* Scrollable list with two visually-grouped sections. */
    lv_obj_t *root = lv_obj_create(scr);
    lv_obj_set_size(root, ALARM_LIST_W, LV_VER_RES_MAX - 80);
    lv_obj_align(root, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(root, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(root, 1, 0);
    lv_obj_set_scrollbar_mode(root, LV_SCROLLBAR_MODE_OFF);

    /* Preset section. */
    create_row(root, "Every Day", 100, true);
    create_row(root, "Weekdays",  101, false);
    create_row(root, "Weekends",  102, false);

    /* 14-px separator visually splits the two sections. */
    lv_obj_t *gap = lv_obj_create(root);
    lv_obj_set_size(gap, LV_PCT(100), 14);
    lv_obj_set_style_bg_opa(gap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(gap, 0, 0);
    lv_obj_clear_flag(gap, LV_OBJ_FLAG_SCROLLABLE);

    /* Per-day section. */
    for (int i = 0; i < 7; i++)
        create_row(root, DAY_LABELS[i], i, i == 0);

    refresh_checks();
}

/*********************
 *   DATA SYNC
 *********************/

static int srv_msg_handler(data_callback_arg_t *arg)
{
    if (!p_app_alarm_edit_repeat && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
        return 0;

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (p_app_alarm_edit_repeat && rsp->result >= 0
            && p_app_alarm_edit_repeat->alarm_idx >= 0)
        {
            data_msg_t msg;
            alarm_msg_t *p = (alarm_msg_t *)data_service_init_msg(
                &msg, ALARMMGR_MSG_GET_ALARM_REQ, sizeof(alarm_msg_t));
            p->idx = p_app_alarm_edit_repeat->alarm_idx;
            datac_send_msg(p_app_alarm_edit_repeat->srv_handle, &msg);
        }
        break;
    }

    case ALARMMGR_MSG_GET_ALARM_RSP:
    {
        alarm_msg_t *data = (alarm_msg_t *)arg->data;
        if (data)
        {
            RT_ASSERT(p_app_alarm_edit_repeat->alarm_idx == data->idx);
            memcpy(&p_app_alarm_edit_repeat->alarm_ctx, &data->ctx,
                   sizeof(alarm_contxt_t));
            refresh_checks();
        }
        break;
    }

    case ALARMMGR_MSG_EDIT_ALARM_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;
        LOG_D("alarm edit err: %d", rsp->result);
        if (rsp->result == 0)
            watch_alarm_local_sync(1 /* EDIT */,
                                   p_app_alarm_edit_repeat->alarm_idx,
                                   &p_app_alarm_edit_repeat->alarm_ctx);
        gui_app_goback();
        break;
    }

    default: break;
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
    if (DATA_CLIENT_INVALID_HANDLE == p_app_alarm_edit_repeat->srv_handle)
    {
        p_app_alarm_edit_repeat->srv_handle = datac_open();
        RT_ASSERT(DATA_CLIENT_INVALID_HANDLE !=
                  p_app_alarm_edit_repeat->srv_handle);
    }
    ui_datac_subscribe(p_app_alarm_edit_repeat->srv_handle, "alarmmgr",
                       srv_msg_handler, 0);
}

static void on_pause(void)
{
    /* Persist the user's selection by sending an EDIT_ALARM_REQ before we
       tear down the page. The original implementation did this here too. */
    data_msg_t msg;
    alarm_msg_t *p = (alarm_msg_t *)data_service_init_msg(
        &msg, ALARMMGR_MSG_EDIT_ALARM_REQ, sizeof(alarm_msg_t));
    p->idx = p_app_alarm_edit_repeat->alarm_idx;
    memcpy(&p->ctx, &p_app_alarm_edit_repeat->alarm_ctx, sizeof(alarm_contxt_t));
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
    case GUI_APP_MSG_ONSTART:  on_start();  break;
    case GUI_APP_MSG_ONRESUME: on_resume(); break;
    case GUI_APP_MSG_ONPAUSE:  on_pause();  break;
    case GUI_APP_MSG_ONSTOP:   on_stop();   break;
    default: break;
    }
}

void app_alarm_edit_repeat(uint32_t alarm_idx)
{
    rt_kprintf("app_alarm_edit_repeat[%d] entry\r\n", alarm_idx);

    RT_ASSERT(NULL == p_app_alarm_edit_repeat);
    p_app_alarm_edit_repeat = (app_alarm_edit_repeat_t *)lv_mem_alloc(
        sizeof(app_alarm_edit_repeat_t));
    memset(p_app_alarm_edit_repeat, 0, sizeof(app_alarm_edit_repeat_t));
    p_app_alarm_edit_repeat->srv_handle = DATA_CLIENT_INVALID_HANDLE;
    p_app_alarm_edit_repeat->alarm_idx = alarm_idx;

    gui_app_create_page(SUBPAGE_NAME, msg_handler);
}
