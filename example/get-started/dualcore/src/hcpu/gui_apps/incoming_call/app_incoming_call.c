/**
 ******************************************************************************
 * @file   app_incoming_call.c
 * @brief  Incoming call screen. Activated when a calling notification arrives,
 *         shows caller title, app icon and accept/hangup buttons.
 *
 *         Answering is tried in three escalating steps, first that succeeds wins:
 *           1. HFP AT ATA / CHUP  -- only works when the watch is bonded to the
 *              phone as a Classic-BT hands-free device (call audio on the watch).
 *           2. iOS ANCS perform-action -- the BLE-only iPhone case.
 *           3. KEY_INCOMMING_CALL_ACCEPT / _REFUSE over BWPS -- the phone fires
 *              the ringing notification's own answer/decline action. This is the
 *              Android path: without an HFP bond steps 1+2 both no-op there.
 *
 *         Steps 1+2 are local and known immediately. Step 3 is a round trip, so
 *         the screen STAYS UP showing "connecting" until the phone reports back
 *         with KEY_CALL_ACTION_RESULT (or the wait times out) -- it must not
 *         close on a call that is still ringing, which is exactly what the old
 *         unconditional gui_app_exit() did whenever HFP was not bonded.
 *
 *         The app also closes when the phone dismisses the originating
 *         notification.
 ******************************************************************************
 */

#include <rtthread.h>
#include <string.h>
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "communicate_protocol.h"
#include "bloc_notification.h"
#include "ui_helper.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"   /* motor_pattern_stop */
#include "app_incoming_call.h"

/* Forward declare HFP HF wrappers — avoid including bts2_app_hfp_hf.h
 * which pulls in BT internal types (U8, BOOL, bts2_app_stru, ...). */
extern int bt_hfp_hf_answer_call_send(void);
extern int bt_hfp_hf_hangup_call_send(void);

/* iOS ANCS perform-action helper, implemented in app_message_database.c.
 * Returns 0 on success, negative on failure (e.g. ANCS not connected). */
extern int ancs_perform_call_action(uint32_t noti_uid, int positive);

/* Ask the phone to answer / reject on our behalf (BWPS notify, payload = the
 * forwarded notification id). Declared in communicate_task.h, which is not on
 * this app's CPPPATH. */
extern bool commu_send_call_accept(const char *id);
extern bool commu_send_call_refuse(const char *id);
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
    #include "ui_img_helper.h"
#endif

#define DBG_TAG "app.incoming_call"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_INCOMING_CALL

LV_IMG_DECLARE(icon_x);
LV_IMG_DECLARE(phone_call);

/* How often the pending-state timer polls the phone verdict, and how long it
 * waits before giving up. 6 s covers a BLE round trip plus the phone actually
 * firing the source app's PendingIntent, without leaving the user staring at a
 * screen that will never resolve. */
#define CALL_PENDING_TICK_MS     200
#define CALL_PENDING_TIMEOUT_MS  6000
#define CALL_PENDING_MAX_TICKS   (CALL_PENDING_TIMEOUT_MS / CALL_PENDING_TICK_MS)

typedef enum
{
    CALL_UI_IDLE = 0,     /* buttons live */
    CALL_UI_ACCEPTING,    /* accept sent to phone, awaiting its verdict */
    CALL_UI_REJECTING,    /* refuse sent to phone, awaiting its verdict */
} call_ui_state_t;

typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *title_label;
    lv_obj_t *status_label;
    lv_obj_t *accept_btn;
    lv_obj_t *hangup_btn;
    lv_timer_t *pending_timer;
    uint16_t pending_ticks;
} app_incoming_call_t;

static app_incoming_call_t *p_app = NULL;
static call_ui_state_t s_ui_state = CALL_UI_IDLE;

/* Phone verdict for the in-flight action: -1 = not answered yet, 0 = the phone
 * could not act, 1 = it did. WRITTEN FROM THE BLE RX THREAD by
 * incoming_call_on_action_result and read by the LVGL timer -- the flag is the
 * whole handoff, no LVGL call ever happens on the BLE thread. */
static volatile int8_t s_phone_result = -1;
static char s_caller_title[NOTIFICATION_TITLE_LEN] = {0};
static char s_caller_id[NOTIFICATION_ID_LEN] = {0};
static uint8_t s_caller_type = Notify_others;
static bool s_is_active = false;
static uint32_t s_ancs_noti_uid = 0; /* 0 = not an iOS ANCS call */

/* Buttons are greyed while an action is in flight so a second tap cannot queue
 * a duplicate accept behind the one already awaiting the phone. */
static void set_buttons_enabled(bool enabled)
{
    if (!p_app) return;
    if (p_app->accept_btn)
    {
        if (enabled) lv_obj_clear_state(p_app->accept_btn, LV_STATE_DISABLED);
        else         lv_obj_add_state(p_app->accept_btn, LV_STATE_DISABLED);
    }
    if (p_app->hangup_btn)
    {
        if (enabled) lv_obj_clear_state(p_app->hangup_btn, LV_STATE_DISABLED);
        else         lv_obj_add_state(p_app->hangup_btn, LV_STATE_DISABLED);
    }
}

static void set_status_text(const char *text)
{
    if (!p_app || !p_app->status_label) return;
    lv_label_set_text(p_app->status_label, text ? text : "");
}

/* Back to an interactive screen, with the reason the phone could not act. The
 * screen deliberately stays open: the call is still ringing, and telling the
 * user to reach for the phone beats silently closing on them. */
static void end_pending(bool accepting, bool failed)
{
    if (!p_app) return;
    if (p_app->pending_timer) lv_timer_pause(p_app->pending_timer);
    p_app->pending_ticks = 0;
    s_ui_state = CALL_UI_IDLE;
    set_buttons_enabled(true);
    if (failed)
    {
        set_status_text(accepting
                        ? LV_EXT_STR_GET_BY_KEY(call_answer_failed, "Could not answer - use your phone")
                        : LV_EXT_STR_GET_BY_KEY(call_hangup_failed, "Could not hang up - use your phone"));
    }
    else
    {
        set_status_text("");
    }
}

static void pending_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    /* gui_app_exit is not necessarily synchronous, so this can still fire once
       after the app started tearing down. */
    if (!p_app || s_ui_state == CALL_UI_IDLE) return;

    bool accepting = (s_ui_state == CALL_UI_ACCEPTING);
    int8_t result = s_phone_result;

    if (result == 1)
    {
        LOG_W("[call] phone confirmed accepting=%d", (int)accepting);
        end_pending(accepting, false);
        gui_app_exit(APP_ID_INCOMING_CALL);
        return;
    }

    p_app->pending_ticks++;
    if (result == 0 || p_app->pending_ticks >= CALL_PENDING_MAX_TICKS)
    {
        /* Two lines, not one: LOG_* silently drops the whole statement past two
           varargs in this firmware, so a wider format string would make this
           failure invisible -- exactly when it is most needed. */
        LOG_W("[call] action failed accepting=%d", (int)accepting);
        LOG_W("[call] verdict=%d ticks=%d", (int)result, (int)p_app->pending_ticks);
        end_pending(accepting, true);
    }
}

static void begin_pending(bool accepting)
{
    if (!p_app) return;
    s_ui_state = accepting ? CALL_UI_ACCEPTING : CALL_UI_REJECTING;
    p_app->pending_ticks = 0;
    set_buttons_enabled(false);
    set_status_text(LV_EXT_STR_GET_BY_KEY(call_connecting, "Connecting..."));
    if (!p_app->pending_timer)
    {
        p_app->pending_timer = lv_timer_create(pending_timer_cb, CALL_PENDING_TICK_MS, NULL);
    }
    else
    {
        lv_timer_resume(p_app->pending_timer);
    }
}

/* Shared accept / hangup path -- see the file header for why the order is
 * HFP, then ANCS, then the phone relay. */
static void start_call_action(bool accepting)
{
    /* Debounce: an action is already in flight. */
    if (!p_app || s_ui_state != CALL_UI_IDLE) return;

    /* The ring pattern is 3 s of buzzing; the user has answered, stop it. */
    motor_pattern_stop();

    int ret = accepting ? bt_hfp_hf_answer_call_send() : bt_hfp_hf_hangup_call_send();
    LOG_W("[call] hfp accepting=%d ret=%d", (int)accepting, ret);
    bool done = (ret == 0);

    if (!done && s_ancs_noti_uid != 0)
    {
        int r = ancs_perform_call_action(s_ancs_noti_uid, accepting ? 1 : 0);
        LOG_W("[call] ancs positive=%d ret=%d", (int)accepting, r);
        done = (r == 0);
    }

    if (done)
    {
        /* Handled on the watch itself -- nothing to wait for. */
        gui_app_exit(APP_ID_INCOMING_CALL);
        return;
    }

    /* Android / no HFP bond: hand it to the phone and wait for the verdict. */
    s_phone_result = -1;
    bool sent = accepting ? commu_send_call_accept(s_caller_id)
                          : commu_send_call_refuse(s_caller_id);
    LOG_W("[call] relayed sent=%d id=%s", (int)sent, s_caller_id);
    if (!sent)
    {
        /* Link down -- no point waiting out the full timeout. */
        end_pending(accepting, true);
        return;
    }
    begin_pending(accepting);
}

static void accept_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }
    start_call_action(true);
}

static void hangup_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }
    start_call_action(false);
}

static lv_obj_t *create_incoming_call_screen(lv_obj_t *scr)
{
    lv_obj_t *bg = lv_obj_create(scr);
    lv_obj_set_size(bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x111111), 0);
    lv_obj_set_style_pad_all(bg, 0, 0);
    lv_obj_set_style_border_width(bg, 0, 0);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);

    /* Caller name at top */
    lv_obj_t *title = lv_label_create(bg);
    lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
    lv_obj_set_width(title, LV_HOR_RES - 40);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_label_set_text(title, s_caller_title[0] ? s_caller_title : "Incoming call");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 60);
    p_app->title_label = title;

    /* Status line: empty while idle, "connecting" while the phone is acting,
       and the reason on failure. WRAP (not DOT) so the failure sentence stays
       readable on the round screen instead of being clipped to an ellipsis. */
    lv_obj_t *status = lv_label_create(bg);
    lv_label_set_long_mode(status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(status, LV_HOR_RES - 60);
    lv_obj_set_style_text_align(status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(status, lv_color_hex(0xBBBBBB), 0);
    lv_obj_set_style_text_font(status, LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_label_set_text(status, "");
    lv_obj_align_to(status, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);
    p_app->status_label = status;

    /* App icon from notification type (same mapping as message list) */
    lv_obj_t *icon = lv_img_create(bg);
    uint8_t type = s_caller_type;
    if (type >= NOTIFICATION_APP_QUANTITY)
    {
        type = Notify_others;
    }
    lv_img_set_src(icon, icon_list[type]);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -20);

    /* Accept button */
    lv_obj_t *accept_btn = lv_btn_create(bg);
    lv_obj_set_size(accept_btn, 100, 100);
    lv_obj_set_style_radius(accept_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(accept_btn, lv_color_hex(0x4CD964), 0);
    lv_obj_align(accept_btn, LV_ALIGN_BOTTOM_LEFT, 40, -70);
    lv_obj_add_event_cb(accept_btn, accept_btn_event_cb, LV_EVENT_CLICKED, NULL);
    p_app->accept_btn = accept_btn;

    lv_obj_t *accept_icon = lv_img_create(accept_btn);
    lv_img_set_src(accept_icon, &phone_call);
    lv_obj_center(accept_icon);

    /* Hangup button */
    lv_obj_t *hangup_btn = lv_btn_create(bg);
    lv_obj_set_size(hangup_btn, 100, 100);
    lv_obj_set_style_radius(hangup_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hangup_btn, lv_color_hex(0xFF3B30), 0);
    lv_obj_align(hangup_btn, LV_ALIGN_BOTTOM_RIGHT, -40, -70);
    lv_obj_add_event_cb(hangup_btn, hangup_btn_event_cb, LV_EVENT_CLICKED, NULL);
    p_app->hangup_btn = hangup_btn;

    lv_obj_t *hangup_icon = lv_img_create(hangup_btn);
    lv_img_set_src(hangup_icon, &icon_x);
    lv_obj_center(hangup_icon);

    return bg;
}

static lv_obj_t *on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app);
    p_app = (app_incoming_call_t *)lv_mem_alloc(sizeof(app_incoming_call_t));
    if (!p_app)
    {
        LOG_E("alloc failed");
        return NULL;
    }
    memset(p_app, 0, sizeof(app_incoming_call_t));
    /* Clear any verdict left over from a previous call before the screen can
       poll it -- a stale 1 would close the new call instantly. */
    s_ui_state = CALL_UI_IDLE;
    s_phone_result = -1;
    p_app->main_window = create_incoming_call_screen(scr);
    s_is_active = true;
    return p_app->main_window;
}

static void on_stop(void)
{
    s_is_active = false;
    s_caller_id[0] = '\0';
    s_caller_title[0] = '\0';
    s_ancs_noti_uid = 0;
    s_ui_state = CALL_UI_IDLE;
    s_phone_result = -1;
    if (p_app)
    {
        /* Sole owner of the timer -- pending_timer_cb only ever pauses it, so
           there is no path where this deletes a timer twice. */
        if (p_app->pending_timer)
        {
            lv_timer_del(p_app->pending_timer);
            p_app->pending_timer = NULL;
        }
        if (p_app->main_window)
        {
            lv_obj_del(p_app->main_window);
            p_app->main_window = NULL;
        }
        lv_mem_free(p_app);
        p_app = NULL;
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start(lv_scr_act());
        break;
    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

void incoming_call_set_caller(const char *title, const char *id, uint8_t type)
{
    LOG_I("incoming_call_set_caller: title:%s, id:%s, type:%d", title, id, type);
    if (title)
    {
        strncpy(s_caller_title, title, sizeof(s_caller_title) - 1);
        s_caller_title[sizeof(s_caller_title) - 1] = '\0';
    }
    else
    {
        s_caller_title[0] = '\0';
    }
    if (id)
    {
        strncpy(s_caller_id, id, sizeof(s_caller_id) - 1);
        s_caller_id[sizeof(s_caller_id) - 1] = '\0';
    }
    else
    {
        s_caller_id[0] = '\0';
    }
    s_caller_type = type;
}

void incoming_call_close_if_active(const char *id)
{
    if (!s_is_active || !id)
    {
        return;
    }
    if (strcmp(s_caller_id, id) == 0)
    {
        LOG_I("incoming_call: phone dismissed, closing app");
        gui_app_exit(APP_ID_INCOMING_CALL);
    }
}

void incoming_call_set_ancs_uid(uint32_t noti_uid)
{
    s_ancs_noti_uid = noti_uid;
}

void incoming_call_on_action_result(bool ok)
{
    /* BLE RX THREAD. Latch only: pending_timer_cb picks this up on the LVGL
       thread and does every widget touch there. Deliberately unconditional --
       a verdict that lands microseconds before on_stop clears the flag is
       harmless, whereas gating on s_is_active here would open a race window. */
    s_phone_result = ok ? 1 : 0;
}

void incoming_call_force_close(void)
{
    if (!s_is_active)
    {
        return;
    }
    LOG_I("incoming_call: force close");
    gui_app_exit(APP_ID_INCOMING_CALL);
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_INCOMING_CALL, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(message), IMG_LOGO, APP_ID_INCOMING_CALL,
                   app_main, 1);

#endif
