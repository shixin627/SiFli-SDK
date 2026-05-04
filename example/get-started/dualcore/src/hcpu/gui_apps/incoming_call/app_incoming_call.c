/**
 ******************************************************************************
 * @file   app_incoming_call.c
 * @brief  Incoming call screen. Activated when a calling notification arrives,
 *         shows caller title, app icon and accept/hangup buttons. Accept sends
 *         KEY_INCOMMING_CALL_ACCEPT to phone, hangup sends KEY_INCOMMING_CALL_REFUSE.
 *         The app closes when the phone dismisses the originating notification.
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
#include "app_incoming_call.h"

/* Forward declare HFP HF wrappers — avoid including bts2_app_hfp_hf.h
 * which pulls in BT internal types (U8, BOOL, bts2_app_stru, ...). */
extern int bt_hfp_hf_answer_call_send(void);
extern int bt_hfp_hf_hangup_call_send(void);

/* iOS ANCS perform-action helper, implemented in app_message_database.c.
 * Returns 0 on success, negative on failure (e.g. ANCS not connected). */
extern int ancs_perform_call_action(uint32_t noti_uid, int positive);
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

typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *title_label;
} app_incoming_call_t;

static app_incoming_call_t *p_app = NULL;
static char s_caller_title[NOTIFICATION_TITLE_LEN] = {0};
static char s_caller_id[NOTIFICATION_ID_LEN] = {0};
static uint8_t s_caller_type = Notify_others;
static bool s_is_active = false;
static uint32_t s_ancs_noti_uid = 0; /* 0 = not an iOS ANCS call */

static void accept_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }
    LOG_I("incoming_call: accept -> HFP ATA");
    int ret = bt_hfp_hf_answer_call_send();
    LOG_I("hfp answer ret=%d", ret);
    if (s_ancs_noti_uid != 0)
    {
        int r = ancs_perform_call_action(s_ancs_noti_uid, 1);
        LOG_I("ancs positive action uid=%u ret=%d", s_ancs_noti_uid, r);
    }
    gui_app_exit(APP_ID_INCOMING_CALL);
}

static void hangup_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }
    LOG_I("incoming_call: hangup -> HFP CHUP");
    int ret = bt_hfp_hf_hangup_call_send();
    LOG_I("hfp hangup ret=%d", ret);
    if (s_ancs_noti_uid != 0)
    {
        int r = ancs_perform_call_action(s_ancs_noti_uid, 0);
        LOG_I("ancs negative action uid=%u ret=%d", s_ancs_noti_uid, r);
    }
    gui_app_exit(APP_ID_INCOMING_CALL);
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
    if (p_app)
    {
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
                   app_main);

#endif
