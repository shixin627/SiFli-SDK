/*********************o_button
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "ui_helper.h"
#include "ui_img_helper.h"
#include "bloc_notification.h"
#include "ui_handler.h"
#include "custom_trans_anim.h"
#include "app_message.h"
#include "bloc_v2t.h"
#include "bloc_motor.h"
#include "app_speech.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
    #include "watch_system_core_task.h"
#endif

#define DBG_TAG "app.message"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_MESSAGE

typedef struct
{
    lv_ex_data_t *title;
    lv_ex_data_t *content;
    lv_ex_data_t *app_name;
    void *title_handle;
    void *content_handle;
    void *app_name_handle;
    bool *can_reply;
} app_message_ctx_t;

typedef struct
{
    lv_obj_t *main_window;
} app_message_t;

LV_IMG_DECLARE(icon_reply);
LV_IMG_DECLARE(message_widget_bg);
LV_IMG_DECLARE(gaus_clock1_bg);
LV_IMG_DECLARE(voice_group);

static app_message_ctx_t *p_app_message_ctx = NULL;
static app_message_t *p_app_message = NULL;

void cont_event_callback(lv_event_t *event)
{
}

static lv_obj_t *lbl_content;
static lv_obj_t *lbl_title;
static lv_obj_t *lbl_app_name;
static notification_t notification;

static bool open_from_message_list = false;
void app_message_set_open_from_message_list(bool open)
{
    open_from_message_list = open;
}

static void navigate_to_reply(void)
{
    strcpy(notification.title, p_app_message_ctx->title->value.s);
    notify_provider.navigate_to_reply(&notification);
    if (!open_from_message_list)
    {
        gui_app_exit(APP_ID_MESSAGE);
    }
}
static void reply_btn_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        LOG_D("reply_btn_event_callback");
        navigate_to_reply();
    }
}

static void ingore_btn_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        gui_app_self_exit();
    }
}

/* Voice reply UI elements */
static lv_obj_t *voice_text_label = NULL;
static lv_obj_t *voice_send_btn = NULL;
static lv_obj_t *voice_send_icon = NULL;
static lv_obj_t *voice_reply_window = NULL;
static void *voice_text_bind_handle = NULL;
static bool voice_reply_active = false;

static void voice_text_label_set_text(lv_obj_t *label, const char *text)
{
    lv_label_set_text(label, text);

    /* text_container is the direct parent of label */
    lv_obj_t *text_container = lv_obj_get_parent(label);
    if (!text_container || !lv_obj_is_valid(text_container))
        return;

    lv_obj_update_layout(text_container);
    lv_coord_t text_h = lv_obj_get_height(label);
    lv_coord_t container_h = lv_obj_get_height(text_container);

    /*
     * Phase 1: text fits inside container — scroll p_window up by text growth
     *          so the current bottom line stays at the same screen position.
     * Phase 2: text overflows container — p_window stays at max scroll,
     *          container scrolls internally to show the latest line.
     */
    if (voice_reply_window && lv_obj_is_valid(voice_reply_window))
    {
        lv_coord_t offset = 52; /* Increase to push text lower on screen */
        if (text_h > container_h)
        {
            offset -= 20;

        }
        lv_coord_t raw_scroll = (text_h < container_h) ? text_h : container_h;
        lv_coord_t extra_scroll = (raw_scroll > offset) ? (raw_scroll - offset) : 0;
        lv_obj_scroll_to_y(voice_reply_window, extra_scroll, LV_ANIM_ON);
    }

    if (text_h > container_h)
    {
        lv_coord_t scroll_pos = text_h - container_h;
        lv_obj_scroll_to_y(text_container, scroll_pos, LV_ANIM_ON);
    }

    if (voice_send_icon && lv_obj_is_valid(voice_send_icon))
    {
        if (text == NULL || text[0] == '\0')
        {
            lv_obj_add_flag(voice_send_icon, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_clear_flag(voice_send_icon, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* Voice reply: send transcribed text as notification reply */
static void message_voice_send(void)
{
    #ifdef BSP_USING_BLOC_V2T
    if (isTextEmpty() == false)
    {
        stop_voice_recognition(V2T_INTENT_NOTHING);
        const char *text = get_combined_voice2text();
        LOG_D("message_voice_send id:%s text:%s", notification.id, text);
        strcpy(replying_notification_id, notification.id);
        handle_user_speech_intent(V2T_INTENT_REMOTE_INPUT, (char *)text);
        count_speech_coding();
        clearVoice2Text();
        /* Ensure notification is removed from list after reply */
        extern void remove_notification_by_id(const char *id);
        remove_notification_by_id(notification.id);
        gui_app_self_exit();
    }
    #endif
}

static void message_voice_clear(void)
{
    #ifdef BSP_USING_BLOC_V2T
    if (isTextEmpty() == false)
    {
        count_speech_coding();
        clearVoice2Text();
        app_speech_set_content((const uint8_t *)"");
    }
    else
    {
        gui_app_self_exit();
    }
    #endif
}

static void message_send_btn_cb(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        message_voice_send();
    }
}

static void message_cancel_btn_cb(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        message_voice_clear();
    }
}

static void message_tap_send(uint8_t gesture)
{
    if (gesture == 1)
    {
        message_voice_send();
    }
}

    #ifdef BSP_USING_UI_HANDLER
static void mic_opa_anim_cb(void *obj, int32_t value)
{
    lv_obj_set_style_bg_opa((lv_obj_t *)obj, value, 0);
}

static void handle_message_mic_status(bool speaking)
{
    LOG_D("handle_message_mic_status: %d", speaking);
    if (lv_obj_is_valid(voice_send_btn))
    {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, voice_send_btn);
        lv_anim_set_values(&a, lv_obj_get_style_bg_opa(voice_send_btn, 0),
                           speaking ? LV_OPA_30 : LV_OPA_10);
        lv_anim_set_exec_cb(&a, mic_opa_anim_cb);
        lv_anim_set_time(&a, 200);
        lv_anim_start(&a);
    }
}
    #endif

static lv_obj_t *reply_btn = NULL;
static lv_obj_t *icon_btn_send = NULL;
static rt_timer_t off_selected_timer = NULL;
static bool selected_message = false;
static void switch_selected_message(bool on)
{
    LOG_D("switch_selected_timer_cb : %d", on);
    if (on)
    {
        lv_obj_set_style_shadow_opa(reply_btn, LV_OPA_100, 0);
    }
    else
    {
        lv_obj_set_style_shadow_opa(reply_btn, LV_OPA_0, 0);
    }
}

static void switch_selected_timer_cb(void *param)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_SWITCH_SELECTED;
    msg.data.switch_selected = *(bool *)param;
    lvgl_send_msg(msg);
    selected_message = false;
}

static void start_selected_timer(void)
{
    selected_message = true;
    switch_selected_timer_cb(&selected_message);
    if (off_selected_timer == NULL)
    {
        off_selected_timer =
            rt_timer_create("off_selected_timer", switch_selected_timer_cb,
                            &selected_message, 500, RT_TIMER_FLAG_ONE_SHOT);
    }
    rt_timer_start(off_selected_timer);
}

static bool messagr_can_reply = false;
static bool reply_lock = true;

static lv_obj_t *icon_standby = NULL;

extern char *replace_nbsp(const char *str);
static void label_set_text(lv_obj_t *obj, const char *text)
{
    char *out_text = replace_nbsp(text);
    lv_label_set_text(obj, out_text);
}

static lv_obj_t *app_icon_builder(lv_obj_t *parent, uint8_t app_index)
{
    lv_obj_t *icon = lv_img_create(parent);
    lv_img_set_src(icon, icon_list[app_index]);
    return icon;
}

static lv_obj_t *message_title_builder(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 270, 40);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 60);
    char *title = replace_nbsp(p_app_message_ctx->title->value.s);
    lv_label_set_text(label, title);
    lv_mem_free(title);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(-1)),
                               0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_update_layout(label);
    return label;
}

static lv_obj_t *message_content_builder(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    // lv_obj_set_size(label, 400, 165);
    lv_obj_set_width(label, 400);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    char *content = replace_nbsp(p_app_message_ctx->content->value.s);
    lv_label_set_text(label, content);
    lv_mem_free(content);
    lv_color_t color = lv_color_white();
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)),
                               0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_opa(label, LV_OPA_60, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_update_layout(label);
    return label;
}

lv_obj_t *app_message_init(lv_obj_t *parent)
{
    lv_obj_t *bg_img = lv_img_create(parent);
    lv_img_set_src(bg_img, &gaus_clock1_bg);
    lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_zoom(bg_img, 512);

    lv_obj_t *p_window = common_black_bg(parent);
    lv_obj_set_style_bg_opa(p_window, LV_OPA_0, 0);
    lv_obj_set_scrollbar_mode(p_window, LV_SCROLLBAR_MODE_OFF);
    voice_reply_window = p_window;
    lv_obj_t *icon_bg = lv_obj_create(p_window);
    lv_obj_set_size(icon_bg, 67, 67);
    lv_obj_align(icon_bg, LV_ALIGN_TOP_MID, 0, -5);
    lv_obj_set_style_bg_opa(icon_bg, LV_OPA_0, 0);
    lv_obj_clear_flag(icon_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(icon_bg, LV_OBJ_FLAG_SCROLLABLE);
    /* App icon + title */
    icon_standby = app_icon_builder(icon_bg, app_message_get_app_index());
    lv_img_set_zoom(icon_standby, 166);
    lv_obj_align(icon_standby, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *message_title = message_title_builder(p_window);

    /* Message content area */
    lv_obj_t *message_widget = lv_obj_create(p_window);
    lv_obj_set_size(message_widget, 400, 160);
    // lv_obj_set_style_radius(message_widget, 30, 0);
    lv_obj_set_style_bg_color(message_widget, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(message_widget, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(message_widget, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(message_widget, message_title, LV_ALIGN_OUT_BOTTOM_MID, 0,
                    0);
    lv_obj_t *message_lable = message_content_builder(message_widget);
    if (lv_obj_get_height(message_lable) + 5 < 160)
    {
        lv_obj_set_height(message_widget, lv_obj_get_height(message_lable) + 5);
    }
    else
    {
        lv_obj_set_height(message_widget, 165);
    }

    if (messagr_can_reply)
    {
        /* Voice reply UI: text container + speech indicator + send/cancel
         * buttons */
        lv_obj_t *voice_container = lv_obj_create(p_window);
        lv_obj_set_size(voice_container, 444, 254);
        lv_obj_set_style_radius(voice_container, 80, 0);
        lv_obj_set_style_bg_color(voice_container, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(voice_container, LV_OPA_TRANSP, 0);
        // lv_obj_set_style_border_width(voice_container, 1, 0);
        // lv_obj_set_style_border_color(voice_container, lv_color_hex(0xFFFFFF),
        //                               0);
        // lv_obj_set_style_border_opa(voice_container, LV_OPA_TRANSP, 0);
        lv_obj_align_to(voice_container, message_widget,
                        LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
        lv_obj_set_scrollbar_mode(voice_container, LV_SCROLLBAR_MODE_OFF);

        /* Background image on p_window (not inside voice_container) so it stays fixed when scrolling */
        lv_obj_t *voice_container_bg = lv_img_create(p_window);
        lv_obj_set_size(voice_container_bg, 442, 252);
        lv_img_set_src(voice_container_bg, &message_widget_bg);
        lv_obj_align_to(voice_container_bg, voice_container, LV_ALIGN_CENTER, 0, 0);

        /* Raise voice_container above bg so text is visible */
        lv_obj_move_foreground(voice_container);

        /* Transcription text label */
        lv_obj_t *voice_text_container = lv_obj_create(voice_container);
        lv_obj_set_size(voice_text_container, 444, 200);
        lv_obj_set_style_bg_opa(voice_text_container, LV_OPA_TRANSP, 0);
        lv_obj_align(voice_text_container, LV_ALIGN_CENTER, 0, 0);

        voice_text_label = lv_label_create(voice_text_container);
        lv_label_set_long_mode(voice_text_label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(voice_text_label, 380);
        lv_obj_align(voice_text_label, LV_ALIGN_TOP_MID, 0, 10);
        lv_obj_set_style_text_font(voice_text_label,
                                   LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_color(voice_text_label, lv_color_white(), 0);
        lv_label_set_text(voice_text_label, "");
        /* Bind to speech content data with custom setter that controls send icon */
        extern lv_ex_data_t *app_speech_get_content_data(void);
        lv_ex_data_t *speech_content = app_speech_get_content_data();
        if (speech_content)
        {
            lv_ex_binding_t binding;
            binding.target = voice_text_label;
            binding.arg_type = LV_EX_DATA_STRING;
            binding.setter = (void *)voice_text_label_set_text;
            voice_text_bind_handle = lv_ex_bind_data(speech_content, &binding);
        }

        voice_send_btn = lv_obj_create(parent);
        lv_obj_t *send_btn = voice_send_btn;
        lv_obj_set_size(send_btn, 62, 62);
        lv_obj_set_style_radius(send_btn, 31, 0);
        lv_obj_set_style_bg_color(send_btn, lv_color_hex(0x00AAFF), 0);
        lv_obj_set_style_bg_opa(send_btn, 10, 0);
        lv_obj_align(send_btn, LV_ALIGN_BOTTOM_MID, 0, 0);

        lv_obj_t *voice_btn_img = lv_img_create(send_btn);
        lv_img_set_src(voice_btn_img, &voice_group);
        lv_obj_align(voice_btn_img, LV_ALIGN_CENTER, 0, 0);
        
        // lv_obj_t *icon_btn_send =
        //     common_icon_button(parent, ICON_SAND, message_send_btn_cb);
        voice_send_icon = lv_img_create(send_btn);
        lv_img_set_src(voice_send_icon, ICON_SAND);
        lv_obj_align(voice_send_icon, LV_ALIGN_CENTER, 2, 2);
        lv_obj_add_event_cb(send_btn, message_send_btn_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_add_flag(voice_send_icon, LV_OBJ_FLAG_HIDDEN);

        lv_obj_t *footer_obj = lv_obj_create(p_window);
        lv_obj_set_size(footer_obj, 400, 106);
        lv_obj_align_to(footer_obj, voice_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_opa(footer_obj, LV_OPA_TRANSP, 0);
    }

    return p_window;
}

static void on_start(void)
{
    RT_ASSERT(NULL == p_app_message);
    p_app_message = (app_message_t *)lv_mem_alloc(sizeof(app_message_t));
    memset(p_app_message, 0, sizeof(app_message_t));
    /* Init voice recognition for inline reply */
    if (messagr_can_reply)
    {
        voice_reply_active = true;
    #ifdef BSP_USING_BLOC_V2T
        voice_provider.vad_init();
        clearVoice2Text();
        app_speech_set_content((const uint8_t *)"");
    #endif
    }
    else
    {
        voice_reply_active = false;
    }
    p_app_message->main_window = app_message_init(lv_scr_act());
    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
}

static void on_resume(void)
{
    #ifdef BSP_USING_UI_HANDLER
    if (voice_reply_active)
    {
        lvgl_msg_handler.handle_vad_status = handle_message_mic_status;
        lvgl_msg_handler.handle_tap_indicator = message_tap_send;
        lvgl_msg_handler.handle_back_event = message_voice_clear;
        #ifdef BSP_USING_BLOC_V2T
        start_voice_recognition(V2T_INTENT_REMOTE_INPUT);
        #endif
    }
    lvgl_msg_handler.handle_switch_selected = switch_selected_message;
    #endif
}

static void on_pause(void)
{
    #ifdef BSP_USING_UI_HANDLER
    if (voice_reply_active)
    {
        #ifdef BSP_USING_BLOC_V2T
        stop_voice_recognition(V2T_INTENT_NOTHING);
        #endif
        lvgl_msg_handler.handle_back_event = NULL;
        lvgl_msg_handler.handle_vad_status = NULL;
    }
    lvgl_msg_handler.handle_tap_event = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
    lvgl_msg_handler.handle_send_message = NULL;
    lvgl_msg_handler.handle_switch_selected = NULL;
    #endif
}

static void on_stop(void)
{
    LOG_D("message_on_stop");
    /* Clean up voice reply resources */
    if (voice_reply_active)
    {
    #ifdef BSP_USING_BLOC_V2T
        stop_voice_recognition(V2T_INTENT_NOTHING);
        /* Unbind our voice text label from speech content data */
        if (voice_text_bind_handle)
        {
            extern lv_ex_data_t *app_speech_get_content_data(void);
            lv_ex_data_t *speech_content = app_speech_get_content_data();
            if (speech_content)
            {
                lv_ex_unbind_data(speech_content, voice_text_bind_handle);
            }
            voice_text_bind_handle = NULL;
        }
    #endif
        voice_reply_active = false;
        voice_text_label = NULL;
        voice_send_btn = NULL;
        voice_send_icon = NULL;
        voice_reply_window = NULL;
    }
    if (p_app_message_ctx->content_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->content,
                          p_app_message_ctx->content_handle);
    }
    if (p_app_message_ctx->title_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->title,
                          p_app_message_ctx->title_handle);
    }
    if (p_app_message_ctx->app_name_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->app_name,
                          p_app_message_ctx->app_name_handle);
    }
    if (p_app_message)
    {
        if (p_app_message->main_window &&
            lv_obj_is_valid(p_app_message->main_window))
        {
            lv_obj_del(p_app_message->main_window);
            p_app_message->main_window = NULL;
        }
        lv_mem_free(p_app_message);
        p_app_message = NULL;
    }

    if (open_from_message_list)
    {
        open_from_message_list = false;
    }
}

static void handle_back_for_reply(void)
{
    if (isTextEmpty() == false)
    {
        clearVoice2Text();
    }
    else
    {
        gui_app_exit(APP_ID_MESSAGE);
    }
}

static void main_msg_handler(gui_app_msg_type_t msg, void *param)
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

static void new_msg_popup_event_callback(lv_event_t *event)
{
    rt_kprintf("new_msg_popup_event_callback %s",
               lv_event_to_name(event->code));
    if (LV_EVENT_READY == event->code)
    {
        // Entry message main page
        gui_app_create_page("main", main_msg_handler);
        gui_app_remove_page("popup");
    }
    else if (LV_EVENT_CANCEL == event->code)
    {
        gui_app_goback();
    }
}

static void popupmsg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        const char *name = intent_get_string(gui_app_get_intent(), "newfrom");

        lv_obj_t *pop_container = lv_lvsfpopup_create(lv_scr_act());

        lv_lvsfpopup_set_icon(pop_container, IMG_MAIL);
        lv_lvsfpopup_set_title(pop_container, "New msg from:");
        lv_lvsfpopup_set_content(pop_container, name);

        lv_lvsfpopup_set_confirm_btn_txt(pop_container, "View");
        lv_lvsfpopup_set_cancel_btn_txt(pop_container, "Ignore");
        lv_obj_align(pop_container, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_event_cb(pop_container, new_msg_popup_event_callback,
                            LV_EVENT_ALL, NULL);
    }
    break;

    case GUI_APP_MSG_ONRESUME:
        break;

    case GUI_APP_MSG_ONPAUSE:
        break;

    case GUI_APP_MSG_ONSTOP:
        break;
    default:
        break;
    }
}

static int app_message_main(intent_t i)
{
    const char *name = intent_get_string(i, "newfrom");
    LOG_D("app_message_main %s", name);
    if (name)
    {
        // Display new message popup page
        gui_app_create_page("popup", popupmsg_handler);
    }
    else
    {
        // Entry message main page
        const char *id = intent_get_string(i, "noti_id");
        if (id)
        {
            strncpy(notification.id, id, NOTIFICATION_ID_LEN);
        }
        LOG_D("temp_noti_id %s", notification.id);
        gui_app_create_page("main", main_msg_handler);
        const char *can_reply = intent_get_string(i, "noti_can_reply");
        if (can_reply)
        {
            messagr_can_reply = (strcmp(can_reply, "true") == 0);
        }
    }
    return 0;
}

void open_message_app(const char *notification_id)
{
    strncpy(notification.id, notification_id, NOTIFICATION_ID_LEN);
    LOG_D("temp_noti_id %s", notification.id);
    on_start();
    on_resume();
}

void close_message_app(void)
{
    on_pause();
    on_stop();
}

void app_message_data_init(void)
{
    if (!p_app_message_ctx)
    {
        p_app_message_ctx =
            (app_message_ctx_t *)rt_malloc(sizeof(app_message_ctx_t));
        memset(p_app_message_ctx, 0, sizeof(app_message_ctx_t));
    }

    p_app_message_ctx->title =
        lv_ex_data_create("msg.title", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->title);
    p_app_message_ctx->content =
        lv_ex_data_create("msg.content", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->content);
    p_app_message_ctx->app_name =
        lv_ex_data_create("msg.app", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->app_name);
}

void app_message_set_app_name(const uint8_t *s)
{
    if (p_app_message_ctx->app_name)
    {
    #if 0
        if ((0 == strncmp("\xE4\xBF\xA1\xE6", (char *)s, strlen("\xE4\xBF\xA1\xE6")))
                || (0 == strncmp("Message", (char *)s, strlen("Message"))))
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"SMS");
        }
        else if ((0 == strncmp("\xe7\x94\xb5\xe8", (char *)s, strlen("\xe7\x94\xb5\xe8")))
                 || (0 == strncmp("Phone", (char *)s, strlen("Phone"))))
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"Phone");
        }
        else
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"WeChat");
        }
    #else
        lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)s);
    #endif
    }
}

void app_message_set_title(const uint8_t *s)
{
    if (p_app_message_ctx->title)
    {
        lv_ex_data_set_value(p_app_message_ctx->title, (void *)s);
    }
}

void app_message_set_content(const uint8_t *s)
{
    if (p_app_message_ctx->content)
    {
        lv_ex_data_set_value(p_app_message_ctx->content, (void *)s);
    }
}

static uint8_t _app_index = 0;
void app_message_set_app_index(uint8_t index)
{
    _app_index = index;
}
uint8_t app_message_get_app_index(void)
{
    return _app_index;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(message), IMG_MAIL, APP_ID_MESSAGE,
                   app_message_main);
#endif
