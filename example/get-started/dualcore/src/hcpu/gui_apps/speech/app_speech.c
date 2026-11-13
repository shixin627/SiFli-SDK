/**
 ******************************************************************************
 * @file   app_speech.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*********************
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
#include "app_speech.h"
#include "bloc_v2t.h"
#include "bloc_setting.h"
#include "notification_reply.h"
#include "common_widget.h"
#include "voice_ai_logo.h"
#include "app_mainmenu.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif
#ifdef BSP_USING_BLOC_V2T
#include "bloc_v2t.h"
#include "bloc_setting.h"
#endif

#define DBG_TAG "app.speech"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_SPEECH

typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *lbl_content;
    app_speech_ripple_t app_speech_ripple[4];
} app_speech_t;

LV_IMG_DECLARE(icon_speech);
extern void app_speech_bind_title(lv_obj_t *target);
extern void app_speech_bind_content(lv_obj_t *target);
extern void unbind_app_speech_data(void);

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static void main_msg_handler(gui_app_msg_type_t msg, void *param);
static void reply_msg_handler(gui_app_msg_type_t msg, void *param);
static app_speech_t *p_app_speech = NULL;

static uint8_t incoming_intent = V2T_INTENT_NOTHING;
static uint32_t reply_input, reply_revision;
static notification_reply_state_t reply_seen;
static char reply_instance[33];
static lv_timer_t *reply_timer;
static lv_obj_t *reply_send_button;
static lv_obj_t *reply_voice_logo;
static lv_obj_t *reply_query_button;
static void reply_query_clicked(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED && !notification_reply_query(reply_input))
        ui_show_hint_toast("暫時無法查詢，文字已保留");
}
bool app_speech_is_notification_reply(void) { return incoming_intent == V2T_INTENT_REMOTE_INPUT; }

static lv_obj_t *speech_indicator_builder(lv_obj_t *src, lv_event_cb_t event_cb)
{
    lv_obj_t *speech_indicator = common_flex_button(src, false, NULL);
    lv_obj_t *ripples = create_animate_ripples(p_app_speech->app_speech_ripple, speech_indicator, 10);
    lv_obj_add_event_cb(speech_indicator, event_cb, LV_EVENT_ALL, NULL);
    return speech_indicator;
}
static lv_obj_t *text_field_builder(lv_obj_t *parent)
{
    lv_obj_t *label;
    label = lv_label_create(parent);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, 350);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    return label;
}

#ifdef BSP_USING_UI_HANDLER
static void send_message(char *message)
{
    handle_user_speech_intent(incoming_intent, message);
}

/* 語音 AI logo(founder 2026-09-23)。這頁 on_resume 就開始錄;logo 點/按住時停下來,文字留著
   給 AI 改,改完再按送出。改過的字寫回 V2T 緩衝,綁定的內容標籤自己會更新。 */
static bool s_speech_listening = false;

static const char *speech_vai_get_text(void)
{
    const char *t = get_combined_voice2text();
    return t ? t : "";
}

static void speech_vai_set_text(const char *text)
{
    voice_ai_logo_replace_v2t(text);
}

static bool speech_vai_stop_dictation(void)
{
    if (!s_speech_listening)
        return false;
    s_speech_listening = false;
    stop_voice_recognition(V2T_INTENT_NOTHING);
    return true;
}

static const voice_ai_ops_t s_speech_vai_ops = {
    .tag = "speech",
    .get_text = speech_vai_get_text,
    .set_text = speech_vai_set_text,
    .stop_dictation = speech_vai_stop_dictation,
    .on_tap = NULL,
};

static void check_and_send_v2t_result_for_reply(void)
{
    LOG_D("check_and_send_v2t_result_for_reply");
    if (voice_ai_logo_busy())
        return; /* logo 在錄修改指示 / 等 AI:送出會送到還沒改好的字 */
    if (isTextEmpty() == false)
    {
        stop_voice_recognition(V2T_INTENT_NOTHING);
        const char *text = get_combined_voice2text();
        if (app_speech_is_notification_reply()) {
            notification_reply_state_t state = notification_reply_submit(reply_input, text);
            if (state == NOTIFICATION_REPLY_WAITING || state == NOTIFICATION_REPLY_UNKNOWN) {
                if (reply_send_button) lv_obj_add_state(reply_send_button, LV_STATE_DISABLED);
            } else ui_show_hint_toast("未能傳送，文字已保留");
            return;
        }
        send_message((char *)text);
        clearVoice2Text();
        gui_app_exit(APP_ID_MESSAGE);
        gui_app_exit(APP_ID_SPEECH);
    }
}

static void press_check_and_send_v2t_result_for_reply(uint8_t press)
{
    if (press == 1)
    {
        check_and_send_v2t_result_for_reply();
    }
}

static void clear_v2t_result(void)
{
    if (isTextEmpty() == false)
    {
        bool resume = app_speech_is_notification_reply() && s_speech_listening;
        if (app_speech_is_notification_reply()) {
            stop_voice_recognition(V2T_INTENT_NOTHING);
            s_speech_listening = false;
            reply_input = notification_reply_clear(reply_input);
        }
        clearVoice2Text();
        if (app_speech_is_notification_reply() && p_app_speech)
            lv_label_set_text(p_app_speech->lbl_content, "");
        if (resume && notification_reply_listen(reply_input)) {
            start_voice_recognition(V2T_INTENT_REMOTE_INPUT);
            s_speech_listening = true;
        }
    }
}

static void handle_mic_status(void *param)
{
    bool status = *(bool *)param;
    trigger_voice_ripple(p_app_speech->app_speech_ripple, status);
}
#endif

static void handle_back_for_reply(void)
{
    if (isTextEmpty() == false)
    {
        clear_v2t_result();
    }
    else
    {
        gui_app_exit(APP_ID_SPEECH);
    }
}

static void cont_event_callback(lv_event_t *event)
{
    // clear_v2t_result();
}

static void cancel_btn_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        clear_v2t_result();
    }
}

static void send_btn_event_callback_for_reply(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        check_and_send_v2t_result_for_reply();
    }
}

static void main_speech_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        // toggle_voice_recognition();
        LOG_D("main_speech_event_callback");
    }
}

static void main_page_event_callback(lv_event_t *event)
{
    LOG_D("main_page_event_callback %s", lv_event_to_name(event->code));
    if (LV_EVENT_REFRESH == event->code)
    {
        // Entry speech main page
        LOG_D("toggle speech recognition");
    }
}


static void on_stop(void);
static void on_pause(void);

lv_obj_t *app_speech_main_init(lv_obj_t *parent)
{
    lv_obj_t *p_window;
    p_window = common_black_bg(parent);
    lv_obj_t *cont = common_container(p_window, 420, 240, cont_event_callback, lv_color_hex(0x000000));
    lv_obj_set_style_border_width(cont, 2, 0);
    lv_obj_set_style_border_color(cont, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(cont, LV_OPA_10, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_70, 0);
    lv_obj_set_style_radius(cont, 50, LV_PART_MAIN);
    p_app_speech->lbl_content = text_field_builder(cont);
    app_speech_bind_content(p_app_speech->lbl_content);

    lv_obj_t *speech_indicator = speech_indicator_builder(p_window, main_speech_event_callback);
    lv_obj_align(speech_indicator, LV_ALIGN_TOP_MID, 0, 20);

    // create trash image button and send image button on the bottom of the screen(left and right)
    lv_obj_t *icon_btn_trash = common_icon_button(p_window, ICON_TRASH, cancel_btn_event_callback);
    lv_obj_align(icon_btn_trash, LV_ALIGN_BOTTOM_LEFT, 30, -20);

    lv_obj_t *send_button;

    send_button = common_icon_button(p_window, ICON_SAND, send_btn_event_callback_for_reply);
    reply_send_button = send_button;
    lvgl_msg_handler.handle_tap_indicator = press_check_and_send_v2t_result_for_reply;
    lvgl_msg_handler.handle_back_event = handle_back_for_reply;
    lv_obj_align_to(send_button, p_window, LV_ALIGN_BOTTOM_RIGHT, -30, -20);

    /* 語音 AI logo:垃圾桶與送出之間。 */
    lv_obj_t *vai = voice_ai_logo_create(p_window, &s_speech_vai_ops, 56);
    reply_voice_logo = vai;
    lv_obj_align(vai, LV_ALIGN_BOTTOM_MID, 0, -22);

    if (app_speech_is_notification_reply()) {
        reply_query_button = lv_btn_create(p_window);
        lv_obj_set_size(reply_query_button, 230, 52);
        lv_obj_align(reply_query_button, LV_ALIGN_BOTTOM_MID, 0, -85);
        lv_obj_add_event_cb(reply_query_button, reply_query_clicked, LV_EVENT_CLICKED, NULL);
        lv_obj_t *label = lv_label_create(reply_query_button);
        lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_label_set_text(label, "查詢狀態"); lv_obj_center(label);
        lv_obj_add_flag(reply_query_button, LV_OBJ_FLAG_HIDDEN);
    }

    return p_window;
}

static lv_obj_t *on_start_main(lv_obj_t *parent)
{
    RT_ASSERT(NULL == p_app_speech);
    p_app_speech = (app_speech_t *)lv_mem_alloc(sizeof(app_speech_t));
    memset(p_app_speech, 0, sizeof(app_speech_t));
    setting_provider.set_power_save_mode(0);
    voice_provider.vad_init();
#ifdef BSP_USING_BLOC_V2T
    clearVoice2Text();
#else
    app_speech_set_content((const uint8_t *)"");
#endif
    p_app_speech->main_window = app_speech_main_init(parent);
    app_speech_set_title((const uint8_t *)"");
    app_speech_set_content((const uint8_t *)"say something...");

    return parent;
}

static void on_start_reply(void)
{
    reply_seen = NOTIFICATION_REPLY_NONE;
    reply_input = intent_get_uint32(gui_app_get_intent(), "reply_input", 0);
    reply_revision = intent_get_uint32(gui_app_get_intent(), "reply_revision", 0);
    const char *instance = intent_get_string(gui_app_get_intent(), "reply_instance");
    rt_snprintf(reply_instance, sizeof(reply_instance), "%s", instance ? instance : "");
    LOG_D("on_start_reply");
    RT_ASSERT(NULL == p_app_speech);
    p_app_speech = (app_speech_t *)lv_mem_alloc(sizeof(app_speech_t));
    memset(p_app_speech, 0, sizeof(app_speech_t));
    setting_provider.set_power_save_mode(0);
    voice_provider.vad_init();
    const char *title = intent_get_string(gui_app_get_intent(), "title");
    if (title)
    {
        app_speech_set_title((const uint8_t *)title);
    }
    else
    {
        app_speech_set_title((const uint8_t *)"");
    }
#ifdef BSP_USING_BLOC_V2T
    clearVoice2Text();
#endif
    app_speech_set_content((const uint8_t *)"");

    p_app_speech->main_window = app_speech_main_init(lv_scr_act());
    const char *draft = intent_get_string(gui_app_get_intent(), "reply_draft");
    if (draft) {
        clearVoice2Text(); setVoice2Text((char *)draft);
        lv_label_set_text(p_app_speech->lbl_content, draft);
    }
}

static void refresh_reply(lv_timer_t *timer)
{
    (void)timer;
    notification_reply_state_t state = notification_reply_state(reply_input);
    if (state != reply_seen) {
        const char *hint = state == NOTIFICATION_REPLY_WAITING ? "等待手機確認回覆" :
            state == NOTIFICATION_REPLY_UNKNOWN ? "回覆結果未確認，可查詢狀態" :
            state == NOTIFICATION_REPLY_FAILED ? "未能派送，文字已保留" :
            state == NOTIFICATION_REPLY_INVALID ? "通知已變更，請重新確認" :
            state == NOTIFICATION_REPLY_SENT ? "此通知已派送回覆" : NULL;
        if (hint) ui_show_hint_toast(hint);
        reply_seen = state;
    }
    if (reply_voice_logo && lv_obj_is_valid(reply_voice_logo)) {
        if (notification_reply_can_submit(reply_input)) lv_obj_clear_state(reply_voice_logo, LV_STATE_DISABLED);
        else lv_obj_add_state(reply_voice_logo, LV_STATE_DISABLED);
    }
    if (reply_query_button) {
        if (state == NOTIFICATION_REPLY_UNKNOWN) lv_obj_clear_flag(reply_query_button, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(reply_query_button, LV_OBJ_FLAG_HIDDEN);
    }
    if (notification_reply_can_finish(reply_input)) {
        clearVoice2Text();
        gui_app_self_exit();
        return;
    }
    if (reply_send_button && lv_obj_is_valid(reply_send_button)) {
        if (!notification_reply_can_submit(reply_input)) lv_obj_add_state(reply_send_button, LV_STATE_DISABLED);
        else lv_obj_clear_state(reply_send_button, LV_STATE_DISABLED);
    }
}
static void render_reply_transcript(uint32_t input, const char *text)
{
    if (input != reply_input || !p_app_speech) return;
    setVoice2Text((char *)text);
    lv_label_set_text(p_app_speech->lbl_content, text);
}
static void on_resume(void)
{
    if (app_speech_is_notification_reply()) {
        if (!reply_input) {
            notification_reply_view_t view = notification_reply_open_version(reply_instance, reply_revision);
            reply_input = view.input;
            if (view.draft) {
                clearVoice2Text(); setVoice2Text((char *)view.draft);
                lv_label_set_text(p_app_speech->lbl_content, view.draft);
            }
        }
        notification_reply_bind_transcript(reply_input, render_reply_transcript);
        if (!reply_timer) reply_timer = lv_timer_create(refresh_reply, 250, NULL);
        refresh_reply(NULL);
        if (!notification_reply_can_submit(reply_input)) return;
        if (!notification_reply_listen(reply_input)) { ui_show_hint_toast("無法開始回覆，請確認連線與通知"); return; }
    }
    // switch_watch_motion_control_mode(true, true);
    start_voice_recognition(incoming_intent);
    s_speech_listening = true;
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_mic = handle_mic_status;
#endif
}

static void on_pause(void)
{
    if (app_speech_is_notification_reply()) {
        notification_reply_cancel(reply_input); reply_input = 0;
        if (reply_timer) { lv_timer_del(reply_timer); reply_timer = NULL; }
    }
    stop_voice_recognition(V2T_INTENT_NOTHING);
    s_speech_listening = false;
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_mic = NULL;
#endif
}

static void on_stop(void)
{
    reply_voice_logo = NULL;
    reply_query_button = NULL;
    reply_send_button = NULL;
    // switch_watch_motion_control_mode(false, false);
    stop_voice_recognition(V2T_INTENT_NOTHING);
#ifdef BSP_USING_UI_HANDLER
    if (lvgl_msg_handler.handle_tap_indicator == press_check_and_send_v2t_result_for_reply)
    {
        lvgl_msg_handler.handle_tap_indicator = NULL;
        lvgl_msg_handler.handle_back_event = NULL;
    }
#endif
    unbind_app_speech_data();
    voice_provider.vad_deinit();
    setting_provider.set_power_save_mode(1);
    if (p_app_speech)
    {
        lv_obj_del(p_app_speech->main_window);
        p_app_speech->main_window = NULL;
        lv_mem_free(p_app_speech);
        p_app_speech = NULL;
    }
}

static void reply_msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start_reply();
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

/// @brief
/// @param msg
/// @param param
static void main_msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start_main(lv_scr_act());
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

static int app_main(intent_t i)
{
    const char *intent = intent_get_string(i, "intent");
    const char *title = intent_get_string(i, "title");
    LOG_D("app speech with name: %s ,intent: %s", title, intent);

    if (intent)
    {
        if (strcmp(intent, "reply") == 0)
        {
            LOG_D("speech_reply");
            gui_app_create_page("reply", reply_msg_handler);
            LOG_D("speech_reply END");
            incoming_intent = V2T_INTENT_REMOTE_INPUT;
        }
    }
    else
    {
        gui_app_create_page("main", main_msg_handler);
        incoming_intent = V2T_INTENT_CHAT;
    }

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(speech), IMG_LOGO, APP_ID_SPEECH, app_main, 1);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
