/**
 ******************************************************************************
 * @file   app_skai.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "watch_system_interact.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "custom_trans_anim.h"
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
    #include "ui_img_helper.h"
#endif

#ifdef BSP_USING_BLOC
    #include "bloc_setting.h"
    #include "bloc_control.h"
    #include "bloc_skaiwalk.h"
    #include "bloc_motion_tracking.h"
    #include "bloc_v2t.h"
#endif

#define DBG_TAG "app.skai.ai"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *main_window;
    lv_obj_t *app_bar;
    lv_obj_t *bottom_sheet;
} app_skai_t;

LV_IMG_DECLARE(icon_mic);

extern void app_speech_bind_title(lv_obj_t *target);
extern void app_speech_bind_content(lv_obj_t *target);
extern void app_speech_set_content(const uint8_t *title);
extern void unbind_app_speech_data(void);

    #define LIST_SKAI_WIDTH (360)
    #define LIST_SKAI_HEIGHT (100)
    #define LIST_SKAI_SPACING (20)

    #define LIST_RADIUS (1000)

    #define LIST_SKAI_RADIUS (260)
    #define LIST_SKAI_BORDER_SIDE LV_BORDER_SIDE_RIGHT

    #define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
    #define FONT_SIZE FONT_BIGL
    #define LIST_ENABLE_ARC_SCROLLBAR 0


static const lv_style_const_prop_t LIST_SKAI_STYLE_PROPS[] = {
    LV_STYLE_CONST_WIDTH(LIST_SKAI_WIDTH),
    LV_STYLE_CONST_HEIGHT(LIST_SKAI_HEIGHT),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t LIST_SKAI_TITLE_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_14),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(LIST_SKAI_STYLE, LIST_SKAI_STYLE_PROPS);
LV_STYLE_CONST_INIT(LIST_SKAI_TITLE_STYLE, LIST_SKAI_TITLE_STYLE_PROPS);


void open_skai_widget_ai(bool open);
void set_skai_widget_input_text(const char *text);
void ai_widget_start(void)
{
    open_skai_widget_ai(true);
    // animate_to_ai_page();
    set_skai_widget_input_text("");
    set_ai_open_mic(true);
    show_speech_indicator(true);
    voice_provider.start_v2t();
    set_free_control_with_arm(false);
}

static lv_obj_t *skai_widget_input_text;
static lv_obj_t *skai_widget_input_prompt;
void send_to_ai(void)
{
    if (isTextEmpty())
    {
        LOG_D("Input text is empty, ignoring send to AI");
        return;
    }
    voice_provider.auto_stop_listening();
}

static void send_to_note(void)
{
    handle_user_speech_intent(V2T_INTENT_SKAIBAR,
                              lv_label_get_text(skai_widget_input_text));
    animate_to_home_from_notification_center();
}


static lv_obj_t *skai_widget_input_text_bg;
static lv_obj_t *skai_widget_ai_reply;
static lv_obj_t *skai_widget_bg_scroller;
lv_obj_t *lv_skai_widget_builder(lv_obj_t *parent)
{
    lv_obj_t *skai_widget_bg = lv_obj_create(parent);
    skai_widget_bg_scroller = skai_widget_bg;
    lv_obj_set_size(skai_widget_bg, 466, 466);
    lv_obj_align(skai_widget_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(skai_widget_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(skai_widget_bg, 0, 0);
    lv_obj_set_style_pad_all(skai_widget_bg, 0, 0);
    lv_obj_set_scroll_dir(skai_widget_bg, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(skai_widget_bg, LV_SCROLLBAR_MODE_OFF);
    /* Prevent vertical scroll chain to ai_page/tileview so scrolling the
       AI reply never slides the outer tileview toward the app grid. */
    lv_obj_clear_flag(skai_widget_bg, LV_OBJ_FLAG_SCROLL_CHAIN_VER);

    skai_widget_input_text_bg = lv_obj_create(skai_widget_bg);
    /* Was 250 — but reset_skai_widget_input_text() snaps to 150 on every
       session reset, and the dynamic grow path that justified the larger
       initial size has been retired. Start at 150 so the first reveal
       matches every subsequent reset. */
    lv_obj_set_size(skai_widget_input_text_bg, 430, 150);
    lv_obj_align(skai_widget_input_text_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(skai_widget_input_text_bg, lv_color_hex(0xFFFFFF),
                              0);
    lv_obj_set_style_bg_opa(skai_widget_input_text_bg, 5, 0);
    lv_obj_set_style_radius(skai_widget_input_text_bg, 75, 0);
    lv_obj_set_style_border_width(skai_widget_input_text_bg, 5, 0);
    lv_obj_set_style_border_color(skai_widget_input_text_bg,
                                  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(skai_widget_input_text_bg, LV_OPA_50, 0);

    skai_widget_input_text = lv_label_create(skai_widget_input_text_bg);
    lv_label_set_text(skai_widget_input_text, "");
    lv_obj_set_style_text_opa(skai_widget_input_text, LV_OPA_50, 0);
    lv_obj_set_style_text_font(skai_widget_input_text,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_label_set_long_mode(skai_widget_input_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(skai_widget_input_text, 380);
    lv_obj_set_style_text_color(skai_widget_input_text, lv_color_white(), 0);
    /* Top-aligned per founder direction 2026-05-19: when transcribed text
       grows from 1 line to multiple lines, top-anchor keeps the first line
       fixed instead of centring (which would bounce the text upward each
       new line). 10 px padding from the bg top so the label has breathing
       room from the pill border. */
    lv_obj_align(skai_widget_input_text, LV_ALIGN_TOP_MID, 0, 10);

    skai_widget_input_prompt = lv_label_create(skai_widget_input_text_bg);
    lv_label_set_text(skai_widget_input_prompt,
                      LV_EXT_STR_GET_BY_KEY(listening, "Listening"));
    lv_obj_set_style_text_font(skai_widget_input_prompt,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_label_set_long_mode(skai_widget_input_prompt, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_opa(skai_widget_input_prompt, LV_OPA_0, 0);
    lv_obj_set_width(skai_widget_input_prompt, 380);
    lv_obj_set_style_text_color(skai_widget_input_prompt, lv_color_white(), 0);
    /* Match the user-text label's top-mid anchor so the placeholder and
       the transcribed text share the same starting baseline. */
    lv_obj_align(skai_widget_input_prompt, LV_ALIGN_TOP_MID, 10, 10);

    /* AI reply label — positioned below the input bg.
       Narrower width (340) so text fits in the visible area of the 466-circle
       at lower y positions, and a large pad_bottom so the last lines can be
       scrolled into the visible center (the bottom corners of the circle
       clip the text). */
    skai_widget_ai_reply = lv_label_create(skai_widget_bg);
    lv_label_set_text(skai_widget_ai_reply, "");
    lv_obj_set_style_text_font(skai_widget_ai_reply,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_label_set_long_mode(skai_widget_ai_reply, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(skai_widget_ai_reply, 340);
    lv_obj_set_style_text_color(skai_widget_ai_reply, lv_color_hex(0xF0F0F0),
                                0);
    lv_obj_set_style_text_opa(skai_widget_ai_reply, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_bottom(skai_widget_ai_reply, 150, 0);
    lv_obj_align_to(skai_widget_ai_reply, skai_widget_input_text_bg,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_add_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN);


    // lv_obj_t *note_label = lv_label_create(skai_widget_note_button);
    // lv_label_set_text(note_label, "筆記");
    // lv_obj_set_style_text_font(note_label,
    // LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    // lv_obj_set_style_text_color(note_label, lv_color_white(), 0);
    // lv_obj_align(note_label, LV_ALIGN_CENTER, 0, 0);

    // 創建 AI 按鈕（右邊）
    // skai_widget_ai_button = lv_obj_create(parent);
    // lv_obj_set_size(skai_widget_ai_button, 100, 70);
    // lv_obj_align_to(skai_widget_ai_button, skai_widget_input_text_bg,
    // LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 10);
    // lv_obj_set_style_bg_color(skai_widget_ai_button, lv_color_hex(0x000000),
    // 0); lv_obj_set_style_bg_opa(skai_widget_ai_button, LV_OPA_80, 0);
    // lv_obj_set_style_radius(skai_widget_ai_button, 20, 0);
    // lv_obj_set_style_border_width(skai_widget_ai_button, 1, 0);
    // lv_obj_set_style_border_color(skai_widget_ai_button,
    // lv_color_hex(0xFFFFFF), 0);
    // lv_obj_set_style_border_opa(skai_widget_ai_button, LV_OPA_30, 0);
    // lv_obj_add_event_cb(skai_widget_ai_button,
    // skai_widget_ai_button_event_cb, LV_EVENT_CLICKED, NULL);

    // lv_obj_t *ai_label = lv_label_create(skai_widget_ai_button);
    // lv_label_set_text(ai_label, "AI");
    // lv_obj_set_style_text_font(ai_label,
    // LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    // lv_obj_set_style_text_color(ai_label, lv_color_white(), 0);
    // lv_obj_align(ai_label, LV_ALIGN_CENTER, 0, 0);

    return skai_widget_input_text_bg;
}

static bool skai_widget_open = false;
void set_skai_widget_opa(uint8_t opa)
{

    uint8_t bg_opa = 20 * opa / LV_OPA_COVER;
    if (skai_widget_input_text_bg != NULL && lv_obj_is_valid(skai_widget_input_text_bg))
    {
        lv_obj_set_style_bg_opa(skai_widget_input_text_bg, bg_opa, 0);
        uint8_t border_opa = LV_OPA_50 * opa / LV_OPA_COVER;
        lv_obj_set_style_border_opa(skai_widget_input_text_bg, border_opa, 0);
    }
    uint8_t text_opa = LV_OPA_COVER * opa / LV_OPA_COVER;
    if (skai_widget_input_text != NULL && lv_obj_is_valid(skai_widget_input_text))
    {
        lv_obj_set_style_text_opa(skai_widget_input_text, text_opa, 0);
    }
    uint8_t prompt_opa = LV_OPA_60 * opa / LV_OPA_COVER;
    if (skai_widget_input_prompt != NULL && lv_obj_is_valid(skai_widget_input_prompt))
    {
        lv_obj_set_style_text_opa(skai_widget_input_prompt, prompt_opa, 0);
    }
    if (skai_widget_ai_reply != NULL && lv_obj_is_valid(skai_widget_ai_reply))
    {
        lv_obj_set_style_text_opa(skai_widget_ai_reply, text_opa, 0);
    }
    // LOG_D("set_skai_widget_opa opa=%d,%d,%d,%d", opa, bg_opa, border_opa,
    //       text_opa);
    if (opa == 0)
        skai_widget_open = false;
    else if (opa == LV_OPA_COVER)
        skai_widget_open = true;
}

extern void add_to_self_note(char *text);
static bool selected_function = true;
bool get_selected_function(void)
{
    return selected_function;
}
static void speech_tap_cb(uint8_t pressed)
{
    if (pressed != 1 || isTextEmpty())
        return;
    if (selected_function)
    {
        send_to_ai();
    }
    else
    {
        send_to_note();
    }
}

static lv_timer_t *listening_anim_timer = NULL;
static uint8_t listening_anim_dot = 0;
static void listening_anim_cb(lv_timer_t *timer)
{
    listening_anim_dot = (listening_anim_dot + 1) % 4;
    if (skai_widget_input_prompt != NULL && skai_widget_input_text_bg != NULL &&
        lv_obj_is_valid(skai_widget_input_prompt))
    {
        char buf[32];
        lv_snprintf(buf, sizeof(buf), "%s",
                    LV_EXT_STR_GET_BY_KEY(listening, "Listening"));
        for (uint8_t i = 0; i < listening_anim_dot; i++)
            strcat(buf, ".");
        if (skai_widget_open)
            lv_obj_set_style_text_opa(skai_widget_input_prompt, LV_OPA_60, 0);
        lv_label_set_text(skai_widget_input_prompt, buf);
        lv_obj_update_layout(skai_widget_input_prompt);
    }
}

static void start_listening_animation(void)
{

    if (!listening_anim_timer)
    {
        listening_anim_dot = 0;
        listening_anim_timer = lv_timer_create(listening_anim_cb, 500, NULL);
    }
    else
    {
        listening_anim_dot = 0;
        lv_timer_resume(listening_anim_timer);
    }
}
static void stop_listening_animation(void)
{

    if (listening_anim_timer)
    {
        lv_timer_pause(listening_anim_timer);
    }
}

static lv_anim_t listening_shadow_anim;
static void indicator_test_anim_exec_cb(void *var, int32_t value)
{
    // uint8_t shadow_width = 50 + (uint8_t)((value * 20) / 100);
    // lv_obj_set_style_shadow_width(skai_widget_input_text_bg, shadow_width,
    // 0);

    uint8_t shadow_opa =
        (uint8_t)(LV_OPA_40 + ((value * (LV_OPA_80 - LV_OPA_40)) / 100));
    lv_obj_set_style_shadow_opa(skai_widget_input_text_bg, shadow_opa, 0);
    // lv_obj_set_style_border_opa(skai_widget_input_text_bg, shadow_opa,
    // LV_STATE_DEFAULT);
}
static void listening_shadow_animation(bool start)
{
    LOG_D("listening_shadow_animation start=%d", start);
    if (start)
    {
        // lv_obj_set_style_shadow_width(skai_widget_input_text_bg, 50, 0);
        lv_anim_init(&listening_shadow_anim);
        lv_anim_set_var(&listening_shadow_anim, NULL);
        lv_anim_set_values(&listening_shadow_anim, 0, 100);
        lv_anim_set_time(&listening_shadow_anim, 1000);
        lv_anim_set_repeat_count(&listening_shadow_anim,
                                 LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_playback_time(&listening_shadow_anim, 1000);
        lv_anim_set_path_cb(&listening_shadow_anim, lv_anim_path_ease_in_out);
        lv_anim_set_exec_cb(&listening_shadow_anim,
                            indicator_test_anim_exec_cb);
        lv_anim_start(&listening_shadow_anim);
    }
    else
    {
        lv_anim_del(&listening_shadow_anim, indicator_test_anim_exec_cb);
        // lv_obj_set_style_shadow_width(skai_widget_input_text_bg, 0, 0);
        lv_obj_set_style_shadow_opa(skai_widget_input_text_bg, LV_OPA_0, 0);
    }
}
extern bool set_is_open_instruction_list_ai(bool open);
void reset_skai_widget_input_text(void);
void clear_skai_widget_ai_reply(void);
static bool prev_open_state = false;
void open_skai_widget_ai(bool open)
{
    if (skai_widget_input_text != NULL && skai_widget_input_text_bg != NULL &&
        lv_obj_is_valid(skai_widget_input_text_bg) && prev_open_state != open)
    {
        if (open)
        {
            lv_label_set_text(skai_widget_input_text, "");
            // lv_label_set_text(skai_widget_input_prompt, "聽取中");
            // lv_obj_set_style_text_opa(skai_widget_input_prompt, LV_OPA_100,
            // 0);
            setting_provider.set_power_save_mode(0);
            start_listening_animation();
            // listening_shadow_animation(true);
            // lv_obj_set_style_outline_width(skai_widget_input_text_bg, 0,
            //                           LV_STATE_DEFAULT);
            // lv_obj_set_style_bg_opa(skai_widget_input_text_bg, 20, 0);
        }
        else
        {
            setting_provider.set_power_save_mode(1);
            stop_listening_animation();
            // listening_shadow_animation(false);
            lv_label_set_text(skai_widget_input_prompt,
                              LV_EXT_STR_GET_BY_KEY(listening, "Listening"));
            // lv_obj_set_style_text_opa(skai_widget_input_prompt, LV_OPA_50,
            // 0); lv_obj_set_style_outline_width(skai_widget_input_text_bg, 5,
            //                           LV_STATE_DEFAULT);
            // lv_obj_set_style_bg_opa(skai_widget_input_text_bg, 5, 0);
            // voice_provider.auto_stop_listening();
            // voice_provider.stop_v2t();
            // reset_skai_widget_input_text();
            // set_is_open_instruction_list_ai(false);
            set_skai_widget_input_text("");
            clear_skai_widget_ai_reply();
            lv_obj_update_layout(skai_widget_input_text);
            lv_obj_set_height(skai_widget_input_text_bg, 150);
        }
        prev_open_state = open;
    }
}

static bool input_text_is_null = true;
bool get_skai_input_text_is_null(void)
{
    return input_text_is_null;
}
extern bool get_voice_recognition_started(void);
extern bool set_is_open_instruction_list_ai(bool open);
extern bool get_is_open_instruction_list_ai(void);
void back_on_skai_widget(void)
{
    if (skai_widget_input_text != NULL &&
        lv_obj_is_valid(skai_widget_input_text))
    {
        const char *text = lv_label_get_text(skai_widget_input_text);
        /* Drop the voice-started gate so back ALWAYS clears the input first when
           there is text: 2-step back = 1st press clears + keeps the widget open,
           2nd press (now empty) closes. The gate broke this once voice auto-stops
           before the user backs — back fell straight through to close without
           clearing (the regressed "back clears the AI widget" behaviour). */
        if (text && strlen(text) > 0 && !input_text_is_null)
        {
            count_speech_coding();
            reset_skai_widget_input_text();
        }
        else if (get_is_open_instruction_list_ai())
        {
            // animate_to_home_from_notification_center();
            
            extern void close_ai_widget(void);
            close_ai_widget();
            
        }
        else
        {
            clock_on_resume();
            animate_to_home_from_notification_center();
            screen_rotate_back_to_original_direction();
        }
    }
}

static void button_selection(gesture_position_t gesture_position)
{
}

extern void show_send_icon(void);
void set_skai_widget_input_text(const char *text)
{
    if (strcmp(text, "") == 0)
    {
        lv_obj_clear_flag(skai_widget_input_prompt, LV_OBJ_FLAG_HIDDEN);
        // lv_obj_set_style_text_opa(skai_widget_input_prompt, LV_OPA_50, 0);
        lv_label_set_text(skai_widget_input_text, "");
        input_text_is_null = true;
        return;
    }
    show_send_icon();
    lv_obj_add_flag(skai_widget_input_prompt, LV_OBJ_FLAG_HIDDEN);
    input_text_is_null = false;
    /* Show skai_widget when first text arrives */
    extern void instruction_ai_show_skai_widget(void);
    instruction_ai_show_skai_widget();
    if (skai_widget_input_text != NULL && skai_widget_input_text_bg != NULL &&
        lv_obj_is_valid(skai_widget_input_text))
    {
        lv_label_set_text(skai_widget_input_text, text);
        /* Dynamic bg-height resize retired: skai_widget_input_text_bg is
           LV_ALIGN_CENTER inside its parent, so growing its height pushed
           the bottom edge down (and the top up) every time the user spoke
           a longer phrase — the input box visibly "ran down" the screen.
           Keep height fixed at the initial 150 px and rely on
           LV_LABEL_LONG_WRAP for multi-line text. Phrases longer than the
           box clip; short voice inputs (the common case) sit cleanly in
           a static frame. */
    }
}

void reset_skai_widget_input_text(void)
{
    if (skai_widget_input_text != NULL &&
        lv_obj_is_valid(skai_widget_input_text))
    {
        // stop_listening_animation();
        // lv_label_set_text(skai_widget_input_text, "");
        clearVoice2Text();
        set_skai_widget_input_text("");
        /* The VISIBLE transcript moved to the instruction_list's own label
           (s_voice_transcript_label) when the AI widget became the device_pager-
           style input box; set_skai_widget_input_text above only clears the
           now-hidden builder label. Clear the visible one too (to the "Listening"
           placeholder, matching refresh_ai_chat_input_message's empty path) — else
           back/reset cleared the data (textLen->0) but left the spoken text on
           screen. */
        extern void instruction_list_set_voice_transcript(const char *text);
        instruction_list_set_voice_transcript(
            LV_EXT_STR_GET_BY_KEY(listening, "Listening"));
        lv_obj_update_layout(skai_widget_input_text);
        lv_obj_set_height(skai_widget_input_text_bg, 150);

        // 重新對齊按鈕位置
        // if (skai_widget_note_button != NULL &&
        // lv_obj_is_valid(skai_widget_note_button))
        // {
        //     lv_obj_align_to(skai_widget_note_button,
        //     skai_widget_input_text_bg, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
        // }
        // if (skai_widget_ai_button != NULL &&
        // lv_obj_is_valid(skai_widget_ai_button))
        // {
        //     lv_obj_align_to(skai_widget_ai_button, skai_widget_input_text_bg,
        //     LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 10);
        // }
    }
}

/* True between the user tapping send and the first AI chunk arriving; lets
   us show a placeholder that the first chunk replaces. */
static bool skai_widget_awaiting_first_chunk = false;

/* AI-reply-in-pill feature retired: founder direction is that the
   instruction-list AI widget only shows the user's spoken text, not the
   AI's reply. The functions below stay as no-ops so existing call sites
   (ui_handler.c streaming hook, KEY_AI_PROCESS_TOOLKIT toolkit-progress
   pushes, lv_instruction_list_layout.c re-ask branch) compile and link
   without a churn pass through every caller. The auto-scroll-to-bottom
   in the old append went with it; with nothing being appended, the
   scroller never moves. The skai_widget_ai_reply LVGL object is left
   created in lv_skai_widget_builder but stays hidden + empty forever. */
void set_skai_widget_processing_text(const char *text)
{
    (void)text;
}

void set_skai_widget_awaiting_ai(void) {}

void append_skai_widget_ai_reply(const char *text)
{
    (void)text;
}

/* Still called from close_ai_widget / logo_click re-ask path. Keep the
   placeholder-flag reset so the (now no-op) awaiting flag never lingers
   true across sessions, and keep the scroll reset so any earlier scroll
   from this session (e.g. user-driven) doesn't carry into the next. */
void clear_skai_widget_ai_reply(void)
{
    skai_widget_awaiting_first_chunk = false;
    if (skai_widget_ai_reply != NULL &&
        lv_obj_is_valid(skai_widget_ai_reply))
    {
        lv_label_set_text(skai_widget_ai_reply, "");
        lv_obj_add_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN);
    }
    if (skai_widget_bg_scroller != NULL &&
        lv_obj_is_valid(skai_widget_bg_scroller))
    {
        lv_obj_scroll_to_y(skai_widget_bg_scroller, 0, LV_ANIM_OFF);
    }
}

/* Always false now: there is no AI-reply state to be in. The re-ask
   branch in logo_click_event_cb (lv_instruction_list_layout.c) becomes
   unreachable, which matches the new flow — mic button always restarts
   v2t, never resumes a prior reply. */
bool skai_widget_has_ai_reply(void) { return false; }

rt_int32_t speech_on_resume(void)
{
    setting_provider.set_power_save_mode(0);
    lvgl_msg_handler.handle_widgets_control = button_selection;
    lvgl_msg_handler.handle_tap_indicator = speech_tap_cb;
    return RT_EOK;
}

rt_int32_t speech_on_pause(void)
{
    if (lvgl_msg_handler.handle_widgets_control == button_selection)
    {
        lvgl_msg_handler.handle_widgets_control = NULL;
    }
    if (lvgl_msg_handler.handle_tap_indicator == speech_tap_cb)
        lvgl_msg_handler.handle_tap_indicator = NULL;
    return RT_EOK;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/