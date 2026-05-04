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
static void display_chat_history(lv_obj_t *list);

    #define LIST_SKAI_WIDTH (360)
    #define LIST_SKAI_HEIGHT (100)
    #define LIST_SKAI_SPACING (20)

    #define LIST_RADIUS (1000)

    #define LIST_SKAI_RADIUS (260)
    #define LIST_SKAI_BORDER_SIDE LV_BORDER_SIDE_RIGHT

    #define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
    #define FONT_SIZE FONT_BIGL
    #define LIST_ENABLE_ARC_SCROLLBAR 0

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */
static app_skai_t *p_app_skai = NULL;

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

static bool open_action_flag = true;
static bool left_hand_mode = true;
static uint16_t selected_message_index = 0;
static lv_obj_t *first_open_ai_window;
static app_speech_ripple_t app_speech_point[3];
static lv_coord_t single_line_height = 0;
static int bottom_of_previous_window = 0;

static void scroll_list(lv_obj_t *obj, int16_t drift)
{
}

static void list_window_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;

    if (obj == NULL)
    {
        return;
    }
    scroll_list(obj, 0);
}

static void list_message_click_event_cb(lv_event_t *evt)
{
    // LOG_D("list_message_click_event_cb %d");
}

static lv_coord_t total_height = 0;
static lv_obj_t *create_message_widget(lv_obj_t *list, chat_t *message,
                                       int selected_message_index, bool last)
{
    int offset = 0;
    if (message->is_self)
    {
        offset = 20;
    }
    else
    {
        offset = -20;
    }
    lv_obj_t *message_widget =
        common_list_widget(list, (lv_style_t *)&LIST_SKAI_STYLE,
                           (LV_HOR_RES_MAX - LIST_SKAI_WIDTH) / 2 + offset,
                           bottom_of_previous_window);
    lv_obj_add_flag(message_widget, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(message_widget, list_message_click_event_cb,
                        LV_EVENT_CLICKED, NULL);

    lv_obj_t *content = lv_label_create(message_widget);
    lv_obj_set_style_text_font(content,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(content, lv_color_white(), 0);
    lv_label_set_long_mode(content, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(content, LIST_SKAI_WIDTH - 20);
    lv_label_set_text(content, message->message);
    if (last)
    {
        create_animate_points(app_speech_point, message_widget);
        wait_for_message(app_speech_point, true);
    }

    lv_obj_align(content, LV_ALIGN_CENTER, 0, 0);

    if (message->is_self)
    {
        lv_obj_set_style_bg_color(message_widget, lv_color_hex(0x97CBFF), 0);
        lv_obj_set_style_bg_opa(message_widget, LV_OPA_10, 0);
    }
    else
    {
        lv_obj_set_style_bg_color(message_widget, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(message_widget, LV_OPA_10, 0);
    }

    lv_obj_set_style_radius(message_widget, 50, LV_PART_MAIN);

    lv_obj_update_layout(content);

    lv_coord_t label_height = lv_obj_get_height(content);
    lv_coord_t padding = 10;
    total_height = label_height + 2 * padding;
    lv_obj_set_height(message_widget, total_height);

    // lv_coord_t text_width = lv_label_get_text_width(content);
    // lv_coord_t total_width = text_width + 20;
    // lv_obj_set_width(message_widget, total_width);

    bottom_of_previous_window =
        bottom_of_previous_window + total_height + LIST_SKAI_SPACING;
    return message_widget;
}

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
    handle_user_speech_intent(V2T_INTENT_NOTE_CREATING,
                              lv_label_get_text(skai_widget_input_text));
    animate_to_home_from_notification_center();
}

static void skai_widget_ai_button_event_cb(lv_event_t *evt)
{
    send_to_ai();
}

static void skai_widget_note_button_event_cb(lv_event_t *evt)
{
    send_to_note();
}
static lv_obj_t *skai_widget_input_text_bg;
static lv_obj_t *skai_widget_ai_bg;
static lv_obj_t *skai_widget_note_button;
static lv_obj_t *skai_widget_ai_button;
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
    lv_obj_set_size(skai_widget_input_text_bg, 430, 250);
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
    lv_obj_align(skai_widget_input_text, LV_ALIGN_CENTER, 0, 0);

    skai_widget_input_prompt = lv_label_create(skai_widget_input_text_bg);
    lv_label_set_text(skai_widget_input_prompt, "聽取中");
    lv_obj_set_style_text_font(skai_widget_input_prompt,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_label_set_long_mode(skai_widget_input_prompt, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_opa(skai_widget_input_prompt, LV_OPA_0, 0);
    lv_obj_set_width(skai_widget_input_prompt, 380);
    lv_obj_set_style_text_color(skai_widget_input_prompt, lv_color_white(), 0);
    lv_obj_align(skai_widget_input_prompt, LV_ALIGN_CENTER, 10, 0);

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

    // 創建筆記按鈕（左邊）
    // skai_widget_note_button = lv_obj_create(parent);
    // lv_obj_set_size(skai_widget_note_button, 100, 70);
    // lv_obj_align_to(skai_widget_note_button, skai_widget_input_text_bg,
    // LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    // lv_obj_set_style_bg_color(skai_widget_note_button,
    // lv_color_hex(0x000000), 0);
    // lv_obj_set_style_bg_opa(skai_widget_note_button, LV_OPA_80, 0);
    // lv_obj_set_style_radius(skai_widget_note_button, 20, 0);
    // lv_obj_set_style_border_width(skai_widget_note_button, 1, 0);
    // lv_obj_set_style_border_color(skai_widget_note_button,
    // lv_color_hex(0xFFFFFF), 0);
    // lv_obj_set_style_border_opa(skai_widget_note_button, LV_OPA_30, 0);
    // lv_obj_add_event_cb(skai_widget_note_button,
    // skai_widget_note_button_event_cb, LV_EVENT_CLICKED, NULL);

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
    lv_obj_set_style_bg_opa(skai_widget_input_text_bg, bg_opa, 0);

    uint8_t border_opa = LV_OPA_50 * opa / LV_OPA_COVER;
    lv_obj_set_style_border_opa(skai_widget_input_text_bg, border_opa, 0);
    uint8_t text_opa = LV_OPA_COVER * opa / LV_OPA_COVER;
    lv_obj_set_style_text_opa(skai_widget_input_text, text_opa, 0);
    uint8_t prompt_opa = LV_OPA_60 * opa / LV_OPA_COVER;
    lv_obj_set_style_text_opa(skai_widget_input_prompt, prompt_opa, 0);
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
        char buf[24] = "聽取中";
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
            lv_label_set_text(skai_widget_input_prompt, "聽取中");
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
        if (text && strlen(text) > 0 && !input_text_is_null &&
            get_voice_recognition_started())
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
    // const int p_y = gesture_position.gesture_position_y;
    // // LOG_D("p_y=%d", p_y);
    // if (p_y > 233) // 上半區
    // {
    //     if (selected_function)
    //         selected_function = false;
    //     lv_obj_set_style_border_opa(skai_widget_ai_button, LV_OPA_20,
    //     LV_STATE_DEFAULT);
    //     lv_obj_set_style_border_width(skai_widget_ai_button, 1,
    //     LV_STATE_DEFAULT);
    //     lv_obj_set_style_border_opa(skai_widget_note_button, LV_OPA_50,
    //     LV_STATE_DEFAULT);
    //     lv_obj_set_style_border_width(skai_widget_note_button, 2,
    //     LV_STATE_DEFAULT);
    // }
    // else if (p_y < 233) // 下半區
    // {
    //     if (!selected_function)
    //         selected_function = true;
    //     lv_obj_set_style_border_opa(skai_widget_ai_button, LV_OPA_50,
    //     LV_STATE_DEFAULT);
    //     lv_obj_set_style_border_width(skai_widget_ai_button, 2,
    //     LV_STATE_DEFAULT);
    //     lv_obj_set_style_border_opa(skai_widget_note_button, LV_OPA_20,
    //     LV_STATE_DEFAULT);
    //     lv_obj_set_style_border_width(skai_widget_note_button, 1,
    //     LV_STATE_DEFAULT);
    // }
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
        lv_obj_update_layout(skai_widget_input_text);
        // 取得文字高度，並根據內容調整背景高度
        lv_coord_t label_height = lv_obj_get_height(skai_widget_input_text);
        lv_coord_t min_height = 150; // 最小高度
        lv_coord_t padding = 20;     // 上下留白
        lv_coord_t new_height = label_height + padding;
        if (new_height < min_height)
            new_height = min_height;
        lv_obj_set_height(skai_widget_input_text_bg, new_height);

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

void reset_skai_widget_input_text(void)
{
    if (skai_widget_input_text != NULL &&
        lv_obj_is_valid(skai_widget_input_text))
    {
        // stop_listening_animation();
        // lv_label_set_text(skai_widget_input_text, "");
        clearVoice2Text();
        set_skai_widget_input_text("");
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

/* Update the placeholder text with the AI's current action (sent via
   KEY_AI_PROCESS_TOOLKIT). Only affects the widget while we're waiting for
   the first AI reply chunk; after real text starts arriving this is a no-op
   so the streamed reply isn't overwritten. */
void set_skai_widget_processing_text(const char *text)
{
    if (text == NULL || skai_widget_ai_reply == NULL ||
        !lv_obj_is_valid(skai_widget_ai_reply))
    {
        return;
    }
    if (!skai_widget_awaiting_first_chunk)
    {
        return;
    }
    lv_label_set_text(skai_widget_ai_reply, text);
    if (lv_obj_has_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_clear_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_align_to(skai_widget_ai_reply, skai_widget_input_text_bg,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_update_layout(skai_widget_ai_reply);
}

/* Show "AI processing..." below the input box so the user gets immediate feedback
   after tapping send. Replaced by the real reply on the first chunk. */
void set_skai_widget_awaiting_ai(void)
{
    if (skai_widget_ai_reply == NULL ||
        !lv_obj_is_valid(skai_widget_ai_reply))
    {
        return;
    }
    skai_widget_awaiting_first_chunk = true;
    lv_label_set_text(skai_widget_ai_reply, "AI processing...");
    if (lv_obj_has_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_clear_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_align_to(skai_widget_ai_reply, skai_widget_input_text_bg,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

/* Append streamed AI reply text under the input box. */
void append_skai_widget_ai_reply(const char *text)
{
    if (skai_widget_ai_reply == NULL ||
        !lv_obj_is_valid(skai_widget_ai_reply) || text == NULL)
    {
        return;
    }
    if (lv_obj_has_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_clear_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN);
    }
    /* First real chunk replaces the "AI processing..." placeholder. */
    if (skai_widget_awaiting_first_chunk)
    {
        lv_label_set_text(skai_widget_ai_reply, "");
        skai_widget_awaiting_first_chunk = false;
    }
    const char *current = lv_label_get_text(skai_widget_ai_reply);
    size_t cur_len = current ? strlen(current) : 0;
    size_t combined_len = cur_len + strlen(text) + 1;
    char *combined = (char *)rt_malloc(combined_len);
    if (combined == NULL)
        return;
    if (cur_len > 0)
        strcpy(combined, current);
    else
        combined[0] = '\0';
    strcat(combined, text);
    lv_label_set_text(skai_widget_ai_reply, combined);
    rt_free(combined);

    lv_obj_align_to(skai_widget_ai_reply, skai_widget_input_text_bg,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_update_layout(skai_widget_ai_reply);

    /* Auto-scroll to the latest line like the old voice_recognition page
       did (lv_obj_scroll_to_y on speech_bg with a large y). */
    if (skai_widget_bg_scroller != NULL &&
        lv_obj_is_valid(skai_widget_bg_scroller))
    {
        lv_obj_scroll_to_y(skai_widget_bg_scroller, 10000, LV_ANIM_ON);
    }
}

/* Clear AI reply; called when user wants to ask again. */
void clear_skai_widget_ai_reply(void)
{
    skai_widget_awaiting_first_chunk = false;
    if (skai_widget_ai_reply == NULL ||
        !lv_obj_is_valid(skai_widget_ai_reply))
    {
        return;
    }
    lv_label_set_text(skai_widget_ai_reply, "");
    lv_obj_add_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN);
    if (skai_widget_bg_scroller != NULL &&
        lv_obj_is_valid(skai_widget_bg_scroller))
    {
        lv_obj_scroll_to_y(skai_widget_bg_scroller, 0, LV_ANIM_OFF);
    }
}

bool skai_widget_has_ai_reply(void)
{
    if (skai_widget_ai_reply == NULL ||
        !lv_obj_is_valid(skai_widget_ai_reply))
    {
        return false;
    }
    if (lv_obj_has_flag(skai_widget_ai_reply, LV_OBJ_FLAG_HIDDEN))
        return false;
    const char *current = lv_label_get_text(skai_widget_ai_reply);
    return current != NULL && current[0] != '\0';
}

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

#ifdef APP_ID_SKAI
static bool last = false;
static void refresh_list(lv_obj_t *list, uint8_t new_item_count,
                         bool wait_for_ai)
{
    /* Delete all children of the list */
    lv_obj_clean(list);
    if (wait_for_ai)
    {
        new_item_count++;
    }
    selected_message_index = new_item_count - 1;
    bottom_of_previous_window = LIST_SKAI_HEIGHT + LIST_SKAI_SPACING;
    /* Repopulate the list with the new number of items */
    for (uint8_t i = 0; i < new_item_count; i++)
    {
        chat_t *message = get_skai_message(get_message_list(),
                                           *skai_message_count_ptr(), i, true);
        if (i == new_item_count - 1 && wait_for_ai)
        {
            message->is_self = false;
            strcpy(message->message, " ");
            last = true;
        }
        create_message_widget(list, message, selected_message_index, last);
        last = false;
    }

    /* Scroll back to the selected item of the list */
    lv_obj_scroll_to_view(lv_obj_get_child(list, selected_message_index - 1),
                          LV_ANIM_OFF);
    lv_obj_scroll_by(list, 0, -total_height + 90, LV_ANIM_OFF);
}

static bool wait_for_ai_respond = false;
static void refresh_message_list(void)
{
    uint16_t message_count = *skai_message_count_ptr();
    LOG_D("refresh_message_list message_count=%d", message_count);
    wait_for_ai_respond = false;
    if (message_count > 0)
    {
        refresh_list(p_app_skai->main_window, message_count, false);
    }
}

static void wait_for_ai_refresh_message_list(void)
{
    wait_for_ai_respond = true;
    uint16_t message_count = *skai_message_count_ptr();
    if (message_count > 0)
    {
        refresh_list(p_app_skai->main_window, message_count, true);
    }
}

static char hint_text[] = "Say something";
static lv_obj_t *input_text;
static lv_obj_t *sheet;
static lv_obj_t *mic_button;

static void set_input_textfield(char *text)
{
    lv_label_set_text(input_text, text);
    lv_obj_update_layout(input_text);
}

static void refresh_input_message(char *text)
{
    LOG_D("refresh_input_AImessage");
    set_input_textfield(text);

    lv_coord_t line_count = lv_obj_get_height(input_text) - 52;

    lv_coord_t total_height = 60 + (line_count);
    lv_obj_set_height(sheet, total_height);

    if (strcmp(text, hint_text) == 0)
    {
        lv_obj_set_style_text_opa(input_text, LV_OPA_50, 0);
    }
    else
    {
        lv_obj_set_style_text_opa(input_text, LV_OPA_100, 0);
    }
}

static void light_sheet_border_while_speaking_or_not(bool speaking)
{
    if (speaking)
    {
        lv_obj_set_style_border_color(sheet, lv_color_hex(0x97CBFF), 0);
        lv_obj_set_style_border_width(sheet, 2, 0);
    }
    else
    {
        // lv_obj_set_style_border_color(sheet, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_border_width(sheet, 0, 0);
    }
}

static int get_text_line_count(lv_obj_t *label)
{
    const char *text = lv_label_get_text(label);
    const lv_font_t *font = lv_obj_get_style_text_font(label, LV_PART_MAIN);
    lv_coord_t letter_space =
        lv_obj_get_style_text_letter_space(label, LV_PART_MAIN);
    lv_coord_t label_width = lv_obj_get_width(label);
    lv_coord_t label_height = lv_obj_get_height(label);
    LOG_D("Label height: %d", label_height);
    lv_coord_t text_width = lv_txt_get_width(text, strlen(text), font,
                                             letter_space, LV_TEXT_FLAG_NONE);
    LOG_D("Text width: %d", text_width);
    // 計算行數
    int line_count = (text_width + 280) / (280); // 向上取整
    return line_count;
}

static lv_coord_t get_text_width_until_wrap(lv_obj_t *label)
{
    const char *text = lv_label_get_text(label);
    const lv_font_t *font = lv_obj_get_style_text_font(label, LV_PART_MAIN);
    lv_coord_t letter_space =
        lv_obj_get_style_text_letter_space(label, LV_PART_MAIN);
    lv_coord_t label_width = lv_obj_get_width(label);

    lv_coord_t text_width = 0;
    const char *ptr = text;
    while (*ptr != '\0')
    {
        lv_coord_t char_width = lv_font_get_glyph_width(font, *ptr, *(ptr + 1));
        text_width += char_width + letter_space;
        if (text_width > label_width)
        {
            break;
        }
        ptr++;
    }

    return text_width;
}

static app_speech_ripple_t app_speech_ripple[4];
static lv_obj_t *speech_button_builder(lv_obj_t *src, lv_event_cb_t event_cb)
{
    lv_obj_t *speech_button = common_flex_button(src, false, NULL);
    lv_obj_t *ripples =
        create_animate_ripples(app_speech_ripple, speech_button, 10);
    // lv_obj_add_event_cb(speech_button, event_cb, LV_EVENT_ALL, NULL);
    return speech_button;
}
static void unused_object_event_callback(lv_event_t *event)
{
    // LOG_D("unused_object_event_callback");
}

static void send_message(char *message)
{
    handle_user_speech_intent(V2T_INTENT_CHAT, message);
}
static void refresh_bottom_sheet(bool voice2text_status)
{
    if (voice2text_status)
    {
        // hide the mic button
        lv_obj_add_flag(mic_button, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(input_text, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        // show the mic button
        lv_obj_clear_flag(mic_button, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(input_text, LV_OBJ_FLAG_HIDDEN);
    }
}
/* Declare the label and background globally */
static void handle_mic_status(void *param)
{
    bool status = *(bool *)param;
    if (first_open_ai_window)
    {
        trigger_voice_ripple(app_speech_ripple, status);
    }
    refresh_bottom_sheet(status);
}

static void handle_vad_status(bool active)
{
    light_sheet_border_while_speaking_or_not(active);
}

static void handle_tap_event(void)
{
    voice_provider.start_v2t();
}

static void cont_event_callback(lv_event_t *event)
{
    handle_tap_event();
}

static void textfield_event_callback(lv_event_t *event)
{
    clearVoice2Text();
}

static lv_obj_t *app_bar_builder(lv_obj_t *parent)
{
    lv_obj_t *container_top = common_container(
        parent, 350, 70, unused_object_event_callback, lv_color_hex(0x000000));
    lv_obj_align(container_top, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(container_top, LV_OPA_80, 0);
    lv_obj_t *title = lv_label_create(container_top);
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(0)),
                               0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_label_set_text(title, "Skai AI");
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_t *speech_button = speech_button_builder(container_top, RT_NULL);
    // lv_obj_align(speech_button, LV_ALIGN_CENTER, 0, 10);
    return container_top;
}

static lv_obj_t *bottom_sheet_builder(lv_obj_t *parent)
{
    // lv_obj_t *text_bar = lv_obj_create(parent);
    sheet = common_container(parent, 300, 60, cont_event_callback,
                             lv_color_hex(0xFFFFFF));
    lv_obj_set_size(sheet, 300, 60); // 60
    lv_obj_align(sheet, LV_ALIGN_BOTTOM_MID, 0, -50);
    lv_obj_set_style_bg_color(sheet, lv_color_hex(0x5B5B5B), 0);
    lv_obj_set_style_radius(sheet, 50, 0);
    lv_obj_clear_flag(sheet, LV_OBJ_FLAG_SCROLLABLE);
    // text field
    input_text = lv_label_create(sheet);
    lv_obj_set_style_text_font(input_text,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(input_text, lv_color_white(), 0);
    lv_label_set_long_mode(input_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(input_text, 263);
    lv_obj_set_style_text_opa(input_text, LV_OPA_50, 0);
    lv_obj_align(input_text, LV_ALIGN_LEFT_MID, 25, 0);
    lv_obj_update_layout(input_text);
    refresh_input_message(hint_text);

    mic_button = lv_img_create(sheet);
    lv_img_set_src(mic_button, &icon_mic);
    lv_obj_align(mic_button, LV_ALIGN_CENTER, 0, 0);

    refresh_bottom_sheet(app_voice_get_voice2text_status());

    return sheet;
}

static void on_stop(void);
static void on_pause(void);

static lv_obj_t *lv_skai_message_list_layout_create(lv_obj_t *parent)
{
    lv_obj_t *p_window = lv_obj_create(parent);

    lv_obj_set_style_bg_opa(p_window, LV_OPA_0, 0);
    lv_obj_set_size(p_window, LV_HOR_RES, LV_VER_RES);
    // lv_obj_set_size(p_window, LV_HOR_RES, LV_VER_RES);
    lv_obj_add_flag(p_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(p_window, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_window, LV_DIR_VER);
    lv_obj_set_style_pad_ver(p_window, LV_VER_RES / 2, 0);

    return p_window;
}

static void handle_back(void)
{
    #ifdef BSP_USING_BLOC_V2T
    // const char *text = getVoice2TextResult()->text;
    const char *text = get_combined_voice2text();
    if (text)
    {
        if (strlen(text) > 0)
        {
            // strcpy(get_skai_message(0, false)->message, "");
            clearVoice2Text();
            refresh_input_message(hint_text);
            wait_for_message(app_speech_point, false);
            // refresh_list(p_app_skai->main_window, skai_message_count(),
            // false);
        }
        else
        {
            gui_app_self_exit();
        }
    }
    else
    #endif
    {
        gui_app_self_exit();
    }
}

// 修改 handle_back 函數定義
void first_open_ai_handle_back(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
    {
        handle_back();
    }
}

void first_open_ai_handle_tap_event(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
    {
        handle_tap_event();
    }
}
static lv_obj_t *speech_indicator_builder(lv_obj_t *src, lv_event_cb_t event_cb)
{
    lv_obj_t *speech_indicator = common_flex_button(src, false, NULL);
    lv_obj_t *ripples =
        create_animate_ripples(app_speech_ripple, speech_indicator, 10);
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
    lv_label_set_text(label, "say something...");
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)),
                               0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    return label;
}

static void main_speech_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        // toggle_voice_recognition();
        LOG_D("main_speech_event_callback");
    }
}

static lv_obj_t *lbl_content;
static void app_speech_main_init(lv_obj_t *parent)
{
    LOG_D("app_speech_main_init");
    first_open_ai_window = common_black_bg(parent);
    lv_obj_t *cont =
        common_container(first_open_ai_window, 420, 240,
                         unused_object_event_callback, lv_color_hex(0x000000));
    lv_obj_set_style_border_width(cont, 2, 0);
    lv_obj_set_style_border_color(cont, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(cont, LV_OPA_10, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_70, 0);
    lv_obj_set_style_radius(cont, 50, LV_PART_MAIN);
    lbl_content = text_field_builder(cont);
    app_speech_bind_content(lbl_content);
    app_speech_set_content((const uint8_t *)"say something...");

    lv_obj_t *speech_indicator = speech_indicator_builder(
        first_open_ai_window, main_speech_event_callback);
    lv_obj_align(speech_indicator, LV_ALIGN_TOP_MID, 0, 20);

    // create trash image button and send image button on the bottom of the
    // screen(left and right) lv_obj_t *icon_btn_trash =
    // common_icon_button(first_open_ai_window, ICON_TRASH,
    // first_open_ai_handle_back); lv_obj_align(icon_btn_trash,
    // LV_ALIGN_BOTTOM_LEFT, 30, -20);

    // lv_obj_t *send_button = common_icon_button(first_open_ai_window,
    // ICON_SAND, first_open_ai_handle_tap_event); lv_obj_align_to(send_button,
    // first_open_ai_window, LV_ALIGN_BOTTOM_RIGHT, -30, -20);
    LOG_D("app_speech_main_init end");
}

static void on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_skai);
    p_app_skai = (app_skai_t *)lv_mem_alloc(sizeof(app_skai_t));
    memset(p_app_skai, 0, sizeof(app_skai_t));
    setting_provider.set_power_save_mode(0);
    voice_provider.vad_init();

    p_app_skai->bg = common_black_bg(scr);

    p_app_skai->main_window = lv_skai_message_list_layout_create(scr);

    // Load and display chat history instead of just refreshing the message list
    display_chat_history(p_app_skai->main_window);

    p_app_skai->app_bar = app_bar_builder(scr);
    p_app_skai->bottom_sheet = bottom_sheet_builder(scr);
    lv_event_send(p_app_skai->main_window, LV_EVENT_SCROLL, NULL);
    // app_speech_main_init(scr);
}

static void on_resume(void)
{
    #ifdef BSP_USING_UI_HANDLER
    // lvgl_msg_handler.handle_mic = handle_mic_status;
    // lvgl_msg_handler.handle_vad_status = handle_vad_status;
    // lvgl_msg_handler.refresh_message_stream = refresh_message_list;
    // lvgl_msg_handler.handle_input_message = refresh_input_message;
    lvgl_msg_handler.handle_tap_event = handle_tap_event;
    lvgl_msg_handler.handle_back_event = handle_back;
    #endif
    app_voice_set_voice2text_intent(V2T_INTENT_CHAT);
    start_voice_recognition(V2T_INTENT_CHAT);
}

static void on_pause(void)
{
    if (first_open_ai_window)
    {
        unbind_app_speech_data();
        lv_obj_del(first_open_ai_window);
        first_open_ai_window = NULL;
    }
    stop_voice_recognition(V2T_INTENT_NOTHING);
    #ifdef BSP_USING_UI_HANDLER
    // lvgl_msg_handler.handle_mic = NULL;
    // lvgl_msg_handler.handle_vad_status = NULL;
    // lvgl_msg_handler.handle_input_message = NULL;
    // lvgl_msg_handler.refresh_message_stream = NULL;
    lvgl_msg_handler.handle_tap_event = NULL;
    lvgl_msg_handler.handle_back_event = NULL;
    #endif
}

static void on_stop(void)
{
    // unbind_app_speech_data();
    voice_provider.vad_deinit();
    setting_provider.set_power_save_mode(1);
    if (p_app_skai)
    {
        lv_obj_del(p_app_skai->bg);
        p_app_skai->bg = NULL;
        lv_obj_del(p_app_skai->main_window);
        p_app_skai->main_window = NULL;
        lv_obj_del(p_app_skai->app_bar);
        p_app_skai->app_bar = NULL;
        lv_obj_del(p_app_skai->bottom_sheet);
        p_app_skai->bottom_sheet = NULL;
        lv_mem_free(p_app_skai);
        p_app_skai = NULL;
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        lv_obj_t *scr = lv_scr_act();
        on_start(scr);
    }
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
    gui_app_regist_msg_handler(APP_ID_SKAI, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(skai_ai), IMG_LOGO, APP_ID_SKAI, app_main);

static void display_chat_history(lv_obj_t *list)
{
    // Clear current messages
    clear_skai_message_list(get_message_list(), skai_message_count_ptr());

    // Allocate space for up to 5 chat entries
    chat_history_entry_t entries[5];
    int count = get_recent_chat_history(entries, 5);

    if (count <= 0)
    {
        // No chat history found, show default message
        chat_t new_message;
        strcpy(new_message.message, "How can I help you today?");
        new_message.state = true;
        new_message.is_self = false;
        update_skai_message(get_message_list(), skai_message_count_ptr(),
                            new_message);
        refresh_list(list, *skai_message_count_ptr(), false);
        return;
    }

    // Add the chat history entries to the message list (from oldest to newest)
    for (int i = count - 1; i >= 0; i--)
    {
        // Add user message
        chat_t user_message;
        strcpy(user_message.message, entries[i].user_text);
        user_message.state = true;
        user_message.is_self = true;
        update_skai_message(get_message_list(), skai_message_count_ptr(),
                            user_message);

        // Add AI response
        chat_t ai_message;
        strcpy(ai_message.message, entries[i].ai_text);
        ai_message.state = true;
        ai_message.is_self = false;
        update_skai_message(get_message_list(), skai_message_count_ptr(),
                            ai_message);
    }

    // Refresh the UI to display the chat history
    refresh_list(list, *skai_message_count_ptr(), false);
}

#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/