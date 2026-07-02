/**
 ******************************************************************************
 * @file   app_interact.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
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
#include <math.h>
#include <string.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"

#ifdef BSP_USING_BLOC
#include "bloc_motor.h"
#include "bloc_setting.h"
#include "bloc_peripheral.h"
#include "bloc_v2t.h"
#include "bloc_system_perception.h"
#include "bloc_skaiwalk.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif
#include "ui_helper.h"
#ifdef BSP_USING_COMMUNICATE
#include "communicate_protocol.h"
#endif
#ifdef BSP_USING_WATCH_SYS_CLIENT
#include "watch_sys_service.h"
#endif

#define DBG_TAG "app.system.interface"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(flow_hint);
LV_IMG_DECLARE(ripple_blue);

static app_gesture_indicator_t gesture_indicator;

void clean_ai_gesture_indicator(void)
{
    if (gesture_indicator.speech_bg)
    {
        gesture_indicator.speech_bg = NULL;
    }
    if (gesture_indicator.ai_tap_hint_bg)
    {
        gesture_indicator.ai_tap_hint_bg = NULL;
    }
    LOG_D("clean_ai_gesture_indicator");
}

#ifdef SHOW_UNKNOWN_GESTURE_INDICATOR
static rt_timer_t timer_unknown_light;
#endif

app_gesture_indicator_t *gui_app_get_gesture_indicator(void)
{
    return &gesture_indicator;
}
bool is_ai_interface_active(void)
{
    lv_obj_t *bg = gui_app_get_gesture_indicator()->speech_bg;
    return bg && !lv_obj_has_flag(bg, LV_OBJ_FLAG_HIDDEN);
}
static const char *get_tool_description(LangchainToolKey tool_key);
/* ============== Indicator Management ============== */
#ifdef SHOW_UNKNOWN_GESTURE_INDICATOR
LV_IMG_DECLARE(unknown_bg_left);
LV_IMG_DECLARE(unknown_bg_right);

static void unknown_indicator_hidden_timer_cb(void *param)
{
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_UNKNOWN_INDICATOR, .data.action = 0};
    lvgl_send_msg(msg);
}
void start_unknown_light_timer(void)
{
    if (!timer_unknown_light)
    {
        timer_unknown_light = rt_timer_create("timer_unknown_light", unknown_indicator_hidden_timer_cb,
                                              RT_NULL, rt_tick_from_millisecond(200),
                                              RT_TIMER_FLAG_ONE_SHOT);
        rt_timer_start(timer_unknown_light);
    }
    else
    {
        rt_timer_start(timer_unknown_light);
    }
}

void unknown_indicator_create(app_gesture_indicator_t *indicator)
{
    if (indicator->unknown_obj_right && indicator->unknown_obj_left)
    {
        if (lv_obj_has_flag(indicator->unknown_obj_right, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(indicator->unknown_obj_right, LV_OBJ_FLAG_HIDDEN);
        }
        if (lv_obj_has_flag(indicator->unknown_obj_left, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(indicator->unknown_obj_left, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_update_layout(indicator->unknown_obj_right);
        lv_obj_update_layout(indicator->unknown_obj_left);
        start_unknown_light_timer();
    }
    else
    {
        LOG_W("UNKNOWN_INDICATOR_RIGHT_LIGHT IS NULL");
    }
}

void unknown_indicator_builder(void *par, app_gesture_indicator_t *indicator)
{
    if (indicator->unknown_obj_right || indicator->unknown_obj_left)
    {
        return;
    }
    lv_obj_t *par_obj = (lv_obj_t *)par;
    lv_obj_t *unknown_bg_right_img = lv_img_create(par_obj);
    lv_img_set_src(unknown_bg_right_img, &unknown_bg_right);
    lv_obj_align(unknown_bg_right_img, LV_ALIGN_RIGHT_MID, 0, 0);
    indicator->unknown_obj_right = unknown_bg_right_img;
    lv_obj_add_flag(indicator->unknown_obj_right, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *unknown_bg_left_img = lv_img_create(par_obj);
    lv_img_set_src(unknown_bg_left_img, &unknown_bg_left);
    lv_obj_align(unknown_bg_left_img, LV_ALIGN_LEFT_MID, 0, 0);
    indicator->unknown_obj_left = unknown_bg_left_img;
    lv_obj_add_flag(indicator->unknown_obj_left, LV_OBJ_FLAG_HIDDEN);
}

void unknown_indicator_hidden(app_gesture_indicator_t *indicator)
{
    if (indicator->unknown_obj_right && lv_obj_is_valid(indicator->unknown_obj_right) &&
        !lv_obj_has_flag(indicator->unknown_obj_right, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(indicator->unknown_obj_right, LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(indicator->unknown_obj_right);
    }
    if (indicator->unknown_obj_left && lv_obj_is_valid(indicator->unknown_obj_left) &&
        !lv_obj_has_flag(indicator->unknown_obj_left, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(indicator->unknown_obj_left, LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(indicator->unknown_obj_left);
    }
}

#endif
#ifdef SHOW_TAP_GESTURE_INDICATOR
LV_IMG_DECLARE(tap_bg_right);
LV_IMG_DECLARE(tap_bg_left);
static inline void tap_indicator_create(app_gesture_indicator_t *indicator)
{
    if ((indicator->tap_obj_right && lv_obj_is_valid(indicator->tap_obj_right)) &&
        (indicator->tap_obj_left && lv_obj_is_valid(indicator->tap_obj_left)))
    {
        if (lv_obj_has_flag(indicator->tap_obj_right, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(indicator->tap_obj_right, LV_OBJ_FLAG_HIDDEN);
        }
        if (lv_obj_has_flag(indicator->tap_obj_left, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(indicator->tap_obj_left, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_update_layout(indicator->tap_obj_right);
        lv_obj_update_layout(indicator->tap_obj_left);
    }
    else
    {
        LOG_W("TAP_INDICATOR_LIGHT IS NULL");
    }
}
static inline void tap_indicator_hidden(app_gesture_indicator_t *indicator)
{
    if (indicator->tap_obj_right && lv_obj_is_valid(indicator->tap_obj_right) &&
        !lv_obj_has_flag(indicator->tap_obj_right, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(indicator->tap_obj_right, LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(indicator->tap_obj_right);
    }
    if (indicator->tap_obj_left && lv_obj_is_valid(indicator->tap_obj_left) &&
        !lv_obj_has_flag(indicator->tap_obj_left, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(indicator->tap_obj_left, LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(indicator->tap_obj_left);
    }
}
static app_gesture_indicator_t *ungrab_indicator;
static inline void ungrab_indicator_hidden(void)
{
    lv_obj_add_flag(ungrab_indicator->ungrab_obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_update_layout(ungrab_indicator->ungrab_obj);
}
static inline void ungrab_indicator_create(app_gesture_indicator_t *indicator)
{
    if (indicator->ungrab_obj && lv_obj_is_valid(indicator->ungrab_obj))
    {
        if (lv_obj_has_flag(indicator->ungrab_obj, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(indicator->ungrab_obj, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_update_layout(indicator->ungrab_obj);
    }
    else
    {
        LOG_W("UNGRAB_INDICATOR_LIGHT IS NULL");
    }
}
static void circle_anim_cb(void *var, int32_t value)
{
    lv_obj_t *obj = (lv_obj_t *)var;
    lv_obj_set_size(obj, value, value);
    lv_obj_align(obj, LV_ALIGN_CENTER, 0, -1);
    int16_t border_width = (478 - value) / 2;
    // LOG_D("border_width: %d", border_width);
    lv_obj_set_style_border_width(obj, border_width, LV_PART_MAIN | LV_STATE_DEFAULT);
}
static void circle_anim_ready_cb(lv_anim_t *a)
{
    LOG_D("circle_anim_ready_cb");
    // lv_obj_t *obj = a->var;
    // lv_obj_set_style_border_opa(obj, LV_OPA_100, LV_PART_MAIN | LV_STATE_DEFAULT);
    // lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    ungrab_indicator_hidden();
}
static void opacity_anim_cb(void *obj, int32_t value)
{
    lv_obj_t *o = obj;
    lv_obj_set_style_border_opa(o, value, LV_PART_MAIN);
}
static void create_circle_animation(lv_obj_t *obj, bool is_expand)
{
    // 創建動畫
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, circle_anim_cb);
    if (is_expand)
    {
        // 從小變大
        lv_anim_set_values(&a, 462, 478);
    }
    else
    {
        // 從大變小
        lv_anim_set_values(&a, 470, 462);
    }
    lv_anim_set_time(&a, 500);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    // lv_anim_set_ready_cb(&a, circle_anim_ready_cb);
    lv_anim_start(&a);
    // 透明度動畫
    lv_anim_t opa_anim;
    lv_anim_init(&opa_anim);
    lv_anim_set_var(&opa_anim, obj);
    lv_anim_set_exec_cb(&opa_anim, opacity_anim_cb);
    if (is_expand)
    {
        // 從透明變不透明
        lv_anim_set_values(&opa_anim, LV_OPA_0, LV_OPA_50);
    }
    else
    {
        // 從不透明變透明
        lv_anim_set_values(&opa_anim, LV_OPA_50, LV_OPA_0);
    }
    lv_anim_set_time(&opa_anim, 500);
    lv_anim_set_path_cb(&opa_anim, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&opa_anim, circle_anim_ready_cb);
    lv_anim_start(&opa_anim);
}
void ungrab_indicator_builder(void *par, app_gesture_indicator_t *indicator)
{
    lv_obj_t *par_obj = (lv_obj_t *)par;
    lv_obj_t *ungrab_obj = lv_obj_create(par_obj);
    lv_obj_clear_flag(ungrab_obj, LV_OBJ_FLAG_CLICKABLE); // Allow touch events to pass through
    lv_obj_align(ungrab_obj, LV_ALIGN_CENTER, 0, 0);
    indicator->ungrab_obj = ungrab_obj;
    lv_obj_set_size(ungrab_obj, 450, 450);
    lv_obj_set_style_bg_color(ungrab_obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ungrab_obj, LV_OPA_0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ungrab_obj, 250, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ungrab_obj, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ungrab_obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_flag(ungrab_obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ungrab_obj, LV_OBJ_FLAG_FLOATING);
    // create_circle_animation(indicator->ungrab_obj);
}
void toggle_ungrab_indicator(app_gesture_indicator_t *indicator, uint8_t gesture)
{
    if (gesture == 1) // press
    {
        ungrab_indicator = indicator;
        ungrab_indicator_create(indicator);
        create_circle_animation(indicator->ungrab_obj, false);
        // Create a timer to hide the ungrab indicator after 0.5 seconds
    }
    else if (gesture == 0) // release
    {
        ungrab_indicator_create(indicator);
        create_circle_animation(indicator->ungrab_obj, true);
    }
}
void tap_indicator_builder(void *par, app_gesture_indicator_t *indicator)
{
    lv_obj_t *par_obj = (lv_obj_t *)par;
    lv_obj_t *tap_bg_right_img = lv_img_create(par_obj);
    lv_img_set_src(tap_bg_right_img, &tap_bg_right);
    lv_obj_align(tap_bg_right_img, LV_ALIGN_RIGHT_MID, 0, 0);
    indicator->tap_obj_right = tap_bg_right_img;
    lv_obj_add_flag(indicator->tap_obj_right, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *tap_bg_left_img = lv_img_create(par_obj);
    lv_img_set_src(tap_bg_left_img, &tap_bg_left);
    lv_obj_align(tap_bg_left_img, LV_ALIGN_LEFT_MID, 0, 0);
    indicator->tap_obj_left = tap_bg_left_img;
    lv_obj_add_flag(indicator->tap_obj_left, LV_OBJ_FLAG_HIDDEN);
}
void toggle_tap_indicator(app_gesture_indicator_t *indicator, uint8_t gesture)
{
    if (gesture == 0) // release
    {
        tap_indicator_hidden(indicator);
    }
    else if (gesture == 1) // press
    {
        tap_indicator_create(indicator);
    }
}
#endif
#define USE_SPEECH_RECOGNITION_HINT
#ifdef USE_SPEECH_RECOGNITION_HINT

static bool is_on_speech_input_variable = false;
void is_on_speech_input(bool is_speech)
{
    is_on_speech_input_variable = is_speech;
    extern bool get_is_open_instruction_list_ai(void);
    if (get_is_open_instruction_list_ai())
    {
        extern void handle_ai_voice_btn_vad(bool speaking);
        handle_ai_voice_btn_vad(is_speech);
    }
}

static void open_ai_prompt_border(app_gesture_indicator_t *indicator, bool open)
{
    if (open && is_ai_interface_active())
    {
        // 顯示白色邊框
        if (indicator->ai_prompt_border_wight && lv_obj_is_valid(indicator->ai_prompt_border_wight) &&
            lv_obj_has_flag(indicator->ai_prompt_border_wight, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(indicator->ai_prompt_border_wight, LV_OBJ_FLAG_HIDDEN);
            lv_obj_update_layout(indicator->ai_prompt_border_wight);
        }
        // 顯示藍色邊框
        if (indicator->ai_prompt_border_blue && lv_obj_is_valid(indicator->ai_prompt_border_blue) &&
            lv_obj_has_flag(indicator->ai_prompt_border_blue, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(indicator->ai_prompt_border_blue, LV_OBJ_FLAG_HIDDEN);
            lv_obj_update_layout(indicator->ai_prompt_border_blue);
        }
    }
    else
    {
        // 隱藏白色邊框
        if (indicator->ai_prompt_border_wight && lv_obj_is_valid(indicator->ai_prompt_border_wight) &&
            !lv_obj_has_flag(indicator->ai_prompt_border_wight, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_add_flag(indicator->ai_prompt_border_wight, LV_OBJ_FLAG_HIDDEN);
            lv_obj_update_layout(indicator->ai_prompt_border_wight);
        }
        // 隱藏藍色邊框
        if (indicator->ai_prompt_border_blue && lv_obj_is_valid(indicator->ai_prompt_border_blue) &&
            !lv_obj_has_flag(indicator->ai_prompt_border_blue, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_add_flag(indicator->ai_prompt_border_blue, LV_OBJ_FLAG_HIDDEN);
            lv_obj_update_layout(indicator->ai_prompt_border_blue);
        }
    }
}

void open_ai_tap_hint_bg(bool open)
{
    LOG_D("open_ai_tap_hint_bg: %d", open);
    if (lv_obj_is_valid(gui_app_get_gesture_indicator()->ai_tap_hint_bg))
    {
        if (open)
        {
            lv_obj_clear_flag(gui_app_get_gesture_indicator()->ai_tap_hint_bg, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(gui_app_get_gesture_indicator()->ai_tap_hint_bg, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_update_layout(gui_app_get_gesture_indicator()->ai_tap_hint_bg);
    }
}

void voice_recognition_hint_hidden(app_gesture_indicator_t *indicator)
{
    if (indicator->speech_indicator && lv_obj_is_valid(indicator->speech_indicator) &&
        !lv_obj_has_flag(indicator->speech_indicator, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(indicator->speech_indicator, LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(indicator->speech_indicator);
    }
}

// Animation callback for background fade in
static void bg_fade_in_cb(void *var, int32_t value)
{
    lv_obj_t *obj = (lv_obj_t *)var;
    lv_obj_set_style_bg_opa(obj, value, LV_PART_MAIN);
}

void in_spire_AI(app_gesture_indicator_t *indicator)
{
    if (indicator->speech_bg && lv_obj_is_valid(indicator->speech_bg))
    {
        lv_anim_t bg_anim;
        lv_anim_init(&bg_anim);
        lv_anim_set_var(&bg_anim, indicator->speech_bg);
        lv_anim_set_exec_cb(&bg_anim, bg_fade_in_cb);
        lv_anim_set_values(&bg_anim, LV_OPA_80, LV_OPA_90);
        lv_anim_set_time(&bg_anim, 300);
        lv_anim_set_path_cb(&bg_anim, lv_anim_path_ease_in_out);
        lv_anim_start(&bg_anim);
    }
}

extern bool get_is_open_instruction_list_ai(void);
extern void set_skai_widget_input_text(const char *text);
void refresh_ai_chat_input_message(char *text)
{
    /* The @-conversation chat room's mic owns the transcript when it's up — show the live partial in
       its (transparent) input box and stop here, so it never bleeds into the launcher widgets
       (founder 2026-06-29). */
    extern bool chat_page_is_open(void);
    extern void chat_page_set_transcript(const char *text);
    if (chat_page_is_open())
    {
        chat_page_set_transcript(text);
        return;
    }
    /* Decide by current screen: if the right device page's skaibar is open, the
       recognised text belongs there, not the left instruction_list widget. */
    extern bool device_pager_skaibar_is_open(void);
    extern void device_pager_skaibar_say(const char *text);
    if (device_pager_skaibar_is_open())
    {
        if (text && text[0] != '\0' && strspn(text, " \t\n\r") < strlen(text))
            device_pager_skaibar_say(text);
        return;
    }
    if (gesture_indicator.speech_input)
    {
        // 只有當文字不為空且不是純空白字符時才更新
        if (text && text[0] != '\0' && strspn(text, " \t\n\r") < strlen(text))
        {
            lv_label_set_text(gesture_indicator.speech_input, text);
            // 實際語音內容使用100%透明度
            lv_obj_set_style_text_opa(gesture_indicator.speech_input, LV_OPA_100, 0);
        }
        else
        {
            // 如果是空字符串或純空白字符，保持顯示「聽取中...」
            lv_label_set_text(gesture_indicator.speech_input, LV_EXT_STR_GET_BY_KEY(listening_dots, "Listening..."));
            // 「聽取中...」使用50%透明度
            lv_obj_set_style_text_opa(gesture_indicator.speech_input, LV_OPA_90, 0);
        }
        lv_obj_update_layout(gesture_indicator.speech_input);
        set_last_refresh_input_message_tick(rt_tick_get());
    }
    if (get_is_open_instruction_list_ai()) // && get_is_open_instruction_list_ai() is_at_speech_interface() &&
    {
        LOG_D("refresh_ai_chat_input_message: %s", text);
        /* The visible instruction_list box is now a device_pager-style box whose
           text lives in s_voice_transcript_label (instruction_list_set_voice_transcript).
           set_skai_widget_input_text still runs to keep the (now hidden) builder's
           input_text_is_null state — the auto-close / send paths read it. */
        extern void instruction_list_set_voice_transcript(const char *text);
        if (text && text[0] != '\0' && strspn(text, " \t\n\r") < strlen(text))
        {
            set_skai_widget_input_text(text);
            instruction_list_set_voice_transcript(text);
        }
        else
        {
            set_skai_widget_input_text("");
            instruction_list_set_voice_transcript(LV_EXT_STR_GET_BY_KEY(listening, "Listening"));
        }
    }
    if (gesture_indicator.speech_skai_reply)
    {
        const char *current_text = lv_label_get_text(gesture_indicator.speech_skai_reply);
        if (current_text && current_text[0] != '\0')
        {
            lv_label_set_text(gesture_indicator.speech_skai_reply, "");
            lv_obj_update_layout(gesture_indicator.speech_skai_reply);
        }
    }
    if (lv_obj_get_style_bg_opa(gesture_indicator.speech_bg, LV_PART_MAIN) != LV_OPA_90)
    {
        in_spire_AI(&gesture_indicator);
    }
    LOG_D("Input message refreshed%d",get_is_open_instruction_list_ai());
}

static void update_ai_process_indicator(app_gesture_indicator_t *indicator, const char *message, bool is_active)
{
    // Show/hide AI status label with message
    if (indicator->ai_status_label && lv_obj_is_valid(indicator->ai_status_label))
    {
        if (is_active)
        {
            if (message != NULL)
            {
                lv_label_set_text(indicator->ai_status_label, message);
            }
            else
            {
                lv_label_set_text(indicator->ai_status_label, LV_EXT_STR_GET_BY_KEY(ai_processing, "AI Processing..."));
            }
            lv_obj_clear_flag(indicator->ai_status_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_update_layout(indicator->ai_status_label);
        }
        else
        {
            lv_obj_add_flag(indicator->ai_status_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_update_layout(indicator->ai_status_label);
        }
    }
}

/* Public wrapper that accepts a description string directly — used by the
   new KEY_AI_PROCESS_TOOLKIT protocol which sends the string over BLE. */
void update_ai_process_indicator_text(app_gesture_indicator_t *indicator,
                                      const char *message, bool is_active)
{
    update_ai_process_indicator(indicator, (char *)message, is_active);
}

static lv_timer_t *ai_thinking_timer = NULL;

static void reset_all_interface_states(void)
{
    // Clean up timers - just check if not NULL before deleting
    if (ai_thinking_timer)
    {
        lv_timer_del(ai_thinking_timer);
        ai_thinking_timer = NULL;
    }

#ifdef SHOW_UNKNOWN_GESTURE_INDICATOR
    if (timer_unknown_light)
    {
        rt_timer_stop(timer_unknown_light);
        rt_timer_delete(timer_unknown_light);
        timer_unknown_light = NULL;
    }
#endif

    // Reset gesture indicator states
    app_gesture_indicator_t *indicator = gui_app_get_gesture_indicator();
    if (indicator)
    {
// Hide and reset all indicator objects
#ifdef SHOW_UNKNOWN_GESTURE_INDICATOR
        unknown_indicator_hidden(indicator);
#endif
#ifdef SHOW_TAP_GESTURE_INDICATOR
        tap_indicator_hidden(indicator);
#endif

        // Stop any animations
        if (indicator->ai_prompt_border_wight && lv_obj_is_valid(indicator->ai_prompt_border_wight) &&
            indicator->ai_prompt_border_blue && lv_obj_is_valid(indicator->ai_prompt_border_blue))
        {
            lv_anim_del(indicator->ai_prompt_border_wight, NULL);
            lv_anim_del(indicator->ai_prompt_border_blue, NULL);
            LOG_D("AI prompt border animation stopped");
        }

        // Reset ripple state - speech_ripple is an array, so just call trigger_voice_ripple
        trigger_voice_ripple(indicator->speech_ripple, false);

        // Ensure all UI elements are properly hidden and reset
        if (indicator->speech_wait_indicator && lv_obj_is_valid(indicator->speech_wait_indicator))
        {
            lv_obj_add_flag(indicator->speech_wait_indicator, LV_OBJ_FLAG_HIDDEN);
        }

        if (indicator->speech_bg && lv_obj_is_valid(indicator->speech_bg) &&
            !lv_obj_has_flag(indicator->speech_bg, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_set_style_border_width(indicator->speech_bg, 6, LV_PART_MAIN);
        }

        if (indicator->ai_status_label && lv_obj_is_valid(indicator->ai_status_label))
        {
            lv_obj_add_flag(indicator->ai_status_label, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(indicator->ai_status_label, "");
        }
    }

    // Reset power save mode if needed
    // setting_provider.set_power_save_mode(1);

    LOG_D("All interface states have been reset");
}

// Timer callback for switching from spinner to AI thinking message
static void ai_thinking_timer_cb(lv_timer_t *timer)
{
    // Hide spinner and show AI thinking message
    if (gesture_indicator.speech_wait_indicator && lv_obj_is_valid(gesture_indicator.speech_wait_indicator))
    {
        lv_obj_add_flag(gesture_indicator.speech_wait_indicator, LV_OBJ_FLAG_HIDDEN);
    }
    if (gesture_indicator.speech_bg && lv_obj_is_valid(gesture_indicator.speech_bg))
    {
        lv_obj_set_style_border_width(gesture_indicator.speech_bg, 6, LV_PART_MAIN);
    }
    open_ai_prompt_border(&gesture_indicator, true);
    update_ai_process_indicator(&gesture_indicator, LV_EXT_STR_GET_BY_KEY(ai_thinking, "Thinking..."), true);

    // Delete the timer as it's one-shot
    lv_timer_del(timer);
    ai_thinking_timer = NULL;
}

void wait_for_ai_reply_message(void)
{
    voice_recognition_hint_hidden(&gesture_indicator);

    // First show spinner for send success animation
    if (gesture_indicator.speech_wait_indicator && lv_obj_is_valid(gesture_indicator.speech_wait_indicator))
    {
        lv_obj_clear_flag(gesture_indicator.speech_wait_indicator, LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(gesture_indicator.speech_wait_indicator);
    }
    if (gesture_indicator.speech_bg && lv_obj_is_valid(gesture_indicator.speech_bg))
    {
        lv_obj_set_style_border_width(gesture_indicator.speech_bg, 0, LV_PART_MAIN);
    }
    open_ai_prompt_border(&gesture_indicator, false);
    // Hide AI status label initially while showing spinner
    if (gesture_indicator.ai_status_label && lv_obj_is_valid(gesture_indicator.ai_status_label))
    {
        lv_obj_add_flag(gesture_indicator.ai_status_label, LV_OBJ_FLAG_HIDDEN);
    }

    // Create timer to switch to "AI思考中" after 1 second
    ai_thinking_timer = lv_timer_create(ai_thinking_timer_cb, 1000, NULL);
}

extern bool get_send_to_ai_again(void);
extern void set_send_to_ai_again(bool value);
void refresh_ai_reply_message(char *new_text)
{
    update_ai_process_indicator(&gesture_indicator, NULL, false);
    LOG_D("refresh_ai_reply_message");
    /* Route AI replies into the AI widget when the instruction-list AI is open. */
    if (get_is_open_instruction_list_ai())
    {
        set_send_to_ai_again(false);
        extern void append_skai_widget_ai_reply(const char *text);
        extern void on_ai_reply_started(void);
        on_ai_reply_started();
        append_skai_widget_ai_reply(new_text);
        set_last_refresh_ai_reply_message_tick(rt_tick_get());
        return;
    }
    if (gesture_indicator.speech_skai_reply)
    {
        const char *current_text = lv_label_get_text(gesture_indicator.speech_skai_reply);
        // Combine text
        size_t combined_length = strlen(current_text) + strlen(new_text) + 1;
        char *combined_text = (char *)malloc(combined_length);
        strcpy(combined_text, current_text);
        strcat(combined_text, new_text);
        // Update text
        lv_label_set_text(gesture_indicator.speech_skai_reply, combined_text);
        free(combined_text);
        // Align the reply text
        lv_obj_align_to(gesture_indicator.speech_skai_reply, gesture_indicator.speech_input, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
        lv_obj_set_style_pad_bottom(gesture_indicator.speech_skai_reply, 60, LV_PART_MAIN);
        lv_obj_update_layout(gesture_indicator.speech_skai_reply);
        set_last_refresh_ai_reply_message_tick(rt_tick_get());
        // lv_obj_scroll_to_view(gesture_indicator.speech_skai_reply, LV_ANIM_ON);
        lv_obj_scroll_to_y(gesture_indicator.speech_bg, 10000, LV_ANIM_ON);
    }
}

// Timer callback for delayed ripple trigger
static void ripple_trigger_timer_cb(lv_timer_t *timer)
{
    app_gesture_indicator_t *indicator = (app_gesture_indicator_t *)timer->user_data;
    trigger_voice_ripple(indicator->speech_ripple, true);
    lv_timer_del(timer);
}

// Animation callback for text fade in
static void text_fade_in_cb(void *var, int32_t value)
{
    lv_obj_t *obj = (lv_obj_t *)var;
    lv_obj_set_style_text_opa(obj, value, LV_PART_MAIN);
}

void voice_recognition_hint_create(app_gesture_indicator_t *indicator)
{
    LOG_D("voice_recognition_hint_create");
    // setting_provider.set_power_save_mode(0);
    // Show background with fade in animation
    if (indicator->speech_bg && lv_obj_is_valid(indicator->speech_bg) &&
        lv_obj_has_flag(indicator->speech_bg, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_clear_flag(indicator->speech_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_opa(indicator->speech_bg, LV_OPA_0, LV_PART_MAIN);
        lv_obj_update_layout(indicator->speech_bg);
        // Create fade in animation for background
        lv_anim_t bg_anim;
        lv_anim_init(&bg_anim);
        lv_anim_set_var(&bg_anim, indicator->speech_bg);
        lv_anim_set_exec_cb(&bg_anim, bg_fade_in_cb);
        lv_anim_set_values(&bg_anim, LV_OPA_0, LV_OPA_80);
        lv_anim_set_time(&bg_anim, 300);
        lv_anim_set_path_cb(&bg_anim, lv_anim_path_ease_in_out);
        lv_anim_start(&bg_anim);
    }
    if (indicator->speech_bg && lv_obj_is_valid(indicator->speech_bg))
    {
        lv_obj_update_layout(indicator->speech_bg);
    }
    open_ai_prompt_border(indicator, true);
    // Show input text with fade in animation
    if (indicator->speech_input && lv_obj_is_valid(indicator->speech_input) &&
        lv_obj_has_flag(indicator->speech_input, LV_OBJ_FLAG_HIDDEN))
    {
        if (lv_obj_has_flag(indicator->speech_input, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(indicator->speech_input, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_text_opa(indicator->speech_input, LV_OPA_0, LV_PART_MAIN);
        }
        lv_label_set_text(indicator->speech_input, LV_EXT_STR_GET_BY_KEY(listening_dots, "Listening..."));
        // LOG_D("voice_recognition_hint_create: set input text to 聽取中...");
        // 設置「聽取中...」的透明度為50%
        lv_obj_update_layout(indicator->speech_input);
        lv_obj_align(indicator->speech_input, LV_ALIGN_CENTER, 0, 0);
        // Create fade in animation for input text (delayed)
        lv_anim_t input_anim;
        lv_anim_init(&input_anim);
        lv_anim_set_var(&input_anim, indicator->speech_input);
        lv_anim_set_exec_cb(&input_anim, text_fade_in_cb);
        lv_anim_set_values(&input_anim, LV_OPA_0, LV_OPA_90);
        lv_anim_set_time(&input_anim, 250);
        lv_anim_set_delay(&input_anim, 150);
        lv_anim_set_path_cb(&input_anim, lv_anim_path_ease_in_out);
        lv_anim_start(&input_anim);
    }
    // Show speech button
    // if (indicator->speech_interact_button && lv_obj_is_valid(indicator->speech_interact_button) &&
    //     lv_obj_has_flag(indicator->speech_interact_button, LV_OBJ_FLAG_HIDDEN))
    // {
    //   lv_obj_clear_flag(indicator->speech_interact_button, LV_OBJ_FLAG_HIDDEN);
    //   lv_obj_update_layout(indicator->speech_interact_button);
    // }
    // Show reply text with fade in animation
    if (indicator->speech_skai_reply && lv_obj_is_valid(indicator->speech_skai_reply) &&
        lv_obj_has_flag(indicator->speech_skai_reply, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_clear_flag(indicator->speech_skai_reply, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_text_opa(indicator->speech_skai_reply, LV_OPA_0, LV_PART_MAIN);
        lv_obj_update_layout(indicator->speech_skai_reply);

        // Create fade in animation for reply text (delayed)
        lv_anim_t reply_anim;
        lv_anim_init(&reply_anim);
        lv_anim_set_var(&reply_anim, indicator->speech_skai_reply);
        lv_anim_set_exec_cb(&reply_anim, text_fade_in_cb);
        lv_anim_set_values(&reply_anim, LV_OPA_0, LV_OPA_100);
        lv_anim_set_time(&reply_anim, 250);
        lv_anim_set_delay(&reply_anim, 200);
        lv_anim_set_path_cb(&reply_anim, lv_anim_path_ease_in_out);
        lv_anim_start(&reply_anim);
    }
    if (indicator->speech_skai_reply && lv_obj_is_valid(indicator->speech_skai_reply))
    {
        lv_label_set_text(indicator->speech_skai_reply, "");
        lv_obj_update_layout(indicator->speech_skai_reply);
        lv_obj_align_to(indicator->speech_skai_reply, indicator->speech_input, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    }
    // Show speech indicator with ripple animation
    if (indicator->speech_indicator && lv_obj_has_flag(indicator->speech_indicator, LV_OBJ_FLAG_HIDDEN) && lv_obj_is_valid(indicator->speech_indicator))
    {
        lv_obj_clear_flag(indicator->speech_indicator, LV_OBJ_FLAG_HIDDEN);
        // Delay the ripple trigger to sync with other animations
        lv_timer_t *ripple_timer = lv_timer_create(ripple_trigger_timer_cb, 250, indicator);
        lv_timer_set_repeat_count(ripple_timer, 1);

        lv_obj_update_layout(indicator->speech_indicator);
    }
    if (indicator->speech_bg && lv_obj_is_valid(indicator->speech_bg))
    {
        // lv_obj_scroll_to_view(indicator->speech_input, LV_ANIM_ON);
        lv_obj_scroll_to_y(indicator->speech_bg, 0, LV_ANIM_ON);
    }

    update_ai_process_indicator(indicator, NULL, false);
    // if (indicator->flow_panel && lv_obj_is_valid(indicator->flow_panel) &&
    //     lv_obj_has_flag(indicator->flow_panel, LV_OBJ_FLAG_HIDDEN))
    // {
    //     LOG_D("voice_recognition_hint_create: showing flow panel");
    //     lv_obj_clear_flag(indicator->flow_panel, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_update_layout(indicator->flow_panel);
    // }
    open_ai_tap_hint_bg(false);
    is_on_speech_input_variable = false;

    // create_ai_prompt_border_anim(indicator);
    // Register message handlers
    // lvgl_msg_handler.handle_input_message = refresh_ai_chat_input_message;
    // lvgl_msg_handler.refresh_message_stream = refresh_ai_reply_message;
}

void quick_ai_hint_hidden(app_gesture_indicator_t *indicator)
{
    LOG_D("quick_ai_hint_hidden");
    char *user_text = lv_label_get_text(indicator->speech_input);
    char *ai_text = lv_label_get_text(indicator->speech_skai_reply);
    // if (user_text && ai_text && (user_text[0] != '\0' || ai_text[0] != '\0'))
    // {
    //   save_user_and_ai_chat(user_text, ai_text);
    // }
    // setting_provider.set_power_save_mode(1);
    // Hide speech indicator
    if (indicator->speech_indicator && lv_obj_is_valid(indicator->speech_indicator) &&
        !lv_obj_has_flag(indicator->speech_indicator, LV_OBJ_FLAG_HIDDEN))
    {
        trigger_voice_ripple(indicator->speech_ripple, false);
        lv_obj_add_flag(indicator->speech_indicator, LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(indicator->speech_indicator);
    }
    // Hide background
    if (indicator->speech_bg && lv_obj_is_valid(indicator->speech_bg) &&
        !lv_obj_has_flag(indicator->speech_bg, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(indicator->speech_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(indicator->speech_bg);
    }

    open_ai_prompt_border(indicator, false);
    // Hide input text
    if (indicator->speech_input && lv_obj_is_valid(indicator->speech_input) &&
        !lv_obj_has_flag(indicator->speech_input, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(indicator->speech_input, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(indicator->speech_input, "");
        lv_obj_update_layout(indicator->speech_input);
    }
    update_ai_process_indicator(indicator, NULL, false);
    // Hide reply text
    if (indicator->speech_skai_reply && lv_obj_is_valid(indicator->speech_skai_reply) &&
        !lv_obj_has_flag(indicator->speech_skai_reply, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(indicator->speech_skai_reply, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(indicator->speech_skai_reply, "");
        lv_obj_update_layout(indicator->speech_skai_reply);
    }
    open_ai_tap_hint_bg(false);
    extern void reset_skai_widget_input_text(void);
    reset_skai_widget_input_text();
    // Clear message handlers
    lvgl_msg_handler.handle_input_message = NULL;
    lvgl_msg_handler.refresh_message_stream = NULL;
    lvgl_msg_handler.handle_vad_status = NULL;
    reset_user_speaking_to_ai();

    // Reset all states when closing
    reset_all_interface_states();
    LOG_D("quick_ai_hint_hidden: reset all interface states");
    animate_to_home_from_ai_page();
    show_instruction_list_time(true);
    voice_recognition_hint_create(indicator);
}

void voice_recognition_hint_bg_show(app_gesture_indicator_t *indicator)
{
    // Show background with full opacity
    if (indicator->speech_bg && lv_obj_is_valid(indicator->speech_bg) &&
        lv_obj_get_style_bg_opa(indicator->speech_bg, LV_PART_MAIN) != LV_OPA_90)
    {
        lv_obj_set_style_bg_opa(indicator->speech_bg, LV_OPA_30, LV_PART_MAIN);
        lv_obj_update_layout(indicator->speech_bg);
    }
    // Show input text with full opacity
    if (indicator->speech_input && lv_obj_is_valid(indicator->speech_input) &&
        lv_obj_get_style_text_opa(indicator->speech_input, LV_PART_MAIN) != LV_OPA_100)
    {
        lv_obj_set_style_text_opa(indicator->speech_input, LV_OPA_100, LV_PART_MAIN);
        lv_obj_update_layout(indicator->speech_input);
    }
    // Show speech button with full opacity
    // if (indicator->speech_interact_button && lv_obj_is_valid(indicator->speech_interact_button) &&
    //     lv_obj_get_style_bg_opa(indicator->speech_interact_button, LV_PART_MAIN) != LV_OPA_100)
    // {
    //   lv_obj_set_style_bg_opa(indicator->speech_interact_button, LV_OPA_100, LV_PART_MAIN);
    //   lv_obj_update_layout(indicator->speech_interact_button);
    // }
    // Show reply text with full opacity
    if (indicator->speech_skai_reply && lv_obj_is_valid(indicator->speech_skai_reply) &&
        lv_obj_get_style_text_opa(indicator->speech_skai_reply, LV_PART_MAIN) != LV_OPA_100)
    {
        lv_obj_set_style_text_opa(indicator->speech_skai_reply, LV_OPA_100, LV_PART_MAIN);
        lv_obj_update_layout(indicator->speech_skai_reply);
    }
}
#endif

// Function to get Chinese description for LangchainToolKey
static const char *get_tool_description(LangchainToolKey tool_key)
{
    switch (tool_key)
    {
    case WeatherTool:
        return LV_EXT_STR_GET_BY_KEY(ai_querying_weather, "Querying weather...");
    case CalendarQueryTool:
        return LV_EXT_STR_GET_BY_KEY(ai_querying_schedule, "Querying schedule...");
    case CalendarCreateTool:
        return LV_EXT_STR_GET_BY_KEY(ai_creating_schedule, "Creating schedule...");
    case CreateNoteTool:
        return LV_EXT_STR_GET_BY_KEY(ai_creating_memo, "Creating memo...");
    case WebPageTool:
        return LV_EXT_STR_GET_BY_KEY(ai_accessing_web, "Accessing web...");
    case WebSearchTool:
        return LV_EXT_STR_GET_BY_KEY(ai_searching_web, "Searching the web...");
    case RagTool:
        return LV_EXT_STR_GET_BY_KEY(ai_querying_kb, "Querying knowledge...");
    case FinanceTool:
        return LV_EXT_STR_GET_BY_KEY(ai_querying_finance, "Querying finance...");
    case CurrencyConversionTool:
        return LV_EXT_STR_GET_BY_KEY(ai_converting_currency, "Converting currency...");
    case ImageAnalyzeTool:
        return LV_EXT_STR_GET_BY_KEY(ai_analyzing_image, "Analyzing image...");
    case UnknownTool:
    default:
        return LV_EXT_STR_GET_BY_KEY(ai_processing, "AI Processing...");
    }
}

/**
 * @brief Create finance object builder
 * @param parent Parent object
 * @param data Finance data pointer (finance_t*)
 * @return The created finance object
 */
lv_obj_t *lv_finance_object_builder(lv_obj_t *parent, void *data)
{
    if (!parent || !data)
    {
        LOG_E("Invalid parameters in lv_finance_object_builder");
        return NULL;
    }

    finance_t *finance = (finance_t *)data;

    // Create main container with rounded corners and subtle border
    lv_obj_t *finance_widget = parent;
    lv_obj_set_size(finance_widget, 360, 240);
    lv_obj_set_style_pad_all(finance_widget, 16, 0);

    // Create header section with stock code and name
    lv_obj_t *header_container = lv_obj_create(finance_widget);
    lv_obj_set_size(header_container, LV_PCT(100), 40);
    lv_obj_set_style_bg_opa(header_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(header_container, 0, 0);
    lv_obj_set_style_pad_all(header_container, 0, 0);
    lv_obj_align(header_container, LV_ALIGN_TOP_LEFT, 0, 0);

    // Stock code with elegant styling
    lv_obj_t *code_label = lv_label_create(header_container);
    lv_label_set_text_fmt(code_label, "%s", (finance->code[0] != '\0') ? finance->code : "-");
    lv_obj_set_style_text_font(code_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(code_label, lv_color_make(0x1E, 0x40, 0xAF), 0);
    lv_obj_set_style_text_letter_space(code_label, 1, 0);
    lv_obj_align(code_label, LV_ALIGN_LEFT_MID, 0, 0);

    // Stock name with refined typography
    lv_obj_t *name_label = lv_label_create(header_container);
    lv_label_set_text_fmt(name_label, "%s", (finance->name[0] != '\0') ? finance->name : "-");
    lv_obj_set_style_text_font(name_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(name_label, lv_color_make(0x47, 0x5A, 0x69), 0);
    lv_obj_align(name_label, LV_ALIGN_RIGHT_MID, 0, 0);

    // Create price section with improved layout
    lv_obj_t *price_container = lv_obj_create(finance_widget);
    lv_obj_set_size(price_container, LV_PCT(100), 60);
    lv_obj_set_style_bg_opa(price_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(price_container, 0, 0);
    lv_obj_set_style_pad_all(price_container, 0, 0);
    lv_obj_align_to(price_container, header_container, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 8);

    // Current price with prominent display
    lv_obj_t *price_label = lv_label_create(price_container);
    lv_label_set_text_fmt(price_label, "$%.2f", finance->price);
    lv_obj_set_style_text_font(price_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(price_label, lv_color_make(0x1F, 0x29, 0x3B), 0);
    lv_obj_set_style_text_letter_space(price_label, 1, 0);
    lv_obj_align(price_label, LV_ALIGN_LEFT_MID, 0, 0);

    // Price change with elegant color coding
    lv_obj_t *change_label = lv_label_create(price_container);
    bool is_positive = finance->change >= 0;
    lv_color_t change_color = is_positive ? lv_color_make(0x10, 0xB9, 0x81) : lv_color_make(0xEF, 0x44, 0x44);

    const char *change_icon = is_positive ? "↗" : "↘";
    lv_label_set_text_fmt(change_label, "%s %+.2f (%.1f%%)",
                          change_icon, finance->change,
                          (finance->price > 0) ? (finance->change / finance->price * 100) : 0);
    lv_obj_set_style_text_font(change_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(change_label, change_color, 0);
    lv_obj_align(change_label, LV_ALIGN_RIGHT_MID, 0, 0);

    // Create bottom section for additional info
    lv_obj_t *bottom_container = lv_obj_create(finance_widget);
    lv_obj_set_size(bottom_container, LV_PCT(100), 30);
    lv_obj_set_style_bg_opa(bottom_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(bottom_container, 0, 0);
    lv_obj_set_style_pad_all(bottom_container, 0, 0);
    lv_obj_align_to(bottom_container, price_container, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);

    // Volume with refined styling
    lv_obj_t *volume_label = lv_label_create(bottom_container);
    lv_label_set_text_fmt(volume_label, "Vol: %.0f", finance->volume);
    lv_obj_set_style_text_font(volume_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(volume_label, lv_color_make(0x94, 0xA3, 0xB8), 0);
    lv_obj_align(volume_label, LV_ALIGN_LEFT_MID, 0, 0);

    // Add a subtle accent line at the top
    lv_obj_t *accent_line = lv_obj_create(finance_widget);
    lv_obj_set_size(accent_line, LV_PCT(100), 2);
    lv_obj_set_style_bg_color(accent_line, lv_color_make(0x3B, 0x82, 0xF6), 0);
    lv_obj_set_style_bg_opa(accent_line, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(accent_line, 1, 0);
    lv_obj_align(accent_line, LV_ALIGN_TOP_LEFT, 0, 0);

    return finance_widget;
}

/**
 * @brief Create currency conversion object builder
 * @param parent Parent object
 * @param data Currency conversion data pointer (currency_conversion_t*)
 * @return The created currency conversion object
 */
lv_obj_t *lv_currency_conversion_object_builder(lv_obj_t *parent, void *data)
{
    if (!parent || !data)
    {
        LOG_E("Invalid parameters in lv_currency_conversion_object_builder");
        return NULL;
    }

    currency_conversion_t *cc = (currency_conversion_t *)data;

    // Create container
    lv_obj_t *cc_widget = parent;
    lv_obj_set_size(cc_widget, 360, 240);
    lv_obj_set_style_bg_opa(cc_widget, LV_OPA_0, 0);
    lv_obj_set_style_radius(cc_widget, 20, 0);
    lv_obj_set_style_pad_all(cc_widget, 16, 0);

    // Title
    lv_obj_t *title_label = lv_label_create(cc_widget);
    lv_label_set_text(title_label, LV_EXT_STR_GET_BY_KEY(exchange_rate, "Exchange Rate"));
    lv_obj_set_style_text_font(title_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(title_label, lv_color_make(0x40, 0x80, 0xFF), 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 12, 8);

    // From currency and amount
    lv_obj_t *from_label = lv_label_create(cc_widget);
    lv_label_set_text_fmt(from_label, "%s %.2f", cc->from_currency, cc->amount);
    lv_obj_set_style_text_font(from_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(from_label, lv_color_black(), 0);
    lv_obj_align_to(from_label, title_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

    // Exchange rate
    lv_obj_t *rate_label = lv_label_create(cc_widget);
    lv_label_set_text_fmt(rate_label, "1 %s = %.4f %s",
                          cc->from_currency, cc->exchange_rate, cc->to_currency);
    lv_obj_set_style_text_font(rate_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(rate_label, lv_color_make(0x99, 0x99, 0x99), 0);
    lv_obj_align_to(rate_label, from_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

    // To currency and converted amount
    lv_obj_t *to_label = lv_label_create(cc_widget);
    lv_label_set_text_fmt(to_label, "%s %.2f", cc->to_currency, cc->converted_amount);
    lv_obj_set_style_text_font(to_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(to_label, lv_color_black(), 0);
    lv_obj_align_to(to_label, rate_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

    // Result text (if available)
    if (cc->result[0] != '\0')
    {
        lv_obj_t *result_label = lv_label_create(cc_widget);
        lv_label_set_text_fmt(result_label, "%s", cc->result);
        lv_obj_set_style_text_font(result_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
        lv_obj_set_style_text_color(result_label, lv_color_make(0x99, 0x99, 0x99), 0);
        lv_obj_align_to(result_label, to_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    }

    return cc_widget;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/