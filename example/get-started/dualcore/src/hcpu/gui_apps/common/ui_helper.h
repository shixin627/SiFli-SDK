/**
 ******************************************************************************
 * @file   ui_helper.h
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
#ifndef UI_HELPER_H
#define UI_HELPER_H
#include "lvgl.h"
#include "watch_global_data.h"
#include "common_widget.h"
#include "bloc_calendar.h"
#ifdef __cplusplus
extern "C"
{
#endif

    extern void set_scroll_anim_time(bool init, uint16_t time);
    extern uint8_t get_system_font_size(int adjust);
    extern void calibrate_reverse_rotation(int16_t *x, int16_t *y);

    extern rt_int32_t clock_on_resume(void);
    extern rt_int32_t clock_on_pause(void);
    extern void clock_on_stop(void);
    extern rt_int32_t instruction_list_resume(void);
    extern rt_int32_t instruction_list_pause(void);
    extern rt_int32_t instruction_list_deinit(void);

    /* Custom instruction list API */
    extern void add_or_update_custom_instruction(const char *id, const char *title,
                                                  const char *trigger_type,
                                                  uint32_t interval_sec,
                                                  bool enabled, uint32_t version);
    extern void clear_custom_instructions(void);
    extern uint8_t get_custom_instruction_count(void);
    extern void set_custom_instruction_tap_cb(void (*cb)(const char *id, bool enabled));
    extern void refresh_custom_instructions(void);
    extern rt_int32_t notification_on_resume(void);
    extern rt_int32_t notification_on_pause(void);
    extern rt_int32_t notification_on_deinit(void);
    extern bool check_if_speech_interact(void);
    extern bool ui_helper_get_interact_mode(void);
    extern void ui_helper_set_interact_mode(bool flag);
    extern void watch_run_app_by_intent(AppIntent *appIntent);
    extern void watch_exit_app(char *app_id);
    extern void lvgl_set_global_keypad_enter_cmd(void);
    extern void lvgl_set_global_keypad_esc_cmd(void);

    extern bool is_at_home(void);
    extern bool is_at_instruction_list(void);
    extern bool is_at_message(void);
    extern bool is_at_widget(void);
    extern bool is_at_speech_interface(void);
    extern void check_is_at_instruction_list(void);
    extern void check_is_at_speech_interface(void);
    extern void check_is_at_mouse_mode(void);
    extern void check_is_at_message(void);
    extern void check_is_at_control_center(void);
    extern void check_is_at_home(void);
    extern void check_is_at_ai_interface(void);
    extern void check_is_at_widget(void);
    extern rt_tick_t get_last_refresh_input_message_tick(void);
    extern void set_last_refresh_input_message_tick(rt_tick_t tick);
    extern rt_tick_t get_last_refresh_ai_reply_message_tick(void);
    extern void set_last_refresh_ai_reply_message_tick(rt_tick_t tick);

    // ----- Widget builder -----
    extern lv_obj_t *lv_weather_widget_builder(lv_obj_t *parent);
    extern lv_obj_t *lv_weather_object_builder(lv_obj_t *parent, void *data);
    extern lv_obj_t *lv_calendar_object_builder(lv_obj_t *parent, void *data);
    extern lv_obj_t *lv_finance_object_builder(lv_obj_t *parent, void *data);
    extern lv_obj_t *lv_currency_conversion_object_builder(lv_obj_t *parent, void *data);

    // calendar
    extern calendar_model_t *get_on_coming_calendar_model(void);
    extern lv_obj_t *lv_calendar_widget_builder(lv_obj_t *parent, calendar_model_t *calendar_model);
    extern void calendar_today_refresh(void);

    extern const char *const icon_list[NOTIFICATION_APP_QUANTITY];
    extern const char *get_app_icon(uint8_t app_id);

    extern void ui_show_hint_toast(const char *hint, ...);

    void animate_to_prev_col_page(lv_obj_t *tv, uint8_t *current_page);
    void animate_to_next_col_page(lv_obj_t *tv, uint8_t *current_page, uint8_t max_page);

    extern void datac_send_data(datac_handle_t handle, uint16_t msg_id, uint8_t *data, uint16_t data_len);
    extern void screen_rotate_to_90_degree(void);
    extern void screen_rotate_back_to_original_direction(void);

    extern void log_touch_sample_rate(void);

    extern void display_status_bar_area(uint32_t idx, bool display);

    typedef struct
    {
        lv_obj_t *tap_obj_right;
        lv_obj_t *tap_obj_left;
        lv_obj_t *ungrab_obj;
        lv_obj_t *unknown_obj_right;
        lv_obj_t *unknown_obj_left;
        lv_obj_t *gesture_release_indicator;
        // lv_obj_t *open_watch_hint;
        lv_obj_t *speech_indicator;
        lv_obj_t *speech_bg;
        lv_obj_t *ai_prompt_border_wight;
        lv_obj_t *ai_prompt_border_blue;
        lv_obj_t *speech_input;
        // lv_obj_t *speech_interact_button;
        lv_obj_t *speech_skai_reply;
        lv_obj_t *speech_wait_indicator;
        lv_obj_t *ai_status_label;
        lv_obj_t *ai_tap_hint_bg;
        lv_obj_t *ai_icon_hint_bg;
        // lv_obj_t *flow_panel;
        lv_obj_t *ai_response_options;
        app_speech_ripple_t speech_ripple[4];
    } app_gesture_indicator_t;
    /// APP system interface
    extern app_gesture_indicator_t *gui_app_get_gesture_indicator(void);
    extern void toggle_tap_indicator(app_gesture_indicator_t *indicator, uint8_t gesture);
    extern void toggle_ungrab_indicator(app_gesture_indicator_t *indicator, uint8_t gesture);
    extern void unknown_indicator_create(app_gesture_indicator_t *indicator);
    extern void unknown_indicator_hidden(app_gesture_indicator_t *indicator);
    extern void voice_recognition_hint_bg_show(app_gesture_indicator_t *indicator);
    extern void wait_for_ai_reply_message(void);
    extern void voice_recognition_hint_hidden(app_gesture_indicator_t *indicator);
    extern void refresh_ai_reply_message(char *new_text);
    extern void hidden_speech_indicator(void);
    extern void voice_recognition_hint_builder(void *par, app_gesture_indicator_t *indicator);
    extern void ai_icon_hint_builder(void *par, app_gesture_indicator_t *indicator);
    extern void voice_recognition_hint_create(app_gesture_indicator_t *indicator);
    extern void quick_ai_hint_hidden(app_gesture_indicator_t *indicator);
    extern void show_speech_indicator(bool show);
    extern void show_ai_processing_indicator(bool show);
    extern void handle_download_progress_update(int progress);
    extern bool is_user_want_to_open_display_to_instruction_list(void);
    extern void set_user_want_to_open_display_to_instruction_list(bool state);

#ifdef __cplusplus
} /*extern "C"*/
#endif
#endif

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/