/**
 ******************************************************************************
 * @file   app_note_chatroom_ui.c
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
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "watch_system_interact.h"
#include "watch_system_core_task.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "custom_trans_anim.h"
#include "bloc_skaiwalk.h" // For chat_t and note list functions
#include "ui_handler.h"
#include "ui_img_helper.h"
#include <time.h>

#ifdef BSP_USING_BLOC
#include "bloc_setting.h"
#include "bloc_control.h"
#include "bloc_skaiwalk.h"
#include "bloc_v2t.h"
#include "bloc_peripheral.h"
#include "bloc_motion_tracking.h"
#endif

#define DBG_TAG "app.skai.ai"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// Define this to enable input text field functionality
// #define USING_INPUT_TEXT

#ifdef APP_ID_NOTE_CHATROOM
typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *main_window;
    lv_obj_t *app_bar;
    lv_obj_t *bottom_sheet;
    lv_obj_t *botton_up;
    lv_obj_t *botton_down;
} app_note_chatroom_ui_t;

typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *note_label;
    lv_obj_t *note_attachment;
    lv_obj_t *note_time;
    lv_obj_t *widget_title;
} note_widget_t;

LV_IMG_DECLARE(icon_send);
LV_IMG_DECLARE(icon_stop);
LV_IMG_DECLARE(icon_record);

extern void app_speech_bind_title(lv_obj_t *target);
extern void app_speech_bind_content(lv_obj_t *target);
extern void app_speech_set_content(const uint8_t *title);
extern void unbind_app_speech_data(void);

#define LIST_NOTE_CHATROOM_UI_WIDTH (440)
#define LIST_NOTE_CHATROOM_UI_HEIGHT (100)
#define LIST_SKAI_SPACING (30)

#define LIST_RADIUS (1000)

#define LIST_SKAI_RADIUS (260)
#define LIST_SKAI_BORDER_SIDE LV_BORDER_SIDE_RIGHT

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define FONT_SIZE FONT_BIGL
#define LIST_ENABLE_ARC_SCROLLBAR 0

// Maximum height limits for note content to prevent widgets from becoming too large
#define MAX_AI_CONTENT_HEIGHT 100 // attachment content
#define MAX_WIDGET_HEIGHT 250     // Total widget height

/*
 * STATIC VARS DEFINITIONS
 *****************************************************************************************
 */

static app_note_chatroom_ui_t *p_app_note_chatroom_ui = NULL;

#ifdef USING_INPUT_TEXT
static char hint_text[] = "Say something";
static lv_obj_t *input_text;
#endif

static lv_obj_t *sheet;
static lv_obj_t *mic_button;
static lv_obj_t *recording_btn;
static lv_obj_t *recording_state;
static lv_obj_t *recording_time_label;
static lv_timer_t *recording_timer = NULL;
static time_t recording_start_time = 0;

note_widget_t p_note_widget;

// Detail page variables
static lv_obj_t *detail_page = NULL;
static chat_t *current_detail_message = NULL;

static const lv_style_const_prop_t LIST_NOTE_CHATROOM_UI_STYLE_PROPS[] = {
    LV_STYLE_CONST_WIDTH(LIST_NOTE_CHATROOM_UI_WIDTH),
    LV_STYLE_CONST_HEIGHT(LIST_NOTE_CHATROOM_UI_HEIGHT),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t LIST_NOTE_CHATROOM_UI_TITLE_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_14),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(LIST_NOTE_CHATROOM_UI_STYLE, LIST_NOTE_CHATROOM_UI_STYLE_PROPS);
LV_STYLE_CONST_INIT(LIST_NOTE_CHATROOM_UI_TITLE_STYLE, LIST_NOTE_CHATROOM_UI_TITLE_STYLE_PROPS);

static bool open_action_flag = true;
static bool left_hand_mode = true;
static uint16_t selected_message_index = 0;
static void scroll_list(lv_obj_t *obj, int16_t drift)
{
    lv_coord_t min_offset = LV_VER_RES;
    uint8_t child_cnt = obj->spec_attr->child_cnt;
    lv_coord_t y_diff = 0;
    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = obj->spec_attr->children[i];
        lv_coord_t y_center = child->coords.y1 + lv_obj_get_height(child) / 2;
        y_diff = y_center - LV_VER_RES / 2 + drift;

        y_diff = LV_ABS(y_diff);
        if (y_diff < min_offset)
        {
            min_offset = y_diff;
            selected_message_index = i;
        }
    }
}

static void list_window_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;

    // if (obj == NULL)
    // {
    //     return;
    // }
    if (evt->code == LV_EVENT_SCROLL)
    {
        scroll_list(obj, 0);
    }
}

// Close detail page and return to list
static void close_detail_page(void)
{
    if (detail_page != NULL)
    {
        lv_obj_del(detail_page);
        detail_page = NULL;
        current_detail_message = NULL;
    }
}

// Back button click handler for detail page
static void detail_back_button_cb(lv_event_t *evt)
{
    close_detail_page();
}

// Create and show detail page for a note
static void show_note_detail_page(chat_t *message)
{
    if (message == NULL || p_app_note_chatroom_ui == NULL)
    {
        return;
    }

    current_detail_message = message;

    // Create full-screen detail page
    detail_page = lv_obj_create(lv_scr_act());
    lv_obj_set_size(detail_page, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(detail_page, 0, 0);
    lv_obj_set_style_bg_color(detail_page, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(detail_page, LV_OPA_100, 0);
    lv_obj_set_style_border_width(detail_page, 0, 0);
    lv_obj_set_style_pad_all(detail_page, 0, 0);
    lv_obj_clear_flag(detail_page, LV_OBJ_FLAG_SCROLLABLE);

    // Title at top center
    lv_obj_t *title = lv_label_create(detail_page);
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_label_set_text(title, "Note Detail");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Create scrollable content area
    lv_obj_t *scroll_area = lv_obj_create(detail_page);
    lv_obj_set_size(scroll_area, LV_HOR_RES - 40, LV_VER_RES - 100);
    lv_obj_align(scroll_area, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_bg_opa(scroll_area, LV_OPA_0, 0);
    lv_obj_set_style_border_width(scroll_area, 0, 0);
    lv_obj_set_style_pad_all(scroll_area, 0, 0);
    lv_obj_set_scroll_dir(scroll_area, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(scroll_area, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(scroll_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scroll_area, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(scroll_area, 20, 0);

    // Main message content
    if (strlen(message->message) > 0)
    {
        lv_obj_t *content = lv_label_create(scroll_area);
        lv_obj_set_style_text_font(content, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_color(content, lv_color_white(), 0);
        lv_label_set_long_mode(content, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(content, LV_HOR_RES - 40);
        lv_label_set_text(content, message->message);
    }

    // Separator if there's attachment content
    if (strlen(message->attachment) > 0)
    {
        lv_obj_t *separator = lv_obj_create(scroll_area);
        lv_obj_set_size(separator, LV_HOR_RES - 40, 2);
        lv_obj_set_style_bg_color(separator, lv_color_hex(0x666666), 0);
        lv_obj_set_style_bg_opa(separator, LV_OPA_100, 0);
        lv_obj_set_style_border_width(separator, 0, 0);
        lv_obj_set_style_pad_all(separator, 0, 0);

        // AI attachment content (no height limit)
        lv_obj_t *ai_content = lv_label_create(scroll_area);
        lv_obj_set_style_text_font(ai_content, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_color(ai_content, lv_color_white(), 0);
        lv_obj_set_style_text_opa(ai_content, LV_OPA_80, 0);
        lv_label_set_long_mode(ai_content, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(ai_content, LV_HOR_RES - 40);
        lv_label_set_text(ai_content, message->attachment);
    }

    // Widget data if available
    if (strlen(message->widget_data) > 0)
    {
        lv_obj_t *ai_widget = lv_obj_create(scroll_area);
        parse_ai_reply_data((uint8_t *)message->widget_data, strlen(message->widget_data), ai_widget);
    }

    // Close button (X icon) at bottom center
    lv_obj_t *close_btn = lv_btn_create(detail_page);
    lv_obj_set_size(close_btn, 60, 60);
    lv_obj_align(close_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x404040), 0);
    lv_obj_set_style_bg_opa(close_btn, LV_OPA_90, 0);
    lv_obj_set_style_radius(close_btn, 30, 0);
    lv_obj_set_style_border_width(close_btn, 0, 0);
    lv_obj_add_event_cb(close_btn, detail_back_button_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *close_icon = lv_img_create(close_btn);
    lv_img_set_src(close_icon, ICON_X);
    lv_img_set_zoom(close_icon, 128); // 128 = 50% zoom (32px from 64px)
    lv_obj_center(close_icon);
}

static void list_message_click_event_cb(lv_event_t *evt)
{
    // Get the message data from the event user data
    chat_t *message = (chat_t *)evt->user_data;
    if (message != NULL)
    {
        show_note_detail_page(message);
    }
}

static app_speech_ripple_t app_speech_point[3];
static lv_coord_t single_line_height = 0;
static int bottom_of_previous_window = 0;

// Add timestamp to chat_t structure for notes
typedef struct
{
    bool state;
    char message[512];
    bool is_self;
    time_t timestamp; // Add timestamp field
} note_chat_t;

void add_to_self_note(char *text);
static void refresh_message_list(void);
static void set_input_textfield(char *text);

static void cont_event_callback(lv_event_t *event)
{
    if (!app_voice_get_voice2text_status())
    {
        // change_recording_icon(true);
        if (!get_bluetooth_connection_status())
        {
            create_connection_tips();
            LOG_D("Bluetooth is connected, ignoring voice recognition event");
            return;
        }
        voice_provider.vad_init();
        clearVoice2Text();
        start_voice_recognition(V2T_INTENT_NOTE_CREATING);
    }
    else
    {
        stop_voice_recognition(V2T_INTENT_NOTE_CREATING);
        char *text = get_combined_voice2text();
        if (strlen(text) > 0)
        {
            // change_recording_icon(false);
            add_to_self_note(text);
            // lv_obj_set_style_border_width(sheet, 0, 0);
            // clearVoice2Text();
        }
    }
}

#ifdef USING_INPUT_TEXT
static void refresh_input_message(char *text)
{
    // ckeck if input_text is NULL
    LOG_D("refresh_input_message: %s", text);
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

static void set_input_textfield(char *text)
{
    lv_label_set_text(input_text, text);
    lv_obj_update_layout(input_text);
}
#endif

static void update_recording_time(lv_timer_t *timer)
{
    static rt_uint32_t last_sec = 0;
    static rt_uint32_t cur_ms = 0;

    time_t now;
    time(&now);
    rt_uint32_t elapsed_sec = (rt_uint32_t)(now - recording_start_time);
    rt_uint32_t minutes = elapsed_sec / 60;
    rt_uint32_t seconds = elapsed_sec % 60;

    if (last_sec != seconds)
    {
        last_sec = seconds;
        cur_ms = 0;
    }
    else
    {
        cur_ms += 50;
    }

    char time_str[16];
    snprintf(time_str, sizeof(time_str), "%02lu:%02lu.%03lu", minutes, seconds, cur_ms);
    lv_label_set_text(recording_time_label, time_str);
}

static bool is_mic_open = false;
static void refresh_bottom_sheet(bool voice2text_status)
{
    is_mic_open = voice2text_status;
    if (voice2text_status)
    {
        // Recording mode: show stop icon on left, time label centered, hide input text
        set_paused_control_with_arm(true);

#ifdef USING_INPUT_TEXT
        // Hide input text
        lv_obj_add_flag(input_text, LV_OBJ_FLAG_HIDDEN);
#endif

        // Show recording time label (centered)
        lv_obj_clear_flag(recording_time_label, LV_OBJ_FLAG_HIDDEN);

        // Change to stop icon, move to left, and show it
        lv_img_set_src(mic_button, &icon_stop);
        lv_obj_align(mic_button, LV_ALIGN_LEFT_MID, 25, 0);
        lv_obj_clear_flag(mic_button, LV_OBJ_FLAG_HIDDEN);

        // Start recording timer
        time(&recording_start_time);
        lv_label_set_text(recording_time_label, "00:00.000");
        if (recording_timer == NULL)
        {
            recording_timer = lv_timer_create(update_recording_time, 50, NULL);
        }
    }
    else
    {
        // Normal mode: show record icon centered, hide recording time label and input text
        set_paused_control_with_arm(false);

#ifdef USING_INPUT_TEXT
        // Hide input text
        lv_obj_add_flag(input_text, LV_OBJ_FLAG_HIDDEN);
#endif
        // Hide recording time label
        lv_obj_add_flag(recording_time_label, LV_OBJ_FLAG_HIDDEN);

        // Change back to record icon, center it, and show it
        lv_img_set_src(mic_button, &icon_record);
        lv_obj_align(mic_button, LV_ALIGN_CENTER, 0, 0);
        lv_obj_clear_flag(mic_button, LV_OBJ_FLAG_HIDDEN);

        // Stop recording timer
        if (recording_timer != NULL)
        {
            lv_timer_del(recording_timer);
            recording_timer = NULL;
        }
    }
}

// Helper function to format time/date display
static void format_time_display(time_t timestamp, char *time_str, size_t max_len)
{
    time_t now = time(NULL);
    struct tm *current_time = localtime(&now);
    struct tm *message_time = localtime(&timestamp);

    if (current_time == NULL || message_time == NULL)
    {
        snprintf(time_str, max_len, "");
        return;
    }

    // Check if it's today
    if (current_time->tm_year == message_time->tm_year &&
        current_time->tm_yday == message_time->tm_yday)
    {
        // Today: show time in hh:mm format
        snprintf(time_str, max_len, "%02d:%02d",
                 message_time->tm_hour, message_time->tm_min);
    }
    else
    {
        // Yesterday or earlier: show date in mm/dd format
        snprintf(time_str, max_len, "%02d/%02d",
                 message_time->tm_mon + 1, message_time->tm_mday);
    }
}

static lv_obj_t *create_message_widget(lv_obj_t *list, chat_t *message)
{
    int offset = 0;
    lv_obj_t *message_widget = lv_obj_create(list);
    lv_obj_set_size(message_widget, LIST_NOTE_CHATROOM_UI_WIDTH, LIST_NOTE_CHATROOM_UI_HEIGHT);
    lv_obj_set_pos(message_widget, (LV_HOR_RES_MAX - LIST_NOTE_CHATROOM_UI_WIDTH) / 2 + offset, bottom_of_previous_window);
    lv_obj_add_flag(message_widget, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(message_widget, 25, LV_PART_MAIN);

    // Dark gray background with subtle transparency
    lv_obj_set_style_bg_color(message_widget, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(message_widget, LV_OPA_50, 0);

    // Add subtle border for depth
    lv_obj_set_style_border_color(message_widget, lv_color_hex(0x404040), 0);
    lv_obj_set_style_border_width(message_widget, 1, 0);

    lv_obj_add_event_cb(message_widget, list_message_click_event_cb, LV_EVENT_CLICKED, message);

    // Create content container with initial height
    lv_obj_t *content_container = lv_obj_create(message_widget);
    lv_obj_set_width(content_container, LIST_NOTE_CHATROOM_UI_WIDTH - 70);
    lv_obj_align(content_container, LV_ALIGN_TOP_MID, 15, 20);
    lv_obj_set_style_bg_opa(content_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(content_container, 0, 0);
    lv_obj_set_style_pad_all(content_container, 0, 0);

    // Main message content
    lv_obj_t *content = lv_label_create(content_container);
    lv_obj_set_style_text_font(content, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(content, lv_color_white(), 0);
    lv_label_set_long_mode(content, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(content, LIST_NOTE_CHATROOM_UI_WIDTH - 70);
    lv_label_set_text(content, message->message);
    lv_obj_align(content, LV_ALIGN_TOP_MID, 0, 0);

    // Force layout update to get actual content height
    lv_obj_update_layout(content);
    lv_coord_t content_height = lv_obj_get_height(content);
    // Update content container height based on content
    lv_obj_set_height(content_container, content_height);

    lv_coord_t ai_content_height = 0;

    // Only create AI response section if attachment is not empty
    if (strlen(message->attachment) > 0)
    {
        lv_obj_t *separator = lv_obj_create(message_widget);
        lv_obj_set_size(separator, LIST_NOTE_CHATROOM_UI_WIDTH - 70, 2);
        lv_obj_align(separator, LV_ALIGN_TOP_MID, 15, 15 + content_height);
        lv_obj_set_style_bg_color(separator, lv_color_hex(0x666666), 0);
        lv_obj_set_style_bg_opa(separator, LV_OPA_100, 0);
        lv_obj_set_style_border_width(separator, 0, 0);
        lv_obj_set_style_pad_all(separator, 0, 0);

        lv_obj_t *ai_content = lv_label_create(message_widget);
        lv_obj_align(ai_content, LV_ALIGN_TOP_MID, 15, 15 + content_height);
        lv_obj_set_style_text_font(ai_content, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_color(ai_content, lv_color_white(), 0);
        lv_obj_set_style_text_opa(ai_content, LV_OPA_80, 0);
        lv_label_set_long_mode(ai_content, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(ai_content, LIST_NOTE_CHATROOM_UI_WIDTH - 70);
        lv_label_set_text(ai_content, message->attachment);

        // Force layout update to get actual content height
        lv_obj_update_layout(ai_content);
        ai_content_height = lv_obj_get_height(ai_content);

        // Limit AI content height to prevent widget from becoming too large
        if (ai_content_height > MAX_AI_CONTENT_HEIGHT)
        {
            // Use CLIP mode to show only the first few lines
            lv_obj_set_height(ai_content, MAX_AI_CONTENT_HEIGHT);
            lv_label_set_long_mode(ai_content, LV_LABEL_LONG_CLIP);
            ai_content_height = MAX_AI_CONTENT_HEIGHT;
        }

        LOG_D("AI content height (limited): %d", ai_content_height);
    }

    if (strlen(message->widget_data) > 0)
    {
        LOG_D("Creating AI widget from data");
        lv_obj_t *ai_widget = lv_obj_create(message_widget);
        parse_ai_reply_data((uint8_t *)message->widget_data, strlen(message->widget_data), ai_widget);
        lv_obj_update_layout(ai_widget);
        ai_content_height += lv_obj_get_height(ai_widget);
        lv_obj_align(ai_widget, LV_ALIGN_TOP_MID, 15, 20 + content_height + ai_content_height);
    }

    // Time/date display
    lv_obj_t *time_label = lv_label_create(message_widget);
    lv_obj_set_style_text_font(time_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(time_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_opa(time_label, LV_OPA_70, 0);

    // Use message timestamp if available, otherwise use current time
    time_t message_timestamp = (message->timestamp != 0) ? message->timestamp : time(NULL);
    char time_str[16];
    format_time_display(message_timestamp, time_str, sizeof(time_str));
    lv_label_set_text(time_label, time_str);

    // Position time label at bottom right with padding
    lv_obj_align(time_label, LV_ALIGN_BOTTOM_RIGHT, -10, -5);

    // Calculate total widget height based on content and padding
    lv_coord_t padding = 15;
    lv_coord_t time_height = 20;
    lv_coord_t total_height = content_height + 2 * padding + time_height + ai_content_height;

    // Set a reasonable maximum height for the entire widget
    // Prevent widget from taking up too much screen space
    if (total_height > MAX_WIDGET_HEIGHT)
    {
        total_height = MAX_WIDGET_HEIGHT;
        LOG_D("Widget height capped at maximum: %d", MAX_WIDGET_HEIGHT);
    }

    // Update message widget height
    lv_obj_set_height(message_widget, total_height);

    // Update bottom position for next message
    bottom_of_previous_window = bottom_of_previous_window + total_height + LIST_SKAI_SPACING;

    return message_widget;
}

static lv_obj_t *bottom_sheet_builder(lv_obj_t *parent)
{
    sheet = common_container(parent, 300, 60, cont_event_callback, lv_color_hex(0x2A2A2A));
    // lv_obj_set_pos(sheet, (LV_HOR_RES_MAX - 460) / 2, bottom_of_previous_window + 20);
    lv_obj_set_style_bg_color(sheet, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_bg_opa(sheet, LV_OPA_90, 0);
    lv_obj_set_style_radius(sheet, 50, 0);
    lv_obj_align(sheet, LV_ALIGN_BOTTOM_MID, 0, -40);

    // Add subtle border for depth
    lv_obj_set_style_border_color(sheet, lv_color_hex(0x404040), 0);
    lv_obj_set_style_border_width(sheet, 1, 0);

    lv_obj_clear_flag(sheet, LV_OBJ_FLAG_SCROLLABLE);

#ifdef USING_INPUT_TEXT
    // text field
    input_text = lv_label_create(sheet);
    lv_obj_set_style_text_font(input_text, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(input_text, lv_color_white(), 0);
    lv_label_set_long_mode(input_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(input_text, 263);
    lv_obj_set_style_text_opa(input_text, LV_OPA_50, 0);
    lv_obj_align(input_text, LV_ALIGN_LEFT_MID, 25, 0);
    lv_obj_update_layout(input_text);
    refresh_input_message(hint_text);
#endif

    // Recording time label (hidden by default, centered)
    recording_time_label = lv_label_create(sheet);
    lv_obj_set_style_text_font(recording_time_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(recording_time_label, lv_color_white(), 0);
    lv_label_set_text(recording_time_label, "00:00.000");
    lv_obj_align(recording_time_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(recording_time_label, LV_OBJ_FLAG_HIDDEN);

    // Mic/recording button (centered by default, will move to left when recording)
    mic_button = lv_img_create(sheet);
    lv_img_set_src(mic_button, &icon_record);
    lv_obj_align(mic_button, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *sheet_obj_hitbox = lv_obj_create(parent);
    lv_obj_set_size(sheet_obj_hitbox, 466, 100);
    lv_obj_align(sheet_obj_hitbox, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_opa(sheet_obj_hitbox, LV_OPA_TRANSP, 0);
    lv_obj_add_event_cb(sheet_obj_hitbox, cont_event_callback, LV_EVENT_CLICKED, NULL);

    refresh_bottom_sheet(app_voice_get_voice2text_status());

    return sheet;
}

static void refresh_list(lv_obj_t *list, uint8_t new_item_count, bool wait_for_ai)
{
    /* Delete all children of the list */
    lv_obj_clean(list);
    bottom_of_previous_window = LIST_NOTE_CHATROOM_UI_HEIGHT + LIST_SKAI_SPACING;

    /* Repopulate the list with the new number of items */
    for (uint8_t i = 0; i < new_item_count; i++)
    {
        chat_t *message = get_skai_message(get_note_list(), *skai_note_count_ptr(), i, true);
        create_message_widget(list, message);
        if (p_note_widget.note_label == NULL)
            continue;
        if (i == new_item_count - 1)
        {
            lv_label_set_text(p_note_widget.note_label, message->message);
            lv_obj_align(p_note_widget.note_label, LV_ALIGN_CENTER, 0, 0);
            // Use message timestamp if available, otherwise use current time
            time_t message_timestamp = (message->timestamp != 0) ? message->timestamp : time(NULL);
            char time_str[16];
            format_time_display(message_timestamp, time_str, sizeof(time_str));
            if (p_note_widget.note_time != NULL)
            {
                lv_label_set_text(p_note_widget.note_time, time_str);
            }
        }
    }
}

static uint8_t page_count = 0;
static void refresh_message_list(void)
{
    uint16_t message_count = *skai_note_count_ptr();
    page_count = message_count;
    LOG_D("refresh_note_list note_count=%d", message_count);
    if (message_count > 0)
    {
        refresh_list(p_app_note_chatroom_ui->main_window, message_count, false);
    }
    set_scroll_segment_count(page_count);
    set_prev_sensor_quat(0);
    // bottom_sheet_builder(p_app_note_chatroom_ui->main_window);
    /* Scroll back to the selected item of the list */
    lv_obj_scroll_to_view(lv_obj_get_child(p_app_note_chatroom_ui->main_window, page_count - 1), LV_ANIM_OFF);
    // rt_kprintf("Scrolling app :%p,%d\n",lv_obj_get_child(p_app_note_chatroom_ui->main_window, selected_message_index - 1),lv_obj_is_visible(lv_obj_get_child(p_app_note_chatroom_ui->main_window, selected_message_index - 1)));
}

static void light_sheet_border_while_speaking_or_note(bool speaking)
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
    lv_coord_t letter_space = lv_obj_get_style_text_letter_space(label, LV_PART_MAIN);
    lv_coord_t label_width = lv_obj_get_width(label);
    lv_coord_t label_height = lv_obj_get_height(label);
    LOG_D("Label height: %d", label_height);
    lv_coord_t text_width = lv_txt_get_width(text, strlen(text), font, letter_space, LV_TEXT_FLAG_NONE);
    LOG_D("Text width: %d", text_width);
    // 計算行數
    int line_count = (text_width + 280) / (280); // 向上取整
    return line_count;
}

static lv_coord_t get_text_width_until_wrap(lv_obj_t *label)
{
    const char *text = lv_label_get_text(label);
    const lv_font_t *font = lv_obj_get_style_text_font(label, LV_PART_MAIN);
    lv_coord_t letter_space = lv_obj_get_style_text_letter_space(label, LV_PART_MAIN);
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
    lv_obj_t *ripples = create_animate_ripples(app_speech_ripple, speech_button, 10);
    // lv_obj_add_event_cb(speech_button, event_cb, LV_EVENT_ALL, NULL);
    return speech_button;
}
static void unused_object_event_callback(lv_event_t *event)
{
    // LOG_D("unused_object_event_callback");
}

static void send_message(char *message)
{
    // handle_user_speech_intent(V2T_INTENT_CHAT, message);
    handle_user_speech_intent(V2T_INTENT_NOTE_CREATING, message);
}

/* Declare the label and background globally */
static void handle_mic_status(void *param)
{
    bool status = *(bool *)param;
    refresh_bottom_sheet(status);
}

static void handle_vad_status(bool active)
{
    light_sheet_border_while_speaking_or_note(active);
}

static void change_recording_icon(bool recording)
{
    if (recording)
    {
        lv_obj_set_size(recording_state, 20, 20);
        lv_obj_align(recording_state, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_radius(recording_state, 5, 0);
    }
    else
    {
        lv_obj_set_size(recording_state, 30, 30);
        lv_obj_align(recording_state, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_radius(recording_state, 15, 0);
    }
}

static uint8_t button_selection_index = 1;

void add_to_self_note(char *text)
{
    // stop_voice_recognition(V2T_INTENT_NOTE_CREATING);
    if (get_note_list() != NULL)
    {
        // Add timestamp to the note (we'll store it in a way that works with existing structure)
        // For now, we'll use the existing append function and handle timestamp separately

        send_message(text);
    }
    else
    {
        LOG_D("get_note_list() is NULL");
    }
    refresh_message_list();
    clearVoice2Text();
}

static void handle_tap_event(uint8_t press)
{
    if (press != 1)
    {
        return; // Only handle press event
    }
    if (!app_voice_get_voice2text_status())
    {
        // change_recording_icon(true);
        // uint16_t message_count = *skai_note_count_ptr();
        // if (selected_message_index == message_count)
        // {
        if (!get_bluetooth_connection_status())
        {
            create_connection_tips();
            LOG_D("Bluetooth is connected, ignoring voice recognition event");
            return;
        }
        voice_provider.vad_init();
        clearVoice2Text();
#ifdef USING_INPUT_TEXT
        refresh_input_message(hint_text);
#endif
        start_voice_recognition(V2T_INTENT_NOTE_CREATING);
        // }
    }
    else
    {
        stop_voice_recognition(V2T_INTENT_NOTE_CREATING);
        char *text = get_combined_voice2text();
        if (strlen(text) > 0)
        {
            // change_recording_icon(false);
            add_to_self_note(text);
#ifdef USING_INPUT_TEXT
            refresh_input_message(hint_text);
            LOG_D("input_text: %s", lv_label_get_text(input_text));
#endif
            // lv_obj_set_style_border_width(sheet, 0, 0);
            // clearVoice2Text();
        }
    }
}

lv_obj_t *lv_note_widget_builder(lv_obj_t *parent)
{
    lv_obj_t *widget = common_widget_container(parent);
    lv_obj_clear_flag(widget, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(widget, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(widget, LV_OPA_50, 0);
    lv_obj_set_style_radius(widget, 25, LV_PART_MAIN);

    // Add subtle border for depth (matching create_message_widget style)
    lv_obj_set_style_border_color(widget, lv_color_hex(0x404040), 0);
    lv_obj_set_style_border_width(widget, 1, 0);

    lv_obj_t *widget_title = lv_label_create(widget);
    lv_obj_set_style_text_font(widget_title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(widget_title, lv_color_hex(0xB2A17D), 0);
    lv_label_set_text(widget_title, "備忘錄");
    lv_obj_align(widget_title, LV_ALIGN_TOP_MID, 0, 10);
    p_note_widget.widget_title = widget_title;

    lv_obj_t *note_bg = lv_obj_create(widget);
    lv_obj_set_size(note_bg, 400, 180);
    lv_obj_align(note_bg, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_clear_flag(note_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(note_bg, LV_OPA_0, 0);
    lv_obj_set_style_border_width(note_bg, 0, 0);

    lv_obj_t *note_label = lv_label_create(note_bg);
    lv_obj_set_style_text_font(note_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(note_label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_long_mode(note_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(note_label, LIST_NOTE_CHATROOM_UI_WIDTH - 70);
    lv_obj_set_height(note_label, 180 - 10);
    lv_obj_set_style_text_align(note_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(note_label, LV_ALIGN_CENTER, 0, 0);
    p_note_widget.note_label = note_label;

    // Time label at bottom right (matching create_message_widget style)
    lv_obj_t *note_time = lv_label_create(widget);
    lv_obj_set_style_text_font(note_time, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(note_time, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_opa(note_time, LV_OPA_70, 0);
    lv_obj_align(note_time, LV_ALIGN_BOTTOM_RIGHT, -10, -5);
    p_note_widget.note_time = note_time;

    int last_note_index = *skai_note_count_ptr() - 1;
    if (last_note_index >= 0)
    {
        chat_t *message = get_skai_message(get_note_list(), *skai_note_count_ptr(), last_note_index, true);
        if (strlen(message->message) > 0)
        {
            LOG_D("lv_note_widget_builder message: %s", message->message);
            lv_label_set_text(p_note_widget.note_label, message->message);
            // Use message timestamp if available, otherwise use current time
            time_t message_timestamp = (message->timestamp != 0) ? message->timestamp : time(NULL);
            char time_str[16];
            format_time_display(message_timestamp, time_str, sizeof(time_str));
            lv_label_set_text(note_time, time_str);
        }
        else
        {
            LOG_D("lv_note_widget_builder no message found, setting default text");
            lv_label_set_text(note_label, "No Note");
            lv_label_set_text(note_time, "");
        }
    }
    else
    {
        LOG_D("lv_note_widget_builder no notes available, setting default text");
        lv_label_set_text(note_label, "No Note");
        lv_label_set_text(note_time, "");
    }

    return widget;
}

static void refresh_new_note_widget(void)
{
    if (p_note_widget.note_label == NULL)
        return;
    int last_note_index = *skai_note_count_ptr() - 1;
    if (last_note_index >= 0)
    {
        chat_t *message = get_skai_message(get_note_list(), *skai_note_count_ptr(), last_note_index, true);
        lv_label_set_text(p_note_widget.note_label, message->message);
        // Use message timestamp if available, otherwise use current time
        time_t message_timestamp = (message->timestamp != 0) ? message->timestamp : time(NULL);
        char time_str[16];
        format_time_display(message_timestamp, time_str, sizeof(time_str));
        if (p_note_widget.note_time != NULL)
        {
            lv_label_set_text(p_note_widget.note_time, time_str);
        }
    }
}

static void textfield_event_callback(lv_event_t *event)
{
    clearVoice2Text();
}

static lv_obj_t *app_bar_builder(lv_obj_t *parent)
{
    lv_obj_t *container_top = common_container(parent, 350, 70, unused_object_event_callback, lv_color_hex(0x1A1A1A));
    lv_obj_align(container_top, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(container_top, LV_OPA_90, 0);
    lv_obj_set_style_radius(container_top, 15, 0);

    // Add subtle border for depth
    lv_obj_set_style_border_color(container_top, lv_color_hex(0x404040), 0);
    lv_obj_set_style_border_width(container_top, 1, 0);

    lv_obj_t *title = lv_label_create(container_top);
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_label_set_text(title, "Note");
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);

    return container_top;
}

static lv_obj_t *button_up(lv_obj_t *parent)
{
    lv_obj_t *button = lv_img_create(parent);
    lv_img_set_src(button, UP_ARROW);
    // lv_obj_align_to(button, p_app_note_chatroom_ui->app_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_align(button, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_img_opa(button, LV_OPA_40, 0);
    return button;
}

static lv_obj_t *button_down(lv_obj_t *parent)
{
    lv_obj_t *button = lv_img_create(parent);
    lv_img_set_src(button, DOWN_ARROW);
    lv_obj_align(button, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_img_opa(button, LV_OPA_40, 0);
    return button;
}

static rt_tick_t last_scroll_time = 0;
static void scroll_note_list_to_index(int8_t page)
{
    if (p_app_note_chatroom_ui == NULL || page_count == 0)
    {
        return;
    }
    extern void set_scroll_anim_time(bool init);
    rt_tick_t now = rt_tick_get();
    if (now - last_scroll_time < 100)
    {
        set_scroll_anim_time(false);
    }
    // LOG_D("scroll_note_list_to_index: %d,%d", page, page_count);
    last_scroll_time = now;
    lv_obj_scroll_to_view(lv_obj_get_child(p_app_note_chatroom_ui->main_window, page), LV_ANIM_ON);
    set_scroll_anim_time(false);
    lv_obj_update_layout(p_app_note_chatroom_ui->main_window);
}

static void button_selection(gesture_position_t gesture_position)
{
    if (peripheral_provider.get_tap_status())
        return;
    int category = (gesture_position.gesture_position_x < 116) ? 0 : ((gesture_position.gesture_position_x > 350) ? 2 : 1);
    if (category == button_selection_index)
    {
        return;
    }
    if ((selected_message_index == 0 && category == 0) || (selected_message_index == *skai_note_count_ptr() && category == 2))
    {
        // If the first message is selected and the up button is pressed, do nothing
        return;
    }
    button_selection_index = category;
    motor_pattern_scrolling_app();
    switch (category)
    {
    case 0:
        lv_obj_set_style_img_opa(p_app_note_chatroom_ui->botton_up, LV_OPA_100, 0);
        lv_obj_set_style_img_opa(p_app_note_chatroom_ui->botton_down, LV_OPA_40, 0);
        break;
    case 1:
        lv_obj_set_style_img_opa(p_app_note_chatroom_ui->botton_up, LV_OPA_40, 0);
        lv_obj_set_style_img_opa(p_app_note_chatroom_ui->botton_down, LV_OPA_40, 0);
        break;
    case 2:
        lv_obj_set_style_img_opa(p_app_note_chatroom_ui->botton_up, LV_OPA_40, 0);
        lv_obj_set_style_img_opa(p_app_note_chatroom_ui->botton_down, LV_OPA_100, 0);
        break;
    default:
        return;
    }
}

static void on_stop(void);
static lv_obj_t *lv_note_chatroom_ui_message_list_layout_create(lv_obj_t *parent)
{
    lv_obj_t *p_window = lv_obj_create(parent);

    lv_obj_set_style_bg_opa(p_window, LV_OPA_0, 0);
    lv_obj_set_size(p_window, LV_HOR_RES, LV_VER_RES);
    lv_obj_add_flag(p_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(p_window, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_window, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(p_window, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_pad_ver(p_window, LV_VER_RES / 2, 0);
    return p_window;
}

void note_list_handle_back(void)
{
#ifdef BSP_USING_BLOC_V2T
    // const char *text = getVoice2TextResult()->text;
    const char *text = get_combined_voice2text();
    if (text)
    {
        if (strlen(text) > 0)
        {
            // strcpy(get_note_chatroom_ui_message(0, false)->message, "");
            count_speech_coding();
            clearVoice2Text();
#ifdef USING_INPUT_TEXT
            refresh_input_message(hint_text);
#endif
            wait_for_message(app_speech_point, false);
            // refresh_list(p_app_note_chatroom_ui->main_window, note_chatroom_ui_message_count(), false);
        }
        else if (is_mic_open)
        {
            // gui_app_self_exit();
            stop_voice_recognition(V2T_INTENT_NOTHING);
        }
        else
        {
            animate_to_home_from_notification_center();
        }
    }
    else if (is_mic_open)
#endif
    {
        // gui_app_self_exit();
        stop_voice_recognition(V2T_INTENT_NOTHING);
    }
    else
    {
        animate_to_home_from_notification_center();
    }
}

void note_widget_start(void)
{
    lvgl_msg_handler.handle_new_note = refresh_new_note_widget;
}

void note_widget_stop(void)
{
    if (lvgl_msg_handler.handle_new_note == refresh_new_note_widget)
        lvgl_msg_handler.handle_new_note = NULL;
}

static void on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_note_chatroom_ui);
    p_app_note_chatroom_ui = (app_note_chatroom_ui_t *)lv_mem_alloc(sizeof(app_note_chatroom_ui_t));
    memset(p_app_note_chatroom_ui, 0, sizeof(app_note_chatroom_ui_t));
    setting_provider.set_power_save_mode(0);
    voice_provider.vad_init();
    // bloc_skai_message_init("How can I help you?");
    // app_speech_set_content((const uint8_t *)"say something...");
    // p_app_note_chatroom_ui->bg = common_black_bg(scr);
    p_app_note_chatroom_ui->main_window = lv_note_chatroom_ui_message_list_layout_create(scr);
    refresh_message_list();
    // p_app_note_chatroom_ui->app_bar = app_bar_builder(scr);
    // p_app_note_chatroom_ui->botton_up = button_up(scr);
    // p_app_note_chatroom_ui->botton_down = button_down(scr);
    // p_app_note_chatroom_ui->bottom_sheet = bottom_sheet_builder(p_app_note_chatroom_ui->main_window);
    bottom_sheet_builder(scr);
    lv_event_send(p_app_note_chatroom_ui->main_window, LV_EVENT_SCROLL, NULL);
    lv_obj_add_event_cb(p_app_note_chatroom_ui->main_window, list_window_scroll_event_cb, LV_EVENT_ALL, NULL);
    uint16_t message_count = *skai_note_count_ptr();
    selected_message_index = message_count - 1;
    lv_obj_scroll_to_view(lv_obj_get_child(p_app_note_chatroom_ui->main_window, selected_message_index), LV_ANIM_OFF);
    LOG_D("note_chatroom_on_start end");
    // cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
}

void note_list_on_resume(void)
{
    // if (get_need_open_gesture_control())
    {
        // switch_watch_motion_control_mode(true, false);
        set_paused_control_with_arm(false);
        set_free_control_with_arm(true);
    }
    set_scroll_segment_count(page_count);
    set_prev_sensor_quat(0);
    // set_open_control_options(true);

#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_mic = handle_mic_status;
    lvgl_msg_handler.handle_vad_status = handle_vad_status;
    lvgl_msg_handler.handle_note_list = refresh_message_list;
#ifdef USING_INPUT_TEXT
    lvgl_msg_handler.handle_input_message = refresh_input_message;
#endif
    lvgl_msg_handler.handle_tap_indicator = handle_tap_event;
    lvgl_msg_handler.handle_nav_bar_control = scroll_note_list_to_index;
    // lvgl_msg_handler.handle_back_event = handle_back;
    // lvgl_msg_handler.handle_widgets_control = button_selection;
#endif
}

void note_list_on_pause(void)
{
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_mic = NULL;
    lvgl_msg_handler.handle_vad_status = NULL;
    lvgl_msg_handler.handle_note_list = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
    if (lvgl_msg_handler.handle_nav_bar_control == scroll_note_list_to_index)
        lvgl_msg_handler.handle_nav_bar_control = NULL;
    // lvgl_msg_handler.handle_back_event = NULL;
    // lvgl_msg_handler.handle_widgets_control = NULL;
#endif
    set_paused_control_with_arm(true);
    set_open_control_options(false);
    setting_provider.set_power_save_mode(1);
}

static void on_stop(void)
{
    stop_voice_recording();
    stop_voice_recognition(V2T_INTENT_NOTHING);
    voice_provider.vad_deinit();
    setting_provider.set_power_save_mode(1);

    // Stop and clean up recording timer
    if (recording_timer != NULL)
    {
        lv_timer_del(recording_timer);
        recording_timer = NULL;
    }

    // Close detail page if open
    close_detail_page();

    // Save note list before stopping
    save_note_list_to_file();

    if (p_app_note_chatroom_ui)
    {
        lv_obj_del(p_app_note_chatroom_ui->bg);
        p_app_note_chatroom_ui->bg = NULL;
        lv_obj_del(p_app_note_chatroom_ui->main_window);
        p_app_note_chatroom_ui->main_window = NULL;
        lv_obj_del(p_app_note_chatroom_ui->app_bar);
        p_app_note_chatroom_ui->app_bar = NULL;
        lv_obj_del(p_app_note_chatroom_ui->bottom_sheet);
        p_app_note_chatroom_ui->bottom_sheet = NULL;
        lv_mem_free(p_app_note_chatroom_ui);
        p_app_note_chatroom_ui = NULL;
    }
}

lv_obj_t *open_note_chatroom_ui_app(lv_obj_t *parent)
{
    on_start(parent);
    return parent;
}

void close_note_chatroom_ui_app(void)
{
    on_stop();
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
        note_list_on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        note_list_on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

void quick_open_note_chatroom_ui_app(void)
{
    gui_app_create_page(APP_ID_NOTE_CHATROOM, msg_handler);
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_NOTE_CHATROOM, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(note_chatroom_ui), IMG_LOGO, APP_ID_NOTE_CHATROOM, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/