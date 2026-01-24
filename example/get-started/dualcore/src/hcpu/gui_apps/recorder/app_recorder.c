/**
 ******************************************************************************
 * @file   app_recorder.c
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
#include <time.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"

#include "app_mainmenu.h"
#include "custom_trans_anim.h"
#include "bloc_motion_tracking.h"
#include "common_widget.h"

#ifdef BSP_USING_BLOC
#include "bloc_v2t.h"
#include "bloc_setting.h"
#include "bloc_control.h"
#include "bloc_peripheral.h"
#include "bloc_filesystem.h"
#endif
#include "ui_helper.h"
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#define DBG_TAG "app.recorder"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_RECORDER

/* Constants and definitions */
#define BASE_SIZE 60
#define CIRCLE_SIZE (2 * BASE_SIZE)
#define WIDGET_WIDTH 200
#define WIDGET_HEIGHT 150

/* Type definitions */
typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *tileview;    // Main tileview container
    lv_obj_t *record_tile; // Top tile for record view
    lv_obj_t *list_tile;   // Bottom tile for file list
    lv_obj_t *scroll_hint; // List of recorded files
    lv_obj_t *file_list;   // List of recorded files
    lv_obj_t *record_button;
    lv_obj_t *record_icon;
    lv_obj_t *record_time_label;
    lv_obj_t *direction_icon[2]; // Icons for left/right direction
    lv_timer_t *record_timer;
    uint32_t record_time;
    time_t record_start_time; // 改為記錄RTC時間
} app_recorder_t;

typedef struct
{
    lv_obj_t *widget_record_button;
    lv_obj_t *widget_record_time_label;
    lv_timer_t *widget_record_timer;
    uint32_t widget_record_time;
    time_t widget_record_start_time; // 改為記錄RTC時間
} widget_recorder_t;

/* Static variables */
static widget_recorder_t *p_widget_recorder = NULL;
static app_recorder_t *p_app_recorder = NULL;
static lv_obj_t *sync_button = NULL;
static lv_obj_t *loading = NULL;
static folder_t *folder;
static char pending_auto_sync_file[MAX_RECORD_PATH_LEN] = {0};

/* Function declarations */
static void toggle_record_btn_event_cb(lv_event_t *e);
static void set_record_button_style(lv_obj_t *button, bool is_recording);
static lv_obj_t *create_recorder_file_list(lv_obj_t *scr, folder_t *folder);
static void file_click_event_handler(lv_event_t *e);
static void file_swipe_event_handler(lv_event_t *e);
static lv_obj_t *create_loading_label(lv_obj_t *parent, const char *text);
static void refresh_folder_listview(void);
static void handle_tap_event(uint8_t press);
static void handle_vad_status(bool user_talking);
static void handle_file_sync(bool state);
static void handle_sync_progress(uint8_t progress);
static void ui_sync_file(char *file_name, lv_obj_t *parent);

/* UI style management functions */
static void set_record_button_style(lv_obj_t *button, bool is_recording)
{
    lv_obj_clean(button);

    if (is_recording)
    {
        // Create stop button (square shape)
        lv_obj_t *inner_rectangle = lv_obj_create(button);
        lv_obj_set_size(inner_rectangle, BASE_SIZE, BASE_SIZE);
        lv_obj_set_style_radius(inner_rectangle, 20, 0);
        lv_obj_set_style_bg_color(inner_rectangle, lv_color_hex(0xFF0000), 0);
        lv_obj_center(inner_rectangle);
        lv_obj_clear_flag(inner_rectangle, LV_OBJ_FLAG_CLICKABLE);
    }
    else
    {
        // Create record button (circle shape)
        lv_obj_t *inner_circle = lv_obj_create(button);
        lv_obj_set_size(inner_circle, CIRCLE_SIZE, CIRCLE_SIZE);
        lv_obj_set_style_radius(inner_circle, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(inner_circle, lv_color_hex(0xFF0000), 0);
        lv_obj_center(inner_circle);
        lv_obj_clear_flag(inner_circle, LV_OBJ_FLAG_CLICKABLE);

        // Set border style
        lv_obj_set_style_border_color(button, lv_color_hex(0x808080), 0);
        lv_obj_set_style_border_width(button, 5, 0);
    }
}

static lv_obj_t *create_loading_label(lv_obj_t *parent, const char *text)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);

    // Set label style
    lv_obj_set_style_bg_opa(label, LV_OPA_50, 0);
    lv_obj_set_style_bg_color(label, lv_color_black(), 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_pad_all(label, 10, 0);
    lv_obj_align(label, LV_ALIGN_RIGHT_MID, 0, 0);

    return label;
}

/* UI creation functions */
lv_obj_t *create_record_view(lv_obj_t *parent)
{
    // Create a container for record view
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE); // Disable internal scrolling

    // Create record button with outer circle
    lv_obj_t *outer_circle = lv_obj_create(container);
    lv_obj_set_size(outer_circle, CIRCLE_SIZE + 10, CIRCLE_SIZE + 10);
    lv_obj_set_style_radius(outer_circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(outer_circle, lv_color_hex(0x808080), 0);
    lv_obj_set_style_bg_opa(outer_circle, LV_OPA_50, 0);
    lv_obj_set_style_border_color(outer_circle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(outer_circle, 5, 0);
    lv_obj_set_style_border_opa(outer_circle, LV_OPA_100, 0);
    lv_obj_set_style_bg_opa(outer_circle, LV_OPA_TRANSP, 0);
    lv_obj_center(outer_circle);
    lv_obj_add_event_cb(outer_circle, toggle_record_btn_event_cb, LV_EVENT_ALL, 0);

    // Set initial button style and store reference
    set_record_button_style(outer_circle, false);
    p_app_recorder->record_button = outer_circle;

    // Create timer label
    lv_obj_t *record_time_label = lv_label_create(container);
    lv_obj_set_style_text_font(record_time_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(record_time_label, lv_color_white(), 0);
    lv_label_set_text(record_time_label, "tap to record");
    lv_obj_align(record_time_label, LV_ALIGN_BOTTOM_MID, 0, -70);
    p_app_recorder->record_time_label = record_time_label;

    // Add hint to scroll down
    lv_obj_t *scroll_hint = lv_label_create(container);
    lv_label_set_text(scroll_hint, "swipe up for recordings ▼");
    lv_obj_set_style_text_font(scroll_hint, LV_EXT_FONT_GET(get_system_font_size(-3)), 0);
    lv_obj_set_style_text_color(scroll_hint, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(scroll_hint, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_text_opa(scroll_hint, LV_OPA_70, 0);
    p_app_recorder->scroll_hint = scroll_hint;

    return container;
}

static uint8_t button_selection_index = 0;
static lv_obj_t *create_recorder_file_list(lv_obj_t *parent, folder_t *folder)
{
    // Create a container for the list view
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_HOR_RES, LV_VER_RES - 40); // Leave space for header
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE); // Disable internal scrolling

    // Add header/title
    lv_obj_t *header = lv_label_create(container);
    lv_label_set_text(header, "Recordings");
    lv_obj_set_style_text_font(header, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(header, lv_color_white(), 0);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 10);

    // Add hint to scroll up
    lv_obj_t *scroll_hint = lv_label_create(container);
    lv_label_set_text(scroll_hint, "▲ swipe down to record");
    lv_obj_set_style_text_font(scroll_hint, LV_EXT_FONT_GET(get_system_font_size(-3)), 0);
    lv_obj_set_style_text_color(scroll_hint, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(scroll_hint, LV_ALIGN_TOP_MID, 0, 40);

    // Create list container
    lv_obj_t *list = lv_list_create(container);
    lv_obj_set_size(list, lv_pct(100), lv_pct(70));
    lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_pad_row(list, 5, 0);
    lv_obj_set_style_bg_color(list, lv_color_black(), 0);
    p_app_recorder->file_list = list;

    // Get sync progress information
    sync_progress_t *progress = get_sync_progress();

    if (folder->file_num == 0)
    {
        lv_obj_t *no_files = lv_label_create(container);
        lv_label_set_text(no_files, "No recordings yet");
        lv_obj_set_style_text_color(no_files, lv_color_hex(0xAAAAAA), 0);
        lv_obj_center(no_files);
        return container;
    }

    // Populate list with files
    for (int i = 0; i < folder->file_num; i++)
    {
        // Create button for file item
        lv_obj_t *btn = lv_btn_create(list);
        lv_obj_set_width(btn, lv_pct(100));
        lv_obj_set_height(btn, 200);
        lv_obj_add_style(btn, (lv_style_t *)&CUSTOM_BUTTON_STYLE, 0);

        // Get file name and create full path
        char *file_name = folder->files[i].name;
        char path[MAX_RECORD_PATH_LEN];
        snprintf(path, sizeof(path), "/recorder/%s", file_name);

        // Handle syncing state
        if (progress->sync_status && strcmp(progress->sync_in_file_path, path) == 0)
        {
            // Show sync progress for this file
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x97CBFF), 0);
            uint8_t percent = progress->percent_complete;
            char text[16];
            snprintf(text, sizeof(text), "Syncing...%d%%", percent);
            loading = create_loading_label(btn, text);
            sync_button = btn;
        }
        else
        {
            // Regular file button
            lv_obj_add_event_cb(btn, file_click_event_handler, LV_EVENT_CLICKED, NULL);
        }

        // Add file name label
        lv_obj_t *lab = lv_label_create(btn);
        lv_label_set_text_fmt(lab, "%s", file_name);
        lv_obj_align(lab, LV_ALIGN_TOP_LEFT, -20, -7);

        // Add swipe to delete functionality
        lv_obj_add_event_cb(btn, file_swipe_event_handler, LV_EVENT_GESTURE, NULL);
        if (button_selection_index == i)
        {
            lv_obj_set_style_border_width(btn, 4, 0);
            lv_obj_set_style_border_color(btn, lv_color_hex(0x97CBFF), 0);
            lv_obj_set_style_border_opa(btn, LV_OPA_100, 0);
            lv_obj_scroll_to_view(btn, LV_ANIM_ON);
        }
    }

    p_app_recorder->direction_icon[0] = lv_img_create(parent);
    lv_img_set_src(p_app_recorder->direction_icon[0], UP_ARROW);
    lv_obj_align(p_app_recorder->direction_icon[0], LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_img_opa(p_app_recorder->direction_icon[0], LV_OPA_30, 0);
    p_app_recorder->direction_icon[1] = lv_img_create(parent);
    lv_img_set_src(p_app_recorder->direction_icon[1], DOWN_ARROW);
    lv_obj_align(p_app_recorder->direction_icon[1], LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_img_opa(p_app_recorder->direction_icon[1], LV_OPA_30, 0);

    return container;
}

static uint32_t last_seconds;
static uint32_t current_milliseconds;
/* Timer callback functions */
static void update_record_time(lv_timer_t *timer)
{
    p_app_recorder->record_time++;
    time_t now;
    time(&now);
    uint32_t elapsed_sec = (uint32_t)(now - p_app_recorder->record_start_time);
    uint32_t minutes = elapsed_sec / 60;
    uint32_t seconds = elapsed_sec % 60;
    if (last_seconds != seconds)
    {
        last_seconds = seconds;
        current_milliseconds = 0;
    }
    else
    {
        current_milliseconds += 50;
    }
    // 若仍需顯示ms，可混合tick與RTC
    char time_str[12];
    snprintf(time_str, sizeof(time_str), "%02d:%02d.%03d", minutes, seconds, current_milliseconds);
    lv_label_set_text(p_app_recorder->record_time_label, time_str);
}

static void update_widget_record_time(lv_timer_t *timer)
{
    p_widget_recorder->widget_record_time++;
    time_t now;
    time(&now);
    uint32_t elapsed_sec = (uint32_t)(now - p_widget_recorder->widget_record_start_time);
    uint32_t minutes = elapsed_sec / 60;
    uint32_t seconds = elapsed_sec % 60;
    if (last_seconds != seconds)
    {
        last_seconds = seconds;
        current_milliseconds = 0;
    }
    else
    {
        current_milliseconds += 50;
    }
    char time_str[12];
    snprintf(time_str, sizeof(time_str), "%02d:%02d.%03d", minutes, seconds, current_milliseconds);
    lv_label_set_text(p_widget_recorder->widget_record_time_label, time_str);
}

/* Recording management functions */
static void start_to_record_voice(void)
{
    start_voice_recording();
    set_record_button_style(p_app_recorder->record_button, true);
    p_app_recorder->record_time = 0;
    time(&p_app_recorder->record_start_time);
    lv_label_set_text(p_app_recorder->record_time_label, "00:00");
    p_app_recorder->record_timer = lv_timer_create(update_record_time, 50, NULL); // 0.05秒刷新
    lv_obj_set_tile_id(p_app_recorder->tileview, 0, 0, LV_ANIM_ON);
}

static void start_widget_to_record_voice(void)
{
    start_voice_recording();
    set_record_button_style(p_widget_recorder->widget_record_button, true);
    p_widget_recorder->widget_record_time = 0;
    time(&p_widget_recorder->widget_record_start_time);
    lv_label_set_text(p_widget_recorder->widget_record_time_label, "00:00");
    p_widget_recorder->widget_record_timer = lv_timer_create(update_widget_record_time, 50, NULL); // 0.05秒刷新
}

static void stop_recording_voice(void)
{
    // Clean up timer
    if (p_app_recorder->record_timer)
    {
        lv_timer_del(p_app_recorder->record_timer);
        p_app_recorder->record_timer = NULL;

        // Stop system recording
        stop_voice_recording();

        // Update UI for stopped state
        set_record_button_style(p_app_recorder->record_button, false);

        lv_label_set_text(p_app_recorder->record_time_label, "tap to record");

        // Save file name for auto-sync after list refresh
        const char* file_path = get_last_recording_file();
        if (file_path && strlen(file_path) > 0)
        {
            // Extract file name from path (remove "/recorder/" prefix)
            const char* file_name = strrchr(file_path, '/');
            if (file_name)
            {
                file_name++; // Skip the '/' character
                strncpy(pending_auto_sync_file, file_name, MAX_RECORD_PATH_LEN - 1);
                pending_auto_sync_file[MAX_RECORD_PATH_LEN - 1] = '\0';
                LOG_D("Pending auto-sync file: %s", pending_auto_sync_file);
            }
        }
    }
}

static void stop_widget_recording_voice(void)
{
    // Clean up timer
    if (p_widget_recorder->widget_record_timer)
    {
        lv_timer_del(p_widget_recorder->widget_record_timer);
        p_widget_recorder->widget_record_timer = NULL;

        // Stop system recording
        stop_voice_recording();

        // Update UI for stopped state
        set_record_button_style(p_widget_recorder->widget_record_button, false);

        lv_label_set_text(p_widget_recorder->widget_record_time_label, "tap to record");

        // Auto-sync the recorded file to phone (widget has no list UI, use direct sync)
        const char* file_path = get_last_recording_file();
        if (file_path && strlen(file_path) > 0)
        {
            // Extract file name from path (remove "/recorder/" prefix)
            const char* file_name = strrchr(file_path, '/');
            if (file_name)
            {
                file_name++; // Skip the '/' character
                LOG_D("Widget auto-syncing recorded file: %s", file_name);
                ui_sync_file((char*)file_name, NULL); // NULL because widget has no list UI
            }
        }
    }
}

static void refresh_folder_listview(void)
{
    // Remove existing list tile if it exists
    if (lv_obj_is_valid(p_app_recorder->list_tile))
        lv_obj_del(p_app_recorder->list_tile);

    // Get updated file list
    folder = list_files_in_recorder();
    LOG_D("3folder->file_num: %d", folder->file_num);
    if (folder != NULL)
    {
        // Configure tile
        p_app_recorder->list_tile = lv_tileview_add_tile(p_app_recorder->tileview, 0, 1, LV_DIR_VER);
        lv_obj_set_pos(p_app_recorder->list_tile, 0, LV_VER_RES);
        lv_obj_set_size(p_app_recorder->list_tile, LV_HOR_RES, LV_VER_RES);

        // Create and position new list
        create_recorder_file_list(p_app_recorder->list_tile, folder);

        // Auto-sync pending file if any
        if (pending_auto_sync_file[0] != '\0')
        {
            LOG_D("Auto-syncing file after list refresh: %s", pending_auto_sync_file);

            // Find the button for this file in the file_list
            uint32_t child_cnt = lv_obj_get_child_cnt(p_app_recorder->file_list);
            for (uint32_t i = 0; i < child_cnt; i++)
            {
                lv_obj_t *btn = lv_obj_get_child(p_app_recorder->file_list, i);
                lv_obj_t *label = lv_obj_get_child(btn, 0);
                const char *file_name = lv_label_get_text(label);

                if (strcmp(file_name, pending_auto_sync_file) == 0)
                {
                    LOG_D("Found file button, starting auto-sync");
                    ui_sync_file(pending_auto_sync_file, btn);
                    break;
                }
            }

            // Clear pending sync file
            pending_auto_sync_file[0] = '\0';
        }

        // Free memory
        free_recorder_folder(folder);
        folder = NULL;
    }

    // Switch to list view after creating a new recording
    // lv_obj_set_tile_id(p_app_recorder->tileview, 0, 1, LV_ANIM_ON);
}

static void scroll_message_list(bool up);
/* UI event handlers */
static void handle_tap_event(uint8_t press)
{
    if (press == 1)
    {
        if (button_selection_index == 0)
        {
            if (!app_voice_get_recording_status())
            {
                start_to_record_voice();
            }
            else
            {
                stop_recording_voice();
                refresh_folder_listview();
            }
        }
        else if (button_selection_index == 1)
        {
            lv_obj_set_tile_id(p_app_recorder->tileview, 0, 1, LV_ANIM_ON);
        }
    }
    else
    {
        if (button_selection_index == 2)
        {
            scroll_message_list(true);
        }
        else if (button_selection_index == 4)
        {
            scroll_message_list(false);
        }
    }
}

static void handle_widget_tap_event(void)
{
    if (!app_voice_get_recording_status())
    {
        start_widget_to_record_voice();
    }
    else
    {
        stop_widget_recording_voice();
    }
}

void recording_widget_handle_tap_event(uint8_t press)
{
    if (press == 1)
    {
        handle_widget_tap_event();
    }
}

void recording_widget_handle_press_event(uint8_t action)
{
    if (action == 1)
    {
        handle_widget_tap_event();
    }
}

static void handle_vad_status(bool user_talking)
{
    // Update button border to indicate voice activity
    if (user_talking)
    {
        lv_obj_set_style_border_color(p_app_recorder->record_button, lv_color_hex(0x97CBFF), 0);
        lv_obj_set_style_border_width(p_app_recorder->record_button, 10, 0);
    }
    else
    {
        lv_obj_set_style_border_color(p_app_recorder->record_button, lv_color_hex(0x808080), 0);
        lv_obj_set_style_border_width(p_app_recorder->record_button, 5, 0);
    }
}

static void handle_file_sync(bool state)
{
    if (state)
    {
        // Started syncing file
        if (sync_button && lv_obj_is_valid(sync_button))
        {
            loading = create_loading_label(sync_button, "Syncing...");
            lv_obj_set_style_bg_color(sync_button, lv_color_hex(0x97CBFF), 0);
        }
    }
    else
    {
        // Finished syncing file
        if (loading && lv_obj_is_valid(loading))
        {
            lv_obj_del(loading);
        }
        sync_button = NULL;
        refresh_folder_listview();
    }
}

static void handle_sync_progress(uint8_t progress)
{
    // Update sync progress indicator
    if (sync_button && lv_obj_is_valid(sync_button))
    {
        char text[16];
        snprintf(text, sizeof(text), "Syncing...%d%%", progress);
        lv_label_set_text(lv_obj_get_child(sync_button, 1), text);
    }
}

static void toggle_record_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        handle_tap_event(1);
    }
}

static void widget_record_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        handle_widget_tap_event();
    }
}

static void ui_sync_file(char *file_name, lv_obj_t *parent)
{
    if (!SkaiWatchSys.connected_to_phone)
    {
        ui_show_hint_toast("Phone not connected, cannot sync file: %s\n", file_name);
        return;
    }
    sync_button = parent;

    // Create full file path
    char file_path[MAX_RECORD_PATH_LEN];
    snprintf(file_path, sizeof(file_path), "/recorder/%s", file_name);

    // Check if already syncing this file
    sync_progress_t *progress = get_sync_progress();
    if (progress->sync_status && strcmp(progress->sync_in_file_path, file_path) == 0)
    {
        return;
    }

    // Check if it's an Opus file, decode to PCM first before syncing
    // char sync_file_path[MAX_RECORD_PATH_LEN];
    // if (strstr(file_path, ".opus") != NULL)
    // {
    //     LOG_D("Opus file detected, decoding to PCM first...");
    //     if (opus_decode_to_pcm(file_path, sync_file_path) == 0)
    //     {
    //         LOG_D("Decoded to: %s", sync_file_path);
    //     }
    //     else
    //     {
    //         LOG_E("Failed to decode Opus file: %s", file_path);
    //         return;
    //     }
    // }
    // else
    // {
    //     strncpy(sync_file_path, file_path, MAX_RECORD_PATH_LEN - 1);
    //     sync_file_path[MAX_RECORD_PATH_LEN - 1] = '\0';
    // }
    // int ret = bloc_file_system.sync_file(sync_file_path, true);

    int ret = bloc_file_system.sync_file(file_path, true);
    if (ret == 0)
    {
        LOG_D("send sync memo successfully");
    }
    else
    {
        LOG_E("Failed to sync memo");
    }
}

static void file_click_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    if (code == LV_EVENT_CLICKED)
    {
        char *text = lv_label_get_text(lv_obj_get_child(obj, 0));
        LV_LOG_USER("Clicked: %s", text);
        ui_sync_file(text, obj);
    }
}

static void delete_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_user_data(e);

    // Get file name
    lv_obj_t *label = lv_obj_get_child(obj, 0);
    const char *file_name = lv_label_get_text(label);

    // Build complete file path
    char file_path[MAX_RECORD_PATH_LEN];
    snprintf(file_path, sizeof(file_path), "/recorder/%s", file_name);

    // Delete file
    if (remove(file_path) == 0)
    {
        LOG_D("File %s deleted successfully", file_path);
    }
    else
    {
        LOG_E("Failed to delete file %s", file_path);
    }

    // Update UI
    lv_obj_del(obj);
    refresh_folder_listview();
}

static void file_swipe_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    if (code == LV_EVENT_GESTURE)
    {
        lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
        if (dir == LV_DIR_LEFT)
        {
            // Create delete button when swiping left
            LOG_D("Swipe left");
            lv_obj_t *delete_btn = lv_btn_create(obj);
            lv_obj_set_size(delete_btn, 60, lv_obj_get_height(obj));
            lv_obj_align(delete_btn, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(delete_btn, lv_color_hex(0xFF0000), 0);

            lv_obj_t *label = lv_label_create(delete_btn);
            lv_label_set_text(label, "Delete");
            lv_obj_center(label);

            lv_obj_add_event_cb(delete_btn, delete_event_handler, LV_EVENT_CLICKED, obj);
        }
    }
}

void recorder_widget_start(void)
{
    RT_ASSERT(NULL == p_widget_recorder);
    p_widget_recorder = (widget_recorder_t *)lv_mem_alloc(sizeof(widget_recorder_t));
    memset(p_widget_recorder, 0, sizeof(widget_recorder_t));
}

void recorder_widget_stop(void)
{
    LOG_D("recorder_widget_stop");
    stop_widget_recording_voice();
    if (p_widget_recorder)
    {
        if (p_widget_recorder->widget_record_timer)
        {
            lv_timer_del(p_widget_recorder->widget_record_timer);
            p_widget_recorder->widget_record_timer = NULL;
        }
        lv_mem_free(p_widget_recorder);
        p_widget_recorder = NULL;
    }
}

static uint8_t recorder_scroll_target_item = 0;
static void scroll_message_list(bool down)
{
    folder = list_files_in_recorder();
    LOG_D("2folder->file_num: %d", folder->file_num);
    if (down)
    {
        // Scroll up to the top
        if (recorder_scroll_target_item < folder->file_num - 1)
        {
            recorder_scroll_target_item++;
        }
    }
    else
    {
        // Scroll down to the bottom
        if (recorder_scroll_target_item > 0)
        {
            recorder_scroll_target_item--;
        }
        else
        {
            lv_obj_set_tile_id(p_app_recorder->tileview, 0, 0, LV_ANIM_ON);
        }
    }

    // Remove border from all items first
    uint32_t child_cnt = lv_obj_get_child_cnt(p_app_recorder->file_list);
    for (uint32_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = lv_obj_get_child(p_app_recorder->file_list, i);
        lv_obj_set_style_border_width(child, 0, 0);
    }

    // Add border to the current item
    lv_obj_t *target = lv_obj_get_child(p_app_recorder->file_list, recorder_scroll_target_item);
    if (target)
    {
        lv_obj_set_style_border_width(target, 4, 0);
        lv_obj_set_style_border_color(target, lv_color_hex(0x97CBFF), 0);
        lv_obj_set_style_border_opa(target, LV_OPA_100, 0);
        lv_obj_scroll_to_view(target, LV_ANIM_ON);
    }
}

static uint8_t page_index = 0;
static void tileview_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_VALUE_CHANGED)
    {
        rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(e);
        LOG_D("Tileview active position changed to: %d", active_pos);
        if (page_index != active_pos)
        {
            page_index = active_pos;
        }
    }
}
/* App lifecycle functions */
lv_obj_t *recorder_on_start(lv_obj_t *scr)
{
    LOG_D("recorder_app_on_start");
    // Initialize app data structure
    RT_ASSERT(NULL == p_app_recorder);
    p_app_recorder = (app_recorder_t *)lv_mem_alloc(sizeof(app_recorder_t));
    memset(p_app_recorder, 0, sizeof(app_recorder_t));

    // Create UI elements
    p_app_recorder->bg = common_black_bg(scr);

    // Create tileview (main container for vertical navigation)
    p_app_recorder->tileview = lv_tileview_create(p_app_recorder->bg);
    lv_obj_set_size(p_app_recorder->tileview, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_scrollbar_mode(p_app_recorder->tileview, LV_SCROLLBAR_MODE_OFF);
    // Create record view (first/top tile)
    p_app_recorder->record_tile = lv_tileview_add_tile(p_app_recorder->tileview, 0, 0, LV_DIR_VER);
    // lv_obj_set_pos(p_app_recorder->record_tile, 0, 0);
    lv_obj_add_event_cb(p_app_recorder->tileview, tileview_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_size(p_app_recorder->record_tile, LV_HOR_RES, LV_VER_RES);
    create_record_view(p_app_recorder->tileview);
    page_index = 0;
    button_selection_index = 0;
    // Load file list and create file list view (second/bottom tile)
    folder = list_files_in_recorder();
    if (folder != NULL)
    {
        LOG_D("file_num: %d", folder->file_num);
        for (int i = 0; i < folder->file_num; i++)
        {
            LOG_D("file_name: %s", folder->files[i].name);
        }

        p_app_recorder->list_tile = lv_tileview_add_tile(p_app_recorder->tileview, 0, 1, LV_DIR_VER);
        // lv_obj_set_pos(p_app_recorder->list_tile, 0, LV_VER_RES);
        lv_obj_set_size(p_app_recorder->list_tile, LV_HOR_RES, LV_VER_RES);
        create_recorder_file_list(p_app_recorder->list_tile, folder);

        free_recorder_folder(folder);
        folder = NULL;
    }

    // Configure system settings
    setting_provider.set_power_save_mode(0);
    // start_to_record_voice();
    return p_app_recorder->tileview;
}

static void reset_record_style(void)
{
    lv_obj_set_style_border_opa(p_app_recorder->record_button, LV_OPA_30, 0);
    lv_obj_set_style_text_opa(p_app_recorder->scroll_hint, LV_OPA_70, 0);
}

static void reset_file_list_style(void)
{
    lv_obj_set_style_img_opa(p_app_recorder->direction_icon[0], LV_OPA_30, 0);
    lv_obj_set_style_img_opa(p_app_recorder->direction_icon[1], LV_OPA_30, 0);
}

static void button_selection(gesture_position_t gesture_position)
{
    if (peripheral_provider.get_tap_status())
        return;
    // 檢查目前是否在錄音tile（record_tile）
    folder = list_files_in_recorder();
    // LOG_D("gesture_position_X: %d, Y: %d,page_index: %d", gesture_position.gesture_position_x, gesture_position.gesture_position_y, page_index);
    if (page_index == 0)
    {
        // 當前在錄音tile，可根據需求進行操作
        int category = (gesture_position.gesture_position_x > 400 && folder->file_num > 0) ? 1 : 0;
        if (category != button_selection_index)
        {
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
        }
        if (category != button_selection_index)
            button_selection_index = category;
        else
            return; // 如果沒有變化，則不需要重置樣式
        reset_record_style();
        switch (category)
        {
        case 0:
        {
            lv_obj_set_style_border_opa(p_app_recorder->record_button, LV_OPA_100, 0);
        }
        break;

        case 1:
        {
            lv_obj_set_style_text_opa(p_app_recorder->scroll_hint, LV_OPA_100, 0);
        }
        break;
        default:
            break;
        }
    }
    else
    {
        int category = (gesture_position.gesture_position_x > 350) ? 2 : ((gesture_position.gesture_position_x > 116) ? 3 : 4);
        if (category != button_selection_index)
        {
            watch_system_interact(INTERACT_MOTOR_VIBRATE_SCROLLING_APP, NULL);
        }
        if (category != button_selection_index)
            button_selection_index = category;
        else
            return; // 如果沒有變化，則不需要重置樣式
        reset_file_list_style();
        switch (category)
        {
        case 2:
        {
            // Scroll up in the file list
            lv_obj_set_style_img_opa(p_app_recorder->direction_icon[1], LV_OPA_100, 0);
        }
        break;
        // case 3:
        // {
        //     lv_obj_t *target = lv_obj_get_child(p_app_recorder->file_list, recorder_scroll_target_item);
        //     if (target)
        //     {
        //         lv_obj_set_style_border_width(target, 4, 0);
        //         lv_obj_set_style_border_color(target, lv_color_hex(0x97CBFF), 0);
        //         lv_obj_set_style_border_opa(target, LV_OPA_100, 0);
        //         lv_obj_scroll_to_view(target, LV_ANIM_ON);
        //     }
        //     break;
        // }
        case 4:
        {
            // Scroll down in the file list
            lv_obj_set_style_img_opa(p_app_recorder->direction_icon[0], LV_OPA_100, 0);
        }
        break;
        default:
            break;
        }
    }
    free_recorder_folder(folder);
    folder = NULL;
}

void recorder_on_resume(void)
{
    reset_lvgl_msg_handler();
    set_open_control_options(true);
#ifdef BSP_USING_UI_HANDLER
    // Register UI event handlers
    lvgl_msg_handler.handle_vad_status = handle_vad_status;
    lvgl_msg_handler.handle_tap_indicator = handle_tap_event;
    lvgl_msg_handler.handle_file_sync = handle_file_sync;
    lvgl_msg_handler.handle_sync_progress = handle_sync_progress;
    lvgl_msg_handler.handle_widgets_control = button_selection;
#endif
}

void recorder_on_pause(void)
{
#ifdef BSP_USING_UI_HANDLER
    // Unregister UI event handlers
    lvgl_msg_handler.handle_vad_status = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
    lvgl_msg_handler.handle_file_sync = NULL;
    lvgl_msg_handler.handle_sync_progress = NULL;
    lvgl_msg_handler.handle_widgets_control = NULL;
#endif
    set_open_control_options(false);
}

void recorder_on_stop(void)
{
    // Stop any active recording
    stop_recording_voice();
    setting_provider.set_power_save_mode(1);
    if (folder)
    {
        free_recorder_folder(folder);
        folder = NULL;
    }
    // Clean up resources
    if (p_app_recorder)
    {
        if (p_app_recorder->record_timer)
        {
            lv_timer_del(p_app_recorder->record_timer);
            p_app_recorder->record_timer = NULL;
        }

        lv_obj_del(p_app_recorder->tileview);
        p_app_recorder->record_tile = NULL; // Deleted by tileview
        p_app_recorder->list_tile = NULL;   // Deleted by tileview

        lv_obj_del(p_app_recorder->bg);
        p_app_recorder->bg = NULL;

        lv_mem_free(p_app_recorder);
        p_app_recorder = NULL;
    }
}

/* Message handler */
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        recorder_on_start(lv_scr_act());
        break;

    case GUI_APP_MSG_ONRESUME:
        recorder_on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        recorder_on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        recorder_on_stop();
        break;

    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_RECORDER, msg_handler);
    return 0;
}

lv_obj_t *lv_recorder_widget_builder(lv_obj_t *parent)
{
    if (!parent)
    {
        LOG_E("Invalid parent in lv_recorder_widget_builder");
        return NULL;
    }

    // 創建widget容器
    lv_obj_t *widget = common_widget_container(parent);
    lv_obj_set_size(widget, WIDGET_WIDTH, WIDGET_HEIGHT);
    if (!widget)
        return NULL;

    // 創建錄音按鈕
    lv_obj_t *record_btn = lv_obj_create(widget);
    lv_obj_set_size(record_btn, CIRCLE_SIZE + 10, CIRCLE_SIZE + 10);
    lv_obj_set_style_radius(record_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(record_btn, lv_color_hex(0x808080), 0);
    lv_obj_set_style_bg_opa(record_btn, LV_OPA_50, 0);
    lv_obj_set_style_border_color(record_btn, lv_color_hex(0x808080), 0);
    lv_obj_set_style_border_width(record_btn, 5, 0);
    lv_obj_center(record_btn);
    lv_obj_add_event_cb(record_btn, widget_record_btn_event_cb, LV_EVENT_ALL, NULL);
    p_widget_recorder->widget_record_button = record_btn;
    // 設置初始按鈕樣式
    set_record_button_style(record_btn, false);

    // 創建時間標籤
    lv_obj_t *time_label = lv_label_create(widget);
    lv_label_set_text(time_label, "tap to record");
    lv_obj_set_style_text_font(time_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(time_label, lv_color_white(), 0);
    lv_obj_align(time_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    p_widget_recorder->widget_record_time_label = time_label;

    return widget;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(recorder), IMG_RECORDER, APP_ID_RECORDER, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/