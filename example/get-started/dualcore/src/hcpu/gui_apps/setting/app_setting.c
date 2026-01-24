/**
 ******************************************************************************
 * @file   app_setting.c
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
#include "lv_ext_resource_manager.h"
#include "gui_app_fwk.h"
#include "custom_trans_anim.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "ui_handler.h"
#include "ui_img_helper.h"

#define DBG_TAG "app.setting"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#include "gesture_recognition_task.h"
#include "watch_global_data.h"

#ifdef APP_ID_SETTING

#define CANVAS_WIDTH (LV_HOR_RES_MAX)
#define CANVAS_HEIGHT (LV_VER_RES_MAX)
#define LIST_MENU_TITLE_HEIGHT 70
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

LV_IMG_DECLARE(dn);
LV_IMG_DECLARE(language);
LV_IMG_DECLARE(airplane);
LV_IMG_DECLARE(clock_40);
LV_IMG_DECLARE(clock_80);
LV_IMG_DECLARE(clock_100);

LV_IMG_DECLARE(img_stocks);
LV_IMG_DECLARE(img_game);
LV_IMG_DECLARE(icon_release);

/**
 *  description of app setting
 *
 */
typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *list1;
    lv_obj_t *selected_lang;
    lv_obj_t *selected_time_format;
    lv_obj_t *selected_font_size;
} app_setting_t;

static app_setting_t *p_app_setting = NULL;

extern void app_setting_display_main(void);
extern void app_setting_sys_main(void);
extern void app_developer_main(void);
extern void app_setting_device_list_main(void);

static void time_format_setting_win_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_CLICKED)
    {
        lv_obj_del(obj->parent);
        p_app_setting->selected_lang = NULL;
    }
}

static void time_format_btn_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_VALUE_CHANGED)
    {
        lv_state_t new_state = lv_obj_get_state(obj);
        if ((p_app_setting->selected_time_format != obj) && (0 == (LV_STATE_PRESSED & new_state)))
        {
            // Clear the previous selection if there was one
            if (p_app_setting->selected_time_format)
            {
                lv_obj_clear_state(p_app_setting->selected_time_format, LV_STATE_CHECKED);
            }

            // Set the new selection
            p_app_setting->selected_time_format = obj;
            uint8_t time_format_flag;
            if (0 == strcmp(lv_list_get_btn_text(NULL, obj), "12hr"))
            {
                time_format_flag = 0x00;
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
                watch_system_interact(TIME_FORMAT_SET, &time_format_flag);
#endif
            }
            else if (0 == strcmp(lv_list_get_btn_text(NULL, obj), "24hr"))
            {
                time_format_flag = 0x01;
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
                watch_system_interact(TIME_FORMAT_SET, &time_format_flag);
#endif
            }
        }
        else if ((p_app_setting->selected_time_format == obj) && (0 == (LV_STATE_PRESSED & new_state)))
        {
            if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
            {
                lv_obj_clear_state(obj, LV_STATE_CHECKED);
            }
            else
            {
                lv_obj_add_state(obj, LV_STATE_CHECKED);
            }
        }
    }
}

static void lang_btn_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    // LOG_D("event:%d,%d", event, lv_btn_get_state(obj));
    if (event == LV_EVENT_VALUE_CHANGED)
    {
        lv_state_t new_state = lv_obj_get_state(obj);

        // Handle selecting a new language
        if ((p_app_setting->selected_lang != obj) && (0 == (LV_STATE_PRESSED & new_state)))
        {
            // Clear the previous selection if there was one
            if (p_app_setting->selected_lang)
            {
                lv_obj_clear_state(p_app_setting->selected_lang, LV_STATE_CHECKED);
            }

            // Set the new selection
            p_app_setting->selected_lang = obj;

#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
            // Update the system language
            watch_system_interact(LANGUAGE_SET, (void *)lv_list_get_btn_text(NULL, obj));
#endif
        }
        else if ((p_app_setting->selected_lang == obj) && (0 == (LV_STATE_PRESSED & new_state)))
        {
            // Toggle the checked state when clicking the already selected language
            if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
            {
                lv_obj_clear_state(obj, LV_STATE_CHECKED);
            }
            else
            {
                lv_obj_add_state(obj, LV_STATE_CHECKED);
            }
        }
    }
}

static void lang_setting_win_back_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_CLICKED)
    {
        lv_obj_del(obj->parent);
        p_app_setting->selected_lang = NULL;
    }
}

static void font_size_switch_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_VALUE_CHANGED)
    {
        lv_state_t new_state = lv_obj_get_state(obj);
        if ((p_app_setting->selected_font_size != obj) && (0 == (LV_STATE_PRESSED & new_state)))
        {
            // Clear the previous selection if there was one
            if (p_app_setting->selected_font_size)
            {
                lv_obj_clear_state(p_app_setting->selected_font_size, LV_STATE_CHECKED);
            }

            // Get the button text and convert to index
            const char *font_size_text = lv_list_get_btn_text(NULL, obj);
            LVSF_FONT_SIZES font_size_index = LVSF_FONT_TITLE; // default to Title (小)

            // Convert text to enum value using language pack keys
            if (strcmp(font_size_text, LV_EXT_STR_GET_BY_KEY(font_size_small, "Small")) == 0)
            {
                font_size_index = LVSF_FONT_TITLE;
            }
            else if (strcmp(font_size_text, LV_EXT_STR_GET_BY_KEY(font_size_medium, "Medium")) == 0)
            {
                font_size_index = LVSF_FONT_BIG;
            }
            else if (strcmp(font_size_text, LV_EXT_STR_GET_BY_KEY(font_size_large, "Large")) == 0)
            {
                font_size_index = LVSF_FONT_HUGE;
            }

            // update font size
            SkaiWatchSys.font_size = font_size_index;

            // Set the new selection
            p_app_setting->selected_font_size = obj;
        }
    }
}

static void font_size_setting_win_back_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_CLICKED)
    {
        lv_obj_del(obj->parent);
        p_app_setting->selected_font_size = NULL;
    }
}

/**
 * Template function to create a setting page with options
 * @param title_key Settings page title key for localization
 * @param title_default Default title text if key not found
 * @param options Array of option text strings
 * @param option_count Number of options in the array
 * @param initial_selected Index of initially selected option (or -1 for none)
 * @param option_callback Event callback for option buttons
 * @param back_callback Event callback for back button
 * @return Pointer to the selected option button (to be tracked by the caller)
 */
static lv_obj_t *create_setting_template_win(
    const char *title_text,
    const char **options,
    int option_count,
    int initial_selected,
    lv_event_cb_t option_callback,
    lv_event_cb_t back_callback)
{
    // Create a container with dark theme styling
    lv_obj_t *cont = lv_obj_create(p_app_setting->main_window);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x121212), 0); // Match main window dark background
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);

    // Add a title at the top
    lv_obj_t *title = lv_label_create(cont);
    lv_label_set_text(title, title_text);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0); // White text for dark theme
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Create container for the options
    lv_obj_t *options_container = lv_obj_create(cont);
    lv_obj_set_size(options_container, LV_PCT(90), LV_SIZE_CONTENT);
    lv_obj_set_style_radius(options_container, 15, 0);                       // Rounded corners
    lv_obj_set_style_bg_color(options_container, lv_color_hex(0x1E1E1E), 0); // Dark gray
    lv_obj_set_style_border_width(options_container, 0, 0);
    lv_obj_set_style_pad_all(options_container, 10, 0);
    lv_obj_set_flex_flow(options_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_align_to(options_container, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    // Keep track of the selected button
    lv_obj_t *selected_btn = NULL;

    // title_text is font size
    bool is_setting_font_size = (strcmp(title_text, LV_EXT_STR_GET_BY_KEY(setting_font_size, "Font Size")) == 0);

    // Add options as buttons
    for (int i = 0; i < option_count; i++)
    {
        // Create a button for each option
        lv_obj_t *btn = lv_btn_create(options_container);
        lv_obj_set_size(btn, LV_PCT(100), BUTTON_HEIGHT);          // Match height with other items
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1E1E1E), 0); // Dark gray
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_radius(btn, 12, 0); // Rounded corners
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);

        // Add label to button
        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, options[i]);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0); // White text
        if (is_setting_font_size)
        {
            // Use mapped font index: 0->3(Title), 1->4(Big), 2->5(Huge)
            extern const int font_size_index_map[];
            lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(font_size_index_map[i]), 0);
        }
        else
        {
            lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        }
        lv_obj_center(label);

        // Add event handler
        if (option_callback)
        {
            lv_obj_add_event_cb(btn, option_callback, LV_EVENT_ALL, NULL);
        }

        // Set the initial selected state if needed
        if (i == initial_selected)
        {
            lv_obj_add_state(btn, LV_STATE_CHECKED);
            selected_btn = btn;
        }
    }

    // Add back button at the bottom
    lv_obj_t *back_btn = lv_btn_create(cont);
    lv_obj_set_size(back_btn, LV_PCT(90), BUTTON_HEIGHT);
    lv_obj_set_style_radius(back_btn, 15, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x1E1E1E), 0);
    lv_obj_align_to(back_btn, options_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, LV_EXT_STR_GET_BY_KEY(back, "Back"));
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(back_label);

    // Add event handler for back button
    if (back_callback)
    {
        lv_obj_add_event_cb(back_btn, back_callback, LV_EVENT_CLICKED, NULL);
    }

    return selected_btn;
}

static void create_lang_setting_win(void)
{
    // Use LV_EXT_LANG_PACK_LIST_ITER to get language options
    const char *curr_locale = lv_ext_get_locale();

    // First, count available languages
    LV_EXT_LANG_PACK_LIST_ITER_DEF(count_iter);
    int lang_count = 0;
    int selected_index = -1;
    int i = 0;

    LV_EXT_LANG_PACK_LIST_ITER(NULL, count_iter)
    {
        LV_EXT_LANG_PACK_ITER(count_iter, lang_pack)
        {
            lang_count++;
        }
    }

    // Allocate array for language options
    const char **lang_options = (const char **)lv_mem_alloc(sizeof(char *) * lang_count);

    // Fill the array with language names
    LV_EXT_LANG_PACK_LIST_ITER_DEF(iter);
    i = 0;

    LV_EXT_LANG_PACK_LIST_ITER(NULL, iter)
    {
        LV_EXT_LANG_PACK_ITER(iter, lang_pack)
        {
            lang_options[i] = LV_EXT_LANG_PACK_ITER_GET_NAME(lang_pack);

            // Check if this is the current language
            if (0 == strcmp(lang_options[i], curr_locale))
            {
                selected_index = i;
            }
            i++;
        }
    }
    const char *title_text = LV_EXT_STR_GET_BY_KEY(setting_language, "Language");
    // Use the template to create the language settings window
    p_app_setting->selected_lang = create_setting_template_win(
        title_text,
        lang_options, lang_count,
        selected_index,
        lang_btn_event_handler,
        lang_setting_win_back_event_handler);

    // Free the allocated array
    lv_mem_free(lang_options);
}

// Map option index to actual font size enum: 0->Title(3), 1->Big(4), 2->Huge(5)
const int font_size_index_map[] = {3, 4, 5};
static void create_font_size_setting_win(void)
{
    const char *title_text = LV_EXT_STR_GET_BY_KEY(setting_font_size, "Font Size");

    // Font size options - 小(Title), 中(Big), 大(Huge) - get from language pack
    const char *font_size_options[] = {
        LV_EXT_STR_GET_BY_KEY(font_size_small, "Small"),
        LV_EXT_STR_GET_BY_KEY(font_size_medium, "Medium"),
        LV_EXT_STR_GET_BY_KEY(font_size_large, "Large")};

    // Map current font size to option index (0=Title, 1=Big, 2=Huge)
    int selected_index = 0;
    if (SkaiWatchSys.font_size == LVSF_FONT_TITLE)
        selected_index = 0;
    else if (SkaiWatchSys.font_size == LVSF_FONT_BIG)
        selected_index = 1;
    else if (SkaiWatchSys.font_size == LVSF_FONT_HUGE)
        selected_index = 2;

    p_app_setting->selected_font_size = create_setting_template_win(
        title_text,
        font_size_options, 3,
        selected_index,
        font_size_switch_event_handler,
        font_size_setting_win_back_event_handler);
}

static void create_time_format_setting_win(void)
{
    // Time format options
    const char *time_format_options[] = {"12hr", "24hr"};

    // Get current time format setting
    uint8_t hour_format = SkaiWatchSys.flag_field.hour_format;
    const char *title_text = LV_EXT_STR_GET_BY_KEY(setting_time, "Time Format");
    // Use the template to create the time format settings window
    p_app_setting->selected_time_format = create_setting_template_win(
        title_text,
        time_format_options, 2,
        hour_format,
        time_format_btn_event_handler,
        time_format_setting_win_event_handler);
}

static void btn_time_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        create_time_format_setting_win();
    }
}

static void btn_lang_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        create_lang_setting_win();
    }
}

static void btn_font_size_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        create_font_size_setting_win();
    }
}

static void list_display_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        app_setting_display_main();
    }
}

static void dinosaur_game_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        LOG_D("dinosaur game item clicked: %s", lv_list_get_btn_text(NULL, obj));
        gui_app_run(APP_ID_GAME_DINOSAUR);
    }
}

static void btn_sysinfo_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        app_setting_sys_main();
    }
}

static void btn_restart_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        watch_system_interact(WATCH_REBOOT, NULL);
    }
}

static void btn_developer_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        app_developer_main();
    }
}

static lv_obj_t *threshold_window = NULL;
static lv_obj_t *threshold_slider = NULL;
static lv_obj_t *threshold_value_label = NULL;
static void threshold_slider_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *slider = lv_event_get_target(e);
    int value = (int)lv_slider_get_value(slider);

    if (code == LV_EVENT_VALUE_CHANGED)
    {
        // Update value in real-time
        set_gesture_recognition_threshold(value);
        lv_label_set_text_fmt(threshold_value_label, "%d", value);
    }
    else if (code == LV_EVENT_RELEASED)
    {
        store_watch_prefs(WATCH_PREFS_KEY_GESTURE_THRESHOLD);
        LOG_I("Gesture threshold saved to flash: %d", value);
    }
}
static void close_threshold_window_cb(lv_event_t *e)
{
    if (threshold_window)
    {
        lv_obj_del(threshold_window);
        threshold_window = NULL;
        threshold_slider = NULL;
        threshold_value_label = NULL;
    }
}
static void create_gesture_threshold_window(void)
{
    if (threshold_window != NULL)
    {
        return;
    }
    threshold_window = lv_obj_create(lv_scr_act());
    lv_obj_set_size(threshold_window, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(threshold_window, lv_color_hex(0x121212), 0);
    lv_obj_align(threshold_window, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(threshold_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *title = lv_label_create(threshold_window);
    lv_label_set_text(title, "Gesture Confidence");
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 40);
    threshold_value_label = lv_label_create(threshold_window);
    int current_threshold = get_gesture_recognition_threshold();
    lv_label_set_text_fmt(threshold_value_label, "%d", current_threshold);
    lv_obj_set_style_text_font(threshold_value_label, LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_obj_set_style_text_color(threshold_value_label, lv_color_hex(0x00FF00), 0);
    lv_obj_align(threshold_value_label, LV_ALIGN_CENTER, 0, -30);
    threshold_slider = lv_slider_create(threshold_window);
    lv_slider_set_range(threshold_slider, 50, 100);
    lv_slider_set_value(threshold_slider, current_threshold, LV_ANIM_OFF);
    lv_obj_set_size(threshold_slider, 300, 20);
    lv_obj_align(threshold_slider, LV_ALIGN_CENTER, 0, 30);
    lv_obj_add_event_cb(threshold_slider, threshold_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(threshold_slider, threshold_slider_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_t *min_label = lv_label_create(threshold_window);
    lv_label_set_text(min_label, "50");
    lv_obj_set_style_text_color(min_label, lv_color_hex(0x888888), 0);
    lv_obj_align_to(min_label, threshold_slider, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    lv_obj_t *max_label = lv_label_create(threshold_window);
    lv_label_set_text(max_label, "100");
    lv_obj_set_style_text_color(max_label, lv_color_hex(0x888888), 0);
    lv_obj_align_to(max_label, threshold_slider, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    lv_obj_t *close_btn = common_text_button(threshold_window, "OK", NULL, 120, 50, close_threshold_window_cb);
    lv_obj_align(close_btn, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_obj_add_event_cb(threshold_window, close_threshold_window_cb, LV_EVENT_CLICKED, NULL);
}
static void btn_gesture_threshold_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event)
    {
        create_gesture_threshold_window();
    }
}
static void btn_device_list_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        LOG_I("Opening device list...");
        app_setting_device_list_main();
    }
}

void app_setting_init(void *param)
{
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    p_app_setting->main_window = cont;
    lv_obj_set_size(cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    // Dark theme background for circular watch face
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x121212), 0);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);
    lv_obj_update_layout(cont);

    // Create a container for settings groups - adjusted for circular display
    lv_obj_t *settings_container = lv_obj_create(cont);
    lv_obj_set_size(settings_container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(settings_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(settings_container, 0, 0);
    lv_obj_set_style_pad_all(settings_container, 0, 0);
    lv_obj_align(settings_container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_pad_bottom(settings_container, 100, 0);

    lv_obj_t *cont_title = lv_label_create(settings_container);
    lv_label_set_text(cont_title, LV_EXT_STR_GET_BY_KEY(setting_title, "Setting"));
    lv_obj_set_style_text_color(cont_title, lv_color_hex(0xFFFFFF), 0); // White text for dark theme
    lv_obj_set_style_text_font(cont_title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align(cont_title, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_update_layout(cont_title);

    // Create general settings group
    lv_obj_t *general_group = lv_obj_create(settings_container);
    lv_obj_set_size(general_group, LV_PCT(90), LV_SIZE_CONTENT);
    lv_obj_set_style_radius(general_group, 15, 0);                       // More rounded corners
    lv_obj_set_style_bg_color(general_group, lv_color_hex(0x1E1E1E), 0); // Dark gray for items
    lv_obj_set_style_pad_row(general_group, 5, 0);                       // More padding between rows
    lv_obj_set_style_pad_top(general_group, 5, 0);
    lv_obj_set_style_pad_bottom(general_group, 5, 0);
    lv_obj_set_flex_flow(general_group, LV_FLEX_FLOW_COLUMN);
    lv_obj_align_to(general_group, cont_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    // Add list items to general group
    lv_obj_t *list_btn; // Gesture Confidence Threshold - FIRST ITEM
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(icon_release), "Gesture Threshold", SkaiWatchSys.font_size, true, 100);
    lv_obj_add_event_cb(list_btn, btn_gesture_threshold_event_callback, LV_EVENT_SHORT_CLICKED, NULL);
    // Time settings
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(clock_40),
                                        LV_EXT_STR_GET_BY_KEY(setting_time, "Time"), SkaiWatchSys.font_size, true, 100);
    lv_obj_add_event_cb(list_btn, btn_time_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

    // Language settings
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(language),
                                        LV_EXT_STR_GET_BY_KEY(setting_language, "Language"), SkaiWatchSys.font_size, true, 100);
    lv_obj_add_event_cb(list_btn, btn_lang_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

    // font size
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(language),
                                        LV_EXT_STR_GET_BY_KEY(setting_font_size, "Font Size"), SkaiWatchSys.font_size, true, 100);
    lv_obj_add_event_cb(list_btn, btn_font_size_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

    // Display settings
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(dn),
                                        LV_EXT_STR_GET_BY_KEY(setting_display, "Display"), SkaiWatchSys.font_size, true, 100);
    lv_obj_add_event_cb(list_btn, list_display_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

    // Game Mode
    list_btn = create_setting_list_item(general_group, IMG_GAME,
                                        "Dinosaur game", SkaiWatchSys.font_size, true, 50);
    lv_obj_add_event_cb(list_btn, dinosaur_game_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

    // System Info
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(airplane),
                                        LV_EXT_STR_GET_BY_KEY(system_info, "System Info"), SkaiWatchSys.font_size, true, 100);
    lv_obj_add_event_cb(list_btn, btn_sysinfo_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

    // BLE Devices button
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(airplane),
                                        "BLE Devices", SkaiWatchSys.font_size, true, 100);
    lv_obj_add_event_cb(list_btn, btn_device_list_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

#if !kReleaseMode
    // Test / Developer button
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(airplane),
                                        LV_EXT_STR_GET_BY_KEY(test_str, "Test"), SkaiWatchSys.font_size, false, 100);
    lv_obj_add_event_cb(list_btn, btn_developer_event_callback, LV_EVENT_SHORT_CLICKED, NULL);
#else
    // Restart button (only in release mode)
    list_btn = create_setting_list_item(general_group, LV_EXT_IMG_GET(airplane),
                                        LV_EXT_STR_GET_BY_KEY(restart, "restart"), SkaiWatchSys.font_size, false, 100);
    lv_obj_add_event_cb(list_btn, btn_restart_event_callback, LV_EVENT_SHORT_CLICKED, NULL);
#endif
    // Add scrollbar styling for circular watch face - make it more subtle
    lv_obj_set_scrollbar_mode(settings_container, LV_SCROLLBAR_MODE_AUTO);

    // Store reference to main list container
    p_app_setting->list1 = settings_container;
}

static void on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_setting);
    p_app_setting = (app_setting_t *)lv_mem_alloc(sizeof(app_setting_t));
    memset(p_app_setting, 0, sizeof(app_setting_t));

    app_setting_init(NULL);
#if 0
    cust_trans_anim_config(CUST_ANIM_TYPE_0, NULL);
#else
    gui_app_trans_anim_t enter_anim_cfg, exit_anim_cfg;

    gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
    gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_NONE);

    gui_app_set_enter_trans_anim(&enter_anim_cfg);
    gui_app_set_exit_trans_anim(&exit_anim_cfg);

    gui_app_set_trans_anim_prio(1, -1);
#endif
}
static void on_resume(void)
{
}

static void on_pause(void)
{
}

static void on_stop(void)
{
    if (p_app_setting)
    {
        lv_obj_del(p_app_setting->main_window);
        p_app_setting->main_window = NULL;
        lv_mem_free(p_app_setting);
        p_app_setting = NULL;
    }
}

static void back_to_main_menu(void)
{
    gui_app_goback();
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        on_start(lv_scr_act());
    }
    break;

    case GUI_APP_MSG_ONRESUME:
    {
        on_resume();
    }
    break;

    case GUI_APP_MSG_ONPAUSE:
    {
        on_pause();
    }
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
    gui_app_regist_msg_handler(APP_ID_SETTING, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(setting), IMG_SETTINGS, APP_ID_SETTING, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/