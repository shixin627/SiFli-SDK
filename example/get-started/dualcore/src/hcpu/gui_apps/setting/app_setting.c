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
#include "ui_datasrv_subscriber.h"
#include "power_manager_service.h"

#define DBG_TAG "app.setting"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#include "gesture_recognition_task.h"
#include "watch_global_data.h"
#include "bloc_control.h"
#include "bloc_peripheral.h"
#include "bloc_setting.h"
#include <dfs_posix.h>
#include <unistd.h>

#ifdef APP_ID_SETTING

#define CANVAS_WIDTH (LV_HOR_RES_MAX)
#define CANVAS_HEIGHT (LV_VER_RES_MAX)
#define LIST_MENU_TITLE_HEIGHT 70
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

LV_IMG_DECLARE(dn);
LV_IMG_DECLARE(language);
LV_IMG_DECLARE(airplane);
LV_IMG_DECLARE(clock_40);
LV_IMG_DECLARE(icon_release);
LV_IMG_DECLARE(sun);
LV_IMG_DECLARE(service_hour);

/**
 *  description of app setting
 *
 */
typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *list1;
    lv_obj_t *selected_lang;
    lv_obj_t *selected_font_size;
    lv_obj_t *brightness_bar;
    lv_obj_t *screen_time_bar;
    lv_obj_t *brightness_fill; /* self-drawn fill; floors at a circle so the bar can't shrink past it */
    lv_obj_t *screen_time_fill;
    lv_obj_t *screen_time_label;
    datac_handle_t pwr_srv_hdl;
} app_setting_t;

static app_setting_t *p_app_setting = NULL;
static lv_obj_t *dnd_quick_btn = NULL;
static lv_obj_t *time_format_quick_btn = NULL;
static lv_obj_t *time_format_title_label = NULL;
static lv_obj_t *time_format_badge_label = NULL;
static lv_obj_t *mouse_press_quick_btn = NULL;

extern void app_developer_main(void);

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
            interact_language_set(lv_list_get_btn_text(NULL, obj));
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
            LVSF_FONT_SIZES font_size_index = LVSF_FONT_TITLE; // default to Title

            // Convert text to enum value using language pack keys
            if (strcmp(font_size_text, LV_EXT_STR_GET_BY_KEY(font_size_small, "Small")) == 0)
            {
                font_size_index = LVSF_FONT_SUBTITLE;
            }
            else if (strcmp(font_size_text, LV_EXT_STR_GET_BY_KEY(font_size_medium, "Medium")) == 0)
            {
                font_size_index = LVSF_FONT_TITLE;
            }
            else if (strcmp(font_size_text, LV_EXT_STR_GET_BY_KEY(font_size_large, "Large")) == 0)
            {
                font_size_index = LVSF_FONT_BIG;
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

    // Options container — transparent flex column (no surrounding box)
    lv_obj_t *options_container = lv_obj_create(cont);
    lv_obj_set_size(options_container, LV_PCT(90), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(options_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(options_container, 0, 0);
    lv_obj_set_style_pad_all(options_container, 0, 0);
    lv_obj_set_style_pad_row(options_container, 10, 0);
    lv_obj_set_flex_flow(options_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_align_to(options_container, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    // Keep track of the selected button
    lv_obj_t *selected_btn = NULL;

    // title_text is font size
    bool is_setting_font_size = (strcmp(title_text, LV_EXT_STR_GET_BY_KEY(setting_font_size, "Font Size")) == 0);

    // Capsule-style options
    for (int i = 0; i < option_count; i++)
    {
        lv_obj_t *btn = lv_btn_create(options_container);
        lv_obj_set_size(btn, LV_PCT(100), 80);
        lv_obj_set_style_radius(btn, 40, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xCECECE), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_10, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_90, LV_STATE_CHECKED);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_pad_all(btn, 0, 0);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);

        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, options[i]);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
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

        if (option_callback)
        {
            lv_obj_add_event_cb(btn, option_callback, LV_EVENT_ALL, NULL);
        }

        if (i == initial_selected)
        {
            lv_obj_add_state(btn, LV_STATE_CHECKED);
            selected_btn = btn;
        }
    }

    // Back button — capsule style
    lv_obj_t *back_btn = lv_btn_create(cont);
    lv_obj_set_size(back_btn, LV_PCT(90), 80);
    lv_obj_set_style_radius(back_btn, 40, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(back_btn, LV_OPA_10, 0);
    lv_obj_set_style_border_width(back_btn, 0, 0);
    lv_obj_set_style_pad_all(back_btn, 0, 0);
    lv_obj_align_to(back_btn, options_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, LV_EXT_STR_GET_BY_KEY(back, "Back"));
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(back_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
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

    // Map current font size to option index (0=Subtitle, 1=Title, 2=Big)
    int selected_index = 0;
    if (SkaiWatchSys.font_size == LVSF_FONT_SUBTITLE)
        selected_index = 0;
    else if (SkaiWatchSys.font_size == LVSF_FONT_TITLE)
        selected_index = 1;
    else if (SkaiWatchSys.font_size == LVSF_FONT_BIG)
        selected_index = 2;

    p_app_setting->selected_font_size = create_setting_template_win(
        title_text,
        font_size_options, 3,
        selected_index,
        font_size_switch_event_handler,
        font_size_setting_win_back_event_handler);
}

static lv_obj_t *create_capsule_item(lv_obj_t *parent, lv_obj_t *anchor,
                                     const void *icon, const char *text,
                                     bool show_arrow)
{
    const lv_coord_t w = LV_HOR_RES * 80 / 100;
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, 80);
    lv_obj_align_to(btn, anchor, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_set_style_radius(btn, 40, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_10, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_pad_all(btn, 0, 0);

    if (icon)
    {
        lv_obj_t *img = lv_img_create(btn);
        lv_img_set_src(img, icon);
        lv_obj_align(img, LV_ALIGN_LEFT_MID, 20, 0);

        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, text);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(label,
                                   LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_align_to(label, img, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    }
    else
    {
        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, text);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(label,
                                   LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_align(label, LV_ALIGN_LEFT_MID, 20, 0);
    }

    if (show_arrow)
    {
        lv_obj_t *arrow = lv_label_create(btn);
        lv_label_set_text(arrow, ">");
        lv_obj_set_style_text_color(arrow, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(arrow,
                                   LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, -20, 0);
    }
    return btn;
}

static void update_time_format_title_label(uint8_t hour_format)
{
    if (time_format_title_label && lv_obj_is_valid(time_format_title_label))
    {
        lv_label_set_text(time_format_title_label,
                          (hour_format == 0x01) ? LV_EXT_STR_GET_BY_KEY(time_format_24, "24-hour") : LV_EXT_STR_GET_BY_KEY(time_format_12, "12-hour"));
    }
    if (time_format_badge_label && lv_obj_is_valid(time_format_badge_label))
    {
        lv_label_set_text(time_format_badge_label,
                          (hour_format == 0x01) ? "24" : "12");
    }
}

static void time_format_switch_event_callback(lv_event_t *e)
{
    if (LV_EVENT_VALUE_CHANGED == lv_event_get_code(e))
    {
        lv_obj_t *sw = lv_event_get_target(e);
        uint8_t new_format = (lv_obj_get_state(sw) & LV_STATE_CHECKED) ? 0x01 : 0x00;
        SkaiWatchSys.flag_field.hour_format = new_format;
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
        setting_provider.set_hour_format(new_format);
#endif
        update_time_format_title_label(new_format);
        LOG_I("Time format toggled: %s", (new_format == 0x01) ? "24h" : "12h");
    }
}

static void mouse_press_switch_event_callback(lv_event_t *e)
{
    if (LV_EVENT_VALUE_CHANGED == lv_event_get_code(e))
    {
        lv_obj_t *sw = lv_event_get_target(e);
        bool new_status = (lv_obj_get_state(sw) & LV_STATE_CHECKED) ? true : false;
        SkaiWatchSys.flag_field.mouse_press_mode = new_status;
        LOG_I("Mouse press mode toggled: %d", new_status);
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


static void btn_restart_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        peripheral_provider.hcpu_reboot();
    }
}

static void recursive_delete(const char *path)
{
    DIR *dir = opendir(path);
    if (!dir)
    {
        unlink(path);
        return;
    }

    struct dirent *entry;
    char filepath[256];
    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        if (strcmp(path, "/") == 0)
            snprintf(filepath, sizeof(filepath), "/%s", entry->d_name);
        else
            snprintf(filepath, sizeof(filepath), "%s/%s", path, entry->d_name);

        LOG_D("Deleting: %s", filepath);
        if (entry->d_type == DT_DIR)
            recursive_delete(filepath);
        else
            unlink(filepath);

        rt_thread_mdelay(10);
    }
    closedir(dir);

    if (strcmp(path, "/") != 0)
        rmdir(path);
}

static void clear_flash_do(void)
{
    static const char *dirs_to_clear[] = {
        "/assets/icons",
        "/assets/images",
        "/prefdb",
        "/recorder",
        "/note_list",
        "/photo",
    };
    LOG_I("Clearing flash data...");
    for (int i = 0; i < ARRAY_SIZE(dirs_to_clear); i++)
    {
        LOG_I("Deleting: %s", dirs_to_clear[i]);
        recursive_delete(dirs_to_clear[i]);
    }
    LOG_I("Flash data cleared.");
}

static lv_obj_t *reset_modal = NULL;

static void reset_modal_close(void)
{
    if (reset_modal && lv_obj_is_valid(reset_modal))
    {
        lv_obj_del(reset_modal);
    }
    reset_modal = NULL;
}

static void reset_modal_close_event_cb(lv_event_t *e)
{
    reset_modal_close();
}

static void reset_modal_confirm_event_cb(lv_event_t *e)
{
    reset_modal_close();
    clear_flash_do();
}

static void show_reset_modal(void)
{
    if (reset_modal && lv_obj_is_valid(reset_modal))
    {
        return;
    }

    /* Full-screen 50%-transparent black overlay */
    reset_modal = lv_obj_create(lv_scr_act());
    lv_obj_set_size(reset_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(reset_modal, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(reset_modal, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(reset_modal, LV_OPA_90, 0);
    lv_obj_set_style_border_width(reset_modal, 0, 0);
    lv_obj_set_style_pad_all(reset_modal, 0, 0);
    lv_obj_clear_flag(reset_modal, LV_OBJ_FLAG_SCROLLABLE);

    /* Title — white, centered, wraps */
    lv_obj_t *title = lv_label_create(reset_modal);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(reset_confirm_title, "Erase all content\nand settings?"));
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(title, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(title, LV_PCT(80));
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 110);

    /* Description — smaller, gray, centered */
    lv_obj_t *desc = lv_label_create(reset_modal);
    lv_label_set_text(desc, LV_EXT_STR_GET_BY_KEY(reset_confirm_desc, "This will reset the\ndevice to factory\ndefaults."));
    lv_obj_set_style_text_color(desc, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(desc,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_align(desc, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(desc, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(desc, LV_PCT(80));
    lv_obj_align_to(desc, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);

    /* Red Reset pill button at the bottom */
    lv_obj_t *reset_btn = lv_btn_create(reset_modal);
    lv_obj_set_size(reset_btn, LV_HOR_RES * 60 / 100, 56);
    lv_obj_align(reset_btn, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_obj_set_style_radius(reset_btn, 28, 0);
    lv_obj_set_style_bg_color(reset_btn, lv_color_hex(0xCC0033), 0);
    lv_obj_set_style_bg_opa(reset_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(reset_btn, 0, 0);
    lv_obj_set_style_shadow_width(reset_btn, 0, 0);
    lv_obj_add_event_cb(reset_btn, reset_modal_confirm_event_cb,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t *reset_label = lv_label_create(reset_btn);
    lv_label_set_text(reset_label, LV_EXT_STR_GET_BY_KEY(reset, "Reset"));
    lv_obj_set_style_text_color(reset_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(reset_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_center(reset_label);

    /* Cancel pill button above Reset */
    lv_obj_t *cancel_btn = lv_btn_create(reset_modal);
    lv_obj_set_size(cancel_btn, LV_HOR_RES * 60 / 100, 56);
    lv_obj_align_to(cancel_btn, reset_btn, LV_ALIGN_OUT_TOP_MID, 0, -10);
    lv_obj_set_style_radius(cancel_btn, 28, 0);
    lv_obj_set_style_bg_color(cancel_btn, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_bg_opa(cancel_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cancel_btn, 0, 0);
    lv_obj_set_style_shadow_width(cancel_btn, 0, 0);
    lv_obj_add_event_cb(cancel_btn, reset_modal_close_event_cb,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t *cancel_label = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_label, LV_EXT_STR_GET_BY_KEY(cancel, "Cancel"));
    lv_obj_set_style_text_color(cancel_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(cancel_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_center(cancel_label);
}

static void btn_clear_flash_event_callback(lv_event_t *e)
{
    if (LV_EVENT_SHORT_CLICKED == lv_event_get_code(e))
    {
        show_reset_modal();
    }
}

#if !kReleaseMode
static void btn_developer_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        app_developer_main();
    }
}
#endif

static void btn_gesture_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event || LV_EVENT_CLICKED == event)
    {
        gui_app_goback();
        gui_app_run(APP_ID_GESTURE);
    }
}

static void btn_find_phone_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event || LV_EVENT_CLICKED == event)
    {
        control_provider.find_phone();
    }
}

static void btn_qrcode_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event || LV_EVENT_CLICKED == event)
    {
        /* Close Setting before launching to keep running-app count under MAX */
        gui_app_goback();
        gui_app_run(APP_ID_QRCODE);
    }
}

static void dnd_switch_event_callback(lv_event_t *e)
{
    if (LV_EVENT_VALUE_CHANGED == lv_event_get_code(e))
    {
        lv_obj_t *sw = lv_event_get_target(e);
        bool new_status = (lv_obj_get_state(sw) & LV_STATE_CHECKED) ? true : false;
        SkaiWatchSys.DNDMode.config.status = new_status;
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
        setting_provider.set_dnd_status(new_status);
#endif
        LOG_I("DND mode toggled: %d", new_status);
    }
}

#if !kReleaseMode
static void btn_gesture_test_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event || LV_EVENT_CLICKED == event)
    {
        gui_app_goback();
        gui_app_run(APP_ID_GESTURE);
    }
}
#endif

/* Dragging fires LV_EVENT_PRESSING every frame. Sending a power-mgr SET_REQ on
 * each one floods the data-service callback queue and the SDK asserts on Qfull
 * (datac_service_usrcbk). Coalesce sends to >= SETTING_BAR_THROTTLE_MS apart and
 * skip unchanged values; always push the final value on release so the persisted
 * state matches where the finger lifted. */
#define SETTING_BAR_THROTTLE_MS 40

/* lv_bar collapses its own indicator to nothing at the minimum value, so instead
 * of using it we draw the fill ourselves (a single rounded child). Its width
 * tracks the value but never drops below SETTING_BAR_FILL_MIN_W, so dragging to
 * the far left bottoms out at a fixed pill (icon-sized) rather than vanishing or
 * needing a separate circle stacked on top. The underlying bar value/range is
 * untouched — only the visual floor changes. */
#define SETTING_BAR_FILL_MIN_W 90

static void setting_bar_apply_fill(lv_obj_t *bar, lv_obj_t *fill)
{
    if (!bar || !fill || !lv_obj_is_valid(bar) || !lv_obj_is_valid(fill))
        return;
    lv_coord_t min = lv_bar_get_min_value(bar);
    lv_coord_t max = lv_bar_get_max_value(bar);
    lv_coord_t value = lv_bar_get_value(bar);
    lv_coord_t barw = lv_obj_get_width(bar);
    lv_coord_t w = SETTING_BAR_FILL_MIN_W;
    if (max > min && barw > SETTING_BAR_FILL_MIN_W)
        w += (lv_coord_t)((int32_t)(value - min) * (barw - SETTING_BAR_FILL_MIN_W) / (max - min));
    if (w < SETTING_BAR_FILL_MIN_W)
        w = SETTING_BAR_FILL_MIN_W;
    if (w > barw)
        w = barw;
    lv_obj_set_width(fill, w);
}

static void brightness_bar_event_cb(lv_event_t *e)
{
    static int16_t last_sent = -1;
    static uint32_t last_send_tick = 0;
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSING)
    {
        lv_obj_t *bar = lv_event_get_target(e);

        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);

        lv_coord_t min = lv_bar_get_min_value(bar);
        lv_coord_t max = lv_bar_get_max_value(bar);

        lv_coord_t w = lv_obj_get_width(bar);
        lv_coord_t rel_x = p.x - lv_obj_get_x(bar);
        if (rel_x < 0)
            rel_x = 0;
        if (rel_x > w)
            rel_x = w;

        lv_coord_t value = (rel_x * (max - min)) / w + min;
        if (value < 5)
            value = 5;
        lv_bar_set_value(bar, value, LV_ANIM_OFF);
        setting_bar_apply_fill(bar, p_app_setting ? p_app_setting->brightness_fill : NULL);
        uint16_t brightness = lv_bar_get_value(bar);
        if ((int16_t)brightness != last_sent &&
            lv_tick_elaps(last_send_tick) >= SETTING_BAR_THROTTLE_MS)
        {
            gui_set_brightness(brightness, true);
            last_sent = (int16_t)brightness;
            last_send_tick = lv_tick_get();
        }
    }
    else if (code == LV_EVENT_PRESSED)
    {
        last_sent = -1;
        last_send_tick = 0;
        if (p_app_setting && p_app_setting->list1)
        {
            lv_obj_clear_flag(p_app_setting->list1, LV_OBJ_FLAG_SCROLLABLE);
        }
    }
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_t *bar = lv_event_get_target(e);
        uint16_t brightness = lv_bar_get_value(bar);
        if ((int16_t)brightness != last_sent)
        {
            gui_set_brightness(brightness, true);
            last_sent = (int16_t)brightness;
        }
        if (p_app_setting && p_app_setting->list1)
        {
            lv_obj_add_flag(p_app_setting->list1, LV_OBJ_FLAG_SCROLLABLE);
        }
    }
}

static void refresh_screen_time_label(uint8_t value)
{
    if (!p_app_setting || !p_app_setting->screen_time_label ||
        !lv_obj_is_valid(p_app_setting->screen_time_label))
    {
        return;
    }
    char buf[16];
    if (value >= 60)
        snprintf(buf, sizeof(buf), "%s", LV_EXT_STR_GET_BY_KEY(screen_time_never, "Never"));
    else
        snprintf(buf, sizeof(buf), LV_EXT_STR_GET_BY_KEY(screen_time_sec_fmt, "%d s"), value);
    lv_label_set_text(p_app_setting->screen_time_label, buf);
}

/* Persist the new auto-off time via setting's own pwr handle so the SET_RSP comes
 * back to setting_powermgr_srv_callback (which calls control_provider.screen_time_smoothly). */
static void screen_time_send_set_req(uint16_t timeout)
{
    if (p_app_setting &&
        DATA_CLIENT_INVALID_HANDLE != p_app_setting->pwr_srv_hdl)
    {
        datac_send_data(p_app_setting->pwr_srv_hdl,
                        PWRMGR_MSG_LCD_AUTO_OFF_TIME_SET_REQ,
                        (uint8_t *)&timeout, sizeof(uint16_t));
    }
}

static void screen_time_bar_event_cb(lv_event_t *e)
{
    static int16_t last_sent = -1;
    static uint32_t last_send_tick = 0;
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSING)
    {
        lv_obj_t *bar = lv_event_get_target(e);

        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);

        lv_coord_t min = lv_bar_get_min_value(bar);
        lv_coord_t max = lv_bar_get_max_value(bar);

        lv_coord_t w = lv_obj_get_width(bar);
        lv_coord_t rel_x = p.x - lv_obj_get_x(bar);
        if (rel_x < 0)
            rel_x = 0;
        if (rel_x > w)
            rel_x = w;

        lv_coord_t value = (rel_x * (max - min)) / w + min;
        if (value < 5)
            value = 5;
        lv_bar_set_value(bar, value, LV_ANIM_OFF);
        setting_bar_apply_fill(bar, p_app_setting ? p_app_setting->screen_time_fill : NULL);
        uint16_t timeout = (uint16_t)lv_bar_get_value(bar);
        refresh_screen_time_label((uint8_t)timeout);
        if ((int16_t)timeout != last_sent &&
            lv_tick_elaps(last_send_tick) >= SETTING_BAR_THROTTLE_MS)
        {
            screen_time_send_set_req(timeout);
            last_sent = (int16_t)timeout;
            last_send_tick = lv_tick_get();
        }
    }
    else if (code == LV_EVENT_PRESSED)
    {
        last_sent = -1;
        last_send_tick = 0;
        if (p_app_setting && p_app_setting->list1)
        {
            lv_obj_clear_flag(p_app_setting->list1, LV_OBJ_FLAG_SCROLLABLE);
        }
    }
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_t *bar = lv_event_get_target(e);
        uint16_t timeout = (uint16_t)lv_bar_get_value(bar);
        if ((int16_t)timeout != last_sent)
        {
            screen_time_send_set_req(timeout);
            last_sent = (int16_t)timeout;
        }
        refresh_screen_time_label((uint8_t)timeout);
        if (p_app_setting && p_app_setting->list1)
        {
            lv_obj_add_flag(p_app_setting->list1, LV_OBJ_FLAG_SCROLLABLE);
        }
    }
}

static int setting_powermgr_srv_callback(data_callback_arg_t *arg)
{
    if (!p_app_setting && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }
    if (p_app_setting &&
        !lv_obj_is_valid(p_app_setting->brightness_bar) &&
        !lv_obj_is_valid(p_app_setting->screen_time_bar) &&
        (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (p_app_setting && rsp->result >= 0)
        {
            data_msg_t msg;
            data_service_init_msg(&msg, PWRMGR_MSG_LCD_BRIGHTNESS_GET_REQ, 0);
            datac_send_msg(p_app_setting->pwr_srv_hdl, &msg);

            data_service_init_msg(&msg, PWRMGR_MSG_LCD_AUTO_OFF_TIME_GET_REQ, 0);
            datac_send_msg(p_app_setting->pwr_srv_hdl, &msg);
        }
    }
    break;

    case PWRMGR_MSG_LCD_BRIGHTNESS_GET_RSP:
    {
        range_msg_t *p_range = (range_msg_t *)arg->data;
        if (lv_obj_is_valid(p_app_setting->brightness_bar))
        {
            lv_bar_set_range(p_app_setting->brightness_bar, p_range->min, p_range->max);
            lv_bar_set_value(p_app_setting->brightness_bar, p_range->cur, LV_ANIM_ON);
            setting_bar_apply_fill(p_app_setting->brightness_bar, p_app_setting->brightness_fill);
        }
    }
    break;

    case PWRMGR_MSG_LCD_AUTO_OFF_TIME_GET_RSP:
    {
        range_msg_t *p_range = (range_msg_t *)arg->data;
        if (lv_obj_is_valid(p_app_setting->screen_time_bar))
        {
            lv_bar_set_range(p_app_setting->screen_time_bar, p_range->min, p_range->max);
            /* Trust SkaiWatchSys.oled_display_time (loaded from prefs) over the
             * service's runtime cur, which may still be a stale default at boot
             * before it gets synced. Clamp to the bar's range so 255 ("never")
             * doesn't visually overflow. */
            int32_t bar_val = SkaiWatchSys.oled_display_time;
            if (bar_val < p_range->min) bar_val = p_range->min;
            if (bar_val > p_range->max) bar_val = p_range->max;
            lv_bar_set_value(p_app_setting->screen_time_bar, bar_val, LV_ANIM_ON);
            setting_bar_apply_fill(p_app_setting->screen_time_bar, p_app_setting->screen_time_fill);
            refresh_screen_time_label(SkaiWatchSys.oled_display_time);
        }
    }
    break;

    case PWRMGR_MSG_LCD_AUTO_OFF_TIME_SET_RSP:
    {
        range_msg_t *p_range = (range_msg_t *)arg->data;
        refresh_screen_time_label((uint8_t)p_range->cur);
#ifdef BSP_USING_BLOC_CONTROL
        control_provider.screen_time_smoothly(p_range->cur);
#endif
    }
    break;

    default:
        break;
    }
    return 0;
}

void app_setting_init(void *param)
{
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    p_app_setting->main_window = cont;
    lv_obj_set_size(cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    // Dark theme background for circular watch face
    // lv_obj_set_style_bg_color(cont, lv_color_hex(0x121212), 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_0, 0);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);
    lv_obj_update_layout(cont);

    lv_obj_t *bg_img = lv_img_create(cont);
    lv_img_set_src(bg_img, GAUS_CLOCK1_BG);
    lv_img_set_zoom(bg_img, 256*2); // 100% zoom
    lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);

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

    /* Brightness bar at the top of settings content */
    const lv_coord_t brightness_bar_w = LV_HOR_RES * 80 / 100;
    lv_obj_t *brightness_bar = lv_bar_create(settings_container);
    lv_bar_set_range(brightness_bar, 0, 100);
    lv_obj_set_size(brightness_bar, brightness_bar_w, 80);
    lv_obj_align_to(brightness_bar, cont_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_set_style_bg_color(brightness_bar, lv_color_hex(0xCECECE), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(brightness_bar, lv_color_hex(0xCECECE), LV_PART_MAIN);
    /* MAIN radius 0 keeps lv_bar off the EPIC GPU indicator path (lv_bar.c:484,
     * gated on bg_radius != 0), which on real hardware paints the indicator across
     * the whole track regardless of value -> a stray full-width oval. MAIN is
     * transparent so its radius has no visual effect; the indicator keeps its own. */
    lv_obj_set_style_radius(brightness_bar, 0, LV_PART_MAIN);
    /* lv_bar draws nothing itself: MAIN + INDICATOR are transparent. The visible
     * fill is the child below, whose width we drive so it floors at a pill instead
     * of collapsing to nothing at the minimum value. */
    lv_obj_set_style_bg_opa(brightness_bar, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(brightness_bar, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_bar_set_value(brightness_bar, SkaiWatchSys.brightness, LV_ANIM_ON);
    lv_obj_add_event_cb(brightness_bar, brightness_bar_event_cb, LV_EVENT_ALL, NULL);
    /* The fill: one rounded child, left-anchored, width tracks the value (see
     * setting_bar_apply_fill). Created before the icon so the icon stays on top. */
    lv_obj_t *brightness_floor = lv_obj_create(brightness_bar);
    lv_obj_set_height(brightness_floor, 80);
    lv_obj_set_style_radius(brightness_floor, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(brightness_floor, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(brightness_floor, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(brightness_floor, 0, 0);
    lv_obj_clear_flag(brightness_floor, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(brightness_floor, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_t *brightness_icon = lv_img_create(brightness_bar);
    lv_img_set_src(brightness_icon, &sun);
    lv_obj_align(brightness_icon, LV_ALIGN_LEFT_MID, 20, 0);
    p_app_setting->brightness_bar = brightness_bar;
    p_app_setting->brightness_fill = brightness_floor;
    lv_obj_update_layout(brightness_bar); /* resolve width before the first fill calc */
    setting_bar_apply_fill(brightness_bar, brightness_floor);

    /* Screen time bar directly below the brightness bar (range 5..60, never@>=60) */
    const lv_coord_t screen_time_bar_w = LV_HOR_RES * 80 / 100;
    lv_obj_t *screen_time_bar = lv_bar_create(settings_container);
    lv_bar_set_range(screen_time_bar, 5, 100);
    lv_obj_set_size(screen_time_bar, screen_time_bar_w, 80);
    lv_obj_align_to(screen_time_bar, brightness_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_set_style_bg_color(screen_time_bar, lv_color_hex(0xCECECE), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(screen_time_bar, lv_color_hex(0xCECECE), LV_PART_MAIN);
    /* See brightness_bar: lv_bar draws nothing, the child fill below is the visual. */
    lv_obj_set_style_radius(screen_time_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen_time_bar, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(screen_time_bar, LV_OPA_TRANSP, LV_PART_MAIN);
    /* Clamp before initial set — oled_display_time may be 255 ("never") which is
     * out of bar range and would visually fill the whole bar at first entry. */
    {
        int32_t initial_val = SkaiWatchSys.oled_display_time;
        LOG_I("Initial screen timeout from sys: %d", initial_val);
        if (initial_val < 5) initial_val = 5;
        if (initial_val > 100) initial_val = 100;
        lv_bar_set_value(screen_time_bar, initial_val, LV_ANIM_OFF);
    }
    lv_obj_add_event_cb(screen_time_bar, screen_time_bar_event_cb, LV_EVENT_ALL, NULL);
    /* The fill (see brightness_bar / setting_bar_apply_fill). */
    lv_obj_t *screen_time_floor = lv_obj_create(screen_time_bar);
    lv_obj_set_height(screen_time_floor, 80);
    lv_obj_set_style_radius(screen_time_floor, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(screen_time_floor, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(screen_time_floor, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screen_time_floor, 0, 0);
    lv_obj_clear_flag(screen_time_floor, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(screen_time_floor, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_t *screen_time_icon = lv_img_create(screen_time_bar);
    lv_img_set_src(screen_time_icon, ICON_SLEEP_MODE);
    lv_obj_align(screen_time_icon, LV_ALIGN_LEFT_MID, 20, 0);
    p_app_setting->screen_time_bar = screen_time_bar;
    p_app_setting->screen_time_fill = screen_time_floor;
    lv_obj_update_layout(screen_time_bar); /* resolve width before the first fill calc */
    setting_bar_apply_fill(screen_time_bar, screen_time_floor);

    p_app_setting->screen_time_label = lv_label_create(screen_time_bar);
    lv_obj_set_style_text_color(p_app_setting->screen_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(p_app_setting->screen_time_label,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align(p_app_setting->screen_time_label, LV_ALIGN_CENTER, 50, 0);
    refresh_screen_time_label(SkaiWatchSys.oled_display_time);

    /* Find Phone wide widget (same width as the bars), icon on the left */
    const lv_coord_t find_phone_btn_w = LV_HOR_RES * 80 / 100;
    lv_obj_t *find_phone_btn = lv_btn_create(settings_container);
    lv_obj_set_size(find_phone_btn, find_phone_btn_w, 80);
    lv_obj_align_to(find_phone_btn, screen_time_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_add_event_cb(find_phone_btn, btn_find_phone_event_callback,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(find_phone_btn, 40, 0);
    lv_obj_set_style_bg_color(find_phone_btn, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(find_phone_btn, LV_OPA_10, 0);
    lv_obj_set_style_border_width(find_phone_btn, 0, 0);
    lv_obj_set_style_pad_all(find_phone_btn, 0, 0);
    lv_obj_t *find_phone_icon = lv_img_create(find_phone_btn);
    lv_img_set_src(find_phone_icon, FIND_PHONE);
    lv_obj_align(find_phone_icon, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_t *find_phone_label = lv_label_create(find_phone_btn);
    lv_label_set_text(find_phone_label, LV_EXT_STR_GET_BY_KEY(find_phone, "Find Phone"));
    lv_obj_set_style_text_color(find_phone_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(find_phone_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(find_phone_label, find_phone_icon, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

    /* QR Code wide widget (same width as the bars), icon on the left */
    const lv_coord_t qrcode_btn_w = LV_HOR_RES * 80 / 100;
    lv_obj_t *qrcode_btn = lv_btn_create(settings_container);
    lv_obj_set_size(qrcode_btn, qrcode_btn_w, 80);
    lv_obj_align_to(qrcode_btn, find_phone_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_add_event_cb(qrcode_btn, btn_qrcode_event_callback,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(qrcode_btn, 40, 0);
    lv_obj_set_style_bg_color(qrcode_btn, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(qrcode_btn, LV_OPA_10, 0);
    lv_obj_set_style_border_width(qrcode_btn, 0, 0);
    lv_obj_set_style_pad_all(qrcode_btn, 0, 0);
    lv_obj_t *qrcode_icon = lv_img_create(qrcode_btn);
    lv_img_set_src(qrcode_icon, ICON_QRCODE);
    lv_obj_align(qrcode_icon, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_t *qrcode_label = lv_label_create(qrcode_btn);
    lv_label_set_text(qrcode_label, LV_EXT_STR_GET_BY_KEY(qr_code, "QR Code"));
    lv_obj_set_style_text_color(qrcode_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(qrcode_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(qrcode_label, qrcode_icon, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

#if !kReleaseMode
    /* Gesture Test wide widget (dev only), same style as Find Phone / QR Code */
    const lv_coord_t gesture_btn_w = LV_HOR_RES * 80 / 100;
    lv_obj_t *gesture_btn = lv_btn_create(settings_container);
    lv_obj_set_size(gesture_btn, gesture_btn_w, 80);
    lv_obj_align_to(gesture_btn, qrcode_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_add_event_cb(gesture_btn, btn_gesture_test_event_callback,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(gesture_btn, 40, 0);
    lv_obj_set_style_bg_color(gesture_btn, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(gesture_btn, LV_OPA_10, 0);
    lv_obj_set_style_border_width(gesture_btn, 0, 0);
    lv_obj_set_style_pad_all(gesture_btn, 0, 0);
    lv_obj_t *gesture_icon = lv_img_create(gesture_btn);
    lv_img_set_src(gesture_icon, IMG_LOGO);
    lv_img_set_pivot(gesture_icon, 32, 32);
    lv_img_set_zoom(gesture_icon, 208); // Scale up the icon for better visibility
    lv_img_set_size_mode(gesture_icon, LV_IMG_SIZE_MODE_REAL);
    lv_obj_align(gesture_icon, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_t *gesture_label = lv_label_create(gesture_btn);
    lv_label_set_text(gesture_label, "Gesture Test");
    lv_obj_set_style_text_color(gesture_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(gesture_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(gesture_label, gesture_icon, LV_ALIGN_OUT_RIGHT_MID, 15, 0);
#endif

    /* DND Mode wide widget (icon + name on the left, switch on the right) */
    const lv_coord_t dnd_btn_w = LV_HOR_RES * 80 / 100;
    dnd_quick_btn = lv_obj_create(settings_container);
    lv_obj_set_size(dnd_quick_btn, dnd_btn_w, 80);
#if !kReleaseMode
    lv_obj_align_to(dnd_quick_btn, gesture_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
#else
    lv_obj_align_to(dnd_quick_btn, qrcode_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
#endif
    lv_obj_set_style_radius(dnd_quick_btn, 40, 0);
    lv_obj_set_style_bg_color(dnd_quick_btn, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(dnd_quick_btn, LV_OPA_10, 0);
    lv_obj_set_style_border_width(dnd_quick_btn, 0, 0);
    lv_obj_set_style_pad_all(dnd_quick_btn, 0, 0);
    lv_obj_clear_flag(dnd_quick_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dnd_icon = lv_img_create(dnd_quick_btn);
    lv_img_set_src(dnd_icon, ICON_DND_MODE);
    lv_obj_align(dnd_icon, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_t *dnd_label = lv_label_create(dnd_quick_btn);
    lv_label_set_text(dnd_label, LV_EXT_STR_GET_BY_KEY(dnd, "DND"));
    lv_obj_set_style_text_color(dnd_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(dnd_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(dnd_label, dnd_icon, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

    lv_obj_t *dnd_sw = lv_switch_create(dnd_quick_btn);
    lv_obj_set_size(dnd_sw, 80, 40);
    lv_obj_align(dnd_sw, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_color(dnd_sw, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(dnd_sw, lv_color_hex(0x0078D7),
                              LV_PART_INDICATOR | LV_STATE_CHECKED);
    if (SkaiWatchSys.DNDMode.config.status)
    {
        lv_obj_add_state(dnd_sw, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(dnd_sw, dnd_switch_event_callback,
                        LV_EVENT_VALUE_CHANGED, NULL);

    /* Time Format wide widget (icon + name on the left, switch on the right;
     * same look as DND. Switch CHECKED = 24h, unchecked = 12h.) */
    const lv_coord_t time_format_btn_w = LV_HOR_RES * 80 / 100;
    time_format_quick_btn = lv_obj_create(settings_container);
    lv_obj_set_size(time_format_quick_btn, time_format_btn_w, 80);
    lv_obj_align_to(time_format_quick_btn, dnd_quick_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_set_style_radius(time_format_quick_btn, 40, 0);
    lv_obj_set_style_bg_color(time_format_quick_btn, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(time_format_quick_btn, LV_OPA_10, 0);
    lv_obj_set_style_border_width(time_format_quick_btn, 0, 0);
    lv_obj_set_style_pad_all(time_format_quick_btn, 0, 0);
    lv_obj_clear_flag(time_format_quick_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *time_format_icon = lv_img_create(time_format_quick_btn);
    lv_img_set_src(time_format_icon, &service_hour);
    lv_obj_align(time_format_icon, LV_ALIGN_LEFT_MID, 20, 0);

    /* White circle badge at the bottom-right of the icon, showing 12 / 24 */
    lv_obj_t *time_format_badge = lv_obj_create(time_format_quick_btn);
    lv_obj_set_size(time_format_badge, 25, 25);
    lv_obj_align_to(time_format_badge, time_format_icon, LV_ALIGN_BOTTOM_RIGHT, -3, -3);
    lv_obj_set_style_radius(time_format_badge, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(time_format_badge, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(time_format_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(time_format_badge, 0, 0);
    lv_obj_set_style_pad_all(time_format_badge, 0, 0);
    lv_obj_clear_flag(time_format_badge, LV_OBJ_FLAG_SCROLLABLE);

    time_format_badge_label = lv_label_create(time_format_badge);
    lv_obj_set_style_text_color(time_format_badge_label, lv_color_hex(0x121212), 0);
    lv_obj_set_style_text_font(time_format_badge_label,
                               LV_EXT_FONT_GET(get_system_font_size(-3)), 0);
    lv_obj_center(time_format_badge_label);

    time_format_title_label = lv_label_create(time_format_quick_btn);
    lv_obj_set_style_text_color(time_format_title_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(time_format_title_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(time_format_title_label, time_format_icon, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    update_time_format_title_label(SkaiWatchSys.flag_field.hour_format);

    lv_obj_t *time_format_sw = lv_switch_create(time_format_quick_btn);
    lv_obj_set_size(time_format_sw, 80, 40);
    lv_obj_align(time_format_sw, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_color(time_format_sw, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(time_format_sw, lv_color_hex(0x0078D7),
                              LV_PART_INDICATOR | LV_STATE_CHECKED);
    if (SkaiWatchSys.flag_field.hour_format == 0x01)
    {
        lv_obj_add_state(time_format_sw, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(time_format_sw, time_format_switch_event_callback,
                        LV_EVENT_VALUE_CHANGED, NULL);

    /* Mouse Press Mode wide widget — toggle FSR sampler behavior
     * OFF: pressure<17000 → move, <10000 → also press left button (default)
     * ON : always allow move, pressure<17000 → press left button */
    const lv_coord_t mouse_press_btn_w = LV_HOR_RES * 80 / 100;
    mouse_press_quick_btn = lv_obj_create(settings_container);
    lv_obj_set_size(mouse_press_quick_btn, mouse_press_btn_w, 80);
    lv_obj_align_to(mouse_press_quick_btn, time_format_quick_btn,
                    LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_set_style_radius(mouse_press_quick_btn, 40, 0);
    lv_obj_set_style_bg_color(mouse_press_quick_btn, lv_color_hex(0xCECECE), 0);
    lv_obj_set_style_bg_opa(mouse_press_quick_btn, LV_OPA_10, 0);
    lv_obj_set_style_border_width(mouse_press_quick_btn, 0, 0);
    lv_obj_set_style_pad_all(mouse_press_quick_btn, 0, 0);
    lv_obj_clear_flag(mouse_press_quick_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *mouse_press_icon = lv_img_create(mouse_press_quick_btn);
    lv_img_set_src(mouse_press_icon, LV_EXT_IMG_GET(mouse_mode_icon));
    lv_obj_align(mouse_press_icon, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_t *mouse_press_label = lv_label_create(mouse_press_quick_btn);
    lv_label_set_text(mouse_press_label, LV_EXT_STR_GET_BY_KEY(mouse_press, "Mouse Press"));
    lv_obj_set_style_text_color(mouse_press_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(mouse_press_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align_to(mouse_press_label, mouse_press_icon,
                    LV_ALIGN_OUT_RIGHT_MID, 20, 0);

    lv_obj_t *mouse_press_sw = lv_switch_create(mouse_press_quick_btn);
    lv_obj_set_size(mouse_press_sw, 80, 40);
    lv_obj_align(mouse_press_sw, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_color(mouse_press_sw, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(mouse_press_sw, lv_color_hex(0x0078D7),
                              LV_PART_INDICATOR | LV_STATE_CHECKED);
    if (SkaiWatchSys.flag_field.mouse_press_mode)
    {
        lv_obj_add_state(mouse_press_sw, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(mouse_press_sw, mouse_press_switch_event_callback,
                        LV_EVENT_VALUE_CHANGED, NULL);

    /* Capsule list items (same style as the wide widgets above).
     * Items that navigate to another page get a ">" arrow on the right. */
    lv_obj_t *list_btn;

    // Gesture (launches gesture app)
    list_btn = create_capsule_item(settings_container, mouse_press_quick_btn,
                                   LV_EXT_IMG_GET(icon_release), LV_EXT_STR_GET_BY_KEY(gesture, "Gesture"),
                                   true);
    lv_obj_add_event_cb(list_btn, btn_gesture_event_callback,
                        LV_EVENT_SHORT_CLICKED, NULL);

    // Language settings (opens sub-window)
    list_btn = create_capsule_item(settings_container, list_btn,
                                   LV_EXT_IMG_GET(language),
                                   LV_EXT_STR_GET_BY_KEY(setting_language, "Language"),
                                   true);
    lv_obj_add_event_cb(list_btn, btn_lang_event_callback,
                        LV_EVENT_SHORT_CLICKED, NULL);

    // Font size (opens sub-window)
    list_btn = create_capsule_item(settings_container, list_btn,
                                   LV_EXT_IMG_GET(language),
                                   LV_EXT_STR_GET_BY_KEY(setting_font_size, "Font Size"),
                                   true);
    lv_obj_add_event_cb(list_btn, btn_font_size_event_callback,
                        LV_EVENT_SHORT_CLICKED, NULL);


    // Reset device (in-place msgbox, no arrow)
    list_btn = create_capsule_item(settings_container, list_btn,
                                   LV_EXT_IMG_GET(airplane), LV_EXT_STR_GET_BY_KEY(reset_device, "Reset device"),
                                   false);
    lv_obj_add_event_cb(list_btn, btn_clear_flash_event_callback,
                        LV_EVENT_SHORT_CLICKED, NULL);

#if !kReleaseMode
    // Development (opens sub-page)
    list_btn = create_capsule_item(settings_container, list_btn,
                                   LV_EXT_IMG_GET(airplane), "Development",
                                   true);
    lv_obj_add_event_cb(list_btn, btn_developer_event_callback,
                        LV_EVENT_SHORT_CLICKED, NULL);
#else
    // Restart (action only, no arrow)
    list_btn = create_capsule_item(settings_container, list_btn,
                                   LV_EXT_IMG_GET(airplane),
                                   LV_EXT_STR_GET_BY_KEY(restart, "restart"),
                                   false);
    lv_obj_add_event_cb(list_btn, btn_restart_event_callback,
                        LV_EVENT_SHORT_CLICKED, NULL);
#endif

#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
    /* Version footer at the very bottom of the settings page */
    lv_obj_t *version_label = lv_label_create(settings_container);
    char version_buf[32];
    snprintf(version_buf, sizeof(version_buf), LV_EXT_STR_GET_BY_KEY(version_fmt, "Version %d.%d.%d"),
             VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION);
    lv_label_set_text(version_label, version_buf);
    lv_obj_set_style_text_color(version_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(version_label,
                               LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_align_to(version_label, list_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
#endif

    // Add scrollbar styling for circular watch face - make it more subtle
    lv_obj_set_scrollbar_mode(settings_container, LV_SCROLLBAR_MODE_AUTO);

    // Store reference to main list container
    p_app_setting->list1 = settings_container;

    // Subscribe to power manager service to sync brightness bar
    p_app_setting->pwr_srv_hdl = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_setting->pwr_srv_hdl);
    ui_datac_subscribe(p_app_setting->pwr_srv_hdl, "powermgr",
                       setting_powermgr_srv_callback, 0);
}

static void on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_setting);
    p_app_setting = (app_setting_t *)lv_mem_alloc(sizeof(app_setting_t));
    memset(p_app_setting, 0, sizeof(app_setting_t));

    app_setting_init(NULL);
    gui_app_trans_anim_t enter_anim_cfg, exit_anim_cfg;

    gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
    gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_NONE);

    gui_app_set_enter_trans_anim(&enter_anim_cfg);
    gui_app_set_exit_trans_anim(&exit_anim_cfg);

    gui_app_set_trans_anim_prio(1, -1);
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
        if (DATA_CLIENT_INVALID_HANDLE != p_app_setting->pwr_srv_hdl)
        {
            datac_close(p_app_setting->pwr_srv_hdl);
            p_app_setting->pwr_srv_hdl = DATA_CLIENT_INVALID_HANDLE;
        }
        lv_obj_del(p_app_setting->main_window);
        p_app_setting->main_window = NULL;
        lv_mem_free(p_app_setting);
        p_app_setting = NULL;
    }
    dnd_quick_btn = NULL;
    time_format_quick_btn = NULL;
    time_format_title_label = NULL;
    time_format_badge_label = NULL;
    mouse_press_quick_btn = NULL;
    /* The modal is parented to lv_scr_act() (and gets deleted with the screen),
     * but reset the static pointer so a stale reference isn't reused. */
    reset_modal = NULL;
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
        /* app_run 直接開啟不經 Main 狀態機，左緣右滑返回 bar 仍隱藏，這裡補開 */
        extern void display_gesture_detect_objs(uint32_t idx, bool display);
        display_gesture_detect_objs(0, true);
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