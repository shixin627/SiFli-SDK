/**
 ******************************************************************************
 * @file   common_button.c
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
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"

/*********************
 *      DEFINES
 *********************/
#define DEFAULT_BUTTON_WIDTH 366
#define DEFAULT_BUTTON_HEIGHT 100

/*********************
 *      STYLES
 *********************/
static const lv_style_const_prop_t BUTTON_STYLE_PROPS[] = {
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_CONST_BG_OPA(LV_OPA_0),
    LV_STYLE_CONST_RADIUS(10),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t CUSTOM_BUTTON_STYLE_PROPS[] = {
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(255, 255, 255)),
    LV_STYLE_CONST_BG_OPA(LV_OPA_10),
    LV_STYLE_CONST_RADIUS(30),
    LV_STYLE_CONST_SHADOW_WIDTH(30),
    LV_STYLE_CONST_SHADOW_COLOR(LV_COLOR_MAKE(255, 255, 255)),
    LV_STYLE_CONST_SHADOW_OPA(LV_OPA_0),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(COMMON_BUTTON_STYLE, BUTTON_STYLE_PROPS);
LV_STYLE_CONST_INIT(CUSTOM_BUTTON_STYLE, CUSTOM_BUTTON_STYLE_PROPS);

/*********************
 *   STATIC FUNCTIONS
 *********************/

/**
 * @brief Apply custom button styling to an object
 * @param obj Button object to style
 * @param radius Radius value for button corners
 */
static void apply_custom_button_style(lv_obj_t *obj, lv_coord_t radius)
{
    lv_obj_set_style_radius(obj, radius, 0);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_50, 0);
}

/**
 * @brief Create a simple button with basic styling
 * @param par Parent object
 * @return Created button object
 */
static lv_obj_t *common_simple_button(lv_obj_t *par)
{
    lv_obj_t *child = lv_btn_create(par);
    lv_obj_add_style(child, (lv_style_t *)&COMMON_BUTTON_STYLE, 0);
    return child;
}

/*********************
 *   PUBLIC FUNCTIONS
 *********************/

/**
 * @brief Create a button with flex layout
 * @param par Parent object
 * @param custom_style Whether to apply custom styling
 * @param radius Radius for button corners (ignored if custom_style is false)
 * @return Created button object
 */
lv_obj_t *common_flex_button(lv_obj_t *par, bool custom_style, lv_coord_t radius)
{
    lv_obj_t *child = common_simple_button(par);
    lv_obj_set_layout(child, LV_LAYOUT_FLEX);
    lv_obj_set_align(child, LV_ALIGN_CENTER);

    if (custom_style)
    {
        apply_custom_button_style(child, radius);
    }

    return child;
}

/**
 * @brief Create a button with an icon
 * @param par Parent object
 * @param img_src Image source for the icon
 * @param event_cb Event callback function
 * @return Created button object
 */
lv_obj_t *common_icon_button(lv_obj_t *par, const void *img_src, lv_event_cb_t event_cb)
{
    lv_obj_t *icon = common_flex_button(par, false, 0);
    lv_obj_t *img = lv_img_create(icon);
    lv_img_set_src(img, img_src);
    lv_obj_add_event_cb(icon, event_cb, LV_EVENT_CLICKED, NULL);
    return icon;
}

/**
 * @brief Create a button with a background image
 * @param par Parent object
 * @param img_src Image source for background
 * @param w Width of button (use 0 for default)
 * @param h Height of button (use 0 for default)
 * @param event_cb Event callback function
 * @return Created button object
 */
lv_obj_t *common_image_button(lv_obj_t *par, const void *img_src, lv_coord_t w, lv_coord_t h, lv_event_cb_t event_cb)
{
    lv_obj_t *button = lv_btn_create(par);
    uint16_t width = (w > 0) ? w : DEFAULT_BUTTON_WIDTH;
    uint16_t height = (h > 0) ? h : DEFAULT_BUTTON_HEIGHT;

    lv_obj_set_size(button, width, height);
    lv_obj_add_flag(button, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_event_cb(button, event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(button, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_10, 0);
    // apply_custom_button_style(button, LV_RADIUS_CIRCLE);
    // lv_obj_set_style_bg_img_src(button, img_src, 0);
    lv_obj_t *img = lv_img_create(button);
    lv_img_set_src(img, img_src);
    lv_obj_center(img);

    return button;
}

/**
 * @brief Create a button with text label
 * @param par Parent object
 * @param text Text to display on button
 * @param font_size Font size (use 0 for default)
 * @param w Width of button (use 0 for default)
 * @param h Height of button (use 0 for default)
 * @param event_cb Event callback function
 * @return Created button object
 */
lv_obj_t *common_text_button(lv_obj_t *par, const char *text, uint8_t font_size, lv_coord_t w, lv_coord_t h, lv_event_cb_t event_cb)
{
    lv_obj_t *button = lv_btn_create(par);
    uint16_t width = (w > 0) ? w : DEFAULT_BUTTON_WIDTH;
    uint16_t height = (h > 0) ? h : DEFAULT_BUTTON_HEIGHT;

    lv_obj_set_size(button, width, height);
    lv_obj_add_event_cb(button, event_cb, LV_EVENT_CLICKED, NULL);

    apply_custom_button_style(button, 50);

    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);

    if (font_size > 0)
    {
        lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(font_size), 0);
    }
    else
    {
        lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(FONT_TITLE), 0);
    }

    lv_obj_center(label);
    return button;
}

// For toggle switches, we need a special create function with dark theme styling
lv_obj_t *create_dark_toggle_item(lv_obj_t *parent, const void *icon, const char *text, bool initial_state, uint8_t font_size, bool show_divider)
{
    lv_obj_t *item = lv_obj_create(parent);
    lv_obj_set_size(item, LV_PCT(100), BUTTON_HEIGHT);          // Larger height for better touch targets
    lv_obj_set_style_bg_color(item, lv_color_hex(0x1E1E1E), 0); // Dark gray
    lv_obj_set_style_border_width(item, 0, 0);
    lv_obj_set_style_pad_left(item, 20, 0);
    lv_obj_set_style_pad_right(item, 20, 0);
    lv_obj_set_style_radius(item, 12, 0); // Rounded corners for better look

    lv_obj_t *label = lv_label_create(item);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0); // White text
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(font_size), 0);

    if (icon)
    {
        lv_obj_t *img = lv_img_create(item);
        lv_img_set_src(img, icon);
        lv_obj_align(img, LV_ALIGN_LEFT_MID, 5, 0);
        lv_obj_align_to(label, img, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    }
    else
    {
        lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    }

    // Create dark theme switch
    lv_obj_t *sw = lv_switch_create(item);
    lv_obj_set_size(sw, 120, LV_PCT(70)); // Larger switch for better touch target
    lv_obj_add_flag(sw, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0x333333), LV_PART_MAIN);                         // Dark gray for off state
    lv_obj_set_style_bg_color(sw, lv_color_hex(0x0078D7), LV_PART_INDICATOR | LV_STATE_CHECKED); // Blue accent when on
    lv_obj_align(sw, LV_ALIGN_RIGHT_MID, -10, 0);

    if (initial_state)
    {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
    }

    if (show_divider)
    {
        lv_obj_t *divider = create_divider_line(parent);
    }
    return sw;
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/