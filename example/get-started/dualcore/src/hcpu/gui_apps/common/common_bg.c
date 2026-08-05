/**
 ******************************************************************************
 * @file   common_bg.c
 * @author Skaiwalk software development team
 * @brief  Common background and container creation utilities
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

#include <rtthread.h>
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "lv_simplified_obj.h"

/**
 * @brief Create a black background
 *
 * @param par Parent object
 * @return lv_obj_t* Background object
 */
lv_obj_t *common_black_bg(lv_obj_t *par)
{
    lv_obj_t *bg = lv_obj_create(par);
    lv_obj_set_size(bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x000000), 0);

    return bg;
}

/**
 * @brief Create a common container with specified size, event callback and color
 *
 * @param parent Parent object
 * @param w Width
 * @param h Height
 * @param event_cb Event callback
 * @param color Background color
 * @return lv_obj_t* Container object
 */
lv_obj_t *common_container(lv_obj_t *parent, lv_coord_t w, lv_coord_t h, lv_event_cb_t event_cb, lv_color_t color)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, w, h);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);

    if (event_cb)
    {
        lv_obj_add_event_cb(cont, event_cb, LV_EVENT_SHORT_CLICKED, NULL);
    }

    lv_obj_set_style_bg_color(cont, color, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_20, 0);

    return cont;
}

/**
 * @brief Create a list widget with style and position
 *
 * @param par Parent object
 * @param style Style to apply
 * @param pos_x X position
 * @param pos_y Y position
 * @return lv_obj_t* Widget object
 */
lv_obj_t *common_list_widget(lv_obj_t *par, lv_style_t *style, lv_coord_t pos_x, lv_coord_t pos_y)
{
    lv_obj_t *widget = lv_obj_create(par);
    lv_obj_add_style(widget, style, 0);
    lv_obj_set_pos(widget, pos_x, pos_y);
    lv_obj_add_flag(widget, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(widget, LV_OBJ_FLAG_SCROLLABLE);

    return widget;
}

/**
 * @brief Create a common widget container (350x250)
 *
 * @param parent Parent object
 * @return lv_obj_t* Widget container object
 */
lv_obj_t *common_widget_container(lv_obj_t *parent)
{
    lv_obj_t *widget = lv_obj_create(parent);
    lv_obj_set_size(widget, 450, 250);
    lv_obj_set_style_bg_color(widget, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(widget, LV_OPA_100, 0);
    lv_obj_set_style_radius(widget, 50, LV_PART_MAIN);

    return widget;
}

// Helper function to create a divider line that can be placed outside a list item
lv_obj_t *create_divider_line(lv_obj_t *parent)
{
    lv_obj_t *line = lv_line_create(parent);
    static lv_point_precise_t line_points[] = {
        {0, 0},
        {(lv_coord_t)(LV_HOR_RES_MAX * 0.85 - 30), 0}};
    lv_line_set_points(line, line_points, 2);
    lv_obj_set_style_line_width(line, 1, 0);
    lv_obj_set_style_line_color(line, lv_color_hex(0x2A2A2A), 0); // Subtle divider
    return line;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/