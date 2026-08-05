/**
 ******************************************************************************
 * @file   app_calculator.c
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "bloc_motion_tracking.h"
#include "bloc_setting.h"
#include "ui_img_helper.h"

#define DBG_TAG "app.calculator"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_CALCULATOR

#define MAX_NUM 10 // 设置文本框显示最大位数

static lv_obj_t *ta = NULL;      // 文本框对象
static lv_obj_t *my_btnm = NULL; // 按钮对象
static char error = 0;
static double before = 0;
static char operate = '\0';
static char num[MAX_NUM];

static double save_num(const char *num) // 将字符串保存成浮点数
{
    return atof(num);
}

static char judge_num(double num) // 比较数字尾数有没有超过指定数目
{
    char n[100];
    sprintf(n, "%g", num);
    LOG_D("%d", strlen(n));
    if (strlen(n) < MAX_NUM)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

static void calculate(double before, double now, char operate, char *num) // 计算浮点数
{
    double result = now;
    switch (operate)
    {
    case '+':
        result = before + now;
        break;
    case '-':
        result = before - now;
        break;
    case '*':
        result = before * now;
        break;
    case '/':
        result = before / now;
        break;
    default:
        break;
    }
    LOG_D("result:%lf", result);
    sprintf(num, "%g", result);
}

static void clear_result(void) // 清空文本框
{
    memset(num, 0, MAX_NUM);
    before = 0;
    error = 0;
    operate = '\0';
}

static void handle_button_text(const char *txt)
{
    volatile double now = 0;

    LOG_D("Button %s is clicked", txt);
    if (error == 0)
    {
        switch (txt[0])
        {
        case '=':
            if (judge_num(before) == 0)
            {
                now = save_num(lv_textarea_get_text(ta));
                calculate(before, now, operate, num);
                operate = '\0';
                before = save_num(num);
                lv_textarea_set_text(ta, num);
                lv_textarea_set_cursor_pos(ta, LV_TEXTAREA_CURSOR_LAST);
            }
            else
            {
                error = 1;
                lv_textarea_set_text(ta, "EOR");
                LOG_E("error");
            }
            break;
        case 'D':
            lv_textarea_del_char(ta);
            break;
        case '+':
        case '-':
        case 'x':
        case '/':
            now = save_num(lv_textarea_get_text(ta));
            if (operate != '\0')
            {
                calculate(before, now, operate, num);
                before = save_num(num);
            }
            else
            {
                before = now;
            }
            operate = (txt[0] == 'x') ? '*' : txt[0];
            lv_textarea_set_text(ta, "");
            break;
        case ' ':
            break;
        default:
            lv_textarea_add_char(ta, txt[0]);
            break;
        }
    }
    if (txt[0] == 'C')
    {
        clear_result();
        lv_textarea_set_text(ta, "");
        now = 0;
    }
    LOG_D("before:%lf, now:%lf", before, now);
}

static void clear_handler(lv_event_t *event)
{
    if (event->code == LV_EVENT_CLICKED)
    {
        handle_button_text("C");
    }
}

static void delete_handler(lv_event_t *event)
{
    if (event->code == LV_EVENT_CLICKED)
    {
        handle_button_text("D");
    }
}

static void equal_handler(lv_event_t *event)
{
    if (event->code == LV_EVENT_CLICKED)
    {
        handle_button_text("=");
    }
}
static void event_handler(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event); // 获取事件目标
    if (event->code == LV_EVENT_CLICKED)
    {
        uint16_t id = lv_btnmatrix_get_selected_btn(obj);
        const char *txt = lv_btnmatrix_get_btn_text(obj, id);
        handle_button_text(txt);
    }
}
static lv_obj_t *lv_ex_textarea(lv_obj_t *screen)
{
    // Create the text area
    lv_obj_t *ta = lv_textarea_create(screen);
    lv_textarea_set_accepted_chars(ta, "0123456789+-.*/EOR");
    lv_textarea_set_max_length(ta, MAX_NUM);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_text(ta, "");

    // Set display style
    lv_obj_set_style_bg_color(ta, lv_color_hex(0x2D2D2D), 0);
    lv_obj_set_style_bg_opa(ta, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(ta, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ta, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_align(ta, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_radius(ta, 10, 0);
    lv_obj_set_style_pad_all(ta, 2, 0);

    lv_obj_set_width(ta, lv_pct(53));
    lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 50); // Adjust position to be closer to the button matrix

    // create a clear button on the left side of text area
    lv_obj_t *clear_btn = common_text_button(screen, "C", get_system_font_size(0), 50, 50, clear_handler);
    lv_obj_set_style_text_color(clear_btn, lv_color_hex(0xC17E7C), 0);
    lv_obj_set_style_bg_opa(clear_btn, LV_OPA_TRANSP, 0);
    lv_obj_align_to(clear_btn, ta, LV_ALIGN_OUT_LEFT_MID, 0, 0);

    // create a delete image button on the right side of text area
    lv_obj_t *float_btn = lv_btn_create(screen);
    lv_obj_set_size(float_btn, 50, 50);
    lv_obj_add_flag(float_btn, LV_OBJ_FLAG_FLOATING);
    lv_obj_align_to(float_btn, ta, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(float_btn, delete_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(float_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(float_btn, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(float_btn, LV_OPA_TRANSP, 0);
    return ta;
}

static const lv_style_const_prop_t BTNM_BG_STYLE_PROPS[] = {
    LV_STYLE_CONST_PAD_COLUMN(4),
    LV_STYLE_CONST_PAD_ROW(4),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0x00, 0x00, 0x00)), // Black background
    LV_STYLE_CONST_BG_OPA(LV_OPA_TRANSP),
    LV_STYLE_CONST_RADIUS(120), // Circular shape
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t BTNM_KEY_STYLE_PROPS[] = {
    LV_STYLE_CONST_RADIUS(15),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0x33, 0x33, 0x33)),   // Dark gray buttons
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)), // White text
    LV_STYLE_CONST_BG_OPA(LV_OPA_TRANSP),
    LV_STYLE_PROP_INV,
};

// Add a new style for pressed buttons
static const lv_style_const_prop_t BTNM_KEY_PRESSED_STYLE_PROPS[] = {
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0x4A, 0x4A, 0x4A)), // Lighter gray when pressed
    LV_STYLE_CONST_BG_OPA(LV_OPA_50),                         // Semi-transparent when pressed
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(BTNM_BG_STYLE, BTNM_BG_STYLE_PROPS);
LV_STYLE_CONST_INIT(BTNM_KEY_STYLE, BTNM_KEY_STYLE_PROPS);
LV_STYLE_CONST_INIT(BTNM_KEY_PRESSED_STYLE, BTNM_KEY_PRESSED_STYLE_PROPS);

static const char *btnm_map[] = {
    "7", "8", "9", "+", "\n",
    "4", "5", "6", "-", "\n",
    "1", "2", "3", "x", "\n",
    " ", "0", ".", "/", ""}; // 将 "0" 移动到 "2" 的正下方

static lv_obj_t *lv_ex_btnmatrix(lv_obj_t *screen)
{
    lv_obj_t *btnm = lv_btnmatrix_create(screen);
    lv_btnmatrix_set_map(btnm, btnm_map);

    // Apply circular styling
    lv_obj_add_style(btnm, (lv_style_t *)&BTNM_BG_STYLE, LV_PART_MAIN);
    lv_obj_add_style(btnm, (lv_style_t *)&BTNM_KEY_STYLE, LV_PART_ITEMS);
    // Add pressed state style to make buttons light up when pressed
    lv_obj_add_style(btnm, (lv_style_t *)&BTNM_KEY_PRESSED_STYLE, LV_PART_ITEMS | LV_STATE_PRESSED);

    lv_obj_set_style_text_font(btnm, LV_EXT_FONT_GET(FONT_HUGE), LV_PART_ITEMS);

    // Set operator buttons text to cyan color
    lv_obj_set_style_text_color(btnm, lv_color_hex(0x3A6D7B), LV_PART_ITEMS | LV_STATE_CHECKED);
    // Set operator buttons bg to transparent color
    lv_obj_set_style_bg_color(btnm, lv_color_hex(0x000000), LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(btnm, LV_OPA_TRANSP, LV_PART_ITEMS | LV_STATE_CHECKED);

    // Mark operator buttons
    lv_btnmatrix_set_btn_ctrl(btnm, 3, LV_BTNMATRIX_CTRL_CHECKED);  // "+"
    lv_btnmatrix_set_btn_ctrl(btnm, 7, LV_BTNMATRIX_CTRL_CHECKED);  // "-"
    lv_btnmatrix_set_btn_ctrl(btnm, 11, LV_BTNMATRIX_CTRL_CHECKED); // "*"
    lv_btnmatrix_set_btn_ctrl(btnm, 15, LV_BTNMATRIX_CTRL_CHECKED); // "/"

    // Add event callback
    lv_obj_add_event_cb(btnm, event_handler, LV_EVENT_ALL, NULL);

    // Set size and position of the button matrix
    lv_obj_set_size(btnm, 380, 360);              // Increase size to make buttons larger
    lv_obj_align(btnm, LV_ALIGN_CENTER, -30, 50); // Position at the center

    return btnm;
}

static lv_point_precise_t _separator_line_points[] = {{0, 0}, {400, 0}};
void lv_create_calculator_screen(lv_obj_t *scr)
{
    ta = lv_ex_textarea(scr);

    // Create a separator line
    lv_obj_t *line = lv_line_create(scr);
    lv_line_set_points(line, _separator_line_points, 2);
    lv_obj_set_style_line_color(line, lv_color_hex(0x404040), 0); // Gray color
    lv_obj_set_style_line_width(line, 2, 0);
    lv_obj_align_to(line, ta, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    my_btnm = lv_ex_btnmatrix(scr);

    // create a equal button on the right side of screen
    lv_obj_t *equal_btn = common_text_button(scr, "=", get_system_font_size(2), 60, 120, equal_handler);
    lv_obj_set_style_bg_color(equal_btn, lv_color_hex(0x3A6D7B), 0);
    lv_obj_set_style_bg_opa(equal_btn, LV_OPA_100, 0);
    lv_obj_align(equal_btn, LV_ALIGN_RIGHT_MID, -30, 0);
}

static void on_start(void)
{
    lv_obj_t *scr = lv_scr_act();
    clear_result();
    lv_create_calculator_screen(scr);
}

static void on_resume(void)
{
    set_open_control_options(false);
    set_free_control_with_arm(false);
    setting_provider.set_power_save_mode(0);
}

static void on_pause(void)
{
    setting_provider.set_power_save_mode(1);
}

static void on_stop(void)
{
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
        on_start();
        break;
    }

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
    gui_app_regist_msg_handler(APP_ID_CALCULATOR, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(calculator), IMG_CALCULATOR, APP_ID_CALCULATOR, app_main, 1);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
