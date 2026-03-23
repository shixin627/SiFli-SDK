/**
 ******************************************************************************
 * @file   skaiwalk_demo.c
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
#if !kReleaseMode
#include <rtdevice.h>
#include <stdio.h>
#include <string.h> // Added for string functions
#include <stdlib.h> // Added for malloc/free
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "bloc_control.h"
#include "bloc_peripheral.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "watch_system_interact.h"
#ifdef BSP_USING_COMMUNICATE
#include "communicate_protocol.h"
#endif
#ifdef BSP_USING_BLOC
#include "bloc_setting.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif

#define APP_ID "Demo"
void create_demo_screen(lv_obj_t *scr)
{
    lv_obj_t *bg = lv_img_create(scr);
    lv_img_set_src(bg, "/assets/images/common_background_blue.bin");
    lv_obj_align(bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *img = lv_img_create(bg);
    lv_img_set_src(img, IMG_LOGO);
    lv_obj_center(img);
}

#if LV_USE_IMGFONT
// LV_IMG_DECLARE(emoji_F617)
LV_IMG_DECLARE(F617)

static bool get_imgfont_path(const lv_font_t *font, void *img_src,
                             uint16_t len, uint32_t unicode, uint32_t unicode_next)
{
    LV_UNUSED(font);
    LV_UNUSED(unicode_next);
    LV_ASSERT_NULL(img_src);

    if (unicode == 0xF617)
    {
        memcpy(img_src, &F617, sizeof(lv_img_dsc_t));
    }
    else
    {
        char *path = (char *)img_src;
        snprintf(path, len, "%s/%04X.%s", "A:lvgl/examples/assets/emoji", unicode, "png");
        path[len - 1] = '\0';
    }

    return true;
}

/**
 * draw img in label or span obj
 */
void lv_example_imgfont_1(void)
{
    lv_font_t *imgfont = lv_imgfont_create(80, get_imgfont_path);
    if (imgfont == NULL)
    {
        LV_LOG_ERROR("imgfont init error");
    }

    imgfont->fallback = LV_FONT_DEFAULT;

    lv_obj_t *label1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label1, "12\uF600\uF617AB");
    lv_obj_set_style_text_font(label1, imgfont, LV_PART_MAIN);
    lv_obj_center(label1);
}

#else

void test_emoji_ttf_display(void)
{
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, 360, 360);
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(cont, 8, 0);

    /* FONT_SMALL (20px) - text mixed with emoji */
    lv_obj_t *label_s = lv_label_create(cont);
    lv_obj_set_style_text_font(label_s, LV_EXT_FONT_GET(FONT_SMALL), 0);
    lv_obj_set_style_text_color(label_s, lv_color_white(), 0);
    lv_label_set_text(label_s, "Small \xF0\x9F\x98\x80\xF0\x9F\x98\x83\xF0\x9F\x98\x84"); // 😀😃😄

    /* FONT_NORMAL (24px) */
    lv_obj_t *label_n = lv_label_create(cont);
    lv_obj_set_style_text_font(label_n, LV_EXT_FONT_GET(FONT_NORMAL), 0);
    lv_obj_set_style_text_color(label_n, lv_color_white(), 0);
    lv_label_set_text(label_n, "Normal \xF0\x9F\x98\x82\xF0\x9F\x98\x8D\xF0\x9F\x98\x8E"); // 😂😍😎

    /* FONT_SUBTITLE (28px) */
    lv_obj_t *label_sub = lv_label_create(cont);
    lv_obj_set_style_text_font(label_sub, LV_EXT_FONT_GET(FONT_SUBTITLE), 0);
    lv_obj_set_style_text_color(label_sub, lv_color_white(), 0);
    lv_label_set_text(label_sub, "Subtitle \xF0\x9F\x98\x98\xF0\x9F\x98\xA2"); // 😘😢

    /* FONT_TITLE (36px) */
    lv_obj_t *label_t = lv_label_create(cont);
    lv_obj_set_style_text_font(label_t, LV_EXT_FONT_GET(FONT_TITLE), 0);
    lv_obj_set_style_text_color(label_t, lv_color_white(), 0);
    lv_label_set_text(label_t, "Title \xF0\x9F\x98\xB1\xF0\x9F\x98\xA1"); // 😱😡

    /* Emoji-only row */
    lv_obj_t *label_emoji = lv_label_create(cont);
    lv_obj_set_style_text_font(label_emoji, LV_EXT_FONT_GET(FONT_NORMAL), 0);
    lv_obj_set_style_text_color(label_emoji, lv_color_white(), 0);
    lv_label_set_text(label_emoji, "😀😁😂😃😄😅😆😉😊😍");                                 
}

#endif

static void on_start(lv_obj_t *scr)
{
    // create_demo_screen(scr);
    test_emoji_ttf_display();
}

static void on_resume(void)
{
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
    gui_app_regist_msg_handler(APP_ID, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(skaiwalk_demo), IMG_LOGO, APP_ID, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/