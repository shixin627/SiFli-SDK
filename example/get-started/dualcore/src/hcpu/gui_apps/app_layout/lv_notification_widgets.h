/**
 ******************************************************************************
 * @file   lv_notification_widgets.h
 * @author Skaiwalk software development team
 * @brief  Notification widget definitions and declarations
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

#ifndef LV_NOTIFICATION_WIDGETS_H
#define LV_NOTIFICATION_WIDGETS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief Create a latest notification widget similar to calendar widget style
 * @param parent Parent object
 * @return Created widget object
 * 
 * This widget displays the latest notification with its icon, truncated title,
 * and count of additional notifications if more than one exists.
 * If no notifications exist, it shows a "No new messages" state.
 * 
 * Features:
 * - Shows icon of the latest notification
 * - Displays truncated title (max 12 characters + "...")
 * - Shows count of additional notifications (+N)
 * - Falls back to "No new messages" when empty
 * 
 * Usage example:
 * ```c
 * lv_obj_t *parent = lv_scr_act();
 * lv_obj_t *notification_widget = lv_latest_notification_widget_create(parent);
 * lv_obj_set_pos(notification_widget, 100, 100);
 * ```
 */
lv_obj_t *lv_latest_notification_widget_create(lv_obj_t *parent);

/**
 * @brief Create a simple notification count widget
 * @param parent Parent object
 * @return Created widget object
 * 
 * This widget displays only the count of notifications in a simple format.
 * Shows the number and "message"/"messages" text.
 * 
 * Features:
 * - Shows notification count as a number
 * - Displays "message" (singular) or "messages" (plural)
 * - Simple, minimal design
 * 
 * Usage example:
 * ```c
 * lv_obj_t *parent = lv_scr_act();
 * lv_obj_t *count_widget = lv_notification_count_widget_create(parent);
 * lv_obj_set_pos(count_widget, 200, 100);
 * ```
 */
lv_obj_t *lv_notification_count_widget_create(lv_obj_t *parent);

/**
 * @brief Build the dial header widget on the clock face.
 *
 * Priority rules:
 * - Music playing: show music title + album art image
 * - No music, notifications exist: show latest notification icon + title
 * - Neither: hidden
 */
void lv_dial_media_header_builder(lv_obj_t *parent);
void dial_media_header_init(void);
void dial_media_header_deinit(void);
void dial_header_on_suspend(void);
void dial_header_on_resume(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_NOTIFICATION_WIDGETS_H*/

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
