/**
 ******************************************************************************
 * @file   app_gesture.h
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

#ifndef __APP_GESTURE_H__
#define __APP_GESTURE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include "lvgl.h"
#include "gui_app_fwk.h"

// Application ID
#define APP_ID_GESTURE "gesture"

// Gesture detection constants
#define GESTURE_TIMER_PERIOD_MS    50    // Timer period for gesture detection
#define GESTURE_CIRCLE_SIZE        120   // Size of the gesture circle
#define GESTURE_CIRCLE_RADIUS      60    // Radius of the gesture circle

// Color definitions
#define GESTURE_COLOR_GRAY         lv_color_make(128, 128, 128)  // Default gray color
#define GESTURE_COLOR_LIGHT_BLUE   lv_color_make(173, 216, 230)  // Light blue when pressed
#define GESTURE_COLOR_BUTTON       lv_color_make(70, 130, 180)   // Button color
#define GESTURE_COLOR_WHITE        lv_color_white()
#define GESTURE_COLOR_BLACK        lv_color_black()

// Function declarations
int app_gesture_main(intent_t i);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GESTURE_H__ */ 