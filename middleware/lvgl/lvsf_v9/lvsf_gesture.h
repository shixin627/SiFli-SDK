/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_GESTURE_H
#define LVSF_GESTURE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
/* This header names lv_obj_t and bool, and is included from translation units
   that have not pulled lvgl.h in yet. */
#include <stdbool.h>
#include <stdint.h>
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

void lvsf_gesture_init(lv_obj_t *parent);
void lvsf_gesture_deinit(void);

void lvsf_gesture_set_image(uint32_t idx, const void *src_img);

void lvsf_gesture_disable(void);
void lvsf_gesture_enable(void);
void lvsf_gesture_bars_realign(void);

/* Declared here rather than as a local `extern` at each call site, which is
   how the v8 tree reached them. */
uint8_t lvsf_gesture_enable_register(uint8_t enable);
void    lvsf_gesture_bring_to_front(void);
void    display_gesture_detect_objs(uint32_t idx, bool display);
extern lv_obj_t *gesture_detect_objs[4];
extern lv_obj_t *gesture_img_objs[4];
extern bool      gesture_is_active;


/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_GESTURE_H*/






