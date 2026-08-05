/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LVSF_EZIPA_H__
#define __LVSF_EZIPA_H__

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "string.h"
#include "rtconfig.h"
#include "lvgl.h"
/* v9 moved lv_obj_t into a private header; widgets embedding it need this. */
#include "lvgl_private.h"

#if USING_EZIPA_DEC != 0
extern const lv_obj_class_t lv_ezipa_class;

#if !defined(PY_GEN)
#include "ezipa_dec.h"

/*********************
 *      DEFINES
 *********************/
#if(16 == LV_COLOR_DEPTH)
#define EZIPA_OUTPUT_CF EZIPA_RGB565
#define CANVAS_CF      EPIC_OUTPUT_RGB565
#elif (24 == LV_COLOR_DEPTH)
#define EZIPA_OUTPUT_CF EZIPA_RGB888
#define CANVAS_CF      EPIC_OUTPUT_RGB888
#else
#error "Unsupport format!"
#endif
#endif

/**********************
 *      TYPEDEFS
 **********************/
typedef void (*lv_ezipa_end_cb_t)(lv_obj_t *ezipa);

#define EZIPA_LOOP_FOREVER          (-1)
#define EZIPA_LOOP_COUNT_DEINIT     (-1)

typedef enum
{
    LV_EZIPA_STOP,
    LV_EZIPA_CURR,
    LV_EZIPA_NEXT,
} lv_ezipa_status_t;

typedef struct
{
    /*No inherited ext. because inherited from the base object*/ /*Ext. of ancestor*/
    /*New data for this type */
    lv_obj_t                         img;
    lv_anim_t                        anim;
#if !defined(PY_GEN)
    ezipa_obj_t                     *ezipa_dec;
#endif
    lv_ezipa_status_t                status;
    /** interval in millisecond, if the value is greater than 0,
        use this value rather than interval extracted from apng data */
    int32_t                          interval;
    lv_obj_t                        *surface;
    const void                      *ezipa_src;
    int                              loop_cnt;
    int                              loop_times;//-1 for loop forever, 0 for loop 1 time
    lv_ezipa_end_cb_t                play_end;
    lv_img_dsc_t                     img_dsc;
    const void                      *orig_src;
    void                            *prefix;          //for ezipa
    void                            *surface_prefix;  //for surface
    void                            *splice_src;
    void                            *surface_src;
    uint16_t                         max_num;
    uint16_t                         delay_play_time;
    uint16_t                         delay_play_num;
} lv_ezipa_ext_t;


/**********************
 * GLOBAL PROTOTYPES
 **********************/

lv_obj_t *lv_ezipa_create(lv_obj_t *parent);

void lv_ezipa_set_src(lv_obj_t *ezipa, const char *src);

void lv_ezipa_set_surface(lv_obj_t *ezipa, const void *src);

void lv_ezipa_pause(lv_obj_t *ezipa);

void lv_ezipa_resume(lv_obj_t *ezipa);

void lv_ezipa_resume_with_delay(lv_obj_t *ezipa, uint16_t delay_time);

void lv_ezipa_set_loop_times(lv_obj_t *ezipa, int times);

void lv_ezipa_set_play_end_cb(lv_obj_t *ezipa, lv_ezipa_end_cb_t cb);

void lv_ezipa_set_interval(lv_obj_t *ezipa, int32_t interval);

void lv_ezipa_set_zoom(lv_obj_t *ezipa, uint16_t zoom);

void lv_ezipa_set_opa(lv_obj_t *ezipa, uint16_t opa);

//select is useful only when prefix set, if loop_times is set, playback for set times
void lv_ezipa_select(lv_obj_t *ezipa, uint8_t idx);

//only support in nand fs, max_num: max is 99
void lv_ezipa_set_select_prefix(lv_obj_t *ezipa, const void *ezipa_prefix, const void *surface_prefix, uint8_t max_num);

#endif

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*__LVSF_EZIPA_H__*/


