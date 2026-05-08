/**
 ******************************************************************************
 * @file   app_mainmenu.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/********************
 *		INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "lv_timer.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#ifdef DL_APP_SUPPORT
    #include "gui_dl_app_utils.h"
    #include "dlfcn.h"
#endif

#include "lv_ext_resource_manager.h"
#include "bf0_lib.h"
#include "custom_trans_anim.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
#endif
#include "ui_helper.h"
#include "bloc_setting.h"
#include "bloc_v2t.h"
#include "bloc_weather.h"
#ifdef BSP_USING_GESTURE_HANDLER
    #include "gesture_handler.h"
#endif
#include "bloc_peripheral.h"
#include "bloc_motion_tracking.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif
#include "gui_app_int.h"

#ifdef APP_ID_MAIN

    #define DBG_TAG "app.mainmenu"
    #define DBG_LVL DBG_LOG
    #include <rtdbg.h>

/**
 * iterate over builtin instruction list
 */
extern const builtin_app_desc_t *gui_builtin_instruction_list_open(void);
extern const builtin_app_desc_t *
gui_builtin_instruction_list_get_next(const builtin_app_desc_t *ptr_app);
extern void
gui_builtin_instruction_list_close(const builtin_app_desc_t *ptr_app);

enum
{
    APP_MAIN_CLOCK,
    APP_MAIN_INSTRUCTION_LIST,
    APP_MAIN_MAX,
};

typedef struct
{
    uint8_t num;
    lv_obj_t *scr;
    lv_obj_t **list;
    lv_indev_t *indev;
    lv_obj_t *message_indicator;
} app_mainmenu_ctx_t;

    #ifndef BSP_USING_LVGL_INPUT_AGENT
static
    #endif
    app_mainmenu_ctx_t app_mainmenu_ctx;

MyStruct_T myLancher[APP_NULL];

    #define INIT_APP_INDEX 1
    #define ROW_APP_MAX_COUNT 2
// tile view
static uint16_t row_app_index = app_index_clock;
static lv_obj_t *tileview;
static void set_stack_app_index(uint16_t index)
{
    if (index != row_app_index)
    {
        row_app_index = index;
        LOG_D("row_app_index: % d", row_app_index);
        if (index == app_index_instruction_list)
        {
            LOG_I("Open instruction list");
            // switch_watch_motion_control_mode(true, true);
            clock_on_pause();
        }
        else if (index == app_index_clock)
        {
            LOG_I("Open watchface");
            clock_on_resume();
        }
    }

    check_is_at_home();
}

void animate_to_home_from_instruction_list(void)
{
    lv_obj_set_tile_id(myLancher[app_index_message].pagetileview, 1, 1,
                       LV_ANIM_ON);
}

    #define TOP_PAGE_INDEX app_index_clock
    #define BOTTOM_PAGE_INDEX app_index_instruction_list

static lv_obj_t *middle_layer_bg;

void close_middle_layer_bg(void)
{
    if (middle_layer_bg)
    {
        LOG_D("close_middle_layer_bg");
        lv_obj_set_style_bg_opa(middle_layer_bg, LV_OPA_0, 0);
        set_instruction_list_time_opa(LV_OPA_0);
    }
    else
    {
        LOG_D("middle_layer_bg is NULL");
    }
}

static void handle_set_instruction_list_opa(uint8_t opa)
{
    if (!middle_layer_bg)
        return;
    lv_obj_set_style_bg_opa(middle_layer_bg, opa, 0);
}

static inline void set_middle_layer_bg_opa(int32_t opa)
{
    if (middle_layer_bg)
        lv_obj_set_style_bg_opa(middle_layer_bg, (lv_opa_t)opa, 0);
}

static void set_test_obj_opa(void *_, int32_t opa)
{
    set_middle_layer_bg_opa(opa);
}

static bool close_instruction_list_opa_set = false;
void set_close_instruction_list_opa_set(bool set)
{
    close_instruction_list_opa_set = set;
}

void handle_tap_event_in_mainmenu(void)
{
    LOG_D("handle_tap_event_in_mainmenu, row_app_index:%d", row_app_index);
    RT_ASSERT(row_app_index < APP_MAIN_MAX);
    RT_ASSERT(myLancher[row_app_index].on_tap != NULL);
    bool is_hidden;
    if (is_at_instruction_list())
    {
        LOG_D("app_index_instruction_list");
        myLancher[app_index_instruction_list].on_tap();
    }
}

static bool _at_ai_interface;
bool is_at_ai_interface(void)
{
    return _at_ai_interface;
}

extern bool get_instruction_list_ai_tapped(void);
extern void set_instruction_list_ai_tapped(void);
void check_is_at_ai_interface(void)
{
    /* PC sim: AI/speech apps excluded → pagetileview never set. Treat as not
     * at AI interface (default false) instead of NULL-deref'ing. */
    if (myLancher[app_index_ai_interface].pagetileview == NULL)
        return;
    bool yes = !lv_obj_has_flag(myLancher[app_index_ai_interface].pagetileview,
                                LV_OBJ_FLAG_HIDDEN);
    if (yes != _at_ai_interface)
    {
        _at_ai_interface = yes;
        if (_at_ai_interface)
        {
            switch_watch_motion_control_mode(true, false);
            setting_provider.set_power_save_mode(0);
            show_instruction_list_time(false);
            extern void reset_ai_coding(void);
            reset_ai_coding();
            if (!get_instruction_list_ai_tapped())
            {
                set_ai_open_mic(true);
                show_speech_indicator(true);
                voice_provider.start_v2t();
            }
            else
            {
                set_instruction_list_ai_tapped();
            }

            set_free_control_with_arm(false);
        }
        else
        {
            voice_provider.stop_v2t();
            show_instruction_list_time(true);
        }
        LOG_I("is_at_ai_interface: %d", _at_ai_interface);
    }
}

static bool _at_instruction_list = false;
bool is_at_instruction_list(void)
{
    // LOG_D("is_at_instruction_list? %d", _at_instruction_list);
    return _at_instruction_list;
}

static void on_tap_wrapper(uint8_t gesture)
{
    if (gesture == 1 && myLancher[app_index_instruction_list].on_tap)
        myLancher[app_index_instruction_list].on_tap();
}

static bool need_open_gesture_control = false;
bool get_need_open_gesture_control(void)
{
    return true;
}
void set_need_open_gesture_control(bool need)
{
    need_open_gesture_control = need;
}
extern void reset_skai_widget_input_text(void);
extern uint8_t get_middle_layer_tileview_index(void);
extern bool get_is_open_instruction_list_ai(void);
extern bool get_app_list_tileview_page(void);
extern void open_skai_widget_ai(bool open);
void check_is_at_instruction_list(void)
{
    bool yes = gui_app_is_actived(APP_ID_MAIN) &&
               !lv_obj_has_flag(myLancher[app_index_message].pagetileview,
                                LV_OBJ_FLAG_HIDDEN) &&
               !_at_ai_interface && get_app_list_tileview_page() &&
               get_middle_layer_tileview_index() == INSTRUCTION_LIST_PAGE_INDEX;
    if (yes != _at_instruction_list)
    {
        _at_instruction_list = yes;
        if (_at_instruction_list)
        {
            LOG_I("DBG inst_list: before set_power_save_mode");
            if (setting_provider.set_power_save_mode)
                setting_provider.set_power_save_mode(1);
            LOG_I("DBG inst_list: before sensor_quat");
            extern uint8_t return_total_list_count(void);
            extern uint16_t get_gesture_starting_value(void);
            set_prev_sensor_quat(get_gesture_starting_value());
            set_scroll_segment_count(return_total_list_count());
            LOG_I("DBG inst_list: after scroll_seg_count");
            // lvgl_msg_handler.handle_tap_indicator = on_tap_wrapper;
            extern void set_arc_stripe_external_offset(int16_t offset_degrees);
            lvgl_msg_handler.handle_set_arc_stripe_external_offset =
                set_arc_stripe_external_offset;
            extern void refersh_weather_icon(void);
            LOG_I("DBG inst_list: before refersh_weather_icon");
            refersh_weather_icon();
            LOG_I("DBG inst_list: before instruction_list_resume");
            instruction_list_resume();
            LOG_I("DBG inst_list: before display_status_bar_area");
            display_status_bar_area(3, false);
            LOG_I("DBG inst_list: before display_gesture_detect_objs");
            display_gesture_detect_objs(0, false);
            LOG_I("DBG inst_list: enter-block done");
        }
        else
        {
            myLancher[app_index_instruction_list].reset_list();
            instruction_list_pause();
            display_gesture_detect_objs(0, true);
            open_skai_widget_ai(false);
            display_status_bar_area(3, true);
        }
        LOG_I("is_at_instruction_list: %d", _at_instruction_list);
    }
}

extern void ai_widget_start(void);
static bool _at_speech_interface;

bool is_at_speech_interface(void)
{
    return _at_speech_interface;
}

extern rt_int32_t speech_on_resume(void);
extern rt_int32_t speech_on_pause(void);
void check_is_at_speech_interface(void)
{
    bool yes = gui_app_is_actived(APP_ID_MAIN) &&
               !lv_obj_has_flag(myLancher[app_index_message].pagetileview,
                                LV_OBJ_FLAG_HIDDEN) &&
               !_at_ai_interface && get_middle_layer_tileview_index() == 4;
    if (yes != _at_speech_interface)
    {
        _at_speech_interface = yes;
        if (_at_speech_interface)
        {
            ai_widget_start();
            // switch_watch_motion_control_mode(true, true);
            speech_on_resume();
            set_open_control_options(true);
            display_status_bar_area(0, false);
            display_status_bar_area(1, false);
            display_status_bar_area(2, false);
            display_status_bar_area(3, false);
        }
        else
        {
            open_skai_widget_ai(false);
            speech_on_pause();
            set_open_control_options(false);
        }
        LOG_I("is_at_speech_interface: %d", _at_speech_interface);
    }
}

static bool _at_mouse_mode = false;

bool is_at_mouse_mode(void)
{
    return _at_mouse_mode;
}

void check_is_at_mouse_mode(void)
{
    bool yes = gui_app_is_actived(APP_ID_MAIN) &&
               !lv_obj_has_flag(myLancher[app_index_message].pagetileview,
                                LV_OBJ_FLAG_HIDDEN) &&
               !_at_ai_interface && get_middle_layer_tileview_index() == 3;
    if (yes != _at_mouse_mode)
    {
        _at_mouse_mode = yes;
        if (_at_mouse_mode)
        {
            ble_app_advertising_start(true, false);
            // switch_watch_motion_control_mode(true, true);
            display_gesture_detect_objs(0, false);
            // display_status_bar_area(3, false);
            extern void watch_system_mouse_resume(void);
            watch_system_mouse_resume();
            lv_obj_clear_flag(myLancher[app_index_message].pagetileview,
                              LV_OBJ_FLAG_SCROLLABLE);
        }
        else
        {
            ble_app_advertising_start(false, false);
            // display_gesture_detect_objs(0, true);
            // display_status_bar_area(3, true);
            extern void watch_system_mouse_pause(void);
            watch_system_mouse_pause();
            lv_obj_add_flag(myLancher[app_index_message].pagetileview,
                            LV_OBJ_FLAG_SCROLLABLE);
        }
        LOG_I("is_at_mouse_mode: %d", _at_mouse_mode);
    }
}

static bool _at_message = false;

bool is_at_message(void)
{
    return _at_message;
}

void check_is_at_message(void)
{
    bool yes = gui_app_is_actived(APP_ID_MAIN) &&
               !lv_obj_has_flag(myLancher[app_index_message].pagetileview,
                                LV_OBJ_FLAG_HIDDEN) &&
               !_at_ai_interface &&
               get_middle_layer_tileview_index() == MESSAGE_PAGE_INDEX;
    if (yes != _at_message)
    {
        _at_message = yes;
        if (_at_message)
        {
            setting_provider.set_power_save_mode(1);
            // switch_watch_motion_control_mode(true, true);
            set_open_control_options(true);
            set_free_control_with_arm(true);
            request_weather_within_six_hours(false);

            set_media_control_threshold(2000);
            reset_control_pos();
            extern uint8_t get_message_page_count(void);
            set_scroll_segment_count(get_message_page_count());
            set_prev_sensor_quat(0);
            notification_on_resume();
            /* Route motion-tracking scroll offset to message list indicator
             * dots */
            extern void set_message_list_arc_stripe_external_offset(
                int16_t offset_degrees);
            lvgl_msg_handler.handle_set_arc_stripe_external_offset =
                set_message_list_arc_stripe_external_offset;
            extern void refersh_weather_icon(void);
            refersh_weather_icon();
            display_status_bar_area(0, false);
            display_status_bar_area(1, false);
            display_status_bar_area(2, false);
            display_status_bar_area(3, false);
        }
        else
        {
            set_free_control_with_arm(false);
            set_open_control_options(false);
            set_media_control_threshold(3000);
            notification_on_pause();
            /* Stop routing motion-tracking offsets to message list dots */
            extern void set_message_list_arc_stripe_external_offset(
                int16_t offset_degrees);
            if (lvgl_msg_handler.handle_set_arc_stripe_external_offset ==
                set_message_list_arc_stripe_external_offset)
            {
                lvgl_msg_handler.handle_set_arc_stripe_external_offset = NULL;
            }
        }
    }
    LOG_I("is_at_message: %d", _at_message);
}

bool _at_control_center;

bool is_at_control_center(void)
{
    return _at_control_center;
}

void check_is_at_control_center(void)
{
    bool is_hidden;
    if (myLancher[app_index_message].pagetileview != NULL)
    {
        is_hidden = lv_obj_has_flag(myLancher[app_index_message].pagetileview,
                                    LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        is_hidden = true;
    }
    bool yes = gui_app_is_actived(APP_ID_MAIN) && !is_hidden &&
               (get_middle_layer_tileview_index() == CONTROL_CENTER_PAGE_INDEX);
    if (yes != _at_control_center)
    {
        _at_control_center = yes;
        if (_at_control_center)
        {
            setting_provider.set_power_save_mode(1);
            // switch_watch_motion_control_mode(false, false);
            extern void control_center_on_resume(void);
            control_center_on_resume();
            // display_status_bar_area(3, false);
            display_status_bar_area(0, false);
            display_status_bar_area(1, false);
            display_status_bar_area(2, false);
            display_status_bar_area(3, false);
        }
        else
        {
            extern void control_center_on_pause(void);
            control_center_on_pause();
            // display_status_bar_area(3, true);
        }
        LOG_I("_at_control_center: %d", _at_control_center);
    }
}

static bool _at_home;

bool is_at_home(void)
{
    return _at_home;
}

void check_is_at_home(void)
{
    bool yes = gui_app_is_actived(APP_ID_MAIN) && !_at_instruction_list &&
               !_at_message && !_at_control_center && !_at_mouse_mode &&
               !_at_ai_interface &&
               !_at_speech_interface;
    if (yes != _at_home)
    {
        _at_home = yes;
        if (_at_home)
        {
            need_open_gesture_control = false;
            set_free_control_with_arm(false);
            setting_provider.set_power_save_mode(1);
            switch_watch_motion_control_mode(false, false);
            screen_rotate_back_to_original_direction();
            extern void set_ai_hint_bg_pos(uint8_t x);
            set_ai_hint_bg_pos(0);
            display_gesture_detect_objs(0, false);
            display_status_bar_area(2, true);
            display_status_bar_area(0, true);
            display_status_bar_area(1, true);
            display_status_bar_area(3, true);
            extern void back_to_instruction_list_btn(void);
            back_to_instruction_list_btn();
        }
        else
        {
            if (!_at_mouse_mode && !_at_instruction_list &&
                !gui_app_is_actived(APP_ID_MOUSE))
            {
                display_gesture_detect_objs(0, true);
            }
        }
        LOG_I("is_at_home: %d", _at_home);
    }
}

static void clear_check_flags(void)
{
    _at_instruction_list = false;
    _at_message = false;
    _at_control_center = false;
    _at_home = false;
    _at_mouse_mode = false;
    _at_ai_interface = false;
    _at_speech_interface = false;
}

extern void scrolling_object(bool open_scrolling_object_flag);
extern void stop_scrolling_object(void);
static bool open_scrolling_app_flag = true;
void set_open_scrolling_app_flag(bool flag)
{
    open_scrolling_app_flag = flag;
}
void open_control_instruction_list(lv_anim_t *a)
{
    if (open_scrolling_app_flag)
    {
        lvgl_msg_handler.handle_tap_indicator = on_tap_wrapper;
        LOG_D("handle_tap_indicator =%p", on_tap_wrapper);

    #if ENABLE_VIRTUAL_TOUCH
        scrolling_object(true);
    #endif
        open_scrolling_app_flag = false;
    }
    clock_on_pause();
}

// extern void instruction_list_scroll_to_app(bool up);
static uint16_t bg_opa = LV_OPA_COVER;
static uint8_t bg_opa2 = LV_OPA_80;
void *app_tap_indicate_function = NULL;
void *app_scroll_list_function = NULL;
static char pevr_app_id[GUI_APP_ID_MAX_LEN] = {0};
static void app_main_Clock_view_event_cb(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target(event);
    switch (event->code)
    {
    case LV_EVENT_SCROLL_BEGIN:
        clock_on_resume();
        break;
    case LV_EVENT_SCROLL:
    {
        lv_coord_t sx = lv_obj_get_scroll_x(obj);
        /* Scale background opacity */
        lv_coord_t opa_scaled =
            ((466 - sx) * (bg_opa + 1)) / 420; /* similar mapping as before */
        if (opa_scaled > bg_opa)
            opa_scaled = bg_opa;

        /* Time text opacity */
        lv_coord_t text_opa = ((466 - sx) * bg_opa) / 466;
        if (text_opa > bg_opa)
            text_opa = bg_opa;
        set_instruction_list_time_opa(text_opa);
        /* Time background opacity */
        lv_coord_t time_bg_opa = ((466 - sx) * bg_opa2) / 466;
        if (time_bg_opa > bg_opa2)
            time_bg_opa = bg_opa2;
        lv_obj_set_style_bg_opa(get_instruction_list_time_bg(), time_bg_opa, 0);
        break;
    }
    case LV_EVENT_SCROLL_END:
    {
        lv_coord_t scroll_x = lv_obj_get_scroll_x(obj);
        if (0 == scroll_x)
        {
            gui_runing_app_t *app = gui_app_get_actived();
            strncpy(pevr_app_id, app->id, sizeof(pevr_app_id) - 1);
            pevr_app_id[sizeof(pevr_app_id) - 1] = '\0';
            lv_obj_set_style_bg_opa(get_instruction_list_time_bg(), bg_opa2, 0);
            set_instruction_list_time_opa(bg_opa);
            LOG_D("app_index_instruction_list");
            set_stack_app_index(app_index_instruction_list);
            extern void reset_gravity_position(void);
            reset_gravity_position();
            clock_on_pause();
            force_release_finger();
            if (open_scrolling_app_flag)
            {
                app_tap_indicate_function =
                    lvgl_msg_handler.handle_tap_indicator;
                app_scroll_list_function =
                    lvgl_msg_handler.handle_gyro_scroll_list;
                lvgl_msg_handler.handle_tap_indicator = on_tap_wrapper;
                LOG_D("handle_tap_indicator =%p", on_tap_wrapper);
                // lvgl_msg_handler.handle_gyro_scroll_list =
                //     instruction_list_scroll_to_app;
    #if ENABLE_VIRTUAL_TOUCH
                scrolling_object(true);
    #endif
                open_scrolling_app_flag = false;
            }
        }
        else if (466 == scroll_x)
        {
            force_release_finger();
            lvgl_msg_t msg;
            msg.type = LVGL_MSG_TYPE_TIME_TEXT;
            lvgl_send_msg(msg);
            lv_obj_set_style_bg_opa(get_instruction_list_time_bg(), LV_OPA_0,
                                    0);
            set_instruction_list_time_opa(LV_OPA_0);
            LOG_D("app_index_clock");
            if (myLancher[app_index_instruction_list].reset_list != NULL)
            {
                myLancher[app_index_instruction_list].reset_list();
            }
            set_stack_app_index(app_index_clock);
            extern void set_q_vertical_movement_magnification(float mag);
            set_q_vertical_movement_magnification(1.0f);

    #if ENABLE_VIRTUAL_TOUCH
            stop_scrolling_object();
    #endif
            if (gui_app_is_actived(APP_ID_MAIN))
            {
                lvgl_msg_handler.handle_tap_indicator = NULL;
                lvgl_msg_handler.handle_gyro_scroll_list = NULL;
            }
            else
            {
                gui_runing_app_t *app = gui_app_get_actived();
                if (strcmp(pevr_app_id, app->id) == 0)
                {
                    lvgl_msg_handler.handle_gyro_scroll_list =
                        app_scroll_list_function;
                    lvgl_msg_handler.handle_tap_indicator =
                        app_tap_indicate_function;
                }
            }
        }
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESSED:
    case LV_EVENT_CLICKED:
    case LV_EVENT_PRESSING:
    case LV_EVENT_SHORT_CLICKED:
    case LV_EVENT_LONG_PRESSED:
    case LV_EVENT_FOCUSED:
    default:
        break;
    }
}

bool message_has_read = false;

void refreh_notification_bar_indicator(uint16_t count)
{
}

void app_launcher_ui_init(void *param)
{
    extern lv_obj_t *build_home_view(lv_obj_t * parent);
    extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t * parent);
    lv_obj_t *instruction_list = build_home_view(lv_scr_act());

    gui_app_trans_anim_t enter_anim_cfg, exit_anim_cfg;

    gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
    gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_NONE);

    gui_app_set_enter_trans_anim(&enter_anim_cfg);
    gui_app_set_exit_trans_anim(&exit_anim_cfg);

    gui_app_set_trans_anim_prio(1, -1);
}

static void on_start(void)
{
    LOG_D("[mainmenu_on_start]");
    gui_set_brightness(SkaiWatchSys.brightness, false);
    row_app_index = app_index_clock;

    app_launcher_ui_init(NULL);

    #ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_set_instruction_list_opa =
        handle_set_instruction_list_opa;
    #endif
}

void check_main_page(void)
{
    check_is_at_message();
    check_is_at_instruction_list();
    check_is_at_control_center();
    check_is_at_home();
}

static void on_resume(void)
{
    LOG_D("[mainmenu_on_resume] %d", get_idle_state());
    LOG_I("on_resume: before check_main_page");
    check_main_page();
    LOG_I("on_resume: after check_main_page");
    check_is_at_ai_interface();
    LOG_I("on_resume: after check_is_at_ai_interface");
    if (is_at_home() || is_at_control_center() || is_at_instruction_list() ||
        is_at_message() || is_at_ai_interface())
    {
        LOG_I("on_resume: before screen_rotate_back");
        screen_rotate_back_to_original_direction();
        LOG_I("on_resume: before clock_on_resume");
        clock_on_resume();
        LOG_I("on_resume: after clock_on_resume");
    }

    LOG_I("on_resume: before lvgl_send_msg");
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_TRIGGER_ACTIVITY;
    lvgl_send_msg(msg);
    LOG_I("on_resume: returning");
}

static void on_pause(void)
{
    LOG_D("[mainmenu_on_pause]");
    instruction_list_pause();
    clock_on_pause();
    extern void reset_gravity_position(void);
    reset_gravity_position();
}

static void on_stop(void)
{
    LOG_D("[mainmenu_on_stop]");
    #ifdef ENABLE_NOTIFICATION_CENTER
    notification_on_deinit();
    #endif
    instruction_list_deinit();
    clock_on_stop();
    unsubscribe_pwr_service();

    #ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_set_instruction_list_opa = NULL;
    lvgl_msg_handler.handle_control_quick_btn = NULL;
    clear_all_handlers();
    #endif
    screen_rotate_back_to_original_direction();
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
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

static int app_mainmenu(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_MAIN, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(mainmenu), NULL, APP_ID_MAIN, app_mainmenu);
#endif /* APP_ID_MAIN */
       /************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
        * FILE****/