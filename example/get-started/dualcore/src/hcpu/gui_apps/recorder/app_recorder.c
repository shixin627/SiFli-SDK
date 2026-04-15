/**
 ******************************************************************************
 * @file   app_recorder.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <time.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"

#include "app_mainmenu.h"
#include "custom_trans_anim.h"
#include "bloc_motion_tracking.h"
#include "common_widget.h"

#ifdef BSP_USING_BLOC
#include "bloc_v2t.h"
#include "bloc_setting.h"
#include "bloc_control.h"
#include "bloc_peripheral.h"
#include "bloc_filesystem.h"
#endif
#include "ui_helper.h"
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#define DBG_TAG "app.recorder"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_RECORDER

/* Constants and definitions */
#define BASE_SIZE 60
#define CIRCLE_SIZE (2 * BASE_SIZE)

/* Type definitions */
typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *record_button;
    lv_obj_t *record_time_label;
    lv_timer_t *record_timer;
    uint32_t record_time;
    time_t record_start_time;
} app_recorder_t;

/* Static variables */
static app_recorder_t *p_app_recorder = NULL;

/* Function declarations */
static void toggle_record_btn_event_cb(lv_event_t *e);
static void set_record_button_style(lv_obj_t *button, bool is_recording);
static void handle_tap_event(uint8_t press);
static void handle_vad_status(bool user_talking);

/* UI style management functions */
static void set_record_button_style(lv_obj_t *button, bool is_recording)
{
    lv_obj_clean(button);

    if (is_recording)
    {
        lv_obj_t *inner_rectangle = lv_obj_create(button);
        lv_obj_set_size(inner_rectangle, BASE_SIZE, BASE_SIZE);
        lv_obj_set_style_radius(inner_rectangle, 20, 0);
        lv_obj_set_style_bg_color(inner_rectangle, lv_color_hex(0xFF0000), 0);
        lv_obj_center(inner_rectangle);
        lv_obj_clear_flag(inner_rectangle, LV_OBJ_FLAG_CLICKABLE);
    }
    else
    {
        lv_obj_t *inner_circle = lv_obj_create(button);
        lv_obj_set_size(inner_circle, CIRCLE_SIZE, CIRCLE_SIZE);
        lv_obj_set_style_radius(inner_circle, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(inner_circle, lv_color_hex(0xFF0000), 0);
        lv_obj_center(inner_circle);
        lv_obj_clear_flag(inner_circle, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_set_style_border_color(button, lv_color_hex(0x808080), 0);
        lv_obj_set_style_border_width(button, 5, 0);
    }
}

/* UI creation functions */
lv_obj_t *create_record_view(lv_obj_t *parent)
{
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *outer_circle = lv_obj_create(container);
    lv_obj_set_size(outer_circle, CIRCLE_SIZE + 10, CIRCLE_SIZE + 10);
    lv_obj_set_style_radius(outer_circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(outer_circle, lv_color_hex(0x808080), 0);
    lv_obj_set_style_bg_opa(outer_circle, LV_OPA_50, 0);
    lv_obj_set_style_border_color(outer_circle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(outer_circle, 5, 0);
    lv_obj_set_style_border_opa(outer_circle, LV_OPA_100, 0);
    lv_obj_set_style_bg_opa(outer_circle, LV_OPA_TRANSP, 0);
    lv_obj_center(outer_circle);
    lv_obj_add_event_cb(outer_circle, toggle_record_btn_event_cb, LV_EVENT_ALL, 0);

    set_record_button_style(outer_circle, false);
    p_app_recorder->record_button = outer_circle;

    lv_obj_t *record_time_label = lv_label_create(container);
    lv_obj_set_style_text_font(record_time_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(record_time_label, lv_color_white(), 0);
    lv_label_set_text(record_time_label, "tap to record");
    lv_obj_align(record_time_label, LV_ALIGN_BOTTOM_MID, 0, -70);
    p_app_recorder->record_time_label = record_time_label;

    return container;
}

static uint32_t last_seconds;
static uint32_t current_milliseconds;
/* Timer callback functions */
static void update_record_time(lv_timer_t *timer)
{
    p_app_recorder->record_time++;
    time_t now;
    time(&now);
    uint32_t elapsed_sec = (uint32_t)(now - p_app_recorder->record_start_time);
    uint32_t minutes = elapsed_sec / 60;
    uint32_t seconds = elapsed_sec % 60;
    if (last_seconds != seconds)
    {
        last_seconds = seconds;
        current_milliseconds = 0;
    }
    else
    {
        current_milliseconds += 50;
    }
    char time_str[12];
    snprintf(time_str, sizeof(time_str), "%02d:%02d.%03d", minutes, seconds, current_milliseconds);
    lv_label_set_text(p_app_recorder->record_time_label, time_str);
}

/* Recording management functions */
static void start_to_record_voice(void)
{
    start_voice_recording();
    set_record_button_style(p_app_recorder->record_button, true);
    p_app_recorder->record_time = 0;
    time(&p_app_recorder->record_start_time);
    lv_label_set_text(p_app_recorder->record_time_label, "00:00");
    p_app_recorder->record_timer = lv_timer_create(update_record_time, 50, NULL);
}

static void stop_recording_voice(void)
{
    if (p_app_recorder->record_timer)
    {
        lv_timer_del(p_app_recorder->record_timer);
        p_app_recorder->record_timer = NULL;

        stop_voice_recording();

        set_record_button_style(p_app_recorder->record_button, false);
        lv_label_set_text(p_app_recorder->record_time_label, "tap to record");

        const char *file_path = get_last_recording_file();
        if (file_path && strlen(file_path) > 0)
        {
            int ret = bloc_file_system.sync_file((char *)file_path, true);
            if (ret == 0)
            {
                LOG_D("sync last recording: %s", file_path);
            }
            else
            {
                LOG_E("failed to sync last recording: %s", file_path);
            }
        }
    }
}

/* UI event handlers */
static void handle_tap_event(uint8_t press)
{
    if (press == 1)
    {
        if (!app_voice_get_recording_status())
        {
            start_to_record_voice();
        }
        else
        {
            stop_recording_voice();
        }
    }
}

static void handle_vad_status(bool user_talking)
{
    if (user_talking)
    {
        lv_obj_set_style_border_color(p_app_recorder->record_button, lv_color_hex(0x97CBFF), 0);
        lv_obj_set_style_border_width(p_app_recorder->record_button, 10, 0);
    }
    else
    {
        lv_obj_set_style_border_color(p_app_recorder->record_button, lv_color_hex(0x808080), 0);
        lv_obj_set_style_border_width(p_app_recorder->record_button, 5, 0);
    }
}

static void toggle_record_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        handle_tap_event(1);
    }
}

/* App lifecycle functions */
lv_obj_t *recorder_on_start(lv_obj_t *scr)
{
    LOG_D("recorder_app_on_start");
    RT_ASSERT(NULL == p_app_recorder);
    p_app_recorder = (app_recorder_t *)lv_mem_alloc(sizeof(app_recorder_t));
    memset(p_app_recorder, 0, sizeof(app_recorder_t));

    p_app_recorder->bg = common_black_bg(scr);
    create_record_view(p_app_recorder->bg);

    setting_provider.set_power_save_mode(0);
    start_to_record_voice();
    return p_app_recorder->bg;
}

void recorder_on_resume(void)
{
    reset_lvgl_msg_handler();
    set_open_control_options(true);
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_vad_status = handle_vad_status;
    lvgl_msg_handler.handle_tap_indicator = handle_tap_event;
#endif
}

void recorder_on_pause(void)
{
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_vad_status = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
    lvgl_msg_handler.handle_widgets_control = NULL;
#endif
    set_open_control_options(false);
}

void recorder_on_stop(void)
{
    stop_recording_voice();
    setting_provider.set_power_save_mode(1);
    if (p_app_recorder)
    {
        if (p_app_recorder->record_timer)
        {
            lv_timer_del(p_app_recorder->record_timer);
            p_app_recorder->record_timer = NULL;
        }

        lv_obj_del(p_app_recorder->bg);
        p_app_recorder->bg = NULL;

        lv_mem_free(p_app_recorder);
        p_app_recorder = NULL;
    }
}

/* Message handler */
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        recorder_on_start(lv_scr_act());
        break;

    case GUI_APP_MSG_ONRESUME:
        recorder_on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        recorder_on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        recorder_on_stop();
        break;

    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_RECORDER, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(recorder), IMG_RECORDER, APP_ID_RECORDER, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
