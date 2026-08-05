/**
 ******************************************************************************
 * @file   app_camera.c
 * @author Skaiwalk software development team
 * @brief  Remote camera shutter app — a 300x300 circular button that triggers
 *         the phone camera shutter by sending a volume-up HID consumer report.
 ******************************************************************************
 */
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "custom_trans_anim.h"
#include "bloc_setting.h"
#include "ble_hid.h"
#include "ui_handler.h"
#include "ui_img_helper.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
#endif

#define DBG_TAG "app.camera"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_CAMERA

typedef struct
{
    lv_obj_t *main_window;
} app_camera_t;

static app_camera_t *p_app_camera = NULL;

#ifdef HID_CONSUMER
extern uint8_t get_main_phonepeer_conn_idx(void);

static void trigger_camera_shutter(void)
{
    uint8_t phone_idx = get_main_phonepeer_conn_idx();
    if (phone_idx == 0xFF)
    {
        LOG_W("camera shutter aborted: no main phone peer");
        return;
    }

    uint8_t saved_idx = ble_hid_get_conn_idx();
    if (saved_idx != phone_idx)
    {
        LOG_I("camera shutter: switch HID conn_idx %d -> %d (phone)",
              saved_idx, phone_idx);
        ble_hid_set_conn_idx(phone_idx);
    }

    volume_up_through_hid();

    if (saved_idx != phone_idx)
    {
        ble_hid_set_conn_idx(saved_idx);
    }
}
#endif

static void shutter_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        LOG_I("camera shutter -> volume up HID");
#ifdef HID_CONSUMER
        trigger_camera_shutter();
#endif
    }
}

static void tap_indicator_cb(uint8_t press)
{
    if (!press)
    {
        return;
    }
    LOG_I("camera tap -> volume up HID");
#ifdef HID_CONSUMER
    trigger_camera_shutter();
#endif
}

static lv_obj_t *create_camera_screen(lv_obj_t *scr)
{
    lv_obj_t *bg = lv_obj_create(scr);
    lv_obj_set_size(bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(bg, lv_color_black(), 0);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *shutter_btn = lv_btn_create(bg);
    lv_obj_set_size(shutter_btn, 300, 300);
    lv_obj_center(shutter_btn);
    lv_obj_set_style_radius(shutter_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(shutter_btn, lv_color_hex(0xE53935), 0);
    lv_obj_set_style_bg_color(shutter_btn, lv_color_hex(0xB71C1C),
                              LV_STATE_PRESSED);
    lv_obj_set_style_border_width(shutter_btn, 6, 0);
    lv_obj_set_style_border_color(shutter_btn, lv_color_white(), 0);
    lv_obj_set_style_shadow_width(shutter_btn, 0, 0);
    lv_obj_add_event_cb(shutter_btn, shutter_btn_event_cb, LV_EVENT_CLICKED,
                        NULL);

    lv_obj_t *label = lv_label_create(shutter_btn);
    lv_label_set_text(label, "Shutter");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);

    return bg;
}

static lv_obj_t *on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_camera);
    p_app_camera = (app_camera_t *)lv_mem_alloc(sizeof(app_camera_t));
    if (!p_app_camera)
    {
        LOG_E("alloc camera app failed");
        return NULL;
    }
    memset(p_app_camera, 0, sizeof(app_camera_t));

    p_app_camera->main_window = create_camera_screen(scr);
    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    return p_app_camera->main_window;
}

static void on_resume(void)
{
    switch_watch_motion_control_mode(true, false);
    setting_provider.set_power_save_mode(0);
    lvgl_msg_handler.handle_tap_indicator = tap_indicator_cb;
}

static void on_pause(void)
{
    switch_watch_motion_control_mode(false, false);
    setting_provider.set_power_save_mode(1);
    if (lvgl_msg_handler.handle_tap_indicator == tap_indicator_cb)
    {
        lvgl_msg_handler.handle_tap_indicator = NULL;
    }
}

static void on_stop(void)
{
    if (p_app_camera)
    {
        if (p_app_camera->main_window)
        {
            lv_obj_del(p_app_camera->main_window);
            p_app_camera->main_window = NULL;
        }
        lv_mem_free(p_app_camera);
        p_app_camera = NULL;
    }
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
        on_start(lv_scr_act());
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
    gui_app_regist_msg_handler(APP_ID_CAMERA, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(camera), IMG_LOGO, APP_ID_CAMERA, app_main, 1);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
