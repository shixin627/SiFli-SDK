/**
 ******************************************************************************
 * @file   app_flashlight.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "gesture_handler.h"
#include "bloc_control.h"
#include "app_mainmenu.h"
#include "custom_trans_anim.h"
#include "common_widget.h"
#include "bloc_setting.h"
#include "bloc_motion_tracking.h"
#include "ui_datasrv_subscriber.h"
#include "data_service.h"
#include "power_manager_service.h"
#include "data_service_subscriber.h"
#include "watch_system_interact.h"
#include "bloc_peripheral.h"

#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
    #include "ui_img_helper.h"
#endif
#define DBG_TAG "app.flashlight"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_FLASHLIGHT
/*********************
 *      DEFINES
 *********************/

typedef struct
{
    lv_obj_t *main_window;
    datac_handle_t pwr_srv_hdl;
} app_flashlight_t;

/* Forward declarations */
static void toggle_flashlight_btn_event_cb(lv_event_t *e);
static int powermgr_srv_callback(data_callback_arg_t *arg);
static void set_amoled_brightness(uint8_t brightness);

/* Global variables */
static app_flashlight_t *p_app_flashlight = NULL;
/**
 * @brief Creates the flashlight screen
 *
 * @param scr Parent screen object
 * @return lv_obj_t* Created screen object
 */
lv_obj_t *create_flashlight_screen(lv_obj_t *scr)
{
    lv_obj_t *bg = lv_obj_create(scr);
    lv_obj_set_size(bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(bg, lv_color_white(), 0);

    lv_obj_t *flashlight_bg = lv_obj_create(bg);
    lv_obj_set_size(flashlight_bg, 200, 200);
    lv_obj_center(flashlight_bg);
    lv_obj_set_style_radius(flashlight_bg, 100, 0);
    lv_obj_set_style_bg_color(flashlight_bg, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(flashlight_bg, LV_OPA_0, 0);

    /* Create the toggle flashlight image button */
    lv_obj_t *button_flashlight = lv_obj_create(bg);
    lv_obj_set_size(button_flashlight, 200, 200);
    lv_obj_add_event_cb(button_flashlight, toggle_flashlight_btn_event_cb,
                        LV_EVENT_ALL, 0);
    lv_obj_center(button_flashlight);

    /* Set the button's background to transparent */
    lv_obj_set_style_bg_color(button_flashlight, LV_COLOR_BLACK, 0);
    lv_obj_set_style_bg_opa(button_flashlight, LV_OPA_TRANSP, 0);

    /* Set the image for the button */
    lv_obj_t *image_flashlight = lv_img_create(button_flashlight);
    lv_img_set_src(image_flashlight, LV_EXT_IMG_GET(BTN_FLASHLIGHT));
    lv_obj_center(image_flashlight);

    return bg;
}

/**
 * @brief Handles gesture event for the flashlight button
 */
static void handle_gesture_event(uint8_t type)
{
    if (type == 1)
    {
        gui_app_exit(APP_ID_FLASHLIGHT);
    }
}

/**
 * @brief Button event callback
 */
static void toggle_flashlight_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        handle_gesture_event(1);
    }
}

/**
 * @brief Power manager service callback
 */
static int powermgr_srv_callback(data_callback_arg_t *arg)
{
    if (!p_app_flashlight && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case PWRMGR_MSG_LCD_BRIGHTNESS_GET_RSP:
    {
        range_msg_t *p_range = (range_msg_t *)arg->data;
        LOG_D("PWRMGR_MSG_LCD_BRIGHTNESS_GET_RSP cur=%d[%d,%d]", p_range->cur,
              p_range->min, p_range->max);
        break;
    }
    case PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP:
    {
        range_msg_t *p_range = (range_msg_t *)arg->data;
        LOG_D("PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP cur=%d[%d,%d]", p_range->cur,
              p_range->min, p_range->max);
        break;
    }
    default:
        break;
    }
    return 0;
}

/**
 * @brief Send data to the data service
 */
static void flashlight_datac_send_data(datac_handle_t handle, uint16_t msg_id,
                                       uint8_t *data, uint16_t data_len)
{
    data_msg_t msg;
    uint8_t *msg_payload;
    LOG_D("brightness:%d", *data);
    msg_payload = data_service_init_msg(&msg, msg_id, data_len);
    memcpy(msg_payload, data, data_len);
    datac_send_msg(handle, &msg);
}

/**
 * @brief Set the AMOLED screen brightness
 *
 * @param brightness Brightness value
 */
static void set_amoled_brightness(uint8_t brightness)
{
    LOG_D("set_amoled_brightness :%d", brightness);

    if (p_app_flashlight &&
        p_app_flashlight->pwr_srv_hdl != DATA_CLIENT_INVALID_HANDLE)
    {
        flashlight_datac_send_data(p_app_flashlight->pwr_srv_hdl,
                                   PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ,
                                   (uint8_t *)&brightness, sizeof(uint16_t));
    }
}

/**
 * @brief Initialize the app on start
 */
static lv_obj_t *on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_flashlight);
    p_app_flashlight =
        (app_flashlight_t *)lv_mem_alloc(sizeof(app_flashlight_t));
    if (!p_app_flashlight)
    {
        LOG_E("Failed to allocate memory for flashlight app");
        return NULL;
    }

    memset(p_app_flashlight, 0, sizeof(app_flashlight_t));
    p_app_flashlight->main_window = create_flashlight_screen(scr);

    p_app_flashlight->pwr_srv_hdl = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_flashlight->pwr_srv_hdl);
    ui_datac_subscribe(p_app_flashlight->pwr_srv_hdl, "powermgr",
                       powermgr_srv_callback, 0);

    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    return p_app_flashlight->main_window;
}

/**
 * @brief Resume the app
 */
static void on_resume(void)
{
    set_open_control_options(false);
    set_free_control_with_arm(false);
    reset_lvgl_msg_handler();
    setting_provider.set_power_save_mode(0);
}

/**
 * @brief Pause the app
 */
static void on_pause(void)
{
    setting_provider.set_power_save_mode(1);
}

/**
 * @brief Stop and clean up the app
 */
static void on_stop(void)
{
    if (p_app_flashlight)
    {
        if (DATA_CLIENT_INVALID_HANDLE != p_app_flashlight->pwr_srv_hdl)
        {
            datac_close(p_app_flashlight->pwr_srv_hdl);
            p_app_flashlight->pwr_srv_hdl = DATA_CLIENT_INVALID_HANDLE;
        }

        if (p_app_flashlight->main_window)
        {
            lv_obj_del(p_app_flashlight->main_window);
            p_app_flashlight->main_window = NULL;
        }

        lv_mem_free(p_app_flashlight);
        p_app_flashlight = NULL;
    }
}

/**
 * @brief Message handler for app lifecycle events
 */
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
        set_amoled_brightness(96); // Maximum brightness for flashlight
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        set_amoled_brightness(
            SkaiWatchSys.brightness); // Restore system brightness
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;

    default:
        break;
    }
}

/**
 * @brief Main entry point for the app
 */
static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_FLASHLIGHT, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(flashlight), IMG_FLASHLIGHT, APP_ID_FLASHLIGHT,
                   app_main, 1);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/