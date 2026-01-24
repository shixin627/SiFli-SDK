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
#include "watch_system_core_task.h"
#include "rgb_control_panel.h"
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
    rgb_led_state_t *rgb_state;
} app_flashlight_t;

/* Forward declarations */
static void toggle_flashlight_btn_event_cb(lv_event_t *e);
static void tap_change_flashlight_state(void);
static void flashlight_switch_cb(lv_event_t *e);
static int powermgr_srv_callback(data_callback_arg_t *arg);
static void set_amoled_brightness(uint8_t brightness);
static void rgb_btn_event_cb(lv_event_t *e);

/* Global variables */
static app_flashlight_t *p_app_flashlight = NULL;
static lv_obj_t *flashlight_switch;
static bool widget_flashlight_on = false;

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
    lv_obj_add_event_cb(button_flashlight, toggle_flashlight_btn_event_cb, LV_EVENT_ALL, 0);
    lv_obj_center(button_flashlight);

    /* Set the button's background to transparent */
    lv_obj_set_style_bg_color(button_flashlight, LV_COLOR_BLACK, 0);
    lv_obj_set_style_bg_opa(button_flashlight, LV_OPA_TRANSP, 0);

    /* Set the image for the button */
    lv_obj_t *image_flashlight = lv_img_create(button_flashlight);
    lv_img_set_src(image_flashlight, LV_EXT_IMG_GET(BTN_FLASHLIGHT));
    lv_obj_center(image_flashlight);

    /* Create RGB LED control button */
    lv_obj_t *rgb_btn = lv_btn_create(bg);
    lv_obj_set_size(rgb_btn, 120, 50);
    lv_obj_align(rgb_btn, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_radius(rgb_btn, 25, 0);
    lv_obj_set_style_bg_color(rgb_btn, lv_color_hex(0x333333), 0);
    lv_obj_set_style_bg_opa(rgb_btn, LV_OPA_70, 0);
    lv_obj_add_event_cb(rgb_btn, rgb_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *rgb_label = lv_label_create(rgb_btn);
    lv_label_set_text(rgb_label, "RGB LED");
    lv_obj_set_style_text_color(rgb_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(rgb_label);

    return bg;
}

/**
 * @brief Handles gesture event for the flashlight button
 */
static void handle_gesture_event(uint8_t type)
{
    if (type == 1)
    {
        // send_virtual_gesture_event(GESTURE_EVENT_FORCE_RELEASE);

        if (lv_obj_is_valid(flashlight_switch))
        {
            lv_obj_clear_state(flashlight_switch, LV_STATE_CHECKED);
        }

        gui_app_exit(APP_ID_FLASHLIGHT);
    }
}

/**
 * @brief Changes the flashlight switch state
 *
 * @param state The state to set (true = on, false = off)
 */
void change_flashlight_switch(bool state)
{
    if (lv_obj_is_valid(flashlight_switch))
    {
        if (state)
        {
            lv_obj_add_state(flashlight_switch, LV_STATE_CHECKED);
        }
        else
        {
            lv_obj_clear_state(flashlight_switch, LV_STATE_CHECKED);
        }
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
 * @brief Creates a flashlight background widget
 *
 * @param parent Parent object
 * @return lv_obj_t* Created widget
 */
static lv_obj_t *flashlight_on_widget(lv_obj_t *parent)
{
    lv_obj_t *flashlight_on = lv_obj_create(parent);
    lv_obj_set_size(flashlight_on, 466, 466);
    lv_obj_set_style_bg_color(flashlight_on, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(flashlight_on, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(flashlight_on, LV_OBJ_FLAG_HIDDEN);
    return flashlight_on;
}

/**
 * @brief Turns off the flashlight
 */
void turn_off_flashlight(void)
{
    if (lv_obj_is_valid(flashlight_switch))
    {
        lv_obj_clear_state(flashlight_switch, LV_STATE_CHECKED);
        widget_flashlight_on = false;
    }
}

/**
 * @brief Changes flashlight state and opens app on tap
 */
static void tap_change_flashlight_state(void)
{
    LOG_D("tap_change_flashlight_state");
    // if (lv_obj_is_valid(flashlight_switch))
    // {
    //     lv_obj_add_state(flashlight_switch, LV_STATE_CHECKED);
    // }
    gui_app_run(APP_ID_FLASHLIGHT);
}

/**
 * @brief Flashlight switch event callback
 */
static void flashlight_switch_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_VALUE_CHANGED == event)
    {
        if (lv_obj_has_state(flashlight_switch, LV_STATE_CHECKED))
        {
            LOG_D("flashlight_switch_cb");
            change_flashlight_switch(true);
            lvgl_msg_t msg;
            msg.type = LVGL_MSG_TYPE_SWITCH_FLASHLIGHT;
            lvgl_send_msg(msg);
            widget_flashlight_on = true;
        }
        else
        {
            widget_flashlight_on = false;
        }
    }
}

/**
 * @brief Creates the flashlight widget
 *
 * @param parent Parent object
 * @return lv_obj_t* Created widget
 */
lv_obj_t *lv_flashlight_widget_builder(lv_obj_t *parent)
{
    lv_obj_t *widget = common_widget_container(parent);
    lv_obj_set_style_bg_opa(widget, LV_OPA_0, 0);
    flashlight_switch = lv_switch_create(widget);
    lv_obj_align(flashlight_switch, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(flashlight_switch, flashlight_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lvgl_msg_handler.handle_switch_flashlight = tap_change_flashlight_state;

    return widget;
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
        LOG_D("PWRMGR_MSG_LCD_BRIGHTNESS_GET_RSP cur=%d[%d,%d]",
              p_range->cur, p_range->min, p_range->max);
        break;
    }
    case PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP:
    {
        range_msg_t *p_range = (range_msg_t *)arg->data;
        LOG_D("PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP cur=%d[%d,%d]",
              p_range->cur, p_range->min, p_range->max);
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
static void flashlight_datac_send_data(datac_handle_t handle, uint16_t msg_id, uint8_t *data, uint16_t data_len)
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

    if (p_app_flashlight && p_app_flashlight->pwr_srv_hdl != DATA_CLIENT_INVALID_HANDLE)
    {
        flashlight_datac_send_data(p_app_flashlight->pwr_srv_hdl,
                                   PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ,
                                   (uint8_t *)&brightness, sizeof(uint16_t));
    }
}

/**
 * @brief RGB button event callback
 */
static void rgb_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED && p_app_flashlight != NULL)
    {
        LOG_I("RGB button clicked");
        if (!is_rgb_panel_open() && p_app_flashlight->rgb_state != NULL)
        {
            create_rgb_control_panel(lv_scr_act(), p_app_flashlight->rgb_state);
        }
    }
}

/**
 * @brief Initialize the app on start
 */
static lv_obj_t *on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_flashlight);
    p_app_flashlight = (app_flashlight_t *)lv_mem_alloc(sizeof(app_flashlight_t));
    if (!p_app_flashlight)
    {
        LOG_E("Failed to allocate memory for flashlight app");
        return NULL;
    }

    memset(p_app_flashlight, 0, sizeof(app_flashlight_t));
    p_app_flashlight->main_window = create_flashlight_screen(scr);

    p_app_flashlight->pwr_srv_hdl = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_app_flashlight->pwr_srv_hdl);
    ui_datac_subscribe(p_app_flashlight->pwr_srv_hdl, "powermgr", powermgr_srv_callback, 0);

    /* Initialize RGB LED state */
    p_app_flashlight->rgb_state = (rgb_led_state_t *)lv_mem_alloc(sizeof(rgb_led_state_t));
    if (p_app_flashlight->rgb_state != NULL)
    {
        rgb_led_state_init(p_app_flashlight->rgb_state);
        LOG_I("RGB LED state initialized");
    }
    else
    {
        LOG_E("Failed to allocate memory for RGB state");
    }

    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    return p_app_flashlight->main_window;
}

/**
 * @brief Resume the app
 */
static void on_resume(void)
{
    // switch_watch_motion_control_mode(true, false);
    set_open_control_options(false);
    set_free_control_with_arm(false);
    reset_lvgl_msg_handler();
    setting_provider.set_power_save_mode(0);

#ifdef BSP_USING_UI_HANDLER
    // lvgl_msg_handler.handle_tap_event = handle_tap_event;
    LOG_D("on_resume: handle_tap_event set");
    // lvgl_msg_handler.handle_tap_indicator = handle_gesture_event;
    LOG_D("on_resume: handle_tap_indicator set");
#endif
}

/**
 * @brief Pause the app
 */
static void on_pause(void)
{
    setting_provider.set_power_save_mode(1);

    /* Close RGB control panel if open */
    if (is_rgb_panel_open())
    {
        close_rgb_control_panel();
    }

#ifdef BSP_USING_UI_HANDLER
    // lvgl_msg_handler.handle_tap_event = NULL;
    // lvgl_msg_handler.handle_tap_indicator = NULL;
#endif
}

/**
 * @brief Stop and clean up the app
 */
static void on_stop(void)
{
    if (p_app_flashlight)
    {
        /* Close RGB control panel if open */
        if (is_rgb_panel_open())
        {
            close_rgb_control_panel();
        }

        /* Cleanup RGB LED state */
        if (p_app_flashlight->rgb_state != NULL)
        {
            rgb_led_state_cleanup(p_app_flashlight->rgb_state);
            lv_mem_free(p_app_flashlight->rgb_state);
            p_app_flashlight->rgb_state = NULL;
        }

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

    change_flashlight_switch(false);
    LOG_I("Flashlight app stopped and resources cleaned up");
}

/**
 * @brief Message handler for app lifecycle events
 */
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start(lv_scr_act());
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        set_amoled_brightness(96); // Maximum brightness for flashlight
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        set_amoled_brightness(SkaiWatchSys.brightness); // Restore system brightness
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

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(flashlight), IMG_FLASHLIGHT, APP_ID_FLASHLIGHT, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/