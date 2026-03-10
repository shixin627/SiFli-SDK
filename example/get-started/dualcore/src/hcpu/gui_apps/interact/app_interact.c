/**
 ******************************************************************************
 * @file   app_interact.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
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
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"

#ifdef BSP_USING_BLOC
#include "bloc_motor.h"
#include "bloc_setting.h"
#include "bloc_peripheral.h"
#include "bloc_v2t.h"
#include "bloc_system_perception.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#include "ui_handler.h"
#include "ui_helper.h"
#include "ui_img_helper.h"
#ifdef BSP_USING_COMMUNICATE
#include "communicate_protocol.h"
#endif
#ifdef BSP_USING_WATCH_SYS_CLIENT
#include "watch_sys_service.h"
#endif

#define DBG_TAG "app.interact"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_INTERACT

/*********************
 *      DEFINES
 *********************/

/*********************
 *      ASSETS
 *********************/
LV_IMG_DECLARE(mobile);

/*********************
 *      TYPEDEFS
 *********************/
/**
 * @brief Interaction states for different app modes
 */
typedef enum
{
    TEST_STATE_UNKNOWN,
    INTERACT_STATE_FIND_ME,        /* Find my device mode */
    INTERACT_STATE_LONG_SIT_ALERT, /* Sedentary reminder */
    LOW_POWER_WARNING,             /* Low battery warning */
    OTA_UPDATING,                  /* Firmware update mode */
    INTERACT_STATE_SHOW_QRCODE,    /* QR code display mode */
} interact_state_t;

/**
 * @brief UI components for different interaction states
 */
typedef struct
{
    lv_obj_t *hint_label;    /* Common hint label for all states */
    lv_obj_t *qr_code;       /* QR code object for show_qrcode state */
    lv_obj_t *ignore_button; /* Ignore button for sit alert state */
    lv_obj_t *status_image;  /* Status image for warning/pairing states */
    lv_obj_t *progress_bar;  /* Progress bar for OTA updating state */
    lv_timer_t *blink_timer; /* Blink timer for image states */
    lv_obj_t *close_btn;     /* Close button for find me state */
} interact_ui_t;

/*********************
 *  STATIC VARIABLES
 *********************/
static interact_state_t interact_state = TEST_STATE_UNKNOWN;
static interact_ui_t ui_components = {0}; /* Initialize all pointers to NULL */

/* Buffer to store QR code data */
char qrcode_data[256];

/*********************
 *  STATIC FUNCTIONS
 *********************/

/**
 * @brief Initialize UI components structure
 */
static void init_ui_components(void)
{
    memset(&ui_components, 0, sizeof(interact_ui_t));
}

/**
 * @brief Clean up UI components and free resources
 */
static void cleanup_ui_components(void)
{
}

/**
 * @brief Get hint label component
 * @return Pointer to hint label or NULL if not created
 */
static lv_obj_t *get_hint_label(void)
{
    return ui_components.hint_label;
}

/**
 * @brief Set hint label component
 * @param label Pointer to hint label object
 */
static void set_hint_label(lv_obj_t *label)
{
    ui_components.hint_label = label;
}

/**
 * @brief Get progress bar component
 * @return Pointer to progress bar or NULL if not created
 */
static lv_obj_t *get_progress_bar(void)
{
    return ui_components.progress_bar;
}

/**
 * @brief Set progress bar component
 * @param bar Pointer to progress bar object
 */
static void set_progress_bar(lv_obj_t *bar)
{
    ui_components.progress_bar = bar;
}

/**
 * @brief Set the hint text based on current interaction state
 * @param state Current interaction state
 */
static void set_text_hint(interact_state_t state)
{
    const char *text = NULL;
    lv_obj_t *hint_label = get_hint_label();

    switch (state)
    {
    case INTERACT_STATE_FIND_ME:
        text = "Finding me...";
        break;

    case INTERACT_STATE_LONG_SIT_ALERT:
        text = "Please stand up";
        break;

    case LOW_POWER_WARNING:
        text = "Please charge";
        break;

    case OTA_UPDATING:
        text = "Updating...";
        break;

    default:
        return; /* No text for unknown state */
    }

    if (lv_obj_is_valid(hint_label))
    {
        lv_label_set_text(hint_label, text);
        lv_obj_set_style_text_font(hint_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    }
}

/**
 * @brief Timer callback to create blinking effect for images
 * @param task Timer task object
 */
static void blink_timer_cb(lv_timer_t *task)
{
    lv_obj_t *img = (lv_obj_t *)task->user_data;
    if (!lv_obj_has_flag(img, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(img, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_flag(img, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief Update OTA progress bar and text
 * @param progress Progress value (0-100)
 */
void update_progress_bar(uint8_t progress)
{
    lv_obj_t *progress_bar = get_progress_bar();
    lv_obj_t *hint_label = get_hint_label();

    if (lv_obj_is_valid(progress_bar) == NULL || lv_obj_is_valid(hint_label) == NULL)
        return;

    lv_bar_set_value(progress_bar, progress, LV_ANIM_ON);

    char text[8];
    sprintf(text, "%d%%", progress);
    lv_label_set_text(hint_label, text);
}

/**
 * @brief Callback for the ignore button on sit alert
 * @param event LVGL event
 */
static void ignore_btn_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        on_sit_alert_dismissed();
        gui_app_self_exit();
    }
}

static void close_btn_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        gui_app_self_exit();
    }
}

static void create_hint_label(lv_obj_t *parent, interact_state_t state)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    set_hint_label(label);
    set_text_hint(state);
}

LV_IMG_DECLARE(img_low_power);

/**
 * @brief Create UI components based on current interaction state
 * @param state Current interaction state
 * @param parent Parent container
 */
static void builder(interact_state_t state, lv_obj_t *parent)
{
    switch (state)
    {
    case INTERACT_STATE_FIND_ME:
    {
        create_hint_label(parent, state);
        lv_obj_center(get_hint_label());
        lv_obj_t *close_btn = common_text_button(parent, "Close", get_system_font_size(0), 366, 100, close_btn_event_callback);
        lv_obj_align(close_btn, LV_ALIGN_BOTTOM_MID, 0, -100);
        ui_components.close_btn = close_btn;

        if (get_motor_switch_state())
        {
            motor_params_t param = {
                .duty_cycle = 70,
                .period = 1000000, // 100ms
                .repeat_times = 30,
            };
            peripheral_provider.control_motor(true, &param);
        }

        break;
    }

    case INTERACT_STATE_LONG_SIT_ALERT:
    {
        create_hint_label(parent, state);
        /* Create ignore button */
        lv_obj_t *ignore_btn = common_text_button(parent, "Ignore", get_system_font_size(0), 366, 100, ignore_btn_event_callback);
        lv_obj_align(ignore_btn, LV_ALIGN_BOTTOM_MID, 0, -100);
        ui_components.ignore_button = ignore_btn;
        if (SkaiWatchSys.DNDMode.config.status) // DND mode
        {
            return;
        }
        if (get_motor_switch_state())
        {
            motor_params_t param = {
                .duty_cycle = 51,
                .period = 200000, // 200ms
                .repeat_times =3,
            };
            peripheral_provider.control_motor(true, &param);
        }
        break;
    }

    case LOW_POWER_WARNING:
    {
        /* Create image based on state */
        lv_obj_t *img = lv_img_create(parent);
        ui_components.status_image = img;
        lv_img_set_src(img, (state == LOW_POWER_WARNING) ? LV_EXT_IMG_GET(img_low_power) : &mobile);
        lv_obj_center(img);

        /* Create and position label */
        create_hint_label(parent, state);
        lv_obj_align_to(get_hint_label(), img, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

        /* Create blinking effect timer */
        ui_components.blink_timer = lv_timer_create(blink_timer_cb, 500, img);
        lv_obj_t *exit_btn = lv_obj_create(parent);
        lv_obj_set_size(exit_btn, 466, 466);
        lv_obj_align(exit_btn, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_opa(exit_btn, LV_OPA_0, 0);
        lv_obj_add_event_cb(exit_btn, close_btn_event_callback, LV_EVENT_CLICKED, NULL);
        break;
    }

    case OTA_UPDATING:
    {
        /* Create progress bar */
        lv_obj_t *bar = lv_bar_create(parent);
        set_progress_bar(bar);
        lv_obj_set_size(bar, lv_pct(80), 20);
        lv_obj_center(bar);
        lv_bar_set_range(bar, 0, 100);
        lv_bar_set_value(bar, 0, LV_ANIM_OFF);

        /* Set progress bar style */
        lv_obj_set_style_bg_color(bar, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);

        /* Create progress text */
        create_hint_label(parent, state);
        lv_obj_align(get_hint_label(), LV_ALIGN_CENTER, 0, 30);
        update_progress_bar(SkaiWatchSys.ota_progress);
        break;
    }

    case INTERACT_STATE_SHOW_QRCODE:
    {
        lv_color_t light_color = lv_palette_lighten(LV_PALETTE_LIGHT_BLUE, 5);
        lv_color_t dark_color = lv_palette_darken(LV_PALETTE_BLUE, 4);

        lv_obj_t *qr = lv_qrcode_create(parent);
        ui_components.qr_code = qr;
        lv_qrcode_setparam(qr, 200, dark_color, light_color);
        lv_qrcode_update(qr, qrcode_data, strlen(qrcode_data));
        lv_obj_center(qr);

        lv_obj_set_style_border_color(qr, light_color, 0);
        lv_obj_set_style_border_width(qr, 5, 0);
        break;
    }

    default:
        break;
    }
}

/**
 * @brief Initialize UI when app starts
 * @param parent Parent container
 * @return Initialized parent container
 */
static lv_obj_t *on_start(lv_obj_t *parent)
{
    init_ui_components();
    ui_helper_set_interact_mode(true);
    builder(interact_state, parent);
    return parent;
}

/**
 * @brief Handle app resume state
 */
static void on_resume(void)
{
    /* Disable power saving mode */
    setting_provider.set_power_save_mode(0);

#ifdef BSP_USING_UI_HANDLER
    /* Set up state-specific handlers */
    switch (interact_state)
    {
    case OTA_UPDATING:
        lvgl_msg_handler.handle_ota_update = update_progress_bar;
        break;

    default:
        /* No specific handlers for other states */
        break;
    }
#endif
}

/**
 * @brief Handle app pause state
 */
static void on_pause(void)
{
    /* Enable power saving mode */
    setting_provider.set_power_save_mode(1);
    if (ui_components.blink_timer)
    {
        lv_timer_del(ui_components.blink_timer);
        ui_components.blink_timer = NULL;
    }
    /* Clean up state-specific resources */
    switch (interact_state)
    {
    case OTA_UPDATING:
        lvgl_msg_handler.handle_ota_update = NULL;
        break;

    default:
        /* No specific cleanup for other states */
        break;
    }
}

/**
 * @brief Handle app stop state
 */
static void on_stop(void)
{
    /* State-specific cleanup */
    switch (interact_state)
    {
    case INTERACT_STATE_FIND_ME:
        peripheral_provider.control_motor(false, NULL);
        break;

    case INTERACT_STATE_LONG_SIT_ALERT:
        sit_alert_acknowledge();
        break;

    default:
        /* No specific cleanup for other states */
        break;
    }

    /* Clean up UI components and reset state */
    cleanup_ui_components();
    interact_state = TEST_STATE_UNKNOWN;
    ui_helper_set_interact_mode(false);
}

/**
 * @brief Main message handler for the app lifecycle
 * @param msg Message type
 * @param param Message parameters
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

/**
 * @brief Main app entry point
 * @param i Intent data
 * @return Status code (0 = success)
 */
static int app_main(intent_t i)
{
    const char *intent = intent_get_string(i, "intent");
    if (intent)
    {
        LOG_I("intent: %s", intent);

        /* Process different intents */
        if (strcmp(intent, "find_me") == 0)
        {
            interact_state = INTERACT_STATE_FIND_ME;
        }
        else if (strcmp(intent, "show_qrcode") == 0)
        {
            interact_state = INTERACT_STATE_SHOW_QRCODE;
        }
        else if (strcmp(intent, "long_sit_alert") == 0)
        {
            interact_state = INTERACT_STATE_LONG_SIT_ALERT;
        }
        else if (strcmp(intent, "low_power_warning") == 0)
        {
            interact_state = LOW_POWER_WARNING;
        }
        else if (strcmp(intent, "ota_update") == 0)
        {
            interact_state = OTA_UPDATING;
        }
    }

    /* Register message handler */
    gui_app_regist_msg_handler(APP_ID_INTERACT, msg_handler);
    return 0;
}

/* Export app to the system */
BUILTIN_APP_EXPORT(LV_EXT_STR_ID(skaiwalk_demo), IMG_LOGO, APP_ID_INTERACT, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/