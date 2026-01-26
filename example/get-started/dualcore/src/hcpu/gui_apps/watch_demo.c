/*********************
 *      INCLUDES
 *********************/
#include "littlevgl2rtt.h"
#include "lv_ext_resource_manager.h"
#include <rtdevice.h>
#ifndef _WIN32
    #include "drv_lcd.h"
#endif
#include "gui_app_fwk.h"
#include "lv_ex_data.h"
#include "app_mem.h"
#include "log.h"
#include "lv_freetype.h"
#include "touch_state_manager.h"
#include "drv_touch.h"
#include "app_message.h"
#include "app_speech.h"
#include "app_mainmenu.h"
#include "ui_handler.h"
#include "ui_helper.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
    #include "watch_system_interact.h"
    #include "watch_system_core_task.h"
#endif
#ifdef BSP_USING_BLOC
    #include "bloc_control.h"
    #include "bloc_v2t.h"
    #include "bloc_setting.h"
    #include "bloc_peripheral.h"
    #include "bloc_skaiwalk.h"
    #include "bloc_motion_tracking.h"
    #include "bloc_flash.h"
    #include "bloc_system_perception.h"
#endif
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
    #include "gui_app_pm.h"
    #include "drv_gpio.h"
#endif /* BSP_USING_PM */

#include "data_service_subscriber.h"

#define APP_WATCH_GUI_TASK_STACK_SIZE 16 * 1024

#define SLEEP_CTRL_PIN (BSP_KEY1_PIN)
#define LCD_DEVICE_NAME "lcd"
#define IDLE_TIME_LIMIT (10000)

typedef enum
{
    BTN_EVT_SHUTDOWN = 0x01,
    BTN_EVT_UI_CLOSE = 0x02,
    BTN_EVT_UI_OPEN = 0x04,
    BTN_EVT_ALL = BTN_EVT_SHUTDOWN | BTN_EVT_UI_CLOSE | BTN_EVT_UI_OPEN,
} btn_evt_type_t;

static struct rt_thread watch_thread;

ALIGN(RT_ALIGN_SIZE)
static uint8_t watch_thread_stack[APP_WATCH_GUI_TASK_STACK_SIZE];
static rt_device_t lcd_device;

static lv_timer_t *button_event_task;
static struct rt_event btn_event;
static lv_obj_t *mbox;

/*Compatible with private lib*/
uint32_t g_mainmenu[2];

static int32_t button_handle;

extern void ui_datac_init(void);

#ifdef BSP_USING_PM
extern bool lv_refreshing_done(void);
#endif /* BSP_USING_PM */

static void handle_back_in_mainmenu(bool is_button)
{
    if (is_at_app_list())
    {
        LOG_D("handle_back_in_mainmenu: is_at_app_list");
        clock_on_resume();
        animate_to_home_from_app_list();
        screen_rotate_back_to_original_direction();
    }
    else if (is_at_home())
    {
        if (is_button)
        {
            LOG_D("handle_back_in_mainmenu: is_at_home");
            watch_system_interact(WATCH_SLEEP, NULL);
        }
    }
    else
    {
        LOG_D("is_at_other");
    }
}

extern void note_list_handle_back(void);
static void handle_back_event(bool is_button)
{
    if (get_idle_state() && get_sys_power_status() == SYS_POWER_STATUS_ON)
    {
        watch_hcpu_resume_with_reason(WAKEUP_REASON_OTHER);
    }
    // else if (gui_app_is_actived(APP_ID_BATTERY) ||
    // gui_app_is_actived(APP_ID_MESSAGE) ||
    // gui_app_is_actived(APP_ID_APP_LIST))
    // {
    //     gui_app_goback();
    // }
    else if (is_ble_dfu_thread_running())
    {
        LOG_I("ESC in ble dfu => return");
        return;
    }
    else if (gui_app_is_actived(APP_ID_SPEECH))
    {
        LOG_D("ESC => trigger_back_event in speech app");
        control_provider.trigger_back_event();
    }
    else if (!gui_app_is_actived(APP_ID_MAIN))
    {
        if (gui_app_is_actived(APP_ID_MOUSE))
        {
            gui_app_goback();
        }
        else if (gui_app_is_actived(APP_ID_EXERCISE))
        {
            LOG_D("ESC => trigger_back_event in exercise app");
            control_provider.trigger_back_event();
        }
        else
        {
            LOG_D("ESC in other app => gui_app_goback");
            gui_app_goback();
            if (is_at_home())
            {
                screen_rotate_back_to_original_direction();
            }
        }
    }
    else if (is_at_ai_interface())
    {
        extern void back_tap_cb(void);
        back_tap_cb();
        LOG_D("ESC in AI interface => quick_ai_hint_hidden");
    }
    else if (is_at_control_center())
    {
        animate_to_home_from_notification_center();
    }
    else if (is_at_note_list())
    {
        LOG_D("ESC in note list => note_list_handle_back");
        note_list_handle_back();
    }
    // else if (is_at_app_list())
    // {
    //     extern void back_on_skai_widget(void);
    //     back_on_skai_widget();
    //     LOG_D("ESC in AI widget => close_skai_widget_ai");
    // }
    else if (is_at_mouse_mode() || is_at_message() || is_at_app_list())
    {
        clock_on_resume();
        animate_to_home_from_notification_center();
        screen_rotate_back_to_original_direction();
    }
    else if (gui_app_is_actived(APP_ID_MAIN))
    {
        LOG_D("ESC in main menu => handle_back_in_mainmenu");
        handle_back_in_mainmenu(is_button);
    }
}


void ui_layer_top_builder(void)
{
#ifdef SHOW_UNKNOWN_GESTURE_INDICATOR
    unknown_indicator_builder(lv_layer_top(), gui_app_get_gesture_indicator());
#endif

#ifdef SHOW_UNGRAB_ENABLE_INDICATOR
    gesture_release_indicator_builder(lv_layer_top(),
                                      gui_app_get_gesture_indicator());
#endif

#ifdef SHOW_OPEN_WATCH_HINT_LIGHT
    open_watch_hint_builder(lv_layer_top(), gui_app_get_gesture_indicator());
#endif

#ifdef SHOW_BAD_SIGNAL_INDICATOR
    bad_signal_indicator_builder(lv_layer_top());
#endif
}

void ui_layer_system_builder(void)
{
    lv_obj_clear_flag(lv_layer_sys(), LV_OBJ_FLAG_CLICKABLE);
    ai_icon_hint_builder(lv_layer_sys(), gui_app_get_gesture_indicator());
    tap_indicator_builder(lv_layer_sys(), gui_app_get_gesture_indicator());
    ungrab_indicator_builder(lv_layer_sys(), gui_app_get_gesture_indicator());
    // create_message_toast(lv_layer_sys());
}

/**
 * return to MAIN_APP or CLOCK_APP
 * \n
 *
 * @param event
 * @return
 * @param key
 * \n
 * @see
 */
static int32_t default_keypad_handler(lv_key_t key, lv_indev_state_t event)
{
    static lv_indev_state_t last_event = LV_INDEV_STATE_REL;

    if (last_event != event) // Not execute repeatly.
    {
        last_event = event;
        LOG_I("[%s]key:%d, event:%d", __func__, key, event);
        if ((LV_INDEV_STATE_PR == event) && (LV_KEY_HOME == key))
        {
            handle_back_event(true);
        }
        else if ((LV_INDEV_STATE_PR == event) && (LV_KEY_ESC == key))
        {
            handle_back_event(true);
        }
        else if ((LV_INDEV_STATE_PR == event) && (LV_KEY_NEXT == key))
        {
            handle_back_event(true);
        }
        else if ((LV_INDEV_STATE_PR == event) && (LV_KEY_ENTER == key))
        {
            LOG_I("ENTER key event was %d", event);
            if (is_at_ai_interface())
            {
                extern bool get_enable_tap_and_hold(void);
                if (!get_enable_tap_and_hold())
                {
                    if (peripheral_provider.get_tap_status())
                    {
                        extern void ai_tap_cb(void);
                        ai_tap_cb();
                    }
                }
                else
                {
                    if (!peripheral_provider.get_tap_status())
                    {
                        extern void ai_tap_cb(void);
                        ai_tap_cb();
                    }
                }
            }
            else if (is_at_message() && !gui_app_is_actived(APP_ID_SPEECH))
            {
                extern bool get_enable_tap_and_hold(void);
                if (!get_enable_tap_and_hold())
                {
                    if (peripheral_provider.get_tap_status())
                    {
                        if (myLancher[app_index_message].on_tap)
                            myLancher[app_index_message].on_tap();
                    }
                }
                else
                {
                    if (!peripheral_provider.get_tap_status())
                    {
                        if (myLancher[app_index_message].on_tap)
                            myLancher[app_index_message].on_tap();
                    }
                }
            }
            else if (is_at_app_list())
            {
                extern bool get_enable_tap_and_hold(void);
                if (!get_enable_tap_and_hold())
                {
                    if (peripheral_provider.get_tap_status())
                    {
                        if (myLancher[app_index_app_list].on_tap)
                            myLancher[app_index_app_list].on_tap();
                    }
                }
                else
                {
                    if (!peripheral_provider.get_tap_status())
                    {
                        if (myLancher[app_index_app_list].on_tap)
                            myLancher[app_index_app_list].on_tap();
                    }
                }
            }
            else if ((app_control_get_mouse_mode() || is_at_mouse_mode()))
            {
                if (peripheral_provider.get_tap_status())
                {
                    control_provider.ble_hid_mouse_left_press();
                }
                else
                {
                    control_provider.ble_hid_mouse_left_release();
                }
            }
            else if (lvgl_msg_handler.handle_tap_indicator)
            {
                if (peripheral_provider.get_tap_status())
                {
                    lvgl_msg_handler.handle_tap_indicator(1);
                }
                else
                {
                    lvgl_msg_handler.handle_tap_indicator(0);
                }
            }
        }
    }

    return LV_BLOCK_EVENT;
}

#ifdef USING_BUTTON_LIB

    #include "button.h"

    #ifdef BSP_KEY1_ACTIVE_HIGH
        #define BUTTON_ACTIVE_POL BUTTON_ACTIVE_HIGH
    #else
        #define BUTTON_ACTIVE_POL BUTTON_ACTIVE_LOW
    #endif

typedef enum
{
    KEYPAD_KEY_HOME = 2,
} keypad_key_code_t;

typedef enum
{
    KEYPAD_KEY_STATE_REL,
    KEYPAD_KEY_STATE_PRESSED,
} keypad_key_state_t;

typedef struct
{
    keypad_key_code_t last_key;
    keypad_key_state_t last_key_state;
} keypad_status_t;

static int32_t key1_button_handle = -1;
static keypad_status_t keypad_status;

void button_key_read(uint32_t *last_key, lv_indev_state_t *state)
{
    if (last_key && state)
    {
        *last_key = keypad_status.last_key;
        if (KEYPAD_KEY_STATE_REL == keypad_status.last_key_state)
        {
            *state = LV_INDEV_STATE_REL;
        }
        else
        {
            *state = LV_INDEV_STATE_PR;
            keypad_status.last_key_state = KEYPAD_KEY_STATE_REL;
        }
    }
}

static void button_event_handler(int32_t pin, button_action_t action)
{
    rt_kprintf("pin: %d, action: %d\n", pin, action);
}

/* button event handler in UI inactive state */
static void handle_button_action(int32_t pin, button_action_t action)
{
    #ifdef BSP_USING_PM
    gui_pm_action_t pm_action;

    LOG_I("button:%d,%d", pin, action);

    if ((SLEEP_CTRL_PIN == pin) &&
        (!gui_is_active() || (action == BUTTON_LONG_PRESSED)))
    {
        pm_action = GUI_PM_ACTION_INVALID;
        switch (action)
        {
        case BUTTON_PRESSED:
        {
            pm_action = GUI_PM_ACTION_BUTTON_PRESSED;
            break;
        }
        case BUTTON_RELEASED:
        {
            pm_action = GUI_PM_ACTION_BUTTON_RELEASED;
            break;
        }
        case BUTTON_CLICKED:
        {
            pm_action = GUI_PM_ACTION_BUTTON_CLICKED;
            break;
        }
        case BUTTON_LONG_PRESSED:
        {
            pm_action = GUI_PM_ACTION_BUTTON_LONG_PRESSED;
            break;
        }
        default:
        {
            pm_action = GUI_PM_ACTION_INVALID;
        }
        }
        if (GUI_PM_ACTION_INVALID != pm_action)
        {
            gui_pm_fsm(pm_action);
        }
    }
    else
    #endif /* BSP_USING_PM */
    {
        lv_disp_trig_activity(NULL);
        switch (action)
        {
        case BUTTON_CLICKED:
        {
            keypad_status.last_key = KEYPAD_KEY_HOME;
            keypad_status.last_key_state = KEYPAD_KEY_STATE_PRESSED;
            break;
        }
        default:
            break;
        }
    }
}

void lvgl_set_global_keypad_enter_cmd(void)
{
    keypad_status.last_key = 1;
    keypad_status.last_key_state = KEYPAD_KEY_STATE_PRESSED;
}

void lvgl_set_global_keypad_esc_cmd(void)
{
    keypad_status.last_key = 3;
    keypad_status.last_key_state = KEYPAD_KEY_STATE_PRESSED;
}

static int button_service_callback(data_callback_arg_t *arg)
{
    LOG_I("button_service_callback: msg_id 0x%x", arg->msg_id);
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        button_action_t action;
        RT_ASSERT(arg->data_len == sizeof(action));
        action = *(button_action_t *)arg->data;

        handle_button_action((int32_t)arg->user_data, action);
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
    }

    return 0;
}

static void init_pin(void)
{
    #if (SLEEP_CTRL_PIN < GPIO1_PIN_NUM)
    button_cfg_t cfg;
        #if defined(BSP_USING_PM) && !defined(SF32LB52X)
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;
        #endif /* BSP_USING_PM && !SF32LB52X */

    cfg.pin = SLEEP_CTRL_PIN;
    cfg.active_state = BUTTON_ACTIVE_POL;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));
    key1_button_handle = id;

        #if defined(BSP_USING_PM) && !defined(SF32LB52X)
    gpio = GET_GPIO_INSTANCE(SLEEP_CTRL_PIN);
    gpio_pin = GET_GPIOx_PIN(SLEEP_CTRL_PIN);

    wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);
    RT_ASSERT(wakeup_pin >= 0);

    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_DOUBLE_EDGE);
        #endif /* BSP_USING_PM && !SF32LB52X */

    #endif /* SLEEP_CTRL_PIN < GPIO1_PIN_NUM */

    button_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != button_handle);
    /* subscribe btn0 service to get button event */
    datac_subscribe(button_handle, "btn0", button_service_callback,
                    SLEEP_CTRL_PIN);
}

#else
    #define init_pin()
#endif /* USING_BUTTON_LIB */

#ifdef BSP_USING_PM
static void opa_anim(void *bg, int32_t v)
{
    lv_obj_set_style_bg_opa(bg, (lv_opa_t)v, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void mbox_event_cb(lv_event_t *event)
{
    lv_obj_t *mbox = lv_event_get_user_data(event);
    uint16_t btn_idx = lv_msgbox_get_active_btn(mbox);

    LOG_I("mbox_event VALUE_CHANGED: %d", btn_idx);
    if (0 == btn_idx)
    {
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_POWEROFF, NULL);
        pm_shutdown();
    }
    else
    {
        /* Delete the parent modal background */
        lv_obj_del_async(lv_obj_get_parent(mbox));
        // Restore HOME key
        keypad_default_handler_register(default_keypad_handler);
    }
}

static void show_shutdown_msgbox(void)
{
    /* Create a base object for the modal background */
    lv_obj_t *obj = lv_obj_create(lv_scr_act());
    lv_obj_set_style_bg_color(obj, LV_COLOR_BLACK,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, LV_HOR_RES, LV_VER_RES);

    static const char *btns2[] = {"Ok", "Cancel", ""};

    /* Create the message box as a child of the modal background */
    lv_obj_t *mbox = lv_msgbox_create(
        obj, "Shutdown", "Are you sure to shutdown?", btns2, false);
    lv_obj_align(mbox, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(mbox, mbox_event_cb, LV_EVENT_VALUE_CHANGED, mbox);

    /* Fade the message box in with an animation */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_time(&a, 500);
    lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_50);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)opa_anim);
    lv_anim_start(&a);

    // Disable HOME key
    keypad_default_handler_register(NULL);
    // lv_label_set_text(info, in_msg_info);
    // lv_obj_align(info, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 5, -5);
}

static void button_event_task_entry(struct _lv_timer_t *task)
{
    rt_uint32_t evt;
    rt_err_t err;
    rt_uint32_t limit_time = IDLE_TIME_LIMIT;
    if (SkaiWatchSys.oled_display_time > 60)
    {
        // never sleep
        limit_time = 0xFFFFFFFF;
    }
    else
    {
        limit_time = SkaiWatchSys.oled_display_time * 1000;
    }

    // #if !kReleaseMode
    extern bool pause_sleep_cause_of_dev_reson(void);
    // if (!pause_sleep_cause_of_dev_reson())
    // #endif
    // {
    //     if (is_timeout && setting_provider.get_power_save_mode() &&
    //         !never_sleep)
    //     {
    //         send_sys_interact_event(SYS_EVENT_HCPU_SUSPEND);
    //     }
    // }

    if (lv_disp_get_inactive_time(NULL) > limit_time && setting_provider.get_power_save_mode() && !pause_sleep_cause_of_dev_reson())
    {
        peripheral_provider.hcpu_suspend();
        gui_pm_fsm(GUI_PM_ACTION_SLEEP);
    }

    err = rt_event_recv(&btn_event, BTN_EVT_ALL,
                        RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO,
                        &evt);

    if (RT_EOK != err)
    {
        return;
    }

    if (evt & BTN_EVT_SHUTDOWN)
    {
        lv_disp_trig_activity(NULL);
        show_shutdown_msgbox();
    }
}

static void pm_event_handler(gui_pm_event_type_t event)
{
    switch (event)
    {
    case GUI_PM_EVT_SUSPEND:
    {
        lv_timer_enable(false);
        break;
    }
    case GUI_PM_EVT_RESUME:
    {
        lv_timer_enable(true);
        break;
    }
    case GUI_PM_EVT_SHUTDOWN:
    {
        // TODO: start power down procedure
        RT_ASSERT(RT_EOK == rt_event_send(&btn_event, BTN_EVT_SHUTDOWN));
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }
}
#else
    #define rt_pm_request(mode)
    #define rt_pm_release(mode)
#endif /* BSP_USING_PM */


/* Touch event callback - directly called from driver */
static void on_touch_event(uint8_t event, uint16_t x, uint16_t y)
{
    /* Update global touch state */
    touch_state_update(event, x, y);
}

/* Touch gesture callback - called after gesture detection */
static void on_touch_gesture(touch_gesture_t gesture, uint16_t x, uint16_t y)
{
    switch (gesture)
    {
    case TOUCH_GESTURE_PRESSED:
        LOG_D("Touch pressed at (%d, %d)", x, y);
        // Handle initial touch moment if needed
        // if (get_idle_state() && get_sys_power_status() == SYS_POWER_STATUS_ON)
        // {
        //     watch_hcpu_resume_with_reason(WAKEUP_REASON_OTHER);
        // }
        break;
    case TOUCH_GESTURE_QUICK_CLICK:
        LOG_D("Quick click at (%d, %d)", x, y);
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
        // Handle quick click wakeup if needed
        if (!gui_is_active())
        {
            gui_pm_fsm(GUI_PM_ACTION_BUTTON_CLICKED);
            peripheral_provider.hcpu_resume();
        }
        // if (get_sys_power_status() == SYS_POWER_STATUS_SLEEP)
        // {
        //     LOG_D("Wakeup from sleep due to quick click");
        //     // watch_hcpu_resume_with_reason(WAKEUP_REASON_OTHER);
        // }
        // else if (get_sys_power_status() == SYS_POWER_STATUS_OFF)
        // {
        //     LOG_D("Wakeup from off due to quick click");
        //     // watch_system_interact(STANDBY_WAKEUP, NULL);
        // }
        else
        {
            LOG_D("System is already ON, no action taken");
        }
#endif
        break;
    case TOUCH_GESTURE_LONG_PRESS:
        LOG_D("Long press at (%d, %d)", x, y);
        break;
    default:
        break;
    }
}

void app_watch_entry(void *parameter)
{
    uint8_t first_loop = 1;
#ifdef _MSC_VER
    {
        extern int wait_platform_init_done(void);
        wait_platform_init_done();
    }
#else
    {
        set_date(2022, 7, 1);
        set_time(9, 0, 0);
    }
#endif /* _MSC_VER */

    init_pin();
    lcd_device = rt_device_find(LCD_DEVICE_NAME);

#ifdef BSP_USING_PM
    rt_event_init(&btn_event, "btn", RT_IPC_FLAG_FIFO);
    #if defined(TOUCH_IRQ_PIN)
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;
    gpio = GET_GPIO_INSTANCE(TOUCH_IRQ_PIN);
    gpio_pin = GET_GPIOx_PIN(TOUCH_IRQ_PIN);

    wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);
    RT_ASSERT(wakeup_pin >= 0);

    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_POS_EDGE);

    /* Initialize touch state manager */
    touch_state_manager_init();

    /* Register callbacks with touch driver */
    rt_touch_set_event_callback(on_touch_event);
    touch_gesture_register_callback(on_touch_gesture);
    #endif
    gui_ctx_init();
    gui_pm_init(lcd_device, pm_event_handler);
#endif /* BSP_USING_PM */

    /* init littlevGL */
    {
        rt_err_t r = littlevgl2rtt_init(LCD_DEVICE_NAME);
        RT_ASSERT(RT_EOK == r);
    }

    lv_ex_data_pool_init();
    resource_init();
#if LV_USING_FREETYPE_ENGINE
    lv_freetype_open_font(true); /* open freetype */
#endif
    gui_app_init();
    ui_datac_init();
    app_message_data_init();
    app_speech_data_init();
// #ifndef BSP_USING_PC_SIMULATOR
    watch_config_struct_flash_read();
    rt_thread_mdelay(30);
    save_device_address_to_fs();
// #else
//     extern void get_notification_list_from_template(void);
//     get_notification_list_from_template();
//     extern void get_messages_list_from_template(void);
//     get_messages_list_from_template();
// #endif
    bloc_setting_load_watch_system();
    // bloc_system_schedule_init();
    load_note_list_from_file();

#ifdef BSP_USING_PM
    SkaiWatchSys.oled_display_time = 5;
    button_event_task = lv_timer_create(button_event_task_entry, 30, 0);
#endif /* BSP_USING_PM */
    keypad_default_handler_register(default_keypad_handler);

    gui_app_run("Main");
    lv_disp_trig_activity(NULL);

    SkaiWatchSys.motion_control_lock = true;
    setting_provider.set_power_save_mode(1);

    SubscribeDualCoreSyncService();
    rt_thread_mdelay(3000);
    peripheral_provider.hcpu_resume();

    rt_thread_mdelay(100);

    extern void ble_dev_mgr_start_main_phone_check_timer(uint32_t interval_ms);
    ble_dev_mgr_start_main_phone_check_timer(5000);

    ui_layer_system_builder();
    // ui_layer_top_builder();
#if defined(GUI_APP_FRAMEWORK) && (!defined(APP_TRANS_ANIMATION_NONE))
    lvsf_gesture_init(lv_layer_top());
#endif /* defined(GUI_APP_FRAMEWORK)&&(!defined (APP_TRANS_ANIMATION_NONE)) */

    while (1)
    {
        int ms;

        rt_pm_request(PM_SLEEP_MODE_IDLE);
        ms = lv_timer_handler();
        rt_pm_release(PM_SLEEP_MODE_IDLE);

#ifdef BSP_USING_PM
        if (gui_is_force_close())
        {
            if (lv_refreshing_done())
            {
                LOG_I("no input:%d", lv_disp_get_inactive_time(NULL));
                gui_suspend();
                LOG_I("ui resume");
                /* force screen to redraw */
                lv_obj_invalidate(lv_scr_act());
                /* reset activity timer */
                lv_disp_trig_activity(NULL);
            }
            else if (ms > 0)
            {
                rt_thread_mdelay(ms); /* Just to let the system breathe */
            }
        }
        else
#endif /* BSP_USING_PM */
        {
            // EventStartB(0);
            if (ms > 0)
                rt_thread_mdelay(ms); /* Just to let the system breathe */
            // EventStopB(0);
        }

        if (first_loop)
        {
#ifndef WIN32
            // Turn on lcd backlight after power on
            uint8_t brightness = 100;
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BRIGHTNESS,
                              &brightness);
#endif /* WIN32 */
            first_loop = 0;
        }
    }
}

void app_register(void)
{
}

int app_watch_init(void)
{
    rt_err_t ret = RT_EOK;
    rt_thread_t thread = RT_NULL;

    ret = rt_thread_init(&watch_thread, "app_watch", app_watch_entry, RT_NULL,
                         watch_thread_stack, APP_WATCH_GUI_TASK_STACK_SIZE,
                         RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);

    if (RT_EOK != ret)
    {
        return RT_ERROR;
    }
    rt_thread_startup(&watch_thread);
    return RT_EOK;
}

#if !defined(_MSC_VER)
    #define APP_MEM_THRESHOLD 16384
void *cxx_mem_allocate(size_t size)
{
    if (size > APP_MEM_THRESHOLD)
    {
        void *p = app_anim_mem_alloc(size, 1);
        rt_kprintf("Allocate %d, %p\n", size, p);
        return p;
    }
    else
        return rt_malloc(size);
}

extern int rt_in_system_heap(void *ptr);
void cxx_mem_free(void *ptr)
{
    if (rt_in_system_heap(ptr))
        rt_free(ptr);
    else
    {
        rt_kprintf("Free %p\n", ptr);
        app_anim_mem_free(ptr);
    }
}
#endif

INIT_APP_EXPORT(app_watch_init);
