/**
 * @file    app_mainmenu.h
 * @brief  app launcher framework for all app UI
 *
 * \n
 * @details
 * entrance of all apps, and it loads inbuilt apps and customized apps
 *
 * \n
 * @version
 * @author  jack
 * @date    2024-4-23
 *
 * @history
 *
 */
#ifndef __APP_MAINMENU_H__
#define __APP_MAINMENU_H__
#include <rtthread.h>
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"

#define ENABLE_NOTIFICATION_CENTER

#define CONTROL_CENTER_PAGE_INDEX MAIN_PAGE_TYPE_DOWN 
#define HOME_PAGE_INDEX MAIN_PAGE_TYPE_HOME
#define APP_LIST_PAGE_INDEX MAIN_PAGE_TYPE_LEFT
#define MESSAGE_PAGE_INDEX MAIN_PAGE_TYPE_UP
#define AI_PAGE_INDEX MAIN_PAGE_TYPE_RIGHT
// #define ENABLE_APP_LIST

typedef enum
{
    MAIN_PAGE_TYPE_DOWN = 0,
    MAIN_PAGE_TYPE_HOME = 1,
    MAIN_PAGE_TYPE_LEFT = 2,
    MAIN_PAGE_TYPE_UP = 3,
    MAIN_PAGE_TYPE_RIGHT = 4
} main_page_type_t;

typedef enum
{
    APP_SPEECH,
    APP_CLOCK,
    APP_FLASHLIGHT,
} app_id_t;

typedef enum
{
    APP_WIDGET_MEDIA,
    APP_WIDGET_FLASHLIGHT,
    APP_WIDGET_SETTING,
    APP_WIDGET_HEART_RATE,
    APP_WIDGET_WEATHER,
} app_widget_id_t;

typedef rt_int32_t (*app_init_cb_t)(lv_obj_t *);
typedef rt_int32_t (*app_func_ptr_t)(void);

/****                     API for single app              ****/
bool is_at_app_list(void);
bool is_at_note_list(void);
bool is_at_mouse_mode(void);
bool is_at_home(void);
bool is_at_ai_interface(void);
bool is_at_message(void);
bool is_at_control_center(void);
bool is_at_speech_interface(void);
bool get_need_open_gesture_control(void);
void set_need_open_gesture_control(bool need);
extern void gui_set_brightness(uint16_t brightness, bool user_action);
extern void gui_set_screen_timeout(uint16_t timeout_sec);
extern void gui_set_screen_rotation(uint8_t rotation);
extern void unsubscribe_pwr_service(void);
extern void dial_widget_event(lv_event_t *e);
/**
 * get current time  -   API for all apps UI
 *
 * \n
 *
 * @param pt  pointer to return time var
 * \n
 * @see
 */
/**
 * Get clock state change context, currently is app ID.
 *
 */
char *app_change_context(void);

#if 0
    /**
    * duplicate an image to SRAM to improve drawn performance
    * \n
    *
    * @return
    * @param copy
    * \n
    * @see
    */
    lv_img_dsc_t *app_img_cache_malloc(const void *copy);

    void app_img_cache_free(lv_img_dsc_t *p_img);
#endif
extern lv_obj_t *standby_page;
extern void handle_tap_event_in_mainmenu(void);
extern void quick_open_note_chatroom_ui_app(void);
extern void get_calendar_list_from_template(void);
extern void animate_to_notification_center(void);
extern void animate_to_app_list(void);
extern void animate_to_message_list(void);
extern void animate_to_ai_page(void);
extern void animate_to_home_from_ai_page(void);
extern void animate_to_home_from_notification_center(void);
extern void animate_to_home_from_app_list(void);
extern bool get_bluetooth_connection_status(void);
extern void create_connection_tips(void);
#endif /*__APP_MAINMENU_H__*/
