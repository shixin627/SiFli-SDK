#ifndef __WATCH_UI_HANDLER_H__
#define __WATCH_UI_HANDLER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "lvgl.h"
#include "bloc_control.h"
#include "bloc_notification.h"
#include "common_widget.h"
#include "ui_helper.h"

#define APP_ID_QRCODE "qrcode"
#define APP_ID_MAIN "Main"
#define APP_ID_CALCULATOR "calculator"
#define APP_ID_TIMER "timer"
#define APP_ID_EXERCISE "exercise"
#define APP_ID_FLASHLIGHT "flashlight"
#define APP_ID_MOUSE "mouse"
#define APP_ID_INTERACT "interact"
#define APP_ID_MESSAGE "message"
#define APP_ID_RECORDER "recorder"
#define APP_ID_SETTING "setting"
#define APP_ID_SPEECH "speech"
#define APP_ID_WEATHER "weather"
#define APP_ID_PHOTO "photo"
#define APP_ID_MEDIA "media"
#define APP_ID_INCOMING_CALL "incoming_call"
#define APP_ID_BATTERY "battery"
#define APP_ID_CAMERA "camera"
// ----- Todo
#define APP_ID_ALARM "alarm"
// ----- Dev
#define APP_ID_GESTURE "gesture"
#define APP_ID_GAME_DINOSAUR "game_dinosaur"
#if !kReleaseMode
#define APP_ID_FILE_BROWSER "file_browser"
#endif

    typedef enum
    {
        app_id_mainmenu,
        app_id_ai,
        app_id_recorder,
        app_id_flashlight,
        app_id_mouse,
        app_id_media,
        app_id_weather,
        app_id_exercise,
        app_id_alarm,
        app_id_setting,
        app_id_calculator,
        app_id_timer,
        app_id_gesture,
        app_id_notification,
        app_id_speech,
        app_id_interact,
        app_id_thirty,
        app_id_game_dinosaur,
        app_id_photo,
    } watch_app_id_t;

    enum
    {
        LVGL_MSG_TYPE_UNKNOWN = 0,
        /***** Media event ******/
        LVGL_MSG_TYPE_MEDIA_TITLE,
        LVGL_MSG_TYPE_MEDIA_IMG,
        LVGL_MSG_TYPE_MEDIA_HEADER_IMG,
        LVGL_MSG_TYPE_MEDIA_VOLUME,
        /***** IMU event ******/
        LVGL_MSG_TYPE_IMU_ACC,
        LVGL_MSG_TYPE_IMU_GYRO,
        LVGL_MSG_TYPE_IMU_ATTITUDE,

        /***** HR event ******/
        LVGL_MSG_TYPE_HR,
        /***** Battery event ******/
        LVGL_MSG_TYPE_BATTERY_VOLTAGE,
        LVGL_MSG_TYPE_BATTERY_LEVEL,
        LVGL_MSG_TYPE_CHARGE_STATUS,
        /***** Notification event ******/
        LVGL_MSG_TYPE_NOTIFICATION,
        /* calendar */
        LVGL_MSG_TYPE_CALNEDAR,
        LVGL_MSG_TYPE_REFRESH_CALENDAR_WIDGET,
        /* weather */
        LVGL_MSG_TYPE_REFRESH_WEATHER_WIDGET,
        /***** message ******/
        LVGL_MSG_TYPE_MESSAGE,
        LVGL_MSG_TYPE_MESSAGE_STREAM,
        LVGL_MSG_TYPE_INPUT_MESSAGE,
        /***** Watchface ******/
        LVGL_MSG_TYPE_WATCHFACE,
        /***** Mic ******/
        LVGL_MSG_TYPE_MIC,
        LVGL_MSG_TYPE_VAD_STATUS,
        /***** [test] Remote control pageview ******/
        LVGL_MSG_TYPE_PAGEVIEW_ACTION,
        /***** [test] Remote control launcher ******/
        LVGL_MSG_TYPE_LAUNCHER_ACTION,
        /***** virtual gesture event ******/
        LVGL_MSG_TYPE_LONGPRESS_EVENT,
        LVGL_MSG_TYPE_UNGRAB_EVENT,
        LVGL_MSG_TYPE_BACK_EVENT,
        /***** Indicator ******/
        LVGL_MSG_TYPE_TAP_INDICATOR,
        LVGL_MSG_TYPE_UNGRAB_INDICATOR,
        LVGL_MSG_TYPE_UNKNOWN_INDICATOR,
        LVGL_MSG_TYPE_RELEASE_INDICATOR,
        LVGL_MSG_TYPE_BAD_SIGNAL_INDICATOR,
        LVGL_MSG_TYPE_SPEECH_INDICATOR,
        LVGL_MSG_TYPE_WAITING_INDICATOR,
        LVGL_MSG_TYPE_HIDDEN_INDICATOR,
        LVGL_MSG_TYPE_SPEECH_SHOW_BG,
        LVGL_MSG_TYPE_UPDATE_PROCESS_TOOLKIT,
        /***** Control ******/
        LVGL_MSG_TYPE_GYRO_SCROLL_LIST,
        /***** Mouse mode ******/
        LVGL_MSG_TYPE_MOUSE_OPEN_V2T,
        LVGL_MSG_TYPE_MOUSE_OPEN_KEYBOARD,
        LVGL_MSG_TYPE_MOUSE_LONG_PRESS,
        LVGL_MSG_TYPE_MOUSE_INPUT_TEXT,
        /***** Loading ******/
        LVGL_MSG_TYPE_LOADING,
        /***** Time text ******/
        LVGL_MSG_TYPE_TIME_TEXT,
        /***** Bluetooth connection ******/
        LVGL_MSG_TYPE_BLUETOOTH_CONNECTION,
        /****Set app list opa****/
        LVGL_MSG_TYPE_APP_LIST_OPA,
        /****Set app list scroll bar****/
        LVGL_MSG_TYPE_APP_LIST_SCROLL_BAR_OFFSET,
        /****SEND MESSAGE****/
        LVGL_MSG_TYPE_SEND_MESSAGE,
        /****SWITCH SELECTED****/
        LVGL_MSG_TYPE_SWITCH_SELECTED,
        /****MOVE QUICK BTN****/
        LVGL_MSG_TYPE_CONTROL_QUICK_BTN,
        /****MEDIA COUTROL****/
        LVGL_MSG_TYPE_MEDIA_CONTROL,
        LVGL_MSG_TYPE_VOLUME_CONTROL,
        /****WIDGETS COUTROL****/
        LVGL_MSG_TYPE_WIDGETS_CONTROL,
        LVGL_MSG_TYPE_HAND_UP,
        LVGL_MSG_TYPE_NAV_BAR_CONTROL,
        /****WIDGET LIST****/
        LVGL_MSG_TYPE_WIDGET_LIST_SELECT,
        /****OPEN STANDBY PAGE****/
        LVGL_MSG_TYPE_OPEN_STANDBY_PAGE,
        /****CLOSE STANDBY PAGE****/
        LVGL_MSG_TYPE_CLOSE_STANDBY_PAGE,
        /****OTA****/
        LVGL_MSG_TYPE_OTA_UPDATE,
        /****SCROLL MESSAGE PAGE****/
        LVGL_MSG_TYPE_OPEN_MESSAGE_PAGE,
        /****CLEAR NOTIFICATION INDICATIOR*/
        LVGL_MSG_TYPE_CLEAR_NOTIFICATION_BAR_INDICATOR,
        // --- file sync ---
        LVGL_MSG_TYPE_SYNC_STATUS,
        LVGL_MSG_TYPE_SYNC_PROGRESS,
        // --- toast ---
        LVGL_MSG_TYPE_TOAST,
        // motion control
        LVGL_MSG_TYPE_GRAVITY_INDICATOR,
        // --- refresh trigger activity ---
        LVGL_MSG_TYPE_TRIGGER_ACTIVITY,
        /****LVGL_MSG_AI****/
        LVGL_MSG_TYPE_AI_TAP_HINT,
        LVGL_MSG_TYPE_RESET_AI_WIDGET,
        /****header****/
        LVGL_MSG_TYPE_DIAL_HEADER_TIMER,
    };

    typedef struct
    {
        float x;
        float y;
        float z;
    } SpaceVector;

    typedef struct
    {
        char *title;
        char *img_path;
    } MediaInfo;

    typedef struct
    {
        uint8_t app_id;
        bool is_gesture_activation;
        const char *app_name;
        void *param;
    } AppIntentUI;

    typedef struct
    {
        uint16_t gesture_position_x;
        uint16_t gesture_position_y;
    } gesture_position_t;

    typedef struct
    {
        uint8_t swich_quick_btn;
        uint16_t new_point;
    } QuickBtn;

    typedef struct
    {
        uint8_t type; // Message type
        // Flexible size Message data
        union
        {
            struct
            {
                uint8_t x;
                uint8_t y;
                uint8_t state;
            } touch;
            struct
            {
                uint8_t key;
                uint8_t state;
            } key;
            bool media_play_state;
            MediaInfo media_data;
            uint8_t media_volume;
            uint8_t amoled_brightness;
            SpaceVector imu_acc;
            SpaceVector imu_gyro;
            SpaceVector imu_attitude;
            uint16_t battery_voltage;
            uint8_t battery_level;
            bool charge_status;
            int hr;
            notification_t *notification;
            uint8_t watchface;
            uint8_t calendar_day;
            uint8_t opa;
            bool mic_state;
            uint8_t action;
            uint8_t gesture;
            struct
            {
                bool vibrate;
                lv_obj_t *obj_to_view;
            } scroll_action;
            bool scroll_up;
            QuickBtn quick_btn_action;
            gesture_position_t widgets_control;
            gesture_position_t media_control;
            uint8_t volume_control;
            int8_t movement;
            bool loading;
            bool switch_selected;
            AppIntentUI app_intent;
            char *message;
            bool bluetooth_connection;
            uint8_t ota_update;
            char *app_message;
            bool sync_state;
            int scroll_offset;
            bool ai_widget_bg_show;
        } data;
    } lvgl_msg_t;
    // create a struct containing the each message type's handle functions
    typedef struct
    {
        void (*handle_touch)(void *param);
        void (*handle_key)(void *param);
        void (*handle_bar_media_play_state)(void *param);
        void (*handle_bar_media_title)(void *param);
        void (*handle_app_media_play_state)(void *param);
        void (*handle_dial_media_play_state)(void *param);
        void (*handle_app_media_title)(void *param);
        void (*handle_app_media_img)(void *param);
        void (*handle_dial_media_title)(void *param);
        void (*handle_dial_media_img)(void *param);
        void (*handle_dial_media_header_title)(void *param);
        void (*handle_dial_media_header_img)(void *param);
        void (*handle_media_volume)(void *param);
        void (*handle_media_control)(gesture_position_t control);
        void (*handle_volume_control)(uint8_t control);
        void (*handle_widgets_control)(gesture_position_t control);
        void (*handle_hand_up)(bool action);
        void (*handle_nav_bar_control)(int8_t movement);
        void (*handle_imu_acc)(void *param);
        void (*handle_imu_gyro)(void *param);
        void (*handle_imu_attitude)(void *param);
        void (*handle_hr)(int hr);
        void (*handle_battery_voltage)(void *param);
        void (*handle_battery_percentage)(uint8_t level);
        void (*refresh_battery_level)(uint8_t level);
        void (*handle_charge_status)(void *param);
        void (*handle_notification)(void *param);
        void (*handle_new_notification)(void);
        void (*handle_dial_header_new_notification)(void);
        void (*refresh_calendar)(void);
        void (*refresh_message)(void);
        void (*refresh_message_stream)(char *text);
        void (*handle_input_message)(char *text);
        void (*handle_watchface)(void *param);
        void (*handle_mic)(void *param);
        void (*handle_vad_status)(bool status);
        void (*handle_pageview_action)(void *param);
        void (*handle_launcher_action)(void *param);
        void (*handle_ota_update)(uint8_t progress);
        void (*handle_control_quick_btn)(QuickBtn param);
        void (*handle_clear_notification_bar_indicator)(void);
        void (*handle_tap_event)(void);
        void (*handle_longpress_event)(void);
        void (*handle_back_event)(void);
        void (*handle_grab_event)(void);
        void (*handle_ungrab_event)(void);
        void (*handle_loading)(bool loading);
        void (*handle_tap_indicator)(uint8_t gesture);
        void (*handle_gyro_scroll_list)(bool up);
        void (*handle_time_text)(void);
        void (*handle_bluetooth_connection)(bool connected);
        void (*handle_send_message)(void);
        void (*handle_set_instruction_list_opa)(uint8_t opa);
        void (*handle_switch_selected)(bool switch_selected);
        void (*handle_set_arc_stripe_external_offset)(int16_t offset_degrees);
        void (*handle_open_message_page_content)(void);
        void (*handle_refresh_calendar_widget)(void);
        void (*handle_refresh_dial_calendar_widget)(void);
        void (*handle_refresh_weather_widget)(void);
        void (*handle_refresh_dial_weather_widget)(void);
        void (*handle_file_sync)(bool state);
        void (*handle_sync_progress)(uint8_t progress);
        void (*handle_toast)(char *text);
        void (*handle_gravity_indicator)(uint8_t action);
    } lvgl_msg_handler_t;

    extern lvgl_msg_handler_t lvgl_msg_handler;
    extern void lvgl_send_msg(lvgl_msg_t msg);
    extern rt_tick_t get_last_refresh_tick(void);

    extern app_gesture_indicator_t *gui_app_get_gesture_indicator(void);
    extern bool is_ai_interface_active(void);
    extern void clear_all_handlers(void);
    extern void tap_indicator_builder(void *par,
                                      app_gesture_indicator_t *indicator);
    extern void open_watch_hint_builder(void *par,
                                        app_gesture_indicator_t *indicator);
    extern void
    voice_recognition_hint_builder(void *par,
                                   app_gesture_indicator_t *indicator);
    extern void tap_indicator_destroy(app_gesture_indicator_t *indicator);
    extern void unknown_indicator_builder(void *par,
                                          app_gesture_indicator_t *indicator);
    extern void ungrab_indicator_builder(void *par,
                                         app_gesture_indicator_t *indicator);
    extern void
    gesture_release_indicator_builder(void *par,
                                      app_gesture_indicator_t *indicator);
    extern void bad_signal_indicator_builder(void *par);
    extern void reset_lvgl_msg_handler(void);

    extern void open_message_app(const char *notification_id);
    extern void close_message_app(void);
    extern void change_flashlight_switch(bool state);

    extern bool get_scrolling_motor_vibrate_status(void);
    extern void enable_scrolling_motor_vibrate(void);
    extern void disable_scrolling_motor_vibrate(void);

    extern void refresh_charge_icon(void);

#ifdef __cplusplus
}
#endif

#endif //__WATCH_SYSTEM_INTERACT_H__
