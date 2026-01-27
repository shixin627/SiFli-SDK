/**
*****************************************************************************************
*     Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      bloc_control.h
* @brief     business logic of control page
* @author    jack
* @date      2023-01-28
* @version   v1.0
**************************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2023 Skaiwalk Corporation</center></h2>
**************************************************************************************
*/

#ifndef __BLOC_CONTROL_H__
#define __BLOC_CONTROL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"

#define SYS_EVENT_PLAY_PAUSE (1 << 0)
#define SYS_EVENT_NEXT (1 << 1)
#define SYS_EVENT_PREV (1 << 2)
#define SYS_EVENT_VOLUME_UP (1 << 3)
#define SYS_EVENT_VOLUME_DOWN (1 << 4)
#define SYS_EVENT_VOLUME_BTN_UP (1 << 5)
#define SYS_EVENT_VOLUME_BTN_DOWN (1 << 6)
#define SYS_EVENT_MULTIPLE_PAGES (1 << 7)

#define MEDIA_SYS_ALL_EVENT (   \
    SYS_EVENT_PLAY_PAUSE |      \
    SYS_EVENT_NEXT |            \
    SYS_EVENT_PREV |            \
    SYS_EVENT_VOLUME_UP |       \
    SYS_EVENT_VOLUME_DOWN |     \
    SYS_EVENT_VOLUME_BTN_UP |   \
    SYS_EVENT_VOLUME_BTN_DOWN | \
    SYS_EVENT_MULTIPLE_PAGES)

#define ENABLE_MEDIA_VOLUMN_BAR_CONTROL 0

    typedef struct
    {
        bool state;
        int index;
    } gesture_t;

    typedef struct
    {
        bool state;
        int x;
        int y;
        int z;
    } coordinate_t;

    typedef union
    {
        uint8_t touch_flags;
        struct
        {
            uint8_t cycle_type : 2;
            uint8_t cycle_mode : 1;
            uint8_t setting_mode : 1;
            uint8_t vol_switch : 1;
            uint8_t play_pause : 1;
            uint8_t last_music : 1;
            uint8_t next_music : 1;
        } b;

    } music_press_flags;

    typedef struct
    {
        bool state;
        char title[256];
    } media_t;

    // enum for device id
    // 1: mobile, 2: desktop
    typedef enum
    {
        SKAILINK_DEVICE_MOBILE = 0x01,
        SKAILINK_DEVICE_DESKTOP = 0x02,
    } SKAILINK_DEVICE;

    typedef enum
    {
        LAUNCHER_ACTION_HOME = 0x00,
        LAUNCHER_ACTION_APP_LIST = 0x01,
        LAUNCHER_ACTION_GO_HOME_FROM_BAR = 0x02,
    } LAUNCHER_ACTION;

    // Provider object
    typedef struct
    {
        void (*notify_pageview_action)(uint8_t action);
        void (*notify_unit_test_action)(uint8_t action);
        void (*trigger_tap_event)(void);
        void (*trigger_longpress_event)(void);
        void (*trigger_back_event)(void);
        void (*trigger_finger_event)(uint8_t finger_event);
        void (*trigger_unknown_event)(void);
        void (*send_virtual_gesture)(uint8_t gesture);
        // bt_speaker
        bool (*bt_speaker_get_status)(void);
        void (*bt_speaker_set_status)(bool status);
        void (*notify_bt_speaker_media_status)(bool status);
        void (*bt_speaker_set_volume)(uint8_t volume, bool notify);
        uint8_t (*bt_speaker_get_volume)(void);
        void (*bt_speaker_media_volume_up)(void);
        void (*bt_speaker_media_volume_down)(void);
        void (*ble_hid_play_pause)(void);
        void (*bt_speaker_media_play)(void);
        void (*bt_speaker_media_pause)(void);
        void (*bt_speaker_media_next)(void);
        void (*bt_speaker_media_prev)(void);
        void (*ble_hid_consumer_back)(void);
        // media
        void (*set_media_title)(char *title);
        char *(*get_media_title)(void);

        // hid
        void (*ble_hid_mouse_left_press)(void);
        void (*ble_hid_mouse_left_release)(void);
        void (*ble_hid_mouse_left_click)(void);
        void (*ble_hid_mouse_right_press)(void);
        void (*ble_hid_mouse_right_release)(void);
        void (*ble_hid_mouse_right_click)(void);
        void (*ble_hid_mouse_back)(void);
        void (*ble_hid_mouse_move)(int8_t dx, int8_t dy);
        void (*ble_hid_mouse_wheel_scroll)(int8_t delta);
        void (*ble_hid_mouse_pan_scroll)(int8_t delta);
        void (*ble_hid_keyboard_multitask)(bool state);
        void (*ble_hid_keyboard_shift)(bool state);
        void (*ble_hid_keyboard_input)(const char *input);
        void (*ble_hid_keyboard_go_home)(void);
        void (*ble_hid_zoom)(uint8_t device, bool state); // true: zoom in, false: zoom out
        void (*ble_hid_zoom_esc)(void);                   // zoom escape
        void (*ble_hid_touch_screen_press)(uint16_t x, uint16_t y);
        void (*ble_hid_touch_screen_release)(uint16_t x, uint16_t y);
        void (*ble_hid_touchpad_press)(uint16_t x, uint16_t y);
        void (*ble_hid_touchpad_release)(uint16_t x, uint16_t y);
        // remote control
        void (*take_photo)(void);
        void (*find_phone)(void);
        // screen
        void (*rotate_screen)(void);
        void (*screen_brightness)(uint16_t brightness);
        void (*screen_brightness_smoothly)(uint16_t brightness);
        void (*screen_time)(uint16_t time);
        void (*screen_time_smoothly)(uint16_t time);
    } ControlProvider;
    extern ControlProvider control_provider;
    extern music_press_flags music_press_flag;
    extern void reset_audio_processing_flag(void);

    extern void sys_media_event_set(uint32_t event);
    extern uint8_t get_volume_control_value(void);

    extern int movement_threshold;
    /* gesture mode variables */
    extern bool app_control_get_gesture_mode(void);
    extern void app_control_set_gesture_mode(bool mode);

    extern coordinate_t *getCoordinate(void);
    extern void setCoordinateX(int value);
    extern void setCoordinateY(int value);
    extern void setCoordinateZ(int value);

    extern gesture_t *getGesture(void);
    extern void setGestureIndex(int index);

    /* audio */
    extern uint8_t app_audio_get_control_command(void);
    extern void app_audio_set_control_command(uint8_t command);
    extern uint8_t *app_audio_get_volume_percent(void);

    /* cursor */
    extern bool app_control_get_cursor_mode(void);
    extern void app_control_set_cursor_mode(bool flag);
    extern void set_cursor_movement(int8_t deltaX, int8_t deltaY, int speed);
    extern void app_control_set_cursor_gesture(uint8_t gesture);
    extern void Send_Cursor_Report(void);
    extern void Send_Touchpad_Gesture(void);
    /* hid */
    extern void app_control_set_hid_event_flag(bool flag);
    extern bool app_control_get_hid_event_flag(void);
    /* mouse */
    extern bool app_control_get_mouse_mode(void);
    extern void app_control_set_mouse_mode(bool flag);
    /* game */
    extern bool app_control_get_game_mode(void);
    extern void app_control_set_game_mode(bool flag);
    extern bool app_control_get_motion_tracking(void);
    extern void app_control_set_motion_tracking(bool flag);
    /* hid control */
    extern void setSelectIndexOfApp(uint8_t index);
    extern void bloc_control_send_slide_command(uint8_t command);
    extern uint8_t bloc_control_get_skaios_mode_state(void);
    extern void bloc_control_send_skaios_mode_state(void);
    /* unit test */
    extern void set_Unicode(uint16_t unicode);
    extern uint16_t get_Unicode(void);
    /* gui status */
    extern bool bloc_control_get_gui_interactive_mode(void);
    extern void bloc_control_set_gui_interactive_mode(bool flag);
    extern void notify_pageview_action(uint8_t action);
    extern void refresh_ble_mode_btn(void);

#ifdef __cplusplus
}
#endif

#endif //__BLOC_CONTROL_H__
