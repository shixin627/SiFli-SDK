/**
 * @file commmon_widget.h
 *
 */

#ifndef COMMON_WIDGET_H
#define COMMON_WIDGET_H
#define WIDGET_FIRST_THRESHOLD 50
#define WIDGET_SECOND_THRESHOLD 184
#define WIDGET_THIRD_THRESHOLD 388
#define WIDGET_SELECTION_THRESHOLD 466 / 5
#include "lvgl.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define BUTTON_HEIGHT 100
    typedef struct
    {
        lv_obj_t *widget;
        lv_anim_t anim;
    } app_speech_ripple_t;

    typedef struct
    {
        lv_obj_t *main_window;
        lv_obj_t *media_title;
        lv_obj_t *icon_btn_play_pause;
    } app_media_t;

    extern const lv_style_t CUSTOM_BUTTON_STYLE;

    extern lv_obj_t *get_app_list_bluetooth_disconnection(void);
    extern lv_obj_t *get_app_list_battery(void);
    extern lv_obj_t *get_app_list_battery_bg(void);
    extern lv_obj_t *get_app_list_battery_label(void);
    extern lv_obj_t *get_app_list_charge_battery_icon(void);
    extern void set_app_list_time_opa(uint8_t opa);
    extern void set_app_list_time_bg_opa(uint8_t opa);
    extern void set_app_list_battery_opa(uint8_t opa);
    extern void set_app_list_battery_bg_opa(uint8_t opa);
    extern lv_obj_t *get_app_list_time_bg(void);
    extern void show_battery(bool show);
    extern void show_app_list_time(bool show);
    extern lv_obj_t *create_animate_ripples(app_speech_ripple_t *ripple, lv_obj_t *par, uint8_t height);
    extern void trigger_voice_ripple(app_speech_ripple_t *ripple, bool start);
    extern lv_obj_t *create_animate_points(app_speech_ripple_t *point, lv_obj_t *par);
    extern void wait_for_message(app_speech_ripple_t *point, bool start);
    extern void gesture_event_handler_close_app(lv_event_t *e);

    extern lv_obj_t *common_black_bg(lv_obj_t *par);
    extern lv_obj_t *common_flex_button(lv_obj_t *par, bool custom_style, lv_coord_t radius);
    extern lv_obj_t *common_text_button(lv_obj_t *par, const char *text, uint8_t font_size, lv_coord_t w, lv_coord_t h, lv_event_cb_t event_cb);
    extern lv_obj_t *common_icon_button(lv_obj_t *par, const void *img_src, lv_event_cb_t event_cb);
    extern lv_obj_t *common_angle_icon_button(lv_obj_t *par, const void *img_src, lv_event_cb_t event_cb, uint16_t angle);
    extern lv_obj_t *common_icon_button_with_bg(lv_obj_t *par, const void *img_src, lv_event_cb_t event_cb);
    extern lv_obj_t *common_image_button(lv_obj_t *par, const void *img_src, lv_coord_t w, lv_coord_t h, lv_event_cb_t event_cb);
    extern lv_obj_t *common_container(lv_obj_t *parent, lv_coord_t w, lv_coord_t h, lv_event_cb_t event_cb, lv_color_t color);

    extern lv_obj_t *common_list_widget(lv_obj_t *par, lv_style_t *style, lv_coord_t pos_x, lv_coord_t pos_y);
    extern lv_obj_t *common_widget_container(lv_obj_t *parent);

    extern lv_obj_t *create_divider_line(lv_obj_t *parent);
    extern lv_obj_t *create_setting_list_item(lv_obj_t *parent, const void *icon, const char *text, uint8_t font_size, bool show_divider, uint8_t icon_scale);
    extern lv_obj_t *create_dark_toggle_item(lv_obj_t *parent, const char *text, bool initial_state, uint8_t font_size, bool show_divider);

    // Label
    extern lv_obj_t *common_gradient_label(lv_obj_t *parent, const char *text);

    typedef enum
    {
        WMS_WID_NONE = 0,
        WMS_WID_CARD_WATCHFACE,
        WMS_WID_CARD_STATUS_BAR,
        WMS_WID_CARD_APP_ENTRY,
        WMS_WID_CARD_NOTIFICATION,
        WMS_WID_CARD_DAY_ACTIVITY,
        WMS_WID_CARD_HR,
        WMS_WID_CARD_STRESS,
        WMS_WID_CARD_SLEEP,
        WMS_WID_CARD_WEATHER,

        WMS_WID_APP_LIST,
        WMS_WID_SETTING_ENTRY,
        WMS_WID_SETTING_LIST_STYLE,
        WMS_WID_SETTING_SWITCH_EFFECT,
        WMS_WID_SETTING_CARD_STYLE,
        WMS_WID_SETTING_TIME_FORMAT,
        WMS_WID_SETTING_LANGUAGE,
        WMS_WID_SETTING_DISPLAY,
        WMS_WID_SETTING_SCREEN_TIMEOUT,
        WMS_WID_SETTING_AIRPLANE_MODE,
        WMS_WID_SETTING_NO_DISTURB,
        WMS_WID_SETTING_SYSTEM_INFO,
        WMS_WID_DEV_TEST,

        WMS_WID_POWER_OFF,

        WMS_WID_MAX,
    } app_wms_wid_e;

    typedef struct MyStruct
    {
        lv_obj_t *outer_frame;
        lv_obj_t *pagetileview;
        lv_obj_t *day;
        lv_obj_t *blurred_pictures;
        void (*reset_list)(void);
        void (*reset_list_top)(uint8_t day);
        void (*reset_list_down)(uint8_t day);
        void (*on_tap)(void);
        bool list_top;
        bool list_end;
    } MyStruct_T;

    typedef enum
    {
        app_index_app_list,
        app_index_clock,
        app_index_message,
        day_index_calendar_list,
        app_index_controlcent,
        app_index_chatroom,
        app_index_calendar,
        app_index_calendar_list,
        app_index_message_for_applist,
        app_index_ai_interface,
        APP_NULL,
    } app_index_t;

    typedef enum
    {
        calendar_day_one,
        calendar_day_two,
        calendar_day_three,
        calendar_day_four,
        calendar_day_five,
        calendar_day_six,
        calendar_day_seven,
        CALENDAR_NULL,
    } calendar_day_index_t;

    extern MyStruct_T myLancher[APP_NULL];
    extern MyStruct_T myList[CALENDAR_NULL];

    typedef struct MyWidget_T
    {
        lv_obj_t *calendar_list;
        lv_obj_t *widget_list;
        lv_obj_t *image;
        lv_obj_t *tilview;
    } MyWidget_T;

    typedef enum
    {
        widget_index_weather_for_applist,
        widget_index_calculator_for_applist,
        widget_index_calendar_for_applist,
        widget_index_chatroom_for_applist,
        widget_index_alarm_for_applist,
        widget_index_flashlight_for_applist,
        widget_index_health_for_applist,
        widget_index_setting_for_applist,
        WIDGET_NULL,
    } widget_index_t;

    typedef enum
    {
        widget_id_tools,
        widget_id_media,
        widget_id_recorder,
        widget_id_calendar,
        widget_id_weather,
        widget_id_activity,
        widget_id_list,
    } widget_id_t;

    typedef enum
    {
        app_message_list,
        calendar_message_list,
        MESSAGE_NULL,
    } message_index_t;

    typedef enum
    {
        Today,
        Tomorrow,
        The_day_after_tomorrow,
        In_three_days,
        In_four_days,
        In_five_days,
        In_six_days,
    } one_week_index_t;

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
