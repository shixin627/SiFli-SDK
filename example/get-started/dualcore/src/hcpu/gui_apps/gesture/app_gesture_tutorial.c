/**
 ******************************************************************************
 * @file   app_gesture_tutorial.c
 * @author Skaiwalk software development team
 * @brief  Interactive tutorial for learning the release gesture
 ******************************************************************************
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
#include "app_mainmenu.h"
#include "common_widget.h"
#include "gesture_recognition_task.h"
#include "bloc_motor.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#ifdef BSP_USING_GESTURE_HANDLER
#include "gesture_handler.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#endif

#define DBG_TAG "app.gesture.tutorial"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_GESTURE_TUTORIAL

LV_IMG_DECLARE(icon_release);

// Tutorial constants
#define TARGET_PRACTICE_COUNT 10
#define PERFECT_THRESHOLD 90
#define RETRY_THRESHOLD 50
#define FEEDBACK_DISPLAY_TIME 1000 // 1 second

// Tutorial state
typedef enum
{
    TUTORIAL_STATE_INTRO,    // Introduction screen
    TUTORIAL_STATE_PRACTICE, // Practice mode
    TUTORIAL_STATE_COMPLETE  // Completion screen
} tutorial_state_t;

// Tutorial data structure
typedef struct
{
    lv_obj_t *container;
    lv_obj_t *title_label;
    lv_obj_t *instruction_label;
    lv_obj_t *count_label;
    lv_obj_t *confidence_label;
    lv_obj_t *feedback_label;
    lv_obj_t *hand_icon;
    lv_obj_t *progress_bar;
    lv_timer_t *feedback_timer;
    lv_timer_t *gesture_poll_timer;
    lv_anim_t hand_anim;

    tutorial_state_t state;
    int remaining_count;
    int last_confidence;
    int last_score;
    bool waiting_for_gesture;
    bool gesture_in_progress;
} gesture_tutorial_t;

static gesture_tutorial_t *tutorial = NULL;

// Forward declarations
static void update_ui_for_state(void);
static void hide_feedback(lv_timer_t *timer);
static void show_feedback(const char *text, lv_color_t color);
static void handle_gesture_detected(int confidence);
static void create_hand_animation(void);
static void gesture_poll_callback(lv_timer_t *timer);

// Feedback timer callback
static void hide_feedback(lv_timer_t *timer)
{
    if (tutorial && tutorial->feedback_label)
    {
        lv_obj_add_flag(tutorial->feedback_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (tutorial && tutorial->feedback_timer)
    {
        lv_timer_del(tutorial->feedback_timer);
        tutorial->feedback_timer = NULL;
    }
}

// Show feedback message with auto-hide
static void show_feedback(const char *text, lv_color_t color)
{
    if (!tutorial || !tutorial->feedback_label)
        return;

    lv_label_set_text(tutorial->feedback_label, text);
    lv_obj_set_style_text_color(tutorial->feedback_label, color, 0);
    lv_obj_clear_flag(tutorial->feedback_label, LV_OBJ_FLAG_HIDDEN);

    // Cancel existing timer if any
    if (tutorial->feedback_timer)
    {
        lv_timer_del(tutorial->feedback_timer);
    }

    // Create new timer to hide feedback after 1 second
    tutorial->feedback_timer = lv_timer_create(hide_feedback, FEEDBACK_DISPLAY_TIME, NULL);
    lv_timer_set_repeat_count(tutorial->feedback_timer, 1);
}

// Handle release gesture detection
static void handle_gesture_detected(int confidence)
{
    if (!tutorial || tutorial->state != TUTORIAL_STATE_PRACTICE)
        return;

    if (!tutorial->waiting_for_gesture)
        return;

    LOG_I("Gesture detected with confidence: %d", confidence);

    tutorial->last_confidence = confidence;
    tutorial->waiting_for_gesture = false;

    // Update confidence display
    lv_label_set_text_fmt(tutorial->confidence_label, "Confidence: %d", confidence);

    // Provide feedback based on confidence
    if (confidence >= PERFECT_THRESHOLD)
    {
        show_feedback("PERFECT!", lv_color_hex(0x00FF00));
        extern void motor_pattern_scrolling_app(void);
        motor_pattern_scrolling_app();

        // Decrement counter
        tutorial->remaining_count--;
        lv_label_set_text_fmt(tutorial->count_label, "%d", tutorial->remaining_count);

        // Update progress bar
        int progress = ((TARGET_PRACTICE_COUNT - tutorial->remaining_count) * 100) / TARGET_PRACTICE_COUNT;
        lv_bar_set_value(tutorial->progress_bar, progress, LV_ANIM_ON);

        // Check if completed
        if (tutorial->remaining_count <= 0)
        {
            tutorial->state = TUTORIAL_STATE_COMPLETE;
            update_ui_for_state();
        }
        else
        {
            // Ready for next attempt after feedback disappears
            tutorial->waiting_for_gesture = true;
        }
    }
    else if (confidence < RETRY_THRESHOLD)
    {
        show_feedback("Try Again!", lv_color_hex(0xFF9900));
        tutorial->waiting_for_gesture = true;
    }
    else
    {
        show_feedback("Good!", lv_color_hex(0x00BFFF));

        tutorial->remaining_count--;
        lv_label_set_text_fmt(tutorial->count_label, "%d", tutorial->remaining_count);

        int progress = ((TARGET_PRACTICE_COUNT - tutorial->remaining_count) * 100) / TARGET_PRACTICE_COUNT;
        lv_bar_set_value(tutorial->progress_bar, progress, LV_ANIM_ON);

        if (tutorial->remaining_count <= 0)
        {
            tutorial->state = TUTORIAL_STATE_COMPLETE;
            update_ui_for_state();
        }
        else
        {
            tutorial->waiting_for_gesture = true;
        }
    }
}

// Create pulsing hand animation
static void hand_anim_cb(void *var, int32_t v)
{
    lv_obj_set_style_transform_zoom(var, v, 0);
}

static void create_hand_animation(void)
{
    if (!tutorial || !tutorial->hand_icon)
        return;

    lv_anim_init(&tutorial->hand_anim);
    lv_anim_set_var(&tutorial->hand_anim, tutorial->hand_icon);
    lv_anim_set_values(&tutorial->hand_anim, 256, 300); // 100% to 117% scale
    lv_anim_set_time(&tutorial->hand_anim, 800);
    lv_anim_set_playback_time(&tutorial->hand_anim, 800);
    lv_anim_set_repeat_count(&tutorial->hand_anim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&tutorial->hand_anim, hand_anim_cb);
    lv_anim_start(&tutorial->hand_anim);
}

// Start button callback
static void start_button_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED && tutorial)
    {
        tutorial->state = TUTORIAL_STATE_PRACTICE;
        tutorial->remaining_count = TARGET_PRACTICE_COUNT;
        tutorial->waiting_for_gesture = true;
        update_ui_for_state();
    }
}

// Exit button callback
static void exit_button_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED)
    {
        gui_app_self_exit();
    }
}

// Restart button callback
static void restart_button_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED && tutorial)
    {
        tutorial->state = TUTORIAL_STATE_PRACTICE;
        tutorial->remaining_count = TARGET_PRACTICE_COUNT;
        tutorial->waiting_for_gesture = true;
        update_ui_for_state();
    }
}

// Create intro screen
static void create_intro_screen(lv_obj_t *parent)
{
    // Title
    tutorial->title_label = lv_label_create(parent);
    lv_label_set_text(tutorial->title_label, "Release Gesture\nTutorial");
    lv_obj_set_style_text_font(tutorial->title_label, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(tutorial->title_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(tutorial->title_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(tutorial->title_label, LV_ALIGN_TOP_MID, 0, 40);

    // Instruction
    tutorial->instruction_label = lv_label_create(parent);
    lv_label_set_text(tutorial->instruction_label,
                      "Learn the release gesture:\n\n"
                      "1. Make a fist\n"
                      "2. Quickly open all fingers\n\n"
                      "Practice 10 times!");
    lv_obj_set_style_text_font(tutorial->instruction_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(tutorial->instruction_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_align(tutorial->instruction_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(tutorial->instruction_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(tutorial->instruction_label, 400);
    lv_obj_align(tutorial->instruction_label, LV_ALIGN_CENTER, 0, -20);

    // Start button
    lv_obj_t *start_btn = common_text_button(parent, "Start", NULL, 150, 50, start_button_cb);
    lv_obj_align(start_btn, LV_ALIGN_BOTTOM_MID, 0, -60);

    // Exit button
    lv_obj_t *exit_btn = common_text_button(parent, "Exit", NULL, 100, 50, exit_button_cb);
    lv_obj_align(exit_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
}

// Create practice screen
static void create_practice_screen(lv_obj_t *parent)
{
    // Remaining count (large, centered)
    tutorial->count_label = lv_label_create(parent);
    lv_label_set_text_fmt(tutorial->count_label, "%d", tutorial->remaining_count);
    lv_obj_set_style_text_font(tutorial->count_label, LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_set_style_text_color(tutorial->count_label, lv_color_hex(0x00FF00), 0);
    lv_obj_align(tutorial->count_label, LV_ALIGN_CENTER, 0, -60);

    // Instruction
    tutorial->instruction_label = lv_label_create(parent);
    lv_label_set_text(tutorial->instruction_label, "Make a fist,\nthen open quickly!");
    lv_obj_set_style_text_font(tutorial->instruction_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(tutorial->instruction_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(tutorial->instruction_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(tutorial->instruction_label, LV_ALIGN_TOP_MID, 0, 60);

    // Hand icon (using icon_release image)
    tutorial->hand_icon = lv_img_create(parent);
    lv_img_set_src(tutorial->hand_icon, LV_EXT_IMG_GET(icon_release));
    lv_obj_align(tutorial->hand_icon, LV_ALIGN_CENTER, 0, 20);
    create_hand_animation();

    // Confidence display
    tutorial->confidence_label = lv_label_create(parent);
    lv_label_set_text(tutorial->confidence_label, "Confidence: --");
    lv_obj_set_style_text_font(tutorial->confidence_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(tutorial->confidence_label, lv_color_hex(0x888888), 0);
    lv_obj_align(tutorial->confidence_label, LV_ALIGN_BOTTOM_MID, 0, -100);

    // Progress bar
    tutorial->progress_bar = lv_bar_create(parent);
    lv_obj_set_size(tutorial->progress_bar, 350, 15);
    lv_bar_set_range(tutorial->progress_bar, 0, 100);
    lv_bar_set_value(tutorial->progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(tutorial->progress_bar, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(tutorial->progress_bar, lv_color_hex(0x00FF00), LV_PART_INDICATOR);
    lv_obj_align(tutorial->progress_bar, LV_ALIGN_BOTTOM_MID, 0, -70);

    // Feedback label (initially hidden)
    tutorial->feedback_label = lv_label_create(parent);
    lv_label_set_text(tutorial->feedback_label, "");
    lv_obj_set_style_text_font(tutorial->feedback_label, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_align(tutorial->feedback_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(tutorial->feedback_label, LV_ALIGN_CENTER, 0, 100);
    lv_obj_add_flag(tutorial->feedback_label, LV_OBJ_FLAG_HIDDEN);

    // Exit button (small, bottom right)
    lv_obj_t *exit_btn = common_text_button(parent, "Exit", NULL, 80, 40, exit_button_cb);
    lv_obj_align(exit_btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);

    // Start gesture polling timer (poll every 100ms)
    if (tutorial->gesture_poll_timer)
    {
        lv_timer_del(tutorial->gesture_poll_timer);
    }
    tutorial->gesture_poll_timer = lv_timer_create(gesture_poll_callback, 100, NULL);
}

// Create completion screen
static void create_completion_screen(lv_obj_t *parent)
{
    // Celebration title
    tutorial->title_label = lv_label_create(parent);
    lv_label_set_text(tutorial->title_label, "Congratulations!");
    lv_obj_set_style_text_font(tutorial->title_label, LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_obj_set_style_text_color(tutorial->title_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_align(tutorial->title_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(tutorial->title_label, LV_ALIGN_CENTER, 0, -80);

    // Message
    tutorial->instruction_label = lv_label_create(parent);
    lv_label_set_text(tutorial->instruction_label,
                      "You've mastered the\nrelease gesture!\n\n"
                      "You can now use it to\ninteract with your watch.");
    lv_obj_set_style_text_font(tutorial->instruction_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(tutorial->instruction_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(tutorial->instruction_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(tutorial->instruction_label, LV_ALIGN_CENTER, 0, 0);

    // Restart button
    lv_obj_t *restart_btn = common_text_button(parent, "Practice Again", NULL, 180, 50, restart_button_cb);
    lv_obj_align(restart_btn, LV_ALIGN_BOTTOM_MID, 0, -60);

    // Exit button
    lv_obj_t *exit_btn = common_text_button(parent, "Done", NULL, 100, 50, exit_button_cb);
    lv_obj_align(exit_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
}

// Update UI based on current state
static void update_ui_for_state(void)
{
    if (!tutorial || !tutorial->container)
        return;

    // Stop gesture polling timer if leaving practice state
    if (tutorial->state != TUTORIAL_STATE_PRACTICE && tutorial->gesture_poll_timer)
    {
        lv_timer_del(tutorial->gesture_poll_timer);
        tutorial->gesture_poll_timer = NULL;
    }

    // Clear container
    lv_obj_clean(tutorial->container);

    // Reset pointers
    tutorial->title_label = NULL;
    tutorial->instruction_label = NULL;
    tutorial->count_label = NULL;
    tutorial->confidence_label = NULL;
    tutorial->feedback_label = NULL;
    tutorial->hand_icon = NULL;
    tutorial->progress_bar = NULL;

    // Create appropriate screen
    switch (tutorial->state)
    {
    case TUTORIAL_STATE_INTRO:
        create_intro_screen(tutorial->container);
        break;
    case TUTORIAL_STATE_PRACTICE:
        create_practice_screen(tutorial->container);
        break;
    case TUTORIAL_STATE_COMPLETE:
        create_completion_screen(tutorial->container);
        break;
    }
}

// Gesture polling timer callback
static void gesture_poll_callback(lv_timer_t *timer)
{
    if (!tutorial || tutorial->state != TUTORIAL_STATE_PRACTICE)
        return;

    if (!tutorial->waiting_for_gesture || tutorial->gesture_in_progress)
        return;

    // Get current confidence score
    int current_score = release_recognition_score;

    // Check if this is a new gesture (score changed significantly and exceeds threshold)
    if (current_score > get_release_recognition_threshold() &&
        current_score != tutorial->last_score)
    {
        tutorial->gesture_in_progress = true;
        tutorial->last_score = current_score;

        handle_gesture_detected(current_score);

        // Reset after short delay to be ready for next gesture
        rt_thread_mdelay(500);
        tutorial->gesture_in_progress = false;
        tutorial->last_score = 0;
    }
}

// Initialize tutorial
static lv_obj_t *on_start(lv_obj_t *parent)
{
    if (tutorial != NULL)
    {
        return tutorial->container;
    }

    tutorial = (gesture_tutorial_t *)lv_mem_alloc(sizeof(gesture_tutorial_t));
    if (!tutorial)
    {
        LOG_E("Failed to allocate memory for tutorial");
        return NULL;
    }
    memset(tutorial, 0, sizeof(gesture_tutorial_t));

    // Create container
    tutorial->container = lv_obj_create(parent);
    lv_obj_set_size(tutorial->container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(tutorial->container, lv_color_hex(0x121212), 0);
    lv_obj_set_style_border_width(tutorial->container, 0, 0);
    lv_obj_align(tutorial->container, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(tutorial->container, LV_OBJ_FLAG_SCROLLABLE);

    // Initialize state
    tutorial->state = TUTORIAL_STATE_INTRO;
    tutorial->remaining_count = TARGET_PRACTICE_COUNT;
    tutorial->waiting_for_gesture = false;

    // Create intro screen
    update_ui_for_state();

    return tutorial->container;
}

static void on_resume(void)
{
    // Resume gesture polling if in practice state
    if (tutorial && tutorial->state == TUTORIAL_STATE_PRACTICE)
    {
        if (tutorial->gesture_poll_timer)
        {
            lv_timer_resume(tutorial->gesture_poll_timer);
        }
    }
}

static void on_pause(void)
{
    // Pause timers
    if (tutorial)
    {
        if (tutorial->feedback_timer)
        {
            lv_timer_del(tutorial->feedback_timer);
            tutorial->feedback_timer = NULL;
        }
        if (tutorial->gesture_poll_timer)
        {
            lv_timer_pause(tutorial->gesture_poll_timer);
        }
    }
}

static void on_stop(void)
{
    if (tutorial)
    {
        if (tutorial->feedback_timer)
        {
            lv_timer_del(tutorial->feedback_timer);
            tutorial->feedback_timer = NULL;
        }
        if (tutorial->gesture_poll_timer)
        {
            lv_timer_del(tutorial->gesture_poll_timer);
            tutorial->gesture_poll_timer = NULL;
        }
        lv_mem_free(tutorial);
        tutorial = NULL;
    }
}

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

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_GESTURE_TUTORIAL, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(gesture_tutorial), LV_EXT_IMG_GET(icon_release),
                   APP_ID_GESTURE_TUTORIAL, app_main);

#endif // APP_ID_GESTURE_TUTORIAL
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
