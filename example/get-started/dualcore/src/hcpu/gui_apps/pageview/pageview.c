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
#include "sensor.h"
#include "hr_service.h"
#include "ui_datasrv_subscriber.h"

#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#endif

#ifdef APP_ID_PAGEVIEW

LV_IMG_DECLARE(common_background_blue);
LV_IMG_DECLARE(img_passbook);

#define MAX_PAGE 10
#define ACTION_PREV 0
#define ACTION_NEXT 1

static lv_obj_t *tileview;
static uint8_t current_page = 0;
static lv_obj_t *page_indicator_bar; // Added page indicator bar

static bool animation_in_progress = false;

static uint8_t _motion_control_value = 0;
uint8_t get_motion_control_value(void)
{
    return _motion_control_value;
}

static void update_page_indicator(uint8_t page)
{
    int value = page * 100 / (MAX_PAGE - 1); // Calculate the value based on the current page
    // Update the page indicator bar value
    lv_bar_set_value(page_indicator_bar, value, LV_ANIM_ON);
    // 0 ~ 100
    _motion_control_value = value; // Update the motion control value
}

static void animate_to_page(uint8_t page_index)
{
    lv_obj_set_tile_id(tileview, 0, page_index, LV_ANIM_ON);
    update_page_indicator(page_index); // Update page indicator
}

static void animate_to_next_page(void)
{
    if (current_page < MAX_PAGE - 1)
    {
        current_page = current_page + 1;
        animate_to_page(current_page);
    }
}

static void animate_to_prev_page(void)
{
    if (current_page > 0)
    {
        current_page = current_page - 1;
        animate_to_page(current_page);
    }
}

#ifdef BSP_USING_UI_HANDLER
static void handle_remote_action(void *param)
{
    uint8_t action = *(uint8_t *)param;
    if (action == ACTION_PREV)
    {
        animate_to_prev_page();
    }
    else if (action == ACTION_NEXT)
    {
        animate_to_next_page();
    }
}
#endif

static void tileview_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_VALUE_CHANGED)
    {
        rt_uint16_t active_pos = (rt_uint16_t)lv_event_get_param(e);
        rt_kprintf("Tileview active tile: %d\n", active_pos);
        current_page = active_pos;         // Update current_page to match active position
        update_page_indicator(active_pos); // Update page indicator when changed via swipe
    }
}

static void next_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        animate_to_next_page();
    }
}

static void prev_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        animate_to_prev_page();
    }
}

void create_page_with_index(lv_obj_t *parent, uint8_t index)
{
    lv_obj_t *label;
    lv_obj_t *img;
    lv_obj_t *prev_btn;
    lv_obj_t *next_btn;

    img = lv_img_create(parent);
    lv_img_set_src(img, LV_EXT_IMG_GET(common_background_blue));
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    prev_btn = lv_btn_create(parent);
    lv_obj_align(prev_btn, LV_ALIGN_LEFT_MID, 0, 0);
    label = lv_label_create(prev_btn);
    lv_label_set_text(label, "Prev");
    lv_obj_add_event_cb(prev_btn, prev_btn_event_cb, LV_EVENT_CLICKED, NULL);

    next_btn = lv_btn_create(parent);
    lv_obj_align(next_btn, LV_ALIGN_RIGHT_MID, 0, 0);
    label = lv_label_create(next_btn);
    lv_label_set_text(label, "Next");
    lv_obj_add_event_cb(next_btn, next_btn_event_cb, LV_EVENT_CLICKED, NULL);

    // Add index text in the center of the page
    lv_obj_t *index_label = lv_label_create(parent);
    lv_label_set_text_fmt(index_label, "%d", index);
    lv_obj_set_style_text_font(index_label, LV_EXT_FONT_GET(get_system_font_size(2)), 0);
    lv_obj_set_style_text_color(index_label, lv_color_hex(0xffffff), 0);
    lv_obj_align(index_label, LV_ALIGN_CENTER, 0, 0);
}

void pageview_init(void)
{
    rt_uint16_t i;

    lv_obj_t *pages[MAX_PAGE]; // Update the array size to hold 10 pages

    tileview = lv_tileview_create(lv_scr_act());

    // Create page indicator bar
    page_indicator_bar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(page_indicator_bar, 200, 10);
    lv_obj_align(page_indicator_bar, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_bar_set_range(page_indicator_bar, 0, 100);
    lv_bar_set_value(page_indicator_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_radius(page_indicator_bar, 5, LV_PART_MAIN);

    // Make indicator bar stay on top
    lv_obj_move_foreground(page_indicator_bar);

    for (i = 0; i < MAX_PAGE; i++) // Change the loop condition to iterate 10 times
    {
        if (i == 0)
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_BOTTOM);
        }
        else if (i == 9)
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_TOP);
        }
        else
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_VER);
        }
        create_page_with_index(pages[i], i); // Use create_page_with_index function to create the page
    }
    lv_obj_add_event_cb(tileview, tileview_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_tile_id(tileview, 0, current_page, LV_ANIM_OFF);
    update_page_indicator(current_page); // Initialize page indicator
}

// extern void set_lcd_rotate_angle(int angle);

static void on_start(void)
{
    // set_lcd_rotate_angle(90); // Set the LCD rotation angle to 0 degrees
    pageview_init();
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_pageview_action = handle_remote_action;
#endif
}

static void on_pause(void)
{
}

static void on_resume(void)
{
}

static void on_stop(void)
{
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_pageview_action = NULL;
#endif
    // set_lcd_rotate_angle(0);
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
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
    gui_app_regist_msg_handler(APP_ID_PAGEVIEW, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(info_list), LV_EXT_IMG_GET(img_passbook), APP_ID_PAGEVIEW, app_main);
#endif