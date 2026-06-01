#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "ui_datasrv_subscriber.h"
#include "power_manager_service.h"
#include "custom_trans_anim.h"
#include "common_widget.h"
#ifdef BSP_USING_BLOC_SETTING
#include "bloc_setting.h"
#endif
#include "ui_handler.h"
#define DBG_TAG "app.setting.display"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_SETTING
LV_IMG_DECLARE(sun);
LV_IMG_DECLARE(dn);
typedef struct
{
    lv_obj_t *brightness;
    lv_obj_t *rotate;
    datac_handle_t pwr_srv_hdl;
} display_ctx_t;

static display_ctx_t *p_display = NULL;
static int powermgr_srv_callback(data_callback_arg_t *arg);

/**
 * @brief Back button event handler
 *
 * @param e Event data
 */
static void back_btn_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();
    }
}

/**
 * @brief Animation callback for screen rotation
 *
 * @param obj Target object
 * @param v Current value
 */
static void rotate_scr_anim(void *obj, int32_t v)
{
    lv_obj_invalidate(lv_scr_act());
}

/**
 * @brief Updates the brightness slider when brightness changes externally
 *
 * @param param Brightness value pointer
 */
static void handle_amoled_brightness_change(void *param)
{
    uint8_t brightness = *(uint8_t *)param;
    LOG_D("handle_amoled_brightness_change :%d", brightness);
    lv_slider_set_value(p_display->brightness, brightness, LV_ANIM_ON);
}

/**
 * @brief Event callback for settings controls
 *
 * @param e Event data
 */
static void event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_VALUE_CHANGED)
    {
        if (p_display->brightness == obj)
        {
            uint16_t v = (uint16_t)lv_slider_get_value(obj);
            LOG_D("brightness:%d", v);
            datac_send_data(p_display->pwr_srv_hdl,
                            PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ,
                            (uint8_t *)&v, sizeof(uint16_t));
        }
        else if (p_display->rotate == obj)
        {
            uint16_t v = (lv_obj_get_state(obj) & LV_STATE_CHECKED) ? 1 : 0;

            datac_send_data(p_display->pwr_srv_hdl,
                            PWRMGR_MSG_LCD_ROTATE_180_SET_REQ,
                            (uint8_t *)&v, sizeof(v));

            // Apply rotation directly to display
            lv_disp_set_rotation(lv_disp_get_default(),
                                 v ? LV_DISP_ROT_180 : LV_DISP_ROT_NONE);

            // Create animation for visual feedback
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, p_display->rotate);
            lv_anim_set_values(&a, 0, 180);
            lv_anim_set_exec_cb(&a, rotate_scr_anim);
            lv_anim_set_time(&a, 100);
            lv_anim_start(&a);
        }
    }
}

static void bar_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSING)
    {
        lv_obj_t *bar = lv_event_get_target(e);

        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);

        lv_coord_t min = lv_bar_get_min_value(bar);
        lv_coord_t max = lv_bar_get_max_value(bar);

        lv_coord_t w = lv_obj_get_width(bar);
        lv_coord_t rel_x = p.x - lv_obj_get_x(bar);
        if (rel_x < 0)
            rel_x = 0;
        if (rel_x > w)
            rel_x = w;

        lv_coord_t value = (rel_x * (max - min)) / w + min;

        uint16_t result = lv_bar_get_value(bar);
        if (p_display->brightness == obj)
        {
            if (value < 10)
                value = 10;

            lv_bar_set_value(bar, value, LV_ANIM_OFF);
            datac_send_data(p_display->pwr_srv_hdl,
                            PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ,
                            (uint8_t *)&result, sizeof(uint16_t));
        }
    }
}

/**
 * @brief Callback function for power manager service
 *
 * @param arg Callback arguments
 * @return int Result code
 */
static int powermgr_srv_callback(data_callback_arg_t *arg)
{
    if (!p_display && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (p_display)
        {
            if (rsp->result >= 0)
            {
                data_msg_t msg;

                data_service_init_msg(&msg, PWRMGR_MSG_LCD_BRIGHTNESS_GET_REQ, 0);
                datac_send_msg(p_display->pwr_srv_hdl, &msg);

                data_service_init_msg(&msg, PWRMGR_MSG_LCD_ROTATE_180_GET_REQ, 0);
                datac_send_msg(p_display->pwr_srv_hdl, &msg);
            }
        }
    }
    break;

    case PWRMGR_MSG_LCD_BRIGHTNESS_GET_RSP:
    {
        range_msg_t *p_range;
        p_range = (range_msg_t *)arg->data;

        LOG_D("PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP cur=%d[%d,%d]",
              p_range->cur, p_range->min, p_range->max);
        lv_bar_set_range(p_display->brightness, p_range->min, p_range->max);
        lv_bar_set_value(p_display->brightness, p_range->cur, LV_ANIM_ON);
    }
    break;
    case PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP:
    {
        range_msg_t *p_range;
        p_range = (range_msg_t *)arg->data;
        LOG_D("PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP cur=%d[%d,%d]",
              p_range->cur, p_range->min, p_range->max);
#ifdef BSP_USING_BLOC_CONTROL
        control_provider.screen_brightness_smoothly(p_range->cur);
#endif
    }
    break;

    case PWRMGR_MSG_LCD_ROTATE_180_GET_RSP:
    case PWRMGR_MSG_LCD_ROTATE_180_SET_RSP:
    {
        uint16_t r = *((uint16_t *)arg->data);

        LOG_D("PWRMGR_MSG_LCD_ROTATE_180_GET/SET_RSP %d", r);

        if (r)
            lv_obj_add_state(p_display->rotate, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(p_display->rotate, LV_STATE_CHECKED);
    }
    break;

    default:
        break;
    }

    return 0;
}

/**
 * @brief Initialize the display settings UI
 */
static void on_start(void)
{
    // Main container
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    // Dark theme background for circular watch face
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x121212), 0);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);
    lv_obj_update_layout(cont);

    // Create a container for settings groups - adjusted for circular display
    lv_obj_t *settings_container = lv_obj_create(cont);
    lv_obj_set_size(settings_container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_opa(settings_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(settings_container, 0, 0);
    lv_obj_set_style_pad_all(settings_container, 0, 0);
    lv_obj_align(settings_container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_pad_bottom(settings_container, 100, 0);

    lv_obj_t *cont_title = lv_label_create(settings_container);
    lv_label_set_text(cont_title, LV_EXT_STR_GET_BY_KEY(setting_display, "Display"));
    lv_obj_set_style_text_color(cont_title, lv_color_hex(0xFFFFFF), 0); // White text for dark theme
    lv_obj_set_style_text_font(cont_title, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_align(cont_title, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_update_layout(cont_title);

    // Create general settings group
    lv_obj_t *general_group = lv_obj_create(settings_container);
    lv_obj_set_size(general_group, LV_PCT(90), LV_SIZE_CONTENT);
    lv_obj_set_style_radius(general_group, 15, 0);                       // More rounded corners
    lv_obj_set_style_bg_color(general_group, lv_color_hex(0x1E1E1E), 0); // Dark gray for items
    lv_obj_set_style_pad_row(general_group, 5, 0);                       // More padding between rows
    lv_obj_set_style_pad_top(general_group, 5, 0);
    lv_obj_set_style_pad_bottom(general_group, 5, 0);
    lv_obj_set_flex_flow(general_group, LV_FLEX_FLOW_COLUMN);
    lv_obj_align_to(general_group, cont_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    // Brightness control
    lv_obj_t *label = lv_label_create(general_group);
    lv_label_set_text(label, "Brightness");
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *item = lv_obj_create(general_group);
    lv_obj_set_size(item, LV_PCT(100), 100);                    // Larger height for better touch targets
    lv_obj_set_style_bg_color(item, lv_color_hex(0x1E1E1E), 0); // Dark gray
    lv_obj_t *bar = lv_bar_create(item);                        // Create a progress bar
    lv_bar_set_range(bar, 10, 100);                             // Set the range of the progress bar
    lv_obj_set_width(bar, LV_PCT(70));
    lv_obj_set_height(bar, 80);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xE5E5EA), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xE5E5EA), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_10, LV_PART_MAIN);
    lv_bar_set_value(bar, SkaiWatchSys.brightness, LV_ANIM_ON);
    lv_obj_add_event_cb(bar, bar_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_t *icon = lv_img_create(bar);
    lv_img_set_src(icon, &sun);
    lv_obj_align(icon, LV_ALIGN_LEFT_MID, 20, 0);
    p_display->brightness = bar;

    // Screen rotation (icon + name + switch)
    p_display->rotate = create_dark_toggle_item(general_group, &dn,
                                                "Rotate Display",
                                                false,
                                                get_system_font_size(0),
                                                false);
    lv_obj_add_event_cb(p_display->rotate, event_cb, LV_EVENT_ALL, 0);

    // Get data from service
    p_display->pwr_srv_hdl = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_display->pwr_srv_hdl);
    ui_datac_subscribe(p_display->pwr_srv_hdl, "powermgr", powermgr_srv_callback, 0);

    // Configure animations
    lv_point_t p = {0, 233};
    cust_trans_anim_config(CUST_ANIM_TYPE_3, &p);
}

/**
 * @brief Resume the display settings page
 */
static void on_resume(void)
{
    // Nothing to do on resume
}

/**
 * @brief Pause the display settings page
 */
static void on_pause(void)
{
    // Nothing to do on pause
}

/**
 * @brief Clean up resources when stopping the page
 */
static void on_stop(void)
{
    if (p_display)
    {
        if (DATA_CLIENT_INVALID_HANDLE != p_display->pwr_srv_hdl)
        {
            datac_close(p_display->pwr_srv_hdl);
            p_display->pwr_srv_hdl = DATA_CLIENT_INVALID_HANDLE;
        }

        /* The page screen and every widget on it (incl. brightness/rotate)
           are cascade-deleted by the app framework on stop — deleting these
           children here is redundant and risks touching framework-managed
           objects. Only the heap context is ours to free. */
        lv_mem_free(p_display);
        p_display = NULL;
    }
}

/**
 * @brief Message handler for display settings page lifecycle
 *
 * @param msg Message type
 * @param param Message parameters
 */
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

/**
 * @brief Create and display the settings page
 */
void app_setting_display_main(void)
{
    RT_ASSERT(NULL == p_display);
    p_display = lv_mem_alloc(sizeof(display_ctx_t));
    LV_ASSERT_MALLOC(p_display);
    p_display->pwr_srv_hdl = DATA_CLIENT_INVALID_HANDLE;

    gui_app_create_page("display", msg_handler);
}

#endif /* APP_ID_SETTING */