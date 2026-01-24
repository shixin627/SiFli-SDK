#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "app_mainmenu.h"
#include "app_clock_main.h"
#include "app_mem.h"
#include "common_widget.h"
#include "watch_system_interact.h"
#include "ui_helper.h"

#define DBG_TAG "app.clock.digital_elegant"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *time_container;
    lv_obj_t *hour_label;
    lv_obj_t *colon_label;
    lv_obj_t *minute_label;
    lv_obj_t *second_label;
    lv_obj_t *date_container;
    lv_obj_t *date_label;
    lv_obj_t *separator_line;
    lv_timer_t *redraw_task;
    app_clock_time_t last_redraw_time;
    uint8_t colon_blink_state;
} app_clock_digital_elegant_t;

static app_clock_digital_elegant_t *p_clk_digital = NULL;

static void app_clock_digital_elegant_redraw(lv_timer_t *task)
{
    app_clock_time_t current_time;
    rt_uint8_t hours, minutes, seconds;
    static char time_buf[8];
    static char date_buf[32];
    static const char *weekday_names[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    static const char *month_names[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    app_clock_main_get_current_time(&current_time);
    hours = current_time.h;
    minutes = current_time.m;
    seconds = current_time.s;

    // Update hour display
    snprintf(time_buf, sizeof(time_buf), "%02d", hours);
    lv_label_set_text(p_clk_digital->hour_label, time_buf);

    // Update minute display
    snprintf(time_buf, sizeof(time_buf), "%02d", minutes);
    lv_label_set_text(p_clk_digital->minute_label, time_buf);

    // Update second display with smooth fade effect
    snprintf(time_buf, sizeof(time_buf), "%02d", seconds);
    lv_label_set_text(p_clk_digital->second_label, time_buf);

    // Blinking colon effect (blink every second)
    if (seconds != p_clk_digital->last_redraw_time.s)
    {
        p_clk_digital->colon_blink_state = !p_clk_digital->colon_blink_state;
        if (p_clk_digital->colon_blink_state)
        {
            lv_obj_set_style_text_opa(p_clk_digital->colon_label, LV_OPA_100, 0);
        }
        else
        {
            lv_obj_set_style_text_opa(p_clk_digital->colon_label, LV_OPA_30, 0);
        }
    }

    // Update date and weekday (only when day changes)
    if (p_clk_digital->last_redraw_time.day != current_time.day)
    {
        // Update date with weekday in one line (e.g., "Mon, Jan 15, 2024")
        snprintf(date_buf, sizeof(date_buf), "%s, %s %d, %d",
                 weekday_names[SkaiWatchSys.Global_Time.weekday % 7],
                 month_names[(SkaiWatchSys.Global_Time.month - 1) % 12],
                 SkaiWatchSys.Global_Time.day,
                 SkaiWatchSys.Global_Time.year);
        lv_label_set_text(p_clk_digital->date_label, date_buf);
    }

    memcpy(&p_clk_digital->last_redraw_time, &current_time, sizeof(app_clock_time_t));
}

static rt_int32_t resume_callback(void)
{
    if (NULL == p_clk_digital->redraw_task)
    {
        p_clk_digital->redraw_task = lv_timer_create(app_clock_digital_elegant_redraw, 30, (void *)0);
        app_clock_digital_elegant_redraw(NULL);
    }

    return RT_EOK;
}

static rt_int32_t pause_callback(void)
{
    if (p_clk_digital->redraw_task)
    {
        lv_timer_del(p_clk_digital->redraw_task);
        p_clk_digital->redraw_task = NULL;
    }

    return RT_EOK;
}

lv_obj_t *lv_digital_elegant_layout_create(lv_obj_t *parent)
{
    p_clk_digital = (app_clock_digital_elegant_t *)rt_malloc(sizeof(app_clock_digital_elegant_t));
    memset(p_clk_digital, 0, sizeof(app_clock_digital_elegant_t));

    // Create background with subtle gradient
    p_clk_digital->bg = lv_obj_create(parent);
    lv_obj_set_size(p_clk_digital->bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(p_clk_digital->bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(p_clk_digital->bg, lv_color_make(10, 10, 10), 0);
    lv_obj_set_style_bg_grad_color(p_clk_digital->bg, lv_color_make(5, 5, 5), 0);
    lv_obj_set_style_bg_grad_dir(p_clk_digital->bg, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(p_clk_digital->bg, 0, 0);
    lv_obj_set_style_radius(p_clk_digital->bg, 0, 0);
    lv_obj_clear_flag(p_clk_digital->bg, LV_OBJ_FLAG_SCROLLABLE);

    // Create main time container
    p_clk_digital->time_container = lv_obj_create(p_clk_digital->bg);
    lv_obj_set_size(p_clk_digital->time_container, LV_HOR_RES_MAX - 20, 140);
    lv_obj_align(p_clk_digital->time_container, LV_ALIGN_CENTER, 0, -30);
    lv_obj_set_style_bg_color(p_clk_digital->time_container, lv_color_make(25, 25, 25), 0);
    lv_obj_set_style_bg_opa(p_clk_digital->time_container, LV_OPA_50, 0);
    lv_obj_set_style_border_width(p_clk_digital->time_container, 1, 0);
    lv_obj_set_style_border_color(p_clk_digital->time_container, lv_color_make(80, 80, 80), 0);
    lv_obj_set_style_border_opa(p_clk_digital->time_container, LV_OPA_20, 0);
    lv_obj_set_style_radius(p_clk_digital->time_container, 20, 0);
    lv_obj_clear_flag(p_clk_digital->time_container, LV_OBJ_FLAG_SCROLLABLE);

    // Create hour label
    p_clk_digital->hour_label = lv_label_create(p_clk_digital->time_container);
    lv_obj_set_style_text_font(p_clk_digital->hour_label, LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_set_style_text_color(p_clk_digital->hour_label, lv_color_white(), 0);
    lv_label_set_text(p_clk_digital->hour_label, "00");
    lv_obj_align(p_clk_digital->hour_label, LV_ALIGN_CENTER, -70, 0);

    // Create colon label with elegant styling
    p_clk_digital->colon_label = lv_label_create(p_clk_digital->time_container);
    lv_obj_set_style_text_font(p_clk_digital->colon_label, LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_set_style_text_color(p_clk_digital->colon_label, lv_color_make(200, 200, 200), 0);
    lv_label_set_text(p_clk_digital->colon_label, ":");
    lv_obj_align(p_clk_digital->colon_label, LV_ALIGN_CENTER, 0, 0);
    p_clk_digital->colon_blink_state = 1;

    // Create minute label
    p_clk_digital->minute_label = lv_label_create(p_clk_digital->time_container);
    lv_obj_set_style_text_font(p_clk_digital->minute_label, LV_EXT_FONT_GET(get_system_font_size(3)), 0);
    lv_obj_set_style_text_color(p_clk_digital->minute_label, lv_color_white(), 0);
    lv_label_set_text(p_clk_digital->minute_label, "00");
    lv_obj_align(p_clk_digital->minute_label, LV_ALIGN_CENTER, 70, 0);

    // Create second label (smaller, positioned at top right of container)
    p_clk_digital->second_label = lv_label_create(p_clk_digital->time_container);
    lv_obj_set_style_text_font(p_clk_digital->second_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(p_clk_digital->second_label, lv_color_make(140, 140, 140), 0);
    lv_label_set_text(p_clk_digital->second_label, "00");
    lv_obj_align(p_clk_digital->second_label, LV_ALIGN_TOP_RIGHT, -15, 15);

    // Create decorative separator line
    p_clk_digital->separator_line = lv_obj_create(p_clk_digital->bg);
    lv_obj_set_size(p_clk_digital->separator_line, 150, 1);
    lv_obj_align(p_clk_digital->separator_line, LV_ALIGN_CENTER, 0, 50);
    lv_obj_set_style_bg_color(p_clk_digital->separator_line, lv_color_make(100, 100, 100), 0);
    lv_obj_set_style_bg_opa(p_clk_digital->separator_line, LV_OPA_30, 0);
    lv_obj_set_style_border_width(p_clk_digital->separator_line, 0, 0);
    lv_obj_set_style_radius(p_clk_digital->separator_line, 0, 0);

    // Create date container
    p_clk_digital->date_container = lv_obj_create(p_clk_digital->bg);
    lv_obj_set_size(p_clk_digital->date_container, LV_HOR_RES_MAX - 40, 80);
    lv_obj_align(p_clk_digital->date_container, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style_bg_opa(p_clk_digital->date_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(p_clk_digital->date_container, 0, 0);
    lv_obj_clear_flag(p_clk_digital->date_container, LV_OBJ_FLAG_SCROLLABLE);

    // Create date label (with weekday)
    p_clk_digital->date_label = lv_label_create(p_clk_digital->date_container);
    lv_obj_set_style_text_font(p_clk_digital->date_label, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(p_clk_digital->date_label, lv_color_make(160, 160, 160), 0);
    lv_label_set_text(p_clk_digital->date_label, "");
    lv_obj_align(p_clk_digital->date_label, LV_ALIGN_CENTER, 0, 0);

    p_clk_digital->redraw_task = NULL;

    return p_clk_digital->bg;
}

static rt_int32_t init(lv_obj_t *parent)
{
    lv_digital_elegant_layout_create(parent);
    return RT_EOK;
}

static rt_int32_t deinit(void)
{
    if (p_clk_digital)
    {
        if (p_clk_digital->redraw_task)
        {
            lv_timer_del(p_clk_digital->redraw_task);
        }
        rt_free(p_clk_digital);
        p_clk_digital = NULL;
    }

    return RT_EOK;
}

static const app_clock_ops_t ops =
{
    .init = init,
    .pause = pause_callback,
    .resume = resume_callback,
    .deinit = deinit,
};

void app_clock_digital_elegant_register(void)
{
    app_clock_register("digital_elegant", &ops);
}
