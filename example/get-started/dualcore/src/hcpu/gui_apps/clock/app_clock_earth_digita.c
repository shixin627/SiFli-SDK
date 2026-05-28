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
#include "lv_ext_resource_manager.h"

#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(img_earth_digital_bg);
LV_IMG_DECLARE(media_gaus_bg);
LV_IMG_DECLARE(dig_1_00);
LV_IMG_DECLARE(dig_1_01);
LV_IMG_DECLARE(dig_1_02);
LV_IMG_DECLARE(dig_1_03);
LV_IMG_DECLARE(dig_1_04);
LV_IMG_DECLARE(dig_1_05);
LV_IMG_DECLARE(dig_1_06);
LV_IMG_DECLARE(dig_1_07);
LV_IMG_DECLARE(dig_1_08);
LV_IMG_DECLARE(dig_1_09);

typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *hour_0_img;
    lv_obj_t *hour_1_img;
    lv_obj_t *minute_0_img;
    lv_obj_t *minute_1_img;
    lv_timer_t *redraw_task;
    app_clock_time_t last_redraw_time;
    lv_obj_t *earth_img;
    lv_obj_t *ampm_label; /* AM/PM tag, filled in 12h mode, empty in 24h */
} app_clock_earth_digital_t;

static app_clock_earth_digital_t *p_clk_earth_digital = NULL;

static void app_clock_earth_digital_redraw(lv_timer_t *task)
{
    app_clock_time_t current_time;
    rt_uint8_t hours, minutes;
    static char time_buf[8];

    app_clock_main_get_current_time(&current_time);
    hours = current_time.h;
    minutes = current_time.m;

    // Respect the 12/24-hour preference before splitting into digit images.
    rt_uint8_t disp_hours = ui_time_display_hour(hours);

    // Update hour display with images
    int hour_0 = disp_hours / 10;
    int hour_1 = disp_hours % 10;
    int min_0 = minutes / 10;
    int min_1 = minutes % 10;

    // 指標陣列對應數字圖片資源
    const lv_img_dsc_t *dig_img[10] = {
        &dig_1_00, &dig_1_01, &dig_1_02, &dig_1_03, &dig_1_04,
        &dig_1_05, &dig_1_06, &dig_1_07, &dig_1_08, &dig_1_09};

    lv_img_set_src(p_clk_earth_digital->hour_0_img, dig_img[hour_0]);
    lv_img_set_src(p_clk_earth_digital->hour_1_img, dig_img[hour_1]);
    lv_img_set_src(p_clk_earth_digital->minute_0_img, dig_img[min_0]);
    lv_img_set_src(p_clk_earth_digital->minute_1_img, dig_img[min_1]);

    if (lv_obj_is_valid(p_clk_earth_digital->ampm_label))
    {
        const char *ampm = ui_time_ampm(hours);
        lv_label_set_text(p_clk_earth_digital->ampm_label, ampm ? ampm : "");
        /* Re-align here (not just at create): by now the minute image has its
           real size, so OUT_BOTTOM_RIGHT actually lands below the digit. The
           create-time align ran before the image size settled. */
        lv_obj_align_to(p_clk_earth_digital->ampm_label,
                        p_clk_earth_digital->minute_1_img,
                        LV_ALIGN_OUT_BOTTOM_RIGHT, -10, 0);
    }

    memcpy(&p_clk_earth_digital->last_redraw_time, &current_time,
           sizeof(app_clock_time_t));
}

static rt_int32_t resume_callback(void)
{
    if (NULL == p_clk_earth_digital->redraw_task)
    {
        p_clk_earth_digital->redraw_task =
            lv_timer_create(app_clock_earth_digital_redraw, 30, (void *)0);
        app_clock_earth_digital_redraw(NULL);
    }

    return RT_EOK;
}

static rt_int32_t pause_callback(void)
{
    if (p_clk_earth_digital->redraw_task)
    {
        lv_timer_del(p_clk_earth_digital->redraw_task);
        p_clk_earth_digital->redraw_task = NULL;
    }

    return RT_EOK;
}

lv_obj_t *lv_earth_digital_layout_create(lv_obj_t *parent)
{
    p_clk_earth_digital = (app_clock_earth_digital_t *)rt_malloc(
        sizeof(app_clock_earth_digital_t));
    memset(p_clk_earth_digital, 0, sizeof(app_clock_earth_digital_t));

    // Create background with subtle gradient
    p_clk_earth_digital->bg = lv_obj_create(parent);
    lv_obj_set_size(p_clk_earth_digital->bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(p_clk_earth_digital->bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(p_clk_earth_digital->bg, lv_color_hex(0x000000),
                              0);
    lv_obj_set_style_radius(p_clk_earth_digital->bg, 233, 0);
    lv_obj_clear_flag(p_clk_earth_digital->bg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *h_0 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(h_0, 100, 136);
    lv_obj_align(h_0, LV_ALIGN_CENTER, -153, -10);
    lv_obj_set_style_bg_color(h_0, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(h_0, 0, 0);

    // Create hour label
    p_clk_earth_digital->hour_0_img = lv_img_create(h_0);
    lv_img_set_src(p_clk_earth_digital->hour_0_img, &dig_1_00);
    lv_obj_center(p_clk_earth_digital->hour_0_img);

    lv_obj_t *h_1 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(h_1, 100, 136);
    lv_obj_align(h_1, LV_ALIGN_CENTER, -52, -10);
    lv_obj_set_style_bg_color(h_1, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(h_1, 0, 0);

    p_clk_earth_digital->hour_1_img = lv_img_create(h_1);
    lv_img_set_src(p_clk_earth_digital->hour_1_img, &dig_1_00);
    lv_obj_center(p_clk_earth_digital->hour_1_img);

    lv_obj_t *m_0 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(m_0, 100, 136);
    lv_obj_align(m_0, LV_ALIGN_CENTER, 52, -10);
    lv_obj_set_style_bg_color(m_0, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(m_0, 0, 0);

    p_clk_earth_digital->minute_0_img = lv_img_create(m_0);
    lv_img_set_src(p_clk_earth_digital->minute_0_img, &dig_1_00);
    lv_obj_set_style_img_opa(p_clk_earth_digital->minute_0_img, LV_OPA_70, 0);
    lv_obj_center(p_clk_earth_digital->minute_0_img);

    lv_obj_t *m_1 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(m_1, 100, 136);
    lv_obj_align(m_1, LV_ALIGN_CENTER, 153, -10);
    lv_obj_set_style_bg_color(m_1, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(m_1, 0, 0);

    p_clk_earth_digital->minute_1_img = lv_img_create(m_1);
    lv_img_set_src(p_clk_earth_digital->minute_1_img, &dig_1_00);
    lv_obj_set_style_img_opa(p_clk_earth_digital->minute_1_img, LV_OPA_70, 0);
    lv_obj_center(p_clk_earth_digital->minute_1_img);

    lv_obj_set_style_bg_opa(h_0, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(h_1, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(m_0, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(m_1, LV_OPA_TRANSP, 0);

    p_clk_earth_digital->earth_img = lv_img_create(p_clk_earth_digital->bg);
    lv_img_set_src(p_clk_earth_digital->earth_img, &img_earth_digital_bg);
    lv_obj_align(p_clk_earth_digital->earth_img, LV_ALIGN_BOTTOM_MID, 0, -30);

    /* AM/PM tag at the bottom-right corner of the minutes — small and
       half-transparent so it reads as a tag, not a digit. Shown only in
       12-hour mode; the redraw task fills it in / clears it based on
       hour_format. */
    p_clk_earth_digital->ampm_label = lv_label_create(p_clk_earth_digital->bg);
    lv_obj_set_style_text_font(p_clk_earth_digital->ampm_label,
                               LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(p_clk_earth_digital->ampm_label,
                                lv_color_white(), 0);
    lv_obj_set_style_text_opa(p_clk_earth_digital->ampm_label, LV_OPA_50, 0);
    lv_label_set_text(p_clk_earth_digital->ampm_label, "");
    /* Below the minute DIGIT, right edge aligned to the minute's right edge
       (-10 trims the digit image's transparent right margin). */
    lv_obj_align_to(p_clk_earth_digital->ampm_label,
                    p_clk_earth_digital->minute_1_img,
                    LV_ALIGN_OUT_BOTTOM_RIGHT, -10, 0);

    p_clk_earth_digital->redraw_task = NULL;

    return p_clk_earth_digital->bg;
}

static rt_int32_t init(lv_obj_t *parent)
{
    lv_earth_digital_layout_create(parent);
    return RT_EOK;
}

static rt_int32_t deinit(void)
{
    if (p_clk_earth_digital)
    {
        if (p_clk_earth_digital->redraw_task)
        {
            lv_timer_del(p_clk_earth_digital->redraw_task);
        }
        rt_free(p_clk_earth_digital);
        p_clk_earth_digital = NULL;
    }

    return RT_EOK;
}

static const app_clock_ops_t ops = {
    .init = init,
    .pause = pause_callback,
    .resume = resume_callback,
    .deinit = deinit,
};

void app_clock_earth_digital_register(void)
{
    app_clock_register("JW_wf1", &ops);
}
