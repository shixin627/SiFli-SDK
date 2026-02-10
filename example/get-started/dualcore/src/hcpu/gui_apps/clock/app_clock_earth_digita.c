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

#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(img_earth_digital_bg);
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
} app_clock_earth_digital_t;

static app_clock_earth_digital_t *p_clk_earth_digital = NULL;

static void app_clock_earth_digital_redraw(lv_timer_t *task)
{
    app_clock_time_t current_time;
    rt_uint8_t hours, minutes, seconds;
    static char time_buf[8];
    static char date_buf[32];

    app_clock_main_get_current_time(&current_time);
    hours = current_time.h;
    minutes = current_time.m;
    seconds = current_time.s;


    // Update hour display with images
    int hour_0 = hours / 10;
    int hour_1 = hours % 10;
    int min_0 = minutes / 10;
    int min_1 = minutes % 10;

    // 指標陣列對應數字圖片資源
    const lv_img_dsc_t* dig_img[10] = {
        &dig_1_00, &dig_1_01, &dig_1_02, &dig_1_03, &dig_1_04,
        &dig_1_05, &dig_1_06, &dig_1_07, &dig_1_08, &dig_1_09
    };

    lv_img_set_src(p_clk_earth_digital->hour_0_img, dig_img[hour_0]);
    lv_img_set_src(p_clk_earth_digital->hour_1_img, dig_img[hour_1]);
    lv_img_set_src(p_clk_earth_digital->minute_0_img, dig_img[min_0]);
    lv_img_set_src(p_clk_earth_digital->minute_1_img, dig_img[min_1]);

    memcpy(&p_clk_earth_digital->last_redraw_time, &current_time, sizeof(app_clock_time_t));
}

static rt_int32_t resume_callback(void)
{
    if (NULL == p_clk_earth_digital->redraw_task)
    {
        p_clk_earth_digital->redraw_task = lv_timer_create(app_clock_earth_digital_redraw, 30, (void *)0);
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
    p_clk_earth_digital = (app_clock_earth_digital_t *)rt_malloc(sizeof(app_clock_earth_digital_t));
    memset(p_clk_earth_digital, 0, sizeof(app_clock_earth_digital_t));

    // Create background with subtle gradient
    p_clk_earth_digital->bg = lv_obj_create(parent);
    lv_obj_set_size(p_clk_earth_digital->bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(p_clk_earth_digital->bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(p_clk_earth_digital->bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(p_clk_earth_digital->bg, 233, 0);
    lv_obj_clear_flag(p_clk_earth_digital->bg, LV_OBJ_FLAG_SCROLLABLE);

    // Create hour label
    p_clk_earth_digital->hour_0_img = lv_img_create(p_clk_earth_digital->bg);
    lv_img_set_src(p_clk_earth_digital->hour_0_img, &dig_1_00);
    lv_obj_align(p_clk_earth_digital->hour_0_img, LV_ALIGN_CENTER, -150, -65);

    p_clk_earth_digital->hour_1_img = lv_img_create(p_clk_earth_digital->bg);
    lv_img_set_src(p_clk_earth_digital->hour_1_img, &dig_1_00);
    lv_obj_align(p_clk_earth_digital->hour_1_img, LV_ALIGN_CENTER, -52, -65);

    p_clk_earth_digital->minute_0_img = lv_img_create(p_clk_earth_digital->bg);
    lv_img_set_src(p_clk_earth_digital->minute_0_img, &dig_1_00);
    lv_obj_set_style_img_opa(p_clk_earth_digital->minute_0_img, LV_OPA_70, 0);
    lv_obj_align(p_clk_earth_digital->minute_0_img, LV_ALIGN_CENTER, 52, -65);

    p_clk_earth_digital->minute_1_img = lv_img_create(p_clk_earth_digital->bg);
    lv_img_set_src(p_clk_earth_digital->minute_1_img, &dig_1_00);
    lv_obj_set_style_img_opa(p_clk_earth_digital->minute_1_img, LV_OPA_70, 0);
    lv_obj_align(p_clk_earth_digital->minute_1_img, LV_ALIGN_CENTER, 150, -65);

    p_clk_earth_digital->earth_img = lv_img_create(p_clk_earth_digital->bg);
    lv_img_set_src(p_clk_earth_digital->earth_img, &img_earth_digital_bg);
    lv_obj_align(p_clk_earth_digital->earth_img, LV_ALIGN_BOTTOM_MID, 0, -5);

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

static const app_clock_ops_t ops =
{
    .init = init,
    .pause = pause_callback,
    .resume = resume_callback,
    .deinit = deinit,
};

void app_clock_earth_digital_register(void)
{
    app_clock_register("JW_wf1", &ops);
}
