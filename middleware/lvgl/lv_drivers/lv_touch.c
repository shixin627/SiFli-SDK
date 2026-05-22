/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "board.h"
#include "drv_touch.h"
/* Optional: the dynamic display-refresh governor lives in the watch app
 * layer. Other projects that share this driver may not ship the header — in
 * that case the operate-press hook compiles out to a no-op. */
#if defined(__has_include)
    #if __has_include("disp_refr_governor.h")
        #include "disp_refr_governor.h"
        #define LV_TOUCH_HAS_REFR_GOVERNOR 1
    #endif
#endif
#ifndef LV_TOUCH_HAS_REFR_GOVERNOR
    #define disp_gov_notify_touch_press() ((void)0)
#endif

static lv_indev_drv_t indev_drv;
static rt_device_t touch_device = NULL;
static rt_uint32_t touch_data_cnt = 0;

/* Wake-up touch suppression. Two independent gates:
 *   - `armed` (boolean): set by SUSPEND hook, lasts through fade-out and
 *     sleep until RESUME explicitly takes over. Survives idle chip events
 *     (it doesn't auto-clear on UP — the chip polls and emits idle UPs at
 *     ~50Hz during fade-out, which would prematurely clear an
 *     auto-clearing flag).
 *   - `window` (deadline tick): set by RESUME hook to cover the brief
 *     burst of post-wake events the chip emits when it's repowered
 *     (the chip replays its last touch state and then idle UPs). Chosen
 *     long enough for the chip to finish replaying; shorter durations
 *     risk the wake-tap leaking through.
 * RESUME's set_window() also clears `armed`, so post-wake the window is
 * the sole gate. */
static volatile bool s_wake_suppress_armed = false;
static volatile uint32_t s_wake_suppress_until_tick = 0;

/* Last reported indev state, for REL->PR rising-edge detection feeding the
 * refresh governor's "operate touch press" hook. */
static lv_indev_state_t s_last_indev_state = LV_INDEV_STATE_REL;

void lv_touch_arm_wake_suppression(void)
{
    s_wake_suppress_armed = true;
}

void lv_touch_set_wake_suppress_window(uint32_t window_ms)
{
    s_wake_suppress_armed = false;
    s_wake_suppress_until_tick = lv_tick_get() + window_ms;
}

void lv_touch_clear_wake_suppression(void)
{
    s_wake_suppress_armed = false;
    s_wake_suppress_until_tick = 0;
}

static rt_err_t rx_indicate(rt_device_t dev, rt_size_t size)
{
    ++touch_data_cnt;

    //rt_kprintf("lv touch device rx indicate!  %d\r\n", touch_data_cnt);

    return RT_EOK;
}

static void input_read(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data)   //(lv_indev_data_t *data)//(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data
{

    if (touch_device)
    {
        struct touch_message touch_data;

        rt_device_read(touch_device, 0, &touch_data, 1);

        bool in_window = (s_wake_suppress_until_tick != 0) &&
            ((int32_t)(s_wake_suppress_until_tick - lv_tick_get()) > 0);
        if (s_wake_suppress_armed || in_window)
        {
            data->state = LV_INDEV_STATE_REL;
        }
        else
        {
            if (s_wake_suppress_until_tick != 0)
            {
                s_wake_suppress_until_tick = 0;
            }
            switch (touch_data.event)
            {
            case TOUCH_EVENT_DOWN:
            case TOUCH_EVENT_MOVE:
                data->state = LV_INDEV_STATE_PR;
                break;

            case TOUCH_EVENT_UP:
            default:
                data->state = LV_INDEV_STATE_REL;
                break;
            }

            /* Refresh governor: a touch press that survived wake suppression
             * is an "operate" input on a wake session. Fire only on the
             * REL->PR rising edge (not while held / moving). The governor
             * applies its own post-wake debounce, so a wake-tap that races
             * past suppression still won't latch. */
            if (data->state == LV_INDEV_STATE_PR &&
                s_last_indev_state != LV_INDEV_STATE_PR)
            {
                disp_gov_notify_touch_press();
            }
        }
        s_last_indev_state = data->state;

        data->point.x = touch_data.x;
        data->point.y = touch_data.y;
        // rt_kprintf("touch event: %d, x: %d, y: %d\r\n", touch_data.event, touch_data.x, touch_data.y);

        if (touch_data_cnt > 0) --touch_data_cnt;
    }

#ifdef BSP_USING_LVGL_INPUT_AGENT
    {
        extern int lv_indev_agent_filter(lv_indev_drv_t *drv, lv_indev_data_t *data);
        lv_indev_agent_filter(indev_drv, data);
    }
#endif
    data->continue_reading = (touch_data_cnt > 0);
}

static lv_indev_t *touch_indev;

void touch_init()
{
    //slv_indev_t *dev;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;

    /*Open touch device*/
    RT_ASSERT(NULL == touch_device);
    touch_device = rt_device_find("touch");
    if (touch_device)
    {
        if (RT_EOK == rt_device_open(touch_device, RT_DEVICE_FLAG_RDONLY))
        {
            touch_device->rx_indicate = rx_indicate;
        }
        else
        {
            touch_device = NULL;
        }
    }

    indev_drv.read_cb = input_read;
    touch_indev = lv_indev_drv_register(&indev_drv);

#ifdef BSP_USING_LVGL_INPUT_AGENT
    {
        extern void lv_indev_agent_init(lv_indev_drv_t *drv);
        lv_indev_agent_init(&indev_drv);
    }
#endif

}

lv_indev_t *touch_get_indev_handler(void)
{
    return touch_indev;
}

