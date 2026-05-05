/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "board.h"
#include "drv_touch.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

typedef struct
{
    rt_device_t rt_device;
    rt_uint32_t data_cnt;
    lv_indev_t *indev_drv;
} lv_touchscreen_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  WAKE-UP TOUCH SUPPRESSION
 **********************/

/* Wake-up touch suppression. Two independent gates:
 *   - `armed` (boolean): set by SUSPEND hook, lasts through fade-out and
 *     sleep until RESUME explicitly takes over. Doesn't auto-clear on
 *     UP — the chip polls and emits idle UPs at ~50Hz during fade-out,
 *     which would prematurely clear an auto-clearing flag.
 *   - `window` (deadline tick): set by RESUME hook to cover the brief
 *     burst of post-wake events the chip emits when it's repowered
 *     (the chip replays its last touch state and then idle UPs).
 * RESUME's set_window() also clears `armed`, so post-wake the window is
 * the sole gate. */
static volatile bool s_wake_suppress_armed = false;
static volatile uint32_t s_wake_suppress_until_tick = 0;

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

/**********************
 *   STATIC FUNCTIONS
 **********************/
static rt_err_t rx_indicate(rt_device_t dev, rt_size_t size)
{
    if (dev->user_data)
    {
        lv_touchscreen_t *p = (lv_touchscreen_t *)dev->user_data;
        p->data_cnt++;
        //LV_LOG_USER("lv touch device rx indicate!  %d\r\n", p->data_cnt);
    }

    return RT_EOK;
}

static void touchscreen_read(lv_indev_t *drv, lv_indev_data_t *data)
{
    lv_touchscreen_t *touchscreen = lv_indev_get_driver_data(drv);

    if (touchscreen->rt_device)
    {
        struct touch_message touch_data;

        rt_device_read(touchscreen->rt_device, 0, &touch_data, 1);

        bool in_window = (s_wake_suppress_until_tick != 0) &&
            ((int32_t)(s_wake_suppress_until_tick - lv_tick_get()) > 0);
        if (s_wake_suppress_armed || in_window)
        {
            data->state = LV_INDEV_STATE_RELEASED;
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
                data->state = LV_INDEV_STATE_PRESSED;
                break;

            case TOUCH_EVENT_UP:
            default:
                data->state = LV_INDEV_STATE_RELEASED;
                break;
            }
        }

        data->point.x = touch_data.x;
        data->point.y = touch_data.y;

        if (touchscreen->data_cnt > 0) --touchscreen->data_cnt;
    }

#ifdef BSP_USING_LVGL_INPUT_AGENT
    {
        extern int lv_indev_agent_filter(lv_indev_drv_t *drv, lv_indev_data_t *data);
        lv_indev_agent_filter(drv, data);
    }
#endif
    data->continue_reading = (touchscreen->data_cnt > 0);
}

static void touchscreen_delete_cb(lv_event_t *e)
{
    lv_indev_t *indev = (lv_indev_t *) lv_event_get_user_data(e);
    lv_touchscreen_t *touchscreen = lv_indev_get_driver_data(indev);
    if (touchscreen)
    {
        lv_indev_set_driver_data(indev, NULL);
        lv_indev_set_read_cb(indev, NULL);

        if (touchscreen->rt_device)
        {
            rt_device_close(touchscreen->rt_device);
            touchscreen->rt_device = NULL;
        }
        lv_free(touchscreen);
        LV_LOG_USER("done");
    }
}

static lv_touchscreen_t *touchscreen_init(rt_device_t dev)
{
    lv_touchscreen_t *touchscreen;
    lv_indev_t *indev = NULL;

    touchscreen = lv_malloc_zeroed(sizeof(lv_touchscreen_t));
    if (touchscreen == NULL)
    {
        LV_LOG_ERROR("touchscreen_s malloc failed");
        return NULL;
    }

    touchscreen->rt_device = dev;
    touchscreen->data_cnt = 0;
    touchscreen->indev_drv = indev = lv_indev_create();

    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touchscreen_read);
    lv_indev_set_driver_data(indev, touchscreen);
    lv_indev_add_event_cb(indev, touchscreen_delete_cb, LV_EVENT_DELETE, indev);

#ifdef BSP_USING_LVGL_INPUT_AGENT
    {
        extern void lv_indev_agent_init(lv_indev_drv_t *drv);
        lv_indev_agent_init(indev);
    }
#endif

    return touchscreen;

}

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_indev_t *lv_touchscreen_create(const char *dev_path)
{
    lv_touchscreen_t *touchscreen;
    rt_device_t dev;

    LV_ASSERT_NULL(dev_path);
    LV_LOG_USER("touchscreen %s opening", dev_path);

    /*Open touch device*/
    dev = rt_device_find(dev_path);
    if (!dev)
    {
        LV_LOG_USER("Error: cannot find touchscreen device");
        return NULL;
    }
    if (RT_EOK == rt_device_open(dev, RT_DEVICE_FLAG_RDONLY))
    {
        dev->rx_indicate = rx_indicate;
    }
    else
    {
        LV_LOG_USER("Error: cannot open touchscreen device");
        return NULL;
    }

    LV_LOG_USER("touchscreen %s open success", dev_path);

    touchscreen = touchscreen_init(dev);

    if (touchscreen == NULL)
    {
        rt_device_close(dev);
    }
    else
    {
        dev->user_data = touchscreen;
    }

    return touchscreen->indev_drv;
}

