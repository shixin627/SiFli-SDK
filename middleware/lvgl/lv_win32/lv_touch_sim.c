/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * See lv_touch_sim.h for design notes.
 */
#include "lv_touch_sim.h"

#include <rtthread.h>
#include "lvgl.h"

#define DBG_TAG "tsim"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_mutex sim_lock;
static lv_coord_t      sim_x = 0;
static lv_coord_t      sim_y = 0;
static bool            sim_pressed = false;
static lv_indev_t     *sim_indev = NULL;
static bool            sim_inited = false;

static void touch_sim_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;
    rt_mutex_take(&sim_lock, RT_WAITING_FOREVER);
    data->point.x = sim_x;
    data->point.y = sim_y;
    data->state   = sim_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    rt_mutex_release(&sim_lock);
}

void lv_touch_sim_init(void)
{
    if (sim_inited) return;

    rt_mutex_init(&sim_lock, "tsim", RT_IPC_FLAG_PRIO);

    static lv_indev_drv_t drv;
    lv_indev_drv_init(&drv);
    drv.type    = LV_INDEV_TYPE_POINTER;
    drv.read_cb = touch_sim_read_cb;
    sim_indev   = lv_indev_drv_register(&drv);

    sim_inited = true;
    LOG_I("lv_touch_sim registered (indev=%p)", sim_indev);
}

void lv_touch_sim_set_state(int x, int y, bool pressed)
{
    if (!sim_inited)
    {
        LOG_W("lv_touch_sim_set_state called before init");
        return;
    }
    rt_mutex_take(&sim_lock, RT_WAITING_FOREVER);
    sim_x       = (lv_coord_t)x;
    sim_y       = (lv_coord_t)y;
    sim_pressed = pressed;
    rt_mutex_release(&sim_lock);
}

bool lv_touch_sim_get_state(int *x, int *y, bool *pressed)
{
    if (!sim_inited) return false;
    rt_mutex_take(&sim_lock, RT_WAITING_FOREVER);
    if (x)       *x = sim_x;
    if (y)       *y = sim_y;
    if (pressed) *pressed = sim_pressed;
    rt_mutex_release(&sim_lock);
    return true;
}
