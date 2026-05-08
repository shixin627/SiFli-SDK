/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * PC simulator stub of drv_touch.h. Mirrors the public API surface used by
 * dualcore project code (touch event constants + rt_touch_set_event_callback)
 * without pulling in drv_io.h (ARM-only HAL).
 */
#ifndef __DRV_TOUCH_H__
#define __DRV_TOUCH_H__

#include "rtthread.h"
#include "rtdevice.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TOUCH_EVENT_UP      (0x01)
#define TOUCH_EVENT_DOWN    (0x02)
#define TOUCH_EVENT_MOVE    (0x03)
#define TOUCH_EVENT_NONE    (0x80)

struct touch_message
{
    rt_uint16_t x;
    rt_uint16_t y;
    rt_uint8_t event;
};
typedef struct touch_message *touch_msg_t;

extern void rt_touch_set_event_callback(void (*callback)(rt_uint8_t event, rt_uint16_t x, rt_uint16_t y));

#ifdef __cplusplus
}
#endif
#endif
