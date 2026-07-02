/**
 * @file touch_state_manager.h
 * @brief Global touch state management module
 *
 * This module provides a centralized way to manage and query touch screen state
 * across the entire application. It handles touch events, gesture detection,
 * and provides APIs for other modules to query touch state.
 */

#ifndef __TOUCH_STATE_MANAGER_H__
#define __TOUCH_STATE_MANAGER_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "drv_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Use touch event types from drv_touch.h:
 * - TOUCH_EVENT_UP      (0x01)
 * - TOUCH_EVENT_DOWN    (0x02)
 * - TOUCH_EVENT_MOVE    (0x03)
 * - TOUCH_EVENT_NONE    (0x80)
 */

/* Touch state information */
typedef struct
{
    uint8_t event;  /* Touch event: TOUCH_EVENT_UP/DOWN/MOVE/NONE */
    uint16_t x;
    uint16_t y;
} touch_info_t;

/* Touch gesture types */
typedef enum
{
    TOUCH_GESTURE_NONE = 0,
    TOUCH_GESTURE_PRESSED,        /* Initial touch moment (on DOWN event) */
    TOUCH_GESTURE_QUICK_CLICK,    /* Quick tap (< 1000ms) */
    TOUCH_GESTURE_LONG_PRESS,     /* Long press (>= 1000ms) */
} touch_gesture_t;

/* Touch callback function type */
typedef void (*touch_state_callback_t)(uint8_t event, uint16_t x, uint16_t y);
typedef void (*touch_gesture_callback_t)(touch_gesture_t gesture, uint16_t x, uint16_t y);

/**
 * @brief Initialize touch state manager
 * @return RT_EOK on success, error code otherwise
 */
int touch_state_manager_init(void);

/**
 * @brief Update touch state (called by touch driver)
 * @param event Touch event type (TOUCH_EVENT_UP/DOWN/MOVE/NONE)
 * @param x X coordinate
 * @param y Y coordinate
 */
void touch_state_update(uint8_t event, uint16_t x, uint16_t y);

/**
 * @brief Check if user is currently touching the screen
 * @return true if touching, false otherwise
 */
bool is_user_touching_screen(void);

/**
 * @brief Register a callback for gesture detection
 * @param callback Callback function to be called on gesture detected
 * @return RT_EOK on success, error code otherwise
 */
int touch_gesture_register_callback(touch_gesture_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __TOUCH_STATE_MANAGER_H__ */
