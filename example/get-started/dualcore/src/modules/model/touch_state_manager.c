/**
 * @file touch_state_manager.c
 * @brief Global touch state management module implementation
 */

#include "touch_state_manager.h"
#include <string.h>

#define DBG_TAG "touch.state"
#define DBG_LVL DBG_ERROR
#include <rtdbg.h>

/* Configuration */
#define MAX_TOUCH_CALLBACKS 4
#define MAX_GESTURE_CALLBACKS 4
#define DEFAULT_LONG_PRESS_MS 1000
#define DEBOUNCE_INTERVAL_MS 100
#define DEFAULT_MAX_MOVE_DISTANCE                                              \
    30 /* Maximum pixels to still be considered a click */

/* Touch state manager context */
typedef struct
{
    /* Current state */
    bool is_touching;
    touch_info_t last_event;

    /* Timing information */
    rt_tick_t last_down_tick;
    rt_tick_t last_up_tick;
    rt_tick_t last_valid_press_tick; /* Last valid press (after debounce) */

    /* Position tracking for gesture detection */
    uint16_t down_x; /* Initial DOWN position */
    uint16_t down_y;
    uint16_t last_x; /* Last valid position (updated on DOWN/MOVE) */
    uint16_t last_y;

    /* Gesture detection */
    touch_gesture_t last_gesture;
    uint32_t long_press_threshold_ms;
    uint32_t max_move_distance; /* Maximum movement to be considered a click */

    /* Callbacks */
    touch_state_callback_t state_callbacks[MAX_TOUCH_CALLBACKS];
    touch_gesture_callback_t gesture_callbacks[MAX_GESTURE_CALLBACKS];

    /* Thread safety */
    struct rt_mutex mutex;
    bool initialized;
} touch_state_ctx_t;

/* Global context */
static touch_state_ctx_t g_touch_ctx = {0};

/* Forward declarations */
static void detect_gesture(uint8_t event, uint16_t x, uint16_t y);
static void notify_state_callbacks(uint8_t event, uint16_t x, uint16_t y);
static void notify_gesture_callbacks(touch_gesture_t gesture, uint16_t x,
                                     uint16_t y);

/**
 * @brief Initialize touch state manager
 */
int touch_state_manager_init(void)
{
    if (g_touch_ctx.initialized)
    {
        LOG_W("Touch state manager already initialized");
        return RT_EOK;
    }

    memset(&g_touch_ctx, 0, sizeof(touch_state_ctx_t));

    /* Initialize mutex */
    rt_mutex_init(&g_touch_ctx.mutex, "touch_st", RT_IPC_FLAG_PRIO);

    /* Set default thresholds */
    g_touch_ctx.long_press_threshold_ms = DEFAULT_LONG_PRESS_MS;
    g_touch_ctx.max_move_distance = DEFAULT_MAX_MOVE_DISTANCE;

    g_touch_ctx.initialized = true;

    LOG_I("Touch state manager initialized");
    return RT_EOK;
}

/**
 * @brief Deinitialize touch state manager
 */
int touch_state_manager_deinit(void)
{
    if (!g_touch_ctx.initialized)
    {
        return RT_EOK;
    }

    rt_mutex_detach(&g_touch_ctx.mutex);

    memset(&g_touch_ctx, 0, sizeof(touch_state_ctx_t));

    LOG_I("Touch state manager deinitialized");
    return RT_EOK;
}

/**
 * @brief Update touch state (called by touch driver)
 */
void touch_state_update(uint8_t event, uint16_t x, uint16_t y)
{
    if (!g_touch_ctx.initialized)
    {
        LOG_W("Touch state manager not initialized");
        return;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);

    rt_tick_t now = rt_tick_get();

    /* Update state based on event */
    switch (event)
    {
    case TOUCH_EVENT_DOWN:
    {
        /* Only process as a new touch if not already touching */
        if (!g_touch_ctx.is_touching)
        {
            g_touch_ctx.is_touching = true;
            g_touch_ctx.last_down_tick = now;

            /* Store initial touch position */
            g_touch_ctx.down_x = x;
            g_touch_ctx.down_y = y;

            /* Update last valid position */
            g_touch_ctx.last_x = x;
            g_touch_ctx.last_y = y;

            /* Debounce: only update valid press tick if enough time has passed
             */
            rt_tick_t diff = now - g_touch_ctx.last_valid_press_tick;
            if (diff > rt_tick_from_millisecond(DEBOUNCE_INTERVAL_MS))
            {
                g_touch_ctx.last_valid_press_tick = now;
            }

            /* Trigger PRESSED gesture for initial touch moment */
            g_touch_ctx.last_gesture = TOUCH_GESTURE_PRESSED;
            LOG_D("Touch DOWN at (%d, %d), tick=%u - PRESSED gesture", x, y, now);

            /* Notify gesture callbacks for PRESSED */
            rt_mutex_release(&g_touch_ctx.mutex);
            notify_gesture_callbacks(TOUCH_GESTURE_PRESSED, x, y);
            rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
        }
        else
        {
            /* Already touching - this is a repeated DOWN event, treat as MOVE
             */
            /* Update last valid position */
            g_touch_ctx.last_x = x;
            g_touch_ctx.last_y = y;
            LOG_D("Repeated DOWN (treat as MOVE) at (%d, %d)", x, y);
        }
        break;
    }

    case TOUCH_EVENT_UP:
    {
        /* Only process gesture if we were actually touching */
        bool was_touching = g_touch_ctx.is_touching;

        g_touch_ctx.is_touching = false;
        g_touch_ctx.last_up_tick = now;

        /* Detect gesture only on first UP event (transition from touching to
         * not touching) */
        if (was_touching)
        {
            detect_gesture(event, x, y);
            LOG_D("Touch UP at (%d, %d), tick=%u", x, y, now);
        }
        else
        {
            /* Already released - this is a repeated UP event, ignore gesture
             * detection */
            LOG_D("Repeated UP (ignore) at (%d, %d)", x, y);
        }
        break;
    }

    case TOUCH_EVENT_MOVE:
        /* Update last valid position */
        g_touch_ctx.last_x = x;
        g_touch_ctx.last_y = y;
        LOG_D("Touch MOVE to (%d, %d)", x, y);
        break;

    default:
        break;
    }

    /* Update last event info */
    g_touch_ctx.last_event.event = event;
    g_touch_ctx.last_event.x = x;
    g_touch_ctx.last_event.y = y;

    rt_mutex_release(&g_touch_ctx.mutex);

    /* Notify callbacks (outside mutex to prevent deadlock) */
    notify_state_callbacks(event, x, y);
}

/**
 * @brief Check if user is currently touching the screen
 */
bool is_user_touching_screen(void)
{
    if (!g_touch_ctx.initialized)
    {
        return false;
    }

    bool touching;
    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
    touching = g_touch_ctx.is_touching;
    rt_mutex_release(&g_touch_ctx.mutex);

    return touching;
}

/**
 * @brief Get the last touch event information
 */
bool touch_state_get_last_event(touch_info_t *info)
{
    if (!g_touch_ctx.initialized || !info)
    {
        return false;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);

    if (g_touch_ctx.last_event.event == TOUCH_EVENT_NONE)
    {
        rt_mutex_release(&g_touch_ctx.mutex);
        return false;
    }

    memcpy(info, &g_touch_ctx.last_event, sizeof(touch_info_t));
    rt_mutex_release(&g_touch_ctx.mutex);

    return true;
}

/**
 * @brief Get the time when last touch down occurred
 */
rt_tick_t touch_state_get_last_down_tick(void)
{
    if (!g_touch_ctx.initialized)
    {
        return 0;
    }

    rt_tick_t tick;
    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
    tick = g_touch_ctx.last_down_tick;
    rt_mutex_release(&g_touch_ctx.mutex);

    return tick;
}

/**
 * @brief Get the time when last touch up occurred
 */
rt_tick_t touch_state_get_last_up_tick(void)
{
    if (!g_touch_ctx.initialized)
    {
        return 0;
    }

    rt_tick_t tick;
    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
    tick = g_touch_ctx.last_up_tick;
    rt_mutex_release(&g_touch_ctx.mutex);

    return tick;
}

/**
 * @brief Get the current touch press duration
 */
rt_tick_t touch_state_get_press_duration(void)
{
    if (!g_touch_ctx.initialized)
    {
        return 0;
    }

    rt_tick_t duration = 0;
    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);

    if (g_touch_ctx.is_touching && g_touch_ctx.last_down_tick > 0)
    {
        duration = rt_tick_get() - g_touch_ctx.last_down_tick;
    }

    rt_mutex_release(&g_touch_ctx.mutex);

    return duration;
}

/**
 * @brief Get the last detected gesture
 */
touch_gesture_t touch_state_get_last_gesture(void)
{
    if (!g_touch_ctx.initialized)
    {
        return TOUCH_GESTURE_NONE;
    }

    touch_gesture_t gesture;
    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
    gesture = g_touch_ctx.last_gesture;
    rt_mutex_release(&g_touch_ctx.mutex);

    return gesture;
}

/**
 * @brief Register touch state callback
 */
int touch_state_register_callback(touch_state_callback_t callback)
{
    if (!g_touch_ctx.initialized || !callback)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);

    /* Find empty slot */
    for (int i = 0; i < MAX_TOUCH_CALLBACKS; i++)
    {
        if (g_touch_ctx.state_callbacks[i] == NULL)
        {
            g_touch_ctx.state_callbacks[i] = callback;
            rt_mutex_release(&g_touch_ctx.mutex);
            LOG_D("Touch state callback registered at slot %d", i);
            return RT_EOK;
        }
    }

    rt_mutex_release(&g_touch_ctx.mutex);
    LOG_E("No free slot for touch state callback");
    return -RT_EFULL;
}

/**
 * @brief Unregister touch state callback
 */
int touch_state_unregister_callback(touch_state_callback_t callback)
{
    if (!g_touch_ctx.initialized || !callback)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);

    for (int i = 0; i < MAX_TOUCH_CALLBACKS; i++)
    {
        if (g_touch_ctx.state_callbacks[i] == callback)
        {
            g_touch_ctx.state_callbacks[i] = NULL;
            rt_mutex_release(&g_touch_ctx.mutex);
            LOG_D("Touch state callback unregistered from slot %d", i);
            return RT_EOK;
        }
    }

    rt_mutex_release(&g_touch_ctx.mutex);
    return -RT_EEMPTY;
}

/**
 * @brief Register gesture callback
 */
int touch_gesture_register_callback(touch_gesture_callback_t callback)
{
    if (!g_touch_ctx.initialized || !callback)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);

    /* Find empty slot */
    for (int i = 0; i < MAX_GESTURE_CALLBACKS; i++)
    {
        if (g_touch_ctx.gesture_callbacks[i] == NULL)
        {
            g_touch_ctx.gesture_callbacks[i] = callback;
            rt_mutex_release(&g_touch_ctx.mutex);
            LOG_D("Gesture callback registered at slot %d", i);
            return RT_EOK;
        }
    }

    rt_mutex_release(&g_touch_ctx.mutex);
    LOG_E("No free slot for gesture callback");
    return -RT_EFULL;
}

/**
 * @brief Unregister gesture callback
 */
int touch_gesture_unregister_callback(touch_gesture_callback_t callback)
{
    if (!g_touch_ctx.initialized || !callback)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);

    for (int i = 0; i < MAX_GESTURE_CALLBACKS; i++)
    {
        if (g_touch_ctx.gesture_callbacks[i] == callback)
        {
            g_touch_ctx.gesture_callbacks[i] = NULL;
            rt_mutex_release(&g_touch_ctx.mutex);
            LOG_D("Gesture callback unregistered from slot %d", i);
            return RT_EOK;
        }
    }

    rt_mutex_release(&g_touch_ctx.mutex);
    return -RT_EEMPTY;
}

/**
 * @brief Configure gesture detection thresholds
 */
int touch_gesture_set_thresholds(uint32_t long_press_ms)
{
    if (!g_touch_ctx.initialized)
    {
        return -RT_ERROR;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
    g_touch_ctx.long_press_threshold_ms = long_press_ms;
    rt_mutex_release(&g_touch_ctx.mutex);

    LOG_I("Gesture thresholds updated: long=%ums", long_press_ms);
    return RT_EOK;
}

/**
 * @brief Configure maximum movement distance for click detection
 */
int touch_gesture_set_max_move_distance(uint32_t max_distance)
{
    if (!g_touch_ctx.initialized)
    {
        return -RT_ERROR;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
    g_touch_ctx.max_move_distance = max_distance;
    rt_mutex_release(&g_touch_ctx.mutex);

    LOG_I("Max move distance updated: %u pixels", max_distance);
    return RT_EOK;
}

/* ========== Internal Functions ========== */

/**
 * @brief Calculate distance between two points
 */
static inline uint32_t calculate_distance(uint16_t x1, uint16_t y1, uint16_t x2,
                                          uint16_t y2)
{
    int32_t dx = (int32_t)x2 - (int32_t)x1;
    int32_t dy = (int32_t)y2 - (int32_t)y1;

    /* Simple Manhattan distance (faster than Euclidean) */
    uint32_t distance = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);

    return distance;
}

/**
 * @brief Detect gesture based on touch timing and movement
 */
static void detect_gesture(uint8_t event, uint16_t x, uint16_t y)
{
    if (event != TOUCH_EVENT_UP)
    {
        return;
    }

    rt_tick_t now = rt_tick_get();
    rt_tick_t duration = now - g_touch_ctx.last_valid_press_tick;

    /* Calculate movement distance using last valid position (not UP position
     * which may be 0,0) */
    uint32_t move_distance =
        calculate_distance(g_touch_ctx.down_x, g_touch_ctx.down_y,
                           g_touch_ctx.last_x, g_touch_ctx.last_y);

    touch_gesture_t gesture = TOUCH_GESTURE_NONE;

    /* Check if movement is too large for a click */
    if (move_distance > g_touch_ctx.max_move_distance)
    {
        /* Too much movement - not a click/press, likely a swipe */
        LOG_D("Movement too large (%u pixels), not a click", move_distance);
        gesture = TOUCH_GESTURE_NONE;
    }
    else if (duration < g_touch_ctx.long_press_threshold_ms)
    {
        gesture = TOUCH_GESTURE_QUICK_CLICK;
        LOG_I("Quick click detected (%u ms, %u pixels)", duration,
              move_distance);
    }
    else
    {
        gesture = TOUCH_GESTURE_LONG_PRESS;
        LOG_I("Long press detected (%u ms, %u pixels)", duration,
              move_distance);
    }

    g_touch_ctx.last_gesture = gesture;

    /* Only notify if a valid gesture was detected */
    if (gesture != TOUCH_GESTURE_NONE)
    {
        /* Use last valid position for gesture callback */
        notify_gesture_callbacks(gesture, g_touch_ctx.last_x,
                                 g_touch_ctx.last_y);
    }
}

/**
 * @brief Notify all registered state callbacks
 */
static void notify_state_callbacks(uint8_t event, uint16_t x, uint16_t y)
{
    touch_state_callback_t callbacks[MAX_TOUCH_CALLBACKS];
    int count = 0;

    /* Copy callbacks while holding mutex */
    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
    for (int i = 0; i < MAX_TOUCH_CALLBACKS; i++)
    {
        if (g_touch_ctx.state_callbacks[i] != NULL)
        {
            callbacks[count++] = g_touch_ctx.state_callbacks[i];
        }
    }
    rt_mutex_release(&g_touch_ctx.mutex);

    /* Call callbacks outside mutex */
    for (int i = 0; i < count; i++)
    {
        callbacks[i](event, x, y);
    }
}

/**
 * @brief Notify all registered gesture callbacks
 */
static void notify_gesture_callbacks(touch_gesture_t gesture, uint16_t x,
                                     uint16_t y)
{
    touch_gesture_callback_t callbacks[MAX_GESTURE_CALLBACKS];
    int count = 0;

    /* Copy callbacks while holding mutex */
    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);
    for (int i = 0; i < MAX_GESTURE_CALLBACKS; i++)
    {
        if (g_touch_ctx.gesture_callbacks[i] != NULL)
        {
            callbacks[count++] = g_touch_ctx.gesture_callbacks[i];
        }
    }
    rt_mutex_release(&g_touch_ctx.mutex);

    /* Call callbacks outside mutex */
    for (int i = 0; i < count; i++)
    {
        callbacks[i](gesture, x, y);
    }
}

/* ========== MSH Command for Testing ========== */
#ifdef RT_USING_FINSH
    #include <finsh.h>

static void touch_state_info(int argc, char **argv)
{
    if (!g_touch_ctx.initialized)
    {
        rt_kprintf("Touch state manager not initialized\n");
        return;
    }

    rt_mutex_take(&g_touch_ctx.mutex, RT_WAITING_FOREVER);

    rt_kprintf("=== Touch State Manager Info ===\n");
    rt_kprintf("Currently touching: %s\n",
               g_touch_ctx.is_touching ? "YES" : "NO");
    rt_kprintf("Last event: %d at (%d, %d)\n", g_touch_ctx.last_event.event,
               g_touch_ctx.last_event.x, g_touch_ctx.last_event.y);
    rt_kprintf("Initial DOWN position: (%u, %u)\n", g_touch_ctx.down_x,
               g_touch_ctx.down_y);
    rt_kprintf("Last valid position: (%u, %u)\n", g_touch_ctx.last_x,
               g_touch_ctx.last_y);
    rt_kprintf("Last down tick: %u\n", g_touch_ctx.last_down_tick);
    rt_kprintf("Last up tick: %u\n", g_touch_ctx.last_up_tick);
    rt_kprintf("Last gesture: %d\n", g_touch_ctx.last_gesture);
    rt_kprintf("\n");
    rt_kprintf("Gesture Configuration:\n");
    rt_kprintf("  Long press threshold: %u ms\n",
               g_touch_ctx.long_press_threshold_ms);
    rt_kprintf("  Max move distance: %u pixels\n",
               g_touch_ctx.max_move_distance);

    if (g_touch_ctx.is_touching)
    {
        rt_tick_t duration = rt_tick_get() - g_touch_ctx.last_down_tick;
        uint32_t move_dist =
            calculate_distance(g_touch_ctx.down_x, g_touch_ctx.down_y,
                               g_touch_ctx.last_x, g_touch_ctx.last_y);
        rt_kprintf("\n");
        rt_kprintf("Current touch:\n");
        rt_kprintf("  Press duration: %u ms\n", duration);
        rt_kprintf("  Movement from initial DOWN: %u pixels\n", move_dist);
    }

    rt_mutex_release(&g_touch_ctx.mutex);
}
MSH_CMD_EXPORT(touch_state_info, Show touch state manager information);

#endif /* RT_USING_FINSH */
