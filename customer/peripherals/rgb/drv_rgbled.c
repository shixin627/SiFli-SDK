/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"
#include "bf0_hal_rcc.h"
#include "drv_rgbled.h"

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_rgbled RGBLED driver
  * @ingroup bsp_driver
  * @brief RGBLED BSP driver
  * @{
  */

#if defined(RGB_SK6812MINI_HS_ENABLE) || defined(_SIFLI_DOXYGEN_)

#include "drv_config.h"
#include "mem_section.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.rgb"
#include <drv_log.h>

#define RGB_COLOR_BITS_PER_LED   24
#define RGB_REST_LEN   300
#define RGB_STOP_LEN   50

// Calculate total buffer size based on configured LED count
#define RGB_TOTAL_COLOR_LEN   (RGB_COLOR_BITS_PER_LED * BSP_RGB_LED_COUNT)
#define RGB_TOTAL_BUFFER_LEN  (RGB_REST_LEN + RGB_TOTAL_COLOR_LEN + RGB_STOP_LEN)

/* PWM timing parameters for SK6812 protocol
 * Based on 48MHz timer clock with prescaler resulting in 30MHz counter
 * Period = 1600 ticks = 53.3μs (close to SK6812's 1.25μs ± 0.6μs requirement)
 */
#define pwm_period  1600        /*!< PWM period in timer ticks */
#define pulse_period  800       /*!< PWM pulse width in timer ticks (50% duty cycle base) */

/* WS2812B/SK6812 protocol timing values (PWM compare register values)
 * Based on 30MHz counter (1 tick = 0.0333μs)
 * WS2812B: T0H=0.3μs, T0L=0.9μs, T1H=0.9μs, T1L=0.3μs
 * SK6812:  T0H=0.3μs, T0L=0.9μs, T1H=0.6μs, T1L=0.6μs
 */
#ifdef WS2812B_TIMING
#define reg_high 29    /*!< PWM compare value for logic '1' (adjusted for WS2812B T1H=0.9μs) */
#define reg_low  9     /*!< PWM compare value for logic '0' (9/30MHz = 0.3μs) */
#else
#define reg_high 18    /*!< PWM compare value for logic '1' (18/30MHz = 0.6μs for SK6812) */
#define reg_low  7     /*!< PWM compare value for logic '0' (7/30MHz = 0.23μs) */
#endif
#define reg_end  50    /*!< PWM compare value for end/reset signal (50/30MHz = 1.67μs) */

// WS2812B requires >50μs reset time for latching
// This is handled by RGB_STOP_LEN low signals, not just reg_end

#ifdef RGB_USING_SK6812MINI_HS_DEV_NAME
    #define RGBLED_NAME "rgbled"
#endif

struct bf0_rgbled
{
    struct rt_device device;                      /*!< RT-Thread device base */
    struct rt_device_pwm *pwm_device;             /*!< PWM device handle */
    char *name;                                   /*!< Device name */
    rt_uint16_t max_led_count;                    /*!< Maximum supported LED count */
    uint16_t rgb_buffer[RGB_TOTAL_BUFFER_LEN];    /*!< Static RGB buffer */
    rt_mutex_t mutex;                             /*!< Mutex for thread-safe operations */
};

#ifdef RGB_USING_SK6812MINI_HS_DEV_NAME
static struct bf0_rgbled bf0_rgbled_obj = {
    .name = RGBLED_NAME, 
    .max_led_count = BSP_RGB_LED_COUNT, 
    .rgb_buffer = {0},
    .pwm_device = RT_NULL,
    .mutex = RT_NULL
};
#endif

static rt_err_t drv_rgbled_control(rt_device_t device, int cmd, void *arg);

/**
 * @brief Create color array for LEDs
 * @param rgb_obj RGB LED device object
 * @param colors Array of RGB color values
 * @param led_count Number of LEDs
 */
static void create_color_array(struct bf0_rgbled *rgb_obj, rt_uint32_t *colors, rt_uint16_t led_count)
{
    if (led_count > rgb_obj->max_led_count)
    {
        LOG_E("LED count %d exceeds maximum %d", led_count, rgb_obj->max_led_count);
        led_count = rgb_obj->max_led_count;
    }

    rt_memset(rgb_obj->rgb_buffer, 0, sizeof(rgb_obj->rgb_buffer));
    
    uint16_t offset = RGB_REST_LEN;

    for (int led = 0; led < led_count; led++)
    {
        uint32_t color = colors[led];
        uint8_t red = (uint8_t)((color & 0x00ff0000) >> 16);
        uint8_t green = (uint8_t)((color & 0x0000ff00) >> 8);
        uint8_t blue = (uint8_t)((color & 0x000000ff) >> 0);

        uint16_t led_offset = offset + (led * RGB_COLOR_BITS_PER_LED);

        // Optimized: Encode all three colors in one loop (GRB order for SK6812)
        for (int i = 0; i < 8; i++)
        {
            // Encode Green (GRB order for SK6812)
            rgb_obj->rgb_buffer[led_offset + i] = ((green << i) & 0x80) ? reg_high : reg_low;
            // Encode Red  
            rgb_obj->rgb_buffer[led_offset + i + 8] = ((red << i) & 0x80) ? reg_high : reg_low;
            // Encode Blue
            rgb_obj->rgb_buffer[led_offset + i + 16] = ((blue << i) & 0x80) ? reg_high : reg_low;
        }
    }
    
    // Add stop signal after all LEDs
    rgb_obj->rgb_buffer[RGB_REST_LEN + (led_count * RGB_COLOR_BITS_PER_LED)] = reg_end;

#ifdef DRV_DEBUG
    LOG_D("Color array created for %d LEDs", led_count);
#endif
}

/**
 * @brief Send RGB data to LEDs
 * @param rgb_obj RGB LED object handle
 * @param led_count Number of LEDs to update
 * @retval RT_EOK if success, otherwise error code
 */
static rt_err_t drv_rgbled_send_data(struct bf0_rgbled *rgb_obj, rt_uint16_t led_count)
{
    struct rt_pwm_configuration config;
    rt_err_t result;
    
#ifndef BSP_USING_RGBLED_CH
    LOG_E("NO CONFIG BSP_USING_RGBLED_CH");
    return -RT_ERROR;
#endif

    rt_memset((void *)&config, 0, sizeof(config));
    config.channel = BSP_USING_RGBLED_CH;
    rt_device_control((struct rt_device *)rgb_obj->pwm_device, PWM_CMD_DISABLE, (void *)&config);
    rt_thread_mdelay(1);

    rt_memset((void *)&config, 0, sizeof(config));
    config.channel = BSP_USING_RGBLED_CH;
    config.period = pwm_period;
    config.pulse = pulse_period;
    config.dma_type = 0;
    config.dma_data = (rt_uint16_t *)rgb_obj->rgb_buffer;
    config.data_len = RGB_REST_LEN + (led_count * RGB_COLOR_BITS_PER_LED) + RGB_STOP_LEN;

    result = rt_device_control((struct rt_device *)rgb_obj->pwm_device, PWM_CMD_SET, (void *)&config);
    if (result != RT_EOK)
    {
        LOG_E("PWM_CMD_SET failed with error %d", result);
        return result;
    }

    result = rt_device_control((struct rt_device *)rgb_obj->pwm_device, PWM_CMD_ENABLE, (void *)&config);
    if (result != RT_EOK)
    {
        LOG_E("PWM_CMD_ENABLE failed with error %d", result);
        return result;
    }

    uint32_t transfer_time_ms = (config.data_len * 2) / 1000 + 2;
    rt_thread_mdelay(transfer_time_ms);

    rt_memset((void *)&config, 0, sizeof(config));
    config.channel = BSP_USING_RGBLED_CH;
    rt_device_control((struct rt_device *)rgb_obj->pwm_device, PWM_CMD_DISABLE, (void *)&config);
    
    return RT_EOK;
}

/**
 * @brief RGB LED device control function
 * @param device RGB device handle
 * @param cmd Control command
 * @param arg Command arguments
 * @retval RT_EOK if success, otherwise error code
 */
static rt_err_t drv_rgbled_control(rt_device_t device, int cmd, void *arg)
{
    struct bf0_rgbled *rgb_obj = (struct bf0_rgbled *) device->user_data;
    rt_err_t result = RT_EOK;

    // Acquire mutex for thread-safe operations
    if (rgb_obj->mutex)
    {
        result = rt_mutex_take(rgb_obj->mutex, RT_WAITING_FOREVER);
        if (result != RT_EOK)
        {
            LOG_E("Failed to acquire RGB mutex");
            return result;
        }
    }

    switch (cmd)
    {
    case RGB_CMD_SET_SINGLE_COLOR: 
    case PWM_CMD_SET_COLOR:
        {
            struct rt_rgbled_configuration *single_config = (struct rt_rgbled_configuration *)arg;
            if (!single_config)
            {
                LOG_E("Invalid single color configuration");
                result = -RT_ERROR;
                break;
            }
            
            rt_uint32_t color = single_config->color_rgb;
            create_color_array(rgb_obj, &color, 1);
            result = drv_rgbled_send_data(rgb_obj, 1);
        }
        break;
        
    case RGB_CMD_SET_MULTI_COLOR:
        {
            struct rt_rgbled_multi_configuration *multi_config = (struct rt_rgbled_multi_configuration *)arg;
            if (!multi_config || !multi_config->color_array || multi_config->led_count == 0)
            {
                LOG_E("Invalid multi-color configuration");
                result = -RT_ERROR;
                break;
            }
            
            create_color_array(rgb_obj, multi_config->color_array, multi_config->led_count);
            result = drv_rgbled_send_data(rgb_obj, multi_config->led_count);
        }
        break;
        
    case RGB_CMD_GET_CAPABILITY:
        {
            rt_uint32_t *led_count = (rt_uint32_t *)arg;
            if (!led_count)
            {
                LOG_E("Invalid capability query parameter");
                result = -RT_ERROR;
                break;
            }
            
            *led_count = BSP_RGB_LED_COUNT;
            result = RT_EOK;
        }
        break;
        
    default:
        LOG_W("Unknown command: %d", cmd);
        result = -RT_EINVAL;
        break;
    }

    // Release mutex
    if (rgb_obj->mutex)
    {
        rt_mutex_release(rgb_obj->mutex);
    }

    return result;
}

/**
 * @brief RGB LED device driver initialization
 * @retval RT_EOK if success, otherwise error code
 */
static int bf0_rgbled_init(void)
{
    int result = RT_EOK;

    // Create mutex for thread-safe operations
    bf0_rgbled_obj.mutex = rt_mutex_create("rgb_drv", RT_IPC_FLAG_PRIO);
    if (!bf0_rgbled_obj.mutex)
    {
        LOG_E("Failed to create RGB mutex");
        return -RT_ENOMEM;
    }

#ifdef RGB_USING_SK6812MINI_HS_PWM_DEV_NAME
    // Find PWM device
    bf0_rgbled_obj.pwm_device = (struct rt_device_pwm *)rt_device_find(RGB_USING_SK6812MINI_HS_PWM_DEV_NAME);
    if (!bf0_rgbled_obj.pwm_device)
    {
        LOG_E("Find pwm device failed");
        rt_mutex_delete(bf0_rgbled_obj.mutex);
        bf0_rgbled_obj.mutex = RT_NULL;
        return -RT_ERROR;
    }
#else
    LOG_E("NO CONFIG RGB_USING_SK6812MINI_HS_PWM_DEV_NAME");
    rt_mutex_delete(bf0_rgbled_obj.mutex);
    bf0_rgbled_obj.mutex = RT_NULL;
    return -RT_ERROR;
#endif

    // Register RGB device
    bf0_rgbled_obj.device.control = (rt_err_t (*)(rt_device_t, int, void *))drv_rgbled_control;
    bf0_rgbled_obj.device.user_data = &bf0_rgbled_obj;

    result = rt_device_register(&bf0_rgbled_obj.device, bf0_rgbled_obj.name, RT_DEVICE_FLAG_RDWR);
    if (result != RT_EOK)
    {
        LOG_E("%s register failed", RGBLED_NAME);
        rt_mutex_delete(bf0_rgbled_obj.mutex);
        bf0_rgbled_obj.mutex = RT_NULL;
        return -RT_ERROR;
    }

    LOG_I("%s register success, supporting %d LEDs", RGBLED_NAME, BSP_RGB_LED_COUNT);
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(bf0_rgbled_init);

/**
 * @brief RGB LED device driver deinitialization
 * @retval RT_EOK if success, otherwise error code
 */
static int bf0_rgbled_deinit(void)
{
    // Unregister device
    rt_device_unregister(&bf0_rgbled_obj.device);
    
    // Delete mutex
    if (bf0_rgbled_obj.mutex)
    {
        rt_mutex_delete(bf0_rgbled_obj.mutex);
        bf0_rgbled_obj.mutex = RT_NULL;
    }
    
    LOG_I("%s deinitialized", RGBLED_NAME);
    return RT_EOK;
}

#endif /* RGB_SK6812MINI_HS_ENABLE */

/** @} drv_rgbled */
/** @} bsp_driver */