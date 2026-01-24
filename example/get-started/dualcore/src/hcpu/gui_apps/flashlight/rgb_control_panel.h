/**
 ******************************************************************************
 * @file   rgb_control_panel.h
 * @author Skaiwalk software development team
 * @brief  RGB LED control panel header file
 ******************************************************************************
 */

#ifndef RGB_CONTROL_PANEL_H
#define RGB_CONTROL_PANEL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "bloc_peripheral.h"
#include <rtthread.h>

    /*********************
     *      DEFINES
     *********************/

    /**********************
     *      TYPEDEFS
     **********************/
    /**
     * @brief RGB LED state structure
     */
    typedef struct
    {
        bool enabled;                        // Power on/off
        rgb_color_t base_color;              // Base color (when not animating)
        uint8_t brightness;                  // Brightness (0-100)
        rgb_animation_mode_t animation_mode; // Current animation mode
        uint32_t period_ms;                  // Animation period in milliseconds
        uint8_t repeat_times;                // Number of repetitions (0 = infinite)
    } rgb_led_state_t;

    /**********************
     * GLOBAL PROTOTYPES
     **********************/

    /**
     * @brief Create and show the RGB control panel
     *
     * Opens a full-screen RGB control panel with color picker,
     * brightness slider, animation mode selection, and power switch.
     *
     * @param parent Parent screen object
     * @param state Pointer to RGB LED state structure
     */
    void create_rgb_control_panel(lv_obj_t *parent, rgb_led_state_t *state);

    /**
     * @brief Close the RGB control panel
     *
     * Closes and deletes the RGB control panel UI.
     */
    void close_rgb_control_panel(void);

    /**
     * @brief Check if RGB control panel is currently open
     *
     * @return true if panel is open, false otherwise
     */
    bool is_rgb_panel_open(void);

    /**
     * @brief Initialize RGB LED state structure
     *
     * @param state Pointer to RGB LED state structure
     */
    void rgb_led_state_init(rgb_led_state_t *state);

    /**
     * @brief Cleanup RGB LED state and stop animations
     *
     * @param state Pointer to RGB LED state structure
     */
    void rgb_led_state_cleanup(rgb_led_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* RGB_CONTROL_PANEL_H */

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
