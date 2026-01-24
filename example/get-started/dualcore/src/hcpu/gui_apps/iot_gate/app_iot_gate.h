/**
 ******************************************************************************
 * @file   app_iot_gate.h
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 */

#ifndef APP_IOT_GATE_H
#define APP_IOT_GATE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "gui_app_fwk.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief Build IOT Gate widget
 * @param parent Parent object
 * @return Created widget object
 */
lv_obj_t *lv_iot_gate_widget_builder(lv_obj_t *parent);

/**
 * @brief Clear IOT Gate widget resources
 */
void clear_iot_gate_widget(void);

/**
 * @brief Start IOT Gate widget
 */
void iot_gate_widget_start(void);

/**
 * @brief Stop IOT Gate widget
 */
void iot_gate_widget_stop(void);

/**
 * @brief Handle tap event for IOT Gate widget
 */
void iot_gate_widget_handle_tap_event(void);

/**
 * @brief Handle press event for IOT Gate widget
 * @param press Press state (1 = pressed, 0 = released)
 */
void iot_gate_widget_handle_press_event(uint8_t press);

/**
 * @brief Trigger gate control by Y position (drag control)
 * @param p_y Y position (0~466)
 *            0~155: Select Close button (right)
 *            155~311: Select Pause button (center)
 *            311~466: Select Open button (left)
 */
void iot_gate_trigger_drag_by_py(int p_y);

/**
 * @brief Handle tap event callback for widget
 */
void iot_gate_widget_tap_event_cb(void);

/**
 * @brief Create close button for IOT Gate
 * @param parent Parent object
 * @return Created button object
 */
lv_obj_t *iot_gate_close_btn_create(lv_obj_t *parent);

/**
 * @brief Create open button for IOT Gate
 * @param parent Parent object
 * @return Created button object
 */
lv_obj_t *iot_gate_open_btn_create(lv_obj_t *parent);

/**
 * @brief Create pause button for IOT Gate
 * @param parent Parent object
 * @return Created button object
 */
lv_obj_t *iot_gate_pause_btn_create(lv_obj_t *parent);

/**
 * @brief Reset IOT Gate widget selection state
 */
void reset_iot_gate_widget(void);

/**
 * @brief Set selection for IOT Gate widget button
 * @param index Button index (0=open, 1=pause, 2=close)
 */
void selection_iot_gate_widget(uint8_t index);

/**
 * @brief Reset widget button background opacity
 */
void reset_widget_btn_bg(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_IOT_GATE_H */

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
