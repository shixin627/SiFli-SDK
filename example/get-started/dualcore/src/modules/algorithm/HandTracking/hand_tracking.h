/**
 ******************************************************************************
 * @file   hand_tracking.h
 * @author Skaiwalk software development team
 * @brief  手腕動作追蹤模組介面：抬手/放手/翻腕/返回手勢偵測。
 ******************************************************************************
 */
#ifndef HAND_TRACKING_H
#define HAND_TRACKING_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
	void (*lift_callback)(uint8_t lift);
	void (*back_callback)(void);
	void (*wrist_pronation_callback)(void);
} HandTrackingProvider;
//-------------------------------------------------------------------------------------------
// Function declarations
void hand_tracking_init(void (*lift_callback)(uint8_t lift), void (*back_callback)(void));
void hand_tracking_data_update(float freq, float gyro_x, float gyro_y, bool open_wrist_rotation, bool if_watchface_visible, bool zero_velocity);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/