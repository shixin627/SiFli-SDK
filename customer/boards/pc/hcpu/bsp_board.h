/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * PC simulator stub of bsp_board.h.
 * Mirrors logical constants from sf32lb56-watch_base/bsp_board.h (BOARD_VER_26)
 * minus the HAL/IO bits that don't exist on Win32. GPIO pin numbers are kept
 * because gui_apps/modules code only uses them as opaque ints in PC sim.
 */
#ifndef __BSP_BOARD_H__
#define __BSP_BOARD_H__

#include "rtconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CUSTOMER_BOARD

#ifdef CUSTOMER_BOARD

#define INT16_to_G (8192.0f)
#define GRAVITY 9.80665f
#define IMU_NOARMAL_SAMPLE_RATE 100
#define IMU_NOARMAL_PERIOD (1000 / IMU_NOARMAL_SAMPLE_RATE)
#define IMU_SLEEPING_SAMPLE_RATE 50
#define IMU_SLEEPING_PERIOD (1000 / IMU_SLEEPING_SAMPLE_RATE)
#define INT16_to_DPS (16.384f)

#define SkaiwalkWatchOS 26
#ifndef kReleaseMode
#define kReleaseMode 0
#endif

#define BOARD_VER_26 26
#define CUSTOMER_BOARD_VER BOARD_VER_26

#define ENABLE_TAP_AND_HOLD 0
#define MAX_RAWDATA_TIME_STEP 35

/* GPIO pins — opaque ints on PC, no actual GPIO. */
#define CHARGE_DETECT_PIN     (-1)
#define AMOLED_DISPLAY_3V3_EN (-1)
#define AMOLED_BATTERY_EN     (-1)
#define AMOLED_1V8_EN         (-1)
#define WATCH_GSENSOR_POWER_EN (-1)
#define IMU_INT_PIN           (-1)
#define PPG_INT_PIN           (-1)
#define PPG_RST_PIN           (-1)
#define PPG_POWER_EN_PIN      (-1)
#define MOTOR_POWER_EN_PIN    (-1)
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define USING_LINEAR_MOTOR_0619
#define WATCH_IMU_REVERSE_180 (1)

#define PWM_LRA_MOTOR
#define REAL_TIME_IMU_DATA_COLLECTION
#define REAL_TIME_ACCEL_STEPS_COLLECTION

#define SHOW_TAP_GESTURE_INDICATOR
#define SHOW_UNKNOWN_GESTURE_INDICATOR

#define AUDIO_ENCODING_TYPE 1

#endif /* CUSTOMER_BOARD */

#define PERIPHERAL_AUD_SPEAKER 0

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BOARD_H__ */
