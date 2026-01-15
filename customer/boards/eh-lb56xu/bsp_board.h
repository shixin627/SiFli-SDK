/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BSP_BOARD_H__
#define __BSP_BOARD_H__

#include "rtconfig.h"
#include "drv_io.h"
#include "bf0_hal.h"
#ifdef PMIC_CTRL_ENABLE
    #include "pmic_controller.h"
#endif /* PMIC_CTRL_ENABLE */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif defined ( __GNUC__ )
extern int __bss_end;
#define HEAP_BEGIN      ((void *)&__bss_end)
#endif

#ifdef SOC_BF0_HCPU
#define HEAP_END       (HCPU_RAM_DATA_START_ADDR + HCPU_RAM_DATA_SIZE) //TODO:
#else
#define HEAP_END       (LCPU_RAM_DATA_START_ADDR + LCPU_RAM_DATA_SIZE) //TODO:
#endif


#define CUSTOMER_BOARD

#ifdef CUSTOMER_BOARD

// default range is +-4G, so conversion factor is (((1 << 15)/4.0f))
// range | factor
// +-2G  | 16384
// +-4G  | 8192
// +-8G  | 4096
// +-16G | 2048
#define INT16_to_G (8192.0f)
#define GRAVITY 9.80665f
#define IMU_NOARMAL_SAMPLE_RATE 100
#define IMU_NOARMAL_PERIOD 1000 / IMU_NOARMAL_SAMPLE_RATE
#define IMU_SLEEPING_SAMPLE_RATE 25
#define IMU_SLEEPING_PERIOD 1000 / IMU_SLEEPING_SAMPLE_RATE
// #define USE_IMU_REPORT_TIMER
// default range is +-2000dps, so conversion factor is (((1 << 15)/4.0f))
// range | factor
// +-250dps  | 131.072
// +-500dps  | 65.536
// +-1000dps | 32.768
// +-2000dps | 16.384
#define INT16_to_DPS (16.384f)
// Magnetic field range typical:
// ±1300µT (x, y-axis), ±2500µT (z-axis)
// Magnetic field resolution of ~0.3µT
// Conversion factor is 1/0.3 = 3.3333
// #define INT16_to_UT (3.3333f)

#define SkaiwalkWatchOS 26
#define kReleaseMode 0

#define BOARD_VER_11 11
#define BOARD_VER_12 12
#define BOARD_VER_13 13
#define BOARD_VER_14 14
#define BOARD_VER_15 15
#define BOARD_VER_16 16
#define BOARD_VER_17 17
#define BOARD_VER_18 18
#define BOARD_VER_19 19
#define BOARD_VER_20 20
#define BOARD_VER_21 21
#define BOARD_VER_22 22
#define BOARD_VER_23 23
#define BOARD_VER_26 26
#define BOARD_VER_27 27

#define CUSTOMER_BOARD_VER BOARD_VER_27

#define ENABLE_TAP_AND_HOLD 0
#define MAX_RAWDATA_TIME_STEP 35

#if (CUSTOMER_BOARD_VER <= BOARD_VER_11)
#define USING_LINEAR_MOTOR_0619
#define USING_BATTERY_ADC_LOW_ACCURACY
#define USING_BATTERY_200MAH
#define CHARGE_DETECT_PIN 114
#define WATCH_GSENSOR_POWER_EN (34)     // GPIO_B34
#define AMOLED_DISPLAY_3V3_EN (2) // PBR2
#define MOTOR_POWER_EN_PIN (-1)
#endif

#if (BOARD_VER_11 < CUSTOMER_BOARD_VER && CUSTOMER_BOARD_VER < BOARD_VER_15)
#define USING_LINEAR_MOTOR_0612
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define CHARGE_DETECT_PIN 114
#define WATCH_GSENSOR_POWER_EN (34) // GPIO_B34
#define PPG_POWER_EN_PIN (162)
// #define MOTOR_POWER_VCC_EN_PIN (162) // PBR2 (160+2=162)
// #define MOTOR_POWER_EN_PIN (115)     // PB19 (19+96=115)
#elif (CUSTOMER_BOARD_VER >= BOARD_VER_15 && CUSTOMER_BOARD_VER < BOARD_VER_17)
#define USING_LINEAR_MOTOR_0612
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define CHARGE_DETECT_PIN 129
#define IMU_INT_PIN 130
#define WATCH_GSENSOR_POWER_EN (2)      // PBR2
#define AMOLED_DISPLAY_3V3_EN (1) // PBR1
#define MOTOR_POWER_EN_PIN (-1)
#elif (CUSTOMER_BOARD_VER == BOARD_VER_17)
#define USING_LINEAR_MOTOR_0612
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define CHARGE_DETECT_PIN 121
#define IMU_INT_PIN 122
#define WATCH_GSENSOR_POWER_EN (2)      // PBR2
#define AMOLED_DISPLAY_3V3_EN (1) // PBR1
#define WATCH_DISPLAY_REVERSE_180
#define MOTOR_POWER_EN_PIN (-1)
#elif (CUSTOMER_BOARD_VER == BOARD_VER_18)
#define USING_LINEAR_MOTOR_0619
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define CHARGE_DETECT_PIN 121
#define IMU_INT_PIN 122
#define WATCH_GSENSOR_POWER_EN (2)      // PBR2
#define AMOLED_DISPLAY_3V3_EN (1) // PBR1
#define WATCH_DISPLAY_REVERSE_180
#define MOTOR_POWER_EN_PIN (-1)
#elif (CUSTOMER_BOARD_VER == BOARD_VER_19)
#define USING_LINEAR_MOTOR_0619
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define MOTOR_POWER_VCC_EN_PIN (162) // PBR2 (160+2=162)
#define MOTOR_POWER_EN_PIN (115)     // PB19 (19+96=115)
#define CHARGE_DETECT_PIN (121)      // PB25 (25+96=121)
#define EU_3V3_EN (33)               // PB33 (33+96=129)
#define AMOLED_BATTERY_EN (34)       // PB34 (34+96=130)
#define AMOLED_1V8_EN (21)           // PB21 (21+96=117)
#define AMOLED_DISPLAY_3V3_EN (0)    // PBR0 (160+0=160)
#define WATCH_GSENSOR_POWER_EN (22)        // PB22 (22+96=118)
#define PPG_POWER_EN_PIN (161)       // PBR1 (160+1=161)
#define IMU_INT_PIN (122)            // PB26 (26+96=122)
#define PPG_INT_PIN (114)            // PB18 (18+96=114)
#define PPG_RST_PIN (5)              // PA05
#define WATCH_DISPLAY_REVERSE_180
#elif (CUSTOMER_BOARD_VER >= BOARD_VER_20 && CUSTOMER_BOARD_VER < BOARD_VER_25)
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define CHARGE_DETECT_PIN (121)      // PB25 (25+96=121)
#define AMOLED_DISPLAY_3V3_EN (33)   // PB33 (33+96=129)
#define AMOLED_BATTERY_EN (34)       // PB34 (34+96=130)
#define AMOLED_1V8_EN (21)           // PB21 (21+96=117)
#define WATCH_GSENSOR_POWER_EN (22)        // PB22 (22+96=118)
// #define GSENSOR_VCC_EN_PIN (118)     // PB22 (22+96=118)
#define IMU_INT_PIN (122)            // PB26 (26+96=122)
#define PPG_INT_PIN (114)            // PB18 (18+96=114)
#define PPG_RST_PIN (115)            // PB19 (19+96=115)
#define PPG_POWER_EN_PIN (161)       // PBR1 (160+1=161)
#define MOTOR_POWER_VCC_EN_PIN (162) // PBR2 (160+2=162)
#define MOTOR_POWER_EN_PIN (5)       // PA05
#define USING_LINEAR_MOTOR_0619
#define WATCH_DISPLAY_REVERSE_180
#elif (CUSTOMER_BOARD_VER == BOARD_VER_26)
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define CHARGE_DETECT_PIN (121)      // PB25 (25+96=121)
#define AMOLED_DISPLAY_3V3_EN (33)   // PB33 (33+96=129)
#define AMOLED_BATTERY_EN (34)       // PB34 (34+96=130)
#define AMOLED_1V8_EN (21)           // PB21 (21+96=117)
#define WATCH_GSENSOR_POWER_EN (22)        // PB22 (22+96=118)
#define IMU_INT_PIN (122)            // PB26 (26+96=122)
#define PPG_INT_PIN (114)            // PB18 (18+96=114)
#define PPG_RST_PIN (162)            // PBR2 (160+2=162)
#define PPG_POWER_EN_PIN (161)       // PBR1 (160+1=161)
#define MOTOR_POWER_VCC_EN_PIN (115) // PB19 (19+96=115)
#define MOTOR_POWER_EN_PIN (5)       // PA05
#define USING_LINEAR_MOTOR_0619
#define WATCH_IMU_REVERSE_180 (1)
#elif (CUSTOMER_BOARD_VER == BOARD_VER_27)
#define USING_BATTERY_ADC_HIGH_ACCURACY
#define USING_BATTERY_300MAH
#define CHARGE_DETECT_PIN (130)    // PB34 (34+96=130)
#define AMOLED_DISPLAY_3V3_EN (33) // PB33 (33+96=129)
#define AMOLED_1V8_EN (21)         // PB21 (21+96=117)
#define WATCH_GSENSOR_POWER_EN (22)      // PB22 (22+96=118)
#define IMU_INT_PIN (122)          // PB26 (26+96=122)
#define PPG_INT_PIN (114)          // PB18 (18+96=114)
#define PPG_RST_PIN (115)          // PB19 (19+96=115)
#define PPG_POWER_EN_PIN (161)     // PBR1 (160+1=161)
#define MOTOR_POWER_EN_PIN (5)     // PA05
#define RGB_LED_CONTROL_PIN (121)   // PB25 (25+96=121)
#define WS2812B_TIMING
#define USING_LINEAR_MOTOR_0619
#define WATCH_DISPLAY_REVERSE_180
// #define WATCH_IMU_REVERSE_180 (1)
#endif

// #define WATCH_DISPLAY_REVERSE_180
// #define RELEASE_WATCH
#define PWM_LRA_MOTOR // Linear Resonant Actuator (LRA) Driver IC
#define REAL_TIME_IMU_DATA_COLLECTION
#define REAL_TIME_ACCEL_STEPS_COLLECTION

#define SHOW_TAP_GESTURE_INDICATOR
#define SHOW_UNKNOWN_GESTURE_INDICATOR
// #define SHOW_UNGRAB_ENABLFE_INDICATOR
// #define SHOW_BAD_SIGNAL_INDICATOR
// #define SHOW_OPEN_WATCH_HINT_LIGHT

#define ENABLE_OPUS_ENCODER

#endif // CUSTOMER_BOARD

#define PERIPHERAL_AUD_SPEAKER 0

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
