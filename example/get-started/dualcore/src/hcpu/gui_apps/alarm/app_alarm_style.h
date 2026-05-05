/**
 ******************************************************************************
 * @file   app_alarm_style.h
 * @author Skaiwalk software development team
 * @brief  Apple-Watch-style design tokens shared across the alarm app pages.
 ******************************************************************************
 */
#ifndef __APP_ALARM_STYLE_H__
#define __APP_ALARM_STYLE_H__

#include "lvgl.h"

/* Colors — borrowed from iOS / Apple Watch dark mode. */
#define ALARM_COLOR_BG              lv_color_hex(0x000000)
#define ALARM_COLOR_CARD            lv_color_hex(0x1C1C1E)
#define ALARM_COLOR_DIVIDER         lv_color_hex(0x2C2C2E)
#define ALARM_COLOR_NEUTRAL         lv_color_hex(0x2C2C2E)
#define ALARM_COLOR_TEXT_PRIMARY    lv_color_hex(0xFFFFFF)
#define ALARM_COLOR_TEXT_SECONDARY  lv_color_hex(0x8E8E93)
#define ALARM_COLOR_TEXT_DIM        lv_color_hex(0x6E6E73)
#define ALARM_COLOR_SWITCH_ON       lv_color_hex(0x34C759)  /* iOS green */
#define ALARM_COLOR_SWITCH_OFF      lv_color_hex(0x39393D)
#define ALARM_COLOR_ACCENT          lv_color_hex(0xFF9F0A)  /* iOS Alarm orange */
#define ALARM_COLOR_DANGER          lv_color_hex(0xFF453A)  /* iOS red */

/* Common geometry. */
#define ALARM_LIST_W                420
#define ALARM_CARD_RADIUS           16

#endif /* __APP_ALARM_STYLE_H__ */
