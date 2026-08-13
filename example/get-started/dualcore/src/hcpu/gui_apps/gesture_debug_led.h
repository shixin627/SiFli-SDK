/**
 * @file    gesture_debug_led.h
 * @brief   兩顆手勢診斷燈（螢幕底部，疊在 lv_layer_sys 上，任何頁面都看得到）。
 *
 *   燈 1（左）：stage 1 切出一個視窗並交給 stage 2 時亮白燈。
 *               不亮 = IMU 那層根本沒擷取到東西（姿態 gate / 冷卻 / 門檻沒過）。
 *   燈 2（右）：stage 2 的模型判決。
 *               藍 = release、綠 = tap、不亮 = 擷取到了但模型沒認出東西。
 *
 * 用途:分辨「手勢沒被擷取」與「擷取到但被攔截/不認」——這兩件事在 log 上長得
 * 很像，但修法完全不同。
 *
 * 執行緒約定（重要）:
 *   - gesture_led_notify_*() 由 motion-tracking / gesture-recognition 執行緒呼叫，
 *     裡面**只寫 volatile 變數，不碰任何 LVGL**。
 *   - 所有 LVGL 操作都在 gesture_led_init() 建立的 lv_timer 裡做，也就是 LVGL
 *     執行緒。這是刻意的:這條路徑上跨執行緒直接呼叫 LVGL 正是我們在追的那類
 *     bug，診斷工具自己不能再犯一次。
 *
 * 整包由 GESTURE_DEBUG_LED_ENABLE 控制，設 0 即完全編譯掉（所有呼叫變成 no-op）。
 * 刻意**不**綁 kReleaseMode —— release 版才是要抓這個問題的地方。
 */
#ifndef __GESTURE_DEBUG_LED_H__
#define __GESTURE_DEBUG_LED_H__

#include <stdbool.h>
#include <stdint.h>

/* Master compile guard. Define to 0 in a board config to strip it entirely. */
#ifndef GESTURE_DEBUG_LED_ENABLE
    #define GESTURE_DEBUG_LED_ENABLE 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    GESTURE_LED_VERDICT_NONE = 0, /* 模型跑了但沒認出來 → 燈 2 不亮 */
    GESTURE_LED_VERDICT_RELEASE,  /* → 燈 2 藍 */
    GESTURE_LED_VERDICT_TAP,      /* → 燈 2 綠 */
} gesture_led_verdict_t;

#if GESTURE_DEBUG_LED_ENABLE

/** 建立兩顆燈並啟動更新 timer。**只能在 LVGL 執行緒呼叫**（ui_layer_system_builder）。 */
void gesture_led_init(void);

/** stage 1 切出一個視窗、已交給 stage 2。任何執行緒可呼叫。 */
void gesture_led_notify_capture(void);

/** stage 2 的模型判決。任何執行緒可呼叫。 */
void gesture_led_notify_verdict(gesture_led_verdict_t verdict);

/** 執行期開關（預設 ON）。關掉時兩顆燈隱藏。 */
void gesture_led_set_enabled(bool enabled);
bool gesture_led_is_enabled(void);

#else /* !GESTURE_DEBUG_LED_ENABLE — 編譯掉 */

static inline void gesture_led_init(void) {}
static inline void gesture_led_notify_capture(void) {}
static inline void gesture_led_notify_verdict(gesture_led_verdict_t v) { (void)v; }
static inline void gesture_led_set_enabled(bool enabled) { (void)enabled; }
static inline bool gesture_led_is_enabled(void) { return false; }

#endif /* GESTURE_DEBUG_LED_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __GESTURE_DEBUG_LED_H__ */
