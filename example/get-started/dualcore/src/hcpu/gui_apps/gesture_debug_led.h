/**
 * @file    gesture_debug_led.h
 * @brief   兩顆手勢診斷燈（螢幕底部，疊在 lv_layer_sys 上，任何頁面都看得到）。
 *
 *   燈 1（左）：白 = stage 1 切出視窗並交給 stage 2。
 *               紅 = stage 2 收到了，但在跑模型之前就被 gate 攔掉。
 *               不亮 = IMU 那層根本沒擷取到東西。
 *   燈 2（右）：白燈時 = 模型判決:藍 = release、綠 = tap、不亮 = 模型不認。
 *               紅燈時 = 是哪一道 gate 攔的（顏色見 gesture_led_gate_t）。
 *
 * 「擷取到但沒判決」有兩種完全不同的原因——模型跑了不認、或根本沒跑到模型——
 * 修法天差地遠。燈 1 的白/紅就是把這條線畫出來，燈 2 再告訴你是哪一道 gate。
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

/* stage 2 在跑模型之前的攔截點。燈 1 轉紅，燈 2 用顏色說是哪一道。 */
typedef enum
{
    GESTURE_LED_GATE_GUI_OFF = 0, /* 青  — 螢幕關著（正常,看不到） */
    GESTURE_LED_GATE_NOT_WORN,    /* 黃  — 配戴偵測說沒戴 */
    GESTURE_LED_GATE_TOUCHING,    /* 橙  — is_user_touching_screen() 為真 */
    GESTURE_LED_GATE_MOTOR,       /* 紫  — 馬達還在震 */
    GESTURE_LED_GATE_OTHER,       /* 白  — model-off / mouse-mode / arm-control */
} gesture_led_gate_t;

#if GESTURE_DEBUG_LED_ENABLE

/** 建立兩顆燈並啟動更新 timer。**只能在 LVGL 執行緒呼叫**（ui_layer_system_builder）。 */
void gesture_led_init(void);

/** stage 1 切出一個視窗、已交給 stage 2。任何執行緒可呼叫。 */
void gesture_led_notify_capture(void);

/** stage 2 在跑模型前把視窗攔掉了。任何執行緒可呼叫。 */
void gesture_led_notify_gate(gesture_led_gate_t gate);

/** stage 2 的模型判決。任何執行緒可呼叫。 */
void gesture_led_notify_verdict(gesture_led_verdict_t verdict);

/** 執行期開關（預設 ON）。關掉時兩顆燈隱藏。 */
void gesture_led_set_enabled(bool enabled);
bool gesture_led_is_enabled(void);

#else /* !GESTURE_DEBUG_LED_ENABLE — 編譯掉 */

static inline void gesture_led_init(void) {}
static inline void gesture_led_notify_capture(void) {}
static inline void gesture_led_notify_gate(gesture_led_gate_t g) { (void)g; }
static inline void gesture_led_notify_verdict(gesture_led_verdict_t v) { (void)v; }
static inline void gesture_led_set_enabled(bool enabled) { (void)enabled; }
static inline bool gesture_led_is_enabled(void) { return false; }

#endif /* GESTURE_DEBUG_LED_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __GESTURE_DEBUG_LED_H__ */
