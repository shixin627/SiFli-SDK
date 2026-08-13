/**
 * @file    gesture_debug_led.c
 * @brief   兩顆手勢診斷燈的實作。設計說明見 gesture_debug_led.h。
 */

#include "gesture_debug_led.h"

#if GESTURE_DEBUG_LED_ENABLE

#include "lvgl.h"
#include <rtthread.h>

#define DBG_TAG "gesture.led"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* ---- 外觀 ---- */

#define GLED_DOT_SIZE 20   /* 直徑 px */
#define GLED_DOT_GAP 16    /* 兩顆之間的間距 */
#define GLED_BOTTOM_OFS -38 /* 距底部;466 圓螢幕邊緣要留白 */

#define GLED_COLOR_OFF 0x141414     /* 熄滅:看得到位置但不搶戲 */
#define GLED_COLOR_CAPTURE 0xFFFFFF /* 燈 1 亮:白 */
#define GLED_COLOR_RELEASE 0x2E7BFF /* 燈 2:藍 = release */
#define GLED_COLOR_TAP 0x27D07C     /* 燈 2:綠 = tap */

#define GLED_OPA_OFF LV_OPA_30
#define GLED_OPA_ON LV_OPA_COVER

/* ---- 亮燈時間 ----
   擷取只是一瞬間的事件,不撐長一點根本看不到。燈 2 撐得比燈 1 久,這樣「燈 1 亮
   了、燈 2 沒亮」這個關鍵組合不會因為兩顆熄滅時間一樣而看不出先後。 */
#define GLED_CAPTURE_HOLD_MS 350
#define GLED_VERDICT_HOLD_MS 800

/* 更新頻率。純讀 volatile + 必要時改 style,很便宜。 */
#define GLED_POLL_MS 50

/* ---- 跨執行緒狀態 ----
   producer(motion / gesture-recognition 執行緒)只寫這裡;consumer(LVGL 執行緒
   的 timer)只讀。沒有 lock:每個欄位都是單字組寬度的 volatile,而且最壞情況只是
   某一幀的燈號晚 50 ms —— 診斷燈不值得為此扛一把鎖進 IMU 熱路徑。 */
static volatile uint32_t s_capture_tick = 0; /* 0 = 從未發生 */
static volatile uint32_t s_verdict_tick = 0;
static volatile uint8_t s_verdict = GESTURE_LED_VERDICT_NONE;

static bool s_enabled = true;

/* ---- LVGL 端(只有 LVGL 執行緒碰) ---- */

static lv_obj_t *s_dot_capture = NULL;
static lv_obj_t *s_dot_verdict = NULL;
static lv_timer_t *s_timer = NULL;

/* 上次套用的顏色,用來避免每 50 ms 重寫 style + 重畫 */
static uint32_t s_applied_capture = GLED_COLOR_OFF;
static uint32_t s_applied_verdict = GLED_COLOR_OFF;

static void gled_apply(lv_obj_t *dot, uint32_t *applied, uint32_t color)
{
    if (*applied == color)
        return;
    lv_obj_set_style_bg_color(dot, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(dot, (color == GLED_COLOR_OFF) ? GLED_OPA_OFF
                                                           : GLED_OPA_ON, 0);
    *applied = color;
}

/* tick 期限判斷。rt_tick 會回繞,所以用有號差值比較,不要直接比大小。 */
static bool gled_within(uint32_t stamp, uint32_t hold_ms)
{
    if (stamp == 0)
        return false;
    return (int32_t)(rt_tick_get() - stamp) < (int32_t)rt_tick_from_millisecond(hold_ms);
}

static void gled_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);

    if (!s_dot_capture || !s_dot_verdict)
        return;

    uint32_t want_capture = gled_within(s_capture_tick, GLED_CAPTURE_HOLD_MS)
                                ? GLED_COLOR_CAPTURE
                                : GLED_COLOR_OFF;

    uint32_t want_verdict = GLED_COLOR_OFF;
    if (gled_within(s_verdict_tick, GLED_VERDICT_HOLD_MS))
    {
        /* 讀一次就好:producer 是先寫 verdict 再寫 tick,所以看到新 tick 時
           verdict 必定已經是對應的那一筆。 */
        switch (s_verdict)
        {
        case GESTURE_LED_VERDICT_RELEASE:
            want_verdict = GLED_COLOR_RELEASE;
            break;
        case GESTURE_LED_VERDICT_TAP:
            want_verdict = GLED_COLOR_TAP;
            break;
        default:
            want_verdict = GLED_COLOR_OFF; /* 認不出來 → 不亮,這就是重點 */
            break;
        }
    }

    bool changed = (want_capture != s_applied_capture) ||
                   (want_verdict != s_applied_verdict);
    if (!changed)
        return;

    gled_apply(s_dot_capture, &s_applied_capture, want_capture);
    gled_apply(s_dot_verdict, &s_applied_verdict, want_verdict);

    /* 顯示調速器閒置時會把 refr timer 降到 1 Hz,不強制刷新的話燈號可能晚一秒
       才出現 —— 對一顆用來看時序的燈來說那等於沒用。這裡就在 LVGL 執行緒上
       (lv_timer 回呼),lv_refr_now() 呼叫是合法的;而且只在狀態改變時做,一次
       手勢頂多幾次。 */
    lv_refr_now(lv_disp_get_default());
}

static lv_obj_t *gled_make_dot(lv_obj_t *parent, lv_coord_t x_ofs)
{
    lv_obj_t *dot = lv_obj_create(parent);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, GLED_DOT_SIZE, GLED_DOT_SIZE);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(GLED_COLOR_OFF), 0);
    lv_obj_set_style_bg_opa(dot, GLED_OPA_OFF, 0);
    lv_obj_align(dot, LV_ALIGN_BOTTOM_MID, x_ofs, GLED_BOTTOM_OFS);
    /* 純顯示:不可點、不吃捲動,免得從畫面最底部偷走任何手勢。 */
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    return dot;
}

void gesture_led_init(void)
{
    if (s_dot_capture)
        return;

    lv_obj_t *layer = lv_layer_sys();
    if (!layer)
    {
        LOG_W("no sys layer yet; gesture LED init skipped");
        return;
    }

    lv_coord_t half = (GLED_DOT_SIZE + GLED_DOT_GAP) / 2;
    s_dot_capture = gled_make_dot(layer, -half);
    s_dot_verdict = gled_make_dot(layer, half);

    if (!s_enabled)
    {
        lv_obj_add_flag(s_dot_capture, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_dot_verdict, LV_OBJ_FLAG_HIDDEN);
    }

    s_timer = lv_timer_create(gled_timer_cb, GLED_POLL_MS, NULL);
    LOG_I("gesture debug LEDs up (capture | verdict)");
}

void gesture_led_notify_capture(void)
{
    if (!s_enabled)
        return;
    /* rt_tick_get() 可能回傳 0(開機瞬間),而 0 是我們的「從未發生」哨兵值,
       所以夾到 1。 */
    uint32_t now = rt_tick_get();
    s_capture_tick = now ? now : 1u;
}

void gesture_led_notify_verdict(gesture_led_verdict_t verdict)
{
    if (!s_enabled)
        return;
    /* 順序有意義:先寫 verdict 再寫 tick,consumer 才不會看到新 tick 配舊判決。 */
    s_verdict = (uint8_t)verdict;
    uint32_t now = rt_tick_get();
    s_verdict_tick = now ? now : 1u;
}

void gesture_led_set_enabled(bool enabled)
{
    s_enabled = enabled;
    if (!s_dot_capture)
        return;
    /* 只有 LVGL 執行緒能改顯示。MSH 是 tshell 執行緒,所以這裡只動 flag 是不夠
       安全的 —— 但 lv_obj_add_flag/clear_flag 在這個專案的既有 MSH 指令裡到處
       都是,維持一致;真要嚴謹應該改走 lvgl_send_msg。 */
    if (enabled)
    {
        lv_obj_clear_flag(s_dot_capture, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_dot_verdict, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(s_dot_capture, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_dot_verdict, LV_OBJ_FLAG_HIDDEN);
    }
}

bool gesture_led_is_enabled(void)
{
    return s_enabled;
}

/* ---- MSH(dev build 才有;燈本身在 release build 也會亮) ---- */
#if defined(RT_USING_FINSH) && !kReleaseMode
#include <finsh.h>

static void gled(int argc, char **argv)
{
    if (argc >= 2)
    {
        if (rt_strcmp(argv[1], "on") == 0)
            gesture_led_set_enabled(true);
        else if (rt_strcmp(argv[1], "off") == 0)
            gesture_led_set_enabled(false);
        else if (rt_strcmp(argv[1], "test") == 0)
        {
            /* 兩顆燈都點一次,確認位置與顏色 */
            gesture_led_notify_capture();
            gesture_led_notify_verdict(argc >= 3 && rt_strcmp(argv[2], "tap") == 0
                                           ? GESTURE_LED_VERDICT_TAP
                                           : GESTURE_LED_VERDICT_RELEASE);
        }
        else
        {
            rt_kprintf("usage: gled [on|off|test [tap|release]]\n");
            return;
        }
    }
    rt_kprintf("gled: enabled=%d  cap_tick=%u verdict=%u verdict_tick=%u\n",
               s_enabled, (unsigned)s_capture_tick, (unsigned)s_verdict,
               (unsigned)s_verdict_tick);
}
MSH_CMD_EXPORT(gled, "gesture debug LEDs: gled [on|off|test [tap|release]]");

#endif /* RT_USING_FINSH && !kReleaseMode */

#endif /* GESTURE_DEBUG_LED_ENABLE */
