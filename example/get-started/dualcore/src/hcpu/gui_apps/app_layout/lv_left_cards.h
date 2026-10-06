/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * 左頁整頁彩色卡片(founder 2026-09-30:「照整頁彩色卡片做」,對照小米 Watch S5 智能助理):
 * 一張卡 = 整個螢幕、一件事,直向翻頁;底色跟著類型;左上圖示+大標題、中間最多三行說明、
 * 底部一顆膠囊按鈕;點卡或按鈕 = 進 app / 執行(由呼叫端決定,這個模組不認識清單)。
 *
 * 這個模組只管「畫卡+翻頁」:資料由呼叫端按需提供(left_cards_get_cb_t,要畫哪張才問哪張,模組自己
 * 不存任何一張的內容),點擊/換頁走回呼。
 * 內容(圖示/標題/說明/按鈕)只為目前這張與前後各一張建立,其餘只留一個空底板,heap 吃緊
 * (R31~R33)所以物件數要有上限。
 */
#ifndef LV_LEFT_CARDS_H
#define LV_LEFT_CARDS_H

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"

#define LEFT_CARDS_MAX 30

typedef struct
{
    const char *title;  /* 大標題 */
    const char *sub;    /* 說明,"" = 沒有(標題會往下置中一點) */
    const void *icon;   /* lv_img 來源(檔案路徑或 dsc);NULL = 不畫圖示 */
    uint32_t accent;    /* 0xRRGGBB:這張卡的底色(頂端);底端自動壓暗 */
    /* AI 通知帶來的選項(最多 3 個):直接畫成卡片上的選項晶片,點了回覆那則通知。卡片沒有底部按鈕
       (founder 2026-10-01:「app 字卡下面不需要有開啟的按鈕」)—— 點卡片本身就是進 app / 執行。 */
    const char *opts[3];
    uint8_t n_opts;
    /* 「大字 + 步進 + 主動作」版面(計時器,founder 2026-10-06 選的 D 版):big != NULL 時不畫說明與選項晶片,
       改畫中間大字 big、(arrows)左右兩個步進箭頭、底下一顆實心大膠囊 act。點擊走選項回呼:
       opt 0 = act 膠囊、1 = 左箭頭、2 = 右箭頭。big 指到的字要在這次呼叫期間有效(可以用 subbuf)。 */
    const char *big;
    const char *act;
    bool arrows;
} left_card_t;

/* 卡片內容按需提供(founder 2026-10-03 要省 SRAM:不再常駐一份 30 張的陣列 + 每張 96B 的即時文字緩衝):
   模組要畫/量某一張才問呼叫端一次。subbuf 是呼叫端可以拿來組即時文字的暫存(LEFT_CARD_SUB_BUF 位元組,
   只在這次呼叫期間有效 —— 模組馬上把文字複製進 label);title/icon/opts 要指向呼叫端自己保證穩定的位置。
   回傳 false = 沒有這張。 */
#define LEFT_CARD_SUB_BUF 96
typedef bool (*left_cards_get_cb_t)(uint8_t idx, left_card_t *out, char *subbuf);
typedef void (*left_cards_tap_cb_t)(uint8_t idx);
typedef void (*left_cards_page_cb_t)(uint8_t idx);
/* 翻頁途中每一幀回報捲動位置:page_x256 = 目前捲到第幾頁 × 256(定點小數,第 2.5 頁 = 640)。
   停穩時也會用整數頁再回報一次。右緣的點點輪盤靠它連續轉動。 */
typedef void (*left_cards_scroll_cb_t)(int32_t page_x256);
/* 點了第 card 張卡上的第 opt 個選項晶片。 */
typedef void (*left_cards_option_cb_t)(uint8_t card, uint8_t opt);

/* 在 parent 底下建立(或重建)整組卡片,停在第 start 張。已存在則先拆掉。 */
lv_obj_t *left_cards_show(lv_obj_t *parent, left_cards_get_cb_t get, uint8_t n, uint8_t start,
                          left_cards_tap_cb_t on_tap, left_cards_page_cb_t on_page,
                          left_cards_scroll_cb_t on_scroll, left_cards_option_cb_t on_option);
void left_cards_hide(void);
bool left_cards_visible(void);
uint8_t left_cards_current(void);
/* 捲到第 idx 張(右緣圓弧撥動換到另一張時用);anim=true 走 LVGL 的捲動動畫,停穩時照常觸發換頁回呼。 */
void left_cards_scroll_to(uint8_t idx, bool anim);
/* 手指正按著或還在慣性捲動 —— 呼叫端要重建資料時先等它停,否則畫面會抖。 */
bool left_cards_busy(void);
/* 浮層水平滑入/滑出時呼叫(tx = 浮層的 translate_x,0 = 完全就位):滑動中把翻頁容器裁成圓形,
   前緣就是一道圓弧(小米的進場長這樣,不是一條直邊);就位後還原成不裁,省掉靜止時的遮罩成本。 */
void left_cards_set_slide(lv_coord_t tx);
/* 說明文字之類的內容變了、但卡片數/標題/底色沒變:只重畫已建好的那幾張,不動翻頁容器(不閃、不重置位置)。
   呼叫端要先確認 left_cards_busy() 為假。 */
void left_cards_refresh(void);
/* obj 是翻頁容器本身或它底下的物件(事件濾除用)。 */
bool left_cards_owns(lv_obj_t *obj);

#endif
