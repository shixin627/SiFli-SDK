/**
 ******************************************************************************
 * @file   lv_instruction_list_layout.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "lvgl.h"
#include "lv_qrcode.h"
#include "lv_simplified_obj.h"
#include "lv_ext_resource_manager.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "arc_scroll.h"
#include "watch_system_interact.h"
#include "custom_trans_anim.h"
#include <math.h>
#include "ui_helper.h"
#include "ui_img_helper.h"
#include "lv_chat_page.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#include "ui_handler.h"
#ifdef BSP_USING_GESTURE_HANDLER
    #include "gesture_handler.h"
#endif
#include "watch_global_data.h"
#ifdef BSP_USING_BLOC
    #include "bloc_v2t.h"
    #include "bloc_peripheral.h"
    #include "bloc_weather.h"
    #include "bloc_calendar.h"
    #include "bloc_motion_tracking.h"
#endif
#include "communicate_protocol.h"
#include "communicate_task.h"
#include <cJSON.h>

LV_IMG_DECLARE(voice_group);
LV_IMG_DECLARE(menu_icon);
LV_IMG_DECLARE(plus);
LV_IMG_DECLARE(icon_mic);
LV_IMG_DECLARE(message_widget_bg);
LV_IMG_DECLARE(skaibar_img); /* 176x31 — the bottom mic bar's resting look (ezip, auto-built) */
LV_IMG_DECLARE(micro_icon);
LV_IMG_DECLARE(micro_open_icon); /* bar 長按語音進行中的淺藍 mic(同 hid_mouse V2T active) */  /* shared mic glyph — the bottom trigger's NEW resting look (matches the chat page) */
LV_IMG_DECLARE(img_logo);  /* 80x80 — 立起輸入面板上方「送給 skaibar」 */
LV_IMG_DECLARE(icon_send); /* 34x34 — 立起輸入面板上方「送回剛剛點的輸入框」 */
LV_IMG_DECLARE(backspace_icon); /* 46x33 — 立起輸入面板下方刪除鍵(沿用滑鼠 app 鍵盤同一顆) */
/* 框裡沒字時刪除鍵改當退出鍵(founder 2026-08-01)。沿用滑鼠 app 輸入框下方那顆「收回」的
   同一張圖 —— 同一個手勢語彙不要在兩個畫面用兩種圖示。 */
LV_IMG_DECLARE(down_arrow);
/* 滑鼠 app 單設備抽屜底部那排最左邊的「切輸入法」鍵(founder 2026-08-17 指名地球圖)。
   與鍵盤輪盤 row4 的 mode_btn 同一張,語彙一致。 */
LV_IMG_DECLARE(erth);

#define DBG_TAG "instruction.list.layout"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define APP_ID "instruction_list"
#include <stdio.h>
#include <stdint.h>

/*******************************************************************************
 * Unified List Item
 ******************************************************************************/
#define MAX_LIST_ITEMS 30
#define LIST_ITEM_ID_LEN 64
#define LIST_ITEM_TITLE_LEN 64

typedef struct
{
    char id[LIST_ITEM_ID_LEN]; // app_id string or instruction UUID
    char title[LIST_ITEM_TITLE_LEN];
    const char *icon;      // icon resource pointer, can be NULL
    char img_path[64];     // file-based image path for instructions
    /* R44:這張圖的檔案內容真的換過(update_instruction_image),下次 refresh 要丟掉它的
       解碼快取。手機每次開清單都會 replace-all 重推一份**內容相同**的清單,那種情況不
       該丟 —— 丟了就得重讀 NAND 重解碼,使用者看到「文字先進來、右邊圖標晚一拍」。 */
    bool img_dirty;
    lv_obj_t *widget;      // app widget obj, NULL for instructions
    bool is_instruction;   // true = custom instruction, false = app
    bool is_interval;      // for instructions: has toggle switch
    bool enabled;          // toggle state
    uint32_t interval_sec; // intervalSeconds
    char trigger_type[32]; // e.g. "interval", "once", etc.
    uint32_t version;      // version from server
    char open_app[32];     // non-empty = tap opens this watch app locally
                           // (offline, no phone relay); see openApp in 0x65/0x6B
    char category;         // '@'=chat / '/'=action / 0=untagged. Drives the
                           // left(@) / right(/) / bottom(all) filtered list
                           // views. Set only via the phone-push "cat" field
                           // (0x65/0x6B); built-in apps & device_pager items
                           // stay untagged (treated as action in the "/" view).
} list_item_t;

static list_item_t list_items[MAX_LIST_ITEMS];
/* R3: set by the explicit CANCEL-close paths (bar tap / swipe-right) right before
   they call close_ai_widget, so close_ai_widget tells the phone to dismiss the
   active skaibar. A COMMIT close (tapping an option) leaves this false, so the
   phone keeps its normal "run action + return to list" flow. Consumed (reset) at
   the top of close_ai_widget. */
static bool s_close_is_cancel = false;
static uint8_t list_item_count =
    0; // total count of all items (app + instructions)
static uint8_t app_base_count =
    0; // pinned prefix protected from clear. Stays 0 since the index-0 Settings
       // pin was removed (2026-07-02); the prefix machinery is kept for the
       // device_pager save/feed/restore bracket.

/* Phone-coupled list mode (founder direction 2026-06-04):
   DEFAULT_APPS — no phone list yet (boot / never-connected): body [1..] holds the
                  watch's built-in apps, treated as default "instructions".
   PHONE        — the phone pushed its own list (0x65/0x6B): the default apps were
                  replaced; body [1..] holds the phone instructions. Only a watch
                  reboot returns to DEFAULT_APPS (RAM-only, no flash). */
typedef enum
{
    LIST_MODE_DEFAULT_APPS = 0,
    LIST_MODE_PHONE,
} list_mode_t;
static list_mode_t s_list_mode = LIST_MODE_DEFAULT_APPS;

/* Phone link state mirrored from the BT connection hook. Only meaningful in
   PHONE mode: when disconnected we render the phone list filtered to the items
   the watch can complete on its own (apps + openApp instructions), hiding the
   phone-relay ones — without dropping them, so reconnect restores the full list
   even if the phone does NOT re-push. */
static bool s_phone_connected = false;
/* Full PHONE-mode list snapshot, held only while disconnect-filtered. Heap-backed
   (rt_malloc'd, freed on reconnect / fresh push) to avoid a permanent duplicate,
   same idiom as instruction_list_save_base. NULL = not currently filtered. */
static list_item_t *s_disc_backup = NULL;
static uint8_t s_disc_backup_count = 0;

/* P2 S2 — transient @-chat / /-action view filter. Same compaction idiom as the
   disconnect filter above (list_items[] is BOTH source and displayed list, and
   create_list_items_ui / create_indicator_dots index it by position — a subset
   view must be a contiguous re-pack; a render-time skip would leave layout gaps).
   TRANSIENT: live only while a filtered browse view is up, dropped by any
   structural op (phone push / disconnect change) and re-derived on the next open.
   s_cat_filter: 0 = all, '@' = chat, '/' = action (untagged counts as action). */
static char s_cat_filter = 0;
/* Pre-filter full-list backup. WAS rt_malloc'd per reveal (sizeof(list_items) ≈ 8KB);
   under heap pressure that allocation could FAIL, and the old code then silently
   skipped the re-pack — so the @ / and main views all showed the SAME full list (the
   "no filtering at all" bug). A fixed BSS buffer makes the filter deterministic: it
   never hinges on a large transient malloc succeeding. s_cat_backup_valid replaces the
   old "s_cat_backup != NULL" filtered-state flag. */
static list_item_t s_cat_backup[MAX_LIST_ITEMS];
static uint8_t s_cat_backup_count = 0;
static bool s_cat_backup_valid = false;
/* R55(founder 2026-08-13):左頁的麥克風 = 先開輸入框,說出來的字**即時篩上面的清單**
   (actions 的標題 + 各設備的 session 標題);全部篩掉了才變成「開新 session」那一個選項。
   這是**第二層**檢視篩選,跟 s_cat_filter 疊在一起用同一份 s_cat_backup 打包 —— 另外開
   一份備份是 sizeof(list_items) ≈ 8KB 的 BSS,而這台的 heap 已經吃緊(進滑鼠頁的大配置
   失敗就 NULL deref)。空字串 = 不篩。 */
static char s_text_filter[48] = {0};
static void pack_text_filter(void); /* 定義在後段(要用 item/字串輔助函式);refresh 出口呼叫 */
static bool search_freeze_active(void); /* 搜尋期間清單凍結(R60) */
/* 篩不到任何東西時補上的合成列:點它 = 用目前控制中的那台開新 session。id 用一個
   list_items[] 不可能出現的字串(手機 action id / conv:<id> 都不會長這樣)。 */
#define NEW_SESSION_ITEM_ID "\x01newsession"
/* The @/-view the user is CURRENTLY looking at ('@' / '/' / 0=all). Distinct from
   s_cat_filter (the filter physically applied to list_items[] right now, which a phone
   push transiently lifts to 0 via cat_filter_restore_full). s_view_cat PERSISTS across a
   push so apply_pending_instruction_batch can re-apply the view afterwards — fixing "a
   push that lands while the @/-list is open reverts it to all". Set on reveal (every
   set_category_filter), cleared when the list slides shut. */
static char s_view_cat = 0;
/* SINGLE-DEVICE skaibar mode for the STANDALONE mouse app (APP_ID_MOUSE). Set by
   instruction_list_bar_tap_device(): the list shows ONE controlled device's options (fed
   from the E7 registry, not the watch-face / aggregated list) and the round-trip is
   single-target (settle sends commu_send_skaibar_open_device, NOT the aggregated
   commu_send_skaibar_view broadcast). Cleared + watch-face list restored when the list
   slides shut (inst_list_slide_out_done_cb) or the mouse app dismisses it on exit. */
static bool s_bar_single_device = false;
/* Does the box the standalone mouse app (APP_ID_MOUSE) is controlling RIGHT NOW have a
   genuinely focused native text input? Pushed down by the phone (KEY_REMOTE_TEXT_FOCUS 0x17,
   see communicate_parse_skailink.c) edge-triggered after every relayed click, so this is a
   cached "last known" value — a bar-tap reads it with zero round-trip latency, never queries
   on demand. Consumed only by instruction_list_bar_tap_device(): true skips the option list
   and opens the input box directly (the user almost certainly just clicked into that field);
   false (the default) keeps the original list-first two-tap flow. Reset on
   instruction_list_bar_device_dismiss() so a stale true never leaks into the next
   device/session — see instruction_list_set_remote_target_focus(). */
static bool s_remote_target_has_focus = false;
/* Which filter the in-flight reveal will land in, applied once at settle
   (reveal_settle_browse_done_cb). Set per entry: bar / IMU browse = 0 (all),
   right-edge pull = '/', left-edge pull = '@'. */
static char s_pending_reveal_filter = 0;
/* Which side the in-flight reveal drag parks on: true = left edge (@ list, parked
   at -HOR_RES, finger-follows rightward toward 0), false = right edge (/ list, or
   the programmatic bar/IMU open, parked at +HOR_RES). Set at PRESSED by each edge
   overlay; consulted by reveal_drag_begin/update/end + dial_blur_track so the same
   list object mirrors in from either edge. */
static bool s_reveal_from_left = false;
static void cat_filter_restore_full(void);
static void instruction_list_set_category_filter(char cat);

/* Callback: tapped or toggled. Receives id string and enabled state. */
static void (*instruction_tap_cb)(const char *id, bool enabled) = NULL;

static lv_obj_t *switch_objs[MAX_LIST_ITEMS]; // toggle switches for any item

#define LIST_ITEM_WIDTH (80)
#define LIST_ITEM_HEIGHT (80)
#define LIST_ITEM_SPACING (-100)
#define LIST_ITEM_BTN_WIDTH (150)
#define LIST_ITEM_BTN_HEIGHT (150)
#define LIST_ITEM_WIDGET_HEIGHT (200)
#define LIST_ITEM_WIDGET_WIDTH (430)

#define LIST_RADIUS (466)
#define LIST_ADJUST_X_POSITION (-466)

#define MOVABLE_ARC_RADIUS (233)
#define MOVABLE_ARC_CENTER_X (233)
#define MOVABLE_ARC_CENTER_Y (233)
#define MOVABLE_ARC_START_ANGLE (-15)
#define MOVABLE_ARC_END_ANGLE (15)
#define MOVABLE_ARC_WIDTH (3)

#define LIST_ITEM_RADIUS (240)
#define LIST_ITEM_BORDER_SIDE LV_BORDER_SIDE_RIGHT

#define DOT_SMOLL_PROPORTION (0.5)
#define DOT_BIG_PROPORTION (1.3)
#define DOT_BG_SIZE (100 * DOT_BIG_PROPORTION) + 2
/* Icon-only shrink factor (1.0 = original). Applied to the dot zoom but NOT to
 * DOT_BG_SIZE / arc positions, so the icon images shrink to 80% in place while
 * the carousel spacing/layout stays identical. */
#define DOT_ICON_SCALE 0.8f
/* 縮放曲線指數：1.0 = 線性、2.0 = 平方（中央放大效果突出，邊緣下降快）、
 * 3.0 = 立方（更陡峭）。值越大，「中央 dot 顯著大、其他 dot 都很小」越明顯 */
#define DOT_ZOOM_EXPONENT 2.0f
/* R29:session 列右緣的來源設備名(取代圓框裡的 icon)最大寬度。選中時 app_icon_frame
   以 DOT_ICON_SCALE * DOT_BIG_PROPORTION 放大置中在 DOT_BG_SIZE 的 dot_bg 裡,名字
   要**置中且不超出框**,所以留一點內距,超過就 … 截斷。 */
#define CONV_NAME_MAX_W 88

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// LV_IMG_DECLARE(img_flashlight);
// LV_IMG_DECLARE(img_activity);
LV_IMG_DECLARE(weather);
// LV_IMG_DECLARE(img_calculator);
// LV_IMG_DECLARE(img_calendar);
// LV_IMG_DECLARE(img_messages);
// LV_IMG_DECLARE(img_note);
// LV_IMG_DECLARE(img_alarm);
LV_IMG_DECLARE(message_bar);
// LV_IMG_DECLARE(img_mouse);
// LV_IMG_DECLARE(img_touchscreen);
// LV_IMG_DECLARE(img_logo);
// LV_IMG_DECLARE(img_photo);
// LV_IMG_DECLARE(img_workout);
// LV_IMG_DECLARE(img_recorder);
// LV_IMG_DECLARE(small_img_logo_matting);
LV_IMG_DECLARE(select_prompt);
LV_IMG_DECLARE(icon_release);
LV_IMG_DECLARE(app_icon_frame);
// LV_IMG_DECLARE(img_messages);

/* LEFT instruction list = PURE custom-instruction list, mirroring the RIGHT
   device_pager (SkaiLink left-list migration). The local utility apps
   (timer / flashlight / recorder) and the Skai AI widget were removed, so this
   definition is intentionally EMPTY → app_base_count == 0 and the list shows
   only the phone-pushed instructions. All sites that walk this array are bounded
   by ARRAY_SIZE / app_base_count, so a zero-length array degrades to no-ops. */
uint16_t INSTRUCTION_LIST_ITEMS_DEFINITION[] = {
#ifdef APP_ID_TIMER
    // app_id_timer,   /* removed: local app off the left list */
#endif
    // app_id_flashlight,  /* removed: local app off the left list */
    // app_id_recorder,    /* removed: local app off the left list */
    // app_id_ai,          /* no Skai input widget on the left list */
#ifdef _MSC_VER
    /* MSVC (PC simulator) rejects the empty `{}` initializer that GCC/Keil accept
       as a zero-length-array extension. This single sentinel slot is never read:
       every loop is bounded by INSTRUCTION_LIST_ITEMS_DEF_COUNT (0), not by
       ARRAY_SIZE of this array. Real firmware (GCC/Keil) keeps the empty list. */
    0xFFFFu,
#endif
};

/* Loop bound for INSTRUCTION_LIST_ITEMS_DEFINITION. The list is intentionally
   empty (app_base_count stays 0); under MSVC the array carries one sentinel slot
   to satisfy the C initializer rule, so pin the count to 0 there explicitly. */
#ifdef _MSC_VER
#define INSTRUCTION_LIST_ITEMS_DEF_COUNT 0u
#else
#define INSTRUCTION_LIST_ITEMS_DEF_COUNT \
    ARRAY_SIZE(INSTRUCTION_LIST_ITEMS_DEFINITION)
#endif

uint8_t return_total_list_count(void)
{
    return list_item_count;
}

/* ── ADR-0020 匯出:左頁(合併 session 列表)在自己的捲動容器裡畫 actions 區段。
   資料與執行路徑都留在本檔;左頁只讀 title/icon、點擊轉發 activate_index。
   回傳的指標指向 list_items[] 靜態儲存 —— LVGL 單執行緒,讀取期間不會被
   BLE thread 改寫(推播走 pending → LVGL thread apply)。 */
const char *instruction_list_export_title(uint8_t i)
{
    if (i >= list_item_count)
        return NULL;
    return list_items[i].title;
}

const char *instruction_list_export_id(uint8_t i)
{
    if (i >= list_item_count)
        return NULL;
    return list_items[i].id;
}

/* ADR-0020 R8:左頁(合併 session+actions)顯示的就是這個浮動清單本體。這個旗標
   由 clock 在左頁 settle 進出時設定;開著時麥克風 pill 的 tap = 開新 session
   (交回 session pager 的 conv_new),不是開 AI 輸入框。 */
static bool s_session_page_mode = false;
void instruction_list_set_session_page_mode(bool on)
{
    s_session_page_mode = on;
}

/* R9(founder:「actions 在下面」):把 conv: 開頭的 session item 穩定移到清單
   前段,actions 落在後段。static 暫存 —— list_item_t ~260B x30 ≈ 7.8KB,放
   LVGL thread 的堆疊太肥。呼叫端(session pager 注入)接著自己 refresh。 */
/* R27:把 conv: 段**強制排成 ids[] 給的順序**(其餘 actions 維持原順序、整段在後)。
   為什麼需要這支:`add_or_update_custom_instruction` 對已存在的 id 只更新內容,
   **位置原地不動**;`instruction_list_move_conv_items_first` 又是穩定分段。所以注入端
   改排序(R26 ts 由新→舊改舊→新)只對「這次才第一次出現」的 session 生效,早就在清單裡
   的仍停在當初插入的位置 —— founder 2026-08-12:「最新的還是在最上面沒有在下面」。
   回傳是否真的動過順序,讓呼叫端只在有變時才 refresh(避免每次輪詢都重繪)。 */
bool instruction_list_order_conv_items(const char *const *ids, uint8_t n)
{
    static list_item_t s_order_tmp[MAX_LIST_ITEMS];
    if (list_item_count == 0)
        return false;
    uint8_t w = 0;
    /* 1) 依 ids[] 指定的順序收 conv 項 */
    for (uint8_t k = 0; k < n && w < MAX_LIST_ITEMS; k++)
    {
        if (ids[k] == NULL || ids[k][0] == '\0')
            continue;
        for (uint8_t i = 0; i < list_item_count; i++)
        {
            if (strncmp(list_items[i].id, "conv:", 5) == 0 &&
                strcmp(list_items[i].id, ids[k]) == 0)
            {
                s_order_tmp[w++] = list_items[i];
                break;
            }
        }
    }
    /* 2) ids[] 沒點到的 conv 項(理論上不該有)接在後面,維持原相對順序 */
    for (uint8_t i = 0; i < list_item_count && w < MAX_LIST_ITEMS; i++)
    {
        if (strncmp(list_items[i].id, "conv:", 5) != 0)
            continue;
        bool taken = false;
        for (uint8_t k = 0; k < w; k++)
            if (strcmp(s_order_tmp[k].id, list_items[i].id) == 0) { taken = true; break; }
        if (!taken)
            s_order_tmp[w++] = list_items[i];
    }
    /* 3) actions 等非 conv 項,原順序,整段在 conv 之後 */
    for (uint8_t i = 0; i < list_item_count && w < MAX_LIST_ITEMS; i++)
        if (strncmp(list_items[i].id, "conv:", 5) != 0)
            s_order_tmp[w++] = list_items[i];
    if (w != list_item_count)
        return false; /* 數不對就別動整份清單(寧可順序舊,也不要掉項目) */
    bool changed = false;
    for (uint8_t i = 0; i < list_item_count; i++)
        if (strcmp(s_order_tmp[i].id, list_items[i].id) != 0) { changed = true; break; }
    if (changed)
        memcpy(list_items, s_order_tmp, sizeof(list_item_t) * list_item_count);
    return changed;
}

/* R31:`instruction_list_move_conv_items_first` 在 R27 被 order_conv_items 取代後成了
   孤兒,但它那份 `static list_item_t[MAX_LIST_ITEMS]`(~7.8KB)還是佔著 SRAM —— 而
   SRAM 少一塊就是 heap 少一塊。這台只剩 ~38KB heap,進滑鼠頁的大配置失敗就 NULL
   deref 當機,所以把重複的那份刪掉,只留 order_conv_items 的暫存。 */

/* instruction_list_focus_first_action 在檔尾(它用到的定位 statics 宣告在後段)。
   R32 的 release/ensure 也在檔尾 —— 它們要 p_instruction_list_layout(宣告在後面)。 */

const void *instruction_list_export_icon(uint8_t i)
{
    if (i >= list_item_count)
        return NULL;
    if (list_items[i].img_path[0] != '\0')
        return list_items[i].img_path; /* lv_img_set_src 認得檔案路徑字串 */
    return list_items[i].icon;         /* 資源指標,可為 NULL */
}

typedef struct
{
    lv_obj_t *list;
    lv_obj_t *p_instruction_list_bg;
    lv_obj_t *p_instruction_list_ai_bg;
    lv_obj_t *p_instruction_list_ai_icon;
    lv_obj_t *mic_bar; /* bottom mic-trigger bar; hidden when AI widget open */
    lv_obj_t *p_app_indicator_btn[MAX_LIST_ITEMS];
    lv_obj_t *indicator_dots[MAX_LIST_ITEMS];
    lv_obj_t *indicator_dots_bg[MAX_LIST_ITEMS];
    lv_obj_t *movable_range_arc; // 可移動範圍圓弧線
    lv_obj_t
        *app_list_tileview;  // vertical tileview: instruction list + app grid
    lv_obj_t *app_list_tile; // tile 1: app grid page
    arc_scroll_handle_t *arc_handle; // 共用 arc-scroll 模組 instance
    lv_obj_t *empty_view;       // list_item_count==0 placeholder (hint text or QR)
    lv_obj_t *empty_qr_card;    // white QR card inside empty_view, shown only when disconnected
    lv_obj_t *empty_hint_label; // caption under/instead of the QR
} instruction_list_layout_t;
static instruction_list_layout_t *p_instruction_list_layout;
static bool created = false;

static bool pause_instruction_list = true;

/* Map an @-conversation row's SERVICE (pushed by the phone as the "svc" field — e.g. "messenger" /
   "whatsapp", derived from the conv id) to its bundled logo so the right-side indicator dot shows the
   service glyph instead of the empty app_icon_frame placeholder (founder 2026-06-29). The watch list
   push only carries the title (NOT the conv id), so the service must arrive over the wire. */
static const char *service_icon(const char *svc)
{
    if (svc == NULL || svc[0] == '\0')
        return NULL;
    if (strcmp(svc, "whatsapp") == 0)  return ICON_WHATSAPP;
    if (strcmp(svc, "messenger") == 0) return ICON_MESSENGER;
    if (strcmp(svc, "instagram") == 0) return ICON_INSTAGRAM;
    if (strcmp(svc, "discord") == 0)   return ICON_DISCORD;
    if (strcmp(svc, "telegram") == 0)  return ICON_TELEGRAM;
    if (strcmp(svc, "line") == 0)      return ICON_LINE;
    if (strcmp(svc, "facebook") == 0)  return ICON_FACEBOOK;
    if (strcmp(svc, "slack") == 0)     return ICON_SLACK;
    if (strcmp(svc, "gmail") == 0)     return ICON_GMAIL;
    return NULL;
}

/* Map a saved Action's TYPE (pushed by the phone as the "ico" field — e.g. "music" /
   "weather", derived from the "New Action" card it was created with) to this watch's copy
   of the SAME glyph the phone draws in front of the name (founder 2026-07-24). The slugs are
   the wire contract in ActionTypeIcon.kt / .swift — keep the two in step.

   NULL for an unknown slug AND for an absent one: a row with no "ico" is not a saved Action
   at all (the aggregated skaibar batch also carries calc / url / memo / ask cards), so it
   keeps the pre-existing no-glyph look rather than picking up a generic one. */
static const char *action_type_icon(const char *ico)
{
    if (ico == NULL || ico[0] == '\0')
        return NULL;
    if (strcmp(ico, "music") == 0)        return ICON_ACT_MUSIC;
    if (strcmp(ico, "navigation") == 0)   return ICON_ACT_NAVIGATION;
    if (strcmp(ico, "drive") == 0)        return ICON_ACT_DRIVE;
    if (strcmp(ico, "webpage") == 0)      return ICON_ACT_WEBPAGE;
    if (strcmp(ico, "translate") == 0)    return ICON_ACT_TRANSLATE;
    if (strcmp(ico, "currency") == 0)     return ICON_ACT_CURRENCY;
    if (strcmp(ico, "stock") == 0)        return ICON_ACT_STOCK;
    if (strcmp(ico, "weather") == 0)      return ICON_ACT_WEATHER;
    if (strcmp(ico, "notification") == 0) return ICON_ACT_NOTIFICATION;
    if (strcmp(ico, "camera") == 0)       return ICON_ACT_CAMERA;
    if (strcmp(ico, "watchapp") == 0)     return ICON_ACT_WATCHAPP;
    if (strcmp(ico, "chat") == 0)         return ICON_ACT_CHAT;
    if (strcmp(ico, "generic") == 0)      return ICON_ACT_GENERIC;
    return NULL;
}

const char *get_app_icon(uint8_t app_id)
{
    switch (app_id)
    {
#ifdef APP_ID_RECORDER
    case app_id_recorder:
        return IMG_RECORDER;
#endif
    case app_id_weather:
        return IMG_GROUP;
    case app_id_exercise:
        return IMG_WORKOUT;
    case app_id_flashlight:
        return IMG_FLASHLIGHT;
#ifdef APP_ID_MEDIA
    case app_id_media:
        return IMG_ITUNES;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur:
        return IMG_GAME;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:
        return IMG_PHOTO;
#endif
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:
        return IMG_CALCULATOR;
#endif
#ifdef APP_ID_TIMER
    case app_id_timer:
        return IMG_ALARM_2;
#endif
#ifdef APP_ID_ALARM
    case app_id_alarm:
        return IMG_ALARM;
#endif
#ifdef APP_ID_SETTING
    case app_id_setting:
        return IMG_SETTINGS;
#endif
#ifdef APP_ID_MOUSE
    case app_id_mouse:
        return IMG_MOUSE;
#endif
    default:
        return IMG_LOGO;
    }
}

typedef struct
{
    lv_obj_t *right;
    lv_obj_t *left;
    lv_obj_t *top;
    lv_obj_t *bottom;
    lv_obj_t *right_img;
    lv_obj_t *left_img;
    lv_obj_t *top_img;
    lv_obj_t *bottom_img;
} quick_open_app_t;

void load_instruction_list(void);

static const lv_style_const_prop_t LIST_ITEM_STYLE_PROPS[] = {
    LV_STYLE_CONST_WIDTH(LIST_ITEM_WIDTH),
    LV_STYLE_CONST_HEIGHT(LIST_ITEM_HEIGHT),
    LV_STYLE_PROP_INV,
};

static const lv_style_const_prop_t LIST_ITEM_TITLE_STYLE_PROPS[] = {
    LV_STYLE_CONST_TEXT_FONT(&lv_font_montserrat_14),
    LV_STYLE_CONST_TEXT_COLOR(LV_COLOR_MAKE(0xFF, 0xFF, 0xFF)),
    LV_STYLE_CONST_BG_COLOR(LV_COLOR_MAKE(0, 0, 0)),
    LV_STYLE_PROP_INV,
};

LV_STYLE_CONST_INIT(LIST_ITEM_STYLE, LIST_ITEM_STYLE_PROPS);
LV_STYLE_CONST_INIT(LIST_ITEM_TITLE_STYLE, LIST_ITEM_TITLE_STYLE_PROPS);

static void set_icon_y(lv_obj_t *obj, lv_coord_t y)
{
    lv_obj_align(obj, LV_ALIGN_RIGHT_MID, -20, y);
}

static void animate_icon_vertical(lv_obj_t *obj, bool move_up)
{
    lv_coord_t end_y = move_up ? -30 : 30;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_values(&a, 0, end_y);
    lv_anim_set_time(&a, 200);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)set_icon_y);
    lv_anim_start(&a);
}

static void set_label_y(lv_obj_t *obj, lv_coord_t y)
{
    lv_obj_align(obj, LV_ALIGN_CENTER, 0, y);
}

static void animate_label_vertical(lv_obj_t *obj, bool move_up)
{
    lv_coord_t end_y = move_up ? -30 : 30;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_values(&a, 0, end_y);
    lv_anim_set_time(&a, 200);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)set_label_y);
    lv_anim_start(&a);
}

lv_obj_t *app_icon_shadow[MAX_LIST_ITEMS];
static bool is_indicator_dots_visible = true;
static uint16_t selected_item_index = 0;
static uint16_t last_zoom[MAX_LIST_ITEMS] = {0};

/* SKAIBAR option-tracking session lifetime — decoupled from
   is_open_instruction_list_ai. The voice/v2t session ends when the
   pill fades (close_ai_widget), but founder direction 2026-05-19 is
   that the phone-side SKAIBAR option highlight should keep following
   the watch's scroll position even after the pill is gone. The
   session ends only when the user leaves the instruction_list page
   entirely (instruction_list_pause).
   Set true inside tap_on_ai_widget (the moment the user invokes the
   SKAIBAR flow); cleared inside instruction_list_pause. Declared up
   here so scroll_list (line ~790) and tap_on_ai_widget /
   instruction_list_pause (further down) all see the same storage. */
static bool s_skaibar_tracking_active = false;

/* SKAI_LINK active target for the LEFT list = the watch's directly-connected
   device (the primary = the phone itself). The primary is deliberately NOT in
   the watch's device_registry (the phone excludes it from the device-list push;
   its actions ARE this left list), so it has no id to assert here. Instead we
   assert an EMPTY active target before each focus/commit: the phone reads "no
   remote device active" as "the watch is on the left list → run locally on the
   primary". This mirrors device_pager asserting a real device id for the right
   page, and clears any stale remote target if the left list was entered without
   passing through device_pager's leave path. De-duped inside
   commu_send_active_device (shared with device_pager), so re-asserting on every
   focus/commit only sends on an actual change. */
static void instruction_list_assert_local_target(void)
{
    /* R3: on the device page the floating list shows the ACTIVE remote device's
       options and IS its controlled surface; asserting the local/primary target
       here would clear that active device (active="") mid focus/commit, so the
       phone falls back to its own launcher and mirrors the WRONG options onto the
       watch. Only assert local when we are NOT controlling a device. */
    extern bool device_pager_is_on_page(void);
    if (device_pager_is_on_page()) return;
    /* STANDALONE 滑鼠 app 正控制一台設備時(app active + 已選設備),清掉 active(="")會斷掉
       relay 目標 → scroll/focus/commit 跑回 primary、且下次 bar summon 因無 target 而 no-op
       (電腦不開)、選項退回聚合(電腦+actions)。用 hid_mouse_owns_active_target 比只看
       s_bar_single_device 穩 —— 後者只在 bar 列表開著時為真,列表收掉(commit 後)仍在控制,
       這時一個 scroll/focus 就會把目標清掉。focus/commit 仍照送(scroll→那台 highlight、
       tap→那台執行),只是不送 active="". */
    extern bool hid_mouse_owns_active_target(void);
    if (hid_mouse_owns_active_target()) return;
    if (s_bar_single_device) return;
    commu_send_active_device(""); /* "" = no remote target → primary/local */
}

/* arc-scroll detached / discrete 模式狀態 — 拖動時 arc 不動 list、由 drag_cb
 * 接管，到 page change 才 snap。完整定義在後面，scroll_list 要先 visible。
 * 用獨立 bool flag 而不是 input 的特殊值當「是否已初始化」 — elastic overshoot
 * 會讓 input 掉到負值（min_input - 50 = -13），不能再用 <0 當 sentinel */
static bool s_inst_arc_drag_active = false;
static bool s_inst_drag_initialized = false;
static int s_inst_drag_input = 0;
static int s_inst_drag_last_idx = -1;    /* 上一次中央的 dot idx */
static rt_tick_t s_last_arc_drag_tick = 0; /* 最後一次 inst_arc_drag_cb 實際驅動的 tick */
/* arc-drag 是否「真的正在驅動」= flag true 且最近有幀在跑。區分兩種狀態：
 *  - arc-drag 進行中：dot/selected 由 inst_arc_drag_cb + page-change snap 管，
 *    scroll_list 讓開（避免打架 — 原本 !s_inst_arc_drag_active gate 的用意）；
 *  - arc-drag 已停但 flag 殘留（release 的 snap_cb 還沒 reset）、list 改由 LVGL
 *    原生慣性 scroll 移動：inst_arc_drag_cb 已不再跑，若仍照 raw flag gate 掉
 *    dot/selected，dot 會卡在殘留位置、label 卻自由跑 → 脫節（真機 [DYN]：adrag=1
 *    時 label 205→345、dot 恆 193）。用「距最後驅動 <120ms」判定 live。 */
static bool arc_drag_is_live(void)
{
    return s_inst_arc_drag_active &&
           (rt_tick_get() - s_last_arc_drag_tick) < rt_tick_from_millisecond(120);
}
static int32_t s_inst_snap_anim_dummy; /* 提前定義：SCROLL_END 的 settle pin 要取消這個 anim */
static void inst_snap_anim_exec_cb(void *var, int32_t value); /* fwd-decl 供 pin 取消用 */
static void inst_arc_reset_drag_state(void);
static void scroll_list_to_index(uint16_t page, bool animate);

static void update_indicator_dots_position(int input_value)
{
    // LOG_I("Updating indicator dots position, input value: %d", input_value);
    if (p_instruction_list_layout == NULL || !is_indicator_dots_visible)
        return;

    int total_dots = list_item_count;
    if (total_dots <= 0)
        return;

    // LOG_I("Updating indicator dots position, input value: %d",
    //       input_value);

    /* 跟 app_exercise.c::apply_circular_layout 對齊：圓心在螢幕正中、
     * radius=200。原本 (120, 233, 300) 那組會讓 arc 中心偏左、半徑大、
     * dots 上下散開比較廣，視覺上跟 exercise 不一樣。
     * center_x 往左偏 30 px：中央 dot zoom 到 1.3x（130 px 寬）時，沒偏的話
     * 右邊會跑出螢幕；偏 30 後最右邊大約在 448，剛好在 466 螢幕內 */
    const int circle_radius = 200;
    const int center_x = LV_HOR_RES / 2 - 20;
    const int center_y = LV_VER_RES / 2;

    const float angle_per_dot = 36.0f; /* 跟 app_exercise.c 的 ICON_SLOT_ANGLE_DEG=36 對齊 */

    float base_input = 63.0f;
    float degrees_per_200_input = angle_per_dot;
    float totlal_input_range = 100 * total_dots;

    float offset_angle =
        ((totlal_input_range - (float)input_value) - base_input) /
        (float)(totlal_input_range / total_dots) * degrees_per_200_input;

    for (int i = 0; i < total_dots; i++)
    {
        if (p_instruction_list_layout->indicator_dots[i] == NULL ||
            !lv_obj_is_valid(p_instruction_list_layout->indicator_dots[i]))
            continue; /* skip freed dots: a rebuild can re-enter here mid-teardown */

        float base_angle = i * angle_per_dot;
        /* 不做 [0,360) normalize — 留 signed angle，方便用 |angle| > 90 一刀
         * 過濾掉「在 list 第一格時 dot N-1 從另一邊 wrap 過來出現在上方」的問題。
         * 例如 N=10 顆 dot，第一格時 dot 9 的 base_angle = 9*36 = 324°，wrap 後
         * 變成 (270, 360) 區間 → 既有 (90,270) 過濾擋不到 → 出現在右上方。
         * signed_angle = 324°（不 wrap）→ > 90° → 直接 hide */
        float current_angle = base_angle - offset_angle;

        float angle_rad = current_angle * M_PI / 180.0f;

        int dot_x = center_x + (int)(circle_radius * cos(angle_rad));
        int dot_y = center_y + (int)(circle_radius * sin(angle_rad));

        /* 用 |signed angle| > 90° 一次過濾掉左半圓 + 從另一邊繞回來的 dots */
        {
            lv_obj_t *dot_bg_obj =
                p_instruction_list_layout->indicator_dots_bg[i];
            if (current_angle < -90.0f || current_angle > 90.0f)
            {
                if (dot_bg_obj != NULL &&
                    !lv_obj_has_flag(dot_bg_obj, LV_OBJ_FLAG_HIDDEN))
                {
                    lv_obj_add_flag(dot_bg_obj, LV_OBJ_FLAG_HIDDEN);
                }
                continue;
            }
            if (dot_bg_obj != NULL &&
                lv_obj_has_flag(dot_bg_obj, LV_OBJ_FLAG_HIDDEN))
            {
                lv_obj_clear_flag(dot_bg_obj, LV_OBJ_FLAG_HIDDEN);
            }
        }

        // 限制 dot_y 超過 450 或小於 16 時，dot_x 不再變動
        static int last_valid_dot_x[MAX_LIST_ITEMS] = {0};
        if (dot_y > 450 || dot_y < 16)
        {
            if (p_instruction_list_layout->indicator_dots_bg[i] != NULL)
            {
                // dot_x 不變，使用上一次合法的 dot_x
                dot_x = last_valid_dot_x[i];
            }
        }
        else
        {
            last_valid_dot_x[i] = dot_x;
        }

        /* 跟 app_exercise.c::apply_circular_layout 一致：用 cos(abs_angle) 做
         * 平滑漸層。current_angle 現在是 signed [-90,90]，直接 fabsf 就是
         * 從水平右軸算起的 abs_angle */
        float abs_angle_deg = fabsf(current_angle);
        float ratio = cosf(abs_angle_deg * (float)M_PI / 180.0f);

        int dot_size = DOT_BG_SIZE;
        int opacity = (int)(LV_OPA_30 + (LV_OPA_COVER - LV_OPA_30) * ratio);
        if (opacity < LV_OPA_30)
            opacity = LV_OPA_30;
        if (opacity > LV_OPA_COVER)
            opacity = LV_OPA_COVER;

        lv_obj_set_style_img_opa(p_instruction_list_layout->indicator_dots[i],
                                 opacity, 0);
        /* R17:conv 列的設備名 label(dot_bg 第 3 個 child)跟 dot 同步淡出。
           label 不吃 zoom(EPIC label 特性),只同步 opa。
           R19(founder):設備名整體再暗一點 —— 基準 55%(dot 全亮時 ≈ 140)。 */
        {
            lv_obj_t *name = lv_obj_get_child(
                p_instruction_list_layout->indicator_dots_bg[i], 2);
            if (name != NULL && lv_obj_is_valid(name))
                lv_obj_set_style_text_opa(
                    name, (lv_opa_t)(((uint16_t)opacity * 140) / 255), 0);
        }

        /* 用指數曲線 ratio^N 取代線性 ratio：N>1 時，中央 dot 大幅放大，
         * 邊緣 dot 快速縮小，視覺上中央更突出 */
        float zoom_ratio = powf(ratio, DOT_ZOOM_EXPONENT);
        uint16_t zoom =
            (uint16_t)(255 * DOT_ICON_SCALE *
                       (DOT_SMOLL_PROPORTION +
                        (DOT_BIG_PROPORTION - DOT_SMOLL_PROPORTION) * zoom_ratio));
        if (abs((int)zoom - (int)last_zoom[i]) > 5)
        {
            /* Guard both handles: a reveal-triggered rebuild (reset_list +
             * refresh_custom_instructions) recreates the dots while last_zoom[]
             * persists (static, not reset), so this branch fires on the first
             * frame after the rebuild. An unguarded lv_img_set_zoom on a stale
             * app_icon_shadow[i] faulted here (mem manage DACCVIOL, ~0x7E9 on
             * img->zoom). Mirror the NULL+valid checks used at the other dot
             * call sites (see the show/hide block above). */
            if (app_icon_shadow[i] != NULL && lv_obj_is_valid(app_icon_shadow[i]))
                lv_img_set_zoom(app_icon_shadow[i], zoom);
            if (p_instruction_list_layout->indicator_dots[i] != NULL &&
                lv_obj_is_valid(p_instruction_list_layout->indicator_dots[i]))
                lv_img_set_zoom(p_instruction_list_layout->indicator_dots[i], zoom);
            last_zoom[i] = zoom;
        }
        lv_obj_center(p_instruction_list_layout->indicator_dots[i]);
        dot_x -= (dot_size + 30) / 2;
        dot_y -= dot_size / 2;

        lv_obj_set_pos(p_instruction_list_layout->indicator_dots_bg[i], dot_x,
                       dot_y);
    }
}

/* fwd decl — dot click 直接走跟 touch_obj 同樣的 click handler，省去重複邏輯 */
static void list_item_click_event_cb(lv_event_t *evt);

static void create_indicator_dots(lv_obj_t *parent)
{
    if (p_instruction_list_layout == NULL)
        return;

    int total_dots = list_item_count;

    for (int i = 0; i < total_dots; i++)
    {
        lv_obj_t *dot_bg = lv_obj_create(parent);
        lv_obj_set_size(dot_bg, DOT_BG_SIZE, DOT_BG_SIZE);
        lv_obj_set_style_bg_opa(dot_bg, LV_OPA_0, 0);
        lv_obj_clear_flag(dot_bg, LV_OBJ_FLAG_SCROLLABLE);
        /* 仿 app_exercise.c 把 icon 本身設成可點擊：tap 任一可見 dot 直接觸發
         * 對應 item 的 click handler，不需要先把它 scroll 到中央再點 */
        lv_obj_add_flag(dot_bg, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(dot_bg, list_item_click_event_cb,
                            LV_EVENT_CLICKED, (void *)&list_items[i]);
        /* R3: bubble presses on a dot (the right-side indicator icon) up to
           p_instruction_list_bg, where the horizontal drawer-drag handler ALSO
           lives — dot_bg is a SIBLING of the scrollable list, so without this a
           drag that starts on a dot never reaches the drag handler (only text,
           which lands on the item's touch_obj inside the list, worked). The img
           children bubble too in case they are hit targets. */
        lv_obj_add_flag(dot_bg, LV_OBJ_FLAG_EVENT_BUBBLE);

        app_icon_shadow[i] = lv_img_create(dot_bg);
        lv_img_set_src(app_icon_shadow[i], &app_icon_frame);
        lv_obj_align(app_icon_shadow[i], LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(app_icon_shadow[i], LV_OBJ_FLAG_EVENT_BUBBLE);

        lv_obj_t *dot = lv_img_create(dot_bg);
        lv_obj_center(dot);
        lv_obj_add_flag(dot, LV_OBJ_FLAG_EVENT_BUBBLE);
        if (list_items[i].img_path[0] != '\0')
        {
            lv_img_set_src(dot, list_items[i].img_path);
        }
        else if (list_items[i].icon != NULL)
        {
            lv_img_set_src(dot, list_items[i].icon);
        }
        else
        {
            /* Instructions without icon: show frame only */
            lv_img_set_src(dot, &app_icon_frame);
        }

        /* R17(founder):session 列的右緣不要圓框,改直接顯示來源設備名(取代
           R11-R15 的標題右下角小副標)。dot img 保留但藏起來(位置/zoom 迴圈
           仍對 img 操作,label 不能頂替它),設備名 label 掛在 dot_bg 上、右緣
           對齊往左長(框寬=文字寬,R15 教訓:別靠 text_align),淡出在
           update_indicator_dots_position 跟 dot 同步。 */
        /* R61:「開新對話」列也走同一套 —— 右緣圓框放的是**要開在哪一台**的名字
           (founder:「右邊的圖片裡面就放設備名稱」),兩台設備就是兩列同名選項、
           只有右邊的設備名不同。 */
        if (strncmp(list_items[i].id, "conv:", 5) == 0 ||
            strncmp(list_items[i].id, NEW_SESSION_ITEM_ID,
                    sizeof(NEW_SESSION_ITEM_ID) - 1) == 0)
        {
            extern const char *session_list_device_name_for(const char *conv_id);
            extern const char *session_list_device_name_of(const char *device_id);
            extern lv_font_t *lvsf_get_font_from_size(uint16_t size);
            const char *dev_name;
            if (list_items[i].id[0] == NEW_SESSION_ITEM_ID[0])
            {
                const char *did = list_items[i].id + (sizeof(NEW_SESSION_ITEM_ID) - 1);
                dev_name = (*did == ':') ? session_list_device_name_of(did + 1) : NULL;
            }
            else
            {
                dev_name = session_list_device_name_for(list_items[i].id);
            }
            if (dev_name != NULL && dev_name[0])
            {
                lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(dot_bg, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
                lv_obj_t *name = lv_label_create(dot_bg);
                lv_font_t *f = lvsf_get_font_from_size(18);
                if (f != NULL)
                    lv_obj_set_style_text_font(name, f, 0);
                lv_obj_set_style_text_color(name, lv_color_hex(0xFFFFFF), 0);
                lv_label_set_text(name, dev_name);
                lv_obj_clear_flag(name, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_add_flag(name, LV_OBJ_FLAG_EVENT_BUBBLE);
                /* R18(founder):超過就 … 截斷。只在超寬時才鎖寬走 LONG_DOT(截斷後
                   文字填滿框,無對齊歧義);短的維持框寬=文字寬,免踩 text_align 不
                   生效的雷(R13-R15)。
                   R29(founder:「設備名稱要剛好在 app_icon_frame 中間,長度也不能
                   超過」):選中框 app_icon_frame 跟 dot img 一樣 center 在 dot_bg,
                   所以名字改 **LV_ALIGN_CENTER**(原本靠右對齊,會偏出框外);寬度上
                   限縮到框內 CONV_NAME_MAX_W,超過就 … 。框寬=文字寬時置中框=置中
                   文字,截斷時文字填滿框也仍置中,兩種情況都不依賴 text_align。 */
                lv_obj_update_layout(name);
                if (lv_obj_get_width(name) > CONV_NAME_MAX_W)
                {
                    lv_label_set_long_mode(name, LV_LABEL_LONG_DOT);
                    lv_obj_set_width(name, CONV_NAME_MAX_W);
                }
                lv_obj_align(name, LV_ALIGN_CENTER, 0, 0);
            }
        }

        p_instruction_list_layout->indicator_dots_bg[i] = dot_bg;
        p_instruction_list_layout->indicator_dots[i] = dot;
    }

    /* 初始定位 */
    update_indicator_dots_position(37);
}

extern void tap_on_ai_hint(void);
static lv_obj_t *ai_voice_btn = NULL;
static lv_obj_t *ai_voice_send_icon = NULL;
static lv_obj_t *ai_gaus_bg = NULL;
/* Transcript label inside the styled skai_widget pill — shows the spoken
   text after voice_say (PC sim) or real ASR (real hw). */
static lv_obj_t *s_voice_transcript_label = NULL;
/* Fixed-height (2 line-heights) clip that hosts the transcript label so only the
   latest two wrapped rows show; older rows scroll up and can be pulled back down
   manually. Mirrors device_pager's skaibar_clip. */
static lv_obj_t *s_voice_transcript_clip = NULL;
/* Reference to the styled pill (lv_skai_widget_builder return value).
   For scroll-fade we animate per-property opa on multiple objects (LVGL
   lv_obj_set_style_opa does not cascade in this build — must use
   _bg_opa / _border_opa / _text_opa / _img_opa explicitly). */
static lv_obj_t *s_skai_widget = NULL;
/* Global persistent input-bar layer on lv_layer_top(): the mic_bar + hit area +
   ai_box live HERE, not inside the LEFT-tile content, so the bar floats above
   every page and stays put (doesn't slide away with tile transitions). The whole
   layer is shown/hidden per page via instruction_list_bar_set_visible(). It is
   full-screen but NON-clickable so non-bar touches fall through to the page. */
static lv_obj_t *s_global_bar_layer = NULL;
/* The voice-group image inside ai_voice_btn — needs img_opa fade. */
static lv_obj_t *s_voice_img = NULL;
/* The pill background image (message_widget_bg) — gives the pill its
   border + shape on real-hw where the LVGL border style is invisible. */
static lv_obj_t *s_pill_bg_img = NULL;
/* The mic glyph inside the bottom mic_bar — faded out as the bar morphs into
   the input box (mirrors the right device_pager skaibar). */
static lv_obj_t *s_mic_bar_icon = NULL;
/* 蓋在 mic_bar 上方的大片 tap 區(mic_hit)。單設備抽屜換成三鍵列時要連它一起藏 ——
   它是 324x106 的透明大片,不藏的話按鈕之間的縫隙全被它吃掉。 */
static lv_obj_t *s_mic_hit = NULL;
/* 單設備抽屜的底部三鍵列正在台上。宣告放這麼前面是因為 instruction_list_refresh_home_bar()
   (每次 check_is_at_home poll 都跑)要拿它當閘門 —— 那支會依「清單有沒有顯示」重算
   mic_bar 的顯藏,三鍵列上場時如果不擋住它,舊的小麥克風下一拍就被掀回來、疊在三鍵列
   後面(founder 2026-08-17:「那三個按鈕後面怎麼還有舊的小麥克風」)。 */
static bool s_drawer_row_shown = false;
static void drawer_row_engage(bool on); /* 單設備抽屜的底部三鍵列上/下場 */

/* mic_bar 身上那顆看得見的麥克風圖示。抽出成函式是為了能「刪掉再建回來」——
   founder 2026-08-17:「那個的功能應該已經完全不需要了吧,不能直接把他整個移除嗎?」
   在滑鼠抽屜裡確實完全不需要(三鍵列取代了它),但這顆是**共用**的:錶盤 session 清單
   底部按住講話搜清單的也是它。所以做法是範圍內的真移除 —— 三鍵列上場時 lv_obj_del,
   下台時原樣建回來。刪掉之後任何寫入點都動不了它(它們一律 lv_obj_is_valid 防護),
   這比連續四輪「藏了又被誰掀回來」可靠。 */
/* 與後面的 LMIC_ICON_Y_OFS 同值 —— 這支抽到檔案前段(refresh_home_bar 之前)才能被
   drawer 那段呼叫,而那個 #define 在 2200 多行才出現。兩者要改一起改。 */
#define MIC_BAR_ICON_Y_OFS 0
static void mic_bar_build_icon(lv_obj_t *bar)
{
    if (!bar || !lv_obj_is_valid(bar))
        return;
    s_mic_bar_icon = lv_img_create(bar);
    lv_img_set_src(s_mic_bar_icon, &micro_icon);
    lv_img_set_pivot(s_mic_bar_icon, micro_icon.header.w / 2, micro_icon.header.h / 2);
    lv_obj_add_flag(s_mic_bar_icon, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_img_set_zoom(s_mic_bar_icon, 128); /* 64px 原生縮 50% → 視覺 32px(founder 2026-08-06);觸控走 mic_hit,尺寸不變 */
    lv_obj_align(s_mic_bar_icon, LV_ALIGN_CENTER, 0, MIC_BAR_ICON_Y_OFS); /* 上移避免被底緣切到 */
    lv_obj_clear_flag(s_mic_bar_icon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_move_background(s_mic_bar_icon); /* 重建時回到 mic_bar 子物件的最底,不擋 ripple */
}

/* mic_hit 的顯藏**永遠跟著 mic_bar 走**(founder 2026-08-17:「不管我怎麼點 skaibar_img
   都不會叫出 session 列表,是不是有透明的東西擋到」——是)。
   mic_hit 是 324x106 的**看不見但 CLICKABLE** 的大片,掛在 lv_layer_top() 的
   s_global_bar_layer 上、BOTTOM_MID −20 —— 正好蓋在滑鼠 app 自己那張 skaibar_img 上面。
   它的 handler(mic_hit_event_cb)在 mic_bar 是 HIDDEN 時直接 return,**但 return 得太晚**:
   hit-test 已經把這一下判給它了,事件不會再往下傳 → 滑鼠 app 的 bottom_logo_cb 根本沒被
   呼叫,畫面上什麼都不會發生、log 也一片安靜。
   會落進這個狀態的路徑:立起輸入面板(open_lift_input_view 藏 mic_bar 卻讓整層留著可見,
   且 floating_bar_visible() 對這個情境刻意回報 false,所以滑鼠自有 bar 照樣顯示)。
   修法=藏 mic_bar 的同時一定連它一起藏,單一真相走這支。 */
static void mic_hit_follow_bar(void)
{
    if (!s_mic_hit || !lv_obj_is_valid(s_mic_hit))
        return;
    lv_obj_t *bar = p_instruction_list_layout ? p_instruction_list_layout->mic_bar : NULL;
    bool bar_hidden = !bar || !lv_obj_is_valid(bar) ||
                      lv_obj_has_flag(bar, LV_OBJ_FLAG_HIDDEN);
    if (bar_hidden)
        lv_obj_add_flag(s_mic_hit, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(s_mic_hit, LV_OBJ_FLAG_HIDDEN);
}

/* Bottom mic-bar → input-box MORPH geometry (mirrors device_pager's skaibar):
   on trigger the slim bottom bar grows + slides up into the 442x252 input box,
   THEN the skai_widget pill (frame + transcript + voice button) fades in on top.
   Bar geometry matches the mic_bar created in the layout; box geometry matches
   the skai_widget pill (BOTTOM_MID, -5). */
#define LMIC_W        176    /* resting bar = skaibar_img native size (176x31) */
#define LMIC_H        31
#define LMIC_Y        (-20)
#define LMIC_RADIUS   8
#define LBOX_W        442
#define LBOX_H        252
#define LBOX_Y        (75)   /* box sits 75px below bottom — top half on-screen (matches device_pager SKAIB_Y) */
#define LBOX_RADIUS   80
#define LMORPH_GROW_MS  220   /* phase 1: bar grows into the box backdrop */
#define LMORPH_FRAME_MS 160   /* phase 2: skai_widget pill fades in */
#define LSLIDE_MS       200   /* R3: list slides in/out from the right edge.
                                 Kept < the shortest close morph (220ms shrink) so
                                 the list parks off-screen before
                                 finalize_close_ai_widget flips its HIDDEN flag. */

/* Pin the 2-row transcript window to its newest content: scroll the clip so the
   last two wrapped rows are visible (older rows scroll up out of view; the user
   can pull them back down manually). No-op when the text fits in two rows. */
static void voice_transcript_scroll_to_bottom(void)
{
    if (!s_voice_transcript_clip || !lv_obj_is_valid(s_voice_transcript_clip))
        return;
    lv_obj_update_layout(s_voice_transcript_clip);
    lv_coord_t bottom = lv_obj_get_scroll_bottom(s_voice_transcript_clip);
    if (bottom > 0)
        lv_obj_scroll_by(s_voice_transcript_clip, 0, -bottom, LV_ANIM_OFF);
    else
        lv_obj_scroll_to_y(s_voice_transcript_clip, 0, LV_ANIM_OFF);
}

void instruction_list_set_voice_transcript(const char *text)
{
    if (s_voice_transcript_label && lv_obj_is_valid(s_voice_transcript_label))
    {
        lv_label_set_text(s_voice_transcript_label, text ? text : "");
        voice_transcript_scroll_to_bottom();   /* keep the latest 2 rows in view */
    }
    /* R55:左頁(session 檢視)裡,說出來的字同時是**清單的搜尋字** —— actions 標題與
       各設備的 session 標題一起篩,都沒中就只剩「開新 session」。其他情境(錶盤 skaibar、
       滑鼠頁)維持原本行為:那裡的查詢是送到手機做的,不在手錶本地篩。 */
    if (s_session_page_mode)
    {
        extern void instruction_list_set_text_filter(const char *text);
        /* 「Listening」是開麥克風時的**佔位字**,不是使用者說的話 —— log 實測它會一路
           走到這裡把清單篩成空的(founder 2026-08-13:[flt] set "Listening")。 */
        const char *placeholder = LV_EXT_STR_GET_BY_KEY(listening, "Listening");
        if (text != NULL && placeholder != NULL && strcmp(text, placeholder) == 0)
            return;
        instruction_list_set_text_filter(text);
    }
}

void set_indicator_dots_visible(bool visible)
{
    if (is_indicator_dots_visible == visible)
        return;
    is_indicator_dots_visible = visible;
    for (int i = 0; i < list_item_count; i++)
    {
        if (p_instruction_list_layout->indicator_dots[i] != NULL)
        {
            if (visible)
            {
                lv_obj_set_style_img_opa(
                    p_instruction_list_layout->indicator_dots[i], LV_OPA_60, 0);
            }
            else
            {
                lv_obj_set_style_img_opa(
                    p_instruction_list_layout->indicator_dots[i], LV_OPA_20, 0);
            }
        }
    }
}


extern lv_img_dsc_t *create_widget_snapshot_img(lv_obj_t *target_obj);
lv_obj_t *app_icon[MAX_LIST_ITEMS];
lv_obj_t *app_widget[MAX_LIST_ITEMS];
lv_obj_t *touch_obj[MAX_LIST_ITEMS];
lv_obj_t *app_label[MAX_LIST_ITEMS];
static lv_obj_t *widget_img = NULL;
static bool left_hand_mode = true;
static bool need_correction = false;
static bool is_widget_animation_active = false; // 追蹤 widget 動畫狀態
static uint8_t app_scroll_target_item = 0;
static uint16_t old_selected_item_index = -1;
static lv_obj_t *selected_label;

void set_arc_stripe_external_offset(int16_t offset_degrees)
{
    update_indicator_dots_position(offset_degrees);
}

static void animate_open_selected_widget_cb(lv_anim_t *a)
{
    if (selected_item_index < list_item_count)
    {
        if (app_widget[selected_item_index] != NULL &&
            lv_obj_is_valid(app_widget[selected_item_index]))
        {
            lv_obj_clear_flag(app_widget[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
        if (touch_obj[selected_item_index] != NULL &&
            lv_obj_is_valid(touch_obj[selected_item_index]))
        {
            lv_obj_clear_flag(touch_obj[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
    }
    lv_obj_add_flag(widget_img, LV_OBJ_FLAG_HIDDEN);

    is_widget_animation_active = false;
}

static void set_widget_img_opa(lv_obj_t *obj, lv_opa_t opa)
{
    // lv_obj_set_style_img_opa(obj, opa, LV_STATE_DEFAULT);
}

static void animate_widget_img_opa(lv_obj_t *obj)
{
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_values(&a, 0, 255);
    lv_anim_set_time(&a, 200);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)set_widget_img_opa);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_set_ready_cb(&a, animate_open_selected_widget_cb);
    lv_anim_start(&a);
}

static lv_coord_t last_y_diff_on_selected = 0;

static bool open_gesture_control = false;
// 停止所有動畫並復原狀態的函數
static void stop_all_animations_and_reset(void)
{
    // 如果沒有進行中的動畫，直接返回
    if (!is_widget_animation_active)
    {
        return;
    }
    else
    {
        LOG_D("Stopping all animations and resetting states");
    }

    if (widget_img != NULL && lv_obj_is_valid(widget_img))
    {
        lv_anim_del(widget_img, (lv_anim_exec_xcb_t)set_widget_img_opa);
        lv_obj_add_flag(widget_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_img_opa(widget_img, LV_OPA_0, LV_STATE_DEFAULT);
    }

    for (uint8_t i = 0; i < list_item_count; i++)
    {
        if (app_icon[i] != NULL && lv_obj_is_valid(app_icon[i]))
        {
            lv_anim_del(app_icon[i], (lv_anim_exec_xcb_t)set_icon_y);
            lv_obj_align(app_icon[i], LV_ALIGN_RIGHT_MID, -25, 0);
        }

        if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
        {
            lv_anim_del(app_label[i], (lv_anim_exec_xcb_t)set_label_y);
            lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
            lv_obj_clear_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
        }

        if (app_widget[i] != NULL && lv_obj_is_valid(app_widget[i]))
        {
            lv_obj_add_flag(app_widget[i], LV_OBJ_FLAG_HIDDEN);
            if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
            {
                lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    is_widget_animation_active = false;

    LOG_D("All animations stopped and states reset");
}

extern void open_skai_widget_ai(bool open);
static bool is_open_instruction_list_ai = false;
bool get_is_open_instruction_list_ai(void)
{
    return is_open_instruction_list_ai;
}

/* Show / hide the global persistent input bar (lv_layer_top). Called per page by
   app_clock (watch face + instruction_list show it; message / control-center hide
   it) and by device_pager for the mouse page (R2). No-ops until the bar exists. */
void instruction_list_bar_set_visible(bool visible)
{
    if (!s_global_bar_layer || !lv_obj_is_valid(s_global_bar_layer)) return;
    if (visible)
    {
        lv_obj_clear_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN);
        { extern void hid_mouse_set_own_bar_hidden(bool); hid_mouse_set_own_bar_hidden(true); } /* 浮層 bar 現→當幀收滑鼠自有 bar */
    }
    else
    {
        /* Hiding the bar layer (message / control-center / app switch): close the
           floating list so neither it NOR its watch-face blur lingers.
           close_on_leave runs the animated close (no-op when the box isn't open);
           we ALSO drop the blur SYNCHRONOUSLY here so a page that shows its own
           gaus_dial_bg backdrop next isn't clobbered by the async close's
           finalize. Both set_blur(false) calls self-guard on s_bar_blur_active. */
        extern void instruction_list_close_ai_on_leave(void);
        extern void instruction_list_bar_set_blur(bool on);
        extern void instruction_list_teardown_empty_qr(void);
        instruction_list_close_ai_on_leave();
        instruction_list_bar_set_blur(false);
        instruction_list_teardown_empty_qr(); /* list closing: don't leave the dynamic QR resident */
        lv_obj_add_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN);
        { extern void hid_mouse_set_own_bar_hidden(bool); hid_mouse_set_own_bar_hidden(false); } /* 浮層 bar 收→當幀還原滑鼠自有 bar */
    }
}
void set_is_open_instruction_list_ai(bool open)
{
    is_open_instruction_list_ai = open;
}

/* 2026-06-25: the bottom mic pill now belongs to the left mixed-list page, not the
   bare watch face. Show it only while the floating list is revealed/open (it sits
   at the list's bottom) or on the mouse page (which uses the bar as its own
   trigger); hide it on the bare home face. s_global_bar_layer itself stays shown on
   home so the left-edge reveal overlay keeps receiving touches — only the pill (and
   its skaibar_img child) is hidden. */
void instruction_list_refresh_home_bar(void)
{
    if (!p_instruction_list_layout || !p_instruction_list_layout->mic_bar ||
        !lv_obj_is_valid(p_instruction_list_layout->mic_bar))
        return;
    extern bool clock_main_page_is_home(void);
    lv_obj_t *bar = p_instruction_list_layout->mic_bar;
    lv_obj_t *bg = p_instruction_list_layout->p_instruction_list_bg;
    bool list_shown = bg && lv_obj_is_valid(bg) &&
                      !lv_obj_has_flag(bg, LV_OBJ_FLAG_HIDDEN);
    bool hide_pill = clock_main_page_is_home() && !list_shown;
    /* 單設備抽屜換上三鍵列時,那條 pill(以及它身上那顆小麥克風)是被三鍵列取代掉的 ——
       這支 poll 不知情就會依「清單有顯示」把它掀回來,疊在三鍵列後面。三鍵列在台上=
       一律藏。長按直開語音那條(animate_open_ai_widget 要拿 mic_bar 做 morph)不受影響:
       它走的是 push_up,三鍵列那時已經下台。 */
    if (s_drawer_row_shown)
        hide_pill = true;
    /* 再往外一層:**整個單設備抽屜 session 期間**這條 pill 都沒有工作了 —— 底部依序由
       三鍵列(抽屜)、語音站自己那排(輸入模式)接管。唯一還需要它的是長按直開語音那條,
       animate_open_ai_widget 拿它當 morph 的起點,那時 ai box 是顯示著的。
       為什麼把判斷放在這支 poll 而不是各進場路徑:mic_bar 的顯藏有六個寫入點
       (立起面板進/出、refresh_home_bar、抽屜三鍵列、animate_open_ai_widget、
       close_ai_widget),任何一次性的 add_flag 都可能被其中某條在下一拍推翻。poll 是
       唯一持續在跑的,讓它當裁決者才不會再漏第七個(founder 2026-08-17 連兩輪回報
       「三個按鈕後面還有舊的小麥克風」)。 */
    if (s_bar_single_device)
    {
        lv_obj_t *box = p_instruction_list_layout->p_instruction_list_ai_bg;
        bool box_shown = box && lv_obj_is_valid(box) &&
                         !lv_obj_has_flag(box, LV_OBJ_FLAG_HIDDEN);
        if (!box_shown)
            hide_pill = true;
    }
    /* Idempotent: lv_obj_add_flag/clear_flag invalidate unconditionally (LVGL v8
       does not short-circuit when the flag is unchanged), and this runs every
       check_is_at_home poll while the watch face is up — only touch the flag when
       the state actually changes, so we don't redraw the pill every tick. */
    bool currently_hidden = lv_obj_has_flag(bar, LV_OBJ_FLAG_HIDDEN);
    if (hide_pill == currently_hidden)
        return;
    if (hide_pill)
    {
        lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);
        /* Reset the skaibar image to full opacity for the next show — the left-edge
           reveal drives its img_opa 0→255 via bar_reveal_opa_track, so an abandoned
           reveal must not leave the next (e.g. mouse-page) standalone show dim. */
        if (s_mic_bar_icon && lv_obj_is_valid(s_mic_bar_icon))
            lv_obj_set_style_img_opa(s_mic_bar_icon, LV_OPA_COVER, 0);
    }
    else
    {
        /* 抽屜 session 期間走到這裡 = 上面兩道閘門沒攔住,pill 又要冒出來。這是
           「不該發生」分支,印出來下次一秒定位(founder 連三輪回報這顆小麥克風)。 */
        if (s_bar_single_device || s_drawer_row_shown)
            LOG_W("[drawer] !! pill un-hidden by refresh_home_bar (single=%d row=%d)",
                  (int)s_bar_single_device, (int)s_drawer_row_shown);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_HIDDEN);
    }
    /* 這支是 mic_bar 顯藏的第三個寫入點(另兩個是立起面板與抽屜三鍵列)—— 大 tap 區
       一定要跟著,否則會出現「bar 藏了但 mic_hit 還在吃 tap」的隱形擋點(R19)。 */
    mic_hit_follow_bar();
}
static bool is_at_ai_widget = false;
static bool scroll_initialized = false;
static bool touching_screen = false;
static lv_obj_t *instruction_list_page = NULL;

// 延遲設置 touching_screen 為 false 的定時器
static rt_timer_t touching_screen_timer = NULL;
static void touching_screen_timer_callback(void *parameter)
{
    touching_screen = false;
    LOG_D("touching_screen set to false after delay");
}

static void start_touching_screen_timer(void)
{
    if (!touching_screen_timer)
    {
        touching_screen_timer = rt_timer_create(
            "touching_screen_timer", touching_screen_timer_callback, NULL,
            rt_tick_from_millisecond(500), RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(touching_screen_timer);
    }
    rt_timer_start(touching_screen_timer);
}

static void stop_touching_screen_timer(void)
{
    if (touching_screen_timer)
    {
        rt_timer_stop(touching_screen_timer);
    }
}

static bool open_scroll_motor = false;
static void scroll_list(lv_obj_t *obj, int16_t drift)
{
    uint16_t min_offset = LV_VER_RES;
    uint8_t child_cnt = obj->spec_attr->child_cnt;
    lv_coord_t y_diff = 0;
    lv_coord_t y_diff2 = 0;
    lv_coord_t first_y_diff = 0;
    lv_coord_t selected_item_y_diff = 0;
    lv_coord_t last_y_diff = 0;
    lv_coord_t widget_hight = LIST_ITEM_WIDGET_HEIGHT;
    if (!scroll_initialized)
    {
        scroll_initialized = true;
    }
    if (instruction_list_page->coords.y1 == 0)
        need_correction = false;
    else
        need_correction = true;

    for (uint8_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = obj->spec_attr->children[i];
        widget_hight = LIST_ITEM_WIDGET_HEIGHT;
        lv_coord_t y_center = child->coords.y1 + widget_hight / 2;
        if (need_correction)
        {
            y_diff = y_center - LV_VER_RES / 2 - instruction_list_page->coords.y1;
        }
        else
        {
            y_diff = y_center - LV_VER_RES / 2;
        }
        if (i == 0)
        {
            first_y_diff = y_diff;
        }
        else if (i == child_cnt - 1)
        {
            last_y_diff = y_diff;
        }
        y_diff2 = y_diff;
        y_diff = LV_ABS(y_diff);
        lv_coord_t x_trans;
        if (y_diff >= LIST_RADIUS)
        {
            if (left_hand_mode)
            {
                x_trans = 0;
            }
            else
            {
                x_trans = LIST_RADIUS;
            }
        }
        else
        {
            if (y_diff < min_offset)
            {
                min_offset = y_diff;
                selected_item_y_diff = y_diff2;
                /* drag_cb 模式下 selected_item_index 由 page-change snap 統一管理，
                 * scroll_list 不要再從 card y_diff 推回去（會跟 snap 動畫打架）。
                 * 但只在 arc-drag「真的在驅動」時讓開；flag 殘留時仍跟 list scroll。*/
                if (!arc_drag_is_live())
                {
                    selected_item_index = i;
                }
            }
            rt_uint32_t x_sqr = LIST_RADIUS * LIST_RADIUS - y_diff * y_diff;
            lv_sqrt_res_t res;
            lv_sqrt(x_sqr, &res, 0x8000);
            if (left_hand_mode)
            {
                x_trans = res.i + LIST_ADJUST_X_POSITION;
            }
            else
            {
                x_trans = LIST_RADIUS - res.i;
            }
        }
        lv_obj_set_style_translate_x(child, x_trans, LV_STATE_DEFAULT);
        lv_obj_mark_layout_as_dirty(child);
        static lv_coord_t s_last_y_diff[MAX_LIST_ITEMS] = {0};
        static uint8_t last_brightness[MAX_LIST_ITEMS] = {255};
        static uint8_t s_last_zoom[MAX_LIST_ITEMS] = {0};
        const lv_coord_t DIFF_THRESHOLD = 15; // 變化超過5才更新

        if (abs((int)y_diff - (int)s_last_y_diff[i]) > DIFF_THRESHOLD)
        {
            s_last_y_diff[i] = y_diff;
            // 計算亮度值：選中時全白(255)，遠離時變暗(最暗到80)
            uint8_t brightness = 0;
            if (y_diff >= 75)
            {
                brightness = 0; // 最暗的灰色
            }
            else
            {
                // 從白色(255)漸變到暗灰(80)
                brightness = 255 - (y_diff * (255 - 0) / 75);
            }
            if (brightness != last_brightness[i])
            {
                /* R12(founder):換回**真透明度**。當年 SDK 的 per-label text_opa
                   會互相干擾(同容器的文字設一個全部跟著動),才改用顏色深淺仿
                   透明;SDK 已再更新,改回 text_opa 上機實測。字色固定白(建立
                   處設一次),這裡只動 opa。 */
                if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                {
                    lv_obj_set_style_text_opa(app_label[i], brightness, 0);
                }
                if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                {
                    lv_obj_set_style_bg_opa(switch_objs[i], brightness, 0);
                    lv_obj_t *knob = lv_obj_get_child(switch_objs[i], 0);
                    if (knob != NULL && lv_obj_is_valid(knob))
                    {
                        lv_obj_set_style_bg_opa(knob, brightness, 0);
                    }
                }
                last_brightness[i] = brightness;
            }
        }
    }
    if (touching_screen)
    {
        /* 手勢用全部項目範圍 */
        int target_value = child_cnt * 100 + first_y_diff - 63;
        if (target_value < 0)
            target_value = 0;
        else if (target_value > get_total_moving_distance())
            target_value = get_total_moving_distance();
        set_prev_sensor_quat(target_value);
    }

    /* 指示點更新移出 if(touching_screen)（founder 2026-07-23 真機 [DYN] 定位）：
     * 放手後 LVGL 的慣性/snap 仍在移動 list，label（在 item 內）跟著滑到最終置中
     * 位置；舊碼把 dot 更新關在 if(touching_screen) 內，手指一放 touching_screen
     * →false 後 dot 就停在放手瞬間的位置、追不上 label 的慣性 →「剛滑過去馬上滑」
     * 時文字已置中、圖片卻卡在半路（實測 tch=0 時 dot_y=66/310，label 恆 205）。
     * 每幀都跟 scroll 更新 dot，放手慣性期間 dot 與 label 一起 settle。drag_cb
     * 模式下 dot 由 inst_arc_drag_cb 用累積 input 自己更新，這裡照舊讓開。 */
    if (SkaiWatchSys.motion_control_lock && !arc_drag_is_live())
    {
        int dots_value = child_cnt * 100 + first_y_diff - 63;
        update_indicator_dots_position(dots_value);
    }
    if (selected_item_index != old_selected_item_index)
    {
        /* is_at_ai_widget = the selection sits on the Skai AI widget, IF the
           list has one. The list is a pure instruction list now (no AI widget),
           so this stays false; computed as a position lookup so it self-corrects
           if app_id_ai is ever re-added. (The old code indexed the array BY the
           app_id_ai ENUM VALUE, reading a wrong/out-of-bounds slot — and would
           read past the end now that the array is empty.) */
        is_at_ai_widget = false;
        for (uint8_t ai = 0;
             ai < INSTRUCTION_LIST_ITEMS_DEF_COUNT; ai++)
        {
            if (INSTRUCTION_LIST_ITEMS_DEFINITION[ai] == app_id_ai)
            {
                is_at_ai_widget = (selected_item_index == ai);
                break;
            }
        }
        set_paused_control_with_arm(false);
        if (selected_item_index == child_cnt - 1)
        {
            last_y_diff_on_selected = last_y_diff;
        }
        /* SKAIBAR option-tracking: report the scrolled-into-focus custom
           instruction as the current skaibar option, same wire format
           as hid_mouse arc selection. Gated on s_skaibar_tracking_active
           rather than is_open_instruction_list_ai so the phone-side
           option highlight keeps following the watch's scroll even
           after the user dismisses the input pill (founder direction
           2026-05-19). Tracking is armed in tap_on_ai_widget and
           cleared in instruction_list_pause. Only fires for custom
           instructions (selected >= app_base_count); the app slots
           before the custom list aren't part of the skaibar option
           set. Dedup is the outer selected_item_index != old check,
           so we don't spam the same idx on every scroll-list re-compute. */
        /* Phone instructions only: the pinned Settings and the DEFAULT_APPS body
           are local apps (is_instruction == false), not skaibar options. */
        if (selected_item_index >= app_base_count &&
            selected_item_index < list_item_count &&
            list_items[selected_item_index].is_instruction)
        {
            uint8_t opt_idx =
                (uint8_t)(selected_item_index - app_base_count);
            /* 滑鼠 app 單設備:手機把清單反轉送來(讓手錶 bottom-up 渲染對齊電腦 top-down),所以
               手錶看到的第 opt_idx 個 = 電腦第 (N-1-opt_idx) 列。送給電腦 highlight 的序號要翻回去,
               否則 highlight/執行會對到鏡像的另一列。N = 設備選項數。 */
            if (s_bar_single_device)
            {
                uint8_t n = (uint8_t)(list_item_count - app_base_count);
                if (n > 0 && opt_idx < n) opt_idx = (uint8_t)(n - 1 - opt_idx);
            }
            /* Mirror the right device_pager: report focus over SKAI_LINK,
               targeting the watch's directly-connected device (primary). Always
               sends on scroll (no skaibar-mode gate) for parity with the right. */
            /* …EXCEPT while the user is actually on the RIGHT device_pager with its
               skaibar open (controlling a remote device). When the phone app is
               foregrounded, its own Skaibar launcher mirrors "問 AI：<transcript>"
               into THIS left list as an instruction batch; the resulting
               programmatic SCROLL would otherwise fire assert_local_target() →
               commu_send_active_device("") and clear the active remote device
               MID-DICTATION (then the phone drops the relay to the device and the
               rest of the dictation leaks to the phone's own launcher; option
               commit/focus stop reaching the device). The left list is background
               here, not the controlled surface, so it must not touch the active
               target. (Confirmed via paired phone+watch logs 2026-05-29.) */
            extern bool device_pager_skaibar_is_open(void);
            if (!device_pager_skaibar_is_open())
            {
                instruction_list_assert_local_target();
                commu_send_option_focus(opt_idx);
            }
            LOG_D("[left] focus option idx=%u (raw=%u, base=%u)",
                  (unsigned)opt_idx,
                  (unsigned)selected_item_index,
                  (unsigned)app_base_count);
        }
        old_selected_item_index = selected_item_index;
        // LOG_D("selected_app_index: %d", selected_item_index);

        LOG_D("DBGinner-loop start child_cnt=%d sel=%d", child_cnt, selected_item_index);
        for (uint8_t i = 0; i < child_cnt; i++)
        {
            lv_obj_t *child = obj->spec_attr->children[i];
            LOG_D("DBGi=%d child=%p", i, child);
            if (i == selected_item_index)
            {
                LOG_D("instruction DEBUG Selected item index: %d", i);
                if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
                    lv_obj_clear_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
                if (!list_items[i].is_instruction)
                {
                    if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                        lv_obj_clear_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                }
                else
                {
                    if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                        lv_obj_clear_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                }
                /* R28(founder:「選到 session 的地方也要有 app_icon_frame 圖片出現」):
                   選中框(app_icon_shadow)本來只給「有 icon」的項目顯示,而 conv: 的
                   session 右緣放的是設備名文字、沒有 icon,所以選到時什麼都不亮。conv
                   項一併放行 —— 框是 dot_bg 的第一個 child、設備名 label 後加,所以
                   框在底、名字在上。 */
                if ((list_items[i].icon != NULL ||
                     list_items[i].img_path[0] != '\0' ||
                     strncmp(list_items[i].id, "conv:", 5) == 0) &&
                    app_icon_shadow[i] != NULL &&
                    lv_obj_is_valid(app_icon_shadow[i]))
                    lv_obj_clear_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);
                if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                    lv_obj_clear_flag(switch_objs[i], LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                /* R28:離開選中一併收框(判定條件要跟上面顯示那支對稱,否則 conv 項的
                   框亮了就再也收不掉)。 */
                if ((list_items[i].icon != NULL ||
                     list_items[i].img_path[0] != '\0' ||
                     strncmp(list_items[i].id, "conv:", 5) == 0) &&
                    app_icon_shadow[i] != NULL &&
                    lv_obj_is_valid(app_icon_shadow[i]))
                    lv_obj_add_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);
                if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
                    lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
                if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                    lv_obj_add_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
                if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                    lv_obj_add_flag(switch_objs[i], LV_OBJ_FLAG_HIDDEN);
            }
            LOG_D("DBGi=%d before app_icon", i);
            if (app_icon[i] != NULL && lv_obj_is_valid(app_icon[i]))
                lv_obj_align(app_icon[i], LV_ALIGN_RIGHT_MID, -25, 0);
            LOG_D("DBGi=%d before app_label", i);
            if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
            LOG_D("DBGi=%d before get_child", i);
            {
                lv_obj_t *first_child = lv_obj_get_child(child, 0);
                LOG_D("DBGi=%d first_child=%p", i, first_child);
                if (first_child)
                    lv_obj_set_style_border_opa(first_child, LV_OPA_10,
                                                LV_STATE_DEFAULT);
                LOG_D("DBGi=%d after set_style_border_opa", i);
            }
        }
        LOG_D("DBGinner-loop end");
        if (get_scrolling_motor_vibrate_status() && open_scroll_motor)
        {
            LOG_D("DBGcalling motor_pattern_scrolling_app");
            motor_pattern_scrolling_app();
            LOG_D("DBGafter motor_pattern_scrolling_app");
        }
        LOG_D("DBGscroll_list returning");
    }
}

static uint8_t prev_app_scroll_target_item = 0;
void open_selected_widget(bool need_widget_img_anima)
{
    if (is_widget_animation_active || created == false)
    {
        return;
    }

    /* Check if selected item has a widget */
    if (selected_item_index >= list_item_count ||
        app_widget[selected_item_index] == NULL)
    {
        if (selected_item_index < list_item_count &&
            touch_obj[selected_item_index] != NULL &&
            lv_obj_is_valid(touch_obj[selected_item_index]))
        {
            lv_obj_clear_flag(touch_obj[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    bool selected_app_has_widget =
        (app_widget[selected_item_index] != NULL &&
         lv_obj_is_valid(app_widget[selected_item_index]));

    if (!selected_app_has_widget)
    {
        if (touch_obj[selected_item_index] != NULL &&
            lv_obj_is_valid(touch_obj[selected_item_index]))
        {
            lv_obj_clear_flag(touch_obj[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    if (widget_img != NULL && lv_obj_is_valid(widget_img))
    {
        lv_obj_del(widget_img);
        widget_img = NULL;
    }

    is_widget_animation_active = true;
    lv_obj_add_flag(app_icon[selected_item_index], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(app_label[selected_item_index], LV_OBJ_FLAG_HIDDEN);
    if (need_widget_img_anima && app_widget[selected_item_index] != NULL &&
        lv_obj_is_valid(app_widget[selected_item_index]))
    {
        lv_obj_t *app_widget_img =
            lv_img_create(p_instruction_list_layout->p_instruction_list_bg);
        /* The selected-item snapshot image sits on p_instruction_list_bg — a
           SIBLING of the scrollable list, NOT inside it — so a press on it would
           bubble to the bg, never reaching the list's drawer-drag handler (only
           text-area presses, which land on the item's touch_obj inside the list,
           worked). Clear CLICKABLE so the press falls THROUGH to the centred
           item's touch_obj below → horizontal drag-to-close + tap-to-open both
           work on the image. (lv_obj is clickable by default here.) */
        lv_obj_clear_flag(app_widget_img, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_radius(app_widget_img, 50, LV_STATE_DEFAULT);
        lv_img_dsc_t *img_desc =
            create_widget_snapshot_img(app_widget[selected_item_index]);
        lv_img_set_src(app_widget_img, img_desc);
        widget_img = app_widget_img;
        lv_obj_add_flag(widget_img, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(widget_img, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_img_opa(widget_img, LV_OPA_0, LV_STATE_DEFAULT);
        animate_widget_img_opa(widget_img);
        if (selected_item_index != 0)
        {
            if (app_icon[selected_item_index - 1] != NULL &&
                lv_obj_is_valid(app_icon[selected_item_index - 1]))
                animate_icon_vertical(app_icon[selected_item_index - 1], true);
            if (app_label[selected_item_index - 1] != NULL &&
                lv_obj_is_valid(app_label[selected_item_index - 1]))
                animate_label_vertical(app_label[selected_item_index - 1],
                                       true);
        }
        if (selected_item_index != list_item_count - 1)
        {
            if (app_icon[selected_item_index + 1] != NULL &&
                lv_obj_is_valid(app_icon[selected_item_index + 1]))
                animate_icon_vertical(app_icon[selected_item_index + 1], false);
            if (app_label[selected_item_index + 1] != NULL &&
                lv_obj_is_valid(app_label[selected_item_index + 1]))
                animate_label_vertical(app_label[selected_item_index + 1],
                                       false);
        }
    }
    else
    {
        is_widget_animation_active = false;
        lv_obj_clear_flag(app_widget[selected_item_index], LV_OBJ_FLAG_HIDDEN);
        if (touch_obj[selected_item_index] != NULL &&
            lv_obj_is_valid(touch_obj[selected_item_index]))
        {
            lv_obj_clear_flag(touch_obj[selected_item_index],
                              LV_OBJ_FLAG_HIDDEN);
        }
        if (selected_item_index != 0)
        {
            if (app_icon[selected_item_index - 1] != NULL &&
                lv_obj_is_valid(app_icon[selected_item_index - 1]))
            {
                lv_obj_align(app_icon[selected_item_index - 1],
                             LV_ALIGN_RIGHT_MID, -20, -30);
            }

            if (app_label[selected_item_index - 1] != NULL &&
                lv_obj_is_valid(app_label[selected_item_index - 1]))
            {
                lv_obj_align(app_label[selected_item_index - 1],
                             LV_ALIGN_CENTER, -20, -30);
            }
        }
        if (selected_item_index != list_item_count - 1)
        {
            if (app_icon[selected_item_index + 1] != NULL &&
                lv_obj_is_valid(app_icon[selected_item_index + 1]))
            {
                lv_obj_align(app_icon[selected_item_index + 1],
                             LV_ALIGN_RIGHT_MID, -20, 30);
            }
            if (app_label[selected_item_index + 1] != NULL &&
                lv_obj_is_valid(app_label[selected_item_index + 1]))
            {
                lv_obj_align(app_label[selected_item_index + 1],
                             LV_ALIGN_CENTER, -30, 30);
            }
        }
    }
    if (selected_label != NULL && lv_obj_is_valid(selected_label))
    {
        lv_obj_set_style_border_opa(selected_label, LV_OPA_30,
                                    LV_STATE_DEFAULT);
    }
}


static rt_timer_t selected_widget_timer = NULL;

/* Forward decls — defined further down with the AI widget block. */
static void ai_widget_fade_on_scroll(void);
void close_ai_widget(void);
/* Slide-drawer anim exec — used by the horizontal finger-drag in
   list_window_scroll_event_cb; defined further down by the open/close morph. */
static void inst_list_slide_anim_cb(void *var, int32_t v);
/* R3 stage3 finger-follow blur — shared by the reveal (overlay) and the left-drag
   close (hdrag) so the dial blur tracks the list both ways. Defined with the
   reveal block below. */
static void dial_blur_track(lv_coord_t tx);
/* Finger-follow page dim for the mouse-page list — shared by the reveal/settle and
   the manual left-drag close so the dim tracks the list both ways. Defined below. */
static void page_dim_track(lv_coord_t tx);
static void reveal_settle_anim_cb(void *var, int32_t v);
/* Gate for ai_widget_fade_on_scroll: set true while refresh_custom_instructions
   does programmatic scrolls. Declared here (not at refresh_* location) so
   list_window_scroll_event_cb can read it. Actually defined below. */
static bool s_in_refresh_scroll = false;
/* R50(founder:「進場過程停在 actions 最下面,停一下又跑到第二個 action,沒到第一個
   session」):進場那一刻清單裡常常**還沒有 session** —— 它們是手機稍後推來、由
   session pager 注入的,所以 R49 的落點在 reset 當下找不到 conv 項,只能退回 actions
   最下面;等 sessions 到了、清單重建,走的卻是「還原原本選中項」那條路,於是停在一個
   action 上。這個旗標把「這次是進場」記到重建之後:只要使用者還沒自己捲動過,每次重建
   都重新套用 R49 落點(最新的 session)。使用者一動手就熄掉,不再搶他的位置。 */
static bool s_entry_landing_pending = false;
/* R80:進場落點寬限窗 —— 兩台桌面的清單分包晚到(相隔可達數秒),第一包落完就熄掉
   pending 會讓第二包(往往才是全域最新)不再補正(founder:「選中的還是在 desktop
   session 第一個」)。進場後 8 秒內的每次重建都重新落到最新 session;使用者一捲動
   (真手勢)或超時就凍結,不會在人停留頁面很久之後還亂跳(R53 的原意保留)。 */
#define ENTRY_LANDING_GRACE_MS 8000
static rt_tick_t s_entry_landing_tick = 0;
/* Latched true when a horizontal drag (the left-to-close drawer) is detected on
   the list, so the item CLICKED LVGL also raises on the release — the VER-scroll
   list never loses the press on a horizontal drag — is ignored: a swipe must not
   select an item. Set while finger-dragging (PRESSING) and by the GESTURE flick;
   reset on the next PRESSED. */
static bool s_list_horiz_swipe = false;
/* Horizontal finger-following drawer drag (drag the whole list left to close).
   s_hdrag_active latches once a press is judged horizontal; the start point lets
   PRESSING compute the live offset. */
static bool s_hdrag_active = false;
static lv_coord_t s_hdrag_start_x = 0;
static lv_coord_t s_hdrag_start_y = 0;

static void list_window_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;
    switch (evt->code)
    {
    case LV_EVENT_SCROLL_BEGIN:
    {
        // 開始滾動時停止延遲定時器
        stop_touching_screen_timer();
        if (is_user_touching_screen())
        {
            if (!touching_screen)
            {
                touching_screen = true;
            }
        }
        // 當開始滾動時停止所有動畫並復原狀態
        stop_all_animations_and_reset();
        /* R3: a list scroll = switch to manual browse — collapse the voice box
           back to the bar but KEEP the list up (ai_widget_fade_on_scroll). It does
           NOT slide the whole list out; that's the full close (swipe-right / tap /
           leave). Gated on !s_in_refresh_scroll so the programmatic scrolls
           refresh_custom_instructions makes don't collapse the box. */
        if (!s_in_refresh_scroll)
        {
            /* R76 diagnostics: founder reports scrolling does NOT collapse the
               open voice box on the session page — log what this path sees. */
            LOG_W("[scr] begin is_open=%d refresh=%d touch=%d",
                  (int)is_open_instruction_list_ai, (int)s_in_refresh_scroll,
                  (int)is_user_touching_screen());
            ai_widget_fade_on_scroll();
            /* R50:使用者自己捲了 → 進場落點的任務結束,之後的重建保留他的位置。
               R78(founder:「開機後第一次進 session 列表停在 actions 最下面,再進
               一次才對」):reset_list_internal 進場的**程式化** scroll_to_view 也
               會發 SCROLL_BEGIN,走到這裡把 pending 熄掉 → sessions 晚一步注入的
               那次重建就不再補正。只有手真的在螢幕上才算使用者捲動。 */
            if (is_user_touching_screen())
                s_entry_landing_pending = false;
        }
        // LOG_D("APP LIST Scroll begin");
        break;
    }
    case LV_EVENT_SCROLL:
    {
        if (is_user_touching_screen())
        {
            if (!touching_screen)
            {
                touching_screen = true;
            }
        }
        scroll_list(obj, 0);
        break;
    }
    case LV_EVENT_SCROLL_END:
    {
        if (prev_app_scroll_target_item != selected_item_index)
        {
            prev_app_scroll_target_item = selected_item_index;
        }
        if (!is_user_touching_screen())
        {
            // 延遲 0.5 秒後才設置 touching_screen 為 false
            start_touching_screen_timer();
            /* Settle (finger already lifted): re-materialise the selected row's
               tidy widget card. scroll_list leaves every row in its browse
               state — raw icon pinned RIGHT_MID, label CENTER — which reads as
               "icon and label don't line up". The re-tidy lives in
               open_selected_widget(), which historically only ran as a side
               effect of reset_list_internal (reveal / phone-push refresh).
               Gating that refresh reset (so it stops yanking scroll to the
               bottom) removed the incidental re-tidy, so settle must now do it
               itself — otherwise the split state persists after release. */
            open_selected_widget(false);
            /* Pin the dot arc onto the selected item's canonical input so the
               image column lines up with the (list-snapped) label. label and dot
               are two independent positioners — label = list-scroll snap, dot =
               arc angle from input_value. On fast consecutive swipes / after a
               phone-push rebuild the dot input can lag the label's selected index
               (photo: label centred on item 0 but the whole dot arc pushed up),
               so force it to the canonical idx→input = 100*(N-idx)-63 (identical
               to the touch-path child_cnt*100 + first_y_diff - 63 once first_y_diff
               has settled — verified against live [DYN]: sel1→37, sel0→137). */
            /* Finger is already up here (outer !is_user_touching_screen) — this IS
               the settle, so pin UNCONDITIONALLY. The earlier !arc_drag_is_live()
               guard was wrong: after an arc-drag then a quick list-scroll, the arc
               residual (<120ms) made is_live() true and skipped the pin exactly when
               it was needed, leaving the dot stranded mid-arc while the label snapped
               (founder photo: 手電筒 label centred, its dot low, 空圈 high). No mid-
               drag risk — during a real drag the finger is down and we never reach
               here. */
            if (SkaiWatchSys.motion_control_lock &&
                selected_item_index < list_item_count)
            {
                /* Cancel any in-flight arc-release snap anim FIRST: it animates the
                   dot toward canonical(s_inst_drag_last_idx), which occasionally
                   diverges from the list's selected (sel=1 but last_idx=0) and would
                   drag the dot back to the sel=0 arc config AFTER this pin — the
                   intermittent "sometimes offset" ([PIN] sel=1 → dot_y=310). Then
                   sync arc state to the list's selected so nothing pulls it away. */
                lv_anim_del(&s_inst_snap_anim_dummy, inst_snap_anim_exec_cb);
                s_inst_drag_last_idx = (int)selected_item_index;
                s_inst_drag_input =
                    100 * ((int)list_item_count - (int)selected_item_index) - 63;
                update_indicator_dots_position(s_inst_drag_input);
            }
        }
        // LOG_D("APP LIST Scroll ended :%d", touching_screen);
        break;
    }
    case LV_EVENT_PRESSED:
    {
        /* Fresh touch: clear the swipe latch + drag state, record the start point
           for the horizontal drawer drag below. */
        s_list_horiz_swipe = false;
        s_hdrag_active = false;
        lv_indev_t *indev = lv_indev_get_act();
        if (indev)
        {
            lv_point_t pt;
            lv_indev_get_point(indev, &pt);
            s_hdrag_start_x = pt.x;
            s_hdrag_start_y = pt.y;
        }
        break;
    }
    case LV_EVENT_PRESSING:
    {
        /* Horizontal finger-following drawer: once a press is clearly horizontal,
           the WHOLE list tracks the finger rightward (and the item CLICKED is
           suppressed). A vertical drag is left to the list's own scroll, so we
           never translate then — the |dx|>|dy| test picks the axis. RIGHT drawer:
           the list rests at 0 and closes by dragging out to the right (+HOR). */
        lv_indev_t *indev = lv_indev_get_act();
        if (!indev) break;
        lv_point_t pt;
        lv_indev_get_point(indev, &pt);
        lv_coord_t dx = pt.x - s_hdrag_start_x;
        lv_coord_t dy = pt.y - s_hdrag_start_y;
        if (!s_hdrag_active && LV_ABS(dx) > 10 && LV_ABS(dx) > LV_ABS(dy))
            s_hdrag_active = true;
        if (s_hdrag_active)
        {
            s_list_horiz_swipe = true;
            /* Drawer closes back toward its entry edge: a right-revealed list drags
               out right (tx 0..+HOR); a left-revealed one drags out left (tx 0..-HOR). */
            lv_coord_t tx = dx;
            if (s_reveal_from_left)
            {
                if (tx > 0) tx = 0;
                if (tx < -LV_HOR_RES) tx = -LV_HOR_RES;
            }
            else
            {
                if (tx < 0) tx = 0;
                if (tx > LV_HOR_RES) tx = LV_HOR_RES;
            }
            lv_obj_t *bg = p_instruction_list_layout
                               ? p_instruction_list_layout->p_instruction_list_bg
                               : NULL;
            if (bg && lv_obj_is_valid(bg))
            {
                lv_anim_del(bg, inst_list_slide_anim_cb);
                lv_anim_del(bg, reveal_settle_anim_cb);
                lv_obj_set_style_translate_x(bg, tx, 0);
                dial_blur_track(tx); /* finger-follow blur fade-out as it drags right */
                page_dim_track(tx);  /* finger-follow page dim: lighten back as it drags out */
            }
        }
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
    {
        if (!s_hdrag_active) break;
        s_hdrag_active = false;
        lv_obj_t *bg = p_instruction_list_layout
                           ? p_instruction_list_layout->p_instruction_list_bg
                           : NULL;
        if (!bg || !lv_obj_is_valid(bg)) break;
        lv_coord_t tx = lv_obj_get_style_translate_x(bg, 0);
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t v = {0, 0};
        if (indev) lv_indev_get_vect(indev, &v);
        /* Past a quarter-screen OR flung right fast → finish the close (close_ai_widget
           slides the rest out from the current offset). Otherwise snap back open. */
        /* Dragged out past ~a quarter toward the entry edge, or flung that way →
           finish the close; else snap back open. Mirror per side. */
        bool past_close = s_reveal_from_left ? (tx <= -LV_HOR_RES / 4 || v.x < -6)
                                             : (tx >= LV_HOR_RES / 4 || v.x > 6);
        if (!lv_obj_has_flag(bg, LV_OBJ_FLAG_HIDDEN) && past_close)
        {
            s_close_is_cancel = true; /* swipe-to-close = user cancel */
            close_ai_widget();
        }
        else
        {
            lv_anim_del(bg, inst_list_slide_anim_cb);
            lv_anim_del(bg, reveal_settle_anim_cb);
            lv_anim_t sl;
            lv_anim_init(&sl);
            lv_anim_set_var(&sl, bg);
            lv_anim_set_values(&sl, tx, 0);
            lv_anim_set_time(&sl, 150);
            lv_anim_set_path_cb(&sl, lv_anim_path_ease_out);
            lv_anim_set_exec_cb(&sl, reveal_settle_anim_cb);
            lv_anim_start(&sl);
        }
        break;
    }
    case LV_EVENT_GESTURE:
        /* Belt-and-suspenders: LVGL's own horizontal flick also latches the swipe
           so the item CLICKED is suppressed even if the press was too quick for
           the PRESSING drag to accumulate. The close itself is the drawer drag
           above (PRESSING / RELEASED). */
        {
            lv_dir_t gdir = lv_indev_get_gesture_dir(lv_indev_get_act());
            if (gdir == LV_DIR_LEFT || gdir == LV_DIR_RIGHT)
                s_list_horiz_swipe = true;
        }
        break;
    default:
        break;
    }
    if (obj == NULL)
    {
        return;
    }
}

extern void check_is_at_instruction_list(void);
extern void set_skai_widget_opa(uint8_t opa);
extern void set_skai_widget_input_text(const char *text);

/* Mic status: animate ai_voice_btn opacity based on VAD */
static void ai_voice_btn_opa_anim_cb(void *obj, int32_t value)
{
    lv_obj_set_style_bg_opa((lv_obj_t *)obj, value, 0);
}

void handle_ai_voice_btn_vad(bool speaking)
{
    if (ai_voice_btn && lv_obj_is_valid(ai_voice_btn))
    {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, ai_voice_btn);
        lv_anim_set_values(&a, lv_obj_get_style_bg_opa(ai_voice_btn, 0),
                           speaking ? LV_OPA_30 : LV_OPA_10);
        lv_anim_set_exec_cb(&a, ai_voice_btn_opa_anim_cb);
        lv_anim_set_time(&a, 200);
        lv_anim_start(&a);
    }
}

/* Animate skai_widget fade-in */
static bool skai_widget_shown = false;
static void skai_fade_in_anim_cb(void *var, int32_t value)
{
    set_skai_widget_opa((uint8_t)value);
}

/* Called from app_skai.c when speech text is updated.
 *
 * Was Phase 2 of a three-element reveal: (a) show ai_gaus_bg Gaussian
 * blur backdrop, (b) raise p_instruction_list_ai_bg bg_opa from 0→50
 * (dark scrim), (c) fade skai_widget pill in. User direction
 * 2026-05-19: the input pill should appear in isolation — no scrim,
 * no blur backdrop, so the instruction list behind it stays readable.
 * Only the pill fade-in (c) survives. send_icon stays revealed because
 * it sits inside the pill, not as a separate backdrop element. */
void instruction_ai_show_skai_widget(void)
{
    if (!is_open_instruction_list_ai || !p_instruction_list_layout)
        return;
    if (skai_widget_shown)
        return;
    skai_widget_shown = true;
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
    {
        lv_obj_clear_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, NULL);
    lv_anim_set_values(&a, 0, LV_OPA_COVER);
    lv_anim_set_time(&a, 300);
    lv_anim_set_exec_cb(&a, skai_fade_in_anim_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
}

static rt_tick_t last_ai_widget_open_time = 0;
static bool ai_widget_opened_by_drag = false;

/* Mock instruction-update timer — simulates phone/PC pushing list updates
   while voice listening is active. Real backend wiring deferred per office-hours
   doc §5 P2 (latency unverified). Cycles a small fake set. */
static rt_timer_t mock_inst_update_timer = RT_NULL;
static const char *MOCK_INST_IDS[] = {
    "mock-timer-001",
    "mock-msg-002",
    "mock-music-003",
    "mock-grocery-004",
};
#define MOCK_INST_COUNT (sizeof(MOCK_INST_IDS) / sizeof(MOCK_INST_IDS[0]))


static void stop_mock_inst_update(void)
{
    if (mock_inst_update_timer != RT_NULL)
    {
        rt_timer_stop(mock_inst_update_timer);
        LOG_I("mock instruction update timer stopped");
    }
}

/* Click handler for the bottom mic bar — re-uses the same flow that the release
   IMU gesture takes (see gesture_recognition_task.c:401). Only TAPs arrive here:
   mic_bar has PRESS_LOCK cleared, so a DRAG transfers the press down to the
   bottom status_bar_area (which finger-follows the watch-face bottom-up = app
   list gesture); only a stationary tap fires LV_EVENT_CLICKED. */
/* ── 舉起帶出的 skaibar:底部 bar 長按 = 對講機語音 ─────────────────────────
   滑鼠 app「錶面立起」帶出 browse 列表後,按住底部 bar 直接啟動 V2T_INTENT_SKAIBAR
   語音(不 morph 輸入框),放開即停 — 轉錄經手機以單設備模式送到電腦 skaibar,電腦端
   完全等於某個已存 action 標題就直接執行。視覺:micro_icon → micro_open_icon +
   同色(0x5DA8FF,同 hid_mouse KBD_MIC_PULSE_COLOR,取自 micro_open_icon 的淺藍)
   藍圈從 icon 中心放大漸淡、循環重播。只在「舉起」帶出的 session 生效
   (s_opened_by_lift) — 手動 tap 的流程維持原兩段式(2nd tap 開 box+語音)。 */
static bool s_opened_by_lift = false;   /* 本次 drawer session 由舉起手勢帶出 */
static bool s_mic_lp_consumed = false;  /* 長按已處理→吃掉隨後的 CLICKED */
static bool s_bar_voice_active = false; /* bar 長按語音進行中 */
static lv_obj_t *s_mic_ripple = NULL;   /* 藍圈脈衝(mic_bar 子物件,隨層鏈刪) */

#define LMIC_RIPPLE_COLOR 0x5DA8FF
#define LMIC_RIPPLE_MIN_D 48
#define LMIC_RIPPLE_MAX_D 176
#define LMIC_RIPPLE_PERIOD_MS 1200
/* 圖示與藍圈圓心的垂直偏移(64px 原生大小置中即可,不會被底緣切到) */
#define LMIC_ICON_Y_OFS 0

/* v: 0..256 — 直徑 MIN→MAX、邊框 opa COVER→0,結束跳回中心重播(不回放)。 */
static void mic_ripple_anim_cb(void *var, int32_t v)
{
    lv_obj_t *ring = (lv_obj_t *)var;
    if (!ring || !lv_obj_is_valid(ring))
        return;
    lv_coord_t d = LMIC_RIPPLE_MIN_D +
                   (lv_coord_t)((LMIC_RIPPLE_MAX_D - LMIC_RIPPLE_MIN_D) * v / 256);
    lv_obj_set_size(ring, d, d);
    lv_obj_align(ring, LV_ALIGN_CENTER, 0, LMIC_ICON_Y_OFS); /* 圓心對齊上移後的圖示 */
    lv_obj_set_style_border_opa(ring, LV_OPA_COVER - (LV_OPA_COVER * v / 256), 0);
}

static void mic_bar_voice_visual(bool on)
{
    if (s_mic_bar_icon && lv_obj_is_valid(s_mic_bar_icon))
        lv_img_set_src(s_mic_bar_icon, on ? &micro_open_icon : &micro_icon);
    if (on)
    {
        if ((!s_mic_ripple || !lv_obj_is_valid(s_mic_ripple)) &&
            p_instruction_list_layout && p_instruction_list_layout->mic_bar &&
            lv_obj_is_valid(p_instruction_list_layout->mic_bar))
        {
            s_mic_ripple = lv_obj_create(p_instruction_list_layout->mic_bar);
            lv_obj_remove_style_all(s_mic_ripple);
            lv_obj_set_style_border_color(s_mic_ripple,
                                          lv_color_hex(LMIC_RIPPLE_COLOR), 0);
            lv_obj_set_style_border_width(s_mic_ripple, 3, 0);
            lv_obj_set_style_radius(s_mic_ripple, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_opa(s_mic_ripple, LV_OPA_TRANSP, 0);
            lv_obj_add_flag(s_mic_ripple, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
            /* 圈只是視覺 — 不可吃掉按住/放開的指標事件 */
            lv_obj_clear_flag(s_mic_ripple, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(s_mic_ripple, LV_OBJ_FLAG_SCROLLABLE);
        }
        if (s_mic_ripple && lv_obj_is_valid(s_mic_ripple))
        {
            lv_obj_clear_flag(s_mic_ripple, LV_OBJ_FLAG_HIDDEN);
            lv_anim_del(s_mic_ripple, mic_ripple_anim_cb);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, s_mic_ripple);
            lv_anim_set_values(&a, 0, 256);
            lv_anim_set_time(&a, LMIC_RIPPLE_PERIOD_MS);
            lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_set_exec_cb(&a, mic_ripple_anim_cb);
            lv_anim_start(&a);
        }
    }
    else if (s_mic_ripple && lv_obj_is_valid(s_mic_ripple))
    {
        lv_anim_del(s_mic_ripple, mic_ripple_anim_cb);
        lv_obj_add_flag(s_mic_ripple, LV_OBJ_FLAG_HIDDEN);
    }
}

static void mic_bar_voice_start(void)
{
    if (s_bar_voice_active)
        return;
    /* NOTE (founder 2026-07-23): voice-input "home to newest" was tried here via
       instruction_list_force_scroll_to_last() but the post-refresh scroll left the
       selected label mis-placed (dropped low + overlapping the old label), only
       corrected by a manual scroll — same label-positioning tangle as the icon/label
       alignment work. Reverted to avoid the worse regression; do it cleanly together
       with the alignment fix as a follow-up. */
#ifndef BSP_USING_PC_SIMULATOR
    /* 沒手機=沒轉錄:不進聽音狀態(同 animate_open_ai_widget 的 BT gate 精神) */
    extern bool get_bluetooth_connection_status(void);
    if (!get_bluetooth_connection_status())
        return;
    /* 不重送 0x0E:hold 流程從列表開啟(已 latch 單設備)到按住之間沒有任何中途
       dismiss,再送一次只會讓電腦端 re-summon + 重新置中 → 面板跳位(founder
       2026-07-06 實測)。2nd-tap 分支需要重 latch 是因它中間夾了 box-close 的
       dismiss;這裡沒有。 */
    set_ai_open_mic(true);
    voice_set_pending_v2t_intent(V2T_INTENT_SKAIBAR);
    voice_provider.start_v2t();
#endif
    s_bar_voice_active = true;
    mic_bar_voice_visual(true);
    LOG_I("[bar_voice] hold-to-talk START (lift session)");
}

static void mic_bar_voice_stop(void)
{
    if (!s_bar_voice_active)
        return;
    s_bar_voice_active = false;
#ifndef BSP_USING_PC_SIMULATOR
    /* 同 box 關閉的停止配對:async STOP 事件 + sync 清 voice2TextStatus */
    voice_provider.stop_v2t();
    stop_voice_recognition(V2T_INTENT_NOTHING);
#endif
    mic_bar_voice_visual(false);
    LOG_I("[bar_voice] hold-to-talk STOP");
}

static void mic_bar_voice_event_cb(lv_event_t *evt)
{
    switch (lv_event_get_code(evt))
    {
    case LV_EVENT_LONG_PRESSED:
    {
        if (!s_opened_by_lift)
            return; /* 只在舉起帶出的 skaibar session 生效 */
        /* box 已開(有自己的語音管線)不重入 */
        bool box_visible =
            p_instruction_list_layout &&
            p_instruction_list_layout->p_instruction_list_ai_bg &&
            !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                             LV_OBJ_FLAG_HIDDEN);
        if (box_visible)
            return;
        s_mic_lp_consumed = true; /* 放開時的 CLICKED 不要 toggle 列表 */
        mic_bar_voice_start();
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        mic_bar_voice_stop(); /* 未啟動時是 no-op */
        break;
    default:
        break;
    }
}

/* 舉起手勢帶出 skaibar 後由 hid_mouse 標記 — 讓 bar 長按語音只在此情境生效。 */
void instruction_list_mark_opened_by_lift(void)
{
    s_opened_by_lift = true;
}

/* ── 2026-07-31 founder 改版：立起姿態 = 完整「輸入面板」（取代 2026-07-16 的大麥克風）──
   舊版：只有控制中那台電腦有聚焦輸入框時才生效，畫面上只有一顆置中的大麥克風，語音逐字
   直接打進電腦上那個輸入框。新版三點改變：
     1. 不再 gate 聚焦狀態 —— 在滑鼠 app 裡立起手錶就會出現面板。
     2. 面板 = 輸入框(與 skaibar 輸入框同一張 message_widget_bg 442x252 + 兩行轉錄)，
        上方一排圖示(img_logo 一定在；icon_send 只在電腦真的有聚焦輸入框時出現)，
        下方一顆「小」麥克風(64px 原生，非舊版 128px)按住講話。
     3. 文字先「暫存」在面板 + 電腦的純輸入框裡（0x0E 帶 inputOnly），由使用者在手錶上
        決定去處：icon_send → 打進剛剛點的那個輸入框(0x1d dest=field，之後離開此模式)；
        img_logo → 當成 skaibar 查詢送出(0x1d dest=skaibar，電腦展開選項、手錶叫出同步
        的清單)。手機端在放開麥克風後會先用 AI 把口語拆字還原(「林是新,士兵的士,心臟的
        心」→「林士心」)再回推兩邊，所以送出的一定是整理後的文字。

   版面(466 圓螢幕，方塊 442x252 置中 → 上下各餘 107px)：
     圖示列中心 y = 233 - 178 = 55（圓內可用寬 300，放得下 80 + 24 + 34）
     小麥克風中心 y = 233 + 170 = 403（圓內可用寬 318）
   物件仍是獨立的一組(NOT 重用 mic_bar/ai_box) —— 那些是底部 bar 與 morph 狀態機的長期
   共用單例，這個面板要能整組顯示/隱藏而不去攪動它們。語音管線(voice_provider /
   set_ai_open_mic / V2T_INTENT_SKAIBAR)沿用與 mic_bar_voice_start/stop 完全相同的後端。 */
static lv_obj_t *s_lift_input_view = NULL;   /* 全螢幕 overlay 容器,s_global_bar_layer 子物件 */
static lv_obj_t *s_lift_input_box = NULL;    /* 置中輸入框(frame img + 轉錄) */
static lv_obj_t *s_lift_input_clip = NULL;   /* 兩行高的轉錄裁切窗(同 ai_box) */
static lv_obj_t *s_lift_input_label = NULL;  /* 轉錄文字 */
static lv_obj_t *s_lift_logo_btn = NULL;     /* 上方 img_logo — 送給 skaibar */
static lv_obj_t *s_lift_send_btn = NULL;     /* 上方 icon_send — 送回聚焦輸入框(有聚焦才顯示) */
static lv_obj_t *s_lift_mic_btn = NULL;      /* 下方小麥克風(按住講話) */
static lv_obj_t *s_lift_box_glow = NULL;     /* 錄音中沿輸入框外圈擴散的藍色脈衝 */
static lv_obj_t *s_lift_del_btn = NULL;      /* 與麥克風並排的刪除鍵 */
/* 建立這組物件的那條執行緒 = LVGL 執行緒。用「身分比對」而不是猜名字,才能安全地分辨
   「空字串是從 BLE 下行(LVGL 執行緒,可以碰 UI)」還是「從 bloc_v2t 直呼(voice_re,不能碰)」。 */
static rt_thread_t s_lift_lvgl_thread = RT_NULL;
/* 面板是否顯示中。只由 LVGL 執行緒寫,其他執行緒(周邊/通訊)唯讀 —— 見
   instruction_list_lift_input_view_open 對於為何不能用 lv_obj_is_valid 的說明。 */
static volatile bool s_lift_view_shown = false;
static lv_obj_t *s_lift_caret = NULL;        /* 長按定位出來的插入游標(細直線) */
/* 游標下方的圓球把手(founder 2026-08-01):按住它拖 = 帶著游標走,**不是**框選。框選走的是
   長按文字本身那條路,兩者刻意分開在不同物件上,不必再靠時間或位移去猜使用者要哪一種。 */
static lv_obj_t *s_lift_caret_ball = NULL;
static bool s_lift_ball_dragging = false;
static lv_coord_t s_lift_ball_grab_dy = 0; /* 按下當刻量到的「球心 → 游標中線」垂直距離 */
#define LIFT_CARET_BALL_D 22
#define LIFT_CARET_BALL_GAP 2
static bool s_lift_voice_active = false;

/* 插入點(字元索引,非 byte)。**-1 = 停在文字最尾端**(預設),不是「沒有游標」——
   founder 2026-08-01:「按過一次刪除鍵後游標就消失了,我不知道下個會刪哪裡」。游標是這個面板
   唯一的位置指示,任何時候都該看得到:預設在尾端(接續講話/刪除都從那裡發生),點或長按可以
   移到別處,刪除時跟著往前退一格。 */
static int32_t s_lift_caret_pos = -1;

/* 目前這次「按住講話」是從哪個物件開始的 —— 小麥克風,或長按輸入框的那一下。冷卻期的補開始
   要回頭確認「手指還按在同一個物件上」,所以不能寫死成麥克風。 */
static lv_obj_t *s_lift_hold_obj = NULL;

/* 長按輸入框後**手指有沒有移動**決定兩種完全不同的意圖(founder 2026-08-01):
     沒動 → 原本的「插入點 + 當場開始講」;
     一拖 → 從按下的那個字開始框選,放開後按刪除鍵整段刪掉。
   anchor 是長按當下那個字的索引;拖動時另一端跟著手指跑。範圍一律正規化成 [from,to)。 */
static int32_t s_lift_sel_anchor = -1;
static int32_t s_lift_sel_from = -1;
static int32_t s_lift_sel_to = -1;
static bool s_lift_dragging = false;
/* 12px:比手指按住時的自然抖動大,又遠小於一個字(52px 行高)的寬度。 */
#define LIFT_SEL_DRAG_SLOP 12
/* 長按到真正開錄音之間的確認窗。**不能在 LONG_PRESSED 當下就開錄音** —— 那樣使用者一拖動
   就得把剛起來的語音管線收掉,而那 0.2 秒已經錄進去的音訊還是會變成一段插進文字裡的雜訊
   (而且這條管線的 stop→start 重入本來就是已知脆弱點)。等 150ms 確認手指沒動再開,使用者
   感覺不出差別,卻讓「框選」和「講話」在時間上完全不重疊。 */
#define LIFT_LONGPRESS_ARM_MS 150

#define LIFT_BOX_W 442  /* == message_widget_bg 原生尺寸,與 skaibar 輸入框同一張圖 */
#define LIFT_BOX_H 252
/* 文字距框上下緣的留白 —— 框是圓角的,太貼邊會被弧線切到。
   20px 是實測算出來的:行高 52,框高 252,留 20 → 可用 212 → 剛好 4 行(208)。
   幾何檢查(圓角 80 + 466 圓螢幕):4 行時文字上緣 y=127,該高度圓角內縮 27px、圓弧可用
   x=39..427,而文字只佔 53..413,不會被切。留 48 會掉到 3 行,白白浪費一行。 */
#define LIFT_BOX_TEXT_INSET_V 20
/* 上限:再多行也放不進 442x252 的可視區,且行數過多在圓形螢幕上兩側會被裁。 */
#define LIFT_BOX_MAX_ROWS 4
#define LIFT_ICON_ROW_DY (-178) /* 圖示列中心相對螢幕中心:-(BOX_H/2 + s3 + 40) */
#define LIFT_MIC_DY 170         /* 小麥克風中心相對螢幕中心:+(BOX_H/2 + s3 + 32) */
/* 麥克風與刪除鍵並排(founder 2026-08-01)。麥克風 64px、刪除 46px,中心相距 92 → 邊緣間隙 37px
   (≥ s3)。y=403 那條線在圓內可用寬 318,兩顆合計 110 綽綽有餘。 */
#define LIFT_MIC_DX (-40)
#define LIFT_DEL_DX 52
#define LIFT_DEL_EXT_CLICK 12   /* 46x33 圖 → 70x57 觸控標的(≥44pt 基線) */
#define LIFT_DEL_REPEAT_MS 120  /* 長按連續刪除的速率;自己控速,不依賴 LVGL 的重複間隔 */
#define LIFT_LOGO_DX (-29)      /* 兩顆並排時:總寬 80+24+34=138,靠左起算的中心 */
#define LIFT_SEND_DX 52
#define LIFT_SEND_EXT_CLICK 15  /* icon_send 原生 34px → 34+2*15=64 觸控標的(≥44pt 基線) */
/* 錄音中的脈衝**畫在整個輸入框外圈**,不是小麥克風旁邊(founder 2026-08-01)。同一套
   「放大 + 漸淡 + 循環」公式,只是形狀從圓變成跟框同尺寸的圓角矩形,從框緣往外擴散。
   小麥克風那圈同時取消 —— 這是「移過來」不是「多加一個」。 */
#define LIFT_BOX_GLOW_GROW 40      /* 往外擴散的最大距離 */
#define LIFT_BOX_GLOW_RADIUS 80    /* 對齊 message_widget_bg 的圓角 */

static void lift_box_glow_anim_cb(void *var, int32_t v)
{
    lv_obj_t *ring = (lv_obj_t *)var;
    if (!ring || !lv_obj_is_valid(ring))
        return;
    lv_coord_t g = (lv_coord_t)(LIFT_BOX_GLOW_GROW * v / 256);
    lv_obj_set_size(ring, LIFT_BOX_W + g * 2, LIFT_BOX_H + g * 2);
    lv_obj_align(ring, LV_ALIGN_CENTER, 0, 0);
    /* 圓角跟著長大,擴散出去的框才會與原框同心而不是越變越方。 */
    lv_obj_set_style_radius(ring, LIFT_BOX_GLOW_RADIUS + g, 0);
    lv_obj_set_style_border_opa(ring, LV_OPA_COVER - (LV_OPA_COVER * v / 256), 0);
}

static void lift_input_voice_visual(bool on)
{
    if (s_lift_mic_btn && lv_obj_is_valid(s_lift_mic_btn))
        lv_img_set_src(s_lift_mic_btn, on ? &micro_open_icon : &micro_icon);
    if (!s_lift_box_glow || !lv_obj_is_valid(s_lift_box_glow))
        return;
    if (on)
    {
        lv_obj_clear_flag(s_lift_box_glow, LV_OBJ_FLAG_HIDDEN);
        lv_anim_del(s_lift_box_glow, lift_box_glow_anim_cb);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, s_lift_box_glow);
        lv_anim_set_values(&a, 0, 256);
        lv_anim_set_time(&a, LMIC_RIPPLE_PERIOD_MS);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_set_exec_cb(&a, lift_box_glow_anim_cb);
        lv_anim_start(&a);
    }
    else
    {
        lv_anim_del(s_lift_box_glow, lift_box_glow_anim_cb);
        lv_obj_add_flag(s_lift_box_glow, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 上一次停止語音的時間戳 + 冷卻時間。真機當機(2026-07-31):
     [2061976] voice STOP → [2062345] voice START(相隔 0.37s) → hard fault on thread: voice_re
   語音管線(opus/PSRAM 緩衝)的拆除不是同步完成的,在拆到一半時重新 start_v2t() 會打在
   voice 執行緒上。**這是既有管線的脆弱點,不是這個面板造成的**,但這個面板讓它變得很好踩:
   常駐面板 + 小麥克風 + 放開偵測失靈 → 使用者自然會連按第二下,而那第二下正好構成
   「STOP 後立刻 START」。這裡只擋掉觸發條件(冷卻期內不重啟),治本要修語音管線本身的
   重入保護 —— 已另案記錄。 */
static rt_tick_t s_lift_voice_stopped_at = 0;
#define LIFT_VOICE_RESTART_COOLDOWN_MS 800

/* 這一按開始錄音的時刻,與「短到不可能是一句話」的門檻。真機量到的誤觸是 179/421/632ms,
   而真正要講話的人光是「按住→開口」就不只這個時間,700 拉得開。 */
static rt_tick_t s_lift_voice_started_at = 0;
#define LIFT_VOICE_MIN_UTTERANCE_MS 700

/* 冷卻期內那一下按下的補開始(見 lift_input_voice_start 的 deferred 分支)。 */
static lv_timer_t *s_lift_voice_retry_timer = NULL;
static void lift_input_voice_start(void);

static void lift_voice_retry_cb(lv_timer_t *t)
{
    (void)t;
    s_lift_voice_retry_timer = NULL; /* one-shot,LVGL 會在 cb 回來後自動刪 */
    /* 只有「手指還按在麥克風上」才補開始 —— 使用者若在冷卻期間就放開了,那一下不算數,
       這裡補開會變成沒人按著卻在錄音。 */
    if (!s_lift_hold_obj || !lv_obj_is_valid(s_lift_hold_obj))
        return;
    if (!lv_obj_has_state(s_lift_hold_obj, LV_STATE_PRESSED))
    {
        LOG_I("[lift_input] deferred START dropped — finger already released");
        return;
    }
    LOG_I("[lift_input] deferred START firing (cooldown over, still held)");
    lift_input_voice_start();
}

static void lift_input_voice_start(void)
{
    if (s_lift_voice_active)
        return;
    if (s_lift_voice_stopped_at != 0 &&
        rt_tick_get() - s_lift_voice_stopped_at <
        rt_tick_from_millisecond(LIFT_VOICE_RESTART_COOLDOWN_MS))
    {
        /* 冷卻期內的按下**不丟掉**,排一支 one-shot 等冷卻結束再補開始(屆時手指還按著才開)。
           真機實測(2026-08-01):純粹 return 的話,使用者放開後很自然地馬上再按(0.3s),那一下被
           靜默吃掉,他就握著一支沒在錄音的麥克風繼續講 —— 比原本要防的當機更難察覺。 */
        rt_tick_t elapsed = rt_tick_get() - s_lift_voice_stopped_at;
        rt_tick_t remain_ticks = rt_tick_from_millisecond(LIFT_VOICE_RESTART_COOLDOWN_MS) - elapsed;
        uint32_t remain_ms = (uint32_t)(remain_ticks * 1000 / RT_TICK_PER_SECOND) + 20;
        LOG_I("[lift_input] voice START deferred %ums (restart cooldown)", remain_ms);
        if (s_lift_voice_retry_timer)
            lv_timer_del(s_lift_voice_retry_timer);
        s_lift_voice_retry_timer = lv_timer_create(lift_voice_retry_cb, remain_ms, NULL);
        lv_timer_set_repeat_count(s_lift_voice_retry_timer, 1);
        return;
    }
#ifndef BSP_USING_PC_SIMULATOR
    extern bool get_bluetooth_connection_status(void);
    if (!get_bluetooth_connection_status())
        return;
    set_ai_open_mic(true);
    voice_set_pending_v2t_intent(V2T_INTENT_SKAIBAR);
    voice_provider.start_v2t();
#endif
    s_lift_voice_active = true;
    s_lift_voice_started_at = rt_tick_get(); /* 放開時用來判斷這段夠不夠長 */
    lift_input_voice_visual(true);
    LOG_I("[lift_input] voice START");
}

static void lift_input_voice_stop(void)
{
    if (!s_lift_voice_active)
        return;
    s_lift_voice_active = false;
    rt_tick_t held = rt_tick_get() - s_lift_voice_started_at;
    uint32_t held_ms = (uint32_t)(held * 1000 / RT_TICK_PER_SECOND);
#ifndef BSP_USING_PC_SIMULATOR
    voice_provider.stop_v2t();
    stop_voice_recognition(V2T_INTENT_NOTHING);
#endif
    s_lift_voice_stopped_at = rt_tick_get();
    lift_input_voice_visual(false);
    LOG_I("[lift_input] voice STOP (held %ums)", held_ms);
    /* **太短的那一段整個作廢。** 使用者長按是為了移游標,手指很快就放開,於是錄到一段
       0.2~0.6 秒、裡面根本沒有話的音訊;STT 拿到空音訊會**把上一句重播回來**,那筆就被插在
       剛定位的插入點上 —— founder 2026-08-03:「我長按就莫名其妙自己插入重複的字」,手機端
       log 三次分別是 179 / 421 / 632ms,插進去的正是上一段的內容。
       重用長按轉框選那條的 cancel:這一按從來就不是一次口述。 */
    if (held_ms < LIFT_VOICE_MIN_UTTERANCE_MS)
    {
        extern bool commu_send_lift_input_cancel_segment(void);
        commu_send_lift_input_cancel_segment();
        LOG_I("[lift_input] too short to be speech — segment discarded");
    }
}

/* 小麥克風 = 按住講話(對講機),與底部 bar 的長按語音同一套後端。啟動只掛在麥克風本身而非
   整片 overlay:面板還有 logo / send 兩顆可按的圖示,整片按住講話會把它們吃掉。 */
/* 2026-07-31:這裡曾經放過一支「PRESSING 停了就收錄音」的按住看門狗 —— **已移除,它是錯的**。
   真機事件流證明 LV_EVENT_PRESSING 只在手指有位移時才送:使用者按住不動時它不來,看門狗於是
   在 600ms 後誤停錄音。那次誤停讓語音管線停在半拆狀態,下一次按下就 hard fault on thread:
   voice_re(真機當場重現)。同一份事件流也證明 LV_EVENT_RELEASED(evt=8)其實會正常送達 ——
   先前看不到 STOP 是因為使用者一直按著在等文字出現,而文字沒出現是轉錄路由的 bug(見
   watch_system_interact.c 的 lift 分支),不是放開事件遺失。
   要做「還按著」的心跳,正確的來源是 LV_EVENT_LONG_PRESSED_REPEAT(evt=6,按住期間穩定重送),
   不是 PRESSING。目前不需要 —— 放開事件本來就會到。 */
static void lift_mic_btn_event_cb(lv_event_t *evt)
{
    lv_event_code_t code = lv_event_get_code(evt);
    /* TEMP DIAG(2026-07-31):印出麥克風收到的輸入類事件。排除 PRESSING(每幀)與
       LONG_PRESSED_REPEAT(按住期間每幾秒一次)這兩個高頻事件以免洗版。定位完拿掉。 */
    if (code <= LV_EVENT_LEAVE && code != LV_EVENT_PRESSING &&
        code != LV_EVENT_LONG_PRESSED_REPEAT)
        LOG_I("[lift_input][diag] mic evt=%d", (int)code);
    switch (code)
    {
    case LV_EVENT_PRESSED:
        s_lift_hold_obj = lv_event_get_target(evt);
        lift_input_voice_start();
        break;
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        lift_input_voice_stop();
        break;
    default:
        break;
    }
}

/* 停止的**兜底**:掛在整片面板容器上,手指不論在哪裡離開螢幕都收掉錄音。
   真機實測(2026-07-31)發現只掛在 64px 麥克風圖示上不夠 —— 按下後手指稍微移動再放開,
   RELEASED 就不會落到該物件上,錄音一路卡在 active(log 見 voice START 後 10 分鐘才等到
   一次 PRESS_LOST 補送),連帶讓「放開才跑」的 AI 整理永遠不觸發。這裡只負責 STOP、不負責
   START,所以不會把 logo / send 的點擊搶走;lift_input_voice_stop() 未啟動時是 no-op,
   重複呼叫安全。 */
static void lift_input_view_event_cb(lv_event_t *evt)
{
    switch (lv_event_get_code(evt))
    {
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        lift_input_voice_stop();
        break;
    default:
        break;
    }
}

/* 面板上目前顯示的文字是否是「真的講出來的內容」(而非 Listening 佔位)。送出前的門檻:
   空面板按 logo/send 只會讓兩端狀態空轉,不如什麼都不做。 */
static bool lift_input_has_text(void); /* fwd — 定義在下面 */

/* 目前標籤上的字數(字元,非 byte)。 */
static uint32_t lift_input_char_count(void)
{
    if (!s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
        return 0;
    const char *t = lv_label_get_text(s_lift_input_label);
    return t ? _lv_txt_get_encoded_length(t) : 0;
}

/* 游標閃爍(founder 2026-08-01)。530ms 是一般文字輸入游標的節奏。只在游標可見時跑,面板收起
   或退回佔位文字就停 —— 沒必要為看不見的東西每半秒喚醒一次 LVGL。 */
#define LIFT_CARET_BLINK_MS 530
static lv_timer_t *s_lift_caret_blink = NULL;

static void lift_caret_blink_stop(void)
{
    if (s_lift_caret_blink)
    {
        lv_timer_del(s_lift_caret_blink);
        s_lift_caret_blink = NULL;
    }
}

static void lift_caret_blink_cb(lv_timer_t *t)
{
    if (!s_lift_caret || !lv_obj_is_valid(s_lift_caret))
    {
        s_lift_caret_blink = NULL;
        lv_timer_del(t); /* 物件先走一步(面板重建),timer 自己收掉 */
        return;
    }
    if (lv_obj_has_flag(s_lift_caret, LV_OBJ_FLAG_HIDDEN))
        lv_obj_clear_flag(s_lift_caret, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(s_lift_caret, LV_OBJ_FLAG_HIDDEN);
}

static bool lift_sel_active(void); /* fwd — 有框選時游標不畫,定義在下面 */

/* 依目前的 s_lift_caret_pos(-1 = 尾端)把游標畫出來。文字每次更新後都要重畫,游標才會一直在。 */
static void lift_caret_render(void)
{
    if (!s_lift_caret || !lv_obj_is_valid(s_lift_caret) ||
        !s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
        return;
    /* 有框選就不畫游標(founder 2026-08-01)——「選了一段」跟「插在某一點」是互斥的兩種狀態,
       兩個都畫只會讓人不確定下一個動作發生在哪。 */
    if (!lift_input_has_text() || lift_sel_active())
    {
        lift_caret_blink_stop();
        lv_obj_add_flag(s_lift_caret, LV_OBJ_FLAG_HIDDEN);
        if (s_lift_caret_ball && lv_obj_is_valid(s_lift_caret_ball))
            lv_obj_add_flag(s_lift_caret_ball, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    uint32_t count = lift_input_char_count();
    uint32_t pos = (s_lift_caret_pos < 0 || (uint32_t)s_lift_caret_pos > count)
                       ? count
                       : (uint32_t)s_lift_caret_pos;
    lv_point_t p;
    lv_label_get_letter_pos(s_lift_input_label, pos, &p);
    /* letter_pos 是 label 相對座標;游標與 label 同一個 parent(clip),直接沿用即可。 */
    const lv_font_t *f = lv_obj_get_style_text_font(s_lift_input_label, LV_PART_MAIN);
    lv_coord_t lh = lv_font_get_line_height(f);
    lv_obj_set_size(s_lift_caret, 2, lh);
    lv_obj_set_pos(s_lift_caret, lv_obj_get_x(s_lift_input_label) + p.x,
                   lv_obj_get_y(s_lift_input_label) + p.y);
    lv_obj_clear_flag(s_lift_caret, LV_OBJ_FLAG_HIDDEN);
    /* 圓球把手貼在游標線正下方、水平置中於那條 2px 的線。游標在**最後一行**時球會掉出
       裁切窗被切掉,所以夾回窗內(那種情況球會與文字下緣重疊,可接受;拖曳的抓取偏移是在
       按下當刻量的,不是寫死的,所以夾過位置也不會算錯字)。 */
    if (s_lift_caret_ball && lv_obj_is_valid(s_lift_caret_ball))
    {
        lv_coord_t by = lv_obj_get_y(s_lift_input_label) + p.y + lh + LIFT_CARET_BALL_GAP;
        lv_coord_t clip_h = lv_obj_get_height(s_lift_input_clip);
        if (by + LIFT_CARET_BALL_D > clip_h)
            by = clip_h - LIFT_CARET_BALL_D;
        lv_obj_set_pos(s_lift_caret_ball,
                       lv_obj_get_x(s_lift_input_label) + p.x + 1 - LIFT_CARET_BALL_D / 2, by);
        lv_obj_clear_flag(s_lift_caret_ball, LV_OBJ_FLAG_HIDDEN);
    }
    /* 拖著把手走的時候不閃 —— 實心跟著手指才看得出停在哪個字。 */
    if (s_lift_ball_dragging)
    {
        lift_caret_blink_stop();
        return;
    }
    /* 剛移動/剛更新的游標先實心亮著,再開始閃 —— 重設相位,不然它可能正好落在暗的那半拍,
       使用者按完刪除看過去以為游標又不見了。 */
    if (!s_lift_caret_blink)
        s_lift_caret_blink = lv_timer_create(lift_caret_blink_cb, LIFT_CARET_BLINK_MS, NULL);
    else
        lv_timer_reset(s_lift_caret_blink);
}

/* 把插入點設到第 pos 個字元前緣並重畫。pos<0 = 回到尾端(預設位置,不是隱藏)。 */
static void lift_input_place_caret(int32_t pos)
{
    s_lift_caret_pos = pos;
    lift_caret_render();
}

/* ── 框選 ──────────────────────────────────────────────────────────────────
   用 LVGL 標籤原生的 text selection 畫(rtconfig 的 LV_LABEL_TEXT_SELECTION=1),索引與
   游標同樣是**字元**(code point)——lv_draw_label 內部就是照 encoded length 數的,兩者對得上。 */
static bool lift_sel_active(void)
{
    return s_lift_sel_from >= 0 && s_lift_sel_to > s_lift_sel_from;
}

static void lift_sel_clear(void)
{
    s_lift_sel_anchor = -1;
    s_lift_sel_from = -1;
    s_lift_sel_to = -1;
    if (s_lift_input_label && lv_obj_is_valid(s_lift_input_label))
    {
        lv_label_set_text_sel_start(s_lift_input_label, LV_DRAW_LABEL_NO_TXT_SEL);
        lv_label_set_text_sel_end(s_lift_input_label, LV_DRAW_LABEL_NO_TXT_SEL);
    }
    lift_caret_render(); /* 選取沒了 → 游標回來 */
}

static void lift_sel_apply(int32_t a, int32_t b)
{
    if (a > b)
    {
        int32_t t = a;
        a = b;
        b = t;
    }
    s_lift_sel_from = a;
    s_lift_sel_to = b;
    if (!s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
        return;
    lv_label_set_text_sel_start(s_lift_input_label, (uint32_t)a);
    lv_label_set_text_sel_end(s_lift_input_label, (uint32_t)b);
    lift_caret_render(); /* 有選取 → 游標收起來 */
}

/* 絕對座標 → 第幾個字。座標要換成 label 相對(lv_label_get_letter_on 內部再扣 padding)。 */
static bool lift_input_letter_at(lv_point_t abs, uint32_t *out)
{
    if (!s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
        return false;
    if (!lift_input_has_text())
        return false;
    lv_point_t p = abs;
    p.x -= s_lift_input_label->coords.x1;
    p.y -= s_lift_input_label->coords.y1;
    *out = lv_label_get_letter_on(s_lift_input_label, &p);
    return true;
}

/* 長按確認窗:窗內手指沒動才開錄音。見 LIFT_LONGPRESS_ARM_MS。 */
static lv_timer_t *s_lift_longpress_arm = NULL;
static void lift_input_voice_start(void);

static void lift_longpress_arm_stop(void)
{
    if (s_lift_longpress_arm)
    {
        lv_timer_del(s_lift_longpress_arm);
        s_lift_longpress_arm = NULL;
    }
}

static void lift_longpress_arm_cb(lv_timer_t *t)
{
    (void)t;
    s_lift_longpress_arm = NULL; /* one-shot,LVGL 會在 cb 回來後自己刪 */
    if (s_lift_dragging)
        return; /* 已經變成框選了 */
    if (!s_lift_hold_obj || !lv_obj_is_valid(s_lift_hold_obj))
        return;
    if (!lv_obj_has_state(s_lift_hold_obj, LV_STATE_PRESSED))
        return; /* 確認窗內就放開了 = 不是要講話 */
    lift_input_voice_start();
}

static void lift_longpress_arm_start(void)
{
    lift_longpress_arm_stop();
    s_lift_longpress_arm = lv_timer_create(lift_longpress_arm_cb, LIFT_LONGPRESS_ARM_MS, NULL);
    lv_timer_set_repeat_count(s_lift_longpress_arm, 1);
}

/* 把觸點換算成「第幾個字」並設為插入點。回傳是否成功(還是 Listening 佔位時沒東西可定位)。
   座標要換成 label 相對(lv_label_get_letter_on 內部再扣 padding、用 content coords)。 */
/* 按下當刻的觸點。**不能**在 SHORT_CLICKED 時才去 lv_indev_get_act() 取 —— 那個事件是手指
   放開之後才送的,那時 indev 的座標已經不是使用者按的位置了(真機 2026-08-01:長按定位都正確
   /pos=4,5,7,17,點一下卻永遠回 0)。長按之所以沒事,是因為 LONG_PRESSED 在手指還按著時送。
   改成一律用按下當刻存下來的點,語意上也才是「你手指戳的那個字」。 */
static lv_point_t s_lift_press_point;
static bool s_lift_press_point_valid = false;

static bool lift_input_set_caret_from_touch(void)
{
    if (!s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
        return false;
    if (!lift_input_has_text())
        return false;
    if (!s_lift_press_point_valid)
        return false;
    uint32_t idx;
    if (!lift_input_letter_at(s_lift_press_point, &idx))
        return false;
    lift_input_place_caret((int32_t)idx);
    extern void motor_pattern_unlocked(void);
    motor_pattern_unlocked(); /* 定位成功的觸覺確認 */
    extern bool commu_send_lift_input_caret(int pos, const char *text);
    commu_send_lift_input_caret((int)idx, lv_label_get_text(s_lift_input_label));
    LOG_I("[lift_input] caret set at letter %u", (unsigned)idx);
    return true;
}

/* 圓球把手:按住拖 = 帶著游標走。跟框選刻意分在**不同物件**上 —— 拖文字是圈選,拖球是移
   游標,不必再用時間或位移去猜使用者想要哪一種。事件不會 bubble 到 clip(LVGL v8 預設不開
   EVENT_BUBBLE),所以按著球不會同時觸發長按錄音或框選。 */
static void lift_caret_ball_event_cb(lv_event_t *evt)
{
    switch (lv_event_get_code(evt))
    {
    case LV_EVENT_PRESSED:
        s_lift_ball_dragging = true;
        lift_caret_blink_stop();
        if (s_lift_caret && lv_obj_is_valid(s_lift_caret))
        {
            lv_obj_clear_flag(s_lift_caret, LV_OBJ_FLAG_HIDDEN);
            /* 球心與游標中線的垂直距離,**按下當刻量**而不是寫死 —— 最後一行的球被夾回窗內時
               這個距離會變小,寫死的話拖曳就會算到隔壁行去。 */
            s_lift_ball_grab_dy =
                (s_lift_caret_ball->coords.y1 + s_lift_caret_ball->coords.y2) / 2 -
                (s_lift_caret->coords.y1 + s_lift_caret->coords.y2) / 2;
        }
        {
            extern void motor_pattern_unlocked(void);
            motor_pattern_unlocked(); /* 抓到把手了 */
        }
        break;
    case LV_EVENT_PRESSING:
    {
        if (!s_lift_ball_dragging || !s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
            break;
        lv_indev_t *indev = lv_indev_get_act();
        if (!indev)
            break;
        lv_point_t cur;
        lv_indev_get_point(indev, &cur);
        /* 手指抓的是球,游標在球**上方**:把觸點往上移回游標那一行的中線再換算字元,
           不然拖到第二行時算出來的會是球所在的那一行。 */
        cur.y -= s_lift_ball_grab_dy;
        uint32_t idx;
        if (lift_input_letter_at(cur, &idx))
            lift_input_place_caret((int32_t)idx);
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        if (s_lift_ball_dragging)
        {
            s_lift_ball_dragging = false;
            lift_caret_render(); /* 放開才恢復閃爍 */
            extern bool commu_send_lift_input_caret(int pos, const char *text);
            commu_send_lift_input_caret((int)s_lift_caret_pos,
                                        lv_label_get_text(s_lift_input_label));
            LOG_I("[lift_input] caret dragged to letter %d", (int)s_lift_caret_pos);
        }
        break;
    default:
        break;
    }
}

/* 輸入框上的三種手勢(founder 2026-08-01,刻意分開):
     點一下(SHORT_CLICKED)= 只把游標移過去,接著用下面的麥克風錄音;
     長按不動(LONG_PRESSED + 確認窗)= 游標移過去**並開始錄音**,按著講、放開就停;
     長按後拖(LONG_PRESSED → PRESSING 超過 slop)= 從按下的那個字開始框選,放開後按刪除鍵整段刪。
   LVGL 天然分得開:一次乾淨的點擊才會送 SHORT_CLICKED,長按只送 LONG_PRESSED + CLICKED。
   「講話」與「框選」則靠 LIFT_LONGPRESS_ARM_MS 這個確認窗分開,兩者不會在時間上重疊。 */
static void lift_input_clip_event_cb(lv_event_t *evt)
{
    switch (lv_event_get_code(evt))
    {
    case LV_EVENT_PRESSED:
    {
        /* 按下當刻抓座標 —— SHORT_CLICKED / LONG_PRESSED / 拖動起點都用它。 */
        lv_indev_t *indev = lv_indev_get_act();
        s_lift_press_point_valid = (indev != NULL);
        if (indev)
            lv_indev_get_point(indev, &s_lift_press_point);
        lift_sel_clear(); /* 新的一次觸碰 = 放掉上一次的選取 */
        s_lift_dragging = false;
        /* TEMP DIAG(2026-08-01):點框定位沒反應時,要分得出「事件沒送到」還是「座標算錯」。 */
        LOG_I("[lift_input][diag] box PRESSED at (%d,%d) valid=%d",
              (int)s_lift_press_point.x, (int)s_lift_press_point.y,
              (int)s_lift_press_point_valid);
        break;
    }
    case LV_EVENT_SHORT_CLICKED:
        LOG_I("[lift_input][diag] box SHORT_CLICKED"); /* TEMP DIAG */
        lift_input_set_caret_from_touch();
        break;
    case LV_EVENT_LONG_PRESSED:
        /* 框裡沒字時也要能長按開始講話(founder 2026-08-01)——定位不到字不代表不能錄音,
           兩件事本來就該分開:定位成功 → 順便當框選起點;定位失敗(空框)→ 純粹開始講。 */
        s_lift_hold_obj = lv_event_get_target(evt);
        s_lift_sel_anchor = lift_input_set_caret_from_touch() ? s_lift_caret_pos : -1;
        lift_longpress_arm_start();
        break;
    case LV_EVENT_PRESSING:
    {
        /* PRESSING 只在觸點**移動**時才送,所以這裡天然就是「手指有動」。 */
        if (s_lift_sel_anchor < 0)
            break;
        lv_indev_t *indev = lv_indev_get_act();
        if (!indev)
            break;
        lv_point_t cur;
        lv_indev_get_point(indev, &cur);
        lv_coord_t dx = cur.x - s_lift_press_point.x;
        lv_coord_t dy = cur.y - s_lift_press_point.y;
        if (!s_lift_dragging &&
            (int32_t)dx * dx + (int32_t)dy * dy < LIFT_SEL_DRAG_SLOP * LIFT_SEL_DRAG_SLOP)
            break; /* 還在抖動範圍內,先當作沒動 */
        if (!s_lift_dragging)
        {
            s_lift_dragging = true;
            bool was_recording = s_lift_voice_active;
            lift_longpress_arm_stop(); /* 確認窗作廢:這是框選不是講話 */
            lift_input_voice_stop();
            /* **停止錄音還不夠,那一段要整個丟掉。** 真人的手勢是「長按 → 停一下 → 才拖」,
               不是 150ms 內立刻拖:真機 2026-08-01 實測長按到開始拖隔了 5 秒,那 5 秒早就錄進去
               並插進文字裡了(founder 看到文字一直變長)。founder 的語意是「移動就是圈選」——
               既然這一按最後被判定成框選,它就從來不是一次口述。 */
            if (was_recording)
            {
                extern bool commu_send_lift_input_cancel_segment(void);
                commu_send_lift_input_cancel_segment();
                LOG_I("[lift_input] drag started mid-dictation — segment discarded");
            }
            extern void motor_pattern_unlocked(void);
            motor_pattern_unlocked(); /* 進入框選的觸覺提示 */
            LOG_I("[lift_input] selection drag from letter %d", (int)s_lift_sel_anchor);
        }
        uint32_t idx;
        if (lift_input_letter_at(cur, &idx))
            lift_sel_apply(s_lift_sel_anchor, (int32_t)idx);
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        lift_longpress_arm_stop();
        lift_input_voice_stop();
        if (s_lift_dragging)
        {
            s_lift_dragging = false;
            s_lift_sel_anchor = -1; /* 選取範圍留著等刪除鍵,只收掉拖動狀態 */
            if (lift_sel_active())
                LOG_I("[lift_input] selection [%d,%d)", (int)s_lift_sel_from, (int)s_lift_sel_to);
            else
                lift_sel_clear(); /* 拖回原點 = 空選取,別留一個看不見的狀態 */
        }
        else
        {
            s_lift_sel_anchor = -1;
        }
        break;
    default:
        break;
    }
}

/* 「沒東西可送」的觸覺回饋。靜默忽略跟「這顆按鈕是死的」在使用者眼裡完全一樣 —— 真機上
   founder 因此連按了 8 次 logo。震一下代表「收到了,但目前沒有文字」。 */
static void lift_input_reject_feedback(void)
{
    extern void motor_pattern_unlocked(void);
    motor_pattern_unlocked();
}

/* 剛講完話、轉錄還在路上的寬限期。真機 2026-08-01:放開麥克風到轉錄抵達實測隔了 33 秒
   (STT 往返 + AI 整理),使用者早就按下送出了,卻被「現在框裡沒字」擋掉,得再按第二次。
   這個窗口內放行,由手機端扣住該次送出、等文字到齊再執行(它本來就有這套 hold 機制)。 */
#define LIFT_PENDING_TEXT_GRACE_MS 45000

static bool lift_input_voice_just_ended(void)
{
    if (s_lift_voice_active)
        return true; /* 還在錄音,文字必然還沒定稿 */
    if (s_lift_voice_stopped_at == 0)
        return false;
    return (rt_tick_get() - s_lift_voice_stopped_at) <
           rt_tick_from_millisecond(LIFT_PENDING_TEXT_GRACE_MS);
}

static bool lift_input_has_text(void)
{
    if (!s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
        return false;
    const char *t = lv_label_get_text(s_lift_input_label);
    if (t == NULL || t[0] == '\0')
        return false;
    return strcmp(t, LV_EXT_STR_GET_BY_KEY(listening, "Listening")) != 0;
}

/* fwd — all defined further down this file. */
static void feed_single_device_options(const char *device_id);
void instruction_list_bar_device_dismiss(void);
void instruction_list_open_browse(void);
void instruction_list_close_lift_input_view(void);
bool instruction_list_lift_input_view_open(void);
static void lift_input_hide_view(void);
static void lift_del_btn_event_cb(lv_event_t *evt);
static void lift_del_update_icon(void); /* 有字=退格圖 / 沒字=退出圖 */

/* icon_send:把暫存文字打進「剛剛點的那個輸入框」。手機收到 0x1d dest=field 後送
   airMouse sendToFocusedInput(電腦打字 + 收掉它的面板),手錶這邊直接收乾淨整個 session
   —— founder:「發給輸入框後手錶就可以離開這模式」。 */
static void lift_send_btn_event_cb(lv_event_t *evt)
{
    if (lv_event_get_code(evt) != LV_EVENT_CLICKED)
        return;
    if (!lift_input_has_text() && !lift_input_voice_just_ended())
    {
        LOG_I("[lift_input] send tapped with no text — ignored");
        lift_input_reject_feedback();
        return;
    }
    extern bool commu_send_lift_input_commit(const char *dest);
    commu_send_lift_input_commit("field");
    instruction_list_close_lift_input_view();
}

/* img_logo:把暫存文字當成 skaibar 查詢送出。電腦端 runSkaibar(submit=false) 會離開
   input-only 讓選項展開,配套的 skaibarQuery 讓那台把重算後的選項 push 回來 → 手錶這邊
   叫出同一份清單(先用 registry placeholder 墊著,push 一到就換成電腦的即時選項)。
   刻意清掉 s_opened_by_lift:清單開出來之後手腕放下不該把它一起收掉(那個 close 路徑
   只服務「舉起帶出的面板」)。 */
static void lift_logo_btn_event_cb(lv_event_t *evt)
{
    if (lv_event_get_code(evt) != LV_EVENT_CLICKED)
        return;
    if (!lift_input_has_text() && !lift_input_voice_just_ended())
    {
        LOG_I("[lift_input] logo tapped with no text — ignored");
        lift_input_reject_feedback();
        return;
    }
    extern bool commu_send_lift_input_commit(const char *dest);
    commu_send_lift_input_commit("skaibar");
    lift_input_voice_stop();
    lift_input_hide_view();
    s_opened_by_lift = false; /* 之後的 wrist-drop 不再收掉這個清單 */
    extern void instruction_list_bar_set_blur(bool on);
    instruction_list_bar_set_blur(false); /* 非錶盤、不模糊 */
    instruction_list_open_browse();
}

static void ensure_lift_input_view(void)
{
    if (s_lift_input_view && lv_obj_is_valid(s_lift_input_view))
        return;
    if (!s_global_bar_layer || !lv_obj_is_valid(s_global_bar_layer))
        return;

    s_lift_input_view = lv_obj_create(s_global_bar_layer);
    lv_obj_remove_style_all(s_lift_input_view);
    lv_obj_set_size(s_lift_input_view, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_lift_input_view, 0, 0);
    lv_obj_clear_flag(s_lift_input_view, LV_OBJ_FLAG_SCROLLABLE);
    /* CLICKABLE 但不掛語音 cb:純粹吃掉落在面板空白處的觸控,不讓它穿到底下的觸控板
       (滑鼠 app 的整頁 trackpad)去亂動游標。 */
    lv_obj_add_flag(s_lift_input_view, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(s_lift_input_view, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_lift_input_view, LV_OPA_40, 0);
    lv_obj_add_flag(s_lift_input_view, LV_OBJ_FLAG_HIDDEN);
    /* 放開的兜底(見 lift_input_view_event_cb):手指在哪裡離開都要結束錄音。 */
    lv_obj_add_event_cb(s_lift_input_view, lift_input_view_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_lift_input_view, lift_input_view_event_cb, LV_EVENT_PRESS_LOST, NULL);

    /* 輸入框 —— 與 skaibar 的 ai_box 同一張 frame 圖 + 同一套兩行轉錄窗,只是置中而非貼底,
       讓上下各留出圖示列與麥克風的位置。 */
    s_lift_input_box = lv_obj_create(s_lift_input_view);
    lv_obj_remove_style_all(s_lift_input_box);
    lv_obj_set_size(s_lift_input_box, LIFT_BOX_W, LIFT_BOX_H);
    lv_obj_align(s_lift_input_box, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(s_lift_input_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_lift_input_box, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *frame = lv_img_create(s_lift_input_box);
    lv_img_set_src(frame, &message_widget_bg);
    lv_obj_center(frame);
    lv_obj_clear_flag(frame, LV_OBJ_FLAG_CLICKABLE);

    const lv_font_t *vt_font = LV_EXT_FONT_GET(get_system_font_size(0));
    s_lift_input_clip = lv_obj_create(s_lift_input_box);
    lv_obj_remove_style_all(s_lift_input_clip);
    lv_obj_set_scrollbar_mode(s_lift_input_clip, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_lift_input_clip, LV_OBJ_FLAG_SCROLL_CHAIN);
    /* 使用者捲動關掉(程式仍可 lv_obj_scroll_* 貼齊最新兩行)。留著會出事:長按開始錄音後手指
       只要微飄,LVGL 就把這次按壓判成捲動、送 PRESS_LOST 停掉錄音,手指還在上面又接著一次
       按下 —— 正好組成「STOP 後立刻 START」,而那個序列會 hard fault 在 voice_re 執行緒
       (2026-07-31 已釘死的既有語音管線重入脆弱點)。 */
    lv_obj_clear_flag(s_lift_input_clip, LV_OBJ_FLAG_SCROLLABLE);
    /* 輸入框上的插入手勢(founder 2026-08-01):點一下移游標、長按移游標並直接開始錄音。 */
    lv_obj_add_flag(s_lift_input_clip, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_lift_input_clip, lift_input_clip_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_lift_input_clip, lift_input_clip_event_cb, LV_EVENT_SHORT_CLICKED, NULL);
    lv_obj_add_event_cb(s_lift_input_clip, lift_input_clip_event_cb, LV_EVENT_LONG_PRESSED, NULL);
    /* 長按後拖動 = 框選。PRESSING 只在觸點移動時送,拿來當「手指動了」的訊號剛好。 */
    lv_obj_add_event_cb(s_lift_input_clip, lift_input_clip_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(s_lift_input_clip, lift_input_clip_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_lift_input_clip, lift_input_clip_event_cb, LV_EVENT_PRESS_LOST, NULL);

    s_lift_input_label = lv_label_create(s_lift_input_clip);
    lv_label_set_text(s_lift_input_label, "");
    lv_obj_set_width(s_lift_input_label, 360);
    lv_label_set_long_mode(s_lift_input_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(s_lift_input_label, lv_color_white(), 0);
    lv_obj_set_style_text_opa(s_lift_input_label, LV_OPA_80, 0);
    lv_obj_set_style_text_font(s_lift_input_label, vt_font, 0); /* CJK 字型,否則中文變豆腐 */
    lv_obj_set_style_text_align(s_lift_input_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_lift_input_label, LV_ALIGN_TOP_MID, 0, 0);
    /* 框選的反白(LV_PART_SELECTED,由 lv_label_set_text_sel_start/end 觸發)。半透明的游標
       同色,讓「插入點」與「選取範圍」看起來是同一套語彙;文字維持白色不被吃掉。 */
    lv_obj_set_style_bg_color(s_lift_input_label, lv_color_hex(LMIC_RIPPLE_COLOR), LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(s_lift_input_label, LV_OPA_30, LV_PART_SELECTED);
    lv_obj_set_style_text_color(s_lift_input_label, lv_color_white(), LV_PART_SELECTED);
    {
        /* 顯示窗高度 = 這個框裝得下的行數,不是底部 skaibar 那種兩行字幕窗。
           founder 2026-08-01 實測回報:電腦顯示「12345語音辨識12345一次測試676789101112」,
           手錶只看得到「12345一次測試676789101112」—— 前面被捲出視野。資料其實是完整的
           (log 證實手錶收到整串、送出也完整),純粹是我沿用了字幕用的 2 行窗:那是為「框大半在
           畫面外、只需看最新兩行」設計的,而立起面板的框 442x252 整個置中、完全看得到,使用者
           正在**編輯**文字,看不到前文就沒法判斷要插在哪。
           超出仍會貼齊最新內容(捲到底),只是門檻高很多。 */
        lv_coord_t lh = lv_font_get_line_height(vt_font);
        lv_coord_t ls = lv_obj_get_style_text_line_space(s_lift_input_label, LV_PART_MAIN);
        lv_coord_t avail = LIFT_BOX_H - LIFT_BOX_TEXT_INSET_V * 2;
        uint8_t rows = (uint8_t)((avail + ls) / (lh + ls));
        if (rows < 2) rows = 2;
        if (rows > LIFT_BOX_MAX_ROWS) rows = LIFT_BOX_MAX_ROWS;
        lv_obj_set_size(s_lift_input_clip, 360, rows * lh + (rows - 1) * ls);
        lv_obj_align(s_lift_input_clip, LV_ALIGN_CENTER, 0, 0); /* 框內置中(框已置中) */
        LOG_I("[lift_input] text window rows=%u (lh=%d ls=%d avail=%d)",
              (unsigned)rows, (int)lh, (int)ls, (int)avail);
    }

    /* 插入游標 —— 長按定位後畫在該字元前緣的細直線。與 label 同 parent(clip),所以會跟著
       兩行窗一起裁切/捲動。非 clickable,不擋長按。 */
    s_lift_caret = lv_obj_create(s_lift_input_clip);
    lv_obj_remove_style_all(s_lift_caret);
    lv_obj_set_style_bg_color(s_lift_caret, lv_color_hex(LMIC_RIPPLE_COLOR), 0);
    lv_obj_set_style_bg_opa(s_lift_caret, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_lift_caret, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_lift_caret, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_lift_caret, LV_OBJ_FLAG_HIDDEN);

    /* 游標下的圓球把手。建在 caret 之後 = 在它上面,而且**要 clickable**(游標本身不是) */
    s_lift_caret_ball = lv_obj_create(s_lift_input_clip);
    lv_obj_remove_style_all(s_lift_caret_ball);
    lv_obj_set_size(s_lift_caret_ball, LIFT_CARET_BALL_D, LIFT_CARET_BALL_D);
    lv_obj_set_style_bg_color(s_lift_caret_ball, lv_color_hex(LMIC_RIPPLE_COLOR), 0);
    lv_obj_set_style_bg_opa(s_lift_caret_ball, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_lift_caret_ball, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_flag(s_lift_caret_ball, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_lift_caret_ball, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_ext_click_area(s_lift_caret_ball, 14); /* 22px 球 → 50px 觸控標的 */
    lv_obj_add_event_cb(s_lift_caret_ball, lift_caret_ball_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_lift_caret_ball, lift_caret_ball_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(s_lift_caret_ball, lift_caret_ball_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_lift_caret_ball, lift_caret_ball_event_cb, LV_EVENT_PRESS_LOST, NULL);
    lv_obj_add_flag(s_lift_caret_ball, LV_OBJ_FLAG_HIDDEN);

    /* 上方圖示列 —— logo(送 AI)恆在、send(送回輸入框)依電腦聚焦狀態顯示。
       兩顆都在時 logo 在左、send 在右(iOS 慣例:主要動作在右);只有 logo 時置中。 */
    s_lift_logo_btn = lv_img_create(s_lift_input_view);
    lv_img_set_src(s_lift_logo_btn, &img_logo);
    lv_obj_add_flag(s_lift_logo_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_lift_logo_btn, lift_logo_btn_event_cb, LV_EVENT_CLICKED, NULL);

    s_lift_send_btn = lv_img_create(s_lift_input_view);
    lv_img_set_src(s_lift_send_btn, &icon_send);
    lv_obj_add_flag(s_lift_send_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_lift_send_btn, LIFT_SEND_EXT_CLICK); /* 34px 圖 → 64px 觸控標的 */
    lv_obj_add_event_cb(s_lift_send_btn, lift_send_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_lift_send_btn, LV_OBJ_FLAG_HIDDEN);

    /* 下方小麥克風 —— 64px 原生(zoom 不動),與底部 bar 的 mic glyph 同尺寸;舊版的 128px
       置中大圖已由 founder 2026-07-31 取消。 */
    s_lift_mic_btn = lv_img_create(s_lift_input_view);
    lv_img_set_src(s_lift_mic_btn, &micro_icon);
    lv_img_set_pivot(s_lift_mic_btn, micro_icon.header.w / 2, micro_icon.header.h / 2);
    lv_obj_add_flag(s_lift_mic_btn, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_align(s_lift_mic_btn, LV_ALIGN_CENTER, LIFT_MIC_DX, LIFT_MIC_DY);
    lv_obj_add_flag(s_lift_mic_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_lift_mic_btn, lift_mic_btn_event_cb, LV_EVENT_ALL, NULL);

    /* 刪除鍵 —— 與麥克風並排。點一下刪一個字、長按連續刪(見 lift_del_btn_event_cb)。 */
    s_lift_del_btn = lv_img_create(s_lift_input_view);
    lv_img_set_src(s_lift_del_btn, &backspace_icon);
    lv_obj_align(s_lift_del_btn, LV_ALIGN_CENTER, LIFT_DEL_DX, LIFT_MIC_DY);
    lv_obj_add_flag(s_lift_del_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(s_lift_del_btn, LIFT_DEL_EXT_CLICK);
    lv_obj_add_event_cb(s_lift_del_btn, lift_del_btn_event_cb, LV_EVENT_SHORT_CLICKED, NULL);
    lv_obj_add_event_cb(s_lift_del_btn, lift_del_btn_event_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(s_lift_del_btn, lift_del_btn_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_lift_del_btn, lift_del_btn_event_cb, LV_EVENT_PRESS_LOST, NULL);

    /* 這個函式只會在 LVGL 執行緒跑(開面板路徑),記下身分供 set_text 分辨空字串來源。 */
    s_lift_lvgl_thread = rt_thread_self();

    /* 錄音脈衝圈:與輸入框同尺寸的圓角矩形,從框緣往外擴散漸淡。建在最後但要移到底層,
       否則會蓋住框本身與圖示。 */
    s_lift_box_glow = lv_obj_create(s_lift_input_view);
    lv_obj_remove_style_all(s_lift_box_glow);
    lv_obj_set_style_border_color(s_lift_box_glow, lv_color_hex(LMIC_RIPPLE_COLOR), 0);
    lv_obj_set_style_border_width(s_lift_box_glow, 3, 0);
    lv_obj_set_style_radius(s_lift_box_glow, LIFT_BOX_GLOW_RADIUS, 0);
    lv_obj_set_style_bg_opa(s_lift_box_glow, LV_OPA_TRANSP, 0);
    lv_obj_set_size(s_lift_box_glow, LIFT_BOX_W, LIFT_BOX_H);
    lv_obj_align(s_lift_box_glow, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(s_lift_box_glow, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_clear_flag(s_lift_box_glow, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_lift_box_glow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_lift_box_glow, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_background(s_lift_box_glow);
}

/* 依「電腦目前有沒有聚焦輸入框」擺放上方圖示:有 → logo 左 / send 右;沒有 → 只有 logo,
   置中。面板開著時焦點變化也會走這裡(instruction_list_set_remote_target_focus)。 */
static void lift_input_layout_icons(void)
{
    if (!s_lift_logo_btn || !lv_obj_is_valid(s_lift_logo_btn))
        return;
    bool show_send = s_remote_target_has_focus;
    if (show_send)
    {
        lv_obj_align(s_lift_logo_btn, LV_ALIGN_CENTER, LIFT_LOGO_DX, LIFT_ICON_ROW_DY);
        if (s_lift_send_btn && lv_obj_is_valid(s_lift_send_btn))
        {
            lv_obj_align(s_lift_send_btn, LV_ALIGN_CENTER, LIFT_SEND_DX, LIFT_ICON_ROW_DY);
            lv_obj_clear_flag(s_lift_send_btn, LV_OBJ_FLAG_HIDDEN);
        }
    }
    else
    {
        lv_obj_align(s_lift_logo_btn, LV_ALIGN_CENTER, 0, LIFT_ICON_ROW_DY);
        if (s_lift_send_btn && lv_obj_is_valid(s_lift_send_btn))
            lv_obj_add_flag(s_lift_send_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 把輸入框設回「Listening」佔位。**只能在 LVGL 執行緒呼叫**(開面板路徑) —— 見
   instruction_list_lift_input_set_text 裡關於 voice_re 堆疊的說明。 */
static void lift_input_reset_placeholder(void)
{
    if (!s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
        return;
    lv_label_set_text(s_lift_input_label, LV_EXT_STR_GET_BY_KEY(listening, "Listening"));
}

/* 公開:手機下行的轉錄(refresh_ai_chat_input_message 的 lift 分支)寫進面板。 */
void instruction_list_lift_input_set_text(const char *text)
{
    /* TEMP DIAG(2026-07-31):確認轉錄真的走到面板。定位完拿掉。 */
    LOG_I("[lift_input][diag] set_text(\"%s\") label=%p", text ? text : "(null)",
          (void *)s_lift_input_label);
    if (!s_lift_input_label || !lv_obj_is_valid(s_lift_input_label))
        return;
    /* ⚠ 空字串這條路是 bloc_v2t 在語音起始時**直接呼叫**進來的,跑在 voice_re 執行緒上
       (不走 LVGL 訊息佇列)。那條執行緒的堆疊是照音訊工作量給的,**吃不下任何 LVGL 遞迴** ——
       連 lv_label_set_text 都不行:WRAP 標籤設字會走文字量測(FreeType)+失效重算。真機
       2026-08-01 打爆兩次,第二次 psr/lr/pc 全是 0x23232323(RT-Thread 堆疊填充值)。
       第一次我只把這條路徑「變便宜」(拿掉 update_layout/捲動)——不夠,照樣爆。
       所以現在**整個忽略空字串**:一行 LVGL 都不碰。
       這對功能反而是對的 —— 佔位文字由開面板時(LVGL 執行緒)的 lift_input_reset_placeholder()
       負責;而在連續補話的情境下,開始講下一段時把框清成「Listening」本來就會把使用者正在
       編輯的內容藏起來,不該做。 */
    bool has = (text != NULL && text[0] != '\0' && strspn(text, " \t\n\r") < strlen(text));
    if (!has)
    {
        /* …**但**「使用者把字一個個刪光」也是走這裡送空字串回來的,那一條來自 BLE 下行、跑在
           LVGL 執行緒上,完全可以碰 UI。不分辨的話畫面會停在最後那個字不放(而且刪除鍵永遠
           不會變成退出鍵)。用建立物件時記下的執行緒身分比對,不猜名字。 */
        if (rt_thread_self() != s_lift_lvgl_thread)
            return;
        lift_input_reset_placeholder();
        lift_sel_clear();
        lift_input_place_caret(-1);
        lift_del_update_icon();
        return;
    }
    uint32_t before = lift_input_char_count();
    lv_label_set_text(s_lift_input_label, text);
    lift_del_update_icon(); /* 從空變有字 → 退出圖換回退格圖 */
    /* 文字換過了,舊的選取索引指向的已經不是同一段字 → 收掉。**但手指正在拖的時候不能收** ——
       轉錄會斷斷續續補送(真機 2026-08-01:同一串文字連推三次),那會在使用者拖到一半把選取
       連同 anchor 一起抹掉。拖動中的選取由下一個 PRESSING 依 anchor 重算,本來就會自己更新。 */
    if (!s_lift_dragging)
        lift_sel_clear();
    /* 游標要**留著跟著走**,不能一換字就收掉 —— 它是這個面板唯一的位置指示,收掉之後使用者
       就不知道下一次刪除/補話會發生在哪(founder 2026-08-01:「按過一次刪除鍵後游標就消失了,
       我不知道下個會刪哪裡」)。手錶不知道手機那邊 prefix/段/suffix 怎麼切,但**字數差就夠了**:
       改動一律發生在插入點上,所以新位置 = 舊位置 + (新字數 - 舊字數)。
         插入一段 5 個字 → 游標落在插入內容的尾巴;
         刪掉一個字     → 游標往前退一格;
         逐字稿逐漸長長 → 游標跟著長。
       停在尾端(-1)的情況不用算,render 每次都取當下字數。 */
    if (s_lift_caret_pos >= 0)
    {
        int32_t moved = s_lift_caret_pos + ((int32_t)lift_input_char_count() - (int32_t)before);
        s_lift_caret_pos = (moved > 0) ? moved : 0;
    }
    lift_caret_render();
    /* 兩行窗釘在最新內容(同 voice_transcript_scroll_to_bottom 的理由)。 */
    if (s_lift_input_clip && lv_obj_is_valid(s_lift_input_clip))
    {
        lv_obj_update_layout(s_lift_input_clip);
        lv_coord_t bottom = lv_obj_get_scroll_bottom(s_lift_input_clip);
        if (bottom > 0)
            lv_obj_scroll_by(s_lift_input_clip, 0, -bottom, LV_ANIM_OFF);
        else
            lv_obj_scroll_to_y(s_lift_input_clip, 0, LV_ANIM_OFF);
    }
}

/* TEMP DIAG(2026-08-01 founder 要求):顯示「還沒經過 AI 修繕」的原始逐字稿。LVGL 執行緒
   (0x1f 下行走既有的訊息佇列)。定位完連同 KEY_LIFT_INPUT_RAW 一起移除。 */
/* 刪除鍵(founder 2026-08-01:「跟麥克風並排,不是清空,要一個個刪除;按一次刪一個字,長按才
   開始連續刪除」)。實際刪除在手機端做 —— 暫存文字的單一真相在那裡,而且刪一個「字」要按
   code point 算(中文一個字是 3 bytes),手錶只負責發指令。 */
static lv_timer_t *s_lift_del_repeat = NULL;

/* 沒字 = 這顆鍵當退出鍵。圖示跟著換,不然使用者按下去會以為是刪除卻整個離開。 */
static void lift_del_update_icon(void)
{
    if (!s_lift_del_btn || !lv_obj_is_valid(s_lift_del_btn))
        return;
    lv_img_set_src(s_lift_del_btn, lift_input_has_text() ? (const void *)&backspace_icon
                                                         : (const void *)&down_arrow);
}

static void lift_del_send_one(void)
{
    extern bool commu_send_lift_input_delete(void);
    extern bool commu_send_lift_input_delete_range(int from, int to);
    /* 有框選就整段刪(founder 2026-08-01);刪完選取消失,長按連刪的後續 tick 自然退回一次一字。 */
    if (lift_sel_active())
    {
        LOG_I("[lift_input] delete selection [%d,%d)", (int)s_lift_sel_from, (int)s_lift_sel_to);
        commu_send_lift_input_delete_range((int)s_lift_sel_from, (int)s_lift_sel_to);
        /* 游標落在被刪範圍的起點 —— 手機端回推的新文字會用字數差把它帶到同一個位置。 */
        s_lift_caret_pos = s_lift_sel_from;
        lift_sel_clear();
        return;
    }
    commu_send_lift_input_delete();
}

static void lift_del_repeat_cb(lv_timer_t *t)
{
    (void)t;
    lift_del_send_one();
}

static void lift_del_stop_repeat(void)
{
    if (s_lift_del_repeat)
    {
        lv_timer_del(s_lift_del_repeat);
        s_lift_del_repeat = NULL;
    }
}

static void lift_del_btn_event_cb(lv_event_t *evt)
{
    /* 框裡沒字 = 這顆鍵是退出鍵(founder 2026-08-01)。放在最前面攔截,連長按連刪也不會啟動 ——
       空面板按住不放不該「連續退出」。 */
    if (!lift_input_has_text())
    {
        if (lv_event_get_code(evt) == LV_EVENT_SHORT_CLICKED)
        {
            LOG_I("[lift_input] delete key acted as EXIT (box empty)");
            instruction_list_close_lift_input_view();
        }
        return;
    }
    switch (lv_event_get_code(evt))
    {
    case LV_EVENT_SHORT_CLICKED:
        lift_del_send_one(); /* 點一下 = 刪一個字 */
        break;
    case LV_EVENT_LONG_PRESSED:
        /* 長按 = 連續刪。自己開 timer 控速,不靠 LVGL 的 LONG_PRESSED_REPEAT ——
           那個間隔由全域設定決定,對「按住刪字」來說太慢。 */
        lift_del_stop_repeat();
        lift_del_send_one(); /* 立刻先刪一個,不要等第一次 tick */
        s_lift_del_repeat = lv_timer_create(lift_del_repeat_cb, LIFT_DEL_REPEAT_MS, NULL);
        break;
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        lift_del_stop_repeat();
        break;
    default:
        break;
    }
}

static void lift_input_hide_view(void)
{
    lift_del_stop_repeat();    /* 面板收掉時別留著連續刪除的 timer */
    lift_caret_blink_stop();   /* 同理:看不見的游標不用繼續閃 */
    lift_longpress_arm_stop(); /* 收面板當下正好在長按確認窗內 → 別在事後才開錄音 */
    lift_sel_clear();
    s_lift_dragging = false;
    s_lift_view_shown = false;
    if (s_lift_input_view && lv_obj_is_valid(s_lift_input_view))
        lv_obj_add_flag(s_lift_input_view, LV_OBJ_FLAG_HIDDEN);
    if (p_instruction_list_layout && p_instruction_list_layout->mic_bar &&
        lv_obj_is_valid(p_instruction_list_layout->mic_bar))
        lv_obj_clear_flag(p_instruction_list_layout->mic_bar, LV_OBJ_FLAG_HIDDEN);
    mic_hit_follow_bar(); /* bar 回來了 → 它的大 tap 區也要回來 */
}

/* 公開：舉起手勢(motion thread→hid_mouse→這裡)呼叫。2026-07-31 起不再看電腦有沒有聚焦
   輸入框 —— 一律開面板；聚焦狀態只決定 icon_send 出不出現。0x0E 帶 forceOpen=true +
   inputOnly=true：電腦一定把 skaibar 叫出來，但只留輸入框(不出選項)，並把召喚前聚焦的
   那個輸入框「記住」當 icon_send 的目的地(而不是邊講邊打進去)。 */
bool instruction_list_open_lift_input_view(const char *device_id)
{
    if (device_id == NULL || device_id[0] == '\0')
        return false; /* 沒有控制目標 → 沒有單一電腦可開 */
    /* 已經開著就什麼都不做。2026-07-31「放下不要直接退出」之後這條變成必要防護:面板會活過
       手腕放下,使用者很自然會「立起→講話→放下看畫面→再立起」,而每次立起 pose 都會再打一次
       這裡。沒有這個 early-return,下面的 instruction_list_lift_input_set_text("") 會把剛講完
       的暫存文字洗回 Listening 佔位,等於默默吃掉使用者的輸入。 */
    if (instruction_list_lift_input_view_open())
        return true;
    if (!p_instruction_list_layout)
    {
        extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t * parent);
        lv_instruction_list_layout_create(lv_scr_act());
        if (!p_instruction_list_layout)
            return false;
    }
    s_bar_single_device = true;
    extern bool commu_send_skaibar_open_device_ex(bool force_open, bool input_only);
    commu_send_skaibar_open_device_ex(true, true);
    /* 先把 registry 的 default_actions 餵進共享清單當 placeholder(清單此刻是隱藏的,看不到)。
       這一步同時把 device_id 記進 s_single_device_id —— logo 送出後要開清單時用得到。 */
    feed_single_device_options(device_id);
    instruction_list_bar_set_visible(true);
    /* 共享層上的 mic_bar 會跟著露出來、和面板疊在一起 —— 這條流程用面板自己的小麥克風,
       把它藏掉(同舊大麥克風流程的做法;instruction_list_floating_bar_visible() 也已把
       「只有立起面板開著」回報成 false,滑鼠 app 自有的 bar 才不會被 overlap-sync 收掉)。 */
    if (p_instruction_list_layout->mic_bar && lv_obj_is_valid(p_instruction_list_layout->mic_bar))
        lv_obj_add_flag(p_instruction_list_layout->mic_bar, LV_OBJ_FLAG_HIDDEN);
    /* 關鍵:mic_hit 一定要跟著藏 —— 否則它會在滑鼠自有 bar 上面「看不見地」吃掉每一次
       tap,症狀就是「怎麼點 skaibar_img 都沒反應」(founder 2026-08-17)。 */
    mic_hit_follow_bar();
    ensure_lift_input_view();
    if (s_lift_input_view && lv_obj_is_valid(s_lift_input_view))
    {
        lift_input_reset_placeholder();    /* 新 session 從 Listening 佔位開始(LVGL 執行緒) */
        lift_sel_clear();                  /* 上一輪的框選不要跟著新 session 進來 */
        lift_input_place_caret(-1);        /* 順序要在佔位之後:render 讀當下文字才收得掉游標 */
        lift_del_update_icon();            /* 空面板開場 = 退出鍵 */
        lift_input_layout_icons();
        lv_obj_clear_flag(s_lift_input_view, LV_OBJ_FLAG_HIDDEN);
        s_lift_view_shown = true; /* 供其他執行緒唯讀查詢,不必走 LVGL 物件樹 */
        lv_obj_move_foreground(s_lift_input_view);
        /* 面板已就緒的觸覺回饋 —— 立起手勢沒有視線外的確認手段,短震讓使用者不用盯著錶
           就知道可以按麥克風講話(同 AI 姿態觸發用的 25ms pattern)。 */
        extern void motor_pattern_unlocked(void);
        motor_pattern_unlocked();
    }
    instruction_list_mark_opened_by_lift();
    return true;
}

/* 公開：立起輸入面板目前是否開著。bloc_motion_tracking.c 的 air_mouse_process 用這個
   gate 在面板開著時跳過 report_air_mouse_data —— 純防禦：輸入期間手腕動作不該兼職當
   游標(若 handfree/頂部飛鼠恰好是開的,舉腕講話會讓游標亂飄)。手寫模式的兩處 gate
   同理。 */
bool instruction_list_lift_input_view_open(void)
{
    /* **純布林值,不碰 LVGL**。原本寫成 lv_obj_is_valid() + lv_obj_has_flag(),但
       lv_obj_is_valid() 會遞迴走遍整棵物件樹,而這個查詢被 air_mouse_process 從**周邊執行緒
       每一幀**呼叫(還有轉錄路由從通訊執行緒呼叫)—— LVGL 執行緒在建/拆面板物件的同時被別條
       執行緒走樹,就會炸。真機 2026-08-01 當場抓到:
         pc=0x10123f40 → obj_valid_child+23 (lv_obj.o), hard fault on thread: peripher
       旗標只由 LVGL 執行緒在顯示/隱藏的當下寫,其他執行緒唯讀,單一 bool 無需鎖。 */
    return s_lift_view_shown;
}

/* 公開：舉起姿態結束(手腕放下)或 icon_send 送出後呼叫。只有這個 session 是舉起帶出的
   才動作(s_opened_by_lift)，手動點 bar 開的清單/輸入框不受影響——同一個 wrist-drop
   事件，如果使用者中途已經手動點開別的東西，這裡不該連帶關掉它。logo 送出後也會清掉
   s_opened_by_lift，讓隨後的 wrist-drop 不去收剛叫出來的清單。 */
void instruction_list_close_lift_input_view(void)
{
    if (!s_opened_by_lift)
        return;
    lift_input_voice_stop();
    lift_input_hide_view();
    /* instruction_list_bar_device_dismiss() resets s_remote_target_has_focus = false as a
       side effect meant for the OLD tap-triggered flow. That flag must track the DESKTOP's
       live focus state (set by phone-relayed KEY_REMOTE_TEXT_FOCUS BLE frames whenever it
       actually changes there) — it must NOT get wiped just because we closed our local view.
       Real-hw found: without restoring it here, lifting the wrist a second time while the
       desktop field is STILL genuinely focused silently changed what the panel offered
       (icon_send missing) until the next spontaneous focus-change broadcast happened to flip
       it back. */
    bool focus_before_dismiss = s_remote_target_has_focus;
    instruction_list_bar_device_dismiss();
    s_remote_target_has_focus = focus_before_dismiss;
}

/* Set by ai_widget_fade_on_scroll (and the session-page bar tap): THIS close
   should collapse the voice box back to the slim bar but KEEP the floating list
   up (browse / switch options). Consumed at the top of close_ai_widget; every
   other close path slides the whole list out. */
static bool s_scroll_keep_list = false;

static void mic_bar_event_cb(lv_event_t *evt)
{
    if (evt->code != LV_EVENT_CLICKED) return;
    /* 長按語音剛結束 → 這顆 CLICKED 是同一次按壓的放開,吃掉(mic_hit 轉呼叫
       也一併被擋,它 forward 進來)。 */
    if (s_mic_lp_consumed)
    {
        s_mic_lp_consumed = false;
        return;
    }
    /* 這顆 tap 的去向只有兩種,兩邊都要看得見 —— 「按了沒反應」查起來最花時間的就是
       分不出「事件沒進來」和「進來了走錯分支」(founder 2026-08-13)。 */
    LOG_W("[mic] tap session_page=%d", (int)s_session_page_mode);
    /* ADR-0020 R8:左頁 session 檢視中,麥克風 = 直接開新 session(founder R6:
       「點麥克風也是直接去開新的 session」)。清單推回來後 session pager 自動
       走進聊天室。 */
    /* R55(founder 2026-08-13 改版):麥克風**先開輸入框**,說的字即時篩上面的清單;
       篩不到才出現「開新 session」那一項,由它去建。所以這裡不再直接建 session ——
       直接建的話,想找既有的 session / action 就沒有入口了。 */
    /* Toggle by the box's ACTUAL visibility, not the is_open flag — the flag can
       get stuck true (e.g. navigating away while open without a close), which made
       a tap take the close branch and silently do nothing instead of opening. The
       box (p_instruction_list_ai_bg) is HIDDEN when closed. animate_open_ai_widget
       / close_ai_widget self-guard re-entry. */
    bool box_visible =
        p_instruction_list_layout &&
        p_instruction_list_layout->p_instruction_list_ai_bg &&
        !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                         LV_OBJ_FLAG_HIDDEN);
    /* R3 #5: the floating list (p_instruction_list_bg) stays up after an A2 scroll
       collapses the voice box (browse state). A SHOWN list captures touches across
       the WHOLE screen (translate_x is draw-only — even a slid-out-but-not-HIDDEN
       list still hit-tests at its on-screen rect), so on the device/mouse page it
       covers the trackpad and the mouse is untouchable until the list is HIDDEN.
       Treat a bar tap while the drawer is up (open box OR browse-state) as "dismiss
       the whole drawer": the always-visible bottom bar then reliably returns the
       user to the page underneath (the mouse). Previously a browse-state tap merely
       re-opened the box, leaving the list covering the trackpad. */
    bool list_shown =
        p_instruction_list_layout &&
        p_instruction_list_layout->p_instruction_list_bg &&
        !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_bg,
                         LV_OBJ_FLAG_HIDDEN);
    extern bool clock_main_page_is_home(void);
    /* On the watch face, a tap in the browse state (list shown, box collapsed)
       should OPEN the input box in place — NOT dismiss the drawer. The dismiss-the-
       whole-drawer behaviour is only needed on the device/mouse page, where a shown
       list covers the trackpad. So: an open box always toggles closed; a browse-
       state list dismisses ONLY off the watch face (on the face it opens the box).
       R74(founder 2026-08-13「點麥克風他會回到主畫面」):左頁 SESSION 檢視也不在
       錶盤(tileview 真的停在左格 idx≠home),browse 態 tap 因此走進 dismiss →
       close_ai_widget 收掉清單 → R24 守門看到「清單沒了但人停在左格」snap home ——
       麥克風一點就回主畫面。session 頁沒有觸控板要讓路,dismiss 規則不適用:
       這裡 tap = 開語音輸入框(R55),跟錶盤同待遇。 */
    if (box_visible && (s_session_page_mode || s_bar_single_device))
    {
        /* R76(founder 2026-08-15「點輸入框他會跑回主畫面」):session 檢視裡輸入框
           開著時,落在 bar/框下緣的 tap 走了整個 dismiss → 清單沒了 → R24 snap
           home。這裡的正確語意=收框留清單(跟捲動收框同一條路):語音停止、篩選
           結果留在畫面上讓使用者挑。滑鼠 app 的單設備搜尋抽屜(s_bar_single_device,
           2026-08-15)同語意:mic 再點=收框留選項,清單內容由電腦即時鏡像。 */
        LOG_W("[mic] tap w/ box open on session page -> collapse, keep list");
        s_scroll_keep_list = true;
        close_ai_widget();
    }
    else if (list_shown && !box_visible && s_bar_single_device)
    {
        /* 滑鼠單設備搜尋抽屜的 browse 態(清單開著、框收著):mic tap = 重開語音輸入框
           (與左頁 session 檢視同待遇)。不能走下面的 dismiss 分支 —— 那條「清單蓋住觸控板
           要讓路」規則是給非搜尋流程的;這裡使用者就是來搜的,dismiss 靠左滑退出。 */
        LOG_W("[mic] tap in single-device browse -> reopen voice box");
        extern void animate_open_ai_widget(void);
        animate_open_ai_widget();
    }
    else if (box_visible || (list_shown && !clock_main_page_is_home() && !s_session_page_mode))
    {
        LOG_I("Mic bar tapped — dismissing AI widget (box=%d list=%d)",
              (int)box_visible, (int)list_shown);
        s_close_is_cancel = true; /* bar-tap close = user cancel, not a commit */
        close_ai_widget();
    }
    else
    {
        LOG_I("Mic bar tapped — opening AI widget (is_open flag=%d)",
              (int)is_open_instruction_list_ai);
        /* The global bar floats above the watch face (HOME) and the transient
           LEFT tile. R3: on the watch face the list floats IN PLACE over a
           blurred dial — NO tile switch (the list now lives on
           s_global_bar_layer). Other pages keep the legacy tile-switch path until
           they are migrated. animate_open_ai_widget reveals the floating list +
           morphs the bar; the blur is turned on here only for the watch face. In
           the browse state the list is already up — animate_open just morphs the
           box open over it (its was_hidden check skips a re-slide). */
        extern void animate_open_ai_widget(void);
        extern void instruction_list_bar_set_blur(bool on);
        extern void instruction_list_open_browse(void);
        /* The list floats in place on EVERY page now — never switch tiles. The
           watch face (HOME) blurs the dial behind it; the mouse page (RIGHT) and
           the transient LEFT tile float it transparently, no blur. */
        if (clock_main_page_is_home())
            instruction_list_bar_set_blur(true);
        /* P2 S1 — two-stage bar on the watch face: the 1st tap (nothing shown
           yet) floats the full list in to the BROWSE state — mic affordance, NO
           input box; a 2nd tap (list already shown) morphs the box open. Off the
           watch face the bar still opens the box directly (its drawer-dismiss
           case is handled in the close branch above). */
        if (clock_main_page_is_home() && !list_shown)
            instruction_list_open_browse();
        else
            animate_open_ai_widget();
    }
}


/* 把 E7 registry 裡 device_id 那台的 default_actions 餵進共享浮層清單 —— 與 device_pager
   的 feed_active_device_options_to_list 同一套 API(save_base 快照錶盤清單→drop_pinned 去掉
   Settings pin→clear→逐筆 add_or_update→refresh)，差別只是直接用 device_id 反查 registry,
   不靠 device_pager 的 p->model(滑鼠 app 沒有 device_pager 模型)。讓滑鼠 app 的 bar 清單顯示
   「正在控制那一台」的選項,而非錶盤的混合清單。LVGL 執行緒呼叫(bar tap)。 */
/* P3:記住目前單設備 skaibar 控制的那台 id。bar tap 是非同步的(tap→送 0x0E→電腦回 0x03
   選項要 ~1s),所以 0x03 一定晚到、要靠它更新後重餵清單。standalone 滑鼠 app 把錶盤拆了、
   device_pager UI 沒了,device_pager_refresh 改呼叫 instruction_list_refeed_single_device。 */
static char s_single_device_id[SYNCED_DEVICE_ID_LEN] = {0};

static void feed_single_device_options(const char *device_id)
{
    /* These are defined later in this file (no header prototype); declare locally —
       same convention the device-page restore/save call sites use above. */
    extern void instruction_list_save_base(void);
    extern void instruction_list_drop_pinned_for_device(void);
    extern void clear_custom_instructions(void);
    extern void add_or_update_custom_instruction(const char *id, const char *title,
            const char *trigger_type, uint32_t interval_sec, bool enabled,
            uint32_t version, const char *open_app);
    extern void refresh_custom_instructions(void);
    if (device_id == NULL || device_id[0] == '\0')
        return;
    /* 記住這台(即使下面因 usable==0 早退):0x03 晚到時 refeed 用得到。 */
    strncpy(s_single_device_id, device_id, sizeof(s_single_device_id) - 1);
    s_single_device_id[sizeof(s_single_device_id) - 1] = '\0';
    const T_SYNCED_DEVICE *dev = NULL;
    uint8_t n = SkaiWatchSys.device_registry.count;
    if (n > MAX_SYNCED_DEVICES) n = MAX_SYNCED_DEVICES;
    for (uint8_t i = 0; i < n; i++)
    {
        if (strncmp(SkaiWatchSys.device_registry.devices[i].id, device_id,
                    SYNCED_DEVICE_ID_LEN) == 0)
        {
            dev = &SkaiWatchSys.device_registry.devices[i];
            break;
        }
    }
    if (dev == NULL)
    {
        LOG_I("[bar_dev] device id not in registry (count=%u) -> keep current list",
              (unsigned)SkaiWatchSys.device_registry.count);
        return;
    }
    uint8_t ic = dev->default_action_count;
    if (ic > MAX_DEFAULT_ACTIONS) ic = MAX_DEFAULT_ACTIONS;
    /* 先數可用選項:0 個就不 feed —— 空清單會讓 open_browse 的 reset_list_internal 捲到
       不存在的 child。保留現有(錶盤)清單(≥1 項、含 Settings pin)、不 save_base/drop_pinned,
       電腦端 skaibar 仍會 summon(settle 送 open_device);restore_base 因沒 save 而 no-op。 */
    uint8_t usable = 0;
    for (uint8_t i = 0; i < ic; i++)
        if (dev->default_actions[i][0] != '\0') usable++;
    LOG_I("[bar_dev] device found, usable options=%u", (unsigned)usable);
    if (usable == 0)
        return;
    instruction_list_save_base();              /* 快照錶盤清單(每段只快照一次,離開時還原) */
    instruction_list_drop_pinned_for_device(); /* 設備選項 0-based、無 Settings pin */
    static uint32_t s_dev_opt_ver = 0;
    s_dev_opt_ver++;
    clear_custom_instructions();
    /* 反轉加入:手錶清單 bottom-up 渲染(option 0 在最下),電腦 skaibar 是 top-down。倒著加 →
       option 0(最下)= registry 最後一筆 = 電腦最下那列;最上 = registry 第一筆 = 電腦最上那列
       → 顯示與電腦對齊(不倒反)。commit 的 focus 翻轉(N-1-opt_idx,本檔 ~1188)正是為此反轉設計、
       兩者一致 → 點選也對到正確的電腦列。registry(0x03 device_actions)本身是電腦 top-down 原序。 */
    for (int i = (int)ic - 1; i >= 0; i--)
    {
        if (dev->default_actions[i][0] == '\0') continue;
        add_or_update_custom_instruction(dev->default_actions[i],
                                         dev->default_actions[i], "once", 0, true,
                                         s_dev_opt_ver, NULL);
    }
    refresh_custom_instructions();
}

/* P3 公開:device-sync(0x03 等)更新 registry 後重餵 standalone 滑鼠 skaibar 清單。給
   device_pager_refresh 在「滑鼠 app active(錶盤已被 gui_app_exit 拆掉、device_pager UI 是
   野指標)」時改呼叫,取代會 UNALIGNED 崩的 device_pager refresh()。只在單設備模式 + 有記住的
   設備時動作。必須在 LVGL 執行緒呼叫(device_pager_refresh 已是,skai_device_ui_refresh 有 defer)。 */
void instruction_list_refeed_single_device(void)
{
    if (!s_bar_single_device || s_single_device_id[0] == '\0')
        return;
    /* 手機 push 的即時選項(帶 @-contact service logo)優先於 registry placeholder。registry 的
       default_actions 不帶 svc,一旦 refeed 就把 push 設好的 service icon clear+rebuild 成預設框
       (真機 log 2026-06-30 證實:set_svc 設好 icon → 隨後的 refeed 把 icon 歸零 → logo 消失)。
       所以清單只要已顯示帶 logo 的 @ 列,就保留 push 版、不用 registry 覆蓋。push 會持續帶來電腦
       skaibar 的即時更新,不需要 registry placeholder 再蓋一次。 */
    for (uint8_t i = 0; i < list_item_count; i++)
        if (list_items[i].icon != NULL && list_items[i].category == '@')
            return;
    feed_single_device_options(s_single_device_id);
}

/* 公開：phone 下行 KEY_REMOTE_TEXT_FOCUS(0x17)呼叫 —— 更新「控制中那台設備目前有沒有聚焦
   輸入框」的快取旗標。單純存值,不觸發任何 UI;下一次 instruction_list_bar_tap_device() 的
   第一下 tap 才會讀它決定要不要跳過選項清單。LVGL/通訊執行緒皆可呼叫,單一 bool 寫入無需鎖。 */
void instruction_list_set_remote_target_focus(bool focused)
{
    s_remote_target_has_focus = focused;
    /* 刻意只存值、不碰 UI —— 這條路徑是通訊執行緒(0x17 下行),在這裡動 LVGL 物件就是本專案
       最典型的當機來源。立起輸入面板的 icon_send 顯示與否在「開面板的那一刻」決定
       (lift_input_layout_icons,已在 LVGL 執行緒);正常操作順序本來就是「先點電腦上的輸入框
       → 再立起手錶」,那時這個快取旗標早就是 true 了。面板開著期間才改變焦點的情形不成立:
       面板是全螢幕 overlay,手錶自己的觸控板被蓋住,使用者沒法用手錶去點電腦上的別的欄位。 */
}

/* 公開：只建立「單一控制設備」的瀏覽 session,**不開任何面板**。給滑鼠 app 的語音站用 ——
   它有自己的輸入 UI(不共用立起面板),但按 logo 送出後要叫出同一份選項清單,而
   instruction_list_open_browse() 第一件事就是 `if (!p_instruction_list_layout) return;`。
   滑鼠 app 從沒走過立起面板,那個 layout 根本沒被建立,清單於是靜默不出現
   (founder 2026-08-03:「只看到他返回觸碰板模式但沒看到有選項列表出現」)。
   這裡做的正是 instruction_list_open_lift_input_view() 的前半:建 layout、記住 device_id、
   把 registry 的 default_actions 灌進去當 placeholder。0x0e 由呼叫端自己送。 */
bool instruction_list_prepare_single_device(const char *device_id)
{
    if (device_id == NULL || device_id[0] == '\0')
        return false;
    if (!p_instruction_list_layout)
    {
        extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t * parent);
        lv_instruction_list_layout_create(lv_scr_act());
        if (!p_instruction_list_layout)
            return false;
    }
    /* R32/R33(2026-08-15 真機抓到「點 bar 沒有選項」):hosted 滑鼠進場會 release 列 UI
       換 heap,此後 refresh 一律 defer —— 底下 feed 的 refresh 會靜默無效=空清單;
       長按直開語音框(animate_open_ai_widget)那條更完全不經 open_browse 的 ensure。
       用 for_feed 輕量版:只解除 deferral,讓 feed 的 refresh 一次建出單設備清單
       (完整 ensure_ui 會先建舊清單再拆,heap 尖峰壓垮過聊天室,見該函式註解)。 */
    {
        extern void instruction_list_ensure_ui_for_feed(void);
        instruction_list_ensure_ui_for_feed();
    }
    s_bar_single_device = true;
    feed_single_device_options(device_id); /* 同時把 device_id 記進 s_single_device_id */
    return true;
}

/* 公開：讀那個快取旗標。滑鼠 app 的語音站要靠它決定 icon_send 出不出現(它自己有一份
   UI,不共用這裡的立起面板)。單一 bool 讀取,任何執行緒皆可。 */
bool instruction_list_remote_target_has_focus(void)
{
    return s_remote_target_has_focus;
}

/* 公開：滑鼠 app 單設備模式(搜尋抽屜/立起面板)是否正佔用共享清單。session_pager 的
   sp_inject_sessions_into_actions 用它擋注入 —— 單設備清單=「那一台電腦面板」的鏡像,
   各台桌面 0x20 定時重推的 session 不該混進來(2026-08-15 founder:「為什麼別台設備的
   session也會進來」)。單一 bool 讀取,任何執行緒皆可。 */
bool instruction_list_single_device_active(void)
{
    return s_bar_single_device;
}


/* 公開：滑鼠 app 離開(destroy / Exit)時呼叫 —— 若還在單設備模式,把共享清單還原成錶盤清單、
   通知電腦收掉它的 skaibar、清旗標,避免設備選項殘留在下次錶盤底部 bar。idempotent。 */
void instruction_list_bar_device_dismiss(void)
{
    if (!s_bar_single_device)
        return;
    s_bar_single_device = false;
    drawer_row_engage(false); /* 三鍵列釋放、mic_bar 還原(離場即放,R32) */
    s_remote_target_has_focus = false; /* 離開單設備模式 → 快取旗標歸零,別讓下一台/下一次殘留 */
    s_opened_by_lift = false;   /* drawer session 結束 */
    mic_bar_voice_stop();       /* 聽音中被收掉 → 停乾淨(no-op if idle) */
    extern bool commu_send_skaibar_dismiss(void);
    extern void instruction_list_restore_base(void);
    commu_send_skaibar_dismiss();
    instruction_list_restore_base();
    /* p_instruction_list_layout/list_bg 是共用單例,不保證離開滑鼠 app 時會被重建——保險
       歸零 translate_x,確保離開時 list_bg 回到乾淨的靜止位置(舊版 s_remote_target_has_
       focus 分支曾把它平移到畫面外且不清 HIDDEN,該分支已拿掉,這裡留著當防呆)。 */
    if (p_instruction_list_layout && p_instruction_list_layout->p_instruction_list_bg &&
        lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg))
    {
        lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
        lv_anim_del(list_bg, inst_list_slide_anim_cb);
        lv_obj_set_style_translate_x(list_bg, 0, 0);
    }
    /* 收掉全域 bar overlay(同 inst_list_slide_out_done_cb 的理由),避免滑鼠 app 離開後它殘留
       蓋住別頁。錶盤狀態機之後會重新顯示。 */
    if (s_global_bar_layer && lv_obj_is_valid(s_global_bar_layer))
        lv_obj_add_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN);
    { extern void hid_mouse_set_own_bar_hidden(bool); hid_mouse_set_own_bar_hidden(false); } /* 浮層 bar 收→當幀還原滑鼠自有 bar */
}

/* P3 公開:給 STANDALONE 滑鼠 app(把錶盤 gui_app_exit 拆掉那種)退出用。只通知電腦收 skaibar
   (0x0C)+ 清單設備旗標 + 忘掉記住的設備 id,【不】做 restore_base / hide overlay —— 因為
   standalone 退出時框架會重建錶盤(Main)自己的 instruction_list + overlay,任何對共享清單的
   操作都會打到 Main 的新清單(bar 消失 + WDT 凍結)。embedded device_pager 路徑(Main 活著、清單
   不重建)仍走完整的 instruction_list_bar_device_dismiss(restore_base + hide)。 */
void instruction_list_skaibar_dismiss_notify_only(void)
{
    if (!s_bar_single_device)
        return;
    s_bar_single_device = false;
    s_single_device_id[0] = '\0';
    extern bool commu_send_skaibar_dismiss(void);
    commu_send_skaibar_dismiss();
}

/* Forwarder for the upward tap-helper band (mic_hit). Forwards a TAP to
   mic_bar_event_cb ONLY while the bar is actually shown — so this off-bar helper
   stays inert wherever the bar is hidden (another app / AI widget open) without
   having to sync its own HIDDEN flag. */
static void mic_hit_event_cb(lv_event_t *evt)
{
    if (!p_instruction_list_layout || !p_instruction_list_layout->mic_bar ||
        lv_obj_has_flag(p_instruction_list_layout->mic_bar, LV_OBJ_FLAG_HIDDEN))
        return;
    mic_bar_event_cb(evt);
}

/* ==========================================================================
   滑鼠 app 單設備抽屜的底部三鍵列(founder 2026-08-17)
   --------------------------------------------------------------------------
   抽屜(上=這台電腦的 session 鏡像清單)底下不再是那條 176x31 的 skaibar,而是
   **語音站同一排**:左=地球(切輸入法→鍵盤輪盤)、中=麥克風、右=收下鍵。

   座標刻意與 hid_mouse 的 kbd_mic_section 那排逐格對齊(中心 ±110 / +170),
   所以按下麥克風進語音站時,語音站自己那排就落在同一個位置 —— 轉場中這排看起來
   完全不動,只有輸入框往上長、session 清單被往上推出畫面(founder 的描述)。
   兩邊的常數若要改,兩處要一起改(hid_mouse.c 的 VOICE_ROW_DY / VOICE_KBD_DX /
   VOICE_DEL_DX / VOICE_BTN_D)。

   heap:hosted 滑鼠模式下(R32/R33 紀律)一律「抽屜開才建、離場即放」。整排掛在
   s_global_bar_layer 底下,一個 lv_obj_del 全鏈收乾淨。
   ========================================================================== */
/* 這一段擺在檔案前段(緊鄰 mic_hit),用到的收尾/動畫工具都定義在後面 —— 前置宣告。 */
static void reveal_settle_anim_cb(void *var, int32_t v);
static void inst_list_slide_out_done_cb(lv_anim_t *a);
static void page_dim_track(lv_coord_t tx);
static void finalize_close_ai_widget(void);

#define DROW_DY      170  /* = hid_mouse VOICE_ROW_DY */
#define DROW_SIDE_DX 110  /* = hid_mouse VOICE_KBD_DX / VOICE_DEL_DX 的絕對值 */
#define DROW_MIC_D    64  /* = hid_mouse VOICE_BTN_D */
#define DROW_SLIDE_MS 260 /* 與 hid_mouse kbd_enter_slide 同步(進場 260ms) */

static lv_obj_t *s_drawer_row = NULL;   /* 三鍵列容器(透明,只當生命週期把手) */
static lv_obj_t *s_drawer_mic_btn = NULL;
/* s_drawer_row_shown 宣告在檔案前段(refresh_home_bar 要用),見該處說明。 */

static void drawer_row_release(void)
{
    if (s_drawer_row)
        LOG_W("[drawer] row release");
    if (s_drawer_row && lv_obj_is_valid(s_drawer_row))
        lv_obj_del(s_drawer_row);
    s_drawer_row = NULL;
    s_drawer_mic_btn = NULL;
    s_drawer_row_shown = false;
}

/* 地球:進輸入模式並直接停在鍵盤輪盤(=語音站裡按同一顆的效果)。 */
static void drawer_globe_cb(lv_event_t *e)
{
    (void)e;
    extern bool mouse_drawer_open_input(bool want_keyboard);
    mouse_drawer_open_input(true);
}

/* 收下鍵:抽屜態框裡永遠沒有字(有字時人已經在語音站),所以這顆恆為 down_arrow ——
   往下收掉整個抽屜、回到觸控板。 */
static void drawer_down_cb(lv_event_t *e)
{
    (void)e;
    extern void instruction_list_drawer_close_down(void);
    instruction_list_drawer_close_down();
}

/* 麥克風:**按住講話、放開停止**,與語音站那顆同手感(hid_mouse kbd_mic_btn_event_cb)。
   按下的當下就開語音站(輸入框往上長 + 清單推出畫面),手指仍停在原地 —— 語音站的
   麥克風就在同一個座標,整段讀起來是同一個連續手勢。
   PRESS_LOCK:轉場途中這顆會被藏起來,press session 必須留在它身上,放開才收得到
   RELEASED 去停錄音(否則變成錄音停不掉)。 */
static void drawer_mic_cb(lv_event_t *e)
{
    extern bool mouse_drawer_open_input(bool want_keyboard);
    extern void mouse_drawer_voice_set(bool on);
    switch (lv_event_get_code(e))
    {
    case LV_EVENT_PRESSED:
    {
        bool ok = mouse_drawer_open_input(false);
        LOG_W("[drawer] mic press -> open_input=%d", (int)ok);
        if (ok)
            mouse_drawer_voice_set(true);
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        mouse_drawer_voice_set(false);
        break;
    default:
        break;
    }
}

static lv_obj_t *drawer_row_add_icon(const void *src, lv_coord_t dx,
                                     lv_event_cb_t cb)
{
    lv_obj_t *img = lv_img_create(s_drawer_row);
    lv_img_set_src(img, src);
    lv_obj_align(img, LV_ALIGN_CENTER, dx, DROW_DY);
    lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(img, 14); /* 視覺可小,觸控補到 44pt 以上 */
    lv_obj_add_event_cb(img, cb, LV_EVENT_CLICKED, NULL);
    return img;
}

/* 建/顯示三鍵列。idempotent。 */
static void drawer_row_show(void)
{
    if (!s_global_bar_layer || !lv_obj_is_valid(s_global_bar_layer))
        return;
    LOG_W("[drawer] row show single=%d built=%d", (int)s_bar_single_device,
          (int)(s_drawer_row != NULL));
    if (s_drawer_row && lv_obj_is_valid(s_drawer_row))
    {
        lv_obj_clear_flag(s_drawer_row, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_drawer_row);
        s_drawer_row_shown = true;
        return;
    }
    s_drawer_row = lv_obj_create(s_global_bar_layer);
    lv_obj_remove_style_all(s_drawer_row);
    lv_obj_set_size(s_drawer_row, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_drawer_row, 0, 0);
    lv_obj_set_style_bg_opa(s_drawer_row, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(s_drawer_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_drawer_row, LV_OBJ_FLAG_CLICKABLE); /* 只有三顆鍵吃觸控 */

    drawer_row_add_icon(&erth, -DROW_SIDE_DX, drawer_globe_cb);

    /* 中間麥克風:語音站同款的深色圓鈕 + micro_icon。 */
    s_drawer_mic_btn = lv_obj_create(s_drawer_row);
    lv_obj_set_size(s_drawer_mic_btn, DROW_MIC_D, DROW_MIC_D);
    lv_obj_align(s_drawer_mic_btn, LV_ALIGN_CENTER, 0, DROW_DY);
    lv_obj_set_style_bg_color(s_drawer_mic_btn, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(s_drawer_mic_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_drawer_mic_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(s_drawer_mic_btn, 0, 0);
    lv_obj_set_style_pad_all(s_drawer_mic_btn, 0, 0);
    lv_obj_clear_flag(s_drawer_mic_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_drawer_mic_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_drawer_mic_btn, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(s_drawer_mic_btn, drawer_mic_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_drawer_mic_btn, drawer_mic_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_drawer_mic_btn, drawer_mic_cb, LV_EVENT_PRESS_LOST, NULL);
    lv_obj_t *mic_img = lv_img_create(s_drawer_mic_btn);
    lv_img_set_src(mic_img, &micro_icon);
    lv_obj_center(mic_img);
    lv_obj_clear_flag(mic_img, LV_OBJ_FLAG_CLICKABLE);

    drawer_row_add_icon(&down_arrow, DROW_SIDE_DX, drawer_down_cb);
    s_drawer_row_shown = true;
}

/* 抽屜態的底部造型切換:三鍵列上場 → 原本那條 mic_bar 與它的大 tap 區(mic_hit)一起讓開。
   離開抽屜時反向還原,錶盤路徑完全不受影響。 */
static void drawer_row_engage(bool on)
{
    lv_obj_t *bar = p_instruction_list_layout ? p_instruction_list_layout->mic_bar : NULL;
    if (bar && lv_obj_is_valid(bar))
    {
        if (on)
            lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(bar, LV_OBJ_FLAG_HIDDEN);
    }
    mic_hit_follow_bar(); /* 單一真相:大 tap 區永遠跟著 mic_bar 的顯藏 */
    if (on)
        drawer_row_show();
    else
        drawer_row_release();
}

/* 公開:抽屜開場時由 instruction_list_open_browse 呼叫(只在單設備模式)。 */
void instruction_list_drawer_row_engage(bool on)
{
    drawer_row_engage(on);
}

/* 公開:由 hid_mouse 的 40ms poll(bar_ai_sync_timer_cb)每拍呼叫 —— 抽屜/語音站期間
   **強制**那條舊 pill 保持隱藏。
   為什麼要用強制執行而不是繼續找兇手:mic_bar 的顯藏至少有六個寫入點(立起面板進/出、
   refresh_home_bar、抽屜三鍵列、animate_open_ai_widget、close_ai_widget),而且進場鏈
   的最後一棒就是專門「把 pill 叫出來」的 refresh_home_bar。逐一堵了三輪、founder 三輪
   都還看得到它 —— 與其再賭第七個寫入點,不如讓一個每拍都跑的裁決者收尾:任何路徑都只能
   讓它閃一幀。順便印出來指名(前幾次),下次要根治時直接有兇手名單。
   例外:ai box 開著=長按直開語音的 morph,那條正拿 mic_bar 當放大起點,不能動它。 */
void instruction_list_drawer_enforce_bar_hidden(void)
{
    if (!s_bar_single_device || !p_instruction_list_layout)
        return; /* 錶盤/一般清單照舊,完全不受影響 */
    lv_obj_t *bar = p_instruction_list_layout->mic_bar;
    if (!bar || !lv_obj_is_valid(bar) || lv_obj_has_flag(bar, LV_OBJ_FLAG_HIDDEN))
        return;
    lv_obj_t *box = p_instruction_list_layout->p_instruction_list_ai_bg;
    if (box && lv_obj_is_valid(box) && !lv_obj_has_flag(box, LV_OBJ_FLAG_HIDDEN))
        return; /* morph 進行中,pill 是它的起點 */
    static uint8_t s_enforce_log_n = 0;
    if (s_enforce_log_n < 5)
    {
        s_enforce_log_n++;
        LOG_W("[drawer] !! pill visible during drawer (row=%d) -> force hide",
              (int)s_drawer_row_shown);
    }
    lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);
    mic_hit_follow_bar();
}

/* ---- 抽屜 ↔ 語音站的垂直轉場 --------------------------------------------
   進語音站:清單往**上**推出畫面(founder:「把 session 列表往上推出畫面」);
   下拉收合回來:清單從上方滑回。收下鍵:清單往**下**收掉(對齊 down_arrow 的方向)。
   translate 是 draw-only —— 清單即使移到畫面外仍會全螢幕 hit-test,所以動畫結束
   一定要補 HIDDEN,否則語音站按不動(這條在本檔多處都踩過)。 */
static void drawer_slide_y_cb(void *var, int32_t v)
{
    lv_obj_t *list_bg = (lv_obj_t *)var;
    if (lv_obj_is_valid(list_bg))
        lv_obj_set_style_translate_y(list_bg, (lv_coord_t)v, 0);
}

/* 往上推出後:藏起來 + 釋放列 UI 換 heap(語音站/鍵盤輪盤要用)。**不**清
   s_bar_single_device —— 電腦面板還開著、鏡像還在推,下拉收合要原地回來。 */
static void drawer_push_up_done_cb(lv_anim_t *a)
{
    (void)a;
    LOG_W("[drawer] push_up done -> hide list+layer, release list UI");
    lv_obj_t *list_bg = p_instruction_list_layout
                            ? p_instruction_list_layout->p_instruction_list_bg
                            : NULL;
    if (list_bg && lv_obj_is_valid(list_bg))
    {
        lv_obj_add_flag(list_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_y(list_bg, 0, 0);
    }
    if (s_global_bar_layer && lv_obj_is_valid(s_global_bar_layer))
    {
        lv_obj_set_style_bg_opa(s_global_bar_layer, LV_OPA_0, 0);
        lv_obj_add_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN);
    }
    /* **只藏不刪**:使用者這一刻很可能還按著三鍵列的麥克風(按住講話)。lv_obj_del 掉
       正被按著的物件,indev 的 act_obj 就沒了 —— RELEASED/PRESS_LOST 不保證還會送到,
       錄音就停不掉。PRESS_LOCK 已設在麥克風上,物件只是 HIDDEN 的話 press session
       仍留在它身上,放開照樣收得到。真正的釋放交給 drawer_row_engage(false)(離開整個
       抽屜流程時),留著這一小塊(~1-2K)換掉一整類 UAF/卡錄音的風險。 */
    if (s_drawer_row && lv_obj_is_valid(s_drawer_row))
        lv_obj_add_flag(s_drawer_row, LV_OBJ_FLAG_HIDDEN);
    s_drawer_row_shown = false;
    {
        extern void instruction_list_release_ui(void);
        instruction_list_release_ui();
    }
    is_open_instruction_list_ai = false;
}

void instruction_list_drawer_push_up(void)
{
    if (!p_instruction_list_layout)
        return;
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    LOG_W("[drawer] push_up bg=%d hidden=%d", (int)(list_bg != NULL),
          (int)(list_bg && lv_obj_is_valid(list_bg) &&
                lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN)));
    if (!list_bg || !lv_obj_is_valid(list_bg) ||
        lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN))
    {
        /* 清單本來就不在畫面上(長按直開語音那條):只要把三鍵列藏掉即可(同上,不刪)。 */
        if (s_drawer_row && lv_obj_is_valid(s_drawer_row))
            lv_obj_add_flag(s_drawer_row, LV_OBJ_FLAG_HIDDEN);
        s_drawer_row_shown = false;
        return;
    }
    lv_anim_del(list_bg, inst_list_slide_anim_cb);
    lv_anim_del(list_bg, reveal_settle_anim_cb);
    lv_anim_del(list_bg, drawer_slide_y_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, list_bg);
    lv_anim_set_exec_cb(&a, drawer_slide_y_cb);
    lv_anim_set_values(&a, 0, -LV_VER_RES);
    lv_anim_set_time(&a, DROW_SLIDE_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, drawer_push_up_done_cb);
    lv_anim_start(&a);
}

/* 從語音站下拉收合回抽屜:重建列 UI、用**最新**的鏡像資料重餵(founder:「拉回已經
   有對應輸入變更過的選項」),清單再從上方滑回原位。 */
void instruction_list_drawer_slide_in(void)
{
    if (!p_instruction_list_layout || !s_bar_single_device)
        return;
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    if (!list_bg || !lv_obj_is_valid(list_bg))
        return;
    {
        extern void instruction_list_ensure_ui_for_feed(void);
        instruction_list_ensure_ui_for_feed();
    }
    feed_single_device_options(s_single_device_id); /* 語音搜尋後的最新選項 */
    if (s_global_bar_layer && lv_obj_is_valid(s_global_bar_layer))
        lv_obj_clear_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN);
    drawer_row_engage(true);
    lv_obj_clear_flag(list_bg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_translate_x(list_bg, 0, 0);
    lv_obj_set_style_translate_y(list_bg, -LV_VER_RES, 0);
    page_dim_track(0); /* 單設備模式=整頁壓暗到底 */
    is_open_instruction_list_ai = false;
    lv_anim_del(list_bg, drawer_slide_y_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, list_bg);
    lv_anim_set_exec_cb(&a, drawer_slide_y_cb);
    lv_anim_set_values(&a, -LV_VER_RES, 0);
    lv_anim_set_time(&a, DROW_SLIDE_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
    {
        extern void check_main_page(void);
        check_main_page();
    }
}

/* 收下鍵:清單往下收掉,落地走 inst_list_slide_out_done_cb 那條既有的完整收尾
   (release_ui → restore_base → 收 overlay → 還原滑鼠自有 bar → 通知電腦)。 */
static void drawer_close_down_done_cb(lv_anim_t *a)
{
    lv_obj_t *list_bg = p_instruction_list_layout
                            ? p_instruction_list_layout->p_instruction_list_bg
                            : NULL;
    if (list_bg && lv_obj_is_valid(list_bg))
        lv_obj_set_style_translate_y(list_bg, 0, 0);
    drawer_row_engage(false);
    {
        extern bool commu_send_skaibar_dismiss(void);
        commu_send_skaibar_dismiss(); /* 抽屜是使用者主動收的 → 電腦面板也收掉 */
    }
    inst_list_slide_out_done_cb(a); /* 共用收尾:hide + release + restore_base */
}

void instruction_list_drawer_close_down(void)
{
    if (!p_instruction_list_layout)
        return;
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    if (!list_bg || !lv_obj_is_valid(list_bg))
        return;
    lv_anim_del(list_bg, inst_list_slide_anim_cb);
    lv_anim_del(list_bg, reveal_settle_anim_cb);
    lv_anim_del(list_bg, drawer_slide_y_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, list_bg);
    lv_anim_set_exec_cb(&a, drawer_slide_y_cb);
    lv_anim_set_values(&a, 0, LV_VER_RES);
    lv_anim_set_time(&a, DROW_SLIDE_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, drawer_close_down_done_cb);
    lv_anim_start(&a);
}

/* The box is scrollable only to ABSORB swipes (so a left-swipe doesn't bubble to
   the page nav and jump to the watchface). When the user actually starts a swipe
   on the open box, dismiss it with the animated reverse morph — mirrors the
   right device_pager's scroll_hides_skaibar_cb. */
static void ai_box_scroll_dismiss_cb(lv_event_t *evt)
{
    (void)evt;
    if (!is_open_instruction_list_ai)
        return;
    /* R76: on the session page a full close empties the left tile → R24 snaps
       home. A swipe starting on the box there = collapse the box, keep the
       (filtered) list for browsing — same resting state as a list scroll. */
    if (s_session_page_mode)
        s_scroll_keep_list = true;
    close_ai_widget();
}

/* Dismiss the AI widget when the user scrolls the instruction list while it is
   open. Modal-ish dismiss model (per office-hours doc Q4): scroll = explicit
   dismissal; user re-invokes via bar tap or release. Kept as a flag for the
   open path to clear; the scroll dismiss itself now routes through
   close_ai_widget (see ai_widget_fade_on_scroll) so every close shares one
   animated reverse morph. */
static bool ai_fade_anim_active = false;

/* Baseline opacities for each animated property when fully visible.
   Match the values set in lv_instruction_list_layout_create. */
#define PILL_BG_IMG_BASELINE  LV_OPA_COVER
#define TRANSCRIPT_BASELINE   LV_OPA_80
#define VOICEBTN_BG_BASELINE  LV_OPA_10
#define VOICEIMG_BASELINE     LV_OPA_COVER

/* Apply a fade fraction (0..255) to all visual sub-parts of the pill.
   value=255 → fully visible (each property = its baseline).
   value=0   → fully invisible. LVGL's lv_obj_set_style_opa does NOT
   cascade in this build, so each sub-part gets its own per-property
   opa call: img_opa for images, text_opa for labels, bg_opa for solid
   bg fills. */
static void skai_widget_fade_anim_cb(void *var, int32_t value)
{
    (void)var;
    lv_opa_t f = (lv_opa_t)value;  /* 0..255 fade factor */
    if (s_pill_bg_img && lv_obj_is_valid(s_pill_bg_img))
    {
        lv_obj_set_style_img_opa(s_pill_bg_img, f, 0);
    }
    if (s_voice_transcript_label && lv_obj_is_valid(s_voice_transcript_label))
    {
        lv_obj_set_style_text_opa(s_voice_transcript_label,
                                  (lv_opa_t)((TRANSCRIPT_BASELINE * f) / 255),
                                  0);
    }
    if (ai_voice_btn && lv_obj_is_valid(ai_voice_btn))
    {
        lv_obj_set_style_bg_opa(ai_voice_btn,
                                (lv_opa_t)((VOICEBTN_BG_BASELINE * f) / 255), 0);
    }
    if (s_voice_img && lv_obj_is_valid(s_voice_img))
    {
        lv_obj_set_style_img_opa(s_voice_img, f, 0);
    }
}

static void ai_widget_fade_on_scroll(void)
{
    /* Scrolling the list = switch to manual browse: collapse the voice input box
       back to the slim bar (stops v2t) but KEEP the list up so you can scroll and
       pick an option. A full close (swipe-right / tap-close / page-leave) slides
       the whole list out instead. No-op when the box isn't open. */
    if (!is_open_instruction_list_ai)
        return;
    s_scroll_keep_list = true;
    close_ai_widget();
}

/* Reset all visual sub-parts to their baseline (fully-visible) opacities.
   Called by animate_open_ai_widget so a prior fade doesn't leave the
   pill stuck at low opacity. */
static void skai_widget_restore_full_opa(void)
{
    if (s_pill_bg_img && lv_obj_is_valid(s_pill_bg_img))
    {
        lv_obj_set_style_img_opa(s_pill_bg_img, PILL_BG_IMG_BASELINE, 0);
    }
    if (s_voice_transcript_label && lv_obj_is_valid(s_voice_transcript_label))
    {
        lv_obj_set_style_text_opa(s_voice_transcript_label, TRANSCRIPT_BASELINE,
                                  0);
    }
    if (ai_voice_btn && lv_obj_is_valid(ai_voice_btn))
    {
        lv_obj_set_style_bg_opa(ai_voice_btn, VOICEBTN_BG_BASELINE, 0);
    }
    if (s_voice_img && lv_obj_is_valid(s_voice_img))
    {
        lv_obj_set_style_img_opa(s_voice_img, VOICEIMG_BASELINE, 0);
    }
}

void tap_on_ai_widget(void);

/* ---- bottom mic-bar → input-box MORPH (mirrors the right device_pager skaibar)
   Trigger the slim bottom mic bar → it grows + slides up into the 442x252 input
   box (phase 1), THEN the skai_widget pill (frame + transcript + voice button)
   fades in on top of that box backdrop (phase 2). Close reverses it. */

/* Guard: blocks a second trigger while the open morph is mid-flight (phase 1,
   before is_open_instruction_list_ai is set in tap_on_ai_widget). */
static bool s_left_morph_busy = false;
/* Guard: true while the reverse-morph close is animating shut (mirrors the right
   skaibar's `skaibar_active` gating). Blocks re-opening mid-close and re-entrant
   close calls. Cleared when the close finishes (finalize_close_ai_widget). */
static bool s_left_closing = false;
/* 立起旗標的時刻。s_left_closing 只在 finalize_close_ai_widget() 被清 —— 而那是一串
   動畫(pill fade → bar shrink)的最後一棒,中間任何一段被 lv_anim_del 掉、或物件在動畫
   途中被釋放,finalize 就永遠不會跑,旗標卡在 true → 之後**每一次**點 bar 都被
   open_browse 開頭那個 early-out 默默擋掉,症狀=「怎麼點都不出現 session 列表」。
   open_browse 用這個時刻做 staleness 自癒(founder 2026-08-17 回報)。 */
static rt_tick_t s_left_closing_tick = 0;
#define LEFT_CLOSING_STALE_TICKS 1500 /* ms;正常關閉動畫全程 < 500ms */
/* list_bg 滑出關閉動畫進行中:擋退出途中(本地 restore 或手機重推)觸發的 UI 重繪閃變。 */
static bool s_list_sliding_out = false;

/* Interpolate the mic bar between its slim bar geometry (f=0) and the full
   input-box geometry (f=255); fade the mic glyph out as it grows. The bar is
   the box backdrop — the skai_widget pill fades in over it. */
static void lmic_grow_cb(void *var, int32_t f)
{
    (void)var;
    lv_obj_t *bar = p_instruction_list_layout ? p_instruction_list_layout->mic_bar : NULL;
    if (!bar || !lv_obj_is_valid(bar)) return;
    lv_obj_set_size(bar, LMIC_W + (LBOX_W - LMIC_W) * f / 255,
                    LMIC_H + (LBOX_H - LMIC_H) * f / 255);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, LMIC_Y + (LBOX_Y - LMIC_Y) * f / 255);
    lv_obj_set_style_radius(bar, LMIC_RADIUS + (LBOX_RADIUS - LMIC_RADIUS) * f / 255, 0);
    /* Rest transparent (the skaibar_img child shows instead) → box backdrop dark@80
       as it grows; crossfades with the image (s_mic_bar_icon) fading out below.
       Border stays 0 (set at creation); the box frame image is the open border. */
    lv_obj_set_style_bg_opa(bar, (lv_opa_t)(LV_OPA_80 * f / 255), 0);
    if (s_mic_bar_icon && lv_obj_is_valid(s_mic_bar_icon))
        lv_obj_set_style_img_opa(s_mic_bar_icon, (lv_opa_t)(255 - f), 0);
}

/* Phase 2 (open): the bar finished growing into the box → reveal the skai_widget
   pill, fading its frame + transcript + voice button in. Carries the original
   widget-reveal logic (tileview show + tap_on_ai_widget + VAD wiring); the mic
   bar is NOT hidden — it stays as the box backdrop. */
static void lmic_open_reveal_cb(lv_anim_t *a)
{
    (void)a;
    s_left_morph_busy = false;
    if (!p_instruction_list_layout) return;
    /* Show the box (now a plain container, not a tileview — no set_tile_id). */
    lv_obj_clear_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                      LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(p_instruction_list_layout->p_instruction_list_ai_bg);
    tap_on_ai_widget();
    /* Keep the box container transparent — only the frame image + label show. */
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_0, 0);
    /* Placeholder while listening, replaced by the transcript (mirrors
       device_pager skaibar_open's "聽取中"). */
    instruction_list_set_voice_transcript(LV_EXT_STR_GET_BY_KEY(listening, "Listening"));
    /* Fade the frame + label in (the box "fills" with its content). */
    if (s_skai_widget && lv_obj_is_valid(s_skai_widget))
    {
        skai_widget_fade_anim_cb(NULL, 0); /* start invisible */
        lv_anim_t fr;
        lv_anim_init(&fr);
        lv_anim_set_var(&fr, s_skai_widget);
        lv_anim_set_values(&fr, 0, 255);
        lv_anim_set_time(&fr, LMORPH_FRAME_MS);
        lv_anim_set_path_cb(&fr, lv_anim_path_ease_out);
        lv_anim_set_exec_cb(&fr, skai_widget_fade_anim_cb);
        lv_anim_start(&fr);
    }
#ifdef BSP_USING_PC_SIMULATOR
    {
        extern void handle_ai_voice_btn_vad(bool speaking);
        lvgl_msg_handler.handle_vad_status = handle_ai_voice_btn_vad;
    }
#endif
}

/* Slide the floating instruction list horizontally via translate_x (R3 left
   drawer). A draw-only offset — no layout/scroll recalculation and no layer
   buffer — so it's cheap on EPIC. Translating X leaves the list's vertical
   snap-scroll math (which reads y1) untouched. */
static void inst_list_slide_anim_cb(void *var, int32_t v)
{
    if (lv_obj_is_valid((lv_obj_t *)var))
        lv_obj_set_style_translate_x((lv_obj_t *)var, (lv_coord_t)v, 0);
}

/* Full-close slide-out finished: the list has parked off-screen right — hide it
   and drop the watch-face blur (set_blur self-guards on other pages). The hide
   lives HERE (not in finalize_close_ai_widget) so a full close that runs when the
   box is ALREADY collapsed — e.g. swipe-right from the browse state — still slides
   the whole list out, instead of finalize snapping it hidden mid-slide. */
static void inst_list_slide_out_done_cb(lv_anim_t *a)
{
    (void)a;
    s_entry_landing_pending = false; /* R50:清單關了,進場落點的任務結束 */
    /* R55:搜尋字不跨開關殘留 —— 下次拉開要看到完整清單,不是上次講到一半的篩選結果。 */
    if (s_text_filter[0] != '\0')
    {
        s_text_filter[0] = '\0';
        cat_filter_restore_full();
    }
    if (p_instruction_list_layout &&
        p_instruction_list_layout->p_instruction_list_bg &&
        lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg))
        lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_bg,
                        LV_OBJ_FLAG_HIDDEN);
    if (s_global_bar_layer && lv_obj_is_valid(s_global_bar_layer))
        lv_obj_set_style_bg_opa(s_global_bar_layer, LV_OPA_0, 0);
    {
        extern void instruction_list_bar_set_blur(bool on);
        instruction_list_bar_set_blur(false);
    }
    /* P2 S2 — list is gone; drop the transient category view so the real (full /
       disconnect-filtered) list_items[] is what everything else sees. s_view_cat is
       cleared too: the list closed, so a later push has no open view to re-apply. */
    s_list_sliding_out = false; /* 列表此時已 HIDDEN(見上),清 flag 讓 off-screen restore 照常重繪 */
    cat_filter_restore_full();
    s_view_cat = 0;
    /* 滑鼠 app 單設備模式:列表完全收掉 → 把共享清單還原成錶盤清單、退出單設備模式,
       否則設備選項會殘留到下次錶盤底部 bar。錶盤 / device_pager 路徑 flag 為 false,無影響。 */
    if (s_bar_single_device)
    {
        s_bar_single_device = false;
        drawer_row_engage(false); /* 三鍵列釋放、mic_bar 還原(離場即放,R32) */
        /* R32 heap 紀律(2026-08-15):先釋放列 UI、再還原資料 —— restore_base 的 refresh
           會撞 R33 deferral 只還原 list_items[](不重建 9 列 UI),聊天室/觸控板才有 heap;
           順序反過來=restore 先建 9 列、下面 release 再拆=白付一次 heap 尖峰(這尖峰在
           聊天室第二輪把 lv_obj_create 壓到回 NULL,hard fault 實測)。 */
        {
            extern bool app_control_get_mouse_mode(void);
            if (app_control_get_mouse_mode())
            {
                extern void instruction_list_release_ui(void);
                instruction_list_release_ui();
            }
        }
        extern void instruction_list_restore_base(void);
        instruction_list_restore_base();
        /* 關鍵:把整個全域 bar overlay(s_global_bar_layer,layer_top 全螢幕)一起隱藏。滑鼠 app
           開清單時 open_browse 顯示了它,但滑鼠 app 不歸錶盤狀態機管、沒人收 → 它底部的全域
           mic_bar 會蓋住滑鼠 app 自己的 bar,害「第二次點 bar」點到全域 bar(走錶盤廣播、不送
           0x0E,電腦不再開)。收掉它,滑鼠 app 自己的 bar 才能再被點、下一次 tap 才走單設備 0x0E。
           錶盤路徑 s_bar_single_device=false 不會進這裡,全域 bar 仍由狀態機常駐。 */
        if (s_global_bar_layer && lv_obj_is_valid(s_global_bar_layer))
            lv_obj_add_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN);
        { extern void hid_mouse_set_own_bar_hidden(bool); hid_mouse_set_own_bar_hidden(false); } /* 浮層 bar 收→當幀還原滑鼠自有 bar */
    }
    /* 2026-08-12 卡死修:清單「真正 HIDDEN」是這一刻(動畫跑完),但關閉路徑
       (back 手勢 close_ai_widget + R16 snap_to_home、或電腦刪 session 觸發的
       refresh)常在清單還在滑出、is_visible() 仍 true 時就跑過 check_main_page,
       把 _at_instruction_list latch 成 true → 邊緣區(通知/媒體/控制中心手勢)被
       關掉後沒人再 poll,回到錶盤看得到畫面卻四向全滑不出。清單此刻已藏,補跑一次
       重評估讓 latch 翻回 false、display_gesture_detect_objs/status_bar_area 重新
       開放邊緣區。
       R23(founder 定位:「進去過 session 列表再回錶盤就會觸發」):這裡原本只重評
       **清單**那一個 latch,但真正把四條邊緣 zone 重新打開的是 `check_is_at_home`
       的進場分支(display_status_bar_area(0..3,true) / reveal overlay)。只翻
       _at_instruction_list 的話 _at_home 仍卡 false —— 畫面回到錶盤、手勢全死,
       左緣還留著 app 返回鍵。改叫 check_main_page():它依序重評 message /
       instruction_list / control_center / home,順序本身就是相依順序。
       R24(真兇,診斷坐實):log 顯示 `[ATINST] 1->0 visible=0 idx=4` 之後**沒有**
       `[ATHOME] 0->1` —— 清單滑掉了,但 **tileview 還停在左頁
       (idx=MAIN_PAGE_TYPE_RIGHT=4)**。左格透明所以看起來像回到錶盤,可是
       check_is_at_home 的 on_device_page 判定直接把 idx==4 當「非錶盤」否決,
       四條 zone 永遠不開。R16 只在 chat 返回 / ESC 關清單兩條路徑補了
       snap_to_home,「人在左頁直接把清單滑掉」這條沒有;這裡是所有關閉路徑的
       共同出口,還停在左頁就 snap 回 HOME(INSTANT,走 set_tile_id 讓 settle
       收尾照跑),再重評狀態機。hosted 滑鼠模式的 tileview 由滑鼠圖層自己管,
       不要碰。 */
    {
        extern uint8_t get_middle_layer_tileview_index(void);
        extern void snap_to_home_from_any_page(void);
        extern bool lv_top_panel_mouse_mode(void);
        if (!lv_top_panel_mouse_mode() &&
            get_middle_layer_tileview_index() == MAIN_PAGE_TYPE_RIGHT)
        {
            LOG_W("[R24] list hidden but tileview parked at left -> snap home");
            snap_to_home_from_any_page();
        }
    }
    /* R43:R33 曾在這裡「一關清單就釋放列物件」,為的是擠出 heap 給滑鼠頁。但滑鼠頁
       OOM 的真兇後來量出來是**鍵盤模式 UI 的 38KB**(R40 改成延遲建立就解決了),所以
       這個釋放只剩代價:每次開清單都要重建整份列,使用者看到的就是「文字先進來、icon
       晚半秒才出現」(founder 連續三輪回報)。改回常駐 —— 滑鼠模式那條路徑仍會
       release(見 app_clock_status_bar 進滑鼠模式處),真正需要記憶體的時候才付這個
       代價。 */
    {
        extern void check_main_page(void);
        check_main_page();
    }
}

/* ---- watch-face right-edge left-pull → finger-reveal the list (L/R swap) ----
   Mirror of the list_window_scroll_event_cb hdrag (which finger-drags the list
   OUT to the right to close). The dial's right-edge overlay (created below) drives
   these: begin/update/end translate the SAME p_instruction_list_bg from its
   parked +LV_HOR_RES toward 0, finger-following. A far/fast release settles into
   the existing BROWSE state (list shown, voice box NOT opened — we deliberately
   skip animate_open_ai_widget / tap_on_ai_widget); a short release slides it back
   out + hides it (inst_list_slide_out_done_cb). */
static bool s_reveal_drag_active = false;
static void reset_list_internal(void); /* fwd: a reveal re-bottoms the list */

/* Pulled far enough: the list finished sliding to 0 → land in the browse state.
   Same resting state ai_widget_fade_on_scroll leaves (list up, box hidden,
   is_open false), minus the box-collapse — we never opened the box. The blur
   stays ON (the browse state keeps the watch-face blur). */
static void reveal_settle_browse_done_cb(lv_anim_t *a)
{
    (void)a;
    is_open_instruction_list_ai = false;
    /* Settled in: top the finger-followed blur up to full (the drag may have
       released before the list was all the way across). */
    {
        extern void instruction_list_bar_set_blur(bool on);
        instruction_list_bar_set_blur(true);
    }
    if (p_instruction_list_layout &&
        p_instruction_list_layout->p_instruction_list_ai_bg &&
        lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_ai_bg))
        lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                        LV_OBJ_FLAG_HIDDEN);
    /* P2 S2 — the view filter is applied at reveal start (drag_begin), not here, so
       the list shows the right content during the finger-drag. Nothing to do on
       settle now. */
    /* ADR-0024 round-trip: a view just settled open (s_view_cat was set at drag_begin:
       '@' left / '/' right / 0 bar). Tell the phone so it fans the matching query to
       every device and pushes THAT view's per-device result list (not a filter of one
       shared 16-item list). The client cat filter above still shows an instant subset of
       the prior list until the re-queried list arrives. */
    /* 滑鼠 app 單設備模式:0x0E 已在 bar tap1 當下送出(見 instruction_list_bar_tap_device),
       這裡不再送 view 廣播 —— 否則會把手機踢出單設備模式、推回聚合清單。只有錶盤/一般路徑
       (非單設備)才在 settle 送 view 廣播。 */
    if (!s_bar_single_device)
    {
        extern bool commu_send_skaibar_view(char cat);
        commu_send_skaibar_view(s_view_cat);
    }
}

/* Watch-face dial blur as a function of the list's translate_x: full at 0 (list
   all the way in), none at +LV_HOR_RES (parked off-screen right). No-op off the
   watch face, so the shared hdrag/close paths leave other pages' backdrops alone.
   Shared by the reveal (overlay) and the right-drag close so the blur tracks the
   list both ways. */
static void dial_blur_track(lv_coord_t tx)
{
    extern bool clock_main_page_is_home(void);
    if (!clock_main_page_is_home())
        return;
    lv_coord_t pulled = LV_HOR_RES - LV_ABS(tx); /* 0 .. LV_HOR_RES, either edge */
    if (pulled < 0) pulled = 0;
    if (pulled > LV_HOR_RES) pulled = LV_HOR_RES;
    {
        extern void instruction_list_bar_set_blur_amount(uint8_t opa);
        instruction_list_bar_set_blur_amount(
            (uint8_t)((pulled * LV_OPA_COVER) / LV_HOR_RES));
    }
}

/* Full-screen page dim for the mouse-page list. Sets the s_global_bar_layer bg
   to BLACK every frame (remove_style_all leaves it white otherwise) and ramps
   opa with how far the list slid in. invalidate forces a whole-screen redraw so
   the entire trackpad darkens, not just the strip the list slides over. */
static void page_dim_track(lv_coord_t tx)
{
    if (!s_global_bar_layer || !lv_obj_is_valid(s_global_bar_layer))
        return;
    if (!s_bar_single_device)
    {
        lv_obj_set_style_bg_opa(s_global_bar_layer, LV_OPA_0, 0);
        return;
    }
    lv_coord_t pulled = LV_HOR_RES - LV_ABS(tx);
    if (pulled < 0) pulled = 0;
    if (pulled > LV_HOR_RES) pulled = LV_HOR_RES;
    lv_obj_set_style_bg_color(s_global_bar_layer, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_global_bar_layer,
                            (lv_opa_t)((pulled * LV_OPA_80) / LV_HOR_RES), 0);
    lv_obj_invalidate(s_global_bar_layer);
}

/* Fade the bottom mic pill in WITH the watch-face left-edge reveal (finger-follow):
   opacity tracks how far the list has slid in, mirroring dial_blur_track. Without
   this the pill popped to full opacity the instant the reveal began. ONLY the
   left-edge reveal (s_reveal_from_left) fades it — programmatic opens (the mouse
   page's bar tap → open_browse, which parks on the right) keep the bar opaque so the
   bar you just tapped doesn't blink back from transparent. */
static void bar_reveal_opa_track(lv_coord_t tx)
{
    if (!s_reveal_from_left)
        return;
    if (!s_mic_bar_icon || !lv_obj_is_valid(s_mic_bar_icon))
        return;
    lv_coord_t pulled = LV_HOR_RES - LV_ABS(tx);
    if (pulled < 0) pulled = 0;
    if (pulled > LV_HOR_RES) pulled = LV_HOR_RES;
    /* Fade the skaibar IMAGE itself (img_opa) — the same property lmic_grow_cb uses
       to dissolve it. A parent LV_STYLE_OPA on mic_bar did NOT visibly fade the
       child image during the finger drag (the pill stayed invisible until the
       settle, so it looked like it popped in at the end). */
    lv_obj_set_style_img_opa(s_mic_bar_icon,
                             (lv_opa_t)((pulled * LV_OPA_COVER) / LV_HOR_RES), 0);
}

/* Settle-animation exec: drive the list translate AND keep the finger-followed
   blur tracking it, so the dial blur ramps smoothly THROUGH the release animation
   instead of jumping at its end (plain inst_list_slide_anim_cb moves only X). */
static void reveal_settle_anim_cb(void *var, int32_t v)
{
    lv_obj_t *list_bg = (lv_obj_t *)var;
    if (!lv_obj_is_valid(list_bg))
        return;
    lv_obj_set_style_translate_x(list_bg, (lv_coord_t)v, 0);
    dial_blur_track((lv_coord_t)v);
    page_dim_track((lv_coord_t)v);
    bar_reveal_opa_track((lv_coord_t)v); /* ramp the bottom mic pill THROUGH the settle */
}

/* Start a finger-reveal: un-hide the parked list and turn the watch-face blur on
   (finger-follow: it fades in as you pull). No-op if a close is animating or the
   list is already up (browse / open) — the reveal is only for pulling a hidden
   list back in. Does NOT touch is_open / start v2t / morph the box. */
/* External driver entry for the full-screen face swipe detector: pick the entry edge
   + view filter atomically, then run the standard finger-follow begin. The edge-strip
   handlers set these two file-private statics inline before begin(); this exposes the
   same setup so a caller outside this file (the watch-face catcher) can start an
   identical reveal. from_left=true + filter=0 == the left-edge mixed-list reveal. */
void instruction_list_reveal_drag_begin(void); /* defined just below */
void instruction_list_reveal_drag_begin_ex(bool from_left, char filter)
{
    s_reveal_from_left = from_left;
    s_pending_reveal_filter = filter;
    instruction_list_reveal_drag_begin();
}

void instruction_list_reveal_drag_begin(void)
{
    if (!p_instruction_list_layout)
        return;
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    if (!list_bg || !lv_obj_is_valid(list_bg))
        return;
    if (s_left_closing || !lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN))
        return;
    /* R37(founder:「從錶盤進入 session 列表時右邊的圖片會先透明的後面才出現」):
       R33 起清單關閉就釋放列的物件,而**手指拖曳進場**這條路徑不像 open_browse 會先
       ensure —— 列是後來某次 refresh 才補建的,所以圖標比清單本體晚一步浮現。手指
       一開始拖就先把列建回來,進場第一幀就是完整內容。 */
    /* R51(founder:「進場過程停在最下面的 actions,之後才跳到 session」):跳動的根源是
       **進場當下清單裡還沒有 session** —— 它們要等手機推播落地才由 session pager 注入。
       R50 讓落點在注入後補正,但補正本身就是看得到的那一跳。這裡在建 UI **之前**先把
       pager 已經握有的 session 資料注入進來,第一幀就是完整清單、落點一次到位。
       (資料早就在 pager 的 s_devices 裡 —— 手機每 5 秒推一次;缺的只是「注入」這個動作。) */
    {
        extern void session_list_actions_changed(void);
        session_list_actions_changed();
    }
    {
        extern void instruction_list_ensure_ui(void);
        instruction_list_ensure_ui();
    }
    instruction_list_bar_set_visible(true); /* idempotent on HOME */
    lv_anim_del(list_bg, inst_list_slide_anim_cb);
    lv_obj_clear_flag(list_bg, LV_OBJ_FLAG_HIDDEN);
    /* Park off-screen on the entry edge; drag_update finger-tracks it toward 0. */
    lv_obj_set_style_translate_x(list_bg,
                                 s_reveal_from_left ? -LV_HOR_RES : LV_HOR_RES, 0);
    /* R50:這次是進場 —— 落點規則要一路管到「sessions 稍後才注入」的那次重建為止。 */
    s_entry_landing_pending = true;
    s_entry_landing_tick = rt_tick_get(); /* R80:寬限窗起點 */
    /* R54(founder:「點 session 列表下面的麥克風沒反應」):`s_session_page_mode` 原本只在
       **整頁 settle 到左頁**時才設(app_clock_status_bar 的 active_pos == MAIN_PAGE_TYPE_RIGHT),
       但從錶盤邊緣拖曳拉出來的是**浮層**這條路,tileview 根本沒換頁 → 旗標留 false →
       麥克風走「開 AI 輸入框」那條而不是開新 session。兩條路開的是同一個左頁內容,語意
       就該一致。滑鼠模式下這個清單是被 hid_mouse 借去當 skaibar 的,維持原本行為。 */
    {
        extern bool lv_top_panel_mouse_mode(void);
        if (!lv_top_panel_mouse_mode())
            s_session_page_mode = true;
    }
    reset_list_internal(); /* R49 起:落點 = 最新的 session,沒有 session 才是清單最後一項 */
    /* R47(founder 定案:「應該是把定位拿掉,先回到原本不做定位的版本」):R9 的
       focus_first_action 整個退場 —— 兩條 reveal 都不定位,落點就是 reset_list_internal
       的結果(清單最下面那項),跟左緣原本的行為一致。R46 曾把它挪到這裡(改成進場前定位
       以消除跳動),但 founder 要的是連定位本身都不要。 */
    /* Backdrop behind the list: transparent on the watch face (blurred dial shows
       through), light scrim elsewhere — same rule as animate_open_ai_widget. */
    {
        lv_obj_set_style_bg_opa(list_bg, LV_OPA_0, 0);
        if (s_global_bar_layer && lv_obj_is_valid(s_global_bar_layer))
            lv_obj_set_style_bg_opa(s_global_bar_layer, LV_OPA_0, 0);
    }
    /* The watch-face blur is faded in by the drag updates (finger-follow), not
       switched on here — see instruction_list_reveal_drag_update. */
    /* Apply this reveal's view filter NOW — the list is un-hidden but parked
       off-screen, so the rebuild happens before it slides in and the finger-drag
       shows the CORRECT content. (Previously applied at settle, which left the
       prior reveal's content visible during the whole pull.) s_pending_reveal_filter
       was set by the caller — the edge overlay at lock, or open_browse to 0 (all). */
    instruction_list_set_category_filter(s_pending_reveal_filter);
    s_reveal_drag_active = true;
    /* The list is now shown — surface the bottom mic pill with it (the bar that
       used to sit on the bare watch face). */
    instruction_list_refresh_home_bar();
    /* Start the pill transparent for the left-edge reveal so it fades in WITH the
       list; drag_update / reveal_settle_anim_cb ramp it to full as the list slides
       in. No-op for programmatic (non-left) opens, which keep the bar opaque. */
    bar_reveal_opa_track(s_reveal_from_left ? -LV_HOR_RES : LV_HOR_RES);
}

/* Finger-follow: dx = how far LEFT the finger has pulled from the press point
   (negative for a leftward pull). Drives the parked list from +LV_HOR_RES toward 0
   (mirror of the hdrag clamp). */
void instruction_list_reveal_drag_update(lv_coord_t dx)
{
    if (!s_reveal_drag_active || !p_instruction_list_layout)
        return;
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    if (!list_bg || !lv_obj_is_valid(list_bg))
        return;
    /* Right edge parks at +HOR_RES (dx<0 leftward pull → tx toward 0); left edge
       parks at -HOR_RES (dx>0 rightward pull → tx toward 0). Mirror the clamp. */
    lv_coord_t park = s_reveal_from_left ? -LV_HOR_RES : LV_HOR_RES;
    lv_coord_t tx = park + dx;
    if (s_reveal_from_left)
    {
        if (tx < -LV_HOR_RES) tx = -LV_HOR_RES;
        if (tx > 0) tx = 0;
    }
    else
    {
        if (tx < 0) tx = 0;
        if (tx > LV_HOR_RES) tx = LV_HOR_RES;
    }
    lv_anim_del(list_bg, inst_list_slide_anim_cb);
    lv_obj_set_style_translate_x(list_bg, tx, 0);
    dial_blur_track(tx); /* finger-follow blur: fade in with the pull */
    page_dim_track(tx);
    bar_reveal_opa_track(tx); /* finger-follow: fade the bottom mic pill in too */
}

/* Release: settle by the live offset. Revealed past a quarter OR flung left fast
   → finish into browse (slide to 0); else slide back out + hide. RIGHT drawer:
   parked at +LV_HOR_RES, pulled in toward 0. */
void instruction_list_reveal_drag_end(lv_coord_t dx, lv_coord_t vx)
{
    (void)dx;
    if (!s_reveal_drag_active)
        return;
    s_reveal_drag_active = false;
    if (!p_instruction_list_layout)
        return;
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    if (!list_bg || !lv_obj_is_valid(list_bg))
        return;
    lv_coord_t tx = lv_obj_get_style_translate_x(list_bg, 0);
    /* Reveal committed if pulled in past ~a quarter or flung toward 0 — mirror the
       thresholds per entry edge (right: tx ≤ ¾·HOR or leftward fling; left:
       tx ≥ -¾·HOR or rightward fling). */
    bool open_it = s_reveal_from_left
                       ? ((tx >= -LV_HOR_RES * 3 / 4) || (vx > 6))
                       : ((tx <= LV_HOR_RES * 3 / 4) || (vx < -6));
    lv_anim_del(list_bg, inst_list_slide_anim_cb);
    lv_anim_del(list_bg, reveal_settle_anim_cb);
    lv_anim_t sl;
    lv_anim_init(&sl);
    lv_anim_set_var(&sl, list_bg);
    lv_anim_set_exec_cb(&sl, reveal_settle_anim_cb);
    if (open_it)
    {
        /* Haptic at the release instant: the pull crossed the open threshold,
           so buzz the moment the finger lifts — not when the settle animation
           lands. Gated on the global toggle; the sister IMU/bar-tap path buzzes
           in instruction_list_open_browse. (A release that falls short and
           springs shut takes the else branch, so no buzz there.) */
        if (get_scrolling_motor_vibrate_status())
        {
            motor_pattern_scrolling_app();
        }
        lv_anim_set_values(&sl, tx, 0);
        lv_anim_set_time(&sl, LSLIDE_MS);
        lv_anim_set_path_cb(&sl, lv_anim_path_ease_out);
        lv_anim_set_ready_cb(&sl, reveal_settle_browse_done_cb);
    }
    else
    {
        lv_anim_set_values(&sl, tx, s_reveal_from_left ? -LV_HOR_RES : LV_HOR_RES);
        lv_anim_set_time(&sl, 150);
        lv_anim_set_path_cb(&sl, lv_anim_path_ease_in);
        lv_anim_set_ready_cb(&sl, inst_list_slide_out_done_cb);
    }
    lv_anim_start(&sl);
}

/* One-shot open to the browse state — no finger drag. The watch-face IMU release
   gesture (handle_gesture_unlock) uses this to bring the floating list in (list
   only, the box stays shut), replacing the pre-R3 animate_to_instruction_list tile
   nav — the LEFT tile is empty post-R3. Reuses the reveal begin (un-hide + park +
   backdrop) then runs the SAME settle-open animation the drag-release uses, so the
   dial blur ramps through and it lands in browse via reveal_settle_browse_done_cb. */
void instruction_list_open_browse(void)
{
    if (!p_instruction_list_layout)
        return;
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    if (!list_bg || !lv_obj_is_valid(list_bg))
        return;
    /* R32:進滑鼠頁時把列的 UI 釋放掉換 heap,這裡是開清單的共同入口 —— 先原路
       重建回來(沒被 release 過就是 no-op)。 */
    {
        extern void instruction_list_ensure_ui(void);
        instruction_list_ensure_ui();
    }
    /* 自癒:關閉旗標卡住超過 LEFT_CLOSING_STALE_TICKS 就當它是死的,強制收尾後照常開。
       沒有這一段的話,一次被中斷的關閉動畫會讓 bar 從此永遠點不開(founder 2026-08-17)。 */
    if (s_left_closing &&
        (rt_tick_get() - s_left_closing_tick) >
            rt_tick_from_millisecond(LEFT_CLOSING_STALE_TICKS))
    {
        LOG_W("[land] stale s_left_closing (%u ticks) -> self-heal",
              (unsigned)(rt_tick_get() - s_left_closing_tick));
        finalize_close_ai_widget();
    }
    /* Only when parked & hidden — already up (browse/open) or sliding shut: leave it. */
    if (s_left_closing || !lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN))
    {
        LOG_W("[land] open_browse early-out closing=%d hidden=%d",
              (int)s_left_closing,
              (int)lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN));
        return;
    }
    /* Haptic at the trigger instant: the IMU release gesture (handle_gesture_
       unlock) or the bar tap just fired and cleared the guards, so the list is
       committed to opening — buzz now, not on settle. Mirrors the drag-release
       path in instruction_list_reveal_drag_end. */
    if (get_scrolling_motor_vibrate_status())
    {
        motor_pattern_scrolling_app();
    }
    s_pending_reveal_filter = 0; /* bar / IMU browse → all (@ + /) view */
    s_reveal_from_left = true;   /* IMU release brings the LEFT list in (the right-edge
                                    drawer is legacy); park on the left, slide to 0 */
    instruction_list_reveal_drag_begin(); /* un-hide + park + backdrop */
    /* 滑鼠 app 單設備抽屜:底部換成語音站同款三鍵列(地球/麥克風/收下),原本那條
       mic_bar 與它的大 tap 區讓開(founder 2026-08-17)。錶盤路徑不進這裡。
       **一定要排在 reveal_drag_begin() 之後** —— 它的最後一行就是
       instruction_list_refresh_home_bar()「list is now shown → surface the bottom mic
       pill」,專門負責把那條 pill 叫出來。排在前面的話就是「我先藏、它後開」,
       結果三鍵列旁邊永遠多一顆舊的小麥克風(founder 連三輪回報)。 */
    if (s_bar_single_device)
        drawer_row_engage(true);
    s_reveal_drag_active = false;         /* gesture-triggered, not a finger drag */
    lv_coord_t tx = lv_obj_get_style_translate_x(list_bg, 0);
    lv_anim_del(list_bg, inst_list_slide_anim_cb);
    lv_anim_del(list_bg, reveal_settle_anim_cb);
    lv_anim_t sl;
    lv_anim_init(&sl);
    lv_anim_set_var(&sl, list_bg);
    lv_anim_set_exec_cb(&sl, reveal_settle_anim_cb);
    lv_anim_set_values(&sl, tx, 0);
    lv_anim_set_time(&sl, LSLIDE_MS);
    lv_anim_set_path_cb(&sl, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&sl, reveal_settle_browse_done_cb);
    lv_anim_start(&sl);
    /* Arm the Main state machine NOW rather than wait up to 1s for the
       gui_state_update poll: the list bg was un-hidden synchronously in
       reveal_drag_begin above, so instruction_list_is_visible() already reads true.
       check_main_page flips _at_instruction_list true (instruction_list_resume =>
       scroll/tap armed) and, in the same pass, _at_home false. */
    extern void check_main_page(void);
    check_main_page();
}

/* True whenever the floating list is shown on screen as the instruction-list
   surface — AI input box open OR closed. The Main state machine
   (check_is_at_instruction_list) keys off this so scroll/tap/back drive the floating
   list instead of the now-empty LEFT tile. It deliberately does NOT exclude the
   box-open case: is_at_instruction_list() must stay true while the voice box is up,
   or back can't route to back_on_skai_widget (watch_demo handle_back_event) and the
   watch lands in no recognised state. The box-open vs browse distinction is made
   separately via get_is_open_instruction_list_ai(). Exposed as a query so
   app_mainmenu need not reach into our internals. */
bool instruction_list_is_visible(void)
{
    return p_instruction_list_layout &&
           p_instruction_list_layout->p_instruction_list_bg &&
           lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg) &&
           !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_bg,
                            LV_OBJ_FLAG_HIDDEN);
}

/* 浮層 bar 容器(s_global_bar_layer)實際是否可見 —— 滑鼠 app 用來 frame-對齊它自有底部 bar
   的隱藏(浮層 bar 一現就收自有 bar、一收就還原)。純唯讀查詢、不改任何顯示行為,故錶盤完全
   不受影響。注意這查的是「整條浮層 bar 層」,不像 instruction_list_is_visible() 只看清單。 */
bool instruction_list_floating_bar_visible(void)
{
    if (!s_global_bar_layer || !lv_obj_is_valid(s_global_bar_layer) ||
        lv_obj_has_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN))
        return false;
    /* 舉起帶出的輸入面板單獨顯示時不算「浮層 bar 已出現」——這個情境下
       mic_bar 本身是隱藏的(見 instruction_list_open_lift_input_view)，滑鼠 app
       自有的底部 bar(skaibar_img)應該維持原樣顯示，不被這裡的 sync 誤收掉。 */
    if (s_opened_by_lift && s_lift_input_view && lv_obj_is_valid(s_lift_input_view) &&
        !lv_obj_has_flag(s_lift_input_view, LV_OBJ_FLAG_HIDDEN))
        return false;
    return true;
}

/* ---- right-edge reveal overlay: the input source for the reveal API above ----
   A thin (LREVEAL_EDGE_W) clickable strip pinned to the screen's right edge on
   s_global_bar_layer, kept in front of the list. On the watch face it's enabled
   (instruction_list_reveal_overlay_set_enabled, driven by check_is_at_home); a
   leftward pull from the edge finger-reveals the list. The handler mirrors the
   list_window_scroll_event_cb hdrag — only leftward, feeding the reveal API. */
#define LREVEAL_EDGE_W 20
static lv_obj_t *s_reveal_edge_overlay = NULL;
static lv_obj_t *s_reveal_edge_overlay_left = NULL; /* P2 S3: left-edge @ pull */
static lv_coord_t s_reveal_start_x = 0;
static lv_coord_t s_reveal_start_y = 0;
static bool s_reveal_axis_locked = false;

static void reveal_edge_overlay_event_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_get_act();
    switch (e->code)
    {
    case LV_EVENT_PRESSED:
        s_reveal_from_left = false; /* right-edge drag parks the list on the RIGHT */
        s_reveal_axis_locked = false;
        if (indev)
        {
            lv_point_t pt;
            lv_indev_get_point(indev, &pt);
            s_reveal_start_x = pt.x;
            s_reveal_start_y = pt.y;
        }
        break;
    case LV_EVENT_PRESSING:
    {
        if (!indev) break;
        lv_point_t pt;
        lv_indev_get_point(indev, &pt);
        lv_coord_t dx = pt.x - s_reveal_start_x;
        lv_coord_t dy = pt.y - s_reveal_start_y;
        /* Lock onto a clearly-leftward pull only (right-edge drawer: a leftward
           pull from the right edge reveals the list). A vertical drag never locks,
           so it can't reveal. */
        if (!s_reveal_axis_locked && dx < -10 && (-dx) > LV_ABS(dy))
        {
            s_reveal_axis_locked = true;
            s_pending_reveal_filter = '/'; /* right-edge pull → action (/) view */
            instruction_list_reveal_drag_begin();
        }
        if (s_reveal_axis_locked)
            instruction_list_reveal_drag_update(dx);
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
    {
        if (!s_reveal_axis_locked) break;
        s_reveal_axis_locked = false;
        lv_coord_t dx = 0;
        lv_coord_t vx = 0;
        if (indev)
        {
            lv_point_t pt;
            lv_indev_get_point(indev, &pt);
            dx = pt.x - s_reveal_start_x;
            lv_point_t v = {0, 0};
            lv_indev_get_vect(indev, &v);
            vx = v.x;
        }
        instruction_list_reveal_drag_end(dx, vx);
        break;
    }
    default:
        break;
    }
}

/* P2 S3 — left-edge RIGHTWARD pull → reveal the @-chat list. Unlike the right
   edge (finger-following leftward pull into the right-parked list), the left edge
   fires the programmatic open once the pull locks: the reveal geometry parks the
   list on the right, so a left finger-follow would move opposite the list. A short
   rightward flick from the left edge opens the @ browse view. Shares the right
   edge's start/lock state (only one edge can be pressed at a time). */
static void reveal_edge_overlay_left_event_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_get_act();
    switch (e->code)
    {
    case LV_EVENT_PRESSED:
        s_reveal_from_left = true; /* left-edge drag parks the list on the LEFT */
        s_reveal_axis_locked = false;
        if (indev)
        {
            lv_point_t pt;
            lv_indev_get_point(indev, &pt);
            s_reveal_start_x = pt.x;
            s_reveal_start_y = pt.y;
        }
        break;
    case LV_EVENT_PRESSING:
    {
        if (!indev) break;
        lv_point_t pt;
        lv_indev_get_point(indev, &pt);
        lv_coord_t dx = pt.x - s_reveal_start_x;
        lv_coord_t dy = pt.y - s_reveal_start_y;
        /* Lock onto a clearly-rightward pull (mirror of the right edge's leftward
           lock): the @ list, parked off-screen LEFT, finger-follows toward 0 — so
           it tracks the finger in from the left edge the instant the pull begins. */
        if (!s_reveal_axis_locked && dx > 10 && dx > LV_ABS(dy))
        {
            s_reveal_axis_locked = true;
            /* 2026-06-25: left-edge pull now reveals the MIXED list (all
               categories), not the @-only chat view. The @ / / split is gone on
               the watch; one combined list slides in from the left. */
            s_pending_reveal_filter = 0; /* left-edge pull → mixed (all) view */
            instruction_list_reveal_drag_begin();
        }
        if (s_reveal_axis_locked)
            instruction_list_reveal_drag_update(dx);
        break;
    }
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
    {
        if (!s_reveal_axis_locked) break;
        s_reveal_axis_locked = false;
        lv_coord_t dx = 0;
        lv_coord_t vx = 0;
        if (indev)
        {
            lv_point_t pt;
            lv_indev_get_point(indev, &pt);
            dx = pt.x - s_reveal_start_x;
            lv_point_t v = {0, 0};
            lv_indev_get_vect(indev, &v);
            vx = v.x;
        }
        instruction_list_reveal_drag_end(dx, vx);
        break;
    }
    default:
        break;
    }
}

/* Enable (show) / disable (hide) the right-edge reveal overlay. A hidden overlay
   gets no input, so this gates the gesture. Driven by check_is_at_home: live on
   the watch face, off everywhere else (tile nav / page gestures own the edge
   there). No-op until the overlay exists. */
void instruction_list_reveal_overlay_set_enabled(bool enabled)
{
    /* 2026-06-25: the RIGHT-edge reveal is retired — the right edge now swipes to
       the App List tile (HOME LV_DIR_RIGHT). Keep its overlay permanently hidden
       so it never intercepts that swipe. Only the LEFT-edge overlay (now the
       mixed list) stays gated by the watch-face `enabled` flag. */
    if (s_reveal_edge_overlay && lv_obj_is_valid(s_reveal_edge_overlay))
    {
        lv_obj_add_flag(s_reveal_edge_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    /* The left-edge (mixed-list) overlay follows the watch-face gating. */
    if (s_reveal_edge_overlay_left && lv_obj_is_valid(s_reveal_edge_overlay_left))
    {
        if (enabled)
            lv_obj_clear_flag(s_reveal_edge_overlay_left, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(s_reveal_edge_overlay_left, LV_OBJ_FLAG_HIDDEN);
    }
}

void animate_open_ai_widget(void)
{
    /* Bluetooth gate (founder 2026-06-05): the input box is the voice→text entry,
       which needs the phone — so when disconnected, don't open it; surface the
       not-connected tip instead. This reverses the earlier always-open policy. It's
       the single funnel for EVERY box-open path (bar tap, 2nd release gesture,
       gravity-AI), so the gate lives here once. The browse LIST stays reachable
       offline via the release gesture (it self-filters to standalone items per
       item_is_standalone) — only the box is gated. */
    extern bool get_bluetooth_connection_status(void);
    extern void create_connection_tips(void);
    if (!get_bluetooth_connection_status())
    {
        create_connection_tips();
        /* The bar-tap caller (mic_bar_event_cb) turns the dial blur ON before it
           calls us, expecting a box to float over it. We're refusing the open, so
           undo that blur — UNLESS the browse list is already up (then the blur is
           the list's, keep it). instruction_list_is_visible() tells the two apart;
           the release/gravity callers set no blur, so this is a no-op for them. */
        extern void instruction_list_bar_set_blur(bool on);
        if (!instruction_list_is_visible())
            instruction_list_bar_set_blur(false);
        LOG_I("animate_open_ai_widget: phone disconnected — showing tips, not "
              "opening box");
        return;
    }
    /* Ignore re-triggers while the morph is opening, already shown, or animating
       shut (would restart the grow mid-flight and glitch). Gate on the box's
       ACTUAL visibility, not the is_open flag — a stuck-true flag would
       otherwise block a legitimate open from a closed (hidden) box. During the
       grow morph the box is still hidden, but s_left_morph_busy covers that. */
    bool box_visible =
        p_instruction_list_layout &&
        p_instruction_list_layout->p_instruction_list_ai_bg &&
        !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                         LV_OBJ_FLAG_HIDDEN);
    if (s_left_morph_busy || s_left_closing || box_visible)
        return;
    last_ai_widget_open_time = rt_tick_get();
    /* R3: snapshot the current instruction list before in-session options (voice
       suggestions on the watch face / device options on the device page) overwrite
       it, so a close can restore it. Guarded — one snapshot per session (device-
       page entry may already have taken it). */
    {
        extern void instruction_list_save_base(void);
        instruction_list_save_base();
    }
    ai_widget_opened_by_drag = false;
    /* Cancel any in-flight scroll-fade so the widget doesn't immediately
       fade back out after the morph completes. */
    if (ai_fade_anim_active)
    {
        if (s_skai_widget && lv_obj_is_valid(s_skai_widget))
        {
            lv_anim_del(s_skai_widget, skai_widget_fade_anim_cb);
        }
        ai_fade_anim_active = false;
    }
    /* Restore all sub-part opacities to their baselines (in case a prior
       fade left them dim). */
    skai_widget_restore_full_opa();

    /* Reveal the floating instruction list (on s_global_bar_layer / lv_layer_top)
       behind the morphing input box, SLIDING it in from the right edge (right
       drawer). Slid back out + hidden by the close path. The watch-face blur
       backdrop is turned on by the caller (mic_bar_event_cb) via
       instruction_list_bar_set_blur. */
    {
        lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
        if (list_bg && lv_obj_is_valid(list_bg))
        {
            /* Slide in only on a FRESH open (the list was hidden). If it's already
               up — the box was collapsed by a scroll but the list kept open — just
               re-open the box over it, with no re-slide. */
            bool was_hidden = lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(list_bg, LV_OBJ_FLAG_HIDDEN);
            /* Backdrop behind the list: transparent on the watch face (the blurred
               dial shows through gaus_dial_bg), but slightly dark on other pages —
               the mouse page has no blur, so a light scrim makes the list stand
               out over the bright trackpad. */
            {
                lv_obj_set_style_bg_opa(list_bg, LV_OPA_0, 0);
            }
            if (was_hidden)
            {
                lv_anim_del(list_bg, inst_list_slide_anim_cb);
                lv_obj_set_style_translate_x(list_bg, LV_HOR_RES, 0);
                lv_anim_t sl;
                lv_anim_init(&sl);
                lv_anim_set_var(&sl, list_bg);
                lv_anim_set_values(&sl, LV_HOR_RES, 0);
                lv_anim_set_time(&sl, LSLIDE_MS);
                lv_anim_set_path_cb(&sl, lv_anim_path_ease_out);
                lv_anim_set_exec_cb(&sl, inst_list_slide_anim_cb);
                lv_anim_start(&sl);
            }
        }
    }

    /* MORPH phase 1: grow the slim bottom mic bar up into the input box; phase 2
       (lmic_open_reveal_cb) fades the skai_widget pill in over it. */
    lv_obj_t *bar = p_instruction_list_layout->mic_bar;
    if (bar && lv_obj_is_valid(bar))
    {
        s_left_morph_busy = true;
        lv_anim_del(bar, lmic_grow_cb);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_HIDDEN);
        if (s_mic_bar_icon && lv_obj_is_valid(s_mic_bar_icon))
            lv_obj_clear_flag(s_mic_bar_icon, LV_OBJ_FLAG_HIDDEN);
        lmic_grow_cb(NULL, 0); /* reset to slim-bar geometry, glyph visible */
        lv_anim_t g;
        lv_anim_init(&g);
        lv_anim_set_var(&g, bar);
        lv_anim_set_values(&g, 0, 255);
        lv_anim_set_time(&g, LMORPH_GROW_MS);
        lv_anim_set_path_cb(&g, lv_anim_path_ease_out);
        lv_anim_set_exec_cb(&g, lmic_grow_cb);
        lv_anim_set_ready_cb(&g, lmic_open_reveal_cb);
        lv_anim_start(&g);
    }
    else
    {
        lmic_open_reveal_cb(NULL); /* no bar — reveal directly */
    }
}

/* ---- reverse-morph close (mirrors the right device_pager skaibar_close) ----
   The old close snapped the box-sized mic bar back to the slim pill instantly.
   To match the right side, the close now animates in two sequenced phases:
     phase 1 (frame): the pill's frame + transcript + voice button fade out.
     phase 2 (grow):  the box-sized mic bar shrinks/slides back to the slim
                      bottom pill.
   The AI strip is hidden only once phase 2 finishes. close runs frame → THEN
   grow (the inverse of the open's grow → THEN frame), exactly like the right's
   skaibar_close. s_left_closing guards re-entry / re-open while it's in flight. */

/* Synchronous teardown of the box container objects — hide the AI strip, reset
   its tile + bg, restore the slim mic bar. Runs at the end of the close morph,
   or immediately on the no-animation path. */
static void finalize_close_ai_widget(void)
{
    s_left_closing = false;
    s_left_morph_busy = false;
    if (!p_instruction_list_layout)
        return;
    if (p_instruction_list_layout->mic_bar &&
        lv_obj_is_valid(p_instruction_list_layout->mic_bar))
    {
        lv_anim_del(p_instruction_list_layout->mic_bar, lmic_grow_cb);
        lmic_grow_cb(NULL, 0); /* slim-bar geometry + glyph fully visible */
        lv_obj_clear_flag(p_instruction_list_layout->mic_bar,
                          LV_OBJ_FLAG_HIDDEN);
    }
    set_skai_widget_opa(0);
    if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
    {
        lv_obj_add_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
    }
    if (p_instruction_list_layout->p_instruction_list_ai_bg &&
        lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_ai_bg))
    {
        lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                                LV_OPA_0, 0);
        /* Box is a plain container now (no tileview) — just hide it. */
        lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                        LV_OBJ_FLAG_HIDDEN);
    }
    /* finalize tears the BOX down only. The floating list + its blur are left
       alone here: a full close hides the list via the slide-out's ready_cb
       (inst_list_slide_out_done_cb); a scroll box-collapse keeps the list up.
       Decoupling the two is what lets a scroll dismiss just the voice box. */
}

/* close phase 2 done: the bar has shrunk back to the slim pill — hide the strip. */
static void lmic_close_done_cb(lv_anim_t *a)
{
    (void)a;
    finalize_close_ai_widget();
}

/* close phase 1 done (pill faded out): shrink the box-sized mic bar back down. */
static void lmic_close_shrink_cb(lv_anim_t *a)
{
    (void)a;
    lv_obj_t *bar = p_instruction_list_layout
                        ? p_instruction_list_layout->mic_bar
                        : NULL;
    if (!bar || !lv_obj_is_valid(bar))
    {
        finalize_close_ai_widget();
        return;
    }
    lv_anim_t g;
    lv_anim_init(&g);
    lv_anim_set_var(&g, bar);
    lv_anim_set_values(&g, 255, 0);
    lv_anim_set_time(&g, LMORPH_GROW_MS);
    lv_anim_set_path_cb(&g, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&g, lmic_grow_cb);
    lv_anim_set_ready_cb(&g, lmic_close_done_cb);
    lv_anim_start(&g);
}

void close_ai_widget(void)
{
    extern void clear_skai_widget_ai_reply(void);
    /* A scroll collapses just the box (keep the list up); every other close path
       slides the whole list out. Consume the one-shot flag FIRST so it can't leak
       into a later close even if this call early-returns while already closing. */
    bool keep_list = s_scroll_keep_list;
    s_scroll_keep_list = false;
    bool is_cancel = s_close_is_cancel;
    s_close_is_cancel = false;
    if (s_left_closing)
        return; /* already animating shut */
    /* R3: an explicit CANCEL full-close (left-swipe / bar tap, not a commit and
       not a scroll box-collapse) tells the phone to dismiss the active skaibar so
       its option list reverts. The phone treats the v2t stop as a pause (not a
       close), so without this the device's / launcher's skaibar stays open and the
       watch list keeps the in-session options. */
    if (!keep_list)
    {
        /* Watch face: reset our own list to the snapshot taken on open — the phone
           launcher's in-session options overwrote it (both a cancel and a commit
           end back on the instruction list). Device page: skip — s_base is the
           watch-face list kept for the page-leave restore, and the dismissed device
           re-streams its default options which we re-feed. */
        extern bool clock_main_page_is_home(void);
        /* 滑鼠 app 單設備模式(s_bar_single_device=true):restore_base 交給滑出動畫完成後的
           inst_list_slide_out_done_cb —— 它先把列表設 HIDDEN 再 restore,使用者看不到內容變。
           若在這裡(動畫開始前)提前 restore,會同步 refresh_custom_instructions() 把單設備
           選項換成錶盤 base 清單,而滑出動畫仍在跑、列表還可見 → 退出途中閃一下變成 actions。
           錶盤/一般路徑(flag=false)維持原本的提前 restore 不變。 */
        if (clock_main_page_is_home() && !s_bar_single_device)
        {
            extern void instruction_list_restore_base(void);
            instruction_list_restore_base();
        }
        /* Any explicit CANCEL close (not a commit) tells the phone to dismiss the
           active skaibar so the device's / launcher's panel closes too. */
        if (is_cancel)
        {
            extern bool commu_send_skaibar_dismiss(void);
            commu_send_skaibar_dismiss();
        }
    }
    stop_mock_inst_update();
    /* Voice / state teardown happens up-front (mirrors the right skaibar_close,
       which stops v2t at the START of the close — not after the animation).
       Three v2t calls because the event-driven STOP path in
       voice_recognition_entry has a "skip if AI processing" early-return that
       leaves voice_recognition_started=true — so the next mic-bar tap's
       VOICE_RECOGNITION_START event short-circuits and the phone never gets a
       fresh handshake. We hit all three teardown surfaces directly so the
       dismiss is unconditional:
         1. voice_provider.stop_v2t()  — fires the async STOP event
         2. stop_voice_recognition()  — sync: clears voice2TextStatus,
            unsubscribes mic, sends user-speaking-end notify
         3. set_voice_recognition_started(false)  — sync: clears the
            re-entry gate the event handler normally clears. */
    voice_provider.stop_v2t();
    stop_voice_recognition(V2T_INTENT_NOTHING);
    set_voice_recognition_started(false);
    set_paused_control_with_arm(false);
    set_ai_open_mic(false);
    skai_widget_shown = false;
    ai_widget_opened_by_drag = false;
    // lvgl_msg_handler.handle_vad_status = NULL;
    clear_skai_widget_ai_reply();

    bool was_open = is_open_instruction_list_ai;
    is_open_instruction_list_ai = false;

    /* FULL close (not a scroll box-collapse): slide the whole floating list out to
       the right; the slide's ready_cb hides it + drops the blur. The hide lives in
       the ready_cb (not finalize) so this still works when the box is ALREADY
       collapsed (swipe-right from the browse state) — there was_open is false and
       finalize runs synchronously, which would otherwise snap the list hidden
       mid-slide. A scroll box-collapse (keep_list) leaves the list untouched. */
    if (!keep_list)
    {
        lv_obj_t *list_bg = p_instruction_list_layout
                                ? p_instruction_list_layout->p_instruction_list_bg
                                : NULL;
        if (list_bg && lv_obj_is_valid(list_bg) &&
            !lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN))
        {
            lv_anim_del(list_bg, inst_list_slide_anim_cb);
            lv_anim_del(list_bg, reveal_settle_anim_cb);
            lv_anim_t sl;
            lv_anim_init(&sl);
            lv_anim_set_var(&sl, list_bg);
            lv_anim_set_values(&sl, lv_obj_get_style_translate_x(list_bg, 0),
                               s_reveal_from_left ? -LV_HOR_RES : LV_HOR_RES);
            lv_anim_set_time(&sl, LSLIDE_MS);
            lv_anim_set_path_cb(&sl, lv_anim_path_ease_in);
            lv_anim_set_exec_cb(&sl, reveal_settle_anim_cb);
            lv_anim_set_ready_cb(&sl, inst_list_slide_out_done_cb);
            s_list_sliding_out = true; /* 滑出關閉動畫啟動 → 擋退出途中的 rebuild 閃變 */
            lv_anim_start(&sl);
        }
        else
        {
            /* List already hidden — nothing to slide; just drop any blur we own. */
            extern void instruction_list_bar_set_blur(bool on);
            instruction_list_bar_set_blur(false);
        }
    }

    /* Box not on screen (or layout gone): finish synchronously — nothing to
       morph. Guards against a stray close while closed re-growing the slim bar. */
    if (!was_open || !p_instruction_list_layout)
    {
        finalize_close_ai_widget();
        return;
    }

    /* Cancel any in-flight open morph so the reverse morph starts clean. */
    lv_obj_t *bar = p_instruction_list_layout->mic_bar;
    if (bar && lv_obj_is_valid(bar))
        lv_anim_del(bar, lmic_grow_cb);
    if (s_skai_widget && lv_obj_is_valid(s_skai_widget))
        lv_anim_del(s_skai_widget, skai_widget_fade_anim_cb);
    ai_fade_anim_active = false;

    /* Assert the box-open visual state as the morph's starting point
       (mirrors skaibar_close's grow(255)/frame(255)). */
    lmic_grow_cb(NULL, 255);        /* box-sized backdrop */
    skai_widget_restore_full_opa(); /* pill fully visible */
    if (bar && lv_obj_is_valid(bar))
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_HIDDEN);

    s_left_closing = true;
    s_left_closing_tick = rt_tick_get();

    /* Phase 1: fade the pill out; phase 2 (bar shrink) chains off ready_cb. */
    if (!s_skai_widget || !lv_obj_is_valid(s_skai_widget))
    {
        lmic_close_shrink_cb(NULL); /* no pill — go straight to the shrink */
        return;
    }
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_skai_widget);
    lv_anim_set_values(&a, 255, 0);
    lv_anim_set_time(&a, LMORPH_FRAME_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&a, skai_widget_fade_anim_cb);
    lv_anim_set_ready_cb(&a, lmic_close_shrink_cb);
    lv_anim_start(&a);
}

/* Public "tap outside the widget = user cancel" entry point for callers OUTSIDE
   this file that own their own full-screen touch surface — currently
   hid_mouse.c's plain_event_cb (the mouse app's trackpad background), which
   sits UNDER s_global_bar_layer in LVGL's input search order and only ever
   sees a tap when the top-layer search finds nothing there first. That
   happens for the s_remote_target_has_focus flow (instruction_list_bar_tap_
   device): p_instruction_list_bg is parked off-screen via translate_x, and in
   this LVGL v8 build translate_x is folded into the object's real x/y by
   lv_obj_refr_pos (NOT a draw-only transform, despite older comments in this
   file assuming so) — so its hit-test rect is genuinely off-screen too, and a
   tap on the visible screen falls through to whatever real on-screen object is
   underneath instead of ever reaching list_window_scroll_event_cb. Exposing
   close as a tiny cancel-flagged wrapper (mirroring every other tap-outside
   path's s_close_is_cancel = true; close_ai_widget();) lets the trackpad's own
   click handler be the one that notices "the widget is open" and closes it,
   rather than trying to out-compete touch_bg in hit-test order.

   Deliberately does NOT touch list_bg's position/HIDDEN flag itself (two
   earlier attempts did, both wrong): forcing HIDDEN before calling
   close_ai_widget() makes its `!lv_obj_has_flag(list_bg, HIDDEN)` check false,
   so it SKIPS the slide-out branch entirely — which means its ready_cb
   (inst_list_slide_out_done_cb) never runs, and THAT callback is what restores
   the mouse app's own bottom bar (hid_mouse_set_own_bar_hidden) and resets
   s_bar_single_device/the base list, so skipping it left the mic icon stuck.
   The actual fix for the visible full-screen sweep this was chasing lives at
   the OPEN side instead: instruction_list_bar_tap_device's remote-focus branch
   now pins s_reveal_from_left = false alongside the +LV_HOR_RES park, so
   close_ai_widget()'s slide start/target match (list stays off-screen,
   ready_cb still fires normally) — this function can just be the same
   `s_close_is_cancel = true; close_ai_widget();` every other cancel path uses. */
void instruction_list_cancel_ai_widget(void)
{
    s_close_is_cancel = true;
    close_ai_widget();
}

/* Auto-dismiss the voice input box when the user navigates away from the
   instruction_list page (called by the page navigator in app_clock_status_bar).
   Mirrors the device page closing its skaibar on leave: the box absorbs a swipe
   that STARTS on it (ai_box_scroll_dismiss_cb), but an edge-back gesture or a
   page switch that doesn't touch the box would otherwise leave it open. Gated on
   the box's REAL visibility (not the is_open flag, which can get stuck). */
void instruction_list_close_ai_on_leave(void)
{
    if (!p_instruction_list_layout)
        return;
    /* Close on leave whenever the LIST is up (voice OR browse state) — not just
       when the box is open — so navigating away always tears the floating panel
       down (full close = slide it out + hide). */
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    bool list_shown = list_bg && lv_obj_is_valid(list_bg) &&
                      !lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN);
    if (list_shown)
        close_ai_widget();
}

/* Instant, animation-free collapse of the floating list to the closed state
   (parked off-screen + HIDDEN). close_ai_widget slides the list out and only sets
   HIDDEN in the slide's ready_cb (async), so a caller mid page-swipe would just
   see it ride out. This hides it NOW. Used on mouse-page entry: a list opened on
   the watch face (animate_open is the only show path) is never closed on a
   HOME->RIGHT switch — both keep the bar visible, so set_visible(false)'s close
   never runs — so without this it rides in with the page. */
void instruction_list_hide_now(void)
{
    if (!p_instruction_list_layout)
        return;
    lv_obj_t *list_bg = p_instruction_list_layout->p_instruction_list_bg;
    if (list_bg && lv_obj_is_valid(list_bg))
    {
        lv_anim_del(list_bg, inst_list_slide_anim_cb); /* kill any in-flight slide */
        lv_anim_del(list_bg, reveal_settle_anim_cb);
        lv_obj_set_style_translate_x(list_bg, LV_HOR_RES, 0); /* park off-screen right, as a finished slide-out leaves it */
        lv_obj_add_flag(list_bg, LV_OBJ_FLAG_HIDDEN);
    }
    is_open_instruction_list_ai = false;
    {
        extern void instruction_list_bar_set_blur(bool on);
        instruction_list_bar_set_blur(false);
    }
}


void tap_on_ai_widget(void)
{
    /* No Bluetooth gate (matches the right device_pager skaibar): the box opens
       on a single tap regardless of connection. The v2t start below is guarded
       to real hardware, so no phone just means no transcript — the box still
       opens. */
    // if (is_open_instruction_list_ai)
    // {
    //     extern void send_to_ai(void);
    //     send_to_ai();
    //     return;
    // }
    // lv_obj_clear_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
    // LV_OBJ_FLAG_HIDDEN); lv_anim_t a; lv_anim_init(&a); lv_anim_set_var(&a,
    // p_instruction_list_layout->p_instruction_list_ai_bg);
    // lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER); lv_anim_set_time(&a,
    // 300); lv_anim_set_exec_cb(&a, set_ai_bg_opa); lv_anim_set_path_cb(&a,
    // lv_anim_path_ease_in_out); lv_anim_start(&a);
    /* Phase 1: show voice indicator only, skai_widget stays transparent */
    skai_widget_shown = false;
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_30, 0);
    set_skai_widget_opa(0);
    if (ai_voice_btn && lv_obj_is_valid(ai_voice_btn))
    {
        lv_obj_clear_flag(ai_voice_btn, LV_OBJ_FLAG_HIDDEN);
    }
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
    {
        lv_obj_add_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
    if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
    {
        lv_obj_add_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
    }
    LOG_D("AI widget opened");
    is_open_instruction_list_ai = true;
    open_skai_widget_ai(true);
    // animate_to_ai_page();
    set_skai_widget_input_text("");
    /* Voice pipeline — REAL HARDWARE ONLY. The PC sim has no mic/BLE/voice
       infra; running this there (mic open, speech-indicator hint, v2t) sets up
       voice state the close path then chokes on and crashes the sim. Guarding
       the whole block lets the box still open on sim (is_open=true) without the
       voice machinery. On hardware it runs regardless of BT connection — no
       phone just means no transcript (the box still opens, matching the right
       device_pager skaibar, which also has no BT gate).
       Founder direction 2026-05-19: route spoken input to the skaibar pipeline
       — voice_set_pending_v2t_intent(V2T_INTENT_SKAIBAR) before start_v2t()
       makes the phone treat the transcript as a skaibar command, not a chat
       query (auto-resets to CHAT after the START handler). */
#ifndef BSP_USING_PC_SIMULATOR
    set_ai_open_mic(true);
    show_speech_indicator(true);
    voice_set_pending_v2t_intent(V2T_INTENT_SKAIBAR);
    voice_provider.start_v2t();
#endif
    /* Arm SKAIBAR option-tracking — scrolls will start reporting idx
       to the phone via commu_send_skaibar_selected. Survives the
       widget close (pill fade), cleared on instruction_list_pause. */
    s_skaibar_tracking_active = true;
    // set_free_control_with_arm(false);
    set_paused_control_with_arm(true);
}

static bool instruction_list_ai_tapped = false;
bool get_instruction_list_ai_tapped(void)
{
    return instruction_list_ai_tapped;
}
void set_instruction_list_ai_tapped(void)
{
    instruction_list_ai_tapped = false;
}
void tap_on_ai_hint(void)
{
    if (!get_bluetooth_connection_status())
    {
        create_connection_tips();
        LOG_D("Bluetooth is connected, ignoring voice recognition event");
        return;
    }
    if (isTextEmpty())
    {
        LOG_D("tap_on_ai_hint: empty, skip send");
        return;
    }
    instruction_list_ai_tapped = true;
    extern void send_to_ai(void);
    extern void set_skai_widget_awaiting_ai(void);
    send_to_ai();
    /* Immediate visual feedback: show "AI處理中..." placeholder inside the
       widget. The first streamed AI chunk replaces it. */
    set_skai_widget_awaiting_ai();
    LOG_D("tap_on_ai_hint: send_to_ai fired");
}

/* Called when the first AI reply chunk arrives: hide the "sending" sand icon
   so the mic button indicates the widget is ready for re-ask. */
void on_ai_reply_started(void)
{
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon) &&
        !lv_obj_has_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
}

void show_send_icon(void)
{
    if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
    {
        lv_obj_clear_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
}

/*******************************************************************************
 * Send instruction update back to phone
 ******************************************************************************/
static void send_instruction_update(list_item_t *item)
{
    if (!item->is_instruction)
        return;

    cJSON *root = cJSON_CreateObject();
    if (!root)
        return;

    cJSON_AddStringToObject(root, "id", item->id);
    cJSON_AddStringToObject(root, "title", item->title);
    cJSON_AddNumberToObject(root, "version", item->version);

    cJSON *trigger = cJSON_CreateObject();
    cJSON_AddStringToObject(trigger, "type", item->trigger_type);
    if (item->is_interval)
        cJSON_AddNumberToObject(trigger, "intervalSeconds", item->interval_sec);
    cJSON_AddItemToObject(root, "trigger", trigger);

    if (item->is_interval)
        cJSON_AddBoolToObject(root, "enabled", item->enabled);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str)
    {
        LOG_I("Send instruction update: %s", json_str);
        commu_send_update_instruction(json_str);
        cJSON_free(json_str);
    }
}

void request_instruction_image(const char *id)
{
    if (!id || id[0] == '\0')
        return;

    extern void set_pending_instruction_img_id(const char *id);
    set_pending_instruction_img_id(id);

    commu_send_get_instruction_img(id);
    LOG_I("Requested instruction image for id=%s", id);
}

extern void media_widget_tap_event_cb(void);
static void on_item_tap(list_item_t *item)
{
    LOG_D("on_item_tap: %s", item->id);
    if (item->is_instruction)
    {
        /* Instruction items are handled in on_tap, not here */
        return;
    }
    if (is_open_instruction_list_ai)
    {
        if (!isTextEmpty())
            tap_on_ai_hint();
        else
            LOG_D("AI input is empty, ignoring tap");
    }
    else if (!is_open_instruction_list_ai)
    {
        animate_to_home_from_instruction_list();
        gui_app_run(item->id);
    }
}
static void flash_restore_cb(lv_timer_t *timer)
{
    lv_obj_t *label = (lv_obj_t *)timer->user_data;
    if (label != NULL && lv_obj_is_valid(label))
    {
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    }
}

static void update_custom_switch_visual(uint8_t idx)
{
    lv_obj_t *sw_bg = switch_objs[idx];
    if (sw_bg == NULL || !lv_obj_is_valid(sw_bg))
        return;
    lv_obj_t *knob = lv_obj_get_child(sw_bg, 0);
    if (knob == NULL || !lv_obj_is_valid(knob))
        return;
    if (list_items[idx].enabled)
    {
        lv_obj_set_style_bg_color(sw_bg, lv_color_hex(0x00CCFF), 0);
        lv_obj_align(knob, LV_ALIGN_RIGHT_MID, -1, 0);
    }
    else
    {
        lv_obj_set_style_bg_color(sw_bg, lv_color_hex(0x444444), 0);
        lv_obj_align(knob, LV_ALIGN_LEFT_MID, 1, 0);
    }
}

static void flash_instruction_label(lv_obj_t *label)
{
    if (label == NULL || !lv_obj_is_valid(label))
        return;
    /* 先亮起（高亮色） */
    lv_obj_set_style_text_color(label, lv_color_hex(0x00CCFF), 0);
    /* 300ms 後恢復白色 */
    lv_timer_t *t = lv_timer_create(flash_restore_cb, 300, (void *)label);
    lv_timer_set_repeat_count(t, 1);
}

/* 執行一個 list item(點擊本體)。從浮層的 click cb 與 ADR-0020 左頁的
   instruction_list_activate_index 兩條路進來,行為完全一致。 */
/* R65(founder:「點了 ANDREW 的 NEW SESSION 卻進到 123++」;log 實證同一下點擊產生**兩次**
   啟動:先是正確的 `newsession:3ad19ddc`,3.6 秒後又一次 `conv:hermes:…123++`):開新對話會
   收輸入框、清搜尋字,清單因此重建 —— 重建後停在同一個位置的是**別的項目**,而那顆還在
   路上的點擊就落到它身上。開完新對話後短暫吃掉後續啟動,讓那顆遲來的點擊無處可去。 */
static rt_tick_t s_activate_suppress_until = 0;

/* R70:開新對話時要「當場」進聊天室,但**不能**在點擊事件處理中途動 UI 樹(清單正被收掉)。
   把標題存下來,用 lv_async_call 延到這一輪處理結束後的下一輪再建浮層。 */
static char s_pending_chat_title[64] = {0};

static void open_pending_chat_async_cb(void *unused)
{
    (void)unused;
    if (s_pending_chat_title[0] == '\0')
        return;
    extern void chat_page_open(const char *title, const char *icon_src);
    extern void chat_page_seed_local_message(const char *text);
    extern bool chat_page_is_open(void);
    /* R71(founder:「新開的 SESSION 聊天室不能從左邊緣往右滑退出」)真兇不在這裡:
       chat_page_open 本來就會武裝左緣偵測器,但點「開新對話」時清單已收掉、ATINST=false,
       輪詢的 check_is_at_home 一翻回 true 就 display_gesture_detect_objs(0,false) 把偵測器
       再藏掉。修在 app_mainmenu.c:_at_home 的條件加 !chat_page_is_open()(聊天開著就不是
       「在錶盤」),偵測器因此保持武裝。先前「重開清單保住 ATINST」的招沒效
       (instruction_list_open_browse 不會讓清單真的 visible),已移除。 */
    extern void chat_page_set_style_hermes(bool hermes);
    chat_page_set_style_hermes(true); /* 開新對話一律 Hermes 房 */
    if (!chat_page_is_open())
        chat_page_open(s_pending_chat_title, NULL);
    chat_page_seed_local_message(s_pending_chat_title); /* 自己那句先顯示,別讓房間空著 */
    s_pending_chat_title[0] = '\0';
}

void instruction_list_open_pending_chat(const char *title)
{
    if (title == NULL || title[0] == '\0')
        return;
    strncpy(s_pending_chat_title, title, sizeof(s_pending_chat_title) - 1);
    s_pending_chat_title[sizeof(s_pending_chat_title) - 1] = '\0';
    lv_async_call(open_pending_chat_async_cb, NULL);
}

/* ── 滑鼠單設備抽屜:tap 落在 session 列 → 手錶自己走進聊天室(2026-08-15 founder:
   「點session手錶也需要開那個聊天室」)。抽屜的 0x03 鏡像列只有 title,用那台的 0x20
   清單按 title 反查 conv id;查得到=session 列,查不到=一般選項照走 commit 給桌面。
   聊天室 overlay 的建立走 lv_async_call —— R70 鐵律:清單自己的點擊處理中途不准建
   layer_top 浮層(close_ai_widget 正在收清單,GUI 會當在半拆半建的樹上)。 */
static char s_sd_chat_title[96];
static char s_sd_chat_id[96];

static void sd_open_chat_async_cb(void *unused)
{
    (void)unused;
    extern void chat_page_set_style_hermes(bool hermes);
    extern void chat_page_open(const char *title, const char *icon_src);
    /* 聊天 overlay 不透明蓋全螢幕:先把錶盤模糊圖硬藏,別讓它的 opa 動畫在低 heap 時
       把 EPIC render 壓爆(sys memory is full,2026-08-15 真機)。 */
    extern void clock_main_blur_force_hide(void);
    clock_main_blur_force_hide();
    chat_page_set_style_hermes(strncmp(s_sd_chat_id, "conv:", 5) == 0);
    chat_page_open(s_sd_chat_title, NULL);
}

static bool single_device_try_open_session(const list_item_t *item)
{
    if (!s_bar_single_device || item == NULL)
        return false;
    extern const char *session_list_find_conv_id(const char *device_id, const char *title);
    const char *conv_id = session_list_find_conv_id(s_single_device_id, item->title);
    if (conv_id == NULL)
        return false;
    strncpy(s_sd_chat_title, item->title, sizeof(s_sd_chat_title) - 1);
    s_sd_chat_title[sizeof(s_sd_chat_title) - 1] = '\0';
    strncpy(s_sd_chat_id, conv_id, sizeof(s_sd_chat_id) - 1);
    s_sd_chat_id[sizeof(s_sd_chat_id) - 1] = '\0';
    LOG_W("[act] single-device session \"%s\" -> chat (%s)", s_sd_chat_title, s_sd_chat_id);
    extern bool commu_send_conv_open(const char *title, const char *id, uint8_t index);
    commu_send_conv_open(s_sd_chat_title, s_sd_chat_id, 0);
    /* 先收抽屜回觸控板:聊天室 overlay 疊在觸控板上,左緣右滑關聊天室後落回觸控板
       (founder:「從左邊邊緣往右滑動回到觸碰板」)。非 cancel:滑出 ready_cb 會
       restore_base + 清 s_bar_single_device;不送 0x0C(同 commit 路徑,避免跟下一次
       點 bar 的 summon 打架)。 */
    s_close_is_cancel = false;
    close_ai_widget();
    lv_async_call(sd_open_chat_async_cb, NULL);
    return true;
}

static void list_item_activate(list_item_t *item)
{
    if (s_activate_suppress_until != 0 &&
        (int32_t)(s_activate_suppress_until - rt_tick_get()) > 0) /* 有號差:tick 回卷也成立 */
    {
        LOG_W("[act] suppressed \"%s\" (just opened a new session)", item->title);
        return;
    }
    s_activate_suppress_until = 0;
    LOG_W("[act] id=\"%s\" title=\"%s\"", item->id, item->title);
    /* 滑鼠單設備抽屜的 session 列:手錶自己進聊天室,不交給桌面執行。 */
    if (single_device_try_open_session(item))
        return;
    /* R55:合成的「開新 session」列 —— 篩不到任何既有項目時才存在。點它就用目前控制中
       的那台電腦開新 session(清單推回來後 session pager 自己走進聊天室),然後收掉輸入框
       與文字篩選,讓清單回到完整狀態。 */
    if (strncmp(item->id, NEW_SESSION_ITEM_ID, sizeof(NEW_SESSION_ITEM_ID) - 1) == 0)
    {
        /* R61:id = NEW_SESSION_ITEM_ID[":"<device_id>] —— 有帶設備就開在那台(使用者
           在畫面上挑的那一列),沒帶就交給 pager 的預設目標。 */
        const char *dev = item->id + (sizeof(NEW_SESSION_ITEM_ID) - 1);
        if (*dev == ':')
            dev++;
        else
            dev = NULL;
        extern void session_list_open_new_with_prompt(const char *device_id,
                                                      const char *prompt);
        extern void instruction_list_set_text_filter(const char *text);
        /* R58:講的那句話沒中任何既有項目 → 它就是新對話的第一句,一起帶過去。 */
        session_list_open_new_with_prompt(dev, s_text_filter);
        /* R70(founder:「連聊天室都沒進去,電腦也沒開新 SESSION」;log:`fatal error on thread:
           app_watc` PC=0x2000072c):R68/R69 在這裡**當場**開聊天室浮層並塞入第一則訊息,好蓋掉
           桌面建立那 4 秒的錶盤空窗。功能是對的,**時機**是錯的 —— 那是在清單自己的點擊處理
           中途去建 layer_top 浮層,而同一時間清單正要被收掉(close_ai_widget 會把它帶走),
           GUI 執行緒就當在半拆半建的物件樹上。改成把同一件事丟給 lv_async_call:等這一輪
           事件處理**完全結束**、清單也收乾淨了,下一輪再開房間。 */
        {
            extern void instruction_list_open_pending_chat(const char *title);
            instruction_list_open_pending_chat(s_text_filter);
        }
        /* R57:只收輸入框,**清單保持開著** —— session 是非同步建的(手機轉給桌面、桌面回
           推清單才有那一列),清單一關就沒人帶使用者走進聊天室了。清單會在 chat_page 開起來
           時自然被蓋掉。 */
        if (is_open_instruction_list_ai)
            close_ai_widget();
        instruction_list_set_text_filter(NULL);
        /* R65:接下來 1 秒內的啟動一律吃掉 —— 清單正在重建,遲到的那顆點擊會落在別的項目上。 */
        s_activate_suppress_until = rt_tick_get() + rt_tick_from_millisecond(1000);
        return;
    }
    /* 滑鼠抽屜:點下電腦鏡像過來的 '@' 類選項(AskSkai / 開新對話那種)時,先武裝 walk-in。
       執行本身照舊交給電腦(下面的 0x06 commit)—— session 是電腦建的,手錶再送一次
       conv_new 會變成建兩個。武裝之後,那台電腦把新 session 推回清單時,session pager 既有的
       walk-in 就會把使用者帶進聊天室(founder 2026-08-17:「電腦端有進聊天室但手錶上沒有」)。
       為什麼非得這樣繞:KEY_CONV_OPEN(0x0F)是 uplink-only,電腦端沒有任何管道能主動叫
       手錶開聊天室。
       category 由 0x03 鏡像帶下來('@'=chat 類 / '/'=一般 action),既有 session 列在上面
       single_device_try_open_session() 就已經接走了,所以這裡的 '@' 實際上就是「會開出一個
       新對話」的那幾種。判斷錯也安全:沒有新 session 出現時這面旗自然過期。 */
    /* 2026-08-17 真機修正:原本用 `item->category == '@'` 收窄,但 log 顯示「問 SKAI」那列
       在手錶上的 category **不是** '@'(`[act]` 有印、`[walkin] armed` 沒印),武裝從沒發生。
       改成「抽屜裡只要不是既有 session 的選項就武裝」—— 既有 session 在上面
       single_device_try_open_session() 已經接走,剩下的本來就以「會開出新對話」為大宗。
       判斷錯的代價很小:沒有新 session 出現時這面旗 30 秒後自然過期(房間還開著則 3 分鐘)。
       順便印出實際的 category,之後要把條件收窄回去就有依據。 */
    if (s_bar_single_device)
    {
        extern void session_list_arm_walkin(const char *device_id);
        LOG_W("[walkin] drawer commit cat=%d -> arm", (int)item->category);
        session_list_arm_walkin(s_single_device_id);
    }

    /* A tapped @-contact opens the in-watch chat room (mirror the desktop @-contact
       tap). The merged mixed list has no separate @ view, so this keys off the tapped
       item's OWN category, not the page filter — @ rows open the conversation, '/' and
       untagged rows fall through to the action/app split below. Guarded off while the AI
       input widget is up. The phone resolves the route from {index,title,id} and streams
       the chat state back via KEY_CONV_STATE. */
    if (item->category == '@' && !is_open_instruction_list_ai)
    {
        int conv_idx = (int)(item - &list_items[0]);
        if (conv_idx < 0 || conv_idx >= MAX_LIST_ITEMS)
            conv_idx = 0;
        commu_send_conv_open(item->title, item->id, (uint8_t)conv_idx);
        /* conv: 前綴 = Hermes session(桌面 IsHermes 同款分流);其他 @ 列是
           WhatsApp/Messenger 等 messaging 房,保留氣泡畫風。 */
        {
            extern void chat_page_set_style_hermes(bool hermes);
            chat_page_set_style_hermes(strncmp(item->id, "conv:", 5) == 0);
        }
        chat_page_open(item->title, item->icon);
        return;
    }

    if (item->is_instruction)
    {
        LOG_I("Custom instruction tapped: id=%s, title=%s", item->id,
              item->title);
        /* Offline "open a watch app" instruction: the phone marked this
           item with an openApp target, so run it entirely on the watch —
           no SKAIBAR commit, no send_instruction_update, no phone relay.
           This is the one instruction kind that works with the phone
           disconnected. gui_app_run returns non-RT_EOK for an unknown app
           name (e.g. a phone-only app), so we guard and stay put. */
        if (item->open_app[0] != '\0')
        {
            LOG_I("[left] local open app (offline): %s", item->open_app);
            if (is_open_instruction_list_ai)
                close_ai_widget();
            rt_err_t r = gui_app_run(item->open_app);
            if (r != RT_EOK)
                LOG_E("[left] gui_app_run('%s') failed (%d) — staying",
                      item->open_app, (int)r);
            else
                animate_to_home_from_instruction_list();
            return;
        }
        /* SKAIBAR commit — tap on a custom instruction while the
           tracking session is alive (mic was opened at some point on
           this page, page not yet paused). Send the option idx the
           phone uses as its action trigger. Independent of the
           AI-widget visibility, the existing send_instruction_update
           flow below, and the toggle/flash visuals — phone may use
           the COMMITTED notify alongside, e.g. to invoke a script. */
        for (uint8_t j = 0; j < list_item_count; j++)
        {
            if (&list_items[j] == item && j >= app_base_count)
            {
                /* SKAI_LINK commit to the primary (same as right device_pager). */
                instruction_list_assert_local_target();
                commu_send_option_commit((uint8_t)(j - app_base_count));
                LOG_D("[left] commit option idx=%u (raw=%u)",
                      (unsigned)(j - app_base_count), (unsigned)j);
                break;
            }
        }
        /* 滑鼠 app 單設備:選項已送去那台電腦執行(電腦 skaibar 執行完自己關) → 只把手錶浮層清單
           收回去(滑出+隱藏)、回到滑鼠 trackpad。**不送 0x0C dismiss**:否則 dismiss 會跟你「第二
           次點 bar」的 summon 打架,害電腦 skaibar 不再開。保持單設備模式,第二次點 bar 直接重新
           summon。close_ai_widget(非 cancel)的滑出 ready_cb 會 restore_base + 清 s_bar_single_device。 */
        if (s_bar_single_device)
        {
            s_close_is_cancel = false;
            close_ai_widget();
            return;
        }
        /* Input box (AI widget) open: a tap on a concrete instruction option
           should RUN that option and CLOSE the box (founder direction
           2026-05-29) — NOT redirect to "ask AI". The old early return here ran
           tap_on_ai_hint() and returned BEFORE send_instruction_update (the
           id-based commit the phone actually executes via onWatchCommitted), so
           options were dead while the box was open. Close the box, then fall
           through to the normal run path below. (commu_send_option_commit above
           already fired; the run below is what reaches the phone's executor.) */
        if (is_open_instruction_list_ai)
        {
            close_ai_widget();
        }
        /* Find index in list_items */
        for (uint8_t j = 0; j < list_item_count; j++)
        {
            if (&list_items[j] == item)
            {
                if (item->is_interval)
                {
                    item->enabled = !item->enabled;
                    update_custom_switch_visual(j);
                }
                flash_instruction_label(app_label[j]);
                break;
            }
        }
        send_instruction_update(item);
        if (instruction_tap_cb)
        {
            instruction_tap_cb(item->id, item->enabled);
        }
    }
    else
    {
        on_item_tap(item);
    }
}

static void list_item_click_event_cb(lv_event_t *evt)
{
    list_item_t *item = (list_item_t *)evt->user_data;
    /* A horizontal swipe (the left-to-close flick, or a right flick) also lands a
       CLICKED on the item: the list scrolls vertically only, so a horizontal drag
       never "loses" the press and LVGL fires CLICKED on release. The list's
       GESTURE handler latches s_list_horiz_swipe for exactly this — ignore the
       click so swipe-right-to-close doesn't ALSO select the item. (A flag, not
       lv_indev_get_gesture_dir, because gesture_dir lingers after the swipe and
       would then wrongly suppress the NEXT genuine tap.) */
    if (s_list_horiz_swipe)
        return;
    LOG_D("ID: %s", item->id);
    list_item_activate(item);
}

/** ADR-0020:左頁 actions 區段的點擊入口。 */
void instruction_list_activate_index(uint8_t i)
{
    LOG_W("[act] index=%u cnt=%u", (unsigned)i, (unsigned)list_item_count);
    if (i >= list_item_count)
        return;
    list_item_activate(&list_items[i]);
}

static void on_tap(void)
{
    LOG_W("[act] on_tap sel=%u cnt=%u", (unsigned)selected_item_index,
          (unsigned)list_item_count);
    if (selected_item_index >= list_item_count)
        return;

    list_item_t *item = &list_items[selected_item_index];

    /* A tapped @-contact opens the in-watch chat room — the raise-wrist gesture path,
       mirror of list_item_click_event_cb. Keys off the item's own category (the merged
       mixed list has no separate @ view). */
    if (item->category == '@' && !is_open_instruction_list_ai)
    {
        commu_send_conv_open(item->title, item->id, (uint8_t)selected_item_index);
        {
            extern void chat_page_set_style_hermes(bool hermes);
            chat_page_set_style_hermes(strncmp(item->id, "conv:", 5) == 0);
        }
        chat_page_open(item->title, item->icon);
        return;
    }

    if (item->is_instruction)
    {
        LOG_I("Custom instruction tapped via gesture: id=%s", item->id);
        /* 滑鼠單設備抽屜的 session 列(手勢路徑,與 list_item_activate 同攔截):
           手錶自己進聊天室,不交給桌面執行。 */
        if (single_device_try_open_session(item))
            return;
        /* Offline "open a watch app" instruction — run locally on the watch
           with no phone relay, same as list_item_click_event_cb's open_app
           branch. This gesture/ENTER path is the one users hit most (raise-
           wrist tap), so it must honour openApp too or the offline-open
           feature only works via touch. gui_app_run guards unknown names. */
        if (item->open_app[0] != '\0')
        {
            LOG_I("[left] local open app via gesture (offline): %s",
                  item->open_app);
            if (is_open_instruction_list_ai)
                close_ai_widget();
            rt_err_t r = gui_app_run(item->open_app);
            if (r != RT_EOK)
                LOG_E("[left] gui_app_run('%s') failed (%d) — staying",
                      item->open_app, (int)r);
            else
                animate_to_home_from_instruction_list();
            return;
        }
        /* SKAIBAR commit — same rationale as list_item_click_event_cb,
           parallel gesture path. selected_item_index is the raw
           list_items index; subtract app_base_count for the 0-based
           skaibar-option index the phone expects. */
        if (selected_item_index >= app_base_count)
        {
            /* SKAI_LINK commit to the primary (gesture path). */
            instruction_list_assert_local_target();
            commu_send_option_commit(
                (uint8_t)(selected_item_index - app_base_count));
            LOG_D("[left] commit option via gesture idx=%u (raw=%u)",
                  (unsigned)(selected_item_index - app_base_count),
                  (unsigned)selected_item_index);
        }
        /* 滑鼠 app 單設備:選項已送去那台電腦執行 → 收回手錶浮層清單(同 list_item_click_event_cb,
           不送 0x0C,避免跟第二次 summon 打架)。 */
        if (s_bar_single_device)
        {
            s_close_is_cancel = false;
            close_ai_widget();
            return;
        }
        if (is_open_instruction_list_ai)
        {
            if (!isTextEmpty())
                tap_on_ai_hint();
            return;
        }
        if (item->is_interval)
        {
            item->enabled = !item->enabled;
            update_custom_switch_visual(selected_item_index);
        }
        flash_instruction_label(app_label[selected_item_index]);
        send_instruction_update(item);
        if (instruction_tap_cb)
        {
            instruction_tap_cb(item->id, item->enabled);
        }
    }
    else
    {
        on_item_tap(item);
    }
}


extern char *get_media_title(void);
extern bool is_have_message_now(void);
static uint16_t gesture_starting_value = 0;

/* 精確置中(定義在後段,R49 的落點要用它 —— lv_obj_scroll_to_view 對這個重疊清單不準)。 */
static void scroll_center_item(lv_obj_t *list, uint16_t target);

static void reset_list_internal(void)
{
    if (p_instruction_list_layout->list == NULL)
    {
        return;
    }
    /* R32(2026-08-15):列 UI 已釋放(滑鼠模式抽屜收合後)—— list 本體還在但列物件/dot
       句柄全 NULL,底下的 scroll_center_item/update dots/open_selected_widget 是懸空
       操作。ensure 重建時 refresh 會重跑落點,這裡直接跳過。 */
    {
        extern bool instruction_list_ui_is_released(void);
        if (instruction_list_ui_is_released())
        {
            LOG_W("[land] reset skipped (list UI released)");
            return;
        }
    }
    open_scroll_motor = false;
    disable_scrolling_motor_vibrate();
    set_paused_control_with_arm(false);
    scroll_initialized = false;
    uint8_t scroll_to_index;
    {
        /* R49(founder 2026-08-13 定案):**進場落點 = 最新的那個 session**,也就是 conv 段
           最下面、緊鄰 actions 上方的那一項(conv 段由舊到新、最新在最下面)。只有完全沒有
           session 時才退回舊行為 —— 停在 actions 最下面。
           (R3 原本一律停在整份清單的最後一項,也就是 actions 最下面;有 session 之後那個
           落點對使用者沒有意義 —— 進來第一眼要看到的是最新的對話。) */
        uint8_t newest_conv = (uint8_t)-1;
        for (uint8_t i = 0; i < list_item_count; i++)
            if (strncmp(list_items[i].id, "conv:", 5) == 0)
                newest_conv = i; /* 不 break:要的是**最後一個** conv 項 */
        LOG_W("[land] reset count=%u conv=%d pending=%d",
              (unsigned)list_item_count, (int)(int8_t)newest_conv,
              (int)s_entry_landing_pending);
        if (newest_conv != (uint8_t)-1)
        {
            app_scroll_target_item = newest_conv;
            /* R52(founder:「文字是對的,但右邊的 icon 後面才跳」):右側指示圈的位置是用
               refresh 尾端那條公式從 selected_item_index 反推的 ——
                   input = 100*cnt - 63 - 100*selected
               舊有的固定值 37 只是這條公式在 selected == cnt-1(最後一項)時的解。落點改成
               「最新的 session」之後再沿用 37,等於把圈畫在最後一項上,直到下一次 refresh
               用正確公式重算才跳過去。這裡直接套同一條公式,進場第一幀就一致。 */
            gesture_starting_value =
                (uint16_t)(100 * (int)list_item_count - 63 - 100 * (int)newest_conv);
            selected_item_index = app_scroll_target_item;
            prev_app_scroll_target_item = app_scroll_target_item;
            scroll_center_item(p_instruction_list_layout->list, newest_conv);
            if (!scroll_initialized)
                scroll_list(p_instruction_list_layout->list, 0);
            /* scroll_list 會從幾何反推 selected,落點不完全精準時會差一格 —— 重新釘回。 */
            selected_item_index = newest_conv;
            app_scroll_target_item = newest_conv;
            /* R53(founder:「已經在 session 頁就算沒滾動過,有更新也不要跳,維持現狀」):
               落點已經落在 session 上 → 進場定位的任務就結束,之後手機推播走的是
               R48 的「還原原本那一項」,位置不動。
               沒有 conv 項的那條 fallback 不清旗標:那代表 session 還沒到,等它到達時
               仍要補正一次(founder 的規則是「沒有 session 才停在 actions 最下面」)。
               R80:落在 conv 上也**不再立刻清** —— 另一台桌面的清單可能晚幾秒才到、
               全域最新在那包裡。寬限窗(refresh 的 R50 分支)負責到期/手勢時熄掉。 */
            update_indicator_dots_position(gesture_starting_value);
            open_selected_widget(false);
            is_widget_animation_active = false;
            enable_scrolling_motor_vibrate();
            open_scroll_motor = true;
            return;
        }
        /* 滾到列表最下面那個項目。list_item_count is uint8_t — with the empty-list
           placeholder (load_default_apps() can now leave list_item_count==0, unlike
           before when the built-in app prefix guaranteed count>0), `count - 1`
           wraps to 255. lv_obj_get_child(list, 255) then returns NULL (out of
           range), and the unconditional lv_obj_scroll_to_view(NULL, ...) below
           faulted on it — unlike the two guarded call sites elsewhere in this file
           that check `if (child && lv_obj_is_valid(child))` first. */
        scroll_to_index = list_item_count > 0 ? list_item_count - 1 : 0;
        app_scroll_target_item = scroll_to_index;
    }
    gesture_starting_value = 37;
    selected_item_index = app_scroll_target_item;
    prev_app_scroll_target_item = app_scroll_target_item;
    lv_obj_t *child = list_item_count > 0
        ? lv_obj_get_child(p_instruction_list_layout->list, scroll_to_index)
        : NULL;
    if (child && lv_obj_is_valid(child))
        lv_obj_scroll_to_view(child, LV_ANIM_OFF);
    if (!scroll_initialized)
    {
        scroll_list(p_instruction_list_layout->list, 0);
    }
    update_indicator_dots_position(gesture_starting_value);
    open_selected_widget(false);
    is_widget_animation_active = false;
    enable_scrolling_motor_vibrate();
    open_scroll_motor = true;
}

/* Thread-aware wrapper assigned to myLancher[...].reset_list. Callers in
   bloc_notification (KE_EVT2) hit reset_list_internal's lv_obj_scroll_to_view
   which cascades through scroll events and blows the 4KB BLE stack. Defer to
   the LVGL thread when invoked off-thread. */
static void reset_list(void)
{
    if (!is_on_lvgl_thread())
    {
        lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_RESET_INSTRUCTION_LIST};
        lvgl_send_msg(msg);
        return;
    }
    reset_list_internal();
}

/* Public entry called by the LVGL_MSG_TYPE_RESET_INSTRUCTION_LIST handler.
   Always invoked from the LVGL thread, so no thread check needed. */
void apply_instruction_list_reset_on_lvgl_thread(void)
{
    reset_list_internal();
}

uint16_t get_gesture_starting_value(void)
{
    return gesture_starting_value;
}

extern void media_widget_trigger_drag_by_py(int p_y);

#ifdef USE_QUICK_OPEN_AI


static rt_timer_t timer_open_quick_app = RT_NULL;


static bool open_vibration = false;
#endif

// 設置指示點位置的公共函數
// input_value: 0-1000 的輸入值
// 100 時第一個點在最右邊中心（0度），900 時最後一個點在最右邊中心（0度）
// 所有點分佈在80度範圍內（-40度到+40度）
void set_instruction_list_indicator_dots_position(int input_value)
{
    update_indicator_dots_position(input_value);
}

#ifdef TEST_INDICATOR_ANIMATION
// 動畫測試相關的變數
static lv_anim_t indicator_test_anim;
static bool indicator_test_running = false;

// 動畫執行回調函數 - 符合LVGL要求的簽名
static int prev_value = 0;
static void indicator_test_anim_exec_cb(void *var, int32_t value)
{
    if (abs(value - prev_value) < 10)
    {
        return; // 如果變化不大，則忽略
    }
    prev_value = value;
    set_instruction_list_indicator_dots_position((int)value);
    LOG_D("Animation test: input_value = %d", (int)value);
}

// 開始動畫測試
void start_indicator_dots_animation_test(void)
{
    if (p_instruction_list_layout == NULL || !created)
    {
        LOG_D("App list layout not created yet, cannot start animation test");
        return;
    }

    if (indicator_test_running)
    {
        LOG_D("Animation test already running");
        return;
    }

    LOG_D("Starting indicator dots animation test (0->1000->0 loop)");
    indicator_test_running = true;

    // 初始化動畫
    lv_anim_init(&indicator_test_anim);
    lv_anim_set_var(&indicator_test_anim, NULL);
    lv_anim_set_exec_cb(&indicator_test_anim, indicator_test_anim_exec_cb);
    lv_anim_set_playback_time(&indicator_test_anim, 5000); // 3秒從0到1000
    lv_anim_set_time(&indicator_test_anim, 5000);          // 3秒從0到1000
    lv_anim_set_values(&indicator_test_anim, 0, 1250);
    lv_anim_set_path_cb(&indicator_test_anim, lv_anim_path_ease_in_out);
    lv_anim_set_repeat_count(&indicator_test_anim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&indicator_test_anim);
}

#endif

static bool send_to_ai_again = false;
bool get_send_to_ai_again(void)
{
    return send_to_ai_again;
}
void set_send_to_ai_again(bool value)
{
    send_to_ai_again = value;
}
extern void set_ai_open_mic(bool is_open);
extern bool skai_widget_has_ai_reply(void);
extern void clear_skai_widget_ai_reply(void);
extern bool get_voice_recognition_started(void);
extern void clearVoice2Text(void);


static void ai_bar_event_cb(lv_event_t *evt)
{
    if (evt->code == LV_EVENT_PRESSED)
    {
        if (!get_bluetooth_connection_status())
        {
            create_connection_tips();
            LOG_D("Bluetooth is connected, ignoring voice recognition event");
            return;
        }
        lv_obj_clear_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                          LV_OBJ_FLAG_HIDDEN);
        /* Pre-set Phase 2 visual state so the skai_widget and gaus_bg are
           already opaque as the user drags ai_page in. Skipped if AI is
           already open (e.g. wrist-raise Phase 1 is active with mic only). */
        if (!is_open_instruction_list_ai)
        {
            if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
            {
                lv_obj_clear_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
            }
            if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
            {
                lv_obj_clear_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
            }
            set_skai_widget_opa(LV_OPA_COVER);
        }
    }
}


/*******************************************************************************
 * Custom Instruction API
 ******************************************************************************/

/* R60(founder:「往上滑到別的選項會瞬間跳回最下面那個 NEW SESSION」+「點了還是沒反應」):
   語音搜尋開著的時候,**清單凍結**。手機每 5 秒 replace-all 一次,我在推播之後重篩 ——
   於是每 5 秒把選中項與捲動位置重設一次(你的上滑被彈回去),而重建被 500ms 防抖延後的
   那段空窗裡,陣列與畫面對不起來(你點到的是別人)。與其在每條推播路徑後面補救,不如讓
   搜尋期間的清單就是一份靜態快照:推播照收(佇列留著),只是先不套用。搜尋一結束,
   cat_filter_restore_full 把搜尋前的完整清單放回來,下一次推播(最多 5 秒)就補上最新內容。 */
static bool search_freeze_active(void)
{
    return s_text_filter[0] != '\0';
}

void clear_custom_instructions(void)
{
    if (search_freeze_active())
        return;
    /* BATCH replace (0x6B) clears here before re-appending. A clear is always a
       phone push, so enter PHONE mode and treat the link as connected; any stale
       disconnect-filter snapshot is now superseded by the incoming list. */
    list_item_count = app_base_count;
    s_list_mode = LIST_MODE_PHONE;
    s_phone_connected = true;
    if (s_disc_backup)
    {
        rt_free(s_disc_backup);
        s_disc_backup = NULL;
    }
}

/* 根據 id 找到已有的指令，回傳 index，找不到回傳 -1 */
static int find_instruction_by_id(const char *id)
{
    for (uint8_t i = app_base_count; i < list_item_count; i++)
    {
        if (strcmp(list_items[i].id, id) == 0)
            return i;
    }
    return -1;
}

void remove_custom_instruction(const char *id)
{
    int idx = find_instruction_by_id(id);
    if (idx < 0)
    {
        LOG_W("remove_custom_instruction: id=%s not found", id);
        return;
    }

    /* Delete image file if exists */
    if (list_items[idx].img_path[0] != '\0')
    {
        remove(list_items[idx].img_path);
        LOG_I("Deleted instruction image: %s", list_items[idx].img_path);
    }

    /* Shift remaining items left */
    for (uint8_t i = idx; i < list_item_count - 1; i++)
    {
        memcpy(&list_items[i], &list_items[i + 1], sizeof(list_item_t));
    }
    list_item_count--;

    LOG_I("Removed instruction: id=%s", id);
}

/* openApp name (an APP_ID_* string the phone sends) → app_id enum, so an openApp
   instruction can borrow the matching built-in app icon via get_app_icon() and
   render like a real app. Returns 0xFF for names with no local app — get_app_icon
   then yields the generic IMG_LOGO, which is also the correct icon for camera
   (its own BUILTIN_APP_EXPORT icon IS IMG_LOGO). flashlight / exercise are always
   defined (mirrors get_app_id_str); the rest are #ifdef-gated by board config. */
static uint8_t app_id_from_name(const char *name)
{
    if (!name || !name[0])
        return 0xFF;
    if (strcmp(name, APP_ID_FLASHLIGHT) == 0)
        return app_id_flashlight;
    if (strcmp(name, APP_ID_EXERCISE) == 0)
        return app_id_exercise;
#ifdef APP_ID_TIMER
    if (strcmp(name, APP_ID_TIMER) == 0)
        return app_id_timer;
#endif
#ifdef APP_ID_RECORDER
    if (strcmp(name, APP_ID_RECORDER) == 0)
        return app_id_recorder;
#endif
#ifdef APP_ID_WEATHER
    if (strcmp(name, APP_ID_WEATHER) == 0)
        return app_id_weather;
#endif
#ifdef APP_ID_CALCULATOR
    if (strcmp(name, APP_ID_CALCULATOR) == 0)
        return app_id_calculator;
#endif
#ifdef APP_ID_ALARM
    if (strcmp(name, APP_ID_ALARM) == 0)
        return app_id_alarm;
#endif
#ifdef APP_ID_SETTING
    if (strcmp(name, APP_ID_SETTING) == 0)
        return app_id_setting;
#endif
#ifdef APP_ID_PHOTO
    if (strcmp(name, APP_ID_PHOTO) == 0)
        return app_id_photo;
#endif
    return 0xFF;
}

void add_or_update_custom_instruction(const char *id, const char *title,
                                      const char *trigger_type,
                                      uint32_t interval_sec, bool enabled,
                                      uint32_t version, const char *open_app)
{
    if (search_freeze_active())
        return; /* R60:搜尋期間清單凍結 —— 見 search_freeze_active 的說明 */
    /* A transient category view filter (@ / /) re-packs list_items[] too — lift it
       first so the push resolves against the real list (the disconnect restore
       below then unwinds the outer layer). */
    cat_filter_restore_full();
    /* If a disconnect-filter snapshot is live, a push means the link is back —
       restore the full PHONE list first so updates resolve against every item
       (including the previously hidden phone-relay ones), not the filtered view. */
    if (s_disc_backup)
    {
        memcpy(list_items, s_disc_backup, sizeof(list_items));
        list_item_count = s_disc_backup_count;
        rt_free(s_disc_backup);
        s_disc_backup = NULL;
    }
    /* First phone instruction while still showing the built-in apps (a 0x65 single
       arriving without a preceding 0x6B clear): drop the default apps — keeping
       only the pinned Settings prefix — and switch to PHONE mode before
       appending. */
    if (s_list_mode == LIST_MODE_DEFAULT_APPS)
    {
        list_item_count = app_base_count;
        s_list_mode = LIST_MODE_PHONE;
    }
    s_phone_connected = true;

    bool is_interval = (trigger_type && strcmp(trigger_type, "interval") == 0);
    int idx = find_instruction_by_id(id);
    if (idx >= 0)
    {
        /* 已存在 — 更新標題、參數和開關狀態 */
        strncpy(list_items[idx].title, title, LIST_ITEM_TITLE_LEN - 1);
        list_items[idx].title[LIST_ITEM_TITLE_LEN - 1] = '\0';
        list_items[idx].is_interval = is_interval;
        list_items[idx].interval_sec = interval_sec;
        list_items[idx].enabled = enabled;
        list_items[idx].version = version;
        if (trigger_type)
        {
            strncpy(list_items[idx].trigger_type, trigger_type, 31);
            list_items[idx].trigger_type[31] = '\0';
        }
        /* open_app may flip from set to unset across edits — always rewrite */
        if (open_app)
        {
            strncpy(list_items[idx].open_app, open_app, 31);
            list_items[idx].open_app[31] = '\0';
        }
        else
            list_items[idx].open_app[0] = '\0';
        /* Borrow the matching built-in app icon for openApp items so they look
           like apps on the watch (same glyph as the default app list). */
        list_items[idx].icon =
            (list_items[idx].open_app[0] != '\0')
                ? get_app_icon(app_id_from_name(list_items[idx].open_app))
                : NULL; /* @-contact service logo is applied later via set_instruction_service_icon */
        LOG_I("Updated id=%s, enabled=%d", id, enabled);
        return;
    }

    if (list_item_count >= MAX_LIST_ITEMS)
        return;

    list_item_t *instr = &list_items[list_item_count];
    memset(instr, 0, sizeof(list_item_t));
    strncpy(instr->id, id, LIST_ITEM_ID_LEN - 1);
    instr->id[LIST_ITEM_ID_LEN - 1] = '\0';
    strncpy(instr->title, title, LIST_ITEM_TITLE_LEN - 1);
    instr->title[LIST_ITEM_TITLE_LEN - 1] = '\0';
    if (trigger_type)
    {
        strncpy(instr->trigger_type, trigger_type, 31);
        instr->trigger_type[31] = '\0';
    }
    /* openApp items borrow the matching built-in app icon (see update path). */
    instr->icon = (open_app && open_app[0] != '\0')
                      ? get_app_icon(app_id_from_name(open_app))
                      : NULL; /* @-contact service logo is applied later via set_instruction_service_icon */
    instr->widget = NULL;
    instr->is_instruction = true;
    instr->is_interval = is_interval;
    instr->enabled = enabled;
    instr->interval_sec = interval_sec;
    instr->version = version;
    if (open_app)
    {
        strncpy(instr->open_app, open_app, 31);
        instr->open_app[31] = '\0';
    }
    list_item_count++;
}

/* P1 cross-device skaibar: tag an already-applied item's @-chat / /-action
   category for the filtered list views. Called right after
   add_or_update_custom_instruction from the phone-push path (0x65/0x6B "cat").
   Re-finds by id so add_or_update_custom_instruction's signature (and its four
   other callers) stay untouched. cat 0 = leave untagged. */
void set_instruction_category(const char *id, char cat)
{
    if (cat == 0)
        return;
    int idx = find_instruction_by_id(id);
    if (idx >= 0)
        list_items[idx].category = cat;
}

/* Apply an @-contact row's service logo to its right-side indicator dot, mirroring
   set_instruction_category: re-find by id (so add_or_update_custom_instruction's signature stays
   untouched) and set the icon from the phone-pushed "svc" field. NULL/unknown service ⇒ leave the
   default frame (founder 2026-06-29). */
void set_instruction_service_icon(const char *id, const char *svc)
{
    const char *icon = service_icon(svc);
    if (icon == NULL)
        return;
    int idx = find_instruction_by_id(id);
    if (idx >= 0)
        list_items[idx].icon = icon;
}

/* Apply a saved Action's type glyph, mirroring set_instruction_service_icon (re-find by id
   so add_or_update_custom_instruction's signature stays untouched).

   Runs LAST of the three icon sources on purpose — it only fills a still-empty slot, never
   overwrites. The other two are strictly more specific about the SAME row: an openApp row
   already borrowed the actual watch app's icon, and an @-contact row already has its
   messaging-service logo. The type glyph is the generic statement ("this is a music Action"),
   so it must lose to both. */
void set_instruction_type_icon(const char *id, const char *ico)
{
    const char *icon = action_type_icon(ico);
    if (icon == NULL)
        return;
    int idx = find_instruction_by_id(id);
    if (idx >= 0 && list_items[idx].icon == NULL)
        list_items[idx].icon = icon;
}

/* Helper: create list item UI objects for items in [start_idx, end_idx) */
/* list_item_count==0 placeholder: already paired but nothing added yet →
   "請在手機上新增" hint; not connected → the SAME pairing QR the Control
   Center's QR Code button shows (app_qrcode.c: BLE public MAC embedded in
   https://skaiwalk.com/download/id=<mac>, dynamic lv_qrcode) — not the
   generic static download QR device_pager.c uses elsewhere; this one is
   per-device so the scanned page can pair with THIS watch.

   The lv_qrcode itself is created lazily and deleted the moment it's not
   needed (leaving the disconnected+empty state, or the list closing via
   instruction_list_bar_set_visible(false)) — never left resident. A
   real-hw lesson recorded in this codebase: a resident TRUE_COLOR QR
   canvas (~60KB for a 148-176px code) has starved the LVGL heap before.
   app_qrcode.c gets this for free (bounded app-open/close lifetime); this
   widget lives inside the persistent instruction-list layout, so it has
   to manage that lifetime itself. */
static void destroy_empty_qr(void)
{
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->empty_qr_card == NULL)
        return;
    lv_obj_clean(p_instruction_list_layout->empty_qr_card);
}

static void ensure_empty_qr_created(void)
{
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->empty_qr_card == NULL)
        return;
    if (lv_obj_get_child_cnt(p_instruction_list_layout->empty_qr_card) > 0)
        return; /* already built */

    typedef struct { uint8_t addr[6]; } bd_addr_t;
    extern uint8_t ble_get_public_address(bd_addr_t *addr);
    bd_addr_t addr;
    char code[20] = {0};
    if (ble_get_public_address(&addr) == 0)
    {
        snprintf(code, sizeof(code), "%x-%x-%x-%x-%x-%x", addr.addr[0],
                 addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4],
                 addr.addr[5]);
    }
    else
    {
        strcpy(code, "0-0-0-0-0-0");
    }
    char url[128];
    snprintf(url, sizeof(url), "https://skaiwalk.com/download/id=%s", code);

    lv_obj_t *qrcode =
        lv_qrcode_create(p_instruction_list_layout->empty_qr_card);
    if (lv_qrcode_setparam(qrcode, 148, lv_color_black(), lv_color_white()) !=
        NULL)
    {
        lv_qrcode_update(qrcode, url, strlen(url));
    }
    lv_obj_center(qrcode);
}

void instruction_list_teardown_empty_qr(void)
{
    destroy_empty_qr();
}

static void update_list_empty_state(void)
{
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->empty_view == NULL)
        return;

    if (list_item_count > 0)
    {
        lv_obj_add_flag(p_instruction_list_layout->empty_view,
                        LV_OBJ_FLAG_HIDDEN);
        destroy_empty_qr();
        return;
    }

    lv_obj_clear_flag(p_instruction_list_layout->empty_view, LV_OBJ_FLAG_HIDDEN);
    if (s_phone_connected)
    {
        lv_obj_add_flag(p_instruction_list_layout->empty_qr_card,
                        LV_OBJ_FLAG_HIDDEN);
        destroy_empty_qr();
        lv_label_set_text(p_instruction_list_layout->empty_hint_label,
                          LV_EXT_STR_GET_BY_KEY(add_on_phone,
                                                "Add on your phone"));
    }
    else
    {
        lv_obj_clear_flag(p_instruction_list_layout->empty_qr_card,
                          LV_OBJ_FLAG_HIDDEN);
        ensure_empty_qr_created();
        lv_label_set_text(p_instruction_list_layout->empty_hint_label,
                          LV_EXT_STR_GET_BY_KEY(scan_to_connect_phone,
                                                "Scan to connect"));
    }
}

static void create_list_items_ui(lv_obj_t *list, uint8_t start_idx,
                                 uint8_t end_idx)
{
    for (uint8_t i = start_idx; i < end_idx; i++)
    {
        lv_obj_t *widget = NULL;
        lv_obj_t *item = lv_simplified_obj_create(list);
        lv_obj_clear_flag(item, LV_OBJ_FLAG_CLICKABLE);
        /* R3: bubble events up to the list so a RIGHT-swipe that lands on an item
           (items fill most of the panel) still reaches the list's GESTURE handler
           = swipe-right-to-close. The list handler ignores non-gesture events, so
           taps/scroll are unaffected. */
        lv_obj_add_flag(item, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_set_size(item, 466, LIST_ITEM_WIDGET_HEIGHT);
        if (i == 0)
        {
            lv_obj_set_pos(item, 0, (100 + LIST_ITEM_SPACING));
        }
        else
        {
            lv_obj_set_pos(item, 0,
                           (LIST_ITEM_WIDGET_HEIGHT + LIST_ITEM_SPACING) * i +
                               (100 + LIST_ITEM_SPACING));
        }

        bool has_widget = false;

        if (!list_items[i].is_instruction)
        {
            /* App items: check for special widgets */
            if (i < INSTRUCTION_LIST_ITEMS_DEF_COUNT)
            {
                if (INSTRUCTION_LIST_ITEMS_DEFINITION[i] == app_id_ai)
                {
                    extern lv_obj_t *lv_skai_widget_builder(lv_obj_t * parent);
                    widget = lv_skai_widget_builder(item);
                    has_widget = true;
                }
            }
        }

        if (has_widget && widget != NULL)
        {
            lv_obj_set_size(widget, LIST_ITEM_WIDGET_WIDTH,
                            LIST_ITEM_WIDGET_HEIGHT);
            if (i < INSTRUCTION_LIST_ITEMS_DEF_COUNT &&
                INSTRUCTION_LIST_ITEMS_DEFINITION[i] != app_id_ai)
            {
                lv_obj_set_style_clip_corner(widget, true, 0);
            }
            else
            {
                lv_obj_add_flag(widget, LV_OBJ_FLAG_SCROLLABLE);
            }
            if (i < INSTRUCTION_LIST_ITEMS_DEF_COUNT &&
                INSTRUCTION_LIST_ITEMS_DEFINITION[i] != app_id_ai)
            {
                lv_obj_set_style_border_color(widget, lv_color_hex(0xFFFFFF),
                                              0);
                lv_obj_set_style_border_width(widget, 2, 0);
                lv_obj_set_style_border_opa(widget, LV_OPA_20, 0);
                lv_obj_add_event_cb(widget, list_item_click_event_cb,
                                    LV_EVENT_CLICKED, (void *)&list_items[i]);
            }
        }

        /* Create touch overlay */
        touch_obj[i] = lv_obj_create(item);
        lv_obj_set_size(touch_obj[i], LIST_ITEM_WIDGET_WIDTH,
                        LIST_ITEM_WIDGET_HEIGHT);
        lv_obj_set_style_bg_opa(touch_obj[i], LV_OPA_0, 0);
        lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_EVENT_BUBBLE); /* R3: see item above */
        lv_obj_align(touch_obj[i], LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_event_cb(touch_obj[i], list_item_click_event_cb,
                            LV_EVENT_CLICKED, (void *)&list_items[i]);

        app_widget[i] = widget;
        list_items[i].widget = widget;

        /* Create app icon */
        p_instruction_list_layout->p_app_indicator_btn[i] = lv_img_create(item);
        if (list_items[i].img_path[0] != '\0')
        {
            lv_img_set_src(p_instruction_list_layout->p_app_indicator_btn[i],
                           list_items[i].img_path);
        }
        else if (list_items[i].icon != NULL)
        {
            lv_img_set_src(p_instruction_list_layout->p_app_indicator_btn[i],
                           list_items[i].icon);
        }
        lv_obj_align(p_instruction_list_layout->p_app_indicator_btn[i],
                     LV_ALIGN_RIGHT_MID, -25, 0);
        app_icon[i] = p_instruction_list_layout->p_app_indicator_btn[i];
        lv_obj_add_event_cb(app_icon[i], list_item_click_event_cb,
                            LV_EVENT_CLICKED, (void *)&list_items[i]);
        /* R3: bubble so a horizontal drag that LANDS ON THE ICON IMAGE still
           reaches the list's drawer-drag handler (without this only text-area
           drags worked — the icon is a separate clickable hit target). */
        lv_obj_add_flag(app_icon[i], LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_add_flag(app_icon[i], LV_OBJ_FLAG_HIDDEN);
        if (list_items[i].icon == NULL && list_items[i].img_path[0] == '\0')
        {
            lv_obj_add_flag(app_icon[i], LV_OBJ_FLAG_HIDDEN);
        }

        /* Create label */
        app_label[i] = lv_label_create(item);
        lv_label_set_text(app_label[i], list_items[i].title);
        lv_obj_set_style_text_font(app_label[i],
                                   LV_EXT_FONT_GET(get_system_font_size(1)), 0);
        lv_obj_set_style_text_color(app_label[i], lv_color_hex(0xFFFFFF), 0);

        /* R17(founder):標題右下角的設備名小副標退場 —— 設備名改顯示在右緣
           dot 輪播的位置(create_indicator_dots 的 conv 分支),字級加大。 */

        /* For instructions with interval, create switch and position label left
         */
        if (list_items[i].is_instruction && list_items[i].is_interval)
        {
            lv_obj_align(app_label[i], LV_ALIGN_LEFT_MID, 30, 0);

            /* 開關底座 */
            lv_obj_t *sw_bg = lv_obj_create(item);
            lv_obj_set_size(sw_bg, 50, 26);
            lv_obj_align(sw_bg, LV_ALIGN_RIGHT_MID, -30, 0);
            lv_obj_set_style_radius(sw_bg, 13, 0);
            lv_obj_set_style_border_width(sw_bg, 0, 0);
            lv_obj_clear_flag(sw_bg, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(sw_bg, LV_OBJ_FLAG_CLICKABLE);

            /* 圓形 knob */
            lv_obj_t *knob = lv_obj_create(sw_bg);
            lv_obj_set_size(knob, 20, 20);
            lv_obj_set_style_radius(knob, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(knob, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_border_width(knob, 0, 0);
            lv_obj_clear_flag(knob, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(knob, LV_OBJ_FLAG_CLICKABLE);

            if (list_items[i].enabled)
            {
                lv_obj_set_style_bg_color(sw_bg, lv_color_hex(0x00CCFF), 0);
                lv_obj_align(knob, LV_ALIGN_RIGHT_MID, -1, 0);
            }
            else
            {
                lv_obj_set_style_bg_color(sw_bg, lv_color_hex(0x444444), 0);
                lv_obj_align(knob, LV_ALIGN_LEFT_MID, 1, 0);
            }

            switch_objs[i] = sw_bg;
        }
        else
        {
            lv_obj_align(app_label[i], LV_ALIGN_CENTER, -20, 0);
            switch_objs[i] = NULL;
        }

        /* Hide labels for ai apps */
        if (!list_items[i].is_instruction &&
            i < INSTRUCTION_LIST_ITEMS_DEF_COUNT)
        {
            if (INSTRUCTION_LIST_ITEMS_DEFINITION[i] == app_id_ai)
            {
                lv_obj_add_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
            }
        }

        /* Default visibility: only the currently selected item exposes its
           label / switch / touch overlay. scroll_list normally maintains this
           when the selection changes, but on initial create / rebuild no
           scroll has fired yet, so non-selected items would otherwise show
           every label at once. */
        if (i != selected_item_index)
        {
            if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                lv_obj_add_flag(app_label[i], LV_OBJ_FLAG_HIDDEN);
            if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                lv_obj_add_flag(switch_objs[i], LV_OBJ_FLAG_HIDDEN);
            if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
                lv_obj_add_flag(touch_obj[i], LV_OBJ_FLAG_HIDDEN);
        }

        LOG_D("List item %d: id=%s, title=%s, is_instruction=%d", i,
              list_items[i].id, list_items[i].title,
              list_items[i].is_instruction);
    }
}

static rt_tick_t s_last_refresh_tick = 0;
/* R42:這次 refresh 是「原樣重建」(ensure_ui 把 release 掉的列建回來)還是「內容變了」。
   原樣重建不可以作廢影像快取,否則每次進場都要重新解碼 NAND 上的圖。 */
static bool s_restore_rebuild = false;
static lv_timer_t *s_pending_refresh_timer = NULL;
/* s_in_refresh_scroll is declared earlier in the file (near the forward
   decls for list_window_scroll_event_cb) so the scroll handler can read
   it. It is set true for the duration of refresh_custom_instructions(). */
/* Set by external callers (e.g. voice_say MSH demo) to make the next
   refresh_custom_instructions auto-scroll to the newly added last item
   so the user sees the addition without manually scrolling. */
static bool s_force_scroll_to_last = false;
/* When set, refresh_custom_instructions overrides the indicator-dot input
   value so the dots span the BOTTOM half of the arc (angles 0..+72°) with
   the newest item at angle +72°. Without this, the natural "selected=last"
   layout places dots in the TOP half (angles -72..0°), which users
   intuitively read as "scrolled to the top of the list". */
static bool s_force_visual_at_bottom = false;

static void deferred_refresh_cb(lv_timer_t *t)
{
    s_pending_refresh_timer = NULL;
    /* Bypass debounce on the trailing run so the latest pending state
       (e.g. multiple instructions added in rapid succession) is rebuilt. */
    s_last_refresh_tick = 0;
    lv_timer_del(t);
    refresh_custom_instructions();
}

/* R3 device-page overlay: snapshot / restore of the watch-face instruction list.
   feed_active_device_options_to_list (device_pager) overwrites this shared list
   with the active device's options while on the device page; we snapshot the
   watch-face list on the way in and restore it on the way out, so reopening the
   watch-face skaibar shows the user's own instructions again instead of the last
   device's options — the phone does NOT re-push them on return. Heap-backed
   (held only while on the device page) to avoid a permanent duplicate array. */
static list_item_t *s_base_items = NULL;
static uint8_t s_base_item_count;
static uint8_t s_base_app_count;
static list_mode_t s_base_list_mode; /* snapshot s_list_mode too: the device-page
   feed calls clear_custom_instructions (→ PHONE mode), which would otherwise leak
   back to the watch face and mix the next phone push into the default-app body. */

void instruction_list_save_base(void)
{
    if (s_base_items) return; /* one snapshot per device-page visit */
    s_base_items = rt_malloc(sizeof(list_items));
    if (!s_base_items) return; /* OOM: skip — restore no-ops, list just stays */
    memcpy(s_base_items, list_items, sizeof(list_items));
    s_base_item_count = list_item_count;
    s_base_app_count = app_base_count;
    s_base_list_mode = s_list_mode;
}

void instruction_list_restore_base(void)
{
    if (!s_base_items) return;
    memcpy(list_items, s_base_items, sizeof(list_items));
    list_item_count = s_base_item_count;
    app_base_count = s_base_app_count;
    s_list_mode = s_base_list_mode;
    rt_free(s_base_items);
    s_base_items = NULL;
    refresh_custom_instructions();
}

/* 把 target item 的中心精確對到螢幕垂直中心(LV_VER_RES/2)。
   lv_obj_scroll_to_view 對「相鄰 item 重疊 100px」的這個清單置中不準(它把 200px 的
   item bbox 塞進 viewport、不是把 item 中心對準畫面中心),會讓選中項 label 落偏下 +
   scroll_list 從幾何反推的 selected 差一格 → label 錯位+重疊(founder 2026-07-23
   revert voice force_scroll 的真因)。改成顯式位移把 child 中心對到畫面中心。 */
static void scroll_center_item(lv_obj_t *list, uint16_t target)
{
    lv_obj_t *child = lv_obj_get_child(list, target);
    if (child == NULL || !lv_obj_is_valid(child)) return;
    lv_obj_update_layout(list);
    lv_area_t ca;
    lv_obj_get_coords(child, &ca);
    lv_coord_t child_center = (ca.y1 + ca.y2) / 2;
    lv_coord_t cur_scroll = lv_obj_get_scroll_y(list);
    lv_obj_scroll_to_y(list, cur_scroll + (child_center - LV_VER_RES / 2),
                       LV_ANIM_OFF);
}

void refresh_custom_instructions(void)
{
    /* 滑出關閉動畫進行中:擋退出途中的 UI 重繪(本地 restore 與手機 replace-all 重推的共同出口)。
       flag 在滑出結束、列表已 HIDDEN 後才清 → gate 只在列表可見的滑出過程生效。 */
    if (s_list_sliding_out) return;

    /* R21:背景刷新(桌面刪 session 等)絕對不可以改變「清單是不是開著」——
       instruction_list_is_visible() 是 check_is_at_instruction_list 的判定來源,
       被刷新過程意外掀開就會把 _at_instruction_list 閂成 true、四條邊緣 zone 關掉,
       畫面明明在錶盤卻只剩 app 返回鍵(=R16 那個假錶盤,換成背景刷新觸發)。
       進場記下 hidden,出場照原樣還原。 */
    bool r21_was_hidden =
        p_instruction_list_layout != NULL &&
        p_instruction_list_layout->p_instruction_list_bg != NULL &&
        lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg) &&
        lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_bg,
                        LV_OBJ_FLAG_HIDDEN);

    /* R30:重建會在 list / bg 底下**建立子物件**,父物件若已被拆掉(滑鼠圖層接管、
       app 切換),lv_obj_class_init_obj → lv_obj_mark_layout_as_dirty 會踩到已釋放
       記憶體 → hard fault(2026-08-12 從媒體頁進滑鼠頁時,R25 的輪詢在背景觸發重建
       打中的就是這個)。NULL 檢查不夠,物件是被 del 掉、指標還在,要 lv_obj_is_valid。 */
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->list == NULL ||
        !lv_obj_is_valid(p_instruction_list_layout->list) ||
        p_instruction_list_layout->p_instruction_list_bg == NULL ||
        !lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg))
    {
        LOG_W("[R30] refresh skipped: list objects gone (torn down)");
        return;
    }

    /* R33:列的 UI 被 release 掉時(滑鼠頁佔著 heap)**不要重建**。桌面 push-on-change
       隨時可能送 0x20 進來,一重建就把 R32 讓出的記憶體又吃回去 —— 而退出滑鼠頁那
       一刻是全域用量最高峰(滑鼠圖層還在,又要亮出 tileview/媒體頁),實測就是在那裡
       `sys memory is full!`。資料已經寫進 list_items[],下次開清單時 ensure_ui 會
       重建成最新內容,什麼都不會漏。 */
    extern bool instruction_list_ui_is_released(void);
    if (instruction_list_ui_is_released())
    {
        LOG_W("[R33] refresh deferred: list UI released (mouse page owns the heap)");
        return;
    }
    /* R33 加固(2026-08-15 真機:語音站開著時一條 refresh 把隱藏清單重建 8 列 →
       EPIC render `sys memory is full!` → assert 重開):滑鼠的鍵盤/語音站模式佔著
       heap 期間,清單又是**隱藏**狀態(抽屜沒開)就沒有任何理由重建 —— 不管 released
       旗標此刻是誰清的,一律重新釋放+defer。抽屜真的要開時 prepare 的 ensure_for_feed
       會解鎖。 */
    {
        extern bool hid_mouse_keyboard_mode_active(void);
        bool list_hidden =
            p_instruction_list_layout == NULL ||
            p_instruction_list_layout->p_instruction_list_bg == NULL ||
            !lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg) ||
            lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_bg,
                            LV_OBJ_FLAG_HIDDEN);
        if (hid_mouse_keyboard_mode_active() && list_hidden)
        {
            extern void instruction_list_release_ui(void);
            instruction_list_release_ui();
            LOG_W("[R33] refresh deferred: keyboard mode owns the heap");
            return;
        }
    }
    open_scroll_motor = false;
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->list == NULL)
        return;

    /* Gate scroll-to-fade: scroll events fired inside this function are
       programmatic and must NOT dismiss the AI widget. */
    s_in_refresh_scroll = true;

    /* Founder direction 2026-05-19: every list update lands on the newest
       (last) item. Previously the s_force_scroll_to_last gate had to be
       opted in per-caller (instruction_list_force_scroll_to_last); now
       refresh itself flips it so single-item upserts (0x65), batch
       replace-all (0x6B), voice_say, and the mock cycler all converge
       on the same UX — newest at the focus / centre. The other two
       branches further down (saved_selected out of range, restore prior
       position) become dead but I leave them so callers that DO want
       to preserve position can clear the flag before refresh.

       Founder direction 2026-07-23: only re-home to the newest item when the
       user is NOT on the instruction-list page. While they ARE browsing it, a
       phone-pushed refresh (batch 0x6B replace-all fires on every reveal, then
       again on any interaction) must NOT yank their scroll back to the bottom —
       preserve where they scrolled to. Same gate the trailing reset_list()
       uses; leaving the page still re-homes so the next open shows newest.

       Use instruction_list_is_visible(), NOT is_at_instruction_list(): the
       latter only flips true on a later poll (~400ms after reveal_drag_begin —
       ATINST 0->1 in the trace), so a "reveal then scroll immediately" gesture
       fires this refresh while is_at is still false → it re-homed AND ran the
       trailing reset mid-scroll, which both yanked scroll to the bottom and tore
       the icon/label slide-anim apart (icon left mid-slide, label recentred).
       is_visible() reads true the instant reveal un-hides the list_bg, so it
       covers that window.

       BUT respect an explicit per-caller opt-in first: instruction_list_force_
       scroll_to_last() (e.g. the mic/voice-input path) sets s_force_scroll_to_last
       true because it WANTS the newest option homed even while the list is visible.
       Only fall back to the visibility rule when nobody opted in — otherwise a
       voice-input refresh preserved the user's prior scroll instead of homing to
       the newest item (founder 2026-07-23). */
    if (!s_force_scroll_to_last)
    {
        /* 預設:只有離開 actions 頁才 re-home 到最新;瀏覽中手機 push 的 replace-all
           保持原 scroll(founder 2026-07-23,別打斷瀏覽)。例外:使用者正在 skaibar
           語音查詢流程中(AI 輸入框開著 或 tracking session)→ 這次刷新帶來的是查詢
           結果選項,定位到最相關(最新)那個,否則使用者查完看不到結果(founder NOTE
           @mic_bar_voice_start 的 follow-up:配合 scroll_center_item 的精確定位一起做)。*/
        s_force_scroll_to_last = !instruction_list_is_visible() ||
                                 get_is_open_instruction_list_ai() ||
                                 s_skaibar_tracking_active;
    }

    LOG_I("Refreshing custom instructions...");
    /* Trailing-edge debounce: within 500ms, skip the immediate run but
       schedule a deferred refresh so the latest call still rebuilds the UI.
       Without this, a burst of N>=2 add_or_update calls would leave
       list_item_count incremented past the number of indicator dots actually
       created, breaking dot positioning. */
    rt_tick_t now = rt_tick_get();
    if (s_last_refresh_tick != 0 &&
        (now - s_last_refresh_tick) < rt_tick_from_millisecond(500))
    {
        LOG_I("refresh_custom_instructions: deferred (debounce)");
        if (s_pending_refresh_timer == NULL)
        {
            s_pending_refresh_timer =
                lv_timer_create(deferred_refresh_cb, 550, NULL);
            lv_timer_set_repeat_count(s_pending_refresh_timer, 1);
        }
        return;
    }
    s_last_refresh_tick = now;
    if (s_pending_refresh_timer != NULL)
    {
        lv_timer_del(s_pending_refresh_timer);
        s_pending_refresh_timer = NULL;
    }

    /* Invalidate the image cache for every instruction's current image path.
       update_instruction_image() running on a non-LVGL thread skips its own
       cache invalidation (deferring it here is safer than calling LVGL APIs
       on KE_EVT2's 4KB stack). When the phone replaces an existing image at
       the same path, we need this flush so lv_img_set_src below picks up the
       new pixels rather than a stale cached entry.

       R42(founder:「文字進來但 icon 是空的,晚一點才出現」):**restore 型重建不要
       作廢快取**。R33 起清單一關就釋放列物件,再開時 ensure_ui 重建 —— 資料一個
       字都沒變,卻把每張圖的解碼結果丟掉。

       R44(founder 給的決定性線索:「從左邊邊緣滑不會消失,從中間往右滑就會先消失後面
       才出現」):那是兩條不同路徑 —— 左緣拖曳是浮層直接開;中間往右滑會 settle 到左
       頁,而 settle 會讓手機把整份 actions **replace-all 重推**一次。重推的內容通常
       一模一樣,卻走「內容變了」這條路把所有圖的快取丟光 → 重讀 NAND、重解碼 → 圖標
       慢一拍。所以作廢範圍縮到「這一項的圖檔真的被換過」(update_instruction_image
       設 img_dirty),而不是「有人重建了清單」。 */
    if (!s_restore_rebuild)
    {
        for (uint8_t i = 0; i < list_item_count; i++)
        {
            if (list_items[i].img_dirty && list_items[i].img_path[0] != '\0')
            {
                lv_img_cache_invalidate_src(list_items[i].img_path);
                list_items[i].img_dirty = false;
            }
        }
    }
    s_restore_rebuild = false;

    /* R32:走到這裡就是要完整 teardown+rebuild,不管先前是不是被 release 過。 */
    {
        extern void instruction_list_mark_ui_rebuilt(void);
        instruction_list_mark_ui_rebuilt();
    }

    /* R56:重建的共同出口 —— 內容不管是誰換的(手機 replace-all、session 注入、分類篩選),
       畫出來之前都在這裡套用一次語音搜尋。pack_text_filter 只動陣列、不再呼叫 refresh,
       所以不會遞迴。 */
    pack_text_filter();

    lv_obj_t *list = p_instruction_list_layout->list;

    /* 先刪除舊的指示點（它們是 bg 的子物件，lv_obj_clean(list) 不會刪到） */
    lv_obj_t *bg = p_instruction_list_layout->p_instruction_list_bg;
    for (uint8_t i = 0; i < MAX_LIST_ITEMS; i++)
    {
        if (p_instruction_list_layout->indicator_dots_bg[i] != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->indicator_dots_bg[i]))
        {
            lv_obj_del(p_instruction_list_layout->indicator_dots_bg[i]);
        }
        /* NULL the dot handles IMMEDIATELY after freeing the bg (app_icon_shadow
           and the dot img are its children, freed with it). lv_obj_del and the
           lv_obj_clean below dispatch DELETE/SCROLL events synchronously, which
           re-enter update_indicator_dots_position via the list scroll handler
           (scroll_list -> update_indicator_dots_position). Leaving these pointing
           at freed objects until the reset loop further down faulted there
           (DACCVIOL @ ~0x7E9). Clearing here closes the free->NULL window. */
        p_instruction_list_layout->indicator_dots_bg[i] = NULL;
        p_instruction_list_layout->indicator_dots[i] = NULL;
        app_icon_shadow[i] = NULL;
    }

    /* Save current scroll position and selected index */
    lv_coord_t saved_scroll_y = lv_obj_get_scroll_y(list);
    uint16_t saved_selected = selected_item_index;
    /* R48(founder:「拉出清單中途在 actions 最下面,之後瞬間跳到 session 第二項」):
       還原捲動位置**不能用像素**。手機的 replace-all 會把 sessions 洗掉再重建,session
       pager 隨即把它們插回清單**最上面**(conv 段在前、actions 在後) —— 插在上面的項目
       把底下全部往下推,於是同一個 saved_scroll_y 對到的已經是別的項目,畫面就往上跳進
       session 段。改記**選中項的 id**,重建後找回同一個 id 再精確置中;找不到(該項真的
       被刪了)才退回舊行為。 */
    char saved_id[sizeof(list_items[0].id)];
    saved_id[0] = '\0';
    if (saved_selected < list_item_count)
    {
        strncpy(saved_id, list_items[saved_selected].id, sizeof(saved_id) - 1);
        saved_id[sizeof(saved_id) - 1] = '\0';
    }

    /* Delete ALL children of the list */
    lv_obj_clean(list);

    /* Reset UI arrays */
    for (uint8_t i = 0; i < MAX_LIST_ITEMS; i++)
    {
        app_icon[i] = NULL;
        app_widget[i] = NULL;
        touch_obj[i] = NULL;
        app_label[i] = NULL;
        switch_objs[i] = NULL;
        app_icon_shadow[i] = NULL;
        p_instruction_list_layout->p_app_indicator_btn[i] = NULL;
        p_instruction_list_layout->indicator_dots[i] = NULL;
        p_instruction_list_layout->indicator_dots_bg[i] = NULL;
    }

    /* Recreate all list item UI */
    LOG_I("[RCK] A before create_list_items_ui n=%d", (int)list_item_count);
    create_list_items_ui(list, 0, list_item_count);
    LOG_I("[RCK] B after create_list_items_ui");
    update_list_empty_state();

    /* 重建指示點 */
    LOG_I("[RCK] C before create_indicator_dots");
    create_indicator_dots(bg);
    LOG_I("[RCK] D after create_indicator_dots");

    /* 新建的 dots 是 bg 的 child，appended 在尾端 → 預設 z-order 在 arc_zone
     * 上面，導致 dots 把 press 從 arc_zone 搶走。先把 arc_zone 拉回最上層，
     * 再把 ai_bg 拉到最上 — 最終順序：dots → arc_zone → ai_bg（top）*/
    if (p_instruction_list_layout->arc_handle != NULL)
    {
        arc_scroll_bring_to_front(p_instruction_list_layout->arc_handle);
    }
    if (p_instruction_list_layout->p_instruction_list_ai_bg != NULL &&
        lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_ai_bg))
    {
        lv_obj_move_foreground(
            p_instruction_list_layout->p_instruction_list_ai_bg);
    }

    /* Force layout so child coords are valid */
    lv_obj_update_layout(list);

    /* Restore scroll position */
    old_selected_item_index = (uint16_t)-1;
    /* R64(founder:「滑到上面 ANDREW 那列會一直跳回下面 DESKTOP 那列」):語音搜尋開著時,
       清單就是使用者正在挑的那幾個選項 —— 任何自動歸位(進場落點、re-home 到最後一項)都
       是在跟他的手指搶。搜尋期間一律走「還原原本選中的那一項」(下面的 id 比對分支)。 */
    if (s_text_filter[0] != '\0')
    {
        s_entry_landing_pending = false;
        s_force_scroll_to_last = false;
    }
    /* R80:寬限窗到期 → 進場落點任務結束,改走 R48「還原原本那一項」。 */
    if (s_entry_landing_pending &&
        (rt_tick_get() - s_entry_landing_tick) >
            rt_tick_from_millisecond(ENTRY_LANDING_GRACE_MS))
        s_entry_landing_pending = false;
    if (s_entry_landing_pending && list_item_count > 0)
    {
        /* R50:這次重建屬於「剛進場」 → 重新套用落點規則(最新的 session,沒有就最後一項),
           而不是還原重建前的位置 —— 進場當下 sessions 常常還沒到。 */
        uint16_t target = (uint16_t)(list_item_count - 1);
        for (uint8_t i = 0; i < list_item_count; i++)
            if (strncmp(list_items[i].id, "conv:", 5) == 0)
                target = i; /* 最後一個 conv 項 = 最新的 session */
        LOG_W("[land] refresh re-land target=%u id=%s", (unsigned)target,
              list_items[target].id);
        app_scroll_target_item = target;
        selected_item_index = target;
        scroll_center_item(list, target);
        lv_obj_update_layout(list);
        scroll_list(list, 0);
        selected_item_index = target;
        app_scroll_target_item = target;
        /* R80(取代 R53 的「落到 conv 就清」):寬限窗內保持 pending —— 另一台桌面的
           清單晚幾秒才到時還要再補正一次;窗由本函式開頭的到期檢查與使用者手勢負責關。 */
    }
    else if (s_force_scroll_to_last && list_item_count > 0)
    {
        /* Voice-say flow: scroll so the LAST (newest) item becomes the
           focused/centered one. LV_ANIM_OFF makes the scroll settle
           synchronously — important because scroll_list() below reads
           child coords, and with ANIM_ON the items would still be at
           their pre-scroll positions, yielding wrong dot placement. */
        s_force_scroll_to_last = false;
        uint16_t target = list_item_count - 1;
        app_scroll_target_item = target;
        selected_item_index = target;
        scroll_center_item(list, target); /* 精確置中,取代不準的 scroll_to_view */
        lv_obj_update_layout(list);
        scroll_list(list, 0);
        /* scroll_list() reassigns selected_item_index to whichever item is
           closest to screen y-center; when the list scroll didn't land
           exactly on the target (LVGL scroll-to-view doesn't always
           perfectly center for short lists), the wrong item becomes
           selected. Re-assert so the label/highlight points to the
           newly-spoken item. */
        selected_item_index = target;
        app_scroll_target_item = target;
        LOG_I("[VOICE] re-asserted selected_item_index=%u (list_item_count=%u)",
              (unsigned)target, (unsigned)list_item_count);
    }
    else if (saved_selected >= list_item_count && list_item_count > 0)
    {
        /* Selected item was removed — scroll to new last item */
        uint16_t target = list_item_count - 1;
        app_scroll_target_item = target;
        selected_item_index = target;
        scroll_center_item(list, target); /* 精確置中,取代不準的 scroll_to_view */
        lv_obj_update_layout(list);
        scroll_list(list, 0);
    }
    else
    {
        /* R48:先用 id 找回原本選中的那一項(它可能已經被上面新插入的 session 推到別的
           index),精確置中 —— 這才是「位置沒變」的正確定義。 */
        uint16_t target = (uint16_t)-1;
        if (saved_id[0] != '\0')
        {
            for (uint8_t i = 0; i < list_item_count; i++)
            {
                if (strcmp(list_items[i].id, saved_id) == 0)
                {
                    target = i;
                    break;
                }
            }
        }
        if (target != (uint16_t)-1)
        {
            app_scroll_target_item = target;
            selected_item_index = target;
            scroll_center_item(list, target);
            lv_obj_update_layout(list);
            scroll_list(list, 0);
            selected_item_index = target;
            app_scroll_target_item = target;
        }
        else
        {
            /* 找不到同一項(例如清單本來就空的)→ 沿用舊的像素還原 */
            lv_obj_scroll_to_y(list, saved_scroll_y, LV_ANIM_OFF);
            lv_obj_update_layout(list);
            scroll_list(list, 0);
        }
    }

    /* Recalculate input_value for indicator dots based on current
     * selected_item_index. With selected=N-1 (last item after voice_say),
     * input_val resolves to 37 → the new item sits at angle 0° (right-mid,
     * y=233), naturally above the pill at y=296+. Older items appear at
     * negative angles (above center). This is the "selected at center,
     * older above" arc convention, which avoids hiding the new item
     * behind the AI widget pill. */
    s_force_visual_at_bottom = false; /* unused now; reset for safety */
    if (list_item_count > 0)
    {
        float total_range = 100.0f * list_item_count;
        float base_input = 63.0f;
        float input_val = total_range - base_input -
                          selected_item_index * (total_range / list_item_count);
        gesture_starting_value = (uint16_t)input_val;
        update_indicator_dots_position(gesture_starting_value);
    }
    open_scroll_motor = true;
    LOG_I("[RCK] E before is_at/reset");
    /* is_visible() not is_at_instruction_list() — same reveal-window reason as
       the s_force_scroll_to_last gate above: a reveal-then-scroll refresh must
       not reset (recenter icon/label, snap scroll) while the user is mid-slide. */
    if (!instruction_list_is_visible())
    {
        reset_list();
    }
    /* item 數變了 → 同步給共用 arc_scroll 模組做 scroll clamp */
    if (p_instruction_list_layout->arc_handle != NULL)
    {
        arc_scroll_set_item_count(p_instruction_list_layout->arc_handle,
                                  list_item_count);
    }
    LOG_I("[RCK] F refresh tail reached (done)");
    LOG_D("refresh_custom_instructions: %d items total", list_item_count);
    s_in_refresh_scroll = false;

    /* ADR-0020:左頁的 actions 區段從 list_items[] 匯出 —— 清單落地後叫它重畫。
       這裡已經在 LVGL thread、且過了 debounce,不會抖。 */
    {
        extern void session_list_actions_changed(void);
        session_list_actions_changed();
    }

    /* R21:還原進場時的 hidden(見函式開頭),再讓狀態機重新評估一次 ——
       刷新若在背景把清單掀開又蓋回,latch 仍可能停在錯值,check_main_page
       是四條邊緣 zone 顯示狀態的唯一 owner,補跑一次就自癒。 */
    if (r21_was_hidden && p_instruction_list_layout != NULL &&
        p_instruction_list_layout->p_instruction_list_bg != NULL &&
        lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg) &&
        !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_bg,
                         LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_bg,
                        LV_OBJ_FLAG_HIDDEN);
        LOG_W("[R21] refresh re-hid the list (was hidden on entry)");
    }
    {
        extern void check_main_page(void);
        check_main_page();
    }
}


void update_instruction_image(const char *id, const char *path)
{
    /* list_items[] is owned by the LVGL thread. If a non-LVGL caller (the
       file-receive thread, when an icon download completes) lands here, defer
       the whole op so the find + img_path write happen on the LVGL thread too —
       otherwise they race refresh_custom_instructions(). The batch/single drain
       calls this already on the LVGL thread, so it runs inline there. */
    if (!is_on_lvgl_thread())
    {
        extern void instruction_op_enqueue_image(const char *id,
                                                 const char *path);
        instruction_op_enqueue_image(id, path);
        return;
    }

    int idx = find_instruction_by_id(id);
    if (idx < 0)
    {
        LOG_W("update_instruction_image: id=%s not found", id);
        return;
    }

    /* Build image path using id prefix (before first '-') */
    char id_prefix[64];
    strncpy(id_prefix, id, sizeof(id_prefix) - 1);
    id_prefix[sizeof(id_prefix) - 1] = '\0';
    char *dash = strchr(id_prefix, '-');
    if (dash)
        *dash = '\0';

    char img_path[128];
    rt_snprintf(img_path, sizeof(img_path), "/assets/images/instruction/%s.bin",
                id_prefix);

    /* Path-string update is thread-safe (single owner per index in practice). */
    strncpy(list_items[idx].img_path, img_path,
            sizeof(list_items[idx].img_path) - 1);
    list_items[idx].img_path[sizeof(list_items[idx].img_path) - 1] = '\0';
    /* R44:這裡是「圖真的換了」的唯一入口(手機下載完新 icon 覆寫同一路徑),所以只有
       這些項目需要在下次 refresh 丟掉解碼快取。見 refresh 內的說明。 */
    list_items[idx].img_dirty = true;

    /* LVGL ops only on the LVGL thread.
       BLE notify (parse_notify) and file-receive callback (bloc_filesystem)
       both run on KE_EVT2 (4KB stack). lv_img_set_src / lv_obj_clear_flag
       cascade through lv_event_send → potentially the list's scroll handler
       → scroll_list, blowing the stack. Defer the rebuild via the LVGL msg
       queue; refresh_custom_instructions invalidates caches and recreates
       the indicator dots from list_items[i].img_path. */
    if (!is_on_lvgl_thread())
    {
        lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_INSTRUCTION_LIST};
        lvgl_send_msg(msg);
        return;
    }

    lv_img_cache_invalidate_src(img_path);

    /* Update the indicator dot directly if UI exists */
    if (p_instruction_list_layout != NULL &&
        p_instruction_list_layout->indicator_dots[idx] != NULL &&
        lv_obj_is_valid(p_instruction_list_layout->indicator_dots[idx]))
    {
        lv_img_set_src(p_instruction_list_layout->indicator_dots[idx],
                       list_items[idx].img_path);
        lv_obj_clear_flag(p_instruction_list_layout->indicator_dots[idx],
                          LV_OBJ_FLAG_HIDDEN);
        LOG_I("Updated instructionwith new image %s", img_path);
    }
}


void back_to_instruction_list_btn(void)
{
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->app_list_tileview == NULL)
        return;
    lv_obj_set_tile_id(p_instruction_list_layout->app_list_tileview, 0, 0,
                       LV_ANIM_ON);
    LOG_I("Navigate back to instruction list via button");
}

bool get_app_list_tileview_page(void)
{
    /* App list tileview was removed; the instruction list is the only page. */
    return true;
}

/* 右側弧形觸控滾動 — 改用共用模組 common/arc_scroll.h，跟 exercise、clock
 * 等其他位置共享同一份偵測算法。LIST_ITEM_SLOT_HEIGHT / LIST_ITEM_SLOT_ANGLE_DEG
 * 仍留著，給 cfg 傳進共用模組用。 */
#define LIST_ITEM_SLOT_HEIGHT (LIST_ITEM_WIDGET_HEIGHT + LIST_ITEM_SPACING)
#define LIST_ITEM_SLOT_ANGLE_DEG 36 /* 與 update_indicator_dots_position::angle_per_dot 一致；跟 exercise 對齊 */

static void inst_arc_reset_drag_state(void)
{
    s_inst_arc_drag_active = false;
    s_inst_drag_initialized = false;
    s_inst_drag_input = 0;
    s_inst_drag_last_idx = -1;
}

/* dot 回彈動畫 — release 時若 input 還在 elastic overshoot 區，把它從當前
 * 位置補間到 canonical（idx 對應的 input value），看得到 dot 平滑回彈 */
/* s_inst_snap_anim_dummy 定義已提前到檔案上方（SCROLL_END pin 需先取消此 anim）*/
static void inst_snap_anim_exec_cb(void *var, int32_t value)
{
    (void)var;
    if (s_inst_arc_drag_active) return;
    update_indicator_dots_position((int)value);
}

static void inst_start_snap_anim(int from, int to)
{
    lv_anim_del(&s_inst_snap_anim_dummy, inst_snap_anim_exec_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_inst_snap_anim_dummy);
    lv_anim_set_exec_cb(&a, inst_snap_anim_exec_cb);
    lv_anim_set_values(&a, from, to);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

/* drag_cb 模式：arc 拖動時不直接動 list，由這裡接管。
 * - 累積 input value，每幀 call update_indicator_dots_position 讓 dot 平滑轉
 * - 偵測「最靠近中央的 dot 換了一顆」（= page change）才 call scroll_list_to_index
 *   把 list snap 到對應 item，list 動畫期間 scroll_list 的 gate 會擋住反推 */
static void inst_arc_drag_cb(lv_coord_t scroll_delta_px, void *ctx)
{
    (void)ctx;
    int total = (int)list_item_count;
    if (total <= 0) return;

    s_inst_arc_drag_active = true;
    s_last_arc_drag_tick = rt_tick_get(); /* 標記 arc-drag 這一幀真的在驅動（供 arc_drag_is_live 區分殘留）*/

    /* 第一次進來：用目前的 selected_item_index 反算 input。
     * input 跟 idx 對應公式（見 update_indicator_dots_position 的 offset_angle）：
     *   input = 100*(N - idx) - 63 */
    if (!s_inst_drag_initialized)
    {
        /* 新的拖動開始 — 取消上一輪 release 起的 snap anim */
        lv_anim_del(&s_inst_snap_anim_dummy, inst_snap_anim_exec_cb);
        int idx = (int)selected_item_index;
        if (idx < 0) idx = 0;
        if (idx >= total) idx = total - 1;
        s_inst_drag_input = 100 * (total - idx) - 63;
        s_inst_drag_last_idx = idx;
        s_inst_drag_initialized = true;
    }

    /* d_input = -d_scroll * 100 / pitch；instruction_list 的 pitch =
     * LIST_ITEM_SLOT_HEIGHT，dots_value 公式裡是 1:1（因為 SLOT_HEIGHT=100），
     * 寫成 generic 式更安全 */
    const int pitch = LIST_ITEM_SLOT_HEIGHT;
    int target_input = s_inst_drag_input - ((int)scroll_delta_px * 100) / pitch;

    int min_input = 100 - 63;             /* idx=N-1 */
    int max_input = 100 * total - 63;     /* idx=0 */
    /* elastic overshoot：跟 message_list 一致，邊界外 0.4 resistance、上限 50
     * input 單位（= 半 slot）。snap_cb 在釋放時把 dot 拉回 valid */
    const int MAX_OVERSHOOT = 50;
    if (target_input < min_input)
    {
        int over_now = (s_inst_drag_input < min_input) ? (min_input - s_inst_drag_input) : 0;
        int over_raw = min_input - target_input;
        if (over_raw > over_now)
        {
            int additional = (over_raw - over_now) * 4 / 10;
            int new_over = over_now + additional;
            if (new_over > MAX_OVERSHOOT) new_over = MAX_OVERSHOOT;
            target_input = min_input - new_over;
        }
    }
    else if (target_input > max_input)
    {
        int over_now = (s_inst_drag_input > max_input) ? (s_inst_drag_input - max_input) : 0;
        int over_raw = target_input - max_input;
        if (over_raw > over_now)
        {
            int additional = (over_raw - over_now) * 4 / 10;
            int new_over = over_now + additional;
            if (new_over > MAX_OVERSHOOT) new_over = MAX_OVERSHOOT;
            target_input = max_input + new_over;
        }
    }
    s_inst_drag_input = target_input;

    update_indicator_dots_position(s_inst_drag_input);

    int closest_idx = total - ((s_inst_drag_input + 63 + 50) / 100); /* round */
    if (closest_idx < 0) closest_idx = 0;
    if (closest_idx >= total) closest_idx = total - 1;

    if (closest_idx != s_inst_drag_last_idx)
    {
        s_inst_drag_last_idx = closest_idx;
        scroll_list_to_index((uint16_t)closest_idx, true); /* arc 圓形滾動：保留動畫平滑切換（founder 要的效果）*/
    }
}

static lv_obj_t *list_arc_tap_cb(lv_point_t pt, void *ctx)
{
    (void)ctx;
    /* tap 收尾，順手 reset drag state */
    inst_arc_reset_drag_state();
    /* arc 模組 overlay 攔走 press → CLICK 不會 bubble 到 dot 或 touch_obj。
     * 仿 app_exercise.c 的 tap 路徑：
     *   1. 先用 press 點比對所有可見 indicator dot 的 bbox，找到哪顆 dot 就 forward
     *      CLICKED 給那顆，dot 上有註冊 list_item_click_event_cb（user_data = 對應
     *      list_items[i] ptr），所以點哪顆 dot 就觸發那 item 的動作。
     *   2. fallback：press 不在任何 dot 上但在選中項 touch_obj 範圍內 → forward 給
     *      touch_obj，行為跟舊版相同（保留中央區塊大面積可點）。 */
    if (p_instruction_list_layout != NULL)
    {
        for (int i = 0; i < list_item_count; i++)
        {
            lv_obj_t *dot_bg = p_instruction_list_layout->indicator_dots_bg[i];
            if (dot_bg == NULL) continue;
            if (!lv_obj_is_valid(dot_bg)) continue;
            if (lv_obj_has_flag(dot_bg, LV_OBJ_FLAG_HIDDEN)) continue;
            lv_area_t a;
            lv_obj_get_coords(dot_bg, &a);
            if (pt.x >= a.x1 && pt.x <= a.x2 && pt.y >= a.y1 && pt.y <= a.y2)
            {
                return dot_bg;
            }
        }
        /* Bottom mic-bar zone: the band's bottom-right overlaps the mic bar /
           input-box footprint, and the arc overlay (brought to front on rebuild)
           sits above the mic_hit there — so a tap in that strip never reaches the
           bar. Forward it to the mic_bar (its CLICKED handler opens / toggles the
           input box), so tapping the bar's right half works like the left half.
           Checked AFTER the dots so a bottom dot still selects its item. */
        if (pt.y >= LV_VER_RES - 90 &&
            p_instruction_list_layout->mic_bar != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->mic_bar))
        {
            return p_instruction_list_layout->mic_bar;
        }
    }
    if (selected_item_index >= list_item_count) return NULL;
    if (touch_obj[selected_item_index] == NULL) return NULL;
    if (!lv_obj_is_valid(touch_obj[selected_item_index])) return NULL;
    if (lv_obj_has_flag(touch_obj[selected_item_index], LV_OBJ_FLAG_HIDDEN)) return NULL;
    lv_area_t a;
    lv_obj_get_coords(touch_obj[selected_item_index], &a);
    if (pt.x < a.x1 || pt.x > a.x2 || pt.y < a.y1 || pt.y > a.y2) return NULL;
    return touch_obj[selected_item_index];
}

static lv_obj_t *list_arc_snap_cb(void *ctx)
{
    (void)ctx;
    /* list 已經在 page change 當下被 snap 過了。如果 drag 結束在 elastic
     * overshoot 區（input 超出 valid 範圍），起一個 anim 把 dot 從當前位置
     * 平滑彈回 idx 對應的 canonical input，看得到回彈動畫 */
    if (s_inst_drag_last_idx >= 0)
    {
        int total = (int)list_item_count;
        if (total > 0)
        {
            int snap_input = 100 * (total - s_inst_drag_last_idx) - 63;
            if (s_inst_drag_input != snap_input)
            {
                inst_start_snap_anim(s_inst_drag_input, snap_input);
            }
        }
    }
    inst_arc_reset_drag_state();
    return NULL;
}

/* EAGER 兜底輪詢(見 instruction_list_init 尾註解):1s 週期 drain 0x6b op 佇列,
   免疫 lvgl_mq 洪流丟 APPLY 通知。空佇列時 apply_pending 內部 head==tail 立即
   return=零成本。deinit 時刪除。
   兼職②:deferred-refresh 逾期補跑 —— debounce 的 550ms one-shot lv_timer 真機
   實測有時不 fire(deferred 之後 ~18s 才被下一個自然 refresh 帶動重繪 = founder
   看到「語音查詢結果殘留、退出再進遲遲不變回 actions」)。這裡每秒檢查:pending
   deferred 存在且距上次成功重繪已 >600ms → 直接補跑(bypass debounce),殘留最多
   ~1s 內更正。 */
static lv_timer_t *s_inst_op_drain_timer = NULL;
static void inst_op_drain_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (search_freeze_active())
        return; /* R60:搜尋期間不套用推播,佇列留著,結束後照常補上 */
    extern void apply_pending_instruction_batch(void);
    apply_pending_instruction_batch();
    if (s_pending_refresh_timer != NULL &&
        (rt_tick_get() - s_last_refresh_tick) > rt_tick_from_millisecond(600))
    {
        LOG_I("deferred refresh overdue — flushing now");
        lv_timer_del(s_pending_refresh_timer);
        s_pending_refresh_timer = NULL;
        s_last_refresh_tick = 0; /* bypass debounce for this catch-up run */
        refresh_custom_instructions();
    }
}

lv_obj_t *lv_instruction_list_layout_create(lv_obj_t *parent)
{
    // 檢查是否已經分配，如果是則先釋放
    if (p_instruction_list_layout != NULL)
    {
        LOG_W("p_instruction_list_layout already exists, cleaning up...");
        instruction_list_deinit();
    }
    size_t allocate_size = sizeof(instruction_list_layout_t);
    p_instruction_list_layout = (instruction_list_layout_t *)lv_mem_alloc(
        sizeof(instruction_list_layout_t));
    if (p_instruction_list_layout == NULL)
    {
        LOG_E("Failed to allocate memory for p_instruction_list_layout");
        return NULL;
    }
    memset(p_instruction_list_layout, 0, sizeof(instruction_list_layout_t));
    memset(switch_objs, 0, sizeof(switch_objs));
    LOG_I("[CHECK_MEMORY]instruction_list_init(%d bytes)", allocate_size);
    instruction_list_page = parent;

    /* 共用 arc_scroll 模組內部自帶 idempotent lock（lock 前先 unlock），
     * 不需要在這邊清 stale 狀態 */

    load_instruction_list();

    /* Global persistent bar layer (lv_layer_top) — hosts the WHOLE instruction
       list (p_instruction_list_bg) plus the mic_bar / ai_box below, so they all
       float above every page. Full-screen but NON-clickable, so only its
       bar/list children take touches and everything else falls through. The
       layer is hidden by default and shown per page by
       instruction_list_bar_set_visible. Created BEFORE the list so the list is at
       the back and the bar/box (added later) stack in front of it. */
    if (s_global_bar_layer && lv_obj_is_valid(s_global_bar_layer))
        lv_obj_del(s_global_bar_layer);
    s_global_bar_layer = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_global_bar_layer);
    lv_obj_set_size(s_global_bar_layer, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_global_bar_layer, 0, 0);
    lv_obj_clear_flag(s_global_bar_layer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_global_bar_layer, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(s_global_bar_layer, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_global_bar_layer, LV_OPA_0, 0);

    /* R3: the list content now lives ON the global bar layer (was the LEFT tile
       in `parent`), so it FLOATS over the current page instead of being a tile
       you scroll to. It starts HIDDEN — a bar tap reveals it
       (animate_open_ai_widget) and the close path hides it again. `parent` (the
       LEFT tile, whose y1 stays 0) is still tracked as instruction_list_page for
       the list's scroll-snap correction; it just no longer hosts the list. */
    lv_obj_t *p_instruction_list_bg = lv_obj_create(s_global_bar_layer);
    p_instruction_list_layout->p_instruction_list_bg = p_instruction_list_bg;
    lv_obj_set_style_bg_opa(p_instruction_list_bg, LV_OPA_0, 0);
    lv_obj_set_size(p_instruction_list_bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(p_instruction_list_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(p_instruction_list_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(p_instruction_list_bg, LV_OBJ_FLAG_HIDDEN);
    /* The horizontal drawer-drag handler ALSO lives on the bg (not only the
       scrollable list) so a drag that starts on a SIBLING-of-the-list element
       parented here — the right-side indicator dots, the centred snapshot image —
       still reaches it (those bubble to this bg, never into the list). The list
       keeps its own copy for presses inside it; the two never double-fire because
       the list doesn't bubble to the bg (only GESTURE does, which is idempotent). */
    lv_obj_add_event_cb(p_instruction_list_bg, list_window_scroll_event_cb,
                        LV_EVENT_ALL, NULL);

    lv_obj_t *p_instruction_list = lv_obj_create(p_instruction_list_bg);
    p_instruction_list_layout->list = p_instruction_list;
    LOG_D("p_instruction_list: %p", p_instruction_list);
    lv_obj_set_size(p_instruction_list, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_opa(p_instruction_list, LV_OPA_0, 0);
    lv_obj_add_flag(p_instruction_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(p_instruction_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p_instruction_list, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(p_instruction_list, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_pad_ver(p_instruction_list, LV_HOR_RES / 2, 0);
    lv_obj_align(p_instruction_list, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(p_instruction_list, list_window_scroll_event_cb,
                        LV_EVENT_ALL, NULL);

    /* Create all list item UI objects (apps + any pre-existing instructions) */
    create_list_items_ui(p_instruction_list, 0, list_item_count);

    /* list_item_count==0 placeholder — the view + white QR card are built once
       and resident (cheap: plain rects, no image data); the actual QR code
       inside the card is created/destroyed lazily by update_list_empty_state()
       (see its comment). Not clickable/scrollable so the swipe-to-close
       gesture on p_instruction_list_bg still fires through it. */
    lv_obj_t *empty_view = lv_obj_create(p_instruction_list_bg);
    lv_obj_remove_style_all(empty_view);
    lv_obj_set_size(empty_view, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(empty_view, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(empty_view, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(empty_view, 16, 0);
    lv_obj_clear_flag(empty_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(empty_view, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(empty_view);
    p_instruction_list_layout->empty_view = empty_view;

    lv_obj_t *empty_qr_card = lv_obj_create(empty_view);
    lv_obj_set_size(empty_qr_card, 181, 181);
    lv_obj_set_style_bg_color(empty_qr_card, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(empty_qr_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(empty_qr_card, 0, 0);
    lv_obj_set_style_radius(empty_qr_card, 16, 0);
    lv_obj_set_style_pad_all(empty_qr_card, 8, 0);
    lv_obj_clear_flag(empty_qr_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(empty_qr_card, LV_OBJ_FLAG_CLICKABLE);
    p_instruction_list_layout->empty_qr_card = empty_qr_card;
    /* Child lv_qrcode created lazily by ensure_empty_qr_created() — left empty here. */

    lv_obj_t *empty_hint_label = lv_label_create(empty_view);
    lv_obj_set_style_text_font(empty_hint_label,
                               LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(empty_hint_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(empty_hint_label, LV_TEXT_ALIGN_CENTER, 0);
    /* English strings (e.g. "Scan to connect your phone") run wider than the
       screen at this font size — wrap instead of overflowing off-screen. */
    lv_obj_set_width(empty_hint_label, 300);
    lv_label_set_long_mode(empty_hint_label, LV_LABEL_LONG_WRAP);
    p_instruction_list_layout->empty_hint_label = empty_hint_label;

    update_list_empty_state();

    // 創建指示點
    create_indicator_dots(p_instruction_list_bg);

    /* 右側弧形觸控滾動 — 用共用模組 common/arc_scroll.h。
     * 放在 ai_bar / ai_bg 之前 → AI 開啟時 ai_bg 蓋在上面，自然停用 */
    arc_scroll_config_t arc_cfg = {
        .parent          = p_instruction_list_bg,
        .list            = p_instruction_list,
        .slot_height_px  = LIST_ITEM_SLOT_HEIGHT,        /* 100 = 200 + (-100) */
        .item_height_px  = LIST_ITEM_WIDGET_HEIGHT,      /* 200，items 互相重疊 100 */
        .slot_angle_deg  = LIST_ITEM_SLOT_ANGLE_DEG,
        .item_count      = list_item_count,
        .band_thickness  = 90,
        .lock_ancestors  = true, /* instruction_list 是 tileview 子層，要鎖外層 */
        .tap_cb          = list_arc_tap_cb,
        .snap_cb         = list_arc_snap_cb,
        .drag_cb         = inst_arc_drag_cb,
        .ctx             = NULL,
    };
    p_instruction_list_layout->arc_handle = arc_scroll_create(&arc_cfg);
    /* DEBUG：顯示 arc band 觸發範圍。確認位置後可以拿掉這行 */
    // arc_scroll_set_debug_visible(p_instruction_list_layout->arc_handle, true);

    lv_obj_t *ai_bar = lv_obj_create(p_instruction_list_bg);
    lv_obj_set_size(ai_bar, 80, LV_VER_RES);
    lv_obj_align(ai_bar, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(ai_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(ai_bar, 0, 0);
    lv_obj_add_event_cb(ai_bar, ai_bar_event_cb, LV_EVENT_ALL, NULL);
    // lv_obj_add_flag(ai_bar, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(ai_bar, LV_OBJ_FLAG_PRESS_LOCK);
    /* Left-edge swipe-to-open DISABLED — the AI input box should only appear
       directly via the mic (like the right device skaibar), not be dragged out
       from the screen edge. */
    lv_obj_add_flag(ai_bar, LV_OBJ_FLAG_HIDDEN);

    /* The bottom-center "+" add-instruction button has been removed — the
       mic-bar voice trigger now owns the bottom-center affordance slot. The
       phone-side create-instruction flow remains available; can be re-added
       to a different location (e.g. settings) if user-facing entry is needed.

    [removed: add_inst_btn block — see git blame for prior 70x50 plus pill] */

    /* Bottom mic bar — slim home-indicator pill IDENTICAL to the right
       device_pager skaibar's bar (100x16 dark fill + white hairline border, no
       glyph). Tap morphs it into the input box. Hidden role passes to the
       widget's own voice button once open. */
    lv_obj_t *mic_bar = lv_obj_create(s_global_bar_layer);
    p_instruction_list_layout->mic_bar = mic_bar;
    lv_obj_set_size(mic_bar, LMIC_W, LMIC_H);
    lv_obj_align(mic_bar, LV_ALIGN_BOTTOM_MID, 0, LMIC_Y);
    lv_obj_set_style_bg_color(mic_bar, lv_color_hex(0x1a1a1a), 0);
    /* Transparent at rest — the skaibar_img child IS the resting look; the dark fill
       only ramps in (lmic_grow_cb) as the bar morphs into the box backdrop. No
       hairline border now: the image supplies the bar's shape; the box's own frame
       image (message_widget_bg) supplies the open-state border. */
    lv_obj_set_style_bg_opa(mic_bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mic_bar, 0, 0);
    lv_obj_set_style_radius(mic_bar, LMIC_RADIUS, 0);
    lv_obj_set_style_pad_all(mic_bar, 0, 0);
    lv_obj_clear_flag(mic_bar, LV_OBJ_FLAG_SCROLLABLE);
    /* Like the watch-face status_bar_area edge zones (which clear PRESS_LOCK so a
       drag transfers to the tileview): a stationary TAP opens the box (CLICKED),
       but a DRAG that leaves the slim pill hands the press DOWN to the bottom
       status_bar_area underneath (which reveals the hidden tileview and
       finger-follows the up-swipe = app list). Without this the bar — being on
       layer_top — would swallow the watch-face bottom-up gesture. */
    lv_obj_clear_flag(mic_bar, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_flag(mic_bar, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_event_cb(mic_bar, mic_bar_event_cb, LV_EVENT_CLICKED, NULL);
    /* 舉起帶出的 session:按住=語音(對講機),放開/滑出=停 */
    lv_obj_add_event_cb(mic_bar, mic_bar_voice_event_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(mic_bar, mic_bar_voice_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(mic_bar, mic_bar_voice_event_cb, LV_EVENT_PRESS_LOST, NULL);
    /* The bar's resting look is now the MIC GLYPH (micro_icon), not the slim skaibar_img bar — matching
       the in-chat voice trigger (founder 2026-06-29). Half-size it (zoom 128) with a CENTRE pivot +
       OVERFLOW_VISIBLE on BOTH the glyph and the bar, so the ezip bitmap scales cleanly and isn't
       clipped to the 31px bar height (a set_size would clip it). Still the s_mic_bar_icon slot, so
       lmic_grow_cb's existing 255->0 img-opa fade dissolves the glyph as the bar morphs into the box.
       Non-clickable so taps fall through to mic_bar / mic_hit (which open the box). */
    lv_obj_add_flag(mic_bar, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    mic_bar_build_icon(mic_bar);

    /* ONE-PIECE tap helper covering the bar + the zone above it (founder 2026-07-06:
       可按區太小 — 長按尤其難:PRESS_LOCK 已清,手指飄出物件就把 press 交出去而中斷;
       舊的「bar 本體 + 上方 40px band」兩件式在交界一跨就斷)。單一 240 x (LMIC_H+56)
       的透明大片、底部對齊 bar 的 BOTTOM(LMIC_Y)向上長 — 仍然不往下進螢幕底緣那
       20px:app-list swipe-up 帶完全保留(這正是當年拆掉對稱 ext_click_area 的原因)。
       蓋住 bar 之後所有 tap/長按都走 mic_hit(CLICKED forward 進 mic_bar_event_cb、
       長按語音 cb 同掛),行為與點 bar 本體一致。PRESS_LOCK cleared so a drag still
       transfers down. Created BEFORE ai_box so the open box covers it (z-order).
       Child of s_global_bar_layer -> chain-deleted with the bar. */
    lv_obj_t *mic_hit = lv_obj_create(s_global_bar_layer);
    s_mic_hit = mic_hit; /* 抽屜三鍵列要把這片大 tap 區一起讓開(見 drawer_row_*) */
    lv_obj_remove_style_all(mic_hit);
    /* founder 2026-07-06 定案:可觸發範圍=原本(216x71 兩件式)的 1.5 倍 → 324x106。 */
    lv_obj_set_size(mic_hit, 324, LMIC_H + 75);
    lv_obj_align(mic_hit, LV_ALIGN_BOTTOM_MID, 0, LMIC_Y); /* bottom = bar BOTTOM;向上長,不進底緣帶 */
    lv_obj_add_flag(mic_hit, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(mic_hit, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(mic_hit, LV_OBJ_FLAG_PRESS_LOCK); /* drag transfers down to the swipe-up */
    lv_obj_add_event_cb(mic_hit, mic_hit_event_cb, LV_EVENT_CLICKED, NULL);
    /* 長按語音在放大的 hit 帶上也要作用(slim bar 難按同理);CLICKED 抑制由
       mic_bar_event_cb 統一處理(mic_hit_event_cb forward 進去)。 */
    lv_obj_add_event_cb(mic_hit, mic_bar_voice_event_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(mic_hit, mic_bar_voice_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(mic_hit, mic_bar_voice_event_cb, LV_EVENT_PRESS_LOST, NULL);

    /* (The above mic_hit grows UPWARD only. The old SEPARATE 240x90 / LBOX_W
       hit-area object, by contrast, was removed: on
       layer_top it covered the whole bottom strip — including the
       status_bar_area_down zone — and swallowed the watch-face "swipe up from the
       bottom" (app list) gesture. The slim mic_bar alone takes taps; a drag falls
       through (PRESS_LOCK cleared above) to the bottom status_bar_area. */

    /* The AI-chat builder (lv_skai_widget_builder) is retained but kept HIDDEN.
       app_skai.c's shared globals (skai_widget_input_text_bg etc.) are
       dereferenced WITHOUT null-guards by set_skai_widget_opa / the re-ask path
       (also called from app_mainmenu), so the objects must exist — but we never
       show it. The visible box below is a clean device_pager-style container,
       so the builder's scrollable AI-reply area (the right-edge SCROLLBAR) and
       the old 2-tile horizontal tileview (the LEFT-SWIPE-off) are both gone from
       the UI. Builder is parented hidden; its globals stay valid. */
    extern lv_obj_t *lv_skai_widget_builder(lv_obj_t * parent);
    /* Parent the builder inside a dedicated HIDDEN host so its whole subtree is
       never shown — and so we never touch p_instruction_list_bg's own
       visibility. (Earlier, hiding lv_obj_get_parent() of the builder's return
       value hid the entire page, because the builder returns its outer
       container, a direct child of p_instruction_list_bg.) */
    lv_obj_t *skai_host = lv_obj_create(p_instruction_list_bg);
    lv_obj_set_size(skai_host, 1, 1);
    lv_obj_set_style_pad_all(skai_host, 0, 0);
    lv_obj_set_style_border_width(skai_host, 0, 0);
    lv_obj_set_style_bg_opa(skai_host, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(skai_host, LV_OBJ_FLAG_HIDDEN);
    lv_skai_widget_builder(skai_host);

    /* Visible AI voice-input box — mirrors the right device_pager skaibar
       (device_pager.c:1148): a plain NON-scrollable container (no tileview)
       holding the message_widget_bg frame + a transcript label. The bottom
       mic_bar morphs into it (lmic_grow_cb); tapping the box toggles it closed;
       scrolling the list dismisses it (animated reverse morph). Reuses the
       p_instruction_list_ai_bg struct field so all open/close/teardown code
       keeps a valid handle. */
    p_instruction_list_layout->p_instruction_list_ai_bg =
        lv_obj_create(s_global_bar_layer);
    lv_obj_t *ai_box = p_instruction_list_layout->p_instruction_list_ai_bg;
    s_skai_widget = ai_box; /* animation key for the frame/label fade */
    lv_obj_set_size(ai_box, LBOX_W, LBOX_H);
    lv_obj_align(ai_box, LV_ALIGN_BOTTOM_MID, 0, LBOX_Y);
    lv_obj_set_style_bg_opa(ai_box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ai_box, 0, 0);
    lv_obj_set_style_pad_all(ai_box, 0, 0);
    lv_obj_set_scrollbar_mode(ai_box, LV_SCROLLBAR_MODE_OFF);
    /* Keep the box SCROLLABLE (scrollbar hidden) AND clear SCROLL_CHAIN so it
       ABSORBS swipes instead of passing them to the page navigation. The frame
       fits the box exactly, so there is no scroll range (the drag rubber-bands,
       nothing moves, no visible bar) and the swipe can no longer bubble up and
       jump to the watchface. A swipe begin dismisses the box
       (ai_box_scroll_dismiss_cb), mirroring device_pager's scroll_hides_skaibar_cb. */
    lv_obj_clear_flag(ai_box, LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_add_flag(ai_box, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ai_box, mic_bar_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ai_box, ai_box_scroll_dismiss_cb, LV_EVENT_SCROLL_BEGIN,
                        NULL);
    lv_obj_add_flag(ai_box, LV_OBJ_FLAG_HIDDEN);
    ai_gaus_bg = NULL; /* gaus glow disabled — all call-sites are guarded no-ops */

    /* Frame image — supplies the box's visible border + rounded shape (LVGL's
       drawn border is too thin on real hw). Centered child of the box, so it
       always tracks the box position. Mirrors device_pager's skaibar_frame. */
    s_pill_bg_img = lv_img_create(ai_box);
    lv_img_set_src(s_pill_bg_img, &message_widget_bg);
    lv_obj_center(s_pill_bg_img);
    lv_obj_clear_flag(s_pill_bg_img, LV_OBJ_FLAG_CLICKABLE);

    /* Transcript label — shows "聽取中" then the spoken text, over the frame.
       Mirrors device_pager's skaibar_label. It lives inside a fixed-height clip
       (exactly two rows tall) so only the latest two wrapped rows show; older rows
       scroll up and can be pulled back down manually. LVGL text height for N rows
       is N*line_height + (N-1)*line_space, so the clip is 2*lh + line_space tall.
       The window is placed one row-pitch (lh + line_space) above the old
       single-line top (y=60), so the newest line still lands at y=60 and the prior
       line sits one row above it. */
    const lv_font_t *vt_font = LV_EXT_FONT_GET(get_system_font_size(0));
    s_voice_transcript_clip = lv_obj_create(ai_box);
    lv_obj_set_style_bg_opa(s_voice_transcript_clip, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_voice_transcript_clip, 0, 0);
    lv_obj_set_style_pad_all(s_voice_transcript_clip, 0, 0);
    lv_obj_set_style_radius(s_voice_transcript_clip, 0, 0);
    lv_obj_set_scrollbar_mode(s_voice_transcript_clip, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(s_voice_transcript_clip, LV_DIR_VER);
    lv_obj_clear_flag(s_voice_transcript_clip, LV_OBJ_FLAG_SCROLL_CHAIN);
    /* Clickable so a drag on the 2-row window scrolls THIS clip (the indev scroll
       search starts at the pressed object and only walks up to parents — if the
       clip weren't the press target the box would scroll instead, and the box has
       no scroll range). A plain tap still toggles the box (own CLICKED handler);
       SCROLL_CHAIN cleared keeps the scroll local so it never trips the box's
       swipe-dismiss (ai_box_scroll_dismiss_cb). */
    lv_obj_add_flag(s_voice_transcript_clip, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_voice_transcript_clip, mic_bar_event_cb, LV_EVENT_CLICKED,
                        NULL);

    s_voice_transcript_label = lv_label_create(s_voice_transcript_clip);
    lv_label_set_text(s_voice_transcript_label, "");
    lv_obj_set_width(s_voice_transcript_label, 360);
    lv_label_set_long_mode(s_voice_transcript_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(s_voice_transcript_label, lv_color_white(), 0);
    lv_obj_set_style_text_opa(s_voice_transcript_label, LV_OPA_80, 0);
    /* CJK-capable font (mirrors device_pager skaibar_label) — the default
       montserrat font renders 聽取中 as tofu boxes. */
    lv_obj_set_style_text_font(s_voice_transcript_label, vt_font, 0);
    lv_obj_set_style_text_align(s_voice_transcript_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_voice_transcript_label, LV_ALIGN_TOP_MID, 0, 0);

    /* Size + place the clip now that the label's font/line-space are known. */
    {
        lv_coord_t lh = lv_font_get_line_height(vt_font);
        lv_coord_t ls =
            lv_obj_get_style_text_line_space(s_voice_transcript_label, LV_PART_MAIN);
        lv_obj_set_size(s_voice_transcript_clip, 360, 2 * lh + ls);
        lv_obj_align(s_voice_transcript_clip, LV_ALIGN_TOP_MID, 0, 60 - (lh + ls));
    }

    /* No separate voice button — the box matches device_pager (frame + label
       only). The VAD-pulse / re-ask handlers are all null-guarded, so leaving
       these NULL makes them no-op. */
    ai_voice_btn = NULL;
    s_voice_img = NULL;
    ai_voice_send_icon = NULL;
    // 創建可移動範圍圓弧線
    // create_movable_range_arc(p_instruction_list_bg);

    /* Right-edge reveal overlay (L/R swap): a thin clickable strip on the screen's
       right edge, in front of the list, that finger-reveals the floating list on a
       leftward pull (watch face only; gated by check_is_at_home). Child of
       s_global_bar_layer so it chain-deletes with the bar. Created LAST so it sits
       in front; move_foreground is belt-and-suspenders. Starts hidden (disabled). */
    s_reveal_edge_overlay = lv_obj_create(s_global_bar_layer);
    lv_obj_remove_style_all(s_reveal_edge_overlay);
    lv_obj_set_size(s_reveal_edge_overlay, LREVEAL_EDGE_W, LV_VER_RES);
    lv_obj_align(s_reveal_edge_overlay, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_clear_flag(s_reveal_edge_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_reveal_edge_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_reveal_edge_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_reveal_edge_overlay, reveal_edge_overlay_event_cb,
                        LV_EVENT_ALL, NULL);
    lv_obj_move_foreground(s_reveal_edge_overlay);

    /* P2 S3 — left-edge reveal overlay (mirror of the right): a rightward pull from
       the left edge opens the mixed-list browse view. Same gating as the right
       (check_is_at_home → instruction_list_reveal_overlay_set_enabled).
       2026-07-02: widened from LREVEAL_EDGE_W (20px) to the full 58px the left
       device_pager tileview handle (status_bar_area_left) used to own — that handle
       is dead now (device_pager launches as an app-list item, not a drag-out tile,
       per the P3 note above), so the 20-58px band was just an unresponsive dead
       strip. Only the LEFT overlay grows; the right stays LREVEAL_EDGE_W since its
       58px band still belongs to the live App List tileview handle. */
    s_reveal_edge_overlay_left = lv_obj_create(s_global_bar_layer);
    lv_obj_remove_style_all(s_reveal_edge_overlay_left);
    lv_obj_set_size(s_reveal_edge_overlay_left, LV_HOR_RES >> 3, LV_VER_RES);
    lv_obj_align(s_reveal_edge_overlay_left, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_clear_flag(s_reveal_edge_overlay_left, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_reveal_edge_overlay_left, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_reveal_edge_overlay_left, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_reveal_edge_overlay_left,
                        reveal_edge_overlay_left_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_move_foreground(s_reveal_edge_overlay_left);

    created = true;
    LOG_I("instruction_list_init: before myLancher reset_list");
    myLancher[app_index_instruction_list].reset_list = reset_list;
    if (myLancher[app_index_instruction_list].reset_list != NULL)
    {
        myLancher[app_index_instruction_list].reset_list();
    }
    LOG_I("instruction_list_init: before lv_event_send SCROLL");
    lv_event_send(p_instruction_list, LV_EVENT_SCROLL, NULL);
    LOG_I("instruction_list_init: after lv_event_send SCROLL");

    myLancher[app_index_instruction_list].on_tap = on_tap;
    /* 結欠帳:開機早期(BLE 比 GUI 早起 ~60s)到達的 0x6b 批次已排進 op 佇列,但其
       APPLY_INSTRUCTION_BATCH 通知常被塞滿的 lvgl_mq 丟掉(serial 實證 type 74
       dropped)→ 批次永不套用、清單空,而手機 dedup 認定已送不重推。清單建好的
       此刻直接 drain 一次(冪等,佇列空=立即 return)。LVGL thread ✓ */
    {
        extern void apply_pending_instruction_batch(void);
        apply_pending_instruction_batch();
    }
    /* EAGER 兜底(終局):不再信任 msg 通知的可達性 —— 連線窗的 msg 洪流(通知
       resync 15/75、device sync 76)把 lvgl_mq 塞滿時,APPLY(74)在「發送端」就被
       丟,接收端任何 debounce/加深都救不到(2026-07-25 冷開機 serial 實證:mq 32
       仍 full、74 連丟)。掛一支 1s periodic lv_timer 直接輪詢 op 佇列:空佇列一次
       檢查=兩個指標比較(零成本);有欠帳=最遲 1s 內套用+重繪 → 手錶連上手機後
       幾秒清單就緒,使用者進來前就是同步好的(founder 的 eager 要求)。 */
    if (s_inst_op_drain_timer == NULL)
    {
        s_inst_op_drain_timer = lv_timer_create(inst_op_drain_tick_cb, 1000, NULL);
    }
    LOG_I("instruction_list_init: returning");

#ifdef TEST_INDICATOR_ANIMATION
    start_indicator_dots_animation_test(); // 開始指示點動畫測試
#endif
    return p_instruction_list_bg;
}

// static rt_uint32_t last_scroll_time = 0;
static void scroll_list_to_index(uint16_t page, bool animate)
{
    if (p_instruction_list_layout->list == NULL ||
        !lv_obj_is_valid(p_instruction_list_layout->list))
    {
        LOG_E("p_instruction_list_layout->list is NULL");
        return;
    }

    app_scroll_target_item = page;
    LOG_D("scroll_list_to_index: %d", page);
    lv_obj_t *child = lv_obj_get_child(p_instruction_list_layout->list, page);
    if (!lv_obj_is_valid(child))
    {
        LOG_W("scroll_list_to_index: child %d is invalid", page);
        return;
    }
    /* drag_cb 模式下 scroll_list 的 selected_item_index 自動更新被 gate 擋住，
     * 這邊先手動 set 起來，等下 scroll_list call 跑到 line 939 那段
     * (selected != old) 的可見性 loop 才會把新的 widget/label show 出來、舊的藏起來。
     * motion 路徑（NAV_BAR_CONTROL）也共用這個函式，多 set 一次同樣 idempotent */
    if (page < list_item_count)
    {
        selected_item_index = page;
    }
    // lv_disp_trig_activity(NULL);
    // set_scroll_anim_time(true);
    /* animate=false（arc 圓形滾動）：即時捲，清單每次 page change 立刻跟上圖示，
       不留 200ms 半截動畫給 phone-push refresh 打斷 → 放開本來就在位、沒有半路
       狀態可被看到（founder 2026-07-23：治本，取代放開後才拉回的治標做法）。
       animate=true（點指示點跳 app 等）：保留原本的平滑動畫。 */
    lv_obj_scroll_to_view(child, animate ? LV_ANIM_ON : LV_ANIM_OFF);
    LOG_D("scroll_list_to_index done: %d", page);
    // set_scroll_anim_time(false);
    scroll_list(p_instruction_list_layout->list, 0);
}

static void instruction_list_scroll_to_app(int8_t action)
{
    if (pause_instruction_list)
    {
        return;
    }

    if (action >= 0 && action < list_item_count)
    {
        scroll_list_to_index(action, true);
    }

    else
    {
        LOG_W("Target index out of bounds: %d", action);
    }
}

/* Before any phone list arrives, the left mixed list shows NO items (founder
   direction: the built-in apps this used to fall back to — timer / flashlight
   / recorder / exercise / sleep / calculator / weather / alarm / photo / mouse
   — are all also in APP_LIST_ITEMS (lv_app_list_layout.c, the right-swipe App
   List), so this was a pure duplicate entry point). Leaving list_items[] empty
   here makes list_item_count==0, which drives update_list_empty_state()'s "add
   on phone" hint / pairing QR — replacing the old app-list fallback. */
static void load_default_apps(void)
{
    list_item_count = app_base_count; /* == 0: no pinned prefix */
    s_list_mode = LIST_MODE_DEFAULT_APPS;
}

/* An item the watch can act on with no phone: a built-in app (Settings / default
   apps, is_instruction == false) or an "open watch app" instruction (openApp
   set). The rest are phone-relay instructions, hidden while disconnected. */
static bool item_is_standalone(const list_item_t *it)
{
    return (!it->is_instruction) || (it->open_app[0] != '\0');
}

/* True if item passes a STRICT category view filter — only ever called for
   cat == '@' or cat == '/' (instruction_list_set_category_filter guards cat==0
   out before reaching this). 'all' (cat==0) shows the full unfiltered list,
   @ items included — the left-swipe skaibar reveal from the bare watch face
   is this mixed list. */
static bool item_matches_cat(const list_item_t *it, char cat)
{
    return it->category == cat;
}

/* Restore the full list if a category view filter is active (no refresh — the
   caller refreshes, or the list is hidden). cat filter is the INNER layer: this
   runs before the disconnect-filter restore so the nesting unwinds correctly. */
static void cat_filter_restore_full(void)
{
    if (s_cat_backup_valid)
    {
        memcpy(list_items, s_cat_backup, sizeof(list_items));
        list_item_count = s_cat_backup_count;
        s_cat_backup_valid = false;
    }
    s_cat_filter = 0;
    /* R62(founder:「為什麼他一直當機?」):R59 曾在這裡直接重新打包,好讓陣列與畫面一致 ——
       但這個函式會被非 refresh 的路徑呼叫,於是**在沒有重建 UI 的情況下改動了清單長度**。
       右緣圓框(indicator_dots[])、開關(switch_objs[])這些物件陣列是照舊長度建的,其他
       程式碼卻照新長度迭代 → 取到已不存在的物件指標。清單長度只能在 refresh 裡跟著 UI
       一起變。R60 的「搜尋期間凍結推播」已經從源頭擋掉那個不一致,這裡不必再打包。 */
}

/* Apply (cat '@'/'/') or clear (cat 0) the view filter and rebuild the UI.
   Re-packs list_items[] to the matching subset over a full-list snapshot, keeping
   the pinned prefix — mirrors instruction_list_set_phone_connected so scroll /
   tap / indicator-dots keep working on the shorter contiguous array unchanged. */
/* R55:大小寫不敏感的子字串比對。中文沒有大小寫,UTF-8 位元組序列直接比就對了;
   英文 action 名稱則要忽略大小寫,不然「打開 Spotify」講成小寫就篩不到。 */
static bool text_contains_ci(const char *hay, const char *needle)
{
    if (needle == NULL || needle[0] == '\0')
        return true;
    if (hay == NULL || hay[0] == '\0')
        return false;
    for (const char *h = hay; *h; h++)
    {
        const char *a = h, *b = needle;
        while (*a && *b)
        {
            char ca = *a, cb = *b;
            if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
            if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
            if (ca != cb)
                break;
            a++; b++;
        }
        if (*b == '\0')
            return true;
    }
    return false;
}

/* R56(founder 第二輪:「測試功能/111/123++ 都還在,還多了一堆問 SKAI」;log 證明
   `pack in=5 out=1` 有跑、畫面卻沒變):在「設定篩選」那一刻改陣列是錯的層級 ——
   手機每 5 秒 replace-all 一次,那條路自己會 `cat_filter_restore_full()` 把全清單放回來
   再重建。跟一條持續重寫同一個陣列的資料流搶,只會補完一個路徑漏下一個。
   改成在**所有重建的共同出口** refresh_custom_instructions 裡套用:不管內容是誰重建的、
   走哪條路徑,畫出來之前都會先被篩過一次。 */
static void pack_text_filter(void)
{
    if (s_text_filter[0] == '\0')
        return;
    if (!s_cat_backup_valid)
    {
        memcpy(s_cat_backup, list_items, sizeof(list_items));
        s_cat_backup_count = list_item_count;
        s_cat_backup_valid = true;
    }
    /* R77(founder:「有出現篩選了,但為什麼 new session 還在?」):先數這輪真正
       比中的項目 —— 有比中就不該出現任何「開新對話」列(先前合成的、手機推來的
       「問 SKAI」列一律丟),沒比中才補。 */
    uint8_t real_matches = 0;
    for (uint8_t r = 0; r < list_item_count; r++)
    {
        if (strncmp(list_items[r].id, NEW_SESSION_ITEM_ID,
                    sizeof(NEW_SESSION_ITEM_ID) - 1) == 0)
            continue;
        if (strstr(list_items[r].title, "SKAI") != NULL ||
            strstr(list_items[r].title, "Skai") != NULL)
            continue;
        if (text_contains_ci(list_items[r].title, s_text_filter))
            real_matches++;
    }
    uint8_t w = 0;
    bool has_new_session_row = false;
    for (uint8_t r = 0; r < list_item_count; r++)
    {
        /* R63(founder:「為什麼他一直當機?」;log:系統還活著、只有畫面卡住):合成的
           「開新對話」列標題含有搜尋字,所以**下一次打包時它自己也會通過比對留下來**,
           然後下面又補一整組 → 每次刷新多 N 列,很快撐到 MAX_LIST_ITEMS,GUI 執行緒被
           反覆重建整份清單卡死(也就是先前看到的「一堆」選項)。認出既有的合成列、原樣
           留下並標記已存在,就不會再追加。 */
        if (strncmp(list_items[r].id, NEW_SESSION_ITEM_ID,
                    sizeof(NEW_SESSION_ITEM_ID) - 1) == 0)
        {
            /* R77:這輪有真比中 → 先前合成的「開新對話」列不再保留。 */
            if (real_matches > 0)
                continue;
            if (w != r)
                memcpy(&list_items[w], &list_items[r], sizeof(list_item_t));
            has_new_session_row = true;
            w++;
            continue;
        }
        /* 手機 launcher 鏡像在有查詢字時會推「問 SKAI」列。左頁 session 檢視裡那一列的
           意思就是開新對話 → 改寫成新 session 那一項,並且只留一列(founder:「多了一堆
           問 SKAI」)。認 title 前綴:那列由手機組字、id 就等於 title,沒有別的識別。 */
        if (strstr(list_items[r].title, "SKAI") != NULL ||
            strstr(list_items[r].title, "Skai") != NULL)
        {
            /* R77:有真比中 → 手機推來的「問 SKAI」列直接丟,不轉成開新對話。 */
            if (real_matches > 0 || has_new_session_row)
                continue;
            if (w != r)
                memcpy(&list_items[w], &list_items[r], sizeof(list_item_t));
            memset(list_items[w].id, 0, sizeof(list_items[w].id));
            strncpy(list_items[w].id, NEW_SESSION_ITEM_ID, sizeof(list_items[w].id) - 1);
            snprintf(list_items[w].title, sizeof(list_items[w].title), "%s %s",
                     LV_EXT_STR_GET_BY_KEY(new_session, "New session"), s_text_filter);
            has_new_session_row = true;
            w++;
            continue;
        }
        if (!text_contains_ci(list_items[r].title, s_text_filter))
            continue;
        if (w != r)
            memcpy(&list_items[w], &list_items[r], sizeof(list_item_t));
        w++;
    }
    if (real_matches == 0 && !has_new_session_row)
    {
        /* R61:一個都沒中 → **每一台已知桌面各一列**「開新對話」,右緣圓框放設備名
           (create_indicator_dots 會依 id 前綴去查),使用者自己挑開在哪台。設備 id 直接
           編在 item id 後面,啟動時解出來用,不必另外存對照表。一台設備都沒有時仍給一列
           (id 不帶設備),由 pager 的預設目標決定 —— 總要有一條出路。 */
        extern int session_list_device_count(void);
        extern const char *session_list_device_id_at(int i);
        int dev_n = session_list_device_count();
        if (dev_n <= 0)
        {
            if (w < MAX_LIST_ITEMS)
            {
                memset(&list_items[w], 0, sizeof(list_item_t));
                strncpy(list_items[w].id, NEW_SESSION_ITEM_ID, sizeof(list_items[w].id) - 1);
                snprintf(list_items[w].title, sizeof(list_items[w].title), "%s %s",
                         LV_EXT_STR_GET_BY_KEY(new_session, "New session"), s_text_filter);
                w++;
            }
        }
        else
        {
            for (int d = 0; d < dev_n && w < MAX_LIST_ITEMS; d++)
            {
                const char *did = session_list_device_id_at(d);
                if (did == NULL || did[0] == '\0')
                    continue;
                memset(&list_items[w], 0, sizeof(list_item_t));
                snprintf(list_items[w].id, sizeof(list_items[w].id), "%s:%s",
                         NEW_SESSION_ITEM_ID, did);
                snprintf(list_items[w].title, sizeof(list_items[w].title), "%s %s",
                         LV_EXT_STR_GET_BY_KEY(new_session, "New session"), s_text_filter);
                w++;
            }
        }
    }
    LOG_W("[flt] pack in=%u out=%u", (unsigned)list_item_count, (unsigned)w);
    list_item_count = w;
    if (selected_item_index >= list_item_count)
        selected_item_index = list_item_count ? (list_item_count - 1) : 0;
}

/* R75(founder:說「語音辨識」只得到 new-session 列,「語音辨識測試」沒被留下;log:
   [flt] set 了 15 bytes = 「語音辨識。」):STT 轉錄尾端帶標點(全形「。」等),拿去做
   子字串比對永遠比不中任何標題。設定篩選字前先剝掉頭部空白與尾端標點(ASCII + 常見
   全形)。同一份字串也是開新對話的第一句,剝掉尾標點無害。 */
static void text_filter_sanitize(char *s)
{
    /* leading spaces */
    size_t lead = 0;
    while (s[lead] == ' ' || s[lead] == '\t')
        lead++;
    if (lead)
        memmove(s, s + lead, strlen(s + lead) + 1);
    /* trailing ASCII punctuation/space + full-width punctuation (3-byte UTF-8) */
    static const char *fw_punct[] = {
        "\xE3\x80\x82" /* 。 */, "\xEF\xBC\x8C" /* , */, "\xEF\xBC\x81" /* ! */,
        "\xEF\xBC\x9F" /* ? */, "\xE3\x80\x81" /* 、 */, "\xEF\xBC\x9B" /* ; */,
        "\xEF\xBC\x9A" /* : */,
    };
    size_t len = strlen(s);
    for (;;)
    {
        if (len == 0)
            break;
        char c = s[len - 1];
        if (c == '.' || c == ',' || c == '!' || c == '?' || c == ';' ||
            c == ':' || c == ' ' || c == '\t')
        {
            len--;
            continue;
        }
        bool stripped = false;
        if (len >= 3)
        {
            for (size_t i = 0; i < sizeof(fw_punct) / sizeof(fw_punct[0]); i++)
            {
                if (memcmp(s + len - 3, fw_punct[i], 3) == 0)
                {
                    len -= 3;
                    stripped = true;
                    break;
                }
            }
        }
        if (!stripped)
            break;
    }
    s[len] = '\0';
}

/* R55:文字篩選層 —— 由輸入框的轉錄驅動。空字串 = 清掉,回到原本的清單。 */
void instruction_list_set_text_filter(const char *text)
{
    char next[sizeof(s_text_filter)];
    next[0] = '\0';
    if (text != NULL)
    {
        strncpy(next, text, sizeof(next) - 1);
        next[sizeof(next) - 1] = '\0';
        text_filter_sanitize(next);
    }
    if (strcmp(next, s_text_filter) == 0)
        return; /* 轉錄會重複推同一串:沒變就別重繪 */
    strncpy(s_text_filter, next, sizeof(s_text_filter) - 1);
    s_text_filter[sizeof(s_text_filter) - 1] = '\0';
    LOG_W("[flt] set \"%s\" (cat=%d vis=%d)", s_text_filter, (int)s_cat_filter,
          (int)instruction_list_is_visible());
    /* 清掉篩選 → 先把完整清單放回來;有篩選 → refresh 出口會套用(pack_text_filter)。 */
    if (s_text_filter[0] == '\0')
        cat_filter_restore_full();
    refresh_custom_instructions();
}

const char *instruction_list_text_filter(void)
{
    return s_text_filter;
}

static void instruction_list_set_category_filter(char cat)
{
    /* Always re-apply + rebuild (no early-return on an unchanged filter): a close
       restores list_items[] WITHOUT refreshing the UI, so the widgets can be stale
       even when s_cat_filter already matches. Called once per reveal (drag_begin),
       so the redundant rebuild is not a hot path. */
    cat_filter_restore_full(); /* lift any current filter back to the full list */
    s_cat_filter = cat;
    s_view_cat = cat; /* remember the view across phone pushes (see instruction_list_reapply_view_filter) */
    if (cat != 0)
    {
        /* Snapshot the full list into the static backup, then re-pack list_items[] to
           the matching subset. No malloc — the filter always runs (see the buffer's
           declaration note: the old per-reveal rt_malloc could fail and silently skip
           filtering, leaving every view full). */
        memcpy(s_cat_backup, list_items, sizeof(list_items));
        s_cat_backup_count = list_item_count;
        s_cat_backup_valid = true;
        uint8_t w = 0;
        for (uint8_t r = 0; r < list_item_count; r++)
        {
            if (r < app_base_count ||
                item_matches_cat(&list_items[r], cat))
            {
                if (w != r)
                    memcpy(&list_items[w], &list_items[r],
                           sizeof(list_item_t));
                w++;
            }
        }
        list_item_count = w;
    }
    if (selected_item_index >= list_item_count)
        selected_item_index = list_item_count ? (list_item_count - 1) : 0;
    refresh_custom_instructions();
}

/* Re-apply the user's current @/-view after a phone push lifted the filter. The push
   path (add_or_update_custom_instruction → cat_filter_restore_full) clears s_cat_filter so
   updates resolve against the full list; without re-applying, a push that arrives while a
   filtered (@ / /) list is OPEN leaves the full list on screen. apply_pending_instruction_batch
   calls this once after draining the op queue. No-op (returns false) when no filtered view is
   open — the caller then does its own plain refresh. */
bool instruction_list_reapply_view_filter(void)
{
    /* R55(founder:「session 測試功能/111/123++ 都還是出現」):手機的 replace-all 會把
       list_items[] 整份換掉、並經 cat_filter_restore_full 把篩選解除 —— 使用者正在用語音
       篩清單時,每 5 秒一次的推播就把篩掉的東西全放回來。文字篩選跟 @/ 檢視一樣要在推播
       之後重新套用。 */
    LOG_W("[flt] reapply text=\"%s\" cat=%d vis=%d", s_text_filter, (int)s_view_cat,
          (int)instruction_list_is_visible());
    if (s_text_filter[0] != '\0' && instruction_list_is_visible())
    {
        instruction_list_set_category_filter(s_view_cat);
        return true;
    }
    if (s_view_cat != 0 && instruction_list_is_visible())
    {
        instruction_list_set_category_filter(s_view_cat); /* re-packs + refreshes */
        return true;
    }
    return false;
}

void load_instruction_list(void)
{
    /* No pinned prefix: the Settings pin at index 0 was removed (founder
       direction 2026-07-02 — the left list should not always lead with
       Settings; it stays reachable from the right-swipe App List). Every
       "r < app_base_count" pinned-prefix guard degrades to a no-op at 0. */
    app_base_count = 0;

    /* DEFAULT_APPS: (re)build the built-in app body so a language change picks up
       new translations (load_instruction_list is the translation-reload hook).
       PHONE: items are phone instructions carrying their own titles — leave them
       so a reload does not clobber the phone's list. */
    if (s_list_mode == LIST_MODE_DEFAULT_APPS)
        load_default_apps();
}

/* Device page bracket: device_pager reuses this shared list (save_base → feed →
   restore_base) to show a REMOTE device's options, which are 0-based with no
   pinned prefix. A no-op since the watch-face Settings pin was removed
   (app_base_count is already 0); kept so the device-page bracket stays explicit
   about the zero-prefix invariant it needs. */
void instruction_list_drop_pinned_for_device(void)
{
    app_base_count = 0;
}

/* Mirror the BT link state into the list (called from the connection hook on the
   LVGL thread). Only PHONE mode reacts: on disconnect we snapshot the full list
   and re-pack to the standalone items the watch can run alone (apps + openApp
   instructions), hiding the phone-relay ones; on reconnect we restore the full
   list from the snapshot — no dependency on the phone re-pushing. App mode never
   filters (every built-in app runs offline). */
void instruction_list_set_phone_connected(bool connected)
{
    if (connected == s_phone_connected)
        return;
    s_phone_connected = connected;

    /* Drop any transient category view first so the (dis)connect re-pack below
       operates on the real list; the view is re-derived on the next open. */
    cat_filter_restore_full();

    if (s_list_mode != LIST_MODE_PHONE)
    {
        /* DEFAULT_APPS mode has no phone-pushed list to re-pack, but a
           list_item_count==0 screen still needs to flip between the
           "add on phone" hint and the pairing QR as the link comes up/down. */
        update_list_empty_state();
        return;
    }

    if (!connected)
    {
        if (s_disc_backup == NULL)
        {
            s_disc_backup = rt_malloc(sizeof(list_items));
            if (s_disc_backup)
            {
                memcpy(s_disc_backup, list_items, sizeof(list_items));
                s_disc_backup_count = list_item_count;
            }
        }
        /* Re-pack in place: keep the pinned prefix + standalone items, in order. */
        uint8_t w = 0;
        for (uint8_t r = 0; r < list_item_count; r++)
        {
            if (r < app_base_count || item_is_standalone(&list_items[r]))
            {
                if (w != r)
                    memcpy(&list_items[w], &list_items[r], sizeof(list_item_t));
                w++;
            }
        }
        list_item_count = w;
    }
    else if (s_disc_backup)
    {
        memcpy(list_items, s_disc_backup, sizeof(list_items));
        list_item_count = s_disc_backup_count;
        rt_free(s_disc_backup);
        s_disc_backup = NULL;
    }

    if (selected_item_index >= list_item_count)
        selected_item_index = list_item_count ? (list_item_count - 1) : 0;

    refresh_custom_instructions();
}


rt_int32_t instruction_list_resume(void)
{
    if (pause_instruction_list == false)
    {
        return RT_EOK;
    }
    open_gesture_control = false;
    // if (get_need_open_gesture_control())
    {
        set_paused_control_with_arm(false);
        open_gesture_control = true;
        // switch_watch_motion_control_mode(true, true);
        set_free_control_with_arm(true);
    }
    extern void reset_speech_coding(void);
    reset_speech_coding();
    pause_instruction_list = false;
    lvgl_msg_handler.handle_nav_bar_control = instruction_list_scroll_to_app;
    LOG_I("instruction_list_resume");
    /* 使用者正要看清單 → 先把 op 佇列裡的欠帳批次結清(APPLY 通知可能曾被
       lvgl_mq full 丟掉,見 instruction_list_init 尾同款註解)。冪等零成本。 */
    {
        extern void apply_pending_instruction_batch(void);
        apply_pending_instruction_batch();
    }
#ifdef USE_QUICK_OPEN_AI
    open_vibration = true;
#endif
    // lvgl_msg_handler.handle_widgets_control = button_selection;
    return RT_EOK;
}

rt_int32_t instruction_list_pause(void)
{
    if (pause_instruction_list == true)
    {
        return RT_EOK;
    }
    pause_instruction_list = true;
    set_paused_control_with_arm(true);
    /* Leaving the instruction_list page ends the SKAIBAR option-tracking
       session. Phone-side highlight stops following watch scroll until
       the user re-enters this page and taps the mic bar again. */
    s_skaibar_tracking_active = false;
    LOG_I("instruction_list_pause");
    if (gui_app_is_actived("Main"))
    {
        if (lvgl_msg_handler.handle_nav_bar_control ==
            instruction_list_scroll_to_app)
        {
            lvgl_msg_handler.handle_nav_bar_control = NULL;
        }
        lvgl_msg_handler.handle_tap_indicator = NULL;
        lvgl_msg_handler.handle_nav_bar_control = NULL;
    }
    return RT_EOK;
}

rt_int32_t instruction_list_deinit(void)
{
    /* EAGER 兜底輪詢隨清單生命週期走(create 建/deinit 刪)。 */
    if (s_inst_op_drain_timer)
    {
        lv_timer_del(s_inst_op_drain_timer);
        s_inst_op_drain_timer = NULL;
    }
    // 清理 touching_screen 定時器
    if (touching_screen_timer)
    {
        rt_timer_stop(touching_screen_timer);
        rt_timer_delete(touching_screen_timer);
        touching_screen_timer = NULL;
    }

#ifdef USE_QUICK_OPEN_AI
    if (timer_open_quick_app)
    {
        rt_timer_stop(timer_open_quick_app);
        rt_timer_delete(timer_open_quick_app);
        timer_open_quick_app = RT_NULL;
    }
#endif
    if (myLancher[app_index_instruction_list].reset_list != NULL)
    {
        myLancher[app_index_instruction_list].reset_list = NULL;
    }

    if (selected_widget_timer)
    {
        rt_timer_stop(selected_widget_timer);
        rt_timer_delete(selected_widget_timer);
        selected_widget_timer = NULL;
    }
    // 停止所有動畫並重置狀態
    stop_all_animations_and_reset();

    /* P3:list_items 的 malloc 快照(s_base_items=device-page save_base、s_disc_backup=手機斷線
       snapshot)都是「當前清單的副本(含指標)」。清單被拆(deinit)時這些快照必然懸空 —— 必須一併
       丟掉,否則重建後對應的 restore(restore_base / 重連 restore)會把舊快照 memcpy 回新清單 →
       refresh 時 deref 野指標 bus fault(實機:滑鼠 app cycle 後回錶盤拉左 @ 列表崩,即 s_base_items)。
       原本只有各自的 restore 路徑釋放,但 standalone 退出走 notify-only / Main 重建會跳過 → 在此兜底。 */
    if (s_base_items)
    {
        rt_free(s_base_items);
        s_base_items = NULL;
    }
    if (s_disc_backup)
    {
        rt_free(s_disc_backup);
        s_disc_backup = NULL;
    }

    if (p_instruction_list_layout)
    {
        // 銷毀所有項目
        for (uint8_t i = 0; i < MAX_LIST_ITEMS; i++)
        {
            if (p_instruction_list_layout->indicator_dots[i] != NULL &&
                lv_obj_is_valid(p_instruction_list_layout->indicator_dots[i]))
            {
                lv_obj_del(p_instruction_list_layout->indicator_dots[i]);
                p_instruction_list_layout->indicator_dots[i] = NULL;
            }
            if (p_instruction_list_layout->indicator_dots_bg[i] != NULL &&
                lv_obj_is_valid(
                    p_instruction_list_layout->indicator_dots_bg[i]))
            {
                lv_obj_del(p_instruction_list_layout->indicator_dots_bg[i]);
                p_instruction_list_layout->indicator_dots_bg[i] = NULL;
            }
            if (app_widget[i] != NULL && lv_obj_is_valid(app_widget[i]))
            {
                lv_obj_del(app_widget[i]);
                app_widget[i] = NULL;
            }
            if (touch_obj[i] != NULL && lv_obj_is_valid(touch_obj[i]))
            {
                lv_obj_del(touch_obj[i]);
                touch_obj[i] = NULL;
            }
            if (app_icon[i] != NULL && lv_obj_is_valid(app_icon[i]))
            {
                lv_obj_del(app_icon[i]);
                app_icon[i] = NULL;
            }
            if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
            {
                lv_obj_del(app_label[i]);
                app_label[i] = NULL;
            }
            if (p_instruction_list_layout->p_app_indicator_btn[i] != NULL &&
                lv_obj_is_valid(
                    p_instruction_list_layout->p_app_indicator_btn[i]))
            {
                lv_obj_del(p_instruction_list_layout->p_app_indicator_btn[i]);
                p_instruction_list_layout->p_app_indicator_btn[i] = NULL;
                LOG_D("instruction list Deleted p_app_indicator_btn %d", i);
            }
            switch_objs[i] = NULL;
        }

        // 銷毀可移動範圍圓弧線
        if (p_instruction_list_layout->movable_range_arc != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->movable_range_arc))
        {
            lv_obj_del(p_instruction_list_layout->movable_range_arc);
            p_instruction_list_layout->movable_range_arc = NULL;
            LOG_D("instruction list Deleted movable range arc");
        }

        if (p_instruction_list_layout->list != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->list))
        {
            lv_obj_del(p_instruction_list_layout->list);
            p_instruction_list_layout->list = NULL;
        }
        /* Global bar layer (lv_layer_top): chain-deletes the mic_bar / hit area /
           ai_box hosted in it, so the bar doesn't linger on layer_top after the
           page is torn down. Done before the ai_bg block below (whose individual
           del then no-ops via lv_obj_is_valid, the object already gone). */
        /* bar 長按語音若還在跑,先停乾淨(mic/BLE/v2t 狀態),再拆 UI。 */
        mic_bar_voice_stop();
        s_opened_by_lift = false;
        s_mic_lp_consumed = false;
        if (s_global_bar_layer != NULL && lv_obj_is_valid(s_global_bar_layer))
        {
            lv_obj_del(s_global_bar_layer);
        }
        s_global_bar_layer = NULL;
        s_reveal_edge_overlay = NULL; /* chain-deleted with the bar layer above */
        s_reveal_edge_overlay_left = NULL; /* P2 S3: same — chain-deleted */
        s_mic_ripple = NULL; /* mic_bar 子物件,已隨層鏈刪 */
        if (p_instruction_list_layout->p_instruction_list_bg != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->p_instruction_list_bg))
        {
            lv_obj_del(p_instruction_list_layout->p_instruction_list_bg);
            p_instruction_list_layout->p_instruction_list_bg = NULL;
        }
        if (p_instruction_list_layout->p_instruction_list_ai_bg != NULL &&
            lv_obj_is_valid(
                p_instruction_list_layout->p_instruction_list_ai_bg))
        {
            lv_obj_del(p_instruction_list_layout->p_instruction_list_ai_bg);
            p_instruction_list_layout->p_instruction_list_ai_bg = NULL;
        }
        if (p_instruction_list_layout->p_instruction_list_ai_icon != NULL &&
            lv_obj_is_valid(
                p_instruction_list_layout->p_instruction_list_ai_icon))
        {
            lv_obj_del(p_instruction_list_layout->p_instruction_list_ai_icon);
            p_instruction_list_layout->p_instruction_list_ai_icon = NULL;
        }

        if (p_instruction_list_layout->app_list_tileview != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->app_list_tileview))
        {
            lv_obj_del(p_instruction_list_layout->app_list_tileview);
            p_instruction_list_layout->app_list_tileview = NULL;
            p_instruction_list_layout->app_list_tile = NULL;
        }

        lv_mem_free(p_instruction_list_layout);
        p_instruction_list_layout = NULL;
        LOG_I("[CHECK_MEMORY]instruction_list_deinit");
    }

    // extern void media_widget_stop(void);
    // media_widget_stop();
    // extern void activity_widget_stop(void);
    // activity_widget_stop();
    // extern void message_widget_stop(void);
    // message_widget_stop();
    // extern void calendar_widget_stop(void);
    // calendar_widget_stop();
    // extern void note_widget_stop(void);
    // note_widget_stop();

    LOG_I("instruction_list_deinit");
    pause_instruction_list = true;
    return RT_EOK;
}

/* ── R32:列 UI 的釋放 / 重建 ────────────────────────────────────────────────
   浮動清單的 LVGL 物件在錶盤閒置時也一直常駐(它只是 HIDDEN),每一列要
   item/touch/label/dot_bg/frame/icon 好幾個 —— session 注入後列數翻倍,heap 只剩
   ~40KB,進 hosted 滑鼠頁那一大塊配置就失敗 → lv_obj_create 回 NULL →
   lv_obj_class_init_obj 解 NULL 父物件 hard fault(founder 2026-08-12「從媒體頁往上
   滑進滑鼠頁就當機」,連兩版都當)。release 只拆「列的 UI」,資料 list_items[] 不動,
   下次開清單前 ensure 走 refresh 原路重建。 */
static bool s_list_ui_released = false;

void instruction_list_release_ui(void)
{
    if (p_instruction_list_layout == NULL || s_list_ui_released)
        return;
    lv_obj_t *list = p_instruction_list_layout->list;
    if (list == NULL || !lv_obj_is_valid(list))
        return;
    /* dots 是 bg 的子物件,lv_obj_clean(list) 不會清到 —— 跟 refresh 的拆除段同序:
       先刪 dot_bg(其 children = frame + icon 一起走),立刻 NULL 掉 handle 陣列,再
       clean list。順序反了會在 DELETE 的同步回呼裡踩到已釋放的 dot(既有註解記錄
       過 DACCVIOL)。 */
    for (uint8_t i = 0; i < MAX_LIST_ITEMS; i++)
    {
        if (p_instruction_list_layout->indicator_dots_bg[i] != NULL &&
            lv_obj_is_valid(p_instruction_list_layout->indicator_dots_bg[i]))
            lv_obj_del(p_instruction_list_layout->indicator_dots_bg[i]);
        p_instruction_list_layout->indicator_dots_bg[i] = NULL;
        p_instruction_list_layout->indicator_dots[i] = NULL;
        app_icon_shadow[i] = NULL;
    }
    lv_obj_clean(list);
    for (uint8_t i = 0; i < MAX_LIST_ITEMS; i++)
    {
        app_icon[i] = NULL;
        app_widget[i] = NULL;
        touch_obj[i] = NULL;
        app_label[i] = NULL;
        switch_objs[i] = NULL;
        p_instruction_list_layout->p_app_indicator_btn[i] = NULL;
    }
    s_list_ui_released = true;
    LOG_W("[R32] list UI released (rebuilds on next open)");
}

/** 開清單前呼叫:被 release 過就原路重建(no-op if never released)。 */
void instruction_list_ensure_ui(void)
{
    if (!s_list_ui_released)
        return;
    s_list_ui_released = false;
    /* R41(founder:「左邊的 icon 還是進去才出現,第一次進去沒問題第二次才會」):
       refresh 有 500ms trailing-edge 去抖 —— 第二次進場的重建請求常常剛好落在
       前一次刷新的去抖窗內,被推遲到 550ms 後才真的建,於是清單先是空的、圖標才
       浮出來。這裡是「使用者正要看到清單」的當下,不能等:清掉去抖時間戳與待辦
       timer,強制這一次同步重建。 */
    s_last_refresh_tick = 0;
    if (s_pending_refresh_timer != NULL)
    {
        lv_timer_del(s_pending_refresh_timer);
        s_pending_refresh_timer = NULL;
    }
    /* R42:原樣重建 —— 資料沒變,別把圖的解碼結果丟掉(見 refresh 內的說明)。 */
    s_restore_rebuild = true;
    refresh_custom_instructions();
}

/** refresh 走到完整 rebuild 時清旗標(它自己就是重建路徑)。 */
void instruction_list_mark_ui_rebuilt(void)
{
    s_list_ui_released = false;
}

/** 同 ensure_ui,但**不**先用當前(舊)資料重建 —— 給「呼叫端接著馬上 clear+feed 新資料」
    的路徑(滑鼠單設備抽屜)。ensure_ui 會先把錶盤那份 9 列(含 session)整包建回來,feed
    再拆掉重建一次:白付一次工,更重要的是 heap 尖峰 —— 滑鼠 UI+聊天室都活著時多這一份
    舊清單就會把 lv_obj_create 壓到回 NULL(R32 記錄過的 lv_obj_class_init_obj hard
    fault,2026-08-15 真機在聊天室第二次返回時複現)。這裡只解除 R33 deferral+清去抖,
    讓 feed 的 refresh 直接一次建出新(較小的)單設備清單。 */
void instruction_list_ensure_ui_for_feed(void)
{
    if (!s_list_ui_released)
        return;
    s_list_ui_released = false;
    s_last_refresh_tick = 0;
    if (s_pending_refresh_timer != NULL)
    {
        lv_timer_del(s_pending_refresh_timer);
        s_pending_refresh_timer = NULL;
    }
    s_restore_rebuild = true; /* 同 ensure_ui:別把已解碼的圖丟掉(R42) */
}

/** R33:目前列的 UI 是不是處於已釋放狀態(refresh 用來決定要不要重建)。 */
bool instruction_list_ui_is_released(void)
{
    return s_list_ui_released;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
