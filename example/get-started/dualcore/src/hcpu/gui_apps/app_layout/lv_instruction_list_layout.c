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
LV_IMG_DECLARE(micro_icon);  /* shared mic glyph — the bottom trigger's NEW resting look (matches the chat page) */

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
    0; // pinned prefix protected from clear (index 0 = Settings). 0 until the
       // first load_instruction_list(), then 1.

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
static void inst_arc_reset_drag_state(void);
static void scroll_list_to_index(uint16_t page);

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
        if (p_instruction_list_layout->indicator_dots[i] == NULL)
            continue;

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

        /* 用指數曲線 ratio^N 取代線性 ratio：N>1 時，中央 dot 大幅放大，
         * 邊緣 dot 快速縮小，視覺上中央更突出 */
        float zoom_ratio = powf(ratio, DOT_ZOOM_EXPONENT);
        uint16_t zoom =
            (uint16_t)(255 * DOT_ICON_SCALE *
                       (DOT_SMOLL_PROPORTION +
                        (DOT_BIG_PROPORTION - DOT_SMOLL_PROPORTION) * zoom_ratio));
        if (abs((int)zoom - (int)last_zoom[i]) > 5)
        {
            lv_img_set_zoom(app_icon_shadow[i], zoom);
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

        p_instruction_list_layout->indicator_dots_bg[i] = dot_bg;
        p_instruction_list_layout->indicator_dots[i] = dot;
    }

    /* 初始定位 */
    update_indicator_dots_position(37);
}

extern void tap_on_ai_hint(void);
static bool is_open_ai_gesture = false;
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

static void create_movable_range_arc(lv_obj_t *parent)
{
    if (p_instruction_list_layout == NULL)
        return;

    lv_obj_t *arc = lv_arc_create(parent);

    lv_obj_set_size(arc, LIST_RADIUS, LIST_RADIUS);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);

    int start_angle = 348;
    int end_angle = 13;

    lv_arc_set_range(arc, 0, 30);
    lv_arc_set_bg_start_angle(arc, start_angle);
    lv_arc_set_bg_end_angle(arc, end_angle);
    lv_arc_set_value(arc, 30);

    lv_obj_set_style_arc_width(arc, 0, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_set_style_arc_width(arc, MOVABLE_ARC_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(arc, LV_OPA_70, LV_PART_INDICATOR);

    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc, 0, LV_PART_KNOB);

    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_SCROLLABLE);

    p_instruction_list_layout->movable_range_arc = arc;

    LOG_D("Movable range arc created: radius=%d, start=%d, end=%d",
          LIST_RADIUS / 2, MOVABLE_ARC_START_ANGLE, MOVABLE_ARC_END_ANGLE);
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
static lv_obj_t *instruction_list_main_status_bar;
static rt_tick_t last_gohame_time = 0;

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
        instruction_list_close_ai_on_leave();
        instruction_list_bar_set_blur(false);
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
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_HIDDEN);
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
                 * scroll_list 不要再從 card y_diff 推回去（會跟 snap 動畫打架）*/
                if (!s_inst_arc_drag_active)
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
                // 使用顏色深淺代替透明度，創建從白色到灰色的漸變
                lv_color_t text_color =
                    lv_color_make(brightness, brightness, brightness);
                if (app_label[i] != NULL && lv_obj_is_valid(app_label[i]))
                {
                    lv_obj_set_style_text_color(app_label[i], text_color, 0);
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

        /* 指示點也用全部項目範圍。drag_cb 模式下 dot 由 inst_arc_drag_cb 用累積
         * 的 input 自己更新，scroll_list 不要再用 list scroll_y 反推蓋過去 */
        int dots_value = child_cnt * 100 + first_y_diff - 63;
        if (SkaiWatchSys.motion_control_lock && !s_inst_arc_drag_active)
        {
            update_indicator_dots_position(dots_value);
        }

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
                if ((list_items[i].icon != NULL ||
                     list_items[i].img_path[0] != '\0') &&
                    app_icon_shadow[i] != NULL &&
                    lv_obj_is_valid(app_icon_shadow[i]))
                    lv_obj_clear_flag(app_icon_shadow[i], LV_OBJ_FLAG_HIDDEN);
                if (switch_objs[i] != NULL && lv_obj_is_valid(switch_objs[i]))
                    lv_obj_clear_flag(switch_objs[i], LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                if ((list_items[i].icon != NULL ||
                     list_items[i].img_path[0] != '\0') &&
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

static void selected_widget_timer_callback(void *parameter)
{
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_WIDGET_LIST_SELECT};
    lvgl_send_msg(msg);
}

static rt_timer_t selected_widget_timer = NULL;
static void selected_widget_timer_start(void)
{
    if (!selected_widget_timer)
    {
        selected_widget_timer = rt_timer_create(
            "speaking_debounce_timer", selected_widget_timer_callback, NULL,
            300, RT_TIMER_FLAG_ONE_SHOT);
    }
    else
    {
        rt_timer_stop(selected_widget_timer);
    }
    rt_timer_start(selected_widget_timer);
}

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
            ai_widget_fade_on_scroll();
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
static void app_list_window_scroll_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = evt->target;
    switch (evt->code)
    {
    case LV_EVENT_VALUE_CHANGED:
    {
        if (lv_obj_get_scroll_y(p_instruction_list_layout->app_list_tileview)/466 != 0)
            return;
        check_is_at_instruction_list();
        break;
    }
    default:
        break;
    }
    if (obj == NULL)
    {
        return;
    }
}
extern void set_skai_widget_opa(uint8_t opa);
extern void set_skai_widget_input_text(const char *text);
static void set_ai_bg_opa(void *obj, int32_t opa)
{
    uint8_t bg_opa = 240 * opa / 255;
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            bg_opa, 0);
    set_skai_widget_opa(opa);
}

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
/* Guard flag: when animate_open_ai_widget drives the tileview from tile 1 → 0
   with LV_ANIM_ON, the resulting SCROLL_END fires VALUE_CHANGED with
   active_pos=0. Without the guard, ai_tileview_event_cb would interpret that
   as "user dismissed to home" and close the widget we just opened. */
static bool s_programmatic_open_in_progress = false;

/* Mock instruction-update timer — simulates phone/PC pushing list updates
   while voice listening is active. Real backend wiring deferred per office-hours
   doc §5 P2 (latency unverified). Cycles a small fake set. */
static rt_timer_t mock_inst_update_timer = RT_NULL;
static int mock_inst_cycle = 0;
static const char *MOCK_INST_TITLES[] = {
    "Set 5 min timer",
    "Send 'on my way' to Mom",
    "Play workout playlist",
    "Add milk to grocery list",
};
static const char *MOCK_INST_IDS[] = {
    "mock-timer-001",
    "mock-msg-002",
    "mock-music-003",
    "mock-grocery-004",
};
#define MOCK_INST_COUNT (sizeof(MOCK_INST_IDS) / sizeof(MOCK_INST_IDS[0]))

static void mock_inst_update_cb(void *param)
{
    int idx = mock_inst_cycle % MOCK_INST_COUNT;
    add_or_update_custom_instruction(MOCK_INST_IDS[idx], MOCK_INST_TITLES[idx],
                                     "once", 0, false, mock_inst_cycle + 1, NULL);
    mock_inst_cycle++;
    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_INSTRUCTION_LIST};
    lvgl_send_msg(msg);
}

static void start_mock_inst_update(void)
{
    if (mock_inst_update_timer == RT_NULL)
    {
        /* SOFT timer required: callback calls add_or_update_custom_instruction
           (strncpy on shared array) + lvgl_send_msg. HARD timer = interrupt
           context, unsafe for either. SOFT timer runs in dedicated timer
           thread context, safe to call kernel APIs. */
        mock_inst_update_timer = rt_timer_create(
            "mock_inst_upd", mock_inst_update_cb, RT_NULL,
            rt_tick_from_millisecond(1500),
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (mock_inst_update_timer != RT_NULL)
    {
        rt_timer_start(mock_inst_update_timer);
        LOG_I("mock instruction update timer started");
    }
}

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
static void mic_bar_event_cb(lv_event_t *evt)
{
    if (evt->code != LV_EVENT_CLICKED) return;
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
       state list dismisses ONLY off the watch face (on the face it opens the box). */
    if (box_visible || (list_shown && !clock_main_page_is_home()))
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

/* 公開 bar-tap 入口：給其他 app（standalone APP_ID_MOUSE 滑鼠 app）驅動「跟錶盤底部
   bar 完全同一套」的兩段式 skaibar——**同一個元件**，故列表 / 輸入框的樣式與行為一致。
   1st tap：列表從右浮入(browse 態、無輸入框) + open_browse 的 settle 會 commu_send_
   skaibar_view 通知 active 設備開它的 skaibar、選項同步；2nd tap(列表已開)：morph 輸入
   框 + 語音；已開輸入框則收掉。永遠 browse-first、不模糊背景(呼叫者非錶盤)。 */
void instruction_list_bar_tap(void)
{
    if (!p_instruction_list_layout)
        return;
    bool box_visible =
        p_instruction_list_layout->p_instruction_list_ai_bg &&
        !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                         LV_OBJ_FLAG_HIDDEN);
    bool list_shown =
        p_instruction_list_layout->p_instruction_list_bg &&
        !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_bg,
                         LV_OBJ_FLAG_HIDDEN);
    extern void animate_open_ai_widget(void);
    extern void instruction_list_open_browse(void);
    extern void instruction_list_bar_set_blur(bool on);
    if (box_visible)
    {
        s_close_is_cancel = true; /* bar-tap close = user cancel */
        close_ai_widget();
    }
    else if (list_shown)
    {
        animate_open_ai_widget(); /* 2nd tap → 輸入框 + 語音 */
    }
    else
    {
        instruction_list_bar_set_blur(false); /* 非錶盤、不模糊 */
        instruction_list_open_browse();       /* 1st tap → 列表浮入 + 通知電腦開 skaibar */
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

/* 公開：給 STANDALONE 滑鼠 app(APP_ID_MOUSE)用的「單一控制設備」版 bar-tap。與
   instruction_list_bar_tap 同一個兩段式狀態機 + 同一個元件(列表/輸入框樣式必然一致),差別:
   1st tap 餵「控制中那台」的選項(非混合清單)、進單設備模式(settle 改送 commu_send_skaibar_
   open_device 叫那台電腦開它的 skaibar、非廣播)。2nd tap / 收掉與錶盤完全相同。device_id 為空
   時退回一般 bar(沒有控制目標就沒有單一電腦可開)。 */
void instruction_list_bar_tap_device(const char *device_id)
{
    LOG_I("[bar_dev] tap device_id=\"%s\"", (device_id && device_id[0]) ? device_id : "(EMPTY)");
    if (!p_instruction_list_layout)
    {
        /* P3 麥克風 OOM 修復:進 standalone 滑鼠 app 時錶盤(Main app)已被 gui_app_exit 整個拆掉,
           它建的 instruction_list 單例也跟著釋放(p 變 NULL)。滑鼠 skaibar 要用 → 這裡 lazy 自建
           一份,掛在滑鼠 app 自己的 active screen。Main 已 destroy、不會再有它的 on_stop 來誤拆
           這份;滑鼠 app 離開(ONSTOP)時自己 deinit。錶盤情境(Main 活著)p 永遠非 NULL,不會進來。 */
        extern lv_obj_t *lv_instruction_list_layout_create(lv_obj_t * parent);
        LOG_I("[bar_dev] no singleton (錶盤已拆) -> lazy create for mouse app");
        lv_instruction_list_layout_create(lv_scr_act());
        if (!p_instruction_list_layout)
            return; /* 建失敗才放棄 */
    }
    if (device_id == NULL || device_id[0] == '\0')
    {
        LOG_I("[bar_dev] EMPTY id -> fallback to generic broadcast bar");
        instruction_list_bar_tap(); /* 沒有控制目標 → 一般(廣播)bar */
        return;
    }
    bool box_visible =
        p_instruction_list_layout->p_instruction_list_ai_bg &&
        !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                         LV_OBJ_FLAG_HIDDEN);
    bool list_shown =
        p_instruction_list_layout->p_instruction_list_bg &&
        !lv_obj_has_flag(p_instruction_list_layout->p_instruction_list_bg,
                         LV_OBJ_FLAG_HIDDEN);
    extern void animate_open_ai_widget(void);
    extern void instruction_list_bar_set_blur(bool on);
    extern void instruction_list_open_browse(void);
    if (box_visible)
    {
        s_close_is_cancel = true; /* bar-tap close = user cancel → 送 dismiss */
        close_ai_widget();
    }
    else if (list_shown)
    {
        /* 2nd tap → 輸入框 + 語音(同錶盤)。開語音前重送 open_device 重新 latch 手機端單設備
           模式 —— 中間的 box-close 會送 dismiss(0x0C) 清掉手機 flag,不重 latch 的話這次語音
           transcript 會誤走廣播而非那一台。idempotent(電腦 skaibar 已開則維持)。 */
        extern bool commu_send_skaibar_open_device(void);
        commu_send_skaibar_open_device();
        animate_open_ai_widget();
    }
    else
    {
        s_bar_single_device = true;            /* 收合時 restore、scroll/commit 不清 active */
        /* 立刻送 0x0E(不等 settle)叫「控制中那台」電腦 summon 它的 skaibar。先餵 registry 的
           default_actions 當即時 placeholder;之後手機把電腦的即時選項 push 過來會直接套上
           (即時更新)。 */
        extern bool commu_send_skaibar_open_device(void);
        commu_send_skaibar_open_device();
        feed_single_device_options(device_id); /* placeholder:手機即時 push 一到就替換成電腦選項 */
        instruction_list_bar_set_blur(false);  /* 非錶盤、不模糊 */
        instruction_list_open_browse();        /* 列表浮入 */
    }
}

/* 公開：滑鼠 app 離開(destroy / Exit)時呼叫 —— 若還在單設備模式,把共享清單還原成錶盤清單、
   通知電腦收掉它的 skaibar、清旗標,避免設備選項殘留在下次錶盤底部 bar。idempotent。 */
void instruction_list_bar_device_dismiss(void)
{
    if (!s_bar_single_device)
        return;
    s_bar_single_device = false;
    extern bool commu_send_skaibar_dismiss(void);
    extern void instruction_list_restore_base(void);
    commu_send_skaibar_dismiss();
    instruction_list_restore_base();
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

/* The box is scrollable only to ABSORB swipes (so a left-swipe doesn't bubble to
   the page nav and jump to the watchface). When the user actually starts a swipe
   on the open box, dismiss it with the animated reverse morph — mirrors the
   right device_pager's scroll_hides_skaibar_cb. */
static void ai_box_scroll_dismiss_cb(lv_event_t *evt)
{
    (void)evt;
    if (is_open_instruction_list_ai)
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

/* Set by ai_widget_fade_on_scroll: THIS close should collapse the voice box back
   to the slim bar but KEEP the floating list up (browse / switch options).
   Consumed at the top of close_ai_widget; every other close path slides the whole
   list out. */
static bool s_scroll_keep_list = false;

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
    cat_filter_restore_full();
    s_view_cat = 0;
    /* 滑鼠 app 單設備模式:列表完全收掉 → 把共享清單還原成錶盤清單、退出單設備模式,
       否則設備選項會殘留到下次錶盤底部 bar。錶盤 / device_pager 路徑 flag 為 false,無影響。 */
    if (s_bar_single_device)
    {
        s_bar_single_device = false;
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
    instruction_list_bar_set_visible(true); /* idempotent on HOME */
    lv_anim_del(list_bg, inst_list_slide_anim_cb);
    lv_obj_clear_flag(list_bg, LV_OBJ_FLAG_HIDDEN);
    /* Park off-screen on the entry edge; drag_update finger-tracks it toward 0. */
    lv_obj_set_style_translate_x(list_bg,
                                 s_reveal_from_left ? -LV_HOR_RES : LV_HOR_RES, 0);
    reset_list_internal(); /* every reveal opens at the list's bottom item (R3) */
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
        lv_anim_set_values(&sl, tx, 0);
        lv_anim_set_time(&sl, LSLIDE_MS);
        lv_anim_set_path_cb(&sl, lv_anim_path_ease_out);
        lv_anim_set_ready_cb(&sl, reveal_settle_browse_done_cb);
    }
    else
    {
        lv_anim_set_values(&sl, tx, LV_HOR_RES);
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
    /* Only when parked & hidden — already up (browse/open) or sliding shut: leave it. */
    if (s_left_closing || !lv_obj_has_flag(list_bg, LV_OBJ_FLAG_HIDDEN))
        return;
    s_pending_reveal_filter = 0; /* bar / IMU browse → all (@ + /) view */
    s_reveal_from_left = true;   /* IMU release brings the LEFT list in (the right-edge
                                    drawer is legacy); park on the left, slide to 0 */
    instruction_list_reveal_drag_begin(); /* un-hide + park + backdrop */
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
    return s_global_bar_layer && lv_obj_is_valid(s_global_bar_layer) &&
           !lv_obj_has_flag(s_global_bar_layer, LV_OBJ_FLAG_HIDDEN);
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
    lv_obj_set_style_bg_opa(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OPA_0, 0);
    /* Box is a plain container now (no tileview) — just hide it. */
    lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                    LV_OBJ_FLAG_HIDDEN);
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
        if (clock_main_page_is_home())
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

void check_ai_widget_auto_close(void)
{
    extern bool get_skai_input_text_is_null(void);
    /* Manual drag-open: user explicitly opened it, don't auto-close */
    if (ai_widget_opened_by_drag)
        return;
    if (is_open_instruction_list_ai && !is_at_ai_widget &&
        get_skai_input_text_is_null())
    {
        rt_tick_t current_time = rt_tick_get();
        if (current_time - last_ai_widget_open_time < 3000) // 5秒后自动关闭
        {
            close_ai_widget();
            is_open_instruction_list_ai = false;
        }
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

static void list_item_click_event_cb(lv_event_t *evt)
{
    list_item_t *item = (list_item_t *)evt->user_data;
    lv_obj_t *obj = evt->target;
    /* A horizontal swipe (the left-to-close flick, or a right flick) also lands a
       CLICKED on the item: the list scrolls vertically only, so a horizontal drag
       never "loses" the press and LVGL fires CLICKED on release. The list's
       GESTURE handler latches s_list_horiz_swipe for exactly this — ignore the
       click so swipe-right-to-close doesn't ALSO select the item. (A flag, not
       lv_indev_get_gesture_dir, because gesture_dir lingers after the swipe and
       would then wrongly suppress the NEXT genuine tap.) */
    if (s_list_horiz_swipe)
        return;
    LOG_D("ID: %s,obj:%p", item->id, obj);

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

static bool tap_to_open_control = false;
static bool open_quick_app = false;
static void on_tap(void)
{
    if (selected_item_index >= list_item_count)
        return;

    list_item_t *item = &list_items[selected_item_index];

    /* A tapped @-contact opens the in-watch chat room — the raise-wrist gesture path,
       mirror of list_item_click_event_cb. Keys off the item's own category (the merged
       mixed list has no separate @ view). */
    if (item->category == '@' && !is_open_instruction_list_ai)
    {
        commu_send_conv_open(item->title, item->id, (uint8_t)selected_item_index);
        chat_page_open(item->title, item->icon);
        return;
    }

    if (item->is_instruction)
    {
        LOG_I("Custom instruction tapped via gesture: id=%s", item->id);
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

static int16_t find_app_index_by_id(uint16_t app_id)
{
    for (int i = 0; i < INSTRUCTION_LIST_ITEMS_DEF_COUNT; i++)
    {
        if (INSTRUCTION_LIST_ITEMS_DEFINITION[i] == app_id)
        {
            return i;
        }
    }
    return -1; // 未找到 — 必須是不可能的 index 哨兵（不能用 app_base_count-1：
               // 設定釘在 index 0 後 app_base_count==1，回 0 會誤判 index 0 命中）
}

extern char *get_media_title(void);
extern bool is_have_message_now(void);
static uint16_t gesture_starting_value = 0;

static void reset_list_internal(void)
{
    if (p_instruction_list_layout->list == NULL)
    {
        return;
    }
    open_scroll_motor = false;
    disable_scrolling_motor_vibrate();
    set_paused_control_with_arm(false);
    scroll_initialized = false;
    uint8_t scroll_to_index;
    {
        /* 滾到列表最下面那個項目 */
        scroll_to_index = list_item_count - 1;
        app_scroll_target_item = scroll_to_index;
    }
    gesture_starting_value = 37;
    selected_item_index = app_scroll_target_item;
    prev_app_scroll_target_item = app_scroll_target_item;
    lv_obj_t *child =
        lv_obj_get_child(p_instruction_list_layout->list, scroll_to_index);
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
static void button_selection(gesture_position_t gesture_position)
{
    const int p_y = gesture_position.gesture_position_y;
    if (selected_item_index == find_app_index_by_id(app_id_media))
    {
        media_widget_trigger_drag_by_py(p_y);
    }
}

#ifdef USE_QUICK_OPEN_AI
static uint8_t quick_app_id = 0;
static void gesture_tap_event_handler(uint8_t gesture)
{
    if (gesture != 1)
    {
        return;
    }
    if (quick_app_id == 0)
    {
        force_release_finger();
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_CONTROL_QUICK_BTN;
        msg.data.quick_btn_action.new_point = 0;
        msg.data.quick_btn_action.swich_quick_btn = 0;
        lvgl_send_msg(msg);
        msg.type = LVGL_MSG_TYPE_SWITCH_FLASHLIGHT;
        lvgl_send_msg(msg);
    }
    else if (quick_app_id == 1)
    {
        force_release_finger();
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_CONTROL_QUICK_BTN;
        msg.data.quick_btn_action.new_point = 0;
        msg.data.quick_btn_action.swich_quick_btn = 0;
        lvgl_send_msg(msg);
    }
}

static void open_quick_app_timer_cb(void *param)
{
    lvgl_msg_handler.handle_tap_indicator = gesture_tap_event_handler;
}

static rt_timer_t timer_open_quick_app = RT_NULL;
static void start_open_quick_app_timer(uint8_t set_app_id)
{
    quick_app_id = set_app_id;
    if (!timer_open_quick_app)
    {
        timer_open_quick_app = rt_timer_create(
            "open_quick_app", open_quick_app_timer_cb, RT_NULL,
            rt_tick_from_millisecond(300), RT_TIMER_FLAG_ONE_SHOT);
        if (timer_open_quick_app == RT_NULL)
        {
            LOG_E("Failed to create timer_open_quick_app");
            return;
        }
        LOG_D("timer_open_quick_app is created");
    }
    else
    {
        rt_timer_stop(timer_open_quick_app);
    }
    rt_timer_start(timer_open_quick_app);
}

static void stop_open_quick_app_timer(void)
{
    if (timer_open_quick_app)
    {
        rt_timer_stop(timer_open_quick_app);
    }
}

static bool open_vibration = false;
static quick_open_app_t quick_open_app_obj;
static void move_quick_icon(QuickBtn pame)
{
    uint16_t distance = pame.new_point;
    uint16_t swich_quick_btn = pame.swich_quick_btn;

    if (swich_quick_btn == 1 && is_at_instruction_list())
    {
    }
    else if (swich_quick_btn == 2)
    {
    }
    else if (swich_quick_btn == 3 && !app_voice_get_voice2text_status() &&
             !is_at_home())
    {
        if (distance > 45)
        {
            if (open_vibration)
            {
                start_open_quick_app_timer(1);
                open_vibration = false;
            }
            open_quick_app = true;
        }
        else
        {
            lvgl_msg_handler.handle_tap_indicator = NULL;
            stop_open_quick_app_timer();
            open_quick_app = false;
            open_vibration = true;
        }
    }
    else if (swich_quick_btn == 4 && !app_voice_get_voice2text_status() &&
             !is_at_home())
    {
        if (open_vibration)
        {
            start_open_quick_app_timer(1);
            open_vibration = false;
        }
        open_quick_app = true;
        lv_obj_set_style_border_opa(quick_open_app_obj.left, LV_OPA_80,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(quick_open_app_obj.left, LV_OPA_100,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_pos(quick_open_app_obj.left, 50, 258);
    }
    else if (swich_quick_btn == 0)
    {
        lvgl_msg_handler.handle_tap_indicator = NULL;
        stop_open_quick_app_timer();
        open_quick_app = false;
        lv_obj_set_style_border_opa(quick_open_app_obj.bottom, LV_OPA_0,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(quick_open_app_obj.bottom, LV_OPA_0,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_pos(quick_open_app_obj.bottom, 258, 0);
        lv_obj_set_style_border_opa(quick_open_app_obj.left, LV_OPA_0,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(quick_open_app_obj.left, LV_OPA_0,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_pos(quick_open_app_obj.left, 0, 258);
    }
}
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

static lv_obj_t *complete_on_phone_tip_window = NULL;
static lv_obj_t *complete_on_phone_tip_label = NULL;
static lv_timer_t *complete_on_phone_tip_timer = NULL;

static void complete_on_phone_tip_fade_ready_cb(lv_anim_t *anim)
{
    if (lv_obj_is_valid(complete_on_phone_tip_window))
    {
        lv_obj_del(complete_on_phone_tip_window);
        complete_on_phone_tip_window = NULL;
        complete_on_phone_tip_label = NULL;
    }
}

static void complete_on_phone_tip_set_opa(void *obj, int32_t opa)
{
    if (lv_obj_is_valid(complete_on_phone_tip_window))
    {
        uint8_t window_opa = opa * LV_OPA_90 / LV_OPA_100;
        lv_obj_set_style_bg_opa(complete_on_phone_tip_window, window_opa,
                                LV_PART_MAIN);
        uint8_t border_opa = opa * LV_OPA_50 / LV_OPA_100;
        lv_obj_set_style_border_opa(complete_on_phone_tip_window, border_opa,
                                    LV_PART_MAIN);
        if (complete_on_phone_tip_label)
        {
            lv_obj_set_style_text_opa(complete_on_phone_tip_label, opa,
                                      LV_PART_MAIN);
        }
    }
}

static void complete_on_phone_tip_timer_cb(lv_timer_t *timer)
{
    if (complete_on_phone_tip_timer)
    {
        lv_timer_del(complete_on_phone_tip_timer);
        complete_on_phone_tip_timer = NULL;
    }
    if (lv_obj_is_valid(complete_on_phone_tip_window))
    {
        lv_anim_t fade_out_anim;
        lv_anim_init(&fade_out_anim);
        lv_anim_set_var(&fade_out_anim, complete_on_phone_tip_window);
        lv_anim_set_exec_cb(&fade_out_anim, complete_on_phone_tip_set_opa);
        lv_anim_set_values(&fade_out_anim, LV_OPA_100, LV_OPA_TRANSP);
        lv_anim_set_time(&fade_out_anim, 600);
        lv_anim_set_path_cb(&fade_out_anim, lv_anim_path_ease_in);
        lv_anim_set_ready_cb(&fade_out_anim,
                             complete_on_phone_tip_fade_ready_cb);
        lv_anim_start(&fade_out_anim);
    }
}

static void show_complete_on_phone_tip(void)
{
    if (complete_on_phone_tip_timer)
    {
        lv_timer_del(complete_on_phone_tip_timer);
        complete_on_phone_tip_timer = NULL;
    }
    if (lv_obj_is_valid(complete_on_phone_tip_window))
    {
        lv_obj_del(complete_on_phone_tip_window);
        complete_on_phone_tip_window = NULL;
        complete_on_phone_tip_label = NULL;
    }

    complete_on_phone_tip_window = lv_obj_create(lv_layer_top());
    lv_obj_set_size(complete_on_phone_tip_window, 260, 70);
    lv_obj_align(complete_on_phone_tip_window, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(complete_on_phone_tip_window,
                              lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(complete_on_phone_tip_window, LV_OPA_90,
                            LV_PART_MAIN);
    lv_obj_set_style_border_color(complete_on_phone_tip_window,
                                  lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(complete_on_phone_tip_window, 1,
                                  LV_PART_MAIN);
    lv_obj_set_style_border_opa(complete_on_phone_tip_window, LV_OPA_50,
                                LV_PART_MAIN);
    lv_obj_set_style_radius(complete_on_phone_tip_window, 35, LV_PART_MAIN);
    lv_obj_clear_flag(complete_on_phone_tip_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(complete_on_phone_tip_window, LV_OBJ_FLAG_CLICKABLE);

    complete_on_phone_tip_label = lv_label_create(complete_on_phone_tip_window);
    lv_label_set_text(complete_on_phone_tip_label, LV_EXT_STR_GET_BY_KEY(complete_on_phone, "Complete on phone"));
    lv_obj_set_style_text_color(complete_on_phone_tip_label, lv_color_white(),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(complete_on_phone_tip_label,
                               LV_EXT_FONT_GET(get_system_font_size(1)),
                               LV_PART_MAIN);
    lv_obj_center(complete_on_phone_tip_label);

    complete_on_phone_tip_timer =
        lv_timer_create(complete_on_phone_tip_timer_cb, 1000, NULL);
    lv_timer_set_repeat_count(complete_on_phone_tip_timer, 1);
}

static void add_instruction_btn_event_cb(lv_event_t *evt)
{
    LOG_I("add_inst_btn clicked: show 'complete on phone' tip");
    if (get_bluetooth_connection_status())
    {
        const char *json = "{\"action\":\"add\"}";
        LOG_I("Send create-instruction request: %s", json);
        commu_send_update_instruction(json);
    }
    show_complete_on_phone_tip();
}

static void logo_click_event_cb(lv_event_t *evt)
{
    /* Re-ask has priority: once the AI has replied, the button's job is to
       clear everything and start a new voice capture — even if the old
       speech text is still in the v2t buffer. */
    if (skai_widget_has_ai_reply() && !get_voice_recognition_started())
    {
        if (!get_bluetooth_connection_status())
        {
            create_connection_tips();
            return;
        }
        send_to_ai_again = true;
        clearVoice2Text();
        clear_skai_widget_ai_reply();
        set_skai_widget_input_text("");
        set_ai_open_mic(true);
        open_skai_widget_ai(true);
        voice_provider.start_v2t();
        return;
    }
    /* Normal flow: if we have speech text in the buffer, send it to AI. */
    if (!isTextEmpty())
    {
        tap_on_ai_hint();
        if (ai_voice_send_icon && lv_obj_is_valid(ai_voice_send_icon))
            lv_obj_add_flag(ai_voice_send_icon, LV_OBJ_FLAG_HIDDEN);
    }
}
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
/* Was 240 = dark backdrop frame around the AI widget. Set to 0 so the
   styled skai_widget pill (Liquid Glass aesthetic) sits cleanly over the
   list without a darker enclosing rectangle. */
static uint16_t ai_bg_opa = 0;
static void ai_tileview_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = lv_event_get_target(evt);
    switch (evt->code)
    {
    case LV_EVENT_SCROLL:
    {
        uint16_t ai_scroll_x =
            (466 - lv_obj_get_scroll_x(obj)) * ai_bg_opa / 350;
        uint8_t calculated_opa =
            (ai_scroll_x > ai_bg_opa) ? ai_bg_opa : ai_scroll_x;
        lv_obj_set_style_bg_opa(
            p_instruction_list_layout->p_instruction_list_ai_bg, calculated_opa,
            0);
        break;
    }
    case LV_EVENT_VALUE_CHANGED:
    {
        lv_coord_t scroll_x = lv_obj_get_scroll_x(obj);
        if (abs(scroll_x) % 466 != 0)
        {
            break;
        }
        rt_uint32_t active_pos = (rt_uint32_t)lv_event_get_param(evt);
        if (active_pos == 0)
        {
            /* Skip close path if we're inside our own animated open —
               this VALUE_CHANGED is the settle of animate_open_ai_widget's
               tile 1→0 anim, not a user dismissal. */
            if (s_programmatic_open_in_progress)
            {
                s_programmatic_open_in_progress = false;
                break;
            }
            // is_open_instruction_list_ai = false;
            voice_provider.stop_v2t();
            stop_voice_recognition(V2T_INTENT_NOTHING);
            set_is_open_instruction_list_ai(false);
            open_skai_widget_ai(false);
            set_paused_control_with_arm(false);
            set_ai_open_mic(false);
            // lvgl_msg_handler.handle_vad_status = NULL;
            skai_widget_shown = false;
            if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
            {
                lv_obj_add_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
            }
            // show_speech_indicator(false);
            // voice_provider.stop_v2t();
            // close_ai_widget();
            lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OBJ_FLAG_HIDDEN);
        }
        else if (active_pos == 1)
        {
            if (!is_open_instruction_list_ai)
            {
                set_ai_open_mic(true);
                tap_on_ai_widget();
                /* Drag-opened: jump straight to Phase 2 (widget visible, dark
                   bg) without the fade-in animation — the drag itself already
                   provides the reveal motion. */
                ai_widget_opened_by_drag = true;
                skai_widget_shown = true;
                if (ai_gaus_bg && lv_obj_is_valid(ai_gaus_bg))
                {
                    lv_obj_clear_flag(ai_gaus_bg, LV_OBJ_FLAG_HIDDEN);
                }
                // if (ai_voice_send_icon &&
                // lv_obj_is_valid(ai_voice_send_icon))
                // {
                //     lv_obj_clear_flag(ai_voice_send_icon,
                //     LV_OBJ_FLAG_HIDDEN);
                // }
                lv_obj_set_style_bg_opa(
                    p_instruction_list_layout->p_instruction_list_ai_bg,
                    LV_OPA_50, 0);
                set_skai_widget_opa(LV_OPA_COVER);
            }
        }
        else
        {
            LOG_W("Unknown tileview position: %d", active_pos);
        }
        break;
    }
    default:
        break;
    }
}

static void home_tileview_event_cb(lv_event_t *evt)
{
    lv_obj_t *obj = lv_event_get_target(evt);
    switch (evt->code)
    {
    case LV_EVENT_RELEASED:
    {
        /* Only hide ai_bg if user pressed without actually opening the AI page.
           `obj` is the home_page tile (always scroll 0,0); check the tileview's
           scroll via its parent, and keep it visible whenever AI is open. */
        lv_obj_t *tileview = lv_obj_get_parent(obj);
        if (!is_open_instruction_list_ai && tileview &&
            lv_obj_get_scroll_x(tileview) == 466)
        {
            lv_obj_add_flag(p_instruction_list_layout->p_instruction_list_ai_bg,
                            LV_OBJ_FLAG_HIDDEN);
        }
        break;
    }
    default:
        break;
    }
}

/*******************************************************************************
 * Custom Instruction API
 ******************************************************************************/

void clear_custom_instructions(void)
{
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

/* Helper: create list item UI objects for items in [start_idx, end_idx) */
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
void instruction_list_force_scroll_to_last(void)
{
    s_force_scroll_to_last = true;
    s_force_visual_at_bottom = true;
}

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

void refresh_custom_instructions(void)
{
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
       to preserve position can clear the flag before refresh. */
    s_force_scroll_to_last = true;

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
       new pixels rather than a stale cached entry. */
    for (uint8_t i = 0; i < list_item_count; i++)
    {
        if (list_items[i].img_path[0] != '\0')
            lv_img_cache_invalidate_src(list_items[i].img_path);
    }

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
    }

    /* Save current scroll position and selected index */
    lv_coord_t saved_scroll_y = lv_obj_get_scroll_y(list);
    uint16_t saved_selected = selected_item_index;

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
    create_list_items_ui(list, 0, list_item_count);

    /* 重建指示點 */
    create_indicator_dots(bg);

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
    if (s_force_scroll_to_last && list_item_count > 0)
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
        lv_obj_t *child = lv_obj_get_child(list, target);
        if (child && lv_obj_is_valid(child))
            lv_obj_scroll_to_view(child, LV_ANIM_OFF);
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
        lv_obj_t *child = lv_obj_get_child(list, target);
        if (child && lv_obj_is_valid(child))
            lv_obj_scroll_to_view(child, LV_ANIM_OFF);
        lv_obj_update_layout(list);
        scroll_list(list, 0);
    }
    else
    {
        /* Add or no change — keep exact same scroll position */
        lv_obj_scroll_to_y(list, saved_scroll_y, LV_ANIM_OFF);
        lv_obj_update_layout(list);
        scroll_list(list, 0);
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
    if (!is_at_instruction_list())
    {
        reset_list();
    }
    /* item 數變了 → 同步給共用 arc_scroll 模組做 scroll clamp */
    if (p_instruction_list_layout->arc_handle != NULL)
    {
        arc_scroll_set_item_count(p_instruction_list_layout->arc_handle,
                                  list_item_count);
    }
    LOG_D("refresh_custom_instructions: %d items total", list_item_count);
    s_in_refresh_scroll = false;
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

static lv_obj_t *ai_tileview = NULL;

static void go_to_app_list_btn_cb(lv_event_t *evt)
{
    if (p_instruction_list_layout == NULL ||
        p_instruction_list_layout->app_list_tileview == NULL)
        return;
    lv_obj_set_tile_id(p_instruction_list_layout->app_list_tileview, 0, 1,
                       LV_ANIM_ON);
    LOG_I("Navigate to app list via button");
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
static int32_t s_inst_snap_anim_dummy;
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
        scroll_list_to_index((uint16_t)closest_idx);
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
    /* The bar's resting look is now the MIC GLYPH (micro_icon), not the slim skaibar_img bar — matching
       the in-chat voice trigger (founder 2026-06-29). Half-size it (zoom 128) with a CENTRE pivot +
       OVERFLOW_VISIBLE on BOTH the glyph and the bar, so the ezip bitmap scales cleanly and isn't
       clipped to the 31px bar height (a set_size would clip it). Still the s_mic_bar_icon slot, so
       lmic_grow_cb's existing 255->0 img-opa fade dissolves the glyph as the bar morphs into the box.
       Non-clickable so taps fall through to mic_bar / mic_hit (which open the box). */
    lv_obj_add_flag(mic_bar, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    s_mic_bar_icon = lv_img_create(mic_bar);
    lv_img_set_src(s_mic_bar_icon, &micro_icon);
    lv_img_set_pivot(s_mic_bar_icon, micro_icon.header.w / 2, micro_icon.header.h / 2);
    lv_obj_add_flag(s_mic_bar_icon, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_img_set_zoom(s_mic_bar_icon, 128);
    lv_obj_center(s_mic_bar_icon);
    lv_obj_clear_flag(s_mic_bar_icon, LV_OBJ_FLAG_CLICKABLE);

    /* UPWARD-ONLY tap helper for the slim 100x16 pill (hard to hit). A transparent
       sibling band glued to the bar's TOP and growing UP only — its bottom is at
       the bar's top (~36px above the screen bottom), so it never reaches the bottom
       edge where the app-list swipe-up presses down. That is the whole point: a
       symmetric ext_click_area (and the removed 240x90 overlay) grew DOWNWARD into
       that edge and swallowed the swipe-up; this can't, and the bar's own 20-36px
       band keeps its existing tap-vs-drag split untouched. mic_hit_event_cb opens
       only while the bar is shown; PRESS_LOCK cleared so a drag still transfers
       down. Created BEFORE ai_box so the open box covers it (z-order). Child of
       s_global_bar_layer -> chain-deleted with the bar. */
    lv_obj_t *mic_hit = lv_obj_create(s_global_bar_layer);
    lv_obj_remove_style_all(mic_hit);
    lv_obj_set_size(mic_hit, LMIC_W + 40, 40);
    lv_obj_align(mic_hit, LV_ALIGN_BOTTOM_MID, 0, LMIC_Y - LMIC_H); /* bottom = bar TOP; grows up to ~76px */
    lv_obj_add_flag(mic_hit, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(mic_hit, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(mic_hit, LV_OBJ_FLAG_PRESS_LOCK); /* drag transfers down to the swipe-up */
    lv_obj_add_event_cb(mic_hit, mic_hit_event_cb, LV_EVENT_CLICKED, NULL);

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
       the left edge opens the @-chat filtered browse view. Same gating as the right
       (check_is_at_home → instruction_list_reveal_overlay_set_enabled). */
    s_reveal_edge_overlay_left = lv_obj_create(s_global_bar_layer);
    lv_obj_remove_style_all(s_reveal_edge_overlay_left);
    lv_obj_set_size(s_reveal_edge_overlay_left, LREVEAL_EDGE_W, LV_VER_RES);
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
    LOG_I("instruction_list_init: returning");

#ifdef TEST_INDICATOR_ANIMATION
    start_indicator_dots_animation_test(); // 開始指示點動畫測試
#endif
    return p_instruction_list_bg;
}

// static rt_uint32_t last_scroll_time = 0;
static void scroll_list_to_index(uint16_t page)
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
    lv_obj_scroll_to_view(child, LV_ANIM_ON);
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
        scroll_list_to_index(action);
    }

    else
    {
        LOG_W("Target index out of bounds: %d", action);
    }
}

/* map_app_id fills a list_item_t from an app_id enum value */
static void map_app_id(uint8_t app_id, list_item_t *item)
{
    memset(item, 0, sizeof(list_item_t));
    item->is_instruction = false;
    item->is_interval = false;
    item->enabled = false;
    item->interval_sec = 0;
    item->widget = NULL;

    const char *title = "Unknown";
    const char *icon = IMG_LOGO;
    const char *id_str = APP_ID_MAIN;

    switch (app_id)
    {
#ifdef APP_ID_RECORDER
    case app_id_recorder:
        title = LV_EXT_STR_GET_BY_KEY(recorder, "Recorder");
        icon = IMG_RECORDER;
        id_str = APP_ID_RECORDER;
        break;
#endif
#ifdef APP_ID_WEATHER
    case app_id_weather:
        title = LV_EXT_STR_GET_BY_KEY(weather, "Weather");
        icon = IMG_GROUP;
        id_str = APP_ID_WEATHER;
        break;
#endif
#ifdef APP_ID_EXERCISE
    case app_id_exercise:
        title = LV_EXT_STR_GET_BY_KEY(exercise, "Exercise");
        icon = IMG_WORKOUT;
        id_str = APP_ID_EXERCISE;
        break;
#endif
#ifdef APP_ID_FLASHLIGHT
    case app_id_flashlight:
        title = LV_EXT_STR_GET_BY_KEY(flashlight, "Flashlight");
        icon = IMG_FLASHLIGHT;
        id_str = APP_ID_FLASHLIGHT;
        break;
#endif
#ifdef APP_ID_MEDIA
    case app_id_media:
        title = LV_EXT_STR_GET_BY_KEY(media, "Media");
        icon = IMG_ITUNES;
        id_str = APP_ID_MEDIA;
        break;
#endif
#ifdef APP_ID_PHOTO
    case app_id_photo:
        title = LV_EXT_STR_GET_BY_KEY(photo, "Photo");
        icon = IMG_PHOTO;
        id_str = APP_ID_PHOTO;
        break;
#endif
#ifdef APP_ID_GAME_DINOSAUR
    case app_id_game_dinosaur:
        title = LV_EXT_STR_GET_BY_KEY(game, "Game");
        icon = IMG_GAME;
        id_str = APP_ID_GAME_DINOSAUR;
        break;
#endif
#ifdef APP_ID_CALCULATOR
    case app_id_calculator:
        title = LV_EXT_STR_GET_BY_KEY(calculator, "Calculator");
        icon = IMG_CALCULATOR;
        id_str = APP_ID_CALCULATOR;
        break;
#endif
#ifdef APP_ID_TIMER
    case app_id_timer:
        title = LV_EXT_STR_GET_BY_KEY(timer, "Timer");
        icon = IMG_ALARM_2;
        id_str = APP_ID_TIMER;
        break;
#endif
#ifdef APP_ID_ALARM
    case app_id_alarm:
        title = LV_EXT_STR_GET_BY_KEY(alarm, "Alarm");
        icon = IMG_ALARM;
        id_str = APP_ID_ALARM;
        break;
#endif
#ifdef APP_ID_SETTING
    case app_id_setting:
        title = LV_EXT_STR_GET_BY_KEY(setting, "Setting");
        icon = IMG_SETTINGS;
        id_str = APP_ID_SETTING;
        break;
#endif
#ifdef APP_ID_MOUSE
    case app_id_mouse:
        title = LV_EXT_STR_GET_BY_KEY(mouse, "Mouse");
        icon = IMG_MOUSE;
        id_str = APP_ID_MOUSE;
        break;
#endif
    default:
        break;
    }

    strncpy(item->id, id_str, LIST_ITEM_ID_LEN - 1);
    item->id[LIST_ITEM_ID_LEN - 1] = '\0';
    strncpy(item->title, title, LIST_ITEM_TITLE_LEN - 1);
    item->title[LIST_ITEM_TITLE_LEN - 1] = '\0';
    item->icon = icon;
}

/* Built-in apps shown as the watch's default "instructions" before any phone
   list arrives. Mirrors APP_LIST_ITEMS in lv_app_list_layout.c (same #ifdef
   guards) — that file's swipe-down control-center grid was removed, so this is
   now the canonical home of the built-in app list. Settings is intentionally
   absent: it is pinned separately at index 0. The debug-only dinosaur game is
   omitted (it needs kReleaseMode, not included here, and is not a real app). */
static const uint16_t DEFAULT_APP_ITEMS[] = {
#ifdef APP_ID_TIMER
    app_id_timer,
#endif
    app_id_flashlight,
#ifdef APP_ID_RECORDER
    app_id_recorder,
#endif
    app_id_exercise,
#ifdef APP_ID_SLEEP
    app_id_sleep,
#endif
#ifdef APP_ID_CALCULATOR
    app_id_calculator,
#endif
#ifdef APP_ID_WEATHER
    app_id_weather,
#endif
#ifdef APP_ID_ALARM
    app_id_alarm,
#endif
#ifdef APP_ID_PHOTO
    app_id_photo,
#endif
    /* P3: mouse / trackpad is now opened as a list app (its left-swipe tile entry
       was removed so the left edge can host the @-list reveal). map_app_id maps
       app_id_mouse → APP_ID_MOUSE; tap runs it locally via gui_app_run. */
    app_id_mouse,
};

/* Fill [app_base_count ..] with the built-in default apps and enter DEFAULT_APPS
   mode. Index 0 (Settings) is set up by load_instruction_list, not here. */
static void load_default_apps(void)
{
    uint8_t slot = app_base_count; /* == 1: right after the pinned Settings */
    for (uint8_t i = 0; i < ARRAY_SIZE(DEFAULT_APP_ITEMS); i++)
    {
        if (slot >= MAX_LIST_ITEMS)
            break;
        map_app_id(DEFAULT_APP_ITEMS[i], &list_items[slot]);
        slot++;
    }
    list_item_count = slot;
    s_list_mode = LIST_MODE_DEFAULT_APPS;
}

/* An item the watch can act on with no phone: a built-in app (Settings / default
   apps, is_instruction == false) or an "open watch app" instruction (openApp
   set). The rest are phone-relay instructions, hidden while disconnected. */
static bool item_is_standalone(const list_item_t *it)
{
    return (!it->is_instruction) || (it->open_app[0] != '\0');
}

/* True if item passes the active category view filter (ADR-0024). The three views show
   DIFFERENT content, not one list filtered loosely:
     - '@' (left)  : STRICT chat — the @ destinations / chat services (@gpt / @google …).
     - '/' (right) : STRICT actions — saved scripts / device actions only.
     - 'all' (bar) : the EXECUTION HISTORY + actions — untagged (recents: apps / urls /
                     calc / files) and '/' actions, but NOT the '@' chat destinations
                     (those are reached only via the '@' drawer; the bar is "what I ran",
                     not the @ service menu). So 'all' = everything EXCEPT '@'. */
static bool item_matches_cat(const list_item_t *it, char cat)
{
    if (cat == 0)
        return it->category != '@'; /* 'all' bar = history + actions, NOT @ destinations */
    return it->category == cat;     /* '@' and '/' are both strict */
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
}

/* Apply (cat '@'/'/') or clear (cat 0) the view filter and rebuild the UI.
   Re-packs list_items[] to the matching subset over a full-list snapshot, keeping
   the pinned prefix — mirrors instruction_list_set_phone_connected so scroll /
   tap / indicator-dots keep working on the shorter contiguous array unchanged. */
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
    if (s_view_cat != 0 && instruction_list_is_visible())
    {
        instruction_list_set_category_filter(s_view_cat); /* re-packs + refreshes */
        return true;
    }
    return false;
}

void load_instruction_list(void)
{
    /* Index 0 is always the pinned Settings entry. Re-mapping it here means a
       language change (load_instruction_list is the translation-reload hook)
       re-translates its title. Tapping it opens the system Settings app via
       on_item_tap → gui_app_run(item->id == APP_ID_SETTING). */
    map_app_id(app_id_setting, &list_items[0]);
    app_base_count = 1;

    /* DEFAULT_APPS: (re)build the built-in app body so a language change picks up
       new translations. PHONE: items [1..] are phone instructions carrying their
       own titles — leave them so a reload does not clobber the phone's list. */
    if (s_list_mode == LIST_MODE_DEFAULT_APPS)
        load_default_apps();
}

/* Device page bracket: device_pager reuses this shared list (save_base → feed →
   restore_base) to show a REMOTE device's options, which are 0-based with no
   pinned Settings entry. It calls this right after save_base to drop the pinned
   prefix for that view; restore_base brings app_base_count (and the Settings item
   at index 0) back for the watch face. Keeps the device page identical to before
   the watch-face Settings pin was added. */
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
        return;

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

static rt_int32_t init(lv_obj_t *parent)
{
    lv_instruction_list_layout_create(parent);
    return RT_EOK;
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
        if (s_global_bar_layer != NULL && lv_obj_is_valid(s_global_bar_layer))
        {
            lv_obj_del(s_global_bar_layer);
        }
        s_global_bar_layer = NULL;
        s_reveal_edge_overlay = NULL; /* chain-deleted with the bar layer above */
        s_reveal_edge_overlay_left = NULL; /* P2 S3: same — chain-deleted */
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
    // extern void weather_widget_stop(void);
    // weather_widget_stop();
    // extern void note_widget_stop(void);
    // note_widget_stop();

    LOG_I("instruction_list_deinit");
    pause_instruction_list = true;
    return RT_EOK;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
