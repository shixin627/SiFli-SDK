/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * 左頁整頁彩色卡片 —— 設計與限制見 lv_left_cards.h。
 */
#include "lv_left_cards.h"
#include <string.h>
#include <math.h>
#include "ui_helper.h"
#include "lv_ext_resource_manager.h"

#define CARD_W LV_HOR_RES
#define CARD_H LV_VER_RES

/* 版面(466x466 圓形螢幕;內容區留在圓內接的安全範圍)。 */
#define X0 84         /* 內容左緣 */
#define ICON_PX 52    /* 左上圖示邊長 */
#define ICON_GAP 14   /* 圖示與標題的間距 */
#define TITLE_Y 98    /* 圖示+標題那一列的上緣 */
#define SUB_Y 170     /* 說明上緣 */
#define SUB_W 280      /* 右緣讓出點點輪盤(最大那顆縮成一半後約 66px) */
#define BTN_W 236
#define BTN_H 64
#define BTN_Y 334

static lv_obj_t *s_pager = NULL;
static const left_card_t *s_cards = NULL;
static uint8_t s_n = 0;
static uint8_t s_cur = 0;
static lv_obj_t *s_card[LEFT_CARDS_MAX];
static bool s_filled[LEFT_CARDS_MAX];
static bool s_rounded = false; /* 滑動中:目前那張卡是圓盤,後面疊光暈 */

/* 柔邊(founder 2026-09-30 給的小米照片:滑入那一頁的前緣是一圈柔化的漸層,不是清楚的線)。
   第一版疊 6 層同色半透明圓盤 → 真機照片(2026-10-01)是一圈一圈清楚分界的亮綠環:幀緩衝是 RGB565,
   暗色區可用色階很少,每一層的一點點變化都被截斷到同幾個色階上,疊起來就是一階一階的斷層(改走漸層
   路徑、想借硬體抖色也沒效果)。所以不疊層:用**一張**放射狀的 A8 透明度圖(lv_left_cards_halo.h,
   64x64,透明度 256 階),用硬體放大蓋住畫面、上卡片的底色,只混一次 —— 透明度是 8 位元、放大時
   EPIC 做內插(2x2 漸層貼圖放大就是靠這個),不會有「每層截斷」的累積誤差。
   只在水平滑動時顯示(靜止與垂直翻頁不付這一次整畫面合成)。 */
#include "lv_left_cards_halo.h"
static lv_obj_t *s_halo_img = NULL;
static const lv_img_dsc_t s_halo_dsc = {
    .header.cf = LV_IMG_CF_ALPHA_8BIT,
    .header.always_zero = 0,
    .header.w = LEFT_CARDS_HALO_N,
    .header.h = LEFT_CARDS_HALO_N,
    .data_size = LEFT_CARDS_HALO_N * LEFT_CARDS_HALO_N,
    .data = left_cards_halo_map,
};
static left_cards_tap_cb_t s_on_tap = NULL;
static left_cards_page_cb_t s_on_page = NULL;
static left_cards_scroll_cb_t s_on_scroll = NULL;

static void card_click_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (s_on_tap != NULL && idx < s_n)
        s_on_tap(idx);
}

static lv_color_t accent_bottom(lv_color_t top)
{
    /* 底端壓到頂端色的 ~30%:小米卡片是「飽和的深色」漸到更深,不是亮色 */
    return lv_color_mix(top, lv_color_black(), 78);
}

static void clear_card(uint8_t i)
{
    if (i >= s_n || s_card[i] == NULL || !s_filled[i])
        return;
    lv_obj_clean(s_card[i]);
    s_filled[i] = false;
}

static void fill_card(uint8_t i)
{
    if (i >= s_n || s_card[i] == NULL || s_filled[i])
        return;
    const left_card_t *c = &s_cards[i];
    lv_obj_t *card = s_card[i];
    /* 字級索引:LVSF_FONT_SMALL..SUPER 由小到大,系統預設是 TITLE,現有介面的一般文字都是
       get_system_font_size(0)。所以卡片標題往大一階(+1 = BIG),說明與按鈕往小一階(-1 =
       SUBTITLE)。PC sim 的 theme 只有兩種 CJK 字面,看不出差異,以真機為準。 */
    const lv_font_t *f_title = LV_EXT_FONT_GET(get_system_font_size(1));
    const lv_font_t *f_sub = LV_EXT_FONT_GET(get_system_font_size(-1));
    const lv_font_t *f_btn = LV_EXT_FONT_GET(get_system_font_size(-1));
    bool has_sub = (c->sub != NULL && c->sub[0] != '\0');
    lv_coord_t row_y = has_sub ? TITLE_Y : (TITLE_Y + 56); /* 沒說明時圖示+標題往中間放 */
    lv_coord_t tx = X0;

    /* 圖示:52px 方塊,原圖(通常 100px)用 zoom 縮。zoom 要 OVERFLOW_VISIBLE 才不被裁。 */
    if (c->icon != NULL)
    {
        lv_obj_t *box = lv_obj_create(card);
        lv_obj_remove_style_all(box);
        lv_obj_set_size(box, ICON_PX, ICON_PX);
        lv_obj_set_pos(box, X0, row_y);
        lv_obj_add_flag(box, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(box, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_t *img = lv_img_create(box);
        lv_img_set_src(img, c->icon);
        lv_img_header_t hdr;
        if (lv_img_decoder_get_info(c->icon, &hdr) == LV_RES_OK && hdr.w > 0)
        {
            lv_img_set_pivot(img, 0, 0);
            lv_img_set_zoom(img, (uint16_t)(256 * ICON_PX / hdr.w));
        }
        lv_obj_set_pos(img, 0, 0);
        lv_obj_add_flag(img, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
        tx = X0 + ICON_PX + ICON_GAP;
    }

    /* 大標題:單行,超寬用 DOT 截斷 */
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, c->title != NULL ? c->title : "");
    lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
    lv_obj_set_width(title, CARD_W - tx - X0 + 14);
    lv_obj_set_style_text_font(title, f_title, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_coord_t th = lv_font_get_line_height(f_title);
    lv_obj_set_pos(title, tx, row_y + (ICON_PX - th) / 2);
    lv_obj_add_flag(title, LV_OBJ_FLAG_EVENT_BUBBLE);

    /* 說明:最多三行,DOT 收尾。色用 label 階層的次要色(bluish #EBEBF5),不是純白半透明。 */
    if (has_sub)
    {
        lv_obj_t *sub = lv_label_create(card);
        lv_label_set_text(sub, c->sub);
        lv_label_set_long_mode(sub, LV_LABEL_LONG_DOT);
        lv_obj_set_size(sub, SUB_W, lv_font_get_line_height(f_sub) * 3 + 8);
        lv_obj_set_style_text_font(sub, f_sub, 0);
        lv_obj_set_style_text_color(sub, lv_color_hex(0xEBEBF5), 0);
        lv_obj_set_style_text_opa(sub, LV_OPA_80, 0);
        lv_obj_set_pos(sub, X0, SUB_Y);
        lv_obj_add_flag(sub, LV_OBJ_FLAG_EVENT_BUBBLE);
    }

    /* 底部膠囊按鈕:半透明白底(在彩色卡上就是「玻璃」),整顆可點 */
    lv_obj_t *btn = lv_obj_create(card);
    lv_obj_remove_style_all(btn);
    lv_obj_set_size(btn, BTN_W, BTN_H);
    lv_obj_set_pos(btn, (CARD_W - BTN_W) / 2, BTN_Y);
    lv_obj_set_style_radius(btn, BTN_H / 2, 0);
    lv_obj_set_style_bg_color(btn, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_20, 0);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(btn, card_click_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, c->btn != NULL ? c->btn : "");
    lv_obj_set_style_text_font(bl, f_btn, 0);
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    lv_obj_center(bl);

    s_filled[i] = true;
}

static uint8_t page_from_scroll(void)
{
    lv_coord_t y = lv_obj_get_scroll_y(s_pager);
    int idx = (int)((y + CARD_H / 2) / CARD_H);
    if (idx < 0)
        idx = 0;
    if (idx >= s_n)
        idx = s_n - 1;
    return (uint8_t)idx;
}

/* 底色層(founder 2026-10-01:「切換過程中保持顏色不變,切換完 0.5 秒沒動後才慢慢變換顏色」)。
   整組卡片共用**一張**底色層(卡片本身是透明的、只帶內容),所以上下翻頁時背景不會跟著一張一張換色;
   最後一次捲動之後再等 BG_SETTLE_MS 才用 BG_FADE_MS 淡到目前停的那一頁的顏色。每一次捲動(翻頁、撥輪盤)
   都把倒數重新計時、把進行到一半的淡入凍在當下顏色,下次停穩再從那個顏色接著淡 —— 不依賴「捲動結束」事件:
   LVGL 會在貼齊動畫跑完之前就先發一次結束事件,之後還有捲動事件,靠事件順序會永遠等不到。
   水平滑入時的圓弧前緣也改畫在這一層(radius),光暈跟著它的顏色。 */
#define BG_SETTLE_MS 500
#define BG_FADE_MS 700
static lv_obj_t *s_bg = NULL;
static lv_color_t s_bg_cur;
static lv_color_t s_bg_from;
static lv_color_t s_bg_to;
static lv_timer_t *s_bg_timer = NULL;
static int32_t s_bg_anim_var;

static void halo_set_color(void)
{
    if (s_halo_img == NULL)
        return;
    lv_obj_set_style_img_recolor(s_halo_img, s_bg_cur, 0);
}

static void bg_apply(lv_color_t top)
{
    s_bg_cur = top;
    if (s_bg != NULL)
    {
        lv_obj_set_style_bg_color(s_bg, top, 0);
        lv_obj_set_style_bg_grad_color(s_bg, accent_bottom(top), 0);
    }
    halo_set_color();
}

static void bg_anim_cb(void *var, int32_t t)
{
    (void)var;
    bg_apply(lv_color_mix(s_bg_to, s_bg_from, (lv_opa_t)t));
}

static void bg_cancel(void)
{
    if (s_bg_timer != NULL)
    {
        lv_timer_del(s_bg_timer);
        s_bg_timer = NULL;
    }
    lv_anim_del(&s_bg_anim_var, bg_anim_cb);
}

static void bg_hold(void);

static void bg_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_bg_timer = NULL; /* 單發計時器,回呼結束後 LVGL 自己刪 */
    if (s_pager == NULL || s_cards == NULL)
        return;
    if (left_cards_busy())
    {
        bg_hold(); /* 手指還按著在拖、或還在滑:接著等 */
        return;
    }
    uint8_t pg = page_from_scroll();
    s_bg_from = s_bg_cur;
    s_bg_to = lv_color_hex(s_cards[pg].accent);
    if (lv_color_eq(s_bg_to, s_bg_from))
        return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_bg_anim_var);
    lv_anim_set_exec_cb(&a, bg_anim_cb);
    lv_anim_set_values(&a, 0, 255);
    lv_anim_set_time(&a, BG_FADE_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
}

/* 每次捲動(以及停到某一頁)呼叫:凍結顏色,並把「停穩後才換色」的倒數重新計時。 */
static void bg_hold(void)
{
    lv_anim_del(&s_bg_anim_var, bg_anim_cb);
    if (s_bg_timer != NULL)
    {
        lv_timer_reset(s_bg_timer);
        return;
    }
    s_bg_timer = lv_timer_create(bg_timer_cb, BG_SETTLE_MS, NULL);
    lv_timer_set_repeat_count(s_bg_timer, 1);
}

static void settle_page(uint8_t idx)
{
    s_cur = idx;
    bg_hold();
    for (uint8_t i = 0; i < s_n; i++)
    {
        if (i + 1 >= idx && i <= idx + 1)
            fill_card(i);
        else
            clear_card(i);
    }
    if (s_on_scroll != NULL)
        s_on_scroll((int32_t)idx * 256);
    if (s_on_page != NULL)
        s_on_page(idx);
}

static void pager_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (s_pager == NULL)
        return;
    if (code == LV_EVENT_SCROLL)
    {
        bg_hold(); /* 翻頁途中顏色不動(進行到一半的淡入也凍在當下),停穩 0.5 秒後才換 */
        if (s_on_scroll != NULL)
            s_on_scroll((int32_t)lv_obj_get_scroll_y(s_pager) * 256 / CARD_H);
        return;
    }
    if (code != LV_EVENT_SCROLL_END)
        return;
    uint8_t idx = page_from_scroll();
    if (idx != s_cur)
        settle_page(idx);
}

lv_obj_t *left_cards_show(lv_obj_t *parent, const left_card_t *cards, uint8_t n, uint8_t start,
                          left_cards_tap_cb_t on_tap, left_cards_page_cb_t on_page,
                          left_cards_scroll_cb_t on_scroll)
{
    left_cards_hide();
    if (parent == NULL || cards == NULL || n == 0)
        return NULL;
    if (n > LEFT_CARDS_MAX)
        n = LEFT_CARDS_MAX;
    if (start >= n)
        start = n - 1;
    s_cards = cards;
    s_n = n;
    s_on_tap = on_tap;
    s_on_page = on_page;
    s_on_scroll = on_scroll;
    memset(s_card, 0, sizeof(s_card));
    memset(s_filled, 0, sizeof(s_filled));
    s_rounded = false;

    /* 柔邊圖比浮層大:預設子物件會被父物件的範圍裁掉,會在浮層邊界被切成一條直線 */
    lv_obj_add_flag(parent, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    s_halo_img = lv_img_create(parent);
    lv_img_set_src(s_halo_img, &s_halo_dsc);
    lv_img_set_pivot(s_halo_img, LEFT_CARDS_HALO_N / 2, LEFT_CARDS_HALO_N / 2);
    lv_img_set_zoom(s_halo_img, LEFT_CARDS_HALO_ZOOM);
    lv_obj_set_pos(s_halo_img, (CARD_W - LEFT_CARDS_HALO_N) / 2, (CARD_H - LEFT_CARDS_HALO_N) / 2);
    /* A8 圖的顏色來自 recolor;recolor_opa 在 A8 上就是整體不透明度(lv_gpu_new_api.c draw_img),
       濃淡已經烤在圖的透明度裡,所以給滿 */
    lv_obj_set_style_img_recolor(s_halo_img, lv_color_hex(cards[start].accent), 0);
    lv_obj_set_style_img_recolor_opa(s_halo_img, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_halo_img, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_halo_img, LV_OBJ_FLAG_HIDDEN);

    /* 底色層:在光暈與翻頁容器之間,整組卡片共用 */
    s_bg = lv_obj_create(parent);
    lv_obj_remove_style_all(s_bg);
    lv_obj_set_size(s_bg, CARD_W, CARD_H);
    lv_obj_set_pos(s_bg, 0, 0);
    lv_obj_set_style_bg_opa(s_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_grad_dir(s_bg, LV_GRAD_DIR_VER, 0);
    lv_obj_clear_flag(s_bg, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    bg_apply(lv_color_hex(cards[start].accent));

    s_pager = lv_obj_create(parent);
    lv_obj_remove_style_all(s_pager);
    lv_obj_set_size(s_pager, CARD_W, CARD_H);
    lv_obj_set_pos(s_pager, 0, 0);
    lv_obj_add_flag(s_pager, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ONE | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_scroll_dir(s_pager, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(s_pager, LV_SCROLL_SNAP_START);
    lv_obj_set_scrollbar_mode(s_pager, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(s_pager, pager_event_cb, LV_EVENT_SCROLL_END, NULL);
    lv_obj_add_event_cb(s_pager, pager_event_cb, LV_EVENT_SCROLL, NULL);

    for (uint8_t i = 0; i < n; i++)
    {
        lv_obj_t *card = lv_obj_create(s_pager);
        lv_obj_remove_style_all(card);
        lv_obj_set_size(card, CARD_W, CARD_H);
        lv_obj_set_pos(card, 0, (lv_coord_t)i * CARD_H);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_add_event_cb(card, card_click_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
        s_card[i] = card;
    }

    lv_obj_update_layout(s_pager);
    lv_obj_scroll_to_y(s_pager, (lv_coord_t)start * CARD_H, LV_ANIM_OFF);
    settle_page(start);
    return s_pager;
}

void left_cards_hide(void)
{
    /* 先把指標清掉再刪:刪除回呼裡任何人回頭問「還在嗎」都要得到否 */
    bg_cancel();
    lv_obj_t *bg = s_bg;
    s_bg = NULL;
    lv_obj_t *halo = s_halo_img;
    s_halo_img = NULL;
    lv_obj_t *pager = s_pager;
    s_pager = NULL;
    s_n = 0;
    s_cur = 0;
    s_on_tap = NULL;
    s_on_page = NULL;
    s_on_scroll = NULL;
    s_cards = NULL;
    memset(s_card, 0, sizeof(s_card));
    memset(s_filled, 0, sizeof(s_filled));
    if (halo != NULL && lv_obj_is_valid(halo))
        lv_obj_del(halo);
    if (bg != NULL && lv_obj_is_valid(bg))
        lv_obj_del(bg);
    if (pager != NULL && lv_obj_is_valid(pager))
        lv_obj_del(pager);
}

void left_cards_set_slide(lv_coord_t tx)
{
    if (!left_cards_visible())
        return;
    bool sliding = (tx != 0);
    if (sliding == s_rounded)
        return;
    s_rounded = sliding;
    /* 用卡片「自己的圓角」,不用 clip_corner:真機的 EPIC 繪圖路徑(lv_gpu_new_api.c)只認圖片遮罩,
       clip_corner 那種圓角遮罩根本不會套用,前緣照舊是直邊(founder 2026-09-30 真機實測「一模一樣」)。
       矩形的圓角+漸層是 EPIC 驅動原生畫的(drv_epic_rl_draw.c 的 rectangle radius),按鈕/聊天氣泡都這樣。
       水平滑動途中視窗裡只有目前這一張,圓盤形的底板不會在垂直翻頁時露出縫。 */
    if (s_bg != NULL)
        lv_obj_set_style_radius(s_bg, sliding ? CARD_W / 2 : 0, 0);
    if (s_halo_img != NULL)
    {
        if (sliding)
            lv_obj_clear_flag(s_halo_img, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(s_halo_img, LV_OBJ_FLAG_HIDDEN);
    }
}

void left_cards_refresh(void)
{
    if (s_pager == NULL || !lv_obj_is_valid(s_pager))
        return;
    for (uint8_t i = 0; i < s_n; i++)
        if (s_filled[i])
        {
            clear_card(i);
            fill_card(i);
        }
}

bool left_cards_visible(void)
{
    return s_pager != NULL && lv_obj_is_valid(s_pager);
}

uint8_t left_cards_current(void)
{
    return s_cur;
}

void left_cards_scroll_to(uint8_t idx, bool anim)
{
    if (!left_cards_visible() || idx >= s_n)
        return;
    lv_obj_scroll_to_y(s_pager, (lv_coord_t)idx * CARD_H, anim ? LV_ANIM_ON : LV_ANIM_OFF);
}

bool left_cards_busy(void)
{
    if (!left_cards_visible())
        return false;
    return lv_obj_is_scrolling(s_pager) || lv_indev_get_scroll_obj(lv_indev_get_act()) == s_pager;
}

bool left_cards_owns(lv_obj_t *obj)
{
    if (!left_cards_visible())
        return false;
    for (; obj != NULL; obj = lv_obj_get_parent(obj))
        if (obj == s_pager)
            return true;
    return false;
}
