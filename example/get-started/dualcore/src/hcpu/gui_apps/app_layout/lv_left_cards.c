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
#define SUB_W 298
#define BTN_W 236
#define BTN_H 64
#define BTN_Y 334
#define RAIL_MAX 9    /* 右緣圓點最多顯示幾顆 */

static lv_obj_t *s_pager = NULL;
static lv_obj_t *s_rail = NULL;
static const left_card_t *s_cards = NULL;
static uint8_t s_n = 0;
static uint8_t s_cur = 0;
static lv_obj_t *s_card[LEFT_CARDS_MAX];
static bool s_filled[LEFT_CARDS_MAX];
static lv_obj_t *s_dot[RAIL_MAX];
static uint8_t s_dot_n = 0;
static bool s_rounded = false; /* 滑動中:翻頁容器裁成圓形 */
static left_cards_tap_cb_t s_on_tap = NULL;
static left_cards_page_cb_t s_on_page = NULL;

static void card_click_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (s_on_tap != NULL && idx < s_n)
        s_on_tap(idx);
}

static lv_color_t accent_bottom(uint32_t accent)
{
    /* 底端壓到 accent 的 ~30%:小米卡片是「飽和的深色」漸到更深,不是亮色 */
    return lv_color_mix(lv_color_hex(accent), lv_color_black(), 78);
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

/* 右緣圓點:目前這張是白色長條,其餘是暗點。卡片多於 RAIL_MAX 張時圓點數固定,目前位置按比例對應
   (只表示「大概在第幾段」,不是一張一顆)。 */
static uint8_t rail_current_dot(void)
{
    if (s_n <= s_dot_n || s_n < 2)
        return s_cur;
    return (uint8_t)(((uint32_t)s_cur * (s_dot_n - 1) + (s_n - 1) / 2) / (s_n - 1));
}

static void rail_update(void)
{
    if (s_rail == NULL)
        return;
    uint8_t cur_dot = rail_current_dot();
    for (uint8_t i = 0; i < s_dot_n; i++)
    {
        bool on = (i == cur_dot);
        lv_obj_set_size(s_dot[i], 6, on ? 16 : 6);
        lv_obj_set_style_bg_opa(s_dot[i], on ? LV_OPA_COVER : LV_OPA_40, 0);
    }
    /* 尺寸變了,重新對位(圓弧上的 x 依 y 變) */
    const lv_coord_t step = 20;
    lv_coord_t total = (lv_coord_t)(s_dot_n - 1) * step;
    float r = (float)CARD_W / 2.0f;
    for (uint8_t i = 0; i < s_dot_n; i++)
    {
        float d = (float)(-total / 2 + (lv_coord_t)i * step);
        float half = sqrtf(r * r - d * d);
        lv_obj_align(s_dot[i], LV_ALIGN_CENTER, (lv_coord_t)(half - 24), (lv_coord_t)d);
    }
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

static void settle_page(uint8_t idx)
{
    s_cur = idx;
    for (uint8_t i = 0; i < s_n; i++)
    {
        if (i + 1 >= idx && i <= idx + 1)
            fill_card(i);
        else
            clear_card(i);
    }
    rail_update();
    if (s_on_page != NULL)
        s_on_page(idx);
}

static void pager_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_SCROLL_END || s_pager == NULL)
        return;
    uint8_t idx = page_from_scroll();
    if (idx != s_cur)
        settle_page(idx);
}

lv_obj_t *left_cards_show(lv_obj_t *parent, const left_card_t *cards, uint8_t n, uint8_t start,
                          left_cards_tap_cb_t on_tap, left_cards_page_cb_t on_page)
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
    memset(s_card, 0, sizeof(s_card));
    memset(s_filled, 0, sizeof(s_filled));
    s_rounded = false;

    s_pager = lv_obj_create(parent);
    lv_obj_remove_style_all(s_pager);
    lv_obj_set_size(s_pager, CARD_W, CARD_H);
    lv_obj_set_pos(s_pager, 0, 0);
    lv_obj_add_flag(s_pager, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ONE | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_scroll_dir(s_pager, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(s_pager, LV_SCROLL_SNAP_START);
    lv_obj_set_scrollbar_mode(s_pager, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(s_pager, pager_event_cb, LV_EVENT_SCROLL_END, NULL);

    for (uint8_t i = 0; i < n; i++)
    {
        lv_obj_t *card = lv_obj_create(s_pager);
        lv_obj_remove_style_all(card);
        lv_obj_set_size(card, CARD_W, CARD_H);
        lv_obj_set_pos(card, 0, (lv_coord_t)i * CARD_H);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(card, lv_color_hex(cards[i].accent), 0);
        lv_obj_set_style_bg_grad_color(card, accent_bottom(cards[i].accent), 0);
        lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_add_event_cb(card, card_click_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
        s_card[i] = card;
    }

    s_dot_n = (n > 1) ? ((n <= RAIL_MAX) ? n : RAIL_MAX) : 0; /* 只有一張就沒有點點 */
    if (s_dot_n > 0)
    {
        s_rail = lv_obj_create(parent);
        lv_obj_remove_style_all(s_rail);
        lv_obj_set_size(s_rail, CARD_W, CARD_H);
        lv_obj_set_pos(s_rail, 0, 0);
        lv_obj_clear_flag(s_rail, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        for (uint8_t i = 0; i < s_dot_n; i++)
        {
            s_dot[i] = lv_obj_create(s_rail);
            lv_obj_remove_style_all(s_dot[i]);
            lv_obj_set_style_radius(s_dot[i], 3, 0);
            lv_obj_set_style_bg_color(s_dot[i], lv_color_white(), 0);
            lv_obj_clear_flag(s_dot[i], LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        }
    }

    lv_obj_update_layout(s_pager);
    lv_obj_scroll_to_y(s_pager, (lv_coord_t)start * CARD_H, LV_ANIM_OFF);
    settle_page(start);
    return s_pager;
}

void left_cards_hide(void)
{
    /* 先把指標清掉再刪:刪除回呼裡任何人回頭問「還在嗎」都要得到否 */
    lv_obj_t *pager = s_pager;
    lv_obj_t *rail = s_rail;
    s_pager = NULL;
    s_rail = NULL;
    s_n = 0;
    s_cur = 0;
    s_dot_n = 0;
    s_on_tap = NULL;
    s_on_page = NULL;
    s_cards = NULL;
    memset(s_card, 0, sizeof(s_card));
    memset(s_filled, 0, sizeof(s_filled));
    if (rail != NULL && lv_obj_is_valid(rail))
        lv_obj_del(rail);
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
    if (s_cur < s_n && s_card[s_cur] != NULL)
        lv_obj_set_style_radius(s_card[s_cur], sliding ? CARD_W / 2 : 0, 0);
}

bool left_cards_visible(void)
{
    return s_pager != NULL && lv_obj_is_valid(s_pager);
}

uint8_t left_cards_current(void)
{
    return s_cur;
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
