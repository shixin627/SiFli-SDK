/*
 * Skai SDK — curated drawing surface for external apps (ADR-0019 Phase 3).
 * See skai_ui.h for why this exists instead of the raw LVGL binding.
 */
#include <string.h>

#include <rtthread.h>

#define DBG_TAG "skai.ui"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "lvgl.h"
#include "ui_handler.h"   /* is_on_lvgl_thread */
#include "lv_ext_resource_manager.h"   /* LV_EXT_FONT_GET, get_system_font_size */

#include "skai/skai_ui.h"

/* The phone app's palette (SkaiLink DESIGN.md), so an app the Bot writes for
 * the watch reads as the same product as the phone it came from — without the
 * Bot having to know a single colour. Sky is the ONE "on / selected / running"
 * signal; systemBlue is the one filled call to action; content sits on solid
 * #1C1C1E cards with a hairline edge on pure black; secondary text is the
 * blue-leaning 60 % grey, never plain white. */
#define SKAI_UI_FG      lv_color_hex(0xFFFFFF)
#define SKAI_UI_ACCENT  lv_color_hex(0xA6D3E6)   /* sky: on-state, progress */
#define SKAI_UI_CTA     lv_color_hex(0x0091FF)   /* systemBlue: primary button */
#define SKAI_UI_CTA_DN  lv_color_hex(0x0074CC)
#define SKAI_UI_TRACK   lv_color_hex(0x2C2C2E)
#define SKAI_UI_SURFACE lv_color_hex(0x1C1C1E)   /* content card */
#define SKAI_UI_PRESSED lv_color_hex(0x2C2C2E)   /* card pressed wash */
#define SKAI_UI_LABEL2  lv_color_hex(0x8D8D93)   /* #EBEBF5 @ 60 % on black */
#define SKAI_UI_GREEN   lv_color_hex(0x30D158)
#define SKAI_UI_ORANGE  lv_color_hex(0xFF9F0A)
#define SKAI_UI_RED     lv_color_hex(0xFF453A)
#define SKAI_UI_RADIUS  24                       /* rCard */
/* Ring stroke. Bounded by the EPIC round-cap mask pool (see skai_ui_arc). */
#define SKAI_UI_ARC_WIDTH 14

/* The type scale an app draws with, in the system's font steps (LVSF_FONT_*:
   20, 24, 28, 36, 40, 64, 90 px on this screen).
 *
 * Fixed, NOT the user's font-size setting plus an offset, which is what every
 * other screen uses. An app is written against a layout the phone checked
 * before install, and a base that moves would make the same program fit on one
 * watch and overflow its ring on another — which is exactly what happened
 * (2026-09-20: the title and the chips overlapped, the countdown spilled out of
 * its ring). The px values are published in skai_ui_theme so the phone's
 * preflight measures with the same numbers. */
static uint8_t font_step(int32_t rel)
{
    int step = (int)rel + 2;          /* rel -2..3 -> step 0..5 */
    if (step < 0) step = 0;
    if (step > 6) step = 6;
    return (uint8_t)step;
}

#define SKAI_UI_ASSET_DIR_MAX 64
#define SKAI_UI_PATH_MAX      (SKAI_UI_ASSET_DIR_MAX + 64)
/* Rows can nest a little; deeper than this is a layout an app should not be
   building by hand. */
#define SKAI_UI_GROUP_DEPTH   4

/* Two containers, because the two placement models do not mix. s_root is the
   full screen and is what ui.align measures against, matching what a built-in
   C app aligns to. s_flow is a padded column for widgets created without an
   explicit position; LVGL flex would otherwise re-place an aligned child on
   every layout pass. */
/* Pages sit above both: s_root and s_flow are always the CURRENT page's pair,
   so every existing call site keeps working unchanged and a one-page app pays
   nothing for the machinery. */
static lv_obj_t *s_tiles;                      /* lv_tileview, the pager */
static lv_obj_t *s_page_root[SKAI_UI_PAGES];
static lv_obj_t *s_page_flow[SKAI_UI_PAGES];
static int       s_page_count;
static int       s_page_cur;

static lv_obj_t *s_root;
static lv_obj_t *s_flow;
static lv_obj_t *s_parent;                     /* current insertion point */
static lv_obj_t *s_groups[SKAI_UI_GROUP_DEPTH];
static int       s_depth;
static lv_obj_t *s_slots[SKAI_UI_SLOTS];
static char      s_asset_dir[SKAI_UI_ASSET_DIR_MAX];

/* Click handlers, one per slot. Kept beside the widget table so detach clears
   both — a handler that outlived its widget would fire on a dead id. */
static skai_ui_click_cb_t s_click_cb[SKAI_UI_SLOTS];
static void              *s_click_arg[SKAI_UI_SLOTS];

/* The padded flex column every page gets. Widgets created without an explicit
   position land here; ui.align pulls them out of it onto the page root. */
static lv_obj_t *make_flow(lv_obj_t *page)
{
    lv_obj_t *f = lv_obj_create(page);

    lv_obj_remove_style_all(f);
    lv_obj_set_size(f, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(f, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(f, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(f, 60, 0);
    lv_obj_set_style_pad_row(f, 8, 0);
    lv_obj_clear_flag(f, LV_OBJ_FLAG_SCROLLABLE);
    return f;
}

/* Which page a widget already lives on. Falls back to the current page for one
   that is still in a flow column and has never been aligned. */
static lv_obj_t *page_root_of(lv_obj_t *o)
{
    for (lv_obj_t *p = o; p != NULL; p = lv_obj_get_parent(p))
        for (int i = 0; i < s_page_count; i++)
            if (p == s_page_root[i])
                return s_page_root[i];
    return s_root;
}

/* Point s_root/s_flow at page `i`. Everything downstream reads those two, so
   this is the whole of "which page am I drawing on". */
static void page_select(int i)
{
    s_page_cur = i;
    s_root = s_page_root[i];
    s_flow = s_page_flow[i];
    s_parent = s_flow;
    s_depth = 0;
}

void skai_ui_attach(void *lv_parent, const char *asset_dir)
{
    skai_ui_detach();

    /* A tileview even for one page. LVGL scrolls only where a tile exists, so
       a single-tile pager behaves exactly like the plain container it replaces
       — and an app that later asks for page 2 does not need its first page
       rebuilt underneath it. */
    s_tiles = lv_tileview_create((lv_obj_t *)lv_parent);
    lv_obj_set_size(s_tiles, LV_PCT(100), LV_PCT(100));
    /* Transparent, not style-stripped: remove_style_all takes the tileview's
       own sizing and scroll setup with it, and the tiles then lay out at the
       top left instead of filling the screen. */
    lv_obj_set_style_bg_opa(s_tiles, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_tiles, 0, 0);
    lv_obj_set_style_pad_all(s_tiles, 0, 0);

    s_page_root[0] = lv_tileview_add_tile(s_tiles, 0, 0, LV_DIR_BOTTOM);
    lv_obj_set_style_bg_opa(s_page_root[0], LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_page_root[0], 0, 0);
    lv_obj_set_style_pad_all(s_page_root[0], 0, 0);
    s_page_flow[0] = make_flow(s_page_root[0]);
    s_page_count = 1;
    page_select(0);

    if (asset_dir)
    {
        rt_strncpy(s_asset_dir, asset_dir, sizeof(s_asset_dir) - 1);
        s_asset_dir[sizeof(s_asset_dir) - 1] = '\0';
    }
}

void skai_ui_detach(void)
{
    /* Only the ids are dropped — the widgets belong to the container, which the
     * host deletes. Clearing the table is what makes a stale id from a previous
     * run unable to address anything in the next one. */
    memset(s_slots, 0, sizeof(s_slots));
    memset(s_click_cb, 0, sizeof(s_click_cb));
    memset(s_click_arg, 0, sizeof(s_click_arg));
    memset(s_groups, 0, sizeof(s_groups));
    memset(s_asset_dir, 0, sizeof(s_asset_dir));
    memset(s_page_root, 0, sizeof(s_page_root));
    memset(s_page_flow, 0, sizeof(s_page_flow));
    s_page_count = 0;
    s_page_cur = 0;
    s_depth = 0;
    s_parent = NULL;
    s_root = NULL;
    s_flow = NULL;
    /* Not deleted: the tileview is a child of the host's container and goes
       with it, same as every widget the script created. */
    s_tiles = NULL;
}

/* Every entry point starts here. Two refusals, both loud:
 * no container (nothing is running) and wrong thread (LVGL is not re-entrant,
 * and a create cannot be deferred because its id is needed now). */
static bool ui_ready(const char *who)
{
    if (s_parent == NULL)
        return false;
    if (!is_on_lvgl_thread())
    {
        LOG_W("%s called off the LVGL thread — refused", who);
        return false;
    }
    return true;
}

int32_t skai_ui_page(void)
{
    lv_obj_t *tile;

    if (!ui_ready("ui.page"))
        return 0;
    if (s_page_count >= SKAI_UI_PAGES)
    {
        LOG_W("ui.page: %d pages is the budget", SKAI_UI_PAGES);
        return 0;
    }

    /* Each new tile can be reached from the one above it, and can go back. */
    tile = lv_tileview_add_tile(s_tiles, 0, (uint8_t)s_page_count,
                                LV_DIR_TOP | LV_DIR_BOTTOM);
    if (tile == NULL)
        return 0;
    lv_obj_set_style_bg_opa(tile, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(tile, 0, 0);
    lv_obj_set_style_pad_all(tile, 0, 0);

    s_page_root[s_page_count] = tile;
    s_page_flow[s_page_count] = make_flow(tile);
    s_page_count++;

    /* Drawing continues on the new page: an app calls ui.page() between the
       page it just finished and the one it is about to build. */
    page_select(s_page_count - 1);
    return (int32_t)s_page_cur;
}

bool skai_ui_goto_page(int32_t index)
{
    if (!ui_ready("ui.goto_page"))
        return false;
    if (index < 0 || index >= s_page_count)
        return false;

    /* ponytail: no forced layout here. A tile has no coordinates until the
       tileview lays out, and the whole script runs before the first layout
       pass — so calling this from top-level script code does nothing, and
       forcing lv_obj_update_layout() to make it work scrolls the host's own
       container sideways instead. Known ceiling: goto_page takes effect from a
       handler (click, on_change) and is ignored during the first paint, which
       is the only case an app cannot express by simply drawing page 0 first.
       Revisit if an app genuinely needs to open on page N. */
    lv_obj_set_tile(s_tiles, s_page_root[index], LV_ANIM_ON);
    return true;
}


/* A widget can now die without going through the slot table -- ui.remove on a
   list takes every row inside it -- so the table learns about deletions from
   LVGL itself. Without this a removed row's id would keep pointing at freed
   memory, and a later click on whatever reused that memory would reach the old
   handler. */
static struct { int16_t ref; uint8_t side; int16_t dx, dy; } s_alto[SKAI_UI_SLOTS];

static void slot_deleted_cb(lv_event_t *e)
{
    int32_t id = (int32_t)(intptr_t)lv_event_get_user_data(e);

    if (id < 1 || id > SKAI_UI_SLOTS)
        return;
    if (s_slots[id - 1] != lv_event_get_target(e))
        return; /* already released (clear/detach), or the slot was reused */
    s_slots[id - 1] = NULL;
    s_click_cb[id - 1] = NULL;
    s_click_arg[id - 1] = NULL;
    s_alto[id - 1].ref = 0;
}

static int32_t slot_alloc(lv_obj_t *obj)
{
    for (int i = 0; i < SKAI_UI_SLOTS; i++)
    {
        if (s_slots[i] == NULL)
        {
            s_slots[i] = obj;
            s_click_cb[i] = NULL;   /* a reused id starts with no handler */
            s_alto[i].ref = 0;
            s_click_arg[i] = NULL;
            lv_obj_add_event_cb(obj, slot_deleted_cb, LV_EVENT_DELETE,
                                (void *)(intptr_t)(i + 1));
            return i + 1; /* ids are 1-based so 0 can mean failure */
        }
    }
    LOG_W("all %d ui slots in use", SKAI_UI_SLOTS);
    lv_obj_del(obj);
    return 0;
}

static lv_obj_t *slot_of(int32_t id)
{
    if (id < 1 || id > SKAI_UI_SLOTS)
        return NULL;
    return s_slots[id - 1];
}

static int32_t clamp_pct(int32_t v)
{
    return (v < 0) ? 0 : ((v > 100) ? 100 : v);
}

int32_t skai_ui_label(const char *text)
{
    lv_obj_t *l;

    if (!ui_ready("ui.label") || text == NULL)
        return 0;

    l = lv_label_create(s_parent);
    if (l == NULL)
        return 0;
    /* lv_label copies the text, so the caller's buffer (a QuickJS CString that
     * is freed right after the call) is not retained. */
    lv_label_set_text(l, text);
    lv_obj_set_style_text_color(l, SKAI_UI_FG, 0);
    return slot_alloc(l);
}

int32_t skai_ui_arc(int32_t percent)
{
    lv_obj_t *a;

    if (!ui_ready("ui.arc"))
        return 0;

    a = lv_arc_create(s_parent);
    if (a == NULL)
        return 0;
    lv_obj_set_size(a, 140, 140);
    lv_arc_set_rotation(a, 270);
    lv_arc_set_bg_angles(a, 0, 360);
    lv_arc_set_range(a, 0, 100);
    lv_arc_set_value(a, clamp_pct(percent));
    /* The stroke is set here, never left to the theme: the EPIC renderer builds
       a round-cap mask of width×width bytes in a 1600-byte pool and ASSERTS
       when it does not fit (drv_epic_rl_draw.c:703, width <= 40). A full or
       empty ring skips that path, so a theme-width ring looks fine until a
       countdown moves it off 0 % — and then the watch resets mid-tick
       (2026-09-20, a Bot-written timer). 14 px is inside the limit whatever
       size the app gives the arc. */
    lv_obj_set_style_arc_width(a, SKAI_UI_ARC_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_arc_width(a, SKAI_UI_ARC_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(a, true, LV_PART_INDICATOR);
    /* No knob and no input: an external app draws a gauge, it does not get a
     * control the user can drag into the app's own event handlers. */
    lv_obj_remove_style(a, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(a, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(a, SKAI_UI_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_color(a, SKAI_UI_ACCENT, LV_PART_INDICATOR);
    return slot_alloc(a);
}

bool skai_ui_set_text(int32_t id, const char *text)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_text") || o == NULL || text == NULL)
        return false;
    if (lv_obj_check_type(o, &lv_label_class))
    {
        lv_label_set_text(o, text);
        return true;
    }
    /* A list row or button carries its caption in its child label, and a
       checkbox owns its own text: a step that shows its countdown in its own
       row is the ordinary case, not a hack. */
    if (lv_obj_check_type(o, &lv_checkbox_class))
    {
        lv_checkbox_set_text(o, text);
        return true;
    }
    if (lv_obj_check_type(o, &lv_btn_class))
    {
        lv_obj_t *l = lv_obj_get_child(o, 0);
        if (l && lv_obj_check_type(l, &lv_label_class))
        {
            lv_label_set_text(l, text);
            return true;
        }
    }
    return false; /* wrong widget kind — refuse, do not reinterpret */
}

bool skai_ui_set_arc(int32_t id, int32_t percent)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_arc") || o == NULL)
        return false;
    if (!lv_obj_check_type(o, &lv_arc_class))
        return false;
    lv_arc_set_value(o, clamp_pct(percent));
    return true;
}

bool skai_ui_clear(void)
{
    if (!ui_ready("ui.clear"))
        return false;
    for (int i = 0; i < SKAI_UI_SLOTS; i++)
    {
        if (s_slots[i] != NULL)
        {
            lv_obj_del(s_slots[i]);
            s_slots[i] = NULL;
        }
    }
    memset(s_click_cb, 0, sizeof(s_click_cb));
    memset(s_click_arg, 0, sizeof(s_click_arg));
    memset(s_groups, 0, sizeof(s_groups));

    /* Pages go too, or "clear and redraw" — the obvious way to handle a data
       push — grows a tile every time and hits the page budget after three
       refreshes. Page 0 stays: clearing is not detaching, and an app that
       cleared its way to no screen at all would have nowhere to draw. */
    for (int i = s_page_count - 1; i > 0; i--)
    {
        lv_obj_del(s_page_root[i]);   /* takes its flow column with it */
        s_page_root[i] = NULL;
        s_page_flow[i] = NULL;
    }
    s_page_count = 1;
    page_select(0);

    /* Page 0's flow column survived the slot sweep (it is not a slot), but any
       widget still parented to it was. Nothing to delete — just draw again. */
    return true;
}

/* ── custom images ── */

/* The entire security story for app-supplied images: the path may not leave
 * the app's own directory. Rejects absolute paths, drive letters, backslashes
 * and any ".." segment. Deliberately a whitelist of shapes rather than a
 * blacklist of tricks — "contains .." misses "a/../../b" spellings that a
 * segment walk catches. */
bool skai_ui_path_ok(const char *rel)
{
    const char *p = rel;

    if (rel == NULL || rel[0] == '\0')
        return false;
    if (strlen(rel) > 63)
        return false;
    if (rel[0] == '/' || rel[0] == '\\')
        return false;            /* absolute */
    if (strchr(rel, ':') != NULL)
        return false;            /* drive letter or LVGL drive prefix */
    if (strchr(rel, '\\') != NULL)
        return false;            /* one separator spelling only */

    /* Walk segments; reject "." and ".." outright. */
    while (*p)
    {
        const char *slash = strchr(p, '/');
        size_t len = slash ? (size_t)(slash - p) : strlen(p);
        if (len == 0)
            return false;                       /* "" from "//" or trailing / */
        if (len == 1 && p[0] == '.')
            return false;
        if (len == 2 && p[0] == '.' && p[1] == '.')
            return false;
        if (!slash)
            break;
        p = slash + 1;
    }
    return true;
}

int32_t skai_ui_image(const char *rel_path)
{
    char full[SKAI_UI_PATH_MAX];
    lv_obj_t *img;

    if (!ui_ready("ui.image"))
        return 0;
    if (s_asset_dir[0] == '\0')
    {
        LOG_W("ui.image: app has no asset directory");
        return 0;   /* fail closed */
    }
    if (!skai_ui_path_ok(rel_path))
    {
        LOG_W("ui.image: path '%s' refused", rel_path ? rel_path : "(null)");
        return 0;
    }

    rt_snprintf(full, sizeof(full), "%s/%s", s_asset_dir, rel_path);

    img = lv_img_create(s_parent);
    if (img == NULL)
        return 0;
    /* ponytail: LVGL's own file loading and the project's existing decoders do
     * the work — no packaging format and no decoder of ours to get wrong. A
     * missing or malformed file leaves an empty image object, which is the
     * behaviour LVGL already has for every other caller. Decode memory is
     * charged to the app's JS quota, so a huge asset starves the app, not the
     * watch. Add a pixel-dimension cap here if that stops being true. */
    lv_img_set_src(img, full);
    return slot_alloc(img);
}

/* The system icon set. A flat table on purpose: adding an icon is one row, the
 * same "one row" the dispatch table promises for capabilities, and there is no
 * registry to keep in sync with anything.
 *
 * Names are generic rather than asset names ("weather.rain", not the symbol
 * weather_thunder) so the firmware can repoint one at a different asset without
 * breaking apps already on people's wrists — the name is the frozen part, the
 * pixels are not.
 *
 * ponytail: only what a real app has asked for so far. This is not a catalogue
 * of every image in the firmware, and it should not become one by default —
 * every name here is a promise that outlives the app that wanted it. */
LV_IMG_DECLARE(weather_sun);
LV_IMG_DECLARE(weather_clear);
LV_IMG_DECLARE(weather_cloudy);
LV_IMG_DECLARE(weather_rain);
LV_IMG_DECLARE(weather_thunder);
LV_IMG_DECLARE(btn_flashlight);

static const struct
{
    const char *name;
    const void *src;
} s_icons[] =
{
    { "weather.sun",     &weather_sun     },
    { "weather.clear",   &weather_clear   },
    { "weather.cloudy",  &weather_cloudy  },
    { "weather.rain",    &weather_rain    },
    { "weather.thunder", &weather_thunder },
    { "flashlight",      &btn_flashlight  },
};

static const void *icon_src(const char *name)
{
    for (unsigned i = 0; i < sizeof(s_icons) / sizeof(s_icons[0]); i++)
        if (strcmp(name, s_icons[i].name) == 0)
            return s_icons[i].src;
    return NULL;
}

int32_t skai_ui_icon(const char *name)
{
    lv_obj_t *img;
    const void *src;

    if (!ui_ready("ui.icon") || name == NULL)
        return 0;

    src = icon_src(name);
    if (src != NULL)
    {
        img = lv_img_create(s_parent);
        if (img == NULL)
            return 0;
        lv_img_set_src(img, src);
        return slot_alloc(img);
    }

    /* Unknown name is not an error the app has to handle: it draws nothing and
       returns 0, so an app built against newer firmware degrades instead of
       failing. Logged because from the developer's side it looks like nothing
       happened. */
    LOG_W("ui.icon: no icon named '%s'", name);
    return 0;
}

bool skai_ui_set_icon(int32_t id, const char *name)
{
    lv_obj_t *o = slot_of(id);
    const void *src;

    if (!ui_ready("ui.set_icon") || o == NULL || name == NULL)
        return false;
    if (!lv_obj_check_type(o, &lv_img_class))
        return false;   /* wrong widget kind -- refuse, do not reinterpret */

    src = icon_src(name);
    if (src == NULL)
    {
        LOG_W("ui.set_icon: no icon named '%s'", name);
        return false;   /* leaves the previous picture, which is the sane one */
    }
    lv_img_set_src(o, src);
    return true;
}


/* ── grouping ── */

int32_t skai_ui_row(void)
{
    lv_obj_t *g;

    if (!ui_ready("ui.row"))
        return 0;
    if (s_depth >= SKAI_UI_GROUP_DEPTH)
    {
        LOG_W("ui.row nested deeper than %d", SKAI_UI_GROUP_DEPTH);
        return 0;
    }

    g = lv_obj_create(s_parent);
    if (g == NULL)
        return 0;
    lv_obj_remove_style_all(g);
    lv_obj_set_size(g, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(g, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(g, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(g, 8, 0);

    s_groups[s_depth++] = s_parent;  /* remember where to go back to */
    s_parent = g;
    return slot_alloc(g);
}

bool skai_ui_end(void)
{
    if (!ui_ready("ui.end") || s_depth == 0)
        return false;
    s_parent = s_groups[--s_depth];
    return true;
}

/* ── button, bar, colour ── */

static void click_trampoline(lv_event_t *e)
{
    int32_t id = (int32_t)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t *o = slot_of(id);
    const char *text = "";

    /* The widget may have been cleared between the click and here. */
    if (o == NULL || o != lv_event_get_target(e))
        return;
    if (!s_click_cb[id - 1])
        return;

    if (lv_obj_check_type(o, &lv_btnmatrix_class))
    {
        const char *k = lv_btnmatrix_get_btn_text(o, lv_btnmatrix_get_selected_btn(o));
        text = k ? k : "";
    }
    else if (lv_obj_check_type(o, &lv_checkbox_class) || lv_obj_check_type(o, &lv_switch_class))
    {
        text = lv_obj_has_state(o, LV_STATE_CHECKED) ? "1" : "0";
    }
    else if (lv_obj_check_type(o, &lv_slider_class))
    {
        static char vbuf[12];
        rt_snprintf(vbuf, sizeof(vbuf), "%d", (int)lv_slider_get_value(o));
        text = vbuf;
    }
    else
    {
        /* A button carries its caption in its child label. */
        lv_obj_t *l = lv_obj_get_child(o, 0);
        if (l && lv_obj_check_type(l, &lv_label_class))
            text = lv_label_get_text(l);
    }
    s_click_cb[id - 1](id, text, s_click_arg[id - 1]);
}

int32_t skai_ui_button(const char *text)
{
    lv_obj_t *b, *l;
    int32_t id;

    if (!ui_ready("ui.button") || text == NULL)
        return 0;

    b = lv_btn_create(s_parent);
    if (b == NULL)
        return 0;
    lv_obj_set_style_bg_color(b, SKAI_UI_CTA, 0);
    lv_obj_set_style_bg_color(b, SKAI_UI_CTA_DN, LV_STATE_PRESSED);
    lv_obj_set_style_radius(b, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_min_height(b, 56, 0);
    lv_obj_set_style_pad_hor(b, 28, 0);
    lv_obj_set_style_pad_ver(b, 12, 0);
    lv_obj_set_style_text_color(b, SKAI_UI_FG, 0);

    l = lv_label_create(b);
    if (l != NULL)
    {
        lv_label_set_text(l, text);
        lv_obj_center(l);
    }

    id = slot_alloc(b);
    if (id > 0)
        lv_obj_add_event_cb(b, click_trampoline, LV_EVENT_CLICKED,
                            (void *)(intptr_t)id);
    return id;
}

bool skai_ui_on_click(int32_t id, skai_ui_click_cb_t cb, void *arg)
{
    lv_obj_t *o = slot_of(id);

    if (o == NULL)
        return false;
    s_click_cb[id - 1] = cb;
    s_click_arg[id - 1] = arg;
    /* Anything an app asks to be tappable is tappable. Rows, labels and the like
       used to take the handler and then never fire it, because only buttons, list
       rows and boxes were wired for clicks at creation — a Bot-written app put its
       handler on a ui.row and every tap on the wrist was dead (2026-09-20).
       Controls with a value keep their VALUE_CHANGED wiring; for the rest the
       CLICKED trampoline is (re)attached once. */
    if (!lv_obj_check_type(o, &lv_checkbox_class) && !lv_obj_check_type(o, &lv_switch_class) &&
            !lv_obj_check_type(o, &lv_slider_class) && !lv_obj_check_type(o, &lv_btnmatrix_class))
    {
        lv_obj_add_flag(o, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_event_cb_with_user_data(o, click_trampoline, (void *)(intptr_t)id);
        lv_obj_add_event_cb(o, click_trampoline, LV_EVENT_CLICKED, (void *)(intptr_t)id);
    }
    return true;
}

int32_t skai_ui_bar(int32_t percent)
{
    lv_obj_t *b;

    if (!ui_ready("ui.bar"))
        return 0;
    b = lv_bar_create(s_parent);
    if (b == NULL)
        return 0;
    lv_obj_set_size(b, 200, 8);
    lv_bar_set_range(b, 0, 100);
    lv_bar_set_value(b, clamp_pct(percent), LV_ANIM_OFF);
    lv_obj_set_style_bg_color(b, SKAI_UI_TRACK, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b, SKAI_UI_ACCENT, LV_PART_INDICATOR);
    return slot_alloc(b);
}

bool skai_ui_set_color(int32_t id, int32_t rgb)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_color") || o == NULL)
        return false;
    lv_obj_set_style_text_color(o, lv_color_hex((uint32_t)rgb & 0xFFFFFFu), 0);
    /* A button matrix paints its keys, not itself. */
    if (lv_obj_check_type(o, &lv_btnmatrix_class))
        lv_obj_set_style_text_color(o, lv_color_hex((uint32_t)rgb & 0xFFFFFFu),
                                    LV_PART_ITEMS);
    return true;
}

bool skai_ui_set_bg(int32_t id, int32_t rgb)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_bg") || o == NULL)
        return false;
    lv_obj_set_style_bg_color(o, lv_color_hex((uint32_t)rgb & 0xFFFFFFu), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    return true;
}

bool skai_ui_set_font(int32_t id, int32_t rel_size)
{
    lv_obj_t *o = slot_of(id);
    int32_t rel = (rel_size < -2) ? -2 : ((rel_size > 3) ? 3 : rel_size);

    if (!ui_ready("ui.set_font") || o == NULL)
        return false;
    /* Relative to the user's chosen system size, so a JS app scales with the
     * watch instead of pinning a pixel height. */
    lv_obj_set_style_text_font(o, LV_EXT_FONT_GET(font_step(rel)), 0);
    if (lv_obj_check_type(o, &lv_btnmatrix_class))
        lv_obj_set_style_text_font(o, LV_EXT_FONT_GET(font_step(rel)),
                                   LV_PART_ITEMS);
    return true;
}

bool skai_ui_set_size(int32_t id, int32_t w, int32_t h)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_size") || o == NULL)
        return false;
    if (w <= 0 || h <= 0 || w > 1024 || h > 1024)
        return false;   /* bounded: no negative or absurd geometry from script */
    lv_obj_set_size(o, (lv_coord_t)w, (lv_coord_t)h);
    return true;
}

bool skai_ui_align(int32_t id, const char *anchor, int32_t dx, int32_t dy)
{
    static const struct { const char *name; lv_align_t a; } k_anchors[] =
    {
        { "center",       LV_ALIGN_CENTER },
        { "top",          LV_ALIGN_TOP_MID },
        { "bottom",       LV_ALIGN_BOTTOM_MID },
        { "left",         LV_ALIGN_LEFT_MID },
        { "right",        LV_ALIGN_RIGHT_MID },
        { "top_left",     LV_ALIGN_TOP_LEFT },
        { "top_right",    LV_ALIGN_TOP_RIGHT },
        { "bottom_left",  LV_ALIGN_BOTTOM_LEFT },
        { "bottom_right", LV_ALIGN_BOTTOM_RIGHT },
    };
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.align") || o == NULL || anchor == NULL)
        return false;
    if (dx < -1024 || dx > 1024 || dy < -1024 || dy > 1024)
        return false;

    for (size_t i = 0; i < sizeof(k_anchors) / sizeof(k_anchors[0]); i++)
    {
        if (strcmp(anchor, k_anchors[i].name) == 0)
        {
            /* Explicit placement means leaving the flow for good: reparent to
               the full screen and opt out of layout, or the next flex pass
               would move the widget straight back. */
            /* The widget's OWN page, not whichever one is current: an app that
               aligns a page-0 widget after calling ui.page() would otherwise
               have it silently jump to page 1. */
            lv_obj_t *page = page_root_of(o);

            lv_obj_set_parent(o, page);
            lv_obj_add_flag(o, LV_OBJ_FLAG_IGNORE_LAYOUT);

            /* ...but lv_obj_set_parent appends as the NEWEST child, so without
               the line below the stacking order would be the order ui.align
               happened to be called in. That is not a rule anyone expects, and
               it is not the rule anywhere else: an app that never calls align
               stacks by creation order, and so does every C app in this
               firmware. Restoring the slot order here makes it one rule —
               later-created draws on top — whether a widget was aligned or not.

               Slot ids are handed out in creation order, so the target index is
               just how many earlier widgets are already on the root. Index 0 is
               reserved for s_flow, which has to stay underneath. */
            {
                int32_t idx = 1;
                for (int32_t s = 0; s < id - 1; s++)
                    if (s_slots[s] != NULL && lv_obj_get_parent(s_slots[s]) == page)
                        idx++;
                lv_obj_move_to_index(o, idx);
            }

            lv_obj_align(o, k_anchors[i].a, (lv_coord_t)dx, (lv_coord_t)dy);
            return true;
        }
    }
    LOG_W("ui.align: unknown anchor '%s'", anchor);
    return false;
}

/* ui.align_to is a relation, not a one-off move: "the number in the middle of
   the ring" must stay in the middle when the number gets longer. LVGL's own
   align_to is computed once, so a label aligned while empty and filled a line
   later sat off-centre. Remember the relation per slot and re-apply it whenever
   the widget changes size. s_alto is declared beside the slot table. */

static void realign_cb(lv_event_t *e)
{
    int32_t id = (int32_t)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t *o = slot_of(id);
    lv_obj_t *ref;

    if (o == NULL || o != lv_event_get_target(e) || s_alto[id - 1].ref == 0)
        return;
    ref = slot_of(s_alto[id - 1].ref);
    if (ref == NULL)
        return;
    lv_obj_align_to(o, ref, (lv_align_t)s_alto[id - 1].side,
                    s_alto[id - 1].dx, s_alto[id - 1].dy);
}

bool skai_ui_align_to(int32_t id, int32_t ref_id, const char *side,
                      int32_t dx, int32_t dy)
{
    static const struct { const char *name; lv_align_t a; } k_sides[] =
    {
        { "below",  LV_ALIGN_OUT_BOTTOM_MID },
        { "above",  LV_ALIGN_OUT_TOP_MID    },
        { "left",   LV_ALIGN_OUT_LEFT_MID   },
        { "right",  LV_ALIGN_OUT_RIGHT_MID  },
        { "center", LV_ALIGN_CENTER         },
        /* The one thing the five above cannot say: "under it, left edges
           flush". A 300 px rule drawn under a 50 px date label has to START
           where the label starts, not sit centred on it — which is what the
           daily weather page does (app_weather.c:484, OUT_BOTTOM_LEFT). Six
           names out of LVGL's twelve OUT_* anchors, still a closed table. */
        { "below_left", LV_ALIGN_OUT_BOTTOM_LEFT },
    };
    lv_obj_t *o = slot_of(id);
    lv_obj_t *ref = slot_of(ref_id);

    if (!ui_ready("ui.align_to") || o == NULL || ref == NULL || side == NULL)
        return false;
    if (o == ref)
        return false;   /* aligning to itself is a loop, not a layout */
    if (dx < -1024 || dx > 1024 || dy < -1024 || dy > 1024)
        return false;

    for (size_t i = 0; i < sizeof(k_sides) / sizeof(k_sides[0]); i++)
    {
        if (strcmp(side, k_sides[i].name) != 0)
            continue;

        /* Both have to be on the same page and out of the flow column, for the
           same reason ui.align reparents: flex would re-place them next pass. */
        {
            lv_obj_t *page = page_root_of(ref);

            lv_obj_set_parent(o, page);
            lv_obj_add_flag(o, LV_OBJ_FLAG_IGNORE_LAYOUT);

            int32_t idx = 1;
            for (int32_t s = 0; s < id - 1; s++)
                if (s_slots[s] != NULL && lv_obj_get_parent(s_slots[s]) == page)
                    idx++;
            lv_obj_move_to_index(o, idx);
        }
        lv_obj_align_to(o, ref, k_sides[i].a, (lv_coord_t)dx, (lv_coord_t)dy);
        if (s_alto[id - 1].ref == 0)
            lv_obj_add_event_cb(o, realign_cb, LV_EVENT_SIZE_CHANGED, (void *)(intptr_t)id);
        s_alto[id - 1].ref = (int16_t)ref_id;
        s_alto[id - 1].side = (uint8_t)k_sides[i].a;
        s_alto[id - 1].dx = (int16_t)dx;
        s_alto[id - 1].dy = (int16_t)dy;
        return true;
    }
    LOG_W("ui.align_to: unknown side '%s'", side);
    return false;
}

/* ── lists and controls ── */

int32_t skai_ui_list(void)
{
    lv_obj_t *g;
    int32_t id;

    if (!ui_ready("ui.list"))
        return 0;
    if (s_depth >= SKAI_UI_GROUP_DEPTH)
    {
        LOG_W("ui.list nested deeper than %d", SKAI_UI_GROUP_DEPTH);
        return 0;
    }
    g = lv_obj_create(s_parent);
    if (g == NULL)
        return 0;
    lv_obj_remove_style_all(g);
    /* Fills what is left of the page: the flow column is a fixed-height flex
       column, so grow takes the remaining height and anything created after
       ui.end() still has room below. */
    lv_obj_set_width(g, LV_PCT(100));
    lv_obj_set_flex_grow(g, 1);
    lv_obj_set_flex_flow(g, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(g, 8, 0);
    /* Room at the bottom so the last row can scroll clear of the round
       screen's narrow edge. */
    lv_obj_set_style_pad_top(g, 4, 0);
    lv_obj_set_style_pad_bottom(g, 40, 0);
    lv_obj_add_flag(g, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(g, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(g, LV_SCROLLBAR_MODE_OFF);
    /* A list inside a page must not hand its vertical drag to the pager. */
    lv_obj_clear_flag(g, LV_OBJ_FLAG_SCROLL_CHAIN_VER);

    id = slot_alloc(g);
    if (id == 0)
        return 0;
    s_groups[s_depth++] = s_parent;
    s_parent = g;
    return id;
}

/* True while the insertion point is a ui.list(): list rows stretch to its width. */
static bool in_list(void)
{
    return s_depth > 0 && lv_obj_has_flag(s_parent, LV_OBJ_FLAG_SCROLLABLE) &&
           lv_obj_get_scroll_dir(s_parent) == LV_DIR_VER;
}

/* A content card as the phone draws one: solid surface, hairline edge, the
   card radius, a pressed wash instead of a ripple or a shadow. */
static void card_style(lv_obj_t *o)
{
    lv_obj_set_style_min_height(o, 64, 0);
    lv_obj_set_style_radius(o, SKAI_UI_RADIUS, 0);
    lv_obj_set_style_bg_color(o, SKAI_UI_SURFACE, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(o, SKAI_UI_PRESSED, LV_STATE_PRESSED);
    lv_obj_set_style_border_color(o, SKAI_UI_FG, 0);
    lv_obj_set_style_border_opa(o, LV_OPA_10, 0);
    lv_obj_set_style_border_width(o, 1, 0);
    lv_obj_set_style_shadow_width(o, 0, 0);
    lv_obj_set_style_pad_hor(o, 20, 0);
    lv_obj_set_style_pad_ver(o, 14, 0);
}

const skai_ui_token_t skai_ui_theme[] =
{
    /* colours, 0xRRGGBB */
    { "bg",          0x000000 },   /* page: pure black */
    { "surface",     0x1C1C1E },   /* content card */
    { "surface2",    0x2C2C2E },   /* raised / pressed / track */
    { "text",        0xFFFFFF },
    { "text2",       0x8D8D93 },   /* secondary: #EBEBF5 at 60 % */
    { "text3",       0x48484A },   /* tertiary / disabled */
    { "accent",      0xA6D3E6 },   /* sky: THE on / selected / running signal */
    { "accent_deep", 0x5C9CB8 },
    { "cta",         0x0091FF },   /* the one filled primary action */
    { "green",       0x30D158 },   /* done / ok */
    { "orange",      0xFF9F0A },   /* warning */
    { "red",         0xFF453A },   /* alert / destructive */
    { "yellow",      0xFFD600 },
    { "purple",      0xBF5AF2 },
    /* metrics, px */
    { "radius",      SKAI_UI_RADIUS },  /* cards */
    { "radius_sm",   12 },         /* chips, small tiles */
    { "pill",        240 },        /* capsule / circle */
    { "pad",         16 },         /* card inner padding */
    { "gap",         8 },          /* between siblings (8-pt grid: 4 8 12 16 24 32) */
    { "tap",         56 },         /* minimum touch target */
    { "screen",      466 },        /* round, diameter */
    { "safe",        330 },        /* square fully inside the circle */
    /* type: the step for ui.set_font, and what that step is in px on the screen
       (LVSF_FONT_SMALL..SUPER = 20, 24, 28, 36, 40, 64, 90), so the phone can
       lay a screen out before it is installed. */
    { "font_px_caption", 24 },
    { "font_px_body",    28 },
    { "font_px_title",   40 },
    { "font_px_display", 64 },
    /* type, relative sizes for ui.set_font */
    { "font_display", 3 },         /* one big number */
    { "font_title",   2 },
    { "font_body",    0 },
    { "font_caption", -1 },
};
const int skai_ui_theme_count = (int)(sizeof(skai_ui_theme) / sizeof(skai_ui_theme[0]));

int32_t skai_ui_box(void)
{
    lv_obj_t *g;
    int32_t id;

    if (!ui_ready("ui.box"))
        return 0;
    if (s_depth >= SKAI_UI_GROUP_DEPTH)
    {
        LOG_W("ui.box nested deeper than %d", SKAI_UI_GROUP_DEPTH);
        return 0;
    }
    g = lv_obj_create(s_parent);
    if (g == NULL)
        return 0;
    lv_obj_remove_style_all(g);
    lv_obj_set_size(g, in_list() ? LV_PCT(100) : LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    card_style(g);
    lv_obj_set_style_pad_all(g, 16, 0);
    lv_obj_set_style_min_height(g, 0, 0);
    lv_obj_set_flex_flow(g, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(g, 8, 0);
    lv_obj_clear_flag(g, LV_OBJ_FLAG_SCROLLABLE);
    /* Tappable like a row: the pressed wash is the feedback. A drag that starts
       on it still scrolls the list it sits in. */
    lv_obj_add_flag(g, LV_OBJ_FLAG_CLICKABLE);
    id = slot_alloc(g);
    if (id == 0)
        return 0;
    lv_obj_add_event_cb(g, click_trampoline, LV_EVENT_CLICKED, (void *)(intptr_t)id);
    s_groups[s_depth++] = s_parent;
    s_parent = g;
    return id;
}

static lv_coord_t clamp_px(int32_t v, int32_t hi)
{
    return (lv_coord_t)((v < 0) ? 0 : ((v > hi) ? hi : v));
}

bool skai_ui_set_radius(int32_t id, int32_t px)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_radius") || o == NULL)
        return false;
    lv_obj_set_style_radius(o, px >= 240 ? LV_RADIUS_CIRCLE : clamp_px(px, 240), 0);
    return true;
}

bool skai_ui_set_border(int32_t id, int32_t rgb, int32_t width)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_border") || o == NULL)
        return false;
    lv_obj_set_style_border_color(o, lv_color_hex((uint32_t)rgb & 0xFFFFFFu), 0);
    lv_obj_set_style_border_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(o, clamp_px(width, 8), 0);
    return true;
}

bool skai_ui_set_pad(int32_t id, int32_t px)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_pad") || o == NULL)
        return false;
    lv_obj_set_style_pad_all(o, clamp_px(px, 64), 0);
    return true;
}

bool skai_ui_set_gap(int32_t id, int32_t px)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_gap") || o == NULL)
        return false;
    lv_obj_set_style_pad_row(o, clamp_px(px, 64), 0);
    lv_obj_set_style_pad_column(o, clamp_px(px, 64), 0);
    return true;
}

bool skai_ui_set_bg_opa(int32_t id, int32_t percent)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_bg_opa") || o == NULL)
        return false;
    lv_obj_set_style_bg_opa(o, (lv_opa_t)(clamp_pct(percent) * 255 / 100), 0);
    return true;
}

int32_t skai_ui_title(const char *text)
{
    lv_obj_t *l;

    if (!ui_ready("ui.title") || text == NULL)
        return 0;
    l = lv_label_create(s_parent);
    if (l == NULL)
        return 0;
    lv_label_set_text(l, text);
    lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
    lv_obj_set_style_max_width(l, LV_PCT(100), 0);
    lv_obj_set_style_text_color(l, SKAI_UI_FG, 0);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(l, LV_EXT_FONT_GET(font_step(2)), 0);
    lv_obj_set_style_pad_bottom(l, 6, 0);
    /* A title is the top of the page. In a list it scrolls with the list; on the
       page itself the flow column centres its children vertically, which put a
       Bot-written focus screen's title in the middle of its ring (2026-09-20) —
       so out of the flow and pinned near the top edge of the round screen. */
    if (!in_list())
    {
        lv_obj_set_parent(l, s_root);
        lv_obj_add_flag(l, LV_OBJ_FLAG_IGNORE_LAYOUT);
        lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 36);
    }
    return slot_alloc(l);
}

int32_t skai_ui_section(const char *text)
{
    lv_obj_t *l;

    if (!ui_ready("ui.section") || text == NULL)
        return 0;
    l = lv_label_create(s_parent);
    if (l == NULL)
        return 0;
    lv_label_set_text(l, text);
    lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(l, in_list() ? LV_PCT(100) : 320);
    lv_obj_set_style_text_color(l, SKAI_UI_LABEL2, 0);
    lv_obj_set_style_text_font(l, LV_EXT_FONT_GET(font_step(-1)), 0);
    /* The grouped-list header: inset to the card's text, air above, tight below. */
    lv_obj_set_style_pad_left(l, 20, 0);
    lv_obj_set_style_pad_top(l, 14, 0);
    lv_obj_set_style_pad_bottom(l, 2, 0);
    return slot_alloc(l);
}

/* The second line of a list row, and the capsule at its end. Found by shape,
   not by a stored pointer: the detail is the one that starts a new flex track. */
static lv_obj_t *row_part(lv_obj_t *row, bool detail, bool create)
{
    uint32_t n;
    lv_obj_t *c;

    if (row == NULL || !lv_obj_check_type(row, &lv_btn_class) ||
            lv_obj_get_style_flex_flow(row, LV_PART_MAIN) != LV_FLEX_FLOW_ROW_WRAP)
        return NULL;
    n = lv_obj_get_child_cnt(row);
    for (uint32_t i = 1; i < n; i++)
    {
        c = lv_obj_get_child(row, (int32_t)i);
        if (lv_obj_has_flag(c, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK) == detail)
            return c;
    }
    if (!create)
        return NULL;
    c = lv_label_create(row);
    if (c == NULL)
        return NULL;
    lv_obj_set_style_text_font(c, LV_EXT_FONT_GET(font_step(-1)), 0);
    if (detail)
    {
        lv_label_set_long_mode(c, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(c, LV_PCT(100));
        lv_obj_add_flag(c, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
        lv_obj_set_style_text_color(c, SKAI_UI_LABEL2, 0);
    }
    else
    {
        lv_obj_set_style_radius(c, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_hor(c, 14, 0);
        lv_obj_set_style_pad_ver(c, 6, 0);
        lv_obj_move_to_index(c, 1);   /* before any detail line */
    }
    return c;
}

bool skai_ui_set_detail(int32_t id, const char *text)
{
    lv_obj_t *d;

    if (!ui_ready("ui.set_detail") || text == NULL)
        return false;
    d = row_part(slot_of(id), true, text[0] != '\0');
    if (d == NULL)
        return text[0] == '\0' && slot_of(id) != NULL;
    lv_label_set_text(d, text);
    if (text[0] == '\0') lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
    else                  lv_obj_clear_flag(d, LV_OBJ_FLAG_HIDDEN);
    return true;
}

bool skai_ui_set_accessory(int32_t id, const char *text, int32_t tone)
{
    /* tone: 0 neutral, 1 active (sky — running, selected), 2 done (green),
       3 warning (orange), 4 alert (red). Fill is the ink at 18 %, the phone's
       chip language: colour is never the only signal, the words are. */
    static const uint32_t k_ink[] = { 0x8D8D93, 0xA6D3E6, 0x30D158, 0xFF9F0A, 0xFF453A };
    lv_obj_t *a;
    uint32_t ink;

    if (!ui_ready("ui.set_accessory") || text == NULL)
        return false;
    a = row_part(slot_of(id), false, text[0] != '\0');
    if (a == NULL)
        return text[0] == '\0' && slot_of(id) != NULL;
    ink = k_ink[(tone < 0 || tone > 4) ? 0 : tone];
    lv_label_set_text(a, text);
    lv_obj_set_style_text_color(a, lv_color_hex(ink), 0);
    lv_obj_set_style_bg_color(a, lv_color_hex(ink), 0);
    lv_obj_set_style_bg_opa(a, tone == 0 ? LV_OPA_20 : 46, 0);
    /* A coloured state also gets the chip's rim (ink @ 55 %), as on the phone. */
    lv_obj_set_style_border_color(a, lv_color_hex(ink), 0);
    lv_obj_set_style_border_opa(a, 140, 0);
    lv_obj_set_style_border_width(a, tone == 0 ? 0 : 2, 0);
    if (text[0] == '\0') lv_obj_add_flag(a, LV_OBJ_FLAG_HIDDEN);
    else                  lv_obj_clear_flag(a, LV_OBJ_FLAG_HIDDEN);
    return true;
}

int32_t skai_ui_item(const char *text)
{
    lv_obj_t *b, *l;
    int32_t id;

    if (!ui_ready("ui.item") || text == NULL)
        return 0;
    b = lv_btn_create(s_parent);
    if (b == NULL)
        return 0;
    lv_obj_set_width(b, in_list() ? LV_PCT(100) : 320);
    lv_obj_set_height(b, LV_SIZE_CONTENT);
    card_style(b);
    /* [caption ........ accessory]
       [detail                    ]   — child 0 is always the caption, which
       is what ui.set_text and the click text read; the other two are made on
       first use by ui.set_accessory / ui.set_detail. */
    lv_obj_set_flex_flow(b, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(b, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(b, 12, 0);
    lv_obj_set_style_pad_row(b, 4, 0);
    l = lv_label_create(b);
    if (l != NULL)
    {
        lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
        lv_obj_set_flex_grow(l, 1);
        lv_label_set_text(l, text);
        lv_obj_set_style_text_color(l, SKAI_UI_FG, 0);
    }
    id = slot_alloc(b);
    if (id > 0)
        lv_obj_add_event_cb(b, click_trampoline, LV_EVENT_CLICKED,
                            (void *)(intptr_t)id);
    return id;
}

/* A white tick inside a checked checkbox's circle, drawn with two lines. */
static void tick_draw_cb(lv_event_t *e)
{
    lv_obj_t *c = lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *d = lv_event_get_draw_part_dsc(e);
    lv_draw_line_dsc_t line;
    lv_point_t p[3];
    lv_coord_t x, y, w, h;

    if (d == NULL || d->part != LV_PART_INDICATOR || d->draw_area == NULL ||
            !lv_obj_has_state(c, LV_STATE_CHECKED))
        return;
    x = d->draw_area->x1;
    y = d->draw_area->y1;
    w = lv_area_get_width(d->draw_area);
    h = lv_area_get_height(d->draw_area);
    lv_draw_line_dsc_init(&line);
    line.color = lv_color_hex(0x000000);   /* dark ink on the sky circle */
    line.width = (w >= 24) ? 4 : 3;
    line.round_start = 1;
    line.round_end = 1;
    p[0].x = x + w * 27 / 100; p[0].y = y + h * 52 / 100;
    p[1].x = x + w * 44 / 100; p[1].y = y + h * 68 / 100;
    p[2].x = x + w * 74 / 100; p[2].y = y + h * 34 / 100;
    lv_draw_line(d->draw_ctx, &line, &p[0], &p[1]);
    lv_draw_line(d->draw_ctx, &line, &p[1], &p[2]);
}

int32_t skai_ui_checkbox(const char *text)
{
    lv_obj_t *c;
    int32_t id;

    if (!ui_ready("ui.checkbox") || text == NULL)
        return 0;
    c = lv_checkbox_create(s_parent);
    if (c == NULL)
        return 0;
    lv_checkbox_set_text(c, text); /* copies */
    lv_obj_set_style_text_color(c, SKAI_UI_FG, 0);
    lv_obj_set_style_pad_column(c, 14, 0);
    lv_obj_set_style_radius(c, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_border_color(c, lv_color_hex(0x8E8E93), LV_PART_INDICATOR);
    lv_obj_set_style_border_width(c, 3, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(c, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(c, SKAI_UI_ACCENT, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(c, SKAI_UI_ACCENT, LV_PART_INDICATOR | LV_STATE_CHECKED);
    /* Done items step back, the way a ticked row does on the phone. */
    lv_obj_set_style_text_color(c, SKAI_UI_LABEL2, LV_STATE_CHECKED);
    /* The theme's tick is LV_SYMBOL_OK in the indicator's font, and the system
       font has no symbol glyphs (it drew a box). Linking a symbol font for one
       glyph cost 16 KB of a nearly full image, so the tick is drawn as two
       lines instead (tick_draw_cb). */
    lv_obj_set_style_bg_img_src(c, NULL, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(c, tick_draw_cb, LV_EVENT_DRAW_PART_END, NULL);
    /* In a list a checklist row is as wide as the list: a wide tap target is
       the difference between usable and fiddly on a wrist. */
    if (in_list())
    {
        lv_obj_set_width(c, LV_PCT(100));
        card_style(c);
    }
    id = slot_alloc(c);
    if (id > 0)
        lv_obj_add_event_cb(c, click_trampoline, LV_EVENT_VALUE_CHANGED,
                            (void *)(intptr_t)id);
    return id;
}

int32_t skai_ui_switch(void)
{
    lv_obj_t *w;
    int32_t id;

    if (!ui_ready("ui.switch"))
        return 0;
    w = lv_switch_create(s_parent);
    if (w == NULL)
        return 0;
    lv_obj_set_size(w, 72, 40);
    lv_obj_set_style_bg_color(w, SKAI_UI_TRACK, LV_PART_MAIN);
    lv_obj_set_style_bg_color(w, SKAI_UI_ACCENT, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(w, 140, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(w, SKAI_UI_FG, LV_PART_KNOB);
    id = slot_alloc(w);
    if (id > 0)
        lv_obj_add_event_cb(w, click_trampoline, LV_EVENT_VALUE_CHANGED,
                            (void *)(intptr_t)id);
    return id;
}

int32_t skai_ui_slider(int32_t value)
{
    lv_obj_t *sl;
    int32_t id;

    if (!ui_ready("ui.slider"))
        return 0;
    sl = lv_slider_create(s_parent);
    if (sl == NULL)
        return 0;
    lv_obj_set_size(sl, 240, 14);
    lv_slider_set_range(sl, 0, 100);
    lv_slider_set_value(sl, clamp_pct(value), LV_ANIM_OFF);
    lv_obj_set_style_bg_color(sl, SKAI_UI_TRACK, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sl, SKAI_UI_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sl, SKAI_UI_FG, LV_PART_KNOB);
    lv_obj_set_style_pad_all(sl, 8, LV_PART_KNOB);
    /* Dragging a slider sideways must not also swipe the page. */
    lv_obj_clear_flag(sl, LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_ext_click_area(sl, 16);
    id = slot_alloc(sl);
    if (id > 0)
        lv_obj_add_event_cb(sl, click_trampoline, LV_EVENT_VALUE_CHANGED,
                            (void *)(intptr_t)id);
    return id;
}

bool skai_ui_set_range(int32_t id, int32_t min, int32_t max)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_range") || o == NULL)
        return false;
    if (min >= max || min < -30000 || max > 30000)
        return false;   /* arc ranges are int16 in LVGL 8 */
    if (lv_obj_check_type(o, &lv_slider_class))
        lv_slider_set_range(o, min, max);
    else if (lv_obj_check_type(o, &lv_bar_class))
        lv_bar_set_range(o, min, max);
    else if (lv_obj_check_type(o, &lv_arc_class))
        lv_arc_set_range(o, (int16_t)min, (int16_t)max);
    else
        return false;
    return true;
}

int32_t skai_ui_value(int32_t id)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.value") || o == NULL)
        return SKAI_NO_DATA;
    if (lv_obj_check_type(o, &lv_slider_class))
        return lv_slider_get_value(o);
    if (lv_obj_check_type(o, &lv_bar_class))
        return lv_bar_get_value(o);
    if (lv_obj_check_type(o, &lv_arc_class))
        return lv_arc_get_value(o);
    if (lv_obj_check_type(o, &lv_checkbox_class) || lv_obj_check_type(o, &lv_switch_class))
        return lv_obj_has_state(o, LV_STATE_CHECKED) ? 1 : 0;
    return SKAI_NO_DATA;
}

bool skai_ui_set_value(int32_t id, int32_t value)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_value") || o == NULL)
        return false;
    if (lv_obj_check_type(o, &lv_slider_class))
        lv_slider_set_value(o, value, LV_ANIM_OFF);
    else if (lv_obj_check_type(o, &lv_bar_class))
        lv_bar_set_value(o, value, LV_ANIM_OFF);
    else if (lv_obj_check_type(o, &lv_arc_class))
        lv_arc_set_value(o, (int16_t)value);
    else if (lv_obj_check_type(o, &lv_checkbox_class) || lv_obj_check_type(o, &lv_switch_class))
    {
        if (value)
            lv_obj_add_state(o, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(o, LV_STATE_CHECKED);
    }
    else
        return false;
    return true;
}

bool skai_ui_set_visible(int32_t id, int32_t visible)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.set_visible") || o == NULL)
        return false;
    if (visible)
        lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    return true;
}

bool skai_ui_remove(int32_t id)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.remove") || o == NULL)
        return false;
    /* A group still being built must not be removed from under the insertion
       point: the next create would land in freed memory. */
    for (lv_obj_t *p = s_parent; p != NULL; p = lv_obj_get_parent(p))
        if (p == o)
            return false;
    /* slot_deleted_cb releases this slot and every slot inside it. */
    lv_obj_del(o);
    return true;
}

bool skai_ui_scroll_to(int32_t id)
{
    lv_obj_t *o = slot_of(id);

    if (!ui_ready("ui.scroll_to") || o == NULL)
        return false;
    lv_obj_scroll_to_view_recursive(o, LV_ANIM_ON);
    return true;
}

/* ── keypad ── */

/* The map is rebuilt into the NULL-terminated, "\n"-separated array LVGL wants.
 * Kept in static storage because lv_btnmatrix_set_map does NOT copy it. */
#define SKAI_UI_KEYS_MAX 32
#define SKAI_UI_KEYBUF   192
static const char *s_key_ptr[SKAI_UI_KEYS_MAX + 1];
static char        s_key_buf[SKAI_UI_KEYBUF];

int32_t skai_ui_keypad(const char *map)
{
    lv_obj_t *m;
    int n = 0;
    size_t used = 0;
    const char *p = map;

    if (!ui_ready("ui.keypad") || map == NULL)
        return 0;

    /* Tokenise: spaces separate keys, "\n" ends a row. A blank key is spelled
     * with two spaces, matching LVGL's own convention of a " " entry. */
    while (*p && n < SKAI_UI_KEYS_MAX)
    {
        const char *start = p;
        size_t len;

        if (*p == '\n')
        {
            p++;
            if (used + 2 > sizeof(s_key_buf))
                break;
            s_key_ptr[n++] = &s_key_buf[used];
            s_key_buf[used++] = '\n';
            s_key_buf[used++] = '\0';
            continue;
        }
        if (*p == ' ')
        {
            /* Run of spaces: a single one separates, a double one is a gap key. */
            p++;
            if (*p == ' ')
            {
                p++;
                if (used + 2 > sizeof(s_key_buf))
                    break;
                s_key_ptr[n++] = &s_key_buf[used];
                s_key_buf[used++] = ' ';
                s_key_buf[used++] = '\0';
            }
            continue;
        }
        while (*p && *p != ' ' && *p != '\n')
            p++;
        len = (size_t)(p - start);
        if (used + len + 1 > sizeof(s_key_buf))
            break;
        s_key_ptr[n++] = &s_key_buf[used];
        memcpy(&s_key_buf[used], start, len);
        used += len;
        s_key_buf[used++] = '\0';
    }
    if (n == 0)
        return 0;
    s_key_ptr[n] = "";   /* LVGL's terminator */

    m = lv_btnmatrix_create(s_parent);
    if (m == NULL)
        return 0;
    lv_btnmatrix_set_map(m, s_key_ptr);
    lv_obj_set_style_bg_opa(m, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(m, 0, 0);
    lv_obj_set_style_bg_opa(m, LV_OPA_TRANSP, LV_PART_ITEMS);
    lv_obj_set_style_border_width(m, 0, LV_PART_ITEMS);
    lv_obj_set_style_text_color(m, SKAI_UI_FG, LV_PART_ITEMS);

    {
        int32_t id = slot_alloc(m);
        if (id > 0)
            lv_obj_add_event_cb(m, click_trampoline, LV_EVENT_VALUE_CHANGED,
                                (void *)(intptr_t)id);
        return id;
    }
}

bool skai_ui_keypad_accent(int32_t id, const char *indices, int32_t rgb)
{
    lv_obj_t *o = slot_of(id);
    const char *p = indices;

    if (!ui_ready("ui.keypad_accent") || o == NULL || indices == NULL)
        return false;
    if (!lv_obj_check_type(o, &lv_btnmatrix_class))
        return false;

    /* CHECKED is only being borrowed as a highlight, so the theme's filled
       look has to be cancelled — otherwise an accented key grows a coloured
       button behind it. */
    lv_obj_set_style_text_color(o, lv_color_hex((uint32_t)rgb & 0xFFFFFFu),
                                LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(o, 0, LV_PART_ITEMS | LV_STATE_CHECKED);
    while (*p)
    {
        int v = 0;
        bool digit = false;
        while (*p >= '0' && *p <= '9')
        {
            v = v * 10 + (*p++ - '0');
            digit = true;
            if (v > SKAI_UI_KEYS_MAX)
                return false;
        }
        if (digit)
            lv_btnmatrix_set_btn_ctrl(o, (uint16_t)v, LV_BTNMATRIX_CTRL_CHECKED);
        if (*p)
            p++;   /* skip the separator */
    }
    return true;
}

int32_t skai_ui_divider(int32_t width)
{
    lv_obj_t *d;

    if (!ui_ready("ui.divider"))
        return 0;
    if (width <= 0 || width > 1024)
        return 0;
    /* ponytail: a styled 2px container, not a new widget kind. lv_line needs a
     * caller-owned point array that would have to outlive the call. */
    d = lv_obj_create(s_parent);
    if (d == NULL)
        return 0;
    lv_obj_remove_style_all(d);
    lv_obj_set_size(d, (lv_coord_t)width, 2);
    lv_obj_set_style_bg_color(d, lv_color_hex(0x404040), 0);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
    return slot_alloc(d);
}
