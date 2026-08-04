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

/* Palette matches skaiapp_render.c so a JS app and a declarative package sit
 * on the same screen without looking like two products. */
#define SKAI_UI_FG      lv_color_hex(0xFFFFFF)
#define SKAI_UI_ACCENT  lv_color_hex(0x0A84FF)
#define SKAI_UI_TRACK   lv_color_hex(0x2C2C2E)

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


static int32_t slot_alloc(lv_obj_t *obj)
{
    for (int i = 0; i < SKAI_UI_SLOTS; i++)
    {
        if (s_slots[i] == NULL)
        {
            s_slots[i] = obj;
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
    if (!lv_obj_check_type(o, &lv_label_class))
        return false; /* wrong widget kind — refuse, do not reinterpret */
    lv_label_set_text(o, text);
    return true;
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
    lv_obj_set_style_bg_color(b, SKAI_UI_ACCENT, 0);
    lv_obj_set_style_radius(b, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_pad_hor(b, 20, 0);
    lv_obj_set_style_pad_ver(b, 10, 0);

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
    if (slot_of(id) == NULL)
        return false;
    s_click_cb[id - 1] = cb;
    s_click_arg[id - 1] = arg;
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
    lv_obj_set_style_text_font(o, LV_EXT_FONT_GET(get_system_font_size((int8_t)rel)), 0);
    if (lv_obj_check_type(o, &lv_btnmatrix_class))
        lv_obj_set_style_text_font(o, LV_EXT_FONT_GET(get_system_font_size((int8_t)rel)),
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
        return true;
    }
    LOG_W("ui.align_to: unknown side '%s'", side);
    return false;
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
