/*
 * Skai SDK — drawing for external apps (ADR-0019 Phase 3, decision 9).
 *
 * This exists because the vendored QuickJS LVGL binding must NOT be used for
 * third-party code: it hands ~200 raw lv_obj_* entry points to untrusted
 * script, which bypasses the capability model and feeds attacker-controlled
 * arguments straight into LVGL. This is the curated alternative — a small set
 * of widgets, projected from the dispatch table like every other capability.
 *
 * Widgets are addressed by SLOT ID, never by pointer. A bad id is refused, not
 * dereferenced, so a hostile app cannot turn a number into a wild write. Same
 * reasoning as skai_timer's slot table.
 *
 * Everything here is SKAI_THREAD_LVGL. Calls from another thread are refused
 * rather than deferred: a create has to return an id immediately, so there is
 * nothing sensible to defer to.
 */
#ifndef SKAI_UI_H
#define SKAI_UI_H

#include <stdbool.h>
#include <stdint.h>
#include "skai/skai_export.h"

/* Concurrent widgets one app may hold. Bounded on purpose — an unbounded table
 * is a memory quota with extra steps — but 16 was too small to be honest: the
 * weather reproduction spends 13 on ONE page and its second page needs ~25
 * more, so pages would arrive dead. Three pointer tables, so the cost of the
 * raise is 48 x 3 x 4 = 576 bytes against a JS heap measured in tens of KB.
 * (40 -> 48 because the daily page's rain figures and row separators put the
 * weather reproduction at 42 in the worst case.) */
#define SKAI_UI_SLOTS 48

/* Full-screen pages an app may declare. The host owns the swipe, the snap and
 * the scrollbar; an app names a page index and never touches a scroll offset,
 * a gesture or an LVGL object — which is what keeps this on the right side of
 * ADR-0019 decision 9. */
#define SKAI_UI_PAGES 4

/* Create a text line. Returns a widget id (>0), or 0 on failure. */
SKAI_EXPORT("ui.label", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_label(const char *text);

/* Create a button carrying `text`. Wire it up with skai.ui.on_click(id, fn). */
SKAI_EXPORT("ui.button", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_button(const char *text);

/* App-supplied image, by path RELATIVE to the app's own directory.
 *
 * ponytail: no packaging format and no decoder of our own — LVGL already loads
 * images from the filesystem and the project already ships the decoders. The
 * only thing added here is confinement: the path is resolved inside the app's
 * directory, and anything with "..", a leading "/" or a drive letter is
 * refused. An app cannot read another app's assets or the system's.
 *
 * Decode cost lands on the JS memory quota, so an app that loads something
 * enormous starves itself rather than the watch. */
SKAI_EXPORT("ui.image", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_image(const char *rel_path);

/* A firmware-owned icon, addressed by name from a closed set — the system's
 * glyphs, where ui.image is the app's own assets.
 *
 * The app supplies a string and never pixels, a path or a decoder, so this
 * grants strictly less than ui.image does. A name that is not in the set draws
 * nothing and returns 0, which is also what makes it safe to ship an app that
 * asks for an icon a older firmware does not have yet. */
SKAI_EXPORT("ui.icon", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_icon(const char *name);

/* Repoint an existing icon at another name from the same closed set. The
 * counterpart to ui.set_text, and just as load-bearing: without it a screen
 * that refreshes can update its words but never its pictures, so an app that
 * redraws on a data push has to delete and rebuild everything. */
SKAI_EXPORT("ui.set_icon", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_set_icon(int32_t id, const char *name);

/* Horizontal group. Widgets created after it land inside until ui.end(). */
SKAI_EXPORT("ui.row", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_row(void);

/* Close the innermost open row. */
SKAI_EXPORT("ui.end", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_end(void);

/* Linear progress bar, 0..100. */
SKAI_EXPORT("ui.bar", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_bar(int32_t percent);

/* Text colour of a label/button, as 0xRRGGBB. */
SKAI_EXPORT("ui.set_color", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_set_color(int32_t id, int32_t rgb);

/* Background colour, as 0xRRGGBB. Also makes the background opaque. */
SKAI_EXPORT("ui.set_bg", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_set_bg(int32_t id, int32_t rgb);

/* Relative text size, -2..3, on the same scale the rest of the watch uses, so
 * a JS app tracks the user's font-size setting instead of pinning pixels. */
SKAI_EXPORT("ui.set_font", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_set_font(int32_t id, int32_t rel_size);

SKAI_EXPORT("ui.set_size", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_set_size(int32_t id, int32_t w, int32_t h);

/* Place a widget explicitly, escaping the default column flow.
 *
 * `anchor` is one of: center, top, bottom, left, right, top_left, top_right,
 * bottom_left, bottom_right. `dx`/`dy` offset from it in pixels.
 *
 * One primitive instead of a layout API: every built-in app positions with
 * lv_obj_align, so this covers what they do without exposing a style system to
 * untrusted code. */
SKAI_EXPORT("ui.align", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_align(int32_t id, const char *anchor, int32_t dx, int32_t dy);

/* Place a widget relative to ANOTHER widget, which is how every built-in screen
 * is actually laid out.
 *
 * `side` is one of: below, above, left, right, center — each meaning "outside
 * that edge of `ref`, centred on the other axis" — plus below_left, which is
 * "under it, left edges flush" and is what a rule drawn beneath a short label
 * needs. Six of lv_obj_align_to's twelve OUT_* anchors, chosen because they are
 * the ones the built-in screens actually use. `dx`/`dy` offset from there.
 *
 * Why this earns its place next to ui.align: without it a reproduction has to
 * precompute every y from font metrics and icon heights, so the layout is
 * correct for exactly one font size and one icon set and silently drifts the
 * moment either changes. It still takes two opaque slot ids and one name from
 * the closed set above — no LVGL object, no style, no coordinate system. */
SKAI_EXPORT("ui.align_to", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_align_to(int32_t id, int32_t ref_id, const char *side,
                      int32_t dx, int32_t dy);

/* A grid of keys as ONE widget, laid out from a map: keys separated by spaces,
 * rows by "\n" — e.g. "7 8 9 +\n4 5 6 -\n1 2 3 x\n  0 . /".
 *
 * One widget rather than one per key, because that is what the underlying
 * button matrix is: bounded by construction, no per-key slot to exhaust, and a
 * hostile app cannot fabricate a key id. The click handler receives the key's
 * text. */
SKAI_EXPORT("ui.keypad", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_keypad(const char *map);

/* Recolour some keys of a keypad — comma-separated indices in map order, e.g.
 * "3,7,11,15" for a column of operators, in 0xRRGGBB. The colour is a
 * parameter because a reproduction of an existing screen has to match its
 * palette, not the SDK's default one. */
SKAI_EXPORT("ui.keypad_accent", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_keypad_accent(int32_t id, const char *indices, int32_t rgb);

/* A horizontal rule. */
SKAI_EXPORT("ui.divider", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_divider(int32_t width);

/* Create a progress arc, 0..100. Returns a widget id (>0), or 0 on failure. */
SKAI_EXPORT("ui.arc", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_arc(int32_t percent);

SKAI_EXPORT("ui.set_text", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_set_text(int32_t id, const char *text);

SKAI_EXPORT("ui.set_arc", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_set_arc(int32_t id, int32_t percent);

/* Delete every widget this app created. */
SKAI_EXPORT("ui.clear", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_clear(void);

/* ── pages ──
 * Start a new full-screen page and make it the target for everything created
 * afterwards. Returns its index (>0; page 0 always exists and is where an app
 * starts), or 0 when the budget is spent.
 *
 * Pages stack vertically and the user swipes between them, which is the shape
 * the built-in screens use. The app declares structure and nothing else: no
 * scroll offset, no gesture, no momentum, no snap. */
SKAI_EXPORT("ui.page", SKAI_T1, SKAI_THREAD_LVGL)
int32_t skai_ui_page(void);

/* Show a page. Animated, like a swipe. */
SKAI_EXPORT("ui.goto_page", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_ui_goto_page(int32_t index);

/* ── host side, not exported ──
 * The app host owns the container and hands it over for the lifetime of a run.
 * Detaching invalidates every id, so a stale id from a previous run cannot
 * address a widget in this one.
 *
 * `asset_dir` is the app's own directory (e.g. "/skaiapp/my-app"); ui.image
 * cannot escape it. NULL means the app has no assets and every ui.image fails
 * closed. */
void skai_ui_attach(void *lv_parent, const char *asset_dir);
void skai_ui_detach(void);

/* Click delivery. Registration lives in the JS runtime rather than the
 * dispatch table because a callback has no projection in the capability type
 * vocabulary — see ADR-0019 decision 13. */
/* `text` is the label that was pressed — the key for a keypad, the caption for
 * a button. Without it a keypad would need one handler per key, which defeats
 * the point of it being a single widget. */
typedef void (*skai_ui_click_cb_t)(int32_t id, const char *text, void *arg);
bool skai_ui_on_click(int32_t id, skai_ui_click_cb_t cb, void *arg);

/* True when `rel_path` stays inside the app's asset directory. Exposed for
 * the self-check — path confinement is the whole security story for images,
 * so it gets tested directly rather than only through LVGL. */
bool skai_ui_path_ok(const char *rel_path);

#endif /* SKAI_UI_H */
