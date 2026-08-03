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

/* Concurrent widgets one app may hold. Small on purpose — a declarative page
 * is the right tool for anything bigger, and an unbounded table is a memory
 * quota with extra steps. */
#define SKAI_UI_SLOTS 16

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
