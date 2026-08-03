/* sleep.js -- Skai SDK JS reproduction of the built-in C app
 * src/hcpu/gui_apps/sleep/app_sleep.c. Every colour, string, font step and
 * pixel offset below is read off that file; the line refs in the comments
 * point at it. Nothing here is eyeballed and nothing is invented.
 *
 * The one thing the C app is actually about -- SkaiWatchSys.sleep_state
 * (total/deep/rem/light/WASO minutes, stage mode, resting HR) -- has NO
 * capability in capability-registry.json. Those numbers therefore hold the
 * values app_sleep.c itself draws before its first poll (lines 163/177/183/
 * 189/195), i.e. what the screen shows on a watch with no sleep data.
 *
 * ASCII source on purpose: the only non-ASCII glyph the C app draws is the
 * em dash, written below as an escape.
 */

/* --- palette: app_sleep.c:71-82 plus the per-label styles ---------------- */
var C_TITLE   = 0xCCCCCC;   /* title text              (app_sleep.c:154) */
var C_WHITE   = 0xFFFFFF;   /* big total, banner text  (162, 207)        */
var C_DIM     = 0x999999;   /* caption, WASO, HR row   (169, 194, 214)   */
var C_DEEP    = 0x1B3FA0;   /* WIRE_TDEEP_SLEEP        (76)              */
var C_REM     = 0xB94DFF;   /* WIRE_TREM_SLEEP         (77)              */
var C_LIGHT   = 0x4F8DFF;   /* WIRE_TSLEEP             (75)              */
var C_NOTWEAR = 0x666666;   /* WIRE_TNOT_WEAR          (79)              */
var C_UNKNOWN = 0x333333;   /* prv_stage_color default (80)              */

/* prv_stage_name's default branch is a literal em dash (app_sleep.c:67).
 * Built from its code point so this file stays byte-for-byte ASCII. */
var EMDASH = String.fromCharCode(0x2014);

/* --- the numbers, all unreachable ---------------------------------------
 * sleep_state is produced on LCPU by the sleep_fusion classifier and reaches
 * HCPU over watch_sys_sync RPC. No capability projects any field of it, so a
 * JS app cannot see one of these. Held at the C app's own initial text rather
 * than faked with plausible-looking minutes. */
var totalMin  = 0;
var deepMin   = 0;
var remMin    = 0;
var lightMin  = 0;
var wasoMin   = 0;
var restingHr = null;       /* sleep_state.resting_hr -- not exposed */

function pad2(n) { return (n < 10 ? '0' : '') + n; }

/* "%uh %02um", app_sleep.c:98 */
function hoursMins(min) {
    return Math.floor(min / 60) + 'h ' + pad2(min % 60) + 'm';
}

/* --- current stage -------------------------------------------------------
 * Of the five WIRE_* modes (app_sleep.c:35-39) only TNOT_WEAR is decidable
 * from the registry: health.worn() reads SkaiWatchSys.flag_field.is_wearing,
 * the same physical fact the classifier turns into TNOT_WEAR. Light / Deep /
 * REM / Awake all need sleep_state.mode and are unreachable, so they fall
 * through to prv_stage_name's default. */
function notWorn() {
    return skai.available('health.worn') && skai.health.worn() === false;
}

function stageName()  { return notWorn() ? 'Not worn' : EMDASH; }
function stageColor() { return notWorn() ? C_NOTWEAR : C_UNKNOWN; }

/* --- HR row, app_sleep.c:118-126 ----------------------------------------
 * health.heart_rate() is SkaiWatchSys.heart_rate_bpm, the live reading, and
 * returns null exactly where the C side stores 0 -- the same "no measurement"
 * test the C app makes with `s.current_hr > 0`. It is not literally
 * sleep_state.current_hr (the classifier's own sample) but it is the same
 * sensor. resting_hr has no capability at all, so its slot stays "--". */
function hrText() {
    var hr = skai.available('health.heart_rate') ? skai.health.heart_rate() : null;
    return 'HR ' + (hr === null ? '--' : hr) +
           '  (rest ' + (restingHr === null ? '--' : restingHr) + ')';
}

/* Every widget in the C app is lv_obj_align'd to the screen with dx 0, which
 * is what ui.align does here (it reparents out of the SDK's flex column). */
function place(id, anchor, dy, rgb) {
    skai.ui.set_color(id, rgb);
    skai.ui.align(id, anchor, 0, dy);
    return id;
}

/* The black backdrop (app_sleep.c:145-146) needs no call: the JS app host
 * already paints its root container black (app_skaijs.c:65-66). */

/* title -- system font size 1, top-mid +60 (app_sleep.c:150-156) */
var title = skai.ui.label('Sleep');
skai.ui.set_font(title, 1);
place(title, 'top', 60, C_TITLE);

/* big total -- system font size 3, white, top-mid +95 (app_sleep.c:159-164) */
var total = skai.ui.label(hoursMins(totalMin));
skai.ui.set_font(total, 3);
place(total, 'top', 95, C_WHITE);

/* subtitle -- top-mid +160 (app_sleep.c:167-171) */
place(skai.ui.label('Total sleep today'), 'top', 160, C_DIM);

/* stage breakdown -- +195 / +225 / +255 / +285 (app_sleep.c:174-196). The
 * columns line up because the C strings pad themselves, so the runs of spaces
 * below are literal, not accidental. */
place(skai.ui.label('Deep   ' + deepMin + ' min'), 'top', 195, C_DEEP);
place(skai.ui.label('REM    ' + remMin + ' min'), 'top', 225, C_REM);
place(skai.ui.label('Light  ' + lightMin + ' min'), 'top', 255, C_LIGHT);
place(skai.ui.label('WASO   ' + wasoMin + ' min'), 'top', 285, C_DIM);

/* current-stage banner -- "Inverted: dark pill, coloured background",
 * bottom-mid -90 (app_sleep.c:198-209).
 *
 * ui.button is the only pill the curated API has: skai_ui.c:363-366 already
 * gives it LV_RADIUS_CIRCLE + 20/10 padding, against the C banner's radius 20
 * + 16/8. There is no radius and no padding capability, so a ui.label here
 * would be a bare rectangle tight around its text. The price of the button is
 * that ui.set_text refuses anything that is not a label (skai_ui.c:175), so
 * this caption is fixed at creation. */
var banner = skai.ui.button('Now: ' + stageName());
skai.ui.set_bg(banner, stageColor());
skai.ui.set_color(banner, C_WHITE);
skai.ui.align(banner, 'bottom', 0, -90);

/* HR row under the banner -- bottom-mid -55 (app_sleep.c:212-216) */
var hrLabel = skai.ui.label(hrText());
place(hrLabel, 'bottom', -55, C_DIM);

/* --- refresh -------------------------------------------------------------
 * The C app repaints on a 1 Hz lv_timer (app_sleep.c:242). No timer is
 * exported to JS at all -- skai_timer.h:4 says so deliberately -- so the only
 * way to re-read anything is an event. Tapping the banner re-reads the heart
 * rate. The banner's own caption cannot be rewritten (see above), so the
 * stage is left alone rather than letting its text and colour disagree.
 *
 * ui.on_click is not a capability and is deliberately absent from sleep.caps:
 * it grants nothing, it only routes an event to a closure this app already
 * owns (skai_js.c:493-497).
 *
 * The C app's tap-anywhere-to-exit (app_sleep.c:136-140, gui_app_exit) has no
 * equivalent -- nothing in the registry closes an app. */
skai.ui.on_click(banner, function () {
    skai.ui.set_text(hrLabel, hrText());
});
