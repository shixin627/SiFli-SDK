/* ---------------------------------------------------------------------------
 * weather.js - Skai SDK reproduction of the built-in C weather app.
 *
 * Every constant below is read off
 *   src/hcpu/gui_apps/weather/app_weather.c :: lv_card_layout_weather_create()
 * which builds tile 1, "current weather" - the screen in the reference shot.
 *
 * Scope: both tiles. The C app is a 2-tile lv_tileview (swipe down for the
 * 5-day summary); ui.page() is the same structure with the host owning the
 * swipe. Not reproduced: arc_scroll, C's right-edge arc-drag tile switch.
 *
 * Font sizes use the same relative scale the C app does:
 *   FONT_SUBTITLE            -> rel -1  (location; equals get_system_font_size(-1)
 *                                        at the default system size, LVSF_FONT_TITLE)
 *   get_system_font_size(0)  -> rel  0  (current temperature)
 *   get_system_font_size(-2) -> rel -2  (forecast row)
 *
 * The degree sign is a literal UTF-8 U+00B0, the same two bytes app_weather.c
 * puts in its own format strings; QuickJS parses the payload as UTF-8.
 * ------------------------------------------------------------------------ */

/* --- location -------------------------------------------------------------
 * C: get_current_location(), falling back to the literal "No Location" when
 * the phone has not sent one - the same fallback weather.location() reports as
 * null. No set_color: ui.label is already 0xFFFFFF, same as lv_color_white(). */
var loc = skai.ui.label(skai.weather.location() || 'No Location');
skai.ui.set_font(loc, -1);
skai.ui.align(loc, 'top', 0, 30);              /* LV_ALIGN_TOP_MID, 0, 30 */

/* --- current temperature --------------------------------------------------
 * C: snprintf("%0.f<deg>C", round(temperature)) on the current slot.
 * weather.temp() is already whole degrees C. It is null before the phone has
 * sent a forecast; the C app renders its zeroed record as 0, so 0 is the
 * faithful substitute rather than a "--" of our own invention.
 *
 * The stack below is the same chain C builds with lv_obj_align_to(...,
 * OUT_BOTTOM_MID): location -> icon (+10) -> temperature -> description. It
 * used to be three baked y constants computed for one font size and one icon
 * height; ui.align_to means the column now reflows when either changes, which
 * is what C does. */
/* C: weather_icon_get(description). condition() returns the token half of that
 * decision, so the name is mostly a concatenation — with two exceptions.
 *
 * "snow" maps to a DIFFERENT glyph than its name: C draws weather_thunder for
 * Snow (app_weather.c:216-219), and there is no weather_snow asset anyway. The
 * SDK reports the honest token and refuses to bake C's choice into the
 * capability layer, which is right — this app's job is to look like C, so the
 * mapping belongs here.
 *
 * Anything else ui.icon has no glyph for returns 0, and `cond || 'sun'` does
 * not catch that (a token like "snow" is truthy). C defaults to weather_sun
 * (:225), so do the same: a dead slot id would make the next align_to fail and
 * drop that row's rain label back into the flow column. */
function iconName(cond) {
    return cond === 'snow' ? 'weather.thunder' : 'weather.' + (cond || 'sun');
}
function iconFor(cond) {
    return skai.ui.icon(iconName(cond)) || skai.ui.icon('weather.sun');
}
function setIcon(id, cond) {
    if (!skai.ui.set_icon(id, iconName(cond)))
        skai.ui.set_icon(id, 'weather.sun');
}
/* Empty slots: C renders its zeroed weather_t as 0, not as a blank. */
function degText(v) { return (v === null ? 0 : v) + '°'; }
var icon = iconFor(skai.weather.condition());
skai.ui.align_to(icon, loc, 'below', 0, 10);   /* C: OUT_BOTTOM_MID, +10 */

function tempText() {
    var t = skai.available('weather.temp') ? skai.weather.temp() : null;
    return (t === null ? 0 : t) + '°C';
}
var cur = skai.ui.label(tempText());
skai.ui.set_font(cur, 0);
skai.ui.align_to(cur, icon, 'below', 0, 0);

/* C: weather_label, the phone's own wording under the temperature
 * (:792-800), 0x999999 at rel -1. Empty when no forecast has arrived, which is
 * also what C renders for a zeroed record. */
var desc = skai.ui.label(skai.weather.description() || '');
skai.ui.set_font(desc, -1);
skai.ui.set_color(desc, 0x999999);
skai.ui.align_to(desc, cur, 'below', 0, 0);

/* C: no_dat_label (:803-819) — same wording, same 0x999999 at rel -1, same
 * CENTER 0,-60. Created unconditionally and blanked when there is data, which
 * is C's own structure (it creates the label always and toggles HIDDEN).
 *
 * It has to be built even when the answer is "not stale", because staleness is
 * NOT a one-shot test: weather_layout_update() hides this label on every
 * refresh (:963) precisely so a watch that opens the screen before the phone
 * has synced stops claiming there is no weather once the data lands. Gating
 * creation on stale() at eval time got that backwards — cold start creates the
 * banner and nothing can ever clear it.
 *
 * ponytail: C centres the two lines within the label box; there is no
 * text-align capability and adding one is not worth it for this, so line 1
 * sits left-aligned under line 2. */
var nodata = skai.ui.label(skai.weather.stale()
                           ? 'No weather data \n please update on the phone' : '');
skai.ui.set_font(nodata, -1);
skai.ui.set_color(nodata, 0x999999);
skai.ui.align(nodata, 'center', 0, -60);

/* C binds lv_ex_data("weather.data") -> refresh_ui (:1047-1053) so the screen
 * repaints when the phone pushes. on_change is the same idea by capability
 * name: the app declared weather.temp, so it may also watch it.
 *
 * Now that ui.set_icon exists this can repaint the picture too, not just the
 * words - previously the condition icon was frozen at whatever it was when the
 * script first ran. */
skai.on_change('weather.temp', function () {
    skai.ui.set_text(cur, tempText());
    skai.ui.set_text(desc, skai.weather.description() || '');
    skai.ui.set_text(loc, skai.weather.location() || 'No Location');
    setIcon(icon, skai.weather.condition());

    /* C hides no_dat_label on every refresh (:963): this handler only runs
       because fresh data arrived, so by definition it is no longer stale. */
    skai.ui.set_text(nodata, '');

    /* C re-aligns weather_label to cur_tem_label right after updating them
       (:966-967), and it is not decoration: on a cold start `desc` is created
       from an empty description, so it is aligned as a ZERO-WIDTH box. Filling
       it later grows it rightward from that point instead of re-centring, and
       the description ends up hanging off to the right. The forecast row does
       not need this — those labels are created with real values, never "". */
    skai.ui.align_to(desc, cur, 'below', 0, 0);

    var k;
    for (k = 0; k < 3 && k < skai.weather.hour_count(); k++) {
        skai.ui.set_text(hrIds[k], skai.weather.hour_time(k) || '12:00 AM');
        skai.ui.set_text(tvIds[k], skai.weather.hour_temp(k) + '°C');
        setIcon(icIds[k], skai.weather.hour_cond(k));
    }

    /* C's refresh_ui calls weather_layout_update() AND
       update_daily_weather_layout() (:1005-1009). Repainting only the first
       page left the daily rows frozen at whatever the phone had sent when the
       script first ran — every push after the first showed stale numbers on
       half the app. */
    var j;
    for (j = 0; j < dtIds.length; j++) {
        skai.ui.set_text(dtIds[j], skai.weather.day_date(j) || '00/00');
        setIcon(diIds[j], skai.weather.day_cond(j));
        skai.ui.set_text(drIds[j], rainText(j));
        skai.ui.set_text(dmaxIds[j], degText(skai.weather.day_max(j)));
        skai.ui.set_text(dminIds[j], degText(skai.weather.day_min(j)));
    }
});

/* --- divider --------------------------------------------------------------
 * C: lv_line, 400 px wide, line_width 2, 0x4B4B4B, LV_ALIGN_CENTER 0,+10
 * (the earlier align_to on that object is immediately overridden).
 * ui.divider is a 2 px bar of the right shape but hardcodes 0x404040, so it is
 * repainted to the C colour. */
var line = skai.ui.divider(400);
skai.ui.set_bg(line, 0x4B4B4B);
skai.ui.align(line, 'center', 0, 10);

/* --- 3-hour forecast row --------------------------------------------------
 * C: create_forecast_widget() three times at x = -130 / 0 / +130 from centre,
 * each column being time label / icon / temperature, colour 0xA0A0A0, rel -2.
 *
 * Each column now shows what its OWN slot says. hour_time(i) arrives already
 * formatted in the user's 12/24-hour setting, so this cannot print "12:00 AM"
 * on a watch set to 24-hour - which the previous hardcoded version did.
 *
 * y: hung off the divider exactly as C does (OUT_BOTTOM_MID, +10), then each
 * column stacks below the one above it. No baked line heights left. */
var dx = [-130, 0, 130];
var slots = skai.weather.hour_count();
var i, hr, tv, ic, t;
var hrIds = [], icIds = [], tvIds = [];

for (i = 0; i < 3; i++) {
    /* Slot i is the i-th forecast after now, nearest first - the same column
       order the C app draws. An empty slot renders what C renders for a
       zeroed record. */
    hr = skai.ui.label((i < slots ? skai.weather.hour_time(i) : null)
                       || '12:00 AM');
    skai.ui.set_font(hr, -2);
    skai.ui.set_color(hr, 0xA0A0A0);
    skai.ui.align_to(hr, line, 'below', dx[i], 10);   /* C: off the divider */
    hrIds.push(hr);

    /* C: create_forecast_widget() puts the condition icon between the hour and
       the temperature, OUT_BOTTOM_MID +10 off the hour label. */
    ic = iconFor(i < slots ? skai.weather.hour_cond(i) : null);
    skai.ui.align_to(ic, hr, 'below', 0, 10);
    icIds.push(ic);

    t = (i < slots) ? skai.weather.hour_temp(i) : null;
    tv = skai.ui.label((t === null ? 0 : t) + '°C');
    skai.ui.set_font(tv, -2);
    skai.ui.set_color(tv, 0xA0A0A0);
    skai.ui.align_to(tv, ic, 'below', 0, 0);
    tvIds.push(tv);
}

/* --- page 2: the 5-day summary ---------------------------------------------
 * C: lv_daily_weather_page_create() - tile 2 of the tileview, five rows of
 * {date, icon, rain%, max, min} (app_weather.c:405-489), reached by swiping
 * down. ui.page() gives the same structure; the host owns the swipe, the snap
 * and the scrollbar, so nothing here touches a scroll offset.
 *
 * Every coordinate below is create_daily_forecast_widget() (:405-487) read off
 * line by line. It used to be five hand-tuned `ui.align('top', dx, y)` calls
 * per row on a 76 px pitch, which was six independent errors compounding — C's
 * pitch is 80, its date column is LEFT-aligned rather than centred, and its
 * temperatures are pinned to the RIGHT EDGE, not centred on a guessed x. None
 * of that needed a new capability; it needed the anchors C itself uses. */
skai.ui.page();

/* C creates all five rain labels up front and hides the dry ones (:442-448).
   Same here with an empty string, so a day that turns wet between pushes has a
   widget to fill — creating them lazily meant on_change could not. */
function rainText(i) {
    var c = skai.weather.day_cond(i), p = skai.weather.day_rain(i);
    return ((c === 'rain' || c === 'thunder') && p > 0) ? p + '%' : '';
}

/* Five rows ALWAYS, exactly like C, which loops i<5 over a fixed-size array and
 * renders its zeroed records rather than skipping them (:506). Gating this on
 * day_count() meant a watch that opened the screen before the phone had synced
 * built no rows at all — and since on_change repaints rows rather than creating
 * them, that page then stayed blank for the rest of the run. The hourly row
 * above already loops unconditionally; this is the same rule. */
var d, y, dt, di, dmax, dmin, dr, sep;
var dtIds = [], diIds = [], drIds = [], dmaxIds = [], dminIds = [];

for (d = 0; d < 5; d++) {
    y = 60 + d * 80;                                 /* C: 60 + index*80 */

    /* "00/00", not a "--/--" of our own invention: on_start's
       request_weather_within_six_hours(:1054) fires refresh_ui with the zeroed
       record, so C's "%02d/%02d" prints 00/00 before any phone data. Using the
       same string also keeps the icon column identical cold and warm, since it
       is aligned to this label's width. */
    dt = skai.ui.label(skai.weather.day_date(d) || '00/00');
    skai.ui.set_font(dt, -2);
    skai.ui.set_color(dt, 0xCCCCCC);
    skai.ui.align(dt, 'top_left', 80, y);            /* C: TOP_LEFT, 80, y */
    dtIds.push(dt);

    /* No set_size: on an image that resizes the WIDGET BOX and crops, it does
       not scale the picture. Native 48 px fits the 80 px row pitch.
       Not reproduced: C aligns this icon while the date label still holds its
       CREATION text — "Today" for row 0, a weekday name for the rest
       (:425) — and update_daily_weather_layout() then overwrites that with
       "MM/DD" without re-aligning (:540). So C's icons sit at a per-row x
       decided by text that is no longer on screen — a uniform 16 px left of
       ours on rows 1-4, exact on row 0 ("Today" happens to match "MM/DD" in
       width). The rain label, anchored to the icon, inherits the shift.
       The SAME quirk hits dmin below: C aligns it off dmax while dmax still
       holds week_list[i] and the update pass overwrites it with week_list[4-i]
       (:530), so its offset is mirror-symmetric about row 2.
       Matching either means reproducing the bug AND exporting a weekday
       capability purely to be measured and discarded. Ours is aligned to the
       text it actually shows. */
    di = iconFor(skai.weather.day_cond(d));
    skai.ui.align_to(di, dt, 'right', 50, 0);        /* C: OUT_RIGHT_MID, 50 */
    diIds.push(di);

    dr = skai.ui.label(rainText(d));                 /* C: 0x4080FF (:443) */
    skai.ui.set_font(dr, -2);
    skai.ui.set_color(dr, 0x4080FF);
    skai.ui.align_to(dr, di, 'right', 10, 0);        /* C: OUT_RIGHT_MID, 10 */
    drIds.push(dr);

    /* C: max at TOP_RIGHT -80, min OUT_LEFT_MID -30 of it (:469-473) — min on
       the LEFT, max on the right, and one text_color 0xCCCCCC for date, min and
       max alike. The right edges line up because max is pinned to the screen
       edge and min hangs off max; centring both on fixed x gave a ragged
       column that no offset could fix. */
    dmax = skai.ui.label(degText(skai.weather.day_max(d)));
    skai.ui.set_font(dmax, -2);
    skai.ui.set_color(dmax, 0xCCCCCC);
    skai.ui.align(dmax, 'top_right', -80, y);        /* C: TOP_RIGHT, -80, y */
    dmaxIds.push(dmax);

    dmin = skai.ui.label(degText(skai.weather.day_min(d)));
    skai.ui.set_font(dmin, -2);
    skai.ui.set_color(dmin, 0xCCCCCC);
    skai.ui.align_to(dmin, dmax, 'left', -30, 0);    /* C: OUT_LEFT_MID, -30 */
    dminIds.push(dmin);

    /* C draws a 300 px 0x404040 rule between rows, not after the last one,
       starting at the date's left edge (:476-486, OUT_BOTTOM_LEFT +10). */
    if (d < 4) {                    /* C: index < WEATHER_DAILY_ITEM_AMOUNT-1 */
        sep = skai.ui.divider(300);
        skai.ui.set_bg(sep, 0x404040);
        /* ui.divider is 2 px, which is right for the page-1 rule (C line_width
           2) and wrong here (C line_width 1, :480). set_size already exists and
           says exactly that, so no new capability is needed to say it. */
        skai.ui.set_size(sep, 300, 1);
        skai.ui.align_to(sep, dt, 'below_left', 0, 10);
    }
}

/* C ends on_start by asking the phone for fresh data (:1054). Without this the
 * screen only ever shows whatever happened to be cached when it opened. */
skai.weather.refresh();
