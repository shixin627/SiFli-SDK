/* ---------------------------------------------------------------------------
 * weather.js - Skai SDK reproduction of the built-in C weather app.
 *
 * Every constant below is read off
 *   src/hcpu/gui_apps/weather/app_weather.c :: lv_card_layout_weather_create()
 * which builds tile 1, "current weather" - the screen in the reference shot.
 *
 * Scope: tile 1 only. The C app is a 2-tile lv_tileview (swipe down for the
 * 5-day summary) driven by arc_scroll; the curated API has no tileview, no
 * scroll container and no gesture, so the second page cannot be expressed.
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
 * the phone has not sent one. There is no location capability, so the fallback
 * is the only string available - and it is what the reference shot renders.
 * No set_color: ui.label is already 0xFFFFFF, same as C's lv_color_white(). */
var loc = skai.ui.label('No Location');
skai.ui.set_font(loc, -1);
skai.ui.align(loc, 'top', 0, 30);              /* LV_ALIGN_TOP_MID, 0, 30 */

/* --- current temperature --------------------------------------------------
 * C: snprintf("%0.f<deg>C", round(temperature)) on the current slot.
 * weather.temp() is already whole degrees C. It is null before the phone has
 * sent a forecast; the C app renders its zeroed record as 0, so 0 is the
 * faithful substitute rather than a "--" of our own invention.
 *
 * dy 107 is baked. C chains lv_obj_align_to(..., LV_ALIGN_OUT_BOTTOM_MID):
 * location -> icon (+10) -> temperature (+0). There is no align-to-sibling
 * capability, so the resulting y is precomputed for the 466x466 panel:
 *   30 (location top) + 19 (line height) + 10 (gap) + 48 (icon) = 107.
 * The 48 px hole above the number is where the weather icon would be. */
var t = skai.available('weather.temp') ? skai.weather.temp() : null;
var cur = skai.ui.label((t === null ? 0 : t) + '°C');
skai.ui.set_font(cur, 0);
skai.ui.align(cur, 'top', 0, 107);

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
 * Neither the forecast hours nor their temperatures are reachable: the weather
 * namespace exposes one slot (now) and no timestamps. The strings below are
 * exactly what the C app prints for an empty slot - ui_time_format_hhmm(0, 0)
 * is "12:00 AM" in 12-hour mode and round(0.0) is 0 - i.e. the reference shot.
 *
 * y: C hangs the row off the divider (OUT_BOTTOM_MID, +10). The divider is at
 * screen centre +10 and 2 px tall, so the time label's top is centre + 21; the
 * temperature is one line height (19) + a 10 px gap + a 48 px icon below that.
 * Centre comes from the capability instead of a baked 233 so the row tracks the
 * divider on any panel; the 19/48 remain baked for want of align-to-sibling. */
var cy = skai.watchinfo.screen_height() / 2;
var dx = [-130, 0, 130];
var i, hr, tv;

for (i = 0; i < 3; i++) {
    hr = skai.ui.label('12:00 AM');
    skai.ui.set_font(hr, -2);
    skai.ui.set_color(hr, 0xA0A0A0);
    skai.ui.align(hr, 'top', dx[i], cy + 21);

    tv = skai.ui.label('0°C');
    skai.ui.set_font(tv, -2);
    skai.ui.set_color(tv, 0xA0A0A0);
    skai.ui.align(tv, 'top', dx[i], cy + 98);
}

/* 9 of the 16 widget slots used. The 7 that remain would not be enough for the
 * 4 missing weather icons plus the 5-day tile anyway. */
