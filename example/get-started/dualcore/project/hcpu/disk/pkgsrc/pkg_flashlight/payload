/* ---------------------------------------------------------------------------
 * flashlight.js - Skai SDK reproduction of the built-in C flashlight app.
 * Constants read off src/hcpu/gui_apps/flashlight/app_flashlight.c.
 *
 * app_flashlight.c pushes LCD brightness to 96 on ONRESUME (:271), restores
 * SkaiWatchSys.brightness on ONPAUSE (:276), holds power-save off (:218) and
 * exits on tap (:110). The first, second and fourth are now capabilities; the
 * restore is not one, because the firmware does it when this app leaves the
 * foreground and never asks the app to - and re-applies it on resume, which is
 * what C's every-ONRESUME set_amoled_brightness does.
 *
 * The previous revision drew the torch as 8 ui.divider rectangles measured off
 * btn_flashlight.png. ui.icon replaced all of it with one call, which also
 * fixed the stair-stepped silhouette, the square barrel foot and the wrong
 * cut-out shape: it is now the same asset the C app draws, not a copy of it.
 * ------------------------------------------------------------------------ */

/* --- white field ----------------------------------------------------------
 * C: lv_obj_create(scr) sized LV_HOR_RES x LV_VER_RES, bg lv_color_white()
 * (:67-68). No capability paints a screen background, so a full-bleed
 * ui.divider stands in for it. Created first because later-created widgets
 * draw on top. */
var W = skai.watchinfo.screen_width() || 466;
var H = skai.watchinfo.screen_height() || 466;

var bg = skai.ui.divider(W);
skai.ui.set_size(bg, W, H);
skai.ui.set_bg(bg, 0xFFFFFF);
skai.ui.align(bg, 'center', 0, 0);

/* --- tap target -----------------------------------------------------------
 * C: a 200x200 fully transparent lv_obj over the icon taking LV_EVENT_ALL ->
 * gui_app_exit() (:96-118). ui.button is the only clickable widget, so it is
 * painted white-on-white; that only vanishes because the field behind it is
 * flat white. The tap calls app.exit(), the same close C does.
 *
 * Created BEFORE the torch, because later-created widgets draw on top and an
 * opaque 200x200 target would hide the icon. C gets the same stacking by
 * nesting the image inside the button (app_flashlight.c:89). */
var hit = skai.ui.button(' ');
skai.ui.set_size(hit, 200, 200);
skai.ui.set_bg(hit, 0xFFFFFF);
skai.ui.set_color(hit, 0xFFFFFF);
skai.ui.align(hit, 'center', 0, 0);
skai.ui.on_click(hit, function () {
    skai.app.exit();                        /* C: gui_app_exit() (:110) */
});

/* --- the torch ------------------------------------------------------------
 * C: lv_img with LV_EXT_IMG_GET(BTN_FLASHLIGHT), 100x100, centred (:88-92).
 * On top of the tap target, so the glyph itself swallows clicks — the same
 * trade the rectangle version had, and invisible to the user either way since
 * the ring around it is still 100 px of live target. */
var torch = skai.ui.icon('flashlight');
skai.ui.align(torch, 'center', 0, 0);

/* --- the actual job ------------------------------------------------------
 * C: PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ with 96 on ONRESUME (:271). Everything
 * above is decoration around this one line — a white rectangle at whatever
 * brightness the user left the watch on is not a flashlight.
 *
 * There is no matching call to put it back: the firmware restores the user's
 * brightness when this app pauses or stops, so a crash here cannot leave the
 * watch at maximum. */
skai.display.set_brightness(96);

/* C: set_power_save_mode(0) on resume (:218), restored on pause (:226). Without
 * it the idle timer dims and blanks the panel mid-use - a torch that turns
 * itself off. Released by the firmware when this app leaves the foreground, so
 * there is no matching call to forget. */
skai.display.set_power_save(false);

/* 3 widgets, down from 11. */
