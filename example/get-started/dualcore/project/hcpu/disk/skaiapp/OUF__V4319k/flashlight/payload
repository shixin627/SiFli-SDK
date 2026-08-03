/* flashlight.js - the built-in C app `flashlight` rebuilt on the curated Skai
 * JS API. Every number below is read off
 * src/hcpu/gui_apps/flashlight/app_flashlight.c or off the icon asset
 * src/hcpu/resource/images/common/ezip/btn_flashlight.png (100x100), never
 * eyeballed. Line refs are to app_flashlight.c.
 *
 * What the C app does that this cannot: max out LCD brightness (powermgr
 * data_service, :171-182 / :271-278), hold power-save off (:219), and exit on
 * tap (:103). None of those have a capability, so this reproduces the screen
 * only. See flashlight.caps for what it is allowed to call.
 */

/* LV_HOR_RES / LV_VER_RES (:67). Fallback is CONFIG_LV_HOR_RES_MAX=466 from
 * project/hcpu/pc_hcpu/proj.conf, for the case watchinfo is not granted. */
var W = skai.watchinfo.screen_width() || 466;
var H = skai.watchinfo.screen_height() || 466;

var BLACK = 0x000000;          /* LV_COLOR_BLACK (:85), and the icon's ink */
var WHITE = 0xFFFFFF;          /* lv_color_white() (:68, :74) */

/* A solid rectangle. ui.divider is the only primitive that draws an unstyled
 * filled block (no radius, no border, no theme), so it stands in for every
 * shape here. ui.align is also the z-order: it reparents to the root, and each
 * newly aligned widget lands on top of the previous one - so these are called
 * back to front. */
function rect(w, h, dy, rgb)
{
    var id = skai.ui.divider(w);
    skai.ui.set_size(id, w, h);
    skai.ui.set_bg(id, rgb);
    skai.ui.align(id, 'center', 0, dy);
    return id;
}

/* -- background: full-screen white (:66-68) -- */
rect(W, H, 0, WHITE);

/* -- the 200x200 tap target, centred (:78-86) --
 * In C it is a transparent lv_obj over the white background; here it is a
 * ui.button painted the same white as the backdrop, which is the only
 * clickable widget the API offers. Its handler is the app's whole behaviour:
 * LV_EVENT_CLICKED -> gui_app_exit(APP_ID_FLASHLIGHT) (:110-118, :99-105). */
var tap = skai.ui.button('');
skai.ui.set_size(tap, 200, 200);
skai.ui.set_bg(tap, WHITE);
skai.ui.align(tap, 'center', 0, 0);
skai.ui.on_click(tap, function ()
{
    /* No capability closes an app, so the tap can only say so. */
    skai.log.write(1, 'flashlight: tap = gui_app_exit in C; no exit capability');
});

/* -- the torch glyph (:89-91) --
 * C draws LV_EXT_IMG_GET(BTN_FLASHLIGHT), a 100x100 image centred in the tap
 * target. A JS app can only ui.image() a file inside its own package, and this
 * app ships no asset, so the silhouette is rebuilt from the asset's own pixel
 * rows. Offsets are (row centre - 50), because the image centre is (50,50) and
 * it is drawn 1:1 at screen centre.
 *
 *   asset rows        w    what
 *   y6..12           34    top rim
 *   y13..14           -    2px gap
 *   y15..19          34    reflector head
 *   y20..25       32->30   head shoulder
 *   y26..29       28->24   taper
 *   y30..31       22->21   taper
 *   y32..93          20    barrel (rounded off below y89)
 *   y41..58 x45..54  10    oval cut-out (reads as background)
 *   y49..57 x48..51   4    button stem inside the cut-out
 */
rect(34,  7, -41, BLACK);      /* rim,     y6..12  centre 9    */
rect(34,  5, -33, BLACK);      /* head,    y15..19 centre 17   */
rect(31,  6, -28, BLACK);      /* shoulder,y20..25 centre 22.5 */
rect(25,  4, -23, BLACK);      /* taper,   y26..29 centre 27.5 */
rect(21,  3, -19, BLACK);      /* taper,   y30..32 centre 31   */
rect(20, 62,  13, BLACK);      /* barrel,  y32..93 centre 62.5 */
rect(10, 18,   0, WHITE);      /* cut-out, y41..58 centre 49.5 */
rect(4,   9,   3, BLACK);      /* stem,    y49..57 centre 53   */
