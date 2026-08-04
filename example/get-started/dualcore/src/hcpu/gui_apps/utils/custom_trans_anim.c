/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "custom_trans_anim.h"

/*
 * Transition animations now come from the framework's lvsf_switchanim
 * catalogue instead of the four hand-written callbacks that used to live here.
 *
 * The SDK removed the callback model these were built on -- gui_anim_obj_t +
 * FLAG_TRANS_ANIM_REVERSE/_FG, installed through gui_app_trans_anim_init_cfg()
 * and a cfg.cust.exe_cb function pointer. The replacement selects an animation
 * by (major, minor) type id and drives it through lv_baseanim_progress_cb,
 * which takes different arguments and has no equivalent of the _FG flag.
 *
 * cust_trans_anim_config() is kept as the seam so the eight callers do not
 * change; only what it installs is different.
 *
 * ponytail: a mapping table, not a re-implementation. Each CUST_ANIM_TYPE_
 * points at the nearest built-in, matched against what the old callback
 * actually drew (noted per case). Porting the originals to
 * lv_baseanim_progress_cb is a separate job worth doing on hardware, where the
 * difference is visible side by side.
 *
 * Only pick from animations this build actually links. lvsf_switchanim.h
 * declares twelve majors, but lvsf_switchanim_com.c registers exactly two --
 * BUILTIN_ANIMATION(pushanim, LV_SWITCHANIM_PUSH, ...) and
 * BUILTIN_ANIMATION(zoomanim, LV_SWITCHANIM_ZOOM, ...) -- and main.map
 * confirms the switch_anim section holds only those two entries. Naming any
 * other major here compiles clean and then misses at runtime.
 */

#ifdef APP_TRANS_ANIMATION_SCALE

void cust_trans_anim_config(CUST_ANIM_TYPE_E type, lv_point_t *pivot)
{
    /* The built-ins take their pivot from the animated object. The old
       enter_or_exit_anim_default() was the only callback that read this
       argument, and every caller passes NULL. */
    (void)pivot;

    switch (type)
    {
    case CUST_ANIM_TYPE_0:
        /* Swung the image through 60 degrees while fading. Nothing rotational
           is available, so it lands on the framework's own default. */
        gui_app_set_enter_anim_type(LV_SWITCHANIM_PUSH, LV_PUSHANIM_AUX, 0);
        gui_app_set_exit_anim_type(LV_SWITCHANIM_PUSH, LV_PUSHANIM_AUX, 0);
        break;

    case CUST_ANIM_TYPE_1:
    case CUST_ANIM_TYPE_2:
    case CUST_ANIM_TYPE_3:
    default:
        /* TYPE_1 was an opacity cross-fade, TYPE_2 collapsed the width to zero
           and back, TYPE_3 zoomed between LV_IMG_ZOOM_NONE >> 2 and << 2. Only
           ZOOM is close to any of them, and LV_ZOOMANIM_MAINMENU aliases
           LV_ZOOMANIM_LARGER, so this is the launch-from-mainmenu pairing the
           framework itself intends. */
        gui_app_set_enter_anim_type(LV_SWITCHANIM_ZOOM, LV_ZOOMANIM_LARGER, 0);
        gui_app_set_exit_anim_type(LV_SWITCHANIM_ZOOM, LV_ZOOMANIM_SMALL, 0);
        break;
    }
}

#else

void cust_trans_anim_config(CUST_ANIM_TYPE_E type, lv_point_t *pivot)
{
}

#endif /* APP_TRANS_ANIMATION_SCALE */
