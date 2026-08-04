/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl.h"
#include "lvsf.h"
#ifdef MENU_FRAMEWORK
    #include "app_nvm.h"
    #include "menu_fwk.h"
#endif
#include "lvsf_switchanim.h"

#if LVSF_USING_SWITCHANIM != 0

uint16_t def_anim_major = LV_SWITCHANIM_PUSH;
uint16_t def_anim_minor = LV_PUSHANIM_AUX;

SECTION_DEF(SWITCH_ANIM_SECTION_NAME, lv_baseanim_cb);

void lv_switchanim_load(void)
{
    lv_baseanim_cb *anim_cb;
    uint32_t *end;
    uint32_t *temp;

    end = (uint32_t *)SECTION_END_ADDR(SWITCH_ANIM_SECTION_NAME);
    temp = (uint32_t *)SECTION_START_ADDR(SWITCH_ANIM_SECTION_NAME);
    while (temp < end)
    {
        anim_cb = (lv_baseanim_cb *)temp;
        if ((anim_cb->name[0] != 0) && anim_cb->progress_cb && anim_cb->major)
        {
            lv_switchanim_register_anim(anim_cb->name, anim_cb->major, anim_cb->progress_cb);
            temp += (sizeof(lv_baseanim_cb) >> 2);
        }
        else
        {
            temp++;
        }
    }

#ifdef MENU_FRAMEWORK
    app_menu_anim_t *cfg = &nvm_sys_get(switch_anim);
    if (0 < cfg->second_lvl && cfg->second_lvl < LV_SWITCHANIM_TURN_SWITCH)
        def_anim_major = cfg->second_lvl;
#endif
}

void lv_switchanim_set_def_anim(uint16_t anim_major, uint16_t anim_minor)
{
    def_anim_major = anim_major;
    def_anim_minor = anim_minor;
}

void lv_switchanim_overwrite(const lv_baseanim_para_t *enter_org, \
                             lv_baseanim_para_t *enter_anim, \
                             const lv_baseanim_para_t *exit_org, \
                             lv_baseanim_para_t *exit_anim, \
                             uint32_t flag)
{
    if (enter_org && enter_anim)
        memcpy(enter_anim, enter_org, sizeof(lv_baseanim_para_t));
    if (exit_anim && exit_org)
        memcpy(exit_anim, exit_org, sizeof(lv_baseanim_para_t));
    if (!(enter_org && enter_anim && exit_org && exit_anim))
        return;

    if (enter_anim->prior > exit_anim->prior)
    {
        exit_anim->major = enter_anim->major;
        exit_anim->minor = enter_anim->minor_aux;
    }
    if (enter_anim->prior < exit_anim->prior)
    {
        enter_anim->major = exit_anim->major;
        enter_anim->minor = exit_anim->minor_aux;
    }

    if ((exit_anim->major == LV_SWITCHANIM_DEFAULT) && (enter_anim->major == LV_SWITCHANIM_DEFAULT))
    {
        exit_anim->major = def_anim_major;
        exit_anim->minor = def_anim_minor;
        enter_anim->major = def_anim_major;
        enter_anim->minor = def_anim_minor;
    }

    if (exit_anim->major == LV_SWITCHANIM_DEFAULT)
        exit_anim->major = enter_anim->major;

    if (enter_anim->major == LV_SWITCHANIM_DEFAULT)
        enter_anim->major = exit_anim->major;
    enter_anim->flag = flag;
    exit_anim->flag = flag;

    if (exit_anim->major == LV_SWITCHANIM_TURN_BOOK || \
            exit_anim->major == LV_SWITCHANIM_SHUTTLE || \
            exit_anim->major == LV_SWITCHANIM_SHUTTER)
        exit_anim->flag |= LV_SWITCHANIM_FLAG_16_ALIGN;

    if (enter_anim->major == LV_SWITCHANIM_TURN_BOOK || \
            enter_anim->major == LV_SWITCHANIM_SHUTTLE || \
            enter_anim->major == LV_SWITCHANIM_SHUTTER)
        enter_anim->flag |= LV_SWITCHANIM_FLAG_16_ALIGN;
}

//-----------------------------------------------default anim----------------------------------------------
void lv_baseanim_progress_default(lv_baseanim_t *baseanim, lv_obj_t *anim_obj, int32_t progress)
{
    if (LV_BASEANIM_EXIT_TYPE == lv_baseanim_get_type(baseanim))
    {
        lv_coord_t pos = ((LV_HOR_RES_MAX * progress) >> 10);
        lv_coord_t opa = LV_OPA_COVER - ((progress * LV_OPA_COVER) >> 10);
        //rt_kprintf("pos:%d\n", pos);
        lv_obj_align_to(anim_obj, NULL, LV_ALIGN_CENTER, pos, 0);
        lv_obj_set_style_img_opa(anim_obj, opa, 0);
    }
    lv_obj_invalidate(lv_scr_act());
}

BUILTIN_ANIMATION(pushanim, LV_SWITCHANIM_PUSH, lv_baseanim_progress_default);

//-----------------------------------------------zoom anim----------------------------------------------
static void lv_zoomanim_progress(lv_baseanim_t *baseanim, lv_obj_t *anim_obj, int32_t progress)
{
    lv_baseanim_para_t *para = lv_baseanim_get_para(baseanim);
    uint16_t minor = para->minor;
    lv_coord_t zoom_start = APP_TRANS_ANIM_ZOOM_NONE;
    lv_coord_t zoom_end = 0;
    lv_coord_t opa_start = LV_OPA_COVER;
    lv_coord_t opa_end = LV_OPA_30;

    if (LV_ZOOMANIM_LARGER == para->minor)
    {
        zoom_start = APP_TRANS_ANIM_ZOOM_NONE;
        zoom_end = APP_TRANS_ANIM_ZOOM_NONE << 2;
        opa_start = LV_OPA_COVER;
        opa_end = LV_OPA_30;
    }
    if (LV_BASEANIM_ENTER_TYPE == lv_baseanim_get_type(baseanim))
    {
        LV_COORD_SWAP(zoom_start, zoom_end);
        LV_COORD_SWAP(opa_start, opa_end);
    }

    lv_coord_t zoom = lv_map(progress, 0, LV_SWITCHANIM_PROGRESS_MAX, zoom_start, zoom_end);
    lv_img_set_zoom(anim_obj, zoom);
    lv_coord_t opa = lv_map(progress, 0, LV_SWITCHANIM_PROGRESS_MAX, opa_start, opa_end);
    lv_obj_set_style_img_opa(anim_obj, opa, 0);
    lv_obj_invalidate(lv_scr_act());
}

BUILTIN_ANIMATION(zoomanim, LV_SWITCHANIM_ZOOM, lv_zoomanim_progress);

#endif
