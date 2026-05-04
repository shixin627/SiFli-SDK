/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "littlevgl2rtt.h"
#include "ui_handler.h"
#include "lvgl.h"
#include "lvsf.h"
#include "app_mainmenu.h"
#define DBG_TAG "app.speech.provider"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
typedef struct
{
    lv_ex_data_t *title;
    lv_ex_data_t *content;
    void *title_handle;
    void *content_handle;
} app_speech_ctx_t;

static app_speech_ctx_t *p_app_speech_ctx = NULL;

void unbind_app_speech_data(void)
{
    if (p_app_speech_ctx->content_handle)
    {
        lv_ex_unbind_data(p_app_speech_ctx->content, p_app_speech_ctx->content_handle);
    }
    if (p_app_speech_ctx->title_handle)
    {
        lv_ex_unbind_data(p_app_speech_ctx->title, p_app_speech_ctx->title_handle);
    }
}

void app_speech_data_init(void)
{
    if (!p_app_speech_ctx)
    {
        p_app_speech_ctx = (app_speech_ctx_t *)rt_malloc(sizeof(app_speech_ctx_t));
        memset(p_app_speech_ctx, 0, sizeof(app_speech_ctx_t));
    }

    p_app_speech_ctx->title = lv_ex_data_create("speech.title", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_speech_ctx->title);
    p_app_speech_ctx->content = lv_ex_data_create("speech.content", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_speech_ctx->content);
}

void app_speech_bind_title(lv_obj_t *target)
{
    lv_ex_binding_t binding;
    RT_ASSERT(p_app_speech_ctx->title);
    binding.target = target;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    p_app_speech_ctx->title_handle = lv_ex_bind_data(p_app_speech_ctx->title, &binding);
}

lv_ex_data_t *app_speech_get_content_data(void)
{
    return p_app_speech_ctx ? p_app_speech_ctx->content : NULL;
}

void app_speech_bind_content(lv_obj_t *target)
{
    lv_ex_binding_t binding;
    RT_ASSERT(p_app_speech_ctx->content);
    binding.target = target;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    p_app_speech_ctx->content_handle = lv_ex_bind_data(p_app_speech_ctx->content, &binding);
}

void app_speech_set_title(const uint8_t *s)
{
    if (p_app_speech_ctx->title)
    {
        lv_ex_data_set_value(p_app_speech_ctx->title, (void *)s);
    }
}

void app_speech_set_content(const uint8_t *s)
{
    if (p_app_speech_ctx->content)
    {
        LOG_D("app_speech_set_content p_app_speech_ctx: %s", s);
        lv_ex_data_set_value(p_app_speech_ctx->content, (void *)s);
    }
}