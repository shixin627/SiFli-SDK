#include "data_service_provider.h"
#include "alarm_manager_service.h"
#include "share_prefs.h"
#include "drivers/alarm.h"
#include "sf_type.h"
#include "time.h"

#define LOG_TAG "svc.alm"
#include "log.h"

#ifndef BSP_ALARM_MAX
#define BSP_ALARM_MAX 8
#endif

typedef struct
{
    rt_alarm_t p_hw_alarm;
    alarm_contxt_t *ctx;
} mgr_alarm_ctx_t;

static datas_handle_t this_service = NULL;
static mgr_alarm_ctx_t *p_mgr_alarms = NULL;
static alarm_contxt_t *p_mgr_alarms_ctx = NULL;
static int32_t g_alarms_num;
static struct rt_mutex s_alm_mgr_mutex;
static void hw_alarm_callback(rt_alarm_t alarm, time_t timestamp);

/* Alarm fire signaling. The hw_alarm_callback runs on the soft-RTC thread,
   which can't safely call into LVGL / motor / GUI APIs. We just record which
   alarm fired and post a sem; a dedicated worker thread invokes the user-
   provided ring hook (bloc_alarm_on_fire, weak — overridden by alarm_client). */
static rt_sem_t s_alarm_fire_sem = RT_NULL;
static volatile int32_t s_alarm_fired_idx = -1;

/* MSVC: RT_WEAK expands to nothing → would collide with the strong def in
 * alarm_client.c. When alarm_client provides the real impl, just declare it. */
#if defined(_MSC_VER) && defined(BSP_USING_ALARM_CLIENT)
extern void bloc_alarm_on_fire(int32_t alarm_idx);
#else
RT_WEAK void bloc_alarm_on_fire(int32_t alarm_idx)
{
    (void)alarm_idx;
}
#endif

static void alarm_fire_thread_entry(void *param)
{
    (void)param;
    while (1)
    {
        if (rt_sem_take(s_alarm_fire_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            int32_t idx = s_alarm_fired_idx;
            s_alarm_fired_idx = -1;
            if (idx >= 0)
            {
                bloc_alarm_on_fire(idx);
            }
        }
    }
}

/* armcc / clang under __STRICT_ANSI__ omits localtime_r from <time.h>; declare
   the toolchain-specific replacement here so the call below resolves. */
extern time_t time(time_t *raw_time);
struct tm *_localtime_r(const time_t *t, struct tm *r);

static rt_alarm_t setup_hw_alarm(alarm_contxt_t *alm_ctx)
{
    struct rt_alarm_setup setup;
    time_t timestamp;
    struct tm now;

    if (NULL == alm_ctx)
        return NULL;
    if (ALARM_STATE_DISABLE == alm_ctx->state)
        return NULL;

    if (ALARM_REPEAT_ONE_SHOT == alm_ctx->days)
    {
        setup.flag = RT_ALARM_ONESHOT;
    }
    else // find out next alarm wday
    {
        /* get time of now */

        timestamp = time(RT_NULL);
// TODO: if __STRICT_ANSI__  is defined,  localtime_r is not available in armcc
#if defined(__CC_ARM) || defined(__CLANG_ARM) || defined(_MSC_VER)
        _localtime_r(&timestamp, &now);
#elif defined(__GNUC__)
        localtime_r(&timestamp, &now);
#else
        localtime_s(&timestamp, &now);
#endif
        /* Find the first wday (starting today) where the alarm should fire.
           The previous loop had its condition inverted: it exited when the
           bit was *clear*, so days==0x7F (every day) walked tm_wday from 0
           to 7, an invalid value that produced an alarm rt_alarm_create()
           treated as "fire immediately". The callback then ran setup again,
           starving the soft-RTC thread → HCPU WDT timeout. */
        int n;
        for (n = 0; n < 7; n++)
        {
            if (alm_ctx->days & (1 << now.tm_wday))
                break;
            now.tm_wday = (now.tm_wday + 1) % 7;
        }
        if (n == 7) /* no day bit set — caller should have rejected this */
            return NULL;

        setup.flag = RT_ALARM_WEEKLY;
        setup.wktime.tm_wday = now.tm_wday;
    }

    setup.wktime.tm_year = RT_ALARM_TM_NOW;
    setup.wktime.tm_mon = RT_ALARM_TM_NOW;
    setup.wktime.tm_mday = RT_ALARM_TM_NOW;
    setup.wktime.tm_hour = alm_ctx->hour;
    setup.wktime.tm_min = alm_ctx->minute;
    setup.wktime.tm_sec = 0;

    return rt_alarm_create(hw_alarm_callback, &setup);
}

static void hw_alarm_callback(rt_alarm_t alarm, time_t timestamp)
{
    /* Stay minimal — this runs on the soft-RTC thread. Heavy work (LVGL,
       motor, BLE) happens in the fire worker thread.

       For RT_ALARM_WEEKLY the kernel auto-rearms; we don't recreate. The
       previous code did delete+recreate, which both leaked the alarm
       (`alarm` argument was never freed before being overwritten) and
       compounded the WDT issue when setup_hw_alarm produced a bad wday. */
    int32_t i;

    for (i = 0; i < g_alarms_num; i++)
    {
        mgr_alarm_ctx_t *p_alm = p_mgr_alarms + i;
        if (p_alm->p_hw_alarm == alarm)
        {
            if (p_alm->ctx->days == ALARM_REPEAT_ONE_SHOT)
            {
                /* Mark disabled so update_alarms() drops it next pass.
                   Don't rt_alarm_delete from the callback — unsafe. */
                p_alm->ctx->state = ALARM_STATE_DISABLE;
            }
            s_alarm_fired_idx = i;
            if (s_alarm_fire_sem) rt_sem_release(s_alarm_fire_sem);
            break;
        }
    }
}

static void update_alarms(void)
{
    int32_t i;

    rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
    for (i = 0; i < g_alarms_num; i++)
    {
        mgr_alarm_ctx_t *p_alm;
        p_alm = p_mgr_alarms + i;

#ifndef _MSC_VER
        struct rt_alarm_setup setup;
        if (p_alm->p_hw_alarm)
        {
            rt_alarm_delete(p_alm->p_hw_alarm);
            p_alm->p_hw_alarm = NULL;
        }

        if (ALARM_STATE_ENABLE == p_alm->ctx->state)
        {
            p_alm->p_hw_alarm = setup_hw_alarm(p_alm->ctx);
            rt_alarm_start(p_alm->p_hw_alarm);
        }
#endif
    }

    rt_mutex_release(&s_alm_mgr_mutex);
}

#ifndef _MSC_VER
/**
 * @brief Read alarm data and set to HW alarm
 */
static void init(void)
{
    share_prefs_t *prefs;
    int32_t i;
    int32_t read_len, content_len;

    rt_mutex_init(&s_alm_mgr_mutex, "alm_mgr", RT_IPC_FLAG_FIFO);

    /* Fire-event worker — keeps hw_alarm_callback off the LVGL/motor path. */
    if (!s_alarm_fire_sem)
    {
        s_alarm_fire_sem = rt_sem_create("alm_fire", 0, RT_IPC_FLAG_FIFO);
        rt_thread_t t = rt_thread_create("alm_fire",
                                         alarm_fire_thread_entry, RT_NULL,
                                         2048, 20, 10);
        if (t) rt_thread_startup(t);
    }

    prefs = share_prefs_open("alarm", SHAREPREFS_MODE_PRIVATE);
    LOG_D("share_prefs_open alm_alarmmgr ok");
    if (prefs != NULL)
    {
        g_alarms_num = share_prefs_get_int(prefs, "num", 0);
        LOG_D("share_prefs_open g_alarms_num = %d", g_alarms_num);
    }
    else
        g_alarms_num = 0;

    if (g_alarms_num > BSP_ALARM_MAX)
        g_alarms_num = BSP_ALARM_MAX;

    p_mgr_alarms_ctx = (alarm_contxt_t *)rt_malloc(sizeof(alarm_contxt_t) * BSP_ALARM_MAX);
    SF_ASSERT(p_mgr_alarms_ctx != NULL);
    memset(p_mgr_alarms_ctx, 0, sizeof(alarm_contxt_t) * BSP_ALARM_MAX);

    if (g_alarms_num > 0)
    {
        // read data to p_mgr_alarms_ctx buffer
        content_len = sizeof(alarm_contxt_t) * g_alarms_num;
        read_len = share_prefs_get_block(prefs, "content", p_mgr_alarms_ctx, content_len);
        if (read_len != content_len)
            SF_ASSERT(0);
    }

    // convert to p_mgr_alarms
    p_mgr_alarms = (mgr_alarm_ctx_t *)rt_malloc(sizeof(mgr_alarm_ctx_t) * BSP_ALARM_MAX);
    SF_ASSERT(p_mgr_alarms != NULL);
    memset(p_mgr_alarms, 0, sizeof(mgr_alarm_ctx_t) * BSP_ALARM_MAX);

    for (i = 0; i < g_alarms_num; i++)
    {
        p_mgr_alarms[i].p_hw_alarm = NULL;
        p_mgr_alarms[i].ctx = &p_mgr_alarms_ctx[i];
    }
    update_alarms();

    if (prefs != NULL)
        share_prefs_close(prefs);
}
#else
/**
 * @brief init alarm from simulator
 */
static void init(void)
{
    rt_mutex_init(&s_alm_mgr_mutex, "alm_mgr", RT_IPC_FLAG_FIFO);

    g_alarms_num = 2;

    if (g_alarms_num > 0)
    {
        int32_t i, content_len;

        // read data to p_mgr_alarms_ctx buffer
        content_len = sizeof(alarm_contxt_t) * BSP_ALARM_MAX;
        p_mgr_alarms_ctx = (alarm_contxt_t *)rt_malloc(content_len);
        SF_ASSERT(p_mgr_alarms_ctx != NULL);

        // convert to p_mgr_alarms
        p_mgr_alarms = (mgr_alarm_ctx_t *)rt_malloc(sizeof(mgr_alarm_ctx_t) * BSP_ALARM_MAX);
        SF_ASSERT(p_mgr_alarms != NULL);

        for (i = 0; i < g_alarms_num; i++)
        {
            p_mgr_alarms_ctx[i].state = ALARM_STATE_ENABLE;
            p_mgr_alarms_ctx[i].hour = 10 + i;
            p_mgr_alarms_ctx[i].minute = 32 + i;
            p_mgr_alarms_ctx[i].days = ALARM_REPEAT_EVERYDAY;
            p_mgr_alarms_ctx[i].snooze = ALARM_SNOOZE_ENABLE;

            p_mgr_alarms[i].p_hw_alarm = NULL;
            p_mgr_alarms[i].ctx = &p_mgr_alarms_ctx[i];
        }

        update_alarms();
    }
}

#endif

static void get_alarm_data(data_msg_t *msg, int32_t idx)
{
    rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
    if (idx < 0) // No alarm exist, or reach alarm list end
        datas_send_response_data(this_service, msg, 0, (uint8_t *)NULL);
    else
    {
        alarm_msg_t rsp_msg;

        rsp_msg.idx = idx;
        memcpy(&rsp_msg.ctx, &p_mgr_alarms_ctx[idx], sizeof(alarm_contxt_t));
        datas_send_response_data(this_service, msg, sizeof(alarm_msg_t), (uint8_t *)&rsp_msg);
    }

    rt_mutex_release(&s_alm_mgr_mutex);
}

static int32_t msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        static uint8_t is_init = 0;
        if (!is_init)
        {
            init();
            is_init = 1;
        }
        break;
    }

    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        break;
    }

    case MSG_SERVICE_CONFIG_REQ:
    {
        break;
    }

    case ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ:
    {
        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);
        int32_t idx;

        if ((g_alarms_num > 0) && (p_mgr_alarms_ctx != NULL))
        {
            if (0 == msg->len) // get first one
            {
                idx = 0;
            }
            else if (p_alm_msg->idx < g_alarms_num - 1)
            {
                idx = p_alm_msg->idx + 1; // get next one
            }
            else
            {
                idx = -1; // reach end of the list
            }
        }
        else
        {
            idx = -1;
        }

        get_alarm_data(msg, idx);
    }
    break;
    case ALARMMGR_MSG_GET_ALARM_REQ:
    {
        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);
        get_alarm_data(msg, p_alm_msg->idx);
        break;
    }
    case ALARMMGR_MSG_ADD_ALARM_REQ:
    {
        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);
        rt_err_t result = RT_ERROR;

        if (p_alm_msg && g_alarms_num < BSP_ALARM_MAX)
        {
            rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
            p_mgr_alarms[g_alarms_num].p_hw_alarm = NULL;
            p_mgr_alarms[g_alarms_num].ctx = &p_mgr_alarms_ctx[g_alarms_num];
            memcpy(&p_mgr_alarms_ctx[g_alarms_num], &p_alm_msg->ctx, sizeof(alarm_contxt_t));

            g_alarms_num++;
            update_alarms();
            rt_mutex_release(&s_alm_mgr_mutex);
            result = RT_EOK;
        }
        else
        {
            result = -RT_EFULL;
        }

        datas_send_response(service, msg, result);
    }
    break;

    case ALARMMGR_MSG_EDIT_ALARM_REQ:
    {
        rt_err_t result = RT_ERROR;

        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);

        if (p_alm_msg)
        {
            if ((p_alm_msg->idx >= 0) && (p_alm_msg->idx < g_alarms_num))
            {
                rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
                memcpy(&p_mgr_alarms_ctx[p_alm_msg->idx], &p_alm_msg->ctx, sizeof(alarm_contxt_t));
                update_alarms();
                rt_mutex_release(&s_alm_mgr_mutex);

                result = RT_EOK;
            }
            else
            {
                result = RT_ERROR;
            }
        }
        else
        {
            result = RT_EINVAL;
        }

        datas_send_response(service, msg, result);
    }
    break;

    case ALARMMGR_MSG_ENABLE_ALARM_REQ:
    case ALARMMGR_MSG_DISABLE_ALARM_REQ:
    {
        rt_err_t result = RT_ERROR;

        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);

        rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
        if (p_alm_msg)
        {
            if ((p_alm_msg->idx >= 0) && (p_alm_msg->idx < g_alarms_num))
            {
                if (ALARMMGR_MSG_ENABLE_ALARM_REQ == msg->msg_id)
                    p_mgr_alarms[p_alm_msg->idx].ctx->state = ALARM_STATE_ENABLE;
                else
                    p_mgr_alarms[p_alm_msg->idx].ctx->state = ALARM_STATE_DISABLE;

                update_alarms();

                result = RT_EOK;
            }
            else
            {
                result = RT_ERROR;
            }
        }
        else
        {
            result = RT_EINVAL;
        }
        rt_mutex_release(&s_alm_mgr_mutex);

        datas_send_response(service, msg, result);
    }
    break;

    case ALARMMGR_MSG_DELETE_ALARM_REQ:
    {
        rt_err_t result = RT_ERROR;

        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);

        rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
        if (p_alm_msg)
        {
            if ((p_alm_msg->idx >= 0) && (p_alm_msg->idx < g_alarms_num))
            {
                int32_t i;
#ifndef _MSC_VER
                rt_alarm_delete(p_mgr_alarms[p_alm_msg->idx].p_hw_alarm);
#endif

                for (i = p_alm_msg->idx; i < g_alarms_num - 1; i++)
                {
                    memcpy(&p_mgr_alarms_ctx[i], &p_mgr_alarms_ctx[i + 1], sizeof(alarm_contxt_t));
                    p_mgr_alarms[i].p_hw_alarm = p_mgr_alarms[i + 1].p_hw_alarm;
                }
                g_alarms_num--;
                result = RT_EOK;
            }
            else
            {
                result = RT_ERROR;
            }
        }
        else
        {
            result = RT_EINVAL;
        }
        rt_mutex_release(&s_alm_mgr_mutex);

        datas_send_response(service, msg, result);
    }
    break;

    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}

static data_service_config_t service_cb =
    {
        .max_client_num = 3,
        .queue = RT_NULL,
        .data_filter = NULL,
        .msg_handler = msg_handler,
};

/* Static on target; externally visible on the PC simulator so the harness
   can call it directly during init. */
#ifndef _MSC_VER
static
#endif
int alarm_manager_service_register(void)
{
    this_service = datas_register("alarmmgr", &service_cb);
    return RT_EOK;
}

INIT_PRE_APP_EXPORT(alarm_manager_service_register);
