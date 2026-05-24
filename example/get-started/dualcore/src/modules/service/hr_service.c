/**
 ******************************************************************************
 * @file   hr_service.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>
#include "hr_service.h"
#include "sensor.h"
#include "bloc_peripheral.h"
#include "watch_sys_service.h"

#define DBG_TAG "DS.HR"
#define DBG_LVL DBG_INFO
#include "rtdbg.h"

#ifdef HR_USING_GH3011
#include "sensor_goodix_gh3011.h"
#define HR_MODEL_NAME "gh3011"

#elif defined(HR_USING_GH3018)
#include "sensor_goodix_gh3018.h"
#define HR_MODEL_NAME "gh3018"

#elif defined(HR_USING_AFE4404)
#include "sensor_ti_afe4404.h"
#define HR_MODEL_NAME "afe4404"
#endif

#define HR_DEV_NAME "hr_" HR_MODEL_NAME

#define HR_TIMER_PERIOD_MS (1000)

#define PPG_TIMER_PERIOD_MS_25HZ (40 * 2)
#define PPG_TIMER_PERIOD_MS_100HZ (10 * 2)
#define PPG_TIMER_PERIOD_MS PPG_TIMER_PERIOD_MS_25HZ

// timestamp to minute, current ts unit is 1 second
#define HR_TS2MIN (60 * 1)
// how long to save hr, in minute
#define HR_HOUR_GAP_MIN (60)
// data count for 1 day
#define HR_HOUR_CNT (24 * 60 / HR_HOUR_GAP_MIN)

enum
{
    HR_GET_DAY_HOURS = (1 << 0),
    HR_GET_MON_RHR = (1 << 1),
    HR_GET_REGION = (1 << 2),
    HR_GET_MAX = (1 << 3),
    HR_GET_MIN = (1 << 4),
    HR_GET_RHR = (1 << 5)
};

typedef struct
{
    uint8_t timestamp;
    uint8_t hr_value;
} hr_service_value_t;

enum
{
    HR_ULTIMATE_LIMIT = 0,
    HR_ANAEROBIC,
    HR_AEROBIC,
    HR_HIIT,
    HR_WARM_UP,
    HR_SPORT_CNT
};
// define heart rate threshold for each sport mode, base on rhr
#define HR_ULTIMATE_THD(x) ((x) * 3)
#define HR_ANAEROBIC_THD(x) ((x) * 5 / 2)
#define HR_AEROBIC_THD(x) ((x) * 2)
#define HR_HIIT_THD(x) ((x) * 3 / 2)
#define HR_WARMUP_THD(x) ((x) * 5 / 4)

// #define USING_PPG_FOR_GESTURE_RECOGNITION
#define PPG_FIFO_LENGTH 4
#define PPG_SENSOR_BUF_SIZE (PPG_FIFO_LENGTH * sizeof(struct rt_sensor_data))
// continue counter for same hr to rhr
#define HR_RHR_THRESHOLD (60)
typedef struct
{
    uint8_t rhr_cnt;
    uint8_t rhr_value;
} hr_service_rhr_t;

typedef struct
{
    hr_service_value_t hour[HR_HOUR_CNT]; // today 24h hr, 1 value per 30 min(HR_HOUR_GAP_SEC) : half hour + value
    hr_service_value_t rhr_arr[30];       // priv 30 days resting heart rate : day + value
    uint8_t status_arr[HR_SPORT_CNT];     // minute for each mode
    uint8_t max_hr;                       // today max heart rate
    uint8_t min_hr;                       // today min heart rate
    uint8_t rhr;                          // current resting heart rate
    time_t today;                         // save today time stamp, to detect date changed
} hr_service_data_t;

typedef struct
{
    int ref_count;

    rt_device_t device;
    datas_handle_t service;
    rt_bool_t hr_subscribed;

    struct rt_sensor_data data;
    rt_timer_t timer;

    hr_service_data_t env;    // data need saved, app and ui need them.
    uint8_t cur_mode;         // current sport mode
    uint8_t cur_mcnt;         // current sport mode duration, mode minute increase after it over a threshold
    hr_service_rhr_t get_rhr; // sturcture for checking resting heart rate(RHR)

#ifdef USING_PPG_FOR_GESTURE_RECOGNITION
    datas_handle_t ppg_service;
    rt_timer_t ppg_timer;
    rt_bool_t ppg_subscribed;
    struct rt_sensor_data ppg_sensor_data[PPG_FIFO_LENGTH];
    rt_uint32_t ppg_fifo[PPG_FIFO_LENGTH];
#endif
    rt_bool_t is_ready;
} hr_service_env_t;

static hr_service_env_t hr_service_env;
static uint8_t hr_update_flag = 0;
static uint8_t last_hr_value = 0;

int hr_service_subscriber_count(void)
{
    return hr_service_env.ref_count;
}

uint8_t hr_service_get_latest_bpm(void)
{
    return last_hr_value;
}

void hr_set_power(uint8_t arg)
{
    if (hr_service_env.is_ready == RT_FALSE)
    {
        LOG_W("PPG sensor not ready");
        return;
    }
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */

    rt_uint32_t power_mode = (arg > 0) ? RT_SENSOR_POWER_NORMAL : RT_SENSOR_POWER_DOWN;
    rt_err_t ret = rt_device_control(hr_service_env.device, RT_SENSOR_CTRL_SET_POWER, (void *)power_mode);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to set power mode %d", power_mode);
    }
    else
    {
        LOG_I("Set PPG sensor power mode %d", power_mode);
    }

#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */
}

static void hr_control_mode(rt_uint32_t power)
{
    if (hr_service_env.device)
    {
        rt_device_control(hr_service_env.device, RT_SENSOR_CTRL_SET_POWER, (void *)power);
    }
}

static int32_t hr_subscribe(datas_handle_t service)
{
    if (hr_service_env.timer == NULL)
    {
        LOG_E("hr_subscribe, timer is NULL");
        return -RT_ERROR;
    }

    if (hr_service_env.ref_count == 0)
    {
        LOG_I("hr_subscribe, start timer");
        rt_timer_start(hr_service_env.timer);
    }

    hr_service_env.ref_count++;
    LOG_I("hr_subscribe, ref_count %d", hr_service_env.ref_count);
    return RT_EOK;
}

static int32_t hr_unsubscribe(datas_handle_t service)
{
    hr_service_env.ref_count--;
    LOG_I("hr_unsubscribe, ref_count %d", hr_service_env.ref_count);

    if (hr_service_env.timer && hr_service_env.ref_count == 0)
    {
        rt_timer_stop(hr_service_env.timer);
    }

    return RT_EOK;
}

static rt_err_t hr_service_config(datas_handle_t service, uint8_t *config)
{
    // Update service configuration. Save config for future filtering.
    return RT_EOK;
}

static rt_err_t hr_service_ping(datas_handle_t service, uint8_t *data)
{
    hr_service_env_t *env = &hr_service_env;

    if (!env->device)
    {
        return RT_ERROR;
    }
    LOG_D("hr_service_ping %d", *data);

    return rt_device_control(env->device, RT_SENSOR_CTRL_SELF_TEST, data);
}

static rt_err_t hr_service_save(datas_handle_t service, uint8_t flag)
{
    hr_service_env_t *env = &hr_service_env;

    if (!env->device)
    {
        return RT_ERROR;
    }
    // check flag, save max hr, min hr, peace hr, normal data?

    return RT_EOK;
}

static data_req_t *hr_service_get_max_min(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == HRS_MAX_MIN_LEN) // just return max , min , current rhr for test
        {
            uint8_t maxmin[HRS_MAX_MIN_LEN];
            maxmin[0] = env->env.max_hr;
            maxmin[1] = env->env.min_hr;
            maxmin[2] = env->env.rhr;
            memcpy(r->data, maxmin, len);
        }
    }

    return r;
}

static data_req_t *hr_service_get_day_table(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        if (len == HRS_DAY_TABLE_LEN) // get day value : 24 hour
        {
            uint8_t today[HRS_DAY_TABLE_LEN];
            int i;

            r->len = len;
            for (i = 0; i < HRS_DAY_TABLE_LEN; i++)
                today[i] = env->env.hour[i].hr_value;
            memcpy(r->data, today, len);
        }
    }
    return r;
}

static data_req_t *hr_service_get_mon_table(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == HRS_MON_TABLE_LEN) // get rhr: 30 days
        {
            uint8_t mon[HRS_MON_TABLE_LEN];
            int i;
            for (i = 0; i < HRS_MON_TABLE_LEN; i++)
                mon[i] = env->env.rhr_arr[i].hr_value;
            memcpy(r->data, mon, len);
        }
    }
    return r;
}

static data_req_t *hr_service_get_region(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == HRS_REGION_LEN) // get region, ul, ana, aer, hiit, warmup
        {
            uint8_t region[HRS_REGION_LEN];
            int i;
            for (i = 0; i < HRS_REGION_LEN; i++)
                region[i] = env->env.status_arr[i];
            memcpy(r->data, region, len);
        }
    }
    return r;
}

static data_req_t *hr_service_get_hist_rhr(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == HRS_RHR_HIST_LEN) // get max rhr, min rhr, average rhr
        {
            uint8_t rhr[HRS_RHR_HIST_LEN];
            int i;
            uint32_t total = 0;
            // get max rhr
            rhr[0] = env->env.rhr_arr[0].hr_value;
            rhr[1] = env->env.rhr_arr[0].hr_value;

            for (i = 0; i < HRS_MON_TABLE_LEN; i++)
            {
                if (env->env.rhr_arr[i].hr_value > rhr[0])
                    rhr[0] = env->env.rhr_arr[i].hr_value;
                if (env->env.rhr_arr[i].hr_value < rhr[1])
                    rhr[1] = env->env.rhr_arr[i].hr_value;
                total += (uint32_t)env->env.rhr_arr[i].hr_value;
            }
            rhr[2] = (uint8_t)(total / HRS_MON_TABLE_LEN);
            memcpy(r->data, rhr, len);
        }
    }
    return r;
}

// update data buffer after day switch
static int32_t hr_service_update(datas_handle_t service)
{
    hr_service_env_t *env = &hr_service_env;
    int i;
    time_t now = time(NULL);
    struct tm today = *localtime(&now);
    struct tm oldday = *localtime(&(env->env.today));
    if (today.tm_yday != oldday.tm_yday) // a new day, update saved day
    {
        today.tm_hour = 0;
        today.tm_min = 0;
        today.tm_sec = 0;
        env->env.today = mktime(&today);
        LOG_D("Hello new day");
    }
    else // day not change, do not need reset table
    {
        return 1;
    }

    env->cur_mode = HR_SPORT_CNT;
    env->cur_mcnt = 0;
    env->get_rhr.rhr_cnt = 0;
    env->get_rhr.rhr_value = 0;

    for (i = 0; i < HR_HOUR_CNT; i++)
        env->env.hour[i].hr_value = 0;

    for (i = 0; i < 29; i++)
        env->env.rhr_arr[i].hr_value = env->env.rhr_arr[i + 1].hr_value;
    env->env.rhr_arr[29].hr_value = env->env.rhr;
    env->env.rhr = 0;

    for (i = 0; i < HR_SPORT_CNT; i++)
        env->env.status_arr[i] = 0;

    env->env.max_hr = 0;
    env->env.min_hr = 0xff;

    hr_update_flag = HR_GET_DAY_HOURS | HR_GET_MON_RHR | HR_GET_REGION | HR_GET_MAX | HR_GET_MIN | HR_GET_RHR;

    return 0;
}

static int32_t hr_service_proc(datas_handle_t service, uint32_t data_size, uint8_t *data)
{
    hr_service_env_t *env = &hr_service_env;
    struct rt_sensor_data *value = (struct rt_sensor_data *)data;

#define HRABS(x, y) ((x) > (y) ? (x) - (y) : (y) - (x))

    if (value->data.hr <= 0) // no valid hr, do not save and process
        return 1;

    if (value->data.hr != last_hr_value)
    {
        last_hr_value = value->data.hr;
        datas_push_data_to_client(service, data_size, data);
    }

    hr_service_update(service);

    // 1. max, min check
    if (value->data.hr > env->env.max_hr)
    {
        LOG_D("Update Max HR from %d to %d", env->env.max_hr, value->data.hr);
        env->env.max_hr = value->data.hr;
        hr_update_flag |= HR_GET_MAX;
    }
    if (value->data.hr < env->env.min_hr)
    {
        env->env.min_hr = value->data.hr;
        LOG_D("Update Min HR to %d", env->env.min_hr);
        hr_update_flag |= HR_GET_MIN;
    }

    // 2. check timer , if save this value(1 update per 30 min)
    // use system timer instead time stamp check
    {
        time_t now = time(NULL);
        struct tm *today = localtime(&now);
        uint8_t hour_idx;
        hour_idx = today->tm_hour * 60 / HR_HOUR_GAP_MIN + today->tm_min / HR_HOUR_GAP_MIN;
        if (env->env.hour[hour_idx].hr_value == 0)
        {
            env->env.hour[hour_idx].hr_value = value->data.hr;
            LOG_D("Update hour table %d: %d", hour_idx, value->data.hr);
            hr_update_flag |= HR_GET_DAY_HOURS;
        }
    }

    // 3. update RHR, diaplay and today
    if ((value->data.hr > 50) && (value->data.hr < 80))
    {
        if (env->get_rhr.rhr_cnt == 0)
        {
            env->get_rhr.rhr_value = value->data.hr;
            env->get_rhr.rhr_cnt++;
        }
        else
        {
            if (HRABS(value->data.hr, env->get_rhr.rhr_value) < 5)
                env->get_rhr.rhr_cnt++;
            else
                env->get_rhr.rhr_cnt = 0;
            if (env->get_rhr.rhr_cnt >= HR_RHR_THRESHOLD) // update rhr
            {
                LOG_D("Update RHR TO %d", env->get_rhr.rhr_value);
                env->get_rhr.rhr_cnt = 0;
                env->env.rhr = env->get_rhr.rhr_value;
                hr_update_flag |= HR_GET_RHR;
            }
        }
    }
    else
    {
        env->get_rhr.rhr_cnt = 0;
    }

    // 4. sport mode update
    if (env->env.rhr > 0)
    {
        if (value->data.hr >= HR_ULTIMATE_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_ULTIMATE_LIMIT)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_ULTIMATE_LIMIT]++;
                    env->cur_mcnt = 0;
                    LOG_D("Ultimate limit %d min", env->env.status_arr[HR_ULTIMATE_LIMIT]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_ULTIMATE_LIMIT;
                env->cur_mcnt = 0;
            }
        }
        else if (value->data.hr >= HR_ANAEROBIC_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_ANAEROBIC)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_ANAEROBIC]++;
                    env->cur_mcnt = 0;
                    LOG_D("Anaerobic %d min", env->env.status_arr[HR_ANAEROBIC]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_ANAEROBIC;
                env->cur_mcnt = 0;
            }
        }
        else if (value->data.hr >= HR_AEROBIC_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_AEROBIC)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_AEROBIC]++;
                    env->cur_mcnt = 0;
                    LOG_D("Aerobic %d min", env->env.status_arr[HR_AEROBIC]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_AEROBIC;
                env->cur_mcnt = 0;
            }
        }
        else if (value->data.hr >= HR_HIIT_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_HIIT)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_HIIT]++;
                    env->cur_mcnt = 0;
                    LOG_D("Hiit %d min", env->env.status_arr[HR_HIIT]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_HIIT;
                env->cur_mcnt = 0;
            }
        }
        else if (value->data.hr >= HR_WARMUP_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_WARM_UP)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_WARM_UP]++;
                    env->cur_mcnt = 0;
                    LOG_D("Warm up %d min", env->env.status_arr[HR_WARM_UP]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_WARM_UP;
                env->cur_mcnt = 0;
            }
        }
        else
        {
            env->cur_mode = HR_SPORT_CNT;
            env->cur_mcnt = 0;
        }
    }

    // Process flag updates and send data to clients
    if ((hr_update_flag & HR_GET_MAX) || (hr_update_flag & HR_GET_MIN) || (hr_update_flag & HR_GET_RHR))
    {
        data_req_t *result = hr_service_get_max_min(service, HRS_MAX_MIN_LEN);
        if (result)
        {
            datas_push_maxmin_to_client(service, result->len, result->data);
            rt_free(result);
        }
    }
    if (hr_update_flag & HR_GET_DAY_HOURS)
    {
        data_req_t *result = hr_service_get_day_table(service, HRS_DAY_TABLE_LEN);
        if (result)
        {
            datas_push_day_table_to_client(service, result->len, result->data);
            rt_free(result);
        }
    }
    if (hr_update_flag & HR_GET_MON_RHR)
    {
        data_req_t *result = hr_service_get_mon_table(service, HRS_MON_TABLE_LEN);
        if (result)
        {
            datas_push_mon_table_to_client(service, result->len, result->data);
            rt_free(result);
            result = NULL;
        }
        result = hr_service_get_hist_rhr(service, HRS_RHR_HIST_LEN);
        if (result)
        {
            datas_push_rhr_value_to_client(service, result->len, result->data);
            rt_free(result);
        }
    }
    if (hr_update_flag & HR_GET_REGION)
    {
        data_req_t *result = hr_service_get_region(service, HRS_REGION_LEN);
        if (result)
        {
            datas_push_region_to_client(service, result->len, result->data);
            rt_free(result);
        }
    }
    hr_update_flag = 0;

    return 0;
}

static int32_t hr_service_data_fetch(datas_handle_t service, uint32_t data_size, uint8_t **data)
{
    hr_service_env_t *env = &hr_service_env;
    rt_size_t size;

    RT_ASSERT(data);
    *data = NULL;
    if (!env->device)
    {
        return -1;
    }

    *data = (uint8_t *)&env->data;
    size = rt_device_read(env->device, 0, *data, 1);
    if (size != 1)
    {
        LOG_W("hr_service_data_fetch size:%d", size);
        *data = NULL;
        return 0;
    }

    return sizeof(env->data);
}

bool hr_service_filter(data_req_t *config, uint16_t msg_id, uint32_t len, uint8_t *data)
{
    // Check if config is compatible with current config.
    return true;
}

static int32_t hr_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    // hr_service_update(service);
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        int res = hr_subscribe(service);
        if (res == RT_EOK)
        {
            hr_service_env_t *env = &hr_service_env;
            env->hr_subscribed = RT_TRUE;
            hr_control_mode(RT_SENSOR_POWER_HIGH);
        }
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        int res = hr_unsubscribe(service);
        if (res == RT_EOK)
        {
            hr_service_env_t *env = &hr_service_env;
            env->hr_subscribed = RT_FALSE;
            // hr_control_mode(RT_SENSOR_POWER_NORMAL);
        }
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = hr_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_PING_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = hr_service_ping(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_TX_REQ:
        // Placeholder for TX request handling
        break;
    case MSG_SERVICE_RX_REQ:
        // Placeholder for RX request handling
        break;
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        uint8_t *data;

        RT_ASSERT(data_ind);

        int32_t size = hr_service_data_fetch(service, data_ind->len, &data);
        if (size > 0)
        {
            hr_service_proc(service, data_ind->len, data);
        }
        break;
    }
    case MSG_SERVICE_SLEEP_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        // before go to sleep, save data, stop algorithm if needed
        rt_err_t result = hr_service_save(service, 0xff);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_HR_DAY_TABLE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = hr_service_get_day_table(service, (uint16_t)req[0]);
        if (result)
        {
            datas_push_day_table_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_HR_MON_TABLE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = hr_service_get_mon_table(service, (uint16_t)req[0]);
        if (result)
        {
            datas_push_mon_table_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_HR_REGION_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = hr_service_get_region(service, (uint16_t)req[0]);
        if (result)
        {
            datas_push_region_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_HR_MAX_MIN_REQ:
    {
        uint8_t *req = data_service_get_msg_body(msg);
        data_req_t *result = hr_service_get_max_min(service, (uint16_t)req[0]);
        if (result)
        {
            datas_push_maxmin_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_RHR_VALUE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = hr_service_get_hist_rhr(service, (uint16_t)req[0]);
        if (result)
        {
            datas_push_rhr_value_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    default:
        LOG_E("hr_service_msg_handler: unknown msg_id %d", msg->msg_id);
        break;
    }

    return 0;
}

static data_service_config_t hr_service_cb =
    {
        .max_client_num = 1,
        .queue = RT_NULL,
        .data_filter = hr_service_filter,
        .msg_handler = hr_service_msg_handler,
};

static void timeout_ind(void *param)
{
    if (hr_service_env.service)
    {
        LOG_D("hr_service timeout_ind");
        datas_ind_size(hr_service_env.service, sizeof(hr_service_env.data));
    }
    else
    {
        LOG_E("hr_service timeout_ind: service is NULL");
    }
}

rt_bool_t is_ppg_service_ready(void)
{
    return hr_service_env.is_ready;
}

#ifdef USING_PPG_FOR_GESTURE_RECOGNITION

static int32_t ppg_subscribe(datas_handle_t service)
{
    if (hr_service_env.ppg_timer)
    {
        rt_timer_start(hr_service_env.ppg_timer);
    }
    return RT_EOK;
}

static int32_t ppg_unsubscribe(datas_handle_t service)
{
    if (hr_service_env.ppg_timer)
    {
        rt_timer_stop(hr_service_env.ppg_timer);
    }
    return RT_EOK;
}

static int32_t ppg_service_fifo_fetch(datas_handle_t service, uint32_t data_size, struct rt_sensor_data *data)
{
    hr_service_env_t *env = &hr_service_env;
    rt_size_t size;

    if (!env->device)
    {
        return -1;
    }

    size = rt_device_read(env->device, 0, data, PPG_FIFO_LENGTH);
    if (size != PPG_FIFO_LENGTH)
    {
        return -1;
    }

    for (size_t i = 0; i < PPG_FIFO_LENGTH; i++)
    {
        env->ppg_fifo[i] = data[i].data.light;
    }

    return sizeof(env->ppg_fifo);
}

static int32_t ppg_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        int res = ppg_subscribe(service);
        if (res == RT_EOK)
        {
            hr_service_env.ppg_subscribed = RT_TRUE;
        }
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        int res = ppg_unsubscribe(service);
        if (res == RT_EOK)
        {
            hr_service_env.ppg_subscribed = RT_FALSE;
        }
        break;
    }
    case MSG_SERVICE_TX_REQ:
    case MSG_SERVICE_RX_REQ:
        // Placeholder for TX/RX request handling
        break;
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        RT_ASSERT(data_ind);

        uint8_t buf_len = data_ind->len / sizeof(struct rt_sensor_data);
        if (buf_len == PPG_FIFO_LENGTH)
        {
            hr_service_env_t *env = &hr_service_env;
            int32_t size = ppg_service_fifo_fetch(service, data_ind->len, env->ppg_sensor_data);
            if (size > 0)
            {
                datas_push_data_to_client(service, sizeof(env->ppg_fifo), (uint8_t *)env->ppg_fifo);
            }
        }
        break;
    }
    default:
        LOG_E("ppg_service_msg_handler: unknown msg_id %d", msg->msg_id);
        break;
    }

    return 0;
}

static data_service_config_t ppg_service_cb =
    {
        .max_client_num = 1,
        .queue = RT_NULL,
        .data_filter = hr_service_filter,
        .msg_handler = ppg_service_msg_handler,
};

void notify_ppg_data(void)
{
    if (hr_service_env.ppg_service && hr_service_env.ppg_subscribed)
    {
        datas_ind_size(hr_service_env.ppg_service, PPG_SENSOR_BUF_SIZE);
    }
}

static void ppg_timeout_ind(void *param)
{
    notify_ppg_data();
}
#endif

/* ===================== Background daily HR-curve sampler =====================
   LCPU-autonomous: every BG_HR_PERIOD_MS, if worn and not on charger, power the
   PPG LED for a short burst, read a representative BPM straight from the sensor
   (independent of the data_service pub/sub path), and push it to HCPU -> phone
   to build a daily heart-rate curve. SOFT rt_timers so it fires while the
   screen is off (the whole point of a daily curve). If the Exercise app is
   already measuring (ref_count>0) we don't start our own LED burst — we just
   forward the latest value it produced. */
#ifndef SOC_BF0_HCPU
#include "wear_detect.h"
#include "bloc_battery.h"

#define BG_HR_PERIOD_MS (15 * 60 * 1000) /* sample every 15 min            */
#define BG_HR_BURST_MS  (25 * 1000)      /* LED on this long: settle + lock */
#define BG_HR_SAMPLE_MS (1000)           /* read cadence during the burst   */

static rt_timer_t bg_hr_period_timer = RT_NULL;
static rt_timer_t bg_hr_sample_timer = RT_NULL;
static rt_bool_t bg_hr_bursting = RT_FALSE;
static uint32_t bg_hr_burst_deadline_ms = 0;
static uint8_t bg_hr_burst_best = 0;

static rt_bool_t bg_hr_should_sample(void)
{
    if (hr_service_env.is_ready != RT_TRUE) return RT_FALSE;
    if (battery_get_charge_state()->is_charging) return RT_FALSE; /* on charger */
    if (!wear_detect_is_wearing()) return RT_FALSE;               /* off wrist  */
    return RT_TRUE;
}

static void bg_hr_finish_burst(void)
{
    /* Forward the best BPM seen this burst, then power the LED back off. */
    if (bg_hr_burst_best > 0 && watch_sys_sync.notify_hr_sample)
    {
        watch_sys_sync.notify_hr_sample((uint32_t)time(NULL), bg_hr_burst_best);
    }
    if (bg_hr_sample_timer) rt_timer_stop(bg_hr_sample_timer);
    /* Only power down if no foreground (Exercise app) subscriber needs PPG.
       There is a tiny (two-statement) window where a subscribe on the ds_proc
       thread could flip ref_count between this read and hr_set_power(0); the
       impact is at worst a transient LED-off that the Exercise path re-powers,
       and it is consistent with the pre-existing unsynchronized PPG-power model
       (multiple writers gated only by ref_count). Not worth a cross-path lock. */
    if (hr_service_env.ref_count == 0)
    {
        hr_set_power(0);
    }
    bg_hr_bursting = RT_FALSE;
}

static void bg_hr_sample_cb(void *param)
{
    (void)param;
    /* Read one HR sample directly from the sensor (same call hr_service uses). */
    struct rt_sensor_data sd;
    if (hr_service_env.device &&
        rt_device_read(hr_service_env.device, 0, &sd, 1) == 1)
    {
        if (sd.data.hr > 0)
        {
            bg_hr_burst_best = (uint8_t)sd.data.hr;
        }
    }
    if (rt_tick_get_millisecond() >= bg_hr_burst_deadline_ms)
    {
        bg_hr_finish_burst();
    }
}

static void bg_hr_period_cb(void *param)
{
    (void)param;
    if (bg_hr_bursting) return;
    if (!bg_hr_should_sample()) return;

    /* Exercise app already measuring -> just forward its latest value, no
       extra LED burst. */
    if (hr_service_env.ref_count > 0)
    {
        uint8_t bpm = hr_service_get_latest_bpm();
        if (bpm > 0 && watch_sys_sync.notify_hr_sample)
            watch_sys_sync.notify_hr_sample((uint32_t)time(NULL), bpm);
        return;
    }

    /* Start a fresh burst: power the LED, read at 1 Hz until the deadline.
       Bail BEFORE powering on if the sample timer is missing (creation failed)
       — otherwise the LED would turn on with nothing to ever finish the burst
       (stuck-on power leak + sampler wedged). */
    if (bg_hr_sample_timer == RT_NULL) return;
    bg_hr_bursting = RT_TRUE;
    bg_hr_burst_best = 0;
    bg_hr_burst_deadline_ms = rt_tick_get_millisecond() + BG_HR_BURST_MS;
    hr_set_power(1);
    rt_timer_start(bg_hr_sample_timer);
}

static void bg_hr_init(void)
{
    bg_hr_period_timer = rt_timer_create(
        "bghr_p", bg_hr_period_cb, RT_NULL,
        rt_tick_from_millisecond(BG_HR_PERIOD_MS),
        RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    bg_hr_sample_timer = rt_timer_create(
        "bghr_s", bg_hr_sample_cb, RT_NULL,
        rt_tick_from_millisecond(BG_HR_SAMPLE_MS),
        RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (bg_hr_period_timer) rt_timer_start(bg_hr_period_timer);
}
#endif /* !SOC_BF0_HCPU */

int hr_service_register(void)
{
    struct rt_sensor_config cfg;
    hr_service_env_t *env = &hr_service_env;
    rt_err_t err;
    int i;
    int res = 0;

    hr_service_env.is_ready = RT_FALSE;

#ifdef HR_USING_GH3011
    res = rt_hw_gh3011_init();
#elif defined(HR_USING_GH3018)
    res = rt_hw_gh3018_init();
#elif defined(HR_USING_AFE4404)
    res = rt_hw_afe4404_init();
#endif

    if (res != 0)
    {
        LOG_W("DataS init %s fail", HR_MODEL_NAME);
        return 1;
    }

#ifdef HR_USING_GH3011
    cfg.intf.dev_name = GH3011_I2C_BUS;
    cfg.irq_pin.pin = GH3011_INT_BIT; // note: if driver is LCPU, iqr_pin must be config
    res = rt_hw_gh3011_register(HR_MODEL_NAME, &cfg);
#elif defined(HR_USING_GH3018)
    cfg.intf.dev_name = GH3018_I2C_BUS;
#ifdef GH3018_INT_BIT
    cfg.irq_pin.pin = GH3018_INT_BIT;
#else
    cfg.irq_pin.pin = PPG_INT_PIN; // note: if driver is LCPU, iqr_pin must be config
#endif

    res = rt_hw_gh3018_register(HR_MODEL_NAME, &cfg);
#elif defined(HR_USING_AFE4404)
    cfg.intf.dev_name = AFE4404_I2C_BUS;
    res = rt_hw_afe4404_register(HR_MODEL_NAME, &cfg);
#endif

    if (res != 0)
    {
        LOG_W("DataS register %s fail", HR_MODEL_NAME);
        return 1;
    }

    env->device = rt_device_find(HR_DEV_NAME);
    if (env->device == NULL)
    {
        LOG_W("DataS find device %s fail", HR_DEV_NAME);
        return 1;
    }

    err = rt_device_open(env->device, RT_DEVICE_OFLAG_RDONLY);
    if (err != RT_EOK)
    {
        LOG_W("DataS open device %s fail", HR_DEV_NAME);
        return 1;
    }

    hr_service_env.service = datas_register("HR", &hr_service_cb);
    RT_ASSERT(hr_service_env.service);

    hr_service_env.timer = rt_timer_create("HR", timeout_ind, 0,
                                           rt_tick_from_millisecond(HR_TIMER_PERIOD_MS),
                                           RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    RT_ASSERT(hr_service_env.timer);

#ifdef USING_PPG_FOR_GESTURE_RECOGNITION
    hr_service_env.ppg_service = datas_register("PPG", &ppg_service_cb);
    RT_ASSERT(hr_service_env.ppg_service);

    hr_service_env.ppg_timer = rt_timer_create("PPG", ppg_timeout_ind, 0,
                                               rt_tick_from_millisecond(PPG_TIMER_PERIOD_MS),
                                               RT_TIMER_FLAG_PERIODIC);
    RT_ASSERT(hr_service_env.ppg_timer);
#endif

    // Initialize values
    hr_service_env.env.max_hr = 0;
    hr_service_env.env.min_hr = 0xff;

    for (i = 0; i < HR_HOUR_CNT; i++)
    {
        hr_service_env.env.hour[i].timestamp = i;
        hr_service_env.env.hour[i].hr_value = 0;
    }

    // Initialize time
    time_t now = time(NULL);
    struct tm today = *localtime(&now);
    today.tm_hour = 0;
    today.tm_min = 0;
    today.tm_sec = 0;
    hr_service_env.env.today = mktime(&today);

    hr_service_env.is_ready = RT_TRUE;
    // peripheral_provider.hr_set_power = hr_set_power;
#ifndef SOC_BF0_HCPU
    bg_hr_init(); /* start the 15-min background daily-HR-curve sampler */
#endif
    return 0;
}

int close_hr_service(void)
{
    hr_service_env_t *env = &hr_service_env;

    if (env->timer)
    {
        rt_timer_stop(env->timer);
        rt_timer_delete(env->timer);
        env->timer = RT_NULL;
    }
    if (env->service)
    {
        env->service = NULL;
    }
    if (env->device)
    {
        rt_device_close(env->device);
        env->device = RT_NULL;
    }
    env->is_ready = RT_FALSE;
    return 0;
}

INIT_COMPONENT_EXPORT(hr_service_register);
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/