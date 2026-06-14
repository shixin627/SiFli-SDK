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
#include "bsp_board.h"          /* CUSTOMER_BOARD_VER / BOARD_VER_29 */

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
#ifdef ACC_USING_BMI270
#include "bmi270_driver.h"   /* bmi270_accel_read() for the HR motion gate */
#endif
/* PPG-HR per-reading quality from the Goodix algo, plumbed via the gh3018 port
   (valid_score = confidence, valid_level = quality level). Diagnostics now; the
   output-quality gate (Apple-style "withhold on low signal quality") builds on
   it later. */
extern void gh3018_get_hr_quality(uint32_t *valid_score, uint32_t *valid_level);
/* Monotonic count of locked algo HR outputs; bg_hr snapshots it at burst start and
   ends warm-up the moment it moves (= algo locked this burst). See gh3018 port. */
extern uint32_t gh3018_get_hr_update_seq(void);

/* Two-stage HR sampling (literature pattern: a low-power signal prescreens
   sleep, PPG activates only when needed). The period timer ticks at the SLEEP
   cadence; while AWAKE we actually burst only every BG_HR_AWAKE_SKIP ticks, so
   the daily HR curve keeps its ~15 min rate and daytime PPG power is unchanged.
   Once sleep_fusion reports asleep (accel-only decision), sleep_service flips
   bg_hr_sleep_active and we burst every tick with a ≥60 s window so HR mean +
   std (HRV proxy) are valid for Deep/REM staging. PPG stays OFF while awake. */
#define BG_HR_PERIOD_MS      (3 * 60 * 1000) /* base tick = sleep-mode cadence    */
#define BG_HR_AWAKE_SKIP     5               /* awake: burst every 5th tick ≈15min */
/* The HBA algo restarts cold each burst and needs ~30 s to lock (hba_out_flag
   stays 0 until then); before that gh3018_get_hr() still returns the PREVIOUS
   burst's stale value. Drop that warm-up window from the HR stats, and keep
   every burst longer than it. */
#define BG_HR_WARMUP_MS      (30 * 1000)     /* warm-up FALLBACK cap; dynamic warm-up (HR update-seq) accepts earlier on real lock */
#define BG_HR_BURST_MS_AWAKE (40 * 1000)     /* warm-up + ~10 s to lock one BPM    */
#define BG_HR_BURST_MS_SLEEP (60 * 1000)     /* warm-up + ~30 s stable for HR std  */
#define BG_HR_SAMPLE_MS      (1000)          /* read cadence during the burst      */
/* HR output motion gate: each 1 Hz read also samples BMI270 accel; if the wrist
   moved this second (or within the guard window after), drop that HR read from
   both the published best and the mean/std window. Suppresses the PPG motion-
   artefact spikes the GH30x built-in comp lets through (its accel feed is time-
   warped ~6x -- see plan). HR-only: does not touch PPG raw or the algo feed. */
#define BG_HR_MOTION_DELTA_THRESH  6     /* >>10 LSB delta; ~0.4g between 1 Hz reads */
#define BG_HR_MOTION_GUARD_MS      3000  /* keep rejecting this long after motion    */

static rt_timer_t bg_hr_period_timer = RT_NULL;
static rt_timer_t bg_hr_sample_timer = RT_NULL;
static rt_bool_t bg_hr_bursting = RT_FALSE;
static uint32_t bg_hr_burst_deadline_ms = 0;
static uint32_t bg_hr_burst_accept_ms = 0;   /* reads before this tick are warm-up (fallback cap) */
static uint32_t bg_hr_burst_start_seq = 0;   /* gh3018 HR-update seq at burst start; dynamic warm-up baseline */
static uint8_t bg_hr_burst_best = 0;

/* Two-stage gate state. bg_hr_sleep_active is set by sleep_service when accel
   says we're asleep -> dense bursts; cleared otherwise -> ~15 min curve rate. */
static rt_bool_t bg_hr_sleep_active = RT_FALSE;
static uint8_t   bg_hr_awake_ticks = 0;                    /* awake skip counter */
static uint32_t  bg_hr_burst_ms = BG_HR_BURST_MS_AWAKE;    /* current burst len  */

/* Per-burst HR accumulators (Σ, Σ², count) over the burst's 1 Hz reads, used
   to publish a mean + std (HRV proxy) window for sleep_fusion. */
static uint32_t bg_hr_burst_sum = 0;
static uint32_t bg_hr_burst_sum_sq = 0;
static uint16_t bg_hr_burst_cnt = 0;
/* Per-burst sensor-read health: if EVERY read fails the cause is HW/I2C, not a
   weak optical signal -- lets bg_hr_finish_burst attribute SENSOR_FAULT vs
   NO_LOCK instead of lumping both as "no point". */
static uint16_t bg_hr_burst_reads = 0;
static uint16_t bg_hr_burst_readfail = 0;

/* Last completed burst's HR window. bg_hr_win_tick_ms == 0 => none yet. */
static uint8_t  bg_hr_win_mean = 0;
static uint8_t  bg_hr_win_std = 0;
static uint32_t bg_hr_win_tick_ms = 0;

/* HR output motion-gate state (see BG_HR_MOTION_* above). bghr_accel_delta()
   mirrors sleep_service.c prv_accel_delta_activity: read BMI270 accel and return
   the L1 delta vs the previous read in >>10 LSB units (±2g => ~16384 LSB/g, so a
   1g single-axis jerk ~= 16). Clearing bghr_prev_accel_valid at burst start
   re-seeds the delta so consecutive bursts don't cross-contaminate. */
static bool     bghr_prev_accel_valid = false;
static uint32_t bghr_last_motion_ms = 0;
static uint16_t bg_hr_burst_motion_rej = 0;   /* HR reads dropped for motion, per burst */

static uint32_t bghr_accel_delta(void)   /* 0 = read failed / first sample of burst */
{
#ifdef ACC_USING_BMI270
    static int16_t prev_ax = 0, prev_ay = 0, prev_az = 0;
    int16_t ax = 0, ay = 0, az = 0;
    if (bmi270_accel_read(&ax, &ay, &az) != 0)
        return 0;
    if (!bghr_prev_accel_valid)
    {
        prev_ax = ax; prev_ay = ay; prev_az = az;
        bghr_prev_accel_valid = true;
        return 0;
    }
    int32_t dx = (int32_t)ax - prev_ax;
    int32_t dy = (int32_t)ay - prev_ay;
    int32_t dz = (int32_t)az - prev_az;
    prev_ax = ax; prev_ay = ay; prev_az = az;
    uint32_t adx = (uint32_t)(dx >= 0 ? dx : -dx);
    uint32_t ady = (uint32_t)(dy >= 0 ? dy : -dy);
    uint32_t adz = (uint32_t)(dz >= 0 ? dz : -dz);
    return (adx + ady + adz) >> 10;
#else
    return 0;
#endif
}

/* Rolling median-of-N over accepted bg-HR reads. Rejects isolated PPG spikes
   (poor-contact / signal-quality artefacts that are NOT wrist motion -- the
   sleep case the accel motion-gate cannot catch) with no magic threshold: a
   lone 115 among ~55s does not move the median. We publish AND accumulate the
   median (Apple-style smoothed, outlier-rejected output) instead of the raw
   read. Reset per burst. */
#define BGHR_MED_WIN 5
static uint8_t bghr_med_buf[BGHR_MED_WIN];
static uint8_t bghr_med_cnt = 0;
static uint8_t bghr_med_idx = 0;

static uint8_t bghr_median_push(uint8_t v)
{
    bghr_med_buf[bghr_med_idx] = v;
    bghr_med_idx = (uint8_t)((bghr_med_idx + 1) % BGHR_MED_WIN);
    if (bghr_med_cnt < BGHR_MED_WIN) bghr_med_cnt++;
    uint8_t s[BGHR_MED_WIN];
    for (uint8_t i = 0; i < bghr_med_cnt; i++) s[i] = bghr_med_buf[i];
    for (uint8_t i = 1; i < bghr_med_cnt; i++)   /* insertion sort, <=5 elems */
    {
        uint8_t k = s[i];
        int j = (int)i - 1;
        while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
        s[j + 1] = k;
    }
    return s[bghr_med_cnt / 2];
}

/* Per-burst PPG-HR quality (min confidence + last level) for the burst log /
   the quality gate below. */
static uint32_t bg_hr_burst_qscore_min = 0xFFFFFFFFu;
static uint32_t bg_hr_burst_qlevel = 0;
static uint16_t bg_hr_burst_qual_rej = 0;  /* reads dropped by the quality gate, per burst */

/* Signal-quality gate (Apple-style "withhold low-confidence readings").
   The Goodix algo grades every output: valid_level 0->1->2 (higher = more
   reliable) and valid_score 0..100. Weak/loose/motion-contaminated sleep PPG
   yields level-0 outputs that the algo itself flags as unreliable; plotting
   them produces the jagged 50<->130 sleep curve. Drop reads below this bar so
   a burst emits a clean median or, if nothing qualifies, nothing at all
   (-> BGHR_NO_LOCK -> phone draws a "weak signal" gap instead of garbage).
   Conservative first cut: reject only level 0. Tune from the burst log's
   qual_rej / acc counts (bench) or the phone's gap rate (sleep unit). */
#define BGHR_MIN_QLEVEL 1

/* Integer std-dev from running Σx / Σx² (n ≤ ~25 keeps it inside uint32).
   Mirrors sleep_service's prv_compute_hr_std so the HRV feed and the
   classifier agree on the math. */
static uint8_t bg_hr_std_from_sums(uint32_t sum, uint32_t sum_sq, uint16_t n)
{
    if (n < 2) return 0;
    uint32_t mean = sum / n;
    uint32_t mean_sq = mean * mean;
    uint32_t mean_of_sq = sum_sq / n;
    if (mean_of_sq <= mean_sq) return 0;
    uint32_t var = mean_of_sq - mean_sq;
    uint32_t r = var >> 1;
    if (r == 0) r = 1;
    for (uint8_t i = 0; i < 8; i++)
    {
        uint32_t next = (r + var / r) >> 1;
        if (next >= r) break;
        r = next;
    }
    return (r > 0xFFu) ? (uint8_t)0xFFu : (uint8_t)r;
}

/* Snapshot of the most recent completed burst's HR window. See header. */
bool hr_service_get_hr_window(uint8_t *mean_bpm, uint8_t *std_bpm, uint32_t *age_ms)
{
    if (bg_hr_win_tick_ms == 0) return false; /* no burst completed yet */
    if (mean_bpm) *mean_bpm = bg_hr_win_mean;
    if (std_bpm)  *std_bpm = bg_hr_win_std;
    if (age_ms)
    {
        uint32_t now = rt_tick_get_millisecond();
        *age_ms = (now >= bg_hr_win_tick_ms) ? (now - bg_hr_win_tick_ms) : 0;
    }
    return true;
}

/* Two-stage gate: sleep_service calls this when accel-detected sleep starts/ends
   so we only burn dense PPG (60 s every 3 min) while actually asleep. */
void hr_service_set_sleep_active(bool active)
{
    bg_hr_sleep_active = active ? RT_TRUE : RT_FALSE;
    if (!active) bg_hr_awake_ticks = 0; /* re-arm the awake skip cadence */
}

/* ===== Skip-reason instrumentation ==================================
   Every period tick the sampler either emits one HR point (OK_SENT) or
   bails for one specific reason. Counting each outcome lets a full night
   be classified after the fact instead of guessed: NOT_WORN => fit /
   wear-detect, NO_LOCK => PPG signal / algorithm, CHARGING => on charger,
   THROTTLE => normal daytime ~15 min rate. Read by stage A's 5-min bucket
   flush for the 0x40 uplink (phone-only; the watch can't be tethered). */
enum
{
    BGHR_OK = 0,    /* one HR point forwarded to phone                 */
    BGHR_NO_LOCK,   /* burst completed but PPG never locked a BPM      */
    BGHR_NOT_WORN,  /* wear_detect says off-wrist                      */
    BGHR_CHARGING,  /* on charger (v29 = off-wrist proxy)              */
    BGHR_NOT_READY, /* PPG sensor not initialised                      */
    BGHR_BUSY,      /* previous burst still running                    */
    BGHR_NO_TIMER,  /* sample timer missing (creation failed)          */
    BGHR_FWD_ZERO,  /* Exercise app subscribed but reported 0 bpm      */
    BGHR_THROTTLE,    /* awake skip -- normal ~15 min daytime cadence  */
    BGHR_SENSOR_FAULT,/* every sensor read this burst failed (I2C / HW) */
    BGHR_REASON_CNT
};
/* ---- 5-min bucket accumulator (stage A) -------------------------------
   The phone bins the HR curve into 5-min buckets. We tally each tick outcome
   PER bucket; when the wall-clock bucket rolls over (detected in
   bg_hr_period_cb) we flush: if the bucket produced >=1 HR point the curve
   already covers it (no skip); else uplink ONE skip with the bucket's dominant
   non-OK reason. Counters are per-bucket, reset on every flush. */
#define BG_HR_BUCKET_SEC     (5 * 60)            /* phone gap-bucket width = 5 min */
static uint32_t  bg_hr_bucket_id = 0;            /* time()/BG_HR_BUCKET_SEC, 0=unset */
static rt_bool_t bg_hr_bucket_had_point = RT_FALSE;
static uint16_t  bg_hr_bucket_reason[BGHR_REASON_CNT];

static void bg_hr_note(int reason)
{
    if (reason < 0 || reason >= BGHR_REASON_CNT) return;
    if (reason == BGHR_OK)
        bg_hr_bucket_had_point = RT_TRUE;        /* curve covers this bucket */
    else
        bg_hr_bucket_reason[reason]++;
}

/* Wire reason codes — FROZEN contract (ADR-0011 D2), decoupled from the
   internal BGHR_* enum order so refactors here never shift the wire values. */
enum
{
    BGHR_WIRE_OK = 0,            /* never sent */
    BGHR_WIRE_NOT_WORN = 1,
    BGHR_WIRE_NO_LOCK = 2,
    BGHR_WIRE_CHARGING = 3,
    BGHR_WIRE_NOT_READY = 4,
    BGHR_WIRE_SENSOR_FAULT = 5,
    BGHR_WIRE_POWER_SAVE = 6,
    BGHR_WIRE_OTHER = 7,
};

static uint8_t bg_hr_reason_to_wire(int r)
{
    switch (r)
    {
    case BGHR_NOT_WORN:     return BGHR_WIRE_NOT_WORN;
    case BGHR_NO_LOCK:      return BGHR_WIRE_NO_LOCK;
    case BGHR_CHARGING:     return BGHR_WIRE_CHARGING;
    case BGHR_NOT_READY:    return BGHR_WIRE_NOT_READY;
    case BGHR_SENSOR_FAULT: return BGHR_WIRE_SENSOR_FAULT;
    case BGHR_THROTTLE:     return BGHR_WIRE_POWER_SAVE;
    case BGHR_BUSY:
    case BGHR_NO_TIMER:
    case BGHR_FWD_ZERO:     return BGHR_WIRE_OTHER;
    default:                return BGHR_WIRE_OK;  /* OK/unknown -> don't send */
    }
}

/* Most frequent non-OK reason this bucket (ties: lowest enum index wins). */
static int bg_hr_bucket_dominant(void)
{
    int best = BGHR_OK;
    uint16_t best_n = 0;
    for (int i = 1; i < BGHR_REASON_CNT; i++)    /* i=0 is OK, skip */
    {
        if (bg_hr_bucket_reason[i] > best_n)
        {
            best_n = bg_hr_bucket_reason[i];
            best = i;
        }
    }
    return best;
}

/* Flush the bucket that just ended. Emit one skip iff it produced no HR point.
   bucket_start_ts is the bucket's first second (watch wall-clock-as-UTC, same
   convention as the HR-curve 0x10 samples, so the phone lines them up on one
   axis via watchEpochToLocal). */
static void bg_hr_flush_bucket(uint32_t bucket_start_ts)
{
    if (!bg_hr_bucket_had_point)
    {
        uint8_t wire = bg_hr_reason_to_wire(bg_hr_bucket_dominant());
        if (wire != BGHR_WIRE_OK && watch_sys_sync.notify_hr_skip)
            watch_sys_sync.notify_hr_skip(bucket_start_ts, wire);
    }
    bg_hr_bucket_had_point = RT_FALSE;
    for (int i = 0; i < BGHR_REASON_CNT; i++) bg_hr_bucket_reason[i] = 0;
}

/* Same gate as before, but returns the SPECIFIC reason instead of a bool so
   the instrumentation can attribute every skipped tick. BGHR_OK => sample. */
static int bg_hr_skip_reason(void)
{
    if (hr_service_env.is_ready != RT_TRUE) return BGHR_NOT_READY;
#if (CUSTOMER_BOARD_VER == BOARD_VER_29)
    /* Production (v29): on the charger means off-wrist, so skip PPG entirely.
       Dev boards (v28) deliberately keep sampling while charging so wear
       detection stays live for bench use. NOTE: charging current can inject
       noise into the optical ADC, so DC/PI read while charging may be less
       reliable -- accepted dev-only trade-off. */
    if (battery_get_charge_state()->is_charging) return BGHR_CHARGING; /* on charger */
#endif
    if (!wear_detect_is_wearing()) return BGHR_NOT_WORN;               /* off wrist  */
    return BGHR_OK;
}

static void bg_hr_finish_burst(void)
{
    /* Burst summary for on-wrist tuning (LCPU console / uart4). qmin = lowest
       Goodix valid_score this burst; correlate low qmin with spiky bursts to
       calibrate a future signal-quality gate. */
    LOG_I("bg_hr burst: reads=%u acc=%u motion_rej=%u qual_rej=%u best=%u qmin=%u qlvl=%u",
          (unsigned)bg_hr_burst_reads, (unsigned)bg_hr_burst_cnt,
          (unsigned)bg_hr_burst_motion_rej, (unsigned)bg_hr_burst_qual_rej,
          (unsigned)bg_hr_burst_best,
          (unsigned)bg_hr_burst_qscore_min, (unsigned)bg_hr_burst_qlevel);

    /* TEMPORARY: forward this burst's quality summary to the phone CSV via the
       wear-diag pipe so BGHR_MIN_QLEVEL can be tuned from real on-wrist data
       (the sleep unit has no serial). Repurposed fields — see
       WEAR_DIAG_EVT_HR_BURST in watch_sys_service.h. Remove with the gate
       tuning. Only when something was actually read this burst. */
    if (watch_sys_sync.notify_wear_diag && bg_hr_burst_reads > 0)
    {
        uint32_t qmin = (bg_hr_burst_qscore_min == 0xFFFFFFFFu)
                            ? 0 : bg_hr_burst_qscore_min;
        watch_sys_wear_diag_t hd;
        hd.ts = (uint32_t)time(NULL);
        hd.evt = WEAR_DIAG_EVT_HR_BURST;
        hd.status = (uint8_t)(bg_hr_burst_qlevel > 0xFF ? 0xFF : bg_hr_burst_qlevel);
        hd.dc_q4 = (bg_hr_burst_reads > 0xFFFF) ? 0xFFFF : (uint16_t)bg_hr_burst_reads;
        hd.pi_e6 = (bg_hr_burst_cnt > 0xFFFF) ? 0xFFFF : (uint16_t)bg_hr_burst_cnt;
        hd.pi_range_e6 = (bg_hr_burst_qual_rej > 0xFFFF) ? 0xFFFF
                                                         : (uint16_t)bg_hr_burst_qual_rej;
        hd.imu_var_e4 = (qmin > 0xFFFF) ? 0xFFFF : (uint16_t)qmin;
        watch_sys_sync.notify_wear_diag(&hd);
    }
    /* Forward the best BPM seen this burst, then power the LED back off. */
    if (bg_hr_burst_best > 0 && watch_sys_sync.notify_hr_sample)
    {
        watch_sys_sync.notify_hr_sample((uint32_t)time(NULL), bg_hr_burst_best);
        bg_hr_note(BGHR_OK);
    }
    else if (bg_hr_burst_reads > 0 && bg_hr_burst_readfail == bg_hr_burst_reads)
    {
        /* Every sensor read this burst failed (device gone / I2C / HW) --
           a hardware fault, distinct from a clean signal that never locked. */
        bg_hr_note(BGHR_SENSOR_FAULT);
    }
    else
    {
        /* Burst ran to completion but PPG never locked a valid BPM (best==0).
           This is the single most common "missing HR point" cause during
           sleep: motion artefact / loose fit / poor optical contact. */
        bg_hr_note(BGHR_NO_LOCK);
    }
    /* Publish the burst's HR window (mean + std over its 1 Hz reads) for
       sleep_fusion's Deep / REM staging. Need ≥2 samples for a meaningful std. */
    if (bg_hr_burst_cnt >= 2)
    {
        bg_hr_win_mean = (uint8_t)(bg_hr_burst_sum / bg_hr_burst_cnt);
        bg_hr_win_std = bg_hr_std_from_sums(bg_hr_burst_sum, bg_hr_burst_sum_sq,
                                            bg_hr_burst_cnt);
        bg_hr_win_tick_ms = rt_tick_get_millisecond();
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
    /* Read one HR sample directly from the sensor (same call hr_service uses).
       DATA-FLOW (easy to mis-trace): sd.data.hr <- gh3018_get_hr() <- loc_hb_value
       <- gh3018_set_hr(), wired in the GH3018 driver's handle_algo_result_update()
       (see commit b0400d1a7 + the "LIVE HR PATH" note there). If HR reads 0
       system-wide, suspect that plumbing, not this sampler. */
    struct rt_sensor_data sd;
    int rd = (hr_service_env.device)
                 ? rt_device_read(hr_service_env.device, 0, &sd, 1) : 0;
    bg_hr_burst_reads++;
    if (rd != 1) bg_hr_burst_readfail++;  /* device gone or I2C/HW read failed */

    /* Sample wrist motion every tick (keeps the accel delta chain continuous). */
    uint32_t bghr_mv = bghr_accel_delta();
    uint32_t bghr_now_ms = rt_tick_get_millisecond();
    if (bghr_mv >= BG_HR_MOTION_DELTA_THRESH) bghr_last_motion_ms = bghr_now_ms;
    bool bghr_motion = bghr_prev_accel_valid &&
                       (bghr_now_ms - bghr_last_motion_ms) < BG_HR_MOTION_GUARD_MS;

    if (rd == 1)
    {
        /* Dynamic warm-up: accept as soon as the algo emits a FRESH locked value
           this burst (gh3018 HR update-seq moved past the burst-start baseline) --
           typically well under the old fixed 30s. The fixed accept_ms is kept ONLY
           as a fallback, so behaviour is never worse than before if the seq never
           moves (algo never locked this burst). Until one of those holds,
           gh3018_get_hr() returns the previous burst's stale value -- must not accept. */
        bool bghr_algo_locked = (gh3018_get_hr_update_seq() != bg_hr_burst_start_seq);
        if (sd.data.hr > 0 && (bghr_algo_locked || bghr_now_ms >= bg_hr_burst_accept_ms))
        {
            /* step->1 (the accel-feed alignment fix, already on main) restored the
               GH30x built-in motion compensation, so this external motion-gate's
               ROOT CAUSE is gone. Demoted REJECT -> diagnostic-only: still count
               motion reads, but no longer drop them -- over-rejecting while sitting
               still starved the 5-tap median window and made the daily curve jumpy.
               The median below still suppresses isolated motion spikes. Re-add a
               looser reject (higher delta thresh) only if dynamic over-read returns. */
            if (bghr_motion)
            {
                bg_hr_burst_motion_rej++;
            }
            if (sd.data.hr < 240)
            {
                /* Record this read's Goodix quality for the burst log (min
                   score + last level) -- done for every in-range read so the
                   log reflects the true signal, gated or not. */
                uint32_t qscore = 0, qlevel = 0;
                gh3018_get_hr_quality(&qscore, &qlevel);
                if (qscore < bg_hr_burst_qscore_min) bg_hr_burst_qscore_min = qscore;
                bg_hr_burst_qlevel = qlevel;

                /* Quality gate: the algo itself flags low-confidence outputs.
                   Drop them BEFORE the median so a loose/weak/moving wrist
                   can't paint a confident-looking spike. An all-dropped burst
                   becomes BGHR_NO_LOCK (honest "weak signal" gap). */
                if (qlevel < BGHR_MIN_QLEVEL)
                {
                    bg_hr_burst_qual_rej++;
                }
                else
                {
                    /* Median-filter the accepted read: suppress isolated
                       spikes before they reach best / mean / std. Publish AND
                       accumulate the MEDIAN, not the raw value. */
                    uint8_t med = bghr_median_push((uint8_t)sd.data.hr);
                    bg_hr_burst_best = med;
                    bg_hr_burst_sum += (uint32_t)med;
                    bg_hr_burst_sum_sq += (uint32_t)med * (uint32_t)med;
                    bg_hr_burst_cnt++;
                }
            }
        }
    }
    if (bghr_now_ms >= bg_hr_burst_deadline_ms)
    {
        bg_hr_finish_burst();
    }
}

static void bg_hr_period_cb(void *param)
{
    (void)param;

    /* Roll the wall-clock 5-min bucket BEFORE any early-return below, so every
       tick is attributed and a rollover is never missed. period tick (3 min) <
       bucket (5 min) guarantees each bucket is visited at least once. While the
       RTC is unset time() stays tiny -> bucket_now small, no bogus uplink. */
    uint32_t bucket_now = (uint32_t)time(NULL) / BG_HR_BUCKET_SEC;
    if (bg_hr_bucket_id == 0)
        bg_hr_bucket_id = bucket_now;                 /* first run: no flush */
    else if (bucket_now != bg_hr_bucket_id)
    {
        bg_hr_flush_bucket(bg_hr_bucket_id * BG_HR_BUCKET_SEC);
        bg_hr_bucket_id = bucket_now;
    }

    if (bg_hr_bursting) { bg_hr_note(BGHR_BUSY); return; }

    int reason = bg_hr_skip_reason();
    if (reason != BGHR_OK) { bg_hr_note(reason); return; }

    /* Exercise app already measuring -> just forward its latest value, no
       extra LED burst. */
    if (hr_service_env.ref_count > 0)
    {
        uint8_t bpm = hr_service_get_latest_bpm();
        if (bpm > 0 && watch_sys_sync.notify_hr_sample)
        {
            watch_sys_sync.notify_hr_sample((uint32_t)time(NULL), bpm);
            bg_hr_note(BGHR_OK);
        }
        else
        {
            bg_hr_note(BGHR_FWD_ZERO);
        }
        return;
    }

    /* Awake: throttle to the ~15 min daily-curve rate (burst every 5th tick).
       Asleep: burst every tick for dense staging HR. */
    if (!bg_hr_sleep_active)
    {
        if (++bg_hr_awake_ticks < BG_HR_AWAKE_SKIP) { bg_hr_note(BGHR_THROTTLE); return; }
        bg_hr_awake_ticks = 0;
    }

    /* Start a fresh burst: power the LED, read at 1 Hz until the deadline.
       Bail BEFORE powering on if the sample timer is missing (creation failed)
       — otherwise the LED would turn on with nothing to ever finish the burst
       (stuck-on power leak + sampler wedged). */
    if (bg_hr_sample_timer == RT_NULL) { bg_hr_note(BGHR_NO_TIMER); return; }
    bg_hr_bursting = RT_TRUE;
    bg_hr_burst_best = 0;
    bg_hr_burst_sum = 0;
    bg_hr_burst_sum_sq = 0;
    bg_hr_burst_cnt = 0;
    bg_hr_burst_reads = 0;
    bg_hr_burst_readfail = 0;
    bg_hr_burst_motion_rej = 0;
    bghr_prev_accel_valid = false;   /* re-seed accel delta for this burst */
    bghr_last_motion_ms = 0;
    bghr_med_cnt = 0;                 /* reset median window for this burst */
    bghr_med_idx = 0;
    bg_hr_burst_qscore_min = 0xFFFFFFFFu;
    bg_hr_burst_qlevel = 0;
    bg_hr_burst_qual_rej = 0;
    bg_hr_burst_ms = bg_hr_sleep_active ? BG_HR_BURST_MS_SLEEP : BG_HR_BURST_MS_AWAKE;
    uint32_t bg_now_ms = rt_tick_get_millisecond();
    bg_hr_burst_accept_ms = bg_now_ms + BG_HR_WARMUP_MS; /* fixed warm-up fallback cap */
    bg_hr_burst_start_seq = gh3018_get_hr_update_seq();  /* dynamic warm-up: accept once the algo emits a NEW locked value past this */
    bg_hr_burst_deadline_ms = bg_now_ms + bg_hr_burst_ms;
    /* Open in HR mode (GH30X_FUNCTION_HR), the same path the foreground HR
       subscribe uses; hr_set_power(1) would open NORMAL = HRV, which never
       yields a BPM for gh3018_get_hr(). Power-down at burst end stays hr_set_power(0). */
    hr_control_mode(RT_SENSOR_POWER_HIGH);
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