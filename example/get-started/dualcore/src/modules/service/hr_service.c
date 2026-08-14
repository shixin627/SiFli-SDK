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
#include "hr_autocorr.h"        /* in-tree resting-HR estimator (replaces vendor) */

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
    /* Raw-data collection holds the sensor unconditionally. Power-DOWN
     * (wear-detect OFF, bg_hr burst end, suspend, charger) would freeze the
     * raw stream on the sensor's last latched value; a redundant power-ON
     * (e.g. HCPU_RESUME after standby) is just as harmful -- the driver has
     * no same-mode guard, so it deinit/reinits the running sensor and rips a
     * hole in the stream. Veto BOTH while collecting; collection itself
     * powers on before raising the flag (see set_imu_rawdata_collection). */
    extern bool imu_rawdata_collection_active(void);
    if (imu_rawdata_collection_active())
    {
        LOG_I("PPG power request (%d) vetoed: raw collection active", arg);
        return;
    }
#ifndef SOC_BF0_HCPU
    /* Continuous-HR diagnostic owns the sensor for the same reason: its whole
     * point is that the HBA algorithm is never re-initialised. wear_detect calls
     * hr_set_power(1) on off-wrist motion and power-cycles it when PPG goes
     * stale (wear_detect.c) -- either would run open_gh3018() -> module_stop() +
     * start(HRV), killing the HR function and cold-starting the algo, i.e.
     * silently converting the experiment back into the thing it is measured
     * against. Veto BOTH directions while it runs. */
    extern bool hr_service_continuous_active(void);
    if (hr_service_continuous_active())
    {
        LOG_I("PPG power request (%d) vetoed: continuous HR diag active", arg);
        return;
    }
#endif
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

/**
 * @brief Re-apply the foreground HR sensor mode once raw collection lets go.
 *
 * The subscribe handler defers hr_control_mode() while collection owns the
 * sensor (see MSG_SERVICE_SUBSCRIBE_REQ). A subscriber that arrived mid-session
 * would otherwise be left with the sensor still in raw mode and never produce a
 * BPM, so set_imu_rawdata_collection() calls this on the way out — the mirror of
 * the hr_set_power(0) it already does when the last subscriber has gone.
 */
void hr_reapply_subscriber_mode(void)
{
    if (hr_service_env.ref_count > 0)
    {
        LOG_I("HR mode re-applied after raw collection (ref_count %d)",
              hr_service_env.ref_count);
        hr_control_mode(RT_SENSOR_POWER_HIGH);
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
        memset(r->data, 0, len); // never push uninitialised heap on bad len
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
        r->len = len;
        memset(r->data, 0, len); // never push uninitialised heap on bad len
        if (len == HRS_DAY_TABLE_LEN) // get day value : 24 hour
        {
            uint8_t today[HRS_DAY_TABLE_LEN];
            int i;

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
        memset(r->data, 0, len); // never push uninitialised heap on bad len
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
        memset(r->data, 0, len); // never push uninitialised heap on bad len
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
        memset(r->data, 0, len); // never push uninitialised heap on bad len
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
            /* Raw-data collection owns the sensor. This call used to go straight
               to rt_device_control(), bypassing the veto hr_set_power() applies —
               and the ASYMMETRY was the whole bug: the switch INTO HIGH (the
               25 Hz HR function) went through, while the matching power-down at
               the end of a measurement does go through hr_set_power() and IS
               vetoed. Collection therefore never got its mode back.

               Measured on the dev watch 2026-08-05 (bloc_motion_tracking.c's
               `ppgdiag`): subscribing here mid-session moves the raw PPG FIFO
               batch period 20 ms -> 522 ms and it stays there, so the gesture
               stream's PPG column carries 2 fresh samples per 522 ms (~3.8 Hz
               instead of 100 Hz) for the rest of the collection session.

               Deferred, not dropped: set_imu_rawdata_collection() calls
               hr_reapply_subscriber_mode() when collection ends. The other two
               hr_control_mode() callers need no guard — bg_hr_period_cb already
               skips its bursts while collecting, and the continuous-HR
               diagnostic deliberately owns the sensor for its whole run. */
            extern bool imu_rawdata_collection_active(void);
            if (imu_rawdata_collection_active())
            {
                LOG_I("HR mode deferred: raw collection owns the sensor");
            }
            else
            {
                hr_control_mode(RT_SENSOR_POWER_HIGH);
            }
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
extern void gh3018_get_hr_quality(uint32_t *valid_score, uint32_t *valid_level,
                                  uint32_t *confi_x100, uint32_t *snr_x100);
/* Algorithm's own motion state / detected scene. Declared here rather than by
   including gh3018.h, matching how every other gh3018 symbol is pulled into this
   file. Without it C99 assumes int(), which happens to work on AAPCS and is
   still undefined behaviour. */
extern void gh3018_get_hr_acc_state(uint32_t *acc_info, uint32_t *acc_scene);
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
/* Sparse-but-long night cadence (2026-07-28 experiment): the residual harmonic
   locks all happen in the first seconds after a cold start, and the workout app
   (continuous measurement) tracks Apple closely on the same wrist — evidence
   that convergence time, not sensor quality, is the limit. Trade burst COUNT for
   burst LENGTH: 10 min apart, 3 min each. Night LED duty 33% -> 30%, so this is
   power-neutral while giving the algo 3x longer to converge. Sleep-staging HR
   thins to one window per 10 min; keep SLEEP_HR_WINDOW_MAX_AGE_MS
   (sleep_service.c) >= this period so no minute is left without HR features.
   Revert both together. */
#define BG_HR_PERIOD_MS      (10 * 60 * 1000) /* base tick = sleep-mode cadence   */
/* Back to the normal intermittent cadence (2026-08-04). The dense daytime
   diagnostic ran its course: it was there to cross-check against a reference
   watch without waiting a night per experiment, and it did — the doubling is now
   characterised and reproduced. Holding PPG at ~30% daytime duty for a question
   that has been answered is just battery. */
#define BG_HR_AWAKE_SKIP     2               /* awake: burst every 2nd tick = 20 min */
/* The HBA algo restarts cold each burst and needs ~30 s to lock (hba_out_flag
   stays 0 until then); before that gh3018_get_hr() still returns the PREVIOUS
   burst's stale value, so reads are gated on a real lock (HR update-seq moving)
   rather than on elapsed time -- keep every burst comfortably longer than ~30 s
   or it will lock nothing and report NO_LOCK. */
/* 40 s awake / 3 min asleep. Both are sized by hr_autocorr's 10.24 s window plus
   margin for the median below — NOT by the vendor algorithm's ~30 s cold-start
   convergence any more, since its answer is no longer the one published. The
   sleep burst stays long because that is where the artefacts live and more
   independent estimates make the median robust. */
#define BG_HR_BURST_MS_AWAKE (40 * 1000)
#define BG_HR_BURST_MS_SLEEP (3 * 60 * 1000) /* long enough to converge past a cold-start harmonic lock */
#define BG_HR_SAMPLE_MS      (1000)          /* read cadence during the burst      */
/* HR output motion gate: each 1 Hz read also samples BMI270 accel; if the wrist
   moved this second (or within the guard window after), drop that HR read from
   both the published best and the mean/std window. Suppresses the PPG motion-
   artefact spikes the GH30x built-in comp lets through (its accel feed is time-
   warped ~6x -- see plan). HR-only: does not touch PPG raw or the algo feed. */
#define BG_HR_MOTION_DELTA_THRESH  6     /* >>10 LSB delta; ~0.4g between 1 Hz reads */
#define BG_HR_MOTION_GUARD_MS      3000  /* keep rejecting this long after motion    */
/* Sleep-time physiologic ceiling for the published HR. A sleeping wrist sits at
   ~40-70 bpm; even REM/arousal rarely passes ~90. A burst reading well above this
   WHILE ASLEEP is a PPG artefact (poor contact/perfusion reads high), not a real
   pulse — Apple's watch on the same wrist capped ~100-103 while ours read 128. We
   WITHHOLD such bursts (draw a gap, not a false spike) so the curve + sleep
   staging stay clean. Heuristic stopgap: a proper signal-quality (PI) gate is
   future work (needs an on-wrist PI-distribution capture to tune, unavailable
   now). Applied ONLY while asleep, so it can never clip a real daytime HR. */
#define SLEEP_HR_ARTEFACT_CEIL     100u

static rt_timer_t bg_hr_period_timer = RT_NULL;
static rt_timer_t bg_hr_sample_timer = RT_NULL;
static rt_bool_t bg_hr_bursting = RT_FALSE;
/* RETRY-UNTIL-MEASURED (awake only).
 *
 * A burst that produces nothing -- PPG never locked, or the curve gate refused
 * the value -- leaves a hole in the daily curve, and at the normal awake cadence
 * that hole is 40 minutes wide (bursts 20 min apart). Consecutive failures stack:
 * measured on 2026-08-14, runs of 2-4 NO_LOCK bursts produced 30, 40, 40 and 50
 * minute holes. A curve that blank for that long reads as broken whatever the
 * reason field says.
 *
 * So while awake and on the wrist, a failed burst switches the period timer to
 * BG_HR_RETRY_MS and keeps measuring until one succeeds -- worst case above
 * becomes ~16 min instead of 50. The cost is LED duty (a 40 s burst every 4 min
 * = 17%) and it is paid ONLY during a failing run on a worn wrist, which is
 * exactly the case where the user is looking at a blank curve.
 *
 * Asleep is excluded: bursts there are 3 minutes long and already fire every
 * tick, so a 4-minute period would run the LED near-continuously all night for
 * no gain. */
#define BG_HR_RETRY_MS       (4 * 60 * 1000)
static rt_bool_t bg_hr_retry_soon = RT_FALSE;   /* last burst produced nothing  */
static rt_bool_t bg_hr_period_short = RT_FALSE; /* timer is at BG_HR_RETRY_MS   */
static void bg_hr_set_period(rt_bool_t shortened);   /* defined with the timer  */
static uint32_t bg_hr_burst_deadline_ms = 0;
static uint32_t bg_hr_burst_start_seq = 0;   /* gh3018 HR-update seq at burst start; warm-up baseline */
/* PPG frame accounting for the 2x-harmonic investigation. The algo is told
   fs=25 Hz and the samples carry no timestamps, so silently lost frames (chip
   FIFO overflow when its interrupt is serviced late) make the pulse look faster
   than it is — lose half and the reported HR doubles. Snapshot the driver's
   monotonic frame counter at burst start, diff at burst end, and express it as a
   PERCENTAGE of the frames a full-rate burst should have delivered
   (25 Hz * burst seconds). ~100% => the timebase is sound and the 2x episodes
   have some other cause; ~50% => the doubling is a lost-frame artefact and the
   real fix is upstream, not a post-hoc threshold. */
extern uint32_t gh3018_get_ppg_frame_count(void);
static uint32_t bg_hr_burst_start_frames = 0;
static uint16_t bg_hr_win_frame_pct = 0;     /* last burst's delivered/expected, % */

/* The decimation stage — where a x2 rate error can actually be born, and which
   bg_hr_win_frame_pct above is blind to because it counts UPSTREAM of it.
   The chip runs at 25 Hz for HR but 100 Hz for HRV/SpO2, so the driver derives
   divider = chip_rate / 25 from an I2C register read at each sampling start and
   feeds the algorithm every Nth frame, always telling it fs = 25. Land on
   divider 2 while the chip is really at 25 Hz and the algorithm sees 12.5 Hz
   believing 25 -> it reports exactly DOUBLE for the whole burst and recovers at
   the next recompute: a stable ~45 min plateau with a clean x2 entry and /2 exit,
   which is precisely the nightly signature.
   Packed as (divider << 8) | algo-frames-as-%-of-25Hz*seconds, capped at 255, so
   one field answers both "was the divider 1?" and "did the algo get 25 Hz worth
   of frames?". Healthy = 0x0163-ish (divider 1, ~99%); divider 2 = the bug. */
extern uint8_t gh3018_get_hr_divider(void);
extern uint32_t gh3018_get_hr_algo_frame_count(void);
static uint16_t bg_hr_win_rate_info = 0;
static uint8_t bg_hr_burst_best = 0;

/* ---- PPG perfusion-index (PI = AC/DC) capture over a burst -------------------
   Fed from the raw PPG frame hook (process_ppg_sensor_data -> ppg_pi_feed), gated
   by bg_hr's burst window. A real pulse has a clear AC swing (high PI); a poor-
   contact / artefact read is flat or noisy (low PI) — the candidate signal-quality
   metric that tells a real sleeping HR from an artefact at the SAME bpm. Captured
   into sleep_diag for now (to learn its threshold on a real night); a future gate
   uses it. Single writer = LCPU frame thread; read once at burst end (same core). */
static volatile bool s_pi_collecting = false;
static uint32_t s_pi_min = 0, s_pi_max = 0, s_pi_sum = 0, s_pi_cnt = 0;
static uint16_t bg_hr_win_pi_e3 = 0;   /* last burst's PI*1000, clamped to u16 */

/* Repeat-sample detection over the same raw stream (see ppg_pi_feed). */
static uint32_t s_rep_prev = 0, s_rep_run = 0, s_rep_run_max = 0;
static uint32_t s_rep_equal = 0, s_rep_total = 0;
static bool     s_rep_have_prev = false;
static uint8_t  bg_hr_win_rep_max = 0;   /* longest identical run this burst   */
static uint8_t  bg_hr_win_rep_pct = 0;   /* % of samples equal to predecessor  */
/* Declared here rather than with the other burst counters below because the
   sleep_diag getters sit above them and would otherwise forward-reference it. */
static uint8_t  bg_hr_burst_own_conf_max = 0;  /* best hr_autocorr confidence this burst */

static void ppg_pi_start(void)
{
    s_pi_min = 0xFFFFFFFFu; s_pi_max = 0; s_pi_sum = 0; s_pi_cnt = 0;
    s_rep_run = 0; s_rep_run_max = 0; s_rep_equal = 0; s_rep_total = 0;
    s_rep_have_prev = false;   /* first sample of a burst has no predecessor */
    s_pi_collecting = true;
    /* Drop the correlation window with the PI window: samples from before this
       burst's LED power-up would be correlated against fresh ones across a
       discontinuity that is not a heartbeat. */
    hr_autocorr_reset();
}

void ppg_pi_feed(uint8_t n, const uint32_t *raw)
{
    if (!s_pi_collecting || raw == NULL) return;
    /* hr_autocorr is NOT fed here any more: it needs the accelerometer aligned
       to each PPG frame, and only the vendor's frame hook has that (see
       Gh30xFrameDataHookFunc). This path keeps the two diagnostics — PI and the
       repeat detector — which need the raw stream but not the alignment. */
    for (uint8_t i = 0; i < n; i++)
    {
        uint32_t v = raw[i];

        /* Repeat detection. The gesture-collection path samples PPG by polling a
           two-slot buffer at the IMU rate, so when that rate outruns the PPG
           update it re-reads the same pair and the waveform comes out as a
           staircase (founder observed exactly this, recovering only after a
           sleep/wake). THIS path takes the FIFO's actual contents instead, so it
           should never repeat — but that is a claim from reading code, and this
           investigation has had several code-reading conclusions turn out wrong.
           A 17-bit ADC on live tissue does not produce two identical consecutive
           samples by chance, so any run > 1 here is real evidence. */
        if (s_rep_have_prev && v == s_rep_prev)
        {
            s_rep_run++;
            s_rep_equal++;
            if (s_rep_run > s_rep_run_max) s_rep_run_max = s_rep_run;
        }
        else
        {
            s_rep_run = 1;
            if (s_rep_run > s_rep_run_max) s_rep_run_max = s_rep_run;
        }
        s_rep_prev = v;
        s_rep_have_prev = true;
        s_rep_total++;

        if (v < s_pi_min) s_pi_min = v;
        if (v > s_pi_max) s_pi_max = v;
        s_pi_sum += v;
        s_pi_cnt++;
    }
}

static void ppg_pi_finish(void)
{
    s_pi_collecting = false;

    bg_hr_win_rep_max = (s_rep_run_max > 255u) ? 255u : (uint8_t)s_rep_run_max;
    bg_hr_win_rep_pct = (s_rep_total > 0)
                            ? (uint8_t)((s_rep_equal * 100u) / s_rep_total) : 0u;

    if (s_pi_cnt == 0 || s_pi_max <= s_pi_min) { bg_hr_win_pi_e3 = 0; return; }
    uint32_t mean = s_pi_sum / s_pi_cnt;
    if (mean == 0) { bg_hr_win_pi_e3 = 0; return; }
    uint32_t pi_e3 = (uint32_t)((uint64_t)(s_pi_max - s_pi_min) * 1000u / mean);
    bg_hr_win_pi_e3 = (pi_e3 > 0xFFFFu) ? 0xFFFFu : (uint16_t)pi_e3;
}

/* Last completed burst's PI*1000 (0 if none). Read by sleep_service for sleep_diag. */
uint16_t hr_service_get_last_pi_e3(void) { return bg_hr_win_pi_e3; }

/**
 * Last burst's diagnostics packed for sleep_diag, high byte first:
 *   [15:8] hr_autocorr's best confidence   — the number missing on 2026-08-05,
 *          when four isolated outliers (171/144/101/38) could not be classified
 *          as "low confidence, raise the gate" vs "confident and wrong, fix the
 *          rule" because confidence only ever reached the LCPU console.
 *   [7:0]  longest run of identical raw PPG samples — proof, not inference, that
 *          the staircase seen in the gesture stream is absent here.
 * rep_pct rides separately (see below) so neither field has to be truncated.
 */
uint16_t hr_service_get_last_own_info(void)
{
    return (uint16_t)(((uint16_t)bg_hr_burst_own_conf_max << 8) | bg_hr_win_rep_max);
}

/** Last burst's share of samples identical to their predecessor, percent. */
uint8_t hr_service_get_last_rep_pct(void) { return bg_hr_win_rep_pct; }

/* Last completed burst's delivered-vs-expected PPG frame percentage. The night
   has no serial console, so this rides sleep_diag to the phone: it is the test
   that separates "the 2x episodes are a lost-frame timebase artefact" (~50%)
   from "the timebase is fine, look at the waveform instead" (~100%). */
uint16_t hr_service_get_last_frame_pct(void) { return bg_hr_win_frame_pct; }

/* (divider << 8) | algo-frames-% — the decimation-stage view. See the declaration
   of bg_hr_win_rate_info: divider != 1 while the chip is at 25 Hz is the x2 bug. */
uint16_t hr_service_get_last_rate_info(void) { return bg_hr_win_rate_info; }

/* Two-stage gate state. bg_hr_sleep_active is set by sleep_service when accel
   says we're asleep OR the wrist is still inside the overnight rest window
   (rest-candidate) -> dense bursts; cleared otherwise -> ~15 min curve rate. */
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


/* ---- curve-level plausibility gate ---------------------------------------
 *
 * The last defence, and the only one that uses information the estimator cannot
 * see. Everything inside a single 10 s window has been tried and measured on the
 * 44 daytime windows captured 2026-08-14, and none of it separates a good
 * reading from a bad one: peak height, confidence and spectral sharpness all
 * overlap almost completely (good conf 37-69, bad conf 36-60), tightening the
 * spectral gate missed the two worst values outright, and the error rate does
 * not even rise monotonically with wrist activity (0% in the 120-199 band, 57%
 * above 200).
 *
 * What DOES separate them is time. Every daytime failure in that day was an
 * ISOLATED point sitting between plausible neighbours — 42 between 76 and 90,
 * 30 between 78 and 52, 196 between 90 and 84. A heart does not halve and come
 * back inside ten minutes.
 *
 * REFUSES, never substitutes. A gap in the curve is an honest "I could not
 * measure this"; an invented number is not, and this project has already
 * shipped one filter that invented values (the octave tracker pinned a genuine
 * 120 to 60 indefinitely).
 *
 * Causal, because the watch is where the value is born. Doing this on the phone
 * would leave the watch's own face and every other consumer showing the value
 * this rejects. The cost of having no future neighbour is one point: measured
 * over the confirmed-worn window of 2026-08-14, the acausal rule keeps 33 of 38
 * and this keeps 32, both with zero surviving outliers, and both preserve the
 * genuine 134 bpm excursion at 16:03.
 *
 * A deviation is refused ONCE. If the next burst confirms the same new level,
 * it is accepted and the history moves there — so a real change (standing up,
 * exercise) costs a single point rather than being suppressed for as long as it
 * lasts. */
#define CURVE_HIST_N       5
#define CURVE_LO_PCT      65        /* below this share of the median: refuse   */
#define CURVE_HI_PCT     160        /* above this share of the median: refuse   */
#define CURVE_MAX_AGE_S 5400        /* history older than 90 min is not context */
#define CURVE_CONFIRM_LO  85        /* the next reading confirms the new level  */
#define CURVE_CONFIRM_HI 118        /* if it lands within this of the refused one */

static uint8_t  s_curve_v[CURVE_HIST_N];
static uint32_t s_curve_t[CURVE_HIST_N];
static uint8_t  s_curve_n;
static uint8_t  s_curve_pending;    /* value refused last time, 0 = none        */

static void curve_push(uint32_t now_s, uint8_t bpm)
{
    if (s_curve_n < CURVE_HIST_N)
    {
        s_curve_v[s_curve_n] = bpm; s_curve_t[s_curve_n] = now_s; s_curve_n++;
    }
    else
    {
        for (uint8_t i = 1; i < CURVE_HIST_N; i++)
        { s_curve_v[i-1] = s_curve_v[i]; s_curve_t[i-1] = s_curve_t[i]; }
        s_curve_v[CURVE_HIST_N-1] = bpm; s_curve_t[CURVE_HIST_N-1] = now_s;
    }
}

static bool curve_accept(uint32_t now_s, uint8_t bpm)
{
    uint8_t live[CURVE_HIST_N]; uint8_t n = 0;
    for (uint8_t i = 0; i < s_curve_n; i++)
        if (now_s - s_curve_t[i] <= CURVE_MAX_AGE_S) live[n++] = s_curve_v[i];

    bool accept = true;
    if (n >= 2)
    {
        for (uint8_t i = 1; i < n; i++)          /* insertion sort, n <= 5 */
        {
            uint8_t v = live[i]; int j = i - 1;
            while (j >= 0 && live[j] > v) { live[j + 1] = live[j]; j--; }
            live[j + 1] = v;
        }
        uint32_t med = live[n / 2];
        uint32_t lo = med * CURVE_LO_PCT, hi = med * CURVE_HI_PCT;
        uint32_t x  = (uint32_t)bpm * 100;
        if (x <= lo || x >= hi)
        {
            /* off the baseline — unless the previous burst was refused at the
               same level, in which case this is a real change, not a slip */
            if (s_curve_pending > 0 &&
                (uint32_t)bpm * 100 >= (uint32_t)s_curve_pending * CURVE_CONFIRM_LO &&
                (uint32_t)bpm * 100 <= (uint32_t)s_curve_pending * CURVE_CONFIRM_HI)
            {
                s_curve_n = 0;                   /* re-establish, do not fight */
            }
            else accept = false;
        }
    }

    if (!accept) { s_curve_pending = bpm; return false; }
    s_curve_pending = 0;
    curve_push(now_s, bpm);
    return true;
}

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
   future quality gate. */
static uint32_t bg_hr_burst_qscore_min = 0xFFFFFFFFu;
static uint32_t bg_hr_burst_qlevel = 0;
/* MAX of each quality field this burst. qmin alone cannot answer "does the lib emit
   confidence at all" -- it starts at UINT32_MAX and one zero-scoring read pins it to 0
   forever, which is exactly the reading ADR 0016 drew "the lib does not implement
   confidence" from. The max is the field that distinguishes "always 0" from "sometimes
   0". Paired with the back_track_len 0 -> 30 change in gh30x_demo_algo_call_hr.c. */
static uint32_t bg_hr_burst_qscore_max = 0;
static uint32_t bg_hr_burst_confi_max = 0;
static uint32_t bg_hr_burst_snr_max = 0;
/* Minimum hr_autocorr confidence to accept a reading. 40 is deliberately loose
   for a first night: the synthetic suite clears 96+ on every real case and pure
   noise scores 0, so the gap is wide and the risk here is over-rejecting a real
   but weak signal, not letting noise through. Tighten once a night's
   own_conf distribution against a reference watch says where the line is. */
#define BG_HR_OWN_MIN_CONF   40
static uint8_t  bg_hr_vendor_bpm = 0;          /* last vendor read, diagnostic only */

/* Window capture for the offline suite. Triggered when an estimate lands far
   from the last PUBLISHED value — that is the shape of all four 2026-08-05
   outliers (171/144/101/38), each a single burst between correct neighbours.
   Bounds chosen wide: this is evidence collection, and a capture that never
   fires is worse than one that occasionally grabs a real rate change. */
#define BGHR_SUSPECT_HI_NUM   15   /* >= 1.5x the last published value */
#define BGHR_SUSPECT_LO_NUM    7   /* <= 0.7x                          */
#define BGHR_SUSPECT_DEN      10
static uint8_t  bg_hr_last_published = 0;
static bool     bg_hr_win_captured = false;   /* one capture per burst */
/* Set when the held capture is a SUSPECT window; such a capture is never
   overwritten by the routine end-of-burst one. See the capture block. */
static bool     bg_hr_win_suspect = false;
static watch_sys_hr_window_t bg_hr_win_dump;

/* Integer std-dev from running Σx / Σx². Dividing before squaring keeps this in
   uint32 for any realistic n (Σx² ≤ n·255², so n up to ~66k is safe; a 3-min
   burst is ~180 reads). Mirrors sleep_service's prv_compute_hr_std so the HRV
   feed and the classifier agree on the math. */
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

/* Goodix HBA sleep-context plumbing. Root cause of the jagged night HR
   (49->92->52 three-minute jumps, bimodal histogram with an 80-99 lobe at ~2x
   the true resting rate): every 3-min burst cold-starts the closed HBA lib in
   scene DEFAULT / mode DYNAMIC with sleep_flg=0 — the algo is never told it is
   doing periodic sampling on a sleeping, low-perfusion wrist, and locks the
   2nd harmonic instead of the fundamental. The vendor ships exactly these
   knobs for this use case (goodix_hba.h: HBA_SCENES_SLEEP, HBA_TEST_SENSELESS,
   input sleep_flg); wire them to the same edge that drives burst density.
   Setters live in gh30x_example_hook.c (LCPU-linked driver; scenario/mode need
   __HBD_ALGORITHM_EXTERNANL_CONFIG_ENABLE__=1 in gh30x_example_config.h).
   Values mirror the vendor enums — keep in sync with goodix_hba.h. */
#define GH_HBA_SCENE_DEFAULT   0   /* HBA_SCENES_DEFAULT: algo self-detects   */
#define GH_HBA_SCENE_SLEEP     20  /* HBA_SCENES_SLEEP                        */
#define GH_HBA_MODE_DYNAMIC    0   /* HBA_TEST_DYNAMIC: continuous default    */
#define GH_HBA_MODE_SENSELESS  2   /* HBA_TEST_SENSELESS: periodic background
                                      sampling (= bg_hr's burst cadence)        */
/* Vendor units are SECONDS ("无感间隔时间秒数" / "无感持续时间秒数", 0 = unknown --
   goodix_hba.h:105-106, GBK). We passed 0/0 = "unknown" since the mode was first
   wired, i.e. the algo was told it does periodic sampling but never told the
   period -- so it had no basis for carrying tracking state across the gap and
   cold-started blind every burst. Derive from the actual cadence so the two can
   never drift apart. */
#define GH_HBA_SENSELESS_STEP_S (BG_HR_PERIOD_MS / 1000)      /* gap between bursts */
#define GH_HBA_SENSELESS_DUR_S  (BG_HR_BURST_MS_SLEEP / 1000) /* length of a burst  */
extern signed char HBD_HbAlgoScenarioConfig(int scenario);
extern void HBD_HbaTestModeConfig(int mode, unsigned short step,
                                  unsigned short duration);
extern void HBD_HbaSleepFlagConfig(unsigned char sleep_flg);

/* Dense-burst gate. sleep_service asserts this while accel-detected sleep is
   active OR the wrist is still inside the overnight rest window
   (verdict-independent — see sleep_service.c SLEEP_REST_*; keeps a mis-scored
   night from starving itself of the dense HR the wake-veto needs to
   self-correct). Cleared -> back to the ~15 min curve rate. */
void hr_service_set_sleep_active(bool active)
{
    bg_hr_sleep_active = active ? RT_TRUE : RT_FALSE;
    if (!active) bg_hr_awake_ticks = 0; /* re-arm the awake skip cadence */

    /* Latch the Goodix sleep context on EDGES only (called every minute-eval).
       scene/mode are read at each burst's algo init, so the next burst picks
       them up; sleep_flg is read every frame. Boot default on both sides is
       the awake trio, so no init-order dependency. */
    static rt_bool_t s_prev_hba_sleep = RT_FALSE;
    if (bg_hr_sleep_active != s_prev_hba_sleep)
    {
        s_prev_hba_sleep = bg_hr_sleep_active;
        HBD_HbaSleepFlagConfig(active ? 1 : 0);
        HBD_HbAlgoScenarioConfig(active ? GH_HBA_SCENE_SLEEP
                                        : GH_HBA_SCENE_DEFAULT);
        HBD_HbaTestModeConfig(active ? GH_HBA_MODE_SENSELESS
                                     : GH_HBA_MODE_DYNAMIC,
                              active ? GH_HBA_SENSELESS_STEP_S : 0,
                              active ? GH_HBA_SENSELESS_DUR_S  : 0);
        LOG_I("bg_hr: HBA sleep context -> %d", active ? 1 : 0);
    }
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
#if kReleaseMode
    /* Release: on the charger means off-wrist, so skip PPG entirely.
       Dev builds deliberately keep sampling while charging so wear detection
       stays live for bench use. Dev and release now share the same physical
       board (VER_29), so this is keyed on kReleaseMode rather than the board
       version. NOTE: charging current can inject noise into the optical ADC,
       so DC/PI read while charging may be less reliable -- accepted dev-only
       trade-off. */
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
    /* Frame accounting: did the chip actually deliver a full-rate stream this
       burst? The algo assumes 25 Hz unconditionally, so a shortfall here IS a
       proportional over-read of HR (see bg_hr_win_frame_pct decl). */
    {
        uint32_t want = (bg_hr_burst_ms / 1000u) * 25u;
        uint32_t got = gh3018_get_ppg_frame_count() - bg_hr_burst_start_frames;
        uint32_t pct = (want > 0) ? (got * 100u / want) : 0u;
        bg_hr_win_frame_pct = (pct > 0xFFFFu) ? 0xFFFFu : (uint16_t)pct;

        /* Post-decimation: what the ALGORITHM actually received, plus the divider
           that produced it (see bg_hr_win_rate_info decl for why this is the
           measurement that can catch the doubling). */
        uint32_t algo = gh3018_get_hr_algo_frame_count();
        uint32_t apct = (want > 0) ? (algo * 100u / want) : 0u;
        if (apct > 255u) apct = 255u;
        bg_hr_win_rate_info = (uint16_t)(((uint16_t)gh3018_get_hr_divider() << 8) |
                                         (uint16_t)apct);
    }

    /* own=  what we publish (hr_autocorr)   vendor= the Goodix answer, kept only
       so the two can be read off against each other on the same line. */
    LOG_I("bg_hr burst: reads=%u acc=%u motion_rej=%u own=%u ownconf=%u vendor=%u fill=%u qmin=%u qmax=%u qlvl=%u confiX100=%u snrX100=%u frames=%u%% div=%u algo=%u%%",
          (unsigned)bg_hr_burst_reads, (unsigned)bg_hr_burst_cnt,
          (unsigned)bg_hr_burst_motion_rej, (unsigned)bg_hr_burst_best,
          (unsigned)bg_hr_burst_own_conf_max, (unsigned)bg_hr_vendor_bpm,
          (unsigned)hr_autocorr_fill(),
          (unsigned)bg_hr_burst_qscore_min, (unsigned)bg_hr_burst_qscore_max,
          (unsigned)bg_hr_burst_qlevel,
          (unsigned)bg_hr_burst_confi_max, (unsigned)bg_hr_burst_snr_max,
          (unsigned)bg_hr_win_frame_pct,
          (unsigned)(bg_hr_win_rate_info >> 8), (unsigned)(bg_hr_win_rate_info & 0xFF));
    /* Sleep-time artefact ceiling: withhold an implausibly high sleeping HR
       (SLEEP_HR_ARTEFACT_CEIL). Zeroing best routes it to the NO_LOCK branch below
       so the phone draws a gap instead of a false spike, and the sleep_fusion HR
       window (published from best) goes absent for that minute rather than feeding
       the wake-veto a garbage-high value. Only while asleep. */
    /* 2026-07-29: this used to ZERO the value, which destroyed the only evidence of
       what the algo had actually locked -- such bursts became indistinguishable from
       a genuine no-lock, and one night put 47% of all bursts in that bucket with no
       way to tell which. x2 of a 50-55 resting HR is 100-110, i.e. just above this
       ceiling, so these are the prime harmonic suspects and we need to SEE them.
       Now: publish the raw value (it is self-identifying -- a >100 sleeping reading
       IS the marker) but keep it out of sleep staging, which is the protection that
       actually mattered. Diagnostic trade-off: the phone curve shows these spikes
       again for now, same as before the ceiling existed. */
    bool bghr_over_ceiling = (bg_hr_sleep_active &&
                              bg_hr_burst_best > SLEEP_HR_ARTEFACT_CEIL);

    /* Forward the best BPM seen this burst, then power the LED back off. */
    /* Ship the captured window before the HR point, so a capture is never lost to
       a disconnect that happens between the two. */
    if (bg_hr_win_captured && watch_sys_sync.notify_hr_window)
    {
        watch_sys_sync.notify_hr_window(&bg_hr_win_dump);
    }

    bool bghr_curve_refused = false;
    bool bghr_published = false;
    if (bg_hr_burst_best > 0 && watch_sys_sync.notify_hr_sample)
    {
        uint32_t now_s = (uint32_t)time(NULL);
        if (curve_accept(now_s, bg_hr_burst_best))
        {
            watch_sys_sync.notify_hr_sample(now_s, bg_hr_burst_best);
            bg_hr_last_published = bg_hr_burst_best;  /* baseline for next burst */
            bghr_published = true;
            bg_hr_note(BGHR_OK);
        }
        else
        {
            /* Publish nothing. @ref curve_accept — a gap says "not measured",
               which is true; a number here would not be. */
            bghr_curve_refused = true;
            LOG_I("[BGHR] curve gate refused %u", (unsigned)bg_hr_burst_best);
        }
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

    /* @ref BG_HR_RETRY_MS — a burst that put nothing on the curve keeps the
       tick short and the throttle bypassed until one does. Every exit above
       that is not a publish counts, including the curve-gate refusal: the user
       cannot tell "refused" from "never locked" by looking at the graph, and
       both are a blank stretch that has to close. */
    bg_hr_retry_soon = bghr_published ? RT_FALSE : RT_TRUE;
    if (bghr_published)          bg_hr_set_period(RT_FALSE);
    else if (!bg_hr_sleep_active) bg_hr_set_period(RT_TRUE);

    /* Publish the burst's HR window for sleep_fusion (wake-veto + staging).
       The headline value is the burst's FINAL rolling median — the converged,
       outlier-rejected read, identical to what the phone curve stores — NOT
       the mean over the whole burst: the mean folds in the pre-convergence
       warm-up prefix (dynamic warm-up accepts reads as soon as the algo seq
       moves), which on poor-contact bursts ran tens of bpm high and fed the
       wake-veto a systematically dirtier stream than the phone ever showed.
       std stays sum-based (spread of the whole burst) — it only steers
       Deep/REM staging. Need ≥2 samples for a meaningful std. */
    if (bg_hr_burst_cnt >= 2)
    {
        /* An over-ceiling burst still publishes to the curve (see above) but must
           never reach staging / the wake-veto: mean 0 makes sleep_service skip this
           window entirely, exactly as the old zeroing did. A curve-gate refusal
           takes the same exit for the same reason -- having just declared the
           value untrustworthy for the curve, feeding it to the wake-veto (where a
           spurious 196 reads as "awake") would only move the lie downstream. */
        bg_hr_win_mean = (bghr_over_ceiling || bghr_curve_refused) ? 0 : bg_hr_burst_best;
        bg_hr_win_std = bg_hr_std_from_sums(bg_hr_burst_sum, bg_hr_burst_sum_sq,
                                            bg_hr_burst_cnt);
        bg_hr_win_tick_ms = rt_tick_get_millisecond();
    }
    if (bg_hr_sample_timer) rt_timer_stop(bg_hr_sample_timer);
    {
        extern int bmi270_set_hr_accel_stream(int en);
        (void)bmi270_set_hr_accel_stream(0);   /* pairs with bg_hr_start_burst */
    }
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
    ppg_pi_finish();   /* finalize this burst's PI for sleep_diag capture */
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

    /* The vendor answer is still read, but ONLY as a diagnostic to sit beside
       ours in the CSV. It stopped being the published value on 2026-08-04: on a
       worn watch it produced a 34-minute plateau at 107-124 while a reference
       watch never passed 84 and the accelerometer read zero, and all six of its
       quality/scene fields read 0 all night, so there is no way to know when to
       distrust it. See hr_autocorr.h. */
    bg_hr_vendor_bpm = (rd == 1 && sd.data.hr > 0 && sd.data.hr < 240)
                           ? (uint8_t)sd.data.hr : 0;

    /* OUR estimate is the reading. It refuses (returns 0) until the 10.24 s
       window is full and a correlation peak clears threshold — that refusal is
       the point, and a burst where it never clears must report NO_LOCK rather
       than fall back to the vendor, or the comparison is meaningless. */
    uint8_t own_conf = 0;
    uint8_t own_bpm = hr_autocorr_estimate(&own_conf);
    if (own_conf > bg_hr_burst_own_conf_max) bg_hr_burst_own_conf_max = own_conf;

    /* accel_pp is the liveness proof for the motion-compensation reference, and
       it is logged unconditionally rather than only when something looks wrong.
       A dead accel feed reads exactly like a motionless wrist — both are a flat
       reference — so the NLMS stage cannot tell them apart and will report
       nothing either way. That silence cost a full night of wrist data: the
       reference had been three constant zeros since the feature shipped. A
       number that is visibly zero at 3 a.m. and visibly large while walking is
       the cheapest thing that would have caught it on day one. */
    extern uint32_t hr_autocorr_accel_act(void);
    LOG_I("[BGHR] own=%u conf=%u accel_act=%u vendor=%u",
          (unsigned)own_bpm, (unsigned)own_conf,
          (unsigned)hr_autocorr_accel_act(), (unsigned)bg_hr_vendor_bpm);

    /* Capture BEFORE the median swallows it: the median is what makes the
       published curve robust, but it also hides the individual bad window that
       the offline suite needs. Snapshot must happen right after estimate() while
       s_work still holds that window. */
    /* Two reasons to keep a window, in priority order:

        1. SUSPECT — its estimate deviates >=1.5x / <=0.7x from the last
           published value. What this capture was built for; never overwritten.
        2. Otherwise the most RECENT ACCEPTED window, so that EVERY periodic HR
           record ships the PPG its published value came out of, not just the
           anomalous ones (founder request 2026-08-05: be able to go back and
           look at the data whenever an HR reading is doubted).

       Caveat worth knowing when reading the dumps: the published number is
       bghr_median_push()'s running median over the burst's windows, so no
       single window IS it. The last accepted window is the closest honest
       answer — it is the window at which the published median was fixed.

       Still exactly one dump per burst either way: ~256 B every BG_HR_PERIOD_MS
       (10 min) = ~37 KB/day. hr_autocorr_last_window() is a pure read of s_work
       (copy + shift + clamp, no side effects), so calling it once per 1 Hz tick
       instead of once per burst costs nothing worth counting. */
    if (own_bpm > 0 && !bg_hr_win_suspect)
    {
        bool suspect = false;
        if (bg_hr_last_published > 0)
        {
            uint32_t hi = (uint32_t)bg_hr_last_published * BGHR_SUSPECT_HI_NUM;
            uint32_t lo = (uint32_t)bg_hr_last_published * BGHR_SUSPECT_LO_NUM;
            uint32_t cur = (uint32_t)own_bpm * BGHR_SUSPECT_DEN;
            suspect = (cur >= hi || cur <= lo);
        }
        /* The routine capture tracks the windows the burst actually counts —
           the same BG_HR_OWN_MIN_CONF gate the median push below applies — so
           the dump matches a window that fed the published value. A suspect
           window is kept regardless of confidence: a confidently-wrong reading
           and a low-confidence one are both what the offline suite wants. */
        if (suspect || own_conf >= BG_HR_OWN_MIN_CONF)
        {
            uint16_t n = hr_autocorr_last_window(bg_hr_win_dump.win,
                                                 WATCH_SYS_HR_WIN_MAX);
            if (n > 0)
            {
                bg_hr_win_dump.ts = (uint32_t)time(NULL);
                bg_hr_win_dump.bpm = own_bpm;
                bg_hr_win_dump.conf = own_conf;
                bg_hr_win_dump.count = n;
                bg_hr_win_captured = true;
                bg_hr_win_suspect = suspect;
                if (suspect)
                {
                    LOG_I("bg_hr: captured suspect window bpm=%u conf=%u (last published %u)",
                          (unsigned)own_bpm, (unsigned)own_conf,
                          (unsigned)bg_hr_last_published);
                }
            }
        }
    }

    if (own_bpm > 0 && own_conf >= BG_HR_OWN_MIN_CONF)
    {
        /* Motion is counted, not rejected: the accel-feed alignment fix restored
           the GH30x motion compensation, and over-rejecting while lying still
           starved the median window and made the curve jumpy. */
        if (bghr_motion) bg_hr_burst_motion_rej++;

        uint32_t qscore = 0, qlevel = 0, qconfi = 0, qsnr = 0;
        gh3018_get_hr_quality(&qscore, &qlevel, &qconfi, &qsnr);
        if (qscore < bg_hr_burst_qscore_min) bg_hr_burst_qscore_min = qscore;
        if (qscore > bg_hr_burst_qscore_max) bg_hr_burst_qscore_max = qscore;
        if (qconfi > bg_hr_burst_confi_max) bg_hr_burst_confi_max = qconfi;
        if (qsnr > bg_hr_burst_snr_max) bg_hr_burst_snr_max = qsnr;
        bg_hr_burst_qlevel = qlevel;

        /* Median over the burst as before — the estimator is independent per
           tick only to the extent its 10 s windows overlap, so an isolated bad
           window still gets outvoted. */
        uint8_t med = bghr_median_push(own_bpm);
        bg_hr_burst_best = med;
        bg_hr_burst_sum += (uint32_t)med;
        bg_hr_burst_sum_sq += (uint32_t)med * (uint32_t)med;
        bg_hr_burst_cnt++;
    }
    if (bghr_now_ms >= bg_hr_burst_deadline_ms)
    {
        bg_hr_finish_burst();
    }
}

/* ===== Continuous-HR diagnostic ====================================
   Founder experiment (2026-08-02). The Exercise app and bg_hr open the IDENTICAL
   sensor mode -- both go hr_control_mode(RT_SENSOR_POWER_HIGH) ->
   open_gh3018_high_power() -> module_start(GH30X_FUNCTION_HR) at 25 Hz -- and both
   read at 1 Hz. They differ in exactly one thing: the Exercise app holds the
   subscription, so the HBA algorithm runs uninterrupted, while bg_hr powers down
   after 3 min and cold-starts the algorithm again 10 min later.

   That single variable is worth isolating, because the nightly 2x has survived
   every other explanation (frames arrive at wall-clock 25 Hz, divider correct,
   algo fed 99% of them, no motion -- and the founder's own observation is that
   the Exercise app's live HR tracks a reference watch closely). If the doubling
   is absent here it is a COLD-START acquisition failure, which is fixable by
   changing the sampling regime rather than by filtering the output afterwards.

   Records the RAW per-second algorithm output -- no median, no motion gate --
   because the question is what the algorithm actually emits, not what a filter
   would have shown. Buffered a minute at a time (see watch_sys_hr_cont_t). */
/* PI window length in continuous mode — see the tick guard in bg_hr_cont_cb. */
#define BG_HR_CONT_PI_SECS   8
static rt_bool_t bg_hr_cont_enabled = RT_FALSE;
static rt_timer_t bg_hr_cont_timer = RT_NULL;
static watch_sys_hr_cont_t bg_hr_cont_buf;
/* Rolling window feeding sleep_fusion in place of the suppressed burst summary. */
static uint32_t bg_hr_cont_sum = 0, bg_hr_cont_sum_sq = 0, bg_hr_cont_n = 0;

bool hr_service_continuous_active(void)
{
    return bg_hr_cont_enabled ? true : false;
}

static void bg_hr_cont_flush(void)
{
    /* Hand the window to HCPU and move on. Whether BLE can carry it right now is
       not knowable here, so the outage retry lives on HCPU (watch_system_client.c
       keeps a backlog ring). */
    if (bg_hr_cont_buf.count == 0) return;
    if (watch_sys_sync.notify_hr_cont)
    {
        watch_sys_sync.notify_hr_cont(&bg_hr_cont_buf);
    }
    bg_hr_cont_buf.count = 0;
    bg_hr_cont_buf.base_ts = 0;
}

static void bg_hr_cont_cb(void *param)
{
    (void)param;
    struct rt_sensor_data sd;
    int rd = (hr_service_env.device)
                 ? rt_device_read(hr_service_env.device, 0, &sd, 1) : 0;

    uint32_t qscore = 0, qlevel = 0, qconfi = 0, qsnr = 0;
    gh3018_get_hr_quality(&qscore, &qlevel, &qconfi, &qsnr);
    uint32_t acc_info = 0, acc_scene = 0;
    gh3018_get_hr_acc_state(&acc_info, &acc_scene);

    /* Close the PI window every BG_HR_CONT_PI_SECS, not every second. PI is
       (max-min)/mean over the window, and at a resting 60 bpm one beat occupies a
       full second — a 1 s window often does not span a complete pulse, so max-min
       collapses. Measured on 2026-08-04: 88% of one-second windows returned 0 and
       the non-zero ones had a median of 1, against 377 from a burst-length
       window. Eight seconds covers at least four beats down to 30 bpm.
       Reusing the burst accumulator is safe here: continuous mode suppresses
       bursts, so nothing else drives it — and it keeps
       hr_service_get_last_pi_e3() live for sleep_diag, which otherwise reported
       pi_e3 = 0 all night. */
    static uint8_t s_cont_pi_tick = 0;
    if (++s_cont_pi_tick >= BG_HR_CONT_PI_SECS)
    {
        s_cont_pi_tick = 0;
        ppg_pi_finish();
        ppg_pi_start();
    }
    uint16_t pi = bg_hr_win_pi_e3;   /* last completed window, held between closes */

    uint32_t mv = bghr_accel_delta();

    /* The buffer is full only when a flush could not be delivered; drop the oldest
       second rather than the newest so the retry still carries the most recent
       state when the link returns. */
    if (bg_hr_cont_buf.count >= WATCH_SYS_HR_CONT_MAX) return;

    uint8_t i = bg_hr_cont_buf.count;
    if (i == 0) bg_hr_cont_buf.base_ts = (uint32_t)time(NULL);
    uint8_t bpm = (rd == 1 && sd.data.hr <= 255) ? (uint8_t)sd.data.hr : 0;
    bg_hr_cont_buf.bpm[i]    = bpm;
    bg_hr_cont_buf.qscore[i] = (qscore > 255) ? 255 : (uint8_t)qscore;
    bg_hr_cont_buf.qlevel[i] = (qlevel > 255) ? 255 : (uint8_t)qlevel;
    bg_hr_cont_buf.accst[i]  = (uint8_t)(((acc_info & 0x07u) << 5) | (acc_scene & 0x1Fu));
    bg_hr_cont_buf.accel[i]  = (mv > 255) ? 255 : (uint8_t)mv;
    bg_hr_cont_buf.pi_e3[i]  = pi;
    bg_hr_cont_buf.count++;

    /* Keep sleep_fusion fed. Suppressing bursts also silenced bg_hr_win_mean/std,
       so the 2026-08-02 night was staged from accelerometer alone with hr = 0 on
       every sleep_diag row. Publish a rolling window instead of a burst summary. */
    if (bpm > 0)
    {
        bg_hr_cont_sum += bpm;
        bg_hr_cont_sum_sq += (uint32_t)bpm * (uint32_t)bpm;
        bg_hr_cont_n++;
    }

    if (bg_hr_cont_buf.count >= WATCH_SYS_HR_CONT_MAX)
    {
        if (bg_hr_cont_n >= 2)
        {
            bg_hr_win_mean = (uint8_t)(bg_hr_cont_sum / bg_hr_cont_n);
            bg_hr_win_std = bg_hr_std_from_sums(bg_hr_cont_sum, bg_hr_cont_sum_sq,
                                                (uint16_t)bg_hr_cont_n);
            bg_hr_win_tick_ms = rt_tick_get_millisecond();
        }
        bg_hr_cont_sum = 0; bg_hr_cont_sum_sq = 0; bg_hr_cont_n = 0;
        bg_hr_cont_flush();
    }
}

/* Enable/disable from the watch Settings toggle (HCPU -> HrContinuousMode). */
void hr_service_set_hr_continuous(bool enable)
{
    if ((bg_hr_cont_enabled ? true : false) == enable) return;

    if (enable)
    {
        /* Abandon any burst in flight WITHOUT running bg_hr_finish_burst(): that
           would power the LED back down, which is the one thing this mode must
           not do. Nothing is published for the partial burst -- correct, since
           it never completed. */
        if (bg_hr_sample_timer) rt_timer_stop(bg_hr_sample_timer);
        {
            /* This path abandons a burst WITHOUT bg_hr_finish_burst(), so the
               accel stream has to be released here as well or it stays on for
               good. */
            extern int bmi270_set_hr_accel_stream(int en);
            (void)bmi270_set_hr_accel_stream(0);
        }
        bg_hr_bursting = RT_FALSE;
        if (bg_hr_period_timer) rt_timer_stop(bg_hr_period_timer);

        bg_hr_cont_buf.count = 0;
        bg_hr_cont_buf.base_ts = 0;
        bg_hr_cont_buf.interval_s = (uint8_t)(BG_HR_SAMPLE_MS / 1000u);
        bg_hr_cont_sum = 0; bg_hr_cont_sum_sq = 0; bg_hr_cont_n = 0;
        bghr_prev_accel_valid = false;   /* re-seed the accel delta chain */
        ppg_pi_start();                  /* first per-second PI window */

        /* Set the flag BEFORE powering up: hr_set_power()'s veto reads it, and a
           wear_detect tick landing between the two would otherwise slip through. */
        bg_hr_cont_enabled = RT_TRUE;
        hr_control_mode(RT_SENSOR_POWER_HIGH);
        if (bg_hr_cont_timer) rt_timer_start(bg_hr_cont_timer);
        LOG_I("bg_hr: CONTINUOUS HR diag ON (1 Hz, algo never re-inits)");
    }
    else
    {
        if (bg_hr_cont_timer) rt_timer_stop(bg_hr_cont_timer);
        bg_hr_cont_flush();                 /* don't discard the partial minute */
        bg_hr_cont_enabled = RT_FALSE;      /* clear before power-down or the veto eats it */
        if (hr_service_env.ref_count == 0) hr_set_power(0);
        if (bg_hr_period_timer) rt_timer_start(bg_hr_period_timer);
        LOG_I("bg_hr: CONTINUOUS HR diag OFF -> back to bursts");
    }
}

/* Retarget the periodic tick. @ref BG_HR_RETRY_MS. RT-Thread needs the timer
   stopped before SET_TIME takes effect on a running periodic timer. */
static void bg_hr_set_period(rt_bool_t shortened)
{
    if (bg_hr_period_short == shortened || bg_hr_period_timer == RT_NULL) return;
    bg_hr_period_short = shortened;
    rt_tick_t t = rt_tick_from_millisecond(shortened ? BG_HR_RETRY_MS
                                                     : BG_HR_PERIOD_MS);
    rt_timer_stop(bg_hr_period_timer);
    rt_timer_control(bg_hr_period_timer, RT_TIMER_CTRL_SET_TIME, &t);
    rt_timer_start(bg_hr_period_timer);
    LOG_I("[BGHR] tick -> %u s", (unsigned)((shortened ? BG_HR_RETRY_MS
                                                       : BG_HR_PERIOD_MS) / 1000));
}

static void bg_hr_period_cb(void *param)
{
    (void)param;
    /* Sleep bursts are 3 min long and already run every tick — a shortened
       period there would hold the LED on all night. Always the base tick. */
    if (bg_hr_sleep_active && bg_hr_period_short) bg_hr_set_period(RT_FALSE);
    /* Continuous diag owns the sensor; its timer is stopped on enable, but a tick
       already queued when the toggle flipped would still land here. */
    if (bg_hr_cont_enabled) return;

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

    /* Raw collection owns the sensor: a burst's switch into HIGH/HR-algo mode
     * re-inits the GH3018 mid-stream (visible LED flicker + a step in the raw
     * data). Skip bursts entirely; collection already holds the LED on via
     * the hr_set_power veto, so no HR is lost that anyone is reading. */
    extern bool imu_rawdata_collection_active(void);
    if (imu_rawdata_collection_active()) { bg_hr_note(BGHR_BUSY); return; }

    int reason = bg_hr_skip_reason();
    if (reason != BGHR_OK)
    {
        /* Off-wrist / charging / not-ready: nothing will be measured until this
           clears, so drop the short retry tick (@ref BG_HR_RETRY_MS) rather
           than spin it against a condition a faster tick cannot fix. */
        bg_hr_set_period(RT_FALSE);
        bg_hr_note(reason);
        return;
    }

    /* Exercise app already measuring -> just forward its latest value, no
       extra LED burst. */
    if (hr_service_env.ref_count > 0)
    {
        uint8_t bpm = hr_service_get_latest_bpm();
        if (bpm > 0 && watch_sys_sync.notify_hr_sample)
        {
            uint32_t now_s = (uint32_t)time(NULL);
            /* NOT gated: the Exercise app is a continuous read the user is
               watching live, and exercise is exactly when a legitimate value
               leaves the resting baseline fastest. But it MUST feed the history,
               or the baseline stays at the pre-workout resting rate and the first
               background burst after the session gets refused for being real. */
            watch_sys_sync.notify_hr_sample(now_s, bpm);
            curve_push(now_s, bpm);
            s_curve_pending = 0;
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
        /* @ref BG_HR_RETRY_MS — while the last burst produced nothing, measure
           every tick (and the tick itself is shortened) until one lands. The
           flag is cleared by a successful publish, not here. */
        if (bg_hr_retry_soon) bg_hr_awake_ticks = 0;
        else if (++bg_hr_awake_ticks < BG_HR_AWAKE_SKIP) { bg_hr_note(BGHR_THROTTLE); return; }
        else bg_hr_awake_ticks = 0;
    }

    /* Start a fresh burst: power the LED, read at 1 Hz until the deadline.
       Bail BEFORE powering on if the sample timer is missing (creation failed)
       — otherwise the LED would turn on with nothing to ever finish the burst
       (stuck-on power leak + sampler wedged). */
    if (bg_hr_sample_timer == RT_NULL) { bg_hr_note(BGHR_NO_TIMER); return; }
    bg_hr_bursting = RT_TRUE;
    ppg_pi_start();   /* begin PI accumulation for this burst */
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
    bg_hr_burst_qscore_max = 0;
    bg_hr_burst_own_conf_max = 0;
    bg_hr_vendor_bpm = 0;
    bg_hr_win_captured = false;   /* one capture per burst */
    bg_hr_win_suspect = false;
    bg_hr_burst_confi_max = 0;
    bg_hr_burst_snr_max = 0;
    bg_hr_burst_qlevel = 0;
    bg_hr_burst_ms = bg_hr_sleep_active ? BG_HR_BURST_MS_SLEEP : BG_HR_BURST_MS_AWAKE;
    uint32_t bg_now_ms = rt_tick_get_millisecond();
    bg_hr_burst_start_seq = gh3018_get_hr_update_seq();  /* warm-up baseline: accept once the algo emits a NEW locked value past this */
    bg_hr_burst_start_frames = gh3018_get_ppg_frame_count(); /* frame-accounting baseline (see decl) */
    bg_hr_burst_deadline_ms = bg_now_ms + bg_hr_burst_ms;
    /* Open in HR mode (GH30X_FUNCTION_HR), the same path the foreground HR
       subscribe uses; hr_set_power(1) would open NORMAL = HRV, which never
       yields a BPM for gh3018_get_hr(). Power-down at burst end stays hr_set_power(0). */
    /* Wake the accelerometer for the burst. Standby leaves it fully shut down,
       which is why the motion compensation — ours and the vendor's, they read
       the same ring — has never seen a moving wrist at night. Paired with the
       disable in bg_hr_finish_burst; leaving it on would hold the LCPU at 25 Hz
       wakeups indefinitely. */
    extern int bmi270_set_hr_accel_stream(int en);
    (void)bmi270_set_hr_accel_stream(1);
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
    bg_hr_cont_timer = rt_timer_create(
        "bghr_c", bg_hr_cont_cb, RT_NULL,
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