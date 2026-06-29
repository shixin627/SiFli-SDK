/**
 ******************************************************************************
 * @file   <file name>
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
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
#include <time.h>
#include "string.h"
#include "board.h"
// new algo headers
#include "health_algo.h"
#include "watch_sys_service.h"

#define DBG_TAG "health.algo"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// Private Functions Declaration
static void pedo_algo_cb(gsa_pedo_info_t *info);
static void sleep_algo_cb(gsa_sleep_stt_e stt, uint32 offset);
static void act_algo_cb(gsa_act_type_e type, uint16 para);
static bool wear_chk_cb(void);
static void sleep_stat_cb(uint16 para);
static void act_lift_hdl(uint16 para);
static void act_twist_hdl(uint16 para);
static void act_wave_hdl(uint16 para);

static gsa_gs_inf_t gsensor_pos = {
    .xpos = GS_XPOS_RIGHT,
    .zpos = GS_ZPOS_UP,
    .odr = 25 * 1024,
};

static usr_prof_t user_profile = {
    .reserve = 0,
    .weight = 0,
    .height = 0,
    .age = 0,
    .gender = 0,
};

// Variables
static gsa_cbs_t gsa_cbs[] = {
    pedo_algo_cb,
    sleep_algo_cb,
    act_algo_cb,
    NULL,
    sleep_stat_cb,
};

static void (*act_cbs[])(uint16) = {
    act_lift_hdl,
    act_twist_hdl,
    act_wave_hdl};

static uint8_t _sleep_status = 0;
static uint32_t current_timestamp = 0;
static bool sleep_detect_enable = false;

static void sleep_algo_cb(gsa_sleep_stt_e stt, uint32 offset)
{
    /*Monitoring period : 18:00 ~ 10:00 +1, not in this do return*/
    if (sleep_detect_enable == false)
    {
        return;
    }

    SleepData_U mSleepData;
    SleepHead_t mSleepHead;
    algorithm_event_sleep_t event = {0};
    uint8_t buffer[8] = {0};
    uint32_t time_stamp = current_timestamp - 60 * offset;

    time_t now;

    now = time(RT_NULL);

    LOG_I("************sleep status:%d**************", stt);
    switch (stt)
    {
    case GSA_SLEEP_OFF: // band not wear TODO:add state corresponding actions
        LOG_I("\n [%s]sleep off \n", ctime(&now));
        break;
    case GSA_SLEEP_WAKE: //  Wake->Sleep
    {
        LOG_I("\n [%s]sleep wake \n", ctime(&now));
        if (_sleep_status == TGET_UP)
        {
            return; // avoid consistent wake state
        }
        event.mode = TGET_UP;
        event.starting_time_stamp = time_stamp;
        _sleep_status = TGET_UP;
    }
    break;
    case GSA_SLEEP_SLEEP: // 1. Sleep -> Wake  2. Sleep->Deep Sleep
    {
        LOG_I("\n [%s]sleep sleep \n", ctime(&now));
        event.mode = TSLEEP;
        event.starting_time_stamp = time_stamp;
        _sleep_status = TSLEEP;
    }
    break;
    case GSA_SLEEP_DEEP: //  1. Deep Sleep->Sleep  2. Deep Sleep -> Wake
    {
        LOG_I("\n [%s]sleep deep sleep \n", ctime(&now));
        event.mode = TDEEP_SLEEP;
        event.starting_time_stamp = time_stamp;
        _sleep_status = TDEEP_SLEEP;
    }
    break;
    default:
        LOG_W("[%s]invalid sleep status:%d\n", ctime(&now), stt);
        break;
    }
    /* fix sleep item length = 1 */
    mSleepHead.length = 1;
    mSleepData.bits.timeStamp = (uint16_t)((event.starting_time_stamp / 60) % 1440);
    mSleepData.bits.mode = event.mode;
    LOG_I(" timeStamp:%d,mode:%d\n\n", mSleepData.bits.timeStamp, mSleepData.bits.mode);
    LOG_I(" mSleepHead.length:%d,  sleep data:%8x\n\n", mSleepHead.length, mSleepData.data);
    if (event.mode > 0)
    {
        watch_sys_sync.notify_sleep_state(event.mode, event.starting_time_stamp);
    }
}

// 定义全局变量
uint8_t global_mode = 2;
uint32_t global_steps = 0;
float global_distance = 0.0f; // m
float global_calories = 0.0f;
uint32_t quarter_steps = 0;
float quarter_distance = 0.0f;
float quarter_calories = 0.0f;
float global_frequency = 0.0f; // steps/m
float global_speed = 0.0f;     // km/h
// 单位转换函数
bool convert_and_check_if_need_to_notify(gsa_pedo_info_t *info, int offset)
{
    bool notify = false;
    // 将定点数转换为浮点数
    float distance = info->distance / 16.0f;   // Q4 -> cm
    distance /= 100.0f;                        // cm -> m
    float speed = info->speed / 256.0f;        // Q8 -> km/h
    float frequency = info->frequency / 64.0f; // Q6 -> steps/m
    float calories = info->calories / 4.0f;    // Q2 -> cal

    // 打印日志
    LOG_I("[%s] Mode=%d Steps=%d Distance=%.2f m Speed=%.2f km/h Frequency=%.2f steps/m Calory=%.2f cal offset=%d",
          __func__,
          info->mode,
          info->steps,
          distance,
          speed,
          frequency,
          calories,
          offset);
    if (global_mode != info->mode)
    {
        global_mode = info->mode;
    }
    if (global_steps != info->steps)
    {
        global_steps = info->steps;
        notify = true;
    }
    if (global_distance != distance)
    {
        global_distance = distance;
    }
    if (global_calories != calories)
    {
        global_calories = calories;
    }
    if (global_frequency != frequency)
    {
        global_frequency = frequency;
    }
    if (global_speed != speed)
    {
        global_speed = speed;
    }
    return notify;
}

void pedo_algo_cb(gsa_pedo_info_t *info)
{
    time_t rawtime;
    struct tm *t;
    rawtime = time(&rawtime);
    /*get GMT TIME*/
    t = localtime(&rawtime);

    int offset = (t->tm_hour) * 4 + t->tm_min / 15;
    // uint8_t offset = ((rawtime / 60) % 1440) / 15;
    if (info != NULL)
    {
        if (convert_and_check_if_need_to_notify(info, offset))
        {
            watch_sys_sync.notify_health_info(global_mode, global_steps, global_distance, global_calories, global_frequency, global_speed);
        }
    }
    else
    {
        LOG_E("error: algorithm callback failed... \n");
    }
}

static void act_algo_cb(gsa_act_type_e type, uint16 para)
{
    act_cbs[type](para);
}

static void sleep_stat_cb(uint16 para)
{
    LOG_I("======sleep_stat_callback:%d======", para);
}

static void act_lift_hdl(uint16 para)
{
    LOG_I("======lift_action_callback:%d======", para);
    if (para == 0) // wrist lift
    {
    }
}

static void act_wave_hdl(uint16 para)
{
    LOG_I("~~~~~wave action calbalck~~~~~");
    /* send take photo action */
}

static void act_twist_hdl(uint16 para)
{
    // LOG_I("======twist_action_callback======\n");
}

static usr_prof_t *get_user_profile(void)
{
    LOG_I("[get_user_profile] gender:%d age:%d height:%d weight:%d\r\n", user_profile.gender,
          user_profile.age,
          user_profile.height,
          user_profile.weight);

    return &user_profile;
}

static gsa_gs_inf_t *get_gsensor_pos(void)
{
    LOG_I("[get_gsensor_pos] xpos:%d zpos:%d odr:%d\r\n", gsensor_pos.xpos,
          gsensor_pos.zpos,
          gsensor_pos.odr);
    return &gsensor_pos;
}

void set_user_profile(uint8_t age, uint8_t gender, float height, float weight)
{
    user_profile.age = age;
    user_profile.gender = gender;
    user_profile.height = (uint16_t)(height * 2); // Q1
    user_profile.weight = (uint16_t)(weight * 2); // Q1
}

void set_odr(gsa_gs_inf_t *gs_info, float odr_hz)
{
    // 将浮点数转换为 Q10 格式的定点数
    gs_info->odr = (uint32_t)(odr_hz * 1024);
}

void enable_sleep_detect(bool enable)
{
    sleep_detect_enable = enable;
}

void health_algo_init(void)
{
    set_user_profile(27, 1, 177.0, 70.0);
    set_odr(&gsensor_pos, 25.0f);
    enable_sleep_detect(true);
    rtk_gsa_init(get_user_profile(), &gsensor_pos, gsa_cbs);

    _sleep_status = TGET_UP;

    gsa_ver_t *rtk_algo_ver;
    rtk_algo_ver = rtk_gsa_get_ver();
    LOG_I("health algo version:%d.%d.%d", rtk_algo_ver->major,
          rtk_algo_ver->minor, rtk_algo_ver->revision);

    /*set action detection on/off*/
    // rtk_gsa_act_switch(GSA_ACT_LIFT, true);
}

void health_algo_reset(float odr_hz)
{
    LOG_I("[%s] odr_hz=%f", __func__, odr_hz);
    set_odr(&gsensor_pos, odr_hz);
    rtk_gsa_init(get_user_profile(), get_gsensor_pos(), gsa_cbs);
}

#define HEALTH_ALGO_TEST
#ifdef HEALTH_ALGO_TEST
#include <string.h>
int cmd_health(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "-reset") == 0)
        {
            health_algo_reset(25.0f);
        }
        else
        {
            LOG_I("Invalid parameter\n");
        }
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_health, __cmd_health, Test health D);
#endif // HEALTH_ALGO_TEST
       /************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/