/**
 ******************************************************************************
 * @file   bloc_exercise.h
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

#ifndef __BLOC_EXERCISE_H__
#define __BLOC_EXERCISE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <time.h>

#define MAX_WORKOUT_RECORDS 50
#define MAX_WORKOUT_PATH_LEN 128
#define MAX_WORKOUT_FILENAME_LEN 40
#define MAX_WORKOUT_NAME_LEN 16

// failure codes for store_exercise_data
#define EXERCISE_DATA_TOO_SHORT -1
#define EXERCISE_NO_HRM_DATA -2
#define EXERCISE_NO_CALORIES_DATA -3
#define EXERCISE_FILE_OPEN_FAILED -4

    // 运动类型枚举
    typedef enum
    {
        WORKOUT_RUNNING,
        WORKOUT_WALKING,
        WORKOUT_CYCLING,
        WORKOUT_SWIMMING,
        WORKOUT_HIKING,
        WORKOUT_YOGA,
        WORKOUT_GYM,
        WORKOUT_COUNT
    } workout_type_t;

#define EXERCISE_DURATION_NOT_ENOUGH -1

#define WORKOUT_RUNNING_TITLE "Running"
#define WORKOUT_WALKING_TITLE "Walking"
#define WORKOUT_CYCLING_TITLE "Cycling"
#define WORKOUT_SWIMMING_TITLE "Swimming"
#define WORKOUT_HIKING_TITLE "Hiking"
#define WORKOUT_YOGA_TITLE "Yoga"
#define WORKOUT_GYM_TITLE "Gym"

    // 运动记录结构
    typedef struct
    {
        char filename[MAX_WORKOUT_FILENAME_LEN]; // 文件名
        time_t timestamp;                        // 时间戳
        workout_type_t type;                     // 运动类型
        uint32_t duration;                       // 持续时间(秒)
        uint16_t calories;                       // 卡路里消耗
    } workout_record_t;

    // 运动历史结构
    typedef struct
    {
        workout_record_t *records; // 记录数组
        uint16_t count;            // 记录数量
    } workout_history_t;

#define MAX_WORKOUT_MINUTES 60 // 最大运动分钟数

    // 运动会话结构
    typedef struct
    {
        workout_type_t type;       // 运动类型
        uint32_t duration;         // 持续时间(秒)
        uint16_t heart_rate;       // 当前心率
        uint16_t calories;         // 卡路里消耗
        uint16_t hrm_array[60];    // 心率记录数组
        uint16_t hrm_count;        // 心率记录数量
        uint32_t sum_of_hr_in_min; // 每分钟心率总和
        uint16_t num_of_hr_in_min; // 每分钟心率数量
        bool is_paused;            // 是否暂停
        rt_timer_t workout_timer;  // 计时器
    } workout_session_t;

    // 函数声明
    float calculate_calories(float base_calories, uint32_t duration_seconds, uint16_t heart_rate_minutes);
    int store_exercise_data(workout_session_t *session);
    workout_history_t *get_workout_history(void);
    void free_workout_history(workout_history_t *history);
    void format_date_string(time_t timestamp, char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // __BLOC_EXERCISE_H__

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
