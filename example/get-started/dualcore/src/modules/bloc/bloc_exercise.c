/**
 ******************************************************************************
 * @file   bloc_exercise.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *j;kldjsf
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dfs_posix.h>
#include "cJSON.h"
#include "bloc_exercise.h"
#include "bloc_filesystem.h"
#include "watch_global_data.h"
#include "communicate_protocol.h"
#include "ui_handler.h"
#include <time.h>

#define DBG_TAG "bloc.exercise"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

// workout title string list
const char *workout_title_list[WORKOUT_COUNT] =
    {WORKOUT_RUNNING_TITLE, WORKOUT_WALKING_TITLE, WORKOUT_CYCLING_TITLE,
     WORKOUT_SWIMMING_TITLE, WORKOUT_HIKING_TITLE, WORKOUT_YOGA_TITLE, WORKOUT_GYM_TITLE};

// Calculate calories burned based on heart rate
float calculate_calories(float base_calories, uint32_t duration_seconds, uint16_t heart_rate_minutes)
{
    if (duration_seconds == 0 || heart_rate_minutes <= 40)
    {
        LOG_W("Invalid duration or heart rate, returning 0 calories burned");
        return 0;
    }
    // Get user data
    float weight = 70.0f; // Default weight in kg
    int age = 30;         // Default age
    bool is_male = true;  // Default gender

#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
    // Use actual user data if available
    weight = SkaiWatchSys.user_data.user_profile.bit_field.weight > 0 ? SkaiWatchSys.user_data.user_profile.bit_field.weight : weight;
    age = SkaiWatchSys.user_data.user_profile.bit_field.age > 0 ? SkaiWatchSys.user_data.user_profile.bit_field.age : age;
    is_male = SkaiWatchSys.user_data.user_profile.bit_field.gender == 1; // Assuming 1 is male
#endif

    LOG_D("Calorie calculation inputs: base_calories=%.2f, duration=%u sec, heart_rate=%u",
          base_calories, duration_seconds, heart_rate_minutes);
    LOG_D("User data: weight=%.1f kg, age=%d, gender=%s",
          weight, age, is_male ? "male" : "female");

    float duration_minutes = duration_seconds / 60.0f;
    float calories_burned = 0;
    float formula_result = 0;

    if (is_male)
    {
        // Men: CB = T × (0.6309×H + 0.1988×W + 0.2017×A - 55.0969) / 4.184
        float hr_term = 0.6309 * heart_rate_minutes;
        float weight_term = 0.1988 * weight;
        float age_term = 0.2017 * age;
        float constant_term = 55.0969;

        formula_result = (hr_term + weight_term + age_term - constant_term) / 4.184;
        calories_burned = duration_minutes * formula_result;

        LOG_D("Male formula breakdown: (0.6309×%u + 0.1988×%.1f + 0.2017×%d - 55.0969)/4.184",
              heart_rate_minutes, weight, age);
        LOG_D("Male formula terms: HR=%.2f, weight=%.2f, age=%.2f, constant=%.2f",
              hr_term, weight_term, age_term, constant_term);
        LOG_D("Formula result=%.2f, Duration=%.2f min, Base calculation=%.2f",
              formula_result, duration_minutes, calories_burned);
    }
    else
    {
        // Women: CB = T × (0.4472×H - 0.1263×W + 0.074×A - 20.4022) / 4.184
        float hr_term = 0.4472 * heart_rate_minutes;
        float weight_term = 0.1263 * weight;
        float age_term = 0.074 * age;
        float constant_term = 20.4022;

        formula_result = (hr_term - weight_term + age_term - constant_term) / 4.184;
        calories_burned = duration_minutes * formula_result;

        LOG_D("Female formula breakdown: (0.4472×%u - 0.1263×%.1f + 0.074×%d - 20.4022)/4.184",
              heart_rate_minutes, weight, age);
        LOG_D("Female formula terms: HR=%.2f, weight=%.2f, age=%.2f, constant=%.2f",
              hr_term, weight_term, age_term, constant_term);
        LOG_D("Formula result=%.2f, Duration=%.2f min, Base calculation=%.2f",
              formula_result, duration_minutes, calories_burned);
    }

    // Apply workout type specific multiplier
    float activity_multiplier = (base_calories / 7.0f);
    calories_burned *= activity_multiplier;

    LOG_D("Activity multiplier (base_calories/7.0): %.2f/7.0 = %.4f",
          base_calories, activity_multiplier);
    LOG_D("Final calories burned: %.2f", calories_burned);

    if (calories_burned < 0)
    {
        LOG_W("Negative calories burned, setting to 0");
        calories_burned = 0;
    }

    return calories_burned;
}

/**
 * @brief 存储运动会话数据到文件
 *
 * 存储格式为JSON，包含:
 * - 运动类型
 * - 持续时间
 * - 卡路里消耗
 * - 心率记录
 */
static char last_exercise_file[MAX_WORKOUT_PATH_LEN] = {0};

const char *get_last_exercise_file(void)
{
    return last_exercise_file;
}

#define MAX_DAILY_EXERCISE_FILE_BYTES 16384

/* Read an existing daily file into a cJSON root, or return NULL. */
static cJSON *read_daily_exercise_file(const char *filename)
{
    FILE *fp = fopen(filename, "r");
    if (fp == NULL)
    {
        return NULL;
    }
    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (file_size <= 0 || file_size > MAX_DAILY_EXERCISE_FILE_BYTES)
    {
        fclose(fp);
        return NULL;
    }
    char *buf = (char *)rt_malloc(file_size + 1);
    if (buf == NULL)
    {
        fclose(fp);
        return NULL;
    }
    size_t read_size = fread(buf, 1, file_size, fp);
    buf[read_size] = '\0';
    fclose(fp);

    cJSON *root = cJSON_Parse(buf);
    rt_free(buf);
    return root;
}

int store_exercise_data(workout_session_t *session)
{
    // 如果沒有有效的運動數據，則不保存
    if (session->duration <= 10)
    {
        LOG_D("Exercise too short, not saving");
        return EXERCISE_DATA_TOO_SHORT;
    }

    if (session->hrm_count == 0)
    {
        LOG_D("No heart rate data, not saving");
        return EXERCISE_NO_HRM_DATA;
    }

    if (session->calories == 0)
    {
        LOG_D("No calories data, not saving");
        return EXERCISE_NO_CALORIES_DATA;
    }

    watch_storage_api_lock();

    time_t now;
    struct tm *tm_info;
    char filename[40];

    // 取得目前時間
    time(&now);
    tm_info = localtime(&now);

    mkdir("/exercise", 0x777);

    // 以日期為單位的檔名：YYYYMMDD.json
    snprintf(filename, sizeof(filename), "/exercise/%04d%02d%02d.json",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday);

    // 讀取當天既有的檔案（若存在），否則建立新的 root
    cJSON *root = read_daily_exercise_file(filename);
    cJSON *sessions = NULL;
    if (root != NULL)
    {
        sessions = cJSON_GetObjectItem(root, "sessions");
        if (sessions == NULL || !cJSON_IsArray(sessions))
        {
            cJSON_DeleteItemFromObject(root, "sessions");
            sessions = cJSON_CreateArray();
            cJSON_AddItemToObject(root, "sessions", sessions);
        }
    }
    else
    {
        root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "date", (long)now);
        sessions = cJSON_CreateArray();
        cJSON_AddItemToObject(root, "sessions", sessions);
    }

    // 建立這次 session 的 JSON 物件
    cJSON *session_item = cJSON_CreateObject();
    cJSON_AddStringToObject(session_item, "activity", workout_title_list[session->type]);
    cJSON_AddNumberToObject(session_item, "timestamp", (long)now);
    cJSON_AddNumberToObject(session_item, "duration_sec", session->duration);
    cJSON_AddNumberToObject(session_item, "calories", session->calories);
    cJSON *hrm_array = cJSON_CreateArray();
    for (int i = 0; i < session->hrm_count; i++)
    {
        cJSON_AddItemToArray(hrm_array, cJSON_CreateNumber(session->hrm_array[i]));
    }
    cJSON_AddItemToObject(session_item, "hrm", hrm_array);

    // 加入當天的 sessions 陣列
    cJSON_AddItemToArray(sessions, session_item);

    // 轉換成字串
    char *json_str = cJSON_Print(root);
    if (json_str == NULL)
    {
        LOG_E("cJSON_Print failed");
        cJSON_Delete(root);
        watch_storage_api_unlock();
        return EXERCISE_FILE_OPEN_FAILED;
    }
    LOG_D("Daily exercise data: %s", json_str);

    // 覆寫檔案
    FILE *fp = fopen(filename, "w");
    if (fp != NULL)
    {
        fwrite(json_str, strlen(json_str), 1, fp);
        fclose(fp);
        strncpy(last_exercise_file, filename, sizeof(last_exercise_file) - 1);
        last_exercise_file[sizeof(last_exercise_file) - 1] = '\0';
    }
    else
    {
        LOG_E("Failed to open file %s", filename);
        cJSON_free(json_str);
        cJSON_Delete(root);
        watch_storage_api_unlock();
        return EXERCISE_FILE_OPEN_FAILED;
    }

    cJSON_free(json_str);
    cJSON_Delete(root);

    rt_thread_mdelay(50);

    watch_storage_api_unlock();
    return 0;
}

/**
 * @brief 将时间戳格式化为日期字符串
 */
void format_date_string(time_t timestamp, char *buffer, size_t buffer_size)
{
    struct tm *tm_info = localtime(&timestamp);
    strftime(buffer, buffer_size, "%Y-%m-%d %H:%M", tm_info);
}

/**
 * @brief 获取运动历史记录
 * @return 包含运动历史记录的结构体指针，需要用户释放
 */
workout_history_t *get_workout_history(void)
{
    DIR *dir;
    struct dirent *ent;
    char path[MAX_WORKOUT_PATH_LEN];
    uint16_t parse_failures = 0;

    // 分配内存
    workout_history_t *history = (workout_history_t *)rt_malloc(sizeof(workout_history_t));
    if (!history)
    {
        LOG_E("Failed to allocate memory for workout history");
        return NULL;
    }
    memset(history, 0, sizeof(workout_history_t));

    // 打开目录
    dir = opendir("/exercise");
    if (dir == NULL)
    {
        LOG_D("Exercise directory not found, creating...");
        mkdir("/exercise", 0x777);
        history->count = 0;
        return history;
    }

    // 分配记录数组内存（以每天多個 session 計，上限 MAX_WORKOUT_RECORDS）
    history->records = (workout_record_t *)rt_malloc(MAX_WORKOUT_RECORDS * sizeof(workout_record_t));
    if (!history->records)
    {
        LOG_E("Failed to allocate memory for workout records");
        rt_free(history);
        closedir(dir);
        return NULL;
    }
    memset(history->records, 0, MAX_WORKOUT_RECORDS * sizeof(workout_record_t));

    // 遍歷所有每日檔案並展開每天的 sessions 陣列
    uint16_t index = 0;
    uint16_t files_processed = 0;
    while ((ent = readdir(dir)) != NULL && index < MAX_WORKOUT_RECORDS)
    {
        if (ent->d_type != DT_REG || !strstr(ent->d_name, ".json"))
        {
            continue;
        }
        files_processed++;
        snprintf(path, MAX_WORKOUT_PATH_LEN, "/exercise/%s", ent->d_name);

        FILE *fp = fopen(path, "r");
        if (!fp)
        {
            LOG_W("Failed to open file %s", ent->d_name);
            parse_failures++;
            continue;
        }

        fseek(fp, 0, SEEK_END);
        long file_size = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        if (file_size <= 0 || file_size > MAX_DAILY_EXERCISE_FILE_BYTES)
        {
            LOG_W("File %s size invalid: %ld bytes", ent->d_name, file_size);
            parse_failures++;
            fclose(fp);
            continue;
        }

        char *json_str = (char *)rt_malloc(file_size + 1);
        if (!json_str)
        {
            LOG_W("Failed to allocate memory for JSON string from %s", ent->d_name);
            parse_failures++;
            fclose(fp);
            continue;
        }
        size_t read_size = fread(json_str, 1, file_size, fp);
        json_str[read_size] = '\0';
        fclose(fp);

        cJSON *root = cJSON_Parse(json_str);
        rt_free(json_str);
        if (!root)
        {
            LOG_W("Failed to parse JSON from file %s", ent->d_name);
            parse_failures++;
            continue;
        }

        cJSON *sessions = cJSON_GetObjectItem(root, "sessions");
        if (!sessions || !cJSON_IsArray(sessions))
        {
            LOG_W("File %s has no sessions array", ent->d_name);
            parse_failures++;
            cJSON_Delete(root);
            continue;
        }

        int n_sessions = cJSON_GetArraySize(sessions);
        for (int si = 0; si < n_sessions && index < MAX_WORKOUT_RECORDS; si++)
        {
            cJSON *s = cJSON_GetArrayItem(sessions, si);
            if (!s) continue;

            cJSON *activity = cJSON_GetObjectItem(s, "activity");
            cJSON *timestamp = cJSON_GetObjectItem(s, "timestamp");
            cJSON *duration = cJSON_GetObjectItem(s, "duration_sec");
            cJSON *calories = cJSON_GetObjectItem(s, "calories");

            if (!activity || !activity->valuestring ||
                !timestamp || !duration || !calories)
            {
                LOG_W("Session %d in %s missing required fields", si, ent->d_name);
                parse_failures++;
                continue;
            }

            strncpy(history->records[index].filename, ent->d_name, MAX_WORKOUT_FILENAME_LEN - 1);
            history->records[index].filename[MAX_WORKOUT_FILENAME_LEN - 1] = '\0';
            history->records[index].timestamp = timestamp->valueint;
            history->records[index].duration = duration->valueint;
            history->records[index].calories = calories->valueint;

            for (int i = 0; i < WORKOUT_COUNT; i++)
            {
                if (strcmp(activity->valuestring, workout_title_list[i]) == 0)
                {
                    history->records[index].type = i;
                    break;
                }
            }

            index++;
        }

        cJSON_Delete(root);
    }

    uint16_t file_count = files_processed;
    closedir(dir);

    // 按时间戳排序 (降序)
    for (int i = 0; i < index - 1; i++)
    {
        for (int j = 0; j < index - i - 1; j++)
        {
            if (history->records[j].timestamp < history->records[j + 1].timestamp)
            {
                // 交换记录
                workout_record_t temp = history->records[j];
                history->records[j] = history->records[j + 1];
                history->records[j + 1] = temp;
            }
        }
    }

    // 更新实际记录数
    history->count = index;

    // Log diagnostic information
    if (file_count != index)
    {
        LOG_D("Workout history summary: found %d files, processed %d files, successfully parsed %d records, failed to parse %d records",
              file_count, files_processed, index, parse_failures);
    }

    return history;
}

/**
 * @brief 释放运动历史记录
 */
void free_workout_history(workout_history_t *history)
{
    if (history)
    {
        if (history->records)
        {
            rt_free(history->records);
        }
        rt_free(history);
    }
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
