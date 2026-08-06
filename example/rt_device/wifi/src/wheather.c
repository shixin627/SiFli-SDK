/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include "lwip/api.h"
#include "lwip/dns.h"
#include <webclient.h>
#include <cJSON.h>

#define GET_HEADER_BUFSZ        1024        // Header buffer size
#define GET_RESP_BUFSZ          1024        // Response buffer size
#define GET_URL_LEN_MAX         256         // Max URL length
#define GET_URI                 "http://%s/v3/weather/now.json?key=%s&location=%s&language=%s" // Weather API
#define WEATHER_HOST                    "api.seniverse.com"
#define WEATHER_KEY_ID                  "SO23_Gmly2oK3kMf4"
#define WEATHER_CITY_ID                 "chongqing" // City ID
#define WEATHER_LANGUAGE_ID             "zh-Hans&unit=c" //

typedef struct
{
    char *txt;         // Weather phenomenon text
    char *code;        // Weather phenomenon code
    char *temperature; // Temperature
} user_seniverse_now_config_t;

static uint8_t is_ip_searching;

/**
 * @brief Seniverse weather data structure
 */
typedef struct
{
    char *id;
    char *name;
    char *country;
    char *path;
    char *timezone;
    char *timezone_offset;
    user_seniverse_now_config_t now_config;
    char *last_update;
} user_seniverse_config_t;

void weather(int argc, char **argv);


void svr_found_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    if (ipaddr != NULL)
    {
        rt_kprintf("DNS lookup succeeded, IP: %s\n", ipaddr_ntoa(ipaddr));
    }
}

int check_internet_access()
{
    int r = 0;
    const char *hostname = WEATHER_HOST;
    ip_addr_t addr = {0};

    {
        err_t err = dns_gethostbyname(hostname, &addr, svr_found_callback, NULL);
        if (err != ERR_OK && err != ERR_INPROGRESS)
        {
            rt_kprintf("Coud not find %s, please check PAN connection\n", hostname);
        }
        else
            r = 1;
    }

    return r;
}

int http_weather_data_parse(char *json_data)
{

    uint8_t i, j;
    uint8_t result_array_size = 0;
    uint8_t now_array_size = 0;
    uint32_t temperature = 0;

    cJSON *item = NULL;
    cJSON *root = NULL;
    cJSON *results_root = NULL;
    cJSON *now_root = NULL;
    user_seniverse_config_t user_sen_config;

    root = cJSON_Parse(json_data);   /* json_data is the raw data from Seniverse */
    if (!root)
    {
        rt_kprintf("Error before: [%s]\n", cJSON_GetErrorPtr());
        return  -1;
    }


    cJSON *Presult = cJSON_GetObjectItem(root, "results");  /* results value is an array */
    result_array_size = cJSON_GetArraySize(Presult);  /* Get number of elements in results array */
    for (i = 0; i < result_array_size; i++)
    {
        cJSON *item_results = cJSON_GetArrayItem(Presult, i);
        char *sresults = cJSON_PrintUnformatted(item_results);
        results_root = cJSON_Parse(sresults);

        if (!results_root)
        {
            rt_kprintf("Error before: [%s]\n", cJSON_GetErrorPtr());
            return  -1;
        }
        cJSON *Plocation = cJSON_GetObjectItem(results_root, "location");

        item = cJSON_GetObjectItem(Plocation, "id");
        user_sen_config.id = cJSON_Print(item);

        item = cJSON_GetObjectItem(Plocation, "name");
        user_sen_config.name = cJSON_Print(item);

        item = cJSON_GetObjectItem(Plocation, "country");
        user_sen_config.country = cJSON_Print(item);

        item = cJSON_GetObjectItem(Plocation, "path");
        user_sen_config.path = cJSON_Print(item);

        item = cJSON_GetObjectItem(Plocation, "timezone");
        user_sen_config.timezone = cJSON_Print(item);

        item = cJSON_GetObjectItem(Plocation, "timezone_offset");
        user_sen_config.timezone_offset = cJSON_Print(item);

        /*-------------------------------------------------------------------*/
        cJSON *Pnow = cJSON_GetObjectItem(results_root, "now");

        item = cJSON_GetObjectItem(Pnow, "text");
        user_sen_config.now_config.txt = cJSON_Print(item);

        item = cJSON_GetObjectItem(Pnow, "code");
        user_sen_config.now_config.code = cJSON_Print(item);

        item = cJSON_GetObjectItem(Pnow, "temperature");
        user_sen_config.now_config.temperature = cJSON_Print(item);

        /*-------------------------------------------------------------------*/
        item = cJSON_GetObjectItem(results_root, "last_update");
        user_sen_config.last_update = cJSON_Print(item);

        cJSON_Delete(results_root);  /* Free memory after each cJSON_Parse call */

    }

    rt_kprintf("id:%s\n", user_sen_config.id);
    rt_kprintf("name:%s\n", user_sen_config.name);
    rt_kprintf("country:%s\n", user_sen_config.country);
    rt_kprintf("path:%s\n", user_sen_config.path);
    rt_kprintf("timezone:%s\n", user_sen_config.timezone);
    rt_kprintf("timezone_offset:%s\n", user_sen_config.timezone_offset);
    rt_kprintf("txt:%s\n", user_sen_config.now_config.txt);
    rt_kprintf("code:%s\n", user_sen_config.now_config.code);
    rt_kprintf("temperature:%s\n", user_sen_config.now_config.temperature);
    rt_kprintf("last_update:%s\n", user_sen_config.last_update);
    cJSON_Delete(root); /* Free memory after each cJSON_Parse call */

    return  0;
}

char *get_weather()
{
    char *buffer = RT_NULL;
    int resp_status;
    struct webclient_session *session = RT_NULL;
    char *weather_url = RT_NULL;
    int content_length = -1, bytes_read = 0;
    int content_pos = 0;

    if (check_internet_access() == 0)
        return buffer;

    /* Allocate memory for weather_url */
    weather_url = rt_calloc(1, GET_URL_LEN_MAX);
    if (weather_url == RT_NULL)
    {
        rt_kprintf("No memory for weather_url!\n");
        goto __exit;
    }
    /* Build GET URL */
    rt_snprintf(weather_url, GET_URL_LEN_MAX, GET_URI,
                WEATHER_HOST, WEATHER_KEY_ID, WEATHER_CITY_ID, WEATHER_LANGUAGE_ID);

    /* Create session and set response size */
    session = webclient_session_create(GET_HEADER_BUFSZ);
    if (session == RT_NULL)
    {
        rt_kprintf("No memory for get header!\n");
        goto __exit;
    }

    /* Send GET request with default header */
    if ((resp_status = webclient_get(session, weather_url)) != 200)
    {
        rt_kprintf("webclient GET request failed, response(%d) error.\n", resp_status);
        goto __exit;
    }

    /* Allocate buffer for received data */
    buffer = rt_calloc(1, GET_RESP_BUFSZ);
    if (buffer == RT_NULL)
    {
        rt_kprintf("No memory for data receive buffer!\n");
        goto __exit;
    }

    content_length = webclient_content_length_get(session);
    if (content_length > 0)
    {
        do
        {
            bytes_read = webclient_read(session, buffer,
                                        content_length - content_pos > GET_RESP_BUFSZ ?
                                        GET_RESP_BUFSZ : content_length - content_pos);
            if (bytes_read <= 0)
            {
                break;
            }
            content_pos += bytes_read;
        }
        while (content_pos < content_length);
    }
    else
    {
        rt_free(buffer);
        buffer = NULL;
    }
__exit:
    /* Free URL memory */
    if (weather_url != RT_NULL)
    {
        rt_free(weather_url);
        weather_url = RT_NULL;
    }

    /* Close session */
    if (session != RT_NULL)
        webclient_close(session);

    return buffer;
}


__ROM_USED void weather(int argc, char **argv)
{
    char *weather = get_weather();

    if (weather)
    {
        http_weather_data_parse(weather);
        rt_free(weather);
    }
}
MSH_CMD_EXPORT(weather, Get Weather)



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

