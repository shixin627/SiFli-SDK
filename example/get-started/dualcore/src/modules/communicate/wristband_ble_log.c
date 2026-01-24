/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
**********************************************************************************************************
* @file     wristband_ble_log.c
* @brief    This file provides code of bluetooth low energy
            Log debug.
* @details
* @author   shixin
* @date     2023-02-08
* @version  v1.0
*********************************************************************************************************
*/
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "communicate_protocol.h"
#include "wristband_ble_log.h"
#include "watch_global_data.h"

#define BLE_LOG_DBG_LV BLE_LOG_LEVEL_INFO

// int g_ble_ulog_enable = 0;

// #define MIN_SEND_INTERVAL_MS 60 // 最小發送間隔，單位為毫秒
static char log_buffer[BLE_LOG_BUFFER_SIZE];

#ifdef MIN_SEND_INTERVAL_MS
static rt_tick_t last_send_tick = 0;
bool check_send_interval(void)
{
    rt_tick_t current_tick = rt_tick_get();
    if (current_tick - last_send_tick < rt_tick_from_millisecond(MIN_SEND_INTERVAL_MS))
    {
        return false;
    }
    last_send_tick = current_tick;
    return true;
}
#endif

void BLE_LOG_OUTPUT(char *buf)
{
    L1SendData data = {.event = L1SEND_BLUETOOTH_LOG, .res.log_buffer_ptr = buf};
    L1_send_event(data);
}

void ble_log_output(rt_uint32_t level, const char *buf, size_t len)
{
    if (SkaiWatchSys.flag_field.debug_mode == 0)
    {
        return;
    }

    if (buf == NULL)
    {
        return;
    }

    if (level > BLE_LOG_DBG_LV)
    {
        return;
    }

    if (len >= BLE_LOG_BUFFER_SIZE)
    {
        len = BLE_LOG_BUFFER_SIZE - 1;
    }

    memcpy(log_buffer, buf, len);
    log_buffer[len] = '\0';
    BLE_LOG_OUTPUT(log_buffer);
}

void BLE_LOG_D(const char *format, ...)
{
#if BLE_LOG_DBG_LV >= BLE_LOG_LEVEL_DEBUG
    memset(log_buffer, 0, BLE_LOG_BUFFER_SIZE);
    va_list args;
    va_start(args, format);
    strcat(log_buffer, "D/");
    vsnprintf(log_buffer + 2, BLE_LOG_BUFFER_SIZE - 2, format, args);
    va_end(args);
    BLE_LOG_OUTPUT(log_buffer);
#endif
}

void BLE_LOG_I(const char *format, ...)
{
#if BLE_LOG_DBG_LV >= BLE_LOG_LEVEL_INFO
    memset(log_buffer, 0, BLE_LOG_BUFFER_SIZE);
    va_list args;
    va_start(args, format);
    strcat(log_buffer, "I/");
    vsnprintf(log_buffer + 2, BLE_LOG_BUFFER_SIZE - 2, format, args);
    va_end(args);
    BLE_LOG_OUTPUT(log_buffer);
#endif
}

void BLE_LOG_W(const char *format, ...)
{
#if BLE_LOG_DBG_LV >= BLE_LOG_LEVEL_WARN
    memset(log_buffer, 0, BLE_LOG_BUFFER_SIZE);
    va_list args;
    va_start(args, format);
    strcat(log_buffer, "W/");
    vsnprintf(log_buffer + 2, BLE_LOG_BUFFER_SIZE - 2, format, args);
    va_end(args);
    BLE_LOG_OUTPUT(log_buffer);
#endif
}

void BLE_LOG_E(const char *format, ...)
{
#if BLE_LOG_DBG_LV >= BLE_LOG_LEVEL_ERROR
    memset(log_buffer, 0, BLE_LOG_BUFFER_SIZE);
    va_list args;
    va_start(args, format);
    strcat(log_buffer, "E/");
    vsnprintf(log_buffer + 2, BLE_LOG_BUFFER_SIZE - 2, format, args);
    va_end(args);
    BLE_LOG_OUTPUT(log_buffer);
#endif
}