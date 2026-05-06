/**
 *********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 *********************************************************************************************************
 * @file     wristband_ble_log.c
 * @brief    BLE log channel — pipes ulog-style messages over the Skaiwatch
 *           private BLE protocol so the phone app can capture device logs in
 *           dev builds.
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
#include "communicate_task.h"
#include "wristband_ble_log.h"
#include "watch_global_data.h"

#if !kReleaseMode

#define BLE_LOG_DBG_LV BLE_LOG_LEVEL_INFO

/* Shared scratch — single-threaded BLE rx context so no locking, but it does
   mean nested LOG calls would clobber. Don't call BLE_LOG_X from a callback
   that itself was invoked while a BLE_LOG_X was building. */
static char log_buffer[BLE_LOG_BUFFER_SIZE];

/* Format and send one BLE log line, prefixed with "<level>/". The caller's
   level filter is applied at the call site so unused levels become empty
   bodies that the compiler can drop. */
static void ble_log_emit_v(char prefix, const char *format, va_list args)
{
    log_buffer[0] = prefix;
    log_buffer[1] = '/';
    vsnprintf(log_buffer + 2, BLE_LOG_BUFFER_SIZE - 2, format, args);
    commu_send_bluetooth_log(log_buffer);
}

/* Body shared by all four level-specific entry points. Wrapped in `if (...)`
   with a literal compile-time constant on each side: the compiler folds the
   condition and DCEs the unused bodies.
   NOTE: matches the legacy BLE_LOG_X functions exactly — no debug_mode guard
   (only ble_log_output below has one). Callers are already inside !kReleaseMode. */
#define BLE_LOG_BODY(level, prefix)                                            \
    if (BLE_LOG_DBG_LV >= (level))                                             \
    {                                                                          \
        va_list args;                                                          \
        va_start(args, format);                                                \
        ble_log_emit_v((prefix), format, args);                                \
        va_end(args);                                                          \
    }

void BLE_LOG_D(const char *format, ...) { BLE_LOG_BODY(BLE_LOG_LEVEL_DEBUG, 'D') }
void BLE_LOG_I(const char *format, ...) { BLE_LOG_BODY(BLE_LOG_LEVEL_INFO,  'I') }
void BLE_LOG_W(const char *format, ...) { BLE_LOG_BODY(BLE_LOG_LEVEL_WARN,  'W') }
void BLE_LOG_E(const char *format, ...) { BLE_LOG_BODY(BLE_LOG_LEVEL_ERROR, 'E') }

#undef BLE_LOG_BODY

/* Used by the ulog backend hook to forward a pre-formatted line. Caps at
   buffer-1 to leave room for the null. */
void ble_log_output(rt_uint32_t level, const char *buf, size_t len)
{
    if (SkaiWatchSys.flag_field.debug_mode == 0) return;
    if (buf == NULL) return;
    if (level > BLE_LOG_DBG_LV) return;

    if (len >= BLE_LOG_BUFFER_SIZE)
    {
        len = BLE_LOG_BUFFER_SIZE - 1;
    }
    memcpy(log_buffer, buf, len);
    log_buffer[len] = '\0';
    commu_send_bluetooth_log(log_buffer);
}

#endif // !kReleaseMode
