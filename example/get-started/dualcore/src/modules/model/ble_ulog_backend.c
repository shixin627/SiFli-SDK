#include <rtthread.h>

#ifdef RT_USING_ULOG
#include "ulog.h"
#include "ulog_def.h"
#include "wristband_ble_log.h"
#include "watch_global_data.h"
#include <string.h>

#if !kReleaseMode
// BLE backend 名稱
#define BLE_ULOG_BACKEND_NAME "ble"

// BLE backend 實例
static struct ulog_backend ble_ulog_backend;

// Re-entrancy guard to prevent recursive logging within ulog lock
static volatile rt_bool_t ble_ulog_outputting = RT_FALSE;

// BLE backend 輸出函數
static void ble_ulog_backend_output(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw, const char *log, size_t len)
{
    // Don't output binary in console.
    if (is_raw == RAW_BIN)
        return;
    // Prevent re-entrant calls that cause ulog output_lock assertion failure
    if (ble_ulog_outputting)
        return;
    ble_ulog_outputting = RT_TRUE;
    // level of rt_kprintf and LOG_D is 7
    // level of LOG_I is 6
    // level of LOG_W is 5
    // level of LOG_E is 4
    ble_log_output(level, log, len);
    ble_ulog_outputting = RT_FALSE;
}

// BLE backend 初始化
static void ble_ulog_backend_init(struct ulog_backend *backend)
{
    // 可選: BLE 初始化
}

// BLE backend flush
static void ble_ulog_backend_flush(struct ulog_backend *backend)
{
    // 可選: BLE flush 實現
}

// BLE backend deinit
static void ble_ulog_backend_deinit(struct ulog_backend *backend)
{
    // 可選: BLE deinit 實現
}

// BLE backend 註冊
__ROM_USED int ble_ulog_backend_register(void)
{
    memset(&ble_ulog_backend, 0, sizeof(ble_ulog_backend));
    strncpy(ble_ulog_backend.name, BLE_ULOG_BACKEND_NAME, RT_NAME_MAX - 1);
    ble_ulog_backend.init = ble_ulog_backend_init;
    ble_ulog_backend.output = ble_ulog_backend_output;
    ble_ulog_backend.flush = ble_ulog_backend_flush;
    ble_ulog_backend.deinit = ble_ulog_backend_deinit;
    ble_ulog_backend.support_color = RT_FALSE;
    ulog_backend_register(&ble_ulog_backend, BLE_ULOG_BACKEND_NAME, RT_FALSE);
    return 0;
}

INIT_APP_EXPORT(ble_ulog_backend_register);

// BLE backend 解除註冊
void ble_ulog_backend_unregister(void)
{
    ulog_backend_unregister(&ble_ulog_backend);
}

#endif /* !kReleaseMode */

#endif /* RT_USING_ULOG */
