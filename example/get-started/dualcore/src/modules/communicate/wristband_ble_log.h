/**
*********************************************************************************************************
*               Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
**********************************************************************************************************
* @file     wristband_ble_log.h
* @brief    This file provides code of bluetooth low energy
            Log debug.
* @details
* @author   jack
* @date     2023-02-08
* @version  v1.0
*********************************************************************************************************
*/
#ifndef __WRISTBAND_BLUETOOTH_LOG_H__
#define __WRISTBAND_BLUETOOTH_LOG_H__

#ifdef __cplusplus
extern "C"
{
#endif
#define BLE_LOG_BUFFER_SIZE 256 //(512 - 5)

    typedef enum
    {
        KEY_DEBUG = 0x01,
    } BLUETOOTH_LOG_KEY;

#define BLE_LOG_LEVEL_DEBUG 7
#define BLE_LOG_LEVEL_INFO 6
#define BLE_LOG_LEVEL_WARN 5
#define BLE_LOG_LEVEL_ERROR 4
#define BLE_LOG_LEVEL_NONE 0

    extern void ble_log_output(rt_uint32_t level, const char *buf, size_t len);
    extern void BLE_LOG_D(const char *format, ...);
    extern void BLE_LOG_I(const char *format, ...);
    extern void BLE_LOG_W(const char *format, ...);
    extern void BLE_LOG_E(const char *format, ...);
    
#ifdef __cplusplus
}
#endif

#endif
