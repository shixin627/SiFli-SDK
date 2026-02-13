/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <board.h>
#include <string.h>
#include <math.h>
#include "bf0_hal_hlp.h"
#include "bf0_sibles.h"
#include "bf0_ble_gap.h"
#include "bf0_sibles_nvds.h"
#include "bf0_ble_fmpt.h"
#include "bf0_ble_pxpr.h"
// #define USING_BLE_SERIAL
#ifdef USING_BLE_SERIAL
    #include "bf0_sibles_serial_trans_service.h"
#endif
#include "bf0_ble_bass.h"
#ifdef BSP_BLE_TIMEC
    #include "bf0_ble_tipc.h"
#endif
#include "bf0_sibles_advertising.h"

#include "watch_global_data.h"
#include "communicate_protocol.h"
#include "bloc_control.h"
#include "bloc_peripheral.h"
#include "bloc_weather.h"
#include "bloc_setting.h"
#include "bloc_notification.h"
#include "watch_system_interact.h"
#include "watch_system_core_task.h"
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
#endif
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising_internal.h"
#include "ble_connection_manager.h"
#include "ble_device_manager.h"
#include "ble_hid.h"

#define DBG_TAG "ble.app"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// Set default strategy if not defined
#ifndef CONNECT_STRATEGY
    #define CONNECT_STRATEGY CONNECT_STRATEGY_GENERAL
#endif

// Storage for target device address
static ble_gap_addr_t g_target_device_addr = {0};
static uint8_t g_has_target_device = 0;

typedef struct
{
    uint8_t is_power_on;
    uint8_t conn_idx;
#if ENABLE_BG_ADV
    uint8_t is_bg_adv_on;
#endif
    struct
    {
        bd_addr_t peer_addr;
        uint16_t mtu;
        uint16_t conn_interval;
        uint8_t peer_addr_type;
    } conn_para;
    struct
    {
        sibles_hdl srv_handle;
        uint8_t is_audio_subscribed;
    } data;
    ble_hid_data_t hid_data; // HID-specific data
    rt_mailbox_t mb_handle;
} app_env_t;

#define BLE_APP_CATEID 0x4

static app_env_t g_app_env;
static app_env_t *ble_app_get_env(void);

#define USE_BLE_RSSI_CHECK_TIMER
#ifdef USE_BLE_RSSI_CHECK_TIMER
// --------- create a timer to read the RSSI value every 5 seconds ---------
extern uint8_t ble_gap_get_remote_rssi(ble_gap_get_rssi_t *rssi);
static rt_timer_t rssi_timer = NULL;
static void rssi_timer_callback(void *parameter)
{
    ble_gap_get_rssi_t rssi;
    rssi.conn_idx = g_app_env.conn_idx;
    uint8_t ret = ble_gap_get_remote_rssi(&rssi);
}
void start_ble_rssi_checker(void)
{
    if (!rssi_timer)
    {
        rssi_timer =
            rt_timer_create("rssi_timer", rssi_timer_callback, NULL,
                            RT_TICK_PER_SECOND, RT_TIMER_FLAG_PERIODIC);
    }
    else
    {
        rt_timer_stop(rssi_timer);
    }
    rt_timer_start(rssi_timer);
}
void stop_ble_rssi_checker(void)
{
    if (rssi_timer)
    {
        rt_timer_stop(rssi_timer);
        rt_timer_delete(rssi_timer);
        rssi_timer = NULL;
    }
}
#endif

// void notify_signal_bad(bool bad)
// {
// #ifdef SHOW_BAD_SIGNAL_INDICATOR
//     #ifdef BSP_USING_UI_HANDLER
//     lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_BAD_SIGNAL_INDICATOR,
//                       .data.action = bad};
//     lvgl_send_msg(msg);
//     #endif
// #endif
// }

// --------- GAP Info Reader (read remote Device Name & Appearance after connection) ---------
typedef struct
{
    uint8_t conn_idx;
    int8_t dev_idx;           // index in device manager
    uint16_t remote_handle;
    uint16_t name_value_hdl;
    uint16_t appearance_hdl;  // Appearance characteristic (0x2A01)
    uint16_t svc_start_hdl;
    uint16_t svc_end_hdl;
    uint8_t busy;
} dev_name_reader_t;

static dev_name_reader_t g_name_reader = {0};
static int dev_name_gattc_callback(uint16_t event_id, uint8_t *data, uint16_t len);

static ble_device_type_t appearance_to_device_type(uint16_t appearance)
{
    // BLE Appearance categories (high 10 bits = category)
    uint16_t category = appearance >> 6;
    switch (category)
    {
    case 1:  // Phone (0x0040-0x007F)
        return DEVICE_TYPE_PHONE;
    case 2:  // Computer (0x0080-0x00BF)
        return DEVICE_TYPE_COMPUTER;
    case 15: // HID (0x03C0-0x03FF) - tablets often report as HID
        return DEVICE_TYPE_TABLET;
    default:
        return DEVICE_TYPE_OTHER;
    }
}

static void gap_reader_cleanup(void)
{
    sibles_unregister_remote_svc(g_name_reader.conn_idx,
                                 g_name_reader.svc_start_hdl,
                                 g_name_reader.svc_end_hdl,
                                 dev_name_gattc_callback);
    g_name_reader.busy = 0;
}

static int dev_name_gattc_callback(uint16_t event_id, uint8_t *data, uint16_t len)
{
    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        if (rsp->status != 0)
        {
            LOG_W("GAP service register failed, status=%d", rsp->status);
            g_name_reader.busy = 0;
            break;
        }
        LOG_I("GAP service registered, reading Device Name (hdl=0x%x)...",
              g_name_reader.name_value_hdl);

        // Read Device Name first
        sibles_read_remote_value_req_t req;
        req.read_type = SIBLES_READ;
        req.handle = g_name_reader.name_value_hdl;
        req.offset = 0;
        req.length = 0;
        int8_t ret = sibles_read_remote_value(g_name_reader.remote_handle,
                                               g_name_reader.conn_idx, &req);
        if (ret != 0)
        {
            LOG_W("Failed to read Device Name, ret=%d", ret);
            g_name_reader.busy = 0;
        }
        break;
    }
    case SIBLES_READ_REMOTE_VALUE_RSP:
    {
        sibles_read_remote_value_rsp_t *rsp = (sibles_read_remote_value_rsp_t *)data;

        LOG_I("GATT read rsp: handle=0x%x, length=%d", rsp->handle, rsp->length);

        if (rsp->handle == g_name_reader.name_value_hdl)
        {
            // Device Name response
            if (rsp->length > 0 && rsp->value)
            {
                char name_buf[DEVICE_NAME_MAX_LEN];
                uint16_t copy_len = rsp->length < (DEVICE_NAME_MAX_LEN - 1)
                                        ? rsp->length
                                        : (DEVICE_NAME_MAX_LEN - 1);
                memcpy(name_buf, rsp->value, copy_len);
                name_buf[copy_len] = '\0';

                LOG_I("Remote Device Name: \"%s\" (len=%d)", name_buf, rsp->length);

                if (g_name_reader.dev_idx >= 0)
                {
                    ble_dev_mgr_update_device_name(g_name_reader.dev_idx, name_buf);
                }
            }
            else
            {
                LOG_W("Device Name read returned empty");
            }

            // Now read Appearance if available
            if (g_name_reader.appearance_hdl != 0)
            {
                LOG_I("Reading Appearance (hdl=0x%x)...", g_name_reader.appearance_hdl);
                sibles_read_remote_value_req_t req;
                req.read_type = SIBLES_READ;
                req.handle = g_name_reader.appearance_hdl;
                req.offset = 0;
                req.length = 0;
                int8_t ret = sibles_read_remote_value(g_name_reader.remote_handle,
                                                       g_name_reader.conn_idx, &req);
                if (ret != 0)
                {
                    LOG_W("Failed to read Appearance, ret=%d", ret);
                    gap_reader_cleanup();
                }
            }
            else
            {
                gap_reader_cleanup();
            }
        }
        else if (rsp->handle == g_name_reader.appearance_hdl)
        {
            // Appearance response
            if (rsp->length >= 2 && rsp->value)
            {
                uint16_t appearance = rsp->value[0] | (rsp->value[1] << 8);
                ble_device_type_t dev_type = appearance_to_device_type(appearance);

                LOG_I("Remote Appearance: 0x%04X -> type=%d", appearance, dev_type);

                if (g_name_reader.dev_idx >= 0)
                {
                    ble_dev_mgr_update_device_type(g_name_reader.dev_idx, dev_type);
                }
            }
            else
            {
                LOG_W("Appearance read returned empty (len=%d)", rsp->length);
            }

            gap_reader_cleanup();
        }
        else
        {
            LOG_W("Unexpected read rsp handle=0x%x", rsp->handle);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

static void dev_name_start_search(uint8_t conn_idx, int8_t dev_idx)
{
    if (g_name_reader.busy)
    {
        LOG_W("Device name reader busy, skip");
        return;
    }
    g_name_reader.conn_idx = conn_idx;
    g_name_reader.dev_idx = dev_idx;
    g_name_reader.busy = 1;

    uint16_t gap_svc_uuid = ATT_UUID_16(0x1800); // Generic Access Service
    int8_t ret = sibles_search_service(conn_idx, ATT_UUID_16_LEN,
                                        (uint8_t *)&gap_svc_uuid);
    if (ret != 0)
    {
        LOG_W("Failed to search GAP service, ret=%d", ret);
        g_name_reader.busy = 0;
    }
}

static bool _rssi_signal_bad = false;
bool is_signal_bad(void)
{
    return _rssi_signal_bad;
}
void set_signal_bad(bool bad)
{
    if (bad != _rssi_signal_bad)
    {
        _rssi_signal_bad = bad;
        // notify_signal_bad(bad);
    }
}

static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}

SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_bg_context);

SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);

/**
 * @brief Get the first bonded device address for directed advertising
 * @param addr Pointer to store the bonded device address
 * @return 1 if bonded device found, 0 otherwise
 */
static uint8_t ble_app_get_bonded_device_addr(ble_gap_addr_t *addr)
{
    if (g_has_target_device)
    {
        memcpy(addr, &g_target_device_addr, sizeof(ble_gap_addr_t));
        LOG_I("Using cached bonded device addr: %02x:%02x:%02x:%02x:%02x:%02x "
              "(type:%d)",
              addr->addr.addr[5], addr->addr.addr[4], addr->addr.addr[3],
              addr->addr.addr[2], addr->addr.addr[1], addr->addr.addr[0],
              addr->addr_type);
        return 1;
    }

    // Get bonded device database from device manager
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    int active_idx = ble_dev_mgr_get_active_device();

    if (!db)
    {
        LOG_E("Failed to get device database");
        return 0;
    }

    // If there's an active device, use it
    if (active_idx >= 0)
    {
        const bonded_device_t *dev = &db->devices[active_idx];
        if (dev->is_valid)
        {
            memcpy(addr->addr.addr, dev->mac_addr, 6);
            addr->addr_type = dev->addr_type;

            // Cache the bonded device
            memcpy(&g_target_device_addr, addr, sizeof(ble_gap_addr_t));
            g_has_target_device = 1;

            LOG_I("Found active device[%d] addr: %02x:%02x:%02x:%02x:%02x:%02x "
                  "(type:%d)",
                  active_idx, addr->addr.addr[5], addr->addr.addr[4],
                  addr->addr.addr[3], addr->addr.addr[2], addr->addr.addr[1],
                  addr->addr.addr[0], addr->addr_type);
            return 1;
        }
    }

    // No active device, try to find any valid bonded device
    for (int i = 0; i < db->count; i++)
    {
        const bonded_device_t *dev = &db->devices[i];
        if (dev->is_valid)
        {
            memcpy(addr->addr.addr, dev->mac_addr, 6);
            addr->addr_type = dev->addr_type;

            // Cache the bonded device
            memcpy(&g_target_device_addr, addr, sizeof(ble_gap_addr_t));
            g_has_target_device = 1;

            LOG_I("Found bonded device[%d] addr: %02x:%02x:%02x:%02x:%02x:%02x "
                  "(type:%d)",
                  i, addr->addr.addr[5], addr->addr.addr[4], addr->addr.addr[3],
                  addr->addr.addr[2], addr->addr.addr[1], addr->addr.addr[0],
                  addr->addr_type);
            return 1;
        }
    }

    LOG_I("No bonded device found");
    return 0;
}

/**
 * @brief Set bonded device address for directed advertising
 * @param addr Pointer to the bonded device address
 */
void ble_app_set_bonded_device_addr(ble_gap_addr_t *addr)
{
    memcpy(&g_target_device_addr, addr, sizeof(ble_gap_addr_t));
    g_has_target_device = 1;
    LOG_I("Saved bonded device addr: %02x:%02x:%02x:%02x:%02x:%02x (type:%d)",
          addr->addr.addr[5], addr->addr.addr[4], addr->addr.addr[3],
          addr->addr.addr[2], addr->addr.addr[1], addr->addr.addr[0],
          addr->addr_type);
}

/**
 * @brief Start advertising with whitelist filter for specific target device
 * @param device_idx Device index in bonded devices database
 * @note This function sets up whitelist-based advertising to only allow
 * connection from the specified device
 */
void ble_app_start_targeted_advertising(uint8_t device_idx)
{
    // Get bonded device database
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db || device_idx >= MAX_BONDED_DEVICES)
    {
        LOG_E("Invalid device index or database");
        return;
    }

    const bonded_device_t *target_dev = &db->devices[device_idx];
    if (!target_dev->is_valid)
    {
        LOG_E("Target device[%d] is not valid", device_idx);
        return;
    }

    LOG_I("Starting targeted advertising for device[%d]: %s", device_idx,
          target_dev->device_name);
    LOG_I("Target MAC: %02X:%02X:%02X:%02X:%02X:%02X", target_dev->mac_addr[5],
          target_dev->mac_addr[4], target_dev->mac_addr[3],
          target_dev->mac_addr[2], target_dev->mac_addr[1],
          target_dev->mac_addr[0]);

    // Set this device as the active device (this will also set it as target
    // device and save to flash)
    ble_dev_mgr_set_active_device(device_idx);

    // Restart advertising with the new target device
    // If currently connected, we pass false to restart_adv
    // If disconnected, we pass true to restart_adv
    bool restart_adv =
        (SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_DISCONNECTED);
    ble_app_advertising_start(restart_adv, false, false);

    LOG_I("Targeted advertising started for device[%d]", device_idx);
}

/**
 * @brief Clear bonded device address and switch advertising to normal mode
 * @note Uses sibles_advertising_reconfig() to switch mode, no need to manually
 * stop/start
 */
void ble_app_clear_bonded_device(void)
{
    LOG_I("Clearing bonded device...");

    // Delete all bonded devices from connection manager
    conn_manager_get_bonded_dev_t bonded_devs;
    uint8_t ret =
        connection_manager_get_bonded_devices((uint8_t *)&bonded_devs);

    if (ret == 0)
    {
        for (int i = 0; i < MAX_PAIR_DEV; i++)
        {
            if (bonded_devs.priority[i] != MAX_PAIR_DEV)
            {
                LOG_I("Deleting bonded device[%d]", i);
                connection_manager_delete_bond(bonded_devs.peer_addr[i]);
            }
        }
    }

    // Clear cached bonded device
    memset(&g_target_device_addr, 0, sizeof(ble_gap_addr_t));
    g_has_target_device = 0;
}

/********************** Start of Skaiwalk BLE Application
 * *********************************/
#define ble_app_service_uuid                                                   \
    {0x53, 0x49, 0x46, 0x4C, 0x49, 0x42, 0x4C, 0x45,                           \
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define ble_app_tx_uuid                                                        \
    {0x53, 0x49, 0x46, 0x4C, 0x49, 0x42, 0x4C, 0x45,                           \
     0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define ble_app_rx_uuid                                                        \
    {0x53, 0x49, 0x46, 0x4C, 0x49, 0x42, 0x4C, 0x45,                           \
     0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

enum ble_app_att_list
{
    BLE_APP_SVC,
    BLE_APP_TX_CHAR,
    BLE_APP_TX_VALUE,
    BLE_APP_RX_CHAR,
    BLE_APP_RX_VALUE,
    BLE_APP_RX_CCCD,
    BLE_APP_ATT_NB
};

#define SERIAL_UUID_16(x) {((uint8_t)(x & 0xff)), ((uint8_t)(x >> 8))}

struct attm_desc_128 ble_app_att_db[] = {
    [BLE_APP_SVC] = {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE), PERM(RD, ENABLE),
                     0, 0},
    [BLE_APP_TX_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),
                         PERM(RD, ENABLE), 0, 0},
    [BLE_APP_TX_VALUE] = {ble_app_tx_uuid,
                          PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) |
                              PERM(WRITE_COMMAND, ENABLE),
                          PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 256},
    [BLE_APP_RX_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),
                         PERM(RD, ENABLE), 0, 0},
    [BLE_APP_RX_VALUE] = {ble_app_rx_uuid,
                          PERM(RD, ENABLE) | PERM(NTF, ENABLE) |
                              PERM(IND, ENABLE),
                          PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 256},
    [BLE_APP_RX_CCCD] = {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG),
                         PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),
                         PERM(RI, ENABLE), 2},
};

static uint8_t g_ble_app_svc[ATT_UUID_128_LEN] = ble_app_service_uuid;
static sibles_hdl g_sifli_test_ble_test_hdl;

#define ENABLE_BLE_MUTEX 0
static rt_mutex_t ble_api_mutex;
static void ble_api_mutex_init(void)
{
#if ENABLE_BLE_MUTEX
    ble_api_mutex = rt_mutex_create("ble_api_mutex", RT_IPC_FLAG_FIFO);
    RT_ASSERT(ble_api_mutex != RT_NULL);
#endif
}

void ble_api_lock(void)
{
#if ENABLE_BLE_MUTEX
    rt_mutex_take(ble_api_mutex, RT_WAITING_FOREVER);
#endif
}

void ble_api_unlock(void)
{
#if ENABLE_BLE_MUTEX
    rt_mutex_release(ble_api_mutex);
#endif
}

// sifli ble test
static uint8_t *ble_app_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    ble_api_lock();
    LOG_D("ble_app_get_cbk %d", idx);
    switch (idx)
    {
    case BLE_APP_TX_VALUE:
    {
        break;
    }
    }
    *len = 1;
    ble_api_unlock();
    return 0;
}

static uint8_t ble_app_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    ble_api_lock();
    switch (para->idx)
    {
    case BLE_APP_TX_VALUE:
    {
        L1_receive_data_without_crc_check(para->value, para->len);
        break;
    }
    break;
    case BLE_APP_RX_CCCD:
    {
        bool subscribed = *(para->value) == 1;
        LOG_I("BLE_APP_RX_CCCD %d,%d", *(para->value), subscribed);
        SkaiWatchSys.connected_to_phone = subscribed;
        notify_provider.bluetooth_connection();
        if (subscribed)
        {
            app_env_t *env = ble_app_get_env();
            SkaiWatchSys.paired_info.paired_flag = 1;
            SkaiWatchSys.paired_info.paired_num = 1;
            rt_memcpy((void *)SkaiWatchSys.paired_info.newest_addr,
                      (const void *)env->conn_para.peer_addr.addr, 6);
        }
        else
        {
            app_env_t *env = ble_app_get_env();
            SkaiWatchSys.paired_info.paired_flag = 0;
            SkaiWatchSys.paired_info.paired_num = 0;
            rt_memset((void *)SkaiWatchSys.paired_info.newest_addr, 0, 8);
        }
    }
    break;
    default:
        break;
    }
    ble_api_unlock();
    return 0;
}

static void skaiwalk_ble_service_init()
{
    sibles_register_svc_128_t svc;

    svc.att_db = (struct attm_desc_128 *)&ble_app_att_db;
    svc.num_entry = BLE_APP_ATT_NB;
    svc.sec_lvl = PERM(SVC_AUTH, SEC_CON) | PERM(SVC_UUID_LEN, UUID_128) |
                  PERM(SVC_MI, DISABLE);
    svc.uuid = g_ble_app_svc;
    g_sifli_test_ble_test_hdl = sibles_register_svc_128(&svc);
    if (g_sifli_test_ble_test_hdl)
    {
        sibles_register_cbk(g_sifli_test_ble_test_hdl, ble_app_get_cbk,
                            ble_app_set_cbk);
    }
}

static uint8_t phone_device_idx = 0xFF;
void skaiwalk_ble_app_update_conn_param(uint8_t conn_idx, uint16_t inv_max,
                                        uint16_t inv_min, uint16_t timeout)
{
    ble_gap_update_conn_param_t conn_para;
    conn_para.conn_idx = conn_idx;
    conn_para.intv_max = inv_max;
    conn_para.intv_min = inv_min;
    /* value = argv * 1.25 */
    // conn_para.ce_len_max = 0x100;
    // conn_para.ce_len_min = 0x1;
    conn_para.latency = 0;
    conn_para.time_out = timeout;

    LOG_D("Request conn param update: intv_min=%d (%.2f ms), intv_max=%d (%.2f "
          "ms), timeout=%d",
          inv_min, inv_min * 1.25f, inv_max, inv_max * 1.25f, timeout);

    uint8_t ret = ble_gap_update_conn_param(&conn_para);
    LOG_D("ble_gap_update_conn_param result: %d", ret);
}

void skaiwalk_ble_gap_connected_ind(ble_gap_connect_ind_t *ind)
{
    if (phone_device_idx == ind->conn_idx)
    {
        SkaiWatchSys.watch_conn_id = ind->conn_idx;
        SkaiWatchSys.conn_interval = ind->con_interval;
        SkaiWatchSys.conn_latency = ind->con_latency;
        SkaiWatchSys.conn_superv_tout = ind->sup_to;
        SkaiWatchSys.connected_to_phone = false; // is subscribed?
        LOG_D("Connected to %d with interval=%d", ind->conn_idx,
              ind->con_interval);

        notify_provider.bluetooth_connection();
    }
    SkaiWatchSys.gap_conn_state = GAP_CONN_STATE_CONNECTED;
}

void skaiwalk_ble_gap_update_conn_param_ind(
    ble_gap_update_conn_param_ind_t *ind)
{
    LOG_I("Updated connection interval: %d (%.2f ms), latency: %d, timeout: %d",
          ind->con_interval, ind->con_interval * 1.25f, ind->con_latency,
          ind->sup_to);
    SkaiWatchSys.conn_interval = ind->con_interval;
    SkaiWatchSys.conn_latency = ind->con_latency;
    SkaiWatchSys.conn_superv_tout = ind->sup_to;
}

void skaiwalk_disconnected_ind(ble_gap_disconnected_ind_t *ind)
{
    if (phone_device_idx == ind->conn_idx)
    {
        LOG_I("Disconnected from %d", ind->conn_idx);
        SkaiWatchSys.connected_to_phone = false;
        SkaiWatchSys.gap_conn_state = GAP_CONN_STATE_DISCONNECTED;
        notify_provider.bluetooth_connection();

#ifdef USE_BLE_RSSI_CHECK_TIMER
        stop_ble_rssi_checker();
#endif
    }
}

void skaiwalk_ble_mtu_exchange_ind(sibles_mtu_exchange_ind_t *ind)
{
    SkaiWatchSys.watch_mtu = ind->mtu;
}

extern void blebredr_rf_power_set(uint8_t type, int8_t txpwr);
void skaiwatch_ble_set_performance(bool status)
{
    app_env_t *env = ble_app_get_env();
    if (env->is_power_on == false)
    {
        return;
    }
    LOG_I("ble_set_performance %d", status);
    if (status)
    {
        blebredr_rf_power_set(0, 10);
        // Use 12 interval units (15ms) - more acceptable to most phones
        // Original: 6 units (7.5ms) may be rejected by iOS/Android
        skaiwalk_ble_app_update_conn_param(SkaiWatchSys.watch_conn_id, 6, 6,
                                           500);
    }
    else
    {
        blebredr_rf_power_set(0, 0);
        skaiwalk_ble_app_update_conn_param(SkaiWatchSys.watch_conn_id, 24, 24,
                                           500);
    }
}

#define BLE_DATA_SYNC_RETRIES_UNTIL_SUCCESS 1
uint16_t skaiwalk_ble_app_notify(uint8_t *p_data, uint16_t data_length)
{
    if (g_sifli_test_ble_test_hdl)
    {
        ble_api_lock();
        app_env_t *env = ble_app_get_env();
        sibles_value_t value;
        value.hdl = g_sifli_test_ble_test_hdl;
        value.idx = BLE_APP_RX_VALUE;
        value.len = data_length;
        value.value = p_data;

        int ret;
        ret = sibles_write_value(phone_device_idx, &value);

#if BLE_DATA_SYNC_RETRIES_UNTIL_SUCCESS
        if (ret == value.len)
        {
            // LOG_D("send success");
        }
        else if (ret == 0)
        {
            while (1)
            {
                rt_thread_mdelay(50);
                ret = sibles_write_value(phone_device_idx, &value);
                if (ret == value.len)
                {
                    LOG_D("send retry success");
                    break;
                }
            }
        }
#endif
        ble_api_unlock();
        return ret;
    }
    else
    {
        LOG_E("no service");
        return 0;
    }
}
/********************** End of Skaiwalk BLE Application
 * *********************************/
/********************** Start of Skaiwalk BLE Audio Application
 * *********************************/
#define AUDIOPROFILE_SERVICE_UUID                                              \
    {0xF0, 0x00, 0xB0, 0x00, 0x04, 0x51, 0x40, 0x00,                           \
     0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define AUDIOPROFILE_START_UUID                                                \
    {0xF0, 0x00, 0xB0, 0x01, 0x04, 0x51, 0x40, 0x00,                           \
     0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define AUDIOPROFILE_AUDIO_UUID                                                \
    {0xF0, 0x00, 0xB0, 0x02, 0x04, 0x51, 0x40, 0x00,                           \
     0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

enum audio_profile_att_list
{
    AUDIOPROFILE_SVC,
    AUDIOPROFILE_START_CHAR,
    AUDIOPROFILE_START_VAL,
    AUDIOPROFILE_START_CCCD,
    AUDIOPROFILE_AUDIO_CHAR,
    AUDIOPROFILE_AUDIO_VAL,
    AUDIOPROFILE_AUDIO_CCCD,
    AUDIOPROFILE_ATT_NB
};

struct attm_desc_128 audio_profile_att_db[] = {
    [AUDIOPROFILE_SVC] = {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE),
                          PERM(RD, ENABLE), 0, 0},
    [AUDIOPROFILE_START_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),
                                 PERM(RD, ENABLE), 0, 0},
    [AUDIOPROFILE_START_VAL] = {AUDIOPROFILE_START_UUID,
                                PERM(RD, ENABLE) | PERM(NTF, ENABLE),
                                PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1},
    [AUDIOPROFILE_START_CCCD] = {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG),
                                 PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                 PERM(RI, ENABLE), 2},
    [AUDIOPROFILE_AUDIO_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),
                                 PERM(RD, ENABLE), 0, 0},
    [AUDIOPROFILE_AUDIO_VAL] = {AUDIOPROFILE_AUDIO_UUID,
                                PERM(RD, ENABLE) | PERM(NTF, ENABLE),
                                PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE),
                                256},
    [AUDIOPROFILE_AUDIO_CCCD] = {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG),
                                 PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                 PERM(RI, ENABLE), 2},
};

static uint8_t g_audio_profile_svc[ATT_UUID_128_LEN] =
    AUDIOPROFILE_SERVICE_UUID;
static sibles_hdl g_audio_profile_hdl;

static uint8_t *audio_profile_get_cbk(uint8_t conn_idx, uint8_t idx,
                                      uint16_t *len)
{
    LOG_D("audio_profile_get_cbk %d", idx);
    *len = 0;
    return NULL;
}

static uint8_t audio_profile_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    app_env_t *env = ble_app_get_env();
    switch (para->idx)
    {
    case AUDIOPROFILE_START_CCCD:
    {
        LOG_I("AUDIOPROFILE_START_CCCD %d", *(para->value));
    }
    break;
    case AUDIOPROFILE_AUDIO_CCCD:
    {
        LOG_I("AUDIOPROFILE_CCCD %d", *(para->value));
        env->data.is_audio_subscribed = *(para->value);
    }
    break;
    default:
        break;
    }
    return 0;
}

static void audio_profile_service_init()
{
    sibles_register_svc_128_t svc;

    svc.att_db = (struct attm_desc_128 *)&audio_profile_att_db;
    svc.num_entry = AUDIOPROFILE_ATT_NB;
    svc.sec_lvl = PERM(SVC_AUTH, SEC_CON) | PERM(SVC_UUID_LEN, UUID_128) |
                  PERM(SVC_MI, DISABLE);
    svc.uuid = g_audio_profile_svc;
    g_audio_profile_hdl = sibles_register_svc_128(&svc);
    if (g_audio_profile_hdl)
    {
        sibles_register_cbk(g_audio_profile_hdl, audio_profile_get_cbk,
                            audio_profile_set_cbk);
    }
}

#define BLE_AUDIO_RETRY_TIMES 0
int audio_profile_send_voice_data(uint8_t *voice_data, uint16_t voice_data_len)
{
    if (g_audio_profile_hdl)
    {
        app_env_t *env = ble_app_get_env();
        if (env->data.is_audio_subscribed == 0)
        {
            return 0;
        }
        ble_api_lock();
        sibles_value_t value;
        value.hdl = g_audio_profile_hdl;
        value.idx = AUDIOPROFILE_AUDIO_VAL;
        value.len = voice_data_len;
        value.value = voice_data;
        int ret = sibles_write_value(phone_device_idx, &value);
#if BLE_AUDIO_RETRY_TIMES
        if (ret == 0 && !is_signal_bad())
        {
            int retry = MAX_RETRIES;
            while (retry > 0)
            {
                retry--;
                rt_thread_mdelay(50);
                ret = sibles_write_value(phone_device_idx, &value);
                if (ret == voice_data_len)
                {
                    break;
                }
            }
        }
        else
        {
            // LOG_D("send success %d", ret);
        }
#endif

        if (is_signal_bad())
        {
            rt_thread_mdelay(50);
        }

        ble_api_unlock();
        return ret;
    }
    else
    {
        LOG_E("no service");
        return 0;
    }
}
/********************** End of Skaiwalk BLE Audio Application
 * *********************************/

void le_disconnect(uint8_t conn_idx)
{
    ble_gap_disconnect_t disconn;
    disconn.conn_idx = conn_idx;
    disconn.reason = 0x13;
    ble_gap_disconnect(&disconn);
}

/* Enable advertise via advertising service. */
#if ENABLE_BG_ADV
static uint8_t ble_app_background_advertising_event(uint8_t event,
                                                    void *context, void *data)
{
    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("Broadcast ADV start resutl %d, mode %d\r", evt->status,
              evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("Broadcast ADV stopped reason %d, mode %d\r", evt->reason,
              evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}

static void ble_app_bg_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    char local_name[] = "SKAIWALK_BG_INFO";
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;

    para.own_addr_type = GAPM_GEN_NON_RSLV_ADDR;
    para.config.adv_mode = SIBLES_ADV_BROADCAST_MODE;
    /* Keep advertising till disable it or connected. */
    para.config.mode_config.broadcast_config.duration = 0x0;
    para.config.mode_config.broadcast_config.interval = 0x140; // 140
    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    para.config.is_auto_restart = 1;
    /* Scan rsp data is same as advertising data. */
    para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed .*/
    para.adv_data.completed_name =
        rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.adv_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.adv_data.completed_name->name, local_name,
              para.adv_data.completed_name->name_len);

    /* Prepare manufacturer filed .*/
    para.adv_data.manufacturer_data =
        rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) +
                  sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data,
              manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_background_advertising_event;

    ret = sibles_advertising_init(g_app_advertising_bg_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_bg_context);
    }

    rt_free(para.adv_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}
#endif

#define NVDS_PATTERN 0x4E564453
#if defined(SOC_SF32LB58X)
    #define NVDS_BUFF_START 0x204FFD00
    #define NVDS_BUFF_SIZE 512
#elif defined(SOC_SF32LB56X)
    #define NVDS_BUFF_START 0x2041FD00
    #define NVDS_BUFF_SIZE 512
#elif defined(SOC_SF32LB52X)
    #define NVDS_BUFF_START 0x2040FE00
    #define NVDS_BUFF_SIZE 512
#else
    #define NVDS_BUFF_START 0
    #define NVDS_BUFF_SIZE 0
#endif

typedef struct
{
    uint32_t pattern;
    uint16_t used_mem;
    uint16_t writting;
} skaiwalk_nvds_mem_init_t;

void bt_stack_update_flash(void)
{
    skaiwalk_nvds_mem_init_t *ptr1 =
        (skaiwalk_nvds_mem_init_t *)NVDS_BUFF_START;
    if (ptr1->pattern == NVDS_PATTERN && ptr1->writting == 0)
    {
        uint8_t *buf_addr = (uint8_t *)(ptr1 + 1);
        rt_kprintf("used mem len:%d\r\n", ptr1->used_mem);

        // 寫入偏移量 8 開始的資料: 03 25 AA AA AA AA
        uint8_t write_data[] = {0xE0, 0x63, 0x66, 0x20, 0x95, 0x01};
        memcpy(&buf_addr[8], write_data, sizeof(write_data));
        sifli_nvds_write(SIFLI_NVDS_TYPE_STACK, ptr1->used_mem,
                         (uint8_t *)(ptr1 + 1));
        rt_kprintf("Written data at offset 8: ");
        for (int i = 8; i < 14 && i < ptr1->used_mem; i++)
        {
            rt_kprintf("%02X ", buf_addr[i]);
        }
        rt_kprintf("\r\n");

        LOG_HEX("nvds_data", 16, (uint8_t *)(ptr1 + 1), ptr1->used_mem);
    }
}

static bool bluetooth_broadcasting_status = false;
bool get_bluetooth_broadcasting_status(void)
{
    return bluetooth_broadcasting_status;
}
extern int check_main_phone_counterpart_connection(void);
void ble_app_advertising_start(bool restart_adv, bool mouse_mode,
                               bool pairing_mode);

static lv_timer_t *main_phone_check_timer = NULL;

void ble_dev_mgr_stop_main_phone_check_timer(void);
static void
check_main_phone_counterpart_connection_timer_callback(lv_timer_t *timer)
{
    if (bluetooth_broadcasting_status)
    {
        // Advertising not started yet, skip check
        // LOG_I("Advertising in progress, skipping main phone check");
        return;
    }
    int device_idx = check_main_phone_counterpart_connection();
    if (device_idx == -1)
    {
        LOG_I(
            "Main phone counterpart not connected, restarting advertising...");
        ble_app_advertising_start(true, false, false);
    }
    else
    {
        // LOG_I("Main phone counterpart is connected, no action needed");
        ble_dev_mgr_stop_main_phone_check_timer();
    }
}

void ble_dev_mgr_start_main_phone_check_timer(uint32_t interval_ms)
{
    // Stop existing timer if any
    if (main_phone_check_timer)
    {
        lv_timer_del(main_phone_check_timer);
        main_phone_check_timer = NULL;
    }
    // LOG_D("Starting main phone counterpart check timer, interval: %d ms",
    //       interval_ms);
    main_phone_check_timer =
        lv_timer_create(check_main_phone_counterpart_connection_timer_callback,
                        interval_ms, NULL);
}

void ble_dev_mgr_stop_main_phone_check_timer(void)
{
    if (main_phone_check_timer)
    {
        lv_timer_del(main_phone_check_timer);
        main_phone_check_timer = NULL;
    }
}

static uint8_t ble_app_advertising_event(uint8_t event, void *context,
                                         void *data)
{
    app_env_t *env = ble_app_get_env();

    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
#if ENABLE_BG_ADV
        if (!env->is_bg_adv_on)
        {
            env->is_bg_adv_on = 1;
            ble_app_bg_advertising_start();
        }
#endif
        bluetooth_broadcasting_status = true;
        if (!is_sleep_mode())
        {
            refresh_ble_mode_btn();
        }
        LOG_I("ADV start resutl %d, mode %d\r", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        bluetooth_broadcasting_status = false;
        if (!is_sleep_mode())
        {
            refresh_ble_mode_btn();
        }
        LOG_I("ADV stopped reason %d, mode %d\r", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}

uint8_t sibles_advertising_disc_mode_get()
{
    return GAPM_ADV_MODE_GEN_DISC;
}

#define USING_ADV_MANUFACTURER_DATA 0

#define DEFAULT_LOCAL_NAME "SkaiWatch"
#define GAP_GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE 192
#define GAP_GATT_APPEARANCE_MOUSE 962
#define ENABLE_ADV_SERVICE_UUID 1
void ble_app_advertising_start(bool restart_adv, bool mouse_mode,
                               bool pairing_mode)
{
    LOG_I("Starting advertising...");
    app_env_t *env = ble_app_get_env();
    if (env->is_power_on == false)
    {
        return;
    }
    sibles_advertising_para_t para = {0};
    uint8_t ret;
    // Local name
    // char local_name[] = DEFAULT_LOCAL_NAME;
    char local_name[16] = {0};

#if USING_ADV_MANUFACTURER_DATA
    // Manufaturer data
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;
#endif
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
        rt_snprintf(local_name, 16, "SkaiWatch %x %x", addr.addr[1],
                    addr.addr[0]);
    else
        memcpy(local_name, DEFAULT_LOCAL_NAME, sizeof(DEFAULT_LOCAL_NAME));

    // Set name to ble service
    ble_gap_dev_name_t *dev_name =
        malloc(sizeof(ble_gap_dev_name_t) + sizeof(local_name));
    dev_name->len = sizeof(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);
    ble_gap_set_dev_name(dev_name);
    free(dev_name);

    // Set advertising address as static
    para.own_addr_type = GAPM_STATIC_ADDR;
    // Check if we have a bonded device for targeted advertising
    ble_gap_addr_t bonded_addr;
    bool has_bonded_device =
        ble_app_get_bonded_device_addr(&bonded_addr) && !pairing_mode;

    if (has_bonded_device)
    {
        para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
        para.config.mode_config.conn_config.duration = 0x0;
        para.config.mode_config.conn_config.interval = 0x140;
        para.config.max_tx_pwr = 0x7F;
    }
    else
    {
        // No bonded device or in pairing mode - use general advertising with
        // background mode
        para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
        para.config.mode_config.conn_config.duration = 0x0;
        para.config.mode_config.conn_config.interval = 0x140;
        para.config.max_tx_pwr = 0x7F;
    }
    // Enable restart after disconnected
    para.config.is_auto_restart = 1;
    // adv data and rsp data use same data
    // para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed. Due to name is too long to put adv data, put it to
     * rsp data.*/
    para.adv_data.completed_name =
        rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.adv_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.adv_data.completed_name->name, local_name,
              para.adv_data.completed_name->name_len);
#ifdef GAP_GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE
    /* Prepare Appearance data filed .*/
    {
        uint16_t appearance;
        if (mouse_mode)
        {
            appearance = GAP_GATT_APPEARANCE_MOUSE;
            // appearance = GAP_GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE;
        }
        else
        {
            // appearance = GAP_GATT_APPEARANCE_MOUSE;
            appearance = GAP_GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE;
        }
        LOG_D("Appearance: %d", appearance);
        para.adv_data.appearance = rt_malloc(sizeof(uint16_t));
        if (para.adv_data.appearance != NULL)
        {
            memcpy(para.adv_data.appearance, &appearance, sizeof(uint16_t));
        }
    }
#endif

#if ENABLE_ADV_SERVICE_UUID
    /* Prepare service data filed .*/
    {
        uint8_t uuid_count = 1;
        sibles_adv_uuid_t adv_uuid = {0};
        para.adv_data.completed_uuid = rt_malloc(
            sizeof(sibles_adv_type_srv_uuid_t) + sizeof(adv_uuid) * uuid_count);
        para.adv_data.completed_uuid->count = uuid_count;

        // HID service
        uint16_t uuid_hid = ATT_SVC_HID;
        // set ATT_SVC_HID to uuid_hids.uuid.uuid_16;
        adv_uuid.uuid_len = 2;
        memcpy(adv_uuid.uuid.uuid_16, &uuid_hid, adv_uuid.uuid_len);
        rt_memcpy(para.adv_data.completed_uuid->uuid_list, &adv_uuid,
                  sizeof(adv_uuid));
    }
#endif

#if USING_ADV_MANUFACTURER_DATA
    para.adv_data.manufacturer_data =
        rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) +
                  sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data,
              manu_additnal_data, sizeof(manu_additnal_data));
#endif
    para.evt_handler = ble_app_advertising_event;

    uint8_t curr_conn_idx = g_app_advertising_context->conn_idx;

    if (restart_adv)
    {
        // stop advertising first
        sibles_advertising_stop(g_app_advertising_context);
        rt_thread_mdelay(100); // Increased delay for proper cleanup
        // Delete existing advertising configuration before reinit
        // This is required to allow switching between advertising modes or
        // reinit with same mode
        sibles_advertising_delete(g_app_advertising_context);
        rt_thread_mdelay(100); // Increased delay for proper cleanup
    }

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (restart_adv)
    {
        sibles_advertising_reconfig(g_app_advertising_context, &para.config);
    }
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
    }

    g_app_advertising_context->conn_idx = curr_conn_idx;

    rt_free(para.adv_data.completed_name);
#if ENABLE_ADV_SERVICE_UUID
    rt_free(para.adv_data.completed_uuid);
#endif

#if USING_ADV_MANUFACTURER_DATA
    rt_free(para.adv_data.manufacturer_data);
#endif

#ifdef GAP_GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE
    rt_free(para.adv_data.appearance);
#endif
}

static uint8_t g_bass_app_bas_lvl = 100;
uint8_t bass_app_callback(uint8_t conn_idx, uint8_t event)
{
    uint8_t ret = 0;
    switch (event)
    {
    case BLE_BASS_GET_BATTERY_LVL:
    {
        ret = g_bass_app_bas_lvl;
        break;
    }
    default:
        break;
    }
    LOG_I("bass callback type %d, ret %d\r", event, ret);
    return ret;
}

void generate_random_public_address(uint8_t device_id)
{
    bd_addr_t addr;
    bt_mac_addr_generate_via_uid(&addr);
    do
    {
        sifli_nvds_write_tag_t *update_tag =
            malloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_BD_ADDRESS);
        if (update_tag == NULL)
            break;

        // Generate random values for the BLE address
        for (int i = 0; i < NVDS_STACK_LEN_BD_ADDRESS - 3; i++)
        {
            update_tag->value.value[i] = rand() % 256;
        }

        update_tag->is_flush = 1;
        update_tag->type = BLE_UPDATE_ALWAYS;
        update_tag->value.tag = NVDS_STACK_TAG_BD_ADDRESS;
        update_tag->value.len = NVDS_STACK_LEN_BD_ADDRESS;

        sifli_nvds_write_tag_value(update_tag);
        free(update_tag);
        LOG_D("update nvds address success");
    } while (0);

    ble_get_public_address(&addr);
    LOG_I("BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X", addr.addr[5], addr.addr[4],
          addr.addr[3], addr.addr[2], addr.addr[1], addr.addr[0]);
    LOG_I("Resetting BLE stack to apply new address...");
}

uint8_t g_diss_conn_idx;
uint16_t g_th_total_cnt;
uint16_t g_th_interval;
uint16_t g_th_packet_size;
void ble_app_entry(void *param)
{
    app_env_t *env = ble_app_get_env();
    ble_api_mutex_init();

    // Start advertising start timeout timer
    // start_ble_adv_start_timer();

    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            // Initialize device manager
            ble_dev_mgr_init();

            ble_hid_service_init(&env->hid_data);

            skaiwalk_ble_service_init();
            audio_profile_service_init();
#ifdef USING_BLE_SERIAL
            ble_serial_tran_init();
#endif
            ble_bass_init(bass_app_callback, g_bass_app_bas_lvl);

            env->is_power_on = 1;
            env->conn_para.mtu = 23; /* Default value. */
            // ble_app_advertising_start(false, false, false);
            ble_dev_mgr_start_main_phone_check_timer(5000);
        }
#ifdef USING_BLE_SERIAL
        else if (value == 0xFF1F)
        {
            uint16_t total_count = g_th_total_cnt;
            LOG_I("receive throughput test!\r");
            uint8_t *array = rt_malloc(g_th_packet_size);
            while (total_count--)
            {
                ble_serial_tran_data_t rsp_header;
                rt_memset(array, (uint8_t)(total_count & 0xFF),
                          g_th_packet_size);
                rsp_header.handle = g_diss_conn_idx;
                rsp_header.cate_id = BLE_APP_CATEID;
                rsp_header.len = g_th_packet_size;
                rsp_header.data = array;
                ret = ble_serial_tran_send_data(&rsp_header);
                while (ret == 0)
                {
                    ret = ble_serial_tran_send_data(&rsp_header);
                }
                // LOG_E("Send failed");
                rt_thread_mdelay(g_th_interval);
            }
            rt_free(array);
        }
#endif
    }
}

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
    #include "dfs_file.h"
    #include "dfs_posix.h"
    #include "drv_flash.h"
    #define NAND_MTD_NAME "root"
int mnt_init(void)
{
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000),
                         FS_REGION_START_ADDR -
                             (FS_REGION_START_ADDR & (0xFC000000)),
                         FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif

int ble_app_init(void)
{
    rt_thread_t tid;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("ble_app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
    tid = rt_thread_create("ble_app", ble_app_entry, NULL, 4096, 15,
                           RT_THREAD_TICK_DEFAULT);
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(ble_app_init);

uint8_t peer_addr[6];
void set_main_phonepeer_addr(void)
{
    app_env_t *env = ble_app_get_env();
    rt_memcpy(peer_addr, env->conn_para.peer_addr.addr, 6);
    phone_device_idx = env->conn_idx;
    LOG_D("phone_device_idx: %d", phone_device_idx);
}

uint8_t get_main_phonepeer_conn_idx(void)
{
    return phone_device_idx;
}

void get_main_phonepeer_addr(uint8_t *addr)
{
    rt_memcpy(addr, peer_addr, 6);
}

int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len,
                          uint32_t context)
{
    ble_api_lock();
    app_env_t *env = ble_app_get_env();
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        if (env->mb_handle)
            rt_mb_send(env->mb_handle, BLE_POWER_ON_IND);
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->conn_para.conn_interval = ind->con_interval;
        env->conn_para.peer_addr_type = ind->peer_addr_type;
        env->conn_para.peer_addr = ind->peer_addr;
        skaiwalk_ble_gap_connected_ind(ind);
        if (ind->role == 0)
            LOG_E("Peripheral should be slave!!!");

        LOG_I(
            "Peer device(%x-%x-%x-%x-%x-%x) connected",
            env->conn_para.peer_addr.addr[5], env->conn_para.peer_addr.addr[4],
            env->conn_para.peer_addr.addr[3], env->conn_para.peer_addr.addr[2],
            env->conn_para.peer_addr.addr[1], env->conn_para.peer_addr.addr[0]);

        // Update device manager with connection
        int dev_idx = ble_dev_mgr_find_device(env->conn_para.peer_addr.addr);
        if (dev_idx < 0)
        {
            // New device, add it (assume phone type for now)
            dev_idx = ble_dev_mgr_add_device(
                env->conn_para.peer_addr.addr, env->conn_para.peer_addr_type,
                NULL, // Name will be discovered later
                DEVICE_TYPE_PHONE);
        }
        if (dev_idx >= 0)
        {
            ble_dev_mgr_update_connection(dev_idx, ind->conn_idx);
            // Set as active device if it's the only one connected
            if (ble_dev_mgr_get_active_device() < 0)
            {
                ble_dev_mgr_set_active_device(dev_idx);
            }

            // Read remote device name via GATT
            dev_name_start_search(ind->conn_idx, dev_idx);
        }

        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind =
            (ble_gap_update_conn_param_ind_t *)data;
        env->conn_para.conn_interval = ind->con_interval;
        skaiwalk_ble_gap_update_conn_param_ind(ind);
        break;
    }
    case SIBLES_REMOTE_CONNECTED_IND:
    {
        // sibles_remote_connected_ind_t *ind = (sibles_remote_connected_ind_t
        // *)data;
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->conn_para.mtu = ind->mtu;
        skaiwalk_ble_mtu_exchange_ind(ind);
        LOG_I("Exchanged MTU size: %d", ind->mtu);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        skaiwalk_disconnected_ind(ind);
        env->data.is_audio_subscribed = 0;

        // Reset HID configuration on disconnect
        ble_hid_reset_on_disconnect();

        LOG_I("BLE_GAP_DISCONNECTED_IND reason:%d", ind->reason);

        // Update device manager with disconnection
        const bonded_devices_db_t *db = ble_dev_mgr_get_database();
        if (db)
        {
            for (int i = 0; i < MAX_BONDED_DEVICES; i++)
            {
                if (db->devices[i].is_valid &&
                    db->devices[i].conn_idx == ind->conn_idx)
                {
                    ble_dev_mgr_update_connection(i,
                                                  0xFF); // Mark as disconnected
                    break;
                }
            }
        }

        break;
    }
    case BLE_GAP_REMOTE_RSSI_IND:
    {
        ble_gap_remote_rssi_ind_t *ind = (ble_gap_remote_rssi_ind_t *)data;
        LOG_D("BLE_GAP_REMOTE_RSSI_IND %d", ind->rssi);
        set_signal_bad(ind->rssi < -80);
        break;
    }
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;

        // Only handle if we initiated the search for GAP service
        if (!g_name_reader.busy)
            break;

        uint16_t gap_svc_uuid = ATT_UUID_16(0x1800);
        if (rsp->search_svc_len != ATT_UUID_16_LEN ||
            memcmp(rsp->search_uuid, &gap_svc_uuid, ATT_UUID_16_LEN) != 0)
            break;

        if (rsp->result != 0 || !rsp->svc)
        {
            LOG_W("GAP service search failed, result=%d", rsp->result);
            g_name_reader.busy = 0;
            break;
        }

        // Find Device Name (0x2A00) and Appearance (0x2A01) characteristics
        uint16_t dev_name_uuid = ATT_UUID_16(0x2A00);
        uint16_t appearance_uuid = ATT_UUID_16(0x2A01);
        sibles_svc_search_char_t *chara =
            (sibles_svc_search_char_t *)rsp->svc->att_db;
        bool found = false;
        g_name_reader.appearance_hdl = 0;

        for (uint8_t i = 0; i < rsp->svc->char_count; i++)
        {
            if (chara->uuid_len == ATT_UUID_16_LEN)
            {
                if (memcmp(chara->uuid, &dev_name_uuid, ATT_UUID_16_LEN) == 0)
                {
                    g_name_reader.name_value_hdl = chara->pointer_hdl;
                    g_name_reader.svc_start_hdl = rsp->svc->hdl_start;
                    g_name_reader.svc_end_hdl = rsp->svc->hdl_end;
                    found = true;
                    LOG_I("Found Device Name char, value_hdl=0x%x",
                          chara->pointer_hdl);
                }
                else if (memcmp(chara->uuid, &appearance_uuid, ATT_UUID_16_LEN) == 0)
                {
                    g_name_reader.appearance_hdl = chara->pointer_hdl;
                    LOG_I("Found Appearance char, value_hdl=0x%x",
                          chara->pointer_hdl);
                }
            }
            uint16_t offset = sizeof(sibles_svc_search_char_t) +
                              chara->desc_count *
                                  sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }

        if (found)
        {
            g_name_reader.remote_handle = sibles_register_remote_svc(
                rsp->conn_idx, rsp->svc->hdl_start, rsp->svc->hdl_end,
                dev_name_gattc_callback);
            LOG_I("Registered GAP service client, remote_handle=%d",
                  g_name_reader.remote_handle);
        }
        else
        {
            LOG_W("Device Name characteristic not found");
            g_name_reader.busy = 0;
        }
        break;
    }
    default:
        break;
    }
    ble_api_unlock();
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);

#ifdef SF32LB52X
static rt_timer_t rc10k_time_handle;
void rc10k_timeout_handler(void *parameter)
{
    if (!HAL_RTC_LXT_ENABLED())
    {
        HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
    }
    else
    {
        rt_timer_stop(rc10k_time_handle);
    }
}
#endif

#ifdef RT_USING_WDT
/**
 * @brief This function is invoked in WDT_IRQHandler.
 *        It can be overidden to do some work when WDT1 timeout occured.
 *        Ex. to store exception context and reboot immediately.
 */
void wdt_store_exception_information(void)
{
    rt_kprintf("HCPU WDT1 timeout occurs.\n");
    extern void drv_reboot(void);
    drv_reboot();
    return;
}

/**
 * @brief WDT ON/OFF
 * @param en 0: OFF 1: ON
 */
static void watchdog_set_status(uint8_t en)
{
    #ifdef RT_USING_WDT
    /* Set wdt status 0. */
    rt_hw_watchdog_set_status(en);
    /* Avoid repeat set hook. */
    rt_hw_watchdog_hook(0);
    if (!en)
    {
        /* Stop wdt. */
        rt_hw_watchdog_deinit();
    }
    else
    {
        /* Set hook for watchdog petting. */
        rt_hw_watchdog_hook(1);
    }
    #endif
}
#endif

int main(void)
{
#ifdef SF32LB52X
    rc10k_time_handle =
        rt_timer_create("rc10", rc10k_timeout_handler, NULL,
                        rt_tick_from_millisecond(15 * 1000),
                        RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_start(rc10k_time_handle);
#endif

#ifdef RT_USING_WDT
    /* Diable WDT. */
    watchdog_set_status(0);
    rt_kprintf("HCPU WDT off.\n");
    /* Enable WDT. */
    watchdog_set_status(1);
    rt_kprintf("HCPU WDT on.(timeout: %d seconds)\n", WDT_TIMEOUT);
    /* Cancel feeding the dog in idle thread. */
    // rt_hw_watchdog_hook(0);
    // rt_kprintf("Unregister idle hook.\n");
#endif /* RT_USING_WDT */

    return RT_EOK;
}
