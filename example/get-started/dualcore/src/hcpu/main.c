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

/* MAC address printf helper. bd_addr_t / bonded_device_t::mac_addr both store
   the address byte-reversed (LSB at index 0); print high-to-low so the output
   matches the canonical XX:XX:XX:XX:XX:XX form humans expect. */
#define MAC_FMT      "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ARG(a)   (a)[5], (a)[4], (a)[3], (a)[2], (a)[1], (a)[0]

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
extern uint8_t ble_gap_get_remote_rssi(ble_gap_get_rssi_t *rssi);
static rt_timer_t rssi_timer = NULL;
static void rssi_timer_callback(void *parameter)
{
    ble_gap_get_rssi_t rssi;
    rssi.conn_idx = g_app_env.conn_idx;
    /* Result is delivered asynchronously via BLE_GAP_REMOTE_RSSI_IND; we only
       need to kick off the request here. */
    ble_gap_get_remote_rssi(&rssi);
}
/* Period is in milliseconds. Two main callers:
   - The MTU-exchange path starts it at 10 s for TX-power control (TPC).
   - V2T starts it at 1 s when the user is on the voice screen, for the
     signal-strength UI indicator. */
void start_ble_rssi_checker(uint32_t period_ms)
{
    rt_tick_t ticks = rt_tick_from_millisecond(period_ms);
    if (!rssi_timer)
    {
        rssi_timer = rt_timer_create("rssi_timer", rssi_timer_callback, NULL,
                                     ticks, RT_TIMER_FLAG_PERIODIC);
    }
    else
    {
        rt_timer_stop(rssi_timer);
        rt_timer_control(rssi_timer, RT_TIMER_CTRL_SET_TIME, &ticks);
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

extern void blebredr_rf_power_set(uint8_t type, int8_t txpwr);

/* ===== TX Power Control (TPC) ==========================================
   Two-tier dynamic TX power adjustment driven by remote RSSI:
     - Tier 0 (LOW):  0 dBm   — close to phone, save power
     - Tier 1 (MID):  +3 dBm  — moderate distance, gentle boost

   +10 dBm is intentionally NOT in this list — that level is reserved for
   fast profile (V2T / file / DFU) where the watch is doing a real-time
   bulk operation and reliability matters more than power. For slow
   profile we prefer to give up the link at the edge of range and let
   the user reconnect when closer, rather than burn current to maintain
   marginal coverage.

   Only active while in slow profile (idle). Fast profile hard-pins
   +10 dBm and pauses TPC. Hysteresis prevents ping-pong:
     0 → 1  when avg RSSI < -75 dBm
     1 → 0  when avg RSSI > -55 dBm (20 dB margin from upgrade)
   Plus a 30 s minimum interval between any two changes. */

#define TPC_UPGRADE_LOW_TO_MID    -75
#define TPC_DOWNGRADE_MID_TO_LOW  -55
#define TPC_MIN_CHANGE_INTERVAL_MS 30000

#define TPC_TIER_COUNT 2
static const int8_t TPC_TIER_DBM[TPC_TIER_COUNT] = {0, 3};
static int8_t   g_tpc_rssi_avg = -50;     /* EMA, dBm */
static uint8_t  g_tpc_rssi_warmup = 0;    /* samples since reset */
static uint8_t  g_tpc_tier = 0;           /* current tier index */
static rt_tick_t g_tpc_last_change_tick = 0;
static bool     g_ble_perf_is_fast = false; /* true while FAST or ULTRA — TPC yields */
static ble_perf_level_t g_ble_perf_level = BLE_PERF_SLOW; /* current applied level, for dedupe */

static void ble_tpc_reset(void)
{
    g_tpc_rssi_avg = -50;
    g_tpc_rssi_warmup = 0;
    g_tpc_tier = 0;
    g_tpc_last_change_tick = 0;
}

static void ble_tpc_on_rssi_sample(int8_t rssi)
{
    /* Bootstrap the EMA on the first few samples instead of dragging the
       seed value into the average for 30 s+. */
    if (g_tpc_rssi_warmup < 5)
    {
        g_tpc_rssi_avg = rssi;
        g_tpc_rssi_warmup++;
        return;
    }
    /* EMA: alpha = 0.3, integer math with rounding. */
    g_tpc_rssi_avg = (int8_t)((g_tpc_rssi_avg * 7 + (int)rssi * 3 + 5) / 10);

    /* TPC only owns TX power in slow profile. Fast profile callers are
       intentionally aggressive — don't fight them. */
    if (g_ble_perf_is_fast)
    {
        return;
    }
    /* Rate-limit changes to avoid oscillation when RSSI sits near a band edge. */
    rt_tick_t now = rt_tick_get();
    if ((now - g_tpc_last_change_tick) <
        rt_tick_from_millisecond(TPC_MIN_CHANGE_INTERVAL_MS))
    {
        return;
    }

    uint8_t target = g_tpc_tier;
    if (g_tpc_tier == 0 && g_tpc_rssi_avg < TPC_UPGRADE_LOW_TO_MID)
        target = 1;
    else if (g_tpc_tier == 1 && g_tpc_rssi_avg > TPC_DOWNGRADE_MID_TO_LOW)
        target = 0;

    if (target != g_tpc_tier)
    {
        blebredr_rf_power_set(0, TPC_TIER_DBM[target]);
        LOG_I("TPC: tier %d → %d (avg RSSI=%d dBm)",
              g_tpc_tier, target, g_tpc_rssi_avg);
        g_tpc_tier = target;
        g_tpc_last_change_tick = now;
    }
}

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
        LOG_I("Using cached bonded device addr: " MAC_FMT " (type:%d)",
              MAC_ARG(addr->addr.addr), addr->addr_type);
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

            LOG_I("Found active device[%d] addr: " MAC_FMT " (type:%d)",
                  active_idx, MAC_ARG(addr->addr.addr), addr->addr_type);
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

            LOG_I("Found bonded device[%d] addr: " MAC_FMT " (type:%d)",
                  i, MAC_ARG(addr->addr.addr), addr->addr_type);
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
    LOG_I("Saved bonded device addr: " MAC_FMT " (type:%d)",
          MAC_ARG(addr->addr.addr), addr->addr_type);
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
    LOG_I("Target MAC: " MAC_FMT, MAC_ARG(target_dev->mac_addr));

    // Set this device as the active device (this will also set it as target
    // device and save to flash)
    ble_dev_mgr_set_active_device(device_idx);

    // Restart advertising with the new target device
    ble_app_advertising_start(false, false);

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
                          PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE),
                          BLE_APP_CHAR_MAX_LEN},
    [BLE_APP_RX_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),
                         PERM(RD, ENABLE), 0, 0},
    [BLE_APP_RX_VALUE] = {ble_app_rx_uuid,
                          PERM(RD, ENABLE) | PERM(NTF, ENABLE) |
                              PERM(IND, ENABLE),
                          PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE),
                          BLE_APP_CHAR_MAX_LEN},
    [BLE_APP_RX_CCCD] = {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG),
                         PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),
                         PERM(RI, ENABLE), 2},
};

static uint8_t g_ble_app_svc[ATT_UUID_128_LEN] = ble_app_service_uuid;
static sibles_hdl g_sifli_test_ble_test_hdl;

// sifli ble test
static uint8_t *ble_app_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    LOG_D("ble_app_get_cbk %d", idx);
    switch (idx)
    {
    case BLE_APP_TX_VALUE:
    {
        break;
    }
    }
    *len = 1;
    return 0;
}

static uint8_t ble_app_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
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
                                        uint16_t inv_min, uint16_t latency,
                                        uint16_t timeout)
{
    ble_gap_update_conn_param_t conn_para;
    conn_para.conn_idx = conn_idx;
    conn_para.intv_max = inv_max;
    conn_para.intv_min = inv_min;
    /* value = argv * 1.25 */
    // conn_para.ce_len_max = 0x100;
    // conn_para.ce_len_min = 0x1;
    conn_para.latency = latency;
    conn_para.time_out = timeout;

    LOG_D("Request conn param update: intv_min=%d (%.2f ms), intv_max=%d (%.2f "
          "ms), latency=%d, timeout=%d",
          inv_min, inv_min * 1.25f, inv_max, inv_max * 1.25f, latency, timeout);

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
        /* Wipe TPC state so the next connection starts at tier 0 (0 dBm).
           Otherwise an old "user-walked-far" tier could persist across a
           reconnect, burning power before the first RSSI sample arrives. */
        ble_tpc_reset();
        g_ble_perf_is_fast = false;
        g_ble_perf_level = BLE_PERF_SLOW;
    }
}

void skaiwalk_ble_mtu_exchange_ind(sibles_mtu_exchange_ind_t *ind)
{
    SkaiWatchSys.watch_mtu = ind->mtu;

    /* Upgrade link PHY to LE 2M now that MTU exchange has completed —
       initial negotiations are done, link is stable. 2M PHY halves the
       radio-active time per packet (same data, half the airtime) for
       roughly 50% less BLE radio energy. TX power is unchanged so the
       link budget loses ~3 dB sensitivity vs 1M; in normal proximity
       use this is invisible, only edge-of-range scenarios may notice.
       If the phone doesn't support 2M PHY the request is ignored and
       the link stays at 1M — no functional impact. */
    ble_gap_update_phy_t phy = {
        .conn_idx = ind->conn_idx,
        .tx_phy   = GAP_PHY_LE_2MBPS,
        .rx_phy   = GAP_PHY_LE_2MBPS,
        .phy_opt  = 0,
    };
    uint8_t ret = ble_gap_update_phy(&phy);
    LOG_I("Request LE 2M PHY for conn %d: ret=%d", ind->conn_idx, ret);

    /* Kick off TX-power control: sample remote RSSI every 10 s while
       connected, feed ble_tpc_on_rssi_sample() to ramp TX power
       between tiers (0 / +3 / +10 dBm). Cheap (one BLE controller read
       per sample) and only runs while the watch is connected. */
    ble_tpc_reset();
    start_ble_rssi_checker(10000);
}

/* blebredr_rf_power_set extern is declared at the top of this file (near
   the TPC block) so static helpers there can call it. */
void skaiwatch_ble_set_performance(ble_perf_level_t level)
{
    app_env_t *env = ble_app_get_env();
    if (env->is_power_on == false)
    {
        return;
    }
    /* Dedupe: each call re-issues a LL_CONNECTION_PARAM_REQ over the air
       even when the params don't change, which briefly stalls data flow.
       Callers can invoke this freely (e.g. start of every received file)
       and we collapse same-level requests into a no-op here. */
    if (level == g_ble_perf_level)
    {
        return;
    }
    LOG_I("ble_set_performance %d -> %d", g_ble_perf_level, level);
    g_ble_perf_level = level;
    g_ble_perf_is_fast = (level != BLE_PERF_SLOW);
    switch (level)
    {
    case BLE_PERF_ULTRA:
        /* Ultra profile (OTA only): same TX pin as FAST, but push the
           connection interval below the Apple Accessory guideline to
           maximize throughput during firmware update. iOS may clamp the
           request up to its 15 ms floor — that's fine, the negotiated
           value is still <= FAST's. Don't use outside OTA: the phone
           may reject the update or fall back to default params, and the
           tight window leaves no slack for other BLE traffic. */
        blebredr_rf_power_set(0, 10);
        // Ultra: Min=6 (7.5 ms), Max=12 (15 ms), latency=0, ST=5 s
        skaiwalk_ble_app_update_conn_param(SkaiWatchSys.watch_conn_id, 6, 12,
                                           0, 500);
        break;
    case BLE_PERF_FAST:
        /* Fast profile (V2T / file transfer / HID): hard-pin TX to +10 dBm
           regardless of TPC state — active operations can't afford a TX
           dip mid-transfer if RSSI happens to be good right then. TPC
           stays paused until we switch back to slow. */
        blebredr_rf_power_set(0, 10);
        // Apple Accessory Design Guidelines:
        //   Interval Min >= 15 ms, Interval Max - Min >= 20 ms,
        //   Slave Latency <= 30, Supervision Timeout <= 6 s
        // Fast: Min=12 (15 ms), Max=28 (35 ms), diff=20 ms, latency=0, ST=5 s
        skaiwalk_ble_app_update_conn_param(SkaiWatchSys.watch_conn_id, 12, 28,
                                           0, 500);
        break;
    case BLE_PERF_SLOW:
    default:
        /* Slow profile (idle): return TX to whichever tier TPC currently
           prefers (0 dBm if signal is good, +3 / +10 if it has been
           ramping up). Without this we'd always drop to 0 dBm on exit
           from fast, undoing TPC's escalation. */
        blebredr_rf_power_set(0, TPC_TIER_DBM[g_tpc_tier]);
        // Slow (idle): Min=80 (100 ms), Max=96 (120 ms), latency=9,
        //   effective wake ~1.2 s when idle, ST=400 (4 s).
        // Phone can still wake us within 120 ms when it has data to send.
        // ST > (latency+1) * max_intv * 2 = 10 * 120ms * 2 = 2400ms < 4000ms ✓
        skaiwalk_ble_app_update_conn_param(SkaiWatchSys.watch_conn_id, 80, 96,
                                           9, 400);
        break;
    }
}

/* Bounded retry on TX-queue saturation. The original implementation looped
   forever with 50 ms sleeps when sibles_write_value returned 0, which under
   silent OTA can wedge the caller — and through the shared _tx_mutex, every
   other commu_send_* caller — for tens of seconds while OTA progress acks
   and regular notifications all compete for the same BLE controller credits.
   1 s total wait is enough to ride out a normal congestion burst at MTU 247;
   beyond that, surface the failure to the caller and let them decide. */
#define BLE_NOTIFY_MAX_RETRIES        20
#define BLE_NOTIFY_RETRY_INTERVAL_MS  50
uint16_t skaiwalk_ble_app_notify(uint8_t *p_data, uint16_t data_length)
{
    if (!g_sifli_test_ble_test_hdl)
    {
        LOG_E("no service");
        return 0;
    }

    sibles_value_t value;
    value.hdl = g_sifli_test_ble_test_hdl;
    value.idx = BLE_APP_RX_VALUE;
    value.len = data_length;
    value.value = p_data;

    int ret = sibles_write_value(phone_device_idx, &value);
    if (ret == value.len) return ret;

    for (int attempt = 0; attempt < BLE_NOTIFY_MAX_RETRIES; attempt++)
    {
        rt_thread_mdelay(BLE_NOTIFY_RETRY_INTERVAL_MS);
        ret = sibles_write_value(phone_device_idx, &value);
        if (ret == value.len)
        {
            LOG_D("send retry success after %d attempts", attempt + 1);
            return ret;
        }
    }
    LOG_W("ble notify dropped after %d retries (%d ms), len=%u",
          BLE_NOTIFY_MAX_RETRIES,
          BLE_NOTIFY_MAX_RETRIES * BLE_NOTIFY_RETRY_INTERVAL_MS,
          data_length);
    return 0;
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

int audio_profile_send_voice_data(uint8_t *voice_data, uint16_t voice_data_len)
{
    if (g_audio_profile_hdl)
    {
        if (ble_app_get_env()->data.is_audio_subscribed == 0)
        {
            return 0;
        }
        // Audio 跟著 HID active device 一起切目標（之前固定送 phone_device_idx，
        // 現在改用 ble_dev_mgr 的 active device conn_idx）
        uint8_t target_conn_idx = phone_device_idx;
        const bonded_devices_db_t *db = ble_dev_mgr_get_database();
        int active_idx = ble_dev_mgr_get_active_device();
        if (db && active_idx >= 0 && active_idx < MAX_BONDED_DEVICES &&
            db->devices[active_idx].conn_idx != 0xFF)
        {
            target_conn_idx = db->devices[active_idx].conn_idx;
        }
        sibles_value_t value;
        value.hdl = g_audio_profile_hdl;
        value.idx = AUDIOPROFILE_AUDIO_VAL;
        value.len = voice_data_len;
        value.value = voice_data;
        int ret = sibles_write_value(target_conn_idx, &value);
        // if (is_signal_bad())
        // {
        //     rt_thread_mdelay(50);
        // }
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

        /* Patch BD_ADDRESS region (NVDS layout: 6 raw MAC bytes start at
           offset 8). The values below are the production MAC override; do
           not change without coordinating with provisioning. */
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
void ble_app_advertising_start(bool mouse_mode, bool pairing_mode);

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
        ble_app_advertising_start(false, false);
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
void ble_app_advertising_start(bool mouse_mode, bool pairing_mode)
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
    char local_name[32] = {0};

#if USING_ADV_MANUFACTURER_DATA
    // Manufaturer data
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;
#endif
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
        rt_snprintf(local_name, sizeof(local_name),
                    "SkaiWatch-%x-%x-%x-%x-%x-%x",
                    addr.addr[0], addr.addr[1], addr.addr[2],
                    addr.addr[3], addr.addr[4], addr.addr[5]);
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

    /* Refresh the bonded-device cache. The targeted-vs-general branching that
       used to live here was removed — both paths set identical config, and
       targeted advertising is enforced via sibles_advertising_reconfig +
       g_target_device_addr down the call chain. pairing_mode and is_bonded
       now drive the adv interval choice below. */
    ble_gap_addr_t bonded_addr;
    uint8_t is_bonded = ble_app_get_bonded_device_addr(&bonded_addr);

    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    para.config.mode_config.conn_config.duration = 0x0;
    para.config.mode_config.conn_config.interval = 0x30;
    para.config.max_tx_pwr = 0x7F;
    // Enable restart after disconnected
    para.config.is_auto_restart = 1;
    // adv data and rsp data use same data
    // para.config.is_rsp_data_duplicate = 1;

    /* Prepare name field. Full name "SkaiWatch-xx-xx-xx-xx-xx-xx" (up to 27
     * chars + 2-byte AD header) doesn't fit alongside flags/appearance/service
     * UUID in the 31-byte adv packet, so put it in scan response data instead. */
    para.rsp_data.completed_name =
        rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.rsp_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.rsp_data.completed_name->name, local_name,
              para.rsp_data.completed_name->name_len);
#ifdef GAP_GATT_APPEARANCE_HUMAN_INTERFACE_DEVICE
    /* Prepare Appearance data filed .*/
    {
        uint16_t appearance;
        if (mouse_mode)
        {
            appearance = GAP_GATT_APPEARANCE_MOUSE;
        }
        else
        {
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

    // Always tear down any existing ADV activity before re-init. sibles_advertising_init
    // resets adv_idx to INVALID, so without deleting first, sibles_advertising_start
    // creates a brand-new GAP activity each call and leaks the previous one — the BLE
    // stack only has a few activity slots and runs out after a few restarts.
    sibles_advertising_stop(g_app_advertising_context);
    rt_thread_mdelay(100);
    sibles_advertising_delete(g_app_advertising_context);
    rt_thread_mdelay(100);

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    sibles_advertising_reconfig(g_app_advertising_context, &para.config);
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

#ifdef USING_BLE_SERIAL
uint8_t g_diss_conn_idx;
uint16_t g_th_total_cnt;
uint16_t g_th_interval;
uint16_t g_th_packet_size;
#endif
void ble_app_entry(void *param)
{
    app_env_t *env = ble_app_get_env();

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
            ble_app_advertising_start(false, false);
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
    /* Idempotent: middleware/lvgl/littlevgl2rtt.c also calls this from
       gui_lib_init() (INIT_COMPONENT) so the root FS is mounted before
       FreeType opens font files. The auto INIT_ENV invocation below is a
       safety fallback for cases where freetype isn't enabled. */
    static int mounted = 0;
    if (mounted)
        return RT_EOK;

    rt_kprintf("[mnt_init] enter, FS_REGION_START_ADDR=0x%08x size=0x%08x\n",
               FS_REGION_START_ADDR, FS_REGION_SIZE);
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000),
                         FS_REGION_START_ADDR -
                             (FS_REGION_START_ADDR & (0xFC000000)),
                         FS_REGION_SIZE, NAND_MTD_NAME);
    rt_kprintf("[mnt_init] register_nand_device done, mounting...\n");
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
        mounted = 1;
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
            {
                rt_kprintf("mount fs on flash success\n");
                mounted = 1;
            }
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

        LOG_I("Peer device(" MAC_FMT ") connected",
              MAC_ARG(env->conn_para.peer_addr.addr));

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
        ble_tpc_on_rssi_sample(ind->rssi);
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
    rt_hw_watchdog_set_status(en);
    /* Avoid repeated hook installation. */
    rt_hw_watchdog_hook(0);
    if (!en)
    {
        rt_hw_watchdog_deinit();
    }
    else
    {
        rt_hw_watchdog_hook(1);
    }
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
