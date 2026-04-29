/**
 * @file ble_device_manager.c
 * @brief Multi-device Bluetooth management implementation
 */

#include "ble_device_manager.h"
#include "bf0_ble_gap.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "lvgl.h"

#define DBG_TAG "ble.dev_mgr"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// Forward declarations for BLE app functions
extern void ble_app_set_bonded_device_addr(ble_gap_addr_t *addr);
extern void ble_app_advertising_start(bool mouse_mode, bool pairing_mode);

// BLE GAP connection parameters
#define CONN_TIMEOUT_MS 10000  // 10 seconds
#define SCAN_INTERVAL 0x30     // 30ms
#define SCAN_WINDOW 0x30       // 30ms
#define CONN_INTERVAL_MIN 0x60 // 120ms
#define CONN_INTERVAL_MAX 0x80 // 160ms
#define CONN_LATENCY 0
#define SUPERVISION_TIMEOUT 500 // 5 seconds
#define CE_LEN_MIN 60
#define CE_LEN_MAX 100

/** Device manager context */
typedef struct
{
    bonded_devices_db_t database;
    dev_mgr_event_cb_t event_cb;
    void *cb_user_data;
    uint8_t initialized;
    rt_mutex_t mutex;
} dev_mgr_ctx_t;

static dev_mgr_ctx_t g_dev_mgr = {0};

/**
 * @brief Lock device manager
 */
static inline void dev_mgr_lock(void)
{
    if (g_dev_mgr.mutex)
    {
        rt_mutex_take(g_dev_mgr.mutex, RT_WAITING_FOREVER);
    }
}

/**
 * @brief Unlock device manager
 */
static inline void dev_mgr_unlock(void)
{
    if (g_dev_mgr.mutex)
    {
        rt_mutex_release(g_dev_mgr.mutex);
    }
}

/**
 * @brief Notify event to registered callback
 */
static void dev_mgr_notify_event(dev_mgr_event_t event, uint8_t device_idx)
{
    if (g_dev_mgr.event_cb)
    {
        g_dev_mgr.event_cb(event, device_idx, g_dev_mgr.cb_user_data);
    }
}

int ble_dev_mgr_init(void)
{
    if (g_dev_mgr.initialized)
    {
        LOG_W("Device manager already initialized");
        return 0;
    }

    memset(&g_dev_mgr, 0, sizeof(dev_mgr_ctx_t));

    g_dev_mgr.mutex = rt_mutex_create("dev_mgr", RT_IPC_FLAG_PRIO);
    if (!g_dev_mgr.mutex)
    {
        LOG_E("Failed to create mutex");
        return -1;
    }

    // Initialize database
    g_dev_mgr.database.count = 0;
    g_dev_mgr.database.active_device_idx = 0xFF; // No active device

    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        g_dev_mgr.database.devices[i].is_valid = 0;
        g_dev_mgr.database.devices[i].conn_idx = 0xFF;
    }

    g_dev_mgr.initialized = 1;

    // Try to load from Flash
    ble_dev_mgr_load_from_flash();

    LOG_I("Device manager initialized, %d devices loaded",
          g_dev_mgr.database.count);

    return 0;
}

int ble_dev_mgr_register_callback(dev_mgr_event_cb_t cb, void *user_data)
{
    if (!g_dev_mgr.initialized)
    {
        LOG_E("Device manager not initialized");
        return -1;
    }

    dev_mgr_lock();
    g_dev_mgr.event_cb = cb;
    g_dev_mgr.cb_user_data = user_data;
    dev_mgr_unlock();

    return 0;
}

int ble_dev_mgr_find_device(const uint8_t *mac_addr)
{
    if (!g_dev_mgr.initialized || !mac_addr)
    {
        return -1;
    }

    dev_mgr_lock();

    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (g_dev_mgr.database.devices[i].is_valid &&
            memcmp(g_dev_mgr.database.devices[i].mac_addr, mac_addr, 6) == 0)
        {
            dev_mgr_unlock();
            return i;
        }
    }

    dev_mgr_unlock();
    return -1;
}

int ble_dev_mgr_add_device(const uint8_t *mac_addr, uint8_t addr_type,
                           const char *device_name,
                           ble_device_type_t device_type)
{
    if (!g_dev_mgr.initialized || !mac_addr)
    {
        LOG_E("Invalid parameters");
        return -1;
    }

    dev_mgr_lock();

    // Check if device already exists
    int existing_idx = -1;
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (g_dev_mgr.database.devices[i].is_valid &&
            memcmp(g_dev_mgr.database.devices[i].mac_addr, mac_addr, 6) == 0)
        {
            existing_idx = i;
            break;
        }
    }

    int device_idx = existing_idx;

    // If device doesn't exist, find empty slot
    if (device_idx == -1)
    {
        for (int i = 0; i < MAX_BONDED_DEVICES; i++)
        {
            if (!g_dev_mgr.database.devices[i].is_valid)
            {
                device_idx = i;
                break;
            }
        }
    }

    if (device_idx == -1)
    {
        LOG_E("Device list full, cannot add more devices");
        dev_mgr_unlock();
        return -2;
    }

    bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];

    // Update device info
    memcpy(dev->mac_addr, mac_addr, 6);
    dev->addr_type = addr_type;
    dev->device_type = device_type;
    dev->is_valid = 1;
    dev->last_connected_time = (uint32_t)time(NULL);

    if (device_name)
    {
        strncpy(dev->device_name, device_name, DEVICE_NAME_MAX_LEN - 1);
        dev->device_name[DEVICE_NAME_MAX_LEN - 1] = '\0';
    }
    else
    {
        // Generate default name based on MAC address
        snprintf(dev->device_name, DEVICE_NAME_MAX_LEN, "Device_%02X%02X",
                 mac_addr[4], mac_addr[5]);
    }

    // Update count if new device
    if (existing_idx == -1)
    {
        g_dev_mgr.database.count++;
        LOG_I("Added new device [%d]: %s (%02X:%02X:%02X:%02X:%02X:%02X)",
              device_idx, dev->device_name, mac_addr[5], mac_addr[4],
              mac_addr[3], mac_addr[2], mac_addr[1], mac_addr[0]);
        dev_mgr_notify_event(DEV_MGR_EVT_DEVICE_ADDED, device_idx);
    }
    else
    {
        LOG_I("Updated device [%d]: %s", device_idx, dev->device_name);
    }

    dev_mgr_unlock();

    // Save to Flash
    ble_dev_mgr_save_to_flash();

    return device_idx;
}

int ble_dev_mgr_remove_device(uint8_t device_idx)
{
    if (!g_dev_mgr.initialized || device_idx >= MAX_BONDED_DEVICES)
    {
        return -1;
    }

    dev_mgr_lock();

    bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];

    if (!dev->is_valid)
    {
        dev_mgr_unlock();
        return -2;
    }

    LOG_I("Removing device [%d]: %s", device_idx, dev->device_name);

    // Clear device data
    memset(dev, 0, sizeof(bonded_device_t));
    dev->is_valid = 0;
    dev->conn_idx = 0xFF;

    g_dev_mgr.database.count--;

    // Update active device if needed
    if (g_dev_mgr.database.active_device_idx == device_idx)
    {
        g_dev_mgr.database.active_device_idx = 0xFF;
    }

    dev_mgr_notify_event(DEV_MGR_EVT_DEVICE_REMOVED, device_idx);
    dev_mgr_unlock();

    // Save to Flash
    ble_dev_mgr_save_to_flash();

    return 0;
}

int ble_dev_mgr_clear_all(void)
{
    if (!g_dev_mgr.initialized)
    {
        return -1;
    }

    LOG_I("Clearing all bonded devices");

    dev_mgr_lock();

    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (g_dev_mgr.database.devices[i].is_valid)
        {
            memset(&g_dev_mgr.database.devices[i], 0, sizeof(bonded_device_t));
            g_dev_mgr.database.devices[i].is_valid = 0;
            g_dev_mgr.database.devices[i].conn_idx = 0xFF;
        }
    }

    g_dev_mgr.database.count = 0;
    g_dev_mgr.database.active_device_idx = 0xFF;

    dev_mgr_unlock();

    // Save to Flash
    ble_dev_mgr_save_to_flash();

    dev_mgr_notify_event(DEV_MGR_EVT_DATABASE_UPDATED, 0xFF);

    return 0;
}

const bonded_devices_db_t *ble_dev_mgr_get_database(void)
{
    if (!g_dev_mgr.initialized)
    {
        return NULL;
    }
    return &g_dev_mgr.database;
}

int ble_dev_mgr_update_connection(uint8_t device_idx, uint8_t conn_idx)
{
    if (!g_dev_mgr.initialized || device_idx >= MAX_BONDED_DEVICES)
    {
        return -1;
    }

    dev_mgr_lock();

    bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];

    if (!dev->is_valid)
    {
        dev_mgr_unlock();
        return -2;
    }

    uint8_t was_connected = (dev->conn_idx != 0xFF);
    uint8_t is_connected = (conn_idx != 0xFF);

    dev->conn_idx = conn_idx;

    if (is_connected && !was_connected)
    {
        dev->last_connected_time = (uint32_t)time(NULL);
        LOG_I("Device [%d] connected: %s (conn_idx=%d)", device_idx,
              dev->device_name, conn_idx);
        dev_mgr_notify_event(DEV_MGR_EVT_DEVICE_CONNECTED, device_idx);
    }
    else if (!is_connected && was_connected)
    {
        LOG_I("Device [%d] disconnected: %s", device_idx, dev->device_name);

        // If the disconnected device was the active device, switch to another
        // connected device
        if (g_dev_mgr.database.active_device_idx == device_idx)
        {
            LOG_I("Active device disconnected, switching to another device...");

            // Find another connected device to set as active
            int new_active = -1;
            for (int i = 0; i < MAX_BONDED_DEVICES; i++)
            {
                if (i != device_idx && g_dev_mgr.database.devices[i].is_valid &&
                    g_dev_mgr.database.devices[i].conn_idx != 0xFF)
                {
                    new_active = i;
                    break;
                }
            }

            if (new_active >= 0)
            {
                g_dev_mgr.database.active_device_idx = new_active;
                LOG_I("Switched active device to [%d]: %s", new_active,
                      g_dev_mgr.database.devices[new_active].device_name);
                dev_mgr_notify_event(DEV_MGR_EVT_ACTIVE_DEVICE_CHANGED,
                                     new_active);
            }
            else
            {
                g_dev_mgr.database.active_device_idx = 0xFF;
                LOG_I("No other connected devices, clearing active device");
            }
        }

        dev_mgr_notify_event(DEV_MGR_EVT_DEVICE_DISCONNECTED, device_idx);
    }

    dev_mgr_unlock();

    return 0;
}

int ble_dev_mgr_set_active_device(uint8_t device_idx)
{
    if (!g_dev_mgr.initialized)
    {
        return -1;
    }

    if (device_idx >= MAX_BONDED_DEVICES && device_idx != 0xFF)
    {
        return -2;
    }

    dev_mgr_lock();

    if (device_idx != 0xFF && !g_dev_mgr.database.devices[device_idx].is_valid)
    {
        dev_mgr_unlock();
        return -3;
    }

    uint8_t old_active = g_dev_mgr.database.active_device_idx;
    g_dev_mgr.database.active_device_idx = device_idx;

    // Set as target device for directed advertising
    if (device_idx != 0xFF)
    {
        const bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];
        ble_gap_addr_t target_addr;
        memcpy(target_addr.addr.addr, dev->mac_addr, 6);
        target_addr.addr_type = dev->addr_type;

        ble_app_set_bonded_device_addr(&target_addr);
        LOG_I("Set device[%d] as target device: %02X:%02X:%02X:%02X:%02X:%02X",
              device_idx, dev->mac_addr[5], dev->mac_addr[4], dev->mac_addr[3],
              dev->mac_addr[2], dev->mac_addr[1], dev->mac_addr[0]);
    }

    if (old_active != device_idx)
    {
        LOG_I("Active device changed: [%d] -> [%d]", old_active, device_idx);
        dev_mgr_notify_event(DEV_MGR_EVT_ACTIVE_DEVICE_CHANGED, device_idx);
    }

    dev_mgr_unlock();

    // Save to flash to persist active/target device
    ble_dev_mgr_save_to_flash();

    return 0;
}

int ble_dev_mgr_get_active_device(void)
{
    if (!g_dev_mgr.initialized)
    {
        return -1;
    }
    return g_dev_mgr.database.active_device_idx;
}

int ble_dev_mgr_get_connected_count(void)
{
    if (!g_dev_mgr.initialized)
    {
        return 0;
    }

    int count = 0;
    dev_mgr_lock();

    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (g_dev_mgr.database.devices[i].is_valid &&
            g_dev_mgr.database.devices[i].conn_idx != 0xFF)
        {
            count++;
        }
    }

    dev_mgr_unlock();
    return count;
}

int ble_dev_mgr_connect_device(uint8_t device_idx)
{
    if (!g_dev_mgr.initialized || device_idx >= MAX_BONDED_DEVICES)
    {
        LOG_E("Invalid parameters or not initialized");
        return -1;
    }

    dev_mgr_lock();

    bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];

    if (!dev->is_valid)
    {
        LOG_E("Device [%d] not valid", device_idx);
        dev_mgr_unlock();
        return -2;
    }

    // Check if already connected
    if (dev->conn_idx != 0xFF)
    {
        LOG_W("Device [%d] already connected (conn_idx=%d)", device_idx,
              dev->conn_idx);
        dev_mgr_unlock();
        return 0;
    }

    LOG_I("Initiating connection to device [%d]: %s "
          "(%02X:%02X:%02X:%02X:%02X:%02X)",
          device_idx, dev->device_name, dev->mac_addr[5], dev->mac_addr[4],
          dev->mac_addr[3], dev->mac_addr[2], dev->mac_addr[1],
          dev->mac_addr[0]);

    // Step 1: Disconnect all currently connected devices
    LOG_I("Step 1: Disconnecting all current connections...");
    int disconnected_count = 0;
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (g_dev_mgr.database.devices[i].is_valid &&
            g_dev_mgr.database.devices[i].conn_idx != 0xFF)
        {
            uint8_t conn_idx = g_dev_mgr.database.devices[i].conn_idx;
            LOG_I("  Disconnecting device [%d] (conn_idx=%d)", i, conn_idx);

            // Prepare disconnect parameters
            ble_gap_disconnect_t disconnect_param;
            disconnect_param.conn_idx = conn_idx;
            disconnect_param.reason = 0x13; // Remote user terminated connection

            // Execute disconnect
            ble_gap_disconnect(&disconnect_param);
            disconnected_count++;
        }
    }

    if (disconnected_count > 0)
    {
        LOG_I("Disconnected %d device(s), waiting for cleanup...",
              disconnected_count);
        // Note: Connection indices will be cleared by disconnect event handler
    }

    // Step 2: Set target bonded device address for directed advertising
    LOG_I("Step 2: Setting target device address for directed advertising...");
    ble_gap_addr_t target_addr;
    target_addr.addr_type = dev->addr_type;
    memcpy(target_addr.addr.addr, dev->mac_addr, BD_ADDR_LEN);

    // ble_app_set_bonded_device_addr(&target_addr);
    // LOG_I("  Target device set: %02X:%02X:%02X:%02X:%02X:%02X (type:%d)",
    //       dev->mac_addr[5], dev->mac_addr[4], dev->mac_addr[3],
    //       dev->mac_addr[2], dev->mac_addr[1], dev->mac_addr[0],
    //       dev->addr_type);
    LOG_I("Initiating connection to device [%d]: %s "
          "(%02X:%02X:%02X:%02X:%02X:%02X)",
          device_idx, dev->device_name, dev->mac_addr[5], dev->mac_addr[4],
          dev->mac_addr[3], dev->mac_addr[2], dev->mac_addr[1],
          dev->mac_addr[0]);

    // Prepare BLE GAP connection parameters
    ble_gap_connection_create_param_t conn_param;
    memset(&conn_param, 0, sizeof(conn_param));

    // Set connection parameters
    conn_param.own_addr_type = GAPM_STATIC_ADDR;
    conn_param.conn_to = CONN_TIMEOUT_MS;
    conn_param.type = GAPM_INIT_TYPE_DIRECT_CONN_EST;

    // 1M PHY parameters
    conn_param.conn_param_1m.scan_intv = SCAN_INTERVAL;
    conn_param.conn_param_1m.scan_wd = SCAN_WINDOW;
    conn_param.conn_param_1m.conn_intv_min = CONN_INTERVAL_MIN;
    conn_param.conn_param_1m.conn_intv_max = CONN_INTERVAL_MAX;
    conn_param.conn_param_1m.conn_latency = CONN_LATENCY;
    conn_param.conn_param_1m.supervision_to = SUPERVISION_TIMEOUT;
    conn_param.conn_param_1m.ce_len_min = CE_LEN_MIN;
    conn_param.conn_param_1m.ce_len_max = CE_LEN_MAX;

    // Set peer device address
    conn_param.peer_addr.addr_type = dev->addr_type;
    memcpy(conn_param.peer_addr.addr.addr, dev->mac_addr, BD_ADDR_LEN);

    dev_mgr_unlock();

    // // Step 3: Restart advertising with directed mode
    // LOG_I("Step 3: Restarting advertising in directed mode...");
    // // restart_adv=true, mouse_mode=false, pairing_mode=false
    // // This will use directed advertising to the bonded device we just set
    // ble_app_advertising_start(true, false, false);

    // LOG_I("Directed advertising started for device [%d], waiting for
    // connection...", device_idx); Initiate BLE connection
    uint8_t result = ble_gap_create_connection(&conn_param);

    if (result != 0)
    {
        LOG_E("Failed to initiate connection to device [%d], error: %d",
              device_idx, result);
        return -3;
    }

    LOG_I("Connection request sent successfully for device [%d]", device_idx);
    return 0;
}

int ble_dev_mgr_disconnect_device(uint8_t device_idx)
{
    if (!g_dev_mgr.initialized || device_idx >= MAX_BONDED_DEVICES)
    {
        LOG_E("Invalid parameters or not initialized");
        return -1;
    }

    dev_mgr_lock();

    bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];

    if (!dev->is_valid)
    {
        LOG_E("Device [%d] not valid", device_idx);
        dev_mgr_unlock();
        return -2;
    }

    if (dev->conn_idx == 0xFF)
    {
        LOG_W("Device [%d] not connected", device_idx);
        dev_mgr_unlock();
        return 0;
    }

    uint8_t conn_idx = dev->conn_idx;

    LOG_I("Disconnecting device [%d]: %s (conn_idx=%d)", device_idx,
          dev->device_name, conn_idx);

    dev_mgr_unlock();

    // Prepare disconnect parameters
    ble_gap_disconnect_t disconnect_param;
    disconnect_param.conn_idx = conn_idx;
    disconnect_param.reason = 0x13; // CO_ERROR_CON_TERM_BY_LOCAL_HOST

    // Send disconnect command
    uint8_t result = ble_gap_disconnect(&disconnect_param);

    if (result != 0)
    {
        LOG_E("Failed to disconnect device [%d], error: %d", device_idx,
              result);
        return -3;
    }

    LOG_I("Disconnect request sent successfully for device [%d]", device_idx);
    return 0;
}

int ble_dev_mgr_save_to_flash(void)
{
    if (!g_dev_mgr.initialized)
    {
        LOG_E("Device manager not initialized");
        return -1;
    }

#ifdef BSP_SHARE_PREFS
    dev_mgr_lock();
    int ret = ble_dev_prefs_save(&g_dev_mgr.database);
    dev_mgr_unlock();
    return ret;
#else
    LOG_W("BSP_SHARE_PREFS not defined, Flash storage not available");
    return -1;
#endif
}

int ble_dev_mgr_load_from_flash(void)
{
    if (!g_dev_mgr.initialized)
    {
        LOG_E("Device manager not initialized");
        return -1;
    }

#ifdef BSP_SHARE_PREFS
    dev_mgr_lock();
    int ret = ble_dev_prefs_load(&g_dev_mgr.database);
    if (ret < 0)
    {
        dev_mgr_unlock();
        return ret;
    }

    // Clear connection indices (devices are not connected at startup)
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        g_dev_mgr.database.devices[i].conn_idx = 0xFF;
    }

    int32_t active_idx = g_dev_mgr.database.active_device_idx;
    bool need_save_default = false;

    // Log loaded devices
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (g_dev_mgr.database.devices[i].is_valid)
        {
            LOG_I("  [%d] %s (%02X:%02X:%02X:%02X:%02X:%02X)%s", i,
                  g_dev_mgr.database.devices[i].device_name,
                  g_dev_mgr.database.devices[i].mac_addr[5],
                  g_dev_mgr.database.devices[i].mac_addr[4],
                  g_dev_mgr.database.devices[i].mac_addr[3],
                  g_dev_mgr.database.devices[i].mac_addr[2],
                  g_dev_mgr.database.devices[i].mac_addr[1],
                  g_dev_mgr.database.devices[i].mac_addr[0],
                  (i == active_idx) ? " [ACTIVE]" : "");
        }
    }

    // Set active device as target device for directed advertising
    // If no active device found in Flash, default to first valid device
    if (active_idx == 0xFF || active_idx >= MAX_BONDED_DEVICES ||
        !g_dev_mgr.database.devices[active_idx].is_valid)
    {
        // Find first valid device
        for (int i = 0; i < MAX_BONDED_DEVICES; i++)
        {
            if (g_dev_mgr.database.devices[i].is_valid)
            {
                active_idx = i;
                g_dev_mgr.database.active_device_idx = i;
                need_save_default = true;
                LOG_I("No active device in Flash, defaulting to first "
                      "device: [%d]",
                      i);
                break;
            }
        }
    }

    // Set the active device as target device
    if (active_idx != 0xFF && active_idx < MAX_BONDED_DEVICES &&
        g_dev_mgr.database.devices[active_idx].is_valid)
    {
        const bonded_device_t *dev =
            &g_dev_mgr.database.devices[active_idx];
        ble_gap_addr_t target_addr;
        memcpy(target_addr.addr.addr, dev->mac_addr, 6);
        target_addr.addr_type = dev->addr_type;

        ble_app_set_bonded_device_addr(&target_addr);
        LOG_I("Set target device: [%d] %s", active_idx, dev->device_name);
    }

    dev_mgr_unlock();

    // Save default active device selection to Flash if needed
    if (need_save_default)
    {
        LOG_I("Saving default active device to Flash");
        ble_dev_mgr_save_to_flash();
    }

    return 0;
#else
    LOG_W("BSP_SHARE_PREFS not defined, Flash storage not available");
    return -1;
#endif
}

int ble_dev_mgr_get_device_state(uint8_t device_idx)
{
    if (!g_dev_mgr.initialized || device_idx >= MAX_BONDED_DEVICES)
    {
        return -1;
    }

    dev_mgr_lock();

    bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];

    if (!dev->is_valid)
    {
        dev_mgr_unlock();
        return -2;
    }

    int state = (dev->conn_idx != 0xFF) ? DEVICE_STATE_CONNECTED
                                        : DEVICE_STATE_DISCONNECTED;

    dev_mgr_unlock();

    return state;
}

int ble_dev_mgr_update_device_name(uint8_t device_idx, const char *device_name)
{
    if (!g_dev_mgr.initialized || device_idx >= MAX_BONDED_DEVICES ||
        !device_name)
    {
        return -1;
    }

    dev_mgr_lock();

    bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];

    if (!dev->is_valid)
    {
        dev_mgr_unlock();
        return -2;
    }

    strncpy(dev->device_name, device_name, DEVICE_NAME_MAX_LEN - 1);
    dev->device_name[DEVICE_NAME_MAX_LEN - 1] = '\0';

    LOG_I("Updated device [%d] name to: %s", device_idx, device_name);

    dev_mgr_unlock();

    // Save to Flash
    ble_dev_mgr_save_to_flash();

    return 0;
}

int ble_dev_mgr_update_device_type(uint8_t device_idx, ble_device_type_t device_type)
{
    if (!g_dev_mgr.initialized || device_idx >= MAX_BONDED_DEVICES)
    {
        return -1;
    }

    dev_mgr_lock();

    bonded_device_t *dev = &g_dev_mgr.database.devices[device_idx];

    if (!dev->is_valid)
    {
        dev_mgr_unlock();
        return -2;
    }

    dev->device_type = device_type;

    LOG_I("Updated device [%d] type to: %d", device_idx, device_type);

    dev_mgr_unlock();

    // Save to Flash
    ble_dev_mgr_save_to_flash();

    return 0;
}

int ble_dev_mgr_switch_to_next_device(void)
{
    if (!g_dev_mgr.initialized)
    {
        LOG_E("Device manager not initialized");
        return -1;
    }

    dev_mgr_lock();

    int current_active = g_dev_mgr.database.active_device_idx;

    // Find all connected devices
    int connected_devices[MAX_BONDED_DEVICES];
    int connected_count = 0;

    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (g_dev_mgr.database.devices[i].is_valid &&
            g_dev_mgr.database.devices[i].conn_idx != 0xFF)
        {
            connected_devices[connected_count++] = i;
        }
    }

    if (connected_count == 0)
    {
        LOG_W("No connected devices to switch to");
        dev_mgr_unlock();
        return -2;
    }

    if (connected_count == 1)
    {
        // Only one device connected, just ensure it's active
        int device_idx = connected_devices[0];
        if (current_active != device_idx)
        {
            g_dev_mgr.database.active_device_idx = device_idx;
            LOG_I("Set single connected device as active [%d]: %s", device_idx,
                  g_dev_mgr.database.devices[device_idx].device_name);
            dev_mgr_notify_event(DEV_MGR_EVT_ACTIVE_DEVICE_CHANGED, device_idx);
        }
        dev_mgr_unlock();
        return device_idx;
    }

    // Multiple devices connected, find next one
    int next_idx = -1;
    bool found_current = false;

    for (int i = 0; i < connected_count; i++)
    {
        if (connected_devices[i] == current_active)
        {
            found_current = true;
            // Get next device (wrap around if at end)
            next_idx = connected_devices[(i + 1) % connected_count];
            break;
        }
    }

    if (!found_current)
    {
        // Current active device not in connected list, use first connected
        // device
        next_idx = connected_devices[0];
    }

    if (next_idx >= 0)
    {
        uint8_t old_active = g_dev_mgr.database.active_device_idx;
        g_dev_mgr.database.active_device_idx = next_idx;

        LOG_I("Switched active device from [%d] to [%d]: %s", old_active,
              next_idx, g_dev_mgr.database.devices[next_idx].device_name);

        dev_mgr_notify_event(DEV_MGR_EVT_ACTIVE_DEVICE_CHANGED, next_idx);
        dev_mgr_unlock();
        return next_idx;
    }

    dev_mgr_unlock();
    return -3;
}

static uint8_t main_addr[6] = {0};
extern void get_main_phonepeer_addr(uint8_t *addr);

/**
 * @brief Check if main phone counterpart device is connected
 * @return device index if connected, -1 if not found or not connected
 */
int check_main_phone_counterpart_connection(void)
{
    get_main_phonepeer_addr(main_addr);

    // Check if main_addr is all zeros (invalid)
    bool is_valid = false;
    for (int i = 0; i < 6; i++)
    {
        if (main_addr[i] != 0)
        {
            is_valid = true;
            break;
        }
    }

    if (!is_valid)
    {
        return -1;
    }
    // Search through all connected devices
    dev_mgr_lock();

    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (g_dev_mgr.database.devices[i].is_valid &&
            g_dev_mgr.database.devices[i].conn_idx != 0xFF)
        {
            // Device is valid and connected, check MAC address
            if (memcmp(g_dev_mgr.database.devices[i].mac_addr, main_addr, 6) ==
                0)
            {
                LOG_I("Main phone found and connected: device[%d] %s", i,
                      g_dev_mgr.database.devices[i].device_name);
                dev_mgr_unlock();
                return i;
            }
        }
    }

    dev_mgr_unlock();

    return -1;
}
