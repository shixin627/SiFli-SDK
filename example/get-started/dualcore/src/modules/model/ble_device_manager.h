/**
 * @file ble_device_manager.h
 * @brief Multi-device Bluetooth management system
 *
 * This module manages multiple bonded BLE devices, similar to Samsung Buds+
 * Features:
 * - Store up to 8 bonded devices
 * - Display device list in UI
 * - Active connection to selected devices
 * - Support for dual concurrent connections
 * - Device switching
 */

#ifndef BLE_DEVICE_MANAGER_H
#define BLE_DEVICE_MANAGER_H

#include <rtthread.h>
#include <rtdevice.h>
#ifdef RT_USING_BLUETOOTH
#include "bf0_ble_gap.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// Define strategy values first
#define CONNECT_STRATEGY_GENERAL 0
#define CONNECT_STRATEGY_DIRECTED 1
#define CONNECT_STRATEGY_WHITELIST 2

/** Select connection strategy - Change this to switch between strategies */
#define CONNECT_STRATEGY CONNECT_STRATEGY_GENERAL

#define ENABLE_BG_ADV 0

/** Maximum number of bonded devices that can be stored */
#define MAX_BONDED_DEVICES 8

/** Maximum number of concurrent connections */
#define MAX_CONCURRENT_CONNECTIONS 2

/** Device name maximum length */
#define DEVICE_NAME_MAX_LEN 32

    /**
     * @brief Device type enumeration
     */
    typedef enum
    {
        DEVICE_TYPE_PHONE = 0, /**< Mobile phone */
        DEVICE_TYPE_COMPUTER,  /**< Computer/Laptop */
        DEVICE_TYPE_TABLET,    /**< Tablet */
        DEVICE_TYPE_OTHER      /**< Other device type */
    } ble_device_type_t;

    /**
     * @brief Connection state enumeration
     */
    typedef enum
    {
        DEVICE_STATE_DISCONNECTED = 0, /**< Device is disconnected */
        DEVICE_STATE_CONNECTING,       /**< Connection in progress */
        DEVICE_STATE_CONNECTED,        /**< Device is connected */
        DEVICE_STATE_BONDING,          /**< Bonding in progress */
    } ble_device_state_t;

    /**
     * @brief Bonded device information structure
     */
    typedef struct
    {
        uint8_t mac_addr[6];                   /**< Device MAC address */
        uint8_t addr_type;                     /**< Address type (public/random) */
        char device_name[DEVICE_NAME_MAX_LEN]; /**< Device name */
        uint8_t device_type;                   /**< Device type @see ble_device_type_t */
        uint32_t last_connected_time;          /**< Last connection timestamp (Unix time) */
        uint8_t is_valid;                      /**< Entry is valid (1) or empty (0) */
        uint8_t conn_idx;                      /**< Current connection index (0xFF if disconnected) */
        uint8_t reserved[2];                   /**< Reserved for future use */
    } bonded_device_t;

    /**
     * @brief Bonded devices database structure
     */
    typedef struct
    {
        uint8_t count;                               /**< Number of bonded devices */
        uint8_t active_device_idx;                   /**< Currently active device index for audio output */
        bonded_device_t devices[MAX_BONDED_DEVICES]; /**< Device list */
    } bonded_devices_db_t;

    /**
     * @brief Device manager event type
     */
    typedef enum
    {
        DEV_MGR_EVT_DEVICE_ADDED = 0,      /**< New device added to list */
        DEV_MGR_EVT_DEVICE_REMOVED,        /**< Device removed from list */
        DEV_MGR_EVT_DEVICE_CONNECTED,      /**< Device connected */
        DEV_MGR_EVT_DEVICE_DISCONNECTED,   /**< Device disconnected */
        DEV_MGR_EVT_ACTIVE_DEVICE_CHANGED, /**< Active device changed */
        DEV_MGR_EVT_DATABASE_UPDATED,      /**< Database updated (for UI refresh) */
    } dev_mgr_event_t;

    /**
     * @brief Device manager event callback
     * @param event Event type
     * @param device_idx Device index in database
     * @param user_data User data passed during registration
     */
    typedef void (*dev_mgr_event_cb_t)(dev_mgr_event_t event, uint8_t device_idx, void *user_data);

    /**
     * @brief Initialize device manager
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_init(void);

    /**
     * @brief Register event callback
     * @param cb Callback function
     * @param user_data User data to pass to callback
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_register_callback(dev_mgr_event_cb_t cb, void *user_data);

    /**
     * @brief Add or update a bonded device
     * @param addr Device address
     * @param addr_type Address type
     * @param device_name Device name (can be NULL)
     * @param device_type Device type
     * @return Device index on success, negative on error
     */
    int ble_dev_mgr_add_device(const uint8_t *mac_addr, uint8_t addr_type,
                               const char *device_name, ble_device_type_t device_type);

    /**
     * @brief Remove a bonded device
     * @param device_idx Device index to remove
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_remove_device(uint8_t device_idx);

    /**
     * @brief Clear all bonded devices
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_clear_all(void);

    /**
     * @brief Get bonded devices database
     * @return Pointer to database (read-only)
     */
    const bonded_devices_db_t *ble_dev_mgr_get_database(void);

    /**
     * @brief Find device by MAC address
     * @param mac_addr Device MAC address
     * @return Device index if found, -1 if not found
     */
    int ble_dev_mgr_find_device(const uint8_t *mac_addr);

    /**
     * @brief Update device connection state
     * @param device_idx Device index
     * @param conn_idx Connection index (0xFF if disconnected)
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_update_connection(uint8_t device_idx, uint8_t conn_idx);

    /**
     * @brief Set active device (for audio output)
     * @param device_idx Device index to set as active
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_set_active_device(uint8_t device_idx);

    /**
     * @brief Get active device index
     * @return Active device index, -1 if none
     */
    int ble_dev_mgr_get_active_device(void);

    /**
     * @brief Get number of currently connected devices
     * @return Number of connected devices
     */
    int ble_dev_mgr_get_connected_count(void);

    /**
     * @brief Initiate connection to a bonded device
     * @param device_idx Device index to connect
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_connect_device(uint8_t device_idx);

    /**
     * @brief Disconnect from a device
     * @param device_idx Device index to disconnect
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_disconnect_device(uint8_t device_idx);

    /**
     * @brief Save database to Flash (for persistence)
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_save_to_flash(void);

    /**
     * @brief Load database from Flash
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_load_from_flash(void);

    /**
     * @brief Save bonded devices database to share_prefs (implemented in watch_global_data.c)
     * @param db Pointer to the database to save
     * @return 0 on success, negative on error
     */
    int ble_dev_prefs_save(const bonded_devices_db_t *db);

    /**
     * @brief Load bonded devices database from share_prefs (implemented in watch_global_data.c)
     * @param db Pointer to the database to load into
     * @return 0 on success, negative on error
     */
    int ble_dev_prefs_load(bonded_devices_db_t *db);

    /**
     * @brief Get device connection state
     * @param device_idx Device index
     * @return Connection state @see ble_device_state_t, negative on error
     */
    int ble_dev_mgr_get_device_state(uint8_t device_idx);

    /**
     * @brief Update device name
     * @param device_idx Device index
     * @param device_name New device name
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_update_device_name(uint8_t device_idx, const char *device_name);

    /**
     * @brief Update device type
     * @param device_idx Device index
     * @param device_type New device type
     * @return 0 on success, negative on error
     */
    int ble_dev_mgr_update_device_type(uint8_t device_idx, ble_device_type_t device_type);

    /**
     * @brief Switch to next connected device (cycle through connected devices)
     * Useful for quick switching between multiple connected devices
     * @return New active device index on success, negative on error
     */
    int ble_dev_mgr_switch_to_next_device(void);

#ifdef RT_USING_BLUETOOTH
    extern void ble_app_set_bonded_device_addr(ble_gap_addr_t *addr);
#endif

    /**
     * @brief Start targeted advertising for specific bonded device
     * @param device_idx Device index in bonded devices database
     * @note This function sets the target device and starts directed advertising
     *       Only the specified device will be able to connect
     */
    extern void ble_app_start_targeted_advertising(uint8_t device_idx);

#ifdef __cplusplus
}
#endif

#endif // BLE_DEVICE_MANAGER_H
