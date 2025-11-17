#ifndef _BATTERY_CALCULATOR_H_
#define _BATTERY_CALCULATOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "battery_table.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup Battery_Calculator_Error_Code_definition Battery calculator Error Code definition
  * @brief Battery calculator Error Code definition
  * @{
  */
#define BATTERY_CALC_SUCCESS                0   /**< Success */
#define BATTERY_CALC_ERR_INVALID_PARAM     -1   /**< Invalid parameter error */
#define BATTERY_CALC_ERR_INVALID_TABLE     -2   /**< Invalid curve table error */
/**
  * @}
  */

/**
 * @brief Battery charging status enumeration.
 */
enum battery_charger_status
{
    BATTERY_CHARGER_STATUS_DISCHARGING = 0,  /**< The battery is discharging. */
    BATTERY_CHARGER_STATUS_CHARGING = 1      /**< The battery is charging. */
};
typedef enum battery_charger_status battery_charger_status_t;

/**
 * @brief Battery calculator configuration structure.
 */
struct battery_calculator_config
{
    const battery_lookup_point_t *charging_table;    /**< Charging curve table. */
    uint32_t charging_table_size;                    /**< Charging curve table size. */
    const battery_lookup_point_t *discharging_table; /**< Discharging curve table. */
    uint32_t discharging_table_size;                 /**< Discharging curve table size. */
    uint32_t charge_filter_threshold;                /**< Charging voltage change filter threshold in mV. */
    uint32_t discharge_filter_threshold;             /**< Discharging voltage change filter threshold in mV. */
    uint8_t filter_count;                            /**< Filter count threshold. */
    bool secondary_filter_enabled;                   /**< Enable secondary filter. */
    uint8_t secondary_filter_weight_pre;             /**< Weight for previous voltage (default 80). */
    uint8_t secondary_filter_weight_cur;             /**< Weight for current voltage (default 20). */
};
typedef struct battery_calculator_config battery_calculator_config_t;

/**
 * @brief Battery calculator handle structure.
 */
struct battery_calculator
{
    const battery_calculator_config_t *config;       /**< Battery calculator configuration. */
    uint8_t discharge_filter_count;                  /**< Discharge filter count. */
    uint8_t charge_filter_count;                     /**< Charge filter count. */
    uint32_t last_voltage;                           /**< Last voltage value in mV. */
    uint8_t last_percent;                            /**< Last battery percent. */
    int8_t last_status;                              /**< Last charging status. */
    uint32_t pre_voltage;                            /**< Previous voltage for secondary filter. */
};
typedef struct battery_calculator battery_calculator_t;

/**
 * @brief Initialize battery calculator.
 *
 * @param calculator is a pointer to battery calculator handle.
 * @param config is a pointer to battery calculator configuration.
 *
 * @return Returns BATTERY_CALC_SUCCESS on success, otherwise returns error code.
 */
int battery_calculator_init(battery_calculator_t *calculator, const battery_calculator_config_t *config);

/**
 * @brief Calculate battery percentage based on voltage.
 *
 * @param calculator is a pointer to battery calculator handle.
 * @param voltage is the battery voltage value in mV.
 *
 * @return Returns battery percentage (0-100).
 */
uint8_t battery_calculator_get_percent(battery_calculator_t *calculator, uint32_t voltage);

/**
 * @brief Find the corresponding battery percentage from the curve table.
 *
 * @param table is a pointer to the curve table.
 * @param table_size is the size of curve table.
 * @param voltage is the voltage value in mV.
 *
 * @return Returns battery percentage.
 */
uint32_t battery_percent_from_curve_table(const battery_lookup_point_t *table, uint32_t table_size, uint32_t voltage);

/**
 * @brief Get the current charging status.
 *
 * @return Returns current charging status.
 */
battery_charger_status_t battery_get_charging_status(void);

#ifdef __cplusplus
}
#endif

#endif /* _BATTERY_CALCULATOR_H_ */