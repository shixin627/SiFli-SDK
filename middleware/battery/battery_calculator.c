#include "battery_calculator.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "drv_gpio.h"

#define DBG_TAG "battery"
#include <rtdbg.h>

/**
 * @brief Find the corresponding battery percentage from the curve table.
 *
 * @param table is a pointer to the curve table.
 * @param table_size is the size of curve table.
 * @param voltage is the voltage value in mV.
 *
 * @return Returns battery percentage.
 */
uint32_t battery_percent_from_curve_table(const battery_lookup_point_t *table, uint32_t table_size, uint32_t voltage)
{
    uint32_t index;
    const battery_lookup_point_t *p_cur = table;

    if (voltage >= p_cur->voltage)
    {
        return p_cur->percent;
    }

    for (index = 0; index < table_size; index++)
    {
        if (voltage > p_cur->voltage)
        {
            break;
        }
        p_cur++;
    }

    if (index == table_size)
    {
        p_cur--;
        return p_cur->percent;
    }

    const battery_lookup_point_t *p_pre = p_cur - 1;

    return p_cur->percent
           + ((p_pre->percent - p_cur->percent)
              * (voltage - p_cur->voltage)
              / (p_pre->voltage - p_cur->voltage));
}

/**
 * @brief Primary voltage filter based on charging status.
 *
 * @param calculator is a pointer to battery calculator handle.
 * @param voltage is the current voltage value in mV.
 * @param status is the current charging status.
 *
 * @return Returns filtered voltage value in mV.
 */
static uint32_t _battery_voltage_filter(battery_calculator_t *calculator, uint32_t voltage, battery_charger_status_t status)
{
    int diff_vol = 0;
    uint32_t temp_voltage = voltage;

    if (calculator->last_voltage == 0)
    {
        calculator->last_voltage = voltage;
    }

    /* Reset all states and filters when charging status changes */
    if (calculator->last_status != (int8_t)status)
    {
        calculator->last_status = (int8_t)status;
        calculator->charge_filter_count = 0;
        calculator->discharge_filter_count = 0;
        calculator->last_voltage = voltage;
        calculator->pre_voltage = 0;
        return voltage;
    }

    diff_vol = abs((int)(voltage - calculator->last_voltage));

    if (BATTERY_CHARGER_STATUS_CHARGING == status)
    {
        calculator->discharge_filter_count = 0;
        if ((voltage >= calculator->last_voltage) && (diff_vol < calculator->config->charge_filter_threshold))
        {
            calculator->last_voltage = voltage;
            calculator->charge_filter_count = 0;
            return temp_voltage;
        }

        calculator->charge_filter_count++;
        temp_voltage = calculator->last_voltage;
        if (calculator->charge_filter_count >= calculator->config->filter_count)
        {
            calculator->charge_filter_count = 0;
            calculator->last_voltage = voltage;
        }
        return temp_voltage;
    }
    else
    {
        calculator->charge_filter_count = 0;

        /* Voltage rising: do not update immediately unless threshold is exceeded multiple times */
        if (voltage > calculator->last_voltage)
        {
            calculator->discharge_filter_count++;
            temp_voltage = calculator->last_voltage;
            if (calculator->discharge_filter_count >= calculator->config->filter_count)
            {
                calculator->discharge_filter_count = 0;
                calculator->last_voltage = voltage;
            }
            return temp_voltage;
        }

        /* Voltage dropping within threshold: update normally */
        if ((voltage <= calculator->last_voltage) && (diff_vol < calculator->config->discharge_filter_threshold))
        {
            calculator->last_voltage = voltage;
            calculator->discharge_filter_count = 0;
            return temp_voltage;
        }

        /* Voltage dropping exceeds threshold */
        calculator->discharge_filter_count++;
        temp_voltage = calculator->last_voltage;
        if (calculator->discharge_filter_count >= calculator->config->filter_count)
        {
            calculator->discharge_filter_count = 0;
            calculator->last_voltage = voltage;
        }
        return temp_voltage;
    }
}

/**
 * @brief Secondary voltage filter using weighted average.
 *
 * @param calculator is a pointer to battery calculator handle.
 * @param voltage is the voltage value in mV after primary filter.
 *
 * @return Returns filtered voltage value in mV.
 */
static uint32_t _battery_secondary_filter(battery_calculator_t *calculator, uint32_t voltage)
{
    if (calculator->config->secondary_filter_enabled)
    {
        /* Initialize pre_voltage for the first use */
        if (calculator->pre_voltage == 0)
        {
            calculator->pre_voltage = voltage;
            return voltage;
        }

        /* Calculate the weighted average voltage */
        calculator->pre_voltage = (calculator->pre_voltage * calculator->config->secondary_filter_weight_pre +
                                   voltage * calculator->config->secondary_filter_weight_cur) /
                                  (calculator->config->secondary_filter_weight_pre + calculator->config->secondary_filter_weight_cur);
        return calculator->pre_voltage;
    }

    /* Return directly if secondary filter is not enabled */
    return voltage;
}

/**
 * @brief Initialize battery calculator.
 *
 * @param calculator is a pointer to battery calculator handle.
 * @param config is a pointer to battery calculator configuration.
 *
 * @return Returns BATTERY_CALC_SUCCESS on success, otherwise returns error code.
 */
int battery_calculator_init(battery_calculator_t *calculator, const battery_calculator_config_t *config)
{
    if (calculator == NULL || config == NULL)
    {
        return BATTERY_CALC_ERR_INVALID_PARAM;
    }

    if (config->charging_table == NULL || config->discharging_table == NULL)
    {
        return BATTERY_CALC_ERR_INVALID_TABLE;
    }

    calculator->config = config;
    calculator->discharge_filter_count = 0;
    calculator->charge_filter_count = 0;
    calculator->last_voltage = 0;
    calculator->last_percent = 0;
    calculator->last_status = -1;
    calculator->pre_voltage = 0;

    return BATTERY_CALC_SUCCESS;
}

/**
 * @brief Get the current charging status.
 *
 * @return Returns current charging status.
 */
battery_charger_status_t battery_get_charging_status(void)
{
    return rt_pin_read(CHARGE_PIN) ? BATTERY_CHARGER_STATUS_CHARGING : BATTERY_CHARGER_STATUS_DISCHARGING;
}

/**
 * @brief Calculate battery percentage based on voltage.
 *
 * @param calculator is a pointer to battery calculator handle.
 * @param voltage is the battery voltage value in mV.
 *
 * @return Returns battery percentage (0-100).
 */
uint8_t battery_calculator_get_percent(battery_calculator_t *calculator, uint32_t voltage)
{
    uint8_t percent;
    uint32_t filtered_voltage;
    uint32_t secondary_filtered_voltage;
    battery_charger_status_t status;

    if (calculator == NULL || calculator->config == NULL)
    {
        return 0;
    }

    /* Get the charging status */
    status = battery_get_charging_status();
    LOG_D("Current status: %s", (status == BATTERY_CHARGER_STATUS_CHARGING) ? "Charging" : "Discharging");

    /* Primary filter: state-based filtering */
    filtered_voltage = _battery_voltage_filter(calculator, voltage, status);

    /* Secondary filter: weighted average filtering */
    secondary_filtered_voltage = _battery_secondary_filter(calculator, filtered_voltage);

    if (BATTERY_CHARGER_STATUS_CHARGING == status)
    {
        percent = battery_percent_from_curve_table(
                      calculator->config->charging_table,
                      calculator->config->charging_table_size,
                      secondary_filtered_voltage);
    }
    else
    {
        percent = battery_percent_from_curve_table(
                      calculator->config->discharging_table,
                      calculator->config->discharging_table_size,
                      secondary_filtered_voltage);
    }

    /* Battery percentage change control */
    if (calculator->last_percent != 0)
    {
        int percent_diff = abs((int)percent - (int)calculator->last_percent);
        bool status_changed = (calculator->last_status != (int8_t)status && calculator->last_status != -1);

        /* Handle status change */
        if (status_changed)
        {
            LOG_D("Status changed, old: %d%%, new: %d%%", calculator->last_percent, percent);

            /* Allow moderate jumps during status changes, but limit the amplitude */
            if (percent_diff > 8)
            {
                /* Jump too large, smooth the percentage */
                if (percent > calculator->last_percent)
                {
                    percent = calculator->last_percent + 1;
                }
                else
                {
                    percent = calculator->last_percent - 1;
                }
                LOG_D("Status change jump too large, smoothed to: %d%%", percent);
            }
        }
        else  /* Handle within same status */
        {
            /* Check if the direction of percentage change is reasonable */
            if (BATTERY_CHARGER_STATUS_CHARGING == status)
            {
                /* Charging: percentage should normally increase */
                if (percent < calculator->last_percent)
                {
                    /* Decreasing anomaly, keep current percentage */
                    percent = calculator->last_percent;
                }
                else if (percent_diff > 2)
                {
                    /* Charging too fast, limit the increase rate */
                    percent = calculator->last_percent + 1;
                }
            }
            else
            {
                /* Discharging: percentage should normally decrease */
                if (percent > calculator->last_percent)
                {
                    /* Increasing anomaly, keep current percentage */
                    percent = calculator->last_percent;
                }
                else if (percent_diff > 2)
                {
                    /* Discharging too fast, limit the decrease rate */
                    percent = calculator->last_percent - 1;
                }
            }
        }
    }

    calculator->last_percent = percent;
    calculator->last_status = status;

    return percent;
}