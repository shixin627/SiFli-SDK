#ifndef _BATTERY_TABLE_H_
#define _BATTERY_TABLE_H_

#include <stdint.h>

/**
 * @brief Battery lookup point structure.
 */
struct battery_lookup_point
{
    uint8_t  percent; /**< Battery percentage. */
    uint32_t voltage; /**< Voltage value in mV. */
};
typedef struct battery_lookup_point battery_lookup_point_t;

extern const battery_lookup_point_t discharge_curve_table[];
extern const battery_lookup_point_t charging_curve_table[];
extern const uint32_t discharge_curve_table_size;
extern const uint32_t charging_curve_table_size;

#endif /* _BATTERY_TABLE_H_ */




