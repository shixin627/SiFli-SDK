#ifndef SKAI_IMU_MEASUREMENT_H
#define SKAI_IMU_MEASUREMENT_H
#include "bloc_peripheral.h"
/* Factory 0x06/0x02: IM, v1, mode(0 stop/1 measure), session u32 LE.
 * Notify 0x04/0x51: IM, v1, kind(0 status/1 samples), session u32 LE.
 * Never changes IMU configuration or the legacy 0x50 stream. */
void imu_measurement_command(const uint8_t *payload, uint16_t length);
void imu_measurement_feed(const motion_data_t *sample);
#endif
