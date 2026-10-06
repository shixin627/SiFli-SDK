/*
 * Copyright (c) 2026 Skaiwalk Technology
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SKAI_IMU_MEASUREMENT_WIRE_H
#define SKAI_IMU_MEASUREMENT_WIRE_H
#include <stdint.h>
#include <string.h>
#define IMU_MEASUREMENT_MAGIC 0x494d5531u
#define IMU_MEASUREMENT_HEADER 8
#define IMU_MEASUREMENT_SAMPLE 76
static void imu_wire_u32(uint8_t *p, uint32_t n)
{
    p[0] = (uint8_t)n; p[1] = (uint8_t)(n >> 8);
    p[2] = (uint8_t)(n >> 16); p[3] = (uint8_t)(n >> 24);
}
static uint32_t imu_wire_read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void imu_wire_float(uint8_t *p, float f)
{
    uint32_t bits; memcpy(&bits, &f, 4); imu_wire_u32(p, bits);
}
static void imu_wire_header(uint8_t *p, uint8_t kind, uint32_t session)
{
    p[0] = 'I'; p[1] = 'M'; p[2] = 1; p[3] = kind;
    imu_wire_u32(p + 4, session);
}
static void imu_wire_sample(uint8_t *p, uint32_t sequence, uint32_t ms,
                            uint16_t rate, const float values[16])
{
    imu_wire_u32(p, sequence); imu_wire_u32(p + 4, ms);
    p[8] = (uint8_t)rate; p[9] = (uint8_t)(rate >> 8);
    p[10] = 1; p[11] = 0;
    for (unsigned i = 0; i < 16; ++i) imu_wire_float(p + 12 + 4 * i, values[i]);
}
#endif
