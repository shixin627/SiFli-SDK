/**
 * Copyright (c) 2026, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
