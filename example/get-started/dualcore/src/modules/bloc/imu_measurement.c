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
#include "imu_measurement.h"
#include "imu_measurement_wire.h"
#include "communicate_task.h"
#include "communicate_protocol.h"
#include "communicate_parse.h"
#include "communicate_parse_notify.h"
#include <string.h>

/* Same BLE interface, separate key: legacy 0x50 always remains linear ACC.
 * 15-second renewable lease; 120-second absolute session cap. No persistent
 * sensor configuration changes, no recognition gates or thresholds touched. */
static volatile uint32_t s_session;
static volatile rt_tick_t s_deadline, s_started;
static volatile bool s_active;

void imu_measurement_command(const uint8_t *p, uint16_t n)
{
    uint8_t ack[10];
    if (!p || n != 8 || p[0] != 'I' || p[1] != 'M' || p[2] != 1 || p[3] > 1)
        return;
    uint32_t session = imu_wire_read_u32(p + 4);
    if (!session) return;
    rt_tick_t now = rt_tick_get();
    rt_base_t level = rt_hw_interrupt_disable();
    bool new_session = s_session != session;
    if (new_session) s_started = now;
    s_session = session;
    s_active = p[3] == 1 &&
        (rt_tick_t)(now - s_started) < rt_tick_from_millisecond(120000);
    s_deadline = now + rt_tick_from_millisecond(15000);
    bool active = s_active;
    rt_hw_interrupt_enable(level);
    imu_wire_header(ack, 0, session);
    ack[8] = active ? 1 : 0; ack[9] = 0; /* 0 accepted, data still required */
    skaiwatch_ble_send_l2(NOTIFY_COMMAND_ID, KEY_IMU_MEASUREMENT_SAMPLE, ack, sizeof(ack));
}

void imu_measurement_feed(const motion_data_t *s)
{
    static uint8_t buffer[IMU_MEASUREMENT_HEADER + 5 * IMU_MEASUREMENT_SAMPLE];
    static uint32_t buffered_session, previous_sequence;
    static unsigned count;
    rt_tick_t now = rt_tick_get();
    rt_base_t level = rt_hw_interrupt_disable();
    uint32_t session = s_session;
    bool active = s_active && (int32_t)(s_deadline - now) > 0 &&
        (rt_tick_t)(now - s_started) < rt_tick_from_millisecond(120000);
    if (!active) s_active = false;
    rt_hw_interrupt_enable(level);
    if (!active || s->measurement_magic != IMU_MEASUREMENT_MAGIC)
    {
        count = 0; buffered_session = 0; return;
    }
    if (buffered_session != session)
    {
        count = 0; buffered_session = session;
        previous_sequence = s->measurement_sequence - 1;
    }
    if (previous_sequence == s->measurement_sequence) return;
    previous_sequence = s->measurement_sequence;
    uint8_t *p = buffer + IMU_MEASUREMENT_HEADER + count * IMU_MEASUREMENT_SAMPLE;
    float values[16] = {
        s->fusion_acc.x, s->fusion_acc.y, s->fusion_acc.z,
        s->fusion_gyro.x, s->fusion_gyro.y, s->fusion_gyro.z,
        s->linear_acce.x, s->linear_acce.y, s->linear_acce.z,
        s->gravity.x, s->gravity.y, s->gravity.z,
        s->global_q.w, s->global_q.x, s->global_q.y, s->global_q.z
    };
    imu_wire_sample(p, s->measurement_sequence, s->timestamp, s->measurement_hz, values);
    if (++count == 5)
    {
        imu_wire_header(buffer, 1, session);
        skaiwatch_ble_send_l2(NOTIFY_COMMAND_ID, KEY_IMU_MEASUREMENT_SAMPLE, buffer, sizeof(buffer));
        count = 0;
    }
}
