/*
 * Copyright (c) 2026 Skaiwalk Technology
 * SPDX-License-Identifier: Apache-2.0
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
