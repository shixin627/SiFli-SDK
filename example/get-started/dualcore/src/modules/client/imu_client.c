/**
 ******************************************************************************
 * @file   imu_client.c
 * @author Skaiwalk software development team
 * @brief imu client source.
 *
 ******************************************************************************
 */
/**
 * @attention
 * Copyright (c) 2018 - 2023,  Skaiwalk Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>
#include "data_service_provider.h"
#include "sensor.h"
#ifdef BSP_USING_BLOC_PERIPHERAL
#include "bloc_peripheral.h"
#endif
#ifdef BSP_USING_WATCH_SYS_CLIENT
#include "watch_sys_service.h"
#endif
#include "watch_system_interact.h"

#define DBG_TAG "CLIENT.IMU"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

static datac_handle_t accel_service_handle = DATA_CLIENT_INVALID_HANDLE;
static datac_handle_t gyro_service_handle = DATA_CLIENT_INVALID_HANDLE;
static datac_handle_t mag_service_handle = DATA_CLIENT_INVALID_HANDLE;

static int imu_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        uint16_t len;
        uint8_t *buffer;
        RT_ASSERT(arg->data);
        len = arg->data_len;

        buffer = arg->data;
        motion_sensor_data_t object = *(motion_sensor_data_t *)buffer;
        process_motion_sensor_data(&object);
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (rsp->result == 0)
        {
            LOG_I("Subscribe IMU id(%d), result(%d) => success\r\n", arg->msg_id, rsp->result);
            watch_sensor.imu_client_num++;
            // LOG_D("Subscribe IMU client_num:%d\n", watch_sensor.imu_client_num);
        }
        else
        {
            LOG_E("Subscribe IMU id(%d), result(%d) => fail\r\n", arg->msg_id, rsp->result);
        }
    }
    else if (MSG_SERVICE_UNSUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (rsp->result == 0)
        {
            LOG_I("Unsubscribed IMU id(%d), result(%d) => success\r\n", arg->msg_id, rsp->result);
            watch_sensor.imu_client_num--;
            // LOG_D("Unsubscribe IMU client_num:%d\n", watch_sensor.imu_client_num);
            // if (accel_service_handle != DATA_CLIENT_INVALID_HANDLE)
            // {
            //     if (RT_EOK != datac_close(accel_service_handle))
            //     {
            //         LOG_E("accel_service_handle close failed\n");
            //         return -RT_ERROR;
            //     }
            //     accel_service_handle = DATA_CLIENT_INVALID_HANDLE;
            //     watch_sensor.imu_client_num = 0;
            // }
        }
        else
        {
            LOG_E("Unsubscribed IMU id(%d), result(%d) => fail\r\n", arg->msg_id, rsp->result);
        }
    }
    else
    {
        LOG_E("imu_callback message id(%d) not support\r\n", arg->msg_id);
    }
    return RT_EOK;
}

bool check_imu_sensor(void)
{
#ifdef BSP_USING_WATCH_SYS_CLIENT
    if (!watch_sys_sync.is_imu_enabled())
    {
        LOG_E("imu sensor is not enabled\n");
        return false;
    }
#endif
    return true;
}

bool accel_service_subscribed(void)
{
    return (accel_service_handle != DATA_CLIENT_INVALID_HANDLE) && (watch_sensor.imu_client_num > 0);
}

void accelerometer_subscribe(void)
{
    if (accel_service_subscribed())
    {
        LOG_I("accel service is subscribed\n");
        return;
    }
    // RT_ASSERT(check_imu_sensor());
    if (accel_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        accel_service_handle = datac_open();
    }
    // RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != accel_service_handle);
    if (accel_service_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_subscribe(accel_service_handle, "ACCE", imu_callback, 0);
        LOG_I("Subscribe IMU\n");
    }
}

void accelerometer_unsubscribe(void)
{
    if (accel_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("accel service is not created\n");
        return;
    }
    if (datac_unsubscribe(accel_service_handle) == RT_EOK)
    {
        LOG_I("Unsubscribed IMU\n");
    }
    else
    {
        LOG_E("Unsubscribed IMU failed\n");
    }
}

void gyroscope_subscribe(void)
{
    if (gyro_service_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("gyro service is subscribed\n");
        return;
    }
    gyro_service_handle = datac_open();
    // RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != gyro_service_handle);
    if (gyro_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("gyro_service_handle is invalid\n");
        return;
    }
    datac_subscribe(gyro_service_handle, "GYRO", imu_callback, 0);
}

void gyroscope_unsubscribe(void)
{
    if (gyro_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("gyro_service_handle is invalid\n");
        return;
    }
    datac_unsubscribe(gyro_service_handle);
    datac_close(gyro_service_handle);
    gyro_service_handle = DATA_CLIENT_INVALID_HANDLE;
}

bool mag_service_subscribed(void)
{
    return mag_service_handle != DATA_CLIENT_INVALID_HANDLE;
}
void magnetometer_subscribe(void)
{
    if (mag_service_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("mag service is subscribed\n");
        return;
    }
    mag_service_handle = datac_open();
    // RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != mag_service_handle);
    if (mag_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("mag_service_handle is invalid\n");
        return;
    }
    datac_subscribe(mag_service_handle, "MAG", imu_callback, 0);
}

void magnetometer_unsubscribe(void)
{
    if (mag_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("mag_service_handle is invalid\n");
        return;
    }
    datac_unsubscribe(mag_service_handle);
    datac_close(mag_service_handle);
    mag_service_handle = DATA_CLIENT_INVALID_HANDLE;
}

//**********************************************************************************************

MSH_CMD_EXPORT(accelerometer_subscribe, Subscribe accelerometer data service);
MSH_CMD_EXPORT(gyroscope_subscribe, Subscribe gyroscope data service);
MSH_CMD_EXPORT(accelerometer_unsubscribe, Unsubscribe accelerometer data service);
MSH_CMD_EXPORT(gyroscope_unsubscribe, Unsubscribe gyroscope data service);
MSH_CMD_EXPORT(magnetometer_subscribe, Subscribe magnetometer data service);
MSH_CMD_EXPORT(magnetometer_unsubscribe, Unsubscribe magnetometer data service);
