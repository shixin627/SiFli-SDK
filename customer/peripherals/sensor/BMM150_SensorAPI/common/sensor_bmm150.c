/**
 ******************************************************************************
 * @file   sensor_bmm150.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensor_bmm150.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.bmm150"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct bmm150_device *bmm_dev;

static rt_err_t _bmm150_init(void)
{
    if (bmm150_initialized() == 0)
    {
        bmm_dev = rt_calloc(1, sizeof(struct bmm150_device));
        if (bmm_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        bmm_dev->bus = (rt_device_t)bmm150_get_bus_handle();
        bmm_dev->i2c_addr = bmm150_get_dev_addr();
        bmm_dev->id = bmm150_get_dev_id();
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _bmm150_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    return RT_EOK;
}

static rt_err_t _bmm150_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        LOG_D("set mode to POLLING");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", mode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _bmm150_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    LOG_I("_bmm150_self_test with mode %d\n", mode);
    res = bmm150_self_check();
    if (res < 0)
    {
        LOG_I("bmm150 selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_err_t _bmm150_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        // MT register will auto clear after read, so disable seems no use
        bmm150_close();
        break;
    case RT_SENSOR_POWER_NORMAL:
        bmm150_open();
        break;
    case RT_SENSOR_POWER_LOW:
        break;
    case RT_SENSOR_POWER_HIGH:
        break;
    default:
        break;
    }
    return RT_EOK;
}

static rt_size_t _bmm150_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    int16_t mag[3];
    if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        bmm150_mag_read(&mag[0], &mag[1], &mag[2]);
        data->type = RT_SENSOR_CLASS_MAG;
        data->data.mag.x = (int32_t)(mag[0]);
        data->data.mag.y = (int32_t)(mag[1]);
        data->data.mag.z = (int32_t)(mag[2]);
        data->timestamp = rt_sensor_get_ts();
        LOG_I("mag x:%d y:%d z:%d\n", data->data.mag.x, data->data.mag.y, data->data.mag.z);
    }
    return 1;
}

static rt_size_t bmm150_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _bmm150_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t bmm150_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = bmm_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _bmm150_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _bmm150_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _bmm150_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _bmm150_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
    {
        bmm150_fetch_data,
        bmm150_control};

int rt_hw_bmm150_register(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_mag = RT_NULL;
    /* magnetometer/compass sensor register */
    {
        sensor_mag = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mag == RT_NULL)
            goto __exit;
        // 1 G = 1×10-4 T＝0.1 mT = 100 µT
        // 1 mG = 0.1 µT
        // Magnetic field range typical:
        // ±1300µT (x, y-axis), ±2500µT (z-axis)
        // Magnetic field resolution of ~0.3µT
        sensor_mag->info.type = RT_SENSOR_CLASS_MAG;
        sensor_mag->info.vendor = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_mag->info.model = "bmm150_mag";
        sensor_mag->info.unit = RT_SENSOR_UNIT_MGAUSS;
        sensor_mag->info.intf_type = RT_SENSOR_INTF_I2C;
        sensor_mag->info.range_max = 60000;  // 25000;
        sensor_mag->info.range_min = -60000; // -25000;
        sensor_mag->info.period_min = 4;

        rt_memcpy(&sensor_mag->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mag->ops = &sensor_ops;
        result = rt_hw_sensor_register(sensor_mag, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_mag)
        rt_free(sensor_mag);
    return -RT_ERROR;
}

int rt_hw_bmm150_init(void)
{
    rt_int8_t result;

    result = _bmm150_init();
    if (result != RT_EOK)
    {
        LOG_E("bmm150 init err code: %d", result);
        if (bmm_dev)
        {
            rt_free(bmm_dev);
            bmm_dev = RT_NULL;
        }
    }
    return result;
}

int rt_hw_bmm150_deinit(void)
{
    int ret = RT_EOK;

    if (bmm_dev)
    {
        rt_free(bmm_dev);
        bmm_dev = RT_NULL;
    }
    return ret;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
