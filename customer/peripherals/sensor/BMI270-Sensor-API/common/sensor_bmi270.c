/**
 ******************************************************************************
 * @file   sensor_bmi270.c
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

#include "sensor_bmi270.h"
#if defined(GSENSOR_UES_FIFO)
#include "sensor_service.h"
#endif
#ifdef BSP_USING_MAHONY_AHRS
#include "sensor_fusion.h"
#endif
#ifdef BSP_USING_BLOC_PERIPHERAL
#include "bloc_peripheral.h"
#endif
#ifdef BSP_USING_HAND_TRACKING
#include "gesture_detect.h"
#endif

#ifdef RT_USING_SENSOR
#undef DBG_TAG
#define DBG_TAG "sensor.bmi270"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#define ACCE_MODEL_NAME "bmi270"
#define ACCE_DEV_NAME "acce_" ACCE_MODEL_NAME
static struct bmi270_device *bmi_dev = RT_NULL;
static rt_sensor_t sensor_acce = RT_NULL;
static rt_sensor_t sensor_gyro = RT_NULL;

static rt_err_t _bmi270_init(void)
{
    if (bmi270_initialized() == RT_EOK)
    {
        bmi_dev = rt_calloc(1, sizeof(struct bmi270_device));
        if (bmi_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        bmi_dev->bus = (rt_device_t)bmi270_get_bus_handle();
        bmi_dev->i2c_addr = bmi270_get_dev_addr();
        bmi_dev->id = bmi270_get_dev_id();
        // mpu_dev->config;
        // bmi270_open();
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _bmi270_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        uint8_t range_ctr;

        if (range <= 2000)
            range_ctr = BMI2_ACC_RANGE_2G;
        else if (range <= 4000)
            range_ctr = BMI2_ACC_RANGE_4G;
        else if (range <= 8000)
            range_ctr = BMI2_ACC_RANGE_8G;
        else
            range_ctr = BMI2_ACC_RANGE_16G;

        LOG_D("acce set range %d", range_ctr);

        bmi270_accel_set_range(range_ctr);
        bmi_dev->config.accel_range = range_ctr;
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        uint8_t range_ctr;

        if (range <= 125000UL)
            range_ctr = BMI2_GYR_RANGE_125;
        else if (range <= 250000UL)
            range_ctr = BMI2_GYR_RANGE_250;
        else if (range <= 500000UL)
            range_ctr = BMI2_GYR_RANGE_500;
        else if (range <= 1000000UL)
            range_ctr = BMI2_GYR_RANGE_1000;
        else
            range_ctr = BMI2_GYR_RANGE_2000;

        LOG_D("gyro set range %d", range);

        bmi270_gyro_set_range(range_ctr);
        bmi_dev->config.gyro_range = range_ctr;
    }
    return RT_EOK;
}

static rt_err_t _bmi270_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

void bmi270_sensor_power_high_mode(void)
{
    if (watch_sensor.imu_data.sample_rate > IMU_SLEEPING_SAMPLE_RATE)
    {
        return;
    }
#ifdef USE_IMU_REPORT_TIMER
    motion_data_report_timer_stop();
#endif
    rt_bmi270_irq_pin_enable(0);
    bmi270_high_performance_mode();
#ifdef BSP_USING_MAHONY_AHRS
    setSampleFrequencyAHRS(IMU_NOARMAL_SAMPLE_RATE);
#endif
    watch_sensor.imu_data.sample_rate = IMU_NOARMAL_SAMPLE_RATE;
    rt_bmi270_irq_pin_enable(1);
#ifdef USE_IMU_REPORT_TIMER
    motion_data_report_timer_start(rt_tick_from_millisecond(IMU_NOARMAL_PERIOD));
#endif
}

void bmi270_sensor_power_low_mode(void)
{
    if (watch_sensor.imu_data.sample_rate == IMU_SLEEPING_SAMPLE_RATE)
    {
        return;
    }
#ifdef USE_IMU_REPORT_TIMER
    motion_data_report_timer_stop();
#endif
    rt_bmi270_irq_pin_enable(0);
    bmi270_low_power_mode();
#ifdef BSP_USING_MAHONY_AHRS
    setSampleFrequencyAHRS(IMU_SLEEPING_SAMPLE_RATE);
#endif
    watch_sensor.imu_data.sample_rate = IMU_SLEEPING_SAMPLE_RATE;
    // 因為在這段期間3.3V開關電源會關閉，可能會產生雜訊干擾I2C總線，所以等待一段時間
    rt_thread_mdelay(100);
    rt_bmi270_irq_pin_enable(1);
#ifdef USE_IMU_REPORT_TIMER
    motion_data_report_timer_start(rt_tick_from_millisecond(IMU_SLEEPING_PERIOD));
#endif
}

static rt_err_t _bmi270_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    LOG_I("_bmi270_set_power: %d\n", power);
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        bmi270_close();
        break;
    case RT_SENSOR_POWER_NORMAL:
        bmi270_open();
        bmi270_sensor_power_low_mode();
        break;
    case RT_SENSOR_POWER_LOW:
        bmi270_sensor_power_low_mode();
        break;
    case RT_SENSOR_POWER_HIGH:
        bmi270_sensor_power_high_mode();
        break;
    default:
        break;
    }
    return RT_EOK;
}

static rt_err_t _bmi270_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    // LOG_I("_bmi270_self_test with mode %d\n", mode);
    res = bmi270_self_check();
    if (res != 0)
    {
        LOG_I("_bmi270_self_test selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

#if defined(GSENSOR_UES_FIFO)

static int _bmi270_read_all_fifo_data(uint8_t *buf, int len)
{
    uint8_t data[2];
    int pattern_id;
    uint16_t fcount[6] = {0};
    sensors_service_fifo_t *f_buf = (sensors_service_fifo_t *)buf;
    rt_memset(&f_buf->acce_fifo->acce_x[0], 0, sizeof(sensors_service_acce_t));
#ifdef USING_GYRO_SENSOR
    rt_memset(&f_buf->gyro_fifo->gyro_x[0], 0, sizeof(sensors_service_gyro_t));
#endif
    for (int i = 0; i < len; i++)
    {
        pattern_id = bmi270_get_fifo_pattern();
        LOG_D("bmi270_get_fifo_pattern: ret %d", pattern_id);

        bmi270_read_fifo(&data[0], sizeof(data));

        switch (pattern_id)
        {
#ifdef USING_GYRO_SENSOR
        case BMI270_FIFO_PATTERN_GX1:
        {
            f_buf->gyro_fifo->gyro_x[fcount[0]] = (int16_t)((data[1] << 8) | (data[0]));
            // LOG_D("BMI270_FIFO_PATTERN_GX1:0x%x 0x%x %d %d",data[0],data[1],f_buf->gyro_fifo->gyro_x[fcount[0]],fcount[0]);
            fcount[0]++;
            break;
        }
        case BMI270_FIFO_PATTERN_GY2:
        {
            f_buf->gyro_fifo->gyro_y[fcount[1]] = (int16_t)((data[1] << 8) | (data[0]));
            // LOG_D("BMI270_FIFO_PATTERN_GY1:0x%x 0x%x %d %d",data[0],data[1],f_buf->gyro_fifo->gyro_y[fcount[1]],fcount[1]);
            fcount[1]++;
            break;
        }
        case BMI270_FIFO_PATTERN_GZ3:
        {
            f_buf->gyro_fifo->gyro_z[fcount[2]] = (int16_t)((data[1] << 8) | (data[0]));
            // LOG_D("BMI270_FIFO_PATTERN_GZ1:0x%x 0x%x %d %d",data[0],data[1],f_buf->gyro_fifo->gyro_z[fcount[2]],fcount[2]);
            fcount[2]++;
            break;
        }
#endif
        case BMI270_FIFO_PATTERN_XLX1:
        {
            f_buf->acce_fifo->acce_x[fcount[3]] = (int16_t)((data[1] << 8) | (data[0]));
            // LOG_D("BMI270_FIFO_PATTERN_XLX1:0x%x 0x%x %d %d",data[0],data[1],f_buf->acce_fifo->acce_x[fcount[3]],fcount[3]);
            fcount[3]++;

            break;
        }
        case BMI270_FIFO_PATTERN_XLY2:
        {
            f_buf->acce_fifo->acce_y[fcount[4]] = (int16_t)((data[1] << 8) | (data[0]));
            // LOG_D("BMI270_FIFO_PATTERN_XLY1:0x%x 0x%x %d %d",data[0],data[1],f_buf->acce_fifo->acce_y[fcount[4]],fcount[4]);
            fcount[4]++;
            break;
        }
        case BMI270_FIFO_PATTERN_XLZ3:
        {
            f_buf->acce_fifo->acce_z[fcount[5]] = (int16_t)((data[1] << 8) | (data[0]));
            // LOG_D("BMI270_FIFO_PATTERN_XLZ1:0x%x 0x%x %d %d",data[0],data[1],f_buf->acce_fifo->acce_z[fcount[5]],fcount[5]);
            fcount[5]++;
            break;
        }
        default:;
        }
    }

    return 0;
}
#endif

static rt_size_t _bmi270_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    // LOG_D("_bmi270_polling_get_data type %d 0x%x 0x%x", sensor->info.type, sensor, data);
#if defined(GSENSOR_UES_FIFO)
    int32_t fifo_data_len;
    // int waterm, over_run, full, empty;
    int16_t x, y, z;
    sensors_service_fifo_t *buf_fifo = (sensors_service_fifo_t *)data;

    // waterm = bmi270_get_waterm_status();
    // over_run = bmi270_get_overrun_status();
    // full = bmi270_get_fifo_full_status();
    // empty = bmi270_get_fifo_empty_status();
    // LOG_D("bmi270_get_waterm_status:waterm %d %d %d %d",waterm,over_run,full,empty);

    fifo_data_len = bmi270_get_fifo_count();
    // LOG_D("bmi270_read_fifo:len %d",fifo_data_len);

#ifdef USING_GYRO_SENSOR
    if (fifo_data_len <= GSENSOR_FIFO_SIZE * 6)
#else
    if (fifo_data_len <= GSENSOR_FIFO_SIZE * 3)
#endif
    {
        _bmi270_read_all_fifo_data((uint8_t *)buf_fifo, fifo_data_len);
        buf_fifo->total_count = fifo_data_len;
    }

    bmi270_set_fifo_mode(BMI270_BYPASS_MODE);
    bmi270_set_fifo_mode(BMI270_FIFO_MODE);

    return (uint32_t)buf_fifo->total_count;
#else

    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        data->type = RT_SENSOR_CLASS_ACCE;
        struct bmi2_sens_axes_data *accel = bmi270_get_accel();
        data->data.acce.x = (int32_t)accel->x;
        data->data.acce.y = (int32_t)accel->y;
        data->data.acce.z = (int32_t)accel->z;
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        data->type = RT_SENSOR_CLASS_GYRO;
        struct bmi2_sens_axes_data *gyro = bmi270_get_gyro();
        data->data.gyro.x = (int32_t)gyro->x;
        data->data.gyro.y = (int32_t)gyro->y;
        data->data.gyro.z = (int32_t)gyro->z;
        data->timestamp = rt_sensor_get_ts(); // gyro->virt_sens_time; // rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_STEP)
    {
        int32_t step;
        bmi270_step_read(&step);
        data->type = RT_SENSOR_CLASS_STEP;
        data->data.step = (uint32_t)step;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
#endif
}

static rt_size_t bmi270_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
        return _bmi270_polling_get_data(sensor, buf);

    return 0;
}

static rt_err_t bmi270_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = bmi_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _bmi270_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        // result = _bmi270_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _bmi270_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        // result = _bmi270_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
    {
        bmi270_fetch_data,
        bmi270_control};

int rt_hw_bmi270_register(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -1;

        sensor_acce->info.type = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor = RT_SENSOR_VENDOR_STM;
        sensor_acce->info.model = "bmi270_acc";
        sensor_acce->info.unit = RT_SENSOR_UNIT_MG;
#if (BMI270_USING_I2C == 1)
        sensor_acce->info.intf_type = RT_SENSOR_INTF_I2C;
#else
        sensor_acce->info.intf_type = RT_SENSOR_INTF_SPI;
#endif
        sensor_acce->info.range_max = 16000;
        sensor_acce->info.range_min = 2000;
        sensor_acce->info.period_min = 5;

        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        sensor_acce->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

#if 0
    /* gyroscope sensor register */
    {
        sensor_gyro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_gyro == RT_NULL)
            goto __exit;

        sensor_gyro->info.type = RT_SENSOR_CLASS_GYRO;
        sensor_gyro->info.vendor = RT_SENSOR_VENDOR_STM;
        sensor_gyro->info.model = "bmi270_gyro";
        sensor_gyro->info.unit = RT_SENSOR_UNIT_MDPS;
#if (BMI270_USING_I2C == 1)
        sensor_acce->info.intf_type = RT_SENSOR_INTF_I2C;
#else
        sensor_acce->info.intf_type = RT_SENSOR_INTF_SPI;
#endif
        sensor_gyro->info.range_max = 2000000;
        sensor_gyro->info.range_min = 250000;
        sensor_gyro->info.period_min = 5;

        rt_memcpy(&sensor_gyro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_gyro->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_gyro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
#endif

#if 0
    /* step sensor register */
    {
        sensor_step = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_step == RT_NULL)
            goto __exit;

        sensor_step->info.type       = RT_SENSOR_CLASS_STEP;
        sensor_step->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_step->info.model      = "bmi270_step";
        sensor_step->info.unit       = RT_SENSOR_UNIT_ONE;
        sensor_step->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_step->info.range_max  = 200000;
        sensor_step->info.range_min  = 1;
        sensor_step->info.period_min = 0;

        rt_memcpy(&sensor_step->config, cfg, sizeof(struct rt_sensor_config));
        sensor_step->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_step, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
#endif
    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_acce)
    {
        rt_free(sensor_acce);
        sensor_acce = RT_NULL;
    }
    if (sensor_gyro)
    {
        rt_free(sensor_gyro);
        sensor_gyro = RT_NULL;
    }
#if 0
    if (sensor_step)
        rt_free(sensor_step);
#endif
    return -RT_ERROR;
}

int rt_hw_bmi270_init(void)
{
    rt_int8_t result;

    result = _bmi270_init();
    if (result != RT_EOK)
    {
        LOG_E("bmi270 init err code: %d", result);
        if (bmi_dev)
        {
            rt_free(bmi_dev);
            bmi_dev = RT_NULL;
        }
    }

    return result;
}

int rt_hw_bmi270_deinit(void)
{
    int ret = RT_EOK;

    if (bmi_dev)
    {
        rt_free(bmi_dev);
        bmi_dev = RT_NULL;
    }
    return ret;
}

#endif // RT_USING_SENSOR
       /************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/