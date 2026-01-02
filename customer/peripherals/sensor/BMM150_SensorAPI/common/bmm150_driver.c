/**
 ******************************************************************************
 * @file   bmm150_driver.c
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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "bmm150_driver.h"
#include "board.h"
// #include "bf0_hal.h"
#ifdef MAG_USING_BMM150

/******************************************************************************/
/*!                Macro definition                                           */

#define DRV_DEBUG
#define LOG_TAG "drv.bmi"
#include <drv_log.h>
#define BMM150_POW_PIN 122
#define bmm150_DEV_NAME "bmm1_dev"

/*!
 * @brief  Structure to store the interface related configurations
 */
struct dev_info
{
    uint8_t open_flag; /* Flag to indicate interface is open or not */
    uint8_t dev_addr;  /* Device address or Chip select of the interface selected */
    /* Bus instance of the interface selected */
    struct rt_i2c_bus_device *bus_handle;
};

/* Sensor initialization configuration. */
static struct bmm150_dev bmm1_dev;
/*! Variable that holds the I2C device address */
static uint8_t dev_addr;

/*! Variable that holds the I2C or SPI bus instance */
struct dev_info mag_dev_info;

/******************************************************************************/
/*!          Function Declaration                                     */
void bmm150_error_codes_print_result(int8_t rslt);
#if (I2C_SOFT_ENABLE == 1)
static int8_t bmm1_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t ret;
    struct dev_info handle = *(struct dev_info *)intf_ptr;
    if (reg_data)
    {
        for (uint32_t i = 0; i < len; i++)
        {
            reg_data[i] = BSP_I2C_Read(reg_addr + i);
        }
        ret = RT_EOK;
    }
    else
    {
        ret = -3;
    }

    return ret;
}

static int8_t bmm1_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t ret;
    struct dev_info handle = *(struct dev_info *)intf_ptr;
    if (reg_data)
    {
        for (uint32_t i = 0; i < len; i++)
        {
            BSP_I2C_Write(reg_addr + i, reg_data[i]);
        }
        ret = RT_EOK;
    }
    else
    {
        ret = -3;
    }

    return ret;
}

#else
static int8_t bmm1_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t ret = RT_EOK;
    struct rt_i2c_msg msgs[2];
    uint32_t res;

    struct dev_info handle = *(struct dev_info *)intf_ptr;
    if (intf_ptr && handle.bus_handle && reg_data)
    {
        msgs[0].addr = handle.dev_addr; /* Slave address */
        msgs[0].flags = RT_I2C_WR;      /* Write flag */
        msgs[0].buf = &reg_addr;        /* Slave register address */
        msgs[0].len = 1;                /* Number of bytes sent */

        msgs[1].addr = handle.dev_addr; /* Slave address */
        msgs[1].flags = RT_I2C_RD;      /* Read flag */
        msgs[1].buf = reg_data;         /* Read data pointer */
        msgs[1].len = len;              /* Number of bytes read */

        res = rt_i2c_transfer(handle.bus_handle, msgs, 2);
        if (res == 2)
        {
            // LOG_D("i2c read reg=%06x, len=%d\n", reg_addr, len);
        }
        else
        {
            LOG_E("bmm1_i2c_read FAIL: %d, dev_addr=0x%x, reg=%06x, len=%d\n", ret, handle.dev_addr, reg_addr, len);
            ret = -2;
        }
    }
    else
    {
        ret = -3;
    }

    return ret;
}

static int8_t bmm1_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    rt_size_t res;
    struct dev_info handle = *(struct dev_info *)intf_ptr;
    if (intf_ptr && handle.bus_handle)
    {
        uint16_t addr16 = (uint16_t)reg_addr;
        res = rt_i2c_mem_write(handle.bus_handle, handle.dev_addr, addr16, 8, (void *)reg_data, len);
        // LOG_D("i2c write res = %d\n", res);
        if (res > 0)
        {
            return RT_EOK;
        }
        else
        {
            LOG_E("bmm1_i2c_write FAIL: %d, dev_addr=0x%x, reg=%06x, len=%d\n", res, handle.dev_addr, reg_addr, len);
            return -2;
        }
    }
    else
    {
        return -3;
    }
}
#endif

static void bmm1_delay_us(uint32_t period, void *intf_ptr)
{
    rt_thread_mdelay(period / 1000);
    // HAL_Delay_us(period);
}

static int8_t configure_sensor(struct bmm150_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;
    struct bmm150_settings settings;

    /* Set powermode as normal mode */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, dev);

    if (rslt == BMM150_OK)
    {
        /* Setting the preset mode as Low power mode
         * i.e. data rate = 10Hz, XY-rep = 1, Z-rep = 2
         */
        settings.preset_mode = BMM150_PRESETMODE_REGULAR;
        // rslt = bmm150_set_presetmode(&settings, dev);

        if (rslt == BMM150_OK)
        {
            /* Map the data interrupt pin */
            settings.int_settings.drdy_pin_en = 0x01;
            // rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev);
        }
    }
    return rslt;
}

static void bmm1_interface_init(struct bmm150_dev *bmm1)
{
    // bmm1->chip_id = BMM150_DEFAULT_I2C_ADDRESS;
    bmm1->read = bmm1_i2c_read;
    bmm1->write = bmm1_i2c_write;
    bmm1->delay_us = bmm1_delay_us;
    bmm1->intf = BMM150_I2C_INTF;
    dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
}

static int bmm150_power_onoff(uint8_t on)
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_err_t ret = RT_EOK;
#if (BMM150_POW_PIN >= 0)
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        // rt_kprintf("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
        return ret;

    m.pin = BMM150_POW_PIN;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = BMM150_POW_PIN;
    st.status = on;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
#endif
    return ret;
}

static int bmm150_i2c_init()
{
#if (I2C_SOFT_ENABLE == 1)
    BSP_SOFT_I2C_Init();

#else
    /* get i2c bus device */
    mag_dev_info.bus_handle = (struct rt_i2c_bus_device *)rt_device_find(BMM150_I2C_BUS);
    if (RT_Device_Class_I2CBUS != mag_dev_info.bus_handle->parent.type)
    {
        mag_dev_info.bus_handle = NULL;
    }
    if (mag_dev_info.bus_handle)
    {
        LOG_D("Find i2c bus device %s\n", BMM150_I2C_BUS);
        rt_device_open((rt_device_t)mag_dev_info.bus_handle, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", BMM150_I2C_BUS);
        return -1;
    }

    {
        struct rt_i2c_configuration configuration =
            {
                .mode = 0,
                .addr = dev_addr,
                .timeout = 500,
                .max_hz = 400000,
            };

        rt_i2c_configure(mag_dev_info.bus_handle, &configuration);
    }
#endif

    mag_dev_info.dev_addr = dev_addr;

    return 0;
}

static void bmm150_i2c_deinit(void)
{
    if (mag_dev_info.bus_handle != NULL)
        rt_device_close((rt_device_t)mag_dev_info.bus_handle);
}

int bmm150_initialized(void)
{
    memset(&mag_dev_info, 0x00, sizeof(struct dev_info));

    bmm150_power_onoff(1);
    rt_thread_mdelay(10);
    bmm1_interface_init(&bmm1_dev);

    /* Assign device address and bus instance to interface pointer */
    int res;
    res = bmm150_i2c_init();
    if (res != 0)
    {
        LOG_E("bmm150_i2c_init fail\n");
        bmm150_i2c_deinit();
        return res;
    }

    bmm1_dev.intf_ptr = ((void *)&mag_dev_info);

    int8_t rslt = bmm150_init(&bmm1_dev);
    if (rslt != BMM150_OK)
    {
        bmm150_error_codes_print_result(rslt);
        return rslt;
    }
    rslt = configure_sensor(&bmm1_dev);
    if (rslt != BMM150_OK)
    {
        bmm150_error_codes_print_result(rslt);
        return rslt;
    }

    LOG_D("bmm150 init done\n");
    return rslt;
}

int bmm150_open(void)
{
    int res;
    uint8_t rst;

    if (mag_dev_info.open_flag == 1) // opened before
        return 0;

    mag_dev_info.open_flag = 1;
    bmm150_power_onoff(1);
    return 0;
}

int bmm150_close(void)
{
    int res;

    if (mag_dev_info.open_flag == 0) // closed before
        return 0;
    // Set the power mode of the accelerometer part of the bmm150 sensor to normal
    bmm150_power_onoff(0);
    mag_dev_info.open_flag = 0;

    return 0;
}

// function for handle and address --------------------
uint32_t bmm150_get_bus_handle(void)
{
    return (uint32_t)(mag_dev_info.bus_handle);
}
uint8_t bmm150_get_dev_addr(void)
{
    return mag_dev_info.dev_addr;
}
uint8_t bmm150_get_dev_id(void)
{
    return bmm1_dev.chip_id;
}

int bmm150_self_check(void)
{
    int ret = 0;
    uint8_t whoami;
    ret = bmm150_get_regs(BMM150_REG_CHIP_ID, &whoami, 1, &bmm1_dev);
    if (whoami != bmm150_get_dev_id())
    {
        return -1;
    }
    return 0;
}

// Magnetometer
int bmm150_mag_read(int16_t *x, int16_t *y, int16_t *z)
{
    struct bmm150_mag_data mag_data;
    int const rc = bmm150_read_mag_data(&mag_data, &bmm1_dev);
    *x = mag_data.x;
    *y = mag_data.y;
    *z = mag_data.z;
    // rt_kprintf("mag: %d, %d, %d\n", *x, *y, *z);

    if (rc == BMM150_OK)
        return RT_EOK;
    else
        return -RT_ERROR;
}

void bmm150_set_range(uint8_t range)
{
    /* Status of api are returned to this variable. */
    // int8_t rslt;
    // TODO: set range
}

#if defined(RT_USING_FINSH) && !defined(LCPU_MEM_OPTIMIZE)
#define DRV_BMM150_TEST
#endif
#ifdef DRV_BMM150_TEST

int bmm150_test(int argc, char *argv[])
{
    if (argc < 2)
    {
        LOG_I("Invalid parameter\n");
        return 0;
    }

    if (strcmp(argv[1], "-open") == 0)
    {
        if (bmm150_initialized() == 0)
        {
            bmm150_open();
            LOG_I("bmm150 open success\n");
        }
        else
        {
            LOG_E("bmm150 open fail\n");
        }
    }
    else if (strcmp(argv[1], "-close") == 0)
    {
        bmm150_close();
    }
    else if (strcmp(argv[1], "-read") == 0)
    {
        int16_t x, y, z;
        if (bmm150_mag_read(&x, &y, &z) == BMM150_OK)
        {
            LOG_I("magX = %d, magY = %d, magZ = %d\n", x, y, z);
        }
        else
        {
            LOG_I("get mag fail\n");
        }
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(bmm150_test, __cmd_bmm150, Test hw bmm150);
#endif // DRV_bmm150_TEST

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmm150_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
    case BMM150_E_NULL_PTR:
        LOG_E(
            "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer,"
            " which has been initialized to NULL.\r\n",
            rslt);
        break;

    case BMM150_E_DEV_NOT_FOUND:
        LOG_E("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
              rslt);
        break;

    case BMM150_E_INVALID_CONFIG:
        LOG_E(
            "Error [%d] : Invalid configuration error. It occurs when there is a mismatch in the requested feature with the "
            "available one\r\n",
            rslt);
        break;

    case BMM150_E_COM_FAIL:
        LOG_E(
            "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due "
            "to power failure during communication\r\n",
            rslt);
        break;

    case BMM150_W_NORMAL_SELF_TEST_YZ_FAIL:
        LOG_E("Warning [%d] : Normal self-test YZ axis fail\r\n", rslt);
        break;

    case BMM150_W_NORMAL_SELF_TEST_XZ_FAIL:
        LOG_E("Warning [%d] : Normal self-test XZ axis fail\r\n", rslt);
        break;

    case BMM150_W_NORMAL_SELF_TEST_Z_FAIL:
        LOG_E("Warning [%d] : Normal self-test Z axis fail\r\n", rslt);
        break;

    case BMM150_W_NORMAL_SELF_TEST_XY_FAIL:
        LOG_E("Warning [%d] : Normal self-test XY axis fail\r\n", rslt);
        break;

    case BMM150_W_NORMAL_SELF_TEST_Y_FAIL:
        LOG_E("Warning [%d] : Normal self-test Y axis fail\r\n", rslt);
        break;

    case BMM150_W_NORMAL_SELF_TEST_X_FAIL:
        LOG_E("Warning [%d] : Normal self-test X axis fail\r\n", rslt);
        break;

    case BMM150_W_NORMAL_SELF_TEST_XYZ_FAIL:
        LOG_E("Warning [%d] : Normal self-test XYZ axis fail\r\n", rslt);
        break;

    case BMM150_W_ADV_SELF_TEST_FAIL:
        LOG_E("Warning [%d] : Advanced self-test fail\r\n", rslt);
        break;

    default:
        LOG_E("Error [%d] : Unknown error code\r\n", rslt);
        break;
    }
}
#endif /*MAG_USING_BMM150*/
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
