/**
 ******************************************************************************
 * @file   bmi270_driver.c
 * @author Sifli software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "bmi270_driver.h"
#include "board.h"
#if (I2C_SOFT_ENABLE == 1)
    #include "i2c_soft_driver.h"
#endif
#ifdef BSP_USING_BLOC_PERIPHERAL
    #include "bloc_peripheral.h"
#endif
#ifdef BSP_USING_HAND_TRACKING
    #include "hand_tracking.h"
#endif
#ifdef BSP_USING_GESTURE_DETECT
    #include "gesture_detect.h"
#endif
#include "bsp_board.h"

#define DBG_TAG "drv.bmi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
/******************************************************************************/
/*!                Macro definition                                           */

#ifdef ACC_USING_BMI270
    #define SPI_DIR_READ 0x80
    #define SPI_DIR_WRITE 0x00

    #define BMI270_DEV_NAME "bmi_dev"
    #define USER_CTL_CS

    #define BMI270_USE_INT
    #define BMI270_USE_INT1 1
    #define BMI270_USE_INT2 0

    /* There are 2 interrupt for BMI270 */
    /*! Macros to select the sensors                   */
    #define ACCEL UINT8_C(0x00)
    #define GYRO UINT8_C(0x01)
    /*! Macro that defines read write length */
    #define READ_WRITE_LEN UINT8_C(46)

    #define THREAD_STACK_SIZE 1 * 1024 + 256
    #define THREAD_PRIORITY 7
    #define THREAD_TIMESLICE RT_THREAD_TICK_DEFAULT

/*!
 * @brief  Structure to store the interface related configurations
 */
struct coines_intf_config
{
    uint8_t open_flag; /* Flag to indicate interface is open or not */
    uint8_t
        dev_addr; /* Device address or Chip select of the interface selected */
    uint8_t power_mode;
    /* Bus instance of the interface selected */
    #if (BMI270_USING_I2C == 1)
    struct rt_i2c_bus_device *bus_handle;
    #else  // SPI
    struct rt_spi_device *bus_handle;
    #endif // BMI270_USING_I2C
};

/* Assign accel and gyro sensor to variable. */
static uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};

/* Sensor initialization configuration. */
static struct bmi2_dev bmi2_dev;
static uint16_t _int_status;
/* Set by bmi270_hw_wrist_wake_enable(); checked in the int handler to
   decide whether to dispatch the wrist-wake feature interrupt. */
static bool hw_wrist_wake_armed = false;
/* Set by bmi270_any_motion_enable(); checked in the int handler to dispatch
   the any_motion feature interrupt as an additional fast wake trigger. */
static bool any_motion_armed = false;
/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/*! Variable that holds the I2C or SPI bus instance */
struct coines_intf_config accel_gyro_dev_info;

struct bmi2_sens_axes_data local_watch_accel;
struct bmi2_sens_axes_data local_watch_gyro;

static rt_mutex_t api_lock = RT_NULL;

/* ===== HW wrist-wake tuning ===========================================
   Override Bosch defaults to better match this watch's SW algorithm
   behavior. All values are chip-scaled (2048 × trig(angle)).
     - min_angle_*    : larger = more sensitive (smaller angle change triggers)
     - max_tilt_*     : larger = more permissive (wider "looking" envelope)
   Bosch defaults shown in trailing comments for reference. */

/* Required attitude change to enter "looking at watch" state.
   1856 = 2048 × cos(25°) — most sensitive allowed by spec, matches SW's
   "should trigger on any deliberate raise" intent.
   Range 1448-1856. */
#define BMI270_WRIST_WAKE_MIN_ANGLE_NONFOCUS    1856  /* default 1774 ≈ 30° */

/* Threshold for dropping OUT of "looking at watch" state.
   Bosch wording: "minimum expected attitude change within 1 sec while in
   focus position". Encoded as 2048 × cos(angle).
     - Higher value (smaller threshold angle, e.g. 1774 ≈ 33°) → drops out
       on smaller wrist movement → ENABLES quick re-fire when user lowers
       and raises wrist again.
     - Lower value (larger threshold angle, e.g. 1024 ≈ 60°) → stays in
       focus state longer → fewer re-fires from wrist twists / fidgeting.
   Note: this does NOT affect the *initial* trigger sensitivity — that is
   controlled by min_angle_nonfocus.
   Range 1024-1774. Set to upper end for fast re-fires. */
#define BMI270_WRIST_WAKE_MIN_ANGLE_FOCUS       1700  /* default 1448 ≈ 45° */

/* Max tilt of watch face right/left (landscape) while still "looking".
   1024 = 30° (the spec ceiling); make both directions symmetric since
   users may wear watch on either wrist.
   Range 700-1024. */
#define BMI270_WRIST_WAKE_MAX_TILT_LR           1024  /* default 1024 ≈ 30° */
#define BMI270_WRIST_WAKE_MAX_TILT_LL           1024  /* default 700 ≈ 20° */

/* Max tilt toward / away from body. PD is chip-hard-limited to 5° (179).
   PU pushed to spec ceiling 1978 ≈ 75° to accept more "looking up at
   watch" postures (e.g., sitting at desk, arm bent low).
   Range PD 0-179, PU 1774-1978. */
#define BMI270_WRIST_WAKE_MAX_TILT_PD           179   /* default 179 ≈ 5° */
#define BMI270_WRIST_WAKE_MAX_TILT_PU           1978  /* default 1925 ≈ 70° */

/******************************************************************************/
/*!          Function Declaration                                     */
static void bmi270_api_lock(void)
{
    if (api_lock)
    {
        rt_mutex_take(api_lock, RT_WAITING_FOREVER);
    }
}

static void bmi270_api_unlock(void)
{
    if (api_lock)
    {
        rt_mutex_release(api_lock);
    }
}

void bmi2_error_codes_print_result(int8_t rslt);
    #if (BMI270_USING_I2C == 1)
    // static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t
    // *reg_data, uint32_t len, void *intf_ptr)
    // {
    //     rt_size_t res;
    //     struct coines_intf_config handle = *(struct coines_intf_config
    //     *)intf_ptr; if (intf_ptr && handle.bus_handle && reg_data)
    //     {
    //         uint16_t addr16 = (uint16_t)reg_addr;
    //         res = rt_i2c_mem_read(handle.bus_handle, handle.dev_addr, addr16,
    //         8, (void *)reg_data, len);
    //         // LOG_D("i2c read res = %d\n", res);
    //         if (res > 0)
    //             return BMI2_INTF_RET_SUCCESS;
    //         else
    //             return -2;
    //     }
    //     else
    //     {
    //         return -3;
    //     }
    // }

        #if (I2C_SOFT_ENABLE == 1)
static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                           uint32_t len, void *intf_ptr)
{
    uint8_t ret = BMI2_INTF_RET_SUCCESS;

    struct coines_intf_config handle = *(struct coines_intf_config *)intf_ptr;
    // write slave address
    BSP_I2C_WriteSlaveAddress(handle.dev_addr, 0);
    BSP_I2C_Write(reg_addr, reg_addr);
    BSP_I2C_Stop();
    if (len > 0)
    {
        BSP_I2C_WriteSlaveAddress(handle.dev_addr, 1);
        for (uint32_t i = 0; i < len; i++)
        {
            reg_data[i] = BSP_I2C_Read(reg_addr);
        }
        BSP_I2C_Stop();
    }
    return ret;
}

static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr,
                                            const uint8_t *reg_data,
                                            uint32_t len, void *intf_ptr)
{
    uint8_t ret = BMI2_INTF_RET_SUCCESS;

    struct coines_intf_config handle = *(struct coines_intf_config *)intf_ptr;
    // write slave address
    BSP_I2C_WriteSlaveAddress(handle.dev_addr, 0);
    BSP_I2C_Write(reg_addr, reg_addr);
    if (len > 0)
    {
        // BSP_I2C_WriteSlaveAddress(handle.dev_addr, 0);
        for (uint32_t i = 0; i < len; i++)
        {
            BSP_I2C_Write(reg_addr, reg_data[i]);
        }
        BSP_I2C_Stop();
    }
    else
    {
        BSP_I2C_Stop();
    }
    return ret;
}
        #else
static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                           uint32_t len, void *intf_ptr)
{
    uint8_t ret = BMI2_INTF_RET_SUCCESS;
    struct rt_i2c_msg msgs[2];
    uint32_t res;

    struct coines_intf_config handle = *(struct coines_intf_config *)intf_ptr;
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
            ret = BMI2_INTF_RET_SUCCESS;
        }
        else
        {
            LOG_E("bmi2_i2c_read FAIL: %d, dev_addr=0x%x, reg=%06x, len=%d\n",
                  ret, handle.dev_addr, reg_addr, len);
            ret = -2;
        }
    }
    else
    {
        ret = -3;
    }

    return ret;
}

static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr,
                                            const uint8_t *reg_data,
                                            uint32_t len, void *intf_ptr)
{
    rt_size_t res;
    struct coines_intf_config handle = *(struct coines_intf_config *)intf_ptr;
    if (intf_ptr && handle.bus_handle)
    {
        uint16_t addr16 = (uint16_t)reg_addr;
        res = rt_i2c_mem_write(handle.bus_handle, handle.dev_addr, addr16, 8,
                               (void *)reg_data, len);
        // LOG_D("i2c write res = %d\n", res);
        if (res > 0)
        {
            return BMI2_INTF_RET_SUCCESS;
        }
        else
        {
            LOG_E("bmi2_i2c_write FAIL: %d, dev_addr=0x%x, reg=%06x, len=%d\n",
                  res, handle.dev_addr, reg_addr, len);
            return -2;
        }
    }
    else
    {
        return -3;
    }
}
        #endif

    #else // SPI
static BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data,
                                           uint32_t len, void *intf_ptr)
{
    struct coines_intf_config handle = *(struct coines_intf_config *)intf_ptr;
    if (intf_ptr && handle.bus_handle)
    {
        #ifndef USER_CTL_CS

        uint16_t addr16 = (uint16_t)reg_addr;
        uint16_t cnt = len;
        uint16_t buf;
        uint16_t value;

        LOG_E("Read addr 0x%x , len %d\n", reg_addr, len);
        while (cnt > 0)
        {
            buf = (addr16 | 0x80) << 8;

            rt_size_t res = rt_spi_transfer(handle.bus_handle, &buf, &value, 1);
            LOG_E("buf value res 0x%x 0x%x 0x%x\n", buf, value,
                  res); // for test
            if (res == 0)
            {
                LOG_E("SPI transmit fail \n");
                return -1;
            }
            *reg_data = (uint16_t)(value & 0xff);
            cnt--;
            addr16++;
            reg_data++;
        }
        return BMI2_INTF_RET_SUCCESS;
        #else
        rt_err_t ret = RT_EOK;
        rt_uint8_t send_buf = reg_addr | SPI_DIR_READ;

        // LOG_I("Read addr 0x%x , len %d\n", reg_addr, len);

        /* Method 1: Send the command to read the ID using
         * rt_spi_send_then_recv() */
        ret = rt_spi_send_then_recv(handle.bus_handle, &send_buf, 1, reg_data,
                                    len);
        /* Method 2 */
        // struct rt_spi_message msg1, msg2;
        // msg1.send_buf = &send_buf;
        // msg1.recv_buf = RT_NULL;
        // msg1.length = 1;
        // msg1.cs_take = 1;
        // msg1.cs_release = 0;
        // msg1.next = &msg2;
        // msg2.send_buf = RT_NULL;
        // msg2.recv_buf = reg_data;
        // msg2.length = len;
        // msg2.cs_take = 0;
        // msg2.cs_release = 1;
        // msg2.next = RT_NULL;
        // rt_spi_transfer_message(handle.bus_handle, &msg1);
        if (ret == RT_EOK)
        {
            // LOG_I("SPI read data 0x%x\n", *reg_data);
            return BMI2_INTF_RET_SUCCESS;
        }
        else
        {
            LOG_E("SPI read fail %d\n", ret);
            return -2;
        }
        #endif
    }
    else
    {
        return -1;
    }
}
static BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr,
                                            const uint8_t *reg_data,
                                            uint32_t len, void *intf_ptr)
{
    struct coines_intf_config handle = *(struct coines_intf_config *)intf_ptr;
    if (intf_ptr && handle.bus_handle && reg_data)
    {
        #ifndef USER_CTL_CS

        uint16_t addr16 = (uint16_t)reg_addr;
        uint16_t buf;
        uint16_t cnt = len;

        LOG_I("Write addr 0x%x , len %d\n", reg_addr, len);
        while (cnt > 0)
        {
            buf = (addr16 << 8) | *reg_data;
            rt_size_t res = rt_spi_transfer(handle.bus_handle, &buf, NULL, 1);
            if (res == 0)
            {
                LOG_I("SPI transmit fail \n");
                return -1;
            }
            cnt--;
            reg_data++;
            addr16++;
        }
        return BMI2_INTF_RET_SUCCESS;
        #else
        uint16_t addr = reg_addr;
        rt_err_t ret =
            rt_spi_send_then_send(handle.bus_handle, &addr, 1, reg_data, len);
        if (ret == RT_EOK)
        {
            // LOG_D("SPI write data 0x%x\n", *reg_data);
            return BMI2_INTF_RET_SUCCESS;
        }
        else
        {
            LOG_E("SPI write fail %d\n", ret);
            return -2;
        }
        #endif
    }
    else
    {
        LOG_E("SPI write fail\n");
        return -1;
    }
}
    #endif // BMI270_USING_I2C

static void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    HAL_Delay_us(period);
}

static void bmi2_interface_init(struct bmi2_dev *bmi, uint8_t intf)
{
    #if (BMI270_USING_I2C == 1)
    /* Bus configuration : I2C */
    if (intf == BMI2_I2C_INTF)
    {
        LOG_D("I2C Interface");
        /* To initialize the user I2C function */
        bmi->intf = BMI2_I2C_INTF;
        bmi->read = bmi2_i2c_read;
        bmi->write = bmi2_i2c_write;
        dev_addr = BMI2_I2C_PRIM_ADDR;
        accel_gyro_dev_info.dev_addr = dev_addr;
    }
    #else
    /* Bus configuration : SPI */
    if (intf == BMI2_SPI_INTF)
    {
        LOG_D("SPI Interface");
        /* To initialize the user SPI function */
        bmi->intf = BMI2_SPI_INTF;
        bmi->read = bmi2_spi_read;
        bmi->write = bmi2_spi_write;
    }
    #endif

    bmi->delay_us = bmi2_delay_us;
    bmi->read_write_len = READ_WRITE_LEN; // Limitation of the Wire library
    bmi->config_file_ptr = NULL;          // Use the default BMI270 config file
}

static int8_t configure_sensor_performance_mode(struct bmi2_dev *dev)
{
    if (accel_gyro_dev_info.power_mode == BMI2_PERF_OPT_MODE)
    {
        return BMI2_OK;
    }
    int8_t rslt = BMI2_OK;
    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config sens_cfg[2];

    /* Configure the type of feature. */
    sens_cfg[ACCEL].type = BMI2_ACCEL;
    sens_cfg[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(sens_cfg, 2, dev);

    if (rslt != BMI2_OK)
        return rslt;

    /* Set Output Data Rate */
    sens_cfg[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_100HZ;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
    sens_cfg[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_4G;

    /* The bandwidth parameter is used to configure the number of sensor samples
     * that are averaged if it is set to 2, then 2^(bandwidth parameter) samples
     * are averaged, resulting in 4 averaged samples.
     * Note1 : For more information, refer the datasheet.
     * Note2 : A higher number of averaged samples will result in a lower noise
     * level of the signal, but this has an adverse effect on the power
     * consumed.
     */
    sens_cfg[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Ultra low power mode
     *  1 -> High performance mode(Default)
     * For more info refer datasheet.
     */
    sens_cfg[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    /* The user can change the following configuration parameters according to
     * their requirement. */
    /* Set Output Data Rate */
    sens_cfg[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;

    /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps.
     */
    sens_cfg[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

    /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in
     * normal mode. */
    sens_cfg[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;

    /* Enable/Disable the noise performance mode for precision yaw rate sensing
     * There are two modes
     *  0 -> Ultra low power mode(Default)
     *  1 -> High performance mode
     */
    sens_cfg[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

    /* Enable/Disable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Ultra low power mode
     *  1 -> High performance mode(Default)
     */
    sens_cfg[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    sens_cfg[GYRO].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    /* Set the accel and gyro configurations. */
    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK)
        return rslt;

    rslt = bmi2_sensor_enable(sensor_list, 2, dev);
    if (rslt != BMI2_OK)
        return rslt;

    accel_gyro_dev_info.power_mode = BMI2_PERF_OPT_MODE;

    return rslt;
}

static int8_t configure_sensor_power_mode(struct bmi2_dev *dev)
{
    if (accel_gyro_dev_info.power_mode == BMI2_POWER_OPT_MODE)
    {
        return BMI2_OK;
    }

    int8_t rslt = BMI2_OK;
    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config sens_cfg[2];

    /* Configure the type of feature. */
    sens_cfg[ACCEL].type = BMI2_ACCEL;
    sens_cfg[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(sens_cfg, 2, dev);

    if (rslt != BMI2_OK)
        return rslt;

    /* Low-power screen-off config: accel at 25 Hz, gyro at 25 Hz, both in
       power-opt filter mode. BMI270 wrist-wake and step counter features
       are documented (datasheet 4.8.6 / 4.8.8) to work without specific
       ODR requirements at this level — let chip's feature engine handle
       its own sampling internally. */
    sens_cfg[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_25HZ;

    sens_cfg[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_4G;

    sens_cfg[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;

    sens_cfg[ACCEL].cfg.acc.filter_perf = BMI2_POWER_OPT_MODE;

    sens_cfg[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;

    sens_cfg[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

    sens_cfg[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;

    sens_cfg[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

    sens_cfg[GYRO].cfg.gyr.filter_perf = BMI2_POWER_OPT_MODE;

    sens_cfg[GYRO].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    /* Set the accel and gyro configurations. */
    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK)
        return rslt;

    rslt = bmi2_sensor_enable(sensor_list, 2, dev);
    if (rslt != BMI2_OK)
        return rslt;

    accel_gyro_dev_info.power_mode = BMI2_POWER_OPT_MODE;

    return rslt;
}

    #ifdef BMI270_USE_INT
static int8_t configure_sensor_interrupt(struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_OK;
    /* Structure to define the type of interrupt pin and its level. */
    struct bmi2_int_pin_config int_pin_cfg;

    /* Get default configuration for hardware Interrupt */
    rslt = bmi2_get_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK)
    {
        return rslt;
    }

        #if BMI270_USE_INT1
    int_pin_cfg.pin_type = BMI2_INT1;
        #else
    int_pin_cfg.pin_type = BMI2_INT2;
        #endif
    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, int_pin_cfg.pin_type, dev);

    if (rslt != BMI2_OK)
    {
        return rslt;
    }

    /* Interrupt pin configuration */
    int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
    int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    /* Set Hardware interrupt pin configuration */
    rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK)
    {
        return rslt;
    }
    return rslt;
}
    #endif

    #ifdef BMI270_USE_INT
static struct rt_semaphore bmi_int_sem;
    #endif
static rt_thread_t bmi270_thread = NULL;
extern rt_err_t rt_hw_spi_device_attach(const char *bus_name,
                                        const char *device_name);

static int bmi270_power_onoff(uint8_t on)
{
    rt_err_t ret = RT_EOK;
#if BMI270_POW_PIN
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_D("GPIO pin device not found at motor ctrl");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
        return ret;

    m.pin = BMI270_POW_PIN;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = BMI270_POW_PIN;
    st.status = on;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
    LOG_D("BMI270 power %s", on ? "on" : "off");
#endif
    return ret;
}

static int bmi270_i2c_init()
{
    #if (BMI270_USING_I2C == 1)
    rt_err_t ret;
        #if (I2C_SOFT_ENABLE == 1)
    BSP_SOFT_I2C_Init();
        #else
    /* get i2c bus device */
    // accel_gyro_dev_info.bus_handle = rt_i2c_bus_device_find(BMI270_BUS_NAME);
    accel_gyro_dev_info.bus_handle =
        (struct rt_i2c_bus_device *)rt_device_find(BMI270_BUS_NAME);
    if (RT_Device_Class_I2CBUS != accel_gyro_dev_info.bus_handle->parent.type)
    {
        accel_gyro_dev_info.bus_handle = NULL;
    }
    if (accel_gyro_dev_info.bus_handle == NULL)
    {
        LOG_E("Can not found i2c bus %s, init fail\n", BMI270_BUS_NAME);
        return -1;
    }
    LOG_D("Find i2c bus device %s\n", BMI270_BUS_NAME);
    ret = rt_device_open((rt_device_t)accel_gyro_dev_info.bus_handle,
                         RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK)
    {
        return ret;
    }
    LOG_D("Open i2c bus device %s\n", BMI270_BUS_NAME);

    /* Configure the i2c bus */
    {
        struct rt_i2c_configuration configuration = {
            .mode = 0,
            .addr = accel_gyro_dev_info.dev_addr,
            .timeout = 500,
            .max_hz = 400000,
        };

        ret = rt_i2c_configure(accel_gyro_dev_info.bus_handle, &configuration);
        if (ret != RT_EOK)
        {
            LOG_E("Can not configure i2c bus %s, init fail\n", BMI270_BUS_NAME);
            return ret;
        }
    }
        #endif

    #else // SPI
    /* get spi bus device */
    rt_err_t ret;
    rt_device_t spi_bus = rt_device_find(BMI270_BUS_NAME);
    // accel_gyro_dev_info.bus_handle = (struct rt_spi_device
    // *)rt_malloc(sizeof(struct rt_spi_device));

    if (spi_bus)
    {
        struct rt_spi_configuration cfg1;
        LOG_D("Find spi bus %s\n", BMI270_BUS_NAME);

        accel_gyro_dev_info.bus_handle =
            (struct rt_spi_device *)rt_device_find(BMI270_DEV_NAME);

        if (accel_gyro_dev_info.bus_handle == NULL)
        {
            ret = rt_hw_spi_device_attach(BMI270_BUS_NAME, BMI270_DEV_NAME);
            if (ret != RT_EOK)
            {
                return -1;
            }
            accel_gyro_dev_info.bus_handle =
                (struct rt_spi_device *)rt_device_find(BMI270_DEV_NAME);

            if (accel_gyro_dev_info.bus_handle == NULL)
            {
                return -1;
            }
        }
        ret = rt_device_open((rt_device_t)accel_gyro_dev_info.bus_handle,
                             RT_DEVICE_FLAG_RDWR);
        if (ret != RT_EOK)
        {
            return ret;
        }
            // rt_spi_bus_attach_device(accel_gyro_dev_info.bus_handle,
            // "spi_bmi", BMI270_BUS_NAME, NULL);

            // rt_spi_take_bus(accel_gyro_dev_info.bus_handle);
            // rt_spi_take(accel_gyro_dev_info.bus_handle);
            // rt_spi_release_bus(accel_gyro_dev_info.bus_handle);
        #ifndef USER_CTL_CS
        cfg1.data_width = 16; // 8; //16;    // auto cs, need total 16 bits to
                              // read data in low 8 bits
        #else
        cfg1.data_width = 8; // 8; //16;
        #endif
        cfg1.max_hz = 4 * 1000 * 1000; // 12hz 6m
        // cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB | RT_SPI_SLAVE;
        cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB | RT_SPI_MASTER;
        cfg1.frameMode = RT_SPI_MOTO; // RT_SPI_TI

        ret = rt_spi_configure(accel_gyro_dev_info.bus_handle, &cfg1);
        if (ret != RT_EOK)
        {
            LOG_E("Can not configure spi bus %s, init fail\n", BMI270_BUS_NAME);
            return ret;
        }
        // ret = rt_spi_take_bus(accel_gyro_dev_info.bus_handle);
        // if (ret != RT_EOK)
        // {
        //     LOG_E("Can not take spi bus %s, init fail\n", BMI270_BUS_NAME);
        //     return ret;
        // }
        // ret = rt_spi_release_bus(accel_gyro_dev_info.bus_handle);
        // if (ret != RT_EOK)
        // {
        //     LOG_E("Can not release spi bus %s, init fail\n",
        //     BMI270_BUS_NAME); return ret;
        // }
    }
    else
    {
        LOG_E("Can not found spi bus %s, init fail\n", BMI270_BUS_NAME);
        return -1;
    }

    #endif // BMI270_USING_I2C
    return 0;
}

static void bmi270_i2c_deinit(void)
{
    if (accel_gyro_dev_info.bus_handle != NULL)
        rt_device_close((rt_device_t)accel_gyro_dev_info.bus_handle);
}

struct bmi2_sens_axes_data *bmi270_get_accel(void)
{
    return &local_watch_accel;
}

struct bmi2_sens_axes_data *bmi270_get_gyro(void)
{
    return &local_watch_gyro;
}

static struct bmi2_sens_axes_data
redirect_sensor_data(struct bmi2_sens_axes_data *data);

/* Read ONE fresh accel sample on demand, returning it in the watch frame
   (same axis convention as imu_data_fetch / the AHRS). Unlike
   bmi270_accel_read() this does NOT gate on the DRDY int-status bit, so it
   is safe to call from the wrist-wake handler where that bit has already
   been consumed by bmi270_int_msg_handler(). Scale is left in raw int16
   counts (callers normalise). Returns 0 on success, -1 on failure. */
int bmi270_read_accel_now(int16_t *px, int16_t *py, int16_t *pz)
{
    if (accel_gyro_dev_info.open_flag == 0 || !px || !py || !pz)
        return -1;

    struct bmi2_sens_data sensor_data = {{0}};
    bmi270_api_lock();
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
    bmi270_api_unlock();
    if (rslt != BMI2_OK)
        return -1;

    struct bmi2_sens_axes_data acc = redirect_sensor_data(&sensor_data.acc);
    *px = acc.x;
    *py = acc.y;
    *pz = acc.z;
    return 0;
}

/* 晶片全域 remap 目前是否為 NEG/NEG/NEG(睡眠中給抬腕 feature engine 用)。跟隨「實際寫入
   成功的晶片狀態」,由 wrist-wake enable/disable 維護。redirect_sensor_data 據此互斥:晶片
   已翻→軟體直通、晶片 identity→軟體翻——總框架(watch frame)任何時刻恆定。2026-07-17:
   方向反根因修成軟體翻+晶片恆 identity 後,抬腕 HW 演算法(吃晶片軸)軸向錯了(founder
   回報);故睡眠中臨時把晶片 remap 寫回 NEG 餵 feature engine、醒來還原,讀值靠本 flag 聯動
   保持一致。 */
static volatile bool s_chip_remap_neg = false;

static struct bmi2_sens_axes_data
redirect_sensor_data(struct bmi2_sens_axes_data *data)
{
    struct bmi2_sens_axes_data dataRedirect;
    dataRedirect.virt_sens_time = data->virt_sens_time;
    if (s_chip_remap_neg)
    {
        /* 晶片已三軸取負(舊框架)→ 軟體做「舊翻法」補回同一 watch frame(z 不再翻)。 */
        #if (WATCH_IMU_REVERSE_180)
        dataRedirect.x = -data->x;
        dataRedirect.y = data->y;
        #else
        dataRedirect.x = data->x;
        dataRedirect.y = -data->y;
        #endif
        dataRedirect.z = data->z;
    }
    else
    {
        /* 晶片 identity(開機/清醒預設)→ 軟體全責翻轉(founder 2026-07-17 方向反根因修法)。 */
        #if (WATCH_IMU_REVERSE_180)
        dataRedirect.x = data->x;
        dataRedirect.y = -data->y;
        #else
        dataRedirect.x = -data->x;
        dataRedirect.y = data->y;
        #endif
        dataRedirect.z = -data->z;
    }
    return dataRedirect;
}

    #define ENABLE_ABNORMAL_CHECK 0

static int imu_data_fetch(imu_sensor_data_t *data)
{
    data->timestamp = rt_sensor_get_ts();
    struct bmi2_sens_axes_data *acce = bmi270_get_accel();
    data->acce.x = (float)acce->x / INT16_to_G * GRAVITY;
    data->acce.y = (float)acce->y / INT16_to_G * GRAVITY;
    data->acce.z = (float)acce->z / INT16_to_G * GRAVITY;

    #if ENABLE_ABNORMAL_CHECK
    uint8_t abnormal_count = 0;
    if (fabs(data->acce.x) < 0.002) // 0.001g
        abnormal_count++;
    if (fabs(data->acce.y) < 0.002)
        abnormal_count++;
    if (fabs(data->acce.z) < 0.002)
        abnormal_count++;
    if (abnormal_count >= 2)
    {
        watch_sensor.imu_abnormal = true;
        LOG_W("Abnormal acce data: %.3f, %.3f, %.3f", data->acce.x,
              data->acce.y, data->acce.z);
        return -1;
    }
    else
    {
        watch_sensor.imu_abnormal = false;
    }
    #endif
    struct bmi2_sens_axes_data *gyro = bmi270_get_gyro();
    data->gyro.x = (float)gyro->x / INT16_to_DPS;
    data->gyro.y = (float)gyro->y / INT16_to_DPS;
    data->gyro.z = (float)gyro->z / INT16_to_DPS;

    return RT_EOK;
}

bool power_opt_mode(void)
{
    return accel_gyro_dev_info.power_mode == BMI2_POWER_OPT_MODE;
}


/* ---- accelerometer for the screen-off HR burst -----------------------------
 *
 * Standby shuts the accelerometer data path down entirely (DRDY un-routed, FIFO
 * disarmed) to save power, and HR bursts run in standby. The consequence went
 * unnoticed for three months: BOTH the in-tree motion compensation and the
 * vendor's own read gsensor_fifo_buffer, and nothing writes it in that mode, so
 * neither has ever seen wrist movement while the screen was off.
 *
 * This re-routes DRDY for the duration of a burst only. Cost is bounded by the
 * burst: 25 Hz wakeups for ~40 s out of every 3-10 minutes. Callers MUST pair
 * enable with disable — leaving it on holds the LCPU awake at 25 Hz forever.
 */
static volatile int s_hr_accel_stream = 0;

int bmi270_hr_accel_stream_active(void)
{
    return s_hr_accel_stream;
}

int bmi270_set_hr_accel_stream(int en)
{
    if (s_hr_accel_stream == (en ? 1 : 0)) return 0;
    s_hr_accel_stream = en ? 1 : 0;
    /* Only touch the routing while asleep. Awake, DRDY is already routed for
       the normal path and un-routing it here would kill gesture detection.
       Note this is also the ONLY place the routing is dropped for a sleeping
       watch once a claim exists: on_lcpu_sleep_mode_changed skips its own
       un-route while the claim is held, so releasing the claim here is what
       finally puts DRDY back down. */
    if (is_sleep_mode())
        return bmi270_set_drdy_int_routing(en ? 1 : 0);
    return 0;
}

static void bmi270_acc_gyro_evt_handler()
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sensor_data = {{0}};
    /* Read the sensor data of accel and gyro. */
    rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
    if (rslt == BMI2_OK)
    {
        local_watch_accel = redirect_sensor_data(&sensor_data.acc);
        local_watch_gyro = redirect_sensor_data(&sensor_data.gyr);
        /* Screen-off HR burst: fill the health accel ring and stop there.
           The gyro is suspended in standby, so the normal path would drive the
           Mahony fusion with a dead gyro and wreck the orientation wrist-wake
           relies on. It would also run gesture detection at a time the power
           design deliberately keeps it off. @ref bmi270_set_hr_accel_stream */
        if (s_hr_accel_stream && is_sleep_mode())
        {
            if (imu_data_fetch(&watch_sensor.imu_data) == RT_EOK)
            {
    #ifdef BSP_USING_GESTURE_DETECT
                feed_health_accel_only(&watch_sensor.imu_data.acce);
    #endif
    #ifdef BSP_USING_WEAR_DETECT
                /* wear_detect needs motion too, and this early return used to
                 * starve it completely: handle_imu_data() below is the ONLY
                 * caller of wear_detect_feed_imu(), so with the screen off it
                 * saw nothing at all. That matters because motion is the only
                 * thing that opens the PPG probe window, and the probe is the
                 * only way to get back from NOT_WEARING to WEARING -- i.e. put
                 * the watch on while it is asleep and it could never notice
                 * (observed 2026-09-01). The sample is already fetched; this
                 * is one sqrtf and a ring store, and try_evaluate() inside it
                 * self-throttles to EVAL_PERIOD_MS, so it does not reintroduce
                 * the gesture-pipeline cost this early return exists to avoid. */
                extern void wear_detect_feed_imu(Vector3 *acce, float sample_rate);
                wear_detect_feed_imu(&watch_sensor.imu_data.acce,
                                     watch_sensor.imu_data.sample_rate);
    #endif
            }
            return;
        }
        if (imu_data_fetch(&watch_sensor.imu_data) == RT_EOK)
        {
    #if ENABLE_IMU_SEM_FIFO
            rt_sem_release(watch_sensor.imu_sem);
    #else
        #ifdef BSP_USING_GESTURE_DETECT
            int cost_time = handle_imu_data(watch_sensor.imu_data.sample_rate,
                                            &watch_sensor.imu_data.acce,
                                            &watch_sensor.imu_data.gyro);
            LOG_D("Handle imu data cost %d ms\n", cost_time);
        #endif
    #endif
        }
    }
    else
    {
        bmi2_error_codes_print_result(rslt);
    }
}

    #ifdef BMI270_USE_INT

static int imuDataAvailable()
{
    uint16_t status;
    bmi2_get_int_status(&status, &bmi2_dev);
    int ret = ((status | _int_status) &
               (BMI2_ACC_DRDY_INT_MASK | BMI2_GYR_DRDY_INT_MASK));
    _int_status = status;
    _int_status &= ~(BMI2_ACC_DRDY_INT_MASK | BMI2_GYR_DRDY_INT_MASK);
    return ret;
}

static int accelerationAvailable()
{
    uint16_t status;
    bmi2_get_int_status(&status, &bmi2_dev);
    int ret = ((status | _int_status) & BMI2_ACC_DRDY_INT_MASK);
    _int_status = status;
    _int_status &= ~BMI2_ACC_DRDY_INT_MASK;
    return ret;
}

static int gyroscopeAvailable()
{
    uint16_t status;
    bmi2_get_int_status(&status, &bmi2_dev);
    int ret = ((status | _int_status) & BMI2_GYR_DRDY_INT_MASK);
    _int_status = status;
    _int_status &= ~BMI2_GYR_DRDY_INT_MASK;
    return ret;
}

static void bmi270_int_msg_handler(void)
{
    /* Read int_status once. The chip clears bits on read, so we have to
       decode every event from this single value — calling imuDataAvailable()
       afterwards would see zeros and miss the DRDY. */
    uint16_t status = 0;
    bmi2_get_int_status(&status, &bmi2_dev);

    /* Per-interrupt diagnostic: noisy when accel DRDY fires every 10 ms,
       so default to LOG_D and only escalate to LOG_I when something
       interesting happens (wrist-wake armed, wrist-wake bit set, or any
       feature bit in the upper byte). */
    const uint16_t drdy_mask = (BMI2_ACC_DRDY_INT_MASK | BMI2_GYR_DRDY_INT_MASK);
    const bool wrist_bit = (status & BMI270_WRIST_WAKE_UP_STATUS_MASK) != 0;
    const bool feat_bits = (status & ~drdy_mask) != 0;
    if (hw_wrist_wake_armed || wrist_bit || feat_bits)
    {
        LOG_I("BMI270 int fired: status=0x%04x, wrist_armed=%d, wrist_bit=%d",
              status, hw_wrist_wake_armed ? 1 : 0, wrist_bit ? 1 : 0);
    }
    else
    {
        LOG_D("BMI270 int fired: status=0x%04x", status);
    }

    /* Feature interrupts first. When HW wrist-wake is armed, INT1 may fire
       solely for the gesture without any DRDY bit set. */
    if (hw_wrist_wake_armed && wrist_bit)
    {
        bmi270_on_wrist_wake_detected();
    }

    /* Wrist-gesture (flick_in/out, pivot_up etc.) shares INT1 with the
       wear-wakeup feature when both are armed. Read which gesture fired
       and dispatch the wake handler for motion-toward-watch gestures. */
    if (hw_wrist_wake_armed && (status & BMI270_WRIST_GEST_STATUS_MASK))
    {
        struct bmi2_feat_sensor_data fdata = { .type = BMI2_WRIST_GESTURE };
        if (bmi270_get_feature_data(&fdata, 1, &bmi2_dev) == BMI2_OK)
        {
            uint8_t g = fdata.sens_data.wrist_gesture_output;
            static const char *names[6] = {
                "unknown", "push_arm_down", "pivot_up",
                "wrist_jiggle", "flick_in", "flick_out"
            };
            if (g == 2 /*pivot_up*/ || g == 4 /*flick_in*/ || g == 5 /*flick_out*/)
            {
                bmi270_on_wrist_wake_detected();
            }
        }
        else
        {
            LOG_W("BMI270 wrist-gesture fired but get_feature_data failed; dispatching anyway");
            bmi270_on_wrist_wake_detected();
        }
    }

    /* any_motion — additional FAST wake trigger. Fires on ANY motion within
       tens of ms (vs wrist-wake's ~1 s). The host-side handler MUST pose-filter
       before waking the screen, else walking lights it up. */
    if (any_motion_armed && (status & BMI270_ANY_MOT_STATUS_MASK))
    {
        bmi270_on_any_motion_detected();
    }

    /* FIFO watermark — screen-off path. The chip accumulates ~1 s of
       accel+gyro samples; drain everything through the AHRS so the
       quaternion stays converged for the next wrist-wake event. Process
       before DRDY since DRDY shouldn't be routed in this mode anyway. */
    if (status & (BMI2_FWM_INT_STATUS_MASK | BMI2_FFULL_INT_STATUS_MASK))
    {
        int n = bmi270_drain_fifo_to_ahrs();
        LOG_I("BMI270 FIFO drain: %d frames (status=0x%04x)", n, status);
    }

    /* Accel / gyro data-ready — same code path as before. */
    if (status & (BMI2_ACC_DRDY_INT_MASK | BMI2_GYR_DRDY_INT_MASK))
    {
        /* Mirror the old _int_status bookkeeping so any other code paths
           that still consult it see consistent state. */
        _int_status = status;
        _int_status &= ~(BMI2_ACC_DRDY_INT_MASK | BMI2_GYR_DRDY_INT_MASK);
        bmi270_acc_gyro_evt_handler();
    }
    else if (status == 0)
    {
        LOG_W("BMI270 spurious int (status=0)");
    }
}

long rt_bmi270_irq_pin_enable(uint32_t enabled)
{
    rt_uint16_t pin;
        #if BMI270_USE_INT1
            #ifdef BMI270_INT_GPIO_BIT
    pin = BMI270_INT_GPIO_BIT;
            #else
    pin = IMU_INT_PIN;
            #endif
        #endif
        #if BMI270_USE_INT2
    pin = BMI270_INT2_GPIO_BIT;
        #endif
    return rt_pin_irq_enable(pin, enabled);
}
/* BMI270 interrupt init, it use gpio input as int */
// check edge
static void bmi270_int1_handle(void *args)
{
    // LOG_D("bmi270_int1_handle");
    rt_bmi270_irq_pin_enable(0);
    rt_sem_release(&bmi_int_sem);

        #if 0
    //int value = (int)args;
    //LOG_I("bmi270_int_handle %d\n", value);
    bmi270_int1_route_t value = {0};
    bmi270_pin_int1_route_get(&sens_cont, &value);
    if (value.int1_step_detector)
    {
        LOG_I("step detect\n");
    }
    if (value.int1_full_flag)
    {
        LOG_I("fifo full\n");
    }
    if (value.int1_fifo_ovr)
    {
        LOG_I("Fifo overflow\n");
    }
    if (value.int1_fth)
    {
        LOG_I("Fifo threshold\n");
    }
        #endif
}

static void bmi270_int2_handle(void *args)
{
    // LOG_E("bmi270_int2_handle\n");
    rt_bmi270_irq_pin_enable(0);
    rt_sem_release(&bmi_int_sem);
}

static int bmi270_gpio_int_enable(void)
{
    struct rt_device_pin_mode m;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at BMI270\n");
        return -1;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
        #if BMI270_USE_INT1
            // int pin cfg
            #ifdef BMI270_INT_GPIO_BIT
    m.pin = BMI270_INT_GPIO_BIT;
            #else
    m.pin = IMU_INT_PIN;
            #endif
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable BMI int
    rt_pin_mode(m.pin, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_FALLING, bmi270_int1_handle,
                      (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
        #endif
        #if BMI270_USE_INT2
    // int pin cfg
    m.pin = BMI270_INT2_GPIO_BIT;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable BMI int
    rt_pin_mode(BMI270_INT2_GPIO_BIT, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, bmi270_int2_handle,
                      (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
        #endif

    rt_device_close(device);

    return 0;
}

static int bmi270_gpio_int_disable(void)
{
    struct rt_device_pin_mode m;

        #if BMI270_USE_INT1
            // int pin cfg
            #ifdef IMU_INT_PIN
    m.pin = IMU_INT_PIN;
            #else
    m.pin = BMI270_INT_GPIO_BIT;
            #endif
        #endif
        // for int2
        #if BMI270_USE_INT2
    // int pin cfg
    m.pin = BMI270_INT2_GPIO_BIT;
        #endif
    rt_pin_irq_enable(m.pin, 0);
    rt_pin_detach_irq(m.pin);
    return 0;
}

    #endif // BMI270_USE_INT

    #define PERIODIC_READ_INTERVAL (1000 / IMU_NOARMAL_SAMPLE_RATE)
void bmi270_sensor_task(void *params) // 20ms
{
    int32_t ret;
    api_lock = rt_mutex_create("bmi270_lock", RT_IPC_FLAG_FIFO);
    RT_ASSERT(api_lock != NULL);
    while (1)
    {
    #ifdef BMI270_USE_INT
        if (rt_sem_take(&bmi_int_sem, RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }
        bmi270_api_lock();
        bmi270_int_msg_handler();
        bmi270_api_unlock();
        rt_bmi270_irq_pin_enable(1);
    #else
        rt_thread_mdelay(PERIODIC_READ_INTERVAL);
        bmi270_acc_gyro_evt_handler();
    #endif
    }
}

int bmi270_initialized(void)
{
    memset(&accel_gyro_dev_info, 0x00, sizeof(struct coines_intf_config));
    bmi270_power_onoff(1);
    rt_thread_mdelay(10);
    #if (BMI270_USING_I2C == 1)
    bmi2_interface_init(&bmi2_dev, BMI2_I2C_INTF);
    #else
    bmi2_interface_init(&bmi2_dev, BMI2_SPI_INTF);
    #endif

    /* Assign device address and bus instance to interface pointer */
    int res = bmi270_i2c_init();
    if (res != 0)
    {
        LOG_E("bmi270_i2c_init fail\n");
        goto err_deinit;
    }

    bmi2_dev.intf_ptr = ((void *)&accel_gyro_dev_info);
    LOG_D("bmi2_dev.intf_ptr = %p\n", bmi2_dev.intf_ptr);
    res = bmi270_init(&bmi2_dev);
    LOG_D("bmi270 init rslt = %d", res);
    if (res != BMI2_OK)
    {
        if (res == BMI2_E_DEV_NOT_FOUND)
        {
            LOG_E("BMI2_E_DEV_NOT_FOUND with chip_id = %d",
                  bmi270_get_dev_id());
        }
        else if (res == BMI2_E_COM_FAIL)
        {
            LOG_E("BMI2_E_COM_FAIL");
        }
        goto err_deinit;
    }
    LOG_I("BMI270 chip id = %d", bmi270_get_dev_id());

    res = configure_sensor_interrupt(&bmi2_dev);
    if (res != BMI2_OK)
        goto err_deinit;

    res = configure_sensor_performance_mode(&bmi2_dev);
    LOG_D("configure sensor rslt = %d", res);
    if (res != BMI2_OK)
        goto err_deinit;

    /* 2026-07-17 dial/air-mouse 方向偶發全反的根因處置:清醒時晶片 remap 保持出廠 identity、
       redirect_sensor_data 軟體層負責翻轉(舊方案 remap 只在第一次螢幕睡眠寫入,開機到那
       之前是 identity 軸→方向全反,剛刷機馬上用必中)。睡眠中 wrist-wake enable 會臨時把
       晶片 remap 寫成 NEG 餵抬腕 feature engine(它吃晶片軸),醒來還原;讀值一致性由
       s_chip_remap_neg 聯動 redirect_sensor_data 保證。勿在 init 這裡寫晶片 remap。 */

    #if defined(GSENSOR_UES_FIFO)
    // set fifo
    #endif

    #ifdef BMI_USING_AWT
    bmi270_awt_enable(1);
    #endif /*  BMI_USING_AWT */

    #ifdef BMI_USING_PEDO
    bmi270_pedo_enable(1);
    #endif /*  BMI_USING_PEDO */

    #ifdef BMI270_USE_INT
    // start a thread to check data available
    rt_sem_init(&bmi_int_sem, "bmi_int", 0, RT_IPC_FLAG_FIFO);
    bmi270_gpio_int_enable();
    #endif

    accel_gyro_dev_info.open_flag = 1;
    LOG_D("BMI270 init done\n");
    return res;

err_deinit:
    bmi270_i2c_deinit();
    return res;
}

int bmi270_open(void)
{
    int res = RT_EOK;
    uint8_t rst;

    if (accel_gyro_dev_info.open_flag == 0)
    {
        res = bmi270_initialized();
        if (res != BMI2_OK)
        {
            LOG_E("BMI270 init fail\n");
            return res;
        }
    }
    else
    {
        LOG_D("BMI270 was already initialized\n");
    }

    // bmi270_api_lock();
    // configure_sensor_performance_mode(&bmi2_dev);
    // bmi270_api_unlock();

    if (bmi270_thread == NULL)
    {
        LOG_D("Create bmi270 thread\n");
        bmi270_thread = rt_thread_create("bmi270", bmi270_sensor_task, NULL,
                                         THREAD_STACK_SIZE, THREAD_PRIORITY,
                                         THREAD_TIMESLICE);
        if (bmi270_thread != NULL)
        {
            rt_thread_startup(bmi270_thread);
            LOG_D("6d thread started\n");
        }
        else
        {
            LOG_E("Create 6d thread fail\n");
            return -1;
        }
    }

    LOG_D("BMI270 open success\n");
    return res;
}

int bmi270_high_performance_mode(void)
{
    int res = RT_EOK;
    if (accel_gyro_dev_info.open_flag == 0) // closed before
        return 0;
    bmi270_api_lock();
    res = configure_sensor_performance_mode(&bmi2_dev);
    bmi270_api_unlock();
    if (res != BMI2_OK)
    {
        return res;
    }

    return res;
}

int bmi270_low_power_mode(void)
{
    int res = RT_EOK;
    if (accel_gyro_dev_info.open_flag == 0) // closed before
        return 0;
    bmi270_api_lock();
    res = configure_sensor_power_mode(&bmi2_dev);
    bmi270_api_unlock();
    if (res != BMI2_OK)
    {
        return res;
    }

    return res;
}

int bmi270_close(void)
{
    int res = RT_EOK;

    if (accel_gyro_dev_info.open_flag == 0) // closed before
        return 0;

    if (bmi270_thread != NULL)
    {
        rt_thread_delete(bmi270_thread);
        bmi270_thread = NULL;
    }

    #ifdef BMI270_USE_INT
    bmi270_gpio_int_disable();
    // rt_sem_release(&bmi_int_sem);
    rt_sem_detach(&bmi_int_sem);
    #endif

    #ifdef BMI_USING_AWT
    /*
     * Disable AWT
     */
    // bmi270_wrist_tilt_sens_set(&sens_cont, 0);
    bmi270_awt_enable(0);
    #endif
    #ifdef BMI_USING_PEDO
    bmi270_pedo_enable(0);
    #endif /*  BMI_USING_PEDO */
    bmi270_i2c_deinit();
    // Set the power mode of the accelerometer part of the BMI270 sensor to
    // normal
    bmi270_power_onoff(0);
    accel_gyro_dev_info.open_flag = 0;

    return res;
}

// functions for fifo --------------------------------------------------
int bmi270_fifo_enable(bmi270_fifo_func_t func, bmi270_fifo_odr_t rate)
{
    return 0;
}

int bmi270_fifo_disable(bmi270_fifo_func_t func)
{
    return 0;
}

int bmi270_get_fifo_count(void)
{
    uint16_t value = 0;
    if (accel_gyro_dev_info.open_flag == 0)
        return 0;
    bmi270_api_lock();
    (void)bmi2_get_fifo_length(&value, &bmi2_dev);
    bmi270_api_unlock();
    return (int)value;
}

int bmi270_read_fifo(uint8_t *buf, int len)
{
    if (buf == NULL || len <= 0)
        return 0;
    if (accel_gyro_dev_info.open_flag == 0)
        return 0;

    struct bmi2_fifo_frame f = { 0 };
    f.data = buf;
    f.length = (uint16_t)len;

    bmi270_api_lock();
    int8_t rslt = bmi2_read_fifo_data(&f, &bmi2_dev);
    bmi270_api_unlock();

    return (rslt == BMI2_OK) ? len : 0;
}

int bmi270_set_fifo_threshold(int thd)
{
    if (accel_gyro_dev_info.open_flag == 0)
        return -1;
    bmi270_api_lock();
    int8_t rslt = bmi2_set_fifo_wm((uint16_t)thd, &bmi2_dev);
    bmi270_api_unlock();
    return (rslt == BMI2_OK) ? 0 : -1;
}

int bmi270_get_waterm_status(void)
{
    return 0;
}

int bmi270_get_overrun_status(void)
{
    return 0;
}

int bmi270_get_fifo_full_status(void)
{
    return 0;
}

int bmi270_get_fifo_empty_status(void)
{
    return 0;
}

int bmi270_set_fifo_mode(uint8_t val)
{
    return 0;
}

int bmi270_get_fifo_pattern(void)
{
    return 0;
}

int bmi270_get_fifo_data_arr(void)
{
    return 0;
}

/* ===== Periodic FIFO drain for screen-off mode =============================
   While the screen is off LCPU is in LIGHT sleep. We can't run the SW AHRS
   per DRDY sample (would defeat the sleep), but we don't want quaternion
   drift either — when wrist-wake fires the next gesture-detect pass needs a
   converged orientation immediately. Compromise: keep accel+gyro running at
   25 Hz, route the BMI270 FIFO watermark to INT1 so the chip wakes us about
   once a second, drain all accumulated frames through the same AHRS path
   one shot, then go back to sleep.

   bmi270_set_fifo_wm_int(en, wm_bytes):
     en=1 — enable header-mode FIFO with accel+gyro, set watermark in BYTES,
            route FWM to INT1.
     en=0 — un-route FWM, disable FIFO data sources.
     wm_bytes — frame size with header mode is 13 (1 hdr + 6 acc + 6 gyr).
            ~1 s @ 25 Hz = 325 bytes; sized in the caller. */
int bmi270_set_fifo_wm_int(int en, uint16_t wm_bytes)
{
    if (accel_gyro_dev_info.open_flag == 0)
        return -1;

    const uint16_t fifo_cfg = (uint16_t)(BMI2_FIFO_ACC_EN |
                                          BMI2_FIFO_GYR_EN |
                                          BMI2_FIFO_HEADER_EN);
    int8_t rslt = BMI2_OK;
    bmi270_api_lock();

    if (en)
    {
        /* Set watermark first so the chip doesn't immediately fire FWM at
           the default (low) threshold when FIFO comes online. */
        rslt = bmi2_set_fifo_wm(wm_bytes, &bmi2_dev);
        if (rslt != BMI2_OK)
            goto unlock;
        rslt = bmi2_set_fifo_config(fifo_cfg, BMI2_ENABLE, &bmi2_dev);
        if (rslt != BMI2_OK)
            goto unlock;
        rslt = bmi2_map_data_int(BMI2_FWM_INT, BMI2_INT1, &bmi2_dev);
        if (rslt != BMI2_OK)
            goto unlock;
        LOG_I("BMI270 FIFO WM int armed: wm=%u bytes", wm_bytes);
    }
    else
    {
        (void)bmi2_map_data_int(BMI2_FWM_INT, BMI2_INT_NONE, &bmi2_dev);
        rslt = bmi2_set_fifo_config(fifo_cfg, BMI2_DISABLE, &bmi2_dev);
        LOG_I("BMI270 FIFO WM int disarmed");
    }

unlock:
    bmi270_api_unlock();
    return (rslt == BMI2_OK) ? 0 : -1;
}

/* Drain the BMI270 FIFO and replay every accel+gyro pair through the same
   AHRS / gesture path that DRDY uses. Returns the number of frame pairs
   processed (negative on error).

   Buffer sized for ~2 s of headroom @ 25 Hz / 13 B per frame = 650 B; max
   FIFO depth on BMI270 is ~2 KB so anything beyond is truncated and the
   next watermark drains the rest. All scratch arrays are static — the
   sensor task stack is only 1.25 KB. */
int bmi270_drain_fifo_to_ahrs(void)
{
    if (accel_gyro_dev_info.open_flag == 0)
        return -1;

    static uint8_t fifo_buf[768];
    static struct bmi2_sens_axes_data acc[60];
    static struct bmi2_sens_axes_data gyr[60];
    struct bmi2_fifo_frame f = { 0 };
    uint16_t fifo_len = 0;
    int8_t rslt;

    bmi270_api_lock();

    rslt = bmi2_get_fifo_length(&fifo_len, &bmi2_dev);
    if (rslt != BMI2_OK || fifo_len == 0)
    {
        bmi270_api_unlock();
        return (rslt == BMI2_OK) ? 0 : -1;
    }
    if (fifo_len > sizeof(fifo_buf))
        fifo_len = sizeof(fifo_buf);

    f.data = fifo_buf;
    f.length = fifo_len;
    rslt = bmi2_read_fifo_data(&f, &bmi2_dev);
    if (rslt != BMI2_OK)
    {
        bmi270_api_unlock();
        return -1;
    }

    uint16_t acc_cnt = sizeof(acc) / sizeof(acc[0]);
    uint16_t gyr_cnt = sizeof(gyr) / sizeof(gyr[0]);
    (void)bmi2_extract_accel(acc, &acc_cnt, &f, &bmi2_dev);
    (void)bmi2_extract_gyro(gyr, &gyr_cnt, &f, &bmi2_dev);

    bmi270_api_unlock();

    uint16_t pairs = (acc_cnt < gyr_cnt) ? acc_cnt : gyr_cnt;
    for (uint16_t i = 0; i < pairs; i++)
    {
        local_watch_accel = redirect_sensor_data(&acc[i]);
        local_watch_gyro = redirect_sensor_data(&gyr[i]);
        /* Update only the global Mahony quaternion — skip gravity / hand
           tracking / health / sensor_q work that handle_imu_data() does
           on the awake path. Goal here is solely to keep global_q from
           drifting so wrist-wake → gesture-detect starts converged. */
        if (imu_data_fetch(&watch_sensor.imu_data) == RT_EOK)
        {
    #ifdef BSP_USING_GESTURE_DETECT
            update_global_attitude(&watch_sensor.imu_data.acce,
                                   &watch_sensor.imu_data.gyro);
    #endif
        }
    }

    return (int)pairs;
}

// function for awt
int bmi270_awt_enable(int en)
{
    int res;
    // enable AWT fucntion, other configure seems not work

    if (en != 0) // enable
    {
        /*
         * Set tilt mask
         */
        /*
         * Enable AWT
         */
        // enable interrupt
    #ifdef BMI270_USE_INT
    #endif
    }
    else // disable
    {
        /*
         * Disable AWT
         */
    #ifdef BMI270_USE_INT
    #endif
    }

    return 0;
}

// function for pedometer ----------------------------
int bmi270_pedo_enable(int en)
{
    return 0;
}

// function for hardware wrist-wake ------------------
/* hw_wrist_wake_armed declared near the top of this file so the int handler
   can see it. */

/* Default handler: log only. The LCPU application overrides this to forward
   the event up to HCPU via main_send_hand_lift_event(). */
RT_WEAK void bmi270_on_wrist_wake_detected(void)
{
    LOG_I("BMI270 HW wrist-wake (default handler, override me) tick=%u",
          (unsigned)rt_tick_get());
}

RT_WEAK void bmi270_on_any_motion_detected(void)
{
    LOG_I("BMI270 any-motion (default handler, override me) tick=%u",
          (unsigned)rt_tick_get());
}

int bmi270_any_motion_is_enabled(void)
{
    return any_motion_armed ? 1 : 0;
}

int bmi270_hw_wrist_wake_is_enabled(void)
{
    return hw_wrist_wake_armed ? 1 : 0;
}

/* Set 0 to use Bosch defaults (start here when bringing wrist-wake up — if
   defaults work but custom values don't, the tuning is too tight). Set 1
   once the basic gesture detection is confirmed working. */
#ifndef BMI270_WRIST_WAKE_USE_CUSTOM_TUNING
    #define BMI270_WRIST_WAKE_USE_CUSTOM_TUNING 1
#endif

/* The raise-wrist WAKE feature (BMI2_WRIST_WEAR_WAKE_UP, the chip's ~1 s lift
   detector) is superseded by the host any_motion path — set 0 to disable it.
   The WRIST_GESTURE feature (flick / pivot / jiggle) is kept regardless. Set 1
   to restore the original WW + GEST behaviour. */
#ifndef BMI270_ENABLE_WRIST_WEAR_WAKEUP
    #define BMI270_ENABLE_WRIST_WEAR_WAKEUP 1
#endif

int bmi270_hw_wrist_wake_enable(int en)
{
    if (accel_gyro_dev_info.open_flag == 0)
    {
        LOG_W("hw_wrist_wake_enable: bmi270 not open");
        return -1;
    }

    int8_t rslt;

    bmi270_api_lock();

    if (en)
    {
        /* Order mirrors Bosch reference (bmi270_examples/wrist_wear_wakeup):
             1. Enable ACCEL + WRIST_WEAR_WAKE_UP atomically in one call.
                Bosch's API turns on the feature engine at this point — if
                accel isn't included in the same list, the feature won't
                receive samples even though the engine is "enabled".
             2. Get default config, optionally override, set back.
             3. Explicit INT pin config (active-low, push-pull, non-latch).
             4. Map the feature interrupt to INT1.
           Each step has its own log so we can tell which one fails. */

    #if BMI270_ENABLE_WRIST_WEAR_WAKEUP
        uint8_t sens_list[3] = { BMI2_ACCEL, BMI2_WRIST_WEAR_WAKE_UP, BMI2_WRIST_GESTURE };
        rslt = bmi270_sensor_enable(sens_list, 3, &bmi2_dev);
    #else
        uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_WRIST_GESTURE };
        rslt = bmi270_sensor_enable(sens_list, 2, &bmi2_dev);
    #endif
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake step1 sensor_enable failed: %d", rslt);
            goto out;
        }
        /* Read accel config back to confirm sensor_enable didn't reset our
           50 Hz / perf-opt config (Bosch API normally only flips the enable
           bit, but we want proof — wrist-wake silently fails if accel falls
           below 50 Hz or filter_perf drops to power-opt). */
        struct bmi2_sens_config acc_dbg = { .type = BMI2_ACCEL };
        if (bmi270_get_sensor_config(&acc_dbg, 1, &bmi2_dev) == BMI2_OK)
        {
            LOG_I("wrist-wake step1: ACCEL+WW enabled (accel odr=0x%02x bwp=0x%02x range=0x%02x perf=%d)",
                  acc_dbg.cfg.acc.odr, acc_dbg.cfg.acc.bwp,
                  acc_dbg.cfg.acc.range, acc_dbg.cfg.acc.filter_perf);
        }
        else
        {
            LOG_I("wrist-wake step1: ACCEL+WW enabled (accel readback failed)");
        }

        /* Step 1.5 — axis remap.
           Bosch's wrist-wake feature engine uses chip-internal axes. If the
           IMU is physically mounted with axes pointing different directions
           than the watch face, the algorithm sees gestures in the wrong
           frame and never fires.

           Per visual comparison with Bosch's BMI270 axis diagram, on this
           watch: Y matches, but X and Z point opposite to the chip's
           natural axes. So we tell the chip to negate X and Z before
           feeding the feature engine.

           Logged before/after so we can confirm what's stored. */
        /* Step 1.5 — 睡眠中臨時把晶片 remap 寫成 NEG/NEG/NEG:抬腕/腕勢 feature engine 吃
           晶片內部軸,identity 下軸向錯、手勢永遠不 fire(founder 2026-07-17 回報抬腕壞)。
           只在螢幕暗的睡眠窗口生效,醒來 disable 時還原 identity(見 else 分支);讀值一致性
           由 s_chip_remap_neg 聯動 redirect_sensor_data 保證(晶片翻→軟體直通)。寫失敗
           fail-safe:flag 不設、晶片留 identity→讀值仍對,只是該晚抬腕失靈+LOG_E。 */
        {
            const struct bmi2_remap remap_neg = {
                .x = BMI2_NEG_X, .y = BMI2_NEG_Y, .z = BMI2_NEG_Z,
            };
            rslt = bmi2_set_remap_axes(&remap_neg, &bmi2_dev);
            if (rslt == BMI2_OK)
            {
                struct bmi2_remap chk = { 0 };
                if (bmi2_get_remap_axes(&chk, &bmi2_dev) == BMI2_OK &&
                    chk.x == remap_neg.x && chk.y == remap_neg.y && chk.z == remap_neg.z)
                {
                    s_chip_remap_neg = true;
                    LOG_I("wrist-wake step1.5: chip remap -> NEG (feature frame), sw redirect passthrough");
                }
                else
                {
                    LOG_E("wrist-wake step1.5 remap verify mismatch — chip frame unknown, keeping sw flip");
                }
            }
            else
            {
                LOG_E("wrist-wake step1.5 set_remap_axes failed: %d — wrist-wake axes wrong tonight", rslt);
            }
            rslt = BMI2_OK; /* remap 失敗不擋整個 enable(accel/INT 部分照常武裝) */
        }

    #if BMI270_ENABLE_WRIST_WEAR_WAKEUP
        struct bmi2_sens_config cfg = { .type = BMI2_WRIST_WEAR_WAKE_UP };
        rslt = bmi270_get_sensor_config(&cfg, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake step2 get_config failed: %d", rslt);
            goto out;
        }
        LOG_I("wrist-wake step2 defaults: angle nf=%u f=%u tilt lr=%u ll=%u pd=%u pu=%u",
              cfg.cfg.wrist_wear_wake_up.min_angle_nonfocus,
              cfg.cfg.wrist_wear_wake_up.min_angle_focus,
              cfg.cfg.wrist_wear_wake_up.max_tilt_lr,
              cfg.cfg.wrist_wear_wake_up.max_tilt_ll,
              cfg.cfg.wrist_wear_wake_up.max_tilt_pd,
              cfg.cfg.wrist_wear_wake_up.max_tilt_pu);

    #if BMI270_WRIST_WAKE_USE_CUSTOM_TUNING
        cfg.cfg.wrist_wear_wake_up.min_angle_focus    = BMI270_WRIST_WAKE_MIN_ANGLE_FOCUS;
        cfg.cfg.wrist_wear_wake_up.min_angle_nonfocus = BMI270_WRIST_WAKE_MIN_ANGLE_NONFOCUS;
        cfg.cfg.wrist_wear_wake_up.max_tilt_lr        = BMI270_WRIST_WAKE_MAX_TILT_LR;
        cfg.cfg.wrist_wear_wake_up.max_tilt_ll        = BMI270_WRIST_WAKE_MAX_TILT_LL;
        cfg.cfg.wrist_wear_wake_up.max_tilt_pd        = BMI270_WRIST_WAKE_MAX_TILT_PD;
        cfg.cfg.wrist_wear_wake_up.max_tilt_pu        = BMI270_WRIST_WAKE_MAX_TILT_PU;
    #endif
        rslt = bmi270_set_sensor_config(&cfg, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake step3 set_config failed: %d", rslt);
            goto out;
        }
        LOG_I("wrist-wake step3 config set (custom_tuning=%d)",
              BMI270_WRIST_WAKE_USE_CUSTOM_TUNING);
    #endif /* BMI270_ENABLE_WRIST_WEAR_WAKEUP */

        /* Explicit INT pin re-config — defensive against any earlier path
           (drdy un-routing, gyro suspend etc.) that may have left the pin
           in a state where the feature interrupt can't drive it. Mirror
           Bosch wrist_gesture_hw_int.c. */
        struct bmi2_int_pin_config pin_cfg = { 0 };
        rslt = bmi2_get_int_pin_config(&pin_cfg, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake step4 get_int_pin_config failed: %d", rslt);
            goto out;
        }
    #if BMI270_USE_INT1
        pin_cfg.pin_type           = BMI2_INT1;
        pin_cfg.pin_cfg[0].lvl        = BMI2_INT_ACTIVE_LOW;
        pin_cfg.pin_cfg[0].od         = BMI2_INT_PUSH_PULL;
        pin_cfg.pin_cfg[0].output_en  = BMI2_INT_OUTPUT_ENABLE;
        pin_cfg.pin_cfg[0].input_en   = BMI2_INT_INPUT_DISABLE;
        const enum bmi2_hw_int_pin int_pin = BMI2_INT1;
    #else
        pin_cfg.pin_type           = BMI2_INT2;
        pin_cfg.pin_cfg[1].lvl        = BMI2_INT_ACTIVE_LOW;
        pin_cfg.pin_cfg[1].od         = BMI2_INT_PUSH_PULL;
        pin_cfg.pin_cfg[1].output_en  = BMI2_INT_OUTPUT_ENABLE;
        pin_cfg.pin_cfg[1].input_en   = BMI2_INT_INPUT_DISABLE;
        const enum bmi2_hw_int_pin int_pin = BMI2_INT2;
    #endif
        pin_cfg.int_latch = BMI2_INT_NON_LATCH;
        rslt = bmi2_set_int_pin_config(&pin_cfg, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake step4 set_int_pin_config failed: %d", rslt);
            goto out;
        }
        LOG_I("wrist-wake step4: pin %d active-low push-pull non-latch", int_pin);

    #if BMI270_ENABLE_WRIST_WEAR_WAKEUP
        struct bmi2_sens_int_config sens_int[2] = {
            { .type = BMI2_WRIST_WEAR_WAKE_UP, .hw_int_pin = int_pin },
            { .type = BMI2_WRIST_GESTURE,      .hw_int_pin = int_pin },
        };
        rslt = bmi270_map_feat_int(sens_int, 2, &bmi2_dev);
    #else
        struct bmi2_sens_int_config sens_int[1] = {
            { .type = BMI2_WRIST_GESTURE, .hw_int_pin = int_pin },
        };
        rslt = bmi270_map_feat_int(sens_int, 1, &bmi2_dev);
    #endif
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake step5 map_feat_int failed: %d", rslt);
            goto out;
        }
        LOG_I("wrist-wake step5: feat int mapped to pin %d", int_pin);

        /* Step 6 — wrist-gesture config (left arm by default; flip via
           BMI270_WRIST_WEAR_ON_RIGHT_ARM=1 if user wears watch on right). */
    #ifndef BMI270_WRIST_WEAR_ON_RIGHT_ARM
        #define BMI270_WRIST_WEAR_ON_RIGHT_ARM 0
    #endif
        struct bmi2_sens_config gest_cfg = { .type = BMI2_WRIST_GESTURE };
        rslt = bmi270_get_sensor_config(&gest_cfg, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake step6 gest get_config failed: %d", rslt);
            goto out;
        }
        gest_cfg.cfg.wrist_gest.wearable_arm = BMI270_WRIST_WEAR_ON_RIGHT_ARM;
        /* Push GEST sensitivity to spec maximum. Bosch defaults are tuned
           for low false-positive rate; we want the opposite — fire fast on
           any plausible flick / pivot_up and let the LCPU-side pose filter
           (is_in_viewing_pose_from_accel) reject if not in watchface_visible envelope.
           Ranges per bmi2_defs.h:2204-2222:
             min_flick_peak    1448..1774  → 1448 (most sensitive)
             min_flick_samples 3..5        → 3    (catches faster flicks)
             max_duration      150..250    → 250  (allows slower motions) */
        gest_cfg.cfg.wrist_gest.min_flick_peak    = 1448;
        gest_cfg.cfg.wrist_gest.min_flick_samples = 3;
        gest_cfg.cfg.wrist_gest.max_duration      = 250;
        rslt = bmi270_set_sensor_config(&gest_cfg, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake step6 gest set_config failed: %d", rslt);
            goto out;
        }
        LOG_I("wrist-wake step6: wrist-gesture armed (arm=%s, min_flick_peak=%u, min_flick_samples=%u, max_duration=%u)",
              BMI270_WRIST_WEAR_ON_RIGHT_ARM ? "right" : "left",
              gest_cfg.cfg.wrist_gest.min_flick_peak,
              gest_cfg.cfg.wrist_gest.min_flick_samples,
              gest_cfg.cfg.wrist_gest.max_duration);

        hw_wrist_wake_armed = true;
        LOG_I("BMI270 HW gesture armed (WEAR_WAKEUP=%d, GESTURE=1)",
              BMI270_ENABLE_WRIST_WEAR_WAKEUP);
    }
    else
    {
        /* Disable both wake features. Don't disable accel here — other
           consumers (SW hand_tracking) re-enable it when screen turns on,
           and the transition would briefly drop accel power. Just turn
           off the feature engines; the leftover INT mapping is harmless
           because the feature engines won't fire. */
    #if BMI270_ENABLE_WRIST_WEAR_WAKEUP
        uint8_t sensors[] = { BMI2_WRIST_WEAR_WAKE_UP, BMI2_WRIST_GESTURE };
        rslt = bmi270_sensor_disable(sensors, 2, &bmi2_dev);
    #else
        uint8_t sensors[] = { BMI2_WRIST_GESTURE };
        rslt = bmi270_sensor_disable(sensors, 1, &bmi2_dev);
    #endif
        if (rslt != BMI2_OK)
        {
            LOG_E("wrist-wake sensor_disable failed: %d", rslt);
            goto out;
        }
        /* 醒來:晶片 remap 還原 identity(清醒時軟體全責翻轉,見 redirect_sensor_data)。
           失敗時 flag 保持 true(跟隨實際晶片狀態→軟體維持直通,讀值框架仍對)+ retry 一次。 */
        if (s_chip_remap_neg)
        {
            const struct bmi2_remap remap_id = { .x = BMI2_X, .y = BMI2_Y, .z = BMI2_Z };
            int8_t r2 = bmi2_set_remap_axes(&remap_id, &bmi2_dev);
            if (r2 != BMI2_OK)
                r2 = bmi2_set_remap_axes(&remap_id, &bmi2_dev);
            if (r2 == BMI2_OK)
            {
                s_chip_remap_neg = false;
                LOG_I("wrist-wake disarm: chip remap -> identity, sw flip active");
            }
            else
            {
                LOG_E("wrist-wake disarm: remap restore FAILED (%d) — chip stays NEG, sw passthrough keeps frame", r2);
            }
        }
        hw_wrist_wake_armed = false;
        LOG_I("BMI270 HW wrist-wake disarmed");
    }

out:
    bmi270_api_unlock();
    return (rslt == BMI2_OK) ? 0 : -1;
}

/* any_motion duration: how long the slope must stay above threshold before the
   interrupt fires. Per the Bosch API doc (bmi270.c:771 "expressed in 50 Hz
   samples (20 msec)"), 1 LSB = a FIXED 20 ms — it tracks the feature engine's
   internal 50 Hz clock, NOT the 25 Hz screen-off accel ODR. So 5 = ~100 ms
   regardless of the accel ODR (the engine samples internally, see :608). */
#ifndef BMI270_ANY_MOT_DURATION
    #define BMI270_ANY_MOT_DURATION  10   /* MAX (13-bit); 8191 x 20ms = ~163 s */
#endif

/* Enable BMI2_ANY_MOTION as an ADDITIONAL fast wake trigger alongside
   wrist-wake. Additive — does not touch the wrist-wake feature. Order follows
   the Bosch reference: configure the feature, map its interrupt, THEN enable
   it last. Accel is already enabled+configured by the wrist-wake path, so it
   is deliberately NOT re-enabled here (avoids disturbing its ODR / perf).
   Arm in the screen-off path next to bmi270_hw_wrist_wake_enable(1). */
int bmi270_any_motion_enable(int en)
{
    if (accel_gyro_dev_info.open_flag == 0)
    {
        LOG_W("any_motion_enable: bmi270 not open");
        return -1;
    }

    int8_t rslt;
    bmi270_api_lock();

    if (en)
    {
        /* 1. Get current (Bosch default) config. */
        struct bmi2_sens_config cfg = { .type = BMI2_ANY_MOTION };
        rslt = bmi270_get_sensor_config(&cfg, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("any_motion step1 get_config failed: %d", rslt);
            goto out;
        }
        LOG_D("any_motion defaults: dur=%u thr=%u x=%u y=%u z=%u",
              cfg.cfg.any_motion.duration, cfg.cfg.any_motion.threshold,
              cfg.cfg.any_motion.select_x, cfg.cfg.any_motion.select_y,
              cfg.cfg.any_motion.select_z);

        /* 2. Override duration and restrict slope detection to the Y axis only
           (raise-wrist motion is dominantly on Y). Keep the Bosch default
           threshold. NOTE: axes are in the feature-engine frame AFTER the
           wrist-wake remap (bmi270_hw_wrist_wake_enable runs first), so "Y"
           here is the watch Y. */
        cfg.cfg.any_motion.duration = BMI270_ANY_MOT_DURATION;
        cfg.cfg.any_motion.select_x = 0;
        cfg.cfg.any_motion.select_y = 0;
        cfg.cfg.any_motion.select_z = 1;
        cfg.cfg.any_motion.threshold = 250; /* Bosch default=170; 2x to reduce false wakes */
        rslt = bmi270_set_sensor_config(&cfg, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("any_motion step2 set_config failed: %d", rslt);
            goto out;
        }
        LOG_I("any_motion set: dur=%u (~%u ms) thr=%u sel x=%u y=%u z=%u",
              cfg.cfg.any_motion.duration,
              (unsigned)(BMI270_ANY_MOT_DURATION * 20),
              cfg.cfg.any_motion.threshold,
              cfg.cfg.any_motion.select_x, cfg.cfg.any_motion.select_y,
              cfg.cfg.any_motion.select_z);

        /* 3. Map the feature interrupt to the same pin wrist-wake uses (its
           electrical config is already applied by the wrist-wake path). */
    #if BMI270_USE_INT1
        const enum bmi2_hw_int_pin int_pin = BMI2_INT1;
    #else
        const enum bmi2_hw_int_pin int_pin = BMI2_INT2;
    #endif
        struct bmi2_sens_int_config sens_int[1] = {
            { .type = BMI2_ANY_MOTION, .hw_int_pin = int_pin },
        };
        rslt = bmi270_map_feat_int(sens_int, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("any_motion step3 map_feat_int failed: %d", rslt);
            goto out;
        }

        /* 4. Enable the feature LAST (Bosch order). ANY_MOTION only — accel
           stays as the wrist-wake path left it. */
        uint8_t sens_list[1] = { BMI2_ANY_MOTION };
        rslt = bmi270_sensor_enable(sens_list, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("any_motion step4 sensor_enable failed: %d", rslt);
            goto out;
        }
        any_motion_armed = true;
        LOG_I("BMI270 any-motion armed (pin %d, dur=%u, thr=%u)",
              int_pin, BMI270_ANY_MOT_DURATION,
              (unsigned)cfg.cfg.any_motion.threshold);
    }
    else
    {
        uint8_t sensors[1] = { BMI2_ANY_MOTION };
        rslt = bmi270_sensor_disable(sensors, 1, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("any_motion sensor_disable failed: %d", rslt);
            goto out;
        }
        any_motion_armed = false;
        LOG_I("BMI270 any-motion disarmed");
    }

out:
    bmi270_api_unlock();
    return (rslt == BMI2_OK) ? 0 : -1;
}

// function for hardware step counter ----------------
int bmi270_hw_step_counter_enable(int en)
{
    if (accel_gyro_dev_info.open_flag == 0)
    {
        LOG_W("hw_step_counter_enable: bmi270 not open");
        return -1;
    }

    int8_t rslt;

    bmi270_api_lock();
    if (en)
    {
        /* Mirror Bosch bmi270_examples/step_counter: enable ACCEL +
           STEP_COUNTER in the same call. Without ACCEL in the list the
           feature engine may not pick up samples (same pattern as wrist-
           wake). */
        uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_STEP_COUNTER };
        rslt = bmi270_sensor_enable(sens_list, 2, &bmi2_dev);
        if (rslt != BMI2_OK)
        {
            LOG_E("step counter enable(ACCEL+SC) failed: %d", rslt);
            goto step_out;
        }

        /* watermark_level = 1 = chip updates step_counter_output every
           20 steps (BMI270 minimum granularity). The 25-param wrist
           algorithm tuning is loaded by the chip's firmware blob and
           does not need host configuration. */
        struct bmi2_sens_config cfg = { .type = BMI2_STEP_COUNTER };
        rslt = bmi270_get_sensor_config(&cfg, 1, &bmi2_dev);
        if (rslt == BMI2_OK)
        {
            cfg.cfg.step_counter.watermark_level = 1;
            rslt = bmi270_set_sensor_config(&cfg, 1, &bmi2_dev);
            if (rslt != BMI2_OK)
            {
                LOG_E("step counter set_config failed: %d", rslt);
            }
        }
        LOG_I("BMI270 HW step counter enabled");
    }
    else
    {
        uint8_t sensors[] = { BMI2_STEP_COUNTER };
        rslt = bmi270_sensor_disable(sensors, 1, &bmi2_dev);
        if (rslt == BMI2_OK)
            LOG_I("BMI270 HW step counter disabled");
        else
            LOG_E("step counter disable failed: %d", rslt);
    }
step_out:
    bmi270_api_unlock();
    return (rslt == BMI2_OK) ? 0 : -1;
}

int bmi270_hw_step_counter_read(uint32_t *steps)
{
    if (accel_gyro_dev_info.open_flag == 0 || steps == NULL)
        return -1;

    struct bmi2_feat_sensor_data fdata = {.type = BMI2_STEP_COUNTER};
    bmi270_api_lock();
    int8_t rslt = bmi270_get_feature_data(&fdata, 1, &bmi2_dev);
    bmi270_api_unlock();
    if (rslt != BMI2_OK)
    {
        LOG_D("step_counter_read failed: %d", rslt);
        return -1;
    }
    *steps = fdata.sens_data.step_counter_output;
    return 0;
}

int bmi270_hw_step_counter_reset(void)
{
    if (accel_gyro_dev_info.open_flag == 0)
        return -1;

    /* Reset by toggling the reset_counter bit in the step config. The chip
       clears the running counter on the rising edge of this bit. */
    struct bmi2_sens_config cfg = {.type = BMI2_STEP_COUNTER};
    bmi270_api_lock();
    int8_t rslt = bmi270_get_sensor_config(&cfg, 1, &bmi2_dev);
    if (rslt == BMI2_OK)
    {
        cfg.cfg.step_counter.reset_counter = 1;
        rslt = bmi270_set_sensor_config(&cfg, 1, &bmi2_dev);
    }
    bmi270_api_unlock();
    return (rslt == BMI2_OK) ? 0 : -1;
}

int bmi270_set_drdy_int_routing(int en)
{
    if (accel_gyro_dev_info.open_flag == 0)
        return -1;

    int8_t rslt;
    bmi270_api_lock();
    #if BMI270_USE_INT1
    enum bmi2_hw_int_pin pin = en ? BMI2_INT1 : BMI2_INT_NONE;
    #else
    enum bmi2_hw_int_pin pin = en ? BMI2_INT2 : BMI2_INT_NONE;
    #endif
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, pin, &bmi2_dev);
    bmi270_api_unlock();
    if (rslt != BMI2_OK)
    {
        LOG_E("set_drdy_int_routing(%d) failed: %d", en, rslt);
        return -1;
    }
    LOG_I("BMI270 DRDY interrupt %s", en ? "routed to INT" : "un-routed");
    return 0;
}

int bmi270_set_gyro_suspend(int suspend)
{
    if (accel_gyro_dev_info.open_flag == 0)
        return -1;

    int8_t rslt;
    uint8_t gyro_sensor = BMI2_GYRO;
    bmi270_api_lock();
    if (suspend)
    {
        rslt = bmi2_sensor_disable(&gyro_sensor, 1, &bmi2_dev);
        if (rslt == BMI2_OK)
            LOG_I("BMI270 gyro suspended");
    }
    else
    {
        rslt = bmi2_sensor_enable(&gyro_sensor, 1, &bmi2_dev);
        if (rslt == BMI2_OK)
            LOG_I("BMI270 gyro resumed");
    }
    bmi270_api_unlock();
    return (rslt == BMI2_OK) ? 0 : -1;
}

int bmi270_pedo_fifo2step(uint8_t *buf, int len)
{
    /*
        just for only pedo mode.
        1 array with 6 bytes
        3 bytes for timestamp, 1 byte not used, 2 byte for number of steps
        return the latest step
    */
    int i, cnt;
    uint16_t *tbuf = (uint16_t *)buf;
    uint32_t timestamp;
    uint16_t step = 0;

    if (buf == NULL || len == 0)
        return 0;

    cnt = len / 6;

    for (i = 0; i < cnt; i++)
    {
        timestamp = (uint32_t)(*tbuf);
        timestamp = (uint32_t)((*(tbuf + 1)) >> 8) | (timestamp << 8);
        step = *(tbuf + 2);
        LOG_D("T %d, S %d\n", timestamp, step);
        tbuf += 3;
    }

    return (int)step;
}

// function for handle and address --------------------
uint32_t bmi270_get_bus_handle(void)
{
    return (uint32_t)(accel_gyro_dev_info.bus_handle);
}
uint8_t bmi270_get_dev_addr(void)
{
    return accel_gyro_dev_info.dev_addr;
}
uint8_t bmi270_get_dev_id(void)
{
    return bmi2_dev.chip_id;
}

int bmi270_self_check(void)
{
    int ret = 0;
    uint8_t whoami;
    ret = bmi2_get_regs(BMI2_CHIP_ID_ADDR, &whoami, 1, &bmi2_dev);
    if (whoami != bmi270_get_dev_id())
    {
        return -1;
    }
    return 0;
}

int bmi270_accel_read(int16_t *psX, int16_t *psY, int16_t *psZ)
{
    #ifdef BMI270_USE_INT
    if (accelerationAvailable() == 0)
        return -1;
    #endif
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sensor_data = {{0}};
    rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
    *psX = sensor_data.acc.x;
    *psY = sensor_data.acc.y;
    *psZ = sensor_data.acc.z;
    LOG_D("Accel: x=%d, y=%d, z=%d", *psX, *psY, *psZ);
    #ifdef BMI270_USE_INT
    return rslt;
    #else
    if (sensor_data.status & BMI2_DRDY_ACC)
    {
        return rslt;
    }
    else
    {
        return -1;
    }
    #endif
}
int bmi270_gyro_read(int16_t *psX, int16_t *psY, int16_t *psZ)
{
    #ifdef BMI270_USE_INT
    if (gyroscopeAvailable() == 0)
        return -1;
    #endif
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sensor_data = {{0}};
    rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
    *psX = sensor_data.gyr.x;
    *psY = sensor_data.gyr.y;
    *psZ = sensor_data.gyr.z;
    LOG_D("Gyro: x=%d, y=%d, z=%d", *psX, *psY, *psZ);
    #ifdef BMI270_USE_INT
    return rslt;
    #else
    if (sensor_data.status & BMI2_DRDY_GYR)
    {
        return rslt;
    }
    else
    {
        return -1;
    }
    #endif
}

int bmi270_tempra_read(float *tempra)
{
    return 0;
}

int bmi270_step_read(int32_t *step)
{
    return 0;
}

void bmi270_accel_set_range(uint8_t range)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi2_sens_config config[1];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 1, &bmi2_dev);

    #ifdef BMI270_USE_INT
    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi2_dev);
    #endif
    if (rslt == BMI2_OK)
    {
        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = range;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 1, &bmi2_dev);
    }
}

void bmi270_gyro_set_range(uint8_t range)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi2_sens_config config[1];

    /* Configure the type of feature. */
    config[0].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 1, &bmi2_dev);

    #ifdef BMI270_USE_INT
    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi2_dev);
    #endif
    if (rslt == BMI2_OK)
    {
        /* Gyroscope Angular Rate Measurement Range.By default the range is
         * 2000dps. */
        config[0].cfg.gyr.range = range;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 1, &bmi2_dev);
    }
}

    #if defined(RT_USING_FINSH) && !defined(LCPU_MEM_OPTIMIZE)
        #define DRV_BMI270_TEST
    #endif
    #ifdef DRV_BMI270_TEST

        #define BMI_TEST_FIFO_CNT (60)
int16_t fifo_buf[BMI_TEST_FIFO_CNT];
int bmi270_test(int argc, char *argv[])
{
    if (argc < 2)
    {
        LOG_I("Invalid parameter\n");
        return 0;
    }

    if (strcmp(argv[1], "-open") == 0)
    {
        if (bmi270_initialized() == 0)
        {
            bmi270_open();
            LOG_I("bmi270 open success\n");
        }
        else
        {
            LOG_E("bmi270 open fail\n");
        }
    }
    else if (strcmp(argv[1], "-close") == 0)
    {
        bmi270_close();
    }
    else if (strcmp(argv[1], "-fifo") == 0)
    {
        int en = atoi(argv[2]);
        if (en == 1)
        {
            bmi270_fifo_enable(0, 2);
            bmi270_fifo_enable(1, 2);
        }
        else
        {
            bmi270_fifo_disable(0);
            bmi270_fifo_disable(1);
        }
    }
    else if (strcmp(argv[1], "-fcnt") == 0)
    {
        int i, len;
        int cnt = bmi270_get_fifo_count();
        LOG_I("fifo deep %d\n", cnt);
        len = cnt > BMI_TEST_FIFO_CNT ? BMI_TEST_FIFO_CNT : cnt;
        bmi270_read_fifo((uint8_t *)fifo_buf, len * 2);
        cnt = bmi270_get_fifo_count();
        LOG_I("fifo deep after read %d:  %d\n", BMI_TEST_FIFO_CNT, cnt);
        for (i = 0; i < BMI_TEST_FIFO_CNT; i++)
        {
            LOG_I("%d ", fifo_buf[i]);
            if (i % 3 == 2)
                LOG_I("\n");
        }
    }
    else if (strcmp(argv[1], "-waterm") == 0)
    {
        int cnt, waterm;
        bmi270_set_fifo_threshold(100 * 6); // 100 array for 3 acc and 3 gyro
        cnt = bmi270_get_fifo_count();
        LOG_I("org fifo deep %d\n", cnt);
        bmi270_fifo_enable(0, 2);
        bmi270_fifo_enable(1, 2);
        do
        {
            rt_thread_delay(5);
            waterm = bmi270_get_waterm_status();
        } while (waterm == 0);
        cnt = bmi270_get_fifo_count();
        LOG_I("final fifo deep %d\n", cnt);
    }
    else if (strcmp(argv[1], "-pedo") == 0)
    {
        int en = 1;
        if (argc >= 3)
            en = atoi(argv[2]);
        if (en == 0)
        {
            bmi270_pedo_enable(0);
            LOG_I("BMI pedometer disable\n");
        }
        else
        {
            bmi270_pedo_enable(1);
            LOG_I("BMI pedometer enable\n");
        }
    }
    else if (strcmp(argv[1], "-fstep") == 0)
    {
        bmi270_fifo_enable(3, 1);
        LOG_I("Enable pedometer to fifo mode\n");
    }
    else if (strcmp(argv[1], "-fpout") == 0)
    {
        int cnt = bmi270_get_fifo_count();
        uint8_t *buf = malloc(cnt * 2);
        int res = bmi270_read_fifo(buf, cnt * 2);
        int step = bmi270_pedo_fifo2step(buf, cnt * 2);
        LOG_I("Step: %d\n", step);
        free(buf);
    }
    else if (strcmp(argv[1], "-awen") == 0)
    {
        int en = 1;
        if (argc >= 3)
            en = atoi(argv[2]);
        if (en == 0)
        {
            bmi270_awt_enable(0);
            LOG_I("BMI awt disable\n");
        }
        else
        {
            bmi270_awt_enable(1);
            LOG_I("BMI awt enable\n");
        }
    }
    else if (strcmp(argv[1], "-acce") == 0)
    {

        int16_t x, y, z;
        if (bmi270_accel_read(&x, &y, &z) == BMI2_OK)
        {
            LOG_I("accX = %d, accY = %d, accZ = %d\n", x, y, z);
        }
        else
        {
            LOG_I("get accelerator fail\n");
        }
    }
    else if (strcmp(argv[1], "-gyro") == 0)
    {
        int16_t x, y, z;
        if (bmi270_gyro_read(&x, &y, &z) == BMI2_OK)
        {
            LOG_I("gyroX = %d, gyroY = %d, gyroZ = %d\n", x, y, z);
        }
        else
        {
            LOG_I("get gyro fail\n");
        }
    }
    else if (strcmp(argv[1], "-temp") == 0)
    {
        float tempr;
        if (bmi270_tempra_read(&tempr) == 0)
        {
            LOG_I("Temperature = %f degC\n", tempr);
        }
        else
        {
            LOG_I("get temperature fail\n");
        }
    }
        #if 0
    else if (strcmp(argv[1], "-plib") == 0) // pedometer lib-c
    {
        int res;
        int32_t accx = 0, accy = 0, accz = 0;
        int32_t loop = 50 * 60 * 5;
        int32_t logcnt = 0;
        SportDataType data = {0};
        if (argc >= 3)
            loop = 50 * 60 * atoi(argv[2]);

        Sport_Init();
        Set_Parameter(175, 80);

        do
        {
            res = bmi270_accel_read(&accx, &accy, &accz);
            if (res != 0)
                LOG_I("get accel data fail\n");
            Sport_Calculator(accx, accy, accz); // suppose parameter mg based
            logcnt++;
            if (logcnt >= 50 * 2)
            {
                logcnt = 0;
                Read_SportData(&data);
                LOG_I("Step count %d\n", data.steps);
                LOG_I("Input 0x%x, 0x%x, 0x%x\n", accx, accy, accz);
            }

            loop--;
            rt_thread_delay(20);    // continue check with 50hz
        }
        while (loop > 0);
    }
        #endif
    else if (strcmp(argv[1], "-awt") == 0)
    {
        #ifdef BMI_USING_AWT
        int32_t ret;
        bmi270_func_src2_t func_src2;
        bmi270_wrist_tilt_ia_t wrist_tilt_ia;
        bmi270_a_wrist_tilt_mask_t a_wrist_tilt_mask;
        uint8_t value;
        ret = bmi270_read_reg(&sens_cont, BMI270_FUNC_SRC2,
                              (uint8_t *)&(func_src2), 1);
        LOG_I("func_src:           %d: 0x%x\n", ret, func_src2);

        ret = bmi270_read_reg(&sens_cont, BMI270_WRIST_TILT_IA,
                              (uint8_t *)&(wrist_tilt_ia), 1);
        LOG_I(
            "wrist_tilt_id: %d, %d, %d, %d, %d, %d\n",
            wrist_tilt_ia.wrist_tilt_ia_xneg, wrist_tilt_ia.wrist_tilt_ia_xpos,
            wrist_tilt_ia.wrist_tilt_ia_yneg, wrist_tilt_ia.wrist_tilt_ia_ypos,
            wrist_tilt_ia.wrist_tilt_ia_zneg, wrist_tilt_ia.wrist_tilt_ia_zpos);

        ret = bmi270_wrist_tilt_sens_get(&sens_cont, &value);
        LOG_I("wrist_tilt_sens:    %d:  0x%x\n", ret, value);

        ret = bmi270_tilt_latency_get(&sens_cont, &value);
        LOG_I("tilt_latency:       %d:  0x%x\n", ret, value);

        ret = bmi270_tilt_threshold_get(&sens_cont, &value);
        LOG_I("tilt_threshold:     %d:  0x%x\n", ret, value);

        bmi270_tilt_src_get(&sens_cont, &a_wrist_tilt_mask);
        LOG_I("tilt_src: %d, %d, %d, %d, %d, %d\n",
              a_wrist_tilt_mask.wrist_tilt_mask_xneg,
              a_wrist_tilt_mask.wrist_tilt_mask_xpos,
              a_wrist_tilt_mask.wrist_tilt_mask_yneg,
              a_wrist_tilt_mask.wrist_tilt_mask_ypos,
              a_wrist_tilt_mask.wrist_tilt_mask_zneg,
              a_wrist_tilt_mask.wrist_tilt_mask_zpos);
        #endif /*  BMI_USING_AWT */
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(bmi270_test, __cmd_bmi270, Test hw bmi270);
    #endif // DRV_BMI270_TEST

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
    case BMI2_OK:

        /* Do nothing */
        break;

    case BMI2_W_FIFO_EMPTY:
        LOG_E("Warning [%d] : FIFO empty\r\n", rslt);
        break;
    case BMI2_W_PARTIAL_READ:
        LOG_E("Warning [%d] : FIFO partial read\r\n", rslt);
        break;
    case BMI2_E_NULL_PTR:
        LOG_E("Error [%d] : Null pointer error. It occurs when the user tries "
              "to assign value (not address) to a pointer,"
              " which has been initialized to NULL.\r\n",
              rslt);
        break;

    case BMI2_E_COM_FAIL:
        LOG_E("Error [%d] : Communication failure error. It occurs due to "
              "read/write operation failure and also due "
              "to power failure during communication\r\n",
              rslt);
        break;

    case BMI2_E_DEV_NOT_FOUND:
        LOG_E("Error [%d] : Device not found error. It occurs when the device "
              "chip id is incorrectly read\r\n",
              rslt);
        break;

    case BMI2_E_INVALID_SENSOR:
        LOG_E("Error [%d] : Invalid sensor error. It occurs when there is a "
              "mismatch in the requested feature with the "
              "available one\r\n",
              rslt);
        break;

    case BMI2_E_SELF_TEST_FAIL:
        LOG_E("Error [%d] : Self-test failed error. It occurs when the "
              "validation of accel self-test data is "
              "not satisfied\r\n",
              rslt);
        break;

    case BMI2_E_INVALID_INT_PIN:
        LOG_E("Error [%d] : Invalid interrupt pin error. It occurs when the "
              "user tries to configure interrupt pins "
              "apart from INT1 and INT2\r\n",
              rslt);
        break;

    case BMI2_E_OUT_OF_RANGE:
        LOG_E("Error [%d] : Out of range error. It occurs when the data "
              "exceeds from filtered or unfiltered data from "
              "fifo and also when the range exceeds the maximum range for "
              "accel and gyro while performing FOC\r\n",
              rslt);
        break;

    case BMI2_E_ACC_INVALID_CFG:
        LOG_E("Error [%d] : Invalid Accel configuration error. It occurs when "
              "there is an error in accel configuration"
              " register which could be one among range, BW or filter "
              "performance in reg address 0x40\r\n",
              rslt);
        break;

    case BMI2_E_GYRO_INVALID_CFG:
        LOG_E("Error [%d] : Invalid Gyro configuration error. It occurs when "
              "there is a error in gyro configuration"
              "register which could be one among range, BW or filter "
              "performance in reg address 0x42\r\n",
              rslt);
        break;

    case BMI2_E_ACC_GYR_INVALID_CFG:
        LOG_E("Error [%d] : Invalid Accel-Gyro configuration error. It occurs "
              "when there is a error in accel and gyro"
              " configuration registers which could be one among range, BW or "
              "filter performance in reg address 0x40 "
              "and 0x42\r\n",
              rslt);
        break;

    case BMI2_E_CONFIG_LOAD:
        LOG_E("Error [%d] : Configuration load error. It occurs when failure "
              "observed while loading the configuration "
              "into the sensor\r\n",
              rslt);
        break;

    case BMI2_E_INVALID_PAGE:
        LOG_E("Error [%d] : Invalid page error. It occurs due to failure in "
              "writing the correct feature configuration "
              "from selected page\r\n",
              rslt);
        break;

    case BMI2_E_SET_APS_FAIL:
        LOG_E("Error [%d] : APS failure error. It occurs due to failure in "
              "write of advance power mode configuration "
              "register\r\n",
              rslt);
        break;

    case BMI2_E_AUX_INVALID_CFG:
        LOG_E("Error [%d] : Invalid AUX configuration error. It occurs when "
              "the auxiliary interface settings are not "
              "enabled properly\r\n",
              rslt);
        break;

    case BMI2_E_AUX_BUSY:
        LOG_E("Error [%d] : AUX busy error. It occurs when the auxiliary "
              "interface buses are engaged while configuring"
              " the AUX\r\n",
              rslt);
        break;

    case BMI2_E_REMAP_ERROR:
        LOG_E("Error [%d] : Remap error. It occurs due to failure in assigning "
              "the remap axes data for all the axes "
              "after change in axis position\r\n",
              rslt);
        break;

    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
        LOG_E("Error [%d] : Gyro user gain update fail error. It occurs when "
              "the reading of user gain update status "
              "fails\r\n",
              rslt);
        break;

    case BMI2_E_SELF_TEST_NOT_DONE:
        LOG_E("Error [%d] : Self-test not done error. It occurs when the "
              "self-test process is ongoing or not "
              "completed\r\n",
              rslt);
        break;

    case BMI2_E_INVALID_INPUT:
        LOG_E("Error [%d] : Invalid input error. It occurs when the sensor "
              "input validity fails\r\n",
              rslt);
        break;

    case BMI2_E_INVALID_STATUS:
        LOG_E("Error [%d] : Invalid status error. It occurs when the "
              "feature/sensor validity fails\r\n",
              rslt);
        break;

    case BMI2_E_CRT_ERROR:
        LOG_E("Error [%d] : CRT error. It occurs when the CRT test has "
              "failed\r\n",
              rslt);
        break;

    case BMI2_E_ST_ALREADY_RUNNING:
        LOG_E("Error [%d] : Self-test already running error. It occurs when "
              "the self-test is already running and "
              "another has been initiated\r\n",
              rslt);
        break;

    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
        LOG_E("Error [%d] : CRT ready for download fail abort error. It occurs "
              "when download in CRT fails due to wrong "
              "address location\r\n",
              rslt);
        break;

    case BMI2_E_DL_ERROR:
        LOG_E("Error [%d] : Download error. It occurs when write length "
              "exceeds that of the maximum burst length\r\n",
              rslt);
        break;

    case BMI2_E_PRECON_ERROR:
        LOG_E("Error [%d] : Pre-conditional error. It occurs when precondition "
              "to start the feature was not "
              "completed\r\n",
              rslt);
        break;

    case BMI2_E_ABORT_ERROR:
        LOG_E("Error [%d] : Abort error. It occurs when the device was shaken "
              "during CRT test\r\n",
              rslt);
        break;

    case BMI2_E_WRITE_CYCLE_ONGOING:
        LOG_E("Error [%d] : Write cycle ongoing error. It occurs when the "
              "write cycle is already running and another "
              "has been initiated\r\n",
              rslt);
        break;

    case BMI2_E_ST_NOT_RUNING:
        LOG_E("Error [%d] : Self-test is not running error. It occurs when "
              "self-test running is disabled while it's "
              "running\r\n",
              rslt);
        break;

    case BMI2_E_DATA_RDY_INT_FAILED:
        LOG_E("Error [%d] : Data ready interrupt error. It occurs when the "
              "sample count exceeds the FOC sample limit "
              "and data ready status is not updated\r\n",
              rslt);
        break;

    case BMI2_E_INVALID_FOC_POSITION:
        LOG_E("Error [%d] : Invalid FOC position error. It occurs when average "
              "FOC data is obtained for the wrong"
              " axes\r\n",
              rslt);
        break;

    default:
        LOG_E("Error [%d] : Unknown error code\r\n", rslt);
        break;
    }
}
#endif /*ACC_USING_BMI270*/

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/