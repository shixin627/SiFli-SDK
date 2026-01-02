/**
 ******************************************************************************
 * @file   ft3168.c
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

#include <rtthread.h>
#include "board.h"
#include "ft3168.h"
#include "drv_touch.h"
#include <stdlib.h>
/* Define -------------------------------------------------------------------*/

#define DBG_LEVEL DBG_INFO //  DBG_LOG //
#define LOG_TAG "drv.ft3168"
#include <drv_log.h>

#define FT_DEV_ADDR (0x38)
#define FT_TD_STATUS (0x02)
#define FT_P1_XH (0x03)
#define FT_P1_XL (0x04)
#define FT_P1_YH (0x05)
#define FT_P1_YL (0x06)
#define FT_ID_G_MODE (0xA4)
#define FT_ID_G_PMODE (0xA5)
#define FT_READ_ID (0x9F)

#define FT_MAX_WIDTH (466)
#define FT_MAX_HEIGHT (466)

#define TOUCH_CHIP_ID_FT3168 (0x60)

// rotate to left with 90, 180, 270
// rotate to left with 360 for mirror
// #define FT_ROTATE_LEFT                 (90)

/* function and value-----------------------------------------------------------*/

static void ft3168_correct_pos(touch_msg_t ppos);
static rt_err_t write_reg(uint8_t reg, rt_uint8_t data);
static rt_err_t read_regs(rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf);

static struct rt_i2c_bus_device *ft_bus = NULL;

static struct touch_drivers ft3168_driver;

static rt_err_t write_reg(uint8_t reg, rt_uint8_t data)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {reg, data};

    msgs.addr = FT_DEV_ADDR; /* slave address */
    msgs.flags = RT_I2C_WR;  /* write flag */
    msgs.buf = buf;          /* Send data pointer */
    msgs.len = 2;

    if (rt_i2c_transfer(ft_bus, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_err_t read_regs(rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = FT_DEV_ADDR; /* Slave address */
    msgs[0].flags = RT_I2C_WR;  /* Write flag */
    msgs[0].buf = &reg;         /* Slave register address */
    msgs[0].len = 1;            /* Number of bytes sent */

    msgs[1].addr = FT_DEV_ADDR; /* Slave address */
    msgs[1].flags = RT_I2C_RD;  /* Read flag */
    msgs[1].buf = buf;          /* Read data pointer */
    msgs[1].len = len;          /* Number of bytes read */

    if (rt_i2c_transfer(ft_bus, msgs, 2) == 2)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static void ft3168_correct_pos(touch_msg_t ppos)
{
    ppos->x = FT_MAX_WIDTH - ppos->x;
    if (ppos->x < 0)
    {
        ppos->x = 0;
    }

    ppos->y = FT_MAX_HEIGHT - ppos->y;
    if (ppos->y < 0)
    {
        ppos->y = 0;
    }

    return;
}

/**
 * @brief Set the power mode of the FT3168 peripheral.
 *
 * This function sets the power mode of the FT3168 peripheral by writing the specified mode
 * to the FT_ID_G_PMODE register. The available power modes are:
 * - 0x00 : P_ACTIVE
 * - 0x01 : P_MONITOR
 * - 0x02 : P_STANDBY
 * - 0x03 : P_HIBERNATE
 *
 * @param mode The power mode to set. Valid values are 0x00, 0x01, 0x02, and 0x03.
 * @return rt_err_t Returns RT_EOK on success, or an error code on failure.
 */
static rt_err_t set_power_mode(uint8_t mode)
{
    if (mode > 3)
    {
        LOG_E("Invalid power mode");
        return RT_ERROR;
    }
    rt_err_t err;
    err = write_reg(FT_ID_G_PMODE, mode);
    if (err != RT_EOK)
    {
        LOG_E("Failed to set power mode");
    }
    return err;
}

static rt_err_t read_point(touch_msg_t p_msg)
{
    int res;
    unsigned char out_val[3];
    uint8_t tp_num;
    rt_err_t err;

    LOG_D("ft3168 read_point");
    rt_touch_irq_pin_enable(1);

    res = 0;
    // LOG_D("tpnum:%d",tp_num);
    if (ft_bus && p_msg)
    {
        err = read_regs(FT_TD_STATUS, 1, &tp_num);
        if (RT_EOK != err)
        {
            goto ERROR_HANDLE;
        }

        if (tp_num > 0)
        {
            // get x positon
            err = read_regs(FT_P1_XL, 1, &out_val[0]);
            if (RT_EOK != err)
            {
                LOG_D("get xL fail\n");
                res = 1;
            }
            err = read_regs(FT_P1_XH, 1, &out_val[1]);
            if (RT_EOK != err)
            {
                LOG_D("get xH fail\n");
                res = 1;
            }
            LOG_D("outx 0x%02x, 0x%02x, 0x%02x\n", out_val[0], out_val[1], out_val[2]);
            p_msg->x = ((out_val[1] & 0x7) << 8) | out_val[0];

            // get y position
            err = read_regs(FT_P1_YL, 1, &out_val[0]);
            if (RT_EOK != err)
            {
                LOG_D("get yL fail\n");
                res = 1;
            }
            err = read_regs(FT_P1_YH, 1, &out_val[1]);
            if (RT_EOK != err)
            {
                LOG_D("get yH fail\n");
                res = 1;
            }
            LOG_D("outy 0x%02x, 0x%02x, 0x%02x\n", out_val[0], out_val[1], out_val[2]);
            p_msg->y = ((out_val[1] & 0x7) << 8) | out_val[0];

            p_msg->event = TOUCH_EVENT_DOWN;
            // ft3168_correct_pos(p_msg);

            LOG_D("Down event, x = %d, y = %d\n", p_msg->x, p_msg->y);

            return (tp_num > 1) ? RT_EOK : RT_EEMPTY;
        }
        else
        {
            res = 1;
            p_msg->x = 0;
            p_msg->y = 0;
            p_msg->event = TOUCH_EVENT_UP;
            LOG_D("Up event, x = %d, y = %d\n", p_msg->x, p_msg->y);
            return RT_EEMPTY;
        }
    }
    else
    {
        // LOG_D("spi or handle error\n");
        res = 1;
    }

ERROR_HANDLE:
    p_msg->x = 0;
    p_msg->y = 0;
    p_msg->event = TOUCH_EVENT_UP;
    LOG_E("Error, return Up event, x = %d, y = %d\n", p_msg->x, p_msg->y);

    return RT_ERROR;
}

void ft3168_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    int value = (int)arg;
    LOG_D("ft3168 touch_irq_handler\n");

    rt_touch_irq_pin_enable(0);

    ret = rt_sem_release(ft3168_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}

static rt_err_t init(void)
{
    rt_err_t err;
    struct touch_message msg;

    LOG_D("ft3168 init");

    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, ft3168_irq_handler, NULL);
    rt_touch_irq_pin_enable(1); // Must enable before read I2C

    err = write_reg(FT_ID_G_MODE, 1);
    if (RT_EOK != err)
    {
        LOG_E("G_MODE set fail\n");
        // return RT_FALSE;
    }
    err = set_power_mode(0x01);

    {
        uint8_t id = 0;
        err = read_regs(FT_READ_ID, 1, &id);

        if (RT_EOK == err)
        {
            LOG_I("ft3168 id=%d", id);
        }
    }
    LOG_D("ft3168 init OK");
    return RT_EOK;
}

static rt_err_t deinit(void)
{
    LOG_D("ft3168 deinit");

    // rt_touch_irq_pin_enable(0);
    // rt_touch_irq_pin_detach();
    return RT_EOK;
}

static rt_bool_t probe(void)
{
    BSP_TP_Reset(0);
    rt_thread_mdelay(100);
    BSP_TP_Reset(1);

    ft_bus = (struct rt_i2c_bus_device *)rt_device_find(TOUCH_DEVICE_NAME);
    if (RT_Device_Class_I2CBUS != ft_bus->parent.type)
    {
        ft_bus = NULL;
    }
    if (ft_bus)
    {
        rt_device_open((rt_device_t)ft_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    }
    else
    {
        LOG_I("bus not find\n");
        return RT_FALSE;
    }

    {
        struct rt_i2c_configuration configuration =
            {
                .mode = 0,
                .addr = 0,
                .timeout = 500,
                .max_hz = 400000,
            };

        rt_i2c_configure(ft_bus, &configuration);
    }

    uint8_t id = 0;
    rt_thread_mdelay(100);
    read_regs(FT_READ_ID, 1, &id);
    int retry_count = 0;
    while (id != TOUCH_CHIP_ID_FT3168 && retry_count < 3)
    {
        LOG_I("ft3168 id=0x%x, retry_count=%d", id, retry_count);
        rt_thread_mdelay(100);
        read_regs(FT_READ_ID, 1, &id);
        retry_count++;
    }

    if (id != TOUCH_CHIP_ID_FT3168)
    {
        LOG_E("ft3168 id=0x%x not supported", id);
        rt_device_close((rt_device_t)ft_bus);
        ft_bus = NULL;
        return RT_FALSE;
    }

    LOG_I("ft3168 probe OK");

    return RT_TRUE;
}

static struct touch_ops ops =
    {
        read_point,
        init,
        deinit};

static int rt_ft3168_init(void)
{
    // rt_bool_t ret = probe();
    // if (ret == RT_FALSE)
    // {
    //     return -1;
    // }
    ft3168_driver.probe = probe;
    ft3168_driver.ops = &ops;
    ft3168_driver.user_data = RT_NULL;
    ft3168_driver.isr_sem = rt_sem_create("ft3168", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&ft3168_driver);

    return 0;
}
INIT_COMPONENT_EXPORT(rt_ft3168_init);

// #define FT3168_FUNC_TEST
#ifdef FT3168_FUNC_TEST

int cmd_ft_test(int argc, char *argv[])
{
    int func = 0;
    if (argc > 1)
    {
        func = atoi(argv[1]);
    }
    else
    {
        LOG_I("usage: ft_test func mode");
        return 0;
    }

    switch (func)
    {
    case 0:
    {
        touch_msg_t msg = {0};
        int res, looper;

        if (argc > 2)
        {
            looper = atoi(argv[2]);
        }
        else
        {
            looper = 0x0fffffff;
        }

        if (NULL == ft_bus)
        {
            init();
        }
        while (looper != 0)
        {
            res = read_point(msg);
            if (msg->event == TOUCH_EVENT_DOWN)
            {
                LOG_I("x = %d, y = %d", msg->x, msg->y);
            }

            looper--;
            rt_thread_delay(100);
        }
    }
    break;
    case 1:
    {
        if (argc > 2)
        {
            int mode = atoi(argv[2]);
            rt_touch_irq_pin_enable(1);
            set_power_mode(mode);
        }
    }
    break;
    default:
        break;
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_ft_test, __cmd_ft_test, Test hw ft3168);
#endif /* FT3168_FUNC_TEST */

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
