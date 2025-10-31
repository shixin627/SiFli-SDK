#include "sht30.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdio.h>
#include "bf0_hal.h"
#include "drv_io.h"

#define SHT30_ADDR 0x44

#define SHT30_CMD_MEAS_HIGHREP_STRETCH_OFF_MSB 0x2C
#define SHT30_CMD_MEAS_HIGHREP_STRETCH_OFF_LSB 0x06

#define SHT30_CMD_SOFT_RESET_MSB 0x30
#define SHT30_CMD_SOFT_RESET_LSB 0xA2

#define SHT30_CMD_READ_STATUS_MSB 0xF3
#define SHT30_CMD_READ_STATUS_LSB 0x2D

#define SHT30_CMD_CLEAR_STATUS_MSB 0x30
#define SHT30_CMD_CLEAR_STATUS_LSB 0x41

#define SHT30_MEASURE_TIME_MS 15 /* high repeatability typical up to 15 ms */
#define SHT30_STARTUP_TIME_MS  2 /* datasheet tPU typical 0.5 - 1 ms, use 2ms */
#define SHT30_RESET_TIME_MS 2 /* soft reset recovery time ~0.5-1ms, keep 2ms */
#define SHT30_STATUS_READ_TIME_MS 1
#define SHT30_MAX_RETRY 5

static struct rt_i2c_bus_device *sht30_i2c_bus = RT_NULL;

/* CRC8 polynomial as datasheet: 0x31, init 0xFF */
static uint8_t sht30_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

static rt_err_t sht30_write_cmd16(uint16_t cmd)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)((cmd >> 8) & 0xFF);
    buf[1] = (uint8_t)(cmd & 0xFF);

    struct rt_i2c_msg msg;
    msg.addr = SHT30_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf = buf;
    msg.len = 2;

    rt_size_t result = rt_i2c_transfer(sht30_i2c_bus, &msg, 1);

    rt_kprintf("I2C transfer result: %d\n", result);

    if (result != 1)
    {
        rt_kprintf("SHT30: I2C write failed, expected 1, got %d\n", result);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t sht30_read_bytes(uint8_t *buf, rt_size_t len)
{
    struct rt_i2c_msg msg;
    msg.addr = SHT30_ADDR;
    msg.flags = RT_I2C_RD;
    msg.buf = buf;
    msg.len = len;
    return rt_i2c_transfer(sht30_i2c_bus, &msg, 1) == 1 ? RT_EOK : -RT_ERROR;
}

rt_err_t sht30_soft_reset(void)
{
    uint16_t cmd = (SHT30_CMD_SOFT_RESET_MSB << 8) |
                   SHT30_CMD_SOFT_RESET_LSB;

    rt_kprintf("SHT30 soft reset cmd: 0x%04X (decimal: %u, MSB: 0x%02X, LSB: 0x%02X)\n",
               cmd, cmd, SHT30_CMD_SOFT_RESET_MSB, SHT30_CMD_SOFT_RESET_LSB);

    rt_err_t result = sht30_write_cmd16(cmd);

    rt_thread_mdelay(SHT30_RESET_TIME_MS);

    if (result != RT_EOK)
    {
        rt_kprintf("SHT30: Soft reset command failed to send.\n");
    }

    return result;
}

rt_err_t sht30_clear_status(void)
{
    uint16_t cmd = (SHT30_CMD_CLEAR_STATUS_MSB << 8) |
                   SHT30_CMD_CLEAR_STATUS_LSB;

    rt_kprintf("SHT30 clear status cmd: 0x%04X\n", cmd);

    rt_err_t result = sht30_write_cmd16(cmd);

    rt_thread_mdelay(1);

    if (result != RT_EOK)
    {
        rt_kprintf("SHT30: Clear status command failed to send.\n");
    }

    return result;
}

rt_err_t sht30_init(const char *i2c_bus_name)
{
    sht30_i2c_bus =
        (struct rt_i2c_bus_device *)rt_i2c_bus_device_find(i2c_bus_name);
    if (!sht30_i2c_bus)
    {
        rt_kprintf("SHT30: can't find i2c bus %s\n", i2c_bus_name);
        return -RT_ERROR;
    }

    rt_thread_mdelay(SHT30_STARTUP_TIME_MS);

    return RT_EOK;
}

/* Start single-shot measurement (high repeatability, clock stretching enabled:0x2C06) */
uint32_t sht30_startmeasure(void)
{
    uint16_t cmd = (SHT30_CMD_MEAS_HIGHREP_STRETCH_OFF_MSB << 8) |
                   SHT30_CMD_MEAS_HIGHREP_STRETCH_OFF_LSB;
    if (sht30_write_cmd16(cmd) != RT_EOK)
        return (uint32_t) -1;
    return 0;
}

/* Blocking convenience API: start + wait + read */
rt_err_t sht30_measure(float *temp, float *humi)
{
    if (sht30_startmeasure() != 0)
        return -RT_ERROR;

    rt_thread_mdelay(SHT30_MEASURE_TIME_MS);

    if (sht30_getmeasureresult(temp, humi) != 0)
        return -RT_ERROR;

    return RT_EOK;
}

/* Read measurement result:
   Sensor returns: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC */
uint32_t sht30_getmeasureresult(float *temp, float *humi)
{
    if (temp == NULL || humi == NULL)
        return (uint32_t) -1;

    uint8_t buf[6];
    memset(buf, 0, sizeof(buf));

    if (sht30_read_bytes(buf, sizeof(buf)) != RT_EOK)
        return (uint32_t) -1;

    /* Validate CRCs */
    if (sht30_crc8(buf, 2) != buf[2])
        return (uint32_t) -1;
    if (sht30_crc8(&buf[3], 2) != buf[5])
        return (uint32_t) -1;

    uint16_t rawT = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t rawRH = ((uint16_t)buf[3] << 8) | buf[4];

    /* Convert per datasheet:
       T [°C] = -45 + 175 * ST/65535
       RH [%] = 100 * SRH/65535
    */
    *temp = -45.0f + 175.0f * ((float)rawT / 65535.0f);
    *humi = 100.0f * ((float)rawRH / 65535.0f);

    return 0;
}