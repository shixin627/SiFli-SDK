/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SPL06.h"
#include <rtthread.h>
#include <math.h>
#include "stdlib.h"
#include "board.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.spl06"
#include <drv_log.h>

#ifdef SENSOR_USING_SPL06

SPL06_HandleTypeDef spl06;
int32_t gs32Pressure0_spl06 = MSLP;
static struct rt_i2c_bus_device *i2cbus = NULL;
uint8_t spl06_dev_addr = SPL06_AD0_HIGH;
static int32_t press_offset = 0;

static void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[1];
    uint8_t value[2];
    uint32_t res;

    if (i2cbus)
    {
        value[0] = RegAddr;
        value[1] = Data;

        msgs[0].addr  = DevAddr;
        msgs[0].flags = RT_I2C_WR;
        msgs[0].buf   = value;
        msgs[0].len   = 2;

        res = rt_i2c_transfer(i2cbus, msgs, 1);
        if (res != 1)
        {
            LOG_D("I2C_WriteOneByte FAIL %d\n", res);
        }
    }
#endif
}

void SPL06_ReadReg(uint8_t RegAddr, uint8_t Num, uint8_t *pBuffer)
{
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
    uint32_t res;

    if (i2cbus)
    {
        msgs[0].addr  = spl06_dev_addr;
        msgs[0].flags = RT_I2C_WR;
        msgs[0].buf   = &RegAddr;
        msgs[0].len   = 1;

        msgs[1].addr  = spl06_dev_addr;
        msgs[1].flags = RT_I2C_RD;
        msgs[1].buf   = pBuffer;
        msgs[1].len   = Num;

        res = rt_i2c_transfer(i2cbus, msgs, 2);
        if (res != 2)
        {
            LOG_D("SPL06_ReadReg FAIL %d\n", res);
        }
    }
#endif
}

void SPL06_WriteReg(uint8_t RegAddr, uint8_t Val)
{
    I2C_WriteOneByte(spl06_dev_addr, RegAddr, Val);
}

static int32_t SPL06_GetScaleFactor(uint8_t oversampling)
{
    int32_t scale_factor;

    switch (oversampling)
    {
    case 0: /* 1 time */
        scale_factor = SPL06_SCALE_FACTOR_1;
        break;
    case 1: /* 2 times */
        scale_factor = SPL06_SCALE_FACTOR_2;
        break;
    case 2: /* 4 times */
        scale_factor = SPL06_SCALE_FACTOR_4;
        break;
    case 3: /* 8 times */
        scale_factor = SPL06_SCALE_FACTOR_8;
        break;
    case 4: /* 16 times */
        scale_factor = SPL06_SCALE_FACTOR_16;
        break;
    case 5: /* 32 times */
        scale_factor = SPL06_SCALE_FACTOR_32;
        break;
    case 6: /* 64 times */
        scale_factor = SPL06_SCALE_FACTOR_64;
        break;
    case 7: /* 128 times */
        scale_factor = SPL06_SCALE_FACTOR_128;
        break;
    default:
        scale_factor = SPL06_SCALE_FACTOR_1;
        break;
    }

    return scale_factor;
}

void SPL06_Read_Calibration(void)
{
    uint8_t coef[18];

    /* Read all calibration coefficients at once */
    SPL06_ReadReg(SPL06_REG_COEF_C0, 18, coef);

    /* c0: 12 bit signed - stored in coef[0] and upper 4 bits of coef[1] */
    spl06.c0 = (int16_t)(((uint16_t)coef[0] << 4) | ((coef[1] >> 4) & 0x0F));
    if (spl06.c0 & 0x0800)
        spl06.c0 |= 0xF000; /* Sign extend */

    /* c1: 12 bit signed - stored in lower 4 bits of coef[1] and coef[2] */
    spl06.c1 = (int16_t)(((uint16_t)(coef[1] & 0x0F) << 8) | coef[2]);
    if (spl06.c1 & 0x0800)
        spl06.c1 |= 0xF000; /* Sign extend */

    /* c00: 20 bit signed - stored in coef[3], coef[4] and upper 4 bits of coef[5] */
    spl06.c00 = (int32_t)(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | ((coef[5] >> 4) & 0x0F));
    if (spl06.c00 & 0x80000)
        spl06.c00 |= 0xFFF00000; /* Sign extend */

    /* c10: 20 bit signed - stored in lower 4 bits of coef[5], coef[6] and coef[7] */
    spl06.c10 = (int32_t)(((uint32_t)(coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | coef[7]);
    if (spl06.c10 & 0x80000)
        spl06.c10 |= 0xFFF00000; /* Sign extend */

    /* c01: 16 bit signed */
    spl06.c01 = (int16_t)(((uint16_t)coef[8] << 8) | coef[9]);

    /* c11: 16 bit signed */
    spl06.c11 = (int16_t)(((uint16_t)coef[10] << 8) | coef[11]);

    /* c20: 16 bit signed */
    spl06.c20 = (int16_t)(((uint16_t)coef[12] << 8) | coef[13]);

    /* c21: 16 bit signed */
    spl06.c21 = (int16_t)(((uint16_t)coef[14] << 8) | coef[15]);

    /* c30: 16 bit signed */
    spl06.c30 = (int16_t)(((uint16_t)coef[16] << 8) | coef[17]);

    LOG_D("SPL06 Calibration: c0=%d c1=%d c00=%d c10=%d\n", spl06.c0, spl06.c1, spl06.c00, spl06.c10);
    LOG_D("SPL06 Calibration: c01=%d c11=%d c20=%d c21=%d c30=%d\n", spl06.c01, spl06.c11, spl06.c20, spl06.c21, spl06.c30);
}

/* Calculate compensated temperature in degrees Celsius */
double SPL06_Compensate_Temperature(int32_t adc_T)
{
    double Traw_sc;
    double temperature;

    Traw_sc = (double)adc_T / (double)spl06.kT;
    temperature = (double)spl06.c0 * 0.5 + (double)spl06.c1 * Traw_sc;

    return temperature;
}

/* Calculate compensated pressure in Pa */
double SPL06_Compensate_Pressure(int32_t adc_P, int32_t adc_T)
{
    double Traw_sc, Praw_sc;
    double pressure;

    Traw_sc = (double)adc_T / (double)spl06.kT;
    Praw_sc = (double)adc_P / (double)spl06.kP;

    pressure = (double)spl06.c00 +
               Praw_sc * ((double)spl06.c10 + Praw_sc * ((double)spl06.c20 + Praw_sc * (double)spl06.c30)) +
               Traw_sc * (double)spl06.c01 +
               Traw_sc * Praw_sc * ((double)spl06.c11 + Praw_sc * (double)spl06.c21);

    return pressure;
}

void SPL06_Get_Temperature_And_Pressure(double *temperature, double *pressure)
{
    uint8_t data[6];
    int32_t adc_P, adc_T;

    /* Read pressure and temperature data */
    SPL06_ReadReg(SPL06_REG_PSR_B2, 6, data);

    /* Pressure: 24 bit 2's complement */
    adc_P = (int32_t)(((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]);
    if (adc_P & 0x800000)
        adc_P |= 0xFF000000; /* Sign extend */

    /* Temperature: 24 bit 2's complement */
    adc_T = (int32_t)(((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5]);
    if (adc_T & 0x800000)
        adc_T |= 0xFF000000; /* Sign extend */

    *temperature = SPL06_Compensate_Temperature(adc_T);
    *pressure = SPL06_Compensate_Pressure(adc_P, adc_T);
}

void SPL06_CalAvgValue(uint8_t *pIndex, int32_t *pAvgBuffer, int32_t InVal, int32_t *pOutVal)
{
    uint8_t i;

    *(pAvgBuffer + ((*pIndex)++)) = InVal;
    *pIndex &= 0x07;

    *pOutVal = 0;
    for (i = 0; i < 8; i++)
    {
        *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}

void SPL06_CalculateAbsoluteAltitude(int32_t *pAltitude, int32_t PressureVal)
{
    float value = pow((PressureVal / (float)gs32Pressure0_spl06), 0.1903);
    *pAltitude = 4430000 * (1 - value);
}

extern uint8_t SPL06_Init()
{
    uint8_t u8Ret = SPL06_RET_OK;
    uint8_t u8ChipID;
    uint8_t u8MeasCfg;
    uint16_t timeout = 0;

    /* Get I2C bus device */
    i2cbus = rt_i2c_bus_device_find(SPL06_I2C_BUS);
    if (i2cbus)
    {
        LOG_D("Find i2c bus device %s\n", SPL06_I2C_BUS);
        rt_i2c_open(i2cbus, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, SPL06_Init fail\n", SPL06_I2C_BUS);
        return SPL06_RET_NG;
    }

    /* Check SPL06 slave address */
    spl06_dev_addr = SPL06_AD0_HIGH;
    u8ChipID = 0;
    SPL06_ReadReg(SPL06_REG_ID, 1, &u8ChipID);
    if (u8ChipID != SPL06_CHIP_ID)
    {
        spl06_dev_addr = SPL06_AD0_LOW;
        u8ChipID = 0;
        SPL06_ReadReg(SPL06_REG_ID, 1, &u8ChipID);
    }

    if (u8ChipID == SPL06_CHIP_ID)
    {
        LOG_D("SPL06 I2C slave address = 0x%x, Chip ID = 0x%x\n", spl06_dev_addr, u8ChipID);
    }
    else
    {
        LOG_E("SPL06 init fail, ChipID = 0x%x\n", u8ChipID);
        return SPL06_RET_NG;
    }

    /* Wait for sensor to be ready */
    do
    {
        SPL06_ReadReg(SPL06_REG_MEAS_CFG, 1, &u8MeasCfg);
        rt_thread_delay(1);
        timeout++;
    } while (((u8MeasCfg & SPL06_MEAS_CFG_SENSOR_RDY) == 0) && (timeout < 100));

    if (timeout >= 100)
    {
        LOG_E("SPL06 sensor not ready\n");
        return SPL06_RET_NG;
    }

    /* Wait for coefficients to be ready */
    timeout = 0;
    do
    {
        SPL06_ReadReg(SPL06_REG_MEAS_CFG, 1, &u8MeasCfg);
        rt_thread_delay(1);
        timeout++;
    } while (((u8MeasCfg & SPL06_MEAS_CFG_COEF_RDY) == 0) && (timeout < 100));

    if (timeout >= 100)
    {
        LOG_E("SPL06 coefficients not ready\n");
        return SPL06_RET_NG;
    }

    LOG_D("SPL06 init successful\n");
    return u8Ret;
}

void SPL06_open(void)
{
    uint8_t prs_cfg, tmp_cfg;

    /* Configure pressure: 16 times oversampling, 1 measurement per second */
    prs_cfg = SPL06_PM_RATE_1 | SPL06_PM_PRC_16;
    SPL06_WriteReg(SPL06_REG_PRS_CFG, prs_cfg);
    spl06.kP = SPL06_GetScaleFactor(SPL06_PM_PRC_16);

    /* Configure temperature: external sensor, 1 measurement per second, single */
    tmp_cfg = SPL06_TMP_EXT | SPL06_TMP_RATE_1 | SPL06_TMP_PRC_1;
    SPL06_WriteReg(SPL06_REG_TMP_CFG, tmp_cfg);
    spl06.kT = SPL06_GetScaleFactor(SPL06_TMP_PRC_1);

    /* Enable pressure result shift (required for oversampling > 8) */
    SPL06_WriteReg(SPL06_REG_CFG_REG, SPL06_CFG_P_SHIFT);

    /* Read calibration coefficients */
    SPL06_Read_Calibration();

    /* Start continuous pressure and temperature measurement */
    SPL06_WriteReg(SPL06_REG_MEAS_CFG, SPL06_MEAS_CTRL_PRS_TMP_CONTINUOUS);

    rt_thread_delay(50); /* Wait for first measurement */
}

void SPL06_close(void)
{
    /* Set to idle/standby mode */
    SPL06_WriteReg(SPL06_REG_MEAS_CFG, SPL06_MEAS_CTRL_IDLE);
}

extern void SPL06_CalTemperatureAndPressureAndAltitude(int32_t *temperature, int32_t *pressure, int32_t *Altitude)
{
    double CurPressure, CurTemperature;
    int32_t CurAltitude;
    static SPL06_AvgTypeDef SPL06_Filter[3];

    SPL06_Get_Temperature_And_Pressure(&CurTemperature, &CurPressure);
    SPL06_CalAvgValue(&SPL06_Filter[0].Index, SPL06_Filter[0].AvgBuffer, (int32_t)(CurPressure), pressure);

    /* Add offset to correct test */
    *pressure -= press_offset;
    SPL06_CalculateAbsoluteAltitude(&CurAltitude, (*pressure));
    SPL06_CalAvgValue(&SPL06_Filter[1].Index, SPL06_Filter[1].AvgBuffer, CurAltitude, Altitude);
    SPL06_CalAvgValue(&SPL06_Filter[2].Index, SPL06_Filter[2].AvgBuffer, (int32_t)CurTemperature * 10, temperature);

    return;
}

void *SPL06GetBus(void)
{
    return (void *)i2cbus;
}

uint8_t SPL06GetDevAddr(void)
{
    return spl06_dev_addr;
}

uint8_t SPL06GetDevId(void)
{
    return SPL06_CHIP_ID;
}

int SPL06_SelfCheck(void)
{
    uint8_t u8ChipID = 0;

    SPL06_ReadReg(SPL06_REG_ID, 1, &u8ChipID);
    if (u8ChipID != SPL06_CHIP_ID)
        return -1;

    return 0;
}

#define DRV_SPL06_TEST

#ifdef DRV_SPL06_TEST
#include <string.h>

int cmd_spl06t(int argc, char *argv[])
{
    int32_t temp, pres, alti;
    if (argc < 2)
    {
        LOG_I("Invalid parameter!\n");
        LOG_I("Usage: spl06t -open/-close/-r <reg>/-tpa/-l <loop>\n");
        return 1;
    }
    if (strcmp(argv[1], "-open") == 0)
    {
        uint8_t res = SPL06_Init();
        if (SPL06_RET_OK == res)
        {
            SPL06_open();
            LOG_I("Open SPL06 success\n");
        }
        else
            LOG_I("Open SPL06 fail\n");
    }
    if (strcmp(argv[1], "-close") == 0)
    {
        SPL06_close();
        LOG_I("SPL06 closed\n");
    }
    if (strcmp(argv[1], "-r") == 0)
    {
        uint8_t rega = atoi(argv[2]) & 0xff;
        uint8_t value;
        SPL06_ReadReg(rega, 1, &value);
        LOG_I("Reg 0x%x value 0x%x\n", rega, value);
    }
    if (strcmp(argv[1], "-tpa") == 0)
    {
        temp = 0;
        pres = 0;
        alti = 0;
        SPL06_CalTemperatureAndPressureAndAltitude(&temp, &pres, &alti);
        LOG_I("Get temperature = %.1f C\n", (float)temp / 10);
        LOG_I("Get pressure = %.2f hPa\n", (float)pres / 100);
        LOG_I("Get altitude = %.2f m\n", (float)alti / 100);
    }
    if (strcmp(argv[1], "-bps") == 0)
    {
        struct rt_i2c_configuration cfg;
        int bps = atoi(argv[2]);
        cfg.addr = 0;
        cfg.max_hz = bps;
        cfg.mode = 0;
        cfg.timeout = 5000;
        rt_i2c_configure(i2cbus, &cfg);
        LOG_I("Config SPL06 I2C speed to %d\n", bps);
    }
    if (strcmp(argv[1], "-l") == 0)
    {
        temp = 0;
        pres = 0;
        alti = 0;
        uint32_t loop = atoi(argv[2]);
        int32_t prev1 = 0, prev2 = 0, prev3 = 0;
        if (loop > 36000)
            loop = 36000;

        do
        {
            SPL06_CalTemperatureAndPressureAndAltitude(&temp, &pres, &alti);
            if (prev1 != temp || prev2 != pres || prev3 != alti)
            {
                prev1 = temp;
                prev2 = pres;
                prev3 = alti;
                LOG_I("Get temperature = %.1f C\n", (float)temp / 10);
                LOG_I("Get pressure = %.2f hPa\n", (float)pres / 100);
                LOG_I("Get altitude = %.2f m\n", (float)alti / 100);
            }
            loop--;
            rt_thread_delay(100);
        } while (loop > 0);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_spl06t, __cmd_spl06t, Test driver SPL06);

#endif /* DRV_SPL06_TEST */

#endif  /* SENSOR_USING_SPL06 */
