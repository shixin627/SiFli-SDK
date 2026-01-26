/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPL06_H
#define __SPL06_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SPL06_RET_OK   0
#define SPL06_RET_NG   1

/*
 *  SPL06 I2C address
 */
#define SPL06_AD0_HIGH     0x77 // SDO pin high (default)
#define SPL06_AD0_LOW      0x76 // SDO pin low (GND)

/*
 *  SPL06 chip id
 */
#define SPL06_CHIP_ID          (0x10)

/*
 *  SPL06 register address
 */
#define SPL06_REG_PSR_B2        0x00    /* Pressure MSB */
#define SPL06_REG_PSR_B1        0x01    /* Pressure middle byte */
#define SPL06_REG_PSR_B0        0x02    /* Pressure LSB */
#define SPL06_REG_TMP_B2        0x03    /* Temperature MSB */
#define SPL06_REG_TMP_B1        0x04    /* Temperature middle byte */
#define SPL06_REG_TMP_B0        0x05    /* Temperature LSB */
#define SPL06_REG_PRS_CFG       0x06    /* Pressure configuration */
#define SPL06_REG_TMP_CFG       0x07    /* Temperature configuration */
#define SPL06_REG_MEAS_CFG      0x08    /* Measurement configuration */
#define SPL06_REG_CFG_REG       0x09    /* Interrupt and FIFO configuration */
#define SPL06_REG_INT_STS       0x0A    /* Interrupt status */
#define SPL06_REG_FIFO_STS      0x0B    /* FIFO status */
#define SPL06_REG_RESET         0x0C    /* Soft reset and FIFO flush */
#define SPL06_REG_ID            0x0D    /* Product and revision ID */

/* Calibration coefficients registers */
#define SPL06_REG_COEF_C0       0x10
#define SPL06_REG_COEF_C0C1     0x11
#define SPL06_REG_COEF_C1       0x12
#define SPL06_REG_COEF_C00_H    0x13
#define SPL06_REG_COEF_C00_M    0x14
#define SPL06_REG_COEF_C00_C10  0x15
#define SPL06_REG_COEF_C10_H    0x16
#define SPL06_REG_COEF_C10_L    0x17
#define SPL06_REG_COEF_C01_H    0x18
#define SPL06_REG_COEF_C01_L    0x19
#define SPL06_REG_COEF_C11_H    0x1A
#define SPL06_REG_COEF_C11_L    0x1B
#define SPL06_REG_COEF_C20_H    0x1C
#define SPL06_REG_COEF_C20_L    0x1D
#define SPL06_REG_COEF_C21_H    0x1E
#define SPL06_REG_COEF_C21_L    0x1F
#define SPL06_REG_COEF_C30_H    0x20
#define SPL06_REG_COEF_C30_L    0x21

/* MEAS_CFG register bits */
#define SPL06_MEAS_CFG_COEF_RDY     (1 << 7)
#define SPL06_MEAS_CFG_SENSOR_RDY   (1 << 6)
#define SPL06_MEAS_CFG_TMP_RDY      (1 << 5)
#define SPL06_MEAS_CFG_PRS_RDY      (1 << 4)

/* MEAS_CTRL measurement modes */
#define SPL06_MEAS_CTRL_IDLE                0x00
#define SPL06_MEAS_CTRL_PRS_SINGLE          0x01
#define SPL06_MEAS_CTRL_TMP_SINGLE          0x02
#define SPL06_MEAS_CTRL_PRS_CONTINUOUS      0x05
#define SPL06_MEAS_CTRL_TMP_CONTINUOUS      0x06
#define SPL06_MEAS_CTRL_PRS_TMP_CONTINUOUS  0x07

/* CFG_REG register bits */
#define SPL06_CFG_T_SHIFT       (1 << 3)
#define SPL06_CFG_P_SHIFT       (1 << 2)
#define SPL06_CFG_FIFO_EN       (1 << 1)

/* Pressure measurement rate (PM_RATE) */
#define SPL06_PM_RATE_1         (0 << 4)
#define SPL06_PM_RATE_2         (1 << 4)
#define SPL06_PM_RATE_4         (2 << 4)
#define SPL06_PM_RATE_8         (3 << 4)
#define SPL06_PM_RATE_16        (4 << 4)
#define SPL06_PM_RATE_32        (5 << 4)
#define SPL06_PM_RATE_64        (6 << 4)
#define SPL06_PM_RATE_128       (7 << 4)

/* Pressure oversampling rate (PM_PRC) */
#define SPL06_PM_PRC_1          0x00    /* Single */
#define SPL06_PM_PRC_2          0x01    /* 2 times (Low Power) */
#define SPL06_PM_PRC_4          0x02    /* 4 times */
#define SPL06_PM_PRC_8          0x03    /* 8 times */
#define SPL06_PM_PRC_16         0x04    /* 16 times (Standard) */
#define SPL06_PM_PRC_32         0x05    /* 32 times */
#define SPL06_PM_PRC_64         0x06    /* 64 times (High Precision) */
#define SPL06_PM_PRC_128        0x07    /* 128 times */

/* Temperature measurement rate (TMP_RATE) */
#define SPL06_TMP_RATE_1        (0 << 4)
#define SPL06_TMP_RATE_2        (1 << 4)
#define SPL06_TMP_RATE_4        (2 << 4)
#define SPL06_TMP_RATE_8        (3 << 4)
#define SPL06_TMP_RATE_16       (4 << 4)
#define SPL06_TMP_RATE_32       (5 << 4)
#define SPL06_TMP_RATE_64       (6 << 4)
#define SPL06_TMP_RATE_128      (7 << 4)

/* Temperature oversampling rate (TMP_PRC) */
#define SPL06_TMP_PRC_1         0x00    /* Single */
#define SPL06_TMP_PRC_2         0x01    /* 2 times */
#define SPL06_TMP_PRC_4         0x02    /* 4 times */
#define SPL06_TMP_PRC_8         0x03    /* 8 times */
#define SPL06_TMP_PRC_16        0x04    /* 16 times */
#define SPL06_TMP_PRC_32        0x05    /* 32 times */
#define SPL06_TMP_PRC_64        0x06    /* 64 times */
#define SPL06_TMP_PRC_128       0x07    /* 128 times */

/* TMP_CFG TMP_EXT bit - use external sensor (MEMS) */
#define SPL06_TMP_EXT           (1 << 7)

/* RESET register */
#define SPL06_RESET_FIFO_FLUSH  (1 << 7)
#define SPL06_RESET_SOFT_RST    0x09

/* Compensation scale factors */
#define SPL06_SCALE_FACTOR_1        524288
#define SPL06_SCALE_FACTOR_2        1572864
#define SPL06_SCALE_FACTOR_4        3670016
#define SPL06_SCALE_FACTOR_8        7864320
#define SPL06_SCALE_FACTOR_16       253952
#define SPL06_SCALE_FACTOR_32       516096
#define SPL06_SCALE_FACTOR_64       1040384
#define SPL06_SCALE_FACTOR_128      2088960

typedef struct
{
    int16_t c0;         /* 12 bit calibration coefficient */
    int16_t c1;         /* 12 bit calibration coefficient */
    int32_t c00;        /* 20 bit calibration coefficient */
    int32_t c10;        /* 20 bit calibration coefficient */
    int16_t c01;        /* 16 bit calibration coefficient */
    int16_t c11;        /* 16 bit calibration coefficient */
    int16_t c20;        /* 16 bit calibration coefficient */
    int16_t c21;        /* 16 bit calibration coefficient */
    int16_t c30;        /* 16 bit calibration coefficient */
    int32_t kT;         /* Temperature scale factor */
    int32_t kP;         /* Pressure scale factor */
} SPL06_HandleTypeDef;

typedef struct
{
    uint8_t Index;
    int32_t AvgBuffer[8];
} SPL06_AvgTypeDef;

#define MSLP     101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)

/*
 *  extern interface
 */
extern uint8_t SPL06_Init(void);
extern void SPL06_CalTemperatureAndPressureAndAltitude(int32_t *temperature, int32_t *pressure, int32_t *Altitude);
void *SPL06GetBus(void);
uint8_t SPL06GetDevAddr(void);
uint8_t SPL06GetDevId(void);

void SPL06_open(void);
void SPL06_close(void);

int SPL06_SelfCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPL06_H */
