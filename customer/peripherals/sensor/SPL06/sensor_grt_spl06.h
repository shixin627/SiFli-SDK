/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSOR_GRT_SPL06_H__
#define SENSOR_GRT_SPL06_H__


#include "board.h"
#include "sensor.h"

#include "SPL06.h"



#ifdef __cplusplus
extern "C" {
#endif

/* spl06 config structure */
struct spl06_config
{
    rt_uint16_t prs_rate;
    rt_uint16_t prs_prc;
    rt_uint16_t tmp_rate;
    rt_uint16_t tmp_prc;
};

/* spl06 device structure */
struct spl06_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    struct spl06_config config;
};

int rt_hw_spl06_init(const char *name, struct rt_sensor_config *cfg);

#ifdef __cplusplus
}
#endif
#endif  // SENSOR_GRT_SPL06_H__
