/*
 * Copyright (c) 2018-2024, Skaiwalk Development Team
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2024-02-20     jack         first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include "board.h"

#define DRV_DEBUG
#define DBG_TAG "utest.adc"
#include <rtdbg.h>
/*
 * Program Listing： ADC Device Usage Routines
 * The routine exports the adc_sample command to the control terminal
 * adc_sample Command call format: adc_sample
 * Program function: The voltage value is sampled by the ADC device and converted to a numerical value.
 * Vin = (R1+R2)/R2*Vref*(ADC_Value/4096)
 *                   The sample code reference voltage is 3.3V and the number of conversion bits is 12 bits.
 */
#if defined(BSP_USING_ADC) && !kReleaseMode

#define ADC_DEV_NAME "bat1"    /* ADC device name */
#define ADC_DEV_CHANNEL 5      /* ADC channel */
#define REFER_VOLTAGE 180      /* Reference voltage 1.8V, data accuracy multiplied by 100 and reserve 2 decimal places*/
#define CONVERT_BITS (1 << 10) /* The number of conversion bits is 10 */

static int skaiwalk_adc_get_mv(uint32_t value)
{
    int offset;
    float ratio;

    // get offset
    offset = 8157;
    ratio = 2366;

    return (int)((ratio / 1000.0) * value) - offset;
}

static int utest_battery_adc(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t chnl = atoi(argv[2]);
    rt_uint32_t value, res;
    rt_device_t adc_dev = rt_device_find(argv[1]);

    if (adc_dev == RT_NULL)
    {
        LOG_E("Can't find adc device %s\n", argv[1]);
        return -RT_ERROR;
    }

    ret = rt_device_open(adc_dev, RT_DEVICE_FLAG_RDONLY);
    if (ret != RT_EOK)
    {
        LOG_E("Open adc device fail\n");
        return ret;
    }

    ret = rt_device_control(adc_dev, RT_ADC_CMD_ENABLE, (void *)chnl);
    if (ret != RT_EOK)
    {
        LOG_E("Enable adc channel %d fail\n", chnl);
        return ret;
    }

    res = rt_device_read(adc_dev, chnl, &value, 1);

    LOG_I("Read ADC channel %d : %d, res = %d \n", chnl, value, res);
    int vol = skaiwalk_adc_get_mv(value);

    LOG_I("Calculate votage %d mv\n", vol);

    return ret;
}
MSH_CMD_EXPORT(utest_battery_adc, read battery adc vol);

#endif