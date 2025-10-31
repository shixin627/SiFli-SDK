#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include "sensor.h"
#include "sht30.h"
#include "sensor_sht30.h"

/* Default I2C bus name */
#ifndef SHT30_I2C_BUS
    #define SHT30_I2C_BUS "i2c2"
#endif

#ifndef SHT30_MODEL_NAME
    #define SHT30_MODEL_NAME "sht30"
#endif

static rt_size_t sht30_fetch_data(struct rt_sensor_device *sensor, void *buf,
                                  rt_size_t len)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    float temp = 0, humi = 0;

    /* Start measurement and read result */
    if (sht30_measure(&temp, &humi) != RT_EOK)
    {
        LOG_E("SHT30 measure failed");
        return 0;
    }

    /* Return both temperature and humidity data */
    data->type = RT_SENSOR_CLASS_TEMP;
    data->data.temp = (rt_int32_t)(temp * 10); /* deci-degree */
    data->timestamp = rt_sensor_get_ts();

    if (len >= 2)
    {
        data++;
        data->type = RT_SENSOR_CLASS_HUMI;
        data->data.humi = (rt_int32_t)(humi * 10); /* deci-percent */
        data->timestamp = rt_sensor_get_ts();
        return 2;
    }

    return 1;
}

int rt_hw_sht30_init(const char *name, struct rt_sensor_config *cfg)
{
    int result = -RT_ERROR;
    rt_sensor_t sensor = RT_NULL;

    rt_kprintf("[DBG] SHT30 hw_init: name=%s, i2c=%s\n", name,
               cfg->intf.dev_name);

    if (sht30_init(cfg->intf.dev_name) != RT_EOK)
    {
        LOG_E("SHT30 init failed");
        return -RT_ERROR;
    }

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (RT_NULL == sensor)
    {
        LOG_E("calloc failed");
        return -RT_ERROR;
    }

    sensor->info.type = RT_SENSOR_CLASS_TEMP;
    sensor->info.vendor = RT_SENSOR_VENDOR_UNKNOWN;
    sensor->info.model = SHT30_MODEL_NAME;
    sensor->info.unit = RT_SENSOR_UNIT_DCELSIUS;
    sensor->info.intf_type = RT_SENSOR_INTF_I2C;
    sensor->info.range_max = 1250; /* 125.0 °C *10 */
    sensor->info.range_min = -400; /* -40.0 °C *10 */
    sensor->info.period_min = 20; /* minimum 20ms (15ms measure + 5ms margin) */

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }
    else
    {
        rt_kprintf("[DBG] SHT30 device registered successfully as: %s\n", name);
        LOG_I("SHT30 sensor init success");
        return RT_EOK;
    }
}

static int rt_sht30_auto_register(void)
{
    struct rt_sensor_config cfg = {0};

    rt_kprintf("[DBG] SHT30 auto register starting...\n");

    /* Set the sensor configuration parameters */
    cfg.intf.dev_name = SHT30_I2C_BUS;
    cfg.intf.user_data = (void *)((uintptr_t)SHT30_ADDR);

    int result = rt_hw_sht30_init(SHT30_MODEL_NAME, &cfg);

    if (result == RT_EOK)
    {
        rt_kprintf("[DBG] SHT30 auto register SUCCESS\n");
    }
    else
    {
        rt_kprintf("[DBG] SHT30 auto register FAILED: %d\n", result);
    }

    return result;
}
INIT_DEVICE_EXPORT(rt_sht30_auto_register);
