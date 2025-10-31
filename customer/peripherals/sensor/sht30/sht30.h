#ifndef SHT30_H
#define SHT30_H

#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>

#define SHT30_ADDR 0x44
#define SHT30_MEASURE_TIME_MS 15 /* high repeatability measurement time */

rt_err_t sht30_soft_reset(void);
rt_err_t sht30_clear_status(void);
uint32_t sht30_startmeasure(void);
uint32_t sht30_getmeasureresult(float *temp, float *humi);
rt_err_t sht30_init(const char *i2c_bus_name);
rt_err_t sht30_measure(float *temp, float *humi);

#endif // SHT30_H