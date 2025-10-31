#ifndef SENSOR_SHT30_H
#define SENSOR_SHT30_H

#include "sht30.h"
#include "sensor.h"
#include <rtthread.h>
#include <rtdbg.h>

int rt_hw_sht30_init(const char *name, struct rt_sensor_config *cfg);

#endif
