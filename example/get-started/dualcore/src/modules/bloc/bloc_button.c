/**
*****************************************************************************************
*     Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      bloc_button.h
* @brief     business logic of button
* @author    Jack
* @date      2024-02-21
* @version   v1.0
**************************************************************************************
* @attention
**************************************************************************************
*/
#include <rtthread.h>
#include <rtdevice.h>
#include "bloc_peripheral.h"

#define DBG_TAG "bloc_button"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef ENCODER_USING_DK05E01TF412
// #define BSP_WHEEL_SAMPLE_HZ 50
// extern rt_sem_t wheel_sem;
// static rt_sem_t wheel_api_sem;
// static void wheel_api_lock(void)
// {
// 	rt_err_t err;
// 	err = rt_sem_take(wheel_api_sem, RT_WAITING_FOREVER);
// 	RT_ASSERT(RT_EOK == err);
// }
// static void wheel_api_unlock(void)
// {
// 	rt_err_t err;
// 	err = rt_sem_release(wheel_api_sem);
// 	RT_ASSERT(RT_EOK == err);
// }

// static void wheel_task(void *parameter)
// {
// 	rt_device_t wheel_device = rt_device_find("wheel");
// 	wheel_api_sem = rt_sem_create("wheel_api_sem", 1, RT_IPC_FLAG_FIFO);
// 	if (wheel_device)
// 	{
// 		if (RT_EOK == rt_device_open(wheel_device, RT_DEVICE_FLAG_RDONLY))
// 		{
// 			int16_t wheel_data;
// 			rt_err_t err;

// 			while (1)
// 			{
// 				if (rt_sem_take(wheel_sem, RT_WAITING_FOREVER) != RT_EOK)
// 				{
// 					continue;
// 				}

// 				do
// 				{
// 					wheel_api_lock();
// 					rt_size_t res = rt_device_read(wheel_device, 0, &wheel_data, 2);
// 					wheel_api_unlock();
// 					if (res == 2 && wheel_data != 0)
// 					{
// 						rt_kprintf("wheel_data = %d\r\n", wheel_data);
// 						if (wheel_data > 0)
// 						{
// 							watch_system_interact(INTERACT_BT_SPEAKER_VOLUME_UP, NULL);
// 						}
// 						else
// 						{
// 							watch_system_interact(INTERACT_BT_SPEAKER_VOLUME_DOWN, NULL);
// 						}
// 					}
// 					rt_thread_delay(RT_TICK_PER_SECOND / BSP_WHEEL_SAMPLE_HZ); // Let system breath
// 				} while (err == RT_EOK);
// 			}
// 		}
// 	}
// }

// int init_wheel(void)
// {
// 	rt_thread_t tid;

// 	// Create the semaphore
// 	wheel_sem = rt_sem_create("wheel_sem", 1, RT_IPC_FLAG_FIFO);

// 	tid = rt_thread_create("wheel", wheel_task, NULL, 1024, 16, 10);
// 	rt_thread_startup(tid);
// 	return 0;
// }
// INIT_APP_EXPORT(init_wheel);
#endif
