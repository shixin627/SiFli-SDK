/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "drv_epictl.h"
#include "string.h"
#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //
#define LOG_TAG                "drv.epictl"
#include "log.h"

#ifdef HAL_EPICTL_ENABLED

#define EPICTL_TRANSFER_TIMEOUT_MS 1000

typedef struct
{

    EPICTL_HandleTypeDef hal_handler;
    uint8_t inited;
    struct rt_semaphore sema;
} DRV_EPICTL_TypeDef;

DRV_EPICTL_TypeDef drv_epictl;

__ROM_USED void EPICTL_IRQHandler(void)
{
    rt_interrupt_enter();

    HAL_EPICTL_IRQHandler(&drv_epictl.hal_handler);
    rt_sem_release(&drv_epictl.sema);

    rt_interrupt_leave();
}

int drv_epictl_init(void)
{
    rt_err_t err;

    if (0 == drv_epictl.inited)
    {
        memset(&drv_epictl, 0, sizeof(DRV_EPICTL_TypeDef));

        drv_epictl.hal_handler.Instance = EPIC;
        HAL_EPICTL_Init(&drv_epictl.hal_handler);

        //Create semaphore
        err = rt_sem_init(&drv_epictl.sema, "epictl", 1, RT_IPC_FLAG_FIFO);
        RT_ASSERT(RT_EOK == err);

        HAL_NVIC_SetPriority(EPIC_TL_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(EPIC_TL_IRQn);

        drv_epictl.inited = 1;
    }

    return 0;
}
INIT_PRE_APP_EXPORT(drv_epictl_init);

void drv_epictl_deinit(void)
{
    if (1 == drv_epictl.inited)
    {
        HAL_NVIC_DisableIRQ(EPIC_TL_IRQn);
        HAL_EPICTL_DeInit(&drv_epictl.hal_handler);
        rt_sem_detach(&drv_epictl.sema);
        memset(&drv_epictl, 0, sizeof(DRV_EPICTL_TypeDef));
    }
}

/**
 * @brief Initiates a data transfer using the EPIC Tranfer Layer.
 *
 * This function starts a transfer operation based on the configuration provided in the
 * EPICTL_DataType structure. Upon completion of the transfer, the specified callback
 * function is invoked.
 *
 * @param p_cfg Pointer to the EPICTL_DataType structure containing transfer configuration.
 * @param cbk Callback function to be called when the transfer is complete.
 * @return RT_EOK on success, or an error code on failure.
 */
rt_err_t drv_epictl_transfer(EPICTL_DataType *p_cfg, drv_epictl_cplt_cbk cbk)
{
    HAL_StatusTypeDef ret;
    rt_err_t err;

    RT_ASSERT(drv_epictl.inited);
    RT_ASSERT(p_cfg != NULL);

    err = rt_sem_take(&drv_epictl.sema, rt_tick_from_millisecond(EPICTL_TRANSFER_TIMEOUT_MS));
    if (err != RT_EOK)
    {
        LOG_E("drv_epictl_transfer take sema failed: %d", err);
        return err;
    }
    //Register callback
    drv_epictl.hal_handler.XferCpltCallback = cbk;
    ret = HAL_EPICTL_Start_IT(&drv_epictl.hal_handler, p_cfg);
    if (HAL_OK == ret)
    {
        return RT_EOK;
    }
    else
    {
        LOG_E("drv_epictl_transfer error: %d, errcode=%d", ret, drv_epictl.hal_handler.ErrorCode);
        //Release semaphore in case of error
        rt_sem_release(&drv_epictl.sema);
        return (rt_err_t)ret;
    }
}
#endif /*HAL_EPICTL_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/