/**
  ******************************************************************************
  * @file   drv_epictl.h
  * @author Sifli software development team
  * @brief  Header for EPIC Transfer Layer (EPICTL) BSP driver
  ******************************************************************************
  */
/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_EPICTL_H__
#define __DRV_EPICTL_H__

#include <rtdevice.h>
#include "bf0_hal.h"

#ifdef __cplusplus
extern "C" {
#endif
#ifdef HAL_EPICTL_ENABLED
typedef void (*drv_epictl_cplt_cbk)(EPICTL_HandleTypeDef *);


int  drv_epictl_init(void);
void drv_epictl_deinit(void);

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
rt_err_t drv_epictl_transfer(EPICTL_DataType *p_cfg, drv_epictl_cplt_cbk cbk);

#endif /*HAL_EPICTL_ENABLED*/
#ifdef __cplusplus
}
#endif

#endif /* __DRV_EPICTL_H__ */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/