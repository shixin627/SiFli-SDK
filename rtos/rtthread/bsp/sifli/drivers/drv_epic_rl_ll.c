/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*

*/
#include "drv_epic.h"
#if defined(BSP_USING_EPIC) && defined(DRV_EPIC_NEW_API)
#include "drv_epic_private.h"


struct rt_semaphore epic_sema;

__ROM_USED void EPIC_IRQHandler(void)
{
    EPIC_HandleTypeDef *epic;

    rt_interrupt_enter();

    RT_ASSERT(NULL != gp_drv_epic);
    epic = drv_get_epic_handle();
    RT_ASSERT(RT_NULL != epic);

    HAL_EPIC_IRQHandler(epic);

    rt_interrupt_leave();
}

#ifdef HAL_EZIP_MODULE_ENABLED

__ROM_USED void EZIP_IRQHandler(void)
{
    EZIP_HandleTypeDef *ezip;

    rt_interrupt_enter();

    RT_ASSERT(NULL != gp_drv_epic);
    ezip = drv_get_ezip_handle();
    RT_ASSERT(RT_NULL != ezip);

    HAL_EZIP_IRQHandler(ezip);

    rt_interrupt_leave();
}
#endif /* HAL_EZIP_MODULE_ENABLED */


#ifdef HAL_JPEGD_MODULE_ENABLED
__ROM_USED void JPEGD_IRQHandler(void)
{
    JPEGD_HandleTypeDef *jpegd;

    rt_interrupt_enter();

    RT_ASSERT(NULL != gp_drv_epic);
    jpegd = drv_get_jpegd_handle();
    RT_ASSERT(RT_NULL != jpegd);

    HAL_JPEGD_IRQHandler(jpegd);

    rt_interrupt_leave();
}
#endif /* HAL_JPEGD_MODULE_ENABLED */



#ifndef __DEBUG__
static void gpu_reset(void)
{
#ifdef HAL_EPIC_MODULE_ENABLED
    if (epic)
    {
        rt_base_t level;
        level = rt_hw_interrupt_disable();

        epic->State = HAL_EPIC_STATE_READY;
        epic->ErrorCode = 0;
        epic->IntXferCpltCallback = NULL;
        HAL_RCC_ResetModule(RCC_MOD_EPIC);
#ifdef HAL_EZIP_MODULE_ENABLED
        epic->hezip->State = HAL_EZIP_STATE_READY;
        epic->hezip->ErrorCode = 0;
        epic->hezip->CpltCallback = NULL;
        HAL_RCC_ResetModule(RCC_MOD_EZIP);
        HAL_NVIC_ClearPendingIRQ(EZIP_IRQn);
#endif /* HAL_EZIP_MODULE_ENABLED */
        HAL_NVIC_ClearPendingIRQ(EPIC_IRQn);

        rt_hw_interrupt_enable(level);
    }
#endif /* HAL_EPIC_MODULE_ENABLED */
}
#endif /* __DEBUG__ */










static void wait_hw_done(void)
{
    __DEBUG_RENDER_LIST_WAIT_EPIC_START__;
    rt_err_t err;
    uint32_t start_ms = rt_tick_get_millisecond();
    do
    {
        err = rt_sem_take(&epic_sema, rt_tick_from_millisecond(GPU_BLEND_EXP_MS));

        if (RT_EOK != err)
        {
            LOG_E("wait_hw_done timeout(-2)? err=%d", err);
#ifndef __DEBUG__
            gpu_reset();
#else
#ifndef BF0_ACPU //Print GPU error info in HCPU.
            print_gpu_error_info();
#endif
            RT_ASSERT(0); //Raise an assertion in debug mode.
#endif /* __RELEASE__ */
        }
    }
    while (RT_EOK != err);
    gp_drv_epic->rd_epic_async_wait  += rt_tick_get_millisecond() - start_ms;
    __DEBUG_RENDER_LIST_WAIT_EPIC_END__;
}



static rt_err_t epic_sem_trytake(void)
{
    rt_err_t err;

    err = rt_sem_trytake(&epic_sema);

    return err;
}


static rt_err_t epic_sem_release(void)
{
    rt_err_t err;
    RT_ASSERT(0 == epic_sema.value);
    err = rt_sem_release(&epic_sema);
    return err;
}




static void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    epic_cbk_ctx_t *p_ctx = (epic_cbk_ctx_t *) epic->user_data;
    epic_sem_release();
}



static inline void statistics_hw_start(void)
{
    gp_drv_epic->last_rd_operation_start_epic_cnt = gp_drv_epic->epic_handle.PerfCnt;
}


static inline void statistics_hw_done(void)
{
    if (gp_drv_epic->p_last_rd_operation)
    {
        gp_drv_epic->p_last_rd_operation->hw += GetElapsedUs(gp_drv_epic->last_rd_operation_start_epic_cnt, gp_drv_epic->epic_handle.PerfCnt) ;
        gp_drv_epic->p_last_rd_operation = NULL;
    }
}

static inline void statistics_hw_restart(void)
{
    statistics_hw_done();
    statistics_hw_start();
}


static inline void statistics_hal_start(void)
{
    gp_drv_epic->start_hal_epic_cnt = HAL_DBG_DWT_GetCycles();
}

static inline void statistics_hal_end(void)
{
    gp_drv_epic->rd_epic_hal_us += GetElapsedUs(gp_drv_epic->start_hal_epic_cnt, HAL_DBG_DWT_GetCycles());
}


HAL_StatusTypeDef Call_Hal_Api(HAL_API_TypeDef api, void *p1, void *p2, void *p3)
{
    HAL_StatusTypeDef ret;
    /*0:Not started, 1:continue blend mode, 2:Stopping continue blend mode*/
    static uint8_t cont_blend_states = 0;

    EPIC_HandleTypeDef *hw_epic_handle = drv_get_epic_handle();

    if (gp_drv_epic->dbg_flag_print_exe_detail)
    {
#define PRINT_API_NAME(api) case api: LOG_I("Call_Hal_Api "#api); break;
        switch (api)
        {
            PRINT_API_NAME(HAL_API_CONT_BLEND);
            PRINT_API_NAME(HAL_API_CONT_BLEND_STOP);
            PRINT_API_NAME(HAL_API_CONT_BLEND_ASYNC_STOP);
            PRINT_API_NAME(HAL_API_BLEND_EX);
            PRINT_API_NAME(HAL_API_TRANSFORM);
            PRINT_API_NAME(HAL_API_COPY);
            PRINT_API_NAME(HAL_API_ALL_STOP);
        default:
            LOG_I("Call_Hal_Api= %d", api);
            break;
        }
        LOG_I("cont_blend_states=%d", cont_blend_states);
    }

    if (HAL_API_CONT_BLEND == api)
    {
        if (cont_blend_states != 1)
        {
            if (2 == cont_blend_states)
            {
                /* Restart cont_blend as required, and keep holding semaphore here. */
                HAL_EPIC_ContBlendStop(hw_epic_handle);
            }
            else
            {
                /*
                    Take new semaphore & Wait previous async things done.
                */
                wait_hw_done();
            }

            statistics_hw_restart();

            cont_blend_states = 1;
            statistics_hal_start();
            ret = HAL_EPIC_ContBlendStart(hw_epic_handle,
                                          (EPIC_LayerConfigTypeDef *)p1,
                                          (EPIC_LayerConfigTypeDef *)p2,
                                          (EPIC_LayerConfigTypeDef *)p3);
            statistics_hal_end();

        }
        else
        {
            statistics_hal_start();
            ret = HAL_EPIC_ContBlendRepeat(hw_epic_handle,
                                           (EPIC_LayerConfigTypeDef *)p1,
                                           (EPIC_LayerConfigTypeDef *)p2,
                                           (EPIC_LayerConfigTypeDef *)p3);
            statistics_hal_end();
        }
        DRV_EPIC_ASSERT(HAL_OK == ret);
        return HAL_OK;
    }
    else if (HAL_API_CONT_BLEND_STOP == api)
    {
        if (cont_blend_states != 0)
        {
            ret = HAL_EPIC_ContBlendStop(hw_epic_handle);
            DRV_EPIC_ASSERT(HAL_OK == ret);
            epic_sem_release();
            cont_blend_states = 0;
            statistics_hw_done();
        }
        return HAL_OK;
    }
    else if (HAL_API_ALL_STOP == api)
    {
        if (cont_blend_states != 0)
        {
            //It is continue blending, stop it.
            ret = HAL_EPIC_ContBlendStop(hw_epic_handle);
            DRV_EPIC_ASSERT(HAL_OK == ret);
            epic_sem_release();
            cont_blend_states = 0;
            statistics_hw_done();
        }
        else
        {
            //Waitting for async blending done.
            wait_hw_done();
            epic_sem_release();
        }
        return HAL_OK;
    }
    else if (HAL_API_CONT_BLEND_ASYNC_STOP == api)
    {
        /*
            the continue blend operations should be restarted next time
        */
        if (1 == cont_blend_states) cont_blend_states = 2;
        return HAL_OK;
    }
    else if (HAL_API_TRANSFORM == api)
    {
        if (cont_blend_states != 0)
        {
            /* Stop cont_blend and keep holding semaphore here. */
            ret = HAL_EPIC_ContBlendStop(hw_epic_handle);
            DRV_EPIC_ASSERT(HAL_OK == ret);
            cont_blend_states = 0;
            /* Keep holding semaphore
                epic_sem_release();
                wait_hw_done();
            */
        }
        else
        {
            /*
                Take new semaphore & Wait previous async things done.
            */
            wait_hw_done();
        }
        statistics_hw_restart();
        statistics_hal_start();
        ret = HAL_EPIC_Adv(hw_epic_handle,
                           (EPIC_LayerConfigTypeDef *)p1,
                           (uint8_t)(uint32_t)p2,
                           (EPIC_LayerConfigTypeDef *)p3);
        statistics_hal_end();
        epic_sem_release();
        return ret;
    }
    else if ((HAL_API_BLEND_EX == api)
             || (HAL_API_COPY == api))
    {
        uint8_t using_shadow_handler = 0;

        if (1 == gp_drv_epic->dbg_flag_dis_ram_instance)
        {
            if (cont_blend_states != 0)
            {
                ret = HAL_EPIC_ContBlendStop(hw_epic_handle);
                DRV_EPIC_ASSERT(HAL_OK == ret);
                cont_blend_states = 0;
                /* Keep holding semaphore
                    epic_sem_release();
                    wait_hw_done();
                */
            }
            else
            {
                wait_hw_done();
            }
        }
        else if (cont_blend_states != 0)
        {
            /*Cont blend is busy*/
            if (HAL_EPIC_IsHWBusy(hw_epic_handle))
                using_shadow_handler = 1;
            else
            {
                ret = HAL_EPIC_ContBlendStop(hw_epic_handle);
                DRV_EPIC_ASSERT(HAL_OK == ret);
                cont_blend_states = 0;
                /* Keep holding semaphore
                    epic_sem_release();
                    wait_hw_done();
                */
            }
        }
        /*Async things are busy*/
        else if (RT_EOK != epic_sem_trytake())
        {
            using_shadow_handler = 1;
        }
        else
        {
            ;
        }

        if (using_shadow_handler)
        {
            EPIC_HandleTypeDef *shadow_epic = &gp_drv_epic->epic_handle2;
            HAL_EPIC_BlendFastStart_Init(shadow_epic);
            shadow_epic->XferCpltCallback = NULL;//Avoid wrong callback if epic has nothing to do.

            statistics_hal_start();
            switch (api)
            {
            case HAL_API_BLEND_EX:
                ret = HAL_EPIC_BlendStartEx_IT(shadow_epic,
                                               (EPIC_LayerConfigTypeDef *)p1,
                                               (uint8_t)(uint32_t)p2,
                                               (EPIC_LayerConfigTypeDef *)p3);
                break;
            case HAL_API_COPY:
                ret = HAL_EPIC_Copy_IT(shadow_epic,
                                       (EPIC_BlendingDataType *)p1,
                                       (EPIC_BlendingDataType *)p2);
                break;
            default:
                DRV_EPIC_ASSERT(0);
                break;
            }
            DRV_EPIC_ASSERT(HAL_OK == ret);
            statistics_hal_end();

            if (HAL_EPIC_STATE_BUSY == shadow_epic->State)
            {
                if (cont_blend_states != 0)
                {
                    ret = HAL_EPIC_ContBlendStop(hw_epic_handle);
                    DRV_EPIC_ASSERT(HAL_OK == ret);
                    cont_blend_states = 0;
                    /* Keep holding semaphore
                        epic_sem_release();
                        wait_hw_done();
                    */
                }
                else
                {
                    wait_hw_done();//Take semaphore
                }
                statistics_hw_restart();
                statistics_hal_start();
                shadow_epic->XferCpltCallback = epic_cplt_callback;
                shadow_epic->user_data = &gp_drv_epic->epic_cb_ctx;
                ret = HAL_EPIC_BlendFastStart_IT(hw_epic_handle, shadow_epic);
                DRV_EPIC_ASSERT(HAL_OK == ret);
                statistics_hal_end();
            }
            else
            {
                ;//Nothing here
            }
        }
        else
        {
            statistics_hw_restart();
            statistics_hal_start();
            //Semapahore should be taken before.
            hw_epic_handle->user_data = &gp_drv_epic->epic_cb_ctx;
            hw_epic_handle->XferCpltCallback = epic_cplt_callback;

            switch (api)
            {
            case HAL_API_BLEND_EX:
                ret = HAL_EPIC_BlendStartEx_IT(hw_epic_handle,
                                               (EPIC_LayerConfigTypeDef *)p1,
                                               (uint8_t)(uint32_t)p2,
                                               (EPIC_LayerConfigTypeDef *)p3);
                break;
            case HAL_API_COPY:
                ret = HAL_EPIC_Copy_IT(hw_epic_handle,
                                       (EPIC_BlendingDataType *)p1,
                                       (EPIC_BlendingDataType *)p2);
                break;
            default:
                DRV_EPIC_ASSERT(0);
                break;
            }
            DRV_EPIC_ASSERT(HAL_OK == ret);
            statistics_hal_end();
        }

        return HAL_OK;
    }
    else
    {
        DRV_EPIC_ASSERT(0);
    }

    return HAL_ERROR;
}




int drv_epic_ll_init(void)
{
    rt_err_t err = rt_sem_init(&epic_sema, "epic", 1, RT_IPC_FLAG_FIFO);
    RT_ASSERT(RT_EOK == err);

    HAL_NVIC_SetPriority(EPIC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EPIC_IRQn);

#ifdef HAL_EZIP_MODULE_ENABLED
    HAL_NVIC_SetPriority(EZIP_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EZIP_IRQn);
#endif /* HAL_EZIP_MODULE_ENABLED */

#ifdef HAL_JPEGD_MODULE_ENABLED
    HAL_NVIC_SetPriority(JPEGD_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(JPEGD_IRQn);
#endif /* HAL_JPEGD_MODULE_ENABLED */
    return 0;
}

INIT_PRE_APP_EXPORT(drv_epic_ll_init);
#endif /* DRV_EPIC_NEW_API */