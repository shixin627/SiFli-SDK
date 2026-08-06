/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */
/** @defgroup DCMI DCMI
  * @brief DCMI HAL module driver
  * @{
  */

#ifdef HAL_DCMI_MODULE_ENABLED


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define HAL_TIMEOUT_DCMI_STOP    ((uint32_t)1000) /* Set timeout to 1s  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void       DCMI_DMAXferCplt(DMA_HandleTypeDef *hdma);
static void       DCMI_DMAError(DMA_HandleTypeDef *hdma);

/* Exported functions --------------------------------------------------------*/

/** @defgroup DCMI_Exported_Functions DCMI Exported Functions
  * @{
  */

/** @defgroup DCMI_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
                ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DCMI
      (+) De-initialize the DCMI

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the DCMI according to the specified
  *         parameters in the DCMI_InitTypeDef and create the associated handle.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DCMI_Init(DCMI_HandleTypeDef *hdcmi)
{
    /* Check the DCMI peripheral state */
    if (hdcmi == NULL)
    {
        return HAL_ERROR;
    }

    HAL_RCC_EnableModule(RCC_MOD_DCMI);
    HAL_RCC_ResetModule(RCC_MOD_DCMI);

    /* Check function parameters */
    HAL_ASSERT(IS_DCMI_PCKPOLARITY(hdcmi->Init.PCKPolarity));
    HAL_ASSERT(IS_DCMI_VSPOLARITY(hdcmi->Init.VSPolarity));
    HAL_ASSERT(IS_DCMI_HSPOLARITY(hdcmi->Init.HSPolarity));
    HAL_ASSERT(IS_DCMI_SYNCHRO(hdcmi->Init.SynchroMode));
    HAL_ASSERT(IS_DCMI_EXTENDED_DATA(hdcmi->Init.ExtendedDataMode));

    if (hdcmi->State == HAL_DCMI_STATE_RESET)
    {
        /* Init the DCMI Callback settings */
#if (USE_HAL_DCMI_REGISTER_CALLBACKS == 1)
        hdcmi->FrameEventCallback = HAL_DCMI_FrameEventCallback; /* Legacy weak FrameEventCallback  */
        hdcmi->VsyncEventCallback = HAL_DCMI_VsyncEventCallback; /* Legacy weak VsyncEventCallback  */
        hdcmi->LineEventCallback  = HAL_DCMI_LineEventCallback;  /* Legacy weak LineEventCallback   */
        hdcmi->ErrorCallback      = HAL_DCMI_ErrorCallback;      /* Legacy weak ErrorCallback       */

        if (hdcmi->MspInitCallback == NULL)
        {
            /* Legacy weak MspInit Callback        */
            hdcmi->MspInitCallback = HAL_DCMI_MspInit;
        }
        /* Initialize the low level hardware (MSP) */
        hdcmi->MspInitCallback(hdcmi);
#else
        /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
        HAL_DCMI_MspInit(hdcmi);
#endif /* (USE_HAL_DCMI_REGISTER_CALLBACKS) */
    }

    /* Change the DCMI state */
    hdcmi->State = HAL_DCMI_STATE_BUSY;

    /* Configures the HS, VS, DE and PC polarity */
    hdcmi->Instance->CR &= ~(DCMI_CR_PCKPOL | DCMI_CR_HSPOL  | DCMI_CR_VSPOL  | DCMI_CR_EDM_Msk);

    hdcmi->Instance->CR |= (uint32_t)(hdcmi->Init.SynchroMode | \
                                      hdcmi->Init.VSPolarity  | hdcmi->Init.HSPolarity  | \
                                      hdcmi->Init.PCKPolarity | hdcmi->Init.ExtendedDataMode);

    if (hdcmi->Init.SynchroMode == DCMI_SYNCHRO_EMBEDDED)
    {
        hdcmi->Instance->ESCR = (((uint32_t)hdcmi->Init.SyncroCode.FrameStartCode)    | \
                                 ((uint32_t)hdcmi->Init.SyncroCode.LineStartCode << DCMI_ESCR_LSC_Pos) | \
                                 ((uint32_t)hdcmi->Init.SyncroCode.LineEndCode << DCMI_ESCR_LEC_Pos) | \
                                 ((uint32_t)hdcmi->Init.SyncroCode.FrameEndCode << DCMI_ESCR_FEC_Pos));

    }

    /* Enable the Line, Vsync, Error and Overrun interrupts */
    __HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC | DCMI_IT_ERR | DCMI_IT_OVR);

    /* Update error code */
    hdcmi->ErrorCode = HAL_DCMI_ERROR_NONE;

    /* Initialize the DCMI state*/
    hdcmi->State  = HAL_DCMI_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  Deinitializes the DCMI peripheral registers to their default reset
  *         values.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_DCMI_DeInit(DCMI_HandleTypeDef *hdcmi)
{
#if (USE_HAL_DCMI_REGISTER_CALLBACKS == 1)
    if (hdcmi->MspDeInitCallback == NULL)
    {
        hdcmi->MspDeInitCallback = HAL_DCMI_MspDeInit;
    }
    /* De-Initialize the low level hardware (MSP) */
    hdcmi->MspDeInitCallback(hdcmi);
#else
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
    HAL_DCMI_MspDeInit(hdcmi);
#endif /* (USE_HAL_DCMI_REGISTER_CALLBACKS) */

    /* Update error code */
    hdcmi->ErrorCode = HAL_DCMI_ERROR_NONE;

    /* Initialize the DCMI state*/
    hdcmi->State = HAL_DCMI_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hdcmi);

    HAL_RCC_DisableModule(RCC_MOD_DCMI);

    return HAL_OK;
}

/**
  * @brief  Initializes the DCMI MSP.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
__weak void HAL_DCMI_MspInit(DCMI_HandleTypeDef *hdcmi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdcmi);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DCMI_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes the DCMI MSP.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
__weak void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef *hdcmi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdcmi);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DCMI_MspDeInit could be implemented in the user file
     */
}

/**
  * @}
  */
/** @defgroup DCMI_Exported_Functions_Group2 IO operation functions
 *  @brief   IO operation functions
 *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure destination address and data length and
          Enables DCMI DMA request and enables DCMI capture
      (+) Stop the DCMI capture.
      (+) Handles DCMI interrupt request.

@endverbatim
  * @{
  */

/**
  * @brief  Enables DCMI DMA request and enables DCMI capture
  * @param  hdcmi     pointer to a DCMI_HandleTypeDef structure that contains
  *                    the configuration information for DCMI.
  * @param  DCMI_Mode DCMI capture mode snapshot or continuous grab.
  * @param  pData     The destination memory Buffer address (LCD Frame buffer).
  * @param  Length    The length of capture to be transferred.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DCMI_Start_DMA(DCMI_HandleTypeDef *hdcmi, uint32_t DCMI_Mode, uint32_t pData, uint32_t Length)
{
    /* Initialize the second memory address */
    uint32_t SecondMemAddress = 0;

    /* Check function parameters */
    HAL_ASSERT(IS_DCMI_CAPTURE_MODE(DCMI_Mode));

    /* Process Locked */
    __HAL_LOCK(hdcmi);

    /* Lock the DCMI peripheral state */
    hdcmi->State = HAL_DCMI_STATE_BUSY;

    /* Enable DCMI by setting DCMIEN bit */
    __HAL_DCMI_ENABLE(hdcmi);

    /* Configure the DCMI Mode */
    hdcmi->Instance->CR &= ~(DCMI_CR_CM);
    hdcmi->Instance->CR |= (uint32_t)(DCMI_Mode);

    /* Set the DMA memory0 conversion complete callback */
    hdcmi->DMA_Handle->XferCpltCallback = DCMI_DMAXferCplt;

    /* Set the DMA error callback */
    hdcmi->DMA_Handle->XferErrorCallback = DCMI_DMAError;

    /* Set the dma abort callback */
    hdcmi->DMA_Handle->XferAbortCallback = NULL;

    /* Reset transfer counters value */
    hdcmi->XferCount = 0;
    hdcmi->XferTransferNumber = 0;
    hdcmi->XferSize = 0;
    hdcmi->pBuffPtr = 0;

    /* Enable the DMA Stream */
    if (HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, (uint32_t)pData, Length) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Enable Capture */
    hdcmi->Instance->CR |= DCMI_CR_CAPTURE;

    /* Release Lock */
    __HAL_UNLOCK(hdcmi);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Disable DCMI DMA request and Disable DCMI capture
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DCMI_Stop(DCMI_HandleTypeDef *hdcmi)
{
    register uint32_t count = HAL_TIMEOUT_DCMI_STOP * (SystemCoreClock / 8U / 1000U);
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hdcmi);

    /* Lock the DCMI peripheral state */
    hdcmi->State = HAL_DCMI_STATE_BUSY;

    /* Disable Capture */
    hdcmi->Instance->CR &= ~(DCMI_CR_CAPTURE);

    /* Check if the DCMI capture effectively disabled */
    do
    {
        if (count-- == 0U)
        {
            /* Update error code */
            hdcmi->ErrorCode |= HAL_DCMI_ERROR_TIMEOUT;

            status = HAL_TIMEOUT;
            break;
        }
    }
    while ((hdcmi->Instance->CR & DCMI_CR_CAPTURE) != 0U);

    /* Disable the DCMI */
    __HAL_DCMI_DISABLE(hdcmi);

    /* Disable the DMA */
    (void)HAL_DMA_Abort(hdcmi->DMA_Handle);

    /* Update error code */
    hdcmi->ErrorCode |= HAL_DCMI_ERROR_NONE;

    /* Change DCMI state */
    hdcmi->State = HAL_DCMI_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hdcmi);

    /* Return function status */
    return status;
}

/**
  * @brief  Suspend DCMI capture
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DCMI_Suspend(DCMI_HandleTypeDef *hdcmi)
{
    register uint32_t count = HAL_TIMEOUT_DCMI_STOP * (SystemCoreClock / 8U / 1000U);
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hdcmi);

    if (hdcmi->State == HAL_DCMI_STATE_BUSY)
    {
        /* Change DCMI state */
        hdcmi->State = HAL_DCMI_STATE_SUSPENDED;

        /* Disable Capture */
        hdcmi->Instance->CR &= ~(DCMI_CR_CAPTURE);

        /* Check if the DCMI capture effectively disabled */
        do
        {
            if (count-- == 0U)
            {
                /* Update error code */
                hdcmi->ErrorCode |= HAL_DCMI_ERROR_TIMEOUT;

                /* Change DCMI state */
                hdcmi->State = HAL_DCMI_STATE_READY;

                status = HAL_TIMEOUT;
                break;
            }
        }
        while ((hdcmi->Instance->CR & DCMI_CR_CAPTURE) != 0U);
    }
    /* Process Unlocked */
    __HAL_UNLOCK(hdcmi);

    /* Return function status */
    return status;
}

/**
  * @brief  Resume DCMI capture
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DCMI_Resume(DCMI_HandleTypeDef *hdcmi)
{
    /* Process locked */
    __HAL_LOCK(hdcmi);

    if (hdcmi->State == HAL_DCMI_STATE_SUSPENDED)
    {
        /* Change DCMI state */
        hdcmi->State = HAL_DCMI_STATE_BUSY;

        /* Disable Capture */
        hdcmi->Instance->CR |= DCMI_CR_CAPTURE;
    }
    /* Process Unlocked */
    __HAL_UNLOCK(hdcmi);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Handles DCMI interrupt request.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for the DCMI.
  * @retval None
  */
void HAL_DCMI_IRQHandler(DCMI_HandleTypeDef *hdcmi)
{
    uint32_t isr_value = READ_REG(hdcmi->Instance->MIS);

    /* Synchronization error interrupt management *******************************/
    if ((isr_value & DCMI_FLAG_ERRRI) == DCMI_FLAG_ERRRI)
    {
        /* Clear the Synchronization error flag */
        __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_FLAG_ERRRI);

        /* Update error code */
        hdcmi->ErrorCode |= HAL_DCMI_ERROR_SYNC;

        /* Change DCMI state */
        hdcmi->State = HAL_DCMI_STATE_ERROR;

        /* Set the synchronization error callback */
        hdcmi->DMA_Handle->XferAbortCallback = DCMI_DMAError;

        /* Abort the DMA Transfer */
        (void)HAL_DMA_Abort_IT(hdcmi->DMA_Handle);
    }
    /* Overflow interrupt management ********************************************/
    if ((isr_value & DCMI_FLAG_OVRRI) == DCMI_FLAG_OVRRI)
    {
        /* Clear the Overflow flag */
        __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_FLAG_OVRRI);

        /* Update error code */
        hdcmi->ErrorCode |= HAL_DCMI_ERROR_OVR;

        /* Change DCMI state */
        hdcmi->State = HAL_DCMI_STATE_ERROR;

        /* Set the overflow callback */
        hdcmi->DMA_Handle->XferAbortCallback = DCMI_DMAError;

        /* Abort the DMA Transfer */
        if (HAL_DMA_Abort_IT(hdcmi->DMA_Handle) != HAL_OK)
        {
            DCMI_DMAError(hdcmi->DMA_Handle);
        }
    }
    /* Line Interrupt management ************************************************/
    if ((isr_value & DCMI_FLAG_LINERI) == DCMI_FLAG_LINERI)
    {
        /* Clear the Line interrupt flag */
        __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_FLAG_LINERI);

        /* Line interrupt Callback */
#if (USE_HAL_DCMI_REGISTER_CALLBACKS == 1)
        /*Call registered DCMI line event callback*/
        hdcmi->LineEventCallback(hdcmi);
#else
        HAL_DCMI_LineEventCallback(hdcmi);
#endif /* USE_HAL_DCMI_REGISTER_CALLBACKS */
    }
    /* VSYNC interrupt management ***********************************************/
    if ((isr_value & DCMI_FLAG_VSYNCRI) == DCMI_FLAG_VSYNCRI)
    {
        /* Clear the VSYNC flag */
        __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_FLAG_VSYNCRI);

        /* VSYNC Callback */
#if (USE_HAL_DCMI_REGISTER_CALLBACKS == 1)
        /*Call registered DCMI vsync event callback*/
        hdcmi->VsyncEventCallback(hdcmi);
#else
        HAL_DCMI_VsyncEventCallback(hdcmi);
#endif /* USE_HAL_DCMI_REGISTER_CALLBACKS */
    }
    /* FRAME interrupt management ***********************************************/
    if ((isr_value & DCMI_FLAG_FRAMERI) == DCMI_FLAG_FRAMERI)
    {
        /* When snapshot mode, disable Vsync, Error and Overrun interrupts */
        if ((hdcmi->Instance->CR & DCMI_CR_CM) == DCMI_MODE_SNAPSHOT)
        {
            /* Disable the Line, Vsync, Error and Overrun interrupts */
            __HAL_DCMI_DISABLE_IT(hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC | DCMI_IT_ERR | DCMI_IT_OVR);
        }

        /* Disable the Frame interrupt */
        __HAL_DCMI_DISABLE_IT(hdcmi, DCMI_IT_FRAME);

        /* Clear the End of Frame flag */
        __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_FLAG_FRAMERI);

        /* Frame Callback */
#if (USE_HAL_DCMI_REGISTER_CALLBACKS == 1)
        /*Call registered DCMI frame event callback*/
        hdcmi->FrameEventCallback(hdcmi);
#else
        HAL_DCMI_FrameEventCallback(hdcmi);
#endif /* USE_HAL_DCMI_REGISTER_CALLBACKS */
    }
}

/**
  * @brief  Error DCMI callback.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
__weak void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdcmi);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DCMI_ErrorCallback could be implemented in the user file
     */
}

/**
  * @brief  Line Event callback.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
__weak void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DCMI_LineEventCallback could be implemented in the user file
     */
}

/**
  * @brief  VSYNC Event callback.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
__weak void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdcmi);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DCMI_VsyncEventCallback could be implemented in the user file
     */
}

/**
  * @brief  Frame Event callback.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
__weak void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdcmi);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DCMI_FrameEventCallback could be implemented in the user file
     */
}

/**
  * @}
  */

/** @defgroup DCMI_Exported_Functions_Group3 Peripheral Control functions
 *  @brief    Peripheral Control functions
 *
@verbatim
 ===============================================================================
                    ##### Peripheral Control functions #####
 ===============================================================================
[..]  This section provides functions allowing to:
      (+) Set embedded synchronization delimiters unmasks.

@endverbatim
  * @{
  */

/**
  * @brief  Set embedded synchronization delimiters unmasks.
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *               the configuration information for DCMI.
  * @param  SyncUnmask pointer to a DCMI_SyncUnmaskTypeDef structure that contains
  *                    the embedded synchronization delimiters unmasks.
  * @retval HAL status
  */
HAL_StatusTypeDef  HAL_DCMI_ConfigSyncUnmask(DCMI_HandleTypeDef *hdcmi, DCMI_SyncUnmaskTypeDef *SyncUnmask)
{
    /* Process Locked */
    __HAL_LOCK(hdcmi);

    /* Lock the DCMI peripheral state */
    hdcmi->State = HAL_DCMI_STATE_BUSY;

    /* Write DCMI embedded synchronization unmask register */
    hdcmi->Instance->ESUR = (((uint32_t)SyncUnmask->FrameStartUnmask) | \
                             ((uint32_t)SyncUnmask->LineStartUnmask << DCMI_ESUR_LSU_Pos) | \
                             ((uint32_t)SyncUnmask->LineEndUnmask << DCMI_ESUR_LEU_Pos) | \
                             ((uint32_t)SyncUnmask->FrameEndUnmask << DCMI_ESUR_FEU_Pos));

    /* Change the DCMI state*/
    hdcmi->State = HAL_DCMI_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hdcmi);

    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup DCMI_Exported_Functions_Group4 Peripheral State functions
 *  @brief    Peripheral State functions
 *
@verbatim
 ===============================================================================
               ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DCMI state.
      (+) Get the specific DCMI error flag.

@endverbatim
  * @{
  */

/**
  * @brief  Return the DCMI state
  * @param  hdcmi pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval HAL state
  */
HAL_DCMI_StateTypeDef HAL_DCMI_GetState(DCMI_HandleTypeDef *hdcmi)
{
    return hdcmi->State;
}

/**
* @brief  Return the DCMI error code
* @param  hdcmi  pointer to a DCMI_HandleTypeDef structure that contains
  *               the configuration information for DCMI.
* @retval DCMI Error Code
*/
uint32_t HAL_DCMI_GetError(DCMI_HandleTypeDef *hdcmi)
{
    return hdcmi->ErrorCode;
}

#if (USE_HAL_DCMI_REGISTER_CALLBACKS == 1)
/**
  * @brief DCMI Callback registering
  * @param hdcmi        pointer to a DCMI_HandleTypeDef structure that contains
  *                     the configuration information for DCMI.
  * @param CallbackID   dcmi Callback ID
  * @param pCallback    pointer to DCMI_CallbackTypeDef structure
  * @retval status
  */
HAL_StatusTypeDef HAL_DCMI_RegisterCallback(DCMI_HandleTypeDef *hdcmi, HAL_DCMI_CallbackIDTypeDef CallbackID, pDCMI_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* update the error code */
        hdcmi->ErrorCode |= HAL_DCMI_ERROR_INVALID_CALLBACK;
        /* update return status */
        status = HAL_ERROR;
    }
    else
    {
        if (hdcmi->State == HAL_DCMI_STATE_READY)
        {
            switch (CallbackID)
            {
            case HAL_DCMI_FRAME_EVENT_CB_ID :
                hdcmi->FrameEventCallback = pCallback;
                break;

            case HAL_DCMI_VSYNC_EVENT_CB_ID :
                hdcmi->VsyncEventCallback = pCallback;
                break;

            case HAL_DCMI_LINE_EVENT_CB_ID :
                hdcmi->LineEventCallback = pCallback;
                break;

            case HAL_DCMI_ERROR_CB_ID :
                hdcmi->ErrorCallback = pCallback;
                break;

            case HAL_DCMI_MSPINIT_CB_ID :
                hdcmi->MspInitCallback = pCallback;
                break;

            case HAL_DCMI_MSPDEINIT_CB_ID :
                hdcmi->MspDeInitCallback = pCallback;
                break;

            default :
                /* Return error status */
                status =  HAL_ERROR;
                break;
            }
        }
        else if (hdcmi->State == HAL_DCMI_STATE_RESET)
        {
            switch (CallbackID)
            {
            case HAL_DCMI_MSPINIT_CB_ID :
                hdcmi->MspInitCallback = pCallback;
                break;

            case HAL_DCMI_MSPDEINIT_CB_ID :
                hdcmi->MspDeInitCallback = pCallback;
                break;

            default :
                /* update the error code */
                hdcmi->ErrorCode |= HAL_DCMI_ERROR_INVALID_CALLBACK;
                /* update return status */
                status = HAL_ERROR;
                break;
            }
        }
        else
        {
            /* update the error code */
            hdcmi->ErrorCode |= HAL_DCMI_ERROR_INVALID_CALLBACK;
            /* update return status */
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief DCMI Callback Unregistering
  * @param hdcmi       dcmi handle
  * @param CallbackID  dcmi Callback ID
  * @retval status
  */
HAL_StatusTypeDef HAL_DCMI_UnRegisterCallback(DCMI_HandleTypeDef *hdcmi, HAL_DCMI_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (hdcmi->State == HAL_DCMI_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_DCMI_FRAME_EVENT_CB_ID :
            hdcmi->FrameEventCallback = HAL_DCMI_FrameEventCallback;  /* Legacy weak  FrameEventCallback  */
            break;

        case HAL_DCMI_VSYNC_EVENT_CB_ID :
            hdcmi->VsyncEventCallback = HAL_DCMI_VsyncEventCallback;  /* Legacy weak VsyncEventCallback       */
            break;

        case HAL_DCMI_LINE_EVENT_CB_ID :
            hdcmi->LineEventCallback = HAL_DCMI_LineEventCallback;    /* Legacy weak LineEventCallback   */
            break;

        case HAL_DCMI_ERROR_CB_ID :
            hdcmi->ErrorCallback = HAL_DCMI_ErrorCallback;           /* Legacy weak ErrorCallback        */
            break;

        case HAL_DCMI_MSPINIT_CB_ID :
            hdcmi->MspInitCallback = HAL_DCMI_MspInit;
            break;

        case HAL_DCMI_MSPDEINIT_CB_ID :
            hdcmi->MspDeInitCallback = HAL_DCMI_MspDeInit;
            break;

        default :
            /* update the error code */
            hdcmi->ErrorCode |= HAL_DCMI_ERROR_INVALID_CALLBACK;
            /* update return status */
            status = HAL_ERROR;
            break;
        }
    }
    else if (hdcmi->State == HAL_DCMI_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_DCMI_MSPINIT_CB_ID :
            hdcmi->MspInitCallback = HAL_DCMI_MspInit;
            break;

        case HAL_DCMI_MSPDEINIT_CB_ID :
            hdcmi->MspDeInitCallback = HAL_DCMI_MspDeInit;
            break;

        default :
            /* update the error code */
            hdcmi->ErrorCode |= HAL_DCMI_ERROR_INVALID_CALLBACK;
            /* update return status */
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        /* update the error code */
        hdcmi->ErrorCode |= HAL_DCMI_ERROR_INVALID_CALLBACK;
        /* update return status */
        status = HAL_ERROR;
    }

    return status;
}
#endif /* USE_HAL_DCMI_REGISTER_CALLBACKS */

/**
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/** @defgroup DCMI_Private_Functions DCMI Private Functions
  * @{
  */
/**
* @brief  DMA conversion complete callback.
* @param  hdma pointer to a DMA_HandleTypeDef structure that contains
*                the configuration information for the specified DMA module.
* @retval None
*/
static void DCMI_DMAXferCplt(DMA_HandleTypeDef *hdma)
{
    uint32_t tmp = 0;

    DCMI_HandleTypeDef *hdcmi = (DCMI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;


    /* Check if the frame is transferred */
    if (hdcmi->XferCount == hdcmi->XferTransferNumber)
    {
        /* Enable the Frame interrupt */
        __HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME);

        /* When snapshot mode, set dcmi state to ready */
        if ((hdcmi->Instance->CR & DCMI_CR_CM) == DCMI_MODE_SNAPSHOT)
        {
            hdcmi->State = HAL_DCMI_STATE_READY;
        }
    }
}

/**
  * @brief  DMA error callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void DCMI_DMAError(DMA_HandleTypeDef *hdma)
{
    DCMI_HandleTypeDef *hdcmi = (DCMI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdcmi->DMA_Handle->ErrorCode != HAL_DMA_ERROR_NONE)
    {
        /* Initialize the DCMI state*/
        hdcmi->State = HAL_DCMI_STATE_READY;

        /* Set DCMI Error Code */
        hdcmi->ErrorCode |= HAL_DCMI_ERROR_DMA;
    }

    /* DCMI error Callback */
#if (USE_HAL_DCMI_REGISTER_CALLBACKS == 1)
    /*Call registered DCMI error callback*/
    hdcmi->ErrorCallback(hdcmi);
#else
    HAL_DCMI_ErrorCallback(hdcmi);
#endif /* USE_HAL_DCMI_REGISTER_CALLBACKS */

}

/**
  * @}
  */

/**
  * @}
  */
#endif /* HAL_DCMI_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
