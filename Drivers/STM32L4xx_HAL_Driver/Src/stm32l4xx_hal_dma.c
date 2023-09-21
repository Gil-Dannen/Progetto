/**
  ******************************************************************************
  * @file    stm32l4xx_hal_dma.c
  * @author  MCD Application Team
  * @brief   DMA HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Direct Memory Access (DMA) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and errors functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) Enable and configure the peripheral to be connected to the DMA Channel
       (except for internal SRAM / FLASH memories: no initialization is
       necessary). Please refer to the Reference manual for connection between peripherals
       and DMA requests.

   (#) For a given Channel, program the required configuration through the following parameters:
       Channel request, Transfer Direction, Source and Destination data formats,
       Circular or Normal mode, Channel Priority level, Source and Destination Increment mode
       using HAL_DMA_Init() function.

       Prior to HAL_DMA_Init the peripheral clock shall be enabled for both DMA & DMAMUX
       thanks to:
      (##) DMA1 or DMA2: __HAL_RCC_DMA1_CLK_ENABLE() or  __HAL_RCC_DMA2_CLK_ENABLE() ;
      (##) DMAMUX1:      __HAL_RCC_DMAMUX1_CLK_ENABLE();

   (#) Use HAL_DMA_GetState() function to return the DMA state and HAL_DMA_GetError() in case of error
       detection.

   (#) Use HAL_DMA_Abort() function to abort the current transfer

     -@-   In Memory-to-Memory transfer mode, Circular mode is not allowed.

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Use HAL_DMA_Start() to start DMA transfer after the configuration of Source
           address and destination address and the Length of data to be transferred
       (+) Use HAL_DMA_PollForTransfer() to poll for the end of current transfer, in this
           case a fixed Timeout can be configured by User depending from his application.

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Configure the DMA interrupt priority using HAL_NVIC_SetPriority()
       (+) Enable the DMA IRQ handler using HAL_NVIC_EnableIRQ()
       (+) Use HAL_DMA_Start_IT() to start DMA transfer after the configuration of
           Source address and destination address and the Length of data to be transferred.
           In this case the DMA interrupt is configured
       (+) Use HAL_DMA_IRQHandler() called under DMA_IRQHandler() Interrupt subroutine
       (+) At the end of data transfer HAL_DMA_IRQHandler() function is executed and user can
              add his own function to register callbacks with HAL_DMA_RegisterCallback().

     *** DMA HAL driver macros list ***
     =============================================
     [..]
       Below the list of macros in DMA HAL driver.

       (+) __HAL_DMA_ENABLE: Enable the specified DMA Channel.
       (+) __HAL_DMA_DISABLE: Disable the specified DMA Channel.
       (+) __HAL_DMA_GET_FLAG: Get the DMA Channel pending flags.
       (+) __HAL_DMA_CLEAR_FLAG: Clear the DMA Channel pending flags.
       (+) __HAL_DMA_ENABLE_IT: Enable the specified DMA Channel interrupts.
       (+) __HAL_DMA_DISABLE_IT: Disable the specified DMA Channel interrupts.
       (+) __HAL_DMA_GET_IT_SOURCE: Check whether the specified DMA Channel interrupt is enabled or not.

     [..]
      (@) You can refer to the DMA HAL driver header file for more useful macros

  @endverbatim
  ******************************************************************************
  */


#include "stm32l4xx_hal.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup DMA DMA
  * @brief DMA HAL module driver
  * @{
  */

#ifdef HAL_DMA_MODULE_ENABLED






/** @defgroup DMA_Private_Functions DMA Private Functions
  * @{
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
#if defined(DMAMUX1)
static void DMA_CalcDMAMUXChannelBaseAndMask(DMA_HandleTypeDef *hdma);
static void DMA_CalcDMAMUXRequestGenBaseAndMask(DMA_HandleTypeDef *hdma);
#endif

/**
  * @}
  */



/** @defgroup DMA_Exported_Functions DMA Exported Functions
  * @{
  */

/** @defgroup DMA_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief   Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]
    This section provides functions allowing to initialize the DMA Channel source
    and destination addresses, incrementation and data sizes, transfer direction,
    circular/normal mode selection, memory-to-memory mode selection and Channel priority value.
    [..]
    The HAL_DMA_Init() function follows the DMA configuration procedures as described in
    reference manual.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the DMA according to the specified
  *         parameters in the DMA_InitTypeDef and initialize the associated handle.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma)
{
  uint32_t tmp;


  if(hdma == NULL)
  {
    return HAL_ERROR;
  }


  assert_param(IS_DMA_ALL_INSTANCE(hdma->Instance));
  assert_param(IS_DMA_DIRECTION(hdma->Init.Direction));
  assert_param(IS_DMA_PERIPHERAL_INC_STATE(hdma->Init.PeriphInc));
  assert_param(IS_DMA_MEMORY_INC_STATE(hdma->Init.MemInc));
  assert_param(IS_DMA_PERIPHERAL_DATA_SIZE(hdma->Init.PeriphDataAlignment));
  assert_param(IS_DMA_MEMORY_DATA_SIZE(hdma->Init.MemDataAlignment));
  assert_param(IS_DMA_MODE(hdma->Init.Mode));
  assert_param(IS_DMA_PRIORITY(hdma->Init.Priority));

  assert_param(IS_DMA_ALL_REQUEST(hdma->Init.Request));


  if ((uint32_t)(hdma->Instance) < (uint32_t)(DMA2_Channel1))
  {

    hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2U;
    hdma->DmaBaseAddress = DMA1;
  }
  else
  {

    hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2U;
    hdma->DmaBaseAddress = DMA2;
  }


  hdma->State = HAL_DMA_STATE_BUSY;


  tmp = hdma->Instance->CCR;


  tmp &= ((uint32_t)~(DMA_CCR_PL    | DMA_CCR_MSIZE  | DMA_CCR_PSIZE  |
                      DMA_CCR_MINC  | DMA_CCR_PINC   | DMA_CCR_CIRC   |
                      DMA_CCR_DIR   | DMA_CCR_MEM2MEM));


  tmp |=  hdma->Init.Direction        |
          hdma->Init.PeriphInc           | hdma->Init.MemInc           |
          hdma->Init.PeriphDataAlignment | hdma->Init.MemDataAlignment |
          hdma->Init.Mode                | hdma->Init.Priority;


  hdma->Instance->CCR = tmp;

#if defined(DMAMUX1)
  /* Initialize parameters for DMAMUX channel :
     DMAmuxChannel, DMAmuxChannelStatus and DMAmuxChannelStatusMask
  */
  DMA_CalcDMAMUXChannelBaseAndMask(hdma);

  if(hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
  {

    hdma->Init.Request = DMA_REQUEST_MEM2MEM;
  }


  hdma->DMAmuxChannel->CCR = (hdma->Init.Request & DMAMUX_CxCR_DMAREQ_ID);


  hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

  if(((hdma->Init.Request > 0U) && (hdma->Init.Request <= DMA_REQUEST_GENERATOR3)))
  {
    /* Initialize parameters for DMAMUX request generator :
       DMAmuxRequestGen, DMAmuxRequestGenStatus and DMAmuxRequestGenStatusMask
    */
    DMA_CalcDMAMUXRequestGenBaseAndMask(hdma);


    hdma->DMAmuxRequestGen->RGCR = 0U;


    hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
  }
  else
  {
    hdma->DMAmuxRequestGen = 0U;
    hdma->DMAmuxRequestGenStatus = 0U;
    hdma->DMAmuxRequestGenStatusMask = 0U;
  }
#endif

#if !defined (DMAMUX1)


  if(hdma->Init.Direction != DMA_MEMORY_TO_MEMORY)
  {

    if (DMA1 == hdma->DmaBaseAddress)
    {

      DMA1_CSELR->CSELR &= ~(DMA_CSELR_C1S << (hdma->ChannelIndex & 0x1cU));


      DMA1_CSELR->CSELR |= (uint32_t) (hdma->Init.Request << (hdma->ChannelIndex & 0x1cU));
    }
    else
    {

      DMA2_CSELR->CSELR &= ~(DMA_CSELR_C1S << (hdma->ChannelIndex & 0x1cU));


      DMA2_CSELR->CSELR |= (uint32_t) (hdma->Init.Request << (hdma->ChannelIndex & 0x1cU));
    }
  }

#endif




  hdma->ErrorCode = HAL_DMA_ERROR_NONE;


  hdma->State = HAL_DMA_STATE_READY;


  hdma->Lock = HAL_UNLOCKED;

  return HAL_OK;
}

/**
  * @brief  DeInitialize the DMA peripheral.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma)
{


  if (NULL == hdma )
  {
    return HAL_ERROR;
  }


  assert_param(IS_DMA_ALL_INSTANCE(hdma->Instance));


  __HAL_DMA_DISABLE(hdma);


  if ((uint32_t)(hdma->Instance) < (uint32_t)(DMA2_Channel1))
  {

    hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2U;
    hdma->DmaBaseAddress = DMA1;
  }
  else
  {

    hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2U;
    hdma->DmaBaseAddress = DMA2;
  }


  hdma->Instance->CCR = 0U;


  hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));

#if !defined (DMAMUX1)


  if (DMA1 == hdma->DmaBaseAddress)
  {

    DMA1_CSELR->CSELR &= ~(DMA_CSELR_C1S << (hdma->ChannelIndex & 0x1cU));
  }
  else
  {

    DMA2_CSELR->CSELR &= ~(DMA_CSELR_C1S << (hdma->ChannelIndex & 0x1cU));
  }
#endif



#if defined(DMAMUX1)

  /* Initialize parameters for DMAMUX channel :
     DMAmuxChannel, DMAmuxChannelStatus and DMAmuxChannelStatusMask */

  DMA_CalcDMAMUXChannelBaseAndMask(hdma);


  hdma->DMAmuxChannel->CCR = 0U;


  hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;


  if(((hdma->Init.Request >  0U) && (hdma->Init.Request <= DMA_REQUEST_GENERATOR3)))
  {
    /* Initialize parameters for DMAMUX request generator :
       DMAmuxRequestGen, DMAmuxRequestGenStatus and DMAmuxRequestGenStatusMask
    */
    DMA_CalcDMAMUXRequestGenBaseAndMask(hdma);


    hdma->DMAmuxRequestGen->RGCR = 0U;


    hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
  }

  hdma->DMAmuxRequestGen = 0U;
  hdma->DMAmuxRequestGenStatus = 0U;
  hdma->DMAmuxRequestGenStatusMask = 0U;

#endif


  hdma->XferCpltCallback = NULL;
  hdma->XferHalfCpltCallback = NULL;
  hdma->XferErrorCallback = NULL;
  hdma->XferAbortCallback = NULL;


  hdma->ErrorCode = HAL_DMA_ERROR_NONE;


  hdma->State = HAL_DMA_STATE_RESET;


  __HAL_UNLOCK(hdma);

  return HAL_OK;
}

/**
  * @}
  */

/** @defgroup DMA_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Input and Output operation functions
 *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and Start DMA transfer
      (+) Configure the source, destination address and data length and
          Start DMA transfer with interrupt
      (+) Abort DMA transfer
      (+) Poll for transfer complete
      (+) Handle DMA interrupt request

@endverbatim
  * @{
  */

/**
  * @brief  Start the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;


  assert_param(IS_DMA_BUFFER_SIZE(DataLength));


  __HAL_LOCK(hdma);

  if(HAL_DMA_STATE_READY == hdma->State)
  {

    hdma->State = HAL_DMA_STATE_BUSY;
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;


    __HAL_DMA_DISABLE(hdma);


    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);


    __HAL_DMA_ENABLE(hdma);
  }
  else
  {

    __HAL_UNLOCK(hdma);
    status = HAL_BUSY;
  }
  return status;
}

/**
  * @brief  Start the DMA Transfer with interrupt enabled.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;


  assert_param(IS_DMA_BUFFER_SIZE(DataLength));


  __HAL_LOCK(hdma);

  if(HAL_DMA_STATE_READY == hdma->State)
  {

    hdma->State = HAL_DMA_STATE_BUSY;
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;


    __HAL_DMA_DISABLE(hdma);


    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);



    if(NULL != hdma->XferHalfCpltCallback )
    {

      __HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));
    }
    else
    {
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
      __HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_TE));
    }

#ifdef DMAMUX1


    if((hdma->DMAmuxChannel->CCR & DMAMUX_CxCR_SE) != 0U)
    {

      hdma->DMAmuxChannel->CCR |= DMAMUX_CxCR_SOIE;
    }

    if(hdma->DMAmuxRequestGen != 0U)
    {


      hdma->DMAmuxRequestGen->RGCR |= DMAMUX_RGxCR_OIE;
    }

#endif


    __HAL_DMA_ENABLE(hdma);
  }
  else
  {

    __HAL_UNLOCK(hdma);


    status = HAL_BUSY;
  }
  return status;
}

/**
  * @brief  Abort the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
    * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
  HAL_StatusTypeDef status = HAL_OK;


  if(hdma->State != HAL_DMA_STATE_BUSY)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;


    __HAL_UNLOCK(hdma);

    return HAL_ERROR;
  }
  else
  {

    __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));

#if defined(DMAMUX1)

    hdma->DMAmuxChannel->CCR &= ~DMAMUX_CxCR_SOIE;
#endif


    __HAL_DMA_DISABLE(hdma);


    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));

#if defined(DMAMUX1)

    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

    if(hdma->DMAmuxRequestGen != 0U)
    {


      hdma->DMAmuxRequestGen->RGCR &= ~DMAMUX_RGxCR_OIE;


      hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
    }

#endif


    hdma->State = HAL_DMA_STATE_READY;


    __HAL_UNLOCK(hdma);

    return status;
  }
}

/**
  * @brief  Aborts the DMA Transfer in Interrupt mode.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(HAL_DMA_STATE_BUSY != hdma->State)
  {

    hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;

    status = HAL_ERROR;
  }
  else
  {

    __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));


    __HAL_DMA_DISABLE(hdma);

#if defined(DMAMUX1)

    hdma->DMAmuxChannel->CCR &= ~DMAMUX_CxCR_SOIE;


    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));


    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

    if(hdma->DMAmuxRequestGen != 0U)
    {


      hdma->DMAmuxRequestGen->RGCR &= ~DMAMUX_RGxCR_OIE;


      hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
    }

#else

    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));
#endif


    hdma->State = HAL_DMA_STATE_READY;


    __HAL_UNLOCK(hdma);


    if(hdma->XferAbortCallback != NULL)
    {
      hdma->XferAbortCallback(hdma);
    }
  }
  return status;
}

/**
  * @brief  Polling for transfer complete.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                  the configuration information for the specified DMA Channel.
  * @param  CompleteLevel Specifies the DMA level complete.
  * @param  Timeout       Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout)
{
  uint32_t temp;
  uint32_t tickstart;

  if(HAL_DMA_STATE_BUSY != hdma->State)
  {

    hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;
    __HAL_UNLOCK(hdma);
    return HAL_ERROR;
  }


  if ((hdma->Instance->CCR & DMA_CCR_CIRC) != 0U)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
    return HAL_ERROR;
  }


  if (HAL_DMA_FULL_TRANSFER == CompleteLevel)
  {

    temp = DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1CU);
  }
  else
  {

    temp = DMA_FLAG_HT1 << (hdma->ChannelIndex & 0x1CU);
  }


  tickstart = HAL_GetTick();

  while((hdma->DmaBaseAddress->ISR & temp) == 0U)
  {
    if((hdma->DmaBaseAddress->ISR & (DMA_FLAG_TE1 << (hdma->ChannelIndex& 0x1CU))) != 0U)
    {



      hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));


      hdma->ErrorCode = HAL_DMA_ERROR_TE;


      hdma->State= HAL_DMA_STATE_READY;


      __HAL_UNLOCK(hdma);

      return HAL_ERROR;
    }

    if(Timeout != HAL_MAX_DELAY)
    {
      if(((HAL_GetTick() - tickstart) > Timeout) ||  (Timeout == 0U))
      {

        hdma->ErrorCode = HAL_DMA_ERROR_TIMEOUT;


        hdma->State = HAL_DMA_STATE_READY;


        __HAL_UNLOCK(hdma);

        return HAL_ERROR;
      }
    }
  }

#if defined(DMAMUX1)

  if(hdma->DMAmuxRequestGen != 0U)
  {

    if((hdma->DMAmuxRequestGenStatus->RGSR & hdma->DMAmuxRequestGenStatusMask) != 0U)
    {

      hdma->DMAmuxRequestGen->RGCR |= DMAMUX_RGxCR_OIE;


      hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;


      hdma->ErrorCode |= HAL_DMA_ERROR_REQGEN;
    }
  }


  if((hdma->DMAmuxChannelStatus->CSR & hdma->DMAmuxChannelStatusMask) != 0U)
  {

    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;


    hdma->ErrorCode |= HAL_DMA_ERROR_SYNC;
  }
#endif

  if(HAL_DMA_FULL_TRANSFER == CompleteLevel)
  {

    hdma->DmaBaseAddress->IFCR = (DMA_FLAG_TC1 << (hdma->ChannelIndex& 0x1CU));


    __HAL_UNLOCK(hdma);

    /* The selected Channelx EN bit is cleared (DMA is disabled and
    all transfers are complete) */
    hdma->State = HAL_DMA_STATE_READY;
  }
  else
  {

    hdma->DmaBaseAddress->IFCR = (DMA_FLAG_HT1 << (hdma->ChannelIndex & 0x1CU));
  }

  return HAL_OK;
}

/**
  * @brief  Handle DMA interrupt request.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval None
  */
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
  uint32_t flag_it = hdma->DmaBaseAddress->ISR;
  uint32_t source_it = hdma->Instance->CCR;


  if (((flag_it & (DMA_FLAG_HT1 << (hdma->ChannelIndex & 0x1CU))) != 0U) && ((source_it & DMA_IT_HT) != 0U))
  {

      if((hdma->Instance->CCR & DMA_CCR_CIRC) == 0U)
      {

        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
      }

      hdma->DmaBaseAddress->IFCR = DMA_ISR_HTIF1 << (hdma->ChannelIndex & 0x1CU);




      if(hdma->XferHalfCpltCallback != NULL)
      {

        hdma->XferHalfCpltCallback(hdma);
      }
  }


  else if (((flag_it & (DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1CU))) != 0U) && ((source_it & DMA_IT_TC) != 0U))
  {
    if((hdma->Instance->CCR & DMA_CCR_CIRC) == 0U)
    {



      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TE | DMA_IT_TC);


      hdma->State = HAL_DMA_STATE_READY;
    }

    hdma->DmaBaseAddress->IFCR = (DMA_ISR_TCIF1 << (hdma->ChannelIndex & 0x1CU));


    __HAL_UNLOCK(hdma);

    if(hdma->XferCpltCallback != NULL)
    {

      hdma->XferCpltCallback(hdma);
    }
  }


  else if (((flag_it & (DMA_FLAG_TE1 << (hdma->ChannelIndex & 0x1CU))) != 0U) && ((source_it & DMA_IT_TE) !=  0U))
  {



    __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));


    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));


    hdma->ErrorCode = HAL_DMA_ERROR_TE;


    hdma->State = HAL_DMA_STATE_READY;


    __HAL_UNLOCK(hdma);

    if (hdma->XferErrorCallback != NULL)
    {

      hdma->XferErrorCallback(hdma);
    }
  }
  else
  {

  }
  return;
}

/**
  * @brief  Register callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Channel.
  * @param  CallbackID           User Callback identifier
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @param  pCallback            pointer to private callbacsk function which has pointer to
  *                               a DMA_HandleTypeDef structure as parameter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)( DMA_HandleTypeDef * _hdma))
{
  HAL_StatusTypeDef status = HAL_OK;


  __HAL_LOCK(hdma);

  if(HAL_DMA_STATE_READY == hdma->State)
  {
    switch (CallbackID)
    {
     case  HAL_DMA_XFER_CPLT_CB_ID:
           hdma->XferCpltCallback = pCallback;
           break;

     case  HAL_DMA_XFER_HALFCPLT_CB_ID:
           hdma->XferHalfCpltCallback = pCallback;
           break;

     case  HAL_DMA_XFER_ERROR_CB_ID:
           hdma->XferErrorCallback = pCallback;
           break;

     case  HAL_DMA_XFER_ABORT_CB_ID:
           hdma->XferAbortCallback = pCallback;
           break;

     default:
           status = HAL_ERROR;
           break;
    }
  }
  else
  {
    status = HAL_ERROR;
  }


  __HAL_UNLOCK(hdma);

  return status;
}

/**
  * @brief  UnRegister callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Channel.
  * @param  CallbackID           User Callback identifier
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;


  __HAL_LOCK(hdma);

  if(HAL_DMA_STATE_READY == hdma->State)
  {
    switch (CallbackID)
    {
     case  HAL_DMA_XFER_CPLT_CB_ID:
           hdma->XferCpltCallback = NULL;
           break;

     case  HAL_DMA_XFER_HALFCPLT_CB_ID:
           hdma->XferHalfCpltCallback = NULL;
           break;

     case  HAL_DMA_XFER_ERROR_CB_ID:
           hdma->XferErrorCallback = NULL;
           break;

     case  HAL_DMA_XFER_ABORT_CB_ID:
           hdma->XferAbortCallback = NULL;
           break;

    case   HAL_DMA_XFER_ALL_CB_ID:
           hdma->XferCpltCallback = NULL;
           hdma->XferHalfCpltCallback = NULL;
           hdma->XferErrorCallback = NULL;
           hdma->XferAbortCallback = NULL;
           break;

    default:
           status = HAL_ERROR;
           break;
    }
  }
  else
  {
    status = HAL_ERROR;
  }


  __HAL_UNLOCK(hdma);

  return status;
}

/**
  * @}
  */



/** @defgroup DMA_Exported_Functions_Group3 Peripheral State and Errors functions
 *  @brief    Peripheral State and Errors functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the DMA handle state.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL state
  */
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma)
{

  return hdma->State;
}

/**
  * @brief  Return the DMA error code.
  * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Channel.
  * @retval DMA Error Code
  */
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *hdma)
{
  return hdma->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup DMA_Private_Functions
  * @{
  */

/**
  * @brief  Sets the DMA Transfer parameter.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval HAL status
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
#if defined(DMAMUX1)

  hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

  if(hdma->DMAmuxRequestGen != 0U)
  {

    hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
  }
#endif


  hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));


  hdma->Instance->CNDTR = DataLength;


  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {

    hdma->Instance->CPAR = DstAddress;


    hdma->Instance->CMAR = SrcAddress;
  }

  else
  {

    hdma->Instance->CPAR = SrcAddress;


    hdma->Instance->CMAR = DstAddress;
  }
}

#if defined(DMAMUX1)

/**
  * @brief  Updates the DMA handle with the DMAMUX  channel and status mask depending on channel number
  * @param  hdma        pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Channel.
  * @retval None
  */
static void DMA_CalcDMAMUXChannelBaseAndMask(DMA_HandleTypeDef *hdma)
{
  uint32_t channel_number;


  if ((uint32_t)hdma->Instance < (uint32_t)DMA2_Channel1)
  {

    hdma->DMAmuxChannel = (DMAMUX1_Channel0 + (hdma->ChannelIndex >> 2U));
  }
  else
  {

    hdma->DMAmuxChannel = (DMAMUX1_Channel7 + (hdma->ChannelIndex >> 2U));
  }

  channel_number = (((uint32_t)hdma->Instance & 0xFFU) - 8U) / 20U;
  hdma->DMAmuxChannelStatus = DMAMUX1_ChannelStatus;
  hdma->DMAmuxChannelStatusMask = 1UL << (channel_number & 0x1FU);
}

/**
  * @brief  Updates the DMA handle with the DMAMUX  request generator params
  * @param  hdma        pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Channel.
  * @retval None
  */

static void DMA_CalcDMAMUXRequestGenBaseAndMask(DMA_HandleTypeDef *hdma)
{
  uint32_t request =  hdma->Init.Request & DMAMUX_CxCR_DMAREQ_ID;


  hdma->DMAmuxRequestGen = (DMAMUX_RequestGen_TypeDef *)((uint32_t)(((uint32_t)DMAMUX1_RequestGenerator0) + ((request - 1U) * 4U)));

  hdma->DMAmuxRequestGenStatus = DMAMUX1_RequestGenStatus;


  hdma->DMAmuxRequestGenStatusMask = 1UL << ((request - 1U) & 0x3U);
}

#endif

/**
  * @}
  */

/**
  * @}
  */

#endif
/**
  * @}
  */

/**
  * @}
  */
