/**
  ******************************************************************************
  * @file    stm32l4xx_hal_dma_ex.c
  * @author  MCD Application Team
  * @brief   DMA Extension HAL module driver
  *         This file provides firmware functions to manage the following
  *         functionalities of the DMA Extension peripheral:
  *           + Extended features functions
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
  The DMA Extension HAL driver can be used as follows:

   (+) Configure the DMA_MUX Synchronization Block using HAL_DMAEx_ConfigMuxSync function.
   (+) Configure the DMA_MUX Request Generator Block using HAL_DMAEx_ConfigMuxRequestGenerator function.
       Functions HAL_DMAEx_EnableMuxRequestGenerator and HAL_DMAEx_DisableMuxRequestGenerator can then be used
       to respectively enable/disable the request generator.

   (+) To handle the DMAMUX Interrupts, the function  HAL_DMAEx_MUX_IRQHandler should be called from
       the DMAMUX IRQ handler i.e DMAMUX1_OVR_IRQHandler.
       As only one interrupt line is available for all DMAMUX channels and request generators , HAL_DMAEx_MUX_IRQHandler should be
       called with, as parameter, the appropriate DMA handle as many as used DMAs in the user project
      (exception done if a given DMA is not using the DMAMUX SYNC block neither a request generator)

     -@-  In Memory-to-Memory transfer mode, Multi (Double) Buffer mode is not allowed.
     -@-  When Multi (Double) Buffer mode is enabled, the transfer is circular by default.
     -@-  In Multi (Double) buffer mode, it is possible to update the base address for
          the AHB memory port on the fly (DMA_CM0ARx or DMA_CM1ARx) when the channel is enabled.


  @endverbatim
  ******************************************************************************
  */


#include "stm32l4xx_hal.h"

#if defined(DMAMUX1)

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup DMAEx DMAEx
  * @brief DMA Extended HAL module driver
  * @{
  */

#ifdef HAL_DMA_MODULE_ENABLED










/** @defgroup DMAEx_Exported_Functions DMAEx Exported Functions
  * @{
  */

/** @defgroup DMAEx_Exported_Functions_Group1 DMAEx Extended features functions
 *  @brief   Extended features functions
 *
@verbatim
 ===============================================================================
                #####  Extended features functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:

    (+) Configure the DMAMUX Synchronization Block using HAL_DMAEx_ConfigMuxSync function.
    (+) Configure the DMAMUX Request Generator Block using HAL_DMAEx_ConfigMuxRequestGenerator function.
       Functions HAL_DMAEx_EnableMuxRequestGenerator and HAL_DMAEx_DisableMuxRequestGenerator can then be used
       to respectively enable/disable the request generator.

@endverbatim
  * @{
  */


/**
  * @brief  Configure the DMAMUX synchronization parameters for a given DMA channel (instance).
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA channel.
  * @param  pSyncConfig : pointer to HAL_DMA_MuxSyncConfigTypeDef : contains the DMAMUX synchronization parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxSync(DMA_HandleTypeDef *hdma, HAL_DMA_MuxSyncConfigTypeDef *pSyncConfig)
{

  assert_param(IS_DMA_ALL_INSTANCE(hdma->Instance));

  assert_param(IS_DMAMUX_SYNC_SIGNAL_ID(pSyncConfig->SyncSignalID));

  assert_param(IS_DMAMUX_SYNC_POLARITY(pSyncConfig-> SyncPolarity));
  assert_param(IS_DMAMUX_SYNC_STATE(pSyncConfig->SyncEnable));
  assert_param(IS_DMAMUX_SYNC_EVENT(pSyncConfig->EventEnable));
  assert_param(IS_DMAMUX_SYNC_REQUEST_NUMBER(pSyncConfig->RequestNumber));


  if(hdma->State == HAL_DMA_STATE_READY)
  {

    __HAL_LOCK(hdma);


    MODIFY_REG( hdma->DMAmuxChannel->CCR, \
               (~DMAMUX_CxCR_DMAREQ_ID) , \
               ((pSyncConfig->SyncSignalID) << DMAMUX_CxCR_SYNC_ID_Pos) | ((pSyncConfig->RequestNumber - 1U) << DMAMUX_CxCR_NBREQ_Pos) | \
               pSyncConfig->SyncPolarity | ((uint32_t)pSyncConfig->SyncEnable << DMAMUX_CxCR_SE_Pos) | \
                 ((uint32_t)pSyncConfig->EventEnable << DMAMUX_CxCR_EGE_Pos));


    __HAL_UNLOCK(hdma);

    return HAL_OK;
  }
  else
  {

    return HAL_ERROR;
  }
}

/**
  * @brief  Configure the DMAMUX request generator block used by the given DMA channel (instance).
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA channel.
  * @param  pRequestGeneratorConfig : pointer to HAL_DMA_MuxRequestGeneratorConfigTypeDef :
  *         contains the request generator parameters.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxRequestGenerator (DMA_HandleTypeDef *hdma, HAL_DMA_MuxRequestGeneratorConfigTypeDef *pRequestGeneratorConfig)
{

  assert_param(IS_DMA_ALL_INSTANCE(hdma->Instance));

  assert_param(IS_DMAMUX_REQUEST_GEN_SIGNAL_ID(pRequestGeneratorConfig->SignalID));

  assert_param(IS_DMAMUX_REQUEST_GEN_POLARITY(pRequestGeneratorConfig->Polarity));
  assert_param(IS_DMAMUX_REQUEST_GEN_REQUEST_NUMBER(pRequestGeneratorConfig->RequestNumber));

  /* check if the DMA state is ready
     and DMA is using a DMAMUX request generator block
  */
  if((hdma->State == HAL_DMA_STATE_READY) && (hdma->DMAmuxRequestGen != 0U))
  {

    __HAL_LOCK(hdma);


    hdma->DMAmuxRequestGen->RGCR = pRequestGeneratorConfig->SignalID | \
                                  ((pRequestGeneratorConfig->RequestNumber - 1U) << DMAMUX_RGxCR_GNBREQ_Pos)| \
                                  pRequestGeneratorConfig->Polarity;

   __HAL_UNLOCK(hdma);

   return HAL_OK;
 }
 else
 {
   return HAL_ERROR;
 }
}

/**
  * @brief  Enable the DMAMUX request generator block used by the given DMA channel (instance).
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_EnableMuxRequestGenerator (DMA_HandleTypeDef *hdma)
{

  assert_param(IS_DMA_ALL_INSTANCE(hdma->Instance));

  /* check if the DMA state is ready
     and DMA is using a DMAMUX request generator block
  */
  if((hdma->State != HAL_DMA_STATE_RESET) && (hdma->DMAmuxRequestGen != 0))
  {


    hdma->DMAmuxRequestGen->RGCR |= DMAMUX_RGxCR_GE;

   return HAL_OK;
 }
 else
 {
   return HAL_ERROR;
 }
}

/**
  * @brief  Disable the DMAMUX request generator block used by the given DMA channel (instance).
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_DisableMuxRequestGenerator (DMA_HandleTypeDef *hdma)
{

  assert_param(IS_DMA_ALL_INSTANCE(hdma->Instance));

  /* check if the DMA state is ready
     and DMA is using a DMAMUX request generator block
  */
  if((hdma->State != HAL_DMA_STATE_RESET) && (hdma->DMAmuxRequestGen != 0))
  {


    hdma->DMAmuxRequestGen->RGCR &= ~DMAMUX_RGxCR_GE;

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Handles DMAMUX interrupt request.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA channel.
  * @retval None
  */
void HAL_DMAEx_MUX_IRQHandler(DMA_HandleTypeDef *hdma)
{

  if((hdma->DMAmuxChannelStatus->CSR & hdma->DMAmuxChannelStatusMask) != 0U)
  {

    hdma->DMAmuxChannel->CCR &= ~DMAMUX_CxCR_SOIE;


    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;


    hdma->ErrorCode |= HAL_DMA_ERROR_SYNC;

    if(hdma->XferErrorCallback != NULL)
    {

      hdma->XferErrorCallback(hdma);
    }
  }

  if(hdma->DMAmuxRequestGen != 0)
  {

    if((hdma->DMAmuxRequestGenStatus->RGSR & hdma->DMAmuxRequestGenStatusMask) != 0U)
    {

      hdma->DMAmuxRequestGen->RGCR &= ~DMAMUX_RGxCR_OIE;


      hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;


      hdma->ErrorCode |= HAL_DMA_ERROR_REQGEN;

      if(hdma->XferErrorCallback != NULL)
      {

        hdma->XferErrorCallback(hdma);
      }
    }
  }
}

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

#endif
