/**
  ******************************************************************************
  * @file    stm32l4xx_hal_dma_ex.h
  * @author  MCD Application Team
  * @brief   Header file of DMA HAL extension module.
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
  */


#ifndef STM32L4xx_HAL_DMA_EX_H
#define STM32L4xx_HAL_DMA_EX_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(DMAMUX1)


#include "stm32l4xx_hal_def.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @addtogroup DMAEx
  * @{
  */


/** @defgroup DMAEx_Exported_Types DMAEx Exported Types
  * @{
  */

/**
  * @brief  HAL DMA Synchro definition
  */


/**
  * @brief  HAL DMAMUX Synchronization configuration structure definition
  */
typedef struct
{
  uint32_t SyncSignalID;  /*!< Specifies the synchronization signal gating the DMA request in periodic mode.
                              This parameter can be a value of @ref DMAEx_DMAMUX_SyncSignalID_selection */

  uint32_t SyncPolarity;  /*!< Specifies the polarity of the signal on which the DMA request is synchronized.
                              This parameter can be a value of @ref DMAEx_DMAMUX_SyncPolarity_selection */

  FunctionalState SyncEnable;  /*!< Specifies if the synchronization shall be enabled or disabled
                                    This parameter can take the value ENABLE or DISABLE*/


  FunctionalState EventEnable;    /*!< Specifies if an event shall be generated once the RequestNumber is reached.
                                       This parameter can take the value ENABLE or DISABLE */

  uint32_t RequestNumber; /*!< Specifies the number of DMA request that will be authorized after a sync event
                               This parameter must be a number between Min_Data = 1 and Max_Data = 32 */


}HAL_DMA_MuxSyncConfigTypeDef;


/**
  * @brief  HAL DMAMUX request generator parameters structure definition
  */
typedef struct
{
 uint32_t SignalID;      /*!< Specifies the ID of the signal used for DMAMUX request generator
                              This parameter can be a value of @ref DMAEx_DMAMUX_SignalGeneratorID_selection */

  uint32_t Polarity;       /*!< Specifies the polarity of the signal on which the request is generated.
                             This parameter can be a value of @ref DMAEx_DMAMUX_RequestGeneneratorPolarity_selection */

  uint32_t RequestNumber;  /*!< Specifies the number of DMA request that will be generated after a signal event
                                This parameter must be a number between Min_Data = 1 and Max_Data = 32 */

}HAL_DMA_MuxRequestGeneratorConfigTypeDef;

/**
  * @}
  */


/** @defgroup DMAEx_Exported_Constants DMAEx Exported Constants
  * @{
  */

/** @defgroup DMAEx_DMAMUX_SyncSignalID_selection DMAMUX SyncSignalID selection
  * @{
  */
#define HAL_DMAMUX1_SYNC_EXTI0                0U
#define HAL_DMAMUX1_SYNC_EXTI1                1U
#define HAL_DMAMUX1_SYNC_EXTI2                2U
#define HAL_DMAMUX1_SYNC_EXTI3                3U
#define HAL_DMAMUX1_SYNC_EXTI4                4U
#define HAL_DMAMUX1_SYNC_EXTI5                5U
#define HAL_DMAMUX1_SYNC_EXTI6                6U
#define HAL_DMAMUX1_SYNC_EXTI7                7U
#define HAL_DMAMUX1_SYNC_EXTI8                8U
#define HAL_DMAMUX1_SYNC_EXTI9                9U
#define HAL_DMAMUX1_SYNC_EXTI10              10U
#define HAL_DMAMUX1_SYNC_EXTI11              11U
#define HAL_DMAMUX1_SYNC_EXTI12              12U
#define HAL_DMAMUX1_SYNC_EXTI13              13U
#define HAL_DMAMUX1_SYNC_EXTI14              14U
#define HAL_DMAMUX1_SYNC_EXTI15              15U
#define HAL_DMAMUX1_SYNC_DMAMUX1_CH0_EVT     16U
#define HAL_DMAMUX1_SYNC_DMAMUX1_CH1_EVT     17U
#define HAL_DMAMUX1_SYNC_DMAMUX1_CH2_EVT     18U
#define HAL_DMAMUX1_SYNC_DMAMUX1_CH3_EVT     19U
#define HAL_DMAMUX1_SYNC_LPTIM1_OUT          20U
#define HAL_DMAMUX1_SYNC_LPTIM2_OUT          21U
#if defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
#define HAL_DMAMUX1_SYNC_DSI_TE              22U
#define HAL_DMAMUX1_SYNC_DSI_EOT             23U
#endif
#define HAL_DMAMUX1_SYNC_DMA2D_EOT           24U
#define HAL_DMAMUX1_SYNC_LDTC_IT             25U

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_SyncPolarity_selection DMAMUX SyncPolarity selection
  * @{
  */
#define HAL_DMAMUX_SYNC_NO_EVENT                               0U
#define HAL_DMAMUX_SYNC_RISING                 DMAMUX_CxCR_SPOL_0
#define HAL_DMAMUX_SYNC_FALLING                DMAMUX_CxCR_SPOL_1
#define HAL_DMAMUX_SYNC_RISING_FALLING         DMAMUX_CxCR_SPOL

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_SignalGeneratorID_selection DMAMUX SignalGeneratorID selection
  * @{
  */

#define HAL_DMAMUX1_REQ_GEN_EXTI0                0U
#define HAL_DMAMUX1_REQ_GEN_EXTI1                1U
#define HAL_DMAMUX1_REQ_GEN_EXTI2                2U
#define HAL_DMAMUX1_REQ_GEN_EXTI3                3U
#define HAL_DMAMUX1_REQ_GEN_EXTI4                4U
#define HAL_DMAMUX1_REQ_GEN_EXTI5                5U
#define HAL_DMAMUX1_REQ_GEN_EXTI6                6U
#define HAL_DMAMUX1_REQ_GEN_EXTI7                7U
#define HAL_DMAMUX1_REQ_GEN_EXTI8                8U
#define HAL_DMAMUX1_REQ_GEN_EXTI9                9U
#define HAL_DMAMUX1_REQ_GEN_EXTI10              10U
#define HAL_DMAMUX1_REQ_GEN_EXTI11              11U
#define HAL_DMAMUX1_REQ_GEN_EXTI12              12U
#define HAL_DMAMUX1_REQ_GEN_EXTI13              13U
#define HAL_DMAMUX1_REQ_GEN_EXTI14              14U
#define HAL_DMAMUX1_REQ_GEN_EXTI15              15U
#define HAL_DMAMUX1_REQ_GEN_DMAMUX1_CH0_EVT     16U
#define HAL_DMAMUX1_REQ_GEN_DMAMUX1_CH1_EVT     17U
#define HAL_DMAMUX1_REQ_GEN_DMAMUX1_CH2_EVT     18U
#define HAL_DMAMUX1_REQ_GEN_DMAMUX1_CH3_EVT     19U
#define HAL_DMAMUX1_REQ_GEN_LPTIM1_OUT          20U
#define HAL_DMAMUX1_REQ_GEN_LPTIM2_OUT          21U
#if defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
#define HAL_DMAMUX1_REQ_GEN_DSI_TE              22U
#define HAL_DMAMUX1_REQ_GEN_DSI_EOT             23U
#endif
#define HAL_DMAMUX1_REQ_GEN_DMA2D_EOT           24U
#define HAL_DMAMUX1_REQ_GEN_LTDC_IT             25U

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_RequestGeneneratorPolarity_selection DMAMUX RequestGeneneratorPolarity selection
  * @{
  */
#define HAL_DMAMUX_REQ_GEN_NO_EVENT       0U
#define HAL_DMAMUX_REQ_GEN_RISING         DMAMUX_RGxCR_GPOL_0
#define HAL_DMAMUX_REQ_GEN_FALLING        DMAMUX_RGxCR_GPOL_1
#define HAL_DMAMUX_REQ_GEN_RISING_FALLING DMAMUX_RGxCR_GPOL

/**
  * @}
  */

/**
  * @}
  */




/** @addtogroup DMAEx_Exported_Functions
  * @{
  */


/** @addtogroup DMAEx_Exported_Functions_Group1
  * @{
  */


HAL_StatusTypeDef HAL_DMAEx_ConfigMuxRequestGenerator (DMA_HandleTypeDef *hdma,
             HAL_DMA_MuxRequestGeneratorConfigTypeDef *pRequestGeneratorConfig);
HAL_StatusTypeDef HAL_DMAEx_EnableMuxRequestGenerator (DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMAEx_DisableMuxRequestGenerator (DMA_HandleTypeDef *hdma);



HAL_StatusTypeDef HAL_DMAEx_ConfigMuxSync(DMA_HandleTypeDef *hdma, HAL_DMA_MuxSyncConfigTypeDef *pSyncConfig);


void HAL_DMAEx_MUX_IRQHandler(DMA_HandleTypeDef *hdma);

/**
  * @}
  */

/**
  * @}
  */




/** @defgroup DMAEx_Private_Macros DMAEx Private Macros
  * @brief    DMAEx private macros
  * @{
  */

#define IS_DMAMUX_SYNC_SIGNAL_ID(SIGNAL_ID) ((SIGNAL_ID) <= HAL_DMAMUX1_SYNC_LDTC_IT)

#define IS_DMAMUX_SYNC_REQUEST_NUMBER(REQUEST_NUMBER) (((REQUEST_NUMBER) > 0U) && ((REQUEST_NUMBER) <= 32U))

#define IS_DMAMUX_SYNC_POLARITY(POLARITY) (((POLARITY) == HAL_DMAMUX_SYNC_NO_EVENT) || \
                                           ((POLARITY) == HAL_DMAMUX_SYNC_RISING)   || \
                                           ((POLARITY) == HAL_DMAMUX_SYNC_FALLING)  || \
                                           ((POLARITY) == HAL_DMAMUX_SYNC_RISING_FALLING))

#define IS_DMAMUX_SYNC_STATE(SYNC) (((SYNC) == DISABLE)   || ((SYNC) == ENABLE))

#define IS_DMAMUX_SYNC_EVENT(EVENT) (((EVENT) == DISABLE)   || \
                                     ((EVENT) == ENABLE))

#define IS_DMAMUX_REQUEST_GEN_SIGNAL_ID(SIGNAL_ID) ((SIGNAL_ID) <= HAL_DMAMUX1_REQ_GEN_LTDC_IT)

#define IS_DMAMUX_REQUEST_GEN_REQUEST_NUMBER(REQUEST_NUMBER) (((REQUEST_NUMBER) > 0U) && ((REQUEST_NUMBER) <= 32U))

#define IS_DMAMUX_REQUEST_GEN_POLARITY(POLARITY) (((POLARITY) == HAL_DMAMUX_REQ_GEN_NO_EVENT)   || \
                                               ((POLARITY) == HAL_DMAMUX_REQ_GEN_RISING)  || \
                                               ((POLARITY) == HAL_DMAMUX_REQ_GEN_FALLING) || \
                                               ((POLARITY) == HAL_DMAMUX_REQ_GEN_RISING_FALLING))

/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

#endif

#ifdef __cplusplus
}
#endif

#endif
