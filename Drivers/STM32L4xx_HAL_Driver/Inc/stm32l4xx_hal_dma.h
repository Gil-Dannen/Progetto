/**
  ******************************************************************************
  * @file    stm32l4xx_hal_dma.h
  * @author  MCD Application Team
  * @brief   Header file of DMA HAL module.
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


#ifndef STM32L4xx_HAL_DMA_H
#define STM32L4xx_HAL_DMA_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32l4xx_hal_def.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @addtogroup DMA
  * @{
  */


/** @defgroup DMA_Exported_Types DMA Exported Types
  * @{
  */

/**
  * @brief  DMA Configuration Structure definition
  */
typedef struct
{
  uint32_t Request;                   /*!< Specifies the request selected for the specified channel.
                                           This parameter can be a value of @ref DMA_request */

  uint32_t Direction;                 /*!< Specifies if the data will be transferred from memory to peripheral,
                                           from memory to memory or from peripheral to memory.
                                           This parameter can be a value of @ref DMA_Data_transfer_direction */

  uint32_t PeriphInc;                 /*!< Specifies whether the Peripheral address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Peripheral_incremented_mode */

  uint32_t MemInc;                    /*!< Specifies whether the memory address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Memory_incremented_mode */

  uint32_t PeriphDataAlignment;       /*!< Specifies the Peripheral data width.
                                           This parameter can be a value of @ref DMA_Peripheral_data_size */

  uint32_t MemDataAlignment;          /*!< Specifies the Memory data width.
                                           This parameter can be a value of @ref DMA_Memory_data_size */

  uint32_t Mode;                      /*!< Specifies the operation mode of the DMAy Channelx.
                                           This parameter can be a value of @ref DMA_mode
                                           @note The circular buffer mode cannot be used if the memory-to-memory
                                                 data transfer is configured on the selected Channel */

  uint32_t Priority;                  /*!< Specifies the software priority for the DMAy Channelx.
                                           This parameter can be a value of @ref DMA_Priority_level */
} DMA_InitTypeDef;

/**
  * @brief  HAL DMA State structures definition
  */
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,
  HAL_DMA_STATE_READY             = 0x01U,
  HAL_DMA_STATE_BUSY              = 0x02U,
  HAL_DMA_STATE_TIMEOUT           = 0x03U,
}HAL_DMA_StateTypeDef;

/**
  * @brief  HAL DMA Error Code structure definition
  */
typedef enum
{
  HAL_DMA_FULL_TRANSFER      = 0x00U,
  HAL_DMA_HALF_TRANSFER      = 0x01U
}HAL_DMA_LevelCompleteTypeDef;


/**
  * @brief  HAL DMA Callback ID structure definition
  */
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID          = 0x00U,
  HAL_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,
  HAL_DMA_XFER_ERROR_CB_ID         = 0x02U,
  HAL_DMA_XFER_ABORT_CB_ID         = 0x03U,
  HAL_DMA_XFER_ALL_CB_ID           = 0x04U
}HAL_DMA_CallbackIDTypeDef;

/**
  * @brief  DMA handle Structure definition
  */
typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef    *Instance;

  DMA_InitTypeDef       Init;

  HAL_LockTypeDef       Lock;

  __IO HAL_DMA_StateTypeDef  State;

  void                  *Parent;

  void                  (* XferCpltCallback)(struct __DMA_HandleTypeDef * hdma);

  void                  (* XferHalfCpltCallback)(struct __DMA_HandleTypeDef * hdma);

  void                  (* XferErrorCallback)(struct __DMA_HandleTypeDef * hdma);

  void                  (* XferAbortCallback)(struct __DMA_HandleTypeDef * hdma);

  __IO uint32_t         ErrorCode;

  DMA_TypeDef           *DmaBaseAddress;

  uint32_t              ChannelIndex;

#if defined(DMAMUX1)
  DMAMUX_Channel_TypeDef           *DMAmuxChannel;

  DMAMUX_ChannelStatus_TypeDef     *DMAmuxChannelStatus;

  uint32_t                         DMAmuxChannelStatusMask;

  DMAMUX_RequestGen_TypeDef        *DMAmuxRequestGen;

  DMAMUX_RequestGenStatus_TypeDef  *DMAmuxRequestGenStatus;

  uint32_t                         DMAmuxRequestGenStatusMask;

#endif

}DMA_HandleTypeDef;
/**
  * @}
  */



/** @defgroup DMA_Exported_Constants DMA Exported Constants
  * @{
  */

/** @defgroup DMA_Error_Code DMA Error Code
  * @{
  */
#define HAL_DMA_ERROR_NONE                 0x00000000U
#define HAL_DMA_ERROR_TE                   0x00000001U
#define HAL_DMA_ERROR_NO_XFER              0x00000004U
#define HAL_DMA_ERROR_TIMEOUT              0x00000020U
#define HAL_DMA_ERROR_NOT_SUPPORTED        0x00000100U
#define HAL_DMA_ERROR_SYNC                 0x00000200U
#define HAL_DMA_ERROR_REQGEN               0x00000400U

/**
  * @}
  */

/** @defgroup DMA_request DMA request
  * @{
  */
#if !defined (DMAMUX1)

#define DMA_REQUEST_0                     0U
#define DMA_REQUEST_1                     1U
#define DMA_REQUEST_2                     2U
#define DMA_REQUEST_3                     3U
#define DMA_REQUEST_4                     4U
#define DMA_REQUEST_5                     5U
#define DMA_REQUEST_6                     6U
#define DMA_REQUEST_7                     7U

#endif

#if defined(DMAMUX1)

#define DMA_REQUEST_MEM2MEM                 0U

#define DMA_REQUEST_GENERATOR0              1U
#define DMA_REQUEST_GENERATOR1              2U
#define DMA_REQUEST_GENERATOR2              3U
#define DMA_REQUEST_GENERATOR3              4U

#define DMA_REQUEST_ADC1                    5U

#if defined (STM32L4P5xx) || defined (STM32L4Q5xx)

#define DMA_REQUEST_ADC2                    6U

#define DMA_REQUEST_DAC1_CH1                7U
#define DMA_REQUEST_DAC1_CH2                8U

#define DMA_REQUEST_TIM6_UP                 9U
#define DMA_REQUEST_TIM7_UP                10U

#define DMA_REQUEST_SPI1_RX                11U
#define DMA_REQUEST_SPI1_TX                12U
#define DMA_REQUEST_SPI2_RX                13U
#define DMA_REQUEST_SPI2_TX                14U
#define DMA_REQUEST_SPI3_RX                15U
#define DMA_REQUEST_SPI3_TX                16U

#define DMA_REQUEST_I2C1_RX                17U
#define DMA_REQUEST_I2C1_TX                18U
#define DMA_REQUEST_I2C2_RX                19U
#define DMA_REQUEST_I2C2_TX                20U
#define DMA_REQUEST_I2C3_RX                21U
#define DMA_REQUEST_I2C3_TX                22U
#define DMA_REQUEST_I2C4_RX                23U
#define DMA_REQUEST_I2C4_TX                24U

#define DMA_REQUEST_USART1_RX              25U
#define DMA_REQUEST_USART1_TX              26U
#define DMA_REQUEST_USART2_RX              27U
#define DMA_REQUEST_USART2_TX              28U
#define DMA_REQUEST_USART3_RX              29U
#define DMA_REQUEST_USART3_TX              30U

#define DMA_REQUEST_UART4_RX               31U
#define DMA_REQUEST_UART4_TX               32U
#define DMA_REQUEST_UART5_RX               33U
#define DMA_REQUEST_UART5_TX               34U

#define DMA_REQUEST_LPUART1_RX             35U
#define DMA_REQUEST_LPUART1_TX             36U

#define DMA_REQUEST_SAI1_A                 37U
#define DMA_REQUEST_SAI1_B                 38U
#define DMA_REQUEST_SAI2_A                 39U
#define DMA_REQUEST_SAI2_B                 40U

#define DMA_REQUEST_OCTOSPI1               41U
#define DMA_REQUEST_OCTOSPI2               42U

#define DMA_REQUEST_TIM1_CH1               43U
#define DMA_REQUEST_TIM1_CH2               44U
#define DMA_REQUEST_TIM1_CH3               45U
#define DMA_REQUEST_TIM1_CH4               46U
#define DMA_REQUEST_TIM1_UP                47U
#define DMA_REQUEST_TIM1_TRIG              48U
#define DMA_REQUEST_TIM1_COM               49U

#define DMA_REQUEST_TIM8_CH1               50U
#define DMA_REQUEST_TIM8_CH2               51U
#define DMA_REQUEST_TIM8_CH3               52U
#define DMA_REQUEST_TIM8_CH4               53U
#define DMA_REQUEST_TIM8_UP                54U
#define DMA_REQUEST_TIM8_TRIG              55U
#define DMA_REQUEST_TIM8_COM               56U

#define DMA_REQUEST_TIM2_CH1               57U
#define DMA_REQUEST_TIM2_CH2               58U
#define DMA_REQUEST_TIM2_CH3               59U
#define DMA_REQUEST_TIM2_CH4               60U
#define DMA_REQUEST_TIM2_UP                61U

#define DMA_REQUEST_TIM3_CH1               62U
#define DMA_REQUEST_TIM3_CH2               63U
#define DMA_REQUEST_TIM3_CH3               64U
#define DMA_REQUEST_TIM3_CH4               65U
#define DMA_REQUEST_TIM3_UP                66U
#define DMA_REQUEST_TIM3_TRIG              67U

#define DMA_REQUEST_TIM4_CH1               68U
#define DMA_REQUEST_TIM4_CH2               69U
#define DMA_REQUEST_TIM4_CH3               70U
#define DMA_REQUEST_TIM4_CH4               71U
#define DMA_REQUEST_TIM4_UP                72U

#define DMA_REQUEST_TIM5_CH1               73U
#define DMA_REQUEST_TIM5_CH2               74U
#define DMA_REQUEST_TIM5_CH3               75U
#define DMA_REQUEST_TIM5_CH4               76U
#define DMA_REQUEST_TIM5_UP                77U
#define DMA_REQUEST_TIM5_TRIG              78U

#define DMA_REQUEST_TIM15_CH1              79U
#define DMA_REQUEST_TIM15_UP               80U
#define DMA_REQUEST_TIM15_TRIG             81U
#define DMA_REQUEST_TIM15_COM              82U

#define DMA_REQUEST_TIM16_CH1              83U
#define DMA_REQUEST_TIM16_UP               84U
#define DMA_REQUEST_TIM17_CH1              85U
#define DMA_REQUEST_TIM17_UP               86U

#define DMA_REQUEST_DFSDM1_FLT0            87U
#define DMA_REQUEST_DFSDM1_FLT1            88U

#define DMA_REQUEST_DCMI                   91U
#define DMA_REQUEST_DCMI_PSSI              91U

#define DMA_REQUEST_AES_IN                 92U
#define DMA_REQUEST_AES_OUT                93U

#define DMA_REQUEST_HASH_IN                94U

#else

#define DMA_REQUEST_DAC1_CH1                6U
#define DMA_REQUEST_DAC1_CH2                7U

#define DMA_REQUEST_TIM6_UP                 8U
#define DMA_REQUEST_TIM7_UP                 9U

#define DMA_REQUEST_SPI1_RX                10U
#define DMA_REQUEST_SPI1_TX                11U
#define DMA_REQUEST_SPI2_RX                12U
#define DMA_REQUEST_SPI2_TX                13U
#define DMA_REQUEST_SPI3_RX                14U
#define DMA_REQUEST_SPI3_TX                15U

#define DMA_REQUEST_I2C1_RX                16U
#define DMA_REQUEST_I2C1_TX                17U
#define DMA_REQUEST_I2C2_RX                18U
#define DMA_REQUEST_I2C2_TX                19U
#define DMA_REQUEST_I2C3_RX                20U
#define DMA_REQUEST_I2C3_TX                21U
#define DMA_REQUEST_I2C4_RX                22U
#define DMA_REQUEST_I2C4_TX                23U

#define DMA_REQUEST_USART1_RX              24U
#define DMA_REQUEST_USART1_TX              25U
#define DMA_REQUEST_USART2_RX              26U
#define DMA_REQUEST_USART2_TX              27U
#define DMA_REQUEST_USART3_RX              28U
#define DMA_REQUEST_USART3_TX              29U

#define DMA_REQUEST_UART4_RX               30U
#define DMA_REQUEST_UART4_TX               31U
#define DMA_REQUEST_UART5_RX               32U
#define DMA_REQUEST_UART5_TX               33U

#define DMA_REQUEST_LPUART1_RX             34U
#define DMA_REQUEST_LPUART1_TX             35U

#define DMA_REQUEST_SAI1_A                 36U
#define DMA_REQUEST_SAI1_B                 37U
#define DMA_REQUEST_SAI2_A                 38U
#define DMA_REQUEST_SAI2_B                 39U

#define DMA_REQUEST_OCTOSPI1               40U
#define DMA_REQUEST_OCTOSPI2               41U

#define DMA_REQUEST_TIM1_CH1               42U
#define DMA_REQUEST_TIM1_CH2               43U
#define DMA_REQUEST_TIM1_CH3               44U
#define DMA_REQUEST_TIM1_CH4               45U
#define DMA_REQUEST_TIM1_UP                46U
#define DMA_REQUEST_TIM1_TRIG              47U
#define DMA_REQUEST_TIM1_COM               48U

#define DMA_REQUEST_TIM8_CH1               49U
#define DMA_REQUEST_TIM8_CH2               50U
#define DMA_REQUEST_TIM8_CH3               51U
#define DMA_REQUEST_TIM8_CH4               52U
#define DMA_REQUEST_TIM8_UP                53U
#define DMA_REQUEST_TIM8_TRIG              54U
#define DMA_REQUEST_TIM8_COM               55U

#define DMA_REQUEST_TIM2_CH1               56U
#define DMA_REQUEST_TIM2_CH2               57U
#define DMA_REQUEST_TIM2_CH3               58U
#define DMA_REQUEST_TIM2_CH4               59U
#define DMA_REQUEST_TIM2_UP                60U

#define DMA_REQUEST_TIM3_CH1               61U
#define DMA_REQUEST_TIM3_CH2               62U
#define DMA_REQUEST_TIM3_CH3               63U
#define DMA_REQUEST_TIM3_CH4               64U
#define DMA_REQUEST_TIM3_UP                65U
#define DMA_REQUEST_TIM3_TRIG              66U

#define DMA_REQUEST_TIM4_CH1               67U
#define DMA_REQUEST_TIM4_CH2               68U
#define DMA_REQUEST_TIM4_CH3               69U
#define DMA_REQUEST_TIM4_CH4               70U
#define DMA_REQUEST_TIM4_UP                71U

#define DMA_REQUEST_TIM5_CH1               72U
#define DMA_REQUEST_TIM5_CH2               73U
#define DMA_REQUEST_TIM5_CH3               74U
#define DMA_REQUEST_TIM5_CH4               75U
#define DMA_REQUEST_TIM5_UP                76U
#define DMA_REQUEST_TIM5_TRIG              77U

#define DMA_REQUEST_TIM15_CH1              78U
#define DMA_REQUEST_TIM15_UP               79U
#define DMA_REQUEST_TIM15_TRIG             80U
#define DMA_REQUEST_TIM15_COM              81U

#define DMA_REQUEST_TIM16_CH1              82U
#define DMA_REQUEST_TIM16_UP               83U
#define DMA_REQUEST_TIM17_CH1              84U
#define DMA_REQUEST_TIM17_UP               85U

#define DMA_REQUEST_DFSDM1_FLT0            86U
#define DMA_REQUEST_DFSDM1_FLT1            87U
#define DMA_REQUEST_DFSDM1_FLT2            88U
#define DMA_REQUEST_DFSDM1_FLT3            89U

#define DMA_REQUEST_DCMI                   90U

#define DMA_REQUEST_AES_IN                 91U
#define DMA_REQUEST_AES_OUT                92U

#define DMA_REQUEST_HASH_IN                93U
#endif

#endif

/**
  * @}
  */

/** @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @{
  */
#define DMA_PERIPH_TO_MEMORY         0x00000000U
#define DMA_MEMORY_TO_PERIPH         DMA_CCR_DIR
#define DMA_MEMORY_TO_MEMORY         DMA_CCR_MEM2MEM
/**
  * @}
  */

/** @defgroup DMA_Peripheral_incremented_mode DMA Peripheral incremented mode
  * @{
  */
#define DMA_PINC_ENABLE              DMA_CCR_PINC
#define DMA_PINC_DISABLE             0x00000000U
/**
  * @}
  */

/** @defgroup DMA_Memory_incremented_mode DMA Memory incremented mode
  * @{
  */
#define DMA_MINC_ENABLE              DMA_CCR_MINC
#define DMA_MINC_DISABLE             0x00000000U
/**
  * @}
  */

/** @defgroup DMA_Peripheral_data_size DMA Peripheral data size
  * @{
  */
#define DMA_PDATAALIGN_BYTE          0x00000000U
#define DMA_PDATAALIGN_HALFWORD      DMA_CCR_PSIZE_0
#define DMA_PDATAALIGN_WORD          DMA_CCR_PSIZE_1
/**
  * @}
  */

/** @defgroup DMA_Memory_data_size DMA Memory data size
  * @{
  */
#define DMA_MDATAALIGN_BYTE          0x00000000U
#define DMA_MDATAALIGN_HALFWORD      DMA_CCR_MSIZE_0
#define DMA_MDATAALIGN_WORD          DMA_CCR_MSIZE_1
/**
  * @}
  */

/** @defgroup DMA_mode DMA mode
  * @{
  */
#define DMA_NORMAL                   0x00000000U
#define DMA_CIRCULAR                 DMA_CCR_CIRC
/**
  * @}
  */

/** @defgroup DMA_Priority_level DMA Priority level
  * @{
  */
#define DMA_PRIORITY_LOW             0x00000000U
#define DMA_PRIORITY_MEDIUM          DMA_CCR_PL_0
#define DMA_PRIORITY_HIGH            DMA_CCR_PL_1
#define DMA_PRIORITY_VERY_HIGH       DMA_CCR_PL
/**
  * @}
  */


/** @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @{
  */
#define DMA_IT_TC                         DMA_CCR_TCIE
#define DMA_IT_HT                         DMA_CCR_HTIE
#define DMA_IT_TE                         DMA_CCR_TEIE
/**
  * @}
  */

/** @defgroup DMA_flag_definitions DMA flag definitions
  * @{
  */
#define DMA_FLAG_GL1                      DMA_ISR_GIF1
#define DMA_FLAG_TC1                      DMA_ISR_TCIF1
#define DMA_FLAG_HT1                      DMA_ISR_HTIF1
#define DMA_FLAG_TE1                      DMA_ISR_TEIF1
#define DMA_FLAG_GL2                      DMA_ISR_GIF2
#define DMA_FLAG_TC2                      DMA_ISR_TCIF2
#define DMA_FLAG_HT2                      DMA_ISR_HTIF2
#define DMA_FLAG_TE2                      DMA_ISR_TEIF2
#define DMA_FLAG_GL3                      DMA_ISR_GIF3
#define DMA_FLAG_TC3                      DMA_ISR_TCIF3
#define DMA_FLAG_HT3                      DMA_ISR_HTIF3
#define DMA_FLAG_TE3                      DMA_ISR_TEIF3
#define DMA_FLAG_GL4                      DMA_ISR_GIF4
#define DMA_FLAG_TC4                      DMA_ISR_TCIF4
#define DMA_FLAG_HT4                      DMA_ISR_HTIF4
#define DMA_FLAG_TE4                      DMA_ISR_TEIF4
#define DMA_FLAG_GL5                      DMA_ISR_GIF5
#define DMA_FLAG_TC5                      DMA_ISR_TCIF5
#define DMA_FLAG_HT5                      DMA_ISR_HTIF5
#define DMA_FLAG_TE5                      DMA_ISR_TEIF5
#define DMA_FLAG_GL6                      DMA_ISR_GIF6
#define DMA_FLAG_TC6                      DMA_ISR_TCIF6
#define DMA_FLAG_HT6                      DMA_ISR_HTIF6
#define DMA_FLAG_TE6                      DMA_ISR_TEIF6
#define DMA_FLAG_GL7                      DMA_ISR_GIF7
#define DMA_FLAG_TC7                      DMA_ISR_TCIF7
#define DMA_FLAG_HT7                      DMA_ISR_HTIF7
#define DMA_FLAG_TE7                      DMA_ISR_TEIF7
/**
  * @}
  */

/**
  * @}
  */


/** @defgroup DMA_Exported_Macros DMA Exported Macros
  * @{
  */

/** @brief  Reset DMA handle state.
  * @param  __HANDLE__ DMA handle
  * @retval None
  */
#define __HAL_DMA_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_DMA_STATE_RESET)

/**
  * @brief  Enable the specified DMA Channel.
  * @param  __HANDLE__ DMA handle
  * @retval None
  */
#define __HAL_DMA_ENABLE(__HANDLE__)        ((__HANDLE__)->Instance->CCR |=  DMA_CCR_EN)

/**
  * @brief  Disable the specified DMA Channel.
  * @param  __HANDLE__ DMA handle
  * @retval None
  */
#define __HAL_DMA_DISABLE(__HANDLE__)       ((__HANDLE__)->Instance->CCR &=  ~DMA_CCR_EN)




/**
  * @brief  Return the current DMA Channel transfer complete flag.
  * @param  __HANDLE__ DMA handle
  * @retval The specified transfer complete flag index.
  */

#define __HAL_DMA_GET_TC_FLAG_INDEX(__HANDLE__) \
(((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_TC1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_TC1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_TC2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_TC2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_TC3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_TC3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_TC4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_TC4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_TC5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel5))? DMA_FLAG_TC5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_TC6 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel6))? DMA_FLAG_TC6 :\
   DMA_FLAG_TC7)

/**
  * @brief  Return the current DMA Channel half transfer complete flag.
  * @param  __HANDLE__ DMA handle
  * @retval The specified half transfer complete flag index.
  */
#define __HAL_DMA_GET_HT_FLAG_INDEX(__HANDLE__)\
(((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_HT1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_HT1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_HT2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_HT2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_HT3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_HT3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_HT4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_HT4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_HT5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel5))? DMA_FLAG_HT5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_HT6 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel6))? DMA_FLAG_HT6 :\
   DMA_FLAG_HT7)

/**
  * @brief  Return the current DMA Channel transfer error flag.
  * @param  __HANDLE__ DMA handle
  * @retval The specified transfer error flag index.
  */
#define __HAL_DMA_GET_TE_FLAG_INDEX(__HANDLE__)\
(((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_TE1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_TE1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_TE2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_TE2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_TE3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_TE3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_TE4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_TE4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_TE5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel5))? DMA_FLAG_TE5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_TE6 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel6))? DMA_FLAG_TE6 :\
   DMA_FLAG_TE7)

/**
  * @brief  Return the current DMA Channel Global interrupt flag.
  * @param  __HANDLE__ DMA handle
  * @retval The specified transfer error flag index.
  */
#define __HAL_DMA_GET_GI_FLAG_INDEX(__HANDLE__)\
(((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_ISR_GIF1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_ISR_GIF1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_ISR_GIF2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_ISR_GIF2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_ISR_GIF3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_ISR_GIF3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_ISR_GIF4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_ISR_GIF4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_ISR_GIF5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel5))? DMA_ISR_GIF5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_ISR_GIF6 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel6))? DMA_ISR_GIF6 :\
   DMA_ISR_GIF7)

/**
  * @brief  Get the DMA Channel pending flags.
  * @param  __HANDLE__ DMA handle
  * @param  __FLAG__ Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCx:  Transfer complete flag
  *            @arg DMA_FLAG_HTx:  Half transfer complete flag
  *            @arg DMA_FLAG_TEx:  Transfer error flag
  *            @arg DMA_FLAG_GLx:  Global interrupt flag
  *         Where x can be from 1 to 7 to select the DMA Channel x flag.
  * @retval The state of FLAG (SET or RESET).
  */
#define __HAL_DMA_GET_FLAG(__HANDLE__, __FLAG__) (((uint32_t)((__HANDLE__)->Instance) > ((uint32_t)DMA1_Channel7))? \
 (DMA2->ISR & (__FLAG__)) : (DMA1->ISR & (__FLAG__)))

/**
  * @brief  Clear the DMA Channel pending flags.
  * @param  __HANDLE__ DMA handle
  * @param  __FLAG__ specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCx:  Transfer complete flag
  *            @arg DMA_FLAG_HTx:  Half transfer complete flag
  *            @arg DMA_FLAG_TEx:  Transfer error flag
  *            @arg DMA_FLAG_GLx:  Global interrupt flag
  *         Where x can be from 1 to 7 to select the DMA Channel x flag.
  * @retval None
  */
#define __HAL_DMA_CLEAR_FLAG(__HANDLE__, __FLAG__) (((uint32_t)((__HANDLE__)->Instance) > ((uint32_t)DMA1_Channel7))? \
 (DMA2->IFCR = (__FLAG__)) : (DMA1->IFCR = (__FLAG__)))

/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param __INTERRUPT__ specifies the DMA interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  * @retval None
  */
#define __HAL_DMA_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->CCR |= (__INTERRUPT__))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __INTERRUPT__ specifies the DMA interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  * @retval None
  */
#define __HAL_DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->CCR &= ~(__INTERRUPT__))

/**
  * @brief  Check whether the specified DMA Channel interrupt is enabled or not.
  * @param  __HANDLE__ DMA handle
  * @param  __INTERRUPT__ specifies the DMA interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  * @retval The state of DMA_IT (SET or RESET).
  */
#define __HAL_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  (((__HANDLE__)->Instance->CCR & (__INTERRUPT__)))

/**
  * @brief  Return the number of remaining data units in the current DMA Channel transfer.
  * @param  __HANDLE__ DMA handle
  * @retval The number of remaining data units in the current DMA Channel transfer.
  */
#define __HAL_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNDTR)

/**
  * @}
  */

#if defined(DMAMUX1)

#include "stm32l4xx_hal_dma_ex.h"
#endif



/** @addtogroup DMA_Exported_Functions
  * @{
  */

/** @addtogroup DMA_Exported_Functions_Group1
  * @{
  */

HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit (DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group2
  * @{
  */

HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)( DMA_HandleTypeDef * _hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group3
  * @{
  */

HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/**
  * @}
  */


/** @defgroup DMA_Private_Macros DMA Private Macros
  * @{
  */

#define IS_DMA_DIRECTION(DIRECTION) (((DIRECTION) == DMA_PERIPH_TO_MEMORY ) || \
                                     ((DIRECTION) == DMA_MEMORY_TO_PERIPH)  || \
                                     ((DIRECTION) == DMA_MEMORY_TO_MEMORY))

#define IS_DMA_BUFFER_SIZE(SIZE) (((SIZE) >= 0x1U) && ((SIZE) < 0x10000U))

#define IS_DMA_PERIPHERAL_INC_STATE(STATE) (((STATE) == DMA_PINC_ENABLE) || \
                                            ((STATE) == DMA_PINC_DISABLE))

#define IS_DMA_MEMORY_INC_STATE(STATE) (((STATE) == DMA_MINC_ENABLE)  || \
                                        ((STATE) == DMA_MINC_DISABLE))

#if !defined (DMAMUX1)

#define IS_DMA_ALL_REQUEST(REQUEST) (((REQUEST) == DMA_REQUEST_0) || \
                                     ((REQUEST) == DMA_REQUEST_1) || \
                                     ((REQUEST) == DMA_REQUEST_2) || \
                                     ((REQUEST) == DMA_REQUEST_3) || \
                                     ((REQUEST) == DMA_REQUEST_4) || \
                                     ((REQUEST) == DMA_REQUEST_5) || \
                                     ((REQUEST) == DMA_REQUEST_6) || \
                                     ((REQUEST) == DMA_REQUEST_7))
#endif

#if defined(DMAMUX1)

#define IS_DMA_ALL_REQUEST(REQUEST)((REQUEST) <= DMA_REQUEST_HASH_IN)

#endif

#define IS_DMA_PERIPHERAL_DATA_SIZE(SIZE) (((SIZE) == DMA_PDATAALIGN_BYTE)     || \
                                           ((SIZE) == DMA_PDATAALIGN_HALFWORD) || \
                                           ((SIZE) == DMA_PDATAALIGN_WORD))

#define IS_DMA_MEMORY_DATA_SIZE(SIZE) (((SIZE) == DMA_MDATAALIGN_BYTE)     || \
                                       ((SIZE) == DMA_MDATAALIGN_HALFWORD) || \
                                       ((SIZE) == DMA_MDATAALIGN_WORD ))

#define IS_DMA_MODE(MODE) (((MODE) == DMA_NORMAL )  || \
                           ((MODE) == DMA_CIRCULAR))

#define IS_DMA_PRIORITY(PRIORITY) (((PRIORITY) == DMA_PRIORITY_LOW )   || \
                                   ((PRIORITY) == DMA_PRIORITY_MEDIUM) || \
                                   ((PRIORITY) == DMA_PRIORITY_HIGH)   || \
                                   ((PRIORITY) == DMA_PRIORITY_VERY_HIGH))

/**
  * @}
  */



/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif
