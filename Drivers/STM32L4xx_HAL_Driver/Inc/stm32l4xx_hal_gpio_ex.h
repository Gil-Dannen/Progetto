/**
  ******************************************************************************
  * @file    stm32l4xx_hal_gpio_ex.h
  * @author  MCD Application Team
  * @brief   Header file of GPIO HAL Extended module.
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


#ifndef STM32L4xx_HAL_GPIO_EX_H
#define STM32L4xx_HAL_GPIO_EX_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32l4xx_hal_def.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup GPIOEx GPIOEx
  * @brief GPIO Extended HAL module driver
  * @{
  */



/** @defgroup GPIOEx_Exported_Constants GPIOEx Exported Constants
  * @{
  */

/** @defgroup GPIOEx_Alternate_function_selection GPIOEx Alternate function selection
  * @{
  */

#if defined(STM32L412xx) || defined(STM32L422xx)

/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)
#define GPIO_AF0_MCO           ((uint8_t)0x00)
#define GPIO_AF0_SWJ           ((uint8_t)0x00)
#define GPIO_AF0_TRACE         ((uint8_t)0x00)

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)
#define GPIO_AF1_TIM2          ((uint8_t)0x01)
#define GPIO_AF1_LPTIM1        ((uint8_t)0x01)
#define GPIO_AF1_IR            ((uint8_t)0x01)

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)
#define GPIO_AF2_TIM2          ((uint8_t)0x02)

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_USART2        ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP1    ((uint8_t)0x03)

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)
#define GPIO_AF4_I2C2          ((uint8_t)0x04)
#define GPIO_AF4_I2C3          ((uint8_t)0x04)

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)
#define GPIO_AF5_SPI2          ((uint8_t)0x05)

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_COMP1         ((uint8_t)0x06)

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)
#define GPIO_AF7_USART2        ((uint8_t)0x07)
#define GPIO_AF7_USART3        ((uint8_t)0x07)

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_LPUART1       ((uint8_t)0x08)

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_TSC           ((uint8_t)0x09)

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_USB_FS       ((uint8_t)0x0A)
#define GPIO_AF10_QUADSPI      ((uint8_t)0x0A)

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_COMP1        ((uint8_t)0x0C)


/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM2         ((uint8_t)0x0E)
#define GPIO_AF14_TIM15        ((uint8_t)0x0E)
#define GPIO_AF14_TIM16        ((uint8_t)0x0E)
#define GPIO_AF14_LPTIM2       ((uint8_t)0x0E)

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT     ((uint8_t)0x0F)

#define IS_GPIO_AF(AF)   ((AF) <= (uint8_t)0x0F)

#endif

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)

/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)
#define GPIO_AF0_MCO           ((uint8_t)0x00)
#define GPIO_AF0_SWJ           ((uint8_t)0x00)
#if defined(STM32L433xx) || defined(STM32L443xx)
#define GPIO_AF0_LCDBIAS       ((uint8_t)0x00)
#endif
#define GPIO_AF0_TRACE         ((uint8_t)0x00)

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)
#define GPIO_AF1_TIM2          ((uint8_t)0x01)
#define GPIO_AF1_LPTIM1        ((uint8_t)0x01)
#define GPIO_AF1_IR            ((uint8_t)0x01)

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)
#define GPIO_AF2_TIM2          ((uint8_t)0x02)

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_USART2        ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP2    ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP1    ((uint8_t)0x03)

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)
#define GPIO_AF4_I2C2          ((uint8_t)0x04)
#define GPIO_AF4_I2C3          ((uint8_t)0x04)

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)
#define GPIO_AF5_SPI2          ((uint8_t)0x05)

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)
#define GPIO_AF6_COMP1         ((uint8_t)0x06)

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)
#define GPIO_AF7_USART2        ((uint8_t)0x07)
#define GPIO_AF7_USART3        ((uint8_t)0x07)

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_LPUART1       ((uint8_t)0x08)

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)
#define GPIO_AF9_TSC           ((uint8_t)0x09)

/**
  * @brief   AF 10 selection
  */
#if defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
#define GPIO_AF10_USB_FS       ((uint8_t)0x0A)
#endif
#define GPIO_AF10_QUADSPI      ((uint8_t)0x0A)

#if defined(STM32L433xx) || defined(STM32L443xx)
/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_LCD          ((uint8_t)0x0B)
#endif

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_SWPMI1       ((uint8_t)0x0C)
#define GPIO_AF12_COMP1        ((uint8_t)0x0C)
#define GPIO_AF12_COMP2        ((uint8_t)0x0C)
#define GPIO_AF12_SDMMC1       ((uint8_t)0x0C)

/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_SAI1         ((uint8_t)0x0D)

/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM2         ((uint8_t)0x0E)
#define GPIO_AF14_TIM15        ((uint8_t)0x0E)
#define GPIO_AF14_TIM16        ((uint8_t)0x0E)
#define GPIO_AF14_LPTIM2       ((uint8_t)0x0E)

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT     ((uint8_t)0x0F)

#define IS_GPIO_AF(AF)   ((AF) <= (uint8_t)0x0F)

#endif

#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)

/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)
#define GPIO_AF0_MCO           ((uint8_t)0x00)
#define GPIO_AF0_SWJ           ((uint8_t)0x00)
#define GPIO_AF0_TRACE         ((uint8_t)0x00)

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)
#define GPIO_AF1_TIM2          ((uint8_t)0x01)
#define GPIO_AF1_LPTIM1        ((uint8_t)0x01)
#define GPIO_AF1_IR            ((uint8_t)0x01)

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)
#define GPIO_AF2_TIM2          ((uint8_t)0x02)
#define GPIO_AF2_TIM3          ((uint8_t)0x02)
#define GPIO_AF2_I2C4          ((uint8_t)0x02)

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TIM1_COMP2    ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP1    ((uint8_t)0x03)
#define GPIO_AF3_USART2        ((uint8_t)0x03)
#define GPIO_AF3_CAN1          ((uint8_t)0x03)
#define GPIO_AF3_I2C4          ((uint8_t)0x03)

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)
#define GPIO_AF4_I2C2          ((uint8_t)0x04)
#define GPIO_AF4_I2C3          ((uint8_t)0x04)
#define GPIO_AF4_I2C4          ((uint8_t)0x04)

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)
#define GPIO_AF5_SPI2          ((uint8_t)0x05)
#define GPIO_AF5_I2C4          ((uint8_t)0x05)

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)
#define GPIO_AF6_DFSDM1        ((uint8_t)0x06)
#define GPIO_AF6_COMP1         ((uint8_t)0x06)

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)
#define GPIO_AF7_USART2        ((uint8_t)0x07)
#define GPIO_AF7_USART3        ((uint8_t)0x07)

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_UART4         ((uint8_t)0x08)
#define GPIO_AF8_LPUART1       ((uint8_t)0x08)
#define GPIO_AF8_CAN1          ((uint8_t)0x08)


/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)
#define GPIO_AF9_TSC           ((uint8_t)0x09)

/**
  * @brief   AF 10 selection
  */
#if defined(STM32L452xx) || defined(STM32L462xx)
#define GPIO_AF10_USB_FS       ((uint8_t)0x0A)
#endif
#define GPIO_AF10_QUADSPI      ((uint8_t)0x0A)
#define GPIO_AF10_CAN1         ((uint8_t)0x0A)

/**
  * @brief   AF 11 selection
  */

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_COMP1        ((uint8_t)0x0C)
#define GPIO_AF12_COMP2        ((uint8_t)0x0C)
#define GPIO_AF12_SDMMC1       ((uint8_t)0x0C)

/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_SAI1         ((uint8_t)0x0D)

/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM2         ((uint8_t)0x0E)
#define GPIO_AF14_TIM15        ((uint8_t)0x0E)
#define GPIO_AF14_TIM16        ((uint8_t)0x0E)
#define GPIO_AF14_TIM17        ((uint8_t)0x0E)
#define GPIO_AF14_LPTIM2       ((uint8_t)0x0E)

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT     ((uint8_t)0x0F)

#define IS_GPIO_AF(AF)   ((AF) <= (uint8_t)0x0F)

#endif

#if defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx)

/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)
#define GPIO_AF0_MCO           ((uint8_t)0x00)
#define GPIO_AF0_SWJ           ((uint8_t)0x00)
#if defined(STM32L476xx) || defined(STM32L486xx)
#define GPIO_AF0_LCDBIAS       ((uint8_t)0x00)
#endif
#define GPIO_AF0_TRACE         ((uint8_t)0x00)

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)
#define GPIO_AF1_TIM2          ((uint8_t)0x01)
#define GPIO_AF1_TIM5          ((uint8_t)0x01)
#define GPIO_AF1_TIM8          ((uint8_t)0x01)
#define GPIO_AF1_LPTIM1        ((uint8_t)0x01)
#define GPIO_AF1_IR            ((uint8_t)0x01)

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)
#define GPIO_AF2_TIM2          ((uint8_t)0x02)
#define GPIO_AF2_TIM3          ((uint8_t)0x02)
#define GPIO_AF2_TIM4          ((uint8_t)0x02)
#define GPIO_AF2_TIM5          ((uint8_t)0x02)

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TIM8          ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP2    ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP1    ((uint8_t)0x03)

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)
#define GPIO_AF4_I2C2          ((uint8_t)0x04)
#define GPIO_AF4_I2C3          ((uint8_t)0x04)

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)
#define GPIO_AF5_SPI2          ((uint8_t)0x05)

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)
#define GPIO_AF6_DFSDM1        ((uint8_t)0x06)

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)
#define GPIO_AF7_USART2        ((uint8_t)0x07)
#define GPIO_AF7_USART3        ((uint8_t)0x07)

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_UART4         ((uint8_t)0x08)
#define GPIO_AF8_UART5         ((uint8_t)0x08)
#define GPIO_AF8_LPUART1       ((uint8_t)0x08)


/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)
#define GPIO_AF9_TSC           ((uint8_t)0x09)

/**
  * @brief   AF 10 selection
  */
#if defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx)
#define GPIO_AF10_OTG_FS       ((uint8_t)0x0A)
#endif
#define GPIO_AF10_QUADSPI      ((uint8_t)0x0A)

#if defined(STM32L476xx) || defined(STM32L486xx)
/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_LCD          ((uint8_t)0x0B)
#endif

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_FMC          ((uint8_t)0x0C)
#define GPIO_AF12_SWPMI1       ((uint8_t)0x0C)
#define GPIO_AF12_COMP1        ((uint8_t)0x0C)
#define GPIO_AF12_COMP2        ((uint8_t)0x0C)
#define GPIO_AF12_SDMMC1       ((uint8_t)0x0C)

/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_SAI1         ((uint8_t)0x0D)
#define GPIO_AF13_SAI2         ((uint8_t)0x0D)
#define GPIO_AF13_TIM8_COMP2   ((uint8_t)0x0D)
#define GPIO_AF13_TIM8_COMP1   ((uint8_t)0x0D)

/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM2         ((uint8_t)0x0E)
#define GPIO_AF14_TIM15        ((uint8_t)0x0E)
#define GPIO_AF14_TIM16        ((uint8_t)0x0E)
#define GPIO_AF14_TIM17        ((uint8_t)0x0E)
#define GPIO_AF14_LPTIM2       ((uint8_t)0x0E)
#define GPIO_AF14_TIM8_COMP1   ((uint8_t)0x0E)

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT     ((uint8_t)0x0F)

#define IS_GPIO_AF(AF)   ((AF) <= (uint8_t)0x0F)

#endif

#if defined(STM32L496xx) || defined(STM32L4A6xx)

/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)
#define GPIO_AF0_MCO           ((uint8_t)0x00)
#define GPIO_AF0_SWJ           ((uint8_t)0x00)
#define GPIO_AF0_TRACE         ((uint8_t)0x00)

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)
#define GPIO_AF1_TIM2          ((uint8_t)0x01)
#define GPIO_AF1_TIM5          ((uint8_t)0x01)
#define GPIO_AF1_TIM8          ((uint8_t)0x01)
#define GPIO_AF1_LPTIM1        ((uint8_t)0x01)
#define GPIO_AF1_IR            ((uint8_t)0x01)

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)
#define GPIO_AF2_TIM2          ((uint8_t)0x02)
#define GPIO_AF2_TIM3          ((uint8_t)0x02)
#define GPIO_AF2_TIM4          ((uint8_t)0x02)
#define GPIO_AF2_TIM5          ((uint8_t)0x02)
#define GPIO_AF2_I2C4          ((uint8_t)0x02)

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TIM8          ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP2    ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP1    ((uint8_t)0x03)
#define GPIO_AF3_CAN2          ((uint8_t)0x03)
#define GPIO_AF3_I2C4          ((uint8_t)0x03)
#define GPIO_AF3_QUADSPI       ((uint8_t)0x03)
#define GPIO_AF3_SPI2          ((uint8_t)0x03)
#define GPIO_AF3_USART2        ((uint8_t)0x03)

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)
#define GPIO_AF4_I2C2          ((uint8_t)0x04)
#define GPIO_AF4_I2C3          ((uint8_t)0x04)
#define GPIO_AF4_I2C4          ((uint8_t)0x04)
#define GPIO_AF4_DCMI          ((uint8_t)0x04)

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)
#define GPIO_AF5_SPI2          ((uint8_t)0x05)
#define GPIO_AF5_DCMI          ((uint8_t)0x05)
#define GPIO_AF5_I2C4          ((uint8_t)0x05)
#define GPIO_AF5_QUADSPI       ((uint8_t)0x05)

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)
#define GPIO_AF6_DFSDM1        ((uint8_t)0x06)
#define GPIO_AF6_I2C3          ((uint8_t)0x06)

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)
#define GPIO_AF7_USART2        ((uint8_t)0x07)
#define GPIO_AF7_USART3        ((uint8_t)0x07)

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_UART4         ((uint8_t)0x08)
#define GPIO_AF8_UART5         ((uint8_t)0x08)
#define GPIO_AF8_LPUART1       ((uint8_t)0x08)
#define GPIO_AF8_CAN2          ((uint8_t)0x08)

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)
#define GPIO_AF9_TSC           ((uint8_t)0x09)

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_OTG_FS       ((uint8_t)0x0A)
#define GPIO_AF10_QUADSPI      ((uint8_t)0x0A)
#define GPIO_AF10_CAN2         ((uint8_t)0x0A)
#define GPIO_AF10_DCMI         ((uint8_t)0x0A)

/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_LCD          ((uint8_t)0x0B)

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_FMC          ((uint8_t)0x0C)
#define GPIO_AF12_SWPMI1       ((uint8_t)0x0C)
#define GPIO_AF12_COMP1        ((uint8_t)0x0C)
#define GPIO_AF12_COMP2        ((uint8_t)0x0C)
#define GPIO_AF12_SDMMC1       ((uint8_t)0x0C)
#define GPIO_AF12_TIM1_COMP2   ((uint8_t)0x0C)
#define GPIO_AF12_TIM1_COMP1   ((uint8_t)0x0C)
#define GPIO_AF12_TIM8_COMP2   ((uint8_t)0x0C)

/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_SAI1         ((uint8_t)0x0D)
#define GPIO_AF13_SAI2         ((uint8_t)0x0D)
#define GPIO_AF13_TIM8_COMP2   ((uint8_t)0x0D)
#define GPIO_AF13_TIM8_COMP1   ((uint8_t)0x0D)

/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM2         ((uint8_t)0x0E)
#define GPIO_AF14_TIM15        ((uint8_t)0x0E)
#define GPIO_AF14_TIM16        ((uint8_t)0x0E)
#define GPIO_AF14_TIM17        ((uint8_t)0x0E)
#define GPIO_AF14_LPTIM2       ((uint8_t)0x0E)
#define GPIO_AF14_TIM8_COMP1   ((uint8_t)0x0E)

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT     ((uint8_t)0x0F)

#define IS_GPIO_AF(AF)   ((AF) <= (uint8_t)0x0F)

#endif

#if defined (STM32L4P5xx) || defined (STM32L4Q5xx)

/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)
#define GPIO_AF0_MCO           ((uint8_t)0x00)
#define GPIO_AF0_SWJ           ((uint8_t)0x00)
#define GPIO_AF0_TRACE         ((uint8_t)0x00)

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)
#define GPIO_AF1_TIM2          ((uint8_t)0x01)
#define GPIO_AF1_TIM5          ((uint8_t)0x01)
#define GPIO_AF1_TIM8          ((uint8_t)0x01)
#define GPIO_AF1_LPTIM1        ((uint8_t)0x01)
#define GPIO_AF1_IR            ((uint8_t)0x01)

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)
#define GPIO_AF2_TIM2          ((uint8_t)0x02)
#define GPIO_AF2_TIM3          ((uint8_t)0x02)
#define GPIO_AF2_TIM4          ((uint8_t)0x02)
#define GPIO_AF2_TIM5          ((uint8_t)0x02)

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_I2C4          ((uint8_t)0x03)
#define GPIO_AF3_OCTOSPIM_P1   ((uint8_t)0x03)
#define GPIO_AF3_SAI1          ((uint8_t)0x03)
#define GPIO_AF3_SPI2          ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP1    ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP2    ((uint8_t)0x03)
#define GPIO_AF3_TIM8          ((uint8_t)0x03)
#define GPIO_AF3_TIM8_COMP1    ((uint8_t)0x03)
#define GPIO_AF3_TIM8_COMP2    ((uint8_t)0x03)
#define GPIO_AF3_USART2        ((uint8_t)0x03)

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)
#define GPIO_AF4_I2C2          ((uint8_t)0x04)
#define GPIO_AF4_I2C3          ((uint8_t)0x04)
#define GPIO_AF4_I2C4          ((uint8_t)0x04)
#define GPIO_AF4_DCMI          ((uint8_t)0x04)
#define GPIO_AF4_PSSI          ((uint8_t)0x04)

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_DCMI          ((uint8_t)0x05)
#define GPIO_AF5_PSSI          ((uint8_t)0x05)
#define GPIO_AF5_DFSDM1        ((uint8_t)0x05)
#define GPIO_AF5_I2C4          ((uint8_t)0x05)
#define GPIO_AF5_OCTOSPIM_P1   ((uint8_t)0x05)
#define GPIO_AF5_OCTOSPIM_P2   ((uint8_t)0x05)
#define GPIO_AF5_SPI1          ((uint8_t)0x05)
#define GPIO_AF5_SPI2          ((uint8_t)0x05)
#define GPIO_AF5_SPI3          ((uint8_t)0x05)

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_DFSDM1        ((uint8_t)0x06)
#define GPIO_AF6_I2C3          ((uint8_t)0x06)
#define GPIO_AF6_OCTOSPIM_P1   ((uint8_t)0x06)
#define GPIO_AF6_OCTOSPIM_P2   ((uint8_t)0x06)
#define GPIO_AF6_SPI3          ((uint8_t)0x06)

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)
#define GPIO_AF7_USART2        ((uint8_t)0x07)
#define GPIO_AF7_USART3        ((uint8_t)0x07)
#define GPIO_AF7_SDMMC2        ((uint8_t)0x07)

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_LPUART1       ((uint8_t)0x08)
#define GPIO_AF8_SDMMC1        ((uint8_t)0x08)
#define GPIO_AF8_SDMMC2        ((uint8_t)0x08)
#define GPIO_AF8_UART4         ((uint8_t)0x08)
#define GPIO_AF8_UART5         ((uint8_t)0x08)

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)
#define GPIO_AF9_LTDC          ((uint8_t)0x09)
#define GPIO_AF9_TSC           ((uint8_t)0x09)

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_DCMI         ((uint8_t)0x0A)
#define GPIO_AF10_PSSI         ((uint8_t)0x0A)
#define GPIO_AF10_OCTOSPIM_P1  ((uint8_t)0x0A)
#define GPIO_AF10_OCTOSPIM_P2  ((uint8_t)0x0A)
#define GPIO_AF10_OTG_FS       ((uint8_t)0x0A)

/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_LTDC         ((uint8_t)0x0B)
#define GPIO_AF11_SDMMC2       ((uint8_t)0x0B)

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_COMP1        ((uint8_t)0x0C)
#define GPIO_AF12_COMP2        ((uint8_t)0x0C)
#define GPIO_AF12_FMC          ((uint8_t)0x0C)
#define GPIO_AF12_SDMMC1       ((uint8_t)0x0C)
#define GPIO_AF12_SDMMC2       ((uint8_t)0x0C)
#define GPIO_AF12_TIM1_COMP1   ((uint8_t)0x0C)
#define GPIO_AF12_TIM1_COMP2   ((uint8_t)0x0C)
#define GPIO_AF12_TIM8_COMP2   ((uint8_t)0x0C)

/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_SAI1         ((uint8_t)0x0D)
#define GPIO_AF13_SAI2         ((uint8_t)0x0D)
#define GPIO_AF13_TIM8_COMP1   ((uint8_t)0x0D)

/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM15        ((uint8_t)0x0E)
#define GPIO_AF14_TIM16        ((uint8_t)0x0E)
#define GPIO_AF14_TIM17        ((uint8_t)0x0E)
#define GPIO_AF14_LPTIM2       ((uint8_t)0x0E)
#define GPIO_AF14_TIM8_COMP2   ((uint8_t)0x0E)

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT     ((uint8_t)0x0F)

#define IS_GPIO_AF(AF)   ((AF) <= (uint8_t)0x0F)

#endif

#if defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)

/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)
#define GPIO_AF0_MCO           ((uint8_t)0x00)
#define GPIO_AF0_SWJ           ((uint8_t)0x00)
#define GPIO_AF0_TRACE         ((uint8_t)0x00)

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)
#define GPIO_AF1_TIM2          ((uint8_t)0x01)
#define GPIO_AF1_TIM5          ((uint8_t)0x01)
#define GPIO_AF1_TIM8          ((uint8_t)0x01)
#define GPIO_AF1_LPTIM1        ((uint8_t)0x01)
#define GPIO_AF1_IR            ((uint8_t)0x01)

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)
#define GPIO_AF2_TIM2          ((uint8_t)0x02)
#define GPIO_AF2_TIM3          ((uint8_t)0x02)
#define GPIO_AF2_TIM4          ((uint8_t)0x02)
#define GPIO_AF2_TIM5          ((uint8_t)0x02)

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_I2C4          ((uint8_t)0x03)
#define GPIO_AF3_OCTOSPIM_P1   ((uint8_t)0x03)
#define GPIO_AF3_SAI1          ((uint8_t)0x03)
#define GPIO_AF3_SPI2          ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP1    ((uint8_t)0x03)
#define GPIO_AF3_TIM1_COMP2    ((uint8_t)0x03)
#define GPIO_AF3_TIM8          ((uint8_t)0x03)
#define GPIO_AF3_TIM8_COMP1    ((uint8_t)0x03)
#define GPIO_AF3_TIM8_COMP2    ((uint8_t)0x03)
#define GPIO_AF3_USART2        ((uint8_t)0x03)

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)
#define GPIO_AF4_I2C2          ((uint8_t)0x04)
#define GPIO_AF4_I2C3          ((uint8_t)0x04)
#define GPIO_AF4_I2C4          ((uint8_t)0x04)
#define GPIO_AF4_DCMI          ((uint8_t)0x04)

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_DCMI          ((uint8_t)0x05)
#define GPIO_AF5_DFSDM1        ((uint8_t)0x05)
#define GPIO_AF5_I2C4          ((uint8_t)0x05)
#define GPIO_AF5_OCTOSPIM_P1   ((uint8_t)0x05)
#define GPIO_AF5_OCTOSPIM_P2   ((uint8_t)0x05)
#define GPIO_AF5_SPI1          ((uint8_t)0x05)
#define GPIO_AF5_SPI2          ((uint8_t)0x05)
#define GPIO_AF5_SPI3          ((uint8_t)0x05)

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_DFSDM1        ((uint8_t)0x06)
#define GPIO_AF6_I2C3          ((uint8_t)0x06)
#define GPIO_AF6_SPI3          ((uint8_t)0x06)

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)
#define GPIO_AF7_USART2        ((uint8_t)0x07)
#define GPIO_AF7_USART3        ((uint8_t)0x07)

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_LPUART1       ((uint8_t)0x08)
#define GPIO_AF8_SDMMC1        ((uint8_t)0x08)
#define GPIO_AF8_UART4         ((uint8_t)0x08)
#define GPIO_AF8_UART5         ((uint8_t)0x08)

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)
#define GPIO_AF9_LTDC          ((uint8_t)0x09)
#define GPIO_AF9_TSC           ((uint8_t)0x09)

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_DCMI         ((uint8_t)0x0A)
#define GPIO_AF10_OCTOSPIM_P1  ((uint8_t)0x0A)
#define GPIO_AF10_OCTOSPIM_P2  ((uint8_t)0x0A)
#define GPIO_AF10_OTG_FS       ((uint8_t)0x0A)

/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_DSI          ((uint8_t)0x0B)
#define GPIO_AF11_LTDC         ((uint8_t)0x0B)

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_COMP1        ((uint8_t)0x0C)
#define GPIO_AF12_COMP2        ((uint8_t)0x0C)
#define GPIO_AF12_DSI          ((uint8_t)0x0C)
#define GPIO_AF12_FMC          ((uint8_t)0x0C)
#define GPIO_AF12_SDMMC1       ((uint8_t)0x0C)
#define GPIO_AF12_TIM1_COMP1   ((uint8_t)0x0C)
#define GPIO_AF12_TIM1_COMP2   ((uint8_t)0x0C)
#define GPIO_AF12_TIM8_COMP2   ((uint8_t)0x0C)

/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_SAI1         ((uint8_t)0x0D)
#define GPIO_AF13_SAI2         ((uint8_t)0x0D)
#define GPIO_AF13_TIM8_COMP1   ((uint8_t)0x0D)

/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM2         ((uint8_t)0x0E)
#define GPIO_AF14_TIM15        ((uint8_t)0x0E)
#define GPIO_AF14_TIM16        ((uint8_t)0x0E)
#define GPIO_AF14_TIM17        ((uint8_t)0x0E)
#define GPIO_AF14_LPTIM2       ((uint8_t)0x0E)
#define GPIO_AF14_TIM8_COMP2   ((uint8_t)0x0E)

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT     ((uint8_t)0x0F)

#define IS_GPIO_AF(AF)   ((AF) <= (uint8_t)0x0F)

#endif

/**
  * @}
  */

/**
  * @}
  */


/** @defgroup GPIOEx_Exported_Macros GPIOEx Exported Macros
  * @{
  */

/** @defgroup GPIOEx_Get_Port_Index GPIOEx_Get Port Index
* @{
  */
#if defined(STM32L412xx) || defined(STM32L422xx)

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL :\
                                      ((__GPIOx__) == (GPIOD))? 3uL : 7uL)

#endif

#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx)

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL :\
                                      ((__GPIOx__) == (GPIOD))? 3uL :\
                                      ((__GPIOx__) == (GPIOE))? 4uL : 7uL)

#endif

#if defined(STM32L432xx) || defined(STM32L442xx)

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL : 7uL)

#endif

#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL :\
                                      ((__GPIOx__) == (GPIOD))? 3uL :\
                                      ((__GPIOx__) == (GPIOE))? 4uL : 7uL)

#endif

#if defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx)

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL :\
                                      ((__GPIOx__) == (GPIOD))? 3uL :\
                                      ((__GPIOx__) == (GPIOE))? 4uL :\
                                      ((__GPIOx__) == (GPIOF))? 5uL :\
                                      ((__GPIOx__) == (GPIOG))? 6uL : 7uL)

#endif

#if defined(STM32L496xx) || defined(STM32L4A6xx)

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL :\
                                      ((__GPIOx__) == (GPIOD))? 3uL :\
                                      ((__GPIOx__) == (GPIOE))? 4uL :\
                                      ((__GPIOx__) == (GPIOF))? 5uL :\
                                      ((__GPIOx__) == (GPIOG))? 6uL :\
                                      ((__GPIOx__) == (GPIOH))? 7uL : 8uL)

#endif

#if defined (STM32L4P5xx) || defined (STM32L4Q5xx) || defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL :\
                                      ((__GPIOx__) == (GPIOD))? 3uL :\
                                      ((__GPIOx__) == (GPIOE))? 4uL :\
                                      ((__GPIOx__) == (GPIOF))? 5uL :\
                                      ((__GPIOx__) == (GPIOG))? 6uL :\
                                      ((__GPIOx__) == (GPIOH))? 7uL : 8uL)

#endif

/**
  * @}
  */

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

