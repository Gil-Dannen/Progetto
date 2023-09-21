/**
  ******************************************************************************
  * @file    stm32l4xx_hal_rcc_ex.c
  * @author  MCD Application Team
  * @brief   Extended RCC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities RCC extended peripheral:
  *           + Extended Peripheral Control functions
  *           + Extended Clock management functions
  *           + Extended Clock Recovery System Control functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "stm32l4xx_hal.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup RCCEx RCCEx
  * @brief RCC Extended HAL module driver
  * @{
  */

#ifdef HAL_RCC_MODULE_ENABLED



/** @defgroup RCCEx_Private_Constants RCCEx Private Constants
 * @{
 */
#define PLLSAI1_TIMEOUT_VALUE     2U
#define PLLSAI2_TIMEOUT_VALUE     2U
#define PLL_TIMEOUT_VALUE         2U

#define DIVIDER_P_UPDATE          0U
#define DIVIDER_Q_UPDATE          1U
#define DIVIDER_R_UPDATE          2U

#define __LSCO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define LSCO_GPIO_PORT            GPIOA
#define LSCO_PIN                  GPIO_PIN_2
/**
  * @}
  */




/** @defgroup RCCEx_Private_Functions RCCEx Private Functions
 * @{
 */
#if defined(RCC_PLLSAI1_SUPPORT)

static HAL_StatusTypeDef RCCEx_PLLSAI1_Config(RCC_PLLSAI1InitTypeDef *PllSai1, uint32_t Divider);

#endif

#if defined(RCC_PLLSAI2_SUPPORT)

static HAL_StatusTypeDef RCCEx_PLLSAI2_Config(RCC_PLLSAI2InitTypeDef *PllSai2, uint32_t Divider);

#endif

#if defined(SAI1)

static uint32_t RCCEx_GetSAIxPeriphCLKFreq(uint32_t PeriphClk, uint32_t InputFrequency);

#endif
/**
  * @}
  */



/** @defgroup RCCEx_Exported_Functions RCCEx Exported Functions
  * @{
  */

/** @defgroup RCCEx_Exported_Functions_Group1 Extended Peripheral Control functions
 *  @brief  Extended Peripheral Control functions
 *
@verbatim
 ===============================================================================
                ##### Extended Peripheral Control functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the RCC Clocks
    frequencies.
    [..]
    (@) Important note: Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to
        select the RTC clock source; in this case the Backup domain will be reset in
        order to modify the RTC Clock source, as consequence RTC registers (including
        the backup registers) are set to their reset values.

@endverbatim
  * @{
  */
/**
  * @brief  Initialize the RCC extended peripherals clocks according to the specified
  *         parameters in the RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit  pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains a field PeriphClockSelection which can be a combination of the following values:
  *            @arg @ref RCC_PERIPHCLK_RTC  RTC peripheral clock
  *            @arg @ref RCC_PERIPHCLK_ADC  ADC peripheral clock
  @if STM32L462xx
  *            @arg @ref RCC_PERIPHCLK_DFSDM1  DFSDM1 peripheral clock (only for devices with DFSDM1)
  @endif
  @if STM32L486xx
  *            @arg @ref RCC_PERIPHCLK_DFSDM1  DFSDM1 peripheral clock (only for devices with DFSDM1)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_DFSDM1  DFSDM1 peripheral clock (only for devices with DFSDM1)
  @endif
  *            @arg @ref RCC_PERIPHCLK_I2C1  I2C1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C2  I2C2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C3  I2C3 peripheral clock
  @if STM32L462xx
  *            @arg @ref RCC_PERIPHCLK_I2C4  I2C4 peripheral clock (only for devices with I2C4)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_I2C4  I2C4 peripheral clock (only for devices with I2C4)
  @endif
  @if STM32L4S9xx
  *            @arg @ref RCC_PERIPHCLK_I2C4  I2C4 peripheral clock (only for devices with I2C4)
  @endif
  *            @arg @ref RCC_PERIPHCLK_LPTIM1  LPTIM1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPTIM2  LPTIM2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPUART1  LPUART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_RNG  RNG peripheral clock
  *            @arg @ref RCC_PERIPHCLK_SAI1  SAI1 peripheral clock (only for devices with SAI1)
  @if STM32L486xx
  *            @arg @ref RCC_PERIPHCLK_SAI2  SAI2 peripheral clock (only for devices with SAI2)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_SAI2  SAI2 peripheral clock (only for devices with SAI2)
  @endif
  @if STM32L4S9xx
  *            @arg @ref RCC_PERIPHCLK_SAI2  SAI2 peripheral clock (only for devices with SAI2)
  @endif
  *            @arg @ref RCC_PERIPHCLK_SDMMC1  SDMMC1 peripheral clock
  @if STM32L443xx
  *            @arg @ref RCC_PERIPHCLK_SWPMI1  SWPMI1 peripheral clock (only for devices with SWPMI1)
  @endif
  @if STM32L486xx
  *            @arg @ref RCC_PERIPHCLK_SWPMI1  SWPMI1 peripheral clock (only for devices with SWPMI1)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_SWPMI1  SWPMI1 peripheral clock (only for devices with SWPMI1)
  @endif
  *            @arg @ref RCC_PERIPHCLK_USART1  USART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USART2  USART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USART3  USART1 peripheral clock
  @if STM32L462xx
  *            @arg @ref RCC_PERIPHCLK_UART4  USART1 peripheral clock (only for devices with UART4)
  @endif
  @if STM32L486xx
  *            @arg @ref RCC_PERIPHCLK_UART4  USART1 peripheral clock (only for devices with UART4)
  *            @arg @ref RCC_PERIPHCLK_UART5  USART1 peripheral clock (only for devices with UART5)
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock (only for devices with USB)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_UART4  USART1 peripheral clock (only for devices with UART4)
  *            @arg @ref RCC_PERIPHCLK_UART5  USART1 peripheral clock (only for devices with UART5)
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock (only for devices with USB)
  @endif
  @if STM32L4S9xx
  *            @arg @ref RCC_PERIPHCLK_UART4  USART1 peripheral clock (only for devices with UART4)
  *            @arg @ref RCC_PERIPHCLK_UART5  USART1 peripheral clock (only for devices with UART5)
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock (only for devices with USB)
  *            @arg @ref RCC_PERIPHCLK_DFSDM1  DFSDM1 peripheral kernel clock (only for devices with DFSDM1)
  *            @arg @ref RCC_PERIPHCLK_DFSDM1AUDIO  DFSDM1 peripheral audio clock (only for devices with DFSDM1)
  *            @arg @ref RCC_PERIPHCLK_LTDC  LTDC peripheral clock (only for devices with LTDC)
  *            @arg @ref RCC_PERIPHCLK_DSI  DSI peripheral clock (only for devices with DSI)
  *            @arg @ref RCC_PERIPHCLK_OSPI  OctoSPI peripheral clock (only for devices with OctoSPI)
  @endif
  *
  * @note   Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
  *         the RTC clock source: in this case the access to Backup domain is enabled.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tmpregister, tickstart;
  HAL_StatusTypeDef ret = HAL_OK;
  HAL_StatusTypeDef status = HAL_OK;


  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

#if defined(SAI1)


  if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI1) == RCC_PERIPHCLK_SAI1))
  {

    assert_param(IS_RCC_SAI1CLK(PeriphClkInit->Sai1ClockSelection));

    switch(PeriphClkInit->Sai1ClockSelection)
    {
    case RCC_SAI1CLKSOURCE_PLL:

#if defined(RCC_PLLSAI2_SUPPORT)
      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_SAI3CLK);
#else
      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_SAI2CLK);
#endif

      break;

    case RCC_SAI1CLKSOURCE_PLLSAI1:

      ret = RCCEx_PLLSAI1_Config(&(PeriphClkInit->PLLSAI1), DIVIDER_P_UPDATE);

      break;

#if defined(RCC_PLLSAI2_SUPPORT)

    case RCC_SAI1CLKSOURCE_PLLSAI2:

      ret = RCCEx_PLLSAI2_Config(&(PeriphClkInit->PLLSAI2), DIVIDER_P_UPDATE);

      break;

#endif

    case RCC_SAI1CLKSOURCE_PIN:
#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
    case RCC_SAI1CLKSOURCE_HSI:
#endif

      break;

    default:
      ret = HAL_ERROR;
      break;
    }

    if(ret == HAL_OK)
    {

      __HAL_RCC_SAI1_CONFIG(PeriphClkInit->Sai1ClockSelection);
    }
    else
    {

      status = ret;
    }
  }

#endif

#if defined(SAI2)


  if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI2) == RCC_PERIPHCLK_SAI2))
  {

    assert_param(IS_RCC_SAI2CLK(PeriphClkInit->Sai2ClockSelection));

    switch(PeriphClkInit->Sai2ClockSelection)
    {
    case RCC_SAI2CLKSOURCE_PLL:

      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_SAI3CLK);

      break;

    case RCC_SAI2CLKSOURCE_PLLSAI1:

      ret = RCCEx_PLLSAI1_Config(&(PeriphClkInit->PLLSAI1), DIVIDER_P_UPDATE);

      break;

    case RCC_SAI2CLKSOURCE_PLLSAI2:

      ret = RCCEx_PLLSAI2_Config(&(PeriphClkInit->PLLSAI2), DIVIDER_P_UPDATE);

      break;

    case RCC_SAI2CLKSOURCE_PIN:
#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
    case RCC_SAI2CLKSOURCE_HSI:
#endif

      break;

    default:
      ret = HAL_ERROR;
      break;
    }

    if(ret == HAL_OK)
    {

      __HAL_RCC_SAI2_CONFIG(PeriphClkInit->Sai2ClockSelection);
    }
    else
    {

      status = ret;
    }
  }
#endif


  if((PeriphClkInit->PeriphClockSelection & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC)
  {
    FlagStatus       pwrclkchanged = RESET;


    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));


    if(__HAL_RCC_PWR_IS_CLK_DISABLED() != 0U)
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }


    SET_BIT(PWR->CR1, PWR_CR1_DBP);


    tickstart = HAL_GetTick();

    while(READ_BIT(PWR->CR1, PWR_CR1_DBP) == 0U)
    {
      if((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
      {
        ret = HAL_TIMEOUT;
        break;
      }
    }

    if(ret == HAL_OK)
    {

      tmpregister = READ_BIT(RCC->BDCR, RCC_BDCR_RTCSEL);

      if((tmpregister != RCC_RTCCLKSOURCE_NONE) && (tmpregister != PeriphClkInit->RTCClockSelection))
      {

        tmpregister = READ_BIT(RCC->BDCR, ~(RCC_BDCR_RTCSEL));

        __HAL_RCC_BACKUPRESET_FORCE();
        __HAL_RCC_BACKUPRESET_RELEASE();

        RCC->BDCR = tmpregister;
      }


      if (HAL_IS_BIT_SET(tmpregister, RCC_BDCR_LSEON))
      {

        tickstart = HAL_GetTick();


        while(READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY) == 0U)
        {
          if((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE)
          {
            ret = HAL_TIMEOUT;
            break;
          }
        }
      }

      if(ret == HAL_OK)
      {

        __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
      }
      else
      {

        status = ret;
      }
    }
    else
    {

      status = ret;
    }


    if(pwrclkchanged == SET)
    {
      __HAL_RCC_PWR_CLK_DISABLE();
    }
  }


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART1) == RCC_PERIPHCLK_USART1)
  {

    assert_param(IS_RCC_USART1CLKSOURCE(PeriphClkInit->Usart1ClockSelection));


    __HAL_RCC_USART1_CONFIG(PeriphClkInit->Usart1ClockSelection);
  }


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART2) == RCC_PERIPHCLK_USART2)
  {

    assert_param(IS_RCC_USART2CLKSOURCE(PeriphClkInit->Usart2ClockSelection));


    __HAL_RCC_USART2_CONFIG(PeriphClkInit->Usart2ClockSelection);
  }

#if defined(USART3)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART3) == RCC_PERIPHCLK_USART3)
  {

    assert_param(IS_RCC_USART3CLKSOURCE(PeriphClkInit->Usart3ClockSelection));


    __HAL_RCC_USART3_CONFIG(PeriphClkInit->Usart3ClockSelection);
  }

#endif

#if defined(UART4)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_UART4) == RCC_PERIPHCLK_UART4)
  {

    assert_param(IS_RCC_UART4CLKSOURCE(PeriphClkInit->Uart4ClockSelection));


    __HAL_RCC_UART4_CONFIG(PeriphClkInit->Uart4ClockSelection);
  }

#endif

#if defined(UART5)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_UART5) == RCC_PERIPHCLK_UART5)
  {

    assert_param(IS_RCC_UART5CLKSOURCE(PeriphClkInit->Uart5ClockSelection));


    __HAL_RCC_UART5_CONFIG(PeriphClkInit->Uart5ClockSelection);
  }

#endif


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPUART1) == RCC_PERIPHCLK_LPUART1)
  {

    assert_param(IS_RCC_LPUART1CLKSOURCE(PeriphClkInit->Lpuart1ClockSelection));


    __HAL_RCC_LPUART1_CONFIG(PeriphClkInit->Lpuart1ClockSelection);
  }


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM1) == (RCC_PERIPHCLK_LPTIM1))
  {
    assert_param(IS_RCC_LPTIM1CLK(PeriphClkInit->Lptim1ClockSelection));
    __HAL_RCC_LPTIM1_CONFIG(PeriphClkInit->Lptim1ClockSelection);
  }


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM2) == (RCC_PERIPHCLK_LPTIM2))
  {
    assert_param(IS_RCC_LPTIM2CLK(PeriphClkInit->Lptim2ClockSelection));
    __HAL_RCC_LPTIM2_CONFIG(PeriphClkInit->Lptim2ClockSelection);
  }


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C1) == RCC_PERIPHCLK_I2C1)
  {

    assert_param(IS_RCC_I2C1CLKSOURCE(PeriphClkInit->I2c1ClockSelection));


    __HAL_RCC_I2C1_CONFIG(PeriphClkInit->I2c1ClockSelection);
  }

#if defined(I2C2)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C2) == RCC_PERIPHCLK_I2C2)
  {

    assert_param(IS_RCC_I2C2CLKSOURCE(PeriphClkInit->I2c2ClockSelection));


    __HAL_RCC_I2C2_CONFIG(PeriphClkInit->I2c2ClockSelection);
  }

#endif


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C3) == RCC_PERIPHCLK_I2C3)
  {

    assert_param(IS_RCC_I2C3CLKSOURCE(PeriphClkInit->I2c3ClockSelection));


    __HAL_RCC_I2C3_CONFIG(PeriphClkInit->I2c3ClockSelection);
  }

#if defined(I2C4)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C4) == RCC_PERIPHCLK_I2C4)
  {

    assert_param(IS_RCC_I2C4CLKSOURCE(PeriphClkInit->I2c4ClockSelection));


    __HAL_RCC_I2C4_CONFIG(PeriphClkInit->I2c4ClockSelection);
  }

#endif

#if defined(USB_OTG_FS) || defined(USB)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USB) == (RCC_PERIPHCLK_USB))
  {
    assert_param(IS_RCC_USBCLKSOURCE(PeriphClkInit->UsbClockSelection));
    __HAL_RCC_USB_CONFIG(PeriphClkInit->UsbClockSelection);

    if(PeriphClkInit->UsbClockSelection == RCC_USBCLKSOURCE_PLL)
    {

      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_48M1CLK);
    }
    else
    {
#if defined(RCC_PLLSAI1_SUPPORT)
      if(PeriphClkInit->UsbClockSelection == RCC_USBCLKSOURCE_PLLSAI1)
      {

        ret = RCCEx_PLLSAI1_Config(&(PeriphClkInit->PLLSAI1), DIVIDER_Q_UPDATE);

        if(ret != HAL_OK)
        {

          status = ret;
        }
      }
#endif
    }
  }

#endif

#if defined(SDMMC1)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SDMMC1) == (RCC_PERIPHCLK_SDMMC1))
  {
    assert_param(IS_RCC_SDMMC1CLKSOURCE(PeriphClkInit->Sdmmc1ClockSelection));
    __HAL_RCC_SDMMC1_CONFIG(PeriphClkInit->Sdmmc1ClockSelection);

    if(PeriphClkInit->Sdmmc1ClockSelection == RCC_SDMMC1CLKSOURCE_PLL)
    {

      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_48M1CLK);
    }
#if defined(RCC_CCIPR2_SDMMCSEL)
    else if(PeriphClkInit->Sdmmc1ClockSelection == RCC_SDMMC1CLKSOURCE_PLLP)
    {

      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_SAI3CLK);
    }
#endif
    else if(PeriphClkInit->Sdmmc1ClockSelection == RCC_SDMMC1CLKSOURCE_PLLSAI1)
    {

      ret = RCCEx_PLLSAI1_Config(&(PeriphClkInit->PLLSAI1), DIVIDER_Q_UPDATE);

      if(ret != HAL_OK)
      {

        status = ret;
      }
    }
    else
    {

    }
  }

#endif


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RNG) == (RCC_PERIPHCLK_RNG))
  {
    assert_param(IS_RCC_RNGCLKSOURCE(PeriphClkInit->RngClockSelection));
    __HAL_RCC_RNG_CONFIG(PeriphClkInit->RngClockSelection);

    if(PeriphClkInit->RngClockSelection == RCC_RNGCLKSOURCE_PLL)
    {

      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_48M1CLK);
    }
#if defined(RCC_PLLSAI1_SUPPORT)
    else if(PeriphClkInit->RngClockSelection == RCC_RNGCLKSOURCE_PLLSAI1)
    {

      ret = RCCEx_PLLSAI1_Config(&(PeriphClkInit->PLLSAI1), DIVIDER_Q_UPDATE);

      if(ret != HAL_OK)
      {

        status = ret;
      }
    }
#endif
    else
    {

    }
  }


#if !defined(STM32L412xx) && !defined(STM32L422xx)
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_ADC) == RCC_PERIPHCLK_ADC)
  {

    assert_param(IS_RCC_ADCCLKSOURCE(PeriphClkInit->AdcClockSelection));


    __HAL_RCC_ADC_CONFIG(PeriphClkInit->AdcClockSelection);

#if defined(RCC_PLLSAI1_SUPPORT)
    if(PeriphClkInit->AdcClockSelection == RCC_ADCCLKSOURCE_PLLSAI1)
    {

      ret = RCCEx_PLLSAI1_Config(&(PeriphClkInit->PLLSAI1), DIVIDER_R_UPDATE);

      if(ret != HAL_OK)
      {

        status = ret;
      }
    }
#endif

#if defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx) || defined(STM32L496xx) || defined(STM32L4A6xx)

    else if(PeriphClkInit->AdcClockSelection == RCC_ADCCLKSOURCE_PLLSAI2)
    {

      ret = RCCEx_PLLSAI2_Config(&(PeriphClkInit->PLLSAI2), DIVIDER_R_UPDATE);

      if(ret != HAL_OK)
      {

        status = ret;
      }
    }

#endif

  }
#endif

#if defined(SWPMI1)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SWPMI1) == RCC_PERIPHCLK_SWPMI1)
  {

    assert_param(IS_RCC_SWPMI1CLKSOURCE(PeriphClkInit->Swpmi1ClockSelection));


    __HAL_RCC_SWPMI1_CONFIG(PeriphClkInit->Swpmi1ClockSelection);
  }

#endif

#if defined(DFSDM1_Filter0)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_DFSDM1) == RCC_PERIPHCLK_DFSDM1)
  {

    assert_param(IS_RCC_DFSDM1CLKSOURCE(PeriphClkInit->Dfsdm1ClockSelection));


    __HAL_RCC_DFSDM1_CONFIG(PeriphClkInit->Dfsdm1ClockSelection);
  }

#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)

  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_DFSDM1AUDIO) == RCC_PERIPHCLK_DFSDM1AUDIO)
  {

    assert_param(IS_RCC_DFSDM1AUDIOCLKSOURCE(PeriphClkInit->Dfsdm1AudioClockSelection));


    __HAL_RCC_DFSDM1AUDIO_CONFIG(PeriphClkInit->Dfsdm1AudioClockSelection);
  }

#endif

#endif

#if defined(LTDC)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LTDC) == RCC_PERIPHCLK_LTDC)
  {

    assert_param(IS_RCC_LTDCCLKSOURCE(PeriphClkInit->LtdcClockSelection));


    __HAL_RCC_PLLSAI2_DISABLE();


    tickstart = HAL_GetTick();


    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI2RDY) != 0U)
    {
      if((HAL_GetTick() - tickstart) > PLLSAI2_TIMEOUT_VALUE)
      {
        ret = HAL_TIMEOUT;
        break;
      }
    }

    if(ret == HAL_OK)
    {

      __HAL_RCC_LTDC_CONFIG(PeriphClkInit->LtdcClockSelection);


      ret = RCCEx_PLLSAI2_Config(&(PeriphClkInit->PLLSAI2), DIVIDER_R_UPDATE);
    }

    if(ret != HAL_OK)
    {

      status = ret;
    }
  }

#endif

#if defined(DSI)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_DSI) == RCC_PERIPHCLK_DSI)
  {

    assert_param(IS_RCC_DSICLKSOURCE(PeriphClkInit->DsiClockSelection));


    __HAL_RCC_DSI_CONFIG(PeriphClkInit->DsiClockSelection);

    if(PeriphClkInit->DsiClockSelection == RCC_DSICLKSOURCE_PLLSAI2)
    {

      ret = RCCEx_PLLSAI2_Config(&(PeriphClkInit->PLLSAI2), DIVIDER_Q_UPDATE);

      if(ret != HAL_OK)
      {

        status = ret;
      }
    }
  }

#endif

#if defined(OCTOSPI1) || defined(OCTOSPI2)


  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_OSPI) == RCC_PERIPHCLK_OSPI)
  {

    assert_param(IS_RCC_OSPICLKSOURCE(PeriphClkInit->OspiClockSelection));


    __HAL_RCC_OSPI_CONFIG(PeriphClkInit->OspiClockSelection);

    if(PeriphClkInit->OspiClockSelection == RCC_OSPICLKSOURCE_PLL)
    {

      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_48M1CLK);
    }
  }

#endif

  return status;
}

/**
  * @brief  Get the RCC_ClkInitStruct according to the internal RCC configuration registers.
  * @param  PeriphClkInit  pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         returns the configuration information for the Extended Peripherals
  *         clocks(SAI1, SAI2, LPTIM1, LPTIM2, I2C1, I2C2, I2C3, I2C4, LPUART1,
  *         USART1, USART2, USART3, UART4, UART5, RTC, ADCx, DFSDMx, SWPMI1, USB, SDMMC1 and RNG).
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{


#if defined(STM32L412xx) || defined(STM32L422xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 |                                               \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   |                        \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 |                                               RCC_PERIPHCLK_USB    | \
                                                                RCC_PERIPHCLK_RNG    |                                                                      \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L431xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 |                                               \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   |                        \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   |                                               \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC    | RCC_PERIPHCLK_SWPMI1 |                        \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L432xx) || defined(STM32L442xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 |                                                                      \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   |                        RCC_PERIPHCLK_I2C3   |                        \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   |                        RCC_PERIPHCLK_USB    | \
                                                                RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC    | RCC_PERIPHCLK_SWPMI1 |                        \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L433xx) || defined(STM32L443xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 |                                               \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   |                        \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   |                        RCC_PERIPHCLK_USB    | \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC    | RCC_PERIPHCLK_SWPMI1 |                        \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L451xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4  |                        \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   | RCC_PERIPHCLK_I2C4   | \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   |                                               \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC    |                        RCC_PERIPHCLK_DFSDM1 | \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L452xx) || defined(STM32L462xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4  |                        \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   | RCC_PERIPHCLK_I2C4   | \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   |                        RCC_PERIPHCLK_USB    | \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC    |                        RCC_PERIPHCLK_DFSDM1 | \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L471xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4  | RCC_PERIPHCLK_UART5  | \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3                          | \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   | RCC_PERIPHCLK_SAI2                          | \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC    | RCC_PERIPHCLK_SWPMI1 | RCC_PERIPHCLK_DFSDM1 | \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4  | RCC_PERIPHCLK_UART5  | \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   |                        \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   | RCC_PERIPHCLK_SAI2   | RCC_PERIPHCLK_USB    | \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC    | RCC_PERIPHCLK_SWPMI1 | RCC_PERIPHCLK_DFSDM1 | \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L496xx) || defined(STM32L4A6xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4  | RCC_PERIPHCLK_UART5  | \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   | RCC_PERIPHCLK_I2C4   | \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   | RCC_PERIPHCLK_SAI2   | RCC_PERIPHCLK_USB    | \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC    | RCC_PERIPHCLK_SWPMI1 | RCC_PERIPHCLK_DFSDM1 | \
                                        RCC_PERIPHCLK_RTC ;

#elif defined(STM32L4R5xx) || defined(STM32L4S5xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4  | RCC_PERIPHCLK_UART5  | \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   | RCC_PERIPHCLK_I2C4   | \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   | RCC_PERIPHCLK_SAI2   | RCC_PERIPHCLK_USB    | \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC                           | RCC_PERIPHCLK_DFSDM1 | \
                                        RCC_PERIPHCLK_DFSDM1AUDIO | RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_OSPI;

#elif defined(STM32L4R7xx) || defined(STM32L4S7xx) || defined(STM32L4Q5xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4  | RCC_PERIPHCLK_UART5  | \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   | RCC_PERIPHCLK_I2C4   | \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   | RCC_PERIPHCLK_SAI2   | RCC_PERIPHCLK_USB    | \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC                           | RCC_PERIPHCLK_DFSDM1 | \
                                        RCC_PERIPHCLK_DFSDM1AUDIO | RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_OSPI  | RCC_PERIPHCLK_LTDC;

#elif defined(STM32L4R9xx) || defined(STM32L4S9xx)

  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_USART1  | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4  | RCC_PERIPHCLK_UART5  | \
                                        RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_I2C2   | RCC_PERIPHCLK_I2C3   | RCC_PERIPHCLK_I2C4   | \
                                        RCC_PERIPHCLK_LPTIM1  | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_SAI1   | RCC_PERIPHCLK_SAI2   | RCC_PERIPHCLK_USB    | \
                                        RCC_PERIPHCLK_SDMMC1  | RCC_PERIPHCLK_RNG    | RCC_PERIPHCLK_ADC                           | RCC_PERIPHCLK_DFSDM1 | \
                                        RCC_PERIPHCLK_DFSDM1AUDIO | RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_OSPI  | RCC_PERIPHCLK_LTDC   | RCC_PERIPHCLK_DSI;

#endif

#if defined(RCC_PLLSAI1_SUPPORT)



  PeriphClkInit->PLLSAI1.PLLSAI1Source = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_Pos;
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)
  PeriphClkInit->PLLSAI1.PLLSAI1M = (READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1M) >> RCC_PLLSAI1CFGR_PLLSAI1M_Pos) + 1U;
#else
  PeriphClkInit->PLLSAI1.PLLSAI1M = (READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U;
#endif
  PeriphClkInit->PLLSAI1.PLLSAI1N = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos;
  PeriphClkInit->PLLSAI1.PLLSAI1P = ((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P) >> RCC_PLLSAI1CFGR_PLLSAI1P_Pos) << 4U) + 7U;
  PeriphClkInit->PLLSAI1.PLLSAI1Q = ((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q) >> RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) + 1U) * 2U;
  PeriphClkInit->PLLSAI1.PLLSAI1R = ((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R) >> RCC_PLLSAI1CFGR_PLLSAI1R_Pos) + 1U) * 2U;

#endif

#if defined(RCC_PLLSAI2_SUPPORT)



  PeriphClkInit->PLLSAI2.PLLSAI2Source = PeriphClkInit->PLLSAI1.PLLSAI1Source;
#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)
  PeriphClkInit->PLLSAI2.PLLSAI2M = (READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2M) >> RCC_PLLSAI2CFGR_PLLSAI2M_Pos) + 1U;
#else
  PeriphClkInit->PLLSAI2.PLLSAI2M = PeriphClkInit->PLLSAI1.PLLSAI1M;
#endif
  PeriphClkInit->PLLSAI2.PLLSAI2N = READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N) >> RCC_PLLSAI2CFGR_PLLSAI2N_Pos;
  PeriphClkInit->PLLSAI2.PLLSAI2P = ((READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2P) >> RCC_PLLSAI2CFGR_PLLSAI2P_Pos) << 4U) + 7U;
#if defined(RCC_PLLSAI2Q_DIV_SUPPORT)
  PeriphClkInit->PLLSAI2.PLLSAI2Q = ((READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2Q) >> RCC_PLLSAI2CFGR_PLLSAI2Q_Pos) + 1U) * 2U;
#endif
  PeriphClkInit->PLLSAI2.PLLSAI2R = ((READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2R)>> RCC_PLLSAI2CFGR_PLLSAI2R_Pos) + 1U) * 2U;

#endif


  PeriphClkInit->Usart1ClockSelection  = __HAL_RCC_GET_USART1_SOURCE();

  PeriphClkInit->Usart2ClockSelection  = __HAL_RCC_GET_USART2_SOURCE();

#if defined(USART3)

  PeriphClkInit->Usart3ClockSelection  = __HAL_RCC_GET_USART3_SOURCE();
#endif

#if defined(UART4)

  PeriphClkInit->Uart4ClockSelection   = __HAL_RCC_GET_UART4_SOURCE();
#endif

#if defined(UART5)

  PeriphClkInit->Uart5ClockSelection   = __HAL_RCC_GET_UART5_SOURCE();
#endif


  PeriphClkInit->Lpuart1ClockSelection = __HAL_RCC_GET_LPUART1_SOURCE();


  PeriphClkInit->I2c1ClockSelection    = __HAL_RCC_GET_I2C1_SOURCE();

#if defined(I2C2)

  PeriphClkInit->I2c2ClockSelection    = __HAL_RCC_GET_I2C2_SOURCE();
#endif


  PeriphClkInit->I2c3ClockSelection    = __HAL_RCC_GET_I2C3_SOURCE();

#if defined(I2C4)

  PeriphClkInit->I2c4ClockSelection    = __HAL_RCC_GET_I2C4_SOURCE();
#endif


  PeriphClkInit->Lptim1ClockSelection  = __HAL_RCC_GET_LPTIM1_SOURCE();


  PeriphClkInit->Lptim2ClockSelection  = __HAL_RCC_GET_LPTIM2_SOURCE();

#if defined(SAI1)

  PeriphClkInit->Sai1ClockSelection    = __HAL_RCC_GET_SAI1_SOURCE();
#endif

#if defined(SAI2)

  PeriphClkInit->Sai2ClockSelection    = __HAL_RCC_GET_SAI2_SOURCE();
#endif


  PeriphClkInit->RTCClockSelection     = __HAL_RCC_GET_RTC_SOURCE();

#if defined(USB_OTG_FS) || defined(USB)

  PeriphClkInit->UsbClockSelection   = __HAL_RCC_GET_USB_SOURCE();
#endif

#if defined(SDMMC1)

  PeriphClkInit->Sdmmc1ClockSelection   = __HAL_RCC_GET_SDMMC1_SOURCE();
#endif


  PeriphClkInit->RngClockSelection   = __HAL_RCC_GET_RNG_SOURCE();

#if !defined(STM32L412xx) && !defined(STM32L422xx)

  PeriphClkInit->AdcClockSelection     = __HAL_RCC_GET_ADC_SOURCE();
#endif

#if defined(SWPMI1)

  PeriphClkInit->Swpmi1ClockSelection  = __HAL_RCC_GET_SWPMI1_SOURCE();
#endif

#if defined(DFSDM1_Filter0)

  PeriphClkInit->Dfsdm1ClockSelection  = __HAL_RCC_GET_DFSDM1_SOURCE();

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)

  PeriphClkInit->Dfsdm1AudioClockSelection  = __HAL_RCC_GET_DFSDM1AUDIO_SOURCE();
#endif
#endif

#if defined(LTDC)

  PeriphClkInit->LtdcClockSelection = __HAL_RCC_GET_LTDC_SOURCE();
#endif

#if defined(DSI)

  PeriphClkInit->DsiClockSelection = __HAL_RCC_GET_DSI_SOURCE();
#endif

#if defined(OCTOSPI1) || defined(OCTOSPI2)

  PeriphClkInit->OspiClockSelection = __HAL_RCC_GET_OSPI_SOURCE();
#endif
}

/**
  * @brief  Return the peripheral clock frequency for peripherals with clock source from PLLSAIs
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk  Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_PERIPHCLK_RTC  RTC peripheral clock
  *            @arg @ref RCC_PERIPHCLK_ADC  ADC peripheral clock
  @if STM32L462xx
  *            @arg @ref RCC_PERIPHCLK_DFSDM1  DFSDM1 peripheral clock (only for devices with DFSDM)
  @endif
  @if STM32L486xx
  *            @arg @ref RCC_PERIPHCLK_DFSDM1  DFSDM1 peripheral clock (only for devices with DFSDM)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_DFSDM1  DFSDM1 peripheral clock (only for devices with DFSDM)
  @endif
  *            @arg @ref RCC_PERIPHCLK_I2C1  I2C1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C2  I2C2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C3  I2C3 peripheral clock
  @if STM32L462xx
  *            @arg @ref RCC_PERIPHCLK_I2C4  I2C4 peripheral clock (only for devices with I2C4)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_I2C4  I2C4 peripheral clock (only for devices with I2C4)
  @endif
  @if STM32L4S9xx
  *            @arg @ref RCC_PERIPHCLK_I2C4  I2C4 peripheral clock (only for devices with I2C4)
  @endif
  *            @arg @ref RCC_PERIPHCLK_LPTIM1  LPTIM1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPTIM2  LPTIM2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPUART1  LPUART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_RNG  RNG peripheral clock
  *            @arg @ref RCC_PERIPHCLK_SAI1  SAI1 peripheral clock (only for devices with SAI1)
  @if STM32L486xx
  *            @arg @ref RCC_PERIPHCLK_SAI2  SAI2 peripheral clock (only for devices with SAI2)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_SAI2  SAI2 peripheral clock (only for devices with SAI2)
  @endif
  @if STM32L4S9xx
  *            @arg @ref RCC_PERIPHCLK_SAI2  SAI2 peripheral clock (only for devices with SAI2)
  @endif
  *            @arg @ref RCC_PERIPHCLK_SDMMC1  SDMMC1 peripheral clock
  @if STM32L443xx
  *            @arg @ref RCC_PERIPHCLK_SWPMI1  SWPMI1 peripheral clock (only for devices with SWPMI1)
  @endif
  @if STM32L486xx
  *            @arg @ref RCC_PERIPHCLK_SWPMI1  SWPMI1 peripheral clock (only for devices with SWPMI1)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_SWPMI1  SWPMI1 peripheral clock (only for devices with SWPMI1)
  @endif
  *            @arg @ref RCC_PERIPHCLK_USART1  USART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USART2  USART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USART3  USART1 peripheral clock
  @if STM32L462xx
  *            @arg @ref RCC_PERIPHCLK_UART4  UART4 peripheral clock (only for devices with UART4)
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock (only for devices with USB)
  @endif
  @if STM32L486xx
  *            @arg @ref RCC_PERIPHCLK_UART4  UART4 peripheral clock (only for devices with UART4)
  *            @arg @ref RCC_PERIPHCLK_UART5  UART5 peripheral clock (only for devices with UART5)
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock (only for devices with USB)
  @endif
  @if STM32L4A6xx
  *            @arg @ref RCC_PERIPHCLK_UART4  UART4 peripheral clock (only for devices with UART4)
  *            @arg @ref RCC_PERIPHCLK_UART5  UART5 peripheral clock (only for devices with UART5)
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock (only for devices with USB)
  @endif
  @if STM32L4S9xx
  *            @arg @ref RCC_PERIPHCLK_UART4  USART1 peripheral clock (only for devices with UART4)
  *            @arg @ref RCC_PERIPHCLK_UART5  USART1 peripheral clock (only for devices with UART5)
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock (only for devices with USB)
  *            @arg @ref RCC_PERIPHCLK_DFSDM1  DFSDM1 peripheral kernel clock (only for devices with DFSDM1)
  *            @arg @ref RCC_PERIPHCLK_DFSDM1AUDIO  DFSDM1 peripheral audio clock (only for devices with DFSDM1)
  *            @arg @ref RCC_PERIPHCLK_LTDC  LTDC peripheral clock (only for devices with LTDC)
  *            @arg @ref RCC_PERIPHCLK_DSI  DSI peripheral clock (only for devices with DSI)
  *            @arg @ref RCC_PERIPHCLK_OSPI  OctoSPI peripheral clock (only for devices with OctoSPI)
  @endif
  * @retval Frequency in Hz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  uint32_t frequency = 0U;
  uint32_t srcclk, pll_oscsource, pllvco, plln;
#if defined(SDMMC1) && defined(RCC_CCIPR2_SDMMCSEL)
  uint32_t pllp;
#endif


  assert_param(IS_RCC_PERIPHCLOCK(PeriphClk));

  if(PeriphClk == RCC_PERIPHCLK_RTC)
  {

    srcclk = __HAL_RCC_GET_RTC_SOURCE();

    switch(srcclk)
    {
    case RCC_RTCCLKSOURCE_LSE:

      if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
      {
        frequency = LSE_VALUE;
      }
      break;
    case RCC_RTCCLKSOURCE_LSI:

      if(HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY))
      {
#if defined(RCC_CSR_LSIPREDIV)
        if(HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIPREDIV))
        {
          frequency = LSI_VALUE/128U;
        }
        else
#endif
        {
          frequency = LSI_VALUE;
        }
      }
      break;
    case RCC_RTCCLKSOURCE_HSE_DIV32:

      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSERDY))
      {
        frequency = HSE_VALUE / 32U;
      }
      break;
    default:

      break;
    }
  }
  else
  {

    pll_oscsource = __HAL_RCC_GET_PLL_OSCSOURCE();


    switch(pll_oscsource)
    {
    case RCC_PLLSOURCE_MSI:
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_MSIRDY))
      {

        pllvco = MSIRangeTable[(__HAL_RCC_GET_MSI_RANGE() >> 4U)];
      }
      else
      {
        pllvco = 0U;
      }
      break;
    case RCC_PLLSOURCE_HSI:
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
      {
        pllvco = HSI_VALUE;
      }
      else
      {
        pllvco = 0U;
      }
      break;
    case RCC_PLLSOURCE_HSE:
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSERDY))
      {
        pllvco = HSE_VALUE;
      }
      else
      {
        pllvco = 0U;
      }
      break;
    default:

      pllvco = 0U;
      break;
    }

    switch(PeriphClk)
    {
#if defined(SAI1)

    case RCC_PERIPHCLK_SAI1:
      frequency = RCCEx_GetSAIxPeriphCLKFreq(RCC_PERIPHCLK_SAI1, pllvco);
      break;

#endif

#if defined(SAI2)

    case RCC_PERIPHCLK_SAI2:
      frequency = RCCEx_GetSAIxPeriphCLKFreq(RCC_PERIPHCLK_SAI2, pllvco);
      break;

#endif

#if defined(USB_OTG_FS) || defined(USB)

    case RCC_PERIPHCLK_USB:

#endif

    case RCC_PERIPHCLK_RNG:

#if defined(SDMMC1) && !defined(RCC_CCIPR2_SDMMCSEL)

    case RCC_PERIPHCLK_SDMMC1:

#endif
      {
        srcclk = READ_BIT(RCC->CCIPR, RCC_CCIPR_CLK48SEL);

        switch(srcclk)
        {
        case RCC_CCIPR_CLK48SEL:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_MSIRDY))
          {

            frequency = MSIRangeTable[(__HAL_RCC_GET_MSI_RANGE() >> 4U)];
          }
          break;
        case RCC_CCIPR_CLK48SEL_1:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLRDY))
          {
            if(HAL_IS_BIT_SET(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN))
            {

              plln = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
              pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));

              frequency = (pllvco / (((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos) + 1U) << 1U));
            }
          }
          break;
#if defined(RCC_PLLSAI1_SUPPORT)
        case RCC_CCIPR_CLK48SEL_0:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLSAI1RDY))
          {
            if(HAL_IS_BIT_SET(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1QEN))
            {
              plln = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos;
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)


              pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1M) >> RCC_PLLSAI1CFGR_PLLSAI1M_Pos) + 1U));
#else

              pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));
#endif

              frequency = (pllvco / (((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q) >> RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) + 1U) << 1U));
            }
          }
          break;
#endif
#if defined(RCC_HSI48_SUPPORT)
        case 0U:
          if(HAL_IS_BIT_SET(RCC->CRRCR, RCC_CRRCR_HSI48RDY))
          {
            frequency = HSI48_VALUE;
          }
          break;
#endif
        default:

          break;
        }
        break;
      }

#if defined(SDMMC1) && defined(RCC_CCIPR2_SDMMCSEL)

    case RCC_PERIPHCLK_SDMMC1:

      if(HAL_IS_BIT_SET(RCC->CCIPR2, RCC_CCIPR2_SDMMCSEL))
      {
        if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLRDY))
        {
          if(HAL_IS_BIT_SET(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN))
          {

            plln = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
            pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));

            pllp = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLPDIV) >> RCC_PLLCFGR_PLLPDIV_Pos;
            if(pllp == 0U)
            {
              if(READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP) != 0U)
              {
                pllp = 17U;
              }
              else
              {
                pllp = 7U;
              }
            }
            frequency = (pllvco / pllp);
          }
        }
      }
      else
      {
        srcclk = READ_BIT(RCC->CCIPR, RCC_CCIPR_CLK48SEL);

        switch(srcclk)
        {
        case RCC_CCIPR_CLK48SEL:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_MSIRDY))
          {

            frequency = MSIRangeTable[(__HAL_RCC_GET_MSI_RANGE() >> 4U)];
          }
          break;
        case RCC_CCIPR_CLK48SEL_1:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLRDY))
          {
            if(HAL_IS_BIT_SET(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN))
            {

              plln = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
              pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));

              frequency = (pllvco / (((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos) + 1U) << 1U));
            }
          }
          break;
        case RCC_CCIPR_CLK48SEL_0:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLSAI1RDY))
          {
            if(HAL_IS_BIT_SET(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1QEN))
            {

              plln = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos;
              pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1M) >> RCC_PLLSAI1CFGR_PLLSAI1M_Pos) + 1U));

              frequency = (pllvco / (((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q) >> RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) + 1U) << 1U));
            }
          }
          break;
        case 0U:
          if(HAL_IS_BIT_SET(RCC->CRRCR, RCC_CRRCR_HSI48RDY))
          {
            frequency = HSI48_VALUE;
          }
          break;
        default:

          break;
        }
      }
      break;

#endif

    case RCC_PERIPHCLK_USART1:
      {

        srcclk = __HAL_RCC_GET_USART1_SOURCE();

        switch(srcclk)
        {
        case RCC_USART1CLKSOURCE_PCLK2:
          frequency = HAL_RCC_GetPCLK2Freq();
          break;
        case RCC_USART1CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_USART1CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        case RCC_USART1CLKSOURCE_LSE:
          if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
          {
            frequency = LSE_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

    case RCC_PERIPHCLK_USART2:
      {

        srcclk = __HAL_RCC_GET_USART2_SOURCE();

        switch(srcclk)
        {
        case RCC_USART2CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_USART2CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_USART2CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        case RCC_USART2CLKSOURCE_LSE:
          if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
          {
            frequency = LSE_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#if defined(USART3)

    case RCC_PERIPHCLK_USART3:
      {

        srcclk = __HAL_RCC_GET_USART3_SOURCE();

        switch(srcclk)
        {
        case RCC_USART3CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_USART3CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_USART3CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        case RCC_USART3CLKSOURCE_LSE:
          if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
          {
            frequency = LSE_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#endif

#if defined(UART4)

    case RCC_PERIPHCLK_UART4:
      {

        srcclk = __HAL_RCC_GET_UART4_SOURCE();

        switch(srcclk)
        {
        case RCC_UART4CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_UART4CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_UART4CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        case RCC_UART4CLKSOURCE_LSE:
          if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
          {
            frequency = LSE_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#endif

#if defined(UART5)

    case RCC_PERIPHCLK_UART5:
      {

        srcclk = __HAL_RCC_GET_UART5_SOURCE();

        switch(srcclk)
        {
        case RCC_UART5CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_UART5CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_UART5CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        case RCC_UART5CLKSOURCE_LSE:
          if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
          {
            frequency = LSE_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#endif

    case RCC_PERIPHCLK_LPUART1:
      {

        srcclk = __HAL_RCC_GET_LPUART1_SOURCE();

        switch(srcclk)
        {
        case RCC_LPUART1CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_LPUART1CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_LPUART1CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        case RCC_LPUART1CLKSOURCE_LSE:
          if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
          {
            frequency = LSE_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

    case RCC_PERIPHCLK_ADC:
      {
        srcclk = __HAL_RCC_GET_ADC_SOURCE();

        switch(srcclk)
        {
        case RCC_ADCCLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
#if defined(RCC_PLLSAI1_SUPPORT)
        case RCC_ADCCLKSOURCE_PLLSAI1:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLSAI1RDY) && (__HAL_RCC_GET_PLLSAI1CLKOUT_CONFIG(RCC_PLLSAI1_ADC1CLK) != 0U))
          {
            plln = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos;
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)


            pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1M) >> RCC_PLLSAI1CFGR_PLLSAI1M_Pos) + 1U));
#else

            pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));
#endif

            frequency = (pllvco / (((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R) >> RCC_PLLSAI1CFGR_PLLSAI1R_Pos) + 1U) << 1U));
          }
          break;
#endif
#if defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx) || defined(STM32L496xx) || defined(STM32L4A6xx)
        case RCC_ADCCLKSOURCE_PLLSAI2:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLSAI2RDY) && (__HAL_RCC_GET_PLLSAI2CLKOUT_CONFIG(RCC_PLLSAI2_ADC2CLK) != 0U))
          {
            plln = READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N) >> RCC_PLLSAI2CFGR_PLLSAI2N_Pos;
#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)


            pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2M) >> RCC_PLLSAI2CFGR_PLLSAI2M_Pos) + 1U));
#else

            pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));
#endif

            frequency = (pllvco / (((READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2R) >> RCC_PLLSAI2CFGR_PLLSAI2R_Pos) + 1U) << 1U));
          }
          break;
#endif
        default:

          break;
        }

        break;
      }

#if defined(DFSDM1_Filter0)

    case RCC_PERIPHCLK_DFSDM1:
      {

        srcclk = __HAL_RCC_GET_DFSDM1_SOURCE();

        if(srcclk == RCC_DFSDM1CLKSOURCE_PCLK2)
        {
          frequency = HAL_RCC_GetPCLK2Freq();
        }
        else
        {
          frequency = HAL_RCC_GetSysClockFreq();
        }

        break;
      }

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)

    case RCC_PERIPHCLK_DFSDM1AUDIO:
      {

        srcclk = __HAL_RCC_GET_DFSDM1AUDIO_SOURCE();

        switch(srcclk)
        {
        case RCC_DFSDM1AUDIOCLKSOURCE_SAI1:
          frequency = RCCEx_GetSAIxPeriphCLKFreq(RCC_PERIPHCLK_SAI1, pllvco);
          break;
        case RCC_DFSDM1AUDIOCLKSOURCE_MSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_MSIRDY))
          {

            frequency = MSIRangeTable[(__HAL_RCC_GET_MSI_RANGE() >> 4U)];
          }
          break;
        case RCC_DFSDM1AUDIOCLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#endif

#endif

    case RCC_PERIPHCLK_I2C1:
      {

        srcclk = __HAL_RCC_GET_I2C1_SOURCE();

        switch(srcclk)
        {
        case RCC_I2C1CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_I2C1CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_I2C1CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#if defined(I2C2)

    case RCC_PERIPHCLK_I2C2:
      {

        srcclk = __HAL_RCC_GET_I2C2_SOURCE();

        switch(srcclk)
        {
        case RCC_I2C2CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_I2C2CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_I2C2CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#endif

    case RCC_PERIPHCLK_I2C3:
      {

        srcclk = __HAL_RCC_GET_I2C3_SOURCE();

        switch(srcclk)
        {
        case RCC_I2C3CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_I2C3CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_I2C3CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#if defined(I2C4)

    case RCC_PERIPHCLK_I2C4:
      {

        srcclk = __HAL_RCC_GET_I2C4_SOURCE();

        switch(srcclk)
        {
        case RCC_I2C4CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_I2C4CLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_I2C4CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#endif

    case RCC_PERIPHCLK_LPTIM1:
      {

        srcclk = __HAL_RCC_GET_LPTIM1_SOURCE();

        switch(srcclk)
        {
        case RCC_LPTIM1CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_LPTIM1CLKSOURCE_LSI:
          if(HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY))
          {
#if defined(RCC_CSR_LSIPREDIV)
            if(HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIPREDIV))
            {
              frequency = LSI_VALUE/128U;
            }
            else
#endif
            {
              frequency = LSI_VALUE;
            }
          }
          break;
        case RCC_LPTIM1CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        case RCC_LPTIM1CLKSOURCE_LSE:
          if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
          {
            frequency = LSE_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

    case RCC_PERIPHCLK_LPTIM2:
      {

       srcclk = __HAL_RCC_GET_LPTIM2_SOURCE();

        switch(srcclk)
        {
        case RCC_LPTIM2CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_LPTIM2CLKSOURCE_LSI:
          if(HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY))
          {
#if defined(RCC_CSR_LSIPREDIV)
            if(HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIPREDIV))
            {
              frequency = LSI_VALUE/128U;
            }
            else
#endif
            {
              frequency = LSI_VALUE;
            }
          }
          break;
        case RCC_LPTIM2CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        case RCC_LPTIM2CLKSOURCE_LSE:
          if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
          {
            frequency = LSE_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#if defined(SWPMI1)

    case RCC_PERIPHCLK_SWPMI1:
      {

        srcclk = __HAL_RCC_GET_SWPMI1_SOURCE();

        switch(srcclk)
        {
        case RCC_SWPMI1CLKSOURCE_PCLK1:
          frequency = HAL_RCC_GetPCLK1Freq();
          break;
        case RCC_SWPMI1CLKSOURCE_HSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
          {
            frequency = HSI_VALUE;
          }
          break;
        default:

          break;
        }

        break;
      }

#endif

#if defined(OCTOSPI1) || defined(OCTOSPI2)

    case RCC_PERIPHCLK_OSPI:
      {

        srcclk = __HAL_RCC_GET_OSPI_SOURCE();

        switch(srcclk)
        {
        case RCC_OSPICLKSOURCE_SYSCLK:
          frequency = HAL_RCC_GetSysClockFreq();
          break;
        case RCC_OSPICLKSOURCE_MSI:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_MSIRDY))
          {

            frequency = MSIRangeTable[(__HAL_RCC_GET_MSI_RANGE() >> 4U)];
          }
          break;
        case RCC_OSPICLKSOURCE_PLL:
          if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLRDY))
          {
            if(HAL_IS_BIT_SET(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN))
            {

              plln = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
              pllvco = ((pllvco * plln) / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));

              frequency = (pllvco / (((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos) + 1U) << 1U));
            }
          }
          break;
        default:

          break;
        }

        break;
      }

#endif

    default:
      break;
    }
  }

  return(frequency);
}

/**
  * @}
  */

/** @defgroup RCCEx_Exported_Functions_Group2 Extended Clock management functions
 *  @brief  Extended Clock management functions
 *
@verbatim
 ===============================================================================
                ##### Extended clock management functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the
    activation or deactivation of MSI PLL-mode, PLLSAI1, PLLSAI2, LSE CSS,
    Low speed clock output and clock after wake-up from STOP mode.
@endverbatim
  * @{
  */

#if defined(RCC_PLLSAI1_SUPPORT)

/**
  * @brief  Enable PLLSAI1.
  * @param  PLLSAI1Init  pointer to an RCC_PLLSAI1InitTypeDef structure that
  *         contains the configuration information for the PLLSAI1
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_EnablePLLSAI1(RCC_PLLSAI1InitTypeDef  *PLLSAI1Init)
{
  uint32_t tickstart;
  HAL_StatusTypeDef status = HAL_OK;


  assert_param(IS_RCC_PLLSAI1SOURCE(PLLSAI1Init->PLLSAI1Source));
  assert_param(IS_RCC_PLLSAI1M_VALUE(PLLSAI1Init->PLLSAI1M));
  assert_param(IS_RCC_PLLSAI1N_VALUE(PLLSAI1Init->PLLSAI1N));
  assert_param(IS_RCC_PLLSAI1P_VALUE(PLLSAI1Init->PLLSAI1P));
  assert_param(IS_RCC_PLLSAI1Q_VALUE(PLLSAI1Init->PLLSAI1Q));
  assert_param(IS_RCC_PLLSAI1R_VALUE(PLLSAI1Init->PLLSAI1R));
  assert_param(IS_RCC_PLLSAI1CLOCKOUT_VALUE(PLLSAI1Init->PLLSAI1ClockOut));


  __HAL_RCC_PLLSAI1_DISABLE();


  tickstart = HAL_GetTick();


  while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) != 0U)
  {
    if((HAL_GetTick() - tickstart) > PLLSAI1_TIMEOUT_VALUE)
    {
      status = HAL_TIMEOUT;
      break;
    }
  }

  if(status == HAL_OK)
  {
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)


    __HAL_RCC_PLLSAI1_CONFIG(PLLSAI1Init->PLLSAI1M, PLLSAI1Init->PLLSAI1N, PLLSAI1Init->PLLSAI1P, PLLSAI1Init->PLLSAI1Q, PLLSAI1Init->PLLSAI1R);
#else


    __HAL_RCC_PLLSAI1_CONFIG(PLLSAI1Init->PLLSAI1N, PLLSAI1Init->PLLSAI1P, PLLSAI1Init->PLLSAI1Q, PLLSAI1Init->PLLSAI1R);
#endif

    __HAL_RCC_PLLSAI1CLKOUT_ENABLE(PLLSAI1Init->PLLSAI1ClockOut);


    __HAL_RCC_PLLSAI1_ENABLE();


    tickstart = HAL_GetTick();


    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) == 0U)
    {
      if((HAL_GetTick() - tickstart) > PLLSAI1_TIMEOUT_VALUE)
      {
        status = HAL_TIMEOUT;
        break;
      }
    }
  }

  return status;
}

/**
  * @brief  Disable PLLSAI1.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_DisablePLLSAI1(void)
{
  uint32_t tickstart;
  HAL_StatusTypeDef status = HAL_OK;


  __HAL_RCC_PLLSAI1_DISABLE();


  tickstart = HAL_GetTick();


  while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) != 0U)
  {
    if((HAL_GetTick() - tickstart) > PLLSAI1_TIMEOUT_VALUE)
    {
      status = HAL_TIMEOUT;
      break;
    }
  }


  __HAL_RCC_PLLSAI1CLKOUT_DISABLE(RCC_PLLSAI1CFGR_PLLSAI1PEN|RCC_PLLSAI1CFGR_PLLSAI1QEN|RCC_PLLSAI1CFGR_PLLSAI1REN);


#if defined(RCC_PLLSAI2_SUPPORT)
  if(READ_BIT(RCC->CR, (RCC_CR_PLLRDY | RCC_CR_PLLSAI2RDY)) == 0U)
  {
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, RCC_PLLSOURCE_NONE);
  }
#else
  if(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0U)
  {
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, RCC_PLLSOURCE_NONE);
  }
#endif

  return status;
}

#endif

#if defined(RCC_PLLSAI2_SUPPORT)

/**
  * @brief  Enable PLLSAI2.
  * @param  PLLSAI2Init  pointer to an RCC_PLLSAI2InitTypeDef structure that
  *         contains the configuration information for the PLLSAI2
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_EnablePLLSAI2(RCC_PLLSAI2InitTypeDef  *PLLSAI2Init)
{
  uint32_t tickstart;
  HAL_StatusTypeDef status = HAL_OK;


  assert_param(IS_RCC_PLLSAI2SOURCE(PLLSAI2Init->PLLSAI2Source));
  assert_param(IS_RCC_PLLSAI2M_VALUE(PLLSAI2Init->PLLSAI2M));
  assert_param(IS_RCC_PLLSAI2N_VALUE(PLLSAI2Init->PLLSAI2N));
  assert_param(IS_RCC_PLLSAI2P_VALUE(PLLSAI2Init->PLLSAI2P));
#if defined(RCC_PLLSAI2Q_DIV_SUPPORT)
  assert_param(IS_RCC_PLLSAI2Q_VALUE(PLLSAI2Init->PLLSAI2Q));
#endif
  assert_param(IS_RCC_PLLSAI2R_VALUE(PLLSAI2Init->PLLSAI2R));
  assert_param(IS_RCC_PLLSAI2CLOCKOUT_VALUE(PLLSAI2Init->PLLSAI2ClockOut));


  __HAL_RCC_PLLSAI2_DISABLE();


  tickstart = HAL_GetTick();


  while(READ_BIT(RCC->CR, RCC_CR_PLLSAI2RDY) != 0U)
  {
    if((HAL_GetTick() - tickstart) > PLLSAI2_TIMEOUT_VALUE)
    {
      status = HAL_TIMEOUT;
      break;
    }
  }

  if(status == HAL_OK)
  {
#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT) && defined(RCC_PLLSAI2Q_DIV_SUPPORT)


    __HAL_RCC_PLLSAI2_CONFIG(PLLSAI2Init->PLLSAI2M, PLLSAI2Init->PLLSAI2N, PLLSAI2Init->PLLSAI2P, PLLSAI2Init->PLLSAI2Q, PLLSAI2Init->PLLSAI2R);
#elif defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)


    __HAL_RCC_PLLSAI2_CONFIG(PLLSAI2Init->PLLSAI2M, PLLSAI2Init->PLLSAI2N, PLLSAI2Init->PLLSAI2P, PLLSAI2Init->PLLSAI2R);
#elif defined(RCC_PLLSAI2Q_DIV_SUPPORT)


    __HAL_RCC_PLLSAI2_CONFIG(PLLSAI2Init->PLLSAI2N, PLLSAI2Init->PLLSAI2P, PLLSAI2Init->PLLSAI2Q, PLLSAI2Init->PLLSAI2R);
#else


    __HAL_RCC_PLLSAI2_CONFIG(PLLSAI2Init->PLLSAI2N, PLLSAI2Init->PLLSAI2P, PLLSAI2Init->PLLSAI2R);
#endif

    __HAL_RCC_PLLSAI2CLKOUT_ENABLE(PLLSAI2Init->PLLSAI2ClockOut);


    __HAL_RCC_PLLSAI2_ENABLE();


    tickstart = HAL_GetTick();


    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI2RDY) == 0U)
    {
      if((HAL_GetTick() - tickstart) > PLLSAI2_TIMEOUT_VALUE)
      {
        status = HAL_TIMEOUT;
        break;
      }
    }
  }

  return status;
}

/**
  * @brief  Disable PLLISAI2.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_DisablePLLSAI2(void)
{
  uint32_t tickstart;
  HAL_StatusTypeDef status = HAL_OK;


  __HAL_RCC_PLLSAI2_DISABLE();


  tickstart = HAL_GetTick();


  while(READ_BIT(RCC->CR, RCC_CR_PLLSAI2RDY) != 0U)
  {
    if((HAL_GetTick() - tickstart) > PLLSAI2_TIMEOUT_VALUE)
    {
      status = HAL_TIMEOUT;
      break;
    }
  }


#if defined(RCC_PLLSAI2Q_DIV_SUPPORT)
  __HAL_RCC_PLLSAI2CLKOUT_DISABLE(RCC_PLLSAI2CFGR_PLLSAI2PEN|RCC_PLLSAI2CFGR_PLLSAI2QEN|RCC_PLLSAI2CFGR_PLLSAI2REN);
#else
  __HAL_RCC_PLLSAI2CLKOUT_DISABLE(RCC_PLLSAI2CFGR_PLLSAI2PEN|RCC_PLLSAI2CFGR_PLLSAI2REN);
#endif


  if(READ_BIT(RCC->CR, (RCC_CR_PLLRDY | RCC_CR_PLLSAI1RDY)) == 0U)
  {
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, RCC_PLLSOURCE_NONE);
  }

  return status;
}

#endif

/**
  * @brief  Configure the oscillator clock source for wakeup from Stop and CSS backup clock.
  * @param  WakeUpClk  Wakeup clock
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_STOP_WAKEUPCLOCK_MSI  MSI oscillator selection
  *            @arg @ref RCC_STOP_WAKEUPCLOCK_HSI  HSI oscillator selection
  * @note   This function shall not be called after the Clock Security System on HSE has been
  *         enabled.
  * @retval None
  */
void HAL_RCCEx_WakeUpStopCLKConfig(uint32_t WakeUpClk)
{
  assert_param(IS_RCC_STOP_WAKEUPCLOCK(WakeUpClk));

  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(WakeUpClk);
}

/**
  * @brief  Configure the MSI range after standby mode.
  * @note   After Standby its frequency can be selected between 4 possible values (1, 2, 4 or 8 MHz).
  * @param  MSIRange  MSI range
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_MSIRANGE_4  Range 4 around 1 MHz
  *            @arg @ref RCC_MSIRANGE_5  Range 5 around 2 MHz
  *            @arg @ref RCC_MSIRANGE_6  Range 6 around 4 MHz (reset value)
  *            @arg @ref RCC_MSIRANGE_7  Range 7 around 8 MHz
  * @retval None
  */
void HAL_RCCEx_StandbyMSIRangeConfig(uint32_t MSIRange)
{
  assert_param(IS_RCC_MSI_STANDBY_CLOCK_RANGE(MSIRange));

  __HAL_RCC_MSI_STANDBY_RANGE_CONFIG(MSIRange);
}

/**
  * @brief  Enable the LSE Clock Security System.
  * @note   Prior to enable the LSE Clock Security System, LSE oscillator is to be enabled
  *         with HAL_RCC_OscConfig() and the LSE oscillator clock is to be selected as RTC
  *         clock with HAL_RCCEx_PeriphCLKConfig().
  * @retval None
  */
void HAL_RCCEx_EnableLSECSS(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_LSECSSON);
}

/**
  * @brief  Disable the LSE Clock Security System.
  * @note   LSE Clock Security System can only be disabled after a LSE failure detection.
  * @retval None
  */
void HAL_RCCEx_DisableLSECSS(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSECSSON) ;


  __HAL_RCC_DISABLE_IT(RCC_IT_LSECSS);
}

/**
  * @brief  Enable the LSE Clock Security System Interrupt & corresponding EXTI line.
  * @note   LSE Clock Security System Interrupt is mapped on RTC EXTI line 19
  * @retval None
  */
void HAL_RCCEx_EnableLSECSS_IT(void)
{

  SET_BIT(RCC->BDCR, RCC_BDCR_LSECSSON) ;


  __HAL_RCC_ENABLE_IT(RCC_IT_LSECSS);


  __HAL_RCC_LSECSS_EXTI_ENABLE_IT();
  __HAL_RCC_LSECSS_EXTI_ENABLE_RISING_EDGE();
}

/**
  * @brief Handle the RCC LSE Clock Security System interrupt request.
  * @retval None
  */
void HAL_RCCEx_LSECSS_IRQHandler(void)
{

  if(__HAL_RCC_GET_IT(RCC_IT_LSECSS))
  {

    HAL_RCCEx_LSECSS_Callback();


    __HAL_RCC_CLEAR_IT(RCC_IT_LSECSS);
  }
}

/**
  * @brief  RCCEx LSE Clock Security System interrupt callback.
  * @retval none
  */
__weak void HAL_RCCEx_LSECSS_Callback(void)
{
  /* NOTE : This function should not be modified, when the callback is needed,
            the @ref HAL_RCCEx_LSECSS_Callback should be implemented in the user file
   */
}

/**
  * @brief  Select the Low Speed clock source to output on LSCO pin (PA2).
  * @param  LSCOSource  specifies the Low Speed clock source to output.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_LSCOSOURCE_LSI  LSI clock selected as LSCO source
  *            @arg @ref RCC_LSCOSOURCE_LSE  LSE clock selected as LSCO source
  * @retval None
  */
void HAL_RCCEx_EnableLSCO(uint32_t LSCOSource)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  FlagStatus       pwrclkchanged = RESET;
  FlagStatus       backupchanged = RESET;


  assert_param(IS_RCC_LSCOSOURCE(LSCOSource));


  __LSCO_CLK_ENABLE();


  GPIO_InitStruct.Pin = LSCO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSCO_GPIO_PORT, &GPIO_InitStruct);


  if(__HAL_RCC_PWR_IS_CLK_DISABLED())
  {
    __HAL_RCC_PWR_CLK_ENABLE();
    pwrclkchanged = SET;
  }
  if(HAL_IS_BIT_CLR(PWR->CR1, PWR_CR1_DBP))
  {
    HAL_PWR_EnableBkUpAccess();
    backupchanged = SET;
  }

  MODIFY_REG(RCC->BDCR, RCC_BDCR_LSCOSEL | RCC_BDCR_LSCOEN, LSCOSource | RCC_BDCR_LSCOEN);

  if(backupchanged == SET)
  {
    HAL_PWR_DisableBkUpAccess();
  }
  if(pwrclkchanged == SET)
  {
    __HAL_RCC_PWR_CLK_DISABLE();
  }
}

/**
  * @brief  Disable the Low Speed clock output.
  * @retval None
  */
void HAL_RCCEx_DisableLSCO(void)
{
  FlagStatus       pwrclkchanged = RESET;
  FlagStatus       backupchanged = RESET;


  if(__HAL_RCC_PWR_IS_CLK_DISABLED())
  {
    __HAL_RCC_PWR_CLK_ENABLE();
    pwrclkchanged = SET;
  }
  if(HAL_IS_BIT_CLR(PWR->CR1, PWR_CR1_DBP))
  {

    HAL_PWR_EnableBkUpAccess();
    backupchanged = SET;
  }

  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSCOEN);


  if(backupchanged == SET)
  {

    HAL_PWR_DisableBkUpAccess();
  }
  if(pwrclkchanged == SET)
  {
    __HAL_RCC_PWR_CLK_DISABLE();
  }
}

/**
  * @brief  Enable the PLL-mode of the MSI.
  * @note   Prior to enable the PLL-mode of the MSI for automatic hardware
  *         calibration LSE oscillator is to be enabled with HAL_RCC_OscConfig().
  * @retval None
  */
void HAL_RCCEx_EnableMSIPLLMode(void)
{
  SET_BIT(RCC->CR, RCC_CR_MSIPLLEN) ;
}

/**
  * @brief  Disable the PLL-mode of the MSI.
  * @note   PLL-mode of the MSI is automatically reset when LSE oscillator is disabled.
  * @retval None
  */
void HAL_RCCEx_DisableMSIPLLMode(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_MSIPLLEN) ;
}

#if defined (OCTOSPI1) && defined (OCTOSPI2)
/**
  * @brief  Configure OCTOSPI instances DQS delays.
  * @param  Delay1  OCTOSPI1 DQS delay
  * @param  Delay2  OCTOSPI2 DQS delay
  * @note   Delay parameters stand for unitary delays from 0 to 15. Actual delay is Delay1 or Delay2 + 1.
  * @retval None
  */
void HAL_RCCEx_OCTOSPIDelayConfig(uint32_t Delay1, uint32_t Delay2)
{
  assert_param(IS_RCC_OCTOSPIDELAY(Delay1));
  assert_param(IS_RCC_OCTOSPIDELAY(Delay2));

  MODIFY_REG(RCC->DLYCFGR, RCC_DLYCFGR_OCTOSPI1_DLY|RCC_DLYCFGR_OCTOSPI2_DLY, (Delay1 | (Delay2 << RCC_DLYCFGR_OCTOSPI2_DLY_Pos))) ;
}
#endif

/**
  * @}
  */

#if defined(CRS)

/** @defgroup RCCEx_Exported_Functions_Group3 Extended Clock Recovery System Control functions
 *  @brief  Extended Clock Recovery System Control functions
 *
@verbatim
 ===============================================================================
                ##### Extended Clock Recovery System Control functions  #####
 ===============================================================================
    [..]
      For devices with Clock Recovery System feature (CRS), RCC Extension HAL driver can be used as follows:

      (#) In System clock config, HSI48 needs to be enabled

      (#) Enable CRS clock in IP MSP init which will use CRS functions

      (#) Call CRS functions as follows:
          (##) Prepare synchronization configuration necessary for HSI48 calibration
              (+++) Default values can be set for frequency Error Measurement (reload and error limit)
                        and also HSI48 oscillator smooth trimming.
              (+++) Macro __HAL_RCC_CRS_RELOADVALUE_CALCULATE can be also used to calculate
                        directly reload value with target and sychronization frequencies values
          (##) Call function HAL_RCCEx_CRSConfig which
              (+++) Resets CRS registers to their default values.
              (+++) Configures CRS registers with synchronization configuration
              (+++) Enables automatic calibration and frequency error counter feature
           Note: When using USB LPM (Link Power Management) and the device is in Sleep mode, the
           periodic USB SOF will not be generated by the host. No SYNC signal will therefore be
           provided to the CRS to calibrate the HSI48 on the run. To guarantee the required clock
           precision after waking up from Sleep mode, the LSE or reference clock on the GPIOs
           should be used as SYNC signal.

          (##) A polling function is provided to wait for complete synchronization
              (+++) Call function HAL_RCCEx_CRSWaitSynchronization()
              (+++) According to CRS status, user can decide to adjust again the calibration or continue
                        application if synchronization is OK

      (#) User can retrieve information related to synchronization in calling function
            HAL_RCCEx_CRSGetSynchronizationInfo()

      (#) Regarding synchronization status and synchronization information, user can try a new calibration
           in changing synchronization configuration and call again HAL_RCCEx_CRSConfig.
           Note: When the SYNC event is detected during the downcounting phase (before reaching the zero value),
           it means that the actual frequency is lower than the target (and so, that the TRIM value should be
           incremented), while when it is detected during the upcounting phase it means that the actual frequency
           is higher (and that the TRIM value should be decremented).

      (#) In interrupt mode, user can resort to the available macros (__HAL_RCC_CRS_XXX_IT). Interrupts will go
          through CRS Handler (CRS_IRQn/CRS_IRQHandler)
              (++) Call function HAL_RCCEx_CRSConfig()
              (++) Enable CRS_IRQn (thanks to NVIC functions)
              (++) Enable CRS interrupt (__HAL_RCC_CRS_ENABLE_IT)
              (++) Implement CRS status management in the following user callbacks called from
                   HAL_RCCEx_CRS_IRQHandler():
                   (+++) HAL_RCCEx_CRS_SyncOkCallback()
                   (+++) HAL_RCCEx_CRS_SyncWarnCallback()
                   (+++) HAL_RCCEx_CRS_ExpectedSyncCallback()
                   (+++) HAL_RCCEx_CRS_ErrorCallback()

      (#) To force a SYNC EVENT, user can use the function HAL_RCCEx_CRSSoftwareSynchronizationGenerate().
          This function can be called before calling HAL_RCCEx_CRSConfig (for instance in Systick handler)

@endverbatim
 * @{
 */

/**
  * @brief  Start automatic synchronization for polling mode
  * @param  pInit Pointer on RCC_CRSInitTypeDef structure
  * @retval None
  */
void HAL_RCCEx_CRSConfig(RCC_CRSInitTypeDef *pInit)
{
  uint32_t value;


  assert_param(IS_RCC_CRS_SYNC_DIV(pInit->Prescaler));
  assert_param(IS_RCC_CRS_SYNC_SOURCE(pInit->Source));
  assert_param(IS_RCC_CRS_SYNC_POLARITY(pInit->Polarity));
  assert_param(IS_RCC_CRS_RELOADVALUE(pInit->ReloadValue));
  assert_param(IS_RCC_CRS_ERRORLIMIT(pInit->ErrorLimitValue));
  assert_param(IS_RCC_CRS_HSI48CALIBRATION(pInit->HSI48CalibrationValue));




  __HAL_RCC_CRS_FORCE_RESET();
  __HAL_RCC_CRS_RELEASE_RESET();




  value = (pInit->Prescaler | pInit->Source | pInit->Polarity);

  value |= pInit->ReloadValue;

  value |= (pInit->ErrorLimitValue << CRS_CFGR_FELIM_Pos);
  WRITE_REG(CRS->CFGR, value);


  /* Set the TRIM[6:0] bits for STM32L412xx/L422xx or TRIM[5:0] bits otherwise
     according to RCC_CRS_HSI48CalibrationValue value */
  MODIFY_REG(CRS->CR, CRS_CR_TRIM, (pInit->HSI48CalibrationValue << CRS_CR_TRIM_Pos));




  SET_BIT(CRS->CR, CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
}

/**
  * @brief  Generate the software synchronization event
  * @retval None
  */
void HAL_RCCEx_CRSSoftwareSynchronizationGenerate(void)
{
  SET_BIT(CRS->CR, CRS_CR_SWSYNC);
}

/**
  * @brief  Return synchronization info
  * @param  pSynchroInfo Pointer on RCC_CRSSynchroInfoTypeDef structure
  * @retval None
  */
void HAL_RCCEx_CRSGetSynchronizationInfo(RCC_CRSSynchroInfoTypeDef *pSynchroInfo)
{

  assert_param(pSynchroInfo != (void *)NULL);


  pSynchroInfo->ReloadValue = (READ_BIT(CRS->CFGR, CRS_CFGR_RELOAD));


  pSynchroInfo->HSI48CalibrationValue = (READ_BIT(CRS->CR, CRS_CR_TRIM) >> CRS_CR_TRIM_Pos);


  pSynchroInfo->FreqErrorCapture = (READ_BIT(CRS->ISR, CRS_ISR_FECAP) >> CRS_ISR_FECAP_Pos);


  pSynchroInfo->FreqErrorDirection = (READ_BIT(CRS->ISR, CRS_ISR_FEDIR));
}

/**
* @brief Wait for CRS Synchronization status.
* @param Timeout  Duration of the timeout
* @note  Timeout is based on the maximum time to receive a SYNC event based on synchronization
*        frequency.
* @note    If Timeout set to HAL_MAX_DELAY, HAL_TIMEOUT will be never returned.
* @retval Combination of Synchronization status
*          This parameter can be a combination of the following values:
*            @arg @ref RCC_CRS_TIMEOUT
*            @arg @ref RCC_CRS_SYNCOK
*            @arg @ref RCC_CRS_SYNCWARN
*            @arg @ref RCC_CRS_SYNCERR
*            @arg @ref RCC_CRS_SYNCMISS
*            @arg @ref RCC_CRS_TRIMOVF
*/
uint32_t HAL_RCCEx_CRSWaitSynchronization(uint32_t Timeout)
{
  uint32_t crsstatus = RCC_CRS_NONE;
  uint32_t tickstart;


  tickstart = HAL_GetTick();


  do
  {
    if(Timeout != HAL_MAX_DELAY)
    {
      if(((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
      {
        crsstatus = RCC_CRS_TIMEOUT;
      }
    }

    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCOK))
    {

      crsstatus |= RCC_CRS_SYNCOK;


      __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_SYNCOK);
    }


    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCWARN))
    {

      crsstatus |= RCC_CRS_SYNCWARN;


      __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_SYNCWARN);
    }


    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_TRIMOVF))
    {

      crsstatus |= RCC_CRS_TRIMOVF;


      __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_TRIMOVF);
    }


    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCERR))
    {

      crsstatus |= RCC_CRS_SYNCERR;


      __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_SYNCERR);
    }


    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCMISS))
    {

      crsstatus |= RCC_CRS_SYNCMISS;


      __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_SYNCMISS);
    }


    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_ESYNC))
    {

      __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_ESYNC);
    }
  } while(RCC_CRS_NONE == crsstatus);

  return crsstatus;
}

/**
  * @brief Handle the Clock Recovery System interrupt request.
  * @retval None
  */
void HAL_RCCEx_CRS_IRQHandler(void)
{
  uint32_t crserror = RCC_CRS_NONE;

  uint32_t itflags = READ_REG(CRS->ISR);
  uint32_t itsources = READ_REG(CRS->CR);


  if(((itflags & RCC_CRS_FLAG_SYNCOK) != 0U) && ((itsources & RCC_CRS_IT_SYNCOK) != 0U))
  {

    WRITE_REG(CRS->ICR, CRS_ICR_SYNCOKC);


    HAL_RCCEx_CRS_SyncOkCallback();
  }

  else if(((itflags & RCC_CRS_FLAG_SYNCWARN) != 0U) && ((itsources & RCC_CRS_IT_SYNCWARN) != 0U))
  {

    WRITE_REG(CRS->ICR, CRS_ICR_SYNCWARNC);


    HAL_RCCEx_CRS_SyncWarnCallback();
  }

  else if(((itflags & RCC_CRS_FLAG_ESYNC) != 0U) && ((itsources & RCC_CRS_IT_ESYNC) != 0U))
  {

    WRITE_REG(CRS->ICR, CRS_ICR_ESYNCC);


    HAL_RCCEx_CRS_ExpectedSyncCallback();
  }

  else
  {
    if(((itflags & RCC_CRS_FLAG_ERR) != 0U) && ((itsources & RCC_CRS_IT_ERR) != 0U))
    {
      if((itflags & RCC_CRS_FLAG_SYNCERR) != 0U)
      {
        crserror |= RCC_CRS_SYNCERR;
      }
      if((itflags & RCC_CRS_FLAG_SYNCMISS) != 0U)
      {
        crserror |= RCC_CRS_SYNCMISS;
      }
      if((itflags & RCC_CRS_FLAG_TRIMOVF) != 0U)
      {
        crserror |= RCC_CRS_TRIMOVF;
      }


      WRITE_REG(CRS->ICR, CRS_ICR_ERRC);


      HAL_RCCEx_CRS_ErrorCallback(crserror);
    }
  }
}

/**
  * @brief  RCCEx Clock Recovery System SYNCOK interrupt callback.
  * @retval none
  */
__weak void HAL_RCCEx_CRS_SyncOkCallback(void)
{
  /* NOTE : This function should not be modified, when the callback is needed,
            the @ref HAL_RCCEx_CRS_SyncOkCallback should be implemented in the user file
   */
}

/**
  * @brief  RCCEx Clock Recovery System SYNCWARN interrupt callback.
  * @retval none
  */
__weak void HAL_RCCEx_CRS_SyncWarnCallback(void)
{
  /* NOTE : This function should not be modified, when the callback is needed,
            the @ref HAL_RCCEx_CRS_SyncWarnCallback should be implemented in the user file
   */
}

/**
  * @brief  RCCEx Clock Recovery System Expected SYNC interrupt callback.
  * @retval none
  */
__weak void HAL_RCCEx_CRS_ExpectedSyncCallback(void)
{
  /* NOTE : This function should not be modified, when the callback is needed,
            the @ref HAL_RCCEx_CRS_ExpectedSyncCallback should be implemented in the user file
   */
}

/**
  * @brief  RCCEx Clock Recovery System Error interrupt callback.
  * @param  Error Combination of Error status.
  *         This parameter can be a combination of the following values:
  *           @arg @ref RCC_CRS_SYNCERR
  *           @arg @ref RCC_CRS_SYNCMISS
  *           @arg @ref RCC_CRS_TRIMOVF
  * @retval none
  */
__weak void HAL_RCCEx_CRS_ErrorCallback(uint32_t Error)
{

  UNUSED(Error);

  /* NOTE : This function should not be modified, when the callback is needed,
            the @ref HAL_RCCEx_CRS_ErrorCallback should be implemented in the user file
   */
}

/**
  * @}
  */

#endif

/**
  * @}
  */

/** @addtogroup RCCEx_Private_Functions
 * @{
 */

#if defined(RCC_PLLSAI1_SUPPORT)

/**
  * @brief  Configure the parameters N & P & optionally M of PLLSAI1 and enable PLLSAI1 output clock(s).
  * @param  PllSai1  pointer to an RCC_PLLSAI1InitTypeDef structure that
  *         contains the configuration parameters N & P & optionally M as well as PLLSAI1 output clock(s)
  * @param  Divider  divider parameter to be updated
  *
  * @note   PLLSAI1 is temporary disable to apply new parameters
  *
  * @retval HAL status
  */
static HAL_StatusTypeDef RCCEx_PLLSAI1_Config(RCC_PLLSAI1InitTypeDef *PllSai1, uint32_t Divider)
{
  uint32_t tickstart;
  HAL_StatusTypeDef status = HAL_OK;



  assert_param(IS_RCC_PLLSAI1SOURCE(PllSai1->PLLSAI1Source));
  assert_param(IS_RCC_PLLSAI1M_VALUE(PllSai1->PLLSAI1M));
  assert_param(IS_RCC_PLLSAI1N_VALUE(PllSai1->PLLSAI1N));
  assert_param(IS_RCC_PLLSAI1CLOCKOUT_VALUE(PllSai1->PLLSAI1ClockOut));


  if(__HAL_RCC_GET_PLL_OSCSOURCE() != RCC_PLLSOURCE_NONE)
  {

    if((__HAL_RCC_GET_PLL_OSCSOURCE() != PllSai1->PLLSAI1Source)
       ||
       (PllSai1->PLLSAI1Source == RCC_PLLSOURCE_NONE)
#if !defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)
       ||
       (((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U) != PllSai1->PLLSAI1M)
#endif
      )
    {
      status = HAL_ERROR;
    }
  }
  else
  {

    switch(PllSai1->PLLSAI1Source)
    {
    case RCC_PLLSOURCE_MSI:
      if(HAL_IS_BIT_CLR(RCC->CR, RCC_CR_MSIRDY))
      {
        status = HAL_ERROR;
      }
      break;
    case RCC_PLLSOURCE_HSI:
      if(HAL_IS_BIT_CLR(RCC->CR, RCC_CR_HSIRDY))
      {
        status = HAL_ERROR;
      }
      break;
    case RCC_PLLSOURCE_HSE:
      if(HAL_IS_BIT_CLR(RCC->CR, RCC_CR_HSERDY))
      {
        if(HAL_IS_BIT_CLR(RCC->CR, RCC_CR_HSEBYP))
        {
          status = HAL_ERROR;
        }
      }
      break;
    default:
      status = HAL_ERROR;
      break;
    }

    if(status == HAL_OK)
    {
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)

      MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, PllSai1->PLLSAI1Source);
#else

      MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM, PllSai1->PLLSAI1Source | (PllSai1->PLLSAI1M - 1U) << RCC_PLLCFGR_PLLM_Pos);
#endif
    }
  }

  if(status == HAL_OK)
  {

    __HAL_RCC_PLLSAI1_DISABLE();


    tickstart = HAL_GetTick();


    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) != 0U)
    {
      if((HAL_GetTick() - tickstart) > PLLSAI1_TIMEOUT_VALUE)
      {
        status = HAL_TIMEOUT;
        break;
      }
    }

    if(status == HAL_OK)
    {
      if(Divider == DIVIDER_P_UPDATE)
      {
        assert_param(IS_RCC_PLLSAI1P_VALUE(PllSai1->PLLSAI1P));
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)


#if defined(RCC_PLLSAI1P_DIV_2_31_SUPPORT)
        MODIFY_REG(RCC->PLLSAI1CFGR,
                   RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1PDIV | RCC_PLLSAI1CFGR_PLLSAI1M,
                   (PllSai1->PLLSAI1N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                   (PllSai1->PLLSAI1P << RCC_PLLSAI1CFGR_PLLSAI1PDIV_Pos) |
                   ((PllSai1->PLLSAI1M - 1U) << RCC_PLLSAI1CFGR_PLLSAI1M_Pos));
#else
        MODIFY_REG(RCC->PLLSAI1CFGR,
                   RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1P | RCC_PLLSAI1CFGR_PLLSAI1M,
                   (PllSai1->PLLSAI1N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                   ((PllSai1->PLLSAI1P >> 4U) << RCC_PLLSAI1CFGR_PLLSAI1P_Pos) |
                   ((PllSai1->PLLSAI1M - 1U) << RCC_PLLSAI1CFGR_PLLSAI1M_Pos));
#endif

#else

#if defined(RCC_PLLSAI1P_DIV_2_31_SUPPORT)
        MODIFY_REG(RCC->PLLSAI1CFGR,
                   RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1PDIV,
                   (PllSai1->PLLSAI1N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                   (PllSai1->PLLSAI1P << RCC_PLLSAI1CFGR_PLLSAI1PDIV_Pos));
#else
        MODIFY_REG(RCC->PLLSAI1CFGR,
                   RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1P,
                   (PllSai1->PLLSAI1N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                   ((PllSai1->PLLSAI1P >> 4U) << RCC_PLLSAI1CFGR_PLLSAI1P_Pos));
#endif

#endif
      }
      else if(Divider == DIVIDER_Q_UPDATE)
      {
        assert_param(IS_RCC_PLLSAI1Q_VALUE(PllSai1->PLLSAI1Q));
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)

        MODIFY_REG(RCC->PLLSAI1CFGR,
                   RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1Q | RCC_PLLSAI1CFGR_PLLSAI1M,
                   (PllSai1->PLLSAI1N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                   (((PllSai1->PLLSAI1Q >> 1U) - 1U) << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) |
                   ((PllSai1->PLLSAI1M - 1U) << RCC_PLLSAI1CFGR_PLLSAI1M_Pos));
#else

        MODIFY_REG(RCC->PLLSAI1CFGR,
                   RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1Q,
                   (PllSai1->PLLSAI1N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                   (((PllSai1->PLLSAI1Q >> 1U) - 1U) << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos));
#endif
      }
      else
      {
        assert_param(IS_RCC_PLLSAI1R_VALUE(PllSai1->PLLSAI1R));
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)

        MODIFY_REG(RCC->PLLSAI1CFGR,
                   RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1R | RCC_PLLSAI1CFGR_PLLSAI1M,
                   (PllSai1->PLLSAI1N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                   (((PllSai1->PLLSAI1R >> 1U) - 1U) << RCC_PLLSAI1CFGR_PLLSAI1R_Pos) |
                   ((PllSai1->PLLSAI1M - 1U) << RCC_PLLSAI1CFGR_PLLSAI1M_Pos));
#else

        MODIFY_REG(RCC->PLLSAI1CFGR,
                   RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1R,
                   (PllSai1->PLLSAI1N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                   (((PllSai1->PLLSAI1R >> 1U) - 1U) << RCC_PLLSAI1CFGR_PLLSAI1R_Pos));
#endif
      }


      __HAL_RCC_PLLSAI1_ENABLE();


      tickstart = HAL_GetTick();


      while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) == 0U)
      {
        if((HAL_GetTick() - tickstart) > PLLSAI1_TIMEOUT_VALUE)
        {
          status = HAL_TIMEOUT;
          break;
        }
      }

      if(status == HAL_OK)
      {

        __HAL_RCC_PLLSAI1CLKOUT_ENABLE(PllSai1->PLLSAI1ClockOut);
      }
    }
  }

  return status;
}

#endif

#if defined(RCC_PLLSAI2_SUPPORT)

/**
  * @brief  Configure the parameters N & P & optionally M of PLLSAI2 and enable PLLSAI2 output clock(s).
  * @param  PllSai2  pointer to an RCC_PLLSAI2InitTypeDef structure that
  *         contains the configuration parameters N & P & optionally M as well as PLLSAI2 output clock(s)
  * @param  Divider  divider parameter to be updated
  *
  * @note   PLLSAI2 is temporary disable to apply new parameters
  *
  * @retval HAL status
  */
static HAL_StatusTypeDef RCCEx_PLLSAI2_Config(RCC_PLLSAI2InitTypeDef *PllSai2, uint32_t Divider)
{
  uint32_t tickstart;
  HAL_StatusTypeDef status = HAL_OK;



  assert_param(IS_RCC_PLLSAI2SOURCE(PllSai2->PLLSAI2Source));
  assert_param(IS_RCC_PLLSAI2M_VALUE(PllSai2->PLLSAI2M));
  assert_param(IS_RCC_PLLSAI2N_VALUE(PllSai2->PLLSAI2N));
  assert_param(IS_RCC_PLLSAI2CLOCKOUT_VALUE(PllSai2->PLLSAI2ClockOut));


  if(__HAL_RCC_GET_PLL_OSCSOURCE() != RCC_PLLSOURCE_NONE)
  {

    if((__HAL_RCC_GET_PLL_OSCSOURCE() != PllSai2->PLLSAI2Source)
       ||
       (PllSai2->PLLSAI2Source == RCC_PLLSOURCE_NONE)
#if !defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)
       ||
       (((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U) != PllSai2->PLLSAI2M)
#endif
      )
    {
      status = HAL_ERROR;
    }
  }
  else
  {

    switch(PllSai2->PLLSAI2Source)
    {
    case RCC_PLLSOURCE_MSI:
      if(HAL_IS_BIT_CLR(RCC->CR, RCC_CR_MSIRDY))
      {
        status = HAL_ERROR;
      }
      break;
    case RCC_PLLSOURCE_HSI:
      if(HAL_IS_BIT_CLR(RCC->CR, RCC_CR_HSIRDY))
      {
        status = HAL_ERROR;
      }
      break;
    case RCC_PLLSOURCE_HSE:
      if(HAL_IS_BIT_CLR(RCC->CR, RCC_CR_HSERDY))
      {
        if(HAL_IS_BIT_CLR(RCC->CR, RCC_CR_HSEBYP))
        {
          status = HAL_ERROR;
        }
      }
      break;
    default:
      status = HAL_ERROR;
      break;
    }

    if(status == HAL_OK)
    {
#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)

      MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, PllSai2->PLLSAI2Source);
#else

      MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM, PllSai2->PLLSAI2Source | (PllSai2->PLLSAI2M - 1U) << RCC_PLLCFGR_PLLM_Pos);
#endif
    }
  }

  if(status == HAL_OK)
  {

    __HAL_RCC_PLLSAI2_DISABLE();


    tickstart = HAL_GetTick();


    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI2RDY) != 0U)
    {
      if((HAL_GetTick() - tickstart) > PLLSAI2_TIMEOUT_VALUE)
      {
        status = HAL_TIMEOUT;
        break;
      }
    }

    if(status == HAL_OK)
    {
      if(Divider == DIVIDER_P_UPDATE)
      {
        assert_param(IS_RCC_PLLSAI2P_VALUE(PllSai2->PLLSAI2P));
#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)


#if defined(RCC_PLLSAI2P_DIV_2_31_SUPPORT)
        MODIFY_REG(RCC->PLLSAI2CFGR,
                   RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2PDIV | RCC_PLLSAI2CFGR_PLLSAI2M,
                   (PllSai2->PLLSAI2N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos) |
                   (PllSai2->PLLSAI2P << RCC_PLLSAI2CFGR_PLLSAI2PDIV_Pos) |
                   ((PllSai2->PLLSAI2M - 1U) << RCC_PLLSAI2CFGR_PLLSAI2M_Pos));
#else
        MODIFY_REG(RCC->PLLSAI2CFGR,
                   RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2P | RCC_PLLSAI2CFGR_PLLSAI2M,
                   (PllSai2->PLLSAI2N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos) |
                   ((PllSai2->PLLSAI2P >> 4U) << RCC_PLLSAI2CFGR_PLLSAI2P_Pos) |
                   ((PllSai2->PLLSAI2M - 1U) << RCC_PLLSAI2CFGR_PLLSAI2M_Pos));
#endif

#else

#if defined(RCC_PLLSAI2P_DIV_2_31_SUPPORT)
        MODIFY_REG(RCC->PLLSAI2CFGR,
                   RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2PDIV,
                   (PllSai2->PLLSAI2N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos) |
                   (PllSai2->PLLSAI2P << RCC_PLLSAI2CFGR_PLLSAI2PDIV_Pos));
#else
        MODIFY_REG(RCC->PLLSAI2CFGR,
                   RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2P,
                   (PllSai2->PLLSAI2N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos) |
                   ((PllSai2->PLLSAI2P >> 4U) << RCC_PLLSAI2CFGR_PLLSAI2P_Pos));
#endif

#endif
      }
#if defined(RCC_PLLSAI2Q_DIV_SUPPORT)
      else if(Divider == DIVIDER_Q_UPDATE)
      {
        assert_param(IS_RCC_PLLSAI2Q_VALUE(PllSai2->PLLSAI2Q));
#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)

        MODIFY_REG(RCC->PLLSAI2CFGR,
                   RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2Q | RCC_PLLSAI2CFGR_PLLSAI2M,
                   (PllSai2->PLLSAI2N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos) |
                   (((PllSai2->PLLSAI2Q >> 1U) - 1U) << RCC_PLLSAI2CFGR_PLLSAI2Q_Pos) |
                   ((PllSai2->PLLSAI2M - 1U) << RCC_PLLSAI2CFGR_PLLSAI2M_Pos));
#else

        MODIFY_REG(RCC->PLLSAI2CFGR,
                   RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2Q,
                   (PllSai2->PLLSAI2N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos) |
                   (((PllSai2->PLLSAI2Q >> 1U) - 1U) << RCC_PLLSAI2CFGR_PLLSAI2Q_Pos));
#endif
      }
#endif
      else
      {
        assert_param(IS_RCC_PLLSAI2R_VALUE(PllSai2->PLLSAI2R));
#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)

        MODIFY_REG(RCC->PLLSAI2CFGR,
                   RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2R | RCC_PLLSAI2CFGR_PLLSAI2M,
                   (PllSai2->PLLSAI2N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos) |
                   (((PllSai2->PLLSAI2R >> 1U) - 1U) << RCC_PLLSAI2CFGR_PLLSAI2R_Pos) |
                   ((PllSai2->PLLSAI2M - 1U) << RCC_PLLSAI2CFGR_PLLSAI2M_Pos));
#else

        MODIFY_REG(RCC->PLLSAI2CFGR,
                   RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2R,
                   (PllSai2->PLLSAI2N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos) |
                   (((PllSai2->PLLSAI2R >> 1U) - 1U) << RCC_PLLSAI2CFGR_PLLSAI2R_Pos));
#endif
      }


      __HAL_RCC_PLLSAI2_ENABLE();


      tickstart = HAL_GetTick();


      while(READ_BIT(RCC->CR, RCC_CR_PLLSAI2RDY) == 0U)
      {
        if((HAL_GetTick() - tickstart) > PLLSAI2_TIMEOUT_VALUE)
        {
          status = HAL_TIMEOUT;
          break;
        }
      }

      if(status == HAL_OK)
      {

        __HAL_RCC_PLLSAI2CLKOUT_ENABLE(PllSai2->PLLSAI2ClockOut);
      }
    }
  }

  return status;
}

#endif

#if defined(SAI1)

static uint32_t RCCEx_GetSAIxPeriphCLKFreq(uint32_t PeriphClk, uint32_t InputFrequency)
{
  uint32_t frequency = 0U;
  uint32_t srcclk = 0U;
  uint32_t pllvco, plln;
#if defined(RCC_PLLP_SUPPORT)
  uint32_t pllp = 0U;
#endif


  if(PeriphClk == RCC_PERIPHCLK_SAI1)
  {
    srcclk = __HAL_RCC_GET_SAI1_SOURCE();
    if(srcclk == RCC_SAI1CLKSOURCE_PIN)
    {
      frequency = EXTERNAL_SAI1_CLOCK_VALUE;
    }

  }
#if defined(SAI2)
  else
  {
    if(PeriphClk == RCC_PERIPHCLK_SAI2)
    {
      srcclk = __HAL_RCC_GET_SAI2_SOURCE();
      if(srcclk == RCC_SAI2CLKSOURCE_PIN)
      {
        frequency = EXTERNAL_SAI2_CLOCK_VALUE;
      }

    }
  }
#endif

  if(frequency == 0U)
  {
    pllvco = InputFrequency;

#if defined(SAI2)
    if((srcclk == RCC_SAI1CLKSOURCE_PLL) || (srcclk == RCC_SAI2CLKSOURCE_PLL))
    {
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLRDY) && (__HAL_RCC_GET_PLLCLKOUT_CONFIG(RCC_PLL_SAI3CLK) != 0U))
      {

        pllvco = (pllvco / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));

        plln = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
#if defined(RCC_PLLP_DIV_2_31_SUPPORT)
        pllp = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLPDIV) >> RCC_PLLCFGR_PLLPDIV_Pos;
#endif
        if(pllp == 0U)
        {
          if(READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP) != 0U)
          {
            pllp = 17U;
          }
          else
          {
            pllp = 7U;
          }
        }
        frequency = (pllvco * plln) / pllp;
      }
    }
    else if(srcclk == 0U)
    {
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLSAI1RDY) && (__HAL_RCC_GET_PLLSAI1CLKOUT_CONFIG(RCC_PLLSAI1_SAI1CLK) != 0U))
      {
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)


        pllvco = (pllvco / ((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1M) >> RCC_PLLSAI1CFGR_PLLSAI1M_Pos) + 1U));
#else

        pllvco = (pllvco / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));
#endif

        plln = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos;
#if defined(RCC_PLLSAI1P_DIV_2_31_SUPPORT)
        pllp = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PDIV) >> RCC_PLLSAI1CFGR_PLLSAI1PDIV_Pos;
#endif
        if(pllp == 0U)
        {
          if(READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P) != 0U)
          {
            pllp = 17U;
          }
          else
          {
            pllp = 7U;
          }
        }
        frequency = (pllvco * plln) / pllp;
      }
    }
#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
    else if((srcclk == RCC_SAI1CLKSOURCE_HSI) || (srcclk == RCC_SAI2CLKSOURCE_HSI))
    {
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
      {
        frequency = HSI_VALUE;
      }
    }
#endif

#else
    if(srcclk == RCC_SAI1CLKSOURCE_PLL)
    {
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLRDY) && (__HAL_RCC_GET_PLLCLKOUT_CONFIG(RCC_PLL_SAI2CLK) != 0U))
      {

        pllvco = (pllvco / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));

        plln = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
#if defined(RCC_PLLP_DIV_2_31_SUPPORT)
        pllp = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLPDIV) >> RCC_PLLCFGR_PLLPDIV_Pos;
#endif
        if(pllp == 0U)
        {
          if(READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP) != 0U)
          {
            pllp = 17U;
          }
          else
          {
            pllp = 7U;
          }
        }
        frequency = (pllvco * plln) / pllp;
      }
      else if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
      {

        frequency = HSI_VALUE;
      }
      else
      {

      }
    }
    else if(srcclk == RCC_SAI1CLKSOURCE_PLLSAI1)
    {
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLSAI1RDY) && (__HAL_RCC_GET_PLLSAI1CLKOUT_CONFIG(RCC_PLLSAI1_SAI1CLK) != 0U))
      {
#if defined(RCC_PLLSAI1M_DIV_1_16_SUPPORT)


        pllvco = (pllvco / ((READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1M) >> RCC_PLLSAI1CFGR_PLLSAI1M_Pos) + 1U));
#else

        pllvco = (pllvco / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));
#endif

        plln = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos;
#if defined(RCC_PLLSAI1P_DIV_2_31_SUPPORT)
        pllp = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PDIV) >> RCC_PLLSAI1CFGR_PLLSAI1PDIV_Pos;
#endif
        if(pllp == 0U)
        {
          if(READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P) != 0U)
          {
            pllp = 17U;
          }
          else
          {
            pllp = 7U;
          }
        }
        frequency = (pllvco * plln) / pllp;
      }
      else if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
      {

        frequency = HSI_VALUE;
      }
      else
      {

      }
    }
#endif

#if defined(RCC_PLLSAI2_SUPPORT)

    else if((srcclk == RCC_SAI1CLKSOURCE_PLLSAI2) || (srcclk == RCC_SAI2CLKSOURCE_PLLSAI2))
    {
      if(HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLLSAI2RDY) && (__HAL_RCC_GET_PLLSAI2CLKOUT_CONFIG(RCC_PLLSAI2_SAI2CLK) != 0U))
      {
#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)


        pllvco = (pllvco / ((READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2M) >> RCC_PLLSAI2CFGR_PLLSAI2M_Pos) + 1U));
#else

        pllvco = (pllvco / ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U));
#endif

        plln = READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N) >> RCC_PLLSAI2CFGR_PLLSAI2N_Pos;
#if defined(RCC_PLLSAI2P_DIV_2_31_SUPPORT)
        pllp = READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2PDIV) >> RCC_PLLSAI2CFGR_PLLSAI2PDIV_Pos;
#endif
        if(pllp == 0U)
        {
          if(READ_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2P) != 0U)
          {
            pllp = 17U;
          }
          else
          {
            pllp = 7U;
          }
        }
        frequency = (pllvco * plln) / pllp;
      }
    }

#endif

    else
    {

    }
  }


  return frequency;
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


