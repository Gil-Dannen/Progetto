/**
  ******************************************************************************
  * @file    stm32l4xx_hal_conf.h
  * @author  MCD Application Team
  * @brief   HAL configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to stm32l4xx_hal_conf.h.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


#ifndef STM32L4xx_HAL_CONF_H
#define STM32L4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif





/**
  * @brief This is the list of modules to be used in the HAL driver
  */
#define HAL_MODULE_ENABLED








































#define HAL_TIM_MODULE_ENABLED

#define HAL_UART_MODULE_ENABLED




#define HAL_GPIO_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED


/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000U)
#endif

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)
#endif

/**
  * @brief Internal Multiple Speed oscillator (MSI) default value.
  *        This value is the default MSI range value after Reset.
  */
#if !defined  (MSI_VALUE)
  #define MSI_VALUE    ((uint32_t)4000000U)
#endif
/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U)
#endif

/**
  * @brief Internal High Speed oscillator (HSI48) value for USB FS, SDMMC and RNG.
  *        This internal oscillator is mainly dedicated to provide a high precision clock to
  *        the USB peripheral by means of a special Clock Recovery System (CRS) circuitry.
  *        When the CRS is not used, the HSI48 RC oscillator runs on it default frequency
  *        which is subject to manufacturing process variations.
  */
#if !defined  (HSI48_VALUE)
 #define HSI48_VALUE   ((uint32_t)48000000U) /*!< Value of the Internal High Speed oscillator for USB FS/SDMMC/RNG in Hz.
                                              The real value my vary depending on manufacturing process variations.*/
#endif

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE)
 #define LSI_VALUE  32000U
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.*/

/**
  * @brief External Low Speed oscillator (LSE) value.
  *        This value is used by the UART, RTC HAL module to compute the system frequency
  */
#if !defined  (LSE_VALUE)
  #define LSE_VALUE    32768U
#endif

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    5000U
#endif

/**
  * @brief External clock source for SAI1 peripheral
  *        This value is used by the RCC HAL module to compute the SAI1 & SAI2 clock source
  *        frequency.
  */
#if !defined  (EXTERNAL_SAI1_CLOCK_VALUE)
  #define EXTERNAL_SAI1_CLOCK_VALUE    2097000U
#endif

/**
  * @brief External clock source for SAI2 peripheral
  *        This value is used by the RCC HAL module to compute the SAI1 & SAI2 clock source
  *        frequency.
  */
#if !defined  (EXTERNAL_SAI2_CLOCK_VALUE)
  #define EXTERNAL_SAI2_CLOCK_VALUE    2097000U
#endif

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */


/**
  * @brief This is the HAL system configuration section
  */

#define  VDD_VALUE					  3300U
#define  TICK_INT_PRIORITY            15U
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              0U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U


/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */



/**
  * @brief Set below the peripheral configuration  to "1U" to add the support
  *        of HAL callback registration/deregistration feature for the HAL
  *        driver(s). This allows user application to provide specific callback
  *        functions thanks to HAL_PPP_RegisterCallback() rather than overwriting
  *        the default weak callback functions (see each stm32l4xx_hal_ppp.h file
  *        for possible callback identifiers defined in HAL_PPP_CallbackIDTypeDef
  *        for each PPP peripheral).
  */
#define USE_HAL_ADC_REGISTER_CALLBACKS        0U
#define USE_HAL_CAN_REGISTER_CALLBACKS        0U
#define USE_HAL_COMP_REGISTER_CALLBACKS       0U
#define USE_HAL_CRYP_REGISTER_CALLBACKS       0U
#define USE_HAL_DAC_REGISTER_CALLBACKS        0U
#define USE_HAL_DCMI_REGISTER_CALLBACKS       0U
#define USE_HAL_DFSDM_REGISTER_CALLBACKS      0U
#define USE_HAL_DMA2D_REGISTER_CALLBACKS      0U
#define USE_HAL_DSI_REGISTER_CALLBACKS        0U
#define USE_HAL_GFXMMU_REGISTER_CALLBACKS     0U
#define USE_HAL_HASH_REGISTER_CALLBACKS       0U
#define USE_HAL_HCD_REGISTER_CALLBACKS        0U
#define USE_HAL_I2C_REGISTER_CALLBACKS        0U
#define USE_HAL_IRDA_REGISTER_CALLBACKS       0U
#define USE_HAL_LPTIM_REGISTER_CALLBACKS      0U
#define USE_HAL_LTDC_REGISTER_CALLBACKS       0U
#define USE_HAL_MMC_REGISTER_CALLBACKS        0U
#define USE_HAL_OPAMP_REGISTER_CALLBACKS      0U
#define USE_HAL_OSPI_REGISTER_CALLBACKS       0U
#define USE_HAL_PCD_REGISTER_CALLBACKS        0U
#define USE_HAL_QSPI_REGISTER_CALLBACKS       0U
#define USE_HAL_RNG_REGISTER_CALLBACKS        0U
#define USE_HAL_RTC_REGISTER_CALLBACKS        0U
#define USE_HAL_SAI_REGISTER_CALLBACKS        0U
#define USE_HAL_SD_REGISTER_CALLBACKS         0U
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS  0U
#define USE_HAL_SMBUS_REGISTER_CALLBACKS      0U
#define USE_HAL_SPI_REGISTER_CALLBACKS        0U
#define USE_HAL_SWPMI_REGISTER_CALLBACKS      0U
#define USE_HAL_TIM_REGISTER_CALLBACKS        0U
#define USE_HAL_TSC_REGISTER_CALLBACKS        0U
#define USE_HAL_UART_REGISTER_CALLBACKS       0U
#define USE_HAL_USART_REGISTER_CALLBACKS      0U
#define USE_HAL_WWDG_REGISTER_CALLBACKS       0U



/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
 * Activated: CRC code is present inside driver
 * Deactivated: CRC code cleaned from driver
 */

#define USE_SPI_CRC                   0U


/**
  * @brief Include module's header file
  */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32l4xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32l4xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32l4xx_hal_dma.h"
#endif

#ifdef HAL_DFSDM_MODULE_ENABLED
  #include "stm32l4xx_hal_dfsdm.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32l4xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32l4xx_hal_adc.h"
#endif

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32l4xx_hal_can.h"
#endif

#ifdef HAL_CAN_LEGACY_MODULE_ENABLED
  #include "Legacy/stm32l4xx_hal_can_legacy.h"
#endif

#ifdef HAL_COMP_MODULE_ENABLED
  #include "stm32l4xx_hal_comp.h"
#endif

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32l4xx_hal_crc.h"
#endif

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32l4xx_hal_cryp.h"
#endif

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32l4xx_hal_dac.h"
#endif

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32l4xx_hal_dcmi.h"
#endif

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32l4xx_hal_dma2d.h"
#endif

#ifdef HAL_DSI_MODULE_ENABLED
  #include "stm32l4xx_hal_dsi.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32l4xx_hal_exti.h"
#endif

#ifdef HAL_GFXMMU_MODULE_ENABLED
  #include "stm32l4xx_hal_gfxmmu.h"
#endif

#ifdef HAL_FIREWALL_MODULE_ENABLED
  #include "stm32l4xx_hal_firewall.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32l4xx_hal_flash.h"
#endif

#ifdef HAL_HASH_MODULE_ENABLED
  #include "stm32l4xx_hal_hash.h"
#endif

#ifdef HAL_HCD_MODULE_ENABLED
  #include "stm32l4xx_hal_hcd.h"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
  #include "stm32l4xx_hal_i2c.h"
#endif

#ifdef HAL_IRDA_MODULE_ENABLED
  #include "stm32l4xx_hal_irda.h"
#endif

#ifdef HAL_IWDG_MODULE_ENABLED
  #include "stm32l4xx_hal_iwdg.h"
#endif

#ifdef HAL_LCD_MODULE_ENABLED
  #include "stm32l4xx_hal_lcd.h"
#endif

#ifdef HAL_LPTIM_MODULE_ENABLED
  #include "stm32l4xx_hal_lptim.h"
#endif

#ifdef HAL_LTDC_MODULE_ENABLED
  #include "stm32l4xx_hal_ltdc.h"
#endif

#ifdef HAL_MMC_MODULE_ENABLED
  #include "stm32l4xx_hal_mmc.h"
#endif

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32l4xx_hal_nand.h"
#endif

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32l4xx_hal_nor.h"
#endif

#ifdef HAL_OPAMP_MODULE_ENABLED
  #include "stm32l4xx_hal_opamp.h"
#endif

#ifdef HAL_OSPI_MODULE_ENABLED
  #include "stm32l4xx_hal_ospi.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
  #include "stm32l4xx_hal_pcd.h"
#endif

#ifdef HAL_PKA_MODULE_ENABLED
  #include "stm32l4xx_hal_pka.h"
#endif

#ifdef HAL_PSSI_MODULE_ENABLED
  #include "stm32l4xx_hal_pssi.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32l4xx_hal_pwr.h"
#endif

#ifdef HAL_QSPI_MODULE_ENABLED
  #include "stm32l4xx_hal_qspi.h"
#endif

#ifdef HAL_RNG_MODULE_ENABLED
  #include "stm32l4xx_hal_rng.h"
#endif

#ifdef HAL_RTC_MODULE_ENABLED
  #include "stm32l4xx_hal_rtc.h"
#endif

#ifdef HAL_SAI_MODULE_ENABLED
  #include "stm32l4xx_hal_sai.h"
#endif

#ifdef HAL_SD_MODULE_ENABLED
  #include "stm32l4xx_hal_sd.h"
#endif

#ifdef HAL_SMARTCARD_MODULE_ENABLED
  #include "stm32l4xx_hal_smartcard.h"
#endif

#ifdef HAL_SMBUS_MODULE_ENABLED
  #include "stm32l4xx_hal_smbus.h"
#endif

#ifdef HAL_SPI_MODULE_ENABLED
  #include "stm32l4xx_hal_spi.h"
#endif

#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32l4xx_hal_sram.h"
#endif

#ifdef HAL_SWPMI_MODULE_ENABLED
  #include "stm32l4xx_hal_swpmi.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32l4xx_hal_tim.h"
#endif

#ifdef HAL_TSC_MODULE_ENABLED
  #include "stm32l4xx_hal_tsc.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32l4xx_hal_uart.h"
#endif

#ifdef HAL_USART_MODULE_ENABLED
  #include "stm32l4xx_hal_usart.h"
#endif

#ifdef HAL_WWDG_MODULE_ENABLED
  #include "stm32l4xx_hal_wwdg.h"
#endif


#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))

  void assert_failed(uint8_t *file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif


