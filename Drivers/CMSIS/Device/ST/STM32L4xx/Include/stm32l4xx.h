/**
  ******************************************************************************
  * @file    stm32l4xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32L4xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32L4xx device used in the target application
  *              - To use or not the peripheral's drivers in application code(i.e.
  *                code will be based on direct access to peripheral's registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
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
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32l4xx
  * @{
  */

#ifndef __STM32L4xx_H
#define __STM32L4xx_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined (STM32L4)
#define STM32L4
#endif

/* Uncomment the line below according to the target STM32L4 device used in your
   application
  */

#if !defined (STM32L412xx) && !defined (STM32L422xx) && \
    !defined (STM32L431xx) && !defined (STM32L432xx) && !defined (STM32L433xx) && !defined (STM32L442xx) && !defined (STM32L443xx) && \
    !defined (STM32L451xx) && !defined (STM32L452xx) && !defined (STM32L462xx) && \
    !defined (STM32L471xx) && !defined (STM32L475xx) && !defined (STM32L476xx) && !defined (STM32L485xx) && !defined (STM32L486xx) && \
    !defined (STM32L496xx) && !defined (STM32L4A6xx) && \
    !defined (STM32L4P5xx) && !defined (STM32L4Q5xx) && \
    !defined (STM32L4R5xx) && !defined (STM32L4R7xx) && !defined (STM32L4R9xx) && !defined (STM32L4S5xx) && !defined (STM32L4S7xx) && !defined (STM32L4S9xx)
























#endif

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
#if !defined  (USE_HAL_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */

#endif

/**
  * @brief CMSIS Device version number
  */
#define __STM32L4_CMSIS_VERSION_MAIN   (0x01)
#define __STM32L4_CMSIS_VERSION_SUB1   (0x07)
#define __STM32L4_CMSIS_VERSION_SUB2   (0x02)
#define __STM32L4_CMSIS_VERSION_RC     (0x00)
#define __STM32L4_CMSIS_VERSION        ((__STM32L4_CMSIS_VERSION_MAIN << 24)\
                                       |(__STM32L4_CMSIS_VERSION_SUB1 << 16)\
                                       |(__STM32L4_CMSIS_VERSION_SUB2 << 8 )\
                                       |(__STM32L4_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(STM32L412xx)
  #include "stm32l412xx.h"
#elif defined(STM32L422xx)
  #include "stm32l422xx.h"
#elif defined(STM32L431xx)
  #include "stm32l431xx.h"
#elif defined(STM32L432xx)
  #include "stm32l432xx.h"
#elif defined(STM32L433xx)
  #include "stm32l433xx.h"
#elif defined(STM32L442xx)
  #include "stm32l442xx.h"
#elif defined(STM32L443xx)
  #include "stm32l443xx.h"
#elif defined(STM32L451xx)
  #include "stm32l451xx.h"
#elif defined(STM32L452xx)
  #include "stm32l452xx.h"
#elif defined(STM32L462xx)
  #include "stm32l462xx.h"
#elif defined(STM32L471xx)
  #include "stm32l471xx.h"
#elif defined(STM32L475xx)
  #include "stm32l475xx.h"
#elif defined(STM32L476xx)
  #include "stm32l476xx.h"
#elif defined(STM32L485xx)
  #include "stm32l485xx.h"
#elif defined(STM32L486xx)
  #include "stm32l486xx.h"
#elif defined(STM32L496xx)
  #include "stm32l496xx.h"
#elif defined(STM32L4A6xx)
  #include "stm32l4a6xx.h"
#elif defined(STM32L4P5xx)
  #include "stm32l4p5xx.h"
#elif defined(STM32L4Q5xx)
  #include "stm32l4q5xx.h"
#elif defined(STM32L4R5xx)
  #include "stm32l4r5xx.h"
#elif defined(STM32L4R7xx)
  #include "stm32l4r7xx.h"
#elif defined(STM32L4R9xx)
  #include "stm32l4r9xx.h"
#elif defined(STM32L4S5xx)
  #include "stm32l4s5xx.h"
#elif defined(STM32L4S7xx)
  #include "stm32l4s7xx.h"
#elif defined(STM32L4S9xx)
  #include "stm32l4s9xx.h"
#else
 #error "Please select first the target STM32L4xx device used in your application (in stm32l4xx.h file)"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))



#define ATOMIC_SET_BIT(REG, BIT)                             \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) | (BIT);       \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)


#define ATOMIC_CLEAR_BIT(REG, BIT)                           \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)


#define ATOMIC_MODIFY_REG(REG, CLEARMSK, SETMASK)                          \
  do {                                                                     \
    uint32_t val;                                                          \
    do {                                                                   \
      val = (__LDREXW((__IO uint32_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U);               \
  } while(0)


#define ATOMIC_SETH_BIT(REG, BIT)                            \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) | (BIT);       \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)


#define ATOMIC_CLEARH_BIT(REG, BIT)                          \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)


#define ATOMIC_MODIFYH_REG(REG, CLEARMSK, SETMASK)                         \
  do {                                                                     \
    uint16_t val;                                                          \
    do {                                                                   \
      val = (__LDREXH((__IO uint16_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U);               \
  } while(0)

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "stm32l4xx_hal.h"
#endif

#ifdef __cplusplus
}
#endif

#endif
/**
  * @}
  */

/**
  * @}
  */

