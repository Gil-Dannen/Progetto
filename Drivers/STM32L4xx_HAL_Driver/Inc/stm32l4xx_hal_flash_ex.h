/**
  ******************************************************************************
  * @file    stm32l4xx_hal_flash_ex.h
  * @author  MCD Application Team
  * @brief   Header file of FLASH HAL Extended module.
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


#ifndef STM32L4xx_HAL_FLASH_EX_H
#define STM32L4xx_HAL_FLASH_EX_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32l4xx_hal_def.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @addtogroup FLASHEx
  * @{
  */




#if defined (FLASH_CFGR_LVEN)
/** @addtogroup FLASHEx_Exported_Constants
  * @{
  */
/** @defgroup FLASHEx_LVE_PIN_CFG FLASHEx LVE pin configuration
  * @{
  */
#define FLASH_LVE_PIN_CTRL     0x00000000U
#define FLASH_LVE_PIN_FORCED   FLASH_CFGR_LVEN
/**
  * @}
  */

/**
  * @}
  */
#endif




/** @addtogroup FLASHEx_Exported_Functions
  * @{
  */


/** @addtogroup FLASHEx_Exported_Functions_Group1
  * @{
  */
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);
/**
  * @}
  */

#if defined (FLASH_CFGR_LVEN)
/** @addtogroup FLASHEx_Exported_Functions_Group2
  * @{
  */
HAL_StatusTypeDef HAL_FLASHEx_ConfigLVEPin(uint32_t ConfigLVE);
/**
  * @}
  */
#endif

/**
  * @}
  */


/** @addtogroup FLASHEx_Private_Functions FLASHEx Private Functions
 * @{
 */
void FLASH_PageErase(uint32_t Page, uint32_t Banks);
void FLASH_FlushCaches(void);
/**
  * @}
  */


/**
  @cond 0
  */
#if defined (FLASH_CFGR_LVEN)
#define IS_FLASH_LVE_PIN(CFG)  (((CFG) == FLASH_LVE_PIN_CTRL) || ((CFG) == FLASH_LVE_PIN_FORCED))
#endif
/**
  @endcond
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

