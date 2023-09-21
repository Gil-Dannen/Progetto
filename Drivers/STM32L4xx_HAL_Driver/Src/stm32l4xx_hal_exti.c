/**
  ******************************************************************************
  * @file    stm32l4xx_hal_exti.c
  * @author  MCD Application Team
  * @brief   EXTI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Extended Interrupts and events controller (EXTI) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                    ##### EXTI Peripheral features #####
  ==============================================================================
  [..]
    (+) Each Exti line can be configured within this driver.

    (+) Exti line can be configured in 3 different modes
        (++) Interrupt
        (++) Event
        (++) Both of them

    (+) Configurable Exti lines can be configured with 3 different triggers
        (++) Rising
        (++) Falling
        (++) Both of them

    (+) When set in interrupt mode, configurable Exti lines have two different
        interrupts pending registers which allow to distinguish which transition
        occurs:
        (++) Rising edge pending interrupt
        (++) Falling

    (+) Exti lines 0 to 15 are linked to gpio pin number 0 to 15. Gpio port can
        be selected through multiplexer.

                     ##### How to use this driver #####
  ==============================================================================
  [..]

    (#) Configure the EXTI line using HAL_EXTI_SetConfigLine().
        (++) Choose the interrupt line number by setting "Line" member from
             EXTI_ConfigTypeDef structure.
        (++) Configure the interrupt and/or event mode using "Mode" member from
             EXTI_ConfigTypeDef structure.
        (++) For configurable lines, configure rising and/or falling trigger
             "Trigger" member from EXTI_ConfigTypeDef structure.
        (++) For Exti lines linked to gpio, choose gpio port using "GPIOSel"
             member from GPIO_InitTypeDef structure.

    (#) Get current Exti configuration of a dedicated line using
        HAL_EXTI_GetConfigLine().
        (++) Provide exiting handle as parameter.
        (++) Provide pointer on EXTI_ConfigTypeDef structure as second parameter.

    (#) Clear Exti configuration of a dedicated line using HAL_EXTI_GetConfigLine().
        (++) Provide exiting handle as parameter.

    (#) Register callback to treat Exti interrupts using HAL_EXTI_RegisterCallback().
        (++) Provide exiting handle as first parameter.
        (++) Provide which callback will be registered using one value from
             EXTI_CallbackIDTypeDef.
        (++) Provide callback function pointer.

    (#) Get interrupt pending bit using HAL_EXTI_GetPending().

    (#) Clear interrupt pending bit using HAL_EXTI_GetPending().

    (#) Generate software interrupt using HAL_EXTI_GenerateSWI().

  @endverbatim
  */


#include "stm32l4xx_hal.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @addtogroup EXTI
  * @{
  */
/** MISRA C:2012 deviation rule has been granted for following rule:
  * Rule-18.1_b - Medium: Array `EXTICR' 1st subscript interval [0,7] may be out
  * of bounds [0,3] in following API :
  * HAL_EXTI_SetConfigLine
  * HAL_EXTI_GetConfigLine
  * HAL_EXTI_ClearConfigLine
  */

#ifdef HAL_EXTI_MODULE_ENABLED



/** @defgroup EXTI_Private_Constants EXTI Private Constants
  * @{
  */
#define EXTI_MODE_OFFSET                    0x08u
#define EXTI_CONFIG_OFFSET                  0x08u
/**
  * @}
  */






/** @addtogroup EXTI_Exported_Functions
  * @{
  */

/** @addtogroup EXTI_Exported_Functions_Group1
 *  @brief    Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Set configuration of a dedicated Exti line.
  * @param  hexti Exti handle.
  * @param  pExtiConfig Pointer on EXTI configuration to be set.
  * @retval HAL Status.
  */
HAL_StatusTypeDef HAL_EXTI_SetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig)
{
  __IO uint32_t *regaddr;
  uint32_t regval;
  uint32_t linepos;
  uint32_t maskline;
  uint32_t offset;


  if ((hexti == NULL) || (pExtiConfig == NULL))
  {
    return HAL_ERROR;
  }


  assert_param(IS_EXTI_LINE(pExtiConfig->Line));
  assert_param(IS_EXTI_MODE(pExtiConfig->Mode));


  hexti->Line = pExtiConfig->Line;


  offset = ((pExtiConfig->Line & EXTI_REG_MASK) >> EXTI_REG_SHIFT);
  linepos = (pExtiConfig->Line & EXTI_PIN_MASK);
  maskline = (1uL << linepos);


  if ((pExtiConfig->Line & EXTI_CONFIG) != 0x00u)
  {
    assert_param(IS_EXTI_TRIGGER(pExtiConfig->Trigger));


    regaddr = (&EXTI->RTSR1 + (EXTI_CONFIG_OFFSET * offset));
    regval = *regaddr;


    if ((pExtiConfig->Trigger & EXTI_TRIGGER_RISING) != 0x00u)
    {
      regval |= maskline;
    }
    else
    {
      regval &= ~maskline;
    }


    *regaddr = regval;


    regaddr = (&EXTI->FTSR1 + (EXTI_CONFIG_OFFSET * offset));
    regval = *regaddr;


    if ((pExtiConfig->Trigger & EXTI_TRIGGER_FALLING) != 0x00u)
    {
      regval |= maskline;
    }
    else
    {
      regval &= ~maskline;
    }


    *regaddr = regval;


    if ((pExtiConfig->Line & EXTI_GPIO) == EXTI_GPIO)
    {
      assert_param(IS_EXTI_GPIO_PORT(pExtiConfig->GPIOSel));
      assert_param(IS_EXTI_GPIO_PIN(linepos));

      regval = SYSCFG->EXTICR[linepos >> 2u];
      regval &= ~(SYSCFG_EXTICR1_EXTI0 << (SYSCFG_EXTICR1_EXTI1_Pos * (linepos & 0x03u)));
      regval |= (pExtiConfig->GPIOSel << (SYSCFG_EXTICR1_EXTI1_Pos * (linepos & 0x03u)));
      SYSCFG->EXTICR[linepos >> 2u] = regval;
    }
  }


  regaddr = (&EXTI->IMR1 + (EXTI_MODE_OFFSET * offset));
  regval = *regaddr;


  if ((pExtiConfig->Mode & EXTI_MODE_INTERRUPT) != 0x00u)
  {
    regval |= maskline;
  }
  else
  {
    regval &= ~maskline;
  }


  *regaddr = regval;


  assert_param(((pExtiConfig->Line & EXTI_EVENT) == EXTI_EVENT) || ((pExtiConfig->Mode & EXTI_MODE_EVENT) != EXTI_MODE_EVENT));


  regaddr = (&EXTI->EMR1 + (EXTI_MODE_OFFSET * offset));
  regval = *regaddr;


  if ((pExtiConfig->Mode & EXTI_MODE_EVENT) != 0x00u)
  {
    regval |= maskline;
  }
  else
  {
    regval &= ~maskline;
  }


  *regaddr = regval;

  return HAL_OK;
}


/**
  * @brief  Get configuration of a dedicated Exti line.
  * @param  hexti Exti handle.
  * @param  pExtiConfig Pointer on structure to store Exti configuration.
  * @retval HAL Status.
  */
HAL_StatusTypeDef HAL_EXTI_GetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig)
{
  __IO uint32_t *regaddr;
  uint32_t regval;
  uint32_t linepos;
  uint32_t maskline;
  uint32_t offset;


  if ((hexti == NULL) || (pExtiConfig == NULL))
  {
    return HAL_ERROR;
  }


  assert_param(IS_EXTI_LINE(hexti->Line));


  pExtiConfig->Line = hexti->Line;


  offset = ((pExtiConfig->Line & EXTI_REG_MASK) >> EXTI_REG_SHIFT);
  linepos = (pExtiConfig->Line & EXTI_PIN_MASK);
  maskline = (1uL << linepos);


  regaddr = (&EXTI->IMR1 + (EXTI_MODE_OFFSET * offset));
  regval = *regaddr;


  if ((regval & maskline) != 0x00u)
  {
    pExtiConfig->Mode = EXTI_MODE_INTERRUPT;
  }
  else
  {
    pExtiConfig->Mode = EXTI_MODE_NONE;
  }


  regaddr = (&EXTI->EMR1 + (EXTI_MODE_OFFSET * offset));
  regval = *regaddr;


  if ((regval & maskline) != 0x00u)
  {
    pExtiConfig->Mode |= EXTI_MODE_EVENT;
  }


  pExtiConfig->Trigger = EXTI_TRIGGER_NONE;
  pExtiConfig->GPIOSel = 0x00u;


  if ((pExtiConfig->Line & EXTI_CONFIG) != 0x00u)
  {
    regaddr = (&EXTI->RTSR1 + (EXTI_CONFIG_OFFSET * offset));
    regval = *regaddr;


    if ((regval & maskline) != 0x00u)
    {
      pExtiConfig->Trigger = EXTI_TRIGGER_RISING;
    }


    regaddr = (&EXTI->FTSR1 + (EXTI_CONFIG_OFFSET * offset));
    regval = *regaddr;


    if ((regval & maskline) != 0x00u)
    {
      pExtiConfig->Trigger |= EXTI_TRIGGER_FALLING;
    }


    if ((pExtiConfig->Line & EXTI_GPIO) == EXTI_GPIO)
    {
      assert_param(IS_EXTI_GPIO_PIN(linepos));

      regval = SYSCFG->EXTICR[linepos >> 2u];
      pExtiConfig->GPIOSel = ((regval << (SYSCFG_EXTICR1_EXTI1_Pos * (3uL - (linepos & 0x03u)))) >> 24);
    }
  }

  return HAL_OK;
}


/**
  * @brief  Clear whole configuration of a dedicated Exti line.
  * @param  hexti Exti handle.
  * @retval HAL Status.
  */
HAL_StatusTypeDef HAL_EXTI_ClearConfigLine(EXTI_HandleTypeDef *hexti)
{
  __IO uint32_t *regaddr;
  uint32_t regval;
  uint32_t linepos;
  uint32_t maskline;
  uint32_t offset;


  if (hexti == NULL)
  {
    return HAL_ERROR;
  }


  assert_param(IS_EXTI_LINE(hexti->Line));


  offset = ((hexti->Line & EXTI_REG_MASK) >> EXTI_REG_SHIFT);
  linepos = (hexti->Line & EXTI_PIN_MASK);
  maskline = (1uL << linepos);


  regaddr = (&EXTI->IMR1 + (EXTI_MODE_OFFSET * offset));
  regval = (*regaddr & ~maskline);
  *regaddr = regval;


  regaddr = (&EXTI->EMR1 + (EXTI_MODE_OFFSET * offset));
  regval = (*regaddr & ~maskline);
  *regaddr = regval;


  if ((hexti->Line & EXTI_CONFIG) != 0x00u)
  {
    regaddr = (&EXTI->RTSR1 + (EXTI_CONFIG_OFFSET * offset));
    regval = (*regaddr & ~maskline);
    *regaddr = regval;

    regaddr = (&EXTI->FTSR1 + (EXTI_CONFIG_OFFSET * offset));
    regval = (*regaddr & ~maskline);
    *regaddr = regval;


    if ((hexti->Line & EXTI_GPIO) == EXTI_GPIO)
    {
      assert_param(IS_EXTI_GPIO_PIN(linepos));

      regval = SYSCFG->EXTICR[linepos >> 2u];
      regval &= ~(SYSCFG_EXTICR1_EXTI0 << (SYSCFG_EXTICR1_EXTI1_Pos * (linepos & 0x03u)));
      SYSCFG->EXTICR[linepos >> 2u] = regval;
    }
  }

  return HAL_OK;
}


/**
  * @brief  Register callback for a dedicated Exti line.
  * @param  hexti Exti handle.
  * @param  CallbackID User callback identifier.
  *         This parameter can be one of @arg @ref EXTI_CallbackIDTypeDef values.
  * @param  pPendingCbfn function pointer to be stored as callback.
  * @retval HAL Status.
  */
HAL_StatusTypeDef HAL_EXTI_RegisterCallback(EXTI_HandleTypeDef *hexti, EXTI_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void))
{
  HAL_StatusTypeDef status = HAL_OK;

  switch (CallbackID)
  {
    case  HAL_EXTI_COMMON_CB_ID:
      hexti->PendingCallback = pPendingCbfn;
      break;

    default:
      status = HAL_ERROR;
      break;
  }

  return status;
}


/**
  * @brief  Store line number as handle private field.
  * @param  hexti Exti handle.
  * @param  ExtiLine Exti line number.
  *         This parameter can be from 0 to @ref EXTI_LINE_NB.
  * @retval HAL Status.
  */
HAL_StatusTypeDef HAL_EXTI_GetHandle(EXTI_HandleTypeDef *hexti, uint32_t ExtiLine)
{

  assert_param(IS_EXTI_LINE(ExtiLine));


  if (hexti == NULL)
  {
    return HAL_ERROR;
  }
  else
  {

    hexti->Line = ExtiLine;

    return HAL_OK;
  }
}


/**
  * @}
  */

/** @addtogroup EXTI_Exported_Functions_Group2
 *  @brief EXTI IO functions.
 *
@verbatim
 ===============================================================================
                       ##### IO operation functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Handle EXTI interrupt request.
  * @param  hexti Exti handle.
  * @retval none.
  */
void HAL_EXTI_IRQHandler(EXTI_HandleTypeDef *hexti)
{
  __IO uint32_t *regaddr;
  uint32_t regval;
  uint32_t maskline;
  uint32_t offset;


  offset = ((hexti->Line & EXTI_REG_MASK) >> EXTI_REG_SHIFT);
  maskline = (1uL << (hexti->Line & EXTI_PIN_MASK));


  regaddr = (&EXTI->PR1 + (EXTI_CONFIG_OFFSET * offset));
  regval = (*regaddr & maskline);

  if (regval != 0x00u)
  {

    *regaddr = maskline;


    if (hexti->PendingCallback != NULL)
    {
      hexti->PendingCallback();
    }
  }
}


/**
  * @brief  Get interrupt pending bit of a dedicated line.
  * @param  hexti Exti handle.
  * @param  Edge Specify which pending edge as to be checked.
  *         This parameter can be one of the following values:
  *           @arg @ref EXTI_TRIGGER_RISING_FALLING
  *         This parameter is kept for compatibility with other series.
  * @retval 1 if interrupt is pending else 0.
  */
uint32_t HAL_EXTI_GetPending(EXTI_HandleTypeDef *hexti, uint32_t Edge)
{
  __IO uint32_t *regaddr;
  uint32_t regval;
  uint32_t linepos;
  uint32_t maskline;
  uint32_t offset;


  assert_param(IS_EXTI_LINE(hexti->Line));
  assert_param(IS_EXTI_CONFIG_LINE(hexti->Line));
  assert_param(IS_EXTI_PENDING_EDGE(Edge));


  offset = ((hexti->Line & EXTI_REG_MASK) >> EXTI_REG_SHIFT);
  linepos = (hexti->Line & EXTI_PIN_MASK);
  maskline = (1uL << linepos);


  regaddr = (&EXTI->PR1 + (EXTI_CONFIG_OFFSET * offset));


  regval = ((*regaddr & maskline) >> linepos);
  return regval;
}


/**
  * @brief  Clear interrupt pending bit of a dedicated line.
  * @param  hexti Exti handle.
  * @param  Edge Specify which pending edge as to be clear.
  *         This parameter can be one of the following values:
  *           @arg @ref EXTI_TRIGGER_RISING_FALLING
  *         This parameter is kept for compatibility with other series.
  * @retval None.
  */
void HAL_EXTI_ClearPending(EXTI_HandleTypeDef *hexti, uint32_t Edge)
{
  __IO uint32_t *regaddr;
  uint32_t maskline;
  uint32_t offset;


  assert_param(IS_EXTI_LINE(hexti->Line));
  assert_param(IS_EXTI_CONFIG_LINE(hexti->Line));
  assert_param(IS_EXTI_PENDING_EDGE(Edge));


  offset = ((hexti->Line & EXTI_REG_MASK) >> EXTI_REG_SHIFT);
  maskline = (1uL << (hexti->Line & EXTI_PIN_MASK));


  regaddr = (&EXTI->PR1 + (EXTI_CONFIG_OFFSET * offset));


  *regaddr =  maskline;
}


/**
  * @brief  Generate a software interrupt for a dedicated line.
  * @param  hexti Exti handle.
  * @retval None.
  */
void HAL_EXTI_GenerateSWI(EXTI_HandleTypeDef *hexti)
{
  __IO uint32_t *regaddr;
  uint32_t maskline;
  uint32_t offset;


  assert_param(IS_EXTI_LINE(hexti->Line));
  assert_param(IS_EXTI_CONFIG_LINE(hexti->Line));


  offset = ((hexti->Line & EXTI_REG_MASK) >> EXTI_REG_SHIFT);
  maskline = (1uL << (hexti->Line & EXTI_PIN_MASK));

  regaddr = (&EXTI->SWIER1 + (EXTI_CONFIG_OFFSET * offset));
  *regaddr = maskline;
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

