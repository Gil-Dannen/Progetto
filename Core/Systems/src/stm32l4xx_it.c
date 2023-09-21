
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */



#include "main.h"
#include "stm32l4xx_it.h"



































extern TIM_HandleTypeDef htim1;








/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{




  while (1)
  {
  }

}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{



  while (1)
  {


  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{



  while (1)
  {


  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{



  while (1)
  {


  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{



  while (1)
  {


  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{






}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{






}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{






}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{







}








/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{



  HAL_TIM_IRQHandler(&htim1);



}





