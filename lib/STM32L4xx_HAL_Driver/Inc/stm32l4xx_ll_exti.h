/**
  ******************************************************************************
  * @file    stm32l4xx_ll_exti.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    16-September-2015
  * @brief   Header file of EXTI LL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4xx_LL_EXTI_H
#define __STM32L4xx_LL_EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"

/** @addtogroup STM32L4xx_LL_Driver
  * @{
  */

#if defined (EXTI)

/** @defgroup EXTI_LL EXTI
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup EXTI_LL_Exported_Constants EXTI Exported Constants
  * @{
  */

/** @defgroup EXTI_LL_EC_LINE LINE
  * @{
  */
#define LL_EXTI_LINE_0        EXTI_IMR1_IM0           /*!< Extended line 0 */
#define LL_EXTI_LINE_1        EXTI_IMR1_IM1           /*!< Extended line 1 */
#define LL_EXTI_LINE_2        EXTI_IMR1_IM2           /*!< Extended line 2 */
#define LL_EXTI_LINE_3        EXTI_IMR1_IM3           /*!< Extended line 3 */
#define LL_EXTI_LINE_4        EXTI_IMR1_IM4           /*!< Extended line 4 */
#define LL_EXTI_LINE_5        EXTI_IMR1_IM5           /*!< Extended line 5 */
#define LL_EXTI_LINE_6        EXTI_IMR1_IM6           /*!< Extended line 6 */
#define LL_EXTI_LINE_7        EXTI_IMR1_IM7           /*!< Extended line 7 */
#define LL_EXTI_LINE_8        EXTI_IMR1_IM8           /*!< Extended line 8 */
#define LL_EXTI_LINE_9        EXTI_IMR1_IM9           /*!< Extended line 9 */
#define LL_EXTI_LINE_10       EXTI_IMR1_IM10          /*!< Extended line 10 */
#define LL_EXTI_LINE_11       EXTI_IMR1_IM11          /*!< Extended line 11 */
#define LL_EXTI_LINE_12       EXTI_IMR1_IM12          /*!< Extended line 12 */
#define LL_EXTI_LINE_13       EXTI_IMR1_IM13          /*!< Extended line 13 */
#define LL_EXTI_LINE_14       EXTI_IMR1_IM14          /*!< Extended line 14 */
#define LL_EXTI_LINE_15       EXTI_IMR1_IM15          /*!< Extended line 15 */
#define LL_EXTI_LINE_16       EXTI_IMR1_IM16          /*!< Extended line 16 */
#define LL_EXTI_LINE_17       EXTI_IMR1_IM17          /*!< Extended line 17 */
#define LL_EXTI_LINE_18       EXTI_IMR1_IM18          /*!< Extended line 18 */
#define LL_EXTI_LINE_19       EXTI_IMR1_IM19          /*!< Extended line 19 */
#define LL_EXTI_LINE_20       EXTI_IMR1_IM20          /*!< Extended line 20 */
#define LL_EXTI_LINE_21       EXTI_IMR1_IM21          /*!< Extended line 21 */
#define LL_EXTI_LINE_22       EXTI_IMR1_IM22          /*!< Extended line 22 */
#define LL_EXTI_LINE_23       EXTI_IMR1_IM23          /*!< Extended line 23 */
#define LL_EXTI_LINE_24       EXTI_IMR1_IM24          /*!< Extended line 24 */
#define LL_EXTI_LINE_25       EXTI_IMR1_IM25          /*!< Extended line 25 */
#define LL_EXTI_LINE_26       EXTI_IMR1_IM26          /*!< Extended line 26 */
#define LL_EXTI_LINE_27       EXTI_IMR1_IM27          /*!< Extended line 27 */
#define LL_EXTI_LINE_28       EXTI_IMR1_IM28          /*!< Extended line 28 */
#define LL_EXTI_LINE_29       EXTI_IMR1_IM29          /*!< Extended line 29 */
#define LL_EXTI_LINE_30       EXTI_IMR1_IM30          /*!< Extended line 30 */
#define LL_EXTI_LINE_31       EXTI_IMR1_IM31          /*!< Extended line 31 */
#define LL_EXTI_LINE_ALL_0_31 ((uint32_t)0xFFFFFFFF)  /*!< All Extended line */


/**
  * @}
  */

/** @addtogroup EXTI_LL_EC_LINE
  * @{
  */
#define LL_EXTI_LINE_32        EXTI_IMR2_IM32          /*!< Extended line 32 */
#define LL_EXTI_LINE_33        EXTI_IMR2_IM33          /*!< Extended line 33 */
#define LL_EXTI_LINE_34        EXTI_IMR2_IM34          /*!< Extended line 34 */
#define LL_EXTI_LINE_35        EXTI_IMR2_IM35          /*!< Extended line 35 */
#define LL_EXTI_LINE_36        EXTI_IMR2_IM36          /*!< Extended line 36 */
#define LL_EXTI_LINE_37        EXTI_IMR2_IM37          /*!< Extended line 37 */
#define LL_EXTI_LINE_38        EXTI_IMR2_IM38          /*!< Extended line 38 */
#define LL_EXTI_LINE_39        EXTI_IMR2_IM39          /*!< Extended line 39 */
#define LL_EXTI_LINE_ALL_32_63 ((uint32_t)0x000000FF)  /*!< All Extended line */
/**
  * @}
  */

/** @addtogroup EXTI_LL_EC_LINE
  * @{
  */
#define LL_EXTI_LINE_ALL ((uint32_t)0xFFFFFFFF)  /*!< All Extended line */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup EXTI_LL_Exported_Macros EXTI Exported Macros
  * @{
  */

/** @defgroup EXTI_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in EXTI register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_EXTI_WriteReg(__REG__, __VALUE__) WRITE_REG(EXTI->__REG__, (__VALUE__))

/**
  * @brief  Read a value in EXTI register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_EXTI_ReadReg(__REG__) READ_REG(EXTI->__REG__)
/**
  * @}
  */


/**
  * @}
  */



/* Exported functions --------------------------------------------------------*/
/** @defgroup EXTI_LL_Exported_Functions EXTI Exported Functions
 * @{
 */
/** @defgroup EXTI_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable ExtiLine Interrupt request for Lines in range 0 to 31
  * @note The reset value for the direct lines (line 17, lines from 23 to 31)
  *       is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @rmtoll IMR1         IMx           LL_EXTI_EnableIT_0_31
  * @param  ExtiLine This parameter can be one of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_17
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  *         @arg @ref LL_EXTI_LINE_23
  *         @arg @ref LL_EXTI_LINE_24
  *         @arg @ref LL_EXTI_LINE_25
  *         @arg @ref LL_EXTI_LINE_26
  *         @arg @ref LL_EXTI_LINE_27
  *         @arg @ref LL_EXTI_LINE_28
  *         @arg @ref LL_EXTI_LINE_29
  *         @arg @ref LL_EXTI_LINE_30
  *         @arg @ref LL_EXTI_LINE_31
  *         @arg @ref LL_EXTI_LINE_ALL_0_31
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_EnableIT_0_31(uint32_t ExtiLine)
{
  SET_BIT(EXTI->IMR1, ExtiLine);
}
/**
  * @brief  Enable ExtiLine Interrupt request for Lines in range 32 to 63
  * @note The reset value for the direct lines (lines from 32 to 34, line
  *       39) is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @rmtoll IMR2         IMx           LL_EXTI_EnableIT_32_63
  * @param  ExtiLine This parameter can be one of the following values:
  *         @arg @ref LL_EXTI_LINE_32
  *         @arg @ref LL_EXTI_LINE_33
  *         @arg @ref LL_EXTI_LINE_34
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  *         @arg @ref LL_EXTI_LINE_39
  *         @arg @ref LL_EXTI_LINE_ALL_32_63
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_EnableIT_32_63(uint32_t ExtiLine)
{
  SET_BIT(EXTI->IMR2, ExtiLine);
}

/**
  * @brief  Disable ExtiLine Interrupt request for Lines in range 0 to 31
  * @note The reset value for the direct lines (line 17, lines from 23 to 31)
  *       is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @rmtoll IMR1         IMx           LL_EXTI_DisableIT_0_31
  * @param  ExtiLine This parameter can be one of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_17
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  *         @arg @ref LL_EXTI_LINE_23
  *         @arg @ref LL_EXTI_LINE_24
  *         @arg @ref LL_EXTI_LINE_25
  *         @arg @ref LL_EXTI_LINE_26
  *         @arg @ref LL_EXTI_LINE_27
  *         @arg @ref LL_EXTI_LINE_28
  *         @arg @ref LL_EXTI_LINE_29
  *         @arg @ref LL_EXTI_LINE_30
  *         @arg @ref LL_EXTI_LINE_31
  *         @arg @ref LL_EXTI_LINE_ALL_0_31
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_DisableIT_0_31(uint32_t ExtiLine)
{
  CLEAR_BIT(EXTI->IMR1, ExtiLine);
}

/**
  * @brief  Disable ExtiLine Interrupt request for Lines in range 32 to 63
  * @note The reset value for the direct lines (lines from 32 to 34, line
  *       39) is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @rmtoll IMR2         IMx           LL_EXTI_DisableIT_32_63
  * @param  ExtiLine This parameter can be one of the following values:
  *         @arg @ref LL_EXTI_LINE_32
  *         @arg @ref LL_EXTI_LINE_33
  *         @arg @ref LL_EXTI_LINE_34
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  *         @arg @ref LL_EXTI_LINE_39
  *         @arg @ref LL_EXTI_LINE_ALL_32_63
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_DisableIT_32_63(uint32_t ExtiLine)
{
  CLEAR_BIT(EXTI->IMR2, ExtiLine);
}

/**
  * @brief  Indicate if ExtiLine Interrupt request is enabled for Lines in range 0 to 31
  * @note The reset value for the direct lines (line 17, lines from 23 to 31)
  *       is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @rmtoll IMR1         IMx           LL_EXTI_IsEnabledIT_0_31
  * @param  ExtiLine This parameter can be one of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_17
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  *         @arg @ref LL_EXTI_LINE_23
  *         @arg @ref LL_EXTI_LINE_24
  *         @arg @ref LL_EXTI_LINE_25
  *         @arg @ref LL_EXTI_LINE_26
  *         @arg @ref LL_EXTI_LINE_27
  *         @arg @ref LL_EXTI_LINE_28
  *         @arg @ref LL_EXTI_LINE_29
  *         @arg @ref LL_EXTI_LINE_30
  *         @arg @ref LL_EXTI_LINE_31
  *         @arg @ref LL_EXTI_LINE_ALL_0_31
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsEnabledIT_0_31(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->IMR1, ExtiLine) == (ExtiLine));
}

/**
  * @brief  Indicate if ExtiLine Interrupt request is enabled for Lines in range 32 to 63
  * @note The reset value for the direct lines (lines from 32 to 34, line
  *       39) is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @rmtoll IMR2         IMx           LL_EXTI_IsEnabledIT_32_63
  * @param  ExtiLine This parameter can be one of the following values:
  *         @arg @ref LL_EXTI_LINE_32
  *         @arg @ref LL_EXTI_LINE_33
  *         @arg @ref LL_EXTI_LINE_34
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  *         @arg @ref LL_EXTI_LINE_39
  *         @arg @ref LL_EXTI_LINE_ALL_32_63
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsEnabledIT_32_63(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->IMR2, ExtiLine) == (ExtiLine));
}

/**
  * @}
  */

/** @defgroup EXTI_LL_EF_Event_Management Event_Management
  * @{
  */

/**
  * @brief  Enable ExtiLine Event request for Lines in range 0 to 31
  * @rmtoll EMR1         EMx           LL_EXTI_EnableEvent_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_17
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  *         @arg @ref LL_EXTI_LINE_23
  *         @arg @ref LL_EXTI_LINE_24
  *         @arg @ref LL_EXTI_LINE_25
  *         @arg @ref LL_EXTI_LINE_26
  *         @arg @ref LL_EXTI_LINE_27
  *         @arg @ref LL_EXTI_LINE_28
  *         @arg @ref LL_EXTI_LINE_29
  *         @arg @ref LL_EXTI_LINE_30
  *         @arg @ref LL_EXTI_LINE_31
  *         @arg @ref LL_EXTI_LINE_ALL_0_31
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_EnableEvent_0_31(uint32_t ExtiLine)
{
  SET_BIT(EXTI->EMR1, ExtiLine);

}

/**
  * @brief  Enable ExtiLine Event request for Lines in range 32 to 63
  * @rmtoll EMR2         EMx           LL_EXTI_EnableEvent_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_32
  *         @arg @ref LL_EXTI_LINE_33
  *         @arg @ref LL_EXTI_LINE_34
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  *         @arg @ref LL_EXTI_LINE_39
  *         @arg @ref LL_EXTI_LINE_ALL_32_63
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_EnableEvent_32_63(uint32_t ExtiLine)
{
  SET_BIT(EXTI->EMR2, ExtiLine);
}

/**
  * @brief  Disable ExtiLine Event request for Lines in range 0 to 31
  * @rmtoll EMR1         EMx           LL_EXTI_DisableEvent_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_17
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  *         @arg @ref LL_EXTI_LINE_23
  *         @arg @ref LL_EXTI_LINE_24
  *         @arg @ref LL_EXTI_LINE_25
  *         @arg @ref LL_EXTI_LINE_26
  *         @arg @ref LL_EXTI_LINE_27
  *         @arg @ref LL_EXTI_LINE_28
  *         @arg @ref LL_EXTI_LINE_29
  *         @arg @ref LL_EXTI_LINE_30
  *         @arg @ref LL_EXTI_LINE_31
  *         @arg @ref LL_EXTI_LINE_ALL
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_DisableEvent_0_31(uint32_t ExtiLine)
{
  CLEAR_BIT(EXTI->EMR1, ExtiLine);
}

/**
  * @brief  Disable ExtiLine Event request for Lines in range 32 to 63
  * @rmtoll EMR2         EMx           LL_EXTI_DisableEvent_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_32
  *         @arg @ref LL_EXTI_LINE_33
  *         @arg @ref LL_EXTI_LINE_34
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  *         @arg @ref LL_EXTI_LINE_39
  *         @arg @ref LL_EXTI_LINE_ALL_32_63
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_DisableEvent_32_63(uint32_t ExtiLine)
{
  CLEAR_BIT(EXTI->EMR2, ExtiLine);
}

/**
  * @brief  Indicate if ExtiLine Event request is enabled for Lines in range 0 to 31
  * @rmtoll EMR1         EMx           LL_EXTI_IsEnabledEvent_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_17
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  *         @arg @ref LL_EXTI_LINE_23
  *         @arg @ref LL_EXTI_LINE_24
  *         @arg @ref LL_EXTI_LINE_25
  *         @arg @ref LL_EXTI_LINE_26
  *         @arg @ref LL_EXTI_LINE_27
  *         @arg @ref LL_EXTI_LINE_28
  *         @arg @ref LL_EXTI_LINE_29
  *         @arg @ref LL_EXTI_LINE_30
  *         @arg @ref LL_EXTI_LINE_31
  *         @arg @ref LL_EXTI_LINE_ALL_0_31
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsEnabledEvent_0_31(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->EMR1, ExtiLine) == (ExtiLine));

}

/**
  * @brief  Indicate if ExtiLine Event request is enabled for Lines in range 32 to 63
  * @rmtoll EMR2         EMx           LL_EXTI_IsEnabledEvent_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_32
  *         @arg @ref LL_EXTI_LINE_33
  *         @arg @ref LL_EXTI_LINE_34
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  *         @arg @ref LL_EXTI_LINE_39
  *         @arg @ref LL_EXTI_LINE_ALL_32_63
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsEnabledEvent_32_63(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->EMR2, ExtiLine) == (ExtiLine));
}

/**
  * @}
  */

/** @defgroup EXTI_LL_EF_Rising_Trigger_Management Rising_Trigger_Management
  * @{
  */

/**
  * @brief  Enable ExtiLine Rising Edge Trigger for Lines in range 0 to 31
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a rising edge on a configurable interrupt
  *       line occurs during a write operation in the EXTI_RTSR register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @rmtoll RTSR1        RTx           LL_EXTI_EnableRisingTrig_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_EnableRisingTrig_0_31(uint32_t ExtiLine)
{
  SET_BIT(EXTI->RTSR1, ExtiLine);

}

/**
  * @brief  Enable ExtiLine Rising Edge Trigger for Lines in range 32 to 63
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a rising edge on a configurable interrupt
  *       line occurs during a write operation in the EXTI_RTSR register, the
  *       pending bit is not set.Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @rmtoll RTSR2        RTx           LL_EXTI_EnableRisingTrig_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_EnableRisingTrig_32_63(uint32_t ExtiLine)
{
  SET_BIT(EXTI->RTSR2, ExtiLine);
}

/**
  * @brief  Disable ExtiLine Rising Edge Trigger for Lines in range 0 to 31
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a rising edge on a configurable interrupt
  *       line occurs during a write operation in the EXTI_RTSR register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @rmtoll RTSR1        RTx           LL_EXTI_DisableRisingTrig_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_DisableRisingTrig_0_31(uint32_t ExtiLine)
{
  CLEAR_BIT(EXTI->RTSR1, ExtiLine);

}

/**
  * @brief  Disable ExtiLine Rising Edge Trigger for Lines in range 32 to 63
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a rising edge on a configurable interrupt
  *       line occurs during a write operation in the EXTI_RTSR register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @rmtoll RTSR2        RTx           LL_EXTI_DisableRisingTrig_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_DisableRisingTrig_32_63(uint32_t ExtiLine)
{
  CLEAR_BIT(EXTI->RTSR2, ExtiLine);
}

/**
  * @brief  Check if rising edge trigger is enabled for Lines in range 0 to 31
  * @rmtoll RTSR1        RTx           LL_EXTI_IsEnabledRisingTrig_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsEnabledRisingTrig_0_31(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->RTSR1, ExtiLine) == (ExtiLine));
}

/**
  * @brief  Check if rising edge trigger is enabled for Lines in range 32 to 63
  * @rmtoll RTSR2        RTx           LL_EXTI_IsEnabledRisingTrig_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsEnabledRisingTrig_32_63(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->RTSR2, ExtiLine) == (ExtiLine));
}

/**
  * @}
  */

/** @defgroup EXTI_LL_EF_Falling_Trigger_Management Falling_Trigger_Management
  * @{
  */

/**
  * @brief  Enable ExtiLine Falling Edge Trigger for Lines in range 0 to 31
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a falling edge on a configurable interrupt
  *       line occurs during a write operation in the EXTI_FTSR register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @rmtoll FTSR1        FTx           LL_EXTI_EnableFallingTrig_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_EnableFallingTrig_0_31(uint32_t ExtiLine)
{
  SET_BIT(EXTI->FTSR1, ExtiLine);
}

/**
  * @brief  Enable ExtiLine Falling Edge Trigger for Lines in range 32 to 63
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a Falling edge on a configurable interrupt
  *       line occurs during a write operation in the EXTI_FTSR register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @rmtoll FTSR2        FTx           LL_EXTI_EnableFallingTrig_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_EnableFallingTrig_32_63(uint32_t ExtiLine)
{
  SET_BIT(EXTI->FTSR2, ExtiLine);
}

/**
  * @brief  Disable ExtiLine Falling Edge Trigger for Lines in range 0 to 31
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a Falling edge on a configurable interrupt
  *       line occurs during a write operation in the EXTI_FTSR register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for the same interrupt line.
  *       In this case, both generate a trigger condition.
  * @rmtoll FTSR1        FTx           LL_EXTI_DisableFallingTrig_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_DisableFallingTrig_0_31(uint32_t ExtiLine)
{
  CLEAR_BIT(EXTI->FTSR1, ExtiLine);
}

/**
  * @brief  Disable ExtiLine Falling Edge Trigger for Lines in range 32 to 63
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a Falling edge on a configurable interrupt
  *       line occurs during a write operation in the EXTI_FTSR register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for the same interrupt line.
  *       In this case, both generate a trigger condition.
  * @rmtoll FTSR2        FTx           LL_EXTI_DisableFallingTrig_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_DisableFallingTrig_32_63(uint32_t ExtiLine)
{
  CLEAR_BIT(EXTI->FTSR2, ExtiLine);
}

/**
  * @brief  Check if falling edge trigger is enabled for Lines in range 0 to 31
  * @rmtoll FTSR1        FTx           LL_EXTI_IsEnabledFallingTrig_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsEnabledFallingTrig_0_31(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->FTSR1, ExtiLine) == (ExtiLine));
}

/**
  * @brief  Check if falling edge trigger is enabled for Lines in range 32 to 63
  * @rmtoll FTSR2        FTx           LL_EXTI_IsEnabledFallingTrig_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsEnabledFallingTrig_32_63(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->FTSR2, ExtiLine) == (ExtiLine));
}

/**
  * @}
  */

/** @defgroup EXTI_LL_EF_Software_Interrupt_Management Software_Interrupt_Management
  * @{
  */

/**
  * @brief  Generate a software Interrupt Event for Lines in range 0 to 31
  * @note If the interrupt is enabled on this line inthe EXTI_IMR, writing a 1 to
  *       this bit when it is at '0' sets the corresponding pending bit in EXTI_PR
  *       resulting in an interrupt request generation.
  *       This bit is cleared by clearing the corresponding bit in the EXTI_PR
  *       register (by writing a 1 into the bit)
  * @rmtoll SWIER1       SWIx          LL_EXTI_GenerateSWI_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_GenerateSWI_0_31(uint32_t ExtiLine)
{
  SET_BIT(EXTI->SWIER1, ExtiLine);
}

/**
  * @brief  Generate a software Interrupt Event for Lines in range 32 to 63
  * @note If the interrupt is enabled on this line inthe EXTI_IMR, writing a 1 to
  *       this bit when it is at '0' sets the corresponding pending bit in EXTI_PR
  *       resulting in an interrupt request generation.
  *       This bit is cleared by clearing the corresponding bit in the EXTI_PR
  *       register (by writing a 1 into the bit)
  * @rmtoll SWIER2       SWIx          LL_EXTI_GenerateSWI_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_GenerateSWI_32_63(uint32_t ExtiLine)
{
  SET_BIT(EXTI->SWIER2, ExtiLine);
}

/**
  * @}
  */

/** @defgroup EXTI_LL_EF_Flag_Management Flag_Management
  * @{
  */

/**
  * @brief  Check if the ExtLine Flag is set or not for Lines in range 0 to 31
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @rmtoll PR1          PRx           LL_EXTI_IsActiveFlag_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsActiveFlag_0_31(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->PR1, ExtiLine) == (ExtiLine));
}

/**
  * @brief  Check if the ExtLine Flag is set or not for  Lines in range 32 to 63
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @rmtoll PR2          PRx           LL_EXTI_IsActiveFlag_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_EXTI_IsActiveFlag_32_63(uint32_t ExtiLine)
{
  return (READ_BIT(EXTI->PR2, ExtiLine) == (ExtiLine));
}

/**
  * @brief  Read ExtLine Combination Flag for Lines in range 0 to 31
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @rmtoll PR1          PRx           LL_EXTI_ReadFlag_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval @note This bit is set when the selected edge event arrives on the interrupt
  */
__STATIC_INLINE uint32_t LL_EXTI_ReadFlag_0_31(uint32_t ExtiLine)
{
  return (uint32_t)(READ_BIT(EXTI->PR1, ExtiLine));
}


/**
  * @brief  Read ExtLine Combination Flag for  Lines in range 32 to 63
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @rmtoll PR2          PRx           LL_EXTI_ReadFlag_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval @note This bit is set when the selected edge event arrives on the interrupt
  */
__STATIC_INLINE uint32_t LL_EXTI_ReadFlag_32_63(uint32_t ExtiLine)
{
  return (uint32_t)(READ_BIT(EXTI->PR2, ExtiLine));
}

/**
  * @brief  Clear ExtLine Flags  for Lines in range 0 to 31
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @rmtoll PR1          PRx           LL_EXTI_ClearFlag_0_31
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_0
  *         @arg @ref LL_EXTI_LINE_1
  *         @arg @ref LL_EXTI_LINE_2
  *         @arg @ref LL_EXTI_LINE_3
  *         @arg @ref LL_EXTI_LINE_4
  *         @arg @ref LL_EXTI_LINE_5
  *         @arg @ref LL_EXTI_LINE_6
  *         @arg @ref LL_EXTI_LINE_7
  *         @arg @ref LL_EXTI_LINE_8
  *         @arg @ref LL_EXTI_LINE_9
  *         @arg @ref LL_EXTI_LINE_10
  *         @arg @ref LL_EXTI_LINE_11
  *         @arg @ref LL_EXTI_LINE_12
  *         @arg @ref LL_EXTI_LINE_13
  *         @arg @ref LL_EXTI_LINE_14
  *         @arg @ref LL_EXTI_LINE_15
  *         @arg @ref LL_EXTI_LINE_16
  *         @arg @ref LL_EXTI_LINE_18
  *         @arg @ref LL_EXTI_LINE_19
  *         @arg @ref LL_EXTI_LINE_20
  *         @arg @ref LL_EXTI_LINE_21
  *         @arg @ref LL_EXTI_LINE_22
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_ClearFlag_0_31(uint32_t ExtiLine)
{
  WRITE_REG(EXTI->PR1, ExtiLine);
}

/**
  * @brief  Clear ExtLine Flags for  Lines in range 32 to 63
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @rmtoll PR2          PRx           LL_EXTI_ClearFlag_32_63
  * @param  ExtiLine This parameter can be a combination of the following values:
  *         @arg @ref LL_EXTI_LINE_35
  *         @arg @ref LL_EXTI_LINE_36
  *         @arg @ref LL_EXTI_LINE_37
  *         @arg @ref LL_EXTI_LINE_38
  * @retval None
  */
__STATIC_INLINE void LL_EXTI_ClearFlag_32_63(uint32_t ExtiLine)
{
  WRITE_REG(EXTI->PR2, ExtiLine);
}

/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

#endif /* EXTI */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_LL_EXTI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

