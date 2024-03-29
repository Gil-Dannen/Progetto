/**
  ******************************************************************************
  * @file    stm32l4xx_ll_tim.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    16-September-2015
  * @brief   Header file of TIM LL module.
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
#ifndef __STM32L4xx_LL_TIM_H
#define __STM32L4xx_LL_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"

/** @addtogroup STM32L4xx_LL_Driver
  * @{
  */

#if defined (TIM1) || defined (TIM8) || defined (TIM2) || defined (TIM3) || defined (TIM4) || defined (TIM5) || defined (TIM15) || defined (TIM16) || defined (TIM17) || defined (TIM6) || defined (TIM7)

/** @defgroup TIM_LL TIM
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup TIM_LL_Private_Variables TIM Private Variables
  * @{
  */
static const uint8_t OFFSET_TAB_CCMRx[] = 
  {
    0x00,   /* 0: TIMx_CH1  */
    0x00,   /* 1: TIMx_CH1N */
    0x00,   /* 2: TIMx_CH2  */
    0x00,   /* 3: TIMx_CH2N */
    0x04,   /* 4: TIMx_CH3  */
    0x04,   /* 5: TIMx_CH3N */    
    0x04,   /* 6: TIMx_CH4  */
    0x3C,   /* 7: TIMx_CH5  */
    0x3C    /* 8: TIMx_CH6  */
  };

static const uint8_t SHIFT_TAB_OCxx[] =
{
  0,            /* 0: OC1M, OC1FE, OC1PE */
  0,            /* 1: - NA */
  8,            /* 2: OC2M, OC2FE, OC2PE */
  0,            /* 3: - NA */
  0,            /* 4: OC3M, OC3FE, OC3PE */
  0,            /* 5: - NA */
  8,            /* 6: OC4M, OC4FE, OC4PE */
  0,            /* 7: OC5M, OC5FE, OC5PE */
  8             /* 8: OC6M, OC6FE, OC6PE */
};

static const uint8_t SHIFT_TAB_ICxx[] =
{
  0,            /* 0: CC1S, IC1PSC, IC1F */
  0,            /* 1: - NA */
  8,            /* 2: CC2S, IC2PSC, IC2F */
  0,            /* 3: - NA */
  0,            /* 4: CC3S, IC3PSC, IC3F */
  0,            /* 5: - NA */
  8,            /* 6: CC4S, IC4PSC, IC4F */
  0,            /* 7: - NA */
  0             /* 8: - NA */
};

static const uint8_t SHIFT_TAB_CCxP[] =
{
  0,            /* 0: CC1P */
  2,            /* 1: CC1NP */
  4,            /* 2: CC2P */
  6,            /* 3: CC2NP */
  8,            /* 4: CC3P */
  10,           /* 5: CC3NP */
  12,           /* 6: CC4P */
  16,           /* 7: CC5P */
  20            /* 8: CC6P */
};

static const uint8_t SHIFT_TAB_OISx[] =
{
  0,            /* 0: OIS1 */
  1,            /* 1: OIS1N */
  2,            /* 2: OIS2 */
  3,            /* 3: OIS2N */
  4,            /* 4: OIS3 */
  5,            /* 5: OIS3N */
  6,            /* 6: OIS4 */
  8,            /* 7: OIS5 */
  10            /* 8: OIS6 */
};
/**
  * @}
  */


/* Private constants ---------------------------------------------------------*/
/** @defgroup TIM_LL_Private_Constants TIM Private Constants
  * @{
  */
/* Generic bit definitions for TIMx_OR2 register */
#define TIMx_OR2_BKINE     TIM1_OR2_BKINE     /*!< BRK BKIN input enable */
#define TIMx_OR2_BKCOMP1E  TIM1_OR2_BKCMP1E   /*!< BRK COMP1 enable */
#define TIMx_OR2_BKCOMP2E  TIM1_OR2_BKCMP2E   /*!< BRK COMP2 enable */
#define TIMx_OR2_BKDFBK0E  TIM1_OR2_BKDFBK0E  /*!< BRK DFSDM_BREAK[0] enable */
#define TIMx_OR2_BKINP     TIM1_OR2_BKINP     /*!< BRK BKIN input polarity */
#define TIMx_OR2_BKCOMP1P  TIM1_OR2_BKCMP1P   /*!< BRK COMP1 input polarity */
#define TIMx_OR2_BKCOMP2P  TIM1_OR2_BKCMP2P   /*!< BRK COMP2 input polarity */
#define TIMx_OR2_ETRSEL    TIM1_OR2_ETRSEL    /*!< TIMx ETR source selection */

/* Generic bit definitions for TIMx_OR3 register */
#define TIMx_OR3_BK2INE    TIM1_OR3_BK2INE     /*!< BRK2 BKIN2 input enable */
#define TIMx_OR3_BK2COMP1E TIM1_OR3_BK2CMP1E   /*!< BRK2 COMP1 enable */
#define TIMx_OR3_BK2COMP2E TIM1_OR3_BK2CMP2E   /*!< BRK2 COMP2 enable */
#define TIMx_OR3_BK2DFBK1E TIM1_OR3_BK2DFBK1E  /*!< BRK2 DFSDM_BREAK[1] enable */
#define TIMx_OR3_BK2INP    TIM1_OR3_BK2INP     /*!< BRK2 BKIN2 input polarity */
#define TIMx_OR3_BK2COMP1P TIM1_OR3_BK2CMP1P   /*!< BRK2 COMP1 input polarity */
#define TIMx_OR3_BK2COMP2P  TIM1_OR3_BK2CMP2P  /*!< BRK2 COMP2 input polarity */

/* Remap mask definitions */
#define TIMx_OR1_RMP_SHIFT ((uint32_t)16)
#define TIMx_OR1_RMP_MASK  ((uint32_t)0x0000FFFF)
#define TIM1_OR1_RMP_MASK  ((uint32_t)((TIM1_OR1_ETR_ADC1_RMP | TIM1_OR1_ETR_ADC3_RMP | TIM1_OR1_TI1_RMP) << TIMx_OR1_RMP_SHIFT))
#define TIM2_OR1_RMP_MASK  ((uint32_t)((TIM2_OR1_TI4_RMP | TIM2_OR1_ETR1_RMP | TIM2_OR1_ITR1_RMP) << TIMx_OR1_RMP_SHIFT))
#define TIM3_OR1_RMP_MASK  ((uint32_t)(TIM3_OR1_TI1_RMP) << TIMx_OR1_RMP_SHIFT)
#define TIM8_OR1_RMP_MASK  ((uint32_t)((TIM8_OR1_ETR_ADC2_RMP | TIM8_OR1_ETR_ADC3_RMP | TIM8_OR1_TI1_RMP) << TIMx_OR1_RMP_SHIFT))
#define TIM15_OR1_RMP_MASK ((uint32_t)((TIM15_OR1_TI1_RMP) << TIMx_OR1_RMP_SHIFT))
#define TIM16_OR1_RMP_MASK ((uint32_t)((TIM16_OR1_TI1_RMP) << TIMx_OR1_RMP_SHIFT))
#define TIM17_OR1_RMP_MASK ((uint32_t)((TIM17_OR1_TI1_RMP) << TIMx_OR1_RMP_SHIFT))

/* Mask used to set the TDG[x:0] of the DTG bits of the TIMx_BDTR register */
#define DT_DELAY_1 ((uint8_t)0x7F)
#define DT_DELAY_2 ((uint8_t)0x3F)
#define DT_DELAY_3 ((uint8_t)0x1F)
#define DT_DELAY_4 ((uint8_t)0x1F)
    
/* Mask used to set the DTG[7:5] bits of the DTG bits of the TIMx_BDTR register */
#define DT_RANGE_1 ((uint8_t)0x00)
#define DT_RANGE_2 ((uint8_t)0x80)
#define DT_RANGE_3 ((uint8_t)0xC0)
#define DT_RANGE_4 ((uint8_t)0xE0)
/**
  * @}
  */


/* Private macros ------------------------------------------------------------*/
/** @defgroup TIM_LL_Private_Macros TIM Private Macros
  * @{
  */
/** @brief  Convert channel id into channel index
  * @param  __CHANNEL__: This parameter can be one of the following values: 
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval none
  */
#define TIM_GET_CHANNEL_INDEX( __CHANNEL__)  \
  (((__CHANNEL__) == LL_TIM_CHANNEL_CH1) ?  0 :\
   ((__CHANNEL__) == LL_TIM_CHANNEL_CH1N) ? 1 :\
   ((__CHANNEL__) == LL_TIM_CHANNEL_CH2) ?  2 :\
   ((__CHANNEL__) == LL_TIM_CHANNEL_CH2N) ? 3 :\
   ((__CHANNEL__) == LL_TIM_CHANNEL_CH3) ?  4 :\
   ((__CHANNEL__) == LL_TIM_CHANNEL_CH3N) ? 5 :\
   ((__CHANNEL__) == LL_TIM_CHANNEL_CH4) ?  6 :\
   ((__CHANNEL__) == LL_TIM_CHANNEL_CH5) ?  7 : 8)

/** @brief  Calculate the deadtime sampling period(in ps)
  * @param  __TIMCLK__: timer input clock frequency (in Hz).
  * @param  __CKD__: This parameter can be one of the following values: 
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  * @retval none
  */
#define TIM_CALC_DTS(__TIMCLK__, __CKD__)                                                       \
    (((int)(__CKD__) == LL_TIM_CLOCKDIVISION_DIV1) ? ((uint64_t)1000000000000/(__TIMCLK__))        : \
     ((int)(__CKD__) == LL_TIM_CLOCKDIVISION_DIV2) ? ((uint64_t)1000000000000/((__TIMCLK__) >> 1)) : \
     ((uint64_t)1000000000000/((__TIMCLK__) >> 2)))
/**
  * @}
  */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup TIM_LL_Exported_Constants TIM Exported Constants
  * @{
  */

/** @defgroup TIM_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_TIM_ReadReg function
  * @{
  */
#define LL_TIM_SR_UIF                          TIM_SR_UIF
#define LL_TIM_SR_CC1IF                        TIM_SR_CC1IF
#define LL_TIM_SR_CC2IF                        TIM_SR_CC2IF
#define LL_TIM_SR_CC3IF                        TIM_SR_CC3IF
#define LL_TIM_SR_CC4IF                        TIM_SR_CC4IF
#define LL_TIM_SR_CC5IF                        TIM_SR_CC5IF
#define LL_TIM_SR_CC6IF                        TIM_SR_CC6IF
#define LL_TIM_SR_COMIF                        TIM_SR_COMIF
#define LL_TIM_SR_TIF                          TIM_SR_TIF
#define LL_TIM_SR_BIF                          TIM_SR_BIF
#define LL_TIM_SR_B2IF                         TIM_SR_B2IF
#define LL_TIM_SR_CC1OF                        TIM_SR_CC1OF
#define LL_TIM_SR_CC2OF                        TIM_SR_CC2OF
#define LL_TIM_SR_CC3OF                        TIM_SR_CC3OF
#define LL_TIM_SR_CC4OF                        TIM_SR_CC4OF
#define LL_TIM_SR_SBIF                         TIM_SR_SBIF
/**
  * @}
  */

/** @defgroup TIM_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_TIM_ReadReg and  LL_TIM_WriteReg functions
  * @{
  */
#define LL_TIM_DIER_UIE                        TIM_DIER_UIE
#define LL_TIM_DIER_CC1IE                      TIM_DIER_CC1IE
#define LL_TIM_DIER_CC2IE                      TIM_DIER_CC2IE
#define LL_TIM_DIER_CC3IE                      TIM_DIER_CC3IE
#define LL_TIM_DIER_CC4IE                      TIM_DIER_CC4IE
#define LL_TIM_DIER_COMIE                      TIM_DIER_COMIE
#define LL_TIM_DIER_TIE                        TIM_DIER_TIE
#define LL_TIM_DIER_BIE                        TIM_DIER_BIE
/**
  * @}
  */

/** @defgroup TIM_LL_EC_UPDATESOURCE UPDATESOURCE
  * @{
  */
#define LL_TIM_UPDATESOURCE_REGULAR            ((uint32_t)0x00000000) /*!< Counter overflow/underflow, Setting the UG bit or Update generation through the slave mode controller generates an update request */ 
#define LL_TIM_UPDATESOURCE_COUNTER            TIM_CR1_URS            /*!< Only counter overflow/underflow generates an update request */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ONEPULSEMODE ONEPULSEMODE
  * @{
  */
#define LL_TIM_ONEPULSEMODE_SINGLE             TIM_CR1_OPM            /*!< Counter is not stopped at update event */ 
#define LL_TIM_ONEPULSEMODE_REPETITIVE         ((uint32_t)0x00000000) /*!< Counter stops counting at the next update event */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_COUNTERMODE COUNTERMODE
  * @{
  */
#define LL_TIM_COUNTERMODE_UP                  ((uint32_t)0x00000000)/*!<Counter used as upcounter */ 
#define LL_TIM_COUNTERMODE_DOWN                TIM_CR1_DIR           /*!< Counter used as downcounter */ 
#define LL_TIM_COUNTERMODE_CENTER_UP           TIM_CR1_CMS_0         /*!< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting down. */ 
#define LL_TIM_COUNTERMODE_CENTER_DOWN         TIM_CR1_CMS_1         /*!<The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting up */ 
#define LL_TIM_COUNTERMODE_CENTER_UP_DOWN      TIM_CR1_CMS           /*!< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting up or down. */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CLOCKDIVISION CLOCKDIVISION
  * @{
  */
#define LL_TIM_CLOCKDIVISION_DIV1              ((uint32_t)0x00000000) /*!< tDTS=tCK_INT */ 
#define LL_TIM_CLOCKDIVISION_DIV2              TIM_CR1_CKD_0          /*!< tDTS=2*tCK_INT */ 
#define LL_TIM_CLOCKDIVISION_DIV4              TIM_CR1_CKD_1          /*!< tDTS=4*tCK_INT */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_COUNTERDIRECTION COUNTERDIRECTION
  * @{
  */
#define LL_TIM_COUNTERDIRECTION_UP             ((uint32_t)0x00000000) /*!< Timer counter counts up */
#define LL_TIM_COUNTERDIRECTION_DOWN           TIM_CR1_DIR  /*!< Timer counter counts down */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CCUPDATESOURCE CCUPDATESOURCE
  * @{
  */
#define LL_TIM_CCUPDATESOURCE_COMG_ONLY        ((uint32_t)0x00000000) /*!< Capture/compare control bits are updated by setting the COMG bit only */ 
#define LL_TIM_CCUPDATESOURCE_COMG_AND_TRGI    TIM_CR2_CCUS           /*!< Capture/compare control bits are updated by setting the COMG bit or when a rising edge occurs on trigger input (TRGI) */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CCDMAREQUEST CCDMAREQUEST
  * @{
  */
#define LL_TIM_CCDMAREQUEST_CC                 ((uint32_t)0x00000000) /*!< CCx DMA request sent when CCx event occurs */ 
#define LL_TIM_CCDMAREQUEST_UPDATE             TIM_CR2_CCDS           /*!< CCx DMA requests sent when update event occurs */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_LOCKLEVEL LOCKLEVEL
  * @{
  */
#define LL_TIM_LOCKLEVEL_OFF                   ((uint32_t)0x00000000) /*!< LOCK OFF - No bit is write protected */ 
#define LL_TIM_LOCKLEVEL_1                     TIM_BDTR_LOCK_0        /*!< LOCK Level 1 */         
#define LL_TIM_LOCKLEVEL_2                     TIM_BDTR_LOCK_1        /*!< LOCK Level 2 */          
#define LL_TIM_LOCKLEVEL_3                     TIM_BDTR_LOCK          /*!< LOCK Level 3 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CHANNEL CHANNEL
  * @{
  */
#define LL_TIM_CHANNEL_CH1                     TIM_CCER_CC1E /*!< Timer input/output channel 1 */
#define LL_TIM_CHANNEL_CH1N                    TIM_CCER_CC1NE /*!< Timer complementary output channel 1 */
#define LL_TIM_CHANNEL_CH2                     TIM_CCER_CC2E /*!< Timer input/output channel 2 */
#define LL_TIM_CHANNEL_CH2N                    TIM_CCER_CC2NE /*!< Timer complementary output channel 2 */
#define LL_TIM_CHANNEL_CH3                     TIM_CCER_CC3E /*!< Timer input/output channel 3 */
#define LL_TIM_CHANNEL_CH3N                    TIM_CCER_CC3NE /*!< Timer complementary output channel 3 */
#define LL_TIM_CHANNEL_CH4                     TIM_CCER_CC4E /*!< Timer input/output channel 4 */
#define LL_TIM_CHANNEL_CH5                     TIM_CCER_CC5E /*!< Timer output channel 5 */
#define LL_TIM_CHANNEL_CH6                     TIM_CCER_CC6E /*!< Timer output channel 6 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCMODE OCMODE
  * @{
  */
#define LL_TIM_OCMODE_FROZEN                   ((uint32_t)0x00000000)                                   /*!<The comparison between the output compare register TIMx_CCRy and the counter TIMx_CNT has no effect on the output channel level */ 
#define LL_TIM_OCMODE_ACTIVE                   TIM_CCMR1_OC1M_0                                         /*!<OCyREF is forced high on compare match*/ 
#define LL_TIM_OCMODE_INACTIVE                 TIM_CCMR1_OC1M_1                                         /*!<OCyREF is forced low on compare match*/ 
#define LL_TIM_OCMODE_TOGGLE                   (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0)                    /*!<OCyREF toggles on compare match*/ 
#define LL_TIM_OCMODE_FORCED_INACTIVE          (TIM_CCMR1_OC1M_2)                                       /*!<OCyREF is forced low*/ 
#define LL_TIM_OCMODE_FORCED_ACTIVE            (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0)                    /*!<OCyREF is forced high*/ 
#define LL_TIM_OCMODE_PWM1                     (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)                    /*!<In upcounting, channel y is active as long as TIMx_CNT<TIMx_CCRy else inactive.  In downcounting, channel y is inactive as long as TIMx_CNT>TIMx_CCRy else active.*/ 
#define LL_TIM_OCMODE_PWM2                     (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0) /*!<In upcounting, channel y is inactive as long as TIMx_CNT<TIMx_CCRy else active.  In downcounting, channel y is active as long as TIMx_CNT>TIMx_CCRy else inactive*/ 
#define LL_TIM_OCMODE_RETRIG_OPM1              TIM_CCMR1_OC1M_3                                         /*!<Retrigerrable OPM mode 1*/ 
#define LL_TIM_OCMODE_RETRIG_OPM2              (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_0)                    /*!<Retrigerrable OPM mode 2*/ 
#define LL_TIM_OCMODE_COMBINED_PWM1            (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_2)                    /*!<Combined PWM mode 1*/ 
#define LL_TIM_OCMODE_COMBINED_PWM2            (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2) /*!<Combined PWM mode 2*/ 
#define LL_TIM_OCMODE_ASSYMETRIC_PWM1          (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2) /*!<Asymmetric PWM mode 1*/ 
#define LL_TIM_OCMODE_ASSYMETRIC_PWM2          (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M)                      /*!<Asymmetric PWM mode 2*/ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCPOLARITY OCPOLARITY
  * @{
  */
#define LL_TIM_OCPOLARITY_HIGH                 ((uint32_t)0x00000000) /*!< OCx active high*/ 
#define LL_TIM_OCPOLARITY_LOW                  TIM_CCER_CC1P          /*!<OCxactive low*/ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCIDLESTATE OCIDLESTATE
  * @{
  */
#define LL_TIM_OCIDLESTATE_LOW                 ((uint32_t)0x00000000) /*!<OCx=0 (after a dead-time if OC is implemented) when MOE=0*/ 
#define LL_TIM_OCIDLESTATE_HIGH                TIM_CR2_OIS1        /*!<OCx=1 (after a dead-time if OC is implemented) when MOE=0*/ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_GROUPCH5 GROUPCH5
  * @{
  */
#define LL_TIM_GROUPCH5_NONE                   (uint32_t)0x00000000  /*!< No effect of OC5REF on OC1REFC, OC2REFC and OC3REFC */
#define LL_TIM_GROUPCH5_OC1REFC                (TIM_CCR5_GC5C1)      /*!< OC1REFC is the logical AND of OC1REFC and OC5REF */
#define LL_TIM_GROUPCH5_OC2REFC                (TIM_CCR5_GC5C2)      /*!< OC2REFC is the logical AND of OC2REFC and OC5REF */
#define LL_TIM_GROUPCH5_OC3REFC                (TIM_CCR5_GC5C3)      /*!< OC3REFC is the logical AND of OC3REFC and OC5REF */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ACTIVEINPUT ACTIVEINPUT
  * @{
  */
#define LL_TIM_ACTIVEINPUT_DIRECTTI            (TIM_CCMR1_CC1S_0 << 16) /*!< ICx is mapped on TIx */ 
#define LL_TIM_ACTIVEINPUT_INDIRECTTI          (TIM_CCMR1_CC1S_1 << 16) /*!< ICx is mapped on TIy */ 
#define LL_TIM_ACTIVEINPUT_TRC                 (TIM_CCMR1_CC1S << 16)   /*!< ICx is mapped on TRC */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ICPSC ICPSC
  * @{
  */
#define LL_TIM_ICPSC_DIV1                      ((uint32_t)0x00000000) /*!< No prescaler, capture is done each time an edge is detected on the capture input */ 
#define LL_TIM_ICPSC_DIV2                      (TIM_CCMR1_IC1PSC_0 << 16)    /*!< Capture is done once every 2 events */ 
#define LL_TIM_ICPSC_DIV4                      (TIM_CCMR1_IC1PSC_1 << 16)    /*!< Capture is done once every 4 events */ 
#define LL_TIM_ICPSC_DIV8                      (TIM_CCMR1_IC1PSC << 16)      /*!< Capture is done once every 8 events */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_IC_FILTER IC FILTER
  * @{
  */
#define LL_TIM_IC_FILTER_FDIV1                 ((uint32_t)0x00000000)                                    /*!< No filter, sampling is done at fDTS */ 
#define LL_TIM_IC_FILTER_FDIV1_N2              (TIM_CCMR1_IC1F_0 << 16)                                          /*!< fSAMPLING=fCK_INT, N=2 */ 
#define LL_TIM_IC_FILTER_FDIV1_N4              (TIM_CCMR1_IC1F_1 << 16)                                          /*!< fSAMPLING=fCK_INT, N=4 */ 
#define LL_TIM_IC_FILTER_FDIV1_N8              ((TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0) << 16)                     /*!< fSAMPLING=fCK_INT, N=8 */ 
#define LL_TIM_IC_FILTER_FDIV2_N6              (TIM_CCMR1_IC1F_2 << 16)                                          /*!< fSAMPLING=fDTS/2, N=6 */ 
#define LL_TIM_IC_FILTER_FDIV2_N8              ((TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_0) << 16)                     /*!< fSAMPLING=fDTS/2, N=8 */ 
#define LL_TIM_IC_FILTER_FDIV4_N6              ((TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1) << 16)                     /*!< fSAMPLING=fDTS/4, N=6 */ 
#define LL_TIM_IC_FILTER_FDIV4_N8              ((TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0) << 16)  /*!< fSAMPLING=fDTS/4, N=8 */ 
#define LL_TIM_IC_FILTER_FDIV8_N6              (TIM_CCMR1_IC1F_3 << 16)                                          /*!< fSAMPLING=fDTS/8, N=6 */ 
#define LL_TIM_IC_FILTER_FDIV8_N8              ((TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_0) << 16)                     /*!< fSAMPLING=fDTS/8, N=8 */ 
#define LL_TIM_IC_FILTER_FDIV16_N5             ((TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_1) << 16)                     /*!< fSAMPLING=fDTS/16, N=5 */ 
#define LL_TIM_IC_FILTER_FDIV16_N6             ((TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0) << 16)  /*!< fSAMPLING=fDTS/16, N=6 */ 
#define LL_TIM_IC_FILTER_FDIV16_N8             ((TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2) << 16)                     /*!< fSAMPLING=fDTS/16, N=8 */ 
#define LL_TIM_IC_FILTER_FDIV32_N5             ((TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_0) << 16)  /*!< fSAMPLING=fDTS/32, N=5 */ 
#define LL_TIM_IC_FILTER_FDIV32_N6             ((TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1) << 16)  /*!< fSAMPLING=fDTS/32, N=6 */ 
#define LL_TIM_IC_FILTER_FDIV32_N8             (TIM_CCMR1_IC1F << 16)                                            /*!< fSAMPLING=fDTS/32, N=8 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_IC_POLARITY IC POLARITY
  * @{
  */
#define LL_TIM_IC_POLARITY_RISING              ((uint32_t)0x00000000)           /*!< The circuit is sensitive to TIxFP1 rising edge, TIxFP1 is not inverted */ 
#define LL_TIM_IC_POLARITY_FALLING             TIM_CCER_CC1P                    /*!< The circuit is sensitive to TIxFP1 falling edge, TIxFP1 is inverted */ 
#define LL_TIM_IC_POLARITY_BOTHEDGE            (TIM_CCER_CC1P | TIM_CCER_CC1NP) /*!< The circuit is sensitive to both TIxFP1 rising and falling edges, TIxFP1 is not inverted */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CLOCKSOURCE CLOCKSOURCE
  * @{
  */
#define LL_TIM_CLOCKSOURCE_INTERNAL            ((uint32_t)0x00000000)                               /*!< The timer is clocked by the internal clock provided from the RCC */
#define LL_TIM_CLOCKSOURCE_EXT_MODE1           (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0 ) /*!< Counter counts at each rising or falling edge on a selected inpu t*/
#define LL_TIM_CLOCKSOURCE_EXT_MODE2           TIM_SMCR_ECE                                        /*!< Counter counts at each rising or falling edge on the external trigger input ETR */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ENCODERMODE_X2 ENCODERMODE X2
  * @{
  */
#define LL_TIM_ENCODERMODE_X2_TI1              TIM_SMCR_SMS_0                    /*!< Encoder mode 1 - Counter counts up/down on TI2FP2 edge depending on TI1FP1 level */ 
#define LL_TIM_ENCODERMODE_X2_TI2              TIM_SMCR_SMS_1                    /*!< Encoder mode 2 - Counter counts up/down on TI1FP1 edge depending on TI2FP2 level */ 
#define LL_TIM_ENCODERMODE_X4_TI12             (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0) /*!< Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges                                                                                                                                                                   depending on the level of the other input l */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TRGO TRGO
  * @{
  */
#define LL_TIM_TRGO_RESET                      ((uint32_t)0x00000000)                          /*!< UG bit from the TIMx_EGR register is used as trigger output */ 
#define LL_TIM_TRGO_ENABLE                     TIM_CR2_MMS_0                                   /*!< Counter Enable signal (CNT_EN) is used as trigger output */ 
#define LL_TIM_TRGO_UPDATE                     TIM_CR2_MMS_1                                   /*!< Update event is used as trigger output */ 
#define LL_TIM_TRGO_CC1IF                      (TIM_CR2_MMS_1 | TIM_CR2_MMS_0)                 /*!< CC1 capture or a compare match is usd as trigger output */ 
#define LL_TIM_TRGO_OC1REF                     TIM_CR2_MMS_2                                   /*!< OC1REF signal is used as trigger output */ 
#define LL_TIM_TRGO_OC2REF                     (TIM_CR2_MMS_2 | TIM_CR2_MMS_0)                 /*!< OC2REF signal is used as trigger output */ 
#define LL_TIM_TRGO_OC3REF                     (TIM_CR2_MMS_2 | TIM_CR2_MMS_1)                 /*!< OC3REF signal is used as trigger output */ 
#define LL_TIM_TRGO_OC4REF                     (TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0) /*!< OC4REF signal is used as trigger output */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TRGO2 TRGO2
  * @{
  */
#define LL_TIM_TRGO2_RESET                     ((uint32_t)0x00000000)                                              /*!< UG bit from the TIMx_EGR register is used as trigger output 2 */ 
#define LL_TIM_TRGO2_ENABLE                    TIM_CR2_MMS2_0                                                      /*!< Counter Enable signal (CNT_EN) is used as trigger output 2 */ 
#define LL_TIM_TRGO2_UPDATE                    TIM_CR2_MMS2_1                                                      /*!< Update event is used as trigger output 2 */ 
#define LL_TIM_TRGO2_CC1F                      (TIM_CR2_MMS2_1 | TIM_CR2_MMS2_0)                                   /*!< CC1 capture or a compare match is used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC1                       TIM_CR2_MMS2_2                                                      /*!< OC1REF signal is used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC2                       (TIM_CR2_MMS2_2 | TIM_CR2_MMS2_0)                                   /*!< OC2REF signal is used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC3                       (TIM_CR2_MMS2_2 | TIM_CR2_MMS2_1)                                   /*!< OC3REF signal is used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC4                       (TIM_CR2_MMS2_2 | TIM_CR2_MMS2_1 | TIM_CR2_MMS2_0)                  /*!< OC4REF signal is used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC5                       TIM_CR2_MMS2_3                                                      /*!< OC5REF signal is used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC6                       (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_0)                                   /*!< OC6REF signal is used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC4_RISINGFALLING         (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_1)                                   /*!< OC4REF rising or falling edges are used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC6_RISINGFALLING         (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_1 | TIM_CR2_MMS2_0)                  /*!< OC6REF rising or falling edges are used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC4_RISING_OC6_RISING     (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2)                                   /*!< OC4REF or OC6REF rising edges are used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC4_RISING_OC6_FALLING    (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2 | TIM_CR2_MMS2_0)                  /*!< OC4REF rising or OC6REF falling edges are used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC5_RISING_OC6_RISING     (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2 |TIM_CR2_MMS2_1)                   /*!< OC5REF or OC6REF rising edges are used as trigger output 2 */ 
#define LL_TIM_TRGO2_OC5_RISING_OC6_FALLING    (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2 | TIM_CR2_MMS2_1 | TIM_CR2_MMS2_0) /*!< OC5REF rising or OC6REF falling edges are used as trigger output 2 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_SLAVEMODE SLAVEMODE
  * @{
  */
#define LL_TIM_SLAVEMODE_DISABLED              ((uint32_t)0x00000000)              /*!< Slave mode disabled */ 
#define LL_TIM_SLAVEMODE_RESET                 TIM_SMCR_SMS_2                      /*!< Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter */ 
#define LL_TIM_SLAVEMODE_GATED                 (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0)   /*!< Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high */ 
#define LL_TIM_SLAVEMODE_TRIGGER               (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1)   /*!< Trigger Mode - The counter starts at a rising edge of the trigger TRGI */ 
#define LL_TIM_SLAVEMODE_COMBINED_RESETTRIGGER TIM_SMCR_SMS_3                      /*!< Combined reset + trigger mode - Rising edge of the selected trigger input (TRGI)  reinitializes the counter, generates an update of the registers and starts the counter */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TS TS
  * @{
  */
#define LL_TIM_TS_ITR0                         ((uint32_t)0x00000000)          /*!< Internal Trigger 0 (ITR0) is used as trigger input */ 
#define LL_TIM_TS_ITR1                         TIM_SMCR_TS_0                   /*!< Internal Trigger 1 (ITR1) is used as trigger input */ 
#define LL_TIM_TS_ITR2                         TIM_SMCR_TS_1                   /*!< Internal Trigger 2 (ITR2) is used as trigger input */ 
#define LL_TIM_TS_ITR3                         (TIM_SMCR_TS_0 | TIM_SMCR_TS_1) /*!< Internal Trigger 3 (ITR3) is used as trigger input */ 
#define LL_TIM_TS_TI1F_ED                      TIM_SMCR_TS_2                   /*!< TI1 Edge Detector (TI1F_ED) is used as trigger input */ 
#define LL_TIM_TS_TI1FP1                       (TIM_SMCR_TS_2 | TIM_SMCR_TS_0) /*!< Filtered Timer Input 1 (TI1FP1) is used as trigger input */ 
#define LL_TIM_TS_TI2FP2                       (TIM_SMCR_TS_2 | TIM_SMCR_TS_1) /*!< Filtered Timer Input 2 (TI12P2) is used as trigger input */ 
#define LL_TIM_TS_ETRF                         TIM_SMCR_TS                     /*!< Filtered external Trigger (ETRF) is used as trigger input */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ETR_POLARITY ETR POLARITY
  * @{
  */
#define LL_TIM_ETR_POLARITY_NONINVERTED        ((uint32_t)0x00000000) /*!< ETR is non-inverted, active at high level or rising edge */ 
#define LL_TIM_ETR_POLARITY_INVERTED           TIM_SMCR_ETP           /*!< ETR is inverted, active at low level or falling edge */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ETR_PRESCALER ETR PRESCALER
  * @{
  */
#define LL_TIM_ETR_PRESCALER_DIV1              ((uint32_t)0x00000000) /*!< ETR prescaler OFF */ 
#define LL_TIM_ETR_PRESCALER_DIV2              TIM_SMCR_ETPS_0        /*!< ETR frequency is divided by 2 */ 
#define LL_TIM_ETR_PRESCALER_DIV4              TIM_SMCR_ETPS_1        /*!< ETR frequency is divided by 4 */ 
#define LL_TIM_ETR_PRESCALER_DIV8              TIM_SMCR_ETPS          /*!< ETR frequency is divided by 8 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ETR_FILTER ETR FILTER
  * @{
  */
#define LL_TIM_ETR_FILTER_FDIV1                ((uint32_t)0x00000000)                               /*!< No filter, sampling is done at fDTS */ 
#define LL_TIM_ETR_FILTER_FDIV1_N2             TIM_SMCR_ETF_0                                       /*!< fSAMPLING=fCK_INT, N=2 */ 
#define LL_TIM_ETR_FILTER_FDIV1_N4             TIM_SMCR_ETF_1                                       /*!< fSAMPLING=fCK_INT, N=4 */ 
#define LL_TIM_ETR_FILTER_FDIV1_N8             (TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0)                    /*!< fSAMPLING=fCK_INT, N=8 */ 
#define LL_TIM_ETR_FILTER_FDIV2_N6             TIM_SMCR_ETF_2                                       /*!< fSAMPLING=fDTS/2, N=6 */ 
#define LL_TIM_ETR_FILTER_FDIV2_N8             (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_0)                    /*!< fSAMPLING=fDTS/2, N=8 */ 
#define LL_TIM_ETR_FILTER_FDIV4_N6             (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1 )                   /*!< fSAMPLING=fDTS/4, N=6 */ 
#define LL_TIM_ETR_FILTER_FDIV4_N8             (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0)   /*!< fSAMPLING=fDTS/4, N=8 */ 
#define LL_TIM_ETR_FILTER_FDIV8_N6             TIM_SMCR_ETF_3                                       /*!< fSAMPLING=fDTS/8, N=8 */ 
#define LL_TIM_ETR_FILTER_FDIV8_N8             (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_0)                    /*!< fSAMPLING=fDTS/16, N=5 */ 
#define LL_TIM_ETR_FILTER_FDIV16_N5            (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_1 )                   /*!< fSAMPLING=fDTS/16, N=6 */ 
#define LL_TIM_ETR_FILTER_FDIV16_N6            (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0)   /*!< fSAMPLING=fDTS/16, N=8 */ 
#define LL_TIM_ETR_FILTER_FDIV16_N8            (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 )                   /*!< fSAMPLING=fDTS/16, N=5 */ 
#define LL_TIM_ETR_FILTER_FDIV32_N5            (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2  | TIM_SMCR_ETF_0)  /*!< fSAMPLING=fDTS/32, N=5 */ 
#define LL_TIM_ETR_FILTER_FDIV32_N6            (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2  | TIM_SMCR_ETF_1)  /*!< fSAMPLING=fDTS/32, N=6 */ 
#define LL_TIM_ETR_FILTER_FDIV32_N8            TIM_SMCR_ETF                                         /*!< fSAMPLING=fDTS/32, N=8 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ETRSOURCE ETRSOURCE
  * @{
  */
#define LL_TIM_ETRSOURCE_LEGACY                ((uint32_t)(0x00000000)) /*!< ETR legacy mode */ 
#define LL_TIM_ETRSOURCE_COMP1                 TIM1_OR2_ETRSEL_0        /*!< COMP1 output connected to ETR input */ 
#define LL_TIM_ETRSOURCE_COMP2                 TIM1_OR2_ETRSEL_1        /*!< COMP2 output connected to ETR input */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK_POLARITY BREAK POLARITY
  * @{
  */
#define LL_TIM_BREAK_POLARITY_LOW              ((uint32_t)0x00000000)   /*!< Break input BRK is active low */ 
#define LL_TIM_BREAK_POLARITY_HIGH             TIM_BDTR_BKP              /*!< Break input BRK is active high */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK_FILTER BREAK FILTER
  * @{
  */
#define LL_TIM_BREAK_FILTER_FDIV1              ((uint32_t)0x00000000)   /*!< No filter, BRK acts asynchronously */ 
#define LL_TIM_BREAK_FILTER_FDIV1_N2           ((uint32_t)0x00010000)   /*!< fSAMPLING=fCK_INT, N=2 */ 
#define LL_TIM_BREAK_FILTER_FDIV1_N4           ((uint32_t)0x00020000)   /*!< fSAMPLING=fCK_INT, N=4 */ 
#define LL_TIM_BREAK_FILTER_FDIV1_N8           ((uint32_t)0x00030000)   /*!< fSAMPLING=fCK_INT, N=8 */ 
#define LL_TIM_BREAK_FILTER_FDIV2_N6           ((uint32_t)0x00040000)   /*!< fSAMPLING=fDTS/2, N=6 */ 
#define LL_TIM_BREAK_FILTER_FDIV2_N8           ((uint32_t)0x00050000)   /*!< fSAMPLING=fDTS/2, N=8 */ 
#define LL_TIM_BREAK_FILTER_FDIV4_N6           ((uint32_t)0x00060000)   /*!< fSAMPLING=fDTS/4, N=6 */ 
#define LL_TIM_BREAK_FILTER_FDIV4_N8           ((uint32_t)0x00070000)   /*!< fSAMPLING=fDTS/4, N=8 */ 
#define LL_TIM_BREAK_FILTER_FDIV8_N6           ((uint32_t)0x00080000)   /*!< fSAMPLING=fDTS/8, N=6 */ 
#define LL_TIM_BREAK_FILTER_FDIV8_N8           ((uint32_t)0x00090000)   /*!< fSAMPLING=fDTS/8, N=8 */ 
#define LL_TIM_BREAK_FILTER_FDIV16_N5          ((uint32_t)0x000A0000)   /*!< fSAMPLING=fDTS/16, N=5 */ 
#define LL_TIM_BREAK_FILTER_FDIV16_N6          ((uint32_t)0x000B0000)   /*!< fSAMPLING=fDTS/16, N=6 */ 
#define LL_TIM_BREAK_FILTER_FDIV16_N8          ((uint32_t)0x000C0000)   /*!< fSAMPLING=fDTS/16, N=8 */ 
#define LL_TIM_BREAK_FILTER_FDIV32_N5          ((uint32_t)0x000D0000)   /*!< fSAMPLING=fDTS/32, N=5 */ 
#define LL_TIM_BREAK_FILTER_FDIV32_N6          ((uint32_t)0x000E0000)   /*!< fSAMPLING=fDTS/32, N=6 */ 
#define LL_TIM_BREAK_FILTER_FDIV32_N8          ((uint32_t)0x000F0000)   /*!< fSAMPLING=fDTS/32, N=8 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK2_POLARITY BREAK2 POLARITY
  * @{
  */
#define LL_TIM_BREAK2_POLARITY_LOW             ((uint32_t)0x00000000) /*!< Break input BRK2 is active low */ 
#define LL_TIM_BREAK2_POLARITY_HIGH            TIM_BDTR_BK2P          /*!< Break input BRK2 is active high */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK2_FILTER BREAK2 FILTER
  * @{
  */
#define LL_TIM_BREAK2_FILTER_FDIV1             ((uint32_t)0x00000000)   /*!< No filter, BRK acts asynchronously */ 
#define LL_TIM_BREAK2_FILTER_FDIV1_N2          ((uint32_t)0x00100000)   /*!< fSAMPLING=fCK_INT, N=2 */ 
#define LL_TIM_BREAK2_FILTER_FDIV1_N4          ((uint32_t)0x00200000)   /*!< fSAMPLING=fCK_INT, N=4 */ 
#define LL_TIM_BREAK2_FILTER_FDIV1_N8          ((uint32_t)0x00300000)   /*!< fSAMPLING=fCK_INT, N=8 */ 
#define LL_TIM_BREAK2_FILTER_FDIV2_N6          ((uint32_t)0x00400000)   /*!< fSAMPLING=fDTS/2, N=6 */ 
#define LL_TIM_BREAK2_FILTER_FDIV2_N8          ((uint32_t)0x00500000)   /*!< fSAMPLING=fDTS/2, N=8 */ 
#define LL_TIM_BREAK2_FILTER_FDIV4_N6          ((uint32_t)0x00600000)   /*!< fSAMPLING=fDTS/4, N=6 */ 
#define LL_TIM_BREAK2_FILTER_FDIV4_N8          ((uint32_t)0x00700000)   /*!< fSAMPLING=fDTS/4, N=8 */ 
#define LL_TIM_BREAK2_FILTER_FDIV8_N6          ((uint32_t)0x00800000)   /*!< fSAMPLING=fDTS/8, N=6 */ 
#define LL_TIM_BREAK2_FILTER_FDIV8_N8          ((uint32_t)0x00900000)   /*!< fSAMPLING=fDTS/8, N=8 */ 
#define LL_TIM_BREAK2_FILTER_FDIV16_N5         ((uint32_t)0x00A00000)   /*!< fSAMPLING=fDTS/16, N=5 */ 
#define LL_TIM_BREAK2_FILTER_FDIV16_N6         ((uint32_t)0x00B00000)   /*!< fSAMPLING=fDTS/16, N=6 */ 
#define LL_TIM_BREAK2_FILTER_FDIV16_N8         ((uint32_t)0x00C00000)   /*!< fSAMPLING=fDTS/16, N=8 */ 
#define LL_TIM_BREAK2_FILTER_FDIV32_N5         ((uint32_t)0x00D00000)   /*!< fSAMPLING=fDTS/32, N=5 */ 
#define LL_TIM_BREAK2_FILTER_FDIV32_N6         ((uint32_t)0x00E00000)   /*!< fSAMPLING=fDTS/32, N=6 */ 
#define LL_TIM_BREAK2_FILTER_FDIV32_N8         ((uint32_t)0x00F00000)   /*!< fSAMPLING=fDTS/32, N=8 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OSSI OSSI
  * @{
  */
#define LL_TIM_OSSI_DISABLE                    ((uint32_t)0x00000000) /*!< When inactive, OCx/OCxN outputs are disabled */ 
#define LL_TIM_OSSI_ENABLE                     TIM_BDTR_OSSI          /*!< When inactive, OxC/OCxN outputs are first forced with their inactive level then forced to their idle level after the deadtime */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OSSR OSSR
  * @{
  */
#define LL_TIM_OSSR_DISABLE                    ((uint32_t)0x00000000) /*!< When inactive, OCx/OCxN outputs are disabled */ 
#define LL_TIM_OSSR_ENABLE                     TIM_BDTR_OSSR          /*!< When inactive, OC/OCN outputs are enabled with their inactive level as soon as CCxE=1 or CCxNE=1 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK_INPUT BREAK INPUT
  * @{
  */
#define LL_TIM_BREAK_INPUT_BKIN                ((uint32_t)0x00000000)  /*!< TIMx_BKIN input */
#define LL_TIM_BREAK_INPUT_BKIN2               ((uint32_t)0x00000004)  /*!< TIMx_BKIN2 input */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BKIN_SOURCE BKIN SOURCE
  * @{
  */
#define LL_TIM_BKIN_SOURCE_BKIN                TIM1_OR2_BKINE     /*!< BKIN input from AF controller */
#define LL_TIM_BKIN_SOURCE_BKCOMP1             TIM1_OR2_BKCMP1E   /*!< internal signal: COMP1 output */
#define LL_TIM_BKIN_SOURCE_BKCOMP2             TIM1_OR2_BKCMP2E   /*!< internal signal: COMP2 output */
#define LL_TIM_BKIN_SOURCE_DFBK                TIM1_OR2_BKDFBK0E  /*!< internal signal: DFSDM break output */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BKIN_POLARITY BKIN POLARITY
  * @{
  */
#define LL_TIM_BKIN_POLARITY_LOW               ((uint32_t)0x00000000) /*!< BRK BKIN input is active low */ 
#define LL_TIM_BKIN_POLARITY_HIGH              TIM1_OR2_BKINP         /*!< BRK BKIN input is active high */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_DMABURST_BASEADDR DMABURST BASEADDR
  * @{
  */
#define LL_TIM_DMABURST_BASEADDR_CR1           ((uint32_t)0x00000000)                                                                                                                               /*!< TIMx_CR1 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CR2           TIM_DCR_DBA_0                                                    /*!< TIMx_CR2 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_SMCR          TIM_DCR_DBA_1                                                    /*!< TIMx_SMCR register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_DIER          (TIM_DCR_DBA_1 |  TIM_DCR_DBA_0)                                 /*!< TIMx_DIER register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_SR            TIM_DCR_DBA_2                                                    /*!< TIMx_SR register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_EGR           (TIM_DCR_DBA_2 | TIM_DCR_DBA_0)                                  /*!< TIMx_EGR register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCMR1         (TIM_DCR_DBA_2 | TIM_DCR_DBA_1)                                  /*!< TIMx_CCMR1 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCMR2         (TIM_DCR_DBA_2 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0)                  /*!< TIMx_CCMR2 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCER          TIM_DCR_DBA_3                                                    /*!< TIMx_CCER register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CNT           (TIM_DCR_DBA_3 | TIM_DCR_DBA_0)                                  /*!< TIMx_CNT register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_PSC           (TIM_DCR_DBA_3 | TIM_DCR_DBA_1)                                  /*!< TIMx_PSC register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_ARR           (TIM_DCR_DBA_3 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0)                  /*!< TIMx_ARR register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_RCR           (TIM_DCR_DBA_3 | TIM_DCR_DBA_2)                                  /*!< TIMx_RCR register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCR1          (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_0)                  /*!< TIMx_CCR1 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCR2          (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1)                  /*!< TIMx_CCR2 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCR3          (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0)  /*!< TIMx_CCR3 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCR4          TIM_DCR_DBA_4                                                    /*!< TIMx_CCR4 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_BDTR          (TIM_DCR_DBA_4 | TIM_DCR_DBA_0)                                  /*!< TIMx_BDTR register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCMR3         (TIM_DCR_DBA_4 | TIM_DCR_DBA_1)                                  /*!< TIMx_CCMR3 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCR5          (TIM_DCR_DBA_4 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0)                  /*!< TIMx_CCR5 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_CCR6          (TIM_DCR_DBA_4 | TIM_DCR_DBA_2)                                  /*!< TIMx_CCR6 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_OR1           (TIM_DCR_DBA_4 | TIM_DCR_DBA_2 | TIM_DCR_DBA_0)                  /*!< TIMx_OR1 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_OR2           (TIM_DCR_DBA_4 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1)                  /*!< TIMx_OR2 register is the DMA base address for DMA burst */ 
#define LL_TIM_DMABURST_BASEADDR_OR3           (TIM_DCR_DBA_4 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0) /*!< TIMx_OR3 register is the DMA base address for DMA burst */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_DMABURST_LENGTH DMABURST LENGTH
  * @{
  */
#define LL_TIM_DMABURST_LENGTH_1TRANSFER       ((uint32_t)0x00000000)                                          /*!< Transfer is done to 1 register starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_2TRANSFERS      TIM_DCR_DBL_0                                                   /*!< Transfer is done to 2 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_3TRANSFERS      TIM_DCR_DBL_1                                                   /*!< Transfer is done to 3 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_4TRANSFERS      (TIM_DCR_DBL_1 |  TIM_DCR_DBL_0)                                /*!< Transfer is done to 4 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_5TRANSFERS      TIM_DCR_DBL_2                                                   /*!< Transfer is done to 5 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_6TRANSFERS      (TIM_DCR_DBL_2 | TIM_DCR_DBL_0)                                 /*!< Transfer is done to 6 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_7TRANSFERS      (TIM_DCR_DBL_2 | TIM_DCR_DBL_1)                                 /*!< Transfer is done to 7 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_8TRANSFERS      (TIM_DCR_DBL_2 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0)                 /*!< Transfer is done to 1 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_9TRANSFERS      TIM_DCR_DBL_3                                                   /*!< Transfer is done to 9 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_10TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_0)                                 /*!< Transfer is done to 10 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_11TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_1)                                 /*!< Transfer is done to 11 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_12TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0)                 /*!< Transfer is done to 12 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_13TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_2)                                 /*!< Transfer is done to 13 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_14TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_0)                 /*!< Transfer is done to 14 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_15TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_1)                 /*!< Transfer is done to 15 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_16TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0) /*!< Transfer is done to 16 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_17TRANSFERS     TIM_DCR_DBL_4                                                   /*!< Transfer is done to 17 registers starting from the DMA busrt base address */ 
#define LL_TIM_DMABURST_LENGTH_18TRANSFERS     (TIM_DCR_DBL_4 |  TIM_DCR_DBL_0)                                /*!< Transfer is done to 18 registers starting from the DMA busrt base address */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM1_ETR_ADC1_RMP  TIM1 ETR ADC1 RMP
* @{
*/
#define LL_TIM_TIM1_ETR_ADC1_RMP_NC   ((uint32_t)0x00000000 | TIM1_OR1_RMP_MASK)    /*!< TIM1_ETR is not connected to ADC1 analog watchdog x */  
#define LL_TIM_TIM1_ETR_ADC1_RMP_AWD1 (TIM1_OR1_ETR_ADC1_RMP_0 | TIM1_OR1_RMP_MASK) /*!< TIM1_ETR is connected to ADC1 analog watchdog 1 */  
#define LL_TIM_TIM1_ETR_ADC1_RMP_AWD2 (TIM1_OR1_ETR_ADC1_RMP_1 | TIM1_OR1_RMP_MASK)  /*!< TIM1_ETR is connected to ADC1 analog watchdog 2 */  
#define LL_TIM_TIM1_ETR_ADC1_RMP_AWD3 (TIM1_OR1_ETR_ADC1_RMP | TIM1_OR1_RMP_MASK)   /*!< TIM1_ETR is connected to ADC1 analog watchdog 3 */ 
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM1_ETR_ADC3_RMP  TIM1 ETR ADC3 RMP
* @{
*/
#define LL_TIM_TIM1_ETR_ADC3_RMP_NC   ((uint32_t)0x00000000 | TIM1_OR1_RMP_MASK)    /*!< TIM1_ETR is not connected to ADC3 analog watchdog  x*/  
#define LL_TIM_TIM1_ETR_ADC3_RMP_AWD1 (TIM1_OR1_ETR_ADC3_RMP_0 | TIM1_OR1_RMP_MASK) /*!< TIM1_ETR is connected to ADC3 analog watchdog 1 */  
#define LL_TIM_TIM1_ETR_ADC3_RMP_AWD2 (TIM1_OR1_ETR_ADC3_RMP_1 | TIM1_OR1_RMP_MASK) /*!< TIM1_ETR is connected to ADC3 analog watchdog 2 */  
#define LL_TIM_TIM1_ETR_ADC3_RMP_AWD3 (TIM1_OR1_ETR_ADC3_RMP | TIM1_OR1_RMP_MASK)   /*!< TIM1_ETR is connected to ADC3 analog watchdog 3 */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM1_TI1_RMP  TIM1 TI1 RMP
* @{
*/
#define LL_TIM_TIM1_TI1_RMP_GPIO  ((uint32_t)0x00000000 | TIM1_OR1_RMP_MASK) /*!< TIM1 input capture 1 is connected to GPIO */  
#define LL_TIM_TIM1_TI1_RMP_COMP1 (TIM1_OR1_TI1_RMP | TIM1_OR1_RMP_MASK)     /*!< TIM1 input capture 1 is connected to COMP1 output */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM2_ITR1_RMP_TIM8  TIM2 ITR1 RMP TIM8
* @{
*/
#define LL_TIM_TIM2_ITR1_RMP_TIM8_TRGO  ((uint32_t)0x00000000 | TIM2_OR1_RMP_MASK) /*!< TIM2_ITR1 is connected to TIM8_TRGO */  
#define LL_TIM_TIM2_ITR1_RMP_OTG_FS_SOF (TIM2_OR1_ITR1_RMP | TIM2_OR1_RMP_MASK)   /*!< TIM2_ITR1 is connected to OTG_FS SOF */  
#define LL_TIM_TIM2_ETR_RMP_GPIO ((uint32_t)0x00000000 | TIM2_OR1_RMP_MASK) /*!< TIM2_ETR is connected to GPIO */  
#define LL_TIM_TIM2_ETR_RMP_LSE  (TIM2_OR1_ETR1_RMP | TIM2_OR1_RMP_MASK)    /*!< TIM2_ETR is connected to LSE  */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM2_TI4_RMP  TIM2 TI4 RMP
* @{
*/
#define LL_TIM_TIM2_TI4_RMP_GPIO        ((uint32_t)0x00000000 | TIM2_OR1_RMP_MASK) /*!< TIM2 input capture 4 is connected to GPIO */  
#define LL_TIM_TIM2_TI4_RMP_COMP1       (TIM2_OR1_TI4_RMP_0 | TIM2_OR1_RMP_MASK)   /*!< TIM2 input capture 4 is connected to COMP1_OUT */  
#define LL_TIM_TIM2_TI4_RMP_COMP2       (TIM2_OR1_TI4_RMP_1 | TIM2_OR1_RMP_MASK)   /*!< TIM2 input capture 4 is connected to COMP2_OUT */  
#define LL_TIM_TIM2_TI4_RMP_COMP1_COMP2 (TIM2_OR1_TI4_RMP | TIM2_OR1_RMP_MASK)     /*!< TIM2 input capture 4 is connected to logical OR between COMP1_OUT and COMP2_OUT */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM3_TI1_RMP  TIM3 TI1 RMP
* @{
*/
#define LL_TIM_TIM3_TI1_RMP_GPIO         ((uint32_t)0x00000000 | TIM3_OR1_RMP_MASK) /*!< TIM3 input capture 1 is connected to GPIO */  
#define LL_TIM_TIM3_TI1_RMP_COMP1        (TIM3_OR1_TI1_RMP_0 | TIM3_OR1_RMP_MASK)   /*!< TIM3 input capture 1 is connected to COMP1_OUT */  
#define LL_TIM_TIM3_TI1_RMP_COMP2        (TIM3_OR1_TI1_RMP_1 | TIM3_OR1_RMP_MASK)   /*!< TIM3 input capture 1 is connected to COMP2_OUT */  
#define LL_TIM_TIM3_TI1_RMP_COMP1_COMP2  (TIM3_OR1_TI1_RMP | TIM3_OR1_RMP_MASK)     /*!< TIM3 input capture 1 is connected to logical OR between COMP1_OUT and COMP2_OUT */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM8_ETR_ADC2_RMP  TIM8 ETR ADC2 RMP
* @{
*/
#define LL_TIM_TIM8_ETR_ADC2_RMP_NC   ((uint32_t)0x00000000 | TIM8_OR1_RMP_MASK)    /*!< TIM8_ETR is not connected to ADC2 analog watchdog x */  
#define LL_TIM_TIM8_ETR_ADC2_RMP_AWD1 (TIM8_OR1_ETR_ADC2_RMP_0 | TIM8_OR1_RMP_MASK) /*!< TIM8_ETR is connected to ADC2 analog watchdog */  
#define LL_TIM_TIM8_ETR_ADC2_RMP_AWD2 (TIM8_OR1_ETR_ADC2_RMP_1 | TIM8_OR1_RMP_MASK) /*!< TIM8_ETR is connected to ADC2 analog watchdog 2 */  
#define LL_TIM_TIM8_ETR_ADC2_RMP_AWD3 (TIM8_OR1_ETR_ADC2_RMP | TIM8_OR1_RMP_MASK)   /*!< TIM8_ETR is connected to ADC2 analog watchdog 3 */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM8_ETR_ADC3_RMP  TIM8 ETR ADC3 RMP
* @{
*/
#define LL_TIM_TIM8_ETR_ADC3_RMP_NC   ((uint32_t)0x00000000 | TIM8_OR1_RMP_MASK)    /*!< TIM8_ETR is not connected to ADC3 analog watchdog x */  
#define LL_TIM_TIM8_ETR_ADC3_RMP_AWD1 (TIM8_OR1_ETR_ADC3_RMP_0 | TIM8_OR1_RMP_MASK) /*!< TIM8_ETR is connected to ADC3 analog watchdog 1 */  
#define LL_TIM_TIM8_ETR_ADC3_RMP_AWD2 (TIM8_OR1_ETR_ADC3_RMP_1 | TIM8_OR1_RMP_MASK) /*!< TIM8_ETR is connected to ADC3 analog watchdog 2 */  
#define LL_TIM_TIM8_ETR_ADC3_RMP_AWD3 (TIM8_OR1_ETR_ADC3_RMP | TIM8_OR1_RMP_MASK)   /*!< TIM8_ETR is connected to ADC3 analog watchdog 3 */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM8_TI1_RMP  TIM8 TI1 RMP
* @{
*/
#define LL_TIM_TIM8_TI1_RMP_GPIO  ((uint32_t)0x00000000 | TIM8_OR1_RMP_MASK) /*!< TIM8 input capture 1 is connected to GPIO */  
#define LL_TIM_TIM8_TI1_RMP_COMP2 (TIM8_OR1_TI1_RMP | TIM8_OR1_RMP_MASK)     /*!< TIM8 input capture 1 is connected to COMP2 output */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM15_TI1_RMP  TIM15 TI1 RMP
* @{
*/
#define LL_TIM_TIM15_TI1_RMP_GPIO ((uint32_t)0x00000000 | TIM15_OR1_RMP_MASK) /*!< TIM15 input capture 1 is connected to GPIO */  
#define LL_TIM_TIM15_TI1_RMP_LSE  (TIM15_OR1_TI1_RMP | TIM15_OR1_RMP_MASK)    /*!< TIM15 input capture 1 is connected to LSE */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM15_ENCODERMODE  TIM15 ENCODERMODE
* @{
*/
#define LL_TIM_TIM15_ENCODERMODE_NOREDIRECTION ((uint32_t)0x00000000 | TIM15_OR1_RMP_MASK)     /*!< No redirection*/  
#define LL_TIM_TIM15_ENCODERMODE_TIM2          (TIM15_OR1_ENCODER_MODE_0 | TIM15_OR1_RMP_MASK) /*!< TIM2 IC1 and TIM2 IC2 are connected to TIM15 IC1 and TIM15 IC2 respectively */  
#define LL_TIM_TIM15_ENCODERMODE_TIM3          (TIM15_OR1_ENCODER_MODE_1 | TIM15_OR1_RMP_MASK) /*!< TIM3 IC1 and TIM3 IC2 are connected to TIM15 IC1 and TIM15 IC2 respectivel y*/  
#define LL_TIM_TIM15_ENCODERMODE_TIM4          (TIM15_OR1_ENCODER_MODE | TIM15_OR1_RMP_MASK)   /*!< TIM4 IC1 and TIM4 IC2 are connected to TIM15 IC1 and TIM15 IC2 respectively */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM16_TI1_RMP  TIM16 TI1 RMP
* @{
*/
#define LL_TIM_TIM16_TI1_RMP_GPIO ((uint32_t)0x00000000 | TIM16_OR1_RMP_MASK) /*!< TIM16 input capture 1 is connected to GPIO */  
#define LL_TIM_TIM16_TI1_RMP_LSI  (TIM16_OR1_TI1_RMP_0 | TIM16_OR1_RMP_MASK)  /*!< TIM16 input capture 1 is connected to LSI */  
#define LL_TIM_TIM16_TI1_RMP_LSE  (TIM16_OR1_TI1_RMP_1 | TIM16_OR1_RMP_MASK)  /*!< TIM16 input capture 1 is connected to LSE */  
#define LL_TIM_TIM16_TI1_RMP_RTC  (TIM16_OR1_TI1_RMP | TIM16_OR1_RMP_MASK)    /*!< TIM16 input capture 1 is connected to RTC wakeup iterrupt */  
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM17_TI1_RMP  TIM17 TI1 RMP
* @{
*/
#define LL_TIM_TIM17_TI1_RMP_GPIO   ((uint32_t)0x00000000 | TIM17_OR1_RMP_MASK) /*!< TIM17 input capture 1 is connected to GPIO */  
#define LL_TIM_TIM17_TI1_RMP_MSI    (TIM17_OR1_TI1_RMP_0 | TIM17_OR1_RMP_MASK)  /*!< TIM17 input capture 1 is connected to MSI */  
#define LL_TIM_TIM17_TI1_RMP_HSE_32 (TIM17_OR1_TI1_RMP_1 | TIM17_OR1_RMP_MASK)  /*!< TIM17 input capture 1 is connected to HSE/32 */  
#define LL_TIM_TIM17_TI1_RMP_MCO    (TIM17_OR1_TI1_RMP | TIM17_OR1_RMP_MASK)    /*!< TIM17 input capture 1 is connected to MCO */  
/**
  * @}
  */

/**
  * @}
  */
  
/* Exported macro ------------------------------------------------------------*/
/** @defgroup TIM_LL_Exported_Macros TIM Exported Macros
  * @{
  */

/** @defgroup TIM_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/**
  * @brief  Write a value in TIM register
  * @param  __INSTANCE__ TIM Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_TIM_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in TIM register
  * @param  __INSTANCE__ TIM Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_TIM_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup TIM_LL_EM_Exported_Macros Exported_Macros
  * @{
  */
/**
  * @brief  HELPER macro retrieving the UIFCPY flag from the counter value
  * @note  Relevant only if UIF flag remapping has been enabled  (UIF status bit is copied
  * to TIMx_CNT register bit 31)
  * @param  __CNT__ Counter value
  * @retval UIF status bit
  */
#define __LL_TIM_GETFLAG_UIFCPY(__CNT__)  \
   (READ_BIT((__CNT__), TIM_CNT_UIFCPY) >> POSITION_VAL(TIM_CNT_UIFCPY))

/**
  * @brief  HELPER macro calculating DTG[0:7] in the TIMx_BDTR register to 
  * achieve the requested dead time duration
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __CKD__ This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  * @param  __DT__ deadtime duration (in us)
  * @retval DTG[0:7]
  */
#define __LL_TIM_CALC_DEADTIME(__TIMCLK__, __CKD__, __DT__)  \
    ( (((uint64_t)((__DT__)*1000)) < ((DT_DELAY_1+1) * TIM_CALC_DTS((__TIMCLK__), (__CKD__))))           ? ((uint8_t)(((uint64_t)((__DT__)*1000)) / TIM_CALC_DTS((__TIMCLK__), (__CKD__))) & DT_DELAY_1) :                            \
      (((uint64_t)((__DT__)*1000)) < (64 + (DT_DELAY_2+1)) * 2 * TIM_CALC_DTS((__TIMCLK__), (__CKD__)))  ? (DT_RANGE_2 | ((uint8_t)(((((uint64_t)((__DT__)*1000))/ TIM_CALC_DTS((__TIMCLK__), (__CKD__))) >> 1) - 64) & DT_DELAY_2)) :\
      (((uint64_t)((__DT__)*1000)) < (32 + (DT_DELAY_3+1)) * 8 * TIM_CALC_DTS((__TIMCLK__), (__CKD__)))  ? (DT_RANGE_3 | ((uint8_t)(((((uint64_t)((__DT__)*1000))/ TIM_CALC_DTS((__TIMCLK__), (__CKD__))) >> 3) - 32) & DT_DELAY_3)) :\
      (((uint64_t)((__DT__)*1000)) < (32 + (DT_DELAY_4+1)) * 16 * TIM_CALC_DTS((__TIMCLK__), (__CKD__))) ? (DT_RANGE_4 | ((uint8_t)(((((uint64_t)((__DT__)*1000))/ TIM_CALC_DTS((__TIMCLK__), (__CKD__))) >> 4) - 32) & DT_DELAY_4)) :\
       0) 

/**
  * @brief  HELPER macro calculating the prescaler value to achieve the required
  * counter clock frequency
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __CNTCLK__ counter clock frequency (in Hz)
  * @retval Prescaler value
  */
#define __LL_TIM_CALC_PSC(__TIMCLK__, __CNTCLK__)   \
   ((__TIMCLK__) >= (__CNTCLK__)) ? (uint32_t)((__TIMCLK__)/(__CNTCLK__) - 1) : 0

/**
  * @brief  HELPER macro calculating the auto-reload value to achieve the
  * required output signal frequency
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __FREQ__ output signal frequency (in Hz)
  * @retval  Auto-reload value
  */
#define __LL_TIM_CALC_ARR(__TIMCLK__, __PSC__, __FREQ__) \
     (((__TIMCLK__)/((__PSC__) + 1)) >= (__FREQ__)) ? ((__TIMCLK__)/((__FREQ__) * ((__PSC__) + 1)) - 1) : 0

/**
  * @brief  HELPER macro calculating the compare value required to achieve the
  * required timer output compare active/inactive delay
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __DELAY__ timer output compare active/inactive delay (in us)
  * @retval Compare value
  */
#define __LL_TIM_CALC_DELAY(__TIMCLK__, __PSC__, __DELAY__)  \
((uint32_t)(((uint64_t)(__TIMCLK__) * (uint64_t)(__DELAY__)) \
          / ((uint64_t)1000000 * (uint64_t)((__PSC__) + 1))))

/**
  * @brief  HELPER macro calculating the auto-reload value to achieve the  
  *   *         required pulse duration (when the timer operates in one pulse mode)
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __DELAY__ timer output compare active/inactive delay (in us)
  * @param  __PULSE__ pulse duration (in us)
  * @retval Auto-reload value
  */
#define __LL_TIM_CALC_PULSE(__TIMCLK__, __PSC__, __DELAY__, __PULSE__)  \
 ((uint32_t)(__LL_TIM_CALC_DELAY((__TIMCLK__), (__PSC__), (__PULSE__)) \
           + __LL_TIM_CALC_DELAY((__TIMCLK__), (__PSC__), (__DELAY__))))

/**
  * @brief  HELPER macro retrieving the ratio of the input capture prescaler 
  * @param  __ICPSC__ Input capture prescaler value
  * @retval Input capture prescaler ratio (1, 2, 4 or 8)
  */
#define __LL_TIM_GET_ICPSC_RATIO(__ICPSC__)  \
   ((uint32_t)(0x01 << (((__ICPSC__) >> 16) >> POSITION_VAL(TIM_CCMR1_IC1PSC))))

/**
  * @}
  */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup TIM_LL_Exported_Functions TIM Exported Functions
  * @{
  */
/** @defgroup TIM_LL_EF_Time_Base Time Base configuration
  * @{
  */
/**
  * @brief  Enable timer counter.
  * @rmtoll CR1          CEN           LL_TIM_EnableCounter
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableCounter(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->CR1, TIM_CR1_CEN);
}

/**
  * @brief  Disable timer counter.
  * @rmtoll CR1          CEN           LL_TIM_DisableCounter
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableCounter(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN);
}

/**
  * @brief  Indicates whether the timer counter is enabled.
  * @rmtoll CR1          CEN           LL_TIM_IsEnabledCounter
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledCounter(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->CR1, TIM_CR1_CEN) == (TIM_CR1_CEN));
}

/**
  * @brief  Enable update event generation.
  * @rmtoll CR1          UDIS          LL_TIM_EnableUpdateEvent
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableUpdateEvent(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->CR1, TIM_CR1_UDIS);
}

/**
  * @brief  Disable update event generation.
  * @rmtoll CR1          UDIS          LL_TIM_DisableUpdateEvent
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableUpdateEvent(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->CR1, TIM_CR1_UDIS);
}

/**
  * @brief  Indicates whether update event generation is enabled.
  * @rmtoll CR1          UDIS          LL_TIM_IsEnabledUpdateEvent
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledUpdateEvent(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->CR1, TIM_CR1_UDIS) == (TIM_CR1_UDIS));
}

/**
  * @brief  Set update event source
  * @note
  *   Update event source set to LL_TIM_UPDATESOURCE_REGULAR: any of the following events 
  *   generate an update interrupt or DMA request if enabled:
  *     Counter overflow/underflow
  *     Setting the UG bit
  *     Update generation through the slave mode controller
  *   Update event source set to LL_TIM_UPDATESOURCE_COUNTER: only counter 
  *   overflow/underflow generates an update interrupt or DMA request if enabled.
  * @rmtoll CR1          URS           LL_TIM_SetUpdateSource
  * @param  TIMx Timer instance
  * @param  UpdateSource This parameter can be one of the following values:
  *         @arg @ref LL_TIM_UPDATESOURCE_REGULAR
  *         @arg @ref LL_TIM_UPDATESOURCE_COUNTER
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetUpdateSource(TIM_TypeDef * TIMx, uint32_t UpdateSource)
{
  MODIFY_REG(TIMx->CR1, TIM_CR1_URS, UpdateSource);
}

/**
  * @brief  Get actual event update source
  * @rmtoll CR1          URS           LL_TIM_GetUpdateSource
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_UPDATESOURCE_REGULAR
  *         @arg @ref LL_TIM_UPDATESOURCE_COUNTER
  */
__STATIC_INLINE uint32_t LL_TIM_GetUpdateSource(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_BIT(TIMx->CR1, TIM_CR1_URS));
}

/**
  * @brief  Set one pulse mode (one shot v.s. repetitive).
  * @rmtoll CR1          OPM           LL_TIM_GetUpdateSource
  * @param  TIMx Timer instance
  * @param  OnePulseMode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ONEPULSEMODE_SINGLE
  *         @arg @ref LL_TIM_ONEPULSEMODE_REPETITIVE
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetOnePulseMode(TIM_TypeDef * TIMx, uint32_t OnePulseMode)
{
  MODIFY_REG(TIMx->CR1, TIM_CR1_OPM, OnePulseMode);
}

/**
  * @brief  Get actual one pulse mode.
  * @rmtoll CR1          OPM           LL_TIM_GetOnePulseMode
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_ONEPULSEMODE_SINGLE
  *         @arg @ref LL_TIM_ONEPULSEMODE_REPETITIVE
  */
__STATIC_INLINE uint32_t LL_TIM_GetOnePulseMode(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_BIT(TIMx->CR1, TIM_CR1_OPM));
}

/**
  * @brief  Set the timer counter counting mode.
  * @note Macro IS_TIM_CC4_INSTANCE(TIMx) can be used to check whether or not
  * the counter mode selection feature is supported by a timer instance.
  * @rmtoll CR1          DIR           LL_TIM_SetCounterMode\n
  *         CR1          CMS           LL_TIM_SetCounterMode
  * @param  TIMx Timer instance
  * @param  CounterMode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERMODE_UP
  *         @arg @ref LL_TIM_COUNTERMODE_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP_DOWN
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetCounterMode(TIM_TypeDef * TIMx, uint32_t CounterMode)
{
  MODIFY_REG(TIMx->CR1, TIM_CR1_DIR | TIM_CR1_CMS, CounterMode);
}

/**
  * @brief  Get actual counter mode.
  * @note Macro IS_TIM_CC4_INSTANCE(TIMx) can be used to check whether or not
  * the counter mode selection feature is supported by a timer instance.
  * @rmtoll CR1          DIR           LL_TIM_GetCounterMode\n
  *         CR1          CMS           LL_TIM_GetCounterMode
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERMODE_UP
  *         @arg @ref LL_TIM_COUNTERMODE_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP_DOWN
  */
__STATIC_INLINE uint32_t LL_TIM_GetCounterMode(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_BIT(TIMx->CR1, TIM_CR1_DIR | TIM_CR1_CMS));
}

/**
  * @brief  Enable auto-reload (ARR) preload.
  * @rmtoll CR1          ARPE          LL_TIM_EnableARRPreload
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableARRPreload(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->CR1, TIM_CR1_ARPE);
}

/**
  * @brief  Disable auto-reload (ARR) preload.
  * @rmtoll CR1          ARPE          LL_TIM_DisableARRPreload
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableARRPreload(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->CR1, TIM_CR1_ARPE);
}

/**
  * @brief  Indicates whether auto-reload (ARR) preload is enabled.
  * @rmtoll CR1          ARPE          LL_TIM_IsEnabledARRPreload
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledARRPreload(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->CR1, TIM_CR1_ARPE) == (TIM_CR1_ARPE));
}

/**
  * @brief  Set the division ratio between the timer clock  and the sampling clock used by the dead-time generators (when supported) and the digital filters.
  * @note Macro IS_TIM_CLOCK_DIVISION_INSTANCE(TIMx) can be used to check 
  * whether or not the clock division feature is supported by the a timer
  * instance.
  * @rmtoll CR1          CKD           LL_TIM_SetClockDivision
  * @param  TIMx Timer instance
  * @param  ClockDivision This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetClockDivision(TIM_TypeDef * TIMx, uint32_t ClockDivision)
{
  MODIFY_REG(TIMx->CR1, TIM_CR1_CKD, ClockDivision);
}

/**
  * @brief  Get the actual division ratio between the timer clock  and the sampling clock used by the dead-time generators (when supported) and the digital filters.
  * @note Macro IS_TIM_CLOCK_DIVISION_INSTANCE(TIMx) can be used to check 
  * whether or not the clock division feature is supported by the a timer
  * instance.
  * @rmtoll CR1          CKD           LL_TIM_GetClockDivision
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  */
__STATIC_INLINE uint32_t LL_TIM_GetClockDivision(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_BIT(TIMx->CR1, TIM_CR1_CKD));
}

/**
  * @brief  Set the counter value.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @rmtoll CNT          CNT           LL_TIM_SetCounter
  * @param  TIMx Timer instance
  * @param  Counter Counter value
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetCounter(TIM_TypeDef * TIMx, uint32_t Counter)
{
  WRITE_REG(TIMx->CNT, Counter);
}

/**
  * @brief  Get the counter value.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * counter.
  * @rmtoll CNT          CNT           LL_TIM_GetCounter
  * @param  TIMx Timer instance
  * @retval Counter value
  */
__STATIC_INLINE uint32_t LL_TIM_GetCounter(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CNT));
}

/**
  * @brief  Get the current direction of the counter
  * @rmtoll CR1          DIR           LL_TIM_GetDirection
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERDIRECTION_UP
  *         @arg @ref LL_TIM_COUNTERDIRECTION_DOWN
  */
__STATIC_INLINE uint32_t LL_TIM_GetDirection(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_BIT(TIMx->CR1, TIM_CR1_DIR));
}

/**
  * @brief  Set the prescaler value.
  * @note The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
  * @note The prescaler can be changed on the fly as this control register is buffered. The new
  * prescaler ratio is taken into account at the next update event.
  * @rmtoll PSC          PSC           LL_TIM_SetPrescaler
  * @param  TIMx Timer instance
  * @param  Prescaler Between 0 and 65535
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetPrescaler(TIM_TypeDef * TIMx, uint32_t Prescaler)
{
  WRITE_REG(TIMx->PSC, Prescaler);
}

/**
  * @brief  Get the prescaler value.
  * @rmtoll PSC          PSC           LL_TIM_GetPrescaler
  * @param  TIMx Timer instance
  * @retval Prescaler value
  */
__STATIC_INLINE uint32_t LL_TIM_GetPrescaler(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->PSC));
}

/**
  * @brief  Set the auto-reload value.
  * @note The counter is blocked while the auto-reload value is null.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @rmtoll ARR          ARR           LL_TIM_SetAutoReload
  * @param  TIMx Timer instance
  * @param  AutoReload Between 0 and 65535
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetAutoReload(TIM_TypeDef * TIMx, uint32_t AutoReload)
{
  WRITE_REG(TIMx->ARR, AutoReload);
}

/**
  * @brief  Get the auto-reload value.
  * @rmtoll ARR          ARR           LL_TIM_GetAutoReload
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @param  TIMx Timer instance
  * @retval Auto-reload value
  */
__STATIC_INLINE uint32_t LL_TIM_GetAutoReload(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->ARR));
}

/**
  * @brief  Set the repetition counter value.
  * @note For advanced timer instances RepetitionCounter can be up to 65535.
  * @note Macro IS_TIM_REPETITION_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a repetition counter.
  * @rmtoll RCR          REP           LL_TIM_SetRepetitionCounter
  * @param  TIMx Timer instance
  * @param  RepetitionCounter Between 0 and 255
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetRepetitionCounter(TIM_TypeDef * TIMx, uint32_t RepetitionCounter)
{
  WRITE_REG(TIMx->RCR, RepetitionCounter);
}

/**
  * @brief  Get the repetition counter value.
  * @note Macro IS_TIM_REPETITION_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a repetition counter.
  * @rmtoll RCR          REP           LL_TIM_GetRepetitionCounter
  * @param  TIMx Timer instance
  * @retval Repetition counter value
  */
__STATIC_INLINE uint32_t LL_TIM_GetRepetitionCounter(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->RCR));
}

/**
  * @brief  Force a continuous copy of the update interrupt flag (UIF) into the timer counter register (bit 31). 
  * @note This allows both the counter value and a potential roll-over condition signalled by the UIFCPY flag to be read in an atomic way.
  * @rmtoll CR1          UIFREMAP      LL_TIM_EnableUIFRemap
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableUIFRemap(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->CR1, TIM_CR1_UIFREMAP);
}

/**
  * @brief  Disable update interrupt flag (UIF) remapping.
  * @rmtoll CR1          UIFREMAP      LL_TIM_DisableUIFRemap
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableUIFRemap(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->CR1, TIM_CR1_UIFREMAP);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Capture_Compare Capture Compare configuration
  * @{
  */
/**
  * @brief  Enable  the capture/compare control bits (CCxE, CCxNE and OCxM) preload.
  * @note CCxE, CCxNE and OCxM bits are preloaded, after having been written, 
  * they are updated only when a commutation event (COM) occurs.
  * @note Only on channels that have a complementary output.
  * @note Macro IS_TIM_COMMUTATION_EVENT_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance is able to generate a commutation event.
  * @rmtoll CR2          CCPC          LL_TIM_CC_EnablePreload
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_CC_EnablePreload(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->CR2, TIM_CR2_CCPC);
}

/**
  * @brief  Disable  the capture/compare control bits (CCxE, CCxNE and OCxM) preload.
  * @note Macro IS_TIM_COMMUTATION_EVENT_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance is able to generate a commutation event.
  * @rmtoll CR2          CCPC          LL_TIM_CC_DisablePreload
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_CC_DisablePreload(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->CR2, TIM_CR2_CCPC);
}

/**
  * @brief  Set the updated source of the capture/compare control bits (CCxE, CCxNE and OCxM).
  * @note Macro IS_TIM_COMMUTATION_EVENT_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance is able to generate a commutation event.
  * @rmtoll CR2          CCUS          LL_TIM_CC_SetUpdate
  * @param  TIMx Timer instance
  * @param  CCUpdateSource This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CCUPDATESOURCE_COMG_ONLY
  *         @arg @ref LL_TIM_CCUPDATESOURCE_COMG_AND_TRGI
  * @retval None
  */
__STATIC_INLINE void LL_TIM_CC_SetUpdate(TIM_TypeDef * TIMx, uint32_t CCUpdateSource)
{
  MODIFY_REG(TIMx->CR2, TIM_CR2_CCUS, CCUpdateSource);
}

/**
  * @brief  Set the trigger of the capture/compare DMA request.
  * @rmtoll CR2          CCDS          LL_TIM_CC_SetDMAReqTrigger
  * @param  TIMx Timer instance
  * @param  DMAReqTrigger This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CCDMAREQUEST_CC
  *         @arg @ref LL_TIM_CCDMAREQUEST_UPDATE
  * @retval None
  */
__STATIC_INLINE void LL_TIM_CC_SetDMAReqTrigger(TIM_TypeDef * TIMx, uint32_t DMAReqTrigger)
{
  MODIFY_REG(TIMx->CR2, TIM_CR2_CCDS, DMAReqTrigger);
}

/**
  * @brief  Set the lock level to freeze the
  * configuration of several  capture/compare parameters.
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  * the lock mechanism is supported by a timer instance.
  * @rmtoll BDTR         LOCK          LL_TIM_CC_SetLockLevel
  * @param  TIMx Timer instance
  * @param  LockLevel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_LOCKLEVEL_OFF
  *         @arg @ref LL_TIM_LOCKLEVEL_1
  *         @arg @ref LL_TIM_LOCKLEVEL_2
  *         @arg @ref LL_TIM_LOCKLEVEL_3
  * @retval None
  */
__STATIC_INLINE void LL_TIM_CC_SetLockLevel(TIM_TypeDef * TIMx, uint32_t LockLevel)
{
  MODIFY_REG(TIMx->BDTR, TIM_BDTR_LOCK, LockLevel);
}

/**
  * @brief  Enable capture/compare channels.
  * @note Macros IS_TIM_CCX_INSTANCE(TIMx, Channel) or 
  * IS_TIM_CCXN_INSTANCE(TIMx, Channel) can be used to check whether or not
  * a channel is supported by the a timer instance.
  * @rmtoll CCER         CC1E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC1NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC2E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC2NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC3E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC3NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC4E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC5E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC6E          LL_TIM_CC_EnableChannel\n
  * @param  TIMx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
__STATIC_INLINE void LL_TIM_CC_EnableChannel(TIM_TypeDef * TIMx, uint32_t Channels)
{
  SET_BIT(TIMx->CCER, Channels);
}

/**
  * @brief  Disable capture/compare channels.
  * @note Macros IS_TIM_CCX_INSTANCE(TIMx, Channel) or 
  * IS_TIM_CCXN_INSTANCE(TIMx, Channel) can be used to check whether or not
  * a channel is supported by the a timer instance.
  * @rmtoll CCER         CC1E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC1NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC2E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC2NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC3E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC3NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC4E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC5E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC6E          LL_TIM_CC_EnableChannel\n
  * @param  TIMx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
__STATIC_INLINE void LL_TIM_CC_DisableChannel(TIM_TypeDef * TIMx, uint32_t Channels)
{
  CLEAR_BIT(TIMx->CCER, Channels);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Output_Channel Output channel configuration
  * @{
  */
/**
  * @brief  Configure an output channel.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1M          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        CC1S          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2M          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        CC2S          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3M          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        CC3S          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4M          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        CC4S          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5M          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        CC5S          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6M          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        CC6S          LL_TIM_OC_ConfigOutput\n
  *         CCER         OC1P          LL_TIM_OC_ConfigOutput\n
  *         CCER         OC2P          LL_TIM_OC_ConfigOutput\n
  *         CCER         OC3P          LL_TIM_OC_ConfigOutput\n
  *         CCER         OC4P          LL_TIM_OC_ConfigOutput\n
  *         CCER         OC5P          LL_TIM_OC_ConfigOutput\n
  *         CCER         OC6P          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS1          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS2          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS3          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS4          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS5          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS6          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_TIM_OCMODE_FROZEN or ... or @ref LL_TIM_OCMODE_ASSYMETRIC_PWM2
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH or @ref LL_TIM_OCPOLARITY_LOW
  *         @arg @ref LL_TIM_OCIDLESTATE_LOW or @ref LL_TIM_OCIDLESTATE_HIGH
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_ConfigOutput(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t Configuration)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel]),  (Configuration & (TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S)) << SHIFT_TAB_OCxx[iChannel]);
  MODIFY_REG(TIMx->CCER, (TIM_CCER_CC1P << SHIFT_TAB_CCxP[iChannel]),  (Configuration & TIM_CCER_CC1P) << SHIFT_TAB_CCxP[iChannel]);
  MODIFY_REG(TIMx->CR2, (TIM_CR2_OIS1 << SHIFT_TAB_OISx[iChannel]),  (Configuration & TIM_CR2_OIS1) << SHIFT_TAB_OISx[iChannel]);
}

/**
  * @brief  Configure a complementary output channel.
  * @note Macro IS_TIM_CCXN_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a complementary channel is supported by the a timer instance.
  * @rmtoll CCER         CC1NP          LL_TIM_OC_ConfigOutputN\n
  *         CCER         CC2NP          LL_TIM_OC_ConfigOutputN\n
  *         CCER         CC3NP          LL_TIM_OC_ConfigOutputN\n
  *         CR2          OIS1N          LL_TIM_OC_ConfigOutputN\n
  *         CR2          OIS2N          LL_TIM_OC_ConfigOutputN\n
  *         CR2          OIS3N          LL_TIM_OC_ConfigOutputN\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH
  *         @arg @ref LL_TIM_OCPOLARITY_LOW
  * @param  IdleState This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCIDLESTATE_LOW
  *         @arg @ref LL_TIM_OCIDLESTATE_HIGH
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_ConfigOutputN(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t Polarity, uint32_t IdleState)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  MODIFY_REG(TIMx->CCER, (TIM_CCER_CC1P << SHIFT_TAB_CCxP[iChannel]),  Polarity << SHIFT_TAB_CCxP[iChannel]);
  MODIFY_REG(TIMx->CR2, (TIM_CR2_OIS1 << SHIFT_TAB_OISx[iChannel]),  IdleState << SHIFT_TAB_OISx[iChannel]);
}

/**
  * @brief  Set the output compare mode of an output channel.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1M          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2M          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3M          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4M          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5M          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6M          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCMODE_FROZEN
  *         @arg @ref LL_TIM_OCMODE_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_TOGGLE
  *         @arg @ref LL_TIM_OCMODE_FORCED_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_FORCED_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_PWM1
  *         @arg @ref LL_TIM_OCMODE_PWM2
  *         @arg @ref LL_TIM_OCMODE_RETRIG_OPM1
  *         @arg @ref LL_TIM_OCMODE_RETRIG_OPM2
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM1
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM2
  *         @arg @ref LL_TIM_OCMODE_ASSYMETRIC_PWM1
  *         @arg @ref LL_TIM_OCMODE_ASSYMETRIC_PWM2
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetMode(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t Mode)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel]),  Mode << SHIFT_TAB_OCxx[iChannel]);
}

/**
  * @brief  Get the output compare mode of an output channel.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1M          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2M          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3M          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4M          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5M          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6M          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCMODE_FROZEN
  *         @arg @ref LL_TIM_OCMODE_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_TOGGLE
  *         @arg @ref LL_TIM_OCMODE_FORCED_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_FORCED_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_PWM1
  *         @arg @ref LL_TIM_OCMODE_PWM2
  *         @arg @ref LL_TIM_OCMODE_RETRIG_OPM1
  *         @arg @ref LL_TIM_OCMODE_RETRIG_OPM2
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM1
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM2
  *         @arg @ref LL_TIM_OCMODE_ASSYMETRIC_PWM1
  *         @arg @ref LL_TIM_OCMODE_ASSYMETRIC_PWM2
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetMode(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));  
  return (READ_BIT(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel])) >> SHIFT_TAB_OCxx[iChannel]);
}

/**
  * @brief  Set the polarity of an output channel.
  * @note Macros IS_TIM_CCX_INSTANCE(TIMx, Channel) or 
  * IS_TIM_CCXN_INSTANCE(TIMx, Channel) can be used to check whether or not
  * a channel is supported by the a timer instance.
  * @rmtoll CCER         CC1P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC1NP         LL_TIM_CC_EnableChannel\n
  *         CCER         CC2P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC2NP         LL_TIM_CC_EnableChannel\n
  *         CCER         CC3P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC3NP         LL_TIM_CC_EnableChannel\n
  *         CCER         CC4P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC5P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC6P          LL_TIM_CC_EnableChannel\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH
  *         @arg @ref LL_TIM_OCPOLARITY_LOW
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetPolarity(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t Polarity)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  MODIFY_REG(TIMx->CCER, (TIM_CCER_CC1P << SHIFT_TAB_CCxP[iChannel]),  Polarity << SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Get the polarity of an output channel.
  * @note Macros IS_TIM_CCX_INSTANCE(TIMx, Channel) or 
  * IS_TIM_CCXN_INSTANCE(TIMx, Channel) can be used to check whether or not
  * a channel is supported by the a timer instance.
  * @rmtoll CCER         CC1P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC1NP         LL_TIM_CC_EnableChannel\n
  *         CCER         CC2P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC2NP         LL_TIM_CC_EnableChannel\n
  *         CCER         CC3P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC3NP         LL_TIM_CC_EnableChannel\n
  *         CCER         CC4P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC5P          LL_TIM_CC_EnableChannel\n
  *         CCER         CC6P          LL_TIM_CC_EnableChannel\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH
  *         @arg @ref LL_TIM_OCPOLARITY_LOW
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetPolarity(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  return (READ_BIT(TIMx->CCER, (TIM_CCER_CC1P << SHIFT_TAB_CCxP[iChannel])) >> SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Set the IDLE state of an output channel
  * @note Macros IS_TIM_CCX_INSTANCE(TIMx, Channel) or 
  * IS_TIM_CCXN_INSTANCE(TIMx, Channel) can be used to check whether or not
  * a channel is supported by the a timer instance.
  * @rmtoll CR2         OIS1          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS2N         LL_TIM_CC_EnableChannel\n
  *         CR2         OIS2          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS2N         LL_TIM_CC_EnableChannel\n
  *         CR2         OIS3          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS3N         LL_TIM_CC_EnableChannel\n
  *         CR2         OIS4          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS5          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS6          LL_TIM_CC_EnableChannel\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  IdleState This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCIDLESTATE_LOW
  *         @arg @ref LL_TIM_OCIDLESTATE_HIGH
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetIdleState(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t IdleState)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  MODIFY_REG(TIMx->CR2, (TIM_CR2_OIS1 << SHIFT_TAB_OISx[iChannel]),  IdleState << SHIFT_TAB_OISx[iChannel]);
}

/**
  * @brief  Get the IDLE state of an output channel
  * @note Macros IS_TIM_CCX_INSTANCE(TIMx, Channel) or 
  * IS_TIM_CCXN_INSTANCE(TIMx, Channel) can be used to check whether or not
  * a channel is supported by the a timer instance.
  * @rmtoll CR2         OIS1          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS2N         LL_TIM_CC_EnableChannel\n
  *         CR2         OIS2          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS2N         LL_TIM_CC_EnableChannel\n
  *         CR2         OIS3          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS3N         LL_TIM_CC_EnableChannel\n
  *         CR2         OIS4          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS5          LL_TIM_CC_EnableChannel\n
  *         CR2         OIS6          LL_TIM_CC_EnableChannel\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCIDLESTATE_LOW
  *         @arg @ref LL_TIM_OCIDLESTATE_HIGH
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetIdleState(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  return (READ_BIT(TIMx->CR2, (TIM_CR2_OIS1 << SHIFT_TAB_OISx[iChannel])) >> SHIFT_TAB_OISx[iChannel]);
}

/**
  * @brief  Enable fast mode for the output channel.
  * @note Acts only if the channel is configured in PWM1 or PWM2 mode.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6FE          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_EnableFast(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TIM_CCMR1_OC1FE << SHIFT_TAB_OCxx[iChannel]));

}

/**
  * @brief  Disable fast mode for the output channel.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5FE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6FE          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_DisableFast(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TIM_CCMR1_OC1FE << SHIFT_TAB_OCxx[iChannel]));

}

/**
  * @brief  Enable compare register (TIMx_CCRx) preload for the output channel.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6PE          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_EnablePreload(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @brief  Disable compare register (TIMx_CCRx) preload for the output channel.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5PE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6PE          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_DisablePreload(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));

}

/**
  * @brief  Enable clearing the output channel on an external event.
  * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
  * @note Macro IS_TIM_OCXREF_CLEAR_INSTANCE(TIMx) can be used to check whether
  * or not a timer instance can clear the OCxREF signal on an external event.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6CE          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_EnableClear(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TIM_CCMR1_OC1CE << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @brief  Disable clearing the output channel on an external event.
  * @note Macro IS_TIM_OCXREF_CLEAR_INSTANCE(TIMx) can be used to check whether
  * or not a timer instance can clear the OCxREF signal on an external event.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        OC1CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        OC2CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC3CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        OC4CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC5CE          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        OC6CE          LL_TIM_OC_ConfigOutput\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_DisableClear(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TIM_CCMR1_OC1CE << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @brief  Set the dead-time delay (delay inserted between the rising edge of the OCxREF signal and the rising edge if the Ocx and OCxN signals).
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  * dead-time insertion feature is supported by a timer instance.
  * @rmtoll BDTR         DTG           LL_TIM_OC_SetDeadTime
  * @param  TIMx Timer instance
  * @param  DeadTime Between 0 and 255
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetDeadTime(TIM_TypeDef * TIMx, uint32_t DeadTime)
{
  MODIFY_REG(TIMx->BDTR, TIM_BDTR_DTG, DeadTime);
}

/**
  * @brief  Set compare value for output channel 1 (TIMx_CCR1)
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC1_INSTANCE(TIMx) can be used to check whether or not
  * output channel 1 is supported by a timer instance.
  * @rmtoll CCR1         CCR1          LL_TIM_OC_SetCompareCH1
  * @param  TIMx Timer instance
  * @param  CompareValue Between 0 and 65535
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH1(TIM_TypeDef * TIMx, uint32_t CompareValue)
{
  WRITE_REG(TIMx->CCR1, CompareValue);
}

/**
  * @brief  Set compare value for output channel 2 (TIMx_CCR2)
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC2_INSTANCE(TIMx) can be used to check whether or not
  * output channel 2 is supported by a timer instance.
  * @rmtoll CCR2         CCR2          LL_TIM_OC_SetCompareCH2
  * @param  TIMx Timer instance
  * @param  CompareValue Between 0 and 65535
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH2(TIM_TypeDef * TIMx, uint32_t CompareValue)
{
  WRITE_REG(TIMx->CCR2, CompareValue);
}

/**
  * @brief  Set compare value for output channel 3 (TIMx_CCR3)
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC3_INSTANCE(TIMx) can be used to check whether or not
  * output channel is supported by a timer instance.
  * @rmtoll CCR3         CCR3          LL_TIM_OC_SetCompareCH3
  * @param  TIMx Timer instance
  * @param  CompareValue Between 0 and 65535
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH3(TIM_TypeDef * TIMx, uint32_t CompareValue)
{
  WRITE_REG(TIMx->CCR3, CompareValue);
}

/**
  * @brief  Set compare value for output channel 4 (TIMx_CCR4)
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC4_INSTANCE(TIMx) can be used to check whether or not
  * output channel 4 is supported by a timer instance.
  * @rmtoll CCR4         CCR4          LL_TIM_OC_SetCompareCH4
  * @param  TIMx Timer instance
  * @param  CompareValue Between 0 and 65535
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH4(TIM_TypeDef * TIMx, uint32_t CompareValue)
{
  WRITE_REG(TIMx->CCR4, CompareValue);
}

/**
  * @brief  Set compare value for output channel 5 (TIMx_CCR5)
  * @note Macro IS_TIM_CC5_INSTANCE(TIMx) can be used to check whether or not
  * output channel 5 is supported by a timer instance.
  * @rmtoll CCR5         CCR5          LL_TIM_OC_SetCompareCH5
  * @param  TIMx Timer instance
  * @param  CompareValue Between 0 and 65535
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH5(TIM_TypeDef * TIMx, uint32_t CompareValue)
{
  WRITE_REG(TIMx->CCR5, CompareValue);
}

/**
  * @brief  Set compare value for output channel 6 (TIMx_CCR6)
  * @note Macro IS_TIM_CC6_INSTANCE(TIMx) can be used to check whether or not
  * output channel 6 is supported by a timer instance.
  * @rmtoll CCR6         CCR6          LL_TIM_OC_SetCompareCH6
  * @param  TIMx Timer instance
  * @param  CompareValue Between 0 and 65535
  * @retval None
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH6(TIM_TypeDef * TIMx, uint32_t CompareValue)
{
  WRITE_REG(TIMx->CCR6, CompareValue);
}

/**
  * @brief  Get compare value (TIMx_CCR1) set for  output channel 1
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC1_INSTANCE(TIMx) can be used to check whether or not
  * output channel 1 is supported by a timer instance.
  * @rmtoll CCR1         CCR1          LL_TIM_OC_GetCompareCH1
  * @param  TIMx Timer instance
  * @retval CompareValue
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH1(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR1));
}

/**
  * @brief  Get compare value (TIMx_CCR2) set for  output channel 2
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC2_INSTANCE(TIMx) can be used to check whether or not
  * output channel 2 is supported by a timer instance.
  * @rmtoll CCR2         CCR2          LL_TIM_OC_GetCompareCH2
  * @param  TIMx Timer instance
  * @retval CompareValue
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH2(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR2));
}

/**
  * @brief  Get compare value (TIMx_CCR3) set for  output channel 3
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC3_INSTANCE(TIMx) can be used to check whether or not
  * output channel 3 is supported by a timer instance.
  * @rmtoll CCR3         CCR3          LL_TIM_OC_GetCompareCH3
  * @param  TIMx Timer instance
  * @retval CompareValue
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH3(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR3));
}

/**
  * @brief  Get compare value (TIMx_CCR4) set for  output channel 4
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC4_INSTANCE(TIMx) can be used to check whether or not
  * output channel 4 is supported by a timer instance.
  * @rmtoll CCR4         CCR4          LL_TIM_OC_GetCompareCH4
  * @param  TIMx Timer instance
  * @retval CompareValue
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH4(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR4));
}

/**
  * @brief  Get compare value (TIMx_CCR5) set for  output channel 5
  * @note Macro IS_TIM_CC5_INSTANCE(TIMx) can be used to check whether or not
  * output channel 5 is supported by a timer instance.
  * @rmtoll CCR5         CCR5          LL_TIM_OC_GetCompareCH5
  * @param  TIMx Timer instance
  * @retval CompareValue
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH5(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR5));
}

/**
  * @brief  Get compare value (TIMx_CCR6) set for  output channel 6
  * @note Macro IS_TIM_CC6_INSTANCE(TIMx) can be used to check whether or not
  * output channel 6 is supported by a timer instance.
  * @rmtoll CCR6         CCR6          LL_TIM_OC_GetCompareCH6
  * @param  TIMx Timer instance
  * @retval CompareValue
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH6(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR6));
}

/**
  * @brief  Select on which reference signal the OC5REF is combined to.
  * @note Macro IS_TIM_COMBINED3PHASEPWM_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports the combined 3-phase PWM mode.
  * @rmtoll CCR5         GC5C3          LL_TIM_SetCH5CombinedChannels\n
  *         CCR5         GC5C2          LL_TIM_SetCH5CombinedChannels\n
  *         CCR5         GC5C1          LL_TIM_SetCH5CombinedChannels\n
  * @param  TIMx Timer instance
  * @param  GroupCH5 This parameter can be one of the following values:
  *         @arg @ref LL_TIM_GROUPCH5_NONE
  *         @arg @ref LL_TIM_GROUPCH5_OC1REFC
  *         @arg @ref LL_TIM_GROUPCH5_OC2REFC
  *         @arg @ref LL_TIM_GROUPCH5_OC3REFC
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetCH5CombinedChannels(TIM_TypeDef * TIMx, uint32_t GroupCH5)
{
  MODIFY_REG(TIMx->CCR5, TIM_CCR5_CCR5, GroupCH5);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Input_Channel Input channel configuration
  * @{
  */
/**
  * @brief  Configure input channel.
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        CC1S          LL_TIM_IC_Config\n
  *         CCMR1        IC1PSC        LL_TIM_IC_Config\n
  *         CCMR1        IC1F          LL_TIM_IC_Config\n
  *         CCMR1        CC2S          LL_TIM_IC_Config\n
  *         CCMR1        IC2PSC        LL_TIM_IC_Config\n
  *         CCMR1        IC2F          LL_TIM_IC_Config\n
  *         CCMR2        CC3S          LL_TIM_IC_Config\n
  *         CCMR2        IC3PSC        LL_TIM_IC_Config\n
  *         CCMR2        IC3F          LL_TIM_IC_Config\n
  *         CCMR2        CC4S          LL_TIM_IC_Config\n
  *         CCMR2        IC4PSC        LL_TIM_IC_Config\n
  *         CCMR2        IC4F          LL_TIM_IC_Config\n
  *         CCER         CC1P          LL_TIM_IC_Config\n
  *         CCER         CC1NP         LL_TIM_IC_Config\n
  *         CCER         CC2P          LL_TIM_IC_Config\n
  *         CCER         CC2NP         LL_TIM_IC_Config\n
  *         CCER         CC3P          LL_TIM_IC_Config\n
  *         CCER         CC3NP         LL_TIM_IC_Config\n
  *         CCER         CC4P          LL_TIM_IC_Config\n
  *         CCER         CC4NP         LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECTTI or @ref LL_TIM_ACTIVEINPUT_INDIRECTTI or @ref LL_TIM_ACTIVEINPUT_TRC
  *         @arg @ref LL_TIM_ICPSC_DIV1 or ... or @ref LL_TIM_ICPSC_DIV8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1 or ... or @ref LL_TIM_IC_FILTER_FDIV32_N8
  *         @arg @ref LL_TIM_IC_POLARITY_RISING or @ref LL_TIM_IC_POLARITY_FALLING or @ref LL_TIM_IC_POLARITY_BOTHEDGE
  * @retval None
  */
__STATIC_INLINE void LL_TIM_IC_Config(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t Configuration)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC | TIM_CCMR1_CC1S) << SHIFT_TAB_ICxx[iChannel]),  ((Configuration >> 16) & (TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC | TIM_CCMR1_CC1S))  << SHIFT_TAB_ICxx[iChannel]);
  MODIFY_REG(TIMx->CCER, ((TIM_CCER_CC1NP | TIM_CCER_CC1P) << SHIFT_TAB_CCxP[iChannel]),  (Configuration & (TIM_CCER_CC1NP | TIM_CCER_CC1P)) << SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Set the active input
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        CC1S          LL_TIM_IC_Config\n
  *         CCMR1        CC2S          LL_TIM_IC_Config\n
  *         CCMR2        CC3S          LL_TIM_IC_Config\n
  *         CCMR2        CC4S          LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ICActiveInput This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECTTI
  *         @arg @ref LL_TIM_ACTIVEINPUT_INDIRECTTI
  *         @arg @ref LL_TIM_ACTIVEINPUT_TRC
  * @retval None
  */
__STATIC_INLINE void LL_TIM_IC_SetActiveInput(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t ICActiveInput)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TIM_CCMR1_CC1S) << SHIFT_TAB_ICxx[iChannel]),  (ICActiveInput >> 16) << SHIFT_TAB_ICxx[iChannel]);
}

/**
  * @brief  Get the current active input
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        CC1S          LL_TIM_IC_Config\n
  *         CCMR1        CC2S          LL_TIM_IC_Config\n
  *         CCMR2        CC3S          LL_TIM_IC_Config\n
  *         CCMR2        CC4S          LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECTTI
  *         @arg @ref LL_TIM_ACTIVEINPUT_INDIRECTTI
  *         @arg @ref LL_TIM_ACTIVEINPUT_TRC
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetActiveInput(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));  
  return ((READ_BIT(*pReg, ((TIM_CCMR1_CC1S) << SHIFT_TAB_ICxx[iChannel])) >> SHIFT_TAB_ICxx[iChannel]) << 16);
}

/**
  * @brief  Set the prescaler of input channel
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        IC1PSC        LL_TIM_IC_Config\n
  *         CCMR1        IC2PSC        LL_TIM_IC_Config\n
  *         CCMR2        IC3PSC        LL_TIM_IC_Config\n
  *         CCMR2        IC4PSC        LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ICPrescaler This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  * @retval None
  */
__STATIC_INLINE void LL_TIM_IC_SetPrescaler(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t ICPrescaler)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TIM_CCMR1_IC1PSC) << SHIFT_TAB_ICxx[iChannel]),  (ICPrescaler >> 16) << SHIFT_TAB_ICxx[iChannel]);
}

/**
  * @brief  Get the current prescaler value acting on an  input channel
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        IC1PSC        LL_TIM_IC_Config\n
  *         CCMR1        IC2PSC        LL_TIM_IC_Config\n
  *         CCMR2        IC3PSC        LL_TIM_IC_Config\n
  *         CCMR2        IC4PSC        LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetPrescaler(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));  
  return ((READ_BIT(*pReg, ((TIM_CCMR1_IC1PSC) << SHIFT_TAB_ICxx[iChannel])) >> SHIFT_TAB_ICxx[iChannel]) << 16);
}

/**
  * @brief  Set the input filter duration
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCMR1        IC1F          LL_TIM_IC_Config\n
  *         CCMR1        IC2F          LL_TIM_IC_Config\n
  *         CCMR2        IC3F          LL_TIM_IC_Config\n
  *         CCMR2        IC4F          LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ICFilter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N8
  * @retval None
  */
__STATIC_INLINE void LL_TIM_IC_SetFilter(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t ICFilter)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TIM_CCMR1_IC1F) << SHIFT_TAB_ICxx[iChannel]),  (ICFilter >> 16) << SHIFT_TAB_ICxx[iChannel]);
}

/**
  * @brief  Get the input filter duration
  * @rmtoll CCMR1        IC1F          LL_TIM_IC_Config\n
  *         CCMR1        IC2F          LL_TIM_IC_Config\n
  *         CCMR2        IC3F          LL_TIM_IC_Config\n
  *         CCMR2        IC4F          LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N8
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetFilter(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1)+ OFFSET_TAB_CCMRx[iChannel]));  
  return ((READ_BIT(*pReg, ((TIM_CCMR1_IC1F) << SHIFT_TAB_ICxx[iChannel])) >> SHIFT_TAB_ICxx[iChannel]) << 16 );
}

/**
  * @brief  Set the input channel polarity
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCER         CC1P          LL_TIM_IC_Config\n
  *         CCER         CC1NP         LL_TIM_IC_Config\n
  *         CCER         CC2P          LL_TIM_IC_Config\n
  *         CCER         CC2NP         LL_TIM_IC_Config\n
  *         CCER         CC3P          LL_TIM_IC_Config\n
  *         CCER         CC3NP         LL_TIM_IC_Config\n
  *         CCER         CC4P          LL_TIM_IC_Config\n
  *         CCER         CC4NP         LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ICPolarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_IC_POLARITY_RISING
  *         @arg @ref LL_TIM_IC_POLARITY_FALLING
  *         @arg @ref LL_TIM_IC_POLARITY_BOTHEDGE
  * @retval None
  */
__STATIC_INLINE void LL_TIM_IC_SetPolarity(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t ICPolarity)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  MODIFY_REG(TIMx->CCER, ((TIM_CCER_CC1NP | TIM_CCER_CC1P) << SHIFT_TAB_CCxP[iChannel]),  ICPolarity << SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Get the current input channel polarity
  * @note Macro IS_TIM_CCX_INSTANCE(TIMx, Channel) can be used to check whether
  * or not a channel is supported by the a timer instance.
  * @rmtoll CCER         CC1P          LL_TIM_IC_Config\n
  *         CCER         CC1NP         LL_TIM_IC_Config\n
  *         CCER         CC2P          LL_TIM_IC_Config\n
  *         CCER         CC2NP         LL_TIM_IC_Config\n
  *         CCER         CC3P          LL_TIM_IC_Config\n
  *         CCER         CC3NP         LL_TIM_IC_Config\n
  *         CCER         CC4P          LL_TIM_IC_Config\n
  *         CCER         CC4NP         LL_TIM_IC_Config\n
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_IC_POLARITY_RISING
  *         @arg @ref LL_TIM_IC_POLARITY_FALLING
  *         @arg @ref LL_TIM_IC_POLARITY_BOTHEDGE
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetPolarity(TIM_TypeDef * TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  return (READ_BIT(TIMx->CCER, ((TIM_CCER_CC1NP | TIM_CCER_CC1P) << SHIFT_TAB_CCxP[iChannel])) >> SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Connect the TIMx_CH1, CH2 and CH3 pins  to the TI1 input (XOR combination).
  * @note Macro IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  * a timer instance provides an XOR input.
  * @rmtoll CR2          TI1S          LL_TIM_IC_EnableXORCombination
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_IC_EnableXORCombination(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->CR2, TIM_CR2_TI1S);
}

/**
  * @brief  Connect the TIMx_CH1 pin to TI1 input.
  * @note Macro IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  * a timer instance provides an XOR input.
  * @rmtoll CR2          TI1S          LL_TIM_IC_DisableXORCombination
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_IC_DisableXORCombination(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->CR2, TIM_CR2_TI1S);
}

/**
  * @brief  Get captured value for input channel 1.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC1_INSTANCE(TIMx) can be used to check whether or not
  * input channel 1 is supported by a timer instance.
  * @rmtoll CCR1         CCR1          LL_TIM_IC_GetCaptureCH1
  * @param  TIMx Timer instance
  * @retval CapturedValue
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCaptureCH1(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR1));
}

/**
  * @brief  Get captured value for input channel 2.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC2_INSTANCE(TIMx) can be used to check whether or not
  * input channel 2 is supported by a timer instance.
  * @rmtoll CCR2         CCR2          LL_TIM_IC_GetCaptureCH2
  * @param  TIMx Timer instance
  * @retval CapturedValue
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCaptureCH2(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR2));
}

/**
  * @brief  Get captured value for input channel 3.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC3_INSTANCE(TIMx) can be used to check whether or not
  * input channel 3 is supported by a timer instance.
  * @rmtoll CCR3         CCR3          LL_TIM_IC_GetCaptureCH3
  * @param  TIMx Timer instance
  * @retval CapturedValue
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCaptureCH3(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR3));
}

/**
  * @brief  Get captured value for input channel 4.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TIM_CC4_INSTANCE(TIMx) can be used to check whether or not
  * input channel 4 is supported by a timer instance.
  * @rmtoll CCR4         CCR4          LL_TIM_IC_GetCaptureCH4
  * @param  TIMx Timer instance
  * @retval CapturedValue
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCaptureCH4(TIM_TypeDef * TIMx)
{
  return (uint32_t)(READ_REG(TIMx->CCR4));
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Clock_Selection Counter clock selection
  * @{
  */
/**
  * @brief  Set the clock source of the counter clock.
  * @ when selected clock source is external clock mode 1, the timer input the external clock is applied is selected by calling the LL_TIM_SetTriggerInput() function. This timer input must be configured by calling the LL_TIM_IC_Config() function.
  * @note Macro IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports external clock mode1.
  * @note Macro IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports external clock mode2.
  * @rmtoll SMCR         SMS           LL_TIM_SetClockSource\n
  *         SMCR         ECE           LL_TIM_SetClockSource\n
  * @param  TIMx Timer instance
  * @param  ClockSource This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKSOURCE_INTERNAL
  *         @arg @ref LL_TIM_CLOCKSOURCE_EXT_MODE1
  *         @arg @ref LL_TIM_CLOCKSOURCE_EXT_MODE2
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetClockSource(TIM_TypeDef * TIMx, uint32_t ClockSource)
{
  MODIFY_REG(TIMx->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE, ClockSource);
}

/**
  * @brief  Set the encoder interface mode.
  * @note Macro IS_TIM_ENCODER_INTERFACE_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance supports the encoder mode.
  * @rmtoll SMCR         SMS           LL_TIM_SetEncoderMode
  * @param  TIMx Timer instance
  * @param  EncoderMode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ENCODERMODE_X2_TI1
  *         @arg @ref LL_TIM_ENCODERMODE_X2_TI2
  *         @arg @ref LL_TIM_ENCODERMODE_X4_TI12
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetEncoderMode(TIM_TypeDef * TIMx, uint32_t EncoderMode)
{
  MODIFY_REG(TIMx->SMCR, TIM_SMCR_SMS, EncoderMode);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Timer_Synchronization timer synchronisation configuration
  * @{
  */
/**
  * @brief  Set the trigger output (TRGO) used for timer synchronization .
  * @note Macro IS_TIM_MASTER_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance can operate as a master timer.
  * @rmtoll CR2          MMS           LL_TIM_SetTriggerOutput
  * @param  TIMx Timer instance
  * @param  TimerSynchronization This parameter can be one of the following values:
  *         @arg @ref LL_TIM_TRGO_RESET
  *         @arg @ref LL_TIM_TRGO_ENABLE
  *         @arg @ref LL_TIM_TRGO_UPDATE
  *         @arg @ref LL_TIM_TRGO_CC1IF
  *         @arg @ref LL_TIM_TRGO_OC1REF
  *         @arg @ref LL_TIM_TRGO_OC2REF
  *         @arg @ref LL_TIM_TRGO_OC3REF
  *         @arg @ref LL_TIM_TRGO_OC4REF
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetTriggerOutput(TIM_TypeDef * TIMx, uint32_t TimerSynchronization)
{
  MODIFY_REG(TIMx->CR2, TIM_CR2_MMS, TimerSynchronization);
}

/**
  * @brief  Set the trigger output 2 (TRGO2) used for ADC synchronization .
  * @note Macro IS_TIM_TRGO2_INSTANCE(TIMx) can be used to check 
  * whether or not a timer instance can be used for ADC synchronization.
  * @rmtoll CR2          MMS2          LL_TIM_SetTriggerOutput2
  * @param  TIMx Timer Instance
  * @param  ADCSynchronization This parameter can be one of the following values:
  *         @arg @ref LL_TIM_TRGO2_RESET
  *         @arg @ref LL_TIM_TRGO2_ENABLE
  *         @arg @ref LL_TIM_TRGO2_UPDATE
  *         @arg @ref LL_TIM_TRGO2_CC1F
  *         @arg @ref LL_TIM_TRGO2_OC1
  *         @arg @ref LL_TIM_TRGO2_OC2
  *         @arg @ref LL_TIM_TRGO2_OC3
  *         @arg @ref LL_TIM_TRGO2_OC4
  *         @arg @ref LL_TIM_TRGO2_OC5
  *         @arg @ref LL_TIM_TRGO2_OC6
  *         @arg @ref LL_TIM_TRGO2_OC4_RISINGFALLING
  *         @arg @ref LL_TIM_TRGO2_OC6_RISINGFALLING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC6_RISING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC6_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC6_RISING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC6_FALLING
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetTriggerOutput2(TIM_TypeDef * TIMx, uint32_t ADCSynchronization)
{
  MODIFY_REG(TIMx->CR2, TIM_CR2_MMS2, ADCSynchronization);
}

/**
  * @brief  Set the synchronization mode of a slave timer.
  * @note Macro IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance can operate as a slave timer.
  * @rmtoll SMCR         SMS           LL_TIM_SetSlaveMode
  * @param  TIMx Timer instance
  * @param  SlaveMode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_SLAVEMODE_DISABLED
  *         @arg @ref LL_TIM_SLAVEMODE_RESET
  *         @arg @ref LL_TIM_SLAVEMODE_GATED
  *         @arg @ref LL_TIM_SLAVEMODE_TRIGGER
  *         @arg @ref LL_TIM_SLAVEMODE_COMBINED_RESETTRIGGER (not supported by STM32F373xC and STM32F378xx devices)
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetSlaveMode(TIM_TypeDef * TIMx, uint32_t SlaveMode)
{
  MODIFY_REG(TIMx->SMCR, TIM_SMCR_SMS, SlaveMode);
}

/**
  * @brief  Set the selects the trigger input to be used to synchronize the counter.
  * @note Macro IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance can operate as a slave timer.
  * @rmtoll SMCR         TS            LL_TIM_SetTriggerInput
  * @param  TIMx Timer instance
  * @param  TriggerInput This parameter can be one of the following values:
  *         @arg @ref LL_TIM_TS_ITR0
  *         @arg @ref LL_TIM_TS_ITR1
  *         @arg @ref LL_TIM_TS_ITR2
  *         @arg @ref LL_TIM_TS_ITR3
  *         @arg @ref LL_TIM_TS_TI1F_ED
  *         @arg @ref LL_TIM_TS_TI1FP1
  *         @arg @ref LL_TIM_TS_TI2FP2
  *         @arg @ref LL_TIM_TS_ETRF
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetTriggerInput(TIM_TypeDef * TIMx, uint32_t TriggerInput)
{
  MODIFY_REG(TIMx->SMCR, TIM_SMCR_TS, TriggerInput);
}

/**
  * @brief  Enable the Master/Slave mode.
  * @note Macro IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance can operate as a slave timer.
  * @rmtoll SMCR         MSM           LL_TIM_EnableMasterSlaveMode
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableMasterSlaveMode(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->SMCR, TIM_SMCR_MSM);
}

/**
  * @brief  Disable the Master/Slave mode.
  * @note Macro IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance can operate as a slave timer.
  * @rmtoll SMCR         MSM           LL_TIM_DisableMasterSlaveMode
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableMasterSlaveMode(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->SMCR, TIM_SMCR_MSM);
}

/**
  * @brief  Configure the external trigger (ETR) input.
  * @note Macro IS_TIM_ETR_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides an external trigger input.
  * @rmtoll SMCR         ETP           LL_TIM_ConfigETR\n
  *         SMCR         ETPS          LL_TIM_ConfigETR\n
  *         SMCR         ETF           LL_TIM_ConfigETR
  * @param  TIMx Timer instance
  * @param  ETRPolarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ETR_POLARITY_NONINVERTED
  *         @arg @ref LL_TIM_ETR_POLARITY_INVERTED
  * @param  ETRPrescaler This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV1
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV2
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV4
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV8
  * @param  ETRFilter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N8
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ConfigETR(TIM_TypeDef * TIMx, uint32_t ETRPolarity, uint32_t ETRPrescaler, uint32_t ETRFilter)
{
  MODIFY_REG(TIMx->SMCR, TIM_SMCR_ETP | TIM_SMCR_ETPS | TIM_SMCR_ETF, ETRPolarity | ETRPrescaler | ETRFilter);
}

/**
  * @brief  Select the external trigger (ETR) input source.
  * @note Macro IS_TIM_ETRSEL_INSTANCE(TIMx) can be used to check whether or
  * not a timer instance supports ETR source selection.
  * @rmtoll OR2          ETRSEL        LL_TIM_SetETRSource
  * @param  TIMx Timer instance
  * @param  ETRSource This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ETRSOURCE_LEGACY
  *         @arg @ref LL_TIM_ETRSOURCE_COMP1
  *         @arg @ref LL_TIM_ETRSOURCE_COMP2
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetETRSource(TIM_TypeDef * TIMx, uint32_t ETRSource)
{

  MODIFY_REG(TIMx->OR2, TIMx_OR2_ETRSEL, ETRSource);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Break_Function Break function configuration
  * @{
  */
/**
  * @brief  Enable the break function.
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @rmtoll BDTR         BKE           LL_TIM_EnableBRK
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableBRK(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->BDTR, TIM_BDTR_BKE);
}

/**
  * @brief  Disable the break function.
  * @rmtoll BDTR         BKE           LL_TIM_DisableBRK
  * @param  TIMx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableBRK(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->BDTR, TIM_BDTR_BKE);
}

/**
  * @brief  Configure the break input.
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @param  TIMx Timer instance
  * @param  BreakPolarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK_POLARITY_HIGH
  * @param  BreakFilter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N8
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ConfigBRK(TIM_TypeDef * TIMx, uint32_t BreakPolarity, uint32_t BreakFilter)
{
  MODIFY_REG(TIMx->BDTR, TIM_BDTR_BKP | TIM_BDTR_BKF, BreakPolarity | BreakFilter);
}

/**
  * @brief  Enable the break 2 function.
  * @note Macro IS_TIM_BKIN2_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a second break input.
  * @rmtoll BDTR         BK2E          LL_TIM_EnableBRK2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableBRK2(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->BDTR, TIM_BDTR_BK2E);
}

/**
  * @brief  Disable the break  2 function.
  * @note Macro IS_TIM_BKIN2_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a second break input.
  * @rmtoll BDTR         BK2E          LL_TIM_DisableBRK2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableBRK2(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->BDTR, TIM_BDTR_BK2E);
}

/**
  * @brief  Configure the break 2 input.
  * @note Macro IS_TIM_BKIN2_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a second break input.
  * @rmtoll BDTR         BK2P          LL_TIM_ConfigBRK2\n
  *         BDTR         BK2F          LL_TIM_ConfigBRK2
  * @param  TIMx Timer instance
  * @param  Break2Polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK2_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK2_POLARITY_HIGH
  * @param  Break2Filter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N8
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ConfigBRK2(TIM_TypeDef * TIMx, uint32_t Break2Polarity, uint32_t Break2Filter)
{
  MODIFY_REG(TIMx->BDTR, TIM_BDTR_BK2P | TIM_BDTR_BK2F, Break2Polarity | Break2Filter);
}

/**
  * @brief  Select the outputs off state (enabled v.s. disabled) in Idle and Run modes.
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @rmtoll BDTR         OSSI          LL_TIM_SetOffStates\n
  *         BDTR         OSSR          LL_TIM_SetOffStates
  * @param  TIMx Timer instance
  * @param  OffStateIdle This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OSSI_DISABLE
  *         @arg @ref LL_TIM_OSSI_ENABLE
  * @param  OffStateRun This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OSSR_DISABLE
  *         @arg @ref LL_TIM_OSSR_ENABLE
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetOffStates(TIM_TypeDef * TIMx, uint32_t OffStateIdle, uint32_t OffStateRun)
{
  MODIFY_REG(TIMx->BDTR, TIM_BDTR_OSSI | TIM_BDTR_OSSR, OffStateIdle | OffStateRun);
}

/**
  * @brief  Enable automatic output (MOE can be set by software or automatically when a break input is active).
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @rmtoll BDTR         AOE           LL_TIM_EnableAutomaticOutput
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableAutomaticOutput(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->BDTR, TIM_BDTR_AOE);
}

/**
  * @brief  Disable automatic output (MOE can be set only by software).
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @rmtoll BDTR         AOE           LL_TIM_DisableAutomaticOutput
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableAutomaticOutput(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->BDTR, TIM_BDTR_AOE);
}

/**
  * @brief  Indicate whether automatic output is enabled.
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @rmtoll BDTR         AOE           LL_TIM_IsEnabledAutomaticOutput
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledAutomaticOutput(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->BDTR, TIM_BDTR_AOE) == (TIM_BDTR_AOE));
}

/**
  * @brief  Enable the outputs (set the MOE bit in TIMx_BDTR register).
  * @note The MOE bit in TIMx_BDTR register allows to enable /disable the outputs by
  * software and is reset in case of break or break2 event
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @rmtoll BDTR         MOE           LL_TIM_EnableAllOutputs
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableAllOutputs(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->BDTR, TIM_BDTR_MOE);
}

/**
  * @brief  Disable the outputs (reset the MOE bit in TIMx_BDTR register).
  * @note The MOE bit in TIMx_BDTR register allows to enable /disable the outputs by
  * software and is reset in case of break or break2 event.
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @rmtoll BDTR         MOE           LL_TIM_DisableAllOutputs
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableAllOutputs(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->BDTR, TIM_BDTR_MOE);
}

/**
  * @brief  Indicates whether outputs are enabled.
  * @note Macro IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not 
  * a timer instance provides a break input.
  * @rmtoll BDTR         MOE           LL_TIM_IsEnabledAllOutputs
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledAllOutputs(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->BDTR, TIM_BDTR_MOE) == (TIM_BDTR_MOE));
}

/**
  * @brief  Enable the signals connected to the designated timer break input.
  * @note Macro IS_TIM_BREAKSOURCE_INSTANCE(TIMx) can be used to check whether
  * or not a timer instance allows for break input selection.
  * @rmtoll OR2          BKINE         LL_TIM_EnableBreakInputSource\n
  *         OR2          BKCMP1E       LL_TIM_EnableBreakInputSource\n
  *         OR2          BKCMP2E       LL_TIM_EnableBreakInputSource\n
  *         OR2          BKDFBK0E      LL_TIM_EnableBreakInputSource\n
  *         OR3          BKINE         LL_TIM_EnableBreakInputSource\n
  *         OR3          BKCMP1E       LL_TIM_EnableBreakInputSource\n
  *         OR3          BKCMP2E       LL_TIM_EnableBreakInputSource\n
  *         OR3          BKDFBK0E      LL_TIM_EnableBreakInputSource
  * @param  TIMx Timer instance
  * @param  BreakInput This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_BKIN
  *         @arg @ref LL_TIM_BREAK_INPUT_BKIN2
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKIN
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKCOMP1
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKCOMP2
  *         @arg @ref LL_TIM_BKIN_SOURCE_DFBK
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableBreakInputSource(TIM_TypeDef * TIMx, uint32_t BreakInput, uint32_t Source)
{
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->OR2)+ BreakInput));
  SET_BIT(*pReg , Source); 
}

/**
  * @brief  Disable the signals connected to the designated timer break input.
  * @note Macro IS_TIM_BREAKSOURCE_INSTANCE(TIMx) can be used to check whether
  * or not a timer instance allows for break input selection.
  * @rmtoll OR2          BKINE         LL_TIM_DisableBreakInputSource\n
  *         OR2          BKCMP1E       LL_TIM_DisableBreakInputSource\n
  *         OR2          BKCMP2E       LL_TIM_DisableBreakInputSource\n
  *         OR2          BKDFBK0E      LL_TIM_DisableBreakInputSource\n
  *         OR3          BKINE         LL_TIM_DisableBreakInputSource\n
  *         OR3          BKCMP1E       LL_TIM_DisableBreakInputSource\n
  *         OR3          BKCMP2E       LL_TIM_DisableBreakInputSource\n
  *         OR3          BKDFBK0E      LL_TIM_DisableBreakInputSource
  * @param  TIMx Timer instance
  * @param  BreakInput This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_BKIN
  *         @arg @ref LL_TIM_BREAK_INPUT_BKIN2
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKIN
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKCOMP1
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKCOMP2
  *         @arg @ref LL_TIM_BKIN_SOURCE_DFBK
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableBreakInputSource(TIM_TypeDef * TIMx, uint32_t BreakInput, uint32_t Source)
{
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->OR2)+ BreakInput));
  CLEAR_BIT(*pReg, Source); 
}

/**
  * @brief  Set the polarity of the break signal for the timer break input.
  * @note Macro IS_TIM_BREAKSOURCE_INSTANCE(TIMx) can be used to check whether
  * or not a timer instance allows for break input selection.
  * @rmtoll OR2          BKINE         LL_TIM_SetBreakInputSourcePolarity\n
  *         OR2          BKCMP1E       LL_TIM_SetBreakInputSourcePolarity\n
  *         OR2          BKCMP2E       LL_TIM_SetBreakInputSourcePolarity\n
  *         OR2          BKINP         LL_TIM_SetBreakInputSourcePolarity\n
  *         OR3          BKINE         LL_TIM_SetBreakInputSourcePolarity\n
  *         OR3          BKCMP1E       LL_TIM_SetBreakInputSourcePolarity\n
  *         OR3          BKCMP2E       LL_TIM_SetBreakInputSourcePolarity\n
  *         OR3          BKINP         LL_TIM_SetBreakInputSourcePolarity
  * @param  TIMx Timer instance
  * @param  BreakInput This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_BKIN
  *         @arg @ref LL_TIM_BREAK_INPUT_BKIN2
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKIN
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKCOMP1
  *         @arg @ref LL_TIM_BKIN_SOURCE_BKCOMP2
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BKIN_POLARITY_LOW
  *         @arg @ref LL_TIM_BKIN_POLARITY_HIGH
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetBreakInputSourcePolarity(TIM_TypeDef * TIMx, uint32_t BreakInput, uint32_t Source, uint32_t Polarity)
{
  register uint32_t * pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->OR2)+ BreakInput));
  MODIFY_REG(*pReg, (TIMx_OR2_BKINP << (POSITION_VAL(Source))) , (Polarity << (POSITION_VAL(Source))));
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_DMA_Burst_Mode DMA burs mode configuration
  * @{
  */
/**
  * @brief  Configures the timer DMA burst feature.
  * @note Macro IS_TIM_DMABURST_INSTANCE(TIMx) can be used to check whether or
  * not a timer instance supports the DMA burst mode.
  * @rmtoll DCR          DBL           LL_TIM_ConfigDMABurst\n
  *         DCR          DBA           LL_TIM_ConfigDMABurst
  * @param  TIMx Timer instance
  * @param  DMABurstBaseAddress This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SMCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_DIER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_EGR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CNT
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_PSC
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_ARR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_RCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR4
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_BDTR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR5
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR6
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_OR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_OR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_OR3
  * @param  DMABurstLength This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_LENGTH_1TRANSFER
  *         @arg @ref LL_TIM_DMABURST_LENGTH_2TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_3TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_4TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_5TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_6TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_7TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_8TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_9TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_10TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_11TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_12TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_13TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_14TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_15TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_16TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_17TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_18TRANSFERS
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ConfigDMABurst(TIM_TypeDef * TIMx, uint32_t DMABurstBaseAddress, uint32_t DMABurstLength)
{
  MODIFY_REG(TIMx->DCR, TIM_DCR_DBL | TIM_DCR_DBA, DMABurstBaseAddress | DMABurstLength);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Timer_Inputs_Remapping Timer input remapping
  * @{
  */
/**
  * @brief  Remap TIM inputs (input channel, internal/external triggers).
  * @note Macro IS_TIM_REMAP_INSTANCE(TIMx) can be used to check whether or not
  * a some timer inputs can be remapped.
  * @rmtoll TIM1_OR1    ETR_ADC1_RMP      LL_TIM_SetRemap\n
  * @rmtoll TIM1_OR1    ETR_ADC3_RMP      LL_TIM_SetRemap\n
  * @rmtoll TIM1_OR1    TI1_RMP           LL_TIM_SetRemap\n
  * @rmtoll TIM8_OR1    ETR_ADC2_RMP      LL_TIM_SetRemap\n
  * @rmtoll TIM8_OR1    ETR_ADC3_RMP      LL_TIM_SetRemap\n
  * @rmtoll TIM8_OR1    TI1_RMP           LL_TIM_SetRemap\n
  * @rmtoll TIM2_OR1    ITR1_RMP          LL_TIM_SetRemap\n
  * @rmtoll TIM2_OR1    ETR1_RMP          LL_TIM_SetRemap\n
  * @rmtoll TIM2_OR1    TI4_RMP           LL_TIM_SetRemap\n
  * @rmtoll TIM3_OR1    TI1_RMP           LL_TIM_SetRemap\n
  * @rmtoll TIM15_OR1   TI1_RMP           LL_TIM_SetRemap\n
  * @rmtoll TIM15_OR1   ENCODER_MODE      LL_TIM_SetRemap\n
  * @rmtoll TIM16_OR1   TI1_RMP           LL_TIM_SetRemap\n
  * @rmtoll TIM17_OR1   TI1_RMP           LL_TIM_SetRemap
  * @param  TIMx: Timer instance 
  * @param  Remap: This parameter can be one of the following values:
  *
  *         TIM1: any combination of TI1_RMP, ADC3_RMP, ADC1_RMP where
  *            ADC1_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM1_ETR_ADC1_RMP_NC
  *            @arg @ref LL_TIM_TIM1_ETR_ADC1_RMP_AWD1
  *            @arg @ref LL_TIM_TIM1_ETR_ADC1_RMP_AWD2
  *            @arg @ref LL_TIM_TIM1_ETR_ADC1_RMP_AWD3
  *            ADC3_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM1_ETR_ADC3_RMP_NC
  *            @arg @ref LL_TIM_TIM1_ETR_ADC3_RMP_AWD1
  *            @arg @ref LL_TIM_TIM1_ETR_ADC3_RMP_AWD2
  *            @arg @ref LL_TIM_TIM1_ETR_ADC3_RMP_AWD3
  *            TI1_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM1_TI1_RMP_GPIO
  *            @arg @ref LL_TIM_TIM1_TI1_RMP_COMP1
  *
  *         TIM2: any combination of ITR1_RMP, ETR1_RMP, TI4_RMP where
  *            ITR1_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM2_ITR1_RMP_TIM8_TRGO
  *            @arg @ref LL_TIM_TIM2_ITR1_RMP_OTG_FS_SOF
  *            ETR1_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM2_ETR_RMP_GPIO
  *            @arg @ref LL_TIM_TIM2_ETR_RMP_LSE
  *            TI4_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM2_TI4_RMP_GPIO
  *            @arg @ref LL_TIM_TIM2_TI4_RMP_COMP1
  *            @arg @ref LL_TIM_TIM2_TI4_RMP_COMP2
  *            @arg @ref LL_TIM_TIM2_TI4_RMP_COMP1_COMP2
  *
  *         TIM3: one of the following values
  *            @arg @ref LL_TIM_TIM3_TI1_RMP_GPIO
  *            @arg @ref LL_TIM_TIM3_TI1_RMP_COMP1
  *            @arg @ref LL_TIM_TIM3_TI1_RMP_COMP2
  *            @arg @ref LL_TIM_TIM3_TI1_RMP_COMP1_COMP2
  *
  *         TIM8: any combination of TI1_RMP, ADC3_RMP, ADC1_RMP where
  *            ADC1_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM8_ETR_ADC2_RMP_NC
  *            @arg @ref LL_TIM_TIM8_ETR_ADC2_RMP_AWD1
  *            @arg @ref LL_TIM_TIM8_ETR_ADC2_RMP_AWD2
  *            @arg @ref LL_TIM_TIM8_ETR_ADC2_RMP_AWD3
  *            ADC3_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM8_ETR_ADC3_RMP_NC
  *            @arg @ref LL_TIM_TIM8_ETR_ADC3_RMP_AWD1
  *            @arg @ref LL_TIM_TIM8_ETR_ADC3_RMP_AWD2
  *            @arg @ref LL_TIM_TIM8_ETR_ADC3_RMP_AWD3
  *            TI1_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM8_TI1_RMP_GPIO
  *            @arg @ref LL_TIM_TIM8_TI1_RMP_COMP2
  *
  *         TIM15: any combination of TI1_RMP, ENCODER_MODE where
  *            TI1_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM15_TI1_RMP_GPIO
  *            @arg @ref LL_TIM_TIM15_TI1_RMP_LSE
  *            ENCODER_MODE can be one of the following values
  *            @arg @ref LL_TIM_TIM15_ENCODERMODE_NOREDIRECTION
  *            @arg @ref LL_TIM_TIM15_ENCODERMODE_TIM2  
  *            @arg @ref LL_TIM_TIM15_ENCODERMODE_TIM3
  *            @arg @ref LL_TIM_TIM15_ENCODERMODE_TIM4
  *
  *         TIM16: one of the following values
  *            @arg @ref LL_TIM_TIM16_TI1_RMP_GPIO
  *            @arg @ref LL_TIM_TIM16_TI1_RMP_LSI
  *            @arg @ref LL_TIM_TIM16_TI1_RMP_LSE
  *            @arg @ref LL_TIM_TIM16_TI1_RMP_RTC
  *
  *         TIM17: one of the following values
  *            @arg @ref LL_TIM_TIM17_TI1_RMP_GPIO
  *            @arg @ref LL_TIM_TIM17_TI1_RMP_MSI
  *            @arg @ref LL_TIM_TIM17_TI1_RMP_HSE_32
  *            @arg @ref LL_TIM_TIM17_TI1_RMP_MCO
  * @retval None
  */
__STATIC_INLINE void LL_TIM_SetRemap(TIM_TypeDef * TIMx, uint32_t Remap)
{
  MODIFY_REG(TIMx->OR1, (Remap >> TIMx_OR1_RMP_SHIFT), (Remap & TIMx_OR1_RMP_MASK));
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_FLAG_Management FLAG_Management
  * @{
  */
/**
  * @brief  Clear the update interrupt flag (UIF).
  * @rmtoll SR           UIF           LL_TIM_ClearFlag_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_UPDATE(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_UIF));
}

/**
  * @brief  Indicate whether update interrupt flag (UIF) is set (update interrupt is pending).
  * @rmtoll SR           UIF           LL_TIM_IsActiveFlag_UPDATE
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_UPDATE(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_UIF) == (TIM_SR_UIF));
}

/**
  * @brief  Clear the Capture/Compare 1 interrupt flag (CC1F).
  * @rmtoll SR           CC1IF         LL_TIM_ClearFlag_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC1(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC1IF));
}

/**
  * @brief  Indicate whether Capture/Compare 1 interrupt flag (CC1F) is set (Capture/Compare 1 interrupt is pending).
  * @rmtoll SR           CC1IF         LL_TIM_IsActiveFlag_CC1
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC1(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC1IF) == (TIM_SR_CC1IF));
}

/**
  * @brief  Clear the Capture/Compare 2 interrupt flag (CC2F).
  * @rmtoll SR           CC2IF         LL_TIM_ClearFlag_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC2(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC2IF));
}

/**
  * @brief  Indicate whether Capture/Compare 2 interrupt flag (CC2F) is set (Capture/Compare 2 interrupt is pending).
  * @rmtoll SR           CC2IF         LL_TIM_IsActiveFlag_CC2
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC2(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC2IF) == (TIM_SR_CC2IF));
}

/**
  * @brief  Clear the Capture/Compare 3 interrupt flag (CC3F).
  * @rmtoll SR           CC3IF         LL_TIM_ClearFlag_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC3(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC3IF));
}

/**
  * @brief  Indicate whether Capture/Compare 3 interrupt flag (CC3F) is set (Capture/Compare 3 interrupt is pending).
  * @rmtoll SR           CC3IF         LL_TIM_IsActiveFlag_CC3
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC3(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC3IF) == (TIM_SR_CC3IF));
}

/**
  * @brief  Clear the Capture/Compare 4 interrupt flag (CC4F).
  * @rmtoll SR           CC4IF         LL_TIM_ClearFlag_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC4(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC4IF));
}

/**
  * @brief  Indicate whether Capture/Compare 4 interrupt flag (CC4F) is set (Capture/Compare 4 interrupt is pending).
  * @rmtoll SR           CC4IF         LL_TIM_IsActiveFlag_CC4
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC4(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC4IF) == (TIM_SR_CC4IF));
}

/**
  * @brief  Clear the Capture/Compare 5 interrupt flag (CC5F).
  * @rmtoll SR           CC5IF         LL_TIM_ClearFlag_CC5
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC5(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC5IF));
}

/**
  * @brief  Indicate whether Capture/Compare 5 interrupt flag (CC5F) is set (Capture/Compare 5 interrupt is pending).
  * @rmtoll SR           CC5IF         LL_TIM_IsActiveFlag_CC5
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC5(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC5IF) == (TIM_SR_CC5IF));
}
/**
  * @brief  Clear the Capture/Compare 6 interrupt flag (CC6F).
  * @rmtoll SR           CC6IF         LL_TIM_ClearFlag_CC6
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC6(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC6IF));
}

/**
  * @brief  Indicate whether Capture/Compare 6 interrupt flag (CC6F) is set (Capture/Compare 6 interrupt is pending).
  * @rmtoll SR           CC6IF         LL_TIM_IsActiveFlag_CC6
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC6(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC6IF) == (TIM_SR_CC6IF));
}

/**
  * @brief  Clear the commutation interrupt flag (COMIF).
  * @rmtoll SR           COMIF         LL_TIM_ClearFlag_COM
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_COM(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_COMIF));
}

/**
  * @brief  Indicate whether commutation interrupt flag (COMIF) is set (commutation interrupt is pending).
  * @rmtoll SR           COMIF         LL_TIM_IsActiveFlag_COM
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_COM(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_COMIF) == (TIM_SR_COMIF));
}

/**
  * @brief  Clear the trigger interrupt flag (TIF).
  * @rmtoll SR           TIF           LL_TIM_ClearFlag_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_TRIG(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_TIF));
}

/**
  * @brief  Indicate whether trigger interrupt flag (TIF) is set (trigger interrupt is pending).
  * @rmtoll SR           TIF           LL_TIM_IsActiveFlag_TRIG
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_TRIG(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_TIF) == (TIM_SR_TIF));
}

/**
  * @brief  Clear the break interrupt flag (BIF).
  * @rmtoll SR           BIF           LL_TIM_ClearFlag_BRK
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_BRK(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_BIF));
}

/**
  * @brief  Indicate whether break interrupt flag (BIF) is set (break interrupt is pending).
  * @rmtoll SR           BIF           LL_TIM_IsActiveFlag_BRK
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_BRK(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_BIF) == (TIM_SR_BIF));
}

/**
  * @brief  Clear the break 2 interrupt flag (B2IF).
  * @rmtoll SR           B2IF          LL_TIM_ClearFlag_BRK2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_BRK2(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_B2IF));
}

/**
  * @brief  Indicate whether break 2 interrupt flag (B2IF) is set (break 2 interrupt is pending).
  * @rmtoll SR           B2IF          LL_TIM_IsActiveFlag_BRK2
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_BRK2(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_B2IF) == (TIM_SR_B2IF));
}

/**
  * @brief  Clear the Capture/Compare 1 over-capture interrupt flag (CC1OF).
  * @rmtoll SR           CC1OF         LL_TIM_ClearFlag_CC1OVR
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC1OVR(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC1OF));
}

/**
  * @brief  Indicate whether Capture/Compare 1 over-capture interrupt flag (CC1OF) is set (Capture/Compare 1 interrupt is pending).
  * @rmtoll SR           CC1OF         LL_TIM_IsActiveFlag_CC1OVR
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC1OVR(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC1OF) == (TIM_SR_CC1OF));
}

/**
  * @brief  Clear the Capture/Compare 2 over-capture interrupt flag (CC2OF).
  * @rmtoll SR           CC2OF         LL_TIM_ClearFlag_CC2OVR
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC2OVR(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC2OF));
}

/**
  * @brief  Indicate whether Capture/Compare 2 over-capture interrupt flag (CC2OF) is set (Capture/Compare 2 over-capture interrupt is pending).
  * @rmtoll SR           CC2OF         LL_TIM_IsActiveFlag_CC2OVR
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC2OVR(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC2OF) == (TIM_SR_CC2OF));
}

/**
  * @brief  Clear the Capture/Compare 3 over-capture interrupt flag (CC3OF).
  * @rmtoll SR           CC3OF         LL_TIM_ClearFlag_CC3OVR
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC3OVR(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC3OF));
}

/**
  * @brief  Indicate whether Capture/Compare 3 over-capture interrupt flag (CC3OF) is set (Capture/Compare 3 over-capture interrupt is pending).
  * @rmtoll SR           CC3OF         LL_TIM_IsActiveFlag_CC3OVR
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC3OVR(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC3OF) == (TIM_SR_CC3OF));
}

/**
  * @brief  Clear the Capture/Compare 4 over-capture interrupt flag (CC4OF).
  * @rmtoll SR           CC4OF         LL_TIM_ClearFlag_CC4OVR
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC4OVR(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_CC4OF));
}

/**
  * @brief  Indicate whether Capture/Compare 4 over-capture interrupt flag (CC4OF) is set (Capture/Compare 4 over-capture interrupt is pending).
  * @rmtoll SR           CC4OF         LL_TIM_IsActiveFlag_CC4OVR
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC4OVR(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_CC4OF) == (TIM_SR_CC4OF));
}

/**
  * @brief  Clear the system break interrupt flag (SBIF).
  * @rmtoll SR           SBIF          LL_TIM_ClearFlag_SYSBRK
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_ClearFlag_SYSBRK(TIM_TypeDef * TIMx)
{
  WRITE_REG(TIMx->SR, ~(TIM_SR_SBIF));
}

/**
  * @brief  Indicate whether system break interrupt flag (SBIF) is set (system break interrupt is pending).
  * @rmtoll SR           SBIF          LL_TIM_IsActiveFlag_SYSBRK
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_SYSBRK(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_SBIF) == (TIM_SR_SBIF));
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_IT_Management IT_Management
  * @{
  */
/**
  * @brief  Enable update interrupt (UIE).
  * @rmtoll DIER         UIE           LL_TIM_EnableIT_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableIT_UPDATE(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_UIE);
}

/**
  * @brief  Disable update interrupt (UIE).
  * @rmtoll DIER         UIE           LL_TIM_DisableIT_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableIT_UPDATE(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_UIE);
}

/**
  * @brief  Indicates whether the update interrupt (UIE) is enabled.
  * @rmtoll DIER         UIE           LL_TIM_IsEnabledIT_UPDATE
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_UPDATE(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_UIE) == (TIM_DIER_UIE));
}

/**
  * @brief  Enable capture/compare 1 interrupt (CC1IE).
  * @rmtoll DIER         CC1IE         LL_TIM_EnableIT_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableIT_CC1(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_CC1IE);
}

/**
  * @brief  Disable capture/compare 1  interrupt (CC1IE).
  * @rmtoll DIER         CC1IE         LL_TIM_DisableIT_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableIT_CC1(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_CC1IE);
}

/**
  * @brief  Indicates whether the capture/compare 1 interrupt (CC1IE) is enabled.
  * @rmtoll DIER         CC1IE         LL_TIM_IsEnabledIT_CC1
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_CC1(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_CC1IE) == (TIM_DIER_CC1IE));
}

/**
  * @brief  Enable capture/compare 2 interrupt (CC2IE).
  * @rmtoll DIER         CC2IE         LL_TIM_EnableIT_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableIT_CC2(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_CC2IE);
}

/**
  * @brief  Disable capture/compare 2  interrupt (CC2IE).
  * @rmtoll DIER         CC2IE         LL_TIM_DisableIT_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableIT_CC2(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_CC2IE);
}

/**
  * @brief  Indicates whether the capture/compare 2 interrupt (CC2IE) is enabled.
  * @rmtoll DIER         CC2IE         LL_TIM_IsEnabledIT_CC2
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_CC2(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_CC2IE) == (TIM_DIER_CC2IE));
}

/**
  * @brief  Enable capture/compare 3 interrupt (CC3IE).
  * @rmtoll DIER         CC3IE         LL_TIM_EnableIT_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableIT_CC3(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_CC3IE);
}

/**
  * @brief  Disable capture/compare 3  interrupt (CC3IE).
  * @rmtoll DIER         CC3IE         LL_TIM_DisableIT_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableIT_CC3(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_CC3IE);
}

/**
  * @brief  Indicates whether the capture/compare 3 interrupt (CC3IE) is enabled.
  * @rmtoll DIER         CC3IE         LL_TIM_IsEnabledIT_CC3
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_CC3(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_CC3IE) == (TIM_DIER_CC3IE));
}

/**
  * @brief  Enable capture/compare 4 interrupt (CC4IE).
  * @rmtoll DIER         CC4IE         LL_TIM_EnableIT_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableIT_CC4(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_CC4IE);
}

/**
  * @brief  Disable capture/compare 4  interrupt (CC4IE).
  * @rmtoll DIER         CC4IE         LL_TIM_DisableIT_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableIT_CC4(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_CC4IE);
}

/**
  * @brief  Indicates whether the capture/compare 4 interrupt (CC4IE) is enabled.
  * @rmtoll DIER         CC4IE         LL_TIM_IsEnabledIT_CC4
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_CC4(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_CC4IE) == (TIM_DIER_CC4IE));
}

/**
  * @brief  Enable commutation interrupt (COMIE).
  * @rmtoll DIER         COMIE         LL_TIM_EnableIT_COM
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableIT_COM(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_COMIE);
}

/**
  * @brief  Disable commutation interrupt (COMIE).
  * @rmtoll DIER         COMIE         LL_TIM_DisableIT_COM
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableIT_COM(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_COMIE);
}

/**
  * @brief  Indicates whether the commutation interrupt (COMIE) is enabled.
  * @rmtoll DIER         COMIE         LL_TIM_IsEnabledIT_COM
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_COM(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_COMIE) == (TIM_DIER_COMIE));
}

/**
  * @brief  Enable trigger interrupt (TIE).
  * @rmtoll DIER         TIE           LL_TIM_EnableIT_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableIT_TRIG(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_TIE);
}

/**
  * @brief  Disable trigger interrupt (TIE).
  * @rmtoll DIER         TIE           LL_TIM_DisableIT_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableIT_TRIG(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_TIE);
}

/**
  * @brief  Indicates whether the trigger interrupt (TIE) is enabled.
  * @rmtoll DIER         TIE           LL_TIM_IsEnabledIT_TRIG
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_TRIG(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_TIE) == (TIM_DIER_TIE));
}

/**
  * @brief  Enable break interrupt (BIE).
  * @rmtoll DIER         BIE           LL_TIM_EnableIT_BRK
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableIT_BRK(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_BIE);
}

/**
  * @brief  Disable break interrupt (BIE).
  * @rmtoll DIER         BIE           LL_TIM_DisableIT_BRK
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableIT_BRK(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_BIE);
}

/**
  * @brief  Indicates whether the break interrupt (BIE) is enabled.
  * @rmtoll DIER         BIE           LL_TIM_IsEnabledIT_BRK
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_BRK(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_BIE) == (TIM_DIER_BIE));
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_DMA_Management DMA_Management
  * @{
  */
/**
  * @brief  Enable update DMA request (UDE).
  * @rmtoll DIER         UDE           LL_TIM_EnableDMAReq_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_UPDATE(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_UDE);
}

/**
  * @brief  Disable update DMA request (UDE).
  * @rmtoll DIER         UDE           LL_TIM_DisableDMAReq_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_UPDATE(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_UDE);
}

/**
  * @brief  Indicates whether the update DMA request  (UDE) is enabled.
  * @rmtoll DIER         UDE           LL_TIM_IsEnabledDMAReq_UPDATE
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_UPDATE(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_UDE) == (TIM_DIER_UDE));
}

/**
  * @brief  Enable capture/compare 1 DMA request (CC1DE).
  * @rmtoll DIER         CC1DE         LL_TIM_EnableDMAReq_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_CC1(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_CC1DE);
}

/**
  * @brief  Disable capture/compare 1  DMA request (CC1DE).
  * @rmtoll DIER         CC1DE         LL_TIM_DisableDMAReq_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_CC1(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_CC1DE);
}

/**
  * @brief  Indicates whether the capture/compare 1 DMA request (CC1DE) is enabled.
  * @rmtoll DIER         CC1DE         LL_TIM_IsEnabledDMAReq_CC1
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_CC1(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_CC1DE) == (TIM_DIER_CC1DE));
}

/**
  * @brief  Enable capture/compare 2 DMA request (CC2DE).
  * @rmtoll DIER         CC2DE         LL_TIM_EnableDMAReq_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_CC2(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_CC2DE);
}

/**
  * @brief  Disable capture/compare 2  DMA request (CC2DE).
  * @rmtoll DIER         CC2DE         LL_TIM_DisableDMAReq_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_CC2(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_CC2DE);
}

/**
  * @brief  Indicates whether the capture/compare 2 DMA request (CC2DE) is enabled.
  * @rmtoll DIER         CC2DE         LL_TIM_IsEnabledDMAReq_CC2
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_CC2(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_CC2DE) == (TIM_DIER_CC2DE));
}

/**
  * @brief  Enable capture/compare 3 DMA request (CC3DE).
  * @rmtoll DIER         CC3DE         LL_TIM_EnableDMAReq_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_CC3(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_CC3DE);
}

/**
  * @brief  Disable capture/compare 3  DMA request (CC3DE).
  * @rmtoll DIER         CC3DE         LL_TIM_DisableDMAReq_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_CC3(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_CC3DE);
}

/**
  * @brief  Indicates whether the capture/compare 3 DMA request (CC3DE) is enabled.
  * @rmtoll DIER         CC3DE         LL_TIM_IsEnabledDMAReq_CC3
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_CC3(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_CC3DE) == (TIM_DIER_CC3DE));
}

/**
  * @brief  Enable capture/compare 4 DMA request (CC4DE).
  * @rmtoll DIER         CC4DE         LL_TIM_EnableDMAReq_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_CC4(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_CC4DE);
}

/**
  * @brief  Disable capture/compare 4  DMA request (CC4DE).
  * @rmtoll DIER         CC4DE         LL_TIM_DisableDMAReq_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_CC4(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_CC4DE);
}

/**
  * @brief  Indicates whether the capture/compare 4 DMA request (CC4DE) is enabled.
  * @rmtoll DIER         CC4DE         LL_TIM_IsEnabledDMAReq_CC4
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_CC4(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_CC4DE) == (TIM_DIER_CC4DE));
}

/**
  * @brief  Enable commutation DMA request (COMDE).
  * @rmtoll DIER         COMDE         LL_TIM_EnableDMAReq_COM
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_COM(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_COMDE);
}

/**
  * @brief  Disable commutation DMA request (COMDE).
  * @rmtoll DIER         COMDE         LL_TIM_DisableDMAReq_COM
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_COM(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_COMDE);
}

/**
  * @brief  Indicates whether the commutation DMA request (COMDE) is enabled.
  * @rmtoll DIER         COMDE         LL_TIM_IsEnabledDMAReq_COM
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_COM(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_COMDE) == (TIM_DIER_COMDE));
}

/**
  * @brief  Enable trigger interrupt (TDE).
  * @rmtoll DIER         TDE           LL_TIM_EnableDMAReq_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_TRIG(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->DIER, TIM_DIER_TDE);
}

/**
  * @brief  Disable trigger interrupt (TDE).
  * @rmtoll DIER         TDE           LL_TIM_DisableDMAReq_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_TRIG(TIM_TypeDef * TIMx)
{
  CLEAR_BIT(TIMx->DIER, TIM_DIER_TDE);
}

/**
  * @brief  Indicates whether the trigger interrupt (TDE) is enabled.
  * @rmtoll DIER         TDE           LL_TIM_IsEnabledDMAReq_TRIG
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_TRIG(TIM_TypeDef * TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_TDE) == (TIM_DIER_TDE));
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_EVENT_Management EVENT_Management
  * @{
  */
/**
  * @brief  Generate an update event.
  * @rmtoll EGR          UG            LL_TIM_GenerateEvent_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_UPDATE(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_UG);
}

/**
  * @brief  Generate Capture/Compare 1 event.
  * @rmtoll EGR          CC1G          LL_TIM_GenerateEvent_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_CC1(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_CC1G);
}

/**
  * @brief  Generate Capture/Compare 2 event.
  * @rmtoll EGR          CC2G          LL_TIM_GenerateEvent_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_CC2(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_CC2G);
}

/**
  * @brief  Generate Capture/Compare 3 event.
  * @rmtoll EGR          CC3G          LL_TIM_GenerateEvent_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_CC3(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_CC3G);
}

/**
  * @brief  Generate Capture/Compare 4 event.
  * @rmtoll EGR          CC4G          LL_TIM_GenerateEvent_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_CC4(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_CC4G);
}

/**
  * @brief  Generate commutation event.
  * @rmtoll EGR          COMG          LL_TIM_GenerateEvent_COM
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_COM(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_COMG);
}

/**
  * @brief  Generate trigger event.
  * @rmtoll EGR          TG            LL_TIM_GenerateEvent_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_TRIG(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_TG);
}

/**
  * @brief  Generate break event.
  * @rmtoll EGR          BG            LL_TIM_GenerateEvent_BRK
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_BRK(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_BG);
}

/**
  * @brief  Generate break 2 event.
  * @rmtoll EGR          B2G           LL_TIM_GenerateEvent_BRK2
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_BRK2(TIM_TypeDef * TIMx)
{
  SET_BIT(TIMx->EGR, TIM_EGR_B2G);
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

#endif /* TIM1 || TIM8 || TIM2 || TIM3 || TIM4 || TIM5 || TIM15 || TIM16 || TIM17 || TIM6 || TIM7 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_LL_TIM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

