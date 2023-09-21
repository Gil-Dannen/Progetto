/**************************************************************************//**
 * @file     core_armv8mml.h
 * @brief    CMSIS Armv8-M Mainline Core Peripheral Access Layer Header File
 * @version  V5.1.0
 * @date     12. September 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if   defined ( __ICCARM__ )
  #pragma system_include
#elif defined (__clang__)
  #pragma clang system_header
#endif

#ifndef __CORE_ARMV8MML_H_GENERIC
#define __CORE_ARMV8MML_H_GENERIC

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

/**
  \page CMSIS_MISRA_Exceptions  MISRA-C:2004 Compliance Exceptions
  CMSIS violates the following MISRA-C:2004 rules:

   \li Required Rule 8.5, object/function definition in header file.<br>
     Function definitions in header files are used to allow 'inlining'.

   \li Required Rule 18.4, declaration of union type or object of union type: '{...}'.<br>
     Unions are used for effective representation of core registers.

   \li Advisory Rule 19.7, Function-like macro defined.<br>
     Function-like macros are used to allow more efficient code.
 */


/*******************************************************************************
 *                 CMSIS definitions
 ******************************************************************************/
/**
  \ingroup Cortex_ARMv8MML
  @{
 */

#include "cmsis_version.h"


#define __ARMv8MML_CMSIS_VERSION_MAIN  (__CM_CMSIS_VERSION_MAIN)
#define __ARMv8MML_CMSIS_VERSION_SUB   (__CM_CMSIS_VERSION_SUB)
#define __ARMv8MML_CMSIS_VERSION       ((__ARMv8MML_CMSIS_VERSION_MAIN << 16U) | \
                                         __ARMv8MML_CMSIS_VERSION_SUB           )

#define __CORTEX_M                     (81U)

/** __FPU_USED indicates whether an FPU is used or not.
    For this, __FPU_PRESENT has to be checked prior to making use of FPU specific registers and functions.
*/
#if defined ( __CC_ARM )
  #if defined __TARGET_FPU_VFP
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

  #if defined(__ARM_FEATURE_DSP)
    #if defined(__DSP_PRESENT) && (__DSP_PRESENT == 1U)
      #define __DSP_USED       1U
    #else
      #error "Compiler generates DSP (SIMD) instructions for a devices without DSP extensions (check __DSP_PRESENT)"
      #define __DSP_USED         0U
    #endif
  #else
    #define __DSP_USED         0U
  #endif

#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #if defined __ARM_FP
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #warning "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

  #if defined(__ARM_FEATURE_DSP)
    #if defined(__DSP_PRESENT) && (__DSP_PRESENT == 1U)
      #define __DSP_USED       1U
    #else
      #error "Compiler generates DSP (SIMD) instructions for a devices without DSP extensions (check __DSP_PRESENT)"
      #define __DSP_USED         0U
    #endif
  #else
    #define __DSP_USED         0U
  #endif

#elif defined ( __GNUC__ )
  #if defined (__VFP_FP__) && !defined(__SOFTFP__)
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

  #if defined(__ARM_FEATURE_DSP)
    #if defined(__DSP_PRESENT) && (__DSP_PRESENT == 1U)
      #define __DSP_USED       1U
    #else
      #error "Compiler generates DSP (SIMD) instructions for a devices without DSP extensions (check __DSP_PRESENT)"
      #define __DSP_USED         0U
    #endif
  #else
    #define __DSP_USED         0U
  #endif

#elif defined ( __ICCARM__ )
  #if defined __ARMVFP__
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

  #if defined(__ARM_FEATURE_DSP)
    #if defined(__DSP_PRESENT) && (__DSP_PRESENT == 1U)
      #define __DSP_USED       1U
    #else
      #error "Compiler generates DSP (SIMD) instructions for a devices without DSP extensions (check __DSP_PRESENT)"
      #define __DSP_USED         0U
    #endif
  #else
    #define __DSP_USED         0U
  #endif

#elif defined ( __TI_ARM__ )
  #if defined __TI_VFP_SUPPORT__
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

#elif defined ( __TASKING__ )
  #if defined __FPU_VFP__
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

#elif defined ( __CSMC__ )
  #if ( __CSMC__ & 0x400U)
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

#endif

#include "cmsis_compiler.h"


#ifdef __cplusplus
}
#endif

#endif

#ifndef __CMSIS_GENERIC

#ifndef __CORE_ARMV8MML_H_DEPENDANT
#define __CORE_ARMV8MML_H_DEPENDANT

#ifdef __cplusplus
 extern "C" {
#endif


#if defined __CHECK_DEVICE_DEFINES
  #ifndef __ARMv8MML_REV
    #define __ARMv8MML_REV               0x0000U
    #warning "__ARMv8MML_REV not defined in device header file; using default!"
  #endif

  #ifndef __FPU_PRESENT
    #define __FPU_PRESENT             0U
    #warning "__FPU_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __MPU_PRESENT
    #define __MPU_PRESENT             0U
    #warning "__MPU_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __SAUREGION_PRESENT
    #define __SAUREGION_PRESENT       0U
    #warning "__SAUREGION_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __DSP_PRESENT
    #define __DSP_PRESENT             0U
    #warning "__DSP_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __NVIC_PRIO_BITS
    #define __NVIC_PRIO_BITS          3U
    #warning "__NVIC_PRIO_BITS not defined in device header file; using default!"
  #endif

  #ifndef __Vendor_SysTickConfig
    #define __Vendor_SysTickConfig    0U
    #warning "__Vendor_SysTickConfig not defined in device header file; using default!"
  #endif
#endif


/**
    \defgroup CMSIS_glob_defs CMSIS Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.
*/
#ifdef __cplusplus
  #define   __I     volatile
#else
  #define   __I     volatile const
#endif
#define     __O     volatile
#define     __IO    volatile


#define     __IM     volatile const
#define     __OM     volatile
#define     __IOM    volatile





/*******************************************************************************
 *                 Register Abstraction
  Core Register contain:
  - Core Register
  - Core NVIC Register
  - Core SCB Register
  - Core SysTick Register
  - Core Debug Register
  - Core MPU Register
  - Core SAU Register
  - Core FPU Register
 ******************************************************************************/
/**
  \defgroup CMSIS_core_register Defines and Type Definitions
  \brief Type definitions and defines for Cortex-M processor based devices.
*/

/**
  \ingroup    CMSIS_core_register
  \defgroup   CMSIS_CORE  Status and Control Registers
  \brief      Core Register type definitions.
  @{
 */

/**
  \brief  Union type to access the Application Program Status Register (APSR).
 */
typedef union
{
  struct
  {
    uint32_t _reserved0:16;
    uint32_t GE:4;
    uint32_t _reserved1:7;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;


#define APSR_N_Pos                         31U
#define APSR_N_Msk                         (1UL << APSR_N_Pos)

#define APSR_Z_Pos                         30U
#define APSR_Z_Msk                         (1UL << APSR_Z_Pos)

#define APSR_C_Pos                         29U
#define APSR_C_Msk                         (1UL << APSR_C_Pos)

#define APSR_V_Pos                         28U
#define APSR_V_Msk                         (1UL << APSR_V_Pos)

#define APSR_Q_Pos                         27U
#define APSR_Q_Msk                         (1UL << APSR_Q_Pos)

#define APSR_GE_Pos                        16U
#define APSR_GE_Msk                        (0xFUL << APSR_GE_Pos)


/**
  \brief  Union type to access the Interrupt Program Status Register (IPSR).
 */
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;


#define IPSR_ISR_Pos                        0U
#define IPSR_ISR_Msk                       (0x1FFUL


/**
  \brief  Union type to access the Special-Purpose Program Status Registers (xPSR).
 */
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:7;
    uint32_t GE:4;
    uint32_t _reserved1:4;
    uint32_t T:1;
    uint32_t IT:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;


#define xPSR_N_Pos                         31U
#define xPSR_N_Msk                         (1UL << xPSR_N_Pos)

#define xPSR_Z_Pos                         30U
#define xPSR_Z_Msk                         (1UL << xPSR_Z_Pos)

#define xPSR_C_Pos                         29U
#define xPSR_C_Msk                         (1UL << xPSR_C_Pos)

#define xPSR_V_Pos                         28U
#define xPSR_V_Msk                         (1UL << xPSR_V_Pos)

#define xPSR_Q_Pos                         27U
#define xPSR_Q_Msk                         (1UL << xPSR_Q_Pos)

#define xPSR_IT_Pos                        25U
#define xPSR_IT_Msk                        (3UL << xPSR_IT_Pos)

#define xPSR_T_Pos                         24U
#define xPSR_T_Msk                         (1UL << xPSR_T_Pos)

#define xPSR_GE_Pos                        16U
#define xPSR_GE_Msk                        (0xFUL << xPSR_GE_Pos)

#define xPSR_ISR_Pos                        0U
#define xPSR_ISR_Msk                       (0x1FFUL


/**
  \brief  Union type to access the Control Registers (CONTROL).
 */
typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t SFPA:1;
    uint32_t _reserved1:28;
  } b;
  uint32_t w;
} CONTROL_Type;


#define CONTROL_SFPA_Pos                    3U
#define CONTROL_SFPA_Msk                   (1UL << CONTROL_SFPA_Pos)

#define CONTROL_FPCA_Pos                    2U
#define CONTROL_FPCA_Msk                   (1UL << CONTROL_FPCA_Pos)

#define CONTROL_SPSEL_Pos                   1U
#define CONTROL_SPSEL_Msk                  (1UL << CONTROL_SPSEL_Pos)

#define CONTROL_nPRIV_Pos                   0U
#define CONTROL_nPRIV_Msk                  (1UL




/**
  \ingroup    CMSIS_core_register
  \defgroup   CMSIS_NVIC  Nested Vectored Interrupt Controller (NVIC)
  \brief      Type definitions for the NVIC Registers
  @{
 */

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IOM uint32_t ISER[16U];
        uint32_t RESERVED0[16U];
  __IOM uint32_t ICER[16U];
        uint32_t RSERVED1[16U];
  __IOM uint32_t ISPR[16U];
        uint32_t RESERVED2[16U];
  __IOM uint32_t ICPR[16U];
        uint32_t RESERVED3[16U];
  __IOM uint32_t IABR[16U];
        uint32_t RESERVED4[16U];
  __IOM uint32_t ITNS[16U];
        uint32_t RESERVED5[16U];
  __IOM uint8_t  IPR[496U];
        uint32_t RESERVED6[580U];
  __OM  uint32_t STIR;
}  NVIC_Type;


#define NVIC_STIR_INTID_Pos                 0U
#define NVIC_STIR_INTID_Msk                (0x1FFUL




/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SCB     System Control Block (SCB)
  \brief    Type definitions for the System Control Block Registers
  @{
 */

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __IM  uint32_t CPUID;
  __IOM uint32_t ICSR;
  __IOM uint32_t VTOR;
  __IOM uint32_t AIRCR;
  __IOM uint32_t SCR;
  __IOM uint32_t CCR;
  __IOM uint8_t  SHPR[12U];
  __IOM uint32_t SHCSR;
  __IOM uint32_t CFSR;
  __IOM uint32_t HFSR;
  __IOM uint32_t DFSR;
  __IOM uint32_t MMFAR;
  __IOM uint32_t BFAR;
  __IOM uint32_t AFSR;
  __IM  uint32_t ID_PFR[2U];
  __IM  uint32_t ID_DFR;
  __IM  uint32_t ID_ADR;
  __IM  uint32_t ID_MMFR[4U];
  __IM  uint32_t ID_ISAR[6U];
  __IM  uint32_t CLIDR;
  __IM  uint32_t CTR;
  __IM  uint32_t CCSIDR;
  __IOM uint32_t CSSELR;
  __IOM uint32_t CPACR;
  __IOM uint32_t NSACR;
        uint32_t RESERVED3[92U];
  __OM  uint32_t STIR;
        uint32_t RESERVED4[15U];
  __IM  uint32_t MVFR0;
  __IM  uint32_t MVFR1;
  __IM  uint32_t MVFR2;
        uint32_t RESERVED5[1U];
  __OM  uint32_t ICIALLU;
        uint32_t RESERVED6[1U];
  __OM  uint32_t ICIMVAU;
  __OM  uint32_t DCIMVAC;
  __OM  uint32_t DCISW;
  __OM  uint32_t DCCMVAU;
  __OM  uint32_t DCCMVAC;
  __OM  uint32_t DCCSW;
  __OM  uint32_t DCCIMVAC;
  __OM  uint32_t DCCISW;
} SCB_Type;


#define SCB_CPUID_IMPLEMENTER_Pos          24U
#define SCB_CPUID_IMPLEMENTER_Msk          (0xFFUL << SCB_CPUID_IMPLEMENTER_Pos)

#define SCB_CPUID_VARIANT_Pos              20U
#define SCB_CPUID_VARIANT_Msk              (0xFUL << SCB_CPUID_VARIANT_Pos)

#define SCB_CPUID_ARCHITECTURE_Pos         16U
#define SCB_CPUID_ARCHITECTURE_Msk         (0xFUL << SCB_CPUID_ARCHITECTURE_Pos)

#define SCB_CPUID_PARTNO_Pos                4U
#define SCB_CPUID_PARTNO_Msk               (0xFFFUL << SCB_CPUID_PARTNO_Pos)

#define SCB_CPUID_REVISION_Pos              0U
#define SCB_CPUID_REVISION_Msk             (0xFUL


#define SCB_ICSR_PENDNMISET_Pos            31U
#define SCB_ICSR_PENDNMISET_Msk            (1UL << SCB_ICSR_PENDNMISET_Pos)

#define SCB_ICSR_NMIPENDSET_Pos            SCB_ICSR_PENDNMISET_Pos
#define SCB_ICSR_NMIPENDSET_Msk            SCB_ICSR_PENDNMISET_Msk

#define SCB_ICSR_PENDNMICLR_Pos            30U
#define SCB_ICSR_PENDNMICLR_Msk            (1UL << SCB_ICSR_PENDNMICLR_Pos)

#define SCB_ICSR_PENDSVSET_Pos             28U
#define SCB_ICSR_PENDSVSET_Msk             (1UL << SCB_ICSR_PENDSVSET_Pos)

#define SCB_ICSR_PENDSVCLR_Pos             27U
#define SCB_ICSR_PENDSVCLR_Msk             (1UL << SCB_ICSR_PENDSVCLR_Pos)

#define SCB_ICSR_PENDSTSET_Pos             26U
#define SCB_ICSR_PENDSTSET_Msk             (1UL << SCB_ICSR_PENDSTSET_Pos)

#define SCB_ICSR_PENDSTCLR_Pos             25U
#define SCB_ICSR_PENDSTCLR_Msk             (1UL << SCB_ICSR_PENDSTCLR_Pos)

#define SCB_ICSR_STTNS_Pos                 24U
#define SCB_ICSR_STTNS_Msk                 (1UL << SCB_ICSR_STTNS_Pos)

#define SCB_ICSR_ISRPREEMPT_Pos            23U
#define SCB_ICSR_ISRPREEMPT_Msk            (1UL << SCB_ICSR_ISRPREEMPT_Pos)

#define SCB_ICSR_ISRPENDING_Pos            22U
#define SCB_ICSR_ISRPENDING_Msk            (1UL << SCB_ICSR_ISRPENDING_Pos)

#define SCB_ICSR_VECTPENDING_Pos           12U
#define SCB_ICSR_VECTPENDING_Msk           (0x1FFUL << SCB_ICSR_VECTPENDING_Pos)

#define SCB_ICSR_RETTOBASE_Pos             11U
#define SCB_ICSR_RETTOBASE_Msk             (1UL << SCB_ICSR_RETTOBASE_Pos)

#define SCB_ICSR_VECTACTIVE_Pos             0U
#define SCB_ICSR_VECTACTIVE_Msk            (0x1FFUL


#define SCB_VTOR_TBLOFF_Pos                 7U
#define SCB_VTOR_TBLOFF_Msk                (0x1FFFFFFUL << SCB_VTOR_TBLOFF_Pos)


#define SCB_AIRCR_VECTKEY_Pos              16U
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)

#define SCB_AIRCR_VECTKEYSTAT_Pos          16U
#define SCB_AIRCR_VECTKEYSTAT_Msk          (0xFFFFUL << SCB_AIRCR_VECTKEYSTAT_Pos)

#define SCB_AIRCR_ENDIANESS_Pos            15U
#define SCB_AIRCR_ENDIANESS_Msk            (1UL << SCB_AIRCR_ENDIANESS_Pos)

#define SCB_AIRCR_PRIS_Pos                 14U
#define SCB_AIRCR_PRIS_Msk                 (1UL << SCB_AIRCR_PRIS_Pos)

#define SCB_AIRCR_BFHFNMINS_Pos            13U
#define SCB_AIRCR_BFHFNMINS_Msk            (1UL << SCB_AIRCR_BFHFNMINS_Pos)

#define SCB_AIRCR_PRIGROUP_Pos              8U
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)

#define SCB_AIRCR_SYSRESETREQS_Pos          3U
#define SCB_AIRCR_SYSRESETREQS_Msk         (1UL << SCB_AIRCR_SYSRESETREQS_Pos)

#define SCB_AIRCR_SYSRESETREQ_Pos           2U
#define SCB_AIRCR_SYSRESETREQ_Msk          (1UL << SCB_AIRCR_SYSRESETREQ_Pos)

#define SCB_AIRCR_VECTCLRACTIVE_Pos         1U
#define SCB_AIRCR_VECTCLRACTIVE_Msk        (1UL << SCB_AIRCR_VECTCLRACTIVE_Pos)


#define SCB_SCR_SEVONPEND_Pos               4U
#define SCB_SCR_SEVONPEND_Msk              (1UL << SCB_SCR_SEVONPEND_Pos)

#define SCB_SCR_SLEEPDEEPS_Pos              3U
#define SCB_SCR_SLEEPDEEPS_Msk             (1UL << SCB_SCR_SLEEPDEEPS_Pos)

#define SCB_SCR_SLEEPDEEP_Pos               2U
#define SCB_SCR_SLEEPDEEP_Msk              (1UL << SCB_SCR_SLEEPDEEP_Pos)

#define SCB_SCR_SLEEPONEXIT_Pos             1U
#define SCB_SCR_SLEEPONEXIT_Msk            (1UL << SCB_SCR_SLEEPONEXIT_Pos)


#define SCB_CCR_BP_Pos                     18U
#define SCB_CCR_BP_Msk                     (1UL << SCB_CCR_BP_Pos)

#define SCB_CCR_IC_Pos                     17U
#define SCB_CCR_IC_Msk                     (1UL << SCB_CCR_IC_Pos)

#define SCB_CCR_DC_Pos                     16U
#define SCB_CCR_DC_Msk                     (1UL << SCB_CCR_DC_Pos)

#define SCB_CCR_STKOFHFNMIGN_Pos           10U
#define SCB_CCR_STKOFHFNMIGN_Msk           (1UL << SCB_CCR_STKOFHFNMIGN_Pos)

#define SCB_CCR_BFHFNMIGN_Pos               8U
#define SCB_CCR_BFHFNMIGN_Msk              (1UL << SCB_CCR_BFHFNMIGN_Pos)

#define SCB_CCR_DIV_0_TRP_Pos               4U
#define SCB_CCR_DIV_0_TRP_Msk              (1UL << SCB_CCR_DIV_0_TRP_Pos)

#define SCB_CCR_UNALIGN_TRP_Pos             3U
#define SCB_CCR_UNALIGN_TRP_Msk            (1UL << SCB_CCR_UNALIGN_TRP_Pos)

#define SCB_CCR_USERSETMPEND_Pos            1U
#define SCB_CCR_USERSETMPEND_Msk           (1UL << SCB_CCR_USERSETMPEND_Pos)


#define SCB_SHCSR_HARDFAULTPENDED_Pos      21U
#define SCB_SHCSR_HARDFAULTPENDED_Msk      (1UL << SCB_SHCSR_HARDFAULTPENDED_Pos)

#define SCB_SHCSR_SECUREFAULTPENDED_Pos    20U
#define SCB_SHCSR_SECUREFAULTPENDED_Msk    (1UL << SCB_SHCSR_SECUREFAULTPENDED_Pos)

#define SCB_SHCSR_SECUREFAULTENA_Pos       19U
#define SCB_SHCSR_SECUREFAULTENA_Msk       (1UL << SCB_SHCSR_SECUREFAULTENA_Pos)

#define SCB_SHCSR_USGFAULTENA_Pos          18U
#define SCB_SHCSR_USGFAULTENA_Msk          (1UL << SCB_SHCSR_USGFAULTENA_Pos)

#define SCB_SHCSR_BUSFAULTENA_Pos          17U
#define SCB_SHCSR_BUSFAULTENA_Msk          (1UL << SCB_SHCSR_BUSFAULTENA_Pos)

#define SCB_SHCSR_MEMFAULTENA_Pos          16U
#define SCB_SHCSR_MEMFAULTENA_Msk          (1UL << SCB_SHCSR_MEMFAULTENA_Pos)

#define SCB_SHCSR_SVCALLPENDED_Pos         15U
#define SCB_SHCSR_SVCALLPENDED_Msk         (1UL << SCB_SHCSR_SVCALLPENDED_Pos)

#define SCB_SHCSR_BUSFAULTPENDED_Pos       14U
#define SCB_SHCSR_BUSFAULTPENDED_Msk       (1UL << SCB_SHCSR_BUSFAULTPENDED_Pos)

#define SCB_SHCSR_MEMFAULTPENDED_Pos       13U
#define SCB_SHCSR_MEMFAULTPENDED_Msk       (1UL << SCB_SHCSR_MEMFAULTPENDED_Pos)

#define SCB_SHCSR_USGFAULTPENDED_Pos       12U
#define SCB_SHCSR_USGFAULTPENDED_Msk       (1UL << SCB_SHCSR_USGFAULTPENDED_Pos)

#define SCB_SHCSR_SYSTICKACT_Pos           11U
#define SCB_SHCSR_SYSTICKACT_Msk           (1UL << SCB_SHCSR_SYSTICKACT_Pos)

#define SCB_SHCSR_PENDSVACT_Pos            10U
#define SCB_SHCSR_PENDSVACT_Msk            (1UL << SCB_SHCSR_PENDSVACT_Pos)

#define SCB_SHCSR_MONITORACT_Pos            8U
#define SCB_SHCSR_MONITORACT_Msk           (1UL << SCB_SHCSR_MONITORACT_Pos)

#define SCB_SHCSR_SVCALLACT_Pos             7U
#define SCB_SHCSR_SVCALLACT_Msk            (1UL << SCB_SHCSR_SVCALLACT_Pos)

#define SCB_SHCSR_NMIACT_Pos                5U
#define SCB_SHCSR_NMIACT_Msk               (1UL << SCB_SHCSR_NMIACT_Pos)

#define SCB_SHCSR_SECUREFAULTACT_Pos        4U
#define SCB_SHCSR_SECUREFAULTACT_Msk       (1UL << SCB_SHCSR_SECUREFAULTACT_Pos)

#define SCB_SHCSR_USGFAULTACT_Pos           3U
#define SCB_SHCSR_USGFAULTACT_Msk          (1UL << SCB_SHCSR_USGFAULTACT_Pos)

#define SCB_SHCSR_HARDFAULTACT_Pos          2U
#define SCB_SHCSR_HARDFAULTACT_Msk         (1UL << SCB_SHCSR_HARDFAULTACT_Pos)

#define SCB_SHCSR_BUSFAULTACT_Pos           1U
#define SCB_SHCSR_BUSFAULTACT_Msk          (1UL << SCB_SHCSR_BUSFAULTACT_Pos)

#define SCB_SHCSR_MEMFAULTACT_Pos           0U
#define SCB_SHCSR_MEMFAULTACT_Msk          (1UL


#define SCB_CFSR_USGFAULTSR_Pos            16U
#define SCB_CFSR_USGFAULTSR_Msk            (0xFFFFUL << SCB_CFSR_USGFAULTSR_Pos)

#define SCB_CFSR_BUSFAULTSR_Pos             8U
#define SCB_CFSR_BUSFAULTSR_Msk            (0xFFUL << SCB_CFSR_BUSFAULTSR_Pos)

#define SCB_CFSR_MEMFAULTSR_Pos             0U
#define SCB_CFSR_MEMFAULTSR_Msk            (0xFFUL


#define SCB_CFSR_MMARVALID_Pos             (SCB_SHCSR_MEMFAULTACT_Pos + 7U)
#define SCB_CFSR_MMARVALID_Msk             (1UL << SCB_CFSR_MMARVALID_Pos)

#define SCB_CFSR_MLSPERR_Pos               (SCB_SHCSR_MEMFAULTACT_Pos + 5U)
#define SCB_CFSR_MLSPERR_Msk               (1UL << SCB_CFSR_MLSPERR_Pos)

#define SCB_CFSR_MSTKERR_Pos               (SCB_SHCSR_MEMFAULTACT_Pos + 4U)
#define SCB_CFSR_MSTKERR_Msk               (1UL << SCB_CFSR_MSTKERR_Pos)

#define SCB_CFSR_MUNSTKERR_Pos             (SCB_SHCSR_MEMFAULTACT_Pos + 3U)
#define SCB_CFSR_MUNSTKERR_Msk             (1UL << SCB_CFSR_MUNSTKERR_Pos)

#define SCB_CFSR_DACCVIOL_Pos              (SCB_SHCSR_MEMFAULTACT_Pos + 1U)
#define SCB_CFSR_DACCVIOL_Msk              (1UL << SCB_CFSR_DACCVIOL_Pos)

#define SCB_CFSR_IACCVIOL_Pos              (SCB_SHCSR_MEMFAULTACT_Pos + 0U)
#define SCB_CFSR_IACCVIOL_Msk              (1UL


#define SCB_CFSR_BFARVALID_Pos            (SCB_CFSR_BUSFAULTSR_Pos + 7U)
#define SCB_CFSR_BFARVALID_Msk            (1UL << SCB_CFSR_BFARVALID_Pos)

#define SCB_CFSR_LSPERR_Pos               (SCB_CFSR_BUSFAULTSR_Pos + 5U)
#define SCB_CFSR_LSPERR_Msk               (1UL << SCB_CFSR_LSPERR_Pos)

#define SCB_CFSR_STKERR_Pos               (SCB_CFSR_BUSFAULTSR_Pos + 4U)
#define SCB_CFSR_STKERR_Msk               (1UL << SCB_CFSR_STKERR_Pos)

#define SCB_CFSR_UNSTKERR_Pos             (SCB_CFSR_BUSFAULTSR_Pos + 3U)
#define SCB_CFSR_UNSTKERR_Msk             (1UL << SCB_CFSR_UNSTKERR_Pos)

#define SCB_CFSR_IMPRECISERR_Pos          (SCB_CFSR_BUSFAULTSR_Pos + 2U)
#define SCB_CFSR_IMPRECISERR_Msk          (1UL << SCB_CFSR_IMPRECISERR_Pos)

#define SCB_CFSR_PRECISERR_Pos            (SCB_CFSR_BUSFAULTSR_Pos + 1U)
#define SCB_CFSR_PRECISERR_Msk            (1UL << SCB_CFSR_PRECISERR_Pos)

#define SCB_CFSR_IBUSERR_Pos              (SCB_CFSR_BUSFAULTSR_Pos + 0U)
#define SCB_CFSR_IBUSERR_Msk              (1UL << SCB_CFSR_IBUSERR_Pos)


#define SCB_CFSR_DIVBYZERO_Pos            (SCB_CFSR_USGFAULTSR_Pos + 9U)
#define SCB_CFSR_DIVBYZERO_Msk            (1UL << SCB_CFSR_DIVBYZERO_Pos)

#define SCB_CFSR_UNALIGNED_Pos            (SCB_CFSR_USGFAULTSR_Pos + 8U)
#define SCB_CFSR_UNALIGNED_Msk            (1UL << SCB_CFSR_UNALIGNED_Pos)

#define SCB_CFSR_STKOF_Pos                (SCB_CFSR_USGFAULTSR_Pos + 4U)
#define SCB_CFSR_STKOF_Msk                (1UL << SCB_CFSR_STKOF_Pos)

#define SCB_CFSR_NOCP_Pos                 (SCB_CFSR_USGFAULTSR_Pos + 3U)
#define SCB_CFSR_NOCP_Msk                 (1UL << SCB_CFSR_NOCP_Pos)

#define SCB_CFSR_INVPC_Pos                (SCB_CFSR_USGFAULTSR_Pos + 2U)
#define SCB_CFSR_INVPC_Msk                (1UL << SCB_CFSR_INVPC_Pos)

#define SCB_CFSR_INVSTATE_Pos             (SCB_CFSR_USGFAULTSR_Pos + 1U)
#define SCB_CFSR_INVSTATE_Msk             (1UL << SCB_CFSR_INVSTATE_Pos)

#define SCB_CFSR_UNDEFINSTR_Pos           (SCB_CFSR_USGFAULTSR_Pos + 0U)
#define SCB_CFSR_UNDEFINSTR_Msk           (1UL << SCB_CFSR_UNDEFINSTR_Pos)


#define SCB_HFSR_DEBUGEVT_Pos              31U
#define SCB_HFSR_DEBUGEVT_Msk              (1UL << SCB_HFSR_DEBUGEVT_Pos)

#define SCB_HFSR_FORCED_Pos                30U
#define SCB_HFSR_FORCED_Msk                (1UL << SCB_HFSR_FORCED_Pos)

#define SCB_HFSR_VECTTBL_Pos                1U
#define SCB_HFSR_VECTTBL_Msk               (1UL << SCB_HFSR_VECTTBL_Pos)


#define SCB_DFSR_EXTERNAL_Pos               4U
#define SCB_DFSR_EXTERNAL_Msk              (1UL << SCB_DFSR_EXTERNAL_Pos)

#define SCB_DFSR_VCATCH_Pos                 3U
#define SCB_DFSR_VCATCH_Msk                (1UL << SCB_DFSR_VCATCH_Pos)

#define SCB_DFSR_DWTTRAP_Pos                2U
#define SCB_DFSR_DWTTRAP_Msk               (1UL << SCB_DFSR_DWTTRAP_Pos)

#define SCB_DFSR_BKPT_Pos                   1U
#define SCB_DFSR_BKPT_Msk                  (1UL << SCB_DFSR_BKPT_Pos)

#define SCB_DFSR_HALTED_Pos                 0U
#define SCB_DFSR_HALTED_Msk                (1UL


#define SCB_NSACR_CP11_Pos                 11U
#define SCB_NSACR_CP11_Msk                 (1UL << SCB_NSACR_CP11_Pos)

#define SCB_NSACR_CP10_Pos                 10U
#define SCB_NSACR_CP10_Msk                 (1UL << SCB_NSACR_CP10_Pos)

#define SCB_NSACR_CPn_Pos                   0U
#define SCB_NSACR_CPn_Msk                  (1UL


#define SCB_CLIDR_LOUU_Pos                 27U
#define SCB_CLIDR_LOUU_Msk                 (7UL << SCB_CLIDR_LOUU_Pos)

#define SCB_CLIDR_LOC_Pos                  24U
#define SCB_CLIDR_LOC_Msk                  (7UL << SCB_CLIDR_LOC_Pos)


#define SCB_CTR_FORMAT_Pos                 29U
#define SCB_CTR_FORMAT_Msk                 (7UL << SCB_CTR_FORMAT_Pos)

#define SCB_CTR_CWG_Pos                    24U
#define SCB_CTR_CWG_Msk                    (0xFUL << SCB_CTR_CWG_Pos)

#define SCB_CTR_ERG_Pos                    20U
#define SCB_CTR_ERG_Msk                    (0xFUL << SCB_CTR_ERG_Pos)

#define SCB_CTR_DMINLINE_Pos               16U
#define SCB_CTR_DMINLINE_Msk               (0xFUL << SCB_CTR_DMINLINE_Pos)

#define SCB_CTR_IMINLINE_Pos                0U
#define SCB_CTR_IMINLINE_Msk               (0xFUL


#define SCB_CCSIDR_WT_Pos                  31U
#define SCB_CCSIDR_WT_Msk                  (1UL << SCB_CCSIDR_WT_Pos)

#define SCB_CCSIDR_WB_Pos                  30U
#define SCB_CCSIDR_WB_Msk                  (1UL << SCB_CCSIDR_WB_Pos)

#define SCB_CCSIDR_RA_Pos                  29U
#define SCB_CCSIDR_RA_Msk                  (1UL << SCB_CCSIDR_RA_Pos)

#define SCB_CCSIDR_WA_Pos                  28U
#define SCB_CCSIDR_WA_Msk                  (1UL << SCB_CCSIDR_WA_Pos)

#define SCB_CCSIDR_NUMSETS_Pos             13U
#define SCB_CCSIDR_NUMSETS_Msk             (0x7FFFUL << SCB_CCSIDR_NUMSETS_Pos)

#define SCB_CCSIDR_ASSOCIATIVITY_Pos        3U
#define SCB_CCSIDR_ASSOCIATIVITY_Msk       (0x3FFUL << SCB_CCSIDR_ASSOCIATIVITY_Pos)

#define SCB_CCSIDR_LINESIZE_Pos             0U
#define SCB_CCSIDR_LINESIZE_Msk            (7UL


#define SCB_CSSELR_LEVEL_Pos                1U
#define SCB_CSSELR_LEVEL_Msk               (7UL << SCB_CSSELR_LEVEL_Pos)

#define SCB_CSSELR_IND_Pos                  0U
#define SCB_CSSELR_IND_Msk                 (1UL


#define SCB_STIR_INTID_Pos                  0U
#define SCB_STIR_INTID_Msk                 (0x1FFUL


#define SCB_DCISW_WAY_Pos                  30U
#define SCB_DCISW_WAY_Msk                  (3UL << SCB_DCISW_WAY_Pos)

#define SCB_DCISW_SET_Pos                   5U
#define SCB_DCISW_SET_Msk                  (0x1FFUL << SCB_DCISW_SET_Pos)


#define SCB_DCCSW_WAY_Pos                  30U
#define SCB_DCCSW_WAY_Msk                  (3UL << SCB_DCCSW_WAY_Pos)

#define SCB_DCCSW_SET_Pos                   5U
#define SCB_DCCSW_SET_Msk                  (0x1FFUL << SCB_DCCSW_SET_Pos)


#define SCB_DCCISW_WAY_Pos                 30U
#define SCB_DCCISW_WAY_Msk                 (3UL << SCB_DCCISW_WAY_Pos)

#define SCB_DCCISW_SET_Pos                  5U
#define SCB_DCCISW_SET_Msk                 (0x1FFUL << SCB_DCCISW_SET_Pos)




/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SCnSCB System Controls not in SCB (SCnSCB)
  \brief    Type definitions for the System Control and ID Register not in the SCB
  @{
 */

/**
  \brief  Structure type to access the System Control and ID Register not in the SCB.
 */
typedef struct
{
        uint32_t RESERVED0[1U];
  __IM  uint32_t ICTR;
  __IOM uint32_t ACTLR;
  __IOM uint32_t CPPWR;
} SCnSCB_Type;


#define SCnSCB_ICTR_INTLINESNUM_Pos         0U
#define SCnSCB_ICTR_INTLINESNUM_Msk        (0xFUL




/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SysTick     System Tick Timer (SysTick)
  \brief    Type definitions for the System Timer Registers.
  @{
 */

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
  __IOM uint32_t CTRL;
  __IOM uint32_t LOAD;
  __IOM uint32_t VAL;
  __IM  uint32_t CALIB;
} SysTick_Type;


#define SysTick_CTRL_COUNTFLAG_Pos         16U
#define SysTick_CTRL_COUNTFLAG_Msk         (1UL << SysTick_CTRL_COUNTFLAG_Pos)

#define SysTick_CTRL_CLKSOURCE_Pos          2U
#define SysTick_CTRL_CLKSOURCE_Msk         (1UL << SysTick_CTRL_CLKSOURCE_Pos)

#define SysTick_CTRL_TICKINT_Pos            1U
#define SysTick_CTRL_TICKINT_Msk           (1UL << SysTick_CTRL_TICKINT_Pos)

#define SysTick_CTRL_ENABLE_Pos             0U
#define SysTick_CTRL_ENABLE_Msk            (1UL


#define SysTick_LOAD_RELOAD_Pos             0U
#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFUL


#define SysTick_VAL_CURRENT_Pos             0U
#define SysTick_VAL_CURRENT_Msk            (0xFFFFFFUL


#define SysTick_CALIB_NOREF_Pos            31U
#define SysTick_CALIB_NOREF_Msk            (1UL << SysTick_CALIB_NOREF_Pos)

#define SysTick_CALIB_SKEW_Pos             30U
#define SysTick_CALIB_SKEW_Msk             (1UL << SysTick_CALIB_SKEW_Pos)

#define SysTick_CALIB_TENMS_Pos             0U
#define SysTick_CALIB_TENMS_Msk            (0xFFFFFFUL




/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_ITM     Instrumentation Trace Macrocell (ITM)
  \brief    Type definitions for the Instrumentation Trace Macrocell (ITM)
  @{
 */

/**
  \brief  Structure type to access the Instrumentation Trace Macrocell Register (ITM).
 */
typedef struct
{
  __OM  union
  {
    __OM  uint8_t    u8;
    __OM  uint16_t   u16;
    __OM  uint32_t   u32;
  }  PORT [32U];
        uint32_t RESERVED0[864U];
  __IOM uint32_t TER;
        uint32_t RESERVED1[15U];
  __IOM uint32_t TPR;
        uint32_t RESERVED2[15U];
  __IOM uint32_t TCR;
        uint32_t RESERVED3[32U];
        uint32_t RESERVED4[43U];
  __OM  uint32_t LAR;
  __IM  uint32_t LSR;
        uint32_t RESERVED5[1U];
  __IM  uint32_t DEVARCH;
        uint32_t RESERVED6[4U];
  __IM  uint32_t PID4;
  __IM  uint32_t PID5;
  __IM  uint32_t PID6;
  __IM  uint32_t PID7;
  __IM  uint32_t PID0;
  __IM  uint32_t PID1;
  __IM  uint32_t PID2;
  __IM  uint32_t PID3;
  __IM  uint32_t CID0;
  __IM  uint32_t CID1;
  __IM  uint32_t CID2;
  __IM  uint32_t CID3;
} ITM_Type;


#define ITM_STIM_DISABLED_Pos               1U
#define ITM_STIM_DISABLED_Msk              (0x1UL << ITM_STIM_DISABLED_Pos)

#define ITM_STIM_FIFOREADY_Pos              0U
#define ITM_STIM_FIFOREADY_Msk             (0x1UL


#define ITM_TPR_PRIVMASK_Pos                0U
#define ITM_TPR_PRIVMASK_Msk               (0xFUL


#define ITM_TCR_BUSY_Pos                   23U
#define ITM_TCR_BUSY_Msk                   (1UL << ITM_TCR_BUSY_Pos)

#define ITM_TCR_TRACEBUSID_Pos             16U
#define ITM_TCR_TRACEBUSID_Msk             (0x7FUL << ITM_TCR_TRACEBUSID_Pos)

#define ITM_TCR_GTSFREQ_Pos                10U
#define ITM_TCR_GTSFREQ_Msk                (3UL << ITM_TCR_GTSFREQ_Pos)

#define ITM_TCR_TSPRESCALE_Pos              8U
#define ITM_TCR_TSPRESCALE_Msk             (3UL << ITM_TCR_TSPRESCALE_Pos)

#define ITM_TCR_STALLENA_Pos                5U
#define ITM_TCR_STALLENA_Msk               (1UL << ITM_TCR_STALLENA_Pos)

#define ITM_TCR_SWOENA_Pos                  4U
#define ITM_TCR_SWOENA_Msk                 (1UL << ITM_TCR_SWOENA_Pos)

#define ITM_TCR_DWTENA_Pos                  3U
#define ITM_TCR_DWTENA_Msk                 (1UL << ITM_TCR_DWTENA_Pos)

#define ITM_TCR_SYNCENA_Pos                 2U
#define ITM_TCR_SYNCENA_Msk                (1UL << ITM_TCR_SYNCENA_Pos)

#define ITM_TCR_TSENA_Pos                   1U
#define ITM_TCR_TSENA_Msk                  (1UL << ITM_TCR_TSENA_Pos)

#define ITM_TCR_ITMENA_Pos                  0U
#define ITM_TCR_ITMENA_Msk                 (1UL


#define ITM_LSR_ByteAcc_Pos                 2U
#define ITM_LSR_ByteAcc_Msk                (1UL << ITM_LSR_ByteAcc_Pos)

#define ITM_LSR_Access_Pos                  1U
#define ITM_LSR_Access_Msk                 (1UL << ITM_LSR_Access_Pos)

#define ITM_LSR_Present_Pos                 0U
#define ITM_LSR_Present_Msk                (1UL




/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_DWT     Data Watchpoint and Trace (DWT)
  \brief    Type definitions for the Data Watchpoint and Trace (DWT)
  @{
 */

/**
  \brief  Structure type to access the Data Watchpoint and Trace Register (DWT).
 */
typedef struct
{
  __IOM uint32_t CTRL;
  __IOM uint32_t CYCCNT;
  __IOM uint32_t CPICNT;
  __IOM uint32_t EXCCNT;
  __IOM uint32_t SLEEPCNT;
  __IOM uint32_t LSUCNT;
  __IOM uint32_t FOLDCNT;
  __IM  uint32_t PCSR;
  __IOM uint32_t COMP0;
        uint32_t RESERVED1[1U];
  __IOM uint32_t FUNCTION0;
        uint32_t RESERVED2[1U];
  __IOM uint32_t COMP1;
        uint32_t RESERVED3[1U];
  __IOM uint32_t FUNCTION1;
        uint32_t RESERVED4[1U];
  __IOM uint32_t COMP2;
        uint32_t RESERVED5[1U];
  __IOM uint32_t FUNCTION2;
        uint32_t RESERVED6[1U];
  __IOM uint32_t COMP3;
        uint32_t RESERVED7[1U];
  __IOM uint32_t FUNCTION3;
        uint32_t RESERVED8[1U];
  __IOM uint32_t COMP4;
        uint32_t RESERVED9[1U];
  __IOM uint32_t FUNCTION4;
        uint32_t RESERVED10[1U];
  __IOM uint32_t COMP5;
        uint32_t RESERVED11[1U];
  __IOM uint32_t FUNCTION5;
        uint32_t RESERVED12[1U];
  __IOM uint32_t COMP6;
        uint32_t RESERVED13[1U];
  __IOM uint32_t FUNCTION6;
        uint32_t RESERVED14[1U];
  __IOM uint32_t COMP7;
        uint32_t RESERVED15[1U];
  __IOM uint32_t FUNCTION7;
        uint32_t RESERVED16[1U];
  __IOM uint32_t COMP8;
        uint32_t RESERVED17[1U];
  __IOM uint32_t FUNCTION8;
        uint32_t RESERVED18[1U];
  __IOM uint32_t COMP9;
        uint32_t RESERVED19[1U];
  __IOM uint32_t FUNCTION9;
        uint32_t RESERVED20[1U];
  __IOM uint32_t COMP10;
        uint32_t RESERVED21[1U];
  __IOM uint32_t FUNCTION10;
        uint32_t RESERVED22[1U];
  __IOM uint32_t COMP11;
        uint32_t RESERVED23[1U];
  __IOM uint32_t FUNCTION11;
        uint32_t RESERVED24[1U];
  __IOM uint32_t COMP12;
        uint32_t RESERVED25[1U];
  __IOM uint32_t FUNCTION12;
        uint32_t RESERVED26[1U];
  __IOM uint32_t COMP13;
        uint32_t RESERVED27[1U];
  __IOM uint32_t FUNCTION13;
        uint32_t RESERVED28[1U];
  __IOM uint32_t COMP14;
        uint32_t RESERVED29[1U];
  __IOM uint32_t FUNCTION14;
        uint32_t RESERVED30[1U];
  __IOM uint32_t COMP15;
        uint32_t RESERVED31[1U];
  __IOM uint32_t FUNCTION15;
        uint32_t RESERVED32[934U];
  __IM  uint32_t LSR;
        uint32_t RESERVED33[1U];
  __IM  uint32_t DEVARCH;
} DWT_Type;


#define DWT_CTRL_NUMCOMP_Pos               28U
#define DWT_CTRL_NUMCOMP_Msk               (0xFUL << DWT_CTRL_NUMCOMP_Pos)

#define DWT_CTRL_NOTRCPKT_Pos              27U
#define DWT_CTRL_NOTRCPKT_Msk              (0x1UL << DWT_CTRL_NOTRCPKT_Pos)

#define DWT_CTRL_NOEXTTRIG_Pos             26U
#define DWT_CTRL_NOEXTTRIG_Msk             (0x1UL << DWT_CTRL_NOEXTTRIG_Pos)

#define DWT_CTRL_NOCYCCNT_Pos              25U
#define DWT_CTRL_NOCYCCNT_Msk              (0x1UL << DWT_CTRL_NOCYCCNT_Pos)

#define DWT_CTRL_NOPRFCNT_Pos              24U
#define DWT_CTRL_NOPRFCNT_Msk              (0x1UL << DWT_CTRL_NOPRFCNT_Pos)

#define DWT_CTRL_CYCDISS_Pos               23U
#define DWT_CTRL_CYCDISS_Msk               (0x1UL << DWT_CTRL_CYCDISS_Pos)

#define DWT_CTRL_CYCEVTENA_Pos             22U
#define DWT_CTRL_CYCEVTENA_Msk             (0x1UL << DWT_CTRL_CYCEVTENA_Pos)

#define DWT_CTRL_FOLDEVTENA_Pos            21U
#define DWT_CTRL_FOLDEVTENA_Msk            (0x1UL << DWT_CTRL_FOLDEVTENA_Pos)

#define DWT_CTRL_LSUEVTENA_Pos             20U
#define DWT_CTRL_LSUEVTENA_Msk             (0x1UL << DWT_CTRL_LSUEVTENA_Pos)

#define DWT_CTRL_SLEEPEVTENA_Pos           19U
#define DWT_CTRL_SLEEPEVTENA_Msk           (0x1UL << DWT_CTRL_SLEEPEVTENA_Pos)

#define DWT_CTRL_EXCEVTENA_Pos             18U
#define DWT_CTRL_EXCEVTENA_Msk             (0x1UL << DWT_CTRL_EXCEVTENA_Pos)

#define DWT_CTRL_CPIEVTENA_Pos             17U
#define DWT_CTRL_CPIEVTENA_Msk             (0x1UL << DWT_CTRL_CPIEVTENA_Pos)

#define DWT_CTRL_EXCTRCENA_Pos             16U
#define DWT_CTRL_EXCTRCENA_Msk             (0x1UL << DWT_CTRL_EXCTRCENA_Pos)

#define DWT_CTRL_PCSAMPLENA_Pos            12U
#define DWT_CTRL_PCSAMPLENA_Msk            (0x1UL << DWT_CTRL_PCSAMPLENA_Pos)

#define DWT_CTRL_SYNCTAP_Pos               10U
#define DWT_CTRL_SYNCTAP_Msk               (0x3UL << DWT_CTRL_SYNCTAP_Pos)

#define DWT_CTRL_CYCTAP_Pos                 9U
#define DWT_CTRL_CYCTAP_Msk                (0x1UL << DWT_CTRL_CYCTAP_Pos)

#define DWT_CTRL_POSTINIT_Pos               5U
#define DWT_CTRL_POSTINIT_Msk              (0xFUL << DWT_CTRL_POSTINIT_Pos)

#define DWT_CTRL_POSTPRESET_Pos             1U
#define DWT_CTRL_POSTPRESET_Msk            (0xFUL << DWT_CTRL_POSTPRESET_Pos)

#define DWT_CTRL_CYCCNTENA_Pos              0U
#define DWT_CTRL_CYCCNTENA_Msk             (0x1UL


#define DWT_CPICNT_CPICNT_Pos               0U
#define DWT_CPICNT_CPICNT_Msk              (0xFFUL


#define DWT_EXCCNT_EXCCNT_Pos               0U
#define DWT_EXCCNT_EXCCNT_Msk              (0xFFUL


#define DWT_SLEEPCNT_SLEEPCNT_Pos           0U
#define DWT_SLEEPCNT_SLEEPCNT_Msk          (0xFFUL


#define DWT_LSUCNT_LSUCNT_Pos               0U
#define DWT_LSUCNT_LSUCNT_Msk              (0xFFUL


#define DWT_FOLDCNT_FOLDCNT_Pos             0U
#define DWT_FOLDCNT_FOLDCNT_Msk            (0xFFUL


#define DWT_FUNCTION_ID_Pos                27U
#define DWT_FUNCTION_ID_Msk                (0x1FUL << DWT_FUNCTION_ID_Pos)

#define DWT_FUNCTION_MATCHED_Pos           24U
#define DWT_FUNCTION_MATCHED_Msk           (0x1UL << DWT_FUNCTION_MATCHED_Pos)

#define DWT_FUNCTION_DATAVSIZE_Pos         10U
#define DWT_FUNCTION_DATAVSIZE_Msk         (0x3UL << DWT_FUNCTION_DATAVSIZE_Pos)

#define DWT_FUNCTION_ACTION_Pos             4U
#define DWT_FUNCTION_ACTION_Msk            (0x1UL << DWT_FUNCTION_ACTION_Pos)

#define DWT_FUNCTION_MATCH_Pos              0U
#define DWT_FUNCTION_MATCH_Msk             (0xFUL




/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_TPI     Trace Port Interface (TPI)
  \brief    Type definitions for the Trace Port Interface (TPI)
  @{
 */

/**
  \brief  Structure type to access the Trace Port Interface Register (TPI).
 */
typedef struct
{
  __IM  uint32_t SSPSR;
  __IOM uint32_t CSPSR;
        uint32_t RESERVED0[2U];
  __IOM uint32_t ACPR;
        uint32_t RESERVED1[55U];
  __IOM uint32_t SPPR;
        uint32_t RESERVED2[131U];
  __IM  uint32_t FFSR;
  __IOM uint32_t FFCR;
  __IOM uint32_t PSCR;
        uint32_t RESERVED3[809U];
  __OM  uint32_t LAR;
  __IM  uint32_t LSR;
        uint32_t RESERVED4[4U];
  __IM  uint32_t TYPE;
  __IM  uint32_t DEVTYPE;
} TPI_Type;


#define TPI_ACPR_SWOSCALER_Pos              0U
#define TPI_ACPR_SWOSCALER_Msk             (0xFFFFUL


#define TPI_SPPR_TXMODE_Pos                 0U
#define TPI_SPPR_TXMODE_Msk                (0x3UL


#define TPI_FFSR_FtNonStop_Pos              3U
#define TPI_FFSR_FtNonStop_Msk             (0x1UL << TPI_FFSR_FtNonStop_Pos)

#define TPI_FFSR_TCPresent_Pos              2U
#define TPI_FFSR_TCPresent_Msk             (0x1UL << TPI_FFSR_TCPresent_Pos)

#define TPI_FFSR_FtStopped_Pos              1U
#define TPI_FFSR_FtStopped_Msk             (0x1UL << TPI_FFSR_FtStopped_Pos)

#define TPI_FFSR_FlInProg_Pos               0U
#define TPI_FFSR_FlInProg_Msk              (0x1UL


#define TPI_FFCR_TrigIn_Pos                 8U
#define TPI_FFCR_TrigIn_Msk                (0x1UL << TPI_FFCR_TrigIn_Pos)

#define TPI_FFCR_FOnMan_Pos                 6U
#define TPI_FFCR_FOnMan_Msk                (0x1UL << TPI_FFCR_FOnMan_Pos)

#define TPI_FFCR_EnFCont_Pos                1U
#define TPI_FFCR_EnFCont_Msk               (0x1UL << TPI_FFCR_EnFCont_Pos)


#define TPI_PSCR_PSCount_Pos                0U
#define TPI_PSCR_PSCount_Msk               (0x1FUL


#define TPI_LSR_nTT_Pos                     1U
#define TPI_LSR_nTT_Msk                    (0x1UL << TPI_LSR_nTT_Pos)

#define TPI_LSR_SLK_Pos                     1U
#define TPI_LSR_SLK_Msk                    (0x1UL << TPI_LSR_SLK_Pos)

#define TPI_LSR_SLI_Pos                     0U
#define TPI_LSR_SLI_Msk                    (0x1UL


#define TPI_DEVID_NRZVALID_Pos             11U
#define TPI_DEVID_NRZVALID_Msk             (0x1UL << TPI_DEVID_NRZVALID_Pos)

#define TPI_DEVID_MANCVALID_Pos            10U
#define TPI_DEVID_MANCVALID_Msk            (0x1UL << TPI_DEVID_MANCVALID_Pos)

#define TPI_DEVID_PTINVALID_Pos             9U
#define TPI_DEVID_PTINVALID_Msk            (0x1UL << TPI_DEVID_PTINVALID_Pos)

#define TPI_DEVID_FIFOSZ_Pos                6U
#define TPI_DEVID_FIFOSZ_Msk               (0x7UL << TPI_DEVID_FIFOSZ_Pos)


#define TPI_DEVTYPE_SubType_Pos             4U
#define TPI_DEVTYPE_SubType_Msk            (0xFUL

#define TPI_DEVTYPE_MajorType_Pos           0U
#define TPI_DEVTYPE_MajorType_Msk          (0xFUL << TPI_DEVTYPE_MajorType_Pos)




#if defined (__MPU_PRESENT) && (__MPU_PRESENT == 1U)
/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_MPU     Memory Protection Unit (MPU)
  \brief    Type definitions for the Memory Protection Unit (MPU)
  @{
 */

/**
  \brief  Structure type to access the Memory Protection Unit (MPU).
 */
typedef struct
{
  __IM  uint32_t TYPE;
  __IOM uint32_t CTRL;
  __IOM uint32_t RNR;
  __IOM uint32_t RBAR;
  __IOM uint32_t RLAR;
  __IOM uint32_t RBAR_A1;
  __IOM uint32_t RLAR_A1;
  __IOM uint32_t RBAR_A2;
  __IOM uint32_t RLAR_A2;
  __IOM uint32_t RBAR_A3;
  __IOM uint32_t RLAR_A3;
        uint32_t RESERVED0[1];
  union {
  __IOM uint32_t MAIR[2];
  struct {
  __IOM uint32_t MAIR0;
  __IOM uint32_t MAIR1;
  };
  };
} MPU_Type;

#define MPU_TYPE_RALIASES                  4U


#define MPU_TYPE_IREGION_Pos               16U
#define MPU_TYPE_IREGION_Msk               (0xFFUL << MPU_TYPE_IREGION_Pos)

#define MPU_TYPE_DREGION_Pos                8U
#define MPU_TYPE_DREGION_Msk               (0xFFUL << MPU_TYPE_DREGION_Pos)

#define MPU_TYPE_SEPARATE_Pos               0U
#define MPU_TYPE_SEPARATE_Msk              (1UL


#define MPU_CTRL_PRIVDEFENA_Pos             2U
#define MPU_CTRL_PRIVDEFENA_Msk            (1UL << MPU_CTRL_PRIVDEFENA_Pos)

#define MPU_CTRL_HFNMIENA_Pos               1U
#define MPU_CTRL_HFNMIENA_Msk              (1UL << MPU_CTRL_HFNMIENA_Pos)

#define MPU_CTRL_ENABLE_Pos                 0U
#define MPU_CTRL_ENABLE_Msk                (1UL


#define MPU_RNR_REGION_Pos                  0U
#define MPU_RNR_REGION_Msk                 (0xFFUL


#define MPU_RBAR_BASE_Pos                   5U
#define MPU_RBAR_BASE_Msk                  (0x7FFFFFFUL << MPU_RBAR_BASE_Pos)

#define MPU_RBAR_SH_Pos                     3U
#define MPU_RBAR_SH_Msk                    (0x3UL << MPU_RBAR_SH_Pos)

#define MPU_RBAR_AP_Pos                     1U
#define MPU_RBAR_AP_Msk                    (0x3UL << MPU_RBAR_AP_Pos)

#define MPU_RBAR_XN_Pos                     0U
#define MPU_RBAR_XN_Msk                    (01UL


#define MPU_RLAR_LIMIT_Pos                  5U
#define MPU_RLAR_LIMIT_Msk                 (0x7FFFFFFUL << MPU_RLAR_LIMIT_Pos)

#define MPU_RLAR_AttrIndx_Pos               1U
#define MPU_RLAR_AttrIndx_Msk              (0x7UL << MPU_RLAR_AttrIndx_Pos)

#define MPU_RLAR_EN_Pos                     0U
#define MPU_RLAR_EN_Msk                    (1UL


#define MPU_MAIR0_Attr3_Pos                24U
#define MPU_MAIR0_Attr3_Msk                (0xFFUL << MPU_MAIR0_Attr3_Pos)

#define MPU_MAIR0_Attr2_Pos                16U
#define MPU_MAIR0_Attr2_Msk                (0xFFUL << MPU_MAIR0_Attr2_Pos)

#define MPU_MAIR0_Attr1_Pos                 8U
#define MPU_MAIR0_Attr1_Msk                (0xFFUL << MPU_MAIR0_Attr1_Pos)

#define MPU_MAIR0_Attr0_Pos                 0U
#define MPU_MAIR0_Attr0_Msk                (0xFFUL


#define MPU_MAIR1_Attr7_Pos                24U
#define MPU_MAIR1_Attr7_Msk                (0xFFUL << MPU_MAIR1_Attr7_Pos)

#define MPU_MAIR1_Attr6_Pos                16U
#define MPU_MAIR1_Attr6_Msk                (0xFFUL << MPU_MAIR1_Attr6_Pos)

#define MPU_MAIR1_Attr5_Pos                 8U
#define MPU_MAIR1_Attr5_Msk                (0xFFUL << MPU_MAIR1_Attr5_Pos)

#define MPU_MAIR1_Attr4_Pos                 0U
#define MPU_MAIR1_Attr4_Msk                (0xFFUL


#endif


#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SAU     Security Attribution Unit (SAU)
  \brief    Type definitions for the Security Attribution Unit (SAU)
  @{
 */

/**
  \brief  Structure type to access the Security Attribution Unit (SAU).
 */
typedef struct
{
  __IOM uint32_t CTRL;
  __IM  uint32_t TYPE;
#if defined (__SAUREGION_PRESENT) && (__SAUREGION_PRESENT == 1U)
  __IOM uint32_t RNR;
  __IOM uint32_t RBAR;
  __IOM uint32_t RLAR;
#else
        uint32_t RESERVED0[3];
#endif
  __IOM uint32_t SFSR;
  __IOM uint32_t SFAR;
} SAU_Type;


#define SAU_CTRL_ALLNS_Pos                  1U
#define SAU_CTRL_ALLNS_Msk                 (1UL << SAU_CTRL_ALLNS_Pos)

#define SAU_CTRL_ENABLE_Pos                 0U
#define SAU_CTRL_ENABLE_Msk                (1UL


#define SAU_TYPE_SREGION_Pos                0U
#define SAU_TYPE_SREGION_Msk               (0xFFUL

#if defined (__SAUREGION_PRESENT) && (__SAUREGION_PRESENT == 1U)

#define SAU_RNR_REGION_Pos                  0U
#define SAU_RNR_REGION_Msk                 (0xFFUL


#define SAU_RBAR_BADDR_Pos                  5U
#define SAU_RBAR_BADDR_Msk                 (0x7FFFFFFUL << SAU_RBAR_BADDR_Pos)


#define SAU_RLAR_LADDR_Pos                  5U
#define SAU_RLAR_LADDR_Msk                 (0x7FFFFFFUL << SAU_RLAR_LADDR_Pos)

#define SAU_RLAR_NSC_Pos                    1U
#define SAU_RLAR_NSC_Msk                   (1UL << SAU_RLAR_NSC_Pos)

#define SAU_RLAR_ENABLE_Pos                 0U
#define SAU_RLAR_ENABLE_Msk                (1UL

#endif


#define SAU_SFSR_LSERR_Pos                  7U
#define SAU_SFSR_LSERR_Msk                 (1UL << SAU_SFSR_LSERR_Pos)

#define SAU_SFSR_SFARVALID_Pos              6U
#define SAU_SFSR_SFARVALID_Msk             (1UL << SAU_SFSR_SFARVALID_Pos)

#define SAU_SFSR_LSPERR_Pos                 5U
#define SAU_SFSR_LSPERR_Msk                (1UL << SAU_SFSR_LSPERR_Pos)

#define SAU_SFSR_INVTRAN_Pos                4U
#define SAU_SFSR_INVTRAN_Msk               (1UL << SAU_SFSR_INVTRAN_Pos)

#define SAU_SFSR_AUVIOL_Pos                 3U
#define SAU_SFSR_AUVIOL_Msk                (1UL << SAU_SFSR_AUVIOL_Pos)

#define SAU_SFSR_INVER_Pos                  2U
#define SAU_SFSR_INVER_Msk                 (1UL << SAU_SFSR_INVER_Pos)

#define SAU_SFSR_INVIS_Pos                  1U
#define SAU_SFSR_INVIS_Msk                 (1UL << SAU_SFSR_INVIS_Pos)

#define SAU_SFSR_INVEP_Pos                  0U
#define SAU_SFSR_INVEP_Msk                 (1UL


#endif


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_FPU     Floating Point Unit (FPU)
  \brief    Type definitions for the Floating Point Unit (FPU)
  @{
 */

/**
  \brief  Structure type to access the Floating Point Unit (FPU).
 */
typedef struct
{
        uint32_t RESERVED0[1U];
  __IOM uint32_t FPCCR;
  __IOM uint32_t FPCAR;
  __IOM uint32_t FPDSCR;
  __IM  uint32_t MVFR0;
  __IM  uint32_t MVFR1;
} FPU_Type;


#define FPU_FPCCR_ASPEN_Pos                31U
#define FPU_FPCCR_ASPEN_Msk                (1UL << FPU_FPCCR_ASPEN_Pos)

#define FPU_FPCCR_LSPEN_Pos                30U
#define FPU_FPCCR_LSPEN_Msk                (1UL << FPU_FPCCR_LSPEN_Pos)

#define FPU_FPCCR_LSPENS_Pos               29U
#define FPU_FPCCR_LSPENS_Msk               (1UL << FPU_FPCCR_LSPENS_Pos)

#define FPU_FPCCR_CLRONRET_Pos             28U
#define FPU_FPCCR_CLRONRET_Msk             (1UL << FPU_FPCCR_CLRONRET_Pos)

#define FPU_FPCCR_CLRONRETS_Pos            27U
#define FPU_FPCCR_CLRONRETS_Msk            (1UL << FPU_FPCCR_CLRONRETS_Pos)

#define FPU_FPCCR_TS_Pos                   26U
#define FPU_FPCCR_TS_Msk                   (1UL << FPU_FPCCR_TS_Pos)

#define FPU_FPCCR_UFRDY_Pos                10U
#define FPU_FPCCR_UFRDY_Msk                (1UL << FPU_FPCCR_UFRDY_Pos)

#define FPU_FPCCR_SPLIMVIOL_Pos             9U
#define FPU_FPCCR_SPLIMVIOL_Msk            (1UL << FPU_FPCCR_SPLIMVIOL_Pos)

#define FPU_FPCCR_MONRDY_Pos                8U
#define FPU_FPCCR_MONRDY_Msk               (1UL << FPU_FPCCR_MONRDY_Pos)

#define FPU_FPCCR_SFRDY_Pos                 7U
#define FPU_FPCCR_SFRDY_Msk                (1UL << FPU_FPCCR_SFRDY_Pos)

#define FPU_FPCCR_BFRDY_Pos                 6U
#define FPU_FPCCR_BFRDY_Msk                (1UL << FPU_FPCCR_BFRDY_Pos)

#define FPU_FPCCR_MMRDY_Pos                 5U
#define FPU_FPCCR_MMRDY_Msk                (1UL << FPU_FPCCR_MMRDY_Pos)

#define FPU_FPCCR_HFRDY_Pos                 4U
#define FPU_FPCCR_HFRDY_Msk                (1UL << FPU_FPCCR_HFRDY_Pos)

#define FPU_FPCCR_THREAD_Pos                3U
#define FPU_FPCCR_THREAD_Msk               (1UL << FPU_FPCCR_THREAD_Pos)

#define FPU_FPCCR_S_Pos                     2U
#define FPU_FPCCR_S_Msk                    (1UL << FPU_FPCCR_S_Pos)

#define FPU_FPCCR_USER_Pos                  1U
#define FPU_FPCCR_USER_Msk                 (1UL << FPU_FPCCR_USER_Pos)

#define FPU_FPCCR_LSPACT_Pos                0U
#define FPU_FPCCR_LSPACT_Msk               (1UL


#define FPU_FPCAR_ADDRESS_Pos               3U
#define FPU_FPCAR_ADDRESS_Msk              (0x1FFFFFFFUL << FPU_FPCAR_ADDRESS_Pos)


#define FPU_FPDSCR_AHP_Pos                 26U
#define FPU_FPDSCR_AHP_Msk                 (1UL << FPU_FPDSCR_AHP_Pos)

#define FPU_FPDSCR_DN_Pos                  25U
#define FPU_FPDSCR_DN_Msk                  (1UL << FPU_FPDSCR_DN_Pos)

#define FPU_FPDSCR_FZ_Pos                  24U
#define FPU_FPDSCR_FZ_Msk                  (1UL << FPU_FPDSCR_FZ_Pos)

#define FPU_FPDSCR_RMode_Pos               22U
#define FPU_FPDSCR_RMode_Msk               (3UL << FPU_FPDSCR_RMode_Pos)


#define FPU_MVFR0_FP_rounding_modes_Pos    28U
#define FPU_MVFR0_FP_rounding_modes_Msk    (0xFUL << FPU_MVFR0_FP_rounding_modes_Pos)

#define FPU_MVFR0_Short_vectors_Pos        24U
#define FPU_MVFR0_Short_vectors_Msk        (0xFUL << FPU_MVFR0_Short_vectors_Pos)

#define FPU_MVFR0_Square_root_Pos          20U
#define FPU_MVFR0_Square_root_Msk          (0xFUL << FPU_MVFR0_Square_root_Pos)

#define FPU_MVFR0_Divide_Pos               16U
#define FPU_MVFR0_Divide_Msk               (0xFUL << FPU_MVFR0_Divide_Pos)

#define FPU_MVFR0_FP_excep_trapping_Pos    12U
#define FPU_MVFR0_FP_excep_trapping_Msk    (0xFUL << FPU_MVFR0_FP_excep_trapping_Pos)

#define FPU_MVFR0_Double_precision_Pos      8U
#define FPU_MVFR0_Double_precision_Msk     (0xFUL << FPU_MVFR0_Double_precision_Pos)

#define FPU_MVFR0_Single_precision_Pos      4U
#define FPU_MVFR0_Single_precision_Msk     (0xFUL << FPU_MVFR0_Single_precision_Pos)

#define FPU_MVFR0_A_SIMD_registers_Pos      0U
#define FPU_MVFR0_A_SIMD_registers_Msk     (0xFUL


#define FPU_MVFR1_FP_fused_MAC_Pos         28U
#define FPU_MVFR1_FP_fused_MAC_Msk         (0xFUL << FPU_MVFR1_FP_fused_MAC_Pos)

#define FPU_MVFR1_FP_HPFP_Pos              24U
#define FPU_MVFR1_FP_HPFP_Msk              (0xFUL << FPU_MVFR1_FP_HPFP_Pos)

#define FPU_MVFR1_D_NaN_mode_Pos            4U
#define FPU_MVFR1_D_NaN_mode_Msk           (0xFUL << FPU_MVFR1_D_NaN_mode_Pos)

#define FPU_MVFR1_FtZ_mode_Pos              0U
#define FPU_MVFR1_FtZ_mode_Msk             (0xFUL




/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_CoreDebug       Core Debug Registers (CoreDebug)
  \brief    Type definitions for the Core Debug Registers
  @{
 */

/**
  \brief  Structure type to access the Core Debug Register (CoreDebug).
 */
typedef struct
{
  __IOM uint32_t DHCSR;
  __OM  uint32_t DCRSR;
  __IOM uint32_t DCRDR;
  __IOM uint32_t DEMCR;
        uint32_t RESERVED4[1U];
  __IOM uint32_t DAUTHCTRL;
  __IOM uint32_t DSCSR;
} CoreDebug_Type;


#define CoreDebug_DHCSR_DBGKEY_Pos         16U
#define CoreDebug_DHCSR_DBGKEY_Msk         (0xFFFFUL << CoreDebug_DHCSR_DBGKEY_Pos)

#define CoreDebug_DHCSR_S_RESTART_ST_Pos   26U
#define CoreDebug_DHCSR_S_RESTART_ST_Msk   (1UL << CoreDebug_DHCSR_S_RESTART_ST_Pos)

#define CoreDebug_DHCSR_S_RESET_ST_Pos     25U
#define CoreDebug_DHCSR_S_RESET_ST_Msk     (1UL << CoreDebug_DHCSR_S_RESET_ST_Pos)

#define CoreDebug_DHCSR_S_RETIRE_ST_Pos    24U
#define CoreDebug_DHCSR_S_RETIRE_ST_Msk    (1UL << CoreDebug_DHCSR_S_RETIRE_ST_Pos)

#define CoreDebug_DHCSR_S_LOCKUP_Pos       19U
#define CoreDebug_DHCSR_S_LOCKUP_Msk       (1UL << CoreDebug_DHCSR_S_LOCKUP_Pos)

#define CoreDebug_DHCSR_S_SLEEP_Pos        18U
#define CoreDebug_DHCSR_S_SLEEP_Msk        (1UL << CoreDebug_DHCSR_S_SLEEP_Pos)

#define CoreDebug_DHCSR_S_HALT_Pos         17U
#define CoreDebug_DHCSR_S_HALT_Msk         (1UL << CoreDebug_DHCSR_S_HALT_Pos)

#define CoreDebug_DHCSR_S_REGRDY_Pos       16U
#define CoreDebug_DHCSR_S_REGRDY_Msk       (1UL << CoreDebug_DHCSR_S_REGRDY_Pos)

#define CoreDebug_DHCSR_C_SNAPSTALL_Pos     5U
#define CoreDebug_DHCSR_C_SNAPSTALL_Msk    (1UL << CoreDebug_DHCSR_C_SNAPSTALL_Pos)

#define CoreDebug_DHCSR_C_MASKINTS_Pos      3U
#define CoreDebug_DHCSR_C_MASKINTS_Msk     (1UL << CoreDebug_DHCSR_C_MASKINTS_Pos)

#define CoreDebug_DHCSR_C_STEP_Pos          2U
#define CoreDebug_DHCSR_C_STEP_Msk         (1UL << CoreDebug_DHCSR_C_STEP_Pos)

#define CoreDebug_DHCSR_C_HALT_Pos          1U
#define CoreDebug_DHCSR_C_HALT_Msk         (1UL << CoreDebug_DHCSR_C_HALT_Pos)

#define CoreDebug_DHCSR_C_DEBUGEN_Pos       0U
#define CoreDebug_DHCSR_C_DEBUGEN_Msk      (1UL


#define CoreDebug_DCRSR_REGWnR_Pos         16U
#define CoreDebug_DCRSR_REGWnR_Msk         (1UL << CoreDebug_DCRSR_REGWnR_Pos)

#define CoreDebug_DCRSR_REGSEL_Pos          0U
#define CoreDebug_DCRSR_REGSEL_Msk         (0x1FUL


#define CoreDebug_DEMCR_TRCENA_Pos         24U
#define CoreDebug_DEMCR_TRCENA_Msk         (1UL << CoreDebug_DEMCR_TRCENA_Pos)

#define CoreDebug_DEMCR_MON_REQ_Pos        19U
#define CoreDebug_DEMCR_MON_REQ_Msk        (1UL << CoreDebug_DEMCR_MON_REQ_Pos)

#define CoreDebug_DEMCR_MON_STEP_Pos       18U
#define CoreDebug_DEMCR_MON_STEP_Msk       (1UL << CoreDebug_DEMCR_MON_STEP_Pos)

#define CoreDebug_DEMCR_MON_PEND_Pos       17U
#define CoreDebug_DEMCR_MON_PEND_Msk       (1UL << CoreDebug_DEMCR_MON_PEND_Pos)

#define CoreDebug_DEMCR_MON_EN_Pos         16U
#define CoreDebug_DEMCR_MON_EN_Msk         (1UL << CoreDebug_DEMCR_MON_EN_Pos)

#define CoreDebug_DEMCR_VC_HARDERR_Pos     10U
#define CoreDebug_DEMCR_VC_HARDERR_Msk     (1UL << CoreDebug_DEMCR_VC_HARDERR_Pos)

#define CoreDebug_DEMCR_VC_INTERR_Pos       9U
#define CoreDebug_DEMCR_VC_INTERR_Msk      (1UL << CoreDebug_DEMCR_VC_INTERR_Pos)

#define CoreDebug_DEMCR_VC_BUSERR_Pos       8U
#define CoreDebug_DEMCR_VC_BUSERR_Msk      (1UL << CoreDebug_DEMCR_VC_BUSERR_Pos)

#define CoreDebug_DEMCR_VC_STATERR_Pos      7U
#define CoreDebug_DEMCR_VC_STATERR_Msk     (1UL << CoreDebug_DEMCR_VC_STATERR_Pos)

#define CoreDebug_DEMCR_VC_CHKERR_Pos       6U
#define CoreDebug_DEMCR_VC_CHKERR_Msk      (1UL << CoreDebug_DEMCR_VC_CHKERR_Pos)

#define CoreDebug_DEMCR_VC_NOCPERR_Pos      5U
#define CoreDebug_DEMCR_VC_NOCPERR_Msk     (1UL << CoreDebug_DEMCR_VC_NOCPERR_Pos)

#define CoreDebug_DEMCR_VC_MMERR_Pos        4U
#define CoreDebug_DEMCR_VC_MMERR_Msk       (1UL << CoreDebug_DEMCR_VC_MMERR_Pos)

#define CoreDebug_DEMCR_VC_CORERESET_Pos    0U
#define CoreDebug_DEMCR_VC_CORERESET_Msk   (1UL


#define CoreDebug_DAUTHCTRL_INTSPNIDEN_Pos  3U
#define CoreDebug_DAUTHCTRL_INTSPNIDEN_Msk (1UL << CoreDebug_DAUTHCTRL_INTSPNIDEN_Pos)

#define CoreDebug_DAUTHCTRL_SPNIDENSEL_Pos  2U
#define CoreDebug_DAUTHCTRL_SPNIDENSEL_Msk (1UL << CoreDebug_DAUTHCTRL_SPNIDENSEL_Pos)

#define CoreDebug_DAUTHCTRL_INTSPIDEN_Pos   1U
#define CoreDebug_DAUTHCTRL_INTSPIDEN_Msk  (1UL << CoreDebug_DAUTHCTRL_INTSPIDEN_Pos)

#define CoreDebug_DAUTHCTRL_SPIDENSEL_Pos   0U
#define CoreDebug_DAUTHCTRL_SPIDENSEL_Msk  (1UL


#define CoreDebug_DSCSR_CDS_Pos            16U
#define CoreDebug_DSCSR_CDS_Msk            (1UL << CoreDebug_DSCSR_CDS_Pos)

#define CoreDebug_DSCSR_SBRSEL_Pos          1U
#define CoreDebug_DSCSR_SBRSEL_Msk         (1UL << CoreDebug_DSCSR_SBRSEL_Pos)

#define CoreDebug_DSCSR_SBRSELEN_Pos        0U
#define CoreDebug_DSCSR_SBRSELEN_Msk       (1UL




/**
  \ingroup    CMSIS_core_register
  \defgroup   CMSIS_core_bitfield     Core register bit field macros
  \brief      Macros for use with bit field definitions (xxx_Pos, xxx_Msk).
  @{
 */

/**
  \brief   Mask and shift a bit field value for use in a register bit range.
  \param[in] field  Name of the register bit field.
  \param[in] value  Value of the bit field. This parameter is interpreted as an uint32_t type.
  \return           Masked and shifted value.
*/
#define _VAL2FLD(field, value)    (((uint32_t)(value) << field ## _Pos) & field ## _Msk)

/**
  \brief     Mask and shift a register value to extract a bit filed value.
  \param[in] field  Name of the register bit field.
  \param[in] value  Value of register. This parameter is interpreted as an uint32_t type.
  \return           Masked and shifted bit field value.
*/
#define _FLD2VAL(field, value)    (((uint32_t)(value) & field ## _Msk) >> field ## _Pos)




/**
  \ingroup    CMSIS_core_register
  \defgroup   CMSIS_core_base     Core Definitions
  \brief      Definitions for base addresses, unions, and structures.
  @{
 */


  #define SCS_BASE            (0xE000E000UL)
  #define ITM_BASE            (0xE0000000UL)
  #define DWT_BASE            (0xE0001000UL)
  #define TPI_BASE            (0xE0040000UL)
  #define CoreDebug_BASE      (0xE000EDF0UL)
  #define SysTick_BASE        (SCS_BASE +  0x0010UL)
  #define NVIC_BASE           (SCS_BASE +  0x0100UL)
  #define SCB_BASE            (SCS_BASE +  0x0D00UL)

  #define SCnSCB              ((SCnSCB_Type    *)     SCS_BASE         )
  #define SCB                 ((SCB_Type       *)     SCB_BASE         )
  #define SysTick             ((SysTick_Type   *)     SysTick_BASE     )
  #define NVIC                ((NVIC_Type      *)     NVIC_BASE        )
  #define ITM                 ((ITM_Type       *)     ITM_BASE         )
  #define DWT                 ((DWT_Type       *)     DWT_BASE         )
  #define TPI                 ((TPI_Type       *)     TPI_BASE         )
  #define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE   )

  #if defined (__MPU_PRESENT) && (__MPU_PRESENT == 1U)
    #define MPU_BASE          (SCS_BASE +  0x0D90UL)
    #define MPU               ((MPU_Type       *)     MPU_BASE         )
  #endif

  #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    #define SAU_BASE          (SCS_BASE +  0x0DD0UL)
    #define SAU               ((SAU_Type       *)     SAU_BASE         )
  #endif

  #define FPU_BASE            (SCS_BASE +  0x0F30UL)
  #define FPU                 ((FPU_Type       *)     FPU_BASE         )

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
  #define SCS_BASE_NS         (0xE002E000UL)
  #define CoreDebug_BASE_NS   (0xE002EDF0UL)
  #define SysTick_BASE_NS     (SCS_BASE_NS +  0x0010UL)
  #define NVIC_BASE_NS        (SCS_BASE_NS +  0x0100UL)
  #define SCB_BASE_NS         (SCS_BASE_NS +  0x0D00UL)

  #define SCnSCB_NS           ((SCnSCB_Type    *)     SCS_BASE_NS      )
  #define SCB_NS              ((SCB_Type       *)     SCB_BASE_NS      )
  #define SysTick_NS          ((SysTick_Type   *)     SysTick_BASE_NS  )
  #define NVIC_NS             ((NVIC_Type      *)     NVIC_BASE_NS     )
  #define CoreDebug_NS        ((CoreDebug_Type *)     CoreDebug_BASE_NS)

  #if defined (__MPU_PRESENT) && (__MPU_PRESENT == 1U)
    #define MPU_BASE_NS       (SCS_BASE_NS +  0x0D90UL)
    #define MPU_NS            ((MPU_Type       *)     MPU_BASE_NS      )
  #endif

  #define FPU_BASE_NS         (SCS_BASE_NS +  0x0F30UL)
  #define FPU_NS              ((FPU_Type       *)     FPU_BASE_NS      )

#endif




/*******************************************************************************
 *                Hardware Abstraction Layer
  Core Function Interface contains:
  - Core NVIC Functions
  - Core SysTick Functions
  - Core Debug Functions
  - Core Register Access Functions
 ******************************************************************************/
/**
  \defgroup CMSIS_Core_FunctionInterface Functions and Instructions Reference
*/




/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_NVICFunctions NVIC Functions
  \brief    Functions that manage interrupts and exceptions via the NVIC.
  @{
 */

#ifdef CMSIS_NVIC_VIRTUAL
  #ifndef CMSIS_NVIC_VIRTUAL_HEADER_FILE
    #define CMSIS_NVIC_VIRTUAL_HEADER_FILE "cmsis_nvic_virtual.h"
  #endif
  #include CMSIS_NVIC_VIRTUAL_HEADER_FILE
#else
  #define NVIC_SetPriorityGrouping    __NVIC_SetPriorityGrouping
  #define NVIC_GetPriorityGrouping    __NVIC_GetPriorityGrouping
  #define NVIC_EnableIRQ              __NVIC_EnableIRQ
  #define NVIC_GetEnableIRQ           __NVIC_GetEnableIRQ
  #define NVIC_DisableIRQ             __NVIC_DisableIRQ
  #define NVIC_GetPendingIRQ          __NVIC_GetPendingIRQ
  #define NVIC_SetPendingIRQ          __NVIC_SetPendingIRQ
  #define NVIC_ClearPendingIRQ        __NVIC_ClearPendingIRQ
  #define NVIC_GetActive              __NVIC_GetActive
  #define NVIC_SetPriority            __NVIC_SetPriority
  #define NVIC_GetPriority            __NVIC_GetPriority
  #define NVIC_SystemReset            __NVIC_SystemReset
#endif

#ifdef CMSIS_VECTAB_VIRTUAL
  #ifndef CMSIS_VECTAB_VIRTUAL_HEADER_FILE
    #define CMSIS_VECTAB_VIRTUAL_HEADER_FILE "cmsis_vectab_virtual.h"
  #endif
  #include CMSIS_VECTAB_VIRTUAL_HEADER_FILE
#else
  #define NVIC_SetVector              __NVIC_SetVector
  #define NVIC_GetVector              __NVIC_GetVector
#endif

#define NVIC_USER_IRQ_OFFSET          16





#define FNC_RETURN                 (0xFEFFFFFFUL)


#define EXC_RETURN_PREFIX          (0xFF000000UL)
#define EXC_RETURN_S               (0x00000040UL)
#define EXC_RETURN_DCRS            (0x00000020UL)
#define EXC_RETURN_FTYPE           (0x00000010UL)
#define EXC_RETURN_MODE            (0x00000008UL)
#define EXC_RETURN_SPSEL           (0x00000004UL)
#define EXC_RETURN_ES              (0x00000001UL)


#if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
#define EXC_INTEGRITY_SIGNATURE     (0xFEFA125AUL)
#else
#define EXC_INTEGRITY_SIGNATURE     (0xFEFA125BUL)
#endif


/**
  \brief   Set Priority Grouping
  \details Sets the priority grouping field using the required unlock sequence.
           The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8] PRIGROUP field.
           Only values from 0..7 are used.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]      PriorityGroup  Priority grouping field.
 */
__STATIC_INLINE void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);

  reg_value  =  SCB->AIRCR;
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk));
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                (PriorityGroupTmp << SCB_AIRCR_PRIGROUP_Pos)  );
  SCB->AIRCR =  reg_value;
}


/**
  \brief   Get Priority Grouping
  \details Reads the priority grouping field from the NVIC Interrupt Controller.
  \return                Priority grouping field (SCB->AIRCR [10:8] PRIGROUP field).
 */
__STATIC_INLINE uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
}


/**
  \brief   Enable Interrupt
  \details Enables a device specific interrupt in the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __COMPILER_BARRIER();
    NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __COMPILER_BARRIER();
  }
}


/**
  \brief   Get Interrupt Enable status
  \details Returns a device specific interrupt enable status from the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt is not enabled.
  \return             1  Interrupt is enabled.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Disable Interrupt
  \details Disables a device specific interrupt in the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __DSB();
    __ISB();
  }
}


/**
  \brief   Get Pending Interrupt
  \details Reads the NVIC pending register and returns the pending bit for the specified device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not pending.
  \return             1  Interrupt status is pending.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Set Pending Interrupt
  \details Sets the pending bit of a device specific interrupt in the NVIC pending register.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Clear Pending Interrupt
  \details Clears the pending bit of a device specific interrupt in the NVIC pending register.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Get Active Interrupt
  \details Reads the active register in the NVIC and returns the active bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not active.
  \return             1  Interrupt status is active.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \brief   Get Interrupt Target State
  \details Reads the interrupt target field in the NVIC and returns the interrupt target bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  if interrupt is assigned to Secure
  \return             1  if interrupt is assigned to Non Secure
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t NVIC_GetTargetState(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Set Interrupt Target State
  \details Sets the interrupt target field in the NVIC and returns the interrupt target bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  if interrupt is assigned to Secure
                      1  if interrupt is assigned to Non Secure
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t NVIC_SetTargetState(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] |=  ((uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL)));
    return((uint32_t)(((NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Clear Interrupt Target State
  \details Clears the interrupt target field in the NVIC and returns the interrupt target bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  if interrupt is assigned to Secure
                      1  if interrupt is assigned to Non Secure
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t NVIC_ClearTargetState(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] &= ~((uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL)));
    return((uint32_t)(((NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
#endif


/**
  \brief   Set Interrupt Priority
  \details Sets the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]      IRQn  Interrupt number.
  \param [in]  priority  Priority to set.
  \note    The priority cannot be set for every processor exception.
 */
__STATIC_INLINE void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->IPR[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
  else
  {
    SCB->SHPR[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}


/**
  \brief   Get Interrupt Priority
  \details Reads the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   IRQn  Interrupt number.
  \return             Interrupt Priority.
                      Value is aligned automatically to the implemented priority bits of the microcontroller.
 */
__STATIC_INLINE uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)NVIC->IPR[((uint32_t)IRQn)]               >> (8U - __NVIC_PRIO_BITS)));
  }
  else
  {
    return(((uint32_t)SCB->SHPR[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - __NVIC_PRIO_BITS)));
  }
}


/**
  \brief   Encode Priority
  \details Encodes the priority for an interrupt with the given priority group,
           preemptive priority value, and subpriority value.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]     PriorityGroup  Used priority group.
  \param [in]   PreemptPriority  Preemptive priority value (starting from 0).
  \param [in]       SubPriority  Subpriority value (starting from 0).
  \return                        Encoded priority. Value can be used in the function \ref NVIC_SetPriority().
 */
__STATIC_INLINE uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}


/**
  \brief   Decode Priority
  \details Decodes an interrupt priority value with a given priority group to
           preemptive priority value and subpriority value.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS) the smallest possible priority group is set.
  \param [in]         Priority   Priority value, which can be retrieved with the function \ref NVIC_GetPriority().
  \param [in]     PriorityGroup  Used priority group.
  \param [out] pPreemptPriority  Preemptive priority value (starting from 0).
  \param [out]     pSubPriority  Subpriority value (starting from 0).
 */
__STATIC_INLINE void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}


/**
  \brief   Set Interrupt Vector
  \details Sets an interrupt vector in SRAM based interrupt vector table.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
           VTOR must been relocated to SRAM before.
  \param [in]   IRQn      Interrupt number
  \param [in]   vector    Address of interrupt handler function
 */
__STATIC_INLINE void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t *vectors = (uint32_t *)SCB->VTOR;
  vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET] = vector;
  __DSB();
}


/**
  \brief   Get Interrupt Vector
  \details Reads an interrupt vector from interrupt vector table.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   IRQn      Interrupt number.
  \return                 Address of interrupt handler function
 */
__STATIC_INLINE uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t *vectors = (uint32_t *)SCB->VTOR;
  return vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET];
}


/**
  \brief   System Reset
  \details Initiates a system reset request to reset the MCU.
 */
__NO_RETURN __STATIC_INLINE void __NVIC_SystemReset(void)
{
  __DSB();                                                          /* Ensure all outstanding memory accesses included
                                                                       buffered write are completed before reset */
  SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
                           (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                            SCB_AIRCR_SYSRESETREQ_Msk    );
  __DSB();

  for(;;)
  {
    __NOP();
  }
}

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \brief   Set Priority Grouping (non-secure)
  \details Sets the non-secure priority grouping field when in secure state using the required unlock sequence.
           The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8] PRIGROUP field.
           Only values from 0..7 are used.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]      PriorityGroup  Priority grouping field.
 */
__STATIC_INLINE void TZ_NVIC_SetPriorityGrouping_NS(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);

  reg_value  =  SCB_NS->AIRCR;
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk));
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                (PriorityGroupTmp << SCB_AIRCR_PRIGROUP_Pos)                      );
  SCB_NS->AIRCR =  reg_value;
}


/**
  \brief   Get Priority Grouping (non-secure)
  \details Reads the priority grouping field from the non-secure NVIC when in secure state.
  \return                Priority grouping field (SCB->AIRCR [10:8] PRIGROUP field).
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetPriorityGrouping_NS(void)
{
  return ((uint32_t)((SCB_NS->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
}


/**
  \brief   Enable Interrupt (non-secure)
  \details Enables a device specific interrupt in the non-secure NVIC interrupt controller when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void TZ_NVIC_EnableIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Get Interrupt Enable status (non-secure)
  \details Returns a device specific interrupt enable status from the non-secure NVIC interrupt controller when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt is not enabled.
  \return             1  Interrupt is enabled.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetEnableIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC_NS->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Disable Interrupt (non-secure)
  \details Disables a device specific interrupt in the non-secure NVIC interrupt controller when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void TZ_NVIC_DisableIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Get Pending Interrupt (non-secure)
  \details Reads the NVIC pending register in the non-secure NVIC when in secure state and returns the pending bit for the specified device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not pending.
  \return             1  Interrupt status is pending.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetPendingIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC_NS->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Set Pending Interrupt (non-secure)
  \details Sets the pending bit of a device specific interrupt in the non-secure NVIC pending register when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void TZ_NVIC_SetPendingIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Clear Pending Interrupt (non-secure)
  \details Clears the pending bit of a device specific interrupt in the non-secure NVIC pending register when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void TZ_NVIC_ClearPendingIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Get Active Interrupt (non-secure)
  \details Reads the active register in non-secure NVIC when in secure state and returns the active bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not active.
  \return             1  Interrupt status is active.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetActive_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC_NS->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Set Interrupt Priority (non-secure)
  \details Sets the priority of a non-secure device specific interrupt or a non-secure processor exception when in secure state.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]      IRQn  Interrupt number.
  \param [in]  priority  Priority to set.
  \note    The priority cannot be set for every non-secure processor exception.
 */
__STATIC_INLINE void TZ_NVIC_SetPriority_NS(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->IPR[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
  else
  {
    SCB_NS->SHPR[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}


/**
  \brief   Get Interrupt Priority (non-secure)
  \details Reads the priority of a non-secure device specific interrupt or a non-secure processor exception when in secure state.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   IRQn  Interrupt number.
  \return             Interrupt Priority. Value is aligned automatically to the implemented priority bits of the microcontroller.
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetPriority_NS(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)NVIC_NS->IPR[((uint32_t)IRQn)]               >> (8U - __NVIC_PRIO_BITS)));
  }
  else
  {
    return(((uint32_t)SCB_NS->SHPR[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - __NVIC_PRIO_BITS)));
  }
}
#endif





#if defined (__MPU_PRESENT) && (__MPU_PRESENT == 1U)

#include "mpu_armv8.h"

#endif


/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_FpuFunctions FPU Functions
  \brief    Function that provides FPU type.
  @{
 */

/**
  \brief   get FPU type
  \details returns the FPU type
  \returns
   - \b  0: No FPU
   - \b  1: Single precision FPU
   - \b  2: Double + Single precision FPU
 */
__STATIC_INLINE uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = FPU->MVFR0;
  if      ((mvfr0 & (FPU_MVFR0_Single_precision_Msk | FPU_MVFR0_Double_precision_Msk)) == 0x220U)
  {
    return 2U;
  }
  else if ((mvfr0 & (FPU_MVFR0_Single_precision_Msk | FPU_MVFR0_Double_precision_Msk)) == 0x020U)
  {
    return 1U;
  }
  else
  {
    return 0U;
  }
}







/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_SAUFunctions SAU Functions
  \brief    Functions that configure the SAU.
  @{
 */

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)

/**
  \brief   Enable SAU
  \details Enables the Security Attribution Unit (SAU).
 */
__STATIC_INLINE void TZ_SAU_Enable(void)
{
    SAU->CTRL |=  (SAU_CTRL_ENABLE_Msk);
}



/**
  \brief   Disable SAU
  \details Disables the Security Attribution Unit (SAU).
 */
__STATIC_INLINE void TZ_SAU_Disable(void)
{
    SAU->CTRL &= ~(SAU_CTRL_ENABLE_Msk);
}

#endif







/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_SysTickFunctions SysTick Functions
  \brief    Functions that configure the System.
  @{
 */

#if defined (__Vendor_SysTickConfig) && (__Vendor_SysTickConfig == 0U)

/**
  \brief   System Tick Configuration
  \details Initializes the System Timer and its interrupt, and starts the System Tick Timer.
           Counter is in free running mode to generate periodic interrupts.
  \param [in]  ticks  Number of ticks between two interrupts.
  \return          0  Function succeeded.
  \return          1  Function failed.
  \note    When the variable <b>__Vendor_SysTickConfig</b> is set to 1, then the
           function <b>SysTick_Config</b> is not included. In this case, the file <b><i>device</i>.h</b>
           must contain a vendor-specific implementation of this function.
 */
__STATIC_INLINE uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);
  }

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
  SysTick->VAL   = 0UL;
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;
  return (0UL);
}

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \brief   System Tick Configuration (non-secure)
  \details Initializes the non-secure System Timer and its interrupt when in secure state, and starts the System Tick Timer.
           Counter is in free running mode to generate periodic interrupts.
  \param [in]  ticks  Number of ticks between two interrupts.
  \return          0  Function succeeded.
  \return          1  Function failed.
  \note    When the variable <b>__Vendor_SysTickConfig</b> is set to 1, then the
           function <b>TZ_SysTick_Config_NS</b> is not included. In this case, the file <b><i>device</i>.h</b>
           must contain a vendor-specific implementation of this function.

 */
__STATIC_INLINE uint32_t TZ_SysTick_Config_NS(uint32_t ticks)
{
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);
  }

  SysTick_NS->LOAD  = (uint32_t)(ticks - 1UL);
  TZ_NVIC_SetPriority_NS (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
  SysTick_NS->VAL   = 0UL;
  SysTick_NS->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                      SysTick_CTRL_TICKINT_Msk   |
                      SysTick_CTRL_ENABLE_Msk;
  return (0UL);
}
#endif

#endif






/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_core_DebugFunctions ITM Functions
  \brief    Functions that access the ITM debug interface.
  @{
 */

extern volatile int32_t ITM_RxBuffer;
#define                 ITM_RXBUFFER_EMPTY  ((int32_t)0x5AA55AA5U)


/**
  \brief   ITM Send Character
  \details Transmits a character via the ITM channel 0, and
           \li Just returns when no debugger is connected that has booked the output.
           \li Is blocking when a debugger is connected, but the previous character sent has not been transmitted.
  \param [in]     ch  Character to transmit.
  \returns            Character to transmit.
 */
__STATIC_INLINE uint32_t ITM_SendChar (uint32_t ch)
{
  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&
      ((ITM->TER & 1UL               ) != 0UL)   )
  {
    while (ITM->PORT[0U].u32 == 0UL)
    {
      __NOP();
    }
    ITM->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}


/**
  \brief   ITM Receive Character
  \details Inputs a character via the external variable \ref ITM_RxBuffer.
  \return             Received character.
  \return         -1  No character pending.
 */
__STATIC_INLINE int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;

  if (ITM_RxBuffer != ITM_RXBUFFER_EMPTY)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ITM_RXBUFFER_EMPTY;
  }

  return (ch);
}


/**
  \brief   ITM Check Character
  \details Checks whether a character is pending for reading in the variable \ref ITM_RxBuffer.
  \return          0  No character available.
  \return          1  Character available.
 */
__STATIC_INLINE int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ITM_RXBUFFER_EMPTY)
  {
    return (0);
  }
  else
  {
    return (1);
  }
}






#ifdef __cplusplus
}
#endif

#endif

#endif
