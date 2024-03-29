/**
  ******************************************************************************
  * @file    stm32l4xx_ll_fmc.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    16-September-2015
  * @brief   FMC Low Layer HAL module driver. 
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Flexible Memory Controller (FMC) peripheral memories:
  *           + Initialization/de-initialization functions
  *           + Peripheral Control functions 
  *           + Peripheral State functions
  *         
  @verbatim
  ==============================================================================
                        ##### FMC peripheral features #####
  ==============================================================================
  [..] The Flexible memory controller (FMC) includes following memory controllers:
       (+) The NOR/PSRAM memory controller
       (+) The NAND memory controller
       
  [..] The FMC functional block makes the interface with synchronous and asynchronous static
       memories and 16-bit PC memory cards. Its main purposes are:
       (+) to translate AHB transactions into the appropriate external device protocol.
       (+) to meet the access time requirements of the external memory devices.
   
  [..] All external memories share the addresses, data and control signals with the controller.
       Each external device is accessed by means of a unique Chip Select. The FMC performs
       only one access at a time to an external device.
       The main features of the FMC controller are the following:
        (+) Interface with static-memory mapped devices including:
           (++) Static random access memory (SRAM).
           (++) NOR Flash memory.
           (++) PSRAM (4 memory banks).
           (++) Two banks of NAND Flash memory with ECC hardware to check up to 8 Kbytes of
                data
        (+) Independent Chip Select control for each memory bank
        (+) Independent configuration for each memory bank
                    
  @endverbatim
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

#if defined (HAL_SRAM_MODULE_ENABLED) || defined(HAL_NOR_MODULE_ENABLED) || defined(HAL_NAND_MODULE_ENABLED)

/** @defgroup FMC_LL FMC Low Layer
  * @brief FMC driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup FMC_LL_Private_Constants FMC Low Layer Private Constants
  * @{
  */

/* ----------------------- FMC registers bit mask --------------------------- */
/* --- BCRx Register ---*/
/* BCRx register clear mask */ 
#define BCRx_CLEAR_MASK                 ((uint32_t)(FMC_BCRx_MBKEN   | FMC_BCRx_MUXEN     |\
                                                    FMC_BCRx_MTYP    | FMC_BCRx_MWID      |\
                                                    FMC_BCRx_FACCEN  | FMC_BCRx_BURSTEN   |\
                                                    FMC_BCRx_WAITPOL | FMC_BCRx_WAITCFG   |\
                                                    FMC_BCRx_WREN    | FMC_BCRx_WAITEN    |\
                                                    FMC_BCRx_EXTMOD  | FMC_BCRx_ASYNCWAIT |\
                                                    FMC_BCRx_CPSIZE  | FMC_BCRx_CBURSTRW))
/* --- BTRx Register ---*/
/* BTRx register clear mask */ 
#define BTRx_CLEAR_MASK                 ((uint32_t)(FMC_BTRx_ADDSET | FMC_BTRx_ADDHLD  |\
                                                    FMC_BTRx_DATAST | FMC_BTRx_BUSTURN |\
                                                    FMC_BTRx_CLKDIV | FMC_BTRx_DATLAT  |\
                                                    FMC_BTRx_ACCMOD))

/* --- BWTRx Register ---*/
/* BWTRx register clear mask */ 
#define BWTRx_CLEAR_MASK                ((uint32_t)(FMC_BWTRx_ADDSET | FMC_BWTRx_ADDHLD |\
                                                    FMC_BWTRx_DATAST | FMC_BWTRx_ACCMOD))

/* --- PCR Register ---*/
/* PCR register clear mask */ 
#define PCR_CLEAR_MASK                  ((uint32_t)(FMC_PCR_PWAITEN | FMC_PCR_PBKEN |\
                                                    FMC_PCR_PTYP    | FMC_PCR_PWID  |\
                                                    FMC_PCR_ECCEN   | FMC_PCR_TCLR  |\
                                                    FMC_PCR_TAR     | FMC_PCR_ECCPS))

/* --- SR Register ---*/
/* SR register clear mask */ 
#define SR_CLEAR_MASK                   ((uint32_t)(FMC_SR_FEMPT))

/* --- PMEM Register ---*/
/* PMEM register clear mask */ 
#define PMEM_CLEAR_MASK                 ((uint32_t)(FMC_PMEM_MEMSET  | FMC_PMEM_MEMWAIT |\
                                                    FMC_PMEM_MEMHOLD | FMC_PMEM_MEMHIZ))

/* --- PATT Register ---*/
/* PATT register clear mask */ 
#define PATT_CLEAR_MASK                 ((uint32_t)(FMC_PATT_ATTSET  | FMC_PATT_ATTWAIT |\
                                                    FMC_PATT_ATTHOLD | FMC_PATT_ATTHIZ))
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup FMC_LL_Private_Macros FMC Low Layer Private Macros
  * @{
  */

/**
  * @}
  */ 

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup FMC_LL_Exported_Functions FMC Low Layer Exported Functions
  * @{
  */

/** @defgroup FMC_LL_Exported_Functions_NORSRAM FMC Low Layer NOR SRAM Exported Functions
  * @brief    NORSRAM Controller functions 
  *
  @verbatim 
  ==============================================================================   
                   ##### How to use NORSRAM device driver #####
  ==============================================================================
 
  [..] 
    This driver contains a set of APIs to interface with the FMC NORSRAM banks in order
    to run the NORSRAM external devices.
      
    (+) FMC NORSRAM bank reset using the function FMC_NORSRAM_DeInit() 
    (+) FMC NORSRAM bank control configuration using the function FMC_NORSRAM_Init()
    (+) FMC NORSRAM bank timing configuration using the function FMC_NORSRAM_Timing_Init()
    (+) FMC NORSRAM bank extended timing configuration using the function 
        FMC_NORSRAM_Extended_Timing_Init()
    (+) FMC NORSRAM bank enable/disable write operation using the functions
        FMC_NORSRAM_WriteOperation_Enable()/FMC_NORSRAM_WriteOperation_Disable()
        

@endverbatim
  * @{
  */
       
/** @defgroup FMC_LL_NORSRAM_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and Configuration functions 
  *
  @verbatim    
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
  [..]  
    This section provides functions allowing to:
    (+) Initialize and configure the FMC NORSRAM interface
    (+) De-initialize the FMC NORSRAM interface 
    (+) Configure the FMC clock and associated GPIOs    
 
@endverbatim
  * @{
  */
  
/**
  * @brief  Initialize the FMC_NORSRAM device according to the specified
  *         control parameters in the FMC_NORSRAM_InitTypeDef
  * @param  Device: Pointer to NORSRAM device instance
  * @param  Init: Pointer to NORSRAM Initialization structure   
  * @retval HAL status
  */
HAL_StatusTypeDef  FMC_NORSRAM_Init(FMC_NORSRAM_TypeDef *Device, FMC_NORSRAM_InitTypeDef* Init)
{ 
  /* Check the parameters */
  assert_param(IS_FMC_NORSRAM_DEVICE(Device));
  assert_param(IS_FMC_NORSRAM_BANK(Init->NSBank));
  assert_param(IS_FMC_MUX(Init->DataAddressMux));
  assert_param(IS_FMC_MEMORY(Init->MemoryType));
  assert_param(IS_FMC_NORSRAM_MEMORY_WIDTH(Init->MemoryDataWidth));
  assert_param(IS_FMC_BURSTMODE(Init->BurstAccessMode));
  assert_param(IS_FMC_WAIT_POLARITY(Init->WaitSignalPolarity));
  assert_param(IS_FMC_WAIT_SIGNAL_ACTIVE(Init->WaitSignalActive));
  assert_param(IS_FMC_WRITE_OPERATION(Init->WriteOperation));
  assert_param(IS_FMC_WAITE_SIGNAL(Init->WaitSignal));
  assert_param(IS_FMC_EXTENDED_MODE(Init->ExtendedMode));
  assert_param(IS_FMC_ASYNWAIT(Init->AsynchronousWait));
  assert_param(IS_FMC_WRITE_BURST(Init->WriteBurst));
  assert_param(IS_FMC_CONTINOUS_CLOCK(Init->ContinuousClock)); 
  assert_param(IS_FMC_WRITE_FIFO(Init->WriteFifo));
  assert_param(IS_FMC_PAGESIZE(Init->PageSize));
  
  /* Set NORSRAM device control parameters */
  if(Init->MemoryType == FMC_MEMORY_TYPE_NOR)
  {
    MODIFY_REG(Device->BTCR[Init->NSBank], BCRx_CLEAR_MASK, (uint32_t)(FMC_NORSRAM_FLASH_ACCESS_ENABLE |\
                                                                       Init->DataAddressMux       |\
                                                                       Init->MemoryType           |\
                                                                       Init->MemoryDataWidth      |\
                                                                       Init->BurstAccessMode      |\
                                                                       Init->WaitSignalPolarity   |\
                                                                       Init->WaitSignalActive     |\
                                                                       Init->WriteOperation       |\
                                                                       Init->WaitSignal           |\
                                                                       Init->ExtendedMode         |\
                                                                       Init->AsynchronousWait     |\
                                                                       Init->WriteBurst           |\
                                                                       Init->ContinuousClock      |\
                                                                       Init->WriteFifo            |\
                                                                       Init->PageSize)
                                                                       );
  }
  else
  {
    MODIFY_REG(Device->BTCR[Init->NSBank], BCRx_CLEAR_MASK, (uint32_t)(FMC_NORSRAM_FLASH_ACCESS_DISABLE |\
                                                                       Init->DataAddressMux       |\
                                                                       Init->MemoryType           |\
                                                                       Init->MemoryDataWidth      |\
                                                                       Init->BurstAccessMode      |\
                                                                       Init->WaitSignalPolarity   |\
                                                                       Init->WaitSignalActive     |\
                                                                       Init->WriteOperation       |\
                                                                       Init->WaitSignal           |\
                                                                       Init->ExtendedMode         |\
                                                                       Init->AsynchronousWait     |\
                                                                       Init->WriteBurst           |\
                                                                       Init->ContinuousClock      |\
                                                                       Init->WriteFifo            |\
                                                                       Init->PageSize)
                                                                       );
  }

  /* Specific bits on bank1 register for bank2..4 */  
  if(Init->NSBank != FMC_NORSRAM_BANK1)
  {
    /* Configure Write FIFO mode when Write Fifo is enabled for bank2..4 */
    SET_BIT(Device->BTCR[FMC_NORSRAM_BANK1], (uint32_t)(Init->WriteFifo));

    /* Configure synchronous mode when Continuous clock is enabled for bank2..4 */
    if(Init->ContinuousClock == FMC_CONTINUOUS_CLOCK_SYNC_ASYNC)
    { 
      Init->BurstAccessMode = FMC_BURST_ACCESS_MODE_ENABLE; 
      MODIFY_REG(Device->BTCR[FMC_NORSRAM_BANK1], FMC_BCRx_BURSTEN | FMC_BCR1_CCLKEN, (uint32_t)(Init->BurstAccessMode  |\
                                                                                                 Init->ContinuousClock));                    
    }
  }

  return HAL_OK;
}


/**
  * @brief  DeInitialize the FMC_NORSRAM peripheral 
  * @param  Device: Pointer to NORSRAM device instance
  * @param  ExDevice: Pointer to NORSRAM extended mode device instance  
  * @param  Bank: NORSRAM bank number  
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NORSRAM_DeInit(FMC_NORSRAM_TypeDef *Device, FMC_NORSRAM_EXTENDED_TypeDef *ExDevice, uint32_t Bank)
{
  /* Check the parameters */
  assert_param(IS_FMC_NORSRAM_DEVICE(Device));
  assert_param(IS_FMC_NORSRAM_EXTENDED_DEVICE(ExDevice));
  assert_param(IS_FMC_NORSRAM_BANK(Bank));
  
  /* Disable the FMC_NORSRAM device */
  __FMC_NORSRAM_DISABLE(Device, Bank);
  
  /* De-initialize the FMC_NORSRAM device */
  /* FMC_NORSRAM_BANK1 */
  if(Bank == FMC_NORSRAM_BANK1)
  {
    Device->BTCR[Bank] = 0x000030DB;    
  }
  /* FMC_NORSRAM_BANK2, FMC_NORSRAM_BANK3 or FMC_NORSRAM_BANK4 */
  else
  {   
    Device->BTCR[Bank] = 0x000030D2; 
  }
  
  Device->BTCR[Bank + 1] = 0x0FFFFFFF;
  ExDevice->BWTR[Bank]   = 0x0FFFFFFF;
   
  return HAL_OK;
}


/**
  * @brief  Initialize the FMC_NORSRAM Timing according to the specified
  *         parameters in the FMC_NORSRAM_TimingTypeDef
  * @param  Device: Pointer to NORSRAM device instance
  * @param  Timing: Pointer to NORSRAM Timing structure
  * @param  Bank: NORSRAM bank number  
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NORSRAM_Timing_Init(FMC_NORSRAM_TypeDef *Device, FMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank)
{
  uint32_t tmpr = 0;
  
  /* Check the parameters */
  assert_param(IS_FMC_NORSRAM_DEVICE(Device));
  assert_param(IS_FMC_ADDRESS_SETUP_TIME(Timing->AddressSetupTime));
  assert_param(IS_FMC_ADDRESS_HOLD_TIME(Timing->AddressHoldTime));
  assert_param(IS_FMC_DATASETUP_TIME(Timing->DataSetupTime));
  assert_param(IS_FMC_TURNAROUND_TIME(Timing->BusTurnAroundDuration));
  assert_param(IS_FMC_CLK_DIV(Timing->CLKDivision));
  assert_param(IS_FMC_DATA_LATENCY(Timing->DataLatency));
  assert_param(IS_FMC_ACCESS_MODE(Timing->AccessMode));
  assert_param(IS_FMC_NORSRAM_BANK(Bank));
  
  /* Set FMC_NORSRAM device timing parameters */
  MODIFY_REG(Device->BTCR[Bank + 1],                                              \
             BTRx_CLEAR_MASK,                                                     \
             (uint32_t)(Timing->AddressSetupTime                                 |\
             ((Timing->AddressHoldTime)       << POSITION_VAL(FMC_BTRx_ADDHLD))  |\
             ((Timing->DataSetupTime)         << POSITION_VAL(FMC_BTRx_DATAST))  |\
             ((Timing->BusTurnAroundDuration) << POSITION_VAL(FMC_BTRx_BUSTURN)) |\
             (((Timing->CLKDivision)-1)       << POSITION_VAL(FMC_BTRx_CLKDIV))  |\
             (((Timing->DataLatency)-2)       << POSITION_VAL(FMC_BTRx_DATLAT))  |\
             (Timing->AccessMode)));
  
  /* Configure Clock division value (in NORSRAM bank 1) when continuous clock is enabled */
  if(HAL_IS_BIT_SET(Device->BTCR[FMC_NORSRAM_BANK1], FMC_BCR1_CCLKEN))
  {
    tmpr = (uint32_t)(Device->BTCR[FMC_NORSRAM_BANK1 + 1] & ~(((uint32_t)0x0F) << POSITION_VAL(FMC_BTRx_CLKDIV))); 
    tmpr |= (uint32_t)(((Timing->CLKDivision)-1) << POSITION_VAL(FMC_BTRx_CLKDIV));
    MODIFY_REG(Device->BTCR[FMC_NORSRAM_BANK1 + 1], FMC_BTRx_CLKDIV, tmpr);
  }

  return HAL_OK;
}

/**
  * @brief  Initialize the FMC_NORSRAM Extended mode Timing according to the specified
  *         parameters in the FMC_NORSRAM_TimingTypeDef
  * @param  Device: Pointer to NORSRAM device instance
  * @param  Timing: Pointer to NORSRAM Timing structure
  * @param  Bank: NORSRAM bank number  
  * @param  ExtendedMode: FMC Extended Mode
  *          This parameter can be one of the following values:
  *            @arg FMC_EXTENDED_MODE_DISABLE
  *            @arg FMC_EXTENDED_MODE_ENABLE
  * @retval HAL status
  */
HAL_StatusTypeDef  FMC_NORSRAM_Extended_Timing_Init(FMC_NORSRAM_EXTENDED_TypeDef *Device, FMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank, uint32_t ExtendedMode)
{  
  /* Check the parameters */
  assert_param(IS_FMC_EXTENDED_MODE(ExtendedMode));
  
  /* Set NORSRAM device timing register for write configuration, if extended mode is used */
  if(ExtendedMode == FMC_EXTENDED_MODE_ENABLE)
  {
    /* Check the parameters */  
    assert_param(IS_FMC_NORSRAM_EXTENDED_DEVICE(Device));  
    assert_param(IS_FMC_ADDRESS_SETUP_TIME(Timing->AddressSetupTime));
    assert_param(IS_FMC_ADDRESS_HOLD_TIME(Timing->AddressHoldTime));
    assert_param(IS_FMC_DATASETUP_TIME(Timing->DataSetupTime));
    assert_param(IS_FMC_ACCESS_MODE(Timing->AccessMode));
    assert_param(IS_FMC_NORSRAM_BANK(Bank));  
    
    /* Set NORSRAM device timing register for write configuration, if extended mode is used */
    MODIFY_REG(Device->BWTR[Bank],                                           \
               BWTRx_CLEAR_MASK,                                             \
               (uint32_t)(Timing->AddressSetupTime                          |\
               ((Timing->AddressHoldTime) << POSITION_VAL(FMC_BTRx_ADDHLD)) |\
               ((Timing->DataSetupTime)   << POSITION_VAL(FMC_BTRx_DATAST)) |\
               (Timing->AccessMode))); 
  }
  else                                        
  {
    Device->BWTR[Bank] = 0x0FFFFFFF;
  }   
  
  return HAL_OK;  
}


/**
  * @}
  */
  
  
/** @defgroup FMC_NORSRAM_Exported_Functions_Group2 Peripheral Control functions 
 *  @brief   management functions 
 *
@verbatim   
  ==============================================================================
                      ##### FMC_NORSRAM Control functions #####
  ==============================================================================  
  [..]
    This subsection provides a set of functions allowing to control dynamically
    the FMC NORSRAM interface.

@endverbatim
  * @{
  */
    
/**
  * @brief  Enables dynamically FMC_NORSRAM write operation.
  * @param  Device: Pointer to NORSRAM device instance
  * @param  Bank: NORSRAM bank number   
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NORSRAM_WriteOperation_Enable(FMC_NORSRAM_TypeDef *Device, uint32_t Bank)
{
  /* Check the parameters */
  assert_param(IS_FMC_NORSRAM_DEVICE(Device));
  assert_param(IS_FMC_NORSRAM_BANK(Bank));
  
  /* Enable write operation */
  SET_BIT(Device->BTCR[Bank], FMC_WRITE_OPERATION_ENABLE);

  return HAL_OK;  
}

/**
  * @brief  Disables dynamically FMC_NORSRAM write operation.
  * @param  Device: Pointer to NORSRAM device instance
  * @param  Bank: NORSRAM bank number   
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NORSRAM_WriteOperation_Disable(FMC_NORSRAM_TypeDef *Device, uint32_t Bank)
{ 
  /* Check the parameters */
  assert_param(IS_FMC_NORSRAM_DEVICE(Device));
  assert_param(IS_FMC_NORSRAM_BANK(Bank));
    
  /* Disable write operation */
  CLEAR_BIT(Device->BTCR[Bank], FMC_WRITE_OPERATION_ENABLE);

  return HAL_OK;  
}

/**
  * @}
  */

/**
  * @}
  */
  
/** @defgroup FMC_LL_Exported_Functions_NAND FMC Low Layer NAND Exported Functions
  * @brief    NAND Controller functions 
  *
  @verbatim 
  ==============================================================================
                    ##### How to use NAND device driver #####
  ==============================================================================
  [..]
    This driver contains a set of APIs to interface with the FMC NAND banks in order
    to run the NAND external devices.
  
    (+) FMC NAND bank reset using the function FMC_NAND_DeInit() 
    (+) FMC NAND bank control configuration using the function FMC_NAND_Init()
    (+) FMC NAND bank common space timing configuration using the function 
        FMC_NAND_CommonSpace_Timing_Init()
    (+) FMC NAND bank attribute space timing configuration using the function 
        FMC_NAND_AttributeSpace_Timing_Init()
    (+) FMC NAND bank enable/disable ECC correction feature using the functions
        FMC_NAND_ECC_Enable()/FMC_NAND_ECC_Disable()
    (+) FMC NAND bank get ECC correction code using the function FMC_NAND_GetECC()    

@endverbatim
  * @{
  */
    
/** @defgroup FMC_LL_NAND_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
  [..]  
    This section provides functions allowing to:
    (+) Initialize and configure the FMC NAND interface
    (+) De-initialize the FMC NAND interface 
    (+) Configure the FMC clock and associated GPIOs
        
@endverbatim
  * @{
  */
  
/**
  * @brief  Initializes the FMC_NAND device according to the specified
  *         control parameters in the FMC_NAND_HandleTypeDef
  * @param  Device: Pointer to NAND device instance
  * @param  Init: Pointer to NAND Initialization structure
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NAND_Init(FMC_NAND_TypeDef *Device, FMC_NAND_InitTypeDef *Init)
{
  /* Check the parameters */
  assert_param(IS_FMC_NAND_DEVICE(Device));
  assert_param(IS_FMC_NAND_BANK(Init->NandBank));
  assert_param(IS_FMC_WAIT_FEATURE(Init->Waitfeature));
  assert_param(IS_FMC_NAND_MEMORY_WIDTH(Init->MemoryDataWidth));
  assert_param(IS_FMC_ECC_STATE(Init->EccComputation));
  assert_param(IS_FMC_ECCPAGE_SIZE(Init->ECCPageSize));
  assert_param(IS_FMC_TCLR_TIME(Init->TCLRSetupTime));
  assert_param(IS_FMC_TAR_TIME(Init->TARSetupTime));   

  /* Set NAND device control parameters */
  /* NAND bank 3 registers configuration */
  MODIFY_REG(Device->PCR, PCR_CLEAR_MASK, (Init->Waitfeature                                    |\
                                          FMC_PCR_MEMORY_TYPE_NAND                              |\
                                          Init->MemoryDataWidth                                 |\
                                          Init->EccComputation                                  |\
                                          Init->ECCPageSize                                     |\
                                          ((Init->TCLRSetupTime) << POSITION_VAL(FMC_PCR_TCLR)) |\
                                          ((Init->TARSetupTime)  << POSITION_VAL(FMC_PCR_TAR))));

  return HAL_OK;

}

/**
  * @brief  Initializes the FMC_NAND Common space Timing according to the specified
  *         parameters in the FMC_NAND_PCC_TimingTypeDef
  * @param  Device: Pointer to NAND device instance
  * @param  Timing: Pointer to NAND timing structure
  * @param  Bank: NAND bank number   
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NAND_CommonSpace_Timing_Init(FMC_NAND_TypeDef *Device, FMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank)
{
  /* Check the parameters */
  assert_param(IS_FMC_NAND_DEVICE(Device));
  assert_param(IS_FMC_SETUP_TIME(Timing->SetupTime));
  assert_param(IS_FMC_WAIT_TIME(Timing->WaitSetupTime));
  assert_param(IS_FMC_HOLD_TIME(Timing->HoldSetupTime));
  assert_param(IS_FMC_HIZ_TIME(Timing->HiZSetupTime));
  assert_param(IS_FMC_NAND_BANK(Bank));
  
  /* Set FMC_NAND device timing parameters */
  /* NAND bank 3 registers configuration */
  MODIFY_REG(Device->PMEM, PMEM_CLEAR_MASK, (Timing->SetupTime                                           |\
                                            ((Timing->WaitSetupTime) << POSITION_VAL(FMC_PMEM_MEMWAIT))  |\
                                            ((Timing->HoldSetupTime) << POSITION_VAL(FMC_PMEM_MEMHOLD))  |\
                                            ((Timing->HiZSetupTime)  << POSITION_VAL(FMC_PMEM_MEMHIZ))));

  return HAL_OK;  
}

/**
  * @brief  Initializes the FMC_NAND Attribute space Timing according to the specified
  *         parameters in the FMC_NAND_PCC_TimingTypeDef
  * @param  Device: Pointer to NAND device instance
  * @param  Timing: Pointer to NAND timing structure
  * @param  Bank: NAND bank number 
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NAND_AttributeSpace_Timing_Init(FMC_NAND_TypeDef *Device, FMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank)
{
  /* Check the parameters */ 
  assert_param(IS_FMC_NAND_DEVICE(Device)); 
  assert_param(IS_FMC_SETUP_TIME(Timing->SetupTime));
  assert_param(IS_FMC_WAIT_TIME(Timing->WaitSetupTime));
  assert_param(IS_FMC_HOLD_TIME(Timing->HoldSetupTime));
  assert_param(IS_FMC_HIZ_TIME(Timing->HiZSetupTime));
  assert_param(IS_FMC_NAND_BANK(Bank));
  
  /* Set FMC_NAND device timing parameters */
  /* NAND bank 3 registers configuration */
  MODIFY_REG(Device->PATT, PATT_CLEAR_MASK, (Timing->SetupTime                                          |\
                                            ((Timing->WaitSetupTime) << POSITION_VAL(FMC_PMEM_MEMWAIT)) |\
                                            ((Timing->HoldSetupTime) << POSITION_VAL(FMC_PMEM_MEMHOLD)) |\
                                            ((Timing->HiZSetupTime)  << POSITION_VAL(FMC_PMEM_MEMHIZ))));

  return HAL_OK;
}


/**
  * @brief  DeInitialize the FMC_NAND device 
  * @param  Device: Pointer to NAND device instance
  * @param  Bank: NAND bank number
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NAND_DeInit(FMC_NAND_TypeDef *Device, uint32_t Bank)
{
  /* Check the parameters */ 
  assert_param(IS_FMC_NAND_DEVICE(Device)); 
  assert_param(IS_FMC_NAND_BANK(Bank));
      
  /* Disable the NAND Bank */
  __FMC_NAND_DISABLE(Device, Bank);
 
  /* Set the FMC_NAND_BANK registers to their reset values */
  WRITE_REG(Device->PCR, 0x00000018);
  WRITE_REG(Device->SR, 0x00000040);
  WRITE_REG(Device->PMEM, 0xFCFCFCFC);
  WRITE_REG(Device->PATT, 0xFCFCFCFC);
  
  return HAL_OK;
}

/**
  * @}
  */
  
  
/** @defgroup FMC_LL_NAND_Exported_Functions_Group2 FMC Low Layer Peripheral Control functions 
 *  @brief   management functions 
 *
@verbatim   
  ==============================================================================
                       ##### FMC_NAND Control functions #####
  ==============================================================================  
  [..]
    This subsection provides a set of functions allowing to control dynamically
    the FMC NAND interface.

@endverbatim
  * @{
  */ 

    
/**
  * @brief  Enables dynamically FMC_NAND ECC feature.
  * @param  Device: Pointer to NAND device instance
  * @param  Bank: NAND bank number
  * @retval HAL status
  */    
HAL_StatusTypeDef FMC_NAND_ECC_Enable(FMC_NAND_TypeDef *Device, uint32_t Bank)
{
  /* Check the parameters */ 
  assert_param(IS_FMC_NAND_DEVICE(Device)); 
  assert_param(IS_FMC_NAND_BANK(Bank));
    
  /* Enable ECC feature */
  SET_BIT(Device->PCR, FMC_PCR_ECCEN);
  
  return HAL_OK;  
}


/**
  * @brief  Disables dynamically FMC_NAND ECC feature.
  * @param  Device: Pointer to NAND device instance
  * @param  Bank: NAND bank number
  * @retval HAL status
  */  
HAL_StatusTypeDef FMC_NAND_ECC_Disable(FMC_NAND_TypeDef *Device, uint32_t Bank)  
{  
  /* Check the parameters */ 
  assert_param(IS_FMC_NAND_DEVICE(Device)); 
  assert_param(IS_FMC_NAND_BANK(Bank));
    
  /* Disable ECC feature */
  CLEAR_BIT(Device->PCR, FMC_PCR_ECCEN);

  return HAL_OK;  
}

/**
  * @brief  Disables dynamically FMC_NAND ECC feature.
  * @param  Device: Pointer to NAND device instance
  * @param  ECCval: Pointer to ECC value
  * @param  Bank: NAND bank number
  * @param  Timeout: Timeout wait value  
  * @retval HAL status
  */
HAL_StatusTypeDef FMC_NAND_GetECC(FMC_NAND_TypeDef *Device, uint32_t *ECCval, uint32_t Bank, uint32_t Timeout)
{
  uint32_t tickstart = 0;

  /* Check the parameters */ 
  assert_param(IS_FMC_NAND_DEVICE(Device)); 
  assert_param(IS_FMC_NAND_BANK(Bank));

  /* Get tick */ 
  tickstart = HAL_GetTick();

  /* Wait until FIFO is empty */
  while(__FMC_NAND_GET_FLAG(Device, Bank, FMC_FLAG_FEMPT) == RESET)
  {
    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
      {
        return HAL_TIMEOUT;
      }
    }  
  }
        
  /* Get the ECCR register value */
  *ECCval = (uint32_t)Device->ECCR;

  return HAL_OK;  
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

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_SRAM_MODULE_ENABLED || HAL_NOR_MODULE_ENABLED || HAL_NAND_MODULE_ENABLED */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

