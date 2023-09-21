/**
  ******************************************************************************
  * @file    mfxstm32l152.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the MFXSTM32L152
  *          IO Expander devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


#include "mfxstm32l152.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup MFXSTM32L152
  * @{
  */



/** @defgroup MFXSTM32L152_Private_Types_Definitions
  * @{
  */



/** @defgroup MFXSTM32L152_Private_Defines
  * @{
  */
#define MFXSTM32L152_MAX_INSTANCE         3



/** @defgroup MFXSTM32L152_Private_Macros
  * @{
  */



/** @defgroup MFXSTM32L152_Private_Variables
  * @{
  */


TS_DrvTypeDef mfxstm32l152_ts_drv =
{
  mfxstm32l152_Init,
  mfxstm32l152_ReadID,
  mfxstm32l152_Reset,

  mfxstm32l152_TS_Start,
  mfxstm32l152_TS_DetectTouch,
  mfxstm32l152_TS_GetXY,

  mfxstm32l152_TS_EnableIT,
  mfxstm32l152_TS_ClearIT,
  mfxstm32l152_TS_ITStatus,
  mfxstm32l152_TS_DisableIT,
};


IO_DrvTypeDef mfxstm32l152_io_drv =
{
  mfxstm32l152_Init,
  mfxstm32l152_ReadID,
  mfxstm32l152_Reset,

  mfxstm32l152_IO_Start,
  mfxstm32l152_IO_Config,
  mfxstm32l152_IO_WritePin,
  mfxstm32l152_IO_ReadPin,

  mfxstm32l152_IO_EnableIT,
  mfxstm32l152_IO_DisableIT,
  mfxstm32l152_IO_ITStatus,
  mfxstm32l152_IO_ClearIT,
};


IDD_DrvTypeDef mfxstm32l152_idd_drv =
{
  mfxstm32l152_Init,
  mfxstm32l152_DeInit,
  mfxstm32l152_ReadID,
  mfxstm32l152_Reset,
  mfxstm32l152_LowPower,
  mfxstm32l152_WakeUp,

  mfxstm32l152_IDD_Start,
  mfxstm32l152_IDD_Config,
  mfxstm32l152_IDD_GetValue,

  mfxstm32l152_IDD_EnableIT,
  mfxstm32l152_IDD_ClearIT,
  mfxstm32l152_IDD_GetITStatus,
  mfxstm32l152_IDD_DisableIT,

  mfxstm32l152_Error_EnableIT,
  mfxstm32l152_Error_ClearIT,
  mfxstm32l152_Error_GetITStatus,
  mfxstm32l152_Error_DisableIT,
  mfxstm32l152_Error_ReadSrc,
  mfxstm32l152_Error_ReadMsg
};



uint8_t mfxstm32l152[MFXSTM32L152_MAX_INSTANCE] = {0};
/**
  * @}
  */



/** @defgroup MFXSTM32L152_Private_Function_Prototypes
  * @{
  */
static uint8_t mfxstm32l152_GetInstance(uint16_t DeviceAddr);
static uint8_t  mfxstm32l152_ReleaseInstance(uint16_t DeviceAddr);
static void mfxstm32l152_reg24_setPinValue(uint16_t DeviceAddr, uint8_t RegisterAddr, uint32_t PinPosition, uint8_t PinValue );



/** @defgroup MFXSTM32L152_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the mfxstm32l152 and configure the needed hardware resources
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_Init(uint16_t DeviceAddr)
{
  uint8_t instance;
  uint8_t empty;


  instance = mfxstm32l152_GetInstance(DeviceAddr);


  if(instance == 0xFF)
  {

    empty = mfxstm32l152_GetInstance(0);

    if(empty < MFXSTM32L152_MAX_INSTANCE)
    {

      mfxstm32l152[empty] = DeviceAddr;


      MFX_IO_Init();
    }
  }

  mfxstm32l152_SetIrqOutPinPolarity(DeviceAddr, MFXSTM32L152_OUT_PIN_POLARITY_HIGH);
  mfxstm32l152_SetIrqOutPinType(DeviceAddr, MFXSTM32L152_OUT_PIN_TYPE_PUSHPULL);
}

/**
  * @brief  DeInitialize the mfxstm32l152 and unconfigure the needed hardware resources
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_DeInit(uint16_t DeviceAddr)
{
  uint8_t instance;


  instance = mfxstm32l152_ReleaseInstance(DeviceAddr);


  if(instance != 0xFF)
  {

    MFX_IO_DeInit();
  }
}

/**
  * @brief  Reset the mfxstm32l152 by Software.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_Reset(uint16_t DeviceAddr)
{

  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL, MFXSTM32L152_SWRST);


  MFX_IO_Delay(10);
}

/**
  * @brief  Put mfxstm32l152 Device in Low Power standby mode
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void  mfxstm32l152_LowPower(uint16_t DeviceAddr)
{

  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL, MFXSTM32L152_STANDBY);


  MFX_IO_EnableWakeupPin();
}

/**
  * @brief  WakeUp mfxstm32l152 from standby mode
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void  mfxstm32l152_WakeUp(uint16_t DeviceAddr)
{
  uint8_t instance;


  instance = mfxstm32l152_GetInstance(DeviceAddr);


  if(instance == 0xFF)
  {

    MFX_IO_EnableWakeupPin();
  }


  MFX_IO_Wakeup();
}

/**
  * @brief  Read the MFXSTM32L152 IO Expander device ID.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval The Device ID (two bytes).
  */
uint16_t mfxstm32l152_ReadID(uint16_t DeviceAddr)
{
  uint8_t id;


  MFX_IO_Delay(1);


  MFX_IO_Init();

  id = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_ID);


  return (id);
}

/**
  * @brief  Read the MFXSTM32L152 device firmware version.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval The Device FW version (two bytes).
  */
uint16_t mfxstm32l152_ReadFwVersion(uint16_t DeviceAddr)
{
  uint8_t  data[2];

  MFX_IO_ReadMultiple((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_FW_VERSION_MSB, data, sizeof(data)) ;


  return ((data[0] << 8) | data[1]);
}

/**
  * @brief  Enable the interrupt mode for the selected IT source
  * @param  DeviceAddr: Device address on communication Bus.
  * @param Source: The interrupt source to be configured, could be:
  *   @arg  MFXSTM32L152_IRQ_GPIO: IO interrupt
  *   @arg  MFXSTM32L152_IRQ_IDD : IDD interrupt
  *   @arg  MFXSTM32L152_IRQ_ERROR : Error interrupt
  *   @arg  MFXSTM32L152_IRQ_TS_DET : Touch Screen Controller Touch Detected interrupt
  *   @arg  MFXSTM32L152_IRQ_TS_NE : Touch Screen FIFO Not Empty
  *   @arg  MFXSTM32L152_IRQ_TS_TH : Touch Screen FIFO threshold triggered
  *   @arg  MFXSTM32L152_IRQ_TS_FULL : Touch Screen FIFO Full
  *   @arg  MFXSTM32L152_IRQ_TS_OVF : Touch Screen FIFO Overflow
  * @retval None
  */
void mfxstm32l152_EnableITSource(uint16_t DeviceAddr, uint8_t Source)
{
  uint8_t tmp = 0;


  tmp = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_SRC_EN);


  tmp |= Source;


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_SRC_EN, tmp);
}

/**
  * @brief  Disable the interrupt mode for the selected IT source
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  Source: The interrupt source to be configured, could be:
  *   @arg  MFXSTM32L152_IRQ_GPIO: IO interrupt
  *   @arg  MFXSTM32L152_IRQ_IDD : IDD interrupt
  *   @arg  MFXSTM32L152_IRQ_ERROR : Error interrupt
  *   @arg  MFXSTM32L152_IRQ_TS_DET : Touch Screen Controller Touch Detected interrupt
  *   @arg  MFXSTM32L152_IRQ_TS_NE : Touch Screen FIFO Not Empty
  *   @arg  MFXSTM32L152_IRQ_TS_TH : Touch Screen FIFO threshold triggered
  *   @arg  MFXSTM32L152_IRQ_TS_FULL : Touch Screen FIFO Full
  *   @arg  MFXSTM32L152_IRQ_TS_OVF : Touch Screen FIFO Overflow
  * @retval None
  */
void mfxstm32l152_DisableITSource(uint16_t DeviceAddr, uint8_t Source)
{
  uint8_t tmp = 0;


  tmp = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_SRC_EN);


  tmp &= ~Source;


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_SRC_EN, tmp);
}


/**
  * @brief  Returns the selected Global interrupt source pending bit value
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  Source: the Global interrupt source to be checked, could be:
  *   @arg  MFXSTM32L152_IRQ_GPIO: IO interrupt
  *   @arg  MFXSTM32L152_IRQ_IDD : IDD interrupt
  *   @arg  MFXSTM32L152_IRQ_ERROR : Error interrupt
  *   @arg  MFXSTM32L152_IRQ_TS_DET : Touch Screen Controller Touch Detected interrupt
  *   @arg  MFXSTM32L152_IRQ_TS_NE : Touch Screen FIFO Not Empty
  *   @arg  MFXSTM32L152_IRQ_TS_TH : Touch Screen FIFO threshold triggered
  *   @arg  MFXSTM32L152_IRQ_TS_FULL : Touch Screen FIFO Full
  *   @arg  MFXSTM32L152_IRQ_TS_OVF : Touch Screen FIFO Overflow
  * @retval The value of the checked Global interrupt source status.
  */
uint8_t mfxstm32l152_GlobalITStatus(uint16_t DeviceAddr, uint8_t Source)
{

  return((MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_PENDING) & Source));
}

/**
  * @brief  Clear the selected Global interrupt pending bit(s)
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  Source: the Global interrupt source to be cleared, could be any combination
  *         of the below values. The acknowledge signal for MFXSTM32L152_GPIOs configured in input
  *         with interrupt is not on this register but in IRQ_GPI_ACK1, IRQ_GPI_ACK2 registers.
  *   @arg  MFXSTM32L152_IRQ_IDD : IDD interrupt
  *   @arg  MFXSTM32L152_IRQ_ERROR : Error interrupt
  *   @arg  MFXSTM32L152_IRQ_TS_DET : Touch Screen Controller Touch Detected interrupt
  *   @arg  MFXSTM32L152_IRQ_TS_NE : Touch Screen FIFO Not Empty
  *   @arg  MFXSTM32L152_IRQ_TS_TH : Touch Screen FIFO threshold triggered
  *   @arg  MFXSTM32L152_IRQ_TS_FULL : Touch Screen FIFO Full
  *   @arg  MFXSTM32L152_IRQ_TS_OVF : Touch Screen FIFO Overflow
  *  /\/\ IMPORTANT NOTE /\/\ must not use MFXSTM32L152_IRQ_GPIO as argument, see IRQ_GPI_ACK1 and IRQ_GPI_ACK2 registers
  * @retval None
  */
void mfxstm32l152_ClearGlobalIT(uint16_t DeviceAddr, uint8_t Source)
{

  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_ACK, Source);
}

/**
  * @brief  Set the global interrupt Polarity of IRQ_OUT_PIN.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  Polarity: the IT mode polarity, could be one of the following values:
  *   @arg  MFXSTM32L152_OUT_PIN_POLARITY_LOW: Interrupt output line is active Low edge
  *   @arg  MFXSTM32L152_OUT_PIN_POLARITY_HIGH: Interrupt line output is active High edge
  * @retval None
  */
void mfxstm32l152_SetIrqOutPinPolarity(uint16_t DeviceAddr, uint8_t Polarity)
{
  uint8_t tmp = 0;


  tmp = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_MFX_IRQ_OUT);


  tmp &= ~(uint8_t)0x02;


  tmp |= Polarity;


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_MFX_IRQ_OUT, tmp);


  MFX_IO_Delay(1);

}

/**
  * @brief  Set the global interrupt Type of IRQ_OUT_PIN.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  Type: Interrupt line activity type, could be one of the following values:
  *   @arg  MFXSTM32L152_OUT_PIN_TYPE_OPENDRAIN: Open Drain output Interrupt line
  *   @arg  MFXSTM32L152_OUT_PIN_TYPE_PUSHPULL: Push Pull output Interrupt line
  * @retval None
  */
void mfxstm32l152_SetIrqOutPinType(uint16_t DeviceAddr, uint8_t Type)
{
  uint8_t tmp = 0;


  tmp = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_MFX_IRQ_OUT);


  tmp &= ~(uint8_t)0x01;


  tmp |= Type;


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_MFX_IRQ_OUT, tmp);


  MFX_IO_Delay(1);

}







/**
  * @brief  Start the IO functionality used and enable the AF for selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  AF_en: 0 to disable, else enabled.
  * @retval None
  */
void mfxstm32l152_IO_Start(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint8_t mode;


  mode = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL);


  mode |= MFXSTM32L152_GPIO_EN;









  if (IO_Pin > 0xFFFF)
  {
    mode |= MFXSTM32L152_ALTERNATE_GPIO_EN;
  }
  else
  {
    mode &= ~MFXSTM32L152_ALTERNATE_GPIO_EN;
  }


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL, mode);


  MFX_IO_Delay(1);
}

/**
  * @brief  Configures the IO pin(s) according to IO mode structure value.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The output pin to be set or reset. This parameter can be one
  *         of the following values:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: where x can be from 0 to 23.
  * @param  IO_Mode: The IO pin mode to configure, could be one of the following values:
  *   @arg  IO_MODE_INPUT
  *   @arg  IO_MODE_OUTPUT
  *   @arg  IO_MODE_IT_RISING_EDGE
  *   @arg  IO_MODE_IT_FALLING_EDGE
  *   @arg  IO_MODE_IT_LOW_LEVEL
  *   @arg  IO_MODE_IT_HIGH_LEVEL
  *   @arg  IO_MODE_INPUT_PU,
  *   @arg  IO_MODE_INPUT_PD,
  *   @arg  IO_MODE_OUTPUT_OD_PU,
  *   @arg  IO_MODE_OUTPUT_OD_PD,
  *   @arg  IO_MODE_OUTPUT_PP_PU,
  *   @arg  IO_MODE_OUTPUT_PP_PD,
  *   @arg  IO_MODE_IT_RISING_EDGE_PU
  *   @arg  IO_MODE_IT_FALLING_EDGE_PU
  *   @arg  IO_MODE_IT_LOW_LEVEL_PU
  *   @arg  IO_MODE_IT_HIGH_LEVEL_PU
  *   @arg  IO_MODE_IT_RISING_EDGE_PD
  *   @arg  IO_MODE_IT_FALLING_EDGE_PD
  *   @arg  IO_MODE_IT_LOW_LEVEL_PD
  *   @arg  IO_MODE_IT_HIGH_LEVEL_PD
  * @retval None
  */
uint8_t mfxstm32l152_IO_Config(uint16_t DeviceAddr, uint32_t IO_Pin, IO_ModeTypedef IO_Mode)
{
  uint8_t error_code = 0;


  switch(IO_Mode)
  {
  case IO_MODE_OFF: /* Off or analog mode */
  case IO_MODE_ANALOG: /* Off or analog mode */
    mfxstm32l152_IO_DisablePinIT(DeviceAddr, IO_Pin); /* first disable IT */
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITHOUT_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_DOWN);
    break;

  case IO_MODE_INPUT: /* Input mode */
    mfxstm32l152_IO_DisablePinIT(DeviceAddr, IO_Pin); /* first disable IT */
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITHOUT_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    break;

  case IO_MODE_INPUT_PU: /* Input mode */
    mfxstm32l152_IO_DisablePinIT(DeviceAddr, IO_Pin); /* first disable IT */
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    break;

  case IO_MODE_INPUT_PD: /* Input mode */
    mfxstm32l152_IO_DisablePinIT(DeviceAddr, IO_Pin); /* first disable IT */
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_DOWN);
    break;

  case IO_MODE_OUTPUT: /* Output mode */
  case IO_MODE_OUTPUT_PP_PD: /* Output mode */
    mfxstm32l152_IO_DisablePinIT(DeviceAddr, IO_Pin); /* first disable IT */
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_OUT);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPO_PUSH_PULL);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_DOWN);
    break;

  case IO_MODE_OUTPUT_PP_PU: /* Output mode */
    mfxstm32l152_IO_DisablePinIT(DeviceAddr, IO_Pin); /* first disable IT */
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_OUT);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPO_PUSH_PULL);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    break;

  case IO_MODE_OUTPUT_OD_PD: /* Output mode */
    mfxstm32l152_IO_DisablePinIT(DeviceAddr, IO_Pin); /* first disable IT */
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_OUT);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPO_OPEN_DRAIN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_DOWN);
    break;

  case IO_MODE_OUTPUT_OD_PU: /* Output mode */
    mfxstm32l152_IO_DisablePinIT(DeviceAddr, IO_Pin); /* first disable IT */
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_OUT);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPO_OPEN_DRAIN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    break;

  case IO_MODE_IT_RISING_EDGE: /* Interrupt rising edge mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITHOUT_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_EDGE);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_HLRE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin); /* last to do: enable IT */
    break;

  case IO_MODE_IT_RISING_EDGE_PU: /* Interrupt rising edge mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_EDGE);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_HLRE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_RISING_EDGE_PD: /* Interrupt rising edge mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_DOWN);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_EDGE);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_HLRE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_FALLING_EDGE: /* Interrupt falling edge mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITHOUT_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_EDGE);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_LLFE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_FALLING_EDGE_PU: /* Interrupt falling edge mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_EDGE);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_LLFE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_FALLING_EDGE_PD: /* Interrupt falling edge mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_DOWN);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_EDGE);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_LLFE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_LOW_LEVEL: /* Low level interrupt mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITHOUT_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_LEVEL);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_LLFE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_LOW_LEVEL_PU: /* Low level interrupt mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_LEVEL);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_LLFE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_LOW_LEVEL_PD: /* Low level interrupt mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_DOWN);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_LEVEL);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_LLFE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_HIGH_LEVEL: /* High level interrupt mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITHOUT_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_LEVEL);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_HLRE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_HIGH_LEVEL_PU: /* High level interrupt mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_UP);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_LEVEL);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_HLRE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  case IO_MODE_IT_HIGH_LEVEL_PD: /* High level interrupt mode */
    mfxstm32l152_IO_EnableIT(DeviceAddr);
    mfxstm32l152_IO_InitPin(DeviceAddr, IO_Pin, MFXSTM32L152_GPIO_DIR_IN);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_TYPE1, IO_Pin, MFXSTM32L152_GPI_WITH_PULL_RESISTOR);
    mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_PUPD1, IO_Pin, MFXSTM32L152_GPIO_PULL_DOWN);
    mfxstm32l152_IO_SetIrqEvtMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_EVT_LEVEL);
    mfxstm32l152_IO_SetIrqTypeMode(DeviceAddr, IO_Pin, MFXSTM32L152_IRQ_GPI_TYPE_HLRE);
    mfxstm32l152_IO_EnablePinIT(DeviceAddr, IO_Pin);  /* last to do: enable IT */
    break;

  default:
    error_code = (uint8_t) IO_Mode;
    break;
  }

  return error_code;
}

/**
  * @brief  Initialize the selected IO pin direction.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any
  *         combination of the following values:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: Where x can be from 0 to 23.
  * @param  Direction: could be MFXSTM32L152_GPIO_DIR_IN or MFXSTM32L152_GPIO_DIR_OUT.
  * @retval None
  */
void mfxstm32l152_IO_InitPin(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t Direction)
{
  mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_DIR1, IO_Pin, Direction);
}

/**
  * @brief  Set the global interrupt Type.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any
  *         combination of the following values:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: Where x can be from 0 to 23.
  * @param  Evt: Interrupt line activity type, could be one of the following values:
  *   @arg  MFXSTM32L152_IRQ_GPI_EVT_LEVEL: Interrupt line is active in level model
  *   @arg  MFXSTM32L152_IRQ_GPI_EVT_EDGE: Interrupt line is active in edge model
  * @retval None
  */
void mfxstm32l152_IO_SetIrqEvtMode(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t Evt)
{
  mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_EVT1, IO_Pin, Evt);
  MFX_IO_Delay(1);
}

/**
  * @brief  Configure the Edge for which a transition is detectable for the
  *         selected pin.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any
  *         combination of the following values:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: Where x can be from 0 to 23.
  * @param  Evt: Interrupt line activity type, could be one of the following values:
  *   @arg  MFXSTM32L152_IRQ_GPI_TYPE_LLFE: Interrupt line is active in Low Level or Falling Edge
  *   @arg  MFXSTM32L152_IRQ_GPI_TYPE_HLRE: Interrupt line is active in High Level or Rising Edge
  * @retval None
  */
void mfxstm32l152_IO_SetIrqTypeMode(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t Type)
{
  mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_TYPE1, IO_Pin, Type);
  MFX_IO_Delay(1);
}

/**
  * @brief  When GPIO is in output mode, puts the corresponding GPO in High (1) or Low (0) level.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The output pin to be set or reset. This parameter can be one
  *         of the following values:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: where x can be from 0 to 23.
  * @param PinState: The new IO pin state.
  * @retval None
  */
void mfxstm32l152_IO_WritePin(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t PinState)
{

  if (PinState != 0)
  {

	mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPO_SET1, IO_Pin, 1);
  }
  else
  {

	mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_GPO_CLR1, IO_Pin, 1);
  }
}

/**
  * @brief  Return the state of the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The output pin to be set or reset. This parameter can be one
  *         of the following values:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: where x can be from 0 to 23.
  * @retval IO pin(s) state.
  */
uint32_t mfxstm32l152_IO_ReadPin(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint32_t  tmp1 = 0;
  uint32_t  tmp2 = 0;
  uint32_t  tmp3 = 0;

  if(IO_Pin & 0x000000FF)
  {
    tmp1 = (uint32_t) MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_STATE1);
  }
  if(IO_Pin & 0x0000FF00)
  {
    tmp2 = (uint32_t) MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_STATE2);
  }
  if(IO_Pin & 0x00FF0000)
  {
    tmp3 = (uint32_t) MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_GPIO_STATE3);
  }

  tmp3 = tmp1 + (tmp2 << 8) + (tmp3 << 16);

  return(tmp3 & IO_Pin);
}

/**
  * @brief  Enable the global IO interrupt source.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_IO_EnableIT(uint16_t DeviceAddr)
{
  MFX_IO_ITConfig();


  mfxstm32l152_EnableITSource(DeviceAddr, MFXSTM32L152_IRQ_GPIO);
}

/**
  * @brief  Disable the global IO interrupt source.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_IO_DisableIT(uint16_t DeviceAddr)
{

  mfxstm32l152_DisableITSource(DeviceAddr, MFXSTM32L152_IRQ_GPIO);
}

/**
  * @brief  Enable interrupt mode for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be enabled. This parameter could be any
  *         combination of the following values:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: where x can be from 0 to 23.
  * @retval None
  */
void mfxstm32l152_IO_EnablePinIT(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_SRC1, IO_Pin, 1);
}

/**
  * @brief  Disable interrupt mode for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be disabled. This parameter could be any
  *         combination of the following values:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: where x can be from 0 to 23.
  * @retval None
  */
void mfxstm32l152_IO_DisablePinIT(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  mfxstm32l152_reg24_setPinValue(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_SRC1, IO_Pin, 0);
}


/**
  * @brief  Check the status of the selected IO interrupt pending bit
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be checked could be:
  *   @arg  MFXSTM32L152_GPIO_PIN_x Where x can be from 0 to 23.
  * @retval Status of the checked IO pin(s).
  */
uint32_t mfxstm32l152_IO_ITStatus(uint16_t DeviceAddr, uint32_t IO_Pin)
{

  uint8_t   tmp1 = 0;
  uint16_t  tmp2 = 0;
  uint32_t  tmp3 = 0;

  if(IO_Pin & 0xFF)
  {
    tmp1 = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_PENDING1);
  }
  if(IO_Pin & 0xFFFF00)
  {
    tmp2 = (uint16_t) MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_PENDING2);
  }
  if(IO_Pin & 0xFFFF0000)
  {
    tmp3 = (uint32_t) MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_PENDING3);
  }

  tmp3 = tmp1 + (tmp2 << 8) + (tmp3 << 16);

  return(tmp3 & IO_Pin);
}

/**
  * @brief  Clear the selected IO interrupt pending bit(s). It clear automatically also the general MFXSTM32L152_REG_ADR_IRQ_PENDING
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: the IO interrupt to be cleared, could be:
  *   @arg  MFXSTM32L152_GPIO_PIN_x: Where x can be from 0 to 23.
  * @retval None
  */
void mfxstm32l152_IO_ClearIT(uint16_t DeviceAddr, uint32_t IO_Pin)
{



  uint8_t pin_0_7, pin_8_15, pin_16_23;

  pin_0_7   = IO_Pin & 0x0000ff;
  pin_8_15  = IO_Pin >> 8;
  pin_8_15   = pin_8_15 & 0x00ff;
  pin_16_23 = IO_Pin >> 16;

  if (pin_0_7)
  {
    MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_ACK1, pin_0_7);
  }
  if (pin_8_15)
  {
    MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_ACK2, pin_8_15);
  }
  if (pin_16_23)
  {
    MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_IRQ_GPI_ACK3, pin_16_23);
  }
}


/**
  * @brief  Enable the AF for aGPIO.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_IO_EnableAF(uint16_t DeviceAddr)
{
  uint8_t mode;


  mode = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL);









  mode |= MFXSTM32L152_ALTERNATE_GPIO_EN;


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL, mode);
}

/**
  * @brief  Disable the AF for aGPIO.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
 void mfxstm32l152_IO_DisableAF(uint16_t DeviceAddr)
{
  uint8_t mode;


  mode = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL);









  mode &= ~MFXSTM32L152_ALTERNATE_GPIO_EN;


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL, mode);

}






/**
  * @brief  Configures the touch Screen Controller (Single point detection)
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None.
  */
void mfxstm32l152_TS_Start(uint16_t DeviceAddr)
{
  uint8_t mode;


  mode = MFX_IO_Read(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL);


  mode |= MFXSTM32L152_TS_EN;


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL, mode);


  MFX_IO_Delay(2);


  /* Configuration:
     - Touch average control    : 4 samples
     - Touch delay time         : 500 uS
     - Panel driver setting time: 500 uS
  */
  MFX_IO_Write(DeviceAddr, MFXSTM32L152_TS_SETTLING, 0x32);
  MFX_IO_Write(DeviceAddr, MFXSTM32L152_TS_TOUCH_DET_DELAY, 0x5);
  MFX_IO_Write(DeviceAddr, MFXSTM32L152_TS_AVE, 0x04);


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_TS_FIFO_TH, 0x01);


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_TS_FIFO_TH, MFXSTM32L152_TS_CLEAR_FIFO);

  /* Touch screen control configuration :
     - No window tracking index
   */
  MFX_IO_Write(DeviceAddr, MFXSTM32L152_TS_TRACK, 0x00);



  mfxstm32l152_IO_ClearIT(DeviceAddr, 0xFFFFFF);


  MFX_IO_Delay(1);
}

/**
  * @brief  Return if there is touch detected or not.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Touch detected state.
  */
uint8_t mfxstm32l152_TS_DetectTouch(uint16_t DeviceAddr)
{
  uint8_t state;
  uint8_t ret = 0;

  state = MFX_IO_Read(DeviceAddr, MFXSTM32L152_TS_FIFO_STA);
  state = ((state & (uint8_t)MFXSTM32L152_TS_CTRL_STATUS) == (uint8_t)MFXSTM32L152_TS_CTRL_STATUS);

  if(state > 0)
  {
    if(MFX_IO_Read(DeviceAddr, MFXSTM32L152_TS_FIFO_LEVEL) > 0)
    {
      ret = 1;
    }
  }

  return ret;
}

/**
  * @brief  Get the touch screen X and Y positions values
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
void mfxstm32l152_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
  uint8_t  data_xy[3];

  MFX_IO_ReadMultiple(DeviceAddr, MFXSTM32L152_TS_XY_DATA, data_xy, sizeof(data_xy)) ;


  *X = (data_xy[1]<<4) + (data_xy[0]>>4);
  *Y = (data_xy[2]<<4) + (data_xy[0]&4);


  MFX_IO_Write(DeviceAddr, MFXSTM32L152_TS_FIFO_TH, MFXSTM32L152_TS_CLEAR_FIFO);
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_TS_EnableIT(uint16_t DeviceAddr)
{
  MFX_IO_ITConfig();


  mfxstm32l152_EnableITSource(DeviceAddr, MFXSTM32L152_IRQ_TS_DET);
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_TS_DisableIT(uint16_t DeviceAddr)
{

  mfxstm32l152_DisableITSource(DeviceAddr, MFXSTM32L152_IRQ_TS_DET);
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval TS interrupts status
  */
uint8_t mfxstm32l152_TS_ITStatus(uint16_t DeviceAddr)
{

  return(mfxstm32l152_GlobalITStatus(DeviceAddr, MFXSTM32L152_IRQ_TS));
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_TS_ClearIT(uint16_t DeviceAddr)
{

  mfxstm32l152_ClearGlobalIT(DeviceAddr, MFXSTM32L152_IRQ_TS);
}





/**
  * @brief  Launch IDD current measurement
  * @param  DeviceAddr: Device address on communication Bus
  * @retval None.
  */
void mfxstm32l152_IDD_Start(uint16_t DeviceAddr)
{
  uint8_t mode = 0;


  mode = MFX_IO_Read((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_CTRL);


  mode |= MFXSTM32L152_IDD_CTRL_REQ;


  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_CTRL, mode);
}

/**
  * @brief  Configures the IDD current measurement
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  MfxIddConfig: Parameters depending on hardware config.
  * @retval None
  */
void mfxstm32l152_IDD_Config(uint16_t DeviceAddr, IDD_ConfigTypeDef MfxIddConfig)
{
  uint8_t value = 0;
  uint8_t mode = 0;


  mode = MFX_IO_Read((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL);

  if((mode & MFXSTM32L152_IDD_EN) != MFXSTM32L152_IDD_EN)
  {

    mode |= MFXSTM32L152_IDD_EN;


    MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_SYS_CTRL, mode);
  }


  value =  ((MfxIddConfig.ShuntNbUsed << 1) & MFXSTM32L152_IDD_CTRL_SHUNT_NB);
  value |= (MfxIddConfig.VrefMeasurement & MFXSTM32L152_IDD_CTRL_VREF_DIS);
  value |= (MfxIddConfig.Calibration & MFXSTM32L152_IDD_CTRL_CAL_DIS);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_CTRL, value);


  value = (MfxIddConfig.PreDelayUnit & MFXSTM32L152_IDD_PREDELAY_UNIT) |
          (MfxIddConfig.PreDelayValue & MFXSTM32L152_IDD_PREDELAY_VALUE);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_PRE_DELAY, value);


  value = (uint8_t) (MfxIddConfig.Shunt0Value >> 8);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT0_MSB, value);
  value = (uint8_t) (MfxIddConfig.Shunt0Value);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT0_LSB, value);


  value = (uint8_t) (MfxIddConfig.Shunt1Value >> 8);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT1_MSB, value);
  value = (uint8_t) (MfxIddConfig.Shunt1Value);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT1_LSB, value);


  value = (uint8_t) (MfxIddConfig.Shunt2Value >> 8);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT2_MSB, value);
  value = (uint8_t) (MfxIddConfig.Shunt2Value);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT2_LSB, value);


  value = (uint8_t) (MfxIddConfig.Shunt3Value >> 8);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT3_MSB, value);
  value = (uint8_t) (MfxIddConfig.Shunt3Value);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT3_LSB, value);


  value = (uint8_t) (MfxIddConfig.Shunt4Value >> 8);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT4_MSB, value);
  value = (uint8_t) (MfxIddConfig.Shunt4Value);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT4_LSB, value);


  value = MfxIddConfig.Shunt0StabDelay;
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SH0_STABILIZATION, value);


  value = MfxIddConfig.Shunt1StabDelay;
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SH1_STABILIZATION, value);


  value = MfxIddConfig.Shunt2StabDelay;
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SH2_STABILIZATION, value);


  value = MfxIddConfig.Shunt3StabDelay;
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SH3_STABILIZATION, value);


  value = MfxIddConfig.Shunt4StabDelay;
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SH4_STABILIZATION, value);


  value = (uint8_t) (MfxIddConfig.AmpliGain >> 8);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_GAIN_MSB, value);
  value = (uint8_t) (MfxIddConfig.AmpliGain);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_GAIN_LSB, value);


  value = (uint8_t) (MfxIddConfig.VddMin >> 8);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_VDD_MIN_MSB, value);
  value = (uint8_t) (MfxIddConfig.VddMin);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_VDD_MIN_LSB, value);


  value = MfxIddConfig.MeasureNb;
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_NBR_OF_MEAS, value);


  value = (MfxIddConfig.DeltaDelayUnit & MFXSTM32L152_IDD_DELTADELAY_UNIT) |
          (MfxIddConfig.DeltaDelayValue & MFXSTM32L152_IDD_DELTADELAY_VALUE);
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_MEAS_DELTA_DELAY, value);


  value = MfxIddConfig.ShuntNbOnBoard;
  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNTS_ON_BOARD, value);
}

/**
  * @brief  This function allows to modify number of shunt used for a measurement
  * @param  DeviceAddr: Device address on communication Bus
  * @retval None.
  */
void mfxstm32l152_IDD_ConfigShuntNbLimit(uint16_t DeviceAddr, uint8_t ShuntNbLimit)
{
  uint8_t mode = 0;


  mode = MFX_IO_Read((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_CTRL);


  mode &= ~(MFXSTM32L152_IDD_CTRL_SHUNT_NB);


  mode |= ((ShuntNbLimit << 1) & MFXSTM32L152_IDD_CTRL_SHUNT_NB);


  MFX_IO_Write((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_CTRL, mode);
}

/**
  * @brief  Get Idd current value
  * @param  DeviceAddr: Device address on communication Bus
  * @param  ReadValue: Pointer on value to be read
  * @retval Idd value in 10 nA.
  */
void mfxstm32l152_IDD_GetValue(uint16_t DeviceAddr, uint32_t *ReadValue)
{
  uint8_t  data[3];

  MFX_IO_ReadMultiple((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_VALUE_MSB, data, sizeof(data)) ;


  *ReadValue = (data[0] << 16) | (data[1] << 8) | data[2];

}

/**
  * @brief  Get Last shunt used for measurement
  * @param  DeviceAddr: Device address on communication Bus
  * @retval Last shunt used
  */
uint8_t  mfxstm32l152_IDD_GetShuntUsed(uint16_t DeviceAddr)
{
  return(MFX_IO_Read((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_IDD_SHUNT_USED));
}

/**
  * @brief  Configure mfx to enable Idd interrupt
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_IDD_EnableIT(uint16_t DeviceAddr)
{
  MFX_IO_ITConfig();


  mfxstm32l152_EnableITSource(DeviceAddr, MFXSTM32L152_IRQ_IDD);
}

/**
  * @brief  Clear Idd global interrupt
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_IDD_ClearIT(uint16_t DeviceAddr)
{

  mfxstm32l152_ClearGlobalIT(DeviceAddr, MFXSTM32L152_IRQ_IDD);
}

/**
  * @brief  get Idd interrupt status
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval IDD interrupts status
  */
uint8_t mfxstm32l152_IDD_GetITStatus(uint16_t DeviceAddr)
{

  return(mfxstm32l152_GlobalITStatus(DeviceAddr, MFXSTM32L152_IRQ_IDD));
}

/**
  * @brief  disable Idd interrupt
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None.
  */
void mfxstm32l152_IDD_DisableIT(uint16_t DeviceAddr)
{

  mfxstm32l152_DisableITSource(DeviceAddr, MFXSTM32L152_IRQ_IDD);
}






/**
  * @brief  Read Error Source.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Error message code with error source
  */
uint8_t mfxstm32l152_Error_ReadSrc(uint16_t DeviceAddr)
{

  return(MFX_IO_Read((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_ERROR_SRC));
}

/**
  * @brief  Read Error Message
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Error message code with error source
  */
uint8_t mfxstm32l152_Error_ReadMsg(uint16_t DeviceAddr)
{

  return(MFX_IO_Read((uint8_t) DeviceAddr, MFXSTM32L152_REG_ADR_ERROR_MSG));
}

/**
  * @brief  Enable Error global interrupt
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */

void mfxstm32l152_Error_EnableIT(uint16_t DeviceAddr)
{
  MFX_IO_ITConfig();


  mfxstm32l152_EnableITSource(DeviceAddr, MFXSTM32L152_IRQ_ERROR);
}

/**
  * @brief  Clear Error global interrupt
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void mfxstm32l152_Error_ClearIT(uint16_t DeviceAddr)
{

  mfxstm32l152_ClearGlobalIT(DeviceAddr, MFXSTM32L152_IRQ_ERROR);
}

/**
  * @brief  get Error interrupt status
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Error interrupts status
  */
uint8_t mfxstm32l152_Error_GetITStatus(uint16_t DeviceAddr)
{

  return(mfxstm32l152_GlobalITStatus(DeviceAddr, MFXSTM32L152_IRQ_ERROR));
}

/**
  * @brief  disable Error interrupt
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None.
  */
void mfxstm32l152_Error_DisableIT(uint16_t DeviceAddr)
{

  mfxstm32l152_DisableITSource(DeviceAddr, MFXSTM32L152_IRQ_ERROR);
}

/**
  * @brief  FOR DEBUG ONLY
  */
uint8_t mfxstm32l152_ReadReg(uint16_t DeviceAddr, uint8_t RegAddr)
{

  return(MFX_IO_Read((uint8_t) DeviceAddr, RegAddr));
}

void mfxstm32l152_WriteReg(uint16_t DeviceAddr, uint8_t RegAddr, uint8_t Value)
{

  MFX_IO_Write((uint8_t) DeviceAddr, RegAddr, Value);
}




/**
  * @brief  Check if the device instance of the selected address is already registered
  *         and return its index
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Index of the device instance if registered, 0xFF if not.
  */
static uint8_t mfxstm32l152_GetInstance(uint16_t DeviceAddr)
{
  uint8_t idx;


  for(idx = 0; idx < MFXSTM32L152_MAX_INSTANCE ; idx ++)
  {
    if(mfxstm32l152[idx] == DeviceAddr)
    {
      return idx;
    }
  }

  return 0xFF;
}

/**
  * @brief  Release registered device instance
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Index of released device instance, 0xFF if not.
  */
static uint8_t mfxstm32l152_ReleaseInstance(uint16_t DeviceAddr)
{
  uint8_t idx;


  for(idx = 0; idx < MFXSTM32L152_MAX_INSTANCE ; idx ++)
  {
    if(mfxstm32l152[idx] == DeviceAddr)
    {
      mfxstm32l152[idx] = 0;
      return idx;
    }
  }
  return 0xFF;
}

/**
  * @brief  Internal routine
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  RegisterAddr: Register Address
  * @param  PinPosition: Pin [0:23]
  * @param  PinValue: 0/1
  * @retval None
  */
void mfxstm32l152_reg24_setPinValue(uint16_t DeviceAddr, uint8_t RegisterAddr, uint32_t PinPosition, uint8_t PinValue )
{
  uint8_t tmp = 0;
  uint8_t pin_0_7, pin_8_15, pin_16_23;

  pin_0_7   = PinPosition & 0x0000ff;
  pin_8_15  = PinPosition >> 8;
  pin_8_15   = pin_8_15 & 0x00ff;
  pin_16_23 = PinPosition >> 16;

  if (pin_0_7)
  {

    tmp = MFX_IO_Read(DeviceAddr, RegisterAddr);


    if (PinValue != 0)
    {
      tmp |= (uint8_t)pin_0_7;
    }
    else
    {
      tmp &= ~(uint8_t)pin_0_7;
    }


    MFX_IO_Write(DeviceAddr, RegisterAddr, tmp);
  }

  if (pin_8_15)
  {

    tmp = MFX_IO_Read(DeviceAddr, RegisterAddr+1);


    if (PinValue != 0)
    {
      tmp |= (uint8_t)pin_8_15;
    }
    else
    {
      tmp &= ~(uint8_t)pin_8_15;
    }


    MFX_IO_Write(DeviceAddr, RegisterAddr+1, tmp);
  }

  if (pin_16_23)
  {

    tmp = MFX_IO_Read(DeviceAddr, RegisterAddr+2);


    if (PinValue != 0)
    {
      tmp |= (uint8_t)pin_16_23;
    }
    else
    {
      tmp &= ~(uint8_t)pin_16_23;
    }


    MFX_IO_Write(DeviceAddr, RegisterAddr+2, tmp);
  }
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

