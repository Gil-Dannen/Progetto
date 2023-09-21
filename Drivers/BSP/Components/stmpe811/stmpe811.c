/**
  ******************************************************************************
  * @file    stmpe811.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the STMPE811
  *          IO Expander devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */  


#include "stmpe811.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */ 
  
/** @defgroup STMPE811
  * @{
  */   

/** @defgroup STMPE811_Private_Types_Definitions
  * @{
  */ 

/** @defgroup STMPE811_Private_Defines
  * @{
  */ 
#define STMPE811_MAX_INSTANCE         2 
/**
  * @}
  */

/** @defgroup STMPE811_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STMPE811_Private_Variables
  * @{
  */ 


TS_DrvTypeDef stmpe811_ts_drv = 
{
  stmpe811_Init,
  stmpe811_ReadID,
  stmpe811_Reset,
  stmpe811_TS_Start,
  stmpe811_TS_DetectTouch,
  stmpe811_TS_GetXY,
  stmpe811_TS_EnableIT,
  stmpe811_TS_ClearIT,
  stmpe811_TS_ITStatus,
  stmpe811_TS_DisableIT,
};


IO_DrvTypeDef stmpe811_io_drv = 
{
  stmpe811_Init,
  stmpe811_ReadID,
  stmpe811_Reset,
  stmpe811_IO_Start,
  stmpe811_IO_Config,
  stmpe811_IO_WritePin,
  stmpe811_IO_ReadPin,
  stmpe811_IO_EnableIT,
  stmpe811_IO_DisableIT,
  stmpe811_IO_ITStatus,
  stmpe811_IO_ClearIT,
};


uint8_t stmpe811[STMPE811_MAX_INSTANCE] = {0};
/**
  * @}
  */ 

/** @defgroup STMPE811_Private_Function_Prototypes
  * @{
  */
static uint8_t stmpe811_GetInstance(uint16_t DeviceAddr); 
/**
  * @}
  */ 

/** @defgroup STMPE811_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the stmpe811 and configure the needed hardware resources
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void stmpe811_Init(uint16_t DeviceAddr)
{
  uint8_t instance;
  uint8_t empty;
  

  instance = stmpe811_GetInstance(DeviceAddr);
  

  if(instance == 0xFF)
  {

    empty = stmpe811_GetInstance(0);
    
    if(empty < STMPE811_MAX_INSTANCE)
    {

      stmpe811[empty] = DeviceAddr;
      

      IOE_Init(); 
      

      stmpe811_Reset(DeviceAddr);
    }
  }
}
 
/**
  * @brief  Reset the stmpe811 by Software.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void stmpe811_Reset(uint16_t DeviceAddr)
{

  IOE_Write(DeviceAddr, STMPE811_REG_SYS_CTRL1, 2);


  IOE_Delay(10); 
  

  IOE_Write(DeviceAddr, STMPE811_REG_SYS_CTRL1, 0);
  

  IOE_Delay(2); 
}

/**
  * @brief  Read the stmpe811 IO Expander device ID.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval The Device ID (two bytes).
  */
uint16_t stmpe811_ReadID(uint16_t DeviceAddr)
{

  IOE_Init(); 
  

  return ((IOE_Read(DeviceAddr, STMPE811_REG_CHP_ID_LSB) << 8) |\
          (IOE_Read(DeviceAddr, STMPE811_REG_CHP_ID_MSB)));
}

/**
  * @brief  Enable the Global interrupt.
  * @param  DeviceAddr: Device address on communication Bus.       
  * @retval None
  */
void stmpe811_EnableGlobalIT(uint16_t DeviceAddr)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_INT_CTRL);
  

  tmp |= (uint8_t)STMPE811_GIT_EN;
  

  IOE_Write(DeviceAddr, STMPE811_REG_INT_CTRL, tmp); 
}

/**
  * @brief  Disable the Global interrupt.
  * @param  DeviceAddr: Device address on communication Bus.      
  * @retval None
  */
void stmpe811_DisableGlobalIT(uint16_t DeviceAddr)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_INT_CTRL);


  tmp &= ~(uint8_t)STMPE811_GIT_EN;
 

  IOE_Write(DeviceAddr, STMPE811_REG_INT_CTRL, tmp);
    
}

/**
  * @brief  Enable the interrupt mode for the selected IT source
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param Source: The interrupt source to be configured, could be:
  *   @arg  STMPE811_GIT_IO: IO interrupt 
  *   @arg  STMPE811_GIT_ADC : ADC interrupt    
  *   @arg  STMPE811_GIT_FE : Touch Screen Controller FIFO Error interrupt
  *   @arg  STMPE811_GIT_FF : Touch Screen Controller FIFO Full interrupt      
  *   @arg  STMPE811_GIT_FOV : Touch Screen Controller FIFO Overrun interrupt     
  *   @arg  STMPE811_GIT_FTH : Touch Screen Controller FIFO Threshold interrupt   
  *   @arg  STMPE811_GIT_TOUCH : Touch Screen Controller Touch Detected interrupt  
  * @retval None
  */
void stmpe811_EnableITSource(uint16_t DeviceAddr, uint8_t Source)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_INT_EN);


  tmp |= Source; 
  

  IOE_Write(DeviceAddr, STMPE811_REG_INT_EN, tmp);   
}

/**
  * @brief  Disable the interrupt mode for the selected IT source
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  Source: The interrupt source to be configured, could be:
  *   @arg  STMPE811_GIT_IO: IO interrupt 
  *   @arg  STMPE811_GIT_ADC : ADC interrupt    
  *   @arg  STMPE811_GIT_FE : Touch Screen Controller FIFO Error interrupt
  *   @arg  STMPE811_GIT_FF : Touch Screen Controller FIFO Full interrupt      
  *   @arg  STMPE811_GIT_FOV : Touch Screen Controller FIFO Overrun interrupt     
  *   @arg  STMPE811_GIT_FTH : Touch Screen Controller FIFO Threshold interrupt   
  *   @arg  STMPE811_GIT_TOUCH : Touch Screen Controller Touch Detected interrupt  
  * @retval None
  */
void stmpe811_DisableITSource(uint16_t DeviceAddr, uint8_t Source)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_INT_EN);


  tmp &= ~Source; 
  

  IOE_Write(DeviceAddr, STMPE811_REG_INT_EN, tmp);   
}

/**
  * @brief  Set the global interrupt Polarity.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  Polarity: the IT mode polarity, could be one of the following values:
  *   @arg  STMPE811_POLARITY_LOW: Interrupt line is active Low/Falling edge      
  *   @arg  STMPE811_POLARITY_HIGH: Interrupt line is active High/Rising edge              
  * @retval None
  */
void stmpe811_SetITPolarity(uint16_t DeviceAddr, uint8_t Polarity)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_INT_CTRL);
  

  tmp &= ~(uint8_t)0x04;
    

  tmp |= Polarity;
  

  IOE_Write(DeviceAddr, STMPE811_REG_INT_CTRL, tmp);
 
}

/**
  * @brief  Set the global interrupt Type. 
  * @param  DeviceAddr: Device address on communication Bus.      
  * @param  Type: Interrupt line activity type, could be one of the following values:
  *   @arg  STMPE811_TYPE_LEVEL: Interrupt line is active in level model         
  *   @arg  STMPE811_TYPE_EDGE: Interrupt line is active in edge model           
  * @retval None
  */
void stmpe811_SetITType(uint16_t DeviceAddr, uint8_t Type)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_INT_CTRL);
  

  tmp &= ~(uint8_t)0x02;
    

  tmp |= Type;
  

  IOE_Write(DeviceAddr, STMPE811_REG_INT_CTRL, tmp);
 
}

/**
  * @brief  Check the selected Global interrupt source pending bit
  * @param  DeviceAddr: Device address on communication Bus. 
  * @param  Source: the Global interrupt source to be checked, could be:
  *   @arg  STMPE811_GIT_IO: IO interrupt 
  *   @arg  STMPE811_GIT_ADC : ADC interrupt    
  *   @arg  STMPE811_GIT_FE : Touch Screen Controller FIFO Error interrupt
  *   @arg  STMPE811_GIT_FF : Touch Screen Controller FIFO Full interrupt      
  *   @arg  STMPE811_GIT_FOV : Touch Screen Controller FIFO Overrun interrupt     
  *   @arg  STMPE811_GIT_FTH : Touch Screen Controller FIFO Threshold interrupt   
  *   @arg  STMPE811_GIT_TOUCH : Touch Screen Controller Touch Detected interrupt      
  * @retval The checked Global interrupt source status.
  */
uint8_t stmpe811_GlobalITStatus(uint16_t DeviceAddr, uint8_t Source)
{

  return((IOE_Read(DeviceAddr, STMPE811_REG_INT_STA) & Source) == Source);
}

/**
  * @brief  Return the Global interrupts status
  * @param  DeviceAddr: Device address on communication Bus. 
  * @param  Source: the Global interrupt source to be checked, could be:
  *   @arg  STMPE811_GIT_IO: IO interrupt 
  *   @arg  STMPE811_GIT_ADC : ADC interrupt    
  *   @arg  STMPE811_GIT_FE : Touch Screen Controller FIFO Error interrupt
  *   @arg  STMPE811_GIT_FF : Touch Screen Controller FIFO Full interrupt      
  *   @arg  STMPE811_GIT_FOV : Touch Screen Controller FIFO Overrun interrupt     
  *   @arg  STMPE811_GIT_FTH : Touch Screen Controller FIFO Threshold interrupt   
  *   @arg  STMPE811_GIT_TOUCH : Touch Screen Controller Touch Detected interrupt      
  * @retval The checked Global interrupt source status.
  */
uint8_t stmpe811_ReadGITStatus(uint16_t DeviceAddr, uint8_t Source)
{

  return((IOE_Read(DeviceAddr, STMPE811_REG_INT_STA) & Source));
}

/**
  * @brief  Clear the selected Global interrupt pending bit(s)
  * @param  DeviceAddr: Device address on communication Bus. 
  * @param  Source: the Global interrupt source to be cleared, could be any combination
  *         of the following values:        
  *   @arg  STMPE811_GIT_IO: IO interrupt 
  *   @arg  STMPE811_GIT_ADC : ADC interrupt    
  *   @arg  STMPE811_GIT_FE : Touch Screen Controller FIFO Error interrupt
  *   @arg  STMPE811_GIT_FF : Touch Screen Controller FIFO Full interrupt      
  *   @arg  STMPE811_GIT_FOV : Touch Screen Controller FIFO Overrun interrupt     
  *   @arg  STMPE811_GIT_FTH : Touch Screen Controller FIFO Threshold interrupt   
  *   @arg  STMPE811_GIT_TOUCH : Touch Screen Controller Touch Detected interrupt 
  * @retval None
  */
void stmpe811_ClearGlobalIT(uint16_t DeviceAddr, uint8_t Source)
{

  IOE_Write(DeviceAddr, STMPE811_REG_INT_STA, Source);
}

/**
  * @brief  Start the IO functionality use and disable the AF for selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  IO_Pin: The IO pin(s) to put in AF. This parameter can be one 
  *         of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7.
  * @retval None
  */
void stmpe811_IO_Start(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint8_t mode;
  

  mode = IOE_Read(DeviceAddr, STMPE811_REG_SYS_CTRL2);
  

  mode &= ~(STMPE811_IO_FCT | STMPE811_ADC_FCT);  
  

  IOE_Write(DeviceAddr, STMPE811_REG_SYS_CTRL2, mode); 


  stmpe811_IO_DisableAF(DeviceAddr, (uint8_t)IO_Pin);
}

/**
  * @brief  Configures the IO pin(s) according to IO mode structure value.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  IO_Pin: The output pin to be set or reset. This parameter can be one 
  *         of the following values:   
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7.
  * @param  IO_Mode: The IO pin mode to configure, could be one of the following values:
  *   @arg  IO_MODE_INPUT
  *   @arg  IO_MODE_OUTPUT
  *   @arg  IO_MODE_IT_RISING_EDGE
  *   @arg  IO_MODE_IT_FALLING_EDGE
  *   @arg  IO_MODE_IT_LOW_LEVEL
  *   @arg  IO_MODE_IT_HIGH_LEVEL            
  * @retval 0 if no error, IO_Mode if error
  */
uint8_t stmpe811_IO_Config(uint16_t DeviceAddr, uint32_t IO_Pin, IO_ModeTypedef IO_Mode)
{
  uint8_t error_code = 0;


  switch(IO_Mode)
  {
  case IO_MODE_INPUT: /* Input mode */
    stmpe811_IO_InitPin(DeviceAddr, IO_Pin, STMPE811_DIRECTION_IN);
    break;
    
  case IO_MODE_OUTPUT: /* Output mode */
    stmpe811_IO_InitPin(DeviceAddr, IO_Pin, STMPE811_DIRECTION_OUT);
    break;
  
  case IO_MODE_IT_RISING_EDGE: /* Interrupt rising edge mode */
    stmpe811_IO_EnableIT(DeviceAddr);
    stmpe811_IO_EnablePinIT(DeviceAddr, IO_Pin);
    stmpe811_IO_InitPin(DeviceAddr, IO_Pin, STMPE811_DIRECTION_IN); 
    stmpe811_SetITType(DeviceAddr, STMPE811_TYPE_EDGE);      
    stmpe811_IO_SetEdgeMode(DeviceAddr, IO_Pin, STMPE811_EDGE_RISING); 
    break;
  
  case IO_MODE_IT_FALLING_EDGE: /* Interrupt falling edge mode */
    stmpe811_IO_EnableIT(DeviceAddr);
    stmpe811_IO_EnablePinIT(DeviceAddr, IO_Pin);
    stmpe811_IO_InitPin(DeviceAddr, IO_Pin, STMPE811_DIRECTION_IN); 
    stmpe811_SetITType(DeviceAddr, STMPE811_TYPE_EDGE);    
    stmpe811_IO_SetEdgeMode(DeviceAddr, IO_Pin, STMPE811_EDGE_FALLING); 
    break;
  
  case IO_MODE_IT_LOW_LEVEL: /* Low level interrupt mode */
    stmpe811_IO_EnableIT(DeviceAddr);
    stmpe811_IO_EnablePinIT(DeviceAddr, IO_Pin);
    stmpe811_IO_InitPin(DeviceAddr, IO_Pin, STMPE811_DIRECTION_IN); 
    stmpe811_SetITType(DeviceAddr, STMPE811_TYPE_LEVEL);
    stmpe811_SetITPolarity(DeviceAddr, STMPE811_POLARITY_LOW);      
    break;
    
  case IO_MODE_IT_HIGH_LEVEL: /* High level interrupt mode */
    stmpe811_IO_EnableIT(DeviceAddr);
    stmpe811_IO_EnablePinIT(DeviceAddr, IO_Pin);
    stmpe811_IO_InitPin(DeviceAddr, IO_Pin, STMPE811_DIRECTION_IN); 
    stmpe811_SetITType(DeviceAddr, STMPE811_TYPE_LEVEL);
    stmpe811_SetITPolarity(DeviceAddr, STMPE811_POLARITY_HIGH);  
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
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.   
  * @param  Direction: could be STMPE811_DIRECTION_IN or STMPE811_DIRECTION_OUT.      
  * @retval None
  */
void stmpe811_IO_InitPin(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t Direction)
{
  uint8_t tmp = 0;   
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_IO_DIR);
  

  if (Direction != STMPE811_DIRECTION_IN)
  {
    tmp |= (uint8_t)IO_Pin;
  }  
  else 
  {
    tmp &= ~(uint8_t)IO_Pin;
  }
  

  IOE_Write(DeviceAddr, STMPE811_REG_IO_DIR, tmp);   
}

/**
  * @brief  Disable the AF for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.        
  * @retval None
  */
void stmpe811_IO_DisableAF(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_IO_AF);


  tmp |= (uint8_t)IO_Pin;


  IOE_Write(DeviceAddr, STMPE811_REG_IO_AF, tmp);
  
}

/**
  * @brief  Enable the AF for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.       
  * @retval None
  */
void stmpe811_IO_EnableAF(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_IO_AF);


  tmp &= ~(uint8_t)IO_Pin;   
  

  IOE_Write(DeviceAddr, STMPE811_REG_IO_AF, tmp); 
}

/**
  * @brief  Configure the Edge for which a transition is detectable for the
  *         selected pin.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.  
  * @param  Edge: The edge which will be detected. This parameter can be one or
  *         a combination of following values: STMPE811_EDGE_FALLING and STMPE811_EDGE_RISING .
  * @retval None
  */
void stmpe811_IO_SetEdgeMode(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t Edge)
{
  uint8_t tmp1 = 0, tmp2 = 0;   
  

  tmp1 = IOE_Read(DeviceAddr, STMPE811_REG_IO_FE);
  tmp2 = IOE_Read(DeviceAddr, STMPE811_REG_IO_RE);


  tmp1 &= ~(uint8_t)IO_Pin;
  

  tmp2 &= ~(uint8_t)IO_Pin;


  if (Edge & STMPE811_EDGE_FALLING)
  {
    tmp1 |= (uint8_t)IO_Pin;
  }


  if (Edge & STMPE811_EDGE_RISING)
  {
    tmp2 |= (uint8_t)IO_Pin;
  }


  IOE_Write(DeviceAddr, STMPE811_REG_IO_FE, tmp1);
  IOE_Write(DeviceAddr, STMPE811_REG_IO_RE, tmp2);
}

/**
  * @brief  Write a new IO pin state.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param IO_Pin: The output pin to be set or reset. This parameter can be one 
  *        of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7. 
  * @param PinState: The new IO pin state.
  * @retval None
  */
void stmpe811_IO_WritePin(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t PinState)
{

  if (PinState != 0)
  {

    IOE_Write(DeviceAddr, STMPE811_REG_IO_SET_PIN, (uint8_t)IO_Pin);
  }
  else
  {

    IOE_Write(DeviceAddr, STMPE811_REG_IO_CLR_PIN, (uint8_t)IO_Pin);
  } 
}

/**
  * @brief  Return the state of the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param IO_Pin: The output pin to be set or reset. This parameter can be one 
  *        of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7. 
  * @retval IO pin(s) state.
  */
uint32_t stmpe811_IO_ReadPin(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  return((uint32_t)(IOE_Read(DeviceAddr, STMPE811_REG_IO_MP_STA) & (uint8_t)IO_Pin));
}

/**
  * @brief  Enable the global IO interrupt source.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void stmpe811_IO_EnableIT(uint16_t DeviceAddr)
{ 
  IOE_ITConfig();
  

  stmpe811_EnableITSource(DeviceAddr, STMPE811_GIT_IO);
  

  stmpe811_EnableGlobalIT(DeviceAddr); 
}

/**
  * @brief  Disable the global IO interrupt source.
  * @param  DeviceAddr: Device address on communication Bus.   
  * @retval None
  */
void stmpe811_IO_DisableIT(uint16_t DeviceAddr)
{

  stmpe811_DisableGlobalIT(DeviceAddr);
  

  stmpe811_DisableITSource(DeviceAddr, STMPE811_GIT_IO);    
}

/**
  * @brief  Enable interrupt mode for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be enabled. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7.
  * @retval None
  */
void stmpe811_IO_EnablePinIT(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_IO_INT_EN);
  

  tmp |= (uint8_t)IO_Pin;
  

  IOE_Write(DeviceAddr, STMPE811_REG_IO_INT_EN, tmp);  
}

/**
  * @brief  Disable interrupt mode for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be disabled. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7.
  * @retval None
  */
void stmpe811_IO_DisablePinIT(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint8_t tmp = 0;
  

  tmp = IOE_Read(DeviceAddr, STMPE811_REG_IO_INT_EN);
  

  tmp &= ~(uint8_t)IO_Pin;
  

  IOE_Write(DeviceAddr, STMPE811_REG_IO_INT_EN, tmp);   
}

/**
  * @brief  Check the status of the selected IO interrupt pending bit
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be checked could be:
  *   @arg  STMPE811_PIN_x Where x can be from 0 to 7.             
  * @retval Status of the checked IO pin(s).
  */
uint32_t stmpe811_IO_ITStatus(uint16_t DeviceAddr, uint32_t IO_Pin)
{

  return(IOE_Read(DeviceAddr, STMPE811_REG_IO_INT_STA) & (uint8_t)IO_Pin); 
}

/**
  * @brief  Clear the selected IO interrupt pending bit(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: the IO interrupt to be cleared, could be:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.            
  * @retval None
  */
void stmpe811_IO_ClearIT(uint16_t DeviceAddr, uint32_t IO_Pin)
{

  stmpe811_ClearGlobalIT(DeviceAddr, STMPE811_GIT_IO);
  

  IOE_Write(DeviceAddr, STMPE811_REG_IO_INT_STA, (uint8_t)IO_Pin);  
  

  IOE_Write(DeviceAddr, STMPE811_REG_IO_ED, (uint8_t)IO_Pin);
  

  IOE_Write(DeviceAddr, STMPE811_REG_IO_RE, (uint8_t)IO_Pin);
  

  IOE_Write(DeviceAddr, STMPE811_REG_IO_FE, (uint8_t)IO_Pin); 
}

/**
  * @brief  Configures the touch Screen Controller (Single point detection)
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None.
  */
void stmpe811_TS_Start(uint16_t DeviceAddr)
{
  uint8_t mode;
  

  mode = IOE_Read(DeviceAddr, STMPE811_REG_SYS_CTRL2);
  

  mode &= ~(STMPE811_IO_FCT);  
  

  IOE_Write(DeviceAddr, STMPE811_REG_SYS_CTRL2, mode); 


  stmpe811_IO_EnableAF(DeviceAddr, STMPE811_TOUCH_IO_ALL);
  

  mode &= ~(STMPE811_TS_FCT | STMPE811_ADC_FCT);  
  

  IOE_Write(DeviceAddr, STMPE811_REG_SYS_CTRL2, mode); 
  

  IOE_Write(DeviceAddr, STMPE811_REG_ADC_CTRL1, 0x49);
  

  IOE_Delay(2); 
  

  IOE_Write(DeviceAddr, STMPE811_REG_ADC_CTRL2, 0x01);
  

  /* Configuration: 
     - Touch average control    : 4 samples
     - Touch delay time         : 500 uS
     - Panel driver setting time: 500 uS 
  */
  IOE_Write(DeviceAddr, STMPE811_REG_TSC_CFG, 0x9A); 
  

  IOE_Write(DeviceAddr, STMPE811_REG_FIFO_TH, 0x01);
  

  IOE_Write(DeviceAddr, STMPE811_REG_FIFO_STA, 0x01);
  

  IOE_Write(DeviceAddr, STMPE811_REG_FIFO_STA, 0x00);
  
  /* Set the range and accuracy pf the pressure measurement (Z) : 
     - Fractional part :7 
     - Whole part      :1 
  */
  IOE_Write(DeviceAddr, STMPE811_REG_TSC_FRACT_XYZ, 0x01);
  

  IOE_Write(DeviceAddr, STMPE811_REG_TSC_I_DRIVE, 0x01);
  
  /* Touch screen control configuration (enable TSC):
     - No window tracking index
     - XYZ acquisition mode
   */
  IOE_Write(DeviceAddr, STMPE811_REG_TSC_CTRL, 0x01);
  

  IOE_Write(DeviceAddr, STMPE811_REG_INT_STA, 0xFF);


  IOE_Delay(2); 
}

/**
  * @brief  Return if there is touch detected or not.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Touch detected state.
  */
uint8_t stmpe811_TS_DetectTouch(uint16_t DeviceAddr)
{
  uint8_t state;
  uint8_t ret = 0;
  
  state = ((IOE_Read(DeviceAddr, STMPE811_REG_TSC_CTRL) & (uint8_t)STMPE811_TS_CTRL_STATUS) == (uint8_t)0x80);
  
  if(state > 0)
  {
    if(IOE_Read(DeviceAddr, STMPE811_REG_FIFO_SIZE) > 0)
    {
      ret = 1;
    }
  }
  else
  {

    IOE_Write(DeviceAddr, STMPE811_REG_FIFO_STA, 0x01);

    IOE_Write(DeviceAddr, STMPE811_REG_FIFO_STA, 0x00);
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
void stmpe811_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
  uint8_t  dataXYZ[4];
  uint32_t uldataXYZ;
  
  IOE_ReadMultiple(DeviceAddr, STMPE811_REG_TSC_DATA_NON_INC, dataXYZ, sizeof(dataXYZ)) ;
  

  uldataXYZ = (dataXYZ[0] << 24)|(dataXYZ[1] << 16)|(dataXYZ[2] << 8)|(dataXYZ[3] << 0);     
  *X = (uldataXYZ >> 20) & 0x00000FFF;     
  *Y = (uldataXYZ >>  8) & 0x00000FFF;     
  

  IOE_Write(DeviceAddr, STMPE811_REG_FIFO_STA, 0x01);

  IOE_Write(DeviceAddr, STMPE811_REG_FIFO_STA, 0x00);
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void stmpe811_TS_EnableIT(uint16_t DeviceAddr)
{
  IOE_ITConfig();
  

  stmpe811_EnableITSource(DeviceAddr, STMPE811_TS_IT); 
  

  stmpe811_EnableGlobalIT(DeviceAddr);
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.    
  * @retval None
  */
void stmpe811_TS_DisableIT(uint16_t DeviceAddr)
{

  stmpe811_DisableGlobalIT(DeviceAddr);
  

  stmpe811_DisableITSource(DeviceAddr, STMPE811_TS_IT); 
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.    
  * @retval TS interrupts status
  */
uint8_t stmpe811_TS_ITStatus(uint16_t DeviceAddr)
{

  return(stmpe811_ReadGITStatus(DeviceAddr, STMPE811_TS_IT));
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void stmpe811_TS_ClearIT(uint16_t DeviceAddr)
{

  stmpe811_ClearGlobalIT(DeviceAddr, STMPE811_TS_IT);
}

/**
  * @brief  Check if the device instance of the selected address is already registered
  *         and return its index  
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Index of the device instance if registered, 0xFF if not.
  */
static uint8_t stmpe811_GetInstance(uint16_t DeviceAddr)
{
  uint8_t idx = 0;
  

  for(idx = 0; idx < STMPE811_MAX_INSTANCE ; idx ++)
  {
    if(stmpe811[idx] == DeviceAddr)
    {
      return idx; 
    }
  }
  
  return 0xFF;
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


