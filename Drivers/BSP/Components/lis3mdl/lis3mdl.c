/**
 ******************************************************************************
 * @file    lis3mdl.c
 * @author  MCD Application Team
 * @brief   This file provides a set of functions needed to manage the LIS3MDL
 *          magnetometer devices
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */


#include "lis3mdl.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup LIS3MDL LIS3MDL
  * @{
  */

/** @defgroup LIS3MDL_Mag_Private_Variables LIS3MDL Mag Private Variables
  * @{
  */ 
MAGNETO_DrvTypeDef Lis3mdlMagDrv =
{
  LIS3MDL_MagInit,
  LIS3MDL_MagDeInit,
  LIS3MDL_MagReadID,
  0,
  LIS3MDL_MagLowPower,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  LIS3MDL_MagReadXYZ
};
/**
  * @}
  */ 


/** @defgroup LIS3MDL_Mag_Private_Functions LIS3MDL Mag Private Functions
  * @{
  */
/**
  * @brief  Set LIS3MDL Magnetometer Initialization.
  * @param  LIS3MDL_InitStruct: pointer to a LIS3MDL_MagInitTypeDef structure 
  *         that contains the configuration setting for the LIS3MDL.
  */
void LIS3MDL_MagInit(MAGNETO_InitTypeDef LIS3MDL_InitStruct)
{  
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG1, LIS3MDL_InitStruct.Register1);
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG2, LIS3MDL_InitStruct.Register2);
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, LIS3MDL_InitStruct.Register3);
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG4, LIS3MDL_InitStruct.Register4);
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG5, LIS3MDL_InitStruct.Register5);
}

/**
  * @brief  LIS3MDL Magnetometer De-initialization.
  */
void LIS3MDL_MagDeInit(void)
{
  uint8_t ctrl = 0x00;
  

  ctrl = SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3);


  ctrl &= ~(LIS3MDL_MAG_SELECTION_MODE);


  ctrl |= LIS3MDL_MAG_POWERDOWN2_MODE;
  

  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, ctrl);  
}

/**
  * @brief  Read LIS3MDL ID.
  * @retval ID 
  */
uint8_t LIS3MDL_MagReadID(void)
{

  SENSOR_IO_Init();  

  return (SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_WHO_AM_I_REG));
}

/**
  * @brief  Set/Unset Magnetometer in low power mode.
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled
  */
void LIS3MDL_MagLowPower(uint16_t status)
{  
  uint8_t ctrl = 0;
  

  ctrl = SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3);


  ctrl &= ~(0x20);


  if(status)
  {
    ctrl |= LIS3MDL_MAG_CONFIG_LOWPOWER_MODE;
  }else
  {
    ctrl |= LIS3MDL_MAG_CONFIG_NORMAL_MODE;
  }
  

  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, ctrl);  
}

/**
  * @brief  Read X, Y & Z Magnetometer values 
  * @param  pData: Data out pointer
  */
void LIS3MDL_MagReadXYZ(int16_t* pData)
{
  int16_t pnRawData[3];
  uint8_t ctrlm= 0;
  uint8_t buffer[6];
  uint8_t i = 0;
  float sensitivity = 0;
  

  ctrlm = SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG2);
  

  SENSOR_IO_ReadMultiple(LIS3MDL_MAG_I2C_ADDRESS_HIGH, (LIS3MDL_MAG_OUTX_L | 0x80), buffer, 6);
  
  for(i=0; i<3; i++)
  {
    pnRawData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
  }
  


  switch(ctrlm & 0x60)
  {
  case LIS3MDL_MAG_FS_4_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA;
    break;
  case LIS3MDL_MAG_FS_8_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA;
    break;
  case LIS3MDL_MAG_FS_12_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA;
    break;
  case LIS3MDL_MAG_FS_16_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA;
    break;    
  }
  

  for(i=0; i<3; i++)
  {
    pData[i]=( int16_t )(pnRawData[i] * sensitivity);
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
  

  
