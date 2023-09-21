/**
  ******************************************************************************
  * @file    ft5336.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the FT5336
  *          touch screen devices.
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


#include "ft5336.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup FT5336
  * @{
  */



/** @defgroup FT5336_Private_Types_Definitions
  * @{
  */



/** @defgroup FT5336_Private_Defines
  * @{
  */



/** @defgroup FT5336_Private_Macros
  * @{
  */



/** @defgroup FT5336_Private_Variables
  * @{
  */


TS_DrvTypeDef ft5336_ts_drv =
{
  ft5336_Init,
  ft5336_ReadID,
  ft5336_Reset,

  ft5336_TS_Start,
  ft5336_TS_DetectTouch,
  ft5336_TS_GetXY,

  ft5336_TS_EnableIT,
  ft5336_TS_ClearIT,
  ft5336_TS_ITStatus,
  ft5336_TS_DisableIT

};


static ft5336_handle_TypeDef ft5336_handle = { FT5336_I2C_NOT_INITIALIZED, 0, 0};

/**
  * @}
  */

/** @defgroup ft5336_Private_Function_Prototypes
  * @{
  */



/**
  * @brief  Return the status of I2C was initialized or not.
  * @param  None.
  * @retval : I2C initialization status.
  */
static uint8_t ft5336_Get_I2C_InitializedStatus(void);

/**
  * @brief  I2C initialize if needed.
  * @param  None.
  * @retval : None.
  */
static void ft5336_I2C_InitializeIfRequired(void);

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT5336 Device address for communication on I2C Bus.
  * @retval Status FT5336_STATUS_OK or FT5336_STATUS_NOT_OK.
  */
static uint32_t ft5336_TS_Configure(uint16_t DeviceAddr);

/** @defgroup ft5336_Private_Functions
  * @{
  */

/** @defgroup ft5336_Public_Function_Body
  * @{
  */




/**
  * @brief  Initialize the ft5336 communication bus
  *         from MCU to FT5336 : ie I2C channel initialization (if required).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @retval None
  */
void ft5336_Init(uint16_t DeviceAddr)
{
  /* Wait at least 200ms after power up before accessing registers
   * Trsi timing (Time of starting to report point after resetting) from FT5336GQQ datasheet */
  TS_IO_Delay(200);


  ft5336_I2C_InitializeIfRequired();
}

/**
  * @brief  Software Reset the ft5336.
  *         @note : Not applicable to FT5336.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @retval None
  */
void ft5336_Reset(uint16_t DeviceAddr)
{


}

/**
  * @brief  Read the ft5336 device ID, pre initialize I2C in case of need to be
  *         able to read the FT5336 device ID, and verify this is a FT5336.
  * @param  DeviceAddr: I2C FT5336 Slave address.
  * @retval The Device ID (two bytes).
  */
uint16_t ft5336_ReadID(uint16_t DeviceAddr)
{
  volatile uint8_t ucReadId = 0;
  uint8_t nbReadAttempts = 0;
  uint8_t bFoundDevice = 0; /* Device not found by default */


  ft5336_I2C_InitializeIfRequired();


  for(nbReadAttempts = 0; ((nbReadAttempts < 3) && !(bFoundDevice)); nbReadAttempts++)
  {

    ucReadId = TS_IO_Read(DeviceAddr, FT5336_CHIP_ID_REG);


    if(ucReadId == FT5336_ID_VALUE)
    {

      bFoundDevice = 1;
    }
  }


  return (ucReadId);
}

/**
  * @brief  Configures the touch Screen IC device to start detecting touches
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
  * @retval None.
  */
void ft5336_TS_Start(uint16_t DeviceAddr)
{

  FT5336_ASSERT(ft5336_TS_Configure(DeviceAddr));



  ft5336_TS_DisableIT(DeviceAddr);
}

/**
  * @brief  Return if there is touches detected or not.
  *         Try to detect new touches and forget the old ones (reset internal global
  *         variables).
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval : Number of active touches detected (can be 0, 1 or 2).
  */
uint8_t ft5336_TS_DetectTouch(uint16_t DeviceAddr)
{
  volatile uint8_t nbTouch = 0;


  nbTouch = TS_IO_Read(DeviceAddr, FT5336_TD_STAT_REG);
  nbTouch &= FT5336_TD_STAT_MASK;

  if(nbTouch > FT5336_MAX_DETECTABLE_TOUCH)
  {

    nbTouch = 0;
  }


  ft5336_handle.currActiveTouchNb = nbTouch;


  ft5336_handle.currActiveTouchIdx = 0;

  return(nbTouch);
}

/**
  * @brief  Get the touch screen X and Y positions values
  *         Manage multi touch thanks to touch Index global
  *         variable 'ft5336_handle.currActiveTouchIdx'.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
void ft5336_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
  volatile uint8_t ucReadData = 0;
  static uint16_t coord;
  uint8_t regAddressXLow = 0;
  uint8_t regAddressXHigh = 0;
  uint8_t regAddressYLow = 0;
  uint8_t regAddressYHigh = 0;

  if(ft5336_handle.currActiveTouchIdx < ft5336_handle.currActiveTouchNb)
  {
    switch(ft5336_handle.currActiveTouchIdx)
    {
    case 0 :
      regAddressXLow  = FT5336_P1_XL_REG;
      regAddressXHigh = FT5336_P1_XH_REG;
      regAddressYLow  = FT5336_P1_YL_REG;
      regAddressYHigh = FT5336_P1_YH_REG;
      break;

    case 1 :
      regAddressXLow  = FT5336_P2_XL_REG;
      regAddressXHigh = FT5336_P2_XH_REG;
      regAddressYLow  = FT5336_P2_YL_REG;
      regAddressYHigh = FT5336_P2_YH_REG;
      break;

    case 2 :
      regAddressXLow  = FT5336_P3_XL_REG;
      regAddressXHigh = FT5336_P3_XH_REG;
      regAddressYLow  = FT5336_P3_YL_REG;
      regAddressYHigh = FT5336_P3_YH_REG;
      break;

    case 3 :
      regAddressXLow  = FT5336_P4_XL_REG;
      regAddressXHigh = FT5336_P4_XH_REG;
      regAddressYLow  = FT5336_P4_YL_REG;
      regAddressYHigh = FT5336_P4_YH_REG;
      break;

    case 4 :
      regAddressXLow  = FT5336_P5_XL_REG;
      regAddressXHigh = FT5336_P5_XH_REG;
      regAddressYLow  = FT5336_P5_YL_REG;
      regAddressYHigh = FT5336_P5_YH_REG;
      break;

    case 5 :
      regAddressXLow  = FT5336_P6_XL_REG;
      regAddressXHigh = FT5336_P6_XH_REG;
      regAddressYLow  = FT5336_P6_YL_REG;
      regAddressYHigh = FT5336_P6_YH_REG;
      break;

    case 6 :
      regAddressXLow  = FT5336_P7_XL_REG;
      regAddressXHigh = FT5336_P7_XH_REG;
      regAddressYLow  = FT5336_P7_YL_REG;
      regAddressYHigh = FT5336_P7_YH_REG;
      break;

    case 7 :
      regAddressXLow  = FT5336_P8_XL_REG;
      regAddressXHigh = FT5336_P8_XH_REG;
      regAddressYLow  = FT5336_P8_YL_REG;
      regAddressYHigh = FT5336_P8_YH_REG;
      break;

    case 8 :
      regAddressXLow  = FT5336_P9_XL_REG;
      regAddressXHigh = FT5336_P9_XH_REG;
      regAddressYLow  = FT5336_P9_YL_REG;
      regAddressYHigh = FT5336_P9_YH_REG;
      break;

    case 9 :
      regAddressXLow  = FT5336_P10_XL_REG;
      regAddressXHigh = FT5336_P10_XH_REG;
      regAddressYLow  = FT5336_P10_YL_REG;
      regAddressYHigh = FT5336_P10_YH_REG;
      break;

    default :
      break;

    } /* end switch(ft5336_handle.currActiveTouchIdx) */


    ucReadData = TS_IO_Read(DeviceAddr, regAddressXLow);
    coord = (ucReadData & FT5336_TOUCH_POS_LSB_MASK) >> FT5336_TOUCH_POS_LSB_SHIFT;


    ucReadData = TS_IO_Read(DeviceAddr, regAddressXHigh);
    coord |= ((ucReadData & FT5336_TOUCH_POS_MSB_MASK) >> FT5336_TOUCH_POS_MSB_SHIFT) << 8;


    *X = coord;


    ucReadData = TS_IO_Read(DeviceAddr, regAddressYLow);
    coord = (ucReadData & FT5336_TOUCH_POS_LSB_MASK) >> FT5336_TOUCH_POS_LSB_SHIFT;


    ucReadData = TS_IO_Read(DeviceAddr, regAddressYHigh);
    coord |= ((ucReadData & FT5336_TOUCH_POS_MSB_MASK) >> FT5336_TOUCH_POS_MSB_SHIFT) << 8;


    *Y = coord;

    ft5336_handle.currActiveTouchIdx++; /* next call will work on next touch */

  } /* of if(ft5336_handle.currActiveTouchIdx < ft5336_handle.currActiveTouchNb) */
}

/**
  * @brief  Configure the FT5336 device to generate IT on given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5336).
  * @retval None
  */
void ft5336_TS_EnableIT(uint16_t DeviceAddr)
{
   uint8_t regValue = 0;
   regValue = (FT5336_G_MODE_INTERRUPT_TRIGGER & (FT5336_G_MODE_INTERRUPT_MASK >> FT5336_G_MODE_INTERRUPT_SHIFT)) << FT5336_G_MODE_INTERRUPT_SHIFT;


   TS_IO_Write(DeviceAddr, FT5336_GMODE_REG, regValue);
}

/**
  * @brief  Configure the FT5336 device to stop generating IT on the given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5336).
  * @retval None
  */
void ft5336_TS_DisableIT(uint16_t DeviceAddr)
{
  uint8_t regValue = 0;
  regValue = (FT5336_G_MODE_INTERRUPT_POLLING & (FT5336_G_MODE_INTERRUPT_MASK >> FT5336_G_MODE_INTERRUPT_SHIFT)) << FT5336_G_MODE_INTERRUPT_SHIFT;


  TS_IO_Write(DeviceAddr, FT5336_GMODE_REG, regValue);
}

/**
  * @brief  Get IT status from FT5336 interrupt status registers
  *         Should be called Following an EXTI coming to the MCU to know the detailed
  *         reason of the interrupt.
  *         @note : This feature is not applicable to FT5336.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @retval TS interrupts status : always return 0 here
  */
uint8_t ft5336_TS_ITStatus(uint16_t DeviceAddr)
{

  return 0;
}

/**
  * @brief  Clear IT status in FT5336 interrupt status clear registers
  *         Should be called Following an EXTI coming to the MCU.
  *         @note : This feature is not applicable to FT5336.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @retval None
  */
void ft5336_TS_ClearIT(uint16_t DeviceAddr)
{

}



#if (TS_MULTI_TOUCH_SUPPORTED == 1)

/**
  * @brief  Get the last touch gesture identification (zoom, move up/down...).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @param  pGestureId : Pointer to get last touch gesture Identification.
  * @retval None.
  */
void ft5336_TS_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId)
{
  volatile uint8_t ucReadData = 0;

  ucReadData = TS_IO_Read(DeviceAddr, FT5336_GEST_ID_REG);

  * pGestureId = ucReadData;
}

/**
  * @brief  Get the touch detailed informations on touch number 'touchIdx' (0..1)
  *         This touch detailed information contains :
  *         - weight that was applied to this touch
  *         - sub-area of the touch in the touch panel
  *         - event of linked to the touch (press down, lift up, ...)
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @param  touchIdx : Passed index of the touch (0..1) on which we want to get the
  *                    detailed information.
  * @param  pWeight : Pointer to to get the weight information of 'touchIdx'.
  * @param  pArea   : Pointer to to get the sub-area information of 'touchIdx'.
  * @param  pEvent  : Pointer to to get the event information of 'touchIdx'.

  * @retval None.
  */
void ft5336_TS_GetTouchInfo(uint16_t   DeviceAddr,
                            uint32_t   touchIdx,
                            uint32_t * pWeight,
                            uint32_t * pArea,
                            uint32_t * pEvent)
{
  volatile uint8_t ucReadData = 0;
  uint8_t regAddressXHigh = 0;
  uint8_t regAddressPWeight = 0;
  uint8_t regAddressPMisc = 0;

  if(touchIdx < ft5336_handle.currActiveTouchNb)
  {
    switch(touchIdx)
    {
    case 0 :
      regAddressXHigh   = FT5336_P1_XH_REG;
      regAddressPWeight = FT5336_P1_WEIGHT_REG;
      regAddressPMisc   = FT5336_P1_MISC_REG;
      break;

    case 1 :
      regAddressXHigh   = FT5336_P2_XH_REG;
      regAddressPWeight = FT5336_P2_WEIGHT_REG;
      regAddressPMisc   = FT5336_P2_MISC_REG;
      break;

    case 2 :
      regAddressXHigh   = FT5336_P3_XH_REG;
      regAddressPWeight = FT5336_P3_WEIGHT_REG;
      regAddressPMisc   = FT5336_P3_MISC_REG;
      break;

    case 3 :
      regAddressXHigh   = FT5336_P4_XH_REG;
      regAddressPWeight = FT5336_P4_WEIGHT_REG;
      regAddressPMisc   = FT5336_P4_MISC_REG;
      break;

    case 4 :
      regAddressXHigh   = FT5336_P5_XH_REG;
      regAddressPWeight = FT5336_P5_WEIGHT_REG;
      regAddressPMisc   = FT5336_P5_MISC_REG;
      break;

    case 5 :
      regAddressXHigh   = FT5336_P6_XH_REG;
      regAddressPWeight = FT5336_P6_WEIGHT_REG;
      regAddressPMisc   = FT5336_P6_MISC_REG;
      break;

    case 6 :
      regAddressXHigh   = FT5336_P7_XH_REG;
      regAddressPWeight = FT5336_P7_WEIGHT_REG;
      regAddressPMisc   = FT5336_P7_MISC_REG;
      break;

    case 7 :
      regAddressXHigh   = FT5336_P8_XH_REG;
      regAddressPWeight = FT5336_P8_WEIGHT_REG;
      regAddressPMisc   = FT5336_P8_MISC_REG;
      break;

    case 8 :
      regAddressXHigh   = FT5336_P9_XH_REG;
      regAddressPWeight = FT5336_P9_WEIGHT_REG;
      regAddressPMisc   = FT5336_P9_MISC_REG;
      break;

    case 9 :
      regAddressXHigh   = FT5336_P10_XH_REG;
      regAddressPWeight = FT5336_P10_WEIGHT_REG;
      regAddressPMisc   = FT5336_P10_MISC_REG;
      break;

    default :
      break;

    } /* end switch(touchIdx) */


    ucReadData = TS_IO_Read(DeviceAddr, regAddressXHigh);
    * pEvent = (ucReadData & FT5336_TOUCH_EVT_FLAG_MASK) >> FT5336_TOUCH_EVT_FLAG_SHIFT;


    ucReadData = TS_IO_Read(DeviceAddr, regAddressPWeight);
    * pWeight = (ucReadData & FT5336_TOUCH_WEIGHT_MASK) >> FT5336_TOUCH_WEIGHT_SHIFT;


    ucReadData = TS_IO_Read(DeviceAddr, regAddressPMisc);
    * pArea = (ucReadData & FT5336_TOUCH_AREA_MASK) >> FT5336_TOUCH_AREA_SHIFT;

  } /* of if(touchIdx < ft5336_handle.currActiveTouchNb) */
}

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

/** @defgroup ft5336_Static_Function_Body
  * @{
  */




/**
  * @brief  Return the status of I2C was initialized or not.
  * @param  None.
  * @retval : I2C initialization status.
  */
static uint8_t ft5336_Get_I2C_InitializedStatus(void)
{
  return(ft5336_handle.i2cInitialized);
}

/**
  * @brief  I2C initialize if needed.
  * @param  None.
  * @retval : None.
  */
static void ft5336_I2C_InitializeIfRequired(void)
{
  if(ft5336_Get_I2C_InitializedStatus() == FT5336_I2C_NOT_INITIALIZED)
  {

    TS_IO_Init();


    ft5336_handle.i2cInitialized = FT5336_I2C_INITIALIZED;
  }
}

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT5336 Device address for communication on I2C Bus.
  * @retval Status FT5336_STATUS_OK or FT5336_STATUS_NOT_OK.
  */
static uint32_t ft5336_TS_Configure(uint16_t DeviceAddr)
{
  uint32_t status = FT5336_STATUS_OK;



  return(status);
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

