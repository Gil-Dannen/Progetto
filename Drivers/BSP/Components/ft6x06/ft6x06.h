/**
  ******************************************************************************
  * @file    ft6x06.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the
  *          ft6x06.c IO expander driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


#ifndef __FT6X06_H
#define __FT6X06_H

#ifdef __cplusplus
extern "C" {
#endif


#ifndef TS_MULTI_TOUCH_SUPPORTED
  #define TS_MULTI_TOUCH_SUPPORTED 0
#endif


#ifndef TS_AUTO_CALIBRATION_SUPPORTED
  #define TS_AUTO_CALIBRATION_SUPPORTED 0
#endif
  

#include "../Common/ts.h"



/** @typedef ft6x06_handle_TypeDef
 *  ft6x06 Handle definition.
 */
typedef struct
{
  uint8_t i2cInitialized;


  uint8_t currActiveTouchNb;


  uint8_t currActiveTouchIdx;

} ft6x06_handle_TypeDef;

  /** @addtogroup BSP
   * @{
   */

  /** @addtogroup Component
   * @{
   */

  /** @defgroup FT6X06
   * @{
   */



  /** @defgroup FT6X06_Exported_Types
   * @{
   */



  /** @defgroup FT6X06_Exported_Constants
   * @{
   */


#define  FT_6206_MAX_WIDTH              ((uint16_t)800)     /* Touchscreen pad max width   */
#define  FT_6206_MAX_HEIGHT             ((uint16_t)480)     /* Touchscreen pad max height  */


#define  FT_6206_MAX_WIDTH_HEIGHT       ((uint16_t)240)     


#define FT6206_STATUS_OK                0
#define FT6206_STATUS_NOT_OK            1


#define FT6206_I2C_NOT_INITIALIZED      0
#define FT6206_I2C_INITIALIZED          1


#define FT6206_MAX_DETECTABLE_TOUCH     2

  /**
   * @brief : Definitions for FT6206 I2C register addresses on 8 bit
   **/


#define FT6206_DEV_MODE_REG             0x00


#define FT6206_DEV_MODE_WORKING         0x00
#define FT6206_DEV_MODE_FACTORY         0x04

#define FT6206_DEV_MODE_MASK            0x7
#define FT6206_DEV_MODE_SHIFT           4


#define FT6206_GEST_ID_REG              0x01


#define FT6206_GEST_ID_NO_GESTURE       0x00
#define FT6206_GEST_ID_MOVE_UP          0x10
#define FT6206_GEST_ID_MOVE_RIGHT       0x14
#define FT6206_GEST_ID_MOVE_DOWN        0x18
#define FT6206_GEST_ID_MOVE_LEFT        0x1C
#define FT6206_GEST_ID_ZOOM_IN          0x48
#define FT6206_GEST_ID_ZOOM_OUT         0x49


#define FT6206_TD_STAT_REG              0x02


#define FT6206_TD_STAT_MASK             0x0F
#define FT6206_TD_STAT_SHIFT            0x00


#define FT6206_TOUCH_EVT_FLAG_PRESS_DOWN 0x00
#define FT6206_TOUCH_EVT_FLAG_LIFT_UP    0x01
#define FT6206_TOUCH_EVT_FLAG_CONTACT    0x02
#define FT6206_TOUCH_EVT_FLAG_NO_EVENT   0x03

#define FT6206_TOUCH_EVT_FLAG_SHIFT     6
#define FT6206_TOUCH_EVT_FLAG_MASK      (3 << FT6206_TOUCH_EVT_FLAG_SHIFT)

#define FT6206_MSB_MASK                 0x0F
#define FT6206_MSB_SHIFT                0


#define FT6206_LSB_MASK                 0xFF
#define FT6206_LSB_SHIFT                0

#define FT6206_P1_XH_REG                0x03
#define FT6206_P1_XL_REG                0x04
#define FT6206_P1_YH_REG                0x05
#define FT6206_P1_YL_REG                0x06


#define FT6206_P1_WEIGHT_REG            0x07


#define FT6206_TOUCH_WEIGHT_MASK        0xFF
#define FT6206_TOUCH_WEIGHT_SHIFT       0


#define FT6206_P1_MISC_REG              0x08


#define FT6206_TOUCH_AREA_MASK         (0x04 << 4)
#define FT6206_TOUCH_AREA_SHIFT        0x04

#define FT6206_P2_XH_REG               0x09
#define FT6206_P2_XL_REG               0x0A
#define FT6206_P2_YH_REG               0x0B
#define FT6206_P2_YL_REG               0x0C
#define FT6206_P2_WEIGHT_REG           0x0D
#define FT6206_P2_MISC_REG             0x0E


#define FT6206_TH_GROUP_REG            0x80


#define FT6206_THRESHOLD_MASK          0xFF
#define FT6206_THRESHOLD_SHIFT         0


#define FT6206_TH_DIFF_REG             0x85


#define FT6206_CTRL_REG                0x86




#define FT6206_CTRL_KEEP_ACTIVE_MODE    0x00


#define FT6206_CTRL_KEEP_AUTO_SWITCH_MONITOR_MODE  0x01


#define FT6206_TIMEENTERMONITOR_REG     0x87


#define FT6206_PERIODACTIVE_REG         0x88


#define FT6206_PERIODMONITOR_REG        0x89


#define FT6206_RADIAN_VALUE_REG         0x91


#define FT6206_OFFSET_LEFT_RIGHT_REG    0x92


#define FT6206_OFFSET_UP_DOWN_REG       0x93


#define FT6206_DISTANCE_LEFT_RIGHT_REG  0x94


#define FT6206_DISTANCE_UP_DOWN_REG     0x95


#define FT6206_DISTANCE_ZOOM_REG        0x96


#define FT6206_LIB_VER_H_REG            0xA1


#define FT6206_LIB_VER_L_REG            0xA2


#define FT6206_CIPHER_REG               0xA3


#define FT6206_GMODE_REG                0xA4

#define FT6206_G_MODE_INTERRUPT_MASK    0x03
#define FT6206_G_MODE_INTERRUPT_SHIFT   0x00


#define FT6206_G_MODE_INTERRUPT_POLLING 0x00
#define FT6206_G_MODE_INTERRUPT_TRIGGER 0x01


#define FT6206_PWR_MODE_REG             0xA5


#define FT6206_FIRMID_REG               0xA6


#define FT6206_CHIP_ID_REG              0xA8


#define FT6206_ID_VALUE                 0x11
#define FT6x36_ID_VALUE                 0xCD


#define FT6206_RELEASE_CODE_ID_REG      0xAF


#define FT6206_STATE_REG                0xBC

  /**
   * @}
   */



  /** @defgroup ft6x06_Exported_Macros
   * @{
   */



  /** @defgroup ft6x06_Exported_Functions
   * @{
   */

  /**
   * @brief ft6x06 Control functions
   */


/**
 * @brief  Initialize the ft6x06 communication bus
 *         from MCU to FT6206 : ie I2C channel initialization (if required).
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
 * @retval None
 */
void ft6x06_Init(uint16_t DeviceAddr);

/**
 * @brief  Software Reset the ft6x06.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
 * @retval None
 */
void ft6x06_Reset(uint16_t DeviceAddr);

/**
 * @brief  Read the ft6x06 device ID, pre intitalize I2C in case of need to be
 *         able to read the FT6206 device ID, and verify this is a FT6206.
 * @param  DeviceAddr: I2C FT6x06 Slave address.
 * @retval The Device ID (two bytes).
 */
uint16_t ft6x06_ReadID(uint16_t DeviceAddr);

/**
 * @brief  Configures the touch Screen IC device to start detecting touches
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
 * @retval None.
 */
void ft6x06_TS_Start(uint16_t DeviceAddr);

/**
 * @brief  Return if there is touches detected or not.
 *         Try to detect new touches and forget the old ones (reset internal global
 *         variables).
 * @param  DeviceAddr: Device address on communication Bus.
 * @retval : Number of active touches detected (can be 0, 1 or 2).
 */
uint8_t ft6x06_TS_DetectTouch(uint16_t DeviceAddr);

/**
 * @brief  Get the touch screen X and Y positions values
 *         Manage multi touch thanks to touch Index global
 *         variable 'ft6x06_handle.currActiveTouchIdx'.
 * @param  DeviceAddr: Device address on communication Bus.
 * @param  X: Pointer to X position value
 * @param  Y: Pointer to Y position value
 * @retval None.
 */
void ft6x06_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);

/**
 * @brief  Configure the FT6206 device to generate IT on given INT pin
 *         connected to MCU as EXTI.
 * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT6206).
 * @retval None
 */
void ft6x06_TS_EnableIT(uint16_t DeviceAddr);

/**
 * @brief  Configure the FT6206 device to stop generating IT on the given INT pin
 *         connected to MCU as EXTI.
 * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT6206).
 * @retval None
 */
void ft6x06_TS_DisableIT(uint16_t DeviceAddr);

/**
 * @brief  Get IT status from FT6206 interrupt status registers
 *         Should be called Following an EXTI coming to the MCU to know the detailed
 *         reason of the interrupt.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
 * @retval TS interrupts status
 */
uint8_t ft6x06_TS_ITStatus (uint16_t DeviceAddr);

/**
 * @brief  Clear IT status in FT6206 interrupt status clear registers
 *         Should be called Following an EXTI coming to the MCU.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
 * @retval TS interrupts status
 */
void ft6x06_TS_ClearIT (uint16_t DeviceAddr);



#if (TS_MULTI_TOUCH_SUPPORTED == 1)

/**
 * @brief  Get the last touch gesture identification (zoom, move up/down...).
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6x06).
 * @param  pGestureId : Pointer to get last touch gesture Identification.
 * @retval None.
 */
void ft6x06_TS_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId);

/**
 * @brief  Get the touch detailed informations on touch number 'touchIdx' (0..1)
 *         This touch detailed information contains :
 *         - weight that was applied to this touch
 *         - sub-area of the touch in the touch panel
 *         - event of linked to the touch (press down, lift up, ...)
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6x06).
 * @param  touchIdx : Passed index of the touch (0..1) on which we want to get the
 *                    detailed information.
 * @param  pWeight : Pointer to to get the weight information of 'touchIdx'.
 * @param  pArea   : Pointer to to get the sub-area information of 'touchIdx'.
 * @param  pEvent  : Pointer to to get the event information of 'touchIdx'.

 * @retval None.
 */
void ft6x06_TS_GetTouchInfo(uint16_t   DeviceAddr,
                            uint32_t   touchIdx,
                            uint32_t * pWeight,
                            uint32_t * pArea,
                            uint32_t * pEvent);

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */



/** @defgroup ft6x06_Imported_Functions
 * @{
 */


extern void     TS_IO_Init(void);
extern void     TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
extern uint8_t  TS_IO_Read(uint8_t Addr, uint8_t Reg);
extern uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
extern void     TS_IO_Delay(uint32_t Delay);

  /**
   * @}
   */



  /** @defgroup ft6x06_Imported_Globals
   * @{
   */



extern TS_DrvTypeDef ft6x06_ts_drv;

  /**
   * @}
   */

#ifdef __cplusplus
}
#endif
#endif /* __FT6X06_H */


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

