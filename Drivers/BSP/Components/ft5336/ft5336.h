/**
  ******************************************************************************
  * @file    ft5336.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the
  *          ft5336.c Touch screen driver.
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


#ifndef __FT5336_H
#define __FT5336_H

#ifdef __cplusplus
extern "C" {
#endif


#if !defined(TS_MONO_TOUCH_SUPPORTED)
#define TS_MULTI_TOUCH_SUPPORTED        1
#endif /* TS_MONO_TOUCH_SUPPORTED */


#include "../Common/ts.h"



#if defined(FT5336_ENABLE_ASSERT)

#define FT5336_ASSERT(__condition__)      do { if(__condition__) \
                                               {  \
                                                 while(1);  \
                                               } \
                                          }while(0)
#else

#define FT5336_ASSERT(__condition__)    do { if(__condition__) \
                                             {  \
                                                ;  \
                                             } \
                                            }while(0)
#endif /* FT5336_ENABLE_ASSERT == 1 */

/** @typedef ft5336_handle_TypeDef
 *  ft5336 Handle definition.
 */
typedef struct
{
  uint8_t i2cInitialized;


  uint8_t currActiveTouchNb;


  uint8_t currActiveTouchIdx;

} ft5336_handle_TypeDef;

  /** @addtogroup BSP
   * @{
   */

  /** @addtogroup Component
   * @{
   */

  /** @defgroup FT5336
   * @{
   */



  /** @defgroup FT5336_Exported_Types
   * @{
   */



  /** @defgroup FT5336_Exported_Constants
   * @{
   */


#define FT5336_I2C_SLAVE_ADDRESS            ((uint8_t)0x70)


#define FT5336_MAX_WIDTH                    ((uint16_t)480)     /* Touchscreen pad max width   */
#define FT5336_MAX_HEIGHT                   ((uint16_t)272)     /* Touchscreen pad max height  */


#define FT5336_STATUS_OK                    ((uint8_t)0x00)
#define FT5336_STATUS_NOT_OK                ((uint8_t)0x01)


#define FT5336_I2C_NOT_INITIALIZED          ((uint8_t)0x00)
#define FT5336_I2C_INITIALIZED              ((uint8_t)0x01)


#define FT5336_MAX_DETECTABLE_TOUCH         ((uint8_t)0x05)

  /**
   * @brief : Definitions for FT5336 I2C register addresses on 8 bit
   **/


#define FT5336_DEV_MODE_REG                 ((uint8_t)0x00)


#define FT5336_DEV_MODE_WORKING             ((uint8_t)0x00)
#define FT5336_DEV_MODE_FACTORY             ((uint8_t)0x04)

#define FT5336_DEV_MODE_MASK                ((uint8_t)0x07)
#define FT5336_DEV_MODE_SHIFT               ((uint8_t)0x04)


#define FT5336_GEST_ID_REG                  ((uint8_t)0x01)


#define FT5336_GEST_ID_NO_GESTURE           ((uint8_t)0x00)
#define FT5336_GEST_ID_MOVE_UP              ((uint8_t)0x10)
#define FT5336_GEST_ID_MOVE_RIGHT           ((uint8_t)0x14)
#define FT5336_GEST_ID_MOVE_DOWN            ((uint8_t)0x18)
#define FT5336_GEST_ID_MOVE_LEFT            ((uint8_t)0x1C)
#define FT5336_GEST_ID_SINGLE_CLICK         ((uint8_t)0x20)
#define FT5336_GEST_ID_DOUBLE_CLICK         ((uint8_t)0x22)
#define FT5336_GEST_ID_ROTATE_CLOCKWISE     ((uint8_t)0x28)
#define FT5336_GEST_ID_ROTATE_C_CLOCKWISE   ((uint8_t)0x29)
#define FT5336_GEST_ID_ZOOM_IN              ((uint8_t)0x40)
#define FT5336_GEST_ID_ZOOM_OUT             ((uint8_t)0x49)


#define FT5336_TD_STAT_REG                  ((uint8_t)0x02)


#define FT5336_TD_STAT_MASK                 ((uint8_t)0x0F)
#define FT5336_TD_STAT_SHIFT                ((uint8_t)0x00)


#define FT5336_TOUCH_EVT_FLAG_PRESS_DOWN    ((uint8_t)0x00)
#define FT5336_TOUCH_EVT_FLAG_LIFT_UP       ((uint8_t)0x01)
#define FT5336_TOUCH_EVT_FLAG_CONTACT       ((uint8_t)0x02)
#define FT5336_TOUCH_EVT_FLAG_NO_EVENT      ((uint8_t)0x03)

#define FT5336_TOUCH_EVT_FLAG_SHIFT         ((uint8_t)0x06)
#define FT5336_TOUCH_EVT_FLAG_MASK          ((uint8_t)(3 << FT5336_TOUCH_EVT_FLAG_SHIFT))

#define FT5336_TOUCH_POS_MSB_MASK           ((uint8_t)0x0F)
#define FT5336_TOUCH_POS_MSB_SHIFT          ((uint8_t)0x00)


#define FT5336_TOUCH_POS_LSB_MASK           ((uint8_t)0xFF)
#define FT5336_TOUCH_POS_LSB_SHIFT          ((uint8_t)0x00)

#define FT5336_P1_XH_REG                    ((uint8_t)0x03)
#define FT5336_P1_XL_REG                    ((uint8_t)0x04)
#define FT5336_P1_YH_REG                    ((uint8_t)0x05)
#define FT5336_P1_YL_REG                    ((uint8_t)0x06)


#define FT5336_P1_WEIGHT_REG                ((uint8_t)0x07)


#define FT5336_TOUCH_WEIGHT_MASK            ((uint8_t)0xFF)
#define FT5336_TOUCH_WEIGHT_SHIFT           ((uint8_t)0x00)


#define FT5336_P1_MISC_REG                  ((uint8_t)0x08)


#define FT5336_TOUCH_AREA_MASK              ((uint8_t)(0x04 << 4))
#define FT5336_TOUCH_AREA_SHIFT             ((uint8_t)0x04)

#define FT5336_P2_XH_REG                    ((uint8_t)0x09)
#define FT5336_P2_XL_REG                    ((uint8_t)0x0A)
#define FT5336_P2_YH_REG                    ((uint8_t)0x0B)
#define FT5336_P2_YL_REG                    ((uint8_t)0x0C)
#define FT5336_P2_WEIGHT_REG                ((uint8_t)0x0D)
#define FT5336_P2_MISC_REG                  ((uint8_t)0x0E)

#define FT5336_P3_XH_REG                    ((uint8_t)0x0F)
#define FT5336_P3_XL_REG                    ((uint8_t)0x10)
#define FT5336_P3_YH_REG                    ((uint8_t)0x11)
#define FT5336_P3_YL_REG                    ((uint8_t)0x12)
#define FT5336_P3_WEIGHT_REG                ((uint8_t)0x13)
#define FT5336_P3_MISC_REG                  ((uint8_t)0x14)

#define FT5336_P4_XH_REG                    ((uint8_t)0x15)
#define FT5336_P4_XL_REG                    ((uint8_t)0x16)
#define FT5336_P4_YH_REG                    ((uint8_t)0x17)
#define FT5336_P4_YL_REG                    ((uint8_t)0x18)
#define FT5336_P4_WEIGHT_REG                ((uint8_t)0x19)
#define FT5336_P4_MISC_REG                  ((uint8_t)0x1A)

#define FT5336_P5_XH_REG                    ((uint8_t)0x1B)
#define FT5336_P5_XL_REG                    ((uint8_t)0x1C)
#define FT5336_P5_YH_REG                    ((uint8_t)0x1D)
#define FT5336_P5_YL_REG                    ((uint8_t)0x1E)
#define FT5336_P5_WEIGHT_REG                ((uint8_t)0x1F)
#define FT5336_P5_MISC_REG                  ((uint8_t)0x20)

#define FT5336_P6_XH_REG                    ((uint8_t)0x21)
#define FT5336_P6_XL_REG                    ((uint8_t)0x22)
#define FT5336_P6_YH_REG                    ((uint8_t)0x23)
#define FT5336_P6_YL_REG                    ((uint8_t)0x24)
#define FT5336_P6_WEIGHT_REG                ((uint8_t)0x25)
#define FT5336_P6_MISC_REG                  ((uint8_t)0x26)

#define FT5336_P7_XH_REG                    ((uint8_t)0x27)
#define FT5336_P7_XL_REG                    ((uint8_t)0x28)
#define FT5336_P7_YH_REG                    ((uint8_t)0x29)
#define FT5336_P7_YL_REG                    ((uint8_t)0x2A)
#define FT5336_P7_WEIGHT_REG                ((uint8_t)0x2B)
#define FT5336_P7_MISC_REG                  ((uint8_t)0x2C)

#define FT5336_P8_XH_REG                    ((uint8_t)0x2D)
#define FT5336_P8_XL_REG                    ((uint8_t)0x2E)
#define FT5336_P8_YH_REG                    ((uint8_t)0x2F)
#define FT5336_P8_YL_REG                    ((uint8_t)0x30)
#define FT5336_P8_WEIGHT_REG                ((uint8_t)0x31)
#define FT5336_P8_MISC_REG                  ((uint8_t)0x32)

#define FT5336_P9_XH_REG                    ((uint8_t)0x33)
#define FT5336_P9_XL_REG                    ((uint8_t)0x34)
#define FT5336_P9_YH_REG                    ((uint8_t)0x35)
#define FT5336_P9_YL_REG                    ((uint8_t)0x36)
#define FT5336_P9_WEIGHT_REG                ((uint8_t)0x37)
#define FT5336_P9_MISC_REG                  ((uint8_t)0x38)

#define FT5336_P10_XH_REG                   ((uint8_t)0x39)
#define FT5336_P10_XL_REG                   ((uint8_t)0x3A)
#define FT5336_P10_YH_REG                   ((uint8_t)0x3B)
#define FT5336_P10_YL_REG                   ((uint8_t)0x3C)
#define FT5336_P10_WEIGHT_REG               ((uint8_t)0x3D)
#define FT5336_P10_MISC_REG                 ((uint8_t)0x3E)


#define FT5336_TH_GROUP_REG                 ((uint8_t)0x80)


#define FT5336_THRESHOLD_MASK               ((uint8_t)0xFF)
#define FT5336_THRESHOLD_SHIFT              ((uint8_t)0x00)


#define FT5336_TH_DIFF_REG                  ((uint8_t)0x85)


#define FT5336_CTRL_REG                     ((uint8_t)0x86)




#define FT5336_CTRL_KEEP_ACTIVE_MODE        ((uint8_t)0x00)


#define FT5336_CTRL_KEEP_AUTO_SWITCH_MONITOR_MODE  ((uint8_t)0x01


#define FT5336_TIMEENTERMONITOR_REG         ((uint8_t)0x87)


#define FT5336_PERIODACTIVE_REG             ((uint8_t)0x88)


#define FT5336_PERIODMONITOR_REG            ((uint8_t)0x89)


#define FT5336_RADIAN_VALUE_REG             ((uint8_t)0x91)


#define FT5336_OFFSET_LEFT_RIGHT_REG        ((uint8_t)0x92)


#define FT5336_OFFSET_UP_DOWN_REG           ((uint8_t)0x93)


#define FT5336_DISTANCE_LEFT_RIGHT_REG      ((uint8_t)0x94)


#define FT5336_DISTANCE_UP_DOWN_REG         ((uint8_t)0x95)


#define FT5336_DISTANCE_ZOOM_REG            ((uint8_t)0x96)


#define FT5336_LIB_VER_H_REG                ((uint8_t)0xA1)


#define FT5336_LIB_VER_L_REG                ((uint8_t)0xA2)


#define FT5336_CIPHER_REG                   ((uint8_t)0xA3)


#define FT5336_GMODE_REG                    ((uint8_t)0xA4)

#define FT5336_G_MODE_INTERRUPT_MASK        ((uint8_t)0x03)
#define FT5336_G_MODE_INTERRUPT_SHIFT       ((uint8_t)0x00)


#define FT5336_G_MODE_INTERRUPT_POLLING     ((uint8_t)0x00)
#define FT5336_G_MODE_INTERRUPT_TRIGGER     ((uint8_t)0x01)


#define FT5336_PWR_MODE_REG                 ((uint8_t)0xA5)


#define FT5336_FIRMID_REG                   ((uint8_t)0xA6)


#define FT5336_CHIP_ID_REG                  ((uint8_t)0xA8)


#define FT5336_ID_VALUE                     ((uint8_t)0x51)


#define FT5336_RELEASE_CODE_ID_REG          ((uint8_t)0xAF)


#define FT5336_STATE_REG                    ((uint8_t)0xBC)

  /**
   * @}
   */



  /** @defgroup ft5336_Exported_Macros
   * @{
   */



  /** @defgroup ft5336_Exported_Functions
   * @{
   */

  /**
   * @brief ft5336 Control functions
   */


/**
 * @brief  Initialize the ft5336 communication bus
 *         from MCU to FT5336 : ie I2C channel initialization (if required).
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
 * @retval None
 */
void ft5336_Init(uint16_t DeviceAddr);

/**
 * @brief  Software Reset the ft5336.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
 * @retval None
 */
void ft5336_Reset(uint16_t DeviceAddr);

/**
 * @brief  Read the ft5336 device ID, pre initialize I2C in case of need to be
 *         able to read the FT5336 device ID, and verify this is a FT5336.
 * @param  DeviceAddr: I2C FT5336 Slave address.
 * @retval The Device ID (two bytes).
 */
uint16_t ft5336_ReadID(uint16_t DeviceAddr);

/**
 * @brief  Configures the touch Screen IC device to start detecting touches
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
 * @retval None.
 */
void ft5336_TS_Start(uint16_t DeviceAddr);

/**
 * @brief  Return if there is touches detected or not.
 *         Try to detect new touches and forget the old ones (reset internal global
 *         variables).
 * @param  DeviceAddr: Device address on communication Bus.
 * @retval : Number of active touches detected (can be 0, 1 or 2).
 */
uint8_t ft5336_TS_DetectTouch(uint16_t DeviceAddr);

/**
 * @brief  Get the touch screen X and Y positions values
 *         Manage multi touch thanks to touch Index global
 *         variable 'ft5336_handle.currActiveTouchIdx'.
 * @param  DeviceAddr: Device address on communication Bus.
 * @param  X: Pointer to X position value
 * @param  Y: Pointer to Y position value
 * @retval None.
 */
void ft5336_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);

/**
 * @brief  Configure the FT5336 device to generate IT on given INT pin
 *         connected to MCU as EXTI.
 * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5336).
 * @retval None
 */
void ft5336_TS_EnableIT(uint16_t DeviceAddr);

/**
 * @brief  Configure the FT5336 device to stop generating IT on the given INT pin
 *         connected to MCU as EXTI.
 * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5336).
 * @retval None
 */
void ft5336_TS_DisableIT(uint16_t DeviceAddr);

/**
 * @brief  Get IT status from FT5336 interrupt status registers
 *         Should be called Following an EXTI coming to the MCU to know the detailed
 *         reason of the interrupt.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
 * @retval TS interrupts status
 */
uint8_t ft5336_TS_ITStatus (uint16_t DeviceAddr);

/**
 * @brief  Clear IT status in FT5336 interrupt status clear registers
 *         Should be called Following an EXTI coming to the MCU.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
 * @retval TS interrupts status
 */
void ft5336_TS_ClearIT (uint16_t DeviceAddr);



#if (TS_MULTI_TOUCH_SUPPORTED == 1)

/**
 * @brief  Get the last touch gesture identification (zoom, move up/down...).
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
 * @param  pGestureId : Pointer to get last touch gesture Identification.
 * @retval None.
 */
void ft5336_TS_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId);

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
                            uint32_t * pEvent);

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */



/** @defgroup ft5336_Imported_Functions
 * @{
 */


extern void     TS_IO_Init(void);
extern void    TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
extern uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg);
extern void    TS_IO_Delay(uint32_t Delay);

  /**
   * @}
   */



  /** @defgroup ft5336_Imported_Globals
   * @{
   */



extern TS_DrvTypeDef ft5336_ts_drv;

  /**
   * @}
   */

#ifdef __cplusplus
}
#endif
#endif /* __FT5336_H */


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

