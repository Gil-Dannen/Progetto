/**
  ******************************************************************************
  * @file    hx8347i.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for HX8347I LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


#include "hx8347i.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup HX8347I
  * @brief     This file provides a set of functions needed to drive the
  *            HX8347I LCD.
  * @{
  */

/** @defgroup HX8347I_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup HX8347I_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup HX8347I_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup HX8347I_Private_Variables
  * @{
  */
LCD_DrvTypeDef   hx8347i_drv =
{
  hx8347i_Init,
  hx8347i_ReadID,
  hx8347i_DisplayOn,
  hx8347i_DisplayOff,
  hx8347i_SetCursor,
  hx8347i_WritePixel,
  hx8347i_ReadPixel,
  hx8347i_SetDisplayWindow,
  hx8347i_DrawHLine,
  hx8347i_DrawVLine,
  hx8347i_GetLcdPixelWidth,
  hx8347i_GetLcdPixelHeight,
  hx8347i_DrawBitmap,
};

static uint8_t Is_hx8347i_Initialized = 0;
static uint16_t ArrayRGB[320] = {0};

/**
  * @}
  */

/** @defgroup HX8347I_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup HX8347I_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the HX8347I LCD Component.
  * @param  None
  * @retval None
  */
void hx8347i_Init(void)
{
  if(Is_hx8347i_Initialized == 0)
  {
    Is_hx8347i_Initialized = 1;


    LCD_IO_Init();


    LCD_Delay(120);


    hx8347i_WriteReg(LCD_REG_64, 0x00);
    hx8347i_WriteReg(LCD_REG_65, 0x00);
    hx8347i_WriteReg(LCD_REG_66, 0x04);
    hx8347i_WriteReg(LCD_REG_67, 0x13);
    hx8347i_WriteReg(LCD_REG_68, 0x0E);
    hx8347i_WriteReg(LCD_REG_69, 0x2D);
    hx8347i_WriteReg(LCD_REG_70, 0x0E);
    hx8347i_WriteReg(LCD_REG_71, 0x57);
    hx8347i_WriteReg(LCD_REG_72, 0x09);
    hx8347i_WriteReg(LCD_REG_73, 0x13);
    hx8347i_WriteReg(LCD_REG_74, 0x19);
    hx8347i_WriteReg(LCD_REG_75, 0x1A);
    hx8347i_WriteReg(LCD_REG_76, 0x1B);
    hx8347i_WriteReg(LCD_REG_80, 0x12);
    hx8347i_WriteReg(LCD_REG_81, 0x31);
    hx8347i_WriteReg(LCD_REG_82, 0x2C);
    hx8347i_WriteReg(LCD_REG_83, 0x3B);
    hx8347i_WriteReg(LCD_REG_84, 0x3F);
    hx8347i_WriteReg(LCD_REG_85, 0x3F);
    hx8347i_WriteReg(LCD_REG_86, 0x28);
    hx8347i_WriteReg(LCD_REG_87, 0x71);
    hx8347i_WriteReg(LCD_REG_88, 0x04);
    hx8347i_WriteReg(LCD_REG_89, 0x05);
    hx8347i_WriteReg(LCD_REG_90, 0x06);
    hx8347i_WriteReg(LCD_REG_91, 0x0C);
    hx8347i_WriteReg(LCD_REG_92, 0x16);
    hx8347i_WriteReg(LCD_REG_93, 0x88);


    hx8347i_WriteReg(LCD_REG_27, 0x1E); /* VRH=4.60V */
    hx8347i_WriteReg(LCD_REG_28, 0x07); /* AP Crosstalk */
    hx8347i_WriteReg(LCD_REG_26, 0x01); /* BT (VGH~15V,VGL~-10V,DDVDH~5V) */
    hx8347i_WriteReg(LCD_REG_36, 0x38); /* VMH */
    hx8347i_WriteReg(LCD_REG_37, 0x5F); /* VML */


    hx8347i_WriteReg(LCD_REG_35, 0x6B);


    hx8347i_WriteReg(LCD_REG_232, 0x7C);
    hx8347i_WriteReg(LCD_REG_236, 0x14);
    hx8347i_WriteReg(LCD_REG_237, 0x0C);
    hx8347i_WriteReg(LCD_REG_228, 0x10);
    hx8347i_WriteReg(LCD_REG_229, 0x02);
    hx8347i_WriteReg(LCD_REG_230, 0x10);
    hx8347i_WriteReg(LCD_REG_231, 0x02);

    hx8347i_DisplayOn();


    hx8347i_WriteReg(LCD_REG_1, 0x00); /* DP_STB = 0, DP_STB_S = 0, SCROLL = 0, */
    hx8347i_SetDisplayWindow(0, 0, hx8347i_GetLcdPixelWidth(), hx8347i_GetLcdPixelHeight());
    hx8347i_WriteReg(LCD_REG_22, 0xA0); /* Memory access control: MY = 1, MX = 0, MV = 1, ML = 0 */
  }


  hx8347i_SetCursor(0, 0);


  LCD_IO_WriteReg(LCD_REG_34);
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void hx8347i_DisplayOn(void)
{

  hx8347i_WriteReg(LCD_REG_24, 0x99); /* Display frame rate = 75Hz UADJ */
  hx8347i_WriteReg(LCD_REG_25, 0x01); /* OSC_EN = 1 */
  hx8347i_WriteReg(LCD_REG_26, 0x04); /* BT[2:0] = 4 before sleep out */
  hx8347i_WriteReg(LCD_REG_31, 0x88); /* GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0*/
  LCD_Delay(5);
  hx8347i_WriteReg(LCD_REG_31, 0x80); /* GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
  LCD_Delay(5);
  hx8347i_WriteReg(LCD_REG_31, 0x90); /* GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
  LCD_Delay(5);
  hx8347i_WriteReg(LCD_REG_31, 0xD0); /* GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0, STB=0 */
  LCD_Delay(5);

  hx8347i_WriteReg(LCD_REG_23, 0x05); /* default 0x06 262k color,  0x05 65k color */

  hx8347i_WriteReg(LCD_REG_54, 0x09); /* SS_PANEL = 1, GS_PANEL = 0,REV_PANEL = 0, BGR_PANEL = 1 */


  hx8347i_WriteReg(LCD_REG_40, 0x38);
  LCD_Delay(60);
  hx8347i_WriteReg(LCD_REG_40, 0x3C);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void hx8347i_DisplayOff(void)
{

  hx8347i_WriteReg(LCD_REG_40, 0x38);
  LCD_Delay(60);
  hx8347i_WriteReg(LCD_REG_40, 0x04);


  hx8347i_WriteReg(LCD_REG_23, 0x0000); /* default 0x06 262k color,  0x05 65k color */
  hx8347i_WriteReg(LCD_REG_24, 0x0000); /* Display frame rate = 70Hz RADJ = '0110' */
  hx8347i_WriteReg(LCD_REG_25, 0x0000); /* OSC_EN = 1 */
  hx8347i_WriteReg(LCD_REG_28, 0x0000); /* AP[2:0] = 111 */
  hx8347i_WriteReg(LCD_REG_31, 0x0000); /* GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
  hx8347i_WriteReg(LCD_REG_54, 0x0000); /* SS_PANEL = 1, GS_PANEL = 0,REV_PANEL = 0, BGR_PANEL = 1 */
}

/**
  * @brief  Get the LCD pixel Width.
  * @param  None
  * @retval The Lcd Pixel Width
  */
uint16_t hx8347i_GetLcdPixelWidth(void)
{
 return (uint16_t)HX8347I_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Get the LCD pixel Height.
  * @param  None
  * @retval The Lcd Pixel Height
  */
uint16_t hx8347i_GetLcdPixelHeight(void)
{
 return (uint16_t)HX8347I_LCD_PIXEL_HEIGHT;
}

/**
  * @brief  Get the HX8347I ID.
  * @param  None
  * @retval The HX8347I ID
  */
uint16_t hx8347i_ReadID(void)
{
  if(Is_hx8347i_Initialized == 0)
  {
    LCD_IO_Init();


    LCD_Delay(120);
  }
  return (hx8347i_ReadReg(0x00));
}

/**
  * @brief  Set Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void hx8347i_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  hx8347i_WriteReg(LCD_REG_6, 0x00);
  hx8347i_WriteReg(LCD_REG_7, Xpos);
  hx8347i_WriteReg(LCD_REG_2, Ypos >> 8);
  hx8347i_WriteReg(LCD_REG_3, Ypos & 0xFF);
}

/**
  * @brief  Write pixel.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
* @param  RGBCode: the RGB pixel color
  * @retval None
  */
void hx8347i_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{

  hx8347i_SetCursor(Xpos, Ypos);


  LCD_IO_WriteReg(LCD_REG_34);


  LCD_IO_WriteMultipleData((uint8_t*)&RGBCode, 2);
}

/**
  * @brief  Read pixel.
  * @param  None
  * @retval the RGB pixel color
  */
uint16_t hx8347i_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{

  hx8347i_SetCursor(Xpos, Ypos);


  LCD_IO_ReadData(LCD_REG_34);


  return (LCD_IO_ReadData(LCD_REG_34));
}

/**
  * @brief  Writes to the selected LCD register.
* @param  LCDReg:      address of the selected register.
* @param  LCDRegValue: value to write to the selected register.
  * @retval None
  */
void hx8347i_WriteReg(uint8_t LCDReg, uint16_t LCDRegValue)
{
  LCD_IO_WriteReg(LCDReg);


  LCD_IO_WriteMultipleData((uint8_t*)&LCDRegValue, 2);
}

/**
  * @brief  Reads the selected LCD Register.
* @param  LCDReg: address of the selected register.
  * @retval LCD Register Value.
  */
uint16_t hx8347i_ReadReg(uint8_t LCDReg)
{

  LCD_IO_WriteReg(LCDReg);


  return (LCD_IO_ReadData(LCDReg));
}

/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void hx8347i_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{

  hx8347i_WriteReg(LCD_REG_6, (Xpos) >> 8); /* SP */
  hx8347i_WriteReg(LCD_REG_7, (Xpos) & 0xFF); /* SP */


  hx8347i_WriteReg(LCD_REG_8, (Xpos + Height - 1) >> 8); /* EP */
  hx8347i_WriteReg(LCD_REG_9, (Xpos + Height - 1) & 0xFF); /* EP */


  hx8347i_WriteReg(LCD_REG_2, (Ypos) >> 8); /* SC */
  hx8347i_WriteReg(LCD_REG_3, (Ypos) & 0xFF); /* SC */


  hx8347i_WriteReg(LCD_REG_4, (Ypos + Width - 1) >> 8); /* EC */
  hx8347i_WriteReg(LCD_REG_5, (Ypos + Width - 1) & 0xFF); /* EC */
}

/**
  * @brief  Draw vertical line.
* @param  RGBCode: Specifies the RGB color
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.
  * @retval None
  */
void hx8347i_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t i = 0;


  hx8347i_SetCursor(Xpos, Ypos);


  LCD_IO_WriteReg(LCD_REG_34);


  for(i = 0; i < Length; i++)
  {
    ArrayRGB[i] = RGBCode;
  }

  LCD_IO_WriteMultipleData((uint8_t*)&ArrayRGB[0], Length * 2);
}

/**
  * @brief  Draw vertical line.
* @param  RGBCode: Specifies the RGB color
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.
  * @retval None
  */
void hx8347i_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t counter = 0;


  hx8347i_SetCursor(Xpos, Ypos);


  LCD_IO_WriteReg(LCD_REG_34);


  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = RGBCode;
  }


  LCD_IO_WriteMultipleData((uint8_t*)&ArrayRGB[0], Length * 2);
}

/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param  BmpAddress: Bmp picture address in the internal Flash.
  * @retval None
  */
void hx8347i_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, size = 0;


  size = *(volatile uint16_t *) (pbmp + 2);
  size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;

  index = *(volatile uint16_t *) (pbmp + 10);
  index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
  size = (size - index)/2;
  pbmp += index;



  hx8347i_WriteReg(LCD_REG_22, 0xE0);


  hx8347i_SetCursor(Xpos, Ypos);


  LCD_IO_WriteReg(LCD_REG_34);

  LCD_IO_WriteMultipleData((uint8_t*)pbmp, size*2);



  hx8347i_WriteReg(LCD_REG_22, 0xA0);
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


