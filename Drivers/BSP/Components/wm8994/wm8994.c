/**
  ******************************************************************************
  * @file    wm8994.c
  * @author  MCD Application Team
  * @brief   This file provides the WM8994 Audio Codec driver.
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


#include "wm8994.h"

/** @addtogroup BSP
  * @{
  */
  
/** @addtogroup Components
  * @{
  */ 

/** @addtogroup wm8994
  * @brief     This file provides a set of functions needed to drive the 
  *            WM8994 audio codec.
  * @{
  */

/** @defgroup WM8994_Private_Types
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup WM8994_Private_Defines
  * @{
  */
/* Uncomment this line to enable verifying data sent to codec after each write 
   operation (for debug purpose) */
#if !defined (VERIFY_WRITTENDATA)  

#endif /* VERIFY_WRITTENDATA */
/**
  * @}
  */ 

/** @defgroup WM8994_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup WM8994_Private_Variables
  * @{
  */


AUDIO_DrvTypeDef wm8994_drv = 
{
  wm8994_Init,
  wm8994_DeInit,
  wm8994_ReadID,

  wm8994_Play,
  wm8994_Pause,
  wm8994_Resume,
  wm8994_Stop,  

  wm8994_SetFrequency,
  wm8994_SetVolume,
  wm8994_SetMute,  
  wm8994_SetOutputMode,

  wm8994_Reset
};

static uint32_t outputEnabled = 0;
static uint32_t inputEnabled = 0;
static uint8_t ColdStartup = 1;

/**
  * @}
  */ 

/** @defgroup WM8994_Function_Prototypes
  * @{
  */
static uint8_t CODEC_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
/**
  * @}
  */ 


/** @defgroup WM8994_Private_Functions
  * @{
  */ 

/**
  * @brief Initializes the audio codec and the control interface.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param OutputInputDevice: can be OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *  OUTPUT_DEVICE_BOTH, OUTPUT_DEVICE_AUTO, INPUT_DEVICE_DIGITAL_MICROPHONE_1,
  *  INPUT_DEVICE_DIGITAL_MICROPHONE_2, INPUT_DEVICE_DIGITAL_MIC1_MIC2, 
  *  INPUT_DEVICE_INPUT_LINE_1 or INPUT_DEVICE_INPUT_LINE_2.
  * @param Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param AudioFreq: Audio Frequency 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_Init(uint16_t DeviceAddr, uint16_t OutputInputDevice, uint8_t Volume, uint32_t AudioFreq)
{
  uint32_t counter = 0;
  uint16_t output_device = OutputInputDevice & 0xFF;
  uint16_t input_device = OutputInputDevice & 0xFF00;
  uint16_t power_mgnt_reg_1 = 0;
  

  AUDIO_IO_Init();

  counter += CODEC_IO_Write(DeviceAddr, 0x102, 0x0003);
  counter += CODEC_IO_Write(DeviceAddr, 0x817, 0x0000);
  counter += CODEC_IO_Write(DeviceAddr, 0x102, 0x0000);


  counter += CODEC_IO_Write(DeviceAddr, 0x39, 0x006C);


  if (input_device > 0)
  {
    counter += CODEC_IO_Write(DeviceAddr, 0x01, 0x0013);
  }
  else
  {
    counter += CODEC_IO_Write(DeviceAddr, 0x01, 0x0003);
  }


  AUDIO_IO_Delay(50);


  if (output_device > 0)
  {
    outputEnabled = 1;

    switch (output_device)
    {
    case OUTPUT_DEVICE_SPEAKER:
      /* Enable DAC1 (Left), Enable DAC1 (Right),
      Disable DAC2 (Left), Disable DAC2 (Right)*/
      counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0C0C);


      counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0000);


      counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0000);


      counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0002);
      break;

    case OUTPUT_DEVICE_HEADPHONE:
      /* Disable DAC1 (Left), Disable DAC1 (Right),
      Enable DAC2 (Left), Enable DAC2 (Right)*/
      counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303);


      counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);


      counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);


      counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0000);


      counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0000);
      break;

    case OUTPUT_DEVICE_BOTH:
      if (input_device == INPUT_DEVICE_DIGITAL_MIC1_MIC2)
      {
        /* Enable DAC1 (Left), Enable DAC1 (Right),
        also Enable DAC2 (Left), Enable DAC2 (Right)*/
        counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303 | 0x0C0C);
        
        /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path
        Enable the AIF1 Timeslot 1 (Left) to DAC 1 (Left) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0003);
        
        /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path
        Enable the AIF1 Timeslot 1 (Right) to DAC 1 (Right) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0003);
        
        /* Enable the AIF1 Timeslot 0 (Left) to DAC 2 (Left) mixer path
        Enable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path  */
        counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0003);
        
        /* Enable the AIF1 Timeslot 0 (Right) to DAC 2 (Right) mixer path
        Enable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0003);
      }
      else
      {
        /* Enable DAC1 (Left), Enable DAC1 (Right),
        also Enable DAC2 (Left), Enable DAC2 (Right)*/
        counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303 | 0x0C0C);
        

        counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);
        

        counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);
        

        counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0002);
        

        counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0002);      
      }
      break;

    case OUTPUT_DEVICE_AUTO :
    default:
      /* Disable DAC1 (Left), Disable DAC1 (Right),
      Enable DAC2 (Left), Enable DAC2 (Right)*/
      counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303);


      counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);


      counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);


      counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0000);


      counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0000);
      break;
    }
  }
  else
  {
    outputEnabled = 0;
  }


  if (input_device > 0)
  {
    inputEnabled = 1;
    switch (input_device)
    {
    case INPUT_DEVICE_DIGITAL_MICROPHONE_2 :
      /* Enable AIF1ADC2 (Left), Enable AIF1ADC2 (Right)
       * Enable DMICDAT2 (Left), Enable DMICDAT2 (Right)
       * Enable Left ADC, Enable Right ADC */
      counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x0C30);


      counter += CODEC_IO_Write(DeviceAddr, 0x450, 0x00DB);


      counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x6000);


      counter += CODEC_IO_Write(DeviceAddr, 0x608, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x609, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x700, 0x000E);
      break;

    case INPUT_DEVICE_INPUT_LINE_1 :

      counter += CODEC_IO_Write(DeviceAddr, 0x28, 0x0011);


      counter += CODEC_IO_Write(DeviceAddr, 0x29, 0x0035);


      counter += CODEC_IO_Write(DeviceAddr, 0x2A, 0x0035);

      /* Enable AIF1ADC1 (Left), Enable AIF1ADC1 (Right)
       * Enable Left ADC, Enable Right ADC */
      counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x0303);


      counter += CODEC_IO_Write(DeviceAddr, 0x440, 0x00DB);


      counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x6350);


      counter += CODEC_IO_Write(DeviceAddr, 0x606, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x607, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x700, 0x000D);
      break;

    case INPUT_DEVICE_DIGITAL_MICROPHONE_1 :
      /* Enable AIF1ADC1 (Left), Enable AIF1ADC1 (Right)
       * Enable DMICDAT1 (Left), Enable DMICDAT1 (Right)
       * Enable Left ADC, Enable Right ADC */
      counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x030C);


      counter += CODEC_IO_Write(DeviceAddr, 0x440, 0x00DB);


      counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x6350);


      counter += CODEC_IO_Write(DeviceAddr, 0x606, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x607, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x700, 0x000D);
      break; 
    case INPUT_DEVICE_DIGITAL_MIC1_MIC2 :
      /* Enable AIF1ADC1 (Left), Enable AIF1ADC1 (Right)
       * Enable DMICDAT1 (Left), Enable DMICDAT1 (Right)
       * Enable Left ADC, Enable Right ADC */
      counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x0F3C);


      counter += CODEC_IO_Write(DeviceAddr, 0x450, 0x00DB);
      

      counter += CODEC_IO_Write(DeviceAddr, 0x440, 0x00DB);


      counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x63A0);


      counter += CODEC_IO_Write(DeviceAddr, 0x606, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x607, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x608, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x609, 0x0002);
      

      counter += CODEC_IO_Write(DeviceAddr, 0x700, 0x000D);
      break;    
    case INPUT_DEVICE_INPUT_LINE_2 :
    default:

      counter++;
      break;
    }
  }
  else
  {
    inputEnabled = 0;
  }
  

  switch (AudioFreq)
  {
  case  AUDIO_FREQUENCY_8K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0003);
    break;
    
  case  AUDIO_FREQUENCY_16K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0033);
    break;

  case  AUDIO_FREQUENCY_32K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0063);
    break;
    
  case  AUDIO_FREQUENCY_48K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0083);
    break;
    
  case  AUDIO_FREQUENCY_96K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x00A3);
    break;
    
  case  AUDIO_FREQUENCY_11K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0013);
    break;
    
  case  AUDIO_FREQUENCY_22K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0043);
    break;
    
  case  AUDIO_FREQUENCY_44K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0073);
    break; 
    
  default:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0083);
    break; 
  }

  if(input_device == INPUT_DEVICE_DIGITAL_MIC1_MIC2)
  {

  counter += CODEC_IO_Write(DeviceAddr, 0x300, 0x4018);    
  }
  else
  {

  counter += CODEC_IO_Write(DeviceAddr, 0x300, 0x4010);
  }
  

  counter += CODEC_IO_Write(DeviceAddr, 0x302, 0x0000);
  

  counter += CODEC_IO_Write(DeviceAddr, 0x208, 0x000A);
  

  counter += CODEC_IO_Write(DeviceAddr, 0x200, 0x0001);

  if (output_device > 0)  /* Audio output selected */
  {
    if (output_device == OUTPUT_DEVICE_HEADPHONE)
    {      

      counter += CODEC_IO_Write(DeviceAddr, 0x2D, 0x0100);
      

      counter += CODEC_IO_Write(DeviceAddr, 0x2E, 0x0100);    
            

      if(ColdStartup)
      {
        counter += CODEC_IO_Write(DeviceAddr,0x110,0x8100);
        
        ColdStartup=0;

        AUDIO_IO_Delay(300);
      }
      else /* Headphone Warm Start-Up */
      { 
        counter += CODEC_IO_Write(DeviceAddr,0x110,0x8108);

        AUDIO_IO_Delay(50);
      }


      counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0000);
    }



    counter += CODEC_IO_Write(DeviceAddr, 0x03, 0x0300);


    counter += CODEC_IO_Write(DeviceAddr, 0x22, 0x0000);


    counter += CODEC_IO_Write(DeviceAddr, 0x23, 0x0000);

    /* Unmute DAC2 (Left) to Left Speaker Mixer (SPKMIXL) path,
    Unmute DAC2 (Right) to Right Speaker Mixer (SPKMIXR) path */
    counter += CODEC_IO_Write(DeviceAddr, 0x36, 0x0300);


    counter += CODEC_IO_Write(DeviceAddr, 0x01, 0x3003);



    if (input_device == INPUT_DEVICE_DIGITAL_MIC1_MIC2)
    {

    counter += CODEC_IO_Write(DeviceAddr, 0x51, 0x0205);
    }
    else
    {

    counter += CODEC_IO_Write(DeviceAddr, 0x51, 0x0005);      
    }



    power_mgnt_reg_1 |= 0x0303 | 0x3003;
    counter += CODEC_IO_Write(DeviceAddr, 0x01, power_mgnt_reg_1);


    counter += CODEC_IO_Write(DeviceAddr, 0x60, 0x0022);


    counter += CODEC_IO_Write(DeviceAddr, 0x4C, 0x9F25);


    AUDIO_IO_Delay(15);


    counter += CODEC_IO_Write(DeviceAddr, 0x2D, 0x0001);


    counter += CODEC_IO_Write(DeviceAddr, 0x2E, 0x0001);



    counter += CODEC_IO_Write(DeviceAddr, 0x03, 0x0030 | 0x0300);


    counter += CODEC_IO_Write(DeviceAddr, 0x54, 0x0033);


    AUDIO_IO_Delay(257);


    counter += CODEC_IO_Write(DeviceAddr, 0x60, 0x00EE);




    counter += CODEC_IO_Write(DeviceAddr, 0x610, 0x00C0);


    counter += CODEC_IO_Write(DeviceAddr, 0x611, 0x00C0);


    counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0010);


    counter += CODEC_IO_Write(DeviceAddr, 0x612, 0x00C0);


    counter += CODEC_IO_Write(DeviceAddr, 0x613, 0x00C0);


    counter += CODEC_IO_Write(DeviceAddr, 0x422, 0x0010);
    

    wm8994_SetVolume(DeviceAddr, Volume);
  }

  if (input_device > 0) /* Audio input selected */
  {
    if ((input_device == INPUT_DEVICE_DIGITAL_MICROPHONE_1) || (input_device == INPUT_DEVICE_DIGITAL_MICROPHONE_2))
    {

      power_mgnt_reg_1 |= 0x0013;
      counter += CODEC_IO_Write(DeviceAddr, 0x01, power_mgnt_reg_1);


      counter += CODEC_IO_Write(DeviceAddr, 0x620, 0x0002);


      counter += CODEC_IO_Write(DeviceAddr, 0x411, 0x3800);
    }
    else if(input_device == INPUT_DEVICE_DIGITAL_MIC1_MIC2)
    {

      power_mgnt_reg_1 |= 0x0013;
      counter += CODEC_IO_Write(DeviceAddr, 0x01, power_mgnt_reg_1);


      counter += CODEC_IO_Write(DeviceAddr, 0x620, 0x0002);
    

      counter += CODEC_IO_Write(DeviceAddr, 0x410, 0x1800);
      

      counter += CODEC_IO_Write(DeviceAddr, 0x411, 0x1800);      
    }    
    else if ((input_device == INPUT_DEVICE_INPUT_LINE_1) || (input_device == INPUT_DEVICE_INPUT_LINE_2))
    {


      counter += CODEC_IO_Write(DeviceAddr, 0x18, 0x000B);


      counter += CODEC_IO_Write(DeviceAddr, 0x1A, 0x000B);


      counter += CODEC_IO_Write(DeviceAddr, 0x410, 0x1800);
    }

    wm8994_SetVolume(DeviceAddr, Volume);
  }

  return counter;  
}

/**
  * @brief  Deinitializes the audio codec.
  * @param  None
  * @retval  None
  */
void wm8994_DeInit(void)
{

  AUDIO_IO_DeInit();
}

/**
  * @brief  Get the WM8994 ID.
  * @param DeviceAddr: Device address on communication Bus.
  * @retval The WM8994 ID 
  */
uint32_t wm8994_ReadID(uint16_t DeviceAddr)
{

  AUDIO_IO_Init();

  return ((uint32_t)AUDIO_IO_Read(DeviceAddr, WM8994_CHIPID_ADDR));
}

/**
  * @brief Start the audio Codec play feature.
  * @note For this codec no Play options are required.
  * @param DeviceAddr: Device address on communication Bus.   
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_Play(uint16_t DeviceAddr, uint16_t* pBuffer, uint16_t Size)
{
  uint32_t counter = 0;
 


  counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_OFF);
  
  return counter;
}

/**
  * @brief Pauses playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_Pause(uint16_t DeviceAddr)
{  
  uint32_t counter = 0;
 


  counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_ON);
  

  counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x01);
 
  return counter;
}

/**
  * @brief Resumes playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_Resume(uint16_t DeviceAddr)
{
  uint32_t counter = 0;
 


  counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_OFF);
  
  return counter;
}

/**
  * @brief Stops audio Codec playing. It powers down the codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @param CodecPdwnMode: selects the  power down mode.
  *          - CODEC_PDWN_SW: only mutes the audio codec. When resuming from this 
  *                           mode the codec keeps the previous initialization
  *                           (no need to re-Initialize the codec registers).
  *          - CODEC_PDWN_HW: Physically power down the codec. When resuming from this
  *                           mode, the codec is set to default configuration 
  *                           (user should re-Initialize the codec in order to 
  *                            play again the audio stream).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_Stop(uint16_t DeviceAddr, uint32_t CodecPdwnMode)
{
  uint32_t counter = 0;

  if (outputEnabled != 0)
  {

    counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_ON);

    if (CodecPdwnMode == CODEC_PDWN_SW)
    {

    }
    else /* CODEC_PDWN_HW */
    {

      counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0200);


      counter += CODEC_IO_Write(DeviceAddr, 0x422, 0x0200);


      counter += CODEC_IO_Write(DeviceAddr, 0x2D, 0x0000);


      counter += CODEC_IO_Write(DeviceAddr, 0x2E, 0x0000);


      counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0000);


      counter += CODEC_IO_Write(DeviceAddr, 0x0000, 0x0000);

      outputEnabled = 0;
    }
  }
  return counter;
}

/**
  * @brief Sets higher or lower the codec volume level.
  * @param DeviceAddr: Device address on communication Bus.
  * @param Volume: a byte value from 0 to 255 (refer to codec registers 
  *         description for more details).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_SetVolume(uint16_t DeviceAddr, uint8_t Volume)
{
  uint32_t counter = 0;
  uint8_t convertedvol = VOLUME_CONVERT(Volume);


  if (outputEnabled != 0)
  {
    if(convertedvol > 0x3E)
    {

      counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_OFF);


      counter += CODEC_IO_Write(DeviceAddr, 0x1C, 0x3F | 0x140);


      counter += CODEC_IO_Write(DeviceAddr, 0x1D, 0x3F | 0x140);


      counter += CODEC_IO_Write(DeviceAddr, 0x26, 0x3F | 0x140);


      counter += CODEC_IO_Write(DeviceAddr, 0x27, 0x3F | 0x140);
    }
    else if (Volume == 0)
    {

      counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_ON);
    }
    else
    {

      counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_OFF);


      counter += CODEC_IO_Write(DeviceAddr, 0x1C, convertedvol | 0x140);


      counter += CODEC_IO_Write(DeviceAddr, 0x1D, convertedvol | 0x140);


      counter += CODEC_IO_Write(DeviceAddr, 0x26, convertedvol | 0x140);


      counter += CODEC_IO_Write(DeviceAddr, 0x27, convertedvol | 0x140);
    }
  }


  if (inputEnabled != 0)
  {
    convertedvol = VOLUME_IN_CONVERT(Volume);


    counter += CODEC_IO_Write(DeviceAddr, 0x400, convertedvol | 0x100);


    counter += CODEC_IO_Write(DeviceAddr, 0x401, convertedvol | 0x100);


    counter += CODEC_IO_Write(DeviceAddr, 0x404, convertedvol | 0x100);


    counter += CODEC_IO_Write(DeviceAddr, 0x405, convertedvol | 0x100);
  }
  return counter;
}

/**
  * @brief Enables or disables the mute feature on the audio codec.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param Cmd: AUDIO_MUTE_ON to enable the mute or AUDIO_MUTE_OFF to disable the
  *             mute mode.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_SetMute(uint16_t DeviceAddr, uint32_t Cmd)
{
  uint32_t counter = 0;
  
  if (outputEnabled != 0)
  {

    if(Cmd == AUDIO_MUTE_ON)
    {

      counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0200);


      counter += CODEC_IO_Write(DeviceAddr, 0x422, 0x0200);
    }
    else /* AUDIO_MUTE_OFF Disable the Mute */
    {

      counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0010);


      counter += CODEC_IO_Write(DeviceAddr, 0x422, 0x0010);
    }
  }
  return counter;
}

/**
  * @brief Switch dynamically (while audio file is played) the output target 
  *         (speaker or headphone).
  * @param DeviceAddr: Device address on communication Bus.
  * @param Output: specifies the audio output target: OUTPUT_DEVICE_SPEAKER,
  *         OUTPUT_DEVICE_HEADPHONE, OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_SetOutputMode(uint16_t DeviceAddr, uint8_t Output)
{
  uint32_t counter = 0; 
  
  switch (Output) 
  {
  case OUTPUT_DEVICE_SPEAKER:
    /* Enable DAC1 (Left), Enable DAC1 (Right), 
    Disable DAC2 (Left), Disable DAC2 (Right)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0C0C);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0000);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0000);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0002);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0002);
    break;
    
  case OUTPUT_DEVICE_HEADPHONE:
    /* Disable DAC1 (Left), Disable DAC1 (Right), 
    Enable DAC2 (Left), Enable DAC2 (Right)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0000);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0000);
    break;
    
  case OUTPUT_DEVICE_BOTH:
    /* Enable DAC1 (Left), Enable DAC1 (Right), 
    also Enable DAC2 (Left), Enable DAC2 (Right)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303 | 0x0C0C);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0002);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0002);
    break;
    
  default:
    /* Disable DAC1 (Left), Disable DAC1 (Right), 
    Enable DAC2 (Left), Enable DAC2 (Right)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0000);
    

    counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0000);
    break;    
  }  
  return counter;
}

/**
  * @brief Sets new frequency.
  * @param DeviceAddr: Device address on communication Bus.
  * @param AudioFreq: Audio frequency used to play the audio stream.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_SetFrequency(uint16_t DeviceAddr, uint32_t AudioFreq)
{
  uint32_t counter = 0;
 

  switch (AudioFreq)
  {
  case  AUDIO_FREQUENCY_8K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0003);
    break;
    
  case  AUDIO_FREQUENCY_16K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0033);
    break;

  case  AUDIO_FREQUENCY_32K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0063);
    break;
    
  case  AUDIO_FREQUENCY_48K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0083);
    break;
    
  case  AUDIO_FREQUENCY_96K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x00A3);
    break;
    
  case  AUDIO_FREQUENCY_11K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0013);
    break;
    
  case  AUDIO_FREQUENCY_22K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0043);
    break;
    
  case  AUDIO_FREQUENCY_44K:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0073);
    break; 
    
  default:

    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0083);
    break; 
  }
  return counter;
}

/**
  * @brief Resets wm8994 registers.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm8994_Reset(uint16_t DeviceAddr)
{
  uint32_t counter = 0;
  

  counter = CODEC_IO_Write(DeviceAddr, 0x0000, 0x0000);
  outputEnabled = 0;
  inputEnabled=0;

  return counter;
}

/**
  * @brief  Writes/Read a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
static uint8_t CODEC_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
  uint32_t result = 0;
  
 AUDIO_IO_Write(Addr, Reg, Value);
  
#ifdef VERIFY_WRITTENDATA

  result = (AUDIO_IO_Read(Addr, Reg) == Value)? 0:1;
#endif /* VERIFY_WRITTENDATA */
  
  return result;
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


