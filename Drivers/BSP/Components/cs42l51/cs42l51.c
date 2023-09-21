/**
  ******************************************************************************
  * @file    cs42l51.c
  * @author  MCD Application Team
  * @brief   This file provides the CS42L51 Audio Codec driver.   
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


#include "cs42l51.h"

/** @addtogroup BSP
  * @{
  */
  
/** @addtogroup Components
  * @{
  */ 

/** @addtogroup CS42L51
  * @brief     This file provides a set of functions needed to drive the 
  *            CS42L51 audio codec.
  * @{
  */

/** @defgroup CS42L51_Exported_Variables
  * @{
  */


AUDIO_DrvTypeDef cs42l51_drv = 
{
  cs42l51_Init,
  cs42l51_DeInit,
  cs42l51_ReadID,

  cs42l51_Play,
  cs42l51_Pause,
  cs42l51_Resume,
  cs42l51_Stop,  
  
  cs42l51_SetFrequency,  
  cs42l51_SetVolume,
  cs42l51_SetMute,  
  cs42l51_SetOutputMode,
  cs42l51_Reset,
};
  
/**
  * @}
  */ 

/** @defgroup CS42L51_Private_Types
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup CS42L51_Private_Defines
  * @{
  */
/* Uncomment this line to enable verifying data sent to codec after each write 
   operation (for debug purpose) */
#if !defined (VERIFY_WRITTENDATA)  
#define VERIFY_WRITTENDATA
#endif /* VERIFY_WRITTENDATA */
/**
  * @}
  */ 

/** @defgroup CS42L51_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup CS42L51_Private_Variables
  * @{
  */

static uint8_t Is_CS42L51_Initialized = 0;
static uint8_t Is_CS42L51_Stop        = 1;

static uint16_t CS42L51_Device = OUTPUT_DEVICE_HEADPHONE;
  
/**
  * @}
  */ 

/** @defgroup CS42L51_Private_Functions
  * @{
  */
static uint8_t CODEC_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
/**
  * @}
  */ 

/** @addtogroup CS42L51_Exported_Functions
  * @{
  */ 

/**
  * @brief Initialize the audio codec and the control interface.
  * @param DeviceAddr: Device address on communication bus.
  * @param Device:     Can be combination values of OUTPUT_DEVICE_HEADPHONE and
  *                       INPUT_DEVICE_MIC1.
  * @param Volume:     Initial output volume level (from 0 (-100dB) to 100 (0dB)).
  * @param AudioFreq:  Initial audio frequency (currently not used).
  * @retval 0 if correct communication, else wrong communication.
  */
uint32_t cs42l51_Init(uint16_t DeviceAddr, uint16_t Device, uint8_t Volume, uint32_t AudioFreq)
{
  uint32_t counter = 0;
  uint8_t  Value;
  

  if(Is_CS42L51_Initialized == 0)
  {

    AUDIO_IO_Init();

    Is_CS42L51_Initialized = 1;
  }
  else
  {

    counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x7E);
    Value = AUDIO_IO_Read(DeviceAddr, 0x03);
    counter += CODEC_IO_Write(DeviceAddr, 0x03, (Value | 0x0E));
    

    Value = AUDIO_IO_Read(DeviceAddr, 0x09);
    counter += CODEC_IO_Write(DeviceAddr, 0x09, (Value & 0xFC));
    

    Value = AUDIO_IO_Read(DeviceAddr, 0x02);
    counter += CODEC_IO_Write(DeviceAddr, 0x02, (Value | 0x01));
  }


  Value = AUDIO_IO_Read(DeviceAddr, 0x03);
  counter += CODEC_IO_Write(DeviceAddr, 0x03, ((Value & 0x0E) | 0xA0));


  counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x0C);
  

  counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x00);


  counter += CODEC_IO_Write(DeviceAddr, 0x06, 0x00);


  counter += CODEC_IO_Write(DeviceAddr, 0x07, 0x32);
  

  counter += CODEC_IO_Write(DeviceAddr, 0x08, 0xC3);


  counter += CODEC_IO_Write(DeviceAddr, 0x09, 0x42);


  counter += CODEC_IO_Write(DeviceAddr, 0x0A, 0xC0);


  counter += CODEC_IO_Write(DeviceAddr, 0x0B, 0xC0);
  

  counter += CODEC_IO_Write(DeviceAddr, 0x0C, 0x00);


  counter += CODEC_IO_Write(DeviceAddr, 0x0D, 0x00);


  counter += CODEC_IO_Write(DeviceAddr, 0x0E, 0x80);
  

  counter += CODEC_IO_Write(DeviceAddr, 0x0F, 0x80);
  

  counter += CODEC_IO_Write(DeviceAddr, 0x10, 0x00);


  counter += CODEC_IO_Write(DeviceAddr, 0x11, 0x00);


  counter += CODEC_IO_Write(DeviceAddr, 0x18, 0x00);

  if(Device & OUTPUT_DEVICE_HEADPHONE)
  {
    Value = VOLUME_CONVERT(Volume);

    counter += CODEC_IO_Write(DeviceAddr, 0x16, Value);

    counter += CODEC_IO_Write(DeviceAddr, 0x17, Value);
  }
  

  CS42L51_Device = Device;
  

  return counter;  
}

/**
  * @brief  Deinitialize the audio codec.
  * @param  None
  * @retval  None
  */
void cs42l51_DeInit(void)
{

  AUDIO_IO_DeInit();
  
  Is_CS42L51_Initialized = 0;
}

/**
  * @brief  Get the CS42L51 ID.
  * @param DeviceAddr: Device address on communication Bus.   
  * @retval The CS42L51 ID 
  */
uint32_t cs42l51_ReadID(uint16_t DeviceAddr)
{
  uint8_t Value;
  
  if(Is_CS42L51_Initialized == 0)
  {

    AUDIO_IO_Init(); 
    
    Value = AUDIO_IO_Read(DeviceAddr, CS42L51_CHIPID_ADDR);
    Value = (Value & CS42L51_ID_MASK);


    AUDIO_IO_DeInit();
  }
  else
  {
    Value = AUDIO_IO_Read(DeviceAddr, CS42L51_CHIPID_ADDR);
    Value = (Value & CS42L51_ID_MASK);
  }
  
  return((uint32_t) Value);
}

/**
  * @brief Start the audio Codec play feature.
  * @note For this codec no Play options are required.
  * @param DeviceAddr: Device address on communication Bus.   
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_Play(uint16_t DeviceAddr, uint16_t* pBuffer, uint16_t Size)
{
  uint32_t counter = 0;
  uint8_t  Value;
  
  if(Is_CS42L51_Stop == 1)
  {

    counter += cs42l51_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

    if(CS42L51_Device & OUTPUT_DEVICE_HEADPHONE)
    {

      counter += CODEC_IO_Write(DeviceAddr, 0x09, 0x42);


      Value = AUDIO_IO_Read(DeviceAddr, 0x02);
      counter += CODEC_IO_Write(DeviceAddr, 0x02, (Value & 0x9F));
    }

    if(CS42L51_Device & INPUT_DEVICE_MIC1)
    {

      Value = AUDIO_IO_Read(DeviceAddr, 0x02);
      counter += CODEC_IO_Write(DeviceAddr, 0x02, (Value & 0xF5));
      

      Value = AUDIO_IO_Read(DeviceAddr, 0x03);
      counter += CODEC_IO_Write(DeviceAddr, 0x03, (Value & 0xF9));
    }
    

    Value = AUDIO_IO_Read(DeviceAddr, 0x02);
    counter += CODEC_IO_Write(DeviceAddr, 0x02, (Value & 0xFE));

    Is_CS42L51_Stop = 0;
  }
  

  return counter;  
}

/**
  * @brief Pause playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_Pause(uint16_t DeviceAddr)
{  
  uint32_t counter = 0;
 


  counter += cs42l51_SetMute(DeviceAddr, AUDIO_MUTE_ON);
   
  return counter;
}

/**
  * @brief Resume playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_Resume(uint16_t DeviceAddr)
{
  uint32_t counter = 0;


  counter += cs42l51_SetMute(DeviceAddr, AUDIO_MUTE_OFF);
  
  return counter;
}

/**
  * @brief Stop audio Codec playing. It powers down the codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @param CodecPdwnMode: selects the  power down mode (currently not used).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_Stop(uint16_t DeviceAddr, uint32_t CodecPdwnMode)
{
  uint32_t counter = 0;
  uint8_t  Value;
  

  counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x7E);
  Value = AUDIO_IO_Read(DeviceAddr, 0x03);
  counter += CODEC_IO_Write(DeviceAddr, 0x03, (Value | 0x0E));
  

  Value = AUDIO_IO_Read(DeviceAddr, 0x09);
  counter += CODEC_IO_Write(DeviceAddr, 0x09, (Value & 0xFC));


  Value = AUDIO_IO_Read(DeviceAddr, 0x02);
  counter += CODEC_IO_Write(DeviceAddr, 0x02, (Value | 0x01));

  Is_CS42L51_Stop = 1;
  return counter;
}

/**
  * @brief Set higher or lower the codec volume level.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param Volume: output volume level (from 0 (-100dB) to 100 (0dB)).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_SetVolume(uint16_t DeviceAddr, uint8_t Volume)
{
  uint32_t counter = 0;
  uint8_t  convertedvol = VOLUME_CONVERT(Volume);


  counter += CODEC_IO_Write(DeviceAddr, 0x16, convertedvol);

  counter += CODEC_IO_Write(DeviceAddr, 0x17, convertedvol);

  return counter;
}

/**
  * @brief Set new frequency.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param AudioFreq: Audio frequency used to play the audio stream.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_SetFrequency(uint16_t DeviceAddr, uint32_t AudioFreq)
{
  return 0;
}

/**
  * @brief Enable or disable the mute feature on the audio codec.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param Cmd: AUDIO_MUTE_ON to enable the mute or AUDIO_MUTE_OFF to disable the
  *             mute mode.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_SetMute(uint16_t DeviceAddr, uint32_t Cmd)
{
  uint32_t counter = 0;
  uint8_t  Value;
  

  Value = AUDIO_IO_Read(DeviceAddr, 0x08);


  if(Cmd == AUDIO_MUTE_ON)
  {

    counter += CODEC_IO_Write(DeviceAddr, 0x08, (Value | 0x03));
  }
  else /* AUDIO_MUTE_OFF Disable the Mute */
  {

    counter += CODEC_IO_Write(DeviceAddr, 0x08, (Value & 0xFC));
  }
  return counter;
}

/**
  * @brief Switch dynamically (while audio file is played) the output target 
  *         (speaker, headphone, etc).
  * @note This function is currently not used (only headphone output device).
  * @param DeviceAddr: Device address on communication Bus.
  * @param Output: specifies the audio output device target. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_SetOutputMode(uint16_t DeviceAddr, uint8_t Output)
{
  return 0;
}

/**
  * @brief Reset CS42L51 registers.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs42l51_Reset(uint16_t DeviceAddr)
{
  if(Is_CS42L51_Initialized == 1)
  {

    AUDIO_IO_DeInit();


    AUDIO_IO_Init();
  }
  return 0;
}

/**
  * @}
  */

/** @addtogroup CS42L51_Private_Functions
  * @{
  */ 

/**
  * @brief  Write and optionally read back a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
static uint8_t CODEC_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
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


