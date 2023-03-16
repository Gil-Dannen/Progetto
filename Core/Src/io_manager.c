/*
 * io_manager.c
 *
 *  Created on: Mar 2, 2023
 *      Author: david
 */

#include "io_manager.h"

#include "..\..\Drivers\BSP\B-L475E-IOT01\stm32l475e_iot01_tsensor.h"
#include "..\..\Drivers\BSP\B-L475E-IOT01\stm32l475e_iot01_hsensor.h"
#include "..\..\Drivers\BSP\B-L475E-IOT01\stm32l475e_iot01_psensor.h"

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"

float (*bspFunctionArray[BSP_COUNT])(void);

static struct IOS_Man_type ios_manager[MF_COUNT];

int16_t value = 0;
float place_holder;



void bspFunctionInit()
{
	BSP_HSENSOR_Init();
	bspFunctionArray[BSP_humidity] = &BSP_HSENSOR_ReadHumidity;
	BSP_TSENSOR_Init();
	bspFunctionArray[BSP_temperature] = &BSP_TSENSOR_ReadTemp;
	BSP_PSENSOR_Init();
	bspFunctionArray[BSP_pressure] = &BSP_PSENSOR_ReadPressure;
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	place_holder = 0.f;


}


void setMappedFunction(MF mf,GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,double offset,double factor)
{
	ios_manager[mf].m_type = GPIOx;
	ios_manager[mf].m_pin = GPIO_Pin;
	ios_manager[mf].m_offset = offset;
	ios_manager[mf].m_factor = factor;
}



void setDigital(MF mf,GPIO_PinState state)
{
	HAL_GPIO_WritePin(ios_manager[mf].m_type, ios_manager[mf].m_pin, state);
}

GPIO_PinState readDigital(MF mf)
{

	if(mf >= MF_COUNT)
		return GPIO_PIN_RESET;
	return HAL_GPIO_ReadPin(ios_manager[mf].m_type, ios_manager[mf].m_pin);
}


float bspGetValue(bspF function)
{
	switch(function)
	{
	case BSP_gyro:
		BSP_GYRO_GetXYZ(&place_holder);
		return place_holder;
		break;
	case BSP_magneto:
		BSP_MAGNETO_GetXYZ(&value);
		return (float)value;
		break;
	case BSP_accellero:
		BSP_ACCELERO_AccGetXYZ(&value);
		return (float)value;
		break;
	default:
		//Not handled
		break;
	}
	if(function >= BSP_magneto)
			return 404;
	return (bspFunctionArray[function])();
}



float readAnalog(MF mf)
{
	return 0.f;
}
