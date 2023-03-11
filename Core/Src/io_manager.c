/*
 * io_manager.c
 *
 *  Created on: Mar 2, 2023
 *      Author: david
 */

#include "io_manager.h"

float (*bspFunctionArray[BSP_COUNT])(void);

static struct IOS_Man_type ios_manager[MF_COUNT];


void bspFunctionInit()
{
	BSP_HSENSOR_Init();
	bspFunctionArray[BSP_humidity] = &BSP_HSENSOR_ReadHumidity;
	BSP_TSENSOR_Init();
	bspFunctionArray[BSP_temperature] = &BSP_TSENSOR_ReadTemp;
	BSP_PSENSOR_Init();
	bspFunctionArray[BSP_pressure] = &BSP_PSENSOR_ReadPressure;
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
		return 404;
	return HAL_GPIO_ReadPin(ios_manager[mf].m_type, ios_manager[mf].m_pin);
}


float bspGetValue(bspF function)
{
	if(function >= BSP_COUNT)
		return 404;
	return (bspFunctionArray[function])();
}



float readAnalog(MF mf)
{

}
