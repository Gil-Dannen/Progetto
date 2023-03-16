/*
 * interface.c
 *
 *  Created on: Mar 2, 2023
 *      Author: david
 */
#include "interface.h"

static const unsigned int timeout = 1000;
static unsigned int timer = 0;

void setup()
{
	uart_init();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
 	bspFunctionInit();
 	timer = 0;
 	setMappedFunction(MF_Button, GPIOC, GPIO_PIN_13,0,1);
 	setMappedFunction(MF_led1,GPIOA, GPIO_PIN_5,0,1);
}

void beforeLoop(uint8_t deltaMs)
{
	timer+=deltaMs;

}

void loop(uint8_t deltaMs)
{
  char Test[100];
  if(timer >= timeout){
	  sprintf(Test, "     Temperature = %d", (int)bspGetValue(BSP_temperature));
	  sendMessage(Test);
	  sprintf(Test, "     Humidity = %d", (int)bspGetValue(BSP_humidity));
	  sendMessage(Test);
	  sprintf(Test, "     Pressure = %d", (int)bspGetValue(BSP_pressure));
	  sendMessage(Test);
	  sprintf(Test, "     Accellerometer = %d", (int)bspGetValue(BSP_accellero));
	  sendMessage(Test);
	  sprintf(Test, "     Magneto = %d", (int)bspGetValue(BSP_magneto));
	  sendMessage(Test);
	  float gyro;
	  BSP_GYRO_GetXYZ(&gyro);
	  sprintf(Test, "     Gyro = %d", (int)gyro);
	  //sprintf(Test, "     Gyro = %d", (int)bspGetValue(BSP_gyro));
	  sendMessage(Test);

	  timer = 0;
  }

	uint8_t button = readDigital(MF_Button);
	setDigital(MF_led1,!button);
}

void afterLoop(uint8_t deltaMs)
{

}




