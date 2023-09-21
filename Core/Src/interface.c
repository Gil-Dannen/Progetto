#include "interface.h"

static const unsigned int timeout = 10000;
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

void testBSPfunctions()
{
  char Test[100];
  sprintf(Test, "     Temperature = %d", (int)bspGetValue(BSP_temperature));
  sendMessage(Test);
  sprintf(Test, "     Humidity = %d", (int)bspGetValue(BSP_humidity));
  sendMessage(Test);
  sprintf(Test, "     Pressure = %d", (int)bspGetValue(BSP_pressure));

  float * temp;
  temp = bspGetTripleValue(BSPT_accellero);
  sendMessage(Test);
  sprintf(Test, "     Accellerometer = %d,%d,%d", (int)temp[0],(int)temp[1],(int)temp[2]);
  sendMessage(Test);
  temp = bspGetTripleValue(BSPT_magneto);
  sprintf(Test, "     Magneto = %d,%d,%d", (int)temp[0],(int)temp[1],(int)temp[2]);
  sendMessage(Test);

  temp = bspGetTripleValue(BSPT_gyro);
  sprintf(Test, "     Gyro = %d,%d,%d", (int)temp[0],(int)temp[1],(int)temp[2]);
  sendMessage(Test);
}

void beforeLoop(uint8_t deltaMs)
{
	timer+=deltaMs;

}

void loop(uint8_t deltaMs)
{
  if(timer >= timeout){
	  testBSPfunctions();
	  timer = 0;
  }

	uint8_t button = readDigital(MF_Button);
	setDigital(MF_led1,!button);
}

void afterLoop(uint8_t deltaMs)
{

}




