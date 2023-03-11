/*
 * interface.c
 *
 *  Created on: Mar 2, 2023
 *      Author: david
 */
#include "interface.h"

UART_HandleTypeDef huart2;

static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static const unsigned int timeout = 1000;
static unsigned int timer = 0;

void setup()
{

	MX_USART2_UART_Init();
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

	bspGetValue(BSP_humidity);

	bspGetValue(BSP_pressure);

	bspGetValue(BSP_temperature);

	uint8_t Test[] = "Hello World !!!\r\n"; //Data to send
	if(timer >= timeout){
		HAL_UART_Transmit(&huart2,Test,sizeof(Test),10);// Sending in normal mode
		timer = 0;
	}
	uint8_t button = readDigital(MF_Button);
	setDigital(MF_led1,!button);
	HAL_Delay(20);
}

void afterLoop(uint8_t deltaMs)
{

}




