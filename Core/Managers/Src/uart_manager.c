#include "uart_manager.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>


extern UART_HandleTypeDef huart1;

static char * txt;

static void reset()
{
	free(txt);
	txt = malloc(sizeof(char) * 1);
}



static void concat(char * a, char * b)
{
	realloc(a,(char) (strlen(a) +strlen(b)));
	strcat(a,b);
}


HAL_StatusTypeDef sendMessage()
{
	return HAL_UART_Transmit(&huart1,(char*)txt,strlen(txt),1000);
	reset();
}

void appendMessage(char * text)
{
	concat(txt,text);
}



void uart_init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  reset();



}
