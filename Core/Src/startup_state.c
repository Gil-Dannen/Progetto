#include "startup_state.h"
#include "state_machine.h"
#include "enable.h"
#include "sensors.h"
#include "main.h"





void startup_enter()
{
    setMappedFunction(MF_Button, GPIOC, GPIO_PIN_13, 0, 1);
    setMappedFunction(MF_led1, GPIOA, GPIO_PIN_5, 0, 1);
    setMappedFunction(MF_led2, GPIOB, GPIO_PIN_14, 0, 1);
    setMappedFunction(MF_BleInt, GPIOE, GPIO_PIN_6, 0, 1);
    setMappedFunction(MF_BleCS, GPIOD, GPIO_PIN_13, 0, 1);
    setMappedFunction(MF_BleReset, GPIOA, GPIO_PIN_8, 0, 1);
	setMappedFunction(MF_TOF, GPIOC, GPIO_PIN_8, 0, 1);


    initTimers();

    uart_init();

    bspFunctionInit();

    setDigital(MF_led2, GPIO_PIN_RESET);

    setStateTimeout(ST_IDLE,200);

    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

	ble_init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(TOF_RESET_GPIO_Port,TOF_RESET_Pin,GPIO_PIN_SET);

	HAL_Delay(10);

	startToF();
	initLPS22hh();

	HAL_TIM_Base_Start_IT(&htim6);

	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);

	initHTS221();
	init_accelerometer();
	init_magnetometer();

}


void startup_beforeLoop(uint8_t deltaMs)
{
}

void startup_loop(uint8_t deltaMs)
{
}

void startup_afterLoop(uint8_t deltaMs)
{

}


