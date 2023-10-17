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
    setMappedFunction(MF_BleInt, BLE_INT_GPIO_Port, BLE_INT_Pin, 0, 1);
    setMappedFunction(MF_BleCS, BLE_CS_GPIO_Port, BLE_CS_Pin, 0, 1);
    setMappedFunction(MF_BleReset, BLE_RESET_GPIO_Port, BLE_RESET_Pin, 0, 1);
	setMappedFunction(MF_TOF, TOF_RESET_GPIO_Port, TOF_RESET_Pin, 0, 1);


    initTimers();

    uart_init();

    bspFunctionInit();

    setDigital(MF_led2, GPIO_PIN_RESET);

    setStateTimeout(ST_IDLE,200);



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


