#include "startup_state.h"
#include <stdio.h>
#include "ble_interface.h"
#include "ble_manager.h"
#include "time_manager.h"
#include "state_machine.h"

void startup_enter()
{
    setMappedFunction(MF_Button, GPIOC, GPIO_PIN_13, 0, 1);
    setMappedFunction(MF_led1, GPIOA, GPIO_PIN_5, 0, 1);
    setMappedFunction(MF_led2, GPIOB, GPIO_PIN_14, 0, 1);
    setMappedFunction(MF_BleInt, GPIOE, GPIO_PIN_6, 0, 1);
    setMappedFunction(MF_BleCS, GPIOD, GPIO_PIN_13, 0, 1);
    setMappedFunction(MF_BleReset, GPIOA, GPIO_PIN_8, 0, 1);

    ble_init();

    sleep(10);

    bleProjectSetup();

    initTimers();

    uart_init();

    bspFunctionInit();

    resetTimer(TF_Main);

    setDigital(MF_led2, GPIO_PIN_RESET);
}


void startup_beforeLoop(uint8_t deltaMs)
{

}

void startup_loop(uint8_t deltaMs)
{
    if(isTimerExpired(TF_Main,1000))
        setState(ST_IDLE);
}

void startup_afterLoop(uint8_t deltaMs)
{

}


