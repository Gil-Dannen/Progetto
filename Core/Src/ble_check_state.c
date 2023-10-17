#include "ble_check_state.h"
#include "state_machine.h"
#include "general_functions.h"


uint8_t blinkStatus = 0;

void blink()
{
	blinkStatus = !blinkStatus;
	setDigital(MF_led1,blinkStatus);
}

void ble_check_enter()
{

	setDigital(MF_led1,GPIO_PIN_SET);

    setTimer(TF_Main, blink, 500);

    setDigital(MF_led2, GPIO_PIN_RESET);

    setExitCondition(ST_IDLE, buttonToggled);

}


void ble_check_beforeLoop(uint8_t deltaMs)
{

}

void ble_check_loop(uint8_t deltaMs)
{

}

void ble_check_afterLoop(uint8_t deltaMs)
{
    __WFI();
}


