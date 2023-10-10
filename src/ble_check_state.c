#include "ble_check_state.h"
#include "ble_interface.h"
#include "ble_manager.h"
#include "state_machine.h"
#include "general_functions.h"

int dataAvailable = 0;
int update = 0;

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

    if (readDigital(MF_BleInt))
	{
		catchBLE();
		return;
	}
    else if (update)
    {
        update = 0;
        updateSignedFloat(CUSTOM_SERVICE_HANDLE,TEMP_CHAR_HANDLE,VALUE_TEMP,9,bspGetValue(BSP_temperature) *10);
		updateSignedFloat(CUSTOM_SERVICE_HANDLE,HUM_CHAR_HANDLE,VALUE_HUM,8,bspGetValue(BSP_humidity) *10);
		//updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCX_CHAR_HANDLE,X_VALUE,10,accx);
		//updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCY_CHAR_HANDLE,Y_VALUE,10,accy);
		//updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCZ_CHAR_HANDLE,Z_VALUE,10,accz);
	    updateSignedFloat(CUSTOM_SERVICE_HANDLE,PRESS_CHAR_HANDLE,VALUE_PRESS,10,bspGetValue(BSP_pressure) *10);
	}
}

void ble_check_afterLoop(uint8_t deltaMs)
{
    __WFI();
}


