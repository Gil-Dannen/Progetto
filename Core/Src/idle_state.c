#include "idle_state.h"
#include "state_machine.h"
#include "general_functions.h"


static float temperature = 0, humidity = 0, pressure = 0,
		*acceleremeter = NULL, *magnetometer = NULL , *gyroscope = NULL;


void testBSPfunctions()
{
    char Test[50];
    sprintf(Test, "Temperature = %d\n\r", (int)temperature);
    appendMessage(Test);
    sprintf(Test, "Humidity = %d\n\r", (int)humidity);
    appendMessage(Test);
    sprintf(Test, "Pressure = %d\n\r", (int)pressure);
    appendMessage(Test);
    sprintf(Test, "Accellerometer = %d,%d,%d\n\r",
    	(int)acceleremeter[0], (int)acceleremeter[1], (int)acceleremeter[2]);
    appendMessage(Test);

    sprintf(Test, "Magneto = %d,%d,%d\n\r",
    	(int)magnetometer[0], (int)magnetometer[1], (int)magnetometer[2]);
    appendMessage(Test);

    sprintf(Test, "Gyro = %d,%d,%d\n\r",
    	(int)gyroscope[0], (int)gyroscope[1], (int)gyroscope[2]);
    appendMessage(Test);

    sendMessage();

}



void idle_enter()
{
    setTimer(TF_Main, testBSPfunctions, 3000);

    setDigital(MF_led2, GPIO_PIN_RESET);

    setDigital(MF_led1,GPIO_PIN_SET);

    setExitCondition(ST_BLE_CHECK, buttonToggled);
}


void idle_beforeLoop(uint8_t deltaMs)
{
	temperature = bspGetValue(BSP_temperature);
	humidity = bspGetValue(BSP_humidity);
	pressure = bspGetValue(BSP_pressure);
	magnetometer = bspGetTripleValue(BSPT_magneto);
	acceleremeter = bspGetTripleValue(BSPT_accellero);
	gyroscope = bspGetTripleValue(BSPT_gyro);
}


void idle_loop(uint8_t deltaMs)
{

}

void idle_afterLoop(uint8_t deltaMs)
{
}


