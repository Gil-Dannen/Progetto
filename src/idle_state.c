#include "idle_state.h"
#include "state_machine.h"
#include "general_functions.h"


static float temperature = 0, humidity = 0, pressure = 0,
		*acceleremeter, *magnetometer , *gyroscope;

void testBSPfunctions()
{
    char Test[100];
    sprintf(Test, "Temperature = %d\n", (int)temperature);
    sendMessage(Test);
    sprintf(Test, "Humidity = %d\n", (int)humidity);
    sendMessage(Test);
    sprintf(Test, "Pressure = %d\n", (int)pressure);

    sendMessage(Test);
    sprintf(Test, "Accellerometer = %d,%d,%d\n",
    	(int)acceleremeter[0], (int)acceleremeter[1], (int)acceleremeter[2]);
    sendMessage(Test);

    sprintf(Test, "Magneto = %d,%d,%d\n",
    	(int)magnetometer[0], (int)magnetometer[1], (int)magnetometer[2]);
    sendMessage(Test);

    sprintf(Test, "Gyro = %d,%d,%d\n",
    	(int)gyroscope[0], (int)gyroscope[1], (int)gyroscope[2]);
    sendMessage(Test);
}



void idle_enter()
{
    setTimer(TF_Main, testBSPfunctions, 3000);

    setDigital(MF_led2, GPIO_PIN_RESET);

    setDigital(MF_led1,GPIO_PIN_RESET);

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


