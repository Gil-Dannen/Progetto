#include "idle_state.h"
#include "state_machine.h"

int dataAvailable = 0;
int update = 0;

void testBSPfunctions()
{
    char Test[100];
    sprintf(Test, "     Temperature = %d", (int)bspGetValue(BSP_temperature));
    // set press

    sendMessage(Test);
    sprintf(Test, "     Humidity = %d", (int)bspGetValue(BSP_humidity));
    sendMessage(Test);
    sprintf(Test, "     Pressure = %d", (int)bspGetValue(BSP_pressure));

    float *temp;
    temp = bspGetTripleValue(BSPT_accellero);
    sendMessage(Test);
    sprintf(Test, "     Accellerometer = %d,%d,%d", (int)temp[0], (int)temp[1], (int)temp[2]);
    sendMessage(Test);
    temp = bspGetTripleValue(BSPT_magneto);
    sprintf(Test, "     Magneto = %d,%d,%d", (int)temp[0], (int)temp[1], (int)temp[2]);
    sendMessage(Test);

    temp = bspGetTripleValue(BSPT_gyro);
    sprintf(Test, "     Gyro = %d,%d,%d", (int)temp[0], (int)temp[1], (int)temp[2]);
    sendMessage(Test);
}

void idle_enter()
{
    setTimer(TF_Main, testBSPfunctions, 3000);

    setDigital(MF_led2, GPIO_PIN_RESET);
}

uint8_t button = 0;

void idle_beforeLoop(uint8_t deltaMs)
{
}

void idle_loop(uint8_t deltaMs)
{
    if (readDigital(MF_Button) && !button)
    {
        setState(ST_BLE_CHECK);
    }
}

void idle_afterLoop(uint8_t deltaMs)
{
}


