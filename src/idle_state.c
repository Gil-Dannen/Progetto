#include "startup_state.h"
#include <stdio.h>
#include "ble_interface.h"
#include "ble_manager.h"
#include "time_manager.h"
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

static uint8_t UUID_CHAR_TEMP[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
static uint8_t TEMP_CHAR_HANDLE[2];
static uint8_t VALUE_TEMP[] = {'{', '\"', 'T', 'e', 'm', 'p', '"', ':', '\"', '+', '0', '0', '0', '.', '0', '\"', '}'};

uint8_t button = 0;

void idle_beforeLoop(uint8_t deltaMs)
{
    if (readDigital(MF_BleInt))
    { // if an event occurs let's catch it
        catchBLE();
        return;
    }
}

void idle_loop(uint8_t deltaMs)
{
    if (readDigital(MF_Button) && !button)
    {
        testBSPfunctions();
    }

    if (update)
    {
        update = 0;
        updateCharValue(CUSTOM_SERVICE_HANDLE, TEMP_CHAR_HANDLE, 0, (17), sprintf("%d", (int)bspGetValue(BSP_temperature) * 10));
    }

    button = readDigital(MF_Button);
    setDigital(MF_led1, !button);
}

void idle_afterLoop(uint8_t deltaMs)
{
    __WFI();
}


