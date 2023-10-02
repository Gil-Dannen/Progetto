#include "interface.h"
#include <stdio.h>
#include "ble_interface.h"
#include "ble_manager.h"
#include "time_manager.h"

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

void setup()
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

    

    setTimer(TF_Main, testBSPfunctions, 3000);

    setDigital(MF_led2, GPIO_PIN_RESET);
}

static uint8_t UUID_CHAR_TEMP[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
static uint8_t TEMP_CHAR_HANDLE[2];
static uint8_t VALUE_TEMP[] = {'{', '\"', 'T', 'e', 'm', 'p', '"', ':', '\"', '+', '0', '0', '0', '.', '0', '\"', '}'};

uint8_t button = 0;

void beforeLoop(uint8_t deltaMs)
{
    if (readDigital(MF_BleInt))
    { // if an event occurs let's catch it
        catchBLE();
        return;
    }
}

void loop(uint8_t deltaMs)
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

void afterLoop(uint8_t deltaMs)
{
    __WFI();
}


