#include "idle_state.h"
#include "state_machine.h"
#include "general_functions.h"
#include "ble_Project_interface.h"


static float temperature = 0, humidity = 0, pressure = 0,
		*accelerometer = NULL, *magnetometer = NULL , *gyroscope = NULL;

extern uint8_t INERTIAL_SERVICE_HANDLE[2];
extern uint8_t ACCX_CHAR_HANDLE[2];
extern uint8_t ACCY_CHAR_HANDLE[2];
extern uint8_t ACCZ_CHAR_HANDLE[2];

extern uint8_t MAGNETIC_SERVICE_HANDLE[2];

extern uint8_t MAGX_CHAR_HANDLE[2];
extern uint8_t MAGY_CHAR_HANDLE[2];
extern uint8_t MAGZ_CHAR_HANDLE[2];

extern uint8_t X_VALUE[];
extern uint8_t Y_VALUE[];
extern uint8_t Z_VALUE[];

int update = 0;
int dataAvailable = 0;



uint8_t blinkStatus = 0;

void blink()
{
	blinkStatus = !blinkStatus;
	setDigital(MF_led1,blinkStatus);
}

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
    	(int)accelerometer[0], (int)accelerometer[1], (int)accelerometer[2]);
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

    setTimer(TF_Main, blink, 500);

}


void idle_beforeLoop(uint8_t deltaMs)
{
	temperature = bspGetValue(BSP_temperature);
	humidity = bspGetValue(BSP_humidity);
	pressure = bspGetValue(BSP_pressure);
	magnetometer = bspGetTripleValue(BSPT_magneto);
	accelerometer = bspGetTripleValue(BSPT_accellero);
	gyroscope = bspGetTripleValue(BSPT_gyro);
}


void idle_loop(uint8_t deltaMs)
{

	  if(readDigital(MF_BleInt)){//if an event occurs let's catch it
		  catchBLE();
		  return;

	  }

	  if(update){
		  update=0;

		  updateMessage(BM_Temperature,temperature);
		  updateMessage(BM_Humidity,humidity);
		  updateMessage(BM_Pressure,pressure);

		  updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCX_CHAR_HANDLE,X_VALUE,10,accelerometer[0]);
		  updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCY_CHAR_HANDLE,Y_VALUE,10,accelerometer[1]);
		  updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCZ_CHAR_HANDLE,Z_VALUE,10,accelerometer[2]);



		  updateSignedMillesimal(MAGNETIC_SERVICE_HANDLE,MAGX_CHAR_HANDLE,X_VALUE,10,magnetometer[0]);
		  updateSignedMillesimal(MAGNETIC_SERVICE_HANDLE,MAGY_CHAR_HANDLE,Y_VALUE,10,magnetometer[1]);
		  updateSignedMillesimal(MAGNETIC_SERVICE_HANDLE,MAGZ_CHAR_HANDLE,Z_VALUE,10,magnetometer[2]);

	  }
}

void idle_afterLoop(uint8_t deltaMs)
{
}


