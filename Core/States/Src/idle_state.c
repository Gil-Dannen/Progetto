#include <ble_project_interface.h>
#include "idle_state.h"
#include "state_machine.h"
#include "general_functions.h"
#include "enable.h"
#include <stdlib.h>


static float temperature = 0, humidity = 0, pressure = 0,
		accelerometer[3], magnetometer[3] , gyroscope[3];

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
    setTimer(TF_SerialSend, testBSPfunctions, 3000);

    setDigital(MF_led2, GPIO_PIN_RESET);

    setDigital(MF_led1,GPIO_PIN_SET);

    setTimer(TF_Blink, blink, 500);

}


void idle_beforeLoop(uint8_t deltaMs)
{
	temperature = bspGetValue(BSP_temperature);
	humidity = bspGetValue(BSP_humidity);
	pressure = bspGetValue(BSP_pressure);
	float * temp = bspGetTripleValue(BSPT_gyro);
	memcpy(gyroscope,temp,sizeof(float)*3);
	temp = bspGetTripleValue(BSPT_magneto);
	memcpy(magnetometer,temp,sizeof(float)*3);
	temp = bspGetTripleValue(BSPT_accellero);
	memcpy(accelerometer,temp,sizeof(float)*3);
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


		  updateMessage(BM_Accelero_x, accelerometer[0]);
		  updateMessage(BM_Accelero_y, accelerometer[1]);
		  updateMessage(BM_Accelero_z, accelerometer[2]);

		  updateMessage(BM_Magneto_x, magnetometer[0]);
		  updateMessage(BM_Magneto_y, magnetometer[1]);
		  updateMessage(BM_Magneto_z, magnetometer[2]);


		  updateMessage(BM_Gyroscope_x, gyroscope[0]);
		  updateMessage(BM_Gyroscope_y, gyroscope[1]);
		  updateMessage(BM_Gyroscope_z, gyroscope[2]);

	  }
}

void idle_afterLoop(uint8_t deltaMs)
{
}


