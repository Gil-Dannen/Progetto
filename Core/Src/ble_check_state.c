#include "ble_check_state.h"
#include "state_machine.h"
#include "general_functions.h"

#include "enable.h"
#include "sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t TOF_CHAR_HANDLE[2];
extern uint8_t CUSTOM_SERVICE_HANDLE[2];
extern uint8_t TOF_VALUE[];
extern uint8_t TEMP_CHAR_HANDLE[];
extern uint8_t HUM_CHAR_HANDLE[];
extern uint8_t VALUE_TEMP[];
extern uint8_t VALUE_HUM[];

extern uint8_t PRESS_CHAR_HANDLE[2];
extern uint8_t VALUE_PRESS[];


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

//int dataAvailable=0;
//int update=0;


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




	  int distanceComplete=0;


	  float press;
	  float hum;
	  float temp;
	  int16_t accx,accy,accz;
	  int16_t magx,magy,magz;

	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
	  if(HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){//if an event occurs let's catch it
		  catchBLE();

	  }else{

		  if(/*update*/ 0){
			  //update=0;

			  getDistance(&distanceComplete);
			  updateSignedMillesimal(CUSTOM_SERVICE_HANDLE,TOF_CHAR_HANDLE,TOF_VALUE,13,distanceComplete);
			  HAL_Delay(10);
			  getTemperature(&temp);
			  updateSignedFloat(CUSTOM_SERVICE_HANDLE,TEMP_CHAR_HANDLE,VALUE_TEMP,9,temp);
			  HAL_Delay(10);

			  getHumidity(&hum);
			  updateSignedFloat(CUSTOM_SERVICE_HANDLE,HUM_CHAR_HANDLE,VALUE_HUM,8,hum);

			  HAL_Delay(10);
			  getAxisAccelerometer(&accx,&accy,&accz);
			  updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCX_CHAR_HANDLE,X_VALUE,10,accx);
			  updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCY_CHAR_HANDLE,Y_VALUE,10,accy);
			  updateSignedMillesimal(INERTIAL_SERVICE_HANDLE,ACCZ_CHAR_HANDLE,Z_VALUE,10,accz);

			  getPressure(&press);
			  updateSignedFloat(CUSTOM_SERVICE_HANDLE,PRESS_CHAR_HANDLE,VALUE_PRESS,10,press);

			  HAL_Delay(10);
			  getAxisMagnetometer(&magx,&magy,&magz);
			  updateSignedMillesimal(MAGNETIC_SERVICE_HANDLE,MAGX_CHAR_HANDLE,X_VALUE,10,magx);
			  updateSignedMillesimal(MAGNETIC_SERVICE_HANDLE,MAGY_CHAR_HANDLE,Y_VALUE,10,magy);
			  updateSignedMillesimal(MAGNETIC_SERVICE_HANDLE,MAGZ_CHAR_HANDLE,Z_VALUE,10,magz);


			  startToF();
			  int pwm=0;
			  pwm=distanceComplete;
			  if(distanceComplete>500){
				  pwm=500;
			  }
			  //__HAL_TIM_SetCompare(&htim15,TIM_CHANNEL_1,pwm);
			   }



	  }




}

void ble_check_afterLoop(uint8_t deltaMs)
{
    __WFI();
}


