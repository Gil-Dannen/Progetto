#include <ble_project_interface.h>
#include "enable.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include "ble_commands.h"
#include "time_manager.h"


extern SPI_HandleTypeDef hspi3;
extern int dataAvailable;
uint8_t deviceName[]={'P','r','o','g','e','t','t','o','L','i','g','m','a'};

uint8_t buffer[255];

uint16_t stackInitCompleteFlag=0;
uint8_t* rxEvent;

void ble_init(){



	//fetching the reset event
	rxEvent=(uint8_t*)malloc(EVENT_STARTUP_SIZE);
	int res;

	while(!dataAvailable);
		res=fetchBleEvent(rxEvent,EVENT_STARTUP_SIZE);

	if(res==BLE_OK){
		res=checkEventResp(rxEvent,EVENT_STATUP_DATA,EVENT_STARTUP_SIZE);
		if(res==BLE_OK)
		   stackInitCompleteFlag|=0x01;

	}

	sleep(10);
	free(rxEvent);

	//INIT GATT


	if(BLE_command(ACI_GATT_INIT,sizeof(ACI_GATT_INIT),ACI_GATT_INIT_COMPLETE,sizeof(ACI_GATT_INIT_COMPLETE),0)==BLE_OK)
	   stackInitCompleteFlag|=0x02;
	free(rxEvent);


	//INIT GAP, actually the handle that i get is a GATT handle of a service, will change the name later

	if(BLE_command(ACI_GAP_INIT,sizeof(ACI_GAP_INIT),ACI_GAP_INIT_COMPLETE,sizeof(ACI_GAP_INIT_COMPLETE),3)==BLE_OK){
	   stackInitCompleteFlag|=0x04;
	   memcpy(GAP_SERVICE_HANDLE,rxEvent+7,2);
	   memcpy(GAP_CHAR_NAME_HANDLE,rxEvent+9,2);
	   memcpy(GAP_CHAR_APP_HANDLE,rxEvent+11,2);
	}
	free(rxEvent);

	//SET THE NAME OF THE BOARD IN THE SERVICE CREATED AUTOMATICALLY
	updateCharValue(GAP_SERVICE_HANDLE,GAP_CHAR_NAME_HANDLE,0,sizeof(deviceName),deviceName);
	stackInitCompleteFlag|=0x08;
	free(rxEvent);

	//INIT AUTH
	if(BLE_command(ACI_GAP_SET_AUTH,sizeof(ACI_GAP_SET_AUTH),ACI_GAP_SET_AUTH_RESP,sizeof(ACI_GAP_SET_AUTH_RESP),0)==BLE_OK){
	   stackInitCompleteFlag|=0x10;
	}
	free(rxEvent);

	//SET_TX_LEVEL

	if(BLE_command(ACI_HAL_SET_TX_POWER_LEVEL,sizeof(ACI_HAL_SET_TX_POWER_LEVEL),ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE,sizeof(ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE),0)==BLE_OK)
	   stackInitCompleteFlag|=0x20;

	free(rxEvent);

	//SET SCAN RESPONSE DATA

	if(BLE_command(HCI_LE_SET_SCAN_RESPONSE_DATA,sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA),HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE,sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE),0)==BLE_OK)
	   stackInitCompleteFlag|=0x40;

	free(rxEvent);

	//This will start the advertisment,
	setConnectable();

	ble_setup();

	return;
}

int fetchBleEvent(uint8_t *container, int size){

  uint8_t master_header[]={0x0b,0x00,0x00,0x00,0x00};
  uint8_t slave_header[5];

  //Wait until it is available an event coming from the BLE module (GPIO PIN COULD CHANGE ACCORDING TO THE BOARD)
  if(HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){

  sleep(5);
  //PIN_CS of SPI2 LOW
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

  //SPI2 in this case, it could change according to the board
  //we send a byte containing a request of reading followed by 4 dummy bytes
  HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
  sleep(1);
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

  HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);

  //let's get the size of data available
  int dataSize;
  dataSize=(slave_header[3]|slave_header[4]<<8);
  int i;
  char dummy=0xff;

  if(dataSize>size){
	  dataSize=size;
  }

  if(dataSize>0){
	    //let's fill the get the bytes availables and insert them into the container variable
  		for(i=0;i<dataSize;i++){
  		HAL_SPI_TransmitReceive(&hspi3,(uint8_t*)&dummy,container+i,1,1);

  		}
  		HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
  	}else{
  		HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
        return -1;
  	}

  //let's stop the SPI2


  dataAvailable=0;
  return BLE_OK;
  }else{
  return -2;
  }
}


int checkEventResp(uint8_t *event, uint8_t *reference, int size){
	int j=0;

	for(j=0;j<size;j++){

		if(event[j]!=reference[j]){
			return -1;
		}
	}

return BLE_OK;
}

//TODO make it not blocking function
void sendCommand(uint8_t *command,int size){

	  uint8_t master_header[]={0x0a,0x00,0x00,0x00,0x00};
	  uint8_t slave_header[5];

	  int result;

	do{


	HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

	//wait until it is possible to write
	//while(!dataAvailable);
	HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);
	int bufferSize=(slave_header[2]<<8|slave_header[1]);
	if(bufferSize>=size){
		HAL_SPI_Transmit(&hspi3,command,size,1);
		result=0;
	}else{
		result=-1;
	}
	//HAL_GPIO_WritePin(CPU_LED_GPIO_Port,CPU_LED_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
	dataAvailable=0;
	}while(result!=0);

}

void catchBLE(){

int result=fetchBleEvent(buffer,127);
	  if(result==BLE_OK){
		  if(checkEventResp(buffer,EVENT_DISCONNECTED,3)==BLE_OK){
			  setConnectable();
		  }


	  }else{
		 // HAL_GPIO_TogglePin(CHG_LED2_GPIO_Port,CHG_LED2_Pin);
		  //something bad is happening if I am here
	  }




}

void setConnectable(){
	   uint8_t* rxEvent;
	   //Start advertising
	   uint8_t *localname;
	   int res;
	   localname=(uint8_t*)malloc(sizeof(deviceName)+5);//carattere di terminazione+listauid+slavetemp
	   memcpy(localname,deviceName,sizeof(deviceName));
	   localname[sizeof(deviceName)+1]=0x00;
	   localname[sizeof(deviceName)+2]=0x00;
	   localname[sizeof(deviceName)+3]=0x00;
	   localname[sizeof(deviceName)+4]=0x00;
	   localname[sizeof(deviceName)]=0x00;


	   ACI_GAP_SET_DISCOVERABLE[11]=sizeof(deviceName)+1;
	   ACI_GAP_SET_DISCOVERABLE[3]=sizeof(deviceName)+5+sizeof(ACI_GAP_SET_DISCOVERABLE)-4;

	   uint8_t *discoverableCommand;
	   discoverableCommand=(uint8_t*)malloc(sizeof(ACI_GAP_SET_DISCOVERABLE)+sizeof(deviceName)+5);
	   memcpy(discoverableCommand,ACI_GAP_SET_DISCOVERABLE,sizeof(ACI_GAP_SET_DISCOVERABLE));
	   memcpy(discoverableCommand+sizeof(ACI_GAP_SET_DISCOVERABLE),localname,sizeof(deviceName)+5);

	   sendCommand(discoverableCommand,sizeof(deviceName)+5+sizeof(ACI_GAP_SET_DISCOVERABLE));
	   rxEvent=(uint8_t*)malloc(7);
	   while(!dataAvailable);
	   res=fetchBleEvent(rxEvent,7);
	   if(res==BLE_OK){
	   res=checkEventResp(rxEvent,ACI_GAP_SET_DISCOVERABLE_COMPLETE,7);
	   if(res==BLE_OK){
		   stackInitCompleteFlag|=0x80;
	   }
	   }

	   free(rxEvent);
	   free(discoverableCommand);
	   free(localname);
	   HAL_Delay(10);
}

int BLE_command(uint8_t* command, int size, uint8_t* result, int sizeRes, int returnHandles){
	   int response;

	   sendCommand(command,size);
	   rxEvent=(uint8_t*)malloc(sizeRes+2*returnHandles);

	   long contatore=0;
	   while(!HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
		   contatore++;
		   if(contatore>30000){
			   break;
		   }
	   }


	   response=fetchBleEvent(rxEvent,sizeRes+returnHandles*2);
	   if(response==BLE_OK){
		   response=checkEventResp(rxEvent,result,sizeRes);
	   }
	   HAL_Delay(10);


	return response;
}

void addService(uint8_t* UUID, uint8_t* handle, int attributes){


	//memcpy
	memcpy(ADD_CUSTOM_SERVICE+5,UUID,16);
    ADD_CUSTOM_SERVICE[22]=attributes;
	   if(BLE_command(ADD_CUSTOM_SERVICE,sizeof(ADD_CUSTOM_SERVICE),ADD_CUSTOM_SERVICE_COMPLETE,sizeof(ADD_CUSTOM_SERVICE_COMPLETE),1)==BLE_OK){
		   handle[0]=rxEvent[7];
		   handle[1]=rxEvent[8];
	    }
	   free(rxEvent);



}

void addCharacteristic(uint8_t* UUID,uint8_t* handleChar, uint8_t* handleService, uint8_t maxsize, uint8_t proprieties){



	memcpy(ADD_CUSTOM_CHAR+7,UUID,16);


	   ADD_CUSTOM_CHAR[4]= handleService[0];
	   ADD_CUSTOM_CHAR[5]= handleService[1];
	   ADD_CUSTOM_CHAR[23]= maxsize;
	   ADD_CUSTOM_CHAR[25]= proprieties;
	   if(BLE_command(ADD_CUSTOM_CHAR,sizeof(ADD_CUSTOM_CHAR),ADD_CUSTOM_CHAR_COMPLETE,sizeof(ADD_CUSTOM_CHAR_COMPLETE),1)==BLE_OK){
		   handleChar[0]=rxEvent[7];
		   handleChar[1]=rxEvent[8];
	    }
	   free(rxEvent);






}

void updateCharValue(uint8_t* handleService,uint8_t* handleChar, int offset, int size,uint8_t* data){

	UPDATE_CHAR[3]=size+6;
	UPDATE_CHAR[4]=handleService[0];
	UPDATE_CHAR[5]=handleService[1];
	UPDATE_CHAR[6]=handleChar[0];
	UPDATE_CHAR[7]=handleChar[1];
	UPDATE_CHAR[8]=offset;
	UPDATE_CHAR[9]=size;

	uint8_t* commandComplete;
	commandComplete=(uint8_t*)malloc(10+size);
	memcpy(commandComplete,UPDATE_CHAR,10);
	memcpy(commandComplete+10,data,size);

	BLE_command(commandComplete,10+size,ADD_CUSTOM_CHAR_COMPLETE,sizeof(ADD_CUSTOM_CHAR_COMPLETE),0);

	free(commandComplete);
	free(rxEvent);

}


//dopo la disconnessione impostiamo riconnectable e poi fine
