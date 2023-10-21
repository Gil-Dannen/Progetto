#include <ble_project_interface.h>
#include "enable.h"
#include <stdio.h>
#include <string.h>

BleService Service[BS_COUNT];
BleMessage Messages[BM_COUNT];

char MessageString[64];

uint8_t uuidServiceDefault[]={0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t uuidServiceDefaultCharateristic[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x0FF,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};

void setupNewService(BleServices service,int attributes, char * name)
{
	uint8_t * tempServiceID = uuidServiceDefault, *tempCharateristicID = uuidServiceDefaultCharateristic;
	tempServiceID[6] = service;
	tempCharateristicID[6] = service;
	Service[service].uuid = tempServiceID;
	Service[service].charateristicId = tempCharateristicID;
	Service[service].messageCount = 1;

	addService(Service[service].uuid,
			Service[service].handle,
		attributes);

	size_t size = strlen(name);

	addCharacteristic(Service[service].charateristicId ,
			Service[service].charateristicHandle,Service[service].handle,size,READABLE);

	uint8_t value[size];

	memcpy(value,name,size);

	updateCharValue(Service[service].handle,Service[service].charateristicHandle,0,size,value);
}

void setupNewMessage(BleMessages msg,BleServices service, char * name)
{
	uint8_t *tempCharateristicID = uuidServiceDefaultCharateristic;
	tempCharateristicID[6] = Messages[msg].service;
	tempCharateristicID[0] = Service[service].messageCount;
	Messages[msg].uuid = tempCharateristicID;
	Messages[msg].service = service;
	Messages[msg].defaultMessage = name;
	Service[service].messageCount++;


	size_t size = strlen(name)+8;
	uint8_t value[size];

	memcpy(value,name,size);
	uint8_t spacer[8];
	memset(spacer,0,8);
	memcpy(value+size,spacer,8);

	addCharacteristic(Messages[msg].uuid,Messages[msg].handle,Service[service].handle,size,READABLE|NOTIFIBLE);

	updateCharValue(Service[service].handle,Messages[msg].handle,0,size,value);
}


void updateMessage(BleMessages msg, float data)
{

	char sign = data<0 ? '-': '+';
	char temp[10];
	uint8_t decimalPoint = (data-(int)data)*10;
	sprintf(temp,"%c%d,%d\0",sign,(uint32_t)data,decimalPoint);

	strcat(MessageString,Messages[msg].defaultMessage);
	strcat(MessageString,temp);

	size_t size = strlen(MessageString);
	uint8_t value[size];
	memcpy(value,MessageString,size);
	memset(MessageString,0,size);

	updateCharValue(Service[Messages[msg].service].handle, Messages[msg].handle, 0, size, value);

}



char valueEnvirontment[]="Name: Environment";
char valueTemperature[] = "Temperature: ";
char valueHumidity[] = "Humidity: ";
char valuePressure[] = "Pressure: ";
char axisX[] = "Axis X: ";
char axisY[] = "Axis Y: ";
char axisZ[] = "Axis Z: ";
char valueInertial[]= "Name: Inertia";
char valueMagnetic[]= "Name: magnetic";
char valueGyroscope[]= "Name: gyroscope";

void ble_setup()
{
	setupNewService(BS_ENVIRONMENT,SET_ATTRIBUTES(1+2+3*2+3+3),valueEnvirontment);

	setupNewMessage(BM_Temperature, BS_ENVIRONMENT, valueTemperature);

	setupNewMessage(BM_Humidity, BS_ENVIRONMENT, valueHumidity);

	setupNewMessage(BM_Pressure, BS_ENVIRONMENT, valuePressure);

	setupNewService(BS_INTERTIAL,SET_ATTRIBUTES(1+2+3*2+3+3),valueInertial);

	setupNewMessage(BM_Accelero_x, BS_INTERTIAL, axisX);

	setupNewMessage(BM_Accelero_y, BS_INTERTIAL, axisY);

	setupNewMessage(BM_Accelero_z, BS_INTERTIAL, axisZ);

	setupNewService(BS_MAGNETIC,SET_ATTRIBUTES(1+2+3*3),valueMagnetic);

	setupNewMessage(BM_Magneto_x, BS_MAGNETIC, axisX);

	setupNewMessage(BM_Magneto_y, BS_MAGNETIC, axisY);

	setupNewMessage(BM_Magneto_z, BS_MAGNETIC, axisZ);

	setupNewService(BS_GYROSCOPE,SET_ATTRIBUTES(1+2+3*3),valueGyroscope);

	setupNewMessage(BM_Gyroscope_x, BS_GYROSCOPE, axisX);

	setupNewMessage(BM_Gyroscope_y, BS_GYROSCOPE, axisY);

	setupNewMessage(BM_Gyroscope_z, BS_GYROSCOPE, axisZ);

}
