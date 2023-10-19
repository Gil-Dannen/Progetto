#include "ble_Project_interface.h"
#include "enable.h"
#include <stdio.h>
#include <string.h>

BleService Service[BS_COUNT];
BleMessage Messages[BM_COUNT];

char MessageString[128];

uint8_t UUID_SERVICE_1[]={0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};//Reversed UUID of the service

uint8_t UUID_CHAR_1[]={0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
char VALUE1[]="Name: Environment";

uint8_t UUID_CHAR_TEMP[]={0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
char VALUE_TEMP[] = "Temperature: ";

uint8_t UUID_CHAR_HUM[]={0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
char VALUE_HUM[] = "Humidity: ";

uint8_t UUID_CHAR_PRESS[]={0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
char VALUE_PRESS[] = "Pressure: ";

void setupNewService(BleServices service,uint8_t * uuid, int attributes,uint8_t * charateristicId, char * name)
{
	Service[service].uuid = uuid;
	Service[service].charateristicId = charateristicId;

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

void setupNewMessage(BleMessages msg,BleServices service,uint8_t * uuid, char * name)
{
	Messages[msg].uuid = uuid;
	Messages[msg].service = service;
	Messages[msg].defaultMessage = name;


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




void ble_setup()
{
	setupNewService(BS_ENVIRONMENT,UUID_SERVICE_1,SET_ATTRIBUTES(1+2+3*2+3+3),UUID_CHAR_1,VALUE1);

	setupNewMessage(BM_Temperature, BS_ENVIRONMENT, UUID_CHAR_TEMP, VALUE_TEMP);

	setupNewMessage(BM_Humidity, BS_ENVIRONMENT, UUID_CHAR_HUM, VALUE_HUM);

	setupNewMessage(BM_Pressure, BS_ENVIRONMENT, UUID_CHAR_PRESS, VALUE_PRESS);
}
