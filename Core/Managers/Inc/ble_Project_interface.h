/*
 * ble_Project_interface.h
 *
 *  Created on: 19 Oct 2023
 *      Author: Mattia Sacchi
 */

#ifndef MANAGERS_INC_BLE_PROJECT_INTERFACE_H_
#define MANAGERS_INC_BLE_PROJECT_INTERFACE_H_

#include <stdint.h>

typedef enum
{
	BS_INVALID = 0,
	BS_ENVIRONMENT,
	BS_INTERTIAL,
	BS_MAGNETIC,
	BS_GYROSCOPE,
	BS_COUNT

}BleServices;

typedef enum
{
	BM_Temperature,
	BM_Humidity,
	BM_Pressure,
	BM_Magneto_x,
	BM_Magneto_y,
	BM_Magneto_z,
	BM_Accelero_x,
	BM_Accelero_y,
	BM_Accelero_z,
	BM_Gyroscope_x,
	BM_Gyroscope_y,
	BM_Gyroscope_z,

	BM_COUNT,

}BleMessages;

typedef struct
{
	uint8_t * uuid;
	uint8_t handle[2];
	uint8_t charateristicHandle[2];
	uint8_t * charateristicId;
	uint8_t messageCount;

}BleService;

typedef struct
{
	uint8_t * uuid;
	uint8_t handle[2];
	char * defaultMessage;
	BleServices service;

}BleMessage;


void setupNewService(BleServices service, int attributes, char * name);

void setupNewMessage(BleMessages msg,BleServices service, char * name);

void updateMessage(BleMessages msg, float data);

void ble_setup();

#endif /* MANAGERS_INC_BLE_PROJECT_INTERFACE_H_ */
