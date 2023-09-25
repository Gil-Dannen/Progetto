#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
    uint8_t *uid;
    uint8_t charHandle[2];
    char *value;
    float optionalValue;
    uint8_t enabled;
} BLE_Object;

#define READABLE 0x02
#define NOTIFIBLE 0x10
#define WRITABLE 0x04

uint8_t CUSTOM_SERVICE_HANDLE[2];

void resetBleModule();

void ble_init();

int fetchBleEvent(uint8_t *container, int size);

// check if the event that was fetched is what I expected
int checkEventResp(uint8_t *event, uint8_t *reference, int size);

void sendCommand(uint8_t *command, int size);

void catchBLE();

void setConnectable();

int BLE_command(uint8_t *command, int size, uint8_t *result, int sizeRes, int returnHandles);

void addService(uint8_t *UUID, uint8_t *handle, int attributes);

void addCharacteristic(uint8_t *UUID, uint8_t *handleChar, uint8_t *handleService, uint8_t maxsize, uint8_t proprieties);

void updateCharValue(uint8_t *handleService, uint8_t *handleChar, int offset, int size, uint8_t *data);

//-+999.9 as value
void updateSignedFloat(uint8_t *service, uint8_t *characteristic, uint8_t *defaultValue, uint8_t offset, float data);

#endif