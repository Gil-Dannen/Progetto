#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

#include "ble_manager.h"

typedef enum BleMessage
{
    BM_HEADER = 0,
    BLE_COUNT,
} BM;

void bleProjectSetup();

void bleLoop(uint8_t);

void initObject(BM bleMessage,uint8_t * uid,uint8_t handle[2], char * message);

BLE_Object objects[BLE_COUNT];

#endif