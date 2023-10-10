#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

#include "ble_manager.h"

extern uint8_t UUID_CHAR_1[];
extern uint8_t CUSTOM_CHAR_HANDLE[2];
extern uint8_t VALUE1[];

extern uint8_t UUID_CHAR_2[];
extern uint8_t MIC_CHAR_HANDLE[2];
extern uint8_t VALUE2[];

extern uint8_t UUID_CHAR_TEMP[];
extern uint8_t TEMP_CHAR_HANDLE[2];
extern uint8_t VALUE_TEMP[];

// uno meno di temp in lunghezza
extern uint8_t UUID_CHAR_HUM[];
extern uint8_t HUM_CHAR_HANDLE[2];
extern uint8_t VALUE_HUM[];

// uno meno di temp in lunghezza
extern uint8_t UUID_CHAR_PRESS[];
extern uint8_t PRESS_CHAR_HANDLE[2];
extern uint8_t VALUE_PRESS[];

extern uint8_t UUID_INERTIAL_SERVICE[];
extern uint8_t INERTIAL_SERVICE_HANDLE[2];

extern uint8_t UUID_CHAR_INERTIAL_NAME[];
extern uint8_t NAME_INERTIAL_HANDLE[2];
extern uint8_t NAME_INERTIAL_VALUE[];

extern uint8_t UUID_CHAR_INERTIAL_ACCX[];
extern uint8_t ACCX_CHAR_HANDLE[2];
extern uint8_t ACCX_INERTIAL_VALUE[];

extern uint8_t UUID_CHAR_INERTIAL_ACCY[];
extern uint8_t ACCY_CHAR_HANDLE[2];
extern uint8_t ACCY_INERTIAL_VALUE[];

extern uint8_t UUID_CHAR_INERTIAL_ACCZ[];
extern uint8_t ACCZ_CHAR_HANDLE[2];
extern uint8_t ACCZ_INERTIAL_VALUE[];

extern uint8_t X_VALUE[];
extern uint8_t Y_VALUE[];
extern uint8_t Z_VALUE[];
extern uint8_t MIC_VALUE[];

extern uint8_t UUID_MAGNETIC_SERVICE[];
extern uint8_t MAGNETIC_SERVICE_HANDLE[2];

extern uint8_t UUID_CHAR_MAGNETIC_NAME[];
extern uint8_t NAME_MAGNETIC_HANDLE[2];
extern uint8_t NAME_MAGNETIC_VALUE[];

extern uint8_t UUID_CHAR_MAGNETIC_MAGX[];
extern uint8_t MAGX_CHAR_HANDLE[2];

extern uint8_t UUID_CHAR_MAGNETIC_MAGY[];
extern uint8_t MAGY_CHAR_HANDLE[2];

extern uint8_t UUID_CHAR_MAGNETIC_MAGZ[];
extern uint8_t MAGZ_CHAR_HANDLE[2];

extern uint8_t UUID_GYROSCOPE_SERVICE[];
extern uint8_t GYROSCOPE_SERVICE_HANDLE[2];

extern uint8_t UUID_CHAR_GYROSCOPE_NAME[];
extern uint8_t NAME_GYROSCOPE_HANDLE[2];
extern uint8_t NAME_GYROSCOPE_VALUE[];

extern uint8_t UUID_CHAR_GYROSCOPE_GYROX[];
extern uint8_t GYROX_CHAR_HANDLE[2];

extern uint8_t UUID_CHAR_GYROSCOPE_GYROY[];
extern uint8_t GYROY_CHAR_HANDLE[2];

extern uint8_t UUID_CHAR_GYROSCOPE_GYROZ[];
extern uint8_t GYROZ_CHAR_HANDLE[2];

typedef enum BleMessage
{
    BM_HEADER = 0,
    BLE_COUNT,
} BM;

void bleProjectSetup();

void bleLoop(uint8_t);

void initObject(BM bleMessage, uint8_t *uid, uint8_t handle[2], char *message);

// BLE_Object objects[BLE_COUNT];

#endif
