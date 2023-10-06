#include "ble_interface.h"

uint8_t UUID_CHAR_1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t CUSTOM_CHAR_HANDLE[2];
uint8_t VALUE1[] = {'{', '\"', 'N', 'a', 'm', 'e', '\"', ':', '\"', 'E', 'n', 'v', 'i', 'r', 'o', 'n', 'm', 'e', 'n', 't', '\"', '}'};

uint8_t UUID_CHAR_2[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t MIC_CHAR_HANDLE[2];
uint8_t VALUE2[] = {'{', '\"', 'M', 'i', 'c', 'r', 'o', 'p', 'h', 'o', 'n', 'e', '\"', ':', '\"', '+', '0', '0', '0', '0', '\"', '}'};

uint8_t UUID_CHAR_TEMP[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t TEMP_CHAR_HANDLE[2];
uint8_t VALUE_TEMP[] = {'{', '\"', 'T', 'e', 'm', 'p', '"', ':', '\"', '+', '0', '0', '0', '.', '0', '\"', '}'};

// uno meno di temp in lunghezza
uint8_t UUID_CHAR_HUM[] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t HUM_CHAR_HANDLE[2];
uint8_t VALUE_HUM[] = {'{', '\"', 'H', 'u', 'm', '"', ':', '\"', '+', '0', '0', '0', '.', '0', '\"', '}'};

// uno meno di temp in lunghezza
uint8_t UUID_CHAR_PRESS[] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t PRESS_CHAR_HANDLE[2];
uint8_t VALUE_PRESS[] = {'{', '\"', 'P', 'r', 'e', 's', 's', '"', ':', '\"', '+', '0', '0', '0', '.', '0', '\"', '}'};

uint8_t UUID_INERTIAL_SERVICE[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t INERTIAL_SERVICE_HANDLE[2];

uint8_t UUID_CHAR_INERTIAL_NAME[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t NAME_INERTIAL_HANDLE[2];
uint8_t NAME_INERTIAL_VALUE[] = {'{', '\"', 'N', 'a', 'm', 'e', '\"', ':', '\"', 'A', 'c', 'c', 'e', 'l', 'e', 'r', 'o', 'm', 'e', 't', 'e', 'r', '\"', '}'};

uint8_t UUID_CHAR_INERTIAL_ACCX[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t ACCX_CHAR_HANDLE[2];
uint8_t ACCX_INERTIAL_VALUE[] = {'{', '\"', 'A', 'x', 'i', 's', 'X', '\"', ':', '\"', '+', '0', '0', '0', '0', '\"', '}'};

uint8_t UUID_CHAR_INERTIAL_ACCY[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t ACCY_CHAR_HANDLE[2];
uint8_t ACCY_INERTIAL_VALUE[] = {'{', '\"', 'A', 'x', 'i', 's', 'Y', '\"', ':', '\"', '+', '0', '0', '0', '0', '\"', '}'};

uint8_t UUID_CHAR_INERTIAL_ACCZ[] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t ACCZ_CHAR_HANDLE[2];
uint8_t ACCZ_INERTIAL_VALUE[] = {'{', '\"', 'A', 'x', 'i', 's', 'Z', '\"', ':', '\"', '+', '0', '0', '0', '0', '\"', '}'};

uint8_t X_VALUE[] = {'{', '\"', 'A', 'x', 'i', 's', 'X', '\"', ':', '\"', '+', '0', '0', '0', '0', '\"', '}'};
uint8_t Y_VALUE[] = {'{', '\"', 'A', 'x', 'i', 's', 'Y', '\"', ':', '\"', '+', '0', '0', '0', '0', '\"', '}'};
uint8_t Z_VALUE[] = {'{', '\"', 'A', 'x', 'i', 's', 'Z', '\"', ':', '\"', '+', '0', '0', '0', '0', '\"', '}'};
uint8_t MIC_VALUE[] = {'{', '\"', 'M', 'i', 'c', 'r', 'o', 'p', 'h', 'o', 'n', 'e', '\"', ':', '\"', '+', '0', '0', '0', '0', '\"', '}'};

uint8_t UUID_MAGNETIC_SERVICE[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t MAGNETIC_SERVICE_HANDLE[2];

uint8_t UUID_CHAR_MAGNETIC_NAME[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t NAME_MAGNETIC_HANDLE[2];
uint8_t NAME_MAGNETIC_VALUE[] = {'{', '\"', 'N', 'a', 'm', 'e', '\"', ':', '\"', 'M', 'a', 'g', 'n', 'e', 't', 'o', 'm', 'e', 't', 'e', 'r', '\"', '}'};

uint8_t UUID_CHAR_MAGNETIC_MAGX[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t MAGX_CHAR_HANDLE[2];

uint8_t UUID_CHAR_MAGNETIC_MAGY[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t MAGY_CHAR_HANDLE[2];

uint8_t UUID_CHAR_MAGNETIC_MAGZ[] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t MAGZ_CHAR_HANDLE[2];

uint8_t UUID_GYROSCOPE_SERVICE[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t GYROSCOPE_SERVICE_HANDLE[2];

uint8_t UUID_CHAR_GYROSCOPE_NAME[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t NAME_GYROSCOPE_HANDLE[2];
uint8_t NAME_GYROSCOPE_VALUE[] = {'{', '\"', 'N', 'a', 'm', 'e', '\"', ':', '\"', 'G', 'y', 'r', 'o', 's', 'c', 'o', 'p', 'e', '\"', '}'};

uint8_t UUID_CHAR_GYROSCOPE_GYROX[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t GYROX_CHAR_HANDLE[2];

uint8_t UUID_CHAR_GYROSCOPE_GYROY[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t GYROY_CHAR_HANDLE[2];

uint8_t UUID_CHAR_GYROSCOPE_GYROZ[] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00};
uint8_t GYROZ_CHAR_HANDLE[2];

void bleProjectSetup()

{
    uint8_t UUID_SERVICE_1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00}; // Reversed UUID of the service
                                                                                                                                 // let's add the first service
    addService(UUID_SERVICE_1, CUSTOM_SERVICE_HANDLE, 1 + 2 + 3 * 2 + 3 + 3);
    // add the charachteristic of the name of the services
    addCharacteristic(UUID_CHAR_1, CUSTOM_CHAR_HANDLE, CUSTOM_SERVICE_HANDLE, (22), READABLE);

    // set the characteristic
    updateCharValue(CUSTOM_SERVICE_HANDLE, CUSTOM_CHAR_HANDLE, 0, (22), VALUE1);

    // add the temp charachteristic
    addCharacteristic(UUID_CHAR_TEMP, TEMP_CHAR_HANDLE, CUSTOM_SERVICE_HANDLE, (17), READABLE | NOTIFIBLE);

    // set temperature
    updateCharValue(CUSTOM_SERVICE_HANDLE, TEMP_CHAR_HANDLE, 0, (17), VALUE_TEMP);

    // add the hum charachteristic
    addCharacteristic(UUID_CHAR_HUM, HUM_CHAR_HANDLE, CUSTOM_SERVICE_HANDLE, (16), READABLE | NOTIFIBLE);

    // set hum
    updateCharValue(CUSTOM_SERVICE_HANDLE, HUM_CHAR_HANDLE, 0, (16), VALUE_HUM);

    // add the press charachteristic
    addCharacteristic(UUID_CHAR_PRESS, PRESS_CHAR_HANDLE, CUSTOM_SERVICE_HANDLE, (18), READABLE | NOTIFIBLE);

    // set press
    updateCharValue(CUSTOM_SERVICE_HANDLE, PRESS_CHAR_HANDLE, 0, (18), VALUE_PRESS);

    // let's add the second service
    addService(UUID_INERTIAL_SERVICE, INERTIAL_SERVICE_HANDLE, (1 + 2 + 3 + 3 + 3)); // 1 the service 2 the readable char e 3x3 the readable notifiable chars

    // add the charachteristic of the name
    addCharacteristic(UUID_CHAR_INERTIAL_NAME, NAME_INERTIAL_HANDLE, INERTIAL_SERVICE_HANDLE, (24), READABLE);

    // set the characteristic
    updateCharValue(INERTIAL_SERVICE_HANDLE, NAME_INERTIAL_HANDLE, 0, (24), NAME_INERTIAL_VALUE);

    // add the charachteristic of the accelerometer
    addCharacteristic(UUID_CHAR_INERTIAL_ACCX, ACCX_CHAR_HANDLE, INERTIAL_SERVICE_HANDLE, (17), READABLE | NOTIFIBLE);

    // set the characteristic
    updateCharValue(INERTIAL_SERVICE_HANDLE, ACCX_CHAR_HANDLE, 0, (17), ACCX_INERTIAL_VALUE);

    // add the charachteristic of the accelerometer Y
    addCharacteristic(UUID_CHAR_INERTIAL_ACCY, ACCY_CHAR_HANDLE, INERTIAL_SERVICE_HANDLE, (17), READABLE | NOTIFIBLE);

    // set the characteristic
    updateCharValue(INERTIAL_SERVICE_HANDLE, ACCY_CHAR_HANDLE, 0, (17), ACCY_INERTIAL_VALUE);

    // add the charachteristic of the accelerometer Z
    addCharacteristic(UUID_CHAR_INERTIAL_ACCZ, ACCZ_CHAR_HANDLE, INERTIAL_SERVICE_HANDLE, (17), READABLE | NOTIFIBLE);

    // set the characteristic
    updateCharValue(INERTIAL_SERVICE_HANDLE, ACCZ_CHAR_HANDLE, 0, (17), ACCZ_INERTIAL_VALUE);

    // let's add the third service
    addService(UUID_MAGNETIC_SERVICE, MAGNETIC_SERVICE_HANDLE, (1 + 2 + 3 * 3)); // 1 the service 2 the readable char e 3x3 the readable notifiable chars

    // add the charachteristic of the name
    addCharacteristic(UUID_CHAR_MAGNETIC_NAME, NAME_MAGNETIC_HANDLE, MAGNETIC_SERVICE_HANDLE, (23), READABLE);

    // set the characteristic
    updateCharValue(MAGNETIC_SERVICE_HANDLE, NAME_MAGNETIC_HANDLE, 0, (23), NAME_MAGNETIC_VALUE);

    // add the charachteristic of the accelerometer
    addCharacteristic(UUID_CHAR_MAGNETIC_MAGX, MAGX_CHAR_HANDLE, MAGNETIC_SERVICE_HANDLE, (17), READABLE | NOTIFIBLE);

    // set the characteristic
    updateCharValue(MAGNETIC_SERVICE_HANDLE, MAGX_CHAR_HANDLE, 0, (17), X_VALUE);

    // add the charachteristic of the accelerometer
    addCharacteristic(UUID_CHAR_MAGNETIC_MAGY, MAGY_CHAR_HANDLE, MAGNETIC_SERVICE_HANDLE, (17), READABLE | NOTIFIBLE);

    // set the characteristic
    updateCharValue(MAGNETIC_SERVICE_HANDLE, MAGY_CHAR_HANDLE, 0, (17), Y_VALUE);

    // add the charachteristic of the accelerometer
    addCharacteristic(UUID_CHAR_MAGNETIC_MAGZ, MAGZ_CHAR_HANDLE, MAGNETIC_SERVICE_HANDLE, (17), READABLE | NOTIFIBLE);

    // set the characteristic
    updateCharValue(MAGNETIC_SERVICE_HANDLE, MAGZ_CHAR_HANDLE, 0, (17), Z_VALUE);
}
