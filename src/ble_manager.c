#include "ble_manager.h"
#include "ble_interface.h"
#include "stm32l4xx_hal_spi.h"
#include "io_manager.h"
#include "time_manager.h"

// BLE Commands

uint8_t CUSTOM_SERVICE_HANDLE[2];

uint8_t EVENT_STATUP_DATA[] = {0x04, 0xff, 0x03, 0x01, 0x00, 0x01};

uint8_t ACI_GATT_INIT[] = {0x01, 0x01, 0xfd, 0x00};
uint8_t ACI_GATT_INIT_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x01, 0xfd, 0x00};

uint8_t ACI_GAP_INIT[] = {0x01, 0x8a, 0xfc, 0x03, 0x01, 0x00, 0x0d};
uint8_t ACI_GAP_INIT_COMPLETE[] = {0x04, 0x0e, 0x0a, 0x01, 0x8a, 0xfc, 0x00};
uint8_t GAP_SERVICE_HANDLE[2];
uint8_t GAP_CHAR_NAME_HANDLE[2];
uint8_t GAP_CHAR_APP_HANDLE[2];

uint8_t ACI_GATT_UPDATE_CHAR_VALUE[] = {0x01, 0x06, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t ACI_GATT_UPDATE_CHAR_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x06, 0xfd, 0x00};

uint8_t ACI_GAP_SET_AUTH[] = {0x01, 0x86, 0xfc, 0x0c, 0x00, 0x00, 0x01, 0x00, 0x07, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t ACI_GAP_SET_AUTH_RESP[] = {0x04, 0x0e, 0x04, 0x01, 0x86, 0xfc, 0x00};

uint8_t ACI_HAL_SET_TX_POWER_LEVEL[] = {0x01, 0x0f, 0xfc, 0x02, 0x01, 0x04};
uint8_t ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x0f, 0xfc, 0x00};

uint8_t HCI_LE_SET_SCAN_RESPONSE_DATA[] = {0x01, 0x09, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x09, 0x20, 0x00};

// 0x40,0x06x2
uint8_t ACI_GAP_SET_DISCOVERABLE[] = {0x01, 0x83, 0xfc, 0xff, 0x00, 0x40, 0x06, 0x40, 0x06, 0x01, 0x00, 0xff, 0x09};
uint8_t ACI_GAP_SET_DISCOVERABLE_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x83, 0xfc, 0x00};

uint8_t ADD_CUSTOM_SERVICE[] = {0x01, 0x02, 0xFD, 0x13, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00, 0x01, 0x09}; // 3C 60 bytes il massimo di memoria per il servizio
uint8_t ADD_CUSTOM_SERVICE_COMPLETE[] = {0x04, 0x0e, 0x06, 0x01, 0x02, 0xFD, 0x00};

uint8_t ADD_CUSTOM_CHAR[] = {0x01, 0x04, 0xFD, 0x1A, 0xff, 0xff, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00, 0x10, 0x01};
uint8_t ADD_CUSTOM_CHAR_COMPLETE[] = {0x04, 0x0e, 0x06, 0x01, 0x04, 0xFD, 0x00};

uint8_t UPDATE_CHAR[] = {0x01, 0x06, 0xFD, 0x09, 0xff, 0xff, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};

uint8_t EVENT_DISCONNECTED[] = {0x04, 0x05, 0x04, 0x00};


static uint8_t fetchMasterHeader[] = {0x0b, 0x00, 0x00, 0x00, 0x00};

static uint8_t sendMasterHeader[] = {0x0a, 0x00, 0x00, 0x00, 0x00};

typedef enum
{
    BE_OK = 0,
    BE_ERROR_NO_DATA,
    BE_ERROR,
    BE_ERROR2,

}BleEvent;



// 1 atribute service +2 attribute char readable+3*(2 NOTIFYABLE READABLE charachteristics)

void resetBleModule()
{
    setDigital(MF_BleReset,GPIO_PIN_RESET);
    sleep(10);
    setDigital(MF_BleReset,GPIO_PIN_SET);
}

extern SPI_HandleTypeDef hspi3;
uint8_t deviceName[] = {'S', 'T', 'M', '3', '2', 'L', '4', '7', '5'};
// char deviceName[]={'S','T','M','3','2'};//NOT REVERSED, INCREDIBLE STRINGS ARE NOT LITTLE ENDIAN
// attributes are 1 for service, 2 for readable char and 3 for notify/readable characteristcs

uint8_t buffer[255];


uint8_t *rxEvent;
static uint16_t stackInitCompleteFlag = 0;

void ble_init()
{
    resetBleModule();

    static const size_t eventStartupSize = 7;

    // fetching the reset event
    rxEvent = (uint8_t *)malloc(eventStartupSize);
    int res = fetchBleEvent(rxEvent, eventStartupSize);

    if (res == BE_OK)
    {
        res = checkEventResp(rxEvent, EVENT_STATUP_DATA, eventStartupSize);
        if (res == BE_OK)
            stackInitCompleteFlag |= 0x01;
        
    }

    sleep(10);
    free(rxEvent);

    // INIT GATT

    if (BLE_command(ACI_GATT_INIT, sizeof(ACI_GATT_INIT), ACI_GATT_INIT_COMPLETE, sizeof(ACI_GATT_INIT_COMPLETE), 0) == BE_OK)
        stackInitCompleteFlag |= 0x02;
    free(rxEvent);

    // INIT GAP, actually the handle that i get is a GATT handle of a service, will change the name later

    if (BLE_command(ACI_GAP_INIT, sizeof(ACI_GAP_INIT), ACI_GAP_INIT_COMPLETE, sizeof(ACI_GAP_INIT_COMPLETE), 3) == BE_OK)
    {
        stackInitCompleteFlag |= 0x04;
        memcpy(GAP_SERVICE_HANDLE, rxEvent + 7, 2);
        memcpy(GAP_CHAR_NAME_HANDLE, rxEvent + 9, 2);
        memcpy(GAP_CHAR_APP_HANDLE, rxEvent + 11, 2);
    }
    free(rxEvent);

    // SET THE NAME OF THE BOARD IN THE SERVICE CREATED AUTOMATICALLY
    updateCharValue(GAP_SERVICE_HANDLE, GAP_CHAR_NAME_HANDLE, 0, sizeof(deviceName), deviceName);
    stackInitCompleteFlag |= 0x08;
    free(rxEvent);

    // INIT AUTH

    if (BLE_command(ACI_GAP_SET_AUTH, sizeof(ACI_GAP_SET_AUTH), ACI_GAP_SET_AUTH_RESP, sizeof(ACI_GAP_SET_AUTH_RESP), 0) == BE_OK)
        stackInitCompleteFlag |= 0x10;
    free(rxEvent);

    // SET_TX_LEVEL

    if (BLE_command(ACI_HAL_SET_TX_POWER_LEVEL, sizeof(ACI_HAL_SET_TX_POWER_LEVEL), ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE, sizeof(ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE), 0) == BE_OK)
        stackInitCompleteFlag |= 0x20;
    free(rxEvent);

    // SET SCAN RESPONSE DATA

    if (BLE_command(HCI_LE_SET_SCAN_RESPONSE_DATA, sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA), HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE, sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE), 0) == BE_OK)
        stackInitCompleteFlag |= 0x40;
    free(rxEvent);

    // This will start the advertisment,
    setConnectable();

    sleep(10);
}


int fetchBleEvent(uint8_t *container, int size)
{

    uint8_t * master_header = fetchMasterHeader;
    uint8_t slave_header[5];

    // Wait until it is available an event coming from the BLE module (GPIO PIN COULD CHANGE ACCORDING TO THE BOARD)
    if (!readDigital(MF_BleInt))
        return BE_ERROR;

    sleep(5);
    // PIN_CS of SPI2 LOW
    setDigital(MF_BleCS, GPIO_PIN_RESET);

    // SPI2 in this case, it could change according to the board
    // we send a byte containing a request of reading followed by 4 dummy bytes
    HAL_SPI_TransmitReceive(&hspi3, master_header, slave_header, 5, 1);
    setDigital(MF_BleCS, GPIO_PIN_SET);
    sleep(1);
    setDigital(MF_BleCS, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi3, master_header, slave_header, 5, 1);

    // let's get the size of data available
    int dataSize = (slave_header[3] | slave_header[4] << 8);

    if (dataSize > size)
        dataSize = size;
    
    if (dataSize > 0)
    {
        // let's fill the get the bytes availables and insert them into the container variable
        char dummy = 0xff;
        for (int i = 0; i < dataSize; i++)
            HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&dummy, container + i, 1, 1);
        
        setDigital(MF_BleCS, GPIO_PIN_SET);
    }
    else
    {
        setDigital(MF_BleCS, GPIO_PIN_SET);
        return BE_ERROR_NO_DATA;
    }

    // let's stop the SPI2
    return BE_OK;
        
}

int checkEventResp(uint8_t *event, uint8_t *reference, int size)
{
    for (int j = 0; j < size; j++)
        if (event[j] != reference[j])
            return BE_ERROR2;

    return BE_OK;
}

// TODO make it not blocking function
void sendCommand(uint8_t *command, int size)
{

    uint8_t * master_header = sendMasterHeader;
    uint8_t slave_header[5];

    int result;

    do
    {

        setDigital(MF_BleCS, GPIO_PIN_RESET);

        // wait until it is possible to write

        HAL_SPI_TransmitReceive(&hspi3, master_header, slave_header, 5, 1);
        int bufferSize = (slave_header[2] << 8 | slave_header[1]);
        if (bufferSize >= size)
        {
            HAL_SPI_Transmit(&hspi3, command, size, 1);
            result = 0;
        }
        else
            result = -1;
        // HAL_GPIO_WritePin(CPU_LED_GPIO_Port,CPU_LED_Pin,GPIO_PIN_RESET);
        setDigital(MF_BleCS, GPIO_PIN_SET);
    } while (result != 0);
}

void catchBLE()
{

    if (fetchBleEvent(buffer, 127) != BE_OK)
    {
        // Big Error 
        return;
    }
    if (checkEventResp(buffer, EVENT_DISCONNECTED, 3) == BE_OK)
        setConnectable();

    
}

void setConnectable()
{
    uint8_t *rxEvent;
    // Start advertising
    uint8_t *localname;
    size_t deviceNameSize = sizeof(deviceName);
    size_t terminationSize = 5;
    localname = (uint8_t *)malloc(deviceNameSize + terminationSize); // carattere di terminazione+listauid+slavetemp
    memcpy(localname, deviceName, sizeof(deviceName));
    for(uint8_t i = 0; i < terminationSize; i++)
        localname[deviceNameSize+i] = 0x00;

    ACI_GAP_SET_DISCOVERABLE[11] = deviceNameSize + 1;
    size_t defineDiscoverableSize = sizeof(ACI_GAP_SET_DISCOVERABLE);
    ACI_GAP_SET_DISCOVERABLE[3] = deviceNameSize + terminationSize + defineDiscoverableSize - 4;

    uint8_t *discoverableCommand;
    discoverableCommand = (uint8_t *)malloc(defineDiscoverableSize + deviceNameSize + terminationSize);
    memcpy(discoverableCommand, ACI_GAP_SET_DISCOVERABLE, defineDiscoverableSize);
    memcpy(discoverableCommand + defineDiscoverableSize, localname, deviceNameSize + terminationSize);

    sendCommand(discoverableCommand, deviceNameSize + 5 + defineDiscoverableSize);
    rxEvent = (uint8_t *)malloc(7);

    int res = fetchBleEvent(rxEvent, 7);
    if (res == BE_OK)
    {
        res = checkEventResp(rxEvent, ACI_GAP_SET_DISCOVERABLE_COMPLETE, 7);
        if (res == BE_OK)
        {
            stackInitCompleteFlag |= 0x80;
        }
    }

    free(rxEvent);
    free(discoverableCommand);
    free(localname);
    sleep(10);
}

int BLE_command(uint8_t *command, int size, uint8_t *result, int sizeRes, int returnHandles)
{

    sendCommand(command, size);
    rxEvent = (uint8_t *)malloc(sizeRes + 2 * returnHandles);

    long contatore = 0;
    while (!readDigital(MF_BleInt)) // 30000 cicli??
    {
        contatore++;
        if (contatore > 30000)
        {
            break;
        }
    }

    int response = fetchBleEvent(rxEvent, sizeRes + returnHandles * 2);
    if (response == BE_OK)
        response = checkEventResp(rxEvent, result, sizeRes);
    
    sleep(10);

    return response;
}

void addService(uint8_t *UUID, uint8_t *handle, int attributes)
{
    // memcpy
    memcpy(ADD_CUSTOM_SERVICE + 5, UUID, 16);
    ADD_CUSTOM_SERVICE[22] = attributes;
    if (BLE_command(ADD_CUSTOM_SERVICE, sizeof(ADD_CUSTOM_SERVICE), ADD_CUSTOM_SERVICE_COMPLETE, sizeof(ADD_CUSTOM_SERVICE_COMPLETE), 1) == BE_OK)
    {
        handle[0] = rxEvent[7];
        handle[1] = rxEvent[8];
    }
    free(rxEvent);
}

void addCharacteristic(uint8_t *UUID, uint8_t *handleChar, uint8_t *handleService, uint8_t maxsize, uint8_t proprieties)
{

    memcpy(ADD_CUSTOM_CHAR + 7, UUID, 16);

    ADD_CUSTOM_CHAR[4] = handleService[0];
    ADD_CUSTOM_CHAR[5] = handleService[1];
    ADD_CUSTOM_CHAR[23] = maxsize;
    ADD_CUSTOM_CHAR[25] = proprieties;
    if (BLE_command(ADD_CUSTOM_CHAR, sizeof(ADD_CUSTOM_CHAR), ADD_CUSTOM_CHAR_COMPLETE, sizeof(ADD_CUSTOM_CHAR_COMPLETE), 1) == BE_OK)
    {
        handleChar[0] = rxEvent[7];
        handleChar[1] = rxEvent[8];
    }
    free(rxEvent);
}

void updateCharValue(uint8_t *handleService, uint8_t *handleChar, int offset, int size, uint8_t *data)
{

    UPDATE_CHAR[3] = size + 6;
    UPDATE_CHAR[4] = handleService[0];
    UPDATE_CHAR[5] = handleService[1];
    UPDATE_CHAR[6] = handleChar[0];
    UPDATE_CHAR[7] = handleChar[1];
    UPDATE_CHAR[8] = offset;
    UPDATE_CHAR[9] = size;

    uint8_t *commandComplete;
    commandComplete = (uint8_t *)malloc(10 + size);
    memcpy(commandComplete, UPDATE_CHAR, 10);
    memcpy(commandComplete + 10, data, size);

    BLE_command(commandComplete, 10 + size, ADD_CUSTOM_CHAR_COMPLETE, sizeof(ADD_CUSTOM_CHAR_COMPLETE), 0);

    free(commandComplete);
    free(rxEvent);
}
