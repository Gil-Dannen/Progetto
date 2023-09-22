
#include "io_manager.h"

#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"

#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_magneto.h"

float (*bspFunctionArray[BSP_COUNT])(void);

static struct IOS_Man_type ios_manager[MF_COUNT];

static int16_t placeHolderInt16[3];
static float placeHolderFloat[3];

void bspFunctionInit()
{
    BSP_HSENSOR_Init();
    bspFunctionArray[BSP_humidity] = &BSP_HSENSOR_ReadHumidity;
    BSP_TSENSOR_Init();
    bspFunctionArray[BSP_temperature] = &BSP_TSENSOR_ReadTemp;
    BSP_PSENSOR_Init();
    bspFunctionArray[BSP_pressure] = &BSP_PSENSOR_ReadPressure;
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_MAGNETO_Init();
}

void setMappedFunction(MF mf, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, double offset, double factor)
{
    ios_manager[mf].m_type = GPIOx;
    ios_manager[mf].m_pin = GPIO_Pin;
    ios_manager[mf].m_offset = offset;
    ios_manager[mf].m_factor = factor;
}

void setDigital(MF mf, GPIO_PinState state)
{
    HAL_GPIO_WritePin(ios_manager[mf].m_type, ios_manager[mf].m_pin, state);
}

GPIO_PinState readDigital(MF mf)
{

    if (mf >= MF_COUNT)
        return GPIO_PIN_RESET;
    return HAL_GPIO_ReadPin(ios_manager[mf].m_type, ios_manager[mf].m_pin);
}

float bspGetValue(bspF function)
{
    if (function >= BSP_COUNT)
        return 404;
    return (bspFunctionArray[function])();
}

float *bspGetTripleValue(bspTF function)
{
    if (function >= BSPT_COUNT)
        return NULL;
    switch (function)
    {
    case BSPT_gyro:
        BSP_GYRO_GetXYZ(placeHolderFloat);
        return placeHolderFloat;

    case BSPT_magneto:
        BSP_MAGNETO_GetXYZ(placeHolderInt16);
        break;

    case BSPT_accellero:
        BSP_ACCELERO_AccGetXYZ(placeHolderInt16);
        break;

    default:
        break;
    }

    for (int i = 0; i < 3; i++)
        placeHolderFloat[i] = placeHolderInt16[i];
    return placeHolderFloat;
}

float readAnalog(MF mf)
{
    return 0.f;
}
