#ifndef INCLUDE_STATE_MACHINE_H_
#define INCLUDE_STATE_MACHINE_H_

#include "stm32l4xx_hal.h"

typedef enum
{
    ST_UNDEFINED,
    ST_STARTUP,
    ST_IDLE,
    ST_COUNT,
}States;

void setup();

void loop(uint8_t delta);

uint8_t setState(States state);

uint64_t timeInCurrentState();

void setStateTimeout(States state, uint64_t timeout);

States state();

void setDebugEnabled(uint8_t);



#endif