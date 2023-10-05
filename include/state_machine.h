#ifndef INCLUDE_STATE_MACHINE_H_
#define INCLUDE_STATE_MACHINE_H_

#include "state_all.h"

void setup();

void loop(uint8_t delta);

uint8_t setState(States state);

uint64_t timeInCurrentState();

void setStateTimeout(States state, uint64_t timeout);

States state();

void setDebugEnabled(uint8_t);



#endif