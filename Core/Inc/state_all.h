#ifndef STATE_ALL_H_
#define STATE_ALL_H_

// In this file you must include all your state header
// You must also create the Enum indentifire and set all the callbacks in other to
// init correctly the state machine
// you will find the callbacks setup in the function externalSetup();


typedef enum
{
    ST_UNDEFINED,
    ST_STARTUP,
    ST_IDLE,
    ST_COUNT,
}States;

#include "startup_state.h"
#include "idle_state.h"
#include <stdint.h>

typedef struct 
{
    void (*enter)();
    void (*beforeLoop)(uint8_t);
    void (*loop)(uint8_t);
    void (*afterLoop)(uint8_t);
    uint8_t (*exitCondition)();
    uint64_t timeout;
    States nextState;
}StateStruct;
// Do not use this struct

void externalSetup(StateStruct states[]);

#endif
