#include "state_machine.h"
#include "time_manager.h"
#include "uart_manager.h"
#include <stdio.h>

uint8_t debugEnabled = 0;

States actualState = ST_UNDEFINED;
States previousState = ST_UNDEFINED;
static uint64_t timeInCurrentStateTimer = 0;

uint8_t initDone = 0;



StateStruct states[ST_COUNT];

// This struct must be only in this file

uint8_t isValidState(States state)
{
    return state > ST_UNDEFINED && state < ST_COUNT; 
}

uint64_t timeInCurrentState()
{
    return timeInCurrentStateTimer;
}

uint8_t setState(States newState)
{
    if(isValidState(newState))
    {
        previousState = actualState;
        actualState = newState;  
        return 1;
    }
    return 0;
}

States state()
{
    return actualState;
}

void setStateTimeout(States state, uint64_t timeout)
{
    if(isValidState(state) && timeout)
    {
        states[actualState].nextState = state;
        states[actualState].timeout = timeout;
    }
}
// This function must be called in enter

void setup()
{
    externalSetup(states);

    setStateMachineTimer(&timeInCurrentStateTimer);

    initDone = 1;
    for(int i = ST_UNDEFINED + 1; i < ST_COUNT; i++)
    {
        states[i].timeout = 0;
        states[i].nextState = ST_UNDEFINED;
        initDone &= states[i].enter && states[i].beforeLoop && states[i].loop && states[i].afterLoop;
    }
    
    setState(ST_STARTUP);
    
    
}

void loop(uint8_t dt)
{
    if(!initDone)
        return;
    resetDeltaTime();
    sleep(dt);
    
    
    if(debugEnabled)
        if((timeInCurrentStateTimer % 10) == 0){
            char text[30];
            sprintf(text,"Actual state: %d",actualState);
            sendMessage(text);
        }

    StateStruct * stActualState = &states[actualState];
    
    if(actualState != previousState){
        timeInCurrentStateTimer = 0;
        stActualState->enter();
        previousState = actualState;
    }
    
    stActualState->beforeLoop(dt);
    stActualState->loop(dt);
    stActualState->afterLoop(dt);

    if(stActualState->timeout 
    && stActualState->nextState
    && stActualState->nextState != actualState
    && timeInCurrentStateTimer >= stActualState->timeout)
        setState(stActualState->nextState);
    

}

void setDebugEnabled(uint8_t status)
{
    debugEnabled = status;
}
