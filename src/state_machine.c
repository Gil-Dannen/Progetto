#include "state_machine.h"
#include "startup_state.h"
#include "idle_state.h"
#include "time_manager.h"
#include "uart_manager.h"
#include <stdio.h>

uint8_t debugEnabled = 0;

States actualState = ST_UNDEFINED;
States previousState = ST_UNDEFINED;
static uint64_t timeInCurrentState_timer = 0;

uint8_t initDone = 0;

typedef struct 
{
    void (*enter)();
    void (*beforeLoop)(uint8_t);
    void (*loop)(uint8_t);
    void (*afterLoop)(uint8_t);
    uint64_t timeout;
    States nextState;
}StateStruct;

StateStruct states[ST_COUNT];

// This struct must be only in this file

uint8_t isValidState(States state)
{
    return state > ST_UNDEFINED && state < ST_COUNT; 
}

uint64_t timeInCurrentState()
{
    return timeInCurrentState_timer;
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
    states[ST_STARTUP].enter = startup_enter;
    states[ST_STARTUP].beforeLoop = startup_beforeLoop;
    states[ST_STARTUP].loop = startup_loop;
    states[ST_STARTUP].afterLoop = startup_afterLoop;

    states[ST_IDLE].enter = idle_enter;
    states[ST_IDLE].beforeLoop = idle_beforeLoop;
    states[ST_IDLE].loop = idle_loop;
    states[ST_IDLE].afterLoop = idle_afterLoop;

    setStateMachineTimer(&timeInCurrentState_timer);

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
    sleep(dt);
    
    if(debugEnabled)
        if((timeInCurrentState_timer % 10) == 0){
            char text[30];
            sprintf(text,"Actual state: %d",actualState);
            sendMessage(text);
        }

    StateStruct * stActualState = &states[actualState];
    
    if(actualState != previousState){
        timeInCurrentState_timer = 0;
        stActualState->enter();
        previousState = actualState;
    }
    
    stActualState->beforeLoop(dt);
    stActualState->loop(dt);
    stActualState->afterLoop(dt);

    if(stActualState->timeout 
    && stActualState->nextState
    && stActualState->nextState != actualState
    && timeInCurrentState_timer>= stActualState->timeout)
        setState(stActualState->nextState);
    

}

void setDebugEnabled(uint8_t status)
{
    debugEnabled = status;
}
