#include "state_machine.h"
#include <stdio.h>

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

void setExitCondition(States nextState, uint8_t (*exitCondition)())
{
    if (!isValidState(nextState) || !exitCondition)
        return;
    states[actualState].exitCondition = exitCondition;
    states[actualState].nextState = nextState;
}

uint8_t setState(States newState)
{
    if (!isValidState(newState))
        return 0;
    previousState = actualState;
    actualState = newState;
    return 1;
}

States state()
{
    return actualState;
}

void setStateTimeout(States state, uint64_t timeout)
{
    if (!isValidState(state) || !timeout)
        return;

    states[actualState].nextState = state;
    states[actualState].timeout = timeout;
}
// This function must be called in enter

void setup()
{
    externalSetup(states);

    setStateMachineTimer(&timeInCurrentStateTimer);

    initDone = 1;
    for (int i = ST_UNDEFINED + 1; i < ST_COUNT; i++)
    {
        states[i].timeout = 0;
        states[i].nextState = ST_UNDEFINED;
        states[i].exitCondition = NULL;
        initDone &= states[i].enter && states[i].beforeLoop && states[i].loop && states[i].afterLoop;
    }

    setState(ST_STARTUP);
}

void loop(uint8_t dt)
{
    if (!initDone || !isValidState(actualState))
        return;
    resetDeltaTime();
    sleep(dt);

    StateStruct *stActualState = &states[actualState];

    if (actualState != previousState)
    {
        timeInCurrentStateTimer = 0;
        stActualState->enter();
        previousState = actualState;
    }

    stActualState->beforeLoop(dt);
    stActualState->loop(dt);
    stActualState->afterLoop(dt);

    if (isValidState(stActualState->nextState) && stActualState->nextState != actualState)
    // controlling if nextState exist and it's valid
    {
        if (stActualState->timeout && timeInCurrentStateTimer >= stActualState->timeout)
        {
            // Automatic timeout exit
            setState(stActualState->nextState);
        }

        if (!stActualState->exitCondition)
            return;

        if (stActualState->exitCondition())
            // Automatic exit on callback condition
            setState(stActualState->nextState);
    }
}
