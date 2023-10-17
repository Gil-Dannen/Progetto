#include "time_manager.h"

static uint64_t deltaTimeCounter = 0; 

uint64_t * stateMachineTimer = NULL;
struct Timer
{
	uint64_t value;
	void (*callback)(void);
	uint64_t timeout;

}timers[TF_COUNT];

void resetDeltaTime()
{
    deltaTimeCounter = 0;
}

uint64_t deltaTime()
{
    return deltaTimeCounter;
}

uint8_t isValidTimer(Timer_Function function)
{
	return function < TF_COUNT && function >= 0;
}

void initTimers()
{
	for(uint8_t i = 0; i < TF_COUNT; i++)
	{
		timers[i].value = 0;
		timers[i].timeout  = 0;
		timers[i].callback = NULL;
	}
}

void setTimer(Timer_Function function,void (*callback)(void),uint64_t timeout)
{
	if(!isValidTimer(function) || !callback)
	{
		timers[function].value = 0;
		timers[function].callback = NULL;
		timers[function].timeout = 0;
		return;
	}

	timers[function].value = 0;
	timers[function].callback = callback;
	timers[function].timeout = timeout;

}


void resetTimer(Timer_Function function)
{
	if(!isValidTimer(function))
			return;
	timers[function].value = 0;
}



void setStateMachineTimer(uint64_t * stateTimer)
{
    stateMachineTimer = stateTimer;
    *stateMachineTimer = 0;
}

void sleep(uint32_t time)
{
	if(!time)
		return;
	for(uint8_t tick = 0; tick <= time; tick ++){
		HAL_Delay(tick);
        *stateMachineTimer += tick;
        deltaTimeCounter += tick;

		for(uint8_t i = 0; i < TF_COUNT; i++){
			timers[i].value += tick;
			if(timers[i].value >= timers[i].timeout
					&& timers[i].timeout
					&& timers[i].callback)
			{
				timers[i].value = 0;
				timers[i].callback();
			}

		}
	}


}

uint8_t isTimerExpired(Timer_Function function, uint64_t timeout)
{
	if(!isValidTimer(function))
		return 0;
	return timers[function].value >= timeout;
}
