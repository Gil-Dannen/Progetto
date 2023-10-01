#include "time_manager.h"


struct Timer
{
	uint64_t value;
	void (*callback)(void);
	uint64_t timeout;

}timers[TF_COUNT];

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
		return;
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

void sleep(uint32_t time)
{

	for(uint8_t tick = 0; tick <= time; tick ++){
		HAL_Delay(tick);

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
