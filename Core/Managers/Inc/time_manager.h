#ifndef INCLUDE_TIME_MANAGER_H_
#define INCLUDE_TIME_MANAGER_H_

#include "stm32l4xx_hal.h"

typedef enum
{
	TF_Blink = 0,
	TF_SerialSend,
	TF_COUNT

}Timer_Function;

void initTimers();

void setStateMachineTimer(uint64_t * stateTimer);

void setTimer(Timer_Function,void (*)(void),uint64_t);

void resetTimer(Timer_Function);

uint64_t deltaTime();

void resetDeltaTime();

void sleep(uint32_t);

uint8_t isTimerExpired(Timer_Function,uint64_t);

#endif /* INCLUDE_TIME_MANAGER_H_ */
