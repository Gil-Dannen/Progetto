#ifndef __IDLE_STATE_H
#define __IDLE_STATE_H


#ifdef __cplusplus
extern "C" {
#endif

#include "state_machine.h"
#include <stdint.h>


void idle_enter();
void idle_beforeLoop(uint8_t deltaMs);
void idle_loop(uint8_t deltaMs);
void idle_afterLoop(uint8_t deltaMs);

#ifdef __cplusplus
}
#endif

#endif
