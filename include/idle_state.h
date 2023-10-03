#ifndef __IDLE_STATE_H
#define __IDLE_STATE_H

#include "io_manager.h"
#include "uart_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

void idle_enter();
void idle_beforeLoop(uint8_t deltaMs);
void idle_loop(uint8_t deltaMs);
void idle_afterLoop(uint8_t deltaMs);

#ifdef __cplusplus
}
#endif

#endif
