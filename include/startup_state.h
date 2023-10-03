#ifndef __STARTUP_STATE_H
#define __STARTUP_STATE_H

#include "io_manager.h"
#include "uart_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

void startup_enter();

void startup_beforeLoop(uint8_t deltaMs);
void startup_loop(uint8_t deltaMs);
void startup_afterLoop(uint8_t deltaMs);

#ifdef __cplusplus
}
#endif

#endif
