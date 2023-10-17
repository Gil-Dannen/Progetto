#ifndef __STARTUP_STATE_H
#define __STARTUP_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void startup_enter();

void startup_beforeLoop(uint8_t deltaMs);
void startup_loop(uint8_t deltaMs);
void startup_afterLoop(uint8_t deltaMs);

#ifdef __cplusplus
}
#endif

#endif
