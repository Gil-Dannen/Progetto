#ifndef __INTERFACE_H
#define __INTERFACE_H

#include "io_manager.h"
#include "uart_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

void setup();

void beforeLoop(uint8_t deltaMs);
void loop(uint8_t deltaMs);
void afterLoop(uint8_t deltaMs);

#ifdef __cplusplus
}
#endif

#endif
