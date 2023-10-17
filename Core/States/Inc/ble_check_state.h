#ifndef __BLE_CHECK_STATE_H
#define __BLE_CHECK_STATE_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void ble_check_enter();
void ble_check_beforeLoop(uint8_t deltaMs);
void ble_check_loop(uint8_t deltaMs);
void ble_check_afterLoop(uint8_t deltaMs);

#ifdef __cplusplus
}
#endif

#endif
