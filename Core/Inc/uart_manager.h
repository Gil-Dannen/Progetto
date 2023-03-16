/*
 * uart_manager.h
 *
 *  Created on: Mar 16, 2023
 *      Author: david
 */

#ifndef INC_UART_MANAGER_H_
#define INC_UART_MANAGER_H_

#include "stm32l4xx_hal.h"

HAL_StatusTypeDef sendMessage(char * text);
void uart_init(void);



#endif /* INC_UART_MANAGER_H_ */
