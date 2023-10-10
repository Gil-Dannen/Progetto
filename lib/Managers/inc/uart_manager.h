
#ifndef INC_UART_MANAGER_H_
#define INC_UART_MANAGER_H_

#include "stm32l4xx_hal.h"

HAL_StatusTypeDef sendMessage();

void appendMessage(char *);

void uart_init(void);



#endif /* INC_UART_MANAGER_H_ */
