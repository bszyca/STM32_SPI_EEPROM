/*
 * uart.h
 *
 *  Created on: Oct 1, 2021
 *      Author: Barbara Szyca
 */

#ifndef INC_UART_H_
#define INC_UART_H_



#endif /* INC_UART_H_ */
#include "stm32g0xx.h"
void UART_Config ( void );
void UART_SendString ( char* string );
void UART_SendChar (uint8_t c);
