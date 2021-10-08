/*
 * spi.h
 *
 *  Created on: Oct 5, 2021
 *      Author: Barbara Szyca
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_



#endif /* INC_SPI_H_ */
#include "stm32g0xx.h"
void SPI_Config(void);
void SPI_Enable(void);
void SPI_Disable(void);
void SPI_Send_Char (char c);
//void SPI_Send_String(char* s);
void SPI_Send_String(uint8_t* s);
void  SPI_Read (uint8_t* val);

