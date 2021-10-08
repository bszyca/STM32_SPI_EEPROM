/*
 * uart.c
 *
 *  Created on: Oct 1, 2021
 *      Author: Barbara Szyca
 */
#include "uart.h"
void UART_Config (void)
{

    RCC->APBENR1 |= (1<<17); //clock UASRT2 enable
    RCC->IOPENR |= ((1<<0) | (1<<1) | (1<<2) |(1<<3)|(1<<4)|(1<<5)); // I/O ports A B C D E F enable //inaczej RCC->IOPENR |= 63;

    GPIOA->OSPEEDR &= ~(0<<4); //on PA2  HIGH speed

    GPIOA->MODER |= (1<<5); //alternate function on PA2
    GPIOA->MODER &= ~(1<<4); //0 na 5 bicie

    GPIOA->AFR[0] |= (1<<8);
    USART2->CR1 |= (1<<2);  //Receive Enable
    USART2->CR1 |= ((0<<12)|(0<<28));// word length - 1 start bit, 8 data bits, n stop bits
    USART2->CR1 &= ~(1<<15); //oversampling by 16

    USART2->BRR |= 0x683; //baud rate 9600, 16MHz

    USART2->CR2 &= ~(3<<12); //1 stop bit

    USART2->CR1 |= (1<<0);  //USART ENABLE
    USART2->CR1 |= (1<<3);//Transmitter Enable

}

void UART_Send_Char (char c)
{

    USART2->TDR = c; //load the data
    while(!(USART2->ISR & (1<<6))); //TC bit set
}

void UART_Send_String (char  *string)
{
    while (*string != 0)
    {
        UART_Send_Char(*string++);

    }
}
