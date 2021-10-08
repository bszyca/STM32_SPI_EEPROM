/*
 * spi.c
 *
 *  Created on: Oct 5, 2021
 *      Author: Barbara Szyca
 */
#include "spi.h"

void SPI_Config(void)
{
    RCC->APBENR2 |= (1<<12); //spi1 clock enable
    RCC->IOPENR |= ((1<<0) | (1<<1) | (1<<2) |(1<<3)|(1<<4)|(1<<5)); // I/O ports A B C D E F enable

    //EEPROM GPIO
    //PC 13-Button- Input
    GPIOC->MODER &= ~(1<<26);
    GPIOC->MODER &= ~(1<<27);

    //PA0- output
    GPIOA->MODER |= (1<<0); //1 na 0
    GPIOA->MODER &= ~(1<<1); //0 na 1

    GPIOA->BSRR |= (1<<0); //set as high


    //PA1-output
    GPIOA->MODER |= (1<<2); //1 na 2
    GPIOA->MODER &= ~(1<<3); //0 na 3
    GPIOA->BSRR |= (1<<1); //set as high

    //SPI GPIO-alternate
    //SCK PA5
    GPIOA->MODER &= ~(1<<10);
    GPIOA->MODER |= (1<<11);
    GPIOA->AFR[0] |= (0<<20); //AF0

    //MISO PA6
    GPIOA->MODER &= ~(1<<12);
    GPIOA->MODER |= (1<<13);
    GPIOA->AFR[0] |= (0<<24);//Af0

    //MOSI PA7
    GPIOA->MODER &= ~(1<<14);
    GPIOA->MODER |= (1<<15);
    GPIOA->AFR[0] |= (0<<28);

    //CS PB0
    GPIOB->MODER |= (1<<0);
    GPIOB->MODER &= ~(1<<1);
    GPIOB->ODR |= (1<<0);//CS high

    //SPI conf
    SPI1->CR1 |= (1 << 2);  //Master configuration
    SPI1->CR1 |= (1<<4); //fpclk /8
    SPI1->CR1 &= ~(1<<0); //CPHA =0
    SPI1->CR1 &= ~(1<<1); //CPOL =0
    SPI1->CR1 &= ~(1<<10);//full duplex
    SPI1->CR1 &= ~(1<<7); //MSB

    SPI1->CR2 |= (7<<8); //8 bit
    SPI1->CR2 |= (1<<2); //SS enabled
    SPI1->CR2 |= (1<<12); //RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)

    SPI1->CR1 |= (1<<6); //spi enable


}

void SPI_Enable(void)
{
    SPI1->CR1 |= (1<<6); //SPI enable
    GPIOB->BRR |= (1<<0);//CS=0

}

void SPI_Disable(void)
{
    SPI1->CR1 &= ~(1<<6); //SPI disable
    GPIOB->BSRR |= (1<<0); //CS=1

}

void SPI_Send_Char (char c)
{
    //have to be like that
    uint8_t d;
    while(!(SPI1->SR & (1<<1)) ); //TXe flag
    *((uint8_t*) &SPI1->DR) = c;
    while(!(SPI1->SR & (1<<0)) ); //RXe flag
    d = *((uint8_t*) &SPI1->DR) ;
}

//void SPI_Send_String(char* s)
void SPI_Send_String(uint8_t* s)
{
    while(*s !=0)
    {
        //SPI1->DR = *s;
        //  while(!(SPI1->SR & (1<<1)) ); //TXe flag
        SPI_Send_Char (*s);
        s++;
    }
    SPI_Send_Char(0);
}

void SPI_Read (uint8_t* val)
{

    while(!(SPI1->SR & (1<<1)) ); //TXe flag
    *((uint8_t*) &SPI1->DR) = 0;
    while(!(SPI1->SR & (1<<0)) ); //RX flag
    *val = *((uint8_t*)&SPI1->DR) ;

}

