# STM32_SPI_EEPROM
EEPROM basic read-write functions using SPI and UART communication.

Using:
-M95256 EEPROM chip [datasheet](https://www.tme.eu/Document/5be30b2aa7342810d9a9eeb5ab0cd0f7/M95256-WMN6P-DTE.pdf),
-G071RB Nucleo board. 

Connect M95256 to Nucleo - communication via SPI
*HOLD and WRITE PROTECT pins set as Output and set the Output level as high
*blue button on pin PC13 set as Input
*standard UART configuartion

UART is displaying values of buffers transmitted and received via SPI. 
There is also POWER FAIL protection which is causing the double Read Status when starting the program:
![alt Text](https://ibb.co/kK18BsF)

























