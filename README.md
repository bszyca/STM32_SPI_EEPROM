# STM32_SPI_EEPROM                                                                                                                                                                             
EEPROM basic read-write functions using SPI and UART communication.                                                                                                                
In the main branch is a program using HAL functions.                                                                                                                                         
In the register branch you have the same program using only registers.                                                                                                            

Using:                                                                                                                                                                                  
-M95256 EEPROM chip [datasheet](https://www.tme.eu/Document/5be30b2aa7342810d9a9eeb5ab0cd0f7/M95256-WMN6P-DTE.pdf),                                                                 
-G071RB Nucleo board. 

 Connect M95256 to Nucleo                                                                                                                                                               
 *HOLD and WRITE PROTECT pins set as Output ( Output level set as high)                                                                                                               
*blue button on pin PC13 set as Input + look for the macros in main.h                                                                                                                       

GPIO:                                                                                                                                                                              
-> PA0 and PA1- HOLD and WRITE PROTECT pins set as Output (high)       			                                       																																																		                
-> PB0 - CS - Output (high)												                    																																																															                                                                 
-> PA5 - SCK - Alternate Function																																																                                                                       														

-> PA6 - MISO - Alternate Function													                                                                                                                       

-> PA7 - MOSI - Alternate Function											                                                                                                                                                                                           				                                                                                 																			
                                                                                                                                                                                              

UART is displaying variables transmitted and received via SPI.                                                                                                                                                   
There is also POWER FAIL protection which is causing the double Read Status when starting the program:

![spi_eeprom](https://user-images.githubusercontent.com/91716038/135610682-273f8405-f37a-4da8-a4de-6a5663e42ba7.PNG)

























