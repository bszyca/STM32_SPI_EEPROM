/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define button_Pin GPIO_PIN_13
#define button_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
//  MAKRA  //
//(GPIOx->IDR & GPIO_Pin)               // READ PIN (zwraca 1<<GPIO_PIN lub 0)
//((GPIOx->IDR & GPIO_Pin)/GPIO_Pin)    // READ PIN (zwraca 1 lub 0)
//GPIOx->BSRR = (uint32_t)GPIO_Pin      // SET PIN
//GPIOx->BRR = (uint32_t)GPIO_Pin       // RESET PIN
//GPIOx->BSRR = ((GPIOx->ODR & GPIO_Pin) << 16u) | (~(GPIOx->ODR) & GPIO_Pin) // TOGGLE PIN

#define READ_PIN(GPIO_Port, GPIO_Pin)     ((GPIO_Port->IDR & GPIO_Pin)/GPIO_Pin)
#define SET_PIN(GPIO_Port, GPIO_Pin)      GPIO_Port->BSRR = (uint32_t)GPIO_Pin
#define RESET_PIN(GPIO_Port, GPIO_Pin)    GPIO_Port->BRR = (uint32_t)GPIO_Pin
#define TOGGLE_PIN(GPIO_Port, GPIO_Pin)   GPIO_Port->BSRR = ((GPIO_Port->ODR & GPIO_Pin) << 16u) | (~(GPIO_Port->ODR) & GPIO_Pin)

#define Read_Button READ_PIN(button_GPIO_Port , button_Pin)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
