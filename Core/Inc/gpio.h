/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define LED1(x) HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,x);
#define LED2(x) HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,x);

#define LED1_toggle() HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin)
#define LED2_toggle() HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin)

#define RGB(x) HAL_GPIO_WritePin(RGB_GPIO_Port,RGB_Pin,x);

#define SPI_IMU_CS(x) HAL_GPIO_WritePin(SPI_IMU_CS_GPIO_Port,SPI_IMU_CS_Pin,x);
#define SPI_FLASH_CS(x) HAL_GPIO_WritePin(SPI_FLASH_CS_GPIO_Port,SPI_FLASH_CS_Pin,x);
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t KEY_FLAG;
void process_rgb_bits(uint8_t R, uint8_t G, uint8_t B);
uint8_t get_keyboard_value(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

