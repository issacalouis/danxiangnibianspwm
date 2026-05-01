/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
#include "oled.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, KEY_V3_Pin|KEY_V4_Pin|KEY_V2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|SPI_IMU_CS_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KEY_V1_GPIO_Port, KEY_V1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RGB_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_FLASH_CS_GPIO_Port, SPI_FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_RES_Pin|OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_V3_Pin KEY_V4_Pin KEY_V2_Pin */
  GPIO_InitStruct.Pin = KEY_V3_Pin|KEY_V4_Pin|KEY_V2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_KEY_Pin */
  GPIO_InitStruct.Pin = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_IMU_CS_Pin */
  GPIO_InitStruct.Pin = SPI_IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_H4_Pin KEY_H1_Pin KEY_H2_Pin KEY_H3_Pin */
  GPIO_InitStruct.Pin = KEY_H4_Pin|KEY_H1_Pin|KEY_H2_Pin|KEY_H3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_V1_Pin OLED_RES_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = KEY_V1_Pin|OLED_RES_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_Pin */
  GPIO_InitStruct.Pin = RGB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RGB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_FLASH_CS_Pin */
  GPIO_InitStruct.Pin = SPI_FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OLED_DC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
uint8_t KEY_FLAG=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	switch(GPIO_Pin)
	{
	case USER_KEY_Pin: LED2_toggle();KEY_FLAG=!KEY_FLAG;OLED_Clear();break;
	}
}


void for_delay_nop(uint32_t nu)
{
    //uint32_t Delay = nu * 168/4;
    do
    {
        __NOP();
    }
    while (nu --);
}


void handle_bit_high(uint8_t bit_position)
{
	RGB(1);
	for_delay_nop(5);
	RGB(0);
	for_delay_nop(1);
}

void handle_bit_low(uint8_t bit_position)
{
	RGB(1);
	for_delay_nop(1);
	RGB(0);
	for_delay_nop(5);
}

void process_rgb_bits(uint8_t R, uint8_t G, uint8_t B)
{
    // (R[23:16] G[15:8] B[7:0])
    uint32_t rgb = ((uint32_t)R << 16) | ((uint32_t)G << 8) | B;

    RGB(0);
    HAL_Delay(2);
    for (int8_t i = 23; i >= 0; i--)
    {
        uint32_t mask = (uint32_t)1 << i;
        if (rgb & mask) {
            handle_bit_high(i);
        } else {
            handle_bit_low(i);
        }
    }
    RGB(1);
}





uint8_t get_keyboard_value(void)
{
    int h_arr[4] = {KEY_H1_Pin, KEY_H2_Pin, KEY_H3_Pin, KEY_H4_Pin};
    int v_arr[4] = {KEY_V1_Pin, KEY_V2_Pin, KEY_V3_Pin, KEY_V4_Pin};
    uint8_t board[16]={'1','2','3','A','4','5','6','B','7','8','9','C','*','0','#','D'};

    int i = 0;
		int j = 0;
    int key_value = 0;
    for (i = 0; i < 4; i++)
    {
    	HAL_GPIO_WritePin((i>0) ? GPIOE:GPIOD,v_arr[i],0);
    	HAL_GPIO_WritePin(((i+1)%4>0) ? GPIOE:GPIOD,v_arr[(i + 1) % 4],1);
    	HAL_GPIO_WritePin(((i+2)%4>0) ? GPIOE:GPIOD,v_arr[(i + 2) % 4],1);
    	HAL_GPIO_WritePin(((i+3)%4>0) ? GPIOE:GPIOD,v_arr[(i + 3) % 4],1);

        HAL_Delay(1);

        for (j = 0; j < 4; j++)
        {
            if (HAL_GPIO_ReadPin(GPIOD, h_arr[j]) == 0)
            {
                key_value = j * 4 + i + 1;
            }
        }

    }
    if (key_value == 0)
    {
        return 0;
    }

    switch(key_value)
	{
    case 1:set_LED_BUZZER(262,50,50);break;
    case 2:set_LED_BUZZER(294,50,50);break;
    case 3:set_LED_BUZZER(330,50,50);break;
    case 4:set_LED_BUZZER(349,50,50);break;
    case 5:set_LED_BUZZER(392,50,50);break;
    case 6:set_LED_BUZZER(440,50,50);break;
    case 7:set_LED_BUZZER(494,50,50);break;
    //default :set_LED_BUZZER(500,50,100);
	}
    return board[key_value-1];
}
/* USER CODE END 2 */
