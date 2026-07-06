#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define OLED_DC_GPIO_Port    GPIOC
#define OLED_DC_Pin          GPIO_PIN_11
#define OLED_CS_GPIO_Port    GPIOD
#define OLED_CS_Pin          GPIO_PIN_1
#define OLED_RES_GPIO_Port   GPIOD
#define OLED_RES_Pin         GPIO_PIN_0

#define OLED_DC_CMD()   HAL_GPIO_WritePin(OLED_DC_GPIO_Port,  OLED_DC_Pin,  GPIO_PIN_RESET)
#define OLED_DC_DATA()  HAL_GPIO_WritePin(OLED_DC_GPIO_Port,  OLED_DC_Pin,  GPIO_PIN_SET)
#define OLED_CS_LOW()   HAL_GPIO_WritePin(OLED_CS_GPIO_Port,  OLED_CS_Pin,  GPIO_PIN_RESET)
#define OLED_CS_HIGH()  HAL_GPIO_WritePin(OLED_CS_GPIO_Port,  OLED_CS_Pin,  GPIO_PIN_SET)
#define OLED_RST_LOW()  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET)
#define OLED_RST_HIGH() HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET)

#define OLED_DC_Clr()   OLED_DC_CMD()
#define OLED_DC_Set()   OLED_DC_DATA()
#define OLED_CS_Clr()   OLED_CS_LOW()
#define OLED_CS_Set()   OLED_CS_HIGH()
#define OLED_RES_Clr()  OLED_RST_LOW()
#define OLED_RES_Set()  OLED_RST_HIGH()

#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_PAGES     8

extern SPI_HandleTypeDef hspi3;

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color);
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t fill);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size);
void OLED_ShowFloat(uint8_t x, uint8_t y, float val, uint8_t decimals, uint8_t size);
void OLED_ShowInt(uint8_t x, uint8_t y, int32_t val, uint8_t size);
void OLED_BoostUI(float v_target, float i_limit, float output_freq_hz, float v_out, float i_out, float i_in, float modulation, uint8_t edit_mode, uint8_t run_state);
void OLED_BoostCalUI(uint8_t cal_mode, float bias, float value);

#endif /* __OLED_H */
