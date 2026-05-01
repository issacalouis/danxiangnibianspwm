/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "boost.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "oled.h"
#include "../IMU/IMU.h"
#include "w25q64.h"
#include "dac.h"
#include "stm32f4xx_hal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t uart1_data[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
   void MX_USER_Init(void)
{
    /* 初始化 OLED（必须在 SPI3 MX_SPI3_Init() 之后） */
    OLED_Init();
    OLED_ShowString(0, 0, "INITIALIZING", 16);
    OLED_Refresh();

    /* 初始化 Boost 控制（必须在 TIM1 和 ADC1 MX_xxx_Init() 之后） */
    Boost_Init();                        /* 启动 PWM 和 ADC          */
    Boost_SetTarget(12.0f);             /* 设定初始目标电压 12V      */
}
    void MX_USER_Loop(void)
  {
    static uint32_t last_ctrl_tick = 0;
    static uint32_t last_disp_tick = 0;
    uint32_t now = HAL_GetTick();

    /* 1ms 执行一次 PI 控制环 */
    if (now - last_ctrl_tick >= 1U) {
        last_ctrl_tick = now;
        Boost_ControlLoop();
    }

    /* 10ms 扫描按键 */
    Boost_KeyScan();

    /* 200ms 刷新 OLED 显示 */
    if (now - last_disp_tick >= 200U) {
        last_disp_tick = now;
        OLED_BoostUI(
            Boost_GetTarget(),   /* 目标电压 */
            Boost_GetVout(),     /* 实测电压 */
            Boost_GetDuty()      /* 当前占空比 */
        );
    }
  }

    static void MX_ADC1_DisplayLoop(void)
  {
    static uint32_t last_adc_disp_tick = 0;
    uint32_t now = HAL_GetTick();

    /* Refresh ADC1 raw value on OLED every 100ms */
    if (now - last_adc_disp_tick >= 100U) {
        last_adc_disp_tick = now;

        OLED_ShowString(0, 48, "                ", 16);
        OLED_ShowString(0, 48, "ADC1:", 16);
        OLED_ShowInt(40, 48, (int32_t)boost.adc_raw, 16);
        OLED_Refresh();
    }
  }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float ypr[3];
char syaw[5];
char spitch[5];
char sroll[5];
uint32_t RGB_RNG=0;
int value[3];
char sadc_v[5];
uint8_t eeprom_write_data=0x55;
uint8_t eeprom_read_data=0;
uint8_t key_board_dat[2]={'0','\0'};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);

//  HAL_DAC_Start(&hdac,DAC1_CHANNEL_1);
//  HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,2048);
   MX_USER_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
  
    /* USER CODE BEGIN 3 */
    MX_USER_Loop();
 //   MX_ADC1_DisplayLoop();
  
  }

    /* USER CODE END 3 */
   
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
