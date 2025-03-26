/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ST75161.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern const  uint8_t font5x7[][5] ;
extern const uint8_t font5x7_italic[][5];
extern const uint8_t font7x9[][7];
extern const uint8_t font5x7_small[][5];
extern unsigned char  logo[];
extern uint8_t dog_icon[16 * 16];
extern const unsigned char epd_bitmap_horse [];
extern const unsigned char grayhorse [];
extern const unsigned char image_data[];
extern const uint16_t font5x7_GRAY[][5];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char LCDString[50],Lcd_char;
int number=0;
uint8_t grayLevel=1;
uint16_t font5x7_gray[96][5]; // Assuming 96 characters from space to ~
uint32_t sine_val[2]={0,4095};


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_DMA_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start(&htim6);// Start for DAC DMA event
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,(uint32_t*) sine_val, 2, DAC_ALIGN_12B_R);
	
	
	
		LCD_SetInterfaceMode(0);//4 wire
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	HAL_Delay(500);

//	lcd_init(0);  // 1 = Grayscale Mode, 0 = Monochrome Mode 
//	ST75161_clear();
//	lcd_draw_black_box();
//	Fill_BLACK();
//	ST75161_clear();
//	lcd_init(1);  // 1 = Grayscale Mode, 0 = Monochrome Mode
//	lcd_fill_gray(0);  // Black
//	lcd_fill_gray(1);  // Dark Gray
//	lcd_fill_gray(2);  // Light Gray
//	lcd_fill_gray(3);  // White
//	
//	
//	
//	
//	
			lcd_init(0);  // 1 = Grayscale Mode, 0 = Monochrome Mode 
      ST75161_clear();
			
//	
//	
//	for(char i=0;i<160;i++){
//		//ST75161_clear_area(10, 9,40, 1);
//		//sprintf(LCDString, "%d", i);
//		lcd_draw_char(10, 72, i,(uint8_t *) font5x7, 5,7);
//		Fill_8_row(0,i);
//	}
//	lcd_draw_black_box();
//	lcd_draw_rectangle(10,3,50,10);
//	lcd_draw_rectangle_border(80,0,30,5,1);
//	
//	lcd_set_pixel(100, 75, 1);  // Turn ON pixel at (x=100, y=75)
//	lcd_set_pixel_gray(100, 100, grayLevel);
//	
//	ST75161_clear();
	lcd_draw_char(0, 0, 'A', (uint8_t *)font5x7, 5, 7);
	lcd_draw_char(10, 0, 'B', (uint8_t *)font5x7, 5, 7);
	lcd_draw_char(20, 0, '0', (uint8_t *)font5x7, 5, 7);
	lcd_draw_char(30, 0, '2', (uint8_t *)font5x7, 5, 7);
	
//	lcd_draw_string_bold(10, 72, "Hello World",(uint8_t *) font5x7, 5,7);
//	lcd_draw_string_bold(10, 56, "ST75161 LCD", (uint8_t *)font5x7_small, 5, 7);
	
	
		
	
//	ST75161_clear();
//	lcd_draw_bitmap(0, 0, epd_bitmap_horse, 160, 125);
//	ST75161_clear();
	lcd_init(1);  // 1 = Grayscale Mode, 0 = Monochrome Mode
	lcd_fill_gray(0);  // White
	lcd_draw_bitmap_gray(0,0,image_data,160,160);
	lcd_fill_gray(0);  // White
	
//	lcd_set_pixel_gray(10, 20, 0);  // Black pixel
//	lcd_set_pixel_gray(11, 20, 1);  // Dark Gray pixel
//	lcd_set_pixel_gray(12, 20, 2);  // Light Gray pixel
//	lcd_set_pixel_gray(13, 20, 3);  // White pixel
//	lcd_set_pixel_gray2(10,21, 0);
//	lcd_set_pixel_gray2(11,21, 1);
//	lcd_set_pixel_gray2(12,21,2);
//	lcd_set_pixel_gray2(13,21, 3);


//lcd_draw_rect_gray(20, 30, 50, 40, 3); // Draw a BLACK rectangle at (20,30) with 50x40 size
//lcd_draw_rect_gray(10, 10, 20, 20, 2); // Medium gray rectangle
//lcd_draw_rect_gray(60, 50, 30, 15, 1); // Light gray rectangle
//lcd_draw_rect_gray(40, 70, 80, 50, 0); // White rectangle (invisible)
	convertFont(font5x7, font5x7_gray, 96);// change standart font to gray scale
	
	




//	lcd_draw_char_gray(0, 8, 'A', (uint16_t *)font5x7_gray, 5, 7,0);

//	lcd_draw_char_gray(5, 8, 'A', (uint16_t *)font5x7_gray, 5, 7,1);

//	lcd_draw_char_gray(10, 8, 'A', (uint16_t *)font5x7_gray, 5, 7,2);

//	lcd_draw_char_gray(15, 8, 'A', (uint16_t *)font5x7_gray, 5, 7,3);

//	lcd_draw_char_gray(27, 8, '@', (uint16_t *)font5x7_gray, 5, 7,grayLevel);
//	lcd_draw_char_gray(35, 8, '1', (uint16_t *)font5x7_gray, 5, 7,grayLevel);
//	
//	lcd_draw_char_gray(43, 8, 'W', (uint16_t *)font5x7_gray, 5, 7,grayLevel);
//	lcd_draw_char_gray(50, 8, 'Q', (uint16_t *)font5x7_gray, 5, 7,grayLevel);
//	lcd_draw_char_gray(56, 8, '2', (uint16_t *)font5x7_gray, 5, 7,grayLevel);
//	lcd_draw_char_gray(61, 8, 'w', (uint16_t *)font5x7_gray, 5, 7,grayLevel);
//	

	lcd_draw_string_gray(10, 56, "Petropargas", (uint16_t *)font5x7_gray, 5, 7, 3);  // White Text
	lcd_draw_string_gray(10, 64, "Electronic Shiraz.", (uint16_t *)font5x7_gray, 5, 7, 0);  // White Text
	lcd_draw_string_gray(10, 72, "Electronic Shiraz.", (uint16_t *)font5x7_gray, 5, 7, 1);  // White Text
	lcd_draw_string_gray(10, 80, "Electronic Shiraz.", (uint16_t *)font5x7_gray, 5, 7, 2);  // White Text
	lcd_draw_string_gray(10, 88, "Electronic Shiraz.", (uint16_t *)font5x7_gray, 5, 7, 3);  // White Text
//	


//	for(char i=32;i<127;i++){
//		lcd_draw_char_gray(100, 8, i, (uint16_t *)font5x7_gray, 5, 7,grayLevel);
//	}


	
	//lcd_update_screen();  // Send buffer to LCD

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IF0_Pin|CSB_Pin|A0_Pin|RES_Pin
                          |D4_Pin|D3_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|D7_Pin|D6_Pin|D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|RWR_Pin|ERD_Pin|SCL_Pin
                          |SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IF1_GPIO_Port, IF1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IF0_Pin */
  GPIO_InitStruct.Pin = IF0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IF0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSB_Pin A0_Pin RES_Pin */
  GPIO_InitStruct.Pin = CSB_Pin|A0_Pin|RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 D7_Pin D6_Pin D5_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|D7_Pin|D6_Pin|D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D3_Pin D2_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D3_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IF1_Pin */
  GPIO_InitStruct.Pin = IF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IF1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RWR_Pin ERD_Pin */
  GPIO_InitStruct.Pin = RWR_Pin|ERD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_SPI1_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
