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
#include "software_timer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define digitNums 10
#define maxLedCl 4
#define maxLedMatrix 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//LED CLOCK
const uint8_t ledMaps[digitNums] = {
		0b1000000, // 0
		0b1111001, // 1
		0b0100100, // 2
		0b0110000, // 3
		0b0011001, // 4
		0b0010010, // 5
		0b0000010, // 6
		0b1111000, // 7
		0b0000000, // 8
		0b0010000  // 9
};

const uint16_t ledUsed[7] ={
		LED_a_Pin, LED_b_Pin, LED_c_Pin,
		LED_d_Pin, LED_e_Pin, LED_f_Pin, LED_g_Pin
};


volatile uint8_t ledClockBuffers[maxLedCl] = {0};
volatile int curLed = 0;
volatile int hr = 12, min = 34, sec = 50;

//LED MATRIX
const uint16_t LED_COL_Pins[8] = {
    COL0_Pin, COL1_Pin, COL2_Pin, COL3_Pin,
    COL4_Pin, COL5_Pin, COL6_Pin, COL7_Pin
};

const uint16_t LED_ROW_Pins[8] = {
    LED_ROW_0_Pin, LED_ROW_1_Pin, LED_ROW_2_Pin, LED_ROW_3_Pin,
    LED_ROW_4_Pin, LED_ROW_5_Pin, LED_ROW_6_Pin, LED_ROW_7_Pin
};

volatile uint8_t ledMatrixBuffers[8] = {0};
const uint8_t fontsMatrix[36][8] = {
		{0x18,0x3C,0x66,0xC3,0xFF,0xFF,0xC3,0xC3}, //A
		{0xFC,0xC6,0xC6,0xFC,0xC6,0xC6,0xC6,0xFC}, //B
		{0x3E,0x63,0xC0,0xC0,0xC0,0xC0,0x63,0x3E}, //C
		{0xFC,0xC6,0xC3,0xC3,0xC3,0xC3,0xC6,0xFC}, //D
		{0xFF,0xC0,0xC0,0xFE,0xC0,0xC0,0xC0,0xFF}, //E
		{0xFF,0xC0,0xC0,0xFE,0xC0,0xC0,0xC0,0xC0}, //F
		{0x3E,0x63,0xC0,0xCF,0xC3,0xC3,0x63,0x3F}, //G
		{0xC3,0xC3,0xC3,0xFF,0xC3,0xC3,0xC3,0xC3}, //H
		{0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x3C}, //I
		{0x1E,0x0C,0x0C,0x0C,0x0C,0xCC,0xCC,0x78}, //J
		{0xC6,0xCC,0xD8,0xF0,0xF0,0xD8,0xCC,0xC6}, //K
		{0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xFF}, //L
		{0xC3,0xE7,0xFF,0xFF,0xDB,0xC3,0xC3,0xC3}, //M
		{0xC3,0xE3,0xF3,0xDB,0xCF,0xC7,0xC3,0xC3}, //N
		{0x3C,0x66,0xC3,0xC3,0xC3,0xC3,0x66,0x3C}, //O
		{0xFC,0xC6,0xC6,0xFC,0xC0,0xC0,0xC0,0xC0}, //P
		{0x3C,0x66,0xC3,0xC3,0xC3,0xDB,0x66,0x3D}, //Q
		{0xFC,0xC6,0xC6,0xFC,0xF0,0xD8,0xCC,0xC6}, //R
		{0x3C,0x66,0x60,0x3C,0x06,0x06,0x66,0x3C}, //S
		{0xFF,0x18,0x18,0x18,0x18,0x18,0x18,0x18}, //T
		{0xC3,0xC3,0xC3,0xC3,0xC3,0xC3,0x66,0x3C}, //U
		{0xC3,0xC3,0xC3,0xC3,0x66,0x66,0x3C,0x18}, //V
		{0xC3,0xC3,0xDB,0xFF,0xFF,0xE7,0xC3,0xC3}, //W
		{0xC3,0x66,0x3C,0x18,0x18,0x3C,0x66,0xC3}, //X
		{0xC3,0x66,0x3C,0x18,0x18,0x18,0x18,0x18}, //Y
		{0xFF,0x06,0x0C,0x18,0x30,0x60,0xC0,0xFF}, //Z
		// ===== 0–9 =====
		{0x3C,0x66,0xC3,0xC3,0xC3,0xC3,0x66,0x3C}, //0
		{0x18,0x38,0x18,0x18,0x18,0x18,0x18,0x7E}, //1
		{0x3C,0x66,0x06,0x0C,0x18,0x30,0x60,0x7E}, //2
		{0x3C,0x66,0x06,0x1C,0x06,0x06,0x66,0x3C}, //3
		{0x0C,0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x0C}, //4
		{0x7E,0x60,0x60,0x7C,0x06,0x06,0x66,0x3C}, //5
		{0x1C,0x30,0x60,0x7C,0x66,0x66,0x66,0x3C}, //6
		{0x7E,0x06,0x0C,0x18,0x30,0x30,0x30,0x30}, //7
		{0x3C,0x66,0x66,0x3C,0x66,0x66,0x66,0x3C}, //8
		{0x3C,0x66,0x66,0x3E,0x06,0x0C,0x18,0x38}, //9
};

const char charArr[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
volatile int  charIdx= 0;
volatile int curLedMatrix = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* LED CLOCK */
/* API for LED CLOCK */
void display7SEG(uint8_t digit) {
	HAL_GPIO_WritePin(GPIOB, ledUsed[0], (ledMaps[digit] & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ledUsed[1], (ledMaps[digit] & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ledUsed[2], (ledMaps[digit] & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ledUsed[3], (ledMaps[digit] & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ledUsed[4], (ledMaps[digit] & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ledUsed[5], (ledMaps[digit] & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ledUsed[6], (ledMaps[digit] & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void clearAllDigits(void) {
	HAL_GPIO_WritePin(GPIOA, LED_0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED_3_Pin, GPIO_PIN_SET);
}

void update7SEG(int index) {
	clearAllDigits();

	switch(index) {
		case 0:
			display7SEG(ledClockBuffers[0]);
	        HAL_GPIO_WritePin(GPIOA, LED_0_Pin, GPIO_PIN_RESET);
	        break;
		case 1:
			display7SEG(ledClockBuffers[1]);
			HAL_GPIO_WritePin(GPIOA, LED_1_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			display7SEG(ledClockBuffers[2]);
			HAL_GPIO_WritePin(GPIOA, LED_2_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			display7SEG(ledClockBuffers[3]);
			HAL_GPIO_WritePin(GPIOA, LED_3_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}

void updateClockBuffer(int hr, int min) {
	ledClockBuffers[0] =hr / 10;
	ledClockBuffers[1] =hr % 10;
	ledClockBuffers[2] =min / 10;
	ledClockBuffers[3] = min % 10;
}


/* LED 88 MATRIX */
/* TEST LED 8x8 MATRIX */
void test(void) {
	for (int row = 0; row < maxLedMatrix; ++row) {
		for (int col = 0; col < maxLedMatrix; ++col) {
			HAL_GPIO_WritePin(GPIOA, LED_COL_Pins[col], GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIOB, LED_ROW_Pins[row], GPIO_PIN_RESET);
	}
}

/* API for LED 8x8 MATRIX */

void clearAllLEDsMatrix(void) {
	for (int row = 0; row < maxLedMatrix; row++) {
		HAL_GPIO_WritePin(GPIOB, LED_ROW_Pins[row], GPIO_PIN_SET);
	}
	for (int col = 0; col < maxLedMatrix; col++) {
	    HAL_GPIO_WritePin(GPIOA, LED_COL_Pins[col], GPIO_PIN_SET);
	}
}

void displayLEDRowCol(int row) {
	uint8_t data = ledMatrixBuffers[row];
	for (int col = 0; col < maxLedMatrix; col++) {
		if (data & 0x80) {
			HAL_GPIO_WritePin(GPIOA, LED_COL_Pins[col], GPIO_PIN_RESET);
		}
		else {
			HAL_GPIO_WritePin(GPIOA, LED_COL_Pins[col], GPIO_PIN_SET);
		}
		data <<= 1; //Shift left 1 bits
	}
	HAL_GPIO_WritePin(GPIOB, LED_ROW_Pins[row], GPIO_PIN_RESET);
}

void updateLEDMatrix(int index) {
	clearAllLEDsMatrix();
	displayLEDRowCol(index);
}

void loadCharToBuffer(char c) {
    int idx = -1;
    if (c >= 'A' && c <= 'Z') idx = c - 'A';
    else if (c >= '0' && c <= '9') {
    	idx = 26 + (c - '0');
    }

    if (idx >= 0 && idx < 36) {
        for (int i = 0; i < maxLedMatrix; ++i) {
            ledMatrixBuffers[i] = fontsMatrix[idx][i];
        }
    }
    else {
        for (int i = 0; i < maxLedMatrix; ++i) {
        	ledMatrixBuffers[i] = 0x00;
        }
    }
}


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
  MX_TIM2_Init();
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  setTimer(0, 50);   // LED 7SEG updated
  setTimer(1, 1000); // Clock buffer updated
  setTimer(2, 500);  // LED DOT and RED Toggle and load buffer for LED 8x8 Matrix
  setTimer(3, 2);    // LED 8x8 Matrix scan

  /* First state
   * Clear all LED 7SEG
   * Clear all LED 8x8 Matrix
   * Load buffer for LED 8x8 Matrix
   * */
  clearAllDigits();
  clearAllLEDsMatrix();
  loadCharToBuffer('A');

  while (1)
  {
	  if (isTimerExpired(0)) {
	  		setTimer(0, 50);
	  		update7SEG(curLed);
	  		curLed = (curLed + 1) % maxLedCl;
	  	}

	  	if (isTimerExpired(1)) {
	  		setTimer(1, 1000);

	  		sec++;
	  		if (sec >= 60) {
	  			sec = 0;
	  			min++;
	  		}
	  		if (min >= 60) {
	  			min = 0;
	  			hr++;
	  		}
	  		if (hr >= 24) {
	  			hr = 0;
	  		}
	  		updateClockBuffer(hr, min);
	  	}

	  	if (isTimerExpired(2)) {
	  		setTimer(2, 500);
	  		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	  		HAL_GPIO_TogglePin(LED_DOT_GPIO_Port, LED_DOT_Pin);

	  		loadCharToBuffer(charArr[charIdx]);
	  		charIdx = (charIdx + 1) % (sizeof(charArr) - 1);
	  	}

	  	if (isTimerExpired(3)) {
	  		setTimer(3, 2);
	  		//test();
	  		updateLEDMatrix(curLedMatrix);
	  		curLedMatrix = (curLedMatrix + 1) % maxLedMatrix;
	  	}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, COL0_Pin|COL1_Pin|LED_DOT_Pin|LED_RED_Pin
                          |LED_0_Pin|LED_1_Pin|LED_2_Pin|LED_3_Pin
                          |COL2_Pin|COL3_Pin|COL4_Pin|COL5_Pin
                          |COL6_Pin|COL7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_a_Pin|LED_b_Pin|LED_c_Pin|LED_ROW_2_Pin
                          |LED_ROW_3_Pin|LED_ROW_4_Pin|LED_ROW_5_Pin|LED_ROW_6_Pin
                          |LED_ROW_7_Pin|LED_d_Pin|LED_e_Pin|LED_f_Pin
                          |LED_g_Pin|LED_ROW_0_Pin|LED_ROW_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COL0_Pin COL1_Pin LED_DOT_Pin LED_RED_Pin
                           LED_0_Pin LED_1_Pin LED_2_Pin LED_3_Pin
                           COL2_Pin COL3_Pin COL4_Pin COL5_Pin
                           COL6_Pin COL7_Pin */
  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|LED_DOT_Pin|LED_RED_Pin
                          |LED_0_Pin|LED_1_Pin|LED_2_Pin|LED_3_Pin
                          |COL2_Pin|COL3_Pin|COL4_Pin|COL5_Pin
                          |COL6_Pin|COL7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_a_Pin LED_b_Pin LED_c_Pin LED_ROW_2_Pin
                           LED_ROW_3_Pin LED_ROW_4_Pin LED_ROW_5_Pin LED_ROW_6_Pin
                           LED_ROW_7_Pin LED_d_Pin LED_e_Pin LED_f_Pin
                           LED_g_Pin LED_ROW_0_Pin LED_ROW_1_Pin */
  GPIO_InitStruct.Pin = LED_a_Pin|LED_b_Pin|LED_c_Pin|LED_ROW_2_Pin
                          |LED_ROW_3_Pin|LED_ROW_4_Pin|LED_ROW_5_Pin|LED_ROW_6_Pin
                          |LED_ROW_7_Pin|LED_d_Pin|LED_e_Pin|LED_f_Pin
                          |LED_g_Pin|LED_ROW_0_Pin|LED_ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	timerRun();

//	 if (isTimerExpired(0)) {
//		  		setTimer(0, 50);
//		  		update7SEG(curLed);
//		  		curLed = (curLed + 1) % 4;
//		  	}
//
//		  	if (isTimerExpired(1)) {
//		  		setTimer(1, 1000);
//
//		  		sec++;
//		  		if (sec >= 60) {
//		  			sec = 0;
//		  			min++;
//		  		}
//		  		if (min >= 60) {
//		  			min = 0;
//		  			hr++;
//		  		}
//		  		if (hr >= 24) {
//		  			hr = 0;
//		  		}
//		  		updateClockBuffer(hr, min);
//		  	}
//
//		  	if (isTimerExpired(2)) {
//		  		setTimer(2, 500);
//		  		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
//		  		HAL_GPIO_TogglePin(LED_DOT_GPIO_Port, LED_DOT_Pin);
//		  	}

}

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
