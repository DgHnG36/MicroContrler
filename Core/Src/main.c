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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SEVEN_LEDS 7
#define NUM_LEDS 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

const uint8_t SevenSegDigits[10] = {
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

const uint16_t segPinsSouth[NUM_SEVEN_LEDS] = {
    LED7_a_S_Pin, LED7_b_S_Pin, LED7_c_S_Pin,
    LED7_d_S_Pin, LED7_e_S_Pin, LED7_f_S_Pin, LED7_g_S_Pin
};

const uint16_t segPinsEast[NUM_SEVEN_LEDS] = {
    LED7_a_E_Pin, LED7_b_E_Pin, LED7_c_E_Pin,
    LED7_d_E_Pin, LED7_e_E_Pin, LED7_f_E_Pin, LED7_g_E_Pin
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void display7SEG(uint16_t segPins[], int num) {
	if (num < 0 || num >= 10) {
		return;
	}
	uint8_t pattern = SevenSegDigits[num];
	for (int idx = 0; idx < 7; ++idx) {
		GPIO_PinState pinState = pattern & (1 << idx) ? GPIO_PIN_SET : GPIO_PIN_RESET;
		HAL_GPIO_WritePin(GPIOB, segPins[idx], pinState);
	}
	return;
}

void resetTrafficLight(void) {
	 HAL_GPIO_WritePin(GPIOB, 0xFF, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, 0xFF, GPIO_PIN_RESET);
}


void runTrafLight(uint16_t arrPin1[], uint16_t segPins1[7],
				  uint16_t arrPin2[], uint16_t segPins2[7],
				  int num1, int num2) {
	// The first row of lights is red
	HAL_GPIO_WritePin(GPIOA, arrPin1[0], GPIO_PIN_SET);
	for (uint16_t idx = 1; idx < NUM_LEDS; ++ idx) {
		HAL_GPIO_WritePin(GPIOA, arrPin1[idx], GPIO_PIN_RESET);
	}

	// The second row of lights is green
	HAL_GPIO_WritePin(GPIOA, arrPin2[2], GPIO_PIN_SET);
	for (uint16_t idx = 0; idx < NUM_LEDS - 1; ++ idx) {
		HAL_GPIO_WritePin(GPIOA, arrPin2[idx], GPIO_PIN_RESET);
	}
	// Counter
	for (int t = num1; t >= 0; --t) {
		display7SEG(segPins1, t);
		if (t - num2 < 0) {
			HAL_GPIO_WritePin(GPIOA, arrPin2[1], GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, arrPin2[0]|arrPin2[2], GPIO_PIN_RESET);
			display7SEG(segPins2, t);
		}
		else {
			display7SEG(segPins2, t - num2);
		}
		HAL_Delay(1000);
	}
}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  resetTrafficLight();
  uint16_t arrPinSouth[NUM_LEDS] = {LED_RED_S_Pin, LED_YELLOW_S_Pin, LED_GREEN_S_Pin};
  uint16_t arrPinEast[NUM_LEDS] = {LED_RED_E_Pin, LED_YELLOW_E_Pin, LED_GREEN_E_Pin};

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// 7-segment LED
//	for (int num = 0; num <= 9; ++num) {
//		display7SEG(num);
//		HAL_Delay(1000);
//	}

	//Traffic light
	//First State : South is Green and East is Red
	runTrafLight(arrPinEast, segPinsEast, arrPinSouth, segPinsSouth, 5, 3);

	//Second State : South is Red and East is Green
	runTrafLight(arrPinSouth, segPinsSouth, arrPinEast, segPinsEast, 5, 3);
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
  HAL_GPIO_WritePin(GPIOA, LED_RED_E_Pin|LED_YELLOW_E_Pin|LED_GREEN_E_Pin|LED_RED_S_Pin
                          |LED_YELLOW_S_Pin|LED_GREEN_S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED7_a_S_Pin|LED7_c_S_Pin|LED7_d_E_Pin|LED7_e_E_Pin
                          |LED7_f_E_Pin|LED7_g_E_Pin|LED7_d_S_Pin|LED7_e_S_Pin
                          |LED7_f_S_Pin|LED7_g_S_Pin|LED7_a_E_Pin|LED7_b_E_Pin
                          |LED7_c_E_Pin|LED7_b_S_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_RED_E_Pin LED_YELLOW_E_Pin LED_GREEN_E_Pin LED_RED_S_Pin
                           LED_YELLOW_S_Pin LED_GREEN_S_Pin */
  GPIO_InitStruct.Pin = LED_RED_E_Pin|LED_YELLOW_E_Pin|LED_GREEN_E_Pin|LED_RED_S_Pin
                          |LED_YELLOW_S_Pin|LED_GREEN_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED7_a_S_Pin LED7_c_S_Pin LED7_d_E_Pin LED7_e_E_Pin
                           LED7_f_E_Pin LED7_g_E_Pin LED7_d_S_Pin LED7_e_S_Pin
                           LED7_f_S_Pin LED7_g_S_Pin LED7_a_E_Pin LED7_b_E_Pin
                           LED7_c_E_Pin */
  GPIO_InitStruct.Pin = LED7_a_S_Pin|LED7_c_S_Pin|LED7_d_E_Pin|LED7_e_E_Pin
                          |LED7_f_E_Pin|LED7_g_E_Pin|LED7_d_S_Pin|LED7_e_S_Pin
                          |LED7_f_S_Pin|LED7_g_S_Pin|LED7_a_E_Pin|LED7_b_E_Pin
                          |LED7_c_E_Pin|LED7_b_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configures the port and pin on which the EVENTOUT Cortex signal will be connected */
  HAL_GPIOEx_ConfigEventout(AFIO_EVENTOUT_PORT_B, AFIO_EVENTOUT_PIN_1);

  /*Enables the Event Output */
  HAL_GPIOEx_EnableEventout();

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
