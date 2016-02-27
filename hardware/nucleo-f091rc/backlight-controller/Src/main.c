/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define LEDS 12
#define TILES_X_COUNT 5
#define TILES_Y_COUNT 3

GPIO_TypeDef *ledGPIO[] = {
				E0_L0_R_E2_L2_R_GPIO_Port,	E0_L0_G_E2_L2_G_GPIO_Port,	E0_L0_B_E2_L2_B_GPIO_Port,
				E0_L1_R_GPIO_Port,		E0_L1_G_GPIO_Port,		E0_L1_B_GPIO_Port,
				E0_L2_R_GPIO_Port,		E0_L2_G_GPIO_Port,		E0_L2_B_GPIO_Port,
				E0_L3_R_GPIO_Port,		E0_L3_G_GPIO_Port,		E0_L3_B_GPIO_Port,
				E0_L4_R_E3_L0_R_GPIO_Port,	E0_L4_G_E3_L0_G_GPIO_Port,	E0_L4_B_E3_L0_B_GPIO_Port,
				E3_L1_R_GPIO_Port,		E3_L1_G_GPIO_Port,		E3_L1_B_GPIO_Port,
				E3_L2_R_E1_L0_R_GPIO_Port,	E3_L2_G_E1_L0_G_GPIO_Port,	E3_L2_B_E1_L0_B_GPIO_Port,
				E1_L1_R_INT_G_GPIO_Port,	E1_L1_G_GPIO_Port,		E1_L1_B_GPIO_Port,
				E1_L2_R_GPIO_Port,		E1_L2_G_GPIO_Port,		E1_L2_B_GPIO_Port,
				E1_L3_R_GPIO_Port,		E1_L3_G_GPIO_Port,		E1_L3_B_GPIO_Port,
				E1_L4_R_E2_L0_R_GPIO_Port,	E1_L4_G_E2_L0_G_GPIO_Port,	E1_L4_B_E2_L0_B_GPIO_Port,
				E2_L1_R_GPIO_Port,		E2_L1_G_GPIO_Port,		E2_L1_B_GPIO_Port,
			};
uint16_t      ledPIN[]  = {
				E0_L0_R_E2_L2_R_Pin,		E0_L0_G_E2_L2_G_Pin,		E0_L0_B_E2_L2_B_Pin,
				E0_L1_R_Pin,			E0_L1_G_Pin,			E0_L1_B_Pin,
				E0_L2_R_Pin,			E0_L2_G_Pin,			E0_L2_B_Pin,
				E0_L3_R_Pin,			E0_L3_G_Pin,			E0_L3_B_Pin,
				E0_L4_R_E3_L0_R_Pin,		E0_L4_G_E3_L0_G_Pin,		E0_L4_B_E3_L0_B_Pin,
				E3_L1_R_Pin,			E3_L1_G_Pin,			E3_L1_B_Pin,
				E3_L2_R_E1_L0_R_Pin,		E3_L2_G_E1_L0_G_Pin,		E3_L2_B_E1_L0_B_Pin,
				E1_L1_R_INT_G_Pin,		E1_L1_G_Pin,			E1_L1_B_Pin,
				E1_L2_R_Pin,			E1_L2_G_Pin,			E1_L2_B_Pin,
				E1_L3_R_Pin,			E1_L3_G_Pin,			E1_L3_B_Pin,
				E1_L4_R_E2_L0_R_Pin,		E1_L4_G_E2_L0_G_Pin,		E1_L4_B_E2_L0_B_Pin,
				E2_L1_R_Pin,			E2_L1_G_Pin,			E2_L1_B_Pin,
			};
uint8_t       led[LEDS*3] = {0};


struct __attribute__((__packed__)) in {
	char preamble;
	char edge_id;
	char led_id;
	char r;
	char g;
	char b;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t uart_buf[1];
    struct in s;
    
    while (1) // Waiting for a preamble
      if (HAL_UART_Receive(&huart2, (uint8_t *)&s, 1, 0) == HAL_OK) {
        if (!*uart_buf) break; // Got the preamble
      }
    
    if (HAL_UART_Receive(&huart2, ((uint8_t *)&s)+1, sizeof(struct in), ~0) == HAL_OK) { // Infinite wait for a frame
      // Correcting values (parsing)
      s.edge_id--;
      s.led_id--;
      if (s.r != 255) s.r--;
      if (s.g != 255) s.g--;
      if (s.b != 255) s.b--;
      uint8_t ledid = 0;
      switch(s.edge_id) {
        case 0: // top
          ledid = s.led_id;
          break;
        case 3: // right
          ledid = (TILES_X_COUNT-1) + s.led_id;
          break;
        case 1: // bottom
          ledid = (TILES_X_COUNT-1) + TILES_Y_COUNT + s.led_id;
          break;
        case 2: // left
          ledid = (TILES_X_COUNT-1) + TILES_Y_COUNT + TILES_X_COUNT + s.led_id;
          break;
      }
      
      // Considering received information
      led[ledid*3 +0] = s.r;
      led[ledid*3 +1] = s.g;
      led[ledid*3 +2] = s.b;
    }
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, E0_L4_B_E3_L0_B_Pin|E0_L4_R_E3_L0_R_Pin|E0_L3_B_Pin|E0_L3_R_Pin 
                          |E0_L4_G_E3_L0_G_Pin|E3_L2_R_E1_L0_R_Pin|E3_L1_B_Pin|E1_L3_B_Pin 
                          |E3_L1_G_Pin|E3_L1_R_Pin|E0_L0_G_E2_L2_G_Pin|E0_L0_R_E2_L2_R_Pin 
                          |E0_L0_B_E2_L2_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, E0_L2_G_Pin|E0_L2_R_Pin|E0_L2_B_Pin|E1_L1_R_INT_G_Pin 
                          |E1_L2_G_Pin|E1_L2_B_Pin|E2_L1_G_Pin|E1_L4_R_E2_L0_R_Pin 
                          |E1_L1_B_Pin|E1_L1_G_Pin|E0_L1_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E0_L3_G_Pin|E1_L4_B_E2_L0_B_Pin|E1_L4_G_E2_L0_G_Pin|E2_L1_B_Pin 
                          |E1_L3_G_Pin|E1_L2_R_Pin|E2_L1_R_Pin|E1_L3_R_Pin 
                          |E0_L1_B_Pin|E3_L2_G_E1_L0_G_Pin|E3_L2_B_E1_L0_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(E0_L1_G_GPIO_Port, E0_L1_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : E0_L4_B_E3_L0_B_Pin E0_L4_R_E3_L0_R_Pin E0_L3_B_Pin E0_L3_R_Pin 
                           E0_L4_G_E3_L0_G_Pin E3_L2_R_E1_L0_R_Pin E3_L1_B_Pin E1_L3_B_Pin 
                           E3_L1_G_Pin E3_L1_R_Pin E0_L0_G_E2_L2_G_Pin E0_L0_R_E2_L2_R_Pin 
                           E0_L0_B_E2_L2_B_Pin */
  GPIO_InitStruct.Pin = E0_L4_B_E3_L0_B_Pin|E0_L4_R_E3_L0_R_Pin|E0_L3_B_Pin|E0_L3_R_Pin 
                          |E0_L4_G_E3_L0_G_Pin|E3_L2_R_E1_L0_R_Pin|E3_L1_B_Pin|E1_L3_B_Pin 
                          |E3_L1_G_Pin|E3_L1_R_Pin|E0_L0_G_E2_L2_G_Pin|E0_L0_R_E2_L2_R_Pin 
                          |E0_L0_B_E2_L2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : E0_L2_G_Pin E0_L2_R_Pin E0_L2_B_Pin E1_L1_R_INT_G_Pin 
                           E1_L2_G_Pin E1_L2_B_Pin E2_L1_G_Pin E1_L4_R_E2_L0_R_Pin 
                           E1_L1_B_Pin E1_L1_G_Pin E0_L1_R_Pin */
  GPIO_InitStruct.Pin = E0_L2_G_Pin|E0_L2_R_Pin|E0_L2_B_Pin|E1_L1_R_INT_G_Pin 
                          |E1_L2_G_Pin|E1_L2_B_Pin|E2_L1_G_Pin|E1_L4_R_E2_L0_R_Pin 
                          |E1_L1_B_Pin|E1_L1_G_Pin|E0_L1_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : E0_L3_G_Pin E1_L4_B_E2_L0_B_Pin E1_L4_G_E2_L0_G_Pin E2_L1_B_Pin 
                           E1_L3_G_Pin E1_L2_R_Pin E2_L1_R_Pin E1_L3_R_Pin 
                           E0_L1_B_Pin E3_L2_G_E1_L0_G_Pin E3_L2_B_E1_L0_B_Pin */
  GPIO_InitStruct.Pin = E0_L3_G_Pin|E1_L4_B_E2_L0_B_Pin|E1_L4_G_E2_L0_G_Pin|E2_L1_B_Pin 
                          |E1_L3_G_Pin|E1_L2_R_Pin|E2_L1_R_Pin|E1_L3_R_Pin 
                          |E0_L1_B_Pin|E3_L2_G_E1_L0_G_Pin|E3_L2_B_E1_L0_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E0_L1_G_Pin */
  GPIO_InitStruct.Pin = E0_L1_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(E0_L1_G_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
