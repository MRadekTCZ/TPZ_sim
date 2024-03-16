/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SLAVE_ADDRESS 0x01
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
typedef struct TAP_INFO_FRAME
{
	unsigned char actual_tap : 4;
	unsigned char tap_up : 1;
	unsigned char tap_down : 1;
	unsigned char b1 : 1;
	unsigned char b0 : 1;
}tap_info_frame;

typedef union BYTE_u
{
	tap_info_frame byte_8;
	unsigned char byte;
}byte_frame_tap;

byte_frame_tap UART_received_frame;
uint8_t uart_send[5];
uint8_t tap;
uint8_t bit_mask_up;
uint8_t bit_mask_down;

uint32_t task_counter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_UART_Transmit_IT(&huart6, 0x2D2D2, 5);
  HAL_UART_Receive_IT(&huart6, &UART_received_frame, 1);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//2Hz timer - Data sending via USB port. LED toggle. This interrupt has lowest priority.
	if(htim->Instance == TIM10)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_UART_Transmit_IT(&huart2, &uart_send, 5);
		task_counter++;
	}
	if(htim->Instance == TIM11)
	{
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		//Conditions checks if packet is for this Phase device
		if((UART_received_frame.byte_8.b0*2 + UART_received_frame.byte_8.b1) == SLAVE_ADDRESS)
		{
		  uart_send[0] = UART_received_frame.byte_8.actual_tap + 0x01;
		  tap = UART_received_frame.byte_8.actual_tap;
		  bit_mask_down = UART_received_frame.byte_8.tap_down;
		  bit_mask_up = UART_received_frame.byte_8.tap_up;

		  switch_off_all_taps();
		  uart_send[1] = 0;
		  uart_send[2] = 0;
		  uart_send[3] = 0;
		  uart_send[4] = 0;

		  switch(tap)
		  {
		  case 0:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOA, TAP0_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOA, TAP0_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[1] = (bit_mask_down << 1) +  (bit_mask_up << 0);
			  break;
		  case 1:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOA, TAP1_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOA, TAP1_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[1] = (bit_mask_down << 3) +  (bit_mask_up << 2);
			  break;
		  case 2:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOA, TAP2_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOB, TAP2_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[1] = (bit_mask_down << 5) +  (bit_mask_up << 4);
			  break;
		  case 3:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOB, TAP3_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOB, TAP3_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[1] = (bit_mask_down << 7) +  (bit_mask_up << 6);
			  break;
		  case 4:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOA, TAP4_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOB, TAP4_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[2] = (bit_mask_down << 1) +  (bit_mask_up << 0);
			  break;
		  case 5:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOB, TAP5_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOB, TAP5_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[2] = (bit_mask_down << 3) +  (bit_mask_up << 2);
			  break;
		  case 6:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOB, TAP6_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOB, TAP6_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[2] = (bit_mask_down << 5) +  (bit_mask_up << 4);
			  break;
		  case 7:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOB, TAP7_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOB, TAP7_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[2] = (bit_mask_down << 7) +  (bit_mask_up << 6);
			  break;
		  case 8:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOA, TAP8_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOB, TAP8_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[3] = (bit_mask_down << 1) +  (bit_mask_up << 0);
			  break;
		  case 9:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOA, TAP9_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOB, TAP9_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[3] = (bit_mask_down << 3) +  (bit_mask_up << 2);
			  break;
		  case 10:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOC, TAP10_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOC, TAP10_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[3] = (bit_mask_down << 5) +  (bit_mask_up << 4);
			  break;
		  case 11:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOC, TAP11_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOC, TAP11_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[3] = (bit_mask_down << 7) +  (bit_mask_up << 6);
			  break;
		  case 12:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOC, TAP12_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOC, TAP12_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[4] = (bit_mask_down << 1) +  (bit_mask_up << 0);
			  break;
		  case 13:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOC, TAP13_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOC, TAP13_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[4] = (bit_mask_down << 3) +  (bit_mask_up << 2);
			  break;
		  case 14:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOC, TAP14_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOC, TAP14_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[4] = (bit_mask_down << 5) +  (bit_mask_up << 4);
			  break;
		  case 15:
			  bit_mask_down ? HAL_GPIO_WritePin(GPIOC, TAP15_DOWN_Pin, GPIO_PIN_SET) : 0;
			  bit_mask_up ? HAL_GPIO_WritePin(GPIOD, TAP15_UP_Pin, GPIO_PIN_SET) : 0;
			  uart_send[4] = (bit_mask_down << 7) +  (bit_mask_up << 6);
			  break;
		  }

		  HAL_UART_Transmit_IT(&huart6, &uart_send, 5);
		  HAL_UART_Receive_IT(&huart6, &UART_received_frame, 1);
	}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) {
	  //This routine should be done offline (without transformer connection to grid)
	  GPIO_Test_routine();
  }
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
