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
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MRB_TRIGONOMETRIC_LIB.h"
#include "Modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned int CM4_cycles_base_algo_task;
unsigned int CM4_cycles_Modbus_task;
unsigned int CM4_cycles_DAC_task;
unsigned int CM4_cycles_PWM_task;

float alfa_sin_dac;
float sin_emulated_double;
double time_s;
unsigned int alfa_sin_decimal_pwm;
unsigned int alfa_sin_decimal_dac;

uint16_t DAC_Output[2] = {0,0};
uint8_t Modbus_buffer_request[10];  //80bits for 10 bytes from UART -> remote control
uint8_t Modbus_rbuffer_response[128];
#define MODBUS_ADDRESS_SLAVE 0x01
Modbus_handler Modbus_request, Modbus_answer;
uint16_t Modbus_registers[100];

volatile uint16_t *MDB_REG_1000 = (uint16_t *)0x38000000;
volatile uint16_t *MDB_REG_1001 = (uint16_t *)0x38000002;
volatile uint16_t *MDB_REG_1002 = (uint16_t *)0x38000004;
volatile uint16_t *MDB_REG_1003 = (uint16_t *)0x38000006;
volatile uint16_t *MDB_REG_1004 = (uint16_t *)0x38000008;
volatile uint16_t *MDB_REG_1005 = (uint16_t *)0x3800000A;
volatile uint16_t *MDB_REG_1006 = (uint16_t *)0x3800000C;
volatile uint16_t *MDB_REG_1007 = (uint16_t *)0x3800000E;
volatile uint16_t *MDB_REG_1008 = (uint16_t *)0x38000010;
volatile uint16_t *MDB_REG_1009 = (uint16_t *)0x38000012;
volatile uint16_t *MDB_REG_1010 = (uint16_t *)0x38000014;
volatile uint16_t *MDB_REG_1011 = (uint16_t *)0x38000016;
volatile uint16_t *MDB_REG_1012 = (uint16_t *)0x38000018;
volatile uint16_t *MDB_REG_1013 = (uint16_t *)0x3800001A;
volatile uint16_t *MDB_REG_1014 = (uint16_t *)0x3800001C;
volatile uint16_t *MDB_REG_1015 = (uint16_t *)0x3800001E;
volatile uint16_t *MDB_REG_1016 = (uint16_t *)0x38000020;
volatile uint16_t *MDB_REG_1017 = (uint16_t *)0x38000022;
volatile uint16_t *MDB_REG_1018 = (uint16_t *)0x38000024;
volatile uint16_t *MDB_REG_1019 = (uint16_t *)0x38000026;
volatile uint16_t *MDB_REG_1020 = (uint16_t *)0x38000028;
volatile uint16_t *MDB_REG_1021 = (uint16_t *)0x3800002A;
volatile uint16_t *MDB_REG_1022 = (uint16_t *)0x3800002C;

volatile uint16_t *MDB_REG_1050 = (uint16_t *)0x38000064;

uint32_t data_read;
uint32_t data_read2;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  MX_USART3_UART_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim15);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_UART_Transmit_IT(&huart3, 0x45, 1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart3, &Modbus_buffer_request, 10);
  // MANUAL INITATE OF GPIO PERIPHAL - STM32 BUG? - it doesn't initialize on CM4
  MX_GPIO_Init();

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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//5Hz timer - LED, counting P,Q,S, SRAM data exchange
	if(htim->Instance == TIM15)
	{
		CM4_cycles_base_algo_task++;
		data_read  = *MDB_REG_1020;
		data_read2  = *MDB_REG_1021;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


	}
	//20kHz - DAC generator

	//50Hz - Modbus and diagnostics
	if(htim->Instance == TIM14)
	{
		  Modbus_registers[0] = 0x12;
		  Modbus_registers[1] = 0x77;
		  Modbus_registers[2] = 0x333;
		  Modbus_registers[5] = 0xABC0;
		  Modbus_registers[20] = *MDB_REG_1020;
		  Modbus_registers[21] = *MDB_REG_1021;
		  Modbus_registers[42] = 0xA337;
		  Modbus_registers[43] = 0xBC23;
		  Modbus_registers[50] = 0xD2D2;
		  Modbus_registers[51] = 0xD2D2;
	}
	if(htim->Instance == TIM7)
	{
		time_s = time_s+ 0.00005;
		alfa_sin_dac = time_s * 50 * 2* MRB_TL_PI;
		if(alfa_sin_dac > 2*MRB_TL_PI) time_s = 0;
		sin_emulated_double = sin_f(alfa_sin_dac);
		alfa_sin_decimal_pwm = floor((sin_emulated_double+1)*500);
		if(sin_emulated_double >=0)
			{
			alfa_sin_decimal_dac = floor((sin_emulated_double)*4095);
			DAC_Output[0] = alfa_sin_decimal_dac;
			DAC_Output[1]= 0;
			}
		else
			{
			alfa_sin_decimal_dac = floor(-1*(sin_emulated_double)*4095);
			DAC_Output[0] = 0;
			DAC_Output[1]= alfa_sin_decimal_dac;
			}
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,DAC_Output[0]);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,DAC_Output[1]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, alfa_sin_decimal_pwm);
	}

}
//MODBUS
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		//Tu ma byc pointer czy nie???
		Modbus_answer = Receive_request(Modbus_buffer_request, MODBUS_ADDRESS_SLAVE, &Modbus_registers);
		 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		 if (Modbus_answer.function.data_u == 3 || Modbus_answer.function.data_u == 4)
		  {
			 HAL_UART_Transmit_IT(&huart3, &Modbus_rbuffer_response, (8 + Modbus_answer.offset_regCount.data_u*2) );
		  }
		 else HAL_UART_Transmit_IT(&huart3, &Modbus_rbuffer_response, 10 );

		 HAL_UART_Receive_IT(&huart3, &Modbus_buffer_request, 10);

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
