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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdbool.h>
#include "Bookmark.h"
#include "MRB_TRIGONOMETRIC_LIB.h"
#include "MRB_PLL.h"
#define RAD_TO_DEGREE_CONV 57.29577951308
#define SCALE 0.003663004
#define PHASE_A 0
#define PHASE_B 1
#define PHASE_C 2
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define COS_PHI_SHIFT 0.2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//BASE ALGO VARAIBLES------------------------
double voltage_phase[3], current_phase[3]; // Phase angles theta i, theta u
double current_RMS[3]; // Current measurements

TPPZ tppz[3];
struct Watchdog watchdog[3];
short int Global_set_actual_tap;

static short int state_machine[3]; // State machine for each phase
static short int set_actual_tap[3] = {0, 0, 0}; // Desired tap number for each phase (may lag behind other phases)
static short int actual_tap[3] = {0, 0, 0}; // Current tap number - during the switching sequence, it may differ for different phases
enum Load Loadtype[3] = {Inductive, Inductive, Inductive};
short int tap_delta;

//PLL------------------------
float q_error; // Error for the q component of the PLL system
uint16_t ADC_raw_current[3];
uint16_t ADC_raw_voltage[2];
uint16_t ADC_main_tap;
//One phase---------
PLL_1phase_Handler PLL_current_phase1;
PLL_1phase_Handler PLL_current_phase2;
PLL_1phase_Handler PLL_current_phase3;
Three_phase_handler I_PLL_1p;
//Three phase---------
//Voltages-------------
struct Axis alfabeta_PLL_U;
struct Axis dq_PLL_U;
PID_handler PID_Voltage_PLL;
Three_phase_handler U_PLL;
double PLL_theta_U;
float PID_PLL_U_output = 0;

//Currents-------------
struct Axis alfabeta_PLL_I;
struct Axis dq_PLL_I;
PID_handler PID_Current_PLL;
Three_phase_handler I_PLL;
double PLL_theta_I;
float PID_PLL_I_output = 0;
//MASTER CONTROL-----------------------
PID_handler Master_Voltage_Control;
#define MVS_SAMPLE_TIME 0.1
#define MVS_DT_TIME 10
double MVS_voltage_set = 220.0;
double MVS_actual_voltage;
double MVS_tap_control;
#define MVS_HISTERESIS 0.05
//EMULATION VARIABLES------------------------

#define EMULATION 1
//#define EMULATION_SIMPLE 2
double current_phase_emulated[3] = {30,150,270}, voltage_phase_emulated[3]={0,120,240};
double actual_current[3], actual_voltage[3];
double theta_U_emulated[3] = {0, PI_BY_3*4, PI_BY_3*2 };
double theta_I_emulated[3] = {0-COS_PHI_SHIFT, PI_BY_3*4-COS_PHI_SHIFT,PI_BY_3*2-COS_PHI_SHIFT};



float DAC_test_RMS;
PLL_1phase_Handler DAC_test_pll;

//DIAGNOSTICS------------------
#include <stdint.h>

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

uint16_t failed_thyristor_ignition = 0;
uint16_t failed_thyristor_ignition_freeze = 0;

byte_frame_tap UART_frame_tap_info[3];
byte_frame_tap UART_received_frame[3];
uint8_t uart_send_frame[3];
uint8_t UART_ERROR;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned int cycles_master_task = 0;
unsigned int cycles_base_algo_task = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//PID calibration
	PID_Voltage_PLL.KP = PID_1_KP;
	PID_Voltage_PLL.KI = PID_1_KI;
	PID_Voltage_PLL.KD = PID_1_KD;
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM13_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_DMA_Init();
  MX_TIM16_Init();
  MX_ADC2_Init();
  MX_TIM17_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim13);

  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);


  HAL_ADC_Start_DMA(&hadc1, (uint16_t*)ADC_raw_voltage, 2);
  HAL_ADC_Start_DMA(&hadc2, (uint16_t*)ADC_raw_current, 3);
  HAL_ADC_Start(&hadc3);
  HAL_UART_Transmit_IT(&huart6, 0x45, 1);
  HAL_UART_Receive_IT(&huart6, &UART_received_frame, 1);
  //HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)rec_data_spi, 1);
  //__HAL_SPI_ENABLE(&hspi3);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 25;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//10 Hz timer
	if(htim->Instance == TIM13)
	{
		//LED for indicationg that the programs is running and there is no timeout.
		//TIM13 interrupt has lowest priority, so if this part has been done, everything is done.
		//If LED toggles too often -> Make new timer with 2Hz frequency.
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		cycles_master_task++;
		*MDB_REG_1020 = cycles_master_task;
		//Tap set from measter read
		//*************************//
		HAL_ADC_Start(&hadc3);
		ADC_main_tap = HAL_ADC_GetValue(&hadc3);
		Global_set_actual_tap = floor(ADC_main_tap*SCALE);
		//**************************//

		//PID - MASTER CONTROL (LOCAL)
		Master_Voltage_Control.KP = 0.1;
		Master_Voltage_Control.KI = 2;
		Master_Voltage_Control.KD = 0;


		MVS_tap_control = PID_Regulator((MVS_voltage_set - MVS_actual_voltage), &Master_Voltage_Control);
		if(MVS_tap_control<-1.0) MVS_tap_control=-1; //Windup
		else if(MVS_tap_control>1.0) MVS_tap_control=1;   //Windup

		if(MVS_tap_control>MVS_HISTERESIS) Global_set_actual_tap -= 1;
		else if(MVS_tap_control<-MVS_HISTERESIS) Global_set_actual_tap += 1;

		if(Global_set_actual_tap>(NUMBER_OF_TAPS-1)) Global_set_actual_tap = NUMBER_OF_TAPS-1;
		else if(Global_set_actual_tap<0) Global_set_actual_tap = 0;

		//Emulation - error always equals 0 in that case
		#ifdef EMULATION
		MVS_actual_voltage = MVS_voltage_set;
		#endif



		//PID calibration
		PID_Voltage_PLL.KP = PID_1_KP;
		PID_Voltage_PLL.KI = PID_1_KI;
		PID_Voltage_PLL.KD = PID_1_KD;


		//Diagnostic data exchange with CM4
		*MDB_REG_1000 = Global_set_actual_tap;
		*MDB_REG_1001 = actual_tap[PHASE_A];
		*MDB_REG_1002 = actual_tap[PHASE_B];
		*MDB_REG_1003 = actual_tap[PHASE_C];
		*MDB_REG_1004 = 366;
		*MDB_REG_1005 = 367;
		*MDB_REG_1006 = 368;
		*MDB_REG_1007 = (unsigned int)(10*current_RMS[PHASE_A]);
		*MDB_REG_1008 = (unsigned int)(10*current_RMS[PHASE_B]);
		*MDB_REG_1009 = (unsigned int)(10*current_RMS[PHASE_C]);

		if(UART_received_frame[0].byte == 0x00) UART_ERROR = 0x01;
		else UART_ERROR = 0x00;
	}

	// 10 kHz frequency timer
	//Base algorithm
	if(htim->Instance == TIM16)
	{

		cycles_base_algo_task++;
		*MDB_REG_1021 = ADC_main_tap;
		//*************************************************************************************************//
		//EMULATION - TESTS AND DEBUGGING
		//*************************************************************************************************//
		#ifdef EMULATION
		for(int i= 0; i<3; i++)
		{
			theta_U_emulated[i] = theta_U_emulated[i]+PI*2*0.0001*50;
			if(theta_U_emulated[i]>PI*2)theta_U_emulated[i] = 0;
		}
		for(int i = 0; i<3; i++)
		{
			theta_I_emulated[i] = theta_I_emulated[i]+PI*2*0.0001*50;
			if(theta_I_emulated[i]>PI*2)theta_I_emulated[i] = 0;
		}

		actual_current[0] = 100*sin_f(theta_I_emulated[0]);
		actual_current[1] = 117*sin_f(theta_I_emulated[1]);
		actual_current[2] = 83*sin_f(theta_I_emulated[2]);
		//actual_current[2] = ADC_raw_current[2] *
		actual_voltage[0] = 311*sin_f(theta_U_emulated[0]);
		actual_voltage[1] = 312*sin_f(theta_U_emulated[1]);
		//actual_voltage[2] = 312*sin_f(theta_U_emulated[2]);
		actual_voltage[2] = -actual_voltage[0] - actual_voltage[1];
		#ifdef EMULATION_SIMPLE
		voltage_phase_emulated[0] =  voltage_phase_emulated[0] + 360*50*0.0001;
		current_phase_emulated[0]=  current_phase_emulated[0] + 360*50*0.0001;
		if(voltage_phase_emulated[0]>360) voltage_phase_emulated[0] = voltage_phase_emulated[0] - 360;
		if(current_phase_emulated[0]>360) current_phase_emulated[0] = current_phase_emulated[0] - 360;
		voltage_phase[0] = voltage_phase_emulated[0]; // input 1 must be voltage
		current_phase[0] = current_phase_emulated[0]; // input 2 must be current
		current_RMS[0] = 50;
		#endif


		#endif
		//*************************************************************************************************//
		//EMULATION - TESTS AND DEBUGGING
		//*************************************************************************************************//

		//*************************************************************************************************//
		//MEASUREMENTS AND PLL ALGORITHM
		//*************************************************************************************************//
		#ifndef EMULATION
		//ADC scalling
		actual_current[0] = (ADC_raw_current[0]-2048)*0.07324218;
		actual_current[1] = (ADC_raw_current[1]-2048)*0.07324218;
		actual_current[2] = (ADC_raw_current[2]-2048)*0.07324218;
		actual_voltage[0] = (ADC_raw_voltage[0]-2048)*0.161133;
		actual_voltage[1] = (ADC_raw_voltage[1]-2048)*0.161133;
		actual_voltage[2] = -actual_voltage[0] - actual_voltage[1];
		#endif
		//*************ONE PHASE PLL - CURRENTS****************
		I_PLL_1p.phase_A = PLL_1phase(actual_current[0], current_RMS[0], &PLL_current_phase1);
		I_PLL_1p.phase_B = PLL_1phase(actual_current[1], current_RMS[1], &PLL_current_phase2);
		I_PLL_1p.phase_C = PLL_1phase(actual_current[2], current_RMS[2], &PLL_current_phase3);

		//*************THREE PHASE PLL - VOLTAGES****************
		alfabeta_PLL_U = AlfaBeta(actual_voltage[0], actual_voltage[1], actual_voltage[2]);
		dq_PLL_U = DQ(alfabeta_PLL_U.x, alfabeta_PLL_U.y, PLL_theta_U);
		PID_PLL_U_output = PID_Regulator(dq_PLL_U.x, &PID_Voltage_PLL);
		PLL_theta_U = Integrator(PID_PLL_U_output, -2*PI, 2*PI, 1);
		U_PLL = PLL_3phase(PLL_theta_U);
		//*************THREE PHASE PLL - Currents - Grid stability****************
		q_error = 0;
		//**************Phases assigned for algorithm
		voltage_phase[0] = U_PLL.phase_A * RAD_TO_DEGREE_CONV;
		current_phase[0] = I_PLL_1p.phase_A * RAD_TO_DEGREE_CONV;
		voltage_phase[1] = U_PLL.phase_B * RAD_TO_DEGREE_CONV;
		current_phase[1] = I_PLL_1p.phase_B * RAD_TO_DEGREE_CONV;
		voltage_phase[2] = U_PLL.phase_C * RAD_TO_DEGREE_CONV;
		current_phase[2] = I_PLL_1p.phase_C * RAD_TO_DEGREE_CONV;
		//*************************************************************************************************//
		//MEASUREMENTS AND PLL ALGORITHM
		//*************************************************************************************************//


		//*************************************************************************************************//
		//MAIN ALGORITHM - 3 phases
		//*************************************************************************************************//
		for (int phase = 0; phase < 3; phase++)
		        {
					//PROTECTIONS
					//************************************
		            tap_delta = Tap_diff(set_actual_tap[phase], actual_tap[phase], step);

		            //protection in case of no current measurement or idling of the transformer
		            if (current_RMS[phase] <= NO_LOAD_STATE)
		            {
		                set_actual_tap[phase] = Global_set_actual_tap;

		                if (actual_tap[phase] != set_actual_tap[phase])
		                {
		                    switch (state_machine[phase])
		                    {
		                    case 0:
		                        if (voltage_phase[phase] < (10 - MARGIN))
		                        {
		                            tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
		                            tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
		                            state_machine[phase] = 1;
		                            UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
		                        }
		                        break;
		                    case 1:
		                        if (voltage_phase[phase] > (350 + MARGIN))
		                        {
		                            tppz[phase].Tap_select[set_actual_tap[phase]].Tap_down = ON;
		                            tppz[phase].Tap_select[set_actual_tap[phase]].Tap_up = ON;
		                            actual_tap[phase] = set_actual_tap[phase];
		                            state_machine[phase] = 0;
		                            UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
		                        }
		                        break;
		                    }
		                }
		            }



		            else
		            {
		                //If everything is ok - Tap set from ADC measurement is taken
		            	if (state_machine[phase] == 0 && (q_error < 0.01))
		                {
		                    Loadtype[phase] = LOAD_DET(voltage_phase[phase], current_phase[phase]);
		                    set_actual_tap[phase] = Global_set_actual_tap;
		                }
		                //protection in case of network transient - PLL overshoot increases rapidly
		                else if (state_machine[phase] == 1 && (q_error >= 0.01))
		                {
		                    set_actual_tap[phase] = actual_tap[phase];
		                    tppz[phase].Tap_select[actual_tap[phase]].Tap_down = ON;
		                    tppz[phase].Tap_select[actual_tap[phase]].Tap_up = ON;
		                    state_machine[phase] = 0;
		                    UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
		                }
		           //************************************
		           //End of protections


		                //*************************************************************************************************//
						//BASE ALGORITHM STARTS HERE
		                //*************************************************************************************************//
						if (actual_tap[phase] == set_actual_tap[phase])
						{
							watchdog[phase].counter = 0; //if tap set is equal to actual tap
							UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
						}
						else if (actual_tap[phase] != set_actual_tap[phase])
						{

							if (actual_tap[phase] < set_actual_tap[phase])
							{
								if (Loadtype[phase] == Resistive) // cos fi -> resistive
								{
									switch (state_machine[phase])
									{
									case 0:
										if (voltage_phase[phase] > (180 + MARGIN)) // non sequantial system
										{
											tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
											tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
											state_machine[phase] = 1;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
										}
										break;

									case 1:
										if (voltage_phase[phase] > (360 - 5 * MARGIN))  //allow for low value short circuit
										{

											tppz[phase].Tap_select[actual_tap[phase]+1].Tap_up = ON;
											tppz[phase].Tap_select[actual_tap[phase]+1].Tap_down = ON;
											state_machine[phase] = 2;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase]+1, phase );
										}
										break;
									case 2:
										if (voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
										{

											actual_tap[phase]++;
											state_machine[phase] = 0;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );

										}
										break;
									}
								}

								if (Loadtype[phase] == Inductive) // cos fi -> inductive
								{
									switch (state_machine[phase])
									{

									case 0:
										// if phases of current and voltage have same polarity
										if (voltage_phase[phase] < (90 - MARGIN) && voltage_phase[phase] > MARGIN && current_phase[phase] < (90 - MARGIN) && current_phase[phase] > MARGIN)
										{
											tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
											tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
											state_machine[phase] = 1;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );

										}
										break;
										// case 0 and case 1 are similiar, there is time for turning off thyristor
									case 1:
										if (voltage_phase[phase] > MARGIN && voltage_phase[phase] < (180 - MARGIN) && current_phase[phase] > MARGIN && current_phase[phase] < (180 - MARGIN))
										{

											tppz[phase].Tap_select[actual_tap[phase] + tap_delta].Tap_up = ON;
											state_machine[phase] = 2;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase] + tap_delta, phase );
										}

										break;

									case 2:
										if (voltage_phase[phase] > (90 + MARGIN) && voltage_phase[phase] < (180 - MARGIN) && current_phase[phase] >(90 + MARGIN))  //Sequential
										{

											tppz[phase].Tap_select[actual_tap[phase] + tap_delta].Tap_down = ON;
											state_machine[phase] = 3;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase] + tap_delta, phase );

										}

										break;

									case 3:
										if (voltage_phase[phase] > MARGIN && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > MARGIN && current_phase[phase] < (90 - MARGIN) && state_machine[phase] == 3)
										{
											//actual_tap[phase]++;
											actual_tap[phase] += tap_delta;
											state_machine[phase] = 0;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );


										}
										break;
									}



								}

								if (Loadtype[phase] == Capacitive) //cof fi -> capacitive
								{
									switch (state_machine[phase])
									{

									case 0:
										if (voltage_phase[phase] < (120 - MARGIN) && voltage_phase[phase] > 2 && current_phase[phase] < (120 - MARGIN) && current_phase[phase] > MARGIN)
										{
											tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
											tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
											state_machine[phase] = 1;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
										}
										break;

									case 1:
										if (voltage_phase[phase] < (180 + MARGIN) && current_phase[phase] < (180 - MARGIN) && current_phase[phase] >(120 + MARGIN))
										{
											tppz[phase].Tap_select[actual_tap[phase] + tap_delta].Tap_down = ON;
											state_machine[phase] = 2;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase] + tap_delta, phase );
										}
										break;

									case 2:
										if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] > (180 + MARGIN))
										{
											tppz[phase].Tap_select[actual_tap[phase] + tap_delta].Tap_up = ON;
											state_machine[phase] = 3;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase] + tap_delta, phase );
										}
										break;

									case 3:
										if (voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
										{
											//actual_tap[phase]++;
											actual_tap[phase] += tap_delta;
											state_machine[phase] = 0;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );

										}
										break;
									}

								}

							}

							else if (actual_tap[phase] > set_actual_tap[phase])
							{
								if (Loadtype[phase] == Resistive) // cos fi -> resistive
								{
									switch (state_machine[phase])
									{
									case 0:
										if (voltage_phase[phase] > (180 + MARGIN))
										{
											tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
											tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
											state_machine[phase] = 1;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
										}
										break;
									case 1:
										if (voltage_phase[phase] > (360 - 5 * MARGIN))
										{
											tppz[phase].Tap_select[actual_tap[phase] - 1].Tap_up = ON;
											tppz[phase].Tap_select[actual_tap[phase] - 1].Tap_down = ON;
											state_machine[phase] = 2;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase]-1, phase );
										}
										break;
									case 2:
										if (voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
										{
											actual_tap[phase]--;
											state_machine[phase] = 0;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );


										}
										break;
									}
								}

								if (Loadtype[phase] == Inductive) //cos fi -> inductive
								{
									switch (state_machine[phase])
									{
									case 0:
										if (voltage_phase[phase] < (180 - MARGIN) && voltage_phase[phase] > 2 && current_phase[phase] < (180 - MARGIN) && current_phase[phase] > MARGIN)
										{
											tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
											tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
											state_machine[phase] = 1;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
										}
										break;
									case 1:
										if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] < (180 - MARGIN)) //bookmark switch
										{
											tppz[phase].Tap_select[actual_tap[phase] - tap_delta].Tap_down = ON;
											state_machine[phase] = 2;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase]-tap_delta, phase );
										}
										break;
									case 2:
										if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] > (180 + MARGIN))
										{
											tppz[phase].Tap_select[actual_tap[phase] - tap_delta].Tap_up = ON;
											state_machine[phase] = 3;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase]-tap_delta, phase );
										}
										break;
									case 3:
										if (voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
										{

											//actual_tap[phase]--;
											actual_tap[phase] -= tap_delta;
											state_machine[phase] = 0;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
										}
										break;
									}

								}

								if (Loadtype[phase] == Capacitive) // cos fi -> capacitive
								{

									//first varant - without circuit shorts
									switch (state_machine[phase])
									{

									case 0:
										if (voltage_phase[phase] > (0 + MARGIN) && current_phase[phase] > (0 + MARGIN) && current_phase[phase] < (180 - MARGIN)) // gdy prad jest w fazie z napieciem
										{
											tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
											state_machine[phase] = 1;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
										}
										break;
									case 1:
										if (voltage_phase[phase] > (90 + MARGIN) && current_phase[phase] > (180 + MARGIN) && voltage_phase[phase] < (180 - MARGIN))  //zezwolenie na zwarcie miedzy zaczepowe o niewielkiej wartoci

										{

											tppz[phase].Tap_select[actual_tap[phase] - tap_delta].Tap_down = ON;
											tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
											state_machine[phase] = 2;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase]- tap_delta, phase );
										}
										break;
									case 2:
										if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] > (180 + MARGIN))  //zezwolenie na zwarcie miedzy zaczepowe o niewielkiej wartoci
										{
											tppz[phase].Tap_select[actual_tap[phase] - tap_delta].Tap_up = ON;
											state_machine[phase] = 3;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase]- tap_delta, phase );
										}
										break;
									case 3:
										if (voltage_phase[phase] > MARGIN && current_phase[phase] > (MARGIN))
										{
											//actual_tap[phase]--;
											actual_tap[phase] -= tap_delta;
											state_machine[phase] = 0;
											UART_frame_tap_info[phase] = Tap_bit(tppz[phase], actual_tap[phase], phase );
										}
										break;
									}
								}

							}

						}
					}
				}

		//Data send to slave described in data sheet
		//bit 7,6 -> SLAVE address (1 or 2 or 3 -> 0b01, 0b10, 0b11)
		//bit 5,4 -> Actual thyristor masking, 11 - down and up on, 10 - down on / up off etc.
		//bit 3:0 -> actual tap number to turn on thyristor (bit mask is put later) (from 0 to 15)
		uart_send_frame[0] = UART_frame_tap_info[0].byte;
		HAL_UART_Transmit_IT(&huart6, &uart_send_frame[0], 1);
		HAL_UART_Receive_IT(&huart6, &UART_received_frame[0].byte, 1);
		//uart_send[1] = UART_frame_tap_info[1].byte;
		//HAL_UART_Transmit_IT(&huart1, &uart_send_frame[1], 1);
		//HAL_UART_Receive_IT(&huart1, &UART_received_frame[1], 1);
		//uart_send[2] = UART_frame_tap_info[2].byte;
		//HAL_UART_Transmit_IT(&huart2, &uart_send_frame[2], 1);
		//HAL_UART_Receive_IT(&huart2, &UART_received_frame[2], 1);

		/*********************************************************************************************************************************/
		//END OF ALGORITHM
		/*********************************************************************************************************************************/


	}
	//Calculating RMS with 1kHz frequency
	//Dividing harshly exhaust micro controller, so it was put in interupt with lower frequency
	if(htim->Instance == TIM17)
	{
		//***************CURRENT RMS***************//

		//Actually it is 1/rms. In debugging it need to write 1/"debugging value" to see actual RMS.
		current_RMS[0] = RMS(actual_current[0], &PLL_current_phase1);
		current_RMS[1] = RMS(actual_current[1], &PLL_current_phase2);
		current_RMS[2] = RMS(actual_current[2], &PLL_current_phase3);

		DAC_test_RMS = RMS((ADC_raw_current[2]-2048)*0.07324218, &DAC_test_pll);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		if((UART_received_frame[0].byte-0x01) != UART_frame_tap_info[0].byte_8.actual_tap)
		{
			failed_thyristor_ignition =+ 1;
			failed_thyristor_ignition_freeze =+ 1;
		}
		else failed_thyristor_ignition = 0;
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
