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
#define DEFREE_TO_RAD_CONV 0.01745329251
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
double voltage_RMS[3]; // Current measurements
TPPZ tppz[3];
struct Watchdog watchdog[3];
short int Global_set_actual_tap;
static short int state_machine[3]; // State machine for each phase
static short int set_actual_tap[3] = {0, 0, 0}; // Desired tap number for each phase (may lag behind other phases)
static short int actual_tap[3] = {0, 0, 0}; // Current tap number - during the switching sequence, it may differ for different phases
enum Load Loadtype[3] = {Inductive, Inductive, Inductive};
short int tap_delta;
enum change_types Tap_change_type = step;

//PLL------------------------
float q_error; // Error for the q component of the PLL system
uint16_t ADC_raw_current[3];
uint16_t ADC_raw_voltage[2];
uint16_t ADC_main_tap;
//One phase---------
PLL_1phase_Handler PLL_current_phase1;
PLL_1phase_Handler PLL_current_phase2;
PLL_1phase_Handler PLL_current_phase3;
PLL_1phase_Handler PLL_voltage_phase1;
PLL_1phase_Handler PLL_voltage_phase2;
PLL_1phase_Handler PLL_voltage_phase3;
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
volatile uint8_t Transformer_ON_OFF = 0x01; //0 -> off 1-> on
//bit 0 : (0-> Local, 1-> Remote, 2-> Debugging)
uint8_t control_mode_Local_Remote = 0x01;
//bit 1 : (0->Automatic, 1-> Manual)
uint8_t control_mode_Automatic_Manual = 0x00;

//EMULATION VARIABLES------------------------

#define EMULATION 1
//#define EMULATION_SIMPLE 2
double current_phase_emulated[3] = {30,150,270}, voltage_phase_emulated[3]={0,120,240};
double actual_current[3], actual_voltage[3];
double theta_U_emulated[3] = {0, PI_BY_3*4, PI_BY_3*2 };
double theta_I_emulated[3] = {0-COS_PHI_SHIFT, PI_BY_3*4-COS_PHI_SHIFT,PI_BY_3*2-COS_PHI_SHIFT};

//DIAGNOSTICS------------------
//#include <stdint.h>
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
volatile uint16_t *MDB_REG_1023 = (uint16_t *)0x3800002E;
volatile uint16_t *MDB_REG_1024 = (uint16_t *)0x38000030;
volatile uint16_t *MDB_REG_1025 = (uint16_t *)0x38000032;
volatile uint16_t *MDB_REG_1026 = (uint16_t *)0x38000034;
volatile uint16_t *MDB_REG_1027 = (uint16_t *)0x38000036;
volatile uint16_t *MDB_REG_1028 = (uint16_t *)0x38000038;
volatile uint16_t *MDB_REG_1029 = (uint16_t *)0x3800003A;
volatile uint16_t *MDB_REG_1030 = (uint16_t *)0x3800003C;
volatile uint16_t *MDB_REG_1031 = (uint16_t *)0x3800003E;
volatile uint16_t *MDB_REG_1032 = (uint16_t *)0x38000040;
volatile uint16_t *MDB_REG_1033 = (uint16_t *)0x38000042;
volatile uint16_t *MDB_REG_1034 = (uint16_t *)0x38000044;
volatile uint16_t *MDB_REG_1035 = (uint16_t *)0x38000046;
volatile uint16_t *MDB_REG_1036 = (uint16_t *)0x38000048;
volatile uint16_t *MDB_REG_1037 = (uint16_t *)0x3800004A;
volatile uint16_t *MDB_REG_1038 = (uint16_t *)0x3800004C;
volatile uint16_t *MDB_REG_1039 = (uint16_t *)0x3800004E;
volatile uint16_t *MDB_REG_1040 = (uint16_t *)0x38000050;
volatile uint16_t *MDB_REG_1041 = (uint16_t *)0x38000052;
volatile uint16_t *MDB_REG_1042 = (uint16_t *)0x38000054;
volatile uint16_t *MDB_REG_1043 = (uint16_t *)0x38000056;
volatile uint16_t *MDB_REG_1044 = (uint16_t *)0x38000058;
volatile uint16_t *MDB_REG_1045 = (uint16_t *)0x3800005A;
volatile uint16_t *MDB_REG_1046 = (uint16_t *)0x3800005C;
volatile uint16_t *MDB_REG_1047 = (uint16_t *)0x3800005E;
volatile uint16_t *MDB_REG_1048 = (uint16_t *)0x38000060;
volatile uint16_t *MDB_REG_1049 = (uint16_t *)0x38000062;
volatile uint16_t *MDB_REG_1050 = (uint16_t *)0x38000064;
volatile uint16_t *MDB_REG_1051 = (uint16_t *)0x38000066;
volatile uint16_t *MDB_REG_1052 = (uint16_t *)0x38000068;
volatile uint16_t *MDB_REG_1053 = (uint16_t *)0x3800006A;
volatile uint16_t *MDB_REG_1054 = (uint16_t *)0x3800006C;
volatile uint16_t *MDB_REG_1055 = (uint16_t *)0x3800006E;
volatile uint16_t *MDB_REG_1056 = (uint16_t *)0x38000070;
volatile uint16_t *MDB_REG_1057 = (uint16_t *)0x38000072;
volatile uint16_t *MDB_REG_1058 = (uint16_t *)0x38000074;
volatile uint16_t *MDB_REG_1059 = (uint16_t *)0x38000076;
volatile uint16_t *MDB_REG_1060 = (uint16_t *)0x38000078;
volatile uint16_t *MDB_REG_1061 = (uint16_t *)0x3800007A;
volatile uint16_t *MDB_REG_1062 = (uint16_t *)0x3800007C;
volatile uint16_t *MDB_REG_1063 = (uint16_t *)0x3800007E;
volatile uint16_t *MDB_REG_1064 = (uint16_t *)0x38000080;
volatile uint16_t *MDB_REG_1065 = (uint16_t *)0x38000082;
volatile uint16_t *MDB_REG_1066 = (uint16_t *)0x38000084;
volatile uint16_t *MDB_REG_1067 = (uint16_t *)0x38000086;
volatile uint16_t *MDB_REG_1068 = (uint16_t *)0x38000088;
volatile uint16_t *MDB_REG_1069 = (uint16_t *)0x3800008A;
volatile uint16_t *MDB_REG_1070 = (uint16_t *)0x3800008C;
volatile uint16_t *MDB_REG_1071 = (uint16_t *)0x3800008E;
volatile uint16_t *MDB_REG_1072 = (uint16_t *)0x38000090;

//Error statuses
uint16_t failed_thyristor_ignition = 0;
uint16_t failed_thyristor_ignition_freeze = 0;
status_16t error_status;
struct Time
{
	uint16_t cnt;
	uint16_t seconds;
	uint16_t minutes;
}app_time;
float delta_fi[3];
//DATA ECHANGE------
byte_frame_tap UART_frame_tap_info[3];
uint8_t uart_send_frame[3];
uint8_t UART_received_frame_1[5];
uint8_t UART_received_frame_2[5];
uint8_t UART_received_frame_3[5];

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
	app_time.cnt = 0;
	app_time.seconds = 0;
	app_time.minutes = 0;
	//Master control PID calibration
	Master_Voltage_Control.KP = 0.1;
	Master_Voltage_Control.KI = 2;
	Master_Voltage_Control.KD = 0;
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
  HAL_UART_Receive_IT(&huart6, &UART_received_frame_1, 5);

  //Diagnostics data initialization*********
  *MDB_REG_1061 = 22000;
  *MDB_REG_1062 = 0x01;
  *MDB_REG_1063 = 0x00;
  *MDB_REG_1064 = 0x0;
  *MDB_REG_1065 = 0x0;
  *MDB_REG_1066 = 10;
  *MDB_REG_1067 = 200;
  *MDB_REG_1068 = 0;
  *MDB_REG_1069 = 500;
  //SCALE VALUES!!!!!
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
		//TIM13 interrupt has lowest priority, so if this part has been done, everything is done.
		//If Yellow Led toggles, that confirms every task has properly ended

		//Condition checking if the task is ending in every cycle;
		error_status.b_bit.diagnosis_task_done = 0;

		HAL_ADC_Start(&hadc3);
		//**************************//
		//PID - MASTER CONTROL (LOCAL)
		if(control_mode_Automatic_Manual)
		{
			MVS_tap_control = PID_Regulator((MVS_voltage_set - MVS_actual_voltage), &Master_Voltage_Control);
			if(MVS_tap_control<-1.0) MVS_tap_control=-1; //Windup
			else if(MVS_tap_control>1.0) MVS_tap_control=1;   //Windup

			if(MVS_tap_control>MVS_HISTERESIS) Global_set_actual_tap -= 1;
			else if(MVS_tap_control<-MVS_HISTERESIS) Global_set_actual_tap += 1;


			#ifdef EMULATION
			//Emulation - error always equals 0 in that case
			MVS_actual_voltage = MVS_voltage_set;
			#endif
			#ifndef EMULATION
			MVS_actual_voltage = voltage_RMS[PHASE_A];
			#endif

		}
		else
		{
			//Tap set from mester read
			//*************************//
			if(control_mode_Local_Remote == 0x01)
			{
				ADC_main_tap = HAL_ADC_GetValue(&hadc3);
				Global_set_actual_tap = floor(ADC_main_tap*SCALE);
			}
			else if(control_mode_Local_Remote == 0x00)
			{
				Global_set_actual_tap = *MDB_REG_1064;

			}
			else{}

		}
		if(Global_set_actual_tap>=(NUMBER_OF_TAPS-1)) Global_set_actual_tap = NUMBER_OF_TAPS-1;
		else if(Global_set_actual_tap<=0) Global_set_actual_tap = 0;



		//PID calibration
		PID_Voltage_PLL.KP = PID_1_KP;
		PID_Voltage_PLL.KI = PID_1_KI;
		PID_Voltage_PLL.KD = PID_1_KD;

		//******************************************
		//DIAGNOSTIC AND CONTROL DATA EXCHANGE WITH CM4
		//******************************************
		*MDB_REG_1000 = Global_set_actual_tap;
		*MDB_REG_1001 = actual_tap[PHASE_A];
		*MDB_REG_1002 = actual_tap[PHASE_B];
		*MDB_REG_1003 = actual_tap[PHASE_C];
		*MDB_REG_1004 = (unsigned int)(100/current_RMS[PHASE_A]);
		*MDB_REG_1005 = (unsigned int)(100/current_RMS[PHASE_B]);
		*MDB_REG_1006 = (unsigned int)(100/current_RMS[PHASE_C]);
		*MDB_REG_1007 = (unsigned int)(100/voltage_RMS[PHASE_A]);
		*MDB_REG_1008 = (unsigned int)(100/voltage_RMS[PHASE_B]);
		*MDB_REG_1009 = (unsigned int)(100/voltage_RMS[PHASE_C]);
		*MDB_REG_1010 = Loadtype[PHASE_A];
		*MDB_REG_1011 = Loadtype[PHASE_B];
		*MDB_REG_1012 = Loadtype[PHASE_C];
		*MDB_REG_1013 = (unsigned int)(100*dq_PLL_U.x);
		*MDB_REG_1014 = (unsigned int)(100*dq_PLL_U.y);
		*MDB_REG_1015 = (unsigned int)(100*dq_PLL_I.x);
		*MDB_REG_1016 = (unsigned int)(100*dq_PLL_I.y);
		*MDB_REG_1017 = (unsigned int)(100*q_error);
		*MDB_REG_1018 = tap_delta;
		*MDB_REG_1019 = 0x4D42; //MB
		*MDB_REG_1020 = cycles_master_task;
		*MDB_REG_1021 = cycles_base_algo_task;

		*MDB_REG_1022 = app_time.seconds;
		*MDB_REG_1023 = app_time.minutes;

		//cosinus fi counting
		if(fabs(voltage_phase[PHASE_A]-current_phase[PHASE_A])<90) delta_fi[PHASE_A] = DEFREE_TO_RAD_CONV*(voltage_phase[PHASE_A]-current_phase[PHASE_A]);
		*MDB_REG_1029 = (unsigned int)(100*delta_fi[PHASE_A]);
		if(fabs(voltage_phase[PHASE_B]-current_phase[PHASE_B])<90) delta_fi[PHASE_B] = DEFREE_TO_RAD_CONV*(voltage_phase[PHASE_B]-current_phase[PHASE_B]);
		*MDB_REG_1033 = (unsigned int)(100*delta_fi[PHASE_B]);
		if(fabs(voltage_phase[PHASE_C]-current_phase[PHASE_C])<90) delta_fi[PHASE_C] = DEFREE_TO_RAD_CONV*(voltage_phase[PHASE_C]-current_phase[PHASE_C]);
		*MDB_REG_1037 = (unsigned int)(100*delta_fi[PHASE_C]);

		*MDB_REG_1041 = state_machine[PHASE_A];
		*MDB_REG_1042 = state_machine[PHASE_B];
		*MDB_REG_1043 = state_machine[PHASE_C];
		*MDB_REG_1044 = UART_received_frame_1[0];
		*MDB_REG_1045 = UART_received_frame_2[0];
		*MDB_REG_1046 = UART_received_frame_3[0];
		*MDB_REG_1047 = error_status.byte_16t;
		*MDB_REG_1048 = failed_thyristor_ignition;
		*MDB_REG_1049 = failed_thyristor_ignition_freeze;
		*MDB_REG_1050 = (UART_received_frame_1[1]<<8) + UART_received_frame_1[2];
		*MDB_REG_1051 = (UART_received_frame_1[3]<<8) + UART_received_frame_1[4];
		*MDB_REG_1052 = (UART_received_frame_2[1]<<8) + UART_received_frame_2[2];
		*MDB_REG_1053 = (UART_received_frame_2[3]<<8) + UART_received_frame_2[4];
		*MDB_REG_1054 = (UART_received_frame_3[1]<<8) + UART_received_frame_3[2];
		*MDB_REG_1055 = (UART_received_frame_3[3]<<8) + UART_received_frame_3[4];

		//Write registers
		MVS_voltage_set = (float)(0.01*(*MDB_REG_1061));
		if(MVS_voltage_set > 280.0) MVS_voltage_set = 280.0;
		else if(MVS_voltage_set < 190.0) MVS_voltage_set = 190.0;
		control_mode_Local_Remote = *MDB_REG_1062;
		control_mode_Automatic_Manual = *MDB_REG_1063;
		Tap_change_type = (enum change_types)(*MDB_REG_1065);
		if(!Tap_change_type) Tap_change_type = step;
		else Tap_change_type = jump;
		Master_Voltage_Control.KP = (float)(0.01*(*MDB_REG_1066));
		Master_Voltage_Control.KI = (float)(0.01*(*MDB_REG_1067));
		Master_Voltage_Control.KD = (float)(0.01*(*MDB_REG_1068));

		//*************************************************************************************************//
		//DIAGNOSTICS - ERRORS STATUS
		//*************************************************************************************************//
		if(UART_received_frame_1[0] == 0x00) error_status.b_bit.UART_error = 0x1;
		else error_status.b_bit.UART_error = 0x0;

		if((actual_tap[PHASE_A] != set_actual_tap[PHASE_A])||(actual_tap[PHASE_B] != set_actual_tap[PHASE_B])||(actual_tap[PHASE_C] != set_actual_tap[PHASE_C]) )
		{
			error_status.b_bit.switchover_process_ongoing = 1;
		}
		else error_status.b_bit.switchover_process_ongoing = 0;

		if(dq_PLL_I.y > 0.1)
		{
			error_status.b_bit.grid_transient_state = 1;
		}
		else error_status.b_bit.grid_transient_state = 0;

		if((actual_tap[PHASE_A] != actual_tap[PHASE_B])||(actual_tap[PHASE_A] != actual_tap[PHASE_C])||(actual_tap[PHASE_B] != actual_tap[PHASE_C]))
		{
			error_status.b_bit.taps_in_phases_not_equal = 1;
		}
		else error_status.b_bit.taps_in_phases_not_equal = 0;

		if((current_RMS[PHASE_A] < NO_LOAD_STATE)||(current_RMS[PHASE_B] < NO_LOAD_STATE)||(current_RMS[PHASE_C] < NO_LOAD_STATE))
		{
			error_status.b_bit.current_idle_state = 1;
		}
		else error_status.b_bit.current_idle_state = 0;

		if((actual_tap[PHASE_A] >= (NUMBER_OF_TAPS-1))||(actual_tap[PHASE_B] >= (NUMBER_OF_TAPS-1))||(actual_tap[PHASE_B] >= (NUMBER_OF_TAPS-1)))
		{
			error_status.b_bit.tap_max = 1;
		}
		else error_status.b_bit.tap_max = 0;

		if((actual_tap[PHASE_A] <= 0)||(actual_tap[PHASE_B] <= 0)||(actual_tap[PHASE_C] <= 0))
		{
			error_status.b_bit.tap_min = 1;
		}
		else error_status.b_bit.tap_min = 0;

		if((Loadtype[PHASE_A] == Resistive)||(Loadtype[PHASE_B] == Resistive)||(Loadtype[PHASE_C] == Resistive))
		{
			error_status.b_bit.resistive_load_warning = 1;
		}
		else error_status.b_bit.resistive_load_warning = 0;

		if((current_RMS[PHASE_A] > OVERCURRENT)||(current_RMS[PHASE_B] > OVERCURRENT)||(current_RMS[PHASE_C] > OVERCURRENT))
		{
			error_status.b_bit.current_idle_state = 1;
		}
		else error_status.b_bit.current_idle_state = 0;

		error_status.b_bit.step_or_jump_mode = Tap_change_type;
		error_status.b_bit.diagnosis_task_done = 1;
		error_status.b_bit.automatic_or_manual = control_mode_Automatic_Manual;
		error_status.b_bit.local_or_remote = control_mode_Local_Remote;

		//Time counting
		cycles_master_task++;
		app_time.cnt++;
		if(app_time.cnt == 10)
		{
			app_time.seconds++;
			app_time.cnt = 0;
		}
		if(app_time.seconds == 60)
		{
			app_time.minutes++;
			app_time.seconds = 0;
		}


		//LED for indicating that the programs is running and there is no timeout.
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	}
	//***************************************************************************************************
	//***************************************************************************************************
	//***************************************************************************************************
	// 10 kHz frequency timer
	//Base algorithm
	if(htim->Instance == TIM16)
	{
		if(Transformer_ON_OFF)
		{
		cycles_base_algo_task++;
		error_status.b_bit.base_task_done = 0;
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

		actual_current[PHASE_A] = 100*sin_f(theta_I_emulated[0]);
		actual_current[PHASE_B] = 117*sin_f(theta_I_emulated[1]);
		actual_current[PHASE_C] = 83*sin_f(theta_I_emulated[2]);
		//actual_current[2] = ADC_raw_current[2] *
		actual_voltage[PHASE_A] = 311*sin_f(theta_U_emulated[0]);
		actual_voltage[PHASE_B] = 312*sin_f(theta_U_emulated[1]);
		//actual_voltage[2] = 312*sin_f(theta_U_emulated[2]);
		actual_voltage[PHASE_C] = -actual_voltage[0] - actual_voltage[1];
		#ifdef EMULATION_SIMPLE
		voltage_phase_emulated[0] =  voltage_phase_emulated[0] + 360*50*0.0001;
		current_phase_emulated[0]=  current_phase_emulated[0] + 360*50*0.0001;
		if(voltage_phase_emulated[0]>360) voltage_phase_emulated[0] = voltage_phase_emulated[0] - 360;
		if(current_phase_emulated[0]>360) current_phase_emulated[0] = current_phase_emulated[0] - 360;
		voltage_phase[0] = voltage_phase_emulated[0]; // input 1 must be voltage
		current_phase[0] = current_phase_emulated[0]; // input 2 must be current
		current_RMS[0] = 50;
		#endif
		error_status.b_bit.measurement_mode = 0;

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
		error_status.b_bit.measurement_mode = 1;
		#endif
		//*************ONE PHASE PLL - CURRENTS****************
		I_PLL_1p.phase_A = PLL_1phase(actual_current[PHASE_A], current_RMS[PHASE_A], &PLL_current_phase1);
		I_PLL_1p.phase_B = PLL_1phase(actual_current[PHASE_B], current_RMS[PHASE_B], &PLL_current_phase2);
		I_PLL_1p.phase_C = PLL_1phase(actual_current[PHASE_C], current_RMS[PHASE_C], &PLL_current_phase3);

		//*************THREE PHASE PLL - VOLTAGES****************
		alfabeta_PLL_U = AlfaBeta(actual_voltage[0], actual_voltage[1], actual_voltage[2]);
		dq_PLL_U = DQ(alfabeta_PLL_U.x, alfabeta_PLL_U.y, PLL_theta_U);
		PID_PLL_U_output = PID_Regulator(dq_PLL_U.x, &PID_Voltage_PLL);
		PLL_theta_U = Integrator(PID_PLL_U_output, -2*PI, 2*PI, 1);
		U_PLL = PLL_3phase(PLL_theta_U);
		//*************THREE PHASE PLL - Currents - Grid stability****************
		q_error = 0;
		//**************Phases assigned for algorithm
		voltage_phase[PHASE_A] = U_PLL.phase_A * RAD_TO_DEGREE_CONV;
		current_phase[PHASE_A] = I_PLL_1p.phase_A * RAD_TO_DEGREE_CONV;
		voltage_phase[PHASE_B] = U_PLL.phase_B * RAD_TO_DEGREE_CONV;
		current_phase[PHASE_B] = I_PLL_1p.phase_B * RAD_TO_DEGREE_CONV;
		voltage_phase[PHASE_C] = U_PLL.phase_C * RAD_TO_DEGREE_CONV;
		current_phase[PHASE_C] = I_PLL_1p.phase_C * RAD_TO_DEGREE_CONV;
		//*************************************************************************************************//
		//MEASUREMENTS AND PLL ALGORITHM
		//*************************************************************************************************//


		//*************************************************************************************************//
		//MAIN ALGORITHM - 3 phases
		//*************************************************************************************************//
		for (int phase = 0; phase <= 2; phase++)
		        {
					//PROTECTIONS
					//************************************
		            tap_delta = Tap_diff(set_actual_tap[phase], actual_tap[phase], Tap_change_type);

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
		uart_send_frame[0] = UART_frame_tap_info[0].byte;
		//uart_send[1] = UART_frame_tap_info[1].byte;
		//uart_send[2] = UART_frame_tap_info[2].byte;
		}
		else uart_send_frame[0] = 0b01000000;
		//Data send to slave described in data sheet
		//bit 7,6 -> SLAVE address (1 or 2 or 3 -> 0b01, 0b10, 0b11)
		//bit 5,4 -> Actual thyristor masking, 11 - down and up on, 10 - down on / up off etc.
		//bit 3:0 -> actual tap number to turn on thyristor (bit mask is put later) (from 0 to 15)

		HAL_UART_Transmit_IT(&huart6, &uart_send_frame[0], 1);
		HAL_UART_Receive_IT(&huart6, &UART_received_frame_1, 5);
		//HAL_UART_Transmit_IT(&huart1, &uart_send_frame[1], 1);
		//HAL_UART_Receive_IT(&huart1, &UART_received_frame[1], 1);
		//HAL_UART_Transmit_IT(&huart2, &uart_send_frame[2], 1);
		//HAL_UART_Receive_IT(&huart2, &UART_received_frame[2], 1);

		/*********************************************************************************************************************************/
		//END OF ALGORITHM
		/*********************************************************************************************************************************/
		error_status.b_bit.base_task_done = 1;

	}
	//Calculating RMS with 1kHz frequency
	//Dividing harshly exhaust micro controller, so it was put in interupt with lower frequency
	if(htim->Instance == TIM17)
	{
		//***************CURRENT RMS***************//

		//Actually it is 1/rms. In debugging it need to write 1/"debugging value" to see actual RMS.
		current_RMS[PHASE_A] = RMS(actual_current[PHASE_A], &PLL_current_phase1);
		current_RMS[PHASE_B] = RMS(actual_current[PHASE_B], &PLL_current_phase2);
		current_RMS[PHASE_C] = RMS(actual_current[PHASE_C], &PLL_current_phase3);

		voltage_RMS[PHASE_A] = RMS(actual_voltage[PHASE_A], &PLL_voltage_phase1);
		voltage_RMS[PHASE_B] = RMS(actual_voltage[PHASE_B], &PLL_voltage_phase2);
		voltage_RMS[PHASE_C] = RMS(actual_voltage[PHASE_C], &PLL_voltage_phase3);

	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		if((UART_received_frame_1[0]-0x01) != UART_frame_tap_info[0].byte_8.actual_tap)
			{
			failed_thyristor_ignition =+ 1;
			failed_thyristor_ignition_freeze =+ 1;
			error_status.b_bit.failed_thyristor_switching = 1;
			}
		else
			{
			failed_thyristor_ignition = 0;
			error_status.b_bit.failed_thyristor_switching = 0;
			}
		//HAL_UART_Receive_IT(&huart6, &UART_received_frame_1, 5);
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
