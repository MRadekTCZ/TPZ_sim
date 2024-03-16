/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TAP10_DOWN_Pin|TAP10_UP_Pin|TAP11_DOWN_Pin|TAP11_UP_Pin
                          |TAP12_DOWN_Pin|TAP12_UP_Pin|TAP13_DOWN_Pin|TAP13_UP_Pin
                          |TAP14_DOWN_Pin|TAP14_UP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TAP0_DOWN_Pin|LD2_Pin|TAP0_UP_Pin|TAP1_DOWN_Pin
                          |TAP1_UP_Pin|TAP4_DOWN_Pin|TAP15_DOWN_Pin|TAP2_DOWN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TAP2_UP_Pin|TAP3_DOWN_Pin|TAP3_UP_Pin|TAP7_UP_Pin
                          |TAP8_DOWN_Pin|TAP8_UP_Pin|TAP9_DOWN_Pin|TAP9_UP_Pin
                          |TAP4_UP_Pin|TAP5_DOWN_Pin|TAP5_UP_Pin|TAP6_DOWN_Pin
                          |TAP6_UP_Pin|TAP7_DOWN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TAP15_UP_GPIO_Port, TAP15_UP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin PCPin PCPin PCPin
                           PCPin PCPin */
  GPIO_InitStruct.Pin = TAP10_DOWN_Pin|TAP10_UP_Pin|TAP11_DOWN_Pin|TAP11_UP_Pin
                          |TAP12_DOWN_Pin|TAP12_UP_Pin|TAP13_DOWN_Pin|TAP13_UP_Pin
                          |TAP14_DOWN_Pin|TAP14_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                           PAPin PAPin */
  GPIO_InitStruct.Pin = TAP0_DOWN_Pin|LD2_Pin|TAP0_UP_Pin|TAP1_DOWN_Pin
                          |TAP1_UP_Pin|TAP2_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin PBPin */
  GPIO_InitStruct.Pin = TAP2_UP_Pin|TAP3_DOWN_Pin|TAP3_UP_Pin|TAP7_UP_Pin
                          |TAP8_DOWN_Pin|TAP8_UP_Pin|TAP9_DOWN_Pin|TAP9_UP_Pin
                          |TAP4_UP_Pin|TAP5_DOWN_Pin|TAP5_UP_Pin|TAP6_DOWN_Pin
                          |TAP6_UP_Pin|TAP7_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = TAP4_DOWN_Pin|TAP15_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = TAP15_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TAP15_UP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
void switch_off_all_taps(void)
{
	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, TAP10_DOWN_Pin|TAP10_UP_Pin|TAP11_DOWN_Pin|TAP11_UP_Pin
	                          |TAP12_DOWN_Pin|TAP12_UP_Pin|TAP13_DOWN_Pin|TAP13_UP_Pin
	                          |TAP14_DOWN_Pin|TAP14_UP_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, TAP0_DOWN_Pin|TAP0_UP_Pin|TAP1_DOWN_Pin
	                          |TAP1_UP_Pin|TAP4_DOWN_Pin|TAP15_DOWN_Pin|TAP2_DOWN_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, TAP2_UP_Pin|TAP3_DOWN_Pin|TAP3_UP_Pin|TAP7_UP_Pin
	                          |TAP8_DOWN_Pin|TAP8_UP_Pin|TAP9_DOWN_Pin|TAP9_UP_Pin
	                          |TAP4_UP_Pin|TAP5_DOWN_Pin|TAP5_UP_Pin|TAP6_DOWN_Pin
	                          |TAP6_UP_Pin|TAP7_DOWN_Pin, GPIO_PIN_RESET);


}
void GPIO_Test_routine(void)
{


for(int i=0;i<16;i++)
{
	switch_off_all_taps();
	HAL_GPIO_TogglePin(GPIOA,LD2_Pin);
	switch(i)
		  {
		  case 0:
			  HAL_GPIO_WritePin(GPIOA, TAP0_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, TAP0_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 1:
			  HAL_GPIO_WritePin(GPIOA, TAP1_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, TAP1_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 2:
			  HAL_GPIO_WritePin(GPIOA, TAP2_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, TAP2_UP_Pin, GPIO_PIN_SET);
			  ;
			  break;
		  case 3:
			  HAL_GPIO_WritePin(GPIOB, TAP3_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, TAP3_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 4:
			  HAL_GPIO_WritePin(GPIOA, TAP4_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, TAP4_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 5:
			  HAL_GPIO_WritePin(GPIOB, TAP5_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, TAP5_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 6:
			  HAL_GPIO_WritePin(GPIOB, TAP6_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, TAP6_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 7:
			  HAL_GPIO_WritePin(GPIOB, TAP7_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, TAP7_UP_Pin, GPIO_PIN_SET) ;
			  break;
		  case 8:
			  HAL_GPIO_WritePin(GPIOA, TAP8_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, TAP8_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 9:
			  HAL_GPIO_WritePin(GPIOA, TAP9_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, TAP9_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 10:
			  HAL_GPIO_WritePin(GPIOC, TAP10_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, TAP10_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 11:
			  HAL_GPIO_WritePin(GPIOC, TAP11_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, TAP11_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 12:
			  HAL_GPIO_WritePin(GPIOC, TAP12_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, TAP12_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 13:
			  HAL_GPIO_WritePin(GPIOC, TAP13_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, TAP13_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 14:
			  HAL_GPIO_WritePin(GPIOC, TAP14_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, TAP14_UP_Pin, GPIO_PIN_SET);
			  break;
		  case 15:
			  HAL_GPIO_WritePin(GPIOC, TAP15_DOWN_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOD, TAP15_UP_Pin, GPIO_PIN_SET);

			  break;
		  }
	HAL_Delay(70);

}
switch_off_all_taps();
}

/* USER CODE END 2 */
