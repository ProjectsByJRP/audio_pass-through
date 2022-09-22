/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_RED_Pin|LED_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB8 PB6 PB3 PB9
                           PB7 PB4 PB5 PB15
                           PB14 PB13 PB1 PB12
                           PB10 PB2 PB11 PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_6|GPIO_PIN_3|GPIO_PIN_9
                          |GPIO_PIN_7|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15
                          |GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_12
                          |GPIO_PIN_10|GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG9 PG13 PG14
                           PG10 PG12 PG6 PG5
                           PG8 PG7 PG4 PG2
                           PG3 PG1 PG0 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_4|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD1 PD7 PD5
                           PD2 PD4 PD0 PD6
                           PD14 PD13 PD15 PD11
                           PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_5
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_0|GPIO_PIN_6
                          |GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA15 PA8 PA12 PA10
                           PA11 PA9 PA3 PA7
                           PA0 PA1 PA4 PA5
                           PA2 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_3|GPIO_PIN_7
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE3 PE0 PE2
                           PE1 PE5 PE14 PE8
                           PE7 PE12 PE13 PE15
                           PE10 PE9 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_2
                          |GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_14|GPIO_PIN_8
                          |GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15
                          |GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC12 PC11 PC10 PC8
                           PC7 PC9 PC6 PC0
                           PC1 PC5 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_8
                          |GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH14 PH13 PH15 PH7
                           PH11 PH9 PH2 PH8
                           PH10 PH12 PH4 PH3
                           PH5 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_7
                          |GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_2|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_3
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF1 PF0 PF2 PF4
                           PF5 PF3 PF10 PF11
                           PF13 PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_13|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DISP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
