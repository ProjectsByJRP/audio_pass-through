/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define BUTTON1_Pin GPIO_PIN_13
#define BUTTON1_GPIO_Port GPIOC
#define LCD_BL_CTRL_Pin GPIO_PIN_15
#define LCD_BL_CTRL_GPIO_Port GPIOG
#define SAI1_SD_A_Pin GPIO_PIN_6
#define SAI1_SD_A_GPIO_Port GPIOE
#define SAI1_SD_B_Pin GPIO_PIN_6
#define SAI1_SD_B_GPIO_Port GPIOF
#define SAI1_SCK_B_Pin GPIO_PIN_8
#define SAI1_SCK_B_GPIO_Port GPIOF
#define SAI1_MCLK_B_Pin GPIO_PIN_7
#define SAI1_MCLK_B_GPIO_Port GPIOF
#define SAI1_FS_B_Pin GPIO_PIN_9
#define SAI1_FS_B_GPIO_Port GPIOF
#define T_VCP_RX_Pin GPIO_PIN_9
#define T_VCP_RX_GPIO_Port GPIOD
#define LCD_DISP_Pin GPIO_PIN_10
#define LCD_DISP_GPIO_Port GPIOD
#define T_VCP_TX_Pin GPIO_PIN_8
#define T_VCP_TX_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_3
#define LED_GREEN_GPIO_Port GPIOC
#define I2C4_SDA_Pin GPIO_PIN_15
#define I2C4_SDA_GPIO_Port GPIOF
#define LCD_RST_Pin GPIO_PIN_6
#define LCD_RST_GPIO_Port GPIOH
#define I2C4_SCL_Pin GPIO_PIN_14
#define I2C4_SCL_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
