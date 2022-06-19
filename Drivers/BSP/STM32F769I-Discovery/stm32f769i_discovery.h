/**
  ******************************************************************************
  * @file    stm32f769i_discovery.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    30-December-2016
  * @brief   This file contains definitions for STM32F769I-Discovery LEDs,
  *          push-buttons hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F769I_DISCOVERY_H
#define __STM32F769I_DISCOVERY_H

#ifdef __cplusplus
 extern "C" {
#endif


 /* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

#if !defined (USE_STM32F769I_DISCO)
 #define USE_STM32F769I_DISCO
#endif

/** @brief Led_TypeDef
  *  STM32F769I_DISCOVERY board leds definitions.
  */
typedef enum
{
 LED1 = 0,
 LED_RED = LED1,
 LED2 = 1,
 LED_GREEN = LED2
} Led_TypeDef;


/* Always four leds for all revisions of Discovery boards */
#define LEDn                             ((uint8_t)2)


/* 2 Leds are connected to MCU directly on PJ13 and PJ5 */
#define LED1_GPIO_PORT                   ((GPIO_TypeDef*)GPIOJ)
#define LED2_GPIO_PORT                   ((GPIO_TypeDef*)GPIOJ)

#define LEDx_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOJ_CLK_ENABLE()

#define LED1_PIN                         ((uint32_t)GPIO_PIN_13)
#define LED2_PIN                         ((uint32_t)GPIO_PIN_5)

#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)


/**
  * @brief User can use this section to tailor I2C4/I2C4 instance used and associated
  * resources (audio codec).
  * Definition for I2C4 clock resources
  */
#define DISCOVERY_AUDIO_I2Cx                             I2C4
#define DISCOVERY_AUDIO_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C4_CLK_ENABLE()
#define DISCOVERY_AUDIO_I2Cx_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()
#define DISCOVERY_AUDIO_I2Cx_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define DISCOVERY_AUDIO_I2Cx_FORCE_RESET()               __HAL_RCC_I2C4_FORCE_RESET()
#define DISCOVERY_AUDIO_I2Cx_RELEASE_RESET()             __HAL_RCC_I2C4_RELEASE_RESET()

/** @brief Definition for I2C4 Pins
  */
#define DISCOVERY_AUDIO_I2Cx_SCL_PIN                     GPIO_PIN_12 /*!< PD12 */
#define DISCOVERY_AUDIO_I2Cx_SCL_AF                      GPIO_AF4_I2C4
#define DISCOVERY_AUDIO_I2Cx_SCL_GPIO_PORT               GPIOD
#define DISCOVERY_AUDIO_I2Cx_SDA_PIN                     GPIO_PIN_7 /*!< PB7 */
#define DISCOVERY_AUDIO_I2Cx_SDA_AF                      GPIO_AF11_I2C4
#define DISCOVERY_AUDIO_I2Cx_SDA_GPIO_PORT               GPIOB
/** @brief Definition of I2C4 interrupt requests
  */
#define DISCOVERY_AUDIO_I2Cx_EV_IRQn                     I2C4_EV_IRQn
#define DISCOVERY_AUDIO_I2Cx_ER_IRQn                     I2C4_ER_IRQn

/**
  * @brief User can use this section to tailor I2C1/I2C1 instance used and associated
  * resources.
  * Definition for I2C1 clock resources
  */
#define DISCOVERY_EXT_I2Cx                             I2C1
#define DISCOVERY_EXT_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

#define DISCOVERY_EXT_I2Cx_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define DISCOVERY_EXT_I2Cx_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

/** @brief Definition for I2C1 Pins
  */
#define DISCOVERY_EXT_I2Cx_SCL_PIN                     GPIO_PIN_8 /*!< PB8 */
#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_PORT           GPIOB
#define DISCOVERY_EXT_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C1
#define DISCOVERY_EXT_I2Cx_SDA_PIN                     GPIO_PIN_9 /*!< PB9 */

/** @brief Definition of I2C interrupt requests
  */
#define DISCOVERY_EXT_I2Cx_EV_IRQn                     I2C1_EV_IRQn
#define DISCOVERY_EXT_I2Cx_ER_IRQn                     I2C1_ER_IRQn

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated from APB1 source clock = 50 MHz */
/* Due to the big MOFSET capacity for adapting the camera level the rising time is very large (>1us) */
/* 0x40912732 takes in account the big rising and aims a clock of 100khz */
#ifndef DISCOVERY_I2Cx_TIMING  
#define DISCOVERY_I2Cx_TIMING                      ((uint32_t)0x40912732)
#endif /* DISCOVERY_I2Cx_TIMING */


/**
  * @}
  */

/** @defgroup STM32F769I_DISCOVERY_LOW_LEVEL_Exported_Macros STM32F769I Discovery Low Level Exported Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32F769I_DISCOVERY_LOW_LEVEL_Exported_Functions STM32F769I Discovery Low Level Exported Functions
  * @{
  */
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* __STM32F769I_DISCOVERY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
