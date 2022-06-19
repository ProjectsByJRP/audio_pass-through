/**
  ******************************************************************************
  * @file    stm32f769i_discovery.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    30-December-2016
  * @brief   This file provides a set of firmware functions to manage LEDs,
  *          push-buttons, external SDRAM, external QSPI Flash, RF EEPROM,
  *          available on STM32F769I-Discovery board (MB1225) from 
  *          STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f769i_discovery.h"


/** @defgroup STM32F769I_DISCOVERY_LOW_LEVEL_Private_Variables LOW_LEVEL Private Variables
  * @{
  */
uint32_t GPIO_PIN[LEDn] = {LED1_PIN,
                           LED2_PIN};

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT,
                                 LED2_GPIO_PORT};

static I2C_HandleTypeDef hI2cAudioHandler = {0};

/**
  * @}
  */

/** @defgroup STM32F769I_DISCOVERY_LOW_LEVEL_Private_FunctionPrototypes LOW_LEVEL Private FunctionPrototypes
  * @{
  */
static void     I2Cx_MspInit(I2C_HandleTypeDef *i2c_handler);
static void     I2Cx_Init(I2C_HandleTypeDef *i2c_handler);

/* AUDIO IO functions */
void            AUDIO_IO_Init(void);
void            AUDIO_IO_DeInit(void);
void            AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
uint16_t        AUDIO_IO_Read(uint8_t Addr, uint16_t Reg);
void            AUDIO_IO_Delay(uint32_t Delay);

/**
  * @brief  Configures LED GPIO.
  * @param  Led: LED to be configured.
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;
  
  LEDx_GPIO_CLK_ENABLE();
  /* Configure the GPIO_LED pin */
  gpio_init_structure.Pin   = GPIO_PIN[Led];
  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull  = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &gpio_init_structure);

}


/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be configured.
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  * @note Led DeInit does not disable the GPIO clock
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;
  
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = GPIO_PIN[Led];
  
  /* Turn off LED */
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  HAL_GPIO_DeInit(GPIO_PORT[Led], gpio_init_structure.Pin);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: LED to be set on
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: LED to be set off
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: LED to be toggled
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}


/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* I2C Routines *********************************/
/**
  * @brief  Initializes I2C MSP.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
static void I2Cx_MspInit(I2C_HandleTypeDef *i2c_handler)
{
  GPIO_InitTypeDef  gpio_init_structure;

  if (i2c_handler == (I2C_HandleTypeDef*)(&hI2cAudioHandler))
  {
  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  DISCOVERY_AUDIO_I2Cx_SCL_GPIO_CLK_ENABLE();
  DISCOVERY_AUDIO_I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Configure I2C Tx as alternate function */
  gpio_init_structure.Pin = DISCOVERY_AUDIO_I2Cx_SCL_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_OD;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Alternate = DISCOVERY_AUDIO_I2Cx_SCL_AF;
  HAL_GPIO_Init(DISCOVERY_AUDIO_I2Cx_SCL_GPIO_PORT, &gpio_init_structure);

  /* Configure I2C Rx as alternate function */
  gpio_init_structure.Pin = DISCOVERY_AUDIO_I2Cx_SDA_PIN;
  gpio_init_structure.Alternate = DISCOVERY_AUDIO_I2Cx_SDA_AF;
  HAL_GPIO_Init(DISCOVERY_AUDIO_I2Cx_SDA_GPIO_PORT, &gpio_init_structure);

  /*** Configure the I2C peripheral ***/
  /* Enable I2C clock */
  DISCOVERY_AUDIO_I2Cx_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  DISCOVERY_AUDIO_I2Cx_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  DISCOVERY_AUDIO_I2Cx_RELEASE_RESET();

  /* Enable and set I2C1 Interrupt to a lower priority */
  HAL_NVIC_SetPriority(DISCOVERY_AUDIO_I2Cx_EV_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_AUDIO_I2Cx_EV_IRQn);

  /* Enable and set I2C1 Interrupt to a lower priority */
  HAL_NVIC_SetPriority(DISCOVERY_AUDIO_I2Cx_ER_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_AUDIO_I2Cx_ER_IRQn);    
    
  }
  else
  {
  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  /* Configure I2C Tx as alternate function */
  gpio_init_structure.Pin = DISCOVERY_EXT_I2Cx_SCL_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_OD;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Alternate = DISCOVERY_EXT_I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

  /* Configure I2C Rx as alternate function */
  gpio_init_structure.Pin = DISCOVERY_EXT_I2Cx_SDA_PIN;
  HAL_GPIO_Init(DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

  /*** Configure the I2C peripheral ***/
  /* Enable I2C clock */
  DISCOVERY_EXT_I2Cx_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  DISCOVERY_EXT_I2Cx_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  DISCOVERY_EXT_I2Cx_RELEASE_RESET();

  /* Enable and set I2C1 Interrupt to a lower priority */
  HAL_NVIC_SetPriority(DISCOVERY_EXT_I2Cx_EV_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_EXT_I2Cx_EV_IRQn);

  /* Enable and set I2C1 Interrupt to a lower priority */
  HAL_NVIC_SetPriority(DISCOVERY_EXT_I2Cx_ER_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_EXT_I2Cx_ER_IRQn);
  }
}

/**
  * @brief  Initializes I2C HAL.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
static void I2Cx_Init(I2C_HandleTypeDef *i2c_handler)
{
  if(HAL_I2C_GetState(i2c_handler) == HAL_I2C_STATE_RESET)
  {
    if (i2c_handler == (I2C_HandleTypeDef*)(&hI2cAudioHandler))
    {
      /* Audio and LCD I2C configuration */
      i2c_handler->Instance = DISCOVERY_AUDIO_I2Cx;
    }
    else
    {
      /* External, camera and Arduino connector  I2C configuration */
      i2c_handler->Instance = DISCOVERY_EXT_I2Cx;
    }
    i2c_handler->Init.Timing           = DISCOVERY_I2Cx_TIMING;
    i2c_handler->Init.OwnAddress1      = 0;
    i2c_handler->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    i2c_handler->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    i2c_handler->Init.OwnAddress2      = 0;
    i2c_handler->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    i2c_handler->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
    I2Cx_MspInit(i2c_handler);
    HAL_I2C_Init(i2c_handler);
  }
}


/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK AUDIO *********************************/

/**
  * @brief  Initializes Audio low level.
  */
void AUDIO_IO_Init(void)
{
  I2Cx_Init(&hI2cAudioHandler);
}

/**
  * @brief  DeInitializes Audio low level.
  */
void AUDIO_IO_DeInit(void)
{

}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
void AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
  uint16_t tmp = Value;
  
  Value = ((uint16_t)(tmp >> 8) & 0x00FF);
  
  Value |= ((uint16_t)(tmp << 8)& 0xFF00);


 // I2Cx_WriteMultiple(&hI2cAudioHandler, Addr, Reg, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Value, 2);
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&hI2cAudioHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *) &Value, 2, 1000);

    /* Check the communication status */
    if(status != HAL_OK)
    {
        /* Re-Initiaize the I2C Bus */
        /* De-initialize the I2C communication bus */
        HAL_I2C_DeInit(&hI2cAudioHandler);

        /* Re-Initialize the I2C communication bus */
        I2Cx_Init(&hI2cAudioHandler);
    }

}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @retval Data to be read
  */
uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg)
{
  uint16_t read_value = 0, tmp = 0;
  
//  I2Cx_ReadMultiple(&hI2cAudioHandler, Addr, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&read_value, 2);
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hI2cAudioHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *) &read_value, 2, 1000);

    /* Check the communication status */
    if(status != HAL_OK)
    {
        /* I2C error occured */
        //I2Cx_Error(&hI2cAudioHandler, Addr);
        /* De-initialize the I2C communication bus */
        HAL_I2C_DeInit(&hI2cAudioHandler);

        /* Re-Initialize the I2C communication bus */
        I2Cx_Init(&hI2cAudioHandler);

    } else {

        tmp = ((uint16_t) (read_value >> 8) & 0x00FF);

        tmp |= ((uint16_t) (read_value << 8) & 0xFF00);

        read_value = tmp;
    }
  return read_value;
}

/**
  * @brief  AUDIO Codec delay
  * @param  Delay: Delay in ms
  */
void AUDIO_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
