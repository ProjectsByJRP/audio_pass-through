/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
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
#include "main.h"
#include "stm32f7xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/**
 * this software builds heavily on the work by https://community.st.com/people/Beaulier.Francois
 **/


#include "../Drivers/BSP/Components/wm8994/wm8994.h"
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


typedef enum {
    BUFFER_OFFSET_NONE = 0,
    BUFFER_OFFSET_HALF = 1,
    BUFFER_OFFSET_FULL = 2,
} BUFFER_StateTypeDef;

#define RECORD_BUFFER_SIZE  4096

volatile uint32_t audio_rec_buffer_state;

/* Buffer containing the PCM samples coming from the microphone */
int16_t RecordBuffer[RECORD_BUFFER_SIZE];

/* Buffer used to stream the recorded PCM samples towards the audio codec. */
int16_t PlaybackBuffer[RECORD_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void CopyBuffer(int16_t *pbuffer1, int16_t *pbuffer2, uint16_t BufferSize);

void BSP_AUDIO_OUT_ClockConfig(uint32_t AudioFreq);

void get_max_val(int16_t *buf, uint32_t size, int16_t amp[]);

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C4_Init();
  MX_SAI1_Init();

  /* USER CODE BEGIN 2 */
    printf("Connected to STM32F769I-Discovery USART 1\r\n");
    printf("\r\n");

    {
        int16_t amp[4];

        /* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */
        BSP_AUDIO_OUT_ClockConfig(SAI_AUDIO_FREQUENCY_44K);
        HAL_SAI_Init(&hsai_BlockA1); // Update internal MCO dividers to match new clock
        HAL_SAI_Init(&hsai_BlockB1); // Update internal MCO dividers to match new clock


        /* Enable SAI peripheral to generate MCLK (required to start talking to codec?) */
        __HAL_SAI_ENABLE(&hsai_BlockA1);

        /* Initialize Audio Recorder with 4 channels to be used */
        if ((wm8994_drv.ReadID(AUDIO_I2C_ADDRESS)) == WM8994_ID) {
            /* Reset the Codec Registers */
            wm8994_drv.Reset(AUDIO_I2C_ADDRESS);
            /* Initialize the audio driver structure */
            printf("Audio I/O initialization OK\r\n");
        } else {
            printf("Audio I/O initialization failed.\r\n");
            Error_Handler();
        }

        /* Initialize the codec internal registers */
        wm8994_drv.Init(AUDIO_I2C_ADDRESS,
                        INPUT_DEVICE_INPUT_LINE_1/*INPUT_DEVICE_ANALOG_MIC*/ | OUTPUT_DEVICE_HEADPHONE, 100, SAI_AUDIO_FREQUENCY_44K);


        /* Start Recording */
        HAL_StatusTypeDef res = HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *) RecordBuffer, RECORD_BUFFER_SIZE);
        if (HAL_OK == res) {
            printf("SAI receive begin OK\r\n");
        } else {
            printf("SAI receive error: %d\r\n", res);
        }

        printf("Copying Record buffer to Playback buffer\r\n");

        /* Play the recorded buffer */
        if (wm8994_drv.Play(AUDIO_I2C_ADDRESS, (uint16_t *) PlaybackBuffer, RECORD_BUFFER_SIZE) != 0) {
            printf("Codec play begin error\r\n");
            Error_Handler();
        } else {
            if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)PlaybackBuffer, RECORD_BUFFER_SIZE) != HAL_OK) {
                printf("SAI transmit begin error\r\n");
                Error_Handler();
            }
            printf("SAI transmit begin OK\r\n");
        }

        printf("\r\n");

        audio_rec_buffer_state = BUFFER_OFFSET_NONE;
        while (1) {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

            /* 1st or 2nd half of the record buffer ready for being copied
            to the Playback buffer */
            if (audio_rec_buffer_state != BUFFER_OFFSET_NONE) {
                /* Copy half of the record buffer to the playback buffer */
                if (audio_rec_buffer_state == BUFFER_OFFSET_HALF) {
                    get_max_val(RecordBuffer, RECORD_BUFFER_SIZE / 2, amp);
                    CopyBuffer(&PlaybackBuffer[0], &RecordBuffer[0], RECORD_BUFFER_SIZE / 2);
                } else {
                    /* if(audio_rec_buffer_state == BUFFER_OFFSET_FULL)*/
                    CopyBuffer(&PlaybackBuffer[RECORD_BUFFER_SIZE / 2],
                               &RecordBuffer[RECORD_BUFFER_SIZE / 2],
                               RECORD_BUFFER_SIZE / 2);
                }
                /* Wait for next data */
                audio_rec_buffer_state = BUFFER_OFFSET_NONE;
            }
        } // end while(1)
    } // end AUDIO_LOOPBACK

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_I2C4;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 200;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART2 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 0xFFFF);

    return ch;
}

/**
  * @brief  Clock Config.
  * @param  hsai: might be required to set audio peripheral predivider if any.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Params
  * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
  *         Being __weak it can be overwritten by the application
  * @retval None
  */
void BSP_AUDIO_OUT_ClockConfig(uint32_t AudioFreq) {
    RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;

    HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);

    /* Set the PLL configuration according to the audio frequency */
    if ((AudioFreq == SAI_AUDIO_FREQUENCY_11K) || (AudioFreq == SAI_AUDIO_FREQUENCY_22K) ||
        (AudioFreq == SAI_AUDIO_FREQUENCY_44K)) {
        /* Configure PLLSAI prescalers */
        /* PLLSAI_VCO: VCO_429M
        SAI_CLK(first level) = PLLSAI_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
        SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ = 214.5/19 = 11.289 Mhz */
        rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
        rcc_ex_clk_init_struct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
        rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 429;
        rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 2;
        rcc_ex_clk_init_struct.PLLI2SDivQ = 19;

        HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);

    } else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
    {
        /* SAI clock config
        PLLSAI_VCO: VCO_344M
        SAI_CLK(first level) = PLLSAI_VCO/PLLSAIQ = 344/7 = 49.142 Mhz
        SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ = 49.142/1 = 49.142 Mhz */
        rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
        rcc_ex_clk_init_struct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
        rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 344;
        rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 7;
        rcc_ex_clk_init_struct.PLLI2SDivQ = 1;

        HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
    }
}

/*
  * get maximum value of the buffer. VU meter, perhaps?
  */
void get_max_val(int16_t *buf, uint32_t size, int16_t amp[]) {
    int16_t maxval[4] = {-32768, -32768, -32768, -32768};
    uint32_t idx;
    for (idx = 0; idx < size; idx += 4) {
        if (buf[idx] > maxval[0])
            maxval[0] = buf[idx];
        if (buf[idx + 1] > maxval[1])
            maxval[1] = buf[idx + 1];
        if (buf[idx + 2] > maxval[2])
            maxval[2] = buf[idx + 2];
        if (buf[idx + 3] > maxval[3])
            maxval[3] = buf[idx + 3];
    }
    memcpy(amp, maxval, sizeof(maxval));
}

/*
 * Cop the contents of the Record buffer to the
 * Playback buffer
 *
 * If you wanted to hook into the signal and do some
 * signal processing, here is a place where you have
 * both buffers available
 *
 */
static void CopyBuffer(int16_t *pbuffer1, int16_t *pbuffer2, uint16_t BufferSize) {
    uint32_t i = 0;
    for (i = 0; i < BufferSize; i++) {
        pbuffer1[i] = pbuffer2[i];
    }
}

/**
  * @brief Manages the DMA Transfer complete interrupt.
  * @param None
  * @retval None
  */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
    audio_rec_buffer_state = BUFFER_OFFSET_FULL;
}


/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    audio_rec_buffer_state = BUFFER_OFFSET_HALF;

}


/**
  * @brief  SAI error callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
    //if(hsai->Instance == AUDIO_OUT_SAIx)
    {
//  BSP_AUDIO_OUT_Error_CallBack();
    }
    //else
    {
        /* This function is called when an Interrupt due to transfer error on or peripheral
           error occurs. */
        /* Display message on the LCD screen */
        //BSP_LCD_SetBackColor(LCD_COLOR_RED);
        //BSP_LCD_DisplayStringAt(0, LINE(14), (uint8_t *)"       DMA  ERROR     ", CENTER_MODE);


        /* Stop the program with an infinite loop */

        /*
        while (BSP_PB_GetState(BUTTON_WAKEUP) != RESET)
        {
            return;
        }
        */

        /* could also generate a system reset to recover from the error */
        /* .... */
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

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
