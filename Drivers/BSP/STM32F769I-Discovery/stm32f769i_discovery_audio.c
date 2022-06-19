/**
  ******************************************************************************
  * @file    stm32f769i_discovery_audio.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    30-December-2016
  * @brief   This file provides the Audio driver for the STM32F769I-DISCOVERY  
  *          board.
  @verbatim
  How To use this driver:
  -----------------------
   + This driver supports STM32F7xx devices on STM32F769I-DISCOVERY (MB1225) Evaluation boards.
   + Call the function BSP_AUDIO_OUT_Init(
                                    OutputDevice: physical output mode (OUTPUT_DEVICE_SPEAKER, 
                                                  OUTPUT_DEVICE_HEADPHONE or OUTPUT_DEVICE_BOTH)
                                    Volume      : Initial volume to be set (0 is min (mute), 100 is max (100%)
                                    AudioFreq   : Audio frequency in Hz (8000, 16000, 22500, 32000...)
                                                  this parameter is relative to the audio file/stream type.
                                   )
      This function configures all the hardware required for the audio application (codec, I2C, SAI, 
      GPIOs, DMA and interrupt if needed). This function returns AUDIO_OK if configuration is OK.
      If the returned value is different from AUDIO_OK or the function is stuck then the communication with
      the codec has failed (try to un-plug the power or reset device in this case).
      - OUTPUT_DEVICE_SPEAKER  : only speaker will be set as output for the audio stream.
      - OUTPUT_DEVICE_HEADPHONE: only headphones will be set as output for the audio stream.
      - OUTPUT_DEVICE_BOTH     : both Speaker and Headphone are used as outputs for the audio stream
                                 at the same time.
      Note. On STM32F769I-DISCOVERY SAI_DMA is configured in CIRCULAR mode. Due to this the application
        does NOT need to call BSP_AUDIO_OUT_ChangeBuffer() to assure streaming.
   + Call the function BSP_AUDIO_OUT_Play(
                                  pBuffer: pointer to the audio data file address
                                  Size   : size of the buffer to be sent in Bytes
                                 )
      to start playing (for the first time) from the audio file/stream.
   + Call the function BSP_AUDIO_OUT_Pause() to pause playing   
   + Call the function BSP_AUDIO_OUT_Resume() to resume playing.
       Note. After calling BSP_AUDIO_OUT_Pause() function for pause, only BSP_AUDIO_OUT_Resume() should be called
          for resume (it is not allowed to call BSP_AUDIO_OUT_Play() in this case).
       Note. This function should be called only when the audio file is played or paused (not stopped).
   + For each mode, you may need to implement the relative callback functions into your code.
      The Callback functions are named BSP_AUDIO_OUT_XXX_CallBack() and only their prototypes are declared in 
      the stm32f769i_discovery_audio.h file. (refer to the example for more details on the callbacks implementations)
   + To Stop playing, to modify the volume level, the frequency, the audio frame slot, 
      the device output mode the mute or the stop, use the functions: BSP_AUDIO_OUT_SetVolume(), 
      AUDIO_OUT_SetFrequency(), BSP_AUDIO_OUT_SetAudioFrameSlot(), BSP_AUDIO_OUT_SetOutputMode(),
      BSP_AUDIO_OUT_SetMute() and BSP_AUDIO_OUT_Stop().

   + Call the function BSP_AUDIO_IN_Init(
                                    AudioFreq: Audio frequency in Hz (8000, 16000, 22500, 32000...)
                                                  this parameter is relative to the audio file/stream type.
                                    BitRes: Bit resolution fixed to 16bit
                                    ChnlNbr: Number of channel to be configured for the DFSDM peripheral
                                   )
      This function configures all the hardware required for the audio in application (DFSDM filters and channels, 
      Clock source for DFSDM periphiral, GPIOs, DMA and interrupt if needed). 
      This function returns AUDIO_OK if configuration is OK.If the returned value is different from AUDIO_OK then
      the configuration should be wrong.
      Note: On STM32F769I-DISCOVERY, four DFSDM Channel/Filters are configured and their DMA streams are configured
            in CIRCULAR mode.
   + Call the function BSP_AUDIO_IN_AllocScratch(
                                        pScratch: pointer to scratch tables
                                        size: size of scratch buffer)
     This function must be called before BSP_AUDIO_IN_RECORD() to allocate buffer scratch for each DFSDM channel
     and its size.
     Note: These buffers scratch are used as intermidiate buffers to collect data within final record buffer.
           size is the total size of the four buffers scratch; If size is 512 then the size of each is 128.
           This function must be called after BSP_AUDIO_IN_Init()
   + Call the function BSP_AUDIO_IN_RECORD(
                                  pBuf: pointer to the recorded audio data file address
                                  Size: size of the buffer to be written in Bytes
                                 )
      to start recording from microphones.

   + Call the function BSP_AUDIO_IN_Pause() to pause recording   
   + Call the function BSP_AUDIO_IN_Resume() to recording playing.
       Note. After calling BSP_AUDIO_IN_Pause() function for pause, only BSP_AUDIO_IN_Resume() should be called
          for resume (it is not allowed to call BSP_AUDIO_IN_RECORD() in this case).
   + Call the function BSP_AUDIO_IN_Stop() to stop recording 
   + For each mode, you may need to implement the relative callback functions into your code.
      The Callback functions are named BSP_AUDIO_IN_XXX_CallBack() and only their prototypes are declared in 
      the stm32f769i_discovery_audio.h file. (refer to the example for more details on the callbacks implementations)

  Driver architecture:
  --------------------
   + This driver provides the High Audio Layer: consists of the function API exported in the stm32f769i_discovery_audio.h file
     (BSP_AUDIO_OUT_Init(), BSP_AUDIO_OUT_Play() ...)
   + This driver provide also the Media Access Layer (MAL): which consists of functions allowing to access the media containing/
     providing the audio file/stream. These functions are also included as local functions into
     the stm32f769i_discovery_audio.c file (DFSDMx_Init(), DFSDMx_DeInit(), SAIx_Init() and SAIx_DeInit())

  Known Limitations:
  ------------------
   1- If the TDM Format used to play in parallel 2 audio Stream (the first Stream is configured in codec SLOT0 and second 
      Stream in SLOT1) the Pause/Resume, volume and mute feature will control the both streams.
   2- Parsing of audio file is not implemented (in order to determine audio file properties: Mono/Stereo, Data size, 
      File size, Audio Frequency, Audio Data header size ...). The configuration is fixed for the given audio file.
   3- Supports only Stereo audio streaming.
   4- Supports only 16-bits audio data size.
  @endverbatim  
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
#include "stm32f769i_discovery_audio.h"

/**
  * @}
  */ 
  
/** @defgroup STM32F769I_DISCOVERY_AUDIO_Private_Variables STM32F769I_DISCOVERY_AUDIO Private Variables
  * @{
  */
/* PLAY */
AUDIO_DrvTypeDef                *audio_drv;
SAI_HandleTypeDef               haudio_out_sai;
SAI_HandleTypeDef               haudio_in_sai;



/**
  * @}
  */ 

/** @defgroup STM32F769I_DISCOVERY_AUDIO_Private_Function_Prototypes STM32F769I_DISCOVERY_AUDIO Private Function Prototypes
  * @{
  */
static void SAI_AUDIO_IN_MspInit(SAI_HandleTypeDef *hsai, void *Params);
static void SAIx_In_Init(uint32_t AudioFreq);
static void SAIx_In_DeInit(void);


/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32f769i_discovery_audio.h) */
  BSP_AUDIO_OUT_TransferComplete_CallBack();
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32f769i_discovery_audio.h) */
  BSP_AUDIO_OUT_HalfTransfer_CallBack();
}

/**
  * @brief  SAI error callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  if(hsai->Instance == AUDIO_OUT_SAIx)
  {
  BSP_AUDIO_OUT_Error_CallBack();
  }
  else
  {
    BSP_AUDIO_IN_Error_CallBack();
  }
}

/**
  * @brief  Manages the DMA full Transfer complete event.
  * @retval None
  */
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @retval None
  */
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
}

/**
  * @brief  Manages the DMA FIFO error event.
  * @retval None
  */
__weak void BSP_AUDIO_OUT_Error_CallBack(void)
{
}

/**
  * @brief  Initializes BSP_AUDIO_OUT MSP.
  * @param  hsai: SAI handle
  * @param  Params  
  * @retval None
  */
__weak void BSP_AUDIO_OUT_MspInit(SAI_HandleTypeDef *hsai, void *Params)
{ 
  static DMA_HandleTypeDef hdma_sai_tx;
  GPIO_InitTypeDef  gpio_init_structure;  
  
  /* Enable SAI clock */
  AUDIO_OUT_SAIx_CLK_ENABLE();
  
  
  /* Enable GPIO clock */
  AUDIO_OUT_SAIx_MCLK_ENABLE();
  AUDIO_OUT_SAIx_SD_FS_CLK_ENABLE();
  
  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  gpio_init_structure.Pin = AUDIO_OUT_SAIx_FS_PIN | AUDIO_OUT_SAIx_SCK_PIN | AUDIO_OUT_SAIx_SD_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_SD_FS_SCK_GPIO_PORT, &gpio_init_structure);
  
  gpio_init_structure.Pin = AUDIO_OUT_SAIx_MCLK_PIN;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_MCLK_GPIO_PORT, &gpio_init_structure);
  
  /* Enable the DMA clock */
  AUDIO_OUT_SAIx_DMAx_CLK_ENABLE();
  
  if(hsai->Instance == AUDIO_OUT_SAIx)
  {
    /* Configure the hdma_saiTx handle parameters */   
    hdma_sai_tx.Init.Channel             = AUDIO_OUT_SAIx_DMAx_CHANNEL;
    hdma_sai_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sai_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai_tx.Init.PeriphDataAlignment = AUDIO_OUT_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_sai_tx.Init.MemDataAlignment    = AUDIO_OUT_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_sai_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_sai_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
    hdma_sai_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
    
    hdma_sai_tx.Instance = AUDIO_OUT_SAIx_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hdma_sai_tx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_sai_tx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_sai_tx);
  }
  
  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_OUT_SAIx_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_OUT_SAIx_DMAx_IRQ);
}

/**
  * @brief  Initializes SAI Audio IN MSP.
  * @param  hsai: SAI handle
  * @param  Params  
  * @retval None
  */
static void SAI_AUDIO_IN_MspInit(SAI_HandleTypeDef *hsai, void *Params)
{
  static DMA_HandleTypeDef hdma_sai_rx;
  GPIO_InitTypeDef  gpio_init_structure;  

  /* Enable SAI clock */
  AUDIO_IN_SAIx_CLK_ENABLE();
  
  /* Enable SD GPIO clock */
  AUDIO_IN_SAIx_SD_ENABLE();
  /* CODEC_SAI pin configuration: SD pin */
  gpio_init_structure.Pin = AUDIO_IN_SAIx_SD_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Alternate = AUDIO_IN_SAIx_AF;
  HAL_GPIO_Init(AUDIO_IN_SAIx_SD_GPIO_PORT, &gpio_init_structure);

  /* Enable Audio INT GPIO clock */
  AUDIO_IN_INT_GPIO_ENABLE();
  /* Audio INT pin configuration: input */
  gpio_init_structure.Pin = AUDIO_IN_INT_GPIO_PIN;
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(AUDIO_IN_INT_GPIO_PORT, &gpio_init_structure);

  /* Enable the DMA clock */
  AUDIO_IN_SAIx_DMAx_CLK_ENABLE();
    
  if(hsai->Instance == AUDIO_IN_SAIx)
  {
    /* Configure the hdma_sai_rx handle parameters */
    hdma_sai_rx.Init.Channel             = AUDIO_IN_SAIx_DMAx_CHANNEL;
    hdma_sai_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sai_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai_rx.Init.PeriphDataAlignment = AUDIO_IN_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_sai_rx.Init.MemDataAlignment    = AUDIO_IN_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_sai_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_sai_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_sai_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
    
    hdma_sai_rx.Instance = AUDIO_IN_SAIx_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmarx, hdma_sai_rx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_sai_rx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_sai_rx);
  }
  
  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_IN_SAIx_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_IN_SAIx_DMAx_IRQ);

  /* Audio INT IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_IN_INT_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_IN_INT_IRQ);
}

/**
  * @brief  De-Initializes SAI Audio IN MSP.
  * @param  hsai: SAI handle
  * @param  Params  
  * @retval None
  */
static void SAI_AUDIO_IN_MspDeInit(SAI_HandleTypeDef *hsai, void *Params)
{
  GPIO_InitTypeDef  gpio_init_structure;
  
  /* SAI DMA IRQ Channel deactivation */
  HAL_NVIC_DisableIRQ(AUDIO_IN_SAIx_DMAx_IRQ);
  
  if(hsai->Instance == AUDIO_IN_SAIx)
  {
    /* Deinitialize the DMA stream */
    HAL_DMA_DeInit(hsai->hdmatx);
  }
  
  /* Disable SAI peripheral */
  __HAL_SAI_DISABLE(hsai);  
  
  /* Deactivates CODEC_SAI pin SD by putting them in input mode */
  gpio_init_structure.Pin = AUDIO_IN_SAIx_SD_PIN;
  HAL_GPIO_DeInit(AUDIO_IN_SAIx_SD_GPIO_PORT, gpio_init_structure.Pin);
  
  gpio_init_structure.Pin = AUDIO_IN_INT_GPIO_PIN;
  HAL_GPIO_DeInit(AUDIO_IN_INT_GPIO_PORT, gpio_init_structure.Pin);
  
  /* Disable SAI clock */
  AUDIO_IN_SAIx_CLK_DISABLE();  
}

/**
  * @brief  Deinitializes SAI MSP.
  * @param  hsai: SAI handle
  * @param  Params  
  * @retval None
  */
__weak void BSP_AUDIO_OUT_MspDeInit(SAI_HandleTypeDef *hsai, void *Params)
{
    GPIO_InitTypeDef  gpio_init_structure;

    /* SAI DMA IRQ Channel deactivation */
    HAL_NVIC_DisableIRQ(AUDIO_OUT_SAIx_DMAx_IRQ);

    if(hsai->Instance == AUDIO_OUT_SAIx)
    {
      /* Deinitialize the DMA stream */
      HAL_DMA_DeInit(hsai->hdmatx);
    }

    /* Disable SAI peripheral */
    __HAL_SAI_DISABLE(hsai);  

    /* Deactivates CODEC_SAI pins FS, SCK, MCK and SD by putting them in input mode */
    gpio_init_structure.Pin = AUDIO_OUT_SAIx_FS_PIN | AUDIO_OUT_SAIx_SCK_PIN | AUDIO_OUT_SAIx_SD_PIN;
    HAL_GPIO_DeInit(AUDIO_OUT_SAIx_SD_FS_SCK_GPIO_PORT, gpio_init_structure.Pin);

    gpio_init_structure.Pin = AUDIO_OUT_SAIx_MCLK_PIN;
    HAL_GPIO_DeInit(AUDIO_OUT_SAIx_MCLK_GPIO_PORT, gpio_init_structure.Pin);
  
    /* Disable SAI clock */
    AUDIO_OUT_SAIx_CLK_DISABLE();

    /* GPIO pins clock and DMA clock can be shut down in the applic 
       by surcharging this __weak function */ 
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
__weak void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq, void *Params)
{ 
  RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;

  HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);
  
  /* Set the PLL configuration according to the audio frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
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
    
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
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

/**
  * @brief  Initializes the Audio Codec audio interface (SAI).
  * @param  AudioFreq: Audio frequency to be configured for the SAI peripheral.
  * @note   The default SlotActive configuration is set to CODEC_AUDIOFRAME_SLOT_0123 
  *         and user can update this configuration using 
  * @retval None
  */
static void SAIx_In_Init(uint32_t AudioFreq)
{
  /* Initialize SAI1 block A in MASTER TX */
  /* Initialize the haudio_out_sai Instance parameter */
  haudio_out_sai.Instance = AUDIO_OUT_SAIx;

  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);

  /* Configure SAI_Block_x
  LSBFirst: Disabled
  DataSize: 16 */
  haudio_out_sai.Init.MonoStereoMode = SAI_STEREOMODE;
  haudio_out_sai.Init.AudioFrequency = AudioFreq;
  haudio_out_sai.Init.AudioMode      = SAI_MODEMASTER_RX;
  haudio_out_sai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  haudio_out_sai.Init.Protocol       = SAI_FREE_PROTOCOL;
  haudio_out_sai.Init.DataSize       = SAI_DATASIZE_16;
  haudio_out_sai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  haudio_out_sai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
  haudio_out_sai.Init.Synchro        = SAI_ASYNCHRONOUS;
  haudio_out_sai.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  haudio_out_sai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  haudio_out_sai.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
  haudio_out_sai.Init.CompandingMode = SAI_NOCOMPANDING;
  haudio_out_sai.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
  haudio_out_sai.Init.Mckdiv         = 0;  

  /* Configure SAI_Block_x Frame
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  haudio_out_sai.FrameInit.FrameLength       = 64;
  haudio_out_sai.FrameInit.ActiveFrameLength = 32;
  haudio_out_sai.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_out_sai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  haudio_out_sai.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

  /* Configure SAI Block_x Slot
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 4
  Slot Active: All slot actives */
  haudio_out_sai.SlotInit.FirstBitOffset = 0;
  haudio_out_sai.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
  haudio_out_sai.SlotInit.SlotNumber     = 4;
  haudio_out_sai.SlotInit.SlotActive     = CODEC_AUDIOFRAME_SLOT_02;

  HAL_SAI_Init(&haudio_out_sai);

  /* Initialize SAI1 block B in SLAVE RX synchronous from SAI1 block A */
  /* Initialize the haudio_in_sai Instance parameter */
  haudio_in_sai.Instance = AUDIO_IN_SAIx;
  
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_in_sai);
  
  /* Configure SAI_Block_x
  LSBFirst: Disabled
  DataSize: 16 */
  haudio_in_sai.Init.MonoStereoMode = SAI_STEREOMODE;
  haudio_in_sai.Init.AudioFrequency = AudioFreq;
  haudio_in_sai.Init.AudioMode      = SAI_MODESLAVE_RX;
  haudio_in_sai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  haudio_in_sai.Init.Protocol       = SAI_FREE_PROTOCOL;
  haudio_in_sai.Init.DataSize       = SAI_DATASIZE_16;
  haudio_in_sai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  haudio_in_sai.Init.ClockStrobing  = SAI_CLOCKSTROBING_RISINGEDGE;
  haudio_in_sai.Init.Synchro        = SAI_SYNCHRONOUS;
  haudio_in_sai.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
  haudio_in_sai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  haudio_in_sai.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
  haudio_in_sai.Init.CompandingMode = SAI_NOCOMPANDING;
  haudio_in_sai.Init.TriState       = SAI_OUTPUT_RELEASED;
  haudio_in_sai.Init.Mckdiv         = 0;

  /* Configure SAI_Block_x Frame
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  haudio_in_sai.FrameInit.FrameLength       = 64;
  haudio_in_sai.FrameInit.ActiveFrameLength = 32;
  haudio_in_sai.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_in_sai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  haudio_in_sai.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;
  
  /* Configure SAI Block_x Slot
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 4
  Slot Active: All slot active */
  haudio_in_sai.SlotInit.FirstBitOffset = 0;
  haudio_in_sai.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
  haudio_in_sai.SlotInit.SlotNumber     = 4;
  haudio_in_sai.SlotInit.SlotActive     = CODEC_AUDIOFRAME_SLOT_02;

  HAL_SAI_Init(&haudio_in_sai);

  /* Enable SAI peripheral */
  __HAL_SAI_ENABLE(&haudio_in_sai);

  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);
}

/**
  * @brief  Deinitializes the output Audio Codec audio interface (SAI).
  * @retval None
  */
static void SAIx_In_DeInit(void)
{
  /* Initialize the haudio_in_sai Instance parameter */
  haudio_in_sai.Instance = AUDIO_IN_SAIx;
  haudio_out_sai.Instance = AUDIO_OUT_SAIx;
  /* Disable SAI peripheral */
  __HAL_SAI_DISABLE(&haudio_in_sai);

  HAL_SAI_DeInit(&haudio_in_sai);
  HAL_SAI_DeInit(&haudio_out_sai);
}

/**
  * @}
  */

/** @defgroup STM32F769I_DISCOVERY_AUDIO_In_Private_Functions STM32F769I_DISCOVERY_AUDIO_In Private Functions
  * @{
  */
/**
  * @brief  Half reception complete callback.
  * @param  hsai : SAI handle.
  * @retval None
  */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32769i_discovery_audio.h) */
  BSP_AUDIO_IN_HalfTransfer_CallBack();
}

/**
  * @brief  Reception complete callback.
  * @param  hsai : SAI handle.
  * @retval None
  */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
  BSP_AUDIO_IN_TransferComplete_CallBack();
}

/**
  * @brief  User callback when record buffer is filled.
  * @retval None
  */
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /* This function should be implemented by the user application.
     It is called into this driver when the current buffer is filled
     to prepare the next buffer pointer and its size. */
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @retval None
  */
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{ 
  /* This function should be implemented by the user application.
     It is called into this driver when the current buffer is filled
     to prepare the next buffer pointer and its size. */
}

/**
  * @brief  Audio IN Error callback function.
  * @retval None
  */
__weak void BSP_AUDIO_IN_Error_CallBack(void)
{   
  /* This function is called when an Interrupt due to transfer error on or peripheral
     error occurs. */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
