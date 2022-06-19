/**
  ******************************************************************************
  * @file    wm8994.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    22-February-2016
  * @brief   This file provides the WM8994 Audio Codec driver.   
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
#include "wm8994.h"

static uint32_t outputEnabled = 0;
static uint32_t inputEnabled = 0;

/**
  * @brief Initializes the audio codec and the control interface.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param OutputInputDevice: can be OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *  OUTPUT_DEVICE_BOTH, OUTPUT_DEVICE_AUTO, INPUT_DEVICE_DIGITAL_MICROPHONE_1,
  *  INPUT_DEVICE_DIGITAL_MICROPHONE_2, INPUT_DEVICE_DIGITAL_MIC1_MIC2, 
  *  INPUT_DEVICE_INPUT_LINE_1 or INPUT_DEVICE_INPUT_LINE_2.
  * @param Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param AudioFreq: Audio Frequency 
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_Init(uint16_t DeviceAddr, uint16_t OutputInputDevice, uint8_t Volume, uint32_t AudioFreq)
{
  int counter = 0;
  uint16_t output_device = OutputInputDevice & 0xFF;
  uint16_t input_device = OutputInputDevice & 0xFF00;
  uint16_t power_mgnt_reg_1 = 0;
  
  /* Initialize the Control interface of the Audio Codec */
  /* wm8994 Errata Work-Arounds */
  counter += CODEC_IO_Write(DeviceAddr, 0x102, 0x0003);
  counter += CODEC_IO_Write(DeviceAddr, 0x817, 0x0000);
  counter += CODEC_IO_Write(DeviceAddr, 0x102, 0x0000);
  
  /* Enable VMID soft start (fast), Start-up Bias Current Enabled */
  counter += CODEC_IO_Write(DeviceAddr, 0x39, 0x006C);
  
    /* Enable bias generator, Enable VMID */
  if (input_device > 0)
  {
    counter += CODEC_IO_Write(DeviceAddr, 0x01, 0x0013);
  }
  else
  {
    counter += CODEC_IO_Write(DeviceAddr, 0x01, 0x0003);
  }

  /* Add Delay */
    CODEC_IO_Delay(50);

  /* Path Configurations for output */
  if (output_device > 0)
  {
    outputEnabled = 1;
    switch (output_device)
    {
    case CODEC_OUT_SPEAKER:
      /* Enable DAC1 (Left), Enable DAC1 (Right),
      Disable DAC2 (Left), Disable DAC2 (Right)*/
      counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0C0C);

      /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0000);

      /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0000);

      /* Disable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0002);

      /* Disable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0002);
      break;

    case CODEC_OUT_HEADPHONE:
      /* Disable DAC1 (Left), Disable DAC1 (Right),
      Enable DAC2 (Left), Enable DAC2 (Right)*/
      counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303);

      /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);

      /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);

      /* Disable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0000);

      /* Disable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0000);
      break;

    case CODEC_OUT_BOTH:
      if (input_device == (CODEC_IN_DIGITAL_MIC1 | CODEC_IN_DIGITAL_MIC2))
      {
        /* Enable DAC1 (Left), Enable DAC1 (Right),
        also Enable DAC2 (Left), Enable DAC2 (Right)*/
        counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303 | 0x0C0C);
        
        /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path
        Enable the AIF1 Timeslot 1 (Left) to DAC 1 (Left) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0003);
        
        /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path
        Enable the AIF1 Timeslot 1 (Right) to DAC 1 (Right) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0003);
        
        /* Enable the AIF1 Timeslot 0 (Left) to DAC 2 (Left) mixer path
        Enable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path  */
        counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0003);
        
        /* Enable the AIF1 Timeslot 0 (Right) to DAC 2 (Right) mixer path
        Enable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0003);
      }
      else
      {
        /* Enable DAC1 (Left), Enable DAC1 (Right),
        also Enable DAC2 (Left), Enable DAC2 (Right)*/
        counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303 | 0x0C0C);
        
        /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);
        
        /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);
        
        /* Enable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0002);
        
        /* Enable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
        counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0002);      
      }
      break;

    case CODEC_OUT_AUTO :
    default:
      /* Disable DAC1 (Left), Disable DAC1 (Right),
      Enable DAC2 (Left), Enable DAC2 (Right)*/
      counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303);

      /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);

      /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);

      /* Disable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0000);

      /* Disable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0000);
      break;
    }
  }
  else
  {
    outputEnabled = 0;
  }

  /* Path Configurations for input */
  if (input_device > 0)
  {
    inputEnabled = 1;
    switch (input_device)
    {
    case CODEC_IN_DIGITAL_MIC2 :
      /* Enable AIF1ADC2 (Left), Enable AIF1ADC2 (Right)
       * Enable DMICDAT2 (Left), Enable DMICDAT2 (Right)
       * Enable Left ADC, Enable Right ADC */
      counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x0C30);

      /* Enable AIF1 DRC2 Signal Detect & DRC in AIF1ADC2 Left/Right Timeslot 1 */
      counter += CODEC_IO_Write(DeviceAddr, 0x450, 0x00DB);

      /* Disable IN1L, IN1R, IN2L, IN2R, Enable Thermal sensor & shutdown */
      counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x6000);

      /* Enable the DMIC2(Left) to AIF1 Timeslot 1 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x608, 0x0002);

      /* Enable the DMIC2(Right) to AIF1 Timeslot 1 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x609, 0x0002);

      /* GPIO1 pin configuration GP1_DIR = output, GP1_FN = AIF1 DRC2 signal detect */
      counter += CODEC_IO_Write(DeviceAddr, 0x700, 0x000E);
      break;

    case CODEC_IN_LINE_1 :
      /* IN1LN_TO_IN1L, IN1LP_TO_VMID, IN1RN_TO_IN1R, IN1RP_TO_VMID */
      counter += CODEC_IO_Write(DeviceAddr, 0x28, 0x0011);

      /* Disable mute on IN1L_TO_MIXINL and +30dB on IN1L PGA output */
      //counter += CODEC_IO_Write(DeviceAddr, 0x29, 0x0035);

      /* CHANGE! MIXOUTR_MIXINR_VOL set to 0 (mute)
       * +30db removed : 0x0020 instead of 0x0035  */
      counter += CODEC_IO_Write(DeviceAddr, 0x29, 0x0020);

      /* Disable mute on IN1R_TO_MIXINL, Gain = +30dB */
      //counter += CODEC_IO_Write(DeviceAddr, 0x2A, 0x0035);

      /* CHANGE! MIXOUTL_MIXINL_VOL set to 0 (mute)
       * +30db removed : 0x0020 instead of 0x0035  */
      counter += CODEC_IO_Write(DeviceAddr, 0x2A, 0x0020);

      /* Enable AIF1ADC1 (Left), Enable AIF1ADC1 (Right)
       * Enable Left ADC, Enable Right ADC */
      counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x0303);

      /* Enable AIF1 DRC1 Signal Detect & DRC in AIF1ADC1 Left/Right Timeslot 0 */
      counter += CODEC_IO_Write(DeviceAddr, 0x440, 0x00DB);

      /* Enable IN1L and IN1R, Disable IN2L and IN2R, Enable Thermal sensor & shutdown */
      counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x6350);

      /* Enable the ADCL(Left) to AIF1 Timeslot 0 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x606, 0x0002);

      /* Enable the ADCR(Right) to AIF1 Timeslot 0 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x607, 0x0002);

      /* GPIO1 pin configuration GP1_DIR = output, GP1_FN = AIF1 DRC1 signal detect */
      counter += CODEC_IO_Write(DeviceAddr, 0x700, 0x000D);
      break;

    case CODEC_IN_DIGITAL_MIC1 :
      /* Enable AIF1ADC1 (Left), Enable AIF1ADC1 (Right)
       * Enable DMICDAT1 (Left), Enable DMICDAT1 (Right)
       * Enable Left ADC, Enable Right ADC */
      counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x030C);

      /* Enable AIF1 DRC2 Signal Detect & DRC in AIF1ADC1 Left/Right Timeslot 0 */
      counter += CODEC_IO_Write(DeviceAddr, 0x440, 0x00DB);

      /* Disable IN1L, IN1R, IN2L, IN2R, Enable Thermal sensor & shutdown */
      counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x6350);

      /* Enable the DMIC2(Left) to AIF1 Timeslot 0 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x606, 0x0002);

      /* Enable the DMIC2(Right) to AIF1 Timeslot 0 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x607, 0x0002);

      /* GPIO1 pin configuration GP1_DIR = output, GP1_FN = AIF1 DRC1 signal detect */
      counter += CODEC_IO_Write(DeviceAddr, 0x700, 0x000D);
      break; 
    case (CODEC_IN_DIGITAL_MIC1 | CODEC_IN_DIGITAL_MIC2) :
      /* Enable AIF1ADC1 (Left), Enable AIF1ADC1 (Right)
       * Enable DMICDAT1 (Left), Enable DMICDAT1 (Right)
       * Enable Left ADC, Enable Right ADC */
      counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x0F3C);

      /* Enable AIF1 DRC2 Signal Detect & DRC in AIF1ADC2 Left/Right Timeslot 1 */
      counter += CODEC_IO_Write(DeviceAddr, 0x450, 0x00DB);
      
      /* Enable AIF1 DRC2 Signal Detect & DRC in AIF1ADC1 Left/Right Timeslot 0 */
      counter += CODEC_IO_Write(DeviceAddr, 0x440, 0x00DB);

      /* Disable IN1L, IN1R, Enable IN2L, IN2R, Thermal sensor & shutdown */
      counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x63A0);

      /* Enable the DMIC2(Left) to AIF1 Timeslot 0 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x606, 0x0002);

      /* Enable the DMIC2(Right) to AIF1 Timeslot 0 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x607, 0x0002);

      /* Enable the DMIC2(Left) to AIF1 Timeslot 1 (Left) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x608, 0x0002);

      /* Enable the DMIC2(Right) to AIF1 Timeslot 1 (Right) mixer path */
      counter += CODEC_IO_Write(DeviceAddr, 0x609, 0x0002);
      
      /* GPIO1 pin configuration GP1_DIR = output, GP1_FN = AIF1 DRC1 signal detect */
      counter += CODEC_IO_Write(DeviceAddr, 0x700, 0x000D);
      break;    
    case CODEC_IN_LINE_2 :
    default:
      /* Actually, no other input devices supported */
      counter++;
      break;
    }
  }
  else
  {
    inputEnabled = 0;
  }
  
  /*  Clock Configurations */
  switch (AudioFreq)
  {
  case  SAI_AUDIO_FREQUENCY_8K:
    /* AIF1 Sample Rate = 8 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0003);
    break;
    
  case  SAI_AUDIO_FREQUENCY_16K:
    /* AIF1 Sample Rate = 16 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0033);
    break;

  case  SAI_AUDIO_FREQUENCY_32K:
    /* AIF1 Sample Rate = 32 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0063);
    break;
    
  case  SAI_AUDIO_FREQUENCY_48K:
    /* AIF1 Sample Rate = 48 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0083);
    break;
    
  case  SAI_AUDIO_FREQUENCY_96K:
    /* AIF1 Sample Rate = 96 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x00A3);
    break;
    
  case  SAI_AUDIO_FREQUENCY_11K:
    /* AIF1 Sample Rate = 11.025 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0013);
    break;
    
  case  SAI_AUDIO_FREQUENCY_22K:
    /* AIF1 Sample Rate = 22.050 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0043);
    break;
    
  case  SAI_AUDIO_FREQUENCY_44K:
    /* AIF1 Sample Rate = 44.1 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0073);
    break; 
    
  default:
    /* AIF1 Sample Rate = 48 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0083);
    break; 
  }

  if(input_device == (CODEC_IN_DIGITAL_MIC1 | CODEC_IN_DIGITAL_MIC2))
  {
  /* AIF1 Word Length = 16-bits, AIF1 Format = DSP mode */
  counter += CODEC_IO_Write(DeviceAddr, 0x300, 0x4018);    
  }
  else
  {
  /* AIF1 Word Length = 16-bits, AIF1 Format = I2S (Default Register Value) */
  counter += CODEC_IO_Write(DeviceAddr, 0x300, 0x4010);
  }
  
  /* slave mode */
  counter += CODEC_IO_Write(DeviceAddr, 0x302, 0x0000);
  
  /* Enable the DSP processing clock for AIF1, Enable the core clock */
  counter += CODEC_IO_Write(DeviceAddr, 0x208, 0x000A);
  
  /* Enable AIF1 Clock, AIF1 Clock Source = MCLK1 pin */
  counter += CODEC_IO_Write(DeviceAddr, 0x200, 0x0001);

  if (output_device > 0)  /* Audio output selected */
  {
    /* Analog Output Configuration */

    /* Enable SPKRVOL PGA, Enable SPKMIXR, Enable SPKLVOL PGA, Enable SPKMIXL */
    counter += CODEC_IO_Write(DeviceAddr, 0x03, 0x0300);

    /* Left Speaker Mixer Volume = 0dB */
    counter += CODEC_IO_Write(DeviceAddr, 0x22, 0x0000);

    /* Speaker output mode = Class D, Right Speaker Mixer Volume = 0dB ((0x23, 0x0100) = class AB)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x23, 0x0000);

    /* Unmute DAC2 (Left) to Left Speaker Mixer (SPKMIXL) path,
    Unmute DAC2 (Right) to Right Speaker Mixer (SPKMIXR) path */
    counter += CODEC_IO_Write(DeviceAddr, 0x36, 0x0300);

    /* Enable bias generator, Enable VMID, Enable SPKOUTL, Enable SPKOUTR */
    counter += CODEC_IO_Write(DeviceAddr, 0x01, 0x3003);

    /* Headphone/Speaker Enable */

    if (input_device == (CODEC_IN_DIGITAL_MIC1 | CODEC_IN_DIGITAL_MIC2))
    {
    /* Enable Class W, Class W Envelope Tracking = AIF1 Timeslots 0 and 1 */
    counter += CODEC_IO_Write(DeviceAddr, 0x51, 0x0205);
    }
    else
    {
    /* Enable Class W, Class W Envelope Tracking = AIF1 Timeslot 0 */
    counter += CODEC_IO_Write(DeviceAddr, 0x51, 0x0005);      
    }

    /* Enable bias generator, Enable VMID, Enable HPOUT1 (Left) and Enable HPOUT1 (Right) input stages */
    /* idem for Speaker */
    power_mgnt_reg_1 |= 0x0303 | 0x3003;
    counter += CODEC_IO_Write(DeviceAddr, 0x01, power_mgnt_reg_1);

    /* Enable HPOUT1 (Left) and HPOUT1 (Right) intermediate stages */
    counter += CODEC_IO_Write(DeviceAddr, 0x60, 0x0022);

    /* Enable Charge Pump */
    counter += CODEC_IO_Write(DeviceAddr, 0x4C, 0x9F25);

    /* Add Delay */
      CODEC_IO_Delay(15);

    /* Select DAC1 (Left) to Left Headphone Output PGA (HPOUT1LVOL) path */
    counter += CODEC_IO_Write(DeviceAddr, 0x2D, 0x0001);

    /* Select DAC1 (Right) to Right Headphone Output PGA (HPOUT1RVOL) path */
    counter += CODEC_IO_Write(DeviceAddr, 0x2E, 0x0001);

    /* Enable Left Output Mixer (MIXOUTL), Enable Right Output Mixer (MIXOUTR) */
    /* idem for SPKOUTL and SPKOUTR */
    counter += CODEC_IO_Write(DeviceAddr, 0x03, 0x0030 | 0x0300);

    /* Enable DC Servo and trigger start-up mode on left and right channels */
    counter += CODEC_IO_Write(DeviceAddr, 0x54, 0x0033);

    /* Add Delay */
      CODEC_IO_Delay(250);

    /* Enable HPOUT1 (Left) and HPOUT1 (Right) intermediate and output stages. Remove clamps */
    counter += CODEC_IO_Write(DeviceAddr, 0x60, 0x00EE);

    /* Unmutes */

    /* Unmute DAC 1 (Left) */
    counter += CODEC_IO_Write(DeviceAddr, 0x610, 0x00C0);

    /* Unmute DAC 1 (Right) */
    counter += CODEC_IO_Write(DeviceAddr, 0x611, 0x00C0);

    /* Unmute the AIF1 Timeslot 0 DAC path */
    counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0000);

    /* Unmute DAC 2 (Left) */
    counter += CODEC_IO_Write(DeviceAddr, 0x612, 0x00C0);

    /* Unmute DAC 2 (Right) */
    counter += CODEC_IO_Write(DeviceAddr, 0x613, 0x00C0);

    /* Unmute the AIF1 Timeslot 1 DAC2 path */
    counter += CODEC_IO_Write(DeviceAddr, 0x422, 0x0000);
    
    /* Volume Control */
    wm8994_SetVolume(DeviceAddr, Volume);
  }

  if (input_device > 0) /* Audio input selected */
  {
    if ((input_device == CODEC_IN_DIGITAL_MIC1) || (input_device == CODEC_IN_DIGITAL_MIC2))
    {
      /* Enable Microphone bias 1 generator, Enable VMID */
      power_mgnt_reg_1 |= 0x0013;
      counter += CODEC_IO_Write(DeviceAddr, 0x01, power_mgnt_reg_1);

      /* ADC oversample enable */
      counter += CODEC_IO_Write(DeviceAddr, 0x620, 0x0002);

      /* AIF ADC2 HPF enable, HPF cut = voice mode 1 fc=127Hz at fs=8kHz */
      counter += CODEC_IO_Write(DeviceAddr, 0x411, 0x3800);
    }
    else if(input_device == (CODEC_IN_DIGITAL_MIC1 | CODEC_IN_DIGITAL_MIC2))
    {
      /* Enable Microphone bias 1 generator, Enable VMID */
      power_mgnt_reg_1 |= 0x0013;
      counter += CODEC_IO_Write(DeviceAddr, 0x01, power_mgnt_reg_1);

      /* ADC oversample enable */
      counter += CODEC_IO_Write(DeviceAddr, 0x620, 0x0002);
    
      /* AIF ADC1 HPF enable, HPF cut = voice mode 1 fc=127Hz at fs=8kHz */
      counter += CODEC_IO_Write(DeviceAddr, 0x410, 0x1800);
      
      /* AIF ADC2 HPF enable, HPF cut = voice mode 1 fc=127Hz at fs=8kHz */
      counter += CODEC_IO_Write(DeviceAddr, 0x411, 0x1800);      
    }    
    else if ((input_device == CODEC_IN_LINE_1) || (input_device == CODEC_IN_LINE_2))
    {

      /* Disable mute on IN1L, IN1L Volume = +0dB */
      counter += CODEC_IO_Write(DeviceAddr, 0x18, 0x000B);

      /* Disable mute on IN1R, IN1R Volume = +0dB */
      counter += CODEC_IO_Write(DeviceAddr, 0x1A, 0x000B);

      /* AIF ADC1 HPF enable, HPF cut = hifi mode fc=4Hz at fs=48kHz */
      counter += CODEC_IO_Write(DeviceAddr, 0x410, 0x1800);
    }
    /* Volume Control */
    wm8994_SetVolume(DeviceAddr, Volume);
  }
  /* Return communication control value */
  return counter;  
}



/**
  * @brief  Get the WM8994 ID.
  * @param DeviceAddr: Device address on communication Bus.
  * @retval The WM8994 ID 
  */
uint32_t wm8994_ReadID(uint16_t DeviceAddr)
{
  return ((uint32_t) CODEC_IO_Read(DeviceAddr, CODEC_WM8994_CHIPID_ADDR));
}

/**
  * @brief Start the audio Codec play feature.
  * @note For this codec no Play options are required.
  * @param DeviceAddr: Device address on communication Bus.   
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_Play(uint16_t DeviceAddr)
{
  int counter = 0;
 
  /* Resumes the audio file playing */  
  /* Unmute the output first */
  counter += wm8994_SetMute(DeviceAddr, CODEC_UNMUTE);
  
  return counter;
}

/**
  * @brief Pauses playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_Pause(uint16_t DeviceAddr)
{  
  int counter = 0;
 
  /* Pause the audio file playing */
  /* Mute the output first */
  counter += wm8994_SetMute(DeviceAddr, CODEC_MUTE);
  
  /* Put the Codec in Power save mode */
  counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x01);
 
  return counter;
}

/**
  * @brief Resumes playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_Resume(uint16_t DeviceAddr)
{
  int counter = 0;
 
  /* Resumes the audio file playing */  
  /* Unmute the output first */
  counter += wm8994_SetMute(DeviceAddr, CODEC_UNMUTE);
  
  return counter;
}

/**
  * @brief Stops audio Codec playing. It powers down the codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @param CodecPdwnMode: selects the  power down mode.
  *          - CODEC_PDWN_SW: only mutes the audio codec. When resuming from this 
  *                           mode the codec keeps the previous initialization
  *                           (no need to re-Initialize the codec registers).
  *          - CODEC_PDWN_HW: Physically power down the codec. When resuming from this
  *                           mode, the codec is set to default configuration 
  *                           (user should re-Initialize the codec in order to 
  *                            play again the audio stream).
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_Stop(uint16_t DeviceAddr, codec_pwdn_t pwdn)
{
  int counter = 0;

  if (outputEnabled != 0)
  {
    /* Mute the output first */
    counter += wm8994_SetMute(DeviceAddr, CODEC_MUTE);

    if (pwdn == CODEC_PWDN_HW) {
      /* Mute the AIF1 Timeslot 0 DAC1 path */
      counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0200);

      /* Mute the AIF1 Timeslot 1 DAC2 path */
      counter += CODEC_IO_Write(DeviceAddr, 0x422, 0x0200);

      /* Disable DAC1L_TO_HPOUT1L */
      counter += CODEC_IO_Write(DeviceAddr, 0x2D, 0x0000);

      /* Disable DAC1R_TO_HPOUT1R */
      counter += CODEC_IO_Write(DeviceAddr, 0x2E, 0x0000);

      /* Disable DAC1 and DAC2 */
      counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0000);

      /* Reset Codec by writing in 0x0000 address register */
      counter += CODEC_IO_Write(DeviceAddr, 0x0000, 0x0000);

      outputEnabled = 0;
    }
  }
  return counter;
}

/**
  * @brief Sets higher or lower the codec volume level.
  * @param DeviceAddr: Device address on communication Bus.
  * @param Volume: a byte value from 0 to 255 (refer to codec registers 
  *         description for more details).
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_SetVolume(uint16_t DeviceAddr, uint8_t Volume)
{
  int counter = 0;
  uint8_t convertedvol = CODEC_VOLUME_CONVERT(Volume);

  /* Output volume */
  if (outputEnabled != 0)
  {
    if(convertedvol > 0x3E)
    {
      /* Unmute audio codec */
      counter += wm8994_SetMute(DeviceAddr, CODEC_UNMUTE);

      /* Left Headphone Volume */
      counter += CODEC_IO_Write(DeviceAddr, 0x1C, 0x3F | 0x140);

      /* Right Headphone Volume */
      counter += CODEC_IO_Write(DeviceAddr, 0x1D, 0x3F | 0x140);

      /* Left Speaker Volume */
      counter += CODEC_IO_Write(DeviceAddr, 0x26, 0x3F | 0x140);

      /* Right Speaker Volume */
      counter += CODEC_IO_Write(DeviceAddr, 0x27, 0x3F | 0x140);
    }
    else if (Volume == 0)
    {
      /* Mute audio codec */
      counter += wm8994_SetMute(DeviceAddr, CODEC_MUTE);
    }
    else
    {
      /* Unmute audio codec */
      counter += wm8994_SetMute(DeviceAddr, CODEC_UNMUTE);

      /* Left Headphone Volume */
      counter += CODEC_IO_Write(DeviceAddr, 0x1C, convertedvol | 0x140);

      /* Right Headphone Volume */
      counter += CODEC_IO_Write(DeviceAddr, 0x1D, convertedvol | 0x140);

      /* Left Speaker Volume */
      counter += CODEC_IO_Write(DeviceAddr, 0x26, convertedvol | 0x140);

      /* Right Speaker Volume */
      counter += CODEC_IO_Write(DeviceAddr, 0x27, convertedvol | 0x140);
    }
  }

  /* Input volume */
  if (inputEnabled != 0)
  {
    convertedvol = CODEC_VOLUME_IN_CONVERT(Volume);

    /* Left AIF1 ADC1 volume */
    counter += CODEC_IO_Write(DeviceAddr, 0x400, convertedvol | 0x100);

    /* Right AIF1 ADC1 volume */
    counter += CODEC_IO_Write(DeviceAddr, 0x401, convertedvol | 0x100);

    /* Left AIF1 ADC2 volume */
    counter += CODEC_IO_Write(DeviceAddr, 0x404, convertedvol | 0x100);

    /* Right AIF1 ADC2 volume */
    counter += CODEC_IO_Write(DeviceAddr, 0x405, convertedvol | 0x100);
  }
  return counter;
}

/**
  * @brief Enables or disables the mute feature on the audio codec.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param mute: AUDIO_MUTE_ON to enable the mute or AUDIO_MUTE_OFF to disable the
  *             mute mode.
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_SetMute(uint16_t DeviceAddr, wm8994_mute_t mute)
{
  int counter = 0;
  
  if (outputEnabled != 0)
  {
    /* Set the Mute mode */
    if(mute == CODEC_MUTE)
    {
      /* Soft Mute the AIF1 Timeslot 0 DAC1 path L&R */
      counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0200);

      /* Soft Mute the AIF1 Timeslot 1 DAC2 path L&R */
      counter += CODEC_IO_Write(DeviceAddr, 0x422, 0x0200);
    }
    else /* Disable the Mute */
    {
      /* Unmute the AIF1 Timeslot 0 DAC1 path L&R */
      counter += CODEC_IO_Write(DeviceAddr, 0x420, 0x0000);

      /* Unmute the AIF1 Timeslot 1 DAC2 path L&R */
      counter += CODEC_IO_Write(DeviceAddr, 0x422, 0x0000);
    }
  }
  return counter;
}

/**
  * @brief Switch dynamically (while audio file is played) the output target 
  *         (speaker or headphone).
  * @param DeviceAddr: Device address on communication Bus.
  * @param Output: specifies the audio output target: OUTPUT_DEVICE_SPEAKER,
  *         OUTPUT_DEVICE_HEADPHONE, OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO 
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_SetOutputMode(uint16_t DeviceAddr, uint8_t Output)
{
  int counter = 0;
  
  switch (Output) 
  {
  case CODEC_OUT_SPEAKER:
    /* Enable DAC1 (Left), Enable DAC1 (Right), 
    Disable DAC2 (Left), Disable DAC2 (Right)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0C0C);
    
    /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0000);
    
    /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0000);
    
    /* Disable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0002);
    
    /* Disable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0002);
    break;
    
  case CODEC_OUT_HEADPHONE:
    /* Disable DAC1 (Left), Disable DAC1 (Right), 
    Enable DAC2 (Left), Enable DAC2 (Right)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303);
    
    /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);
    
    /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);
    
    /* Disable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0000);
    
    /* Disable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0000);
    break;
    
  case CODEC_OUT_BOTH:
    /* Enable DAC1 (Left), Enable DAC1 (Right), 
    also Enable DAC2 (Left), Enable DAC2 (Right)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303 | 0x0C0C);
    
    /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);
    
    /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);
    
    /* Enable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0002);
    
    /* Enable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0002);
    break;
    
  default:
    /* Disable DAC1 (Left), Disable DAC1 (Right), 
    Enable DAC2 (Left), Enable DAC2 (Right)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x0303);
    
    /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x601, 0x0001);
    
    /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x602, 0x0001);
    
    /* Disable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x604, 0x0000);
    
    /* Disable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
    counter += CODEC_IO_Write(DeviceAddr, 0x605, 0x0000);
    break;    
  }  
  return counter;
}

/**
  * @brief Sets new frequency.
  * @param DeviceAddr: Device address on communication Bus.
  * @param AudioFreq: Audio frequency used to play the audio stream.
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_SetFrequency(uint16_t DeviceAddr, uint32_t AudioFreq)
{
  int counter = 0;
 
  /*  Clock Configurations */
  switch (AudioFreq)
  {
  case  SAI_AUDIO_FREQUENCY_8K:
    /* AIF1 Sample Rate = 8 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0003);
    break;
    
  case  SAI_AUDIO_FREQUENCY_16K:
    /* AIF1 Sample Rate = 16 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0033);
    break;
    
  case  SAI_AUDIO_FREQUENCY_48K:
    /* AIF1 Sample Rate = 48 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0083);
    break;
    
  case  SAI_AUDIO_FREQUENCY_96K:
    /* AIF1 Sample Rate = 96 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x00A3);
    break;
    
  case  SAI_AUDIO_FREQUENCY_11K:
    /* AIF1 Sample Rate = 11.025 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0013);
    break;
    
  case  SAI_AUDIO_FREQUENCY_22K:
    /* AIF1 Sample Rate = 22.050 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0043);
    break;
    
  case  SAI_AUDIO_FREQUENCY_44K:
    /* AIF1 Sample Rate = 44.1 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0073);
    break; 
    
  default:
    /* AIF1 Sample Rate = 48 (KHz), ratio=256 */ 
    counter += CODEC_IO_Write(DeviceAddr, 0x210, 0x0083);
    break; 
  }
  return counter;
}

/**
  * @brief Resets wm8994 registers.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
int wm8994_Reset(uint16_t DeviceAddr)
{
  int counter;
  
  /* Reset Codec by writing in 0x0000 address register */
  counter = CODEC_IO_Write(DeviceAddr, 0x0000, 0x0000);
  outputEnabled = 0;
  inputEnabled=0;

  return counter;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
