/**
  ******************************************************************************
  * @file    wm8994.h
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    22-February-2016
  * @brief   This file contains all the functions prototypes for the 
  *          wm8994.c driver.
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
#ifndef __WM8994_H
#define __WM8994_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <sai.h>


#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)

/******************************************************************************/
/***************************  Codec User defines ******************************/
/******************************************************************************/
/* Codec output DEVICE */
#define CODEC_OUT_SPEAKER         0x0001U
#define CODEC_OUT_HEADPHONE       0x0002U
#define CODEC_OUT_BOTH            0x0004U
#define CODEC_OUT_AUTO            0x0008U
#define CODEC_IN_DIGITAL_MIC1     0x0100U
#define CODEC_IN_DIGITAL_MIC2     0x0200U
#define CODEC_IN_LINE_1           0x0400U
#define CODEC_IN_LINE_2           0x0800U

/* Volume Levels values */
#define CODEC_VOLMIN                0x00
#define CODEC_VOLMAX                0xFF
#define CODEC_VOLSTEP               0x04

/* Codec POWER DOWN modes */
typedef enum  {
    CODEC_PWDN_HW,
    CODEC_PWDN_SW
} codec_pwdn_t;

/* MUTE commands */
typedef enum {
    CODEC_MUTE,
    CODEC_UNMUTE
} wm8994_mute_t;

#define CODEC_VOLUME_CONVERT(Volume)        (((Volume) > 100)? 100:((uint8_t)(((Volume) * 63) / 100)))
#define CODEC_VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 240) / 100)))

#define  CODEC_WM8994_ID    0x8994

#define CODEC_WM8994_CHIPID_ADDR                  0x00


/*------------------------------------------------------------------------------
                           Audio Codec functions 
------------------------------------------------------------------------------*/
/* High Layer codec functions */
int wm8994_Init(uint16_t DeviceAddr, uint16_t OutputInputDevice, uint8_t Volume, uint32_t AudioFreq);
uint32_t wm8994_ReadID(uint16_t DeviceAddr);
int wm8994_Play(uint16_t DeviceAddr);
int wm8994_Pause(uint16_t DeviceAddr);
int wm8994_Resume(uint16_t DeviceAddr);
int wm8994_Stop(uint16_t DeviceAddr, codec_pwdn_t pwdn);
int wm8994_SetVolume(uint16_t DeviceAddr, uint8_t Volume);
int wm8994_SetMute(uint16_t DeviceAddr, wm8994_mute_t mute);
int wm8994_SetOutputMode(uint16_t DeviceAddr, uint8_t Output);
int wm8994_SetFrequency(uint16_t DeviceAddr, uint32_t AudioFreq);
int wm8994_Reset(uint16_t DeviceAddr);

/* CODEC IO functions */
uint8_t CODEC_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
uint8_t CODEC_IO_Read(uint8_t Addr, uint16_t Reg);
void CODEC_IO_Delay(uint32_t Delay);

#endif /* __WM8994_H */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
