/**
 ******************************************************************************
 * @file   CO5300.h
 * @author Sifli software development team
 * @brief   This file contains all the functions prototypes for the CO5300.c
 *          driver.
 ******************************************************************************
 */
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __CO5300_H
#define __CO5300_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "../common/lcd.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @addtogroup CO5300
 * @{
 */

/** @defgroup CO5300_Exported_Types
 * @{
 */
/**
 * @}
 */

/** @defgroup CO5300_Exported_Constants
 * @{
 */

/**
 * @brief CO5300 chip IDs
 */
#define CO5300_ID 0x009C01

/**
 * @brief  CO5300 Size
 */
#define CO5300_LCD_PIXEL_WIDTH (466)
#define CO5300_LCD_PIXEL_HEIGHT (466)

/**
 *  @brief LCD_OrientationTypeDef
 *  Possible values of Display Orientation
 */
#define CO5300_ORIENTATION_PORTRAIT (0x00)         /* Portrait orientation choice of LCD screen  */
#define CO5300_ORIENTATION_LANDSCAPE (0x01)        /* Landscape orientation choice of LCD screen */
#define CO5300_ORIENTATION_LANDSCAPE_ROT180 (0x02) /* Landscape rotated 180 orientation choice of LCD screen */

/**
 * @brief  CO5300 Registers
 */
#define CO5300_SW_RESET 0x01
#define CO5300_LCD_ID 0x04
#define CO5300_DSI_ERR 0x05
#define CO5300_POWER_MODE 0x0A
#define CO5300_SLEEP_IN 0x10
#define CO5300_SLEEP_OUT 0x11
#define CO5300_PARTIAL_DISPLAY 0x12
#define CO5300_DISPLAY_INVERSION 0x21
#define CO5300_DISPLAY_OFF 0x28
#define CO5300_DISPLAY_ON 0x29
#define CO5300_WRITE_RAM 0x2C
#define CO5300_READ_RAM 0x2E
#define CO5300_CASET 0x2A
#define CO5300_RASET 0x2B
#define CO5300_PART_CASET 0x30
#define CO5300_PART_RASET 0x31
#define CO5300_VSCRDEF 0x33 /* Vertical Scroll Definition */
#define CO5300_VSCSAD 0x37  /* Vertical Scroll Start Address of RAM */
#define CO5300_TEARING_EFFECT 0x35
#define CO5300_NORMAL_DISPLAY 0x36
#define CO5300_IDLE_MODE_OFF 0x38
#define CO5300_IDLE_MODE_ON 0x39
#define CO5300_COLOR_MODE 0x3A
#define CO5300_CONTINUE_WRITE_RAM 0x3C
#define CO5300_WBRIGHT 0x51 /* Write brightness*/
#define CO5300_RBRIGHT 0x53 /* Read brightness*/
#define CO5300_PORCH_CTRL 0xB2
#define CO5300_FRAME_CTRL 0xB3
#define CO5300_GATE_CTRL 0xB7
#define CO5300_VCOM_SET 0xBB
#define CO5300_LCM_CTRL 0xC0
#define CO5300_SET_TIME_SRC 0xC2
#define CO5300_SET_DISP_MODE 0xC4
#define CO5300_VCOMH_OFFSET_SET 0xC5
#define CO5300_FR_CTRL 0xC6
#define CO5300_POWER_CTRL 0xD0
#define CO5300_PV_GAMMA_CTRL 0xE0
#define CO5300_NV_GAMMA_CTRL 0xE1
#define CO5300_SPI2EN 0xE7

  /**
   * @}
   */

  /** @defgroup CO5300_Exported_Functions
   * @{
   */
  void CO5300_Init(LCDC_HandleTypeDef *hlcdc);
  uint32_t CO5300_ReadID(LCDC_HandleTypeDef *hlcdc);

  void CO5300_DisplayOn(LCDC_HandleTypeDef *hlcdc);
  void CO5300_DisplayOff(LCDC_HandleTypeDef *hlcdc);
  void CO5300_Sleep_In(LCDC_HandleTypeDef *hlcdc);
  void CO5300_Sleep_Out(LCDC_HandleTypeDef *hlcdc);

  void CO5300_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1);
  void CO5300_WritePixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos, const uint8_t *RGBCode);
  void CO5300_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1);

  uint32_t CO5300_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos);
  void CO5300_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode);
  void CO5300_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t bright);
  void CO5300_IdleModeOn(LCDC_HandleTypeDef *hlcdc);
  void CO5300_IdleModeOff(LCDC_HandleTypeDef *hlcdc);

  /* LCD driver structure */

#ifdef __cplusplus
}
#endif

#endif /* __CO5300_H */
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
