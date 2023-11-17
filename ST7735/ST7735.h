#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdarg.h"

#include "struct_typedef.h"

#include "spi.h"

/**
 * SPI2_CS      CS      PB12
 * SPI2_CLK     SCK     PB13
 * SPI2_MOSI    SDA     PB15
 * SPI2_MISO    RST     PB14
 * I2C2_SDA     DC      PF0
 */

extern const unsigned char ASCII_12_32[][96];
extern const unsigned char ASCII_8_16[][16];
extern const unsigned char JetBrainMono[][16];

#define LCD_W 128
#define LCD_H 160
#define TotalPixel (LCD_W * LCD_H)
#define SizeOfGram (2 * TotalPixel)

#define lcd_delay_ms(ms) HAL_Delay(ms)

#define SetPortLcdDCCmd() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET)
#define SetPortLcdDCData() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET)

#define SetPortLcdResetLow() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define SetPortLcdResetHigh() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)

#define TransColor888to565(Color)                \
    (uint16_t)                                   \
    {                                            \
        ((uint16_t)((Color & 0x00F80000) >> 8) | \
         (uint16_t)((Color & 0x0000FC00) >> 5) | \
         (uint16_t)((Color & 0x000000F8) >> 3))  \
    }

void Compile_Test(void);
uint16_t LCD_SetColor(uint32_t Color);
void LCD_SetRegion(uint16_t x, uint16_t x2, uint16_t y, uint16_t y2);
void LCD_Fill(uint16_t color);
void LCD_Upgrade_Gram(void);
void LCD_Fill_Region(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color);
void LCD_DrawPoint(uint32_t x, uint32_t y, uint16_t color);
void LCD_DrawLine(uint32_t x1, uint32_t x2, uint32_t y1, uint32_t y2, uint16_t color);
void LCD_DrawFromIMG(uint32_t x, uint32_t y, const uint8_t *IMG, uint8_t IMG_W, uint8_t IMG_H, uint16_t color);
void LCD_DrawFromIMGWithBackGround(uint32_t x, uint32_t y, const uint8_t *IMG, uint8_t IMG_W, uint8_t IMG_H, uint16_t color, uint16_t BGcolor);
void LCD_Print(uint32_t x, uint32_t y, uint32_t font_num, uint16_t color, char *fmt, ...);
void LCD_PrintWithBackGroundColor(uint32_t x, uint32_t y, uint32_t font_num, uint16_t color, uint16_t BGcolor, char *fmt, ...);
void LCD_DrawPic(const unsigned char *pic, uint32_t W, uint32_t H);
void LCD_FastCircle(uint32_t x, uint32_t y, uint32_t r, uint32_t color);
void LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r, uint32_t color);
void LCD_OCS(int32_t data);
void Lcd_Init(void);

void Monitor_Frame_Upgrade(void *arg);

