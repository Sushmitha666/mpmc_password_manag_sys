#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>

#define SSD1306_I2C_ADDR 0x78

extern I2C_HandleTypeDef hi2c1;

void ssd1306_Init(void);
void ssd1306_Fill(uint8_t color);
void ssd1306_UpdateScreen(void);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_WriteChar(char ch);
void ssd1306_WriteString(char *str);

#endif
