#ifndef __LCD_I2C_H
#define __LCD_I2C_H

#include "stm32f3xx_hal.h" // change if using other STM32 family

#define LCD_ADDR (0x27 << 1) // I2C address, check your module

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_SendCmd(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendString(char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);

#endif
