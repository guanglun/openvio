#ifndef __LCD_INIT_H__
#define __LCD_INIT_H__

#include "main.h"

#define USE_HORIZONTAL 0  //0或1为竖屏 2或3为横屏


#define LCD_W 240
#define LCD_H 240

void lcd_spi_init(void);
void lcd_init(void);
void LCD_WR_DATA8(uint8_t dat);
void LCD_WR_DATA(uint16_t dat);
void LCD_WR_REG(uint8_t dat);
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void LCD_Writ_Buffer(uint8_t *buf,uint16_t size);


#endif
