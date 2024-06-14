/*
 * lcd.h
 *
 *  Created on: Jun 10, 2024
 *      Author: andre
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_
#include "stm32f4xx_hal.h"

#define MODULE_ADDR 0x27 << 1  //endereço slave I2C
#define FUNCTION_SET 0x28  //DL=0(4-bit) N=1(2 lines) F=0(5x8 5 linhas 8 colunas)
#define DISPLAY_OFF 0x08  //D=0 Display off C=0 cursor off B=0 blinking off
#define DISPLAY_ON 0x0C  //D=1 Display on
#define CLR_DISPLAY 0x01  //Clear display
#define INC_CURSOR 0x06  //Entry mode set Increment e no shift
#define CURSOR_BLINK 0x0F //Cursor visivel e piscando
#define CURSOR_NO_BLINK 0x0E //Cursor visivel e sem piscar
#define SHIFT_LEFT 0x18 //Faz shift para esquerda
#define SHIFT_RIGHT 0x1C //Shift para direita
#define CURSOR_LEFT 0x10 //move cursor para esquerda
#define CURSOR_RIGHT 0x14 //move cursor para direita


void lcd_init();
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_str(char *str);
void lcd_clear();
void lcd_shift_left();
void lcd_shift_right();
void lcd_cursor_blink(uint8_t blink);
void lcd_cursor_left();
void lcd_cursor_right();
void lcd_set_cursor(uint8_t row, uint8_t col);

#endif /* INC_LCD_H_ */
