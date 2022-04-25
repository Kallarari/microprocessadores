/*
 * lcd.h
 *
 *  Created on: Feb 28, 2020
 *      Author: Renan Augusto Starke
 *
 *      Adaptado de AVR e Arduino: Técnicas de Projeto, 2a ed. - 2012.
 *      Instituto Federal de Santa Catarina
 */

#ifndef _LCD_H
#define _LCD_H

#include <msp430.h>
#include <stdint.h>


#define SET(BIT) (1 << BIT)
#define SET_BIT(Y,BIT) (Y |= (BIT))
#define CLR_BIT(Y,BIT) (Y &= ~(BIT))
#define CPL_BIT(Y,BIT) (Y ^= (BIT))
#define TST_BIT(Y,BIT) (Y & (BIT))

/* Configurações de hardware */
/* 0 para via de dados do LCD nos 4 LSBs do PORT empregado (Px0-D4, Px1-D5, Px2-D6, Px3-D7)
 * 1 para via de dados do LCD nos 4 MSBs do PORT empregado (Px4-D4, Px5-D5, Px6-D6, Px7-D7) */
#define DATA_NIBBLE 0

/* Portas */
#define LCD_DATA_PORT P6
#define LCD_CTRL_PORT P3

/* Pinos de controle */
#define E_PIN  BIT1
#define RS_PIN BIT0

typedef enum {LCD_CMD, LCD_DATA} lcd_data_t;

enum DISPLAY_CMDS {
    LCD_TURN_OFF = 0x08,
    LCD_CLEAR = 0x01,
    LCD_LINE_0 = 0x80,
    LCD_LINE_1 = 0xC0
};

/**
  * @brief  Configura hardware.
  * @param  Nenhum
  *
  * @retval Nenhum.
  */
void lcd_init_4bits();

/**
  * @brief  Envia um dado estático para o display: caractere ou comando.
  * @param data: valor do comando.
  * @param data_type: LCD_CMD para comando. LCD_DATA para caractere.
  *
  * @retval Nenhum
  */
void lcd_send_data(uint8_t data, lcd_data_t data_type);

/**
  * @brief  Escreve um string estática (sem printf) no LCD.
  * @param c: ponteiro para a string em RAM
  *
  * @retval Nenhum
  */
void lcd_write_string(char *c);



#endif
