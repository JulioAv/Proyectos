/* 
 * File: LCD.c  
 * Se utiliz� y se adaptaron las librer�as de Ligo George 
 * de la p�gina www.electrosome.com
 * Enlace: https://electrosome.com/lcd-pic-mplab-xc8/
 * Revision history: 
 */

//LCD Functions Developed by electroSome

#define _XTAL_FREQ 8000000
#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))

#include "LCD.h"


void Lcd_Cmd(char a) {
    RS = 0; // => RS = 0 // Dato en el puerto lo va interpretar como comando
    PORTD = a;
    EN = 1; // => E = 1
    __delay_ms(5);
    EN = 0; // => E = 0
}

void Lcd_Clear(void) {
    Lcd_Cmd(0);
    Lcd_Cmd(1);
}

void Lcd_Set_Cursor(char a, char b) {
    if (a == 1){
        Lcd_Cmd((b & 0x0F)|0x80);
    }
	else if (a == 2){
        Lcd_Cmd((b & 0x0F)|0xC0);
    }
	
}

void Lcd_Init(void) {
    PORTD = (0x00);
    __delay_ms(20);
    Lcd_Cmd(0x30);      //0011 0000
    __delay_ms(5);
    Lcd_Cmd(0x30);      //0011 0000
    __delay_ms(11);
    Lcd_Cmd(0x30);      //0011 0000
    /////////////////////////////////////////////////////
    Lcd_Cmd(0x38);      //0011 1000
    Lcd_Cmd(0x10);      //0000 1000
    Lcd_Cmd(0x01);      //0000 0001
    Lcd_Cmd(0x06);      //0000 0110
    Lcd_Cmd(0x10);
    Lcd_Cmd(0x0C);
    __delay_ms(100);
    
}

void Lcd_Write_Char(char a) {
    //temp = a & 0x0F;
    //y = a & 0xF0;
    RS = 1; // => RS = 1
    PORTD = a; //Data transfer
    EN = 1;
    __delay_us(40);
    //EN = 0;
    //Lcd_Port(temp);
    //EN = 1;
    //__delay_us(40);
    EN = 0;
}

void Lcd_Write_String(char *a) {
    int i;
    for (i = 0; a[i] != '\0'; i++)
        Lcd_Write_Char(a[i]);
}

void Lcd_Shift_Right(void) {
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left(void) {
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x08);
}


