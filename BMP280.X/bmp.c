/*
 * File:   bmp.c
 * Author: swimm
 *
 * Created on 24 de agosto de 2021, 08:42 PM
 */
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
#define _XTAL_FREQ 8000000
#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))

long temperature;
unsigned long pressure;
char buffer[];

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "I2C.h"
#include "BMP280.h"

char pre1, pre2, buffer[], cen, dec, uni;

void USART_CONFIG(char frec, char tr, char rc){
    RCSTAbits.SPEN = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.TX9 = 0;
    RCSTAbits.RX9 = 0;
    switch(frec){
        case 4:
            SPBRG = 25;
            TXSTAbits.BRGH = 1;
            BAUDCTLbits.BRG16 = 0;
            break;
        case 8:
            SPBRG = 12;
            TXSTAbits.BRGH = 0;
            BAUDCTLbits.BRG16 = 0;
            break;     
    }
    if(tr==1){
        TXEN = 1;
    }
    else{
        TXEN = 0;
    }
    if(rc == 1){
        RCSTAbits.CREN = 1;
    }
    else{
        RCSTAbits.CREN = 0;
    }
}

void UART_write(unsigned char* word){   //Función que transmite datos
    while (*word != 0){                 //Verifica que el puntero aumente
        TXREG = (*word);                //Envía el caracter que toca de la cadena
        while(!TXSTAbits.TRMT);         //Espera a que se haya enviado el dato
        word++;                         //Aumenta el apuntador para ir al
    }                                   //siguente caracter
    return;
}

void Division(char y){
        cen = (y/100);
        dec = ((y%100)/10);
        uni = ((y%100)%10);
}

void setup(){
    ANSEL = 0x00;
    ANSELH = 0x00;
    
    TRISA = 0x00;
    TRISB = 0x00;
    TRISC = 0x00;
    TRISD = 0x00;
    
    OSCCONbits.IRCF = 0B111;
    OSCCONbits.OSTS = 0;
    OSCCONbits.SCS = 1;
    
    USART_CONFIG(8, 1, 0);
    
    I2C_Master_Init(100000);
}

void main(void) {
    setup();
    UART_write("medicion: ");
    if(BMP280_begin(0x03, 0x01, 0x01, 0x00, 0x00) == 0)
  {  // connection error or device address wrong!
     UART_write("Connection error!");
     while(1);  // stay here
  }
 
  while(1)
  {
    // Read temperature (in hundredths C) and pressure (in Pa)
    //   values from the BMP280 sensor
    BMP280_readTemperature(&temperature);  // read temperature
    BMP280_readPressure(&pressure);        // read pressure
 
    // print data on the LCD screen
    // 1: print temperature
    if(temperature < 0){
      sprintf(buffer, "Temp:-%d.%d°C ", (abs(temperature)/100), (abs(temperature)%100));
      UART_write(buffer);
    }
    else{
      sprintf(buffer, "Temp: %d.%d°C ", (temperature/100), (temperature%100));
    UART_write(buffer);
    
    // 2: print pressure
    sprintf(buffer, "Pres: %d.%dhPa", (pressure/100), (pressure%100));
    UART_write(buffer);
    }
    
    __delay_ms(2000);  // wait 2 seconds
  }
}
