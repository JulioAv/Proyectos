/*
 * File:   Master_Test.c
 * Author: swimm
 *
 * Created on 23 de agosto de 2021, 01:44 PM
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

#include "I2C.h"
#include <xc.h>
#include "LCD.h"
#include "adc.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

char pot, con, buffer[], cen, dec, uni, temp, luz, presion2, var;
int frec, tr, rc, presion1;

char my_delay_s(char tiempo){
    while(tiempo != 0){
        tiempo--;
        __delay_ms(1000);
    }
}

void Division(char y){
        cen = (y/100);
        dec = ((y%100)/10);
        uni = ((y%100)%10);
}

void USART_CONFIG(int frec, tr, rc){
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
        PIE1bits.RCIE = 1;
        PIR1bits.RCIF = 0;
    }
    else{
        RCSTAbits.CREN = 0;
    }
}

void UART_write(unsigned char* word){   //Funcin que transmite datos
    while (*word != 0){                 //Verifica que el puntero aumente
        TXREG = (*word);                //Envía el caracter que toca de la cadena
        while(!TXSTAbits.TRMT);         //Espera a que se haya enviado el dato
        word++;                         //Aumenta el apuntador para ir al
    }                                   //siguente caracter
    return;
}

void __interrupt()isr(void){
    if(ADIF){
        if(ADCON0bits.CHS == 0){
            pot = ADRESH;
        }
        ADIF = 0;
    }
    if(RBIF){
        if(RB0==1){
            con++;
        }
        else if(RB1==1){
            con--;
        }
        RBIF = 0;
    }
    if(RCIF){
        if(RCREG == '1'){
            TXREG = (presion1);
            while(!TXSTAbits.TRMT);
            TXREG = temp*397/255;
            while(!TXSTAbits.TRMT);
            TXREG = luz;
            while(!TXSTAbits.TRMT);
        }
        else{
            presion2 = RCREG;
            
        }
        RCIF = 0;
    }
}

void setup(void) {
    ANSEL = 0x01;
    ANSELH = 0x00;
    
    TRISA = 0x01;
    TRISB = 0x03;
    TRISC = 0x00;
    TRISD = 0x00;
    
    OSCCONbits.IRCF = 0B111;
    OSCCONbits.OSTS = 0;
    OSCCONbits.SCS = 1;
    
    ADC_CONFIG(8);
    PIE1bits.ADIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    USART_CONFIG(8 ,1, 1);
    
    INTCONbits.RBIE = 1;
    WPUB = 0x03;
    IOCB = 0x03;
    OPTION_REGbits.nRBPU = 0;
    
    I2C_Master_Init(100000);
    Lcd_Init();
    Lcd_Clear();
    Lcd_Set_Cursor(1,0);
    Lcd_Write_String("Tem Luz t(s) TO");
    //UART_write("Presión atmosférica");
    presion2 = 110;
}


void main(void){
    setup();
    while(1){
        
        ADC_IF();
                
        I2C_Master_Start();
        I2C_Master_Write(0x51);
        temp = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(500);
        
//        I2C_Master_Start();
//        I2C_Master_Write(0x51);
//        temp = I2C_Master_Read(0);
//        I2C_Master_Stop();
//        __delay_ms(50);
        
        Division((pot*50/255));
        Lcd_Set_Cursor(2,13);
        //sprintf(buffer, "%d", cen);
        //Lcd_Write_String(buffer);
        sprintf(buffer, "%d", dec);
        Lcd_Write_String(buffer);
        sprintf(buffer, "%d", uni);
        Lcd_Write_String(buffer);
        
        
        Division(temp*397/255);
        Lcd_Set_Cursor(2,1);
        //sprintf(buffer, "%d", cen);
        //Lcd_Write_String(buffer);
        sprintf(buffer, "%d", dec);
        Lcd_Write_String(buffer);
        sprintf(buffer, "%d", uni);
        Lcd_Write_String(buffer);
        
        I2C_Master_Start();
        I2C_Master_Write(0x31);
        luz = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(50);
        
//        I2C_Master_Start();
//        I2C_Master_Write(0x31);
//        luz = I2C_Master_Read(0);
//        I2C_Master_Stop();
//        __delay_ms(50);
        
        Division(con);
        Lcd_Set_Cursor(2,8);
        sprintf(buffer, "%d", cen);
        Lcd_Write_String(buffer);
        sprintf(buffer, "%d", dec);
        Lcd_Write_String(buffer);
        sprintf(buffer, "%d", uni);
        Lcd_Write_String(buffer);
        
        I2C_Master_Start();
        I2C_Master_Write(0xEE);
        I2C_Master_Write(0B11110110);
        I2C_Master_Stop();
        __delay_ms(50);
        
        I2C_Master_Start();
        I2C_Master_Write(0xEE | 1);
        var = I2C_Master_Read(0);
        presion1 = var<<8;
        var = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(50);
        presion1 = presion1 & var;
        presion1 = presion1+968;
        presion1 = presion1/1000;
        
        
        
        if(luz==0){
            
            Lcd_Set_Cursor(2, 4);
            Lcd_Write_String("No");
            
            I2C_Master_Start();
            I2C_Master_Write(0x50);
            I2C_Master_Write(0x01);
            I2C_Master_Stop();
            my_delay_s(con);
            
            I2C_Master_Start();
            I2C_Master_Write(0x50);
            I2C_Master_Write(0x00);
            I2C_Master_Stop();
            __delay_ms(50);
        }
        else if(luz==1){
            Lcd_Set_Cursor(2, 4);
            Lcd_Write_String("Si");
        }
        
        if((temp*400/255)>=(pot*50/255)){
            I2C_Master_Start();
            I2C_Master_Write(0x30);
            I2C_Master_Write(0xf0);
            I2C_Master_Stop();
            __delay_ms(50);
        }
        else if((temp*400/255)<(pot*50/255)){
            I2C_Master_Start();
            I2C_Master_Write(0x30);
            I2C_Master_Write(0x00);
            I2C_Master_Stop();
            __delay_ms(50);
        }
        
        if((presion1)>=(presion2)){
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("PRECAUCION");
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("PRESION ALTA");
            while((presion1)>=(presion2));
            Lcd_Clear();
            Lcd_Set_Cursor(1,0);
            Lcd_Write_String("Tem Luz t(s) TO");
            
        }
//        Division(presion1);
//        sprintf(buffer, "%d", cen);
//        UART_write(buffer);
//        sprintf(buffer, "%d", dec);
//        UART_write(buffer);
//        sprintf(buffer, "%d", uni);
//        UART_write(buffer);
       
    }
}
