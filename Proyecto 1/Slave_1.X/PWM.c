/*
 * File:   PWM.c
 * Author: swimm
 *
 * Created on 19 de agosto de 2021, 12:26 PM
 */


#include <xc.h>
#include "PWM.h"

void PWM_CONFIG(void) {
    TRISCbits.TRISC2 = 1;       //Los puertos del PWM como entradas      
    PR2 = 255;                  //Valor para obtener el periodo
    CCP1CONbits.P1M = 0B00;     //Módulo configurado en modo PWM
    CCP1CONbits.CCP1M = 0B1100;
    CCPR1L = 0x0F;              //Valor inicial para el módulo
    CCP1CONbits.DC1B = 0;
    
    T2CONbits.TOUTPS = 0;       //TMR2 configurado sin postescaler y con
    T2CONbits.T2CKPS = 0B11;    //prescaler a 16
    
    T2CONbits.TMR2ON = 1;       //Se inicia el TMR2
    PIR1bits.TMR2IF = 0;
    
    while(PIR1bits.TMR2IF == 0);//Se espera un ciclo del TMR
    PIR1bits.TMR2IF = 0;
    TRISCbits.TRISC2 = 0;       //Se ponen como salidas los pines 
}
