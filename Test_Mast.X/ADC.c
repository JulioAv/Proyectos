/*
 * File:   ADC.c
 * Author: swimm
 *
 * Created on 15 de julio de 2021, 03:34 PM
 */
#define _XTAL_FREQ 8000000
#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))


#include <xc.h>
#include "adc.h"

void ADC_CONFIG(int frec){
    
    ADCON0bits.ADON = 1;
    ADCON0bits.CHS = 0;
    ADCON1bits.ADFM = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.GO = 0;
    
    switch(frec){
        case 1:
            ADCON0bits.ADCS = 0B00;
            break;
        case 4:
            ADCON0bits.ADCS = 0B01;
            break;
        case 8:
            ADCON0bits.ADCS = 0B10;
            break;
    }
}

void ADC_IF(void){
    if(ADCON0bits.GO == 0){
            __delay_us(50);
            ADCON0bits.GO = 1;
        }
        
}
