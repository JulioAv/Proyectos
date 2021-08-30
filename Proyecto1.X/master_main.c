/*
 * Archivo:   master_main.c
 * Dispositivo: PIC16F887
 * Autor: Margareth Vela 
 * 
 * Programa: I2C
 * Hardware: LCD en PORTD y FTDI en PORTC.
 * 
 * Creado: Agosto 22, 2021
 * Última modificación: Agosto, 2021
 */

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------
#include <xc.h>
#include <stdint.h>
#include "I2C.h"
#include "LCD.h" 

//------------------------------------------------------------------------------
//                          Directivas del compilador
//------------------------------------------------------------------------------
#define _XTAL_FREQ 8000000 //Oscilador

//------------------------------------------------------------------------------
//                          Variables
//------------------------------------------------------------------------------
uint8_t Primer_digito = 0; //Para conversion a decimal
uint8_t Segundo_digito = 0;
uint8_t Tercer_digito = 0;

uint8_t slave1;
uint8_t slave2;
uint8_t sensor1;
uint8_t sensor2;
uint8_t sensor3;

uint8_t sensor1_centenas;
uint8_t sensor1_decenas;
uint8_t sensor1_unidades;

uint8_t sensor2_centenas;
uint8_t sensor2_decenas;
uint8_t sensor2_unidades;

uint8_t sensor3_centenas;
uint8_t sensor3_decenas;
uint8_t sensor3_unidades;

uint8_t u_flag = 1; //Variables para el valor que recibe el pic 
uint8_t d_flag = 0;
uint8_t c_flag = 0;
uint8_t unidad = 0;
uint8_t decena = 0;
uint8_t valor = 0;
uint8_t var_temp = 0; //Variables temporales
uint8_t cont_temp = 0;

uint16_t cont = 0;
uint8_t val = 0;
uint8_t flag = 0;
//------------------------------------------------------------------------------
//                          Palabras de configuración
//------------------------------------------------------------------------------
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT//Oscillator Selection bits(INTOSCIO 
                              //oscillator: I/O function on RA6/OSC2/CLKOUT pin, 
                              //I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled and 
                          //can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR  
                                //pin function is digital input, MCLR internally 
                                //tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                //protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                //protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                //Internal/External Switchover mode is disabled
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                //(Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                //Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                //(Write protection off)

//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------
void setup(void);  //Configuración
void Decimal(uint8_t variable); //Conversion a decimal

//------------------------------------------------------------------------------
//                          Código Principal
//------------------------------------------------------------------------------
void main(void) {
    setup(); 
    Lcd_Init();
    while(1){              
        I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x51);     //Direccion de lectura del primer esclavo
        slave1 = I2C_Master_Read(0); //Se agrega el valor del potenciometro
        I2C_Master_Stop();          //Termina la comunicacion 
        __delay_ms(200);
        
        I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x31);     //Direccion de lectura del segundo esclavo
        slave2 = I2C_Master_Read(0); //Se agrega el valor del contador
        I2C_Master_Stop();          //Termina la comunicacion 
        __delay_ms(200);
        
        /*I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x72);     //Direccion de lectura del primer esclavo
        I2C_Master_Write(0x80);     //Configuracion del sensor
        I2C_Master_Write(0x07);     //Configuracion del sensor
        I2C_Master_Stop();          //Termina la comunicacion 
        __delay_ms(200);
        
        I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x72);     //Direccion de lectura del primer esclavo
        I2C_Master_Write(0x9C);     //Configuracion del sensor
        I2C_Master_RepeatedStart();
        I2C_Master_Write(0x73);     //Direccion de lectura del primer esclavo
        sensor1 = I2C_Master_Read(0); //Se agrega el valor del contador
        I2C_Master_Stop();          //Termina la comunicacion 
        __delay_ms(200);*/
                
        if(val == 0){
            CCPR1L = (0x00>>1) + 128; //Swift y ajuste de señal
            __delay_ms(18);
        }
        
        if(val == 1){
            CCPR1L = (0x7f>>1) + 128; //Swift y ajuste de señal
            __delay_ms(18);
        }
               
        Decimal(255);
        sensor1_centenas= Primer_digito; 
        sensor1_decenas= Segundo_digito; 
        sensor1_unidades= Tercer_digito; 
        Decimal(80);
        sensor2_centenas= Primer_digito; 
        sensor2_decenas= Segundo_digito; 
        sensor2_unidades= Tercer_digito;
        Decimal(100);
        sensor3_centenas= Primer_digito; 
        sensor3_decenas= Segundo_digito; 
        sensor3_unidades= Tercer_digito;
        
        Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("S1:");
        Lcd_Write_String(" ");
        Lcd_Write_String("S2:");
        Lcd_Write_String(" ");
        Lcd_Write_String("OP1");
        Lcd_Write_String(" ");
        Lcd_Write_String("OP2");
        
        Lcd_Set_Cursor(2,1);
        Lcd_Write_Char(sensor1_centenas);
        Lcd_Write_Char(sensor1_decenas);
        Lcd_Write_Char(sensor1_unidades);
        Lcd_Write_String(" ");
        Lcd_Write_Char(sensor2_centenas);
        Lcd_Write_Char(sensor2_decenas);
        Lcd_Write_Char(sensor2_unidades);
        Lcd_Write_String(" ");
        Decimal(50);
        Lcd_Write_Char(Primer_digito);
        Lcd_Write_Char(Segundo_digito);
        Lcd_Write_Char(Tercer_digito);
        Lcd_Write_String(" ");
        if(flag){
            Decimal(PORTA);
        }
        else { 
            Decimal(40);
        }
        Lcd_Write_Char(Primer_digito);
        Lcd_Write_Char(Segundo_digito);
        Lcd_Write_Char(Tercer_digito);
        
                
        if (PIR1bits.TXIF){
                    __delay_ms(20);
                    TXREG = sensor1_centenas; //Envía el valor de centenas del contador
                    __delay_ms(20);
                    TXREG = sensor1_decenas; //Envía el valor de decenas del contador
                    __delay_ms(20);
                    TXREG = sensor1_unidades; //Envía el valor de unidades del contador
                    __delay_ms(20);
                    TXREG = 0x2C; //Envia el valor de enter
                    __delay_ms(20);
                    TXREG = sensor2_centenas; //Envía el valor de centenas del contador
                    __delay_ms(20);
                    TXREG = sensor2_decenas; //Envía el valor de decenas del contador
                    __delay_ms(20);
                    TXREG = sensor2_unidades; //Envía el valor de unidades del contador
                    __delay_ms(20);
                    TXREG = 0x2C; //Envia el valor de enter
                    __delay_ms(20);
                    TXREG = sensor3_centenas; //Envía el valor de centenas del contador
                    __delay_ms(20);
                    TXREG = sensor3_decenas; //Envía el valor de decenas del contador
                    __delay_ms(20);
                    TXREG = sensor3_unidades; //Envía el valor de unidades del contador
                    __delay_ms(20);
                    TXREG = 0x0D; //Envia el valor de enter
                    __delay_ms(20);
            }
        PIR1bits.TXIF = 0; //Se limpia la bandera
    }
    return;
}

//------------------------------------------------------------------------------
//                          Interrupciones
//------------------------------------------------------------------------------
void __interrupt()isr(void){
     
    if (INTCONbits.T0IF){           // INTERRUPCION TMR0
        cont--;
        if(cont == 0){
            val = 0;
        }
        TMR0 = 248;
        INTCONbits.T0IF = 0;        // Limpiar la bandera de interrupción TMR0
    }
    
    if(INTCONbits.RBIF){
        if(PORTBbits.RB1 == 1){
            if(flag){
               cont = PORTA*1000;
           }
           else{
               cont = 5*1000;
           }
           val = 1;   
        }
      INTCONbits.RBIF = 0;
    }
    
    if(PIR1bits.RCIF == 1){ //Empieza a recibir datos del USART
        if (RCREG ==  0x0D){
        PORTA = valor; //Se coloca el valor que recibe el pic a PORTA
        cont_temp = 0;
        valor = 0;
        u_flag = 1;
        d_flag = 0;
        c_flag = 0;
        flag = 1;
        }
        
        if (RCREG !=  0x0D){
        var_temp = RCREG; //Cambio de valor ASCII a decimal
            if(var_temp==48){
                cont_temp = 0;
            }else if(var_temp==49){
                cont_temp = 1;
            }else if(var_temp==50){
                cont_temp = 2;
            }else if(var_temp==51){
                cont_temp = 3;
            }else if(var_temp==52){
                cont_temp = 4;
            }else if(var_temp==53){
                cont_temp = 5;
            }else if(var_temp==54){
                cont_temp = 6;
            }else if(var_temp==55){
                cont_temp = 7;
            }else if(var_temp==56){
                cont_temp = 8;
            }else if(var_temp==57){
                cont_temp = 9;
            }
        if (u_flag == 1){
            valor = cont_temp;
            unidad = cont_temp;
            u_flag = 0;
            d_flag = 1; //Se enciende el valor de decenas
        }
        else if (d_flag == 1){
            valor = (unidad*10)+cont_temp;
            decena = cont_temp;
            d_flag = 0;
            c_flag = 1; //Se enciende el valor de centenas
        }
        else if (c_flag == 1){
            valor = (unidad*100)+(decena*10)+cont_temp;
        }     
        }
        
        if(RCREG == 115){ //s -> stop
            flag =0;
        }
    }
    
}

//------------------------------------------------------------------------------
//                          Subrutinas
//------------------------------------------------------------------------------
void Decimal(uint8_t variable){        // Función para obtener valor decimal
    uint8_t valor;
    valor = variable;                  
    Primer_digito = (valor/100) ;                //Valor del tercer digito
    valor = (valor - (Primer_digito*100));
    Segundo_digito = (valor/10);              //Valor del segundo digito
    valor = (valor - (Segundo_digito*10));
    Tercer_digito = valor;                //Valor del primer digito
    
    Primer_digito = Primer_digito + 48;          //Conversion a ascii
    Segundo_digito = Segundo_digito + 48;
    Tercer_digito = Tercer_digito + 48;
    
}

//------------------------------------------------------------------------------
//                          Configuración
//------------------------------------------------------------------------------
void setup(void){
    //Configuracion reloj
    OSCCONbits.IRCF2 = 1; //Frecuencia a 8MHZ
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    
    //Configurar entradas y salidas
    ANSELH = 0x00;//Pines digitales
    ANSEL = 0x00; //Pines digitales
    
    TRISA = 0x00; //Salidas
    TRISB = 0x03;
    TRISC = 0x80; 
    TRISD = 0x00; 
    TRISE = 0x00; 
    
    OPTION_REGbits.nRBPU =  0 ; //Se habilita el pull-up interno en PORTB
    WPUB = 0x03;  //Se habilita los pull ups para los pines
    
    PORTA = 0x00; //Se limpian los puertos
    PORTC = 0x00;    
    PORTD = 0x00;
    PORTE = 0x00;
    
    //Configurar la interrupcion
    INTCONbits.GIE = 1;  //Enable interrupciones globales   
    INTCONbits.PEIE = 1; //Enable de periféricos
    INTCONbits.T0IE = 1;
    INTCONbits.T0IF = 0;
    PIE1bits.RCIE = 1;   //Enable RC
    PIR1bits.RCIF = 0;   //Se limpia la bandera
    
    //Interrupt on change
    PORTB = 0x00;
    IOCB = 0x03; // setear interrupciones de todos los pines   
    INTCONbits.RBIF = 0;
    
    //Configuracion PWM
    PR2 = 250; //Valor inicial de PR2
    CCP1CONbits.P1M = 0; //PWM bits de salida
    CCP1CONbits.CCP1M = 0b00001100; //Se habilita PWM   
    
    CCPR1L = 0x0F; 
    CCP1CONbits.DC1B = 0; //Bits menos significativos del Duty Cycle
    
    PIR1bits.TMR2IF = 0; //Se limpia la bandera
    T2CONbits.T2CKPS1 = 1; //Prescaler de 16
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1; //Se enciende el TMR2
    
    while (!PIR1bits.TMR2IF); //Se espera una interrupción
    PIR1bits.TMR2IF = 0;
    
    //Configuración de TX y RX
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    
    BAUDCTLbits.BRG16 = 1;
    
    SPBRG = 207;
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;//Configuración del USART y Baud Rate
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    
    TXSTAbits.TXEN = 1; 
    
    //Configurar TMR0
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS2 = 1; //Prescaler 1:256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    TMR0 = 248;
    
   I2C_Master_Init(100000); // Se inicializa la frecuencia del master a 100kHz
}

