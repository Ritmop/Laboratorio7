/* 
 * File:   Lab7.c
 * Device: PIC16F887
 * Author: Judah Pérez - 21536
 *Compiler: XC8 (v2.40)
 * 
 * Program: TMR0 Interrupt
 * Hardware:
 * 
 * Created: April 11, 2023, 6:03 AM
 * Last updated:
 */

/*--------------------------------- LIBRARIES --------------------------------*/
#include <xc.h>
#include <PIC16F887.h>
#include <stdio.h>
#include <stdlib.h>

/*---------------------------- CONFIGURATION BITS ----------------------------*/
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

/*----------------------- GLOBAL VARIABLES & CONSTANTS -----------------------*/
#define tmr0_n  131
#define tmr2_n  131

uint8_t counter; //Contador
uint8_t pot0_in; //Analog to digital input of pot0
uint8_t pot1_in; //Analog to digital input of pot1

/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
void TMR0_reset(void);
void TMR2_reset(void);

/*------------------------------- RESET VECTOR -------------------------------*/

/*----------------------------- INTERRUPT VECTOR -----------------------------*/
void __interrupt() isr(void){
    if(T0IF){
        //tmr0 - Period set to 1ms
        //counter++;
        TMR0_reset();
        ADCON0bits.GO  = 1; //Start ADC conversion
    }
    if(ADIF){
        //ADC
        //Switch ADC input
        (ADCON0bits.CHS0)?(ADCON0bits.CHS0 = 0):(ADCON0bits.CHS0 = 1);
        (ADCON0bits.CHS1)?(ADCON0bits.CHS1 = 0):(ADCON0bits.CHS1 = 1);
        //Display output on PORT
        if(ADCON0bits.CHS1){
            pot0_in = ADRESH;            
        }
        if(ADCON0bits.CHS0){
            pot1_in = ADRESH;
        }
        
        ADIF = 0;   //Reset flag
    }
    return;
}

/*--------------------------- INTERRUPT SUBROUTINES --------------------------*/

/*---------------------------------- TABLES ----------------------------------*/

/*----------------------------------- SETUP ----------------------------------*/

/*----------------------------------- MAIN -----------------------------------*/
int main(void) {
    setup();
    while(1){
        //Loop
        if(TMR2IF){
            TRISCbits.TRISC2 = 0; //RC2 as output
            TMR2_reset();
        }
        else{
            TRISCbits.TRISC2 = 1; //RC2 as input
            T2CONbits.TMR2ON  = 1;  //Turn ON TMR2 module
        }
        PORTD = pot0_in;
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
    //I/O CONFIG
    TRISAbits.TRISA0 = 0;
    
    ANSELbits.ANS1   = 1; //RA1 as analog
    TRISAbits.TRISA1 = 1; //input
    ANSELbits.ANS2   = 1; //RA2 as analog
    TRISAbits.TRISA2 = 1; //input
    TRISCbits.TRISC2 = 0; //RC2 as output
    
    //TRISC  = 0; //PORTC as output
    TRISD  = 0; //PORTD as output
    TRISE  = 0; //PORTE as output
    PORTC  = 0; //Clear PORTC
    PORTD  = 0; //Clear PORTD
    PORTE  = 0; //Clear PORTE
    
    //OSCILLATOR CONFIG
    IRCF2 = 1;  //Internal clock frequency 1MHz
    IRCF1 = 0;
    IRCF0 = 0;
    SCS   = 1;
    
    //TMR0 CONFIG
    T0CS = 0;   //TMR0 Internal clock source
    PSA  = 0;   //Prescaler Assigned to TMR0
    PS2  = 0;   //Prescaler Rate 1:2
    PS1  = 0;
    PS0  = 0;
    TMR0_reset();
    
    //TMR2 CONFIG
    T2CONbits.T2CKPS0 = 0;  //TMR2 Prescaler 1:4
    T2CONbits.T2CKPS1 = 1;
    T2CONbits.TOUTPS0 = 1;  //TMR2 Postscaler 1:10
    T2CONbits.TOUTPS1 = 0;
    T2CONbits.TOUTPS2 = 0;
    T2CONbits.TOUTPS3 = 1;
    T2CONbits.TMR2ON  = 0;  //Turn OFF TMR2 module
    
    PR2 = 124;  //TMR2 reset every 20ms
    TMR2_reset();
    
    //INTERRUPT CONFIG
    GIE  = 1;   //Global Interrupt Enable
    T0IE = 1;   //TMR0 Interrupt Enable
    ADIE = 1;   //ADC Interrupt Enable
    
    //ADC CONFIG
    ADCON1bits.ADFM  = 0;   //Left justified
    ADCON1bits.VCFG0 = 0;   //Vref+ = Vdd
    ADCON1bits.VCFG1 = 0;   //Vref- = Vss
    
    ADCON0bits.ADCS1 = 0;   //Conversion clock = Fosc/2
    ADCON0bits.ADCS0 = 0;
    ADCON0bits.CHS3  = 0;   //ADC input from AN1
    ADCON0bits.CHS2  = 0;
    ADCON0bits.CHS1  = 0;
    ADCON0bits.CHS0  = 1;
    ADCON0bits.ADON  = 1;   //Enable ADC Module
    
    //CCP1 CONFIG
    CCP1CONbits.CCP1M0 = 0; //PWM Mode & All P1x Active-High
    CCP1CONbits.CCP1M1 = 0;
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M3 = 1;
    
    CCP1CONbits.P1M0 = 0;   //Single output P1A modulated
    CCP1CONbits.P1M1 = 0;
    
    CCPR1L = 0b01111101;    //Pulse width 2ms
    CCP1CONbits.DC1B0 = 0;
    CCP1CONbits.DC1B1 = 0;
    
    
    return;
}

void TMR0_reset(void){    
    TMR0 = tmr0_n;  //Load TMR0 value
    T0IF = 0;       //Clear Flag
    return;
}

void TMR2_reset(void){
    TMR2IF = 0;       //Clear Flag
    return;
}