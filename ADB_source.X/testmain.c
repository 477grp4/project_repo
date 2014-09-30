/* 
 * File:   testmain.c
 * Author: ee63pc4-user
 *
 * Created on September 30, 2014, 12:27 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <pic16f1783.h>
#pragma config WDTE = OFF

/*
 * 
 */
int counter;
int main(int argc, char** argv) {
    TRISC=0;
    PORTC=0x00;
    RC5=1;
    RC7=0;

    PS0 = 1; //prescale to 1:256
    PS1 = 0;
    PS2 = 1;

    PSA = 0; //prescale enable
    TMR0CS = 0; //8 bit mode using T0ck
    TMR0SE = 0; //Falling edge
    TMR0IE=1;   //Enable TIMER0 Interrupt
    PEIE=1;     //Enable Peripheral Interrupt
    GIE=1;      //Enable INTs globally
    while(1);
    return(EXIT_SUCCESS);
}

void interrupt isr()
{
    if (TMR0IF && TMR0IE){
        counter++;
        if(counter==4){
            RC5 = LATC5 ^ 1; //to use all of portd use PORTD  NOT RD
            RC7 = LATC7 ^ 1;
            counter=0;
        }
        TMR0IF = 0;
    }
}