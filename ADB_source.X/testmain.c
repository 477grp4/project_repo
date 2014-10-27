/* 
 * File:   testmain.c
 * Author: ee63pc4-user
 *
 * Created on September 30, 2014, 12:27 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <pic16f1783.h>
#include "beacon.h"
#include <xc.h>
#include <pic.h>
#include <string.h>
#pragma config FOSC=INTOSC, PLLEN=OFF, MCLRE=ON, WDTE=OFF
#pragma config LVP=ON, CLKOUTEN=OFF, BOREN=OFF
//volatile unsigned int gpsMessage[50];
//volatile int gpsIndex;

//BEACON BOARD DEBUGGING MAIN

int main(int argc, char** argv)
{
    OSCCONbits.IRCF = 0xF; //8Mhz
    OSCCONbits.SCS  = 2;
    TRISCbits.TRISC3 = 0;
    TRISBbits.TRISB5 = 0;
    ANSELBbits.ANSB5 = 0;
    RB5 = 1;
    RC3 = 1;
    int x;
    InitDAC();
    __delay_us(10);
    //TMR1IE              = 1;    //Enable TIMR1 interrupts
    //DACCON0bits.DACEN = 1;      //turn on DAC

    while(1)
    {
        for(x=0;x<=0xFF;x++)
        {
            __delay_us(2);
            DACCON1 = x;
        }
        for(x=0xFF;x>=0;x--)
        {
            __delay_us(2);
            DACCON1 = x;
        }
    }
}