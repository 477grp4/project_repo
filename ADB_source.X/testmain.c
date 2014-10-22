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
#pragma config LVP=OFF, CLKOUTEN=OFF, BOREN=OFF
volatile unsigned int uart_data;

//BEACON BOARD DEBUGGING MAIN
/*
int main(int argc, char** argv)
{
    OSCCONbits.IRCF = 0xE;
    OSCCONbits.SCS  = 2;
    TRISCbits.TRISC3 = 0;
    RC3 = 1;
    //int x;
    //InitDAC();
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
         
         RC3 = 1;
    }
}
*/

void DecodeGPS()
{

}

int main(int argc, char** argv)
{
    OSCCONbits.SCS = 0x02; //set the SCS bits to select internal oscillator block
    OSCCONbits.IRCF = 0x0F; //set OSCCON IRCF bits to select OSC frequency=16Mhz
    unsigned char GPSupdateMessage[22]  = "$PSRF103,01,00,03,00*"; //Send for an update

    initUART();             //Initialize UART module
    ToggleSleepGPS();       //Turn GPS on

    SetupGPS();             //Setup Lat/Long recording
    
    while(1){}
    

}

void interrupt ISR() {
    if (PIR1bits.RCIF)
    {
        uart_data = RCREG;  //Grab byte from receive reg
        PIR1bits.RCIF = 0;
    }
}