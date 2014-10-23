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
volatile unsigned int gpsMessage[50];
volatile int gpsIndex;

//BEACON BOARD DEBUGGING MAIN
/*
int main(int argc, char** argv)
{
    OSCCONbits.IRCF = 0xF; //8Mhz
    OSCCONbits.SCS  = 2;
    TRISCbits.TRISC3 = 0;
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

*/


UpdateGPS()
{
    unsigned char GPSupdateMessage[22]  = "$PSRF103,01,01,00,00*"; //Send for an update
    //Set interrupts for handling incoming GPS signals
    //This will interrupt any outgoing transmission, so only do it when needed.
    RCIE = 1;
    PEIE = 1;
    GIE = 1;
    
    //Get a single GLL message
    gpsIndex = 0;
    uart_write_message(GPSupdateMessage,  22);
}

/*
void DecodeGPS()
{
    //Used to decode a GLL output message
    unsigned char messageID[7] = "$GPGLL";
    float latitude;
    float longitude;
    unsigned char northSouth;
    unsigned char eastWest;
    unsigned char status;
}
*/

int main(int argc, char** argv)
{
    OSCCONbits.SCS = 0x02; //set the SCS bits to select internal oscillator block
    OSCCONbits.IRCF = 0x0F; //set OSCCON IRCF bits to select OSC frequency=16Mhz
    gpsIndex = 1;
    initUART();             //Initialize UART module
    ToggleSleepGPS();       //Turn GPS on

    SetupGPS();             //Setup Lat/Long recording

    __delay_ms(1000);

    UpdateGPS();        //NOT WORKING, some sort of overrun error?
    
    while(1){}
   
}

void interrupt ISR() {
    if (PIR1bits.RCIF)
    {
        //Overrun is occuring, not sure why...
        if(OERR)
        {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }
        gpsMessage[gpsIndex++] = RCREG;
        PIR1bits.RCIF = 0;
    }
}
