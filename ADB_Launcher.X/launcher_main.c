/* 
 * File:   launcher_main.c
 * Author: 477grp4
 *
 * Created on October 25, 2014, 6:11 PM
 */

//#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <pic16f1783.h>
#include "launcher.h"
#include <xc.h>
#include <pic.h>
//#include <cstring.h>
#pragma config FOSC=INTOSC, PLLEN=OFF, MCLRE=ON, WDTE=0x01
#pragma config LVP=ON, CLKOUTEN=OFF, BOREN=OFF


int main(int argc, char** argv) {
    
    //Initializations
    InitCLK();
    InitGPIO();
    InitADC();
    InitSPI();
    InitTimer1();
    InitWatchdog();
    initUART();             //Initialize UART module

    gpsIndex = 1;

    INTCONbits.GIE = 1;

    ToggleSleepGPS();       //Turn GPS on

    SetupGPS();             //Setup Lat/Long recording
    //__delay_ms(1000);

    
    

    while(1){
        GoToSleep(); //sleep
        UpdateGPS(); //tell GPS to send an update
        while(!messageDoneFlag); //wait for GPS to finish transferring message
        DecodeGPS(); //decode the message sent by the GPS
        messageDoneFlag = 0; //clear the message done flag
        if(gpsInvalidFlag) //turn on red LED if invalid message
        {
            PORTBbits.PORTB = LATBbits.LATB | 0x20; //turn red LED on
            PORTBbits.PORTB = LATBbits.LATB & 0xEF; //turn green LED off
        }
        else //turn on green LED if valid message
        {
            PORTBbits.PORTB = LATBbits.LATB | 0x10; //turn green LED on
            PORTBbits.PORTB = LATBbits.LATB & 0xDF; //turn red LED off
        }
    }
/*

    //Main Loop
    while(1)
    {
        if (recordModeFlag)
            TMR1IE              = 1;    //Enable TIMR1 interrupts
        else
            TMR1IE              = 0;    //Disable TIMR1 interrupts
    }
*/

    return (EXIT_SUCCESS);
}

interrupt void isr(void)
{
    //UART isr
    if (PIR1bits.RCIF)
    {
        //Overrun is occuring, not sure why...
        if(OERR)
        {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }
        gpsMessage[gpsIndex++] = RCREG;
        if(gpsMessage[gpsIndex-1] == 0x0A)
        {
            messageDoneFlag = 1;
        }
        else
        {
            messageDoneFlag = 0;
        }
        PIR1bits.RCIF = 0;
    }

    //startADC
    if (TMR1IF && TMR1IE)
    {
        TMR1H               = 0xFF; //Set Timer counter to 50 for 10kHz
        TMR1L               = 0xCD;
        TMR1IF              = 0;

        if(recordModeFlag)
        {
            ADCON0bits.ADGO     = 1; //Start Conversion
        }
    }

    //ADC_ISR
    if(PIR1bits.ADIF & PIE1bits.ADIE)
    {
        if(!isFull())
        {
            WriteBuffer(ADRESH);
        }
        PIR1bits.ADIF = 0;
    }

    if(INTCONbits.IOCIE &&  IOCBFbits.IOCBF3)
    {
        IOCBFbits.IOCBF3 = 0;
        if(IOCBPbits.IOCBP3 & PORTBbits.RB3)
        {
            recordModeFlag = 1;
        }
        if(IOCBNbits.IOCBN3 & !PORTBbits.RB3)
        {
            recordModeFlag = 0;
        }
    }

}
