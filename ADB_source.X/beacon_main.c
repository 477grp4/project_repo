/* 
 * File:   beacon_main.c
 * Author: 477grp4
 *
 * Created on October 23, 2014, 5:42 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <pic16f1783.h>
#include "beacon.h"
#include <xc.h>
#include <pic.h>
#include <string.h>
#pragma config FOSC=INTOSC, PLLEN=OFF, MCLRE=ON, WDTE=0x01
#pragma config LVP=ON, CLKOUTEN=OFF, BOREN=OFF



interrupt void isr()
{
    //Timer0 used for DAC output rate
    if(TMR0IF)
    {
        if(dacOutputFlag)
        {
            if(!isEmpty())          //Read if not empty
            {
                DACCON1 = ReadBuffer();
            }
        }

        TMR0 = SAMPLE_COUNT;        //Reset Timer0 counter
        TMR0IF = 0;
    }
    //Playback IOC
    if(INTCONbits.IOCIE && IOCAFbits.IOCAF3)
    {
        IOCAFbits.IOCAF3 = 0;

        if(IOCAPbits.IOCAP3 && PORTAbits.RA3)
        {
            playbackFlag = 1;
        }
        if(IOCANbits.IOCAN3 && !PORTAbits.RA3)
        {
            playbackFlag = 0;
        }
        
    }
}


int main(int argc, char** argv)
{

    //Initializations
    InitCLK();
    InitGPIO();
    InitDAC();
    InitSPI();
    InitTimer0();
    InitWatchdog();

    //Setup interrupts
    PEIE = 1;
    GIE = 1;

    SPI_CS = CS_IDLE; //must be changed!


    SSPCON1bits.SSPEN=0;  // Disable SPI Port
    //PORTCbits.RC5 = 0;    //Set MOSI low
    //PORTCbits.RC3 = 0;    //Set SCK low
    PORTCbits.PORTC = LATCbits.LATC & 0xD7; //set MOSI and SCK low

    while(1)
    {
        SPI_CS = CS_IDLE; //hands off mode for testing the launcher

        //Check if beacon has been launched
        CheckDisconnect(); //DONE+TESTED

        if(playbackFlag&&!MEM_ACCESS)
        {
            PlaybackMode();
            playbackFlag = 0;
        }

        //Transmit message
        if(transmitFlag) //DONE+TESTED
            TransmitMode(); //DONE+TESTED

        //Go back to sleep, wait for interrupts
        Hibernate();
    }

    return (EXIT_SUCCESS);
}

