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
    if(INTCONbits.IOCIF)
    {
        if(IOCAFbits.IOCAF3 && PORTAbits.RA3)
        {
            playbackFlag = playbackFlag?0:1;
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
    //GIE = 1;
    SPI_CS = 1; //hands off mode for testing the launcher

    //Main loop
    while(1)
    {

        //Check if beacon has been launched
        //CheckDisconnect(); //DONE+TESTED

        //Transmit message
        //if(transmitFlag) //DONE+TESTED
        //    TransmitMode(); //DONE+TESTED
               
        //Check if beacon needs to playback message
        //else if(playbackFlag)
        //{
            //PlaybackMode();
            //playbackFlag = 0;
        //}




        //Go back to sleep, wait for interrupts
        //SLEEP();
    }

    return (EXIT_SUCCESS);
}

