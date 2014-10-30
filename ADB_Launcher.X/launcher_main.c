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

    unsigned char periodicCounter = MIN_PERIOD;
    gpsIndex = 1;

    INTCONbits.GIE = 1;

    if(!PORTCbits.RC7)
        ToggleSleepGPS();       //Turn GPS on
    SetupGPS();             //Setup Lat/Long recording

    SSPCON1bits.SSPEN=0;      // Disable SPI Port
    PORTCbits.RC5 = 0;        //Set MOSI low
    
    SPI_CS = CS_IDLE;//prerecord

    while(1){

        
        //PRE_RECORD FUNCTION
        //InitSPI();            //Start-up SPI again
        //PreRecordMode();
        //SSPCON1bits.SSPEN=0;  // Disable SPI Port
        //PORTCbits.RC5 = 0;    //Set MOSI low
        //SPI_CS = CS_IDLE;//prerecord
        //__delay_ms(5);
        
        SPI_CS = CS_IDLE;
        
        //Check Flags
        if(PORTAbits.RA1)
        {
             //Strobe LED
            PORTBbits.RB0 = 1;
            __delay_ms(100);
            PORTBbits.RB0 = 0;
        }

        if(recordFlag)
        {
            InitSPI();            //Start-up SPI again
            RecordMode();
            SSPCON1bits.SSPEN=0;  // Disable SPI Port
            PORTCbits.RC5 = 0;    //Set MOSI low
        }

        //Not recording, Update the GPS
        UpdateGPS(); //tell GPS to send an update

        if(gpsInvalidFlag) //turn on red LED if invalid message
        {
            PORTBbits.PORTB = LATBbits.LATB | 0x20; //turn red LED on
            PORTBbits.PORTB = LATBbits.LATB & 0xEF; //turn green LED off
            if(periodicCounter < MAX_PERIOD)
                periodicCounter++;
        }
        else //turn on green LED if valid message
        {
            PORTBbits.PORTB = LATBbits.LATB | 0x10; //turn green LED on
            PORTBbits.PORTB = LATBbits.LATB & 0xDF; //turn red LED off
            periodicCounter = MIN_PERIOD;
            ToggleSleepGPS();                       //Hibernate
        }
        
        if(!recordFlag)
        {
            if(strobeFlag)
                GoToSleep(MIN_PERIOD);
            else if(gpsInvalidFlag)
                GoToSleep(periodicCounter);
            else
                Hibernate();
        }

    }

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

        if(recordFlag)
        {
            ADCON0bits.ADGO     = 1; //Start Conversion
        }
    }

    //ADC_ISR
    if(PIR1bits.ADIF & PIE1bits.ADIE)
    {
        if(!isFull())
        {
            if(recordFlag)
                WriteBuffer(ADRESH);
        }
        PIR1bits.ADIF = 0;
    }

    //Record Button IOC
    if(INTCONbits.IOCIE &&  IOCBFbits.IOCBF3)
    {
        IOCBFbits.IOCBF3 = 0;
        if(IOCBPbits.IOCBP3 & PORTBbits.RB3)
        {
            recordFlag = 1;
        }
        if(IOCBNbits.IOCBN3 & !PORTBbits.RB3)
        {
            recordFlag = 0;
        }
    }
    
    //Strobe-Select IOC
    if(INTCONbits.IOCIE &&  IOCAFbits.IOCAF1)
    {
        IOCAFbits.IOCAF1 = 0;
        if(IOCAPbits.IOCAP1 & PORTAbits.RA1)
        {
            strobeFlag = 1;
        }
        if(IOCANbits.IOCAN1 & !PORTAbits.RA1)
        {
            strobeFlag = 0;
        }
    }

}
