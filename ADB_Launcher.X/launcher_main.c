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
    //InitSPI();
    InitTimer0();
    InitTimer1();
    InitWatchdog();
    InitUART();             //Initialize UART module

    unsigned char periodicCounter = MIN_PERIOD;
    gpsIndex = 0;

    INTCONbits.GIE = 1;

    ToggleSleepGPS();       //Turn GPS on
    SetupGPS();             //Setup Lat/Long recording

    PORTBbits.PORTB = LATBbits.LATB & 0xDF; //turn red LED off
    PORTBbits.PORTB = LATBbits.LATB | 0x10; //turn green LED on
    __delay_ms(100);
    PORTBbits.PORTB = LATBbits.LATB & 0xEF; //turn green LED off
    
    
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
          RecordMode();
          recordFlag = 0;
          __delay_ms(500); //trying to get gps to not hangup
        }

        //Not recording, Update the GPS
        if(gpsTimeoutState==0)
            UpdateGPS(); //tell GPS to send an update
        else if(gpsTimeoutState==1)
        {
            ToggleSleepGPS();
            gpsTimeoutState = 2;
        }
        else if(gpsTimeoutState==2)
        {
            ToggleSleepGPS();
            gpsTimeoutState = 3;
        }
        else if(gpsTimeoutState==3)
        {
            __delay_ms(1000);
            gpsTimeoutState = 0;
        }
        else
        {
            gpsTimeoutState = 0;
        }

        if(gpsInvalidFlag) //turn on red LED if invalid message
        {
            PORTBbits.PORTB = LATBbits.LATB | 0x20; //turn red LED on
            PORTBbits.PORTB = LATBbits.LATB & 0xEF; //turn green LED off
            __delay_ms(250);
            PORTBbits.PORTB = LATBbits.LATB & 0xDF; //turn red LED off
            if(periodicCounter < MAX_PERIOD)
                periodicCounter++;
        }
        else //turn on green LED if valid message
        {
            PORTBbits.PORTB = LATBbits.LATB | 0x10; //turn green LED on
            PORTBbits.PORTB = LATBbits.LATB & 0xDF; //turn red LED off
            __delay_ms(250);
            PORTBbits.PORTB = LATBbits.LATB & 0xEF; //turn green LED off
            periodicCounter = MIN_PERIOD;
        }
        
        if(!recordFlag)
        {
            if(PORTAbits.RA1) //check strobe
                GoToSleep(MIN_PERIOD);
            else if(gpsInvalidFlag)
                GoToSleep(periodicCounter);
            else
            {
                ToggleSleepGPS();       //Turn GPS off
                Hibernate();
                ToggleSleepGPS();       //Turn GPS on
            }
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
        if(gpsIndex >= 50)
        {
            messageDoneFlag = 1;
            gpsIndex = 0;
        }
        else
        {
            if(gpsMessage[gpsIndex-1] == 0x0A)
            {
                messageDoneFlag = 1;
            }
            else
            {
                messageDoneFlag = 0;
            }
        }
        PIR1bits.RCIF = 0;
    }

    //startADC
    if (TMR1IF && TMR1IE)
    {
        TMR1H               = SAMPLE_PERIOD_UPPER; //Set Timer counter to 50 for 10kHz
        TMR1L               = SAMPLE_PERIOD_LOWER;
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

    /*
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        eeprom_timeoutFlag = 1;
        INTCONbits.TMR0IF = 0;
    }
     * */

}
