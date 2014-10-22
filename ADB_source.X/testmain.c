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
#include<pic.h>
#pragma config FOSC=INTOSC, PLLEN=OFF, MCLRE=ON, WDTE=OFF
#pragma config LVP=OFF, CLKOUTEN=OFF, BOREN=OFF
volatile unsigned int uart_data;
/*
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
*/


/* record/playback test
interrupt void isr(void)
{
    //startADC
    if (TMR1IF && TMR1IE)
    {
        TMR1H               = 0xFF; //Set Timer counter to 100 for 10kHz
        TMR1L               = 0x9B; //was 9B

        TMR1IF = 0;
        if(DAC_output_flag)
        {
            if(!isEmpty()) //!isEmpty
            {
                DACCON1 = ReadBuffer();
            }
        }
        else if(ADC_sample_flag)
        {
            ADCON0bits.ADGO     = 1;
        }       //Start Conversion
    }

    //ADC_ISR
    if(ADIF & ADIE)
    {
        if(!isFull())
        {
            WriteBuffer(ADRESH);
        }
        ADIF = 0;
    }

    if(IOCIE &&  IOCBF2)
    {
        IOCBF2 = 0;
        if(IOCBP2 & RB2)
        {
            record_flag = 1;
        }
        if(IOCBN2 & !RB2)
        {
            record_flag = 0;
            
        }
    }
    
}
 * 

int main(int argc, char** argv)
{
    __delay_us(10);
    InitTimer1();
    InitSPI();
    InitADC();
    InitDAC();
    InitRecord();

    //TMR1ON = 0;
    __delay_us(10);
    TMR1IE              = 1;    //Enable TIMR1 interrupts


    INTCONbits.GIE = 1;

    int first_record = 1;
    long int record_end_address = 0;
    record_flag = 0;


    while(1)
    {
        if(!first_record)
        {
            playback_mode(record_end_address);
        }

        if(record_flag)
        {
            first_record = 0;
            record_end_address = record_mode();
        }
    }
    
} //record/playback test
*/
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
        
       /* for(x=0;x<=0xFF;x++)
        {
            __delay_us(2);
            DACCON1 = x;
        }
        for(x=0xFF;x>=0;x--)
        {
            __delay_us(2);
            DACCON1 = x;
        }
        * */
         RC3 = 1;
    }
}

/*
int main(int argc, char** argv)
{
    OSCCONbits.IRCF = 0xF;
    OSCCONbits.SCS  = 2;

    int x;

    InitTimer1();
    InitSPI();
    InitADC();
    InitDAC();
    
    int count;
    //Wait
    __delay_us(10);
    TMR1IE              = 1;    //Enable TIMR1 interrupts
    while(1)
    {
    DACCON0bits.DACEN = 0;      //turn off DAC
    ADCON0bits.ADON     = 1;    //turn on ADC
    count = 0;
    //WriteSPI_overhead(WRITE_ADDRESS);

    while(count < 256)
    {
        if(!isEmpty())
        {
            WriteSPI(ReadBuffer());
            count++;
        }
    }
    SPI_CS = 1; //write page
    ADCON0bits.ADON     = 0;    //turn off ADC

    int DAC_count = 0;
    DAC_output_flag = 1;
    DACCON0bits.DACEN = 1;      //turn on DAC
    while(1){
        ReadSPI_overhead(READ_ADDRESS);
        DAC_count = 0;
        while(DAC_count<256)
        {
            if(!isFull())
            {
                DAC_count++;
                WriteBuffer(ReadSPI());
            }
        }


        SPI_CS = 1; //finish read
    }


    }
}
*/
/*  //triangle wave
    initDAC();
    int x;
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
*/

/*
int main(int argc, char** argv)
{
    OSCCONbits.SCS = 0x02; //set the SCS bits to select internal oscillator block
    OSCCONbits.IRCF = 0x0F; //set OSCCON IRCF bits to select OSC frequency=16Mhz
    initUART();

    TRISAbits.TRISA6 = 0; //ON/OFF pin
    RA6 = 0;
    __delay_ms(1000);
    RA6 = 1;
    __delay_ms(100);
    RA6 = 0;
    
   // unsigned char init_100[23] = "$PSRF100,1,4800,8,1,0*";
  //  unsigned char init_101[31] = "$PSRF101,0,0,0,96000,0,0,12,8*";
    //unsigned char init_102[21] = "$PSRF102,4800,8,1,0*";
    
    unsigned char init_103[22]  = "$PSRF103,01,00,03,00*";
    unsigned char init_103a[22] = "$PSRF103,00,00,00,00*"; //gga
    unsigned char init_103b[22] = "$PSRF103,02,00,00,00*"; //gpgsa
    unsigned char init_103c[22] = "$PSRF103,03,00,00,00*"; //gsv
    unsigned char init_103d[22] = "$PSRF103,04,00,00,00*"; //rmc
    unsigned char init_103e[22] = "$PSRF103,05,00,00,00*"; //vtg
    unsigned char init_103f[22] = "$PSRF103,08,00,00,00*"; //zda

    //unsigned char init_105[12] = "$PSRF105,1*"; //debug
    //unsigned char init_117[13] = "$PSRF117,16*"; //turn off

    //uart_write_message(init_105, 12); 
    //uart_write_message(init_100, 23); //set baud
    //uart_write_message(init_101, 31); //initialize
    __delay_ms(1000);
    //Setup GLL, each disables a single message
    uart_write_message(init_103,  22); //factory reset
    uart_write_message(init_103a, 22); //factory reset
    uart_write_message(init_103b, 22); //factory reset
    uart_write_message(init_103c, 22); //factory reset
    uart_write_message(init_103d, 22); //factory reset
    uart_write_message(init_103e, 22); //factory reset
    uart_write_message(init_103f, 22); //factory reset
    
    //uart_write_message(init_117, 13); //factory reset

    while(1){
    }
    

}

void interrupt ISR() {
    if (PIR1bits.RCIF)
    {
        uart_data = RCREG;  //Grab byte from receive reg
        PIR1bits.RCIF = 0;
    }
}
 * */