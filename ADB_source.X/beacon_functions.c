#include <stdio.h>
#include <stdlib.h>
#include <pic16f1783.h>
#include "beacon.h"
#include <xc.h> // include standard header file
#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions
#define SPI_CS LATC2



#define SPI_PAGE_SIZE (256)

void InitSPI()
{
    // set up oscillator control register
    OSCCONbits.IRCF = 0x0F; //set OSCCON IRCF bits to select OSC frequency=16Mhz
    OSCCONbits.SCS = 0x02; //set the SCS bits to select internal oscillator block
    
    //PORT C Assignments
    TRISCbits.TRISC2 = 0; // RC2 = output to SEE CS Pin
    TRISCbits.TRISC3 = 0; // RC3 = SCK output SEE
    TRISCbits.TRISC4 = 1; // RC4 = SDI input from SEE
    TRISCbits.TRISC5 = 0; // RC5 = SDO output to SEE

    SPI_CS = 1; // start high. Need falling edge to do anything

    // Setup SPI as Master mode, Mode 00 and clock rate of Fosc/16
    SSPCON1bits.SSPEN=0x00;      // Disable SPI Port for configuration
    SSPCON1bits.SSPM=0x01;       // SPI Master mode, clock = Fosc/16 (1 Mhz)
    SSPCON1bits.CKP=0;           // Idle state for clock is low
    SSPSTATbits.CKE=1;          // Transmit occurs on transition falling clock state
    SSPSTATbits.SMP=0;          // Data is sampled at middle of data output time
    SSPCON1bits.SSPEN=0x01;      // Enable SPI Port
}

void WriteSPI(unsigned char databyte)
{
    unsigned int buffer;
    buffer = SSPBUF;            // Read the buffer to clear any remaining data and clear the buffer full bit
    PIR1bits.SSP1IF=0;          // clear SSP interrupt bit
    SSPBUF = databyte;          // Write data byte to the buffer to initiate transmission
    while(!PIR1bits.SSP1IF);    // Wait for interrupt flag to go high indicating transmission is complete
}

unsigned char ReadSPI(void)
{
    unsigned char databyte;

    SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
    while(!SSPSTATbits.BF);     // Wait for interrupt flag to go high indicating transmission is complete
    databyte = SSPBUF;          // Read the incoming data byte
    PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
    return (databyte);
}

void SPIWritePage(unsigned char *data, unsigned int address, unsigned int size)
{
    unsigned char StatusReg;
    unsigned char address_bytes[3];
    
    address_bytes[0]=(unsigned char)(address>>8);
    address_bytes[1]=(unsigned char)(address>>4);
    address_bytes[2]=(unsigned char)(address);
    
    SPI_CS = 0;         //bring chip select low
    WriteSPI(SPI_WREN); //send write enable
    SPI_CS=1;

   /* do
    {
        StatusReg = (Read_SPI_StatusReg() & 0x02);   // Read the Status Register and mask out
    } while (StatusReg);                             // the WIP bit (bit 0)
    */
    __delay_ms(5);          // If you don't want to use the polling method you can
                               // just wait for the max write cycle time (5ms)

    SPI_CS=0;
    WriteSPI(SPI_WRITE);         // Send write command
    WriteSPI(address_bytes[0]);
    WriteSPI(address_bytes[1]);
    WriteSPI(address_bytes[2]);

    int lcv=0;
    for(lcv=0;lcv<size;lcv++)
    {
        WriteSPI(data[lcv]);
    }

    SPI_CS = 1;
}

void SPIWriteData(unsigned char *data, unsigned int address, unsigned int size)
{
    int data_index=0;
    int page_remaining_bytes =(SPI_PAGE_SIZE-(address%SPI_PAGE_SIZE));
    int StatusReg;
    while(size>page_remaining_bytes)
    {
        do
        {
            StatusReg = (Read_SPI_StatusReg() & 0x01);   // Read the Status Register and mask out
        } while (StatusReg);
        
        SPIWritePage(&data[data_index], address, page_remaining_bytes);
        address += page_remaining_bytes;
        data_index += page_remaining_bytes;
        size -= page_remaining_bytes;
        page_remaining_bytes =(SPI_PAGE_SIZE-(address%SPI_PAGE_SIZE));
    }

    do
    {
        StatusReg = (Read_SPI_StatusReg() & 0x01);   // Read the Status Register and mask out
    } while (StatusReg);
    SPIWritePage(&data[data_index], address, size);
}

void SPIReadData(unsigned char *data, unsigned int address, unsigned int size)
{
    unsigned char address_bytes[3];
    address_bytes[0]=(unsigned char)(address>>8);
    address_bytes[1]=(unsigned char)(address>>4);
    address_bytes[2]=(unsigned char)(address);

    int StatusReg;
    do
    {
        StatusReg = (Read_SPI_StatusReg() & 0x01);   // Read the Status Register and mask out
    } while (StatusReg);

    SPI_CS = 0;
    WriteSPI(SPI_READ);
    WriteSPI(address_bytes[0]);
    WriteSPI(address_bytes[1]);
    WriteSPI(address_bytes[2]);

    int lcv;
    for(lcv=0;lcv<size;lcv++)
    {
        data[lcv]=ReadSPI();
    }

    SPI_CS = 1;
}

unsigned char Read_SPI_StatusReg(void)
{
    unsigned char data_read;

    SPI_CS=0;
    WriteSPI(SPI_RDSR);         // Send read status register command
    data_read = ReadSPI();  // Read the data
    SPI_CS=1;

    return(data_read);
}


void InitADC()
{
    //Configure pin as Analog input
    TRISB1              = 1;    //ADC input pin
    ANSB1               = 1;    //Set RB1 to analog input

    //Configure ADC module
    ADCON1bits.ADCS     = 0x05; //set ADC clock to FOSC/2
    ADCON1bits.ADPREF   = 0x00; //set ADC positive reference to Vcc
    ADCON1bits.ADNREF   = 0x00; //set ADC negative reference to Vss
    ADCON0bits.CHS      = 0xA; //use AN10 for ADC
    ADCON2bits.CHSN     = 0xF; //Set negative input to Vref-
    ADCON1bits.ADFM     = 0;    //set output mode to sign-magnitude
    ADCON0bits.ADRMD    = 1;    //set ADC to 10-bit mode
    ADCON0bits.ADON     = 1;    //turn on ADC

    //Configure ADC interrupt (Optional)
    ADIF                = 0;    //Clear ADC interrupt flag
    ADIE                = 1;    //Enable ADC interrupt
    PEIE                = 1;    //Enable periph. interrupts
    //GIE                 = 1;    //Enable global interrupts
}

unsigned char readADC()
{
   unsigned char adc_buffer;
   ADCON0bits.ADGO     = 1;        //Start Conversion
   while(ADCON0bits.ADGO);         //Wait for completion

   adc_buffer          = ADRESH;   //Read result
   ADIF                = 0;        //Clear ADC interrupt flag
   return adc_buffer;
}

void InitDAC()
{
    TRISA5 = 1; //set RA5 (OPA1 inverting input) as input
    TRISA1 = 0; //set RA1 (OPA1 output) as output
    OPA1CONbits.OPA1CH = 0x2;   //DAC output
    OPA1CONbits.OPA1SP = 1;     //Needed
    OPA1CONbits.OPA1EN = 1;     //Enable OPA

    //Configure DAC
    DACCON0bits.DACOE1 = 0;     //DACOUT1 disabled, only fed to OPA
    DACCON0bits.DACOE2 = 0;     //DACOUT2 disabled
    DACCON0bits.DACPSS = 0x0;   //Positive sourced from Vdd
    DACCON0bits.DACNSS = 0;     //Negative source from Vss
    DACCON0bits.DACEN = 1;      //DAC enabled
    DACCON1 = 0x00;
}

void ReadSPI_overhead(int address)
{
    unsigned char address_bytes[3];
    address_bytes[0]=(unsigned char)(address>>8);
    address_bytes[1]=(unsigned char)(address>>4);
    address_bytes[2]=(unsigned char)(address);

    int StatusReg;
    do
    {
        StatusReg = (Read_SPI_StatusReg() & 0x01);   // Read the Status Register and mask out
    } while (StatusReg);

    SPI_CS = 0;
    WriteSPI(SPI_READ);
    WriteSPI(address_bytes[0]);
    WriteSPI(address_bytes[1]);
    WriteSPI(address_bytes[2]);

    return;
}

void WriteSPI_overhead(int address)
{

    unsigned char address_bytes[3];
    unsigned char StatusReg;

    address_bytes[0]=(unsigned char)(address>>8);
    address_bytes[1]=(unsigned char)(address>>4);
    address_bytes[2]=(unsigned char)(address);

    do
    {
        StatusReg = (Read_SPI_StatusReg() & 0x02);   // Read the Status Register and mask out write in progress flag
    } while (StatusReg);

    SPI_CS = 0;         //bring chip select low
    WriteSPI(SPI_WREN); //send write enable
    SPI_CS=1;

    __delay_ms(5);          // If you don't want to use the polling method you can
                               // just wait for the max write cycle time (5ms)

    SPI_CS=0;
    WriteSPI(SPI_WRITE);         // Send write command
    WriteSPI(address_bytes[0]);
    WriteSPI(address_bytes[1]);
    WriteSPI(address_bytes[2]);

    return;
}

void WriteBuffer(unsigned char data)
{
    buffer[end] = data;
    //Increment end index, wrap if needed
    if (end == BUFFERSIZE)
        end = 0;
    else
        end++;
}

unsigned char ReadBuffer()
{
    unsigned char read_data;

    read_data = buffer[start];
    if (start == BUFFERSIZE)
        start = 0;
    else
        start++;
    return read_data;
}

void InitTimer1()
{
    T1CONbits.TMR1CS    = 0;    //FOSC/4
    T1CONbits.T1CKPS    = 2;    //FOSC/8
    T1CONbits.nT1SYNC   = 1;    //Non synchronized input
    T1GCONbits.TMR1GE   = 0;    //Disregard gate function
    TMR1H               = 0xFF; //Set Timer counter to 100 for 10kHz
    TMR1L               = 0x9B; //was 9B
    T1CONbits.TMR1ON    = 1;    //Enable timer

}

void playback_mode(long int record_end_address)
{
  DAC_output_flag = 1;
  ADC_sample_flag = 0;

  long int curr_address = 0;
  int DAC_count = 0;

  while((!record_flag) && (curr_address<=record_end_address))
  {
    ReadSPI_overhead(curr_address);
    DAC_count = 0;
    while((!record_flag) && (DAC_count<256) && (curr_address <= record_end_address))
    {
      if(!isFull())
      {
        DAC_count++;
        curr_address++;
        WriteBuffer(ReadSPI());
      }
    }
    SPI_CS = 1;
  }
}

long int record_mode()
{
    DAC_output_flag = 0;
    RING_START = 0; //clear the buffer
    RING_END = 0;
    ADC_sample_flag = 1;
    long int address = 0;
    int count = 256;
    TMR1ON = 1;
    while((record_flag) && (address < RECORD_END_ADDRESS))
    {
        if(count>=256) //perform overhead on next page
        {
            SPI_CS = 1;
            count = 0;
            WriteSPI_overhead(address);
        }
        if(!isEmpty())
        {
            WriteSPI(ReadBuffer());
            address++;
            count++;
        }
    }
    SPI_CS = 1;
    RING_START = 0; //clear the buffer
    RING_END = 0;
    if(address >= RECORD_END_ADDRESS)
    {
      record_flag = 0;
      return RECORD_END_ADDRESS-1;
    }
    return address;
}

void InitRecord()
{
    TRISBbits.TRISB2 = 1; //set RB2 to input
    ANSELBbits.ANSB2 = 0; //set RB2 to digital because portB is weird
    IOCBNbits.IOCBN2 = 1; //enable negative edge interrupt for pin RB2
    IOCBPbits.IOCBP2 = 1; //enable positive edge interrupt for pin RB2
    IOCBF = 0x00;
    INTCONbits.IOCIE = 1; //enable interrupts on change

}

/* record/playback test
 
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
 */

//Likely a test of the entire ADC-DAC chain
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



void initUART()
{
    //NOTE: pins 6 and 7 on portC cannot be analog
    TRISCbits.TRISC6 = 0; //TX out
    TRISCbits.TRISC7 = 1; //RX in
    TXSTAbits.BRGH = 0; //low speed baud rate
    //target baud: 4800
    SPBRGH = 0; //Fosc/64
    SPBRGL = 51; //[16MHz/(4800 * 64)] - 1
    TXSTAbits.TX9 = 0; //8 data bit mode
    TXSTAbits.SYNC = 0; //asynchronous mode
    TXSTAbits.TXEN = 1; //transmit enabled

    RCSTAbits.SPEN = 1; //serial port enabled
    RCSTAbits.RX9 = 0; //8 data bit mode
    RCSTAbits.CREN = 1; //receive enabled
}
unsigned char compute_checksum(unsigned char * data, int size)
{
    unsigned char checksum = 0;
    int x;
    for(x=0; x<size; x++)
    {
        if ((data[x] != '$') && (data[x] != '*') && (data[x] != '!'))
            checksum ^= data[x];
    }
    return checksum;
}
void uart_xmit(unsigned char mydata_byte)
{

    while(!TXSTAbits.TRMT);    // make sure buffer full bit is high before transmitting
    TXREG = mydata_byte;       // transmit data
}


void uart_write_message(unsigned char * data, int size)
{
    int x;
    unsigned char checksum;
    unsigned char ascii_checksum[2];

    checksum = compute_checksum(data, size);
    ascii_checksum[0] = checksum >> 4;
    ascii_checksum[1] = checksum & 0x0F;

    ascii_checksum[0] += (ascii_checksum[0] < 10 ? 48 : 55);
    ascii_checksum[1] += (ascii_checksum[1] < 10 ? 48 : 55);

    for(x=0; x<size-1; x++)
    {
        uart_xmit(data[x]);
    }
    uart_xmit(ascii_checksum[0]);
    uart_xmit(ascii_checksum[1]);

    uart_xmit(0x0D);
    uart_xmit(0x0A);

}



void ToggleSleepGPS()
{
    TRISAbits.TRISA6 = 0; //ON/OFF pin

    RA6 = 0;
    __delay_ms(1000); //Must wait 1 second between pulses
    RA6 = 1;
    __delay_ms(100);  //.1ms pulse width
    RA6 = 0;
}
void DisableGPS()
{
    //Disables the GPS, the previous settings remain intact
    unsigned char init_117[13] = "$PSRF117,16*"; //OFF command
    uart_write_message(init_117, 13); //Send OFF command
}


void SetupGPS()
{
    int x;
    unsigned char startSequence[6] = "$PSRF";
    unsigned char MID[4] = "103";
    unsigned char message[22];

    //Disable GSA,GSV,RMC,VTG
    for(x = 0; x < 6; x++)
    {
        __delay_ms(1000);
        sprintf(message, "%s%s,0%d,00,00,00*", startSequence, MID, x);
        uart_write_message(message,  22);
    }

    __delay_ms(1000);
    //Enable GLL in query mode
    sprintf(message, "%s%s,01,01,01,00*", startSequence, MID);
    uart_write_message(message,  22);

}
