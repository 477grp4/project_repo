#include "beacon.h"
#include <pic.h>

void InitCLK()
{
    OSCCONbits.IRCF = 0xF; //16/32Mhz depending on PLL
    OSCCONbits.SCS  = 2;
}
void InitGPIO()
{
    //SPI and DAC outputs initialized in respective init funcitons

    //Playback Button Interrupt initialization
    TRISAbits.TRISA3 = 1;       //Set RA3 to input
    ANSELAbits.ANSA3 = 0;
    IOCAPbits.IOCAP3 = 1;       //Enable positive edge interrupt for pin RA1
    IOCANbits.IOCAN3 = 1;       //enable negative edge interrupt for pin RA1
    IOCAF            = 0x00;    //Clear IOC flags
    INTCONbits.IOCIE = 1;       //Enable interrupts on change

    //Port A
    TRISAbits.TRISA0 = 0;       //Output, unused
    //TRISAbits.TRISA1 = 0;     //set RA1 (OPA1 output) as output
    TRISAbits.TRISA2 = 0;       //Output, unused
    //TRISAbits.TRISA3 = 1;       //Play-Button input
    TRISAbits.TRISA4 = 1;       //Mem-Access input
    //TRISAbits.TRISA5 = 1;     //set RA5 (OPA1 inverting input) as input
    TRISAbits.TRISA6 = 0;       //Output, unused
    TRISAbits.TRISA7 = 0;       //Output, unused

    //Port B
    TRISBbits.TRISB0 = 1;       //Input for routing purposes
    TRISBbits.TRISB1 = 1;       //Input for routing purposes
    TRISBbits.TRISB2 = 1;       //Input for routing purposes
    TRISBbits.TRISB3 = 0;       //Output, unused
    TRISBbits.TRISB4 = 0;       //Output, unused
    TRISBbits.TRISB5 = 0;       //Transmitter Enable
    PORTBbits.RB5    = 0;       //Keep low until transmit

    //Port C
    TRISCbits.TRISC0 = 0;       //Detach-detect-poll
    TRISCbits.TRISC1 = 1;       //Detach-detect-read
    //TRISCbits.TRISC2 = 0;       // RC2 = output to SEE CS Pin
    //TRISCbits.TRISC3 = 0;       // RC3 = SCK output SEE
    //TRISCbits.TRISC4 = 1;       // RC4 = SDI input from SEE
    //TRISCbits.TRISC5 = 0;       // RC5 = SDO output to SEE
    TRISCbits.TRISC6 = 0;       //Output, unused
    TRISCbits.TRISC7 = 0;       //Output, unused
    PORTCbits.RC0    = 0;       //Keep low until check
}

void InitDAC()
{
    TRISAbits.TRISA5 = 1;       //set RA5 (OPA1 inverting input) as input
    TRISAbits.TRISA1 = 0;       //set RA1 (OPA1 output) as output
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

void InitSPI()
{
    // set up oscillator control register
    //OSCCONbits.IRCF = 0x0F; //set OSCCON IRCF bits to select OSC frequency=16Mhz
    //OSCCONbits.SCS = 0x02; //set the SCS bits to select internal oscillator block

    //PORT C Assignments
    TRISCbits.TRISC2 = 0; // RC2 = output to SEE CS Pin
    TRISCbits.TRISC3 = 0; // RC3 = SCK output SEE
    TRISCbits.TRISC4 = 1; // RC4 = SDI input from SEE
    TRISCbits.TRISC5 = 0; // RC5 = SDO output to SEE

    SPI_CS = CS_IDLE;     // start high. Need falling edge to do anything

    // Setup SPI as Master mode, Mode 00 and clock rate of Fosc/16
    SSPCON1bits.SSPEN=0x00;      // Disable SPI Port for configuration
    SSPCON1bits.SSPM=0x01;       // SPI Master mode, clock = Fosc/16 (1 Mhz)
    SSPCON1bits.CKP=0;           // Idle state for clock is low
    SSPSTATbits.CKE=1;          // Transmit occurs on transition rising clock state
    SSPSTATbits.SMP=0;          // Data is sampled at middle of data output time
    SSPCON1bits.SSPEN=0x01;      // Enable SPI Port
}

void InitTimer0()
{
    //for playback and transmission frequency
    //interrupt once every ~10kHz
    TMR0IE              = 0;    //turn off timer0 interrupt
    TMR0IF              = 0;    //turn off timer0 interrupt flag
    TMR0CS              = 0;    //set clock source to internal instruction clock (FOSC/4)
    PSA                 = 0;    //turn on prescalar to timer0
    PS0                 = 0;    //set prescalar to 2
    PS1                 = 0;
    PS2                 = 0;
    TMR0                = SAMPLE_COUNT; //count 0d40 times to get from (16MHz/4) to 10kHz (was 9B)
    TMR0IE              = 1;    //Enable timer0 interrupt
}

void InitWatchdog()
{
    //Used to wakeup from sleep mode
    WDTCONbits.WDTPS = 0x0D;    //Set timing interval
    WDTCONbits.SWDTEN = 0;      //Software enable or WDT

}

void CheckDisconnect()
{
    PORTCbits.RC0 = 1;          //Set poll line high
    
    if (PORTCbits.RC1)          //Check read line
        transmitFlag = 1;
    else
        transmitFlag = 0;

    PORTCbits.RC0 = 0;          //Reset poll line low
}

void TransmitMode()
{
    //Add some sort of playback length
    TRANS_ENABLE    = 1;    //Set transmitter enable
    PlaybackMode();
    TRANS_ENABLE    = 0;    //Disable transmitter
}


void PlaybackMode()
{
  long int curr_address = 0;
  long int end_address;
  InitSPI();            //Start-up SPI again
  
  dacOutputFlag = 1;      //signals DAC to output next byte

  ReadOverheadSPI(PLAYBACK_BEGIN_ADDRESS);

  //Extract Header Data
  end_address = ReadSPI();
  end_address = end_address<<8;
  end_address +=ReadSPI();
  end_address = end_address<<8;
  end_address +=ReadSPI();

  Longitude[0] = ReadSPI();     //degrees
  Longitude[1] = ReadSPI();     //minutes
  Longitude[2] = ReadSPI();     //seconds

  NorthSouth = ReadSPI();       //North/South (N/S)

  Latitude[0] = ReadSPI();      //degrees
  Latitude[1] = ReadSPI();      //minutes
  Latitude[2] = ReadSPI();      //seconds

  EastWest = ReadSPI();         //East/West (E/W)

  //Checking transmit/playback before reading a page
  while((transmitFlag || (!transmitFlag && !MEM_ACCESS && playbackFlag)) && (curr_address<=end_address))
  {
    
    //Checking transmit/playback before reading each byte of page
    //while((transmitFlag || (!transmitFlag && !MEM_ACCESS)) && (DAC_count<SPI_PAGE_SIZE) && (curr_address <= RECORD_END_ADDRESS))
    //{
      if(!isFull())
      {
        //DAC_count++;
        curr_address++;
        WriteBuffer(ReadSPI()); //Read single byte into circular buffer
      }
  }

  //TODO: read off GPS coordinates

  SSPCON1bits.SSPEN=0;  // Disable SPI Port
  PORTCbits.RC5 = 0;    //Set MOSI low
  SPI_CS = CS_IDLE;
  dacOutputFlag = 0;   //signals DAC to stop output

}

//SPI Functions
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
    unsigned char dataByte;

    SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
    while(!SSPSTATbits.BF);     // Wait for interrupt flag to go high indicating transmission is complete
    dataByte = SSPBUF;          // Read the incoming data byte
    PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
    return (dataByte);
}

/*
void WritePageSPI(unsigned char *data, unsigned int address, unsigned int size)
{
    //unsigned char statusReg;
    unsigned char addressBytes[3];

    addressBytes[0]=(unsigned char)(address>>8);
    addressBytes[1]=(unsigned char)(address>>4);
    addressBytes[2]=(unsigned char)(address);

    SPI_CS = 0;             
    WriteSPI(SPI_WREN);     //send write enable
    SPI_CS = 1;

    //do
   // {
    //    statusReg = (Read_SPI_StatusReg() & 0x02);   // Read the Status Register and mask out
    //} while (StatusReg);                             // the WIP bit (bit 0)
    
    __delay_ms(5);          // If you don't want to use the polling method you can
                               // just wait for the max write cycle time (5ms)

    SPI_CS = 0;
    WriteSPI(SPI_WRITE);         // Send write command
    WriteSPI(addressBytes[0]);
    WriteSPI(addressBytes[1]);
    WriteSPI(addressBytes[2]);

    int lcv = 0;
    for(lcv=0;lcv<size;lcv++)
    {
        WriteSPI(data[lcv]);
    }

    SPI_CS = 1;
}

void WriteDataSPI(unsigned char *data, unsigned int address, unsigned int size)
{
    int dataIndex=0;
    int pageRemainingBytes =(SPI_PAGE_SIZE-(address%SPI_PAGE_SIZE));
    int statusReg;

    while(size > pageRemainingBytes)
    {
        do
        {
            statusReg = (ReadStatusSPI() & 0x01);   // Read the Status Register and mask out
        } while (statusReg);

        WritePageSPI(&data[dataIndex], address, pageRemainingBytes);
        address += pageRemainingBytes;
        dataIndex += pageRemainingBytes;
        size -= pageRemainingBytes;
        pageRemainingBytes =(SPI_PAGE_SIZE-(address%SPI_PAGE_SIZE));
    }

    do
    {
        statusReg = (ReadStatusSPI() & 0x01);   // Read the Status Register and mask out
    } while (statusReg);
    WritePageSPI(&data[dataIndex], address, size);
}
*/
/*
void ReadDataSPI(unsigned char *data, unsigned int address, unsigned int size)
{
    unsigned char addressBytes[3];
    addressBytes[0]=(unsigned char)(address>>8);
    addressBytes[1]=(unsigned char)(address>>4);
    addressBytes[2]=(unsigned char)(address);

    int StatusReg;
    do
    {
        StatusReg = (ReadStatusSPI() & 0x01);   // Read the Status Register and mask out
    } while (StatusReg);

    SPI_CS = 0;
    WriteSPI(SPI_READ);
    WriteSPI(addressBytes[0]);
    WriteSPI(addressBytes[1]);
    WriteSPI(addressBytes[2]);

    int lcv;
    for(lcv=0;lcv<size;lcv++)
    {
        data[lcv]=ReadSPI();
    }

    SPI_CS = 1;
}
*/

unsigned char ReadStatusSPI(void)
{
    unsigned char dataRead;

    SPI_CS = CS_ACTIVE;
    WriteSPI(SPI_RDSR);     // Send read status register command
    dataRead = ReadSPI();   // Read the data
    SPI_CS = CS_IDLE;

    return(dataRead);
}

void ReadOverheadSPI(long int address)
{
    unsigned char addressBytes[3];
    addressBytes[0]=(unsigned char)(address>>16);
    addressBytes[1]=(unsigned char)(address>>8);
    addressBytes[2]=(unsigned char)(address);
    //SPI_CS = 0; //must be removed!

    int StatusReg;
    do
    {
        StatusReg = (ReadStatusSPI() & 0x01);   // Read the Status Register and mask out WIP bit
    } while (StatusReg);

    //__delay_ms(5);
    SPI_CS = CS_ACTIVE; //must be changed!
    WriteSPI(SPI_READ);
    WriteSPI(addressBytes[0]);
    WriteSPI(addressBytes[1]);
    WriteSPI(addressBytes[2]);

    return;
}

//Circular Buffer Functions
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


//FOR TESTING PURPOSES ONLY
void WriteSPI_overhead(long int address)
{

    unsigned char address_bytes[3];
    unsigned char StatusReg;

    address_bytes[0]=(unsigned char)(address>>16);
    address_bytes[1]=(unsigned char)(address>>8);
    address_bytes[2]=(unsigned char)(address);

    do
    {
        StatusReg = (ReadStatusSPI() & 0x01);   // Read the Status Register and mask out write in progress flag
    } while (StatusReg);

    //__delay_ms(5);
    SPI_CS = CS_ACTIVE;     //bring chip select low must be changed!
    WriteSPI(SPI_WREN);     //send write enable
    SPI_CS = CS_IDLE;       //must be changed!

    do
    {
        StatusReg = (ReadStatusSPI() & 0x02);   // Read the Status Register and mask out write in progress flag
    } while (!StatusReg);
    //__delay_ms(5);          // If you don't want to use the polling method you can
                               // just wait for the max write cycle time (5ms)

    SPI_CS = CS_ACTIVE;
    WriteSPI(SPI_WRITE);         // Send write command
    WriteSPI(address_bytes[0]);
    WriteSPI(address_bytes[1]);
    WriteSPI(address_bytes[2]);

    return;
}

