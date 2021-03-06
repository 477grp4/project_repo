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
    ANSELAbits.ANSA4 = 0;
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
    PS0                 = 1;    //set prescalar to 2
    PS1                 = 0;
    PS2                 = 0;
    TMR0                = SAMPLE_COUNT; //count 0d40 times to get from (16MHz/4) to 10kHz (was 9B)
    TMR0IE              = 1;    //Enable timer0 interrupt
}

void InitWatchdog()
{
    //Used to wakeup from sleep mode
    WDTCONbits.WDTPS = MAX_PERIOD;    //Set timing interval
    WDTCONbits.SWDTEN = 0;      //Software disable of WDT

}

void CheckDisconnect()
{
    PORTCbits.RC0 = 1;          //Set poll line high
    __delay_us(2);
    
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

void PlayAddress(long int startAddress, long int endAddress)
{
    long int curr_address = startAddress;

    dacOutputFlag = 1;              //signals DAC to output next byte
    ReadOverheadSPI(startAddress);  //Set address

    while((transmitFlag || (!transmitFlag && !MEM_ACCESS && playbackFlag)) && curr_address<=endAddress)
    {
      if(!isFull())
      {
        //DAC_count++;
        curr_address++;
        WriteBuffer(ReadSPI()); //Read single byte into circular buffer
      }
    }

    SPI_CS = CS_IDLE;
    dacOutputFlag = 0;   //signals DAC to stop output
}

void PlaybackMode()
{
  long int curr_address = PLAYBACK_BEGIN_ADDRESS;
  long int end_address = 0;
  //long int end_address = PLAYBACK_END_ADDRESS;
  InitSPI();            //Start-up SPI again
  
  ReadOverheadSPI(PLAYBACK_BEGIN_ADDRESS);
  if(MEM_ACCESS)
  {
      SSPCON1bits.SSPEN=0;  // Disable SPI Port
      //PORTCbits.RC5 = 0;    //Set MOSI low
      //PORTCbits.RC3 = 0;    //Set SCK low
      PORTCbits.PORTC = LATCbits.LATC & 0xD7; //set MOSI and SCK low
      SPI_CS = CS_IDLE;
      dacOutputFlag = 0;   //signals DAC to stop output
      return;
  }

  //Extract Header Data
  end_address = ReadSPI();
  end_address = end_address<<8;
  end_address +=ReadSPI();
  end_address = end_address<<8;
  end_address +=ReadSPI();

  Latitude[0] = ReadSPI();     //degrees
  Latitude[1] = ReadSPI();     //minutes
  Latitude[2] = ReadSPI();     //seconds

  NorthSouth = ReadSPI();       //North/South (N/S)

  Longitude[0] = ReadSPI();      //degrees
  Longitude[1] = ReadSPI();      //minutes
  Longitude[2] = ReadSPI();      //seconds

  EastWest = ReadSPI();         //East/West (E/W)

  dacOutputFlag = 1;      //signals DAC to output next byte
  //Checking transmit/playback before reading a page
  while((transmitFlag || (!transmitFlag && !MEM_ACCESS && playbackFlag)) && (curr_address<=end_address))
  {
    if(!isFull())
    {
      curr_address++;
      WriteBuffer(ReadSPI()); //Read single byte into circular buffer
    }
  }
  SPI_CS = CS_IDLE;
  while((transmitFlag || (!transmitFlag && !MEM_ACCESS && playbackFlag)) && !isEmpty());

  //TODO: read off GPS coordinates
  GPStoAudio();

  SSPCON1bits.SSPEN=0;  // Disable SPI Port
  //PORTCbits.RC5 = 0;    //Set MOSI low
  //PORTCbits.RC3 = 0;    //Set SCK low
  PORTCbits.PORTC = LATCbits.LATC & 0xD7; //set MOSI and SCK low
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
    SPI_CS = CS_ACTIVE;
    __delay_ms(1);
    SPI_CS = CS_IDLE;

    //int StatusReg;

    if(MEM_ACCESS)
        return;
    //do
    //{
    //    StatusReg = (ReadStatusSPI() & 0x01);   // Read the Status Register and mask out WIP bit
    //} while (StatusReg&&!MEM_ACCESS);
    __delay_ms(5); //or wait the max write time
    if(MEM_ACCESS)
        return;

    //__delay_ms(5);
    SPI_CS = CS_ACTIVE;
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
    } while (StatusReg&&!MEM_ACCESS);
    if(MEM_ACCESS)
        return;

    //__delay_ms(5);
    SPI_CS = CS_ACTIVE;     //bring chip select low must be changed!
    WriteSPI(SPI_WREN);     //send write enable
    SPI_CS = CS_IDLE;       //must be changed!

    do
    {
        StatusReg = (ReadStatusSPI() & 0x02);   // Read the Status Register and mask out write in progress flag
    } while (!StatusReg&&!MEM_ACCESS);
    if(MEM_ACCESS)
        return;
    //__delay_ms(5);          // If you don't want to use the polling method you can
                               // just wait for the max write cycle time (5ms)

    SPI_CS = CS_ACTIVE;
    WriteSPI(SPI_WRITE);         // Send write command
    WriteSPI(address_bytes[0]);
    WriteSPI(address_bytes[1]);
    WriteSPI(address_bytes[2]);

    return;
}


void GPStoAudio()
{
    int x = 0;
    unsigned char datas[21];

    datas[0] = Latitude[0]/10+48;
    datas[1] = (Latitude[0]%10)+48;
    datas[2] = 'D';

    datas[3] = Latitude[1]/10+48;
    datas[4] = (Latitude[1]%10)+48;
    datas[5] = 'M';

    datas[6] = Latitude[2]/10+48;
    datas[7] = (Latitude[2]%10)+48;
    datas[8] = 'X';

    datas[9] = NorthSouth;

    datas[10] = Longitude[0]/100+48;
    datas[11] = ((Longitude[0]%100)/10)+48;
    datas[12] = Longitude[0]%10+48;
    datas[13] = 'D';

    datas[14] = Longitude[1]/10+48;
    datas[15] = (Longitude[1]%10)+48;
    datas[16] = 'M';

    datas[17] = Longitude[2]/10+48;
    datas[18] = (Longitude[2]%10)+48;
    datas[19] = 'X';

    datas[20] = EastWest;

    if(transmitFlag || (!transmitFlag && !MEM_ACCESS && playbackFlag))
    {
        __delay_ms(200);
        PlayAddress(START_A, START_0);
    }

    while((transmitFlag || (!transmitFlag && !MEM_ACCESS && playbackFlag)) && (x < 21))
    {
        __delay_ms(200);
        switch (datas[x]) {
            case '0':
                PlayAddress(START_0, START_1);
                break;
            case '1' :
                PlayAddress(START_1, START_2);
                break;
            case '2':
                PlayAddress(START_2, START_3);
                break;
            case '3' :
                PlayAddress(START_3, START_4);
                break;
            case '4':
                PlayAddress(START_4, START_5);
                break;
            case '5' :
                PlayAddress(START_5, START_6);
                break;
            case '6':
                PlayAddress(START_6, START_7);
                break;
            case '7' :
                PlayAddress(START_7, START_8);
                break;
            case '8':
                PlayAddress(START_8, START_9);
                break;
            case '9' :
                PlayAddress(START_9, START_N);
                break;
            case 'N':
                PlayAddress(START_N, START_S);
                break;
            case 'S' :
                PlayAddress(START_S, START_E);
                break;
            case 'E':
                PlayAddress(START_E, START_W);
                break;
            case 'W' :
                PlayAddress(START_W, START_D);
                break;
            case 'D':
                PlayAddress(START_D, START_M);
                break;
            case 'M' :
                PlayAddress(START_M, START_SS);
                break;
            case 'X':
                PlayAddress(START_SS, PLAYBACK_BEGIN_ADDRESS);
                break;
            default:
                break;
        }
        x++;
    }
}

void Hibernate()
{
    unsigned char count = 0;
    WDTCONbits.WDTPS = MAX_PERIOD;
    WDTCONbits.SWDTEN = 1;
    while(!playbackFlag && count++ < NUM_PERIODS)
    {
        SLEEP();
    }
    WDTCONbits.SWDTEN = 0;
}