#include "launcher.h"
#include <pic.h>
#include <stdio.h>
#include <string.h>


//Initialization Functions
void InitCLK()
{
    OSCCONbits.IRCF = 0xF; //16Mhz depending on PLL
    OSCCONbits.SCS  = 2;    //set the SCS bits to select internal oscillator block
}
void InitGPIO()
{
    //SPI/UART pins are declared in their respective Init functions
    
    //Stobe-Select Interrupt initialization
    TRISAbits.TRISA1 = 1;       //Set RA1 to input
    ANSELAbits.ANSA1 = 0;
    IOCAPbits.IOCAP1 = 1;       //Enable positive edge interrupt for pin RA1
    IOCANbits.IOCAN1 = 1;       //enable negative edge interrupt for pin RA1
    IOCAF            = 0x00;    //Clear IOC flags
    INTCONbits.IOCIE = 1;       //Enable interrupts on change

    //Record Button Interrupt initialization
    TRISBbits.TRISB3 = 1; //set RB3 to input
    ANSELBbits.ANSB3 = 0; //change to digital mode because portB is weird
    IOCBNbits.IOCBN3 = 1; //enable negative edge interrupt for pin RB3
    IOCBPbits.IOCBP3 = 1; //enable positive edge interrupt for pin RB3
    IOCBF = 0x00;
    INTCONbits.IOCIE = 1; //enable interrupts on change

    //Port A
    TRISAbits.TRISA0 = 1;       //Analog input, microphone signal
    //TRISAbits.TRISA1 = 1;       //Strobe Select
    TRISAbits.TRISA2 = 1;       //Beacon charging indicator
    TRISAbits.TRISA3 = 1;       //Play-Button input
    TRISAbits.TRISA4 = 0;       //Mem-Access output
    TRISAbits.TRISA5 = 0;       //Output, unused
    TRISAbits.TRISA6 = 0;       //GPS On/OFF (used for sleep)
    TRISAbits.TRISA7 = 0;       //Output, unused

    //Port B
    TRISBbits.TRISB0 = 0;       //Strobe light output
    TRISBbits.TRISB1 = 1;       //Launcher charging indicator
    TRISBbits.TRISB2 = 0;       //Output, unused
    //TRISBbits.TRISB3 = 1;       //Record Button
    TRISBbits.TRISB4 = 0;       //Green LED
    TRISBbits.TRISB5 = 0;       //Red LED
    
    PORTBbits.RB0    = 0;       //make sure LEDs turn off
    PORTBbits.RB4    = 0;       //make sure LEDs turn off
    PORTBbits.RB5    = 0;       //make sure LEDs turn off

    ANSELBbits.ANSB0 = 0;       //set to digital
    ANSELBbits.ANSB1 = 0;       //set to digital
    ANSELBbits.ANSB2 = 0;       //set to digital
    //ANSELBbits.ANSB3 = 0;       //set to digital
    ANSELBbits.ANSB4 = 0;       //set to digital
    ANSELBbits.ANSB5 = 0;       //set to digital

    //Port C
    TRISCbits.TRISC0 = 0;       //Output, unused
    TRISCbits.TRISC1 = 0;       //Output, unused
    //TRISCbits.TRISC2 = 0;       // RC2 = output to SEE CS Pin
    //TRISCbits.TRISC3 = 0;       // RC3 = SCK output SEE
    //TRISCbits.TRISC4 = 1;       // RC4 = SDI input from SEE
    //TRISCbits.TRISC5 = 0;       // RC5 = SDO output to SEE
    //TRISCbits.TRISC6 = 0;       //UART TX
    //TRISCbits.TRISC7 = 1;       //UART RX
}

void InitADC()
{
    //Configure pin as Analog input
    TRISAbits.TRISA0    = 1;    //ADC input pin
    ANSELAbits.ANSA0    = 1;    //Set RA0 to analog input

    //Configure ADC module
    ADCON1bits.ADCS     = 0x05; //set ADC clock to FOSC/2
    ADCON1bits.ADPREF   = 0x00; //set ADC positive reference to Vcc
    ADCON1bits.ADNREF   = 0x00; //set ADC negative reference to Vss
    ADCON0bits.CHS      = 0x00; //use AN0 for ADC
    ADCON2bits.CHSN     = 0xF;  //Set negative input to Vref-
    ADCON1bits.ADFM     = 0;    //set output mode to sign-magnitude
    ADCON0bits.ADRMD    = 1;    //set ADC to 10-bit mode
    ADCON0bits.ADON     = 1;    //turn on ADC

    //Configure ADC interrupt (Optional)
    ADIF                = 0;    //Clear ADC interrupt flag
    ADIE                = 1;    //Enable ADC interrupt
    PEIE                = 1;    //Enable periph. interrupts
    //GIE                 = 1;    //Enable global interrupts
}

void InitTimer0()
{
    INTCONbits.TMR0IE = 0;
    OPTION_REGbits.TMR0CS = 0; //use instruction clock = FOSC/4 = 4Mhz
    OPTION_REGbits.PSA = 0; //use prescalar
    OPTION_REGbits.PS = 0x3; //use 256 prescalar
    TMR0 = 0;
}

void InitTimer1()
{
    T1CONbits.TMR1CS    = 0;    //FOSC/4
    T1CONbits.T1CKPS    = 2;    //FOSC/8
    T1CONbits.nT1SYNC   = 1;    //Non synchronized input
    T1GCONbits.TMR1GE   = 0;    //Disregard gate function
    TMR1H               = SAMPLE_PERIOD_UPPER; //Set Timer counter to 50 for 10kHz
    TMR1L               = SAMPLE_PERIOD_LOWER;
    T1CONbits.TMR1ON    = 1;    //Enable timer
    
}

void InitSPI()
{
    //PORT C Assignments
    TRISCbits.TRISC2 = 0; // RC2 = output to SEE CS Pin
    TRISCbits.TRISC3 = 0; // RC3 = SCK output SEE
    TRISCbits.TRISC4 = 1; // RC4 = SDI input from SEE
    TRISCbits.TRISC5 = 0; // RC5 = SDO output to SEE

    SPI_CS = CS_IDLE; // start high. Need falling edge to do anything

    // Setup SPI as Master mode, Mode 00 and clock rate of Fosc/16
    SSPCON1bits.SSPEN=0x00;      // Disable SPI Port for configuration
    SSPCON1bits.SSPM=0x01;       // SPI Master mode, clock = Fosc/16 (1 Mhz)
    SSPCON1bits.CKP=0;           // Idle state for clock is low
    SSPSTATbits.CKE=1;          // Transmit occurs on transition falling clock state
    SSPSTATbits.SMP=0;          // Data is sampled at middle of data output time
    SSPCON1bits.SSPEN=0x01;      // Enable SPI Port
}

void InitWatchdog()
{
    //Used to wakeup from sleep mode
    WDTCONbits.WDTPS = 0x0A;    //Set timing interval (0x0A ~= 1s)
    WDTCONbits.SWDTEN = 0;      //Software enable of WDT
}


//SPI Functions
void WriteSPI(unsigned char databyte)
{
    unsigned int temp;
    temp = SSPBUF;            // Read the buffer to clear any remaining data and clear the buffer full bit
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

unsigned char ReadStatusSPI(void)
{
    unsigned char dataRead;

    SPI_CS=CS_ACTIVE;
    WriteSPI(SPI_RDSR);         // Send read status register command
    dataRead = ReadSPI();  // Read the data
    SPI_CS=CS_IDLE;

    return(dataRead);
}

void WriteOverheadSPI(long int address)
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
    SPI_CS = CS_ACTIVE;         //bring chip select low
    WriteSPI(SPI_WREN); //send write enable
    SPI_CS = CS_IDLE;

    do
    {
        StatusReg = (ReadStatusSPI() & 0x02);   // Read the Status Register and mask out write in progress flag
    } while (!StatusReg);

    //__delay_ms(5);          // If you don't want to use the polling method you can
                               // just wait for the max write cycle time (5ms)

    SPI_CS=CS_ACTIVE;
    WriteSPI(SPI_WRITE);         // Send write command
    WriteSPI(address_bytes[0]);
    WriteSPI(address_bytes[1]);
    WriteSPI(address_bytes[2]);

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
void ReadOverheadSPI(long int address)
{
    unsigned char addressBytes[3];
    addressBytes[0]=(unsigned char)(address>>16);
    addressBytes[1]=(unsigned char)(address>>8);
    addressBytes[2]=(unsigned char)(address);

    int StatusReg;

    do
    {
        StatusReg = (ReadStatusSPI() & 0x01);   // Read the Status Register and mask out WIP bit
    } while (StatusReg);


    //__delay_ms(5);
    SPI_CS = CS_ACTIVE;
    WriteSPI(SPI_READ);
    WriteSPI(addressBytes[0]);
    WriteSPI(addressBytes[1]);
    WriteSPI(addressBytes[2]);

    return;
}


//UART Functions
void InitUART()
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
    while(!TXSTAbits.TRMT); //wait for last byte to transmit
}


//GPS Functions
void ToggleSleepGPS()
{
    PORTAbits.RA6 = 0;
    __delay_ms(1000); //Must wait 1 second between pulses
    PORTAbits.RA6 = 1;
    __delay_ms(100);  //.1ms pulse width
    PORTAbits.RA6 = 0;
    __delay_ms(1000);
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
    unsigned char message[50];

    //Disable GGA,GGL,GSA,GSV,RMC,VTG
    for(x = 0; x < 6; x++)
    {
        __delay_ms(1000);
        sprintf(message, "%s%s,0%d,00,00,00*", startSequence, MID, x);
        uart_write_message(message,  22);
    }

    __delay_ms(1000);
    //Enable GLL in query mode
    //sprintf(message, "%s%s,01,01,00,00*", startSequence, MID);
    //uart_write_message(message,  22);

    //Initialize with West Lafayette's cordinates
    //sprintf(message, "%s104,40.441944,-86.9125,0,0,0,0,12,2*", startSequence);
    //uart_write_message(message,  42);

}


void UpdateGPS()
{
    //unsigned int count = 0;
    unsigned char GPSupdateMessage[22]  = "$PSRF103,01,01,01,00*"; //Send for an update
    //Set interrupts for handling incoming GPS signals
    //This will interrupt any outgoing transmission, so only do it when needed.
    RCIE = 1;
    PEIE = 1;
    GIE = 1;

    //if(!PORTCbits.RC7)          //if GPS is off
    //    ToggleSleepGPS();       //Turn GPS on

    gpsIndex = 0;
    uart_write_message(GPSupdateMessage,  22);  //Get a single GLL message
    PORTBbits.PORTB = LATBbits.LATB | 0x20; //turn red LED on
    PORTBbits.PORTB = LATBbits.LATB & 0xEF; //turn green LED off
    TMR0IF = 0;
    TMR0 = TIMEOUT_PERIOD;
    while(!messageDoneFlag&&!TMR0IF);
    if(TMR0IF)
    {
        gpsTimeoutState = 1;
        return;
    }
    PORTBbits.PORTB = LATBbits.LATB & 0xDF; //turn red LED off
    messageDoneFlag = 0;        //clear the message done flag
    DecodeGPS();                //decode the message sent by the GPS


}


void DecodeGPS()
{
    //Used to decode a GLL output message
    unsigned char messageID[7] = "$GPGLL";
    unsigned char latitude[3] = {0,0,0};
    unsigned char longitude[3] = {0,0,0};
    unsigned char northSouth;
    unsigned char eastWest;
    unsigned char status;
    int x;

    for(x=0;x<6;x++)
    {
        if(gpsMessage[x] != messageID[x])
        {
            gpsInvalidFlag = 1;
            return;
        }
    }

    //Skip comma
    x = x + 1;

    //Latitude
    if(gpsMessage[x]!= ',')
    {
        //decode degrees
        latitude[0] = (gpsMessage[x++] - 48)*10;
        latitude[0] += (gpsMessage[x++] - 48);

        //decode minutes
        latitude[1] = (gpsMessage[x++] - 48)*10;
        latitude[1] += (gpsMessage[x++] - 48);

        //skip period
        x = x + 1;

        //decode seconds
        latitude[2] = (gpsMessage[x++] - 48)*10;
        latitude[2] += (gpsMessage[x++] - 48);
        latitude[2] = (latitude[2]*60)/100;

        //skip lesser significant decimals
        x = x + 2;
    }

    //Skip comma
    x = x + 1;

    //North - South
    if(gpsMessage[x] != ',')
        northSouth = gpsMessage[x++];

    //Skip comma
    x = x + 1;

    //Longitude
    if(gpsMessage[x]!= ',')
    {
        //decode degrees
        longitude[0] = (gpsMessage[x++] - 48)*100;
        longitude[0] += (gpsMessage[x++] - 48)*10;
        longitude[0] += (gpsMessage[x++] - 48);

        //decode minutes
        longitude[1] = (gpsMessage[x++] - 48)*10;
        longitude[1] += (gpsMessage[x++] - 48);

        //skip period
        x = x + 1;

        //decode seconds
        longitude[2] = (gpsMessage[x++] - 48)*10;
        longitude[2] += (gpsMessage[x++] - 48);
        longitude[2] = (longitude[2]*60)/100;

        //skip lesser significant decimals
        x = x + 2;
    }

    //Skip comma
    x = x + 1;

    //East - West
    if(gpsMessage[x] != ',')
        eastWest = gpsMessage[x++];

    //Skip comma
    x = x + 1;

    //Skip UTC time
    while(gpsMessage[x++] != ','){}

    //Status
    status = gpsMessage[x];

    if(status == 'A')
        gpsInvalidFlag = 0;
    else
        gpsInvalidFlag = 1;



    if(!gpsInvalidFlag)
    {
      //Copy the valid GPS coordinates to the global buffer
      for(x=0;x<3;x++)
      {
        validLatitude[x]   = latitude[x];
        validLongitude[x]  = longitude[x];
      }
      validNorthSouth = northSouth;
      validEastWest   = eastWest;

      hasValidGPSFlag = 1;

      //SendGPSSPI();
    }
}

void GoToSleep(unsigned char count)
{
    WDTCONbits.WDTPS = count;    //Set timing interval
    WDTCONbits.SWDTEN = 1;                  //Software enable of WDT
    SLEEP();
    WDTCONbits.SWDTEN = 0;                  //Software disable of WDT
}

void Hibernate()
{
    unsigned char count = 0;
    WDTCONbits.WDTPS = MAX_PERIOD;
    WDTCONbits.SWDTEN = 1;
    while(!recordFlag && count++ < NUM_PERIODS)
    {
        SLEEP();
    }
    WDTCONbits.SWDTEN = 0;
}

void RecordMode()
{
    MEM_ACCESS = 1;
    long int address = RECORD_BEGIN_ADDRESS+HEADER_OFFSET;
    int count = BEGIN_PAGE_OFFSET;

    InitSPI();            //Start-up SPI again

    //__delay_ms(1);
    if(CheckDisconnect())
    {
        SPI_CS = CS_IDLE;
        RING_START = 0; //clear the buffer
        RING_END = 0;
        SSPCON1bits.SSPEN=0;  // Disable SPI Port
        //PORTCbits.RC5 = 0;    //Set MOSI low
        //PORTCbits.RC3 = 0;    //Set SCK low
        PORTCbits.PORTC = LATCbits.LATC & 0xD7; //set MOSI and SCK low
        MEM_ACCESS = 0;
        return;
    }

    RING_START = 0; //clear the buffer
    RING_END = 0;
    
    __delay_ms(300); //wait for mic to charge up
    WriteOverheadSPI(address);
    TMR1IF  = 0;    //Clear interrupt flag
    TMR1IE  = 1;    //Start Interrupt
    TMR1ON  = 1;    //Start Timer
    PORTBbits.PORTB = LATBbits.LATB | 0x20; //turn red LED on
    PORTBbits.PORTB = LATBbits.LATB & 0xEF; //turn green LED off

    while((recordFlag) && (address < RECORD_END_ADDRESS))
    {
        if(count>=256) //perform overhead on next page
        {
            SPI_CS = CS_IDLE;   //Used for end of a page
            count = 0;
            WriteOverheadSPI(address);
        }
        if(!isEmpty())
        {
            WriteSPI(ReadBuffer());
            address++;
            count++;
        }

    }
    //Finish writing the circular buffer over spi
    while(!isEmpty() && (address < RECORD_END_ADDRESS))
    {
        if(count>=256) //perform overhead on next page
        {
            SPI_CS = CS_IDLE;   //Used for end of a page
            count = 0;
            WriteOverheadSPI(address);
        }
        if(!isEmpty())
        {
            WriteSPI(ReadBuffer());
            address++;
            count++;
        }
    }
    SPI_CS = CS_IDLE;
    TMR1IE  = 0;    //Stop Interrupt
    TMR1ON  = 0;    //Stop Timer
    __delay_ms(5);

    //Write header end address
    WriteOverheadSPI(RECORD_BEGIN_ADDRESS);
    if(address >= RECORD_END_ADDRESS)
    {
        WriteSPI((RECORD_END_ADDRESS - 1)>>16);
        WriteSPI((RECORD_END_ADDRESS - 1)>>8);
        WriteSPI(RECORD_END_ADDRESS - 1);
    }
    else
    {
        WriteSPI(address>>16);
        WriteSPI(address>>8);
        WriteSPI(address);
    }
    
    SPI_CS = CS_IDLE;
    if(hasValidGPSFlag)
    {
        SendGPSSPI();
        hasValidGPSFlag = 0;
    }
    PORTBbits.PORTB = LATBbits.LATB & 0xDF; //turn red LED off
    RING_START = 0; //clear the buffer
    RING_END = 0;
    SSPCON1bits.SSPEN=0;  // Disable SPI Port
    //PORTCbits.RC5 = 0;    //Set MOSI low
    //PORTCbits.RC3 = 0;    //set SCK low
    PORTCbits.PORTC = LATCbits.LATC & 0xD7; //set MOSI and SCK low
    MEM_ACCESS = 0;
}

void SendGPSSPI()
{
    long int address = RECORD_BEGIN_ADDRESS + 3;
    int x;
    MEM_ACCESS = 1;
    if(SSPCON1bits.SSPEN == 0)
        InitSPI();
/*
    if(CheckDisconnect())
    {
        SPI_CS = CS_IDLE;
        //RING_START = 0; //clear the buffer
        //RING_END = 0;
        SSPCON1bits.SSPEN=0;  // Disable SPI Port
        PORTCbits.RC5 = 0;    //Set MOSI low
        MEM_ACCESS = 0;
        return;
    }
*/
    //Write the Header GPS Data
    WriteOverheadSPI(address);
    for(x=0;x<3;x++)
    {
        WriteSPI(validLatitude[x]);
    }
    WriteSPI(validNorthSouth);
    for(x=0;x<3;x++)
    {
        WriteSPI(validLongitude[x]);
    }
    WriteSPI(validEastWest);

    SPI_CS = CS_IDLE;
    /*
    RING_START = 0; //clear the buffer
    RING_END = 0;
    SSPCON1bits.SSPEN=0;  // Disable SPI Port
    PORTCbits.RC5 = 0;    //Set MOSI low
    MEM_ACCESS = 0;
    */
    return;

}

unsigned char CheckDisconnect()
{
    unsigned char status;
    status = ReadStatusSPI();
    if(status==0xff)
        return 1;
    else
        return 0;
}

//to make pre-recorded messages only!
void PreRecordMode()
{
    long int address = 0;
    int count = 256;
    int x;
    RING_START = 0;
    RING_END = 0;
    MEM_ACCESS = 1;

    if(CheckDisconnect())
    {
        SPI_CS = CS_IDLE;
        RING_START = 0; //clear the buffer
        RING_END = 0;
        SSPCON1bits.SSPEN=0;  // Disable SPI Port
        //PORTCbits.RC5 = 0;    //Set MOSI low
        //PORTCbits.RC3 = 0;    //Set SCK low
        PORTCbits.PORTC = LATCbits.LATC & 0xD7; //set MOSI and SCK low
        MEM_ACCESS = 0;
        return;
    }

    for(x = 0;x < 18; x++)
    {
        while(!recordFlag);
        __delay_ms(320); //wait for mic to charge
        TMR1IF  = 0;    //Clear interrupt flag
        TMR1IE  = 1;    //Start Interrupt
        TMR1ON  = 1;    //Start Timer
        while(((recordFlag) || !isEmpty()) && (address < RECORD_END_ADDRESS))
        {
            if(count>=256) //perform overhead on next page
            {
                SPI_CS = CS_IDLE;
                count = 0;
                WriteOverheadSPI(address);
            }
            if(!isEmpty())
            {
                WriteSPI(ReadBuffer());
                address++;
                count++;
            }
        }
        SPI_CS = CS_IDLE;
        TMR1IF  = 0;    //Clear interrupt flag
        TMR1IE  = 0;    //Stop Interrupt
        TMR1ON  = 0;    //Stop Timer
        preRecordingAddresses[x] = address;
    }
    return;

}

