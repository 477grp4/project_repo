#include <stdio.h>
#include <stdlib.h>
#include <pic16f1783.h>
#include "beacon.h"
#include <xc.h> // include standard header file
#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions
#define SPI_CS LATC2

#define SPI_READ (0x03) //read command
#define SPI_WRITE (0x02) //write command
#define SPI_WREN (0x06) //write enable latch
#define SPI_RDSR (0x05) //read STATUS register
#define SPI_WRDI (0x04) //reset write enable latch

#define SPI_PAGE_SIZE (256)

void InitSPI()
{
    // set up oscillator control register
    OSCCONbits.IRCF = 0x0F; //set OSCCON IRCF bits to select OSC frequency=16Mhz
    OSCCONbits.SCS = 0x02; //set the SCS bits to select internal oscillator block

    //	PORT C Assignments
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

    PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
    SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
    while(!SSPSTATbits.BF);     // Wait for interrupt flag to go high indicating transmission is complete
    databyte = SSPBUF;          // Read the incoming data byte
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