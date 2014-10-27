/*
 * File:   beacon.h
 * Author: 477grp4
 *
 * Created on October 3, 2014, 9:49 PM
 */

#ifndef BEACON_H
#define	BEACON_H

#ifdef	__cplusplus
extern "C" {
#endif
#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions


#define isEmpty() (((RING_START)==(RING_END))?(1):(0))
#define isFull() (((RING_END)==(BUFFERSIZE))?(((RING_START)==0)?(1):(0)):(((RING_END+1)==(RING_START))?(1):(0)))
#define BUFFERSIZE 51
#define RING_START (start)
#define RING_END (end)

#define SPI_CS LATC2
#define SPI_READ (0x03) //read command
#define SPI_WRITE (0x02) //write command
#define SPI_WREN (0x06) //write enable latch
#define SPI_RDSR (0x05) //read STATUS register
#define SPI_WRDI (0x04) //reset write enable latch
#define DATA_SIZE 10
#define READ_SIZE 10
#define WRITE_ADDRESS 0x000000
#define READ_ADDRESS 0x000000

#define SPI_PAGE_SIZE (256)
#define RECORD_END_ADDRESS 131072

//Global variables
int start, end = 0;
unsigned char buffer[BUFFERSIZE];
unsigned char read_buffer[READ_SIZE];
int DAC_output_flag = 0;
int ADC_sample_flag = 0;
int record_flag = 0;
int playback_flag = 0;
volatile unsigned char gpsMessage[50];
volatile char gpsIndex;
unsigned char gpsInvalidFlag = 0;

//function declarations
void InitCLK();
void InitSPI();
void WriteSPI(unsigned char databyte);
unsigned char ReadSPI(void);
void ReadSPI_overhead(int address);
void WriteSPI_overhead(int address);
void WriteBuffer(unsigned char data);
unsigned char ReadBuffer();
void InitTimer1();
//void SPIWritePage(unsigned char *data, unsigned int address, unsigned int size);
//void SPIWriteData(unsigned char *data, unsigned int address, unsigned int size);
//void SPIReadData(unsigned char *data, unsigned int address, unsigned int size);
unsigned char Read_SPI_StatusReg(void);
void playback_mode(long int record_end_address);
long int record_mode();
void InitRecord();

void InitADC();
unsigned char readADC();

void InitDAC();
void initUART();
unsigned char compute_checksum(unsigned char * data, int size);
void uart_xmit(unsigned char mydata_byte);
void uart_write_message(unsigned char * data, int size);
void ToggleSleepGPS();
void DisableGPS();
void SetupGPS();
void DecodeGPS();

#ifdef	__cplusplus
}
#endif

#endif	/* BEACON_H */

