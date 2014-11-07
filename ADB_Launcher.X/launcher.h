/* 
 * File:   launcher.h
 * Author: 477grp4
 *
 * Created on October 25, 2014, 6:11 PM
 */

#ifndef LAUNCHER_H
#define	LAUNCHER_H

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
#define RECORD_BEGIN_ADDRESS 0xBB68 //beginning of recording header. BEGIN_PAGE_OFFSET depends on this value!
#define RECORD_END_ADDRESS 131072
#define HEADER_OFFSET 11 //BEGIN_PAGE_OFFSET depends on this value!
#define BEGIN_PAGE_OFFSET 115 //121 //this should be (RECORD_BEGINADDRESS+HEADER_OFFSET)%256

#define SAMPLE_PERIOD_UPPER 0xFF
#define SAMPLE_PERIOD_LOWER 0x71

#define STROBE_LED PORTBbits.RB0
#define GREEN_LED PORTBbits.RB4
#define RED_LED PORTBbits.RB5
    
#define MIN_PERIOD 10
#define MAX_PERIOD 14
//#define MAX_PERIOD 18
#define NUM_PERIODS 3   //number of MAX_PERIODS to sleep on hibernate

#define CS_IDLE (1)
#define CS_ACTIVE (0)
    
#define MEM_ACCESS PORTAbits.RA4

#define TIMEOUT_PERIOD 0x00;
#define TIMEOUT_COUNT 61;

//Initializaton Functions
void InitCLK();
void InitGPIO();
void InitADC();
void InitTimer0();
void InitTimer1();
void InitSPI();
void InitWatchdog();


//SPI Functions
void WriteSPI(unsigned char databyte);
unsigned char ReadSPI(void);
unsigned char ReadStatusSPI(void);
void WriteOverheadSPI(long int address);
void SendGPSSPI();
unsigned char CheckDisconnect();

//Circular Buffer Functions
void WriteBuffer(unsigned char data);
unsigned char ReadBuffer();

//FOR TESTING PURPOSES ONLY
void ReadOverheadSPI(long int address);
long int preRecordingAddresses[18];
void PreRecordMode();

//UART Functions
void InitUART();
void uart_xmit(unsigned char mydata_byte);
void uart_write_message(unsigned char * data, int size);

//GPS Functions
unsigned char compute_checksum(unsigned char * data, int size);
void ToggleSleepGPS();
void DisableGPS();
void SetupGPS();
void DecodeGPS();
void UpdateGPS();

//General Functions
void GoToSleep(unsigned char count);
void Hibernate();
void RecordMode();


//Global Variables
unsigned char recordFlag = 0;
int start, end = 0;
unsigned char buffer[BUFFERSIZE];
volatile unsigned char gpsMessage[50];
volatile char gpsIndex;
unsigned char gpsInvalidFlag = 1;
unsigned char messageDoneFlag = 0;
unsigned char strobeFlag = 0;
unsigned char eeprom_timeoutFlag = 0;
unsigned char hasValidGPSFlag = 0;
unsigned char gpsTimeoutState = 0;


//EEPROM Memory Buffers
unsigned char validLatitude[3]  = {40,25,20};
unsigned char validNorthSouth  = 'N';
unsigned char validLongitude[3] = {86,68,20};
unsigned char validEastWest    = 'E';
long int recordEndAdress;

#ifdef	__cplusplus
}
#endif

#endif	/* LAUNCHER_H */

