#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions
#define SPI_CS LATC2        //Chip-select for SPI
#define SAMPLE_COUNT (0x48) //sampling/playback frequency for DAC and ADC
#define MEM_ACCESS PORTAbits.RA4
#define TRANS_ENABLE PORTBbits.RB5

#define isEmpty() (((RING_START)==(RING_END))?(1):(0))
#define isFull() (((RING_END)==(BUFFERSIZE))?(((RING_START)==0)?(1):(0)):(((RING_END+1)==(RING_START))?(1):(0)))
#define BUFFERSIZE 51
#define RING_START (start)
#define RING_END (end)

#define SPI_READ (0x03) //read command
#define SPI_WRITE (0x02) //write command
#define SPI_WREN (0x06) //write enable latch
#define SPI_RDSR (0x05) //read STATUS register
#define SPI_WRDI (0x04) //reset write enable latch
#define SPI_PAGE_SIZE (256)
//#define RECORD_END_ADDRESS 131072

#define CS_IDLE (1)
#define CS_ACTIVE (0)
//for testing
#define PLAYBACK_BEGIN_ADDRESS 0
#define RECORD_END_ADDRESS 72755

//Global variables
int start, end = 0;
unsigned char buffer[BUFFERSIZE];
int playbackFlag = 0;
unsigned char transmitFlag;
unsigned char dacOutputFlag =0;

unsigned char Latitude[3];
unsigned char NorthSouth;
unsigned char Longitude[3];
unsigned char EastWest;


//Function Declarations

//Initializations
void InitCLK();
void InitGPIO();
void InitDAC();
void InitSPI();
void InitTimer0();
void InitWatchdog();

//GPIO
void CheckDisconnect();
void TransmitMode();
void PlaybackMode();

//SPI
void WriteSPI(unsigned char databyte);
unsigned char ReadSPI(void);
//void WritePageSPI(unsigned char *data, unsigned int address, unsigned int size);
//void WriteDataSPI(unsigned char *data, unsigned int address, unsigned int size);
void ReadDataSPI(unsigned char *data, unsigned int address, unsigned int size);
unsigned char ReadStatusSPI(void);
void ReadOverheadSPI(long int address);

//Circular Buffer
void WriteBuffer(unsigned char data);
unsigned char ReadBuffer();

//FOR TESTING PURPOSES ONLY
void WriteSPI_overhead(long int address);
unsigned char Read_SPI_StatusReg(void);