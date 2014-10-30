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
#define PLAYBACK_BEGIN_ADDRESS 0xE5A0
#define RECORD_END_ADDRESS 72755


#define START_0 0x0
#define START_1 0xBAF
#define START_2 0x1F33
#define START_3 0x3239
#define START_4 0x44AB
#define START_5 0x55E4
#define START_6 0x692B
#define START_7 0x76BB
#define START_8 0x8190
#define START_9 0x8C3A
#define START_N 0x9668
#define START_S 0xA2CA
#define START_E 0xB02D
#define START_W 0xBB4C
#define START_D 0xC746
#define START_M 0xD279
#define START_SS 0xDBF7
//e5a0




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

void PlayAddress(long int startAddress, long int endAddress);
void ConvertGPS();