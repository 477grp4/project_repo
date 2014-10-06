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

void InitSPI();
void WriteSPI(unsigned char databyte);
unsigned char ReadSPI(void);
void SPIWritePage(unsigned char *data, unsigned int address, unsigned int size);
void SPIWriteData(unsigned char *data, unsigned int address, unsigned int size);
void SPIReadData(unsigned char *data, unsigned int address, unsigned int size);
unsigned char Read_SPI_StatusReg(void);



#ifdef	__cplusplus
}
#endif

#endif	/* BEACON_H */

