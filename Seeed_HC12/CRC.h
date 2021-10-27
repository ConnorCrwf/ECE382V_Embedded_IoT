/*
 * CRC.h
 *
 *  Created on: Oct 26, 2021
 *      Author: cdcrw
 */

#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>


typedef uint8_t crc;
#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */
#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
crc crcTable[256];

void crcInit(void);

/* crc calculation code from valvano */
crc crcFast(uint8_t const message[], int nBytes);


#endif /* CRC_H_ */
