/*
 * HC12.h
 *
 *  Created on: Oct 26, 2021
 *      Author: cdcrw
 */

#ifndef HC12_H_
#define HC12_H_

#include <stdint.h>
#include "..\inc\LaunchPad.h"
#include "..\inc\UART0.h"
#include "..\inc\UART1.h"
#include "msp.h"

// for debugging with LEDs
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
// P3->OUT is 8-bit port at 0x4000.4C22
// I/O address be 0x4000.0000+n, and let b represent the bit 0 to 7.
// n=0x4C22, b=0
// bit banded address is 0x4200.0000 + 32*n + 4*b
#define SET (*((volatile uint8_t *)(0x42098440)))

char HC12data;

void HC12_Init(uint32_t baud);
void HC12_ReadAllInput(void);


#endif /* HC12_H_ */
