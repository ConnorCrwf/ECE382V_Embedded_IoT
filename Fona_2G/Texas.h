// *****************Texas.h**************
// thread profile logic analyzer to TExaSdisplay
// uses TA3_0_IRQHandler
// Runs on MSP432
// Daniel and Jonathan Valvano
// October 4, 2020

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
   ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2020

   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2020

 Copyright 2020 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */


#ifndef TEXAS_H_
#define TEXAS_H_

char volatile LogicData;
// this is value transmitted at 10kHz to TExaSdisplay

// ************TExaS_Init*****************
// Initialize thread profiler, triggered by periodic timer
// This needs to be called once
// Inputs: none
// Outputs: none
void TExaS_Init(void);

// ************TExaS_Stop*****************
// Stop the transfer
// Inputs:  none
// Outputs: none
void TExaS_Stop(void);

// toggle bit to indicate time Task 0 is started
#define TExaS_Task0() LogicData^=0x01

// toggle bit to indicate time Task 1 is started
#define TExaS_Task1() LogicData^=0x02

// toggle bit to indicate time Task 2 is started
#define TExaS_Task2() LogicData^=0x04

// toggle bit to indicate time Task 3 is started
#define TExaS_Task3() LogicData^=0x08

// toggle bit to indicate time Task 4 is started
#define TExaS_Task4() LogicData^=0x10

// toggle bit to indicate time Task 5 is started
#define TExaS_Task5() LogicData^=0x20

// toggle bit to indicate time Task 6 is started
#define TExaS_Task6() LogicData^=0x40

#endif
