// *****************Texas.c**************
// thread profiler, profile at 10 kHz to TExaSdisplay
//
// Runs on MSP432
// Uses the general board support package on the MSP432.
//
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

#include <stdint.h>
#include "TExaS.h"
#include "../inc/UART0.h"
#include "../inc/CortexM.h"
#include "msp.h"

void LogicAnalyzer(void){ // called 10k/sec
  UCA0TXBUF = LogicData;  // send data to PC
}

// ------------PeriodicTask2_Init------------
// Activate an interrupt to run a user task periodically.
// Give it a priority 0 to 6 with lower numbers
// signifying higher priority.  Equal priority is
// handled sequentially.
// assumes SMCLK is 12MHz, using divide by 24, 500kHz/65536=7.6 Hz
// Input:  task is a pointer to a user function
//         freq is number of interrupts per second
//           1 Hz to 10 kHz
//         priority is a number 0 to 6
// Output: none
void (*PeriodicTask2)(void);   // user function
void PeriodicTask2_Init(void(*task)(void), uint32_t freq, uint8_t priority){
  if((freq < 8) || (freq > 10000)){
    return;                        // invalid input
  }
  if(priority > 6){
    priority = 6;
  }
  PeriodicTask2 = task;            // user function
  // ***************** TimerA3 initialization *****************
  TIMER_A3->CTL &= ~0x0030;               // halt Timer A3
  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=10,       input clock divider /4
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           no interrupt on timer
  // bit0=0,           clear interrupt pending
  TIMER_A3->CTL = 0x0280;
  TIMER_A3->EX0 = 0x0005;  // configure for input clock divider /6
  // bits15-14=00,     no capture mode
  // bits13-12=XX,     capture/compare input select
  // bit11=X,          synchronize capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=0,           compare mode
  // bits7-5=XXX,      output mode
  // bit4=1,           enable capture/compare interrupt on CCIFG
  // bit3=X,           read capture/compare input from here
  // bit2=0,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending
  TIMER_A3->CCTL[0] = 0x0010;
  TIMER_A3->CCR[0] = (500000/freq - 1);      // compare match value
// interrupts enabled in the main program after all devices initialized
  NVIC->IP[3] = (NVIC->IP[3]&0xFF00FFFF)|(priority<<21); // priority
  NVIC->ISER[0] = 0x00004000;         // enable interrupt 14 in NVIC
  TIMER_A3->CTL |= 0x0014;                // reset and start Timer A3 in up mode
}

void TA3_0_IRQHandler(void){
  TA3CCTL0 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  (*PeriodicTask2)();              // execute user task
}

// ------------PeriodicTask2_Stop------------
// Deactivate the interrupt running a user task
// periodically.
// Input: none
// Output: none
void PeriodicTask2_Stop(void){
  TIMER_A3->CCTL[0] &= ~0x0001;             // acknowledge capture/compare interrupt 0
  NVIC->ICER[0] = 0x00004000;         // disable interrupt 14 in NVIC
}

// ************TExaS_Init*****************
// Initialize grader, triggered by periodic timer
// This needs to be called once
// Inputs: Grading or Logic analyzer
// Outputs: none
void TExaS_Init(void){
  // 10 kHz periodic interrupt
  UART0_Init(); // 115200 baud
  LogicData |= 0x80; // bit 7 means logic data
  // enable 10k periodic interrupt if logic analyzer mode
  PeriodicTask2_Init(&LogicAnalyzer,10000,5); // run grader
}

// ************TExaS_Stop*****************
// Stop the transfer
// Inputs:  none
// Outputs: none
void TExaS_Stop(void){
  PeriodicTask2_Stop();
}




