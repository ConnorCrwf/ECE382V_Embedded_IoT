// main.c
// Runs on MSP432
// Profile of three periodic threads
// Daniel and Jonathan Valvano
// July 18, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Debugging output on
// Task1 P2.4
// Task2 P2.5
// Task3 P2.6

#include <stdint.h>
#include "msp.h"

#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTick.h"

#define P27OUT (*((volatile uint8_t *)(0x42000000+32*0x4C03+4*7)))
#define P26OUT (*((volatile uint8_t *)(0x42000000+32*0x4C03+4*6)))
#define P25OUT (*((volatile uint8_t *)(0x42000000+32*0x4C03+4*5)))
#define P24OUT (*((volatile uint8_t *)(0x42000000+32*0x4C03+4*4)))
/*
#define N1 1000
void Task1(void){
// every 1ms, 10% utilization
  P24OUT = 1;
  Clock_Delay1us(100);
  P24OUT = 0;
}
#define N2 1500
void Task2(void){
// every 1.5ms, 10% utilization
  P25OUT = 1;
  Clock_Delay1us(150);
  P25OUT = 0;
}
#define N3 10000
void Task3(void){
// every 10ms, 1% utilization
  P26OUT = 1;
  Clock_Delay1us(100); //
  P26OUT = 0;
}
*/
#define N1 1000
uint32_t Last1,Max1,Min1;
void Task1(void){uint32_t now,diff;
// every 1ms, 10% utilization
  now = SysTick->VAL;
  if(Last1){
    diff = (Last1-now)&0x00FFFFFF;
    if(diff>Max1) Max1=diff;
    if(diff<Min1) Min1=diff;
  }
  Clock_Delay1us(100);
  Last1 = now;
}
#define N2 1500
uint32_t Last2,Max2,Min2;
void Task2(void){uint32_t now,diff;
// every 1.5ms, 10% utilization
  now = SysTick->VAL;
  if(Last2){
    diff = (Last2-now)&0x00FFFFFF;
    if(diff>Max2) Max2=diff;
    if(diff<Min2) Min2=diff;
  }
  Clock_Delay1us(150);
  Last2 = now;
}
#define N3 10000
uint32_t Last3,Max3,Min3;
void Task3(void){uint32_t now,diff;
// every 10ms, 1% utilization
  now = SysTick->VAL;
  if(Last3){
    diff = (Last3-now)&0x00FFFFFF;
    if(diff>Max3) Max3=diff;
    if(diff<Min3) Min3=diff;
  }
  Clock_Delay1us(100); //
  Last3 = now;
}


void TimerA0_Init(void){
  TIMER_A0->CTL &= ~0x0030;       // 0) halt Timer A0
  TIMER_A0->CTL = 0x0240;         // 1) SMCLK, divide by 2
  TIMER_A0->EX0 = 0x0005;         //    divide by 6
  TIMER_A0->CCTL[1] = 0x0010;       // 2) compare mode, arm CCIFG
  TIMER_A0->CCTL[2] = 0x0010;       //    compare mode, arm CCIFG
  TIMER_A0->CCTL[3] = 0x0010;       //    compare mode, arm CCIFG
  TIMER_A0->CCR[1] = N1/2;          // 3) time of first interrupt
  TIMER_A0->CCR[2] = N2/2;          //
  TIMER_A0->CCR[3] = N3/2;          //
  NVIC->IP[9] = 0x40;         // 4) priority 2
  NVIC->ISER[0] = 0x00000200; // 5) enable interrupt 9 in NVIC
  TIMER_A0->CTL |= 0x0024;    // 6) reset and start in continuous mode
  EnableInterrupts();         // 7) enable interrupts
}
void TA0_N_IRQHandler(void){
  if(TIMER_A0->CCTL[1]&0x0001){
    TIMER_A0->CCTL[1] &= ~0x0001;   // acknowledge compare interrupt 1
    TIMER_A0->CCR[1] = TIMER_A0->CCR[1]+N1;  // set up for next time
    Task1();               // execute user task
  }
  if(TIMER_A0->CCTL[2]&0x0001){
    TIMER_A0->CCTL[2] &= ~0x0001;   // acknowledge compare interrupt 2
    TIMER_A0->CCR[2] = TIMER_A0->CCR[2]+N2;  // set up for next time
    Task2();               // execute user task
  }
  if(TIMER_A0->CCTL[3]&0x0001){
    TIMER_A0->CCTL[3] &= ~0x0001;   // acknowledge compare interrupt 3
    TIMER_A0->CCR[3] = TIMER_A0->CCR[3]+N3;  // set up for next time
    Task3();               // execute user task
  }
}

/**
 * main.c
 */
void main(void){
  Clock_Init48MHz();  // makes bus clock 48 MHz
  SysTick_Init();
  P2->DIR |= 0xF0; // P2.4 P2.5 P2.6, P2.7 debugging profile
  TimerA0_Init();
  Last1=Last2=Last3=0;
  Max1=Max2=Max3=0;
  Min1=Min2=Min3=0x1000000;
  EnableInterrupts();
  while(1){
    P27OUT ^= 1;  // foreground thread
  }
}
