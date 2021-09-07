// StepperMotorRobotmain.c
// Runs on MSP432
// Runs a finite state machine with two stepper motors
// Jonathan Valvano
// July 16, 2017

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

#include <stdint.h>
#include "msp.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\Clock.h"
#include "..\inc\LaunchPad.h"
// P5.1 positive logic left bump switch
// P5.0 positive logic right bump switch
// P4.7-P4.4 right wheel stepper motor
// P4.3-P4.0 left wheel stepper
//name       pin
//1A (left)  P4.3
//2A (left)  P4.2
//3A (left)  P4.1
//4A (left)  P4.0
//1A (right) P4.4
//2A (right) P4.5
//4A (right) P4.6 (reversed so both motors move robot forward)
//3A (right) P4.7

struct State{
  uint8_t out;      // P4.7-P4.4 to right motor, P4.3-P4.0 to left
  uint32_t wait;    // in bus cycle units
  uint32_t next[4];
   // input 0x00 means ok,
   //       0x01 means right side bumped something,
   //       0x02 means left side bumped something,
   //       0x03 means head-on collision (both sides bumped something)
};
typedef const struct State State_t;
#define T20 (20*48000)
#define T50 (50*48000)
State_t Fsm[10] = {
  {0x55,T20,{1, 4, 7, 3} }, // S0)initial state where bumpers checked
  {0x66,T20,{2, 2, 2, 0} }, // S1)both forward
  {0xAA,T20,{3, 3, 3, 1} }, // S2)both forward
  {0x99,T20,{0, 0, 0, 2} }, // S3)both forward
  {0x69,T50,{5, 5, 5, 5} }, // S4)left forward; right reverse; turn left
  {0xAA,T50,{6, 6, 6, 6} }, // S5)left forward; right reverse; turn left
  {0x96,T50,{0, 0, 0, 0} }, // S6)left forward; right reverse; turn left
  {0x96,T50,{8, 8, 8, 8} }, // S7)left reverse; right forward; turn right
  {0xAA,T50,{9, 9, 9, 9} }, // S8)left reverse; right forward; turn right
  {0x69,T50,{0, 0, 0, 0} }  // S9)left reverse; right forward; turn right
};
uint32_t cState;        // current State (0 to 9)
int main(void){
  DisableInterrupts();
  Clock_Init48MHz();    // 48 MHz HFXTCLK, SMCLK = 12 MHz
  P4->SEL0 = 0x00;
  P4->SEL1 = 0x00;      // configure stepper motor pins as GPIO
  P4->DIR = 0xFF;       // make stepper motor pins out
  P4->OUT = 0;
  P5->SEL0 &= ~0x03;
  P5->SEL1 &= ~0x03;    // configure bump pins as GPIO
  P5->DIR &= ~0x03;     // make bump switch pins input
  cState = 0;           // initial state = 0
  LaunchPad_Init();
  while(LaunchPad_Input()==0){
    LaunchPad_Output(RED);LaunchPad_LED(0);
    Clock_Delay1ms(250);
    LaunchPad_Output(0);LaunchPad_LED(1);
    Clock_Delay1ms(250);
  }
  LaunchPad_LED(0);
  while(LaunchPad_Input()){
    LaunchPad_Output(0);
    Clock_Delay1ms(125);
    LaunchPad_Output(GREEN);
    Clock_Delay1ms(125);
  }
  SysTick_Init(T20,2);  // initial period will be ignored
  P4->OUT = Fsm[cState].out;         // first output to stepper motor
  SysTick->LOAD = Fsm[cState].wait; // dwell time for first state
  EnableInterrupts();
  while(1){
// other unrelated software tasks
  }
}
void SysTick_Handler(void){ uint32_t input;
  input = P5->IN&0x03;                // get Input 0,1,2,3
  cState = Fsm[cState].next[input];   // next state
  P4->OUT = Fsm[cState].out;          // output to stepper motor
  SysTick->LOAD = Fsm[cState].wait;   // dwell time for state
  SysTick->VAL = 0;         // any value written to counter clears
  LaunchPad_Output(input);
}

