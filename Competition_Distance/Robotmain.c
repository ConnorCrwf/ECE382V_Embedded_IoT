// Robotmain.c
// Runs on MSP432
// Starter project for robot competition
// Daniel and Jonathan Valvano
// March 24, 2019

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

// see bump.h for bumper hardware connections
// see Motor.h for motor hardware connections
// see IRDistance.h IR distance sensor hardware connections
// see Tachometer.h for tachometer sensor hardware connections
// see SSD1306.h for OLED hardware connections
// see Reflectance.h for line sensor hardware connections
// see UART0.h for UART0 hardware connections

#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "..\inc\ADC14.h"
#include "..\inc\Bump.h"
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\IRDistance.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\LPF.h"
#include "..\inc\Motor.h"
#include "..\inc\SSD1306.h"
#include "..\inc\Reflectance.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\Tachometer.h"
#include "..\inc\TimerA1.h"
#include "..\inc\UART0.h"

//**************distance sampling*******************
volatile uint32_t nr,nc,nl; // filtered ADC samples
uint32_t Right,Center,Left; // distance in mm
void IRsampling(void){  // runs at 2000 Hz
  uint32_t raw17,raw14,raw16;
  ADC_In17_14_16(&raw17,&raw14,&raw16); // sample
  nr = LPF_Calc(raw17);   // right is channel 17 P9.0
  nc = LPF_Calc2(raw14);  // center is channel 14, P6.1
  nl = LPF_Calc3(raw16);  // left is channel 16, P9.1
}


uint16_t LeftSpeed;            //  rotations per minute
uint16_t RightSpeed;           // rotations per minute
uint16_t LeftTach;             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;    // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;             // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach;            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;   // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;            // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)

volatile uint32_t Time;    // ms
volatile uint8_t Data;     // QTR-8RC
volatile int32_t Position; // -332 to +332 mm from line
void SysTick_Handler(void){ // every 1ms
  Time = Time + 1;
  if(Time%10==1){
    Reflectance_Start(); // start every 10ms
  }
  if(Time%10==2){
    Data = Reflectance_End(); // finish 1ms later
    Position = Reflectance_Position(Data);
  }
}
// OLED and printf (to PC) used for debugging
void main(void){
  uint32_t raw17,raw14,raw16;
  int i;
  DisableInterrupts();
  Clock_Init48MHz();                     // set system clock to 48 MHz
  ADC0_InitSWTriggerCh17_14_16();        // initialize channels 17,14,16
  ADC_In17_14_16(&raw17, &raw14, &raw16);// sample
  LPF_Init(raw17, 64);  // P9.0/channel 17, 64-wide FIR filter
  LPF_Init2(raw14, 64); // P6.1/channel 14, 64-wide FIR filter
  LPF_Init3(raw16, 64); // P9.1/channel 16, 64-wide FIR filter
  UART0_Initprintf();   // serial port channel to PC, with printf
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_Clear();
  LaunchPad_Init();
  TimerA1_Init(&IRsampling, 250);   // 1/(2us*250)=2000 Hz sampling
  Bump_Init();
  Tachometer_Init();
  Motor_Init();
  Time = 0;
  Reflectance_Init();    // line sensor
  SysTick_Init(48000,3); // set up SysTick for 1kHz interrupts
  EnableInterrupts();    // TimerA1 is priority 2, SysTick is priority 3
  while(1){
    Motor_Stop();
    SSD1306_Clear();
    SSD1306_SetCursor(0, 0);           // zero leading spaces, first row
    SSD1306_OutString("TI-RSLK MAX"); printf("\n\rTI-RSLK MAX\n\r");
    SSD1306_SetCursor(0, 1);
    SSD1306_OutString("Bump to go"); printf("Bump to go\n\r");

    while(Bump_Read() == 0){   // flash the blue LED while not touched
      LaunchPad_Output(BLUE);
      Clock_Delay1ms(100);     // delay ~0.1 sec at 48 MHz
      LaunchPad_Output(0x00);
      Clock_Delay1ms(100);     // delay ~0.1 sec at 48 MHz
    }
    SSD1306_SetCursor(0, 1);
    SSD1306_OutString("Release   "); printf("Release\n\r");

    while(Bump_Read() != 0){   // flash the red LED while touched
       LaunchPad_Output(RED);
       Clock_Delay1ms(100);    // delay ~0.1 sec at 48 MHz
       LaunchPad_Output(0x00);
       Clock_Delay1ms(100);    // delay ~0.1 sec at 48 MHz
    }
    SSD1306_SetCursor(0, 1);
    SSD1306_OutString("Ready ");printf("Ready\n\r");
    for(i=5; i; i=i-1){    // flash the yellow LED
      LaunchPad_Output(RED+GREEN);       // red LED is in bit 0, green LED is in bit 1
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
      SSD1306_OutChar(i+'0');
      LaunchPad_Output(0x00);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }

    while(Bump_Read() == 0){// repeat the control code until the robot hits the wall
      LaunchPad_Output(GREEN);           // green LED is in bit 1
      Motor_Forward(3000, 3000);
      SSD1306_SetCursor(0, 1);
      SSD1306_OutString("Forward   "); printf("Forward\n\r");

      Tachometer_Get(&LeftTach, &LeftDir, &LeftSteps, &RightTach, &RightDir, &RightSteps);
      // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
      LeftSpeed = 2000000/LeftTach;
      RightSpeed = 2000000/RightTach;
      Right = RightConvert(nr);
      Center = CenterConvert(nc);
      Left = LeftConvert(nl);
      // control logic: avoid forward wall by turning due to no room on right or left
      if(Center < 150){
        LaunchPad_Output(RED);
        Motor_Backward(2500, 2500);
        SSD1306_SetCursor(0, 1);
        SSD1306_OutString("Back      ");printf("Back\n\r");
        Clock_Delay1ms(2000);            // delay 2 sec at 48 MHz
      }else if(Left < 150){
        LaunchPad_Output(RED+GREEN);    // yellow
        Motor_Right(2500, 2500);
        SSD1306_OutString("Right     ");printf("Right\n\r");
        Clock_Delay1ms(2000);          // delay 1 sec at 48 MHz
      }else if(Right < 150){
        LaunchPad_Output(RED+BLUE);
        Motor_Left(2500, 2500);
        SSD1306_OutString("Left      ");printf("Left\n\r");
        Clock_Delay1ms(2000);          // delay 1 sec at 48 MHz
      }

      Clock_Delay1ms(62);              // delay ~0.062 sec at 48 MHz
    }
  }
}
