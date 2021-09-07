// Demonstration project for ASEE 2018
// TI RSLK
// Lab 7 FSM line following
// Lab 12 simple motor
// Lab 16 tachometer (tach on bits 5,4)
// Lab 17 automated road racing
// Lab 19 BLE (phone controlled racing)
// Jonathan Valvano and Daniel Valvano
// January 18, 2019
// *********************NOTICE***************
// This project only contains object code because in essence it contains
// solutions to most of the labs
// *********************NOTICE***************

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
#include "blinker.h"
#include "Lab7.h"
#include "Lab12.h"
#include "Lab16.h"
#include "Lab17.h"
#include "Lab19.h"
#include "OutputType.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
#include "../inc/Nokia5110.h"
#include "../inc/MotorSimple.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/LPF.h"
#include "../inc/SysTickInts.h"
#include "../inc/ADC14.h"
#include "../inc/SSD1306.h"

enum outputtype CurrentOutputType = OLED;

#define ROBOT 3

// ***********************************************************
// starts with menu
int main(void){ uint8_t in;
  Clock_Init48MHz();
  LaunchPad_Init();   // built-in switches and LEDs
  Bump_Init();        // bump switches
  Blinker_Init();     // blinker LEDs
  while(1){int i=0;
    if(CurrentOutputType == LCD){
      Nokia5110_Init();
      Nokia5110_Clear();
      Nokia5110_SetCursor(0,0); Nokia5110_OutString("0: 7  FSM");
      Nokia5110_SetCursor(0,1); Nokia5110_OutString("1: 12 motor");
      Nokia5110_SetCursor(0,2); Nokia5110_OutString("2: 16 tach");
      Nokia5110_SetCursor(0,3); Nokia5110_OutString("3: 17 cntrl");
      Nokia5110_SetCursor(0,4); Nokia5110_OutString("4: 19 BLE");
      Nokia5110_SetCursor(0,5); Nokia5110_OutString("Robot ");Nokia5110_OutUDec16(ROBOT);
    }else if(CurrentOutputType == OLED){
      SSD1306_Init(SSD1306_SWITCHCAPVCC);
      SSD1306_Clear();
      SSD1306_SetCursor(0,0); SSD1306_OutString("RSLK 1.1, Valvano");
      SSD1306_SetCursor(0,1); SSD1306_OutString("Press bumper for lab");
      SSD1306_SetCursor(0,2); SSD1306_OutString("0: 7  FSM line follow");
      SSD1306_SetCursor(0,3); SSD1306_OutString("1: 12 open loop motor");
      SSD1306_SetCursor(0,4); SSD1306_OutString("2: 16 closedloop tach");
      SSD1306_SetCursor(0,5); SSD1306_OutString("3: 17 wall follow");
      SSD1306_SetCursor(0,6); SSD1306_OutString("4: 19 BLE");
      SSD1306_SetCursor(0,7); SSD1306_OutString("Robot ");SSD1306_OutUDec16(ROBOT);
    }else if(CurrentOutputType == UART){
      UART0_Initprintf();
      UART0_OutString("\r\nASEE project by Valvano and Valvano\r\n");
      UART0_OutString("High-level test of RSLK 1.1\r\n");
      UART0_OutString("Press bumper switch to select which lab to run:\r\n");
      UART0_OutString("Bump 0 for Lab 7  FSM line following\r\n");
      UART0_OutString("Bump 1 for Lab 12 open loop motor controller\r\n");
      UART0_OutString("Bump 2 for Lab 16 tachometer, constant speed controller\r\n");
      UART0_OutString("Bump 3 for Lab 17 Distance to wall proportional control\r\n");
      UART0_OutString("Bump 4 for Lab 19 BLE, ASEE version\r\n");
      UART0_OutString("Robot "); UART0_OutUDec4(ROBOT); UART0_OutString("\r\n\r\n");
    }
    do{ // wait for touch
      Clock_Delay1ms(250);
      LaunchPad_Output(0); // off
      Blinker_Output(FR_RGHT+BK_LEFT);
      Clock_Delay1ms(250);
      if(i<=ROBOT){
        LaunchPad_Output(3); // red/green
        Blinker_Output(BK_RGHT+FR_LEFT);
      }
      i = (i+1)%(ROBOT+4);
      in = Bump_Read();
    }while (in == 0);
    if(CurrentOutputType == LCD){
      Nokia5110_Clear();
      if(in&0x01) {Nokia5110_SetCursor(0,0); Nokia5110_OutString("0: 7  FSM");}
      if(in&0x02) {Nokia5110_SetCursor(0,1); Nokia5110_OutString("1: 12 motor");}
      if(in&0x04) {Nokia5110_SetCursor(0,2); Nokia5110_OutString("2: 16 tach");}
      if(in&0x08) {Nokia5110_SetCursor(0,3); Nokia5110_OutString("3: 17 cntrl");}
      if(in&0x10) {Nokia5110_SetCursor(0,4); Nokia5110_OutString("4: 19 BLE");}
    }else if(CurrentOutputType == OLED){
      SSD1306_Clear();
      if(in&0x01) {SSD1306_SetCursor(0,2); SSD1306_OutString("0: 7  FSM line follow");}
      if(in&0x02) {SSD1306_SetCursor(0,3); SSD1306_OutString("1: 12 open loop motor");}
      if(in&0x04) {SSD1306_SetCursor(0,4); SSD1306_OutString("2: 16 closedloop tach");}
      if(in&0x08) {SSD1306_SetCursor(0,5); SSD1306_OutString("3: 17 wall follow");}
      if(in&0x10) {SSD1306_SetCursor(0,6); SSD1306_OutString("4: 19 BLE");}
    }else if(CurrentOutputType == UART){
      if(in&0x01) {UART0_OutString("0: 7  FSM\r\n");}
      if(in&0x02) {UART0_OutString("1: 12 motor\r\n");}
      if(in&0x04) {UART0_OutString("2: 16 tach\r\n");}
      if(in&0x08) {UART0_OutString("3: 17 cntrl\r\n");}
      if(in&0x10) {UART0_OutString("4: 19 BLE\r\n");}
    }
    while(Bump_Read()){ // wait for release
      Clock_Delay1ms(200);
      LaunchPad_Output(0); // off
      Blinker_Output(0);
      Clock_Delay1ms(200);
      LaunchPad_Output(1); // red
      Blinker_Output(FR_RGHT+FR_LEFT);
    }
    if(in&0x01) main7(CurrentOutputType);
    if(in&0x02) main12(CurrentOutputType);
    if(in&0x04) main16(CurrentOutputType);
    if(in&0x08) main17(CurrentOutputType);
    if(in&0x10) main19(ROBOT, CurrentOutputType);
  }
}




