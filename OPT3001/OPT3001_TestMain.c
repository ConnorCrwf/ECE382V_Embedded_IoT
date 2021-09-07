// OPT3001_TestMain.c
//*****************************************************************************
// test main for Robot with OPT3001 temperature sensor
// MSP432 with RSLK Max and BP-BASSENSORSMKII boosterpack
// Daniel and Jonathan Valvano
// March 1, 2020
//****************************************************************************
/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2020
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2020, Jonathan Valvano, All rights reserved.

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
// see OPT3001.h for OPT3001 hardware connections
// see Nokia5110.h LCD hardware connections
// see SSD1306.h for OLED hardware connections
// see UART0.h for UART0 hardware connections

/* J3.25 P4.2
RSLK BMP1 (bumper switch 1), switch to ground and software internal pullup
BP-BASSENSORSMKII Sensor OPT_INT
because the INT pin functionality on OPT3001 Ambient Light Sensor is open-drain, there is not an electrical conflict between the signals

J3.27 P4.5
RSLK BMP3 (bumper switch 3), switch to ground and software internal pullup
BP-BASSENSORSMKII Sensor OPT_V+
this one is different from the other power-down configurations because the OPT3001 is powered directly from P4.5, which works because of its very low quiescent current (3.7 µA)
a) To use the OPT3001 Ambient Light Sensor and not BMP3-5: Un-bolt and remove the left bump switch assembly (BMP3-5) and make P4.5 a GPIO output high to power the Sensor. Warning: Do not make P4.5 a GPIO output high and then press BMP3.
b) To use BMP3 and not the OPT3001 Ambient Light Sensor: Make P4.5 a GPIO input from BMP3 (pressing BMP3 temporarily un-powers OPT3001 Ambient Light Sensor)

 */


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/OPT3001.h"


// Select one of the following three output possibilities
// define USENOKIA
//#define USEOLED 1
#define USEUART

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#define OutUFix1 Nokia5110_OutUFix1

#endif

#ifdef USEOLED
// this batch configures for OLED
#include "../inc/SSD1306.h"
void OLEDinit(void){SSD1306_Init(SSD1306_SWITCHCAPVCC);}
#define Init OLEDinit
#define Clear SSD1306_Clear
#define SetCursor SSD1306_SetCursor
#define OutChar SSD1306_OutChar
#define OutString SSD1306_OutString
#define OutUDec SSD1306_OutUDec
#define OutUFix1 SSD1306_OutUFix1
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "../inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 4){
    UART0_OutString("\n\rL= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutUFix1 UART0_OutUFix1
#endif

/**
 * main.c
 */
OPT3001_Result Light; // lux = 0.01*(2^Exponent)*Result
void main(void){
  DisableInterrupts();
  Clock_Init48MHz();
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  OPT3001_Init();
  Init();
  Clear();
  OutString("OPT3001 test\n");
  SetCursor(0,2); OutString("L = ");
  while(1){
    Clock_Delay1ms(2000);
    Light = OPT3001_ReadLight();
    SetCursor(4,1);
    OutUDec(Light.Exponent);
    OutChar(',');
    OutUDec(Light.Result);
  }
}

