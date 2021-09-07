// TMP117_TestMain.c
//*****************************************************************************
// test main for Robot with TMP117 temperature sensor
// MSP432 with RSLK Max and BP-BASSENSORSMKII boosterpack
// Daniel and Jonathan Valvano
// Feb 5, 2020
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
// see TMP117.h for TMP117 hardware connections
// see Nokia5110.h LCD hardware connections
// see SSD1306.h for OLED hardware connections
// see UART0.h for UART0 hardware connections
/* RSLK pin conflict on J3.28 P4.7
   RSLK BMP5 (bumper switch 5)
   BP-BASSENSORSMKII Sensor TMP_LMT_V+
The TMP117 is powered directly from P4.7, which works because of its very low quiescent current (3.5-220 µA)
a) To use the TMP117 Temperature Sensor and not BMP3-5: Un-bolt and remove the left bump switch assembly (BMP3-5) and make P4.7 a GPIO output high to power the Sensor. Warning: Do not make P4.7 a GPIO output high and then press BMP5.
b) To use BMP5 and not the TMP117 Temperature Sensor: Make P4.7 a GPIO input from BMP5 (pressing BMP5 temporarily un-powers TMP116 Temperature Sensor)

RSLK pin conflict on J3.26 P4.4
  RSLK reserved P4.4 for TExaS Scope input
  BP-BASSENSORSMKII uses P4.4 for Sensor TMP_ALERT
Conflict resolution: do not use both TExaS Scope and Sensor TMP_ALERT

 */


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/TMP117.h"
int16_t RawTemperature; // 0.0078125 C
// 1/0.0078125C = 128
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
  if(newX == 6){
    UART0_OutString("\n\rT(C)= ");
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
void main1(void){int32_t fixed;
  DisableInterrupts();
  Clock_Init48MHz();
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  TMP117_Init();
  Init();
  Clear();
  OutString("TMP117 test\nopen loop\n");
  SetCursor(0,2); OutString("T(C)= ");
  while(1){
    Clock_Delay1ms(2000);
    RawTemperature = TMP117_ReadTemperature();
    SetCursor(6,2);
    fixed = (10*RawTemperature)/128; // 0.1C
    OutUFix1(fixed);
  }
}
int32_t Temperature; // units are 0.1 C
int32_t Semaphore;
void Background(int16_t temperature){
// called when new temperature data is ready
// temperature units are 0.0078125 C
  Temperature = (10*temperature)/128; // 0.1C
  Semaphore = 1;
}
void main(void){Semaphore=0;
  DisableInterrupts();
  Clock_Init48MHz();
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  TMP117_Arm(&Background);
  Init(); Clear();
  OutString("TMP117 test\ninterrupt driven\n");
  SetCursor(0,2); OutString("T(C)= ");
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
    if(Semaphore){ Semaphore=0;
      SetCursor(6,2); OutUFix1(Temperature);
    }
  }
}
