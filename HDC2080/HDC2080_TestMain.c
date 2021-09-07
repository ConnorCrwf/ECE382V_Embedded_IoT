// HDC2080_TestMain.c
//*****************************************************************************
// test main for Robot with HDC2080 humidity/temperature sensor
// MSP432 with RSLK Max and BP-BASSENSORSMKII boosterpack
// Daniel and Jonathan Valvano
// March 2, 2020
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
// see HDC2080.h for HDC2080 hardware connections
// see Nokia5110.h LCD hardware connections
// see SSD1306.h for OLED hardware connections
// see UART0.h for UART0 hardware connections

/* J1.6 P4.3
RSLK BMP2 (bumper switch 2), switch to ground and software internal pullup
BP-BASSENSORSMKII Sensor HDC_V+
Conflict: cannot use both HDC2080 Temperature/Humidity Sensor and bump switch BMP2
Resolution:
a) To use the HDC2080 Temperature and Humidity Sensor and not BMP2: Make P4.3 a GPIO output low to power the Sensor (BMP2 has no functionality if the switch is pressed or released). The software will not have the functionality of being able to turn off power to HDC2080, because if the software makes P4.3 GPIO output high and the user activates BMP2, then P4.3 output high will be shorted to ground. Do not make P4.3 a GPIO output high and then press BMP2.
b) To use BMP2 and not the HDC2080 Temperature and Humidity Sensor: Make P4.3 a GPIO input from BMP2 (pressing BMP2 temporarily powers HDC2080 Temperature and Humidity Sensor).

J3.23 P6.1
RSLK DISTC (analog input from center distance sensor)
BP-BASSENSORSMKII Sensor HDC_INT
Conflict: you cannot use both HDC_INT on the HDC2080 and RSLK DISTC
this is disconnected by default unless the user attached an analog signal here, which will probably need to be disconnected to use the interrupt pin HDC_INT on the HDC2080 Temperature and Humidity Sensor (disabled by default)

 */


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/HDC2080.h"

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
  if(newX == 5){
    UART0_OutString("\n\rH,T = ");
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
uint32_t Humidity;
int32_t Temperature;
void main(void){
  DisableInterrupts();
  Clock_Init48MHz();
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  HDC2080_Init();
  Init();
  Clear();
  OutString("HDC2080 test\n");
  SetCursor(0,2); OutString("H,T = ");
  while(1){
    Clock_Delay1ms(2000);
    Humidity = HDC2080_ReadHumidity();
    Temperature = HDC2080_ReadTemperature();
    SetCursor(5,1);
    OutUFix1(Humidity);
    OutChar(',');
    OutUFix1(Temperature);
  }
}

