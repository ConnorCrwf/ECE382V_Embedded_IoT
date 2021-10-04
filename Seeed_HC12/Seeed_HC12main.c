// Seeed_HC12main.c
// Runs on MSP432
// Use the SysTick timer to request interrupts at a particular period.
// Jonathan Valvano
// July 22, 2021

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2020
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2021, Jonathan Valvano, All rights reserved.

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


// built-in LED1 connected to P1.0
// P1.0, P2.0 are an output to profiling scope/logic analyzer
// UCA2RXD (VCP receive) connected to P3.2
// UCA2TXD (VCP transmit) connected to P3.3
// HC12 pin connection
// 5 SET to GPIO      J2.18 from LaunchPad to HC12  (GPIO output){MSP432 P3.0}
// 4 SO to serial RxD J1.3 from HC12 to LaunchPad (UCA2TXD RxD){MSP432 P3.2}
// 3 SI to serial TxD J1.4 from LaunchPad to HC12  (UCA2TXD TxD){MSP432 P3.3}
// 2 GND
// 1 VCC 1N4004 diode to +5V Vin (plus side of diode on +5V)
// Power pin, the requirements of 3.2V to 5.5V
// DC power supply, the supply current is not less
// than 200mA. Note: If the module is to work
// for a long time in the transmit state, it is
// recommended that the power supply voltage
// of more than 4.5V when connected to a
// 1N4007/1N4004 diode, to avoid the module built-in
// LDO fever.
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "msp.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Clock.h"
#include "..\inc\UART0.h"
#include "..\inc\UART1.h"
volatile uint32_t Time,MainCount;
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
// P3->OUT is 8-bit port at 0x4000.4C22
// I/O address be 0x4000.0000+n, and let b represent the bit 0 to 7.
// n=0x4C22, b=0
// bit banded address is 0x4200.0000 + 32*n + 4*b
#define SET (*((volatile uint8_t *)(0x42098440)))

// #define STARTER // uncomment to use start code
// #define LAB1E   // uncomment to use part E code
#define LAB1F   // uncomment to use part F code

/* lab 1 part e globals */
const char* msg = "Hello World!";
uint8_t midx = 0;
uint8_t led = 0;

/* lab 1 message structs */
typedef struct Header {
    uint8_t src_id;
    uint8_t dst_id;
    uint32_t fcnt;
    uint8_t len;
} header_t;

typedef struct Footer {
    uint8_t err_chk;
} footer_t;

uint32_t G_FRAME_COUNT = 0;
uint32_t G_UNSENT_BYTES = 0;
const char header_compare[] = {0xde, 0xad, 0xbe, 0xef, 0x00};

bool G_HEADER_SENT = false;
bool G_BUF_SENT = false;
header_t G_SEND_HEADER;
char G_SEND_BUF[UINT8_MAX];
footer_t G_SEND_FOOTER;
char * G_S_HEADER_PTR = (char *)&G_SEND_HEADER;
char * G_S_BUF_PTR = (char *)&G_SEND_BUF;
char * G_S_FOOTER_PTR = (char *)&G_SEND_FOOTER;

/*
    sets up message send data structures
    returns without doing anything if previous message is not
    finished sending
*/
void send_msg(uint8_t src_id, uint8_t dst_id, char* msg, uint8_t len)
{
    if (G_UNSENT_BYTES > 0) return;
    G_SEND_HEADER.src_id = src_id;
    G_SEND_HEADER.dst_id = dst_id;
    G_SEND_HEADER.len = len;
    G_SEND_HEADER.fcnt = ++G_FRAME_COUNT;
    G_SEND_FOOTER.err_chk = 0x00;
    for (uint8_t i = 0; i < len; ++i) {
        G_SEND_FOOTER.err_chk ^= msg[i];
        G_SEND_BUF[i] = msg[i];
    }
    // while (G_UNSENT_BYTES > 0);
    G_HEADER_SENT = false;
    G_BUF_SENT = false;
    G_S_HEADER_PTR = (char *)&G_SEND_HEADER;
    G_S_BUF_PTR = (char *)&G_SEND_BUF;
    G_S_FOOTER_PTR = (char *)&G_SEND_FOOTER;
    UART1_OutString(header_compare);
    G_UNSENT_BYTES = sizeof(header_t) + len + sizeof(footer_t);
}

bool G_HEADER_RECV = false;
uint8_t G_RECV_LEN = 0;
header_t G_RECV_HEADER;
char G_RECV_BUF[UINT8_MAX];
footer_t G_RECV_FOOTER;
char * G_R_HEADER_PTR = (char *)&G_RECV_HEADER + 4;
char * G_R_BUF_PTR = (char *)&G_RECV_BUF;
char * G_R_FOOTER_PTR = (char *)&G_RECV_FOOTER;

bool G_RECV_START = false;
bool got_incoming_header(char in)
{
    static int cnt = 0;
    if (in == header_compare[cnt]) ++cnt;
    else cnt = 0;
    return (cnt == 4);
}

char HC12data;
// every 10ms
void SysTick_Handler(void){
  uint8_t in;
  LEDOUT ^= 0x01;       // toggle P1.0
  LEDOUT ^= 0x01;       // toggle P1.0
  Time = Time + 1;
  uint8_t ThisInput = LaunchPad_Input();   // either button
  if (ThisInput) send_msg(0, 1, "Hello World!", 12);

#ifdef STARTER
/* begin starter code */
  if(ThisInput){
    if((Time%100) == 0){ // 1 Hz
      HC12data = HC12data^0x01; // toggle '0' to '1'
      if(HC12data == '1'){
        printf("S1\n");
      }else{
        printf("S0\n");
      }
      UART1_OutChar(HC12data);
    }
  }
#elif defined LAB1E
/* begin reliability testing code */
  if (ThisInput){
      if((Time%100) == 0){ // 1 Hz
        printf("Sending: %c\n", msg[midx]);
        UART1_OutChar(msg[midx++]);
        if (midx == sizeof(msg)) midx = 0;
      }
  }
#elif defined LAB1F
  /* handle sending three distinct components of a packet */
  if (G_UNSENT_BYTES > 0) {
      if (!G_HEADER_SENT) {
          UART1_OutChar(*G_S_HEADER_PTR);
          G_HEADER_SENT = (++G_S_HEADER_PTR == G_S_HEADER_PTR + sizeof(header_t));
      } else if (!G_BUF_SENT) {
          UART1_OutChar(*G_S_BUF_PTR);
          G_BUF_SENT = (++G_S_BUF_PTR == G_S_BUF_PTR + G_SEND_HEADER.len);
      } else { // send footer
          UART1_OutChar(*G_S_FOOTER_PTR);
      }
      --G_UNSENT_BYTES;
  }
#endif

  in = UART1_InCharNonBlock();
  if(in){
#ifdef STARTER
    switch(in){
      case '0':
        printf("R0\n");
        LaunchPad_Output(0); // off
        break;
      case '1':
        printf("R1\n");
        LaunchPad_Output(BLUE);
        break;
    }
#elif defined LAB1E
    led ^= 0x01;
    printf("read: %c\n", in);
    LaunchPad_Output(led ? BLUE : 0);
#elif defined LAB1F
    G_RECV_START |= got_incoming_header(in);
    if (G_RECV_START) {
        if (!G_HEADER_RECV) {
            *G_R_HEADER_PTR = in;
            G_HEADER_RECV = (++G_R_HEADER_PTR == (char *)&G_RECV_HEADER + sizeof(header_t));
            if (G_HEADER_RECV) G_RECV_LEN = G_RECV_HEADER.len;
        } else if (G_RECV_LEN > 0) {
            *G_R_BUF_PTR = in;
            --G_RECV_LEN;
        } else {
            *G_R_FOOTER_PTR = in;
            if (G_R_FOOTER_PTR == (char *)&G_RECV_FOOTER + sizeof(footer_t)) {
                *G_R_BUF_PTR = 0;
                /*
                    message has been fully received
                */
                printf("RECV:\nSrc ID: %d, Dst ID: %d, Frame Count: %d\nMSG: %s\n",
                       G_RECV_HEADER.src_id,
                       G_RECV_HEADER.dst_id,
                       G_RECV_HEADER.fcnt,
                       G_RECV_BUF);
                // reset receiving structs and variables
                G_HEADER_RECV = false;
                G_RECV_LEN = 0;
                G_R_HEADER_PTR = (char *)&G_RECV_HEADER + 4;
                G_R_BUF_PTR = (char *)&G_RECV_BUF;
                G_R_FOOTER_PTR = (char *)&G_RECV_FOOTER;
            }
        }
    }
#endif
  }
  LEDOUT ^= 0x01;       // toggle P1.0
}

void HC12_ReadAllInput(void){uint8_t in;
// flush receiver buffer
  in = UART1_InCharNonBlock();
  while(in){
    UART0_OutChar(in);
    in = UART1_InCharNonBlock();
  }
}
void HC12_Init(uint32_t baud){
  P3->SEL0 &= ~0x01;
  P3->SEL1 &= ~0x01;    // configure P3.0 as GPIO
  P3->DIR |= 0x01;      // make P3.0 out
  UART1_InitB(baud);    // serial port to HC12
  HC12data = '1';
  //************ configure the HC12 module**********************
  SET = 0;       // enter AT command mode
  Clock_Delay1ms(40);
  UART1_OutString("AT+B9600\n");  // UART baud rate set to 9600
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  // Important because need to make it work with other groups
  UART1_OutString("AT+C007\n");   // channel 7 selected (001 to 100 valid)
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  UART1_OutString("AT+P8\n");    // highest power level (1 to 8)
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  UART1_OutString("AT+RF\n");    // read FU transmission mode (FU3)
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  UART1_OutString("AT+V\n");    // read firmware
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  SET = 1;  // exit AT command mode
  Clock_Delay1ms(200);
  HC12_ReadAllInput(); // remove any buffered input
  //************ configuration ended***********************
  printf("\nRF_XMT initialization done\n");
}
/**
 * main.c
 */
void main(void){
  Clock_Init48MHz();        // running on crystal
  Time = MainCount = 0;
  SysTick_Init(480000,2);   // set up SysTick for 100 Hz interrupts
  LaunchPad_Init();         // P1.0 is red LED on LaunchPad
  UART0_Initprintf();       // serial port to PC for debugging
  EnableInterrupts();
  printf("\nSeeed_HC12 example -Valvano\n");
  HC12_Init(UART1_BAUD_9600);
  while(1){
    WaitForInterrupt();
     // foreground thread
    MainCount++;
  }
}

