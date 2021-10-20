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
    uint32_t src_id;
    uint32_t dst_id;
    uint32_t fcnt;
    uint32_t len;
} header_t;

typedef struct Footer {
    uint32_t err_chk;
    uint32_t magic;
} footer_t;

#define MAX_MSG_LEN (UINT8_MAX)
#define MAX_PACKET_LEN (sizeof(header_t) + MAX_MSG_LEN + sizeof(footer_t))

const uint8_t header_compare[] = {0xde, 0xad, 0xbe, 0xef, 0x00};

header_t G_SEND_HEADER;
footer_t G_SEND_FOOTER;
uint32_t G_FRAME_COUNT = 0;
int32_t G_UNSENT_BYTES = 0;
char G_SEND_BUF[MAX_PACKET_LEN];
char * G_SEND_PTR = NULL;

#define MY_ID 1

/*
    sets up message send data structures
    returns without doing anything if previous message is not
    finished sending
*/
void send_msg(uint32_t src_id, uint32_t dst_id, char* msg, uint32_t len)
{
    if (G_UNSENT_BYTES > 0) return;
    printf("sending message\n");

    /* set up data in header */
    G_SEND_HEADER.src_id = src_id;
    G_SEND_HEADER.dst_id = dst_id;
    G_SEND_HEADER.len = len;
    G_SEND_HEADER.fcnt = ++G_FRAME_COUNT;

    /* set up data in footer */
    G_SEND_FOOTER.err_chk = 0x88888888; /* TODO */
    G_SEND_FOOTER.magic = 0x88888888;

    //destination pointer, source pointer, length
    memcpy(G_SEND_BUF, &G_SEND_HEADER, sizeof(header_t));
    memcpy(G_SEND_BUF+sizeof(header_t), msg, len);
    memcpy(G_SEND_BUF+sizeof(header_t)+len, &G_SEND_FOOTER, sizeof(footer_t));

    G_SEND_PTR = G_SEND_BUF;
    // while (G_UNSENT_BYTES > 0);
    UART1_OutString(header_compare);
    G_UNSENT_BYTES = sizeof(header_t) + len + sizeof(footer_t);
}

char G_RECV_BUF[MAX_PACKET_LEN];
char * G_RECV_PTR = NULL;

int32_t G_UNRECV_BYTES = 0;
void parse_incoming_header(char in)
{
    static int cnt = 0;
    if (in == header_compare[cnt]) ++cnt;
    else cnt = 0;
    if (cnt == 4) {
        G_UNRECV_BYTES = sizeof(header_t);
        G_RECV_PTR = G_RECV_BUF;
    }
}

void on_incoming_message(header_t* hdr, uint8_t* msg, footer_t* ftr)
{
    printf("src_id: %d\n", hdr->src_id);
    printf("dst_id: %d\n", hdr->dst_id);
    printf("fcnt: %d\n", hdr->fcnt);
    printf("len: %d\n", hdr->len);
    printf("message: ");
}

char HC12data;
// every 10ms
void SysTick_Handler(void){
  uint8_t in;
  LEDOUT ^= 0x01;       // toggle P1.0
  LEDOUT ^= 0x01;       // toggle P1.0
  Time = Time + 1;
  uint8_t ThisInput = LaunchPad_Input();   // either button
  if (ThisInput) send_msg(MY_ID, 0, "Hello World!", 12);

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
      UART1_OutChar(*G_SEND_PTR);
      ++G_SEND_PTR;
      --G_UNSENT_BYTES;
  }
#endif

  if(UART1_InStatus()){
      in = UART1_InChar();
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
    printf("read: %2X\n", in);
    LaunchPad_Output(led ? BLUE : 0);
#elif defined LAB1F
    if (G_UNRECV_BYTES > 0) {
        *G_RECV_PTR = in;
        --G_UNRECV_BYTES;
        printf("in: %2X, unrecv: %d\n", in, G_UNRECV_BYTES);
        if (G_UNRECV_BYTES == 0) {
            if (in == 0x88) {
                /* got magic in footer, terminate message */
                printf("msg recv'd: sender: %ld\n", ((header_t*)(G_RECV_BUF))->src_id);
            } else {
                /* we're not done yet... */
                G_UNRECV_BYTES = ((header_t*)(G_RECV_BUF))->len + sizeof(footer_t);
                printf("unrecv: %d\n", G_UNRECV_BYTES);
            }
        }
        ++G_RECV_PTR;
    } else {
        parse_incoming_header(in);
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

