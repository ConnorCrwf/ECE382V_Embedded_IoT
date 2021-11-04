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

#include "Seeed_HC12.h"

/* lab 1 part e globals */
const char* msg = "Hello World!";
const char* returnMsg;

#define MY_ID 1
#define DEST_ID 2

/**
 * main.c
 */
void main(void){
  DisableInterrupts();
  Clock_Init48MHz();        // running on crystal
  Time = MainCount = 0;
  SysTick_Init(48000000/1000,1);   // set up SysTick for 1KHz interrupts, priority 1
  TimerA0_Init(&PeriodicTask,12000000/9600);  // set up TimerA0 for 9.6KHz interrupts
  LaunchPad_Init();         // P1.0 is red LED on LaunchPad
  UART0_Initprintf();       // serial port to PC for debugging
  crcInit();
  HC12_Init(UART1_BAUD_9600);
  printf("\nLab1 Init Done\n");
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
     // foreground thread
    MainCount++;
  }
}

// every 1ms
volatile bool SEND_PINGS = false;
void SysTick_Handler(void){
  LEDOUT ^= 0x01;       // toggle P1.0 (i think this is just red)
  LEDOUT ^= 0x01;       // toggle P1.0
  Time = Time + 1;
//  ++SEC_COUNT;
//  if (SEC_COUNT == 10000) {
//      printf("bw: %ld bps\n", TOT_BYTES_SEC*8/10);
//      if (LATENCY_CNT) printf("lat: %ld ms\n", LATENCY_SEC/LATENCY_CNT);
//      TOT_BYTES_SEC = SEC_COUNT = LATENCY_SEC = LATENCY_CNT = 0;
//  }
  uint8_t ThisInput = LaunchPad_Input();   // either button
  if (ThisInput) {
//      /* start/stop ping with button press */
//      SEND_PINGS = !SEND_PINGS;
//      if (SEND_PINGS) {
          ping_t pingout = {Time, MY_ID, "Hello World!"};
          //create message which is then sent out one char at a time with every trigger of the Periodic task
          //send_msg(MY_ID, (MY_ID == 1 ? 2 : 1), &pingout, sizeof(ping_t));
          send_msg(MY_ID, DEST_ID, &pingout, sizeof(ping_t));
//      }
  }
  LEDOUT ^= 0x01;       // toggle P1.0
}

/* Period Task to process bytes received
runs at 9.6KHz (UART at 96000 baud supports theoretical 9600 bytes/s) 
*/
void PeriodicTask(void){
    /* send to the other device  */
    if (G_UNSENT_BYTES > 0) {
        //UART outputs the pointer
        UART1_OutChar(*G_SEND_PTR);
        ++G_SEND_PTR;
        --G_UNSENT_BYTES;
    }

    /* Checkt if we've received anything */
    if(UART1_InStatus()){
        LaunchPad_Output(BLUE);

        // loads bytes in as they come, one char at a time
        uint8_t in = UART1_InChar();
        if (G_UNRECV_BYTES > 0) { //checks if there are some char bytes left in the pipeline then decrements it
            *G_RECV_PTR = in;
            --G_UNRECV_BYTES;
            // checks if there are no char bytes left in pipeline
            //we only get into this once the full message is received
            if (G_UNRECV_BYTES == 0) {
                if (in == 0x88) { /* got magic in footer, terminate message */
                    header_t* hdr = (header_t*)(G_RECV_BUF);  //TODO why are we typecasting G_RECV_BUF?
                    if (hdr->dst_id == MY_ID) {
                        //TODO should we also check if hdr->src_id == DEST_ID to make sure we don't get anything from other devices?
                        on_incoming_message(
                                ((header_t*)(G_RECV_BUF)),
                                (uint8_t*)(G_RECV_BUF+sizeof(header_t)),
                                (footer_t*)(G_RECV_BUF+sizeof(header_t)+hdr->len)
                        );
                    }
                } else {  /* we're not done yet...TODO not done with what yet? */   
                    //TODO what's the purpose of this sizeof(footer_t)? why doesn't footer have a length
                    G_UNRECV_BYTES = ((header_t*)(G_RECV_BUF))->len + sizeof(footer_t);
                }
            }
            //increments pointer to look for next byte coming in once periodic task runs again
            ++G_RECV_PTR;
        } else {  
            // sets G_UNRECV_BYTES = sizeof(header_t) once the header coming in has been compared to deadbeef check value
            // this is important, otherwise the above if g_unrecv_bytes > 0 will never be met
            parse_incoming_header(in);
        }
    } 
    else {  //if nothing is detected as coming in
        if (G_UNRECV_BYTES > 0) { /*but if still haven't received all the bytes we should have, 
            then we probably lost some bytes in the transmission */
            /* time out after 1 second */
            LaunchPad_Output(RED);
            if (++RX_TIMEOUT_CNT == RX_TIMEOUT) {
                printf("recv timeout, missing %ld bytes\n", G_UNRECV_BYTES);
                G_UNRECV_BYTES = 0;
            }
        } else {
            RX_TIMEOUT_CNT = 0;
            LaunchPad_Output(0);
        }
    }
}

/*
    sets up message send data structures
    returns without doing anything if previous message is not
    finished sending
*/
void send_msg(uint32_t src_id, uint32_t dst_id, const uint8_t* msg, uint32_t len)
{
    if (G_UNSENT_BYTES > 0) return;
    // printf("sending message\n");

    /* set up data in header */
    G_SEND_HEADER.src_id = src_id;
    G_SEND_HEADER.dst_id = dst_id;
    G_SEND_HEADER.len = len;
    G_SEND_HEADER.fcnt = ++G_FRAME_COUNT;

    /* set up data in footer */
//    G_SEND_FOOTER.err_chk = 0x88888888; /* TODO */
    //calculates err_chk to be sent anda compared with calculation of error check on receiver side in the fcn on_incoming_msg
    G_SEND_FOOTER.err_chk = crcFast(msg, len);
    G_SEND_FOOTER.magic = 0x88888888;

    //destination pointer, source pointer, length
    //destination is set to appropriate buffer memory position and then incremented to appropriate position in buffer memory for msg and footer
    //copies the source pointer data into the desitnation pointer to be sent out
    //this G_SEND_PTR is sent out when the Periodic Timer is triggered and if G_UNSENT_BYTES > 0 
    //G_UNSENT_BYTES is made to be > 0 when parse_incoming_header is called and the header compare evalutes to true
    memcpy(G_SEND_BUF, &G_SEND_HEADER, sizeof(header_t));
    memcpy(G_SEND_BUF+sizeof(header_t), msg, len);
    memcpy(G_SEND_BUF+sizeof(header_t)+len, &G_SEND_FOOTER, sizeof(footer_t));
    TOT_BYTES_SEC+=len;

    G_SEND_PTR = G_SEND_BUF;
    // while (G_UNSENT_BYTES > 0);
    // TODO Does this create a bottleneck/issue for the Periodictimer since the header compare may still be getting sent or processed 
    // while the G_SEND_BUFFER is also being sent ?
    UART1_OutString(header_compare);
    G_UNSENT_BYTES = sizeof(header_t) + len + sizeof(footer_t);
}

void parse_incoming_header(char in)
{
    static int cnt = 0;
    if (in == header_compare[cnt]) ++cnt;
    else cnt = 0;
    if (cnt == 4) {
        G_UNRECV_BYTES = sizeof(header_t);
        G_RECV_PTR = G_RECV_BUF;
        cnt = 0;
    }
}

void on_incoming_message(header_t* hdr, uint8_t* msg, footer_t* ftr)
{
    
    if (ftr->err_chk != crcFast(msg, hdr->len)) {
        printf("crc mismatch!\n");
    } else {
        TOT_BYTES_SEC+=hdr->len;
    }

    /* ping pong message */
    ping_t* pingback = (ping_t*) msg;
    if (pingback->sender == MY_ID) {  //checks to see if the message received is a result of a ping initialized by THIS device
//        LATENCY_SEC += (Time - pingback->tsent)/2;
//        ++LATENCY_CNT;
        printf("My ID: %ld , Destination ID: %ld \n", MY_ID, DEST_ID);
        printf("ping: %ld ms\n", (Time - pingback->tsent)/2);   
        printf("no timeout, missing %ld bytes\n", 0);
        printf("reply message check is: %s\n", pingback->msg);
//        printf("*replay message = %c\n", *pingback->msg);  //only prints first char
            //printf("%c\n", *name++);
        // if (SEND_PINGS) {
//            ping_t pingout = {Time, MY_ID, "Hello World!"};
//            send_msg(MY_ID, (MY_ID == 1 ? 2 : 1), &pingout, sizeof(ping_t));
        // }
    } else {   //if message coming in is not a result of a ping initialized by THIS device, sends message back to device that originally sent message

        //TODO before we send, should we also check if MY_ID matches the hdr_dst_id?
       
        //src_id, dst_id and other characteristics of the header are set once the button is pressed and the send_msg command is called
        //src_id is in destination parameter spot of send_msg function since that's where we want to send the message back to 
        //it's reversed of way it is set in the send_msg command of SysTick_Handler
        send_msg(hdr->dst_id, hdr->src_id, msg, sizeof(ping_t));
    }
}

int intToAscii(int number) {
    return '0' + number;
}
