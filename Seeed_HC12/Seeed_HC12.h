/*
 * Seed_HC12.h
 *
 *  Created on: Oct 26, 2021
 *      Author: cdcrw
 */

#ifndef SEEED_HC12_H_
#define SEEED_HC12_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "msp.h"
#include "CRC.h"
#include "HC12.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTickInts.h"
#include "TimerA0.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Clock.h"
#include "..\inc\UART0.h"
#include "..\inc\UART1.h"



volatile uint32_t Time,MainCount;

#define MAX_MSG_LEN (UINT8_MAX)
#define MAX_PACKET_LEN (sizeof(header_t) + MAX_MSG_LEN + sizeof(footer_t))


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

typedef struct Ping {
    uint32_t tsent;
    uint32_t sender;
    uint8_t msg[MAX_MSG_LEN - 2*sizeof(uint32_t)];
} ping_t;

volatile uint32_t SEC_COUNT = 0;
volatile uint32_t TOT_BYTES_SEC = 0;

char G_RECV_BUF[MAX_PACKET_LEN];
char * G_RECV_PTR = NULL;

int32_t G_UNRECV_BYTES = 0;


uint8_t midx = 0;
uint8_t led = 0;

const uint8_t header_compare[] = {0xde, 0xad, 0xbe, 0xef, 0x00};

header_t G_SEND_HEADER;
footer_t G_SEND_FOOTER;
uint32_t G_FRAME_COUNT = 0;
int32_t G_UNSENT_BYTES = 0;
char G_SEND_BUF[MAX_PACKET_LEN];
char * G_SEND_PTR = NULL;

volatile uint32_t LATENCY_SEC = 0;
volatile uint32_t LATENCY_CNT = 0;

void send_msg(uint32_t src_id, uint32_t dst_id, const uint8_t* msg, uint32_t len);
void parse_incoming_header(char in);
void on_incoming_message(header_t* hdr, uint8_t* msg, footer_t* ftr);

void SysTick_Handler(void);
void PeriodicTask(void);

#define RX_TIMEOUT 9600
uint32_t RX_TIMEOUT_CNT = 0;

#endif /* SEEED_HC12_H_ */
