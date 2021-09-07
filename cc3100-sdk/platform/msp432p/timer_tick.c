/*
 *
 *   Copyright (C) 2015 Texas Instruments Incorporated
 *   Jonathan Valvano 12/16/2018, moved to Timer2A
 *   All rights reserved. Property of Texas Instruments Incorporated.
 *   Restricted rights to use, duplicate or disclose this code are
 *   granted through contract.
 *
 *   The program may not be used without the written permission of
 *   Texas Instruments Incorporated or against the terms and conditions
 *   stipulated in the agreement under which this program has been supplied,
 *   and under no circumstances can it be used with non-TI connectivity device.
 *
 */

#include "timer_tick.h"
#include "msp.h"

/*!
  \brief   Configures and starts the timer for simplelink time-stamping feature
  
  \note    This internal function is used to configure and start timer A
*/

// ***************** TimerA2_Init ****************
// Activate Timer A2 simple count up mode
// Count on Clock A (32.768 kHz), divide by 10
// Inputs:  none
// Outputs: none
void TimerA2_Init(void){
  TIMER_A2->CTL &= ~0x0030;       // halt Timer A2
  // bits15-10=XXXXXX, reserved
  // bits9-8=01,       TASSEL clock source to ACLK
  // bits7-6=01,       ID input clock divider /2
  // bits5-4=00,       MC stop mode
  // bit3=X,           reserved
  // bit2=0,           TACLR, set this bit to clear
  // bit1=0,           no interrupt on timer
  // bit0=X,           TAIFG
  TIMER_A2->CTL = 0x0140;
  // bits15-14=00,     no capture mode
  // bits13-12=XX,     capture/compare input select
  // bit11=X,          synchronize capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=0,           compare mode
  // bits7-5=XXX,      output mode
  // bit4=0,           disable capture/compare interrupt on CCIFG
  // bit3=X,           read capture/compare input from here
  // bit2=0,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending
  TIMER_A2->CCTL[0] = 0x0000;
  TIMER_A2->CCR[0] = 0xFFFF;  // max value
  TIMER_A2->EX0 = 0x0004;     // configure for input clock divider /5
  NVIC->ICER[0] = 0x00001000; // disable interrupt 12 in NVIC
  // bits5-4=00,       MC 01b = Up mode: Timer counts up to TAxCCR0
  // bit2=0,           TACLR, set this bit to clear
  TIMER_A2->CTL |= 0x0014;    // reset and start Timer A2 in up mode
}

/*!
  \brief   returns the time stamp value, 16 bit up counter at 3.278kHz   
  \note    This API uses Timer A2 peripheral for the SL host driver, hence should not be used by the application
*/
unsigned long timer_GetCurrentTimestamp(void){
  unsigned long timer_value = 0;
//Verify timer has been started by checking that the control register was set
  if((0x0010) != ((TIMER_A2->CTL) & (0x0010))){ // up mode set?
    TimerA2_Init();
  }
  timer_value = (unsigned long) TIMER_A2->R;  // 16-bit up counter
  return timer_value;
}
