/*
 * board.c - MSP432P4xx launchpad configuration
 * -edited by Jonathan Valvano 12/16/2018
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
 /* CC3100 booster pack MSP432 connections (unused pins can be used by user application)
Pin  Signal         Direction   Pin   Signal      Direction
P1.1  3.3  VCC        IN        P2.1  Gnd   GND       IN
P1.2  P6.0 UNUSED     NA        P2.2  P2.5  IRQ       OUT
P1.3  P3.2 UART1_TX   OUT       P2.3  P3.0  SPI_CS    IN
P1.4  P3.3 UART1_RX   IN        P2.4  P5.7  UNUSED    NA
P1.5  P4.1 nHIB       IN        P2.5  Reset nRESET    IN
P1.6  P4.3 UNUSED     NA        P2.6  P1.6  SPI_MOSI  IN
P1.7  P1.5 SPI_CLK    IN        P2.7  P1.7  SPI_MISO  OUT
P1.8  P4.6 UNUSED     NA        P2.8  P5.0  UNUSED    NA
P1.9  P6.5 UNUSED     NA        P2.9  P5.2  UNUSED    NA
P1.10 P6.4 UNUSED     NA        P2.10 P3.6  UNUSED    NA

Pin   Signal        Direction   Pin   Signal        Direction
P3.1  +5   +5V        IN        P4.1  P2.7  UNUSED      OUT
P3.2  Gnd  GND        IN        P4.2  P2.6  UNUSED      OUT
P3.3  P6.1 UNUSED     NA        P4.3  P2.4  UNUSED      NA
P3.4  P4.0 UNUSED     NA        P4.4  P5.6  UART1_CTS   IN
P3.5  P4.2 UNUSED     NA        P4.5  P6.6  UART1_RTS   OUT
P3.6  P4.4 UNUSED     NA        P4.6  P6.7  UNUSED      NA
P3.7  P4.5 UNUSED     NA        P4.7  P2.3  NWP_LOG_TX  OUT
P3.8  P4.7 UNUSED     NA        P4.8  P5.2  WLAN_LOG_TX OUT
P3.9  P5.4 UNUSED     NA        P4.9  P3.5  UNUSED      IN (see R74)
P3.10 P5.5 UNUSED     NA        P4.10 P3.7  UNUSED      OUT(see R75)

// UCA0RXD (VCP receive) connected to P1.2, debugging output
// UCA0TXD (VCP transmit) connected to P1.3

*/
#include "simplelink.h"
#include "board.h"
#include "Clock.h"
#include "fpu.h"

#define XT1_XT2_PORT_SEL    P5SEL
#define XT1_ENABLE          (BIT4 + BIT5)
#define XT2_ENABLE          (BIT2 + BIT3)
#define PMM_STATUS_ERROR    1
#define PMM_STATUS_OK       0
#define XT1HFOFFG           0

P_EVENT_HANDLER             pIraEventHandler = 0;
_u8                         IntIsMasked;

/*!
    \brief          Initialize the system clock of MCU
    \param[in]      none
    \return         none
    \note           Bus clock 48MHz, SMCLK 12 MHz, ACLK 32.768kHz
    \warning        FPU on with lasy stacking
*/
void initClk(void){
  Clock_Init48MHz();
  FPU_enableModule();
  FPU_enableLazyStacking();
    /* Globally enable interrupts */
}

/*!
    \brief register an interrupt handler for the host IRQ
    \param[in]      InterruptHdl    -    pointer to interrupt handler function
    \param[in]      pValue          -    pointer to a memory strcuture that is
                    passed to the interrupt handler.
    \return         upon successful registration, the function shall return 0.
                    Otherwise, -1 shall be returned
    \sa
    \note           If there is already registered interrupt handler, the
                    function should overwrite the old handler with the new one
    \warning
*/
int registerInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue){
    pIraEventHandler = InterruptHdl;
    return 0;
}

/*!
    \brief          Disables the CC3100
    \param[in]      none
    \return         none
    \note           nHIB=0
    \warning
*/
void CC3100_disable(void){
  P4->OUT &= ~BIT1;    // nHIB=0
}

/*!
    \brief          Enables the CC3100
    \param[in]      none
    \return         none
    \note           nHIB=1
    \warning
*/
void CC3100_enable(void){
  P4->OUT |= BIT1;    // nHIB=1
}

/*!
    \brief          Enables the interrupt from the CC3100
    \param[in]      none
    \return         none
    \note           edge triggered on IRQ input on P2.5
    \warning
*/
void CC3100_InterruptEnable(void){ 
  P2->DIR &= ~BIT5; // SPI IRQ line on P2.5
  P2->IES &= ~BIT5;
  P2->IE |= BIT5;
  NVIC->IP[36] = 0x40; // priority 2
  NVIC->ISER[1] = 0x00000010;      // enable interrupt 36 in NVIC
}

/*!
    \brief          Disables the interrupt from the CC3100
    \param[in]      none
    \return         none
    \note           stop edge trigger on IRQ input on P2.5
    \warning
*/
void CC3100_InterruptDisable(void){
  P2->DIR &= ~BIT5; // SPI IRQ line on P2.5
  P2->IE &= ~BIT5;
}

/*!
    \brief      Masks the Host IRQ
    \param[in]      none
    \return         none
    \warning
*/
void MaskIntHdlr(void){
    IntIsMasked = TRUE;
}

/*!
    \brief     Unmasks the Host IRQ
    \param[in]      none
    \return         none
    \warning
*/
void UnMaskIntHdlr(void){
    IntIsMasked = FALSE;
}

/*!
    \brief          Stops the Watch Dog timer
    \param[in]      none
    \return         none
    \note
    \warning
*/
void stopWDT(void){
  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;            // Halt the WDT
}


void PORT2_IRQHandler(void){// SPI IRQ line on P2.5
  switch (P2->IV){
/* Vector  P2IV_P2IFG5:  P1IV P1IFG.2 */
    case P2IV__P2IFG5:
      if(pIraEventHandler){
        pIraEventHandler(0);
      }
      break;

/* Default case */
    default:
      break;
  }
}

/*!
    \brief     Produce delay in ms
    \param[in]         interval - Time in ms
    \return            none
    \note
    \warning  not calibrated
*/
void Delay(unsigned long interval){
  while(interval > 0){
    for(volatile int i=0;i<10000;i++){};
    interval--;
  }
}



