/*
 * spi.c - MSP432P4xx launchpad spi interface implementation
 * -edited by Jonathan Valvano 12/14/2018
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
P3.8  P4.7 UNUSED     NA        P4.8  P5.1  WLAN_LOG_TX OUT
P3.9  P5.4 UNUSED     NA        P4.9  P3.5  UNUSED      IN (see R74)
P3.10 P5.5 UNUSED     NA        P4.10 P3.7  UNUSED      OUT(see R75)

// UCA0RXD (VCP receive) connected to P1.2, debugging output
// UCA0TXD (VCP transmit) connected to P1.3

*/
#include "msp.h"
#include "simplelink.h"
#include "spi.h"
#include "board.h"

#define ASSERT_CS()     (P3->OUT &= ~BIT0)
#define DEASSERT_CS()   (P3->OUT |= BIT0)


int spi_Close(Fd_t fd){
    /* Disable WLAN Interrupt ... */
    CC3100_InterruptDisable();
    return NONOS_RET_OK;
}
// P3.0 SPI_CS  (GPIO output, active low)
// P1.5 SPI_CLK (UCB0, 12 MHz)
// P1.6 SPI_MOSI(UCB0, polarity 0, phase 0)
// P1.7 SPI_MISO(UCB0, polarity 0, phase 0)
// P4.1 nHIB (GPIO output, 1 to enable)
// P5.1 WLAN_LOG_TX UART log data arrives from CC3100 into MSP432, but this line is not used
// P2.3 NWP_LOG_TX UART log data arrives into MSP432, but this line is not used
// P2.5 IRQ edge triggered interrupt, pull down
// P3.2 UART1_TX UART communication arrives from CC3100 into MSP432, but this line is not used
// P3.3 UART1_RX UART communication to CC3100, but this line is not used, pulled high in this function
// P5.6 UART1_CTS from MSP432 to CC3100, not initialized, this line is initially low, but then goes high
// P6.6 UART1_RTS from CC3100 to MSP432, not used, this line is initially low, but then goes high
Fd_t spi_Open(char *ifName, unsigned long flags){
    /* Configure the SPI CS to be on P3.0 */
    P3->OUT  |= BIT0;  // CS=1
    P3->SEL1 &= ~BIT0;
    P3->SEL0 &= ~BIT0;
    P3->DIR |= BIT0;

    P4->SEL0 &= ~(BIT1); // P4.1 nHIB (GPIO output)
    P4->SEL1 &= ~(BIT1);
    P4->OUT &= ~BIT1;    // nHIB=0
    P4->DIR |=  BIT1;

  //  P1->OUT |= BIT7;
    
    /* Enable pull up on P3.3, CC3100 UART RX */
    P3->OUT |= BIT3;
    P3->SEL0 &= ~(BIT3); 
    P3->SEL1 &= ~(BIT3);
    P3->DIR &=  ~BIT3;  // input, pull up
    P3->REN |= BIT3;

    /* Select the SPI lines: MOSI/MISO on P1.6,7 CLK on P1.5 */
    P1->SEL0 |= (BIT5+BIT6+BIT7);
    P1->SEL1 &= ~(BIT5+BIT6+BIT7);
    /* Put state machine in reset */
    EUSCI_B0->CTLW0 |= UCSWRST;
  //  UCB0CTLW0 |= UCSWRST;
    /* 3-pin, 8-bit SPI master, 
        most sig bit first, 
        polarity 0, phase 0
        SMCLK (12 MHz)  
        keep RESET */
    EUSCI_B0->CTLW0 = UCMSB + UCMST + UCSYNC + UCCKPH + UCSWRST + UCSSEL__SMCLK;
  //  UCB0CTLW0 = UCMSB + UCMST + UCSYNC + UCCKPH;
  //  UCB0CTL1 = UCSWRST + UCSSEL_2;

    /* Set SPI clock */
  //  UCB0BR0 = 0x00; /* f_UCxCLK = 8MHz */
  //  UCB0BR1 = 0;
    EUSCI_B0->BRW = 1;      // SMCLK/1 = 12 MHz
   // UCB0CTL1 &= ~UCSWRST;
    EUSCI_B0->CTLW0 &= ~UCSWRST;

  /* P4.3 - WLAN enable full DS */
 //   P4->SEL1 &= ~BIT3;
 //   P4->SEL0 &= ~BIT3;

 //   P4->OUT &= ~BIT3;
 //   P4->DIR |=  BIT3;

    /* Configure SPI IRQ line on P2.5 */         /*  00 (Setting as a GPIO)*/
    P2->DIR &= ~BIT5;
    P2->SEL1 &= ~BIT5;
    P2->SEL0 &= ~BIT5;
    P2->OUT &= ~BIT5;  // pull down
    P2->REN |= BIT5;

    /* 50 ms delay */
    Delay(50);
    __enable_interrupt();
    
    /* Enable WLAN interrupt */
    CC3100_InterruptEnable();

    return NONOS_RET_OK;
}


int spi_Write(Fd_t fd, unsigned char *pBuff, int len){
  int len_to_return = len;
  ASSERT_CS();
  while (len){
    while (!(EUSCI_B0->IFG&UCTXIFG));
    EUSCI_B0->TXBUF = *pBuff;
    while (!(EUSCI_B0->IFG&UCRXIFG));
    EUSCI_B0->RXBUF;
    len --;
    pBuff++;
  }
    /* At lower SPI clock frequencies the clock may not be in idle state
     * soon after exiting the above loop. Therefore, the user should poll for 
     * for the clock pin (P2.2) to go to idle state(low) before de-asserting 
     * the Chip Select.
     * 
     * while(P2IN & BIT2);
     */
  DEASSERT_CS();
  return len_to_return;
}


int spi_Read(Fd_t fd, unsigned char *pBuff, int len){
  int i = 0;
  ASSERT_CS();
  for(i = 0; i < len; i ++)    {
    while (!(EUSCI_B0->IFG&UCTXIFG));
    EUSCI_B0->TXBUF = 0xFF;
    while (!(EUSCI_B0->IFG&UCRXIFG));
    pBuff[i] = EUSCI_B0->RXBUF;
  }
    /* At lower SPI clock frequencies the clock may not be in idle state
     * soon after exiting the above loop. Therefore, the user should poll for 
     * for the clock pin (P2.2) to go to idle state(low) before de-asserting 
     * the Chip Select.
     * 
     * while(P2IN & BIT2);
     */
  DEASSERT_CS();
  return len;
}

