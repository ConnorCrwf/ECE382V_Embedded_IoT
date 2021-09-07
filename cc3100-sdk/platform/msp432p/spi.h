/*
 * spi.h - MSP432P4xx launchpad spi interface implementation
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
P3.8  P4.7 UNUSED     NA        P4.8  P5.1  WLAN_LOG_TX OUT
P3.9  P5.4 UNUSED     NA        P4.9  P3.5  UNUSED      IN (see R74)
P3.10 P5.5 UNUSED     NA        P4.10 P3.7  UNUSED      OUT(see R75)

// UCA0RXD (VCP receive) connected to P1.2, debugging output
// UCA0TXD (VCP transmit) connected to P1.3

*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
    \brief   type definition for the spi channel file descriptor
    \note    On each porting or platform the type could be whatever is needed
            - integer, pointer to structure etc.
*/
typedef short* Fd_t;


/*!
    \brief open spi communication port to be used for communicating with a
           SimpleLink device

    Given an interface name and option flags, this function opens the spi
    communication port and creates a file descriptor. This file descriptor can
    be used afterwards to read and write data from and to this specific spi
    channel.
    The SPI speed, clock polarity, clock phase, chip select and all other
    attributes are all set to hardcoded values in this function.

    \param[in]      ifName    -    points to the interface name/path. The
                    interface name is an optional attributes that the simple
                    link driver receives on opening the device. in systems that
                    the spi channel is not implemented as part of the os device
                    drivers, this parameter could be NULL.
    \param[in]      flags     -    option flags

    \return         upon successful completion, the function shall open the spi
                    channel and return a non-negative integer representing the
                    file descriptor. Otherwise, -1 shall be returned

    \sa             spi_Close , spi_Read , spi_Write
    \note
    \warning
*/

Fd_t spi_Open(char *ifName, unsigned long flags);

/*!
    \brief closes an opened spi communication port

    \param[in]      fd    -     file descriptor of an opened SPI channel

    \return         upon successful completion, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa             spi_Open
    \note
    \warning
*/
int spi_Close(Fd_t fd);

/*!
    \brief attempts to read up to len bytes from SPI channel into a buffer
           starting at pBuff.

    \param[in]      fd     -    file descriptor of an opened SPI channel

    \param[in]      pBuff  -    points to first location to start writing the
                    data

    \param[in]      len    -    number of bytes to read from the SPI channel

    \return         upon successful completion, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa             spi_Open , spi_Write
    \note
    \warning
*/
int spi_Read(Fd_t fd, unsigned char *pBuff, int len);

/*!
    \brief attempts to write up to len bytes to the SPI channel

    \param[in]      fd        -    file descriptor of an opened SPI channel

    \param[in]      pBuff     -    points to first location to start getting the
                    data from

    \param[in]      len       -    number of bytes to write to the SPI channel

    \return         upon successful completion, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa             spi_Open , spi_Read
    \note           This function could be implemented as zero copy and return
                    only upon successful completion of writing the whole buffer,
                    but in cases that memory allocation is not too tight, the
                    function could copy the data to internal buffer, return
                    back and complete the write in parallel to other activities
                    as long as the other SPI activities would be blocked untill
                    the entire buffer write would be completed
    \warning
*/
int spi_Write(Fd_t fd, unsigned char *pBuff, int len);

#ifdef  __cplusplus
}
#endif // __cplusplus
#endif // __SPI_H__
