// FFTtestmain.c
// Runs on any processor
// Jonathan Valvano
// July 13, 2020

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
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/FFT.h"

// MSP432 memory limited to q=11, N=2048
#define q   11      /* for 2^11 points */
#define N   (1<<q)  /* N-point FFT, iFFT */
complex_t a[N], scratch[N];
uint32_t StartTime;
uint32_t TimeToConvert; // in usec
/*Time to convert
   n    Time(us)
     64   1659
    128   3892
    256   8979
    512  20440
   1024  46035
   2048 102760
 */
int main(void){
  Clock_Init48MHz();
  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
  SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
  /* Fill a[] with a function of known FFT */
  for(int k=0; k<N; k++){
    a[k].Real = 1.0*cos(4*PI*k/(float)N); // all energy in k=2 bin
    a[k].Imag = 0.0;
  }
  /* FFT, iFFT of a[] */
  StartTime = SysTick->VAL;
  fft(a, N, scratch);
  TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48; // usec

  ifft(a, N, scratch);

  while(1){};
}
