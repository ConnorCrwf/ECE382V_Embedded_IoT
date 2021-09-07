// RobotArmmain.c
// Runs on MSP432
// Basic test functionality for three-servo robot arm.
// Daniel and Jonathan Valvano
// November 16, 2018

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

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

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// The servos exceed the current limitation of standard USB, so when they
// are in use, the system must be powered with a bench power supply or
// the Pololu #3543 Motor Driver and Power Distribution Board.

// Pololu #3543 Vreg (5V regulator output) connected to all three Pololu #136 GP2Y0A21YK0F Vcc's (+5V) and MSP432 +5V (J3.21)
// Pololu #3543 Vreg (5V regulator output) connected to positive side of three 10 uF capacitors physically near the sensors
// Pololu ground connected to all three Pololu #136 GP2Y0A21YK0F grounds and MSP432 ground (J3.22)
// Pololu ground connected to negative side of all three 10 uF capacitors
// MSP432 P9.0 (J5) (analog input to MSP432) connected to right GP2Y0A21YK0F Vout
// MSP432 P6.1 (J3.23) (analog input to MSP432) connected to center GP2Y0A21YK0F Vout
// MSP432 P9.1 (J5) (analog input to MSP432) connected to left GP2Y0A21YK0F Vout

// Sever VCCMD=VREG jumper on Motor Driver and Power Distribution Board and connect VCCMD to 3.3V.
//   This makes P3.7 and P3.6 low power disables for motor drivers.  0 to sleep/stop.
// Sever nSLPL=nSLPR jumper.
//   This separates P3.7 and P3.6 allowing for independent control

// Optional diagnostic LEDs
// P8.0 connected to yellow front right LED
// P8.5 connected to yellow front left LED
// P8.6 connected to red back left LED
// P8.7 connected to red back right LED

// PWM outputs for servos
// P2.4/PM_TA0.1 (PMAP from TA1.1) connected to arm height servo
// P3.5/PM_UCB2CLK (PMAP from TA1.2) connected to arm tilt servo
// P5.7/TA2.2/VREF-/VeREF-/C1.6 connected to gripper servo

// Analog inputs from servos
// P8.4/A21 connected to arm height servo
// P8.3/TA3CLK/A22 connected to arm tilt servo
// P8.2/TA3.2/A23 connected to gripper servo

#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "..\inc\ADC14.h"
#include "..\inc\Bump.h"
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "..\inc\PWM.h"
#include "..\inc\SSD1306.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\Tachometer.h"



#define HEIGHTARRAYLEN 30
// The full range of the arm height servo should be from 1000 탎 (fully raised) to 1900 탎 (fully lowered).
const uint16_t HEIGHTDUTY[HEIGHTARRAYLEN] = {
 3000, 3093, 3186, 3279, 3372, 3466, 3559, 3652, 3745, 3838,
 3931, 4024, 4117, 4210, 4303, 4397, 4490, 4583, 4676, 4769,
 4862, 4955, 5048, 5141, 5234, 5328, 5421, 5514, 5607, 5700};
int HeightIndex = 18;
#define TILTARRAYLEN 30
// The full range of the arm tilt servo should be from 1200 탎 (fully down) to 1900 탎 (fully up).
const uint16_t TILTDUTY[TILTARRAYLEN] = {
 3600, 3672, 3745, 3817, 3890, 3962, 4034, 4107, 4179, 4252,
 4324, 4397, 4469, 4541, 4614, 4686, 4759, 4831, 4903, 4976,
 5048, 5121, 5193, 5266, 5338, 5410, 5483, 5555, 5628, 5700};
int TiltIndex = 9;
#define GRIPARRAYLEN 30
// The full range of the gripper servo should be from about 500 탎 (fully open) to 2400 탎 (fully closed).
const uint16_t GRIPDUTY[GRIPARRAYLEN] = {
 1500, 1697, 1893, 2090, 2286, 2483, 2679, 2876, 3072, 3269,
 3466, 3662, 3859, 4055, 4252, 4448, 4645, 4841, 5038, 5234,
 5431, 5628, 5824, 6021, 6217, 6414, 6610, 6807, 7003, 7200};
int GripIndex = 3;
uint32_t HeightADC, TiltADC, GripADC;

// Expect an approximately 1:1 correspondence between the servo position in milliseconds and the feedback voltage in millivolts.
// So for example, the feedback voltage will be around 1.5 V at the position corresponding to 1.5 ms servo pulses.
// Convert units of period duration to ADC units:
// (16,384 ADC counts/3.3 V) * (1 V/1 millisecond) * (1 millisecond/3,000 timer counts) =
// (16,384 ADC counts/9,900 timer counts) =
// 1.654949495 ADC counts/timer counts
// Check the measured position of each servo and compare it
// with the expected position.  Return the measured position
// relative to the expected position such that 100 is correct,
// 80 is 20% too low, 120 is 20% too high, etc.
uint16_t getactualheight(void){
  uint32_t expectedADC;
  expectedADC = (PWM_RobotArmGetDuty0()<<14)/9900;
  if(expectedADC > 0){
    return ((uint16_t)((HeightADC*100)/expectedADC));
  }else{
    return 0;
  }
}

uint16_t getactualtilt(void){
  uint32_t expectedADC;
  expectedADC = (PWM_RobotArmGetDuty1()<<14)/9900;
  if(expectedADC > 0){
    return ((uint16_t)((TiltADC*100)/expectedADC));
  }else{
    return 0;
  }
}

uint16_t getactualgrip(void){
  uint32_t expectedADC;
  expectedADC = (PWM_RobotArmGetDuty2()<<14)/9900;
  if(expectedADC > 0){
    return ((uint16_t)((GripADC*100)/expectedADC));
  }else{
    return 0;
  }
}

uint32_t Time = 0;               // increments system time every 100 ms
enum RobotArmMode {
  NONE,                          // initial/dummy mode
  MANUAL,                        // use bump switches to manipulate the arm to desired position
  DEMO,                          // automatically attempt to pick up, move, and drop an item
  WAVE                           // eye-catching stationary demo where the arm moves back and forth
} currentMode = NONE;

// ------------avg------------
// Simple math function that returns the average
// value of an array.
// Input: array is an array of 16-bit unsigned numbers
//        length is the number of elements in 'array'
// Output: the average value of the array
// Note: overflow is not considered
uint16_t avg(uint16_t *array, int length){
  int i;
  uint32_t sum = 0;
  for(i=0; i<length; i=i+1){
    sum = sum + array[i];
  }
  return (sum/length);
}


uint16_t DesiredL = 50;                  // desired rotations per minute
uint16_t DesiredR = 50;                  // desired rotations per minute
uint16_t ActualL;                        // actual rotations per minute
uint16_t ActualR;                        // actual rotations per minute
uint16_t LeftDuty = 3750;                // duty cycle of left wheel (0 to 14,998)
uint16_t RightDuty = 3750;               // duty cycle of right wheel (0 to 14,998)
#define TACHBUFF 10                      // number of elements in tachometer array
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)

void SysTick_Handler(void){ // every 100ms
  static uint8_t previous = 0;   // previous button status
  uint8_t current;               // current button status
  uint16_t actual;

  P8->OUT ^= 0x80;               // toggle P8.7
  Time = Time + 1;               // increment system time

  current = Bump_Read();
  if(currentMode == MANUAL){
    if(((current&0x01) == 0x01) && ((previous&0x01) != 0x01)){
      if(HeightIndex > 0){
        HeightIndex = HeightIndex - 1;
        PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
      }
    }
    if(((current&0x02) == 0x02) && ((previous&0x02) != 0x02)){
      if(HeightIndex < (HEIGHTARRAYLEN - 1)){
        HeightIndex = HeightIndex + 1;
        PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
      }
    }
    if(((current&0x04) == 0x04) && ((previous&0x04) != 0x04)){
      if(TiltIndex > 0){
        TiltIndex = TiltIndex - 1;
        PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
      }
    }
    if(((current&0x08) == 0x08) && ((previous&0x08) != 0x08)){
      if(TiltIndex < (TILTARRAYLEN - 1)){
        TiltIndex = TiltIndex + 1;
        PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
      }
    }
    if(((current&0x10) == 0x10) && ((previous&0x10) != 0x10)){
      if(GripIndex > 0){
        GripIndex = GripIndex - 1;
        PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
      }
    }
    if(((current&0x20) == 0x20) && ((previous&0x20) != 0x20)){
      if(GripIndex < (GRIPARRAYLEN - 1)){
        GripIndex = GripIndex + 1;
        PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
      }
    }
  }
  if(currentMode == DEMO){
    if(current != 0){
      // hit the wall or user stop
      Motor_Stop();
    }
  }
  previous = current;

  ADC_In21_22_23(&HeightADC, &TiltADC, &GripADC);// sample ADC

  actual = getactualheight();
  if((actual > 110) || (actual < 90)){
    P8->OUT |= 0x01;             // turn on P8.0
  }else{
    P8->OUT &= ~0x01;            // turn off P8.0
  }
  actual = getactualtilt();
  if((actual > 110) || (actual < 90)){
    P8->OUT |= 0x20;             // turn on P8.5
  }else{
    P8->OUT &= ~0x20;            // turn off P8.5
  }
  actual = getactualgrip();
  if((actual > 110) || (actual < 90)){
    P8->OUT |= 0x40;             // turn on P8.6
  }else{
    P8->OUT &= ~0x40;            // turn off P8.6
  }
}

void updatescreen(void){
  SSD1306_SetCursor(5, 1);
  SSD1306_OutUDec(PWM_RobotArmGetDuty0()/3); // divide by 3 to put in units of usec
  SSD1306_SetCursor(10, 1);
  SSD1306_OutUDec(HeightIndex);
  SSD1306_SetCursor(15, 1);
  SSD1306_OutUDec(getactualheight());
  SSD1306_SetCursor(5, 2);
  SSD1306_OutUDec(PWM_RobotArmGetDuty1()/3); // divide by 3 to put in units of usec
  SSD1306_SetCursor(10, 2);
  SSD1306_OutUDec(TiltIndex);
  SSD1306_SetCursor(15, 2);
  SSD1306_OutUDec(getactualtilt());
  SSD1306_SetCursor(5, 3);
  SSD1306_OutUDec(PWM_RobotArmGetDuty2()/3); // divide by 3 to put in units of usec
  SSD1306_SetCursor(10, 3);
  SSD1306_OutUDec(GripIndex);
  SSD1306_SetCursor(15, 3);
  SSD1306_OutUDec(getactualgrip());
}

void main(void){
  int i;
  uint8_t bump;
  int32_t leftStepsInitial, rightStepsInitial;
  DisableInterrupts();
  Clock_Init48MHz();             // set system clock to 48 MHz
  ADC0_InitSWTriggerCh21_22_23();// initialize channels 21,22,23
  PWM_RobotArmInit(60000, HEIGHTDUTY[HeightIndex], TILTDUTY[TiltIndex], GRIPDUTY[GripIndex]);
  LaunchPad_Init();
  Bump_Init();
  Tachometer_Init();
  Motor_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  // display initial menu
  SSD1306_Clear();
  SSD1306_SetCursor(0, 0); SSD1306_OutString("RSLK 1.1, Valvano");
  SSD1306_SetCursor(0, 1); SSD1306_OutString("Press bumper to run");
  SSD1306_SetCursor(0, 2); SSD1306_OutString("0: Manual Control");
  SSD1306_SetCursor(0, 3); SSD1306_OutString("1: Automatic Demo");
  SSD1306_SetCursor(0, 4); SSD1306_OutString("2: Stationary Wave");
  i = 0;
  do{
    bump = Bump_Read();
    if((bump == 0x00) && (i == 1)){
      i = 2;
    }else if((bump == 0x01) && (i == 0)){
      currentMode = MANUAL;
      i = 1;
      SSD1306_SetCursor(0, 2); SSD1306_OutString("0: Manual Control");
      SSD1306_SetCursor(0, 3); SSD1306_OutString("                    ");
      SSD1306_SetCursor(0, 4); SSD1306_OutString("                    ");
    }else if((bump == 0x02) && (i == 0)){
      currentMode = DEMO;
      i = 1;
      SSD1306_SetCursor(0, 2); SSD1306_OutString("                    ");
      SSD1306_SetCursor(0, 3); SSD1306_OutString("1: Automatic Demo");
      SSD1306_SetCursor(0, 4); SSD1306_OutString("                    ");
    }else if((bump == 0x04) && (i == 0)){
      currentMode = WAVE;
      i = 1;
      SSD1306_SetCursor(0, 2); SSD1306_OutString("                    ");
      SSD1306_SetCursor(0, 3); SSD1306_OutString("                    ");
      SSD1306_SetCursor(0, 4); SSD1306_OutString("2: Stationary Wave");
    }
    Clock_Delay1ms(250);
  }while(i != 2);
  SSD1306_Clear();
  SSD1306_SetCursor(0, 0);
  SSD1306_OutString("      Duty Indx Act%");
  SSD1306_SetCursor(0, 1);
  SSD1306_OutString("Hght");
  SSD1306_SetCursor(0, 2);
  SSD1306_OutString("Tilt");
  SSD1306_SetCursor(0, 3);
  SSD1306_OutString("Grip");
  P8->SEL0 &= ~0xE1;
  P8->SEL1 &= ~0xE1;             // configure P8.7-P8.5 and P8.0 as GPIO
  P8->DIR |= 0xE1;               // make P8.7-P8.5 and P8.0 out
  P8->OUT &= ~0xE1;              // P8.7-P8.5 and P8.0 initially off
  SysTick_Init(4800000, 3);      // set up SysTick for 10 Hz interrupts
  EnableInterrupts();            // SysTick is priority 3
  while(1){
    if(currentMode == MANUAL){
      updatescreen();
      WaitForInterrupt();
    }else if(currentMode == DEMO){
      Motor_Stop();
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Press Bump SW.");
     // pre-determined settings
/*      HeightIndex = 3;
      PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
      TiltIndex = 21;
      PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
      GripIndex = 3;
      PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);*/
      HeightIndex = 24;
      PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
      TiltIndex = 9;
      PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
      GripIndex = 3;
      PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
      while(Bump_Read() == 0){
        // flash the blue LED
        LaunchPad_Output((Time&0x02)<<1);
        WaitForInterrupt();
        WaitForInterrupt();
      }
      Time = 0;
      while(Time < 20){
        // flash the yellow LED
        LaunchPad_Output(0x03);
        WaitForInterrupt();
        LaunchPad_Output(0x00);
        WaitForInterrupt();
      }
      LaunchPad_Output(0x02);
/*      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Lowering Arm  ");
      // pre-determined settings
      i = 0;
      while((HeightIndex < 24) || (TiltIndex > 9)){
        if((i%3) == 0){
          // try to lower arm
          if((getactualheight() > 85) && (getactualheight() < 115) && (HeightIndex < 24)){
            HeightIndex = HeightIndex + 1;
            PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
          }
          // try to lower wrist
          if((getactualtilt() > 85) && (getactualtilt() < 115) && (TiltIndex > 9)){
            TiltIndex = TiltIndex - 1;
            PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
          }
        }
        updatescreen();
        WaitForInterrupt();
        i = i + 1;
      }
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Drive 20 cm.  ");
      Tachometer_Get(&LeftTach[0], &LeftDir, &LeftSteps, &RightTach[0], &RightDir, &RightSteps);
      i = 0;
      DesiredL = 50;
      DesiredR = 50;
      LeftDuty = 3750;
      RightDuty = 3750;
      leftStepsInitial = LeftSteps;
      rightStepsInitial = RightSteps;
      while((Bump_Read() == 0) && (LeftSteps < (leftStepsInitial + 327)) && (RightSteps < (rightStepsInitial + 327))){
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        i = i + 1;
        if(i >= TACHBUFF){
          i = 0;
          // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
          ActualL = 2000000/avg(LeftTach, TACHBUFF);
          ActualR = 2000000/avg(RightTach, TACHBUFF);
          // very simple, very stupid controller
          if((ActualL > (DesiredL + 3)) && (LeftDuty > 100)){
            LeftDuty = LeftDuty - 100;
          }else if((ActualL < (DesiredL - 3)) && (LeftDuty < 14898)){
            LeftDuty = LeftDuty + 100;
          }
          if((ActualR > (DesiredR + 3)) && (RightDuty > 100)){
            RightDuty = RightDuty - 100;
          }else if((ActualR < (DesiredR - 3)) && (RightDuty < 14898)){
            RightDuty = RightDuty + 100;
          }
          Motor_Forward(LeftDuty, RightDuty);
        }
        WaitForInterrupt();
      }
      Motor_Stop();
      if(Bump_Read() != 0){
        SSD1306_SetCursor(0, 5);
        SSD1306_OutString("Crashed! Press Reset");
        while(1){
          // flash the red LED
          LaunchPad_Output(Time&0x01);
          WaitForInterrupt();
        }
      }*/
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Closing Claw  ");
      // pre-determined settings
      i = 0;
      while(i < 50){
        // try to tighten grip
        if((getactualgrip() > 85) && (getactualgrip() < 115) && (GripIndex < (GRIPARRAYLEN - 1))){
          GripIndex = GripIndex + 1;
          PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
        }
        updatescreen();
        WaitForInterrupt();
        i = i + 1;
      }
      if(GripIndex > ((8*GRIPARRAYLEN)/10)){
        // the claw has closed all the way
        SSD1306_SetCursor(0, 5);
        SSD1306_OutString("Missed! Press Reset ");
        GripIndex = 3;
        PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
        while(1){
          // flash the red LED
          LaunchPad_Output(Time&0x01);
          WaitForInterrupt();
        }
      }
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Raising Arm   ");
      // pre-determined settings
      i = 0;
      while((HeightIndex > 3) || (TiltIndex < 21)){
        if((i%3) == 0){
          // try to raise arm
          if((getactualheight() > 85) && (getactualheight() < 115) && (HeightIndex > 3)){
            HeightIndex = HeightIndex - 1;
            PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
          }
          // try to raise wrist
          if((getactualtilt() > 90) && (getactualtilt() < 110) && (TiltIndex < 21)){
            TiltIndex = TiltIndex + 1;
            PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
          }
        }
        updatescreen();
        WaitForInterrupt();
        i = i + 1;
      }
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Turn around.  ");
      Tachometer_Get(&LeftTach[0], &LeftDir, &LeftSteps, &RightTach[0], &RightDir, &RightSteps);
      i = 0;
      DesiredL = 50;
      DesiredR = 50;
      LeftDuty = 3750;
      RightDuty = 3750;
      leftStepsInitial = LeftSteps;
      rightStepsInitial = RightSteps;
      // measure a 14 cm diameter, which may vary depending on how far the wheels are pressed on the motors
      while((Bump_Read() == 0) && (LeftSteps > (leftStepsInitial - 324)) && (RightSteps < (rightStepsInitial + 324))){
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        i = i + 1;
        if(i >= TACHBUFF){
          i = 0;
          // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
          ActualL = 2000000/avg(LeftTach, TACHBUFF);
          ActualR = 2000000/avg(RightTach, TACHBUFF);
          // very simple, very stupid controller
          if((ActualL > (DesiredL + 3)) && (LeftDuty > 100)){
            LeftDuty = LeftDuty - 100;
          }else if((ActualL < (DesiredL - 3)) && (LeftDuty < 14898)){
            LeftDuty = LeftDuty + 100;
          }
          if((ActualR > (DesiredR + 3)) && (RightDuty > 100)){
            RightDuty = RightDuty - 100;
          }else if((ActualR < (DesiredR - 3)) && (RightDuty < 14898)){
            RightDuty = RightDuty + 100;
          }
          Motor_Left(LeftDuty, RightDuty);
        }
        WaitForInterrupt();
      }
      Motor_Stop();
      if(Bump_Read() != 0){
        SSD1306_SetCursor(0, 5);
        SSD1306_OutString("Crashed! Press Reset");
        while(1){
          // flash the red LED
          LaunchPad_Output(Time&0x01);
          WaitForInterrupt();
        }
      }
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Drive 20 cm.  ");
      Tachometer_Get(&LeftTach[0], &LeftDir, &LeftSteps, &RightTach[0], &RightDir, &RightSteps);
      i = 0;
      DesiredL = 50;
      DesiredR = 50;
      LeftDuty = 3750;
      RightDuty = 3750;
      leftStepsInitial = LeftSteps;
      rightStepsInitial = RightSteps;
      while((Bump_Read() == 0) && (LeftSteps < (leftStepsInitial + 327)) && (RightSteps < (rightStepsInitial + 327))){
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        i = i + 1;
        if(i >= TACHBUFF){
          i = 0;
          // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
          ActualL = 2000000/avg(LeftTach, TACHBUFF);
          ActualR = 2000000/avg(RightTach, TACHBUFF);
          // very simple, very stupid controller
          if((ActualL > (DesiredL + 3)) && (LeftDuty > 100)){
            LeftDuty = LeftDuty - 100;
          }else if((ActualL < (DesiredL - 3)) && (LeftDuty < 14898)){
            LeftDuty = LeftDuty + 100;
          }
          if((ActualR > (DesiredR + 3)) && (RightDuty > 100)){
            RightDuty = RightDuty - 100;
          }else if((ActualR < (DesiredR - 3)) && (RightDuty < 14898)){
            RightDuty = RightDuty + 100;
          }
          Motor_Forward(LeftDuty, RightDuty);
        }
        WaitForInterrupt();
      }
      Motor_Stop();
      if(Bump_Read() != 0){
        SSD1306_SetCursor(0, 5);
        SSD1306_OutString("Crashed! Press Reset");
         while(1){
          // flash the red LED
          LaunchPad_Output(Time&0x01);
          WaitForInterrupt();
        }
      }
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Lowering Arm  ");
      // pre-determined settings
      i = 0;
      while((HeightIndex < 24) || (TiltIndex > 9)){
        if((i%3) == 0){
          // try to lower arm
          if((getactualheight() > 85) && (getactualheight() < 115) && (HeightIndex < 24)){
            HeightIndex = HeightIndex + 1;
            PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
          }
          // try to lower wrist
          if((getactualtilt() > 85) && (getactualtilt() < 115) && (TiltIndex > 9)){
            TiltIndex = TiltIndex - 1;
            PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
          }
        }
        updatescreen();
        WaitForInterrupt();
        i = i + 1;
      }
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Opening Claw  ");
       // pre-determined settings
      i = 0;
      while(GripIndex > 3){
        // try to loosen grip
        GripIndex = GripIndex - 1;
        PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
        updatescreen();
        WaitForInterrupt();
        i = i + 1;
      }
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Backing 10 cm.");
      Tachometer_Get(&LeftTach[0], &LeftDir, &LeftSteps, &RightTach[0], &RightDir, &RightSteps);
      i = 0;
      DesiredL = 50;
      DesiredR = 50;
      LeftDuty = 3750;
      RightDuty = 3750;
      leftStepsInitial = LeftSteps;
      rightStepsInitial = RightSteps;
      while((Bump_Read() == 0) && (LeftSteps > (leftStepsInitial - 164)) && (RightSteps > (rightStepsInitial - 164))){
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        i = i + 1;
        if(i >= TACHBUFF){
          i = 0;
          // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
          ActualL = 2000000/avg(LeftTach, TACHBUFF);
          ActualR = 2000000/avg(RightTach, TACHBUFF);
          // very simple, very stupid controller
          if((ActualL > (DesiredL + 3)) && (LeftDuty > 100)){
            LeftDuty = LeftDuty - 100;
          }else if((ActualL < (DesiredL - 3)) && (LeftDuty < 14898)){
            LeftDuty = LeftDuty + 100;
          }
          if((ActualR > (DesiredR + 3)) && (RightDuty > 100)){
            RightDuty = RightDuty - 100;
          }else if((ActualR < (DesiredR - 3)) && (RightDuty < 14898)){
            RightDuty = RightDuty + 100;
          }
          Motor_Backward(LeftDuty, RightDuty);
        }
        WaitForInterrupt();
      }
      Motor_Stop();
      SSD1306_SetCursor(0, 5);
      SSD1306_OutString("Demo: Done!         ");
      i = 0;
      while((i < 500) && (Bump_Read() == 0)){
        switch(i&0x07){
          case 0: LaunchPad_Output(0x01); break;
          case 1: LaunchPad_Output(0x03); break;
          case 2: LaunchPad_Output(0x02); break;
          case 3: LaunchPad_Output(0x06); break;
          case 4: LaunchPad_Output(0x04); break;
          case 5: LaunchPad_Output(0x05); break;
          case 6: LaunchPad_Output(0x07); break;
          case 7: LaunchPad_Output(0x00); break;
        }
        i = i + 1;
        WaitForInterrupt();
      }
    }else if(currentMode == WAVE){
      // pre-determined settings
      HeightIndex = 24;
      PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
      TiltIndex = 9;
      PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
      GripIndex = 3;
      PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
      Time = 0;
      while(Time < 20){
        updatescreen();
        WaitForInterrupt();
      }
      Time = 0;
      while(Time < 35){
        // try to raise arm
        if((getactualheight() > 85) && (getactualheight() < 115) && (HeightIndex > 3)){
          HeightIndex = HeightIndex - 1;
          PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
        }
        updatescreen();
        WaitForInterrupt();
      }
      Time = 0;
      while(Time < 20){
        updatescreen();
        WaitForInterrupt();
      }
      for(i=0; i<3; i=i+1){
        Time = 0;
        while(Time < 20){
          // try to raise wrist
          if((getactualtilt() > 90) && (getactualtilt() < 110) && (TiltIndex < 21)){
            TiltIndex = TiltIndex + 1;
            PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
          }
          // try to tighten grip
          if((getactualgrip() > 85) && (getactualgrip() < 115) && (GripIndex < (GRIPARRAYLEN - 1))){
            GripIndex = GripIndex + 1;
            PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
          }
          updatescreen();
          WaitForInterrupt();
        }
        Time = 0;
        while(Time < 20){
          // try to lower wrist
          if((getactualtilt() > 85) && (getactualtilt() < 115) && (TiltIndex > 9)){
            TiltIndex = TiltIndex - 1;
            PWM_RobotArmDuty1(TILTDUTY[TiltIndex]);
          }
          // try to loosen grip
          GripIndex = GripIndex - 1;
          PWM_RobotArmDuty2(GRIPDUTY[GripIndex]);
          updatescreen();
          WaitForInterrupt();
        }
      }
      Time = 0;
      while(Time < 20){
        updatescreen();
        WaitForInterrupt();
      }
      Time = 0;
      while(Time < 35){
        // try to lower arm
        if((getactualheight() > 85) && (getactualheight() < 115) && (HeightIndex < 24)){
          HeightIndex = HeightIndex + 1;
          PWM_RobotArmDuty0(HEIGHTDUTY[HeightIndex]);
        }
        updatescreen();
        WaitForInterrupt();
      }
    }else{
      WaitForInterrupt();
    }
  }
}
