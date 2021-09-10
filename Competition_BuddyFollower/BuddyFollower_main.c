// Buddy follower example.
// RSLK MAX follows object in front of it
// using OPT3101 and fuzzy logic
// December 22, 2019

// *********************NOTICE***************
// This project only contains object code because in essence it contains
// solutions to most of the labs
// *********************NOTICE***************

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

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/blinker.h"
#include "../inc/Bump.h"
#include "../inc/LPF.h"
#include "../inc/TimerA1.h"
#include "../inc/I2CB1.h"
#include "../inc/opt3101.h"
#include "../inc/Fuzzy.h"

// Select one of the following three output possibilities
#define USENOKIA 1
//#define USEOLED 1
// #define USEUART 1

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#endif

#ifdef USEOLED
// this batch configures for OLED
#include "../inc/SSD1306.h"
void OLEDinit(void){SSD1306_Init(SSD1306_SWITCHCAPVCC);}
#define Init OLEDinit
#define Clear SSD1306_Clear
#define SetCursor SSD1306_SetCursor
#define OutChar SSD1306_OutChar
#define OutString SSD1306_OutString
#define OutUDec SSD1306_OutUDec
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "../inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#endif

//*****************System1*************************
// Fuzzy logic buddy follow
// Stage 1) Take measurements to determine crisp Inputs
#define FILTERSIZE 4   // replace with your choice see "FIR_Digital_LowPassFilter.xls"
uint32_t FilteredDistances[3];

int32_t Left;         // distance to left object in mm
int32_t Center;       // distance to center object in mm
int32_t Right;        // distance to right object in mm
int32_t Distance;     // distance to closed object in mm
int32_t LeftBuddy;    // Center-Left, positive if closer to left
int32_t RightBuddy;   // Center-Right, positive if closer to right
#define OVERFLOW 20000  // over this value is no object
#define TURN 100
void Measurements(void){
  Left   = umin32(FilteredDistances[0],OVERFLOW); // convert to signed
  Center = umin32(FilteredDistances[1],OVERFLOW); // convert to signed
  Right  = umin32(FilteredDistances[2],OVERFLOW); // convert to signed
  if(Bump_Read()){
    Distance = 0; // crash
    RightBuddy = 0;
    LeftBuddy = 0;
  }else{
    Distance = min32(Left,Center,Right); // closest object
    if(Right < Left){
      LeftBuddy = 0;
      if(Right < OVERFLOW){
        if(Center < OVERFLOW){
          RightBuddy  = Center-Right;
        }else{
          RightBuddy = TURN;
        }
      }else{
          RightBuddy = 0;
      }
    }else{
      RightBuddy = 0;
      if(Left < OVERFLOW){
        if(Center < OVERFLOW){
          LeftBuddy = Center-Left;
        }else{
          LeftBuddy = TURN;
        }
      }else{
        LeftBuddy = 0;
      }
    }
  }
}
// Stage 2) Determine Fuzzy Input Membership Set
#define TOOCLOSE 100  // minimum distance in cm
#define DESIRED 250   // desired distance in cm
#define TOOFAR 400    // max distance in cm
#define TOLERANCE 25  // +/- error is acceptable
#define MAXRANGE1 500  // out of range distance in cm, it will not capture object further than this
#define MAXRANGE2 600  // out of range distance in cm, it will not capture object further than this
fuz_t BuddyTooClose, OK, BuddyTooFar, NoBuddy; // distance sets
fuz_t BuddyToTheLeft,BuddyToTheRight;   // turn sets
#define fs 100   // controller speed

void Fuzzification(void){
    //output is integers between 0 and 255, they're used in FuzzyLogic
  BuddyTooClose = MinFuzzification(Distance,TOOCLOSE,DESIRED-TOLERANCE);
  OK = CenterFuzzification(Distance,TOOCLOSE,DESIRED,TOOFAR);
  BuddyTooFar = MaxFuzzification(Distance,DESIRED+TOLERANCE,TOOFAR);
  NoBuddy = MaxFuzzification(Distance,MAXRANGE1,MAXRANGE2);
  BuddyToTheLeft = MaxFuzzification(LeftBuddy,0,TURN);
  BuddyToTheRight = MaxFuzzification(RightBuddy,0,TURN);
}
// Stage 3) Determine Fuzzy Output Membership Set
fuz_t Forward,Hover,Backward,TurnLeft,GoStraight,TurnRight,Stop;
// Fuzzy logic converts input set to output set
void FuzzyLogic(void){
  Stop       = NoBuddy;
  Hover      = OK;
  Backward   = and(BuddyTooClose,not(NoBuddy));
  Forward    = and(BuddyTooFar,not(NoBuddy));
  TurnLeft   = and(BuddyToTheLeft,not(BuddyToTheRight));
  GoStraight = and(not(BuddyToTheLeft),not(BuddyToTheRight));
  TurnRight  = and(not(BuddyToTheLeft),BuddyToTheRight);
}
fuz_t *Display[12]={
  &BuddyTooClose, &OK, &BuddyTooFar, &NoBuddy,
  &BuddyToTheLeft,&BuddyToTheRight,
  &Forward,&Hover,&Backward,&TurnLeft,&GoStraight,&TurnRight
};

// Stage 4) Transform output membership set to crisp Outputs
int32_t Power,Direction;
int32_t UR, UL;  // PWM duty 0 to 14,998
int32_t Mode;
uint8_t Last;
#define MINDUTY 200
#define MAXDUTY 4000
#define GAIN 3000
#define STEERING 1500
void CheckDuty(void){
  if(UR < MINDUTY) UR = MINDUTY;
  if(UR > MAXDUTY) UR = MAXDUTY;
  if(UL < MINDUTY) UL = MINDUTY;
  if(UL > MAXDUTY) UL = MAXDUTY;
}
void Defuzzification(void){
  if(NoBuddy >128){
    Motor_Stop();
  }else{
      //TODO: ask how this works and why we're dividing
    Power = (GAIN*(Forward-Backward))/(Forward+Hover+Backward);
    Direction =(STEERING*(TurnLeft-TurnRight))/(TurnLeft+GoStraight+TurnRight);
    if(Power > MINDUTY){
      UR = UL = Power;
      UR = UR + Direction;
      UL = UL - Direction;
      CheckDuty();
      Motor_Forward(UL,UR);
      Mode = 2; // forward
    }else if (Power < -MINDUTY){
      UR = UL = -Power;
      UR = UR - Direction;
      UL = UL + Direction;
      CheckDuty();
      Motor_Backward(UL,UR);
      Mode = 3; // backward
    }else{
      if(Direction < -MINDUTY){
        Mode = 4; // right
        Motor_Right(-Direction,-Direction);
      }else if(Direction > MINDUTY){
        Mode = 5; // left
        Motor_Left(Direction,Direction);
      }else{
        Motor_Stop();
        Mode = 1;
      }
    }
  }
}
typedef struct {
  uint32_t on;  // blinker bits
  uint32_t off; // blinker bits
  uint32_t bit; // specifies rate
} blink_t;
const blink_t Blinks[6]={
  {BK_RGHT,BK_LEFT,0x04},                  // Mode=0, stopped, inactive
  {BK_RGHT+BK_LEFT,BK_RGHT+BK_LEFT,0x04},  // Mode=1, stopped, active
  {FR_RGHT+FR_LEFT,0,0x08},                // Mode=2, forward, active
  {BK_RGHT+BK_LEFT,0,0x10},                // Mode=3, backward, active
  {FR_RGHT+BK_RGHT,0,0x20},                // Mode=4, right, active
  {FR_LEFT+BK_LEFT,0,0x20}                 // Mode=5, left, active
};

//Called with a Timer Interrupt
void FuzzyControl(void){
uint8_t in;
static int count=0;
  in = Bump_Read();
  if(in&&(Last==0)){
    Mode = 0; // stop
    Distance = 0;
    Motor_Stop();
  }
  Last = in;
  count++;
  if(Mode){
    Measurements();     // Calculate distances to objects
    Fuzzification();    // Sets input membership sets
    FuzzyLogic();       // Sets output membership sets
    Defuzzification();  // Calculate crisp output, set power to motors
  }
  if(count&Blinks[Mode].bit){
    Blinker_Output(Blinks[Mode].on);
  }else{
    Blinker_Output(Blinks[Mode].off);
  }
}


void WaitForOperator(void){uint32_t in;
  Mode = 0;
  do{ // wait for touch
    Clock_Delay1ms(300);
    LaunchPad_Output(0); // off
    Clock_Delay1ms(300);
    LaunchPad_Output(3); // red/green
    in = Bump_Read();
  }while (in == 0);
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(150);
    LaunchPad_Output(0); // off
    Clock_Delay1ms(150);
    LaunchPad_Output(1); // red
  }
  for(uint32_t i=8;i>4;i=i-1){
    Clock_Delay1ms(50*i);
    LaunchPad_Output(2); // green
    Clock_Delay1ms(50*i);
    LaunchPad_Output(0); // off
  }
  Mode = 1;
}
uint32_t Distances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
void BuddyFollow(void){ // Buddy follow, OPT3101 uses interrupt synchronization
  uint32_t channel = 1;
  uint32_t time=0;
  Clear();
  OutString("OPT3101");
  SetCursor(0, 1); OutString("Touch bump");
  SetCursor(0, 2); OutString("  to start");
  SetCursor(0, 3); OutString("Valvano");
  SetCursor(0, 4); OutString("Buddy Follow");
  SetCursor(0, 5); OutString("Fuzzy Logic");
  WaitForOperator();
  SetCursor(0, 1);
  OutString("Left =");
  SetCursor(0, 2);
  OutString("Centr=");
  SetCursor(0, 3);
  OutString("Right=");

  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(DESIRED,FILTERSIZE);
  LPF_Init2(DESIRED,FILTERSIZE);
  LPF_Init3(DESIRED,FILTERSIZE);
  FilteredDistances[0]=FilteredDistances[1]=FilteredDistances[2]=DESIRED;
  TimerA1_Init(&FuzzyControl,500000/fs);  // 2us*5000=10ms or fs=100 Hz controller
  EnableInterrupts();
  while(1){
    if(TxChannel <= 2){ // 0,1,2 means new data
      time++;
      if((time&0x3F) == 0){
        LaunchPad_LED(1);
      }
      if((time&0x3F) == 0x20){
        LaunchPad_LED(0);
      }
      if(TxChannel==0){
        FilteredDistances[0] = LPF_Calc(Distances[0]);
      }else if(TxChannel==1){
        FilteredDistances[1] = LPF_Calc2(Distances[1]);
      }else {
        FilteredDistances[2] = LPF_Calc3(Distances[2]);
      }
      SetCursor(6, TxChannel+1);
      OutUDec(FilteredDistances[TxChannel]);
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
      SetCursor(0, 5);
      for(int i=0;i<12;i++){
        OutChar(FuzzyChar(*Display[i]));
      }
    }
    WaitForInterrupt();
    if(Mode == 0){
      WaitForOperator();
    }
  }
}

//*****************System2*************************
// Wall follow, robot racing from Jacki project
#define FAST 0
void CheckForCrash(void){  // runs at about 10 Hz
  uint8_t in;
  static int count=0;
  in = Bump_Read();
  if(in&&(Last==0)){
    Mode = 0; // stop
    Distance = 0;
    Motor_Stop();
  }
  Last = in;
  count++;
  if(count&Blinks[Mode].bit){
    Blinker_Output(Blinks[Mode].on);
  }else{
     Blinker_Output(Blinks[Mode].off);
  }
}

#define WALLMIN  250  // if below this distance, the corresponding wheel must spin faster to turn away (units of mm)
#define WALLMAX  450  // if above this distance, the corresponding wheel must spin slower to turn towards (units of mm)
#define WALLFORWARD 250          // if forward measurement below this distance, initiate slow turn (units of mm)
#define WALLTOOCLOSE 150         // if forward measurement below this distance and left and right also low, stuck in a corner so initiate reverse (units of mm)
#define WALLREVERSEAMOUNT 75    // rough, imprecise distance to back up if stuck in a corner (units of mm)
#define WALLOUTOFSIGHT 600
#define WALLCENTER ((WALLMIN + WALLMAX)/2) // middle of the distance range between 'WALLMIN' and 'WALLMAX'; used in proportional controller for zero error condition (units of mm)
#define PWMNOMINAL 3750           // duty cycle of wheels if no error (0 to 14,998)
#define PWMSWING 2250             // maximum duty cycle deviation to clamp equation; PWMNOMINAL +/- SWING must not exceed the range 0 to 14,998
#define Kp  10                    // proportional controller gain

// Private function that clamps PWM duty cycles to valid range according to constants.
void clamp(void){
  if(UR < (PWMNOMINAL-PWMSWING)) UR = PWMNOMINAL-PWMSWING; // 1,500 to 6,000
  if(UR > (PWMNOMINAL+PWMSWING)) UR = PWMNOMINAL+PWMSWING;
  if(UL < (PWMNOMINAL-PWMSWING)) UL = PWMNOMINAL-PWMSWING;   // 1,500 to 6,000
  if(UL > (PWMNOMINAL+PWMSWING)) UL = PWMNOMINAL+PWMSWING;
}

void RightWallFollow(void){  // wall follow system
  uint32_t channel = 1;
  uint32_t time=0;

  Clear();
  OutString("OPT3101");
  SetCursor(0, 1); OutString("Touch bump");
  SetCursor(0, 2); OutString("  to start");
  SetCursor(0, 3); OutString("Valvano");
  SetCursor(0, 4); OutString("Wall Follow");
  SetCursor(0, 5); OutString("Proportional");
  WaitForOperator();
  SetCursor(0, 1);
  OutString("Left =");
  SetCursor(0, 2);
  OutString("Centr=");
  SetCursor(0, 3);
  OutString("Right=");
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,FILTERSIZE);
  LPF_Init2(100,FILTERSIZE);
  LPF_Init3(100,FILTERSIZE);
  UR = UL = PWMNOMINAL;   // initialize
  Motor_Forward(UL, UR);
  EnableInterrupts();
  while(1){
    CheckForCrash();
    if(Mode == 0){
      WaitForOperator();
    }
    WaitForInterrupt();
    if(TxChannel <= 2){ // 0,1,2 means new data
      time++;
      if((time&0x3F) == 0){
        LaunchPad_LED(1);
      }
      if((time&0x3F) == 0x20){
        LaunchPad_LED(0);
      }
      if(TxChannel==0){
        FilteredDistances[0] = LPF_Calc(Distances[0]);
        Left   = umin32(FilteredDistances[0],OVERFLOW); // convert to signed
      }else if(TxChannel==1){
        FilteredDistances[1] = LPF_Calc2(Distances[1]);
        Center = umin32(FilteredDistances[1],OVERFLOW); // convert to signed
      }else {
        FilteredDistances[2] = LPF_Calc3(Distances[2]);
        Right  = umin32(FilteredDistances[2],OVERFLOW); // convert to signed
      }
      Distance = min32(Left,Center,Right); // closest object
      SetCursor(6, TxChannel+1);
      OutUDec(FilteredDistances[TxChannel]);
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
    }
    if(Mode == 3){
      if((Distance > (WALLTOOCLOSE + WALLREVERSEAMOUNT))){
        Mode = 1;
// throw away previous wheel settings and initially try going straight
// another track-specific assumption could be to initially try a slight right turn
// ultimately, this might not matter if the next pass through the control loop picks something else
        UR = UL = PWMNOMINAL;
        Motor_Forward(UL, UR);
      }
    }
    else if(Distance < WALLTOOCLOSE){
      // control logic: avoid forward wall by reversing due to no room on right or left
      Motor_Backward(PWMNOMINAL, PWMNOMINAL-1000); // back up with a slight left turn bias
      Mode = 3;
    }
    else if(Center < WALLFORWARD){
      Mode = 5; // hard turn left
      UR = PWMNOMINAL;
      UL = PWMNOMINAL/2;
      Motor_Forward(UL, UR);
    }
    else if(Right < WALLMAX){
     // control logic: stay close to the right wall
      Mode = 2;
      UR = PWMNOMINAL + (WALLCENTER - Right)*Kp;
      UL = PWMNOMINAL;
      clamp();
      Motor_Forward(UL, UR);
    }
    else if (Right < WALLOUTOFSIGHT){
    // control logic: stay close to the right wall
      Mode = 4;
      UR = PWMNOMINAL + (WALLCENTER - Right)*Kp;
      UL = PWMNOMINAL - (WALLCENTER - Right)*Kp;
      clamp();
      Motor_Forward(UL, UR);
    }
    else{  // wall too far away
      Mode = 1; //  right turn
      UR = PWMNOMINAL-1000;
      UL = PWMNOMINAL+1000;
      Motor_Forward(UL, UR);
    }
  }
}

#define FASTWALLMIN  350  // if below this distance, the corresponding wheel must spin faster to turn away (units of mm)
#define FASTWALLMAX  550  // if above this distance, the corresponding wheel must spin slower to turn towards (units of mm)
#define FASTWALLFORWARD 250          // if forward measurement below this distance, initiate slow turn (units of mm)
#define FASTWALLTOOCLOSE 150         // if forward measurement below this distance and left and right also low, stuck in a corner so initiate reverse (units of mm)
#define FASTWALLREVERSEAMOUNT 75    // rough, imprecise distance to back up if stuck in a corner (units of mm)
#define FASTWALLOUTOFSIGHT 700
#define FASTWALLCENTER ((FASTWALLMIN + FASTWALLMAX)/2) // middle of the distance range between 'WALLMIN' and 'WALLMAX'; used in proportional controller for zero error condition (units of mm)
#define FASTPWMNOMINAL 4750           // duty cycle of wheels if no error (0 to 14,998)
#define FASTPWMSWING 2250             // maximum duty cycle deviation to clamp equation; PWMNOMINAL +/- SWING must not exceed the range 0 to 14,998
void Fastclamp(void){
  if(UR < (FASTPWMNOMINAL-FASTPWMSWING)) UR = FASTPWMNOMINAL-FASTPWMSWING; // 1,500 to 6,000
  if(UR > (FASTPWMNOMINAL+FASTPWMSWING)) UR = FASTPWMNOMINAL+FASTPWMSWING;
  if(UL < (FASTPWMNOMINAL-FASTPWMSWING)) UL = FASTPWMNOMINAL-FASTPWMSWING;   // 1,500 to 6,000
  if(UL > (FASTPWMNOMINAL+FASTPWMSWING)) UL = FASTPWMNOMINAL+FASTPWMSWING;
}
void FastRightWallFollow(void){  // wall follow system
  uint32_t channel = 1;
  uint32_t time=0;

  Clear();
  OutString("OPT3101");
  SetCursor(0, 1); OutString("Touch bump");
  SetCursor(0, 2); OutString("  to start");
  SetCursor(0, 3); OutString("Fast mode");
  SetCursor(0, 4); OutString("Wall Follow");
  SetCursor(0, 5); OutString("Proportional");
  WaitForOperator();
  SetCursor(0, 1);
  OutString("Left =");
  SetCursor(0, 2);
  OutString("Centr=");
  SetCursor(0, 3);
  OutString("Right=");
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,FILTERSIZE);
  LPF_Init2(100,FILTERSIZE);
  LPF_Init3(100,FILTERSIZE);
  UR = UL = FASTPWMNOMINAL;   // initialize
  Motor_Forward(UL, UR);
  EnableInterrupts();
  while(1){
    CheckForCrash();
    if(Mode == 0){
      WaitForOperator();
    }
    WaitForInterrupt();
    if(TxChannel <= 2){ // 0,1,2 means new data
      time++;
      if((time&0x3F) == 0){
        LaunchPad_LED(1);
      }
      if((time&0x3F) == 0x20){
        LaunchPad_LED(0);
      }
      if(TxChannel==0){
        FilteredDistances[0] = LPF_Calc(Distances[0]);
        Left   = umin32(FilteredDistances[0],OVERFLOW); // convert to signed
      }else if(TxChannel==1){
        FilteredDistances[1] = LPF_Calc2(Distances[1]);
        Center = umin32(FilteredDistances[1],OVERFLOW); // convert to signed
      }else {
        FilteredDistances[2] = LPF_Calc3(Distances[2]);
        Right  = umin32(FilteredDistances[2],OVERFLOW); // convert to signed
      }
      Distance = min32(Left,Center,Right); // closest object
      SetCursor(6, TxChannel+1);
      OutUDec(FilteredDistances[TxChannel]);
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
    }
    if(Mode == 3){
      if((Distance > (FASTWALLTOOCLOSE + FASTWALLREVERSEAMOUNT))){
        Mode = 1;
// throw away previous wheel settings and initially try going straight
// another track-specific assumption could be to initially try a slight right turn
// ultimately, this might not matter if the next pass through the control loop picks something else
        UR = UL = FASTPWMNOMINAL;
        Motor_Forward(UL, UR);
      }
    }
    else if(Distance < FASTWALLTOOCLOSE){
      // control logic: avoid forward wall by reversing due to no room on right or left
      Motor_Backward(FASTPWMNOMINAL, FASTPWMNOMINAL-1000); // back up with a slight left turn bias
      Mode = 3;
    }
    else if(Center < FASTWALLFORWARD){
      Mode = 5; // hard turn left
      UR = FASTPWMNOMINAL;
      UL = FASTPWMNOMINAL/2;
      Motor_Forward(UL, UR);
    }
    else if(Right < FASTWALLMAX){
     // control logic: stay close to the right wall
      Mode = 2;
      UR = FASTPWMNOMINAL + (FASTWALLCENTER - Right)*Kp;
      UL = FASTPWMNOMINAL;
      Fastclamp();
      Motor_Forward(UL, UR);
    }
    else if (Right < FASTWALLOUTOFSIGHT){
    // control logic: stay close to the right wall
      Mode = 4;
      UR = FASTPWMNOMINAL + (FASTWALLCENTER - Right)*Kp;
      UL = FASTPWMNOMINAL - (FASTWALLCENTER - Right)*Kp;
      Fastclamp();
      Motor_Forward(UL, UR);
    }
    else{  // wall too far away
      Mode = 1; //  right turn
      UR = FASTPWMNOMINAL-1000;
      UL = FASTPWMNOMINAL+1000;
      Motor_Forward(UL, UR);
    }
  }
}
//*****************System3*************************
void Opt3101Display(void){ // interrupt implementation
  uint32_t channel = 1;
  Clear();
  OutString("OPT3101");
  SetCursor(0, 1);
  OutString("Left =");
  SetCursor(0, 2);
  OutString("Centr=");
  SetCursor(0, 3);
  OutString("Right=");
  SetCursor(0, 4);
  OutString("Interrupts");
  SetCursor(0, 5);
  OutString("Valvano");
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,32);
  LPF_Init2(100,32);
  LPF_Init3(100,32);
  EnableInterrupts();
  while(1){
    if(TxChannel <= 2){ // 0,1,2 means new data
      if(TxChannel==0){
        FilteredDistances[0] = LPF_Calc(Distances[0]);
      }else if(TxChannel==1){
        FilteredDistances[1] = LPF_Calc2(Distances[1]);
      }else {
        FilteredDistances[2] = LPF_Calc3(Distances[2]);
      }
      SetCursor(6, TxChannel+1);
      OutUDec(FilteredDistances[TxChannel]);
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
    }
    WaitForInterrupt();
  }
}
void main(void){uint8_t in;
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  Bump_Init(); // bump switches
  LaunchPad_LED(0);
  Last = Bump_Read();
  Motor_Stop();
  Blinker_Init();
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  Init();
  while(1){
    Clear();
    SetCursor(0,0); OutString("RSLK MAX");
    SetCursor(0,1); OutString("Press bumper");
    SetCursor(0,2); OutString("0: Display");
    SetCursor(0,3); OutString("1: Buddy");
    SetCursor(0,4); OutString("2: Wall");
    SetCursor(0,5); OutString("5: Fast wall");
    do{ // wait for touch
      Clock_Delay1ms(250);
      LaunchPad_Output(0); // off
      Blinker_Output(FR_RGHT+BK_LEFT);
      Clock_Delay1ms(250);
      LaunchPad_Output(3); // red/green
      Blinker_Output(BK_RGHT+FR_LEFT);
      in = Bump_Read();
    }while (in == 0);
    SetCursor(0,1); OutString("Valvano     ");
    SetCursor(0,2); OutString("Release bump");
    SetCursor(0,3); OutString("OPT3101     ");
    if(in&0x01){
      SetCursor(0,4); OutString("Simple      ");
      SetCursor(0,5); OutString(" display    ");
    }
    if(in&0x02) {
      SetCursor(0,4); OutString("Buddy       ");
      SetCursor(0,5); OutString(" follow     ");
    }
    if(in&0x04) {
      SetCursor(0,4); OutString("Right wall  ");
      SetCursor(0,5); OutString(" follow     ");
    }
    if(in&0x20) {
      SetCursor(0,4); OutString("Fast Right  ");
      SetCursor(0,5); OutString(" wall follow");
    }
   while(Bump_Read()){ // wait for release
      Clock_Delay1ms(200);
      LaunchPad_Output(0); // off
      Blinker_Output(0);
      Clock_Delay1ms(200);
      LaunchPad_Output(1); // red
      Blinker_Output(FR_RGHT+FR_LEFT);
    }
    if(in&0x01) Opt3101Display();
    if(in&0x02) BuddyFollow();
    if(in&0x04) RightWallFollow();
    if(in&0x20) FastRightWallFollow();
  }
}

