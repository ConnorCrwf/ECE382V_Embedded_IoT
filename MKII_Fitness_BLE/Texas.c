// *****************Texas.c**************
// grading engine for Lab 6
//
// Runs on MSP432
// Uses the general board support package on the MSP432.
//
// Daniel and Jonathan Valvano
// November 20, 2016

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
   ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2016

 Copyright 2016 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

//  J1   J3               J4   J2
// [ 1] [21]             [40] [20]
// [ 2] [22]             [39] [19]
// [ 3] [23]             [38] [18]
// [ 4] [24]             [37] [17]
// [ 5] [25]             [36] [16]
// [ 6] [26]             [35] [15]
// [ 7] [27]             [34] [14]
// [ 8] [28]             [33] [13]
// [ 9] [29]             [32] [12]
// [10] [30]             [31] [11]

// +3.3V connected to J1.1 (power)
// joystick horizontal (X) connected to J1.2 (analog)
// UART from BoosterPack to LaunchPad connected to J1.3 (UART)
// UART from LaunchPad to BoosterPack connected to J1.4 (UART)
// joystick Select button connected to J1.5 (digital)
// microphone connected to J1.6 (analog)
// LCD SPI clock connected to J1.7 (SPI)
// ambient light (OPT3001) interrupt connected to J1.8 (digital)
// ambient light (OPT3001) and temperature sensor (TMP006) I2C SCL connected to J1.9 (I2C)
// ambient light (OPT3001) and temperature sensor (TMP006) I2C SDA connected to J1.10 (I2C)

// temperature sensor (TMP006) interrupt connected to J2.11 (digital)
// nothing connected to J2.12 (SPI CS_Other)
// LCD SPI CS connected to J2.13 (SPI)
// nothing connected to J2.14 (SPI MISO)
// LCD SPI data connected to J2.15 (SPI)
// nothing connected to J2.16 (reset)
// LCD !RST connected to J2.17 (digital)
// nothing connected to J2.18 (SPI CS_Wireless)
// servo PWM connected to J2.19 (PWM)
// GND connected to J2.20 (ground)

// +5V connected to J3.21 (power)
// GND connected to J3.22 (ground)
// accelerometer X connected to J3.23 (analog)
// accelerometer Y connected to J3.24 (analog)
// accelerometer Z connected to J3.25 (analog)
// joystick vertical (Y) connected to J3.26 (analog)
// nothing connected to J3.27 (I2S WS)
// nothing connected to J3.28 (I2S SCLK)
// nothing connected to J3.29 (I2S SDout)
// nothing connected to J3.30 (I2S SDin)

// LCD RS connected to J4.31 (digital)
// user Button2 (bottom) connected to J4.32 (digital)
// user Button1 (top) connected to J4.33 (digital)
// gator hole switch connected to J4.34 (digital)
// nothing connected to J4.35
// nothing connected to J4.36
// RGB LED blue connected to J4.37 (PWM)
// RGB LED green connected to J4.38 (PWM)
// RGB LED red (jumper up) or LCD backlight (jumper down) connected to J4.39 (PWM)
// buzzer connected to J4.40 (PWM)

#include <stdint.h>
#include "TExaS.h"
#include "../inc/BSP.h"
#include "../inc/UART0.h"
#include "../inc/CortexM.h"
#include "msp.h"
#include "AP_Lab6.h"



void static SetGrade(unsigned long lab, unsigned long grade, unsigned long id);
uint16_t edXNum; // 0x1000 to 0x9999 (in hex)

// standard ASCII symbols
#define CR   0x0D
#define LF   0x0A
#define BS   0x08
#define ESC  0x1B
#define SP   0x20
#define DEL  0x7F
char volatile LogicData;
uint32_t Id;
static uint32_t grade=0;
char MagicCode[10];
char EncriptedMagicCode[10];
uint32_t static Random; 

//---------------- execution profile of tasks  ----------------
#define TOLERANCE 50     // tasks 2,5 have 5.0%
#define JITTER_ERROR0 6  // task 0 must be less than or equal to 6us of jitter
#define JITTER_ERROR1 18 // task 1 must be less than or equal to 18us of jitter
#define PROFILESIZE0 100
#define PROFILESIZE1 100
#define PROFILESIZE2 100
#define PROFILESIZE3 100
#define PROFILESIZE4 10
#define PROFILESIZE5 10
#define PROFILESIZE6 10
uint32_t TimeBuffer0[PROFILESIZE0];  // time started Task0
uint32_t Index0;
uint32_t TimeBuffer1[PROFILESIZE1];  // time started Task1
uint32_t Index1;
uint32_t TimeBuffer2[PROFILESIZE2];  // time started Task2
uint32_t Index2;
uint32_t TimeBuffer3[PROFILESIZE3];  // time started Task3
uint32_t Index3;
uint32_t TimeBuffer4[PROFILESIZE4];  // time started Task4
uint32_t Index4;
uint32_t TimeBuffer5[PROFILESIZE5];  // time started Task5
uint32_t Index5;
uint32_t TimeBuffer6[PROFILESIZE6];  // time started Task6
uint32_t Index6;
#define EXPECTED0 1000
#define EXPECTED1 100000
#define EXPECTED2 100000
#define EXPECTED3 10000
#define MAX_AVE3  50000
#define EXPECTED4 1000000
#define MAX_AVE4  1200000
#define EXPECTED5 1000000
#define EXPECTED6 800000
#define MAX_AVE6  960000
void LogicAnalyzer(void){ // called 10k/sec
  UCA0TXBUF = LogicData;  // send data to PC
}

//**********TExaS_Grade*****
// called by Bluetooth when edXNum is written
// Inputs:  none
// Outputs: none
void TExaS_Grade(void){uint32_t thegrade;
  // called on send characteristic
	Id = ((edXNum&0xF000)>>12)*1000; // 1000's
	Id += ((edXNum&0x0F00)>>8)*100;  // 100's
	Id += ((edXNum&0x00F0)>>4)*10;   // 10's
	Id += edXNum&0x000F;             // 1's
  thegrade = grade;
  if(thegrade>=45) thegrade+=45;
  UART0_OutString("\n\rId= "); UART0_OutUDec(Id);
  UART0_OutString("\n\rGrade= "); UART0_OutUDec(thegrade);
  SetGrade(6,thegrade,Id);    
  UART0_OutString("\n\redX code= "); UART0_OutString(EncriptedMagicCode);
}


// ------------PeriodicTask2_Init------------
// Activate an interrupt to run a user task periodically.
// Give it a priority 0 to 6 with lower numbers
// signifying higher priority.  Equal priority is
// handled sequentially.
// Input:  task is a pointer to a user function
//         freq is number of interrupts per second
//           1 Hz to 10 kHz
//         priority is a number 0 to 6
// Output: none
void (*PeriodicTask2)(void);   // user function
void PeriodicTask2_Init(void(*task)(void), uint32_t freq, uint8_t priority){//long sr;
  if((freq == 0) || (freq > 10000)){
    return;                        // invalid input
  }
  if(priority > 6){
    priority = 6;
  }
//  sr = StartCritical();
  PeriodicTask2 = task;            // user function
  // ***************** TimerA3 initialization *****************
  TIMER_A3->CTL &= ~0x0030;               // halt Timer A3
  // bits15-10=XXXXXX, reserved
  // bits9-8=01,       clock source to ACLK (32,768 Hz REFOCLK after BSP_Clock_InitFastest() called)
  // bits7-6=00,       input clock divider /1
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           no interrupt on timer
  // bit0=0,           clear interrupt pending
  TIMER_A3->CTL = 0x0100;
  TIMER_A3->EX0 &= ~0x0007;               // configure for input clock divider /1
  // bits15-14=00,     no capture mode
  // bits13-12=XX,     capture/compare input select
  // bit11=X,          synchronize capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=0,           compare mode
  // bits7-5=XXX,      output mode
  // bit4=1,           enable capture/compare interrupt on CCIFG
  // bit3=X,           read capture/compare input from here
  // bit2=0,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending
  TIMER_A3->CCTL[0] = 0x0010;
  TIMER_A3->CCR[0] = (32768/freq - 1);      // compare match value
// interrupts enabled in the main program after all devices initialized
  NVIC->IP[3] = (NVIC->IP[3]&0xFF00FFFF)|(priority<<21); // priority
  NVIC->ISER[0] = 0x00004000;         // enable interrupt 14 in NVIC
  TIMER_A3->CTL |= 0x0014;                // reset and start Timer A3 in up mode

 // EndCritical(sr);
}

void TA3_0_IRQHandler(void){
  TA3CCTL0 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  (*PeriodicTask2)();              // execute user task
}

// ------------PeriodicTask2_Stop------------
// Deactivate the interrupt running a user task
// periodically.
// Input: none
// Output: none
void PeriodicTask2_Stop(void){
  TIMER_A3->CCTL[0] &= ~0x0001;             // acknowledge capture/compare interrupt 0
  NVIC->ICER[0] = 0x00004000;         // disable interrupt 14 in NVIC
}

// ************TExaS_Init*****************
// Initialize grader, triggered by periodic timer
// This needs to be called once
// Inputs: Grading or Logic analyzer
// Outputs: none
static uint32_t Lab6GradeGetStatusMsg(void);
static uint32_t Lab6GradeRegisterServiceMsg(void); 
static uint32_t Lab6GradeGetVersionMsg(void);
static uint32_t Lab6GradeAddServiceMsg(void);
static uint32_t Lab6GradeAddCharValueMsg(void);
static uint32_t Lab6GradeAddCharDescriptorMsg(void);
static uint32_t Lab6GradeAddNotifyCharDescriptorMsg(void);
static uint32_t Lab6GradeSetDeviceNameMsg(void);
static uint32_t Lab6GradeSetAdvertisementData1Msg(void);
static uint32_t Lab6GradeSetAdvertisementDataMsg(void);
static uint32_t Lab6GradeStartAdvertisementMsg(void);
void TExaS_Init(enum TExaSmode mode){uint16_t x,y; uint8_t sel;
  // 32 bit free running timer
  // 10 kHz periodic interrupt
  // edge triggered interrupt on Profile bits
  // grade mode will collect 10 seconds of profile (4 arrays)
  // logic analyzer will 10 kHz output to serial port (pack 4 bits): 8 bit bit7 set then LA, but 7 ASCII
 // Id = edXcode;
  BSP_Time_Init();  // system timer in usec
  BSP_Joystick_Init();
  BSP_Joystick_Input(&x,&y,&sel);
  Random = x*y+sel; // seed
  BSP_Joystick_Input(&x,&y,&sel);
  Random += x*y;    // seed
  
  Index0 = Index1 = Index2 = Index3 = Index4 = Index5 = Index6 =0;
  LogicData |= 0x80; // bit 7 means logic data
  if(mode == LOGICANALYZER){
  // enable 10k periodic interrupt if logic analyzer mode
    PeriodicTask2_Init(&LogicAnalyzer,10000,5); // run grader
  } else if(mode == GRADER){
    if(UCA0CTLW0 != 0x00C0){
      UART0_Init(); // if not on, enable
    }
    UART0_OutString("\n\r**Start Lab 6 Grader**MSP432 Version 1.01** ");
    
    grade =  Lab6GradeGetStatusMsg();               // 5
    grade += Lab6GradeGetVersionMsg();              // 5
    grade += Lab6GradeRegisterServiceMsg();         // 5
    grade += Lab6GradeAddServiceMsg();              // 5
    grade += Lab6GradeAddCharValueMsg();            // 5
    grade += Lab6GradeAddCharDescriptorMsg();       // 5
    grade += Lab6GradeAddNotifyCharDescriptorMsg(); // 5
    grade += Lab6GradeSetDeviceNameMsg();           // 5
    grade += Lab6GradeSetAdvertisementData1Msg();   // 5
    grade += Lab6GradeSetAdvertisementDataMsg();    // 5
    grade += Lab6GradeStartAdvertisementMsg();      // 5
  }
}

// ************TExaS_Stop*****************
// Stop the transfer
// Inputs:  none
// Outputs: none
void TExaS_Stop(void){
  PeriodicTask2_Stop();
}


// record time Task 0 is started
void TExaS_Task0(void){
  if(Index0<PROFILESIZE0){
    TimeBuffer0[Index0] = BSP_Time_Get(); //usec
  }
  Index0++;
  LogicData ^= 0x01;
}
// record time Task 1 is started
void TExaS_Task1(void){
  if(Index1<PROFILESIZE1){
    TimeBuffer1[Index1] = BSP_Time_Get();
  }
  Index1++;
  LogicData ^= 0x02;
}
// record time Task 2 is started
void TExaS_Task2(void){
  if(Index2<PROFILESIZE2){
    TimeBuffer2[Index2] = BSP_Time_Get();
  }
  Index2++;
  LogicData ^= 0x04;
}
// record time Task 3 is started
void TExaS_Task3(void){
  if(Index3<PROFILESIZE3){
    TimeBuffer3[Index3] = BSP_Time_Get();
  }
  Index3++;
  LogicData ^= 0x08;
}
// record time Task 4 is started
void TExaS_Task4(void){
  if(Index4<PROFILESIZE4){
    TimeBuffer4[Index4] = BSP_Time_Get();
  }
  Index4++;
  LogicData ^= 0x10;
}
// record time Task 5 is started
void TExaS_Task5(void){
  if(Index5<PROFILESIZE5){
    TimeBuffer5[Index5] = BSP_Time_Get();
  }
  Index5++;
  LogicData ^= 0x20;
}
// record time Task 6 is started
void TExaS_Task6(void){
  if(Index6<PROFILESIZE6){
    TimeBuffer6[Index6] = BSP_Time_Get();
  }
  Index6++;
  LogicData ^= 0x40;
}
//#define TExaS_Task0()  if(Index0<PROFILESIZE0){TimeBuffer0[Index0++] = BSP_Time_Get();}
//#define TExaS_Task1()  if(Index1<PROFILESIZE1){TimeBuffer1[Index1++] = BSP_Time_Get();}
//#define TExaS_Task2()  if(Index2<PROFILESIZE2){TimeBuffer2[Index2++] = BSP_Time_Get();}
//#define TExaS_Task3()  if(Index3<PROFILESIZE3){TimeBuffer3[Index3++] = BSP_Time_Get();}
//#define TExaS_Task4()  if(Index4<PROFILESIZE4){TimeBuffer4[Index4++] = BSP_Time_Get();}
//#define TExaS_Task5()  if(Index5<PROFILESIZE5){TimeBuffer5[Index5++] = BSP_Time_Get();}
//#define TExaS_Task6()  if(Index6<PROFILESIZE6){TimeBuffer6[Index6++] = BSP_Time_Get();}



void static SetGrade(unsigned long lab, unsigned long grade, unsigned long id){

}

// **************Lab6 graders and solution******************
#define SOF  254
uint32_t AP_GetSize(uint8_t *pt);
uint8_t static TrueMsgBuffer[32];
uint8_t static TestMsgBuffer[32];
uint32_t static NextRandom(void){
  Random = 1664525*Random+1013904223;
  return Random>>24;
}
int Lab6OutCheck(uint8_t mine[], uint8_t yours[]){int i,j,size;
  size = AP_GetSize(mine)+6;
  for(j=0;j<size;j++){
    if(mine[j] != yours[j]){
      UART0_OutString("\n\r  True msg:");
      for(i=0;i<size-1;i++){
        UART0_OutUHex2(mine[i]);
        UART0_OutChar(',');
      } UART0_OutUHex2(mine[i]);
       UART0_OutString("\n\r  Your msg:");
      for(i=0;i<size-1;i++){
        UART0_OutUHex2(yours[i]);
        UART0_OutChar(',');
      } UART0_OutUHex2(yours[i]);
      return 1; // fail
    }
  }
  return 0; // success
}
// **********SetFCSSolution**************
// helper function, add check byte to message
// assumes every byte in the message has been set except the FCS
// used the length field, assumes less than 256 bytes
// FCS = 8-bit EOR(all bytes except SOF and the FCS itself)
// Inputs: pointer to message
//         stores the FCS into message itself
// Outputs: none
void static SetFCSSolution(uint8_t *msg){
  uint8_t fcs; uint32_t size; int i;
  size = AP_GetSize(msg);
  fcs=0;
  for(i=1;i<size+5;i++){
    fcs=fcs^msg[i]; // payload
  }
  msg[i] = fcs;
}
//*************BuildGetStatusMsgSolution**************
// Create a Get Status message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will Get Status
void static Lab6BuildGetStatusMsgSolution(uint8_t *msg){
// hint: see NPI_GetStatus in AP.c
  msg[0] = SOF;
  msg[1] = 0;    // length = 0
  msg[2] = 0;    
  msg[3] = 0x55;
  msg[4] = 0x06; // SNP Get Status
  SetFCSSolution(msg); 
}
void BuildGetStatusMsg(uint8_t *msg);
static uint32_t Lab6GradeGetStatusMsg(void){ 
  UART0_OutString("\n\rBuildGetStatusMsg ");
  Lab6BuildGetStatusMsgSolution(TrueMsgBuffer);
  BuildGetStatusMsg(TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  UART0_OutString("ok");
  return 5;   // ok
}

//*************BuildGetVersionMsgSolution**************
// Create a Get Version message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will Get Status
void static Lab6BuildGetVersionMsgSolution(uint8_t *msg){
// hint: see NPI_GetVersion in AP.c
  msg[0] = SOF;
  msg[1] = 0;    // length = 0
  msg[2] = 0;    
  msg[3] = 0x35;
  msg[4] = 0x03; // SNP Get Version
  SetFCSSolution(msg); 
}
void BuildGetVersionMsg(uint8_t *msg);
static uint32_t Lab6GradeGetVersionMsg(void){ 
  UART0_OutString("\n\rBuildGetVersionMsg ");
  Lab6BuildGetVersionMsgSolution(TrueMsgBuffer);
  BuildGetVersionMsg(TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  UART0_OutString("ok");
  return 5;   // ok
}


//*************BuildAddServiceMsgSolution**************
// Create an Add service message, used in Lab 6
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        pointer to empty buffer of at least 9 bytes
// Output none
// build the necessary NPI message that will add a service
void static Lab6BuildAddServiceMsgSolution(uint16_t uuid, uint8_t *msg){
  msg[0] = SOF;
  msg[1] = 3;  // length = 3
  msg[2] = 0;    
  msg[3] = 0x35;
  msg[4] = 0x81;      // SNP Add Service
  msg[5] = 0x01;      // Primary Service
  msg[6] = uuid&0xFF;
  msg[7] = uuid>>8;
  SetFCSSolution(msg); 
}
void BuildAddServiceMsg(uint16_t uuid, uint8_t *msg);
static uint32_t Lab6GradeAddServiceMsg(void){int i; uint32_t uuid;
  UART0_OutString("\n\rBuildAddServiceMsg ");
  uuid = 0xFF00;
  Lab6BuildAddServiceMsgSolution(uuid,TrueMsgBuffer);
  BuildAddServiceMsg(uuid,TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  for(i=0;i<4;i++){
    uuid = (NextRandom()<<8)+NextRandom();
    Lab6BuildAddServiceMsgSolution(uuid,TrueMsgBuffer);
    BuildAddServiceMsg(uuid,TestMsgBuffer);
    if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
      return 0; // fail
    }
  }
  UART0_OutString("ok");
  return 5;
}

//*************AP_BuildRegisterServiceMsgSolution**************
// Create a register service message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will register a service
void static Lab6BuildRegisterServiceMsgSolution(uint8_t *msg){
  msg[0] = SOF;
  msg[1] = 0;     // length = 0
  msg[2] = 0;    
  msg[3] = 0x35;
  msg[4] = 0x84;  // SNP Register Service
  SetFCSSolution(msg); 
}
void BuildRegisterServiceMsg(uint8_t *msg);
static uint32_t Lab6GradeRegisterServiceMsg(void){ 
  UART0_OutString("\n\rBuildRegisterServiceMsg ");
  Lab6BuildRegisterServiceMsgSolution(TrueMsgBuffer);
  BuildRegisterServiceMsg(TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  UART0_OutString("ok");
  return 5;   // ok
}

//*************BuildAddCharValueMsgSolution**************
// Create a Add Characteristic Value Declaration message, used in Lab 6
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        permission is GATT Permission, 0=none,1=read,2=write, 3=Read+write 
//        properties is GATT Properties, 2=read,8=write,0x0A=read+write
//        pointer to empty buffer of at least 14 bytes
// Output none
// build the necessary NPI message that will add a characteristic value
void static Lab6BuildAddCharValueMsgSolution(uint16_t uuid,  
  uint8_t permission, uint8_t properties, uint8_t *msg){
// set RFU to 0 and
// set the maximum length of the attribute value=512
// for a hint see NPI_AddCharValue in AP.c
// for a hint see first half of AP_AddCharacteristic and first half of AP_AddNotifyCharacteristic
  msg[0] = SOF;
  msg[1] = 8;          // length = 8
  msg[2] = 0;    
  msg[3] = 0x35;
  msg[4] = 0x82;       // SNP Add Characteristic Value Declaration
  msg[5] = permission; // 0=none,1=read,2=write, 3=Read+write, GATT Permission
  msg[6] = properties; // 2=read,8=write,0x0A=read+write,0x10=notify, GATT Properties
  msg[7] = 0;      
  msg[8] = 0;          // RFU
  msg[9] = 0;
  msg[10] = 2;         // Maximum length of the attribute value=512
  msg[11] = uuid&0xFF;
  msg[12] = uuid>>8;
  SetFCSSolution(msg); 
}
void BuildAddCharValueMsg(uint16_t uuid,  
  uint8_t permission, uint8_t properties, uint8_t *msg);
static uint32_t Lab6GradeAddCharValueMsg(void){int i; 
  uint32_t uuid; uint8_t perm,prop;
  UART0_OutString("\n\rBuildAddCharValueMsg ");
  for(i=0;i<4;i++){
    uuid = (NextRandom()<<8)+NextRandom();
    perm = i; // 0,1,2,3
    if(i==0){
      prop = 0x10; // notify
    }else{
      prop = ((perm&0x02)<<2)+((perm&0x01)<<8); // 0x10,2,8,0x0A
    }
    Lab6BuildAddCharValueMsgSolution(uuid,perm,prop,TrueMsgBuffer);
    BuildAddCharValueMsg(uuid,perm,prop,TestMsgBuffer);
    if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
      return 0; // fail
    }
  }
  UART0_OutString("ok");
  return 5;
}

//*************BuildAddCharDescriptorMsgSolution**************
// Create a Add Characteristic Descriptor Declaration message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 20 bytes
//        pointer to empty buffer of at least 14 bytes
// Output none
// build the necessary NPI message that will add a Descriptor Declaration
void static Lab6BuildAddCharDescriptorMsgSolution(char name[], uint8_t *msg){
// set length and maxlength to the string length
// set the permissions on the string to read
// for a hint see NPI_AddCharDescriptor in AP.c
// for a hint see second half of AP_AddCharacteristic
  int i;
  msg[0] = SOF;
  i=0;
  while((i<20)&&(name[i])){
    msg[11+i] = name[i]; i++;
  }
  msg[11+i] = 0; i++;
  msg[1] = 6+i;  // frame length
  msg[2] = 0;
  msg[3] = 0x35; // SNP Add Characteristic Descriptor Declaration
  msg[4] = 0x83;
  msg[5] = 0x80; // User Description String
  msg[6] = 0x01; // GATT Read Permissions
  msg[7] = msg[9] = i;  // string length
  msg[8] = msg[10] = 0; // string length
  SetFCSSolution(msg); 
}
void BuildAddCharDescriptorMsg(char name[], uint8_t *msg);
static uint32_t Lab6GradeAddCharDescriptorMsg(void){ 
  UART0_OutString("\n\rBuildAddCharDescriptorMsg ");
  Lab6BuildAddCharDescriptorMsgSolution("data",TrueMsgBuffer);
  BuildAddCharDescriptorMsg("data",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  Lab6BuildAddCharDescriptorMsgSolution("key",TrueMsgBuffer);
  BuildAddCharDescriptorMsg("key",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  UART0_OutString("ok");
  return 5;   // ok
}


//*************BuildAddNotifyCharDescriptorMsgSolution**************
// Create a Add Notify Characteristic Descriptor Declaration message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 20 bytes
//        pointer to empty buffer of at least 14 bytes
// Output none
// build the necessary NPI message that will add a Descriptor Declaration
void static Lab6BuildAddNotifyCharDescriptorMsgSolution(char name[], uint8_t *msg){
// set length and maxlength to the string length
// set the permissions on the string to read
// set User Description String
// set CCCD parameters read+write
// for a hint see NPI_AddCharDescriptor4 in VerySimpleApplicationProcessor.c
// for a hint see second half of AP_AddNotifyCharacteristic
  int i;
  msg[0] = SOF;
  i=0;
  while((i<20)&&(name[i])){
    msg[12+i] = name[i]; i++;
  }
  msg[12+i] = 0; i++;
  msg[1] = 7+i;  // frame length
  msg[2] = 0;
  msg[3] = 0x35; // SNP Add Characteristic Descriptor Declaration
  msg[4] = 0x83;
  msg[5] = 0x84; // User Description String, and CCCD permissions
  msg[6] = 0x03;      // CCCD parameters read+write
  msg[7] = 0x01;      // GATT Read Permissions
  msg[8] = msg[10] = i; // string length
  msg[9] = msg[11] = 0; // string length
  SetFCSSolution(msg); 
}
void BuildAddNotifyCharDescriptorMsg(char name[], uint8_t *msg);
static uint32_t Lab6GradeAddNotifyCharDescriptorMsg(void){ 
  UART0_OutString("\n\rBuildAddNotifyCharDescriptorMsg ");
  Lab6BuildAddNotifyCharDescriptorMsgSolution("data",TrueMsgBuffer);
  BuildAddNotifyCharDescriptorMsg("data",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  Lab6BuildAddNotifyCharDescriptorMsgSolution("key",TrueMsgBuffer);
  BuildAddNotifyCharDescriptorMsg("key",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  UART0_OutString("ok");
  return 5;   // ok
}  

//*************Lab6BuildSetDeviceNameMsgSolution**************
// Create a Set GATT Parameter message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 24 bytes
//        pointer to empty buffer of at least 36 bytes
// Output none
// build the necessary NPI message to set Device name
void static Lab6BuildSetDeviceNameMsgSolution(char name[], uint8_t *msg){
// for a hint see NPI_GATTSetDeviceNameMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_GATTSetDeviceName in AP.c
  int i;
  msg[0] = SOF;
  i=0;
  while((i<24)&&(name[i])){
    msg[8+i] = name[i]; i++;
  }
  msg[1] = 3+i;  // frame length
  msg[2] = 0;
  msg[3] = 0x35; // SNP Set GATT Parameter
  msg[4] = 0x8C;
  msg[5] = 0x01; // Generic Access Service
  msg[6] = 0x00; // Device Name
  msg[7] = 0;
  SetFCSSolution(msg); 
}
void BuildSetDeviceNameMsg(char name[], uint8_t *msg);
static uint32_t Lab6GradeSetDeviceNameMsg(void){ 
  UART0_OutString("\n\rBuildSetDeviceNameMsg ");
  Lab6BuildSetDeviceNameMsgSolution("Yerraballi",TrueMsgBuffer);
  BuildSetDeviceNameMsg("Yerraballi",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  Lab6BuildSetDeviceNameMsgSolution("Valvano",TrueMsgBuffer);
  BuildSetDeviceNameMsg("Valvano",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  UART0_OutString("ok");
  return 5;   // ok
}  
//*************BuildSetAdvertisementData1MsgSolution**************
// Create a Set Advertisement Data message, used in Lab 6
// Inputs pointer to empty buffer of at least 16 bytes
// Output none
// build the necessary NPI message for Non-connectable Advertisement Data
void static Lab6BuildSetAdvertisementData1MsgSolution(uint8_t *msg){
// for a hint see NPI_SetAdvertisementMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_SetAdvertisement1 in AP.c
// Non-connectable Advertisement Data
// GAP_ADTYPE_FLAGS,DISCOVERABLE | no BREDR  
// Texas Instruments Company ID 0x000D
// TI_ST_DEVICE_ID = 3
// TI_ST_KEY_DATA_ID
// Key state=0
  msg[0] = SOF;
  msg[1] = 11;    // frame length
  msg[2] = 0;
  msg[3] = 0x55;
  msg[4] = 0x43;  // SNP Set Advertisement Data
  msg[5] = 0x01;  // Not connected Advertisement Data
  msg[6] = 0x02;
  msg[7] = 0x01;
  msg[8] = 0x06;  // GAP_ADTYPE_FLAGS,DISCOVERABLE | no BREDR
  msg[9] = 0x06;
  msg[10] = 0xFF; // length, manufacturer specific
  msg[11] = 0x0D;
  msg[12] = 0x00; // Texas Instruments Company ID
  msg[13] = 0x03; // TI_ST_DEVICE_ID
  msg[14] = 0x00; // TI_ST_KEY_DATA_ID
  msg[15] = 0x00; // Key state
  SetFCSSolution(msg); 
}
void BuildSetAdvertisementData1Msg(uint8_t *msg);
static uint32_t Lab6GradeSetAdvertisementData1Msg(void){ 
  UART0_OutString("\n\rBuildSetAdvertisementData1Msg ");
  Lab6BuildSetAdvertisementData1MsgSolution(TrueMsgBuffer);
  BuildSetAdvertisementData1Msg(TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  UART0_OutString("ok");
  return 5;   // ok
}

//*************Lab6BuildSetAdvertisementDataMsgSolutionold**************
// Create a Set Advertisement Data message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 24 bytes
//        pointer to empty buffer of at least 36 bytes
// Output none
// build the necessary NPI message for Scan Response Data
void static Lab6BuildSetAdvertisementDataMsgSolutionold(char name[], uint8_t *msg){
// for a hint see NPI_SetAdvertisementDataMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_SetAdvertisementData in AP.c
  int i;
  msg[0] = SOF;
  i=0;
  while((i<24)&&(name[i])){
    msg[8+i] = name[i]; i++;
  }
  msg[1] = 11+i;  // frame length
  msg[2] = 0;
  msg[3] = 0x55;  // Set Advertisement Data
  msg[4] = 0x43;
  msg[5] = 0x00;  // Scan Response Data
  msg[6] = i;     // length
  msg[7] = 0x09;  // type=LOCAL_NAME_COMPLETE
// connection interval range
  msg[8+i] = 0x05;   // length of this data
  msg[9+i] = 0x12;   // GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE
  msg[10+i] = 0x50;
  msg[11+i] = 0x00;  // DEFAULT_DESIRED_MIN_CONN_INTERVAL
  msg[12+i] = 0x20;
  msg[13+i] = 0x03;  // DEFAULT_DESIRED_MAX_CONN_INTERVAL
// Tx power level
  msg[14+i] = 0x02;  // length of this data
  msg[15+i] = 0x0A;  // GAP_ADTYPE_POWER_LEVEL
  msg[16+i] = 0x00;  // 0dBm  
  SetFCSSolution(msg); 
}
void BuildSetAdvertisementDataMsg(char name[], uint8_t *msg);

static uint32_t Lab6GradeSetAdvertisementDataMsgold(void){ 
  UART0_OutString("\n\rThere was a bug in the original grader, checking the bugged solution\n\rBuildSetAdvertisementDataMsg ");
  Lab6BuildSetAdvertisementDataMsgSolutionold("Yerraballi",TrueMsgBuffer);
  BuildSetAdvertisementDataMsg("Yerraballi",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  Lab6BuildSetAdvertisementDataMsgSolutionold("Valvano",TrueMsgBuffer);
  BuildSetAdvertisementDataMsg("Valvano",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return 0; // fail
  }
  UART0_OutString("ok, you get credit, but your solution has a bug");
  return 5;   // ok
} 
//*************Lab6BuildSetAdvertisementDataMsgSolution**************
// Create a Set Advertisement Data message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 24 bytes
//        pointer to empty buffer of at least 36 bytes
// Output none
// build the necessary NPI message for Scan Response Data
void static Lab6BuildSetAdvertisementDataMsgSolution(char name[], uint8_t *msg){
// for a hint see NPI_SetAdvertisementDataMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_SetAdvertisementData in AP.c
  int i;
  msg[0] = SOF;
  i=0;
  while((i<24)&&(name[i])){
    msg[8+i] = name[i]; i++;
  } 
  i++;
  msg[1] = 11+i;  // frame length
  msg[2] = 0;
  msg[3] = 0x55;  // Set Advertisement Data
  msg[4] = 0x43;
  msg[5] = 0x00;  // Scan Response Data
  msg[6] = i;     // length
  msg[7] = 0x09;  // type=LOCAL_NAME_COMPLETE
// connection interval range
  msg[7+i] = 0x05;   // length of this data
  msg[8+i] = 0x12;   // GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE
  msg[9+i] = 0x50;
  msg[10+i] = 0x00;  // DEFAULT_DESIRED_MIN_CONN_INTERVAL
  msg[11+i] = 0x20;
  msg[12+i] = 0x03;  // DEFAULT_DESIRED_MAX_CONN_INTERVAL
// Tx power level
  msg[13+i] = 0x02;  // length of this data
  msg[14+i] = 0x0A;  // GAP_ADTYPE_POWER_LEVEL
  msg[15+i] = 0x00;  // 0dBm  
  SetFCSSolution(msg); 
}

static uint32_t Lab6GradeSetAdvertisementDataMsg(void){ 
  UART0_OutString("\n\rBuildSetAdvertisementDataMsg ");
  Lab6BuildSetAdvertisementDataMsgSolution("Yerraballi",TrueMsgBuffer);
  BuildSetAdvertisementDataMsg("Yerraballi",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return Lab6GradeSetAdvertisementDataMsgold(); // fail
  }
  Lab6BuildSetAdvertisementDataMsgSolution("Valvano",TrueMsgBuffer);
  BuildSetAdvertisementDataMsg("Valvano",TestMsgBuffer);
  if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
    return Lab6GradeSetAdvertisementDataMsgold(); // fail
  }
  UART0_OutString("ok");
  return 5;   // ok
}  

// Lab6BuildStartAdvertisementMsgSolution
// Create a Start Advertisement Data message, used in Lab 6
// Inputs advertising interval
//        pointer to empty buffer of at least 20 bytes
// Output none
// build the necessary NPI message to start advertisement
void static Lab6BuildStartAdvertisementMsgSolution(uint16_t interval, uint8_t *msg){
// for a hint see NPI_StartAdvertisementMsg in VerySimpleApplicationProcessor.c
// for a hint see NPI_StartAdvertisement in AP.c
  msg[0] = SOF;
  msg[1] = 14;    // frame length
  msg[2] = 0;
  msg[3] = 0x55;
  msg[4] = 0x42;  // SNP Start Advertisement
  msg[5] = 0x00;  // Connectable Undirected Advertisements
  msg[6] = 0x00;  // Advertise infinitely.
  msg[7] = 0x00;  
  msg[8] = interval&0xFF;
  msg[9] = interval>>8;  // Advertising Interval (100 * 0.625 ms=62.5ms)
  msg[10] = 0x00; // Filter Policy RFU
  msg[11] = 0x00; // Initiator Address Type RFU
  msg[12] = 0x00;
  msg[13] = 0x01;
  msg[14] = 0x00;
  msg[15] = 0x00;
  msg[16] = 0x00;
  msg[17] = 0xC5; // RFU
  msg[18] = 0x02; // Advertising will restart with connectable advertising when a connection is terminated
  SetFCSSolution(msg); 
}
void BuildStartAdvertisementMsg(uint16_t interval,  uint8_t *msg);
static uint32_t Lab6GradeStartAdvertisementMsg(void){int i; 
  uint32_t interval; 
  UART0_OutString("\n\rBuildStartAdvertisementMsg ");
  for(i=0;i<4;i++){
    interval = NextRandom()+50;
    Lab6BuildStartAdvertisementMsgSolution(interval,TrueMsgBuffer);
    BuildStartAdvertisementMsg(interval,TestMsgBuffer);
    if(Lab6OutCheck(TrueMsgBuffer,TestMsgBuffer)){
      return 0; // fail
    }
  }
  UART0_OutString("ok");
  return 5;
}


