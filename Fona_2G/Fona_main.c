// Fona_main.c
// Runs on MSP432
// Test Fona SMS functions
// inputs from BP-BASSENSORSMKII
//    TMP117 temperature
//    HDC2030 temperature and humidity
//    OPT3001 light intensity
//    BMI160/BMM150 9-axis IMU
// Daniel and Jonathan Valvano
// July 26, 2021

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2021
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

//***********Serial port to PC****************
// P1.2 UCA0RXD (VCP receive) connected to
// P1.3 UCA0TXD (VCP transmit) connected to
//***********Fona****************
// Bat     to 3.7V LiIon battery, also on JST-2
// GND     to MSP432 ground and battery ground, also on JST-2
// +SPKR   to speaker+
// -SPKR   to speaker-
// Rst     to J2.18, MSP432 GPIO output, P3.0
// PS      not connected
// Key     grounded
// RI      not connected
// TX      to J1.3, MSP432 UCA2RXD P3.2
// RX      to J1.4, MSP432 UCA2TXD P3.3
// NS      not connected
// Vio     to 3.3V
//***********BP-BASSENSORSMKII****************
// P4.1 INT1      BMI160 IMU
// P5.0 INT2      BMI160 IMU
// P4.3 HDC_V+    HDC2080
// P6.1 HDC_INT   HDC2080
// P4.5 OPT_V+    OPT3001 VDD   pin 1 <- P4.5 power to sensor
// P6.4 SDA       OPT3001 SDA   pin 6 <> P6.4 I2C data SDA
// P6.5 SCL       OPT3001 SCL   pin 4 <- P6.5 I2C clock SCL
// P4.2 OPT_INT   OPT3001 INT   pin 5 -> P4.2 interrupt
// P4.7 TMP_V+    TMP117 V+    pin 5 <- P4.7 power to sensor
// P6.4 SDA       TMP117 SDA   pin 6 <> P6.4 I2C data SDA
// P6.5 SCL       TMP117 SCL   pin 1 <- P6.5 I2C clock SCL
// P4.4 TMP_ALERT TMP117 ALERT pin 3 -> P4.4 alert
#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "os.h"
#include "Texas.h"
#include "../inc/Fona.h"
#include "../inc/Clock.h"
#include "../inc/UART0.h"
#include "../inc/UART1.h"
#include "../inc/LaunchPad.h"
#include "../inc/I2CB1.h"
#include "../inc/HDC2080.h"
#include "../inc/OPT3001.h"
#include "../inc/TMP117.h"
#include "../inc/bmi160.h"
#include "../inc/bmm150.h"
#define PHONE (uint8_t *)"15129682240"
// demonstrates sending text messages with Adafruit Fona 2G
// simple echo from terminal to Fona
int mainterminal(void){
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  UART0_Initprintf(); // initialize UART and printf
  UART0_OutString("\n\rTerminal program for Adafruit Fona 2G\n\r");
  UART0_OutString("Valvano July 26, 2021\n\r");
  Fona_Init(UART1_BAUD_115200); // 115200 should work too
  UART0_OutString("\n\rInit done\n\r");
  while(1){char c;
    if(EUSCI_A0->IFG&0x01){
      c =(char)(EUSCI_A0->RXBUF);
      UART0_OutChar(c);
      UART1_OutChar(c);
    }
    c = UART1_InCharNonBlock();
    if(c){
      UART0_OutChar(c);
    }
  }
}
// demonstrates sending text messages with Adafruit Fona 2G
int main(void){ //mainHelloWorld(void){
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  LaunchPad_Init();
  UART0_Initprintf(); // initialize UART and printf
  UART0_OutString("\n\rHello world program for Adafruit Fona 2G\n\r");
  UART0_OutString("Valvano July 27, 2021\n\r");
  Fona_Init(UART1_BAUD_115200); // 9600 should work too
  Fona_SetSMSStorage(SMS_SIM);  // store SMS on SIM card
  Fona_SendSMS(PHONE,(uint8_t *)"Hello World");
  while(1){
  }
}
// demonstrates sending text messages with Adafruit Fona 2G
int mainsimple(void){
  char string[64];
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  LaunchPad_Init();
  UART0_Initprintf(); // initialize UART and printf
  UART0_OutString("\n\rSimple program for Adafruit Fona 2G\n\r");
  UART0_OutString("Valvano July 26, 2021\n\r");
  Fona_Init(UART1_BAUD_115200); // 9600 should work too
  Fona_SetSMSStorage(SMS_SIM);  // store SMS on SIM card
  UART0_OutString("\n\rType message to send text\n\r");
  while(1){
    UART0_OutString("\n\rText: ");
    UART0_InString(string,63); // user enters a string
    LaunchPad_LED(1);
    Fona_SendSMS(PHONE,(uint8_t *)string);
    LaunchPad_LED(0);
  }
}

// *************RTOS version***************
int32_t I2Cmutex;         // exclusive access to I2C
OPT3001_Result Light;     // lux = 0.01*(2^Exponent)*Result
uint32_t TimeMs;          // elapsed time in ms
uint32_t Humidity;        // units 0.1% (ranges from 0 to 1000, 123 means 12.3%
int32_t Temperature;      // units 0.1 C
int16_t RawTemperature;   // 0.0078125 C
int32_t FixedTemperature; // 0.1C
int32_t Ax,Ay,Az; // acceleration
int32_t Rx,Ry,Rz; // gyro pitch, roll, yaw
int32_t Mx,My,Mz; // magnetic field strength
// Select whether or not to check for errors
#define ERRORCHECK 1
// Choose LogicAnalyzer or Fona
//#define LOGICANALYZER 1
//---------------- Task0 real time periodic event ----------------
// Event thread run by OS in real time at 1000 Hz
// *********Task0_Init*********
// initializes TimeMs
// Task0 maintains time
// Inputs:  none
// Outputs: none
void Task0_Init(void){
  TimeMs = 0;
}
// *********Task0*********
// Periodic event thread runs in real time at 1000 Hz
// Task0 maintains time
// Periodic events cannot block, spin, suspend or sleep
// Periodic events can call OS_Signal but not OS_Wait
// Inputs:  none
// Outputs: none
// Warning: Execution time must be much less than 1ms
void Task0(void){
#ifdef LOGICANALYZER
  TExaS_Task0();     // toggle virtual logic analyzer
#endif
  TimeMs = TimeMs + 1;
}
/* ****************************************** */
/*          End of Task0 Section              */
/* ****************************************** */

//---------------- Task1 handles text messages ----------------
// Main thread scheduled by OS round robin preemptive scheduler
//------------UART0_InCharBlocking------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
char UART0_InCharBlocking(void){
  while((EUSCI_A0->IFG&0x01) == 0){
    OS_Suspend();
  }
  return((char)(EUSCI_A0->RXBUF));
}
void UART0_InStringBlocking(char *bufPt, uint16_t max) {
int length=0;
char character;
  character = UART0_InCharBlocking();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART0_OutChar(BS);
      }
    }
    else if(length < max){
      *bufPt = character;
      bufPt++;
      length++;
      UART0_OutChar(character);
    }
    character = UART0_InChar();
  }
  *bufPt = 0;
}
void UART0_OutCharBlocking(char letter){
  while((EUSCI_A0->IFG&0x02) == 0){
    OS_Suspend();
  }
  EUSCI_A0->TXBUF = letter;
}
void UART0_OutStringBlocking(char *pt){
  while(*pt){
    UART0_OutCharBlocking(*pt);
    pt++;
  }
}
void Task1(void){
    // can't have logic analyzer and output from Fona
#ifdef LOGICANALYZER
#else
  char string[64];
  UART0_OutStringBlocking("\n\rRTOS version for Adafruit Fona 2G\n\r");
  UART0_OutStringBlocking("Valvano July 26, 2021\n\r");
  Fona_Init(UART1_BAUD_115200); // 9600 should work too
  Fona_SetSMSStorage(SMS_SIM);  // store SMS on SIM card
  UART0_OutStringBlocking("\n\rType message to send text\n\r");
#endif
  while(1){
#ifdef LOGICANALYZER
    TExaS_Task1();     // toggles virtual logic analyzer
    OS_Sleep(100);
#else
    UART0_OutStringBlocking("\n\rText: ");
    UART0_InStringBlocking(string,63); // user enters a string
    LaunchPad_LED(1);
    Fona_SendSMS(PHONE,(uint8_t *)string);
    LaunchPad_LED(0);
#endif
  }
}
/* ****************************************** */
/*          End of Task1 Section              */
/* ****************************************** */


//---------------- Task2 measures temperature and humidity from HC2080
// Main thread scheduled by OS round robin preemptive scheduler
void Task2(void){
  OS_Wait(&I2Cmutex);
  HDC2080_Init();
  OS_Signal(&I2Cmutex);
  while(1){
#ifdef LOGICANALYZER
    TExaS_Task2();     // toggles virtual logic analyzer
#endif
    OS_Wait(&I2Cmutex);
    Humidity = HDC2080_ReadHumidity();
    Temperature = HDC2080_ReadTemperature();
    OS_Signal(&I2Cmutex);
    OS_Sleep(2000);     // every 2 seconds
  }
}
/* ****************************************** */
/*          End of Task2 Section              */
/* ****************************************** */


//------------Task3 handles accelerator-------



/* 1 frames containing a 1 byte header, 6 bytes of accelerometer,
 * 6 bytes of gyroscope and 8 bytes of magnetometer data. This results in
 * 21 bytes per frame. Additional 40 bytes in case sensor time readout is enabled */
#define FIFO_SIZE   250

/* Variable declarations */
struct bmi160_dev bmi;
struct bmm150_dev bmm;
uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_aux_data aux_data;
struct bmm150_mag_data mag_data;
struct bmi160_sensor_data gyro_data, accel_data;
int8_t rslt;

/* Auxiliary function definitions */
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
 // (void) id; /* id is unused here */
  return bmi160_aux_read(reg_addr, reg_data, len, &bmi);
}

int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
//  (void) id; /* id is unused here */
  return bmi160_aux_write(reg_addr, reg_data, len, &bmi);
}

int8_t I2cGetRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
  if(len == 1){
    I2CB1_Send(dev_addr, &reg_addr, 1);
    data[0] = I2CB1_Recv1(dev_addr);
  }else{
    I2CB1_Send(dev_addr, &reg_addr, 1);
    I2CB1_Recv(dev_addr,data,len);
  }
  return BMI160_OK;
}

int8_t I2cSetRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
  if(len == 1){
    I2CB1_Send2(dev_addr, reg_addr, data[0]);
    return BMI160_OK;
  }
  if(len == 2){
    I2CB1_Send3(dev_addr, reg_addr, data);
    return BMI160_OK;
  }
  if(len == 3){
    I2CB1_Send4(dev_addr, reg_addr, data);
    return BMI160_OK;
  }
  return BMI160_E_INVALID_INPUT;
}
#ifdef ERRORCHECK
void CheckFail(char *message){
  if(rslt){
    while(1){
      P1->OUT ^= 0x01;         // profile
      Clock_Delay1ms(500);
    }
  }
}
#else
#define CheckFail(X)
#endif

// Main thread scheduled by OS round robin preemptive scheduler
// non-real-time task
// reads data from 9-axis IMU
// Inputs:  none
// Outputs: none
void Task3(void){
  OS_Wait(&I2Cmutex);
  bmi.id = BMI160_I2C_ADDR;
  bmi.read = I2cGetRegs;
  bmi.write = I2cSetRegs;
  bmi.delay_ms = Clock_Delay1ms;
  bmi.interface = BMI160_I2C_INTF;

      /* The BMM150 API tunnels through the auxiliary interface of the BMI160 */
      /* Check the pins of the BMM150 for the right I2C address */
  bmm.dev_id = BMI160_AUX_BMM150_I2C_ADDR;
  bmm.intf = BMM150_I2C_INTF;
  bmm.read = bmm150_aux_read;
  bmm.write = bmm150_aux_write;
  bmm.delay_ms = Clock_Delay1ms;

  rslt = bmi160_soft_reset(&bmi);
  CheckFail("bmi160_soft_reset");
  rslt = bmi160_init(&bmi);
  CheckFail("bmi160_init");

      /* Configure the BMI160's auxiliary interface for the BMM150 */
  bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE;
  bmi.aux_cfg.aux_i2c_addr = bmm.dev_id;
  bmi.aux_cfg.manual_enable = BMI160_ENABLE; /* Manual mode */
  bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3; /* 8 bytes */
  rslt = bmi160_aux_init(&bmi);
  CheckFail("bmi160_aux_init");

  rslt = bmm150_init(&bmm);
  CheckFail("bmm150_init");

      /* Configure the accelerometer */
  bmi.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
  bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
  bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
  bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

      /* Configure the gyroscope */
  bmi.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
  bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
  bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
  bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

  rslt = bmi160_set_sens_conf(&bmi);
  CheckFail("bmi160_set_sens_conf");

     /* Configure the magnetometer. The regular preset supports up to 100Hz in Forced mode */
  bmm.settings.preset_mode = BMM150_PRESETMODE_REGULAR;
  rslt = bmm150_set_presetmode(&bmm);
  CheckFail("bmm150_set_presetmode");

      /* It is important that the last write to the BMM150 sets the forced mode.
       * This is because the BMI160 writes the last value to the auxiliary sensor
       * after every read */
  bmm.settings.pwr_mode = BMM150_FORCED_MODE;
  rslt = bmm150_set_op_mode(&bmm);
  CheckFail("bmm150_set_op_mode");

  uint8_t bmm150_data_start = BMM150_DATA_X_LSB;
  bmi.aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ;
  rslt = bmi160_set_aux_auto_mode(&bmm150_data_start, &bmi);
  CheckFail("bmi160_set_aux_auto_mode");

      /* Link the FIFO memory location */
  fifo_frame.data = fifo_buff;
  fifo_frame.length = FIFO_SIZE;
  bmi.fifo = &fifo_frame;

   /* Clear all existing FIFO configurations */
  rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK , BMI160_DISABLE, &bmi);
  CheckFail("bmi160_set_aux_auto_mode");

  uint8_t fifo_config = BMI160_FIFO_HEADER | BMI160_FIFO_AUX |  BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO;
  rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
  CheckFail("bmi160_set_fifo_config");
  OS_Signal(&I2Cmutex);

  while(1){
#ifdef LOGICANALYZER
    TExaS_Task3();     // toggles virtual logic analyzer
#endif
    /* It is VERY important to reload the length of the FIFO memory as after the
     * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
     * number of bytes read from the FIFO */
    OS_Wait(&I2Cmutex);
    bmi.fifo->length = FIFO_SIZE;
    rslt = bmi160_get_fifo_data(&bmi);
    /* Check rslt for any error codes */

    uint8_t aux_inst = 1, gyr_inst = 1, acc_inst = 1;
    rslt = bmi160_extract_aux(&aux_data, &aux_inst, &bmi);
    CheckFail("bmi160_extract_aux");
    rslt = bmi160_extract_gyro(&gyro_data, &gyr_inst, &bmi);
    CheckFail("bmi160_extract_gyro");
    rslt = bmi160_extract_accel(&accel_data, &acc_inst, &bmi);
    CheckFail("bmi160_extract_accel");

    rslt = bmm150_aux_mag_data(&aux_data.data[0], &bmm);
    OS_Signal(&I2Cmutex);
    CheckFail("bmm150_aux_mag_data");
        /* Copy the compensated magnetometer data */
    mag_data = bmm.data;
    Rx = gyro_data.x;
    Ry = gyro_data.y;
    Rz = gyro_data.z;
    Ax = accel_data.x;
    Ay = accel_data.y;
    Az = accel_data.z;
    Mx = mag_data.x;
    My = mag_data.y;
    Mz = mag_data.z;
    OS_Sleep(100); // 10 Hz
  }
}
/* ****************************************** */
/*          End of Task3 Section              */
/* ****************************************** */


//------------Task4 measures temperature from TMP117-------
// Main thread scheduled by OS round robin preemptive scheduler
// measures temperature
// Inputs:  none
// Outputs: none
void Task4(void){
  OS_Wait(&I2Cmutex);
  TMP117_Init();
  OS_Signal(&I2Cmutex);
  while(1){
#ifdef LOGICANALYZER
    TExaS_Task4();     // toggles virtual logic analyzer
#endif
    OS_Wait(&I2Cmutex);
    RawTemperature = TMP117_ReadTemperature();
    OS_Signal(&I2Cmutex);
    FixedTemperature = (10*RawTemperature)/128; // 0.1C
    OS_Sleep(1000);     // every 1 second
  }
}
/* ****************************************** */
/*          End of Task4 Section              */
/* ****************************************** */


//------- Task5 measures light from OPT3001 -----------
// Main thread scheduled by OS round robin preemptive scheduler
void Task5(void){
  OS_Wait(&I2Cmutex);
  OPT3001_Init();
  OS_Signal(&I2Cmutex);
  while(1){
#ifdef LOGICANALYZER
    TExaS_Task5();     // toggles virtual logic analyzer
#endif
    OS_Wait(&I2Cmutex);
    Light = OPT3001_ReadLight();
    OS_Signal(&I2Cmutex);
    OS_Sleep(1000);     // every 1 second
  }
}
/* ****************************************** */
/*          End of Task5 Section              */
/* ****************************************** */

//---------------- Task6 dummy task ----------------
// *********Task6*********
// This OS needs one foreground task that never blocks or sleeps
// Main thread scheduled by OS round robin preemptive scheduler
uint32_t CPUTimeAvailable; // the faster this counts the more CPU time available
void Task6(void){
  CPUTimeAvailable = 0;
  while(1){
#ifdef LOGICANALYZER
    TExaS_Task6();     // toggles virtual logic analyzer
#endif
    CPUTimeAvailable++;
    OS_Suspend();   // release processor to other tasks
  }
}
/* ****************************************** */
/*          End of Task6 Section              */
/* ****************************************** */

// Remember that you must have exactly one main() function, so
// to work on this step, you must rename all other main()
// functions in this file.
// can't run both TExaS logic analyzer and Fona debugging
int realmain(void){ //RTOS main
  OS_Init();
  Task0_Init();       // real-time init
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  LaunchPad_Init();
#ifdef LOGICANALYZER
  TExaS_Init();
#else
  UART0_Initprintf(); // initialize UART and printf
#endif
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  OS_InitSemaphore(&I2Cmutex, 1); // 1 means free
  OS_FIFO_Init();                 // initialize FIFO used to send data between Task1 and Task2
  // Task 0 should run every 1ms
  OS_AddPeriodicEventThread(&Task0, 1);

  // Task1, Task2, Task3, Task4, Task5, Task6 are main threads
  OS_AddThreads(&Task1, &Task2, &Task3, &Task4, &Task5, &Task6);
  OS_Launch(48000); // 1ms switching, doesn't return, interrupts enabled in here
  return 0;         // this never executes
}
