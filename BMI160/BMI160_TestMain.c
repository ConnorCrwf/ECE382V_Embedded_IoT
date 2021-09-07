// BMI160_TestMain.c
//*****************************************************************************
// Lab 21 main for Robot with BMI160/BMM150 Inertial Measurement Unit IMU
// MSP432 with BP-BASSENSORSMKII boosterpack
// derived from https://github.com/BoschSensortec/BMI160_driver/wiki/How-to-use-an-auxiliary-sensor-or-magnetometer-with-the-BMI160.
// Daniel and Jonathan Valvano
// July 6, 2020
//****************************************************************************
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
// see BMI160.h for BMI160 hardware connections


/* J1.5 P4.1
RSLK CC3100 nHIB for WIFI
BP-BASSENSORSMKII Sensor INT1 on BMI160 IMU
Conflict: cannot use both BP-BASSENSORSMKII and CC3100 WIFI booster
Resolution: plug only one boosterpack and not both

J2.13 P5.0
RSLK ERB (right wheel tachometer)
BP-BASSENSORSMKII Sensor INT2
Conflict: You cannot use INT2 functionality on the BMI160 Six-Axis Inertial Measurement sensor.
a) Remove R9 to disconnect INT2 BMI160 Six-Axis Inertial Measurement
b) Program INT2 disabled (default setting, see below)

 */


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/bmi160.h"
#include "../inc/bmm150.h"
#include "../inc/LaunchPad.h"
#include "../inc/TimerA1.h"


// Select whether or not to check for errors
#define ERRORCHECK 1

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
int Semaphore;
int32_t Ax,Ay,Az; // acceleration
int32_t Rx,Ry,Rz; // gyro pitch, roll, yaw
int32_t Mx,My,Mz; // magnetic field strength
void Background_ISR(void){
    P1->OUT ^= 0x01;         // profile
    P1->OUT ^= 0x01;         // profile
    /* It is VERY important to reload the length of the FIFO memory as after the
     * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
     * number of bytes read from the FIFO */
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
    Semaphore++;
    P1->OUT ^= 0x01;         // profile
}

void main(void){
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init();

   /* Initialize your host interface to the BMI160 */
  I2CB1_Init(30);      // baud rate = 12MHz/30=400kHz

    /* This example uses I2C as the host interface */
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

  TimerA1_Init(&Background_ISR,5000);    // 100 Hz sampling
  Semaphore = 0;

  EnableInterrupts();

  while(1) {
    if(Semaphore){
        // process data
      Semaphore = 0;
    }
  }
}



