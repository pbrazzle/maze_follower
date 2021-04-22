// Lab21_BMI160_TestMain.c
//*****************************************************************************
// Lab 21 main for Robot with BMI160/BMM150 Inertial Measurement Unit IMU
// MSP432 with RSLK Max and BP-BASSENSORSMKII boosterpack
// derived from https://github.com/BoschSensortec/BMI160_driver/wiki/How-to-use-an-auxiliary-sensor-or-magnetometer-with-the-BMI160.
// Daniel and Jonathan Valvano
// July 27, 2020
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
// see Nokia5110.h LCD hardware connections
// see SSD1306.h for OLED hardware connections
// see UART0.h for UART0 hardware connections

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
#include "../inc/TimerA1.h"
#include "../inc/LaunchPad.h"
#include "../inc/BumpInt.h"
#include "../inc/Motor.h"
#include "../inc/LPF.h"


// Select whether or not to check for errors
#define ERRORCHECK 1

// Select one of the following three output possibilities
// #define USENOKIA
#define USEOLED 1
//#define USEUART

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#define OutSDec Nokia5110_OutSDec
#define OutUFix1 Nokia5110_OutUFix1

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
#define OutSDec SSD1306_OutSDec
#define OutUFix1 SSD1306_OutUFix1
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "../inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 5){
    UART0_OutString("\n\rH,T = ");
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
#define OutSDec UART0_OutSDec
#define OutUFix1 UART0_OutUFix1
#endif







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
int semaphore;

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
    OutString(message);
    while(1){
      P1->OUT ^= 0x01;         // profile
      Clock_Delay1ms(500);
    }
  }
}
#else
#define CheckFail(X)
#endif
int32_t Xsum,Ysum,Xm,Ym;
#define MAGCOUNT 64
void Background_ISR(void){   // 100 Hz real time
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
    Xsum += mag_data.x + 120; // calibration
    Ysum += mag_data.y + 120; // calibration
    semaphore++;
    P1->OUT ^= 0x01;         // profile
}
uint8_t CollisionData, CollisionFlag;  // mailbox
void Crash(uint8_t sensor){
  Motor_Stop();
  CollisionData = sensor;
  CollisionFlag = 1;
}
void Display(void){
  SetCursor(4,1); OutSDec(Xm);
  SetCursor(4,2); OutSDec(Ym);
  SetCursor(4,3); OutSDec(gyro_data.x);
  SetCursor(4,4); OutSDec(gyro_data.y);
  SetCursor(4,5); OutSDec(gyro_data.z);
  SetCursor(4,6); OutSDec(accel_data.x);
  SetCursor(4,7); OutSDec(accel_data.y);
}
#define CLOSE 4
#define FAR  10
// timeout is in 10ms*MAGCOUNT = 640ms
// direction 1 for Right, 0 for Left
// U is motor speed
int TurnNorth(uint32_t U, int direction, uint32_t timeout){
  SetCursor(0,0); OutString("TurnNorth");
  if(direction){
    Motor_Right(U,U); // spin
  }else{
    Motor_Left(U,U); // spin
  }
  semaphore = 0;
  while(timeout){
    if(semaphore >= MAGCOUNT){
      semaphore = 0;
      timeout--;
      Xm = Xsum/MAGCOUNT; Xsum=0;
      Ym = Ysum/MAGCOUNT; Ysum=0;
      if((Xm > -CLOSE)&&(Xm < CLOSE)&&(Ym > FAR)){
        Motor_Stop(); // pointing north
        SetCursor(0,0); OutString("North    ");
        return 1; // success
      }
      Display();
    }
  }
  return 0; // success
}
int TurnEast(uint32_t U, int direction, uint32_t timeout){
  SetCursor(0,0); OutString("TurnEast ");
  if(direction){
    Motor_Right(U,U); // spin
  }else{
    Motor_Left(U,U); // spin
  }
  semaphore = 0;
  while(timeout){
    if(semaphore >= MAGCOUNT){
      semaphore = 0;
      timeout--;
      Xm = Xsum/MAGCOUNT; Xsum=0;
      Ym = Ysum/MAGCOUNT; Ysum=0;
      if((Ym > -CLOSE)&&(Ym < CLOSE)&&(Xm > FAR)){
        Motor_Stop(); // pointing east
        SetCursor(0,0); OutString("East     ");
        return 1; // success
      }
      Display();
    }
  }
  return 0; // success
}
// timeout is in 10ms*MAGCOUNT = 640ms
// direction 1 for Right, 0 for Left
// U is motor speed
int TurnSouth(uint32_t U, int direction, uint32_t timeout){
  SetCursor(0,0); OutString("TurnSouth");
  if(direction){
    Motor_Right(U,U); // spin
  }else{
    Motor_Left(U,U); // spin
  }
  semaphore = 0;
  while(timeout){
    if(semaphore >= MAGCOUNT){
      semaphore = 0;
      timeout--;
      Xm = Xsum/MAGCOUNT; Xsum=0;
      Ym = Ysum/MAGCOUNT; Ysum=0;
      if((Xm > -CLOSE)&&(Xm < CLOSE)&&(Ym < -FAR)){
        Motor_Stop(); // pointing south
        SetCursor(0,0); OutString("South    ");
        return 1; // success
      }
      Display();
    }
  }
  return 0; // success
}
int TurnWest(uint32_t U, int direction, uint32_t timeout){
  SetCursor(0,0); OutString("TurnWest ");
  if(direction){
    Motor_Right(U,U); // spin
  }else{
    Motor_Left(U,U); // spin
  }
  semaphore = 0;
  while(timeout){
    if(semaphore >= MAGCOUNT){
      semaphore = 0;
      timeout--;
      Xm = Xsum/MAGCOUNT; Xsum=0;
      Ym = Ysum/MAGCOUNT; Ysum=0;
      if((Ym > -CLOSE)&&(Ym < CLOSE)&&(Xm < -FAR)){
        Motor_Stop(); // pointing west
        SetCursor(0,0); OutString("West     ");
        return 1; // success
      }
      Display();
    }
  }
  return 0; // success
}



void main1(void){ // display data
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init();

   /* Initialize your host interface to the BMI160 */
  I2CB1_Init(30);      // baud rate = 12MHz/30=400kHz
  Init();      // initialize output device
  Clear();     // clear output device
  OutString("BMM150\n");
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
  semaphore = 0; Xsum=0; Ysum=0;
  SetCursor(0,1); OutString("mx= ");
  SetCursor(0,2); OutString("my= ");
  SetCursor(0,3); OutString("gx= ");
  SetCursor(0,4); OutString("gy= ");
  SetCursor(0,5); OutString("gz= ");
  SetCursor(0,6); OutString("ax= ");
  SetCursor(0,7); OutString("ay= ");
  EnableInterrupts();
  while(1) {
    if(semaphore==MAGCOUNT){
      semaphore = 0;
      Xm = Xsum/MAGCOUNT; Xsum=0;
      Ym = Ysum/MAGCOUNT; Ysum=0;
      SetCursor(4,1); OutSDec(Xm);
      SetCursor(4,2); OutSDec(Ym);
      SetCursor(4,3); OutSDec(gyro_data.x);
      SetCursor(4,4); OutSDec(gyro_data.y);
      SetCursor(4,5); OutSDec(gyro_data.z);
      SetCursor(4,6); OutSDec(accel_data.x);
      SetCursor(4,7); OutSDec(accel_data.y);
    }
  }
}

void main(void){// magnet_main
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init();
  BumpInt_Init(&Crash);
  Motor_Init();        // activate Lab 13 software

   /* Initialize your host interface to the BMI160 */
  I2CB1_Init(30);      // baud rate = 12MHz/30=400kHz
  Init();      // initialize output device
  Clear();     // clear output device
  OutString("BMM150\n");
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
  semaphore = 0; Xsum=0; Ysum=0;
  SetCursor(0,1); OutString("mx= ");
  SetCursor(0,2); OutString("my= ");
  SetCursor(0,3); OutString("gx= ");
  SetCursor(0,4); OutString("gy= ");
  SetCursor(0,5); OutString("gz= ");
  SetCursor(0,6); OutString("ax= ");
  SetCursor(0,7); OutString("ay= ");
  EnableInterrupts();
  TurnNorth(1000,1,16); // right, 16 sec timeout
/*
  Clock_Delay1ms(1000);
  TurnSouth(1000,1,16); // right, 16 sec timeout
  Clock_Delay1ms(1000);
  TurnNorth(1000,0,16); // left, 16 sec timeout
  Clock_Delay1ms(1000);
  TurnSouth(1000,0,16); // left, 16 sec timeout
  Clock_Delay1ms(1000);
  TurnWest(1000,1,16); // right, 16 sec timeout
  Clock_Delay1ms(1000);
  TurnEast(1000,1,16); // right, 16 sec timeout
  Clock_Delay1ms(1000);
  TurnWest(1000,0,16); // left, 16 sec timeout
  Clock_Delay1ms(1000);
  TurnEast(1000,0,16); // left, 16 sec timeout
  */
  while(1) {
    if(semaphore==MAGCOUNT){
      semaphore = 0;
      Xm = Xsum/MAGCOUNT; Xsum=0;
      Ym = Ysum/MAGCOUNT; Ysum=0;
      SetCursor(4,1); OutSDec(Xm);
      SetCursor(4,2); OutSDec(Ym);
      SetCursor(4,3); OutSDec(gyro_data.x);
      SetCursor(4,4); OutSDec(gyro_data.y);
      SetCursor(4,5); OutSDec(gyro_data.z);
      SetCursor(4,6); OutSDec(accel_data.x);
      SetCursor(4,7); OutSDec(accel_data.y);
    }
  }
}

int32_t XMagMin = -20;    // when slowly turning in a circle, this is the minimum X-value of the compass
int32_t XMagMax = 40;     // when slowly turning in a circle, this is the maximum X-value of the compass
int32_t YMagMin = -20;    // when slowly turning in a circle, this is the minimum Y-value of the compass
int32_t YMagMax = 20;     // when slowly turning in a circle, this is the maximum Y-value of the compass
#define XTURNRANGE 3      // turn is complete when +/- this range
#define YTURNRANGE 3      // turn is complete when +/- this range
//********TurnRight90*****************
// Attempts to turn right 90 degrees using the compass,
// based on the heading at the time when this function is
// called.  Enters low power mode waiting for sensor data to
// come in and returns from this function after completing
// the turn or timing out.
// Inputs: U        motor duty cycle (0 to 14,998)
//         timeout  time in units 10ms*MAGCOUNT = 640ms
//         verbose  zero for no output, non-zero for output
// Outputs: zero if failed by timeout, one if successful
// Assumes: Motor_Init() has been called; interrupts enabled
// Note: will stall forever if 'semaphore' global variable
//  is never incremented by the data collection ISR.
int TurnRight90(uint32_t U, uint32_t timeout, int verbose){
  int32_t xinit, yinit, xtarget, ytarget;
  if(verbose){
    SetCursor(0, 0); OutString("TurnRight");
  }
  semaphore = 0;
  // get initial heading
  while(semaphore < MAGCOUNT){
    WaitForInterrupt();
  }
  xinit = Xsum/semaphore; Xsum = 0;
  yinit = Ysum/semaphore; Ysum = 0;
  // calculate the target values
  if(xinit > 0){
    xtarget = XMagMax - xinit;
  }else{
    xtarget = XMagMin - xinit;
  }
  if(yinit > 0){
    ytarget = YMagMax - yinit;
  }else{
    ytarget = YMagMin - yinit;
  }
// debug code
  if(verbose){
    SetCursor(4, 1); OutSDec(xinit); OutChar('/'); OutSDec(xtarget);
    SetCursor(4, 2); OutSDec(yinit); OutChar('/'); OutSDec(ytarget);
  }
// end of debug code
  // turn on the motors
  Motor_Right(U, U);
  // attempt to discard transient magnetic noise when motors start
  Clock_Delay1ms(1000);
  semaphore = 0;
  while(timeout){
    if(semaphore >= MAGCOUNT){
      Xm = Xsum/semaphore; Xsum = 0;
      Ym = Ysum/semaphore; Ysum = 0;
      semaphore = 0;
      timeout = timeout - 1;
      if((Xm > (xtarget - XTURNRANGE)) &&
         (Xm < (xtarget + XTURNRANGE)) &&
         (Ym > (ytarget - YTURNRANGE)) &&
         (Ym < (ytarget + YTURNRANGE))){
        Motor_Stop();
        if(verbose){
          SetCursor(0 ,0); OutString("Right 90 ");
        }
        return 1; // success
      }
      // debug code
      if(verbose){
        Display();
      }// end of debug code
    }
    WaitForInterrupt();
  }
  Motor_Stop();
  return 0; // failed
}

#define COLLISIONSIZE 100         // size of acceleration arrays
int16_t XAccArray[COLLISIONSIZE]; // holds the last COLLISIONSIZE/100 seconds of X-axis acceleration data
int16_t YAccArray[COLLISIONSIZE]; // holds the last COLLISIONSIZE/100 seconds of Y-axis acceleration data
int32_t YjerkArray[COLLISIONSIZE]; // holds the last COLLISIONSIZE/100 seconds of Y-axis jerk data
int32_t jerk,ax,ay;
int AccArrayIndex = 0;            // index into parallel acceleration data arrays
int CrashIndex;     // index YAccArray that hold first crash
void clearaccglobals(void){
  int i;
  for(i=0; i<COLLISIONSIZE; i=i+1){
    XAccArray[i] = 0;
    YAccArray[i] = 0;
    YjerkArray[i] = 0;
  }
  AccArrayIndex = 0;
  CrashIndex = 0;
}
void mainCollision(void){ // collision detector
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init();
  BumpInt_Init(&Crash); // existing method
  Motor_Init();        // activate Lab 13 software

   /* Initialize your host interface to the BMI160 */
  I2CB1_Init(30);      // baud rate = 12MHz/30=400kHz
  Init();      // initialize output device
  Clear();     // clear output device
  OutString("BMM150\n");
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

  // edit this
  semaphore = 0; Xsum=0; Ysum=0;
  SetCursor(0,1); OutString("mx= ");
  SetCursor(0,2); OutString("my= ");
  SetCursor(0,3); OutString("gx= ");
  SetCursor(0,4); OutString("gy= ");
  SetCursor(0,5); OutString("gz= ");
  SetCursor(0,6); OutString("ax= ");
  SetCursor(0,7); OutString("ay= ");
  clearaccglobals();
  int crashFlag = 0;
  int startupTime = 50; // 0.5sec
  LPF_Init(0,4);
  LPF_Init2(0,4);
  EnableInterrupts();
  while(1) {
    if(semaphore >= 1){
      semaphore = 0;
      if((CollisionFlag == 1)&&(crashFlag==0)){
        CollisionFlag = 0;
        crashFlag = 1;
        CrashIndex = AccArrayIndex;
        if(AccArrayIndex >= COLLISIONSIZE){
          LaunchPad_Output(0x03); // yellow LED means array full when crashed
        }else{
          LaunchPad_Output(0x01); // red LED means data in array valid
        }
      }
      // jerk = a(n)+2a(n-1)-2a(n-2)-a(n-3)  y-axis only
      // collision if jerk > 5000
      SetCursor(4,6); OutSDec(accel_data.x);
      SetCursor(4,7); OutSDec(accel_data.y);
      if(startupTime>0){
        startupTime--;
      }else{
        if(AccArrayIndex < COLLISIONSIZE){
          ax = LPF_Calc(accel_data.x);
          XAccArray[AccArrayIndex] = ax;
          ay = LPF_Calc2(accel_data.y);
          YAccArray[AccArrayIndex] = ay;
          if(AccArrayIndex>3){
            // jerk = a(n) + 2a(n-1) -2a(n-2) –a(n-3)
            jerk = YAccArray[AccArrayIndex]+2*YAccArray[AccArrayIndex-1]
               -2*YAccArray[AccArrayIndex-2]-YAccArray[AccArrayIndex-3];
          }
          else{
            jerk = 0;
          }
          YjerkArray[AccArrayIndex] = jerk;
          AccArrayIndex = AccArrayIndex + 1;
        }
      }

      if(LaunchPad_Input()){
        for(int i=0; i<3; i=i+1){
          LaunchPad_Output(0x00);
          Clock_Delay1ms(250);
          LaunchPad_Output(0x03);
          Clock_Delay1ms(250);
        }
        Motor_Forward(2500, 2500);
//        Motor_Forward(5000, 5000);
//        Motor_Forward(12500, 12500);
        LaunchPad_Output(0x02);
        clearaccglobals();
        crashFlag = 0;
        CollisionFlag = 0;
        startupTime = 50; // 0.5sec
      }
//90 degree turn test
/*
      if(LaunchPad_Input()){
        for(int i=0; i<3; i=i+1){
          LaunchPad_Output(0x00);
          Clock_Delay1ms(250);
          LaunchPad_Output(0x03);
          Clock_Delay1ms(250);
        }
        if(TurnRight90(1000, 16, 1)){
          LaunchPad_Output(0x02);
        }else{
          LaunchPad_Output(0x01);
        }
      }
*/
//end of 90 degree turn test
    }
  }
}

