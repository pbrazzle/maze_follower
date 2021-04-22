/*
 * Lab19.c
 * solution for Lab 19, RSLK1.1 Android app
 *
 *  Created on: June 17, 2018
 *      Authors: Jonathan Valvano and Daniel Valvano
 *  Modified: August 18, 2018
 */
/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

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

//**********************************************************
#include <stdint.h>
#include "msp.h"
#include "blinker.h"
#include "OutputType.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
#include "../inc/Nokia5110.h"
#include "../inc/MotorSimple.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/LPF.h"
#include "../inc/SysTickInts.h"
#include "../inc/ADC14.h"
#include "../inc/SSD1306.h"

// BLE variables
uint8_t JackiCommand=0;
uint8_t JackiBumpSensor=0;
uint16_t JackiSpeed[2]={1000,1000}; // steering control, and overall speed
uint16_t Switch1;       // 16-bit notify data from Button 1
uint32_t Switch2;       // 32-bit notify data from Button 2
uint32_t time=0;
int32_t Left19,Center19,Right19; // IR distances
int16_t SensorData19[4];   // 3 distances and the bumper
int32_t Right,Left;
uint8_t NPI_GATTSetDeviceNameJacki19[] = {
  SOF,15,0x00,    // length = 15
//  SOF,18,0x00,    // length = 18
  0x35,0x8C,      // SNP Set GATT Parameter (0x8C)
  0x01,           // Generic Access Service
  0x00,0x00,      // Device Name
//  'J','a','c','k','i',' ','A','S','E','E',' ','0',
  'J','a','c','k','i',' ','R','S','L','K',' ','0',
//    'R','S','L','K',' ','E','x','p','l','o','r','e','r',' ','0',
  0x77};          // FCS (calculated by AP_SendMessageResponse)
uint8_t NPI_SetAdvertisementDataJacki19[] = {
  SOF,24,0x00,    // length = 24
//  SOF,27,0x00,    // length = 27
  0x55,0x43,      // SNP Set Advertisement Data
  0x00,           // Scan Response Data
  13,0x09,        // length, type=LOCAL_NAME_COMPLETE
//  'J','a','c','k','i',' ','A','S','E','E',' ','0',
  'J','a','c','k','i',' ','R','S','L','K',' ','0',
//    'R','S','L','K',' ','E','x','p','l','o','r','e','r',' ','0',

// connection interval range
  0x05,           // length of this data
  0x12,           // GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE
  0x50,0x00,      // DEFAULT_DESIRED_MIN_CONN_INTERVAL
  0x20,0x03,      // DEFAULT_DESIRED_MAX_CONN_INTERVAL
// Tx power level
  0x02,           // length of this data
  0x0A,           // GAP_ADTYPE_POWER_LEVEL
  0x00,           // 0dBm
  0x77};          // FCS (calculated by AP_SendMessageResponse)
#define RECVSIZE 128
extern uint8_t RecvBuf[RECVSIZE];
extern const uint8_t NPI_SetAdvertisement1[];
extern const uint8_t NPI_StartAdvertisement[];
//*************AP_StartAdvertisementJacki**************
// Start advertisement for Jacki
// Input:  num is robot number 0 to 99
// Output: APOK if successful,
//         APFAIL if notification not configured, or if SNP failure
int AP_StartAdvertisementJacki19(uint8_t num){volatile int r;
  if(num<10){
    NPI_GATTSetDeviceNameJacki19[18] = ' ';
    NPI_GATTSetDeviceNameJacki19[19] = '0'+num%10;
    NPI_SetAdvertisementDataJacki19[18] = ' ';
    NPI_SetAdvertisementDataJacki19[19] = '0'+num%10;
//    NPI_GATTSetDeviceNameJacki19[21] = ' ';
//    NPI_GATTSetDeviceNameJacki19[22] = '0'+num%10;
//    NPI_SetAdvertisementDataJacki19[21] = ' ';
//    NPI_SetAdvertisementDataJacki19[22] = '0'+num%10;
  }else{
    num = num%100; // 0 to 99
    NPI_GATTSetDeviceNameJacki19[18] = '0'+num/10;
    NPI_GATTSetDeviceNameJacki19[19] = '0'+num%10;
    NPI_SetAdvertisementDataJacki19[18] = '0'+num/10;
    NPI_SetAdvertisementDataJacki19[19] = '0'+num%10;
//    NPI_GATTSetDeviceNameJacki19[21] = '0'+num/10;
//    NPI_GATTSetDeviceNameJacki19[22] = '0'+num%10;
//    NPI_SetAdvertisementDataJacki19[21] = '0'+num/10;
//    NPI_SetAdvertisementDataJacki19[22] = '0'+num%10;
  }
  r =AP_SendMessageResponse((uint8_t*)NPI_GATTSetDeviceNameJacki19,RecvBuf,RECVSIZE);
  r =AP_SendMessageResponse((uint8_t*)NPI_SetAdvertisement1,RecvBuf,RECVSIZE);
  r =AP_SendMessageResponse((uint8_t*)NPI_SetAdvertisementDataJacki19,RecvBuf,RECVSIZE);
  r =AP_SendMessageResponse((uint8_t*)NPI_StartAdvertisement,RecvBuf,RECVSIZE);
  return r;
}

// ********OutValue**********
// Debugging dump of a data value to virtual serial port to PC
// data shown as 1 to 8 hexadecimal characters
// Inputs:  response (number returned by last AP call)
// Outputs: none
void OutValue(char *label,uint32_t value){
  UART0_OutString(label);
  UART0_OutUHex(value);
}
void ReadCommand(void){ // called on a SNP Characteristic Read Indication for characteristic JackiCommand
  OutValue("\n\rRead JackiCommand=",JackiCommand);
}
void ReadJackiBumpSensor(void){ // called on a SNP Characteristic Read Indication for characteristic JackiSensor
  JackiBumpSensor = Bump_Read();
  OutValue("\n\rRead JackiBumpSensor=",JackiBumpSensor);
}
void RunJacki(void){
  if((JackiCommand==0)||(JackiCommand>6)||(Bump_Read()&&(JackiCommand!=2))){
    JackiCommand = 0;
    UART0_OutString(" Stop");
    Blinker_Output(BK_RGHT+BK_LEFT);
    Motor_Stop();
  }
  if(JackiSpeed[0]>4000){
    JackiSpeed[0] = 4000; // direction
  }
  if(JackiSpeed[1]>14000){
    JackiSpeed[1] = 14000; // overall speed
  }
  if(JackiCommand==1){
    UART0_OutString(" Go");
    Blinker_Output(FR_RGHT+FR_LEFT);
    // steering control, and overall speed
    Right = JackiSpeed[1]+(JackiSpeed[1]*(2000-JackiSpeed[0]))/2000;
    Left = JackiSpeed[1]+(JackiSpeed[1]*(JackiSpeed[0]-2000))/2000;
    if(Left>14000){
      Left = 14000;
    }
    if(Right>14000){
      Right = 14000;
    }
    Motor_Forward(Left,Right);
  }
  if(JackiCommand==2){
    UART0_OutString(" Back");
    Blinker_Output(BK_RGHT+BK_LEFT);
    Motor_Backward(JackiSpeed[1],JackiSpeed[1]);
    time=0;
  }
  if(JackiCommand==3){
    UART0_OutString(" Right");
    Blinker_Output(FR_RGHT);
    Motor_Right(JackiSpeed[1],JackiSpeed[1]);
    time=0;
  }
  if(JackiCommand==4){
    UART0_OutString(" Left");
    Blinker_Output(FR_LEFT);
    Motor_Left(JackiSpeed[1],JackiSpeed[1]);
    time=0;
  }
  if(JackiCommand==5){
    UART0_OutString(" Gentle Right");
    Blinker_Output(FR_RGHT);
    Motor_Forward(JackiSpeed[1],(3*JackiSpeed[1])/4);
  }
  if(JackiCommand==6){
    UART0_OutString(" Gentle Left");
    Blinker_Output(FR_LEFT);
    Motor_Forward((3*JackiSpeed[1])/4,JackiSpeed[1]);
  }
}
void WriteCommand(void){ // called on a SNP Characteristic Write Indication on characteristic JackiCommand
  OutValue("\n\rWrite JackiCommand=",JackiCommand);
  RunJacki();
}
void ReadJackiSpeed(void){ // called on a SNP Characteristic Read Indication for characteristic JackiSpeed
  OutValue("\n\rRead LeftJackiSpeed=",JackiSpeed[0]);
  OutValue("\n\rRead RightJackiSpeed=",JackiSpeed[1]);
}
void WriteJackiSpeed(void){  // called on a SNP Characteristic Write Indication on characteristic JackiSpeed
  OutValue("\n\rWrite LeftJackiSpeed=",JackiSpeed[0]);
  OutValue("\n\rWrite RightJackiSpeed=",JackiSpeed[1]);
  RunJacki();
}
void ReadDistance(void){
  OutValue("\n\rRead Left Distance=",SensorData19[0]);
  OutValue("\n\rRead Center Distance=",SensorData19[1]);
  OutValue("\n\rRead Right Distance=",SensorData19[2]);
  OutValue("\n\rBump Sensors=",0xff&SensorData19[3]);
}
void Button1(void){ // called on SNP CCCD Updated Indication
  OutValue("\n\rRight IR CCCD=",AP_GetNotifyCCCD(0));
}
void Button2(void){
  OutValue("\n\rLeft IR CCCD=",AP_GetNotifyCCCD(1));
}
void BLE_Init(uint8_t num){volatile int r;
  UART0_Init();
  EnableInterrupts();
  UART0_OutString("\n\rJacki test project - MSP432-CC2650\n\r");
  r = AP_Init();
  AP_GetStatus();  // optional
  AP_GetVersion(); // optional
  AP_AddService(0xFFF0);
  //------------------------
  JackiCommand = 0;  // read/write parameter
  AP_AddCharacteristic(0xFFF1,1,&JackiCommand,0x03,0x0A,"JackiCommand",&ReadCommand,&WriteCommand);
  //------------------------
  JackiBumpSensor = Bump_Read(); // read only parameter (get from bump sensors)
  AP_AddCharacteristic(0xFFF2,1,&JackiBumpSensor,0x01,0x02,"JackiBumpSensor",&ReadJackiBumpSensor,0);
  //------------------------
  JackiSpeed[0] = JackiSpeed[1] = 500;   // read/write parameter
  AP_AddCharacteristic(0xFFF3,4,JackiSpeed,0x03,0x0A,"JackiSpeed",&ReadJackiSpeed,&WriteJackiSpeed);
  //------------------------
  // four 16-bit read only parameters (get from IR distance sensors)
  AP_AddCharacteristic(0xFFF6,8,SensorData19,0x01,0x02,"JackSensors",&ReadDistance,0);
//  Switch1 = 0;
//  AP_AddNotifyCharacteristic(0xFFF4,2,&Right19,"Right IR",&Button1);
  //------------------------
//  Switch2 = 0x00000000;
//  AP_AddNotifyCharacteristic(0xFFF5,2,&Left19,"Left IR",&Button2);
  //------------------------
  AP_RegisterService();
  AP_StartAdvertisementJacki19(num);
  AP_GetStatus(); // optional
}


volatile uint32_t nr,nc,nl;
volatile uint32_t ADCflag;
void IRsampling19(void){  // runs at 2000 Hz
  uint32_t raw17,raw14,raw16;
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw14); // center is channel 14, P6.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  Left19 = LeftConvert(nl);
  Center19 = CenterConvert(nc);
  Right19 = RightConvert(nr);
  SensorData19[0] = Left19;   // 16 bit versions
  SensorData19[1] = Center19;
  SensorData19[2] = Right19;
  SensorData19[3] = Bump_Read()+256*JackiCommand;
  ADCflag = 1;           // semaphore
}
// *******************************************************************
// main19.c
// Parameter robot  index of robot used in Bluetooth characteristic
// Enumerated parameter "NONE", "LCD", "UART" or "OLED"
// Runs on MSP432
void main19(uint32_t robot, enum outputtype outputType){uint8_t in,last,bump;
uint32_t raw17,raw14,raw16;
  int i = 0;
  uint32_t time1=0,time2 = 0;                    // incremented with every pass through main loop
  in = Bump_Read();
  Motor_Stop();
  Blinker_Output(BK_RGHT+BK_LEFT);
  if(outputType == LCD){
    Nokia5110_Clear();
    Nokia5110_OutString("Priming...  left      mmcenter    mmright     mm            num");
  }else if(outputType == OLED){
    SSD1306_Clear();
    SSD1306_SetCursor(0, 0); SSD1306_OutString("Priming...");
    SSD1306_SetCursor(0, 1); SSD1306_OutString("left      mm");
    SSD1306_SetCursor(0, 2); SSD1306_OutString("center    mm");
    SSD1306_SetCursor(0, 3); SSD1306_OutString("right     mm");
    SSD1306_SetCursor(0, 4); SSD1306_OutString("num");
  }else if(outputType == UART){
    UART0_OutString("Lab19 BLE");
    UART0_OutUDec(robot);
    UART0_OutString("\r\n\r\nLeft (mm)");
    UART0_OutChar(','); // comma
    UART0_OutString("Center (mm)");
    UART0_OutChar(','); // comma
    UART0_OutString("Right (mm)");
    UART0_OutChar(','); // comma
    UART0_OutString("Bump\r\n");
  }
  // prime the arrays with some samples
 // IRDistance_Init();
  ADC0_InitSWTriggerCh17_14_16();   // initialize channels 17,14,16
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  LPF_Init(raw17,512);     // P9.0/channel 17
  LPF_Init2(raw14,512);    // P6.1/channel 14
  LPF_Init3(raw16,512);    // P9.1/channel 16


  // switch to Average mode
  if(outputType == LCD){
    Nokia5110_SetCursor(0, 0);           // zero leading spaces, first row
    Nokia5110_OutString("Lab 19 BLE"); Nokia5110_OutUDec2(robot);
    Nokia5110_SetCursor(0, 5);           // zero leading spaces, sixth row
    Nokia5110_OutString("Bump        ");
  }else if(outputType == OLED){
    SSD1306_SetCursor(0, 0);           // zero leading spaces, first row
    SSD1306_OutString("Lab 19 BLE"); SSD1306_OutUDec2(robot);
    SSD1306_SetCursor(0, 5);           // zero leading spaces, sixth row
    SSD1306_OutString("Bump        ");
  }
  BLE_Init(robot);
  TimerA1_Init(&IRsampling19,250);    // 2000 Hz sampling
  EnableInterrupts();
  last = LaunchPad_Input();
  while(1){
    time++; time1++;
    AP_BackgroundProcess();  // handle incoming SNP frames
    // if bump sensor, then stop
    if(((JackiCommand>1)&&(JackiCommand<5)&&(time>250000))||(JackiCommand>6)||(Bump_Read()&&(JackiCommand!=2))){
      JackiCommand=0;
      Motor_Stop();
    }
    if(time1>1000000){
      time1 = 0;
      Switch1 = LaunchPad_Input()&0x01;   // Button 1
      if(AP_GetNotifyCCCD(0)){
        OutValue("\n\rNotify Right IR=",Right19);
        AP_SendNotification(0);
      }
      Switch2 = (LaunchPad_Input()>>1)&0x01;   // Button 2
      if(AP_GetNotifyCCCD(1)){
        OutValue("\n\rNotify Left IR=",Left19);
        AP_SendNotification(1);
      }
    }
    in = LaunchPad_Input();
    if(((last&0x01)==0)&&(in&0x01)){
      Clock_Delay1ms(2);  // debounce
      if(JackiCommand==0){
        JackiCommand = 1;
        RunJacki();
      }
    }
    if(((last&0x02)==0)&&(in&0x02)){
      Clock_Delay1ms(2);  // debounce
      if(JackiSpeed[0]<12000){
        JackiSpeed[0] += 2000;
        JackiSpeed[1] = JackiSpeed[0];
      }else{
        JackiSpeed[0] = JackiSpeed[1] = 1000;
      }
      RunJacki();
    }
    if(((last&0x03))&&(in&0x03)!=0x03){
      Clock_Delay1ms(2);  // debounce
    }
    last = in;
    time2 = time2 + 1;
    if((time2%17777) == 0){              // calibration value is basically a guess to get about 10 Hz
      time2 = 0;
      // take IR distance measurements
      LaunchPad_Output((i&0x01)<<2);     // toggle the blue LED
      // print IR distance average
      if(outputType == LCD){
        Nokia5110_SetCursor(5, 1);       // five leading spaces, second row
        Nokia5110_OutUDec(Left19);
        Nokia5110_SetCursor(5, 2);       // five leading spaces, third row
        Nokia5110_OutUDec(Center19);
        Nokia5110_SetCursor(5, 3);       // five leading spaces, fourth row
        Nokia5110_OutUDec(Right19);
      }else if(outputType == OLED){
        SSD1306_SetCursor(5, 1);       // five leading spaces, second row
        SSD1306_OutUDec(Left19);
        SSD1306_SetCursor(5, 2);       // five leading spaces, third row
        SSD1306_OutUDec(Center19);
        SSD1306_SetCursor(5, 3);       // five leading spaces, fourth row
        SSD1306_OutUDec(Right19);
      }else if(outputType == UART){
        UART0_OutUDec(Left19);
        UART0_OutChar(','); // comma
        UART0_OutUDec(Center19);
        UART0_OutChar(','); // comma
        UART0_OutUDec(Right19);
        UART0_OutChar(','); // comma
      }
      // print the status of the bump sensors
      bump = Bump_Read();
      if(outputType == LCD){
        Nokia5110_SetCursor(5, 5);       // five leading spaces, sixth row
        if((bump&0x01) == 0){
          Nokia5110_OutChar('0');
        }else{
          Nokia5110_OutChar(' ');
        }
        if((bump&0x02) == 0){
          Nokia5110_OutChar('1');
        }else{
          Nokia5110_OutChar(' ');
        }
        if((bump&0x04) == 0){
          Nokia5110_OutChar('2');
        }else{
          Nokia5110_OutChar(' ');
        }
        if((bump&0x08) == 0){
          Nokia5110_OutChar('3');
        }else{
          Nokia5110_OutChar(' ');
        }
        if((bump&0x10) == 0){
          Nokia5110_OutChar('4');
        }else{
          Nokia5110_OutChar(' ');
        }
        if((bump&0x20) == 0){
          Nokia5110_OutChar('5');
        }else{
          Nokia5110_OutChar(' ');
        }
      }else if(outputType == OLED){
        SSD1306_SetCursor(5, 5);       // five leading spaces, sixth row
        if((bump&0x01) == 0){
          SSD1306_OutChar('0');
        }else{
          SSD1306_OutChar(' ');
        }
        if((bump&0x02) == 0){
          SSD1306_OutChar('1');
        }else{
          SSD1306_OutChar(' ');
        }
        if((bump&0x04) == 0){
          SSD1306_OutChar('2');
        }else{
          SSD1306_OutChar(' ');
        }
        if((bump&0x08) == 0){
          SSD1306_OutChar('3');
        }else{
          SSD1306_OutChar(' ');
        }
        if((bump&0x10) == 0){
          SSD1306_OutChar('4');
        }else{
          SSD1306_OutChar(' ');
        }
        if((bump&0x20) == 0){
          SSD1306_OutChar('5');
        }else{
          SSD1306_OutChar(' ');
        }
      }else if(outputType == UART){
        if((bump&0x01) == 0){
          UART0_OutChar('0');
        }else{
          UART0_OutChar(' ');
        }
        if((bump&0x02) == 0){
          UART0_OutChar('1');
        }else{
          UART0_OutChar(' ');
        }
        if((bump&0x04) == 0){
          UART0_OutChar('2');
        }else{
          UART0_OutChar(' ');
        }
        if((bump&0x08) == 0){
          UART0_OutChar('3');
        }else{
          UART0_OutChar(' ');
        }
        if((bump&0x10) == 0){
          UART0_OutChar('4');
        }else{
          UART0_OutChar(' ');
        }
        if((bump&0x20) == 0){
          UART0_OutChar('5');
        }else{
          UART0_OutChar(' ');
        }
        UART0_OutChar(CR); UART0_OutChar(LF); // carriage return
      }
    }
  }
}


