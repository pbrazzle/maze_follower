//*****************************************************************************
//
// Jacki FSM test main
// MSP432 with Jacki
// Daniel and Jonathan Valvano
// July 11, 2019
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
#include "../inc/TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
// This program just uses the middle two sensors
// reflectance sensor 4 connected to P7.3 right of center
// reflectance sensor 5 connected to P7.4 left of center


// Linked data structure
struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center &fsm[0]
#define Left   &fsm[1]
#define Right  &fsm[2]
State_t fsm[3]={
  {0x03, 50, { Right, Left,   Right,  Center }},  // Center
  {0x02, 50, { Left,  Center, Right,  Center }},  // Left
  {0x01, 50, { Right, Left,   Center, Center }}   // Right
};
State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;


uint8_t Data; // QTR-8RC



void TestMotor(uint16_t duty){uint32_t in;
  do{
    in = LaunchPad_Input();
  }while((in&0x01)==0);  // start on switch 1 press
  Motor_Forward(duty,duty);
  do{
    in = LaunchPad_Input();
  }while((in&0x02)==0);  // stop on switch 2 press
}
uint8_t JackiCommand=0;
uint8_t JackiBumpSensor=0;
uint16_t JackiSpeed=0;
uint16_t Switch1;       // 16-bit notify data from Button 1
uint32_t Switch2;       // 32-bit notify data from Button 2
uint32_t time=0;
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
  if((JackiCommand==0)||(JackiCommand>5)||(Bump_Read())){
    JackiCommand = 0;
    UART0_OutString(" Stop");
    Motor_Stop();
  }
  if(JackiSpeed>14000){
    JackiSpeed = 1000;
  }
  if(JackiCommand==1){
    UART0_OutString(" Go");
    Motor_Forward(JackiSpeed,JackiSpeed);
  }
  if(JackiCommand==2){
    UART0_OutString(" Back");
    Motor_Backward(JackiSpeed/2,JackiSpeed/2);
    time=0;
  }
  if(JackiCommand==3){
    UART0_OutString(" Right");
    Motor_Right(JackiSpeed/2,JackiSpeed/2);
    time=0;
  }
  if(JackiCommand==4){
    UART0_OutString(" Left");
    Motor_Left(JackiSpeed/2,JackiSpeed/2);
    time=0;
  }
  if(JackiCommand==5){
    UART0_OutString(" FSM line follow");
    Motor_Forward(JackiSpeed,JackiSpeed);
    time=0;
  }
}
void WriteCommand(void){ // called on a SNP Characteristic Write Indication on characteristic JackiCommand
  OutValue("\n\rWrite JackiCommand=",JackiCommand);
  RunJacki();
}
void ReadJackiSpeed(void){ // called on a SNP Characteristic Read Indication for characteristic JackiSpeed
  OutValue("\n\rRead JackiSpeed=",JackiSpeed);
}
void WriteJackiSpeed(void){  // called on a SNP Characteristic Write Indication on characteristic JackiSpeed
  OutValue("\n\rJackiSpeed=",JackiSpeed);
  RunJacki();
}
void Button1(void){ // called on SNP CCCD Updated Indication
  OutValue("\n\rButton 1 CCCD=",AP_GetNotifyCCCD(0));
}
void Button2(void){
  OutValue("\n\rButton 2 CCCD=",AP_GetNotifyCCCD(1));
}
void BLE_Init(void){volatile int r;
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
  JackiBumpSensor = LaunchPad_Input(); // read only parameter (get from switches)
  AP_AddCharacteristic(0xFFF2,1,&JackiBumpSensor,0x01,0x02,"JackiBumpSensor",&ReadJackiBumpSensor,0);
  //------------------------
  JackiSpeed = 100;   // write only parameter
  AP_AddCharacteristic(0xFFF3,2,&JackiSpeed,0x03,0x0A,"JackiSpeed",&ReadJackiSpeed,&WriteJackiSpeed);
    //------------------------
  Switch1 = 0;
  AP_AddNotifyCharacteristic(0xFFF4,2,&Switch1,"Button 1",&Button1);
  //------------------------
  Switch2 = 0x00000000;
  AP_AddNotifyCharacteristic(0xFFF5,4,&Switch2,"Button 2",&Button2);
    //------------------------
  AP_RegisterService();
  AP_StartAdvertisementJacki();
  AP_GetStatus(); // optional
}
uint16_t left,right;
void main(void){//uint8_t in,last;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  Bump_Init(); // bump switches
 // in = Bump_Read();
  Motor_Stop();
  Reflectance_Init();
  TExaS_Init(LOGICANALYZER);  // optional
  BLE_Init();
  Spt = Center;
  EnableInterrupts();
//  last = LaunchPad_Input();
  JackiSpeed = 2000;
  JackiCommand = 5;  // line follow
  while(1){
    time++;
    AP_BackgroundProcess();  // handle incoming SNP frames
     // if bump sensor, then stop
    if(((JackiCommand>1)&&(time>1000000))||(JackiCommand>5)||(Bump_Read())){
      JackiCommand=0;
      Motor_Stop();
    }else if(JackiCommand==5){
      Output = Spt->out;            // set output from FSM
      if(Output&2){
        left = JackiSpeed;
      }else{
        left = 10;
      }
      if(Output&1){
        right = JackiSpeed;
      }else{
        right = 10;
      }
      Motor_Forward(left,right);
      TExaS_Set(Input<<2|Output);   // optional, send data to logic analyzer
      Clock_Delay1ms(Spt->delay);   // wait
      Input = Reflectance_Center(1000);    // read sensors 0,1,2,3
      Spt = Spt->next[Input];       // next depends on input and state
    }
 /*   in = LaunchPad_Input();
    if(((last&0x01)==0)&&(in&0x01)){
      Clock_Delay1ms(2);  // debounce
      if(JackiCommand==0){
        JackiCommand = 1;
        RunJacki();
      }
    }
    if(((last&0x02)==0)&&(in&0x02)){
      Clock_Delay1ms(2);  // debounce
      if(JackiSpeed<12000){
        JackiSpeed += 2000;
      }else{
        JackiSpeed = 1000;
      }
      RunJacki();
    }
    if(((last&0x03))&&(in&0x03)!=0x03){
      Clock_Delay1ms(2);  // debounce
    }
    last = in;
    */
  }
//  while(1){
//    TestMotor(5000);
//    TestMotor(10000);
//    TestMotor(14000);
//  }
}
