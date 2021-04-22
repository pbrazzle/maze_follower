// Lab17_Control.c
// Runs on MSP432
// Implementation of the control system.
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
#include <stdio.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/Nokia5110.h"
#include "../inc/LPF.h"
#include "../inc/SysTickInts.h"
#include "../inc/Tachometer.h"
#include "../inc/Reflectance.h"
#include "../inc/UART0.h"

#define OUTUART 1

//******************************************************
// Lab 17 solution, Distance to wall proportional control

volatile uint32_t nr,nc,nl;
volatile uint32_t ADCflag; // Set every 500us on ADC sample
volatile uint32_t ControllerFlag; // set every 10ms on controller execution
int32_t UR, UL;  // PWM duty 0 to 14,998
int32_t Left,Center,Right; // IR distances in mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
int32_t Ki=5;  // integral controller gain
int32_t Kp=4;  // proportional controller gain

void IRsampling(void){  // runs at 2000 Hz
  uint32_t raw17,raw14,raw16;
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw14); // center is channel 14, P6.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  Left = LeftConvert(nl);
  Center = CenterConvert(nc);
  Right = RightConvert(nr);
  ADCflag = 1;           // semaphore
}
void LCDClear1(void){
#if OUTUART
  UART0_Initprintf();
  printf("\n\rLab 17 wall follow proportional controller\n\r");
  printf(" L(mm), C(mm), R(mm), E(mm)\n\r");
#else
  Nokia5110_Init();
  Nokia5110_Clear(); // erase entire display
  Nokia5110_OutString("17: control");
  Nokia5110_SetCursor(0,1); Nokia5110_OutString("IR distance");
  Nokia5110_SetCursor(0,2); Nokia5110_OutString("L= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,3); Nokia5110_OutString("C= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,4); Nokia5110_OutString("R= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,5); Nokia5110_OutString("E= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
#endif
}
void LCDOut1(void){
#if OUTUART
#define USIZE 100
  static uint32_t Rsum=0,Csum=0,Lsum=0,I=0;
  static int32_t Esum=0;
  Lsum +=Left;
  Csum +=Center;
  Rsum +=Right;
  Esum +=Error;
  I = I+1;
  if(I==USIZE){
     printf("%5d, %5d, %5d, %5d\n\r",Lsum/USIZE,Csum/USIZE,Rsum/USIZE,Esum/USIZE);
     Rsum=0,Csum=0,Lsum=0,Esum=0,I=0;
  }
#else
  Nokia5110_SetCursor(3,2); Nokia5110_OutSDec(Left);
  Nokia5110_SetCursor(3,3); Nokia5110_OutSDec(Center);
  Nokia5110_SetCursor(3,4); Nokia5110_OutSDec(Right);
  Nokia5110_SetCursor(3,5); Nokia5110_OutSDec(Error);
#endif
}
#define TOOCLOSE 200
#define DESIRED 250
int32_t SetPoint = 250;
#define TOOFAR 400

#define PWMNOMINAL 2500
#define SWING 1000
#define PWMMIN (PWMNOMINAL-SWING)
#define PWMMAX (PWMNOMINAL+SWING)
void SysTick_Handler(void){ // runs at 100 Hz
  if(Mode){
    if((Left>DESIRED)&&(Right>DESIRED)){
      SetPoint = (Left+Right)/2;
    }else{
      SetPoint = DESIRED;
    }
    if(Left < Right ){
      Error = Left-SetPoint;
    }else {
      Error = SetPoint-Right;
    }
 //   UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UL = PWMNOMINAL-Kp*Error; // proportional control
    if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;
    Motor_Forward(UL,UR);
    ControllerFlag = 1;
  }
}
void Pause(void){int i;
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(200); LaunchPad_Output(0); // off
    Clock_Delay1ms(200); LaunchPad_Output(1); // red
  }
  while(Bump_Read()==0){// wait for touch
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(3); // red/green
  }
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(4); // blue
  }
  for(i=1000;i>100;i=i-200){
    Clock_Delay1ms(i); LaunchPad_Output(0); // off
    Clock_Delay1ms(i); LaunchPad_Output(2); // green
  }
  // restart Jacki
  UR = UL = PWMNOMINAL;    // reset parameters
  Mode = 1;
  ControllerFlag = 0;
}
// Lab 17 solution with proportional control, distance to wall
mainProportialControl(void){
  uint32_t raw17,raw14,raw16;
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  Bump_Init();   // bump switches
  TimerA1_Init(&IRsampling,250);    // 2000 Hz sampling
  Motor_Stop();
  LCDClear1();
  Mode = 0;
  UR = UL = PWMNOMINAL; //initial power
  ADCflag = ControllerFlag = 0; // semaphores

// prime the filters with some samples
  ADC0_InitSWTriggerCh17_14_16();   // initialize channels 17,14,16
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  LPF_Init(raw17,64);     // P9.0/channel 17
  LPF_Init2(raw14,64);    // P4.1/channel 14
  LPF_Init3(raw16,64);    // P9.1/channel 16
  SysTick_Init(480000,2); // 100 Hz
  Pause();
  EnableInterrupts();
  while(1){
    if(Bump_Read()){ // collision
      Mode = 0;
      Motor_Stop();
      Pause();
    }
    if(ControllerFlag){ // 100 Hz , not real time
      LCDOut1();
      ControllerFlag = 0;
    }
  }
}
//****************************************************************************
// incremental speed control from Lab 16

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
#define DESIREDMAX 120                   // maximum rotations per minute
#define DESIREDMIN  30                   // minimum rotations per minute (works poorly at 30 RPM due to 16-bit timer overflow)
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
void main(void){//main_incremental(void){  // incremental control of constant speed (straight line) using tachometer
  int i = 0;
  Clock_Init48MHz();                     // set system clock to 48 MHz
#if OUTUART
  UART0_Initprintf();
  printf("\n\rLab 17 speed controller\n\r");
#else
  Nokia5110_Init();
  Nokia5110_Clear();
  Nokia5110_OutString("Desired(RPM)L     R     Actual (RPM)L     R     Distance(mm)");
#endif
  LaunchPad_Init();
  Bump_Init();
  Tachometer_Init();
  Motor_Init();
  EnableInterrupts();
#if OUTUART
  printf("DesiredR= %d, DesiredL= %d\n\r",DesiredR,DesiredL);
#endif
  while(1){
    Motor_Stop();
    while(Bump_Read() == 0){
#if OUTUART

#else
      // update the screen
      Nokia5110_SetCursor(1, 1);         // one leading space, second row
      Nokia5110_OutUDec(DesiredL);
      Nokia5110_SetCursor(7, 1);         // seven leading spaces, second row
      Nokia5110_OutUDec(DesiredR);
#endif
      if((LaunchPad_Input()&0x01) != 0x00){
        // Button1 has been pressed
        DesiredR = DesiredR + 10;
        if(DesiredR > DESIREDMAX){
          DesiredR = DESIREDMIN;
        }
#if OUTUART
        printf("DesiredR= %d, DesiredL= %d\n\r",DesiredR,DesiredL);
#endif
      }
      if((LaunchPad_Input()&0x02) != 0x00){
        // Button2 has been pressed
        DesiredL = DesiredL + 10;
        if(DesiredL > DESIREDMAX){
          DesiredL = DESIREDMIN;
        }
#if OUTUART
        printf("DesiredR= %d, DesiredL= %d\n\r",DesiredR,DesiredL);
#endif
        }
      // flash the blue LED
      i = i + 1;
      LaunchPad_Output((i&0x01)<<2);
      Clock_Delay1ms(200);               // delay ~0.2 sec at 48 MHz
    }
    for(i=0; i<10; i=i+1){
      // flash the yellow LED
      LaunchPad_Output(0x03);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
      LaunchPad_Output(0x00);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }
    LaunchPad_Output(0x02);
    i = 0;
    while(Bump_Read() == 0){
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
#if OUTUART
        printf("%5d rpm, %5d rpm, %5d steps, %5d steps\n\r",ActualL,ActualR,LeftSteps,RightSteps);
#else        // update the screen
        Nokia5110_SetCursor(1, 3);       // one leading space, fourth row
        Nokia5110_OutUDec(ActualL);
        Nokia5110_SetCursor(7, 3);       // seven leading spaces, fourth row
        Nokia5110_OutUDec(ActualR);
        Nokia5110_SetCursor(0, 5);       // zero leading spaces, sixth row


        if(LeftSteps < 0){
          Nokia5110_OutChar('-');
          Nokia5110_OutUDec((-1*LeftSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
        }else{
          Nokia5110_OutChar(' ');
          Nokia5110_OutUDec((LeftSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
        }
        Nokia5110_SetCursor(6, 5);       // six leading spaces, sixth row
        if(RightSteps < 0){
          Nokia5110_OutChar('-');
          Nokia5110_OutUDec((-1*RightSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
        }else{
          Nokia5110_OutChar(' ');
          Nokia5110_OutUDec((RightSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
        }
#endif
      }
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }
    Motor_Stop();
    i = 0;
    while(Bump_Read() != 0){
      // flash the red LED
      i = i + 1;
      LaunchPad_Output(i&0x01);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }
  }
}
//*********************************************************************
// Lab 17 solution
// Proportional control, Line follower
// Not fully tested; needs tuning
uint8_t LineData;       // direct measure from line sensor
int32_t Position;      // position in 0.1mm relative to center of line
int32_t Kp3=3;
uint32_t Time;
void LCDClear3(void){
  Nokia5110_Init();
  Nokia5110_Clear(); // erase entire display
  Nokia5110_OutString("17: control");
  Nokia5110_SetCursor(0,1); Nokia5110_OutString("Line Follow");
  Nokia5110_SetCursor(0,2); Nokia5110_OutString("D =  "); Nokia5110_OutUDec(0);
  Nokia5110_SetCursor(0,3); Nokia5110_OutString("P = "); Nokia5110_OutSDec(0);
  Nokia5110_SetCursor(0,4); Nokia5110_OutString("UR=  "); Nokia5110_OutUDec(0);
  Nokia5110_SetCursor(0,5); Nokia5110_OutString("UL=  "); Nokia5110_OutUDec(0);
}
void LCDOut3(void){
  Nokia5110_SetCursor(5,2); Nokia5110_OutUDec(LineData);
  Nokia5110_SetCursor(4,3); Nokia5110_OutSDec(Position);
  Nokia5110_SetCursor(5,4); Nokia5110_OutUDec(UR);
  Nokia5110_SetCursor(5,5); Nokia5110_OutUDec(UL);
}
int32_t change=0;
void Controller(void){
// every 1ms
//  LEDOUT ^= 0x01;       // toggle P1.0
//  LEDOUT ^= 0x01;       // toggle P1.0
  Time = Time + 1;
  if(Time%10==1){
    Reflectance_Start(); // start every 10ms
  }
  if(Time%10==2){
    LineData = Reflectance_End(); // finish 1ms later
    Position = Reflectance_Position(LineData);
    if(Mode){
//      change = Kp3*Position; // proportional control
      if(Position > 0){
        change = change+100; // incremental control
      }
      if(Position < 0){
        change = change-100;
      }
      if(change > SWING)change=SWING;
      if(change < -SWING)change=-SWING;

      UR = PWMNOMINAL+change;
      UL = PWMNOMINAL-change; // proportional control
//      if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 1000 to 9000
//      if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
//      if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 1000 to 9000
//      if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;
      Motor_Forward(UL,UR);
    }
    ControllerFlag = 1;
  }
//  LEDOUT ^= 0x01;       // toggle P1.0
}
void Pause3(void){int i;
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(200); LaunchPad_Output(0); // off
    Clock_Delay1ms(200); LaunchPad_Output(1); // red
    if(ControllerFlag){ // 100 Hz , not real time
      LCDOut3();
      ControllerFlag = 0;
    }
  }
  while(Bump_Read()==0){// wait for touch
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(3); // red/green
    if(ControllerFlag){ // 100 Hz , not real time
      LCDOut3();
      ControllerFlag = 0;
    }
  }
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(4); // blue
    if(ControllerFlag){ // 100 Hz , not real time
      LCDOut3();
      ControllerFlag = 0;
    }
  }
  for(i=1000;i>100;i=i-200){
    Clock_Delay1ms(i); LaunchPad_Output(0); // off
    Clock_Delay1ms(i); LaunchPad_Output(2); // green
    if(ControllerFlag){ // 100 Hz , not real time
      LCDOut3();
      ControllerFlag = 0;
    }
  }
  // restart Jacki
  UR = UL = PWMNOMINAL;    // reset parameters
  Mode = 1;
  ControllerFlag = 0;
}
void mainreal(void){//void mainreal(void){
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  Bump_Init();   // bump switches
  Reflectance_Init();
  TimerA1_Init(&Controller,500);    // 1000 Hz interrupt, 100 Hz sampling, 100 Hz controller
  Motor_Stop();
  LCDClear3();
  Mode = 0;
  Time = 0;
  UR = UL = PWMNOMINAL; // initial power
  EnableInterrupts();
  Pause3();

  while(1){
    if(Bump_Read()){ // collision
      Mode = 0;
      Motor_Stop();
      Pause3();
    }
    if(ControllerFlag){ // 100 Hz , not real time
      LCDOut3();
      ControllerFlag = 0;
    }
  }
}
