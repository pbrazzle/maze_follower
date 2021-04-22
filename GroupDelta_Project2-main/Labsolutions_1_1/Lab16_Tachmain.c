// Lab16_Tachmain.c
// Runs on MSP432
// Test the operation of the tachometer by implementing
// a simple DC motor speed controller.
// Daniel Valvano
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

// See Bump.c for bumper connections (Port 8 or Port 4)

// Debug heartbeat connected to P2.0 (built-in red LED)
// Debug heartbeat connected to P2.4

// Pololu kit v1.1 connections:
// Left Encoder A connected to P10.5 (J5)
// Left Encoder B connected to P5.2 (J2.12)
// Right Encoder A connected to P10.4 (J5)
// Right Encoder B connected to P5.0 (J2.13)

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

// Negative logic bump sensors defined in Bump.c (use Port 4)
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// Debug heartbeat connected to P2.0 (built-in red LED)
// Debug heartbeat connected to P1.0 (built-in LED1)

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/Nokia5110.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/TExaS.h"
#include "../inc/FlashProgram.h"
#include "../inc/Bump.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))
#define P1_0 (*((volatile uint8_t *)(0x42098040)))


uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0;               // Timer A3 first edge, P10.4
int Done0;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure0(uint16_t time){
  P2_0 = P2_0^0x01;            // thread profile, P2.0
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;               // setup for next
  Done0 = 1;
}
uint16_t Period1;              // (1/SMCLK) units = 83.3 ns units
uint16_t First1;               // Timer A3 first edge, P10.5
int Done1;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure1(uint16_t time){
  P1_0 = P1_0^0x01;            // thread profile, P1.0
  Period1 = (time - First1)&0xFFFF; // 16 bits, 83.3 ns resolution
  First1 = time;               // setup for next
  Done1 = 1;
}
int main0(void){ //Program16_1(void){
  DisableInterrupts();
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
  P1->SEL0 &= ~0x01;
  P1->SEL1 &= ~0x01;   // configure P1.0 as GPIO
  P1->DIR |= 0x01;     // P1.0 output
  P2->SEL0 &= ~0x01;
  P2->SEL1 &= ~0x01;   // configure P2.0 as GPIO
  P2->DIR |= 0x01;     // P2.0 output
  First0 = First1 = 0; // first will be wrong
  Done0 = Done1 = 0;   // set on subsequent
  Motor_Init();        // activate Lab 12 software
  TimerA3Capture_Init01(&PeriodMeasure0, &PeriodMeasure1);
  Motor_Forward(7500, 7500); // 50%
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
  }
}
uint16_t SpeedBuffer[500];      // RPM
uint32_t PeriodBuffer[500];     // 1/12MHz = 0.083 usec
uint32_t Duty,DutyBuffer[500];  // 0 to 15000
uint32_t Time; // in 0.01 sec
void Collect(void){
  P2_1 = P2_1^0x01;    // thread profile, P2.1
  if(Done0==0) Period0 = 65534; // stopped
  if(Done1==0) Period1 = 65534; // stopped
  Done0 = Done1 = 0;   // set on subsequent
  if(Time==100){       // 1 sec
    Duty = 7500;
    Motor_Forward(7500, 7500);  // 50%
  }
  if(Time==200){       // 2 sec
    Duty = 11250;
    Motor_Forward(11250, 11250);// 75%
  }
  if(Time==300){       // 3 sec
    Duty = 7500;
    Motor_Forward(7500, 7500);  // 50%
  }
  if(Time==400){       // 4 sec
    Duty = 3750;
    Motor_Forward(3750, 3750);  // 25%
  }
  if(Time<500){        // 5 sec
    SpeedBuffer[Time] = 2000000/Period0;
    PeriodBuffer[Time] = Period0;
    DutyBuffer[Time] = Duty;
    Time = Time + 1;
  }
  if((Time==500)||Bump_Read()){
    Duty = 0;
    Motor_Stop();      // 0%
    TimerA1_Stop();
  }
}
int main(void){ //main1(void){
  DisableInterrupts();
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
  P1->SEL0 &= ~0x01;
  P1->SEL1 &= ~0x01;   // configure P1.0 as GPIO
  P1->DIR |= 0x01;     // P1.0 output
  P2->SEL0 &= ~0x01;
  P2->SEL1 &= ~0x01;   // configure P2.0 as GPIO
  P2->DIR |= 0x01;     // P2.0 output
  First0 = First1 = 0; // first will be wrong
  Done0 = Done1 = 0;   // set on subsequent
  Time = 0; Duty = 3750;
  Bump_Init();
  Motor_Init();        // activate Lab 12 software
  TimerA3Capture_Init01(&PeriodMeasure0, &PeriodMeasure1);
  TimerA1_Init(&Collect, 5000); // 100 Hz
  Motor_Forward(3750, 3750); // 25%
  TExaS_Init(LOGICANALYZER_P10);
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
  }
}
#define FLASH_BANK1_MIN     0x00020000  // Flash Bank1 minimum address
#define FLASH_BANK1_MAX     0x0003FFFF  // Flash Bank1 maximum address
void Debug_FlashInit(void){ uint32_t addr;
  Flash_Init(48);
  for(addr=FLASH_BANK1_MIN;addr<0x0003FFFF;addr=addr+4096){
    if(Flash_Erase(addr)==ERROR){
      while(1){
        LaunchPad_Output(BLUE);  Clock_Delay1ms(200);
        LaunchPad_Output(RED);  Clock_Delay1ms(500);
        LaunchPad_Output(GREEN);  Clock_Delay1ms(300);
      }
    }
  }
}
// record 32 halfwords
void Debug_FlashRecord(uint16_t *pt){uint32_t addr;
  addr=FLASH_BANK1_MIN;
  while(*(uint32_t*)addr != 0xFFFFFFFF){ // find first free block
    addr=addr+64;
    if(addr>FLASH_BANK1_MAX) return; // full
  }
  Flash_FastWrite((uint32_t *)pt, addr, 16); // 16 words is 32 halfwords, 64 bytes
}
int main2(void){
int i;
  DisableInterrupts();
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
  LaunchPad_Init();
  if(LaunchPad_Input()){
    LaunchPad_Output(RED);
    Debug_FlashInit(); // erase flash if either switch pressed
    while(LaunchPad_Input()){}; // wait for release
  }
  First0 = First1 = 0; // first will be wrong
  Done0 = Done1 = 0;   // set on subsequent
  Time = 0; Duty = 3750;
  Bump_Init();
  Motor_Init();        // activate Lab 12 software
  TimerA3Capture_Init01(&PeriodMeasure0, &PeriodMeasure1);
  TimerA1_Init(&Collect, 5000); // 100 Hz
  Motor_Forward(3750, 3750); // 25%
  TExaS_Init(LOGICANALYZER_P10);
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
    if(Time>=500){
      LaunchPad_Output(GREEN);
      for(i=0;i<16;i++){
        Debug_FlashRecord(&SpeedBuffer[32*i]);
      }
      while(1){
        LaunchPad_Output(BLUE);  Clock_Delay1ms(200);
        LaunchPad_Output(0);  Clock_Delay1ms(200);
     }
    }
  }
}

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
void realmain(void){
  int i = 0;
  Clock_Init48MHz();                     // set system clock to 48 MHz
  Nokia5110_Init();
  Nokia5110_Clear();
  Nokia5110_OutString("Desired(RPM)L     R     Actual (RPM)L     R     Distance(mm)");
  LaunchPad_Init();
  Bump_Init();
  Tachometer_Init();
  Motor_Init();
  EnableInterrupts();
  while(1){
    Motor_Stop();
    while(Bump_Read() == 0){
      // update the screen
      Nokia5110_SetCursor(1, 1);         // one leading space, second row
      Nokia5110_OutUDec(DesiredL);
      Nokia5110_SetCursor(7, 1);         // seven leading spaces, second row
      Nokia5110_OutUDec(DesiredR);
      if((LaunchPad_Input()&0x01) != 0x00){
        // Button1 has been pressed
        DesiredR = DesiredR + 10;
        if(DesiredR > DESIREDMAX){
          DesiredR = DESIREDMIN;
        }
      }
      if((LaunchPad_Input()&0x02) != 0x00){
        // Button2 has been pressed
        DesiredL = DesiredL + 10;
        if(DesiredL > DESIREDMAX){
          DesiredL = DESIREDMIN;
        }
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
        // update the screen
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


