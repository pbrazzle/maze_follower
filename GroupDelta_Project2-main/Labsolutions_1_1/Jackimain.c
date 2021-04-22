// Jackimain.c
// Runs on MSP432
// Main program for the robot to complete a particular
// challenge where it navigates around a track while 
// avoiding running into the walls.
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


#include <stdint.h>
#include "msp.h"
#include "../inc/ADC14.h"
#include "../inc/Bump.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/IRDistance.h"
#include "../inc/LaunchPad.h"
#include "../inc/LPF.h"
#include "../inc/Motor.h"
#include "../inc/Nokia5110.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "../inc/TA3InputCapture.h"

#define FAST 0

//**************Program 15.2*******************
volatile uint32_t nr,nc,nl;
volatile uint32_t ADCflag;
void Program15_2_ISR(void){  // runs at 2000 Hz
  uint32_t raw17,raw14,raw16;
  P1OUT ^= 0x01;         // profile
  P1OUT ^= 0x01;         // profile
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw14); // center is channel 14, P6.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  ADCflag = ADCflag + 1; // semaphore
  P1OUT ^= 0x01;         // profile
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

uint16_t ActualL;                        // actual rotations per minute
uint16_t ActualR;                        // actual rotations per minute
int16_t LeftDuty = 3750;                 // duty cycle of left wheel (0 to 14,998)
int16_t RightDuty = 3750;                // duty cycle of right wheel (0 to 14,998)
#define TACHBUFF 10                      // number of elements in tachometer array
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
#define IRMIN  250                       // if below this distance, the corresponding wheel must spin faster to turn away (units of mm)
#define IRMAX  350                       // if above this distance, the corresponding wheel must spin slower to turn towards (units of mm)
#if FAST
#define IRFORWARD 250
#define IRSTUCK 225
#define IRREVERSEAMOUNT 125
#else
#define IRFORWARD 400                    // if forward measurement below this distance, initiate slow turn (units of mm)
#define IRSTUCK 375                      // if forward measurement below this distance and left and right also low, stuck in a corner so initiate reverse (units of mm)
#define IRREVERSEAMOUNT 75               // rough, imprecise distance to back up if stuck in a corner (units of mm)
#endif
#define IRCENTER ((IRMIN + IRMAX)/2)     // middle of the distance range between 'IRMIN' and 'IRMAX'; used in proportional controller for zero error condition (units of mm)
int32_t SetPoint = IRCENTER;             // desired distance from left and right sensors to the wall (units of mm)
#if FAST
#define PWMNOMINAL 5000
#else
#define PWMNOMINAL 3750                  // duty cycle of wheels if no error (0 to 14,998)
#endif
#define PWMSWING 2250                    // maximum duty cycle deviation to clamp equation; PWMNOMINAL +/- SWING must not exceed the range 0 to 14,998
#define Kp  10                           // proportional controller gain

// Private function that clamps PWM duty cycles to valid range according to constants.
void clamp(void){
  if(RightDuty < (PWMNOMINAL-PWMSWING)) RightDuty = PWMNOMINAL-PWMSWING; // 1,500 to 6,000
  if(RightDuty > (PWMNOMINAL+PWMSWING)) RightDuty = PWMNOMINAL+PWMSWING;
  if(LeftDuty < (PWMNOMINAL-PWMSWING)) LeftDuty = PWMNOMINAL-PWMSWING;   // 1,500 to 6,000
  if(LeftDuty > (PWMNOMINAL+PWMSWING)) LeftDuty = PWMNOMINAL+PWMSWING;
}

void main(void){
  uint32_t raw17,raw14,raw16;
  uint32_t distancesize;
  int i = 0;
  Clock_Init48MHz();                     // set system clock to 48 MHz
  ADCflag = 0;
  distancesize = 64;                     // replace with your choice see "FIR_Digital_LowPassFilter.xls"
  ADC0_InitSWTriggerCh17_14_16();        // initialize channels 17,14,16
  ADC_In17_14_16(&raw17, &raw14, &raw16);// sample
  LPF_Init(raw17, distancesize);         // P9.0/channel 17
  LPF_Init2(raw14, distancesize);        // P6.1/channel 14
  LPF_Init3(raw16, distancesize);        // P9.1/channel 16
  Nokia5110_Init();
  Nokia5110_Clear();
  LaunchPad_Init();
  TimerA1_Init(&Program15_2_ISR, 250);   // 2000 Hz sampling
  Bump_Init();
  Tachometer_Init();
  Motor_Init();
  EnableInterrupts();
  while(1){
    Motor_Stop();
    Nokia5110_Clear();
    Nokia5110_SetCursor(1, 0);           // one leading space, first row
#if FAST
    Nokia5110_OutString("Jacki v2.0");
#else
    Nokia5110_OutString("Jacki v1.0");
#endif
    Nokia5110_SetCursor(0, 2);           // zero leading spaces, third row
    Nokia5110_OutString("Press and");
    Nokia5110_SetCursor(0, 3);           // zero leading spaces, fourth row
    Nokia5110_OutString("release bump");
    Nokia5110_SetCursor(0, 4);           // zero leading spaces, fifth row
    Nokia5110_OutString("sensor to");
    Nokia5110_SetCursor(0, 5);           // zero leading spaces, sixth row
    Nokia5110_OutString("start.");
    while(Bump_Read() == 0){
      // flash the blue LED
      i = i + 1;
      LaunchPad_Output((i&0x01)<<2);
      Clock_Delay1ms(200);               // delay ~0.2 sec at 48 MHz
    }
    Nokia5110_Clear();
    Nokia5110_SetCursor(0, 0);           // zero leading spaces, first row
    Nokia5110_OutString("Desired (%)");
    Nokia5110_SetCursor(0, 1);           // zero leading spaces, second row
    Nokia5110_OutString("L     R     ");
    Nokia5110_SetCursor(0, 2);           // zero leading spaces, third row
    Nokia5110_OutString("Actual (RPM)");
    Nokia5110_SetCursor(0, 3);           // zero leading spaces, fourth row
    Nokia5110_OutString("L     R     ");
    Nokia5110_SetCursor(0, 4);           // zero leading spaces, fifth row
    Nokia5110_OutString("L     R     ");
    Nokia5110_SetCursor(0, 5);           // zero leading spaces, sixth row
    Nokia5110_OutString("C     ");
    for(i=0; i<10; i=i+1){
      // flash the yellow LED
      LaunchPad_Output(0x03);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
      LaunchPad_Output(0x00);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }
    LaunchPad_Output(0x02);
    i = 0;
    LeftDuty = RightDuty = PWMNOMINAL;   // initialize
    SetPoint = IRCENTER;
    Motor_Forward(LeftDuty, RightDuty);
    while(Bump_Read() == 0){
      Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
      i = i + 1;
      if(i >= TACHBUFF){
        i = 0;
        // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
        ActualL = 2000000/avg(LeftTach, TACHBUFF);
        ActualR = 2000000/avg(RightTach, TACHBUFF);
        // update the screen
        Nokia5110_SetCursor(1, 1);       // one leading space, second row
        Nokia5110_OutUDec((uint16_t)LeftDuty/150);
        Nokia5110_SetCursor(7, 1);       // seven leading spaces, second row
        Nokia5110_OutUDec((uint16_t)RightDuty/150);
        Nokia5110_SetCursor(1, 3);       // one leading space, fourth row
        Nokia5110_OutUDec(ActualL);
        Nokia5110_SetCursor(7, 3);       // seven leading spaces, fourth row
        Nokia5110_OutUDec(ActualR);
      }
      if(ADCflag >= 2000){
        ADCflag = 0;
        // update the screen
        Nokia5110_SetCursor(1, 4);       // one leading space, fifth row
        Nokia5110_OutUDec((uint16_t)LeftConvert(nl));
        Nokia5110_SetCursor(7, 4);       // seven leading spaces, fifth row
        Nokia5110_OutUDec((uint16_t)RightConvert(nr));
        Nokia5110_SetCursor(1, 5);       // one leading space, sixth row
        Nokia5110_OutUDec((uint16_t)CenterConvert(nc));
      }
      // control logic: avoid forward wall by reversing due to no room on right or left
      if((CenterConvert(nc) < IRSTUCK) && (RightConvert(nr) < 350) && (LeftConvert(nl) < 350)){
        LaunchPad_Output(0x01);
        // back up with a slight right turn bias
        Motor_Backward(2500, 3750);
        while((CenterConvert(nc) < (IRSTUCK + IRREVERSEAMOUNT)) && (Bump_Read() == 0)){
          // bumper switches should not be triggered while reversing during normal operation
          // however, the robot may erroneously enter this mode while being picked up (due to hand in front of sensor)
          // this allows the user to press the bump switch to immediately leave reverse mode and stop the motors
        }
        // throw away previous wheel settings and initially try going straight
        // another track-specific assumption could be to initially try a slight right turn
        // ultimately, this might not matter if the next pass through the control loop picks something else
        LeftDuty = RightDuty = PWMNOMINAL;
        LaunchPad_Output(0x02);
        Motor_Forward(LeftDuty, RightDuty);
      // control logic: avoid forward wall by slowing right wheel to turn right
      }else if((CenterConvert(nc) < IRFORWARD) && (RightConvert(nr) >= 350)){
#if FAST
        RightDuty = 5000;
        LeftDuty = 5000;
        Motor_Right(LeftDuty, RightDuty);// fast right turn by spinning right wheel backwards
#else
        RightDuty = 1500;
        LeftDuty = 3750;
        Motor_Forward(LeftDuty, RightDuty);
#endif
      // control logic: avoid forward wall by slowing left wheel to turn left
      }else if((CenterConvert(nc) < IRFORWARD) && (LeftConvert(nl) >= 350)){
#if FAST
        RightDuty = 5000;
        LeftDuty = 5000;
        Motor_Left(LeftDuty, RightDuty); // fast left turn by spinning left wheel backwards
#else
        RightDuty = 3750;
        LeftDuty = 1500;
        Motor_Forward(LeftDuty, RightDuty);
#endif
      // control logic: stay close to the left wall when the right wall is out of sight
      }else if(((LeftConvert(nl) < IRMIN) || (LeftConvert(nl) > IRMAX)) && (RightConvert(nr) >= 500)){
        RightDuty = PWMNOMINAL;
        LeftDuty = PWMNOMINAL - (LeftConvert(nl) - IRCENTER)*Kp;
        clamp();
        Motor_Forward(LeftDuty, RightDuty);
      // control logic: stay close to the right wall when the left wall is out of sight
      }else if(((RightConvert(nr) < IRMIN) || (RightConvert(nr) > IRMAX)) && (LeftConvert(nl) > 500)){
        RightDuty = PWMNOMINAL - (RightConvert(nr) - IRCENTER)*Kp;
        LeftDuty = PWMNOMINAL;
        clamp();
        Motor_Forward(LeftDuty, RightDuty);
      // control logic: drive down the middle of the path when both walls are in sight
      }else if((LeftConvert(nl) < 500) && (RightConvert(nr) < 500)){
        if((LeftConvert(nl) > IRCENTER) && (RightConvert(nr) > IRCENTER)){
          SetPoint = (LeftConvert(nl) + RightConvert(nr))/2;
        }else{
          SetPoint = IRCENTER;
        }
        if(LeftConvert(nl) < RightConvert(nr)){
          // closer to the left wall than to the right wall
          RightDuty = PWMNOMINAL + (LeftConvert(nl) - SetPoint)*Kp;
          LeftDuty = PWMNOMINAL - (LeftConvert(nl) - SetPoint)*Kp;
        }else{
          // closer to the right wall than to the left wall
          RightDuty = PWMNOMINAL + (SetPoint - RightConvert(nr))*Kp;
          LeftDuty = PWMNOMINAL - (SetPoint - RightConvert(nr))*Kp;
        }
        clamp();
        Motor_Forward(LeftDuty, RightDuty);
      }
      Clock_Delay1ms(62);                // delay ~0.062 sec at 48 MHz
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
