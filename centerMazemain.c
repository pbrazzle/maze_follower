/*
 * Distance to wall proportional control
 *  Created on: June 20, 2018
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
#include <stdint.h>
#include <stdio.h>
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
#include "../inc/Motors.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/SysTickInts.h"
#include "../inc/ADC14.h"
#include "Ultrasonic.h"


//******************************************************


int16_t UR, UL;  // PWM duty 0 to 14,998
double Left17,Center17,Right17; // IR distances in mm
int32_t Error;
int32_t Ki=10;  // integral controller gain
int32_t Kp=32;  // proportional controller gain

#define TOOCLOSE 200
#define DESIRED 250
int32_t SetPoint = 250;
#define TOOFAR 400

int16_t PWMnominal=2500;
#define SWING 1000
#define PWMMIN (PWMnominal-SWING)
#define PWMMAX (PWMnominal+SWING)
void SysTick_Handler(void){ // runs at 100 Hz
    if((Left17>DESIRED)&&(Right17>DESIRED)){
      SetPoint = (Left17+Right17)/2;
    }else{
      SetPoint = DESIRED;
    }
    if(Left17 < Right17 ){
      Error = Left17-SetPoint;
    }else{
      Error = SetPoint-Right17;
    }
 //   UR = UR + Ki*Error;      // adjust right motor
    UR = PWMnominal+Kp*Error; // proportional control
    UL = PWMnominal-Kp*Error; // proportional control
    if(UR < (PWMnominal-SWING)) UR = PWMnominal-SWING; // 3,000 to 7,000
    if(UR > (PWMnominal+SWING)) UR = PWMnominal+SWING;
    if(UL < (PWMnominal-SWING)) UL = PWMnominal-SWING; // 3,000 to 7,000
    if(UL > (PWMnominal+SWING)) UL = PWMnominal+SWING;
    Motor_DutyLeft(UL);
    Motor_DutyRight(UR);
}


// *******************************************************************
// solution with proportional control, distance to wall
// Runs on MSP432
void main(){//mainProportialControl(void){
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  Bump_Init();   // bump switches
  Ultrasonic_Init();
  Motor_Stop();
  UR = UL = PWMnominal; //initial power
  SysTick_Init(480000,2); // 100 Hz
  EnableInterrupts();
  Motor_Init(UL,UR);
  Motor_Start();
  while(1)
  {
     Left17 = readLeft() * 0.001 * 343;
     Center17 = readCenter() * 0.001 * 343;
     Right17 = readRight() * 0.001 * 343;
  }
}
