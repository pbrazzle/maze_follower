/*
 * Lab12.c
 * Solution to PWM output to motors
 *
 *  Created on: June 4, 2018
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

static enum outputtype OutputType12 = NONE;
//**************RSLK1.1***************************
// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)
// *******************************************************************
// Lab12_Motorsmain.c
void Pause12(void){int i;
  Blinker_Output(BK_RGHT+BK_LEFT);
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(200); LaunchPad_Output(0); // off
    Clock_Delay1ms(200); LaunchPad_Output(1); // red
  }
  while(Bump_Read()==0){// wait for touch
    Clock_Delay1ms(100);
    LaunchPad_Output(0); // off
    Blinker_Output(0);
    Clock_Delay1ms(100);
    Blinker_Output(BK_RGHT+BK_LEFT);
    LaunchPad_Output(3); // red/green
  }
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(100);
    Blinker_Output(0);
    LaunchPad_Output(0); // off
    Clock_Delay1ms(100);
    Blinker_Output(FR_RGHT+FR_LEFT);
    LaunchPad_Output(4); // blue
  }
  for(i=500;i>100;i=i-100){
    Clock_Delay1ms(i); LaunchPad_Output(0); // off
    Clock_Delay1ms(i); LaunchPad_Output(2); // green
  }
}

// *******************************************************************
// main12.c
// Enumerated parameter "NONE", "LCD", "UART" or "OLED"
// Runs on MSP432
// does the robot move straight?
void main12(enum outputtype outputType){ // Program12_4
  OutputType12 = outputType;
  Motor_InitSimple(); // initialization
  if((OutputType12 == NONE) || (OutputType12 == LCD)|| (OutputType12 == OLED)){
    TExaS_Init(LOGICANALYZER);  // You can have logic analyzer or UART debugging (not both)
  }
  if(OutputType12 == LCD){
    Nokia5110_Clear();
    Nokia5110_OutString("Lab12 Motor");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("Hit bumper");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString(" to start");
  }else if(OutputType12 == OLED){
    SSD1306_Clear();
    SSD1306_OutString("Lab12 Motor");
    SSD1306_SetCursor(0,1); SSD1306_OutString("Hit bumper");
    SSD1306_SetCursor(0,2); SSD1306_OutString(" to start");
  }else if(OutputType12 == UART){
    UART0_OutString("Lab12 Motor\r\n");
    UART0_OutString("Hit bumper to start\r\n");
  }
  Pause12(); // start on SW1 or SW2
  while(1){
    Blinker_Output(FR_RGHT+FR_LEFT);
    LaunchPad_Output(0x02);
    if(OutputType12 == LCD){
      Nokia5110_SetCursor(0,1); Nokia5110_OutString("Forward   ");
      Nokia5110_SetCursor(0,2); Nokia5110_OutString("pwm = 25%");
    }else if(OutputType12 == OLED){
      SSD1306_SetCursor(0,1); SSD1306_OutString("Forward   ");
      SSD1306_SetCursor(0,2); SSD1306_OutString("pwm = 25%");
    }else if(OutputType12 == UART){
      UART0_OutString("Forward  ");
      UART0_OutChar(9); // tab
      UART0_OutString("pwm = 25%\r\n");
    }
    Motor_ForwardSimple(2500,350);  // 3.5 seconds and stop
    LaunchPad_Output(0x00);
    Blinker_Output(BK_RGHT+BK_LEFT);
    if(OutputType12 == LCD){
      Nokia5110_SetCursor(0,1); Nokia5110_OutString("Stopped  ");
      Nokia5110_SetCursor(0,2); Nokia5110_OutString("         ");
    }else if(OutputType12 == OLED){
      SSD1306_SetCursor(0,1); SSD1306_OutString("Stopped  ");
      SSD1306_SetCursor(0,2); SSD1306_OutString("         ");
    }else if(OutputType12 == UART){
      UART0_OutString("Stopped  \r\n");
    }
    Motor_StopSimple(); Clock_Delay1ms(1000);
    LaunchPad_Output(0x01);
    if(OutputType12 == LCD){
      Nokia5110_SetCursor(0,1); Nokia5110_OutString("Backward ");
      Nokia5110_SetCursor(0,2); Nokia5110_OutString("pwm = 15%");
    }else if(OutputType12 == OLED){
      SSD1306_SetCursor(0,1); SSD1306_OutString("Backward ");
      SSD1306_SetCursor(0,2); SSD1306_OutString("pwm = 15%");
    }else if(OutputType12 == UART){
      UART0_OutString("Backward ");
      UART0_OutChar(9); // tab
      UART0_OutString("pwm = 15%\r\n");
    }
    Motor_BackwardSimple(1500,200); // reverse 2 sec
    LaunchPad_Output(0x03);
    Blinker_Output(FR_RGHT);
    if(OutputType12 == LCD){
      Nokia5110_SetCursor(0,1); Nokia5110_OutString("Right    ");
      Nokia5110_SetCursor(0,2); Nokia5110_OutString("pwm = 15%");
    }else if(OutputType12 == OLED){
      SSD1306_SetCursor(0,1); SSD1306_OutString("Backward ");
      SSD1306_SetCursor(0,2); SSD1306_OutString("pwm = 15%");
    }else if(OutputType12 == UART){
      UART0_OutString("Right    ");
      UART0_OutChar(9); // tab
      UART0_OutString("pwm = 15%\r\n");
    }
    Motor_LeftSimple(1500,200);     // right turn 2 sec
    if(Bump_Read()){
      Blinker_Output(BK_RGHT+BK_LEFT);
      LaunchPad_Output(0x01);
      if(OutputType12 == LCD){
        Nokia5110_SetCursor(0,1); Nokia5110_OutString("Backward ");
        Nokia5110_SetCursor(0,2); Nokia5110_OutString("pwm = 15%");
      }else if(OutputType12 == OLED){
        SSD1306_SetCursor(0,1); SSD1306_OutString("Backward ");
        SSD1306_SetCursor(0,2); SSD1306_OutString("pwm = 15%");
      }else if(OutputType12 == UART){
        UART0_OutString("Backward ");
        UART0_OutChar(9); // tab
        UART0_OutString("pwm = 15%\r\n");
      }
      Motor_BackwardSimple(1500,100);// reverse 1 sec
      Blinker_Output(FR_RGHT);
      LaunchPad_Output(0x03);
      if(OutputType12 == LCD){
        Nokia5110_SetCursor(0,1); Nokia5110_OutString("Right    ");
        Nokia5110_SetCursor(0,2); Nokia5110_OutString("pwm = 15%");
      }else if(OutputType12 == OLED){
        SSD1306_SetCursor(0,1); SSD1306_OutString("Right    ");
        SSD1306_SetCursor(0,2); SSD1306_OutString("pwm = 15%");
      }else if(OutputType12 == UART){
        UART0_OutString("Right    ");
        UART0_OutChar(9); // tab
        UART0_OutString("pwm = 15%\r\n");
      }
      Motor_LeftSimple(1500,200);   // right turn 2 sec
    }
  }
}

// Initialize SysTick with busy wait running at bus clock.
void SysTick12_Init(void){
  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
  SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
}
// Time delay using busy wait.
// The delay parameter is in units of the core clock.
// assumes 48 MHz bus clock
void SysTick12_Wait(uint32_t delay){
  // method #1: set Reload Value Register, clear Current Value Register, poll COUNTFLAG in Control and Status Register
  if((OutputType12 == NONE) || (OutputType12 == LCD)){
    TExaS_Set((P2->OUT>>4)|(P1->OUT>>6)); // send data to logic analyzer
  }
  if(delay <= 1){
    // without this step:
    // if delay == 0, this function will wait 0x00FFFFFF cycles
    // if delay == 1, this function will never return (because COUNTFLAG is set on 1->0 transition)
    return;                   // do nothing; at least 1 cycle has already passed anyway
  }
  SysTick->LOAD = (delay - 1);// count down to zero
  SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
  while(( SysTick->CTRL&0x00010000) == 0){
    if((OutputType12 == NONE) || (OutputType12 == LCD)|| (OutputType12 == OLED)){
      TExaS_Set((P2->OUT>>4)|(P1->OUT>>6)); // send data to logic analyzer
    }
  }
  // method #2: repeatedly evaluate elapsed time
/*  volatile uint32_t elapsedTime;
  uint32_t startTime = SYSTICK->VAL;
  do{
    elapsedTime = (startTime-SYSTICK->VAL)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);*/
}
// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick12_Wait10ms(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
    SysTick12_Wait(480000);  // wait 10ms (assumes 48 MHz clock)
  }
}
// *******Lab 12 solution*******

void Motor_InitSimple(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs
  P5->SEL0 &= ~0x30;
  P5->SEL1 &= ~0x30;  // configure as GPIO
  P5->DIR |= 0x30;    // make pins out
  P5->OUT &= ~0x30;
// initialize P2.6 and P2.7 and make them outputs
  P2->SEL0 &= ~0xC0;
  P2->SEL1 &= ~0xC0;  // configure as GPIO
  P2->DIR |= 0xC0;    // make pins out
  P2->OUT &= ~0xC0;   // P2.6, P2.7 low
 // initialize P3.6 and P3.7 and make them outputs
  P3->SEL0 &= ~0xC0;
  P3->SEL1 &= ~0xC0;  // configure as GPIO
  P3->DIR |= 0xC0;    // make pins out
  P3->OUT &= ~0xC0;   // low current sleep mode
  SysTick12_Init();
  Bump_Init();
  if((OutputType12 == NONE) || (OutputType12 == LCD)){
    TExaS_Set((P2->OUT>>4)|(P1->OUT>>6)); // send data to logic analyzer
  }
}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
  P5->OUT &= ~0x30;
  P2->OUT &= ~0xC0;   // off
  P3->OUT &= ~0xC0;   // low current sleep mode
  if((OutputType12 == NONE) || (OutputType12 == LCD)){
    TExaS_Set((P2->OUT>>4)|(P1->OUT>>6)); // send data to logic analyzer
  }
}
void Motor_ForwardSimple(uint16_t duty, uint32_t time){int i;
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
  P5->OUT &= ~0x30;   // set direction of both motors
  P3->OUT |= 0xC0;    // activate both motors

  for(i=0;i<time;i++){
    if(Bump_Read()) break; // stop on bump switch
    P2->OUT |= 0xC0;       // both motors on
    SysTick12_Wait(48*duty);
    P2->OUT &= ~0xC0;      // both motors off
    SysTick12_Wait(48*(10000-duty));
  }
  Motor_StopSimple();
}
void Motor_BackwardSimple(uint16_t duty, uint32_t time){int i;
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Runs even if any bumper switch is active
// Returns after time*10ms
  P5->OUT |= 0x30;    // set back direction of both motors
  P3->OUT |= 0xC0;    // activate both motors

  for(i=0;i<time;i++){
    P2->OUT |= 0xC0;       // both motors on
    SysTick12_Wait(48*duty);
    P2->OUT &= ~0xC0;      // both motors off
    SysTick12_Wait(48*(10000-duty));
  }
  Motor_StopSimple();
}
void Motor_LeftSimple(uint16_t duty, uint32_t time){ int i;
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
  P5->OUT &= ~0x10;   // set direction of left motor
  P3->OUT &= ~0x40;   // deactivate right motor
  P3->OUT |= 0x80;    // activate left motor

  for(i=0;i<time;i++){
    if(Bump_Read()) break; // stop on bump switch
    P2->OUT |= 0x80;   // left on
    SysTick12_Wait(48*duty);
    P2->OUT &= ~0x80;  // left off
    SysTick12_Wait(48*(10000-duty));
  }
  Motor_StopSimple();
}
void Motor_RightSimple(uint16_t duty, uint32_t time){int i;
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
  P5->OUT &= ~0x20;   // set direction of right motor
  P3->OUT &= ~0x80;   // deactivate left motor
  P3->OUT |= 0x40;    // activate right motor

  for(i=0;i<time;i++){
    if(Bump_Read()) break; // stop on bump switch
    P2->OUT |= 0x40;   // right on
    SysTick12_Wait(48*duty);
    P2->OUT &= ~0x40;  // right off
    SysTick12_Wait(48*(10000-duty));
  }
  Motor_StopSimple();
}

//**********************************************************



