/*
 * Lab17.c
   Lab 17 solution, Distance to wall proportional control
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

static enum outputtype OutputType17 = UART;

//******************************************************


volatile uint32_t nr,nc,nl;
volatile uint32_t ADCflag; // Set every 500us on ADC sample
volatile uint32_t ControllerFlag; // set every 10ms on controller execution
int32_t UR, UL;  // PWM duty 0 to 14,998
int32_t Left17,Center17,Right17; // IR distances in mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
int32_t Ki=10;  // integral controller gain
int32_t Kp=32;  // proportional controller gain

void IRsampling(void){  // runs at 2000 Hz
  uint32_t raw17,raw14,raw16;
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw14); // center is channel 14, P6.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  Left17 = LeftConvert(nl);
  Center17 = CenterConvert(nc);
  Right17 = RightConvert(nr);
  ADCflag = 1;           // semaphore
}
void LCDClear1(void){
  if(OutputType17 == LCD){
    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("17: control");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("Autonomous");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString(" driving");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("Hit bumper");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString(" to select");
    Nokia5110_SetCursor(0,5); Nokia5110_OutString(" base PWM");
  }else if(OutputType17 == OLED){
    SSD1306_Clear(); // erase entire display
    SSD1306_SetCursor(0,0); SSD1306_OutString("Lab 17: control");
    SSD1306_SetCursor(0,1); SSD1306_OutString("Autonomous driving");
    SSD1306_SetCursor(0,2); SSD1306_OutString("Hit bumper to");
    SSD1306_SetCursor(0,3); SSD1306_OutString("select base PWM");
  }else if(OutputType17 == UART){
    UART0_Initprintf();
    printf("\n\rLab 17 wall follow proportional controller\n\r");
    printf(" L(mm), C(mm), R(mm), E(mm)\n\r");
  }
}
void LCDOut1(void){
#define USIZE 100
  static uint32_t Rsum=0,Csum=0,Lsum=0,I=0;
  static int32_t Esum=0;
  Lsum +=Left17;
  Csum +=Center17;
  Rsum +=Right17;
  Esum +=Error;
  I = I+1;

  if(OutputType17 == LCD){
//    Nokia5110_SetCursor(5,1); Nokia5110_OutUDec(PWMnominal);
    Nokia5110_SetCursor(3,2); Nokia5110_OutSDec(Left17);
    Nokia5110_SetCursor(3,3); Nokia5110_OutSDec(Center17);
    Nokia5110_SetCursor(3,4); Nokia5110_OutSDec(Right17);
    Nokia5110_SetCursor(3,5); Nokia5110_OutSDec(Error);
  }else if(OutputType17 == OLED){
//    SSD1306_SetCursor(5,1); SSD1306_OutUDec(PWMnominal);
    SSD1306_SetCursor(3,2); SSD1306_OutSDec(Left17);
    SSD1306_SetCursor(3,3); SSD1306_OutSDec(Center17);
    SSD1306_SetCursor(3,4); SSD1306_OutSDec(Right17);
    SSD1306_SetCursor(3,5); SSD1306_OutSDec(Error);
  }else if(OutputType17 == UART){
    if(I==USIZE){
     printf("%5d, %5d, %5d, %5d\n\r",Lsum/USIZE,Csum/USIZE,Rsum/USIZE,Esum/USIZE);
     Rsum=0,Csum=0,Lsum=0,Esum=0,I=0;
    }
  }
}
#define TOOCLOSE 200
#define DESIRED 250
int32_t SetPoint = 250;
#define TOOFAR 400

int32_t PWMnominal=2500;
#define SWING 1000
#define PWMMIN (PWMnominal-SWING)
#define PWMMAX (PWMnominal+SWING)
void SysTick_Handler(void){ // runs at 100 Hz
  if(Mode){
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
    Motor_Forward(UL,UR);
    ControllerFlag = 1;
  }
}
void Pause17(void){int i;
  Blinker_Output(BK_RGHT+BK_LEFT);
  LCDClear1();
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(200); LaunchPad_Output(0); // off
    Clock_Delay1ms(200); LaunchPad_Output(1); // red
  }
  while(Bump_Read()==0){// wait for touch
    Clock_Delay1ms(100);
    LaunchPad_Output(0); // off
    Blinker_Output(0);
    Clock_Delay1ms(100);
    LaunchPad_Output(3); // red/green
    Blinker_Output(BK_RGHT+BK_LEFT);
  }
  uint8_t in = Bump_Read();
  PWMnominal = 2500;
  while((in&0x01)==0){ // 2500, 3000, 3500, 4000, 4500, 5000
    PWMnominal += 500;
    in = in>>1;
  }
  if(OutputType17 == LCD){
    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("17: control");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("PWM= "); Nokia5110_OutUDec(PWMnominal);
  }else if(OutputType17 == OLED){
    SSD1306_Clear(); // erase entire display
    SSD1306_OutString("Lab 17: control");
    SSD1306_SetCursor(0,1); SSD1306_OutString("PWM= "); SSD1306_OutUDec(PWMnominal);
  }else if(OutputType17 == UART){
    UART0_OutString("\r\n17: control\r\n");
    UART0_OutString("PWM= ");
    UART0_OutUDec(PWMnominal);
    UART0_OutString("\r\n");
  }

  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(100);
    LaunchPad_Output(0); // off
    Blinker_Output(0);
    Clock_Delay1ms(100);
    LaunchPad_Output(4); // blue
    Blinker_Output(FR_RGHT+FR_LEFT);
  }
  for(i=500;i>100;i=i-100){
    Clock_Delay1ms(i); LaunchPad_Output(0); // off
    Clock_Delay1ms(i); LaunchPad_Output(2); // green
  }
  if(OutputType17 == LCD){
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("L= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("C= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("R= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
    Nokia5110_SetCursor(0,5); Nokia5110_OutString("E= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  }else if(OutputType17 == OLED){
    SSD1306_SetCursor(0,2); SSD1306_OutString("L= "); SSD1306_OutSDec(0); SSD1306_OutString(" mm");
    SSD1306_SetCursor(0,3); SSD1306_OutString("C= "); SSD1306_OutSDec(0); SSD1306_OutString(" mm");
    SSD1306_SetCursor(0,4); SSD1306_OutString("R= "); SSD1306_OutSDec(0); SSD1306_OutString(" mm");
    SSD1306_SetCursor(0,5); SSD1306_OutString("E= "); SSD1306_OutSDec(0); SSD1306_OutString(" mm");
  }else if(OutputType17 == UART){
    UART0_OutString("L= "); UART0_OutUDec(0); UART0_OutString(" mm\r\n");
    UART0_OutString("C= "); UART0_OutUDec(0); UART0_OutString(" mm\r\n");
    UART0_OutString("R= "); UART0_OutUDec(0); UART0_OutString(" mm\r\n");
    UART0_OutString("E= "); UART0_OutUDec(0); UART0_OutString(" mm\r\n");
  }
  // restart Jacki
  UR = UL = PWMnominal;    // reset parameters
  Mode = 1;
  ControllerFlag = 0;
}

// *******************************************************************
// main17.c
// Lab 17 solution with proportional control, distance to wall
// Enumerated parameter "NONE", "LCD", "UART" or "OLED"
// Runs on MSP432
void main17(enum outputtype outputType){//mainProportialControl(void){
  uint32_t raw17,raw14,raw16;
  OutputType17 = outputType;
  DisableInterrupts();
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  Bump_Init();   // bump switches
  TimerA1_Init(&IRsampling,250);    // 2000 Hz sampling
  Motor_Stop();
  Mode = 0;
  UR = UL = PWMnominal; //initial power
  ADCflag = ControllerFlag = 0; // semaphores

// prime the filters with some samples
  ADC0_InitSWTriggerCh17_14_16();   // initialize channels 17,12,16
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  LPF_Init(raw17,64);     // P9.0/channel 17
  LPF_Init2(raw14,64);    // P6.1/channel 14
  LPF_Init3(raw16,64);    // P9.1/channel 16
  SysTick_Init(480000,2); // 100 Hz
  if((OutputType17 == NONE) || (OutputType17 == LCD) || (OutputType17 == OLED)){
    TExaS_Init(LOGICANALYZER_P27_P26_P82_P92_P104_P105);
    // You can have logic analyzer or UART debugging (not both)
  }
  Pause17();
  EnableInterrupts();
  while(1){
    if(Bump_Read()){ // collision
      Mode = 0;
      Motor_Stop();
      Pause17();
    }
    if(ControllerFlag){ // 100 Hz , not real time
      LCDOut1();
      ControllerFlag = 0;
    }
  }
}
