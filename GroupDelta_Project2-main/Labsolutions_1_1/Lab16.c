/*
 * Lab16.c
 * Solution to tach input, constant speed controller
 *
 *  Created on: June 4, 2018
 *      Authors: Jonathan Valvano and Daniel Valvano
 *  Modified: January 22, 2020
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
// *******************************************************************
// main16.c
// Enumerated parameter "NONE", "LCD", "UART" or "OLED"
// Runs on MSP432
void main16(enum outputtype outputType){
  int i = 0;
  if(outputType == LCD){
    Nokia5110_Clear();
    Nokia5110_OutString("Lab16 Tach");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("Set points");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("SW1 right");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("SW2 left");
  }else if(outputType == OLED){
    SSD1306_Clear();
    SSD1306_OutString("Lab16 Tach");
    SSD1306_SetCursor(0,2); SSD1306_OutString("Set points");
    SSD1306_SetCursor(0,3); SSD1306_OutString("SW1 right");
    SSD1306_SetCursor(0,4); SSD1306_OutString("SW2 left");
  }else if(outputType == UART){
    UART0_OutString("Lab16 Tach\r\n\r\n");
    UART0_OutString("Set points\r\n");
    UART0_OutString("SW1 right\r\n");
    UART0_OutString("SW2 left\r\n\r\n");
  }
  Blinker_Output(BK_RGHT+BK_LEFT);

  if(outputType == LCD){
    // this delay leaves instructions on the screen for 4 seconds
    for(i=4;i>0;i--){
      Nokia5110_SetCursor(0,5); Nokia5110_OutUDec16(i);
      Clock_Delay1ms(1000);              // delay 4 sec at 48 MHz
      if(i&1){
        Blinker_Output(BK_RGHT+BK_LEFT);
      }else{
        Blinker_Output(0);
      }
    }
  }else if(outputType == OLED){
      // this delay leaves instructions on the screen for 4 seconds
    for(i=4;i>0;i--){
    //  SSD1306_SetCursor(0,5); SSD1306_OutUDec16(i);
    //  SSD1306_Display();
      Clock_Delay1ms(1000);              // delay 4 sec at 48 MHz
      if(i&1){
        Blinker_Output(BK_RGHT+BK_LEFT);
      }else{
        Blinker_Output(0);
      }
    }
  }else if(outputType == UART){
    // this delay is unnecessary
/*    for(i=4;i>0;i--){
      UART0_OutChar(' ');
      UART0_OutChar(' ');
      UART0_OutChar(' ');
      UART0_OutUDec((uint32_t)i);
      UART0_OutChar(CR); UART0_OutChar(LF);
      Clock_Delay1ms(1000);              // delay 4 sec at 48 MHz
      if(i&1){
        Blinker_Output(BK_RGHT+BK_LEFT);
      }else{
        Blinker_Output(0);
      }
    }*/
  }

  Tachometer_Init();
  Motor_Init();
  if((outputType == NONE) || (outputType == LCD)|| (outputType == OLED)){
    TExaS_Init(LOGICANALYZER_P27_P26_P82_P92_P104_P105);
    // You can have logic analyzer or UART debugging (not both)
  }
  EnableInterrupts();
  Blinker_Output(BK_RGHT+BK_LEFT);
  while(1){
    Motor_Stop();
    if(outputType == LCD){
      Nokia5110_Clear();
      Nokia5110_OutString("Desired(RPM)L     R");
      Nokia5110_SetCursor(0,4); Nokia5110_OutString("Hit bumper");
      Nokia5110_SetCursor(0,5); Nokia5110_OutString(" to start");
      Nokia5110_SetCursor(1, 1);         // one leading space, second row
      Nokia5110_OutUDec16(DesiredL);
      Nokia5110_SetCursor(7, 1);         // seven leading spaces, second row
      Nokia5110_OutUDec16(DesiredR);
    } else if(outputType == OLED){
      SSD1306_Clear();
      SSD1306_SetCursor(0, 0); SSD1306_OutString("Incremental control");
      SSD1306_SetCursor(0, 1); SSD1306_OutString("Desired L="); SSD1306_OutUDec16(DesiredL);
      SSD1306_SetCursor(0, 2); SSD1306_OutString("Desired R="); SSD1306_OutUDec16(DesiredR);
      SSD1306_SetCursor(0,4); SSD1306_OutString("Hit bumper");
      SSD1306_SetCursor(0,5); SSD1306_OutString(" to start");
    }else if(outputType == UART){
      UART0_OutString("Hit bumper to start\r\n\r\n");
      UART0_OutString("Desired(RPM)L");
      UART0_OutChar(','); // comma
      UART0_OutString("Desired(RPM)R\r\n");
      UART0_OutUDec((uint32_t)DesiredL);
      UART0_OutChar(','); // comma
      UART0_OutUDec((uint32_t)DesiredR);
      UART0_OutChar(CR); UART0_OutChar(LF); // carriage return
    }
    while(Bump_Read() == 0){
      // update the screen
      if((LaunchPad_Input()&0x01) != 0x00){
        // Button1 has been pressed
        DesiredR = DesiredR + 10;
        if(DesiredR > DESIREDMAX){
          DesiredR = DESIREDMIN;
        }
        if(outputType == LCD){
          Nokia5110_SetCursor(1, 1);     // one leading space, second row
          Nokia5110_OutUDec16(DesiredL);
          Nokia5110_SetCursor(7, 1);     // seven leading spaces, second row
          Nokia5110_OutUDec16(DesiredR);
        }else if(outputType == OLED){
          SSD1306_SetCursor(0, 1); SSD1306_OutString("Desired L="); SSD1306_OutUDec16(DesiredL);
          SSD1306_SetCursor(0, 2); SSD1306_OutString("Desired R="); SSD1306_OutUDec16(DesiredR);
        }else if(outputType == UART){
          UART0_OutUDec((uint32_t)DesiredL);
          UART0_OutChar(','); // comma
          UART0_OutUDec((uint32_t)DesiredR);
          UART0_OutChar(CR); UART0_OutChar(LF); // carriage return
        }
        while(LaunchPad_Input()){}; // wait for release
      }
      if((LaunchPad_Input()&0x02) != 0x00){
        // Button2 has been pressed
        DesiredL = DesiredL + 10;
        if(DesiredL > DESIREDMAX){
          DesiredL = DESIREDMIN;
        }
        if(outputType == LCD){
          Nokia5110_SetCursor(1, 1);     // one leading space, second row
          Nokia5110_OutUDec16(DesiredL);
          Nokia5110_SetCursor(7, 1);     // seven leading spaces, second row
          Nokia5110_OutUDec16(DesiredR);
        }else if(outputType == OLED){
          SSD1306_SetCursor(0, 1); SSD1306_OutString("Desired L="); SSD1306_OutUDec16(DesiredL);
          SSD1306_SetCursor(0, 2); SSD1306_OutString("Desired R="); SSD1306_OutUDec16(DesiredR);
        }else if(outputType == UART){
          UART0_OutUDec((uint32_t)DesiredL);
          UART0_OutChar(','); // comma
          UART0_OutUDec((uint32_t)DesiredR);
          UART0_OutChar(CR); UART0_OutChar(LF); // carriage return
        }
        while(LaunchPad_Input()){}; // wait for release
      }
      // flash the blue LED
      i = i + 1;
      LaunchPad_Output((i&0x01)<<2);
      Clock_Delay1ms(200);               // delay ~0.2 sec at 48 MHz
    }
    if(outputType == LCD){
      Nokia5110_Clear();
      Nokia5110_OutString("Desired(RPM)L     R     Actual (RPM)L     R     Distance(mm)");
      Nokia5110_SetCursor(1, 1);         // one leading space, second row
      Nokia5110_OutUDec16(DesiredL);
      Nokia5110_SetCursor(7, 1);         // seven leading spaces, second row
      Nokia5110_OutUDec16(DesiredR);
    }else if(outputType == OLED){
      SSD1306_Clear();
      SSD1306_SetCursor(0, 0); SSD1306_OutString("Incremental control");
      SSD1306_SetCursor(0, 1); SSD1306_OutString("Desired L="); SSD1306_OutUDec16(DesiredL);
      SSD1306_SetCursor(0, 2); SSD1306_OutString("Desired R="); SSD1306_OutUDec16(DesiredR);
      SSD1306_SetCursor(0, 3); SSD1306_OutString("Actual L=");
      SSD1306_SetCursor(0, 4); SSD1306_OutString("Actual R=");
      SSD1306_SetCursor(0, 5); SSD1306_OutString("Left Dist=");
      SSD1306_SetCursor(0, 6); SSD1306_OutString("Rght Dist=");
    }else if(outputType == UART){
      UART0_OutString("\r\n\r\nDesired(RPM)L");
      UART0_OutChar(','); // comma
      UART0_OutString("Desired(RPM)R");
      UART0_OutChar(','); // comma
      UART0_OutString("Actual(RPM)L");
      UART0_OutChar(','); // comma
      UART0_OutString("Actual(RPM)R");
      UART0_OutChar(','); // comma
      UART0_OutString("Duty Cycles L");
      UART0_OutChar(','); // comma
      UART0_OutString("Duty Cycles R");
      UART0_OutChar(','); // comma
      UART0_OutString("Distance(mm)L");
      UART0_OutChar(','); // comma
      UART0_OutString("Distance(mm)R\r\n");
    }
    for(i=0; i<4; i=i+1){
      // flash the yellow LED
      Blinker_Output(FR_RGHT+FR_LEFT);
      LaunchPad_Output(0x03);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
      Blinker_Output(0);
      LaunchPad_Output(0x00);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }
    LaunchPad_Output(0x02);
    i = 0;
    Blinker_Output(FR_RGHT+FR_LEFT);

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
        if(outputType == LCD){
          Nokia5110_SetCursor(1, 3);     // one leading space, fourth row
          Nokia5110_OutUDec16(ActualL);
          Nokia5110_SetCursor(7, 3);     // seven leading spaces, fourth row
          Nokia5110_OutUDec16(ActualR);
          Nokia5110_SetCursor(0, 5);     // zero leading spaces, sixth row
          if(LeftSteps < 0){
            Nokia5110_OutChar('-');
            Nokia5110_OutUDec((-1*LeftSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }else{
            Nokia5110_OutChar(' ');
            Nokia5110_OutUDec((LeftSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }
          Nokia5110_SetCursor(6, 5);     // six leading spaces, sixth row
          if(RightSteps < 0){
            Nokia5110_OutChar('-');
            Nokia5110_OutUDec((-1*RightSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }else{
            Nokia5110_OutChar(' ');
            Nokia5110_OutUDec((RightSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }
        }else if(outputType == OLED){
          SSD1306_SetCursor(9, 3); SSD1306_OutUDec16(ActualL);
          SSD1306_SetCursor(9, 4); SSD1306_OutUDec16(ActualR);
          SSD1306_SetCursor(10, 5);
          if(LeftSteps < 0){
            SSD1306_OutChar('-');
            SSD1306_OutUDec((-1*LeftSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }else{
            SSD1306_OutChar(' ');
            SSD1306_OutUDec((LeftSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }
          SSD1306_SetCursor(10, 6);     // six leading spaces, sixth row
          if(RightSteps < 0){
            SSD1306_OutChar('-');
            SSD1306_OutUDec((-1*RightSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }else{
            SSD1306_OutChar(' ');
            SSD1306_OutUDec((RightSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }
        }else if(outputType == UART){
          UART0_OutUDec((uint32_t)DesiredL);
          UART0_OutChar(','); // comma
          UART0_OutUDec((uint32_t)DesiredR);
          UART0_OutChar(','); // comma
          UART0_OutUDec((uint32_t)ActualL);
          UART0_OutChar(','); // comma
          UART0_OutUDec((uint32_t)ActualR);
          UART0_OutChar(','); // comma
          UART0_OutUDec((uint32_t)LeftDuty);
          UART0_OutChar(','); // comma
          UART0_OutUDec((uint32_t)RightDuty);
          UART0_OutChar(','); // comma
          if(LeftSteps < 0){
            UART0_OutChar('-');
            UART0_OutUDec((-1*LeftSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }else{
            UART0_OutUDec((LeftSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }
          UART0_OutChar(','); // comma
          if(RightSteps < 0){
            UART0_OutChar('-');
            UART0_OutUDec((-1*RightSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }else{
            UART0_OutUDec((RightSteps*220)/360);// 70mm diameter wheel; ~220mm circumference divided by 360 steps
          }
          UART0_OutChar(CR); UART0_OutChar(LF); // carriage return
        }
      }
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }
    Motor_Stop();
    i = 0;
    Blinker_Output(BK_RGHT+BK_LEFT);
    while(Bump_Read() != 0){
      // flash the red LED
      i = i + 1;
      LaunchPad_Output(i&0x01);
      Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
    }
  }
}
