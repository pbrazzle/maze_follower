/*
 * Lab7.c
 * solution to FSM line following
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
// *******************************************************************
// Lab07_FSMmain.c
// Runs on MSP432
/*(Left,Right) Motors, call LaunchPad_Output (positive logic)
3   1,1     both motors, yellow means go straight
2   1,0     left motor,  green  means turns right
1   0,1     right motor, red    means turn left
0   0,0     both off,    dark   means stop
(Left,Right) Sensors, call LaunchPad_Input (positive logic)
3   1,1     both buttons pushed means on line,
2   1,0     SW2 pushed          means off to right
1   0,1     SW1 pushed          means off to left
0   0,0     neither button      means lost
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

#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
#include "../inc/Nokia5110.h"
#include "../inc/MotorSimple.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "../inc/UART0.h"
#include "../inc/IRDistance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/LPF.h"
#include "../inc/SysTickInts.h"
#include "../inc/ADC14.h"
#include "../inc/SSD1306.h"

// Linked data structure
struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
  char name[10];
  uint32_t blink;              // blinker outputs
};
typedef const struct State State_t;

#define Center    &fsm[0]
#define Left1     &fsm[1]
#define Left2     &fsm[2]
#define WayLeft1  &fsm[3]
#define WayLeft2  &fsm[4]
#define LostLeft  &fsm[5]
#define Right1    &fsm[6]
#define Right2    &fsm[7]
#define WayRight1 &fsm[8]
#define WayRight2 &fsm[9]
#define LostRight &fsm[10]

// solution
State_t fsm[11]={
  {0x03, 100, { WayRight1, Left1,  Right1, Center }, "Center   ",FR_RGHT+FR_LEFT}, // Center
  {0x02,  50, { WayLeft1,  Left2,  Right1, Left2  }, "Left1    ",FR_RGHT}, // Left1
  {0x03,  50, { Left1,     Left1,  Right2, Center }, "Left2    ",0}, // Left2
  {0x02, 100, { WayLeft2,  Left2,  Right1, Left2  }, "WayLeft1 ",FR_RGHT}, // WayLeft1
  {0x03,  50, { WayLeft1,  Left2,  Left2,  Left2  }, "WayLeft2 ",0}, // WayLeft2
  {0x00,  25, { LostLeft,  Left2,  Left2,  Left2  }, "LostLeft ",BK_RGHT+BK_LEFT}, // LostLeft
  {0x01,  50, { WayRight1, Left1,  Right2, Right2 }, "Right1   ",BK_LEFT}, // Right1
  {0x03,  50, { Right1,    Left2,  Right1, Center }, "Right2   ",0}, // Right2
  {0x01, 100, { WayRight2, Left1,  Right2, Right2 }, "WayRight1",BK_LEFT}, // WayRight1
  {0x03,  50, { WayRight1, Right2, Right2, Right2 }, "WayRight2",0}, // WayRight2
  {0x00,  25, { LostRight, Right2, Right2, Right2 }, "LostRight",BK_RGHT+BK_LEFT}  // LostRight
};


State_t *Spt7;  // pointer to the current state
uint32_t Input7;
uint32_t Output7;
uint16_t Left7,Right7;
uint16_t SpeedL7,SpeedR7=0;
uint8_t Sensor7;
int32_t Position7;

/*Run FSM continuously
1) Output depends on State (Motors)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
void CheckBump7(void){uint8_t in; static uint8_t last=0;
int i;
  if(Bump_Read()==0){
    in = LaunchPad_Input();
    if(((last&0x01)==0)&&(in&0x01)){ // button 1 will increase right PWM
      if(SpeedR7<5000){
        SpeedR7 += 500;
      }else{
        SpeedR7 = 2000;
      }
    }
    if(((last&0x02)==0)&&(in&0x02)){// button 2 will increase left PWM
      if(SpeedL7<5000){
        SpeedL7 += 500;
      }else{
        SpeedL7 = 2000;
      }
    }
    last = in;
    return; // no collision
  }
  Blinker_Output(BK_RGHT+BK_LEFT);
  Motor_Stop();       // stop
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
}

// *******************************************************************
// Lab07_FSMmain.c
// Enumerated parameter "NONE", "LCD", "OLED" or "UART"
// Runs on MSP432
void main7(enum outputtype outputType){ uint32_t heart=0;
  if((outputType == NONE) || (outputType == LCD) || (outputType == OLED)){
    TExaS_Init(LOGICANALYZER);  // You can have logic analyzer or UART debugging (not both)
  }
  Spt7 = Center;
  Motor_Stop();       // initialize and stop
  Reflectance_Init();
  EnableInterrupts();
//  last = LaunchPad_Input();
  SpeedR7 = SpeedL7 =2000;
  if(outputType == LCD){
    Nokia5110_Clear();
    Nokia5110_OutString("Lab 7 FSM");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("Line follow");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("I="); Nokia5110_OutHex7(Input7); Nokia5110_OutString(", O="); Nokia5110_OutHex7(Output7);
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("IR =");
    Nokia5110_SetCursor(0,5); Nokia5110_OutString("Pos=");
  }else if(outputType == OLED){
    SSD1306_Clear();
    SSD1306_OutString("Lab 7 FSM");
    SSD1306_SetCursor(0,1); SSD1306_OutString("Line follow");
    SSD1306_SetCursor(0,2); SSD1306_OutString("I="); SSD1306_OutHex7(Input7); SSD1306_OutString(", O="); SSD1306_OutHex7(Output7);
    SSD1306_SetCursor(0,4); SSD1306_OutString("IR =");
    SSD1306_SetCursor(0,5); SSD1306_OutString("Pos=");
    SSD1306_SetCursor(0,6); SSD1306_OutString("Button 1 -> right PWM");
    SSD1306_SetCursor(0,7); SSD1306_OutString("Button 2 -> left PWM");
  }else if(outputType == UART){
    UART0_OutString("Lab 7 FSM\r\n");
    UART0_OutString("Line follow\r\n");
    UART0_OutString("I"); UART0_OutChar(9); // tab
    UART0_OutString("O"); UART0_OutChar(9); // tab
    UART0_OutString("State"); UART0_OutChar(9); // tab
    UART0_OutString("IR"); UART0_OutChar(9); // tab
    UART0_OutString("Pos\r\n");
  }
  while(1){
    Output7 = Spt7->out;            // set output from FSM
    if(Output7&2){
      Left7 = SpeedL7;
    }else{
      Left7 = 10;  // stopped
    }
    if(Output7&1){
      Right7 = SpeedR7;
    }else{
      Right7 = 10;  // stopped
    }
    Blinker_Output(Spt7->blink);
    Motor_Forward(Left7,Right7);
    if(outputType == LCD){
      TExaS_Set(Input7<<2|Output7);   // optional, send data to logic analyzer
      Nokia5110_SetCursor(2,2); Nokia5110_OutHex7(Input7); Nokia5110_OutString(", O="); Nokia5110_OutHex7(Output7);
      Nokia5110_SetCursor(0,3); Nokia5110_OutString((char *)Spt7->name);
      Nokia5110_SetCursor(4,4); Nokia5110_OutUHex7(Sensor7);
      Nokia5110_SetCursor(4,5); Nokia5110_OutSDec(Position7);
    } else if(outputType == OLED){
      TExaS_Set(Input7<<2|Output7);   // optional, send data to logic analyzer
      SSD1306_SetCursor(2,2); SSD1306_OutHex7(Input7); SSD1306_OutString(", O="); SSD1306_OutHex7(Output7);
      SSD1306_SetCursor(0,3); SSD1306_OutString((char *)Spt7->name);
      SSD1306_SetCursor(4,4); SSD1306_OutUHex7(Sensor7);
      SSD1306_SetCursor(4,5); SSD1306_OutSDec(Position7);
    }else if(outputType == UART){
      UART0_OutUHex(Input7); UART0_OutChar(9); // Input7 -> tab
      UART0_OutUHex(Output7); UART0_OutChar(9); // Output7 -> tab
      UART0_OutString((char *)Spt7->name); UART0_OutChar(9); // state name -> tab
      UART0_OutUHex(Sensor7); UART0_OutChar(9); // Sensor7 -> tab
      if(Position7 < 0){
        UART0_OutChar('-'); // crude signed output
        UART0_OutUDec(-1*Position7); UART0_OutChar(CR); // Position7 -> carriage return
      }else{
        UART0_OutUDec(Position7); UART0_OutChar(CR); // Position7 -> carriage return
      }
    }
    Clock_Delay1ms(Spt7->delay);   // wait
    Sensor7 = Reflectance_Read(1000);
    Position7 = Reflectance_Position(Sensor7);
    Input7 = (Sensor7&0x18)>>3;    // read sensors 3,4
    Spt7 = Spt7->next[Input7];       // next depends on input and state
    heart++;
    LaunchPad_LED(heart>>4);         // optional, debugging heartbeat
    CheckBump7();
  }
}
