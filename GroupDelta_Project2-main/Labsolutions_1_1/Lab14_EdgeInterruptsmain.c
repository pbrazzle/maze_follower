// Lab14_EdgeInterruptsmain.c
// Runs on MSP432, interrupt version
// Main test program for interrupt driven bump switches the robot.
// Daniel Valvano and Jonathan Valvano
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

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/BumpInt.h"
#include "../inc/TExaS.h"
#include "../inc/TimerA1.h"
#include "../inc/FlashProgram.h"

uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
}
int mainreal(void){
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
  CollisionFlag = 0;
  Motor_Init();        // activate Lab 13 software
  Motor_Forward(7500,7500); // 50%
  BumpInt_Init(&HandleCollision);
  TExaS_Init(LOGICANALYZER_P4_765320);
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
  }
}
//1.  Back Up slowly for 1 second
//2.  Turn Right slowly for 5 seconds (90 degrees)
//3.  Go Forward quickly for 1 minute (infinite time)
//4.  Repeat steps
typedef struct command{
    uint16_t RightPWM; // 0 to 14998
    uint16_t LeftPWM;  // 0 to 14998
    void (*MotorFunction)(uint16_t,uint16_t);
    uint32_t Duration; // time to run in ms
}command_t;

#define NUM 6
const command_t Rule[NUM]={
{   200,  200,&Motor_Backward,  500}, // stopped
{  2000, 2000,&Motor_Backward, 2000},
{  2000, 2000,&Motor_Right,    5000},
{ 10000,10000,&Motor_Forward,  5000},
{  4000, 8000,&Motor_Forward, 10000},
{  8000, 4000,&Motor_Forward, 10000}
};
uint32_t Step; // 0, 1, 2...,NUM-1
uint32_t Time; // in ms
void Interpreter(void){ // runs every 1ms
  Time++;
  if(Time > Rule[Step].Duration){
     Step = (Step+1)%NUM;
     Rule[Step].MotorFunction(Rule[Step].LeftPWM,Rule[Step].RightPWM);
     Time = 0;
  }
}
void Collision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   Step = 0;
   Rule[Step].MotorFunction(Rule[Step].LeftPWM,Rule[Step].RightPWM);
   Time = 0;
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

int main(void){//main2(void){
  DisableInterrupts();
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
  LaunchPad_Init();
  if(LaunchPad_Input()){
    LaunchPad_Output(RED);
    Debug_FlashInit(); // erase flash if either switch pressed
    while(LaunchPad_Input()){}; // wait for release
  }
  CollisionFlag = 0;
  Motor_Init();        // activate Lab 13 software
  BumpInt_Init(&Collision);
  TExaS_Init(LOGICANALYZER_P4_765320);
  TimerA1_Init(&Interpreter,500);
  Step = 2;
  Rule[Step].MotorFunction(Rule[Step].RightPWM,Rule[Step].LeftPWM);
  Time = 0;
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
  }
}
