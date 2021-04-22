// Lab10_Debugmain.c
// Runs on MSP432
// Solution to Debug lab
// Daniel and Jonathan Valvano
// September 4, 2017
// Interrupt interface for QTRX reflectance sensor array
// Pololu part number 3672.
// Debugging dump, and Flash black box recorder

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

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"
/*

void Debug_Init(void){
  // write this as part of Lab 10
}
void Debug_Dump(uint8_t x, uint8_t y){
  // write this as part of Lab 10
}
void Debug_FlashInit(void){ 
  // write this as part of Lab 10
}
void Debug_FlashRecord(uint16_t *pt){
  // write this as part of Lab 10
}
void SysTick_Handler(void){ // every 1ms
  // write this as part of Lab 10
}

int main(void){
  // write this as part of Lab 10

  while(1){
  // write this as part of Lab 10

  }
}
*/
volatile uint8_t Data; // QTR-8RC
volatile uint8_t Bump; // 6 bump sensors
volatile uint32_t Time;
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define SIZE 256  // must be a power of 2
uint16_t Buffer[SIZE];
uint32_t I,J; // I is RAM index, J is ROM index
void Debug_Init(void){
  I=0;
}
void Debug_Dump(uint8_t x, uint8_t y){
  Buffer[I]=(x<<8)|y;
  I = (I+1)&(SIZE-1);
}

#define FLASH_BANK1_MIN     0x00020000  // Flash Bank1 minimum address
#define FLASH_BANK1_MAX     0x0003FFFF  // Flash Bank1 maximum address
void Debug_FlashInit(void){ uint32_t addr;
  Flash_Init(48);
  J = 0;
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
void Debug_FlashRecord(uint16_t *pt){uint32_t addr;
  addr=FLASH_BANK1_MIN;
  while(*(uint32_t*)addr != 0xFFFFFFFF){ // find first free block
    addr=addr+64;
    if(addr>FLASH_BANK1_MAX)return; // full
  }
  Flash_FastWrite((uint32_t *)pt, addr, 16); // 16 words is 32 halfwords, 64 bytes
}
uint8_t dummydata=0;
void SysTick_Handler(void){ // every 1ms
  LEDOUT ^= 0x01;       // toggle P1.0
  LEDOUT ^= 0x01;       // toggle P1.0
  Time = Time + 1;
  if(Time%10==1){
    Reflectance_Start(); // start every 10ms
  }
  if(Time%10==2){
    Data = Reflectance_End(); // finish 1ms later
    Bump = Bump_Read();
    Debug_Dump(Data,Bump);
//    Debug_Dump(dummydata,dummydata+1);dummydata=dummydata+2; // linear sequence
  }
  LEDOUT ^= 0x01;       // toggle P1.0
}


int main(void){
  Clock_Init48MHz();     // running on crystal
  Time = 0;
  LaunchPad_Init();      // P1.0 is red LED on LaunchPad
  Reflectance_Init();
  Bump_Init();
  Debug_Init();
  SysTick_Init(48000,2); // set up SysTick for 1kHz interrupts
  if(LaunchPad_Input()==0x01){
    Debug_FlashInit();   // erase Flash if a switch SW1 is pressed
  }
  while((LaunchPad_Input()&0x02)==0x00){ // wait for SW2
    LEDOUT^= 0x01;       // toggle P1.0
    Clock_Delay1ms(100);
  }
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
    P2->OUT ^= 0x01; // foreground thread
    if(((J+32)&0xFF)==I){ // time to write another buffer
      Debug_FlashRecord(&Buffer[J]); // 114us
      J = (J+32)&(SIZE-1);
    }
  }
}
int Program10_1(void){ uint8_t data=0;
  Clock_Init48MHz();
  Debug_Init();
  LaunchPad_Init();
  while(1){
    P1->OUT |= 0x01;
    Debug_Dump(data,data+1);// linear sequence
    P1->OUT &= ~0x01;
    data=data+2;
  }
}


// Driver test
#define SIZE 256  // feel free to adjust the size
uint16_t Buffer[SIZE];
int Program10_2(void){ uint16_t i;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  for(i=0;i<SIZE;i++){
    Buffer[i] = (i<<8)+(255-i); // test data
  }
  i = 0;
  while(1){
    P1->OUT |= 0x01;
    Debug_FlashInit();
    P1->OUT &= ~0x01;
    P2->OUT |= 0x01;
    Debug_FlashRecord(Buffer); // 114us
    P2->OUT &= ~0x01;
    i++;
  }
}


int Program10_3(void){ uint16_t i;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  for(i=0;i<SIZE;i++){
    Buffer[i] = (i<<8)+(255-i); // test data
  }
  P1->OUT |= 0x01;
  Debug_FlashInit();
  P1->OUT &= ~0x01;
  i = 0;
  while(1){
    P2->OUT |= 0x01;
    Debug_FlashRecord(Buffer);
    P2->OUT &= ~0x01;
    i++;
  }
}

/*
uint8_t Buffer[1000];
uint32_t I=0;
uint8_t *pt;
void DumpI(uint8_t x){
  if(I<1000){
    Buffer[I]=x;
    I++;
  }
}
void DumpPt(uint8_t x){
  if(pt<&Buffer[1000]){
    *pt=x;
    pt++;
  }
}
void Activity(void){
  DumpI(5);
  DumpI(6);
  pt = Buffer;
  DumpPt(7);
  DumpPt(8);

}
*/
