// Lab18_UARTmain.c
// Runs on MSP432
// Lab 18 solution
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

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
#include "..\inc\Bump.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"

// At 115200, the bandwidth = 11,520 characters/sec
// 86.8 us/character
// normally one would expect it to take 31*86.8us = 2.6ms to output 31 characters
// Random number generator
// from Numerical Recipes
// by Press et al.
// number from 0 to 31
uint32_t Random(void){
static uint32_t M=1;
  M = 1664525*M+1013904223;
  return(M>>27);
}
char WriteData,ReadData;
uint32_t NumSuccess,NumErrors;
void TestFifo(void){char data;
  while(TxFifo0_Get(&data)==FIFOSUCCESS){
    if(ReadData==data){
      ReadData = (ReadData+1)&0x7F; // 0 to 127 in sequence
      NumSuccess++;
    }else{
      ReadData = data; // restart
      NumErrors++;
    }
  }
}
uint32_t Size;
int main(void){ //Program18_1(void){
    // test of TxFifo0, NumErrors should be zero
  uint32_t i;
  Clock_Init48MHz();
  WriteData = ReadData = 0;
  NumSuccess = NumErrors = 0;
  TxFifo0_Init();
  TimerA1_Init(&TestFifo,43);  // 83us, = 12kHz
  EnableInterrupts();
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      TxFifo0_Put(WriteData);
      WriteData = (WriteData+1)&0x7F; // 0 to 127 in sequence
    }
    Clock_Delay1ms(10);
  }
}

char String[64];
uint32_t MaxTime,First,Elapsed;
int Program18_2(void){
    // measurement of busy-wait version of OutString
  uint32_t i;
  DisableInterrupts();
  Clock_Init48MHz();
  UART0_Init();
  WriteData = 'a';
  SysTick_Init();
  MaxTime = 0;
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      String[i] = WriteData;
      WriteData++;
      if(WriteData == 'z') WriteData = 'a';
    }
    String[i] = 0; // null termination
    First = SysTick->VAL;
    UART0_OutString(String);
    Elapsed = ((First - SysTick->VAL)&0xFFFFFF)/48; // usec

    if(Elapsed > MaxTime){
        MaxTime = Elapsed;
    }
    UART0_OutChar(CR);UART0_OutChar(LF);
    Clock_Delay1ms(100);
  }
}
int Program18_3(void){
    // measurement of interrupt-driven version of OutString
  uint32_t i;
  DisableInterrupts();
  Clock_Init48MHz();
  EUSCIA0_Init();
  WriteData = 'a';
  SysTick_Init();
  MaxTime = 0;
  EnableInterrupts();
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      String[i] = WriteData;
      WriteData++;
      if(WriteData == 'z') WriteData = 'a';
    }
    String[i] = 0; // null termination
    First = SysTick->VAL;
    EUSCIA0_OutString(String);
    Elapsed = ((First - SysTick->VAL)&0xFFFFFF)/48; // usec
    if(Elapsed > MaxTime){
        MaxTime = Elapsed;
    }
    EUSCIA0_OutChar(CR);EUSCIA0_OutChar(LF);
    Clock_Delay1ms(100);
  }
}
int Program18_4(void){
    // demonstrates features of the EUSCIA0 driver
  char ch;
  char string[20];
  uint32_t n;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
  EUSCIA0_OutString("\nLab 18 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
     EUSCIA0_OutChar(ch);
  }
  EUSCIA0_OutChar(LF);
  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
    EUSCIA0_OutChar(ch);
  }
  while(1){
    EUSCIA0_OutString("\n\rInString: ");
    EUSCIA0_InString(string,19); // user enters a string
    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
  }
}

// command line interpreter

int isWhite(char c){
  if( c == ' ' )  return 1; // space
  if( c == '\t' ) return 1; // tab
  return 0; // not space or tab
}
int isPrinting(char c){
  if( c < 0x21 ) return 0; // non printing or space
  if( c > 0x7E ) return 0; // non printing
  return 1; // between 21 and 7E
}

int isNumber(char *pt, int *n){
  char letter; int sign=1;
  int radix=10;
  if(*pt == '-'){
    sign = -1;
    pt++;
  }
  if(*pt == '$'){
    radix = 16;
    pt++;
  }
  *n = 0;  // result
  while(*pt){
    letter = *pt++;
    if(radix==16){
      if((letter >= 'a')&&(letter <= 'f')) letter = '0'+(letter-'a')+10;
      if((letter >= 'A')&&(letter <= 'F')) letter = '0'+(letter-'A')+10;
    }
    if(((letter < '0')||(letter > '9'))&&(radix==10)) return 0; // no
    if(((letter < '0')||(letter > '9'+6))&&(radix==16)) return 0; // no
    (*n) = radix*(*n) + (letter-'0');
    if(*n > 65535) return 0; // too big
  }
  (*n) = sign*(*n);
  return 1; // valid number
}

#define STACKSIZE 10
int UserStack[STACKSIZE+1]; // LIFO structure for interpreter data
int *UserSP;
void Push(int data){
  if( UserSP > UserStack ){ // got room?
    UserSP--;
    *UserSP = data;  // save data on stack
  } else{
    EUSCIA0_OutString("\n\rStack full\n\r");
  }
}
int CanPop(int num){ // is it possible to pop num values?
  if( UserSP <= &UserStack[STACKSIZE-num] ){ // got data?
    return 1;
  }
  EUSCIA0_OutString("\n\rStack empty\n\r");
  return 0;
}
int Pop(void){  int data=0;
  if( CanPop(1) ){ // got room?
    data = *UserSP;  // retrieve data on stack
    UserSP++;
  }
  return data;
}
void Add(void){ int data;
  if( CanPop(2) ){
    data = Pop();
    *UserSP = *UserSP + data;
  }
}
void Subtract(void){ int data;
  if( CanPop(2) ){
    data = Pop();
    *UserSP = *UserSP - data;
  }
}
void Multiply(void){ int data;
  if( CanPop(2) ){
    data = Pop();
    *UserSP = *UserSP * data;
  }
}
void Divide(void){ int data;
  if( CanPop(2) ){
    data = Pop();
    *UserSP = *UserSP / data;
  }
}
void PopPrintHex(void){ int data;
  if( CanPop(1) ){
    data = Pop();
    EUSCIA0_OutUHex(data);
  }
}
void PopPrintDec(void){ int data;
  if( CanPop(1) ){
    data = Pop();
    if(data<0){
      EUSCIA0_OutChar('-');
      data = -data;
    }
    EUSCIA0_OutUDec(data);
  }
}
void PopPrintUDec(void){ int data;
  if( CanPop(1) ){
    data = Pop();
    EUSCIA0_OutUDec(data);
  }
}

void PopPrintFix(void){ int data;
  if( CanPop(1) ){
    data = Pop();
    EUSCIA0_OutUFix2(data);
  }
}

void Reset(void){
  UserSP = &UserStack[STACKSIZE]; // empty stack
}
int Running; // 0 means stopped
uint32_t Time;
void CheckBumper(void){
  if(Running){
    Time++;
    if(Bump_Read()){
      Motor_Stop(); // stop on bump switch
      Running = 0;
      EUSCIA0_OutString(" Crash! Motor stopped\n\r");

    }
  }
}

void outLED(void){ // set LEDs on LaunchPad
  int value;
  if( CanPop(1) ){
    value  = Pop();          // Read value  from stack
    LaunchPad_Output(value);
  }
}
int Speed = 1000;
void setSpeed(void){
  if( CanPop(1) ){
    Speed = Pop();
    if(Speed<0)Speed = 0;
    if(Speed>14990)Speed = 14990;
    EUSCIA0_OutString(" Motor speed =");
    EUSCIA0_OutUDec(Speed);
    EUSCIA0_OutString("\n\r");
  }
}
void forwardMotor(void){
  Motor_Forward(Speed,Speed);
  Running = 1;
  EUSCIA0_OutString(" Motor forward at speed =");
  EUSCIA0_OutUDec(Speed);
  EUSCIA0_OutString("\n\r");
}
void backMotor(void){
  Motor_Backward(Speed,Speed);
  Running = 0; // don't stop on bump
  EUSCIA0_OutString(" Motor backward at speed =");
  EUSCIA0_OutUDec(Speed);
  EUSCIA0_OutString("\n\r");
}
void leftMotor(void){
  Motor_Left(Speed,Speed);
  Running = 1;
  EUSCIA0_OutString(" Motor turn left at speed =");
  EUSCIA0_OutUDec(Speed);
  EUSCIA0_OutString("\n\r");

}
void rightMotor(void){
  Motor_Right(Speed,Speed);
  Running = 1;
  EUSCIA0_OutString(" Motor turn right at speed =");
  EUSCIA0_OutUDec(Speed);
  EUSCIA0_OutString("\n\r");
}
void goSquare(void){ int i;
  EUSCIA0_OutString(" Robot goes in a square at speed =");
  EUSCIA0_OutUDec(Speed);
  EUSCIA0_OutString("\n\r");
  Running = 1;
  for(i=0;i<4;i++){
    Time = 0;
    EUSCIA0_OutString("turn ");
    Motor_Right(Speed/2,Speed/2);
    while(Time<20){
      if(Running==0) break; // crash
    }
    EUSCIA0_OutString("go ");
    Time = 0;
    Motor_Forward(Speed,Speed);
    while(Time<30){
      if(Running==0) break; // crash
    }
  }
  EUSCIA0_OutString("done");
}
void readBump(void){
  Push(Bump_Read());
}

void stopMotor(void){
  Motor_Stop(); // stop
  Running = 0;  // not running
  EUSCIA0_OutString(" Motor stopped\n\r");

}
void help(void){
  EUSCIA0_OutString("\n\rLab 18 interpreter\n\r");
  EUSCIA0_OutString("Numbers pushed on stack, parameters popped from top of stack (TOS). \n\r");
  EUSCIA0_OutString("l=   output TOS to LED\n\r");
  EUSCIA0_OutString("+    pop two, add two numbers, push sum\n\r");
  EUSCIA0_OutString("-    pop two, subtract two numbers, push difference\n\r");
  EUSCIA0_OutString("*    pop two, multiple two numbers, push product\n\r");
  EUSCIA0_OutString("/    pop two, divide two numbers, push quotient\n\r");
  EUSCIA0_OutString("=    pop one, print as signed decimal \n\r");
  EUSCIA0_OutString("u=   pop one, print as unsigned decimal \n\r");
  EUSCIA0_OutString("h=   pop one, print as unsigned hexadecimal \n\r");
  EUSCIA0_OutString("f=   pop one, print as unsigned fixed point 0.01 format\n\r");
  EUSCIA0_OutString("duty set robot motor speeds (0 to 14990)\n\r");
  EUSCIA0_OutString("s    stop the motors  \n\r");
  EUSCIA0_OutString("g    go motors forward   \n\r");
  EUSCIA0_OutString("l    turn robot left\n\r");
  EUSCIA0_OutString("r    turn robot right\n\r");
  EUSCIA0_OutString("sq   run robot in a square patter\n\r");
  EUSCIA0_OutString("b    read bump sensors\n\r");
  EUSCIA0_OutString("help help\n\r");
  EUSCIA0_OutString("?    help\n\r");
  Reset();
}
typedef struct {
  char CmdName[8];          // name of command
  void (*fnctPt)(void);     // to execute this command
  void *Next;
 }Cmd_t;
const Cmd_t LL[18]={ // linear linked list
  { "l=",     &outLED,        (void*)&LL[1]},
  { "s",      &stopMotor,     (void*)&LL[2]},
  { "g",      &forwardMotor,  (void*)&LL[3]},
  { "+",      &Add,           (void*)&LL[4]},
  { "-",      &Subtract,      (void*)&LL[5]},
  { "*",      &Multiply,      (void*)&LL[6]},
  { "/",      &Divide,        (void*)&LL[7]},
  { "=",      &PopPrintDec,   (void*)&LL[8]},
  { "u=",     &PopPrintUDec,  (void*)&LL[9]},
  { "h=",     &PopPrintHex,   (void*)&LL[10]},
  { "f=",     &PopPrintFix,   (void*)&LL[11]},
  { "l",      &leftMotor,     (void*)&LL[12]},
  { "r",      &rightMotor,    (void*)&LL[13]},
  { "duty",   &setSpeed,      (void*)&LL[14]},
  { "b",      &readBump,      (void*)&LL[15]},
  { "help",   &help,          (void*)&LL[16]},
  { "?",      &help,          (void*)&LL[17]},
  { "sq",     &goSquare,      (void*)0 }
};


int rmain(void) {
  char buf[40];  char token[40];
  int i,j;  int number; int ok;
  Cmd_t *pt;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  Motor_Init();
  Motor_Stop();
  LaunchPad_Init();
  Bump_Init();
  TimerA1_Init(&CheckBumper,50000);  // 10 Hz, stop on crash
  EUSCIA0_Init();     // initialize UART
  UserSP = &UserStack[STACKSIZE]; // empty
  EnableInterrupts();
  help();

  while(1){                     // Loop forever
    EUSCIA0_OutString("\n\r>");      // prompt
    EUSCIA0_InString(buf,39);        // receive input string, CR terminated
    i = 0;
    while(buf[i]){                // got data?
      while(isWhite(buf[i])) i++; // skip spaces and tabs
      if(isPrinting(buf[i])){
        j = 0;
        while(isPrinting(buf[i])){
          token[j] = buf[i];  // build token
          if((token[j]>='A')&&(token[j]<='Z')){
            token[j] = token[j]+('a'-'A'); // make lower case
          }
          j++; i++;
        }
        token[j] = 0;   // terminated
        pt = (Cmd_t *)&LL[0];           // first node to check
        ok = 0;
        while(pt){
          if(strcmp(token,pt->CmdName) == 0){
            pt->fnctPt();
            ok = 1;  // found and executed token
            break;
          }
          pt = pt->Next;
        }
        if(ok==0){
          if(isNumber(token, &number)){
            Push(number);
          } else{
            EUSCIA0_OutString(" ?? \n\r");
          }
        }
      }
    }
  }
}

