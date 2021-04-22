// main.c
// Runs on MSP432
// Switches_LED lab solution
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

// P5.0 is positive logic Activate, Toggle switch, internal pull-down
// P5.1 is positive logic Window1, momentary switch, internal pull-down
// P5.2 is positive logic Window2, momentary switch, internal pull-down
// P5.4 is positive logic alarm, LED
// Activate    Window                Alarm
//   off       doesn't matter        LED should be off
//   on    either window not pressed LED flashes 5Hz
//   on,   both sensors are pressed  LED should be off
#include <stdint.h>
#include "msp.h"
#include "..\inc\TExaS.h"
#include "..\inc\Clock.h"


// *********Debug software************
uint8_t sensor;
int Program8_1(void){
  Clock_Init48MHz();  // makes bus clock 48 MHz
  P5->SEL0 &= ~0x01;  // configure P5.0 GPIO
  P5->SEL1 &= ~0x01;
  P5->DIR &= ~0x01;   // make P5.0 in
  P5->REN |= 0x01;    // enable pull resistors on P5.0
  P5->OUT &= ~0x01;   // P5.0 pull-down
  while(1){
    sensor = P5->IN&0x01; // read switch
  }
}

int Program8_2(void){
  Clock_Init48MHz();  // makes bus clock 48 MHz
  TExaS_Init(LOGICANALYZER);
  P5->SEL0 &= ~0x01;  // configure P5.0 GPIO
  P5->SEL1 &= ~0x01;
  P5->DIR &= ~0x01;   // make P5.0 in
  P5->REN |= 0x01;    // enable pull resistors on P5.0
  P5->OUT &= ~0x01;   // P5.0 pull-down
  while(1){
    sensor = P5->IN&0x01; // read switch
    TExaS_Set(sensor);
  }
}
void LED_Init(void){
  P5->SEL0 &= ~0x01;  // configure P5.0 GPIO
  P5->SEL1 &= ~0x01;
  P5->DIR |= 0x01;    // make P5.0 output
}
void LED_On(void){
  P5->OUT |= 0x01;  // turn on
}
void LED_Off(void){
  P5->OUT &= ~0x01; // turn off
}
void LED_Toggle(void){
  P5->OUT ^= 0x01;  // change
}
int Program8_3(void){
  Clock_Init48MHz();  // makes bus clock 48 MHz
  LED_Init();         // activate output for LED
  while(1){
    LED_On();
    LED_Off();
  }
}
int Program8_4(void){
  Clock_Init48MHz();  // makes bus clock 48 MHz
  TExaS_Init(LOGICANALYZER_P5);
  LED_Init();         // activate output for LED
  while(1){
    LED_Toggle();
    Clock_Delay1ms(100);  // approximately 100 ms
  }
}
// *********Lab software************

void Security_Init(void){
  P5->SEL0 &= ~0x17; // configure P5.4,P5.2,P5.1,P5.0 GPIO
  P5->SEL1 &= ~0x17;
  P5->DIR |= 0x10;   // make P5.4 out
  P5->DIR &= ~0x07;  // make P5.0, P5.1, P5.2 in
  P5->REN |= 0x07;   // enable pull resistors on P5.0, P5.1, P5.2
  P5->OUT &= ~0x07;  // P5.0, P5.1, P5.2 pull-down
  P5->OUT &= ~0x10;  // alarm off
}
// solution 1 uses functions and is more abstract
//*********Security_InputActivate******
// read arm/disarm input
// inputs: none
// output: true if armed, false if disarmed
uint8_t Security_InputActivate(void){
  return (P5->IN&0x01);   // read P5.0 input
}
//*********Security_InputSensors******
// read window sensors input
// inputs: none
// output: 0 both not pressed
//         0x01 one pressed
//         0x02 the other pressed
//         0x03 both pressed
uint8_t Security_InputSensors(void){
  return (P5->IN&0x06)>>1;   // read P5.1, P5.2 input
}
//*********Security_OutputAlarm******
// write to alarm
// inputs: 0 off
//         1 on
// output: none
void Security_OutputAlarm(uint8_t data){
  P5->OUT = (P5->OUT&~0x10)|(data<<4);
}
//*********Security_ToggleAlarm******
// toggle alarm output
// inputs: none
// output: none
void Security_ToggleAlarm(void){
  P5->OUT = P5->OUT^0x10;
}
#define SIZE 100
uint32_t TimeBuffer[SIZE];  // array of debugging dumps
uint8_t InputBuffer[SIZE];
uint8_t OutputBuffer[SIZE];
uint32_t DumpCnt=0;
uint8_t LastIn=0xFF;
uint8_t LastOut=0xFF;
void Dump(uint8_t in, uint8_t out, uint32_t t){
  if(DumpCnt>=SIZE) return; // full
  if((in==LastIn)&&(out==LastOut)) return; // no change
  TimeBuffer[DumpCnt] = t;
  InputBuffer[DumpCnt] = in;
  OutputBuffer[DumpCnt] = out;
  LastIn = in;
  LastOut = out;
  DumpCnt++;
}
int main1(void){ uint8_t arm,sensors;
  uint32_t time;
  Clock_Init48MHz(); // makes it 48 MHz
  TExaS_Init(LOGICANALYZER_P5);
  Security_Init();   // sensors and alarm
  time = 0; DumpCnt=0;
  while(1){
    Clock_Delay1ms(100);              // 100ms delay makes a 5Hz period
    time++;
    arm = Security_InputActivate();    // arm 0 if deactivated, 1 if activated
    sensors = Security_InputSensors(); // 3 means ok, 0 means break in
    if((arm == 0x01)&&(sensors != 0x03)){
      Security_ToggleAlarm();   // toggle output for alarm
    }else{
      Security_OutputAlarm(0);   // LED off if deactivated or sensors are ok
    }
    Dump(P5->IN,P5->OUT,time);
  }
}


// less abstract, more efficient solution using bit-banding
// P5IN  0x40004C40
// P5.0 is 0x42000000+32*0x4C40+4*0 = 0x42098800
// P5.1 is 0x42000000+32*0x4C40+4*1 = 0x42098804
// P5.2 is 0x42000000+32*0x4C40+4*2 = 0x42098808
// P5OUT 0x40004C42
// P5.4 is 0x42000000+32*0x4C42+4*4 = 0x42098850

#define ACTIVATEIN (*((volatile uint8_t *)(0x42098800)))
#define WINDOW1IN (*((volatile uint8_t *)(0x42098804)))
#define WINDOW2IN (*((volatile uint8_t *)(0x42098808)))
#define ALARMOUT (*((volatile uint8_t *)(0x42098850)))

int main(void){
  uint32_t time;     // in 100 ms units
  Clock_Init48MHz(); // makes it 48 MHz
  TExaS_Init(LOGICANALYZER_P5);
  Security_Init();   // sensors and alarm
  time = 0; DumpCnt=0;
  while(1){
    Clock_Delay1ms(100);              // 100ms delay makes a 5Hz period
    time++;
    if((ACTIVATEIN)&&((WINDOW1IN == 0)||(WINDOW2IN == 0))){
      ALARMOUT = ALARMOUT^0x01;   // toggle output for alarm
    }else{
      ALARMOUT = 0;   // LED off if deactivated or sensors are ok
    }
    Dump(P5->IN,P5->OUT,time);
    TExaS_Set((P5->IN)&0x17);
  }
}


/* Challenge
int Challenge(void){ uint32_t Count=0;
  Clock_Init48MHz();  // makes bus clock 48 MHz
  Switch_Init();      // activate input from switch
  while(1){
    while(Switch_Input()==0){}; // wait for touch
    Count++;
    while(Switch_Input()!=0){}; // wait for release
  }
}

 */
