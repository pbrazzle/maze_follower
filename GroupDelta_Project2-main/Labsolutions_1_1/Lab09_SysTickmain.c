// Lab09_SysTickmain.c
// Runs on MSP432
// Solution version of SysTick, lab9
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

// built-in LED1 connected to P1.0
// negative logic built-in Button 1 connected to P1.1
// negative logic built-in Button 2 connected to P1.4
// built-in red LED connected to P2.0
// built-in green LED connected to P2.1
// built-in blue LED connected to P2.2
// RC circuit connected to P2.6, to create DAC (main4, main5)
#include <stdint.h>
#include "msp.h"
#include "..\inc\TExaS.h"
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTick.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Bump.h"
uint32_t const DutyBuf[100]={
    240000, 255063, 270067, 284953, 299661, 314133, 328313, 342144, 355573, 368545,
    381010, 392918, 404223, 414880, 424846, 434083, 442554, 450226, 457068, 463053,
    468158, 472363, 475651, 478008, 479427, 479900, 479427, 478008, 475651, 472363,
    468158, 463053, 457068, 450226, 442554, 434083, 424846, 414880, 404223, 392918,
    381010, 368545, 355573, 342144, 328313, 314133, 299661, 284953, 270067, 255063,
    240000, 224937, 209933, 195047, 180339, 165867, 151687, 137856, 124427, 111455,
    98990, 87082, 75777, 65120, 55154, 45917, 37446, 29774, 22932, 16947,
    11842, 7637, 4349, 1992, 573, 100, 573, 1992, 4349, 7637,
    11842, 16947, 22932, 29774, 37446, 45917, 55154, 65120, 75777, 87082,
    98990, 111455, 124427, 137856, 151687, 165867, 180339, 195047, 209933, 224937
};
const uint32_t PulseBuf[100]={
    5000, 5308, 5614, 5918, 6219, 6514, 6804, 7086, 7361, 7626,
    7880, 8123, 8354, 8572, 8776, 8964, 9137, 9294, 9434, 9556,
    9660, 9746, 9813, 9861, 9890, 9900, 9890, 9861, 9813, 9746,
    9660, 9556, 9434, 9294, 9137, 8964, 8776, 8572, 8354, 8123,
    7880, 7626, 7361, 7086, 6804, 6514, 6219, 5918, 5614, 5308,
    5000, 4692, 4386, 4082, 3781, 3486, 3196, 2914, 2639, 2374,
    2120, 1877, 1646, 1428, 1224, 1036,  863,  706,  566,  444,
     340,  254,  187,  139,  110,  100,  110,  139,  187,  254,
     340,  444,  566,  706,  863, 1036, 1224, 1428, 1646, 1877,
    2120, 2374, 2639, 2914, 3196, 3486, 3781, 4082, 4386, 4692};
void SysTick_Wait1us(uint32_t delay){
    // write this code
    SysTick->LOAD = (48*delay - 1);// count down to zero
    SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
    while(( SysTick->CTRL&0x00010000) == 0){};
}

int Program9_1(void){
  Clock_Init48MHz();  // makes bus clock 48 MHz
  SysTick_Init();
  LaunchPad_Init();   // buttons and LEDs
  TExaS_Init(LOGICANALYZER_P1);
  while(1){
    P1->OUT |= 0x01;   // red LED on
    SysTick_Wait1us(7500);
    P1->OUT &= ~0x01;  // red LED off
    SysTick_Wait1us(2500);
  }
}
int Program9_2(void){uint32_t H,L;
  Clock_Init48MHz();  // makes bus clock 48 MHz
  SysTick_Init();
  TExaS_Init(SCOPE);
  P2->SEL0 &= ~0x40;
  P2->SEL1 &= ~0x40; // 1) configure P2.6 as GPIO
  P2->DIR |= 0x40;   // P2.6 output
  H = 7500;
  L = 10000-H;
  while(1){
    P2->OUT |= 0x40;   // on
    SysTick_Wait1us(H);
    P2->OUT &= ~0x40;  // off
    SysTick_Wait1us(L);
  }
}

// Operation
// The heartbeat starts when the operator pushes Button 1
// The heartbeat stops when the operator pushes Button 2
// When beating, the P1.0 LED oscillates at 100 Hz (too fast to see with the eye)
//  and the duty cycle is varied sinuosoidally once a second
int main1(void){ uint32_t i; int run=0;
  uint32_t time,H,L; uint8_t in;
  Clock_Init48MHz(); // makes it 48 MHz
  TExaS_Init(LOGICANALYZER_P1);
  LaunchPad_Init();   // buttons and LEDs
  SysTick_Init();
  time = 0; i=0;
  EnableInterrupts();
  while(1){
    time++;
    in = LaunchPad_Input();
    if(in&0x01) run=1;  // start on switch 1 press
    if(in&0x02) run=0;  // stop on switch 2 press
    if(run){
      H = PulseBuf[i];
      L = 10000-H;
      LaunchPad_LED(1); // red LED on
      SysTick_Wait1us(H);
      LaunchPad_LED(0); // red LED off
      SysTick_Wait1us(L);
      i = (i+1)%100;
    }else{
      LaunchPad_LED(0); // red LED off
      SysTick_Wait1us(10000); // 10 ms
    }
  }
}
int main3(void){ uint32_t i; int run=0;
  uint32_t time; uint8_t in;
  Clock_Init48MHz(); // makes it 48 MHz
  TExaS_Init(LOGICANALYZER_P1);
  LaunchPad_Init();   // buttons and LEDs
  SysTick_Init();
  time = 0; i=0;
  while(1){
    time++;
    in = LaunchPad_Input();
    if(in&0x01) run=1;  // start on switch 1 press
    if(in&0x02) run=0;  // stop on switch 2 press
    if(run){
      LaunchPad_LED(1); // red LED on
      SysTick_Wait(DutyBuf[i]);
      LaunchPad_LED(0); // red LED off
      SysTick_Wait(480000-DutyBuf[i]);
      i = (i+1)%100;
    }else{
      LaunchPad_LED(0); // red LED off
      SysTick_Wait(480000); // 10 ms
    }
  }
}

int main4(void){ uint32_t i; int run=0;
  uint32_t time,H,L; uint8_t in;
  Clock_Init48MHz(); // makes it 48 MHz
  LaunchPad_Init();   // buttons and LEDs
  TExaS_Init(SCOPE);
  P2->SEL0 &= ~0x40;
  P2->SEL1 &= ~0x40; // 1) configure P2.6 as GPIO
  P2->DIR |= 0x40;   // P2.6 output
  SysTick_Init();
  time = 0; i=0;
  while(1){
    time++;
    in = LaunchPad_Input();
    if(in&0x01) run=1;  // start on switch 1 press
    if(in&0x02) run=0;  // stop on switch 2 press
    if(run){
      H = PulseBuf[i];
      L = 10000-H;
      P2->OUT |= 0x40;   // on
      SysTick_Wait1us(H);
      P2->OUT &= ~0x40;   // off
      SysTick_Wait1us(L);
      i = (i+1)%100;
    }else{
      P2->OUT &= ~0x40;   // off
      SysTick_Wait1us(10000); // 10 ms
    }
  }
}
uint32_t time,H,L;
int main5(void){ uint32_t i; int run=0;
 uint8_t in;
  Clock_Init48MHz(); // makes it 48 MHz
  LaunchPad_Init();   // buttons and LEDs
  TExaS_Init(SCOPE);
  P2->SEL0 &= ~0x40;
  P2->SEL1 &= ~0x40; // 1) configure P2.6 as GPIO
  P2->DIR |= 0x40;   // P2.6 output
  SysTick_Init();
  time = 0; i=0;
  H = 1000;
  L = 10000-H;
  while(1){
    time++;
    in = LaunchPad_Input();
    if(in&0x01) run=1;  // start on switch 1 press
    if(in&0x02){
      if(run==1){
        H = H+2000;
        if(H>9000){
          H=1000;
        }
      }
      L = 10000-H;
      run=0;  // stop on switch 2 press
    }
    if(run){
      P2->OUT |= 0x40;   // on
      SysTick_Wait1us(H);
      P2->OUT &= ~0x40;   // off
      SysTick_Wait1us(L);
      i = (i+1)%100;
    }else{
      P2->OUT &= ~0x40;   // off
      SysTick_Wait1us(10000); // 10 ms
    }
  }
}

int main7(void){ uint8_t in;
  LaunchPad_Init();   // buttons and LEDs
  in = LaunchPad_Input();
  if(in&0x01){
    main1();
  }else{
    main4();
  }
  while(1){};
}

// Extra credit, create sounds,
// Output on P3.6
// R C filter with cutoff about 1kHz
// rejects 20 kHz, passes 440 to 880 Hz
// approximate PWM frequency 20 kHz, period = 50us, 2400 bus cycles
struct sound{
  const uint16_t *Wave;
  uint32_t Size;
  uint32_t Period; // H+L in cycles
};
typedef const struct sound sound_t;


// desired sound frequency 343.14540624999 Hz, adjusted to 330Hz
// actual assumes clock frequency of 48000000 Hz
// actual sound frequency 343.112026076514 Hz
// Precision=2412 alternatives, 58-element sine wave
const uint16_t wave343[58] = {
  1206,1304,1401,1495,1586,1673,1754,1829,1897,1956,2006,
  2048,2079,2100,2111,2111,2100,2079,2048,2006,1956,1897,
  1829,1754,1673,1586,1495,1401,1304,1206,1108,1011,
  917,826,739,658,583,515,456,406,364,333,
  312,301,301,312,333,364,406,456,515,583,
  658,739,826,917,1011,1108};


// desired sound frequency 454.460093896714 Hz, tuned to be 440 Hz, A note
// assumes clock frequency of 48000000 Hz
// sound frequency 454.545454545455 Hz
// Precision=2400 alternatives, 44-element sine wave
const uint16_t  wave454[44] = {
  1200,1328,1454,1574,1687,1789,1880,1957,2019,2064,2091,
  2100,2091,2064,2019,1957,1880,1789,1687,1574,1454,1328,
  1200,1072,946,826,713,611,520,443,381,336,
  309,300,309,336,381,443,520,611,713,826,
  946,1072};

const uint16_t wave440[45] = {
//  1212,1367,1519,1664,1801,1927,2038,2134,2211,2270,2307,
//  2323,2318,2291,2243,2175,2088,1984,1866,1734,1592,1443,
//  1290,1134,981,832,690,558,440,336,249,181,
//  133,106,101,117,154,213,290,386,497,623,
//  760,905,1057};
  1212,1339,1463,1583,1695,1798,1890,1968,2032,2079,2110,
  2123,2119,2097,2058,2002,1931,1846,1748,1640,1524,1402,
  1276,1148,1022,900,784,676,578,493,422,366,
  327,305,301,314,345,392,456,534,626,729,
  841,961,1085};

// desired sound frequency 513.556927423088 Hz, B note
// actual assumes clock frequency of 48000000 Hz
// actual sound frequency 513.462340746446 Hz
// Precision=2398 alternatives, 39-element sine wave
const uint16_t wave514[39] = {
  1199,1343,1484,1617,1739,1847,1939,2011,2063,2091,2097,
  2080,2040,1978,1895,1795,1679,1551,1414,1271,1127,984,
  847,719,603,503,420,358,318,301,307,335,
  387,459,551,659,781,914,1055};

// desired sound frequency 523.251130601197 Hz, C note
// actual assumes clock frequency of 48000000 Hz
// actual sound frequency 523.263419526447 Hz
// Precision=2414 alternatives, 38-element sine wave
const uint16_t wave523[38] = {
  1207,1389,1566,1734,1887,2021,2134,2221,2280,2310,2310,
  2280,2221,2134,2021,1887,1734,1566,1389,1207,1025,848,
  680,527,393,280,193,134,104,104,134,193,
  280,393,527,680,848,1025};

// desired sound frequency 576.448160678787 Hz
// actual assumes clock frequency of 48000000 Hz
// actual sound frequency 576.472707620249 Hz
// Precision=2380 alternatives, 35-element sine wave
const uint16_t wave576[35] = {
  1190,1349,1503,1646,1776,1886,1974,2036,2072,2079,2058,
  2008,1933,1833,1713,1576,1427,1270,1110,953,804,667,
  547,447,372,322,301,308,344,406,494,604,
  734,877,1031};

// desired sound frequency 659.25511382574 Hz, E note
// desired sound frequency 685.250977935587 Hz
// assumes clock frequency of 48000000 Hz
// actual sound frequency 685.371599914329 Hz
// Precision=2416 alternatives, 29-element sine wave
const uint16_t wave685[29] = {
  1208,1446,1673,1879,2052,2187,2276,2314,2301,2237,2125,
  1970,1779,1562,1328,1088,854,637,446,291,179,115,
  102,140,229,364,537,743,970};

// desired sound frequency 769.465978737057 Hz
// actual assumes clock frequency of 48000000 Hz
// actual sound frequency 769.551415653958 Hz
// Precision=2400 alternatives, 26-element sine wave
const uint16_t wave769[26] = {
  1200,1415,1618,1797,1941,2042,2093,2093,2042,1941,1797,
  1618,1415,1200,985,782,603,459,358,307,307,358,
  459,603,782,985};

// desired sound frequency 783.990871963499 Hz, G note
// assumes clock frequency of 48000000 Hz
// actual sound frequency 783.929446349829 Hz
// Precision=2356 alternatives, 26-element sine wave
const uint16_t wave784[26] = {
  1178,1436,1679,1893,2065,2186,2248,2248,2186,2065,1893,
  1679,1436,1178,920,677,463,291,170,108,108,170,
  291,463,677,920};


// desired sound frequency 880 Hz, A note
// desired sound frequency 909.988249118684 Hz
// assumes clock frequency of 48000000 Hz
// actual sound frequency 909.849116688149 Hz
// Precision=2398 alternatives, 22-element sine wave
const uint16_t wave910[22] = {
  1199,1509,1793,2030,2199,2287,2287,2199,2030,1793,1509,
  1199,889,605,368,199,111,111,199,368,605,889,
  };

// desired sound frequency 1046.50226120239 Hz, C note
// actual assumes clock frequency of 48000000 Hz
// actual sound frequency 1046.52683905289 Hz
// Precision=2414 alternatives, 19-element sine wave
const uint16_t wave1047[19] = {
  1207,1566,1887,2134,2280,2310,2221,2021,1734,1389,1025,
  680,393,193,104,134,280,527,848};

// desired sound frequency 1174.65907166963 Hz, D note
// desired sound frequency 1222.51940585635 Hz
// actual assumes clock frequency of 48000000 Hz
// actual sound frequency 1222.49388753056 Hz
// Precision=2454 alternatives, 16-element sine wave
const uint16_t wave1223[16] = {
  1227,1658,2024,2268,2354,2268,2024,1658,1227,796,430,
  186,100,186,430,796};

#define NUM_NOTES 6
sound_t Notes[NUM_NOTES]={
  {wave343,58,2412},  // E note, 330 Hz
  {wave454,44,2400},  // A note, 440 Hz
  {wave514,39,2398},  // B note 494 Hz
  //  {wave523,38,2414},  // C note, 523 Hz
  {wave576,35,2380},  // DF note 554 Hz
  {wave685,29,2416},  // E note, 659 Hz
  {wave769,26,2400}   // GF note, 740 Hz
//  {wave784,26,2356},  // G note, 784 Hz
//  {wave910,22,2398},  // A note, 880 Hz
//  {wave1047,19,2414}, // C note, 1047 Hz
//  {wave1223,16,2454}  // D note, 1175 Hz
};
// Time delay using busy wait.
// The delay parameter is in units of the core clock.
// assumes 48 MHz bus clock
uint32_t startTime;
void SysTick_Wait2(uint32_t delay){
  volatile uint32_t elapsedTime;
  do{
    elapsedTime = (startTime-SysTick->VAL)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
  startTime = SysTick->VAL;
}
int main(void){ uint32_t i,note; int run;
  uint32_t time,H,L; uint8_t in,last;
  Clock_Init48MHz(); // makes it 48 MHz
  LaunchPad_Init();   // buttons and LEDs
  TExaS_Init(SCOPE);
  Bump_Init();
  P3->SEL0 &= ~0x40;
  P3->SEL1 &= ~0x40; // 1) configure P3.6 as GPIO
  P3->DIR |= 0x40;   // P3.6 output
  SysTick_Init();
  time = 0; i=0; run=0; note=NUM_NOTES-1;
  last = Bump_Read();
  while(1){
    time++;
//    in = LaunchPad_Input();
//    if((in&0x01)&&(run==0)){
//      note = (note+1)%NUM_NOTES; // 0,1,2,...NUM_NOTES-1
//      run = 1;  // start on switch 1 press
//      i = 0;    // index into sin wave
//    }
//    if(in&0x02) run=0;  // stop on switch 2 press
    in = Bump_Read();
    if(in&&(last==0)){
      switch(in){
        case 1: note = 0; break;
        case 2: note = 1; break;
        case 4: note = 2; break;
        case 8: note = 3; break;
        case 16: note = 4; break;
        case 32: note = 5; break;
      }
      run = 1;  // start on switch 1 press
      i = 0;    // index into sin wave
    }
    if(in==0) run = 0;
    last = in;
    if(run){
      H = Notes[note].Wave[i];
      L = Notes[note].Period-H;
      SysTick_Wait2(L);
      P3->OUT |= 0x40;   // on
      SysTick_Wait2(H);
      P3->OUT &= ~0x40;  // off
      i = (i+1)%(Notes[note].Size);
    }else{
      P3->OUT &= ~0x40;   // off
      SysTick_Wait1us(10000); // 10 ms
    }
  }
}



