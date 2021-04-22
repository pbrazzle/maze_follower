// Motors.c
// Runs on MSP432
// PWM on P2.6 (Right Motor) using TimerA0 TA0.CCR3
// PWM on P2.7 (Left Motor) using TimerA0 TA0.CCR4
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz

/*
* CTL Register Info
* bit  mode
* 9-8  10    TASSEL, SMCLK=12MHz
* 7-6  00    ID, divide by 1
* 5-4  11    MC, up-down mode
* 2    0     TACLR, no clear
* 1    0     TAIE, no interrupt
* 0          TAIFG
*/

#include "msp.h"
#include "Motors.h"

#define PERIOD 10000

void Motor_Init(int16_t dutyLeft, int16_t dutyRight){
    if(dutyLeft >= PERIOD || -dutyLeft >= PERIOD) return;
    if(dutyRight >= PERIOD || -dutyRight >= PERIOD) return;           // bad input

    //Initialize Motors
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;                                                // P5.4 & 5 as GPIO
    P5->DIR |= 0x30;                                                  // make P5.4 & 5 out
    P5->OUT &= ~0x30;

    if (dutyLeft < 0) P5->OUT |= 0x10;
    if (dutyRight < 0) P5->OUT |= 0x20;

    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;                                                // P3.6 & 7 as GPIO
    P3->DIR |= 0xC0;                                                  // make P3.6 & 7 out
    P3->OUT &= ~0xC0;                                                 // puts driver to sleep

    P2->DIR |= 0xC0;                                                  // P2.6&7 output
    P2->SEL0 |= 0xC0;                                                 // P2.6&7 Timer0A functions
    P2->SEL1 &= ~0xC0;                                                // P2.6&7 Timer0A functions

    //Initialize TimerA
    TIMER_A0->CCTL[0] = 0x0080;                                       // CCI0 toggle
    TIMER_A0->CCR[0] = PERIOD;                                        // set period register to 10,000
    TIMER_A0->CTL = 0x0230;                                           // SMCLK=12MHz, divide by 1, up-down mode
    TIMER_A0->EX0 = 0x0000;                                           // divide by 1

    TIMER_A0->CCTL[3] = 0x0040;                                       // CCR3 toggle/reset
    TIMER_A0->CCR[3] = (dutyLeft < 0) ? -dutyLeft : dutyLeft;         // CCR3 duty cycle is duty1/period
    TIMER_A0->CCTL[4] = 0x0040;                                       // CCR4 toggle/reset
    TIMER_A0->CCR[4] = (dutyRight < 0) ? -dutyRight : dutyRight;      // CCR4 duty cycle is duty1/period
}

void Motor_DutyLeft(int16_t dutyLeft){
    if(dutyLeft >= PERIOD || -dutyLeft >= PERIOD) return;

    if (dutyLeft < 0)
    {
        P5->OUT |= 0x20;
        TIMER_A0->CCR[4] = -dutyLeft;
    } else
    {
        P5->OUT &= ~0x20;
        TIMER_A0->CCR[4] = dutyLeft;
    }
}

void Motor_DutyRight(int16_t dutyRight){
    if(dutyRight >= PERIOD || -dutyRight >= PERIOD) return;

    if (dutyRight < 0)
    {
        P5->OUT |= 0x10;
        TIMER_A0->CCR[3] = -dutyRight;
    } else
    {
        P5->OUT &= ~0x10;
        TIMER_A0->CCR[3] = dutyRight;
    }
}

void Motor_Start(){
    P3->OUT |= 0xC0;    // disable sleep
}

void Motor_Stop(){
  P2->OUT &= ~0xC0;
  P3->OUT &= ~0xC0;   // sleep mode
}
