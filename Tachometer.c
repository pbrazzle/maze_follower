#include <stdint.h>
#include "msp.h"
#include "Tachometer.h"

uint16_t firstRightTime, secondRightTime;
uint16_t firstLeftTime, secondLeftTime;
uint16_t leftPeriod, rightPeriod;

void Tachometer_Init(void){
  //right 10.4 left 5.2/10.5
  //GPIO
  P10->SEL0 |= 0x30;
  P10->SEL1 &= ~0x30;
  P10->DIR &= ~0x30;

  //Halt TimerA3
  TIMER_A3->CTL &= ~0x0030;

  //Configure submodules (synchronous capture interrupt on rising edge)
  TIMER_A3->CCTL[0] = 0x4910;
  TIMER_A3->CCTL[1] = 0x4910;

  //Set up interrupts
  NVIC->IP[3] = (NVIC->IP[3]&0x1F1FFFFF)|0x40400000;
  NVIC->ISER[0] = 0x0000C000;

  //Reset and start continuous up mode
  TIMER_A3->CTL = 0x0224;
}

//Right tachometer interrupt
void TA3_0_IRQHandler(void){
  TIMER_A3->CCTL[0] &= ~0x0001; //ack
  firstRightTime = secondRightTime;
  secondRightTime = TIMER_A3->CCR[0];
  //rightPeriod = secondRightTime - firstRightTime;
  rightPeriod = firstRightTime - secondRightTime;
}

//Left tachometer interrupt
void TA3_N_IRQHandler(void){
  TIMER_A3->CCTL[1] &= ~0x0001; //ack
  firstLeftTime = secondLeftTime;
  secondLeftTime = TIMER_A3->CCR[1];
  //leftPeriod = secondLeftTime - firstLeftTime;
  leftPeriod = firstLeftTime - secondLeftTime;
}

uint16_t getLeftPeriod()
{
    return leftPeriod;
}

uint16_t getRightPeriod()
{
    return rightPeriod;
}

double get_velocity_left()
{
    return (leftPeriod) * 0.083 * 0.001 * 18 / 11;
}

double get_velocity_right()
{
    return (rightPeriod) * 0.083 * 0.001 * 18 / 11;
}
