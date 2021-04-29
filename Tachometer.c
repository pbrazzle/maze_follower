#include <stdint.h>
#include "msp.h"
#include "Tachometer.h"

uint16_t firstRightTime, secondRightTime;
uint16_t firstLeftTime, secondLeftTime;
uint16_t leftPeriod, rightPeriod;
uint16_t leftDegrees = 0, rightDegrees = 0;
uint16_t rightForward = 0, leftForward = 0;

void Tachometer_Init(void){
  //right 10.4 left 5.2/10.5
  //right dir 5.0 left dir 5.2
  //GPIO
  P10->SEL0 |= 0x30;
  P10->SEL1 &= ~0x30;
  P10->DIR &= ~0x30;
  
  P5->SEL0 &= ~0x05;
  P5->SEL1 &= ~0x05;
  P5->DIR &= ~0x05;

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
  rightDegrees = (rightDegrees > 0) ? rightDegrees-1 : 0;
  if (P5->IN & 0x01) rightForward = 1;
  else rightForward = 0;
}

//Left tachometer interrupt
void TA3_N_IRQHandler(void){
  TIMER_A3->CCTL[1] &= ~0x0001; //ack
  firstLeftTime = secondLeftTime;
  secondLeftTime = TIMER_A3->CCR[1];
  //leftPeriod = secondLeftTime - firstLeftTime;
  leftPeriod = firstLeftTime - secondLeftTime;
  leftDegrees = (leftDegrees > 0) ? leftDegrees-1 : 0;
  if (P5->IN & 0x04) leftForward = 1;
  else leftForward = 0;
}

void turnLeft(int degrees)
{
	leftDegrees = degrees - 25;
	while(leftDegrees);
}

void turnRight(int degrees)
{
	rightDegrees = degrees - 25;
	while(rightDegrees);
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
	double vel = (leftPeriod) * 0.083 * 0.001 * 18 / 11;
	if (!leftForward) vel = -vel;
    return vel;
}

double get_velocity_right()
{
	double vel = (rightPeriod) * 0.083 * 0.001 * 18 / 11;
	if (!rightForward) vel = -vel;
    return vel;
}
