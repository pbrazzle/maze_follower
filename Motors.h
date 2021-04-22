/*
 * Motors.h
 *
 *  Created on: Mar 20, 2021
 *  Author: Preston Brazzle
 */
#ifndef MOTORS_H_
#define MOTORS_H_

#define PERIOD 10000

/*
 * Initializes robot motors and TimerA functions
 *
 * Inputs: dutyLeft
 *         dutyRight
 *
 * Outputs: none
 */
void Motor_Init(int16_t dutyLeft, int16_t dutyRight);

/*
 * Changes duty cycle of left motor
 *
 * Inputs:  dutyLeft
 *
 * Outputs: none
*/
void Motor_DutyLeft(int16_t dutyLeft);

/*
 * Changes duty cycle of right motor
 *
 * Inputs:  dutyRight
 *
 * Outputs: none
*/
void Motor_DutyRight(int16_t dutyRight);

/*
 * Disables sleep mode for the motors
 *
 * Inputs: none
 *
 * Outputs: none
 */
void Motor_Start();

/*
 * Enables sleep mode for the motors
 *
 * Inputs: none
 *
 * Outputs: none
 */
void Motor_Stop();

#endif /* MOTORS_H_ */
