/*
 * Ultrasonic.h
 *
 *  Created on: Apr 13, 2021
 *      Author: haleighdefoor
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

void Ultrasonic_Init(void);

void Ultrasonic_Start(void);

uint8_t UltrasonicL_End(void);

uint8_t UltrasonicC_End(void);

uint8_t UltrasonicR_End(void);

void LPF_Init(uint32_t oldData, uint32_t size);

uint32_t LPF_Calc(uint32_t newData);

#endif /* ULTRASONIC_H_ */