/*
 * Ultrasonic.c
 *
 *  Created on: Apr 13, 2021
 *      Author: haleighdefoor
 */
#include <stdint.h>
#include "msp432.h"
#include "..\inc\Clock.h"

void Ultrasonic_Init(void){
    //Right sensor
    //P8.7 trigger 8.4 echo
    P8->DIR |= 0x80;
    P8->DIR &= ~0x10;

    //Center sensor
    //P8.0 echo 7.5 trigger
    P8->DIR &= ~0x01;
    P7->DIR |= 0x20;

    //Left sensor
    //P10.0 trigger 10.1 echo
    P10->DIR |= 0x01;
    P10->DIR &= ~0x02;

    P6->DIR &= ~BIT7;
    P8->DIR &= ~BIT0;
    P10->DIR &= ~BIT5;
    P6->REN |= BIT7;
    P8->REN |= BIT0;
    P10->REN |= BIT5;
    P6->OUT &= ~BIT7;
    P8->OUT &= ~BIT0;
    P10->OUT &= ~BIT5;

    P6->SEL0 = 0;
    P6->SEL1 = 0;
    P8->SEL0 = 0;
    P8->SEL1 = 0;
    P10->SEL0 = 0;
    P10->SEL1 = 0;

    P6->IFG = 0;
    P8->IFG = 0;
    P10->IFG = 0;

    P6->IE |= BIT7;
    P8->IE |= BIT0;
    P10->IE |= BIT5;

    P6->IES &= ~BIT7;
    P8->IES &= ~BIT0;
    P10->IES &= ~BIT5;
}

void Ultrasonic_Start(void){
    //P5.0(L), P5.1(R), P5.2(C) are triggers
    P5->DIR |= 0x07; //0000 0111
    P5->OUT |= 0x07;
    Clock_Delay1us(10);
    P5->OUT &= ~0x07;
}

uint8_t UltrasonicL_End(void){
    //P6.7 is echo
    uint8_t reading;
    reading = P6->OUT &= 0x80;
    return reading;
}

uint8_t UltrasonicC_End(void){
    //P8.0 is echo
    uint8_t reading;
    reading = P8->OUT &= 0x01;
    return reading;
}

uint8_t UltrasonicR_End(void){
    //P10.5 is echo
    uint8_t reading;
    reading = P10->OUT &= 0x20;
    return reading;
}

uint32_t sz, x[1024], index, sum;
uint32_t *pointer;

void LPF_Init(uint32_t oldData, uint32_t size){
    if(size>1024)
        size = 1024;

    index = size-1;
    sz = size;

    sum = sz*oldData;

    int i = 0;
    for(i=0; i<sz; i++){
        x[i] = oldData;
    }
}

uint32_t LPF_Calc(uint32_t newData){
    uint32_t LPFCalc;

    if(index == 0){
      index = sz-1;
    }
    else
    index--;

    sum = sum+newData-x[index];
    x[index] = newData;

    LPFCalc = sum/sz;

    return LPFCalc;
}

uint32_t readLeft()
{
    //Send pulse
    P10->OUT |= 0x01;
    Clock_Delay1us(10);
    P10->OUT &= ~0x01;

    //Wait for echo in
    uint8_t result = P10->IN & 0x02;
    while (result == 0) result = P10->IN & 0x02;

    //Wait for echo to finish
    uint32_t duration = 0;
    while (result != 0)
    {
        duration++;
        result = P10->IN & 0x02;
    }

    return duration;
}

uint32_t readCenter()
{
    //Send pulse
    P7->OUT |= 0x20;
    Clock_Delay1us(10);
    P7->OUT &= ~0x20;

    //Wait for echo in
    uint8_t result = P8->IN & 0x01;
    while (result == 0) result = P8->IN & 0x01;

    //Wait for echo to finish
    uint32_t duration = 0;
    while (result != 0)
    {
        duration++;
        result = P8->IN & 0x01;
    }

    return duration;
}

uint32_t readRight()
{
    //Send pulse
    P8->OUT |= 0x80;
    Clock_Delay1us(10);
    P8->OUT &= ~0x80;

    //Wait for echo in
    uint8_t result = P8->IN & 0x10;
    while (result == 0) result = P8->IN & 0x10;

    //Wait for echo to finish
    uint32_t duration = 0;
    while (result != 0 && duration < 3000)
    {
        duration++;
        result = P8->IN & 0x10;
    }

    return duration;
}
