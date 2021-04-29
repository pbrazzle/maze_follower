/*
 * Ultrasonic.c
 *
 *  Created on: Apr 13, 2021
 *      Author: haleighdefoor
 */
#include <stdint.h>
#include "msp432.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "Ultrasonic.h"

void Ultrasonic_Init(void){
    //Right sensor
    //P8.7 trigger 8.4 echo
    P8->DIR |= 0x80;
    P8->DIR &= ~0x10;

    //Center sensor
    //P10.3 echo 10.4 trigger
    P10->DIR &= ~0x08;
    P10->DIR |= 0x10;

    //Left sensor
    //P10.0 trigger 10.1 echo
    P10->DIR |= 0x01;
    P10->DIR &= ~0x02;

    P8->SEL0 = 0;
    P8->SEL1 = 0;
    P10->SEL0 = 0;
    P10->SEL1 = 0;

    P8->IFG = 0;
    P10->IFG = 0;

}

uint32_t readLeft()
{
    DisableInterrupts();
    uint32_t to=0;
    //Send pulse
    P10->OUT |= 0x01;
    Clock_Delay1us(10);
    P10->OUT &= ~0x01;

    //Wait for echo in
    uint8_t result = P10->IN & 0x02;
    while (result == 0 && to != 100000)
    {
        result = P10->IN & 0x02;
        to++;
    }

    //Wait for echo to finish
    uint32_t duration = 0;
    while (result != 0)
    {
        duration++;
        result = P10->IN & 0x02;
    }
    EnableInterrupts();
    return duration;
}

uint32_t readCenter()
{
    DisableInterrupts();
    uint32_t to=0;
    //Send pulse
    P10->OUT |= 0x10;
    Clock_Delay1us(10);
    P10->OUT &= ~0x10;

    //Wait for echo in
    uint8_t result = P10->IN & 0x08;
    while (result == 0 && to != 100000)
       {
           result = P10->IN & 0x08;
           to++;
       }

    //Wait for echo to finish
    uint32_t duration = 0;
    while (result != 0)
    {
        duration++;
        result = P10->IN & 0x08;
    }
    EnableInterrupts();
    return duration;
}

uint32_t readRight()
{
    DisableInterrupts();
    uint32_t to=0;
    //Send pulse
    P8->OUT |= 0x80;
    Clock_Delay1us(10);
    P8->OUT &= ~0x80;

    //Wait for echo in
    uint8_t result = P8->IN & 0x10;
    while (result == 0 && to != 100000)
    {
        result = P8->IN & 0x10;
        to++;
    }

    //Wait for echo to finish
    uint32_t duration = 0;
    while (result != 0)
    {
        duration++;
        result = P8->IN & 0x10;
    }
    EnableInterrupts();
    return duration;
}

