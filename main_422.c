//Maze Follower main implementation
//Preston Brazzle, Thomas Driscoll, and Anthony Caamano

//System includes
#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\LaunchPad.h"
#include "../inc/TA3InputCapture.h"

//Our includes
#include "Motors.h"
#include "Tachometer.h"
#include "Ultrasonic.h"

float globalSpeed = 0.50;
double l,r,c, leftSpeed, rightSpeed;

int main()
{
    Clock_Init48MHz();
    Tachometer_Init();
    Ultrasonic_Init();
    Motor_Init(PERIOD*globalSpeed, PERIOD*globalSpeed);
    Motor_Start();
    EnableInterrupts();
    while(1)
    {
        l = readLeft() * 0.00001 * 343;
        c = readCenter() * 0.00001 * 343;
        r = readRight() * 0.00001 * 343;

        leftSpeed = get_velocity_left();
        rightSpeed = get_velocity_right();
    }
}
