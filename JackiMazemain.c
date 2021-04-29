/*
 * Team Delta Design Project 2
 * Authors. Anthony Caamano, Haleigh Defoor, Preston Brazzle, Thomas Driscoll
 * ECE 1188, Cyber-Pysical Systems
 * University of Pittsburgh
 * April 2021
 *
 * This code implements a maze-solving state machine that incorporates
 *  - Bump sensor interrupt
 *  - Line sensor SysTick interrupt
 *  - PWM motor control
 *  - Bluetooth start, stop, and polling updates
 *  - PID motor control
 *  - Ultrasonic Distance Sensors (triple)
 */

// swap slashes for windows
#include <stdint.h>
#include "msp.h"
#include "../inc/ADC14.h"
#include "../inc/AP.h"
#include "../inc/BumpInt.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "Debug.h"
#include "../inc/IRDistance.h"
#include "../inc/LaunchPad.h"
#include "../inc/LPF.h"
#include "../inc/Motors.h"
#include "../inc/Nokia5110.h"
#include "../inc/PWM.h"
#include "../inc/Reflectance.h"
#include "../inc/SysTickInts.h"
#include "../inc/TExaS.h"
#include "../inc/TimerA1.h"
#include "../inc/UART0.h"

uint32_t Input;
uint32_t Output;
float globalSpeed = 0.40;

uint8_t buffer[256*2]; //Debug buffer

uint8_t controlState;
uint8_t bumpSensorData;
uint8_t reading;

// Motor interface, see Motor.c
// IR sensor interface, see IRdistance.c
// bump sensor interface, see Bump.c
// Nokia interface, see Nokia5110.c
void OutValue(char *label,uint32_t value){
  UART0_OutString(label);
  UART0_OutUHex(value);
}



/*********************************
 *      START / STOP CONTROL
 *********************************/
void StartMaze(void) {
    LaunchPad_Output(GREEN);
    Motor_Start();
}
void StopMaze(void) {
    LaunchPad_Output(RED);
    Motor_Stop();
}

/*********************************
 *      BUMP SENSOR INTERRUPT
 *********************************/

void HandleCollision(uint8_t bumpSensor){
    bumpSensorData = bumpSensor;
    StopMaze();
}

/*********************************
 *      LINE SENSOR INTERRUPT
 *********************************/

uint32_t i = 0;

void SysTick_Handler(void){ // every 1ms

    if (i%10==0) {
        Reflectance_Start();
    }
    else if (i%10==1) {
        reading = Reflectance_End();
    }
    i++;
}

/*********************************
 *      BLUETOOOTH FUNCTIONS
 *********************************/

void ReadControlState(void) {
    OutValue("\n\rRead Control State=",controlState);
}
void WriteControlState(void) {
    if(controlState == 0x00)        StopMaze(); //Stop
    else if(controlState == 0x01)   StartMaze(); //Go
    OutValue("\n\rWrite Control State=",controlState);
}
void NotifyBumpSensor(void){ // called on SNP CCCD Updated Indication
    OutValue("\n\rRead Bump Sensor=",AP_GetNotifyCCCD(0));
}
void NotifyLineSensor(void) {
    OutValue("\n\rNotify Line Sensor=",AP_GetNotifyCCCD(1));
}
void BLE_Init(void){
    // write this as part of Lab 19
    UART0_OutString("\n\rApplication Processor - MSP432-CC2650\n\r");
    volatile int r = AP_Init();
    AP_GetStatus();  // optional
    AP_GetVersion(); // optional
    AP_AddService(0xFFF0);

    controlState = 0x00;
    AP_AddCharacteristic(0xFFF1,1,&controlState,0x03,0x0A,"Control State",&ReadControlState,&WriteControlState);

    bumpSensorData = 0x00;
    AP_AddNotifyCharacteristic(0xFFF6,1,&bumpSensorData,"Bump Sensor",&NotifyBumpSensor);

    reading = 0x00;
    AP_AddNotifyCharacteristic(0xFFF5,1,&reading,"Line Sensor",&NotifyLineSensor);

    AP_RegisterService();
    AP_StartAdvertisement();
    AP_GetStatus(); // optional
}

int time=0;
void SendBluetoothData(void) {
    AP_BackgroundProcess();  // handle incoming SNP frames
    time++;

    //Update Line Sensor Data Periodically
    if(time>1000000){
      time = 0;
      if(AP_GetNotifyCCCD(1)) {
          OutValue("\n\rNotify Line Sensor=",reading);
          AP_SendNotification(1);
      }
    }
    //Update Bump Sensor Data on Collision
    if(bumpSensorData != 0x00) {
        if(AP_GetNotifyCCCD(0)) {
            OutValue("\n\rNotify Bump Sensor=",bumpSensorData);
            AP_SendNotification(0);
            bumpSensorData = 0x00;
        }
    }
}
/*********************************
 *      MAIN FUNCTION
 *********************************/

void main(void){
    DisableInterrupts();

    Clock_Init48MHz();
    UART0_Init();
    LaunchPad_Init();  // input from switches, output to LEDs on LaunchPad
    Reflectance_Init();
    SysTick_Init(48000,2);
    Motor_Init(0, 0);
    BumpInt_Init(&HandleCollision);
    EnableInterrupts();

    BLE_Init();

    StopMaze();
    while(LaunchPad_Input()==0) { // wait for touch
        if(controlState == 1) break;
        AP_BackgroundProcess();  // handle incoming SNP frames
    };
    while(LaunchPad_Input());     // wait for release
    controlState = 0x01;
    StartMaze();

    int bindex = 0;
    while(1){
        SendBluetoothData();

        if(controlState) {
            //Traverse Maze
            DriveController();

            // ROM Debug
            buffer[bindex]= reading; // adds line reading to buffer
            if(bindex<256*2)
                bindex++; // increments index
            else
            {
                bindex=0; // resets index when buffer is full
                Debug_FlashRecord((uint16_t *) buffer); // puts buffer into ROM
            }
        }
    }

}

void Drive_Controller(void) {
    ////////////////////////
    // Drive Controller
    ////////////////////////
    Motor_DutyLeft(PERIOD * globalSpeed);      //Drive Left Motor
    Motor_DutyRight(PERIOD * globalSpeed);    //Drive Right Motor
    Clock_Delay1ms(10);     // wait
}
