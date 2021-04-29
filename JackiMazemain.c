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
#include "Tachometer.h"
#include "../inc/TExaS.h"
#include "../inc/TimerA1.h"
#include "../inc/UART0.h"
#include "Ultrasonic.h"

float globalSpeed = 0.10;

uint8_t buffer[256*2]; //Debug buffer

uint8_t controlState;
uint8_t controlTurn;
uint8_t bumpSensorData;
uint8_t reading;

/*********************************
 *      START / STOP CONTROL
 *********************************/
void StartMaze(void) {
    LaunchPad_Output(GREEN);
    Motor_Start();
    controlState = 0x01;
}
void StopMaze(void) {
    LaunchPad_Output(RED);
    Motor_Stop();
    controlState = 0x00;
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
 *      FOUND LINES
 *********************************/
void Lines_Found(void){
    
    if(reading!=0 && reading!=255) {
    int count=0;
    MazeStop();
    LaunchPad_Output(BLUE);
        while(count<5){  
        LaunchPad_LED(1);
        Clock_Delay1us(50000);
        LaunchPad_LED(0);
        Clock_Delay1us(50000);
        }
    }        
} 
/*********************************
 *      ULTRASONIC SENSORS
 *********************************/
uint32_t Left_mm,Right_mm,Center_mm; // IR distances in mm
#define OPEN_DIST_L 250
#define OPEN_DIST_R 250
#define OPEN_DIST_C 250
void PingUltrasonicSensors() {
    Left_mm = readLeft() * 0.001 * 343;
    Right_mm = readRight() * 0.001 * 343;
    Center_mm = readCenter() * 0.001 * 343;
}

/*********************************
 *      BLUETOOOTH FUNCTIONS
 *********************************/
void OutValue(char *label,uint32_t value){
  UART0_OutString(label);
  UART0_OutUHex(value);
}
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
 *      PWM FUNTIONS
 *********************************/
int16_t UR, UL;  // PWM duty 0 to 14,998
int32_t Error;
int32_t Ki=10;  // integral controller gain
int32_t Kp=32;  // proportional controller gain

#define TOOCLOSE 200
#define DESIRED 250
int32_t SetPoint = 250;
#define TOOFAR 400

int16_t PWMnominal=2500;
#define SWING 1000
#define PWMMIN (PWMnominal-SWING)
#define PWMMAX (PWMnominal+SWING)
void PID_Motor_Drive(void){ // runs at 100 Hz
    if((Left_mm>DESIRED)&&(Right_mm>DESIRED)){
      SetPoint = (Left_mm+Right_mm)/2;
    }else{
      SetPoint = DESIRED;
    }
    if(Left_mm < Right_mm ){
      Error = Left_mm-SetPoint;
    }else{
      Error = SetPoint-Right_mm;
    }
 //   UR = UR + Ki*Error;      // adjust right motor
    UR = PWMnominal+Kp*Error; // proportional control
    UL = PWMnominal-Kp*Error; // proportional control
    if(UR < (PWMnominal-SWING)) UR = PWMnominal-SWING; // 3,000 to 7,000
    if(UR > (PWMnominal+SWING)) UR = PWMnominal+SWING;
    if(UL < (PWMnominal-SWING)) UL = PWMnominal-SWING; // 3,000 to 7,000
    if(UL > (PWMnominal+SWING)) UL = PWMnominal+SWING;
    Motor_DutyLeft(UL);
    Motor_DutyRight(UR);
}

/*********************************
 *      DRIVE CONTROLLER
 *********************************/
void DriveController(void) {
    if(Right_mm > OPEN_DIST_R) {
        //Turn Right
        controlTurn = 0x01;
        //Forward 360
        Motor_DutyLeft(PERIOD * globalSpeed);   //Drive Left Motor
        Motor_DutyRight(PERIOD * globalSpeed);  //Drive Right Motor
        turnBoth(360);
        //Pivot 90 Right
        Motor_DutyLeft(-PERIOD * globalSpeed);   //Drive Left Motor
        Motor_DutyRight(PERIOD * globalSpeed); //Drive Right Motor
        turnBoth(180);
        //Forward 360
        Motor_DutyLeft(PERIOD * globalSpeed);   //Drive Left Motor
        Motor_DutyRight(PERIOD * globalSpeed);  //Drive Right Motor
        turnBoth(200);

    }
    else if(Center_mm > OPEN_DIST_C) {
        //Go Forward
        controlTurn = 0x00;
//        Motor_DutyLeft(PERIOD * globalSpeed);   //Drive Left Motor
//        Motor_DutyRight(PERIOD * globalSpeed);  //Drive Right Motor
//        Clock_Delay1ms(10);     // wait
        PID_Motor_Drive();
    }
    else if(Left_mm > OPEN_DIST_L) {
        //Turn Left
        controlTurn = 0x02;
        //Forward 360
        Motor_DutyLeft(PERIOD * globalSpeed);   //Drive Left Motor
        Motor_DutyRight(PERIOD * globalSpeed);  //Drive Right Motor
        turnBoth(360);
        //Pivot 90 Left
        Motor_DutyLeft(PERIOD * globalSpeed);  //Drive Left Motor
        Motor_DutyRight(-PERIOD * globalSpeed);  //Drive Right Motor
        turnBoth(180);
        //Forward 360
        Motor_DutyLeft(PERIOD * globalSpeed);   //Drive Left Motor
        Motor_DutyRight(PERIOD * globalSpeed);  //Drive Right Motor
        turnBoth(200);
    }
    else {
        //Turn Around
        controlTurn = 0x03;
        //Pivot 360 Right
        Motor_DutyLeft(-PERIOD * globalSpeed);  //Drive Left Motor
        Motor_DutyRight(PERIOD * globalSpeed);  //Drive Right Motor
        turnBoth(360);
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
    Ultrasonic_Init();
    UR = UL = PWMnominal; //initial power
    Motor_Init(0,0);
    SysTick_Init(48000,2);
    BumpInt_Init(&HandleCollision);
    Tachometer_Init();
    EnableInterrupts();

    BLE_Init();

    StopMaze();

    int bindex = 0;
    while(1){
        SendBluetoothData();
        PingUltrasonicSensors();

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
        else {
            while(LaunchPad_Input()==0) { // wait for touch
                if(controlState == 1) break;
                SendBluetoothData();
                PingUltrasonicSensors();
                AP_BackgroundProcess();  // handle incoming SNP frames
            };
            while(LaunchPad_Input());     // wait for release
            StartMaze();
        }
    }

}
