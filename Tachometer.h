//Tachometer Controls
//Preston Brazzle
//TODO: return direction of velocity

#ifndef TACHOMETER_H_
#define TACHOMETER_H_

#include <stdint>

enum TachDirection{
  FORWARD, /**< Wheel is making robot move forward */
  STOPPED, /**< Wheel is stopped */
  REVERSE  /**< Wheel is making robot move backward */
};

void Tachometer_Init(void);

double get_velocity_left();
double get_velocity_right();

uint16_t getLeftPeriod();
uint16_t getRightPeriod();

#endif
