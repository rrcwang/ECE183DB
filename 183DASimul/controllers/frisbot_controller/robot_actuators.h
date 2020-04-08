#ifndef ROBOT_ACTUATORS_H
#define ROBOT_ACTUATORS_H

#include "robot_init.h"

#define MAX_SPEED 10

typedef enum 
{
    ACTUATOR_OK = 0,
    ACTUATOR_ERROR = 1
} ACTUATOR_STATUS;

// robot moves forward at target speed speed
// returns function status
ACTUATOR_STATUS forward(float speed);


//TODO: backwards, turning
#endif
