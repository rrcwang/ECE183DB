#ifndef ROBOT_INIT_H
#define ROBOT_INIT_H

#include <webots/robot.h>
#include <webots/motor.h>

// device tag variables
WbDeviceTag front_left_motor;
WbDeviceTag front_right_motor;
WbDeviceTag back_left_motor;
WbDeviceTag back_right_motor;

// retrieves devices, call once before calling any other functions
void frisbot_init();

#endif