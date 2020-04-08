#include "robot_actuators.h"

ACTUATOR_STATUS forward(float speed)
{
    if (speed > MAX_SPEED)
    {
        return ACTUATOR_ERROR;
    }
    else
    {
        wb_motor_set_position(front_left_motor, INFINITY);
        wb_motor_set_position(front_right_motor, INFINITY);
        wb_motor_set_position(back_left_motor, INFINITY);
        wb_motor_set_position(back_right_motor, INFINITY);
        wb_motor_set_velocity(front_left_motor, speed);
        wb_motor_set_velocity(front_right_motor, speed);
        wb_motor_set_velocity(back_left_motor, speed);
        wb_motor_set_velocity(back_right_motor, speed);
        return ACTUATOR_OK;
    }
}
