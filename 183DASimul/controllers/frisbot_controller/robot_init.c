#include "robot_init.h"

void frisbot_init()
{
    // devices retrieved here
    front_left_motor = wb_robot_get_device("front left wheel");
    front_right_motor = wb_robot_get_device("front right wheel");
    back_left_motor = wb_robot_get_device("back left wheel");
    back_right_motor = wb_robot_get_device("back right wheel");
    return;
}

