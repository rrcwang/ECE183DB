/*
 * File:          epuck_go_forward.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 10

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   WbDeviceTag front_left_motor = wb_robot_get_device("front left wheel");
   WbDeviceTag front_right_motor = wb_robot_get_device("front right wheel");
   WbDeviceTag back_left_motor = wb_robot_get_device("back left wheel");
   WbDeviceTag back_right_motor = wb_robot_get_device("back right wheel");
   
   wb_motor_set_position(front_left_motor, INFINITY);
   wb_motor_set_position(front_right_motor, INFINITY);
   wb_motor_set_position(back_left_motor, INFINITY);
   wb_motor_set_position(back_right_motor, INFINITY);
   
   wb_motor_set_velocity(front_left_motor, MAX_SPEED);
   wb_motor_set_velocity(front_right_motor, MAX_SPEED);
   wb_motor_set_velocity(back_left_motor, MAX_SPEED);
   wb_motor_set_velocity(back_right_motor, MAX_SPEED);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
