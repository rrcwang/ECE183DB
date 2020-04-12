"""altino_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
front_left_motor = robot.getMotor('left_front_wheel')
front_right_motor = robot.getMotor('right_front_wheel')
back_left_motor = robot.getMotor('left_rear_wheel')
back_right_motor = robot.getMotor('right_rear_wheel')

Motor.setPosition(front_left_motor, float('inf'))
Motor.setPosition(front_right_motor, float('inf'))
Motor.setPosition(back_left_motor, float('inf'))
Motor.setPosition(back_right_motor, float('inf'))

Motor.setVelocity(front_left_motor, 20)
Motor.setVelocity(front_right_motor, 20)
Motor.setVelocity(back_left_motor, 20)
Motor.setVelocity(front_right_motor, 20)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
