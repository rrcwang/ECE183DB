"""altino_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Keyboard
from altino import Altino

# create the Robot instance.
altino = Altino()

# get the time step of the current world.
timestep = altino.timeStep

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

keyboard = Keyboard()
keyboard.enable(10)
steer = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while altino.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    key = keyboard.getKey()
    if key == Keyboard.UP:
        altino.forward(20)
    elif key == Keyboard.DOWN:
        altino.back(20)
    elif key == Keyboard.LEFT:
        steer -= 0.1
    elif key == Keyboard.RIGHT:
        steer += 0.1
    else:
        altino.forward(0)
    
    altino.set_steer(steer)

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
