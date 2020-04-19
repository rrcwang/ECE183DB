"""altino_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Keyboard
from altino import Altino
import csv
import cv2
import numpy as np

def log_gps_data(time, vector):
    with open('gps_log.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time, vector[0], vector[1], vector[2]])

def log_range_data(image):
    np.savetxt('range_log.csv', image, delimiter = ', ')


# create the Robot instance.
altino = Altino()

# clear logs, add column headers
f = open('gps_log.csv', "w+")
with open('gps_log.csv', 'a', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["t(ms)", "x", "y", "z"])

f = open('range_log.csv', "w+")

# get the time step of the current world.
timestep = altino.timeStep

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
altino.enable_sensors()

keyboard = Keyboard()
keyboard.enable(10)
steer = 0
current_time = 0;
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while altino.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()    

    gps_data = altino.gps.getValues()
    log_gps_data(current_time, gps_data)
    range_data = altino.range_finder.getRangeImageArray()
    current_time += timestep
    
    key = keyboard.getKey()
    if key == Keyboard.UP:
        altino.set_speed(50)
    elif key == Keyboard.DOWN:
        log_range_data(range_data)
    elif key == Keyboard.LEFT:
        steer -= 0.2
    elif key == Keyboard.RIGHT:
        steer += 0.2
    else:
        altino.set_speed(0)
    altino.set_steer(steer)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
workbook.close()
