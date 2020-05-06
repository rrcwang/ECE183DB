"""altino_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Keyboard
import altino as al
import camera_detection as cd
import numpy as np
import csv
import cv2

def log_gps_data(time, vector):
    with open('gps_log.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time, vector[0], vector[1], vector[2]])

def log_range_data(image):
    np.savetxt('range_log.csv', image, delimiter = ', ')

def log_frisbee_data(time, state):
    with open('frisbee_log.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time, state.x, state.y, state.z, state.dx, state.dy, state.dz, state.roll, state.pitch, state.yaw, state.d_roll, state.d_pitch, state.d_yaw])

# create the Robot instance.
altino = al.Altino()
# clear logs, add column headers
f = open('gps_log.csv', "w+")
with open('gps_log.csv', 'a', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["t(ms)", "x", "y", "z"])
    
f = open('frisbee_log.csv', "w+")
with open('frisbee_log.csv', 'a', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["t(ms)", "x", "y", "z", "dx", "dy", "dz", "roll", "pitch", "yaw", "d_roll", "d_pitch", "d_yaw"])

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

    gps_data      = altino.gps.getValues()
    range_data    = altino.range_finder.getRangeImageArray()
    frisbee_data  = cd.wb_detect(altino, [0, 1, 0])
    #altino.camera.getImage()
    #camera_status = altino.camera.saveImage('frame.png', 0)
    current_time += timestep
    radius = float('inf')
    
    key = keyboard.getKey()
    if key == Keyboard.UP:
        altino.set_speed(50)
    elif key == Keyboard.DOWN:
        altino.set_speed(-20)
    elif key == Keyboard.LEFT:
        radius = -0.3
        altino.set_speed(0)
    elif key == Keyboard.RIGHT:
        radius = 0.5
        altino.set_speed(20)
    else:
        altino.set_speed(0)
    altino.set_steer(radius)
    # Process sensor data here.
    log_gps_data(current_time, gps_data)
    if frisbee_data:
        pos = frisbee_data.get_position()
        frisbee_state = cd.get_frisbee_state(pos[0], pos[1], pos[2], 0, 0, 0)
        log_frisbee_data(current_time, frisbee_state)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
workbook.close()
