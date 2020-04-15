"""altino_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Keyboard
from altino import Altino
import csv
import cv2
import xlsxwriter

def log_gps_data(time, vector):
    with open('log.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time, vector[0], vector[1], vector[2]])

def log_lidar_data(image):
    workbook = xlsxwriter.Workbook('lidar_data.xlsx')
    row = 0
    for col, data in enumerate(image):
        worksheet.write_column(row, col, data)

# create the Robot instance.
altino = Altino()

# clear log, add column headers
f = open('log.csv', "w+")
with open('log.csv', 'a', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["t(ms)", "x", "y", "z"])

# create workbook for lidar data
workbook = xlsxwriter.Workbook('lidar_data.xlsx')
worksheet = workbook.add_worksheet()

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
    key = keyboard.getKey()
    if key == Keyboard.UP:
        altino.forward(50)
    elif key == Keyboard.DOWN:
        altino.back(20)
    elif key == Keyboard.LEFT:
        steer -= 0.2
    elif key == Keyboard.RIGHT:
        steer += 0.2
    else:
        altino.forward(0)
    
    altino.set_steer(steer)
    gps_data = altino.gps.getValues()
    log_gps_data(current_time, gps_data)
    lidar_data = altino.lidar.getRangeImageArray()
    current_time += timestep
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
workbook.close()
