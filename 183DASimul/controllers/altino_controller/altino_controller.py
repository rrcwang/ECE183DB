"""altino_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Keyboard
import altino as al
import camera_detection as cd
import numpy as np
import pure_pursuit as pp
import csv
import cv2
import convert_angles

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
alti = al.Altino()
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
timestep = alti.timeStep

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
alti.enable_sensors()
keyboard = Keyboard()
keyboard.enable(10)
steer = 0
current_time = 0
#path = pp.get_path()
#path = pp.enhance_path([0, -4.2], path)
pos = alti.gps.getValues() # get initial position

initial_state_guess = [ 0,      -4.2,       1,        # x, y, z
                        5,      7,          0,          # dx, dy, dz
                        0,      -0.3,       0,          # phi, theta, gamma
                        0,      0,          100 ]
alti.initialize_state_estimator(initial_state_guess)
state_estimate = initial_state_guess
state_estimator = alti.getSE()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while alti.step(timestep) != -1:
    # Read the sensors:
    gps_data      = alti.gps.getValues()
    range_data    = alti.range_finder.getRangeImageArray()
    frisbee_data  = cd.wb_detect(alti, [0, 1, 0])
    bearing       = alti.get_bearing()    
    car_position  = alti.gps.getValues()    
    current_time += timestep
    
    frisbee_measurement = [0,0,0,0,0,0]
    if frisbee_data is not None:
        xyz = frisbee_data.get_position()

        # read orientations
        orientation = frisbee_data.get_orientation()
        car_rotation = [0,1,0,bearing]
        sensor_rotation = [0, -0.7071, 0.7071, 3.14159]

        # rotate into world frame
        q_orientation = convert_angles.aa2quaternion(orientation)
        q_car_rotation = convert_angles.aa2quaternion(car_rotation)
        q_sensor_rotation = convert_angles.aa2quaternion(sensor_rotation)

        q1 = q_sensor_rotation.inverse() * q_orientation # rotate into car frame
        q = q_car_rotation.inverse() * q1 # rotate into world frame

        frisbee_orientation_euler = convert_angles.quaternion2euler(q)
        
        location_measurement = [car_position[0]-xyz[0], car_position[1]-xyz[2]+0.05, car_position[2]-xyz[1]]
        frisbee_measurement = location_measurement + frisbee_orientation_euler

    state_estimator.dynamics_propagation()
    state_estimator.measurement_update(frisbee_measurement)

    SE = state_estimator.get_state().aslist()
    print(SE)

    #altino.camera.getImage()
    #camera_status = altino.camera.saveImage('frame.png', 0)
    current_time += timestep

    if current_time % 5:
        ase = state_estimator.predict_path(SE)
        print(ase)

    # Read Path
    path_raw = np.loadtxt('../frisbee_controller/position_data.csv', delimiter = ',')
    path = np.delete(path_raw, 1, 1).tolist()
    path = pp.enhance_path(car_position, path)
    
    # Pure Pursuit
    pp.pp_update(alti, (gps_data[0], gps_data[2]), alti.get_bearing(), path, current_time/2)
    
    # Process sensor data here.
    log_gps_data(current_time, gps_data)
    pass

# Enter here exit cleanup code.
workbook.close()
