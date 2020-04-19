"""frisbee_flight controller."""

# importing the Supervisor module
from controller import Robot
from controller import Supervisor
import numpy as np

# create the Supervisor instance.
supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
print(timestep)

# instantiate object handles for the frisbee
frisbee_node = supervisor.getFromDef("test_frisbee")
trans_field = frisbee_node.getField("translation")
rotation_field = frisbee_node.getField("rotation")

# import the frisbee simulation data
data = np.genfromtxt("trajectory.csv", delimiter=',')
position_data = data[:,0:3]
rotation_data = np.genfromtxt("rotations.csv", delimiter=',')

time_index = 0
read = True
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    # this is done repeatedly
    try:
        if read:
            # update the position and rotation of the frisbee
            position = position_data[time_index,:].tolist()
            trans_field.setSFVec3f(position)
            
            rotation = rotation_data[time_index,:].tolist()
            rotation_field.setSFRotation(rotation)
            
            time_index += 1
    except(IndexError):
        print("End of trajectory file reached")
        read = False
    pass



# Enter here exit cleanup code.
