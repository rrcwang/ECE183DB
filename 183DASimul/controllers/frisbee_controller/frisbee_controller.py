"""frisbee_flight controller."""

# importing the Supervisor module
from controller import Robot
from controller import Supervisor
import FrisPy
import numpy as np
import convert_angles

# create the Supervisor instance.
supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
print(timestep)

# instantiate object handles for the frisbee
frisbee_node = supervisor.getFromDef("frisbee")
trans_field = frisbee_node.getField("translation")
rotation_field = frisbee_node.getField("rotation")

# frisbee simulation time variables
frisbee_timestep = 6
N = 1000
scaling_factor = 0.2

sim_times = np.linspace(0,N,N+1) * frisbee_timestep / 1000

init_conditions = np.genfromtxt("init_conditions.csv").tolist()
init_conditions[6:9] = [0,0,0]

# generate flight path trajectory
disc = FrisPy.create_disc(initial_conditions = init_conditions)
times, trajectory = FrisPy.get_trajectory(disc, full_trajectory=True, times=sim_times) #times = sim_times, 
position_data = trajectory[:,0:3]# * scaling_factor

# format position and rotation data
position_data = trajectory[:,0:3]

swap_col = np.copy(position_data[:,2])
position_data[:,2] = position_data[:,1]
position_data[:,1] = swap_col
#swap_cols = np.copy(position_data[:,0:2])
#position_data[:,0] = position_data[:,2]
#position_data[:,1:3] = swap_cols

# position data
np.savetxt("position_data.csv",position_data,delimiter=',')

rot = trajectory[:,6:9]
rotation_data = convert_angles.convert_euler2aa(rot)


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

            #print(trajectory[time_index,:])
            
            time_index += 1
    except(IndexError):
        print("End of trajectory file reached")
        read = False
    pass



# Enter here exit cleanup code.
