"""frisbee_flight controller."""

# importing the Supervisor module
from controller import Robot
from controller import Supervisor
import FrisPy
import numpy as np
import euler_to_AA

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
ground_offset = 0.25
sim_times = np.linspace(0,N,N+1) * frisbee_timestep / 1000
print(sim_times)

init_conditions = [ 0,      -4.2,       1,        # x, y, z
                    0,      6,          0,          # dx, dy, dz
                    1,      0,          0,          # phi, theta, gamma
                    0,      0,          100 ]       # phidot, thetadot, gammadot

# generate flight path trajectory
disc = FrisPy.create_disc(initial_conditions = init_conditions)
times, trajectory = FrisPy.get_trajectory(disc, full_trajectory=True, times=sim_times) #times = sim_times, 
position_data = trajectory[:,0:3] * scaling_factor

data = np.genfromtxt('traj_final.csv', delimiter=',')

position_data =data[:,0:3]
rotation_data = data[:,6:10]


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
