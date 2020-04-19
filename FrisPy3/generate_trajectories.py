import FrisPy
import numpy as np

# using Webots default simulation time step of 32
timestep = 32
scaling_factor = 0.2
inital_offset = [0,0,0.06]
N = 500
sim_times = np.linspace(0,N,N+1) * timestep / 1000

# generate flight path trajectory
disc = FrisPy.create_disc(filename = "test2.txt")
times, trajectory = FrisPy.get_trajectory(disc, full_trajectory=True)

trajectory[:,0:3] = trajectory[:,0:3] *scaling_factor + inital_offset

# save data
np.savetxt("trajectory.csv", trajectory, delimiter=',')
print(trajectory)

# DEBUG, visualizer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(trajectory[:,0],trajectory[:,1],trajectory[:,2])
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_xlim(0,3)
ax.set_ylim(3,0)
ax.set_zlim(0,1)
plt.show()