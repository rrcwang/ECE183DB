import FrisPy
import numpy as np

# using Webots default simulation time step of 32
timestep = 32
scaling_factor = 0.2
inital_offset = [0,0,0.06]
N = 100
sim_times = np.linspace(0,N,N+1) * timestep / 1000

trajectories = np.empty((30001,12,6))

# generate flight path trajectories
for i in range(1,6):
    disc = FrisPy.create_disc(filename = "test"+ str(i) + ".txt")
    times, trajectory = FrisPy.get_trajectory(disc, full_trajectory=True)
    trajectories[:,:,i] = trajectory

    # save data as csv
    np.savetxt("trajectory" + str(i) + ".csv", trajectory, delimiter=',')

trajectory[:,0:3] = trajectory[:,0:3] *scaling_factor + inital_offset

# save data
print(trajectory)

# DEBUG, visualizer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(2,3):
    ax.plot(trajectories[:,0,i],trajectories[:,1,i],trajectories[:,2,i])

# test the midpoint
disc = FrisPy.create_disc(filename = "midpoint1.txt")
times, trajectory = FrisPy.get_trajectory(disc, full_trajectory=True)
ax.plot(trajectory[:,0],trajectory[:,1],trajectory[:,2])

disc = FrisPy.create_disc(filename = "midpoint2.txt")
times, trajectory = FrisPy.get_trajectory(disc, full_trajectory=True)
ax.plot(trajectory[:,0],trajectory[:,1],trajectory[:,2])

disc = FrisPy.create_disc(filename = "midpoint3.txt")
times, trajectory = FrisPy.get_trajectory(disc, full_trajectory=True)
ax.plot(trajectory[:,0],trajectory[:,1],trajectory[:,2])

disc = FrisPy.create_disc(filename = "midpoint4.txt")
times, trajectory = FrisPy.get_trajectory(disc, times=sim_times, full_trajectory=True)

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_xlim(0,20)
ax.set_ylim(10,-10)
ax.set_zlim(0,3)
plt.show()