import FrisPy
import numpy as np

timestep = 32
sim_times = np.linspace(0,157,158) * timestep / 1000

disc = FrisPy.create_disc(filename = "simple_initial_conditions.txt")
times, trajectory = FrisPy.get_trajectory(disc, times = sim_times)

np.savetxt("trajectory.csv", trajectory, delimiter=',')


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(trajectory[:,0],trajectory[:,1],trajectory[:,2])
plt.show()