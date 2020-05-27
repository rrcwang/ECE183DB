import FrisPy_quat
import FrisPy
import numpy as np

init_conditions = [ 0,      -4.2,       1,        # x, y, z
                    5,      7,          0,          # dx, dy, dz
                    0,      -0.3,          0,          # phi, theta, gamma
                    0,      0,          100 ]       # phidot, thetadot, gammadot

disc1 = FrisPy_quat.create_disc(initial_conditions = init_conditions)
times, trajectory1 = FrisPy.get_trajectory(disc1, full_trajectory=True)

disc2 = FrisPy.create_disc(initial_conditions = init_conditions)
times, trajectory2 = FrisPy.get_trajectory(disc2, full_trajectory=True)

np.savetxt('traj1.csv',trajectory1,delimiter=',')
np.savetxt('traj2.csv',trajectory2,delimiter=',')

# DEBUG, visualizer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(trajectory1[:,0],trajectory1[:,1],trajectory1[:,2])
ax.plot(trajectory2[:,0],trajectory2[:,1],trajectory2[:,2])

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_xlim(0,20)
ax.set_ylim(10,-10)
ax.set_zlim(0,3)
plt.show()
