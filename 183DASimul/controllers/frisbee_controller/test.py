import FrisPy_quat
import FrisPy
import FrisPy_q2
import numpy as np

init_conditions = [ 0,      -4.2,       1,        # x, y, z
                    0,      10,          0,          # dx, dy, dz
                    0,      0,          0,          # phi, theta, gamma
                    0,      0,          100 ]       # phidot, thetadot, gammadot

disc1 = FrisPy_quat.create_disc(initial_conditions = init_conditions)
times, trajectory1 = FrisPy.get_trajectory(disc1, full_trajectory=True)

disc2 = FrisPy.create_disc(initial_conditions = init_conditions)
times, trajectory2 = FrisPy.get_trajectory(disc2, full_trajectory=True)

disc3 = FrisPy_q2.create_disc(initial_conditions = init_conditions)
times, trajectory3 = FrisPy_q2.get_trajectory(disc3, full_trajectory=True)

np.savetxt('traj1.csv',trajectory1,delimiter=',')
np.savetxt('traj2.csv',trajectory2,delimiter=',')
np.savetxt('traj3.csv',trajectory3,delimiter=',')

# DEBUG, visualizer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(trajectory1[:,0],trajectory1[:,1],trajectory1[:,2])
ax.plot(trajectory2[:,0],trajectory2[:,1],trajectory2[:,2])
ax.plot(trajectory3[:,0],trajectory3[:,1],trajectory3[:,2])

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_xlim(0,20)
ax.set_ylim(10,-10)
ax.set_zlim(0,3)
plt.show()
