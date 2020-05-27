import numpy as np
import quaternion
import numpy.linalg as linalg
import quaternion_to_AA

data = np.genfromtxt("traj1.csv", delimiter=',')

quaternion_rotation = data[6:10]
axis_angle_rotation = []
for i in range(data.shape[0]):
    row = data[i,:]
    q = np.quaternion(row[0],row[1],row[2],row[3])
    axis_angle = quaternion_to_AA.convert(q)

    axis_angle_rotation.append(axis_angle)


# reorient axes
swap_col = np.copy(data[:,1])
data[:,1] = data[:,2]
data[:,2] = swap_col

data = data * 0.2

swap_col = np.copy(data[:,7])
data[:,7] = data[:,8]
data[:,8] = swap_col

data[:,6:10] = axis_angle_rotation

data = data[::8,:]

np.savetxt('traj_final.csv',data,delimiter=',')
