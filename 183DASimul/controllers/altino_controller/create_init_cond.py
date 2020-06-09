import numpy as np

vz = input("Input frisbee velocity in z direction: ")

vx = input("Input frisbee velocity in y direction: ")

pitch = input("Input frisbee pitch (in radians): ")

roll = input("Input frisbee roll (in radians): ")

spin_rate = input("Input frisbee rotation rate (in rad/s)")

init_cond = [   0,      -4.2,   1,          # x, y, z
                vx,     vz,     0,          # dx, dy, dz
                roll,   pitch,  0,     # phi, theta, gamma
                0,      0,      15 ]

np.savetxt("controllers\init_conditions",init_cond,delimiter=',')
