import numpy as np

vz = float(input("Input frisbee velocity in z direction: "))

vx = float(input("Input frisbee velocity in y direction: "))

pitch = float(input("Input frisbee pitch (in radians): "))

roll = float(input("Input frisbee roll (in radians): "))

spin_rate = float(input("Input frisbee rotation rate (in rad/s): "))

init_cond = [   0,      -4.2,   1,          # x, y, z
                vx,     vz,     0,          # dx, dy, dz
                roll,   pitch,  0,     # phi, theta, gamma
                0,      0,      15 ]

np.savetxt("183DASimul/controllers/frisbee_controller/init_conditions.csv",init_cond,delimiter=',')
