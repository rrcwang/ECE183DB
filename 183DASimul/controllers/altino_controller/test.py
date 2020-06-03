import numpy as np
import quaternion
import convert_angles


car_rotation = [0,1,0,0]
sensor_rotation = [1,0,0,1.5708]
frisbee_orientation = [0,1,0,0]
measured = [-1,0,0,1.5708]

q_measured = convert_angles.aa2quaternion(measured)
q_car_rotation = convert_angles.aa2quaternion(car_rotation)
q_sensor_rotation = convert_angles.aa2quaternion(sensor_rotation)
q_frisbee_orientation = convert_angles.aa2quaternion(frisbee_orientation)

q_car = q_sensor_rotation.inverse() * q_measured
print(q_car)
car = convert_angles.quaternion2aa(q_car)
print(car)


print()