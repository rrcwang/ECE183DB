import numpy as np
import quaternion

def euler_to_quaternion(roll, pitch, yaw):
    '''converts the xyz euler angle representation to quaternions'''
    cr = np.cos(roll*0.5)
    sr = np.sin(roll*0.5)
    cp = np.cos(pitch*0.5)
    sp = np.sin(pitch*0.5)
    cy = np.cos(yaw*0.5)
    sy = np.sin(yaw*0.5)

    q = np.quaternion(1,0,0,0)
    q.w = cr*cp*cy - sr*sp*sy 
    q.x = sr*cp*cy + cr*sp*sy
    q.y = cr*sp*cy - sr*cp*sy
    q.z = sr*sp*cy + cr*cp*sy

    return q.normalized()

def matrix_to_quaternion(matrix):
    '''converts rotation matrix to quaternion'''
    

def rotation_rate_to_quaternion(droll, dpitch, dyaw):
    '''converts the xyz euler angle derivatives to quaternions, dq/dt'''
    pass
    #dw_L = np.dot(dw_D,R)
    #q_dot = 0.5*dw_L*q
