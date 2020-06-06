import numpy as np
import quaternion

def aa2euler(axis_angle):
    x,y,z,angle = axis_angle

    if np.abs(np.pi*2-angle) < 0.001:
        angle = 0

   # q_robot = np.quaternion(np.cos(heading/2),0,0,np.sin(heading/2))

    ca = np.cos(angle)
    sa = np.sin(angle)

    R =[[ ca+(1-ca)*x**2, x*y*(1-ca)-z*sa, x*z*(1-ca)+y*sa ],
        [ x*y*(1-ca)+z*sa, ca+(1-ca)*y**2, y*z*(1-ca)-x*sa ],
        [ x*z*(1-ca)-y*sa, y*z*(1-ca)+x*sa, ca+(1-ca)*z**2 ] ]

    R = np.array(R)
    
    st = R[2,0]
    ct = R[0,0]
    sp = R[1,2]
    cp = R[1,1]

    theta = np.arctan2(st, ct)
    phi = np.arctan2(sp, cp)

    if phi==np.pi/2:
        raise("Gimbal lock detected")

    return [phi, theta, 0]

import numpy as np

def convert_euler2aa(rots):
    ''' takes inputs as phi, theta, gamma and returns in the format required by Webots
    '''
    axis_angle = np.empty((0,4))

    for i in range(rots.shape[0]):
        axis_ang = euler2aa(rots[i,:]) #
        
        #if (i == 0):
            #d_ang = axis_ang[3]
        #else:
            #d_ang = (axis_ang[3] - axis_angle[i-1,3]) % (2 * np.pi)
        
        #axis_ang[3] = d_ang

        axis_angle = np.append(axis_angle, np.array([axis_ang]), axis=0)
    
    # xyz swap to zxy
    swap_cols = np.copy(axis_angle[:,0:2])
    axis_angle[:,0] = axis_angle[:,2]
    axis_angle[:,1:3] = swap_cols

    return axis_angle

# code adapted from https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToAngle/
def euler2aa(r_e, normalize=True):
    ''' convert xyz to axis angle rotations (defined in the original frisbee sim frame)
    '''
    x_e = r_e[0]
    y_e = r_e[1]
    z_e = r_e[2]
    # Assuming the angles are in radians.
    c1 = np.cos(x_e/2)
    s1 = np.sin(x_e/2)
    c2 = np.cos(y_e/2)
    s2 = np.sin(y_e/2)
    c3 = np.cos(z_e/2)
    s3 = np.sin(z_e/2)
    c1c2 = c1*c2
    s1s2 = s1*s2
    w = c1c2*c3 - s1s2*s3
    x = c1c2*s3 + s1s2*c3
    y = s1*c2*c3 + c1*s2*s3
    z = c1*s2*c3 - s1*c2*s3
    angle = 2 * np.arccos(w)
    if normalize:
        norm = x*x+y*y+z*z
        if norm < 0.001:
            # when all euler angles are zero angle = 0 so
            # we can set axis to anything to avoid divide by zero
            x = 1
            y = 0
            z = 0
        else:
            norm = np.sqrt(norm)
            x /= norm
            y /= norm
            z /= norm
    return [x, y, z, angle]

def quaternion2aa(q):
    ''' convert quaternions to axis angle
    '''
    q = q.normalized()    
    
    angle = 2 * np.arccos(q.w)
    s = np.sqrt(1-q.w*q.w)

    if (s < 0.001):
        x = 0
        y = 1
        z = 0
    else:
        x = q.x / s
        y = q.y / s
        z = q.z / s

    return [x,y,z,angle]

def aa2quaternion(aa):
    ''' convert axis angle to quaternion
    '''
    x,y,z,angle = aa

    sa2 = np.sin(angle/2)

    return np.quaternion(np.cos(angle/2),x*sa2,y*sa2,z*sa2)

    
def quaternion2euler(q):
    aa = quaternion2aa(q)
    return aa2euler(aa)

#def apply_aa_rotation(r1,r2):
