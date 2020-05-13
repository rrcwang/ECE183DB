import numpy as np

def convert(rots):
    ''' takes inputs as phi, theta, gamma and returns in the format required by Webots
    '''
    axis_angle = np.empty((0,4))

    for i in range(rots.shape[0]):
        axis_ang = euler_xyz_to_axis_angle(rots[i,:])
        
        axis_angle = np.append(axis_angle, np.array([axis_ang]), axis=0)

    swap_col = np.copy(axis_angle[:,0])
    axis_angle[:,0] = axis_angle[:,1]
    axis_angle[:,1] = swap_col

    return axis_angle

# code adapted from https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToAngle/
def euler_xyz_to_axis_angle(r_e, normalize=True):
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
