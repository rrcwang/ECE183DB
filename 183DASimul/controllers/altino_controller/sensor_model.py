import numpy as np

position_covar = 0.1
rotation_covar = 0.05

def add_noise(measurement):
    position_noise = np.random.normal(scale=position_covar)
    rotation_noise = np.random.normal(scale=rotation_covar)
    
    pos, rot = measurement[0:3], measurement[3:6]

    pos_noised = [ p + np.random.normal(scale=position_covar) for p in pos ]
    rot_noised = [ r + np.random.normal(scale=rotation_covar) for r in rot]

    return pos_noised + rot_noised