import numpy as np

def convert(q):
    q = q.normalized()
    denominator = np.sqrt(1-(q.w)**2)
    a = 2*np.arccos(q.w)
    if a < 0.005:
        return [0,1,0,0]
    x = q.x / denominator
    y = q.y / denominator
    z = q.z / denominator

    magnitude = np.sqrt(x**2 + y**2 + z**2)
    x, y, z = x / magnitude, y / magnitude, z / magnitude

    return [x,y,z,a]