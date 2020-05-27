import numpy as np
import math
import csv

LOOKAHEAD_DISTANCE = 0.5
SPEED = 40

def get_path():
    """Retrieves the path of points from path.csv. Call this from the
    controller file and pass the path to pp_update"""
    path = np.loadtxt('path.csv', dtype='float', delimiter=',').tolist()
    return path

def sign(d):
    """Sign function used for lookahead circle-path intersection calculation"""
    if d < 0:
        return -1
    else:
        return 1

def is_in_bounds(p1, p2, p3):
    """Returns True if p2 is between p1 and p3, False otherwise"""
    ret = None
    if (min(p1[0], p3[0]) - 1*10**-6 <= p2[0] <= max(p1[0], p3[0]) + 1*10**-6) and \
       (min(p1[1], p3[1]) - 1*10**-6 <= p2[1] <= max(p1[1], p3[1]) + 1*10**-6):
        ret = True
    else:
        ret = False
    return ret

def distance(p1, p2):
    """returns distance between two points"""
    return np.linalg.norm(np.array(p2) - np.array(p1))


def get_intersection(center, p1, p2):
    """Calculates the intersection between the lookahead circle and
    a line segment defined by coordinates p1 and p2. Equations lifted
    from https://mathworld.wolfram.com/Circle-LineIntersection.html.
    Returns list of intersection points in line segment. If there are
    two intersects, returns the intersect closer to p2"""
    # find vectors relative to center
    point = None
    v1_x = p1[0] - center[0]
    v1_z = p1[1] - center[1]
    v2_x = p2[0] - center[0]
    v2_z = p2[1] - center[1]

    # Calculate distances
    d_x = v2_x - v1_x
    d_z = v2_z - v1_z
    d_r = np.sqrt(d_x**2 + d_z**2)
    D = (v1_x*v2_z) - (v2_x*v1_z)

    # Calculate intersection points
    det = (LOOKAHEAD_DISTANCE**2)*(d_r**2) - D**2
    if det == 0:
        # One intersection point
        x1 = ((D*d_z)/d_r**2) + center[0]
        z1 = ((-1*D*d_x)/d_r**2) + center[1]
        # Check that point is within bounds
        if is_in_bounds(p1, (x1, z1), p2):
            return (x1, z1)
    elif det > 0:
        # Two intersection points
        x1 = (D*d_z + sign(d_z)*d_x*np.sqrt(det))/(d_r**2) + center[0]
        z1 = (-1*D*d_x + abs(d_z)*np.sqrt(det))/(d_r**2) + center[1]
        x2 = (D*d_z - sign(d_z)*d_x*np.sqrt(det))/(d_r**2) + center[0]
        z2 = (-1*D*d_x - abs(d_z)*np.sqrt(det))/(d_r**2) + center[1]
        # Check if points are in bounds
        #print("point 1: ", (x1, z1))
        #print("point 2: ", (x2, z2))
        if is_in_bounds(p1, (x1, z1), p2) and not is_in_bounds(p1, (x2, z2), p2):
            point = (x1, z1)
        elif not is_in_bounds(p1, (x1, z1), p2) and is_in_bounds(p1, (x2, z2), p2):
            point = (x2, z2)
        elif is_in_bounds(p1, (x1, z1), p2) and is_in_bounds(p1, (x2, z2), p2):
            if distance([x1, z1], p2) <= distance([x2, z1], p2):
                point = (x1, z1)
            else:
                point = (x2, z2)
    # No Intersection points, leave point as None
    return point

def get_turning_radius(p1, p2, deg):
    """Calculates target turning radius to get to lookahead point
    p1 is the xz coordinate of the car, p2 is that lookahead point
    deg is degrees of rotation from the north((0, 1) in Webots) axis"""
    # Transform frame of reference
    #print("deg: ", deg)
    rad = deg*(math.pi/180)
    rot = np.array([[math.cos(rad), math.sin(rad)], [-1*math.sin(rad), math.cos(rad)]])
    v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
    v1 = rot @ v1
    #print("vec: ", v1)
    # Distance should always be equal to LOOKAHEAD_DISTANCE
    dist = np.sqrt(v1[0]**2 + v1[1]**2)

    # Calculate target turning radius
    radius = (dist**2)/(-2*v1[0])
    return radius

def pp_update(alti, pos, deg, path):
    """Pure pursuit update. Takes the position of the robot,
    its bearing and a path (list of xz points) and sets its
    turning radius and speed. It his highly recommended to
    append the robots starting position to the beginning of
    the path"""
    la_point = None
    pth = path

    # Calculate lookahead point from (# path points - 1) segments
    for i in range(len(pth) - 1):
        point = get_intersection(pos, pth[i], pth[i + 1])
        if point is not None:
            #print("retval: ", point)
            #print("segment: ", pth[i + 1])
            la_point = point
    #print("final la_point: ", la_point)
    #print("position: ", pos)
    
    radius = get_turning_radius(pos, la_point, deg)
    alti.set_steer(radius)
    alti.set_speed(SPEED)
