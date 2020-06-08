"""This Module contains the CV functions for the Frisbee-Catching Robot"""
import numpy as np
import cv2
import math

BUFFER_SIZE = 10
TIME_STEP = 64

class Frisbee_State():
    def __init__(self, x, y, z, dx, dy, dz, roll, pitch, yaw, d_roll, d_pitch, d_yaw):
        # define state variables
        self.x = x
        self.y = y
        self.z = z
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.d_roll = d_roll
        self.d_pitch = d_pitch
        self.d_yaw = d_yaw

# computer vision functions
def sph_to_cart(r, theta, phi):
    """Converts spherical coordinates to cartesians"""
    x = r*math.cos(theta)*math.sin(phi)
    y = r*math.sin(theta)*math.sin(phi)
    z = r*math.cos(phi)
    return x, y, z

def img_to_cart(img, row, col):
    """Given a range image and a pixel location, returns coords of point"""
    c_factor = 1.367*10**-3 # radians per pixel for 80 degree FOV and a 1024 x 512 image
    r = img[row][col]
    theta = math.pi/2 + ((row - 512)*c_factor)
    phi = math.pi/2 - ((col - 256)*c_factor)
    return sph_to_cart(r, theta, phi)

def cv_detect(altino, range_data):
    """Update Computer Vision Display with new image (deprecated)"""
    g_lower = np.array([20, 120, 20])
    g_upper = np.array([100, 255, 100])
    g_dark_lower = np.array([])
    g_dark_upper = np.array([])

    # process image
    frame = cv2.imread("frame.png", 1)

    # create mask
    mask = cv2.inRange(frame, g_lower, g_upper)

    # retrieve pixel position of green dot
    px, py = get_xy(mask)

    #create visualization
    #output = cv2.bitwise_and(frame, frame, mask=mask)
    #cv2.imshow('output', output)
    #cv2.waitKey(32)
    altino.display.setColor((0x000000))
    altino.display.fillRectangle(0, 0, 512, 272)
    altino.display.setColor((0x00FF00))
    altino.display.fillRectangle(int(px), int(py), 10, 10)

    # calculate frisbee position
    coords = img_to_cart(range_data, 2*int(px), 2*int(py))
    return coords

def get_xy(mask):
    """calculate and return pixel position of dot of given color"""

    #find nonzero indicies
    ind = np.nonzero(mask)
    if ind[0].size != 0 and ind[1].size != 0:
        y = np.average(ind[0])
        x = np.average(ind[1])
    else:
        x = 0
        y = 0
    return x, y

def wb_detect(altino, colors):
    """Retrieve State of Frisbee from Webots Built-In Image Recognition"""
    objects = altino.camera.getRecognitionObjects()
    for i in range(len(objects)):
        if objects[i].get_colors() == colors:
            return objects[i]

# frisbee state data processing
data_buffer = []

def average(lst):
    return sum(lst)/len(lst)

def moving_average(data_buffer):
    """Returns the average of the Frisbee_State parameters in the data buffer
       Computes first order differential state variables based on a moving average"""
    # state variables
    x_list = []
    y_list = []
    z_list = []
    dx_list = []
    dy_list = []
    dz_list = []
    roll_list = []
    pitch_list = []
    yaw_list = []
    d_roll_list = []
    d_pitch_list = []
    d_yaw_list = []

    for i in range(len(data_buffer)):
        x_list.append(data_buffer[i].x)
        y_list.append(data_buffer[i].y)
        z_list.append(data_buffer[i].z)
        dx_list.append(data_buffer[i].dx)
        dy_list.append(data_buffer[i].dy)
        dz_list.append(data_buffer[i].dz)
        roll_list.append(data_buffer[i].roll)
        pitch_list.append(data_buffer[i].pitch)
        yaw_list.append(data_buffer[i].yaw)
        d_roll_list.append(data_buffer[i].d_roll)
        d_pitch_list.append(data_buffer[i].d_pitch)
        d_yaw_list.append(data_buffer[i].d_yaw)

    x = average(x_list)
    y = average(y_list)
    z = average(z_list)
    dx = average(dx_list)
    dy = average(dy_list)
    dz = average(dz_list)
    roll = average(roll_list)
    pitch = average(pitch_list)
    yaw = average(yaw_list)
    d_roll = average(d_roll_list)
    d_pitch = average(d_pitch_list)
    d_yaw = average(d_yaw_list)

    return Frisbee_State(x, y, z, dx, dy, dz, roll, pitch, yaw, d_roll, d_pitch, d_yaw)

def get_frisbee_state(x, y, z, roll, pitch, yaw):
    """returns moving average of the frisbee state based on incoming data
       call this function every timestep to process frisbee data"""
    # Handle first measurement
    if not data_buffer:
        for i in range(BUFFER_SIZE):
            data_buffer.append(Frisbee_State(x, y, z, 0, 0, 0, roll, pitch, yaw, 0, 0, 0))
    
    # Handle incoming measurement
    dx_temp = (x - data_buffer[BUFFER_SIZE - 1].x)/(TIME_STEP*10**-3)
    dy_temp = (y - data_buffer[BUFFER_SIZE - 1].y)/(TIME_STEP*10**-3)
    dz_temp = (z - data_buffer[BUFFER_SIZE - 1].z)/(TIME_STEP*10**-3)
    d_roll_temp = (roll - data_buffer[BUFFER_SIZE - 1].roll)/(TIME_STEP*10**-3)
    d_pitch_temp = (pitch - data_buffer[BUFFER_SIZE - 1].pitch)/(TIME_STEP*10**-3)
    d_yaw_temp = (yaw - data_buffer[BUFFER_SIZE - 1].yaw)/(TIME_STEP*10**-3)
    data_buffer.append(Frisbee_State(x, y, z, dx_temp, dy_temp, dz_temp, roll, pitch, yaw, d_roll_temp, d_pitch_temp, d_yaw_temp))
    data_buffer.pop(0)

    return moving_average(data_buffer)
