"""This Module contains the CV functions for the Frisbee-Catching Robot"""
import numpy as np
import cv2
import math
import altino

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
    """Update Computer Vision Display with new image"""
    g_lower = np.array([20, 100, 20])
    g_upper = np.array([120, 255, 140])

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
    print(coords)

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
