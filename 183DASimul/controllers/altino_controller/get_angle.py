import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import skimage
import skimage.transform

disc_ecc_ratio = 1.2630053228734255

img = cv.imread('get_angle/x0_z0.png',0)
edges = cv.Canny(img,100,200)

boundary = np.argwhere(edges != 0)
disc_box = cv.fitEllipse(boundary)

# Window name in which image is displayed 
window_name = 'Image'
startAngle = 0
center = (disc_box[0][1],disc_box[0][0])
axes = (disc_box[1][1],disc_box[1][0])
angle = disc_box[2]
box = (center, axes, angle)
endAngle = 360
# Red color in BGR 
color = (0, 0, 255)
   
# Line thickness of 5 px 
thickness = 2

# Using cv2.ellipse() method 
# Draw a ellipse with red line borders of thickness of 5 px 
img = np.stack((img,img,img),axis=2)
image = cv.ellipse(img, box, color, thickness)

# Calculate the tilt angle
height = 

# Displaying the image
cv.imshow(window_name, image)
cv.waitKey(5000)
cv.destroyAllWindows()
