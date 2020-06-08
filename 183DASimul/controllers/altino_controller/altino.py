"""altino device tags and methods"""

from controller import Robot
from controller import Motor
from controller import Camera
import numpy as np
import math
import state_estimator

# Altino Robot Class
class Altino(Robot):

    def __init__(self):
        super(Altino, self).__init__()
        # define constants
        self.timeStep = 32
        self.maxSpeed = 200
        self.maxSteer = 1.34
        self.minSteer = -1.34

        # define operational parameters
        self.target_speed = 0
        self.target_radius = np.inf
        self.radius_pos = np.array([np.inf, 0])
        self.left_rear_pos = np.array([-0.04, 0.1])
        self.right_rear_pos = np.array([0.04, 0.1])
        self.middle_rear_pos = np.array([0, 0.1])

        # get motor device tags
        self.rear_left_motor = self.getMotor('left_rear_wheel')
        self.rear_right_motor = self.getMotor('right_rear_wheel')
        self.left_steer = self.getMotor('left_steer')
        self.right_steer = self.getMotor('right_steer')

        # set wheels' initial parameters
        Motor.setPosition(self.rear_left_motor, float('inf'))
        Motor.setPosition(self.rear_right_motor, float('inf'))
        Motor.setVelocity(self.rear_right_motor, 0)
        Motor.setVelocity(self.rear_left_motor, 0)
        Motor.setAcceleration(self.rear_right_motor, 400)
        Motor.setAcceleration(self.rear_left_motor, 400)

        # get sensors
        self.gps = self.getGPS('gps')
        self.range_finder = self.getRangeFinder('range finder')
        self.camera = self.getCamera('camera')
        self.camera_led = self.getLED('camera led')
        self.compass = self.getCompass('compass')

        # get display for visualizations
        self.display = self.getDisplay('display')

    # Motor Functions
    def set_speed(self, speed):
        """sets target speed"""
        if self.maxSpeed*-1 <= speed <= self.maxSpeed:
            self.target_speed = speed
            self.actuate()
        else:
            print("error: requested speed exceeds max speed: ", self.maxSpeed)
            self.target_speed = self.maxSpeed
            self.actuate()

    def set_steer(self, radius):
        """sets target turning radius"""
        # need steering compensation for speed
        if self.target_speed != 0:
            adjusted_radius = radius/(self.target_speed/10)
        else:
            adjusted_radius = radius
        self.target_radius = adjusted_radius
        self.radius_pos = np.array([adjusted_radius, 0])
        self.actuate()

    def actuate(self):
        """calculates wheel speeds and steers based on target speed and radius"""
        #print("target speed: ", self.target_speed)
        #print("target radius: ", self.target_radius)
        # Check for infinite turn radius
        if self.target_radius == float('inf'):
            Motor.setPosition(self.left_steer, 0)
            Motor.setPosition(self.right_steer, 0)
            Motor.setVelocity(self.rear_right_motor, self.target_speed)
            Motor.setVelocity(self.rear_left_motor, self.target_speed)
            return

        # Calculate wheel angles with target radius
        if self.target_radius > 0:
            angle_right = np.pi/2 - angle(np.array([0, -1]), self.radius_pos - self.right_rear_pos)
            angle_left = np.pi/2 - angle(np.array([0, -1]), self.radius_pos - self.left_rear_pos)
        else:
            angle_right = angle(np.array([0, -1]), self.radius_pos - self.right_rear_pos) - np.pi/2
            angle_left = angle(np.array([0, -1]), self.radius_pos - self.left_rear_pos) - np.pi/2
        #print("angle right: ", angle_right)
        #print("angle left: ", angle_left)
        # Calculate wheel speeds based on target radius
        rear_right_speed = (self.target_speed/self.target_radius)*(self.target_radius - 0.04)
        rear_left_speed = (self.target_speed/self.target_radius)*(self.target_radius + 0.04)

        # Check for valid turning radii, speed and set steer, speed
        if abs(angle_right) <= self.maxSteer and abs(angle_left) <= self.maxSteer:
            Motor.setPosition(self.left_steer, angle_left)
            Motor.setPosition(self.right_steer, angle_right)
        else:
            print("correcting radius...")
            if angle_right > self.maxSteer or angle_left > self.maxSteer:
                self.target_radius += 0.5
                self.radius_pos = np.array([self.target_radius, 0])
                self.actuate()
            elif angle_right < self.minSteer or angle_left < self.minSteer:
                self.target_radius -= 0.5
                self.radius_pos = np.array([self.target_radius, 0])
                self.actuate()
            self.target_speed = 20

        if abs(rear_left_speed) <= self.maxSpeed and abs(rear_right_speed) <= self.maxSpeed:
            Motor.setVelocity(self.rear_left_motor, rear_left_speed)
            Motor.setVelocity(self.rear_right_motor, rear_right_speed)
        else:
            print("error: requested max speed too large")
            self.target_speed = 0

    # Sensor Functions
    def enable_sensors(self):
        """enables all sensors"""
        self.gps.enable(self.timeStep)
        self.range_finder.enable(self.timeStep)
        self.camera.enable(self.timeStep)
        self.camera.recognitionEnable(self.timeStep)
        self.compass.enable(self.timeStep)
        self.camera_led.set(1)

    def get_bearing(self):
        """Returns bearing in degrees. Code inspired by sample code
        at https://www.cyberbotics.com/doc/reference/compass?tab-language=python"""
        north = self.compass.getValues()
        rad = math.atan2(north[0], north[2])
        
        return np.pi/2 - rad

    def initialize_state_estimator(self,initial_state_estimate):
        """Initialize state estimator with given state estimate
        """
        self.state_estimator = state_estimator.StateEstimator(initial_state_estimate)

    def getSE(self):
        return self.state_estimator
    

def distance(p1, p2):
    """returns distance between two points"""
    return np.linalg.norm(p2 - p1)

def angle(v1, v2):
    """returns angle between two vectors"""
    return np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
