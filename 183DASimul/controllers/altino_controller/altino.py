"""altino device tags and methods"""

from controller import Robot
from controller import Motor
from controller import Camera
import math

# Altino Robot Class
class Altino(Robot):

    def __init__(self):
        super(Altino, self).__init__()
        self.timeStep = 32
        self.maxSpeed = 50
        self.maxSteer = 1.34
        self.minSteer = -1.34

        # get motor device tags
        self.front_left_motor = self.getMotor('left_front_wheel')
        self.front_right_motor = self.getMotor('right_front_wheel')
        self.back_left_motor = self.getMotor('left_rear_wheel')
        self.back_right_motor = self.getMotor('right_rear_wheel')
        self.left_steer = self.getMotor('left_steer')
        self.right_steer = self.getMotor('right_steer')

        # get sensors
        self.gps = self.getGPS('gps')
        self.range_finder = self.getRangeFinder('range finder')
        self.camera = self.getCamera('camera')
        self.camera_led = self.getLED('camera led')

        # get display and attach camera
        self.display = self.getDisplay('display')

    # Motor Functions
    def set_speed(self, velocity):
        """turns all wheels at specified velocity"""
        if self.maxSpeed*-1 <= velocity <= self.maxSpeed:
            Motor.setPosition(self.front_left_motor, float('inf'))
            Motor.setPosition(self.front_right_motor, float('inf'))
            Motor.setPosition(self.back_left_motor, float('inf'))
            Motor.setPosition(self.back_right_motor, float('inf'))

            Motor.setVelocity(self.front_left_motor, velocity)
            Motor.setVelocity(self.front_right_motor, velocity)
            Motor.setVelocity(self.back_left_motor, velocity)
            Motor.setVelocity(self.back_right_motor, velocity)
        else:
            print("error: requested speed exceeds max speed: ", self.maxSpeed)

    def set_steer(self, angle):
        """sets angle of front wheels"""
        if  self.minSteer <= angle <= self.maxSteer:
            Motor.setPosition(self.left_steer, angle)
            Motor.setPosition(self.right_steer, angle)

    # Sensor Functions
    def enable_sensors(self):
        """enables all sensors"""
        self.gps.enable(32)
        self.range_finder.enable(32)
        self.camera.enable(32)
        self.camera_led.set(1)
