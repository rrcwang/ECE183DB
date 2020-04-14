"""altino device tags and methods"""

from controller import Robot
from controller import Motor

class Altino(Robot):

    def __init__(self):
        super(Altino, self).__init__()
        self.timeStep = 32
        self.maxSpeed = 20
        self.maxSteer = 1.34
        self.minSteer = -1.34

        # get motor device tags
        self.front_left_motor = self.getMotor('left_front_wheel')
        self.front_right_motor = self.getMotor('right_front_wheel')
        self.back_left_motor = self.getMotor('left_rear_wheel')
        self.back_right_motor = self.getMotor('right_rear_wheel')
        self.left_steer = self.getMotor('left_steer')
        self.right_steer = self.getMotor('right_steer')
    
    def forward(self, velocity):
        """turns all wheels forward at specified velocity"""
        if velocity <= self.maxSpeed:
            Motor.setPosition(self.front_left_motor, float('inf'))
            Motor.setPosition(self.front_right_motor, float('inf'))
            Motor.setPosition(self.back_left_motor, float('inf'))
            Motor.setPosition(self.back_right_motor, float('inf'))

            Motor.setVelocity(self.front_left_motor, velocity)
            Motor.setVelocity(self.front_right_motor, velocity)
            Motor.setVelocity(self.back_left_motor, velocity)
            Motor.setVelocity(self.back_right_motor, velocity)

    def back(self, velocity):
        """turns all wheels backwards at specified velocity"""
        if velocity <= self.maxSpeed:
            Motor.setPosition(self.front_left_motor, float('inf'))
            Motor.setPosition(self.front_right_motor, float('inf'))
            Motor.setPosition(self.back_left_motor, float('inf'))
            Motor.setPosition(self.back_right_motor, float('inf'))

            Motor.setVelocity(self.front_left_motor, -1*velocity)
            Motor.setVelocity(self.front_right_motor, -1*velocity)
            Motor.setVelocity(self.back_left_motor, -1*velocity)
            Motor.setVelocity(self.back_right_motor, -1*velocity)

    def set_steer(self, angle):
        """sets angle of front wheels"""
        if  self.minSteer <= angle <= self.maxSteer:
            Motor.setPosition(self.left_steer, angle)
            Motor.setPosition(self.right_steer, angle)