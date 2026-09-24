import numpy as np
class vehicle:
    def __init__(self, velocity=0, acceleration=0, starting_x=0, starting_y=0, starting_steering_angle=0, starting_vehicle_angle=0, wheel_base=1):
        self.velocity = velocity #in m/s
        self.acceleration = acceleration # in m/s^2
        self.x=starting_x
        self.y=starting_y
        self.wheel_base=wheel_base
        self.steering_angle = starting_steering_angle
        self.vehicle_angle= starting_vehicle_angle

    def update(self):
        self.x = self.x + self.velocity*np.cos(self.vehicle_angle)
        self.y = self.y + self.velocity*np.sin(self.vehicle_angle)
        self.velocity =self.velocity+self.acceleration
        self.vehicle_angle=self.vehicle_angle+(self.velocity*np.tan(self.steering_angle))/self.wheel_base

    