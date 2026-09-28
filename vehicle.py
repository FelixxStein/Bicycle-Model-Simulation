import numpy as np
class vehicle:
    def __init__(self, velocity=0, acceleration=0, starting_x=0, starting_y=0, starting_steering_angle=0, starting_vehicle_angle=0, wheel_base=2, delta_t=0.01,max_acceleration=14, max_break=45):
        self.velocity = velocity #in m/s
        self.max_acceleration=max_acceleration
        self.max_break=max_break
        self.acceleration = acceleration # in m/s^2
        self.position=np.array([starting_x,starting_y])
        self.wheel_base=wheel_base
        self.steering_angle = starting_steering_angle
        self.vehicle_angle= starting_vehicle_angle
        self.delta_t=delta_t

    def time_step(self):
        self.position[0] = self.position[0] + self.velocity*np.cos(self.vehicle_angle*(np.pi/180)) * self.delta_t
        self.position[1] = self.position[1] + self.velocity*np.sin(self.vehicle_angle*(np.pi/180)) * self.delta_t
        self.velocity =self.velocity+self.acceleration * self.delta_t
        self.vehicle_angle=self.vehicle_angle+(((self.velocity*np.tan(self.steering_angle*(np.pi/180)))/self.wheel_base) * self.delta_t)*(180/np.pi)

    def update_acceleration(self, acceleration):
        if acceleration>0:
            self.acceleration=min(self.max_acceleration,acceleration)
        else:
            self.acceleration=-min(self.max_break,abs(acceleration))

    def update_steering_angle(self,steering_angle):
        self.steering_angle=steering_angle

    def get_position(self):
        return list(self.position)

    def get_vehicle_angle(self):
        return self.vehicle_angle

    def get_velocity(self):
        return self.velocity
    def get_wheelbase(self):
        return self.wheel_base

    def set_starting_angle(self, racetrack):
        vector=racetrack[1]-racetrack[0]
        self.vehicle_angle=np.atan2(vector[1],vector[0])*(180/np.pi)
        