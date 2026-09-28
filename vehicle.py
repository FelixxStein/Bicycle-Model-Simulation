import numpy as np
class vehicle:
    def __init__(self,
                 max_speed=100, 
                 starting_velocity=0, 
                 starting_acceleration=0, 
                 starting_x=0, 
                 starting_y=0, 
                 starting_steering_angle=0, 
                 starting_vehicle_angle=0, 
                 wheel_base=2, 
                 delta_t=0.01,
                 max_acceleration=14, 
                max_brake=35):
        self.velocity = starting_velocity #in m/s
        self.max_acceleration=max_acceleration
        self.max_brake=max_brake
        self.max_speed=max_speed
        self.acceleration = starting_acceleration # in m/s^2
        self.position=np.array([starting_x,starting_y], dtype=float)
        self.wheel_base=wheel_base
        self.steering_angle = starting_steering_angle
        self.vehicle_angle= starting_vehicle_angle
        self.delta_t=delta_t

    def time_step(self):
        self.position[0] = self.position[0] + self.velocity*np.cos(self.vehicle_angle*(np.pi/180)) * self.delta_t
        self.position[1] = self.position[1] + self.velocity*np.sin(self.vehicle_angle*(np.pi/180)) * self.delta_t
        self.velocity =min(self.max_speed,self.velocity+self.acceleration * self.delta_t)
        self.vehicle_angle=self.vehicle_angle+(((self.velocity*np.tan(self.steering_angle*(np.pi/180)))/self.wheel_base) * self.delta_t)*(180/np.pi)

    def update_acceleration(self, acceleration):
        if acceleration>0:
            self.acceleration=min(self.max_acceleration,acceleration)
        else:
            self.acceleration=-min(self.max_brake,abs(acceleration))

    def update_steering_angle(self,steering_angle):
        self.steering_angle=min(20,steering_angle)

    def get_position(self):
        return list(self.position)

    def get_vehicle_angle(self):
        return self.vehicle_angle

    def get_velocity(self):
        return self.velocity
    def get_wheelbase(self):
        return self.wheel_base

    def get_max_brake(self):
        return self.max_brake

    def get_max_speed(self):
        return self.max_speed

    def set_starting_angle(self, racetrack):
        vector=racetrack[1]-racetrack[0]
        self.vehicle_angle=np.atan2(vector[1],vector[0])*(180/np.pi)


class dynamic_vehicle(vehicle):
    def __init__(self,
             max_speed=100, 
             starting_vx=0,              
             starting_vy=0,              
             starting_yaw_rate=0,        
             starting_acceleration=0, 
             starting_x=0, 
             starting_y=0, 
             starting_steering_angle=0, 
             starting_vehicle_angle=0,  
             mass=800,
             COG_to_front_axis=1.5, 
             COG_to_rear_axis=2.0,       
             yaw_moment_of_inertia=1000,
             front_cornering_stiffness=3140,
             rear_cornering_stiffness=4000,  
             delta_t=0.01,
             max_acceleration=14, 
             max_brake=10):
        calculated_wheel_base=COG_to_front_axis+COG_to_rear_axis
        starting_velocity=starting_vx+starting_vy
        super().__init__(max_speed, starting_velocity,starting_acceleration, starting_x, starting_y, starting_steering_angle, starting_vehicle_angle, calculated_wheel_base, delta_t, max_acceleration, max_brake)
        self.mass=mass
        self.yaw_moment_of_inertia=yaw_moment_of_inertia
        self.front_cornering_stiffness=front_cornering_stiffness
        self.rear_cornering_stiffness=rear_cornering_stiffness
        self.yaw_rate=starting_yaw_rate
        self.vx=starting_vx
        self.vy=starting_vy
        self.COG_to_front_axis=COG_to_front_axis
        self.COG_to_rear_axis=COG_to_rear_axis

    def time_step(self):
        front_slip_angle=self.steering_angle-np.degrees(np.arctan((self.vy+self.COG_to_front_axis*np.radians(self.yaw_rate))/max(0.5,self.vx)))
        rear_slip_angle=-np.degrees(np.arctan((self.vy-self.COG_to_rear_axis*np.radians(self.yaw_rate))/max(0.5,self.vx)))
        front_lateral_force=self.front_cornering_stiffness*front_slip_angle
        rear_lateral_forces=self.rear_cornering_stiffness*rear_slip_angle
        v_dot_x=self.acceleration+self.vy*np.radians(self.yaw_rate)-(front_lateral_force*np.sin(np.radians(self.steering_angle)))/self.mass
        v_dot_y=((front_lateral_force*np.cos(np.radians(self.steering_angle))+rear_lateral_forces)/self.mass)-self.vx*np.radians(self.yaw_rate)
        yaw_acceleration=np.degrees((self.COG_to_front_axis*front_lateral_force*np.cos(np.radians(self.steering_angle))-self.COG_to_rear_axis*rear_lateral_forces)/self.yaw_moment_of_inertia)
        self.vx+=v_dot_x*self.delta_t
        self.vy+=v_dot_y*self.delta_t
        self.vehicle_angle += self.yaw_rate * self.delta_t
        self.yaw_rate+=yaw_acceleration*self.delta_t
        self.position[0]+=(self.vx*np.cos(np.radians(self.vehicle_angle))-self.vy*np.sin(np.radians(self.vehicle_angle)))*self.delta_t
        self.position[1]+=(self.vx*np.sin(np.radians(self.vehicle_angle))+self.vy*np.cos(np.radians(self.vehicle_angle)))*self.delta_t

    def get_velocity(self):
        return np.sqrt(self.vx**2+self.vy**2)


