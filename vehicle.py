import numpy as np
import constants
def integrator(derivative_function ,state ,control_inputs, delta_t):
    change=derivative_function(state,control_inputs)
    state=state+change*delta_t
    return state
def tire_function(slip_angle,stiffness_factor,shape_factor,peak_factor,curvature_factor):
    return peak_factor*np.sin(shape_factor*np.arctan(stiffness_factor*slip_angle-curvature_factor*(stiffness_factor*slip_angle-np.arctan(stiffness_factor*slip_angle))))

class vehicle():
    def __init__(self,
                state,
                max_speed=100, 
                wheel_base=2, 
                max_acceleration=14, 
                max_brake=35):
        self.state=state
        self.max_speed=max_speed
        self.wheel_base=wheel_base
        self.max_acceleration=max_acceleration
        self.max_brake=max_brake

    def get_position(self):
        return np.array([self.state[0], self.state[1]])

    def time_step(self,control_inputs, delta_t):
        copy_control_inputs=control_inputs.copy()
        copy_control_inputs[0]=self.update_steering_angle(copy_control_inputs[0])
        copy_control_inputs[1]=self.update_acceleration(copy_control_inputs[1])
        state=integrator(self.derivative_function,self.state,copy_control_inputs, delta_t)
        state=self.apply_state_limits(state)
        self.state=state


    def get_yaw(self):
        return self.state[2]
    
    def get_wheelbase(self):
         return self.wheel_base
    
    def get_max_brake(self):
        return self.max_brake

    def get_max_speed(self):
        return self.max_speed

    def set_starting_angle(self, racetrack):
        vector=racetrack[1]-racetrack[0]
        self.state[2]=np.atan2(vector[1],vector[0])

    def update_acceleration(self, acceleration):
        return np.clip(acceleration,-self.max_brake,self.max_acceleration)

    def update_steering_angle(self,steering_angle):
        return np.clip(steering_angle,-0.35, 0.35)

    


    
class kinematic_vehicle(vehicle):
    def __init__(self,
                max_speed=100, 
                starting_velocity=0, 
                starting_x=0, 
                starting_y=0, 
                starting_steering_angle=0, 
                starting_yaw=0, 
                wheel_base=2, 
                max_acceleration=14, 
                max_brake=35):
        state=np.array([starting_x,starting_y,starting_yaw,starting_velocity])
        super().__init__(state,max_speed,wheel_base,max_acceleration,max_brake)
        self.steering_angle = starting_steering_angle

    def derivative_function(self, state, control_inputs):
        yaw=state[2]
        v=state[3]
        steering_angle=control_inputs[0]
        acceleration=control_inputs[1]
        delta_x = v*np.cos(yaw)
        delta_y = v*np.sin(yaw)
        delta_v = acceleration
        yaw_rate=(v*np.tan(steering_angle))/self.wheel_base
        return np.array([delta_x, delta_y, yaw_rate, delta_v])

    def apply_state_limits(self,state):
            v=state[3]
            v=np.maximum(0,v)
            return np.array([state[0],state[1],state[2],v])


    def get_velocity(self):
        return self.state[3]



class dynamic_vehicle(vehicle):
    def __init__(self,
             max_speed=90, 
             starting_vx=0,              
             starting_vy=0,              
             starting_yaw_rate=0,        
             starting_x=0, 
             starting_y=0, 
             starting_yaw=0,  
             mass=800,
             COG_to_front_axis=2, 
             COG_to_rear_axis=1.5,       
             yaw_moment_of_inertia=1000,
             front_cornering_stiffness=180000,
             rear_cornering_stiffness=230000,  
             max_acceleration=11, 
             max_brake=40,
             shape_factor=1.3,
             curvature_factor=0,
             friction_coefficient_front=1.6,
             friction_coefficient_rear=1.6,
             downforce_area=4.5,
             drag_area=1.15,
             rolling_resistance_coefficient=0.015,
             aero_balance_front=0.45,
             brake_balance_front=0.55
             ):
        calculated_wheel_base=COG_to_front_axis+COG_to_rear_axis
        state=np.array([starting_x,starting_y,starting_yaw,starting_vx,starting_vy,starting_yaw_rate])
        super().__init__(state,max_speed,calculated_wheel_base, max_acceleration, max_brake)
        self.yaw_moment_of_inertia=yaw_moment_of_inertia
        self.front_cornering_stiffness=front_cornering_stiffness
        self.rear_cornering_stiffness=rear_cornering_stiffness
        self.COG_to_front_axis=COG_to_front_axis
        self.COG_to_rear_axis=COG_to_rear_axis
        self.mass=mass
        self.friction_coefficient_front=friction_coefficient_front
        self.friction_coefficient_rear=friction_coefficient_rear
        self.shape_factor=shape_factor
        self.curvature_factor=curvature_factor
        self.vertical_force_front=(self.mass*constants.GRAVITY*self.COG_to_rear_axis)/self.wheel_base
        self.vertical_force_rear=(self.mass*constants.GRAVITY*self.COG_to_front_axis)/self.wheel_base
        peak_factor_front=friction_coefficient_front*self.vertical_force_front
        peak_factor_rear=friction_coefficient_rear*self.vertical_force_rear
        self.stiffness_factor_front=self.front_cornering_stiffness/(self.shape_factor*peak_factor_front)
        self.stiffness_factor_rear=self.rear_cornering_stiffness/(self.shape_factor*peak_factor_rear)
        self.downforce_factor=0.5*constants.AIR_DENSITY*downforce_area
        self.drag_constant=0.5*constants.AIR_DENSITY*drag_area
        self.rolling_resistance_coefficient=rolling_resistance_coefficient
        self.aero_balance_front=aero_balance_front
        self.brake_balance_front=brake_balance_front
        
    def friction_circle(self, acceleration,lateral_force_front,lateral_force_rear, front_axle_load,rear_axle_load):
        longitudinal_force_required=self.mass*acceleration
        if longitudinal_force_required>0:
            longitudinal_tire_force_front=0
            longitudinal_tire_force_rear=longitudinal_force_required
        else:
            longitudinal_tire_force_front=self.brake_balance_front*longitudinal_force_required
            longitudinal_tire_force_rear=(1-self.brake_balance_front)*longitudinal_force_required
        budget_rear=self.friction_coefficient_rear*rear_axle_load
        budget_front=self.friction_coefficient_front*front_axle_load
        longitudinal_tire_force_rear=np.clip(longitudinal_tire_force_rear,-budget_rear,budget_rear)
        longitudinal_tire_force_front=np.clip(longitudinal_tire_force_front,-budget_front,budget_front)
        ratio_front=longitudinal_tire_force_front/budget_front
        ratio_rear=longitudinal_tire_force_rear/budget_rear
        factor_rear=np.sqrt(1-ratio_rear**2)
        factor_front=np.sqrt(1-ratio_front**2)
        lateral_force_front=lateral_force_front*factor_front
        lateral_force_rear=lateral_force_rear*factor_rear
        return longitudinal_tire_force_front,longitudinal_tire_force_rear, lateral_force_front, lateral_force_rear



    def derivative_function(self, state, control_inputs):
        yaw=state[2]
        vx=state[3]
        vy=state[4]
        yaw_rate=state[5]
        steering_angle=control_inputs[0]
        acceleration=control_inputs[1]
        down_force=self.downforce_factor*(vx**2)
        drag_force=self.drag_constant*vx*abs(vx)
        roll_force=self.rolling_resistance_coefficient*(self.mass*constants.GRAVITY+down_force)*np.sign(vx)
        front_axle_load=self.vertical_force_front+self.aero_balance_front*down_force
        rear_axle_load=self.vertical_force_rear+(1-self.aero_balance_front)*down_force
        peak_factor_front=self.friction_coefficient_front*front_axle_load
        peak_factor_rear=self.friction_coefficient_rear*rear_axle_load
        front_slip_angle=steering_angle-np.arctan((vy+self.COG_to_front_axis*yaw_rate)/np.maximum(0.5,vx))
        rear_slip_angle=-np.arctan((vy-(self.COG_to_rear_axis*yaw_rate))/np.maximum(0.5,vx))
        front_lateral_force=tire_function(front_slip_angle,self.stiffness_factor_front,self.shape_factor,peak_factor_front,self.curvature_factor)
        rear_lateral_force=tire_function(rear_slip_angle,self.stiffness_factor_rear,self.shape_factor,peak_factor_rear, self.curvature_factor)
        longitudinal_tire_force_front,longitudinal_tire_force_rear, front_lateral_force, rear_lateral_force=self.friction_circle(acceleration, front_lateral_force,rear_lateral_force,front_axle_load,rear_axle_load)
        v_dot_x=(longitudinal_tire_force_front*np.cos(steering_angle)-front_lateral_force*np.sin(steering_angle)+longitudinal_tire_force_rear-drag_force-roll_force)/self.mass+vy*yaw_rate
        v_dot_y=(longitudinal_tire_force_front*np.sin(steering_angle)+front_lateral_force*np.cos(steering_angle)+rear_lateral_force)/self.mass-vx*yaw_rate
        yaw_acceleration=(self.COG_to_front_axis*(longitudinal_tire_force_front*np.sin(steering_angle)+front_lateral_force*np.cos(steering_angle))-self.COG_to_rear_axis*rear_lateral_force)/self.yaw_moment_of_inertia
        
        
        x_dot=vx*np.cos(yaw)-vy*np.sin(yaw)
        y_dot=vx*np.sin(yaw)+vy*np.cos(yaw)
        return np.array([x_dot, y_dot, yaw_rate, v_dot_x, v_dot_y, yaw_acceleration])

    def get_velocity(self):
        return np.sqrt(self.state[3]**2+self.state[4]**2)

    def apply_state_limits(self,state):
        vx=state[3]
        vx=np.maximum(0,vx)
        return np.array([state[0],state[1],state[2],vx,state[4],state[5]])


