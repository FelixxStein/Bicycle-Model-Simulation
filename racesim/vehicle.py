import numpy as np
import racesim.constants as constants
def integrator(derivative_function ,state ,control_inputs, delta_t):
    change=derivative_function(state,control_inputs)
    state=state+change*delta_t
    return state
def tire_function(slip_angle,stiffness_factor,shape_factor,peak_factor,curvature_factor):
    return peak_factor*np.sin(shape_factor*np.arctan(stiffness_factor*slip_angle-curvature_factor*(stiffness_factor*slip_angle-np.arctan(stiffness_factor*slip_angle))))

class Vehicle():
    def __init__(self,
                state,
                max_speed, 
                wheel_base, 
                max_acceleration, 
                max_brake,
                max_steering_angle,
                mass,
                max_steering_rate,
                steering_time_constant,
                standstill_threshold=1):
        self.max_steering_rate=max_steering_rate
        self.steering_time_constant=steering_time_constant
        self.standstill_threshold=standstill_threshold
        self.state=state
        self.max_speed=max_speed
        self.wheel_base=wheel_base
        self.max_acceleration=max_acceleration
        self.max_brake=max_brake
        self.max_steering_angle=max_steering_angle
        self.mass=mass

    def get_max_acceleration(self):
        return self.max_acceleration

    def get_position(self):
        return self.state[...,0:2]

    def calculate_new_state(self,state,control_inputs,delta_t):
        copy_control_inputs=control_inputs.copy()
        copy_control_inputs[...,0]=self.update_steering_angle(copy_control_inputs[...,0])
        copy_control_inputs[...,1]=self.update_acceleration(copy_control_inputs[...,1])
        state_after=integrator(self.derivative_function,state,copy_control_inputs, delta_t)
        state_after[...,3]=np.where((copy_control_inputs[...,1]<0)&(np.sign(state[...,3])!=np.sign(state_after[...,3]))&(np.linalg.norm(state[...,3:5], axis=-1)<=self.standstill_threshold),0,state_after[...,3])
        return state_after


    def time_step(self,control_inputs, delta_t):
        state=self.calculate_new_state(self.state,control_inputs,delta_t)
        self.state=state

    def get_state(self):
        return self.state.copy()
    
    def get_yaw(self):
        return self.state[...,2]
    
    def get_wheelbase(self):
         return self.wheel_base
    
    def get_max_brake(self):
        return self.max_brake

    def get_max_speed(self):
        return self.max_speed

    def set_starting_angle(self, racetrack):
        vector=racetrack[1]-racetrack[0]
        self.state[2]=np.atan2(vector[1],vector[0])

    def set_starting_position(self,racetrack):
        self.state[0]=racetrack[0][0]
        self.state[1]=racetrack[0][1]

    def update_acceleration(self, acceleration):
        return np.clip(acceleration,-self.max_brake,self.max_acceleration)

    def update_steering_angle(self,steering_angle):
        return np.clip(steering_angle,-self.max_steering_angle, self.max_steering_angle)

    def get_max_steering_angle(self):
        return self.max_steering_angle

    def get_mass(self):
        return self.mass

class DynamicVehicle(Vehicle):
    def __init__(self,
             max_speed=90.0, 
             starting_vx=0.0,              
             starting_vy=0.0,              
             starting_yaw_rate=0.0,        
             starting_x=0.0, 
             starting_y=0.0, 
             starting_yaw=0.0,  
             mass=800,
             COG_to_front_axis=2, 
             COG_to_rear_axis=1.5,
             COG_height=0.3,   
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
             brake_balance_front=0.55,
             relaxation_time=0.05,
             blend_speed_low=3,
             blend_speed_high=8,
             max_steering_angle=0.35,
             starting_steering_angle=0.0,
             max_steering_rate=1.0,
             steering_time_constant=0.05
             ):
        calculated_wheel_base=COG_to_front_axis+COG_to_rear_axis
        state=np.array([starting_x,starting_y,starting_yaw,starting_vx,starting_vy,starting_yaw_rate,starting_steering_angle])
        super().__init__(state=state,max_speed=max_speed,wheel_base=calculated_wheel_base,max_acceleration=max_acceleration,max_brake=max_brake,max_steering_angle=max_steering_angle,mass=mass, max_steering_rate=max_steering_rate, steering_time_constant=steering_time_constant)
        self.yaw_moment_of_inertia=yaw_moment_of_inertia
        self.front_cornering_stiffness=front_cornering_stiffness
        self.rear_cornering_stiffness=rear_cornering_stiffness
        self.COG_to_front_axis=COG_to_front_axis
        self.COG_to_rear_axis=COG_to_rear_axis
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
        self.relaxation_time=relaxation_time
        self.blend_speed_high=blend_speed_high
        self.blend_speed_low=blend_speed_low
        self.COG_height=COG_height
        
    def friction_circle(self, acceleration,lateral_force_front,lateral_force_rear, front_axle_load,rear_axle_load, vx):
        longitudinal_force_required=self.mass*acceleration
        longitudinal_tire_force_front=np.where(longitudinal_force_required>0,0,np.sign(vx)*self.brake_balance_front*longitudinal_force_required)
        longitudinal_tire_force_rear=np.where(longitudinal_force_required>0,longitudinal_force_required,np.sign(vx)*(1-self.brake_balance_front)*longitudinal_force_required)
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

    def derivative_function_kinematic(self,vx,vy,yaw_rate,acceleration,steering_angle,drag_force, rolling_force,down_force):
        yaw_rate_target=vx*np.tan(steering_angle)/self.wheel_base
        vy_target=self.COG_to_rear_axis*yaw_rate_target
        vx_dot=np.where(acceleration<0,np.sign(vx)*acceleration-(drag_force+rolling_force)/self.mass,acceleration-(drag_force+rolling_force)/self.mass)
        vy_dot=(vy_target-vy)/self.relaxation_time
        a_max=(self.get_min_friction_coefficient()*(self.mass*constants.GRAVITY+down_force))/self.mass
        vy_dot=np.clip(vy_dot,-a_max,a_max)
        yaw_acceleration=(yaw_rate_target-yaw_rate)/self.relaxation_time
        return vx_dot, vy_dot, yaw_acceleration

    def calculate_axle_load(self,down_force,load_transfer):
        front_axle_load=np.maximum(self.vertical_force_front+self.aero_balance_front*down_force-load_transfer,0)
        rear_axle_load=np.maximum((self.vertical_force_rear+(1-self.aero_balance_front)*down_force+load_transfer),0)
        return front_axle_load, rear_axle_load


    def derivative_function_dynamic(self,vx,vy,yaw_rate,acceleration,steering_angle,down_force,drag_force,rolling_force): 
        load_transfer=(self.mass*acceleration*self.COG_height)/self.wheel_base
        front_axle_load, rear_axle_load=self.calculate_axle_load(down_force,load_transfer)
        peak_factor_front=self.friction_coefficient_front*front_axle_load
        peak_factor_rear=self.friction_coefficient_rear*rear_axle_load
        front_slip_angle=steering_angle-np.arctan((vy+self.COG_to_front_axis*yaw_rate)/np.maximum(0.5,np.abs(vx)))
        rear_slip_angle=-np.arctan((vy-(self.COG_to_rear_axis*yaw_rate))/np.maximum(0.5,np.abs(vx)))
        front_lateral_force=tire_function(front_slip_angle,self.stiffness_factor_front,self.shape_factor,peak_factor_front,self.curvature_factor)
        rear_lateral_force=tire_function(rear_slip_angle,self.stiffness_factor_rear,self.shape_factor,peak_factor_rear, self.curvature_factor)
        longitudinal_tire_force_front,longitudinal_tire_force_rear, front_lateral_force, rear_lateral_force=self.friction_circle(acceleration, front_lateral_force,rear_lateral_force,front_axle_load,rear_axle_load, vx)
        yaw_acceleration=(self.COG_to_front_axis*(longitudinal_tire_force_front*np.sin(steering_angle)+front_lateral_force*np.cos(steering_angle))-self.COG_to_rear_axis*rear_lateral_force)/self.yaw_moment_of_inertia
        vx_dot=(longitudinal_tire_force_front*np.cos(steering_angle)-front_lateral_force*np.sin(steering_angle)+longitudinal_tire_force_rear-drag_force-rolling_force)/self.mass+vy*yaw_rate
        vy_dot=(longitudinal_tire_force_front*np.sin(steering_angle)+front_lateral_force*np.cos(steering_angle)+rear_lateral_force)/self.mass-vx*yaw_rate
        return vx_dot, vy_dot, yaw_acceleration

    def calculate_steering_rate(self,wanted_steering_angle, current_steering_angle):
        steering_rate=(wanted_steering_angle-current_steering_angle)/self.steering_time_constant
        steering_rate=np.clip(steering_rate, -self.max_steering_rate,self.max_steering_rate)
        return steering_rate


    def derivative_function(self,state, control_inputs):
            yaw=state[...,2]
            vx=state[...,3]
            vy=state[...,4]
            yaw_rate=state[...,5]
            current_steering_angle=state[...,6]
            wanted_steering_angle=control_inputs[...,0]
            acceleration=control_inputs[...,1]
            down_force=self.downforce_factor*(vx**2)
            drag_force=self.drag_constant*vx*abs(vx)
            rolling_force=self.rolling_resistance_coefficient*(self.mass*constants.GRAVITY+down_force)*np.sign(vx)
            x_dot=vx*np.cos(yaw)-vy*np.sin(yaw)
            y_dot=vx*np.sin(yaw)+vy*np.cos(yaw)
            vx_dot_kinematic, vy_dot_kinematic, yaw_acceleration_kinematic=self.derivative_function_kinematic(vx,vy,yaw_rate, acceleration,current_steering_angle,drag_force, rolling_force,down_force)
            vx_dot_dynamic, vy_dot_dynamic, yaw_acceleration_dynamic=self.derivative_function_dynamic(vx,vy,yaw_rate,acceleration,current_steering_angle,down_force,drag_force,rolling_force)
            blend_weight=np.clip((np.linalg.norm(state[...,3:5], axis=-1)-self.blend_speed_low)/(self.blend_speed_high-self.blend_speed_low),0,1)
            vx_dot=blend_weight*vx_dot_dynamic+(1-blend_weight)*vx_dot_kinematic
            vy_dot=blend_weight*vy_dot_dynamic+(1-blend_weight)*vy_dot_kinematic
            yaw_acceleration=blend_weight*yaw_acceleration_dynamic+(1-blend_weight)*yaw_acceleration_kinematic
            steering_rate=self.calculate_steering_rate(wanted_steering_angle=wanted_steering_angle,current_steering_angle=current_steering_angle)
            return np.stack([x_dot,y_dot,yaw_rate,vx_dot,vy_dot,yaw_acceleration, steering_rate],axis=-1,dtype=float)
            

    def get_velocity(self):
        return np.sqrt(self.state[...,3]**2+self.state[...,4]**2)

    def get_vx(self):
        return self.state[...,3]

    def get_friction_coefficient_front(self):
        return self.friction_coefficient_front

    def get_friction_coefficient_rear(self):
        return self.friction_coefficient_rear

    def get_downforce_factor(self):
        return self.downforce_factor

    def get_yaw_rate(self):
        return self.state[...,5]

    def get_min_friction_coefficient(self):
        return min(self.friction_coefficient_front, self.friction_coefficient_rear)

    def get_current_steering_rate(self):
        return self.state[...,6]
