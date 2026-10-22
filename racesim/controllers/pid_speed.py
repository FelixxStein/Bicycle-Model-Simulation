import numpy as np
import racesim.constants as constants
class PIDSpeedController:
    def __init__(self,track,max_break,max_acceleration, max_speed,lookahead_distance, max_lateral_acceleration, Kp,Ki,Kd, grip_usage, friction_coefficient, downforce_factor, car_mass):
        self.max_break=max_break
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.max_speed=max_speed
        self.max_lateral_acceleration=max_lateral_acceleration
        self.ek_prev=0
        self.Ik=0
        self.track=track
        self.path=track.get_midline()
        self.lookahead_distance=lookahead_distance
        self.cumulative_point_distance=track.get_cumulative_point_distance()
        self.curvature_per_pathpoint=track.get_curvature()
        self.grip_usage=grip_usage
        self.friction_coefficient=friction_coefficient
        self.downforce_factor=downforce_factor
        self.max_acceleration=max_acceleration
        self.car_mass=car_mass
        self.planner_break=0.6*grip_usage*friction_coefficient*constants.GRAVITY


        self.corner_speed_per_point=np.where(abs(self.curvature_per_pathpoint)>1e-5,np.minimum(np.sqrt(self.max_lateral_acceleration/abs(self.curvature_per_pathpoint)),max_speed), max_speed)
        

       

    def calculate_distance(self,vek2, vek1):
        return np.sqrt(np.sum((vek2 - vek1)**2))


    def calculate_min_v_allowed(self,index):
        v_allowed_min=self.max_speed
        for i in range(index, index+self.lookahead_distance):
            v_allowed=np.sqrt(self.corner_speed_per_point[i%len(self.path)]**2+2*self.planner_break*(self.track.forward_distance(index,i%len(self.path))))
            if v_allowed<v_allowed_min:
                v_allowed_min=v_allowed
        return v_allowed_min
    def calculate_driver_limits(self,index, vx, yaw_rate):
        available = self.grip_usage*self.friction_coefficient*(constants.GRAVITY+self.downforce_factor*(vx**2)/self.car_mass)
        lateral_needed_pathbased= (vx**2)*abs(self.curvature_per_pathpoint[index])
        lateral_needed_measured=vx*abs(yaw_rate)
        lateral_needed=np.maximum(lateral_needed_pathbased, lateral_needed_measured)
        allowed_longitudinal =np.sqrt(max(0, (available**2)-(lateral_needed**2)))
        
        return allowed_longitudinal

        
    def compute_control(self, index, velocity, vx, yaw_rate, delta_t):
        v_allowed_min=self.calculate_min_v_allowed(index)
        v_target=np.minimum(self.max_speed, v_allowed_min)
        ek=v_target-velocity
        self.Ik = self.Ik + ek * delta_t
        self.Ik=np.clip(self.Ik,-5.0,+5.0)
    
        Dk=(ek-self.ek_prev)/delta_t
        acceleration=self.Kp*ek+self.Ki*self.Ik+self.Kd*Dk
        self.ek_prev=ek
        allowed_longitudinal=self.calculate_driver_limits(index, vx, yaw_rate)
        acceleration=np.clip(acceleration,-allowed_longitudinal, min(allowed_longitudinal, self.max_acceleration))
        return acceleration
    

    
